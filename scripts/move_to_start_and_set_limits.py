"""
script to
- overwrite the joint_limits.yaml used by moveit
- move the arm to its start_pose

if the script terminates before stopping ROS
  killall -9 rosmaster
"""
import copy
from pathlib import Path
from typing import List, Tuple, Dict

import roslaunch
import rospy

from compapy.scripts.CoMPaPy import CoMPaPy
from compapy.scripts.utils import setup_logger, read_yaml


def copy_a_to_b(a: Path, b: Path) -> None:
    with a.open('r') as f_a:
        data = f_a.read()

    with b.open('w') as f_b:
        f_b.write(data)


def move_to(
        target_joint_values: List,
        use_sim: bool,
        use_move_j: bool = True
) -> Tuple[bool, str]:
    compapy = CoMPaPy()

    if not use_sim:
        compapy.open_gripper()
        compapy.close_gripper()

    if use_sim:
        # with rviz, the initial pose is already the target_pose
        # therefore, move a bit the arm to see the impact of the next "move to start_pose"
        init_pose = compapy.move_group.get_current_pose().pose
        init_pose.position.z += 0.1
        success, error_message = compapy.move_j(init_pose)
        if not success:
            return success, error_message

    # either use `move_j()` (expects a `JointState`) ...
    if use_move_j:
        current_state = compapy.move_group.get_current_state()
        current_joint_state = current_state.joint_state
        target_joint_state = copy.deepcopy(current_joint_state)
        target_joint_state.position = tuple(target_joint_values)

        print(f'[{len(current_joint_state.position)}] current: {[round(e, 5) for e in current_joint_state.position]}')
        print(f'[{len(target_joint_state.position)}]  target: {[round(e, 5) for e in target_joint_state.position]}')

        target_joint_state.name = target_joint_state.name[:-2]  # ignore the two gripper joints
        success, error_message = compapy.move_j(target_joint_state)

    # or use `go()` (passing a list of joint values) ...
    else:
        current_joint_values = compapy.move_group.get_current_joint_values()
        print(f'[{len(current_joint_values)}] current: {[round(e, 5) for e in current_joint_values]}')
        print(f'[{len(target_joint_values)}]  target: {[round(e, 5) for e in target_joint_values]}')

        success = compapy.move_group.go(target_joint_values, wait=True)
        error_message = ''
        compapy.move_group.stop()  # ensures that there is no residual movement

    rospy.sleep(1)  # todo: needed?
    del compapy  # todo: needed?
    return success, error_message


def get_data(use_sim: bool) -> Dict:
    start_pose_path = Path('../franka_ros/franka_control/config/start_pose.yaml')

    joint_limits_path = Path('../franka_ros/franka_description/robots/panda/joint_limits.yaml')
    joint_limits_7_dof_fr3_path = Path('../franka_ros/franka_description/robots/fr3/joint_limits.yaml')
    joint_limits_5_dof_fr3_path = Path('config/joint_limits_fr3_noj3_noj5.yaml')

    launch_path = Path(f"launch/{'sim' if use_sim else 'real'}.launch")
    launch_path_abs_str = str(launch_path.resolve().absolute())

    for p in [
        start_pose_path,
        joint_limits_path,
        joint_limits_7_dof_fr3_path,
        joint_limits_5_dof_fr3_path,
        launch_path,
    ]:
        assert p.exists(), f'file does not exist: {p}\n' \
                           f'make sure that:\n\t' \
                           f'- this script is run from `compapy/`\n\t' \
                           f'- `franka_ros` is located in the same directory as `compapy`'

    start_pose = read_yaml(start_pose_path)
    joint_pose = {k.replace('$(arg arm_id)_joint', 'j'): v for k, v in start_pose['joint_pose'].items()}
    target_joint_values = [joint_pose[f'j{i + 1}'] for i in range(len(joint_pose))]
    expected_len = 7
    assert len(target_joint_values) == expected_len, \
        f'target_joint_values is not of len {expected_len}: {target_joint_values}'

    return {
        'joint_limits_path': joint_limits_path,
        'joint_limits_5_dof_fr3_path': joint_limits_5_dof_fr3_path,
        'joint_limits_7_dof_fr3_path': joint_limits_7_dof_fr3_path,
        'launch_path_abs_str': launch_path_abs_str,
        'target_joint_values': target_joint_values,
    }


def main(
        dof: int,
        use_sim: bool = False
):
    data = get_data(use_sim=use_sim)
    joint_limits_path = data['joint_limits_path']
    joint_limits_5_dof_fr3_path = data['joint_limits_5_dof_fr3_path']
    joint_limits_7_dof_fr3_path = data['joint_limits_7_dof_fr3_path']
    launch_path_abs_str = data['launch_path_abs_str']
    target_joint_values = data['target_joint_values']

    logger = setup_logger(name=Path(__file__).name, log_file=Path(f'{Path(__file__).stem}.log'))

    logger.info('setting 7 dof ...')
    copy_a_to_b(
        a=joint_limits_7_dof_fr3_path,
        b=joint_limits_path,
    )

    logger.info('starting ROS ...')
    # https://wiki.ros.org/roslaunch/API%20Usage
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    cli_args = [launch_path_abs_str]
    if not use_sim:
        cli_args.append('robot_ip:=172.16.0.2')
    cli_params = cli_args[1:] if len(cli_args) > 1 else []
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_params)]
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    parent.start()
    rospy.loginfo("started")

    logger.info('waiting for rviz to be ready ...')
    rospy.sleep(8)

    logger.info('moving to start pose ...')
    try:
        success, error_message = move_to(target_joint_values=target_joint_values, use_sim=use_sim)
        if not success:
            logger.error(f'move failed: {error_message}')
    except Exception as e:
        logger.error(f'could not move to start pose ({target_joint_values}): {e}')

    logger.info('stopping ROS ...')
    parent.shutdown()

    if dof == 5:
        logger.info('setting 5 dof ...')
        copy_a_to_b(
            a=joint_limits_5_dof_fr3_path,
            b=joint_limits_path,
        )


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--dof', default=7, type=int, choices=[5, 7])
    parser.add_argument('--use_sim', action='store_true')
    args = parser.parse_args()

    main(dof=args.dof, use_sim=args.use_sim)
