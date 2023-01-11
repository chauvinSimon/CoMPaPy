"""
socket client for the panda robot-arm
"""
import argparse
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
import socket
import time

from geometry_msgs.msg import Pose

from compapy.scripts.MyCoMPaPy import MyCoMPaPy
from compapy.scripts.utils import setup_logger
from compapy.scripts.socket_interface.frame_conversion import link8_in_base, ee_in_base


def state_to_str(
        gripper_gap_mm,
        joints_rad,
        tcp_pose,
) -> str:
    state_gripper = f'[gripper_gap_mm={gripper_gap_mm}]'
    state_joints = ''.join([f'[j_{i}={joints_rad[i]}]' for i in range(6)])
    state_pose = ''.join([f'[p_{i}={tcp_pose[i]}]' for i in range(6)])
    return state_gripper + state_joints + state_pose


def main(
        teleport: bool = False,
        ignore_gripper: bool = False,
        process_target: bool = True,
):
    saving_dir = Path('logs') / str(time.strftime("%Y%m%d_%H%M%S"))
    saving_dir.mkdir(exist_ok=True, parents=True)
    compapy = MyCoMPaPy(
        log_file=saving_dir / 'compapy.log',
        save_planning_res=True
    )

    logger = setup_logger(name=Path(__file__).name, log_file=saving_dir / 'socket.log')

    host = "127.0.0.1"
    port = 65432

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))

        while True:
            data = s.recv(1024)
            data = data.decode('utf-8')
            logger.info(f'received {data!r}')
            success = False
            error_msg = ''
            if data == '>exit<':
                logger.info('closing client socket - bye')
                break

            elif '>move<' in data:
                try:
                    target_pose_str = data[len('>move<>'):-len('<')]
                    target_pose_ee_in_base = [float(x) for x in target_pose_str.split('<>')]
                    logger.debug(f'target_pose_ee_in_base: {[round(e, 3) for e in target_pose_ee_in_base]}')
                    # example:
                    # data='>move<>0.561<>-0.487<>0.348<>2.786<>-0.586<>0.509<'
                    # target_pose_ee_in_base=[0.561, -0.487, 0.348, 2.786, -0.586, 0.509]
                    #   IMPORTANT: pose of the tip of the end-effector (not link8!) relative to the robot base
                    #   IMPORTANT: Rot-Vec, not RPY/Euler!

                    target_pose_l8_in_base = link8_in_base(ee_in_base=target_pose_ee_in_base)

                    target_pose = Pose()
                    target_pose.position.x = target_pose_l8_in_base[0]
                    target_pose.position.y = target_pose_l8_in_base[1]
                    target_pose.position.z = target_pose_l8_in_base[2]

                    rot_vec = target_pose_l8_in_base[3:6]
                    logger.debug(f'l8: rot_vec = {[round(e, 3) for e in rot_vec]}')
                    euler_rad = Rotation.from_rotvec(rot_vec).as_euler('xyz')
                    logger.debug(f'l8: euler = {[round(np.rad2deg(e), 1) for e in euler_rad]} (ext deg)')

                    q = Rotation.from_rotvec(rot_vec).as_quat()

                    target_pose.orientation.x = q[0]
                    target_pose.orientation.y = q[1]
                    target_pose.orientation.z = q[2]
                    target_pose.orientation.w = q[3]

                    if teleport:
                        # todo success = compapy.teleport(target_pose)
                        raise NotImplementedError
                    else:
                        success, move_err_msg = compapy.my_move(
                            target_pose=target_pose,
                            process_target=process_target
                        )
                        if move_err_msg:
                            error_msg += f' move_err_msg=[{move_err_msg}] '

                except Exception as e:
                    msg = f'[move] exception=[{e}] data=[{data}]'
                    logger.error(msg)
                    error_msg += f' {msg} '

            elif '>gripper<' in data:
                if data == '>gripper<>1<':
                    logger.info('trying to open')
                    try:
                        if ignore_gripper:
                            logger.info('skip open_gripper()')
                            success = True
                        else:
                            success = compapy.open_gripper()
                    except Exception as e:
                        msg = f'[open-gripper] exception=[{e}]'
                        logger.error(msg)
                        error_msg += f' {msg} '
                elif data == '>gripper<>0<':
                    logger.info('trying to close')
                    try:
                        if ignore_gripper:
                            logger.info('skip close_gripper()')
                            success = True
                        else:
                            success = compapy.close_gripper()
                    except Exception as e:
                        msg = f'[close-gripper] exception=[{e}]'
                        logger.error(msg)
                        error_msg += f' {msg} '
                else:
                    msg = f'[gripper]: data=[{data}] not supported'
                    logger.error(msg)
                    error_msg += f' {msg} '

            else:
                msg = f'data=[{data}] not supported'
                logger.error(msg)
                error_msg += f' {msg} '

            l8_pose = compapy.get_pose()

            # prepare conversion from L8 to EE
            l8_in_base_out = [0.0] * 6

            l8_in_base_out[0] = l8_pose.position.x
            l8_in_base_out[1] = l8_pose.position.y
            l8_in_base_out[2] = l8_pose.position.z

            logger.debug('reading out values')
            logger.debug(f'l8_position_in_base_out = {[round(e, 2) for e in l8_in_base_out[:3]]}')

            r_out = Rotation.from_quat([
                l8_pose.orientation.x,
                l8_pose.orientation.y,
                l8_pose.orientation.z,
                l8_pose.orientation.w
            ])
            logger.debug(f'r_base_to_l8 = {[round(e, 2) for e in r_out.as_euler("xyz", degrees=True)]} (euler-deg)')
            l8_in_base_out[3:6] = r_out.as_rotvec()

            ee_in_base_out = ee_in_base(l8_in_base_out)

            logger.debug(f'ee_in_base_out: {[round(e, 2) for e in ee_in_base_out]}')
            logger.debug(
                f'r_base_to_ee = '
                f'{[round(e, 2) for e in Rotation.from_rotvec(ee_in_base_out[3:]).as_euler("xyz", degrees=True)]}'
                f'(euler-deg)'
            )

            gripper_gap_mm, gripper_err_msg = compapy.get_gripper_width_mm()
            if gripper_gap_mm is None:
                success = False
                msg = f'could not read gripper_gap: [{gripper_err_msg}]'
                logger.error(msg)
                error_msg += f' {msg} '

            state_str = state_to_str(
                gripper_gap_mm=gripper_gap_mm,
                joints_rad=compapy.get_joints(),
                tcp_pose=ee_in_base_out,
            )

            if error_msg:
                state_str = f'msg=[{error_msg}] || state_str={state_str}'
            if not success:
                state_str = f'error: {state_str}'

            logger.debug(f'state_str = {state_str}')
            state_bytes = bytes(state_str, 'utf-8')

            s.sendall(state_bytes)
            time.sleep(0.1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--teleport', action='store_true',
                        help='speed up the simulation by [plan+teleport] instead of [plan+execute]')
    parser.add_argument('--ignore_gripper', action='store_true',
                        help='speed up the simulation by not opening/closing the gripper')
    parser.add_argument('--ignore_target_processing', action='store_true',
                        help='disable the specific processing '
                             'e.g. make sure the gripper stays vertical and wrap its yaw angle to a particular range')
    args = parser.parse_args()

    main(
        teleport=args.teleport,
        ignore_gripper=args.ignore_gripper,
        process_target=not args.ignore_target_processing,
    )
