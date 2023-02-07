"""
socket client for the panda robot-arm
"""
import argparse
from pathlib import Path
from scipy.spatial.transform import Rotation
import socket
import time

from compapy.scripts.MyCoMPaPy import MyCoMPaPy
from compapy.scripts.utils import setup_logger
from compapy.scripts.socket_interface.frame_conversion import link8_in_base, ee_in_base
from compapy.scripts.socket_interface.pose_conversion import pose_from_xyz_and_rotvec, xyz_and_rotvec_from_pose


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
        s.setblocking(True)  # equivalent to sock.settimeout(None), otherwise "socket.timeout: timed out"
        s.connect((host, port))
        compapy.logger.info(f'socket timeout = {s.gettimeout()}')

        gripper_gap_mm_overwritten = 1000 * compapy.config['open_gripper']['width']

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

                    target_l8_pose = pose_from_xyz_and_rotvec(
                        xyz_and_rotvec=link8_in_base(ee_in_base=target_pose_ee_in_base)
                    )

                    if teleport:
                        # todo success = compapy.teleport(target_pose)
                        raise NotImplementedError
                    else:
                        success, move_err_msg = compapy.my_move(
                            target_pose=target_l8_pose,
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

            l8_xyz_and_rotvec = xyz_and_rotvec_from_pose(
                pose=l8_pose
            )
            ee_in_base_out = ee_in_base(l8_xyz_and_rotvec)

            logger.debug(f'ee_in_base_out: {[round(e, 2) for e in ee_in_base_out]}')
            logger.debug(
                f'r_base_to_ee = '
                f'{[round(e, 2) for e in Rotation.from_rotvec(ee_in_base_out[3:]).as_euler("xyz", degrees=True)]}'
                f'(euler-deg)'
            )

            if ignore_gripper:
                if data == '>gripper<>0<':
                    gripper_gap_mm_overwritten = 57.4  # todo: read from config
                elif data == '>gripper<>1<':
                    gripper_gap_mm_overwritten = 1000 * compapy.config['open_gripper']['width']
                gripper_gap_mm = gripper_gap_mm_overwritten

            else:
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
    parser.add_argument('--use_target_processing', action='store_true',
                        help='enable the specific processing '
                             'e.g. make sure the gripper stays vertical and wrap its yaw angle to a particular range')
    args = parser.parse_args()

    main(
        teleport=args.teleport,
        ignore_gripper=args.ignore_gripper,
        process_target=args.use_target_processing,
    )
