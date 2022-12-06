"""
socket client for the panda robot-arm
"""
import logging
import numpy as np
from scipy.spatial.transform import Rotation
import socket
import time

from compapy.scripts.CoMPaPy import CoMPaPy
from geometry_msgs.msg import Pose

from frame_conversion import link8_in_base, ee_in_base


def main():
    compapy = CoMPaPy()

    host = "192.168.125.5"
    #  host = "127.0.0.1"
    port = 65432

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))

        while True:
            data = s.recv(1024)
            data = data.decode('utf-8')
            print(f'\nreceived {data!r}')
            if data == '>exit<':
                print('closing client socket - bye')
                break

            elif '>move<' in data:
                try:
                    target_pose_str = data[len('>move<>'):-len('<')]
                    target_pose_ee_in_base = [float(x) for x in target_pose_str.split('<>')]
                    print(f'target_pose_ee_in_base: {[round(e, 3) for e in target_pose_ee_in_base]}')
                    # example:
                    # data='>move<>0.561<>-0.487<>0.348<>2.786<>-0.586<>0.509<'
                    # target_pose_ee_in_base=[0.561, -0.487, 0.348, 2.786, -0.586, 0.509]
                    #   IMPORTANT: pose of the tip of the end-effector relative to the robot base
                    #   IMPORTANT: Rot-Vec, not RPY/Euler!

                    target_pose_l8_in_base = link8_in_base(ee_in_base=target_pose_ee_in_base)

                    target_pose = Pose()
                    target_pose.position.x = target_pose_l8_in_base[0]
                    target_pose.position.y = target_pose_l8_in_base[1]
                    target_pose.position.z = target_pose_l8_in_base[2]

                    # todo: directly from rot_vec to quaternion
                    rot_vec = target_pose_l8_in_base[3:6]
                    print(f'l8: rot_vec = {rot_vec}')
                    euler = Rotation.from_rotvec(rot_vec).as_euler('xyz')
                    print(f'l8: euler (external rad) = {euler}')
                    euler_deg = [np.rad2deg(e) for e in euler]
                    print(f'l8: euler (external deg) = {euler_deg}')
                    q = Rotation.from_rotvec(rot_vec).as_quat()

                    target_pose.orientation.x = q[0]
                    target_pose.orientation.y = q[1]
                    target_pose.orientation.z = q[2]
                    target_pose.orientation.w = q[3]

                    compapy.move_l(target_pose)
                    # todo: process response

                except Exception as e:
                    logging.error(f'[move] data=[{data}] exception={e}')
                    # todo: write failure to PC socket

            elif '>gripper<' in data:
                if data == '>gripper<>1<':
                    print('trying to open')
                    try:
                        compapy.open_gripper()
                    except Exception as e:
                        logging.error(f'[open-gripper] exception={e}')
                        # todo: write failure to PC socket
                elif data == '>gripper<>0<':
                    print('trying to close')
                    try:
                        compapy.close_gripper()
                    except Exception as e:
                        logging.error(f'[close-gripper] exception={e}')
                        # todo: write failure to PC socket
                else:
                    raise NotImplementedError(f'about gripper: data={data}')
                    # todo: write failure to PC socket
                print(f'width_mm = {compapy.get_gripper_width_mm():.1f}')

            else:
                raise NotImplementedError(f'data={data}')
                # todo: write failure to PC socket

            l8_pose = compapy.get_pose()

            # prepare conversion from L8 to EE
            l8_in_base_out = [0.0] * 6

            l8_in_base_out[0] = l8_pose.position.x
            l8_in_base_out[1] = l8_pose.position.y
            l8_in_base_out[2] = l8_pose.position.z

            print('\nreading out values')
            print(f'l8_in_base_out = {[round(e, 2) for e in l8_in_base_out[:3]]}')

            # todo: directly from quat to rotvec
            roll, pitch, yaw = Rotation.from_quat([
                l8_pose.orientation.x,
                l8_pose.orientation.y,
                l8_pose.orientation.z,
                l8_pose.orientation.w
            ]).as_euler('xyz')

            print(f'r_base_to_l8 [euler-deg] = {[round(np.rad2deg(e), 2) for e in [roll, pitch, yaw]]}')
            rot_vec = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_rotvec()
            l8_in_base_out[3:6] = rot_vec

            ee_in_base_out = ee_in_base(l8_in_base_out)

            print(f'ee_in_base_out: {[round(e, 2) for e in ee_in_base_out]}')
            print(f'euler [ee out ext deg]: '
                  f'{[round(e, 2) for e in Rotation.from_rotvec(ee_in_base_out[3:]).as_euler("xyz", degrees=True)]}')

            def to_str(
                    gripper_gap_mm,
                    joints_rad,
                    tcp_pose,
            ) -> str:
                state_gripper = f'[gripper_gap_mm={gripper_gap_mm}]'
                state_joints = ''.join([f'[j_{i}={joints_rad[i]}]' for i in range(6)])
                state_pose = ''.join([f'[p_{i}={tcp_pose[i]}]' for i in range(6)])
                return state_gripper + state_joints + state_pose

            state_str = to_str(
                gripper_gap_mm=compapy.get_gripper_width_mm(),
                joints_rad=compapy.get_joints(),
                tcp_pose=ee_in_base_out,
            )
            state_bytes = bytes(state_str, 'utf-8')
            s.sendall(state_bytes)
            time.sleep(0.1)


if __name__ == '__main__':
    main()
