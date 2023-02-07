from scipy.spatial.transform import Rotation
from typing import List

from geometry_msgs.msg import Pose


def pose_from_xyz_and_rotvec(
        xyz_and_rotvec: List
) -> Pose:
    pose = Pose()

    pose.position.x = xyz_and_rotvec[0]
    pose.position.y = xyz_and_rotvec[1]
    pose.position.z = xyz_and_rotvec[2]

    q = Rotation.from_rotvec(xyz_and_rotvec[3:6]).as_quat()
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    return pose


def xyz_and_rotvec_from_pose(
        pose: Pose
) -> List:
    xyz_and_rotvec = [0.0] * 6

    xyz_and_rotvec[0] = pose.position.x
    xyz_and_rotvec[1] = pose.position.y
    xyz_and_rotvec[2] = pose.position.z

    rot = Rotation.from_quat([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ])
    xyz_and_rotvec[3:6] = rot.as_rotvec()
    return xyz_and_rotvec
