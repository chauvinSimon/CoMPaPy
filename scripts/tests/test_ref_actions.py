import copy
import numpy as np
from scipy.spatial.transform import Rotation
import unittest

from geometry_msgs.msg import Pose

from compapy.scripts.CoMPaPy import CoMPaPy


class TestRefActions(unittest.TestCase):

    def setUp(self) -> None:
        self.compapy = CoMPaPy()
        # todo: move to init pose

    def test_open_close(self, n_repetition: int = 2, max_error_mm: float = 1.0):
        self.compapy.config['close_gripper']['width'] = 0.0
        self.compapy.config['close_gripper']['epsilon_inner'] = 0.001
        self.compapy.config['close_gripper']['epsilon_outer'] = 0.001
        for _ in range(n_repetition):
            for name in ['open_gripper', 'close_gripper']:
                success = self.compapy.close_gripper() if name == 'close_gripper' else self.compapy.open_gripper()
                self.assertTrue(success, msg=f'action [{name}] failed')

                target_width_mm = 1000 * self.compapy.config[name]['width']
                actual_width_mm, error_msg = self.compapy.get_gripper_width_mm()
                print(f'target = [{target_width_mm:.1f}]  actual = [{actual_width_mm:.1f}] (mm)')
                error_msg = f'[{target_width_mm:.1f}] not reached: {actual_width_mm:.1f}'
                self.assertAlmostEqual(target_width_mm, actual_width_mm, delta=max_error_mm, msg=error_msg)

    def test_go_to_joint_state(self, max_error_deg: float = 0.1):
        """
        reach a ref position, stretching up as a giraffe
        """
        self.compapy.go_to_joint_state()

        tau = 2.0 * np.pi
        target_joints = [0, -tau / 8, 0, -tau / 4, 0, tau / 6, 0]
        actual_joints = self.compapy.get_joints()
        for i, (target_joint, actual_joint) in enumerate(zip(target_joints, actual_joints)):
            self.assertAlmostEqual(
                target_joint,
                actual_joint,
                delta=np.deg2rad(max_error_deg),
                msg=f'joint [{i}]: target={np.rad2deg(target_joint):.1f} actual={np.rad2deg(actual_joint):.1f} (deg)'
            )

    def test_move_l_in_cube(self, cube_size: float = 0.05):
        """
        move inside a virtual cube
        """
        init_pose = self.compapy.get_pose()
        targets = []

        # 1- along the internal "long" diagonal
        target_1 = copy.deepcopy(init_pose)
        target_1.position.x += cube_size
        target_1.position.y += cube_size
        target_1.position.z += cube_size
        targets.append(target_1)

        # 2- along the external diagonal of one face
        target_2 = copy.deepcopy(init_pose)
        target_2.position.y += cube_size
        targets.append(target_2)

        # 3- along the edge of one face, vertically
        target_3 = copy.deepcopy(init_pose)
        target_3.position.y += cube_size
        target_3.position.z += cube_size
        targets.append(target_3)

        # 4- along the external diagonal of one face, back to the init
        targets.append(init_pose)

        for target_pose in targets:
            self.compapy.move_l(target_pose)
            self.assert_pose(target_pose=target_pose, actual_pose=self.compapy.get_pose())

    def test_up_and_down(self):
        init_pose = self.compapy.get_pose()

        target_pose = copy.deepcopy(init_pose)
        target_pose.position.x += 0.3
        target_pose.position.z -= 0.3

        self.compapy.move_l(target_pose)
        self.assert_pose(target_pose=target_pose, actual_pose=self.compapy.get_pose())

        self.compapy.move_l(init_pose)
        self.assert_pose(target_pose=init_pose, actual_pose=self.compapy.get_pose())

    def assert_pose(
            self,
            target_pose: Pose,
            actual_pose: Pose,
            delta_mm: float = 4.0,
            delta_deg: float = 1.0
    ):
        for i, (target_p, actual_p) in enumerate(zip(
                [target_pose.position.x, target_pose.position.y, target_pose.position.z],
                [actual_pose.position.x, actual_pose.position.y, actual_pose.position.z]
        )):
            target_p_mm = target_p * 1000
            actual_p_mm = actual_p * 1000
            self.assertAlmostEqual(
                target_p_mm, actual_p_mm,
                delta=delta_mm,
                msg=f'pose [{i}]: target_mm={target_p_mm:.1f} actual_mm={actual_p_mm:.1f} (mm)'
            )

        t_o = target_pose.orientation
        a_o = actual_pose.orientation
        target_euler = Rotation.from_quat([t_o.x, t_o.y, t_o.z, t_o.w]).as_euler('xyz', degrees=True)
        actual_euler = Rotation.from_quat([a_o.x, a_o.y, a_o.z, a_o.w]).as_euler('xyz', degrees=True)

        wrap_180 = lambda x: (x + 180) % (2 * 180) - 180
        for i, (target_r, actual_r) in enumerate(zip(target_euler, actual_euler)):
            self.assertAlmostEqual(
                wrap_180(target_r), wrap_180(actual_r),
                delta=delta_deg,
                msg=f'euler [{i}]: target_r={target_r:.1f} actual_r={actual_r:.1f} (deg)'
            )


if __name__ == '__main__':
    unittest.main()
