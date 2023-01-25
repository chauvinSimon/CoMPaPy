"""
before testing: start robot (either real or gazebo simulation)
"""

import numpy as np
import unittest

from compapy.scripts.CoMPaPy import CoMPaPy


class TestRotateEE(unittest.TestCase):

    def setUp(self) -> None:
        self.compapy = CoMPaPy()

    def test_rotate_eef(
            self,
            a_deg_max: int = 10,
            n_positive_angles: int = 5,
            max_error_angle_deg: float = 1.0
    ):

        a_degs = list(np.linspace(start=0, stop=a_deg_max, num=n_positive_angles)) + [0.]  # back to zero at the end
        for a_deg in a_degs:
            for sign in [1, -1]:
                a_deg *= sign
                success = self.compapy.rotate_joint('panda_joint7', np.deg2rad(a_deg))
                if not success:
                    raise RuntimeError(f'failed to move eef to [{a_deg}] deg')
                res_deg = np.rad2deg(self.compapy.get_joints()[6])
                print(f'target = [{a_deg:.1f}]  actual = [{res_deg:.1f}] (deg)')
                self.assertAlmostEqual(
                    res_deg,
                    a_deg,
                    delta=max_error_angle_deg,
                    msg=f'{a_deg:.1f} not reached: {res_deg:.1f}'
                ),


if __name__ == '__main__':
    unittest.main()
