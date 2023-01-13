"""
~/catkin_ws/src/compapy$ py.test scripts/socket_interface/test_frame_conversion.py
"""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation


from compapy.scripts.socket_interface.frame_conversion import link8_in_base, ee_in_base


@pytest.mark.parametrize("ee_in_base, l8_in_base", [
    # ee is at the base-origin, with fingers pointing up.
    # Therefore, l8 below z=0 and rz=+45°
    ([0., 0., 0., 0., 0., 0.], [0., 0., -0.113, 0., 0., np.deg2rad(45)]),

    # same orientation, but offset (-1.9, 2.0) for ee
    ([-1.9, 2., 0., 0., 0., 0.], [-1.9, 2., -0.113, 0., 0., np.deg2rad(45)]),

    # ee is at the base-origin, but with fingers pointing down
    # l8 higher than ee
    # in base, first 180 around x, then -45 around z
    ([0., 0., 0., np.pi, 0., 0.], [0., 0., 0.113,
                                   *Rotation.from_euler(
                                       angles=[np.pi, 0., -np.deg2rad(45)], seq='xyz'
                                   ).as_rotvec().tolist()
                                   ]),

    # same with offset (0.9, -0.1)
    ([0.9, -0.1, 0., np.pi, 0., 0.], [0.9, -0.1, 0.113,
                                      *Rotation.from_euler(
                                          angles=[np.pi, 0., -np.deg2rad(45)], seq='xyz'
                                      ).as_rotvec().tolist()
                                      ]),

    # the fingers look at the yumi
    ([0., 0., 0., -np.pi / 2, 0., 0.], [0., -0.113, 0.,
                                        *Rotation.from_euler(
                                            angles=[-np.pi / 2, np.deg2rad(45), 0.], seq='xyz'
                                        ).as_rotvec().tolist()
                                        ]),

])
def test_link8_in_base(ee_in_base, l8_in_base):
    tolerance = 1e-3
    l8_in_base_expected = np.array(l8_in_base)
    l8_in_base_res = np.array(link8_in_base(ee_in_base=ee_in_base))
    np.testing.assert_allclose(
        l8_in_base_expected,
        l8_in_base_res,
        atol=tolerance,
        err_msg=f''
                f'l8_in_base_expected: [{l8_in_base_expected.tolist()}]'
                f'l8_in_base_res: [{l8_in_base_res.tolist()}]'
                f''
    )


@pytest.mark.parametrize("l8_in_base, ee_in_base_expected", [
    # l8 is at the base-origin, with fingers pointing up.
    # Therefore, ee above z=0 and rz=-45°
    ([0., 0., 0., 0., 0., 0.], [0., 0., 0.113, 0., 0., -np.deg2rad(45)]),

    # same orientation, but offset (-1.9, 2.0) for ee
    ([-1.9, 2., 0., 0., 0., 0.], [-1.9, 2., 0.113, 0., 0., -np.deg2rad(45)]),

    # l8 is at the base-origin, but with fingers pointing down
    # ee lower than l8
    # in base, first 180 around x, then -45 around z
    (
            [0., 0., 0., *Rotation.from_euler(
                angles=[np.pi, 0., -np.deg2rad(45)], seq='xyz'
            ).as_rotvec().tolist()],
            [0., 0., -0.113, np.pi, 0., 0.]
    ),

    # same with offset (0.9, -0.1)
    (
            [0.9, -0.1, 0., *Rotation.from_euler(
                angles=[np.pi, 0., -np.deg2rad(45)], seq='xyz'
            ).as_rotvec().tolist()],
            [0.9, -0.1, -0.113, np.pi, 0., 0.]
    ),

    # the fingers look at the yumi
    (
            [0., 0., 0., *Rotation.from_euler(
                angles=[-np.pi / 2, np.deg2rad(45), 0.], seq='xyz'
            ).as_rotvec().tolist()],
            [0., 0.113, 0., -np.pi / 2, 0., 0.]
    ),
])
def test_ee_in_base(l8_in_base, ee_in_base_expected):
    tolerance = 1e-3
    ee_in_base_expected = np.array(ee_in_base_expected)
    ee_in_base_res = np.array(ee_in_base(l8_in_base=l8_in_base))
    np.testing.assert_allclose(
        ee_in_base_expected,
        ee_in_base_res,
        atol=tolerance,
        err_msg=f''
                f'ee_in_base_expected: [{ee_in_base_expected.tolist()}]'
                f'ee_in_base_res: [{ee_in_base_res.tolist()}]'
                f''
    )
