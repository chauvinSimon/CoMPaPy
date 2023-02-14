"""
convert a pose (position and orientation) between
- `panda_EE`
- `panda_link8`
the pose is a 6d list, where the orientation is encoded as a rotvec (! NOT EULER / RPY !)
"""

import numpy as np
from scipy.spatial.transform import Rotation

d_ee_l8_m = 0.1035


def ee_in_base(l8_in_base):
    # todo: test
    ee_in_l8 = np.array([0, 0, d_ee_l8_m])
    r_l8_to_ee = Rotation.from_euler(angles=[0, 0, -45], seq='xyz', degrees=True)

    print(f'l8_in_base: {[round(e, 2) for e in l8_in_base[:3]]}')
    base_to_l8_euler_deg = Rotation.from_rotvec(l8_in_base[3:]).as_euler('xyz', degrees=True)
    print(f'base_to_l8_euler_deg: {[round(e, 2) for e in base_to_l8_euler_deg]}')

    ee_in_base = np.zeros_like(np.array(l8_in_base))

    # rotations: ee_in_base = l8_in_base * ee_in_l8
    r_b_to_l8 = Rotation.from_rotvec(l8_in_base[3:])
    r_b_to_ee = r_b_to_l8 * r_l8_to_ee
    ee_in_base[3:] = r_b_to_ee.as_rotvec()

    # translations: b->ee = ee->l8 + rot(b->l8) * 8->ee
    ee_in_base[:3] = l8_in_base[:3] + r_b_to_l8.as_matrix() @ ee_in_l8

    print(f'ee_in_base: {[round(e, 2) for e in ee_in_base[:3]]}')
    base_to_ee_euler_deg = Rotation.from_rotvec(ee_in_base[3:]).as_euler('xyz', degrees=True)
    print(f'base_to_ee_euler_deg: {[round(e, 2) for e in base_to_ee_euler_deg]}')

    return ee_in_base


def link8_in_base(ee_in_base):
    l8_in_ee = np.array([0, 0, -d_ee_l8_m])
    r_ee_to_l8 = Rotation.from_euler(angles=[0, 0, 45], seq='xyz', degrees=True)

    print(f'ee_in_base: {[round(e, 2) for e in ee_in_base[:3]]}')
    base_to_ee_euler_deg = Rotation.from_rotvec(ee_in_base[3:]).as_euler('xyz', degrees=True)
    print(f'base_to_ee_euler_deg: {[round(e, 2) for e in base_to_ee_euler_deg]}')

    l8_in_base = np.zeros_like(np.array(ee_in_base))

    # rotations: l8_in_base = ee_in_base * l8_in_ee
    r_b_to_ee = Rotation.from_rotvec(ee_in_base[3:])
    r_b_to_l8 = r_b_to_ee * r_ee_to_l8
    l8_in_base[3:] = r_b_to_l8.as_rotvec()

    # translations: b->8 = b->ee + rot(b->e) * ee->8
    l8_in_base[:3] = ee_in_base[:3] + r_b_to_ee.as_matrix() @ l8_in_ee

    print(f'l8_in_base: {[round(e, 2) for e in l8_in_base[:3]]}')
    base_to_l8_euler_deg = Rotation.from_rotvec(l8_in_base[3:]).as_euler('xyz', degrees=True)
    print(f'base_to_l8_euler_deg: {[round(e, 2) for e in base_to_l8_euler_deg]}')

    return l8_in_base


def main():
    # example, with orientation of the default pose
    ee_xyz_rot_vec_in_base = link8_in_base([0., 0., 0., np.pi, 0., 0.])
    print(ee_xyz_rot_vec_in_base)


if __name__ == '__main__':
    Rotation.from_euler(
        angles=[np.pi, 0., np.deg2rad(45)], seq='xyz', degrees=True
    ).as_rotvec().tolist()

    main()
