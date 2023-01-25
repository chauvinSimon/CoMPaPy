import numpy as np

from compapy.scripts.CoMPaPy import CoMPaPy


def rotate_eef(
        a_deg_max: int,
        n_positive_angles: int,
        max_error_angle_deg: float = 1.0
):
    compapy = CoMPaPy()

    a_degs = list(np.linspace(start=0, stop=a_deg_max, num=n_positive_angles)) + [0.]  # back to zero at the end
    for a_deg in a_degs:
        for sign in [1, -1]:
            a_deg *= sign
            success = compapy.rotate_joint('panda_joint7', np.deg2rad(a_deg))
            if not success:
                raise RuntimeError(f'failed to move eef to [{a_deg}] deg')
            res_deg = np.rad2deg(compapy.get_joints()[6])
            print(f'target = [{a_deg:.1f}]  actual = [{res_deg:.1f}] (deg)')
            assert abs(res_deg - a_deg) < max_error_angle_deg, f'{a_deg:.1f} not reached: {res_deg:.1f}'


def main():
    rotate_eef(
        a_deg_max=10,
        n_positive_angles=5
    )


if __name__ == '__main__':
    main()
