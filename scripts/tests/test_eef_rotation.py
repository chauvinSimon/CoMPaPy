import numpy as np

from compapy.scripts.CoMPaPy import CoMPaPy


def rotate_eef(
        a_deg_max: int,
        n_positive_angles: int
):
    compapy = CoMPaPy()

    for a_deg in np.linspace(start=0, stop=a_deg_max, num=n_positive_angles):
        success = compapy.rotate_joint('panda_joint7', np.deg2rad(a_deg))
        if not success:
            raise RuntimeError(f'failed to move eef to [{a_deg}] deg')

        success = compapy.rotate_joint('panda_joint7', -np.deg2rad(a_deg))
        if not success:
            raise RuntimeError(f'failed to move eef to [{a_deg}] deg')


def main():
    rotate_eef(
        a_deg_max=10,
        n_positive_angles=5
    )


if __name__ == '__main__':
    main()
