"""
result ~30Hz
"""

import time

from compapy.scripts.CoMPaPy import CoMPaPy


def main(n_measures: int = 1000):
    compapy = CoMPaPy()

    start_time = time.time()
    for _ in range(n_measures):
        joints = compapy.get_joints()
        # print(joints)
    duration = time.time() - start_time
    print(f'freq = {n_measures / duration:.1f}')


if __name__ == '__main__':
    main()
