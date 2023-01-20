"""
testing `plan_l` for A->B with different B
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

from compapy.scripts.CoMPaPy import CoMPaPy
from compapy.scripts.utils import list_to_pose, pose_to_list

np.random.seed(0)


def plan_or_move_l_in_target_space(
        compapy,
        n_samples: int = 10,
        n_plan_trials: int = 5,
        fallback_plan_j: bool = False
):
    # move to A
    target_init_pose = [
        0.5, 0, 0.6125,
        0.923879532, -0.38268343, 0.0, 0.0
    ]
    target_init_pose = list_to_pose(target_init_pose)
    # target_init_pose = compapy.move_group.get_current_pose().pose

    success, error_msg = compapy.move_j(target=target_init_pose)
    if not success:
        raise RuntimeError(f'cannot reach init pose: {error_msg}')

    init_pose = compapy.move_group.get_current_pose().pose
    for res, expect in zip(pose_to_list(init_pose)[:3], pose_to_list(target_init_pose)[:3]):
        assert abs(res - expect) < 1e-2, f'{init_pose} \n!= \n{target_init_pose}'

    # target space
    x_semi_size = 0.15
    x_min = 0.01 - x_semi_size
    x_max = 0.01 + x_semi_size

    y_semi_size = 0.05
    y_min = -0.65 - y_semi_size
    y_max = -0.65 + y_semi_size

    l8_to_ee = 0.113
    z_min = -0.164 + l8_to_ee
    z_max = -0.164 + l8_to_ee
    z_above_offset = 0.55

    rz_deg_min = -360
    rz_deg_max = 360

    resolution_m = compapy.config['move_l']['resolution_m']
    resolution_m_offset = 0
    jump_threshold = compapy.config['move_l']['jump_threshold']
    jump_threshold_offset = 0

    failure_msgs = []
    failure_angles = []
    successful_multiple_plan_l = 0
    successful_fallback_plan_j = 0
    saver_resolution_offsets = []
    saver_jump_offsets = []

    # sample a B and try to plan A->B
    for i in range(n_samples):
        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)
        z = np.random.uniform(z_min, z_max)
        z += z_above_offset
        rz_ee_deg = np.random.uniform(rz_deg_min, rz_deg_max)

        quat = Rotation.from_euler(
            # assume gripper pointing down (therefore roll=180°)
            # target (rz_ee_deg) is ee. need to convert to link8
            angles=[np.pi, 0., -np.pi / 4 + np.deg2rad(rz_ee_deg)], seq='xyz'
        ).as_quat().tolist()

        target_pose = [
            x, y, z,
            *quat,
        ]
        target_pose = list_to_pose(target_pose)

        print(
            f'[{i}]/[{n_samples - 1}]: '
            f'x={target_pose.position.x * 100:.1f}  '
            f'y={target_pose.position.y * 100:.1f}  '
            f'z={target_pose.position.z * 100:.1f}  '
            f'rz_ee_deg={rz_ee_deg:.1f}  '
        )

        success_plan = False
        fractions = []
        for i_trial in range(n_plan_trials):
            if i_trial > 0:
                resolution_m_offset = np.random.uniform(-0.005, 0.005)
                jump_threshold_offset = np.random.uniform(-2.0, 2.0)
            success_plan, plan, fraction = compapy.plan_l(
                target_pose=target_pose,
                resolution_m=resolution_m + resolution_m_offset,
                jump_threshold=jump_threshold + jump_threshold_offset
            )
            fractions.append(fraction)
            if success_plan:
                if i_trial > 0:
                    compapy.logger.info('Hurra! trying multiple params for `plan_l` was worth')
                    successful_multiple_plan_l += 1
                    saver_resolution_offsets.append(resolution_m_offset)
                    saver_jump_offsets.append(jump_threshold_offset)
                break
            if fallback_plan_j:
                compapy.logger.warning('trying to fallback with `plan_j` ...')
                success_plan, plan, _ = compapy.plan_j(
                    target=target_pose
                )
                if success_plan:
                    successful_fallback_plan_j += 1
                    compapy.logger.info('Hurra! managed to fallback!')

        if not success_plan:
            msg = f'[{i + 1:<3}/{n_samples}] failed fraction in [{min(fractions):.3%}, {max(fractions):.3%}] ' \
                  f'(width = {(max(fractions) - min(fractions)):.3%}) after [{n_plan_trials}] trials' \
                  f'([x={x * 100:.2f} cm] y=[{y * 100:.2f} cm] [z={z * 100:.2f} cm] [rz_ee_deg={rz_ee_deg:.1f} deg])'
            failure_msgs.append(msg)
            failure_angles.append(rz_ee_deg)

    if failure_msgs:
        percent_str = f'{len(failure_msgs) / n_samples:.1%}'
        print(f'\n{len(failure_msgs)} failures [{percent_str}]:')
        for msg in failure_msgs:
            print(msg)

        positives = [x for x in failure_angles if x >= 0]
        negatives = [x for x in failure_angles if x <= 0]
        bounds_str = f'[{min(failure_angles):.1f}'
        if negatives:
            bounds_str += f' | {max(negatives):.1f}'
        if positives:
            bounds_str += f' | {min(positives):.1f}'
        bounds_str += f' | {max(failure_angles):.1f}]'
        print(f'problematic angles (deg) bounds {bounds_str}: {failure_msgs}')
        data = failure_angles
        bin_width = 1

        plt.hist(
            data,
            bins=np.arange(np.floor(min(data)), np.ceil(max(data) + bin_width), bin_width)
        )
        plt.xlim(xmin=rz_deg_min, xmax=rz_deg_max)
        plt.title(f'{len(failure_angles)} ({percent_str}) failure_angles {bounds_str} (deg)')
        plt.savefig('failures.png')
        plt.show()

    else:
        print(f'no problem in [{n_samples}] samples :)')

    print(f'successful_fallback_plan_j = [{successful_fallback_plan_j}]')

    print(f'successful_multiple_plan_l = [{successful_multiple_plan_l}]:')
    if successful_multiple_plan_l > 0:
        print(f'\tsaver_resolution_offsets = {saver_resolution_offsets}')
        print(f'\tsaver_jump_offsets = {saver_jump_offsets}')
        for name, data in [
            ('resolution_offsets', saver_resolution_offsets),
            ('jump_offsets', saver_jump_offsets),
        ]:
            plt.hist(data)
            plt.title('saver_resolution_offsets')
            plt.show()


def main():
    compapy = CoMPaPy()

    plan_or_move_l_in_target_space(
        compapy=compapy,
        n_samples=100,
        n_plan_trials=10
    )


if __name__ == '__main__':
    main()
