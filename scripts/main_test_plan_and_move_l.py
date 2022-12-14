import numpy as np
from scipy.spatial.transform import Rotation

from compapy.scripts.CoMPaPy import CoMPaPy
from compapy.scripts.utils import list_to_pose, pose_to_list


def plan_or_move_l_in_target_space(
        compapy,
        want_to_move: bool = False,
        n_samples: int = 10,
        n_plan_trials: int = 3
):
    target_init_pose = [
        0.5, 0, 0.5,
        0.923879532, 0.38268343, 0.0, 0.0
    ]
    target_init_pose = list_to_pose(target_init_pose)
    success = compapy.move_j(target_pose=target_init_pose)
    if not success:
        raise RuntimeError('cannot reach init pose')

    init_pose = compapy.move_group.get_current_pose().pose
    for res, expect in zip(pose_to_list(init_pose)[:3], pose_to_list(target_init_pose)[:3]):
        assert abs(res - expect) < 1e-2, f'{init_pose} \n!= \n{target_init_pose}'

    # target space
    xy_semi_size = 0.03
    x_min = -0.118 - xy_semi_size
    x_max = -0.118 + xy_semi_size

    y_min = -0.636 - xy_semi_size
    y_max = -0.636 + xy_semi_size

    z_min = -0.159
    z_max = -0.159
    z_above_offset = 0.15

    # todo: why not error when [-160°, 20°] and problems for >20°?
    rz_deg_min = -160
    rz_deg_max = 20

    resolution_m = compapy.config['move_l']['resolution_m']
    resolution_m_offset = 0
    jump_threshold = compapy.config['move_l']['jump_threshold']
    jump_threshold_offset = 0

    failure_msgs = []
    n_success_moves = 0

    for i in range(n_samples):
        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)
        z = np.random.uniform(z_min, z_max)
        z += z_above_offset
        rz_deg = np.random.uniform(rz_deg_min, rz_deg_max)

        quat = Rotation.from_euler(
            angles=[np.pi, 0., np.deg2rad(45) + np.deg2rad(rz_deg)], seq='xyz'
        ).as_quat().tolist()

        target_pose = [x, y, z,
                       *quat,
                       # 0., 0., 0.0, 0.0
                       ]
        target_pose = list_to_pose(target_pose)

        if want_to_move:
            print('trying [to-target] ...')
            success_to_target = compapy.move_l(target_pose=target_pose)
            i_str = f'[{i + 1}/{n_samples}]'
            print(f'{i_str} [to-target] [{"success" if success_to_target else "FAIL"}]')

            # back to start
            print('trying [back-to-start] ...')
            success_back = compapy.move_l(target_pose=target_init_pose)
            print(f'{i_str} [back-to-start] [{"success" if success_back else "FAIL"}]')

            if success_to_target and success_back:
                n_success_moves += 1
            print(f'{i_str} [to-target and back-to-start] '
                  f'[{"success" if (success_back and success_to_target) else "FAIL"}] '
                  f'(n_success_moves={n_success_moves}/{i + 1})')

        else:  # just plan
            success_plan = False
            fractions = []
            for i_trial in range(n_plan_trials):
                if i_trial > 0:
                    resolution_m_offset = np.random.uniform(-0.005, 0.005)
                    jump_threshold_offset = np.random.uniform(-2.0, 2.0)
                plan, fraction = compapy.plan_move_l(
                    start_pose=init_pose,
                    target_pose=target_pose,
                    resolution_m=resolution_m + resolution_m_offset,
                    jump_threshold=jump_threshold + jump_threshold_offset
                )
                fractions.append(fraction)
                if fraction == 1.0:
                    success_plan = True
                    break

            if not success_plan:
                msg = f'[{i + 1:<3}/{n_samples}] failed fraction in [{min(fractions):.3%}, {max(fractions):.3%}] ' \
                      f'(width = {(max(fractions) - min(fractions)):.3%}) after [{n_plan_trials}] trials' \
                      f'([x={x * 100:.2f} cm] y=[{y * 100:.2f} cm] [z={z * 100:.2f} cm] [rz_deg={rz_deg:.1f} deg])'
                failure_msgs.append(msg)

    if not want_to_move:
        if failure_msgs:
            print(f'\n{len(failure_msgs)} failures [{len(failure_msgs) / n_samples:.1%}]:')
            for msg in failure_msgs:
                print(msg)
        else:
            print(f'no problem in [{n_samples}] samples :)')


def main():
    compapy = CoMPaPy()

    plan_or_move_l_in_target_space(
        compapy=compapy,
        n_samples=100
    )

    plan_or_move_l_in_target_space(
        compapy=compapy,
        n_samples=3,
        want_to_move=True
    )


if __name__ == '__main__':
    main()
