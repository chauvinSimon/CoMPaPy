"""
usage:
    roslaunch panda_moveit_config demo_gazebo.launch
    python scripts/test_move_l_planner.py
"""

import copy
import numpy as np
from pathlib import Path
import random
import time
from typing import Optional

from compapy.scripts.CoMPaPy import CoMPaPy
from compapy.scripts.utils import json_load, list_to_pose, get_latest_folder, setup_logger

# todo: even with these two `seed()` settings, results differ from one experiment to the next one
#  probably a cpp seed setting is required instead
random.seed(0)
np.random.seed(0)

logger = setup_logger('test_move_l')


def generate_plans(n_experiments: int = 10) -> Path:
    saving_dir = Path('logs') / str(time.strftime("%Y%m%d_%H%M%S"))
    saving_dir.mkdir(exist_ok=True, parents=True)
    compapy = CoMPaPy(
        log_file=saving_dir / 'compapy.log',
        save_planning_res=True
    )

    # 1/2 - either use explicit poses (e.g. to reproduce/replay logs) ...
    # start_pose = [0.3069338191380155, -3.3656799456633656e-05, 0.5905545988619569, -0.9238508175468446,
    #               0.3827520856183703, -0.0006625223844351978, 0.0002625583832704818]
    # target_pose = [0.3069338191380155, -3.3656799456633656e-05, 0.640554598861957, -0.9238508175468446,
    #                0.3827520856183703, -0.0006625223844351978, 0.0002625583832704818]
    # init_pose = list_to_pose(start_pose)
    # target_pose = list_to_pose(target_pose)

    # 2/2 - ... or with current pose and offset
    init_pose = compapy.move_group.get_current_pose().pose
    target_pose = copy.deepcopy(init_pose)
    target_pose.position.x += 0.15
    target_pose.position.z += 0.15

    for i_trial in range(n_experiments):
        print(i_trial)
        compapy.plan_move_l(
            start_pose=init_pose,
            target_pose=target_pose
        )
    return saving_dir


def check_saved_plans(planning_res: Optional[Path] = None):
    if planning_res is None:
        planning_res = get_latest_folder(root_folder=Path('logs'), pattern='*/planning_res')

    logger.info(f'checking folder [{planning_res}]')

    planning_results = [
        json_load(f_json)
        for f_json in planning_res.glob('*.json')
    ]
    n_points = [len(res['plan']['points']) for res in planning_results]
    print(f'n_points = {n_points}')

    if len(set(n_points)) != 1:
        error_msg = f'plans do not have same size: n_points = {n_points}'
        logger.error(error_msg)
        raise ValueError(error_msg)

    for i_point in range(n_points[0]):
        print(f'\n[{i_point}/{n_points[0]}]')
        points = [res['plan']['points'][i_point] for res in planning_results]

        start_position = [res['start_pose']['position'] for res in planning_results]
        start_position = [[p['x'], p['y'], p['z']] for p in start_position]

        start_orientation = [res['start_pose']['orientation'] for res in planning_results]
        start_orientation = [[p['x'], p['y'], p['z'], p['w']] for p in start_orientation]

        target_position = [res['target_pose']['position'] for res in planning_results]
        target_position = [[p['x'], p['y'], p['z']] for p in target_position]

        target_orientation = [res['target_pose']['orientation'] for res in planning_results]
        target_orientation = [[p['x'], p['y'], p['z'], p['w']] for p in target_orientation]

        positions = [p['positions'] for p in points]
        velocities = [p['velocities'] for p in points]
        accelerations = [p['accelerations'] for p in points]
        nsecs = [p['time_from_start']['nsecs'] for p in points]

        for values, name, threshold in [
            (start_position, 'start_position', 1e-7),
            (start_orientation, 'start_orientation', 1e-7),

            (target_position, 'target_position', 1e-7),
            (target_orientation, 'target_orientation', 1e-7),

            (positions, 'positions', 1e-1),
            (velocities, 'velocities', 1e-1),
            (accelerations, 'accelerations', 1e-1),
            (nsecs, 'nsecs', 1e6),
        ]:
            values = np.array(values)
            deltas = np.max(values, axis=0) - np.min(values, axis=0)
            max_deltas = np.max(deltas)
            print(f'[{name}].deltas.max = {max_deltas}')

            if max_deltas > threshold:
                print(f'on delta in [{name}] is big')
                print(f'max = {max_deltas}')
                print(f'deltas = {deltas}')
                print(f'values = {values}')
                raise ValueError(f'delta in [{name}] ({i_point}-th point)')


def main():
    # 1/2 - either plan and test ...
    saving_dir = generate_plans()
    check_saved_plans(planning_res=saving_dir / 'planning_res')

    # 2/2 - ... or test the latest
    # check_saved_plans()


if __name__ == '__main__':
    main()
