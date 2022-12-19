from dataclasses import dataclass
import json
import logging
import numpy as np
import os
from pathlib import Path
from typing import List, Optional, Dict
import yaml

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

formatter = logging.Formatter('%(levelname)s - %(asctime)s - %(module)s: %(message)s')


# todo: in pycharm (log highlighting ideolog) set format pattern to
#   ^(\d{4}-\d{2}-\d{2}\s\d{2}:\d{2}:\d{2},\d{3})\s(\S*)\s*(\w*)\s*(.*)$
#   note: configure `Grep Console` to format logs into the console


def setup_logger(name: str, log_file: Optional[Path] = None, level=logging.DEBUG):
    print(f'logging [{name}] at [{log_file}]')

    logger = logging.getLogger(name)
    logger.setLevel(level)

    if log_file is not None:
        log_file.parent.mkdir(exist_ok=True, parents=True)

        handler = logging.FileHandler(log_file)
        handler.setFormatter(formatter)
        logger.addHandler(handler)

    ch = logging.StreamHandler()
    ch.setLevel(level)
    ch.setFormatter(formatter)
    logger.addHandler(ch)

    return logger


def json_load(path: Path) -> Dict:
    with path.open('r') as f:
        return json.load(f)


def json_dump(obj, path: Path) -> None:
    with path.open('w') as f:
        json.dump(obj, f, indent=4)


def read_yaml(p: Path) -> Dict:
    if not p.exists():
        raise FileExistsError(f'{p} does not exist')
    with p.open("r") as stream:
        return yaml.safe_load(stream)


def list_to_pose(li: List[float]) -> Pose:
    return Pose(
        Point(*li[:3]),
        Quaternion(*li[3:])
    )


def pose_to_list(pose: Pose) -> List[float]:
    return [
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]


def wrap_to_pi(a_rad: float) -> float:
    return (a_rad + np.pi) % (2 * np.pi) - np.pi


def get_latest_folder(root_folder: Path, pattern: str = '*/') -> Path:
    latest_folder = max([
        p for p in Path(root_folder).glob(pattern)
        if p.is_dir()
    ],
        key=os.path.getmtime
    )
    return latest_folder


@dataclass
class PlanningRes:
    start_pose: Pose
    target_pose: Pose
    plan: RobotTrajectory
    fraction: float

    @staticmethod
    def pose_to_dict(p) -> Dict:
        return {
            'position': {
                'x': p.position.x,
                'y': p.position.y,
                'z': p.position.z,
            },
            'orientation': {
                'x': p.orientation.x,
                'y': p.orientation.y,
                'z': p.orientation.z,
                'w': p.orientation.w,
            },
        }

    @staticmethod
    def plan_to_dict(t: RobotTrajectory) -> Dict:
        def p_to_dict(_p: JointTrajectoryPoint) -> Dict:
            return {
                'positions': _p.positions,
                'velocities': _p.velocities,
                'accelerations': _p.accelerations,
                'effort': _p.effort,
                'time_from_start': {
                    'secs': _p.time_from_start.secs,
                    'nsecs': _p.time_from_start.nsecs
                },
            }

        return {
            'joint_names': t.joint_trajectory.joint_names,
            'points': [
                p_to_dict(p)
                for p in t.joint_trajectory.points
            ]
        }

    def save(self, saving_path: Path) -> None:
        d = {
            'start_pose': self.pose_to_dict(self.start_pose),
            'target_pose': self.pose_to_dict(self.target_pose),
            'plan': self.plan_to_dict(self.plan),
            'fraction': self.fraction
        }

        json_dump(d, saving_path)


def log_example():
    # first file logger
    logger = setup_logger('first_logger', Path('logs/first_logfile.log'))
    logger.info('This is just info message')

    # second file logger
    super_logger = setup_logger('second_logger', Path('logs/second_logfile.log'))
    super_logger.error('This is an error message')


if __name__ == '__main__':
    log_example()
