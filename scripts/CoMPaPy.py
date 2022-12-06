#! /usr/bin/env python3

# todo: make a node from it instead of running it?
#  with a callback and some spin()

import actionlib
import franka_gripper.msg
import geometry_msgs
from compapy.scripts.utils import setup_logger, read_yaml
from geometry_msgs.msg import Pose
import json
import moveit_msgs.msg
import numpy as np
from pathlib import Path
import time
from typing import Dict, Optional

from moveit_tutorials.doc.move_group_python_interface.scripts.move_group_python_interface_tutorial import \
    MoveGroupPythonInterfaceTutorial


def json_load(path: Path):
    with path.open('r') as f:
        return json.load(f)


class CoMPaPy(MoveGroupPythonInterfaceTutorial):
    def __init__(self, log_file: Optional[Path] = None):
        super(CoMPaPy, self).__init__()

        if log_file is None:
            saving_dir = Path('logs') / str(time.strftime("%Y%m%d_%H%M%S"))
            saving_dir.mkdir(exist_ok=True, parents=True)
            log_file = saving_dir / 'log.log'
        self.logger = setup_logger(self.__class__.__name__, log_file=log_file)

        self.config = read_yaml(Path('config/compapy.yaml'))

        obstacles_file = Path('config/obstacles.json')  # todo: make it param
        if not obstacles_file.exists():
            raise FileExistsError(f'obstacles_file not found: [{obstacles_file}]\n'
                                  f'did you run from [~/catkin_ws/src/compapy]?')
        obstacles_data = json_load(obstacles_file)
        for obstacle in obstacles_data['obstacles']:
            self._add_scene_element(obstacle)

    def _add_scene_element(self, element: Dict) -> None:
        box_name = element["name"]

        if element["type"] == "static_box":
            box_pose = geometry_msgs.msg.PoseStamped()
            box_pose.pose.position.x = float(element["x"])
            box_pose.pose.position.y = float(element["y"])
            box_pose.pose.position.z = float(element["z"])
            box_pose.header.frame_id = "panda_link0"

            box_size = (
                float(element["size_x"]),
                float(element["size_y"]),
                float(element["size_z"])
            )
            self.scene.add_box(name=box_name, pose=box_pose, size=box_size)
            # todo: wait_for_state_update

        else:
            raise NotImplementedError(f'cannot deal with element["type"]=[{element["type"]}]')

    def move_j(self, target_pose: geometry_msgs.msg.Pose) -> bool:
        self.move_group.set_pose_target(target_pose)
        success = self.move_group.go(wait=True)  # todo: how to set speed?

        self.move_group.stop()  # ensures that there is no residual movement
        self.move_group.clear_pose_targets()

        return success

    def move_l(self, target_pose: geometry_msgs.msg.Pose) -> bool:
        # do not add current point to the waypoint list
        #   otherwise: "Trajectory message contains waypoints that are not strictly increasing in time."
        #   `https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/`
        start = self.move_group.get_current_pose().pose
        distance = np.linalg.norm([
            target_pose.position.x - start.position.x,
            target_pose.position.y - start.position.y,
            target_pose.position.z - start.position.z,
        ])
        self.logger.info(f'want to travel [{distance * 100:.1f} cm]')

        waypoints = [
            target_pose,
        ]

        self.move_group.limit_max_cartesian_link_speed(
            speed=self.config['move_l']['speed']
        )

        # todo: tune it
        #  "The computed trajectory is too short to detect jumps in joint-space Need at least 10 steps,
        #   only got 3. Try a lower max_step."
        resolution_m = self.config['move_l']['resolution_m']

        # takes as input waypoints of end effector poses, and outputs a joint trajectory that visits each pose
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints=waypoints,

            # configurations are computed for every eef_step meters
            # it should be a step size of at most `eef_step` meters between end effector configurations
            #   of consecutive points in the result trajectory
            eef_step=resolution_m,

            # jump_threshold against joint flips (cannot continue following the waypoints without reorienting the arm)
            # solution: limit the maximum distance ("jump") in joint positions between two consecutive trajectory points
            # todo: see tuning: https://thomasweng.com/moveit_cartesian_jump_threshold/
            # todo: enable jump_threshold (not 0.0), to check for infeasible jumps in joint space
            #  disabling the jump threshold while operating real hardware can cause
            #  large unpredictable motions of redundant joints and could be a safety issue
            jump_threshold=self.config['move_l']['jump_threshold'],

            # avoid_collisions=True  # todo: understand. collision wrt robot or wrt scene-obstacles?
        )

        self.logger.info(f'[{fraction:.1%}] = fraction of the path achieved as described by the waypoints')
        if fraction == -1.0:
            self.logger.error('error with compute_cartesian_path')
            # todo: fallback

        else:
            n_points = len(plan.joint_trajectory.points)
            self.logger.info(f'[{n_points}] points (resolution_m = {resolution_m * 100:.1f} cm) -> '
                             f'path.length at most {n_points * resolution_m * 100:.1f} cm')
            if fraction < 1.0:
                self.logger.warning('path not complete')
                # todo: fallback

        # Note: we are just planning, not asking move_group to actually move the robot yet

        # display a saved trajectory
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

        # todo: check that the robot's current joint state is within some tolerance of the first waypoint (current)
        self.move_group.execute(plan, wait=True)

        success = True  # todo: find if success or not
        return success

    def open_gripper(self):
        # Initialize actionLib client
        move_client = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)
        move_client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = franka_gripper.msg.MoveGoal()
        goal.width = self.config['open_gripper']['width']
        goal.speed = self.config['open_gripper']['speed']

        # Sends the goal to the action server.
        move_client.send_goal(goal)

        # Waits for the server to finish performing the action.
        move_client.wait_for_result()

        # Prints out the result of executing the action
        result = move_client.get_result()
        return result.success

    def close_gripper(self):
        # Initialize actionLib client
        grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)
        grasp_client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = franka_gripper.msg.GraspGoal()
        goal.width = self.config['close_gripper']['width']
        goal.epsilon.inner = self.config['close_gripper']['epsilon_inner']
        goal.epsilon.outer = self.config['close_gripper']['epsilon_outer']
        goal.speed = self.config['close_gripper']['speed']
        goal.force = self.config['close_gripper']['force']

        # Sends the goal to the action server.
        grasp_client.send_goal(goal)

        # Waits for the server to finish performing the action.
        grasp_client.wait_for_result()

        # Prints out the result of executing the action
        result = grasp_client.get_result()
        return result.success

    def get_pose(self):
        return self.move_group.get_current_pose().pose

    def get_joints(self):
        return self.move_group.get_current_joint_values()

    def get_gripper_width_mm(self):
        import rospy
        from sensor_msgs.msg import JointState

        msg = rospy.wait_for_message('/franka_gripper/joint_states', JointState, timeout=5)

        if msg.name != ['panda_finger_joint1', 'panda_finger_joint2']:
            self.logger.error(f'[gripper] msg.name = {msg.name}')

        joint_positions = msg.position
        self.logger.info(f'[gripper] joint_positions = {joint_positions}')

        # as strange as it can be, "width" can be directly derived from the two joint angles
        width_mm = 1000 * sum(joint_positions)

        return width_mm
