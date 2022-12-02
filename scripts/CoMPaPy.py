#! /usr/bin/env python3

# todo: make a node from it instead of running it?
#  with a callback and some spin()

import actionlib
import franka_gripper.msg
import geometry_msgs
from geometry_msgs.msg import Pose
import json
import moveit_msgs.msg
import numpy as np
from pathlib import Path
from typing import Dict

from moveit_tutorials.doc.move_group_python_interface.scripts.move_group_python_interface_tutorial import \
    MoveGroupPythonInterfaceTutorial


def json_load(path: Path):
    with path.open('r') as f:
        return json.load(f)


class CoMPaPy(MoveGroupPythonInterfaceTutorial):
    def __init__(self, scene_file: Path):
        super(CoMPaPy, self).__init__()
        scene = json_load(scene_file)
        for element in scene["elements"]:
            self.add_scene_element(element)

    def add_scene_element(self, element: Dict) -> None:
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

    def cleanup_scene(self):
        self.move_group.remove_all_elements()

    def move_j(self, target_pose: geometry_msgs.msg.Pose) -> bool:
        self.move_group.set_pose_target(target_pose)
        success = self.move_group.go(wait=True)

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
        print(f'want to travel [{distance * 100:.1f} cm]')

        waypoints = [
            target_pose,
        ]

        # todo: tune it
        #  "The computed trajectory is too short to detect jumps in joint-space Need at least 10 steps,
        #   only got 3. Try a lower max_step."
        resolution_m = 0.005

        self.move_group.limit_max_cartesian_link_speed(speed=0.05)

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
            jump_threshold=5.0,

            # avoid_collisions=True
        )

        print(f'[{fraction:.1%}] = fraction of the path achieved as described by the waypoints')
        if fraction == -1.0:
            print('error with compute_cartesian_path')
        else:
            n_points = len(plan.joint_trajectory.points)
            print(f'[{n_points}] points (resolution_m = {resolution_m * 100:.1f} cm) -> '
                  f'path.length at most {n_points * resolution_m * 100:.1f} cm')
        # Note: we are just planning, not asking move_group to actually move the robot yet

        # display a saved trajectory
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

        # todo: check that the robot's current joint state is within some tolerance of the first waypoint (current)
        self.move_group.execute(plan, wait=True)

        success = True
        return success

    @staticmethod
    def _open_gripper(
            width=0.08,
            speed=0.4
    ):
        # Initialize actionLib client
        move_client = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)
        move_client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = franka_gripper.msg.MoveGoal()
        goal.width = width
        goal.speed = speed

        # Sends the goal to the action server.
        move_client.send_goal(goal)

        # Waits for the server to finish performing the action.
        move_client.wait_for_result()

        # Prints out the result of executing the action
        result = move_client.get_result()
        return result.success

    @staticmethod
    def _close_gripper(
            width=0.05,
            epsilon_inner=0.01,
            epsilon_outer=0.01,
            force=20,
            speed=0.1,
    ):
        # todo: compare with https://github.com/frankaemika/franka_ros/issues/256
        # goal.width = 0.022
        # goal.epsilon.inner = 0.005
        # goal.epsilon.outer = 0.005
        # goal.speed = 0.1
        # goal.force = 5

        # Initialize actionLib client
        grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)
        grasp_client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = franka_gripper.msg.GraspGoal()
        goal.width = width
        goal.epsilon.inner = epsilon_inner
        goal.epsilon.outer = epsilon_outer
        goal.speed = speed
        goal.force = force

        # Sends the goal to the action server.
        grasp_client.send_goal(goal)

        # Waits for the server to finish performing the action.
        grasp_client.wait_for_result()

        # Prints out the result of executing the action
        result = grasp_client.get_result()
        return result.success

    def open_gripper(self):
        self._open_gripper()

    def close_gripper(self):
        self._close_gripper()


def main():
    import copy

    scene_file = Path('config/scene.json')
    my_move_interface = CoMPaPy(scene_file=scene_file)

    # gripper
    my_move_interface.open_gripper()
    my_move_interface.close_gripper()
    my_move_interface.open_gripper()
    my_move_interface.close_gripper()

    # reach a ref position, stretching up
    my_move_interface.go_to_joint_state()

    # move inside a virtual cube
    cube_size = 0.05
    init_pose = my_move_interface.move_group.get_current_pose().pose
    targets = []
    # 1- along the internal "long" diagonal
    target_1 = copy.deepcopy(init_pose)
    target_1.position.x += cube_size
    target_1.position.y += cube_size
    target_1.position.z += cube_size
    targets.append(target_1)

    # 2- along the external diagonal of one face
    target_2 = copy.deepcopy(init_pose)
    target_2.position.y += cube_size
    targets.append(target_2)

    # 3- along the edge of one face, vertically
    target_3 = copy.deepcopy(init_pose)
    target_3.position.y += cube_size
    target_3.position.z += cube_size
    targets.append(target_3)

    # 4- along the external diagonal of one face, back to the init
    targets.append(init_pose)

    for target_pose in targets:
        my_move_interface.move_l(target_pose)


if __name__ == '__main__':
    main()
