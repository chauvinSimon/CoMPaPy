import actionlib
from datetime import datetime
import numpy as np
from pathlib import Path
import rospy
from scipy.spatial.transform import Rotation
import time
from typing import Dict, Optional, Tuple, List

import franka_gripper.msg
import geometry_msgs
from geometry_msgs.msg import Pose
import moveit_msgs.msg
from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import JointState

from moveit_tutorials.doc.move_group_python_interface.scripts.move_group_python_interface_tutorial import \
    MoveGroupPythonInterfaceTutorial

from compapy.scripts.utils import setup_logger, read_yaml, json_load, pose_to_list, PlanningRes, wrap_to_pi


class CoMPaPy(MoveGroupPythonInterfaceTutorial):
    def __init__(
            self,
            log_file: Optional[Path] = None,
            save_planning_res: bool = False
    ):
        super(CoMPaPy, self).__init__()

        if log_file is None:
            saving_dir = Path('logs') / str(time.strftime("%Y%m%d_%H%M%S"))
            saving_dir.mkdir(exist_ok=True, parents=True)
            log_file = saving_dir / 'log.log'
        else:
            saving_dir = log_file.parent

        self.planning_res_dir = None
        if save_planning_res:
            self.planning_res_dir = saving_dir / 'planning_res'
            self.planning_res_dir.mkdir(parents=True, exist_ok=True)

        self.logger = setup_logger(self.__class__.__name__, log_file=log_file)

        self.config = read_yaml(Path('config/compapy.yaml'))

        obstacles_file = Path(self.config['obstacles']['config_path'])
        if not obstacles_file.exists():
            raise FileExistsError(f'obstacles_file not found: [{obstacles_file}]\n'
                                  f'did you run from [~/catkin_ws/src/compapy]?')
        obstacles_data = json_load(obstacles_file)
        for obstacle in obstacles_data['obstacles']:
            self._add_scene_element(obstacle)

        self._log_joints()

    def exe_plan(
            self,
            plan: RobotTrajectory
    ) -> bool:
        exe_success = self.move_group.execute(plan)
        if not exe_success:
            self.logger.error('failed to execute plan')

        self.move_group.stop()  # ensures that there is no residual movement
        self.move_group.clear_pose_targets()

        return exe_success

    def move_j(
            self,
            target_pose: Pose
    ) -> Tuple[bool, str]:
        plan_success, plan_traj, plan_error_code = self.plan_j(target_pose)
        if not plan_success:
            err_msg = 'planning failed'
            self.logger.error(err_msg)
            return False, err_msg

        exe_success = self.exe_plan(plan_traj)
        self._compute_move_error(target_pose=target_pose, move_name='move_j')
        if not exe_success:
            error_msg = 'execution of plan failed'
            self.logger.error(error_msg)
            return exe_success, error_msg

        return exe_success, ''

    def move_l(
            self,
            target_pose: Pose
    ) -> Tuple[bool, str]:
        plan_success, plan, fraction = self.plan_l(
            target_pose=target_pose,
            resolution_m=self.config['move_l']['resolution_m'],
            jump_threshold=self.config['move_l']['jump_threshold']
        )
        self._save_plan(target_pose=target_pose, plan=plan)

        if not plan_success:
            error_msg = f'planning failed: fraction=[{fraction:.1%}]'
            self.logger.error(error_msg)
            return False, error_msg

        exe_success = self.exe_plan(plan)
        self._compute_move_error(target_pose=target_pose, move_name='move_l')
        if not exe_success:
            error_msg = 'execution of plan failed'
            self.logger.error(error_msg)
            return exe_success, error_msg

        return exe_success, ''

    def plan_j(
            self,
            target_pose: Pose,
    ) -> Tuple[bool, RobotTrajectory, str]:
        plan_success, plan_traj, plan_time, plan_error_code = self.move_group.plan(target_pose)
        self._show_plan(plan_traj)
        self.logger.info(
            f'plan_success = [{plan_success}] in plan_time = [{plan_time}] with plan_error_code = [{plan_error_code}]'
        )
        return plan_success, plan_traj, plan_error_code

    def plan_l(
            self,
            target_pose: Pose,
            resolution_m: Optional[float] = None,
            jump_threshold: Optional[float] = None,
    ) -> Tuple[bool, RobotTrajectory, float]:

        if jump_threshold is None:
            jump_threshold = self.config['move_l']['jump_threshold']

        if resolution_m is None:
            resolution_m = self.config['move_l']['resolution_m']

        # do not add current point to the waypoint list
        #   otherwise: "Trajectory message contains waypoints that are not strictly increasing in time."
        #   `https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/`
        waypoints = [
            target_pose,
        ]

        # todo: add constraints
        #  https://github.com/ros-planning/moveit_tutorials/pull/518/files#diff-77946a0e5e0e873f97288add4d30861477c31fa4528736e414a0903fbaa9c438
        self.move_group.limit_max_cartesian_link_speed(
            speed=self.config['move_l']['speed']
        )

        # takes as input waypoints of end effector poses, and outputs a joint trajectory that visits each pose
        plan, fraction = self.move_group.compute_cartesian_path(
            waypoints=waypoints,

            # configurations are computed for every eef_step meters
            # it should be a step size of at most `eef_step` meters between end effector configurations
            #   of consecutive points in the result trajectory
            # todo: tune it
            #  "The computed trajectory is too short to detect jumps in joint-space Need at least 10 steps,
            #   only got 3. Try a lower max_step."
            eef_step=resolution_m,

            # jump_threshold against joint flips (cannot continue following the waypoints without reorienting the arm)
            # solution: limit the maximum distance ("jump") in joint positions between two consecutive trajectory points
            # todo: see tuning: https://thomasweng.com/moveit_cartesian_jump_threshold/
            # todo: enable jump_threshold (not 0.0), to check for infeasible jumps in joint space
            #  disabling the jump threshold while operating real hardware can cause
            #  large unpredictable motions of redundant joints and could be a safety issue
            jump_threshold=jump_threshold,

            # todo: understand. collision wrt robot or wrt scene-obstacles?
            # avoid_collisions=True

            # todo: add path_constraints?
            # todo: how to check the current joint-bounds written in `joint_limits.yaml`?
        )
        self._show_plan(plan)

        if fraction == -1.0:
            self.logger.error('error with compute_cartesian_path')

        else:
            n_points = len(plan.joint_trajectory.points)
            self.logger.info(f'[{n_points}] points (resolution_m = {resolution_m * 100:.1f} cm) -> '
                             f'path.length at most {n_points * resolution_m * 100:.1f} cm')
            if fraction < 1.0:
                self.logger.error(f'path not complete [{fraction:.1%}]')
                self._log_joints()  # todo: pass last Pose of the plan

        return fraction == 1.0, plan, fraction

    def rotate_joint(
            self,
            joint_name: str,
            target_rad: float
    ) -> Tuple[bool, str]:
        """
        docs of `Joint` object:
            https://docs.ros.org/en/jade/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander_1_1Joint.html
        """
        j = self.robot.get_joint(joint_name)

        bounds = j.bounds()
        if (target_rad < bounds[0]) or (bounds[1] < target_rad):
            bounds_deg_str = [f'{x:.1f}' for x in np.rad2deg(bounds).tolist()]
            error_msg = f'target_rad [{target_rad:.3f}] rad not in bounds {bounds} rad ' \
                        f'([{np.rad2deg(target_rad):.2f}] deg not in {bounds_deg_str} deg)'
            self.logger.error(error_msg)
            return False, error_msg

        current_rad = j.value()
        self.logger.info(f'want to rotate [{joint_name}] '
                         f'[{np.rad2deg(current_rad):.2f}] -> [{np.rad2deg(target_rad):.2f}] deg '
                         f'([{current_rad:.3f}] -> [{target_rad:.3f}] rad)'
                         f'')

        success = j.move(target_rad)
        if success:
            end_rad = j.value()
            diff_rad = wrap_to_pi(end_rad) - wrap_to_pi(target_rad)
            if abs(np.rad2deg(diff_rad)) > 0.5:
                error_msg = f'after successful rotation ' \
                            f'([{np.rad2deg(current_rad):.2f}] -> [{np.rad2deg(target_rad):.2f}] deg): ' \
                            f'diff_rad={np.rad2deg(diff_rad):.5f} ' \
                            f'target_deg={np.rad2deg(target_rad):.1f} ' \
                            f'current_deg={np.rad2deg(end_rad):.1f} '
                self.logger.error(error_msg)
                return False, error_msg

        return success, ''

    def open_gripper(self) -> bool:
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

    def close_gripper(self) -> bool:
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

    def get_pose(self) -> Pose:
        return self.move_group.get_current_pose().pose

    def get_joints(self) -> List[float]:
        return self.move_group.get_current_joint_values()

    def get_gripper_width_mm(self) -> float:
        msg = rospy.wait_for_message('/franka_gripper/joint_states', JointState, timeout=5)

        if msg.name != ['panda_finger_joint1', 'panda_finger_joint2']:
            self.logger.error(f'[gripper] msg.name = {msg.name}')

        joint_positions = msg.position
        self.logger.info(f'[gripper] joint_positions = {joint_positions}')

        # as strange as it can be, "width" can be directly derived from the two joint angles
        width_mm = 1000 * sum(joint_positions)

        return width_mm

    def _add_scene_element(
            self,
            element: Dict
    ) -> None:
        box_name = element["name"]

        if element["type"] == "static_box":
            box_pose = geometry_msgs.msg.PoseStamped()
            box_pose.pose.position.x = element["x"]
            box_pose.pose.position.y = element["y"]
            box_pose.pose.position.z = element["z"]
            box_pose.header.frame_id = "panda_link0"

            box_size = (
                element["size_x"],
                element["size_y"],
                element["size_z"]
            )
            self.scene.add_box(name=box_name, pose=box_pose, size=box_size)
            # todo: wait_for_state_update

        else:
            raise NotImplementedError(f'cannot deal with element["type"]=[{element["type"]}]')

    def _compute_move_error(
            self,
            target_pose: Pose,
            move_name: str
    ) -> None:
        current_p = self.get_pose()

        target_pose_position = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z])
        l8_pose_position = np.array([current_p.position.x, current_p.position.y, current_p.position.z])
        delta_cm = 100 * np.linalg.norm((target_pose_position - l8_pose_position))
        self.logger.info(f'after [{move_name}]: delta = [{delta_cm:0.2f} cm]')

        target_q = np.array([
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w,
        ])
        l8_q = np.array([
            current_p.orientation.x,
            current_p.orientation.y,
            current_p.orientation.z,
            current_p.orientation.w,
        ])
        target_euler_rad = Rotation.from_quat(target_q).as_euler('xyz')
        l8_euler_rad = Rotation.from_quat(l8_q).as_euler('xyz')
        delta_euler_rad = np.linalg.norm((wrap_to_pi(l8_euler_rad - target_euler_rad)))
        self.logger.info(f'after [{move_name}]: delta_euler_deg = [{np.rad2deg(delta_euler_rad):0.1f}]')

        if delta_cm > 1.0:
            self.logger.error(f'after [{move_name}]: delta_cm = [{delta_cm:.2f} cm] between target and current pose')

        if delta_euler_rad > np.deg2rad(1.0):
            self.logger.error(f'after [{move_name}]: delta_euler_deg = [{np.rad2deg(delta_euler_rad):.1f}] '
                              f'between target ({np.rad2deg(target_euler_rad)}) '
                              f'and current pose ({np.rad2deg(l8_euler_rad)})')

    def _log_joints(self) -> None:
        # todo: pass optional Pose. If pose is None, read current.
        self.logger.info('joint bounds:')
        unused = []
        for j_name in self.robot.get_joint_names():
            j = self.robot.get_joint(j_name)
            bounds_rad = j.bounds()
            if bounds_rad:
                bounds_deg = [np.rad2deg(a) for a in bounds_rad]
                bounds_deg_str = f'[{bounds_deg[0]:<6.1f}, {bounds_deg[1]:<6.1f}]'
                current_deg = np.rad2deg(j.value())
                percent_first = (current_deg - bounds_deg[0]) / (bounds_deg[1] - bounds_deg[0])
                percent_second = (bounds_deg[1] - current_deg) / (bounds_deg[1] - bounds_deg[0])
                other_info_str = f'{bounds_deg_str} - current = [{current_deg :<5.1f}] deg ' \
                                 f'([{percent_first:<5.1%}, {percent_second:<5.1%}] to [b0, b1])'
                self.logger.info(f'\t{j_name:<20}: {other_info_str}')
            else:
                unused.append(j_name)
        self.logger.info(f'unused joints: {unused}')

    def _save_plan(
            self,
            target_pose: Pose,
            plan: RobotTrajectory,
            fraction: Optional[float] = None,
            start_pose: Optional[Pose] = None,
    ) -> None:
        if self.planning_res_dir is None:
            return
        if start_pose is None:
            start_pose = self.move_group.get_current_pose().pose

        planning_res = PlanningRes(
            start_pose=start_pose,
            target_pose=target_pose,
            plan=plan,
            fraction=fraction,
        )
        fraction_str = f'{fraction:.0%}' if fraction is not None else ''
        curr_time = datetime.now()
        timestamp = curr_time.strftime('%Y%m%d_%H%M%S_%f')
        distance = self._compute_distance(target_pose=target_pose, start_pose=start_pose)
        file_name = f'{timestamp}_cm[{int(distance * 100)}]_f[{fraction_str}].json'
        planning_res.save(saving_path=self.planning_res_dir / file_name)

    def _show_plan(
            self,
            plan: RobotTrajectory
    ) -> None:
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

    def _compute_distance(
            self,
            target_pose: Pose,
            start_pose: Optional[Pose] = None,
            log: bool = False
    ) -> float:
        if start_pose is None:
            start_pose = self.move_group.get_current_pose().pose

        distance = np.linalg.norm([
            target_pose.position.x - start_pose.position.x,
            target_pose.position.y - start_pose.position.y,
            target_pose.position.z - start_pose.position.z,
        ])
        if log:
            self.logger.info(f'want to travel [{distance * 100:.1f} cm]')
            # self.logger.info(f'... with resolution_m=[{resolution_m:.5f}] and jump_threshold=[{jump_threshold:.3f}]')
            self.logger.info(f'... from start_pose  = {pose_to_list(start_pose)}')
            self.logger.info(f'... to   target_pose = {pose_to_list(target_pose)}')
        return distance
