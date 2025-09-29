"""
The main ROS2 node for the husky monitor. This node is responsible for:

- Setting up the pybullet simulation
- Setting up the mocap client
- Updating the simulation state
- Handling user input
"""
import sys, re
print(f"Running with Python: {sys.executable}")

from collections import defaultdict
import os
import time, copy
import numpy as np

from typing import List, Tuple
from scipy.spatial.transform import Rotation as R

import rclpy
import rclpy.executors
from rclpy.node import Node

import pybullet as p
import pybullet_planning as pp

from husky_assembly_teleop import DATA_DIRECTORY
import husky_assembly_teleop.husky_world as world
from husky_assembly_teleop.husky_robot import UR5e_HOME_STATE
from husky_assembly_teleop.common import (
    Button, Slider, SliderGroup, Husky, TrackedObject, HuskyObject, AssemblyObject, HUSKY_UR5e_JOINT_NAMES, lerp, load_gripper
)
from husky_assembly_teleop.optitrack.NatNetClient import NatNetClient
from husky_assembly_teleop.utils import pose_from_frame, frame_from_pose, pose_from_transformation, transformation_from_pose

DEFAULT_GREY = [0.2, 0.2, 0.2, 0.7]
GOAL_BLUE = [0, 0.2, 0.5, 0.7]
TRAJECTORY_GREEN = [0, 0.5, 0.2, 0.7]
TRANSPARENT = [0, 0.0, 0.0, 0.0]

EXISTING_ELEMENT_COLOR = pp.RED
CURRENT_ELEMENT_COLOR = pp.BLUE
DEFAULT_BAR_POS = pp.Point(0.8, 0, 1.3)

CLIENT_IP = '192.168.0.7' # Set to your own IP
MOCAP_IP = '192.168.0.117' # set to the mocap PC's IP, get this from Motive Settings>Streaming pane->Local interface

FILENAME_SUFFIX = '_vary_pos_vary_yaw'
# VALIDATION_PROBLEM_NAME = '250905Orientation_test'
VALIDATION_PROBLEM_NAME = '250929_New_Anntenna'
  
class HuskyMonitor(Node):
    USE_MOCAP = 0
    FAKE_HARDWARE = 0

    GRASP_PARTITION = 8
    BAR_GOAL_MODE = 0

    CALIBRATION = 0

    BAR_HOLDING_ACCURACY_TEST = 0
    DUAL_ARM_ACCURACY_TEST = 0

    ASSEMBLY_MODE = 0
    
    BOARD_VALIDATION = 1

    def __init__(self):
        super().__init__('husky_monitor')
        self.tick_timer = self.create_timer(0.05, self.update)
        
        # simple async tasks to be executed every tick
        self.tasks = []
        
        self.huskies = []
        self.tracked_objects = []
        self.name_from_mocap_id = {}

        self.static_obstacles = {}
        self.assembly_objects = []
        self.current_seq_index = 0

        self.calibration_data = []
        self.marker_set_data = []
        self.dual_arm_EE_mocap_data = []
        
        # UI
        self.buttons = []
        self.assembly_position_sliders = []
        self.joint_state_sliders = []
        self.assembly_goal_position_slider_group = None
        self.bar_goal_pose_slider_group = None
        self.bar_grasp_long_distance_slider = None
        self.dump_sep_sliders = []
        self.calib_joint_range_slider = None
        self.calib_target_axis_slider = None

        self.selected_robot_slider = None
        self.selected_robot_id = 0
        
        # Board validation mode variables
        self.board_validation_state_slider = None
        self.trajectory_selection_slider = None
        self.available_robot_cell_states = []
        self.selected_state_index = 0
        self.available_joint_trajectories = []  # Store available JointTrajectory files
        self.selected_trajectory_index = 0
        
        # Cache for RobotCell to avoid reloading
        self._robot_cell_cache = None
        self._robot_cell_cache_path = None
        
        # goal and trajectory interface
        self.selected_arm_index = 0
        self.goal_base_pose = (np.zeros(3), np.array([0, 0, 0, 1]))
        self.goal_gripper = 0.0
        self.goal_arm_pose = [np.zeros(6), np.zeros(6)]
        self.show_goal_state = True

        self.goal_model = None
        self.goal_gripper_model = None

        self.base_from_goal_bar_pos = None
        self.world_from_goal_bar_euler = None
        self.goal_element = None 

        self.calib_tool_from_robot_arm_id = defaultdict(lambda: defaultdict(lambda: None))
        self.calib_joint_range = np.pi/2
        self.calib_target_axis = 0

        self.goal_bar_grasp = None
        self.grasp_theta_index = 0
        self.grasp_distance = 0.0 # fixed for now
        self.goal_element_axis = 0

        self.trajectory_time = 20 if self.CALIBRATION else 60

        # list of conf, velocity, total time, attachment other than the ee
        self.planned_arm_trajectory = [(None, None, None, None), (None, None, None, None)]
        self.free_arm_trajectory = None
        self.linear_arm_trajectory = None

        self.plan_traj_seg = None
        self.planned_base_trajectory = (None, None)

        # call setup code
        self.start_pybullet()
        if self.USE_MOCAP:
            self.start_mocap()
        
        world.init(self)
        
        # Load goal model after robots are created to ensure it matches the actual robot
        self.load_goal_model()

        # ! an inflated bar for goal
        goal_bar_body = pp.create_cylinder((0.025)/2, 1.0, mass=pp.STATIC_MASS)
        far_away_pose = pp.Pose(pp.Point(0,0,100))
        self.goal_element = AssemblyObject(self, 'b_goal', goal_bar_body, far_away_pose, 
                                           pp.unit_pose())
        pp.set_color(self.goal_element.body, GOAL_BLUE)

        # Initialize board validation if enabled
        if self.BOARD_VALIDATION:
            self.available_robot_cell_states = self._load_available_robot_cell_states()
            self.available_joint_trajectories = self._load_available_joint_trajectories()
        
        self.build_ui()
        self.update_partial_assembly()
        self.update_goal_model_and_color()
        
    def add_tracked_object(self, obstacle: TrackedObject):
        """Registers an object to be tracked by mocap"""
        self.tracked_objects.append(obstacle)
        self.name_from_mocap_id[obstacle.mocap_id] = obstacle.name

    def add_assembly_objects(self, aobject: AssemblyObject):
        self.assembly_objects.append(aobject)

    def add_static_obstacles(self, pb_body, name):
        self.static_obstacles[name] = pb_body
        
    def add_husky(self, husky: Husky):
        """Registers a husky to connect to ROS and be tracked by mocap"""
        self.huskies.append(husky)
        self.name_from_mocap_id[husky.mocap_id] = husky.name

    def assign_calibration_tool_to_robot(self, robot_id, arm_id, tool_name):
        """Assigns a calibration tool to a robot's arm"""
        if robot_id < 0 or robot_id >= len(self.huskies):
            raise ValueError(f"Invalid robot_id: {robot_id}")
        self.calib_tool_from_robot_arm_id[robot_id][arm_id] = tool_name

    @property
    def active_calib_tool_name(self):
        """Returns the active calibration tool for the selected robot and arm"""
        return self.calib_tool_from_robot_arm_id[self.selected_robot_id][self.selected_arm_index]
        
    def set_base_trajectry(self, base_trajectory: Tuple[List[Tuple[np.ndarray, np.ndarray]], float]):
            """ set base trajectory for visualization"""
            self.planned_base_trajectory = base_trajectory
            
            # draw
            points = [
                pos for pos, _ in self.planned_base_trajectory[0]
            ]
            with pp.LockRenderer():
                with pp.HideOutput():
                    if self.plan_traj_seg is not None:
                       pp.remove_all_debug()
                    self.plan_traj_seg = pp.add_segments(points)
    
    def set_arm_trajectory(self, arm_trajectory, index=0):
        """ set arm trajectory for visualization"""
        # Tuple[List[np.ndarray], List[np.ndarray] | None, float], AssemblyObject
        # list of confs, list of velocities, total time, grasped element
        self.planned_arm_trajectory[index] = arm_trajectory

    def _reset_planned_arm_trajectory(self):
        # reset the planned arm trajectory to None
        self.planned_arm_trajectory = [(None, None, None, None), (None, None, None, None)]
        self.free_arm_trajectory = None
        self.linear_arm_trajectory = None

    def append_calibration_data(self, data):
        self.calibration_data.append(data)

    def record_calibration_data(self):
        world.save_calibration(self)
        self.calibration_data = []

    def record_markerset_data(self):
        world.save_markerset_data(self, filename_suffix=FILENAME_SUFFIX)
        self.marker_set_data = []
        
    def reset_ui(self, target_conf=None):
        # reset all sliders to default value by recreating them...
        # pybullet seems to lack a setUserDebugParameter() method :(
        p.removeAllUserParameters()
        self.buttons.clear()
        self.assembly_position_sliders.clear()
        self.joint_state_sliders.clear()
        self.dump_sep_sliders.clear()
        self.build_ui(target_conf)
        
    def toggle_show_goal_state(self):
        self.show_goal_state = not self.show_goal_state
        self.goal_model.set_color(GOAL_BLUE if self.show_goal_state else TRAJECTORY_GREEN)

    def set_to_show_goal_state(self):
        self.show_goal_state = False
        self.toggle_show_goal_state()

    def set_to_show_traj_state(self):
        self.show_goal_state = True
        self.toggle_show_goal_state()

    def update_selected_robot_id(self, robot_id):
        new_id = np.clip(int(robot_id), 0, len(self.huskies)-1)
        if new_id != self.selected_robot_id:
            self.selected_robot_id = new_id
            # update goal pose based on sensed base pose since we are teleoperating the base
            hi = self.huskies[self.selected_robot_id].interface
            self.goal_base_pose = (hi.position, hi.rotation)
            self.update_goal_model_and_color()
            self.reset_ui()
            
    def update_selected_arm_id(self, arm_index):
        new_index = np.clip(int(arm_index), 0, 1)
        if new_index != self.selected_arm_index:
            self.selected_arm_index = new_index
            self.reset_ui(target_conf=self.goal_arm_pose) #[self.selected_arm_index])

    def update_trajectory_time(self, time):
        self.trajectory_time = time

    def update_calib_joint_range(self, value):
        self.calib_joint_range = value

    def update_calib_target_axis(self, value):
        self.calib_target_axis = int(np.floor(value))

    def update_goal_align_axis(self, value):
        self.goal_element_axis = value

    def show_previous_in_sequence(self):
        if self.current_seq_index >= 1:
            self.current_seq_index -= 1
            self.update_partial_assembly()

    def show_next_in_sequence(self):
        if self.current_seq_index < len(self.assembly_objects) - 1:
            self.current_seq_index += 1
            self.update_partial_assembly()

    def update_partial_assembly(self):
        for i, obj in enumerate(self.assembly_objects):
            if i <= self.current_seq_index:
                obj.show()
                pp.set_color(obj.body, EXISTING_ELEMENT_COLOR)
            else:
                obj.hide()
        pp.set_color(self.assembly_objects[self.current_seq_index].body, CURRENT_ELEMENT_COLOR)

        # if the partial assembly changes, the previously planned arm trajectory is invalidated
        self._reset_planned_arm_trajectory()

    def update_assembly_goal_position(self, centroid):
        for i, obj in enumerate(self.assembly_objects):
            obj.update_goal_pose((np.array(centroid) + obj.archived_goal_position, obj.goal_pose[1]))
        self.update_partial_assembly()

    def update_base_conf(self, base_conf):
        base_pose = pp.pose_from_base_values(base_conf)
        self.huskies[self.selected_robot_id].interface.position = base_pose[0]
        self.huskies[self.selected_robot_id].interface.rotation = base_pose[1]
        # # since we are teloperating the base, update the base goal pose
        # self.goal_pose = base_pose
        
        # if the base changes, the previously planned arm trajectory is invalidated
        self._reset_planned_arm_trajectory()

    def update_traj_goal_configuration(self):
        self.goal_model.set_pose(self.goal_base_pose, self.goal_arm_pose)

    def plan_arm_to_transfer_element_reuse_grasp(self):
        if self.planned_arm_trajectory[3] is not None:
            obj = self.planned_arm_trajectory[3]
            world.plan_arm_to_transfer_element(self, obj.grasp)
            self.set_to_show_traj_state()
        else:
            print('No grasp saved in the planned trajectory to reuse!')

    def plan_arm_to_transfer_element(self, grasp=None):
        world.plan_arm_to_transfer_element(self)
        self.set_to_show_traj_state()

    def plan_arm_to_retract_to_home(self):
        world.plan_arm_to_retract_to_home(self)
        self.set_to_show_traj_state()

    def execute_linear_trajectory(self):
        # only execute part of the traj returned by transfer planning
        if self.linear_arm_trajectory is None:
            print('Linear arm trajectory is not planned!')
        else:
            self.execute_arm_trajectory(self.linear_arm_trajectory)

    def execute_free_trajectory(self):
        if self.free_arm_trajectory is None:
            print('Free arm trajectory is not planned!')
        else:
            self.execute_arm_trajectory(self.free_arm_trajectory)
    
    def execute_arm_trajectory(self, trajectory=None):
        # TODO merge dual arm execution into this one
        # Make a trajectory class that contains robot index info
        # Since we are already using compas_fab, consider extending their JointTrajectory class
        # https://compas.dev/compas_fab/latest/api/generated/compas_fab.robots.JointTrajectory.html
        if trajectory is None:
            trajectory = self.planned_arm_trajectory[self.selected_arm_index]

        if not self.FAKE_HARDWARE:
            world.execute_arm_trajectory(self, trajectory, index=self.selected_arm_index)
        else:
            # fake execution in sim
            if trajectory is None:
                self.get_logger().warn('Arm trajectory must be planed before executing!')
            else: 
                ho = self.huskies[self.selected_robot_id].object
                hi = self.huskies[self.selected_robot_id].interface
                if trajectory[3] is not None:
                    obj = trajectory[3]
                    gripper_tcp_from_object = obj.grasp

                for conf in trajectory[0]:
                    hi.arm_joint_pose[self.selected_arm_index] = conf
                    ho.set_pose((hi.position, hi.rotation), hi.arm_joint_pose)

                    if trajectory[3] is not None:
                        # update attached object based on FK
                        world_from_tcp = ho.get_link_pose_from_name("ur_arm_tool0")
                        object_pose = pp.multiply(world_from_tcp, gripper_tcp_from_object)
                        obj.set_pose(object_pose)
                    
                    hi.is_arm_executing = True
                    pp.wait_for_duration(0.01)

                hi.is_arm_executing = False

    def execute_arm_trajectory_with_servoing(self, trajectory=None):
        if trajectory is None:
            trajectory = self.planned_arm_trajectory[self.selected_arm_index]

        if self.FAKE_HARDWARE:
            self.logger.warn('Fake hardware does not support servoing!')
        else:
            # TODO make compatiable with dual arm
            world.execute_task_goal_arm_trajectory_with_servoing(self, trajectory, 
                                                                 log_data=0)

    def set_goal_joint_0_to_zero(self):
        self.goal_arm_pose[self.selected_arm_index][0] = 0.0
        self.reset_ui(self.goal_arm_pose)

    def sample_calib_traj(self):
        attachments = [ee[1] for ee in self.huskies[self.selected_robot_id].object.ee_list]
        obstacles = list(self.static_obstacles.values())
        packed_trajs = world.sample_calib_motion(self, int(self.selected_arm_index), int(self.calib_target_axis), self.calib_joint_range, 
                                                 attachments=attachments, obstacles=obstacles)

        if packed_trajs is not None:
            full_traj, transit_traj, calib_traj = packed_trajs
            self.set_arm_trajectory(full_traj, index=self.selected_arm_index)
            self.free_arm_trajectory = transit_traj
            self.linear_arm_trajectory = calib_traj
            self.set_to_show_traj_state()

    def execute_calib_traj(self):
        if self.linear_arm_trajectory is None or self.free_arm_trajectory is None:
            self.get_logger().warn('Transit and calib trajectories must be planned before executing!')
        else:
            # conf = self.planned_arm_trajectory[self.selected_arm_index][0].pop(0)
            # world.execute_arm_conf(self, conf, index=self.selected_arm_index)

            world.execute_arm_trajectory_and_record_each_conf(self, self.free_arm_trajectory, self.linear_arm_trajectory, index=self.selected_arm_index)

    def get_world_from_bar_goal_pose(self):
        world_from_base_link = self.goal_model.get_link_pose_from_name("base_footprint")
        world_pos = pp.multiply(world_from_base_link, pp.Pose(point=self.base_from_goal_bar_pos))[0]
        world_quat = pp.Pose(euler=pp.Euler(*self.world_from_goal_bar_euler))[1]
        return world_pos, world_quat
    
    def update_bar_goal_pose(self, slider_inputs):
        # ! keep bar pos relative to the robot base, but orientation absolute to the world
        # print('tiggered')

        self.base_from_goal_bar_pos = pp.Point(*slider_inputs[:3])
        self.world_from_goal_bar_euler = pp.Euler(*slider_inputs[3:])

        # self.world_from_goal_bar_euler = pp.Euler(*slider_inputs)

        # world_from_bar = pp.Pose(point=pp.Point(0.8, 0, 1.4), euler=pp.Euler(roll=np.pi/2))
        goal_bar_pose = self.get_world_from_bar_goal_pose()
        self.goal_element.set_pose(goal_bar_pose)
        world.update_goal_gripper_model_pose(self, goal_bar_pose, self.grasp_theta_index, self.grasp_distance)

        # arm_conf, grasp = world.compute_ik_for_bar(self, goal_bar_pose, self.grasp_theta_index, self.grasp_distance)
        # print('arm_conf:', arm_conf)
        # print('grasp:', grasp)
        # if arm_conf is not None and grasp is not None:
        #     self.goal_arm_pose = arm_conf
        # self.goal_bar_grasp = grasp

    def next_grasp_theta(self):
        self.set_to_show_goal_state()

        self.grasp_theta_index = (self.grasp_theta_index + 1) % self.GRASP_PARTITION
        goal_bar_pose = self.get_world_from_bar_goal_pose()
        world.update_goal_gripper_model_pose(self, goal_bar_pose, self.grasp_theta_index, self.grasp_distance)

        # arm_conf, grasp = world.compute_ik_for_bar(self, goal_bar_pose, self.grasp_theta_index, self.grasp_distance)
        # print('arm_conf:', arm_conf)
        # print('grasp:', grasp)
        # if arm_conf is not None and grasp is not None:
        #     self.goal_arm_pose = arm_conf
        #     self.goal_bar_grasp = grasp

    def update_grasp_dist(self, value):
        self.set_to_show_goal_state()

        goal_bar_pose = self.get_world_from_bar_goal_pose()
        world.update_goal_gripper_model_pose(self, goal_bar_pose, self.grasp_theta_index, self.grasp_distance)

        # arm_conf, grasp = world.compute_ik_for_bar(self, goal_bar_pose, self.grasp_theta_index, self.grasp_distance)
        # print('arm_conf:', arm_conf)
        # print('grasp:', grasp)
        # if arm_conf is not None and grasp is not None:
        #     self.goal_arm_pose = arm_conf
        #     self.goal_bar_grasp = grasp

    def rotate_bar_euler_angle(self, angle, axis='roll'):
        self.set_to_show_goal_state()

        goal_bar_pose = pp.multiply(self.get_world_from_bar_goal_pose(), pp.Pose(euler=pp.Euler(**{axis: angle})))
        self.world_from_goal_bar_euler = pp.euler_from_quat(goal_bar_pose[1])
        self.goal_element.set_pose(goal_bar_pose)
        world.update_goal_gripper_model_pose(self, goal_bar_pose, self.grasp_theta_index, self.grasp_distance)
        self.reset_ui()

    def compute_ik_for_bar(self):
        arm_conf, grasp = world.compute_ik_for_bar(self, self.get_world_from_bar_goal_pose(), self.grasp_theta_index, self.grasp_distance)
        if arm_conf is not None and grasp is not None:
            self.goal_arm_pose = arm_conf
            self.goal_bar_grasp = grasp
            self.reset_ui(self.goal_arm_pose)

    def sample_bar_location_for_ik_and_transfer(self, bar_goal_axis=None, target_grasp_index=None):
        # goal_bar_pose = self.get_world_from_bar_goal_pose()
        traj, rand_pos, bar_goal_quat, theta_index, grasp_dist = world.randomize_bar_location_for_ik_and_transfer(self, bar_goal_axis, target_grasp_index) #, goal_bar_pose[1]
        if traj is None:
            return

        self.base_from_goal_bar_pos = pp.Point(*rand_pos)
        self.world_from_goal_bar_euler = pp.euler_from_quat(bar_goal_quat)

        self.set_arm_trajectory(traj)
        self.grasp_theta_index = theta_index
        self.grasp_distance = grasp_dist

        self.set_to_show_traj_state()
    
    def sample_dual_arm_configuration(self):
        """
        Sample a dual-arm configuration and set the trajectories.
        This method calls the world.sample_dual_arm_configuration function.
        """
        # Compute tool0_to_tool0 transform from the JSON file
        json_filepath = os.path.join(
            DATA_DIRECTORY,
            'husky_assembly_design_study',
            '250714_robot_centric_IK_grasp_test',
            'RobotCellStates',
            'robotx_box_A0-IK_test_GraspTargets.json'
        )
        self.get_logger().info(f"Loading tool0_to_tool0 transform from JSON: {json_filepath}")

        # try:
        tool0_to_tool0_transform, tool0_2_from_bar = world.compute_tool0_to_tool0_transform_from_json(json_filepath)

        husky = self.huskies[self.selected_robot_id]
        robot = husky.object.robot
        bar_attachment_right = pp.Attachment(robot, pp.link_from_name(robot, 'right_ur_arm_tool0'), tool0_2_from_bar, self.goal_element.body)
        # except Exception as e:
        #     print(f"Failed to load tool0_to_tool0 transform from JSON: {e}")
        #     # Fallback to default transform
        #     tool0_to_tool0_transform = pp.Pose(
        #         point=pp.Point(0.5, 0, 0),  # 0.5m offset in x direction
        #         euler=pp.Euler(0, 0, 0)      # No rotation
        #     )
        
        # Call the world function to sample configuration
        attachments = [ee[1] for ee in self.huskies[self.selected_robot_id].object.ee_list] + [bar_attachment_right]
        with pp.WorldSaver():
            result = world.sample_dual_arm_configuration(
                self, 
                tool0_to_tool0_transform,
                max_attempts=100,
                ik_attempts=10,
                attachments=attachments
            )
        
        if result is not None:
            left_trajectory, right_trajectory = result
            
            # Set the trajectories for both arms
            self.set_arm_trajectory(left_trajectory, index=0)
            self.set_arm_trajectory(right_trajectory, index=1)
            
            # Show trajectory state
            self.set_to_show_traj_state()
            
            print("Successfully sampled dual-arm configuration!")
        else:
            print("Failed to sample valid dual-arm configuration.")
    
    def load_board_validation_state(self):
        """
        Load a robot cell state for board validation and update the goal robot configuration.
        """
        if not self.available_robot_cell_states:
            print("No robot cell states available!")
            return
            
        if self.selected_state_index >= len(self.available_robot_cell_states):
            print(f"Invalid state index: {self.selected_state_index}")
            return
            
        selected_state_file = self.available_robot_cell_states[self.selected_state_index]
        state_filepath = os.path.join(
            DATA_DIRECTORY,
            'husky_assembly_design_study',
            VALIDATION_PROBLEM_NAME,
            'RobotCellStates',
            selected_state_file
        )
        
        print(f"Loading robot cell state: {selected_state_file}")
        
        # # Check if there's a corresponding JointTrajectory file with the same prefix
        # state_prefix = selected_state_file.replace('_RobotCellState.json', '')
        # corresponding_trajectory_file = f"{state_prefix}_JointTrajectory.json"
        # trajectory_filepath = os.path.join(
        #     DATA_DIRECTORY,
        #     'husky_assembly_design_study',
        #     VALIDATION_PROBLEM_NAME,
        #     'RobotCellStates',
        #     corresponding_trajectory_file
        # )
        
        # if os.path.exists(trajectory_filepath):
        #     print(f"Found corresponding joint trajectory: {corresponding_trajectory_file}")
        #     # Update the available joint trajectories list to include this file if not already present
        #     if corresponding_trajectory_file not in self.available_joint_trajectories:
        #         self.available_joint_trajectories.append(corresponding_trajectory_file)
        #         self.available_joint_trajectories.sort()
        # else:
        #     print(f"No corresponding joint trajectory found for: {selected_state_file}")
        
        try:
            # Load the robot cell state
            from compas.data import json_load
            robot_cell_state = json_load(state_filepath)

            match = re.search(r'_A(\d+)-', selected_state_file)
            active_bar_name = f"b{match.group(1)}_0" if match else None
            self.get_logger().info(f"Active bar name: {active_bar_name}")
            
            # Load rigid body states as static obstacles
            self.load_rigid_body_states_as_obstacles(robot_cell_state, active_bar_name)
            
            # Get the robot configuration from the state
            if hasattr(robot_cell_state, 'robot_configuration'):
                robot_config = robot_cell_state.robot_configuration
                
                # Extract base pose and arm joint states
                if hasattr(robot_config, 'values') and hasattr(robot_config, 'joint_names'):
                    # Find base and arm joint values
                    # base_joint_names = ['base_joint_x', 'base_joint_y', 'base_joint_yaw']
                    from husky_assembly_teleop.utils import HUSKY_DUAL_UR5e_JOINT_NAMES
                    left_arm_names = HUSKY_DUAL_UR5e_JOINT_NAMES[0]
                    right_arm_names = HUSKY_DUAL_UR5e_JOINT_NAMES[1]
                  
                    # Extract arm joint states
                    left_arm_joint_values = [robot_config[name] for name in left_arm_names]
                    right_arm_joint_values = [robot_config[name] for name in right_arm_names]

                    # Update goal robot configuration
                    self.goal_arm_pose[0] = np.array(left_arm_joint_values)
                    self.goal_arm_pose[1] = np.array(right_arm_joint_values)
                    
                    # Update the UI to reflect the new configuration
                    self.reset_ui(self.goal_arm_pose)
                    
                    print(f"Updated goal robot configuration from {selected_state_file}")
                    print(f"Left arm joints: {self.goal_arm_pose[0]}")
                    print(f"Right arm joints: {self.goal_arm_pose[1]}")

                    self.set_to_show_goal_state()
                else:
                    print("Robot configuration does not have expected structure")
            else:
                print("Robot cell state does not contain robot configuration")

            if hasattr(robot_cell_state, 'robot_base_frame'):
                self.goal_base_pose = pose_from_frame(robot_cell_state.robot_base_frame)
                print(f"Updated goal base pose from {selected_state_file}: {self.goal_base_pose}")
 
        except Exception as e:
            print(f"Error loading robot cell state: {e}")

    def _load_robot_cell(self, validation_problem_name):
        """
        Load and cache the RobotCell.json file for the given validation problem.
        
        Parameters
        ----------
        validation_problem_name : str
            The name of the validation problem directory.
            
        Returns
        -------
        RobotCell
            The loaded robot cell.
        """
        robot_cell_path = os.path.join(
            DATA_DIRECTORY,
            'husky_assembly_design_study',
            validation_problem_name,
            'RobotCell.json'
        )
        
        # Check if we already have this robot cell cached
        if self._robot_cell_cache is not None and self._robot_cell_cache_path == robot_cell_path:
            return self._robot_cell_cache
            
        if not os.path.exists(robot_cell_path):
            print(f"RobotCell.json not found at: {robot_cell_path}")
            return None
            
        try:
            from compas.data import json_load
            robot_cell = json_load(robot_cell_path)
            
            # Cache the robot cell
            self._robot_cell_cache = robot_cell
            self._robot_cell_cache_path = robot_cell_path
            
            print(f"Loaded and cached RobotCell from: {robot_cell_path}")
            return robot_cell
            
        except Exception as e:
            print(f"Error loading RobotCell: {e}")
            return None

    def load_rigid_body_states_as_obstacles(self, robot_cell_state, active_bar_name):
        """
        Load rigid body states from a RobotCellState and create/update static obstacles.
        
        Parameters
        ----------
        robot_cell_state : RobotCellState
            The robot cell state containing rigid body states to load as obstacles.
        """
        if not hasattr(robot_cell_state, 'rigid_body_states'):
            print("No rigid body states found in robot cell state")
            return
            
        # Load the RobotCell to get rigid body models
        robot_cell = self._load_robot_cell(VALIDATION_PROBLEM_NAME)
        if robot_cell is None:
            print("Could not load RobotCell, falling back to simple box obstacles")
            self._load_rigid_body_states_as_simple_obstacles(robot_cell_state)
            return
            
        # Dictionary to track existing obstacles by name
        existing_obstacles = {}
        for obstacle in self.static_obstacles:
            if hasattr(obstacle, 'name'):
                existing_obstacles[obstacle.name] = obstacle
        
        # Process each rigid body state
        for rigid_body_name, rigid_body_state in robot_cell_state.rigid_body_states.items():
            if active_bar_name and rigid_body_name != active_bar_name:
                continue

            # Skip hidden rigid bodies
            if rigid_body_state.is_hidden:
                continue
                
            # Skip rigid bodies that are attached to tools or links (they move with the robot)
            if rigid_body_state.attached_to_tool or rigid_body_state.attached_to_link:
                continue
                
            # Get the frame from the rigid body state
            if rigid_body_state.frame is None:
                print(f"Warning: No frame data for rigid body {rigid_body_name}")
                continue
                
            # Convert frame to pose (position and quaternion)
            pose = pose_from_frame(rigid_body_state.frame)
 
            # Check if obstacle already exists
            if rigid_body_name in existing_obstacles:
                # Update existing obstacle pose
                obstacle = existing_obstacles[rigid_body_name]
                if hasattr(obstacle, 'body') and obstacle.body is not None:
                    pp.set_pose(obstacle.body, pose)
                    print(f"Updated obstacle {rigid_body_name} pose")
            else:
                # Create new obstacle using real collision geometry from RobotCell
                obstacle_body = self._create_rigid_body_obstacle(rigid_body_name, robot_cell, pose)
                if obstacle_body is not None:
                    # Create a simple wrapper to store the obstacle with name
                    # obstacle = StaticObstacle(rigid_body_name, obstacle_body)
                    self.add_static_obstacles(obstacle_body, rigid_body_name)
                    print(f"Created new obstacle {rigid_body_name} with real collision geometry")
                else:
                    print(f"Failed to create obstacle {rigid_body_name}, falling back to simple box")
                    # Fallback to simple box
                    obstacle_body = pp.create_box(0.1, 0.1, 0.1, color=pp.GREY, mass=pp.STATIC_MASS)
                    pp.set_pose(obstacle_body, pose)
                   
                    self.add_static_obstacles(obstacle_body, rigid_body_name)
                    print(f"Created fallback box obstacle {rigid_body_name}")

    def _create_rigid_body_obstacle(self, rigid_body_name, robot_cell, pose):
        """
        Create a PyBullet obstacle from a rigid body model using real collision geometry.
        
        Parameters
        ----------
        rigid_body_name : str
            The name of the rigid body.
        robot_cell : RobotCell
            The robot cell containing rigid body models.
        pose : tuple
            The pose (position, quaternion) for the obstacle.
            
        Returns
        -------
        int or None
            The PyBullet body ID, or None if creation failed.
        """
        if rigid_body_name not in robot_cell.rigid_body_models:
            print(f"Rigid body model {rigid_body_name} not found in RobotCell")
            return None
            
        rigid_body_model = robot_cell.rigid_body_models[rigid_body_name]
        
        try:
            # Create temporary directory for mesh files (similar to PyBullet client)
            import tempfile
            temp_dir = tempfile.mkdtemp()
            
            # Process visual meshes
            visual_path = None
            if rigid_body_model.visual_meshes and len(rigid_body_model.visual_meshes) > 0:
                from compas.datastructures import Mesh
                visual_mesh = Mesh()
                for m in rigid_body_model.visual_meshes_in_meters:
                    visual_mesh.join(m, precision=12)
                visual_path = os.path.join(temp_dir, f"{rigid_body_name}_visual.obj")
                visual_mesh.to_obj(visual_path)
            
            # Process collision meshes
            collision_path = None
            if rigid_body_model.collision_meshes and len(rigid_body_model.collision_meshes) > 0:
                from compas.datastructures import Mesh
                collision_mesh = Mesh()
                for m in rigid_body_model.collision_meshes_in_meters:
                    collision_mesh.join(m, precision=12)
                collision_path = os.path.join(temp_dir, f"{rigid_body_name}_collision.obj")
                collision_mesh.to_obj(collision_path)
            
            # Create PyBullet body from mesh files
            obstacle_body = pp.create_obj(visual_path or collision_path, mass=pp.STATIC_MASS)
            
            if obstacle_body is not None:
                # Set the pose
                pp.set_pose(obstacle_body, pose)
                # Set color
                pp.set_color(obstacle_body, pp.GREY)
                
            # Clean up temporary directory
            import shutil
            shutil.rmtree(temp_dir, ignore_errors=True)
            
            return obstacle_body
            
        except Exception as e:
            print(f"Error creating rigid body obstacle {rigid_body_name}: {e}")
            return None

    def _load_rigid_body_states_as_simple_obstacles(self, robot_cell_state):
        """
        Fallback method to create simple box obstacles when RobotCell is not available.
        
        Parameters
        ----------
        robot_cell_state : RobotCellState
            The robot cell state containing rigid body states to load as obstacles.
        """
        # Dictionary to track existing obstacles by name
        existing_obstacles = {}
        for obstacle in self.static_obstacles:
            if hasattr(obstacle, 'name'):
                existing_obstacles[obstacle.name] = obstacle
        
        # Process each rigid body state
        for rigid_body_name, rigid_body_state in robot_cell_state.rigid_body_states.items():
            # Skip hidden rigid bodies
            if rigid_body_state.is_hidden:
                continue
                
            # Skip rigid bodies that are attached to tools or links (they move with the robot)
            if rigid_body_state.attached_to_tool or rigid_body_state.attached_to_link:
                continue
                
            # Get the frame from the rigid body state
            if rigid_body_state.frame is None:
                print(f"Warning: No frame data for rigid body {rigid_body_name}")
                continue
                
            # Convert frame to pose (position and quaternion)
            pose = pose_from_frame(rigid_body_state.frame)
           
            # Check if obstacle already exists
            if rigid_body_name in existing_obstacles:
                # Update existing obstacle pose
                obstacle = existing_obstacles[rigid_body_name]
                if hasattr(obstacle, 'body') and obstacle.body is not None:
                    pp.set_pose(obstacle.body, pose)
                    print(f"Updated obstacle {rigid_body_name} pose")
            else:
                # Create simple box obstacle as fallback
                obstacle_body = pp.create_box(0.1, 0.1, 0.1, color=pp.GREY, mass=pp.STATIC_MASS)
                pp.set_pose(obstacle_body, pose)
                 
                # obstacle = StaticObstacle(rigid_body_name, obstacle_body)
                self.add_static_obstacles(obstacle_body, rigid_body_name)
                print(f"Created fallback box obstacle {rigid_body_name}")

    def load_joint_trajectory(self):
        """
        Load a JointTrajectory file and convert it to planned_arm_trajectory format.
        """
        if not self.available_joint_trajectories:
            print("No joint trajectory files available!")
            return
            
        if self.selected_trajectory_index >= len(self.available_joint_trajectories):
            print(f"Invalid trajectory index: {self.selected_trajectory_index}")
            return
            
        selected_trajectory_file = self.available_joint_trajectories[self.selected_trajectory_index]
        trajectory_filepath = os.path.join(
            DATA_DIRECTORY,
            'husky_assembly_design_study',
            VALIDATION_PROBLEM_NAME,
            'RobotCellStates',
            selected_trajectory_file
        )
        
        print(f"Loading joint trajectory: {selected_trajectory_file}")
        
        try:
            # Load the joint trajectory using standard json
            import json
            with open(trajectory_filepath, 'r') as f:
                joint_trajectory_data = json.load(f)
            
            # Extract trajectory data
            if 'data' in joint_trajectory_data and 'points' in joint_trajectory_data['data']:
                points = joint_trajectory_data['data']['points']
                
                # Get joint names from the trajectory
                if points and 'joint_names' in points[0]:
                    joint_names = points[0]['joint_names']
                    
                    # Find indices for left and right arm joints
                    from husky_assembly_teleop.utils import HUSKY_DUAL_UR5e_JOINT_NAMES
                    left_arm_names = HUSKY_DUAL_UR5e_JOINT_NAMES[0]
                    right_arm_names = HUSKY_DUAL_UR5e_JOINT_NAMES[1]
                    
                    # Find indices for each arm's joints
                    left_arm_indices = [joint_names.index(name) for name in left_arm_names if name in joint_names]
                    right_arm_indices = [joint_names.index(name) for name in right_arm_names if name in joint_names]
                    
                    if len(left_arm_indices) != 6 or len(right_arm_indices) != 6:
                        print(f"Warning: Expected 6 joints per arm, got {len(left_arm_indices)} left, {len(right_arm_indices)} right")
                    
                    # Extract joint values for each arm
                    left_arm_trajectory = []
                    right_arm_trajectory = []
                    
                    for point in points:
                        if 'joint_values' in point:
                            left_joint_values = [point['joint_values'][i] for i in left_arm_indices]
                            right_joint_values = [point['joint_values'][i] for i in right_arm_indices]
                            left_arm_trajectory.append(np.array(left_joint_values))
                            right_arm_trajectory.append(np.array(right_joint_values))
                    
                    # Convert to planned_arm_trajectory format: (configurations, velocities, time, grasped_element)
                    # For now, we assume no grasped element (None) and no velocity information
                    left_trajectory_tuple = (left_arm_trajectory, None, self.trajectory_time, None)
                    right_trajectory_tuple = (right_arm_trajectory, None, self.trajectory_time, None)
                    
                    # Set the trajectories
                    self.set_arm_trajectory(left_trajectory_tuple, index=0)
                    self.set_arm_trajectory(right_trajectory_tuple, index=1)
                    
                    # Show trajectory state
                    self.set_to_show_traj_state()
                    
                    print(f"Successfully loaded joint trajectory from {selected_trajectory_file}")
                    print(f"Left arm trajectory: {len(left_arm_trajectory)} points")
                    print(f"Right arm trajectory: {len(right_arm_trajectory)} points")
                else:
                    print("Joint trajectory does not have expected joint_names structure")
            else:
                print("Joint trajectory does not have expected data structure")
                
        except Exception as e:
            print(f"Error loading joint trajectory: {e}")

    def update_board_validation_state_index(self, state_index):
        """
        Update the selected robot cell state index.
        """
        new_index = int(state_index)
        if 0 <= new_index < len(self.available_robot_cell_states):
            self.selected_state_index = new_index
            print(f"Selected state: {self.available_robot_cell_states[self.selected_state_index]}")

    def update_trajectory_index(self, trajectory_index):
        """
        Update the selected joint trajectory index.
        """
        new_index = int(trajectory_index)
        if 0 <= new_index < len(self.available_joint_trajectories):
            self.selected_trajectory_index = new_index
            print(f"Selected trajectory: {self.available_joint_trajectories[self.selected_trajectory_index]}")

    def _load_available_robot_cell_states(self):
        """
        Load available robot cell state files from the hardcoded directory.
        """
        state_dir = os.path.join(
            DATA_DIRECTORY,
            'husky_assembly_design_study',
            VALIDATION_PROBLEM_NAME,
            'RobotCellStates'
        )
        
        if not os.path.exists(state_dir):
            print(f"Robot cell states directory does not exist: {state_dir}")
            return []
        
        # Find all JSON files ending with _RobotCellState.json
        state_files = []
        for filename in os.listdir(state_dir):
            if filename.endswith('_RobotCellState.json'):
                state_files.append(filename)
        
        # Sort files for consistent ordering
        state_files.sort()
        
        print(f"Found {len(state_files)} robot cell state files:")
        for i, filename in enumerate(state_files):
            print(f"  {i}: {filename}")
        
        return state_files
    
    def _load_available_joint_trajectories(self):
        """
        Load available JointTrajectory files from the hardcoded directory.
        """
        state_dir = os.path.join(
            DATA_DIRECTORY,
            'husky_assembly_design_study',
            VALIDATION_PROBLEM_NAME,
            'RobotCellStates'
        )
        
        if not os.path.exists(state_dir):
            print(f"Robot cell states directory does not exist: {state_dir}")
            return []
        
        # Find all JSON files ending with _JointTrajectory.json
        trajectory_files = []
        for filename in os.listdir(state_dir):
            if filename.endswith('_JointTrajectory.json'):
                trajectory_files.append(filename)
        
        # Sort files for consistent ordering
        trajectory_files.sort()
        
        print(f"Found {len(trajectory_files)} joint trajectory files:")
        for i, filename in enumerate(trajectory_files):
            print(f"  {i}: {filename}")
        
        return trajectory_files
    
    # --- --- --- --- --- SETUP PYBULLET --- --- --- --- ---
    def start_pybullet(self):
        # start pybullet simulator
        pp.connect(use_gui=True, shadows=True, color=[0.9, 0.9, 1.0])
        # turn on the GUI panels
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1, physicsClientId=pp.CLIENT)
        
        # draw world frame
        pp.draw_pose(pp.unit_pose(), 0.1)
        
    def load_goal_model(self):
        """
        Load goal robot model that mirrors the actual robot loaded in world.init.
        This ensures the goal model has the same configuration as the real robot.
        """
        # Get the first husky robot to determine the configuration
        if not self.huskies:
            self.get_logger().warn('No husky robots loaded yet. Cannot create goal model.')
            return
        
        # Get the configuration from the first robot
        first_husky = self.huskies[0]
        dual_arm = first_husky.dual_arm
        calibration = self.CALIBRATION
        
        # Determine end effector types from the actual robot
        ee_types = []
        if hasattr(first_husky.object, 'ee_list'):
            # Extract end effector types from the actual robot
            for ee, attachment in first_husky.object.ee_list:
                # Try to determine the type based on the end effector properties
                # This is a heuristic approach since we don't store the type directly
                if hasattr(attachment, 'child') and attachment.child is not None:
                    # Check if it's a validation tool by looking at the body properties
                    # For now, we'll use the same logic as in world.init
                    if calibration:
                        ee_types.append("calib_tip")
                    else:
                        # TODO make this a parameter that corresponds to the real robot setup
                        # Default to validation_tool_pair for dual arm, victor_gripper for single arm
                        if dual_arm:
                            # ee_types.append("validation_tool_pair")
                            ee_types.extend(["victor_gripper", "victor_gripper"])
                        else:
                            ee_types.append("victor_gripper")
                    break  # For single arm, we only need one type
        
        # If we couldn't determine the types, use defaults
        if not ee_types:
            if calibration:
                ee_types = ["calib_tip"]
            else:
                if dual_arm:
                    ee_types = ["validation_tool_pair"]
                else:
                    ee_types = ["victor_gripper"]
        
        # Load only the goal model that matches the actual robot configuration
        with pp.LockRenderer():
            with pp.HideOutput():
                if dual_arm:
                    # Load dual arm goal model
                    self.goal_model = HuskyObject(
                        calibration=calibration, 
                        dual_arm=True, 
                        ee_types=ee_types,  # Use all types for dual arm
                        force_regenerate=False
                    )
                    self.goal_model_single = None  # Not needed for dual arm
                    self.goal_model_dual = self.goal_model
                else:
                    # Load single arm goal model
                    self.goal_model = HuskyObject(
                        calibration=calibration, 
                        dual_arm=False, 
                        ee_types=ee_types[:1] if ee_types else None,  # Take first type for single arm
                        force_regenerate=False
                    )
                    self.goal_model_single = self.goal_model
                    self.goal_model_dual = None  # Not needed for single arm
                
                self.goal_model.set_color(TRANSPARENT)

                # Load goal gripper model
                self.goal_gripper_model = load_gripper(calibration)
                pp.set_color(self.goal_gripper_model, GOAL_BLUE)

    def update_goal_model_and_color(self):
        # Since we now load only the goal model that matches the actual robot,
        # we don't need to switch between single and dual arm models
        # Just update the color based on the current state
        self.goal_model.set_color(GOAL_BLUE if self.show_goal_state else TRAJECTORY_GREEN)
        
    def build_ui(self, target_conf=None):
        self.selected_robot_slider = Slider("robot id", self.update_selected_robot_id, 0, len(self.huskies)+1, self.selected_robot_id)
        self.arm_slider = Slider("arm id (0:L,1:R)", self.update_selected_arm_id, 0, 2, self.selected_arm_index)

        self.trajectory_time_slider = Slider("traj time", self.update_trajectory_time, 1.0, 60.0, self.trajectory_time)

        self.time_slider = p.addUserDebugParameter("Traj viz time", 0.0, 1.0, 1.0)
        
        self.buttons.append(Button('Toggle Goal/Trajectory', self.toggle_show_goal_state))
        self.buttons.append(Button('Reset Goal State', self.reset_ui))
        
        if not self.USE_MOCAP:
            # teleop base when no mocap
            # self.dump_sep_sliders.append(Slider("----------Base Control", lambda : None))
            # pose2d = pp.pose2d_from_pose((self.huskies[self.selected_robot_id].interface.position, self.huskies[self.selected_robot_id].interface.rotation), tolerance=0.1)
            # self.teleop_base_slider_group = SliderGroup(["teleop base {}".format(t) for t in ["x","y","yaw"]], self.update_base_conf, [-5.0, -5.0, -np.pi], [5.0,5.0,np.pi], pose2d)
            # self.state_sliders.append(p.addUserDebugParameter("x", -5.0, 5.0, pose2d[0]))
            # self.state_sliders.append(p.addUserDebugParameter("y", -5.0, 5.0, pose2d[1]))
            # self.state_sliders.append(p.addUserDebugParameter("yaw", -np.pi, np.pi, pose2d[2]))
            pass
        else:
            pass
            # self.buttons.append(Button('Plan base', lambda: world.plan_to_goal(self)))
            # self.buttons.append(Button('Exec Base', lambda: world.move_to_goal(self)))
               
        if self.ASSEMBLY_MODE:
            self.dump_sep_sliders.append(Slider("----------Assembly Control", lambda : None))
            self.buttons.append(Button('Prev in sequence', self.show_previous_in_sequence))
            self.buttons.append(Button('Next in sequence', self.show_next_in_sequence))
            self.buttons.append(Button('Plan arm to assemble current element', self.plan_arm_to_transfer_element))
            self.buttons.append(Button('Plan arm to assemble, reuse grasp', self.plan_arm_to_transfer_element_reuse_grasp))
            self.buttons.append(Button('Plan arm to retract to home', self.plan_arm_to_retract_to_home))

        # self.buttons.append(Button('Plan S.Arm to conf target', lambda : world.plan_arm_to_goal(self)))
        # self.buttons.append(Button('Exec S.Arm Traj', self.execute_arm_trajectory))
        self.buttons.append(Button('Exec Both Arm Trajs', lambda: world.execute_arm_trajectory_both(self)))

        # Add dual arm configuration sampling button
        # self.buttons.append(Button('Sample Dual Arm Config', self.sample_dual_arm_configuration))

        # Add buttons for planning both arms to goal (sequential and composite)
        # self.buttons.append(Button('Plan Both Arms to Goal (sequential)', lambda: world.plan_both_arms_to_goal(self, use_composite=False)))
        self.buttons.append(Button('Plan Both Arms to Goal (composite)', lambda: world.plan_both_arms_to_goal(self, use_composite=True)))

        # Button to export planned trajectory to JSON
        self.buttons.append(Button(
            'Export Trajectory (JSON)',
            lambda: self.export_planned_trajectory_to_json()
        ))

        if self.BOARD_VALIDATION:
            self.dump_sep_sliders.append(Slider("----------State Loading", lambda : None))
            
            # Load available robot cell states if not already loaded
            if not self.available_robot_cell_states:
                self.available_robot_cell_states = self._load_available_robot_cell_states()
            
            # Create slider for selecting robot cell state
            if self.available_robot_cell_states:
                max_index = len(self.available_robot_cell_states) - 1
                self.board_validation_state_slider = Slider(
                    "Robot Cell State", 
                    self.update_board_validation_state_index, 
                    0, max_index, self.selected_state_index
                )
                
                # Add button to load the selected state
                self.buttons.append(Button('Load Robot Cell State', self.load_board_validation_state))
                
                # Create slider for selecting joint trajectory
                if self.available_joint_trajectories:
                    max_traj_index = len(self.available_joint_trajectories) - 1
                    self.trajectory_selection_slider = Slider(
                        "Joint Trajectory", 
                        self.update_trajectory_index, 
                        0, max_traj_index, self.selected_trajectory_index
                    )
                    
                    # Add button to load joint trajectory
                    self.buttons.append(Button('Load Joint Trajectory', self.load_joint_trajectory))
            else:
                print("No robot cell state files found for board validation")

        if not self.CALIBRATION:
            # in calibration mode, we do not have task space targets so this is disabled
            pass
            # self.buttons.append(Button('Exec S.Arm Traj with servoing', self.execute_arm_trajectory_with_servoing))

        # if not self.CALIBRATION:
        #     self.buttons.append(Button('Exec Free Motion', self.execute_free_trajectory))
        #     self.buttons.append(Button('Exec Linear Motion', self.execute_linear_trajectory))
        # self.buttons.append(Button('Plan arm wave', lambda: world.plan_arm_wave(self)))

        if not self.FAKE_HARDWARE and not self.CALIBRATION:
            self.dump_sep_sliders.append(Slider("----------Gripper", lambda : None))
            # self.gripper_slider = p.addUserDebugParameter("gripper", 0, 1.0, 0.1)
            # self.buttons.append(Button('Exec Gripper', lambda: world.set_gripper(self)))
            # self.buttons.append(Button('Open Gripper', lambda: world.open_gripper_full(self)))
            # self.buttons.append(Button('Close Gripper', lambda: world.close_gripper_for_bar(self)))
            
            # New toggle buttons for gripper and screw control
            self.buttons.append(Button('Toggle Left Gripper', lambda: self.huskies[self.selected_robot_id].interface.toggle_gripper(0)))
            self.buttons.append(Button('Toggle Left Screw', lambda: self.huskies[self.selected_robot_id].interface.toggle_screw(0)))
            
            # Only show right arm buttons if the robot is configured for dual arm
            if self.huskies[self.selected_robot_id].dual_arm:
                self.buttons.append(Button('Toggle Right Gripper', lambda: self.huskies[self.selected_robot_id].interface.toggle_gripper(1)))
                self.buttons.append(Button('Toggle Right Screw', lambda: self.huskies[self.selected_robot_id].interface.toggle_screw(1)))

        # self.buttons.append(Button('Compute ik', self.compute_ik_for_bar))

        if self.BAR_HOLDING_ACCURACY_TEST:
            self.dump_sep_sliders.append(Slider("----------Bar Holding Acc Test", lambda : None))
            self.goal_axis_slider = Slider("bar aligned axis", self.update_goal_align_axis, 0, 2, self.goal_element_axis)
            self.buttons.append(Button('Rand bar loc for ik, fix axis', lambda : self.sample_bar_location_for_ik_and_transfer(int(self.goal_element_axis))))
            self.buttons.append(Button('Record markerset data', self.send_request_to_mocap))
            self.buttons.append(Button('Save markerset data', self.record_markerset_data))

        if self.BAR_GOAL_MODE:
            self.dump_sep_sliders.append(Slider("----------Bar Target Control", lambda : None))
            if self.base_from_goal_bar_pos is None or self.world_from_goal_bar_euler is None:
                bar_target_euler = pp.Euler(roll=np.pi/2)
                pos, quat = pp.Pose(point=DEFAULT_BAR_POS, euler=bar_target_euler)
            else:
                pos, quat = self.base_from_goal_bar_pos, pp.quat_from_euler(self.world_from_goal_bar_euler)

            euler = pp.euler_from_quat(quat)
            self.bar_goal_pose_slider_group = SliderGroup([
                "bar {}".format(t) for t in ["x","y","z", "r", "p", "y"]], 
                self.update_bar_goal_pose, 
                # [-np.pi, -np.pi, -np.pi], 
                # [np.pi,  np.pi,  np.pi], 
                # [euler[0], euler[1], euler[2]]
                [-2, -2, -2, -np.pi, -np.pi, -np.pi], 
                [2,  2,  2, np.pi,  np.pi,  np.pi], 
                [pos[0], pos[1], pos[2], euler[0], euler[1], euler[2]]
                )
            self.update_bar_goal_pose(list(pos) + list(euler))
            # self.update_bar_goal_pose(list(euler))

            self.buttons.append(Button('Step bar r', lambda : self.rotate_bar_euler_angle(np.pi/2, 'roll')))
            self.buttons.append(Button('Step bar p', lambda : self.rotate_bar_euler_angle(np.pi/2, 'pitch')))
            self.buttons.append(Button('Step bar y', lambda : self.rotate_bar_euler_angle(np.pi/2, 'yaw')))

            self.buttons.append(Button('Step grasp theta', self.next_grasp_theta))

            # self.bar_grasp_long_distance_silder = Slider("Grasp dist from mid", self., -0.5, 0.5, 0)
            
        if self.DUAL_ARM_ACCURACY_TEST:
            self.dump_sep_sliders.append(Slider("----------Dual Arm Acc Test", lambda : None))
            self.buttons.append(Button('Compute Trajectory', lambda: world.next_dual_arm_bar_trajectory(self)))
            self.buttons.append(Button('Exec Arms', lambda: world.execute_arm_trajectory_both(self)))
            self.buttons.append(Button('Exec Arms and Record', lambda: self.tasks.append(world.execute_and_log_mocap(self))))
            self.buttons.append(Button('Record EE mocap pose', lambda: world.record_dual_arm_E_mocap(self)))
            self.buttons.append(Button('Save EE mocap data', lambda: world.save_dual_arm_E_mocap(self)))
            
        if not self.BAR_GOAL_MODE:
            pass
            # self.dump_sep_sliders.append(Slider("----------Joint Target (Left Arm)", lambda : None))
            # left_joint_names = self.huskies[self.selected_robot_id].object.get_arm_joint_names(index=0)
            # for i, j in enumerate(pp.joints_from_names(self.huskies[self.selected_robot_id].object.robot, left_joint_names)):
            #     lower, upper = pp.get_joint_limits(self.huskies[self.selected_robot_id].object.robot, j)
            #     if target_conf is None:
            #         self.joint_state_sliders.append(p.addUserDebugParameter(f'Left Joint {i}', lower, upper, self.goal_arm_pose[0][i]))
            #     else:
            #         self.joint_state_sliders.append(p.addUserDebugParameter(f'Left Joint {i}', lower, upper, target_conf[0][i]))
            # self.dump_sep_sliders.append(Slider("----------Joint Target (Right Arm)", lambda : None))
            # right_joint_names = self.huskies[self.selected_robot_id].object.get_arm_joint_names(index=1)
            # for i, j in enumerate(pp.joints_from_names(self.huskies[self.selected_robot_id].object.robot, right_joint_names)):
            #     lower, upper = pp.get_joint_limits(self.huskies[self.selected_robot_id].object.robot, j)
            #     if target_conf is None:
            #         self.joint_state_sliders.append(p.addUserDebugParameter(f'Right Joint {i}', lower, upper, self.goal_arm_pose[1][i]))
            #     else:
            #         self.joint_state_sliders.append(p.addUserDebugParameter(f'Right Joint {i}', lower, upper, target_conf[1][i]))
            
        if self.CALIBRATION:
            self.dump_sep_sliders.append(Slider("----------Calibration", lambda : None))
            self.calib_joint_range_slider = Slider("calib joint range", self.update_calib_joint_range, 0.0, np.pi, np.pi/2)
            self.calib_target_axis_slider = Slider("calib target joint id", self.update_calib_target_axis, 0, 1, 0)
            self.buttons.append(Button('Sample calib path', self.sample_calib_traj))

            self.buttons.append(Button('Execute transit to calib traj', self.execute_free_trajectory))
            self.buttons.append(Button('Execute calib traj', self.execute_calib_traj))

            # self.buttons.append(Button('Set joint 0 to zero', self.set_goal_joint_0_to_zero))
            # self.buttons.append(Button('Calib joint 1', lambda: world.calibrate_joint(self, 1, self.active_calib_tool_name)))

        self.dump_sep_sliders.append(Slider("----------DEBUG utils", lambda : None))
        self.buttons.append(Button('Record current calib conf', lambda: world.calibrate_button(self, self.active_calib_tool_name)))
        self.buttons.append(Button('Export calib conf to json', self.record_calibration_data))
        self.buttons.append(Button('Remove all drawing', lambda : pp.remove_all_debug()))
        # Button to load RobotCellState from file and update arm goal configuration
        # self.buttons.append(Button(
        #     'Load RobotCellState (robotx_box_A15-S13)',
        #     lambda: world.load_robotcellstate_and_update_goal(
        #         self,
        #         os.path.join(
        #             DATA_DIRECTORY,
        #             'robotx_box',
        #             'robotx_box_A15-S13_RobotCellState.json'
        #         )
        #     )
        # ))
  
    # --- --- --- --- --- MOCAP --- --- --- --- --- 
    def start_mocap(self):
        print('Starting mocap!')
        self.mocap_client = NatNetClient()
        self.mocap_client.set_client_address(CLIENT_IP)
        self.mocap_client.set_server_address(MOCAP_IP)
        self.mocap_client.set_use_multicast(False)
        self.mocap_client.print_level = 1

        self.mocap_client.rigid_body_listener = self.receive_rigid_body_frame
        self.mocap_client.new_frame_listener = self.receive_mocap_frame
        if self.BAR_HOLDING_ACCURACY_TEST:
            self.mocap_client.labeled_marker_listener = self.receive_labeled_marker
        
        if self.mocap_client.run():
            start_connect = time.time()
            while not self.mocap_client.connected():
                time.sleep(0.25)
                if time.time() - start_connect > 5:
                    break
            print(f"mocap client connected: {self.mocap_client.connected()}")
        else:
            print('Failed to run mocap client!')

    def send_request_to_mocap(self):
        # self.mocap_client.send_request(self.mocap_client.command_socket, self.mocap_client.NAT_REQUEST_MODELDEF,    "",  (self.mocap_client.server_ip_address, self.mocap_client.command_port) )
        # time.sleep(1)
        world.request_marketset_button(self, 'bar_rig')
    
    # mocap updates are happening in a separate thread
    _mocap_rigidbody_cache = {}
    def receive_rigid_body_frame(self, id, pos, rot):
        if id not in self.name_from_mocap_id:
            return
        
        # y up to z up
        pos = np.array((pos[2], pos[0], pos[1]))
        rot = np.array((rot[2], rot[0], rot[1], rot[3]))       
        
        name = self.name_from_mocap_id[id]
        for h in self.huskies:
            if h.name == name:
                self._mocap_rigidbody_cache[name] = (pos, rot)
                
        for o in self.tracked_objects:
            if o.name == name:
                self._mocap_rigidbody_cache[name] = (pos, rot)
    
    def receive_mocap_frame(self, data):
        ts = data['timestamp']
        for h in self.huskies:
            if h.name not in self._mocap_rigidbody_cache:
                continue
            world_from_mocap = self._mocap_rigidbody_cache[h.name]
            # apply calibrated base transformation here
            # we keep the raw mocap data in _mocap_rigidbody_cache
            calibrated_pose = pp.multiply(world_from_mocap, h.base_mocap_from_base_footprint)
            h.interface.mocap_callback(np.array(calibrated_pose[0]), np.array(calibrated_pose[1]), ts)

        for o in self.tracked_objects:
            if o.name not in self._mocap_rigidbody_cache:
                continue
            (pos, rot) = self._mocap_rigidbody_cache[o.name]
            o.mocap_callback(pos, rot, ts)
        # self._mocap_rigidbody_cache.clear()

    _mocap_labeled_marker_cache = defaultdict(dict)
    def receive_labeled_marker(self, labeled_marker_from_model_id):
        # print('Received labeled marker data:', labeled_marker_from_model_id)
        # name = self.name_from_mocap_id[id]
        # if name not in self._mocap_rigidbody_cache:
        #     self.get_logger().warn(f'Mocap {name} not found in rb cache!')
        #     return
        # rb_pose = self._mocap_rigidbody_cache[name]

        for model_id, marker_datas in labeled_marker_from_model_id.items():
            if model_id not in self.name_from_mocap_id:
                continue

            name = self.name_from_mocap_id[model_id]
            if name not in self._mocap_labeled_marker_cache:
                self._mocap_labeled_marker_cache[name] = {}

            for marker_id, marker_data in marker_datas.items():
                # y up to z up
                pos = [marker_data['pos'][2], marker_data['pos'][0], marker_data['pos'][1]]
                self._mocap_labeled_marker_cache[name][marker_id] = {
                    'pos': pos,
                    'size': marker_data['size'],
                    'error': marker_data['error'],
                }
            # print(f'Received marker set data for {name}:', self._mocap_labeled_marker_cache[name])
     
    # --- --- --- --- --- UPDATE --- --- --- --- --- 
    def update(self):
        for b in self.buttons:
            b.update()
 
        # update tracked objects
        for i, o in enumerate(self.tracked_objects):
            o.set_pose((o.pos, o.rot))
        
        # update robot state
        for i, h in enumerate(self.huskies):
            hi = h.interface
            if self.USE_MOCAP:
                # these position and rotation are updated by mocap in a differen thread
                h.object.set_pose((hi.position, hi.rotation), hi.arm_joint_pose)
                # set the goal pose of base since we are teleoperating the base
                self.goal_base_pose = (hi.position, hi.rotation)
            else:
                h.object.set_pose(self.goal_base_pose, hi.arm_joint_pose)

        # pp.draw_pose(self.goal_model.get_link_pose_from_name("ur_arm_base_link"))

        self.selected_robot_slider.update()
        self.arm_slider.update()
        self.trajectory_time_slider.update()

        if self.CALIBRATION:
            self.calib_joint_range_slider.update()
            self.calib_target_axis_slider.update()

        if self.BAR_HOLDING_ACCURACY_TEST:
            self.goal_axis_slider.update()
            
        if self.BOARD_VALIDATION and self.board_validation_state_slider:
            self.board_validation_state_slider.update()
            
        if self.BOARD_VALIDATION and hasattr(self, 'trajectory_selection_slider') and self.trajectory_selection_slider:
            self.trajectory_selection_slider.update()

        if not self.USE_MOCAP:
            pass
            # self.teleop_base_slider_group.update()
        
        # update goal robot base state
        # state_slider_values = [p.readUserDebugParameter(ps) for ps in self.state_sliders]
        # self.goal_pose = (
        #     np.array((state_slider_values[0], state_slider_values[1], 0)),
        #     R.from_euler("z", state_slider_values[2], degrees=False).as_quat()
        # )
        # if not self.FAKE_HARDWARE:
        #     self.goal_gripper = p.readUserDebugParameter(self.gripper_slider)

        if self.BAR_GOAL_MODE:
            self.bar_goal_pose_slider_group.update()
            # update_bar_goal_pose
        else:
            # Update both arms' goal conf from sliders
            pass
            # n_joints = 6
            # left_slider_vals = [p.readUserDebugParameter(ps) for ps in self.joint_state_sliders[:n_joints]]
            # right_slider_vals = [p.readUserDebugParameter(ps) for ps in self.joint_state_sliders[n_joints:2*n_joints]]
            # self.goal_arm_pose[0] = np.array(left_slider_vals)
            # self.goal_arm_pose[1] = np.array(right_slider_vals)

        # update assembly goal position
        # self.assembly_goal_position_slider_group.update()
            
        preview_time = p.readUserDebugParameter(self.time_slider)
        goal_base_pose = self.goal_base_pose
        goal_arm_pose = self.goal_arm_pose
        if not self.show_goal_state:
            # if self.planned_base_trajectory[0] is not None:
            #     N = len(self.planned_base_trajectory[0])
            #     print('N:', N)
            #     base_traj_idx = int(preview_time * (N - 1))
            #     # TODO sometime the trajectory preview gets cut off halfway
            #     goal_base_pose = self.planned_base_trajectory[0][base_traj_idx]

            for i in range(0,2):
                if self.planned_arm_trajectory[i][0] is not None:
                    N = len(self.planned_arm_trajectory[i][0])
                    arm_traj_idx_float = preview_time * (N - 1)
                    arm_traj_idx = int(arm_traj_idx_float)
                    
                    # jg: i reenabled interpolation to see the whole motion including on sparse trajectories
                    # jg: the prerecorded trajectory had weird joint values in the >pi ranges which would lead to double rotations and self intersections
                    
                    if arm_traj_idx < len(self.planned_arm_trajectory[i][0]) and len(self.planned_arm_trajectory[i][0]) > 0:
                        goal_arm_pose[i] = self.planned_arm_trajectory[i][0][arm_traj_idx]

                    # we don't do interpolation here bc I want to see the exact trajectory points
                    # dt = arm_traj_idx_float - arm_traj_idx
                    # arm_traj_idx_plus = min(int(preview_time * (N - 1) + 1), N-1)
                    # goal_arm_pose[i] = lerp(self.planned_arm_trajectory[i][0][arm_traj_idx], self.planned_arm_trajectory[i][0][arm_traj_idx_plus], dt)

                if self.planned_arm_trajectory[i][3] is not None:
                    # update attached object based on FK
                    obj = self.planned_arm_trajectory[i][3]
                    gripper_tcp_from_object = obj.grasp
                    world_from_tcp = self.goal_model.get_link_pose_from_name("ur_arm_tool0")
                    object_pose = pp.multiply(world_from_tcp, gripper_tcp_from_object)
                    obj.set_pose(object_pose)
 
        # always update goal robot based on current slider values
        self.goal_model.set_pose(goal_base_pose, goal_arm_pose)
                        
        # run tasks
        for t in self.tasks:
            try:
               next(t)
            except StopIteration:
                self.tasks.remove(t)
                
        world.update(self)

    def export_planned_trajectory_to_json(self, filename='planned_trajectory.json', arm_index=None):
        """
        Export the planned arm trajectory to a JSON file as a list of joint configurations.
        Save to the DATA_DIRECTORY/robotx_box subfolder.
        """
        import json
        if arm_index is None:
            arm_index = self.selected_arm_index
        traj = self.planned_arm_trajectory[arm_index][0]
        if traj is None or len(traj) == 0:
            print('No planned trajectory to export!')
            return
        # Convert numpy arrays to lists
        traj_list = [list(map(float, conf)) for conf in traj]
        # Save to DATA_DIRECTORY/robotx_box
        out_dir = os.path.join(DATA_DIRECTORY, 'robotx_box')
        os.makedirs(out_dir, exist_ok=True)
        # Add arm index to the filename before the extension
        base, ext = os.path.splitext(filename)
        filename_with_arm = f"{base}_arm{arm_index}{ext}"
        out_path = os.path.join(out_dir, filename_with_arm)
        with open(out_path, 'w') as f:
            json.dump(traj_list, f, indent=2)
        print(f'Trajectory exported to {out_path}')

# --- --- --- --- --- MAIN --- --- --- --- --- 
def main(args=None):
    rclpy.init(args=args)

    husky_monitor = HuskyMonitor()

    rclpy.spin(husky_monitor)

    husky_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':     
    main()
