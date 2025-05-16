"""
The main ROS2 node for the husky monitor. This node is responsible for:

- Setting up the pybullet simulation
- Setting up the mocap client
- Updating the simulation state
- Handling user input
"""

from collections import defaultdict
import os
import time, copy
import numpy as np
from scipy.spatial.transform import Rotation as R

from typing import List, Tuple

import rclpy
import rclpy.executors
from rclpy.node import Node

import pybullet as p
import pybullet_planning as pp

import husky_assembly_teleop.husky_world as world
from husky_assembly_teleop.husky_robot import UR5e_HOME_STATE
from husky_assembly_teleop.common import (
    Button, Slider, SliderGroup, Husky, TrackedObject, HuskyObject, AssemblyObject, HUSKY_UR5e_JOINT_NAMES, lerp, load_gripper
)

from husky_assembly_teleop.optitrack.NatNetClient import NatNetClient
import rclpy.task

DEFAULT_GREY = [0.2, 0.2, 0.2, 0.7]
GOAL_BLUE = [0, 0.2, 0.5, 0.7]
TRAJECTORY_GREEN = [0, 0.5, 0.2, 0.7]
TRANSPARENT = [0, 0.0, 0.0, 0.0]

EXISTING_ELEMENT_COLOR = pp.RED
CURRENT_ELEMENT_COLOR = pp.BLUE
DEFAULT_BAR_POS = pp.Point(0.8, 0, 1.3)

CLIENT_IP = '192.168.0.7' # Set to your own IP
MOCAP_IP = '192.168.0.117' # set to the mocap PC's IP, get this from Motive Settings>Streaming pane->Local interface
  
class HuskyMonitor(Node):
    USE_MOCAP = 0
    FAKE_HARDWARE = 1
    CALIBRATION = 0
    BAR_GOAL_MODE = 1
    BAR_HOLDING_ACCURACY_TEST = 0
    DUAL_ARM_ACCURACY_TEST = 1

    GRASP_PARTITION = 8

    def __init__(self):
        super().__init__('husky_monitor')
        self.tick_timer = self.create_timer(0.05, self.update)
        
        # simple async tasks to be executed every tick
        self.tasks = []
        
        self.huskies = []
        self.tracked_objects = []
        self.name_from_mocap_id = {}

        self.static_obstacles = []
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

        self.selected_robot_slider = None
        self.selected_robot_id = 0
        
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

        self.goal_bar_grasp = None
        self.grasp_theta_index = 0
        self.grasp_distance = 0.0 # fixed for now

        self.trajectory_time = 2 if self.CALIBRATION else 5

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

        # ! an inflated bar for goal
        goal_bar_body = pp.create_cylinder((0.1)/2, 1.0, mass=pp.STATIC_MASS)
        far_away_pose = pp.Pose(pp.Point(0,0,100))
        self.goal_element = AssemblyObject(self, 'b_goal', goal_bar_body, far_away_pose, 
                                           pp.unit_pose())
        pp.set_color(self.goal_element.body, GOAL_BLUE)

        self.build_ui()
        self.update_partial_assembly()
        self.update_goal_model_and_color()
        
    def add_tracked_object(self, obstacle: TrackedObject):
        """Registers an object to be tracked by mocap"""
        self.tracked_objects.append(obstacle)
        self.name_from_mocap_id[obstacle.mocap_id] = obstacle.name

    def add_assembly_objects(self, aobject: AssemblyObject):
        self.assembly_objects.append(aobject)

    def add_static_obstacles(self, pb_body):
        self.static_obstacles.append(pb_body)
        
    def add_husky(self, husky: Husky):
        """Registers a husky to connect to ROS and be tracked by mocap"""
        self.huskies.append(husky)
        self.name_from_mocap_id[husky.mocap_id] = husky.name
        
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

    def reset_goal_state(self):
        # TODO somehow breaks trajectory preview
        self.goal_arm_pose = self.huskies[0].interface.arm_joint_pose
        self.reset_ui()

    def append_calibration_data(self, data):
        self.calibration_data.append(data)

    def record_calibration_data(self):
        world.save_calibration(self)
        self.calibration_data = []

    def record_markerset_data(self):
        world.save_markerset_data(self)
        self.marker_set_data = []
        
    def reset_ui(self, target_conf=None):
        # reset all sliders to default value by recreating them...
        # pybullet seems to lack a setUserDebugParameter() method :(
        p.removeAllUserParameters()
        self.buttons.clear()
        self.assembly_position_sliders.clear()
        self.joint_state_sliders.clear()
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
            self.reset_ui()

    def update_trajectory_time(self, time):
        self.trajectory_time = time

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
                    ho.set_pose((hi.position, hi.rotation), [conf])

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
        self.goal_arm_pose[0] = 0.0
        self.reset_ui(self.goal_arm_pose)

    def execute_one_step(self):
        # pop the first element from planned_arm_trajectory
        if self.planned_arm_trajectory[0] is None:
            self.get_logger().warn('Arm trajectory must be planed before executing!')
        else:
            conf = self.planned_arm_trajectory[0].pop(0)
            world.execute_arm_conf(self, conf)
            # self.goal_arm_pose = conf
            # self.show_goal_state = True

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

    def sample_bar_location_for_ik_and_transfer(self):
        # goal_bar_pose = self.get_world_from_bar_goal_pose()
        traj, rand_pos, bar_goal_quat, theta_index, grasp_dist = world.randomize_bar_location_for_ik_and_transfer(self) #, goal_bar_pose[1]

        self.base_from_goal_bar_pos = pp.Point(*rand_pos)
        self.world_from_goal_bar_euler = pp.euler_from_quat(bar_goal_quat)

        self.set_arm_trajectory(traj)
        self.grasp_theta_index = theta_index
        self.grasp_distance = grasp_dist

        self.set_to_show_traj_state()
    
    # --- --- --- --- --- SETUP PYBULLET --- --- --- --- ---
    def start_pybullet(self):
        # start pybullet simulator
        pp.connect(use_gui=True, shadows=True, color=[0.9, 0.9, 1.0])
        # turn on the GUI panels
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1, physicsClientId=pp.CLIENT)
        
        # draw world frame
        pp.draw_pose(pp.unit_pose(), 0.1)

        # load goal robot model
        with pp.LockRenderer():
            with pp.HideOutput():                
                self.goal_model_single = HuskyObject(calibration=self.CALIBRATION)
                self.goal_model_single.set_color(TRANSPARENT)
                self.goal_model_single
                
                self.goal_model_dual = HuskyObject(calibration=self.CALIBRATION, dual_arm=True)
                self.goal_model_dual.set_color(TRANSPARENT)
                
                self.goal_model = self.goal_model_single

                self.goal_gripper_model = load_gripper(self.CALIBRATION)
                pp.set_color(self.goal_gripper_model, GOAL_BLUE)
                
    def update_goal_model_and_color(self):
        if self.goal_model.dual_arm != self.huskies[self.selected_robot_id].dual_arm:
            self.goal_model.set_color(TRANSPARENT)
            self.goal_model = self.goal_model_dual if self.huskies[self.selected_robot_id].dual_arm else self.goal_model_single
        self.goal_model.set_color(GOAL_BLUE if self.show_goal_state else TRAJECTORY_GREEN)
        
    def build_ui(self, target_conf=None):
        # default_base_position = [0,0,0]
        # self.assembly_goal_position_slider_group = SliderGroup(["target base {}".format(t) for t in ["x","y","z"]], self.update_assembly_goal_position, [0, -5, 0], [5,5,1], default_base_position)


        self.buttons.append(Button('Prev in sequence', self.show_previous_in_sequence))
        self.buttons.append(Button('Next in sequence', self.show_next_in_sequence))

        self.selected_robot_slider = Slider("robot id", self.update_selected_robot_id, 0, len(self.huskies)+1, self.selected_robot_id)
        self.arm_slider = Slider("arm id", self.update_selected_arm_id, 0, 2, self.selected_arm_index)

        self.trajectory_time_slider = Slider("traj time", self.update_trajectory_time, 1.0, 60.0, self.trajectory_time)

        self.time_slider = p.addUserDebugParameter("time", 0.0, 1.0, 1.0)
        
        self.buttons.append(Button('Toggle Goal/Trajectory', self.toggle_show_goal_state))
        self.buttons.append(Button('Reset Goal State', self.reset_ui))
        
        if not self.USE_MOCAP:
            # teleop base when no mocap
            pose2d = pp.pose2d_from_pose((self.huskies[self.selected_robot_id].interface.position, self.huskies[self.selected_robot_id].interface.rotation), tolerance=0.1)
            self.teleop_base_slider_group = SliderGroup(["teleop base {}".format(t) for t in ["x","y","yaw"]], self.update_base_conf, [-5.0, -5.0, -np.pi], [5.0,5.0,np.pi], pose2d)
            # self.state_sliders.append(p.addUserDebugParameter("x", -5.0, 5.0, pose2d[0]))
            # self.state_sliders.append(p.addUserDebugParameter("y", -5.0, 5.0, pose2d[1]))
            # self.state_sliders.append(p.addUserDebugParameter("yaw", -np.pi, np.pi, pose2d[2]))

        # self.buttons.append(Button('Plan base', lambda: world.plan_to_goal(self)))
        # self.buttons.append(Button('Exec Base', lambda: world.move_to_goal(self)))
               
        self.buttons.append(Button('Plan arm to assemble current element', self.plan_arm_to_transfer_element))
        self.buttons.append(Button('Plan arm to assemble, reuse grasp', self.plan_arm_to_transfer_element_reuse_grasp))

        self.buttons.append(Button('Plan arm to retract to home', self.plan_arm_to_retract_to_home))

        self.buttons.append(Button('Exec Arm Traj', self.execute_arm_trajectory))
        self.buttons.append(Button('Exec Arm Traj with servoing', self.execute_arm_trajectory_with_servoing))

        # if not self.CALIBRATION:
        #     self.buttons.append(Button('Exec Free Motion', self.execute_free_trajectory))
        #     self.buttons.append(Button('Exec Linear Motion', self.execute_linear_trajectory))

        # self.buttons.append(Button('Plan arm wave', lambda: world.plan_arm_wave(self)))

        if not self.FAKE_HARDWARE:
            self.gripper_slider = p.addUserDebugParameter("gripper", 0, 1.0, 0.1)
            self.buttons.append(Button('Exec Gripper', lambda: world.set_gripper(self)))

            self.buttons.append(Button('Open Gripper', lambda: world.open_gripper_full(self)))
            self.buttons.append(Button('Close Gripper', lambda: world.close_gripper_for_bar(self)))

        self.buttons.append(Button('Compute ik', self.compute_ik_for_bar))
        self.buttons.append(Button('Plan arm to conf target', lambda: world.plan_arm_to_goal(self)))

        self.buttons.append(Button('Rand bar loc for ik', self.sample_bar_location_for_ik_and_transfer))

        # bar_goal_pose_slider_group
        if self.BAR_GOAL_MODE:
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

            if self.BAR_HOLDING_ACCURACY_TEST:
                self.buttons.append(Button('Record markerset data', self.send_request_to_mocap))
                self.buttons.append(Button('Save markerset data', self.record_markerset_data))
            
            if self.DUAL_ARM_ACCURACY_TEST:
                self.buttons.append(Button('Compute Trajectory', lambda: world.next_dual_arm_bar_trajectory(self)))
                self.buttons.append(Button('Exec Arms', lambda: world.execute_arm_trajectory_both(self)))
                self.buttons.append(Button('Exec Arms and Record', lambda: self.tasks.append(world.execute_and_log_mocap(self))))
                self.buttons.append(Button('Record EE mocap pose', lambda: world.record_dual_arm_E_mocap(self)))
                self.buttons.append(Button('Save EE mocap data', lambda: world.save_dual_arm_E_mocap(self)))

        if True:
            # TODO use selected robot id
            for i, j in enumerate(pp.joints_from_names(self.huskies[0].object.robot, self.huskies[0].object.get_arm_joint_names())):
                lower, upper = pp.get_joint_limits(self.huskies[0].object.robot, j)
                if target_conf is None:
                    self.joint_state_sliders.append(p.addUserDebugParameter(f'Joint {i}', lower, upper, self.huskies[0].interface.arm_joint_pose[self.selected_arm_index][i]))
                else:
                    self.joint_state_sliders.append(p.addUserDebugParameter(f'Joint {i}', lower, upper, target_conf[i]))
            
        if self.CALIBRATION:
            self.buttons.append(Button('Calib joint 0', lambda: world.calibrate_joint(self, 0, 'calib_tool')))
            self.buttons.append(Button('Set joint 0 to zero', self.set_goal_joint_0_to_zero))
            self.buttons.append(Button('Calib joint 1', lambda: world.calibrate_joint(self, 1, 'calib_tool')))
            self.buttons.append(Button('Execute one step', self.execute_one_step))

        self.buttons.append(Button('Record current calib conf', lambda: world.calibrate_button(self, 'calib_tool')))
        self.buttons.append(Button('Export calib conf to json', self.record_calibration_data))
        self.buttons.append(Button('Remove all drawing', lambda : pp.remove_all_debug()))

    
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
            # these position and rotation are updated by mocap in a differen thread
            h.object.set_pose((hi.position, hi.rotation), hi.arm_joint_pose)
            # set the goal pose of base since we are teleoperating the base
            self.goal_base_pose = (hi.position, hi.rotation)

        # pp.draw_pose(self.goal_model.get_link_pose_from_name("ur_arm_base_link"))

        self.selected_robot_slider.update()
        self.trajectory_time_slider.update()

        if not self.USE_MOCAP:
            self.teleop_base_slider_group.update()
        
        # update goal robot base state
        # state_slider_values = [p.readUserDebugParameter(ps) for ps in self.state_sliders]
        # self.goal_pose = (
        #     np.array((state_slider_values[0], state_slider_values[1], 0)),
        #     R.from_euler("z", state_slider_values[2], degrees=False).as_quat()
        # )
        if not self.FAKE_HARDWARE:
            self.goal_gripper = p.readUserDebugParameter(self.gripper_slider)

        if self.BAR_GOAL_MODE:
            self.bar_goal_pose_slider_group.update()
            # update_bar_goal_pose
        else:
            self.goal_arm_pose[self.selected_arm_index] = np.array([p.readUserDebugParameter(ps) for ps in self.joint_state_sliders])

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
                    
                    # if arm_traj_idx < len(self.planned_arm_trajectory[i][0]) and len(self.planned_arm_trajectory[i][0]) > 0:
                        #goal_arm_pose[i] = self.planned_arm_trajectory[i][0][arm_traj_idx]

                    # we don't do interpolation here bc I want to see the exact trajectory points
                    dt = arm_traj_idx_float - arm_traj_idx
                    arm_traj_idx_plus = min(int(preview_time * (N - 1) + 1), N-1)
                    goal_arm_pose[i] = lerp(self.planned_arm_trajectory[i][0][arm_traj_idx], self.planned_arm_trajectory[i][0][arm_traj_idx_plus], dt)

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
    
                


# --- --- --- --- --- MAIN --- --- --- --- --- 
def main(args=None):
    rclpy.init(args=args)

    husky_monitor = HuskyMonitor()

    rclpy.spin(husky_monitor)

    husky_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':     
    main()
