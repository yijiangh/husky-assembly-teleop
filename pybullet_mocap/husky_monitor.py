"""
The main ROS2 node for the husky monitor. This node is responsible for:

- Setting up the pybullet simulation
- Setting up the mocap client
- Updating the simulation state
- Handling user input
"""

import os
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

from typing import List, Tuple

import rclpy
from rclpy.node import Node

import pybullet as p
import pybullet_planning as pp

import pybullet_mocap.husky_world as world
from pybullet_mocap.husky_robot import UR5e_HOME_STATE
from pybullet_mocap.common import (
    Button, Slider, SliderGroup, Husky, TrackedObject, HuskyObject, AssemblyObject, HUSKY_UR5e_JOINT_NAMES, lerp
)

from pybullet_mocap.optitrack.NatNetClient import NatNetClient

TOOL0_FROM_TIP = pp.Pose(point=(0, 0, 73.65 * 1e-3))

DEFAULT_GREY = [0.2, 0.2, 0.2, 0.7]
GOAL_BLUE = [0, 0.2, 0.5, 0.7]
TRAJECTORY_GREEN = [0, 0.5, 0.2, 0.7]

EXISTING_ELEMENT_COLOR = pp.RED
CURRENT_ELEMENT_COLOR = pp.BLUE

CLIENT_IP = '192.168.0.7' # Set to your own IP
MOCAP_IP = '192.168.0.117' # set to the mocap PC's IP, get this from Motive Settings>Streaming pane->Local interface
  
class HuskyMonitor(Node):
    USE_MOCAP = True
    FAKE_HARDWARE = False
    CALIBRATION = False

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
        
        # UI
        self.buttons = []
        self.assembly_position_sliders = []
        self.joint_state_sliders = []
        self.assembly_goal_position_slider_group = None
        self.base_state_slider_group = None

        self.selected_robot_slider = None
        self.selected_robot_id = 0
        
        # goal and trajectory interface
        self.goal_pose = (np.zeros(3), np.array([0, 0, 0, 1]))
        self.goal_gripper = 0.0
        self.goal_arm_pose = UR5e_HOME_STATE
        self.show_goal_state = True

        self.trajectory_time = 2 if self.CALIBRATION else 10

        # list of conf, velocity, total time, attachment other than the ee
        self.planned_arm_trajectory = (None, None, None, None)
        self.plan_traj_seg = None
        self.planned_base_trajectory = (None, None)

        # call setup code
        self.start_pybullet()
        if self.USE_MOCAP:
            self.start_mocap()
        
        world.init(self)

        self.build_ui()
        self.update_partial_assembly()
        
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
    
    def set_arm_trajectory(self, arm_trajectory):
        """ set arm trajectory for visualization"""
        # : Tuple[List[np.ndarray], List[np.ndarray] | None, float]
        self.planned_arm_trajectory = arm_trajectory

    def append_calibration_data(self, data):
        self.calibration_data.append(data)

    def record_calibration_data(self):
        world.save_calibration(self)
        self.calibration_data = []
        
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

    def update_selected_robot_id(self, robot_id):
        self.selected_robot_id = np.clip(int(robot_id), 0, len(self.huskies)-1)
        # update goal pose based on sensed base pose since we are teleoperating the base
        hi = self.huskies[self.selected_robot_id].interface
        self.goal_pose = (hi.position, hi.rotation)

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
        self.planned_arm_trajectory = (None, None, None, None)

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
        self.planned_arm_trajectory = (None, None, None, None)

    def update_traj_goal_configuration(self):
        self.goal_model.set_pose(self.goal_pose, self.goal_arm_pose)

    def plan_arm_to_transfer_element(self):
        world.plan_arm_to_transfer_element(self)
        self.show_goal_state = True
        self.toggle_show_goal_state()

    def plan_arm_to_retract_to_home(self):
        world.plan_arm_to_retract_to_home(self)
        self.show_goal_state = True
        self.toggle_show_goal_state()

    def execute_arm_trajectory(self):
        if not self.FAKE_HARDWARE:
            world.execute_arm_trajectory(self)
        else:
            # fake execution in sim
            if self.planned_arm_trajectory[0] is None:
                self.get_logger().warn('Arm trajectory must be planed before executing!')
            else: 
                ho = self.huskies[self.selected_robot_id].object
                hi = self.huskies[self.selected_robot_id].interface
                if self.planned_arm_trajectory[3] is not None:
                    obj = self.planned_arm_trajectory[3]
                    gripper_tcp_from_object = obj.grasp

                for conf in self.planned_arm_trajectory[0]:
                    hi.arm_joint_pose = conf
                    ho.set_pose((hi.position, hi.rotation), conf)

                    if self.planned_arm_trajectory[3] is not None:
                        # update attached object based on FK
                        world_from_tcp = ho.get_link_pose_from_name("ur_arm_tool0")
                        object_pose = pp.multiply(world_from_tcp, gripper_tcp_from_object)
                        obj.set_pose(object_pose)
                    
                    hi.is_arm_executing = True
                    pp.wait_for_duration(0.01)

                hi.is_arm_executing = False

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
                self.goal_model = HuskyObject(calibration=self.CALIBRATION)
                self.goal_model.set_color(GOAL_BLUE)
        
    def build_ui(self, target_conf=None):
        # default_base_position = [0,0,0]
        # self.assembly_goal_position_slider_group = SliderGroup(["target base {}".format(t) for t in ["x","y","z"]], self.update_assembly_goal_position, [0, -5, 0], [5,5,1], default_base_position)

        self.buttons.append(Button('Prev in sequence', self.show_previous_in_sequence))
        self.buttons.append(Button('Next in sequence', self.show_next_in_sequence))

        self.selected_robot_slider = Slider("robot id", self.update_selected_robot_id, 0, len(self.huskies)+1, 0)
        # p.addUserDebugParameter("robot id", 0, len(self.huskies)+1, 0)

        self.trajectory_time_slider = Slider("traj time", self.update_trajectory_time, 1.0, 30.0, self.trajectory_time)

        self.time_slider = p.addUserDebugParameter("time", 0.0, 1.0, 1.0)
        
        self.buttons.append(Button('Toggle Goal/Trajectory', self.toggle_show_goal_state))
        self.buttons.append(Button('Reset Goal State', self.reset_ui))
        
        if not self.USE_MOCAP:
            pose2d = pp.pose2d_from_pose((self.huskies[self.selected_robot_id].interface.position, self.huskies[self.selected_robot_id].interface.rotation), tolerance=0.1)
            self.teleop_base_slider_group = SliderGroup(["teleop base {}".format(t) for t in ["x","y","yaw"]], self.update_base_conf, [-5.0, -5.0, -np.pi], [5.0,5.0,np.pi], pose2d)
            # self.state_sliders.append(p.addUserDebugParameter("x", -5.0, 5.0, pose2d[0]))
            # self.state_sliders.append(p.addUserDebugParameter("y", -5.0, 5.0, pose2d[1]))
            # self.state_sliders.append(p.addUserDebugParameter("yaw", -np.pi, np.pi, pose2d[2]))
        # self.buttons.append(Button('Plan base', lambda: world.plan_to_goal(self)))
        # self.buttons.append(Button('Exec Base', lambda: world.move_to_goal(self)))
               
        self.buttons.append(Button('Plan arm to assemble current element', self.plan_arm_to_transfer_element))
        self.buttons.append(Button('Plan arm to retract to home', self.plan_arm_to_retract_to_home))
        self.buttons.append(Button('Exec Arm', self.execute_arm_trajectory))

        self.buttons.append(Button('Plan arm wave', lambda: world.plan_arm_wave(self)))

        self.gripper_slider = p.addUserDebugParameter("gripper", 0, 1.0, 0.1)
        self.buttons.append(Button('Exec Gripper', lambda: world.set_gripper(self)))

        self.buttons.append(Button('Open Gripper', lambda: world.open_gripper_full(self)))
        self.buttons.append(Button('Close Gripper', lambda: world.close_gripper_for_bar(self)))

        for i, j in enumerate(pp.joints_from_names(self.huskies[0].object.robot, HUSKY_UR5e_JOINT_NAMES)):
            lower, upper = pp.get_joint_limits(self.huskies[0].object.robot, j)
            if target_conf is None:
                self.joint_state_sliders.append(p.addUserDebugParameter(f'Joint {i}', lower, upper, self.huskies[0].interface.arm_joint_pose[i]))
            else:
                self.joint_state_sliders.append(p.addUserDebugParameter(f'Joint {i}', lower, upper, target_conf[i]))
        self.buttons.append(Button('Plan arm to conf target', lambda: world.plan_arm_to_goal(self)))

        self.buttons.append(Button('Calib joint 0', lambda: world.calibrate_joint(self, 0, 'calib_tool')))
        self.buttons.append(Button('Set joint 0 to zero', self.set_goal_joint_0_to_zero))
        self.buttons.append(Button('Calib joint 1', lambda: world.calibrate_joint(self, 1, 'calib_tool')))
        self.buttons.append(Button('Execute one step', self.execute_one_step))
        self.buttons.append(Button('Record current calib conf', lambda: world.calibrate_button(self, 'calib_tool')))
        self.buttons.append(Button('Export calib conf to json', self.record_calibration_data))

    
    # --- --- --- --- --- MOCAP --- --- --- --- --- 
    def start_mocap(self):
        print('Starting mocap!')
        mocap_client = NatNetClient()
        mocap_client.set_client_address(CLIENT_IP)
        mocap_client.set_server_address(MOCAP_IP)
        mocap_client.set_use_multicast(False)
        mocap_client.print_level = 1
        mocap_client.rigid_body_listener = self.receive_rigid_body_frame
        mocap_client.new_frame_listener = self.receive_mocap_frame
        
        if mocap_client.run():
            start_connect = time.time()
            while not mocap_client.connected():
                time.sleep(0.25)
                if time.time() - start_connect > 5:
                    break
            print(f"mocap client connected: {mocap_client.connected()}")
        else:
            print('Failed to run mocap client!')
    
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
            h.object.set_pose((hi.position, hi.rotation), hi.arm_joint_pose)
            # set the goal pose of base since we are teleoperating the base
            self.goal_pose = (hi.position, hi.rotation)

        # pp.draw_pose(self.goal_model.get_link_pose_from_name("ur_arm_base_link"))

        self.selected_robot_slider.update()
        self.trajectory_time_slider.update()

        if not self.USE_MOCAP:
            self.teleop_base_slider_group.update()
        
        # update goal robot state
        # state_slider_values = [p.readUserDebugParameter(ps) for ps in self.state_sliders]
        # self.goal_pose = (
        #     np.array((state_slider_values[0], state_slider_values[1], 0)),
        #     R.from_euler("z", state_slider_values[2], degrees=False).as_quat()
        # )
        self.goal_gripper = p.readUserDebugParameter(self.gripper_slider)
        self.goal_arm_pose = np.array([p.readUserDebugParameter(ps) for ps in self.joint_state_sliders])

        # update assembly goal position
        # self.assembly_goal_position_slider_group.update()
            
        preview_time = p.readUserDebugParameter(self.time_slider)
        goal_pose = self.goal_pose
        goal_arm_pose = self.goal_arm_pose
        if not self.show_goal_state:
            if self.planned_base_trajectory[0] is not None:
                N = len(self.planned_base_trajectory[0])
                base_traj_idx = int(preview_time * (N - 1))
                goal_pose = self.planned_base_trajectory[0][base_traj_idx]

            if self.planned_arm_trajectory[0] is not None:
                N = len(self.planned_arm_trajectory[0])
                arm_traj_idx_float = preview_time * (N - 1)
                arm_traj_idx = int(arm_traj_idx_float)
                goal_arm_pose = self.planned_arm_trajectory[0][arm_traj_idx]

                # dt = arm_traj_idx_float - arm_traj_idx
                # arm_traj_idx_plus = min(int(preview_time * (N - 1) + 1), N-1)
                # goal_arm_pose = lerp(self.planned_arm_trajectory[0][arm_traj_idx], self.planned_arm_trajectory[0][arm_traj_idx_plus], dt)

            if self.planned_arm_trajectory[3] is not None:
                # update attached object based on FK
                obj = self.planned_arm_trajectory[3]
                gripper_tcp_from_object = obj.grasp
                world_from_tcp = self.goal_model.get_link_pose_from_name("ur_arm_tool0")
                object_pose = pp.multiply(world_from_tcp, gripper_tcp_from_object)
                obj.set_pose(object_pose)
 
        self.goal_model.set_pose(goal_pose, goal_arm_pose)
                        
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
