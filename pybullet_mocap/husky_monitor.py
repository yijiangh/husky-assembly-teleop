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

from pybullet_mocap import DATA_DIRECTORY
import pybullet_mocap.husky_world as world
from pybullet_mocap.husky_robot import HuskyRobotInterface
from pybullet_mocap.common import (
    Button, Husky, TrackedObject, HuskyObject, HUSKY_UR5e_JOINT_NAMES, HUSKY_JOINT_NAMES
)

from pybullet_mocap.optitrack.NatNetClient import NatNetClient

TOOL0_FROM_TIP = pp.Pose(point=(0, 0, 73.65 * 1e-3))

DEFAULT_GREY = [0.2, 0.2, 0.2, 0.7]
GOAL_BLUE = [0, 0.2, 0.5, 0.7]
TRAJECTORY_GREEN = [0, 0.5, 0.2, 0.7]

CLIENT_IP = '192.168.0.7' # Set to your own IP
MOCAP_IP = '192.168.0.117' # set to the mocap PC's IP, get this from Motive Settings>Streaming pane->Local interface
        
class HuskyMonitor(Node):
    def __init__(self):
        super().__init__('husky_monitor')
        
        self.tick_timer = self.create_timer(0.05, self.update)
        
        # simple async tasks to be executed every tick
        self.tasks = []
        
        self.huskies = []
        self.tracked_objects = []
        self.name_from_mocap_id = {}
        
        # UI
        self.buttons = []
        self.state_sliders = []
        self.joint_state_sliders = []
        
        # goal and trajectory interface
        self.goal_pose = (np.zeros(3), np.array([0, 0, 0, 1]))
        self.goal_gripper = 0.0
        self.goal_arm_pose = np.zeros(6)

        self.planned_arm_trajectory = None
        self.plan_traj_seg = None
        self.planned_base_trajectory = None

        # call setup code
        self.start_pybullet()
        self.start_mocap()
        
        world.init(self)

        self.build_ui()
        
    def add_tracked_object(self, obstacle: TrackedObject):
        """Registers an object to be tracked by mocap"""
        self.tracked_objects.append(obstacle)
        self.name_from_mocap_id[obstacle.mocap_id] = obstacle.name
        
    def add_husky(self, husky: Husky):
        """Registers a husky to connect to ROS and be tracked by mocap"""
        self.huskies.append(husky)
        self.name_from_mocap_id[husky.mocap_id] = husky.name
        
    def set_base_trajectry(self, base_trajectory: List[Tuple[np.ndarray, np.ndarray]]):
            """ set base trajectory for visualization"""
            self.planned_base_trajectory = base_trajectory
            
            # draw
            points = [
                pos for pos, _ in self.planned_base_trajectory
            ]
            with pp.LockRenderer():
                with pp.HideOutput():
                    if self.plan_traj_seg is not None:
                       pp.remove_all_debug()
                    self.plan_traj_seg = pp.add_segments(points)
    
    def set_arm_trajectory(self, arm_trajectory: List[np.ndarray]):
        """ set arm trajectory for visualization"""
        self.planned_arm_trajectory = arm_trajectory
        
    def reset_ui(self):
        # reset all sliders to default value by recreating them...
        # pybullet seems to lack a setUserDebugParameter() method :(
        p.removeAllUserParameters()
        self.buttons.clear()
        self.state_sliders.clear()
        self.joint_state_sliders.clear()
        self.build_ui()
    
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
                self.goal_model = HuskyObject()
                self.goal_model.set_color(GOAL_BLUE)
        
    def build_ui(self):
        self.time_slider = p.addUserDebugParameter("time", 0.0, 1.0, 1.0)
        
        self.buttons.append(Button('Reset Goal State', self.reset_ui))
        
        self.state_sliders.append(p.addUserDebugParameter("x", -5.0, 5.0, 0))
        self.state_sliders.append(p.addUserDebugParameter("x", -5.0, 5.0, 0))
        self.state_sliders.append(p.addUserDebugParameter("yaw", -np.pi, np.pi, 0))
        
        self.buttons.append(Button('Plan', lambda: world.plan_to_goal(self)))
        self.buttons.append(Button('Exec Base', lambda: world.move_to_goal(self)))
        
        for i, j in enumerate(pp.joints_from_names(self.husky[0].object.robot, HUSKY_UR5e_JOINT_NAMES)):
            lower, upper = pp.get_joint_limits(self.husky[0].object.robot, j)
            self.joint_state_sliders.append(p.addUserDebugParameter(f'Joint {i}', lower, upper, self.huskies[0].interface.arm_joint_pose[i]))
        
        self.buttons.append(Button('Plan', lambda: world.plan_arm_to_goal(self)))
        self.buttons.append(Button('Plan Wave', lambda: world.plan_arm_wave(self)))
        self.buttons.append(Button('Exec Arm', lambda: world.execute_arm_trajectory(self)))

        self.gripper_slider = p.addUserDebugParameter("gripper", 0, 1.0)
        self.buttons.append(Button('Exec Gripper', lambda: world.set_gripper(self)))
    
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
            (pos, rot) = self._mocap_rigidbody_cache[h.name]
            h.interface.mocap_callback(pos, rot, ts)
        for o in self.tracked_objects:
            if o.name not in self._mocap_rigidbody_cache:
                continue
            (pos, rot) = self._mocap_rigidbody_cache[o.name]
            o.mocap_callback(pos, rot, ts)
        self._mocap_rigidbody_cache.clear()
        
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
            h.object[i].set_pose((hi.position, hi.rotation), hi.arm_joint_pose)
        
        # update goal robot state
        state_slider_values = [p.readUserDebugParameter(ps) for ps in self.state_sliders]
        self.goal_pose = (
            np.array((state_slider_values[0], state_slider_values[1], 0)),
            R.from_euler("z", state_slider_values[2], degrees=False).as_quat()
        )
        self.goal_gripper = p.readUserDebugParameter(self.gripper_slider)
        self.goal_arm_pose = np.array([p.readUserDebugParameter(ps) for ps in self.joint_state_sliders])
            
        preview_time = p.readUserDebugParameter(self.time_slider)
        goal_pose = (self.goal_pos, self.goal_rot)
        goal_arm_pose = self.goal_arm_pose
        if not np.isclose(preview_time, 1.0):
            if self.planned_base_trajectory:
                base_traj_idx = int(preview_time * (len(self.planned_base_trajectory) - 1))
                goal_pose = self.planned_base_trajectory[base_traj_idx]
            if self.planned_arm_trajectory:
                arm_traj_idx = int(preview_time * (len(self.planned_arm_trajectory) - 1))
                goal_arm_pose = self.planned_arm_trajectory[arm_traj_idx]
            
        self.goal_model.set_pose(goal_pose, goal_arm_pose)
        
        # run tasks
        i = 0
        while i < len(self.tasks):
            task = self.tasks[i]
            if task():
                i += 1
            else:
                self.tasks.remove(task)
                
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
