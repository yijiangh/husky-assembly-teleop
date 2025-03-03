"""
A collection of common functions and classes used in the pybullet_mocap package.
"""

import os
import numpy as np
import pybullet as p

import pybullet_planning as pp

from pybullet_mocap import DATA_DIRECTORY
from pybullet_mocap.husky_robot import HuskyRobotInterface

# --- --- PYBULLET OBJECTS --- ---

HUSKY_JOINT_NAMES = ['x', 'y', 'theta']
HUSKY_UR5e_JOINT_NAMES = ["ur_arm_shoulder_pan_joint", 
                      "ur_arm_shoulder_lift_joint",
                      "ur_arm_elbow_joint", 
                      "ur_arm_wrist_1_joint", 
                      "ur_arm_wrist_2_joint", 
                      "ur_arm_wrist_3_joint" ]

def load_robot(ik_from_arm_base=True, load_calib_tip=False):
    # robot_srdf = os.path.join(DATA_DIRECTORY, 'husky_urdf/mt_husky_moveit_config/config/husky.srdf')
    robot_urdf = os.path.join(DATA_DIRECTORY,'husky_urdf/mt_husky_moveit_config/urdf/husky_ur5_e.urdf')
    # robot_urdf = os.path.join(DATA_DIRECTORY,'husky_urdf/mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint.urdf')

    if load_calib_tip:
        gripper_obj = os.path.join(DATA_DIRECTORY,'calibration_probe.obj')
        gripper_scale = 1
    else:
        gripper_obj = os.path.join(DATA_DIRECTORY,'husky_urdf/robotiq_85/meshes/static/robotiq_85_close_20mm.obj')
        gripper_scale = 1

    assert os.path.exists(robot_urdf)
    assert os.path.exists(gripper_obj)

    robot = pp.load_pybullet(robot_urdf, fixed_base=False, cylinder=False)
    robot_pose = pp.get_pose(robot)

    tool0_pose = pp.get_link_pose(robot, pp.link_from_name(robot, 'ur_arm_tool0'))
    ee = pp.create_obj(gripper_obj, scale=gripper_scale) 
    pp.set_pose(ee, pp.multiply(tool0_pose, pp.Pose(euler=pp.Euler(yaw=-np.pi/2))))
    
    ee_attachment = pp.create_attachment(robot, pp.link_from_name(robot, 'ur_arm_tool0'), ee)

    return robot, ee, ee_attachment

class AssemblyObject:
    def __init__(self, monitor, index: int, pb_body, init_pose, goal_pose, grasp=None):
        self.name = index
        self.body = pb_body
        self.init_pose = init_pose
        self.archived_goal_position = goal_pose[0]
        self.goal_pose = goal_pose
        self.grasp = grasp # gripper_tcp_from_object

        monitor.add_assembly_objects(self)

    def show(self):
        pp.set_pose(self.body, self.goal_pose)

    def hide(self):
        pp.set_pose(self.body, self.init_pose)

    def update_goal_pose(self, goal_pose):
        self.goal_pose = goal_pose

    def update_grasp(self, grasp):
        self.grasp = grasp

    def set_pose(self, pose):
        pp.set_pose(self.body, pose)
    
class TrackedObject:
    """PyBullet objects with pose tracked using mocap"""
    def __init__(self, monitor, name, mocap_id, pos, rot, scale, model_file):
        self.name = name
        self.mocap_id = mocap_id
        self.pos = pos
        self.rot = rot
        
        with pp.LockRenderer():
            with pp.HideOutput():
                self.body = pp.create_obj(os.path.join(DATA_DIRECTORY, model_file), scale=scale)
        
        monitor.add_tracked_object(self)
        
    def mocap_callback(self, pos, rot, ts):
        self.pos = pos
        self.rot = rot
        
    def set_pose(self, base_pose):
        pp.set_pose(self.body, base_pose)

class Husky():
    """A husky interface with corresponding husky object."""
    def __init__(self, monitor, name, mocap_id=None, pos=np.zeros(3), rot=np.array((0, 0, 0, 1)), connect_arm=True, connect_gripper=True):
        self.name = name
        self.mocap_id = mocap_id
        self.interface = HuskyRobotInterface(monitor, name, use_odom=(mocap_id is None), connect_arm=connect_arm, connect_gripper=connect_gripper)
        self.object = HuskyObject()
        
        self.interface.position = pos
        self.interface.rotation = rot
        
        monitor.add_husky(self)

class HuskyObject():
    """Collection of pybullet objects representing a husky"""
    def __init__(self):
        with pp.LockRenderer():
            with pp.HideOutput():
                robot, ee, ee_attachment = load_robot(load_calib_tip=False)
                self.robot = robot
                self.ee = ee
                self.ee_attachment = ee_attachment
                self.old_color = None
       
    def set_pose(self, base_pose, arm_joint_states):
        pp.set_pose(self.robot, base_pose)
        arm_joints = pp.joints_from_names(self.robot, HUSKY_UR5e_JOINT_NAMES)
        pp.set_joint_positions(self.robot, arm_joints, arm_joint_states)
        self.ee_attachment.assign()
        
    def set_color(self, new_color):
        if self.old_color != new_color:
            self.old_color = new_color
            pp.set_color(self.robot, new_color)
            pp.set_color(self.ee, new_color)
            
    def get_ee_pose(self):
        return pp.get_pose(self.ee)

    def get_link_pose_from_name(self, link_name):
        return pp.get_link_pose(self.robot, pp.link_from_name(self.robot, link_name))

class Button:
    def __init__(self, name, action):
        self.dbg_param = p.addUserDebugParameter(name, 1.0, 0.0, 0.0)
        self.prev_value = p.readUserDebugParameter(self.dbg_param)
        self.action = action
        
    def update(self):
        new_value = p.readUserDebugParameter(self.dbg_param)
        if new_value != self.prev_value:
            self.prev_value = new_value
            self.action()

class Slider:
    def __init__(self, name, action, min_val=0.0, max_val=1.0, current_val=0.0):
        self.dbg_param = p.addUserDebugParameter(name, min_val, max_val, current_val)
        self.prev_value = p.readUserDebugParameter(self.dbg_param)
        self.action = action
        
    def update(self):
        new_value = p.readUserDebugParameter(self.dbg_param)
        if new_value != self.prev_value:
            self.prev_value = new_value
            self.action(new_value)

class SliderGroup:
    def __init__(self, names, action, min_vals, max_vals, current_vals):
        self.dbg_params = [p.addUserDebugParameter(name, min_val, max_val, current_val) for name, min_val, max_val, current_val in zip(names, min_vals, max_vals, current_vals)]
        self.prev_values = [p.readUserDebugParameter(dbgp) for dbgp in self.dbg_params]
        self.action = action
        
    def update(self):
        new_values = [p.readUserDebugParameter(param) for param in self.dbg_params]
        # use numpy to determin if any value has changed
        if not np.allclose(new_values, self.prev_values):
            self.prev_values = new_values
            self.action(new_values)

# --- --- QUATERNION AND MATH FUNCTIONS --- ---

def lerp(a, b, t):
    return a + t * (b - a)

def quat_lerp(q1, q2, t):
    if np.dot(q1,q2) < 0:
        q2 = -q2
    
    res = lerp(q1, q2, t)
    res /= np.linalg.norm(res)
    
    return res