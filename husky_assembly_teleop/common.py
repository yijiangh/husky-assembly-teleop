"""
A collection of common functions and classes used in the husky_assembly_teleop package.
"""

import os
import numpy as np
import pybullet as p
import json

import pybullet_planning as pp

from husky_assembly_teleop import DATA_DIRECTORY
from husky_assembly_teleop.husky_robot import HuskyRobotInterface
from husky_assembly_teleop.utils import UR5E_JOINT_NAMES

# --- --- PYBULLET OBJECTS --- ---

HUSKY_UR5e_JOINT_NAMES = ["ur_arm_shoulder_pan_joint", 
                      "ur_arm_shoulder_lift_joint",
                      "ur_arm_elbow_joint", 
                      "ur_arm_wrist_1_joint", 
                      "ur_arm_wrist_2_joint", 
                      "ur_arm_wrist_3_joint" ]

HUSKY_DUAL_UR5e_JOINT_NAMES = [["left_ur_arm_shoulder_pan_joint", 
                      "left_ur_arm_shoulder_lift_joint",
                      "left_ur_arm_elbow_joint", 
                      "left_ur_arm_wrist_1_joint", 
                      "left_ur_arm_wrist_2_joint", 
                      "left_ur_arm_wrist_3_joint" ],
                                ["right_ur_arm_shoulder_pan_joint", 
                      "right_ur_arm_shoulder_lift_joint",
                      "right_ur_arm_elbow_joint", 
                      "right_ur_arm_wrist_1_joint", 
                      "right_ur_arm_wrist_2_joint", 
                      "right_ur_arm_wrist_3_joint" ]]

def load_robot(ik_from_arm_base=True, load_calib_tip=False, dual_arm=False):
    # robot_srdf = os.path.join(DATA_DIRECTORY, 'husky_urdf/mt_husky_moveit_config/config/husky.srdf')
    # robot_urdf = os.path.join(DATA_DIRECTORY,'husky_urdf/mt_husky_moveit_config/urdf/husky_ur5_e.urdf')
    robot_urdf = None
    print('loading robot urdf from:', DATA_DIRECTORY)
    if dual_arm:
        robot_urdf = os.path.join(DATA_DIRECTORY,'husky_urdf/mt_husky_dual_ur5_e_moveit_config/urdf/husky_dual_ur5_e.urdf')
    else:
        robot_urdf = os.path.join(DATA_DIRECTORY,'husky_urdf/mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint.urdf')

    if load_calib_tip:
        # gripper_obj = os.path.join(DATA_DIRECTORY,'calibration_probe.obj')
        # gripper_scale = 1
        # ee = pp.create_obj(gripper_obj, scale=gripper_scale) 
        ee = pp.create_box(0.15, 0.15, 0.05)
        pp.set_color(ee, pp.apply_alpha(pp.GREY, 0.3))
    else:
        gripper_obj = os.path.join(DATA_DIRECTORY,'husky_urdf/robotiq_85/meshes/static/robotiq_85_close_20mm.obj')
        assert os.path.exists(gripper_obj)
        gripper_scale = 1
        ee = pp.create_obj(gripper_obj, scale=gripper_scale) 

    assert os.path.exists(robot_urdf)

    robot = pp.load_pybullet(robot_urdf, fixed_base=False, cylinder=False)
    
    ee_list = []

    left_tool0_pose = pp.get_link_pose(robot, pp.link_from_name(robot, ('left_' if dual_arm else '') + 'ur_arm_tool0'))
    left_ee = pp.create_obj(gripper_obj, scale=gripper_scale) 
    pp.set_pose(left_ee, pp.multiply(left_tool0_pose, pp.Pose(euler=pp.Euler(yaw=-np.pi/2))))
    left_ee_attachment = pp.create_attachment(robot, pp.link_from_name(robot, ('left_' if dual_arm else '') + 'ur_arm_tool0'), left_ee)
    ee_list.append((left_ee, left_ee_attachment))
    
    if dual_arm:
        right_tool0_pose = pp.get_link_pose(robot, pp.link_from_name(robot, 'right_ur_arm_tool0'))
        right_ee = pp.create_obj(gripper_obj, scale=gripper_scale) 
        pp.set_pose(right_ee, pp.multiply(right_tool0_pose, pp.Pose(euler=pp.Euler(yaw=-np.pi/2))))
        right_ee_attachment = pp.create_attachment(robot, pp.link_from_name(robot, 'right_ur_arm_tool0'), right_ee)
        ee_list.append((right_ee, right_ee_attachment))

    return robot, ee_list

def load_gripper(load_calib_tip=False):
    if load_calib_tip:
        gripper_obj = os.path.join(DATA_DIRECTORY,'calibration_tip.stl')
        gripper_scale = 1
    else:
        gripper_obj = os.path.join(DATA_DIRECTORY,'husky_urdf/robotiq_85/meshes/static/robotiq_85_close_20mm.obj')
        gripper_scale = 1

    return pp.create_obj(gripper_obj, scale=gripper_scale)

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
    def __init__(self, monitor, name, mocap_id, pos, rot, scale, model_file=None):
        self.name = name
        self.mocap_id = mocap_id
        self.pos = pos
        self.rot = rot
        self.model_base_pose = None # an additional transformation that should be carried to all pose
        
        self.body = None
        if model_file:
            with pp.LockRenderer():
                with pp.HideOutput():
                    self.body = pp.create_obj(os.path.join(DATA_DIRECTORY, model_file), scale=scale)
        
        monitor.add_tracked_object(self)
        
    def mocap_callback(self, pos, rot, ts):
        self.pos = pos
        self.rot = rot
        
    def set_pose(self, base_pose):
        if self.body:
            if self.model_base_pose is not None:
                base_pose = pp.multiply(base_pose, self.model_base_pose)
            pp.set_pose(self.body, base_pose)

class Husky():
    """A husky interface with corresponding husky object."""
    def __init__(self, monitor, name, mocap_id=None, pos=np.zeros(3), rot=np.array((0, 0, 0, 1)), 
                 connect_arm=True, connect_gripper=True, base_calibration_file=None, calibration=False, dual_arm=False):
        self.name = name
        self.mocap_id = mocap_id
        self.interface = HuskyRobotInterface(monitor, 
                                             name, 
                                             use_odom=(mocap_id is None), 
                                             connect_arm=connect_arm, 
                                             connect_gripper=connect_gripper, 
                                             dual_arm=dual_arm
                                             )
        self.object = HuskyObject(calibration=calibration, dual_arm=dual_arm)
        self.dual_arm = dual_arm
        
        self.interface.position = pos
        self.interface.rotation = rot

        self.base_mocap_from_base_footprint = pp.Pose(point=np.zeros(3))
        if base_calibration_file:
            # read from json
            with open(base_calibration_file, 'r') as file:
                data = json.load(file)
            pose = data['base_mocap_from_base_footprint']
            self.base_mocap_from_base_footprint = (pose[0], pose[1])
        
        monitor.add_husky(self)

class HuskyObject():
    """Collection of pybullet objects representing a husky"""
    def __init__(self, calibration=False, dual_arm=False):
        with pp.LockRenderer():
            with pp.HideOutput():
                robot, ee_list = load_robot(load_calib_tip=calibration, dual_arm=dual_arm)
                self.robot = robot
                self.ee_list = ee_list
                self.old_color = None
                self.dual_arm = dual_arm
       
    def set_pose(self, base_pose, arm_joint_states, index=0):
        """Set pose of base and ur5e arm(s). arm_joint_states must be of shape [[joint_values]] or [[left_joints], [right_joints]]"""        
        pp.set_pose(self.robot, base_pose)
        
        if len(arm_joint_states) == 1:
            arm_joints = pp.joints_from_names(self.robot, self.get_arm_joint_names(index))
            pp.set_joint_positions(self.robot, arm_joints, arm_joint_states[0])
        elif len(arm_joint_states) == 2:
            arm_joints = pp.joints_from_names(self.robot, self.get_arm_joint_names(index=0))
            pp.set_joint_positions(self.robot, arm_joints, arm_joint_states[0])
            if self.dual_arm:
                arm_joints = pp.joints_from_names(self.robot, self.get_arm_joint_names(index=1))
                pp.set_joint_positions(self.robot, arm_joints, arm_joint_states[1])
        else:
            # print('set_pose arm_joint_states has invalid shape!')
            # return
            raise ValueError(f'set_pose arm_joint_states has invalid shape! {arm_joint_states}')
        
        # jg: why was this removed?
        for (ee, ee_attachment) in self.ee_list:
            ee_attachment.assign()

             
    def set_color(self, new_color):
        if self.old_color != new_color:
            self.old_color = new_color
            pp.set_color(self.robot, new_color)
            for (ee, ee_attachment) in self.ee_list: 
                pp.set_color(ee, new_color)
                
    def get_arm_joint_names(self, index=0):
        if self.dual_arm:
            return HUSKY_DUAL_UR5e_JOINT_NAMES[index]
        else:
            return HUSKY_UR5e_JOINT_NAMES
            
    def get_ee_pose(self, index=0):
        return pp.get_pose(self.ee_list[index][0])

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
    q1 = np.array(q1)
    q2 = np.array(q2)
    if np.dot(q1,q2) < 0:
        q2 = -q2
    
    res = lerp(q1, q2, t)
    res /= np.linalg.norm(res)
    
    return res