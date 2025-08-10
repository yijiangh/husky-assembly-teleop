"""
A collection of common functions and classes used in the husky_assembly_teleop package.
"""

import os
import numpy as np
import pybullet as p
import json

import pybullet_planning as pp
from compas.data import json_load
from compas_fab.backends import PyBulletClient, PyBulletPlanner

from husky_assembly_teleop import DATA_DIRECTORY
from husky_assembly_teleop.husky_robot import HuskyRobotInterface
from husky_assembly_teleop.utils import UR5E_JOINT_NAMES

# Design data directory for validation point tools
DESIGN_DATA_DIRECTORY = os.path.join(DATA_DIRECTORY, 'husky_assembly_design_study')

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

# --- --- END EFFECTOR MANAGEMENT --- ---

def load_robot(dual_arm=False):
    """
    Load robot URDF without end effectors.
    
    Args:
        dual_arm: Whether this is a dual-arm robot
        
    Returns:
        robot: PyBullet robot body ID
    """
    robot_urdf = None
    print('loading robot urdf from:', DATA_DIRECTORY)
    if dual_arm:
        robot_urdf = os.path.join(DATA_DIRECTORY,'husky_urdf/mt_husky_dual_ur5_e_moveit_config/urdf/husky_dual_ur5_e_no_base_joint.urdf')
    else:
        robot_urdf = os.path.join(DATA_DIRECTORY,'husky_urdf/mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint.urdf')

    assert os.path.exists(robot_urdf)
    robot = pp.load_pybullet(robot_urdf, fixed_base=False, cylinder=False)
    
    return robot

def create_end_effector(ee_type="victor_gripper", load_calib_tip=False, dual_arm=False):
    """
    Create end effector based on type.
    
    Args:
        ee_type: Type of end effector ("victor_gripper", "robotiq_gripper", "custom_gripper", "validation_point_tool", or "calib_tip")
        load_calib_tip: Whether to load calibration tip (overrides ee_type)
        dual_arm: Whether this is for a dual-arm robot
        
    Returns:
        ee: PyBullet end effector body ID or list of IDs for dual arm validation tools
    """
    if load_calib_tip:
        ee = pp.create_box(0.12, 0.12, 0.12)
        pp.set_color(ee, pp.apply_alpha(pp.GREY, 0.3))
        return ee
    
    if ee_type == "victor_gripper":
        gripper_urdf_path = os.path.join(DATA_DIRECTORY, 'grasp_screw_tool_description/urdf/grasp_screw_tool_unactuated.urdf')
        ee = pp.load_pybullet(gripper_urdf_path, fixed_base=False, cylinder=False)
        return ee
    elif ee_type == "robotiq_gripper":
        gripper_obj = os.path.join(DATA_DIRECTORY,'husky_urdf/robotiq_85/meshes/static/robotiq_85_close_20mm.obj')
        assert os.path.exists(gripper_obj)
        gripper_scale = 1
        ee = pp.create_obj(gripper_obj, scale=gripper_scale)
        return ee
    elif ee_type == "validation_point_tool":
        # Load tools from compas_fab robot cell
        problem_name = '250806_RobotX_box_redo'
        robot_cell = json_load(os.path.join(DESIGN_DATA_DIRECTORY, problem_name, 'RobotCell.json'))
        
        # For now, use a default state file - you might want to make this configurable
        state_file = 'robotx_box_A5-S4_end_RobotCellState.json'
        robot_cell_state = json_load(os.path.join(DESIGN_DATA_DIRECTORY, problem_name, 'RobotCellStates', state_file))
        
        # Create a separate PyBullet client for compas_fab to avoid conflicts
        with PyBulletClient(connection_type="direct", verbose=False) as client:
            # Make pp know the client id created by compas_fab client
            original_client_id = pp.get_client()
            
            pp.set_client(client.client_id)
            pp.CLIENTS[client.client_id] = None  # Direct connection, no GUI
            
            planner = PyBulletPlanner(client)
            
            # Set robot cell and state - this creates the PyBullet bodies
            planner.set_robot_cell(robot_cell)
            planner.set_robot_cell_state(robot_cell_state)
            
            # Get the attached tools for left and right arms
            left_group = "base_left_arm_manipulator"
            right_group = "base_right_arm_manipulator"
            
            left_tool = robot_cell.get_attached_tool(robot_cell_state, left_group)
            right_tool = robot_cell.get_attached_tool(robot_cell_state, right_group)
            
            # Get the PyBullet UIDs for the tools
            tool_uids = []
            
            if left_tool and left_tool.name in client.tools_puids:
                left_tool_uid = client.tools_puids[left_tool.name]
                # Clone the tool body to the original PyBullet instance
                pp.set_client(original_client_id)
                cloned_left_tool = pp.clone_body(left_tool_uid, client=client.client_id)
                tool_uids.append(cloned_left_tool)
                pp.set_client(client.client_id)
            
            if dual_arm and right_tool and right_tool.name in client.tools_puids:
                right_tool_uid = client.tools_puids[right_tool.name]
                # Clone the tool body to the original PyBullet instance
                pp.set_client(original_client_id)
                cloned_right_tool = pp.clone_body(right_tool_uid, client=client.client_id)
                tool_uids.append(cloned_right_tool)
                pp.set_client(client.client_id)
            
            # Restore the original client
            pp.set_client(original_client_id)
            
            # Return single tool for single arm, list for dual arm
            if len(tool_uids) == 1:
                return tool_uids[0]
            else:
                return tool_uids
            
    elif ee_type == "custom_gripper":
        # Example of adding a new end effector type
        # You can load from URDF, OBJ, or create a simple geometric shape
        custom_gripper_path = os.path.join(DATA_DIRECTORY, 'custom_gripper_description/urdf/custom_gripper.urdf')
        if os.path.exists(custom_gripper_path):
            # Load from URDF if available
            ee = pp.load_pybullet(custom_gripper_path, fixed_base=False, cylinder=False)
        else:
            # Fallback to simple geometric shape
            ee = pp.create_cylinder(radius=0.05, height=0.15, color=(0.8, 0.8, 0.8, 1))
        return ee
    else:
        raise ValueError(f"Unknown end effector type: {ee_type}. Valid types: victor_gripper, robotiq_gripper, custom_gripper, validation_point_tool, calib_tip")

def attach_end_effectors(robot, ee_list, dual_arm=False):
    """
    Attach end effectors to robot tool0 links.
    
    Args:
        robot: PyBullet robot body ID
        ee_list: List of end effector body IDs
        dual_arm: Whether this is a dual-arm robot
        
    Returns:
        attached_ee_list: List of (ee_body, ee_attachment) tuples
    """
    attached_ee_list = []
    
    for i, ee in enumerate(ee_list):
        if dual_arm:
            if i == 0:  # Left arm
                tool0_link_name = 'left_ur_arm_tool0'
            else:  # Right arm
                tool0_link_name = 'right_ur_arm_tool0'
        else:
            tool0_link_name = 'ur_arm_tool0'
        
        tool0_pose = pp.get_link_pose(robot, pp.link_from_name(robot, tool0_link_name))
        
        # Set end effector pose
        pp.set_pose(ee, tool0_pose)
        
        # Create attachment
        ee_attachment = pp.create_attachment(robot, pp.link_from_name(robot, tool0_link_name), ee)
        attached_ee_list.append((ee, ee_attachment))
    
    return attached_ee_list

def load_robot_with_end_effectors(ee_types=None, load_calib_tip=False, dual_arm=False):
    """
    Load robot with end effectors (legacy function for backward compatibility).
    
    Args:
        ee_types: List of end effector types
        load_calib_tip: Whether to load calibration tip
        dual_arm: Whether this is a dual-arm robot
        
    Returns:
        robot: PyBullet robot body ID
        attached_ee_list: List of (ee_body, ee_attachment) tuples
    """
    robot = load_robot(dual_arm=dual_arm)
    
    if ee_types is None:
        if load_calib_tip:
            ee_types = ["calib_tip"]
        else:
            ee_types = ["victor_gripper"]
    
    if dual_arm:
        if len(ee_types) == 1:
            ee_types = [ee_types[0], ee_types[0]]  # Use same type for both arms
    
    ee_list = []
    for ee_type in ee_types:
        if ee_type == "calib_tip":
            ee = create_end_effector(load_calib_tip=True, dual_arm=dual_arm)
        else:
            ee = create_end_effector(ee_type=ee_type, dual_arm=dual_arm)
        
        # Handle validation_point_tool which might return a list
        if isinstance(ee, list):
            ee_list.extend(ee)
        else:
            ee_list.append(ee)
    
    attached_ee_list = attach_end_effectors(robot, ee_list, dual_arm=dual_arm)
    
    return robot, attached_ee_list

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
    """
    A husky interface with corresponding husky object.
    
    End effectors can now be specified at creation time using the ee_types parameter:
    - For single-arm robots: ee_types=["victor_gripper"] or ee_types=["robotiq_gripper"] or ee_types=["custom_gripper"]
    - For dual-arm robots: ee_types=["victor_gripper", "victor_gripper"] or ee_types=["robotiq_gripper", "custom_gripper"]
    - For calibration: set calibration=True (automatically uses calib_tip)
    """
    def __init__(self, monitor, name, mocap_id=None, pos=np.zeros(3), rot=np.array((0, 0, 0, 1)), 
                 connect_arm=True, connect_gripper=True, base_calibration_file=None, calibration=False, dual_arm=False, ee_types=None):
        self.name = name
        self.mocap_id = mocap_id
        self.interface = HuskyRobotInterface(monitor, 
                                             name, 
                                             use_odom=(mocap_id is None), 
                                             connect_arm=connect_arm, 
                                             connect_gripper=connect_gripper, 
                                             dual_arm=dual_arm
                                             )
        self.object = HuskyObject(calibration=calibration, dual_arm=dual_arm, ee_types=ee_types)
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
    """
    Collection of pybullet objects representing a husky.
    
    End effectors are now created and attached during initialization based on the ee_types parameter.
    This makes it easier to specify different end effectors for different robots at the high level.
    """
    def __init__(self, calibration=False, dual_arm=False, ee_types=None):
        with pp.LockRenderer():
            with pp.HideOutput():
                robot = load_robot(dual_arm=dual_arm)
                self.robot = robot
                self.dual_arm = dual_arm
                
                # Handle end effectors
                if ee_types is None:
                    if calibration:
                        ee_types = ["calib_tip"]
                    else:
                        ee_types = ["victor_gripper"]
                
                if dual_arm:
                    if len(ee_types) == 1:
                        ee_types = [ee_types[0], ee_types[0]]  # Use same type for both arms
                
                ee_list = []
                for ee_type in ee_types:
                    if ee_type == "calib_tip":
                        ee = create_end_effector(load_calib_tip=True, dual_arm=dual_arm)
                    else:
                        ee = create_end_effector(ee_type=ee_type, dual_arm=dual_arm)
                    ee_list.append(ee)
                
                self.ee_list = attach_end_effectors(robot, ee_list, dual_arm=dual_arm)
                self.old_color = None
       
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