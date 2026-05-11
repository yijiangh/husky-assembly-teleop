"""
A collection of common functions and classes used in the husky_assembly_teleop package.
"""

import os
import numpy as np
import pybullet as p
import json

import pybullet_planning as pp

from husky_assembly_teleop import DATA_DIRECTORY, DESIGN_DATA_DIRECTORY
from husky_assembly_teleop.husky_robot import HuskyRobotInterface
from husky_assembly_teleop.utils import UR5E_JOINT_NAMES

# --- --- UI BACKEND HOOK --- ---
# set by HuskyMonitor.__init__ before any widget is created.
_global_backend = None  # type: Optional['ui_backend.UIBackend']


def _backend():
    """Lazy accessor; raises if used before the monitor sets it up."""
    if _global_backend is None:
        raise RuntimeError(
            "_global_backend is None - Button/Slider used before "
            "HuskyMonitor.__init__ initialized the UI backend."
        )
    return _global_backend

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
#
# Tool / robot URDF spawning for COLLISION / PLANNING is now handled by
# `cfab_session.CfabSession` via `planner.set_robot_cell(robot_cell)`. This
# module retains a separate pp-based husky scene for visualization only;
# the pp scene uses simple gripper geometry instead of the per-problem
# tool URDFs.

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
        robot_urdf = os.path.join(DATA_DIRECTORY,'husky_urdf/mt_husky_dual_ur5_e_moveit_config/urdf/husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf')
    else:
        # INSERT_YOUR_CODE
        print("WARNING: Loading uncalibrated URDF for the single arm Husky robot.")
        robot_urdf = os.path.join(DATA_DIRECTORY,'husky_urdf/mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint_Alice_Calibrated.urdf')

    assert os.path.exists(robot_urdf)
    robot = pp.load_pybullet(robot_urdf, fixed_base=False, cylinder=False)
    
    return robot

def create_end_effector(ee_type="victor_gripper", load_calib_tip=False, dual_arm=False, force_regenerate=False, punch_tool_offset=None):
    """
    Create end effector for the pp visualization scene only.

    For COLLISION / PLANNING, tool geometry is now spawned by the cfab
    PyBullet client via `RobotCell.tool_models` / `RigidBodyState
    .attached_to_link`. This pp-side helper produces a lightweight
    visualization-only proxy.

    Args:
        ee_type: One of "victor_gripper" | "robotiq_gripper" |
            "custom_gripper" | "punch_tool" | "calib_tip".
        load_calib_tip: Whether to load calibration tip (overrides ee_type)
        dual_arm: Whether this is for a dual-arm robot (unused; kept for
            backwards compat with callers).
        force_regenerate: Unused (kept for backwards compat).
        punch_tool_offset: numpy array [x, y, z] offset from tool0 to
            punch tip (only used for punch_tool).

    Returns:
        ee: PyBullet end effector body ID.
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
    elif ee_type == "punch_tool":
        # Punch tool for calibration validation: cone with tip at punch offset, base at tool0
        import math
        if punch_tool_offset is not None:
            cone_height = float(np.linalg.norm(punch_tool_offset))
        else:
            cone_height = 0.15  # fallback
        cone_radius = 0.015  # 15mm base radius
        num_segments = 12

        vertices = []
        indices = []
        # Apex (tip) at punch tip position relative to tool0
        if punch_tool_offset is not None:
            vertices.append([float(punch_tool_offset[0]), float(punch_tool_offset[1]), float(punch_tool_offset[2])])
        else:
            vertices.append([0.0, 0.0, cone_height])
        # Base circle vertices at z=0 (tool0 attachment point)
        for i in range(num_segments):
            angle = 2.0 * math.pi * i / num_segments
            x = cone_radius * math.cos(angle)
            y = cone_radius * math.sin(angle)
            vertices.append([x, y, 0.0])
        # Side faces from apex to base ring
        for i in range(num_segments):
            next_i = (i + 1) % num_segments
            indices.extend([0, i + 1, next_i + 1])
        # Base cap
        base_center_idx = len(vertices)
        vertices.append([0.0, 0.0, 0.0])
        for i in range(num_segments):
            next_i = (i + 1) % num_segments
            indices.extend([base_center_idx, next_i + 1, i + 1])

        col_shape = p.createCollisionShape(
            p.GEOM_MESH, vertices=vertices, indices=indices,
            physicsClientId=pp.CLIENT
        )
        vis_shape = p.createVisualShape(
            p.GEOM_MESH, vertices=vertices, indices=indices,
            rgbaColor=[0.8, 0.2, 0.2, 0.8],
            physicsClientId=pp.CLIENT
        )
        ee = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=col_shape,
            baseVisualShapeIndex=vis_shape,
            physicsClientId=pp.CLIENT
        )
        return ee

    elif ee_type == "custom_gripper":
        # Example of adding a new end effector type
        # You can load from URDF, OBJ, or create a simple geometric shape
        custom_gripper_path = os.path.join(DATA_DIRECTORY, 'custom_gripper_description/urdf/custom_gripper.urdf')
        if os.path.exists(custom_gripper_path):
            # Load from URDF if available
            ee = pp.load_pybullet(custom_gripper_path, fixed_base=False, cylinder=False)
        else:
            # Fallback to simple geometric shape
            # ee = pp.create_cylinder(radius=0.05, height=0.15, color=(0.8, 0.8, 0.8, 1))
            ee = pp.create_box(0.12, 0.12, 0.01, color=(0.8, 0.8, 0.8, 1))
        return ee
    else:
        raise ValueError(f"Unknown end effector type: {ee_type}. Valid types: victor_gripper, robotiq_gripper, custom_gripper, punch_tool, calib_tip")

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
    
    End effectors are visualization proxies for the pp scene only; the
    planner side uses the cfab client + RobotCell tool_models.

    - For single-arm robots: ee_types=["victor_gripper"|"robotiq_gripper"|"custom_gripper"]
    - For dual-arm robots: ee_types=["victor_gripper", "victor_gripper"] (etc.)
    - For calibration: set calibration=True (automatically uses calib_tip)
    """
    def __init__(self, monitor, name, mocap_id=None, pos=np.zeros(3), rot=np.array((0, 0, 0, 1)),
                 connect_arm=True, connect_gripper=True, base_calibration_file=None, calibration=False, dual_arm=False, ee_types=None, force_regenerate=False, punch_tool_offset=None):
        self.name = name
        self.mocap_id = mocap_id
        self.interface = HuskyRobotInterface(monitor,
                                             name,
                                             use_odom=(mocap_id is None),
                                             connect_arm=connect_arm,
                                             connect_gripper=connect_gripper,
                                             dual_arm=dual_arm
                                             )
        self.object = HuskyObject(calibration=calibration, dual_arm=dual_arm, ee_types=ee_types, force_regenerate=force_regenerate, punch_tool_offset=punch_tool_offset)
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
    def __init__(self, calibration=False, dual_arm=False, ee_types=None, force_regenerate=False, punch_tool_offset=None):
        with pp.LockRenderer(False):
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
                per_arm_punch_offsets = None
                if dual_arm and isinstance(punch_tool_offset, (list, tuple)) and len(punch_tool_offset) == 2:
                    candidate_offsets = list(punch_tool_offset)
                    if all(hasattr(offset, '__len__') and len(offset) == 3 for offset in candidate_offsets):
                        per_arm_punch_offsets = candidate_offsets

                for ee_index, ee_type in enumerate(ee_types):
                    ee_punch_tool_offset = punch_tool_offset
                    if per_arm_punch_offsets is not None and ee_type == "punch_tool":
                        ee_punch_tool_offset = per_arm_punch_offsets[ee_index]

                    if ee_type == "calib_tip":
                        ee = create_end_effector(load_calib_tip=True, dual_arm=dual_arm)
                    else:
                        ee = create_end_effector(
                            ee_type=ee_type,
                            dual_arm=dual_arm,
                            force_regenerate=force_regenerate,
                            punch_tool_offset=ee_punch_tool_offset,
                        )
                    ee_list.append(ee)

                if dual_arm:
                    assert len(ee_list) == 2, f"Expected 2 end effectors for dual_arm, got {len(ee_list)}"
                else:
                    assert len(ee_list) == 1, f"Expected 1 end effector for single arm, got {len(ee_list)}"

                self.ee_types = ee_types
                self.ee_list = attach_end_effectors(robot, ee_list, dual_arm=dual_arm)
                self.old_color = None
       
    def set_pose(self, base_pose, arm_joint_states, index=0):
        """Set pose of base and ur5e arm(s). arm_joint_states must be of shape [[joint_values]] or [[left_joints], [right_joints]]"""        
        pp.set_pose(self.robot, base_pose)
        
        if len(arm_joint_states) == 0:
            raise ValueError(f'set_pose arm_joint_states is empty! {arm_joint_states}')
        elif len(arm_joint_states) == 1:
            # Single arm case - update arm at index 0
            if len(arm_joint_states[0]) > 0:
                arm_joints = pp.joints_from_names(self.robot, self.get_arm_joint_names(index=0))
                pp.set_joint_positions(self.robot, arm_joints, arm_joint_states[0])
            else:
                raise ValueError(f'set_pose arm_joint_states[0] is empty! {arm_joint_states}')
        elif len(arm_joint_states) == 2:
            # Dual arm case - update each arm independently if state is non-empty
            if len(arm_joint_states[0]) > 0:
                arm_joints = pp.joints_from_names(self.robot, self.get_arm_joint_names(index=0))
                pp.set_joint_positions(self.robot, arm_joints, arm_joint_states[0])
            
            if len(arm_joint_states[1]) > 0:
                if self.dual_arm:
                    arm_joints = pp.joints_from_names(self.robot, self.get_arm_joint_names(index=1))
                    pp.set_joint_positions(self.robot, arm_joints, arm_joint_states[1])
                else:
                    raise ValueError(f'Received second arm state but dual_arm is False! {arm_joint_states}')
        else:
            raise ValueError(f'set_pose arm_joint_states has invalid shape (>2 arms)! {arm_joint_states}')
       
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
        self.name = name
        self.action = action
        self._handle = _backend().add_button(name, action)

    def update(self):
        # In DPG mode this is a no-op (callback already fired). In PyBullet
        # legacy mode, this polls the debug param and dispatches on toggle.
        _backend().poll(self._handle, "button", self.action)

class Slider:
    def __init__(self, name, action, min_val=0.0, max_val=1.0, current_val=0.0, *,
                 integer=False):
        self.name = name
        self.action = action
        self.integer = integer
        be = _backend()
        if integer:
            self._handle = be.add_slider_int(name, int(round(min_val)),
                                             int(round(max_val)),
                                             int(round(current_val)), action)
            self._kind = "slider_int"
        else:
            self._handle = be.add_slider_float(name, min_val, max_val, current_val, action)
            self._kind = "slider_float"

    def update(self):
        _backend().poll(self._handle, self._kind, self.action)

class SliderGroup:
    def __init__(self, names, action, min_vals, max_vals, current_vals):
        self.names = names
        self.action = action
        be = _backend()
        # add_slider_group returns a list; we keep the first handle for poll dispatch
        # (both backends route the whole group through one composite poll).
        handles = be.add_slider_group(names, min_vals, max_vals, current_vals, action)
        self._handle = handles[0] if handles else None

    def update(self):
        if self._handle is None:
            return
        _backend().poll(self._handle, "slider_group", self.action)


class Toggle:
    """Boolean checkbox (DPG-only - degrades to a 0..1 slider in legacy mode)."""
    def __init__(self, name, action, current=False):
        self.name = name
        self.action = action
        self._handle = _backend().add_checkbox(name, bool(current), action)

    def update(self):
        _backend().poll(self._handle, "checkbox", self.action)


class Dropdown:
    """Select-one combo box. action receives the selected index (int)."""
    def __init__(self, name, action, options, current=0):
        self.name = name
        self.action = action
        self.options = list(options)
        self._handle = _backend().add_combo(name, self.options, int(current), action)

    def update(self):
        _backend().poll(self._handle, "combo", self.action)


class TextInput:
    """Free-form text or numeric entry. action(str) for text, action(float) if numeric=True."""
    def __init__(self, name, action, default="", *, numeric=False):
        self.name = name
        self.action = action
        self._handle = _backend().add_text_input(name, default, action, numeric=numeric)

    def update(self):
        _backend().poll(self._handle, "text_input", self.action)


class FilePicker:
    """Click-to-pick file/folder. action(absolute_path: str). Optional base_dir / ext_filter."""
    def __init__(self, name, action, *, base_dir=None, ext_filter=None):
        self.name = name
        self.action = action
        self._handle = _backend().add_file_dialog(
            name, action, base_dir=base_dir, ext_filter=ext_filter)

    def update(self):
        _backend().poll(self._handle, "file_dialog", self.action)


class LivePlot:
    """Streaming line plot. source() is polled each backend step()."""
    def __init__(self, name, source, history=200):
        self.name = name
        self.source = source
        self._handle = _backend().add_live_plot(name, source, history=history)

    def update(self):
        # plot is ticked inside backend.step(); nothing to do here
        pass


class Group:
    """Context manager that groups widgets into a (collapsible) section.

    Usage:
        with Group("State Loading"):
            Button(...)
            Slider(...)
    """
    def __init__(self, label, *, collapsible=True):
        self.label = label
        self.collapsible = collapsible

    def __enter__(self):
        _backend().begin_group(self.label, collapsible=self.collapsible)
        return self

    def __exit__(self, exc_type, exc, tb):
        _backend().end_group()
        return False  # don't suppress exceptions


class Separator:
    """Decorative section divider (replaces the legacy 'dump_sep_sliders' pattern).

    Use this instead of `Slider("----------XYZ", lambda: None, 0, 0, 0)`.
    """
    def __init__(self, label):
        self._handle = _backend().add_separator(label)

    def update(self):
        _backend().poll(self._handle, "separator", lambda *_: None)

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
