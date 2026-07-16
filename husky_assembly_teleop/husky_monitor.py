"""
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
import threading
import json
import csv
import yaml
from datetime import datetime
from types import SimpleNamespace
import numpy as np

from typing import List, Tuple
from scipy.spatial.transform import Rotation as R

import rclpy
import rclpy.executors
from rclpy.node import Node

import pybullet as p
import pybullet_planning as pp

from husky_assembly_teleop import DATA_DIRECTORY, DESIGN_DATA_DIRECTORY, CALIBRATION_BATCHES, DESIGN_PROBLEM_NAME, CALIBRATION_DATE
import husky_assembly_teleop.husky_world as world
from husky_assembly_teleop.husky_world import _solve_bar_action_goal_ik
import husky_assembly_teleop.mocap_experiment as mocap_experiment
from husky_assembly_teleop.mocap_experiment import (
    fit_bar_from_markerset, bar_deviation_from_goal, draw_marker_take_in_pp,
)
from husky_assembly_teleop.husky_robot import UR5e_HOME_STATE
from husky_assembly_teleop.common import (
    Button, Slider, SliderGroup, Separator, LiveMultiPlot, Husky, TrackedObject, HuskyObject, AssemblyObject, HUSKY_UR5e_JOINT_NAMES, lerp, load_gripper
)
from husky_assembly_teleop.optitrack.NatNetClient import NatNetClient
from husky_assembly_teleop.utils import (
    pose_from_frame, frame_from_pose, pose_from_transformation, transformation_from_pose,
    mocap_pos_y_up_to_z_up, mocap_quat_y_up_to_z_up,
    vec12_from_conf, conf_from_12vec, conf_from_6vec,
    joint_trajectory_from_path, path_12_from_joint_trajectory,
    HUSKY_DUAL_ARM_HOME_CONF_12, HUSKY_DUAL_UR5e_JOINT_NAMES, MOCAP_SET_RIG_RB_NAME,
)

# BarAction (gdrive design-study) loading
from husky_assembly_teleop.bar_action_io import (
    parse_bar_action, list_bar_actions, find_movement,
)
from husky_assembly_teleop.cfab_session import (
    CfabSession, build_default_robot_cell, plan_free_motion,
    arm_joint_names_for_group, SINGLE_ARM_GROUP,
    HUSKY_DUAL_URDF_PATH, HUSKY_DUAL_SRDF_PATH,
)
from husky_assembly_teleop import common as _common
from husky_assembly_teleop.ui_backend import make_backend, DearPyGuiBackend, bind_default_font

from compas.data import json_load, json_dump
from compas.geometry import Frame, Transformation
from compas_fab.backends import CollisionCheckError
from compas_fab.robots import JointTrajectory, JointTrajectoryPoint
from compas_fab.robots.time_ import Duration
from compas_robots import Configuration
from compas_robots.model import Joint

# TAMP motion-planner API. Safe to import at module top: no import-time
# side effects and no circular dependency back into this package.
from husky_assembly_tamp.motion_planner.api import (
    plan_free_dual_arm, plan_constrained_dual_arm, plan_constrained_dual_arm_linear,
    plan_dual_arm_linear_independent, _fk_link_frame, _collect_obstacle_puids,
)

DEFAULT_GREY = [0.2, 0.2, 0.2, 0.7]
GOAL_BLUE = [0, 0.2, 0.5, 0.7]
TRAJECTORY_GREEN = [0, 0.5, 0.2, 0.7]
TRANSPARENT = [0, 0.0, 0.0, 0.0]

# M1 constrained-planner resolutions. Single source for both the plan call
# and the CDFM sparse validation that re-checks the planned path.
M1_POSITION_RES = 0.01   # meters
M1_ROTATION_RES = 0.025  # radians
# Which planning stage the M1 constrained planner runs. Stage 3 is the
# grasped-bar transport stage (see STAGE3_GRASP_MASK_LINKS). This used to be
# adjustable via a "Constrained Stage" GUI slider, but in practice only stage 3
# was ever used, so it is now a fixed constant.
M1_PLANNER_STAGE = 3

EXISTING_ELEMENT_COLOR = pp.RED
CURRENT_ELEMENT_COLOR = pp.BLUE
DEFAULT_BAR_POS = pp.Point(0.8, 0, 1.3)

CLIENT_IP = '192.168.0.25' # Set to your own IP
MOCAP_IP = '192.168.0.117' # set to the mocap PC's IP, get this from Motive Settings>Streaming pane->Local interface
# Where the 'collect cameras data' button drops its JSON+CSV (gdrive folder also
# holding import_mocap_cameras_rhino.py).
MOCAP_CAMERA_EXPORT_DIR = (
    "/home/su/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/"
    "2025-03 Husky Assembly/data_experiment/visualise_mocap_camera"
)

# Folder under DESIGN_DATA_DIRECTORY (gdrive)/<...>/RobotCellStates/
# from which CALIBRATION-mode state + trajectory loaders pull files.
# Keyed by selected_arm_index (0=left, 1=right); see _calibration_state_dir().
# Full design-study archive lives on GitHub: yijiangh/husky_assembly_design_study.
CALIBRATION_STATE_SETS = {
    0: '260630_calib_trajs_Alice',              # left arm & single arm
    1: '260225_extrinsic_calib_trajs_Cindy_Right',  # right arm for Cindy
}

class HuskyMonitor(Node):
    USE_MOCAP = 0
    FAKE_HARDWARE = 1

    # * Set 0 to skip connecting the UR SetIO service clients (gripper/screw IO).
    # Saves the 2.5 s startup wait + "SetIO Service i not available!" warning
    # when io_and_status_controller isn't running. set_screw() then just logs
    # an "Invalid arm index" error instead of calling the service.
    CONNECT_IO_SERVICES = 0
    # * Set 0 to skip querying controller_manager/list_controllers on startup.
    # Saves the 2.5 s per-arm wait + "list_controllers service unavailable"
    # warning; active_controller stays "" (first switch_controller request may
    # then be rejected by controller_manager, see _seed_active_controllers).
    LIST_CONTROLLER_SERVICES = 0

    # When USE_MOCAP=1, by default the husky base in PyBullet tracks mocap.
    # Set USE_CELL_STATE_BASE_POSE=1 to override that and pin the base to
    # whatever was loaded from the goal RobotCellState's robot_base_frame
    # (or set via sliders). Useful for testing planning with mocap on for
    # end-effector tracking but the husky physically far from the assembly
    # scaffolding (e.g., at the lab desk during dual-arm accuracy tests).
    USE_CELL_STATE_BASE_POSE = 0
    USE_DPG_UI = 1   # 0 = legacy PyBullet debug GUI; 1 = Dear PyGui control panel
    UI_FONT_SIZE = 26  # base size for all DPG widgets (separators override to 20 in the backend)

    CALIBRATION = 0

    BAR_ACTION_LIVE_REPLAN_EXE = 1      # show Load BarAction / Load Movement / replan buttons
    BAR_ACTION_MOCAP_ACCURACY_TEST = 1  # show Record + Fit + Viz / Save markerset data
    DUAL_ARM_EE_CONSTR_ACCURACY_MOCAP_TEST = 0

    # =========================================================================
    # MOCK LIVE POSE FOR REPLAN (temporary; remove when real mocap + robot
    # are available and Button 2 has been validated end-to-end on hardware).
    # =========================================================================
    # When set to 1, `replan_free_to_movement_start_live` temporarily patches
    # `huskies[0].interface` for the duration of the Button 2 call so the
    # method sees a synthetic "live" pose.
    #
    # `MOCK_LIVE_ARM_CONF` picks the arm-conf source:
    #   'perturb': current_movement.start_state.robot_configuration + small
    #              random joint noise (default; represents the realistic
    #              operator scenario -- robot slightly off from the
    #              movement start, needing a short IK re-projection and
    #              short free-motion plan. Composite BiRRT can solve this).
    #   'home'  : HUSKY_DUAL_ARM_HOME_CONF_12 (the M4 dispatcher's home
    #              target -- represents the "robot parked between
    #              BarActions" case. Stress test: IK converges via the
    #              fallback branch, but the 12-DOF free plan from home
    #              extended arms to bar-holding grip is a genuinely hard
    #              corridor problem the sampler often can't solve.)
    #
    # `MOCK_LIVE_BASE_XY_OFFSET_M` is added (metres) to
    # current_movement.start_state.robot_base_frame's XY position to stand
    # in for real-world mocap drift.
    #
    # The interface is restored right after Button 2 returns so no other
    # code path sees the mock values. Toggle back to 0 once the live
    # mocap/robot pipeline is available.
    # =========================================================================
    MOCK_LIVE_POSE_FOR_REPLAN = 0
    MOCK_LIVE_ARM_CONF = 'perturb'
    MOCK_LIVE_ARM_PERTURB_STD_RAD = 0.02

    # Temporary: when 1, the live M2/M3 replan button
    # (`replan_free_to_movement_start_live`) relaxes the composite free-motion
    # plan's collision checking to robot self-collision (CC.1) ONLY -- also
    # skipping robot<->tool (CC.2) and environment checks (CC.3/4/5). Set back
    # to 0 to plan against tools + the full environment before running paths
    # on real hardware.
    REPLAN_SKIP_ENV_COLLISIONS_IN_MOTION_PLAN = 0
    MOCK_LIVE_ARM_PERTURB_MAX_TRIES = 10
    MOCK_LIVE_BASE_XY_OFFSET_M = (-0.3, 0.2)

    # Mocap (y-up) -> z-up axis convention. See utils.mocap_pos_y_up_to_z_up.
    # 'rhino'   : rhino_x = mocap_x, rhino_y = -mocap_z, rhino_z = mocap_y (preferred).
    # 'rotated' : legacy convention previously hardcoded in receive_*_frame.
    MOCAP_AXIS_CONVENTION = "rhino"

    PUNCH_CALIB_VALIDATION = 0

    DUAL_ARM_KISSING_REP_EXPERIMENT = 0 # set 1 to enable kissing experiment + compliance controller buttons

    # When 1, HuskyRobotInterface creates the compliant-controller ROS interfaces
    # (target_wrench publishers, start_force_mode / zero_ftsensor / switch_controller
    # service clients). Off by default so we don't block startup waiting on
    # services that aren't running on most rigs.
    CONNECT_COMPLIANT_CONTROLLER = 0

    def __init__(self):
        super().__init__('husky_monitor')
        self.tick_timer = self.create_timer(0.05, self.update)

        # simple async tasks to be executed every tick
        self.tasks = []

        # Marks this instance as the live ROS-driven monitor (vs. a headless
        # test harness that bypasses __init__). Headless flows skip
        # _hide_cfab_robot since there's no overlapping pp-side husky.
        self._is_live_monitor = True

        self.huskies = []
        self.tracked_objects = []
        self.name_from_mocap_id = {}
        self._mocap_cache_lock = threading.Lock()
        self._mocap_rigidbody_cache = {}
        self._mocap_rigidbody_id_from_name = {}
        self._mocap_labeled_marker_cache = defaultdict(dict)
        self.mocap_experiment_recording = None
        self.mocap_experiment_last_output_path = None

        # Legacy pp-side scene state (used by free trajectory / calibration
        # code paths). The BarAction flow does NOT populate this; collision
        # checking for planning goes through monitor.cfab.planner.
        self.static_obstacles = {}
        self.active_bar_body = None       # legacy pp body; None on BarAction path
        self.active_bar_aabb_dims = None  # cached from rs RigidBody mesh on BarAction path
        self.active_bar_name = None
        self.active_extra_bodies = []     # legacy
        self.bar_from_extra = []          # legacy

        # BarAction / cfab planning state.
        self.cfab = None                       # CfabSession (default cell at startup; per-problem on BarAction load)
        self.cfab_default_state = None         # default RobotCellState from build_default_robot_cell
        self.current_action = None             # rs_data_structure BarAssemblyAction
        self.current_movement = None           # selected Movement
        self.current_movement_index = None     # int
        self.movement_start_state = None       # compas_fab RobotCellState
        self.target_ee_frames = None           # {"left": Frame, "right": Frame} | None
        self.grasp_link_from_bar = None        # compas.geometry.Frame
        self.staging_free_trajectory = [None, None]   # left, right (per-arm tuples)
        self.constrained_trajectory = [None, None]
        self.constrained_display_mode = 0  # 0=FREE_STAGE, 1=CONSTRAINED
        self.constrained_start_conf = None  # 12-DOF target for manual staging
        self.constrained_goal_conf = None   # 12-DOF constrained-plan endpoint
        # cfab→pp bridge state for the BarAction planning path.
        self._bar_action_husky = None          # SimpleNamespace husky stub (cfab robot)
        self._bar_action_ghost_bodies = set()  # tiny invisible EE proxy pybullet bodies
        self._bar_action_cfab_id = None        # cfab client_id the ghosts belong to
        self._trajectory_waypoint_sliders = None          # cached waypoint-slider state (see _build_trajectory_waypoint_sliders)
        self.assembly_objects = []
        self.current_seq_index = 0

        self.calibration_data = []
        self.marker_set_data = []
        self.dual_arm_EE_mocap_data = []
        self._bar_holding_fit_line_uids = []
        self.goal_base_pose_frozen = False
        self._current_action_path = None

        # Per-movement BarAction loader (replaces single-movement load_bar_action).
        self._loaded_action = None              # BarAssemblyAction | None
        self._loaded_movements = []             # list[Movement]; M0..M4 straight from the JSON
        self._selected_action_file_idx = 0
        self._selected_movement_idx = 0
        self._ee_target_pose_uids = []          # pp.add_line uids for drawn EE targets
        # Per-movement attached-body ghosts. The bodies are the ones cfab
        # already spawned via set_robot_cell_state; we just re-color them
        # TRAJECTORY_GREEN and re-pose them via goal_model FK each tick so
        # they ride along the trajectory preview.
        self._traj_ghost_bodies = []            # list[{'body','link','attach'}]
        self._traj_ghost_orig_colors = {}       # body puid -> RGBA (for restore)

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
        self.data_collection_mode_slider = None
        self.data_collection_mode = True  # True = data collection mode, False = validation mode
        self.calib_batch_slider = None
        self.selected_calib_batch_index = 0

        self.selected_robot_id = 0
        
        # Board validation mode variables
        self.board_validation_state_slider = None
        self.trajectory_selection_slider = None
        self.available_bar_actions = []
        self.selected_state_index = 1
        self.available_joint_trajectories = []  # Store available JointTrajectory files
        self.selected_trajectory_index = 0

        # CALIBRATION-mode state/trajectory loaders (RobotCellState +
        # JointTrajectory files under DESIGN_DATA_DIRECTORY (gdrive)/
        # <CALIBRATION_STATE_SET>/RobotCellStates/).
        self.calibration_state_slider = None
        self.calibration_trajectory_slider = None
        self.available_calibration_states = []
        self.selected_calibration_state_index = 0
        self.available_calibration_trajectories = []
        self.selected_calibration_trajectory_index = 0
        

        # goal and trajectory interface
        self.selected_arm_index = 0
        
        # Punch tool calibration validation
        default_punch_tool_offset = np.array([0.0, 0.0, 0.15], dtype=float)
        self.punch_tool_offsets = {
            0: default_punch_tool_offset.copy(),
            1: default_punch_tool_offset.copy(),
        }
        self.punch_tool_offset = self.punch_tool_offsets[self.selected_arm_index].copy()
        self.tool0_from_punch_tip = pp.Pose(point=self.punch_tool_offset)
        self.punch_validation_results = []

        self.goal_base_pose = (np.zeros(3), np.array([0, 0, 0, 1]))
        self.goal_gripper = 0.0
        self.gripper_slider = None
        self.goal_arm_pose = [np.zeros(6), np.zeros(6)]
        self.show_goal_state = True  

        self.goal_model = None
        self.goal_gripper_model = None

        self.base_from_goal_bar_pos = None
        self.world_from_goal_bar_euler = None
        self.goal_element = None 

        self.calib_tool_from_robot_arm_id = defaultdict(lambda: defaultdict(lambda: None))
        self.calib_joint_range = np.pi*2
        self.calib_target_axis = 0

        self.goal_bar_grasp = None
        self.grasp_distance = 0.0 # fixed for now
        self.goal_element_axis = 0

        self.trajectory_time_max = 90 # 20 if self.CALIBRATION else 30
        self.trajectory_time = self.trajectory_time_max
        self.traj_viz_time = 1.0  # trajectory preview scrub position (0..1)

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

        # Load punch tool config before world.init so cone dimensions match the offset
        if self.PUNCH_CALIB_VALIDATION:
            self._load_punch_tool_config()

        # Initialize the UI backend BEFORE world.init / build_ui creates any widgets.
        _common._global_backend = make_backend(
            use_dpg=bool(self.USE_DPG_UI),
            window_title="Husky Monitor",
            font_size=int(self.UI_FONT_SIZE),
        )

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
        if self.BAR_ACTION_LIVE_REPLAN_EXE:
            self.available_bar_actions = self._load_available_bar_actions()
            self.available_joint_trajectories = self._load_available_joint_trajectories()

        if self.CALIBRATION:
            self.available_calibration_states = self._load_available_calibration_states()
            self.available_calibration_trajectories = self._load_available_calibration_trajectories()

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

    def _get_selected_trajectory_filename_suffix(self) -> str:
        """
        Return a filesystem-friendly suffix derived from the currently selected joint trajectory filename.
        Example: "ext_calib_0806_J1_traj0_JointTrajectory.json" -> "ext_calib_0806_J1_traj0_JointTrajectory"
        """
        # Prefer a cached attribute if present (set when loading / selecting trajectories)
        selected = getattr(self, "selected_trajectory_file", None)
        if not selected and getattr(self, "available_joint_trajectories", None):
            try:
                selected = self.available_joint_trajectories[self.selected_trajectory_index]
            except Exception:
                selected = None

        if not selected:
            return ""

        # Remove extension and sanitize to avoid problematic characters in filenames
        base = os.path.splitext(os.path.basename(str(selected)))[0]
        sanitized = re.sub(r"[^A-Za-z0-9_.-]+", "_", base).strip("_")
        return sanitized

    def update_calib_batch_index(self, value):
        self.selected_calib_batch_index = int(np.clip(int(value), 0, len(CALIBRATION_BATCHES) - 1))

    @property
    def selected_calib_batch(self):
        return CALIBRATION_BATCHES[self.selected_calib_batch_index]

    def record_calibration_data(self):
        if self.data_collection_mode:
            # In data collection mode, use the selected trajectory filename as suffix
            filename_suffix = self._get_selected_trajectory_filename_suffix()
        else:
            # In validation mode, use "validation" as suffix
            filename_suffix = "validation"
        world.save_calibration(self, filename_suffix=filename_suffix,
                               date_folder=CALIBRATION_DATE,
                               data_batch=self.selected_calib_batch)
        self.calibration_data = []

    def record_markerset_data(self):
        world.save_markerset_data(self)
        self.marker_set_data = []
        
    def reset_ui(self, target_conf=None):
        # reset all sliders to default value by recreating them...
        # pybullet seems to lack a setUserDebugParameter() method :(
        # p.removeAllUserParameters()
        # Clear the ACTIVE backend's widgets (PyBullet params OR DPG widgets) so the
        # rebuild below doesn't stack a duplicate panel in DPG mode.
        if _common._global_backend is not None:
            _common._global_backend.clear()
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

    def update_selected_arm_id(self, arm_index):
        new_index = np.clip(int(arm_index), 0, self.get_active_arm_count() - 1)
        if new_index != self.selected_arm_index:
            self.selected_arm_index = new_index
            self._set_active_punch_tool_offset(new_index)
            if self.CALIBRATION:
                # Calib reference set is per-arm (_calibration_state_dir); reload
                # so the rebuilt sliders show the new arm's files.
                self.available_calibration_states = self._load_available_calibration_states()
                self.available_calibration_trajectories = self._load_available_calibration_trajectories()
                self.selected_calibration_state_index = 0
                self.selected_calibration_trajectory_index = 0
            self.reset_ui(target_conf=self.goal_arm_pose) #[self.selected_arm_index])

    def update_trajectory_time(self, time):
        self.trajectory_time = time

    def update_traj_viz_time(self, value):
        # Scrub position (0..1) for the trajectory preview; read in update().
        self.traj_viz_time = float(value)

    def update_calib_joint_range(self, value):
        self.calib_joint_range = value

    def update_calib_target_axis(self, value):
        self.calib_target_axis = int(np.floor(value))

    def update_data_collection_mode(self, value):
        """Update data collection mode: 0 = validation mode, 1 = data collection mode"""
        self.data_collection_mode = bool(round(value))

    @staticmethod
    def _arm_name_from_index(arm_index):
        return 'left' if int(arm_index) == 0 else 'right'

    def get_punch_tool_offset(self, arm_index=None):
        arm_index = self.selected_arm_index if arm_index is None else int(arm_index)
        return np.array(self.punch_tool_offsets[arm_index], dtype=float)

    def get_tool0_from_punch_tip(self, arm_index=None):
        return pp.Pose(point=self.get_punch_tool_offset(arm_index))

    def _set_active_punch_tool_offset(self, arm_index=None):
        self.punch_tool_offset = self.get_punch_tool_offset(arm_index)
        self.tool0_from_punch_tip = pp.Pose(point=self.punch_tool_offset)

    def get_active_arm_count(self):
        if self.huskies:
            return 2 if self.huskies[self.selected_robot_id].dual_arm else 1
        return 2

    # --- Joint live-stream plot (radians/degrees readout + scrolling record) ---
    def _joint_stream_source(self):
        """Flat list of the active robot's live joint angles, in radians.

        Returns 6 values for a single-arm robot, or 12 (left arm then right
        arm) for a dual-arm robot, matching _joint_stream_labels(). Per-arm
        order follows arm_joint_pose: pan, lift, elbow, wrist_1, wrist_2, wrist_3.

        Returns:
            list[float]: The live joint angles of the active robot in radians.
        """
        hi = self.huskies[self.selected_robot_id].interface
        values = []
        for arm in range(self.get_active_arm_count()):
            values.extend(float(q) for q in hi.arm_joint_pose[arm])
        return values

    def _joint_stream_labels(self):
        """Legend/readout labels lining up with _joint_stream_source().

        Short joint names, prefixed 'L '/'R ' per arm on a dual-arm robot.

        Returns:
            list[str]: One label per joint (6 single-arm, 12 dual-arm).
        """
        short = ['pan', 'lift', 'elbow', 'w1', 'w2', 'w3']
        if self.get_active_arm_count() == 2:
            return [f'{side} {name}' for side in ('L', 'R') for name in short]
        return list(short)

    def toggle_joint_live_stream(self):
        """Show or hide the live joint-angle stream (text readout + plot).

        The plot records continuously once built (in build_ui); this button only
        flips the section's visibility. Live plots need the Dear PyGui backend,
        so in PyBullet mode (USE_DPG_UI=0) this warns and does nothing.
        """
        if self.joint_stream_plot is None:
            self.get_logger().warn(
                "Joint live stream needs the Dear PyGui UI (set USE_DPG_UI=1).")
            return
        self._joint_stream_visible = not getattr(self, '_joint_stream_visible', False)
        self.joint_stream_plot.set_visible(self._joint_stream_visible)

    # --- Punch tool calibration validation ---
    def _load_punch_tool_config(self):
        """Load punch tool offset from config.yaml."""
        try:
            punch_config_path = os.path.join(
                DATA_DIRECTORY, 'calibration_data', CALIBRATION_DATE, 'config.yaml'
            )
            with open(punch_config_path, 'r') as f:
                config = yaml.safe_load(f) or {}

            punch_config = config.get('punch_tool') or {}
            updated_offsets = {
                arm_index: np.array(offset, dtype=float)
                for arm_index, offset in self.punch_tool_offsets.items()
            }

            legacy_offset = punch_config.get('offset_xyz')
            if legacy_offset is not None:
                legacy_offset = np.array(legacy_offset, dtype=float)
                updated_offsets = {
                    0: legacy_offset.copy(),
                    1: legacy_offset.copy(),
                }

            for arm_index, arm_name in enumerate(('left', 'right')):
                arm_config = punch_config.get(arm_name) or {}
                if 'offset_xyz' in arm_config:
                    updated_offsets[arm_index] = np.array(arm_config['offset_xyz'], dtype=float)

            self.punch_tool_offsets = updated_offsets
            self._set_active_punch_tool_offset(self.selected_arm_index)
            self.get_logger().info(
                'Loaded punch tool offsets: '
                f"left={self.punch_tool_offsets[0].tolist()}, "
                f"right={self.punch_tool_offsets[1].tolist()}"
            )
        except Exception as e:
            self.get_logger().warn(f'Failed to load punch tool config: {e}')

    def record_punch_reference_pose(self):
        """Record the current punch tip pose in world frame via FK."""
        world.record_punch_reference(self, date_folder=CALIBRATION_DATE)

    def save_punch_validation_data(self):
        """Save all accumulated punch validation results to JSON."""
        world.save_punch_validation_data(self, date_folder=CALIBRATION_DATE)

    def collect_mocap_camera_data(self):
        """Snapshot mocap camera poses (mocap-origin frame), convert to the Rhino
        z-up frame, and save JSON+CSV into the gdrive visualise_mocap_camera folder."""
        inventory = self.get_mocap_camera_inventory(refresh=True)
        if not inventory or not inventory.get('cameras'):
            self.get_logger().warn('No mocap cameras found (is mocap connected?)')
            return
        conv = self.MOCAP_AXIS_CONVENTION  # 'rhino' by default

        # List comprehension: build a new list by looping `for c in ...` and
        # producing one {dict} per camera. Equivalent to a for-loop that appends,
        # but shorter. Each camera's y-up mocap pose is converted to z-up here.
        cameras = [{
            'name': c['name'],
            'position': mocap_pos_y_up_to_z_up(c['position'], conv),
            'orientation': mocap_quat_y_up_to_z_up(c['orientation'], conv),
        } for c in inventory['cameras']]

        # exist_ok=True => don't error if the folder already exists (avoids a
        # try/except). strftime formats 'now' into a sortable timestamp string;
        # `stem` is the shared filename (no extension) for the .json and .csv.
        os.makedirs(MOCAP_CAMERA_EXPORT_DIR, exist_ok=True)
        stem = 'mocap_cameras_' + datetime.now().strftime('%Y%m%d_%H%M%S')
        json_path = os.path.join(MOCAP_CAMERA_EXPORT_DIR, stem + '.json')
        csv_path = os.path.join(MOCAP_CAMERA_EXPORT_DIR, stem + '.csv')

        # `with open(...) as f` is a context manager: it auto-closes the file even
        # if an error happens inside the block. json.dump writes a dict as JSON;
        # indent=2 pretty-prints it. The dict also stores metadata (frame, units)
        # so the file is self-describing.
        with open(json_path, 'w') as f:
            json.dump({'frame': 'mocap_origin', 'axis_convention': conv,
                       'position_units': 'meters', 'orientation': 'quaternion_xyzw',
                       'camera_count': len(cameras), 'cameras': cameras}, f, indent=2)

        # newline='' is the csv module's required idiom to stop blank rows on
        # Windows. The `*` is "unpacking": *c['position'] spreads the [x,y,z] list
        # into separate cells, so one row = name + 3 position + 4 quaternion cols.
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['name', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
            for c in cameras:
                w.writerow([c['name'], *c['position'], *c['orientation']])
        self.get_logger().info(
            f'Saved {len(cameras)} mocap cameras to {json_path}')

    def record_raw_mocap_take(self):
        if not self.USE_MOCAP:
            self.get_logger().warn('MoCap experiment recording requires USE_MOCAP.')
            return
        if not hasattr(self, 'mocap_client') or not self.mocap_client.connected():
            self.get_logger().warn('MoCap client is not connected.')
            return
        if self.mocap_experiment_recording is not None:
            self.get_logger().warn('A MoCap experiment take is already recording.')
            return

        try:
            config_path, config = mocap_experiment.load_experiment_config()
        except Exception as exc:
            self.get_logger().error(f'Failed to load MoCap experiment config: {exc}')
            return

        selected_husky = self.huskies[self.selected_robot_id]
        output_paths = mocap_experiment.prepare_take_output(config)
        self.mocap_experiment_recording = {
            'config_path': config_path,
            'config': config,
            'output_paths': output_paths,
            'target_rigid_body': selected_husky.name,
            'selected_robot_id': int(self.selected_robot_id),
            'wall_start_time': time.monotonic(),
            'frames': [],
            'rigid_body_ids': {},
            'auto_reference_images': [],
            'mocap_camera_inventory': self.get_mocap_camera_inventory(refresh=True),
            'webcam_timelapse': None,
        }

        webcam_asset = mocap_experiment.capture_workspace_webcam_image(config, output_paths)
        if webcam_asset is not None:
            self.mocap_experiment_recording['auto_reference_images'].append(webcam_asset)
            if webcam_asset.get('status') == 'captured':
                self.get_logger().info(
                    f"Captured workspace image to "
                    f"{os.path.join(output_paths['session_dir'], webcam_asset['session_relative_path'])}"
                )
            else:
                self.get_logger().warn(
                    f"Workspace webcam capture failed: {webcam_asset.get('reason', 'unknown_error')}"
                )

        webcam_timelapse = mocap_experiment.start_workspace_webcam_timelapse(config, output_paths)
        self.mocap_experiment_recording['webcam_timelapse'] = webcam_timelapse
        if webcam_timelapse is not None and webcam_timelapse.get('status') == 'capture_failed':
            self.get_logger().warn(
                f"Workspace webcam timelapse failed to start: {webcam_timelapse.get('reason', 'unknown_error')}"
            )

        self.get_logger().info(
            f"Started raw MoCap take for '{selected_husky.name}' "
            f"({config['experiment']['duration_sec']:.1f}s) using {config_path}"
        )

    def test_webcam_capture(self):
        try:
            config_path, config = mocap_experiment.load_experiment_config()
        except Exception as exc:
            self.get_logger().error(f'Failed to load MoCap experiment config: {exc}')
            return

        test_config = copy.deepcopy(config)
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        base_take_id = str(test_config.get('take', {}).get('take_id', '') or 'webcam_test')
        test_config['take']['take_id'] = f'{base_take_id}_webcam_test_{timestamp}'
        output_paths = mocap_experiment.prepare_take_output(test_config)
        webcam_asset = mocap_experiment.capture_workspace_webcam_image(test_config, output_paths)

        if webcam_asset is None:
            self.get_logger().warn('Webcam test capture is disabled in the current config.')
            return

        if webcam_asset.get('status') == 'captured':
            asset_path = os.path.join(output_paths['session_dir'], webcam_asset['session_relative_path'])
            self.get_logger().info(f'Webcam test capture saved to {asset_path}')
        else:
            self.get_logger().warn(
                f"Webcam test capture failed: {webcam_asset.get('reason', 'unknown_error')}"
            )

    def _record_raw_mocap_snapshot(self, timestamp, raw_snapshot, rigid_body_ids):
        recording = self.mocap_experiment_recording
        if recording is None:
            return

        elapsed_sec = time.monotonic() - recording['wall_start_time']
        frame_payload = {
            'timestamp': float(timestamp),
            'elapsed_sec': float(elapsed_sec),
            'rigid_bodies': {
                name: {
                    'position_m': [float(value) for value in pose[0]],
                    'quaternion_xyzw': [float(value) for value in pose[1]],
                }
                for name, pose in sorted(raw_snapshot.items())
            },
        }
        recording['frames'].append(frame_payload)
        recording['rigid_body_ids'].update({name: int(rb_id) for name, rb_id in rigid_body_ids.items()})
        recording['webcam_timelapse'] = mocap_experiment.step_workspace_webcam_timelapse(
            recording.get('webcam_timelapse'),
            elapsed_sec,
            recording['output_paths'],
        )

        if elapsed_sec >= recording['config']['experiment']['duration_sec']:
            self._finalize_raw_mocap_take(stop_reason='duration_elapsed')

    def _finalize_raw_mocap_take(self, stop_reason):
        recording = self.mocap_experiment_recording
        if recording is None:
            return

        webcam_timelapse_result = mocap_experiment.finalize_workspace_webcam_timelapse(
            recording.get('webcam_timelapse'),
            recording['output_paths'],
        )

        payload = mocap_experiment.build_take_payload(
            config=recording['config'],
            config_path=recording['config_path'],
            output_paths=recording['output_paths'],
            target_rigid_body=recording['target_rigid_body'],
            selected_robot_id=recording['selected_robot_id'],
            frames=recording['frames'],
            rigid_body_ids=recording['rigid_body_ids'],
            stop_reason=stop_reason,
            auto_reference_images=recording.get('auto_reference_images', []),
            mocap_camera_inventory=recording.get('mocap_camera_inventory'),
            webcam_timelapse=webcam_timelapse_result,
        )
        take_path = mocap_experiment.save_take_payload(
            payload=payload,
            take_path=recording['output_paths']['take_path'],
            manifest_path=recording['output_paths']['manifest_path'],
        )

        self.mocap_experiment_last_output_path = take_path
        self.mocap_experiment_recording = None
        if webcam_timelapse_result is not None and webcam_timelapse_result.get('status') == 'created':
            self.get_logger().info(
                f"Saved webcam timelapse to "
                f"{os.path.join(recording['output_paths']['session_dir'], webcam_timelapse_result['session_relative_path'])}"
            )
        self.get_logger().info(
            f"Saved raw MoCap take with {payload['frame_count']} frames to {take_path}"
        )

    def update_goal_align_axis(self, value):
        self.goal_element_axis = value

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
        # goal_arm_pose is always length 2 (per __init__); slice for single-arm goal_model.
        arm_pose = self.goal_arm_pose if self.goal_model.dual_arm else self.goal_arm_pose[:1]
        self.goal_model.set_pose(self.goal_base_pose, arm_pose)

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

                # Spread the waypoints over the requested trajectory time so fake
                # execution takes as long as the real robot would (mirrors the
                # real-hardware dt = traj_time / (n - 1) in husky_robot.py).
                step_dt = self.trajectory_time / max(len(trajectory[0]) - 1, 1)
                for conf in trajectory[0]:
                    hi.arm_joint_pose[self.selected_arm_index] = conf
                    ho.set_pose((hi.position, hi.rotation), hi.arm_joint_pose)

                    if trajectory[3] is not None:
                        # update attached object based on FK
                        world_from_tcp = ho.get_link_pose_from_name("ur_arm_tool0")
                        object_pose = pp.multiply(world_from_tcp, gripper_tcp_from_object)
                        obj.set_pose(object_pose)

                    hi.is_arm_executing = True
                    pp.wait_for_duration(step_dt)

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
        # if self.linear_arm_trajectory is None or self.free_arm_trajectory is None:
        #     self.get_logger().warn('Transit and calib trajectories must be planned before executing!')
        # else:
            # conf = self.planned_arm_trajectory[self.selected_arm_index][0].pop(0)
            # world.execute_arm_conf(self, conf, index=self.selected_arm_index)

        world.execute_arm_trajectory_and_record_each_conf(self, self.planned_arm_trajectory[self.selected_arm_index], index=self.selected_arm_index)
        self.record_calibration_data()

    def get_world_from_bar_goal_pose(self):
        world_from_base_link = self.goal_model.get_link_pose_from_name("base_footprint")
        world_pos = pp.multiply(world_from_base_link, pp.Pose(point=self.base_from_goal_bar_pos))[0]
        world_quat = pp.Pose(euler=pp.Euler(*self.world_from_goal_bar_euler))[1]
        return world_pos, world_quat

    def get_bar_action_goal_bar_pose(self):
        """world_from_bar from M2 cell state: target_ee_frames[side] ∘ attachment_frame.

        Returns ``(pos, quat)`` or ``None`` if no BarAction is loaded.
        """
        if self.movement_start_state is None or self.target_ee_frames is None:
            return None
        if not self.active_bar_name:
            return None
        rb_states = getattr(self.movement_start_state, 'rigid_body_states', {}) or {}
        bar_rb = rb_states.get(self.active_bar_name)
        if bar_rb is None or bar_rb.attachment_frame is None:
            return None
        attached_link = getattr(bar_rb, 'attached_to_link', '') or ''
        side = 'left' if 'left' in attached_link else 'right'
        target = self.target_ee_frames.get(side)
        if target is None:
            return None
        world_from_tool = pose_from_frame(target)
        tool_from_bar = pose_from_frame(bar_rb.attachment_frame)
        return pp.multiply(world_from_tool, tool_from_bar)

    def get_movement_start_bar_pose(self):
        """world_from_bar of the active bar in the current movement's start state.

        The mocap "bar holding accuracy" experiment drives the robot to a
        movement's start_state and measures the actual held-bar pose, so the
        reference we compare against is the bar's world pose *at that start
        state*. Two cases are handled:

        - Bar held by a gripper link: the start_state only stores the grasp
          (``attachment_frame``), so we forward-kinematics the holding link at
          the start configuration and compose the grasp onto it.
        - Bar resting in the world (installed / pre-pickup): the start_state
          stores the world ``frame`` directly, so we return that.

        Returns:
            tuple | None: ``(pos, quat_xyzw)`` of plain floats, or ``None``
            when the bar / movement / cfab session isn't available.
        """
        state = getattr(self, 'movement_start_state', None)
        bar_name = getattr(self, 'active_bar_name', None)
        if state is None or not bar_name or self.cfab is None:
            return None
        rb_states = getattr(state, 'rigid_body_states', {}) or {}
        bar_rb = rb_states.get(bar_name)
        if bar_rb is None:
            return None

        # Free-standing bar: its world frame is authored on the rigid body.
        if getattr(bar_rb, 'frame', None) is not None:
            pos, quat = pose_from_frame(bar_rb.frame)
            return ([float(v) for v in pos], [float(v) for v in quat])

        # Held bar: only the grasp frame is stored. Recover the world pose by
        # FK-ing the holding link at the start configuration, then composing
        # the grasp. The FK reads the cfab pybullet client, so pin pp.CLIENT to
        # it for the query and restore afterwards (the monitor's update loop
        # keeps pp.CLIENT on its own world) -- same swap the planners use.
        attach = getattr(bar_rb, 'attachment_frame', None)
        link = getattr(bar_rb, 'attached_to_link', None)
        if attach is None or not link:
            return None
        saved_client = pp.CLIENT
        pp.CLIENT = self.cfab.client.client_id
        pp.CLIENTS.setdefault(pp.CLIENT, True)
        try:
            world_from_link = _fk_link_frame(self.cfab.planner, state, link)
        except Exception as e:
            self.get_logger().warn(f"start-state bar pose FK failed: {e}")
            return None
        finally:
            pp.CLIENT = saved_client
        world_from_link_pose = (list(world_from_link.point),
                                list(world_from_link.quaternion.xyzw))
        tool_from_bar = pose_from_frame(attach)
        pos, quat = pp.multiply(world_from_link_pose, tool_from_bar)
        return ([float(v) for v in pos], [float(v) for v in quat])

    def update_constrained_display_mode(self, val):
        self.constrained_display_mode = int(round(float(val)))
        self._refresh_constrained_displayed_trajectory()

    def _refresh_constrained_displayed_trajectory(self):
        src = self.constrained_trajectory if self.constrained_display_mode == 1 \
              else self.staging_free_trajectory
        if src[0] is not None and src[1] is not None:
            self.set_arm_trajectory(src[0], index=0)
            self.set_arm_trajectory(src[1], index=1)
            self.set_to_show_traj_state()

    def _goal_matches_constrained_start(self):
        """True when the current goal is the staged start of the constrained path."""
        start_conf = getattr(self, "constrained_start_conf", None)
        if start_conf is None:
            return False
        goal_conf = np.concatenate([
            np.asarray(self.goal_arm_pose[0], dtype=float),
            np.asarray(self.goal_arm_pose[1], dtype=float),
        ])
        return np.allclose(goal_conf, np.asarray(start_conf, dtype=float), atol=1e-4)

    def _capture_manual_staging_plan(self, arm_index=None):
        """Cache manual free plans in display slot 0 when they target constrained start."""
        if not self._goal_matches_constrained_start():
            return

        if arm_index is None:
            if self.planned_arm_trajectory[0][0] is None or self.planned_arm_trajectory[1][0] is None:
                return
            self.staging_free_trajectory = [
                copy.deepcopy(self.planned_arm_trajectory[0]),
                copy.deepcopy(self.planned_arm_trajectory[1]),
            ]
            self.constrained_display_mode = 0
            print("Cached manual both-arm staging plan as Display Traj = 0.")
            return

        arm_index = int(arm_index)
        if self.planned_arm_trajectory[arm_index][0] is None:
            return
        self.staging_free_trajectory[arm_index] = copy.deepcopy(
            self.planned_arm_trajectory[arm_index]
        )
        self.constrained_display_mode = 0
        print(f"Cached manual arm {arm_index} staging plan as Display Traj = 0.")

    def _set_goal_to_constrained_start(self):
        """Restore manual staging target to the constrained trajectory start."""
        start_conf = getattr(self, "constrained_start_conf", None)
        if start_conf is None:
            return
        start_conf = np.asarray(start_conf, dtype=float)
        self.goal_arm_pose[0] = start_conf[:6].copy()
        self.goal_arm_pose[1] = start_conf[6:].copy()
        self.update_traj_goal_configuration()

    def sample_random_goal_conf(self, max_attempts=200):
        """Sample a collision-free random arm conf for the active husky and
        stage it as ``goal_arm_pose``. Auto-adapts to single/dual arm via
        ``HuskyObject.get_arm_joint_names`` + ``husky.dual_arm``."""
        husky = self.huskies[self.selected_robot_id]
        ho = husky.object
        robot = ho.robot
        if husky.dual_arm:
            arm_specs = [('left_', 0), ('right_', 1)]
            joint_names = list(ho.get_arm_joint_names(0)) + list(ho.get_arm_joint_names(1))
            attachments = [ho.ee_list[0][1], ho.ee_list[1][1]]
        else:
            arm_specs = [('', 0)]
            joint_names = list(ho.get_arm_joint_names(0))
            attachments = [ho.ee_list[0][1]]

        # ACM: wrist links vs mounted tool body. Mirrors plan_transit_motion's
        # extra_disabled_collisions logic (utils.py:233-272). Without these,
        # the tool body collides with its own mount link / nearby wrist links.
        ee_types = getattr(ho, "ee_types", None) or []
        extra_disabled_collisions = []
        for arm_prefix, idx in arm_specs:
            attach = attachments[idx]
            ee_type = ee_types[idx] if idx < len(ee_types) else None
            wrist_links = ['ur_arm_wrist_3_link']  # mount link
            if isinstance(ee_type, str):
                if ee_type.startswith('assembly_tool_v3'):
                    wrist_links += ['ur_arm_wrist_2_link', 'ur_arm_wrist_1_link']
                elif ee_type == 'robotiq_gripper':
                    wrist_links += ['ur_arm_wrist_1_link']
            for wl in wrist_links:
                extra_disabled_collisions.append(
                    ((robot, pp.link_from_name(robot, arm_prefix + wl)),
                     (attach.child, pp.BASE_LINK))
                )

        joints = pp.joints_from_names(robot, joint_names)
        obstacles = list(self.static_obstacles.values())
        sample_fn = pp.get_sample_fn(robot, joints)
        collision_fn = pp.get_collision_fn(
            robot, joints,
            obstacles=obstacles,
            attachments=attachments,
            self_collisions=1,
            extra_disabled_collisions=extra_disabled_collisions,
            max_distance=0,
        )
        with pp.WorldSaver():
            for attempt in range(max_attempts):
                q = sample_fn()
                if not collision_fn(q):
                    if husky.dual_arm:
                        self.goal_arm_pose[0] = np.array(q[:6])
                        self.goal_arm_pose[1] = np.array(q[6:])
                    else:
                        self.goal_arm_pose[0] = np.array(q)
                    self.update_traj_goal_configuration()
                    self.get_logger().info(
                        f"Sampled collision-free goal conf in {attempt+1} attempts."
                    )
                    return
        self.get_logger().warn(
            f"No collision-free goal conf in {max_attempts} attempts."
        )

    def plan_single_arm_to_goal_action(self):
        """Plan selected arm, then cache it as manual staging if applicable.

        Prefers the cfab-backed single-group planner (obstacles + ACM from
        the cell state); falls back to the legacy pp planner when no cfab
        session exists.
        """
        self._set_goal_to_constrained_start()
        if not self._plan_single_arm_with_cfab():
            world.plan_arm_to_goal(self)
        self._capture_manual_staging_plan(self.selected_arm_index)

    def _plan_single_arm_with_cfab(self):
        """cfab-backed single-arm free plan to goal_arm_pose[selected].

        Returns True when a trajectory was planned and stored; False when
        the cfab route is unavailable (caller falls back to pp planning).
        """
        if self.cfab is None or getattr(self.cfab, 'planner', None) is None:
            return False
        template = getattr(self, 'movement_start_state', None) \
            or getattr(self, 'cfab_default_state', None)
        if template is None:
            return False
        name_sets = self._arm_joint_name_sets()
        arm_idx = min(self.selected_arm_index, len(name_sets) - 1)
        groups = self.cfab.robot_cell.robot_semantics.groups
        if 'base_left_arm_manipulator' in groups:
            group = ('base_left_arm_manipulator', 'base_right_arm_manipulator')[arm_idx]
        else:
            group = SINGLE_ARM_GROUP
        state = template.copy()
        self._inject_live_conf_into_state(state)
        # * Pass a compas Configuration as the goal so cfab_session.plan_free_motion
        # takes its dict-style path — keeps the tamp-API contract uniform.
        # For single-arm husky (index 0), we always use the arm-index-0 joint names.
        goal6 = conf_from_6vec(
            np.asarray(self.goal_arm_pose[self.selected_arm_index], dtype=float),
            arm_index=arm_idx,
        )
        # Pause GUI rendering during the search (no-op when headless).
        with pp.LockRenderer():
            path, info = plan_free_motion(
                self.cfab.planner, state, goal6, group=group,
                max_time=30.0, max_iterations=100,
            )
        if path is None:
            self.get_logger().warn(
                f"[single-arm cfab] planning failed: {info.get('failure_reason')}; "
                "falling back to pp planner.")
            return False
        self.set_arm_trajectory(
            (np.asarray(path), None, self.trajectory_time, None),
            index=self.selected_arm_index)
        self.set_to_show_traj_state()
        print(f"[single-arm cfab] OK: group={group}, {len(path)} waypoints.")
        return True

    def plan_both_arms_to_goal_action(self, use_composite=True, debug=False):
        """Plan both arms, then cache it as manual staging if applicable."""
        self._set_goal_to_constrained_start()
        world.plan_both_arms_to_goal(self, use_composite=use_composite, debug=debug)
        self._capture_manual_staging_plan()

    def plan_free_to_movement_start_with_cfab_cc(self):
        """Free dual-arm plan from LIVE robot conf -> start_conf of the
        currently selected movement, with cfab collision checking.

        Analogous to plan_both_arms_to_goal_action (composite) but the goal
        is taken from mv.start_state.robot_configuration.
        """
        if self.current_movement is None:
            self.get_logger().warn("Load a movement first.")
            return
        mv = self.current_movement
        if mv.start_state is None or mv.start_state.robot_configuration is None:
            self.get_logger().warn(
                f"Movement {mv.movement_id!r} has no start_state.robot_configuration."
            )
            return
        # Goal = the movement's authored/planned start conf (read BEFORE the
        # live injection below overwrites it in the state copy). Pass the
        # compas Configuration directly so the tamp API's dict-indexed
        # extraction succeeds without falling back to sequence coercion.
        goal_conf = mv.start_state.robot_configuration
        # Plan against the LIVE husky base, not the BarAction-authored one.
        if not self._apply_live_base_to_movement(mv):
            return
        state = mv.start_state.copy()
        self._inject_live_conf_into_state(state)

        # Pause GUI rendering during the search (no-op when headless).
        with pp.LockRenderer():
            path, info = plan_free_dual_arm(
                self.cfab.planner, state, goal_conf,
                max_time=120.0, max_iterations=1000,
            )
        if path is None:
            self.get_logger().warn(
                f"plan_free→mv-start failed: {info.get('failure_reason', 'unknown')}"
            )
            return

        left_path = np.array([q[:6] for q in path])
        right_path = np.array([q[6:] for q in path])
        t = self.trajectory_time
        self.set_arm_trajectory((left_path, None, t, None), index=0)
        self.set_arm_trajectory((right_path, None, t, None), index=1)
        self.set_to_show_traj_state()
        print(f"[plan free→mv-start, cfab CC] OK: {mv.movement_id!r} "
              f"({len(path)} waypoints)")

    # --- --- --- --- --- BARACTION LOADING --- --- --- --- ---

    def load_bar_action(self, action_path=None, movement=0, *, update_goal_state=True):
        """Load one movement of a BarAssemblyAction via the cfab planner.

        Replaces the legacy ``load_board_validation_state`` flow. Scene
        materialization (rigid bodies, attached tool bodies, ACM) goes
        through ``self.cfab.planner.set_robot_cell_state(...)`` — no
        per-body pp spawning, no manual ACM translation.

        Parameters
        ----------
        action_path : str | None
            Absolute path or bare filename (resolved under
            ``DESIGN_DATA_DIRECTORY/<problem>/BarActions/``). If None, uses
            the slider-selected entry of ``available_bar_actions``.
        movement : int | str
            Integer index OR movement_id substring (e.g. ``"M1"``).
        update_goal_state : bool
            If True, refresh the UI's goal display after loading.

        Returns
        -------
        bool
            True on success, False otherwise.
        """
        # 1) Resolve action path.
        if action_path is None:
            if not self.available_bar_actions:
                print("No BarAction files available!")
                return False
            if self.selected_state_index >= len(self.available_bar_actions):
                print(f"Invalid BarAction index: {self.selected_state_index}")
                return False
            action_path = self.available_bar_actions[self.selected_state_index]
        if not os.path.isabs(action_path):
            action_path = os.path.join(
                DESIGN_DATA_DIRECTORY, DESIGN_PROBLEM_NAME,
                'BarActions', action_path,
            )
        self._current_action_path = action_path

        for uid in getattr(self, '_bar_holding_fit_line_uids', []) or []:
            try:
                pp.remove_debug(uid)
            except Exception:
                pass
        self._bar_holding_fit_line_uids = []

        print(f"Loading BarAction: {action_path}")

        # 2) Parse + resolve movement.
        try:
            action = parse_bar_action(action_path)
            idx, mv = find_movement(action, movement)
        except Exception as e:
            print(f"Error parsing BarAction: {e}")
            return False

        # 3) Ensure a cfab session for this problem.
        if self.cfab is None or self.cfab.problem_name != DESIGN_PROBLEM_NAME:

            if self.cfab is not None:
                self.cfab.close()
            try:
                existing_client_id = pp.CLIENT if pp.is_connected() else None
                self.cfab = CfabSession(DESIGN_PROBLEM_NAME,
                                        connection_type="gui",
                                        enable_debug_gui=True,
                                        existing_client_id=existing_client_id)
                if existing_client_id is not None:
                    pp.CLIENTS.setdefault(existing_client_id, True)
            except Exception as e:
                print(f"Error initializing CfabSession for {DESIGN_PROBLEM_NAME}: {e}")
                self.cfab = None
                return False

        # Cfab's set_robot_cell loads its own husky URDF (+ tool URDFs) into
        # the shared GUI client, overlapping the real robot from world.init.
        # Hide them so the live scene reads cleanly. Collision/FK on the cfab
        # side still use these bodies. Idempotent on subsequent calls.
        # Skipped in headless tests where no pp-side husky overlaps.
        if getattr(self, '_is_live_monitor', False):
            self._hide_cfab_robot()

        if mv.start_state is None:
            print(f"Movement {mv.movement_id!r} has no start_state; skipping.")
            return False

        # 4) Reset monitor BarAction tracking fields.
        self.current_action = action
        self.current_movement = mv
        self.current_movement_index = idx
        self.movement_start_state = mv.start_state
        self.target_ee_frames = mv.target_ee_frames or None
        self.active_bar_name = f"bar_{action.active_bar_id}"

        # Read grasp (= attachment_frame of the active bar in the gripper
        # link's frame). Same info already lives in start_state; we cache
        # for downstream planner consumers.
        rb_states = getattr(mv.start_state, 'rigid_body_states', {}) or {}
        bar_rb = rb_states.get(self.active_bar_name)
        if bar_rb is not None and bar_rb.attachment_frame is not None:
            self.grasp_link_from_bar = bar_rb.attachment_frame
        else:
            self.grasp_link_from_bar = None

        # 5) Push state into the cfab planner. This materializes all rigid
        # body poses, attaches tool bodies to their parent links, and sets
        # up the ACM internally.
        try:
            self.cfab.planner.set_robot_cell_state(mv.start_state)
        except Exception as e:
            print(f"Error setting cfab robot cell state: {e}")
            return False

        # Bridge the loaded cfab scene into the pp-side state that the
        # CDFM validation / waypoint sliders / inspector consume.
        try:
            self._bridge_cfab_to_pp_for_bar_action()
        except Exception as e:
            print(f"Error bridging cfab scene to pp for BarAction: {e}")
            return False

        # 6) Sanity-check the start state for collisions (non-fatal).
        try:
            self.cfab.planner.check_collision(
                mv.start_state,
                {"_skip_set_robot_cell_state": True,
                 "full_report": False, "verbose": False},
            )
            print(f"Start state of {mv.movement_id} is collision-free.")
        except CollisionCheckError as e:
            n_pairs = len(getattr(e, 'collision_pairs', None) or [])
            first = (e.message.splitlines()[0] if e.message else "(no message)")
            print(f"WARN: start state of {mv.movement_id} has "
                  f"{n_pairs} collision pair(s); continuing. First: {first}")

        # 7) Extract goal_arm_pose / goal_base_pose from start_state's
        # robot_configuration (for visualization + downstream IK seed).
        if hasattr(mv.start_state, 'robot_configuration') and \
                mv.start_state.robot_configuration is not None:
            robot_config = mv.start_state.robot_configuration
            if hasattr(robot_config, 'joint_values') and hasattr(robot_config, 'joint_names'):
                left_arm_names = HUSKY_DUAL_UR5e_JOINT_NAMES[0]
                right_arm_names = HUSKY_DUAL_UR5e_JOINT_NAMES[1]
                try:
                    self.goal_arm_pose[0] = np.array(
                        [robot_config[n] for n in left_arm_names])
                    self.goal_arm_pose[1] = np.array(
                        [robot_config[n] for n in right_arm_names])
                    if update_goal_state:
                        self.reset_ui(self.goal_arm_pose)
                except (KeyError, AttributeError) as e:
                    print(f"WARN: could not extract arm joint values: {e}")
        if hasattr(mv.start_state, 'robot_base_frame') and \
                mv.start_state.robot_base_frame is not None:
            self.goal_base_pose = pose_from_frame(mv.start_state.robot_base_frame)
            if self.BAR_ACTION_LIVE_REPLAN_EXE:
                self.goal_base_pose_frozen = True

        # 7b) For BAR_ACTION_LIVE_REPLAN_EXE, override goal_arm_pose with the IK
        # solution on target_ee_frames so the goal ghost reflects the target
        # EE pose (not the movement's start config, which can be identical
        # across adjacent movements: M2.start == M1.end etc).
        if self.BAR_ACTION_LIVE_REPLAN_EXE and self.target_ee_frames is not None:
            conf12 = _solve_bar_action_goal_ik(
                self, mv.start_state, skip_env_collisions=True, verbose=False,
            )
            if conf12 is not None:
                self.goal_arm_pose[0] = np.asarray(conf12[:6])
                self.goal_arm_pose[1] = np.asarray(conf12[6:])
                if update_goal_state:
                    self.reset_ui(self.goal_arm_pose)
                print(
                    f"BAR_ACTION_LIVE_REPLAN_EXE: goal_arm_pose overridden from "
                    f"IK on target_ee_frames (movement {mv.movement_id})."
                )
            else:
                print(
                    f"WARN: IK on target_ee_frames failed for {mv.movement_id}; "
                    f"goal ghost falls back to start_state config."
                )

        if update_goal_state:
            self.set_to_show_goal_state()

        print(
            f"Loaded BarAction {action.action_id} "
            f"movement[{idx}]={mv.movement_id} ({type(mv).__name__}) "
            f"active_bar={action.active_bar_id} "
            f"rigid_bodies={len(self.cfab.client.rigid_bodies_puids)}"
        )
        return True

    def _hide_cfab_robot(self):
        """Tint the cfab-side robot URDF + tools (red, alpha=0.5) so their
        pose updates from `set_robot_cell_state` are visible during cfab CC
        debugging.

        Tools are tinted the same translucent red as the body so you can
        confirm each mounted tool (assembly tool, gripper, punch cone) is
        actually attached to tool0 in the planning scene.
        """
        if self.cfab is None or self.cfab.client is None:
            return
        client = self.cfab.client
        if client.robot_puid is not None:
            pp.set_color(client.robot_puid, [1.0, 0.0, 0.0, 0.5])
        for tool_puid in (client.tools_puids or {}).values():
            pp.set_color(tool_puid, [1.0, 0.0, 0.0, 0.5])

    def _bridge_cfab_to_pp_for_bar_action(self):
        # TODO this looks a bit suspicious with the manually created sphere proxy etc. need to double check if still correct
        """Wire the loaded cfab scene into the pp-side state that the CDFM
        validation, waypoint sliders, and collision inspector consume.
        Headless-equivalent of the bridge block in
        scripts/headless_live_monitor_test.py.

        Does NOT permanently change pp.CLIENT (the monitor's update() loop
        needs the monitor's own pp client); consumers do a temporary swap
        when they run.
        """
        client = self.cfab.client
        robot_puid = client.robot_puid
        cid = client.client_id

        # 1) Ghost EE proxy bodies (tiny invisible spheres) — recreate per
        #    cfab session. pp routes EE attachments through get_collision_fn,
        #    so the child must be a distinct body (robot-vs-robot collapses).
        if getattr(self, "_bar_action_cfab_id", None) != cid:
            col = p.createCollisionShape(p.GEOM_SPHERE, radius=0.001, physicsClientId=cid)
            ghost_L = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                                          basePosition=[0.0, 0.0, -100.0], physicsClientId=cid)
            col2 = p.createCollisionShape(p.GEOM_SPHERE, radius=0.001, physicsClientId=cid)
            ghost_R = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col2,
                                          basePosition=[0.0, 0.0, -100.0], physicsClientId=cid)
            self._bar_action_ghost_bodies = {ghost_L, ghost_R}
            self._bar_action_cfab_id = cid
            # Need pp.CLIENT == cid for link_from_name / Attachment below.
            _saved = pp.CLIENT
            pp.CLIENT = cid
            pp.CLIENTS.setdefault(cid, True)
            try:
                left_tool_link = pp.link_from_name(robot_puid, 'left_ur_arm_tool0')
                right_tool_link = pp.link_from_name(robot_puid, 'right_ur_arm_tool0')
                identity_grasp = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
                self._bar_action_husky = SimpleNamespace(object=SimpleNamespace(
                    robot=robot_puid,
                    ee_list=[
                        (ghost_L, pp.Attachment(robot_puid, left_tool_link, identity_grasp, ghost_L)),
                        (ghost_R, pp.Attachment(robot_puid, right_tool_link, identity_grasp, ghost_R)),
                    ],
                ))
            finally:
                pp.CLIENT = _saved

        # 2) Active bar + static obstacles (exclude the ghosts).
        ghosts = getattr(self, "_bar_action_ghost_bodies", set())
        puids = client.rigid_bodies_puids
        self.active_bar_body = (puids.get(self.active_bar_name) or [None])[0]
        self.static_obstacles = {
            n: ids[0] for n, ids in puids.items()
            if ids and n != self.active_bar_name and ids[0] not in ghosts
        }
        self.active_extra_bodies = []
        self.bar_from_extra = []
        self.active_bar_aabb_dims = self.get_active_bar_aabb_dims()

    def _apply_live_base_to_movement(self, mv):
        """Overwrite ``mv.start_state.robot_base_frame`` with the live husky
        base pose and push the updated state to cfab.

        Mutates ``mv.start_state`` in place so every downstream reader sees
        the live base — both ``monitor.movement_start_state`` (same object,
        per ``load_selected_movement``) and the per-role dispatchers in
        ``plan_selected_movement`` which read ``mv.start_state`` directly.

        Returns True on success; False (with a warn) on any precondition
        miss. The authored base from the BarAction file is overwritten in
        memory; to restore it, re-load the BarAction.
        """
        if mv is None or mv.start_state is None:
            self.get_logger().warn("apply live base: mv has no start_state.")
            return False
        if not self.huskies:
            self.get_logger().warn("apply live base: no husky available.")
            return False
        if self.cfab is None or self.cfab.planner is None:
            self.get_logger().warn("apply live base: cfab planner not initialized.")
            return False
        hi = self.huskies[self.selected_robot_id].interface
        mv.start_state.robot_base_frame = frame_from_pose((hi.position, hi.rotation))
        try:
            self.cfab.planner.set_robot_cell_state(mv.start_state)
        except Exception as e:
            self.get_logger().warn(f"apply live base: set_robot_cell_state failed: {e}")
            return False
        return True

    def replan_free_from_live_base(self):
        """Replan the loaded movement from the live base+conf; hide goal bar.

        "Live replan" is just the normal per-role dispatch with the live
        robot pose written into the movement's start_state first.
        """
        mv = self.current_movement
        if mv is None or mv.start_state is None:
            self.get_logger().warn("Load a movement first.")
            return
        self._inject_live_conf_into_state(mv.start_state)
        self.plan_selected_movement()
        self._hide_goal_bar()

    def replan_constrained_from_live_base(self):
        """Replan constrained (M1) from the live base+conf."""
        mv = self.current_movement
        if mv is None or mv.start_state is None:
            self.get_logger().warn("Load a movement first.")
            return
        self._inject_live_conf_into_state(mv.start_state)
        self.plan_selected_movement()
        self._show_goal_bar()

    def _hide_goal_bar(self):
        if getattr(self, 'goal_gripper_model', None) is not None:
            pp.set_color(self.goal_gripper_model, TRANSPARENT)

    def _show_goal_bar(self):
        if getattr(self, 'goal_gripper_model', None) is not None:
            pp.set_color(self.goal_gripper_model, GOAL_BLUE)

    # --- --- --- --- --- PER-MOVEMENT BARACTION FLOW --- --- --- --- ---

    def _match_movement_role(self, mv):
        """Return 'M0' | 'M1' | 'M2' | 'M3' | 'M4' | None based on movement_id.

        Movement ids follow the producer convention `<bar>_M<n>_<desc>`,
        e.g. 'B6_M0_free_to_M1_start'.
        """
        mid = getattr(mv, 'movement_id', '') or ''
        match = re.search(r'_M([0-9])_', mid)
        return f'M{match.group(1)}' if match else None

    def _print_cfab_collision_check_setup(self, state, header='cfab CC setup'):
        """Pretty-print the Allowed-Collision-Matrix (ACM) that cfab's
        `check_collision` would apply at the given RobotCellState.

        The cfab checker runs 5 categories (see
        compas_fab/backends/pybullet/.../pybullet_check_collision.py):

          CC.1  robot link ↔ robot link
                SKIP if {a,b} in client.unordered_disabled_collisions (SRDF).
          CC.2  robot link ↔ tool
                SKIP if link_name in tool_state.touch_links, or tool hidden.
          CC.3  robot link ↔ rigid body
                SKIP if link_name in rb_state.touch_links, or rb hidden.
          CC.4  attached rigid body ↔ other rigid body
                SKIP if neither body is attached, hidden, or in the other's
                touch_bodies.
          CC.5  tool ↔ rigid body
                SKIP if rb attached to that tool, tool hidden, rb hidden,
                or tool in rb_state.touch_bodies.

        This dump tells you, at a glance, why a given pair WOULD be
        checked or skipped — useful when you see an obvious tool↔link
        overlap getting flagged: the tool's touch_links is missing that
        link.
        """
        if self.cfab is None or getattr(self.cfab, 'client', None) is None:
            print(f"[{header}] cfab session not initialized; skipping.")
            return
        client = self.cfab.client
        rc = client.robot_cell
        robot_name = getattr(getattr(rc, 'robot_model', None), 'name', None) or '?'
        n_links = len(client.robot_link_puids or {})
        tools_puids = client.tools_puids or {}
        bodies_puids = client.rigid_bodies_puids or {}
        tool_states = (state.tool_states or {}) if state is not None else {}
        rb_states = (state.rigid_body_states or {}) if state is not None else {}

        print(f"\n=== {header} ===")
        print(f"robot: '{robot_name}'  ({n_links} links)")
        print(f"tools loaded: {len(tools_puids)} | rigid bodies loaded: {len(bodies_puids)}")

        # CC.1
        disabled = getattr(client, 'unordered_disabled_collisions', None) or set()
        total_pairs = n_links * (n_links - 1) // 2 if n_links else 0
        print(f"\n[CC.1]  robot link ↔ robot link")
        print(f"  pairs:        {total_pairs}")
        print(f"  SRDF-skipped: {len(disabled)}")
        sample = list(disabled)[:6]
        for s in sample:
            a, b = sorted(s)
            print(f"    SKIP  {a}  <->  {b}")
        if len(disabled) > 6:
            print(f"    … +{len(disabled) - 6} more SRDF-disabled pair(s)")

        # CC.2
        print(f"\n[CC.2]  robot link ↔ tool")
        if not tools_puids:
            print(f"  (no tools loaded)")
        for tool_name in sorted(tools_puids):
            ts = tool_states.get(tool_name)
            if ts is None:
                print(f"  tool '{tool_name}': NO tool_state — every (link, tool) pair is checked")
                continue
            hidden = bool(getattr(ts, 'is_hidden', False))
            touch = sorted(getattr(ts, 'touch_links', None) or [])
            flag = " [HIDDEN — all CC.2 SKIP]" if hidden else ""
            print(f"  tool '{tool_name}'{flag}")
            print(f"    touch_links ({len(touch)}): {touch if touch else '∅'}")
            if not hidden:
                missing = sorted(set(client.robot_link_puids or {}) - set(touch))
                # Show only the closest robot-arm links to flag missing ACM
                # for tool-mounted geometry; full list is long.
                arm_link_keywords = (
                    'tool0', 'flange', 'wrist_3', 'wrist_2', 'wrist_1',
                    'forearm', 'upper_arm', 'shoulder', 'elbow',
                )
                missing_arm = [l for l in missing
                               if any(k in l for k in arm_link_keywords)]
                if missing_arm:
                    print(f"    arm-links NOT in touch_links (CC.2 will CHECK these against '{tool_name}'):")
                    for l in missing_arm:
                        print(f"      CHECK  {l}  <->  {tool_name}")

        # CC.3 / CC.4 / CC.5: per rigid body.
        print(f"\n[CC.3 / CC.4 / CC.5]  rigid bodies (state-attached / touch info)")
        if not rb_states:
            print(f"  (no rigid_body_states in state)")
        arm_link_keywords = (
            'tool0', 'flange', 'wrist_3', 'wrist_2', 'wrist_1',
            'forearm', 'upper_arm', 'shoulder', 'elbow',
        )
        all_links = list(client.robot_link_puids or {})
        for body_name in sorted(rb_states):
            rb = rb_states[body_name]
            hidden = bool(getattr(rb, 'is_hidden', False))
            att_link = getattr(rb, 'attached_to_link', None)
            att_tool = getattr(rb, 'attached_to_tool', None)
            touch_links = sorted(getattr(rb, 'touch_links', None) or [])
            touch_bodies = sorted(getattr(rb, 'touch_bodies', None) or [])
            flags = []
            if hidden:
                flags.append('HIDDEN')
            if att_link:
                flags.append(f"attached_to_link={att_link!r}")
            if att_tool:
                flags.append(f"attached_to_tool={att_tool!r}")
            tag = ('  [' + ', '.join(flags) + ']') if flags else ''
            print(f"  body '{body_name}'{tag}")
            print(f"    CC.3 touch_links  ({len(touch_links)}): "
                  f"{touch_links if touch_links else '∅'}")
            print(f"    CC.4/5 touch_bodies ({len(touch_bodies)}): "
                  f"{touch_bodies if touch_bodies else '∅'}")
            # For attached rigid bodies, surface the arm-side links that
            # are NOT in touch_links — those are the ones CC.3 will FLAG
            # the moment the body's mesh overlaps them by a hair. This is
            # almost always how a missing ACM entry shows up (e.g.
            # tool-mesh overlaps forearm/elbow on a folded-wrist pose).
            if att_link and not hidden:
                # Pick the "side" of the robot the body is mounted on
                # (left_/right_) so we only surface the relevant arm.
                side = None
                if att_link.startswith('left_'):
                    side = 'left_'
                elif att_link.startswith('right_'):
                    side = 'right_'
                missing_arm = [
                    l for l in all_links
                    if (side is None or l.startswith(side))
                    and any(k in l for k in arm_link_keywords)
                    and l not in touch_links
                ]
                if missing_arm:
                    print(f"    arm-links NOT in touch_links "
                          f"(CC.3 will CHECK these against '{body_name}'):")
                    for l in missing_arm:
                        print(f"      CHECK  {l}  <->  {body_name}")
        print(f"=== end {header} ===\n")

    def _arm_joint_name_sets(self):
        """Return the per-arm UR joint-name lists of the loaded cfab cell.

        Dual rig: [left 6 names, right 6 names]. Single rig: [6 names].
        Derived from the cell's SRDF groups so the same code serves Alice /
        Belle (single-arm) and Cindy (dual-arm).
        """
        cell = self.cfab.robot_cell
        groups = cell.robot_semantics.groups
        if 'base_left_arm_manipulator' in groups:
            return [arm_joint_names_for_group(cell, 'base_left_arm_manipulator'),
                    arm_joint_names_for_group(cell, 'base_right_arm_manipulator')]
        return [arm_joint_names_for_group(cell, SINGLE_ARM_GROUP)]

    def _fill_missing_start_conf(self, state):
        """Fill a None robot_configuration with the dual-arm home pose.

        Authored BarAction states before chain planning can carry no
        robot_configuration; planners still need a seed dict for IK.
        No-op when the state already has one.

        Args:
            state: RobotCellState modified in place (None is ignored).
        """
        if state is None or state.robot_configuration is not None:
            return
        state.robot_configuration = self.cfab.robot_cell.zero_full_configuration()
        for i, names in enumerate(self._arm_joint_name_sets()):
            for n, v in zip(names, HUSKY_DUAL_ARM_HOME_CONF_12[i * 6:(i + 1) * 6]):
                state.robot_configuration[n] = float(v)
        print("[fill] start_state.robot_configuration was None; seeded with "
              "dual-arm home.")

    def _inject_live_conf_into_state(self, state):
        """Overwrite a state's base frame + arm joints with the LIVE robot pose.

        Used wherever a movement's start must reflect where the robot
        actually is right now: the native M0 (whose authored
        robot_configuration is null), the free-to-movement-start planner,
        and the live-replan buttons.

        Args:
            state: RobotCellState modified in place. If its
                robot_configuration is None, a zero full configuration is
                created first so the live values have a place to land.
        """
        # Diagnostic: one-shot dump of cfab's ACM at this state. Fires only
        # once per cfab session so per-movement reloads don't spam.
        if not getattr(self, '_cfab_acm_printed_for_cid', None) == getattr(
                getattr(self.cfab, 'client', None), 'client_id', None):
            try:
                self._print_cfab_collision_check_setup(
                    state, header="cfab CC setup @ live-injected state",
                )
            except Exception as e:
                print(f"[cfab CC setup] ERROR: {e}")
            self._cfab_acm_printed_for_cid = getattr(
                getattr(self.cfab, 'client', None), 'client_id', None)
        hi = self.huskies[self.selected_robot_id].interface
        state.robot_base_frame = frame_from_pose((hi.position, hi.rotation))
        if state.robot_configuration is None:
            state.robot_configuration = self.cfab.robot_cell.zero_full_configuration()
        for i, names in enumerate(self._arm_joint_name_sets()):
            values = hi.arm_joint_pose[i] if len(hi.arm_joint_pose) > i else hi.arm_joint_pose[0]
            for n, v in zip(names, values):
                state.robot_configuration[n] = float(v)

    def load_bar_action_file(self):
        """Parse the selected BarAction JSON; log the movement roster.

        The JSON natively carries all movements M0..M4. M0's authored
        robot_configuration is null (its start is wherever the robot lives
        right now), so its start_state gets the live pose injected here and
        again on every 'Load Movement'.
        """
        files = self.available_bar_actions
        if not files:
            if hasattr(self, '_load_available_bar_actions'):
                self.available_bar_actions = self._load_available_bar_actions()
                files = self.available_bar_actions
        if not files:
            self.get_logger().warn("No BarAction files available.")
            return
        idx = max(0, min(self._selected_action_file_idx, len(files) - 1))
        fname = files[idx]
        action_path = fname if os.path.isabs(fname) else os.path.join(
            DESIGN_DATA_DIRECTORY, DESIGN_PROBLEM_NAME, 'BarActions', fname,
        )
        self._current_action_path = action_path
        self._loaded_action = parse_bar_action(action_path)

        if not self._loaded_action.movements:
            self.get_logger().warn("BarAction has no movements.")
        self._loaded_movements = list(self._loaded_action.movements)

        self.get_logger().info(f"Loading BarAction from file {action_path}")

        # Init the per-problem cfab session + robot cell now so 'Load
        # Movement' is just a state push afterwards. The startup default
        # session (problem_name None, no design bars) is replaced here.
        if self.cfab is None or self.cfab.problem_name != DESIGN_PROBLEM_NAME:
            if self.cfab is not None:
                self.cfab.close()
                self.cfab = None
            try:
                existing_client_id = pp.CLIENT if pp.is_connected() else None
                with pp.LockRenderer():
                    self.cfab = CfabSession(DESIGN_PROBLEM_NAME,
                                            connection_type="gui",
                                            enable_debug_gui=True,
                                            existing_client_id=existing_client_id)
                if existing_client_id is not None:
                    pp.CLIENTS.setdefault(existing_client_id, True)
            except Exception as e:
                print(f"Error initializing CfabSession: {e}")
                return
            if getattr(self, '_is_live_monitor', False):
                self._hide_cfab_robot()

        # Native M0 ships with robot_configuration null: fill it (and the
        # base frame) from the live robot so downstream consistency checks
        # and planning see real values.
        for mv in self._loaded_movements:
            if self._match_movement_role(mv) == 'M0' and mv.start_state is not None:
                self._inject_live_conf_into_state(mv.start_state)

        print(f"[BarAction] loaded {os.path.basename(action_path)} "
              f"with {len(self._loaded_movements)} movements:")
        for i, mv in enumerate(self._loaded_movements):
            print(f"  [{i}] {mv.movement_id!r} role={self._match_movement_role(mv)}")
        # Refresh UI so the Movement slider's range now matches the loaded
        # movement count (was 0..8 before; now 0..len(movements)-1).

        self.reset_ui(self.goal_arm_pose)

        # Trajectories now live on mv objects in memory (loaded natively via
        # compas json_load when a `.live-solved.json` sidecar is opened via
        # this same Load BarAction button). Print the initial roster.
        self._print_movement_roster(tag='LoadBarAction')

    def load_selected_movement(self):
        """Load the selected movement's start state into cfab + goal ghost."""
        if not self._loaded_movements:
            self.get_logger().warn("No BarAction loaded; click 'Load BarAction' first.")
            return
        idx = max(0, min(self._selected_movement_idx, len(self._loaded_movements) - 1))
        mv = self._loaded_movements[idx]

        # If M0, re-snapshot live conf/base into its start_state so a robot
        # that moved since 'Load BarAction' still plans from where it is.
        if self._match_movement_role(mv) == 'M0' and mv.start_state is not None:
            self._inject_live_conf_into_state(mv.start_state)

        if mv.start_state is None:
            self.get_logger().warn(f"Movement {mv.movement_id!r} has no start_state.")
            return

        if self.cfab is None:
            self.get_logger().warn("cfab not initialized; click 'Load BarAction' first.")
            return

        self.current_action = self._loaded_action
        self.current_movement = mv
        self.current_movement_index = idx
        self.movement_start_state = mv.start_state
        self.target_ee_frames = mv.target_ee_frames or None
        bar_id = getattr(self._loaded_action, 'active_bar_id', None) if self._loaded_action else None
        self.active_bar_name = f"bar_{bar_id}" if bar_id else None

        # Restore previously-ghosted bodies' original colors before pushing
        # the new state (which may re-spawn or change which bodies are attached).
        for body, c in list(self._traj_ghost_orig_colors.items()):
            try:
                pp.set_color(body, c)
            except Exception:
                pass
        self._traj_ghost_bodies = []
        self._traj_ghost_orig_colors = {}

        try:
            self.cfab.planner.set_robot_cell_state(mv.start_state)
        except Exception as e:
            print(f"Error setting cfab robot cell state: {e}")
            return
        try:
            self._bridge_cfab_to_pp_for_bar_action()
        except Exception as e:
            print(f"Error bridging cfab scene to pp: {e}")
            return

        rb_states = getattr(mv.start_state, 'rigid_body_states', {}) or {}
        bar_rb = rb_states.get(self.active_bar_name) if self.active_bar_name else None
        self.grasp_link_from_bar = bar_rb.attachment_frame if (bar_rb and bar_rb.attachment_frame) else None

        # M0/M4 are pre-pickup / post-place free transits: visually the bar
        # (and any attached joint pieces) should NOT ride the robot during
        # goal-conf / trajectory preview. Tool bodies stay attached so the
        # ghost still shows the actual TCP geometry.
        movement_role = self._match_movement_role(mv)
        hide_non_tool_attachments = movement_role in ('M0', 'M4')
        _TOOL_BODY_NAMES = {'AssemblyLeftArmToolBody', 'AssemblyRightArmToolBody'}

        # Collect attached-body ghosts (bar + any joint pieces). Color the
        # cfab-spawned body green; cache original RGBA so we can restore it
        # on next load.
        for name, rbs in rb_states.items():
            if getattr(rbs, 'attached_to_link', None) is None:
                continue
            if getattr(rbs, 'attachment_frame', None) is None:
                continue
            ids = (self.cfab.client.rigid_bodies_puids or {}).get(name) or []
            if not ids:
                continue
            body = ids[0]
            try:
                vis = p.getVisualShapeData(body)
                self._traj_ghost_orig_colors[body] = list(vis[0][7]) if vis else [0.7, 0.7, 0.7, 1.0]
            except Exception:
                self._traj_ghost_orig_colors[body] = [0.7, 0.7, 0.7, 1.0]

            if hide_non_tool_attachments and name not in _TOOL_BODY_NAMES:
                # Don't drag it with the robot in preview; hide the
                # cfab-spawned body (cached above for restore on next load).
                try:
                    pp.set_color(body, TRANSPARENT)
                except Exception:
                    pass
                continue

            try:
                pp.set_color(body, TRAJECTORY_GREEN)
            except Exception:
                pass
            self._traj_ghost_bodies.append({
                'body': body,
                'link': rbs.attached_to_link,
                'attach': pose_from_frame(rbs.attachment_frame),
            })
        if self._traj_ghost_bodies:
            print(f"[Movement] attached-body ghosts: "
                  f"{[g['link'] for g in self._traj_ghost_bodies]}")
        if hide_non_tool_attachments:
            print(f"[Movement] {movement_role}: hid non-tool attachments "
                  f"(bar/joint) for preview")

        if mv.start_state.robot_configuration is not None:
            rc = mv.start_state.robot_configuration
            try:
                self.goal_arm_pose[0] = np.array(
                    [rc[n] for n in HUSKY_DUAL_UR5e_JOINT_NAMES[0]])
                self.goal_arm_pose[1] = np.array(
                    [rc[n] for n in HUSKY_DUAL_UR5e_JOINT_NAMES[1]])
            except (KeyError, AttributeError) as e:
                print(f"WARN: could not extract arm joint values: {e}")
        if mv.start_state.robot_base_frame is not None:
            self.goal_base_pose = pose_from_frame(mv.start_state.robot_base_frame)
            if self.BAR_ACTION_LIVE_REPLAN_EXE:
                self.goal_base_pose_frozen = True

            # In FAKE_HARDWARE mode, teleport the real-robot base exactly to
            # the movement's start_state base. With FAKE_HARDWARE=0, leave
            # the live mocap reading to drive the real-robot base via
            # receive_mocap_frame.
            if self.FAKE_HARDWARE:
                hi = self.huskies[self.selected_robot_id].interface
                hi.position = np.asarray(self.goal_base_pose[0], dtype=float)
                hi.rotation = np.asarray(self.goal_base_pose[1], dtype=float)

        for uid in self._ee_target_pose_uids:
            try:
                pp.remove_debug(uid)
            except Exception:
                pass
        self._ee_target_pose_uids = []
        if mv.target_ee_frames:
            for side, frame in mv.target_ee_frames.items():
                if frame is None:
                    continue
                pose = pose_from_frame(frame)
                uids = pp.draw_pose(pose, length=0.15)
                if uids:
                    self._ee_target_pose_uids.extend(uids if isinstance(uids, (list, tuple)) else [uids])

        self.reset_ui(self.goal_arm_pose)
        self.set_to_show_goal_state()

        print(f"[Movement] loaded [{idx}] {mv.movement_id!r} type={type(mv).__name__} "
              f"role={self._match_movement_role(mv)} "
              f"has_targets={bool(mv.target_ee_frames)} traj={mv.trajectory is not None}")

        # If mv already carries a trajectory in memory (loaded from a
        # `.live-solved.json` sidecar), auto-wire it into the viz so the
        # traj-viz time slider is immediately previewable without another
        # click on 'Load Movement Trajectory'.
        if getattr(mv, 'trajectory', None) is not None:
            self.load_selected_movement_trajectory()

    def plan_selected_movement(self):
        """Dispatch the right planner for the loaded movement; store trajectory."""
        if self.current_movement is None:
            self.get_logger().warn("No movement loaded; click 'Load Movement' first.")
            return
        mv = self.current_movement
        role = self._match_movement_role(mv)
        if role is None:
            self.get_logger().warn(f"Unknown movement role for {mv.movement_id!r}; skipping.")
            return
        if mv.trajectory is not None:
            self.get_logger().warn(
                f"Overwriting existing trajectory for {mv.movement_id!r}"
            )

        # Plan against the LIVE husky base, not the BarAction-authored one.
        # Mutates mv.start_state.robot_base_frame in place + pushes to cfab.
        if not self._apply_live_base_to_movement(mv):
            return

        # Authored states may carry no robot_configuration (M1 before its
        # chain is planned): give the planner's IK a home seed to work from
        # (same as fill_missing_config in headless_bar_action_planner).
        self._fill_missing_start_conf(mv.start_state)

        dispatch = {
            'M0': self._plan_M0_dispatch,
            'M1': self._plan_M1_dispatch,
            'M2': self._plan_M2_dispatch,
            'M3': self._plan_M3_dispatch,
            'M4': self._plan_M4_dispatch,
        }[role]
        jt = dispatch(mv)
        if jt is None:
            self.get_logger().warn(f"Plan for {mv.movement_id!r} ({role}) FAILED.")
            if role == 'M1':
                self._clear_m1_start_conf_without_trajectory()
            return

        self._accept_trajectory(mv, jt, source='Plan', role=role)

    # --- --- --- Chain planning (Button 1) --- --- ---

    # Canonical BarAction plan order:
    #   M1 (owns its derived start via `derive_start=True`)
    #   -> M2 (start comes from M1.traj[-1])
    #   -> M3 (start comes from M2.traj[-1])
    #   -> M0 (goal = M1.start_state.robot_configuration, backfilled after M1)
    #   -> M4 (start comes from M3.traj[-1], goal is fixed home)
    _CHAIN_ROLE_ORDER = ('M1', 'M2', 'M3', 'M0', 'M4')

    def plan_movement_chain_live(self):
        # TODO this should be moved to husky_planning.py
        """Plan the M1 -> M2 -> M3 -> M0 -> M4 chain against the live base.

        For each role in ``_CHAIN_ROLE_ORDER`` that is present in the loaded
        BarAction: set the movement slider to that index, call
        ``load_selected_movement()`` so the cfab scene / goal viz sync, then
        call ``plan_selected_movement()``. ``plan_selected_movement`` already
        applies the live base via ``_apply_live_base_to_movement``, warm-starts
        IK from any stored start conf, dispatches to the role-specific
        planner, and routes through ``_accept_trajectory`` (state propagation
        to the next movement in the list order).

        Stop-on-first-failure: if ``mv.trajectory`` is None after
        ``plan_selected_movement`` returns, break out of the loop. Previously
        planned movements' trajectories stay on their mv objects AND get
        written to the sidecar. Any exception inside ``plan_selected_movement``
        bubbles up unhandled (no defensive try/except).

        Sidecar export: on loop exit (full success or early break), if at
        least one movement has a trajectory, serialize the mutated
        ``self._loaded_action`` (whose ``movements`` share object identity with
        ``self._loaded_movements``) via ``compas.data.json_dump`` to
        ``<original>.live-solved.json`` in the same directory.
        """
        if not self._loaded_movements:
            self.get_logger().warn(
                "No movements loaded; click 'Load BarAction' first."
            )
            return
        if not self._current_action_path:
            self.get_logger().warn(
                "No BarAction file path known (was it loaded via 'Load BarAction'?)."
            )
            return

        # Build the ordered index list from _CHAIN_ROLE_ORDER; skip missing roles.
        role_to_idx = {}
        for i, mv in enumerate(self._loaded_movements):
            r = self._match_movement_role(mv)
            if r and r not in role_to_idx:
                role_to_idx[r] = i
        sequence = [role_to_idx[r] for r in self._CHAIN_ROLE_ORDER if r in role_to_idx]
        if not sequence:
            self.get_logger().warn(
                "[Plan Chain] no movements matched any of "
                f"{self._CHAIN_ROLE_ORDER}; nothing to plan."
            )
            return

        # Wipe any pre-existing in-memory trajectories on the roles we're
        # about to plan so _accept_trajectory's rejection-on-mismatch does
        # not warn about a stale value we intentionally overwrite. Roles
        # NOT in the sequence (rare) keep their trajectories untouched.
        for i in sequence:
            self._loaded_movements[i].trajectory = None

        planned_ids = []
        stopped_at_role = None
        for step, idx in enumerate(sequence, start=1):
            mv = self._loaded_movements[idx]
            role = self._match_movement_role(mv)
            print(f"\n=== [Plan Chain {step}/{len(sequence)}] {role} idx={idx} "
                  f"id={mv.movement_id!r} ===")

            # Simulate the UI: slider -> Load Movement -> Plan Movement.
            self._selected_movement_idx = idx
            self.load_selected_movement()
            self.plan_selected_movement()

            planned_traj = getattr(self.current_movement, 'trajectory', None)
            if planned_traj is None:
                stopped_at_role = role
                self.get_logger().warn(
                    f"[Plan Chain] {role} ({mv.movement_id!r}) FAILED; "
                    "stopping chain. Previously planned movements are kept."
                )
                break
            planned_ids.append(mv.movement_id)

        # Export sidecar iff at least one mv now carries a trajectory.
        # _loaded_movements shares object identity with _loaded_action.movements
        # (see load_bar_action_file), so mutations already show up in the action.
        any_traj = any(getattr(mv, 'trajectory', None) is not None
                       for mv in self._loaded_movements)
        if any_traj:
            from compas.data import json_dump
            stem, ext = os.path.splitext(self._current_action_path)
            out_path = f"{stem}.live-solved{ext}"
            try:
                json_dump(self._loaded_action, out_path)
                print(f"[Plan Chain] sidecar written -> {out_path}")
            except Exception as e:
                self.get_logger().warn(
                    f"[Plan Chain] failed to write sidecar {out_path}: {e}"
                )
        else:
            print("[Plan Chain] no trajectories to export; skipping sidecar.")

        if stopped_at_role is None:
            print(f"[Plan Chain] SUCCESS: planned "
                  f"{len(planned_ids)}/{len(sequence)} movements.")
        else:
            print(f"[Plan Chain] STOPPED at {stopped_at_role}: planned "
                  f"{len(planned_ids)}/{len(sequence)} movements before failure.")
        self._print_movement_roster(tag='Plan Chain')

    # --- --- --- Reset (per-movement + all) --- --- ---

    def reset_selected_movement_to_clean(self):
        """Revert the currently loaded movement to its authored 'clean' state.

        Re-reads the pristine BarAction JSON from
        ``self._current_action_path`` and overwrites
        ``self._loaded_movements[current_movement_index]`` and
        ``self._loaded_action.movements[current_movement_index]`` with the
        clean-file version (fresh ``start_state``, no propagated
        ``robot_configuration`` from a downstream chain break, and
        ``trajectory=None``). Other movements are untouched: their propagated
        start_confs may now be stale, and a subsequent 'Plan Chain (Live)'
        will re-populate them.
        """
        if self.current_movement is None:
            self.get_logger().warn(
                "No movement loaded; click 'Load Movement' first."
            )
            return
        if not self._current_action_path or not os.path.isfile(self._current_action_path):
            self.get_logger().warn(
                "No BarAction file path known; cannot reset."
            )
            return
        idx = self.current_movement_index
        if idx is None or self._loaded_action is None:
            self.get_logger().warn(
                "Loaded-action state missing; cannot reset."
            )
            return
        try:
            clean = parse_bar_action(self._current_action_path)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse clean BarAction: {e}")
            return
        if idx >= len(clean.movements):
            self.get_logger().warn(
                f"Clean file has {len(clean.movements)} movements; index "
                f"{idx} out of range."
            )
            return
        clean_mv = clean.movements[idx]
        # Replace by index in BOTH lists so identity stays consistent for
        # any subsequent sidecar export.
        self._loaded_movements[idx] = clean_mv
        self._loaded_action.movements[idx] = clean_mv
        self.current_movement = clean_mv
        print(f"[Reset Mv] reverted [{idx}] {clean_mv.movement_id!r} to clean.")
        # Re-run the standard Load Movement path so movement_start_state,
        # target_ee_frames, and the cfab scene sync to the fresh object.
        self.load_selected_movement()

    def reset_all_movements_to_clean(self):
        """Reload the pristine BarAction from disk (matches
        `headless_bar_action_planner --load clean`).

        Discards every in-memory trajectory and every propagated
        ``start_state.robot_configuration`` value on the currently loaded
        BarAction. Behaviourally equivalent to clicking 'Load BarAction'
        again on the Rhino-authored clean file; the separate wording makes
        the destructive intent explicit.

        Refuses if the currently loaded action is itself a
        ``.live-solved.json`` sidecar (that's not the clean file).
        """
        if not self._current_action_path:
            self.get_logger().warn(
                "No BarAction file path known; cannot reset."
            )
            return
        if self._current_action_path.endswith('.live-solved.json'):
            self.get_logger().warn(
                "Currently loaded action is a `.live-solved.json` sidecar, "
                "not the clean file. Load the clean BarAction JSON first."
            )
            return
        print(f"[Reset All] reloading clean BarAction from "
              f"{self._current_action_path}")
        self.load_bar_action_file()

    # --- --- --- Replan free -> movement start (Button 2) --- --- ---

    def replan_free_to_movement_start_live(self):
        """Fresh live-base IK to the movement's start EE targets, then a
        composite free plan from the live conf to that IK-solved conf.

        Only supports M2 / M3 (their ``start_state`` carries an authored
        ``robot_configuration`` whose FK gives the target start EE frames).
        Combines ``ik_live_base_for_selected_movement`` (which sets
        ``goal_arm_pose`` to the IK-solved conf) with
        ``world.plan_both_arms_to_goal(use_composite=True)`` (composite free
        plan against cfab collision checking).
        """
        if self.current_movement is None:
            self.get_logger().warn(
                "No movement loaded; click 'Load Movement' first."
            )
            return
        role = self._match_movement_role(self.current_movement)
        if role not in ('M2', 'M3'):
            self.get_logger().warn(
                f"Replan Free -> Mv Start only supports M2 / M3; current "
                f"is {role!r}."
            )
            return

        # ---- MOCK LIVE POSE (temporary; see MOCK_LIVE_POSE_FOR_REPLAN
        # class flag) --------------------------------------------------------
        # Toggle at the class flag; when off, this block is a no-op.
        revert_mock = None
        if self.MOCK_LIVE_POSE_FOR_REPLAN:
            revert_mock = self._apply_mock_live_pose_for_replan(
                self.current_movement,
            )
        # -------------------------------------------------------------------

        try:
            # Pause GUI rendering across the whole IK + free-plan search
            # (no-op when headless). The IK descent and the BiRRT sampling
            # both push cfab cell states onto the shared GUI client, which
            # otherwise redraws the (red) cfab robot on every sample.
            with pp.LockRenderer():
                # Step 1: live-base IK sets goal_arm_pose to the IK-solved 12-vec.
                if not self.ik_live_base_for_selected_movement():
                    return

                # Step 2: overwrite mv.start_state.robot_base_frame with the live
                # husky base so the composite free plan uses the live-base state as
                # template. plan_both_arms_to_goal reads movement_start_state.
                if not self._apply_live_base_to_movement(self.current_movement):
                    return

                # Step 3: composite free plan from live conf -> goal_arm_pose. After
                # Configuration adoption in husky_world (Change 1), this now works
                # even though the goal is built via np.concatenate internally.
                # Env collisions are (temporarily) skipped in this motion plan when
                # REPLAN_SKIP_ENV_COLLISIONS_IN_MOTION_PLAN is set.
                world.plan_both_arms_to_goal(
                    self, use_composite=True,
                    skip_env_collisions=bool(self.REPLAN_SKIP_ENV_COLLISIONS_IN_MOTION_PLAN))

                # Step 4: verify the planned path's ENDPOINT actually lands the
                # tool0s on the authored targets. FK at (live base + last
                # waypoint arm conf) should equal the target EE frames derived
                # from the movement's authored start_state at Step 1's FK.
                self._verify_replan_endpoint_matches_target()
        finally:
            if revert_mock is not None:
                revert_mock()

    def _verify_replan_endpoint_matches_target(self,
                                                pos_tol_m: float = 0.005,
                                                ang_tol_deg: float = 1.0) -> None:
        """After a successful Button 2 composite plan, compare the last
        waypoint's tool0 world-frame poses (FK at live base + planned end
        arm conf) against the authored target EE frames the IK solved for.

        This surfaces two situations:
          * Composite plan landed on the IK-solved goal exactly -> tiny
            residual, both arms hit the authored target frames.
          * IK fell back to `alt_seed_conf12` verbatim (no arm-side
            compensation for the base offset) -> residual roughly equal to
            the base offset (~14 mm for the (-0.3, 0.2) mock -> ~360 mm).
            Emits a warning so the operator knows the tool0 targets are
            NOT met and any downstream linear motion needs re-planning.

        Args:
            pos_tol_m: position tolerance in metres. Default 5 mm.
            ang_tol_deg: orientation tolerance in degrees. Default 1 deg.
        """
        target = getattr(self, '_last_ik_target_ee_frames', None)
        if not target or 'left' not in target or 'right' not in target:
            return
        pat = getattr(self, 'planned_arm_trajectory', None)
        if (pat is None
                or pat[0] is None or pat[0][0] is None
                or pat[1] is None or pat[1][0] is None):
            return
        left_last = np.asarray(pat[0][0][-1], dtype=float)
        right_last = np.asarray(pat[1][0][-1], dtype=float)
        if left_last.shape != (6,) or right_last.shape != (6,):
            return
        mv = self.current_movement
        # mv.start_state.robot_base_frame was set to the live base by
        # `_apply_live_base_to_movement` earlier in this flow, so a copy
        # inherits the live base for FK.
        verify_state = mv.start_state.copy()
        left_names = HUSKY_DUAL_UR5e_JOINT_NAMES[0]
        right_names = HUSKY_DUAL_UR5e_JOINT_NAMES[1]
        for name, val in zip(left_names, left_last):
            verify_state.robot_configuration[name] = float(val)
        for name, val in zip(right_names, right_last):
            verify_state.robot_configuration[name] = float(val)
        try:
            fk_left = _fk_link_frame(
                self.cfab.planner, verify_state, "left_ur_arm_tool0")
            fk_right = _fk_link_frame(
                self.cfab.planner, verify_state, "right_ur_arm_tool0")
        except Exception as e:
            self.get_logger().warn(f"[Replan verify] FK on planned end failed: {e}")
            return

        def _residual(fk_frame, tg_frame):
            d_pos = float(np.linalg.norm(
                np.asarray(fk_frame.point) - np.asarray(tg_frame.point)
            ))
            q_fk = np.asarray(fk_frame.quaternion.xyzw, dtype=float)
            q_tg = np.asarray(tg_frame.quaternion.xyzw, dtype=float)
            d_ang = 2.0 * float(np.arccos(
                np.clip(abs(float(np.dot(q_fk, q_tg))), 0.0, 1.0)
            ))
            return d_pos, d_ang

        d_pos_L, d_ang_L = _residual(fk_left, target['left'])
        d_pos_R, d_ang_R = _residual(fk_right, target['right'])
        print(
            f"[Replan verify] planned-end tool0 (live base FK) vs "
            f"authored target EE frames: "
            f"L pos={d_pos_L*1000:.2f} mm ang={np.degrees(d_ang_L):.3f} deg | "
            f"R pos={d_pos_R*1000:.2f} mm ang={np.degrees(d_ang_R):.3f} deg"
        )
        ang_tol_rad = np.radians(ang_tol_deg)
        max_pos = max(d_pos_L, d_pos_R)
        max_ang = max(d_ang_L, d_ang_R)
        if max_pos > pos_tol_m or max_ang > ang_tol_rad:
            self.get_logger().warn(
                f"[Replan verify] tool0 endpoint MISMATCH: max pos="
                f"{max_pos*1000:.2f} mm (tol {pos_tol_m*1000:.1f} mm), "
                f"max ang={np.degrees(max_ang):.3f} deg "
                f"(tol {ang_tol_deg:.1f} deg). "
                f"The composite plan likely landed on the IK fallback "
                f"(alt_seed_conf12 verbatim), which does NOT compensate "
                f"the arm conf for the base offset -- world-frame tool0 "
                f"error scales with the base offset. Any downstream linear "
                f"motion that assumes the authored EE targets should be "
                f"re-planned."
            )

    # ---- MOCK LIVE POSE (temporary; see MOCK_LIVE_POSE_FOR_REPLAN class
    # flag). Delete this method and the flag once real mocap + robot are
    # available. --------------------------------------------------------------
    def _apply_mock_live_pose_for_replan(self, target_mv):
        """MOCK: patch the live husky interface to simulate mocap + robot.

        Overrides ``huskies[0].interface.{position, rotation, arm_joint_pose}``
        so that ``ik_live_base_for_selected_movement`` and the following
        composite free plan see a synthetic "live" pose. See the class-flag
        block for the picker knobs ``MOCK_LIVE_ARM_CONF`` /
        ``MOCK_LIVE_ARM_PERTURB_STD_RAD`` / ``MOCK_LIVE_BASE_XY_OFFSET_M``.

        Returns a callable that restores the original interface values.
        """
        hi = self.huskies[self.selected_robot_id].interface

        # Cache the AUTHORED base frame the first time we see this movement,
        # so repeated Button 2 presses don't compound the offset (each press
        # ends with `_apply_live_base_to_movement` writing hi.position ->
        # mv.start_state.robot_base_frame, which would otherwise become the
        # next mock's base source).
        if not hasattr(self, '_mock_authored_bases'):
            self._mock_authored_bases = {}
        mv_key = id(target_mv)
        if mv_key not in self._mock_authored_bases:
            base_frame = target_mv.start_state.robot_base_frame
            self._mock_authored_bases[mv_key] = (
                base_frame.copy() if hasattr(base_frame, 'copy') else base_frame
            )
        cached_base = self._mock_authored_bases[mv_key]

        pos, rot = pose_from_frame(cached_base)
        dx, dy = self.MOCK_LIVE_BASE_XY_OFFSET_M
        mock_pos = np.asarray(pos, dtype=float) + np.array([float(dx), float(dy), 0.0])
        mock_rot = np.asarray(rot, dtype=float)

        arm_source = getattr(self, 'MOCK_LIVE_ARM_CONF', 'perturb')
        if arm_source == 'home':
            arm_12 = np.asarray(HUSKY_DUAL_ARM_HOME_CONF_12, dtype=float)
            source_tag = "HUSKY_DUAL_ARM_HOME_CONF_12 (extended arms)"
        elif arm_source == 'perturb':
            start_conf = target_mv.start_state.robot_configuration
            if start_conf is None:
                # No propagated start yet -- fall back to home.
                arm_12 = np.asarray(HUSKY_DUAL_ARM_HOME_CONF_12, dtype=float)
                source_tag = "HUSKY_DUAL_ARM_HOME_CONF_12 (fallback: no start_conf)"
            else:
                names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]) \
                        + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
                base_12 = np.array(
                    [float(start_conf[n]) for n in names], dtype=float,
                )
                std = float(self.MOCK_LIVE_ARM_PERTURB_STD_RAD)
                max_tries = int(self.MOCK_LIVE_ARM_PERTURB_MAX_TRIES)

                # Deterministic-ish noise seeded on target_mv id so repeated
                # runs perturb the same way (aids debugging). Retry up to
                # max_tries if the perturbed conf is in self-collision --
                # small std should almost never collide, but the retry
                # keeps the mock reliable across noise draws.
                rng = np.random.default_rng(
                    abs(hash(target_mv.movement_id)) & 0xFFFFFFFF,
                )
                arm_12 = base_12
                colliding_tries = 0
                try_state = target_mv.start_state.copy()
                try_state.robot_base_frame = frame_from_pose((mock_pos, mock_rot))
                for attempt in range(max_tries):
                    candidate = base_12 + rng.normal(0.0, std, size=12)
                    for n, v in zip(names, candidate):
                        try_state.robot_configuration[n] = float(v)
                    try:
                        self.cfab.planner.check_collision(try_state, {"verbose": False})
                        arm_12 = candidate
                        break
                    except Exception:
                        colliding_tries += 1
                        continue
                else:
                    # No non-colliding perturbation found; use base_12 itself
                    # (the movement's own start_conf, known collision-free
                    # by construction) for the mock.
                    arm_12 = base_12

                source_tag = (
                    f"{target_mv.movement_id!r}.start_conf + Gaussian noise "
                    f"(std={std:.3f} rad; skipped {colliding_tries} "
                    f"colliding draw(s))"
                )
        else:
            raise ValueError(
                f"Unknown MOCK_LIVE_ARM_CONF: {arm_source!r}; expected "
                f"'perturb' or 'home'."
            )

        hi.arm_joint_pose = [arm_12[:6].copy(), arm_12[6:].copy()]
        hi.position = mock_pos
        hi.rotation = mock_rot

        print(
            f"[MOCK live pose] arm_conf <- {source_tag}; "
            f"base_pos <- {target_mv.movement_id!r}.start.base + "
            f"({dx:.3f}, {dy:.3f}, 0.0) = {mock_pos.tolist()}."
        )
        print(
            "[MOCK live pose] NOT reverting hi after Button 2 -- the mock "
            "values persist so the ghost display / goal viz keeps reflecting "
            "the mocked live pose. Cached authored base is used for the "
            "next mock draw so the offset does not compound."
        )

        # No-op revert: caller may still invoke it, but state is left as
        # mocked. The cached authored base above ensures repeated presses
        # remain stable.
        def _revert():
            return None

        return _revert
    # -------------------------------------------------------------------------

    # --- --- --- Auto-dispatch execute --- --- ---

    def exec_selected_movement_traj(self):
        """Execute the currently loaded movement's trajectory. Auto-dispatch:
        M2/M3 -> cartesian_compliance_controller via
          ``world.execute_planned_trajectory_compliant`` (a generator queued
          on ``self.tasks`` so the monitor tick pumps it).
        else  -> joint-tracking via ``world.execute_arm_trajectory_both``.
        """
        if self.current_movement is None:
            self.get_logger().warn(
                "No movement loaded; click 'Load Movement' first."
            )
            return
        role = self._match_movement_role(self.current_movement)
        if role in ('M2', 'M3'):
            self.tasks.append(world.execute_planned_trajectory_compliant(self))
        else:
            world.execute_arm_trajectory_both(self)

    def _accept_trajectory(self, mv, jt, *, source='Plan', role=None):
        """Common post-step after a trajectory is either planned or loaded.

        Assigns mv.trajectory, propagates first/last conf to start states,
        wires the visualizer, runs CDFM validation, and prints the movement
        roster. Persistence lives on the ``<action>.live-solved.json`` sidecar
        that ``plan_movement_chain_live`` writes -- no per-movement JSONs.
        """
        mv.trajectory = jt
        path = path_12_from_joint_trajectory(jt)
        if path:
            chain_role = role if role is not None else self._match_movement_role(mv)
            start_vec = np.asarray(path[0], dtype=float)
            if chain_role in ('M2', 'M3') and mv.start_state is not None:
                existing = mv.start_state.robot_configuration
                if existing is None:
                    self.get_logger().warn(
                        f"{source} {mv.movement_id!r} has no propagated start_conf; "
                        "rejecting trajectory."
                    )
                    mv.trajectory = None
                    return
                diff = float(np.abs(start_vec - vec12_from_conf(existing)).max())
                if diff > 1e-3:
                    self.get_logger().warn(
                        f"{source} start of {mv.movement_id!r} differs from "
                        f"propagated start_conf by max {diff:.4f} rad/m; "
                        "rejecting trajectory."
                    )
                    mv.trajectory = None
                    return
            else:
                # M1 owns its generated start_conf; M0/M4 keep the legacy
                # behavior of mirroring trajectory start into start_state.
                mv.start_state.robot_configuration = conf_from_12vec(start_vec)

            # Step (3) forward-chain propagation — role-based:
            #   M1/M2/M3: strict chain owners; ALWAYS overwrite next.start
            #     with traj[-1] (warn first if there's an existing value).
            #   M0/M4:    NOT part of the chain. M0 stages live -> M1.start
            #     (M1 owns its own start_conf via its plan), M4 is the
            #     sequence terminator. Neither writes the next list-index
            #     movement's start_state.robot_configuration.
            if chain_role in ('M0', 'M4'):
                pass
            elif self.current_movement_index + 1 < len(self._loaded_movements):
                next_mv = self._loaded_movements[self.current_movement_index + 1]
                if next_mv.start_state is not None:
                    existing = next_mv.start_state.robot_configuration
                    new_end = conf_from_12vec(path[-1])
                    existing_vec = None
                    if existing is not None:
                        existing_vec = vec12_from_conf(existing)
                    elif self._trajectory_has_waypoints(next_mv):
                        # If next.start_state has not been populated yet, its
                        # loaded trajectory still owns the effective start.
                        existing_vec = path_12_from_joint_trajectory(next_mv.trajectory)[0]
                    if existing_vec is None:
                        next_mv.start_state.robot_configuration = new_end
                        print(
                            f"[{source}] propagated {mv.movement_id!r}.traj[-1] "
                            f"-> {next_mv.movement_id!r}."
                            f"start_state.robot_configuration (was None)."
                        )
                    else:
                        diff = np.abs(path[-1] - existing_vec).max()
                        if diff > 1e-3:
                            self.get_logger().warn(
                                f"{source} end of {mv.movement_id!r} differs from "
                                f"existing {next_mv.movement_id!r}.start by "
                                f"max {diff:.4f} rad/m; overwriting "
                                f"(M1/M2/M3 chain rule)."
                            )
                            if chain_role == 'M1':
                                self._drop_m2_m3_after_m1_chain_break(
                                    f"{source} M1 endpoint changed by max {diff:.4f} rad/m"
                                )
                            elif chain_role == 'M2' and self._match_movement_role(next_mv) == 'M3':
                                self._drop_movement_trajectory(
                                    next_mv,
                                    f"{source} M2 endpoint changed by max {diff:.4f} rad/m"
                                )
                        next_mv.start_state.robot_configuration = new_end

            # Backward continuity check: previous movement's last traj point
            # should match this movement's first traj point.
            if self.current_movement_index > 0:
                prev_mv = self._loaded_movements[self.current_movement_index - 1]
                prev_jt = getattr(prev_mv, 'trajectory', None)
                if prev_jt is not None:
                    prev_path = path_12_from_joint_trajectory(prev_jt)
                    if prev_path:
                        diff = float(np.abs(
                            np.asarray(prev_path[-1]) - np.asarray(path[0])
                        ).max())
                        if diff > 1e-3:
                            self.get_logger().warn(
                                f"{source} start of {mv.movement_id!r} differs "
                                f"from {prev_mv.movement_id!r}.trajectory[-1] "
                                f"by max {diff:.4f} rad/m."
                            )
                        else:
                            print(
                                f"[{source}] start agrees with "
                                f"{prev_mv.movement_id!r}.trajectory[-1] "
                                f"(max diff {diff:.6f})."
                            )

        self.planned_arm_trajectory = [
            (np.asarray([q[:6] for q in path]), None, self.trajectory_time, None),
            (np.asarray([q[6:] for q in path]), None, self.trajectory_time, None),
        ]
        self.set_to_show_traj_state()
        tag = f"{source}{' ' + role if role else ''}"
        print(f"[{tag}] {mv.movement_id!r}: {len(path)} waypoints stored.")
        self._validate_cdfm_planned_path(mv, path)

        # M0's goal is wherever M1 starts. Once M1's trajectory is accepted
        # (its start_state now carries a planned robot_configuration), copy
        # that configuration into M0.target_configuration so M0 can plan
        # without re-loading the BarAction.
        if role == 'M1' or self._match_movement_role(mv) == 'M1':
            self._backfill_m0_target_from_m1()

        self._print_movement_roster(tag=tag)

    def _backfill_m0_target_from_m1(self):
        """Set M0.target_configuration = M1.start_state.robot_configuration.

        The authored M0 has no target of its own (the producer can't know
        the planned M1 start). No-op when there's no M0/M1 pair or M1's
        start configuration is still missing.
        """
        movements = self._loaded_movements or []
        m0 = next((m for m in movements if self._match_movement_role(m) == 'M0'), None)
        m1 = next((m for m in movements if self._match_movement_role(m) == 'M1'), None)
        if m0 is None or m1 is None:
            return
        if m1.start_state is None or m1.start_state.robot_configuration is None:
            return
        m0.target_configuration = m1.start_state.robot_configuration
        print(f"[backfill] M0.target_configuration <- "
              f"{m1.movement_id!r}.start_state.robot_configuration.")

    def load_selected_movement_trajectory(self):
        """Push the currently loaded movement's in-memory trajectory into the viz.

        Reads ``mv.trajectory`` (populated when the BarAction JSON was
        loaded -- either the clean file with authored trajectories, or a
        ``<action>.live-solved.json`` sidecar written by
        ``plan_movement_chain_live``). Wires ``planned_arm_trajectory`` so the
        traj-viz time slider previews it, then routes through
        ``_accept_trajectory`` so forward-chain propagation + backward
        continuity checks match what fresh planning would do.

        No separate per-movement JSON is read: trajectories live on the mv
        object, and persistence is the sidecar path only.
        """
        if self.current_movement is None:
            self.get_logger().warn("No movement loaded; click 'Load Movement' first.")
            return
        mv = self.current_movement
        jt = getattr(mv, 'trajectory', None)
        if jt is None:
            self.get_logger().warn(
                f"{mv.movement_id!r} has no trajectory in memory. Re-load a "
                ".live-solved.json sidecar via 'Load BarAction', or run "
                "'Plan Chain (Live)'."
            )
            return
        print(f"[LoadTraj] using in-memory trajectory for {mv.movement_id!r}")
        self._accept_trajectory(
            mv, jt,
            source='LoadTraj',
            role=self._match_movement_role(mv),
        )

    def _print_movement_roster(self, tag='roster'):
        """Print which loaded movements have a start_conf and a trajectory."""
        print(f"[{tag}] movement roster:")
        for i, m in enumerate(self._loaded_movements):
            has_conf = (m.start_state is not None
                        and getattr(m.start_state, 'robot_configuration', None) is not None)
            has_traj = getattr(m, 'trajectory', None) is not None
            print(f"  [{i}] {m.movement_id!r}")
            print(f"     - start state: has robot_conf = {self._color_bool(has_conf)}")
            print(f"     - has trajectory = {self._color_bool(has_traj)}")

    def _trajectory_has_waypoints(self, mv):
        """Return True only when a movement has a non-empty 12-DOF trajectory."""
        jt = getattr(mv, 'trajectory', None)
        if jt is None:
            return False
        try:
            return bool(path_12_from_joint_trajectory(jt))
        except Exception:
            # If parsing fails, treat any raw points as a trajectory so stale
            # files still get invalidated instead of being silently kept.
            return bool(getattr(jt, 'points', None))

    def _drop_movement_trajectory(self, mv, reason):
        """Clear a movement trajectory in memory.

        With per-movement JSON persistence removed (trajectories now live only
        on the sidecar ``<action>.live-solved.json``), this is memory-only:
        any downstream write goes through the next Plan Chain export.
        """
        had_traj = getattr(mv, 'trajectory', None) is not None
        mv.trajectory = None
        if self._match_movement_role(mv) == 'M1':
            # M1 start_conf is generated by M1 planning; without M1 traj it is
            # stale by definition and must not survive as an authored start.
            if mv.start_state is not None:
                mv.start_state.robot_configuration = None
        if had_traj:
            print(f"[drop-traj] {mv.movement_id!r}: {reason}")

    def _drop_m2_m3_after_m1_chain_break(self, reason):
        """Drop stale downstream linear trajectories after M1 endpoint changes."""
        dropped = 0
        for m in self._loaded_movements:
            if self._match_movement_role(m) in ('M2', 'M3') and self._trajectory_has_waypoints(m):
                self._drop_movement_trajectory(m, reason)
                dropped += 1
        return dropped

    def _clear_m1_start_conf_without_trajectory(self):
        """Keep invariant: M1 has start_conf only when it has a trajectory."""
        for m in self._loaded_movements:
            if self._match_movement_role(m) != 'M1':
                continue
            if m.start_state is None or self._trajectory_has_waypoints(m):
                continue
            if getattr(m.start_state, 'robot_configuration', None) is not None:
                m.start_state.robot_configuration = None
                print(f"[M1] cleared start_state.robot_configuration because M1 has no trajectory.")

    def _color_bool(self, value):
        """Return a terminal-colored bool string for planning status prints."""
        if bool(value):
            return "\033[32mTrue\033[0m"
        return "\033[31mFalse\033[0m"

    def _validate_cdfm_planned_path(self, mv, path12):
        """Run sparse path_validation checks for any planned CDFM path."""
        movement_id = getattr(mv, 'movement_id', '') or ''
        if 'CDFM' not in movement_id:
            return
        if not path12:
            self.get_logger().warn("[CDFM validation] skipped: empty planned path.")
            return

        husky = getattr(self, "_bar_action_husky", None)
        if self.cfab is None or husky is None:
            self.get_logger().warn(f"[CDFM validation] skipped for {movement_id!r}: cfab pp robot is unavailable.")
            return
        state = getattr(mv, 'start_state', None)
        bar_rb = (state.rigid_body_states.get(self.active_bar_name)
                  if state is not None and self.active_bar_name else None)
        if bar_rb is None or bar_rb.attached_to_link is None or bar_rb.attachment_frame is None:
            self.get_logger().warn(
                f"[CDFM validation] skipped for {movement_id!r}: bar not "
                f"attached in start_state.")
            return

        # ! Keep these two imports deferred (function-level). Importing these
        # ! modules creates/truncates log files as an import-time side effect,
        # ! which we must not trigger just by loading husky_monitor.
        from husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.core import STAGE3_GRASP_MASK_LINKS
        from husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.path_validation import validate_stage_trajectory

        saved_client = pp.CLIENT
        pp.CLIENT = self.cfab.client.client_id
        pp.CLIENTS.setdefault(pp.CLIENT, True)
        try:
            robot = husky.object.robot
            joint_names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]) + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
            arm_joints = pp.joints_from_names(robot, joint_names)
            tool_link_left = pp.link_from_name(robot, "left_ur_arm_tool0")
            attach_link = pp.link_from_name(robot, bar_rb.attached_to_link)
            attach_pose = pose_from_frame(bar_rb.attachment_frame)

            # Everything the sparse validator used to read from the (now
            # removed) plan ctx is re-derived here from the movement's own
            # start_state: bar world pose per waypoint via FK on the pp-side
            # robot, grasp at the first waypoint, obstacles from the cell.
            with pp.WorldSaver():
                pose_path = []
                for q in path12:
                    pp.set_joint_positions(robot, arm_joints, np.asarray(q, dtype=float))
                    pose_path.append(pp.multiply(
                        pp.get_link_pose(robot, attach_link), attach_pose))
                pp.set_joint_positions(robot, arm_joints, np.asarray(path12[0], dtype=float))
                grasp_bar_from_left = pp.multiply(
                    pp.invert(pose_path[0]), pp.get_link_pose(robot, tool_link_left))
            obstacles = _collect_obstacle_puids(
                self.cfab.planner, exclude={self.active_bar_name})

            scene = {
                "robot": robot,
                "arm_joints": arm_joints,
                "tool_link_left": tool_link_left,
                "tool_link_right": pp.link_from_name(robot, "right_ur_arm_tool0"),
                # Keep the scene shaped like run.py even though sparse mode
                # only consumes robot/joints/tool links.
                "bar_body": self.active_bar_body,
                "grasp_bar_from_left": grasp_bar_from_left,
                "collision_obstacles": obstacles,
                "bar_label": self.active_bar_name,
            }
            validation = validate_stage_trajectory(
                stage=M1_PLANNER_STAGE,
                scene=scene,
                path=pose_path,
                joint_path=[np.asarray(q, dtype=float) for q in path12],
                original_joint_path=None,
                joint_path_source="monitor_planned_path",
                joint_path_reason=None,
                urdf_path=HUSKY_DUAL_URDF_PATH,
                srdf_path=HUSKY_DUAL_SRDF_PATH,
                grasp_mask_links=STAGE3_GRASP_MASK_LINKS,
                target_label=self.active_bar_name,
                position_res=M1_POSITION_RES,
                rotation_res=M1_ROTATION_RES,
                dense_joint_validation_step_rad=0.0,
                skip_dense_collision_checks=True,
                # Monitor validation is visual-only: show the plot in the
                # live GUI monitor, never write a PNG report. Headless runs
                # skip the plot (no display; Qt would abort the process).
                save_plot=False,
                show_plot=bool(getattr(self, '_is_live_monitor', False)),
            )
        except Exception as exc:
            self.get_logger().warn(f"[CDFM validation] failed for {movement_id!r}: {exc}")
            return
        finally:
            pp.CLIENT = saved_client

        wrap_count = int(validation.get("raw_wrap_segment_count") or 0)
        rel_ok = validation.get("relative_transform_ok")
        joint_ok = validation.get("joint_continuity_ok")
        max_dq = validation.get("joint_continuity_max_delta_rad")
        max_trans = validation.get("relative_transform_max_translation_m")
        max_axis = validation.get("relative_transform_max_axis_angle_deg") or {}
        max_axis_deg = max((v for v in max_axis.values() if v is not None), default=None)
        max_dq_text = None if max_dq is None else f"{max_dq:.4f} rad"
        max_trans_text = None if max_trans is None else f"{max_trans * 1000.0:.3f} mm"
        max_axis_text = None if max_axis_deg is None else f"{max_axis_deg:.3f} deg"
        print(
            f"[CDFM validation] {movement_id!r} sparse checks: "
            f"joint_continuity={joint_ok}, raw_wraps={wrap_count}, "
            f"ee_constraint={rel_ok}, max_dq={max_dq_text}, "
            f"ee_trans={max_trans_text}, ee_rot_axis={max_axis_text}"
        )
        if wrap_count or joint_ok is False or rel_ok is False:
            self.get_logger().warn(f"[CDFM validation] sparse validation FAILED for {movement_id!r}.")

    def _plan_M0_dispatch(self, mv):
        """Free dual-arm from live conf -> M0.target (= M1's planned start)."""
        if mv.target_configuration is None:
            # M1's start conf becomes M0's goal once M1 is planned/loaded.
            self._backfill_m0_target_from_m1()
        if mv.target_configuration is None:
            self.get_logger().warn(
                "M0 has no target_configuration; plan M1 first (its start "
                "conf is backfilled as M0's goal).")
            return None
        # M0's start must be the LIVE robot at plan time — the user may have
        # moved it since Load Movement.
        self._inject_live_conf_into_state(mv.start_state)
        try:
            self.cfab.planner.set_robot_cell_state(mv.start_state)
        except Exception as e:
            print(f"[M0] WARN: cfab set_robot_cell_state after live-conf resync failed: {e}")
        # Pause GUI rendering during the search (no-op when headless).
        with pp.LockRenderer():
            path, info = plan_free_dual_arm(
                self.cfab.planner, mv.start_state, mv.target_configuration,
                max_time=120.0, max_iterations=50,
            )
        if path is None:
            print(f"[M0] plan_free_dual_arm failed: {info.get('failure_reason')}")
            return None
        return joint_trajectory_from_path(path)

    def _plan_M1_dispatch(self, mv):
        """Constrained dual-arm (bar held): state-based task-space RRT.

        Grasps, bar pose, obstacles, and collision setup are all derived by
        the planner from mv.start_state; ``derive_start=True`` asks it to
        compute a feasible grasp-consistent start conf (the authored start
        is a placeholder).
        """
        if not self.active_bar_name:
            self.get_logger().warn("M1: active_bar_name not set.")
            return None
        if not mv.target_ee_frames:
            self.get_logger().warn("M1: missing target_ee_frames.")
            return None
        # Prefer the authored M2 start conf as M1's goal: it skips the
        # planner's own goal IK (which can pick a ±2π-wrapped branch) and
        # pins the goal bar pose to the authored conf's FK. Note the joint
        # path's END still follows the derived start's IK branch (upstream
        # pose-RRT behavior), so M2 can still land on a hard seed — replan
        # M1 when M2's linear IK cannot reach its first waypoint.
        goal_conf = None
        m2 = next((m for m in (self._loaded_movements or [])
                   if self._match_movement_role(m) == 'M2'), None)
        if (m2 is not None and m2.start_state is not None
                and m2.start_state.robot_configuration is not None):
            goal_conf = m2.start_state.robot_configuration
            print("[M1] goal_conf <- authored M2 start conf (wrap-safe branch).")
        # Multi-start: when a run fails, retry with a re-seeded derived
        # start and a widened bar sweep box (hard scenes like B226 need a
        # different home bar pose to find a corridor).
        start_retries = 3
        path = info = None
        for retry_idx in range(start_retries):
            extra = {}
            if retry_idx > 0:
                extra = dict(
                    start_random_seed=9973 * retry_idx,
                    start_bar_sweep_box=((-0.4, 0.4), (-0.4, 0.4), (-0.5, 0.3)),
                )
                print(f"[M1] retry {retry_idx + 1}/{start_retries} with re-seeded "
                      f"derived start.")
            # Pause GUI rendering during the search (no-op when headless).
            with pp.LockRenderer():
                path, info = plan_constrained_dual_arm(
                    self.cfab.planner, mv.start_state,
                    active_bar_id=self.active_bar_name,
                    goal_conf=goal_conf,
                    goal_ee_frames=mv.target_ee_frames if goal_conf is None else None,
                    stage=M1_PLANNER_STAGE,
                    position_res=M1_POSITION_RES,
                    rotation_res=M1_ROTATION_RES,
                    max_time=120.0,
                    derive_start=True,
                    **extra,
                )
            if path is not None:
                break
            print(f"[M1] plan_constrained_dual_arm failed: {info.get('failure_reason')}")
        if path is None:
            return None
        # Feed the per-arm display + waypoint-slider consumers (Display slider
        # mode 1, cfab waypoint sliders) from the planned path.
        self.constrained_trajectory = [
            (np.asarray([q[:6] for q in path]), None, self.trajectory_time, None),
            (np.asarray([q[6:] for q in path]), None, self.trajectory_time, None),
        ]
        return joint_trajectory_from_path(path)

    def _plan_M2_dispatch(self, mv):
        """Constrained linear (bar-held): planner derives grasps + bar goal
        pose internally from mv.start_state + the target EE frames."""
        if mv.start_state.robot_configuration is None:
            self.get_logger().warn("M2: missing start conf.")
            return None
        # The API wants exactly one goal kind; prefer the authored EE frames.
        goal_ee = mv.target_ee_frames or None
        goal_conf = mv.target_configuration if goal_ee is None else None
        if goal_ee is None and goal_conf is None:
            self.get_logger().warn("M2: missing target_configuration / target_ee_frames.")
            return None
        # Pause GUI rendering during the IK loop (no-op when headless).
        with pp.LockRenderer():
            jt = plan_constrained_dual_arm_linear(
                self.cfab.planner, mv.start_state,
                active_bar_id=self.active_bar_name,
                goal_conf=goal_conf,
                goal_ee_frames=goal_ee,
                skip_env_collisions=False,
            )
        if jt is not None:
            self._check_inter_ee_invariance(jt, mv.start_state)
        return jt

    def _check_inter_ee_invariance(self, jt, template_state):
        """For an M2 (bar-held) trajectory, verify the left_from_right
        relative pose is constant over the path. Logs max/mean translation
        + rotation drift relative to the first waypoint.
        """
        planner = self.cfab.planner
        path = path_12_from_joint_trajectory(jt)
        if len(path) < 2:
            return
        left_names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0])
        right_names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
        names_12 = left_names + right_names

        state = template_state.copy()
        relatives = []
        for q12 in path:
            for n, v in zip(names_12, q12):
                state.robot_configuration[n] = float(v)
            lf = _fk_link_frame(planner, state, "left_ur_arm_tool0")
            rf = _fk_link_frame(planner, state, "right_ur_arm_tool0")
            T_l = Transformation.from_frame(lf)
            T_r = Transformation.from_frame(rf)
            relatives.append(T_l.inverted() * T_r)

        ref = relatives[0]
        ref_inv = ref.inverted()
        pos_devs = []
        ang_devs = []
        for rel in relatives:
            delta = ref_inv * rel
            tv = list(Frame.from_transformation(delta).point)
            pos_devs.append(float(np.linalg.norm(tv)))
            qw = abs(float(Frame.from_transformation(delta).quaternion.w))
            qw = min(max(qw, 0.0), 1.0)
            ang_devs.append(2.0 * float(np.arccos(qw)))
        pos_max = max(pos_devs); pos_mean = float(np.mean(pos_devs))
        ang_max = max(ang_devs); ang_mean = float(np.mean(ang_devs))
        print(
            f"[M2 inter-EE invariance] over {len(path)} waypoints: "
            f"pos drift max={pos_max*1000:.2f} mm (mean={pos_mean*1000:.2f}); "
            f"rot drift max={np.degrees(ang_max):.3f} deg (mean={np.degrees(ang_mean):.3f})"
        )

    def _plan_M3_dispatch(self, mv):
        """Linear retreat with independent per-arm EE interpolation."""
        if mv.start_state.robot_configuration is None:
            self.get_logger().warn("M3: missing start conf.")
            return None
        goal_ee = mv.target_ee_frames or None
        goal_conf = mv.target_configuration if goal_ee is None else None
        if goal_ee is None and goal_conf is None:
            self.get_logger().warn("M3: missing target_configuration / target_ee_frames.")
            return None
        # Pause GUI rendering during the IK loop (no-op when headless).
        with pp.LockRenderer():
            return plan_dual_arm_linear_independent(
                self.cfab.planner, mv.start_state,
                goal_conf=goal_conf,
                goal_ee_frames=goal_ee,
                skip_env_collisions=False,
            )

    def _plan_M4_dispatch(self, mv):
        """Free dual-arm from M3 end -> fixed home conf.

        The action's authored M4 target is a placeholder; the known-good
        dual-arm home is used instead (matches headless_bar_action_planner).
        """
        if mv.start_state.robot_configuration is None:
            self.get_logger().warn("M4: missing start_state.robot_configuration.")
            return None
        # * Wrap the fixed home 12-vec in a compas Configuration so the tamp
        # helper's dict-indexed extraction works (raw numpy 12-vecs raise
        # IndexError on string joint-name indexing).
        goal_conf = conf_from_12vec(HUSKY_DUAL_ARM_HOME_CONF_12)
        # Pause GUI rendering during the search (no-op when headless).
        with pp.LockRenderer():
            path, info = plan_free_dual_arm(
                self.cfab.planner, mv.start_state, goal_conf, max_time=30.0,
            )
        if path is None:
            print(f"[M4] plan_free_dual_arm failed: {info.get('failure_reason')}")
            return None
        return joint_trajectory_from_path(path)

    def ik_live_base_for_selected_movement(self):
        """IK at the LIVE base for the current movement's START EE frames.

        Intended for M2/M3 (their start_state carries an authored
        robot_configuration, so the start EE frames come from FK). Solves
        dual-arm IK to those world-frame EE poses but for the LIVE base,
        warm-started from the LIVE robot arm conf, with full cfab collision
        checking against the movement's start_state ACM. On success it sets
        goal_arm_pose; the user then clicks 'Plan Both Arms to Goal
        (composite)' to plan a free transit there. Does NOT write mv.trajectory.

        Start EE frames are derived (in order of preference):
          1. FK from mv.start_state.robot_configuration + robot_base_frame
             (stored base, NOT live).
          2. Previous movement's target_ee_frames.

        Returns:
            bool: True on success (goal_arm_pose updated), False on any
            precondition miss or IK failure. Existing UI callers ignore the
            return value; the new ``replan_free_to_movement_start_live``
            uses it to bail cleanly.
        """
        if self.current_movement is None:
            self.get_logger().warn("Load a movement first.")
            return False
        mv = self.current_movement

        # 1) Derive start EE frames.
        start_ee_frames = None
        if mv.start_state is not None and mv.start_state.robot_configuration is not None:
            try:
                self.cfab.planner.set_robot_cell_state(mv.start_state)
                left_frame = _fk_link_frame(self.cfab.planner, mv.start_state, "left_ur_arm_tool0")
                right_frame = _fk_link_frame(self.cfab.planner, mv.start_state, "right_ur_arm_tool0")
                start_ee_frames = {"left": left_frame, "right": right_frame}
                print("[IK Live Base] start EE frames from FK at start_state.")
            except Exception as e:
                self.get_logger().warn(f"FK from start_state failed: {e}")
        if start_ee_frames is None and self.current_movement_index > 0:
            prev = self._loaded_movements[self.current_movement_index - 1]
            if prev.target_ee_frames:
                start_ee_frames = prev.target_ee_frames
                print(f"[IK Live Base] start EE frames from prev mv {prev.movement_id!r} target_ee_frames.")
        # Cache the derived target EE frames on self so
        # `replan_free_to_movement_start_live`'s endpoint verification can
        # compare the composite plan's final tool0 poses back against the
        # authored targets that drove this IK call.
        self._last_ik_target_ee_frames = start_ee_frames
        if not start_ee_frames or 'left' not in start_ee_frames or 'right' not in start_ee_frames:
            self.get_logger().warn(
                "Cannot derive start EE frames (no FK seed in start_state, "
                "no prev-movement target_ee_frames)."
            )
            return False

        # 2) IK at live base using the derived start EE frames. Inject the
        # live base + live arm conf so IK is warm-started from where the
        # robot actually is now (not the movement's authored start conf).
        # Trac_ik may return joint values that are 2*pi-offset from the
        # nearest branch when the seed is far from the target; the
        # composite free plan step downstream unwraps the goal to
        # +/- pi of the start conf so the BiRRT can still connect.
        live_state = mv.start_state.copy()
        hi = self.huskies[self.selected_robot_id].interface
        self._inject_live_conf_into_state(live_state)
        self.cfab.planner.set_robot_cell_state(live_state)
        # Override target_ee_frames so _solve_bar_action_goal_ik uses the
        # start-state derived frames (it reads monitor.target_ee_frames).
        # Also pass mv.start_state.robot_configuration as an alternate IK
        # seed: it's a bar-holding pose whose FK produces the very target
        # frames, so trac_ik seeded there converges to that (or a nearby)
        # collision-free branch, escaping the self-colliding branches
        # trac_ik lands on when seeded from the extended-arm HOME conf.
        alt_seed = None
        if mv.start_state.robot_configuration is not None:
            try:
                alt_seed = vec12_from_conf(mv.start_state.robot_configuration)
            except Exception:
                alt_seed = None
        saved_targets = self.target_ee_frames
        self.target_ee_frames = start_ee_frames
        try:
            conf12 = _solve_bar_action_goal_ik(
                self, live_state, skip_env_collisions=False, verbose=False,
                alt_seed_conf12=alt_seed,
            )
        finally:
            self.target_ee_frames = saved_targets

        if conf12 is None:
            self.get_logger().warn("IK at live base FAILED.")
            return False
        self.goal_arm_pose[0] = np.asarray(conf12[:6])
        self.goal_arm_pose[1] = np.asarray(conf12[6:])
        # Ghost must render live_base + IK conf together; otherwise the
        # ghost's tool0 drifts (live_base != start_state base, so rendering
        # stored-base + IK-conf gives a different tool0).
        self.goal_base_pose = (hi.position, hi.rotation)

        # Self-test: FK at the GOAL state (live_base + IK_conf, set by
        # _solve_bar_action_goal_ik on monitor.movement_goal_state). Do NOT
        # use the local live_state here — _solve_bar_action_goal_ik writes
        # the new conf onto a copy, so live_state.robot_configuration is
        # still the OLD seed conf, which would FK to (live_base * FK(old))
        # — i.e. the target offset by exactly the base offset, masking a
        # successful IK as an apparent failure.
        gs = getattr(self, 'movement_goal_state', None)
        try:
            fk_left = _fk_link_frame(self.cfab.planner, gs, "left_ur_arm_tool0")
            fk_right = _fk_link_frame(self.cfab.planner, gs, "right_ur_arm_tool0")
            def _residual(fk_frame, tg_frame):
                d_pos = float(np.linalg.norm(
                    np.asarray(fk_frame.point) - np.asarray(tg_frame.point)
                ))
                q_fk = np.asarray(fk_frame.quaternion.xyzw, dtype=float)
                q_tg = np.asarray(tg_frame.quaternion.xyzw, dtype=float)
                d_ang = 2.0 * float(np.arccos(
                    np.clip(abs(float(np.dot(q_fk, q_tg))), 0.0, 1.0)
                ))
                return d_pos, d_ang
            d_pos_L, d_ang_L = _residual(fk_left, start_ee_frames['left'])
            d_pos_R, d_ang_R = _residual(fk_right, start_ee_frames['right'])
            print(
                f"[IK Live Base] FK self-test residual: "
                f"L pos={d_pos_L*1000:.2f} mm ang={np.degrees(d_ang_L):.3f} deg | "
                f"R pos={d_pos_R*1000:.2f} mm ang={np.degrees(d_ang_R):.3f} deg"
            )
        except Exception as e:
            self.get_logger().warn(f"FK self-test failed: {e}")

        self.reset_ui(self.goal_arm_pose)
        self.set_to_show_goal_state()
        print("[IK Live Base] OK - goal_arm_pose updated (start-EE targets); "
              "click composite plan to drive.")
        return True

    def record_bar_holding_marker_take(self):
        """Record one labeled-marker take + run inline fit + log deviation."""
        world.request_marketset_button(self, MOCAP_SET_RIG_RB_NAME)

    def record_bar_take_with_shared_viz(self):
        """Record a bar marker take, fit a line, and viz via the shared
        ``mocap_experiment.draw_marker_take_in_pp`` helper.

        Same record-target as record_bar_holding_marker_take (the 'bar_rig'
        labeled-marker set, persisted to ``self.marker_set_data`` so the
        existing 'Save markerset data' button picks it up), but the drawing
        goes through the same helper that the offline
        ``data/bar_holding_acc_data/1_compare_to_cell_state.py`` script uses
        (red marker points + blue fitted bar line), so live and offline
        visuals match.
        """
        rb_mocap_name = MOCAP_SET_RIG_RB_NAME
        if rb_mocap_name not in self._mocap_labeled_marker_cache:
            self.get_logger().warn(f'Mocap {rb_mocap_name} not found!')
            return
        labeled = copy.deepcopy(self._mocap_labeled_marker_cache[rb_mocap_name])
        # Minimal take payload; matches the field offline analysis reads.
        self.marker_set_data.append({rb_mocap_name: labeled})

        try:
            fit = fit_bar_from_markerset(labeled)
        except Exception as e:
            self.get_logger().warn(f"bar take fit failed: {e}")
            return
        uids = draw_marker_take_in_pp(labeled, fit)
        self._bar_holding_fit_line_uids.extend(uids)
        ocf = fit['ocf_position']
        self.get_logger().info(
            f"[bar take, shared viz] ocf=({ocf[0]:.3f},{ocf[1]:.3f},{ocf[2]:.3f}) m | "
            f"max_resid={fit['center_to_line_dist_max_m']*1000:.2f} mm | "
            f"bar_len={fit['bar_length_observed']:.4f} m"
        )

    def save_bar_holding_marker_data(self):
        """Save accumulated marker takes to the gdrive experiment dir; clear viz."""
        world.save_markerset_data(self, use_experiment_dir=True)
        self.marker_set_data = []
        for uid in self._bar_holding_fit_line_uids:
            try:
                pp.remove_debug(uid)
            except Exception:
                pass
        self._bar_holding_fit_line_uids = []

    def _build_trajectory_waypoint_sliders(self):
        """Add up to two "step through waypoints" sliders on the cfab PyBullet
        window so you can inspect a planned trajectory pose by pose.

        Each planned trajectory is a list of waypoints (robot configurations).
        This builds one slider per available trajectory - one for the staging
        (free) path and one for the constrained path. Dragging a slider moves
        the on-screen robot to the corresponding waypoint, so you can visually
        walk through the plan and check for problems before executing it.

        To make that instant while dragging, the full RobotCellState for every
        waypoint is precomputed here and cached in
        self._trajectory_waypoint_sliders. The cache is read every frame by
        _service_trajectory_waypoint_sliders (called from update())."""
        left_names = HUSKY_DUAL_UR5e_JOINT_NAMES[0]
        right_names = HUSKY_DUAL_UR5e_JOINT_NAMES[1]
        client_id = self.cfab.client.client_id

        def _build_waypoint_states(traj):
            if traj is None or traj[0] is None or traj[1] is None:
                return []
            left_path = traj[0][0]
            right_path = traj[1][0]
            n = len(left_path)
            if n < 1 or n != len(right_path):
                return []
            states = []
            for i in range(n):
                wp = self.movement_start_state.copy()
                for j, name in enumerate(left_names):
                    wp.robot_configuration[name] = float(left_path[i][j])
                for j, name in enumerate(right_names):
                    wp.robot_configuration[name] = float(right_path[i][j])
                states.append(wp)
            return states

        staging_states = _build_waypoint_states(getattr(self, "staging_free_trajectory", None))
        constrained_states = _build_waypoint_states(getattr(self, "constrained_trajectory", None))

        ns = len(staging_states)
        nc = len(constrained_states)
        if ns == 0 and nc == 0:
            self._trajectory_waypoint_sliders = None
            return

        staging_slider = None
        constrained_slider = None
        if ns > 0:
            staging_slider = p.addUserDebugParameter(
                f"Staging t (0..{ns-1})", 0.0, float(max(ns - 1, 0)), 0.0,
                physicsClientId=client_id,
            )
        if nc > 0:
            constrained_slider = p.addUserDebugParameter(
                f"Constrained t (0..{nc-1})", 0.0, float(max(nc - 1, 0)), 0.0,
                physicsClientId=client_id,
            )
        self._trajectory_waypoint_sliders = {
            "client_id": client_id,
            "staging_slider": staging_slider,
            "constrained_slider": constrained_slider,
            "staging_states": staging_states,
            "constrained_states": constrained_states,
            "last_staging": -1,
            "last_constrained": -1,
        }
        print(f"[waypoint sliders] '{self.current_movement.movement_id}' plan loaded: "
              f"staging={ns} wp, constrained={nc} wp. Drag the sliders on the "
              f"cfab PyBullet panel to step through the waypoints.")

    def _service_trajectory_waypoint_sliders(self):
        """Read the waypoint sliders once per frame and, when a slider has been
        dragged to a new waypoint index, re-pose the cfab scene to that
        waypoint. Does nothing when no waypoint sliders are active."""
        s = self._trajectory_waypoint_sliders
        if s is None or self.cfab is None:
            return
        if self.cfab.client.client_id != s["client_id"]:
            self._trajectory_waypoint_sliders = None
            return
        cid = s["client_id"]
        if s["staging_slider"] is not None:
            t = p.readUserDebugParameter(s["staging_slider"], physicsClientId=cid)
            n = len(s["staging_states"])
            idx = max(0, min(n - 1, int(round(t))))
            if idx != s["last_staging"]:
                self.cfab.planner.set_robot_cell_state(s["staging_states"][idx])
                s["last_staging"] = idx
        if s["constrained_slider"] is not None:
            t = p.readUserDebugParameter(s["constrained_slider"], physicsClientId=cid)
            n = len(s["constrained_states"])
            idx = max(0, min(n - 1, int(round(t))))
            if idx != s["last_constrained"]:
                self.cfab.planner.set_robot_cell_state(s["constrained_states"][idx])
                s["last_constrained"] = idx

    def get_active_bar_aabb_dims(self):
        """AABB extents (m) of the active bar mesh from the RobotCell model.

        Used by the constrained planner to seed RRT feature points. Cached
        on first call.
        """
        if self.active_bar_aabb_dims is not None:
            return self.active_bar_aabb_dims
        if self.cfab is None or self.active_bar_name is None:
            return None
        rb_model = self.cfab.robot_cell.rigid_body_models.get(self.active_bar_name)
        if rb_model is None:
            return None
        # Walk visual meshes (in meters) and compute the per-axis extents.
        try:
            meshes = getattr(rb_model, 'visual_meshes_in_meters', None) or []
            if not meshes:
                meshes = getattr(rb_model, 'collision_meshes_in_meters', None) or []
            if not meshes:
                return None
            xs, ys, zs = [], [], []
            for m in meshes:
                for v in m.vertices():
                    pt = m.vertex_coordinates(v)
                    xs.append(pt[0]); ys.append(pt[1]); zs.append(pt[2])
            if not xs:
                return None
            self.active_bar_aabb_dims = (
                max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs),
            )
            return self.active_bar_aabb_dims
        except Exception as e:
            print(f"WARN: failed to compute active bar AABB: {e}")
            return None


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
        # Cache for downstream logging / filenames (e.g., calibration record suffix)
        self.selected_trajectory_file = selected_trajectory_file
        trajectory_filepath = os.path.join(
            DESIGN_DATA_DIRECTORY,
            DESIGN_PROBLEM_NAME,
            'Trajectories',
            selected_trajectory_file
        )
        
        print(f"Loading joint trajectory: {selected_trajectory_file}")
        
        try:
            # Load the joint trajectory using standard json
            with open(trajectory_filepath, 'r') as f:
                joint_trajectory_data = json.load(f)
            
            # Extract trajectory data
            if 'data' in joint_trajectory_data and 'points' in joint_trajectory_data['data']:
                points = joint_trajectory_data['data']['points']
                
                # Get joint names from the trajectory
                if points and 'joint_names' in points[0]:
                    joint_names = points[0]['joint_names']
                    
                    # Find indices for left and right arm joints
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
                    
                    print(f"[Load Joint Traj] dual-arm trajectory: "
                          f"{len(left_arm_trajectory)} waypoints "
                          f"(left={len(left_arm_trajectory)}, "
                          f"right={len(right_arm_trajectory)}) "
                          f"from {selected_trajectory_file}")
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
        if 0 <= new_index < len(self.available_bar_actions):
            self.selected_state_index = new_index
            print(f"Selected state: {self.available_bar_actions[self.selected_state_index]}")

    def update_trajectory_index(self, trajectory_index):
        """
        Update the selected joint trajectory index.
        """
        new_index = int(trajectory_index)
        if 0 <= new_index < len(self.available_joint_trajectories):
            self.selected_trajectory_index = new_index
            self.selected_trajectory_file = self.available_joint_trajectories[self.selected_trajectory_index]
            print(f"Selected trajectory: {self.available_joint_trajectories[self.selected_trajectory_index]}")

    def _load_available_bar_actions(self):
        """Return sorted *.json BarAction filenames under <problem>/BarActions/.

        Attribute is kept under the legacy name for back-compat with
        UI/widgets and existing callers; contents are now BarAction files.
        """
        action_dir = os.path.join(
            DESIGN_DATA_DIRECTORY, DESIGN_PROBLEM_NAME, 'BarActions',
        )
        files = list_bar_actions(action_dir)
        if not files:
            print(f"No BarAction *.json files under: {action_dir}")
            return []
        print(f"Found {len(files)} BarAction files:")
        for i, fname in enumerate(files):
            print(f"  {i}: {fname}")
        return files

    def _load_available_joint_trajectories(self):
        """
        Load available JointTrajectory files from the hardcoded directory.
        """
        trajectory_dir = os.path.join(
            DESIGN_DATA_DIRECTORY,
            DESIGN_PROBLEM_NAME,
            'Trajectories'
        )

        if not os.path.exists(trajectory_dir):
            print(f"Trajectories directory does not exist: {trajectory_dir}")
            return []

        trajectory_files = [f for f in os.listdir(trajectory_dir) if f.endswith('.json')]
        trajectory_files.sort()

        print(f"Found {len(trajectory_files)} joint trajectory files:")
        for i, filename in enumerate(trajectory_files):
            print(f"  {i}: {filename}")

        return trajectory_files

    # --- CALIBRATION state/trajectory loaders (CALIBRATION_STATE_SET) ---
    def _calibration_state_dir(self):
        state_set = CALIBRATION_STATE_SETS.get(
            self.selected_arm_index, CALIBRATION_STATE_SETS[0])
        return os.path.join(
            DESIGN_DATA_DIRECTORY, state_set, 'RobotCellStates',
        )

    def _load_available_calibration_states(self):
        """Return sorted *_RobotCellState.json filenames in the calib state dir."""
        d = self._calibration_state_dir()
        if not os.path.exists(d):
            print(f"Calib state dir missing: {d}")
            return []
        files = sorted(f for f in os.listdir(d) if f.endswith('_RobotCellState.json'))
        print(f"Found {len(files)} calib RobotCellState files under {d}")
        for i, f in enumerate(files):
            print(f"  {i}: {f}")
        return files

    def _load_available_calibration_trajectories(self):
        """Return sorted *_JointTrajectory.json filenames in the calib state dir."""
        d = self._calibration_state_dir()
        if not os.path.exists(d):
            return []
        files = sorted(f for f in os.listdir(d) if f.endswith('_JointTrajectory.json'))
        print(f"Found {len(files)} calib JointTrajectory files under {d}")
        for i, f in enumerate(files):
            print(f"  {i}: {f}")
        return files

    def update_calibration_state_index(self, state_index):
        new_index = int(state_index)
        if 0 <= new_index < len(self.available_calibration_states):
            self.selected_calibration_state_index = new_index
            print(f"Selected calib state: {self.available_calibration_states[new_index]}")

    def update_calibration_trajectory_index(self, trajectory_index):
        new_index = int(trajectory_index)
        if 0 <= new_index < len(self.available_calibration_trajectories):
            self.selected_calibration_trajectory_index = new_index
            print(f"Selected calib trajectory: {self.available_calibration_trajectories[new_index]}")

    def load_calibration_state(self):
        """Load a RobotCellState and set goal_arm_pose / goal_base_pose from it."""
        if not self.available_calibration_states:
            print("No calib robot cell states available!")
            return
        if self.selected_calibration_state_index >= len(self.available_calibration_states):
            print(f"Invalid calib state index: {self.selected_calibration_state_index}")
            return
        selected = self.available_calibration_states[self.selected_calibration_state_index]
        filepath = os.path.join(self._calibration_state_dir(), selected)
        print(f"Loading calib RobotCellState: {selected}")
        try:
            state = json_load(filepath)
            if hasattr(state, 'robot_configuration') and state.robot_configuration is not None:
                cfg = state.robot_configuration
                if hasattr(cfg, 'joint_names') and hasattr(cfg, 'joint_values'):
                    # Auto-detect flavour like load_calibration_trajectory:
                    # single-arm cfg uses un-prefixed ur_arm_* (-> slot 0),
                    # dual-arm cfg uses left_/right_ prefixed names.
                    cfg_names = list(cfg.joint_names)

                    def _get(names):
                        return (np.array([cfg[n] for n in names])
                                if all(n in cfg_names for n in names) else None)

                    single = _get(HUSKY_UR5e_JOINT_NAMES)
                    left = _get(HUSKY_DUAL_UR5e_JOINT_NAMES[0])
                    right = _get(HUSKY_DUAL_UR5e_JOINT_NAMES[1])

                    if left is not None or right is not None:
                        if left is not None:
                            self.goal_arm_pose[0] = left
                        if right is not None:
                            self.goal_arm_pose[1] = right
                    elif single is not None:
                        self.goal_arm_pose[0] = single  # single-arm robot -> slot 0
                    else:
                        print(f"WARN: could not extract arm joint values; got "
                              f"left={0 if left is None else 6} "
                              f"right={0 if right is None else 6} "
                              f"single={0 if single is None else 6}")
                        single = left = right = None

                    if single is not None or left is not None or right is not None:
                        self.reset_ui(self.goal_arm_pose)
                        print(f"goal_arm_pose updated from {selected}")
                        print(f"  left:  {self.goal_arm_pose[0]}")
                        print(f"  right: {self.goal_arm_pose[1]}")
                        self.set_to_show_goal_state()
                else:
                    print("Robot configuration missing joint_names/joint_values")
            else:
                print("RobotCellState has no robot_configuration")
            if hasattr(state, 'robot_base_frame') and state.robot_base_frame is not None:
                self.goal_base_pose = pose_from_frame(state.robot_base_frame)
                print(f"goal_base_pose updated from {selected}: {self.goal_base_pose}")
        except Exception as e:
            print(f"Error loading calib RobotCellState: {e}")

    def load_calibration_trajectory(self):
        """Load a JointTrajectory from the calib state dir into planned_arm_trajectory."""
        if not self.available_calibration_trajectories:
            print("No calib joint trajectory files available!")
            return
        if self.selected_calibration_trajectory_index >= len(self.available_calibration_trajectories):
            print(f"Invalid calib trajectory index: {self.selected_calibration_trajectory_index}")
            return
        selected = self.available_calibration_trajectories[self.selected_calibration_trajectory_index]
        # Cache for downstream calib record filename suffix.
        self.selected_trajectory_file = selected
        filepath = os.path.join(self._calibration_state_dir(), selected)
        print(f"Loading calib JointTrajectory: {selected}")
        try:
            with open(filepath, 'r') as f:
                jt = json.load(f)
            if 'data' not in jt or 'points' not in jt['data']:
                print("JointTrajectory missing data.points")
                return
            points = jt['data']['points']
            if not points or 'joint_names' not in points[0]:
                print("JointTrajectory points missing joint_names")
                return
            joint_names = points[0]['joint_names']

            def _extract(idx):
                traj = []
                for pt in points:
                    jv = pt.get('joint_values')
                    if jv is None:
                        continue
                    traj.append(np.array([jv[i] for i in idx]))
                return traj

            def _match(names):
                # return trajectory if all 6 named joints present, else None
                idx = [joint_names.index(n) for n in names if n in joint_names]
                return (_extract(idx), len(idx))

            # Auto-detect flavour: single-arm files use un-prefixed ur_arm_*
            # names (-> slot 0); dual-arm files use left_/right_ prefixed names.
            single_traj, single_n = _match(HUSKY_UR5e_JOINT_NAMES)
            left_traj, left_n = _match(HUSKY_DUAL_UR5e_JOINT_NAMES[0])
            right_traj, right_n = _match(HUSKY_DUAL_UR5e_JOINT_NAMES[1])

            NONE = (None, None, None, None)
            if left_n == 6 or right_n == 6:
                # left and/or right arm; absent arm -> NONE (no ghost arm)
                lt = (left_traj, None, self.trajectory_time, None) if left_n == 6 else NONE
                rt = (right_traj, None, self.trajectory_time, None) if right_n == 6 else NONE
                self.set_arm_trajectory(lt, index=0)
                self.set_arm_trajectory(rt, index=1)
                n_wp = len(left_traj) if left_n == 6 else len(right_traj)
            elif single_n == 6:
                # single-arm file -> slot 0 only
                self.set_arm_trajectory((single_traj, None, self.trajectory_time, None), index=0)
                self.set_arm_trajectory(NONE, index=1)
                n_wp = len(single_traj)
            else:
                print(f"WARN: no complete arm (6 joints); got "
                      f"left={left_n} right={right_n} single={single_n}; aborting load")
                return
            self.set_to_show_traj_state()
            print(f"[Load Calib Traj] {n_wp} waypoints from {selected}")
        except Exception as e:
            print(f"Error loading calib JointTrajectory: {e}")

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
        ee_types = first_husky.object.ee_types

        # Load only the goal model that matches the actual robot configuration
        with pp.LockRenderer():
            with pp.HideOutput():
                if dual_arm:
                    # Load dual arm goal model
                    self.goal_model = HuskyObject(
                        calibration=calibration, 
                        dual_arm=True, 
                        ee_types=ee_types,  # Use all types for dual arm
                        force_regenerate=False,
                        punch_tool_offset=[self.get_punch_tool_offset(0), self.get_punch_tool_offset(1)]
                    )
                    self.goal_model_single = None  # Not needed for dual arm
                    self.goal_model_dual = self.goal_model
                else:
                    # Load single arm goal model
                    self.goal_model = HuskyObject(
                        calibration=calibration, 
                        dual_arm=False, 
                        ee_types=ee_types[:1] if ee_types else None,  # Take first type for single arm
                        force_regenerate=False,
                        punch_tool_offset=self.get_punch_tool_offset(0)
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

    # --- mocap base XYZ offset side-window (standalone DPG) ---
    def _init_mocap_offset_window(self):
        """Spawn standalone DPG window with x/y/z text inputs + Apply/Reset.
        Independent of _common._global_backend so PyBullet primary UI is unaffected.

        Idempotent: build_ui runs both at __init__ AND on every reset_ui (e.g.
        BarAction load). dpg.create_context() must NOT be called twice — the
        second call corrupts DPG's C state and SEGFAULTS the process. Early
        return if the context was already set up on a prior build_ui pass.
        """
        if getattr(self, '_offset_dpg', None) is not None:
            return
        self._offset_dpg = None
        self._mocap_offset_pending = [0.0, 0.0, 0.0]

        # Avoid a 2nd DPG create_context() when primary backend is already DPG.
        if isinstance(_common._global_backend, DearPyGuiBackend):
            print("[mocap offset] primary backend is DPG; skipping private offset window.")
            return
        try:
            # Lazy/optional import: dearpygui is only needed for this offset
            # window and may not be installed, so keep it function-level.
            import dearpygui.dearpygui as dpg
        except ImportError:
            print("[mocap offset] dearpygui not installed; offset textboxes disabled. "
                  "`pip install dearpygui` to enable.")
            return

        self._offset_dpg = dpg
        dpg.create_context()
        dpg.create_viewport(title="Husky Base Mocap Offset", width=340, height=220)
        bind_default_font(dpg, int(self.UI_FONT_SIZE))
        dpg.setup_dearpygui()
        with dpg.window(tag="offset_window", label="Base XYZ Offset (world, m)",
                        width=340, height=220, no_close=True):
            dpg.add_input_float(tag="offset_x", label="x [m]", default_value=0.0,
                                step=0.0, format="%.4f",
                                callback=lambda s, a, u: self._set_pending_offset(0, a))
            dpg.add_input_float(tag="offset_y", label="y [m]", default_value=0.0,
                                step=0.0, format="%.4f",
                                callback=lambda s, a, u: self._set_pending_offset(1, a))
            dpg.add_input_float(tag="offset_z", label="z [m]", default_value=0.0,
                                step=0.0, format="%.4f",
                                callback=lambda s, a, u: self._set_pending_offset(2, a))
            dpg.add_separator()
            dpg.add_button(label="Apply", callback=lambda *a: self._apply_base_offset())
            dpg.add_button(label="Reset to Zero", callback=lambda *a: self._reset_base_offset())
        dpg.set_primary_window("offset_window", True)
        dpg.show_viewport()

    def _set_pending_offset(self, i, v):
        try:
            self._mocap_offset_pending[i] = float(v)
        except (TypeError, ValueError):
            pass

    def _apply_base_offset(self):
        h = self.huskies[self.selected_robot_id]
        h.mocap_base_offset_xyz = np.array(self._mocap_offset_pending, dtype=float)
        print(f"[mocap offset] applied: {h.mocap_base_offset_xyz.tolist()}")

    def _reset_base_offset(self):
        h = self.huskies[self.selected_robot_id]
        h.mocap_base_offset_xyz = np.zeros(3)
        self._mocap_offset_pending = [0.0, 0.0, 0.0]
        if self._offset_dpg is not None:
            dpg = self._offset_dpg
            for tag in ("offset_x", "offset_y", "offset_z"):
                if dpg.does_item_exist(tag):
                    dpg.set_value(tag, 0.0)
        print("[mocap offset] reset to zero")

    def _pump_mocap_offset_window(self):
        dpg = getattr(self, '_offset_dpg', None)
        if dpg is None:
            return
        if dpg.is_dearpygui_running():
            dpg.render_dearpygui_frame()

    def _shutdown_mocap_offset_window(self):
        dpg = getattr(self, '_offset_dpg', None)
        if dpg is not None:
            dpg.destroy_context()
            self._offset_dpg = None

    def build_ui(self, target_conf=None):
        arm_slider_label = "arm id (0 only)" if self.get_active_arm_count() == 1 else "arm id (0:L,1:R)"
        arm_slider_max = 1   # integer 0/1; single-arm extra clips to 0 in update_selected_arm_id
        self.arm_slider = Slider(arm_slider_label, self.update_selected_arm_id,
                                 0, arm_slider_max, self.selected_arm_index, integer=True)

        self.trajectory_time_slider = Slider("traj time", self.update_trajectory_time, 1.0, self.trajectory_time_max, self.trajectory_time)

        # self.time_slider = p.addUserDebugParameter("Traj viz time", 0.0, 1.0, 1.0)
        # Shim Slider: a PyBullet debug param in PyBullet mode, a DPG widget in DPG
        # mode (so it lives in whichever GUI is active). Its callback updates
        # self.traj_viz_time, which the preview reads in update().
        self.traj_viz_time_slider = Slider("Traj viz time", self.update_traj_viz_time, 0.0, 1.0, 1.0)

        # Live joint-angle stream: the button toggles a SEPARATE floating window
        # showing every joint of the active robot as color-chipped text (radians
        # + degrees) plus a continually-recording scrolling plot. The window is
        # hidden until toggled and only records while shown. Dear PyGui only.
        self.buttons.append(Button("Toggle Joint Live Stream", self.toggle_joint_live_stream))
        if self.USE_DPG_UI:
            # Restore the last shown/hidden choice across UI rebuilds (reset_ui).
            visible = getattr(self, '_joint_stream_visible', False)
            _common._global_backend.add_window(
                "Joint Live Stream", tag="joint_stream_window",
                width=560, height=620, show=visible)
            self.joint_stream_plot = LiveMultiPlot(
                "joints", self._joint_stream_source, self._joint_stream_labels(),
                header_source=lambda: self.huskies[self.selected_robot_id].name,
                parent="joint_stream_window", group_size=6)
            self.joint_stream_plot.set_visible(visible)
        else:
            self.joint_stream_plot = None

        self.buttons.append(Button('Toggle Goal/Trajectory', self.toggle_show_goal_state))
        self.buttons.append(Button('Reset Goal State', self.reset_ui))
                      
        self.buttons.append(Button('Plan S.Arm to conf target', self.plan_single_arm_to_goal_action))
        self.buttons.append(Button('Exec S.Arm Traj', self.execute_arm_trajectory))

        # Add buttons for planning both arms to goal (sequential and composite)
        # self.buttons.append(Button('Plan Both Arms to Goal (sequential)', lambda: world.plan_both_arms_to_goal(self, use_composite=False)))
        self.buttons.append(Button('Plan Both Arms to Goal (composite)', self.plan_both_arms_to_goal_action))
        self.buttons.append(Button('Exec Both Arm Trajs', lambda: world.execute_arm_trajectory_both(self)))

        # Constrained dual-arm planner controls — only when the active robot is dual-arm.
        # Stored as named attributes so update() polls them — items
        # appended to self.dump_sep_sliders are not polled.
        if self.huskies[self.selected_robot_id].dual_arm:
            # TODO these two buttons seems to have very similar functions, and also unclear whether
            # Replan Current Movement Live should stick to its stored conf target or recompute IK from target ee, probably need to be movement depednent
            # then this could just merge with the debug buttons below

            # self.buttons.append(Button(
            #     'Replan Free (live base)',
            #     self.replan_free_from_live_base,
            # ))
            # self.buttons.append(Button(
            #     'Replan Constrained (live base)',
            #     self.replan_constrained_from_live_base,
            # ))

            # TODO I think these two should be renamed a bit better, one keep the old arm conf (but new base) and plan a motion from current conf to go there
            # TODO the other recompute a new ik based on the movement start conf's FK EE targets and then plan a motion from current conf to go there
            # self.buttons.append(Button('Plan Free → Mv Start (offline target)', self.plan_free_to_movement_start_with_cfab_cc))
            self.buttons.append(Button('Replan IK & Transit → Mv Start (live, M2/M3)', self.ik_live_base_for_selected_movement))
            # * Button 2: live-base IK + composite free plan to the selected
            # M2/M3's start EE targets, in one click. Uses cfab CC.
            self.buttons.append(Button(
                'Replan IK & Transit → Mv Start (live, M2/M3)',
                self.replan_free_to_movement_start_live))

            self.buttons.append(Button(
                'Export Dual-Traj',
                self.export_constrained_dual_arm_trajectory,
            ))
            self.buttons.append(Button(
                'Load Dual-Traj',
                self.parse_constrained_dual_arm_trajectory,
            ))
            self.constrained_display_slider = Slider(
                "Display Traj (0=Free,1=Constrained)",
                self.update_constrained_display_mode,
                0, 1, 0,
            )
        else:
            # Clear stale handles from a prior dual-arm build (reset_ui removes
            # the underlying pybullet params but leaves Python attrs behind).
            if hasattr(self, 'constrained_display_slider'):
                delattr(self, 'constrained_display_slider')

        if self.CONNECT_COMPLIANT_CONTROLLER:
            # self.dump_sep_sliders.append(Slider("----------CONTROLLERS", lambda: None))
            self.dump_sep_sliders.append(Separator("CONTROLLERS"))
            def _switch_to_compliance_both():
                h = self.huskies[self.selected_robot_id]
                for i in range(2 if h.dual_arm else 1):
                    h.interface.switch_controller(
                        'scaled_joint_trajectory_controller',
                        'cartesian_compliance_controller', i)
            def _switch_to_joint_both():
                h = self.huskies[self.selected_robot_id]
                for i in range(2 if h.dual_arm else 1):
                    h.interface.switch_controller(
                        'cartesian_compliance_controller',
                        'scaled_joint_trajectory_controller', i)
            def _zero_force_sensor_both():
                h = self.huskies[self.selected_robot_id]
                for i in range(2 if h.dual_arm else 1):
                    h.interface.zero_ft_sensor(i)
            self.buttons.append(Button('Switch to Compliance (BOTH)', _switch_to_compliance_both))
            self.buttons.append(Button('Switch to Joint (BOTH)', _switch_to_joint_both))   # = "ensure joint controller"
            self.buttons.append(Button('Zero Force Sensor (BOTH)', _zero_force_sensor_both))
            self.buttons.append(Button('Draw TCP Pose', lambda: world.draw_tcp_pose(self)))

        # if self.USE_MOCAP:
        #     self.dump_sep_sliders.append(Slider("----------MoCap Experiment", lambda : None))
        #     self.buttons.append(Button('Test Webcam Capture', self.test_webcam_capture))
        #     self.buttons.append(Button('Record Raw MoCap Take', self.record_raw_mocap_take))

        # if not self.CALIBRATION:
        #     # in calibration mode, we do not have task space targets so this is disabled
        #     pass
        #     # self.buttons.append(Button('Exec S.Arm Traj with servoing', self.execute_arm_trajectory_with_servoing))

        # if not self.CALIBRATION:
        #     self.buttons.append(Button('Exec Free Motion', self.execute_free_trajectory))
        #     self.buttons.append(Button('Exec Linear Motion', self.execute_linear_trajectory))
        # self.buttons.append(Button('Plan arm wave', lambda: world.plan_arm_wave(self)))

        # Scaffolding tool control removed - outdated, will be remade later.

        if self.BAR_ACTION_LIVE_REPLAN_EXE:
            # self.dump_sep_sliders.append(Slider("----------BarAction live replan & exe", lambda: None))
            self.dump_sep_sliders.append(Separator("BarAction live replan & exe"))
            if not self.available_bar_actions and hasattr(self, '_load_available_bar_actions'):
                self.available_bar_actions = self._load_available_bar_actions()
            n_files = len(self.available_bar_actions)
            # Reset on rebuild; slider only created when >=2 entries
            # (a 1-entry slider has rangeMin == rangeMax which segfaults
            # pybullet's GUI thread — same hazard as board_validation_state_slider).
            self.bar_action_file_slider = None
            if n_files > 1:
                self.bar_action_file_slider = Slider(
                    "BarAction file (idx)",
                    lambda v: setattr(self, '_selected_action_file_idx', int(round(float(v)))),
                    0, n_files - 1,
                    int(self._selected_action_file_idx),
                    integer=True,
                )
            self.buttons.append(Button('Load BarAction', self.load_bar_action_file))
            n_movs = len(self._loaded_movements)
            # Same 1-entry segfault guard as bar_action_file_slider.
            self.bar_movement_slider = None
            if n_movs > 1:
                self.bar_movement_slider = Slider(
                    "Movement (idx; 0=M0_synth)",
                    lambda v: setattr(self, '_selected_movement_idx', int(round(float(v)))),
                    0, n_movs - 1,
                    int(self._selected_movement_idx),
                    integer=True,
                )
            self.buttons.append(Button('Load Movement', self.load_selected_movement))
            self.buttons.append(Button('Plan Movement', self.plan_selected_movement))
            self.buttons.append(Button('Load Movement Trajectory', self.load_selected_movement_trajectory))
            # * Button 1: plan the M1->M2->M3->M0->M4 chain in one click,
            # export the mutated action as `<name>.live-solved.json` sidecar.
            self.buttons.append(Button('Plan Chain (Live)', self.plan_movement_chain_live))
            # * Reset the currently loaded movement to its authored ("clean")
            # state; downstream propagated start_confs may become stale, and
            # the next chain plan will re-populate them.
            self.buttons.append(Button(
                'Reset Selected Mv to Clean',
                self.reset_selected_movement_to_clean))
            # * Reset every movement of the currently loaded BarAction back
            # to the clean file (matches --load clean in headless_bar_action_planner).
            self.buttons.append(Button(
                'Reset All Mvs to Clean',
                self.reset_all_movements_to_clean))

            # self.dump_sep_sliders.append(Slider("---------- live movement debug", lambda: None))
            self.dump_sep_sliders.append(Separator("live movement debug"))

            # self.dump_sep_sliders.append(Slider("---------- movement exe", lambda: None))
            self.dump_sep_sliders.append(Separator("movement exe"))

            self.buttons.append(Button(
                'Exec Compliant (M2/M3 only)',
                lambda: self.tasks.append(world.execute_planned_trajectory_compliant(self))))
            # * Auto-dispatch: M2/M3 -> compliant controller, else joint tracking.
            self.buttons.append(Button(
                'Exec Selected Mv Traj (auto)',
                self.exec_selected_movement_traj))

            self.buttons.append(Button(
                'Move Arms to Movement Start (offline target)',
                lambda: world.move_arms_to_movement_start(self)))

        if self.BAR_ACTION_MOCAP_ACCURACY_TEST:
            self.buttons.append(Button('Record markerset take', self.record_bar_holding_marker_take))
            self.buttons.append(Button('Record + Fit + Viz (shared)', self.record_bar_take_with_shared_viz))
            self.buttons.append(Button('Save markerset data', self.save_bar_holding_marker_data))

        if self.DUAL_ARM_EE_CONSTR_ACCURACY_MOCAP_TEST:
            # self.dump_sep_sliders.append(Slider("----------Dual Arm Acc Test", lambda : None))
            self.dump_sep_sliders.append(Separator("Dual Arm Acc Test"))
            self.buttons.append(Button('Compute Trajectory', lambda: world.next_dual_arm_bar_trajectory(self)))
            self.buttons.append(Button('Exec Arms', lambda: world.execute_arm_trajectory_both(self)))
            self.buttons.append(Button('Exec Arms and Record', lambda: self.tasks.append(world.execute_and_log_mocap(self))))
            self.buttons.append(Button('Record EE mocap pose', lambda: world.record_dual_arm_E_mocap(self)))
            self.buttons.append(Button('Save EE mocap data', lambda: world.save_dual_arm_E_mocap(self)))

        if self.DUAL_ARM_KISSING_REP_EXPERIMENT:
            # self.dump_sep_sliders.append(Slider("----------KISSING EXPERIMENT", lambda: None))
            self.dump_sep_sliders.append(Separator("KISSING EXPERIMENT"))
            self.buttons.append(Button('Conduct Kissing Experiment',
                lambda: self.tasks.append(world.kissing_experiment(self))))
            self.buttons.append(Button('Move Forward 1cm',
                lambda: world.move_left_linear_z(self, 0.01, 0.001)))
            self.buttons.append(Button('Move Back 1cm',
                lambda: world.move_left_linear_z(self, -0.01, 0.001)))
            
        if self.CALIBRATION:
            # self.dump_sep_sliders.append(Slider("----------Calibration", lambda : None))
            self.dump_sep_sliders.append(Separator("Calibration"))
            # self.calib_joint_range_slider = Slider("calib joint range", self.update_calib_joint_range, 0.0, np.pi*2, np.pi*2)
            # self.calib_target_axis_slider = Slider("calib target joint id", self.update_calib_target_axis, 0, 1, 0)
            # Mode slider: 0 = validation mode, 1 = data collection mode
            # self.data_collection_mode_slider = Slider(
            #     "Mode (0:validation, 1:data_collection)",
            #     self.update_data_collection_mode,
            #     0.0, 1.0,
            #     1.0 if self.data_collection_mode else 0.0
            # )
            # integer=True -> snaps to whole values, like the Calib idx sliders below.
            self.data_collection_mode_slider = Slider(
                "Mode (0:validation, 1:data_collection)",
                self.update_data_collection_mode,
                0, 1,
                1 if self.data_collection_mode else 0,
                integer=True,
            )
            # self.calib_batch_slider = Slider(
            #     "Batch (0:j0,1:j1,2:valid,3:punch)",
            #     self.update_calib_batch_index,
            #     0, len(CALIBRATION_BATCHES) - 1,
            #     self.selected_calib_batch_index
            # )
            self.calib_batch_slider = Slider(
                "Batch (0:j0,1:j1,2:valid,3:punch)",
                self.update_calib_batch_index,
                0, len(CALIBRATION_BATCHES) - 1,
                self.selected_calib_batch_index,
                integer=True,
            )
            # --- Calibration state/trajectory loaders (CALIBRATION_STATE_SET) ---
            # self.dump_sep_sliders.append(Slider("----------State Loading", lambda: None))
            self.dump_sep_sliders.append(Separator("State Loading"))
            self.calibration_state_slider = None
            if self.available_calibration_states and len(self.available_calibration_states) > 1:
                max_idx = len(self.available_calibration_states) - 1
                self.calibration_state_slider = Slider(
                    "Calib RobotCellState (idx)",
                    self.update_calibration_state_index,
                    0, max_idx,
                    int(np.clip(self.selected_calibration_state_index, 0, max_idx)),
                    integer=True,
                )
            if self.available_calibration_states:
                self.buttons.append(Button('Load Calib RobotCellState', self.load_calibration_state))

            self.calibration_trajectory_slider = None
            if self.available_calibration_trajectories and len(self.available_calibration_trajectories) > 1:
                max_idx = len(self.available_calibration_trajectories) - 1
                self.calibration_trajectory_slider = Slider(
                    "Calib JointTrajectory (idx)",
                    self.update_calibration_trajectory_index,
                    0, max_idx,
                    int(np.clip(self.selected_calibration_trajectory_index, 0, max_idx)),
                    integer=True,
                )
            if self.available_calibration_trajectories:
                self.buttons.append(Button('Load Calib JointTrajectory', self.load_calibration_trajectory))

            # self.buttons.append(Button('Set joint 0 to zero', self.set_goal_joint_0_to_zero))
            # self.buttons.append(Button('Calib joint 1', lambda: world.calibrate_joint(self, 1, self.active_calib_tool_name)))

            # self.buttons.append(Button('Sample calib path', self.sample_calib_traj))
            # self.buttons.append(Button('Execute transit to calib traj', self.execute_free_trajectory))
            self.buttons.append(Button('Execute calib traj', self.execute_calib_traj))
            self.buttons.append(Button('Record current calib conf',
                                       lambda: world.calibrate_button(self, self.active_calib_tool_name)))
            self.buttons.append(Button('Export calib data to json', self.record_calibration_data))
            self.buttons.append(Button('collect cameras data', self.collect_mocap_camera_data))


        if self.PUNCH_CALIB_VALIDATION:
            # self.dump_sep_sliders.append(Slider("----------Punch Calib Validation", lambda : None))
            self.dump_sep_sliders.append(Separator("Punch Calib Validation"))
            self.buttons.append(Button('Record Punch Take', self.record_punch_reference_pose))
            self.buttons.append(Button('Save Punch Validation Data', self.save_punch_validation_data))

        if not self.CALIBRATION:
            # Gripper controls — only when the active robot connected its gripper.
            self.gripper_slider = None
            if self.huskies[self.selected_robot_id].connect_gripper:
                # self.dump_sep_sliders.append(Slider("----------Gripper", lambda: None))
                self.dump_sep_sliders.append(Separator("Gripper"))
                self.gripper_slider = Slider(
                    "gripper pos (0=open, 0.85=closed)",
                    lambda v: setattr(self, 'goal_gripper', float(v)),
                    0.0, 0.85, self.goal_gripper,
                )
                self.buttons.append(Button('Open Gripper Full', lambda: world.open_gripper_full(self)))
                self.buttons.append(Button('Close Gripper for Bar', lambda: world.close_gripper_for_bar(self)))
                self.buttons.append(Button('Set Gripper (slider)', lambda: world.set_gripper(self)))

            # Scaffolding V3 controls — only when active robot has assembly_tool_v3_*.
            active_husky = self.huskies[self.selected_robot_id]
            has_scaffold_left = any('assembly_tool_v3_left' in (t or '') for t in active_husky.ee_types)
            has_scaffold_right = any('assembly_tool_v3_right' in (t or '') for t in active_husky.ee_types)
            if has_scaffold_left or has_scaffold_right:
                # self.dump_sep_sliders.append(Slider("----------Scaffolding V3", lambda: None))
                self.dump_sep_sliders.append(Separator("Scaffolding V3"))

                def send_scaffolding_cmd_both_motors(direction, arm_index):
                    interface = self.huskies[self.selected_robot_id].interface
                    interface.send_scaffolding_cmd(direction, 1, arm_index)
                    interface.send_scaffolding_cmd(direction, 2, arm_index)

                def send_scaffolding_cmd_motor(direction, motor, arm_index):
                    self.huskies[self.selected_robot_id].interface.send_scaffolding_cmd(direction, motor, arm_index)

                if has_scaffold_left:
                    self.buttons.append(Button('- L Stop All', lambda: send_scaffolding_cmd_both_motors(0, 0)))
                    self.buttons.append(Button('- L Tighten Gripper', lambda: send_scaffolding_cmd_motor(1, 1, 0)))
                    self.buttons.append(Button('- L Loosen Gripper', lambda: send_scaffolding_cmd_motor(-1, 1, 0)))
                    self.buttons.append(Button('- L Tighten Joint', lambda: send_scaffolding_cmd_motor(1, 2, 0)))
                    self.buttons.append(Button('- L Loosen Joint', lambda: send_scaffolding_cmd_motor(-1, 2, 0)))

                if has_scaffold_right and active_husky.dual_arm:
                    self.buttons.append(Button('- R Stop All', lambda: send_scaffolding_cmd_both_motors(0, 1)))
                    self.buttons.append(Button('- R Tighten Gripper', lambda: send_scaffolding_cmd_motor(1, 1, 1)))
                    self.buttons.append(Button('- R Loosen Gripper', lambda: send_scaffolding_cmd_motor(-1, 1, 1)))
                    self.buttons.append(Button('- R Tighten Joint', lambda: send_scaffolding_cmd_motor(1, 2, 1)))
                    self.buttons.append(Button('- R Loosen Joint', lambda: send_scaffolding_cmd_motor(-1, 2, 1)))



        # self.dump_sep_sliders.append(Slider("----------DEBUG utils", lambda : None))
        self.dump_sep_sliders.append(Separator("DEBUG utils"))
        self.buttons.append(Button('Sample Random Goal Conf', self.sample_random_goal_conf))
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

        if self.USE_MOCAP:
            self._init_mocap_offset_window()

    # --- --- --- --- --- MOCAP --- --- --- --- ---
    _ANSI_GREEN = '\033[92m'
    _ANSI_RED = '\033[91m'
    _ANSI_RESET = '\033[0m'

    def start_mocap(self):
        self.get_logger().info('Starting mocap!')
        self.mocap_client = NatNetClient()
        self.mocap_client.set_client_address(CLIENT_IP)
        self.mocap_client.set_server_address(MOCAP_IP)
        self.mocap_client.set_use_multicast(False)
        self.mocap_client.print_level = 1

        self.mocap_client.rigid_body_listener = self.receive_rigid_body_frame
        self.mocap_client.new_frame_listener = self.receive_mocap_frame
        if self.BAR_ACTION_MOCAP_ACCURACY_TEST:
            self.mocap_client.labeled_marker_listener = self.receive_labeled_marker

        if self.mocap_client.run():
            start_connect = time.time()
            while not self.mocap_client.connected():
                time.sleep(0.25)
                if time.time() - start_connect > 5:
                    break
            connected = self.mocap_client.connected()
            color = self._ANSI_GREEN if connected else self._ANSI_RED
            self.get_logger().info(f"{color}mocap client connected: {connected}{self._ANSI_RESET}")
            if connected:
                self.mocap_client.request_model_definitions()
        else:
            self.get_logger().info(f"{self._ANSI_RED}Failed to run mocap client!{self._ANSI_RESET}")

    def get_mocap_camera_inventory(self, refresh=False, timeout_sec=0.5):
        if not hasattr(self, 'mocap_client') or not self.mocap_client.connected():
            return None

        if refresh:
            self.mocap_client.request_model_definitions()

        deadline = time.time() + timeout_sec
        data_descs = self.mocap_client.get_latest_data_descriptions()
        while data_descs is None and time.time() < deadline:
            time.sleep(0.05)
            data_descs = self.mocap_client.get_latest_data_descriptions()

        if data_descs is None:
            return None

        camera_list = []
        for camera in getattr(data_descs, 'camera_list', []):
            camera_list.append(
                {
                    'name': camera.name.decode('utf-8') if isinstance(camera.name, bytes) else str(camera.name),
                    'position': [float(value) for value in camera.position],
                    'orientation': [float(value) for value in camera.orientation],
                }
            )

        return {
            'snapshot_time': time.time(),
            'camera_count': len(camera_list),
            'cameras': camera_list,
        }

    def send_request_to_mocap(self):
        # self.mocap_client.send_request(self.mocap_client.command_socket, self.mocap_client.NAT_REQUEST_MODELDEF,    "",  (self.mocap_client.server_ip_address, self.mocap_client.command_port) )
        # time.sleep(1)
        world.request_marketset_button(self, MOCAP_SET_RIG_RB_NAME)

    # mocap updates are happening in a separate thread
    def receive_rigid_body_frame(self, id, pos, rot):
        pos = np.array(mocap_pos_y_up_to_z_up(pos, self.MOCAP_AXIS_CONVENTION))
        rot = np.array(mocap_quat_y_up_to_z_up(rot, self.MOCAP_AXIS_CONVENTION))

        name = self.name_from_mocap_id.get(id, f'rigid_body_{id}')
        with self._mocap_cache_lock:
            self._mocap_rigidbody_cache[name] = (pos, rot)
            self._mocap_rigidbody_id_from_name[name] = int(id)
    
    def receive_mocap_frame(self, data):
        ts = data['timestamp']
        with self._mocap_cache_lock:
            raw_snapshot = {
                name: (np.array(pose[0], dtype=float), np.array(pose[1], dtype=float))
                for name, pose in self._mocap_rigidbody_cache.items()
            }
            rigid_body_ids = dict(self._mocap_rigidbody_id_from_name)

        if self.mocap_experiment_recording is not None:
            self._record_raw_mocap_snapshot(ts, raw_snapshot, rigid_body_ids)

        for h in self.huskies:
            if h.name not in raw_snapshot:
                continue
            world_from_mocap = raw_snapshot[h.name]
            # apply calibrated base transformation here
            # we keep the raw mocap data in _mocap_rigidbody_cache
            calibrated_pose = pp.multiply(world_from_mocap, h.base_mocap_from_base_footprint)
            # World-frame XYZ offset; rebind from UI thread is atomic in CPython.
            pos_with_offset = np.array(calibrated_pose[0]) + h.mocap_base_offset_xyz
            h.interface.mocap_callback(pos_with_offset, np.array(calibrated_pose[1]), ts)

        for o in self.tracked_objects:
            if o.name not in raw_snapshot:
                continue
            (pos, rot) = raw_snapshot[o.name]
            o.mocap_callback(pos, rot, ts)
        # self._mocap_rigidbody_cache.clear()

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
                pos = mocap_pos_y_up_to_z_up(marker_data['pos'], self.MOCAP_AXIS_CONVENTION)
                self._mocap_labeled_marker_cache[name][marker_id] = {
                    'pos': pos,
                    'size': marker_data['size'],
                    'error': marker_data['error'],
                }
            # print(f'Received marker set data for {name}:', self._mocap_labeled_marker_cache[name])
     
    # --- --- --- --- --- UPDATE --- --- --- --- --- 
    def update(self):
        if _common._global_backend is not None:
            if not _common._global_backend.step():
                # User closed the UI window - request a clean shutdown.
                rclpy.shutdown()
                return

        self._pump_mocap_offset_window()

        # Keyboard shortcuts removed - outdated, will be remade later.

        for b in self.buttons:
            b.update()

        # Scaffolding-tool live status overlay removed - outdated, will be remade later.

        # update tracked objects
        for i, o in enumerate(self.tracked_objects):
            o.set_pose((o.pos, o.rot))
        
        # update robot state
        for i, h in enumerate(self.huskies):
            hi = h.interface
            if self.USE_MOCAP and not self.USE_CELL_STATE_BASE_POSE:
                # mocap drives the husky base pose
                h.object.set_pose((hi.position, hi.rotation), hi.arm_joint_pose)
                # set the goal pose of base since we are teleoperating the base
                if not self.goal_base_pose_frozen:
                    self.goal_base_pose = (hi.position, hi.rotation)
            else:
                # base is whatever the cell state set (or sliders set);
                # mocap only drives EE tracking in this branch
                h.object.set_pose(self.goal_base_pose, hi.arm_joint_pose)

        # pp.draw_pose(self.goal_model.get_link_pose_from_name("ur_arm_base_link"))

        self.arm_slider.update()
        self.trajectory_time_slider.update()
        self.traj_viz_time_slider.update()  # PyBullet mode: poll fires update_traj_viz_time
        if self.gripper_slider is not None:
            self.gripper_slider.update()

        # "Step through waypoints" sliders on the cfab GUI window (no-op until a
        # trajectory has been loaded, e.g. via 'Load Dual-Traj').
        self._service_trajectory_waypoint_sliders()

        # if self.CALIBRATION:
        #     self.calib_joint_range_slider.update()
        #     self.calib_target_axis_slider.update()
        
        if self.CALIBRATION and self.data_collection_mode_slider:
            self.data_collection_mode_slider.update()
        if self.CALIBRATION and self.calib_batch_slider:
            self.calib_batch_slider.update()
        if self.CALIBRATION and self.calibration_state_slider:
            self.calibration_state_slider.update()
        if self.CALIBRATION and self.calibration_trajectory_slider:
            self.calibration_trajectory_slider.update()

        if self.board_validation_state_slider:
            self.board_validation_state_slider.update()

        if hasattr(self, 'constrained_display_slider'):
            self.constrained_display_slider.update()

        if self.BAR_ACTION_LIVE_REPLAN_EXE:
            if hasattr(self, 'bar_action_file_slider') and self.bar_action_file_slider:
                self.bar_action_file_slider.update()
            if hasattr(self, 'bar_movement_slider') and self.bar_movement_slider:
                self.bar_movement_slider.update()

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

        # update assembly goal position
        # self.assembly_goal_position_slider_group.update()
            
        # preview_time = p.readUserDebugParameter(self.time_slider)
        preview_time = self.traj_viz_time  # updated by update_traj_viz_time (both UI modes)
        goal_base_pose = self.goal_base_pose
        # Preview must not mutate self.goal_arm_pose; planners consume that
        # field as the actual target configuration.
        goal_arm_pose = [
            np.array(self.goal_arm_pose[0], dtype=float).copy(),
            np.array(self.goal_arm_pose[1], dtype=float).copy(),
        ]
        if not self.show_goal_state:
            # Trajectory preview rides on the LIVE robot's base pose (not the
            # frozen goal_base_pose / cell-state base) so the planned arm
            # motion is shown as it would actually look at the real-robot
            # location. The arm conf below is read from the planned
            # trajectory; pairing it with the live base matches what gets
            # executed.
            if self.huskies:
                _hi = self.huskies[self.selected_robot_id].interface
                goal_base_pose = (_hi.position, _hi.rotation)
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
        # goal_arm_pose is always length 2 (per __init__); slice for single-arm goal_model.
        arm_pose = goal_arm_pose if self.goal_model.dual_arm else goal_arm_pose[:1]
        self.goal_model.set_pose(goal_base_pose, arm_pose)

        # Drag attached-body ghosts along with the goal_model: pose follows
        # the parent link's FK at the current goal_arm_pose / preview-time
        # interpolation, composed with the stored attachment_frame.
        for g in self._traj_ghost_bodies:
            try:
                world_from_link = self.goal_model.get_link_pose_from_name(g['link'])
                pp.set_pose(g['body'], pp.multiply(world_from_link, g['attach']))
            except Exception:
                pass
                        
        # run tasks
        for t in self.tasks:
            try:
               next(t)
            except StopIteration:
                self.tasks.remove(t)
                
        world.update(self)

    def _trajectories_dir(self):
        d = os.path.join(DESIGN_DATA_DIRECTORY, DESIGN_PROBLEM_NAME, 'Trajectories')
        os.makedirs(d, exist_ok=True)
        return d

    def export_constrained_dual_arm_trajectory(self, filename=None):
        """Export self.constrained_trajectory (left+right) as a single 12-DOF
        compas_fab JointTrajectory JSON, written to <problem>/Trajectories/."""
        traj = self.constrained_trajectory
        if not (traj and traj[0] is not None and traj[1] is not None):
            print("No constrained dual-arm trajectory to export. Plan an M1 movement first.")
            return None
        left_path, _, left_time, _ = traj[0]
        right_path, _, right_time, _ = traj[1]
        n = len(left_path)
        if n == 0 or n != len(right_path):
            print(f"Constrained trajectory length mismatch: left={n}, right={len(right_path)}.")
            return None

        joint_names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]) + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
        joint_types = [Joint.REVOLUTE] * len(joint_names)
        total_time = float(left_time if left_time is not None else (right_time or 0.0))
        points = []
        for i in range(n):
            joint_values = [float(v) for v in left_path[i]] + [float(v) for v in right_path[i]]
            t = (total_time * i / (n - 1)) if n > 1 else 0.0
            secs = int(t)
            nsecs = int((t - secs) * 1e9)
            points.append(JointTrajectoryPoint(
                joint_values=joint_values,
                joint_types=joint_types,
                joint_names=joint_names,
                time_from_start=Duration(secs, nsecs),
            ))
        start_configuration = Configuration(
            joint_values=list(points[0].joint_values),
            joint_types=joint_types,
            joint_names=joint_names,
        ) if points else None
        jt = JointTrajectory(
            trajectory_points=points,
            joint_names=joint_names,
            start_configuration=start_configuration,
            fraction=1.0,
        )

        if filename is None:
            mv = self.current_movement
            act = self.current_action
            if mv is not None and act is not None:
                stem = f"{act.action_id}_{mv.movement_id}_constrained_dual_arm_JointTrajectory"
            else:
                stem = f"constrained_dual_arm_JointTrajectory_{int(time.time())}"
            filename = stem + '.json'
        out_path = os.path.join(self._trajectories_dir(), filename)
        jt.to_json(out_path, pretty=True)
        print(f"Exported constrained dual-arm trajectory ({n} waypoints) to {out_path}")
        # Refresh available list so the parse-side slider can pick it up.
        self.available_joint_trajectories = self._load_available_joint_trajectories()
        return out_path

    def parse_constrained_dual_arm_trajectory(self, filename=None):
        """Load a 12-DOF compas_fab JointTrajectory JSON from <problem>/Trajectories/
        and populate self.constrained_trajectory + per-arm display trajectories."""
        if filename is None:
            if not self.available_joint_trajectories:
                self.available_joint_trajectories = self._load_available_joint_trajectories()
            if not self.available_joint_trajectories:
                print("No JointTrajectory files in Trajectories/ to parse.")
                return False
            idx = self.selected_trajectory_index
            if not (0 <= idx < len(self.available_joint_trajectories)):
                print(f"Invalid trajectory index: {idx}")
                return False
            filename = self.available_joint_trajectories[idx]
        path = filename if os.path.isabs(filename) else os.path.join(self._trajectories_dir(), filename)
        if not os.path.isfile(path):
            print(f"Trajectory file not found: {path}")
            return False

        try:
            jt = JointTrajectory.from_json(path)
        except Exception as e:
            print(f"Failed to load JointTrajectory from {path}: {e}")
            return False

        left_names = HUSKY_DUAL_UR5e_JOINT_NAMES[0]
        right_names = HUSKY_DUAL_UR5e_JOINT_NAMES[1]
        # Resolve per-point joint name list (fall back to trajectory-level names).
        traj_names = list(jt.joint_names) if jt.joint_names else []
        try:
            left_idx = [traj_names.index(n) for n in left_names]
            right_idx = [traj_names.index(n) for n in right_names]
        except ValueError as e:
            print(f"Trajectory missing required dual-arm joints: {e}")
            return False

        left_path, right_path, times = [], [], []
        for pt in jt.points:
            names = pt.joint_names if pt.joint_names else traj_names
            if names == traj_names:
                li, ri = left_idx, right_idx
            else:
                try:
                    li = [list(names).index(n) for n in left_names]
                    ri = [list(names).index(n) for n in right_names]
                except ValueError as e:
                    print(f"Trajectory point missing required joints: {e}")
                    return False
            jv = pt.joint_values
            left_path.append(np.array([jv[i] for i in li], dtype=float))
            right_path.append(np.array([jv[i] for i in ri], dtype=float))
            times.append(pt.time_from_start.seconds)

        total_time = float(times[-1]) if times and times[-1] > 0 else float(self.trajectory_time)
        left_arr = np.array(left_path)
        right_arr = np.array(right_path)
        self.constrained_trajectory = [
            (left_arr, None, total_time, None),
            (right_arr, None, total_time, None),
        ]
        self.constrained_start_conf = np.concatenate([left_arr[0], right_arr[0]])
        self.constrained_goal_conf = np.concatenate([left_arr[-1], right_arr[-1]])
        self.set_arm_trajectory(self.constrained_trajectory[0], index=0)
        self.set_arm_trajectory(self.constrained_trajectory[1], index=1)
        self.constrained_display_mode = 1
        try:
            self._refresh_constrained_displayed_trajectory()
        except Exception:
            pass
        try:
            self.set_to_show_traj_state()
        except Exception:
            pass
        if self.cfab is not None and self.movement_start_state is not None:
            self._build_trajectory_waypoint_sliders()
        print(f"[Parse Constrained Traj] dual-arm trajectory: "
              f"{len(left_path)} waypoints from {path}")
        return True

    def export_planned_trajectory_to_json(self, filename='planned_trajectory.json', arm_index=None):
        """
        Export the planned arm trajectory to a JSON file as a list of joint configurations.
        Save to the DATA_DIRECTORY/robotx_box subfolder.
        """
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

    def destroy_node(self):
        if _common._global_backend is not None:
            try:
                _common._global_backend.shutdown()
            except Exception as e:
                self.get_logger().warn(f"UI backend shutdown error: {e}")
            _common._global_backend = None
        try:
            self._shutdown_mocap_offset_window()
        except Exception as e:
            self.get_logger().warn(f"mocap offset window shutdown error: {e}")
        super().destroy_node()

# --- --- --- --- --- MAIN --- --- --- --- ---
def main(args=None):
    rclpy.init(args=args)

    husky_monitor = HuskyMonitor()

    rclpy.spin(husky_monitor)

    husky_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':     
    main()
