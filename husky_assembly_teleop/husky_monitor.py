"""
- Setting up the pybullet simulation
- Setting up the mocap client
- Updating the simulation state
- Handling user input
"""
import sys, re
print(f"Running with Python: {sys.executable}")

from collections import defaultdict
from contextlib import nullcontext
import os
import time, copy
import threading
import traceback
import json
import csv
import yaml
from datetime import datetime
from types import SimpleNamespace
import numpy as np

from typing import Generator, List, Optional, Tuple
from scipy.spatial.transform import Rotation as R

import rclpy
import rclpy.executors
from rclpy.node import Node

import pybullet as p
import pybullet_planning as pp

from husky_assembly_teleop import DATA_DIRECTORY, DESIGN_DATA_DIRECTORY, EXPERIMENT_DATA_DIRECTORY, CALIBRATION_DATA_DIRECTORY, CALIBRATION_BATCHES, DESIGN_PROBLEM_NAME, CALIBRATION_DATE
import husky_assembly_teleop.husky_world as world
from husky_assembly_teleop.husky_world import _solve_bar_action_goal_ik, solve_goal_ik_generic
import husky_assembly_teleop.mocap_experiment as mocap_experiment
from husky_assembly_teleop.mocap_experiment import (
    fit_bar_from_markerset, bar_deviation_from_goal, draw_marker_take_in_pp,
)
from husky_assembly_teleop.husky_robot import GRIPPER_MOTOR, JOINT_MOTOR, UR5e_HOME_STATE
from husky_assembly_teleop.common import (
    Button, Slider, SliderGroup, Toggle, StatusText, Separator, TextInput, LiveMultiPlot, HistoryPlot, Husky, TrackedObject, HuskyObject, AssemblyObject, HUSKY_UR5e_JOINT_NAMES, lerp, load_gripper,
    Group,
)
from husky_assembly_teleop.cc_diagnosis import (
    clear_collision_diagnosis, visualize_goal_ik_collision,
    collect_collision_contacts, print_collision_contacts, draw_collision_contacts,
)
from husky_assembly_teleop.m1_derive_report import print_m1_derivation_summary
from husky_assembly_teleop.dashboard.run_writer import write_m1_run
from husky_assembly_teleop.optitrack.NatNetClient import NatNetClient
from husky_assembly_teleop.utils import (
    pose_from_frame, frame_from_pose, pose_from_transformation, transformation_from_pose,
    mocap_pos_y_up_to_z_up, mocap_quat_y_up_to_z_up,
    vec12_from_conf, conf_from_12vec, conf_from_6vec,
    joint_trajectory_from_path, path_12_from_joint_trajectory, path_from_joint_trajectory,
    HUSKY_DUAL_ARM_HOME_CONF_12, HUSKY_DUAL_UR5e_JOINT_NAMES, MOCAP_SET_RIG_RB_NAME,
)

# BarAction (gdrive design-study) loading
from husky_assembly_teleop.bar_action_io import (
    parse_bar_action, list_bar_actions, find_movement,
    load_action_cycle, slot_of_index, cycle_start_ee_sources,
    MovementKind, STATIONARY_KINDS, SINGLE_ARM_KINDS, DUAL_ARM_KINDS, COMPLIANT_KINDS,
    movement_kind, step_kind, kind_fits_robot, is_free_home, check_action_kinds,
    default_trajectory_time, is_built_assembly_body, bar_body_name, find_bar_body,
    clean_action_path,
)
from rs_data_structure.bar_action import CONTROLLER_CARTESIAN_COMPLIANT, CONTROLLER_JOINT_TRACKING, Movement
from husky_assembly_teleop.cfab_session import (
    CfabSession, build_default_robot_cell, plan_free_motion, plan_linear_motion,
    arm_joint_names_for_group, SINGLE_ARM_GROUP,
    HUSKY_DUAL_URDF_PATH, HUSKY_DUAL_SRDF_PATH,
    GROUND_RIGID_BODY_NAME,
    inject_ground_rigid_body_state, apply_obstacle_robot_beliefs,
)
from husky_assembly_teleop.robot_registry import RobotSpec, robot_by_name
from husky_assembly_teleop.progress_io import (
    BELIEF_ASSUMED, BELIEF_LIVE, STATUS_PENDING, RobotBelief, belief_after, belief_from_live,
    load_progress, new_run_id, obstacle_tool_states, obstacle_sources, obstacle_sources_line,
    recompute_belief, save_progress,
)
from husky_assembly_teleop.schedule_io import (
    SCHEDULE_FILENAME, LoadedEntry, ScheduleEntry, is_executable_by, load_entry, load_schedule,
    problem_root,
)
from husky_assembly_teleop.schedule_ui import (
    EntryFlags, entry_row_text, knobs_for_assembly_robot, missing_action_files, now_line_text,
    row_color, scan_entry_flags, step_button_label, visible_row_window,
)
from husky_assembly_teleop import common as _common
from husky_assembly_teleop.ui_backend import make_backend, DearPyGuiBackend, bind_default_font

from compas.data import json_load, json_dump
from compas.geometry import Frame, Transformation
from compas_fab.backends import CollisionCheckError
from compas_fab.robots import JointTrajectory, JointTrajectoryPoint, RobotCellState
from compas_fab.robots.time_ import Duration
from compas_robots import Configuration
from compas_robots.model import Joint

# TAMP motion-planner API. Safe to import at module top: no import-time
# side effects and no circular dependency back into this package.
from husky_assembly_tamp.motion_planner.api import (
    plan_free_dual_arm, plan_constrained_dual_arm, plan_constrained_dual_arm_linear,
    plan_dual_arm_linear_independent, _fk_link_frame, _collect_obstacle_puids,
    # M1 start-derivation stage on its own (see derive_m1_endpoints_live).
    _derive_constrained_start_for_plan, _build_cfab_collision_fn, _bar_body_id,
    _state_with_conf12, TOOL_LINK_LEFT, TOOL_LINK_RIGHT,
)
from husky_assembly_teleop.m1_manual_start import bar_pose_mb, goal_geometry, manual_m1_start

DEFAULT_GREY = [0.2, 0.2, 0.2, 0.7]
GOAL_BLUE = [0, 0.2, 0.5, 0.7]
TRAJECTORY_GREEN = [0, 0.5, 0.2, 0.7]
TRANSPARENT = [0, 0.0, 0.0, 0.0]
# Faint grey for built bars / joints the planner ignores (IGNORE_BUILT_ASSEMBLY_COLLISIONS):
# still drawn where they stand, but clearly "not counted".
BUILT_IGNORED_RGBA = (0.6, 0.6, 0.6, 0.25)
# See-through bar that follows the manual M1 start sliders in PyBullet.
MANUAL_BAR_ORANGE = [1.0, 0.55, 0.0, 0.6]
# See-through bar at the loaded action's assembled pose (M3 start), the target to drive to.
ASSEMBLED_BAR_PINK = [1.0, 0.3, 0.7, 0.5]

# Constrained dual-arm free/transfer motion (CDFM) planner resolutions.
# Used by: M1 initial plan, Button 2b (IK Replan & Transfer → Mv Start),
# Button 3 (Servo live loop for M1), Button 3b (Servo transfer loop).
# Single source for both the plan call and CDFM sparse validation.
# CDFM_POSITION_RES = 0.002   # meters
# CDFM_ROTATION_RES = 0.0125  # radians
CDFM_POSITION_RES = 0.01   # meters
CDFM_ROTATION_RES = 0.025  # radians

# --- Free movements (M0 / M4), planned live by plan_free_dual_arm ---
#
# ! plan_free_dual_arm does NO swept collision checking: it only tests the
# ! discrete configurations its extend function produces, so anything thinner
# ! than one step can be passed straight through. At a shoulder joint with a
# ! ~0.9 m lever, the upstream 0.05 rad default is ~45 mm of tool travel between
# ! checks -- wider than an assembly bar, which is how a planned M0 ends up
# ! visibly sweeping through the built structure while every waypoint is clear.
# ! Its post-plan shortcutting makes this worse: it replaces wiggly sequences
# ! with long straight segments and re-checks them at this same resolution.
#
# So: plan at a step small enough to hit a bar (slower to plan), and then verify
# the returned path at a finer step still (_validate_free_planned_path), which is
# what actually gates acceptance.
FM_JOINT_RESOLUTION = 0.1        # rad, planning / collision-check step
FM_VALIDATION_STEP_RAD = 0.01    # rad, post-plan re-check step (2x finer)
# FM_JOINT_RESOLUTION = 0.01        # rad, planning / collision-check step
# FM_VALIDATION_STEP_RAD = 0.005    # rad, post-plan re-check step (2x finer)

# Below this the live arms count as already standing at a preplanned path's
# first waypoint, so there is nothing to bridge. Well under the 0.1 rad the
# controller itself demands (HuskyRobotInterface.to_trajectory_msg).
FM_PATCH_TOLERANCE_RAD = 0.005
# Which planning stage the M1 constrained planner runs. Stage 3 is the
# grasped-bar transport stage (see STAGE3_GRASP_MASK_LINKS). This used to be
# adjustable via a "Constrained Stage" GUI slider, but in practice only stage 3
# was ever used, so it is now a fixed constant.
M1_PLANNER_STAGE = 3

# M1 derived-start "home bar" carry anchor choices, indexed by the GUI slider
# (0 = sample all anchors hierarchically). Labels must match the keys of
# core.HOME_BAR_ANCHORS in husky_assembly_tamp (kept as a local tuple so the
# monitor does not import that deep module just for names).
M1_HOME_ANCHOR_CHOICES = ('all', 'horizontal', 'vertical', 'back')
# Manual M1 start pose sliders (human-in-the-loop start, see
# confirm_m1_manual_start): shifts in metres, roll in degrees, both ways.
M1_MANUAL_SHIFT_RANGE_M = 0.5
M1_MANUAL_ROLL_RANGE_DEG = 180.0

# Pre-execution safeguard thresholds for a bar-held ("transfer") path. They
# mirror path_validation.validate_stage_trajectory's defaults so the live DPG
# safeguard flags the same problems the offline validator would:
#   - joint continuity: a per-step joint jump above this hints at a 2*pi wrap
#     or a discontinuous replan (the arm would snap).
#   - EE drift: how far the left->right relative tool0 pose is allowed to move
#     from the first waypoint before the bar grasp is no longer "rigid".
TRANSFER_JOINT_STEP_THRESHOLD_DEG = 1.0
TRANSFER_EE_TRANS_THRESHOLD_MM = 0.5
TRANSFER_EE_ROT_THRESHOLD_DEG = float(np.degrees(1e-2))  # ~0.573 deg

# M2 (mate) is executed in two chunks: a RIGID approach under the joint
# controller, then the last stretch under compliance. This is the handover
# point, measured as tool0 distance from the assembled pose.
#
# Running M2 wholly compliant does not work: the moment compliance engages the
# arms sag under gravity and pull the bar off target far enough that the screw
# cannot catch its first thread. Position control holds the bar on the planned
# line while the screw catches; compliance then lets the screw pull the bar in.
#
# 0 = fully rigid (the screw never gets compliant control -- warned about at
# execution). At or above the movement's own travel = fully compliant, which is
# the old behaviour. Clamped at runtime to whatever the loaded M2 travels.
M2_COMPLIANT_SPLIT_MM = 5.0
M2_COMPLIANT_SPLIT_MM_MAX = 30.0

# Cartesian path density for M3's replanned retreat -- how finely the straight
# line from the arms' ACTUAL tool0 poses to the movement's authored goal frames
# is sampled before each waypoint is IK'd. Same units and defaults as the
# offline linear planners (headless_bar_action_planner's DEFAULT_MAX_STEP_*).
M3_REPLAN_MAX_STEP_DISTANCE = 0.001   # m
M3_REPLAN_MAX_STEP_ANGLE = 0.05       # rad

# * Another husky's mocap base is used as its obstacle pose only when its rigid
# * body arrived this recently (seconds); an older pose falls back to the belief.
LIVE_OBSTACLE_MAX_AGE_S = 0.5
# * 'Mark entry done' stores the connected robot's mocap base as its belief only
# * when mocap saw it this recently (seconds); otherwise the action's authored base.
LIVE_BELIEF_MAX_AGE_S = 1.0

# Legend for the "planned joint values" preview plot. A BarAction trajectory
# waypoint is always a 12-vec (left arm's 6 joints, then the right arm's), in
# the shoulder -> wrist order of HUSKY_DUAL_UR5e_JOINT_NAMES.
PLANNED_JOINT_PLOT_LABELS = [
    f'{side} {name}' for side in ('L', 'R')
    for name in ('pan', 'lift', 'elbow', 'w1', 'w2', 'w3')
]
# A support robot's (single-arm) waypoint is a 6-vec: one arm, no side prefix.
PLANNED_JOINT_PLOT_LABELS_SINGLE_ARM = ['pan', 'lift', 'elbow', 'w1', 'w2', 'w3']

# * How many schedule entry rows the schedule panel shows at once (a window
# * around the selected entry, see schedule_ui.visible_row_window).
SCHEDULE_ROWS_SHOWN = 12

# * The BUILT ASSEMBLY's rigid-body names (bars 'bar_<id>' / 'env_bar_<id>' and
# * their connectors) now live in bar_action_io (BUILT_ASSEMBLY_RB_PREFIXES,
# * is_built_assembly_body), and the ground's wheel allowance in cfab_session
# * (GROUND_TOUCH_LINKS, applied by inject_ground_rigid_body_state).

EXISTING_ELEMENT_COLOR = pp.RED
CURRENT_ELEMENT_COLOR = pp.BLUE
DEFAULT_BAR_POS = pp.Point(0.8, 0, 1.3)

CLIENT_IP = '192.168.0.25' # Set to your own IP
# ! Both are DHCP leases and do move. If mocap logs "connected: False", see
# ! doc/calibration_manual.md section 1.4 before touching anything else.
MOCAP_IP = '192.168.0.28' # set to the mocap PC's IP, get this from Motive Settings>Streaming pane->Local interface
# Where the 'collect cameras data' button drops its JSON+CSV (gdrive folder also
# holding import_mocap_cameras_rhino.py).
MOCAP_CAMERA_EXPORT_DIR = os.path.join(EXPERIMENT_DATA_DIRECTORY, 'visualise_mocap_camera')

# Folder under DESIGN_DATA_DIRECTORY (gdrive)/<...>/RobotCellStates/
# from which CALIBRATION-mode state + trajectory loaders pull files.
# Keyed by selected_arm_index (0=left, 1=right); see _calibration_state_dir().
# Full design-study archive lives on GitHub: yijiangh/husky_assembly_design_study.
CALIBRATION_STATE_SETS = {
    0: '260630_calib_trajs_Alice',              # left arm & single arm
    1: '260225_extrinsic_calib_trajs_Cindy_Right',  # right arm for Cindy
}

class HuskyMonitor(Node):
    # * Set 0 for the robot-centric experiment: replay a pre-planned BarAction
    # without any external tracking. The husky base is then assumed to be
    # exactly where the plan says it is (each movement's
    # start_state.robot_base_frame), see _live_base_pose().
    USE_MOCAP = 1
    FAKE_HARDWARE = 0

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
    # * Set 0 to skip creating the compliant-controller ROS interfaces
    # (target_wrench publishers + start_force_mode / zero_ftsensor /
    # switch_controller service clients). Saves 2.5 s per client (5 waits on a
    # dual arm) + the "... Service Client False" lines when the UR driver's
    # force_mode / io_and_status / controller_manager services aren't running.
    # With it off: switch_controller() and zero_ft_sensor() log and return
    # False, so a compliant M2/M3 execution aborts with a clear error instead
    # of moving, the end-of-M0 FT zero just warns, and the CONTROLLERS buttons
    # are not built. The FT *subscription* is created unconditionally, so the
    # live force plot works either way.
    # ! Set 1 for any session that executes M2/M3 (cartesian compliance).
    CONNECT_COMPLIANT_CONTROLLER = 0

    # When USE_MOCAP=1, by default the husky base in PyBullet tracks mocap.
    # Set USE_CELL_STATE_BASE_POSE=1 to override that and pin the base to
    # whatever was loaded from the goal RobotCellState's robot_base_frame
    # (or set via sliders). Useful for testing planning with mocap on for
    # end-effector tracking but the husky physically far from the assembly
    # scaffolding (e.g., at the lab desk during dual-arm accuracy tests).
    USE_CELL_STATE_BASE_POSE = 0
    USE_DPG_UI = 1   # 0 = legacy PyBullet debug GUI; 1 = Dear PyGui control panel
    UI_FONT_SIZE = 20  # base size for all DPG widgets (separators override to 20 in the backend)
    # ! Size of the PyBullet window's mesh buffer, in MB. When it is full, bodies
    # ! loaded later are silently NOT drawn. PyBullet's default is too small for
    # ! three huskies + the goal ghost + the compas_fab planning robot: the blue/
    # ! green ghost and the red planning robot vanished. 512 verified; a 1024 MB
    # ! request did not take effect on this machine.
    PYBULLET_GUI_MESH_BUFFER_MB = 512

    CALIBRATION = 0

    BAR_ACTION_LIVE_REPLAN_EXE = 1    # show Load BarAction / Load Movement / replan buttons
    # * Set 0 to always use the legacy BarAction file list. With 1, a design
    # * problem whose ActionSchedule.json is COMPLETE (every entry's action file,
    # * clean export or .live-solved sidecar, is on disk) is driven entry by entry
    # * from the schedule panel instead (see _load_schedule_state). Incomplete
    # * schedules (e.g. an accuracy-test export) keep the legacy list.
    USE_ACTION_SCHEDULE = 1
    # Set 1 for the mocap bar-holding accuracy experiment: adds the markerset
    # record/save buttons + the servoing tracker, hides the already-built
    # assembly (so its bars are ignored by collision checks; legacy BarAction
    # list only, schedule mode uses IGNORE_BUILT_ASSEMBLY_COLLISIONS below), and
    # force-attaches the active bar for the transfer replan. Keep 0 for the robot-centric
    # replay demo, where the built assembly should stay visible and collision-checked.
    # ! These three flags move together, so flip all of them when switching mode:
    # !   mocap accuracy test  : USE_MOCAP=1, BAR_ACTION_LIVE_REPLAN_EXE = 1, USE_CELL_STATE_BASE_POSE=0, BAR_ACTION_MOCAP_ACCURACY_TEST=1
    # !   robot-centric demo   : USE_MOCAP=0, BAR_ACTION_LIVE_REPLAN_EXE = 1, USE_CELL_STATE_BASE_POSE=1, BAR_ACTION_MOCAP_ACCURACY_TEST=0
    # (see doc/bar_holding_acc_manual.md, "Pre-flight checklist")
    BAR_ACTION_MOCAP_ACCURACY_TEST = 1  # show Record + Fit + Viz / Save markerset data
    # * Schedule mode: 1 = the planner and IK ignore collisions with the already-built
    # * bars and joints, which are then drawn faint (BUILT_IGNORED_RGBA) so it shows
    # * that they do not count. 0 = the built structure is a real obstacle, drawn
    # * as usual. The panel toggle "Ignore built-bar collisions" changes it while
    # * the monitor runs. See _ignore_built_assembly.
    IGNORE_BUILT_ASSEMBLY_COLLISIONS = 0
    DUAL_ARM_EE_CONSTR_ACCURACY_MOCAP_TEST = 0

    # Set to 1 to dump cfab's collision-check setup (its ACM / allowed-collision
    # matrix at the current state) once per cfab session. Handy when diagnosing
    # why a movement plans into collision, but noisy otherwise, so keep it off by
    # default. See _inject_live_conf_into_state.
    DEBUG_CFAB_CC_SETUP = 0

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
    #              BarActions" case. Stress test: the goal IK frequently
    #              fails outright from this seed, and even when it solves,
    #              the 12-DOF free plan from home extended arms to a
    #              bar-holding grip is a genuinely hard corridor problem
    #              the sampler often can't solve. The IK failure prints and
    #              draws whatever rejected it -- see cc_diagnosis.py.)
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

    def __init__(self):
        super().__init__('husky_monitor')
        self.tick_timer = self.create_timer(0.05, self.update)

        # simple async tasks to be executed every tick
        self.tasks = []
        self._running_task = None  # the task update() is stepping right now (see its task loop)

        # Marks this instance as the live ROS-driven monitor (vs. a headless
        # test harness that bypasses __init__). Headless flows skip
        # _hide_cfab_robot since there's no overlapping pp-side husky.
        self._is_live_monitor = True

        self.huskies = []
        self.tracked_objects = []
        self.name_from_mocap_id = {}
        self._mocap_cache_lock = threading.Lock()
        self._mocap_rigidbody_cache = {}
        # time.monotonic() of the last real (non-zero) pose per rigid body; the
        # cache above keeps a body's last pose forever, so this says how fresh it is.
        self._mocap_rigidbody_stamp = {}
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
        # Result of the last 'M1: Derive Start/Goal only' click (see
        # derive_m1_endpoints_live); consumed by adopt_m1_derived_start.
        self._m1_derived = None
        # Ticked by the 'Adopt also saves ...' checkbox: whether adopting the
        # derived start also writes the confs to the BarAction file on disk.
        self._m1_adopt_writes_file = False
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
        self._loaded_action = None              # first half's action | None
        self._loaded_action_slots = []          # list[(action, path)]; both halves of the cycle
        self._loaded_movements = []             # list[Movement]; every movement across those halves
        self._loaded_start_ee_sources = []      # per movement: {side: movement authoring its START pose}
        self._live_start_indices = set()        # movements whose start is the live robot pose
        # ActionSchedule progress (progress_io.Progress) of the design problem, or
        # None (legacy problem / not loaded yet). Set by the schedule UI; read via
        # getattr everywhere because the headless harnesses skip __init__.
        self._progress = None
        # * ActionSchedule mode (_load_schedule_state). All of these stay None /
        # * empty on a legacy problem; like _progress, read them via getattr.
        self._schedule = None                   # schedule_io.ActionSchedule
        self._schedule_flags = {}               # entry index -> schedule_ui.EntryFlags
        self._schedule_run_id = None            # progress_io.new_run_id of this monitor run
        self._selected_entry_idx = 0            # what the 'Schedule entry' slider points at
        self._loaded_entry = None               # schedule_io.ScheduleEntry last loaded
        self._loaded_entry_bundle = None        # its schedule_io.LoadedEntry
        self._schedule_row_window = (0, 0)      # (lo, hi) entries shown as rows
        self._clean_entries = set()             # entries reopened this run: load their clean export
        self.schedule_header_text = None
        self.schedule_entry_slider = None
        self.schedule_entry_text = None
        self.schedule_now_text = None
        self.schedule_rows = []
        self.ignore_built_toggle = None         # "Ignore built-bar collisions" Toggle
        self._selected_action_file_idx = 0
        self._selected_movement_idx = 0
        # M1 home carry anchor index into M1_HOME_ANCHOR_CHOICES (0 = all).
        self._m1_home_anchor_idx = 0
        # Manual M1 start adjustments (the sliders are rebuilt from these).
        self._m1_manual_slide_m = 0.0
        self._m1_manual_roll_deg = 0.0
        self._m1_manual_perp1_m = 0.0
        self._m1_manual_perp2_m = 0.0
        self._ee_target_pose_uids = []          # pp.add_line uids for drawn EE targets
        # Collision-diagnosis drawing (cc_diagnosis.py), run automatically every
        # time a live goal IK fails. Handles are pp debug-item uids; the colour
        # cache is keyed by (body puid, link index) so restoring a highlighted
        # link is lossless.
        self._cc_diag_handles = []              # list[int]
        self._cc_diag_orig_colors = {}          # (body, link) -> RGBA before highlight
        # Per-movement attached-body ghosts. The bodies are the ones cfab
        # already spawned via set_robot_cell_state; we just re-color them
        # TRAJECTORY_GREEN and re-pose them via goal_model FK each tick so
        # they ride along the trajectory preview.
        self._traj_ghost_bodies = []            # list[{'body','link','attach'}]
        # Original RGBA of the built-assembly bodies HIDDEN by
        # _sync_pp_visibility_to_hidden (restored at the top of the next load).
        self._traj_ghost_orig_colors = {}       # body puid -> RGBA
        # When the built assembly is ignored (_ignore_built_assembly): True once it
        # has been flagged hidden + drawn faint for the CURRENTLY loaded BarAction. The set
        # of hidden bodies depends only on the action's active bar, so switching
        # MOVEMENTS can skip the whole show/re-hide cycle. Reset by
        # load_bar_action_file when a new action (new active bar) is parsed.
        self._mocap_hide_applied = False
        # Names of the built bodies _hide_built_assembly_for_mocap flagged hidden
        # for the loaded action: the planner ignores them, the view draws them
        # faint. Bodies the export itself hides (not built yet) are not in it.
        # Emptied when a new action is parsed (_finish_action_load).
        self._collision_ignored_bodies = set()
        # Original RGBA of the PREVIEW bodies (bar/joints/tools) recoloured by
        # _refresh_preview_attached_bodies. Kept separate from the hidden-body
        # cache so re-syncing the preview at trajectory time never un-hides the
        # built assembly.
        self._preview_body_orig_colors = {}     # body puid -> RGBA
        # Motion type of the trajectory currently staged for preview, set at
        # each planner call site: 'free' (bar not mounted) or 'bar_held' (bar +
        # its installed joints ride with the robot). Drives whether the preview
        # mounts the bar/joints, independent of the authored start_state (the
        # replan buttons override the authored type -- see
        # _refresh_preview_attached_bodies).
        self.planned_trajectory_motion_type = 'free'

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
        # Where M2 hands over from rigid tracking to compliance, as tool0
        # distance from the assembled pose. See M2_COMPLIANT_SPLIT_MM
        # for the other execution knobs, and
        # world.execute_planned_trajectory_compliant for what it does.
        self.m2_compliant_split_mm = M2_COMPLIANT_SPLIT_MM
        # Operator toggles, both surfaced as 0/1 sliders in the movement-exe
        # section so their state is visible rather than only logged.
        #   m2_exec_rigid_only: run ALL of M2 under the joint controller and
        #     never engage compliance -- the baseline the split is compared to.
        #   fm_swept_validation_enabled: gate M0/M4 plans on the dense
        #     between-waypoint collision re-check. On by default; turning it off
        #     accepts plans unverified (each skip is warned about).
        self.m2_exec_rigid_only = False
        self.fm_swept_validation_enabled = True
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
            # Schedule mode or the legacy file list -- decided once, here.
            self._load_schedule_state()
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
        
    def clear_all_debug_drawing(self) -> None:
        """Wipe every debug drawing, including the collision diagnosis.

        ``pp.remove_all_debug()`` only deletes lines and text. The collision
        diagnosis also recolours the links it highlights, so those have to be
        restored separately or they stay orange/cyan forever.
        """
        pp.remove_all_debug()
        clear_collision_diagnosis(self)

    def toggle_show_goal_state(self):
        """Flip the ghost robot between the goal conf and the planned trajectory.

        Two things the ghost can show, and this picks which:

        - GOAL view (blue): the ghost sits at ``self.goal_arm_pose``, the
          configuration the planners are actually aiming at.
        - TRAJECTORY view (green): the ghost is driven by
          ``self.planned_arm_trajectory``, scrubbed by the 'Traj viz time'
          slider, and rides the live base pose so the preview matches what
          execution will do (see update()).

        ? The colour IS the mode indicator -- blue is the goal, green is the
        ? trajectory -- which is why the two views never share one.
        """
        self.show_goal_state = not self.show_goal_state
        # The button can be clicked before any goal model has been loaded; the
        # flag still flips so the view is right once one appears.
        if self.goal_model is not None:
            self.goal_model.set_color(GOAL_BLUE if self.show_goal_state else TRAJECTORY_GREEN)
        if self.show_goal_state:
            print("[view] GOAL conf (blue ghost).")
        else:
            # Trajectory view with nothing planned leaves the ghost parked on
            # the goal conf but coloured green, which reads as a stuck preview.
            has_traj = any(t[0] is not None for t in self.planned_arm_trajectory)
            print("[view] planned TRAJECTORY (green ghost); scrub it with "
                  "'Traj viz time'." if has_traj else
                  "[view] planned TRAJECTORY (green ghost) -- but NOTHING is "
                  "planned yet, so the ghost stays on the goal conf. Plan a "
                  "movement first.")

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

    def _connected_robot(self) -> RobotSpec:
        """The registry spec of the robot this monitor run drives.

        ``husky_world.init`` sets ``connected_robot`` from ROS_DOMAIN_ID. The
        headless harnesses build the monitor without running it, so Cindy (the
        only robot they drive) is the fallback.

        Returns:
            RobotSpec: The connected robot.
        """
        return getattr(self, 'connected_robot', None) or robot_by_name('Cindy')

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

    # --- Visual-servoing live tracker (per-iteration tool0 error + base drift) ---
    def _servoing_plots(self):
        """The four servoing tracker plots, or None if the DPG UI is off."""
        pos = getattr(self, 'servoing_pos_plot', None)
        if pos is None:
            return None
        return (pos, self.servoing_rot_plot,
                self.servoing_base_pos_plot, self.servoing_base_rot_plot)

    def reset_servoing_tracker(self):
        """Start a fresh run: clear the persisted history and blank the plots.

        The tracker window is kept HIDDEN here (and hidden on every rebuild while
        `_servoing_tracker_visible` is False) so it does not pop up during the
        first move's trajectory preview. `show_servoing_tracker()` expands it once
        the operator confirms and execution begins.
        """
        # History persists on the monitor (not just in the plots) because the
        # servoing loop calls ik_live_base... which calls reset_ui() -> build_ui(),
        # rebuilding the plots as empty every iteration. build_ui repopulates the
        # rebuilt plots from this list, so points accumulate across iterations.
        self._servoing_history = []
        self._servoing_tracker_visible = False
        plots = self._servoing_plots()
        if plots is None:
            return
        for plot in plots:
            plot.reset()
            plot.set_visible(False)

    def show_servoing_tracker(self):
        """Expand/show the live tracker window (called when execution begins, so
        it stays collapsed during the trajectory preview/confirm pause)."""
        plots = self._servoing_plots()
        if plots is None:
            return
        self._servoing_tracker_visible = True
        for plot in plots:
            plot.set_visible(True)

    def push_servoing_tracker(self, iter_i, tool0_err, base_diff):
        """Record one per-iteration sample: persist it AND draw it on the plots.

        Args:
            iter_i (int): Servoing iteration index (used as the plot x value).
            tool0_err (dict): Output of ``world.measure_servo_tool0_error`` -- per
                side 'pos_err_mm' (3 values) and 'rot_err_deg' (3 values).
            base_diff (dict): Output of ``world.measure_base_pose_diff`` --
                'pos_diff_mm' (3 values) and 'rot_diff_deg' (3 values).
        """
        hist = getattr(self, '_servoing_history', None)
        if hist is None:
            hist = self._servoing_history = []
        hist.append((iter_i, tool0_err, base_diff))
        self._draw_servoing_sample(iter_i, tool0_err, base_diff)

    def _draw_servoing_sample(self, iter_i, tool0_err, base_diff):
        """Push one sample onto the four plots at x=iter_i (no history append).

        Also used by build_ui to repopulate freshly-rebuilt plots from history.
        """
        if self._servoing_plots() is None:
            return
        # Position pushes end each arm's / the base's block with the |d| norm,
        # matching the group_size=4 (x, y, z, |d|) layout of these plots.
        self.servoing_pos_plot.push(
            list(tool0_err['left']['pos_err_mm']) + [tool0_err['left']['pos_norm_mm']]
            + list(tool0_err['right']['pos_err_mm']) + [tool0_err['right']['pos_norm_mm']],
            x=iter_i)
        self.servoing_rot_plot.push(
            list(tool0_err['left']['rot_err_deg']) + list(tool0_err['right']['rot_err_deg']),
            x=iter_i)
        self.servoing_base_pos_plot.push(
            list(base_diff['pos_diff_mm']) + [base_diff['pos_norm_mm']], x=iter_i)
        self.servoing_base_rot_plot.push(list(base_diff['rot_diff_deg']), x=iter_i)

    def _repopulate_servoing_tracker(self):
        """Redraw all persisted samples onto the (freshly-built) plots."""
        for iter_i, tool0_err, base_diff in getattr(self, '_servoing_history', []):
            self._draw_servoing_sample(iter_i, tool0_err, base_diff)

    # --- --- Movement preview: planned joint values --- ---

    def show_planned_joint_values(self, path12, *, label=''):
        """Plot the selected movement's planned joint values, per waypoint.

        One series per joint (left arm then right arm), x = waypoint index.
        This is what the arms are ABOUT to do, so the operator can spot a
        wrapped joint or a near-limit excursion before pressing execute --
        as opposed to the "Joint Live Stream" window, which reports what the
        real robot is doing right now.

        The curve is persisted on ``self._preview_joint_data`` so it survives
        the ``reset_ui`` -> ``build_ui`` rebuild that every Load Movement
        triggers (same reason as the transfer-validation curves).

        Args:
            path12 (Sequence): Planned waypoints, each a 12-vec of joint
                values in radians (left 6 then right 6).
            label (str): Short tag for the log line (e.g. movement id).
        """
        rows_deg = [list(np.degrees(np.asarray(q, dtype=float)))
                    for q in (path12 or [])]
        if not rows_deg:
            self.get_logger().warn(
                f"[movement preview] {label!r}: empty planned path, nothing to plot.")
            return
        self._preview_joint_data = {'label': label, 'rows_deg': rows_deg}
        self._draw_preview_joint_values()
        flat = np.asarray(rows_deg, dtype=float)
        print(f"[movement preview] {label!r}: {len(rows_deg)} waypoints, "
              f"joint range [{flat.min():.1f}, {flat.max():.1f}] deg.")

    def _draw_preview_joint_values(self):
        """Push the persisted planned-joint curve onto the (rebuilt) plot."""
        plot = getattr(self, 'preview_joint_plot', None)
        data = getattr(self, '_preview_joint_data', None)
        if plot is None or not data:
            return
        plot.reset()
        for i, row in enumerate(data['rows_deg']):
            plot.push(row, x=i)
        plot.set_visible(True)

    # --- --- Transfer (bar-held) pre-execution safeguard --- ---

    def _transfer_validation_plots(self):
        """The two transfer-validation plots, or None if the DPG UI is off."""
        step = getattr(self, 'transfer_joint_step_plot', None)
        if step is None:
            return None
        return (step, self.transfer_ee_drift_plot)

    def show_transfer_validation(self, path12, template_state, *, label=''):
        """Draw the bar-held path safeguard curves and log a PASS/FAIL verdict.

        Two things a bar-held ("transfer") path must satisfy before we let it
        run on hardware, plotted per waypoint in the "Movement Preview"
        DPG window so the operator can eyeball them before confirming:

          1. Joint continuity -- the largest single-joint jump between two
             consecutive waypoints. A spike above
             ``TRANSFER_JOINT_STEP_THRESHOLD_DEG`` means a 2*pi wrap or a
             discontinuous replan (the arm would snap on execution).
          2. Bar-hold rigidity -- how far the left->right relative tool0 pose
             has drifted from the first waypoint (translation + rotation).
             Growth here means the grasp is not being held rigid.

        The curves are persisted on ``self._transfer_validation_data`` so they
        survive the ``reset_ui`` -> ``build_ui`` rebuild that the servoing
        loop's IK step triggers (same reason as the servoing tracker history).

        Args:
            path12 (Sequence): Planned waypoints, each a 12-vec.
            template_state (RobotCellState): Base + non-arm joints for FK.
            label (str): Short tag for the log line (e.g. movement id).

        Returns:
            dict: ``{'joint_ok': bool, 'ee_ok': bool,
            'max_joint_step_deg': float, 'max_ee_trans_mm': float,
            'max_ee_rot_deg': float}``. ``joint_ok`` / ``ee_ok`` are None when
            the path is too short to evaluate.
        """
        path12 = [np.asarray(q, dtype=float) for q in (path12 or [])]
        verdict = {'joint_ok': None, 'ee_ok': None, 'max_joint_step_deg': 0.0,
                   'max_ee_trans_mm': 0.0, 'max_ee_rot_deg': 0.0}
        if len(path12) < 2:
            self.get_logger().warn(
                f"[transfer validation] {label!r}: path too short to validate.")
            return verdict

        # Curve 1: max per-joint change between consecutive waypoints (deg).
        # One value per step -> N-1 values; index 0 is the 0th->1st step.
        step_deltas_deg = [
            float(np.degrees(np.max(np.abs(path12[i + 1] - path12[i]))))
            for i in range(len(path12) - 1)
        ]
        # Curve 2: bar-hold EE drift vs the first waypoint (mm + deg).
        pos_devs_m, ang_devs_rad = self._bar_hold_ee_drift(path12, template_state)
        ee_trans_mm = [v * 1000.0 for v in pos_devs_m]
        ee_rot_deg = [float(np.degrees(v)) for v in ang_devs_rad]

        verdict['max_joint_step_deg'] = max(step_deltas_deg)
        verdict['max_ee_trans_mm'] = max(ee_trans_mm)
        verdict['max_ee_rot_deg'] = max(ee_rot_deg)
        verdict['joint_ok'] = (
            verdict['max_joint_step_deg'] <= TRANSFER_JOINT_STEP_THRESHOLD_DEG)
        verdict['ee_ok'] = (
            verdict['max_ee_trans_mm'] <= TRANSFER_EE_TRANS_THRESHOLD_MM
            and verdict['max_ee_rot_deg'] <= TRANSFER_EE_ROT_THRESHOLD_DEG)

        # Persist for the reset_ui rebuild, then draw.
        self._transfer_validation_data = {
            'label': label,
            'step_deltas_deg': step_deltas_deg,
            'ee_trans_mm': ee_trans_mm,
            'ee_rot_deg': ee_rot_deg,
        }
        self._draw_transfer_validation()

        joint_tag = 'OK' if verdict['joint_ok'] else 'FAIL'
        ee_tag = 'OK' if verdict['ee_ok'] else 'FAIL'
        msg = (f"[transfer validation] {label!r}: joint continuity {joint_tag} "
               f"(max step {verdict['max_joint_step_deg']:.2f} deg / "
               f"thresh {TRANSFER_JOINT_STEP_THRESHOLD_DEG:.1f}); bar-hold {ee_tag} "
               f"(max drift {verdict['max_ee_trans_mm']:.2f} mm, "
               f"{verdict['max_ee_rot_deg']:.3f} deg)")
        if verdict['joint_ok'] and verdict['ee_ok']:
            self.get_logger().info(msg)
        else:
            self.get_logger().warn(msg)
        return verdict

    def _draw_transfer_validation(self):
        """Push the persisted transfer-validation curves onto the plots.

        Also used by build_ui to repopulate the freshly-rebuilt plots after a
        reset_ui. The thresholds are printed under each plot (see the ``footer``
        text set in build_ui) instead of being drawn as flat reference lines,
        which used to stretch the y axis and squash the measured curves.
        """
        plots = self._transfer_validation_plots()
        data = getattr(self, '_transfer_validation_data', None)
        if plots is None or not data:
            return
        step_plot, ee_plot = plots
        step_plot.reset()
        ee_plot.reset()
        # Joint-step plot: max step delta between consecutive waypoints.
        for i, d in enumerate(data['step_deltas_deg']):
            step_plot.push([d], x=i)
        # EE-drift plot: translation (mm) + rotation (deg).
        for i in range(len(data['ee_trans_mm'])):
            ee_plot.push([data['ee_trans_mm'][i], data['ee_rot_deg'][i]], x=i)
        for plot in plots:
            plot.set_visible(True)

    # --- --- Compliant execution: live tool0 wrench --- ---

    def _compliant_wrench_plots(self):
        """The force + torque plots, or None if the DPG UI is off."""
        force = getattr(self, 'compliant_force_plot', None)
        if force is None:
            return None
        return (force, self.compliant_torque_plot)

    def reset_compliant_wrench(self, label=''):
        """Start recording a fresh wrench profile and show the plots.

        Called at the top of a compliant M2/M3 execution. Each run replaces the
        previous profile rather than appending to it, so the window always shows
        the movement that is running (or the last one that ran).

        Args:
            label (str): Short tag for the run (e.g. movement id), kept with the
                samples so a redraw after a UI rebuild still knows what it shows.
        """
        self._compliant_wrench_data = {'label': label, 'samples': []}
        plots = self._compliant_wrench_plots()
        if plots is None:
            return
        for plot in plots:
            plot.reset()
            plot.set_visible(True)

    def push_compliant_wrench(self, elapsed_s, left_wrench, right_wrench):
        """Record one force/torque sample: persist it AND draw it live.

        Fed once per monitor tick (~20 Hz) by the compliant executor's on_tick.

        ! These are RAW sensor readings -- the compliant path deliberately never
        ! zeroes the FT sensors (zeroing while the bar is held would subtract the
        ! bar's weight and mask exactly the contact we want to watch), so the
        ! curves carry the tool + bar load as a standing offset. Read them for
        ! CHANGE -- the step when the bar seats, the ramp as the screw bites --
        ! not as absolute contact force.

        Args:
            elapsed_s (float): Seconds since this execution started (plot x).
            left_wrench (Sequence[float]): Left arm [fx, fy, fz, tx, ty, tz].
            right_wrench (Sequence[float]): Right arm, same layout.
        """
        data = getattr(self, '_compliant_wrench_data', None)
        if data is None:
            data = self._compliant_wrench_data = {'label': '', 'samples': []}
        sample = (float(elapsed_s),
                  [float(v) for v in left_wrench],
                  [float(v) for v in right_wrench])
        data['samples'].append(sample)
        self._draw_compliant_wrench_sample(*sample)

    def _draw_compliant_wrench_sample(self, elapsed_s, left_wrench, right_wrench):
        """Push one sample onto the two plots (no history append).

        Force and torque get separate plots because they carry different units
        (N vs Nm) and would otherwise share a y axis that suits neither.
        """
        if self._compliant_wrench_plots() is None:
            return
        # Each wrench is [fx, fy, fz, tx, ty, tz]: first three to the force
        # plot, last three to the torque plot, left arm then right arm.
        self.compliant_force_plot.push(
            list(left_wrench[:3]) + list(right_wrench[:3]), x=elapsed_s)
        self.compliant_torque_plot.push(
            list(left_wrench[3:6]) + list(right_wrench[3:6]), x=elapsed_s)

    def _repopulate_compliant_wrench(self):
        """Redraw the persisted wrench profile onto the (freshly-built) plots."""
        plots = self._compliant_wrench_plots()
        data = getattr(self, '_compliant_wrench_data', None)
        if plots is None or not data or not data['samples']:
            return
        for plot in plots:
            plot.reset()
        for sample in data['samples']:
            self._draw_compliant_wrench_sample(*sample)
        for plot in plots:
            plot.set_visible(True)

    def toggle_servoing_tracker(self):
        """Show or hide the visual-servoing live tracker window.

        Live plots need the Dear PyGui backend, so in PyBullet mode (USE_DPG_UI=0)
        this warns and does nothing.
        """
        plots = self._servoing_plots()
        if plots is None:
            self.get_logger().warn(
                "Servoing tracker needs the Dear PyGui UI (set USE_DPG_UI=1).")
            return
        self._servoing_tracker_visible = not getattr(
            self, '_servoing_tracker_visible', False)
        for plot in plots:
            plot.set_visible(self._servoing_tracker_visible)

    # --- Punch tool calibration validation ---
    def _load_punch_tool_config(self):
        """Load punch tool offset from config.yaml."""
        try:
            punch_config_path = os.path.join(
                CALIBRATION_DATA_DIRECTORY, CALIBRATION_DATE, 'config.yaml'
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

                    hi.is_arm_executing[self.selected_arm_index] = True
                    pp.wait_for_duration(step_dt)

                hi.is_arm_executing[self.selected_arm_index] = False

    # NOTE: the old single-arm `execute_arm_trajectory_with_servoing` shim was
    # removed. The bar-holding visual servoing is now the dual-arm generator
    # `world.servo_to_movement_start_live`, wired to the 'Servo to Mv Start
    # (live loop)' button in the bar-holding accuracy section.

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
        spec = self._connected_robot()
        try:
            side = spec.side_of_link(attached_link)
        except KeyError:
            # Not a flange / arm base link (e.g. a tool link): the only side of a
            # single-arm robot, else guess the side from the link's name.
            side = (spec.side_keys[0] if spec.n_arms == 1
                    else ('left' if 'left' in attached_link else 'right'))
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

        - Bar resting in the world (installed / pre-pickup): the start_state
          stores the world ``frame`` directly, so we return that.
        - Bar held by a gripper link: the start_state only stores the grasp
          (``attachment_frame``), so the holding tool0's world pose comes from
          the authored ``target_ee_frames`` of the movement that ran BEFORE this
          one (a movement starts where the previous one ended) and the grasp is
          composed onto it. Which movement that is comes from the cycle's
          start-EE map (``_start_ee_source``): the split export puts screw events
          between the moving ones, and those have no authored targets, so the
          list neighbour is not always the one that ends where we start.

        ! The held case is never derived by FK from
        ! ``start_state.robot_configuration``. The visual-servoing loop
        ! overwrites that configuration with the LIVE arm pose after every
        ! executed iteration (``husky_world.servo_to_movement_start_live``) and
        ! then restores the AUTHORED base frame, so FK there would return the
        ! live arm pose rendered at the authored base -- the reference would
        ! carry the operator's base-parking error and the measured deviation
        ! would be wrong by exactly that amount. Same rule as the live-base IK
        ! in ``ik_live_base_for_selected_movement``: authored EE targets are the
        ! single source of truth.

        This matters for the never-unmount protocol: ``Replan Transfer`` force-
        attaches the bar into movements whose authored start_state had it
        installed (see ``_ensure_bar_attached_for_mocap``), which clears the
        static ``frame`` and so routes those movements through the held case.

        Returns:
            tuple | None: ``(pos, quat_xyzw)`` of plain floats, or ``None``
            when the bar / movement / cfab session isn't available.

        Raises:
            RuntimeError: When the bar is held but no earlier movement authored
                ``target_ee_frames`` for the holding arm. Deliberately fatal
                rather than falling back to FK: a silently wrong reference pose
                would be stamped into the saved take and corrupt the offline
                accuracy numbers.
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

        # Held bar: only the grasp frame is stored, so the holding tool0's world
        # pose is taken from the authored targets of the movement BEFORE this
        # one -- a movement starts where the previous one ended, so M3's start
        # EE poses are M2's authored targets. See the docstring for why FK at
        # start_state.robot_configuration is not used here.
        attach = getattr(bar_rb, 'attachment_frame', None)
        link = getattr(bar_rb, 'attached_to_link', None)
        if attach is None or not link:
            return None
        side = self._connected_robot().side_of_link(link)
        idx = self.current_movement_index
        prev = self._start_ee_source(idx, side)
        if prev is None:
            raise RuntimeError(
                f"Cannot stamp the reference pose of held bar {bar_name!r}: no "
                f"movement before {self.current_movement.movement_id!r} authors "
                f"target_ee_frames[{side!r}] without an arm moving in between. "
                f"EE targets are never derived from FK, so there is no safe "
                f"fallback -- load the whole cycle (both halves of a split "
                f"export) so the bar-held insert is in the list.")
        world_from_tool = pose_from_frame(prev.target_ee_frames[side])
        tool_from_bar = pose_from_frame(attach)
        pos, quat = pp.multiply(world_from_tool, tool_from_bar)
        return ([float(v) for v in pos], [float(v) for v in quat])

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
            Integer index OR movement_id substring (e.g. ``"LM_insert"``).
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

        # 3) Ensure a cfab session for this problem (the connected robot's cell).
        if self._cfab_needs_design_session():

            if self.cfab is not None:
                self.cfab.close()
            try:
                existing_client_id = pp.CLIENT if pp.is_connected() else None
                self.cfab = CfabSession(DESIGN_PROBLEM_NAME,
                                        cell_filename=self._connected_robot().cell_file,
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
        self.active_bar_name = self._active_bar_body_name(mv.start_state, action.active_bar_id)

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
        # up the ACM internally. The freshly parsed state must first be given
        # the ground body the session added to the cell (compas_fab requires
        # cell and state to hold the same rigid-body ids), and the other robots
        # posed where we believe they are.
        self._inject_ground_rigid_body_state(mv.start_state)
        self._apply_obstacle_beliefs(mv.start_state)
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

        # 1) Ghost EE proxy bodies (tiny invisible spheres), one per arm flange
        #    of the connected robot (left + right for Cindy, one for a support
        #    robot) — recreate per cfab session. pp routes EE attachments
        #    through get_collision_fn, so the child must be a distinct body
        #    (robot-vs-robot collapses).
        if getattr(self, "_bar_action_cfab_id", None) != cid:
            flange_links = self._connected_robot().flange_links
            ghosts = []
            for _flange in flange_links:
                col = p.createCollisionShape(p.GEOM_SPHERE, radius=0.001, physicsClientId=cid)
                ghosts.append(p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                                                basePosition=[0.0, 0.0, -100.0], physicsClientId=cid))
            self._bar_action_ghost_bodies = set(ghosts)
            self._bar_action_cfab_id = cid
            # Need pp.CLIENT == cid for link_from_name / Attachment below.
            _saved = pp.CLIENT
            pp.CLIENT = cid
            pp.CLIENTS.setdefault(cid, True)
            try:
                identity_grasp = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
                ee_list = []
                for flange, ghost in zip(flange_links, ghosts):
                    tool_link = pp.link_from_name(robot_puid, flange)
                    ee_list.append(
                        (ghost, pp.Attachment(robot_puid, tool_link, identity_grasp, ghost)))
                self._bar_action_husky = SimpleNamespace(object=SimpleNamespace(
                    robot=robot_puid,
                    ee_list=ee_list,
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

    # --- --- --- the connected robot's design cell + the other robots --- --- ---

    def _cfab_needs_design_session(self) -> bool:
        """Whether ``self.cfab`` must be (re)created for the current design problem.

        The session must hold the CONNECTED robot's own cell of this problem
        (``RobotCell.json`` for Cindy, ``RobotCell_<Name>.json`` for a support
        robot), so it is replaced when either the problem or the cell differs --
        e.g. the startup default rig, whose problem_name is None.

        Returns:
            bool: True when there is no session or it holds another cell.
        """
        return (self.cfab is None
                or self.cfab.problem_name != DESIGN_PROBLEM_NAME
                or getattr(self.cfab, 'cell_filename', None) != self._connected_robot().cell_file)

    def _active_bar_body_name(self, state: Optional[RobotCellState],
                              bar_id: Optional[str]) -> Optional[str]:
        """Rigid-body name of the active bar in the loaded cell.

        Cindy's cell calls it ``bar_<id>``, the support robots' cells
        ``env_bar_<id>``; whichever the state carries wins, else the connected
        robot's naming.

        Args:
            state (RobotCellState | None): A movement's start state.
            bar_id (str | None): The action's active bar id, e.g. ``'B3'``.

        Returns:
            str | None: e.g. ``'bar_B3'`` or ``'env_bar_B3'``; None without a bar id.
        """
        if not bar_id:
            return None
        rb_states = getattr(state, 'rigid_body_states', None) or {}
        return (find_bar_body(rb_states, bar_id)
                or bar_body_name(bar_id, self._connected_robot().rb_prefix))

    def _movement_starts_live(self, idx: int, mv) -> bool:
        """Whether a movement's start is wherever the robot is right now.

        True for Cindy's travel to the loading pose (its authored
        robot_configuration is null), for
        the first movement of any action whose start configuration is null
        (a support robot's ``H_M0``), and for movement 0 of a schedule entry of
        kind 'J' or 'H': those always start wherever the robot is, even when a
        ``.live-solved.json`` sidecar stored the start of an earlier run. Such a
        start_state gets the LIVE arm joints (and tracked base) injected at load.

        ! Only meaningful on freshly parsed movements: after the first injection
        ! the configuration is no longer null. ``_finish_action_load`` therefore
        ! keeps the answer in ``self._live_start_indices``.

        Args:
            idx (int): The movement's index in ``_loaded_movements``.
            mv: The Movement.

        Returns:
            bool: True when the live pose must be injected.
        """
        if mv.start_state is None:
            return False
        # load_schedule_entry sets _loaded_entry before the load that asks this.
        entry = getattr(self, '_loaded_entry', None)
        if (idx == 0 and getattr(self, '_schedule', None) is not None
                and entry is not None and entry.kind in ('J', 'H')):
            return True
        return (self._is_free_to_load(mv)
                or (idx == 0 and mv.start_state.robot_configuration is None))

    def _mocap_sees_husky(self, husky, max_age_s: float = LIVE_OBSTACLE_MAX_AGE_S) -> bool:
        """Whether mocap reported a husky's rigid body within the last ``max_age_s`` seconds.

        ! The rigid-body cache is never cleared and keeps being re-fed, so
        ! being in it says nothing about being seen; the stamps do.

        Args:
            husky (Husky): The husky (its ``name`` is the rigid-body name).
            max_age_s (float): How old the last sighting may be, in seconds.

        Returns:
            bool: True when the husky's rigid body arrived recently enough.
        """
        with getattr(self, '_mocap_cache_lock', None) or nullcontext():
            stamp = (getattr(self, '_mocap_rigidbody_stamp', None) or {}).get(
                getattr(husky, 'name', None))
        return stamp is not None and time.monotonic() - stamp < max_age_s

    def _obstacle_beliefs_with_sources(self) -> tuple:
        """Where each OTHER robot's obstacle tool goes, and where that came from.

        Two layers:
          1. With a schedule loaded (``_progress`` set): the progress_io rule --
             released its hold -> parked; else its progress.json belief; else the
             loaded action's exported tool state; else parked.
          2. On top: any other husky that mocap tracks RIGHT NOW gets its LIVE
             base, keeping the belief's joints -- or the joints it is drawn with
             when there is no belief. "Right now" means all of: mocap drives the
             bases (USE_MOCAP on, USE_CELL_STATE_BASE_POSE off), its rigid body
             arrived less than ``LIVE_OBSTACLE_MAX_AGE_S`` ago, and it has a base
             calibration (without one the drawn base is the marker body, not
             the robot's footprint).
        Without a schedule and with nothing tracked, both are empty and the
        exported tool states are left as they are.

        Returns:
            tuple: ``({obstacle_tool_name: (Frame, Configuration)},
            {obstacle_tool_name: source})``, source one of progress_io's
            ``BELIEF_*`` values.
        """
        connected = self._connected_robot()
        progress = getattr(self, '_progress', None)
        beliefs, sources = {}, {}
        if progress is not None:
            exported = getattr(self, '_loaded_action', None)
            beliefs = obstacle_tool_states(progress, connected.name, exported_action=exported)
            sources = obstacle_sources(progress, connected.name, exported_action=exported)

        if not self._base_pose_is_tracked():
            return beliefs, sources
        for name, husky in (getattr(self, 'husky_by_name', None) or {}).items():
            if name == connected.name or not self._mocap_sees_husky(husky):
                continue
            if not getattr(husky, 'has_base_calibration', False):
                continue
            spec = robot_by_name(name)
            hi = husky.interface
            # The live base, with the joints the husky is drawn with.
            live = belief_from_live(spec, (hi.position, hi.rotation), hi.arm_joint_pose)
            belief = progress.belief(name) if progress is not None else None
            configuration = belief.configuration if belief is not None else live.configuration
            beliefs[spec.obstacle_tool_name] = (live.base_frame, configuration)
            sources[spec.obstacle_tool_name] = BELIEF_LIVE
        return beliefs, sources

    def _obstacle_beliefs(self) -> dict:
        """Where to pose each other robot's obstacle tool (see _obstacle_beliefs_with_sources).

        Returns:
            dict: ``{obstacle_tool_name: (Frame, Configuration)}``; empty when
            there is no schedule progress and no other husky is tracked.
        """
        return self._obstacle_beliefs_with_sources()[0]

    def _apply_obstacle_beliefs(self, state, beliefs: Optional[dict] = None) -> None:
        """Re-pose the other robots' obstacle tools in a state (in place).

        Also lets a support robot that holds a bar right now touch that bar
        (``cfab_session.apply_obstacle_robot_beliefs``).

        Args:
            state (RobotCellState | None): The state to edit.
            beliefs (dict | None): Precomputed ``_obstacle_beliefs()``, to share
                one lookup across many states; computed here when None.
        """
        if state is None or self.cfab is None:
            return
        if beliefs is None:
            beliefs = self._obstacle_beliefs()
        progress = getattr(self, '_progress', None)
        holding = progress.holding_bars() if progress is not None else None
        apply_obstacle_robot_beliefs(state, self.cfab.robot_cell,
                                     self._connected_robot().name, beliefs,
                                     holding_bars=holding)

    def _apply_live_base_to_movement(self, mv):
        """Point ``mv.start_state`` at the live husky base and push it to cfab.

        Mutates ``mv.start_state`` in place so every downstream reader sees
        the live base — both ``monitor.movement_start_state`` (same object,
        per ``load_selected_movement``) and the per-kind planners in
        ``_plan_by_kind`` which read ``mv.start_state`` directly.

        ! Only overwrites the base frame when something actually TRACKS the base
        ! (see _base_pose_is_tracked). Without mocap, ``hi.position`` is the
        ! wheel-odometry pose -- and with no odometry publishing at all it is
        ! still its ``np.zeros(3)`` default. Writing that in unconditionally put
        ! the planning robot at the WORLD ORIGIN: compas_fab's
        ! set_robot_cell_state applies robot_base_frame unconditionally
        ! (``client._set_base_frame(...)`` -> ``resetBasePositionAndOrientation``),
        ! so the whole cfab robot teleported off the cell. Every collision check
        ! then ran from the origin, which is how a "validated" M0 plan could
        ! still drive through the structure.
        !
        ! This fires on EVERY 'Plan Movement' (plan_selected_movement calls it
        ! before dispatching), which is why the symptom appeared on planning but
        ! never on plain 'Load Movement'.

        Returns True on success; False (with a warn) on any precondition
        miss. When the base IS tracked the authored base from the BarAction file
        is overwritten in memory; to restore it, re-load the BarAction.
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
        if self._base_pose_is_tracked():
            before = mv.start_state.robot_base_frame
            mv.start_state.robot_base_frame = frame_from_pose((hi.position, hi.rotation))
            if before is not None:
                moved = float(np.linalg.norm(
                    np.asarray(list(mv.start_state.robot_base_frame.point), dtype=float)
                    - np.asarray(list(before.point), dtype=float)))
                if moved > 1e-3:
                    print(f"[apply live base] {mv.movement_id!r}: base moved "
                          f"{moved:.3f} m from the authored frame to the tracked "
                          f"live pose.")
        # The other robots too: live mocap base > progress belief > export.
        self._apply_obstacle_beliefs(mv.start_state)
        try:
            self.cfab.planner.set_robot_cell_state(mv.start_state)
        except Exception as e:
            self.get_logger().warn(f"apply live base: set_robot_cell_state failed: {e}")
            return False
        return True

    def replan_free_from_live_base(self):
        """Replan the loaded movement from the live base+conf; hide goal bar.

        "Live replan" is just the normal per-kind dispatch with the live
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

    # * The planner, the exec, the UI and the chain rules all steer by a movement's
    # * KIND (its exported class) plus one extra question for Cindy's DUAL_FREE
    # * moves: is it the travel out to the loading pose, or the free move home?
    # * The helpers below answer those for the loaded movements.
    # ! _loaded_movements / _loaded_action_slots are read through getattr: the
    # ! headless harnesses build the monitor without running __init__.

    def _kind_of(self, mv) -> Optional[MovementKind]:
        """The kind of step a movement is, or None when it can not be told.

        Args:
            mv (Movement | None): A movement (usually one of ``_loaded_movements``).

        Returns:
            MovementKind | None: Its kind; None for ``mv is None`` or a movement
            class bar_action_io does not know.
        """
        if mv is None:
            return None
        try:
            return movement_kind(mv)
        except TypeError:
            return None

    def _is_free_home(self, mv) -> bool:
        """Whether a loaded movement is Cindy's free move HOME (the end of an action).

        Asks ``bar_action_io.is_free_home`` with the loaded action file that holds
        this very movement (matched by identity, not by id), so it works for a
        schedule entry (one file), the legacy J + R list (two files) and a legacy
        single file alike.

        Args:
            mv (Movement | None): A movement.

        Returns:
            bool: True for the free move home; False otherwise, also when ``mv``
            is not one of the loaded movements.
        """
        for action, _path in getattr(self, '_loaded_action_slots', None) or []:
            if any(m is mv for m in action.movements):
                return is_free_home(action, mv)
        return False

    def _is_free_to_load(self, mv) -> bool:
        """Whether a movement is Cindy's free travel OUT to the bar-loading pose.

        Args:
            mv (Movement | None): A movement.

        Returns:
            bool: True for a DUAL_FREE movement that is not the free move home.
        """
        return self._kind_of(mv) is MovementKind.DUAL_FREE and not self._is_free_home(mv)

    def _loaded_index_of(self, kind: MovementKind,
                         free_home: Optional[bool] = None) -> Optional[int]:
        """Index of the first loaded movement of a kind.

        Args:
            kind (MovementKind): The kind to look for.
            free_home (bool | None): When given, the movement's ``_is_free_home``
                must also equal it (to pick the travel to load or the free move
                home among the DUAL_FREE movements).

        Returns:
            int | None: Its index in ``_loaded_movements``, or None when there is
            no such movement.
        """
        for i, m in enumerate(getattr(self, '_loaded_movements', None) or []):
            if self._kind_of(m) is not kind:
                continue
            if free_home is not None and self._is_free_home(m) != free_home:
                continue
            return i
        return None

    def _loaded_movement_of(self, kind: MovementKind,
                            free_home: Optional[bool] = None) -> Optional[Movement]:
        """The first loaded movement of a kind (see ``_loaded_index_of``).

        Args:
            kind (MovementKind): The kind to look for.
            free_home (bool | None): See ``_loaded_index_of``.

        Returns:
            Movement | None: The movement, or None when there is none.
        """
        i = self._loaded_index_of(kind, free_home)
        return None if i is None else self._loaded_movements[i]

    def _warn_controller_mismatch(self, mv, runs: str) -> None:
        """Warn once per movement when the export asks for another arm controller.

        The monitor picks the controller from the movement's kind (the flow that
        is proven on hardware); a re-export that changes ``Movement.controller``
        should still be noticed, but not on every click.

        Args:
            mv (Movement): The movement about to run.
            runs (str): The controller the monitor runs it under.
        """
        if mv.controller == runs:
            return
        warned = getattr(self, '_controller_mismatch_warned', None)
        if warned is None:
            warned = self._controller_mismatch_warned = set()
        if mv.movement_id in warned:
            return
        kind = self._kind_of(mv)
        kind_text = kind.value if kind is not None else type(mv).__name__
        self.get_logger().warn(
            f"{mv.movement_id!r}: the export asks for the {mv.controller} controller; "
            f"the monitor runs this {kind_text} movement under {runs}.")
        warned.add(mv.movement_id)

    def _chain_sequence(self) -> list:
        """The loaded movement indices 'Plan Chain' plans, in order.

        Every movement of each kind in ``_CHAIN_KIND_ORDER``; among the DUAL_FREE
        ones the travel to load comes before the free move home.

        Returns:
            list[int]: Indices into ``_loaded_movements``.
        """
        movements = getattr(self, '_loaded_movements', None) or []
        sequence = []
        for kind in self._CHAIN_KIND_ORDER:
            indices = [i for i, m in enumerate(movements) if self._kind_of(m) is kind]
            if kind is MovementKind.DUAL_FREE:
                # False (travel to load) sorts before True (home); the sort is stable.
                indices.sort(key=lambda i: self._is_free_home(movements[i]))
            sequence.extend(indices)
        return sequence

    def _slot_of_movement(self, idx):
        """Which loaded FILE the movement at ``idx`` of the cycle came from.

        Args:
            idx (int): Index into ``self._loaded_movements``.

        Returns:
            tuple | None: ``(action, path, index_within_that_action)``, or None
            when the index is out of range.
        """
        return slot_of_index(getattr(self, '_loaded_action_slots', []), idx)

    def _start_ee_source(self, idx, side):
        """The movement whose authored target gives ``side``'s flange pose at
        the START of movement ``idx``.

        Resolved for the whole cycle at Load BarAction
        (``bar_action_io.cycle_start_ee_sources``), so every caller gets the same
        answer instead of walking the movement list itself.

        Args:
            idx (int): Index into ``self._loaded_movements``.
            side (str): ``'left'`` or ``'right'``.

        Returns:
            Movement | None: The authoring movement, or None when nothing
            authored that flange's start pose.
        """
        sources = getattr(self, '_loaded_start_ee_sources', None) or []
        if idx is None or not 0 <= idx < len(sources):
            return None
        return sources[idx].get(side)

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

    def _arm_joint_name_sets(self) -> list:
        """Return the per-arm UR joint-name lists of the loaded cfab cell.

        Dual rig: [left 6 names, right 6 names]. Single rig: [6 names].
        Derived from the cell's SRDF groups so the same code serves Alice /
        Belle (single-arm) and Cindy (dual-arm).

        Returns:
            list: One list of 6 joint names per arm.
        """
        cell = self.cfab.robot_cell
        groups = cell.robot_semantics.groups
        if 'base_left_arm_manipulator' in groups:
            return [arm_joint_names_for_group(cell, 'base_left_arm_manipulator'),
                    arm_joint_names_for_group(cell, 'base_right_arm_manipulator')]
        # The support robot's planning group ('manipulator': the design cells
        # attach the SupportGripper to it; same 6 joints as base_arm_manipulator).
        return [arm_joint_names_for_group(cell, self._connected_robot().planning_groups[0])]

    def _fill_missing_start_conf(self, state):
        """Fill a None robot_configuration with the robot's home pose.

        Authored BarAction states before chain planning can carry no
        robot_configuration; planners still need a seed dict for IK.
        Nothing changes when the state already has one. Home is the dual-arm
        home for Cindy, UR5e home for a single-arm robot.

        Args:
            state: RobotCellState modified in place (None is ignored).
        """
        if state is None or state.robot_configuration is not None:
            return
        name_sets = self._arm_joint_name_sets()
        dual = len(name_sets) == 2
        home = HUSKY_DUAL_ARM_HOME_CONF_12 if dual else UR5e_HOME_STATE
        state.robot_configuration = self.cfab.robot_cell.zero_full_configuration()
        for i, names in enumerate(name_sets):
            for n, v in zip(names, home[i * 6:(i + 1) * 6]):
                state.robot_configuration[n] = float(v)
        print("[fill] start_state.robot_configuration was None; seeded with "
              f"{'dual-arm' if dual else 'UR5e'} home.")

    def _base_pose_is_tracked(self):
        """True when an external tracker measures where the husky base is.

        The only such source is mocap, and only when it is actually driving
        the base (USE_CELL_STATE_BASE_POSE pins the base to the loaded cell
        state instead, so mocap then only serves end-effector tracking).

        Returns:
            bool: True if the base pose is measured, False if it is assumed.
        """
        return bool(self.USE_MOCAP) and not self.USE_CELL_STATE_BASE_POSE

    def _live_base_pose(self):
        """Base pose to pair with live or planned arm configurations.

        With mocap tracking the base, this is where the real husky is. Without
        it -- the robot-centric replay of a pre-planned BarAction -- nothing
        measures the base, so we take the pose the plan was authored at:
        ``self.goal_base_pose``, which ``load_selected_movement`` sets from the
        movement's ``start_state.robot_base_frame``.

        ! Do NOT read ``hi.position`` / ``hi.rotation`` directly for this. On
        ! the real robot those are filled from the wheel-odometry TF, whose
        ! origin is wherever the husky booted -- unrelated to the plan's world
        ! frame. Using them would draw the trajectory preview in the wrong
        ! place, and would make the compliant M2/M3 targets miss by the same
        ! offset (silently rejected by send_arm_cmd_cartesian's 5 cm guard).

        Returns:
            tuple: ``(position, quaternion)`` base pose in the world frame.
        """
        if self._base_pose_is_tracked():
            hi = self.huskies[self.selected_robot_id].interface
            return (hi.position, hi.rotation)
        return self.goal_base_pose

    def _inject_live_conf_into_state(self, state):
        """Overwrite a state's base frame + arm joints with the LIVE robot pose.

        Used wherever a movement's start must reflect where the robot
        actually is right now: the native M0 (whose authored
        robot_configuration is null), the free-to-movement-start planner,
        and the live-replan buttons.

        The arm joints always come from the live robot (they are measured by
        the joint encoders either way). The base frame is only overwritten
        when something actually tracks it -- see _live_base_pose; otherwise
        the state keeps the base frame the plan was authored at.

        Args:
            state: RobotCellState modified in place. If its
                robot_configuration is None, a zero full configuration is
                created first so the live values have a place to land.
        """
        # Diagnostic: one-shot dump of cfab's ACM at this state. Gated behind the
        # DEBUG_CFAB_CC_SETUP flag, and even then fires only once per cfab session
        # so per-movement reloads don't spam.
        if self.DEBUG_CFAB_CC_SETUP and not getattr(
                self, '_cfab_acm_printed_for_cid', None) == getattr(
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
        if self._base_pose_is_tracked():
            state.robot_base_frame = frame_from_pose((hi.position, hi.rotation))
        if state.robot_configuration is None:
            state.robot_configuration = self.cfab.robot_cell.zero_full_configuration()
        for i, names in enumerate(self._arm_joint_name_sets()):
            values = hi.arm_joint_pose[i] if len(hi.arm_joint_pose) > i else hi.arm_joint_pose[0]
            for n, v in zip(names, values):
                state.robot_configuration[n] = float(v)

    def load_bar_action_file(self):
        """Parse the selected BarAction JSON; log the movement roster.

        Loads the bar's WHOLE cycle, which the split export keeps in two files
        (``B6__J.json`` = M0/M1/M2, ``B6__R.json`` = M3/M4). Selecting either
        half opens both, so ``_loaded_movements`` again holds every movement of
        the cycle in order -- what the accuracy test needs, since measuring at the
        retreat (M3) reads the insert's (M2) authored targets and grasp. A legacy
        single-file action loads exactly as before.

        M0's authored robot_configuration is null (its start is wherever the
        robot lives right now), so its start_state gets the live pose injected
        here and again on every 'Load Movement'.

        ! A file whose movement kinds do not add up (``check_action_kinds``: two
        ! transfers, no retreat, ...) is refused with one error line.

        ! Refused while an ActionSchedule drives the run: there 'Load entry'
        ! loads one entry's file with its predecessor and the other robots' beliefs.
        """
        if getattr(self, '_schedule', None) is not None:
            self.get_logger().warn(
                "An ActionSchedule drives this run; use 'Load entry' to load an action.")
            return
        files = self.available_bar_actions
        if not files:
            if hasattr(self, '_load_available_bar_actions'):
                self.available_bar_actions = self._load_available_bar_actions()
                files = self.available_bar_actions
        if not files:
            self.get_logger().warn("No BarAction files available.")
            return
        # Resolve through the shared helper so this opens exactly the file
        # the '-> file' readout names (see _slider_index for why the widget's
        # live position beats the cached index).
        idx = self._slider_index(getattr(self, 'bar_action_file_slider', None),
                                 self._selected_action_file_idx, len(files))
        self._selected_action_file_idx = idx
        fname = files[idx]
        action_path = fname if os.path.isabs(fname) else os.path.join(
            DESIGN_DATA_DIRECTORY, DESIGN_PROBLEM_NAME, 'BarActions', fname,
        )
        # Both halves of the cycle, each paired with the file it came from, so a
        # sidecar save later goes back to its own file.
        slots = load_action_cycle(action_path)
        # ! The transfer / insert / retreat are found by kind, so a file with two
        # ! of them (or none) would silently pick the wrong one: refuse it.
        try:
            for slot_action, slot_path in slots:
                check_action_kinds(slot_action, os.path.basename(slot_path))
        except (ValueError, TypeError) as e:
            self.get_logger().error(f"Not loading {fname}: {e}")
            return
        self.get_logger().info(f"Loading BarAction from file {action_path}")
        self._finish_action_load(slots[0][0], slots[0][1], slots=slots)

    def _finish_action_load(self, action, path: str, loaded=None, *,
                            slots: Optional[list] = None) -> None:
        """Make a parsed action the loaded one: session, states, UI, movement 0.

        Shared tail of every action loader: the legacy file slider
        (``load_bar_action_file``, which passes the bar's whole J + R cycle as
        ``slots``) and the schedule loader (one file per entry, with its
        ``schedule_io.LoadedEntry``).

        Steps: the connected robot's design cell session (created or replaced
        when the problem or the cell changed); ``_loaded_action`` /
        ``_current_action_path`` / ``_loaded_action_slots`` / ``_loaded_movements``;
        start-EE sources (from ``loaded`` when given, else the cycle
        helper); live pose injection into the movements that start live; the
        ground body; the other robots' obstacle poses; for a schedule entry the
        traj time of its first arm movement; UI rebuild; movement 0.

        M0's authored robot_configuration is null (its start is wherever the
        robot lives right now), so its start_state gets the live pose injected
        here and again on every 'Load Movement'.

        Args:
            action: The parsed action (first half of the cycle for the legacy loader).
            path (str): The file it came from.
            loaded (schedule_io.LoadedEntry | None): The schedule entry bundle;
                its start-EE sources (which walk the predecessor entry too) are
                used instead of recomputing them from this file alone.
            slots (list | None): ``(action, path)`` pairs when more than one file
                makes up the loaded movements (the legacy J + R cycle); defaults
                to ``[(action, path)]``.
        """
        self._loaded_action_slots = slots if slots is not None else [(action, path)]
        self._loaded_action, self._current_action_path = action, path

        self._loaded_movements = [
            mv for slot_action, _path in self._loaded_action_slots
            for mv in slot_action.movements
        ]
        if not self._loaded_movements:
            self.get_logger().warn("BarAction has no movements.")
        if loaded is not None:
            self._loaded_start_ee_sources = list(loaded.start_ee_sources)
        else:
            # Where each movement's flanges START, resolved once for the whole
            # cycle so no call site has to walk the list (and get it wrong).
            self._loaded_start_ee_sources = cycle_start_ee_sources(
                self._loaded_movements, self._connected_robot().side_keys)
        # New action => new active bar => the built-assembly hide must be redone
        # (the previously active bar has to go back to hidden, and this action's
        # active bar has to become visible). The first Load Movement below
        # re-applies it for the whole action.
        self._mocap_hide_applied = False
        self._collision_ignored_bodies = set()

        # Init the per-problem cfab session + robot cell now so 'Load
        # Movement' is just a state push afterwards. The startup default
        # session (problem_name None, no design bars) is replaced here, and so
        # is a session on another robot's cell.
        if self._cfab_needs_design_session():
            if self.cfab is not None:
                self.cfab.close()
                self.cfab = None
            try:
                existing_client_id = pp.CLIENT if pp.is_connected() else None
                with pp.LockRenderer():
                    self.cfab = CfabSession(DESIGN_PROBLEM_NAME,
                                            cell_filename=self._connected_robot().cell_file,
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
        # and planning see real values. Every freshly parsed state also needs
        # the ground body, which the Rhino export does not carry (the cell has
        # it, and compas_fab requires cell and state to agree), and the other
        # robots posed where we believe they are (the export's guess may be
        # stale, e.g. a held bar's release parks its own holder).
        # ! Decide which movements start live BEFORE injecting anything: the
        # ! injection fills the null robot_configuration that the rule looks at,
        # ! so asking again later ('Load Movement') would always say no.
        self._live_start_indices = {
            i for i, mv in enumerate(self._loaded_movements)
            if self._movement_starts_live(i, mv)}
        beliefs, sources = self._obstacle_beliefs_with_sources()
        for i, mv in enumerate(self._loaded_movements):
            if mv.start_state is None:
                continue
            if i in self._live_start_indices:
                self._inject_live_conf_into_state(mv.start_state)
            self._inject_ground_rigid_body_state(mv.start_state)
            self._apply_obstacle_beliefs(mv.start_state, beliefs)
        if sources:
            print("[obstacle robots] posed from: " + ", ".join(
                f"{name} <- {source}" for name, source in sorted(sources.items())))

        loaded_names = ' + '.join(
            os.path.basename(p) for _action, p in self._loaded_action_slots)
        print(f"[BarAction] loaded {loaded_names} "
              f"with {len(self._loaded_movements)} movements:")
        for i, mv in enumerate(self._loaded_movements):
            kind = self._kind_of(mv)
            print(f"  [{i}] {mv.movement_id!r} "
                  f"kind={kind.value if kind is not None else type(mv).__name__}")
        # * Load entry: the traj time starts at the entry's first default, not at
        # * the slider's start-up value (its 90 s maximum). The first movement that
        # * has one: a release opens with tool steps where no arm moves.
        if loaded is not None:
            for mv in self._loaded_movements:
                if self._set_default_trajectory_time(mv):
                    break
        # Refresh UI so the Movement slider's range now matches the loaded
        # movement count (was 0..8 before; now 0..len(movements)-1). In the live
        # GUI monitor a freshly selected action starts at its first movement
        # (M0), so reset the index before the rebuild so the slider comes back
        # at 0. Headless scripts (_is_live_monitor=False) load their own target
        # movement explicitly, so leave their selection alone.
        live = getattr(self, '_is_live_monitor', False)
        if live:
            self._selected_movement_idx = 0
        self.reset_ui(self.goal_arm_pose)

        # Trajectories now live on mv objects in memory (loaded natively via
        # compas json_load when a `.live-solved.json` sidecar is opened via
        # this same Load BarAction button). Print the initial roster.
        self._print_movement_roster(tag='LoadBarAction')

        # Push the first movement's state into the cfab / PyBullet scene so the
        # visualizer reflects the NEWLY selected action right away. The cfab
        # session (and its spawned RobotCell) is created only on the FIRST Load
        # BarAction; every later action reuses that session, so nothing else
        # repositions the bodies -- without this the 3D view would keep showing
        # the previously loaded action until 'Load Movement' is clicked. Only
        # in the live monitor; the scripts drive their own movement loads.
        if live:
            self.load_selected_movement()
            # * Mount-once mocap test: mark where this bar ends up, so the base
            # * can be parked by eye before the transfer loop.
            if self.BAR_ACTION_MOCAP_ACCURACY_TEST:
                self._show_assembled_bar_ghost()

    def _inject_ground_rigid_body_state(self, state):
        """Give a cell state the ground body, with the wheels-only allowance.

        See ``cfab_session.inject_ground_rigid_body_state`` (this monitor's
        session cell supplies the ground body).

        Args:
            state: RobotCellState to edit in place. Left unchanged when there is
                no session, the cell has no ground body, or the state already
                carries it.
        """
        if state is None or self.cfab is None:
            return
        inject_ground_rigid_body_state(getattr(self.cfab, 'robot_cell', None), state)

    def _ignore_built_assembly(self) -> bool:
        """Whether the planner and IK ignore the already-built bars and joints right now.

        Schedule mode: the ``IGNORE_BUILT_ASSEMBLY_COLLISIONS`` switch (the
        panel toggle sets it on this monitor). Legacy BarAction list: the mocap
        accuracy test's ``BAR_ACTION_MOCAP_ACCURACY_TEST``, as before.

        Returns:
            bool: True when ``_hide_built_assembly_for_mocap`` should run.
        """
        # Read through getattr: the headless harnesses skip __init__.
        if getattr(self, '_schedule', None) is not None:
            return bool(self.IGNORE_BUILT_ASSEMBLY_COLLISIONS)
        return bool(self.BAR_ACTION_MOCAP_ACCURACY_TEST)

    def _hide_built_assembly_for_mocap(self, state, sync_visibility: bool = True) -> None:
        """Flag the already-built (static) assembly bars/joints ``is_hidden`` so
        the cfab planner and IK ignore collisions with them.

        Only the built assembly's own bars/joints (``BUILT_ASSEMBLY_RB_PREFIXES``)
        are hidden. Everything else stays collision-checked and visible:
          - environment collision obstacles (``obstacle_env*``) -- the walls /
            fixed geometry the robot must still avoid;
          - the grasped bar (``active_bar_name``, attached to a tool) and the
            joints installed on it (attached to the tool0 links);
          - robot self-collision (CC.1) and robot<->tool (CC.2).
        ``is_hidden`` only skips the collision + repositioning steps.

        Runs when ``_ignore_built_assembly()`` says so: the schedule-mode switch
        ``IGNORE_BUILT_ASSEMBLY_COLLISIONS``, or the bar-holding accuracy
        experiment, where the built structure's real-world placement is
        approximate and must not block the live replans. Mutates
        ``state.rigid_body_states`` in place; does nothing to a state without
        rigid bodies. The names of the bodies it flags are added to
        ``_collision_ignored_bodies`` (the view draws those faint); bodies the
        export already hides (not built yet) are left alone, so they stay blanked.

        Applied ONCE PER BARACTION (see ``_mocap_hide_applied``): the hidden set
        depends only on the action's active bar, so every movement of the same
        action shares it. The flags are written onto EVERY loaded movement's
        start_state, which also makes each later ``set_robot_cell_state`` skip
        repositioning those bodies (compas_fab skips ``is_hidden`` rigid bodies),
        so switching movements no longer re-adds and re-removes the built bars.

        Args:
            state: The RobotCellState whose rigid_body_states to edit in place.
            sync_visibility (bool): Also redraw the flagged bodies (faint) in the
                PyBullet view. True for the state whose scene is currently shown;
                False when only tagging the other movements' states (their bodies
                are the same ones, already drawn).
        """
        rb_states = getattr(state, 'rigid_body_states', None) or {}
        active = getattr(self, 'active_bar_name', None)
        hidden = []
        for name, rb in rb_states.items():
            if name == active:
                continue  # never hide the grasped bar itself
            if rb.attached_to_tool or rb.attached_to_link:
                continue  # grasped bar's joints / any held body: keep checked
            if not is_built_assembly_body(name):
                continue  # environment obstacles etc.: keep shown + checked
            if rb.is_hidden:
                continue  # hidden by the export (not built yet), or flagged earlier
            rb.is_hidden = True
            hidden.append(name)
        # Read through getattr: scripts/derive_m1_headless.py passes a stand-in
        # object that has no such set.
        self._collision_ignored_bodies = (
            set(getattr(self, '_collision_ignored_bodies', None) or ()) | set(hidden))
        # Mirror the flags into the PyBullet view, so it shows that planning/IK
        # ignore these bodies (drawn faint) -- see _sync_pp_visibility_to_hidden.
        if sync_visibility:
            self._sync_pp_visibility_to_hidden(state)

    def _sync_pp_visibility_to_hidden(self, state):
        """Draw each ``is_hidden`` rigid body faint or transparent in the PyBullet scene.

        ``is_hidden`` only tells the cfab planner / IK to skip collisions with a
        body; the body itself stays drawn. This makes the picture match the
        model:
          - a built body the planner only ignores (``_collision_ignored_bodies``)
            is still there: drawn faint (``BUILT_IGNORED_RGBA``) at its state
            frame;
          - any other hidden body (the export hides it: not built yet, or the
            not-yet-mounted active bar) is blanked (``TRANSPARENT``).
        Its pre-hide colour is cached in ``_traj_ghost_orig_colors``
        -- the same cache ``load_selected_movement`` restores from at the top of
        the next load -- so the body reappears (with its original colour) once
        it is no longer hidden. Only the hide direction is applied here; the
        restore is that reload-time pass.

        Args:
            state: The RobotCellState whose ``rigid_body_states`` drive
                visibility. Its bodies are matched to PyBullet ids via
                ``self.cfab.client.rigid_bodies_puids``.
        """
        if self.cfab is None or getattr(self.cfab, 'client', None) is None:
            return
        puids_by_name = self.cfab.client.rigid_bodies_puids or {}
        rb_states = getattr(state, 'rigid_body_states', None) or {}
        ignored = getattr(self, '_collision_ignored_bodies', None) or set()
        for name, rb in rb_states.items():
            if not bool(getattr(rb, 'is_hidden', False)):
                continue
            faint = name in ignored
            for body in puids_by_name.get(name, []) or []:
                # Cache the current colour once so the reload-time restore is
                # lossless (skip if already cached this load).
                if body not in self._traj_ghost_orig_colors:
                    try:
                        vis = p.getVisualShapeData(body)
                        self._traj_ghost_orig_colors[body] = (
                            list(vis[0][7]) if vis else [0.7, 0.7, 0.7, 1.0])
                    except Exception:
                        self._traj_ghost_orig_colors[body] = [0.7, 0.7, 0.7, 1.0]
                try:
                    pp.set_color(body, BUILT_IGNORED_RGBA if faint else TRANSPARENT)
                except Exception:
                    pass
                if faint:
                    # ! compas_fab does not move is_hidden bodies, so place it here.
                    # Only static bodies are ever flagged (see
                    # _hide_built_assembly_for_mocap), so the state frame is its pose.
                    pp.set_pose(body, pose_from_frame(rb.frame))

    def _hide_unmounted_active_bar(self, mv) -> None:
        """Hide the active bar in M0, where it is not mounted yet.

        ! The Rhino export leaves the active bar in M0's start_state at its
        ! ASSEMBLED pose (loose, not hidden). But between M0 and M1 an operator
        ! mounts the bar on the robot's hands, so during M0 the bar is not in the
        ! scene at all. Left there, it makes a derived M1 start (= M0's goal)
        ! that reaches through the bar's future place fail M0 as "end
        ! configuration is in collision", and it is drawn where nothing is.
        ! The planner is told to ignore it; the view blanks it at Load Movement.

        Only the travel to the loading pose is touched: in the retreat and the
        free move home the bar really is installed there.
        Mutates ``mv.start_state`` in place.

        Args:
            mv: The movement whose start_state to edit.
        """
        if not self._is_free_to_load(mv):
            return
        bar_rb = (mv.start_state.rigid_body_states.get(self.active_bar_name)
                  if mv.start_state is not None and self.active_bar_name else None)
        if bar_rb is not None and not bar_rb.attached_to_link and not bar_rb.is_hidden:
            bar_rb.is_hidden = True
            print(f"[{mv.movement_id}] ignoring the not-yet-mounted active bar "
                  f"{self.active_bar_name!r} at its assembled pose.")

    def _authored_motion_type(self, mv) -> str:
        """The motion type implied by a movement's authored start state.

        ``'bar_held'`` when the start state has a built-assembly body (the bar or
        one of its joints) attached to a robot link or a tool, else ``'free'``.
        That covers the arm movements that carry the bar (transfer, insert) and
        also the tool and manual steps in between, so those draw the bar in the
        tools too. Used for the goal-state / first-trajectory preview at Load
        Movement; the replan buttons override it with the type of the plan they
        actually ran.

        Args:
            mv: The Movement to classify.

        Returns:
            str: ``'bar_held'`` or ``'free'``.
        """
        state = getattr(mv, 'start_state', None)
        if state is None:
            return 'free'
        rb_states = getattr(state, 'rigid_body_states', None) or {}
        held = any(is_built_assembly_body(name)
                   and (rb.attached_to_link or rb.attached_to_tool)
                   for name, rb in rb_states.items())
        return 'bar_held' if held else 'free'

    def _refresh_preview_attached_bodies(self, motion_type, held_state):
        """Recolor the cfab bar/joint/tool bodies so the green preview robot
        carries exactly what the staged trajectory holds.

        The preview reuses the cfab-spawned rigid bodies (no separate copies):
        the ones that should ride are colored ``TRAJECTORY_GREEN`` and re-posed
        each tick from the goal-model FK (see ``update``); a bar/joint that must
        NOT ride is blanked (``TRANSPARENT``). This is driven by ``motion_type``
        -- NOT the authored ``held_state`` alone -- so a FREE transit to a
        bar-held movement's start shows no bar, and a BAR_HELD transfer to a
        movement whose authored start has the bar released still shows it (the
        transfer path force-attaches the bar + joints into ``held_state`` first).

        Tool bodies (the grippers, when present as rigid bodies) always ride,
        both free and bar_held, since they are the actual TCP geometry.

        Args:
            motion_type (str): ``'free'`` or ``'bar_held'``.
            held_state (RobotCellState): State whose ``rigid_body_states``
                supply the attachments (bar + joints + tools) and their grasps.
        """
        self.planned_trajectory_motion_type = motion_type
        # Restore the previously-previewed bodies to their original colors, then
        # forget them (leaves the hidden built-assembly cache untouched).
        # Read through getattr: the headless harnesses build a monitor without
        # running __init__ and only stub the attributes they know about.
        for body, c in list((getattr(self, '_preview_body_orig_colors', None) or {}).items()):
            try:
                pp.set_color(body, c)
            except Exception:
                pass
        self._preview_body_orig_colors = {}
        self._traj_ghost_bodies = []
        if held_state is None or self.cfab is None or getattr(self.cfab, 'client', None) is None:
            return

        # In a free motion the bar/joints are NOT mounted, so blank them; only
        # tool bodies keep riding.
        hide_bar_joints = (motion_type == 'free')
        _TOOL_BODY_NAMES = {'AssemblyLeftArmToolBody', 'AssemblyRightArmToolBody'}
        rb_states = getattr(held_state, 'rigid_body_states', {}) or {}
        for name, rbs in rb_states.items():
            if getattr(rbs, 'attached_to_link', None) is None:
                continue
            if getattr(rbs, 'attachment_frame', None) is None:
                continue
            ids = (self.cfab.client.rigid_bodies_puids or {}).get(name) or []
            if not ids:
                continue
            body = ids[0]
            # Cache the current color once so the next refresh / load restores it.
            # ! A body _sync_pp_visibility_to_hidden drew faint / blank in another
            # ! movement (e.g. B1's joints: standing in J_M0, held in J_M3) keeps its
            # ! true colour in that cache; take it from there, or the faint colour
            # ! would come back as this body's "original".
            if body not in self._preview_body_orig_colors and body in self._traj_ghost_orig_colors:
                self._preview_body_orig_colors[body] = list(self._traj_ghost_orig_colors[body])
            if body not in self._preview_body_orig_colors:
                try:
                    vis = p.getVisualShapeData(body)
                    self._preview_body_orig_colors[body] = (
                        list(vis[0][7]) if vis else [0.7, 0.7, 0.7, 1.0])
                except Exception:
                    self._preview_body_orig_colors[body] = [0.7, 0.7, 0.7, 1.0]

            if hide_bar_joints and name not in _TOOL_BODY_NAMES:
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
            print(f"[preview] {motion_type}. ")
            # bar/joints ride f"{[g['link'] for g in self._traj_ghost_bodies]}")
        elif hide_bar_joints:
            print(f"[preview] {motion_type}: bar/joints hidden (not mounted).")

    def _slider_index(self, slider, cached_idx: int, n_items: int) -> int:
        """Resolve an index slider to a valid position in a list of `n_items`.

        The same two steps `load_bar_action_file` and `load_selected_movement`
        take: prefer the widget's live position over the cached index (a
        slider rebuilt by reset_ui can miss the next drag's callback), then
        clamp into range.

        Args:
            slider (Slider | None): The widget, or None when it was skipped
                (a 1-entry slider would segfault pybullet, so it is not made).
            cached_idx (int): The index its on-change callback last stored.
            n_items (int): Length of the list being indexed.

        Returns:
            int: A valid index, or -1 when the list is empty.
        """
        if n_items <= 0:
            return -1
        idx = cached_idx
        if slider is not None:
            v = slider.value
            if v is not None:
                idx = int(round(float(v)))
        return max(0, min(idx, n_items - 1))

    def _refresh_goal_view_readout(self):
        """Keep the '-> view' line in step with `show_goal_state`.

        Mirrors the flag rather than updating the line at each call site:
        `set_to_show_goal_state` / `set_to_show_traj_state` flip it from all
        over the codebase (a finished plan switches to the trajectory view by
        itself), so the button is far from the only thing that changes it.
        """
        text = getattr(self, 'goal_view_text', None)
        if text is not None:
            text.set_text("GOAL conf (blue)" if self.show_goal_state
                          else "planned TRAJECTORY (green)")

    def _refresh_bar_action_readouts(self):
        """Point the two readout lines at whatever the sliders now show.

        Called every UI tick. Reads the live widget positions rather than the
        cached indices, so the text always names the file / movement that
        'Load BarAction' and 'Load Movement' would actually act on -- a label
        that lagged a drag would be worse than no label at all.
        """
        text = getattr(self, 'bar_action_file_text', None)
        if text is not None:
            files = self.available_bar_actions or []
            idx = self._slider_index(getattr(self, 'bar_action_file_slider', None),
                                     self._selected_action_file_idx, len(files))
            text.set_text("(no BarAction files)" if idx < 0 else
                          f"[{idx}] {os.path.basename(files[idx])}")

        text = getattr(self, 'bar_movement_text', None)
        if text is not None:
            movements = self._shown_movements() or []
            idx = self._slider_index(getattr(self, 'bar_movement_slider', None),
                                     self._selected_movement_idx, len(movements))
            if idx < 0:
                text.set_text("(load a BarAction first)")
            else:
                mv = movements[idx]
                # The id plus its kind (what the planner and exec steer by). The
                # slider itself shows the index.
                kind = self._kind_of(mv)
                kind_text = kind.value if kind is not None else type(mv).__name__
                text.set_text(f"{mv.movement_id}  ({kind_text})")

    def _set_default_trajectory_time(self, mv) -> bool:
        """Set ``self.trajectory_time`` (the "traj time" slider) to a movement's default.

        The movement kind's default (``bar_action_io.default_trajectory_time``);
        Cindy's free move home gets its own, shorter one. The slider shows the
        value at its next rebuild (``reset_ui``), so callers set it before that.

        Args:
            mv: A movement of the loaded action.

        Returns:
            bool: True when set; False for a step where no arm moves (or a
            movement class bar_action_io does not know), which keeps the slider.
        """
        free_home = self._is_free_home(mv)
        try:
            default_traj_time = default_trajectory_time(mv, free_home=free_home)
        except TypeError as e:
            self.get_logger().warn(f"No default traj time for {mv.movement_id!r}: {e}")
            return False
        if default_traj_time is None:
            return False
        # Keep it inside the slider's own range, or the rebuilt widget would
        # clamp and silently disagree with self.trajectory_time.
        self.trajectory_time = float(
            min(max(default_traj_time, 1.0), self.trajectory_time_max))
        print(f"[Movement] traj time -> {self.trajectory_time:.0f}s "
              f"(default for {movement_kind(mv).value}"
              f"{', free move home' if free_home else ''})")
        return True

    def load_selected_movement(self):
        """Load the selected movement's start state into cfab + goal ghost.

        In schedule mode the planned-trajectory slots and the joint preview are
        emptied first (and refilled below when the movement carries its own
        trajectory), so 'Exec' can never replay the previous movement's path.
        """
        if self._refuse_while_tasks_run('Load Movement', schedule_only=True):
            return
        # ! Another robot's entry is only shown: its states name that robot's
        # ! joints, which the connected robot's cell does not have.
        if self._refuse_display_only_entry('Load Movement'):
            return
        if not self._loaded_movements:
            self.get_logger().warn(f"No BarAction loaded; click {self._load_hint()} first.")
            return
        # Same shared resolution as load_bar_action_file, so this loads
        # exactly the movement the '-> movement' readout names.
        idx = self._slider_index(getattr(self, 'bar_movement_slider', None),
                                 self._selected_movement_idx,
                                 len(self._loaded_movements))
        self._selected_movement_idx = idx
        mv = self._loaded_movements[idx]

        # If M0 (or an action's first movement with no authored start conf),
        # re-snapshot live conf/base into its start_state so a robot that moved
        # since 'Load BarAction' still plans from where it is. The set was
        # worked out at load, before the first injection filled the null conf.
        if idx in getattr(self, '_live_start_indices', set()):
            self._inject_live_conf_into_state(mv.start_state)

        if mv.start_state is None:
            self.get_logger().warn(f"Movement {mv.movement_id!r} has no start_state.")
            return

        if self.cfab is None:
            self.get_logger().warn(f"cfab not initialized; click {self._load_hint()} first.")
            return

        # * Schedule mode: drop the previous movement's planned path and joint
        # * preview together with the movement itself. A movement that carries
        # * its own trajectory re-fills both at the end of this load.
        if getattr(self, '_schedule', None) is not None:
            self._reset_planned_arm_trajectory()
            self._preview_joint_data = None

        self.current_action = self._loaded_action
        self.current_movement = mv
        self.current_movement_index = idx
        self.movement_start_state = mv.start_state
        self.target_ee_frames = mv.target_ee_frames or None
        bar_id = getattr(self._loaded_action, 'active_bar_id', None) if self._loaded_action else None
        self.active_bar_name = self._active_bar_body_name(mv.start_state, bar_id)

        # Point the "traj time" slider at this movement's default duration. Set it
        # BEFORE the reset_ui() below, which rebuilds the slider with
        # self.trajectory_time as its current value -- and before the
        # auto-load of the trajectory at the end of this method, since
        # _accept_trajectory stamps this duration onto planned_arm_trajectory.
        self._set_default_trajectory_time(mv)

        # When the built assembly is ignored (_ignore_built_assembly) it is hidden
        # once per BarAction, so a plain movement switch must NOT un-hide it: skipping
        # the restore below (and the re-hide further down) is what stops the
        # built bars from flashing back in and out on every Load Movement. The
        # colour cache is deliberately kept: _sync_pp_visibility_to_hidden only
        # caches a body it hasn't seen, so keeping it preserves the TRUE
        # original colours (clearing it would later cache TRANSPARENT as the
        # "original" and lose them for good).
        skip_built_assembly_resync = (self._ignore_built_assembly()
                                      and getattr(self, '_mocap_hide_applied', False))

        if not skip_built_assembly_resync:
            # Restore built-assembly bodies hidden on the previous load before
            # pushing the new state (the preview bodies are restored by the
            # helper below, from their own cache).
            for body, c in list(self._traj_ghost_orig_colors.items()):
                try:
                    pp.set_color(body, c)
                except Exception:
                    pass
            self._traj_ghost_orig_colors = {}

        self._hide_unmounted_active_bar(mv)
        # Re-pose the other robots so the view shows where they are now (a
        # husky mocap started tracking since the load moves to its live base).
        self._apply_obstacle_beliefs(mv.start_state)
        try:
            self.cfab.planner.set_robot_cell_state(mv.start_state)
        except Exception as e:
            print(f"Error setting cfab robot cell state: {e}")
            return
        self._check_cfab_base_matches(mv, 'LoadMovement')
        try:
            self._bridge_cfab_to_pp_for_bar_action()
        except Exception as e:
            print(f"Error bridging cfab scene to pp: {e}")
            return

        rb_states = getattr(mv.start_state, 'rigid_body_states', {}) or {}
        bar_rb = rb_states.get(self.active_bar_name) if self.active_bar_name else None
        self.grasp_link_from_bar = bar_rb.attachment_frame if (bar_rb and bar_rb.attachment_frame) else None

        # Set up the preview bar/joints for the movement's AUTHORED type (goal
        # state + first trajectory): bar held when its start state has the bar
        # attached (see _authored_motion_type). Each
        # planner call site re-runs this with the real motion type afterwards,
        # so e.g. a free transit to a bar-held movement's start still shows no
        # bar.
        # ! Done BEFORE the hidden bodies are drawn below: it first gives the
        # ! previous movement's preview bodies their colours back, and a joint
        # ! held there may be a built body ignored here (B1's joints: held in
        # ! R_M0, standing in R_M2). Done after, it would undo the faint colour
        # ! and leave the preview's colour as the "original" in the hidden cache.
        self._refresh_preview_attached_bodies(
            self._authored_motion_type(mv), mv.start_state)

        # Only when the built assembly is ignored (_ignore_built_assembly: the
        # schedule-mode switch, or the bar-holding accuracy experiment): ignore
        # collisions with the already-built assembly (static bars/joints) for all
        # subsequent planning/IK, keeping self, tools, and the grasped bar + its joints
        # checked. Done ONCE PER BARACTION, on the first Load Movement after the
        # action is parsed -- and after the state push above, so the built
        # bodies are spawned/positioned in the scene before they're flagged.
        # The flags go onto EVERY movement's start_state (they're all the same
        # action, so the same hidden set applies): later movements keep their
        # planning/IK correct AND their set_robot_cell_state skips repositioning
        # those bodies, so no more add/remove churn when switching movements.
        if (self._ignore_built_assembly()
                and not getattr(self, '_mocap_hide_applied', False)):
            self._hide_built_assembly_for_mocap(mv.start_state)
            for other in self._loaded_movements:
                if other is mv or getattr(other, 'start_state', None) is None:
                    continue
                self._hide_built_assembly_for_mocap(
                    other.start_state, sync_visibility=False)
            self._mocap_hide_applied = True
            self.get_logger().info(
                f"[built bars] collisions with {len(self._collision_ignored_bodies)} "
                f"built bodies ignored (drawn faint)")

        # Blank every body this state marks hidden (built bodies the planner only
        # ignores: drawn faint), so the view shows what the
        # planner checks -- e.g. the not-yet-mounted active bar in M0 (hidden
        # just before the state push above). The restore at the top of the next
        # load brings it back.
        self._sync_pp_visibility_to_hidden(mv.start_state)

        if mv.start_state.robot_configuration is not None:
            rc = mv.start_state.robot_configuration
            try:
                # One entry per arm of the loaded cell (2 for Cindy, 1 otherwise).
                for i, names in enumerate(self._arm_joint_name_sets()):
                    self.goal_arm_pose[i] = np.array([rc[n] for n in names])
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

        kind = self._kind_of(mv)
        print(f"[Movement] loaded [{idx}] {mv.movement_id!r} type={type(mv).__name__} "
              f"kind={kind.value if kind is not None else None} "
              f"has_targets={bool(mv.target_ee_frames)} traj={mv.trajectory is not None}")

        # If mv already carries a trajectory in memory (loaded from a
        # `.live-solved.json` sidecar), auto-wire it into the viz so the
        # traj-viz time slider is immediately previewable without another
        # click on 'Load Movement Trajectory'.
        if getattr(mv, 'trajectory', None) is not None:
            self.load_selected_movement_trajectory()

    def plan_selected_movement(self):
        """Plan the loaded movement with the planner of its kind (``_plan_by_kind``)."""
        if self._refuse_while_tasks_run('Plan Movement', schedule_only=True):
            return
        if self._refuse_display_only_entry('Plan Movement'):
            return
        if self.current_movement is None:
            self.get_logger().warn("No movement loaded; click 'Load Movement' first.")
            return
        self._plan_by_kind(self.current_movement)

    def _plan_by_kind(self, mv) -> None:
        """Plan a movement by its movement kind, then accept the trajectory.

        Cindy (dual arm):
        - DUAL_FREE: the travel to the loading pose (``_plan_M0_dispatch``) or,
          when ``_is_free_home``, the free move home (``_plan_M4_dispatch``).
        - DUAL_CONSTRAINED_FREE: the bar transfer (``_plan_M1_dispatch``).
        - DUAL_CONSTRAINED_LINEAR: the insert (``_plan_M2_dispatch``).
        - DUAL_INDEPENDENT_LINEAR: the retreat (``_plan_M3_dispatch``).
        Support robot (single arm):
        - SINGLE_FREE (e.g. ``H_M0``): joint-space BiRRT from the LIVE arm joints
          to the authored ``target_configuration``.
        - SINGLE_LINEAR (e.g. ``H_M2``): straight flange line from the movement's
          start configuration (the previous movement's end, chained by
          ``_accept_trajectory``) to ``target_ee_frames['arm']``.
        Tool / manual steps: nothing to plan (no arm moves).
        Every plan runs at the live base with the other robots posed from
        their beliefs (``_apply_live_base_to_movement``).

        Args:
            mv: The loaded Movement (``self.current_movement``).
        """
        kind = self._kind_of(mv)
        if kind is None:
            self.get_logger().warn(
                f"Can not plan {mv.movement_id!r}: unknown movement class; skipping.")
            return
        if kind in STATIONARY_KINDS:
            self.get_logger().info(
                f"Nothing to plan for {mv.movement_id!r} ({kind.value}): no arm moves in it.")
            return
        spec = self._connected_robot()
        # ! The legacy file slider still lists every robot's files (B3__H, B3__HR),
        # ! so a movement of another robot can end up loaded here.
        if not kind_fits_robot(kind, spec.dual_arm):
            arms = 'dual-arm' if kind in DUAL_ARM_KINDS else 'single-arm'
            self.get_logger().warn(
                f"{mv.movement_id!r} is a {arms} movement ({kind.value}); the "
                f"connected robot is {spec.name}. Not planning it.")
            return
        if mv.trajectory is not None:
            self.get_logger().warn(f"Overwriting existing trajectory for {mv.movement_id!r}")

        if kind in DUAL_ARM_KINDS:
            # * Cindy: the preamble her four planners share.
            # Re-read the swept-check slider's LIVE position: a rebuilt widget can
            # miss its next drag callback, and this one decides whether a free
            # move's plan is verified at all. Same hazard as the other exec sliders.
            sld = getattr(self, 'fm_swept_validation_slider', None)
            if sld is not None:
                v = sld.value
                if v is not None and bool(round(float(v))) != self.fm_swept_validation_enabled:
                    self.fm_swept_validation_enabled = bool(round(float(v)))
                    print(f"[Plan] swept collision check: "
                          f"{'ON' if self.fm_swept_validation_enabled else 'OFF'}")

            # Plan against the LIVE husky base, not the BarAction-authored one.
            # Mutates mv.start_state.robot_base_frame in place + pushes to cfab.
            if not self._apply_live_base_to_movement(mv):
                return

            # Authored states may carry no robot_configuration (the transfer
            # before its chain is planned): give the planner's IK a home seed to
            # work from (same as fill_missing_config in headless_bar_action_planner).
            # ! Dual-arm only: a support robot's linear move must keep refusing a
            # ! missing start configuration (below) instead of sweeping from home.
            self._fill_missing_start_conf(mv.start_state)

            # ! Built per call from the instance attributes: the dispatch check
            # ! (headless_schedule_smoke) replaces these methods on the instance.
            planner = {
                MovementKind.DUAL_FREE: (self._plan_M4_dispatch if self._is_free_home(mv)
                                         else self._plan_M0_dispatch),
                MovementKind.DUAL_CONSTRAINED_FREE: self._plan_M1_dispatch,
                MovementKind.DUAL_CONSTRAINED_LINEAR: self._plan_M2_dispatch,
                MovementKind.DUAL_INDEPENDENT_LINEAR: self._plan_M3_dispatch,
            }[kind]
            jt = planner(mv)
            if jt is None and kind is MovementKind.DUAL_CONSTRAINED_FREE:
                self._clear_m1_start_conf_without_trajectory()
        elif kind == MovementKind.SINGLE_FREE:
            group = spec.planning_groups[0]
            if mv.target_configuration is None:
                self.get_logger().warn(
                    f"{mv.movement_id!r} has no target_configuration to plan to.")
                return
            # A free approach starts wherever the arm is right now.
            self._inject_live_conf_into_state(mv.start_state)
            if not self._apply_live_base_to_movement(mv):
                return
            # Pause GUI rendering during the search (does nothing when headless).
            with pp.LockRenderer():
                path, info = plan_free_motion(
                    self.cfab.planner, mv.start_state, mv.target_configuration,
                    group=group, max_time=30.0, max_iterations=100)
            if path is None:
                self.get_logger().warn(
                    f"[single-arm free] {mv.movement_id!r}: {info.get('failure_reason')}")
            # Name the waypoints in the order plan_free_motion wrote them.
            jt = (joint_trajectory_from_path(
                      path, arm_joint_names_for_group(self.cfab.robot_cell, group))
                  if path is not None else None)
        else:
            group = spec.planning_groups[0]
            side = spec.side_keys[0]
            target = (mv.target_ee_frames or {}).get(side)
            if target is None:
                self.get_logger().warn(
                    f"{mv.movement_id!r} has no target_ee_frames[{side!r}] to move "
                    f"the flange to; not planning it.")
                return
            # ! A linear move from a made-up (home) start would sweep somewhere
            # ! else entirely, so refuse instead of filling one in.
            if mv.start_state.robot_configuration is None:
                self.get_logger().warn(
                    f"{mv.movement_id!r} has no start configuration: plan the "
                    f"movement before it first (its end becomes this start).")
                return
            if not self._apply_live_base_to_movement(mv):
                return
            with pp.LockRenderer():
                jt = plan_linear_motion(self.cfab.planner, mv.start_state, target, group=group)

        if jt is None:
            self.get_logger().warn(f"Plan for {mv.movement_id!r} ({kind.value}) FAILED.")
            # ! Clear the preview. Without this, planned_arm_trajectory still
            # ! holds the PREVIOUS movement's path, so the traj-viz scrub and the
            # ! joint plot keep animating that one -- which reads as "the plan I
            # ! just asked for is bad" (e.g. the previous movement's bar-held
            # ! path sweeping past the structure) when in fact no plan was
            # ! produced at all. Showing nothing is the honest state.
            self._reset_planned_arm_trajectory()
            self._preview_joint_data = None
            self._draw_preview_joint_values()
            self.get_logger().warn(
                f"Trajectory preview CLEARED -- it was still showing the "
                f"previously loaded movement, not {mv.movement_id!r}.")
            return
        self._accept_trajectory(mv, jt, source='Plan')

    # --- --- --- Chain planning (Button 1) --- --- ---

    # * Canonical plan order, by movement kind. The old order was transfer ->
    # * insert -> retreat -> travel to load -> free move home; DUAL_FREE covers
    # * the last two, travel to load first (see _chain_sequence).
    #   transfer (DCF) owns its derived start (`derive_start=True`)
    #   -> insert (start comes from the transfer's last waypoint)
    #   -> retreat (start comes from the insert's last waypoint)
    #   -> travel to load (goal = the transfer's start, backfilled after it)
    #   -> free move home (goal is the fixed home)
    _CHAIN_KIND_ORDER = (
        MovementKind.DUAL_CONSTRAINED_FREE,
        MovementKind.DUAL_CONSTRAINED_LINEAR,
        MovementKind.DUAL_INDEPENDENT_LINEAR,
        MovementKind.DUAL_FREE,
    )

    def plan_movement_chain_live(self):
        # TODO this should be moved to husky_planning.py
        """Plan transfer -> insert -> retreat -> travel to load -> free move home, live.

        For each index of ``_chain_sequence()`` (the loaded movements of the kinds
        in ``_CHAIN_KIND_ORDER``): set the movement slider to that index, call
        ``load_selected_movement()`` so the cfab scene / goal viz sync, then
        call ``plan_selected_movement()``. ``plan_selected_movement`` already
        applies the live base via ``_apply_live_base_to_movement``, warm-starts
        IK from any stored start conf, dispatches to the kind's
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
        if self._refuse_while_tasks_run('Plan Chain', schedule_only=True):
            return
        if self._refuse_display_only_entry('Plan Chain'):
            return
        if not self._loaded_movements:
            self.get_logger().warn(
                f"No movements loaded; click {self._load_hint()} first."
            )
            return
        if not self._current_action_path:
            self.get_logger().warn(
                f"No BarAction file path known (was it loaded via {self._load_hint()}?)."
            )
            return

        # The ordered index list, by kind (see _chain_sequence).
        sequence = self._chain_sequence()
        if not sequence:
            self.get_logger().warn(
                "[Plan Chain] no movement of kind "
                f"{', '.join(k.value for k in self._CHAIN_KIND_ORDER)}; nothing to plan."
            )
            return

        # Wipe any pre-existing in-memory trajectories on the movements we're
        # about to plan so _accept_trajectory's rejection-on-mismatch does
        # not warn about a stale value we intentionally overwrite. Movements
        # NOT in the sequence (tool / manual steps) keep their trajectories.
        for i in sequence:
            self._loaded_movements[i].trajectory = None

        planned_ids = []
        stopped_at = None
        for step, idx in enumerate(sequence, start=1):
            mv = self._loaded_movements[idx]
            kind = self._kind_of(mv)
            print(f"\n=== [Plan Chain {step}/{len(sequence)}] {kind.value} idx={idx} "
                  f"id={mv.movement_id!r} ===")

            # Simulate the UI: slider -> Load Movement -> Plan Movement.
            self._selected_movement_idx = idx
            self.load_selected_movement()
            self.plan_selected_movement()

            planned_traj = getattr(self.current_movement, 'trajectory', None)
            if planned_traj is None:
                stopped_at = mv.movement_id
                self.get_logger().warn(
                    f"[Plan Chain] {kind.value} ({mv.movement_id!r}) FAILED; "
                    "stopping chain. Previously planned movements are kept."
                )
                break
            planned_ids.append(mv.movement_id)

        # Export a sidecar for every half that now carries a trajectory.
        # _loaded_movements shares object identity with the loaded actions'
        # movements (see load_bar_action_file), so the plans are already in them.
        planned = [mv for mv in self._loaded_movements
                   if getattr(mv, 'trajectory', None) is not None]
        if planned:
            written = self._write_action_halves_for(planned, 'Plan Chain')
            if written:
                print(f"[Plan Chain] sidecar written -> {', '.join(written)}")
        else:
            print("[Plan Chain] no trajectories to export; skipping sidecar.")

        if stopped_at is None:
            print(f"[Plan Chain] SUCCESS: planned "
                  f"{len(planned_ids)}/{len(sequence)} movements.")
        else:
            print(f"[Plan Chain] STOPPED at {stopped_at!r}: planned "
                  f"{len(planned_ids)}/{len(sequence)} movements before failure.")
        self._print_movement_roster(tag='Plan Chain')

    # --- --- --- Reset (per-movement + all) --- --- ---

    def reset_selected_movement_to_clean(self):
        """Revert the currently loaded movement to its authored 'clean' state.

        Re-reads the pristine BarAction JSON of the half this movement came from
        (the cycle can span a jointing and a release file) and overwrites the
        movement in both ``self._loaded_movements`` and that half's own
        ``movements`` list with the clean-file version (fresh ``start_state``, no
        propagated ``robot_configuration`` from a downstream chain break, and
        ``trajectory=None``). Other movements are untouched: their propagated
        start_confs may now be stale, and a subsequent 'Plan Chain (Live)'
        will re-populate them.
        """
        if self._refuse_while_tasks_run('Reset Selected Mv to Clean', schedule_only=True):
            return
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
        slot = self._slot_of_movement(idx)
        if idx is None or slot is None:
            self.get_logger().warn(
                "Loaded-action state missing; cannot reset."
            )
            return
        # Re-read this movement's OWN half, at its index within that half.
        action, half_path, local_idx = slot
        read_path = half_path
        if getattr(self, '_schedule', None) is not None:
            # * Schedule mode: the loaded file may be the entry's .live-solved
            # * sidecar; "clean" means the exported file next to it.
            clean_path = clean_action_path(half_path)
            if os.path.isfile(clean_path):
                read_path = clean_path
        try:
            clean = parse_bar_action(read_path)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse clean BarAction: {e}")
            return
        if local_idx >= len(clean.movements):
            self.get_logger().warn(
                f"Clean file has {len(clean.movements)} movements; index "
                f"{local_idx} out of range."
            )
            return
        clean_mv = clean.movements[local_idx]
        # Replace by index in BOTH lists so identity stays consistent for
        # any subsequent sidecar export.
        self._loaded_movements[idx] = clean_mv
        action.movements[local_idx] = clean_mv
        self.current_movement = clean_mv
        # The cycle's start-EE map points at movement OBJECTS, and this just
        # swapped one, so rebuild it rather than leave an entry naming the
        # replaced object. A schedule entry's first movements start where its
        # predecessor entry (J before R, H before HR) ended, so walk that too.
        loaded = getattr(self, '_loaded_entry_bundle', None)
        pred = (list(loaded.predecessor.movements) if getattr(self, '_schedule', None) is not None
                and loaded is not None and loaded.predecessor is not None else [])
        self._loaded_start_ee_sources = cycle_start_ee_sources(
            pred + self._loaded_movements, self._connected_robot().side_keys)[len(pred):]
        print(f"[Reset Mv] reverted [{idx}] {clean_mv.movement_id!r} to clean.")
        # The freshly parsed state carries neither the ground body (the cell has
        # it, and compas_fab requires cell and state to agree) nor the
        # `is_hidden` flags, so both must be re-applied -- otherwise the state
        # push raises, or this movement collides against the built assembly
        # again. Clearing the flag makes the Load Movement below redo the hide.
        self._inject_ground_rigid_body_state(clean_mv.start_state)
        self._mocap_hide_applied = False
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

        * Schedule mode: reloads the loaded ENTRY from its clean export instead
        * (``load_schedule_entry(..., prefer_sidecar=False)``); the sidecar stays
        * on disk.
        """
        if getattr(self, '_schedule', None) is not None:
            if self._refuse_while_tasks_run('Reset All Mvs to Clean'):
                return
            entry = getattr(self, '_loaded_entry', None)
            if entry is None:
                self.get_logger().warn("No schedule entry loaded; click 'Load entry' first.")
                return
            print(f"[Reset All] reloading schedule entry {entry.index} "
                  f"({entry.action_id}) from its clean export")
            # Keep using the clean export for the rest of this run (as after a
            # Reopen), so a later plain 'Load entry' does not bring the sidecar back.
            self._clean_entries = getattr(self, '_clean_entries', set()) | {entry.index}
            self._rescan_schedule_flags([entry.index])
            self.load_schedule_entry(entry.index, prefer_sidecar=False)
            return
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
        kind = self._kind_of(self.current_movement)
        if kind not in COMPLIANT_KINDS:
            self.get_logger().warn(
                f"Replan Free -> Mv Start only works on the insert or the retreat; "
                f"{self.current_movement.movement_id} is a "
                f"{kind.value if kind is not None else type(self.current_movement).__name__}."
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
                # self._verify_replan_endpoint_matches_target()

                # This is the FREE transit (bar not mounted yet), so the
                # preview must NOT show the bar even though the target
                # movement's authored start_state may hold it.
                self._refresh_preview_attached_bodies('free', self.current_movement.start_state)
        finally:
            if revert_mock is not None:
                revert_mock()

    def _ensure_bar_attached_for_mocap(self, mv):
        """Force the active bar to be 'held' in ``mv.start_state`` for the
        bar-accuracy test.

        Protocol: the bar is physically mounted once and never dismounted, so a
        movement whose authored start_state has the bar released/installed (e.g.
        M3 retreat, where the bar rests in the world) must still be planned as a
        bar-held transfer. If the active bar's rigid body is not attached, copy
        the full mounted config -- the bar AND the joints installed on it (the
        other bodies held in the donor) -- grasp (``attached_to_link`` +
        ``attachment_frame``) and all, from a sibling movement of the same
        BarAction where the bar IS attached (prefer M2, the install approach
        whose grasp the operator physically mounts), clearing each copied body's
        static world ``frame`` so it follows the arm. Copying the joints too
        keeps the planner's collision state and the preview both showing the
        real mounted geometry, and each copied body's ``is_hidden`` is cleared so
        the newly held geometry is collision-checked and posed again. Mutates
        ``mv.start_state`` in place (persists, mirroring the real mount).

        Args:
            mv: The Movement whose start_state to edit.

        Returns:
            bool: True if the active bar is (now) attached, else False.
        """
        if not self.active_bar_name or mv is None or mv.start_state is None:
            return False
        rb_states = getattr(mv.start_state, 'rigid_body_states', None) or {}
        bar_rb = rb_states.get(self.active_bar_name)
        if bar_rb is None:
            return False
        if bar_rb.attached_to_link:
            return True  # already held

        # Find a sibling MOVEMENT whose active-bar rigid body is held; prefer the insert.
        donor_mv = None
        for mv2 in getattr(self, '_loaded_movements', []):
            if mv2 is mv or getattr(mv2, 'start_state', None) is None:
                continue
            rb2 = (getattr(mv2.start_state, 'rigid_body_states', None) or {}).get(
                self.active_bar_name)
            if rb2 is not None and rb2.attached_to_link and rb2.attachment_frame is not None:
                donor_mv = mv2
                if self._kind_of(mv2) is MovementKind.DUAL_CONSTRAINED_LINEAR:
                    break
        if donor_mv is None:
            self.get_logger().warn(
                f"[mocap-acc] cannot inject bar attachment for "
                f"{self.active_bar_name!r}: no sibling movement has it held.")
            return False

        # Copy every held body from the donor (bar + its installed joints) into
        # this movement's start_state, so the mounted geometry matches exactly.
        donor_rbs = getattr(donor_mv.start_state, 'rigid_body_states', None) or {}
        injected = []
        for name, drb in donor_rbs.items():
            if not (drb.attached_to_link and drb.attachment_frame is not None):
                continue
            target_rb = rb_states.get(name)
            if target_rb is None or target_rb.attached_to_link:
                continue  # missing here, or already held
            target_rb.attached_to_link = drb.attached_to_link
            target_rb.attachment_frame = drb.attachment_frame
            target_rb.frame = None  # held body has no static world frame (FK from the link)
            # The once-per-action built-assembly hide may have flagged this body
            # while it was still resting in the world. Now that it rides with the
            # arm it must be collision-checked and repositioned again --
            # compas_fab skips both steps for a hidden body.
            target_rb.is_hidden = False
            # ! The allowed-collision lists come along too. The authored release
            # ! state lets the bar and its joints touch only the tools; once
            # ! both are held again they overlap each other by design (the
            # ! joint is fitted onto the bar), and cfab's CC.4 checks every
            # ! held body against every other body unless one lists the other.
            target_rb.touch_bodies = sorted(set(target_rb.touch_bodies or []) | set(drb.touch_bodies or []))
            target_rb.touch_links = sorted(set(target_rb.touch_links or []) | set(drb.touch_links or []))
            injected.append(name)
        # Held bodies may touch each other (bar <-> its fitted joints), both ways.
        held = [n for n, rb in rb_states.items() if rb.attached_to_link]
        for name in held:
            rb_states[name].touch_bodies = sorted(
                set(rb_states[name].touch_bodies or []) | (set(held) - {name}))
        self.grasp_link_from_bar = bar_rb.attachment_frame
        self.get_logger().info(
            f"[mocap-acc] injected held attachment for {len(injected)} body(ies) "
            f"from {donor_mv.movement_id!r}: {injected}.")
        return bool(bar_rb.attached_to_link)

    def replan_transfer_to_movement_start_live(self, show_validation=True):
        """Fresh live-base IK to the movement's start EE targets, then a
        CONSTRAINED dual-arm ("transfer") plan from the live conf to that
        IK-solved conf, keeping the mounted bar's rigid grasp intact.

        Bar-held sibling of ``replan_free_to_movement_start_live`` (Button
        2). Once the bar is manually mounted in the grippers it stays
        mounted for the whole servoing session, so every move between
        targets must keep both tool0s rigidly locked to the bar. The tamp
        constrained planner (the same one M1 uses) enforces exactly that:
        with ``derive_start=False`` it trusts the live start conf, reads
        the bar's live pose from the state (the bar is ATTACHED in the
        M2/M3 start_state, so it follows the injected live arm conf),
        derives the rigid grasps by FK, and plans a bar-constrained path
        to the bar pose implied by the IK-solved goal conf.

        Only supports M2 / M3 (same reason as Button 2), and requires the
        bar to be attached in the movement's start_state — that attachment
        is what makes the planner treat the bar as held.

        Args:
            show_validation (bool): When True (button default), draw the
                pre-execution safeguard curves (joint continuity + bar-hold
                EE drift) in the "Movement Preview" DPG window and log a
                PASS/FAIL verdict. The transfer servoing loop passes False on
                its small later-iteration corrections so only the first (large)
                move is gated.
        """
        if self.current_movement is None:
            self.get_logger().warn(
                "No movement loaded; click 'Load Movement' first."
            )
            return
        mv = self.current_movement
        kind = self._kind_of(mv)
        if kind not in COMPLIANT_KINDS:
            self.get_logger().warn(
                f"Replan Transfer -> Mv Start only works on the insert or the retreat; "
                f"{mv.movement_id} is a "
                f"{kind.value if kind is not None else type(mv).__name__}."
            )
            return
        # ! The bar must be attached in the start_state: the constrained
        # planner derives the rigid grasp from it, and set_robot_cell_state
        # makes the bar follow the live arm conf.
        bar_rb = (mv.start_state.rigid_body_states.get(self.active_bar_name)
                  if mv.start_state is not None and self.active_bar_name else None)
        # Bar-accuracy test: the bar is mounted for the whole session and never
        # dismounted, so inject the 'held' attachment into movements whose
        # authored start_state has it released/installed (e.g. M3 retreat)
        # instead of refusing to plan.
        if (self.BAR_ACTION_MOCAP_ACCURACY_TEST and bar_rb is not None
                and bar_rb.attached_to_link is None):
            self._ensure_bar_attached_for_mocap(mv)
            bar_rb = mv.start_state.rigid_body_states.get(self.active_bar_name)
        if bar_rb is None or bar_rb.attached_to_link is None:
            self.get_logger().warn(
                f"Replan Transfer: bar {self.active_bar_name!r} is not "
                f"attached in the movement's start_state; use the free "
                f"Transit button (2) instead."
            )
            return

        # ---- MOCK LIVE POSE (same temporary hook as Button 2; see
        # MOCK_LIVE_POSE_FOR_REPLAN class flag) ------------------------------
        revert_mock = None
        if self.MOCK_LIVE_POSE_FOR_REPLAN:
            revert_mock = self._apply_mock_live_pose_for_replan(mv)
        # --------------------------------------------------------------------

        try:
            # Pause GUI rendering across the whole IK + constrained search
            # (no-op when headless) — same reasoning as Button 2.
            with pp.LockRenderer():
                # Step 1: live-base IK sets goal_arm_pose to the IK-solved 12-vec.
                if not self.ik_live_base_for_selected_movement():
                    return

                # Step 2: live base into mv.start_state so the transfer plan
                # uses the live-base state as its template.
                if not self._apply_live_base_to_movement(mv):
                    return

                # Step 3: constrained transfer plan, live conf -> IK goal conf.
                # Passing goal_conf (not goal_ee_frames) pins the goal bar
                # pose to FK at the IK-solved conf, skipping the planner's
                # own goal IK.
                state = mv.start_state.copy()
                self._inject_live_conf_into_state(state)
                goal_conf = conf_from_12vec(np.concatenate(
                    [self.goal_arm_pose[0], self.goal_arm_pose[1]]))
                path, info = plan_constrained_dual_arm(
                    self.cfab.planner, state,
                    active_bar_id=self.active_bar_name,
                    goal_conf=goal_conf,
                    stage=M1_PLANNER_STAGE,
                    position_res=CDFM_POSITION_RES,
                    rotation_res=CDFM_ROTATION_RES,
                    max_time=120.0,
                )
                if path is None:
                    self.get_logger().warn(
                        f"[transfer plan] constrained plan failed: "
                        f"{info.get('failure_reason', 'unknown')}."
                    )
                    return
                print(f"[transfer plan] OK ({len(path)} waypoints, bar-held).")

                # Fill the same trajectory slots Button 2 fills, so 'Exec
                # Both Arm Trajs' and the servoing loop pick the path up
                # unchanged.
                t = self.trajectory_time
                self.set_arm_trajectory(
                    (np.array([q[:6] for q in path]), None, t, None), index=0)
                self.set_arm_trajectory(
                    (np.array([q[6:] for q in path]), None, t, None), index=1)
                # This is a BAR_HELD transfer: mount the bar + its installed
                # joints on the preview from the planned (force-attached) state,
                # even for a movement whose authored start_state released it.
                self._refresh_preview_attached_bodies('bar_held', state)
                self.set_to_show_traj_state()

                # Step 4: check the planned endpoint against BOTH the IK goal
                # conf and the authored targets, per arm. NOTE the constrained
                # planner's endpoint may sit on a different joint branch than
                # the IK goal conf (it tracks poses, not joints) — the tool0
                # POSE residual is what matters, and that is what servoing
                # cares about. This also reports whether the physically held
                # bar grasp matches the authored one, which is what decides
                # how much right-arm error the servo loop can ever remove.
                self._verify_transfer_endpoint(
                    path, state,
                    np.concatenate([self.goal_arm_pose[0], self.goal_arm_pose[1]]))

                # Step 5: pre-execution preview + safeguard. The planned joint
                # values always (that is the preview itself, cheap); the joint
                # continuity + bar-hold EE drift curves when asked, so the
                # operator can confirm before running it (the servoing loop
                # suppresses those on its small later-iteration corrections).
                # `state` is the live-base template used for the plan, so its
                # FK matches.
                self.show_planned_joint_values(path, label=mv.movement_id)
                if show_validation:
                    self.show_transfer_validation(
                        path, state, label=mv.movement_id)
        finally:
            if revert_mock is not None:
                revert_mock()

    def _verify_replan_endpoint_matches_target(self,
                                                pos_tol_m: float = 0.005,
                                                ang_tol_deg: float = 1.0) -> None:
        """After a successful Button 2 composite plan, compare the last
        waypoint's tool0 world-frame poses (FK at live base + planned end
        arm conf) against the authored target EE frames the IK solved for.

        The two should agree to within a millimetre: the goal IK is the only
        thing that sets the composite plan's goal conf, and it only returns a
        conf whose tool0s land on those authored frames. A residual beyond the
        tolerances therefore means the composite plan did NOT reach its goal
        (a truncated or mis-unwrapped path), so a warning is emitted -- the
        tool0 targets are not met and any downstream linear motion that
        assumes them needs re-planning.

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
                f"The composite plan did not land on the IK-solved goal conf, "
                f"so the tool0s miss the authored EE targets. Any downstream "
                f"linear motion that assumes those targets should be "
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

    # Both execution paths silently refuse to move when the live arms are not
    # already at the trajectory's first waypoint: the joint path is rejected by
    # HuskyRobotInterface.to_trajectory_msg (0.1 rad), and the compliant path by
    # send_arm_cmd_cartesian's 5 cm target guard. Check it up front so the
    # operator gets one clear message instead of a silent no-op.
    EXEC_START_CONF_TOLERANCE_RAD = 0.1

    def _arms_at_trajectory_start(self):
        """Per-arm max |live joint - planned first waypoint|, in radians.

        Returns:
            list[float] | None: One value per arm, or None when there is no
            planned trajectory to compare against.
        """
        hi = self.huskies[self.selected_robot_id].interface
        deviations = []
        for i in range(min(2, len(hi.arm_joint_pose))):
            planned = self.planned_arm_trajectory[i][0]
            if planned is None or len(planned) == 0:
                return None
            live = np.asarray(hi.arm_joint_pose[i], dtype=float)
            deviations.append(
                float(np.max(np.abs(live - np.asarray(planned[0], dtype=float)))))
        return deviations

    def exec_selected_movement_traj(self):
        """Execute the currently loaded movement's trajectory, routed by its kind:

        - the insert / the retreat (``COMPLIANT_KINDS``) -> cartesian compliance
          controller via ``world.execute_planned_trajectory_compliant`` (a
          generator queued on ``self.tasks`` so the monitor tick pumps it; it
          tightens the joint screws / loosens the grippers).
        - a support robot's single-arm free / linear movement -> joint tracking
          via ``world.execute_arm_trajectory_all`` (arm 0).
        - the travel to the loading pose (DUAL_FREE, not the free move home) ->
          joint tracking, then a force/torque re-zero once the arms settle, via
          ``world.execute_trajectory_and_zero_ft`` (also a queued generator).
          It is the last movement with empty tools, so it is the only safe
          place to tare -- see that function.
        - everything else (the transfer, the free move home) -> joint tracking
          via ``world.execute_arm_trajectory_both``.
        When the export asks for another controller, that is warned once per
        movement (``_warn_controller_mismatch``). A movement class the monitor
        does not know is refused.

        Refuses when the live arms are not already parked at the trajectory's
        first waypoint -- press 'Move Arms to Movement Start' first. Also
        refuses a gripper / scaffolding-tool / manual step: those are run by
        ``exec_selected_movement_step``. In schedule mode also refused while a
        queued step / execution still runs (see ``_refuse_while_tasks_run``).
        """
        if self._refuse_while_tasks_run('Exec Selected Mv Traj', schedule_only=True):
            return
        if self._refuse_display_only_entry('Exec Selected Mv Traj'):
            return
        if self.current_movement is None:
            self.get_logger().warn(
                "No movement loaded; click 'Load Movement' first."
            )
            return
        mv = self.current_movement
        kind = self._kind_of(mv)
        if kind is None:
            self.get_logger().warn(
                f"{mv.movement_id!r}: unknown movement class; can not tell how to run it.")
            return
        # ! No arm moves in a tool / manual step, so the only trajectory there
        # ! is one left over from the movement before it -- running it would
        # ! replay that motion.
        if kind in STATIONARY_KINDS:
            self.get_logger().warn(
                f"{self.current_movement.movement_id!r} is a "
                f"{step_kind(self.current_movement)} step, not an arm movement: "
                f"nothing to execute here. Run it with the step exec "
                f"(exec_selected_movement_step) instead.")
            return
        if any(self.planned_arm_trajectory[i][0] is None
               for i in range(self._connected_robot().n_arms)):
            self.get_logger().warn(
                "No planned trajectory for this movement; click 'Load Movement "
                "Trajectory' (or 'Plan Movement') first."
            )
            return

        # Re-read the traj time slider's LIVE position rather than trusting the
        # cached self.trajectory_time. 'Load Movement' rebuilds this widget (via
        # reset_ui) seeded with the movement's default, and a freshly-rebuilt widget
        # can miss its next drag callback -- so an operator who slowed the move
        # down before pressing execute would otherwise be ignored, and the arms
        # would run at the default speed. Same hazard as the BarAction sliders.
        sld = getattr(self, 'trajectory_time_slider', None)
        if sld is not None:
            v = sld.value
            if v is not None and float(v) != self.trajectory_time:
                self.trajectory_time = float(v)
                print(f"[Exec] traj time from slider: {self.trajectory_time:.0f}s")

        # Same live re-read for M2's rigid->compliant handover distance: it
        # decides where position control stops holding the bar on target, so a
        # missed drag callback would silently run the operator's last-but-one
        # setting.
        sld = getattr(self, 'm2_split_slider', None)
        if sld is not None:
            v = sld.value
            if v is not None and float(v) != self.m2_compliant_split_mm:
                self.m2_compliant_split_mm = float(v)
                print(f"[Exec] insert split from slider: "
                      f"{self.m2_compliant_split_mm:.1f} mm to goal")

        # Rigid-only is a whole different execution mode, so read it live too.
        sld = getattr(self, 'm2_rigid_only_slider', None)
        if sld is not None:
            v = sld.value
            if v is not None and bool(round(float(v))) != self.m2_exec_rigid_only:
                self.m2_exec_rigid_only = bool(round(float(v)))
                print(f"[Exec] insert mode from slider: "
                      f"{'RIGID ONLY' if self.m2_exec_rigid_only else 'rigid+compliant split'}")

        deviations = self._arms_at_trajectory_start()
        if deviations and not self.FAKE_HARDWARE:
            if max(deviations) > self.EXEC_START_CONF_TOLERANCE_RAD:
                per_arm = ' / '.join(f'{side}={d:.3f}' for side, d
                                     in zip(self._connected_robot().side_keys, deviations))
                self.get_logger().warn(
                    f"Arms are not at the start of "
                    f"{self.current_movement.movement_id!r}: max joint offset "
                    f"{per_arm} rad "
                    f"(tolerance {self.EXEC_START_CONF_TOLERANCE_RAD:.2f}). "
                    f"Press 'Move Arms to Movement Start (offline target)' "
                    f"first -- executing now would be silently rejected by the "
                    f"controller."
                )
                return

        # Only the insert and the retreat run compliant; everything else (also a
        # single arm, where only joint tracking is wired) runs joint tracking.
        # Say so (once per movement) when the export asks for another controller.
        runs = (CONTROLLER_CARTESIAN_COMPLIANT if kind in COMPLIANT_KINDS
                else CONTROLLER_JOINT_TRACKING)
        self._warn_controller_mismatch(mv, runs)
        # ! Always call through the world module (world.execute_*): the dispatch
        # ! check (headless_schedule_smoke) replaces these module attributes.
        if kind in COMPLIANT_KINDS:
            self.tasks.append(world.execute_planned_trajectory_compliant(self))
        elif kind in SINGLE_ARM_KINDS:
            world.execute_arm_trajectory_all(self)
        elif kind is MovementKind.DUAL_FREE and not self._is_free_home(mv):
            self.tasks.append(world.execute_trajectory_and_zero_ft(self))
        else:
            world.execute_arm_trajectory_both(self)

    def exec_selected_movement_step(self) -> None:
        """Run the loaded movement, whatever kind of step it is (the schedule's one exec button).

        Arm movements go through ``exec_selected_movement_traj``. The other steps
        wait (for the operator's 'Confirm Exec', the gripper or the tool), so they
        are queued on ``self.tasks`` for the monitor tick to pump:

        - gripper  -> ``world.exec_gripper_tool_movement`` (open, or close with the
          compliant handoff);
        - manual   -> ``world.run_manual_step`` (the operator confirms it is done);
        - scaffold -> ``world.run_scaffolding_tool_step`` (grasp / untighten, or
          mark-only for the steps the compliant insert / retreat already runs).

        Refused while ANY queued step / execution still runs or waits for
        'Confirm Exec' (one click on 'Confirm Exec' would answer both waits),
        while another robot's entry is shown, and for a movement class the
        monitor does not know.
        """
        if self._refuse_while_tasks_run('Exec step'):
            return
        if self._refuse_display_only_entry('Exec step'):
            return
        mv = self.current_movement
        if mv is None:
            self.get_logger().warn("No movement loaded; click 'Load Movement' first.")
            return
        try:
            kind = step_kind(mv)
        except TypeError as e:
            self.get_logger().warn(
                f"{mv.movement_id!r}: {e}. Not run as a step; an arm movement runs "
                f"with 'Exec Selected Mv Traj (auto)'.")
            return
        if kind == 'arm':
            self.exec_selected_movement_traj()
            return
        run_step = {
            'gripper': world.exec_gripper_tool_movement,
            'manual': world.run_manual_step,
            'scaffold': world.run_scaffolding_tool_step,
        }[kind]
        self.get_logger().info(f"Queued {kind} step {mv.movement_id!r}.")
        self.tasks.append(run_step(self, mv))

    def move_arms_to_movement_start(self) -> None:
        """'Move Arms to Movement Start' button: ``world.move_arms_to_movement_start``.

        Refused while another robot's schedule entry is loaded for display only,
        and in schedule mode while a queued step / execution still runs.
        """
        if self._refuse_while_tasks_run('Move Arms to Movement Start', schedule_only=True):
            return
        if self._refuse_display_only_entry('Move Arms to Movement Start'):
            return
        world.move_arms_to_movement_start(self)

    def _accept_trajectory(self, mv, jt, *, source: str = 'Plan') -> None:
        """Common post-step after a trajectory is either planned or loaded.

        Assigns mv.trajectory, propagates first/last conf to start states (the
        chain rules are keyed by the movement's kind), wires the visualizer,
        runs CDFM validation, and prints the movement roster. Persistence lives
        on the ``<action>.live-solved.json`` sidecar that
        ``plan_movement_chain_live`` writes -- no per-movement JSONs.
        A single-arm robot takes ``_accept_single_arm_trajectory`` instead.

        Args:
            mv: The Movement the trajectory belongs to.
            jt (JointTrajectory): The planned or loaded trajectory.
            source (str): Log tag, e.g. ``'Plan'`` or ``'LoadTraj'``.
        """
        if self._connected_robot().n_arms == 1:
            self._accept_single_arm_trajectory(mv, jt, source=source)
            return
        mv.trajectory = jt
        path = path_12_from_joint_trajectory(jt)
        kind = self._kind_of(mv)
        if path:
            start_vec = np.asarray(path[0], dtype=float)
            if kind in COMPLIANT_KINDS and mv.start_state is not None:
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
                # The transfer owns its generated start_conf; the free moves
                # keep the legacy behavior of mirroring trajectory start into
                # start_state.
                mv.start_state.robot_configuration = conf_from_12vec(start_vec)

            # Step (3) forward-chain propagation, by kind:
            #   transfer / insert / retreat: strict chain owners; ALWAYS
            #     overwrite next.start with traj[-1] (warn first if there's an
            #     existing value).
            #   DUAL_FREE: NOT part of the chain. The travel to load ends at
            #     the transfer's start, which the transfer owns (its own plan);
            #     the free move home ends the action. Neither writes the next
            #     list-index movement's start_state.robot_configuration.
            if kind is MovementKind.DUAL_FREE:
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
                                f"(transfer / insert / retreat chain rule)."
                            )
                            if kind is MovementKind.DUAL_CONSTRAINED_FREE:
                                self._drop_m2_m3_after_m1_chain_break(
                                    f"{source} {mv.movement_id} endpoint changed by "
                                    f"max {diff:.4f} rad/m"
                                )
                            elif (kind is MovementKind.DUAL_CONSTRAINED_LINEAR
                                  and self._kind_of(next_mv) is MovementKind.DUAL_INDEPENDENT_LINEAR):
                                self._drop_movement_trajectory(
                                    next_mv,
                                    f"{source} {mv.movement_id} endpoint changed by "
                                    f"max {diff:.4f} rad/m"
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
        # Mount the bar/joints on the preview per this movement's authored type
        # (bar held when its start state has the bar attached). Fresh planning
        # and 'Load Movement Trajectory' both land here.
        self._refresh_preview_attached_bodies(
            self._authored_motion_type(mv), mv.start_state)
        self.set_to_show_traj_state()
        tag = source
        print(f"[{tag}] {mv.movement_id!r}: {len(path)} waypoints stored.")

        # "Movement Preview" plots, so the operator can review the path before
        # pressing execute. Joint evolution for every movement; the bar-hold
        # safeguard only for the movements that actually carry the bar (the
        # transfer and the insert), since its EE-drift curve is meaningless when
        # nothing is held. The transfer's CDFM path additionally gets the sparse
        # stage validator below.
        # The joint-evolution plot is cheap (no FK) and is the preview itself, so
        # it always runs. The bar-hold + CDFM checks below re-derive poses along
        # the whole path; they are fast enough now to always run on every accepted
        # trajectory.
        self.show_planned_joint_values(path, label=mv.movement_id)
        # ! The rigid two-hand hold is a property of these two classes. Kept off
        # ! the motion type on purpose: in the bar-accuracy test the retreat's
        # ! start gets the bar attached (_ensure_bar_attached_for_mocap), and a
        # ! bar-hold drift check on the retreat would mean nothing.
        bar_hold_kinds = (MovementKind.DUAL_CONSTRAINED_FREE, MovementKind.DUAL_CONSTRAINED_LINEAR)
        if kind in bar_hold_kinds:
            self.show_transfer_validation(path, mv.start_state, label=mv.movement_id)
        self._validate_cdfm_planned_path(mv, path)

        # The travel to load's goal is wherever the transfer starts. Once the
        # transfer's trajectory is accepted (its start_state now carries a
        # planned robot_configuration), copy that configuration into the travel
        # to load's target_configuration so it can plan without re-loading the
        # BarAction.
        if kind is MovementKind.DUAL_CONSTRAINED_FREE:
            self._backfill_m0_target_from_m1()

        self._print_movement_roster(tag=tag)

    def _merge_arm_values(self, state, joint_names: list, values) -> None:
        """Write arm joint values into a state's robot_configuration, BY NAME.

        Args:
            state (RobotCellState): Edited in place. A None robot_configuration
                is first created as the cell's zero configuration.
            joint_names (list): The joints to set.
            values (Sequence[float]): One value per joint.
        """
        if state.robot_configuration is None:
            state.robot_configuration = self.cfab.robot_cell.zero_full_configuration()
        for name, value in zip(joint_names, values):
            state.robot_configuration[name] = float(value)

    def _accept_single_arm_trajectory(self, mv, jt, *, source: str = 'Plan') -> None:
        """Single-arm version of ``_accept_trajectory`` (support robots).

        Stores the trajectory, copies its first waypoint into this movement's
        start configuration and its last waypoint into the NEXT movement's, and
        wires the preview. The last waypoint is carried on through steps where
        no arm moves (gripper / manual) into the next arm movement, so e.g.
        ``H_M2`` starts where ``H_M0`` ended with ``H_M1`` (gripper open) in
        between. Cindy's M-role chain rules (M1 owns its start, M2/M3 start
        checks, ...) do not apply.

        Args:
            mv: The Movement the trajectory belongs to.
            jt (JointTrajectory): The planned or loaded trajectory.
            source (str): Log tag, e.g. ``'Plan'`` or ``'LoadTraj'``.
        """
        names = list(self._connected_robot().arm_joint_names[0])
        mv.trajectory = jt
        path = path_from_joint_trajectory(jt, names)
        if path:
            if mv.start_state is not None:
                self._merge_arm_values(mv.start_state, names, path[0])
            idx = self.current_movement_index
            for next_mv in self._loaded_movements[idx + 1:] if idx is not None else []:
                if next_mv.start_state is not None:
                    self._merge_arm_values(next_mv.start_state, names, path[-1])
                    print(f"[{source}] propagated {mv.movement_id!r}.traj[-1] -> "
                          f"{next_mv.movement_id!r}.start_state.robot_configuration.")
                if movement_kind(next_mv) not in STATIONARY_KINDS:
                    break  # an arm moves here: whatever follows starts from its end

        self.planned_arm_trajectory = [
            (np.asarray(path), None, self.trajectory_time, None),
            (None, None, None, None),
        ]
        self._refresh_preview_attached_bodies(
            self._authored_motion_type(mv), mv.start_state)
        self.set_to_show_traj_state()
        print(f"[{source}] {mv.movement_id!r}: {len(path)} waypoints stored.")
        self.show_planned_joint_values(path, label=mv.movement_id)
        self._print_movement_roster(tag=source)

    def _backfill_m0_target_from_m1(self) -> None:
        """Set the travel to load's target_configuration to the transfer's start configuration.

        The authored travel to load has no target of its own (the producer
        can't know the planned transfer start). Does nothing when one of the
        two is missing or the transfer's start configuration is still missing.
        """
        free_to_load = self._loaded_movement_of(MovementKind.DUAL_FREE, free_home=False)
        transfer = self._loaded_movement_of(MovementKind.DUAL_CONSTRAINED_FREE)
        if free_to_load is None or transfer is None:
            return
        if transfer.start_state is None or transfer.start_state.robot_configuration is None:
            return
        free_to_load.target_configuration = transfer.start_state.robot_configuration
        print(f"[backfill] {free_to_load.movement_id!r}.target_configuration <- "
              f"{transfer.movement_id!r}.start_state.robot_configuration.")

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
                f".live-solved.json sidecar via {self._load_hint()}, or run "
                "'Plan Chain (Live)'."
            )
            return
        print(f"[LoadTraj] using in-memory trajectory for {mv.movement_id!r}")
        self._accept_trajectory(mv, jt, source='LoadTraj')

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
        if self._kind_of(mv) is MovementKind.DUAL_CONSTRAINED_FREE:
            # The transfer's start_conf is generated by its planning; without its
            # traj it is stale by definition and must not survive as an authored start.
            if mv.start_state is not None:
                mv.start_state.robot_configuration = None
        if had_traj:
            print(f"[drop-traj] {mv.movement_id!r}: {reason}")

    def _drop_m2_m3_after_m1_chain_break(self, reason: str) -> int:
        """Drop the insert's and the retreat's trajectories after the transfer's endpoint changed.

        Args:
            reason (str): Why, for the log line of each dropped trajectory.

        Returns:
            int: How many trajectories were dropped.
        """
        dropped = 0
        for m in self._loaded_movements:
            if self._kind_of(m) in COMPLIANT_KINDS and self._trajectory_has_waypoints(m):
                self._drop_movement_trajectory(m, reason)
                dropped += 1
        return dropped

    def _clear_m1_start_conf_without_trajectory(self):
        """Keep invariant: the transfer has a start_conf only when it has a trajectory."""
        for m in self._loaded_movements:
            if self._kind_of(m) is not MovementKind.DUAL_CONSTRAINED_FREE:
                continue
            if m.start_state is None or self._trajectory_has_waypoints(m):
                continue
            if getattr(m.start_state, 'robot_configuration', None) is not None:
                m.start_state.robot_configuration = None
                print(f"[{m.movement_id}] cleared start_state.robot_configuration "
                      f"because it has no trajectory.")

    def _color_bool(self, value):
        """Return a terminal-colored bool string for planning status prints."""
        if bool(value):
            return "\033[32mTrue\033[0m"
        return "\033[31mFalse\033[0m"

    def _bar_action_write_path(self, path: str = None) -> str:
        """Decide which file the loaded BarAction may be written back to.

        ! A CLEAN Rhino export -- a basename carrying no dotted tag, such as
        ! `B45.json` -- is never overwritten. That write is diverted to
        ! `<stem>.live-solved.json`, preserving the invariant that the clean
        ! file is only ever produced by the offline exporter. A file that is
        ! already tagged (a previous `.live-solved.json`, say) is written in
        ! place, so repeated saves do not pile up `.live-solved.live-solved`.

        Args:
            path (str): Which of the loaded cycle's files to write back.
                Defaults to the first half, which is the whole action for a
                legacy single-file export.

        Returns:
            str: Absolute path to write the action to.
        """
        path = path or self._current_action_path
        basename = os.path.basename(path)
        if basename.count('.') > 1:
            return path
        stem, ext = os.path.splitext(path)
        diverted = f"{stem}.live-solved{ext}"
        self.get_logger().warn(
            f"{basename} is a CLEAN export and will not be overwritten; "
            f"writing to {os.path.basename(diverted)} instead.")
        return diverted

    def _write_action_halves_for(self, movements, tag: str):
        """Save the cycle halves that own ``movements``, each to its own file.

        Only the half a changed movement lives in is written. Step A adopts M0
        and M1, and the split export keeps both in the jointing file, so the
        release half has nothing new and gets no sidecar of its own. Reloading
        still gives the whole cycle: ``load_action_cycle`` looks for the release
        sidecar, does not find one, and falls back to the clean release export.

        A legacy single-file action has one half and writes one file, as before.
        The clean export itself is never overwritten -- see
        ``_bar_action_write_path``.

        Args:
            movements (list): The movements whose files must be saved.
            tag (str): Caller name, used in the log lines.

        Returns:
            list[str] | None: The paths written, or None if a write failed.
        """
        written = []
        for action, path in self._loaded_action_slots:
            owns = any(any(mv is m for m in action.movements) for mv in movements)
            if not owns:
                continue
            out_path = self._bar_action_write_path(path)
            try:
                json_dump(action, out_path)
            except Exception as e:
                self.get_logger().error(
                    f"[{tag}] failed to write {out_path}: {e}")
                return None
            written.append(out_path)
        return written

    def _save_m1_m0_confs_to_bar_action_file(self):
        """Persist the just-adopted M0/M1 configurations into the BarAction JSON.

        Writes the whole action, so the three configurations the mount-once
        protocol derives outlive the session:

        - `M1.start_state.robot_configuration` -- the derived start
        - `M0.target_configuration`            -- the same conf, M0's goal
        - `M1.target_configuration`            -- the derived goal

        Reloading that file next session lets you go straight to
        'Load Movement' -> 'Plan Movement' on M0, with no second run of
        'M1: Derive Start/Goal only'.

        ! Valid only while the base has NOT moved. All three confs were
        ! derived against the live base pose, so after driving the Husky
        ! somewhere else they describe a goal that no longer lines up with the
        ! bar -- derive again instead of reloading them.

        ? Writing `M1.target_configuration` is a record for whoever reads the
        ? file; the planner still takes M1's goal from the authored M2 start
        ? conf (see _m1_goal_conf), so it does not change how M1 plans.

        No trajectory is written: adopting deliberately leaves M1 without one
        (see adopt_m1_derived_start).

        Returns:
            str | None: The path written, or None when nothing was written.
        """
        if not self._loaded_action or not self._current_action_path:
            self.get_logger().warn(
                f"No BarAction loaded; click {self._load_hint()} first.")
            return None

        # The travel to load and the transfer are what adopting changed, so only
        # their half is saved.
        free_to_load = self._loaded_movement_of(MovementKind.DUAL_FREE, free_home=False)
        transfer = self._loaded_movement_of(MovementKind.DUAL_CONSTRAINED_FREE)
        adopted = [m for m in (free_to_load, transfer) if m is not None]
        written = self._write_action_halves_for(adopted, 'Adopt')
        if not written:
            return None
        out_path = written[0]

        names = ', '.join(m.movement_id for m in adopted)
        self.get_logger().info(
            f"Saved the adopted configurations of {names} -> {', '.join(written)}. As "
            f"long as the base stays put, reload this file next session and plan "
            f"the travel to load directly.")
        # A diverted write creates a file that was not in the list; refresh it
        # so the new file can be selected without restarting.
        if out_path != self._current_action_path:
            self.available_bar_actions = self._load_available_bar_actions()
            self.reset_ui(self.goal_arm_pose)
        return out_path

    def export_m0_plan_to_bar_action_file(self):
        """Write the live-planned M0 trajectory back into the BarAction JSON.

        M0 is the one movement the offline planner leaves unsolved -- its
        trajectory depends on wherever the robot happens to be, so it ships as
        ``null`` and is planned live. That means every session re-plans it.
        This saves the one you just planned so the next session can simply load
        it, like M1..M4.

        Writes the WHOLE action (``self._loaded_action``, which shares object
        identity with ``self._loaded_movements``), so any other trajectory
        currently in memory -- a replanned M4, say -- is preserved alongside.
        Trajectories serialize as compas_fab ``JointTrajectory``, matching what
        ``headless_bar_action_planner`` now writes.

        ! Never overwrites a CLEAN Rhino export. If the loaded file is one
        ! (``B45.json`` -- a basename with no dotted tag), the action is written
        ! to ``<stem>.live-solved.json`` instead, preserving the invariant that
        ! the clean file is only ever produced by the exporter.

        Returns:
            str | None: The path written, or None if nothing was written.
        """
        if not self._loaded_action or not self._current_action_path:
            self.get_logger().warn(
                f"No BarAction loaded; click {self._load_hint()} first.")
            return None
        m0 = self._loaded_movement_of(MovementKind.DUAL_FREE, free_home=False)
        if m0 is None:
            self.get_logger().warn("This BarAction has no travel-to-load movement to export.")
            return None
        path12 = path_12_from_joint_trajectory(getattr(m0, 'trajectory', None))
        if not path12:
            self.get_logger().warn(
                f"{m0.movement_id!r} has no planned trajectory to export. "
                f"Select {m0.movement_id} and click 'Plan Movement' first.")
            return None

        # Only its own half is saved; the release half is untouched by this.
        written = self._write_action_halves_for([m0], 'Export travel-to-load')
        if not written:
            return None
        out_path = written[0]

        solved = [m.movement_id for m in self._loaded_movements
                  if getattr(m, 'trajectory', None) is not None]
        self.get_logger().info(
            f"Exported travel-to-load {m0.movement_id!r} ({len(path12)} waypoints) -> {out_path}. "
            f"The file now carries trajectories for: {solved}.")
        # A diverted write creates a new file; refresh the slider list so it can
        # be selected without restarting.
        if out_path != self._current_action_path:
            self.available_bar_actions = self._load_available_bar_actions()
            self.reset_ui(self.goal_arm_pose)
        return out_path

    def _validate_free_planned_path(self, mv, path12):
        """Re-check a free (M0/M4) path densely BETWEEN its waypoints.

        ``plan_free_dual_arm`` only tests the configurations its extend function
        lands on, and does no swept checking -- so a path whose every waypoint is
        collision-free can still sweep an arm straight through a bar on the way.
        This walks each segment at ``FM_VALIDATION_STEP_RAD`` and asks the cfab
        planner (the same authority the planner itself used, so the ACM and the
        wheels-on-ground allowance are respected) about every intermediate
        sample.

        On a hit it also runs a raw pybullet pass over the spawned rigid bodies
        purely to NAME what was struck, so the report is "segment 47->48 hits
        bar_B21" rather than an opaque failure.

        Args:
            mv: The movement whose path this is (for log lines).
            path12 (Sequence): Planned waypoints, each a 12-vec.

        Skipped entirely when ``self.fm_swept_validation_enabled`` is False (the
        "swept collision check" slider). It then reports OK so the callers' gates
        pass -- but says so loudly, because a silently-skipped safety check is
        worse than no check at all.

        Args:
            mv: The movement whose path this is (for log lines).
            path12 (Sequence): Planned waypoints, each a 12-vec.

        Returns:
            dict: ``{'ok': bool, 'samples': int, 'bad_segments': list, 'bodies':
            list, 'skipped': bool}``. ``ok`` is True when nothing was hit.
            ``bad_segments`` holds ``(segment_index, sorted_body_names)`` tuples.
        """
        verdict = {'ok': True, 'samples': 0, 'bad_segments': [], 'bodies': [],
                   'skipped': False}
        if not getattr(self, 'fm_swept_validation_enabled', True):
            verdict['skipped'] = True
            self.get_logger().warn(
                f"[free validation] {getattr(mv, 'movement_id', '?')!r}: SWEPT "
                f"COLLISION CHECK DISABLED -- the path is accepted unverified. "
                f"Its waypoints are collision-free but the motion BETWEEN them "
                f"is not checked, so it may sweep through the structure.")
            return verdict
        path12 = [np.asarray(q, dtype=float) for q in (path12 or [])]
        if len(path12) < 2 or self.cfab is None:
            return verdict

        planner = self.cfab.planner
        names_12 = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]) + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
        template = mv.start_state
        # Raw-pybullet naming needs the cfab client's bodies; the ground is
        # excluded because the wheels rest on it by construction (the ACM allows
        # it, so compas_fab is right to stay quiet about it).
        rb_puids = getattr(self.cfab.client, 'rigid_bodies_puids', {}) or {}
        body_names = {ids[0]: n for n, ids in rb_puids.items()
                      if ids and n != GROUND_RIGID_BODY_NAME}
        robot_puid = self.cfab.client.robot_puid

        def _state_at(conf):
            s = template.copy()
            for n, v in zip(names_12, conf):
                s.robot_configuration[n] = float(v)
            return s

        def _names_hit():
            """Bodies the robot penetrates in the CURRENT scene (already posed)."""
            hit = set()
            for puid, name in body_names.items():
                try:
                    if p.getClosestPoints(robot_puid, puid, distance=0.0,
                                          physicsClientId=self.cfab.client.client_id):
                        hit.add(name)
                except Exception:
                    pass
            return hit

        saved_client = pp.CLIENT
        pp.CLIENT = self.cfab.client.client_id
        pp.CLIENTS.setdefault(pp.CLIENT, True)
        all_bodies = set()
        try:
            for i in range(len(path12) - 1):
                a, b = path12[i], path12[i + 1]
                n_steps = max(1, int(np.ceil(
                    float(np.abs(b - a).max()) / FM_VALIDATION_STEP_RAD)))
                seg_bodies = set()
                seg_bad = False
                # Endpoints are already known-good from planning; only the
                # interior of each segment is new information here.
                for k in range(1, n_steps):
                    q = a + (b - a) * (k / n_steps)
                    verdict['samples'] += 1
                    state = _state_at(q)
                    try:
                        planner.check_collision(state, options={"verbose": False})
                        continue
                    except CollisionCheckError:
                        pass
                    seg_bad = True
                    # Colliding: re-apply so the scene matches this sample, then
                    # name the bodies. A hit that names nothing is still a hit --
                    # it just means the pair was robot-self or robot-vs-tool,
                    # neither of which is a rigid body we can point at.
                    try:
                        planner.set_robot_cell_state(state)
                        seg_bodies |= _names_hit()
                    except Exception:
                        pass
                if seg_bad and not seg_bodies:
                    seg_bodies.add('self/tool (no rigid body named)')
                if seg_bad:
                    verdict['bad_segments'].append((i, sorted(seg_bodies)))
                    all_bodies |= seg_bodies
        except Exception as exc:
            # ! FAIL CLOSED. This used to return the freshly-initialised verdict,
            # ! whose 'ok' is True -- so any exception in here silently turned
            # ! "could not verify" into "verified fine" and the caller accepted
            # ! an unchecked path. An unverifiable path must be treated exactly
            # ! like a failed one; the operator can still force it through with
            # ! the swept-check toggle if they know better.
            self.get_logger().error(
                f"[free validation] {mv.movement_id!r}: check RAISED ({exc!r}); "
                f"treating the path as unsafe. Full traceback:")
            traceback.print_exc()
            verdict['ok'] = False
            verdict['error'] = repr(exc)
            return verdict
        finally:
            pp.CLIENT = saved_client

        verdict['bodies'] = sorted(all_bodies)
        verdict['ok'] = not verdict['bad_segments']
        if verdict['ok']:
            self.get_logger().info(
                f"[free validation] {mv.movement_id!r}: clear -- "
                f"{verdict['samples']} interpolated samples at "
                f"{FM_VALIDATION_STEP_RAD} rad over {len(path12) - 1} segments.")
        else:
            self.get_logger().error(
                f"[free validation] {mv.movement_id!r}: SWEPT COLLISION on "
                f"{len(verdict['bad_segments'])}/{len(path12) - 1} segments; "
                f"bodies hit: {verdict['bodies']}")
            for seg, bodies in verdict['bad_segments'][:10]:
                a, b = path12[seg], path12[seg + 1]
                print(f"   segment {seg}->{seg + 1} "
                      f"(max joint step {np.degrees(np.abs(b - a).max()):.2f} deg) "
                      f"hits {bodies}")
            if len(verdict['bad_segments']) > 10:
                print(f"   ... and {len(verdict['bad_segments']) - 10} more segments")
        return verdict

    def _plan_free_and_validate(self, mv, tag: str, goal_conf, **plan_kwargs):
        """Plan a free (M0/M4) movement, then gate it on the dense re-check.

        Returns the JointTrajectory only if the path survives
        ``_validate_free_planned_path``; a path that sweeps through the built
        assembly is REJECTED rather than handed on with a warning, because the
        preview looks perfectly fine in that case (every waypoint is clear) and
        the operator has no other cue before pressing execute.

        Args:
            mv: The movement being planned.
            tag (str): Log tag (the movement id).
            goal_conf: Goal configuration passed to ``plan_free_dual_arm``.
            **plan_kwargs: Extra arguments for ``plan_free_dual_arm``.

        Returns:
            JointTrajectory | None: None when planning or validation failed.
        """
        with pp.LockRenderer():
            path, info = plan_free_dual_arm(
                self.cfab.planner, mv.start_state, goal_conf,
                joint_resolution=FM_JOINT_RESOLUTION, **plan_kwargs)
        if path is None:
            reason = info.get('failure_reason')
            print(f"[{tag}] plan_free_dual_arm failed: {reason}")
            if reason == 'start_or_goal_in_collision':
                # The planner only says "start or goal"; name the pairs and
                # draw them, and check joint limits while we are at it.
                self._diagnose_free_plan_endpoints(mv, goal_conf, tag)
            return None
        print(f"[{tag}] planned {len(path)} waypoints at "
              f"{FM_JOINT_RESOLUTION} rad; verifying swept path...")
        if not self._validate_free_planned_path(mv, path)['ok']:
            self.get_logger().error(
                f"[{tag}] plan REJECTED: it sweeps through the scene between "
                f"waypoints. Re-run 'Plan Movement' for a different RRT sample.")
            return None
        return joint_trajectory_from_path(path)

    def _diagnose_free_plan_endpoints(self, mv, goal_conf, tag: str):
        """Explain a free plan's ``start_or_goal_in_collision`` failure.

        ``plan_free_dual_arm`` rejects the request when either endpoint fails
        the cfab collision check, but pybullet_planning only prints
        "initial/end configuration is in collision" -- no pairs, no depths.
        This re-checks BOTH endpoints with the same collision setup (ACM,
        attached tools/bar) through cc_diagnosis: every colliding pair is
        printed deepest-first with its witness points, and the first colliding
        endpoint's pairs are drawn in the PyBullet window (the drawing is
        cleared on the next diagnosis, or by hand).

        Joint limits are reported separately: cfab's collision check ignores
        them, so a limit violation is never the cause of "in collision", but a
        goal outside the URDF limits cannot be reached by the BiRRT (its
        sampler stays inside them) and the two failures look alike from the
        planner's one-line verdict. A value that is a 2*pi wrap of an in-range
        one is flagged as such (the usual way an IK-derived conf ends up out
        of range).

        A diagnostic must never turn a soft planning failure into a crash, so
        the body is guarded and any error is reported and swallowed.

        Args:
            mv: The movement whose ``start_state`` (already resynced to the
                live arms + live base) is the plan's start.
            goal_conf: The plan's goal, a compas Configuration or a 12-vec.
            tag (str): Log tag (the movement id).
        """
        planner = self.cfab.planner if self.cfab is not None else None
        if planner is None or mv is None or mv.start_state is None:
            return
        names_12 = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]) + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
        try:
            goal12 = (np.asarray(goal_conf, dtype=float)
                      if isinstance(goal_conf, (list, tuple, np.ndarray))
                      else vec12_from_conf(goal_conf))
            endpoints = [
                ('START (live arms at the live base)', mv.start_state),
                ('GOAL', _state_with_conf12(mv.start_state, goal12, names_12)),
            ]

            # --- Collisions: who against who, per endpoint.
            clear_collision_diagnosis(self)
            first_hit = None  # (label, records, state) of the first colliding endpoint
            for label, state in endpoints:
                records = collect_collision_contacts(planner, state)
                print_collision_contacts(
                    records, header=f"[{tag} diag] {label}:")
                if records and first_hit is None:
                    first_hit = (label, records, state)
            if first_hit is not None and pp.has_gui():
                label, records, state = first_hit
                # collect() left the scene at the LAST endpoint checked; put the
                # colliding one back so the drawing lines up with the bodies.
                planner.set_robot_cell_state(state)
                draw_collision_contacts(self, records)
                print(f"[{tag} diag] drawn: the {label} pairs (deepest first).")

            # --- Joint limits vs the URDF, per endpoint (pp reads limits from
            # the cfab robot, so point pp.CLIENT at that client for the query).
            saved_client = pp.CLIENT
            pp.CLIENT = planner.client.client_id
            pp.CLIENTS.setdefault(pp.CLIENT, True)
            try:
                robot = planner.client.robot_puid
                joints = pp.joints_from_names(robot, names_12)
                for label, state in endpoints:
                    vec = vec12_from_conf(state.robot_configuration)
                    bad = []
                    for name, j, v in zip(names_12, joints, vec):
                        if not pp.violates_limit(robot, j, v):
                            continue
                        lo, hi = pp.get_joint_limits(robot, j)
                        wrapped = any(lo <= v + k * 2 * np.pi <= hi for k in (-1, 1))
                        bad.append(f"{name}={v:+.3f} rad (limits [{lo:+.3f}, {hi:+.3f}])"
                                   + (" -- a 2*pi wrap of an in-range value" if wrapped else ""))
                    if bad:
                        print(f"[{tag} diag] {label}: {len(bad)} joint(s) OUTSIDE URDF limits:")
                        for line in bad:
                            print(f"    {line}")
                    else:
                        print(f"[{tag} diag] {label}: all 12 joints within URDF limits.")
            finally:
                pp.CLIENT = saved_client
            # Leave the scene at the plan's start, as the planner found it.
            planner.set_robot_cell_state(mv.start_state)
        except Exception as e:
            print(f"[{tag} diag] ERROR while diagnosing the endpoints: {e}")

    def _validate_cdfm_planned_path(self, mv, path12):
        """Run sparse path_validation checks for a planned transfer (CDFM) path.

        Only the transfer (DUAL_CONSTRAINED_FREE) is checked; any other movement
        returns right away.

        Args:
            mv: The movement the path belongs to.
            path12 (Sequence): Planned waypoints, each a 12-vec.
        """
        movement_id = getattr(mv, 'movement_id', '') or ''
        if self._kind_of(mv) is not MovementKind.DUAL_CONSTRAINED_FREE:
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
                position_res=CDFM_POSITION_RES,
                rotation_res=CDFM_ROTATION_RES,
                dense_joint_validation_step_rad=0.0,
                skip_dense_collision_checks=True,
                # Keep the authoritative pass/fail logging, but never pop the
                # matplotlib window: the native "Movement Preview" DPG plots
                # (drawn by _accept_trajectory) replace it. The pyplot-interactive popup spins
                # its own GUI loop and can crash the live DPG + PyBullet monitor.
                save_plot=False,
                show_plot=False,
            )
        except Exception as exc:
            self.get_logger().warn(f"[CDFM validation] failed for {movement_id!r}: {exc}")
            return
        finally:
            pp.CLIENT = saved_client

        # (The native DPG safeguard curves for this path are drawn by the
        # caller, _accept_trajectory, which does it for every bar-held movement
        # rather than only the CDFM ones this validator handles.)

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
            transfer = self._loaded_movement_of(MovementKind.DUAL_CONSTRAINED_FREE)
            transfer_id = transfer.movement_id if transfer is not None else 'none loaded'
            self.get_logger().warn(
                f"{mv.movement_id} has no goal yet: confirm or plan the transfer "
                f"start first ({transfer_id}); that start becomes this movement's goal.")
            return None
        self._hide_unmounted_active_bar(mv)
        self._resync_start_state_to_live(mv, mv.movement_id)
        return self._plan_free_and_validate(
            mv, mv.movement_id, mv.target_configuration,
            max_time=120.0, max_iterations=50,
        )

    # How far the cfab planning robot may sit from the base frame it was told to
    # stand at before we call it a mismatch. Pure bookkeeping error, so tight.
    CFAB_BASE_MISMATCH_TOL_M = 1e-3

    def _check_cfab_base_matches(self, mv, tag):
        """Warn if the cfab planning robot is not where the state says it is.

        Everything the planner decides -- reachability, and above all which
        configurations are collision-free -- is computed with the cfab robot at
        the pose ``set_robot_cell_state`` put it. If that pose has drifted from
        the movement's authored ``robot_base_frame``, the plan is checked against
        the structure as seen from the WRONG place: it can look clean to the
        planner and drive straight through a bar in reality.

        This is a symptom check, not a fix. It exists because that failure is
        otherwise invisible until you notice the translucent red planning robot
        standing somewhere the real husky is not.

        Args:
            mv: The movement whose ``start_state`` was just pushed.
            tag (str): Short label for the log line (call site).

        Returns:
            bool: True when they agree (or nothing could be compared).
        """
        state = getattr(mv, 'start_state', None)
        if self.cfab is None or state is None or state.robot_base_frame is None:
            return True
        robot_puid = getattr(self.cfab.client, 'robot_puid', None)
        if robot_puid is None:
            return True
        try:
            authored = np.asarray(pose_from_frame(state.robot_base_frame)[0], dtype=float)
            actual = np.asarray(p.getBasePositionAndOrientation(
                robot_puid, physicsClientId=self.cfab.client.client_id)[0], dtype=float)
        except Exception as e:
            print(f"[{tag}] could not read the cfab robot base: {e}")
            return True
        err = float(np.linalg.norm(authored - actual))

        # Three robots share this scene and are easy to confuse by eye: the
        # translucent RED cfab planning robot, the solid live husky, and the
        # green goal ghost. Report all three against the authored base so one
        # line says WHICH is displaced rather than leaving it to the screenshot.
        others = {}
        try:
            if self.huskies:
                ho = self.huskies[self.selected_robot_id].object
                others['live husky'] = np.asarray(
                    p.getBasePositionAndOrientation(ho.robot)[0], dtype=float)
        except Exception:
            pass
        try:
            others['goal ghost'] = np.asarray(
                p.getBasePositionAndOrientation(self.goal_model.robot)[0], dtype=float)
        except Exception:
            pass
        others_txt = '; '.join(
            f"{k} {np.round(v, 3)} (err {np.linalg.norm(v - authored):.3f} m)"
            for k, v in others.items())

        if err <= self.CFAB_BASE_MISMATCH_TOL_M:
            # cfab is right. If a robot still looks out of place on screen it is
            # one of the others, so say where they are -- silently returning
            # True here is what makes that case impossible to diagnose.
            bad_others = {k: v for k, v in others.items()
                          if np.linalg.norm(v - authored) > self.CFAB_BASE_MISMATCH_TOL_M}
            if bad_others:
                self.get_logger().warn(
                    f"[{tag}] cfab planning robot is correctly placed at "
                    f"{np.round(authored, 3)}, but: {others_txt}")
            return True
        self.get_logger().error(
            f"[{tag}] cfab PLANNING ROBOT IS MISPLACED by {err:.3f} m: the "
            f"movement's base frame is {np.round(authored, 3)} but the robot "
            f"stands at {np.round(actual, 3)}. Collision checks are being run "
            f"from the wrong place, so any plan made now is unsafe. "
            f"Other robots: {others_txt or 'n/a'}")
        return False

    def replan_linear_to_target_from_live(self, mv, tag: str):
        """Replan a straight cartesian run from the arms' ACTUAL pose to the goal.

        Used for M3's second chunk. Compliance leaves the tools sideways off the
        planned line (gravity sag), so replaying the preplanned joint tail means
        snapping from wherever they are to a waypoint several millimetres away,
        at trajectory speed, along a path nothing checked from the real pose.
        Instead: take the live configuration, forward-kinematic the tool0 poses
        from it, and plan a fresh straight line to the movement's AUTHORED
        ``target_ee_frames``.

        The heavy lifting is ``plan_dual_arm_linear_independent``, which already
        does exactly this: FK of the given start state, cartesian interpolation
        at ``max_step_distance`` / ``max_step_angle``, then per waypoint ssik
        analytical branches seeded from the previous waypoint and sorted
        nearest-first, each candidate gated on joint-step continuity (the joint
        flip check) and on ``planner.check_collision`` with the cfab ACM.

        ! ``skip_env_collisions=True`` matches the offline planner's own M2/M3
        ! settings: self and tool collisions are checked, robot-vs-rigid-body
        ! ones are not. Residual tool-bar contact right after the gripper
        ! releases would otherwise fail the very first waypoints.

        Args:
            mv: The movement being executed; supplies ``start_state`` (for the
                cell/ACM and the base frame) and ``target_ee_frames``.
            tag (str): Log tag (the movement id).

        Returns:
            list[numpy.ndarray] | None: The replanned 12-vec path, whose first
            waypoint IS the live configuration, or None when the movement lacks
            target frames or the planner could not produce a valid path.
        """
        if self.cfab is None or mv is None or mv.start_state is None:
            self.get_logger().warn(f"[{tag}] replan: no cfab session or start_state.")
            return None
        targets = getattr(mv, 'target_ee_frames', None) or None
        if not targets or 'left' not in targets or 'right' not in targets:
            self.get_logger().warn(
                f"[{tag}] replan: {mv.movement_id!r} has no left/right "
                f"target_ee_frames to aim at.")
            return None

        # Start state = the movement's cell state with the LIVE arm joints.
        # ! The base frame is only replaced when something actually tracks it.
        # ! Writing hi.position in unconditionally is what once teleported the
        # ! planning robot to the world origin (see _apply_live_base_to_movement).
        hi = self.huskies[self.selected_robot_id].interface
        live_state = mv.start_state.copy()
        if live_state.robot_configuration is None:
            live_state.robot_configuration = self.cfab.robot_cell.zero_full_configuration()
        for names, values in zip(self._arm_joint_name_sets(), hi.arm_joint_pose):
            for n, v in zip(names, values):
                live_state.robot_configuration[n] = float(v)
        if self._base_pose_is_tracked():
            live_state.robot_base_frame = frame_from_pose((hi.position, hi.rotation))

        print(f"[{tag}] replanning a linear cartesian retreat from the live pose "
              f"at {M3_REPLAN_MAX_STEP_DISTANCE * 1000:.1f} mm / "
              f"{np.degrees(M3_REPLAN_MAX_STEP_ANGLE):.1f} deg steps...")
        try:
            jt = plan_dual_arm_linear_independent(
                self.cfab.planner, live_state,
                goal_ee_frames=targets,
                max_step_distance=M3_REPLAN_MAX_STEP_DISTANCE,
                max_step_angle=M3_REPLAN_MAX_STEP_ANGLE,
                skip_env_collisions=True,
            )
        except Exception as exc:
            self.get_logger().error(f"[{tag}] replan RAISED: {exc!r}")
            traceback.print_exc()
            return None
        if jt is None:
            # The planner already logged which waypoint failed and why
            # (unreachable / discontinuous / colliding).
            self.get_logger().error(
                f"[{tag}] replan FAILED: no collision-free, continuous cartesian "
                f"path from the live pose to the authored target frames.")
            return None

        path = path_12_from_joint_trajectory(jt)
        if not path:
            self.get_logger().error(f"[{tag}] replan returned an empty path.")
            return None
        return path

    def _live_arm_conf_12(self):
        """The live robot's twelve arm joint values (left 6 then right 6).

        Returns:
            numpy.ndarray: 12-vec of joint values in radians.
        """
        hi = self.huskies[self.selected_robot_id].interface
        return np.concatenate([
            np.asarray(hi.arm_joint_pose[i], dtype=float) for i in (0, 1)])

    def _patch_preplanned_to_live(self, mv, tag: str):
        """Stage 1 of the free-movement live replan: bridge, don't rebuild.

        A movement that follows the compliant M3 starts a little away from its
        preplanned first waypoint, because compliance settles wherever contact
        allows. That drift is usually small -- a degree or two -- and throwing
        away a good preplanned path over it is wasteful.

        So: interpolate a straight joint-space line from the live conf to the
        preplanned path's first waypoint, spaced at ``FM_JOINT_RESOLUTION``, and
        collision-check it (densely, at ``FM_VALIDATION_STEP_RAD``, via the same
        validator the full planner is gated on). If it is clear, that line is
        prepended to the preplanned path as a "patch" and the whole thing is
        returned as one trajectory -- so the joint-value preview plot and the
        traj-viz scrub show the patch and the original path as a single motion.

        A straight line is only safe over a SHORT distance in free space, which
        is exactly the case this handles. It is deliberately not a planner: when
        the line is blocked this returns None and the caller falls back to a
        full replan (stage 2).

        Args:
            mv: The movement carrying the preplanned ``trajectory``.
            tag (str): Log tag (the movement id).

        Returns:
            JointTrajectory | None: The patched trajectory, the untouched
            preplanned one when no patch is needed, or None when there is no
            preplanned path or the bridge is blocked (caller must replan).
        """
        preplanned = path_12_from_joint_trajectory(getattr(mv, 'trajectory', None))
        if not preplanned:
            print(f"[{tag}] no preplanned trajectory to patch; full replan.")
            return None

        live = self._live_arm_conf_12()
        start = np.asarray(preplanned[0], dtype=float)
        gap = float(np.abs(live - start).max())
        if gap <= FM_PATCH_TOLERANCE_RAD:
            self.get_logger().info(
                f"[{tag}] arms are already at the preplanned start "
                f"(max {np.degrees(gap):.3f} deg); keeping it unchanged.")
            return mv.trajectory

        # Waypoints at the planning resolution; the last interpolation step IS
        # preplanned[0], so it is left to the preplanned path to supply.
        n_steps = max(1, int(np.ceil(gap / FM_JOINT_RESOLUTION)))
        patch = [live + (start - live) * (k / n_steps) for k in range(n_steps)]

        patched = patch + [np.asarray(q, dtype=float) for q in preplanned]

        print(f"[{tag}] stage 1: bridging {np.degrees(gap):.2f} deg from the live "
              f"arms to the preplanned start with {len(patch)} waypoint(s) at "
              f"{FM_JOINT_RESOLUTION} rad; sweep-checking the full "
              f"{len(patched)}-waypoint patched path...")
        # ! Validate the WHOLE patched path, bridge + preplanned tail -- not just
        # ! the bridge. This runs only on a live replan ('Plan Movement'); plain
        # ! 'Load Movement' never reaches here, so replaying a preplanned path
        # ! stays instant. The tail deserves the check despite being "known
        # ! good": it was planned offline at a coarse joint resolution with no
        # ! swept checking, which is exactly the case that slips an arm through
        # ! a bar between waypoints.
        verdict = self._validate_free_planned_path(mv, patched)
        if not verdict['ok']:
            # Say WHERE it failed: a blocked bridge means the arms drifted
            # somewhere awkward, a blocked tail means the stored path itself is
            # unsafe from here. Either way stage 2 replans, but the distinction
            # tells the operator whether the stored plan is the problem.
            n_patch = len(patch)
            in_bridge = [s for s, _ in verdict['bad_segments'] if s < n_patch]
            in_tail = [s for s, _ in verdict['bad_segments'] if s >= n_patch]
            where = []
            if in_bridge:
                where.append(f"{len(in_bridge)} segment(s) in the bridge")
            if in_tail:
                where.append(f"{len(in_tail)} segment(s) in the PREPLANNED tail")
            self.get_logger().warn(
                f"[{tag}] stage 1 FAILED: {' and '.join(where)} hit "
                f"{verdict['bodies']}. Falling back to a full replan.")
            return None

        self.get_logger().info(
            f"[{tag}] stage 1 OK: patched trajectory is {len(patched)} waypoints "
            f"({len(patch)} patch + {len(preplanned)} preplanned), sweep-verified "
            f"end to end. The patch is waypoints 0..{len(patch) - 1} of the preview.")
        return joint_trajectory_from_path(patched)

    def _resync_start_state_to_live(self, mv, tag: str):
        """Point a free movement's start_state at where the arms ACTUALLY are.

        Both free movements are re-planned against the live robot, for the same
        reason but from different causes:

          M0  the operator may have jogged the arms since 'Load Movement'.
          M4  M3 ran under the COMPLIANCE controller, which settles wherever the
              contact lets it -- so the real end conf is never exactly M3's
              planned end. That planned end is what the chain propagated into
              M4.start_state, so planning M4 from it produces a path whose first
              waypoint the robot is not at. Executing that makes the arms snap
              to the first point at trajectory speed.

        Without this, pressing 'Plan Movement' again does NOT help: the planner
        keeps starting from the stale propagated conf, so the gap survives every
        replan.

        Args:
            mv: The movement whose ``start_state`` is resynced in place.
            tag (str): Log tag (the movement id).

        Returns:
            None.
        """
        before = None
        if mv.start_state is not None and mv.start_state.robot_configuration is not None:
            before = vec12_from_conf(mv.start_state.robot_configuration)
        self._inject_live_conf_into_state(mv.start_state)
        if before is not None:
            after = vec12_from_conf(mv.start_state.robot_configuration)
            drift = float(np.abs(np.asarray(after) - np.asarray(before)).max())
            print(f"[{tag}] start_state resynced to the live arms "
                  f"(max joint drift from the propagated conf: "
                  f"{np.degrees(drift):.2f} deg).")
        try:
            self.cfab.planner.set_robot_cell_state(mv.start_state)
        except Exception as e:
            print(f"[{tag}] WARN: cfab set_robot_cell_state after live-conf "
                  f"resync failed: {e}")
        # The check that matters most: everything planned below is collision-
        # checked with the robot wherever this leaves it.
        self._check_cfab_base_matches(mv, f'{tag} pre-plan')

    def _m1_goal_conf(self):
        """The transfer's goal configuration: the authored insert start conf, if any.

        Prefer the authored M2 start conf as M1's goal: it skips the planner's
        own goal IK (which can pick a +/-2pi-wrapped branch) and pins the goal
        bar pose to the authored conf's FK. Note the joint path's END still
        follows the derived start's IK branch (upstream pose-RRT behavior), so
        M2 can still land on a hard seed -- replan M1 when M2's linear IK
        cannot reach its first waypoint.

        Returns:
            Configuration | None: M2's ``start_state.robot_configuration``, or
            None when there is no M2 / it carries no configuration (the caller
            then falls back to M1's authored ``target_ee_frames``).
        """
        m2 = self._loaded_movement_of(MovementKind.DUAL_CONSTRAINED_LINEAR)
        if (m2 is not None and m2.start_state is not None
                and m2.start_state.robot_configuration is not None):
            print("[M1] goal_conf <- authored M2 start conf (wrap-safe branch).")
            return m2.start_state.robot_configuration
        return None

    def _m1_home_anchor(self):
        """The home carry anchor picked on the GUI slider, or None for 'all'.

        Reads the widget's live position rather than trusting the cached
        index: a slider rebuilt by reset_ui can miss the next drag's on-change
        callback (same hazard as bar_action_file_slider in
        load_bar_action_file).

        Returns:
            str | None: an ``M1_HOME_ANCHOR_CHOICES`` label, or None when the
            slider sits on index 0 ('all' = let the planner sample every
            anchor).
        """
        sld = getattr(self, 'm1_home_anchor_slider', None)
        if sld is not None:
            v = sld.value
            if v is not None:
                self._m1_home_anchor_idx = int(round(float(v)))
        anchor_idx = max(0, min(int(self._m1_home_anchor_idx),
                                len(M1_HOME_ANCHOR_CHOICES) - 1))
        home_anchor = None if anchor_idx == 0 else M1_HOME_ANCHOR_CHOICES[anchor_idx]
        if home_anchor is not None:
            print(f"[M1] home anchor override: {home_anchor}")
        return home_anchor

    def _m1_live_context(self, what='Derive Start/Goal'):
        """Guards and preamble shared by the M1 start buttons.

        Makes sure M1 is loaded with its bar, pushes the live base into the
        movement state (what the planner would see), fills a missing start
        configuration, and picks up the goal and the anchor slider.

        Args:
            what (str): the button's name, for the warnings.

        Returns:
            tuple | None: ``(mv, state, goal_conf, home_anchor)``, or None when
            a precondition failed (already warned).
        """
        mv = self.current_movement
        ti = self._loaded_index_of(MovementKind.DUAL_CONSTRAINED_FREE)
        transfer = None if ti is None else self._loaded_movements[ti]
        if mv is None or mv.start_state is None:
            if transfer is None:
                self.get_logger().warn(f"{what}: this action has no transfer movement.")
            else:
                self.get_logger().warn(
                    f"Load the transfer movement first ({transfer.movement_id}, "
                    f"index {ti}) -> Load Movement.")
            return None
        kind = self._kind_of(mv)
        if kind is not MovementKind.DUAL_CONSTRAINED_FREE:
            transfer_id = transfer.movement_id if transfer is not None else 'none in this action'
            kind_text = kind.value if kind is not None else type(mv).__name__
            self.get_logger().warn(
                f"{what} only works on the transfer movement ({transfer_id}); "
                f"loaded is {mv.movement_id} ({kind_text}).")
            return None
        if not self.active_bar_name:
            self.get_logger().warn("M1: active_bar_name not set.")
            return None
        # Same preamble as plan_selected_movement, so this sees exactly what
        # the planner would: live base pushed into the state, IK seed present.
        if not self._apply_live_base_to_movement(mv):
            return None
        self._fill_missing_start_conf(mv.start_state)
        goal_conf = self._m1_goal_conf()
        if goal_conf is None and not mv.target_ee_frames:
            self.get_logger().warn("M1: no authored M2 start conf and no target_ee_frames.")
            return None
        home_anchor = self._m1_home_anchor()
        self._m1_derived = None
        clear_collision_diagnosis(self)
        return mv, mv.start_state, goal_conf, home_anchor

    def _m1_pybullet_handles(self):
        """PyBullet ids of the cfab robot, its twelve arm joints and both tool0 links.

        Returns:
            tuple: ``(robot_puid, names_12, arm_joints, tool_link_left, tool_link_right)``.
        """
        robot_puid = self.cfab.planner.client.robot_puid
        names_12 = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]) + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
        arm_joints = pp.joints_from_names(robot_puid, names_12)
        tool_l = pp.link_from_name(robot_puid, TOOL_LINK_LEFT)
        tool_r = pp.link_from_name(robot_puid, TOOL_LINK_RIGHT)
        return robot_puid, names_12, arm_joints, tool_l, tool_r

    def _on_m1_anchor_slider(self, value):
        """Anchor slider callback: cache the index and move the bar preview."""
        self._m1_home_anchor_idx = int(round(float(value)))
        self._preview_m1_manual_bar()

    def _on_m1_manual_slider(self, attr, value):
        """Manual start slider callback: cache the value and move the bar preview.

        Args:
            attr (str): the cached attribute the slider drives.
            value: the slider's new value.
        """
        setattr(self, attr, float(value))
        self._preview_m1_manual_bar()

    def _m1_manual_geometry(self, mv):
        """The goal geometry the manual bar pose is built from, once per movement + goal.

        Base independent (grasps, bar axis, goal in the mobile-base frame), so
        it is cached until the movement or its goal changes.

        Args:
            mv: the loaded M1 movement.

        Returns:
            dict | None: see ``m1_manual_start.goal_geometry``; None without a
            goal configuration or with the bar not attached.
        """
        goal_conf = self._m1_goal_conf()
        if goal_conf is None:
            return None
        key = (getattr(mv, 'movement_id', None),
               tuple(round(float(v), 6) for v in vec12_from_conf(goal_conf)))
        cached = getattr(self, '_m1_manual_geom_cache', None)
        if cached is not None and cached[0] == key:
            return cached[1]
        state = mv.start_state
        self._fill_missing_start_conf(state)
        robot_puid, names_12, arm_joints, tool_l, tool_r = self._m1_pybullet_handles()
        # One-off and cheap (two FK probes), so no LockRenderer: that helper
        # only works on clients opened through pybullet_planning.
        geom = goal_geometry(self.cfab.planner, state, self.active_bar_name, goal_conf,
                             names_12, robot_puid, arm_joints, tool_l, tool_r)
        self.cfab.planner.set_robot_cell_state(state)
        self._m1_manual_geom_cache = (key, geom)
        return geom

    def _bar_extent_local(self):
        """Centre, length and radius of the active bar, from the real body's box.

        Returns:
            tuple | None: ``(centre_local, length, radius)`` -- the centre in
            the bar's own frame (the frame origin sits at a tip on some
            exports), the length along the bar and the radius, metres; None
            without a bar body.
        """
        puids = (self.cfab.client.rigid_bodies_puids or {}).get(self.active_bar_name) or []
        if not puids:
            return None
        # ! The box must be measured with the bar at the identity pose: a
        # ! world-axis-aligned box around a tilted bar is both too short along
        # ! the bar and off-centre. Put the body back where it was afterwards.
        body = puids[0]
        pose = pp.get_pose(body)
        pp.set_pose(body, ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
        try:
            lower, upper = pp.get_aabb(body)
        finally:
            pp.set_pose(body, pose)
        lower, upper = np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
        extents = sorted(float(v) for v in (upper - lower))
        length, radius = extents[-1], max(0.015, 0.5 * extents[0])
        centre_local = 0.5 * (lower + upper)
        return centre_local, float(length), float(radius)

    def _build_bar_ghost(self, colour, stub_grasps=None):
        """A see-through stand-in for the held bar, built from primitives.

        A cylinder of the bar's length along its long axis (the bar frame's
        z), plus -- when ``stub_grasps`` is given -- a short stub at each
        grasp point pointing where that arm's tool sits, so the roll is
        visible. Visual only: no collision shapes, not registered with the cfab
        cell, so no planner ever sees it. Primitives rather than a mesh clone:
        the clone of the cfab bar mesh rendered nothing in the GUI.

        Args:
            colour: RGBA for every primitive.
            stub_grasps: bar-from-tool0 poses of the two grasps (pybullet
                ``(pos, quat)``), or None for the bare bar.

        Returns:
            tuple: ``(body_id, centre_local, length, n_shapes)``, or
            ``(None, None, 0.0, 0)`` without a bar body.
        """
        extent = self._bar_extent_local()
        if extent is None:
            return None, None, 0.0, 0
        centre_local, length, radius = extent
        client = self.cfab.client.client_id
        shapes = [dict(shapeType=p.GEOM_CYLINDER, radius=radius, length=length,
                       position=tuple(centre_local.tolist()), orientation=(0.0, 0.0, 0.0, 1.0))]
        # Tool stubs: from each grasp point back along the tool's z (the flange
        # side), so the operator sees which way the tools point.
        stub = 0.15
        for grasp in (stub_grasps or []):
            g_pos = np.asarray(grasp[0], dtype=float)
            tool_z = np.asarray(pp.matrix_from_quat(grasp[1]), dtype=float)[:, 2]
            shapes.append(dict(shapeType=p.GEOM_BOX, halfExtents=(0.02, 0.02, 0.5 * stub),
                               position=tuple((g_pos - 0.5 * stub * tool_z).tolist()),
                               orientation=tuple(grasp[1])))
        visual = p.createVisualShapeArray(
            shapeTypes=[sh['shapeType'] for sh in shapes],
            radii=[sh.get('radius', 0.0) for sh in shapes],
            lengths=[sh.get('length', 0.0) for sh in shapes],
            halfExtents=[sh.get('halfExtents', (0.0, 0.0, 0.0)) for sh in shapes],
            visualFramePositions=[sh['position'] for sh in shapes],
            visualFrameOrientations=[sh['orientation'] for sh in shapes],
            rgbaColors=[colour] * len(shapes),
            physicsClientId=client)
        body = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=-1,
                                 baseVisualShapeIndex=visual, physicsClientId=client)
        return body, centre_local, float(length), len(shapes)

    def _m1_manual_ghost(self, geom):
        """The orange stand-in for the held bar that the manual sliders move (once per bar).

        Args:
            geom (dict): from ``m1_manual_start.goal_geometry`` (grasps).

        Returns:
            int | None: the PyBullet body id, or None without a bar.
        """
        name = self.active_bar_name
        cache = getattr(self, '_m1_manual_ghost_cache', None)
        if cache is not None and cache[0] == name:
            return cache[1]
        if cache is not None:
            pp.remove_body(cache[1])
            self._m1_manual_ghost_cache = None
        ghost, centre_local, length, n_shapes = self._build_bar_ghost(
            MANUAL_BAR_ORANGE, stub_grasps=(geom['grasp_bar_from_left'], geom['grasp_bar_from_right']))
        if ghost is None:
            return None
        self._m1_manual_ghost_cache = (name, ghost)
        self._m1_manual_ghost_shapes = n_shapes
        # For the debug-line drawing of the same bar (always rendered).
        self._m1_manual_ghost_dims = (centre_local, length)
        return ghost

    def _assembled_bar_pose(self):
        """World pose of the active bar at its assembled position (M3's start = M2's goal).

        Read from the authored release state (M3's start, where the bar rests
        in the structure); when that state does not carry a static bar frame,
        from M2's authored target tool0 frame composed with the grasp.

        Returns:
            tuple | None: pybullet ``(pos, quat_xyzw)``, or None when the action
            has neither.
        """
        m3 = self._loaded_movement_of(MovementKind.DUAL_INDEPENDENT_LINEAR)
        if m3 is not None and m3.start_state is not None:
            rb = (m3.start_state.rigid_body_states or {}).get(self.active_bar_name)
            if rb is not None and not rb.attached_to_link and rb.frame is not None:
                return pose_from_frame(rb.frame)
        m2 = self._loaded_movement_of(MovementKind.DUAL_CONSTRAINED_LINEAR)
        if m2 is not None and m2.start_state is not None and m2.target_ee_frames:
            rb = (m2.start_state.rigid_body_states or {}).get(self.active_bar_name)
            if rb is not None and rb.attached_to_link and rb.attachment_frame is not None:
                side = 'left' if 'left' in rb.attached_to_link else 'right'
                frame = m2.target_ee_frames.get(side)
                if frame is not None:
                    return pp.multiply(pose_from_frame(frame), pose_from_frame(rb.attachment_frame))
        return None

    def _show_assembled_bar_ghost(self):
        """Pink line along the loaded action's bar at its assembled pose, kept until the next Load BarAction.

        The guide for parking the base in the mount-once mocap test: the bar in
        the grippers has to end up on it after the transfer loop. Drawn as a
        debug line (bodies created at run time do not show in this viewer),
        and re-issued once a second by ``update`` so a debug wipe elsewhere
        (``clear_all_debug_drawing``) does not lose it.
        """
        self._assembled_bar_line = None
        if self.cfab is None or not self.active_bar_name:
            return
        pose = self._assembled_bar_pose()
        extent = self._bar_extent_local()
        if pose is None or extent is None:
            self.get_logger().warn(f"[BarAction] no assembled pose for {self.active_bar_name!r}; no pink bar.")
            return
        centre_local, length, _radius = extent
        # The bar's long axis is its own z (only the sign varies per export --
        # irrelevant for a line).
        half = 0.5 * length * np.array([0.0, 0.0, 1.0])
        self._assembled_bar_line = {
            'bar': self.active_bar_name,
            'a': tuple(pp.tform_point(pose, tuple((centre_local - half).tolist()))),
            'b': tuple(pp.tform_point(pose, tuple((centre_local + half).tolist()))),
            'centre': tuple(pp.tform_point(pose, tuple(centre_local.tolist()))),
            'ids': {},
            'stamp': 0.0,
        }
        self._redraw_assembled_bar_line(force=True)
        print(f"[BarAction] pink line = {self.active_bar_name} at its assembled pose "
              f"(M3 start), centre {np.round(self._assembled_bar_line['centre'], 3).tolist()}; "
              "it stays until the next Load BarAction.")

    def _redraw_assembled_bar_line(self, force=False):
        """(Re)issue the pink assembled-bar line; at most once a second unless forced."""
        line = getattr(self, '_assembled_bar_line', None)
        if line is None or self.cfab is None:
            return
        now = time.time()
        if not force and now - line['stamp'] < 1.0:
            return
        line['stamp'] = now
        client = self.cfab.client.client_id
        ids = line['ids']
        try:
            ids['axis'] = p.addUserDebugLine(
                line['a'], line['b'], lineColorRGB=ASSEMBLED_BAR_PINK[:3], lineWidth=8.0,
                lifeTime=0, replaceItemUniqueId=ids.get('axis', -1), physicsClientId=client)
            ids['text'] = p.addUserDebugText(
                f"{line['bar']} assembled (M3 start)",
                (np.asarray(line['centre']) + np.array([0.0, 0.0, 0.12])).tolist(),
                textColorRGB=ASSEMBLED_BAR_PINK[:3], textSize=1.4, lifeTime=0,
                replaceItemUniqueId=ids.get('text', -1), physicsClientId=client)
        except Exception as exc:  # drawing must never break the UI loop
            if not getattr(self, '_assembled_bar_line_warned', False):
                self._assembled_bar_line_warned = True
                self.get_logger().warn(f"[BarAction] pink bar line unavailable: {exc!r}")

    def _draw_m1_manual_bar_lines(self, world, geom):
        """Draw the manual bar preview as thick debug lines + a label at the given pose.

        Debug lines are drawn by the viewer no matter what happens to body
        rendering, so this is the preview that must always be visible: the bar
        along its grasp axis and a stub at each grasp point toward the tool.
        Items are replaced in place on every move.

        Args:
            world: the bar frame's world pose ``(pos, quat_xyzw)``.
            geom (dict): from ``m1_manual_start.goal_geometry``.
        """
        client = self.cfab.client.client_id
        centre_local, length = getattr(self, '_m1_manual_ghost_dims', (np.zeros(3), 1.4))
        axis_local = np.asarray(geom['axis_local'], dtype=float)
        ids = getattr(self, '_m1_manual_debug_ids', None) or {}
        segments = {'bar': (centre_local - 0.5 * length * axis_local,
                            centre_local + 0.5 * length * axis_local)}
        for side, grasp in (('left', geom['grasp_bar_from_left']),
                            ('right', geom['grasp_bar_from_right'])):
            g_pos = np.asarray(grasp[0], dtype=float)
            tool_z = np.asarray(pp.matrix_from_quat(grasp[1]), dtype=float)[:, 2]
            segments[side] = (g_pos, g_pos - 0.15 * tool_z)
        colour = MANUAL_BAR_ORANGE[:3]
        for key, (a, b) in segments.items():
            a_w = pp.tform_point(world, tuple(a.tolist()))
            b_w = pp.tform_point(world, tuple(b.tolist()))
            ids[key] = p.addUserDebugLine(
                a_w, b_w, lineColorRGB=colour, lineWidth=8.0 if key == 'bar' else 5.0,
                lifeTime=0, replaceItemUniqueId=ids.get(key, -1), physicsClientId=client)
        centre_w = np.asarray(pp.tform_point(world, tuple(centre_local.tolist())), dtype=float)
        ids['text'] = p.addUserDebugText(
            'M1 start bar (manual)', (centre_w + np.array([0.0, 0.0, 0.12])).tolist(),
            textColorRGB=colour, textSize=1.4, lifeTime=0,
            replaceItemUniqueId=ids.get('text', -1), physicsClientId=client)
        self._m1_manual_debug_ids = ids
        return centre_w

    def _remove_m1_manual_bar_lines(self):
        """Delete the debug-line preview of the manual bar."""
        ids = getattr(self, '_m1_manual_debug_ids', None) or {}
        for item in ids.values():
            try:
                p.removeUserDebugItem(item, physicsClientId=self.cfab.client.client_id)
            except Exception:
                pass
        self._m1_manual_debug_ids = {}

    def _focus_camera_on(self, target, distance=2.5):
        """Point the PyBullet camera at a world point, keeping its current angles.

        Args:
            target: world point ``(x, y, z)``.
            distance (float): camera distance in metres.
        """
        try:
            cam = p.getDebugVisualizerCamera(physicsClientId=self.cfab.client.client_id)
            yaw, pitch = float(cam[8]), float(cam[9])
        except Exception:
            yaw, pitch = 50.0, -35.0
        pp.set_camera(yaw, pitch, distance, list(map(float, target)))

    def _color_m1_manual_ghost(self, color):
        """Recolour every primitive of the manual bar preview."""
        cache = getattr(self, '_m1_manual_ghost_cache', None)
        if cache is None:
            return
        for k in range(getattr(self, '_m1_manual_ghost_shapes', 1)):
            p.changeVisualShape(cache[1], -1, shapeIndex=k, rgbaColor=color,
                                physicsClientId=self.cfab.client.client_id)

    def _hide_m1_manual_ghost(self):
        """Blank the manual bar preview (it reappears on the next slider move)."""
        self._color_m1_manual_ghost(TRANSPARENT)
        self._remove_m1_manual_bar_lines()

    def _poll_m1_manual_preview(self):
        """Each tick: refresh the bar preview when a manual slider or the anchor moved.

        The slider callbacks already do this, but a widget rebuilt by reset_ui
        can miss its next drag callback (see ``_m1_home_anchor``), and this also
        shows the bar right after M1 is loaded, before any slider is touched.
        """
        mv = self.current_movement
        if (mv is None or self.cfab is None
                or self._kind_of(mv) is not MovementKind.DUAL_CONSTRAINED_FREE):
            return
        key = (int(self._m1_home_anchor_idx),) + tuple(round(v, 4) for v in self._m1_manual_offsets())
        if key != getattr(self, '_m1_manual_preview_key', None):
            self._m1_manual_preview_key = key
            self._preview_m1_manual_bar()

    def _preview_m1_manual_bar(self):
        """Move the see-through bar to the pose the manual sliders describe (no IK).

        Runs on every slider change while M1 is loaded, so it only does
        geometry: the cached goal grasps, the anchor's canonical pose, the
        operator's roll / slide / shifts, and the live base to place it in the
        world. The arms appear after Confirm. Anchor `all` previews the first
        anchor (horizontal).
        """
        mv = self.current_movement
        if (mv is None or mv.start_state is None or self.cfab is None
                or not self.active_bar_name
                or self._kind_of(mv) is not MovementKind.DUAL_CONSTRAINED_FREE):
            return
        try:
            geom = self._m1_manual_geometry(mv)
            if geom is None:
                if not getattr(self, '_m1_manual_preview_warned', False):
                    self._m1_manual_preview_warned = True
                    self.get_logger().warn(
                        "[M1 manual] no bar preview: M2 has no authored start conf, or the "
                        "bar is not attached in M1's start state.")
                return
            idx = max(0, min(int(self._m1_home_anchor_idx), len(M1_HOME_ANCHOR_CHOICES) - 1))
            anchor = M1_HOME_ANCHOR_CHOICES[idx] if idx > 0 else 'horizontal'
            slide_m, roll_deg, perp1_m, perp2_m = self._m1_manual_offsets()
            pose = bar_pose_mb(geom, anchor, slide_m=slide_m, roll_deg=roll_deg,
                               perp1_m=perp1_m, perp2_m=perp2_m)
            created = getattr(self, '_m1_manual_ghost_cache', None) is None
            ghost = self._m1_manual_ghost(geom)
            if ghost is None:
                return
            world = pp.multiply(self._live_base_pose(), (pose['pos'], pose['quat']))
            pp.set_pose(ghost, world)
            self._color_m1_manual_ghost(MANUAL_BAR_ORANGE)
            centre_w = self._draw_m1_manual_bar_lines(world, geom)
            if created:
                mid = pose['bar_mid_mb']
                print(f"[M1 manual] orange preview bar created (body {ghost}); it follows the "
                      f"sliders (now centred {100 * mid[0]:+.0f} cm forward, {100 * mid[1]:+.0f} cm "
                      f"left, {100 * mid[2]:+.0f} cm up of the base; centre at world "
                      f"{np.round(centre_w, 2).tolist()}). Camera pointed at it.")
                # Point the camera at the new bar once, so it cannot be missed.
                self._focus_camera_on(centre_w)
        except Exception as exc:  # a preview must never break the UI loop
            if not getattr(self, '_m1_manual_preview_warned', False):
                self._m1_manual_preview_warned = True
                where = traceback.format_exc().strip().splitlines()[-3:]
                self.get_logger().warn(
                    f"[M1 manual] bar preview unavailable: {exc!r} at "
                    + " | ".join(line.strip() for line in where))

    def _m1_manual_offsets(self):
        """The four manual start sliders, read live from the widgets.

        Same hazard as ``_m1_home_anchor``: a slider rebuilt by reset_ui can
        miss its next drag callback, so the widget is read rather than the
        cached value (which is refreshed here).

        Returns:
            tuple[float, float, float, float]: ``(slide_m, roll_deg, perp1_m, perp2_m)``.
        """
        for widget, attr in (('m1_manual_slide_slider', '_m1_manual_slide_m'),
                             ('m1_manual_roll_slider', '_m1_manual_roll_deg'),
                             ('m1_manual_perp1_slider', '_m1_manual_perp1_m'),
                             ('m1_manual_perp2_slider', '_m1_manual_perp2_m')):
            sld = getattr(self, widget, None)
            if sld is not None:
                v = sld.value
                if v is not None:
                    setattr(self, attr, float(v))
        return (self._m1_manual_slide_m, self._m1_manual_roll_deg,
                self._m1_manual_perp1_m, self._m1_manual_perp2_m)

    def _show_m1_endpoints(self, state, start_conf, goal_arr, bar_start, bar_goal,
                           grasp_l, grasp_r, goal_conf, corridor=None, source='derived'):
        """Report both M1 endpoints, keep them in ``_m1_derived`` and put them on screen.

        Shared by the automatic derivation and the manual start: prints the
        feasibility report (collision-free / grasp-consistent flags for both
        confs, goal branch swap), stores ``self._m1_derived`` and drives the
        green preview robot (``Traj viz time`` 0 = START, 1 = GOAL, or the
        whole corridor when one exists) plus the red cfab robot's waypoint
        slider.

        Args:
            state (RobotCellState): M1's start state (bar attached).
            start_conf: the start 12-vector.
            goal_arr: the goal 12-vector actually used.
            bar_start, bar_goal: the bar's world poses at start / goal.
            grasp_l, grasp_r: the bar-from-tool0 grasp transforms.
            goal_conf: the authored goal configuration (to report a branch swap), or None.
            corridor: the derivation's collision-free corridor ``(poses, confs)``, or None.
            source (str): ``'derived'`` or ``'manual'``, for the closing hint.
        """
        planner = self.cfab.planner
        robot_puid, names_12, arm_joints, tool_l, tool_r = self._m1_pybullet_handles()
        # Kept local like the STAGE3 import in _plan_free_and_validate: core
        # pulls in the whole RRT stack, which the monitor otherwise never
        # imports at module level.
        from husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.core import validate_dual_arm_bar_pose

        # --- Independent feasibility report (mirrors the headless probe).
        with pp.LockRenderer():
            collide = _build_cfab_collision_fn(planner, state, names_12)
            goal_hit = collide(goal_arr)
            planner.set_robot_cell_state(state)
            start_hit = collide(start_conf)
            planner.set_robot_cell_state(state)

            def _grasp_ok(conf, bar_pose):
                return validate_dual_arm_bar_pose(
                    robot=robot_puid, arm_joints=arm_joints,
                    tool_link_left=tool_l, tool_link_right=tool_r,
                    full_conf=conf, bar_pose=bar_pose,
                    grasp_bar_from_left=grasp_l, grasp_bar_from_right=grasp_r,
                    pos_tolerance=1e-3, ori_tolerance=1e-2)
            goal_ok = _grasp_ok(goal_arr, bar_goal)
            start_ok = _grasp_ok(start_conf, bar_start)
            planner.set_robot_cell_state(state)

        def _fmt(vec):
            return "[" + ", ".join(f"{float(v):+.3f}" for v in vec) + "]"
        d_endpoints = float(np.abs(np.asarray(start_conf) - np.asarray(goal_arr)).max())
        title = 'live base, no RRT' if source == 'derived' else 'manual start, live base'
        print(f"\n=================== M1 ENDPOINTS ({title}) ===================")
        print(f"  GOAL  conf : {_fmt(goal_arr)}")
        print(f"        bar pos (xyz): {np.round(bar_goal[0], 4)}")
        print(f"        collision-free : {self._color_bool(not goal_hit)}    "
              f"grasp-consistent : {self._color_bool(goal_ok)}")
        if goal_conf is not None:
            # ssik may re-pick the goal on a branch compatible with the home
            # anchor; say so, because M1's end then no longer equals M2's start.
            d_goal = float(np.abs(np.asarray(goal_arr) - vec12_from_conf(goal_conf)).max())
            print(f"        max |goal - authored M2 start| : {d_goal:.4f} rad"
                  + ("  <-- BRANCH SWAPPED by ssik pairing" if d_goal > 1e-3 else ""))
        print(f"  START conf : {_fmt(start_conf)}")
        print(f"        bar pos (xyz): {np.round(bar_start[0], 4)}")
        print(f"        collision-free : {self._color_bool(not start_hit)}    "
              f"grasp-consistent : {self._color_bool(start_ok)}")
        print(f"  max |start - goal| joint delta: {d_endpoints:.4f} rad")
        print(f"  corridor collision-free (valid M1 path already): "
              f"{self._color_bool(corridor is not None)}")
        print("=========================================================================\n")

        self._m1_derived = {
            'start_conf': np.asarray(start_conf, dtype=float),
            'goal_conf': np.asarray(goal_arr, dtype=float),
            'corridor': corridor,
            'source': source,
        }

        # --- Show it. The path scrubbed by the preview is the corridor when
        # there is one (home -> goal = M1's direction), else just the two
        # endpoints: update() shows exact waypoints without interpolating, so
        # 'Traj viz time' reads START below t=1 and GOAL at t=1.
        if corridor is not None:
            _poses, confs = corridor
            path = [np.asarray(q, dtype=float) for q in reversed(confs)]
        else:
            path = [np.asarray(start_conf, dtype=float), np.asarray(goal_arr, dtype=float)]
        t = self.trajectory_time
        self.constrained_trajectory = [
            (np.asarray([q[:6] for q in path]), None, t, None),
            (np.asarray([q[6:] for q in path]), None, t, None),
        ]
        self.staging_free_trajectory = [None, None]
        self.set_arm_trajectory(self.constrained_trajectory[0], index=0)
        self.set_arm_trajectory(self.constrained_trajectory[1], index=1)
        # Bar + joints ride on the green preview robot from M1's authored grasp.
        self._refresh_preview_attached_bodies('bar_held', state)
        self.set_to_show_traj_state()
        # Red cfab robot: 'Constrained t' slider on the PyBullet panel.
        self._build_trajectory_waypoint_sliders()
        hint = ("Click 'Adopt derived start -> travel-to-load goal' to make it the "
                "transfer's start / the travel to load's goal."
                if source == 'derived' else "")
        print("[M1 preview] 'Traj viz time' 0 = START (bar-loading), 1 = GOAL "
              "(approach); cfab 'Constrained t' slider steps the same waypoints. " + hint)

    def confirm_m1_manual_start(self):
        """Human-in-the-loop M1 start: IK-check the bar pose chosen on the sliders and adopt it.

        Instead of the automatic derivation (a 120 s sweep that is sensitive
        to the base pose), the operator chooses the bar-loading pose: the
        carry anchor from the ``M1 home anchor`` slider (0 = try horizontal,
        vertical, back in that order and keep the first that works), then
        *slide along bar*, *roll about bar* and two *perpendicular shifts*
        (base axes, printed per anchor). This solves the dual-arm IK holding
        the bar there with the goal's grasps, on the branch nearest the goal,
        collision-checked against the cell; on success it shows both
        endpoints and ADOPTS the start (M1's start, M0's goal -- and the
        BarAction file when the 'Adopt also saves' toggle is ticked). 'Plan
        Movement' on M1 then runs the BiRRT from that start; 'Plan Movement'
        on M0 drives the arms there. Re-adjust and confirm again freely --
        each confirm takes well under a second.
        """
        ctx = self._m1_live_context('Confirm manual start')
        if ctx is None:
            return
        mv, state, goal_conf, home_anchor = ctx
        if goal_conf is None:
            self.get_logger().warn(
                "[M1 manual] needs M2's authored start conf as the goal "
                "(target_ee_frames alone are not enough here).")
            return
        anchor = home_anchor or 'all'
        slide_m, roll_deg, perp1_m, perp2_m = self._m1_manual_offsets()
        planner = self.cfab.planner
        robot_puid, names_12, arm_joints, tool_l, tool_r = self._m1_pybullet_handles()
        print(f"[M1 manual] anchor {anchor}: slide {slide_m:+.3f} m, roll {roll_deg:+.0f} deg, "
              f"perpendicular shifts {perp1_m:+.3f} / {perp2_m:+.3f} m ...")
        with pp.LockRenderer():
            result = manual_m1_start(
                planner, state, self.active_bar_name, goal_conf, names_12,
                robot_puid, arm_joints, tool_l, tool_r,
                anchor=anchor, slide_m=slide_m, roll_deg=roll_deg,
                perp1_m=perp1_m, perp2_m=perp2_m)
        if result.get('bar_mid_mb') is not None:
            pos = result['bar_mid_mb']
            print(f"[M1 manual] bar centre (between the grippers) in the robot frame: "
                  f"{100 * pos[0]:+.0f} cm forward, "
                  f"{100 * pos[1]:+.0f} cm left, {100 * pos[2]:+.0f} cm up "
                  f"(anchor {result['anchor']}; perp 1 = base {result['perp_axes'][0]}, "
                  f"perp 2 = base {result['perp_axes'][1]})")
        reason = result['reason']
        if reason == 'bar_not_attached':
            self.get_logger().warn(
                "[M1 manual] the bar is not attached in M1's start state; the transfer "
                "needs it in the grippers (see _ensure_bar_attached_for_mocap).")
            return
        if reason == 'no_ik':
            self.get_logger().warn(
                "[M1 manual] no arm configuration holds the bar there -- slide, roll or "
                "shift it, or pick another anchor.")
            return
        if reason == 'collision':
            self.get_logger().warn(
                "[M1 manual] the arms collide holding the bar there (pair drawn) -- "
                "slide, roll or shift it, or pick another anchor.")
            diag_state = state.copy()
            for n, v in zip(names_12, result['collision_conf']):
                diag_state.robot_configuration[n] = float(v)
            visualize_goal_ik_collision(self, diag_state)
            return
        if result['reseeded']:
            print("[M1 manual] note: the branch nearest the goal collided; another "
                  "IK branch was taken (the BiRRT may find it harder to connect).")
        self._show_m1_endpoints(
            state, result['start_conf'], result['goal_conf'],
            result['world_from_bar_start'], result['world_from_bar_goal'],
            result['grasp_bar_from_left'], result['grasp_bar_from_right'],
            goal_conf, corridor=None, source='manual')
        self._hide_m1_manual_ghost()
        self.adopt_m1_derived_start()
        fi = self._loaded_index_of(MovementKind.DUAL_FREE, free_home=False)
        free_to_load = ('the travel to load' if fi is None else
                        f"{self._loaded_movements[fi].movement_id} (index {fi})")
        print(f"[M1 manual] start adopted -> transfer start / travel-to-load goal. 'Plan "
              f"Movement' on {mv.movement_id} runs the BiRRT from it; {free_to_load} -> "
              f"'Plan Movement' drives the arms there.")

    def derive_m1_endpoints_live(self):
        """Run ONLY M1's start derivation against the live base -- no RRT --
        and put both endpoints on screen.

        ``plan_constrained_dual_arm(derive_start=True)`` is two stages: first
        ``_derive_constrained_start_for_plan`` solves the GOAL (the authored M2
        start conf, or IK on the authored target frames) and derives a
        bar-loading START by tracking the held bar backward from that goal to a
        home pose; only then does the SE(3) RRT search a path between them.
        When M1 "struggles", the interesting question is which of the two
        stages fails, and what the two endpoint configurations look like. This
        button runs stage one alone, prints the same feasibility report the
        headless ``--probe-endpoints`` mode does, and shows the result:

        * the green preview robot (with the bar riding in its grippers)
          follows the ``Traj viz time`` slider between START (t=0) and GOAL
          (t=1) -- or along the whole corridor when the derivation found a
          collision-free one (that corridor IS a valid M1 path);
        * the red cfab robot gets a ``Constrained t`` slider on the PyBullet
          panel stepping through the same waypoints, re-posing the full cell
          state.

        On ``goal_in_collision`` the colliding pair of the authored goal conf is
        drawn (cc_diagnosis), since a goal that collides at the LIVE base but
        not at the authored one is the usual reason an "easy" M1 fails after
        the base was parked by hand.

        The derived start is kept in ``self._m1_derived``; nothing is written
        into the movement until 'M1: Adopt derived start' is clicked.

        ! For the mount-once mocap protocol only the START matters (M1 is
        ! never executed -- the transfer loop leaves from the bar-loading pose
        ! directly), so this + Adopt replaces 'Plan Movement' on M1 entirely.
        """
        ctx = self._m1_live_context('Derive Start/Goal')
        if ctx is None:
            return
        mv, state, goal_conf, home_anchor = ctx
        planner = self.cfab.planner
        robot_puid, names_12, arm_joints, tool_l, tool_r = self._m1_pybullet_handles()

        print(f"[M1 derive] goal <- {'authored M2 start conf' if goal_conf is not None else 'IK on target_ee_frames'}, "
              f"anchor {home_anchor or 'all'}, up to 120 s ...")
        # Pause GUI rendering across the derivation (no-op when headless).
        with pp.LockRenderer():
            planner.set_robot_cell_state(state)
            bar_body = _bar_body_id(planner, self.active_bar_name)
            obstacles = _collect_obstacle_puids(planner, exclude={self.active_bar_name})
            (start_conf, bar_start, bar_goal, goal_arr,
             grasp_l, grasp_r, info) = _derive_constrained_start_for_plan(
                planner, state,
                active_bar_id=self.active_bar_name,
                bar_body=bar_body,
                obstacles=obstacles,
                robot_puid=robot_puid,
                arm_joints=arm_joints,
                tool_link_left=tool_l,
                tool_link_right=tool_r,
                joint_names_12=names_12,
                goal_conf=goal_conf,
                goal_ee_frames=mv.target_ee_frames if goal_conf is None else None,
                # Same first-attempt settings as _plan_M1_dispatch.
                random_seed=None,
                max_ik_attempts=20,
                bar_sweep_box=None,
                position_res=CDFM_POSITION_RES,
                rotation_res=CDFM_ROTATION_RES,
                home_anchor=home_anchor,
            )
            planner.set_robot_cell_state(state)

        if start_conf is None:
            reason = info.get('failure_reason', 'unknown')
            self.get_logger().warn(f"[M1 derive] FAILED: {reason}.")
            self._save_m1_derive_report(info, home_anchor, state)
            if reason == 'goal_in_collision' and goal_conf is not None:
                # Show WHY: the authored goal conf, FK'd at the live base, hits
                # something. (ssik may have re-picked a branch before the check;
                # the authored conf is the closest thing we can draw.)
                diag_state = state.copy()
                for n, v in zip(names_12, vec12_from_conf(goal_conf)):
                    diag_state.robot_configuration[n] = float(v)
                print("[M1 derive] drawing the authored goal conf's collision at the live base:")
                visualize_goal_ik_collision(self, diag_state)
            return

        self._show_m1_endpoints(state, start_conf, goal_arr, bar_start, bar_goal,
                                grasp_l, grasp_r, goal_conf, corridor=info.get('corridor'))
        self._save_m1_derive_report(info, home_anchor, state)

    def _save_m1_derive_report(self, info, home_anchor, state, plan_path=None, attempt=None):
        """Print the sweep summary and record the run for the dashboard.

        Every derivation result carries one entry per home candidate the sweep
        tried plus a timing profile. The terminal gets the short table; the run
        file gets everything, including which link hit which body, so the
        dashboard (``scripts/m1_dashboard_server.py``) can show the attempt in
        3D. A failed derivation is recorded too -- that is exactly the case
        worth looking at.

        A report must never turn a derivation into a crash, so the write is
        guarded; the cell state is restored either way, because the collision
        annotation moves the simulated robot around.

        Also records a full M1 plan (derive + RRT): pass the planner's ``info``
        and the path it returned, and the run carries the search's trees and
        outcome for the dashboard's "RRT search" card.

        Args:
            info: The ``info`` dict from ``_derive_constrained_start_for_plan``
                or from ``plan_constrained_dual_arm``.
            home_anchor (str | None): The anchor selection used.
            state: The movement start state the derivation ran on.
            plan_path: The joint path a full plan returned (None for derive-only
                runs and failed plans).
            attempt (int | None): Which retry of a plan this was.
        """
        print_m1_derivation_summary(info)
        bar_action = os.path.splitext(os.path.basename(
            self._current_action_path or ''))[0] or (self.active_bar_name or 'unknown')
        # Every pose in the run file is measured from this state's base, so the
        # dashboard is told which base that is: the live mocap reading when
        # something tracks it (_apply_live_base_to_movement wrote it in), else
        # the base the BarAction file authored for M1.
        base_source = ('mocap_live' if self._base_pose_is_tracked()
                       else 'bar_action_file')
        try:
            with pp.LockRenderer():
                write_m1_run(
                    self.cfab.planner, state, info,
                    problem=self.cfab.problem_name or DESIGN_PROBLEM_NAME,
                    bar_action=bar_action, active_bar=self.active_bar_name,
                    movement_id=getattr(self.current_movement, 'movement_id', None),
                    home_anchor=home_anchor, source='monitor',
                    plan_path=plan_path, attempt=attempt, base_source=base_source)
        except Exception as e:
            print(f"[M1 derive] could not record the run for the dashboard: {e}")
        finally:
            try:
                self.cfab.planner.set_robot_cell_state(state)
            except Exception:
                pass

    def adopt_m1_derived_start(self):
        """Make the last derived START M1's start conf and M0's goal.

        Writes ``self._m1_derived['start_conf']`` into M1's
        ``start_state.robot_configuration`` and backfills M0's
        ``target_configuration`` from it, exactly what a successful M1 plan
        would have left behind -- minus the trajectory. The derived goal is
        recorded on M1's ``target_configuration`` at the same time. Step A of
        the mount-once protocol then continues as usual: load M0, Plan
        Movement, execute, mount the bar.

        With 'Adopt also saves travel-to-load / transfer confs to file' ticked, those three confs
        are additionally written to the BarAction JSON
        (``_save_m1_m0_confs_to_bar_action_file``), so a session that exits
        can reload the file and plan M0 straight away -- as long as the base
        has not moved since. Unticked, they stay in memory only.

        ! This leaves M1 with a start conf but no trajectory on purpose. The
        ! "start only with a trajectory" invariant
        ! (_clear_m1_start_conf_without_trajectory) is enforced only when an
        ! M1 plan FAILS, so the adopted start survives normal use; a later
        ! successful 'Plan Movement' on M1 simply overwrites it.
        """
        derived = getattr(self, '_m1_derived', None)
        if not derived:
            self.get_logger().warn(
                "Nothing to adopt: click 'Transfer start: Derive start/goal only' first.")
            return
        m1 = self._loaded_movement_of(MovementKind.DUAL_CONSTRAINED_FREE)
        if m1 is None or m1.start_state is None:
            self.get_logger().warn("No transfer movement with a start_state is loaded.")
            return
        m1.start_state.robot_configuration = conf_from_12vec(derived['start_conf'])
        print(f"[M1 adopt] {m1.movement_id!r}.start_state.robot_configuration <- derived start "
              f"(no M1 trajectory; intended for the mount-once protocol).")
        # Keep the goal that was derived in the same breath as the start, so
        # the pair is readable in the file rather than only in the log.
        goal_conf = derived.get('goal_conf')
        if goal_conf is not None:
            m1.target_configuration = conf_from_12vec(goal_conf)
            print(f"[M1 adopt] {m1.movement_id!r}.target_configuration <- derived goal.")
        self._backfill_m0_target_from_m1()

        # * Optionally make the adopted confs outlive the session. Read the
        # * checkbox live and re-sync the flag first: a checkbox rebuilt by
        # * reset_ui can miss the next click's callback, exactly like the
        # * sliders around it.
        toggle = getattr(self, 'm1_adopt_save_toggle', None)
        if toggle is not None and toggle.value is not None:
            self._m1_adopt_writes_file = bool(toggle.value)
        if self._m1_adopt_writes_file:
            self._save_m1_m0_confs_to_bar_action_file()
        else:
            print("[M1 adopt] confs kept in memory only; tick 'Adopt also saves "
                  "travel-to-load / transfer confs to file' to write them to the BarAction JSON.")

        fi = self._loaded_index_of(MovementKind.DUAL_FREE, free_home=False)
        if fi is not None:
            print(f"[M1 adopt] next: select {self._loaded_movements[fi].movement_id} "
                  f"(index {fi}) -> Load Movement -> Plan Movement -> Exec.")

    # How far the tool0_L -> tool0_R transform may differ between M1's start
    # and its goal before the two configurations count as holding the bar
    # differently. Deliberately tight: a genuinely reused start (written by
    # Adopt, or read back from the BarAction JSON) matches to float precision,
    # so anything past round-off really is a different grasp.
    M1_GRASP_POS_TOL_M = 1e-3           # 1 mm
    M1_GRASP_ORI_TOL_RAD = 1e-2         # 0.57 deg of true relative rotation

    def _m1_relative_flange_pose(self, conf12, robot_puid, arm_joints, tool_l, tool_r):
        """The tool0_left -> tool0_right transform at a 12-DOF configuration.

        Holding one bar rigidly in both grippers fixes this transform -- it is
        the same at every waypoint of an EndEffectorConstrained movement. That
        makes it the cheapest way to ask whether two configurations hold the
        same bar the same way, needing neither a bar pose nor a grasp.

        ! Moves the pybullet robot, so callers restore the cell state after.

        Args:
            conf12: 12-DOF configuration, left arm's 6 then right arm's 6.
            robot_puid (int): PyBullet body id of the planning robot.
            arm_joints: The 12 arm joint indices, in conf12's order.
            tool_l (int): Left tool0 link index.
            tool_r (int): Right tool0 link index.

        Returns:
            tuple: PyBullet pose (point, quat) of right tool0 in left tool0.
        """
        pp.set_joint_positions(robot_puid, arm_joints, np.asarray(conf12, dtype=float))
        world_from_l = pp.get_link_pose(robot_puid, tool_l)
        world_from_r = pp.get_link_pose(robot_puid, tool_r)
        return pp.multiply(pp.invert(world_from_l), world_from_r)

    def _m1_reusable_start_conf(self, mv, goal_conf):
        """Decide whether M1's stored start configuration can be used as-is.

        'M1: Adopt derived start' and a reloaded BarAction both leave a real
        start configuration on ``mv.start_state``. Deriving another one costs
        a fresh sampling sweep and -- worse -- can land on a DIFFERENT start
        than the robot is already parked at, which strands M0: it drove the
        arms to the stored start, so a new one means re-planning and
        re-executing M0. When the stored start already holds the bar the way
        the goal does, it is used directly and the sweep is skipped.

        The gate is the rigid-bar constraint itself: tool0_L -> tool0_R must
        match at start and goal. That also rejects the dual-arm home pose
        ``_fill_missing_start_conf`` seeds into an authored M1 (whose start
        conf ships as null) -- holding no bar, its flange transform cannot
        match the goal's.

        ? Only the start/goal PAIR is checked here. Whether the stored start
        ? is still collision-free at the live base is left to the planner,
        ? which reports it as a normal failure reason.

        Args:
            mv: The M1 movement, whose ``start_state`` carries the candidate.
            goal_conf: M1's goal configuration, or None when the goal is
                authored as ``target_ee_frames`` instead.

        Returns:
            np.ndarray | None: The 12-vec that will be planned from, or None
            to fall back to deriving a start.
        """
        state = getattr(mv, 'start_state', None)
        if state is None or state.robot_configuration is None:
            return None
        start12 = np.asarray(vec12_from_conf(state.robot_configuration), dtype=float)

        planner = self.cfab.planner
        robot_puid = planner.client.robot_puid
        names_12 = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]) + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
        arm_joints = pp.joints_from_names(robot_puid, names_12)
        tool_l = pp.link_from_name(robot_puid, TOOL_LINK_LEFT)
        tool_r = pp.link_from_name(robot_puid, TOOL_LINK_RIGHT)

        with pp.LockRenderer():
            rel_start = self._m1_relative_flange_pose(
                start12, robot_puid, arm_joints, tool_l, tool_r)
            if goal_conf is not None:
                rel_goal = self._m1_relative_flange_pose(
                    np.asarray(vec12_from_conf(goal_conf), dtype=float),
                    robot_puid, arm_joints, tool_l, tool_r)
            else:
                # No goal conf: the authored target frames give the same
                # transform directly, with no IK needed.
                frames = mv.target_ee_frames or {}
                if frames.get('left') is None or frames.get('right') is None:
                    planner.set_robot_cell_state(state)
                    return None
                rel_goal = pp.multiply(pp.invert(pose_from_frame(frames['left'])),
                                       pose_from_frame(frames['right']))
            # Undo the FK probes above; they moved the planning robot.
            planner.set_robot_cell_state(state)

        pos_err = float(np.linalg.norm(
            np.asarray(rel_start[0], dtype=float) - np.asarray(rel_goal[0], dtype=float)))
        # ! pp.quat_angle_between is acos(q0 . q1), which is HALF the angle
        # ! the two orientations actually differ by. Double it so the number
        # ! compared (and printed) is the real twist of the bar.
        ori_err = 2.0 * float(pp.quat_angle_between(rel_start[1], rel_goal[1]))
        if (pos_err <= self.M1_GRASP_POS_TOL_M
                and ori_err <= self.M1_GRASP_ORI_TOL_RAD):
            print(f"[M1] reusing the STORED start conf: it holds the bar like "
                  f"the goal (flange transform differs by {pos_err * 1e3:.2f} mm / "
                  f"{np.degrees(ori_err):.3f} deg). Skipping the derivation sweep.")
            return start12

        # Not a match. Say which of the two cases this is -- an authored M1
        # that was merely seeded, or a real start that disagrees with the goal.
        home12 = np.asarray(HUSKY_DUAL_ARM_HOME_CONF_12, dtype=float)
        if np.allclose(start12, home12, atol=1e-6):
            print("[M1] no stored start (authored M1 ships a null start conf, "
                  "seeded with dual-arm home); deriving one.")
        else:
            self.get_logger().warn(
                f"[M1] the stored start conf does NOT hold the bar like the "
                f"goal: flange transform differs by {pos_err * 1e3:.1f} mm / "
                f"{np.degrees(ori_err):.2f} deg (tolerance "
                f"{self.M1_GRASP_POS_TOL_M * 1e3:.1f} mm / "
                f"{np.degrees(self.M1_GRASP_ORI_TOL_RAD):.2f} deg). Ignoring it "
                f"and deriving a start instead -- the two would need the bar to "
                f"slip in the grippers.")
        return None

    def _plan_M1_dispatch(self, mv):
        """Constrained dual-arm (bar held): state-based task-space RRT.

        Grasps, bar pose, obstacles, and collision setup are all derived by
        the planner from mv.start_state. The start configuration comes from
        one of two routes:

        - ``derive_start=False`` when ``mv.start_state`` already carries a
          start that holds the bar like the goal does (Adopt put it there, or
          it was reloaded from a saved file). No sampling, and the start stays
          the one M0 already drove the arms to.
        - ``derive_start=True`` otherwise, asking the planner to sample a
          feasible grasp-consistent start -- the authored M1 ships a null
          start conf, so a fresh action always takes this route. Retried up
          to 3 times with a re-seeded sweep.

        See _m1_reusable_start_conf for the gate between them.
        """
        if not self.active_bar_name:
            self.get_logger().warn("M1: active_bar_name not set.")
            return None
        if not mv.target_ee_frames:
            self.get_logger().warn("M1: missing target_ee_frames.")
            return None
        goal_conf = self._m1_goal_conf()
        home_anchor = self._m1_home_anchor()
        common = dict(
            active_bar_id=self.active_bar_name,
            goal_conf=goal_conf,
            goal_ee_frames=mv.target_ee_frames if goal_conf is None else None,
            stage=M1_PLANNER_STAGE,
            position_res=CDFM_POSITION_RES,
            rotation_res=CDFM_ROTATION_RES,
            max_time=120.0,
        )
        path = info = None

        # * A start already on the movement (Adopt, or a reloaded file) is
        # * planned from directly: derive_start=False makes the planner read
        # * start_state.robot_configuration instead of sampling a new one.
        reuse_start = self._m1_reusable_start_conf(mv, goal_conf)
        if reuse_start is not None:
            # Pause GUI rendering during the search (no-op when headless).
            with pp.LockRenderer():
                path, info = plan_constrained_dual_arm(
                    self.cfab.planner, mv.start_state,
                    derive_start=False, **common,
                )
            # Every plan attempt is recorded for the dashboard, found or not.
            self._save_m1_derive_report(info, home_anchor, mv.start_state, plan_path=path)
            if path is None:
                # ! Deliberately NOT falling back to deriving. A derived start
                # ! is a DIFFERENT conf from the one the robot was driven to by
                # ! M0, so silently searching for one here would stand the arms
                # ! somewhere they are not -- the exact surprise reusing the
                # ! stored start exists to avoid.
                self.get_logger().warn(
                    f"[M1] planning from the stored start failed: "
                    f"{info.get('failure_reason')}. The stored start is left "
                    f"untouched. To search for a new one, run 'Transfer start: "
                    f"Derive start/goal only (no RRT)' and Adopt it -- the travel "
                    f"to load then has to be re-planned and re-executed to reach it.")
                return None
        else:
            # Multi-start: when a run fails, retry with a re-seeded derived
            # start and a widened bar sweep box (hard scenes like B226 need a
            # different home bar pose to find a corridor). The anchor selection
            # stays fixed across retries (it composes with the widened box).
            start_retries = 3
            for retry_idx in range(start_retries):
                extra = {}
                if retry_idx > 0:
                    extra = dict(
                        start_random_seed=9973 * retry_idx,
                        start_bar_sweep_box=((-0.4, 0.4), (-0.4, 0.4), (-0.5, 0.3)),
                    )
                    print(f"[M1] retry {retry_idx + 1}/{start_retries} with re-seeded "
                          f"derived start.")
                with pp.LockRenderer():
                    path, info = plan_constrained_dual_arm(
                        self.cfab.planner, mv.start_state,
                        derive_start=True,
                        start_home_anchor=home_anchor,
                        **common, **extra,
                    )
                self._save_m1_derive_report(info, home_anchor, mv.start_state,
                                            plan_path=path, attempt=retry_idx + 1)
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

    def _bar_hold_ee_drift(self, path12, template_state):
        """Per-waypoint drift of the left_from_right relative EE pose.

        For a bar held rigidly by both grippers the left-tool0->right-tool0
        relative transform must stay constant along the whole path. This
        FKs both tool0s at every waypoint (on a copy of ``template_state``)
        and measures how far each waypoint's relative transform has drifted
        from the FIRST waypoint's.

        Args:
            path12 (Sequence): Waypoints, each a 12-vec (left 6 + right 6
                joint values).
            template_state (RobotCellState): State providing the base frame
                + non-arm joints for FK; copied, not mutated.

        Returns:
            tuple[list[float], list[float]]: ``(pos_dev_m, ang_dev_rad)``,
            one entry per waypoint (both start at 0.0 for waypoint 0).
        """
        planner = self.cfab.planner
        names_12 = (list(HUSKY_DUAL_UR5e_JOINT_NAMES[0])
                    + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1]))

        # ! Push the cell state ONCE, then move only the arm joints per waypoint.
        # ! tool0 FK depends solely on the base frame and the arm joints, so the
        # ! per-waypoint set_robot_cell_state this used to do was re-placing the
        # ! robot AND all ~112 rigid bodies 2x per waypoint for no gain -- ~10 s
        # ! on a 457-waypoint M1, and it is what made the monitor visibly trace
        # ! the trajectory when a preplanned movement was loaded. Now ~0.04 s.
        #
        # Staying in pybullet pose space also drops the compas
        # Frame<->Transformation round-trip the old version went through, which
        # shifts the reported drift by ~1.6 um / 2e-5 deg (321x and 26000x
        # inside the thresholds this feeds). If anything it is the more faithful
        # number -- fewer conversions -- but do not be surprised that historical
        # logs differ in the 4th decimal.
        planner.set_robot_cell_state(template_state.copy())

        saved_client = pp.CLIENT
        pp.CLIENT = self.cfab.client.client_id
        pp.CLIENTS.setdefault(pp.CLIENT, True)
        relatives = []
        try:
            robot = self.cfab.client.robot_puid
            arm_joints = pp.joints_from_names(robot, names_12)
            left_link = pp.link_from_name(robot, "left_ur_arm_tool0")
            right_link = pp.link_from_name(robot, "right_ur_arm_tool0")
            restore = pp.get_joint_positions(robot, arm_joints)
            try:
                for q12 in path12:
                    pp.set_joint_positions(robot, arm_joints,
                                           np.asarray(q12, dtype=float))
                    # left_from_right: the transform the grasp must hold rigid.
                    relatives.append(pp.multiply(
                        pp.invert(pp.get_link_pose(robot, left_link)),
                        pp.get_link_pose(robot, right_link)))
            finally:
                pp.set_joint_positions(robot, arm_joints, restore)
        finally:
            pp.CLIENT = saved_client

        ref_inv = pp.invert(relatives[0])
        pos_devs = []
        ang_devs = []
        for rel in relatives:
            delta = pp.multiply(ref_inv, rel)
            pos_devs.append(float(np.linalg.norm(pp.point_from_pose(delta))))
            # pybullet quaternions are xyzw; the scalar part is the rotation
            # magnitude, so angle = 2*acos(|w|).
            qw = abs(float(pp.quat_from_pose(delta)[3]))
            qw = min(max(qw, 0.0), 1.0)
            ang_devs.append(2.0 * float(np.arccos(qw)))
        return pos_devs, ang_devs

    def _frame_pose_residual(self, frame_a, frame_b):
        """Position + orientation difference between two compas Frames.

        Args:
            frame_a (Frame): First frame.
            frame_b (Frame): Second frame.

        Returns:
            tuple[float, float]: ``(position difference in metres, orientation
            difference in radians)``. The orientation difference uses the
            absolute quaternion dot product, so a sign-flipped quaternion (the
            same rotation) correctly reads as zero.
        """
        d_pos = float(np.linalg.norm(
            np.asarray(frame_a.point) - np.asarray(frame_b.point)
        ))
        q_a = np.asarray(frame_a.quaternion.xyzw, dtype=float)
        q_b = np.asarray(frame_b.quaternion.xyzw, dtype=float)
        d_ang = 2.0 * float(np.arccos(
            np.clip(abs(float(np.dot(q_a, q_b))), 0.0, 1.0)
        ))
        return d_pos, d_ang

    def _verify_transfer_endpoint(self, path12, template_state, goal_conf12):
        """Check, per arm, whether a constrained (CDFM) transfer path really
        ends where the live-base IK asked it to.

        ! Why this matters: `plan_constrained_dual_arm` does NOT aim at the
        ! goal *conf*. When it is given a goal_conf (our case) it FKs only the
        ! LEFT tool0 of that conf and turns it into a BAR pose
        ! (`world_from_bar_goal = FK_left(goal_conf) * inv(grasp_bar_from_left)`,
        ! api.py). Every waypoint's RIGHT tool0 is then placed by the rigid
        ! grasp captured at the path's START (`grasp_bar_from_right`), i.e. by
        ! where the arms physically are right now -- not by the goal conf's
        ! right arm. So the LEFT arm lands on target by construction while the
        ! RIGHT arm inherits any mismatch between the authored left->right
        ! relative tool0 transform and the physically held one. Check D below
        ! measures exactly that mismatch, and it predicts the right arm's
        ! residual in the servoing tracker.

        Prints four comparisons (nothing is mutated):
          A. joint delta   -- path end vs goal conf, per arm.
          B. tool0 pose    -- FK(path end) vs FK(goal conf), per arm. Differs
             from A when the planner lands on another joint branch that has
             the same pose.
          C. tool0 pose    -- FK(path end) vs the authored target EE frames.
             This is what the servoing tracker plots.
          D. bar grasp     -- authored left->right relative tool0 transform vs
             the one actually held at the path's start.

        Args:
            path12 (Sequence): Planned waypoints, each a 12-vec (left 6 joint
                values then right 6).
            template_state (RobotCellState): State supplying the base frame +
                non-arm joints for FK. Copied, never mutated.
            goal_conf12 (Sequence[float]): The 12-vec the live-base IK solved,
                which was handed to the planner as its goal.
        """
        if not len(path12):
            return
        planner = self.cfab.planner
        left_names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0])
        right_names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
        names_12 = left_names + right_names

        def _state_at(conf12):
            """Copy the template and write one 12-vec arm conf into it."""
            state = template_state.copy()
            for name, value in zip(names_12, conf12):
                state.robot_configuration[name] = float(value)
            return state

        def _tool0(state, side):
            """World-frame tool0 Frame of one arm at the given state."""
            return _fk_link_frame(planner, state, f"{side}_ur_arm_tool0")

        end12 = np.asarray(path12[-1], dtype=float)
        goal12 = np.asarray(goal_conf12, dtype=float)
        end_state = _state_at(end12)
        goal_state = _state_at(goal12)

        # A. Joint-space agreement, per arm.
        d_joint_L = float(np.max(np.abs(end12[:6] - goal12[:6])))
        d_joint_R = float(np.max(np.abs(end12[6:] - goal12[6:])))
        print(
            f"[transfer verify A] path end vs IK goal conf (max joint delta): "
            f"L={np.degrees(d_joint_L):.3f} deg | R={np.degrees(d_joint_R):.3f} deg"
        )

        # E. How far the path actually travels. The pre-execution safeguard only
        # compares the FIRST and LAST waypoints, so a path that swings far out
        # and comes back reads as a tiny move (e.g. "33 waypoints, max joint
        # delta 0.2 deg"). This reports the real excursion instead.
        path_arr = np.asarray(path12, dtype=float)
        excursion_L = float(np.max(np.abs(path_arr[:, :6] - path_arr[0, :6])))
        excursion_R = float(np.max(np.abs(path_arr[:, 6:] - path_arr[0, 6:])))
        print(
            f"[transfer verify E] max joint excursion from the path's start over "
            f"all {len(path_arr)} waypoints: L={np.degrees(excursion_L):.2f} deg | "
            f"R={np.degrees(excursion_R):.2f} deg"
        )

        # B. Pose agreement with the goal conf, per arm. A large value here with
        # a small A means the planner tracked a different bar pose; a small
        # value here with a large A is just a different (equivalent) branch.
        try:
            for side, label in (("left", "L"), ("right", "R")):
                d_pos, d_ang = self._frame_pose_residual(
                    _tool0(end_state, side), _tool0(goal_state, side))
                print(
                    f"[transfer verify B] {label} path-end tool0 vs goal-conf "
                    f"tool0: pos={d_pos * 1000:.2f} mm ang={np.degrees(d_ang):.3f} deg"
                )
        except Exception as e:
            self.get_logger().warn(f"[transfer verify] FK comparison failed: {e}")
            return

        # C. Pose agreement with the AUTHORED targets -- the quantity the
        # servoing tracker plots, so these two should match each other.
        targets = getattr(self, '_last_ik_target_ee_frames', None)
        if not targets or 'left' not in targets or 'right' not in targets:
            return
        for side, label in (("left", "L"), ("right", "R")):
            d_pos, d_ang = self._frame_pose_residual(
                _tool0(end_state, side), targets[side])
            print(
                f"[transfer verify C] {label} path-end tool0 vs AUTHORED target: "
                f"pos={d_pos * 1000:.2f} mm ang={np.degrees(d_ang):.3f} deg"
            )

        # D. The over-constraint check. With one bar rigidly held by both
        # grippers the left->right relative tool0 transform is fixed by the
        # physical grasp, so the two authored targets can only BOTH be reached
        # if they imply that same relative transform. Whatever is left over
        # here is a residual the transfer servo loop can never null out.
        start_state = _state_at(np.asarray(path12[0], dtype=float))
        authored_rel = (Transformation.from_frame(targets['left']).inverted()
                        * Transformation.from_frame(targets['right']))
        held_rel = (Transformation.from_frame(_tool0(start_state, 'left')).inverted()
                    * Transformation.from_frame(_tool0(start_state, 'right')))
        delta = Frame.from_transformation(authored_rel.inverted() * held_rel)
        d_pos = float(np.linalg.norm(list(delta.point)))
        d_ang = 2.0 * float(np.arccos(min(max(abs(float(delta.quaternion.w)), 0.0), 1.0)))
        print(
            f"[transfer verify D] authored left->right relative tool0 vs the "
            f"one physically held: pos={d_pos * 1000:.2f} mm "
            f"ang={np.degrees(d_ang):.3f} deg"
        )
        if d_pos > 0.002:
            self.get_logger().warn(
                f"[transfer verify] The mounted bar's grasp does not match the "
                f"authored one by {d_pos * 1000:.1f} mm. Both arms hold ONE "
                f"rigid bar, so both authored tool0 targets cannot be reached "
                f"at once: the planner puts the LEFT arm on target and the "
                f"RIGHT arm absorbs the whole mismatch. Expect ~this much "
                f"steady-state right-arm error in the servoing tracker, and no "
                f"amount of extra iterations will remove it."
            )

    def _check_inter_ee_invariance(self, jt, template_state):
        """For an M2 (bar-held) trajectory, verify the left_from_right
        relative pose is constant over the path. Logs max/mean translation
        + rotation drift relative to the first waypoint.
        """
        path = path_12_from_joint_trajectory(jt)
        if len(path) < 2:
            return
        pos_devs, ang_devs = self._bar_hold_ee_drift(path, template_state)
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
        """Free dual-arm to the fixed home conf, starting from the LIVE arms.

        The action's authored M4 target is a placeholder; the known-good
        dual-arm home is used instead (matches headless_bar_action_planner).

        M4 always follows the compliant M3, which settles wherever contact
        allows -- so the conf M3 propagated into M4's start_state is never quite
        where the arms are. Two stages handle that, cheapest first:

          1. PATCH. Bridge the (usually small) drift with a collision-checked
             straight line and keep the preplanned path behind it. Preserves a
             known-good trajectory and costs no search.
          2. REPLAN. Only if that line is blocked: resync to live and plan the
             whole movement again, gated on the dense swept check.

        Returns:
            JointTrajectory | None: None when both stages fail.
        """
        patched = self._patch_preplanned_to_live(mv, mv.movement_id)
        if patched is not None:
            return patched

        # * Wrap the fixed home 12-vec in a compas Configuration so the tamp
        # helper's dict-indexed extraction works (raw numpy 12-vecs raise
        # IndexError on string joint-name indexing).
        goal_conf = conf_from_12vec(HUSKY_DUAL_ARM_HOME_CONF_12)
        # No robot_configuration check first: the resync fills it from the live
        # arms, so an absent or stale propagated conf is not a blocker.
        self._resync_start_state_to_live(mv, mv.movement_id)
        return self._plan_free_and_validate(mv, mv.movement_id, goal_conf, max_time=30.0)

    def ik_live_base_for_selected_movement(self):
        """IK at the LIVE base for the current movement's START EE frames.

        Intended for M2/M3 (and a support robot's linear moves). Solves IK for
        every arm of the connected robot (``RobotSpec.side_keys``) to the
        authored world-frame start EE poses but for the LIVE base, warm-started
        from the LIVE robot arm conf, with full cfab collision checking against
        the movement's start_state ACM. On success it sets goal_arm_pose (one
        entry per arm); the user then clicks
        'Plan Both Arms to Goal (composite)' to plan a free transit there. Does
        NOT write mv.trajectory.

        The start EE frames come from ONE source: the previous movement's
        authored ``target_ee_frames`` (a movement starts where the previous one
        ended). They are never derived by FK from
        ``start_state.robot_configuration`` -- that configuration is overwritten
        with the live arm pose by the servoing loop, so an FK-derived target
        would drift with the robot instead of staying at the authored pose. If
        the authored frames are missing this warns and does nothing.

        Returns:
            bool: True on success (goal_arm_pose updated), False on any
            precondition miss (including missing authored start EE frames) or IK
            failure. Existing UI callers ignore the return value; the new
            ``replan_free_to_movement_start_live`` uses it to bail cleanly.
        """
        if self._refuse_display_only_entry('IK Live Base'):
            return False
        if self.current_movement is None:
            self.get_logger().warn("Load a movement first.")
            return False
        mv = self.current_movement

        # 1) Fetch the AUTHORED start EE frames.
        # ! The authored `target_ee_frames` data is the single source of truth for
        # ! EE targets -- never derive them by FK. A movement's START EE pose is
        # ! the authored target of the movement that ran BEFORE it (M2 starts
        # ! where M1 ended, M3 where M2 ended). WHICH movement that is comes from
        # ! the cycle's own start-EE map, resolved once at Load BarAction -- not
        # ! from `index - 1`, which the split export breaks (between the insert
        # ! and the retreat sit two screw events that author nothing).
        # ! Why FK is wrong: the visual-servoing loop overwrites
        # ! mv.start_state.robot_configuration with the LIVE arm pose after every
        # ! executed iteration (see husky_world.servo_to_movement_start_live). FK
        # ! from that configuration therefore returns wherever the robot currently
        # ! is, so the "target" drifts along with the robot each pass and the loop
        # ! can never converge on the pose the designer actually authored.
        idx = self.current_movement_index
        spec = self._connected_robot()
        sources = {side: self._start_ee_source(idx, side)
                   for side in spec.side_keys}
        start_ee_frames = {
            side: src.target_ee_frames[side]
            for side, src in sources.items() if src is not None
        } or None
        # Cache the authored target EE frames on self so
        # `replan_free_to_movement_start_live`'s endpoint verification can
        # compare the composite plan's final tool0 poses back against the
        # authored targets that drove this IK call.
        self._last_ik_target_ee_frames = start_ee_frames
        missing = [side for side, src in sources.items() if src is None]
        if missing:
            # No authored data to aim at -- warn and do nothing rather than
            # silently falling back to an FK-derived (drifting) target.
            self.get_logger().warn(
                f"No authored start EE frames for {mv.movement_id!r}: nothing "
                f"before it authors target_ee_frames for {missing} without an "
                f"arm moving in between. Doing nothing (EE targets are never "
                f"derived from FK). If this is a split export, check that Load "
                f"BarAction opened BOTH halves -- the insert that authors these "
                f"poses lives in the jointing file."
            )
            return False
        print(f"[IK Live Base] start EE frames from "
              f"{ {side: src.movement_id for side, src in sources.items()} }.")

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
        # The other robots where they are now (a husky mocap sees moves live).
        self._apply_obstacle_beliefs(live_state)
        self.cfab.planner.set_robot_cell_state(live_state)
        # Also pass mv.start_state.robot_configuration as an alternate IK
        # seed: it's a bar-holding pose whose FK produces the very target
        # frames, so trac_ik seeded there converges to that (or a nearby)
        # collision-free branch, escaping the self-colliding branches
        # trac_ik lands on when seeded from the extended-arm HOME conf.
        if spec.dual_arm:
            # Override target_ee_frames so _solve_bar_action_goal_ik uses the
            # start-state derived frames (it reads monitor.target_ee_frames).
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
            arm_confs = None if conf12 is None else [conf12[:6], conf12[6:]]
        else:
            # Single-arm robot: same IK core, one group, one flange target.
            seeds = ([mv.start_state.robot_configuration]
                     if mv.start_state.robot_configuration is not None else [])
            conf = solve_goal_ik_generic(
                self.cfab.planner, live_state,
                {spec.group_for_side(side): frame for side, frame in start_ee_frames.items()},
                seed_confs=seeds)
            arm_confs = None
            if conf is not None:
                goal_state = live_state.copy()
                goal_state.robot_configuration = conf
                self.movement_goal_state = goal_state
                arm_confs = [[conf[n] for n in names] for names in spec.arm_joint_names]

        if arm_confs is None:
            self.get_logger().warn("IK at live base FAILED.")
            return False
        for i, arm_conf in enumerate(arm_confs):
            self.goal_arm_pose[i] = np.asarray(arm_conf, dtype=float)
        # Ghost must render live_base + IK conf together; otherwise the
        # ghost's tool0 drifts (live_base != start_state base, so rendering
        # stored-base + IK-conf gives a different tool0).
        self.goal_base_pose = (hi.position, hi.rotation)

        # Self-test: FK at the GOAL state (live_base + IK_conf, stored on
        # monitor.movement_goal_state by the IK above). Do NOT use the local
        # live_state here — the IK writes the new conf onto a copy, so
        # live_state.robot_configuration is still the OLD seed conf, which would
        # FK to (live_base * FK(old)) — i.e. the target offset by exactly the
        # base offset, masking a successful IK as an apparent failure.
        gs = getattr(self, 'movement_goal_state', None)
        try:
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
            parts = []
            for side in spec.side_keys:
                fk = _fk_link_frame(self.cfab.planner, gs, spec.flange_for_side(side))
                d_pos, d_ang = _residual(fk, start_ee_frames[side])
                parts.append(f"{side} pos={d_pos*1000:.2f} mm ang={np.degrees(d_ang):.3f} deg")
            print("[IK Live Base] FK self-test residual: " + " | ".join(parts))
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
        self.discard_bar_holding_marker_takes()

    def discard_bar_holding_marker_takes(self):
        """Drop every recorded-but-unsaved marker take and its fit drawing.

        Every 'Record + Fit + Viz' click is kept in memory until the next
        'Save markerset data', which writes ALL of them into one file stamped
        with ONE reference pose (the loaded movement's start bar pose). A take
        recorded somewhere else -- the fit sanity check right after mounting
        the bar at the bar-loading pose, or a take with a bad marker fit --
        must therefore be thrown away before the real takes of a bar are
        saved, or it lands in that bar's file and is scored against a
        reference it never aimed at. This is that throw-away; Save calls it
        too once the file is written.
        """
        n = len(self.marker_set_data)
        self.marker_set_data = []
        for uid in self._bar_holding_fit_line_uids:
            try:
                pp.remove_debug(uid)
            except Exception:
                pass
        self._bar_holding_fit_line_uids = []
        if n:
            self.get_logger().info(f"[bar take] discarded {n} unsaved take(s).")

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

        One entry per BAR, not per file: the split export's release half
        (``B6__R.json``) is left out of the list because selecting the jointing
        half already opens both (``load_action_cycle``). Legacy problems are
        unaffected -- they have no release halves.

        Attribute is kept under the legacy name for back-compat with
        UI/widgets and existing callers; contents are now BarAction files.
        """
        action_dir = os.path.join(
            DESIGN_DATA_DIRECTORY, DESIGN_PROBLEM_NAME, 'BarActions',
        )
        files = list_bar_actions(action_dir, cycle_only=True)
        if not files:
            print(f"No BarAction *.json files under: {action_dir}")
            return []
        print(f"Found {len(files)} BarAction files:")
        for i, fname in enumerate(files):
            print(f"  {i}: {fname}")
        return files

    # * ------------------------------------------------------------------
    # * ActionSchedule panel: one schedule entry (one action file, one robot)
    # * is the unit of work. The connected robot plans and runs its OWN
    # * entries; another robot's entry is only shown (and can be marked done).
    # * Everything below reads its state with getattr(...): the headless
    # * harnesses build the monitor without __init__, so none of it exists there
    # * and they keep the legacy behaviour.
    # * ------------------------------------------------------------------

    def _problem_dir(self) -> str:
        """The design problem folder, ``<DESIGN_DATA_DIRECTORY>/<DESIGN_PROBLEM_NAME>``.

        Returns:
            str: The folder holding ``ActionSchedule.json``, ``progress.json`` and ``BarActions/``.
        """
        return problem_root(DESIGN_PROBLEM_NAME, DESIGN_DATA_DIRECTORY)

    def _load_schedule_state(self) -> None:
        """Pick schedule mode or the legacy BarAction list, once at start-up.

        Schedule mode needs ``USE_ACTION_SCHEDULE`` on and a COMPLETE schedule:
        ``ActionSchedule.json`` loads and every entry's action file (clean export
        or sidecar) is on disk. Otherwise the legacy file slider stays and ONE
        line says why -- accuracy-test problems also ship a schedule, but only a
        few of its files. In schedule mode this also loads ``progress.json``
        (which is what poses the other robots from their beliefs), selects the
        first pending entry and prints the roster.
        """
        self._schedule = None
        self._progress = None
        self._schedule_flags = {}
        self._clean_entries = set()
        legacy = "-- using the legacy BarAction list."
        if not self.USE_ACTION_SCHEDULE:
            print(f"[Schedule] USE_ACTION_SCHEDULE is 0 {legacy}")
            return
        root = self._problem_dir()
        try:
            schedule = load_schedule(root)
        except ValueError as e:
            print(f"[Schedule] {SCHEDULE_FILENAME} is not a valid schedule ({e}) {legacy}")
            return
        if schedule is None:
            print(f"[Schedule] {DESIGN_PROBLEM_NAME} has no {SCHEDULE_FILENAME} {legacy}")
            return
        missing = missing_action_files(schedule)
        if missing:
            print(f"[Schedule] {SCHEDULE_FILENAME} lists {len(schedule.entries)} entries but "
                  f"{len(missing)} files are missing {legacy}")
            return
        try:
            progress = load_progress(root, schedule)
        except ValueError as e:
            print(f"[Schedule] cannot use progress.json ({e}) {legacy}")
            return
        self._schedule, self._progress = schedule, progress
        self._schedule_run_id = new_run_id(self._connected_robot().name)
        self._selected_entry_idx = min(progress.current_index, len(schedule.entries) - 1)
        self._rescan_schedule_flags()
        self._print_schedule_roster()
        # * Schedule mode is decided only here (after husky_world.init built the
        # * huskies), so this is where the connected robot takes its own belief.
        world._seed_connected_husky_from_progress(self, self._connected_robot())

    def _rescan_schedule_flags(self, indices: Optional[list] = None) -> None:
        """Re-read how far each entry's action file is solved (schedule_ui.EntryFlags).

        Scans the file 'Load entry' would load: its ``.live-solved.json`` sidecar
        when there is one (unless the entry was reopened in this run), else the
        clean export. ! A file that cannot be read or parsed only warns and shows
        zero counts: this also runs inside 'Mark entry done', where an exception
        would stop the monitor tick.

        Args:
            indices (list | None): Entry indices to rescan; None = every entry.
        """
        schedule = getattr(self, '_schedule', None)
        if schedule is None:
            return
        if indices is None:
            indices = range(len(schedule.entries))
        clean_entries = getattr(self, '_clean_entries', None) or set()
        self._schedule_flags = dict(getattr(self, '_schedule_flags', None) or {})
        for i in indices:
            path = schedule.action_path(schedule.entry(i), prefer_sidecar=i not in clean_entries)
            try:
                self._schedule_flags[i] = scan_entry_flags(path)
            except (OSError, ValueError, KeyError) as e:
                self.get_logger().warn(f"[Schedule] cannot scan entry {i} ({path}): {e}")
                self._schedule_flags[i] = EntryFlags(0, 0, 0, False)

    def _schedule_header_text(self) -> str:
        """The panel's header: connected robot, problem, progress; then the other robots.

        The second line says where each other robot's pose comes from
        (``progress_io.obstacle_sources_line``), so it is visible without the
        log. Mocap may still override a base on top (see
        ``_obstacle_beliefs_with_sources``).

        Returns:
            str: Two lines, e.g. ``'robot Cindy (domain 86) | problem ... |
            progress 3/48 done'`` and ``'others: Alice <- live (entry 3) | Belle <- parked'``.
        """
        schedule, progress = self._schedule, self._progress
        connected = self._connected_robot()
        n_done = sum(progress.is_done(e.index) for e in schedule.entries)
        others = obstacle_sources_line(progress, connected.name,
                                       exported_action=getattr(self, '_loaded_action', None))
        return (f"robot {connected.name} (domain {connected.domain_id}) | "
                f"problem {schedule.problem_name} | progress {n_done}/{len(schedule.entries)} done"
                f"\nothers: {others}")

    def _schedule_row_text(self, index: int, *, selected: bool) -> str:
        """One entry's row of the list (schedule_ui.entry_row_text).

        Args:
            index (int): Schedule index.
            selected (bool): Whether the entry slider points at it.

        Returns:
            str: The row.
        """
        entry = self._schedule.entry(index)
        return entry_row_text(entry, self._schedule_flags[index], self._progress.status(index),
                              executable=self._entry_is_executable_here(entry),
                              selected=selected)

    def _print_schedule_roster(self) -> None:
        """Print the header lines and one row per schedule entry to the terminal."""
        for line in self._schedule_header_text().split('\n'):
            print(f"[Schedule] {line}")
        for e in self._schedule.entries:
            print("  " + self._schedule_row_text(e.index, selected=e.index == self._selected_entry_idx))

    def _selected_schedule_index(self) -> int:
        """The entry the 'Schedule entry' slider points at, read live (see _slider_index).

        Returns:
            int: A valid schedule index.
        """
        return self._slider_index(getattr(self, 'schedule_entry_slider', None),
                                  getattr(self, '_selected_entry_idx', 0),
                                  len(self._schedule.entries))

    def _entry_is_executable_here(self, entry: Optional[ScheduleEntry] = None) -> bool:
        """Whether this monitor run may plan / execute an entry.

        Args:
            entry (ScheduleEntry | None): The entry; None = the loaded one.

        Returns:
            bool: True when the connected robot runs the entry, when no entry is
            loaded, and always on a legacy problem (no schedule).
        """
        if getattr(self, '_schedule', None) is None:
            return True
        if entry is None:
            entry = getattr(self, '_loaded_entry', None)
        return entry is None or is_executable_by(entry, self._connected_robot().name)

    def _refuse_display_only_entry(self, what: str) -> bool:
        """Warn and say True when another robot's entry is loaded (for display only).

        Args:
            what (str): The refused action, for the warning (e.g. ``'Plan Movement'``).

        Returns:
            bool: True when the caller must stop.
        """
        if self._entry_is_executable_here():
            return False
        entry = self._loaded_entry
        self.get_logger().warn(
            f"{what} refused: entry {entry.index} ({entry.action_id}) belongs to "
            f"{entry.robot}; it is loaded for display only on "
            f"{self._connected_robot().name}.")
        return True

    def _refuse_while_tasks_run(self, what: str, *, schedule_only: bool = False) -> bool:
        """Warn and return True while a queued step / execution is running or waiting for 'Confirm Exec'.

        ! One task at a time: every wait shares the single 'Confirm Exec' /
        ! 'Cancel Exec' pair, so a second queued task would be released by the
        ! same click. Loading another entry or movement mid-task would also change
        ! what the running task reads.

        Args:
            what (str): The refused action, for the warning (e.g. ``'Load entry'``).
            schedule_only (bool): Only refuse in schedule mode. For Cindy's
                existing buttons, which never refused on a legacy problem.

        Returns:
            bool: True when the caller must stop.
        """
        if schedule_only and getattr(self, '_schedule', None) is None:
            return False
        # The task that is running right now (the caller may be inside it) is
        # not "another" task -- see the task loop in update().
        running = getattr(self, '_running_task', None)
        if not [t for t in (getattr(self, 'tasks', None) or []) if t is not running]:
            return False
        self.get_logger().warn(
            f"{what} refused: a step / execution is still running or waiting for "
            f"'Confirm Exec'. Finish it or click 'Cancel Exec' first.")
        return True

    def queue_servo_to_movement_start(self, use_transfer: bool = False) -> None:
        """Queue the visual-servoing loop ('3) / 3b) Servo to Mv Start').

        ! In schedule mode it is refused while another task runs: the loop waits
        ! on the same 'Confirm Exec' as the steps, so one click would release both.
        ! Legacy problems keep the old behaviour (always queued).

        Args:
            use_transfer (bool): Plan a bar-held constrained transfer each
                iteration (button 3b) instead of a free transit (button 3).
        """
        if self._refuse_while_tasks_run('Servo to Mv Start', schedule_only=True):
            return
        self.tasks.append(world.servo_to_movement_start_live(self, use_transfer=use_transfer))

    def _load_hint(self) -> str:
        """Name of the button that loads an action, for the "click ... first" messages.

        Returns:
            str: ``"'Load entry'"`` in schedule mode, else ``"'Load BarAction'"``.
        """
        return "'Load entry'" if getattr(self, '_schedule', None) is not None else "'Load BarAction'"

    def _shown_movements(self) -> list:
        """The movements the Movement slider and its readouts list.

        Returns:
            list: The connected robot's loaded movements, or -- while another
            robot's entry is loaded for display only -- that entry's movements.
        """
        loaded = getattr(self, '_loaded_entry_bundle', None)
        if loaded is not None and not self._entry_is_executable_here():
            return list(loaded.movements)
        return self._loaded_movements

    def _now_line(self) -> str:
        """The 'Now:' line: loaded entry and movement (schedule_ui.now_line_text).

        Returns:
            str: The line.
        """
        entry = getattr(self, '_loaded_entry', None)
        if entry is None:
            return now_line_text(None, None, 0, None)
        if self._entry_is_executable_here(entry):
            movements = self._loaded_movements
            return now_line_text(entry, self.current_movement_index, len(movements),
                                 self.current_movement)
        # Display only: nothing is loaded into the cell, so follow the slider.
        movements = self._shown_movements()
        idx = self._slider_index(getattr(self, 'bar_movement_slider', None),
                                 self._selected_movement_idx, len(movements))
        if idx < 0:
            return now_line_text(entry, None, 0, None)
        return now_line_text(entry, idx, len(movements), movements[idx])

    def _refresh_schedule_readouts(self) -> None:
        """Keep the schedule panel's lines in step with the sliders (every UI tick).

        The '-> entry' readout names the entry 'Load entry' would open, the
        'Now:' line what is loaded, and the rows move their '>' marker. Row
        colours only change on the next rebuild.
        """
        schedule = getattr(self, '_schedule', None)
        if schedule is None:
            return
        selected = self._selected_schedule_index()
        text = getattr(self, 'schedule_entry_text', None)
        if text is not None:
            entry = schedule.entry(selected)
            text.set_text(f"{self._schedule_row_text(selected, selected=False).strip()}  "
                          f"({os.path.basename(entry.file)})")
        text = getattr(self, 'schedule_now_text', None)
        if text is not None:
            text.set_text(self._now_line())
        lo, hi = getattr(self, '_schedule_row_window', (0, 0))
        for row, i in zip(getattr(self, 'schedule_rows', None) or [], range(lo, hi)):
            row.set_text(self._schedule_row_text(i, selected=i == selected))

    def select_prev_entry(self) -> None:
        """'Prev entry' button: point the entry slider one entry up (loads nothing)."""
        self._select_entry(self._selected_schedule_index() - 1)

    def select_next_entry(self) -> None:
        """'Next entry' button: point the entry slider one entry down (loads nothing)."""
        self._select_entry(self._selected_schedule_index() + 1)

    def _select_entry(self, index: int) -> None:
        """Point the entry slider at an entry, clamped to the schedule.

        Args:
            index (int): Wanted schedule index.
        """
        if getattr(self, '_schedule', None) is None:
            return
        idx = max(0, min(int(index), len(self._schedule.entries) - 1))
        self._selected_entry_idx = idx
        slider = getattr(self, 'schedule_entry_slider', None)
        if slider is not None:
            slider.set_value(idx)
        # ! A PyBullet debug slider can not be moved from code, and its position
        # ! wins in _selected_schedule_index, so rebuild the panel there instead.
        # On DPG, rebuild only when the entry left the shown rows, so the rows
        # re-centre on it and its row is highlighted.
        lo, hi = getattr(self, '_schedule_row_window', (0, 0))
        if not isinstance(_common._global_backend, DearPyGuiBackend) or not lo <= idx < hi:
            self.reset_ui(self.goal_arm_pose)

    def load_schedule_entry(self, index: Optional[int] = None, *,
                            prefer_sidecar: Optional[bool] = None) -> None:
        """'Load entry' button: load one schedule entry's action.

        The connected robot's own entry goes through ``_finish_action_load``
        (cell session, live start, obstacle beliefs, UI, movement 0), exactly like
        'Load BarAction'. ! Another robot's entry is loaded for DISPLAY ONLY: its
        states name that robot's joints, so they are never pushed into this
        robot's cell. Its movements are listed, the readouts follow it, Plan /
        Exec / step stay refused, and 'Mark entry done' still works.

        The previous entry's loaded movement, planned path and joint preview are
        dropped first, so nothing planned for it can be executed on this one.
        Refused while a queued step / execution still runs, and (one error line,
        nothing loaded) when ``schedule_io.load_entry`` rejects the file.

        Args:
            index (int | None): Schedule index; None = the entry slider's.
            prefer_sidecar (bool | None): Load the ``.live-solved.json`` sidecar
                when there is one. None = yes, unless the entry was reopened in
                this run (``_clean_entries``); False = the clean export (see
                ``reset_all_movements_to_clean``).
        """
        if self._refuse_while_tasks_run('Load entry'):
            return
        schedule = getattr(self, '_schedule', None)
        if schedule is None:
            self.get_logger().warn("No ActionSchedule loaded; use 'Load BarAction'.")
            return
        idx = self._selected_schedule_index() if index is None else int(index)
        self._selected_entry_idx = idx
        entry = schedule.entry(idx)
        if prefer_sidecar is None:
            prefer_sidecar = idx not in (getattr(self, '_clean_entries', None) or set())
        # ! A file that does not fit its entry (type / robot, its predecessor, or
        # ! movement kinds that do not add up) is refused with one clear error
        # ! line; nothing is loaded and the previous entry stays as it was.
        try:
            loaded = load_entry(schedule, entry, prefer_sidecar=prefer_sidecar)
        except ValueError as e:
            self.get_logger().error(f"Not loading entry {idx} ({entry.file}): {e}")
            return
        # Set before the load below: its UI rebuild reads the loaded entry.
        self._loaded_entry, self._loaded_entry_bundle = entry, loaded
        # ! Forget the previous entry's movement and planned path: 'Exec' must
        # ! never replay a trajectory that was planned for another entry.
        self.current_movement = None
        self.current_movement_index = None
        self.movement_start_state = None
        self.target_ee_frames = None
        self._reset_planned_arm_trajectory()
        self._preview_joint_data = None
        self.get_logger().info(
            f"Loading schedule entry {idx} ({entry.action_id}, {entry.robot}) from {loaded.path}")
        if self._entry_is_executable_here(entry):
            self._finish_action_load(loaded.action, loaded.path, loaded)
            return

        self.get_logger().warn(
            f"Entry {idx} belongs to {entry.robot}; loaded for display only -- "
            f"Plan/Exec are disabled on {self._connected_robot().name}.")
        print(f"[Schedule] entry {idx} {entry.action_id} "
              f"({os.path.basename(loaded.path)}), {len(loaded.movements)} movements:")
        for i, (mv, kind) in enumerate(zip(loaded.movements, loaded.kinds)):
            print(f"  [{i}] {mv.movement_id} [{kind.value}] ctrl={mv.controller}")
        if getattr(self, '_is_live_monitor', False):
            self._selected_movement_idx = 0
        self.reset_ui(self.goal_arm_pose)

    def set_ignore_built_assembly_collisions(self, on: bool) -> None:
        """'Ignore built-bar collisions' toggle: set the switch and reload the loaded entry.

        The ignore flags are written into every movement's start state at the
        first Load Movement of an action, so the entry is reloaded ('Load entry')
        to start again from clean states; that load then applies the new setting.
        ! Planned paths not saved to a sidecar are dropped by the reload.

        Args:
            on (bool): True = the planner and IK ignore the built bars and joints
                (drawn faint); False = they are obstacles again.
        """
        self.IGNORE_BUILT_ASSEMBLY_COLLISIONS = int(bool(on))
        self._mocap_hide_applied = False
        entry = getattr(self, '_loaded_entry', None)
        self.get_logger().info(
            f"[built bars] ignore built-bar collisions: {'ON' if on else 'OFF'}"
            + (f" -- reloading entry {entry.index}" if entry is not None else ""))
        if entry is not None:
            self.load_schedule_entry(entry.index)

    def mark_entry_done(self) -> None:
        """'Mark entry done' button: queue ``_mark_entry_done_task`` (it may wait for Confirm Exec).

        The loaded entry is captured NOW, at the click. Refused while any queued
        step / execution still runs or waits: one 'Confirm Exec' click would
        otherwise answer both waits (and a double click would mark twice).
        """
        if self._refuse_while_tasks_run("'Mark entry done'"):
            return
        # ! Arm motions sent directly (not as tasks) are invisible to the guard
        # ! above; marking done mid-motion would store a mid-motion 'live' belief.
        # ! Checked here only, so a stuck flag cannot lock out every button.
        hi = self.huskies[self.selected_robot_id].interface if getattr(self, 'huskies', None) else None
        if hi is not None and any(bool(x) for x in getattr(hi, 'is_arm_executing', [])):
            self.get_logger().warn("'Mark entry done' refused: an arm is still moving. "
                                   "Wait until it stops.")
            return
        self.tasks.append(self._mark_entry_done_task(
            getattr(self, '_loaded_entry', None), getattr(self, '_loaded_entry_bundle', None)))

    def _mark_entry_done_task(self, entry: Optional[ScheduleEntry],
                              loaded: Optional[LoadedEntry]) -> Generator[None, None, None]:
        """Mark an entry done, store the acting robot's belief, save ``progress.json``.

        - The connected robot's own entry: the belief is LIVE -- the mocap base
          (or, not seen by mocap, the action's authored base) and the measured
          arm joints. When the entry's action carries planned trajectories they
          are saved to its ``.live-solved.json`` sidecar (the clean export is
          never written).
        - Another robot's entry: only after the operator's 'Confirm Exec'; the
          belief is that robot's EXPORTED end state, recorded as 'assumed'.

        Then the drawn other huskies move to their new beliefs, the entry's flags
        are rescanned, the slider moves to the first pending entry and the panel
        is rebuilt.

        ! Runs on the monitor tick, which does not catch exceptions (one would
        ! stop the monitor), so reading the belief and saving only log errors.

        Args:
            entry (ScheduleEntry | None): The entry loaded when the button was clicked.
            loaded (LoadedEntry | None): Its loaded bundle.

        Yields:
            None: One yield per monitor tick while waiting for the operator.
        """
        progress = getattr(self, '_progress', None)
        if entry is None or loaded is None or progress is None:
            self.get_logger().warn("No schedule entry loaded; click 'Load entry' first.")
            return
        connected = self._connected_robot()
        executable = self._entry_is_executable_here(entry)
        if not executable:
            # Clear a stale 'Cancel Exec' so it cannot skip this confirm.
            self._servo_abort = False
            confirmed = yield from world.wait_for_operator_confirm(
                self,
                f"Entry {entry.index} belongs to {entry.robot}. Confirm Exec to mark it "
                f"done with the EXPORTED end state (belief 'assumed'); Cancel Exec to abort.")
            # A 'Cancel Exec' used here must not cancel the next step's confirm.
            self._servo_abort = False
            if not confirmed:
                return
        try:
            belief = (self._live_belief(loaded) if executable else
                      belief_after(loaded.action, loaded.spec, entry.index, source=BELIEF_ASSUMED))
        except Exception as e:
            # Any failure here (a missing base frame, a malformed action) must not
            # escape: the monitor tick does not catch task exceptions.
            self.get_logger().error(
                f"Entry {entry.index} NOT marked done: no end state for {entry.robot} ({e}).")
            return

        run_id = getattr(self, '_schedule_run_id', None) or new_run_id(connected.name)
        progress.mark_done(entry, connected.name, run_id, belief)
        # A new mark: 'Load entry' may prefer this entry's sidecar again.
        self._clean_entries = getattr(self, '_clean_entries', set()) - {entry.index}
        try:
            path = save_progress(progress, self._problem_dir())
            print(f"[Schedule] entry {entry.index} {entry.action_id} marked done by "
                  f"{connected.name}; {entry.robot}'s belief <- {belief.source}. Saved {path}.")
        except Exception as e:
            self.get_logger().error(
                f"Entry {entry.index} is marked done in this run only: progress.json could "
                f"not be saved ({e}). Fix the problem, then 'Mark entry done' again.")
        self._redraw_other_huskies_from_progress()
        # * Keep what was planned for this entry, in its sidecar. (The writer
        # * logs its own write errors.)
        if executable and any(getattr(mv, 'trajectory', None) is not None
                              for mv in loaded.action.movements):
            written = self._write_action_halves_for(loaded.action.movements, tag='Mark done')
            if written:
                print(f"[Schedule] planned trajectories saved to "
                      f"{', '.join(os.path.basename(w) for w in written)}.")
        self._rescan_schedule_flags([entry.index])
        self._selected_entry_idx = min(progress.current_index, len(self._schedule.entries) - 1)
        self.reset_ui(self.goal_arm_pose)

    def _live_belief(self, loaded: LoadedEntry) -> RobotBelief:
        """The connected robot's belief right now, for marking its own entry done.

        Args:
            loaded (LoadedEntry): The loaded entry (its robot is the connected one).

        Returns:
            RobotBelief: Source 'live': the mocap base when mocap drives the base
            and saw this robot in the last ``LIVE_BELIEF_MAX_AGE_S`` seconds, else
            (with a warning when mocap should have seen it) the base the action
            was authored at; the measured arm joints.
        """
        spec = loaded.spec
        husky = self.huskies[self.selected_robot_id]
        hi = husky.interface
        if self._base_pose_is_tracked() and self._mocap_sees_husky(husky, LIVE_BELIEF_MAX_AGE_S):
            base_pose = (hi.position, hi.rotation)
        else:
            base_pose = pose_from_frame(loaded.action.movements[-1].start_state.robot_base_frame)
            if self._base_pose_is_tracked():
                self.get_logger().warn(
                    f"Mocap has not seen {spec.name} in the last {LIVE_BELIEF_MAX_AGE_S:.0f} s; "
                    f"its belief stores the base {loaded.entry.action_id} was authored at.")
        arms = [hi.arm_joint_pose[i] for i in range(spec.n_arms)]
        return belief_from_live(spec, base_pose, arms, loaded.entry.index)

    def _redraw_other_huskies_from_progress(self) -> None:
        """Move the drawn not-connected huskies to where the progress now puts them.

        Same rule as at start-up (``husky_world._seed_viz_huskies_from_progress``,
        which reads the in-memory progress here): belief, or parked when released
        or without a belief. A husky mocap sees right now keeps its live base.
        Called after the progress changes, so the view matches the collision scene.
        """
        husky_by_name = getattr(self, 'husky_by_name', None)
        if not husky_by_name:
            return
        tracked = frozenset(name for name, husky in husky_by_name.items()
                            if self._mocap_sees_husky(husky))
        world._seed_viz_huskies_from_progress(self, self._connected_robot(), mocap_tracked=tracked)

    def reopen_entry(self, index: Optional[int] = None) -> None:
        """'Reopen entry (reload clean)' button: put the loaded entry back to pending.

        ``Progress.reopen`` undoes what marking it done recorded (its hold; the
        acting robot's belief when it came from this entry), ``progress.json`` is
        saved, and the entry is reloaded from its CLEAN export, so trajectories
        planned before are dropped from memory. Its sidecar stays on disk: this
        run keeps loading the clean export (``_clean_entries``) until the entry is
        marked done again; after a restart the sidecar is preferred again.
        Refused for an entry that is still pending, and while a queued step /
        execution still runs.

        ! When the button targets the LOADED entry but the entry slider shows a
        ! different one, it first asks for 'Confirm Exec' naming both -- the
        ! readout the operator is looking at is the slider's, not the loaded one.

        Args:
            index (int | None): Schedule index; None = the LOADED entry (the entry
                slider's when nothing is loaded).
        """
        if self._refuse_while_tasks_run('Reopen entry'):
            return
        schedule = getattr(self, '_schedule', None)
        progress = getattr(self, '_progress', None)
        if schedule is None or progress is None:
            self.get_logger().warn("No ActionSchedule loaded; nothing to reopen.")
            return
        loaded = getattr(self, '_loaded_entry', None)
        if index is not None:
            idx = int(index)
        elif loaded is not None:
            idx = loaded.index
        else:
            idx = self._selected_schedule_index()
        entry = schedule.entry(idx)
        if progress.status(idx) == STATUS_PENDING:
            self.get_logger().warn(
                f"Reopen refused: entry {idx} ({entry.action_id}) is still pending.")
            return
        slider_idx = self._selected_schedule_index()
        if index is None and loaded is not None and slider_idx != idx:
            # The operator may think they are reopening the slider's entry.
            self.tasks.append(self._reopen_entry_after_confirm(idx, slider_idx))
            return
        self._reopen_entry_now(idx)

    def _reopen_entry_after_confirm(self, idx: int, slider_idx: int) -> Generator[None, None, None]:
        """Ask for 'Confirm Exec' before reopening the LOADED entry the slider does not show.

        Args:
            idx (int): The loaded entry, which will be reopened.
            slider_idx (int): The entry the slider shows, named in the prompt.

        Yields:
            None: One yield per monitor tick while waiting.
        """
        entry = self._schedule.entry(idx)
        self._servo_abort = False
        try:
            ok = yield from world.wait_for_operator_confirm(
                self, f"Reopen the LOADED entry {idx} ({entry.action_id}), not the slider's entry "
                      f"{slider_idx}? Its recorded end state is dropped. 'Confirm Exec' to reopen, "
                      f"'Cancel Exec' to keep it done.", warn=True)
        finally:
            self._servo_abort = False
        if ok:
            self._reopen_entry_now(idx)

    def _reopen_entry_now(self, idx: int) -> None:
        """Put entry ``idx`` back to pending, save, and reload it from its clean export.

        After ``Progress.reopen`` drops the acting robot's belief (when it came
        from this entry), the belief of that robot's previous DONE entry is
        restored (``recompute_belief``). Without it, a support robot that still
        holds a bar would vanish from the other robots' collision scenes -- e.g.
        reopening B3_HR leaves Alice holding B3 with no belief, and the exported
        B3__R parks her.

        Args:
            idx (int): Schedule index of a done entry.
        """
        schedule, progress = self._schedule, self._progress
        entry = schedule.entry(idx)
        progress.reopen(entry)
        if progress.belief(entry.robot) is None:
            try:
                previous = recompute_belief(progress, schedule, entry.robot)
            except Exception as e:
                previous = None
                self.get_logger().warn(
                    f"[Schedule] could not rebuild {entry.robot}'s belief from its previous "
                    f"done entry ({e}); it falls back to the exported pose.")
            if previous is not None:
                progress.set_belief(entry.robot, previous)
                print(f"[Schedule] {entry.robot}'s belief restored from entry {previous.after_entry}.")
        try:
            path = save_progress(progress, self._problem_dir())
        except Exception as e:
            self.get_logger().error(f"[Schedule] could not save progress.json after the reopen: {e}")
            path = '(not saved)'
        self._clean_entries = getattr(self, '_clean_entries', set()) | {idx}
        print(f"[Schedule] entry {idx} {entry.action_id} reopened (pending). Saved {path}.")
        sidecar = schedule.action_path(entry)
        if sidecar != schedule.action_path(entry, prefer_sidecar=False):
            print(f"[Schedule] {os.path.basename(sidecar)} stays on disk: this run loads the "
                  f"clean export until the entry is marked done again; after a restart the "
                  f"sidecar is preferred again.")
        self._redraw_other_huskies_from_progress()
        self._rescan_schedule_flags([idx])
        self.load_schedule_entry(idx)

    def rescan_schedule_status(self) -> None:
        """'Rescan schedule status' button: re-read ``progress.json`` and every entry's flags.

        Refused while a queued step / execution still runs.
        """
        if self._refuse_while_tasks_run('Rescan schedule status'):
            return
        schedule = getattr(self, '_schedule', None)
        if schedule is None:
            return
        self._progress = load_progress(self._problem_dir(), schedule)
        self._redraw_other_huskies_from_progress()
        self._rescan_schedule_flags()
        self._print_schedule_roster()
        self.reset_ui(self.goal_arm_pose)

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
        """Open the PyBullet window, with a mesh buffer big enough for every robot we draw.

        ``pp.connect`` has no way to pass PyBullet's window options, so
        ``pybullet.connect`` is wrapped for this one call only to append
        ``--max_shape_capacity_in_bytes`` / ``--max_num_object_capacity`` (see
        ``PYBULLET_GUI_MESH_BUFFER_MB``), then restored.
        """
        extra_options = (
            f' --max_shape_capacity_in_bytes={int(self.PYBULLET_GUI_MESH_BUFFER_MB) * 1024 * 1024}'
            f' --max_num_object_capacity={256 * 1024}')
        pybullet_connect = p.connect

        def _connect_with_bigger_buffer(method, options='', **kwargs):
            """``pybullet.connect`` with the bigger window buffer appended to its options."""
            return pybullet_connect(method, options=(options or '') + extra_options, **kwargs)

        p.connect = _connect_with_bigger_buffer
        try:
            # start pybullet simulator
            pp.connect(use_gui=True, shadows=True, color=[0.9, 0.9, 1.0])
        finally:
            p.connect = pybullet_connect
        # * PyBullet's debug GUI panel (the on-screen parameter sliders) is only
        # * used by the legacy PyBulletBackend control panel. When the Dear PyGui
        # * UI is enabled, all the controls live in the separate DPG window, so we
        # * leave the debug panel off to keep the 3D view clean. We still keep the
        # * 3D window itself (use_gui=True above) so the robots stay visible.
        p.configureDebugVisualizer(
            p.COV_ENABLE_GUI,
            0 if self.USE_DPG_UI else 1,
            physicsClientId=pp.CLIENT,
        )
        
        # draw world frame
        pp.draw_pose(pp.unit_pose(), 1)
        
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
                        punch_tool_offset=[self.get_punch_tool_offset(0), self.get_punch_tool_offset(1)],
                        name=first_husky.name,  # same (calibrated) URDF as the real robot
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
                        punch_tool_offset=self.get_punch_tool_offset(0),
                        name=first_husky.name,  # same (calibrated) URDF as the real robot
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

    # --- mocap base XYZ offset controls ---
    def _build_mocap_offset_ui(self):
        """Put the base XYZ offset controls wherever the active GUI can show them.

        Which of the two placements is used depends on the primary UI backend:
          * Dear PyGui panel (USE_DPG_UI=1): the controls become the last
            section of the main window. They cannot live in their own window
            here, because a second dpg.create_context() corrupts DPG's C state
            and SEGFAULTS the process.
          * PyBullet debug GUI (USE_DPG_UI=0): a small standalone DPG window
            pops up beside the PyBullet viewer, since PyBullet's debug GUI has
            no text-entry widget of its own.

        Called from build_ui, which runs at __init__ AND on every reset_ui
        (e.g. a BarAction load), so both placements tolerate being re-entered.
        """
        if isinstance(_common._global_backend, DearPyGuiBackend):
            self._add_mocap_offset_section()
        else:
            self._init_mocap_offset_window()

    def _add_mocap_offset_section(self):
        """Append the x/y/z boxes + Apply/Reset as the last main-DPG-panel section.

        reset_ui wipes and rebuilds every main-panel widget, so this just builds
        a fresh set each time; no idempotency guard is needed.
        """
        # The old boxes are gone, so fill the new ones with the offset that is
        # actually in effect -- otherwise a rebuild would make an applied offset
        # look like it had gone back to zero.
        applied = self.huskies[self.selected_robot_id].mocap_base_offset_xyz
        self._mocap_offset_pending = [float(v) for v in applied]

        self.dump_sep_sliders.append(Separator("MOCAP base XYZ offset (world, m)"))
        self._mocap_offset_inputs = [
            TextInput(f"offset {axis} [m]",
                      lambda v, i=i: self._set_pending_offset(i, v),
                      default=self._mocap_offset_pending[i], numeric=True)
            for i, axis in enumerate(('x', 'y', 'z'))
        ]
        self.buttons.append(Button('Apply Base Offset', self._apply_base_offset))
        self.buttons.append(Button('Reset Base Offset to Zero', self._reset_base_offset))

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
        # This window is built once and survives every later reset_ui, so its
        # boxes start at zero and are never re-seeded from the applied offset.
        self._mocap_offset_pending = [0.0, 0.0, 0.0]
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
        # Blank the boxes too, in whichever of the two placements they live.
        for box in getattr(self, '_mocap_offset_inputs', []):
            box.set_value(0.0)
        dpg = getattr(self, '_offset_dpg', None)
        if dpg is not None:
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

    def _build_schedule_section(self) -> None:
        """Build the schedule panel, in place of the legacy file slider + 'Load BarAction'.

        A header (connected robot, problem, progress; where the other robots'
        poses come from), the 'Schedule entry'
        slider with its '-> entry' readout, Prev / Next / Load / Mark done /
        Reopen / Rescan, the 'Now:' line, and a collapsible list of entry rows
        around the selected entry (yellow = selected, green = done, grey = another
        robot's). ``_refresh_schedule_readouts`` keeps the texts current every
        tick; the row colours change on the next rebuild (every load / mark /
        reopen does one).
        """
        schedule = self._schedule
        n = len(schedule.entries)
        selected = max(0, min(int(self._selected_entry_idx), n - 1))
        self.schedule_header_text = StatusText("  schedule", self._schedule_header_text())
        # Never a single-value range: that segfaults pybullet's legacy slider
        # (same guard as the Movement slider).
        self.schedule_entry_slider = Slider(
            "Schedule entry (idx)",
            lambda v: setattr(self, '_selected_entry_idx', int(round(float(v)))),
            0, max(1, n - 1), selected, integer=True,
        )
        self.schedule_entry_text = StatusText("  -> entry", "")
        self.buttons.append(Button('Prev entry', self.select_prev_entry))
        self.buttons.append(Button('Next entry', self.select_next_entry))
        self.buttons.append(Button('Load entry', self.load_schedule_entry))
        self.buttons.append(Button('Mark entry done', self.mark_entry_done))
        self.buttons.append(Button('Reopen entry (reload clean)', self.reopen_entry))
        self.buttons.append(Button('Rescan schedule status', self.rescan_schedule_status))
        # * Ticked: the planner and IK ignore the built bars and joints, which are
        # * drawn faint so it is visible that the switch is on. Changing it reloads
        # * the loaded entry. Seeded from the flag so a reset_ui rebuild keeps the tick.
        self.ignore_built_toggle = Toggle(
            "Ignore built-bar collisions (bars drawn faint)",
            self.set_ignore_built_assembly_collisions,
            bool(self.IGNORE_BUILT_ASSEMBLY_COLLISIONS),
        )
        self.schedule_now_text = StatusText("  now", self._now_line())
        lo, hi = visible_row_window(n, selected, SCHEDULE_ROWS_SHOWN)
        self._schedule_row_window = (lo, hi)
        self.schedule_rows = []
        with Group(f"Schedule entries {lo}..{hi - 1} of {n}"):
            for i in range(lo, hi):
                is_selected = i == selected
                color = row_color(self._progress.status(i),
                                  executable=self._entry_is_executable_here(schedule.entry(i)),
                                  selected=is_selected)
                self.schedule_rows.append(StatusText(
                    f"  entry {i}", self._schedule_row_text(i, selected=is_selected), color=color))

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

        # * Switch the ghost between the goal conf (blue) and the planned
        # * trajectory (green, scrubbed by the slider above). Lives here
        # * because 'Traj viz time' only does anything in the trajectory view.
        self.buttons.append(Button('Toggle Goal / Trajectory view',
                                   self.toggle_show_goal_state))
        self.goal_view_text = StatusText("  -> view", "")

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

        # Visual-servoing live tracker: four per-iteration plots (tool0 position
        # error [mm], tool0 orientation error [deg], mobile-base position drift
        # [mm], mobile-base orientation drift [deg]). Fed one point per servoing
        # iteration by world.servo_to_movement_start_live via push_servoing_tracker.
        # Dear PyGui only, and only for the bar-holding accuracy experiment.
        if self.USE_DPG_UI and self.BAR_ACTION_MOCAP_ACCURACY_TEST:
            visible = getattr(self, '_servoing_tracker_visible', False)
            _common._global_backend.add_window(
                "Servoing Live Tracker", tag="servoing_tracker_window",
                width=620, height=900, show=visible)
            # Position plots carry an extra |d| euclidean-norm curve (group_size=4);
            # rotation plots are per-axis only (group_size=3). Left arm = reds,
            # right arm = greens, base = blues (see world.SERVO_*_RGB); within each
            # family the 4th shade is the |d| line.
            arm_labels_xyz = [f'{side} {ax}' for side in ('Left', 'Right')
                              for ax in ('x', 'y', 'z')]
            arm_labels_xyzd = [f'{side} {ax}' for side in ('Left', 'Right')
                               for ax in ('x', 'y', 'z', '|d|')]
            arm_pal_xyz = world.SERVO_LEFT_ARM_RGB[:3] + world.SERVO_RIGHT_ARM_RGB[:3]
            arm_pal_xyzd = world.SERVO_LEFT_ARM_RGB + world.SERVO_RIGHT_ARM_RGB
            self.servoing_pos_plot = HistoryPlot(
                "tool0 pos err", arm_labels_xyzd, "tool0 pos err [mm]",
                parent="servoing_tracker_window", group_size=4, palette=arm_pal_xyzd)
            self.servoing_rot_plot = HistoryPlot(
                "tool0 rot err", arm_labels_xyz, "tool0 rot err [deg]",
                parent="servoing_tracker_window", group_size=3, palette=arm_pal_xyz)
            self.servoing_base_pos_plot = HistoryPlot(
                "base pos diff", ['x', 'y', 'z', '|d|'], "base pos diff [mm]",
                parent="servoing_tracker_window", group_size=4,
                palette=world.SERVO_BASE_RGB)
            self.servoing_base_rot_plot = HistoryPlot(
                "base rot diff", ['x', 'y', 'z'], "base rot diff [deg]",
                parent="servoing_tracker_window", palette=world.SERVO_BASE_RGB[:3])
            for plot in (self.servoing_pos_plot, self.servoing_rot_plot,
                         self.servoing_base_pos_plot, self.servoing_base_rot_plot):
                plot.set_visible(visible)
            # These plots were just rebuilt empty (build_ui runs on every
            # reset_ui, which the servoing loop triggers). Re-draw the points
            # collected so far so the live tracker accumulates instead of
            # blanking every iteration.
            self._repopulate_servoing_tracker()
        else:
            self.servoing_pos_plot = None
            self.servoing_rot_plot = None
            self.servoing_base_pos_plot = None
            self.servoing_base_rot_plot = None

        # "Movement Preview": everything the operator should look at BEFORE
        # pressing execute on the selected movement. Filled by _accept_trajectory
        # (so both a freshly planned and a loaded-from-file trajectory land
        # here), hidden until the first trajectory arrives.
        #   1. planned joint values along the path -- every movement
        #   2. max joint step between waypoints -- bar-held movements (M1/M2)
        #   3. bar-hold EE drift                 -- bar-held movements (M1/M2)
        # Plots 2 and 3 print their pass/fail thresholds as a text line under
        # the readout. They used to draw them as flat curves, but a threshold
        # sits orders of magnitude above the typical drift and flattened the
        # very data the operator is checking.
        # ! Deliberately NOT gated on BAR_ACTION_MOCAP_ACCURACY_TEST: the
        # ! robot-centric replay demo turns that flag off but still needs these.
        if self.USE_DPG_UI and self.BAR_ACTION_LIVE_REPLAN_EXE:
            preview_visible = (getattr(self, '_preview_joint_data', None) is not None
                               or getattr(self, '_transfer_validation_data', None) is not None)
            _common._global_backend.add_window(
                "Movement Preview", tag="movement_preview_window",
                width=620, height=780, show=preview_visible)
            # Planned joint values per waypoint. Distinct from the "Joint Live
            # Stream" window, which shows what the real robot is doing now.
            # One series per arm joint of the CONNECTED robot (n_arms * 6): a
            # waypoint is a 12-vec on Cindy (left arm then right arm) and a
            # 6-vec on a support robot. A row of another length is not drawn.
            planned_labels = (PLANNED_JOINT_PLOT_LABELS
                              if self._connected_robot().n_arms == 2
                              else PLANNED_JOINT_PLOT_LABELS_SINGLE_ARM)
            self.preview_joint_plot = HistoryPlot(
                "planned joint values", planned_labels,
                "joint value [deg]", parent="movement_preview_window",
                group_size=6, history=4096)
            # max joint step [deg]; the continuity threshold is printed, not drawn.
            self.transfer_joint_step_plot = HistoryPlot(
                "joint step", ['max joint step'],
                "max joint step [deg]", parent="movement_preview_window",
                palette=[(220, 70, 70)], history=4096,
                footer=f"threshold: {TRANSFER_JOINT_STEP_THRESHOLD_DEG:.3f} deg")
            # bar-hold EE drift: translation [mm] + rotation [deg]. Drift here is
            # normally a few thousandths of a mm, so show 5 decimals.
            self.transfer_ee_drift_plot = HistoryPlot(
                "bar-hold EE drift", ['trans [mm]', 'rot [deg]'],
                "EE drift", parent="movement_preview_window",
                palette=[(70, 130, 220), (70, 200, 130)], history=4096,
                decimals=5,
                footer=(f"thresholds: trans {TRANSFER_EE_TRANS_THRESHOLD_MM:.5f} mm"
                        f" / rot {TRANSFER_EE_ROT_THRESHOLD_DEG:.5f} deg"))
            for plot in (self.preview_joint_plot, self.transfer_joint_step_plot,
                         self.transfer_ee_drift_plot):
                plot.set_visible(preview_visible)
            # These plots were just rebuilt empty (build_ui runs on every
            # reset_ui, which Load Movement triggers). Redraw the persisted
            # curves so the window survives the rebuild.
            self._draw_preview_joint_values()
            self._draw_transfer_validation()

            # "Compliant Exec Force": the live tool0 wrench DURING an M2/M3
            # compliant execution, one sample per monitor tick, x = seconds since
            # the move started. This is the signal that tells the operator what
            # the arms are actually feeling while the bar seats and the joint
            # motor bites -- the M2 hold can run up to 300 s waiting on stall,
            # and without this the only feedback is the stall flag flipping.
            # Separate window from Movement Preview on purpose: that one is the
            # BEFORE-you-press-execute review, this one is the DURING.
            wrench_visible = bool(
                (getattr(self, '_compliant_wrench_data', None) or {}).get('samples'))
            _common._global_backend.add_window(
                "Compliant Exec Force", tag="compliant_wrench_window",
                width=620, height=560, show=wrench_visible)
            # Left arm = reds, right arm = greens (same families as the servoing
            # tracker). Force and torque are split: N and Nm can't share a y axis.
            wrench_labels = [f'{side} {ax}' for side in ('L', 'R')
                             for ax in ('x', 'y', 'z')]
            wrench_palette = (world.SERVO_LEFT_ARM_RGB[:3]
                              + world.SERVO_RIGHT_ARM_RGB[:3])
            # history: the 300 s M2 stall ceiling at the ~20 Hz tick rate is
            # ~6000 samples, so size the ring buffer above that to keep a whole
            # worst-case hold on screen.
            self.compliant_force_plot = HistoryPlot(
                "tool0 force", wrench_labels, "force [N]",
                parent="compliant_wrench_window", group_size=3,
                palette=wrench_palette, history=8192)
            self.compliant_torque_plot = HistoryPlot(
                "tool0 torque", wrench_labels, "torque [Nm]",
                parent="compliant_wrench_window", group_size=3,
                palette=wrench_palette, history=8192)
            for plot in (self.compliant_force_plot, self.compliant_torque_plot):
                plot.set_visible(wrench_visible)
            self._repopulate_compliant_wrench()
        else:
            self.preview_joint_plot = None
            self.transfer_joint_step_plot = None
            self.transfer_ee_drift_plot = None
            self.compliant_force_plot = None
            self.compliant_torque_plot = None

        # self.buttons.append(Button('Reset Goal State', self.reset_ui))
                      
        # self.buttons.append(Button('Plan S.Arm to conf target', self.plan_single_arm_to_goal_action))
        # self.buttons.append(Button('Exec S.Arm Traj', self.execute_arm_trajectory))

        # # Add buttons for planning both arms to goal (sequential and composite)
        # # self.buttons.append(Button('Plan Both Arms to Goal (sequential)', lambda: world.plan_both_arms_to_goal(self, use_composite=False)))
        # self.buttons.append(Button('Plan Both Arms to Goal (composite)', self.plan_both_arms_to_goal_action))
        # self.buttons.append(Button('Exec Both Arm Trajs', lambda: world.execute_arm_trajectory_both(self)))

        # Constrained dual-arm planner + visual-servoing controls — only shown for
        # a dual-arm robot AND when the mocap-accuracy test is enabled, since these
        # buttons (live-base IK, servo loop, dual-traj export/load) only drive that
        # test's workflow.
        if (self.huskies[self.selected_robot_id].dual_arm
                and self.BAR_ACTION_MOCAP_ACCURACY_TEST):
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

            # TODO I think these two should be renamed a bit better
            # TODO one keep the old arm conf (but new base) and plan a motion from current conf to go there
            # TODO the other recompute a new ik based on the movement start conf's FK EE targets and then plan a motion from current conf to go there
            # self.buttons.append(Button('Plan Free → Mv Start (offline target)', self.plan_free_to_movement_start_with_cfab_cc))
            # * Button 1: live-base IK only — sets the goal arm pose to the
            # selected M2/M3 start EE targets (no trajectory). "Loads" the pose.
            self.buttons.append(Button(
                '1) IK Live Base → Set Mv Start Goal',
                self.ik_live_base_for_selected_movement))
            # * Button 2: live-base IK + composite free plan to the selected
            # insert's / retreat's start EE targets, in one click (produces the transit
            # trajectory → enables the traj viz slider). Uses cfab CC.
            self.buttons.append(Button(
                '2) IK Replan & Transit → Mv Start (live, insert/retreat)',
                self.replan_free_to_movement_start_live))
            # * Button 2b: same live-base IK, but the plan keeps the mounted
            # bar's rigid grasp (constrained "transfer" planner). Use this
            # once the bar is physically mounted in the grippers.
            self.buttons.append(Button(
                '2b) IK Replan & Transfer → Mv Start (bar held)',
                self.replan_transfer_to_movement_start_live))
            # Visual-servoing loop: repeat live-base IK + transit + exec until the
            # tool0 residual converges; logs each iteration and saves a static
            # matplotlib plot + JSON at the end. It pauses after planning the first
            # (long) move; preview it with the traj viz slider, then click
            # 'Confirm Servo Exec' to run it (later iterations run unattended).
            self.buttons.append(Button('3) Servo to Mv Start (live loop)',
                lambda: self.queue_servo_to_movement_start()))
            # Same servoing loop, but every iteration plans a bar-held
            # constrained transfer (Button 2b) instead of a free transit —
            # for when the bar stays mounted throughout the session.
            self.buttons.append(Button('3b) Servo to Mv Start (transfer loop)',
                lambda: self.queue_servo_to_movement_start(use_transfer=True)))

            self.buttons.append(Button(
                'Export Dual-Traj',
                self.export_constrained_dual_arm_trajectory,
            ))
            self.buttons.append(Button(
                'Load Dual-Traj',
                self.parse_constrained_dual_arm_trajectory,
            ))

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
        #     self.buttons.append(Button('Exec Free Motion', self.execute_free_trajectory))
        #     self.buttons.append(Button('Exec Linear Motion', self.execute_linear_trajectory))
        # self.buttons.append(Button('Plan arm wave', lambda: world.plan_arm_wave(self)))

        # Scaffolding tool control removed - outdated, will be remade later.

        if self.BAR_ACTION_LIVE_REPLAN_EXE:
            # self.dump_sep_sliders.append(Slider("----------BarAction live replan & exe", lambda: None))
            self.dump_sep_sliders.append(Separator("BarAction live replan & exe"))
            # Which design problem the BarAction files below belong to (the
            # folder name under DESIGN_DATA_DIRECTORY, set in __init__.py).
            self.design_problem_text = StatusText(
                "  problem", f"design problem: {os.path.basename(str(DESIGN_PROBLEM_NAME))}")
            # * A problem with a complete ActionSchedule is driven entry by entry
            # * (see _load_schedule_state); every other problem keeps the file list.
            schedule_mode = getattr(self, '_schedule', None) is not None
            if schedule_mode:
                self.bar_action_file_slider = None
                self.bar_action_file_text = None
                self._build_schedule_section()
            else:
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
                # Spells out which file that index actually is, so you can see what
                # 'Load BarAction' will open before clicking it. Created even when
                # the slider was skipped (1 file), since naming that one file is
                # exactly as useful. Kept current by _refresh_bar_action_readouts.
                self.bar_action_file_text = StatusText("  -> file", "(none)")
                self.buttons.append(Button('Load BarAction', self.load_bar_action_file))
            # * Cindy's M1 / M2 tuning widgets only apply to the assembly robot's
            # * own entries (schedule mode); the legacy list always shows them.
            # Hidden ones are set to None, which every reader of them tolerates.
            show_assembly_knobs = (not schedule_mode or knobs_for_assembly_robot(
                getattr(self, '_loaded_entry', None), self._connected_robot()))
            # The movements of the loaded action -- or of another robot's entry
            # loaded for display only (see _shown_movements).
            n_movs = len(self._shown_movements())
            # Always built, so the layout does not jump when a BarAction gets
            # loaded and 'Load Movement' always sits right under this slider.
            # Before a load it spans 0..1 with nothing behind it (the readout
            # says so and _slider_index returns -1); the range is never a
            # single value, which is what would segfault pybullet's legacy
            # GUI slider (same guard as bar_action_file_slider).
            self.bar_movement_slider = Slider(
                "Movement (index in this action file)",
                lambda v: setattr(self, '_selected_movement_idx', int(round(float(v)))),
                0, max(1, n_movs - 1),
                int(self._selected_movement_idx),
                integer=True,
            )
            # Same idea for the movement index: show the movement_id it picks.
            self.bar_movement_text = StatusText("  -> movement", "(none)")
            self.buttons.append(Button('Load Movement', self.load_selected_movement))
            if show_assembly_knobs:
                # Transfer derived-start carry anchor selector (see M1_HOME_ANCHOR_CHOICES).
                # Fixed 0..3 range -> always >=2 entries, so the 1-entry segfault
                # guard the sliders above need does not apply here.
                self.m1_home_anchor_slider = Slider(
                    "Transfer start: home anchor (0:all,1:horiz,2:vert,3:back)",
                    self._on_m1_anchor_slider,
                    0, len(M1_HOME_ANCHOR_CHOICES) - 1,
                    int(self._m1_home_anchor_idx),
                    integer=True,
                )
                # * Manual transfer start (human in the loop): adjust the anchor's bar
                # * pose -- a see-through orange bar follows the sliders in
                # * PyBullet -- then Confirm runs the collision-checked IK and adopts it.
                # * Which base axes the two perpendicular shifts are depends on the
                # * anchor (printed on Confirm; table in bar_holding_acc_manual.md).
                self.m1_manual_slide_slider = Slider(
                    "Transfer start: slide along bar (m)",
                    lambda v: self._on_m1_manual_slider('_m1_manual_slide_m', v),
                    -M1_MANUAL_SHIFT_RANGE_M, M1_MANUAL_SHIFT_RANGE_M, float(self._m1_manual_slide_m))
                self.m1_manual_roll_slider = Slider(
                    "Transfer start: roll about bar (deg)",
                    lambda v: self._on_m1_manual_slider('_m1_manual_roll_deg', v),
                    -M1_MANUAL_ROLL_RANGE_DEG, M1_MANUAL_ROLL_RANGE_DEG, float(self._m1_manual_roll_deg))
                self.m1_manual_perp1_slider = Slider(
                    "Transfer start: shift perp. 1 (m)",
                    lambda v: self._on_m1_manual_slider('_m1_manual_perp1_m', v),
                    -M1_MANUAL_SHIFT_RANGE_M, M1_MANUAL_SHIFT_RANGE_M, float(self._m1_manual_perp1_m))
                self.m1_manual_perp2_slider = Slider(
                    "Transfer start: shift perp. 2 (m)",
                    lambda v: self._on_m1_manual_slider('_m1_manual_perp2_m', v),
                    -M1_MANUAL_SHIFT_RANGE_M, M1_MANUAL_SHIFT_RANGE_M, float(self._m1_manual_perp2_m))
                self.buttons.append(Button('Transfer start: Confirm manual pose (IK check)',
                                           self.confirm_m1_manual_start))
            else:
                self.m1_home_anchor_slider = None
                self.m1_manual_slide_slider = None
                self.m1_manual_roll_slider = None
                self.m1_manual_perp1_slider = None
                self.m1_manual_perp2_slider = None
            self.buttons.append(Button('Plan Movement', self.plan_selected_movement))
            if show_assembly_knobs:
                # * The transfer start in two clicks without the RRT: derive + show
                # the start/goal confs, then adopt the start as the transfer's
                # start / the travel to load's goal.
                self.buttons.append(Button('Transfer start: Derive start/goal only (no RRT)',
                                           self.derive_m1_endpoints_live))
                self.buttons.append(Button('Adopt derived start -> travel-to-load goal',
                                           self.adopt_m1_derived_start))
                # * Ticked: adopting ALSO writes the travel to load's and the
                # * transfer's configurations into the BarAction file, so a session
                # * that exits can reload and plan the travel to load without
                # * deriving the transfer start again. Only meaningful while the base
                # * has not moved -- see _save_m1_m0_confs_to_bar_action_file.
                # Seeded from the flag so a reset_ui rebuild keeps the tick.
                self.m1_adopt_save_toggle = Toggle(
                    "Adopt also saves travel-to-load / transfer confs to file",
                    lambda v: setattr(self, '_m1_adopt_writes_file', bool(v)),
                    bool(self._m1_adopt_writes_file),
                )
            else:
                self.m1_adopt_save_toggle = None
            self.buttons.append(Button('Load Movement Trajectory', self.load_selected_movement_trajectory))
            # * Button 1: plan transfer -> insert -> retreat -> travel to load ->
            # free move home in one click,
            # export the mutated action as `<name>.live-solved.json` sidecar.
            self.buttons.append(Button('Plan Chain (Live)', self.plan_movement_chain_live))
            if show_assembly_knobs:
                # * Persist the live-planned travel to load so the next session can
                # load it instead of re-planning. Writes the whole action back to the
                # file that is loaded (never to a clean Rhino export -- see the method).
                self.buttons.append(Button(
                    'Save travel-to-load plan to BarAction file',
                    self.export_m0_plan_to_bar_action_file))
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

            # # self.dump_sep_sliders.append(Slider("---------- live movement debug", lambda: None))
            # self.dump_sep_sliders.append(Separator("live movement debug"))

            # self.dump_sep_sliders.append(Slider("---------- movement exe", lambda: None))
            self.dump_sep_sliders.append(Separator("movement exe"))

            # * The single execute-movement button. Auto-dispatch by kind:
            # insert / retreat -> cartesian_compliance_controller, else joint tracking.
            self.buttons.append(Button(
                'Exec Selected Mv Traj (auto)',
                self.exec_selected_movement_traj))
            # * Schedule mode: the ONE button that runs a step where no arm moves
            # * (gripper / manual / scaffolding tool), labelled with what it does.
            # * Only built for such a step of the connected robot's own entry.
            step_label = (step_button_label(self.current_movement)
                          if schedule_mode and self._entry_is_executable_here() else None)
            if step_label is not None:
                self.buttons.append(Button(step_label, self.exec_selected_movement_step))

            # ! Shared confirm/cancel pair (world.wait_for_operator_confirm).
            # ! These MUST live here, next to the execute button, not in the
            # ! mocap-accuracy block: M3 pauses for confirmation mid-execution,
            # ! and with BAR_ACTION_MOCAP_ACCURACY_TEST=0 that block is not
            # ! built -- the pause would have had no way to be answered.
            # Also used by the servoing loop's own confirm pauses.
            self.buttons.append(Button('Confirm Exec',
                lambda: setattr(self, '_servo_exec_confirmed', True)))
            # Stop at the next yield (a confirm pause / between servo iterations).
            # A trajectory already sent to the robot still finishes; this only
            # prevents anything further being sent.
            self.buttons.append(Button('Cancel Exec',
                lambda: setattr(self, '_servo_abort', True)))

            if show_assembly_knobs:
                # The insert hands over from the rigid joint controller to compliance this
                # far short of the assembled pose. Read live at execution time.
                self.m2_split_slider = Slider(
                    "Insert: rigid->compliant split (mm to goal)",
                    lambda v: setattr(self, 'm2_compliant_split_mm', float(v)),
                    0.0, M2_COMPLIANT_SPLIT_MM_MAX, float(self.m2_compliant_split_mm),
                )
                # 1 = run all of the insert rigid, never engaging compliance (the split
                # slider above is then ignored).
                self.m2_rigid_only_slider = Slider(
                    "Insert exec (0:rigid+compliant split, 1:rigid only)",
                    lambda v: setattr(self, 'm2_exec_rigid_only', bool(round(float(v)))),
                    0, 1, int(bool(self.m2_exec_rigid_only)), integer=True,
                )
            else:
                self.m2_split_slider = None
                self.m2_rigid_only_slider = None
            # 0 disables the dense swept collision re-check that gates the free moves'
            # plans. Faster planning, unverified paths.
            self.fm_swept_validation_slider = Slider(
                "Free moves: swept collision check (0:off, 1:on)",
                lambda v: setattr(self, 'fm_swept_validation_enabled',
                                  bool(round(float(v)))),
                0, 1, int(bool(self.fm_swept_validation_enabled)), integer=True,
            )

            self.buttons.append(Button(
                'Move Arms to Movement Start (offline target)',
                self.move_arms_to_movement_start))

        if self.BAR_ACTION_MOCAP_ACCURACY_TEST:
            self.buttons.append(Button('Record markerset take', self.record_bar_holding_marker_take))
            self.buttons.append(Button('Record + Fit + Viz (shared)', self.record_bar_take_with_shared_viz))
            self.buttons.append(Button('Save markerset data', self.save_bar_holding_marker_data))
            # Drops unsaved takes (e.g. the post-mount fit check) so they never
            # ride into the next bar's saved file.
            self.buttons.append(Button('Discard unsaved takes', self.discard_bar_holding_marker_takes))
            self.buttons.append(Button('Toggle Servoing Tracker', self.toggle_servoing_tracker))

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
                    interface.send_scaffolding_cmd(direction, GRIPPER_MOTOR, arm_index)
                    interface.send_scaffolding_cmd(direction, JOINT_MOTOR, arm_index)

                def send_scaffolding_cmd_motor(direction, motor, arm_index):
                    self.huskies[self.selected_robot_id].interface.send_scaffolding_cmd(direction, motor, arm_index)

                if has_scaffold_left:
                    self.buttons.append(Button('- L Stop All', lambda: send_scaffolding_cmd_both_motors(0, 0)))
                    self.buttons.append(Button('- L Tighten Gripper', lambda: send_scaffolding_cmd_motor(1, GRIPPER_MOTOR, 0)))
                    self.buttons.append(Button('- L Loosen Gripper', lambda: send_scaffolding_cmd_motor(-1, GRIPPER_MOTOR, 0)))
                    self.buttons.append(Button('- L Tighten Joint', lambda: send_scaffolding_cmd_motor(1, JOINT_MOTOR, 0)))
                    self.buttons.append(Button('- L Loosen Joint', lambda: send_scaffolding_cmd_motor(-1, JOINT_MOTOR, 0)))

                if has_scaffold_right and active_husky.dual_arm:
                    self.buttons.append(Button('- R Stop All', lambda: send_scaffolding_cmd_both_motors(0, 1)))
                    self.buttons.append(Button('- R Tighten Gripper', lambda: send_scaffolding_cmd_motor(1, GRIPPER_MOTOR, 1)))
                    self.buttons.append(Button('- R Loosen Gripper', lambda: send_scaffolding_cmd_motor(-1, GRIPPER_MOTOR, 1)))
                    self.buttons.append(Button('- R Tighten Joint', lambda: send_scaffolding_cmd_motor(1, JOINT_MOTOR, 1)))
                    self.buttons.append(Button('- R Loosen Joint', lambda: send_scaffolding_cmd_motor(-1, JOINT_MOTOR, 1)))



        # self.dump_sep_sliders.append(Slider("----------DEBUG utils", lambda : None))
        self.dump_sep_sliders.append(Separator("DEBUG utils"))
        self.buttons.append(Button('Sample Random Goal Conf', self.sample_random_goal_conf))
        self.buttons.append(Button('Remove all drawing', self.clear_all_debug_drawing))
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
            self._build_mocap_offset_ui()

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
            # ! An all-zero position is how an untracked body comes through (NatNet
            # ! calls this before it parses tracking_valid), so it must not count
            # ! as a fresh sighting.
            if np.any(pos):
                self._mocap_rigidbody_stamp[name] = time.monotonic()
    
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

        for i, h in enumerate(self.huskies):
            if h.name not in raw_snapshot:
                continue
            # ! The cache is never cleared: an OTHER (not connected) husky follows
            # ! mocap only while it is really seen, else it stays where
            # ! progress.json put it -- the same freshness rule its collision
            # ! obstacle uses, so the drawing matches the planning scene.
            if i != self.selected_robot_id and not self._mocap_sees_husky(h):
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
        # Display-only, so it is refreshed here rather than polled. Outside the
        # BAR_ACTION_LIVE_REPLAN_EXE block below: the goal/trajectory view
        # exists whether or not that workflow is switched on.
        self._refresh_goal_view_readout()

        # Scaffolding-tool live status overlay removed - outdated, will be remade later.

        # update tracked objects
        for i, o in enumerate(self.tracked_objects):
            o.set_pose((o.pos, o.rot))
        
        # update robot state
        for i, h in enumerate(self.huskies):
            hi = h.interface
            if i != self.selected_robot_id:
                # * A husky this run does not drive (viz-only): always drawn where
                # * mocap puts it -- until mocap sees it, at its progress.json
                # * belief or else parked far away (husky_world seeds both). It
                # * never follows or changes the goal base pose.
                h.object.set_pose((hi.position, hi.rotation), hi.arm_joint_pose)
            elif self._base_pose_is_tracked():
                # mocap drives the husky base pose
                h.object.set_pose((hi.position, hi.rotation), hi.arm_joint_pose)
                # set the goal pose of base since we are teleoperating the base
                if not self.goal_base_pose_frozen:
                    self.goal_base_pose = (hi.position, hi.rotation)
            else:
                # Base is whatever the cell state set (or the sliders set). This
                # is also the robot-centric replay case (USE_MOCAP=0): nothing
                # measures the base, so the plan's authored base pose IS the
                # base pose. Keep this in step with _live_base_pose(), which the
                # trajectory preview below uses for the same reason.
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

        if self.BAR_ACTION_LIVE_REPLAN_EXE:
            if hasattr(self, 'bar_action_file_slider') and self.bar_action_file_slider:
                self.bar_action_file_slider.update()
            if hasattr(self, 'bar_movement_slider') and self.bar_movement_slider:
                self.bar_movement_slider.update()
            if hasattr(self, 'm1_home_anchor_slider') and self.m1_home_anchor_slider:
                self.m1_home_anchor_slider.update()
            for name in ('m1_manual_slide_slider', 'm1_manual_roll_slider',
                         'm1_manual_perp1_slider', 'm1_manual_perp2_slider'):
                sld = getattr(self, name, None)
                if sld:
                    sld.update()
            self._poll_m1_manual_preview()
            self._redraw_assembled_bar_line()
            if hasattr(self, 'm1_adopt_save_toggle') and self.m1_adopt_save_toggle:
                self.m1_adopt_save_toggle.update()
            sld = getattr(self, 'schedule_entry_slider', None)
            if sld:
                sld.update()
            tgl = getattr(self, 'ignore_built_toggle', None)
            if tgl:
                tgl.update()
            # Display-only, so it is refreshed here rather than polled.
            self._refresh_bar_action_readouts()
            self._refresh_schedule_readouts()
            if hasattr(self, 'm2_split_slider') and self.m2_split_slider:
                self.m2_split_slider.update()
            if hasattr(self, 'm2_rigid_only_slider') and self.m2_rigid_only_slider:
                self.m2_rigid_only_slider.update()
            if hasattr(self, 'fm_swept_validation_slider') and self.fm_swept_validation_slider:
                self.fm_swept_validation_slider.update()

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
            # Trajectory preview rides on the base pose the arms will actually
            # be executed from: the mocap-tracked pose when mocap drives the
            # base, otherwise the plan's authored base (see _live_base_pose).
            # The arm conf below comes from the planned trajectory; pairing it
            # with that same base is what makes the preview match execution --
            # and it is what keeps the goal ghost and the live pp husky in the
            # same frame, which the compliant M2/M3 exec relies on (it FKs its
            # cartesian targets off the ghost and its arm-base off the live one).
            if self.huskies:
                goal_base_pose = self._live_base_pose()
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
            # Remember which task is running: code inside it may call a method
            # that refuses while tasks run (e.g. a confirmed reopen reloads its
            # entry), and that task must not count as "another" task.
            self._running_task = t
            try:
               next(t)
            except StopIteration:
                self.tasks.remove(t)
            finally:
                self._running_task = None
                
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
            print("No constrained dual-arm trajectory to export. Plan the transfer movement first.")
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
