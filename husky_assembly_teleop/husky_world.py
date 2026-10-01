"""
This module contains the world definition and high level actions or sequences of actions for the huskies.
"""

import os, time
import contextlib
from typing import TYPE_CHECKING, Generator, Optional
import asyncio.runners
import asyncio
from matplotlib.pyplot import bar
import numpy as np
import copy
from husky_assembly_teleop.husky_robot import GRIPPER_MOTOR, JOINT_MOTOR, HuskyRobotInterface
import rclpy

import pybullet as p
import pybullet_planning as pp

from husky_assembly_teleop import DATA_DIRECTORY, CALIBRATION_DATE, EXPERIMENT_DATA_DIRECTORY, CALIBRATION_DATA_DIRECTORY, DESIGN_PROBLEM_NAME
from husky_assembly_teleop.common import Husky, TrackedObject, AssemblyObject
from husky_assembly_teleop.robot_registry import RobotSpec, other_robots, robot_from_env
from husky_assembly_teleop.bar_action_io import COMPLIANT_KINDS, MovementKind, tool_event
from husky_assembly_teleop.schedule_io import load_schedule, problem_root
from husky_assembly_teleop.progress_io import PARKED_BASE_FRAME, RobotBelief, load_progress
import husky_assembly_teleop.husky_planning as planning
import husky_assembly_teleop.husky_control as control
from husky_assembly_teleop.utils import HUSKY_DUAL_UR5e_JOINT_NAMES, HUSKY_DUAL_ARM_HOME_CONF_12, UR5E_JOINT_NAMES, MOCAP_SET_RIG_RB_NAME, conf_from_12vec, get_arm_ik_for_grasp_bar, get_custom_limits, notify, plan_transit_motion, pose_from_frame
from husky_assembly_teleop.scaffolding import parse_mt_geometric, create_collision_bodies, create_couplers, flatten_list
from husky_assembly_teleop.cfab_session import CfabSession, build_default_robot_cell
from husky_assembly_teleop.cc_diagnosis import visualize_goal_ik_collision
import json
from datetime import datetime

from compas_fab.backends import CollisionCheckError, InverseKinematicsError, PlanningGroupNotSupported
from compas_fab.robots import FrameTarget, RobotCellState, TargetMode
from compas_robots import Configuration
from rs_data_structure.bar_action import GripperToolMovement, ManualMovement, ScaffoldingToolMovement

import matplotlib.pyplot as plt
import compas

import cv2

if TYPE_CHECKING:
    # Only for the type hints: husky_monitor imports this module, so importing
    # it for real here would be circular.
    from husky_assembly_teleop.husky_monitor import HuskyMonitor

assembly_objects = []

# Use the centralized DATA_DIRECTORY from the package
DATA_DIR = DATA_DIRECTORY

# Dated calibration datasets live on the gdrive (see CALIBRATION_DATA_DIRECTORY).
CALIB_DATA_DIR = CALIBRATION_DATA_DIRECTORY
BAR_HOLDING_ACC_DATA_DIR = os.path.join(DATA_DIR, "bar_holding_acc_data")
BAR_HOLDING_ACC_EXPERIMENT_DIR = os.path.join(EXPERIMENT_DATA_DIRECTORY, "bar_holding_acc_data")

# Servoing tracker line colors (RGB 0-255), shared by the DPG live tracker
# (husky_monitor build_ui) and the static plot. Per family the order is
# x, y, z, |d| (the euclidean-norm curve), shaded light -> dark so |d| reads as
# the bold summary line. Left arm = reds, right arm = greens, mobile base = blues.
SERVO_LEFT_ARM_RGB = [(240, 130, 120), (225, 70, 55), (150, 20, 20), (90, 0, 0)]     # x, y, z, |d|
SERVO_RIGHT_ARM_RGB = [(150, 220, 140), (70, 180, 80), (20, 130, 40), (0, 70, 20)]   # x, y, z, |d|
SERVO_BASE_RGB = [(150, 190, 230), (70, 130, 200), (30, 80, 160), (10, 40, 100)]     # x, y, z, |d|
DUAL_ARM_ACC_DATA_DIR = os.path.join(DATA_DIR, "dual_arm_acc_data")
# The config.yaml template stays in the repo, next to the analysis scripts.
CALIB_CONFIG_TEMPLATE = os.path.join(DATA_DIR, "calibration_data", "_data_template", "config.yaml")

# Kissing experiment constants (ported from c81e373)
KISSING_DATA_DIR = os.path.join(DATA_DIR, "kissing_experiment_data")
Z_MOVE_TO_INSERT = 0.035
CARTESIAN_SPEEDUP = 5
TIME_PER_ROTATION = 14
PROBE_END_WAIT_TIME = 1

# BarAction planner hyperparameters. Constrained resolution controls SE(3)
# RRT interpolation; free joint resolution controls joint-space extension.
# CONSTRAINED_POSITION_RES = 0.1
# CONSTRAINED_ROTATION_RES = 0.1
CONSTRAINED_POSITION_RES = 0.005
CONSTRAINED_ROTATION_RES = 0.017
FREE_JOINT_RESOLUTION = 0.05


def arm_index_to_name(arm_index):
    return "left" if int(arm_index) == 0 else "right"


def get_runtime_arm_name(dual_arm, arm_index):
    if not dual_arm:
        return "single"
    return arm_index_to_name(arm_index)


def _ensure_calibration_conf(monitor, folder_path):
    """Create a folder-local config.yaml from the calibration template if needed."""
    conf_path = os.path.join(folder_path, "config.yaml")
    if os.path.exists(conf_path):
        return

    with open(CALIB_CONFIG_TEMPLATE, "r") as f:
        config_text = f.read()

    husky = monitor.huskies[monitor.selected_robot_id]
    robot_name = husky.name.split("_")[-1].lstrip("/") if husky.name else str(monitor.selected_robot_id)
    arm_name = arm_index_to_name(monitor.selected_arm_index)

    import re

    config_text = re.sub(r'(^robot_name:\s*)".*?"', rf'\1"{robot_name}"', config_text, flags=re.MULTILINE)
    config_text = re.sub(r'(^arm:\s*)".*?"', rf'\1"{arm_name}"', config_text, flags=re.MULTILINE)

    with open(conf_path, "w") as f:
        f.write(config_text)


def _warn_available_calib_tools(monitor, missing_tool_name):
    """Log configured calibration tools and suggest an arm switch when applicable."""
    robot_id = int(monitor.selected_robot_id)
    current_arm_index = int(monitor.selected_arm_index)
    tool_map = monitor.calib_tool_from_robot_arm_id[robot_id]
    mocap_cache = getattr(monitor, "_mocap_rigidbody_cache", {}) or {}

    configured_tools = []
    for arm_index in sorted(tool_map.keys()):
        tool_name = tool_map[arm_index]
        if tool_name:
            in_cache = tool_name in mocap_cache
            configured_tools.append(f"arm {arm_index}: '{tool_name}' (in mocap cache: {in_cache})")

    if configured_tools:
        monitor.get_logger().warn(
            f"Configured calibration tools for robot {robot_id}: {', '.join(configured_tools)}"
        )
    else:
        monitor.get_logger().warn(f"No calibration tool is configured for robot {robot_id}.")

    for arm_index in sorted(tool_map.keys()):
        tool_name = tool_map[arm_index]
        if not tool_name or arm_index == current_arm_index:
            continue
        if tool_name in mocap_cache:
            monitor.get_logger().warn(
                f"Requested tool '{missing_tool_name}' is missing, but arm {arm_index} tool "
                f"'{tool_name}' is present in the mocap cache. Consider changing "
                f"selected_arm_index from {current_arm_index} to {arm_index}."
            )
            return

def create_husky_with_end_effectors(monitor, name, mocap_id=None, pos=np.zeros(3), rot=np.array((0, 0, 0, 1)),
                                   connect_arm=True, connect_gripper=True, base_calibration_file=None,
                                   calibration=False, dual_arm=False, ee_types=None, force_regenerate=False,
                                   punch_tool_offset=None, connect_compliant_controller=False,
                                   connect_ros: bool = True):
    """
    Helper function to create a Husky robot with specified end effectors.

    Args:
        monitor: The monitor instance
        name: Robot name
        mocap_id: Mocap ID for tracking
        pos: Initial position
        rot: Initial rotation
        connect_arm: Whether to connect to arm hardware
        connect_gripper: Whether to connect to gripper hardware
        base_calibration_file: Path to base calibration file
        calibration: Whether this is for calibration (uses calib_tip)
        dual_arm: Whether this is a dual-arm robot
        ee_types: List of end effector types. Options:
                 - "assembly_tool_v3_left": Assembly tool v3 (left variant mesh)
                 - "assembly_tool_v3_right": Assembly tool v3 (right variant mesh)
                 - "robotiq_gripper": Robotiq gripper
                 - "custom_gripper": Custom gripper (example)
                 - "punch_tool": Punch tool for calibration validation
                 - "validation_tool_pair": Validation tool pair (PointTool and BoardTool)
                 - "calib_tip": Calibration tip
                 For dual-arm robots, provide a list of two types.
                 For single-arm robots, provide a list of one type.
                 If None, defaults to assembly_tool_v3_left/right or calib_tip based on calibration flag.
        force_regenerate: Force regeneration of URDF cache (only used for validation_tool_pair)
        punch_tool_offset: numpy array [x, y, z] offset from tool0 to punch tip (only used for punch_tool)
        connect_compliant_controller: Whether to create the compliant-controller ROS clients
        connect_ros (bool): False for a husky that is only drawn (fed by mocap):
            no ROS subscriptions, publishers or clients at all.

    Returns:
        Husky: The created (and registered) husky.
    """
    if ee_types is None:
        if calibration:
            ee_types = ["calib_tip"]
        elif dual_arm:
            ee_types = ["assembly_tool_v3_left", "assembly_tool_v3_right"]
        else:
            ee_types = ["assembly_tool_v3_left"]

    return Husky(monitor, name=name, mocap_id=mocap_id, pos=pos, rot=rot,
                connect_arm=connect_arm, connect_gripper=connect_gripper,
                base_calibration_file=base_calibration_file, calibration=calibration,
                dual_arm=dual_arm, ee_types=ee_types, force_regenerate=force_regenerate,
                punch_tool_offset=punch_tool_offset,
                connect_compliant_controller=connect_compliant_controller,
                connect_ros=connect_ros)

def _base_calibration_file_for(monitor, spec: RobotSpec) -> Optional[str]:
    """The base (mocap) calibration file of one husky, or None when there is none.

    When MOCAP_AXIS_CONVENTION='rhino', prefer a `_rhino`-tagged calibration
    file (generated by data/calibration_data/convert_to_rhino.py). The values
    are identical to the legacy file (calibration is convention-invariant);
    the tag just makes the active convention explicit. Falls back to the
    untagged file if the tagged one is missing.

    Args:
        monitor: The HuskyMonitor (reads MOCAP_AXIS_CONVENTION, logs warnings).
        spec (RobotSpec): The husky.

    Returns:
        str | None: Path of the calibration JSON, or None (with a warning) when
        no file exists -- the husky is then drawn without base calibration.
    """
    convention = getattr(monitor, 'MOCAP_AXIS_CONVENTION', 'rotated')
    base_calibration_file = os.path.join(
        CALIB_DATA_DIR, CALIBRATION_DATE, spec.base_calibration_filename(convention))
    if os.path.exists(base_calibration_file):
        return base_calibration_file
    fallback = os.path.join(CALIB_DATA_DIR, CALIBRATION_DATE, spec.base_calibration_filename())
    if convention == 'rhino' and os.path.exists(fallback):
        monitor.get_logger().warn(
            f'Rhino-tagged calibration not found ({base_calibration_file}); '
            f'falling back to {fallback}. Run data/calibration_data/convert_to_rhino.py '
            'to silence this warning.'
        )
        return fallback
    monitor.get_logger().warn(
        f'Base calibration file not found for robot {spec.serial}: {base_calibration_file}. '
        'Continuing without base calibration.'
    )
    return None


def create_registry_huskies(monitor, connected: RobotSpec, *, ee_types: list,
                            punch_offset=None, connect_ros: bool = True) -> list:
    """Create one Husky per registry robot: the connected one first, then the others viz-only.

    * The connected robot is created FIRST, so it is ``monitor.huskies[0]`` and
    * ``monitor.selected_robot_id = 0`` keeps pointing at it everywhere. The other
    * two huskies get no ROS connection at all; they are drawn in the viewer and
    * moved by mocap (their mocap ids are registered like the connected one's).

    Args:
        monitor: The HuskyMonitor (``add_husky`` registers each husky on it).
        connected (RobotSpec): The robot this monitor run drives.
        ee_types (list): End-effector meshes of the CONNECTED robot (the punch /
            calibration modes override its registry default). The others use
            their registry ``ee_types``.
        punch_offset: Punch-tool offset(s) of the connected robot, or None.
        connect_ros (bool): Whether the connected robot connects to ROS. Only
            False for offline checks that build the world without ROS.

    Returns:
        list: The created Husky objects, connected robot first.
    """
    fake_hardware = bool(monitor.FAKE_HARDWARE)
    compliant = bool(getattr(monitor, 'CONNECT_COMPLIANT_CONTROLLER', 0))
    huskies = []
    for spec in [connected] + other_robots(connected.name):
        is_connected = spec.name == connected.name
        huskies.append(create_husky_with_end_effectors(
            monitor,
            name=spec.namespace,
            mocap_id=spec.mocap_id,
            pos=np.array((0, 0, 0)),
            connect_arm=is_connected and not fake_hardware,
            connect_gripper=is_connected and spec.connect_gripper and not fake_hardware,
            connect_compliant_controller=is_connected and compliant and not fake_hardware,
            connect_ros=is_connected and connect_ros,
            calibration=monitor.CALIBRATION if is_connected else False,
            dual_arm=spec.dual_arm,
            ee_types=list(ee_types) if is_connected else list(spec.ee_types),
            base_calibration_file=_base_calibration_file_for(monitor, spec),
            force_regenerate=False,
            punch_tool_offset=punch_offset if is_connected else None,
        ))
    monitor.connected_robot = connected
    monitor.husky_by_name = {
        spec.name: husky
        for spec, husky in zip([connected] + other_robots(connected.name), huskies)}
    assert monitor.huskies[0].name == connected.namespace, (
        f"the connected robot {connected.name} must be huskies[0], "
        f"got {monitor.huskies[0].name}")
    return huskies


def _seed_viz_huskies_from_progress(monitor, connected: RobotSpec, *,
                                    mocap_tracked: frozenset = frozenset()) -> None:
    """Draw the not-connected huskies where the schedule progress puts them.

    Same rule as the collision scene (``progress_io.obstacle_tool_states``): a
    husky that released its hold (``Progress.released_robots``) has left the
    scene and a husky without a belief is unknown, so both go to the far-away
    parked spot (``progress_io.PARKED_BASE_FRAME``) -- never at the world origin
    on top of the connected robot. A husky with a belief is moved to it (base
    and arm joints). Mocap draws the base as soon as it tracks a husky: one in
    ``mocap_tracked`` keeps its base and only takes the belief's arm joints.

    Called at start-up (reads ``progress.json``) and by the monitor whenever the
    progress changes (then its in-memory ``_progress`` is used).

    ! Never raises: a missing, unreadable or malformed schedule / progress file
    ! (or a monitor without huskies, headless) only logs a warning, so it can not
    ! stop the monitor from starting nor the tick that marks an entry done.

    Args:
        monitor: The HuskyMonitor (after create_registry_huskies).
        connected (RobotSpec): The connected robot (not seeded: it is live).
        mocap_tracked (frozenset): Names of the other robots mocap sees right now.
    """
    parked_pos, parked_quat = pose_from_frame(PARKED_BASE_FRAME)
    try:
        others = [(spec, monitor.husky_by_name[spec.name].interface)
                  for spec in other_robots(connected.name)]
        for spec, hi in others:
            if spec.name not in mocap_tracked:
                # Rebind (never write in place): see Husky.__init__.
                hi.position = np.array(parked_pos, dtype=float)
                hi.rotation = np.array(parked_quat, dtype=float)
        progress = getattr(monitor, '_progress', None)
        if progress is None:
            root = problem_root(DESIGN_PROBLEM_NAME)
            schedule = load_schedule(root)
            progress = load_progress(root, schedule) if schedule is not None else None
        if progress is None:
            return
        released = progress.released_robots()
        for spec, hi in others:
            belief = progress.belief(spec.name)
            if spec.name in released:
                print(f"[world] {spec.name} released its hold: drawn parked until mocap sees it.")
                continue
            if belief is None:
                continue
            _pose_husky_at_belief(hi, spec, belief, keep_base=spec.name in mocap_tracked)
            where = "base from mocap" if spec.name in mocap_tracked else "until mocap sees it"
            print(f"[world] {spec.name} drawn at its {belief.source} belief "
                  f"(after entry {belief.after_entry}; {where}).")
    except Exception as e:
        monitor.get_logger().warn(
            f"Could not read the schedule / progress of {DESIGN_PROBLEM_NAME}: {e}; "
            "the other huskies stay parked until mocap sees them.")


def _pose_husky_at_belief(hi, spec: RobotSpec, belief: RobotBelief, *, keep_base: bool) -> None:
    """Move one husky's interface to a belief: arm joints, and the base unless kept.

    Args:
        hi: The husky's interface (``position``, ``rotation``, ``arm_joint_pose``).
        spec (RobotSpec): The husky's robot (its per-arm joint names).
        belief (RobotBelief): Where it is believed to be.
        keep_base (bool): True when something else (mocap) drives the base.
    """
    pos, quat = pose_from_frame(belief.base_frame)
    # Read every joint before changing anything, so a malformed belief
    # leaves this husky where it was instead of half-moved.
    arm_joint_pose = [np.array([belief.configuration[n] for n in names], dtype=float)
                      for names in spec.arm_joint_names]
    if not keep_base:
        # Rebind (never write in place): see Husky.__init__.
        hi.position = np.array(pos, dtype=float)
        hi.rotation = np.array(quat, dtype=float)
    hi.arm_joint_pose = arm_joint_pose


def _seed_connected_husky_from_progress(monitor, connected: RobotSpec) -> None:
    """Start the connected robot where its own progress.json belief left it.

    * In fake hardware nothing else ever sets the simulated arms, so this is
    * the state the simulation continues from (e.g. Cindy still holding B3
    * after Alice's run). On the real robot the first ``joint_states`` message
    * overwrites the arms. The base is set only when mocap does not drive it;
    * the drawing then reads ``monitor.goal_base_pose``, so that is set too
    * (the first 'Load Movement' replaces it with the movement's base).

    Only in schedule mode (the monitor's in-memory ``_progress`` is set by
    ``_load_schedule_state``); does nothing when the robot has no belief yet.

    Args:
        monitor: The HuskyMonitor (after create_registry_huskies).
        connected (RobotSpec): The connected robot.
    """
    progress = getattr(monitor, '_progress', None)
    belief = progress.belief(connected.name) if progress is not None else None
    if belief is None:
        return
    tracked = monitor._base_pose_is_tracked()
    hi = monitor.huskies[monitor.selected_robot_id].interface
    _pose_husky_at_belief(hi, connected, belief, keep_base=tracked)
    if not tracked:
        monitor.goal_base_pose = (hi.position, hi.rotation)
    monitor.get_logger().info(
        f"[Schedule] {connected.name} starts from the progress.json state "
        f"(after entry {belief.after_entry}, {belief.source}).")


def init(monitor):
    # * The connected robot comes from ROS_DOMAIN_ID (robot_registry: 84 Alice
    # * 0804, 85 Belle 0805, 86 Cindy 0806), so each robot can run in its own
    # * terminal without editing this file. All three huskies are created; only
    # * the connected one talks to ROS, the other two are viz-only and mocap-tracked.
    spec, was_default = robot_from_env()
    if was_default and 'ROS_DOMAIN_ID' in os.environ:
        monitor.get_logger().warn(
            f"ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']!r} names no robot in the "
            f"robot registry; defaulting to {spec.name} ({spec.serial})."
        )

    robot_name = spec.serial
    dual_arm = spec.dual_arm

    # Determine ee_types based on active mode
    if monitor.PUNCH_CALIB_VALIDATION:
        ee_types = ["punch_tool", "punch_tool"] if dual_arm else ["punch_tool"]
        punch_offset = (
            [monitor.get_punch_tool_offset(0), monitor.get_punch_tool_offset(1)]
            if dual_arm else monitor.get_punch_tool_offset(0)
        )
    elif monitor.CALIBRATION:
        ee_types = ["custom_gripper", "custom_gripper"] if dual_arm else ["custom_gripper"]
        punch_offset = None
    else:
        ee_types = list(spec.ee_types)
        punch_offset = None

    create_registry_huskies(monitor, spec, ee_types=ee_types, punch_offset=punch_offset)
    _seed_viz_huskies_from_progress(monitor, spec)

    # Example of creating a single-arm robot with robotiq gripper (commented out)
    """create_husky_with_end_effectors(
        monitor, 
        name='/a200_0804', 
        mocap_id=4568, 
        pos=np.array((0,0,0)), 
        connect_arm=not monitor.FAKE_HARDWARE, 
        connect_gripper=not monitor.FAKE_HARDWARE, 
        calibration=monitor.CALIBRATION,
        dual_arm=False,
        ee_types=["robotiq_gripper"],  # Specify end effector for single arm
        base_calibration_file=os.path.join(CALIB_DATA_DIR, 'calibrated_transformation_0804.json')
    )"""

    # Example of creating a robot with calibration tips
    """create_husky_with_end_effectors(
        monitor, 
        name='/a200_0805', 
        mocap_id=1033, 
        pos=np.array((0,1,0)), 
        calibration=True,  # This will automatically use calib_tip
        dual_arm=True
    )"""

    # Example of creating a robot with custom gripper
    """create_husky_with_end_effectors(
        monitor, 
        name='/a200_0806', 
        mocap_id=4592, 
        pos=np.array((1,0,0)), 
        dual_arm=True,
        ee_types=["custom_gripper", "assembly_tool_v3_right"]  # Mixed end effectors
    )"""

    # * add static obstacles
    monitor.add_static_obstacles(pp.create_plane(color=(0.9, 0.9, 0.9, 1)), 'base_plane')
    
    # wall_right = pp.create_box(10, 0.4, 3)
    # pp.set_color(wall_right, pp.GREY)
    # pp.set_pose(wall_right, pp.Pose(pp.Point(0, 2.6, 0)))

    # wall_left = pp.create_box(10, 0.4, 3)
    # pp.set_pose(wall_left, pp.Pose(pp.Point(0, -3.0, 0)))
    # pp.set_color(wall_left, pp.GREY)
    # monitor.add_static_obstacles(wall_left, 'wall_left')
    # monitor.add_static_obstacles(wall_right, 'wall_right')

    # * add tracked obstacles
    # TODO use one tracked box to indicate where to put the assembly
    if monitor.CALIBRATION:
        left_tool_name = 'calib_tool_left'
        TrackedObject(monitor, left_tool_name, 1862, np.zeros(3), np.array((0, 0, 0, 1)), 0.2)
        monitor.assign_calibration_tool_to_robot(0, 0, left_tool_name)

        right_tool_name = 'calib_tool_right'
        TrackedObject(monitor, right_tool_name, 1861, np.zeros(3), np.array((0, 0, 0, 1)), 0.2)
        monitor.assign_calibration_tool_to_robot(0, 1, right_tool_name)

    if monitor.BAR_ACTION_MOCAP_ACCURACY_TEST:
        bar_rig = TrackedObject(monitor, MOCAP_SET_RIG_RB_NAME, 1002, np.zeros(3), np.array((0, 0, 0, 1)), 0.2)
        bar_rig.body = pp.create_cylinder(radius=0.01, height=1, color=(1, 0, 0, 0.2))
        bar_rig.model_base_pose = pp.Pose(euler=pp.Euler(roll=np.pi/2))
        
    if monitor.DUAL_ARM_EE_CONSTR_ACCURACY_MOCAP_TEST:
        left_EE = TrackedObject(monitor, 'left_EE', 1862, np.zeros(3), np.array((0, 0, 0, 1)), 0.2)
        left_EE.body = pp.create_box(0.1, 0.1, 0.1)
        right_EE = TrackedObject(monitor, 'right_EE', 1861, np.zeros(3), np.array((0, 0, 0, 1)), 0.2)
        right_EE.body = pp.create_box(0.1, 0.1, 0.1)

    # * default cfab session from startup (no BarAction needed): build a
    # RobotCell programmatically for whichever rig this is, so free /
    # single-arm planning can run through cfab immediately. Loading a
    # BarAction later swaps in the per-problem cell as before.
    try:
        cell, default_state = build_default_robot_cell(
            ee_types, dual_arm=dual_arm, robot_name=robot_name,
            punch_tool_offsets=punch_offset)
        existing_client_id = pp.CLIENT if pp.is_connected() else None
        with pp.LockRenderer():
            monitor.cfab = CfabSession(None, robot_cell=cell,
                                       connection_type="gui",
                                       enable_debug_gui=True,
                                       existing_client_id=existing_client_id)
        if existing_client_id is not None:
            pp.CLIENTS.setdefault(existing_client_id, True)
        monitor.cfab_default_state = default_state
        # Apply the default state so the attached tools ride on tool0 (the
        # cell spawns them at the origin until a state positions them).
        monitor.cfab.planner.set_robot_cell_state(default_state)
        if getattr(monitor, '_is_live_monitor', False):
            monitor._hide_cfab_robot()
        print(f"[cfab] default RobotCell ready at startup "
              f"(dual_arm={dual_arm}, tools={list(cell.tool_models)}).")
    except Exception as e:
        monitor.cfab = None
        monitor.cfab_default_state = None
        monitor.get_logger().warn(
            f"default cfab session unavailable ({e}); cfab planning starts "
            "when a BarAction is loaded.")

    #boxes.append(TrackedObject(monitor, 'box1', 4457, np.zeros(3), np.array((0, 0, 0, 1)), 0.2, 'cube.obj'))
    #boxes.append(TrackedObject(monitor, 'box2', 4484, np.zeros(3), np.array((0, 0, 0, 1)), 0.2, 'cube.obj'))
    #boxes.append(TrackedObject(monitor, 'box3', 1031, np.zeros(3), np.array((0, 0, 0, 1)), 0.2, 'cube.obj'))

pre_position_trajectory = False
dual_arm_trajectory = None
bar_pose =  pp.Pose([0.5, 0, 0.5], [0, np.pi/2, 0])
next_bar_pose = bar_pose
sphere_center = np.array([0, 0, 0.5])
def next_dual_arm_bar_trajectory(monitor):
    global pre_position_trajectory, dual_arm_trajectory, bar_pose, next_bar_pose
    
    """
    def new_traj():
        pp.draw_pose(bar_pose)
        bar_traj = []
        drr = np.array([-np.pi, 0.25, 0.25]) + np.random.random((3)) * np.array([2*np.pi, 1, 1])
        for j in range(10):
            arc_len = j * 0.1 * 0.2
            yrot1 = pp.Pose(euler=[0, drr[0], 0])
            yoffset = pp.Pose(point=[0, drr[1], 0])
            zrot = pp.Pose(euler=[0, 0, arc_len/drr[1]])
            zoffset = pp.Pose(point=[0, 0, drr[2]])
            yrot = pp.Pose(euler=[0, arc_len/drr[2], 0])
            bar_traj.append(pp.multiply(bar_pose, zoffset, yrot, pp.invert(zoffset), yoffset, zrot, pp.invert(yoffset)))
            pp.draw_pose(bar_traj[-1])
        next_bar_pose = bar_traj[-1]
        pp.draw_pose(next_bar_pose)
        
        return bar_traj
    """
    
    #monitor.set_arm_trajectory(([hi.arm_joint_pose[0], dual_arm_trajectory[0][0][0]], None, 10, None), index=0)
    #monitor.set_arm_trajectory(([hi.arm_joint_pose[1], dual_arm_trajectory[1][0][0]], None, 10, None), index=1)
    
    def new_random_bar_pose(bar_pose):
        rand_dir = np.array([-1, -1, -1]) + np.random.random((3)) * 2
        rand_dir = rand_dir / np.linalg.norm(rand_dir)
        rand_angle = np.array([-np.pi/4, -np.pi/4, -np.pi/4]) + np.random.random((3)) * np.pi/2
        
        rand_pose = pp.Pose(rand_dir*0.2, rand_angle)
        return pp.multiply(bar_pose, rand_pose)
    
    while True:
        if not pre_position_trajectory:
            next_bar_pose = new_random_bar_pose(bar_pose)
            bar_traj = planning.dual_arm_bar_arc(bar_pose, next_bar_pose, 10)
            for p in bar_traj:
                pp.draw_pose(p)
            dual_arm_trajectory = planning.plan_dual_arm_motion(monitor.huskies[0], bar_traj, list(monitor.static_obstacles.values()))
        if dual_arm_trajectory is not None:
            hi = monitor.huskies[monitor.selected_robot_id].interface
            if np.max(np.abs(hi.arm_joint_pose[0]-dual_arm_trajectory[0][0][0]) > 0.1) or np.max(np.abs(hi.arm_joint_pose[1]-dual_arm_trajectory[1][0][0]) > 0.1):
                # this fails to find transitmotions often, apparently one or both arm configs are in collision... but they arent
                #L = planning.plan_arm_motion(monitor.huskies[monitor.selected_robot_id], dual_arm_trajectory[0][0][0], [], 10, arm_index=0)
                #R = planning.plan_arm_motion(monitor.huskies[monitor.selected_robot_id], dual_arm_trajectory[1][0][0], [], 10, arm_index=1)
                #monitor.set_arm_trajectory(L, index=0)
                #monitor.set_arm_trajectory(R, index=1)
                monitor.set_arm_trajectory(([hi.arm_joint_pose[0], dual_arm_trajectory[0][0][0]], None, 10, None), index=0)
                monitor.set_arm_trajectory(([hi.arm_joint_pose[1], dual_arm_trajectory[1][0][0]], None, 10, None), index=1)
                pre_position_trajectory = True
            else:
                monitor.set_arm_trajectory(dual_arm_trajectory[0], index=0)
                monitor.set_arm_trajectory(dual_arm_trajectory[1], index=1)
                pre_position_trajectory = False
            break


def update(monitor):
    pass

def plan_base_to_goal(monitor):
    base = planning.plan_base_motion(monitor.huskies[monitor.selected_robot_id], monitor.goal_pose, [])
    monitor.set_base_trajectry(base)

# def plan_arm_wave(monitor):
#     monitor.set_arm_trajectory(planning.plan_arm_wave(monitor.huskies[monitor.selected_robot_id], monitor.trajectory_time))

def plan_arm_to_goal(monitor):
    obstacles = [monitor.assembly_objects[i].body for i in range(monitor.current_seq_index)] + _get_manual_staging_obstacles(monitor)
    
    print(f"Planning from {monitor.huskies[monitor.selected_robot_id].interface.arm_joint_pose[monitor.selected_arm_index]} to {monitor.goal_arm_pose[monitor.selected_arm_index]} with obstacles {obstacles}")
    
    monitor.set_arm_trajectory(
        planning.plan_arm_motion(
            monitor.huskies[monitor.selected_robot_id], 
            monitor.goal_arm_pose[monitor.selected_arm_index], 
            obstacles, 
            monitor.trajectory_time,
            grasped_element=monitor.goal_element, 
            grasp=monitor.goal_bar_grasp, 
            arm_index=monitor.selected_arm_index
            ), 
        index=monitor.selected_arm_index
        )
    monitor.set_to_show_traj_state()

#################################

def sample_calib_motion(monitor, arm_index, target_joint_index, calib_joint_range, attachments=None, obstacles=None):
    assert target_joint_index in [0,1], "only support calibrating for joint 0 or 1 for now"

    # Sample calibration conf:
    ATTEMPTS = 100
    TRAJ_MAX_LENGTH = 200
    steps = 20
    joint_resolutions = np.ones(6) * 0.05

    attachments = attachments or []
    obstacles = obstacles or []
    
    # use correct joint names for dual arm husky
    if monitor.huskies[monitor.selected_robot_id].dual_arm:
        if arm_index == 0:
            arm_prefix = "left_"
            joint_names = HUSKY_DUAL_UR5e_JOINT_NAMES[0]
        else:
            arm_prefix = "right_"
            joint_names = HUSKY_DUAL_UR5e_JOINT_NAMES[1]
    else:
        joint_names = UR5E_JOINT_NAMES
        arm_prefix = ""

    robot = monitor.huskies[monitor.selected_robot_id].object.robot
    hi = monitor.huskies[monitor.selected_robot_id].interface

    current_conf = hi.arm_joint_pose[arm_index]
    custom_limits_from_joint_name = {}
    original_joint_limits = []
    for joint_name in joint_names:
        original_joint_limits.append(pp.get_joint_limits(robot, pp.joint_from_name(robot, joint_name)))
    # * Set custom limits around current configuration for each joint
    for i, joint_name in enumerate(joint_names):
        if i != target_joint_index:  # Skip the target joint as we'll set it separately
            # Set limits to current value ± pi/2, but ensure within original joint limits
            custom_limits_from_joint_name[joint_name] = (
                max(current_conf[i] - np.pi/3, original_joint_limits[i][0]+np.pi/5),
                min(current_conf[i] + np.pi/3, original_joint_limits[i][1]-np.pi/5)
            )

    # * For the target joint, set limits to current value ± calib_joint_range
    target_joint_pb_id = pp.joint_from_name(robot, joint_names[target_joint_index])
    targt_joint_limits = pp.get_joint_limits(robot, target_joint_pb_id)
    # custom_limits_from_joint_name[joint_names[target_joint_index]] = (targt_joint_limits[0] + calib_joint_range, targt_joint_limits[1] - calib_joint_range)

    # * Clamp the first joint to 0 if target joint == 1
    # if target_joint_index == 0:
    #     # clamp the first joint to value 0
    #     custom_limits_from_joint_name[joint_names[0]] = (-np.pi,-np.pi)
    if target_joint_index == 1:
        custom_limits_from_joint_name[joint_names[0]] = (0.0,0.0)

    custom_limits = get_custom_limits(robot, custom_limits_from_joint_name)
    print(custom_limits)

    # disabled_collisions = disabled_collisions or {}
    extra_disabled_collisions = [
        ((robot, pp.link_from_name(robot, arm_prefix + 'ur_arm_wrist_3_link')), 
         (attachments[0].child, pp.BASE_LINK)), 
         # pp.link_from_name(ee_body, 'robotiq_85_base_link'))),
        ]

    movable_joints = pp.joints_from_names(robot, joint_names)
    transit_sample_fn = pp.get_sample_fn(robot, movable_joints) #, custom_limits=custom_limits)
    distance_fn = pp.get_distance_fn(robot, movable_joints) #, weights=weights)
    extend_fn = pp.get_extend_fn(robot, movable_joints, resolutions=joint_resolutions)

    sample_fn = pp.get_sample_fn(robot, movable_joints, custom_limits=custom_limits)
    collision_fn = pp.get_collision_fn(robot, movable_joints, obstacles=obstacles,
                                              attachments=attachments, 
                                              self_collisions=1,
                                              disabled_collisions={}, 
                                              extra_disabled_collisions=extra_disabled_collisions,
                                              custom_limits={}, 
                                              max_distance=0)

    # * the robot base pose should be udpated by the main loop in monitor according to mocap observation before the planning starts
    diagnose = 0
    with pp.WorldSaver():
        with pp.LockRenderer(False):
            for i in range(ATTEMPTS):
                valid_calib_path = True
                start_conf = np.array(sample_fn())
                pp.set_joint_positions(robot, movable_joints, start_conf)

                if target_joint_index == 0:
                    start_conf[target_joint_index] = -np.pi

                # pp.wait_if_gui()

                print(f'Attempt #{i+1}/{ATTEMPTS}, start_conf: {start_conf} | current conf: {hi.arm_joint_pose[arm_index]}')
                # - click `execute calib` will first execute the transit path in one go, and then execute the calib path point by point, waiting for the arm to settle before moving to the next point. It will save the calibration data for each point, and in the end export the data to a json file.

                # - check start conf is in collision or not
                if not collision_fn(start_conf, diagnosis=diagnose):
                    # - check the interpolated calib path is safe, if not resample

                    # interpolate between current conf and goal conf
                    # Create goal_conf by copying start_conf and modifying only the target_joint_index value
                    goal_conf = np.copy(start_conf)
                    goal_conf[target_joint_index] += calib_joint_range

                    calib_path = []
                    for j in range(steps):
                        joint_conf = np.array(start_conf) + (j+1)/steps * (np.array(goal_conf) - np.array(start_conf))
                        print(f'step {j}: joint conf: {joint_conf}')
                        if collision_fn(joint_conf, diagnosis=False):
                            valid_calib_path = False
                            monitor.get_logger().warn(f"Collision detected at calb conf #{j}/{steps}, resampling...")
                            break
                        calib_path.append(joint_conf)
                    if not valid_calib_path:
                        break

                    if valid_calib_path:
                        # - check if the transit path is too long, if so, resample
                        # * plan transit arm motion
                        transit_path = None
                        if pp.check_initial_end(current_conf, start_conf, collision_fn, diagnosis=diagnose):
                            # TODO: this might plan path that causes collision between the two arms
                            transit_path = pp.solve_motion_plan(current_conf, start_conf, 
                                                        distance_fn, transit_sample_fn, extend_fn,
                                                        collision_fn,
                                                        algorithm='birrt', 
                                                        max_time=10, 
                                                        max_iterations=20, 
                                                        smooth=20, diagnosis=diagnose,
                                                        coarse_waypoints=False,
                                                        ) 
                        else:
                            notify('Transit initial and end conf not valid')

                        if transit_path is not None:
                            if len(transit_path) < TRAJ_MAX_LENGTH:
                                monitor.get_logger().info(f"Transit planning succeeded with {len(transit_path)} points!")
                                # - collage both trajectory together for viz, save transit to free_arm_trajectory, save calib to linear_arm_trajectory
                                planned_arm_trajectory = [np.array(p) for p in transit_path + calib_path]

                                fm_time = monitor.trajectory_time # len(transit_path) / len(planned_arm_trajectory)
                                lm_time = 2*len(calib_path)
                                # len(calib_path) / len(planned_arm_trajectory)

                                # time here will be overwritten anyway
                                return (planned_arm_trajectory, None, fm_time + lm_time, None), \
                                       (np.array(transit_path), None, fm_time, None), \
                                       (np.array(calib_path), None, lm_time, None)

                            else:
                                monitor.get_logger().warn(f"Transit planning trajectory too long {len(transit_path)}!")
                        else:
                            monitor.get_logger().warn("Transit planning failed!")
                else:
                    monitor.get_logger().warn("Collision detected at start conf, resampling...")

    monitor.get_logger().warn(f"Calibration motion planning failed after {ATTEMPTS} attempts!")

def calibrate_button(monitor, tool_mocap_name, index=0):
    # record current joint conf and add to record
    h = monitor.huskies[monitor.selected_robot_id]
    hi = h.interface
    ho = h.object
    # fetch calibration mocap set frame
    flange_mocap_pose = None
    base_mocap_pose = None

    if index > 0:
        # must be using the dual arm
        tool0_link_name = 'right_ur_arm_tool0'
    else:
        if pp.has_link(ho.robot, "ur_arm_tool0"):
            tool0_link_name = 'ur_arm_tool0'
        else:
            tool0_link_name = 'left_ur_arm_tool0'

    if monitor.USE_MOCAP:
        # need to get the raw data from mocap
        print(monitor._mocap_rigidbody_cache)
        if h.name in monitor._mocap_rigidbody_cache:
            base_mocap_pose = monitor._mocap_rigidbody_cache[h.name]
        else:
            monitor.get_logger().warn(f"Base mocap pose for '{h.name}' not found in mocap cache!")
        if tool_mocap_name in monitor._mocap_rigidbody_cache:
            flange_mocap_pose = monitor._mocap_rigidbody_cache[tool_mocap_name]
        else:
            monitor.get_logger().warn(f"Flange mocap pose for '{tool_mocap_name}' not found in mocap cache!")
    else:
        pass
        # base_mocap_pose = ho.get_link_pose_from_name("base_footprint")
        # flange_mocap_pose = ho.get_link_pose_from_name(tool0_link_name)

    tool0_fk_pose = ho.get_link_pose_from_name(tool0_link_name)

    # Visualization for debugging mocap poses
    DEBUG_MOCAP_POSES = False  # Toggle this to enable/disable mocap pose visualization

    if DEBUG_MOCAP_POSES:
        # Make all robot links transparent for easier visualization
        # robot = ho.robot
        # for link_id in range(pp.get_num_joints(robot)):
        #     pp.set_color(robot, [1, 1, 1, 0.2], link=link_id)  # Use RGBA where A<1 for transparency
        # # Also set the base link transparent
        # pp.set_color(robot, [1, 1, 1, 0.2], link=-1)

        # Determine the arm_base_link name based on dual arm setup and index
        if monitor.huskies[monitor.selected_robot_id].dual_arm:
            if index > 0:
                arm_base_link_name = 'right_ur_arm_base_link_inertia'
                arm_prefix = 'right_'
            else:
                arm_base_link_name = 'left_ur_arm_base_link_inertia'
                arm_prefix = 'left_'
        else:
            arm_base_link_name = 'ur_arm_base_link_inertia'
            arm_prefix = ''

        # Get all poses for visualization
        base_footprint_pose = ho.get_link_pose_from_name("base_footprint")
        arm_base_link_pose = ho.get_link_pose_from_name(arm_base_link_name)
        tool0_pose = ho.get_link_pose_from_name(tool0_link_name)

        # Draw the poses with annotations
        if base_mocap_pose is not None:
            pp.draw_pose(base_mocap_pose, length=0.15)
            pp.add_text("base_mocap", position=base_mocap_pose[0])

        if flange_mocap_pose is not None:
            pp.draw_pose(flange_mocap_pose, length=0.15)
            pp.add_text("flange_mocap (calib_tool)", position=flange_mocap_pose[0])

        pp.draw_pose(tool0_pose, length=0.15)
        pp.add_text(f"{tool0_link_name}", position=tool0_pose[0])

        # pp.draw_pose(base_footprint_pose, length=0.15)
        # pp.add_text("base_footprint_link", position=base_footprint_pose[0])

        # pp.draw_pose(arm_base_link_pose, length=0.15)
        # pp.add_text(f"{arm_base_link_name}", position=arm_base_link_pose[0])

        # # Visualize all link poses between arm_base_link_inertia and tool0
        # arm_link_names = [
        #     f"{arm_prefix}ur_arm_shoulder_link",
        #     f"{arm_prefix}ur_arm_upper_arm_link",
        #     f"{arm_prefix}ur_arm_forearm_link",
        #     f"{arm_prefix}ur_arm_wrist_1_link",
        #     f"{arm_prefix}ur_arm_wrist_2_link",
        #     f"{arm_prefix}ur_arm_wrist_3_link",
        #     f"{arm_prefix}ur_arm_tool0"
        # ]

        # for link_name in arm_link_names:
        #     try:
        #         link_pose = ho.get_link_pose_from_name(link_name)
        #         pp.draw_pose(link_pose, length=0.1)
        #         pp.add_text(link_name, position=[p + 0.015 for p in link_pose[0]])
        #     except:
        #         pass  # Skip if link doesn't exist

    if flange_mocap_pose is None:
        if monitor.CALIBRATION:
            monitor.get_logger().warn(f'Mocap {tool_mocap_name} not found!')
            _warn_available_calib_tools(monitor, tool_mocap_name)
            return
        else:
            pp.draw_pose(base_mocap_pose)
            monitor.append_calibration_data({
                    'robot_id' : int(monitor.selected_robot_id),
                    'arm_index' : int(monitor.selected_arm_index),
                    'joint_conf' : list(hi.arm_joint_pose[monitor.selected_arm_index]), 
                    'base_mocap_pose' : [list(v) for v in base_mocap_pose],
                    "flange_mocap_pose" : [],
                    'tool0_fk_pose' : [list(v) for v in tool0_fk_pose],
                    'tool0_fk_from_mocap' : [],
                 })
    else:
        tool_0_fk_from_mocap = pp.multiply(pp.invert(tool0_fk_pose), flange_mocap_pose)

        # Draw the poses with annotations
        if base_mocap_pose is not None:
            pp.draw_pose(base_mocap_pose, length=0.15)
            pp.add_text("base_mocap", position=base_mocap_pose[0])

        if flange_mocap_pose is not None:
            pp.draw_pose(flange_mocap_pose, length=0.15)
            pp.add_text("flange_mocap (calib_tool)", position=flange_mocap_pose[0])

        # tool0_pose = ho.get_link_pose_from_name(tool0_link_name)
        # pp.draw_pose(tool0_pose, length=0.15)
        # pp.add_text(f"{tool0_link_name}", position=tool0_pose[0])

        monitor.append_calibration_data({
                'robot_id' : int(monitor.selected_robot_id),
                'arm_index' : int(monitor.selected_arm_index),
                'joint_conf' : list(hi.arm_joint_pose[monitor.selected_arm_index]), 
                'base_mocap_pose' : [list(v) for v in base_mocap_pose],
                "flange_mocap_pose" : [list(v) for v in flange_mocap_pose],
                'tool0_fk_pose' : [list(v) for v in tool0_fk_pose],
                'tool0_fk_from_mocap' : [list(v) for v in tool_0_fk_from_mocap],
             })

def save_calibration(monitor, filename_suffix="", date_folder=None, data_batch=None):
    # save monitor.calibration_data to json, file name with time stamp
    # save to CALIB_DATA_DIR (gdrive)/<date_folder>/<data_batch>/
    timestamp = datetime.now().strftime("%Y%m%d_%H%M")

    if date_folder is None:
        date_folder = datetime.now().strftime("%Y%m%d")

    date_folder_path = os.path.join(CALIB_DATA_DIR, date_folder)
    subfolder_path = date_folder_path
    if data_batch:
        subfolder_path = os.path.join(subfolder_path, data_batch)

    os.makedirs(subfolder_path, exist_ok=True)
    os.makedirs(date_folder_path, exist_ok=True)
    _ensure_calibration_conf(monitor, date_folder_path)

    if filename_suffix:
        filename = os.path.join(subfolder_path, f"calibration_{timestamp}_{filename_suffix}.json")
    else:
        filename = os.path.join(subfolder_path, f"calibration_{timestamp}.json")

    with open(filename, 'w') as f:
        json.dump({'raw_data' : monitor.calibration_data}, f, indent=4)

    monitor.get_logger().info(f"Calibration data saved to {filename}")

#################################
# Punch tool calibration validation
#################################

def record_punch_reference(monitor, date_folder=None):
    """Record the current world_from_punch_tip pose using FK + punch offset.

    Appends the result to monitor.punch_validation_results for later analysis.
    """
    h = monitor.huskies[monitor.selected_robot_id]
    ho = h.object
    hi = h.interface
    arm_index = int(monitor.selected_arm_index)
    arm_name = get_runtime_arm_name(h.dual_arm, arm_index)
    tool0_from_punch_tip = monitor.get_tool0_from_punch_tip(arm_index)

    # Get tool0 link name based on arm
    if h.dual_arm:
        tool0_link_name = 'left_ur_arm_tool0' if arm_index == 0 else 'right_ur_arm_tool0'
    else:
        tool0_link_name = 'ur_arm_tool0'

    # Ensure sim state is up to date
    ho.set_pose((hi.position, hi.rotation), hi.arm_joint_pose)

    # FK: world_from_tool0 * tool0_from_punch_tip
    world_from_tool0 = ho.get_link_pose_from_name(tool0_link_name)
    world_from_punch_tip = pp.multiply(world_from_tool0, tool0_from_punch_tip)

    # Visualize
    take_num = 1 + sum(
        1 for take in monitor.punch_validation_results
        if int(take.get('arm_index', -1)) == arm_index
    )
    pp.draw_pose(world_from_punch_tip, length=0.05)
    pp.add_text(f"{arm_name.upper()} TAKE {take_num}", position=world_from_punch_tip[0])

    # Append to validation results
    result = {
        'timestamp': datetime.now().isoformat(),
        'arm_index': arm_index,
        'arm_name': arm_name,
        'tool0_link_name': tool0_link_name,
        'joint_conf': [float(v) for v in hi.arm_joint_pose[arm_index]],
        'base_pose': {
            'position': [float(v) for v in hi.position],
            'quaternion': [float(v) for v in hi.rotation],
        },
        'world_from_punch_tip': {
            'position': [float(v) for v in world_from_punch_tip[0]],
            'quaternion': [float(v) for v in world_from_punch_tip[1]],
        },
        'tool0_from_punch_tip': {
            'position': [float(v) for v in tool0_from_punch_tip[0]],
            'quaternion': [float(v) for v in tool0_from_punch_tip[1]],
        },
    }
    monitor.punch_validation_results.append(result)

    monitor.get_logger().info(
        f'Punch validation take {take_num} recorded. '
        f'position: {world_from_punch_tip[0]}'
    )


def save_punch_validation_data(monitor, date_folder=None):
    """Save all accumulated punch validation results to JSON."""
    if not monitor.punch_validation_results:
        monitor.get_logger().warn('No punch validation results to save!')
        return

    if date_folder is None:
        date_folder = datetime.now().strftime("%Y%m%d")

    punch_dir = os.path.join(CALIB_DATA_DIR, date_folder, "punch_validation")
    os.makedirs(punch_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M")
    grouped_results = {}
    for take in monitor.punch_validation_results:
        arm_index = int(take.get('arm_index', 0))
        grouped_results.setdefault(arm_index, []).append(take)

    for arm_index, takes in sorted(grouped_results.items()):
        arm_name = takes[0].get('arm_name', arm_index_to_name(arm_index))
        filename = os.path.join(punch_dir, f'punch_validation_{arm_name}_{timestamp}.json')

        with open(filename, 'w') as f:
            json.dump({
                'arm_index': arm_index,
                'arm_name': arm_name,
                'tool0_link_name': takes[0].get('tool0_link_name'),
                'tool0_from_punch_tip': takes[0]['tool0_from_punch_tip'],
                'takes': takes,
            }, f, indent=4)

        monitor.get_logger().info(
            f'Punch validation data saved to {filename} ({len(takes)} {arm_name} takes)'
        )

    monitor.punch_validation_results = []


#################################

def request_marketset_button(monitor, rb_mocap_name):
    # record current joint conf and add to record
    h = monitor.huskies[monitor.selected_robot_id]
    hi = h.interface
    ho = h.object
    # fetch calibration mocap set frame
    base_mocap_pose = None
    base_link_pose = ho.get_link_pose_from_name("base_footprint")

    if monitor.USE_MOCAP and h.name in monitor._mocap_rigidbody_cache:
        # need to get the raw data from mocap
            base_mocap_pose = monitor._mocap_rigidbody_cache[h.name]
    else:
        base_mocap_pose = base_link_pose

    # print(monitor._mocap_labeled_marker_cache)

    if rb_mocap_name not in monitor._mocap_labeled_marker_cache:
        monitor.get_logger().warn(f'Mocap {rb_mocap_name} not found!')
        return
    else:
        labeled_marker_data = monitor._mocap_labeled_marker_cache[rb_mocap_name]

        for marker_name, marker_data in labeled_marker_data.items():
            pp.draw_point(marker_data['pos'])

        bar_pose = None
        if hasattr(monitor, 'get_bar_action_goal_bar_pose'):
            bar_pose = monitor.get_bar_action_goal_bar_pose()
        if bar_pose is None:
            try:
                bar_pose = monitor.get_world_from_bar_goal_pose()
            except Exception:
                bar_pose = None

        take = {
            'joint_conf' : list(hi.arm_joint_pose[monitor.selected_arm_index]),
            'base_mocap_pose' : [list(v) for v in base_mocap_pose],
            'footprint_base_link_pose' : base_link_pose,
            rb_mocap_name : copy.deepcopy(labeled_marker_data),
        }
        if bar_pose is not None:
            take['world_from_bar_pose'] = bar_pose
            take['bar_euler_angles'] = list(pp.euler_from_quat(bar_pose[1]))
        monitor.marker_set_data.append(take)

        try:
            from husky_assembly_teleop.mocap_experiment import (
                fit_bar_from_markerset, bar_deviation_from_goal,
            )
            fit = fit_bar_from_markerset(labeled_marker_data)
            enrichment = {
                'fitted_line': fit['fitted_line'],
                'ocf_position': fit['ocf_position'],
                'bar_end_points': fit['bar_end_points'],
                'bar_length_observed': fit['bar_length_observed'],
                'center_to_line_dist_max_m': fit['center_to_line_dist_max_m'],
                'center_to_line_dist_rms_m': fit['center_to_line_dist_rms_m'],
            }
            if bar_pose is not None:
                dev = bar_deviation_from_goal(fit, bar_pose)
                enrichment.update({
                    'pos_dev_m': dev['pos_dev_m'],
                    'angle_dev_rad': dev['angle_rad'],
                    'lateral_dev_m': dev['lateral_dev_m'],
                })
            monitor.marker_set_data[-1].update(enrichment)

            if not hasattr(monitor, '_bar_holding_fit_line_uids'):
                monitor._bar_holding_fit_line_uids = []
            uid = pp.add_line(fit['bar_end_points'][0], fit['bar_end_points'][1], color=[0, 0, 1])
            monitor._bar_holding_fit_line_uids.append(uid)

            ocf = fit['ocf_position']
            if bar_pose is not None:
                monitor.get_logger().info(
                    f"[bar take] ocf=({ocf[0]:.3f},{ocf[1]:.3f},{ocf[2]:.3f}) m | "
                    f"pos_dev={dev['pos_dev_m']*1000:.2f} mm | "
                    f"angle_dev={np.degrees(dev['angle_rad']):.2f} deg | "
                    f"center_to_line_dist_max={fit['center_to_line_dist_max_m']*1000:.2f} mm | "
                    f"bar_len={fit['bar_length_observed']:.4f} m"
                )
            else:
                monitor.get_logger().info(
                    f"[bar take] ocf=({ocf[0]:.3f},{ocf[1]:.3f},{ocf[2]:.3f}) m | "
                    f"center_to_line_dist_max={fit['center_to_line_dist_max_m']*1000:.2f} mm | "
                    f"bar_len={fit['bar_length_observed']:.4f} m | no goal pose"
                )
        except Exception as e:
            monitor.get_logger().warn(f"bar take fit/dev failed: {e}")

def save_markerset_data(monitor, filename_suffix="", use_experiment_dir=False):
    print(monitor.calibration_data)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M")
    # Create a date subfolder (format: YYYYMMDD)
    date_subfolder = datetime.now().strftime("%Y%m%d")
    root_dir = BAR_HOLDING_ACC_EXPERIMENT_DIR if use_experiment_dir else BAR_HOLDING_ACC_DATA_DIR
    subfolder_path = os.path.join(root_dir, date_subfolder + f'{filename_suffix}')

    # Create the subfolder if it doesn't exist
    if not os.path.exists(subfolder_path):
        os.makedirs(subfolder_path)
        monitor.get_logger().info(f"Created subfolder: {subfolder_path}")

    # * The take names its movement by the real movement id (e.g.
    # B1_R_M2_LM_retreat); the offline 0_/1_ scripts match on it. Older takes
    # stamped a legacy role name (M0..M4) here, which 1_compare_to_cell_state.py
    # still reads.
    mv = monitor.current_movement
    movement_id = getattr(mv, 'movement_id', None)

    # Name the file this movement actually came from: a split export keeps the
    # cycle in two files and the monitor loads both, so the retreat is usually
    # in the release half while _current_action_path names the jointing one.
    # movement_index is the index INSIDE that file (action_file), not the index
    # in the monitor's concatenated list.
    action_path = getattr(monitor, '_current_action_path', None)
    slot = monitor._slot_of_movement(getattr(monitor, 'current_movement_index', None))
    if slot is not None:
        action_path = slot[1]
    movement_index = slot[2] if slot else None
    action_file = os.path.basename(action_path) if action_path else None

    # Bar world pose + AABB dimensions in the movement's start state, stamped
    # here so the offline scripts don't have to re-parse the BarAction file
    # (older on-disk BarActions may no longer import cleanly).
    bar_pose = None
    try:
        bar_pose = monitor.get_movement_start_bar_pose()
    except Exception as e:
        # ! Loud, not a warning: the take is still written (the offline
        # ! 1_compare_to_cell_state.py re-derives the goal from the BarAction
        # ! when the stamp is null), but without this pose the take cannot be
        # ! scored from its own file, so the operator must see it go by.
        monitor.get_logger().error(
            f"could not compute start-state bar pose, saving take with a NULL "
            f"reference pose: {e}")
    bar_dims = None
    try:
        dims = monitor.get_active_bar_aabb_dims()
        if dims is not None:
            bar_dims = [float(v) for v in dims]
    except Exception:
        bar_dims = None

    # Warn loudly if nothing was loaded: the fields below would be null and the
    # take couldn't be matched to a movement later.
    if not getattr(monitor, '_current_action_path', None) or mv is None:
        monitor.get_logger().warn(
            "Save markerset: no BarAction / movement is loaded, so "
            "bar_action_path / movement_id / bar pose will be null. Click "
            "'Load BarAction' then 'Load Movement' before recording so the "
            "take can be matched to a movement later."
        )

    # Save the file in the date subfolder
    filename = os.path.join(subfolder_path, f"bar_holding_acc_{timestamp}.json")
    with open(filename, 'w') as f:
        payload = {
            'mocap_axis_convention': getattr(monitor, 'MOCAP_AXIS_CONVENTION', 'rotated'),
            'bar_action_path': action_path,
            'action_file': action_file,
            'movement_id': movement_id,
            'movement_index': movement_index,
            'bar_name': getattr(monitor, 'active_bar_name', None),
            'bar_start_position': bar_pose[0] if bar_pose is not None else None,
            'bar_start_quaternion': bar_pose[1] if bar_pose is not None else None,
            'bar_dimensions': bar_dims,
            'raw_data': monitor.marker_set_data,
        }
        json.dump(payload, f, indent=4)

    monitor.get_logger().info(f"Bar holding acc data saved to {filename}")

#################################

def record_dual_arm_E_mocap(monitor):
    left_EE_mocap_name = "left_EE"
    right_EE_mocap_name = "right_EE"
    # record current joint conf and add to record
    h = monitor.huskies[monitor.selected_robot_id]
    hi = h.interface
    ho = h.object
    left_EE_pose = None
    right_EE_pose = None
    if monitor.USE_MOCAP:
        # need to get the raw data from mocap
        if h.name in monitor._mocap_rigidbody_cache:
            base_mocap_pose = monitor._mocap_rigidbody_cache[h.name]
        if left_EE_mocap_name in monitor._mocap_rigidbody_cache:
            left_EE_pose = monitor._mocap_rigidbody_cache[left_EE_mocap_name]
        else:
            monitor.get_logger().warn(f'Mocap {left_EE_mocap_name} not found!')
            return
        if right_EE_mocap_name in monitor._mocap_rigidbody_cache:
            right_EE_pose = monitor._mocap_rigidbody_cache[right_EE_mocap_name]
        else:
            monitor.get_logger().warn(f'Mocap {right_EE_mocap_name} not found!')
            return
    else:
        monitor.get_logger().warn(f'Mocap must be active to conduct dual arm test!')
        return

    pp.draw_pose(left_EE_pose)
    pp.draw_pose(right_EE_pose)
    
    monitor.dual_arm_EE_mocap_data.append(
        {
            'left_EE_pose': [list(v) for v in left_EE_pose],
            'right_EE_pose': [list(v) for v in right_EE_pose]
        }
    )

def save_dual_arm_E_mocap(monitor, filename_suffix="", metadata=None):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M")
    # Create a date subfolder (format: YYYYMMDD)
    date_subfolder = datetime.now().strftime("%Y%m%d")
    subfolder_path = os.path.join(DUAL_ARM_ACC_DATA_DIR, date_subfolder)

    # Create the subfolder if it doesn't exist
    if not os.path.exists(subfolder_path):
        os.makedirs(subfolder_path)
        monitor.get_logger().info(f"Created subfolder: {subfolder_path}")

    # Save the file in the date subfolder
    filename = os.path.join(subfolder_path, f"dual_arm_acc_{timestamp}_{filename_suffix}.json")
    payload = {'raw_data': monitor.dual_arm_EE_mocap_data}
    if metadata:
        payload['metadata'] = metadata
    with open(filename, 'w') as f:
        json.dump(payload, f, indent=4)

    monitor.get_logger().info(f"Dual arm acc data saved to {filename}")

def _capture_reference_relative_EE(monitor):
    # Reference relative TF (right_from_left) from current mocap snapshot.
    # Constraint should hold here at start_conf; deviations during execution
    # are tracker error.
    cache = monitor._mocap_rigidbody_cache
    if 'left_EE' not in cache or 'right_EE' not in cache:
        return None
    L = cache['left_EE']
    Rp = cache['right_EE']
    rel = pp.multiply(pp.invert(Rp), L)
    return [list(rel[0]), list(rel[1])]

def execute_and_log_mocap(monitor):
    ref = _capture_reference_relative_EE(monitor)
    if ref is None:
        monitor.get_logger().warn('left_EE / right_EE not in mocap cache; aborting record.')
        return
    execute_arm_trajectory_both(monitor)
    while monitor.huskies[monitor.selected_robot_id].interface.is_arm_executing[0] or monitor.huskies[monitor.selected_robot_id].interface.is_arm_executing[1]:
        record_dual_arm_E_mocap(monitor)
        yield
    save_dual_arm_E_mocap(monitor, metadata={'reference_right_from_left': ref})

#################################
 
def calibrate_joint(monitor, joint_id, tool_mocap_name):
    raise DeprecationWarning("This function is deprecated.")

    print('Triggered joint calibration for joint id:', joint_id)

    hi = monitor.huskies[monitor.selected_robot_id].interface
    ho = monitor.huskies[monitor.selected_robot_id].object
    current_conf = hi.arm_joint_pose[monitor.selected_arm_index]
    goal_conf = np.copy(monitor.goal_arm_pose[monitor.selected_arm_index])
    # check if values are close between current conf and goal conf, except for the joint id
    diff_vec = np.abs(np.array(current_conf) - np.array(goal_conf))
    diff_vec[joint_id] = 0
    if not np.all(diff_vec < 1e-4):
        monitor.get_logger().warn(f'Current conf and goal conf differs in axes other than the target joint {joint_id}: {diff_vec}!')
        return
   
    # joint_limit = pp.get_joint_limits(ho.robot, pp.joint_from_name(ho.robot, HUSKY_UR5e_JOINT_NAMES[joint_id]))

    steps = 20
    # interpolate between current conf and goal conf
    joint_confs = []
    for i in range(steps):
        joint_conf = np.array(current_conf) + (i+1)/steps * (np.array(goal_conf) - np.array(current_conf))
        joint_confs.append(joint_conf)

    monitor.set_arm_trajectory(
        (joint_confs, None, monitor.trajectory_time, None),
        index=monitor.selected_arm_index
        )
    monitor.set_to_show_traj_state()
    
def execute_arm_conf(monitor, conf, index=0):
    # execute a single arm conf trajectory
    hi = monitor.huskies[monitor.selected_robot_id].interface
    monitor.huskies[monitor.selected_robot_id].interface.send_arm_cmd([hi.arm_joint_pose[monitor.selected_arm_index], conf], 
                                                                      None, monitor.trajectory_time, index=index)

def execute_arm_trajectory_and_record_each_conf(monitor, calib_traj, time_between_confs=2, index=0):
    # settle_time = 4
    settle_time = 6
    time_between_confs = 1
    hi = monitor.huskies[monitor.selected_robot_id].interface
    # last_conf = hi.arm_joint_pose[index]
    # print(transit_traj)
    # execute_arm_trajectory(monitor, transit_traj, index=index)

    total_num_confs = len(calib_traj[0])

    # ! there seems to be a delay in arm conf, resulting in a one-step lag between the conf and the mocap data
    # TODO investigate
    for i, conf in enumerate(calib_traj[0]):
        monitor.get_logger().info(f'Executing arm conf {i+1}/{len(calib_traj[0])}...')
        hi.send_arm_cmd(
            [hi.arm_joint_pose[monitor.selected_arm_index], conf], 
            # [conf], 
            None, 
            time_between_confs,
            index=index
            )

        # wait until it finishes
        time.sleep(time_between_confs + settle_time)

        # ! since the joint state is updated in the main thread and is blocked when running this function, 
        # we need to manually update the last conf here
        # Todo: change to Jakob's task system to avoid blocking the main thread
        # ! important to update it before the calibrate button, since it needs the latest conf
        hi.arm_joint_pose[monitor.selected_arm_index] = conf

        calibrate_button(monitor, monitor.active_calib_tool_name)
        monitor.get_logger().info(f'Saved calibration data {i}/{total_num_confs}.')

    # save_calibration(monitor, filename_suffix=f'arm_{monitor.selected_arm_index}_j_{monitor.calib_target_axis}')
    # monitor.calibration_data = []

#################################

def execute_arm_trajectory(monitor, trajectory, index=0):
    if trajectory is None:
        monitor.get_logger().warn('Arm trajectory must be planed before executing!')
        return
    # trajectory confs, velocity, total time
    monitor.huskies[monitor.selected_robot_id].interface.send_arm_cmd(trajectory[0], trajectory[1], monitor.trajectory_time, index=index)

def _per_axis_axis_angles_deg(quat_a, quat_b):
    """Angle between the x/y/z axes of two orientations, per axis, in degrees.

    Each orientation is an xyzw quaternion. We compare the corresponding columns
    (the world x/y/z axes) of the two rotation matrices, which is the same
    per-axis metric the previous servoing loop reported.

    Args:
        quat_a (Sequence[float]): First orientation as an xyzw quaternion.
        quat_b (Sequence[float]): Second orientation as an xyzw quaternion.

    Returns:
        list[float]: [x_angle, y_angle, z_angle] in degrees.
    """
    rot_a = pp.matrix_from_quat(quat_a)
    rot_b = pp.matrix_from_quat(quat_b)
    angles = []
    for axis in range(3):
        va = rot_a[:3, axis] / np.linalg.norm(rot_a[:3, axis])
        vb = rot_b[:3, axis] / np.linalg.norm(rot_b[:3, axis])
        dot = min(1.0, max(-1.0, float(np.dot(va, vb))))
        angles.append(float(np.degrees(np.arccos(dot))))
    return angles


def measure_servo_tool0_error(monitor, target_ee_frames):
    """Tool0 pose error of each arm at the live robot state vs the authored target.

    Forward kinematics is taken at the live mocap base composed with the live
    (last-executed) arm configuration -- exactly the quantity visual servoing
    drives to zero. Uses the same set_pose -> get_link_pose_from_name idiom used
    elsewhere in this module.

    Args:
        monitor: The HuskyMonitor node.
        target_ee_frames (dict): {"left": Frame, "right": Frame}, the authored
            world-frame tool0 targets as compas Frames.

    Returns:
        dict: Per side ("left"/"right") a dict with:
            "pos_err_mm" (list[float]): [dx, dy, dz] position error in millimetres.
            "pos_norm_mm" (float): position error magnitude in millimetres.
            "rot_err_deg" (list[float]): per-axis orientation error in degrees.
    """
    h = monitor.huskies[monitor.selected_robot_id]
    ho = h.object
    hi = h.interface
    # Sync the sim to the live mocap base + live arm joints, then read tool0 FK.
    ho.set_pose((hi.position, hi.rotation), hi.arm_joint_pose)
    link_from_side = {"left": "left_ur_arm_tool0", "right": "right_ur_arm_tool0"}
    out = {}
    for side, link_name in link_from_side.items():
        observed = ho.get_link_pose_from_name(link_name)
        target = pose_from_frame(target_ee_frames[side])
        pos_err_mm = (np.array(observed[0]) - np.array(target[0])) * 1e3
        rot_err_deg = _per_axis_axis_angles_deg(observed[1], target[1])
        out[side] = {
            "pos_err_mm": pos_err_mm.tolist(),
            "pos_norm_mm": float(np.linalg.norm(pos_err_mm)),
            "rot_err_deg": rot_err_deg,
        }
    return out


def measure_arm_tracking_tool0_error(monitor, planned_final_confs):
    """Tool0 gap between where the arms ARE and the last waypoint they were sent.

    ! Both poses are evaluated at the SAME live mocap base, so the result is
    ! purely the arms' joint-tracking error expressed at the tool flange -- it
    ! does not change when the mobile base drifts. That is what separates the two
    ! ways a servo iteration can miss its target:
    !   - large here                -> the arms never reached the commanded
    !                                  configuration (execution problem);
    !   - small here, but a large
    !     tool0-vs-target error     -> the arms tracked fine and the base moved
    !                                  between planning and measuring.
    The joint-space consistency check next to this one cannot make that call: its
    0.05 rad tolerance is roughly 50 mm of tool0 error at a UR5e's reach, which is
    the same order as the residual being chased here.

    Args:
        monitor: The HuskyMonitor node.
        planned_final_confs (Sequence): ``[left 6-vec, right 6-vec]`` -- the last
            waypoint of each arm's executed trajectory.

    Returns:
        dict: Per side ("left"/"right") the tool0 position gap in millimetres.
    """
    h = monitor.huskies[monitor.selected_robot_id]
    ho = h.object
    hi = h.interface
    base = (hi.position, hi.rotation)
    link_from_side = {"left": "left_ur_arm_tool0", "right": "right_ur_arm_tool0"}

    ho.set_pose(base, hi.arm_joint_pose)
    live = {s: ho.get_link_pose_from_name(ln) for s, ln in link_from_side.items()}
    ho.set_pose(base, list(planned_final_confs))
    planned = {s: ho.get_link_pose_from_name(ln) for s, ln in link_from_side.items()}
    # Put the sim back on the live pose so nothing downstream sees the probe pose.
    ho.set_pose(base, hi.arm_joint_pose)

    return {
        s: float(np.linalg.norm(
            (np.array(live[s][0]) - np.array(planned[s][0])) * 1e3))
        for s in link_from_side
    }


def measure_base_pose_diff(monitor, start_base_pose):
    """Live mocap base pose difference vs the servoing run's start base pose.

    Args:
        monitor: The HuskyMonitor node.
        start_base_pose (tuple): (position, quaternion_xyzw) captured at the start
            of the current servoing run.

    Returns:
        dict: {
            "pos_diff_mm" (list[float]): [dx, dy, dz] base translation drift in mm.
            "pos_norm_mm" (float): euclidean translation drift magnitude in mm.
            "rot_diff_deg" (list[float]): per-axis base orientation drift in degrees.
        }.
    """
    hi = monitor.huskies[monitor.selected_robot_id].interface
    pos_diff_mm = (np.array(hi.position) - np.array(start_base_pose[0])) * 1e3
    rot_diff_deg = _per_axis_axis_angles_deg(hi.rotation, start_base_pose[1])
    return {
        "pos_diff_mm": pos_diff_mm.tolist(),
        "pos_norm_mm": float(np.linalg.norm(pos_diff_mm)),
        "rot_diff_deg": rot_diff_deg,
    }


def _live_trajectory_time(monitor):
    """Duration (s) to run the next servoing move over, read live from the slider.

    ``reset_ui`` rebuilds the "traj time" slider on every live-base IK call, and
    a freshly rebuilt widget can miss its next drag callback -- so
    ``monitor.trajectory_time`` may still hold the movement role's default
    (5 s for M3, set by 'Load Movement') while the slider on screen shows what
    the operator actually dialled in. Reading the widget itself is what lets a
    drag made DURING the confirm pause take effect on the first move, which
    matters because that first transfer can be a 600 mm swing. Same live re-read
    as ``HuskyMonitor.exec_selected_movement_traj``.

    Args:
        monitor: The HuskyMonitor node.

    Returns:
        float: The slider's current value in seconds, also written back to
        ``monitor.trajectory_time``; falls back to ``monitor.trajectory_time``
        when the slider does not exist (headless harnesses).
    """
    sld = getattr(monitor, 'trajectory_time_slider', None)
    if sld is not None:
        v = sld.value
        if v is not None and float(v) != monitor.trajectory_time:
            monitor.trajectory_time = float(v)
            print(f"[servo] traj time from slider: {monitor.trajectory_time:.0f}s")
    return float(monitor.trajectory_time)


def _path_motion_summary(planned_arm_trajectory):
    """How long and how large a planned dual-arm move is.

    Args:
        planned_arm_trajectory: The monitor's ``planned_arm_trajectory`` pair;
            each entry is a ``(path, ..., ..., ...)`` tuple whose path is the
            list of 6-joint waypoints for that arm (``None`` when unplanned).

    Returns:
        tuple[int, float]: ``(n_waypoints, max_joint_delta_deg)`` from the first
        to the last waypoint across both arms, or ``(0, 0.0)`` when either arm
        has no path.
    """
    # A path exists when it is not None AND non-empty. Use `is not None` +
    # `len(...)` (never bool(array)/.any()/.all()): `is not None` is an
    # identity check that always returns a plain bool, and `len()` works on
    # both python lists and numpy arrays -- so neither trips the "truth value
    # of an array is ambiguous" error. The len guard also prevents an
    # IndexError on the [0] / [-1] accesses below.
    left_path, right_path = planned_arm_trajectory[0][0], planned_arm_trajectory[1][0]
    if (left_path is None or len(left_path) == 0
            or right_path is None or len(right_path) == 0):
        return 0, 0.0
    # Max joint delta between start (first waypoint) and end (last waypoint)
    max_delta_rad = max(
        float(np.max(np.abs(np.asarray(path[-1], dtype=float)
                            - np.asarray(path[0], dtype=float))))
        for path in (left_path, right_path)
    )
    return len(left_path), float(np.rad2deg(max_delta_rad))


def servo_to_movement_start_live(monitor, max_iters=8, pos_tol_mm=0.2,
                                 later_iter_traj_time=3.0, settle_seconds=2.0,
                                 confirm_first_iter=True, use_transfer=False,
                                 log_data=True):
    """Iterative visual-servoing loop to the current M2/M3 movement start pose.

    One iteration mirrors the manual three clicks: (1) live-base IK from the latest
    mocap pose (``ik_live_base_for_selected_movement``), (2) IK Replan & Transit to
    that goal (``replan_free_to_movement_start_live``), (3) Exec Both Arms; then wait
    for the arms to settle and measure the residual tool0 error at the live base +
    last conf. Because the arm
    motion shifts the base (its centre of gravity changes), each pass re-solves for
    the freshly-sensed base and the residual shrinks; we stop once both arms are
    within ``pos_tol_mm`` or after ``max_iters`` passes.

    With ``use_transfer=True``, step (2) plans a bar-held constrained TRANSFER
    instead (``replan_transfer_to_movement_start_live``, Button 2b): once the bar
    is manually mounted in the grippers it stays mounted across all iterations,
    so every correction move must keep both tool0s rigidly locked to the bar.

    Trajectory duration differs by iteration: the FIRST transit is a large move
    from wherever the arms are now, so it runs over the "traj time" slider's
    LIVE value (read via ``_live_trajectory_time`` right before the move, so a
    drag made during the confirm pause still counts -- 'Load Movement' resets
    that slider to the role default of 5 s, which is far too fast for the first
    transfer); every later iteration is a tiny near-target correction and runs
    over the short ``later_iter_traj_time``.

    This is a generator run as a monitor task (``monitor.tasks``); it yields between
    steps so the 20 Hz tick keeps flowing mocap while the arms move. Progress is
    logged per iteration and saved as a static plot + JSON at the end.

    Args:
        monitor: The HuskyMonitor node.
        max_iters (int): Hard cap on servoing iterations.
        pos_tol_mm (float): Stop once both arms' tool0 position error is below this.
        later_iter_traj_time (float): Trajectory duration (seconds) for iterations
            after the first. The first iteration uses ``monitor.trajectory_time``.
        settle_seconds (float): Extra time to wait after each trajectory finishes so
            the centre-of-gravity-shifted base settles in the mocap stream before we
            measure. Added on top of the trajectory's own duration.
        confirm_first_iter (bool): If True, pause after planning the first (long)
            move so the operator can preview it via the traj viz slider and click
            'Confirm Servo Exec' before it runs. Later iterations always run
            unattended. If False, the whole loop runs without any prompt.
        use_transfer (bool): If True, every iteration plans with the constrained
            dual-arm transfer planner (bar mounted in the grippers); if False,
            with the free composite transit planner (no bar).
        log_data (bool): Save a JSON + PNG record of the run when done.

    Yields:
        None: cooperative yields so the monitor tick keeps running.
    """
    h = monitor.huskies[monitor.selected_robot_id]
    hi = h.interface

    # ! Preconditions: this flow is dual-arm (M2/M3 bar-holding) on real hardware.
    if not h.dual_arm:
        monitor.get_logger().warn('Servoing loop needs a dual-arm robot.')
        return
    if monitor.FAKE_HARDWARE:
        monitor.get_logger().warn('Servoing loop needs real hardware (not FAKE_HARDWARE).')
        return
    mv = monitor.current_movement
    if mv is None or mv.start_state is None:
        monitor.get_logger().warn('Load the insert or the retreat movement first.')
        return
    kind = monitor._kind_of(mv)
    if kind not in COMPLIANT_KINDS:
        monitor.get_logger().warn(
            f'Servoing loop only supports the insert / retreat; {mv.movement_id} is a '
            f'{getattr(kind, "value", type(mv).__name__)}.')
        return

    # ! Keep the reference target constant across iterations. Button 2 mutates
    # mv.start_state.robot_base_frame to the live base (via
    # _apply_live_base_to_movement), and the target tool0 frames are re-derived by
    # FK at mv.start_state -- so we must restore the authored base before each
    # iteration, otherwise the target would drift with the base every pass.
    authored_base = copy.deepcopy(mv.start_state.robot_base_frame)

    # Reference base pose for the base-difference metrics (this run's start pose).
    start_base_pose = (copy.copy(hi.position), copy.copy(hi.rotation))

    # Blank + show the live DPG tracker window for this run.
    monitor.reset_servoing_tracker()

    # Clear any stale abort request so a leftover click can't cancel this run.
    # The 'Cancel Servo Loop' button sets this True; we poll it at each yield.
    monitor._servo_abort = False

    def _aborted():
        return getattr(monitor, '_servo_abort', False)

    data = []
    target = None

    def _record(iter_i):
        """Measure the residuals, log them, push to the live tracker, store them."""
        err = measure_servo_tool0_error(monitor, target)
        base = measure_base_pose_diff(monitor, start_base_pose)
        data.append({'iter': iter_i, 'tool0': err, 'base': base})
        monitor.push_servoing_tracker(iter_i, err, base)
        monitor.get_logger().info(
            f"[servo {iter_i}] tool0 pos L={err['left']['pos_norm_mm']:.2f} "
            f"R={err['right']['pos_norm_mm']:.2f} mm")
        return err

    # * the visual servoing iterations
    for it in range(1, max_iters + 1):
        if _aborted():
            break
        monitor.get_logger().info(f'### SERVOING iteration {it}/{max_iters}')
        # Clear any stale plan first so a failed replan (which returns without
        # updating the trajectory) is detected below instead of re-executing the
        # previous iteration's path.
        monitor.planned_arm_trajectory[0] = (None, None, None, None)
        monitor.planned_arm_trajectory[1] = (None, None, None, None)
        # Restore the authored base so both the IK below and the replan derive the
        # fixed authored target (Button 2's live-base step mutates it each pass).
        mv.start_state.robot_base_frame = copy.deepcopy(authored_base)
        # Step 1: recompute IK from the LATEST mocap pose (Button 1:
        # ik_live_base_for_selected_movement) so the arm goal is re-solved for
        # wherever the base has settled this pass. Then Step 2 plans the transit.
        if not monitor.ik_live_base_for_selected_movement():
            monitor.get_logger().warn('Servoing: live-base IK failed; stopping.')
            break
        # Step 2: replan the move to that goal — a bar-held constrained transfer
        # (Button 2b) when the bar is mounted, else a free transit (Button 2).
        # The transfer safeguard plot is only worth showing on the first (large)
        # move; later iterations are tiny near-target corrections, so suppress
        # the validation plot there (it == 1 only).
        if use_transfer:
            monitor.replan_transfer_to_movement_start_live(show_validation=(it == 1))
        else:
            monitor.replan_free_to_movement_start_live()

        # Bail if IK / plan produced no usable trajectory. A path is missing when
        # it is None OR empty. Use `is None` + `len(...)` (never bool(array)) so a
        # numpy path never trips the "truth value of an array is ambiguous" error.
        pat = monitor.planned_arm_trajectory
        left_path, right_path = pat[0][0], pat[1][0]
        if (left_path is None or len(left_path) == 0
                or right_path is None or len(right_path) == 0):
            monitor.get_logger().warn('Servoing: replan produced no trajectory; stopping.')
            break

        # Capture the constant target once (the frames the IK just solved to) and
        # record the pre-execution baseline as iteration 0.
        if target is None:
            target = copy.deepcopy(monitor._last_ik_target_ee_frames)
            if not target or 'left' not in target or 'right' not in target:
                monitor.get_logger().warn('Servoing: no target EE frames; stopping.')
                break
            _record(0)

        # The first transit is a big move from wherever the arms are now: pause
        # (while still yielding, so the traj viz slider keeps previewing the
        # planned path) until the operator clicks 'Confirm Servo Exec'. Later
        # iterations are short near-target corrections and run unattended.
        if it == 1 and confirm_first_iter:
            n_waypoints, max_delta_deg = _path_motion_summary(
                monitor.planned_arm_trajectory)
            yield from wait_for_operator_confirm(
                monitor,
                f"[servo] First move planned: {n_waypoints} waypoints, max joint "
                f"delta {max_delta_deg:.1f} deg, over "
                f"{_live_trajectory_time(monitor):.0f} s. Preview it with the "
                f"traj viz slider; drag 'traj time' now if that is too fast (it "
                f"is re-read when you confirm), then click 'Confirm Exec' to "
                f"execute (later iterations run automatically).")

        # Abort requested during the confirm pause (or otherwise): stop before
        # sending the first move to the robot.
        if _aborted():
            break

        # * Safeguard: any iteration's plan with >10 waypoints or >5° max joint
        # * delta requires operator confirmation (prevents unexpected large moves).
        # * This catches servo iterations where stale state might cause wrong plans.
        n_waypoints, max_delta_deg = _path_motion_summary(
            monitor.planned_arm_trajectory)
        if n_waypoints:
            needs_confirm = (n_waypoints > 10 or max_delta_deg > 5.0)
            if needs_confirm and (it > 1 or not confirm_first_iter):
                # Iteration > 1 OR first iter without confirm: safeguard pause
                yield from wait_for_operator_confirm(
                    monitor,
                    f'[servo] SAFEGUARD: large/long motion detected iter {it}: '
                    f'{n_waypoints} waypoints, max joint delta {max_delta_deg:.1f}°. '
                    f'Preview with the slider, then click "Confirm Exec" to proceed.',
                    warn=True,
                )
                if _aborted():
                    break

        # Execution is starting now (the first move was confirmed above): expand
        # the live tracker, which was kept collapsed during the preview. Later
        # iterations stay visible across UI rebuilds via _servoing_tracker_visible.
        if it == 1:
            monitor.show_servoing_tracker()

        # First transit is a big move (use the slider time, re-read here so a
        # drag made during the confirm pause counts); later iterations are tiny
        # near-target corrections (use the short later_iter_traj_time).
        traj_time = (_live_trajectory_time(monitor) if it == 1
                     else later_iter_traj_time)

        # Execute both arms, then wait out the whole motion + a settle margin.
        # ! We wait by TIME, not by hi.is_arm_executing: that flag clears after
        # only ARM_NOT_EXECUTING_TIME (1 s) of no joint change, so a controller
        # that is slow to start a motion trips it and the next iteration would
        # fire while the arm is still moving. The trajectory was sent over
        # traj_time, so waiting that long (+ settle) is reliable.
        execute_arm_trajectory_both(monitor, traj_time)
        tick_dt = 0.05  # matches the 20 Hz monitor tick that pumps this task
        wait_ticks = int(np.ceil((traj_time + settle_seconds) / tick_dt))
        for _ in range(wait_ticks):
            # Abort stops the loop, but the trajectory already sent to the robot
            # keeps running to completion (there is no in-flight cancel here).
            if _aborted():
                break
            yield
        if _aborted():
            break

        # * Read the actual robot config from the live Husky interface, then
        # * safeguard-check it against the executed trajectory's final point.
        yield  # let monitor tick to ensure fresh joint state
        try:
            hi = monitor.huskies[monitor.selected_robot_id].interface
            # Update start_state from live arm pose (same pattern as _apply_live_base_to_movement)
            for i, names in enumerate(monitor._arm_joint_name_sets()):
                values = hi.arm_joint_pose[i] if len(hi.arm_joint_pose) > i else hi.arm_joint_pose[0]
                for n, v in zip(names, values):
                    mv.start_state.robot_configuration[n] = float(v)

            # Safeguard: check numerical consistency with the executed trajectory.
            # No "does a path exist?" guard here -- we only reach this point after
            # the plan passed the checks above and was sent to the robot, so both
            # arm paths are known to be present and non-empty. (The old guard did
            # `if pat[0][0] ...`, which called bool() on a numpy path and raised
            # "truth value of an array with more than one element is ambiguous".)
            pat = monitor.planned_arm_trajectory
            final_left = np.asarray(pat[0][0][-1], dtype=float)
            final_right = np.asarray(pat[1][0][-1], dtype=float)
            live_left = np.asarray(hi.arm_joint_pose[0], dtype=float)
            live_right = np.asarray(hi.arm_joint_pose[1] if len(hi.arm_joint_pose) > 1 else hi.arm_joint_pose[0], dtype=float)

            left_error = float(np.max(np.abs(final_left - live_left)))
            right_error = float(np.max(np.abs(final_right - live_right)))
            tol_rad = 0.05  # ~3 degrees; warning if larger

            if left_error > tol_rad or right_error > tol_rad:
                monitor.get_logger().warn(
                    f'[servo] WARNING: execution mismatch iter {it}: '
                    f'left {np.rad2deg(left_error):.1f}°, right {np.rad2deg(right_error):.1f}° '
                    f'(traj vs live interface)'
                )
            else:
                monitor.get_logger().info(
                    f'[servo] Synced to live pose; consistency check OK '
                    f'(L {np.rad2deg(left_error):.1f}°, R {np.rad2deg(right_error):.1f}°)'
                )

            # * The same check expressed at the TOOL FLANGE, in millimetres, at
            # * the live base. Read this next to the '[servo N] tool0 pos' line
            # * that follows: a small gap here with a large tool0-vs-target
            # * error means the arms tracked their command and the world moved
            # * under them (base / mocap), while a large gap here means the
            # * arms simply did not arrive.
            track = measure_arm_tracking_tool0_error(
                monitor, [final_left, final_right])
            monitor.get_logger().info(
                f"[servo {it}] arms vs last commanded waypoint: "
                f"L={track['left']:.2f} R={track['right']:.2f} mm"
            )
        except Exception as e:
            monitor.get_logger().warn(f'[servo] Failed to sync live config: {e}')

        err = _record(it)
        if max(err['left']['pos_norm_mm'], err['right']['pos_norm_mm']) < pos_tol_mm:
            monitor.get_logger().info(
                f'### SERVOING converged at iter {it} (< {pos_tol_mm} mm)')
            break

    if _aborted():
        monitor.get_logger().info('### SERVOING cancelled by user.')

    # Leave the movement's authored base restored (clean) for downstream users.
    mv.start_state.robot_base_frame = authored_base

    # Save only when at least one move ran (baseline + >=1 executed iteration).
    # A run cancelled during the confirm pause has just the baseline point and
    # isn't worth a record.
    if log_data and len(data) > 1:
        _save_servoing_data(monitor, data,
                            planner_mode='transfer' if use_transfer else 'free')


def _save_servoing_data(monitor, data, planner_mode='free'):
    """Save a servoing run's per-iteration residuals as JSON + a summary PNG.

    Writes into a dated ``<YYYYMMDD>-servoing`` subfolder of the Google Drive
    experiment data dir (BAR_HOLDING_ACC_EXPERIMENT_DIR), alongside the marker
    takes for this experiment.

    Args:
        monitor: The HuskyMonitor node (for logging only).
        data (list[dict]): One entry per recorded iteration, each with keys
            'iter', 'tool0' (per-side pos/rot error) and 'base' (base drift).
        planner_mode (str): 'free' (transit, no bar) or 'transfer' (bar-held
            constrained) — recorded in the JSON so offline analysis can tell
            the two run kinds apart.
    """
    subfolder = os.path.join(BAR_HOLDING_ACC_EXPERIMENT_DIR,
                             f"{datetime.now().strftime('%Y%m%d')}-servoing")
    os.makedirs(subfolder, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    data_filename = os.path.join(subfolder, f'servoing_data_{timestamp}.json')
    with open(data_filename, 'w') as f:
        json.dump({'planner_mode': planner_mode, 'servoing_data': data}, f, indent=2)
    monitor.get_logger().info(f'Servoing data saved to {data_filename}')

    # ! Build the figure with the pure Agg backend (Figure + FigureCanvasAgg),
    # NOT pyplot. pyplot picks an interactive backend (Qt/Tk) whose figure manager
    # runs its own GUI loop; creating one from inside the live monitor's DearPyGui
    # + PyBullet event loop crashes the app. The object-oriented Figure API only
    # renders to a PNG and never touches any GUI, so it is safe here.
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg

    iters = [d['iter'] for d in data]
    axis_names = ('x', 'y', 'z')
    # Left arm = red shades, right arm = green shades (per axis), matching the
    # DPG live tracker. Convert the shared 0-255 tuples to matplotlib 0-1.
    side_rgb = {'left': SERVO_LEFT_ARM_RGB, 'right': SERVO_RIGHT_ARM_RGB}
    def _arm_color(side, a):
        return tuple(v / 255.0 for v in side_rgb[side][a])
    def _base_color(a):
        return tuple(v / 255.0 for v in SERVO_BASE_RGB[a])

    fig = Figure(figsize=(13, 9))
    FigureCanvasAgg(fig)
    axes = fig.subplots(2, 2)

    # Tool0 position error, per axis + euclidean norm, both arms (mm).
    for side in ('left', 'right'):
        for a, ax_name in enumerate(axis_names):
            axes[0, 0].plot(iters, [d['tool0'][side]['pos_err_mm'][a] for d in data],
                            'o-', color=_arm_color(side, a),
                            label=f'{side.capitalize()} {ax_name}')
        axes[0, 0].plot(iters, [d['tool0'][side]['pos_norm_mm'] for d in data],
                        'o-', linewidth=2.2, color=_arm_color(side, 3),
                        label=f'{side.capitalize()} |d|')
    axes[0, 0].set_title('Tool0 position error (per axis + |d|)')
    axes[0, 0].set_xlabel('iteration'); axes[0, 0].set_ylabel('mm')
    axes[0, 0].legend(fontsize=7, ncol=2); axes[0, 0].grid(True)

    # Tool0 orientation error, per axis, both arms (deg).
    for side in ('left', 'right'):
        for a, ax_name in enumerate(axis_names):
            axes[0, 1].plot(iters, [d['tool0'][side]['rot_err_deg'][a] for d in data],
                            'o-', color=_arm_color(side, a),
                            label=f'{side.capitalize()} {ax_name}')
    axes[0, 1].set_title('Tool0 orientation error (per axis)')
    axes[0, 1].set_xlabel('iteration'); axes[0, 1].set_ylabel('deg')
    axes[0, 1].legend(fontsize=7, ncol=2); axes[0, 1].grid(True)

    # Mobile-base position drift vs run start, per axis + euclidean norm (mm).
    for a, ax_name in enumerate(axis_names):
        axes[1, 0].plot(iters, [d['base']['pos_diff_mm'][a] for d in data],
                        'o-', color=_base_color(a), label=ax_name)
    axes[1, 0].plot(iters, [d['base']['pos_norm_mm'] for d in data],
                    'o-', linewidth=2.2, color=_base_color(3), label='|d|')
    axes[1, 0].set_title('Base position drift vs run start (per axis + |d|)')
    axes[1, 0].set_xlabel('iteration'); axes[1, 0].set_ylabel('mm')
    axes[1, 0].legend(fontsize=8); axes[1, 0].grid(True)

    # Mobile-base orientation drift vs run start, per axis (deg).
    for a, ax_name in enumerate(axis_names):
        axes[1, 1].plot(iters, [d['base']['rot_diff_deg'][a] for d in data],
                        'o-', color=_base_color(a), label=ax_name)
    axes[1, 1].set_title('Base orientation drift vs run start (per axis)')
    axes[1, 1].set_xlabel('iteration'); axes[1, 1].set_ylabel('deg')
    axes[1, 1].legend(fontsize=8); axes[1, 1].grid(True)

    fig.tight_layout()
    plot_filename = os.path.join(subfolder, f'servoing_performance_{timestamp}.png')
    fig.savefig(plot_filename)
    monitor.get_logger().info(f'Servoing performance plot saved to {plot_filename}')

def move_base_to_goal(monitor):
    if monitor.planned_base_trajectory[0] is None:
        monitor.get_logger().warn('Base trajectory must be planed before executing!')
        return
    monitor.tasks.append(control.execute_base_trajectory(monitor, monitor.huskies[0], monitor.planned_base_trajectory))
    
#################################

# * Robotiq gripper (support robots) command values, shared by the buttons below
# and the schedule's gripper steps (exec_gripper_tool_movement).
GRIPPER_OPEN_POS = 0.426          # 'Open Gripper Full'
GRIPPER_CLOSE_FOR_BAR_POS = 0.8   # 'Close Gripper for Bar'
GRIPPER_EFFORT = 0.1
# How long a gripper step waits for the gripper's final answer after a plain
# open / close. The stroke itself takes about a second.
GRIPPER_RESULT_TIMEOUT_S = 5.0


def open_gripper_full(monitor: 'HuskyMonitor') -> bool:
    """Open the active robot's gripper (arm 0) all the way.

    Args:
        monitor (HuskyMonitor): Provides the active husky's interface.

    Returns:
        bool: True if the goal was sent (see ``HuskyRobotInterface.send_gripper_cmd``).
    """
    return monitor.huskies[monitor.selected_robot_id].interface.send_gripper_cmd(GRIPPER_OPEN_POS, GRIPPER_EFFORT)

def close_gripper_for_bar(monitor: 'HuskyMonitor') -> bool:
    """Close the active robot's gripper (arm 0) onto a bar.

    Args:
        monitor (HuskyMonitor): Provides the active husky's interface.

    Returns:
        bool: True if the goal was sent (see ``HuskyRobotInterface.send_gripper_cmd``).
    """
    return monitor.huskies[monitor.selected_robot_id].interface.send_gripper_cmd(GRIPPER_CLOSE_FOR_BAR_POS, GRIPPER_EFFORT)

def set_gripper(monitor: 'HuskyMonitor') -> None:
    """Send the active robot's gripper (arm 0) to the position set on the gripper slider.

    Args:
        monitor (HuskyMonitor): Provides the active husky's interface and
            ``goal_gripper`` (the slider value).
    """
    monitor.huskies[monitor.selected_robot_id].interface.send_gripper_cmd(monitor.goal_gripper, GRIPPER_EFFORT)

####################################

def execute_arm_trajectory_all(monitor: 'HuskyMonitor', traj_time: float = None) -> None:
    """Execute every arm's planned trajectory over ``traj_time`` seconds.

    Two arms (Cindy) go out together as one dual-arm command; a single-arm
    robot (Alice / Belle) gets one plain arm command on arm 0.

    Args:
        monitor: The HuskyMonitor node.
        traj_time (float): Duration to run the trajectory over. Defaults to
            ``monitor.trajectory_time`` when None (the slider-set value); the
            servoing loop passes a short time for its near-target iterations.
    """
    spec = monitor._connected_robot()
    n = spec.n_arms
    # Cindy keeps her historical [LEFT] / [RIGHT] tags in the warning.
    arm_tags = [side.upper() for side in spec.side_keys]
    for i in range(n):
        if monitor.planned_arm_trajectory[i][0] is None:
            monitor.get_logger().warn(f'Arm trajectory must be planed before executing! [{arm_tags[i]}]')
            return

    traj_time = monitor.trajectory_time if traj_time is None else traj_time

    if not monitor.FAKE_HARDWARE:
        # Stamp the requested duration onto every arm's planned_arm_trajectory tuple.
        for i in range(n):
            monitor.planned_arm_trajectory[i] = (
                monitor.planned_arm_trajectory[i][0],
                monitor.planned_arm_trajectory[i][1],
                traj_time,
                monitor.planned_arm_trajectory[i][3]
            )
        hi = monitor.huskies[monitor.selected_robot_id].interface
        if n == 2:
            hi.send_dual_arm_cmd(monitor.planned_arm_trajectory)
        else:
            path, vel, t, _ = monitor.planned_arm_trajectory[0]
            hi.send_arm_cmd(path, vel, t, index=0)
    else:
        _fake_execute_arm_trajectories(
            monitor, [monitor.planned_arm_trajectory[i] for i in range(n)], traj_time)


def _fake_execute_arm_trajectories(monitor: 'HuskyMonitor', trajectories: list,
                                   traj_time: float) -> None:
    """FAKE_HARDWARE: play the arm trajectories on the simulated robot (no ROS command).

    The simulated arms are the connected husky's ``interface.arm_joint_pose``;
    they step through the waypoints, all arms together, and objects attached
    to an arm follow its flange. Blocks for about ``traj_time`` seconds.

    Args:
        monitor (HuskyMonitor): The monitor (its connected husky moves).
        trajectories (list): One ``(path, velocities, time, attached object)``
            tuple per arm, as in ``monitor.planned_arm_trajectory``.
        traj_time (float): Seconds to spread the waypoints over.
    """
    spec = monitor._connected_robot()
    n = len(trajectories)
    ho = monitor.huskies[monitor.selected_robot_id].object
    hi = monitor.huskies[monitor.selected_robot_id].interface

    # Objects attached to each arm, and where each sits relative to the flange
    attached = [traj[3] for traj in trajectories]
    tcp_from_object = [obj.grasp if obj is not None else None for obj in attached]

    # Execute all trajectories simultaneously
    max_points = max(len(traj[0]) for traj in trajectories)

    # Spread the waypoints over the requested trajectory time so fake
    # execution takes as long as the real robot would (mirrors the
    # real-hardware dt = traj_time / (n - 1) in husky_robot.py).
    step_dt = traj_time / max(max_points - 1, 1)

    for k in range(max_points):
        # Update each arm's configuration
        for i, traj in enumerate(trajectories):
            if k < len(traj[0]):
                hi.arm_joint_pose[i] = traj[0][k]

        # Update robot pose
        ho.set_pose((hi.position, hi.rotation), hi.arm_joint_pose)

        # Update attached objects based on FK
        for i, traj in enumerate(trajectories):
            if attached[i] is not None and k < len(traj[0]):
                world_from_tcp = ho.get_link_pose_from_name(spec.flange_links[i])
                attached[i].set_pose(pp.multiply(world_from_tcp, tcp_from_object[i]))

        # Set execution flags
        for i in range(n):
            hi.is_arm_executing[i] = True

        pp.wait_for_duration(step_dt)

    # Clear execution flags
    for i in range(n):
        hi.is_arm_executing[i] = False


# The historical dual-arm name; every Cindy caller still uses it.
execute_arm_trajectory_both = execute_arm_trajectory_all

def load_robotcellstate_and_update_goal(monitor, filepath):
    """
    Loads a RobotCellState from a JSON file using compas.json_load,
    and updates the arm goal configuration for both arms in the monitor.
    """
    robot_cell_state = compas.json_load(filepath)
    if not isinstance(robot_cell_state, RobotCellState):
        monitor.get_logger().warn(f"File {filepath} did not contain a RobotCellState.")
        return
    # Update the arm goal configuration for both arms
    # robot_cell_state.robot_configuration.data['joint_values'] is a list of all joint values
    # The robot configuration is a compas JointConfiguration, which contains .joint_names and .joint_values
    joint_config = robot_cell_state.robot_configuration
    joint_names = getattr(joint_config, 'joint_names', None)
    joint_values = getattr(joint_config, 'joint_values', None)
    if joint_names is None or joint_values is None:
        monitor.get_logger().warn(f"Robot configuration does not contain 'joint_names' or 'joint_values'.")
        return

    # Get the expected joint names for each arm
    left_arm_joint_names = monitor.huskies[monitor.selected_robot_id].object.get_arm_joint_names(index=0)
    right_arm_joint_names = monitor.huskies[monitor.selected_robot_id].object.get_arm_joint_names(index=1)

    # Map joint names to values
    joint_map = dict(zip(joint_names, joint_values))

    # Assign values to each arm in the correct order
    try:
        left_arm_values = [joint_map[name] for name in left_arm_joint_names]
        right_arm_values = [joint_map[name] for name in right_arm_joint_names]
        monitor.goal_arm_pose[0] = np.array(left_arm_values)
        monitor.goal_arm_pose[1] = np.array(right_arm_values)
        monitor.get_logger().info(f"Loaded RobotCellState from {filepath} and updated both arm goal configurations.")
        monitor.reset_ui()  # Optionally reset UI to reflect new goals
    except KeyError as e:
        monitor.get_logger().warn(f"Joint name {e} not found in loaded RobotCellState.")

def compute_tool0_to_tool0_transform_from_json(json_filepath):
    """
    Parse the JSON file containing GraspTarget objects and compute the tool0_to_tool0 transformation.
    
    Parameters:
    -----------
    json_filepath : str
        Path to the JSON file containing GraspTarget objects
        
    Returns:
    --------
    pp.Pose
        Transformation from first tool0 to second tool0
    """
    import json
    import numpy as np
    
    # Load the JSON file
    with open(json_filepath, 'r') as f:
        grasp_targets = json.load(f)
    
    if len(grasp_targets) < 2:
        raise ValueError("JSON file must contain at least 2 GraspTarget objects")
    
    # Extract the world_from_tool0 transformations
    world_from_tool0_1_matrix = np.array(grasp_targets[0]["data"]["world_from_tool0"]["data"]["matrix"])
    world_from_tool0_2_matrix = np.array(grasp_targets[1]["data"]["world_from_tool0"]["data"]["matrix"])
    world_from_bar_matrix = np.array(grasp_targets[1]["data"]['world_from_bar']['data']['matrix'])
    
    # Convert to pybullet_planning poses
    world_from_tool0_1 = pp.pose_from_tform(world_from_tool0_1_matrix)
    world_from_tool0_2 = pp.pose_from_tform(world_from_tool0_2_matrix)
    world_from_bar = pp.pose_from_tform(world_from_bar_matrix)
    
    # Compute tool0_1_from_tool0_2 = world_from_tool0_1 * tool0_2_from_world
    # tool0_2_from_world = inverse(world_from_tool0_2)
    tool0_1_from_world = pp.invert(world_from_tool0_1)
    # tool0_2_from_world = pp.invert(world_from_tool0_2)
    tool0_1_from_tool0_2 = pp.multiply(tool0_1_from_world, world_from_tool0_2)
    tool0_2_from_bar = pp.multiply(pp.invert(world_from_tool0_2), world_from_bar)
    
    # print(f"Tool0_1 pose: {world_from_tool0_1}")
    # print(f"Tool0_2 pose: {world_from_tool0_2}")
    # print(f"Tool0_1_from_Tool0_2 transformation: {tool0_1_from_tool0_2}")
    
    return tool0_1_from_tool0_2, tool0_2_from_bar

@contextlib.contextmanager
def _free_planner_skip_env_collisions(enable):
    """Temporarily relax ``plan_free_dual_arm``'s internal collision predicate
    to check ONLY robot self-collision (CC.1). Skips robot<->tool (CC.2) and
    all environment checks: link<->rigid-body (CC.3), attached-rb<->rigid-body
    (CC.4), tool<->rigid-body (CC.5).

    ``plan_free_dual_arm`` builds its collision fn via the module-level
    ``_build_cfab_collision_fn`` (which hardcodes full-CC options), so we
    swap that symbol for the duration of the call and restore it after.
    A no-op when ``enable`` is False.

    Args:
        enable (bool): When True, patch in the self-collision-only fn.
    """
    from husky_assembly_tamp.motion_planner import api as _api
    if not enable:
        yield
        return
    _orig = _api._build_cfab_collision_fn

    def _patched(planner, template_state, joint_names_12):
        cc_opts = {"verbose": False, "_skip_cc2": True, "_skip_cc3": True,
                   "_skip_cc4": True, "_skip_cc5": True}

        def _fn(conf_12, *_a, **_k):
            s = _api._state_with_conf12(template_state, conf_12, joint_names_12)
            try:
                planner.check_collision(s, options=cc_opts)
            except _api.CollisionCheckError:
                return True
            return False

        return _fn

    _api._build_cfab_collision_fn = _patched
    try:
        yield
    finally:
        _api._build_cfab_collision_fn = _orig


def plan_both_arms_to_goal(monitor, use_composite=False, debug=False,
                           skip_env_collisions=False):
    """
    Plan motions for both arms from current to goal joint configurations.
    If use_composite is False, plan left then right sequentially.
    If True, plan in the composite joint space.
    Sets the resulting trajectories in the monitor.

    Args:
        skip_env_collisions (bool): Composite branch only. When True, the
            free-motion planner checks ONLY robot self-collision (CC.1),
            skipping robot<->tool (CC.2) and all environment checks
            (CC.3/4/5). Temporary escape hatch for the live M2/M3 replan
            button.
    """
    husky = monitor.huskies[monitor.selected_robot_id]
    left_conf = np.array(monitor.goal_arm_pose[0])
    right_conf = np.array(monitor.goal_arm_pose[1])
    print(f"target left_conf: {left_conf}")
    print(f"target right_conf: {right_conf}")

    left_trajectory = None
    right_trajectory = None

    if not use_composite:
        # Sequential planning uses the legacy pp-side planner, which needs
        # husky.object.robot (real Husky wrapper). The composite branch runs
        # entirely through cfab and doesn't need it, so we lazy-open the pp
        # handles only here.
        robot = husky.object.robot
        left_joint_names = HUSKY_DUAL_UR5e_JOINT_NAMES[0]
        right_joint_names = HUSKY_DUAL_UR5e_JOINT_NAMES[1]
        left_joints = pp.joints_from_names(robot, left_joint_names)
        right_joints = pp.joints_from_names(robot, right_joint_names)

        current_left_conf = pp.get_joint_positions(robot, left_joints)
        current_right_conf = pp.get_joint_positions(robot, right_joints)
        print("Current left arm joint configuration:", current_left_conf)
        print("Current right arm joint configuration:", current_right_conf)

        # Print joint limits for all arm joints
        all_joint_names = left_joint_names + right_joint_names
        all_joints = pp.joints_from_names(robot, all_joint_names)
        lower_limits = [pp.get_joint_info(robot, j).jointLowerLimit for j in all_joints]
        upper_limits = [pp.get_joint_info(robot, j).jointUpperLimit for j in all_joints]
        print("All arm joint names:", all_joint_names)
        print("All arm joint lower limits:", lower_limits)
        print("All arm joint upper limits:", upper_limits)

        attachments = [ee[1] for ee in husky.object.ee_list]
        obstacles = _get_manual_staging_obstacles(monitor)

        # Sequential planning: left arm, then right arm
        pp.set_joint_positions(robot, left_joints, current_left_conf)
        left_trajectory = planning.plan_arm_motion(
            husky, left_conf, obstacles, monitor.trajectory_time, arm_index=0, debug=debug
        )
        if left_trajectory[0] is None:
            monitor.get_logger().warn('Left arm planning failed!')
            return
        # Set left arm to end conf, right arm to current
        pp.set_joint_positions(robot, left_joints, left_trajectory[0][-1])
        pp.set_joint_positions(robot, right_joints, current_right_conf)
        right_trajectory = planning.plan_arm_motion(
            husky, right_conf, obstacles, monitor.trajectory_time, arm_index=1, debug=debug
        )
        if right_trajectory[0] is None:
            monitor.get_logger().warn('Right arm planning failed!')
            return
        
        # Create composite trajectories to show proper timing
        # Left arm moves first, then right arm moves while left arm holds its final position
        left_path = left_trajectory[0]
        right_path = right_trajectory[0]
        
        # Pad left trajectory with its final configuration for the duration of right arm movement
        left_final_conf = left_path[-1]
        padded_left_path = np.vstack([left_path, np.tile(left_final_conf, (len(right_path), 1))])
        
        # Pad right trajectory with its initial configuration for the duration of left arm movement
        right_initial_conf = right_path[0]  # This should be current_right_conf
        padded_right_path = np.vstack([np.tile(right_initial_conf, (len(left_path), 1)), right_path])
        
        # Create composite trajectories with proper timing
        total_time = monitor.trajectory_time * 2  # Total time for both movements
        left_trajectory = (padded_left_path, None, total_time, None)
        right_trajectory = (padded_right_path, None, total_time, None)
    else:
        # Composite planning: plan in the joint space of both arms through
        # the cfab planner. The start state is the loaded movement's state
        # (if any) or the startup default cell state, with the live robot
        # pose injected — obstacles + ACM come from that state.
        if monitor.cfab is None:
            monitor.get_logger().warn(
                "Composite planning needs the cfab session (created at "
                "startup); it is not available.")
            return
        template = getattr(monitor, 'movement_start_state', None) \
            or getattr(monitor, 'cfab_default_state', None)
        if template is None:
            monitor.get_logger().warn(
                "Composite planning: no movement start_state or default cell "
                "state available.")
            return
        state = template.copy()
        monitor._inject_live_conf_into_state(state)

        # * Unwrap the goal joint values to be within +/- pi of the start
        # conf. Trac_IK can return goal joints that are 2*pi-offset from the
        # nearest branch (e.g. right_shoulder ~ -4.12 rad when +2.16 rad
        # yields the same tool0 pose). BiRRT then has to traverse > pi in
        # that joint, which the 12-DOF free sampler almost never solves in
        # the time budget. The M0 transit-failure probe in
        # scripts/headless_live_monitor_test.py showed this decisively:
        # planning to the raw goal failed at 120s; planning to the
        # canonical (+/- pi of start) goal succeeded quickly. UR5e joints
        # have limits well past 2*pi so an unwrap is always in-range.
        left_joint_names = HUSKY_DUAL_UR5e_JOINT_NAMES[0]
        right_joint_names = HUSKY_DUAL_UR5e_JOINT_NAMES[1]
        start_left = np.asarray(
            [float(state.robot_configuration[n]) for n in left_joint_names],
            dtype=float,
        )
        start_right = np.asarray(
            [float(state.robot_configuration[n]) for n in right_joint_names],
            dtype=float,
        )
        start_12 = np.concatenate([start_left, start_right])
        raw_goal_12 = np.concatenate([left_conf, right_conf])
        two_pi = 2.0 * np.pi
        unwrapped_goal_12 = raw_goal_12 - np.round((raw_goal_12 - start_12) / two_pi) * two_pi
        max_wrap_delta = float(np.max(np.abs(raw_goal_12 - unwrapped_goal_12)))
        if max_wrap_delta > 1e-6:
            wrapped_joints = np.where(np.abs(raw_goal_12 - unwrapped_goal_12) > 1e-6)[0]
            print(
                f"[composite plan] unwrapped goal to +/- pi of start "
                f"(max wrap correction {max_wrap_delta:.4f} rad on "
                f"{len(wrapped_joints)} joint(s) at indices {wrapped_joints.tolist()})."
            )

        # * Pass a compas Configuration as the goal so `_conf12_from_target` in
        # the tamp API takes its dict-style path — raw numpy arrays raise
        # IndexError under string joint-name indexing there.
        composite_goal = conf_from_12vec(unwrapped_goal_12)
        # * Log the joint-delta magnitudes so we can tell whether the
        # sampler was fighting large deltas vs a genuinely blocked path.
        max_j_delta = float(np.max(np.abs(unwrapped_goal_12 - start_12)))
        l2_delta = float(np.linalg.norm(unwrapped_goal_12 - start_12))
        print(f"[composite plan] joint deltas: max={max_j_delta:.3f} rad, "
              f"L2={l2_delta:.3f} rad.")
        
        # TODO I want the opposite, try corase first, and if succeeds move on to the final res, but don't jhust return on the coarse  - that's a quick proxy for feasibility
        # * Try a fine resolution first; if that fails, fall back to a
        # coarse resolution (2x). BiRRT with a fine resolution rejects
        # motions with narrow-passage collisions; a coarser resolution is
        # more forgiving and lets the sampler cover more ground per second,
        # at the cost of possibly skipping over small obstacles. When both
        # start and goal are collision-free (they are here -- the goal IK only
        # returns a conf whose merged state passed the collision check), the
        # coarse pass is a reasonable escape.
        from husky_assembly_tamp.motion_planner.api import plan_free_dual_arm
        if skip_env_collisions:
            print("[composite plan] collision checks RELAXED to robot "
                  "self-collision only (skipping robot<->tool + environment).")
        composite_path = None
        # Pause GUI rendering during the BiRRT sampling (no-op when headless);
        # each sample pushes a cfab cell state onto the shared GUI client,
        # which otherwise redraws the robot on every step.
        with pp.LockRenderer(), _free_planner_skip_env_collisions(skip_env_collisions):
            for jr, mt, tag in [
                (FREE_JOINT_RESOLUTION, 120.0, 'fine'),
                (FREE_JOINT_RESOLUTION * 2.0, 300.0, 'coarse'),
            ]:
                print(f"[composite plan] {tag} pass: joint_resolution={jr:.3f} rad, "
                      f"max_time={mt:.0f}s.")
                composite_path, info = plan_free_dual_arm(
                    monitor.cfab.planner, state, composite_goal,
                    max_time=mt, max_iterations=2000,
                    joint_resolution=jr,
                    debug=debug,
                )
                if composite_path is not None:
                    print(f"[composite plan] {tag} pass SUCCEEDED "
                          f"({len(composite_path)} waypoints).")
                    break
                print(f"[composite plan] {tag} pass failed: "
                      f"{info.get('failure_reason', 'unknown')}.")
        if composite_path is None:
            monitor.get_logger().warn(
                f"Composite planning failed: {info.get('failure_reason', 'unknown')}."
            )
            return
        left_trajectory = (np.array([q[:6] for q in composite_path]), None, monitor.trajectory_time, None)
        right_trajectory = (np.array([q[6:] for q in composite_path]), None, monitor.trajectory_time, None)

    # Set the trajectories for both arms
    monitor.set_arm_trajectory(left_trajectory, index=0)
    monitor.set_arm_trajectory(right_trajectory, index=1)
    monitor.set_to_show_traj_state()
    print("Successfully planned both arms to goal ({} mode)!".format('composite' if use_composite else 'sequential'))


def _get_manual_staging_obstacles(monitor):
    """Obstacles for free staging; mirror constrained-start validation."""
    import re as _re

    bar_name_re = _re.compile(r"^b\d+(_0|_joint_\d+)$")
    excluded = set()
    active_bar_body = getattr(monitor, "active_bar_body", None)
    if active_bar_body is not None:
        excluded.add(active_bar_body)
    excluded.update(getattr(monitor, "active_extra_bodies", []) or [])

    obstacles = []
    excluded_names = []
    excluded_assembly = []
    for name, body in (getattr(monitor, "static_obstacles", {}) or {}).items():
        if body in excluded:
            excluded_names.append(name)
            continue
        if bar_name_re.match(str(name)):
            # The constrained-start IK ignores future design-study bars; the
            # manual free staging target must be checked against the same set.
            excluded_assembly.append(name)
            continue
        obstacles.append(body)

    active_name = getattr(monitor, "active_bar_name", None)
    if active_bar_body is not None and active_bar_body not in obstacles:
        print(f"Manual staging ignores active bar {active_name} body={active_bar_body}.")
    if excluded_names:
        print(f"Manual staging excluded held bodies: {', '.join(excluded_names)}")
    if excluded_assembly:
        print(f"Manual staging excluded {len(excluded_assembly)} design-study assembly bodies: "
              f"{', '.join(excluded_assembly[:6])}{'...' if len(excluded_assembly) > 6 else ''}")
    print(f"Manual staging planner sees {len(obstacles)} obstacle bodies.")
    return obstacles


def _first_puid_or_none(client, name):
    ids = client.rigid_bodies_puids.get(name)
    return ids[0] if ids else None


def _state_with_seed(state, seed_conf: Configuration) -> RobotCellState:
    """A copy of ``state`` whose joints named in ``seed_conf`` take its values.

    Args:
        state (RobotCellState): The template (its robot_configuration must
            already name every joint in ``seed_conf``).
        seed_conf (Configuration): Joint values to write, matched BY NAME.

    Returns:
        RobotCellState: The seeded copy.
    """
    seeded = state.copy()
    for n, v in zip(seed_conf.joint_names, seed_conf.joint_values):
        seeded.robot_configuration[n] = float(v)
    return seeded


def solve_goal_ik_generic(planner, start_state, targets: dict, *, seed_confs=(),
                          check_collision: bool = True, verbose: bool = False,
                          skip_env_collisions: bool = False, ik_max_results: int = 20,
                          ik_max_descend_iterations: int = 200,
                          report: Optional[dict] = None) -> Optional[Configuration]:
    """Goal IK for one or more planning groups, with cfab collision checking.

    Group-generic core of the live-base IK: Cindy passes her two arm groups,
    a support robot its one ``manipulator``. The groups are solved one after
    the other, each seeded with the previous group's result, then the merged
    configuration is collision-checked ONCE.

    * Each group's IK runs with check_collision=False even when a checked
    * result is wanted. Reason: the LEFT IK's per-call collision check would run
    * against a state where the RIGHT arm is still at its seed (usually HOME) but
    * the bar -- attached to the LEFT tool0 -- has already moved to the LEFT
    * target. That intermediate state can put the bar into the still-at-seed
    * right arm, so the LEFT IK would be rejected even though the FINAL merged
    * state (both arms at their targets) is collision-free.

    Tries ``start_state`` first, then ``start_state`` seeded with each of
    ``seed_confs`` in turn, and returns the first valid solution.

    Args:
        planner: compas_fab PyBulletPlanner with the robot cell loaded.
        start_state (RobotCellState): First IK seed, and the template every
            candidate goal state is copied from (tools, bodies, base).
        targets (dict): ``{planning group: Frame}``, world-frame flange (tool0)
            targets, solved in this order.
        seed_confs (Iterable[Configuration]): Extra seeds, merged by joint
            name into a copy of ``start_state``.
        check_collision (bool): Collision-check the merged configuration.
        verbose (bool): Pass-through to compas_fab IK / collision check.
        skip_env_collisions (bool): Skip the CC.3 (link <-> rigid body), CC.4
            (attached body <-> body) and CC.5 (tool <-> body) checks, keeping only
            robot self-collision (CC.1) and robot <-> tool (CC.2).
        ik_max_results (int): compas_fab IK ``max_results``.
        ik_max_descend_iterations (int): compas_fab IK ``max_descend_iterations``.
        report (dict | None): When given, filled with ``last_error`` (str),
            ``first_colliding_state`` (the first merged state the collision check
            rejected, or None) and ``state_push_failed`` (bool), for diagnostics.

    Returns:
        Configuration | None: The full robot configuration of the first valid
        solution, or None.
    """
    # pybullet's whole-body IK can converge by recruiting a joint outside the
    # requested arm group (typically the other arm); compas_fab rejects that
    # with PlanningGroupNotSupported. Treat it as a soft IK failure (the arm
    # can't reach the target from this seed) instead of a hard crash.
    ik_fail = (InverseKinematicsError, CollisionCheckError, PlanningGroupNotSupported)
    report = {} if report is None else report
    report.update(last_error=None, first_colliding_state=None, state_push_failed=False)

    # Push state defensively so the planner's ACM/attachments match what
    # we're about to feed into IK (held bar -> gripper touch-links etc).
    try:
        planner.set_robot_cell_state(start_state)
    except Exception as e:
        print(f"[goal IK] set_robot_cell_state failed: {e}")
        report.update(last_error=f"set_robot_cell_state failed: {e}", state_push_failed=True)
        return None

    ik_targets = {
        group: FrameTarget(frame, target_mode=TargetMode.ROBOT,
                           tolerance_position=0.001, tolerance_orientation=0.01)
        for group, frame in targets.items()
    }
    ik_options = {
        "max_results": ik_max_results,
        "max_descend_iterations": ik_max_descend_iterations,
        "return_full_configuration": True,
        "check_collision": False,  # see the note above: checked once, merged
        "verbose": verbose,
    }
    cc_options = {"verbose": verbose}
    if skip_env_collisions:
        # Skip env-related collision checks; keep robot self + robot<->tool.
        for opts in (ik_options, cc_options):
            opts["_skip_cc3"] = True
            opts["_skip_cc4"] = True
            opts["_skip_cc5"] = True

    def _solve_groups(seed_state):
        # Returns (goal_state, error_message, colliding_state). The third slot
        # carries the merged state that the collision check REJECTED, so a
        # diagnostic viz has real geometry to show; None when a group failed at
        # the IK stage or everything succeeded.
        state = seed_state
        for group, target in ik_targets.items():
            try:
                conf = planner.inverse_kinematics(target, state, group, ik_options)
            except ik_fail as e:
                return None, f"{group} FAIL: {getattr(e, 'message', e)}", None
            state = seed_state.copy()
            state.robot_configuration = conf
        if check_collision:
            try:
                planner.check_collision(state, cc_options)
            except CollisionCheckError as e:
                first_line = (e.message or '').splitlines()[0] if e.message else ''
                return None, f"GOAL COLLISION: {first_line}", state
        return state, None, None

    candidates = [start_state] + [_state_with_seed(start_state, c) for c in seed_confs]
    for attempt, seed_state in enumerate(candidates, start=1):
        goal_state, err, colliding = _solve_groups(seed_state)
        if goal_state is not None:
            print(f"[goal IK] attempt {attempt}/{len(candidates)}: OK")
            return goal_state.robot_configuration
        if colliding is not None and report['first_colliding_state'] is None:
            report['first_colliding_state'] = colliding
        report['last_error'] = err
        print(f"[goal IK] attempt {attempt}/{len(candidates)}: {err}")
    return None


def _solve_bar_action_goal_ik(monitor, start_state,
                              ik_max_results: int = 20,
                              ik_max_descend_iterations: int = 200,
                              max_outer_attempts: int = 5,
                              random_seed: int = 0,
                              verbose: bool = False,
                              skip_env_collisions: bool = False,
                              alt_seed_conf12=None):
    """Solve Cindy's dual-arm goal IK for a BarAction movement from `target_ee_frames`.

    Returns a 12-vector (left_conf || right_conf) on success, or None on
    failure. Mirrors `core.robot_cell.solve_dual_arm_ik` in
    bar_joint_rhino_design_workflow: left then right, merging configs,
    using the cfab planner + state-defined ACM (held bar + tool touch-
    links). The solving itself is `solve_goal_ik_generic`; this wrapper adds
    Cindy's seed list and, on failure, a `check_collision=False` retry so we
    can tell "unreachable" from "ACM/collision rejection", plus a drawing of
    what collided.

    Args:
        monitor: The HuskyMonitor holding the cfab session, target_ee_frames,
            and -- on success only -- movement_goal_state.
        start_state: RobotCellState used as the primary IK seed and as the
            template every candidate goal state is copied from.
        ik_max_results (int): compas_fab IK ``max_results``.
        ik_max_descend_iterations (int): compas_fab IK ``max_descend_iterations``.
        max_outer_attempts (int): Seeds tried in total (start_state included).
        random_seed (int): numpy seed for the random seed perturbations.
        verbose (bool): Pass-through to compas_fab.
        skip_env_collisions: When True, the compas_fab check_collision CC.3
            (link↔rigid-body), CC.4 (attached-rigid-body↔rigid-body), and CC.5
            (tool↔rigid-body) steps are bypassed during IK. Only CC.1 (robot
            self-collision) and CC.2 (robot↔tool) remain. Use when the env scene
            is irrelevant to the local replan.
        alt_seed_conf12: Optional 12-vector (left||right) used ONLY as an extra
            IK seed and as the centre of the random seed perturbations --
            typically the movement's own start_conf, a bar-holding pose whose FK
            produces the target frames by construction. It is never accepted as
            a goal in its own right: it is authored against the movement's own
            base, so at a different live base its tool0s miss the targets by
            the base offset. When trac_ik plus the collision check cannot find
            a valid branch, this returns None.

    Returns:
        np.ndarray | None: The 12-vector goal conf, or None on failure.
    """
    if monitor.target_ee_frames is None:
        return None

    planner = monitor.cfab.planner
    targets = {
        "base_left_arm_manipulator": monitor.target_ee_frames["left"],
        "base_right_arm_manipulator": monitor.target_ee_frames["right"],
    }
    all_names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]) + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])

    # * Build a list of seed configurations to try across outer attempts.
    # Trac_ik's descent is deterministic in the neighbourhood of its seed,
    # so retrying with only different numpy state does not escape a
    # colliding local minimum. This list gives trac_ik a shot at each of:
    # the caller-supplied seed (usually the "live" pose, often HOME with
    # extended arms, tried first by solve_goal_ik_generic), the known-good
    # dual-arm home, an optional user-supplied `alt_seed_conf12` (typically
    # the movement's own start_conf -- a bar-holding pose that FK-produces the
    # target frames by construction), and small random perturbations.
    np.random.seed(random_seed)
    caller_seed_12 = np.array(
        [float(start_state.robot_configuration[n]) for n in all_names], dtype=float
    )
    seed_confs = [conf_from_12vec(HUSKY_DUAL_ARM_HOME_CONF_12)]   # 2. known-good home
    # Small random perturbations. When alt_seed_conf12 is provided we
    # perturb around IT (a bar-holding pose whose FK produces the target
    # frames by construction) so trac_ik stays in the neighbourhood of a
    # valid IK branch. Perturbing around the caller seed instead was
    # letting trac_ik jump to wildly-wrapped IK branches that unwrap
    # couldn't fully correct, leaving huge composite-plan deltas.
    perturb_center = (
        np.asarray(alt_seed_conf12, dtype=float)
        if alt_seed_conf12 is not None
        else caller_seed_12
    )
    if alt_seed_conf12 is not None:
        seed_confs.append(conf_from_12vec(alt_seed_conf12))      # 3. movement start
    while len(seed_confs) + 1 < max_outer_attempts:              # +1: start_state itself
        perturb = np.random.normal(0.0, 0.05, size=12)
        seed_confs.append(conf_from_12vec(perturb_center + perturb))

    ik_kwargs = dict(verbose=verbose, skip_env_collisions=skip_env_collisions,
                     ik_max_results=ik_max_results,
                     ik_max_descend_iterations=ik_max_descend_iterations)
    report = {}
    conf_LR = solve_goal_ik_generic(planner, start_state, targets, seed_confs=seed_confs,
                                    check_collision=True, report=report, **ik_kwargs)

    if conf_LR is None:
        if report['state_push_failed']:
            return None
        # Diagnostic: try without collision check. If THIS succeeds, the
        # target is reachable and the failure was ACM/collision rejection
        # — usually a missing touch-link on the held bar or a stale ACM.
        report_nc = {}
        conf_nc = solve_goal_ik_generic(planner, start_state, targets,
                                        check_collision=False, report=report_nc, **ik_kwargs)
        gs_nc = None
        if conf_nc is not None:
            gs_nc = start_state.copy()
            gs_nc.robot_configuration = conf_nc
            print(
                "[goal IK] DIAGNOSTIC: IK is reachable WITHOUT collision check "
                "but rejected WITH collision check. Last with-CC error: "
                f"{report['last_error']}. Likely missing touch-link on the held bar or "
                "stale ACM. Inspect monitor.cfab.planner state, the bar's "
                "rigid_body_states[...].touch_links, and the start_state "
                "passed to IK."
            )
        else:
            print(
                f"[goal IK] DIAGNOSTIC: IK ALSO fails without collision check "
                f"({report_nc['last_error']}); the EE targets are unreachable from the current "
                "base. Move the base closer to the goal-ghost base pose."
            )
        # * Diagnostic viz: always report WHERE the best candidate goal state
        # collides -- printed, and drawn in the shared PyBullet window when a GUI
        # is attached. Each run clears the previous drawing first, so this cannot
        # pile up; 'Remove all drawing' wipes it by hand. The state we show is the
        # first candidate that reached the collision check (the seed list is
        # ordered most- to least-preferred, so that one is closest to what the
        # operator asked for); when every attempt failed at the IK stage instead,
        # fall back to the un-collision-checked solution, which is then the only
        # geometry there is to look at.
        state_to_show = (report['first_colliding_state']
                         if report['first_colliding_state'] is not None else gs_nc)
        if state_to_show is not None:
            visualize_goal_ik_collision(
                monitor, state_to_show, skip_env_collisions=skip_env_collisions,
            )
        return None

    goal_state = start_state.copy()
    goal_state.robot_configuration = conf_LR
    monitor.movement_goal_state = goal_state
    return np.array([conf_LR[n] for n in all_names])


def kissing_experiment(monitor):
    hi: HuskyRobotInterface = monitor.huskies[monitor.selected_robot_id].interface
    robot = monitor.huskies[monitor.selected_robot_id].object.robot

    # store current neutral pose
    left_tool0_pose = pp.get_link_pose(monitor.goal_model.robot, pp.link_from_name(monitor.goal_model.robot, 'left_ur_arm_tool0'))
    right_tool0_pose = pp.get_link_pose(monitor.goal_model.robot, pp.link_from_name(monitor.goal_model.robot, 'right_ur_arm_tool0'))
    
    neutral_bar_pose, _, _ = compute_bar_pose_from_EE_poses(left_tool0_pose, right_tool0_pose)
    pp.draw_pose(neutral_bar_pose)
    
    monitor.get_logger().info('### MOVE TO NEUTRAL POSE')
    reset = generate_reset_trajectory_bar(monitor, 0.01, neutral_bar_pose)
    hi.send_dual_arm_cmd(reset)
    while hi.is_arm_executing[0] or hi.is_arm_executing[1]:
        yield
        
    root2 = 1.414213562
    
    for i in range(0, 3):        
        # sample
        offset = [0.000 + 0.005 * i, 0.000, 0.00, 0.00] # x y (0.005) a b (0.05) # 0.001 * i
        
        monitor.get_logger().info(f'### SAMPLED_{offset[0]:.4f}_{offset[1]:.4f}_{offset[2]:.4f}_{offset[3]:.4f}')
        
        # move to starting pose
        starting_bar_pose = pp.multiply(neutral_bar_pose, pp.Pose(pp.Point(offset[0], offset[1], 0), pp.Euler(0, 0, 0)))
        
        monitor.get_logger().info('### MOVE TO STARTING POSE')
        start_bar_movement = generate_reset_trajectory_bar(monitor, 0.01, starting_bar_pose)
        #monitor.set_arm_trajectory(start_bar_movement[0], 0)
        #monitor.set_arm_trajectory(start_bar_movement[1], 1)
        hi.send_dual_arm_cmd(start_bar_movement)
        while hi.is_arm_executing[0] or hi.is_arm_executing[1]:
            yield
        
        task = kissing_probe_once(monitor, neutral_bar_pose, starting_bar_pose, offset, DATA_FOLDER, f'dual_offset_{offset[0]:.4f}_{offset[1]:.4f}_{offset[2]:.4f}_{offset[3]:.4f}')
        yield
        while True:
            try:
                next(task)
                yield
            except StopIteration:
                break

def draw_tcp_pose(monitor):
    hi: HuskyRobotInterface = monitor.huskies[monitor.selected_robot_id].interface
    robot = monitor.huskies[monitor.selected_robot_id].object.robot
    world_from_arm_base = pp.get_link_pose(robot, pp.link_from_name(robot, 'left_ur_arm_base_link'))
    world_from_tool0 = pp.get_link_pose(robot, pp.link_from_name(robot, 'left_ur_arm_tool0'))
    arm_base_from_tool0 = pp.multiply(pp.invert(world_from_arm_base), world_from_tool0)
    pp.draw_pose(pp.multiply(world_from_arm_base, hi.arm_tcp_pose[0]))
    
    print(f"Tool0 LOCAL {arm_base_from_tool0}")
    print(f"TCP Pose LOCAL {hi.arm_tcp_pose[0]}")
    
def compute_bar_pose_from_EE_poses(left, right):
    inter = list(pp.interpolate_poses_by_num_steps(left, right, 2))
    middle_pose = inter[1]
    to_left = pp.multiply(pp.invert(middle_pose), left)
    to_right = pp.multiply(pp.invert(middle_pose), right)
    
    pp.draw_pose(middle_pose)
    print(f'MIDDLE POSE {middle_pose}')
    
    d_left = np.linalg.norm(np.array(pp.point_from_pose(to_left)))
    d_right = np.linalg.norm(np.array(pp.point_from_pose(to_right)))
    
    print(f'LEFT DISTANCE {d_left}')
    print(f'RIGHT DISTANCE {d_right}')
    
    return (middle_pose, to_left, to_right)

def execute_linear_cartesian_move(robot, hi, start_time, cartesian_trajectory, index, arm_base_link: str = None):
    """Publish one tick of a linear cartesian move as the arm's compliance target.

    Args:
        robot (int): PyBullet body of the live husky (gives the arm base pose).
        hi (HuskyRobotInterface): The husky's interface.
        start_time (float): ``time.time()`` when the move started.
        cartesian_trajectory (list): ``[start_pose_world, end_pose_world, t_move, t_wait]``.
        index (int): Arm index (0 = left or the only arm, 1 = right).
        arm_base_link (str): The arm's base link (``RobotSpec.arm_base_links[index]``).
            None keeps Cindy's ``left_/right_ur_arm_base_link`` by index.

    Returns:
        bool: False once the move (plus its wait) is over, else True.
    """
    time_elapsed = time.time() - start_time

    if time_elapsed > cartesian_trajectory[2] + cartesian_trajectory[3] + PROBE_END_WAIT_TIME:
        return False

    if arm_base_link is None:
        arm_base_link = 'left_ur_arm_base_link' if index == 0 else 'right_ur_arm_base_link'
    world_from_arm_base = pp.get_link_pose(robot, pp.link_from_name(robot, arm_base_link))
    
    start_pose_world = cartesian_trajectory[0]
    end_pose_world = cartesian_trajectory[1]
    
    offset = pp.multiply(end_pose_world,pp.invert(start_pose_world))
    
    linear_offset = pp.point_from_pose(offset)
    quat_1 = pp.quat_from_pose(start_pose_world)
    quat_2 = pp.quat_from_pose(end_pose_world)
    
    t = min(time_elapsed / cartesian_trajectory[2], 1.0)
    
    lerped = pp.Pose(np.array(pp.point_from_pose(start_pose_world)) + np.array(linear_offset) * t, pp.euler_from_quat(pp.quaternion_slerp(quat_1, quat_2, t)))
    arm_base_from_tool0 = pp.multiply(pp.invert(world_from_arm_base), lerped)
    
    #pp.draw_pose(lerped)
    hi.send_arm_cmd_cartesian(arm_base_from_tool0, index)
    
    return True

# How long to wait for every arm to acknowledge a controller switch. A healthy
# switch acks well inside a second; this is only a ceiling so a dead or
# unresponsive controller_manager cannot wedge the task queue forever.
CONTROLLER_SWITCH_TIMEOUT_S = 5.0


def switch_arm_controllers(monitor: 'HuskyMonitor', to_ctrl: str, timeout_s: float = CONTROLLER_SWITCH_TIMEOUT_S,
                           arm_indices: Optional[list] = None) -> Generator[None, None, bool]:
    """Switch the arms to ``to_ctrl``; yield until every one acknowledges.

    ! This reports failure, it does NOT fall back. Every caller must check the
    ! result and abort: continuing after a failed switch would drive the arms
    ! through the WRONG controller -- e.g. publishing cartesian compliance
    ! targets while the joint trajectory controller still holds the arms, or
    ! (worse) leaving the compliance controller live afterwards. There is no
    ! safe "carry on anyway" here, which is why nothing in this function
    ! silently proceeds.

    Gives up on any of three signals:
      1. the request could not even be sent (no service client);
      2. controller_manager explicitly rejected it (reported per arm via
         ``hi.controller_switch_error``, so a known failure aborts at once
         rather than sitting out the timeout);
      3. ``timeout_s`` elapsed with no acknowledgement -- the dead-service case,
         which used to spin here forever and block every other queued task.

    Args:
        monitor (HuskyMonitor): Provides the husky interface and the connected
            robot (how many arms it has, their side names for the log).
        to_ctrl (str): Controller to activate.
        timeout_s (float): Ceiling on the wait, in seconds.
        arm_indices (list[int] | None): Arms to switch. None = every arm of the
            connected robot (both for Cindy, arm 0 for a support robot).

    Yields:
        None: One yield per monitor tick while waiting.

    Returns:
        bool: True when every listed arm reports ``to_ctrl`` active; False on
        any of the failure signals above, each of which is logged as an error first.
    """
    hi: HuskyRobotInterface = monitor.huskies[monitor.selected_robot_id].interface
    spec = monitor._connected_robot()
    arms = list(range(spec.n_arms)) if arm_indices is None else list(arm_indices)
    sides = [spec.side_keys[i] for i in arms]
    # Already there: say so without touching controller_manager. A switch whose
    # deactivate and activate name the SAME controller is rejected under STRICT,
    # so asking anyway would turn "nothing to do" into a spurious failure.
    # Callers rely on this to cheaply ensure a controller (e.g. the joint
    # controller before M2's rigid chunk) without knowing what is active.
    if all(hi.active_controller[i] == to_ctrl for i in arms):
        return True
    sent = [hi.switch_controller(hi.active_controller[i], to_ctrl, i) for i in arms]
    if not all(sent):
        monitor.get_logger().error(
            f'Controller switch to {to_ctrl!r} could not be requested; aborting.')
        return False

    deadline = time.time() + float(timeout_s)
    while any(hi.active_controller[i] != to_ctrl for i in arms):
        errors = [hi.controller_switch_error[i] for i in arms]
        if any(errors):
            detail = ', '.join(f'{side}: {err}' for side, err in zip(sides, errors))
            monitor.get_logger().error(
                f'Controller switch to {to_ctrl!r} FAILED ({detail}); aborting.')
            return False
        if time.time() > deadline:
            active = ', '.join(f'{side}={hi.active_controller[i]!r}' for side, i in zip(sides, arms))
            monitor.get_logger().error(
                f'Controller switch to {to_ctrl!r} timed out after {timeout_s:.1f}s '
                f'(active now: {active}). Is controller_manager '
                f'running? Aborting.')
            return False
        yield
    return True


# The historical dual-arm name; Cindy's M2/M3 flows still use it.
switch_dual_arm_controller = switch_arm_controllers


def execute_cartesian_linear_dual(monitor, cartesian_trajectories,
                                  on_tick=None, should_continue=None):
    """Drive both arms along a per-arm linear cartesian segment.

    cartesian_trajectories = [[L_start_pose, L_end_pose, t_move, t_wait],
                              [R_start_pose, R_end_pose, t_move, t_wait]]
    on_tick(hi, robot): optional per-tick callback (log wrench/pose, etc).
    should_continue(): optional bool; if returns False, loop exits early.
                       Default: True (run until time budget exhausts).
    """
    hi: HuskyRobotInterface = monitor.huskies[monitor.selected_robot_id].interface
    robot = monitor.huskies[monitor.selected_robot_id].object.robot
    arm_base_links = monitor._connected_robot().arm_base_links
    start_time = time.time()

    def _step():
        l = execute_linear_cartesian_move(robot, hi, start_time,
                                          cartesian_trajectories[0], 0, arm_base_links[0])
        r = execute_linear_cartesian_move(robot, hi, start_time,
                                          cartesian_trajectories[1], 1, arm_base_links[1])
        return l or r

    cont = should_continue if should_continue is not None else (lambda: True)
    while cont() and _step():
        if on_tick is not None:
            on_tick(hi, robot)
        yield


def _scaffolding_joint_motor_stalled(hi, index):
    """v3 stall read for the scaffolding tool's JOINT motor (the screw).

    The tool has two motors, and the firmware still names them M1/M2 -- which
    reads confusingly next to the movement roles M0..M4, so everything on the
    Python side calls them by what they do instead:

        gripper motor = M1 (msg field ``state_m1``, ``motor=1``) -- holds the bar
        joint motor   = M2 (msg field ``state_m2``, ``motor=2``) -- drives the screw

    Args:
        hi (HuskyRobotInterface): Interface carrying ``scaffolding_status``.
        index (int): Arm index (0 = left, 1 = right).

    Returns:
        bool: True once the joint motor reports STALLED. False until the first
        status message arrives, so callers fall back on their time budget.
    """
    s = hi.scaffolding_status[index]
    return s is not None and s.state_m2 == 'STALLED'


def _scaffolding_gripper_motor_stalled(hi: HuskyRobotInterface, index: int) -> bool:
    """v3 stall read for the scaffolding tool's GRIPPER motor (holds the bar).

    Mirror of ``_scaffolding_joint_motor_stalled``: gripper motor = M1, msg
    field ``state_m1``.

    Args:
        hi (HuskyRobotInterface): Interface carrying ``scaffolding_status``.
        index (int): Arm index (0 = left, 1 = right).

    Returns:
        bool: True once the gripper motor reports STALLED. False until the first
        status message arrives, so callers fall back on their time budget.
    """
    s = hi.scaffolding_status[index]
    return s is not None and s.state_m1 == 'STALLED'


"""
Conducts a single kissing motion TODO dont follow local z on rotated starting pose, still follow neutral local z
"""
def kissing_probe_once(monitor, neutral_bar_pose, starting_bar_pose, offset, file_location, name):
    hi: HuskyRobotInterface = monitor.huskies[monitor.selected_robot_id].interface
    robot = monitor.huskies[monitor.selected_robot_id].object.robot

    monitor.get_logger().info('### PROBE ONCE')

    wrench_profile_left = []
    wrench_profile_right = []
    pose_left_trajectory = []
    pose_right_trajectory = []

    _, insertion_trajectories_cartesian = generate_insertion_motion_bar(
        monitor, Z_MOVE_TO_INSERT, 0.002 / TIME_PER_ROTATION,
        cartesian_speedup=CARTESIAN_SPEEDUP,
        neutral_start_pose=starting_bar_pose,
    )
    if insertion_trajectories_cartesian is None:
        return

    hi.zero_ft_sensor(0)
    hi.zero_ft_sensor(1)

    # ! No fallback: without the compliance controller the cartesian targets
    # ! below go nowhere, so abort before touching the screw motor rather than
    # ! driving it against a probe that will never move.
    if not (yield from switch_dual_arm_controller(
            monitor, 'cartesian_compliance_controller')):
        monitor.get_logger().error(
            f'Aborting kissing probe {name!r}: the arms are NOT under the '
            f'compliance controller.')
        return

    # v3 screw motor: clear any residual command, then TIGHTEN the joint
    # motor on both arms.
    hi.send_scaffolding_cmd(0, JOINT_MOTOR, 0)
    hi.send_scaffolding_cmd(0, JOINT_MOTOR, 1)
    hi.send_scaffolding_cmd(1, JOINT_MOTOR, 0)
    hi.send_scaffolding_cmd(1, JOINT_MOTOR, 1)

    start_time = time.time()

    def _log_tick(hi_, robot_):
        wrench_profile_left.append(hi_.arm_ft_sensor[0])
        wrench_profile_right.append(hi_.arm_ft_sensor[1])
        pose_left_trajectory.append(
            pp.get_link_pose(robot_, pp.link_from_name(robot_, 'left_ur_arm_tool0')))
        pose_right_trajectory.append(
            pp.get_link_pose(robot_, pp.link_from_name(robot_, 'right_ur_arm_tool0')))

    yield from execute_cartesian_linear_dual(
        monitor, insertion_trajectories_cartesian,
        on_tick=_log_tick,
        should_continue=lambda: not (
            _scaffolding_joint_motor_stalled(hi, 0) and _scaffolding_joint_motor_stalled(hi, 1)),
    )

    # STOP the joint motor once insertion completes (by stall or by time budget).
    hi.send_scaffolding_cmd(0, JOINT_MOTOR, 0)
    hi.send_scaffolding_cmd(0, JOINT_MOTOR, 1)

    motor_stalled_left = _scaffolding_joint_motor_stalled(hi, 0)
    motor_stalled_right = _scaffolding_joint_motor_stalled(hi, 1)
    # is_arm_executing isn't driven by the cartesian compliance controller; the
    # JSON fields stay for log-format stability but are not meaningful here.
    trajectory_finished_left = not hi.is_arm_executing[0]
    trajectory_finished_right = not hi.is_arm_executing[1]

    monitor.get_logger().info(
        f'### FINISHED PROBE (stalled_left={motor_stalled_left}, '
        f'stalled_right={motor_stalled_right}, '
        f'trajectory_finished_left={trajectory_finished_left}, '
        f'trajectory_finished_right={trajectory_finished_right})'
    )

    finish_time = time.time()
    while time.time() - finish_time < PROBE_END_WAIT_TIME:
        yield

    class NumpyEncoder(json.JSONEncoder):
        def default(self, obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            return super().default(obj)

    data = {
        'name': name,
        'start_time': start_time,
        'finish_time': finish_time,
        'neutral_bar_pose': neutral_bar_pose,
        'starting_bar_pose': starting_bar_pose,
        'offset': offset,
        'motor_stalled_left': motor_stalled_left,
        'motor_stalled_right': motor_stalled_right,
        'trajectory_finished_left': trajectory_finished_left,
        'trajectory_finished_right': trajectory_finished_right,
        'wrench_profile_left': wrench_profile_left,
        'wrench_profile_right': wrench_profile_right,
        'pose_left_trajectory': pose_left_trajectory,
        'pose_right_trajectory': pose_right_trajectory,
    }
    with open(file_location + '/' + name + '.json', 'w') as f:
        json.dump(data, f, indent=4, cls=NumpyEncoder)

    monitor.get_logger().info('### RETREAT')

    _, retreat_trajectories_cartesian = generate_insertion_motion_bar(
        monitor, -Z_MOVE_TO_INSERT, 0.002 / TIME_PER_ROTATION * CARTESIAN_SPEEDUP)
    if retreat_trajectories_cartesian is None:
        hi.send_scaffolding_cmd(0, JOINT_MOTOR, 0)
        hi.send_scaffolding_cmd(0, JOINT_MOTOR, 1)
        if not (yield from switch_dual_arm_controller(
                monitor, 'scaled_joint_trajectory_controller')):
            monitor.get_logger().error(
                'ARMS LEFT UNDER cartesian_compliance_controller after the '
                'kissing probe bailed out. Restore the joint controller before '
                'commanding any joint trajectory.')
        return

    # LOOSEN the joint motor during retreat.
    hi.send_scaffolding_cmd(-1, JOINT_MOTOR, 0)
    hi.send_scaffolding_cmd(-1, JOINT_MOTOR, 1)

    yield from execute_cartesian_linear_dual(
        monitor, retreat_trajectories_cartesian)

    # STOP the joint motor after retreat.
    hi.send_scaffolding_cmd(0, JOINT_MOTOR, 0)
    hi.send_scaffolding_cmd(0, JOINT_MOTOR, 1)

    current_left_tool_world_pose = pp.get_link_pose(
        robot, pp.link_from_name(robot, 'left_ur_arm_tool0'))
    current_right_tool_world_pose = pp.get_link_pose(
        robot, pp.link_from_name(robot, 'right_ur_arm_tool0'))
    while (np.linalg.norm(np.array(retreat_trajectories_cartesian[0][1][0]) - np.array(pp.point_from_pose(current_left_tool_world_pose))) > 0.02) or \
          (np.linalg.norm(np.array(retreat_trajectories_cartesian[1][1][0]) - np.array(pp.point_from_pose(current_right_tool_world_pose))) > 0.02):
        print("retreat did not work! retry!")
        print(f'LEFT: {np.array(retreat_trajectories_cartesian[0][1][0])} vs {np.array(pp.point_from_pose(current_left_tool_world_pose))}')
        print(f'RIGHT: {np.array(retreat_trajectories_cartesian[1][1][0])} vs {np.array(pp.point_from_pose(current_right_tool_world_pose))}')

        start_retry_time = time.time()
        while time.time() - start_retry_time < 5:
            yield

        current_left_tool_world_pose = pp.get_link_pose(
            robot, pp.link_from_name(robot, 'left_ur_arm_tool0'))
        current_right_tool_world_pose = pp.get_link_pose(
            robot, pp.link_from_name(robot, 'right_ur_arm_tool0'))

    if not (yield from switch_dual_arm_controller(
            monitor, 'scaled_joint_trajectory_controller')):
        monitor.get_logger().error(
            'ARMS LEFT UNDER cartesian_compliance_controller at the end of the '
            'kissing probe. Restore the joint controller before commanding any '
            'joint trajectory.')


# A rigid approach shorter than this is not worth switching controllers for.
MIN_RIGID_CHUNK_MM = 0.5


def _tool0_points_along_path(monitor, path12):
    """FK both tool0 origins at every waypoint, on the ghost robot.

    Uses ``monitor.goal_model.robot`` -- the same robot the compliant executor
    FKs its endpoint poses on -- and restores its joint positions afterwards so
    the preview is not disturbed.

    Args:
        monitor: The HuskyMonitor node.
        path12 (Sequence): Waypoints, each a 12-vec (left 6 then right 6).

    Returns:
        tuple[list, list]: ``(left_points, right_points)``, each a list of
        3-element numpy arrays in world coordinates.
    """
    ghost = monitor.goal_model.robot
    left_joints = pp.joints_from_names(ghost, HUSKY_DUAL_UR5e_JOINT_NAMES[0])
    right_joints = pp.joints_from_names(ghost, HUSKY_DUAL_UR5e_JOINT_NAMES[1])
    left_tool0 = pp.link_from_name(ghost, 'left_ur_arm_tool0')
    right_tool0 = pp.link_from_name(ghost, 'right_ur_arm_tool0')
    saved_left = pp.get_joint_positions(ghost, left_joints)
    saved_right = pp.get_joint_positions(ghost, right_joints)
    lefts, rights = [], []
    try:
        for q in path12:
            q = np.asarray(q, dtype=float)
            pp.set_joint_positions(ghost, left_joints, q[:6])
            pp.set_joint_positions(ghost, right_joints, q[6:])
            lefts.append(np.array(pp.point_from_pose(pp.get_link_pose(ghost, left_tool0))))
            rights.append(np.array(pp.point_from_pose(pp.get_link_pose(ghost, right_tool0))))
    finally:
        pp.set_joint_positions(ghost, left_joints, saved_left)
        pp.set_joint_positions(ghost, right_joints, saved_right)
    return lefts, rights


def split_path_by_distance_to_goal(monitor, path12, split_mm, from_start=False):
    """Cut a linear path where the tool is ``split_mm`` short of its final pose.

    M2 is executed in two chunks -- rigid position control for the approach, then
    compliance for the last stretch (see execute_planned_trajectory_compliant).
    This finds the handover point, measured as straight-line tool0 distance from
    the FINAL waypoint (so the operator's slider reads "how far from seated do I
    give the screw control", independent of how long the movement is).

    Distance is taken as the MAX over the two arms. They travel together on a
    bar-held linear move, but nothing here depends on that.

    ! Makes no assumption about even waypoint spacing or a straight path -- the
    ! per-waypoint distances are measured by FK and the split is interpolated
    ! between whichever two waypoints bracket it.

    Args:
        monitor: The HuskyMonitor node.
        path12 (Sequence): Planned waypoints, each a 12-vec.
        split_mm (float): Where to cut, in millimetres. Measured from the GOAL
            by default (M2: "the last N mm are compliant"); from the START when
            ``from_start`` is set (M3: "the first N mm are compliant").
        from_start (bool): Interpret ``split_mm`` from the start of the path.

    Returns:
        tuple: ``(leading_path12, trailing_path12, at_goal_mm, total_mm)``.
        The two paths share the interpolated handover configuration so they join
        without a gap. ``leading_path12`` is ``None`` when the cut leaves no
        meaningful leading chunk -- the caller then runs a single mode the whole
        way. ``at_goal_mm`` is the achieved distance-to-goal at the handover.
    """
    path12 = [np.asarray(q, dtype=float) for q in (path12 or [])]
    if len(path12) < 2:
        return None, 0.0, 0.0

    lefts, rights = _tool0_points_along_path(monitor, path12)
    # Distance from each waypoint to the FINAL one, per arm, worst case.
    dists_mm = [
        max(float(np.linalg.norm(lefts[-1] - lefts[i])),
            float(np.linalg.norm(rights[-1] - rights[i]))) * 1000.0
        for i in range(len(path12))
    ]
    total_mm = dists_mm[0]

    # `split_mm` is quoted by the caller either as a distance from the goal (M2:
    # "the last 10 mm are compliant") or from the start (M3: "the first 10 mm
    # are compliant"). Convert the latter so everything below works in one
    # convention -- distance-to-goal at the handover.
    at_goal_mm = (total_mm - split_mm) if from_start else split_mm

    # Leave the leading chunk out entirely unless it is worth running. The
    # margin also absorbs FK round-off: a slider set to exactly the movement's
    # travel would otherwise produce a degenerate sub-micron leading chunk
    # instead of the single-mode behaviour the operator asked for.
    if at_goal_mm >= total_mm - MIN_RIGID_CHUNK_MM:
        print(f"[split] requested {split_mm:.1f} mm leaves no meaningful leading "
              f"chunk within the movement's {total_mm:.1f} mm travel; "
              f"running one mode the whole way.")
        return None, None, total_mm, total_mm
    if at_goal_mm <= 0.0:
        return list(path12), None, 0.0, total_mm

    # First waypoint (from the start) that is already CLOSER than at_goal_mm;
    # the handover lies between it and the one before.
    idx = next((i for i, d in enumerate(dists_mm) if d <= at_goal_mm), len(path12) - 1)
    if idx == 0:
        return None, None, total_mm, total_mm
    d_before, d_after = dists_mm[idx - 1], dists_mm[idx]
    span = d_before - d_after
    # Fraction along [idx-1, idx] where the distance-to-goal equals at_goal_mm.
    frac = 0.0 if span <= 1e-9 else (d_before - at_goal_mm) / span
    frac = min(max(frac, 0.0), 1.0)
    split_conf = path12[idx - 1] + (path12[idx] - path12[idx - 1]) * frac

    leading = [q.copy() for q in path12[:idx]] + [split_conf]
    # The trailing chunk restarts AT the handover so the two join without a gap.
    trailing = [split_conf.copy()] + [q.copy() for q in path12[idx:]]
    # Report what we actually achieved, not what was asked for.
    split_l, split_r = _tool0_points_along_path(monitor, [split_conf])
    actual_mm = max(float(np.linalg.norm(lefts[-1] - split_l[0])),
                    float(np.linalg.norm(rights[-1] - split_r[0]))) * 1000.0
    return leading, trailing, actual_mm, total_mm


# Default handover distance for M2, in millimetres from the assembled pose.
# The monitor owns the live value (slider); this is the fallback.
M2_COMPLIANT_SPLIT_MM_DEFAULT = 10.0


def _live_tool0_poses(monitor):
    """Every arm's tool0 world pose at the LIVE arm configuration.

    FK on the ghost robot at ``hi.arm_joint_pose``, restoring its joints
    afterwards. Used at the rigid->compliant handover so the compliant segment
    starts where the arm actually is. Joint and flange names come from the
    connected robot.

    Returns:
        tuple: One pybullet (point, quat) pose per arm, in side order:
        ``(left_pose, right_pose)`` for Cindy, ``(arm_pose,)`` for a support robot.
    """
    hi: HuskyRobotInterface = monitor.huskies[monitor.selected_robot_id].interface
    spec = monitor._connected_robot()
    ghost = monitor.goal_model.robot
    arm_joints = [pp.joints_from_names(ghost, names) for names in spec.arm_joint_names]
    saved = [pp.get_joint_positions(ghost, joints) for joints in arm_joints]
    try:
        for i, joints in enumerate(arm_joints):
            pp.set_joint_positions(ghost, joints, np.asarray(hi.arm_joint_pose[i], dtype=float))
        return tuple(pp.get_link_pose(ghost, pp.link_from_name(ghost, link))
                     for link in spec.flange_links)
    finally:
        for joints, q in zip(arm_joints, saved):
            pp.set_joint_positions(ghost, joints, q)


def _execute_rigid_chunk(monitor, rigid_path12, t_rigid, on_tick=None,
                         stall_exits=True, label: str = 'rigid'):
    """Run M2's approach under the joint controller, ending early on stall.

    Sends ``rigid_path12`` as one dual-arm joint trajectory and waits it out.

    ! Waits by TIME, not by ``hi.is_arm_executing``: that flag clears after only
    ! ARM_NOT_EXECUTING_TIME (1 s) of no joint change, so a controller slow to
    ! start trips it and we would hand over to compliance mid-approach.

    If BOTH joint motors report STALLED the wait returns immediately: the thread
    bit earlier than the nominal split, and the caller's switch to compliance
    then deactivates the joint controller, abandoning the rest of the
    trajectory. That controller switch IS the cut-short mechanism -- the topic
    trajectory interface has no cancel.

    Args:
        monitor: The HuskyMonitor node.
        rigid_path12 (Sequence): Waypoints for the chunk, each a 12-vec.
        t_rigid (float): Duration for the chunk, seconds.
        on_tick (callable): Optional ``fn(hi, robot)`` called every tick, so the
            wrench recording spans this chunk too.
        stall_exits (bool): End early when both joint motors report STALLED.
            True for M2's approach (the thread bit early). M3 must pass False:
            its joint motor is not driving, so a STALLED flag left over from the
            preceding M2 would abort the retreat on its very first tick.
        label (str): Log tag (the movement id).

    Yields:
        None: One yield per monitor tick.

    Returns:
        bool: True if the chunk ran (or was legitimately skipped); False if the
        joint controller could not be secured, in which case nothing was sent.
    """
    hi: HuskyRobotInterface = monitor.huskies[monitor.selected_robot_id].interface
    robot = monitor.huskies[monitor.selected_robot_id].object.robot

    if len(rigid_path12) < 2:
        # to_trajectory_msg divides by (len - 1); a single waypoint means the
        # arms are already at the handover, so there is simply nothing to run.
        print(f'[{label}] rigid chunk is a single waypoint; nothing to run.')
        return True

    # The arms are normally already under the joint controller (M1 was a joint
    # move, and every compliant exec restores it), so this is usually a no-op --
    # switch_dual_arm_controller returns True without touching controller_manager.
    if not (yield from switch_dual_arm_controller(
            monitor, 'scaled_joint_trajectory_controller')):
        monitor.get_logger().error(
            f'[{label}] cannot run the rigid chunk: the arms are NOT under '
            'scaled_joint_trajectory_controller. Aborting before any motion.')
        return False

    chunk = [
        (np.asarray([q[:6] for q in rigid_path12], dtype=float), None, t_rigid, None),
        (np.asarray([q[6:] for q in rigid_path12], dtype=float), None, t_rigid, None),
    ]
    print(f'[{label}] rigid chunk: {len(rigid_path12)} waypoints over {t_rigid:.1f}s')
    hi.send_dual_arm_cmd(chunk)

    tick_dt = 0.05  # matches the 20 Hz monitor tick that pumps this task
    deadline = time.time() + t_rigid + tick_dt
    while time.time() < deadline:
        if on_tick is not None:
            on_tick(hi, robot)
        if stall_exits and (_scaffolding_joint_motor_stalled(hi, 0)
                            and _scaffolding_joint_motor_stalled(hi, 1)):
            monitor.get_logger().info(
                f'[{label}] both joint motors STALLED during the rigid approach -- the '
                'thread bit early; handing over to compliance now.')
            return True
        yield
    return True


# How long a non-M2 linear move (i.e. M3) keeps republishing its END pose after
# the nominal motion time, so both arms can actually converge under compliance.
#
# ! Without this the loop stopped PROBE_END_WAIT_TIME (1 s) after the nominal
# ! time and the finally immediately restored the joint controller, which HOLDS
# ! wherever it activates -- so whichever arm was still lagging got frozen short.
# ! That is how M3 ended with one arm visibly less retracted than the other:
# ! nothing here ever checked arrival, it only ran a clock.
# ! Currently 0: hardware showed the end-of-compliant-chunk drift is gravity SAG
# ! (the tools move sideways OFF the retreat line -- travelled 6.3 mm yet 9.1 mm
# ! from the goal), not a failure to converge along it. Holding the end pose
# ! longer does not fix that, so the settle buys nothing; the drift is handled
# ! by replanning the following chunk from where the arms actually are. Raise
# ! this again if a move ever needs time to settle ALONG its own path.
LINEAR_SETTLE_SECONDS = 0.0

# Both arms within this of their commanded end pose counts as arrived, which
# ends the settle early instead of always burning the full window.
LINEAR_ARRIVAL_TOL_M = 0.002

# Ceiling on any wait for the scaffolding tool's motors to report STALLED (M2's
# screw tightening, the schedule's grasp step), for firmware that never reports it.
HOLD_FOR_STALL_TIMEOUT_S = 30.0


def wait_for_operator_confirm(monitor, prompt, warn=False):
    """Hold here, yielding, until the operator confirms or cancels.

    Yielding rather than blocking is what keeps the monitor tick alive, so the
    traj-viz scrub and the DPG plots stay interactive while the operator
    inspects whatever is about to run. Clears any stale confirmation first so a
    leftover click cannot wave through the next pause.

    Buttons: 'Confirm Exec' sets ``_servo_exec_confirmed``; 'Cancel Exec' sets
    ``_servo_abort``.

    Args:
        monitor: The HuskyMonitor node.
        prompt (str): What the operator should look at before confirming.
        warn (bool): Log at warn level (a safeguard pause) rather than info.

    Yields:
        None: One yield per monitor tick while waiting.

    Returns:
        bool: True if confirmed, False if cancelled.
    """
    monitor._servo_exec_confirmed = False
    # ! One line per severity, on purpose. rclpy records which severity a given
    # ! logging LINE first used and raises "Logger severity cannot be changed
    # ! between calls." if that same line later logs at another level. Choosing
    # ! the method inline put both levels on one line, so the first safeguard
    # ! pause after an ordinary one killed the monitor mid servo loop.
    if warn:
        monitor.get_logger().warn(prompt)
    else:
        monitor.get_logger().info(prompt)
    while not getattr(monitor, '_servo_exec_confirmed', False):
        if getattr(monitor, '_servo_abort', False):
            monitor.get_logger().warn('Cancelled by operator; nothing sent.')
            return False
        yield
    return True


def _tool0_poses_at_conf(monitor, conf12):
    """Both tool0 world poses with the ghost robot at ``conf12``.

    Args:
        monitor: The HuskyMonitor node.
        conf12 (Sequence[float]): 12-vec, left arm then right.

    Returns:
        tuple: ``(left_pose, right_pose)`` as pybullet (point, quat) pairs.
    """
    ghost = monitor.goal_model.robot
    left_joints = pp.joints_from_names(ghost, HUSKY_DUAL_UR5e_JOINT_NAMES[0])
    right_joints = pp.joints_from_names(ghost, HUSKY_DUAL_UR5e_JOINT_NAMES[1])
    saved_left = pp.get_joint_positions(ghost, left_joints)
    saved_right = pp.get_joint_positions(ghost, right_joints)
    try:
        q = np.asarray(conf12, dtype=float)
        pp.set_joint_positions(ghost, left_joints, q[:6])
        pp.set_joint_positions(ghost, right_joints, q[6:])
        return (pp.get_link_pose(ghost, pp.link_from_name(ghost, 'left_ur_arm_tool0')),
                pp.get_link_pose(ghost, pp.link_from_name(ghost, 'right_ur_arm_tool0')))
    finally:
        pp.set_joint_positions(ghost, left_joints, saved_left)
        pp.set_joint_positions(ghost, right_joints, saved_right)


def _tool0_pose_errors(monitor, cartesian_trajectories):
    """Per-arm progress along the commanded linear segment, right now.

    Args:
        monitor: The HuskyMonitor node.
        cartesian_trajectories: The ``[[start, end, t_move, t_wait], ...]`` pair
            handed to ``execute_cartesian_linear_dual``.

    Returns:
        list[dict]: One entry per arm with ``travel_mm`` (the commanded
        distance), ``achieved_mm`` (how far it actually got from the start) and
        ``remaining_mm`` (distance still to the commanded end pose).
    """
    now_poses = _live_tool0_poses(monitor)
    out = []
    for now, traj in zip(now_poses, cartesian_trajectories):
        start = np.asarray(pp.point_from_pose(traj[0]), dtype=float)
        end = np.asarray(pp.point_from_pose(traj[1]), dtype=float)
        here = np.asarray(pp.point_from_pose(now), dtype=float)
        out.append({
            'travel_mm': float(np.linalg.norm(end - start)) * 1000.0,
            'achieved_mm': float(np.linalg.norm(here - start)) * 1000.0,
            'remaining_mm': float(np.linalg.norm(end - here)) * 1000.0,
        })
    return out


def _both_arms_arrived(monitor, cartesian_trajectories, tol_m=LINEAR_ARRIVAL_TOL_M):
    """True when BOTH tool0s are within ``tol_m`` of their commanded end pose."""
    try:
        return all(e['remaining_mm'] <= tol_m * 1000.0
                   for e in _tool0_pose_errors(monitor, cartesian_trajectories))
    except Exception:
        # Never let an FK hiccup end the motion early -- keep driving.
        return False


def _hold_until_joint_motors_stall(monitor, timeout_s, on_tick=None, label: str = 'insert'):
    """Tick in place until both joint motors stall, or the ceiling expires.

    Used by M2's rigid-only mode: after the joint trajectory finishes, the
    controller holds its last waypoint on its own, so all this has to do is keep
    the task alive (and the wrench recording running) while the screw tightens.

    Args:
        monitor: The HuskyMonitor node.
        timeout_s (float): Hard ceiling, for firmware that never reports stall.
        on_tick (callable): Optional ``fn(hi, robot)`` called every tick.
        label (str): Log tag (the movement id).

    Yields:
        None: One yield per monitor tick.
    """
    hi: HuskyRobotInterface = monitor.huskies[monitor.selected_robot_id].interface
    robot = monitor.huskies[monitor.selected_robot_id].object.robot
    deadline = time.time() + float(timeout_s)
    while time.time() < deadline:
        if on_tick is not None:
            on_tick(hi, robot)
        if (_scaffolding_joint_motor_stalled(hi, 0)
                and _scaffolding_joint_motor_stalled(hi, 1)):
            monitor.get_logger().info(
                f'[{label}] both joint motors STALLED; rigid-only hold complete.')
            return
        yield
    monitor.get_logger().warn(
        f'[{label}] rigid-only hold hit its {timeout_s:.0f}s ceiling without both '
        f'joint motors reporting STALLED.')


def execute_planned_trajectory_compliant(monitor):
    """Execute the loaded insert (DUAL_CONSTRAINED_LINEAR) or retreat
    (DUAL_INDEPENDENT_LINEAR) movement as linear cartesian motion.

    M3 (retreat) runs entirely under ``cartesian_compliance_controller``, as one
    linear segment between the planned path's endpoints.

    M2 (mate) is SPLIT in two, because running it wholly compliant does not
    work: the instant compliance takes over, the arms sag under gravity and pull
    the bar off the target far enough that the screw cannot catch its first
    thread. So:

      1. RIGID chunk -- the approach, under ``scaled_joint_trajectory_controller``,
         following the planned joint waypoints. Position control holds the bar
         exactly on the planned line, and the joint motor (already tightening) can
         catch its first thread against a bar that is where it should be.
      2. COMPLIANT chunk -- the last ``monitor.m2_compliant_split_mm`` millimetres,
         where the screw needs the arms free to pull the bar in.

    The handover point is the operator's slider, measured as tool0 distance from
    the final assembled pose. It moves to compliance early if both joint motors
    stall during the rigid chunk (the thread bit sooner than expected) -- driving
    position control against an engaged screw is exactly what this split avoids.

    Safety contract: the live arms must already be at the planned start conf;
    otherwise `send_arm_cmd_cartesian` rejects targets (>5 cm from current TCP)
    and the motion will not run.
    """
    if monitor.current_movement is None:
        monitor.get_logger().warn(
            "No movement loaded; click 'Load Movement' first.")
        return
    mv = monitor.current_movement
    kind = monitor._kind_of(mv)
    tag = mv.movement_id
    if kind not in COMPLIANT_KINDS:
        monitor.get_logger().warn(
            f"Compliant exec only runs the insert or the retreat; {tag} is a "
            f"{getattr(kind, 'value', type(mv).__name__)}.")
        return
    # * The insert tightens the joint screws; the retreat loosens the grippers.
    is_insert = kind is MovementKind.DUAL_CONSTRAINED_LINEAR
    is_retreat = kind is MovementKind.DUAL_INDEPENDENT_LINEAR
    if monitor.planned_arm_trajectory[0][0] is None or \
       monitor.planned_arm_trajectory[1][0] is None:
        monitor.get_logger().warn(
            "planned_arm_trajectory missing; plan or load a trajectory first.")
        return

    hi: HuskyRobotInterface = monitor.huskies[monitor.selected_robot_id].interface
    ghost_robot = monitor.goal_model.robot
    left_joints = pp.joints_from_names(ghost_robot, HUSKY_DUAL_UR5e_JOINT_NAMES[0])
    right_joints = pp.joints_from_names(ghost_robot, HUSKY_DUAL_UR5e_JOINT_NAMES[1])
    saved_left = pp.get_joint_positions(ghost_robot, left_joints)
    saved_right = pp.get_joint_positions(ghost_robot, right_joints)
    left_tool0 = pp.link_from_name(ghost_robot, 'left_ur_arm_tool0')
    right_tool0 = pp.link_from_name(ghost_robot, 'right_ur_arm_tool0')

    left_path = monitor.planned_arm_trajectory[0][0]
    right_path = monitor.planned_arm_trajectory[1][0]

    try:
        pp.set_joint_positions(ghost_robot, left_joints, left_path[0])
        pp.set_joint_positions(ghost_robot, right_joints, right_path[0])
        L_start = pp.get_link_pose(ghost_robot, left_tool0)
        R_start = pp.get_link_pose(ghost_robot, right_tool0)

        pp.set_joint_positions(ghost_robot, left_joints, left_path[-1])
        pp.set_joint_positions(ghost_robot, right_joints, right_path[-1])
        L_end = pp.get_link_pose(ghost_robot, left_tool0)
        R_end = pp.get_link_pose(ghost_robot, right_tool0)
    finally:
        pp.set_joint_positions(ghost_robot, left_joints, saved_left)
        pp.set_joint_positions(ghost_robot, right_joints, saved_right)

    t_total = float(monitor.trajectory_time) or 5.0

    # M2 only: work out where to hand over from rigid tracking to compliance.
    # rigid_path is None for M3, and for an M2 whose split distance covers the
    # whole movement -- both of which then run exactly as they always did.
    rigid_path, split_mm, total_mm = None, 0.0, 0.0
    t_rigid = 0.0
    # Rigid-only mode (operator toggle): run ALL of M2 under the joint
    # controller and never engage compliance, holding the final pose while the
    # screw tightens. The comparison baseline for the split, and the fallback if
    # compliance misbehaves. M3 ignores this -- it is compliant by nature.
    rigid_only = bool(is_insert
                      and getattr(monitor, 'm2_exec_rigid_only', False))
    if rigid_only:
        rigid_path = [np.concatenate([np.asarray(l, dtype=float),
                                      np.asarray(r, dtype=float)])
                      for l, r in zip(left_path, right_path)]
        t_rigid = t_total
        monitor.get_logger().info(
            f"[{tag}] RIGID-ONLY mode: the whole movement runs under "
            f"scaled_joint_trajectory_controller ({len(rigid_path)} waypoints, "
            f"{t_rigid:.1f}s), then holds position until both joint motors "
            f"stall. Compliance is never engaged.")
    elif is_insert:
        requested_mm = float(getattr(monitor, 'm2_compliant_split_mm',
                                     M2_COMPLIANT_SPLIT_MM_DEFAULT))
        path12 = [np.concatenate([np.asarray(l, dtype=float),
                                  np.asarray(r, dtype=float)])
                  for l, r in zip(left_path, right_path)]
        rigid_path, _m2_tail, split_mm, total_mm = split_path_by_distance_to_goal(
            monitor, path12, requested_mm)
        if rigid_path is not None:
            rigid_mm = total_mm - split_mm
            # Same tool speed in both chunks: share the budget by distance.
            t_rigid = t_total * (rigid_mm / total_mm) if total_mm > 0 else 0.0
            t_total = max(t_total - t_rigid, 0.5)
            monitor.get_logger().info(
                f"[{tag}] split at {split_mm:.1f} mm from the assembled pose: "
                f"{rigid_mm:.1f} mm RIGID ({len(rigid_path)} waypoints, "
                f"{t_rigid:.1f}s) then {split_mm:.1f} mm COMPLIANT ({t_total:.1f}s).")
            if split_mm <= 0.0:
                monitor.get_logger().warn(
                    f"[{tag}] split distance is 0: the whole move runs rigid and the "
                    "screw never gets compliant control. Raise the slider unless "
                    "this is deliberate.")

    # M3 is the SAME split MIRRORED. The compliant zone is the stretch nearest
    # the ASSEMBLED pose in both movements -- for M2 that is the end of the
    # motion, for M3 (the retreat) it is the beginning. So M3 runs compliant
    # first, while the bar is still engaged and the tool needs to be free to
    # break out, then hands to rigid position control for the clear run back.
    # Same slider, same number of millimetres.
    m3_rigid_tail = None
    t_m3_rigid = 0.0
    if is_retreat:
        requested_mm = float(getattr(monitor, 'm2_compliant_split_mm',
                                     M2_COMPLIANT_SPLIT_MM_DEFAULT))
        path12 = [np.concatenate([np.asarray(l, dtype=float),
                                  np.asarray(r, dtype=float)])
                  for l, r in zip(left_path, right_path)]
        compliant_path, trailing, at_goal_mm, total_mm = split_path_by_distance_to_goal(
            monitor, path12, requested_mm, from_start=True)
        if compliant_path is not None and trailing is not None:
            compliant_mm = total_mm - at_goal_mm
            # The compliant chunk now ends at the handover, not at the retreated
            # pose, so re-aim the cartesian segment there.
            L_end, R_end = _tool0_poses_at_conf(monitor, compliant_path[-1])
            t_compliant = t_total * (compliant_mm / total_mm) if total_mm > 0 else t_total
            t_m3_rigid = max(t_total - t_compliant, 0.5)
            t_total = max(t_compliant, 0.5)
            m3_rigid_tail = trailing
            monitor.get_logger().info(
                f"[{tag}] split at {compliant_mm:.1f} mm from the assembled pose: "
                f"{compliant_mm:.1f} mm COMPLIANT ({t_total:.1f}s) then "
                f"{at_goal_mm:.1f} mm RIGID ({len(trailing)} waypoints, "
                f"{t_m3_rigid:.1f}s).")

    # The M2 movement holds the end pose under compliance while the JOINT motor
    # tightens the screw against the scaffold; the loop must NOT terminate on
    # motion-time budget alone. Inflate t_wait so execute_linear_cartesian_move
    # keeps publishing the end pose, and exit only when both arms' joint motors
    # report STALLED. Large t_wait (HOLD_FOR_STALL_TIMEOUT_S) is a hard ceiling
    # fallback if firmware never reports stall.
    if is_insert:
        t_wait = HOLD_FOR_STALL_TIMEOUT_S
        should_continue_fn = lambda: not (
            _scaffolding_joint_motor_stalled(hi, 0) and _scaffolding_joint_motor_stalled(hi, 1))
    else:
        # M3: keep republishing the end pose for a settle window so a lagging
        # arm can catch up, and end it as soon as BOTH arms have arrived rather
        # than always burning the whole window. Mirrors M2's structure, with
        # "arrived" in place of "stalled" as the exit condition.
        t_wait = LINEAR_SETTLE_SECONDS
        # ! Anchor the nominal-motion deadline on the FIRST tick, not here: the
        # ! controller switch runs in between and can take a second or more, so
        # ! a deadline set now would open the settle gate mid-motion.
        _motion = {}

        def should_continue_fn():
            if 'deadline' not in _motion:
                _motion['deadline'] = time.time() + t_total
                return True
            if time.time() < _motion['deadline']:
                return True    # still driving the nominal segment
            # Settling: stop as soon as both arms are on their end pose.
            return not _both_arms_arrived(monitor, cartesian_trajectories)

    cartesian_trajectories = [
        [L_start, L_end, t_total, t_wait],
        [R_start, R_end, t_total, t_wait],
    ]

    def _stop_all_both_arms():
        # mirror of the 'L/R Stop All' buttons: STOP both tool motors on both arms.
        print('[scaffolding] L Stop All: gripper + joint motor (arm 0)')
        hi.send_scaffolding_cmd(0, GRIPPER_MOTOR, 0)
        hi.send_scaffolding_cmd(0, JOINT_MOTOR, 0)
        print('[scaffolding] R Stop All: gripper + joint motor (arm 1)')
        hi.send_scaffolding_cmd(0, GRIPPER_MOTOR, 1)
        hi.send_scaffolding_cmd(0, JOINT_MOTOR, 1)

    # ! important: DO NOT zero when the robot is holding the bar
    # the only good time to zero is when the robot is holding nothing but the tool
    # ! Consequence for the force plot below: the curves carry the tool + bar
    # ! weight as a standing offset, so read them for CHANGE (the step when the
    # ! bar seats, the ramp as the screw bites), not as absolute contact force.

    # Live tool0 wrench, one sample per monitor tick, streamed to the
    # "Compliant Exec Force" DPG window. This is the operator's only continuous
    # feedback during the move -- especially for M2, which can hold at the
    # assembled pose for up to HOLD_FOR_STALL_TIMEOUT_S waiting on the stall flag.
    wrench_profile_left = []
    wrench_profile_right = []
    monitor.reset_compliant_wrench(label=monitor.current_movement.movement_id)
    exec_start_time = time.time()

    def _record_wrench(hi_, robot_):
        left = list(hi_.arm_ft_sensor[0])
        right = list(hi_.arm_ft_sensor[1])
        wrench_profile_left.append(left)
        wrench_profile_right.append(right)
        monitor.push_compliant_wrench(time.time() - exec_start_time, left, right)

    # Pre-exec scaffolding tool commands, per movement role:
    #   M2 (mate)    -> Stop All + TIGHTEN the joint motor on both arms
    #                   (screws the bar down against the scaffold).
    #   M3 (retreat) -> Stop All + LOOSEN the gripper motor on both arms
    #                   (releases the bar before backing away).
    _stop_all_both_arms()
    if is_insert:
        print(f'[scaffolding] {tag}: TIGHTEN joint motor on L arm (arm 0)')
        hi.send_scaffolding_cmd(1, JOINT_MOTOR, 0)
        print(f'[scaffolding] {tag}: TIGHTEN joint motor on R arm (arm 1)')
        hi.send_scaffolding_cmd(1, JOINT_MOTOR, 1)
    elif is_retreat:
        print(f'[scaffolding] {tag}: LOOSEN gripper motor on L arm (arm 0)')
        hi.send_scaffolding_cmd(-1, GRIPPER_MOTOR, 0)
        print(f'[scaffolding] {tag}: LOOSEN gripper motor on R arm (arm 1)')
        hi.send_scaffolding_cmd(-1, GRIPPER_MOTOR, 1)

    switched_to_compliance = False
    try:
        # --- Chunk 1: the rigid approach (M2 only) ---
        if rigid_path is not None:
            ran = yield from _execute_rigid_chunk(
                monitor, rigid_path, t_rigid, on_tick=_record_wrench, label=tag)
            if not ran:
                return
            if rigid_only:
                # No chunk 2. The joint controller holds the final waypoint, so
                # just keep ticking (recording wrench) until the screw stalls or
                # the ceiling expires; the finally below stops the motors.
                yield from _hold_until_joint_motors_stall(
                    monitor, HOLD_FOR_STALL_TIMEOUT_S, on_tick=_record_wrench,
                    label=tag)
                return
            # The arms are now at (or, on an early stall, short of) the split
            # conf. Re-derive the compliant chunk's start from where they
            # ACTUALLY are rather than from the planned split -- correct for
            # both the early-stall cut-short and ordinary tracking error, and it
            # keeps execute_linear_cartesian_move's lerp starting at the arm.
            L_start, R_start = _live_tool0_poses(monitor)
            cartesian_trajectories[0][0] = L_start
            cartesian_trajectories[1][0] = R_start

        # --- Chunk 2: the compliant phase (all of M3; M2's final split_mm) ---
        # wait until the switch is completely (yield will go back to top level monitor to get updated state)
        switched_to_compliance = yield from switch_dual_arm_controller(
            monitor,
            'cartesian_compliance_controller',
        )
        # ! No fallback: the cartesian targets below are only meaningful under
        # ! the compliance controller. If the switch failed we skip the motion
        # ! entirely rather than publishing target frames nothing is listening
        # ! to -- which would look like a silent no-op on the real robot.
        if not switched_to_compliance:
            monitor.get_logger().error(
                f'Aborting compliant exec of '
                f'{monitor.current_movement.movement_id} ({kind.value}): the arms '
                f'are NOT under the compliance controller, so no motion was '
                f'commanded. Fix the controller switch and re-run this movement.')
            return

        yield from execute_cartesian_linear_dual(
            monitor, cartesian_trajectories, on_tick=_record_wrench,
            should_continue=should_continue_fn)

        # Measure where compliance actually left each arm, BEFORE the finally
        # restores the joint controller (which freezes whatever it finds). The
        # motion is open-loop -- nothing else checks arrival -- so without this
        # a short-retracted arm is only visible by eye in the viewer.
        try:
            errs = _tool0_pose_errors(monitor, cartesian_trajectories)
            for side, e in zip(('L', 'R'), errs):
                pct = (100.0 * e['achieved_mm'] / e['travel_mm']
                       if e['travel_mm'] > 1e-9 else 100.0)
                print(f"[{tag}] {side} tool0: travelled {e['achieved_mm']:.1f} / "
                      f"{e['travel_mm']:.1f} mm ({pct:.0f}%), "
                      f"{e['remaining_mm']:.1f} mm short of the commanded end.")
            worst = max(e['remaining_mm'] for e in errs)
            spread = abs(errs[0]['achieved_mm'] - errs[1]['achieved_mm'])
            if worst > LINEAR_ARRIVAL_TOL_M * 1000.0:
                monitor.get_logger().warn(
                    f"[{tag}] arms did NOT reach the commanded end pose: worst "
                    f"{worst:.1f} mm short, L/R differ by {spread:.1f} mm. The "
                    f"joint controller is about to freeze them there.")
        except Exception as exc:
            print(f"[{tag}] could not measure the final tool0 error: {exc}")

        # --- Chunk 2 for M3: the rigid run back, once clear of the joint ---
        if m3_rigid_tail is not None:
            # Back under position control BEFORE replanning and previewing, so
            # what the operator inspects is planned for, and will run under, the
            # controller that is actually active.
            if not (yield from switch_dual_arm_controller(
                    monitor, 'scaled_joint_trajectory_controller')):
                monitor.get_logger().error(
                    f'[{tag}] cannot start the retreat: the arms are NOT under '
                    'scaled_joint_trajectory_controller. Nothing was commanded.')
                return
            switched_to_compliance = False   # the finally has nothing to restore

            # Replan a straight line from where compliance ACTUALLY left the
            # tools to the movement's authored goal frames.
            tail = monitor.replan_linear_to_target_from_live(
                monitor.current_movement, tag)
            if tail is not None:
                print(f'[{tag}] replanned retreat: {len(tail)} waypoints from the '
                      f'live pose to the authored target frames.')
            else:
                # ! Falling back to the preplanned tail means the FIRST step is a
                # ! jump from the drifted pose onto the planned line -- the fast
                # ! motion this replan exists to remove. Say so plainly; the
                # ! confirm pause below is what stops it happening unnoticed.
                live12 = np.concatenate([
                    np.asarray(hi.arm_joint_pose[0], dtype=float),
                    np.asarray(hi.arm_joint_pose[1], dtype=float)])
                tail = [live12] + [np.asarray(q, dtype=float) for q in m3_rigid_tail[1:]]
                jump_deg = float(np.degrees(np.abs(
                    np.asarray(tail[1]) - live12).max())) if len(tail) > 1 else 0.0
                monitor.get_logger().error(
                    f'[{tag}] replan failed -- FALLING BACK to the preplanned tail. '
                    f'Its first step jumps {jump_deg:.1f} deg from where the arms '
                    f'are now, which is the fast snap the replan was meant to '
                    f'avoid. Inspect the preview carefully before confirming.')

            # Show exactly what will run: the traj-viz scrub and the joint plot.
            # ! Deliberately NOT via _accept_trajectory -- that does chain
            # ! propagation and would overwrite mv.trajectory with this
            # ! transient, execution-time replan.
            monitor.set_arm_trajectory(
                (np.asarray([q[:6] for q in tail], dtype=float), None, t_m3_rigid, None), 0)
            monitor.set_arm_trajectory(
                (np.asarray([q[6:] for q in tail], dtype=float), None, t_m3_rigid, None), 1)
            monitor.show_planned_joint_values(
                tail, label=f'{monitor.current_movement.movement_id} (retreat)')
            monitor.set_to_show_traj_state()

            if not (yield from wait_for_operator_confirm(
                    monitor,
                    f'[{tag}] retreat ready: {len(tail)} waypoints over '
                    f'{t_m3_rigid:.1f}s. Scrub "Traj viz time" and check the '
                    f'joint plot, then click "Confirm Exec" to run it.')):
                return

            # ! stall_exits=False: M3's joint motor is not driving, so a STALLED
            # ! flag left over from the preceding M2 would abort this on tick 1.
            yield from _execute_rigid_chunk(
                monitor, tail, t_m3_rigid, on_tick=_record_wrench,
                stall_exits=False, label=tag)
    finally:
        # Always stop motors first, then restore the joint controller.
        _stop_all_both_arms()

        # Only worth restoring if we actually left the joint controller. If this
        # one fails there is nothing left to abort -- the move is over -- but the
        # arms are stranded under compliance, so say so loudly: the next joint
        # trajectory (any M0/M1/M4, or 'Move Arms to Movement Start') would be
        # published to a controller that is not running.
        if switched_to_compliance:
            restored = yield from switch_dual_arm_controller(
                monitor,
                'scaled_joint_trajectory_controller',
            )
            if not restored:
                monitor.get_logger().error(
                    'ARMS LEFT UNDER cartesian_compliance_controller -- the '
                    'switch back to scaled_joint_trajectory_controller failed. '
                    'Do NOT run another movement until the controller is '
                    'restored (see the Switch to Joint (BOTH) button).')

    # Roll the recorded wrench up into the completion line so the terminal
    # reports the peak load even when the operator was not watching the plot.
    # Peak |force| per arm over the whole move; see the zeroing caveat above --
    # this includes the standing tool + bar weight.
    def _peak_force(profile):
        if not profile:
            return float('nan')
        return max(float(np.linalg.norm(w[:3])) for w in profile)

    monitor.get_logger().info(
        f"Compliant exec done for {monitor.current_movement.movement_id} "
        f"({kind.value}); {len(wrench_profile_left)} wrench samples, "
        f"peak |F| L={_peak_force(wrench_profile_left):.1f} N / "
        f"R={_peak_force(wrench_profile_right):.1f} N")


# Seconds to let the arms come to rest after a trajectory before taring the FT
# sensors. The zero must be taken standing still, or whatever the controller is
# still settling out gets baked into the offset.
FT_ZERO_SETTLE_SECONDS = 2.0


def execute_trajectory_and_zero_ft(monitor):
    """Run the loaded joint trajectory, then re-zero both arms' FT sensors.

    Used for M0, and only M0. M0 ends parked at the bar-loading position with
    the tools still EMPTY -- the operator mounts the bar right afterwards, and
    M1 carries it from there. That makes the end of M0 the one moment in the
    whole action when taring the sensors is correct:

      - before it, the arms have been moving and any earlier zero has drifted;
      - after it, every reading includes the bar's weight, so zeroing would
        subtract exactly the load the compliant M2/M3 execution must measure
        (the same reason execute_planned_trajectory_compliant refuses to zero).

    So the tare lands on a settled, unloaded tool, and the force plot during M2
    then reads contact rather than payload.

    Args:
        monitor (HuskyMonitor): Provides the planned trajectory, the husky
            interface and the trajectory time.

    Yields:
        None: One yield per monitor tick while waiting out the motion. Pumped
        by ``monitor.update``'s task loop.
    """
    traj_time = float(monitor.trajectory_time) or 5.0
    execute_arm_trajectory_both(monitor, traj_time)

    # ! Wait by TIME, not by hi.is_arm_executing -- same reason as the servoing
    # loop: that flag clears after only ARM_NOT_EXECUTING_TIME (1 s) of no joint
    # change, so a controller slow to start trips it while the arm is still
    # moving and we would tare mid-motion.
    tick_dt = 0.05  # matches the 20 Hz monitor tick that pumps this task
    for _ in range(int(np.ceil((traj_time + FT_ZERO_SETTLE_SECONDS) / tick_dt))):
        yield

    if monitor.FAKE_HARDWARE:
        monitor.get_logger().info(
            'Travel to load done; skipping FT zero (FAKE_HARDWARE, no sensor to tare).')
        return

    hi: HuskyRobotInterface = monitor.huskies[monitor.selected_robot_id].interface
    zeroed = [hi.zero_ft_sensor(i) for i in range(monitor.get_active_arm_count())]
    if all(zeroed):
        monitor.get_logger().info(
            'Travel to load done; FT sensors zeroed on the unloaded tools. Mount the bar '
            'now, then load and execute the transfer.')
    else:
        monitor.get_logger().warn(
            'Travel to load done, but the FT zero did not go out on every arm -- the '
            'compliant M2/M3 force readings will carry a stale offset.')


MOVE_TO_MOVEMENT_START_MAX_DELTA_RAD = np.pi / 3.0


def move_arms_to_movement_start(monitor: 'HuskyMonitor') -> None:
    """Send a 2-waypoint joint trajectory (current -> target) on every arm,
    taking the live arms to `current_movement.start_state.robot_configuration`.

    Two arms (Cindy) go out as one dual-arm command; a single-arm robot gets
    one plain arm command on arm 0. Joint names come from the loaded cell
    (``monitor._arm_joint_name_sets()``). With FAKE_HARDWARE nothing is
    sent: the simulated arms play the same 2-waypoint path
    (``_fake_execute_arm_trajectories``).

    Safety guard: refuses if any arm's per-joint max |delta| exceeds
    pi/3 rad. Protects against large unintended sweeps when the live arms
    are far from the planned start.

    Args:
        monitor: The HuskyMonitor node.
    """
    if monitor.current_movement is None:
        monitor.get_logger().warn(
            "No movement loaded; click 'Load Movement' first.")
        return
    mv = monitor.current_movement
    if mv.start_state is None or mv.start_state.robot_configuration is None:
        monitor.get_logger().warn(
            f"Movement {mv.movement_id!r} has no start_state.robot_configuration.")
        return
    rc = mv.start_state.robot_configuration
    try:
        targets = [np.array([rc[n] for n in names], dtype=float)
                   for names in monitor._arm_joint_name_sets()]
    except KeyError as e:
        monitor.get_logger().warn(
            f"start_state missing joint key {e}; cannot build target conf.")
        return

    hi: HuskyRobotInterface = monitor.huskies[monitor.selected_robot_id].interface
    currents = [np.asarray(hi.arm_joint_pose[i], dtype=float) for i in range(len(targets))]

    deltas = [float(np.max(np.abs(t - c))) for t, c in zip(targets, currents)]
    # Cindy keeps her historical "L=... R=..." log format.
    tags = ('L', 'R') if len(targets) == 2 else ('arm',)
    deltas_txt = ' '.join(f"{tag}={d:.3f}" for tag, d in zip(tags, deltas))
    limit = MOVE_TO_MOVEMENT_START_MAX_DELTA_RAD
    if any(d > limit for d in deltas):
        monitor.get_logger().warn(
            f"Refusing move to {mv.movement_id!r} start: max |delta_q| "
            f"{deltas_txt} rad exceeds pi/3 "
            f"({limit:.3f} rad)."
        )
        return

    t_total = float(monitor.trajectory_time) or 5.0
    monitor.get_logger().info(
        f"Moving arms to {mv.movement_id!r} start "
        f"(max |delta_q| {deltas_txt} rad, "
        f"t={t_total:.1f}s)"
    )
    if monitor.FAKE_HARDWARE:
        # * No robot to command: move the simulated arms, like the fake-hardware exec.
        _fake_execute_arm_trajectories(
            monitor, [([c, t], None, t_total, None) for c, t in zip(currents, targets)], t_total)
    elif len(targets) == 2:
        multi_arm_trajectory = [
            ([currents[0], targets[0]], None, t_total, None),
            ([currents[1], targets[1]], None, t_total, None),
        ]
        hi.send_dual_arm_cmd(multi_arm_trajectory)
    else:
        hi.send_arm_cmd([currents[0], targets[0]], None, t_total, index=0)


# ---------------------------------------------------------------------------
# * Schedule steps where no arm path is planned: gripper, scaffolding tool, manual
# ---------------------------------------------------------------------------
# Each one is a generator for monitor.tasks (the monitor tick pumps it): every
# one waits -- for the operator's 'Confirm Exec', the gripper's answer or the
# tool's stall flag -- and must never block the tick while it does.

def _gripper_server_ready(hi: HuskyRobotInterface, arm_index: int) -> bool:
    """Whether the arm's Robotiq gripper action server answers.

    The action client is created for every connected husky, even one with no
    gripper (Cindy) or in FAKE_HARDWARE, so the client alone proves nothing.

    Args:
        hi (HuskyRobotInterface): The husky's interface.
        arm_index (int): Arm index (0 = the only arm of a support robot).

    Returns:
        bool: True when the gripper action server is ready.
    """
    clients = getattr(hi, 'act_grippers', None)
    return bool(clients) and arm_index < len(clients) and clients[arm_index].server_is_ready()


def _gripper_result_text(result: dict) -> str:
    """One-line summary of a gripper result dict, for the log.

    Args:
        result (dict): ``hi.gripper_result[i]`` (position may be None for a rejected goal).

    Returns:
        str: e.g. ``'position 0.612, stalled True, reached_goal False'``.
    """
    pos = result.get('position')
    pos_txt = 'unknown' if pos is None else f'{pos:.3f}'
    return f"position {pos_txt}, stalled {result.get('stalled')}, reached_goal {result.get('reached_goal')}"


def _wait_for_gripper_result(monitor: 'HuskyMonitor', arm_index: int, timeout_s: float,
                             tag: str, must_reach_goal: bool = False) -> Generator[None, None, Optional[dict]]:
    """Yield until the gripper answers the goal just sent, or the ceiling expires.

    Args:
        monitor (HuskyMonitor): Provides the active husky's interface and the logger.
        arm_index (int): Arm the goal went to.
        timeout_s (float): Ceiling on the wait, in seconds.
        tag (str): Log prefix.
        must_reach_goal (bool): Warn (instead of the plain info line) when the
            gripper did not reach its goal position. True for an open, which
            has nothing to stop on; a close normally stalls on the bar instead.

    Yields:
        None: One yield per monitor tick while waiting.

    Returns:
        dict | None: The gripper's result dict, or None when it did not answer in time.
    """
    hi: HuskyRobotInterface = monitor.huskies[monitor.selected_robot_id].interface
    deadline = time.time() + float(timeout_s)
    while hi.gripper_result[arm_index] is None:
        if time.time() > deadline:
            monitor.get_logger().warn(
                f'{tag} no result from gripper {arm_index} after {timeout_s:.1f}s; '
                f'check the gripper before going on.')
            return None
        yield
    result = hi.gripper_result[arm_index]
    # ! One log line per severity (rclpy refuses to change a line's severity).
    if must_reach_goal and not result.get('reached_goal'):
        monitor.get_logger().warn(
            f'{tag} gripper {arm_index} did NOT reach its goal: {_gripper_result_text(result)}. '
            f'Check that nothing blocks the fingers.')
    else:
        monitor.get_logger().info(f'{tag} gripper {arm_index} done: {_gripper_result_text(result)}')
    return result


def _live_tool0_in_arm_base(monitor: 'HuskyMonitor', arm_index: int) -> tuple:
    """The arm's live tool0 pose in its arm base frame (what ``send_arm_cmd_cartesian`` takes).

    FK on the ghost robot (``monitor.goal_model``, same calibrated URDF as the
    real robot) at the LIVE arm joints ``hi.arm_joint_pose[arm_index]``; the
    ghost's joints are put back afterwards.

    ! Both links are read from the SAME body. tool0-in-arm-base depends only on
    ! the arm joints, so where the ghost's base sits does not matter -- but
    ! mixing bodies does: the ghost stands at the movement's base while the live
    ! robot stands at the mocap base, so tool0 from one and the arm base from the
    ! other would shift the target by the difference between the two bases.

    Args:
        monitor (HuskyMonitor): Provides the ghost, the live joints and the
            connected robot's joint / link names.
        arm_index (int): Arm index (0 = left or the only arm, 1 = right).

    Returns:
        tuple: pybullet (point, quat) pose of tool0 in the arm base frame.
    """
    hi: HuskyRobotInterface = monitor.huskies[monitor.selected_robot_id].interface
    spec = monitor._connected_robot()
    ghost = monitor.goal_model.robot
    joints = pp.joints_from_names(ghost, spec.arm_joint_names[arm_index])
    saved = pp.get_joint_positions(ghost, joints)
    try:
        pp.set_joint_positions(ghost, joints, np.asarray(hi.arm_joint_pose[arm_index], dtype=float))
        world_from_arm_base = pp.get_link_pose(ghost, pp.link_from_name(ghost, spec.arm_base_links[arm_index]))
        world_from_tool0 = pp.get_link_pose(ghost, pp.link_from_name(ghost, spec.flange_links[arm_index]))
        return pp.multiply(pp.invert(world_from_arm_base), world_from_tool0)
    finally:
        pp.set_joint_positions(ghost, joints, saved)


# * Compliant gripper handoff limits.
# How far the live tool0 may move from the pose where the arm went compliant
# before the hold is ended (the arm then goes back to joint tracking).
GRIP_HANDOFF_MAX_DRIFT_M = 0.02  # ? tune on hardware
# ! Must match HuskyRobotInterface.send_arm_cmd_cartesian: it drops (with a
# ! warning) every target further than this from the arm-reported TCP, per
# ! position axis (m) and per quaternion component.
CARTESIAN_TARGET_GUARD_TOL = 0.05
# Wait after zeroing the F/T sensor so the zero is done before the fingers move.
GRIP_FT_ZERO_SETTLE_S = 0.5


def grip_with_compliant_handoff(monitor: 'HuskyMonitor', close_pos: float = GRIPPER_CLOSE_FOR_BAR_POS,
                                effort: float = GRIPPER_EFFORT, switch_fraction: float = 0.8,
                                timeout_s: float = 30.0, arm_index: int = 0) -> Generator[None, None, bool]:
    """Close the support gripper on the bar, the arm going compliant for the end of the stroke.

    As the fingers close they pull the bar -- and so the arm -- into their
    centre. A stiff arm fights that; a compliant one gives way like a spring.
    Modeled on ``execute_planned_trajectory_compliant``:

      1. make sure the arm is under scaled_joint_trajectory_controller, then
         zero its F/T sensor (the fingers are still open and touch nothing);
      2. send the close goal and watch the gripper's feedback position;
      3. once the fingers have covered ``switch_fraction`` of the stroke, capture
         the arm's tool0 pose while it is still stiff (the ANCHOR), check it
         against the TCP the arm itself reports, and switch the arm to
         cartesian_compliance_controller (no handoff if either step fails);
      4. every tick, publish the ANCHOR as the compliance target and a zero
         wrench, so the arm springs around that pose instead of pushing back,
         until the gripper's result arrives (stalled on the bar or goal
         reached). The hold ends early if the arm moves more than
         ``GRIP_HANDOFF_MAX_DRIFT_M`` from the anchor;
      5. ``finally``: ALWAYS switch back to scaled_joint_trajectory_controller,
         with a loud error if that fails.

    Without CONNECT_COMPLIANT_CONTROLLER, with FAKE_HARDWARE, or when no
    gripper server answers, this is a plain close with a warning. 'Cancel Exec'
    (``_servo_abort``) ends the wait early; the gripper goal is NOT cancelled
    (it keeps closing with the arm stiff) and the finally still restores the
    joint controller.

    Args:
        monitor (HuskyMonitor): Provides the husky interface, the ghost robot
            for the FK, the flags and the logger.
        close_pos (float): Gripper target position ('Close Gripper for Bar').
        effort (float): Gripper max effort.
        switch_fraction (float): Share of the stroke (from the start position to
            ``close_pos``, by feedback position) after which the arm goes compliant.
            ? Tune on hardware: it must trigger BEFORE the fingers touch the bar,
            ? or the gripper stalls first and no handoff happens.
        timeout_s (float): Ceiling on the whole grip, from the close command.
        arm_index (int): Arm whose gripper closes (0 for a support robot).

    Yields:
        None: One yield per monitor tick.

    Returns:
        bool: True when the gripper reported its result (stalled or goal
        reached); False on a timeout, a cancel, or a failed step.
    """
    hi: HuskyRobotInterface = monitor.huskies[monitor.selected_robot_id].interface
    log = monitor.get_logger()
    tag = f'[grip arm {arm_index}]'
    joint_ctrl = 'scaled_joint_trajectory_controller'
    compliance_ctrl = 'cartesian_compliance_controller'

    # * No compliance controller, simulated hardware, or no gripper: a plain close.
    if (not monitor.CONNECT_COMPLIANT_CONTROLLER or monitor.FAKE_HARDWARE
            or not _gripper_server_ready(hi, arm_index)):
        if not monitor.CONNECT_COMPLIANT_CONTROLLER:
            why = 'CONNECT_COMPLIANT_CONTROLLER is 0'
        elif monitor.FAKE_HARDWARE:
            why = 'FAKE_HARDWARE is 1'
        else:
            why = 'no gripper action server is ready'
        log.warn(f'{tag} {why}: plain close WITHOUT the compliant handoff '
                 f'(the arm stays stiff under {joint_ctrl}).')
        if not hi.send_gripper_cmd(close_pos, effort, arm_index):
            return False
        result = yield from _wait_for_gripper_result(monitor, arm_index, GRIPPER_RESULT_TIMEOUT_S, tag)
        return result is not None

    # * Phase 1: the arm starts under the joint controller.
    log.info(f'{tag} phase 1/4: ensuring {joint_ctrl} on arm {arm_index}.')
    if not (yield from switch_arm_controllers(monitor, joint_ctrl, arm_indices=[arm_index])):
        log.error(f'{tag} the arm is NOT under {joint_ctrl}; nothing sent. '
                  f"Close by hand with 'Close Gripper for Bar' if needed.")
        return False

    # Zero the F/T sensor now, while the fingers are open and touch nothing, so
    # the compliance controller does not read a stale offset as a force to
    # follow. (Never zero with a bar in the gripper: it would tare its weight.)
    zero_ft_sensor = getattr(hi, 'zero_ft_sensor', None)
    if zero_ft_sensor is not None and zero_ft_sensor(arm_index):
        log.info(f'{tag} F/T sensor zeroed (fingers open, touching nothing); '
                 f'closing in {GRIP_FT_ZERO_SETTLE_S:.1f}s.')
        t_zero = time.time()
        while time.time() - t_zero < GRIP_FT_ZERO_SETTLE_S:
            yield

    compliance_requested = False  # the phase-3 switch request went out
    switched = False              # ... and was acknowledged in time
    closing = False               # the task was closed from outside (GeneratorExit)
    try:
        # * Phase 2: close, the arm still stiff.
        # Where the fingers are now: the previous goal's final position, else
        # the last feedback, else fully open. Read before sending: the send
        # clears both.
        start = GRIPPER_OPEN_POS
        for known in (hi.gripper_result[arm_index], hi.gripper_feedback[arm_index]):
            if known and known.get('position') is not None:
                start = known['position']
                break
        if not hi.send_gripper_cmd(close_pos, effort, arm_index):
            log.error(f'{tag} the close goal was not sent; nothing to hand over.')
            return False
        deadline = time.time() + float(timeout_s)
        span = close_pos - start
        log.info(f'{tag} phase 2/4: close goal sent (position {start:.3f} -> {close_pos:.3f}, '
                 f'effort {effort}); arm under {joint_ctrl} until {switch_fraction:.0%} of the stroke.')
        while True:
            fb = hi.gripper_feedback[arm_index]
            if fb is not None and fb.get('position') is not None:
                # A zero-length stroke (already at close_pos) counts as done.
                frac = (fb['position'] - start) / span if abs(span) > 1e-6 else 1.0
                if frac >= switch_fraction:
                    break
            if hi.gripper_result[arm_index] is not None:
                log.warn(f'{tag} the gripper finished before {switch_fraction:.0%} of its stroke '
                         f'({_gripper_result_text(hi.gripper_result[arm_index])}); '
                         f'no compliant handoff happened.')
                return True
            if time.time() > deadline:
                log.warn(f'{tag} no gripper feedback past {switch_fraction:.0%} of the stroke '
                         f'within {timeout_s:.1f}s; no compliant handoff.')
                return False
            if getattr(monitor, '_servo_abort', False):
                log.warn(f'{tag} cancelled by the operator before the handoff; the gripper goal '
                         f'is NOT cancelled and keeps closing with the arm stiff.')
                return False
            yield

        # * Phase 3: hand the arm over to compliance.
        # The anchor: tool0 where the arm is NOW, still stiff. Phase 4 holds it.
        anchor = _live_tool0_in_arm_base(monitor, arm_index)
        # ! send_arm_cmd_cartesian drops every target that is off the arm-reported
        # ! TCP (hi.arm_tcp_pose, same arm base frame) by more than the guard
        # ! tolerance. If FK and the arm disagree that much, every target of the
        # ! hold would be dropped -- so do not go compliant at all.
        tcp = hi.arm_tcp_pose[arm_index]
        if not (np.isclose(tcp[0], anchor[0], atol=CARTESIAN_TARGET_GUARD_TOL).all()
                and np.isclose(tcp[1], anchor[1], atol=CARTESIAN_TARGET_GUARD_TOL).all()):
            apart_mm = 1000.0 * float(np.linalg.norm(np.subtract(tcp[0], anchor[0])))
            log.error(f'{tag} FK tool0 and the arm-reported TCP disagree by more than 5 cm '
                      f'(or 0.05 in a quaternion component): position {apart_mm:.0f} mm apart, '
                      f'FK {np.round(anchor[0], 3).tolist()} / {np.round(anchor[1], 3).tolist()} vs '
                      f'TCP {np.round(tcp[0], 3).tolist()} / {np.round(tcp[1], 3).tolist()}. '
                      f'No handoff, the gripper closes with the arm stiff.')
            return False
        log.info(f"{tag} phase 3/4: gripper at {fb['position']:.3f} ({frac:.0%} of the stroke); "
                 f'switching arm {arm_index} {joint_ctrl} -> {compliance_ctrl}.')
        compliance_requested = True
        switched = yield from switch_arm_controllers(monitor, compliance_ctrl, arm_indices=[arm_index])
        if not switched:
            log.error(f'{tag} switch to {compliance_ctrl} FAILED: no handoff, the gripper '
                      f'keeps closing with the arm stiff.')
            return False

        # * Phase 4: hold the anchor, zero wrench, until the gripper is done.
        # ! The target stays FIXED at the anchor. Re-reading the live pose every
        # ! tick would move the target along with the arm, so the controller's
        # ! stiffness would never pull back and the arm would drift along any
        # ! force it senses.
        limit_mm = GRIP_HANDOFF_MAX_DRIFT_M * 1000.0
        log.info(f'{tag} phase 4/4: arm under {compliance_ctrl} (target = tool0 pose captured at '
                 f'the switch, zero wrench) until the gripper stalls or reaches its goal; the hold '
                 f'ends if the arm moves more than {limit_mm:.0f} mm from that pose.')
        while True:
            live = _live_tool0_in_arm_base(monitor, arm_index)
            drift = float(np.linalg.norm(np.subtract(live[0], anchor[0])))
            if drift > GRIP_HANDOFF_MAX_DRIFT_M:
                log.error(f'{tag} arm moved {drift * 1000.0:.1f} mm from where it went compliant '
                          f'(limit {limit_mm:.0f} mm); ending the hold.')
                return False
            hi.send_arm_cmd_cartesian(anchor, arm_index)
            hi.send_arm_cmd_cartesian_force([0.0, 0.0, 0.0], arm_index)
            result = hi.gripper_result[arm_index]
            if result is not None:
                log.info(f'{tag} gripper done: {_gripper_result_text(result)}.')
                if not (result.get('stalled') or result.get('reached_goal')):
                    log.warn(f'{tag} the gripper goal ended without stalling or reaching '
                             f'its goal (status {result.get("status")}); check the grip.')
                return bool(result.get('stalled') or result.get('reached_goal'))
            if time.time() > deadline:
                log.error(f'{tag} no gripper result within {timeout_s:.1f}s of the close command; '
                          f'ending the compliant hold. Check the grip.')
                return False
            if getattr(monitor, '_servo_abort', False):
                log.warn(f'{tag} cancelled by the operator during the compliant hold; the gripper '
                         f'goal is NOT cancelled and keeps closing with the arm stiff.')
                return False
            yield
    except GeneratorExit:
        # The task was closed from outside mid-step. A closed generator must not
        # yield again, so the finally below can only REQUEST the switch back.
        closing = True
        raise
    finally:
        if closing:
            not_joint = hi.active_controller[arm_index] != joint_ctrl
            if not_joint:
                hi.switch_controller(hi.active_controller[arm_index], joint_ctrl, arm_index)
            if compliance_requested or not_joint:
                log.error(
                    f'{tag} the grip step was stopped from outside during the handoff. The switch of '
                    f'ARM {arm_index} back to {joint_ctrl} was only REQUESTED, not confirmed: verify '
                    f"the arm is under {joint_ctrl} (use 'Switch to Joint (BOTH)') before running "
                    f'anything else.')
            else:
                log.warn(f'{tag} the grip step was stopped from outside before the handoff; the arm '
                         f'stayed under {joint_ctrl}. The gripper goal is NOT cancelled.')
        else:
            if compliance_requested and not switched:
                # ! The phase-3 wait gave up, but controller_manager may still
                # ! answer late and put the arm under compliance with nobody
                # ! publishing its target. Watch a while longer; an explicit
                # ! rejection ends the watch at once (the switch did not happen).
                late_deadline = time.time() + CONTROLLER_SWITCH_TIMEOUT_S
                while (hi.active_controller[arm_index] != compliance_ctrl
                       and not hi.controller_switch_error[arm_index]
                       and time.time() < late_deadline):
                    yield
                if hi.active_controller[arm_index] == compliance_ctrl:
                    log.warn(f'{tag} the switch to {compliance_ctrl} was acknowledged late; '
                             f'switching arm {arm_index} back now.')
                elif not hi.controller_switch_error[arm_index]:
                    log.error(
                        f'{tag} the switch of ARM {arm_index} to {compliance_ctrl} never answered and '
                        f'may still take effect later: verify the arm is under {joint_ctrl} '
                        f"(use 'Switch to Joint (BOTH)') before running anything else.")
            # * Always back to joint tracking (returns at once when already there).
            log.info(f'{tag} restoring {joint_ctrl} on arm {arm_index}.')
            restored = yield from switch_arm_controllers(monitor, joint_ctrl, arm_indices=[arm_index])
            if not restored:
                log.error(
                    f'ARM {arm_index} MAY BE LEFT UNDER {compliance_ctrl} -- the switch back to '
                    f'{joint_ctrl} failed. Do NOT run another movement until the controller is '
                    f"restored (see the 'Switch to Joint (BOTH)' button).")
        # A 'Cancel Exec' used in this step must not cancel the next step's confirm.
        monitor._servo_abort = False


def exec_gripper_tool_movement(monitor: 'HuskyMonitor',
                               mv: GripperToolMovement) -> Generator[None, None, None]:
    """Run a support robot's gripper step (``GripperToolMovement``) once the operator confirms.

    'open'  -> open the gripper fully and wait for its answer (short ceiling).
    'close' -> close on the bar with ``grip_with_compliant_handoff`` (a plain
               close, with a warning, without the compliance controller).

    Args:
        monitor (HuskyMonitor): Provides the husky interface, the flags, the
            confirm / cancel buttons' state and the logger.
        mv (GripperToolMovement): The loaded gripper step.

    Yields:
        None: One yield per monitor tick.
    """
    # Clear a stale 'Cancel Exec' so it cannot skip this step's confirm.
    monitor._servo_abort = False
    try:
        tool_action, tool_names, _ = tool_event(mv)
        tag = f'[gripper step {mv.movement_id}]'
        if tool_action == 'open':
            what = f'OPEN the gripper fully (position {GRIPPER_OPEN_POS})'
        elif tool_action == 'close':
            compliant = monitor.CONNECT_COMPLIANT_CONTROLLER and not monitor.FAKE_HARDWARE
            what = (f'CLOSE the gripper on the bar (position {GRIPPER_CLOSE_FOR_BAR_POS}), '
                    + ('the arm going compliant for the end of the stroke' if compliant
                       else 'plain close: CONNECT_COMPLIANT_CONTROLLER is 0 or FAKE_HARDWARE is 1, '
                            'the arm stays stiff'))
        else:
            monitor.get_logger().warn(f'{tag} unknown gripper action {tool_action!r}; nothing sent.')
            return
        if not (yield from wait_for_operator_confirm(
                monitor,
                f"{tag} {what} [{', '.join(tool_names)}]. Click 'Confirm Exec' to send it, "
                f"'Cancel Exec' to skip.")):
            return
        if tool_action == 'open':
            if open_gripper_full(monitor):
                yield from _wait_for_gripper_result(monitor, 0, GRIPPER_RESULT_TIMEOUT_S, tag,
                                                    must_reach_goal=True)
        else:
            yield from grip_with_compliant_handoff(monitor)
    finally:
        # A 'Cancel Exec' used in this step must not cancel the next step's confirm.
        monitor._servo_abort = False


def run_manual_step(monitor: 'HuskyMonitor', mv: ManualMovement) -> Generator[None, None, None]:
    """Pause for a step the operator does by hand (``ManualMovement``), until they confirm it.

    Args:
        monitor (HuskyMonitor): Provides the confirm / cancel buttons' state
            and the logger.
        mv (ManualMovement): The loaded manual step.

    Yields:
        None: One yield per monitor tick.
    """
    # Clear a stale 'Cancel Exec' so it cannot skip this step's confirm.
    monitor._servo_abort = False
    tag = f'[manual step {mv.movement_id}]'
    try:
        if (yield from wait_for_operator_confirm(
                monitor,
                f"{tag} {mv.tag!r}: do this by hand now, then click 'Confirm Exec' "
                f"('Cancel Exec' if it was not done).")):
            monitor.get_logger().info(f'{tag} confirmed done by the operator.')
    finally:
        # A 'Cancel Exec' used in this step must not cancel the next step's confirm.
        monitor._servo_abort = False


def run_scaffolding_tool_step(monitor: 'HuskyMonitor',
                              mv: ScaffoldingToolMovement) -> Generator[None, None, None]:
    """Run one of Cindy's scaffolding-tool steps (``ScaffoldingToolMovement``).

    - 'tighten' with ``overlaps_next`` (J_M4) and 'ungrasp' (R_M1): mark only.
      ``execute_planned_trajectory_compliant`` already sends them: the compliant
      insert (M2) starts the joint motors TIGHTENING, the compliant retreat (M3)
      starts the gripper motors LOOSENING.
    - 'untighten' (R_M0): mark only, on purpose (user decision 2026-09-30). The
      proven compliant retreat (M3) never reverses the JOINT motor, and doing so
      may back the just-tightened joint screw off the bar. If the tool has to
      back off, the operator uses the manual 'Loosen Joint' button.
    - 'grasp' (J_M2): after the operator confirms, STOP (clears a left-over
      stall), then gripper motor TIGHTEN (+1) on every arm until each reports
      STALLED (ceiling HOLD_FOR_STALL_TIMEOUT_S), then STOP.

    Motor and direction are the ones the manual 'Tighten Gripper' button sends.
    'Cancel Exec' during the wait stops the motors early.

    Args:
        monitor (HuskyMonitor): Provides the husky interface, the connected
            robot, the confirm / cancel buttons' state and the logger.
        mv (ScaffoldingToolMovement): The loaded tool step.

    Yields:
        None: One yield per monitor tick.
    """
    log = monitor.get_logger()
    tool_action, tool_names, overlaps_next = tool_event(mv)
    tag = f'[tool step {mv.movement_id}]'
    if overlaps_next or tool_action == 'ungrasp':
        runs_in = ('the compliant insert that follows' if overlaps_next
                   else 'the compliant retreat')
        log.info(f"{tag} '{tool_action}' is issued by the compliant insert/retreat "
                 f"execution ({runs_in}); marked done here, nothing sent.")
        return
    if tool_action == 'untighten':
        # ! Mark only (see the docstring): the joint motor is never reversed here.
        log.info(f"{tag} 'untighten' is left to the operator, like the proven M3 flow: "
                 f"nothing sent. Use the manual 'Loosen Joint' button if the tool must back off.")
        return

    hi: HuskyRobotInterface = monitor.huskies[monitor.selected_robot_id].interface
    spec = monitor._connected_robot()
    arms = list(range(spec.n_arms))
    where = f"the {' + '.join(spec.side_keys)} tool(s) [{', '.join(tool_names)}]"
    if tool_action == 'grasp':
        motor, direction, motor_txt = GRIPPER_MOTOR, 1, 'gripper motor (tool M1) TIGHTEN (direction +1)'
        prompt = (f'{tag} GRASP: STOP (clears a left-over stall), then {motor_txt} on {where} '
                  f'until it reports STALLED (at most {HOLD_FOR_STALL_TIMEOUT_S:.0f}s), then STOP.')
    else:
        log.warn(f'{tag} no handler for scaffolding action {tool_action!r} '
                 f'(overlaps_next={overlaps_next}); nothing sent.')
        return

    # Clear a stale 'Cancel Exec' so it cannot skip this step's confirm.
    monitor._servo_abort = False
    try:
        if not (yield from wait_for_operator_confirm(
                monitor, f"{prompt} Click 'Confirm Exec' to send it, 'Cancel Exec' to skip.")):
            return

        # ! STOP first, as Cindy's compliant flow does (_stop_all_both_arms):
        # ! STOP clears the tool's stall flag, and the firmware refuses to
        # ! start a STALLED motor ("ERR 2 STALLED", see crl_husky
        # ! onboard/protocol.md) -- e.g. a gripper motor left STALLED by a
        # ! manual 'Tighten Gripper' with no STOP after it.
        for i in arms:
            print(f'[scaffolding] {tag}: STOP on arm {i} (clears a left-over stall)')
            hi.send_scaffolding_cmd(0, motor, i)
        for i in arms:
            print(f'[scaffolding] {tag}: {motor_txt} on arm {i}')
            hi.send_scaffolding_cmd(direction, motor, i)
        t0 = time.time()
        try:
            # ! The driver polls the tool's status at 20 Hz, so the first
            # ! reading(s) after the command can still be the old STALLED from
            # ! before the STOP. An arm counts only once it has shown NOT
            # ! stalled since the command; otherwise that stale reading would
            # ! end the grasp on its very first tick.
            moving = [False] * len(arms)
            while True:
                for k, i in enumerate(arms):
                    s = hi.scaffolding_status[i]
                    if s is not None and s.state_m1 != 'STALLED':
                        moving[k] = True
                if all(moving[k] and _scaffolding_gripper_motor_stalled(hi, i)
                       for k, i in enumerate(arms)):
                    log.info(f'{tag} every gripper motor STALLED after {time.time() - t0:.1f}s: bar clamped.')
                    break
                if time.time() - t0 > HOLD_FOR_STALL_TIMEOUT_S:
                    log.warn(f'{tag} hit the {HOLD_FOR_STALL_TIMEOUT_S:.0f}s ceiling without every '
                             f'gripper motor reporting STALLED; check the grip.')
                    break
                if getattr(monitor, '_servo_abort', False):
                    log.warn(f'{tag} cancelled by the operator; stopping the motors.')
                    break
                yield
        finally:
            # Direction 0 = STOP (the tool stops both of its motors).
            for i in arms:
                hi.send_scaffolding_cmd(0, motor, i)
            log.info(f'{tag} STOP sent to {where}.')
    finally:
        # A 'Cancel Exec' used in this step must not cancel the next step's confirm.
        monitor._servo_abort = False


def move_left_linear_z(monitor, length, speed):
    husky = monitor.huskies[monitor.selected_robot_id]
    hi: HuskyRobotInterface = husky.interface
    robot = husky.object.robot
    
    # DISABLED 2026-05-15: hi.set_screw API is outdated and may damage the tool
    # hardware. Re-enable only after the screw-motor firmware/API is updated.
    # if length > 0:
    #     hi.set_screw(False, 0)
    #     hi.set_screw(True, 0)
    # else:
    #     hi.set_screw(True, 0)
    #     hi.set_screw(False, 0)

    trajectory, _ = generate_insertion_motion_bar(monitor, length, speed)
    hi.send_arm_cmd(trajectory[0], trajectory[1], trajectory[2], index=0)
    
def generate_insertion_motion_bar(monitor, depth, speed, cartesian_speedup=1, neutral_start_pose=None):
    husky = monitor.huskies[monitor.selected_robot_id]
    hi: HuskyRobotInterface = husky.interface
    robot = husky.object.robot
    
    obstacles = list(monitor.static_obstacles.values())
    attachments = [[husky.object.ee_list[0][1]], [husky.object.ee_list[1][1]]]
    start_pose, to_left, to_right = compute_bar_pose_from_EE_poses(pp.get_link_pose(robot, pp.link_from_name(robot, 'left_ur_arm_tool0')), pp.get_link_pose(robot, pp.link_from_name(robot, 'right_ur_arm_tool0')))
    if neutral_start_pose is not None:
        start_pose = neutral_start_pose
        
    end_pose = pp.multiply(start_pose, pp.Pose(pp.Point(0, 0, depth)))
        
    left_gripper_start_pose = pp.multiply(start_pose, to_left)
    right_gripper_start_pose = pp.multiply(start_pose, to_right)
    
    left_gripper_end_pose = pp.multiply(end_pose, to_left)
    right_gripper_end_pose = pp.multiply(end_pose, to_right)
    
    init_conf_left = hi.arm_joint_pose[0]
    init_conf_right = hi.arm_joint_pose[1]
    
    time = max(1, abs(depth/speed))
    arm_trajectories = [([], None, time, None), ([], None, time, None)]
    cartesian_trajectories = [[left_gripper_start_pose, left_gripper_end_pose, time/cartesian_speedup, time - time/cartesian_speedup], [right_gripper_start_pose, right_gripper_end_pose, time/cartesian_speedup, time - time/cartesian_speedup]]
    
    for i in range(0, 5):
        pose = pp.multiply(start_pose, pp.Pose(pp.Point(0, 0, i * depth/4.0)))
        
        left_pose = pp.multiply(pose, to_left)
        right_pose = pp.multiply(pose, to_right)

        arm_conf_left = get_arm_ik_for_grasp_bar(husky.object.robot, planning.IK_SOLVER_DUAL[0], left_pose, attachments[0], obstacles, hint_conf=init_conf_left)
        arm_conf_right = get_arm_ik_for_grasp_bar(husky.object.robot, planning.IK_SOLVER_DUAL[1], right_pose, attachments[1], obstacles, hint_conf=init_conf_right)
        if arm_conf_left is None:
            monitor.get_logger().warn("IK left failed!")
            return None, cartesian_trajectories
        if arm_conf_right is None:
            monitor.get_logger().warn("IK right failed!")
            return None, cartesian_trajectories
        init_conf_left = arm_conf_left
        init_conf_right = arm_conf_right
        arm_trajectories[0][0].append(arm_conf_left)
        arm_trajectories[1][0].append(arm_conf_right)
        
    return arm_trajectories, cartesian_trajectories
            
          
# TODO adapt to dual arm and bar  
def generate_reset_trajectory_bar(monitor, speed, goal_pose):
    husky = monitor.huskies[monitor.selected_robot_id]
    hi: HuskyRobotInterface = husky.interface
    robot = husky.object.robot
    
    obstacles = list(monitor.static_obstacles.values())
    attachments = [[husky.object.ee_list[0][1]], [husky.object.ee_list[1][1]]]
    start_pose, to_left, to_right = compute_bar_pose_from_EE_poses(pp.get_link_pose(robot, pp.link_from_name(robot, 'left_ur_arm_tool0')), pp.get_link_pose(robot, pp.link_from_name(robot, 'right_ur_arm_tool0')))
    
    init_conf_left = hi.arm_joint_pose[0]
    init_conf_right = hi.arm_joint_pose[1]
    
    # TODO compute distance to compute time
    offset = np.array(pp.point_from_pose(start_pose)) - np.array(pp.point_from_pose(goal_pose))
    distance = np.linalg.norm(offset)
    
    time = max(1, abs(distance/speed))
    arm_trajectories = [([], None, time, None), ([], None, time, None)]
    
    bar_trajectory = pp.interpolate_poses_by_num_steps(start_pose, goal_pose, 5)
    
    for pose in bar_trajectory:
        left_pose = pp.multiply(pose, to_left)
        right_pose = pp.multiply(pose, to_right)

        arm_conf_left = get_arm_ik_for_grasp_bar(husky.object.robot, planning.IK_SOLVER_DUAL[0], left_pose, attachments[0], obstacles, hint_conf=init_conf_left)
        arm_conf_right = get_arm_ik_for_grasp_bar(husky.object.robot, planning.IK_SOLVER_DUAL[1], right_pose, attachments[1], obstacles, hint_conf=init_conf_right)
        if arm_conf_left is None:
            monitor.get_logger().warn("IK left failed!")
            return None
        if arm_conf_right is None:
            monitor.get_logger().warn("IK right failed!")
            return None
        init_conf_left = arm_conf_left
        init_conf_right = arm_conf_right
        arm_trajectories[0][0].append(arm_conf_left)
        arm_trajectories[1][0].append(arm_conf_right)
    
    return arm_trajectories
