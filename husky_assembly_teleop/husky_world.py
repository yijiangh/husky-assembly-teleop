"""
This module contains the world definition and high level actions or sequences of actions for the huskies.
"""

import os, time
import contextlib
import asyncio.runners
import asyncio
from matplotlib.pyplot import bar
import numpy as np
import copy
from husky_assembly_teleop.husky_robot import HuskyRobotInterface
import rclpy

import pybullet as p
import pybullet_planning as pp

from husky_assembly_teleop import DATA_DIRECTORY, CALIBRATION_DATE, EXPERIMENT_DATA_DIRECTORY
from husky_assembly_teleop.common import Husky, TrackedObject, AssemblyObject
import husky_assembly_teleop.husky_planning as planning
import husky_assembly_teleop.husky_control as control
from husky_assembly_teleop.utils import HUSKY_DUAL_UR5e_JOINT_NAMES, UR5E_JOINT_NAMES, MOCAP_SET_RIG_RB_NAME, conf_from_12vec, get_arm_ik_for_grasp_bar, get_custom_limits, notify, plan_transit_motion, pose_from_frame
from husky_assembly_teleop.scaffolding import parse_mt_geometric, create_collision_bodies, create_couplers, flatten_list
from husky_assembly_teleop.cfab_session import CfabSession, build_default_robot_cell
import json
from datetime import datetime

from compas_fab.robots import RobotCellState

import matplotlib.pyplot as plt
import compas

import cv2

assembly_objects = []

# Use the centralized DATA_DIRECTORY from the package
DATA_DIR = DATA_DIRECTORY

CALIB_DATA_DIR = os.path.join(DATA_DIR, "calibration_data")
BAR_HOLDING_ACC_DATA_DIR = os.path.join(DATA_DIR, "bar_holding_acc_data")
BAR_HOLDING_ACC_EXPERIMENT_DIR = os.path.join(EXPERIMENT_DATA_DIRECTORY, "bar_holding_acc_data")
DUAL_ARM_ACC_DATA_DIR = os.path.join(DATA_DIR, "dual_arm_acc_data")
CALIB_CONFIG_TEMPLATE = os.path.join(CALIB_DATA_DIR, "_data_template", "config.yaml")

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
                                   punch_tool_offset=None, connect_compliant_controller=False):
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
                connect_compliant_controller=connect_compliant_controller)

def init(monitor):
    # Per-robot config keyed by ROS_DOMAIN_ID so Alice 0804 (ROS_DOMAIN_ID=84),
    # Belle 0805 (ROS_DOMAIN_ID=85), and Cindy 0806 (ROS_DOMAIN_ID=86) can run in parallel
    # terminals without editing this file. Only fields that genuinely differ
    # live here; everything else (dual_arm, base_calibration_file,
    # punch_tool overrides) derives below.
    ROBOT_CONFIGS = {
        '84': dict(
            robot_namespace='/a200_0804',
            # Alice 0804 (ROS_DOMAIN_ID=84),
            mocap_id=1031,
            connect_gripper=True,
            ee_types_default=['robotiq_gripper'],
        ),
        '85': dict(
            robot_namespace='/a200_0805',
            # Belle 0805 (ROS_DOMAIN_ID=85),
            mocap_id=1021,
            connect_gripper=True,
            ee_types_default=['robotiq_gripper'],
        ),
        '86': dict(
            robot_namespace='/a200_0806',
            # Cindy 0806 (ROS_DOMAIN_ID=86),
            mocap_id=1011,
            connect_gripper=False,
            ee_types_default=['assembly_tool_v3_left', 'assembly_tool_v3_right'],
        ),
    }
    domain_id = os.environ.get('ROS_DOMAIN_ID', '86')
    cfg = ROBOT_CONFIGS.get(domain_id)
    if cfg is None:
        monitor.get_logger().warn(
            f"ROS_DOMAIN_ID={domain_id!r} not in ROBOT_CONFIGS; defaulting to 0806."
        )
        cfg = ROBOT_CONFIGS['86']

    robot_namespace = cfg['robot_namespace']
    mocap_id = cfg['mocap_id']
    robot_name = robot_namespace.split('_')[-1]
    dual_arm = (robot_name == '0806')

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
        ee_types = cfg['ee_types_default']
        punch_offset = None

    # When MOCAP_AXIS_CONVENTION='rhino', prefer a `_rhino`-tagged calibration
    # file (generated by data/calibration_data/convert_to_rhino.py). The values
    # are identical to the legacy file (calibration is convention-invariant);
    # the tag just makes the active convention explicit. Falls back to the
    # untagged file if the tagged one is missing.
    convention = getattr(monitor, 'MOCAP_AXIS_CONVENTION', 'rotated')
    suffix = '_rhino' if convention == 'rhino' else ''
    base_calibration_file = os.path.join(
        CALIB_DATA_DIR, CALIBRATION_DATE,
        f'calibrated_transformation_{robot_name}{suffix}.json',
    )
    if not os.path.exists(base_calibration_file):
        fallback = os.path.join(
            CALIB_DATA_DIR, CALIBRATION_DATE,
            f'calibrated_transformation_{robot_name}.json',
        )
        if suffix and os.path.exists(fallback):
            monitor.get_logger().warn(
                f'Rhino-tagged calibration not found ({base_calibration_file}); '
                f'falling back to {fallback}. Run data/calibration_data/convert_to_rhino.py '
                'to silence this warning.'
            )
            base_calibration_file = fallback
        else:
            monitor.get_logger().warn(
                f'Base calibration file not found for robot {robot_name}: {base_calibration_file}. '
                'Continuing without base calibration.'
            )
            base_calibration_file = None

    create_husky_with_end_effectors(
        monitor,
        name=robot_namespace,
        mocap_id=mocap_id,
        pos=np.array((0,0,0)),
        connect_arm=not monitor.FAKE_HARDWARE,
        connect_gripper=cfg['connect_gripper'] and not monitor.FAKE_HARDWARE,
        connect_compliant_controller=bool(getattr(monitor, 'CONNECT_COMPLIANT_CONTROLLER', 0)) and not monitor.FAKE_HARDWARE,
        calibration=monitor.CALIBRATION,
        dual_arm=dual_arm,
        # ee_types=["validation_tool_pair"],  # Specify end effectors for both arms
        # ee_types=["custom_gripper", "custom_gripper"],
        ee_types=ee_types,
        base_calibration_file=base_calibration_file,
        force_regenerate=False,
        punch_tool_offset=punch_offset,
    )
    
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
        TrackedObject(monitor, left_tool_name, 1013, np.zeros(3), np.array((0, 0, 0, 1)), 0.2)
        monitor.assign_calibration_tool_to_robot(0, 0, left_tool_name)

        right_tool_name = 'calib_tool_right'
        TrackedObject(monitor, right_tool_name, 1012, np.zeros(3), np.array((0, 0, 0, 1)), 0.2)
        monitor.assign_calibration_tool_to_robot(0, 1, right_tool_name)

    if monitor.BAR_ACTION_MOCAP_ACCURACY_TEST:
        bar_rig = TrackedObject(monitor, MOCAP_SET_RIG_RB_NAME, 1033, np.zeros(3), np.array((0, 0, 0, 1)), 0.2)
        bar_rig.body = pp.create_cylinder(radius=0.01, height=1, color=(1, 0, 0, 0.2))
        bar_rig.model_base_pose = pp.Pose(euler=pp.Euler(roll=np.pi/2))
        
    if monitor.DUAL_ARM_EE_CONSTR_ACCURACY_MOCAP_TEST:
        left_EE = TrackedObject(monitor, 'left_EE', 1013, np.zeros(3), np.array((0, 0, 0, 1)), 0.2)
        left_EE.body = pp.create_box(0.1, 0.1, 0.1)
        right_EE = TrackedObject(monitor, 'right_EE', 1012, np.zeros(3), np.array((0, 0, 0, 1)), 0.2)
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
    # save to data/calibration_data/<date_folder>/<data_batch>/
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

    # Resolve the chosen movement's short string role ('M1', 'M2', ...). We
    # key downstream processing (the 0_/1_ scripts) on this string id now, not
    # the integer list index, so movement_index is intentionally dropped.
    mv = monitor.current_movement
    movement_id = None
    if mv is not None:
        movement_id = monitor._match_movement_role(mv) or getattr(mv, 'movement_id', None)

    # Bar world pose + AABB dimensions in the movement's start state, stamped
    # here so the offline scripts don't have to re-parse the BarAction file
    # (older on-disk BarActions may no longer import cleanly).
    bar_pose = None
    try:
        bar_pose = monitor.get_movement_start_bar_pose()
    except Exception as e:
        monitor.get_logger().warn(f"could not compute start-state bar pose: {e}")
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
            'bar_action_path': getattr(monitor, '_current_action_path', None),
            'movement_id': movement_id,
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

def execute_task_goal_arm_trajectory_with_servoing(monitor, trajectory, index=0, log_data=False):
    if trajectory is None:
        monitor.get_logger().warn('Arm trajectory must be planed before executing!')
        return
    if trajectory[3] is None:
        monitor.get_logger().warn('Arm trajectory must be have a grasped element attached to specify task space goal!')
        return

    num_iters = 4
    settle_time = 2
    data = [{} for _ in range(num_iters)]

    obstacles = list(monitor.static_obstacles.values())
    
    hi = monitor.huskies[monitor.selected_robot_id].interface
    ho = monitor.huskies[monitor.selected_robot_id].object

    # get ideal tool0 pose, not related to mocap obs
    # TODO this should be generalized to any world_from_tool0 and attachment
    transfer_element = trajectory[3]
    world_from_tool0 = pp.multiply(transfer_element.goal_pose, pp.invert(transfer_element.grasp))
    attachments = [ho.ee_list[monitor.selected_arm_index][1], pp.Attachment(ho.robot, pp.link_from_name(ho.robot, 'ur_arm_tool0'), transfer_element.grasp, transfer_element.body)]

    # ! IMPORTANT
    # TODO ** This needs to take selected_arm_index into account, otherwise it will always use the first arm

    for iter_i in range(num_iters):
        monitor.get_logger().info(f'Servoing arm trajectory {iter_i+1}/{num_iters}...')

        data[iter_i]['before_exe_footprint_pose'] = copy.copy(hi.position), copy.copy(hi.rotation)

        # execute the trajectory
        if iter_i != 0:
            traj_time = 2
        else:
            traj_time = trajectory[2] 

        monitor.huskies[monitor.selected_robot_id].interface.send_arm_cmd(trajectory[0], trajectory[1], traj_time, index=index)

        # wait until it finishes
        # TODO hopefully the extra 2 seconds will be enough for the mocap estimation to roll in? To be checked
        # TODO: ideally this should let the ros node spin until the execution is done, while blocking the thread here
        # time.sleep(monitor.trajectory_time + 2)

        # Spin ROS node for 1 second to allow updated data to flow
        spin_time = traj_time + settle_time
        time.sleep(spin_time)

        # ! for some reasons, the spin_once will make the main node stop working after this function is finished
        # monitor.get_logger().info(f'Spinning ROS node for {spin_time} second to process incoming data...')
        # start_time = time.time()
        # while time.time() - start_time < spin_time:
        #     rclpy.spin_once(monitor, timeout_sec=0.1)
        #     print('hi position: {}, hi rotation: {}, arm conf: {}'.format(hi.position, hi.rotation, hi.arm_joint_pose))
        # monitor.get_logger().info('Finished spinning ROS node')

        # the footprint pose is updated bc the mocap works asynchronously
        data[iter_i]['after_exe_footprint_pose'] = copy.copy(hi.position), copy.copy(hi.rotation)

        # compute the difference between the before and after exe footprint pose
        diff_pos_vec = np.array(hi.position) - np.array(data[iter_i]['before_exe_footprint_pose'][0])
        diff_quat_vec = np.array(hi.rotation) - np.array(data[iter_i]['before_exe_footprint_pose'][1])
        # Convert position difference from meters to millimeters
        diff_pos_vec_mm = diff_pos_vec * 1000
        monitor.get_logger().info(f'Footprint pose diff: {diff_pos_vec_mm} mm, quat diff: {diff_quat_vec}')
        # raise warning if the diff is strictly zero
        if np.all(diff_pos_vec < 1e-9) and np.all(diff_quat_vec < 1e-9):
            monitor.get_logger().warn(f'Footprint pose diff is zero!')

        # ! until we make the ros main thread spin properly, we need to manually update the robot base pose in sim accroding to the mocap
        # ! we assume that the robot arm conf is exactly the last traj point
        hi.arm_joint_pose[monitor.selected_arm_index] = trajectory[0][-1]
        ho.set_pose((hi.position, hi.rotation), hi.arm_joint_pose)

        # compute current world_from_tool0
        observed_world_from_tool0 = ho.get_link_pose_from_name("ur_arm_tool0")
        # Compute position distance between observed and ideal tool0 poses
        pos_distance = np.linalg.norm(np.array(observed_world_from_tool0[0]) - np.array(world_from_tool0[0]))
        monitor.get_logger().info(f'tool0 pos difference: {pos_distance*1e3:.1f} mm')

        # Extract rotation matrices from quaternions
        observed_rotation = pp.matrix_from_quat(observed_world_from_tool0[1])
        ideal_rotation = pp.matrix_from_quat(world_from_tool0[1])
        
        # Extract individual axes from rotation matrices
        observed_axes = [observed_rotation[:3, i] for i in range(3)]  # x, y, z axes
        ideal_axes = [ideal_rotation[:3, i] for i in range(3)]  # x, y, z axes
        
        # Compute angle differences between corresponding axes
        axis_angles = []
        axis_names = ['x', 'y', 'z']
        for j in range(3):
            # Ensure normalized vectors
            v1 = observed_axes[j] / np.linalg.norm(observed_axes[j])
            v2 = ideal_axes[j] / np.linalg.norm(ideal_axes[j])
            # Compute angle between vectors (in degrees)
            dot_product = min(1.0, max(-1.0, np.dot(v1, v2)))
            angle = np.arccos(dot_product) * 180 / np.pi
            axis_angles.append(angle)
            monitor.get_logger().info(f'tool0 {axis_names[j]}-axis angle difference: {angle:.4f}°')

        data[iter_i]['observed_world_from_tool0'] = observed_world_from_tool0
        data[iter_i]['ideal_world_from_tool0'] = world_from_tool0
        data[iter_i]['world_from_tool0_pos_distance'] = pos_distance
        data[iter_i]['world_from_tool0_axis_angles'] = axis_angles

        pp.draw_pose(observed_world_from_tool0, length=0.2)  # Visualize observed pose
        pp.draw_pose(world_from_tool0, length=0.3, width=2)  # Visualize ideal pose
        # pp.camera_focus_on_point(world_from_tool0[0])

        # plan again for the same task goal, the ik will use the current arm conf as initial guess, and should succeed in the first iter
        arm_conf = planning.arm_ik(monitor.huskies[monitor.selected_robot_id], world_from_tool0, attachments, obstacles) #, hint_conf=trajectory[0][-1])

        if arm_conf is None:
            monitor.get_logger().warn("IK failed!")
            return

        trajectory = planning.plan_arm_motion(
            monitor.huskies[monitor.selected_robot_id], 
            arm_conf, 
            obstacles, 
            traj_time,
            grasped_element=monitor.goal_element, 
            grasp=monitor.goal_bar_grasp
            )
    
    # Plot position distance and axis angles across iterations
    if log_data:
        import matplotlib.pyplot as plt
        
        # Extract data for plotting
        iterations = list(range(1, num_iters + 1))
        pos_distances = [d['world_from_tool0_pos_distance']*1e3 for d in data]
        x_angles = [d['world_from_tool0_axis_angles'][0] for d in data]
        y_angles = [d['world_from_tool0_axis_angles'][1] for d in data]
        z_angles = [d['world_from_tool0_axis_angles'][2] for d in data]
        
        # Create figure with two subplots
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        # Plot position distance
        ax1.plot(iterations, pos_distances, 'o-', color='blue')
        ax1.set_xlabel('Iteration')
        ax1.set_ylabel('Position Distance (mm)')
        ax1.set_title('Tool0 Position Error Across Iterations')
        ax1.grid(True)
        
        # Plot axis angles
        ax2.plot(iterations, x_angles, 'o-', color='red', label='X-axis')
        ax2.plot(iterations, y_angles, 'o-', color='green', label='Y-axis')
        ax2.plot(iterations, z_angles, 'o-', color='blue', label='Z-axis')
        ax2.set_xlabel('Iteration')
        ax2.set_ylabel('Angular Difference (degrees)')
        ax2.set_title('Tool0 Orientation Error Across Iterations')
        ax2.legend()
        ax2.grid(True)
        
        plt.tight_layout()

        # Create a date-specific servoing subfolder
        servoing_subfolder_name = f"{datetime.now().strftime('%Y%m%d')}-servoing"
        servoing_subfolder_path = os.path.join(BAR_HOLDING_ACC_DATA_DIR, servoing_subfolder_name)

        # Create the subfolder if it doesn't exist
        if not os.path.exists(servoing_subfolder_path):
            os.makedirs(servoing_subfolder_path)
            monitor.get_logger().info(f"Created servoing subfolder: {servoing_subfolder_path}")

        # Update the plot and data file paths to use the new subfolder
        timestamp = datetime.now().strftime("%Y%m%d_%H%M")
        plot_filename = os.path.join(servoing_subfolder_path, f"servoing_performance_{timestamp}.png")
        data_filename = os.path.join(servoing_subfolder_path, f"servoing_data_{timestamp}.json")
        
        # Save the plot
        plt.savefig(plot_filename)
        monitor.get_logger().info(f"Performance plot saved to {plot_filename}")
 
        # Also save the data
        with open(data_filename, 'w') as f:
            json.dump({'servoing_data': data}, f, default=lambda x: str(x) if isinstance(x, np.ndarray) else x, indent=4)
        monitor.get_logger().info(f"Servoing data saved to {data_filename}")

        plt.show()

     
def move_base_to_goal(monitor):
    if monitor.planned_base_trajectory[0] is None:
        monitor.get_logger().warn('Base trajectory must be planed before executing!')
        return
    monitor.tasks.append(control.execute_base_trajectory(monitor, monitor.huskies[0], monitor.planned_base_trajectory))
    
#################################

def open_gripper_full(monitor):
    monitor.huskies[monitor.selected_robot_id].interface.send_gripper_cmd(0.426, 0.1)

def close_gripper_for_bar(monitor):
    monitor.huskies[monitor.selected_robot_id].interface.send_gripper_cmd(0.8, 0.1)

def set_gripper(monitor):
    monitor.huskies[monitor.selected_robot_id].interface.send_gripper_cmd(monitor.goal_gripper, 0.1)
    
####################################

def execute_arm_trajectory_both(monitor):
    if monitor.planned_arm_trajectory[0][0] is None:
        monitor.get_logger().warn('Arm trajectory must be planed before executing! [LEFT]')
        return
    if monitor.planned_arm_trajectory[1][0] is None:
        monitor.get_logger().warn('Arm trajectory must be planed before executing! [RIGHT]')
        return
    
    if not monitor.FAKE_HARDWARE:
        # Update the trajectory time in both planned_arm_trajectory tuples to match monitor.trajectory_time
        monitor.planned_arm_trajectory[0] = (
            monitor.planned_arm_trajectory[0][0],
            monitor.planned_arm_trajectory[0][1],
            monitor.trajectory_time,
            monitor.planned_arm_trajectory[0][3]
        )
        monitor.planned_arm_trajectory[1] = (
            monitor.planned_arm_trajectory[1][0],
            monitor.planned_arm_trajectory[1][1],
            monitor.trajectory_time,
            monitor.planned_arm_trajectory[1][3]
        )
        monitor.huskies[monitor.selected_robot_id].interface.send_dual_arm_cmd(monitor.planned_arm_trajectory)
    else:
        # fake execution in sim for both arms
        ho = monitor.huskies[monitor.selected_robot_id].object
        hi = monitor.huskies[monitor.selected_robot_id].interface
        
        # Get trajectories for both arms
        left_trajectory = monitor.planned_arm_trajectory[0]
        right_trajectory = monitor.planned_arm_trajectory[1]
        
        # Get attached objects for both arms
        left_obj = left_trajectory[3] if left_trajectory[3] is not None else None
        right_obj = right_trajectory[3] if right_trajectory[3] is not None else None
        
        left_gripper_tcp_from_object = left_obj.grasp if left_obj is not None else None
        right_gripper_tcp_from_object = right_obj.grasp if right_obj is not None else None
        
        # Execute both trajectories simultaneously
        max_points = max(len(left_trajectory[0]), len(right_trajectory[0]))
        
        for i in range(max_points):
            # Update left arm configuration
            if i < len(left_trajectory[0]):
                hi.arm_joint_pose[0] = left_trajectory[0][i]
            
            # Update right arm configuration  
            if i < len(right_trajectory[0]):
                hi.arm_joint_pose[1] = right_trajectory[0][i]
            
            # Update robot pose
            ho.set_pose((hi.position, hi.rotation), hi.arm_joint_pose)
            
            # Update attached objects based on FK
            if left_obj is not None and i < len(left_trajectory[0]):
                world_from_tcp = ho.get_link_pose_from_name("left_ur_arm_tool0")
                object_pose = pp.multiply(world_from_tcp, left_gripper_tcp_from_object)
                left_obj.set_pose(object_pose)
            
            if right_obj is not None and i < len(right_trajectory[0]):
                world_from_tcp = ho.get_link_pose_from_name("right_ur_arm_tool0")
                object_pose = pp.multiply(world_from_tcp, right_gripper_tcp_from_object)
                right_obj.set_pose(object_pose)
            
            # Set execution flags
            hi.is_arm_executing[0] = True
            hi.is_arm_executing[1] = True
            
            pp.wait_for_duration(0.01)
        
        # Clear execution flags
        hi.is_arm_executing[0] = False
        hi.is_arm_executing[1] = False

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
        # start and goal are collision-free (they are here -- IK + fallback
        # validated the goal), the coarse pass is a reasonable escape.
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


def _solve_bar_action_goal_ik(monitor, start_state,
                              ik_max_results: int = 20,
                              ik_max_descend_iterations: int = 200,
                              max_outer_attempts: int = 5,
                              random_seed: int = 0,
                              verbose: bool = False,
                              skip_env_collisions: bool = False,
                              alt_seed_conf12=None):
    """Solve goal IK for a BarAction movement from `target_ee_frames`.

    Returns a 12-vector (left_conf || right_conf) on success, or None on
    failure. Mirrors `core.robot_cell.solve_dual_arm_ik` in
    bar_joint_rhino_design_workflow: left then right, merging configs,
    using the cfab planner + state-defined ACM (held bar + tool touch-
    links). On failure, runs a `check_collision=False` retry so we can
    tell "unreachable" from "ACM/collision rejection".

    skip_env_collisions: when True, the compas_fab check_collision CC.3
    (link↔rigid-body), CC.4 (attached-rigid-body↔rigid-body), and CC.5
    (tool↔rigid-body) steps are bypassed during IK. Only CC.1 (robot
    self-collision) and CC.2 (robot↔tool) remain. Use when the env scene
    is irrelevant to the local replan.
    """
    from compas_fab.backends import CollisionCheckError, InverseKinematicsError
    from compas_fab.backends.pybullet.exceptions import PlanningGroupNotSupported
    from compas_fab.robots import FrameTarget, TargetMode

    # pybullet's whole-body IK can converge by recruiting a joint outside the
    # requested arm group (typically the other arm); compas_fab rejects that
    # with PlanningGroupNotSupported. Treat it as a soft IK failure (the arm
    # can't reach the target from this seed) instead of a hard crash.
    _IK_FAIL = (InverseKinematicsError, CollisionCheckError, PlanningGroupNotSupported)

    if monitor.target_ee_frames is None:
        return None

    np.random.seed(random_seed)

    planner = monitor.cfab.planner
    left_group = "base_left_arm_manipulator"
    right_group = "base_right_arm_manipulator"

    # Push state defensively so the planner's ACM/attachments match what
    # we're about to feed into IK (held bar -> gripper touch-links etc).
    try:
        planner.set_robot_cell_state(start_state)
    except Exception as e:
        print(f"[goal IK] set_robot_cell_state failed: {e}")
        return None

    target_L = FrameTarget(
        monitor.target_ee_frames["left"], target_mode=TargetMode.ROBOT,
        tolerance_position=0.001, tolerance_orientation=0.01,
    )
    target_R = FrameTarget(
        monitor.target_ee_frames["right"], target_mode=TargetMode.ROBOT,
        tolerance_position=0.001, tolerance_orientation=0.01,
    )

    def _ik_options(check_collision):
        opts = {
            "max_results": ik_max_results,
            "max_descend_iterations": ik_max_descend_iterations,
            "return_full_configuration": True,
            "check_collision": check_collision,
            "verbose": verbose,
        }
        if skip_env_collisions:
            # Skip env-related collision checks; keep robot self + robot↔tool.
            opts["_skip_cc3"] = True
            opts["_skip_cc4"] = True
            opts["_skip_cc5"] = True
        return opts

    def _solve_pair(check_collision: bool, seed_state=None):
        # * Solve each arm's IK with check_collision=False even when the
        # outer request wants collision-checked results. Reason: the LEFT
        # IK's per-call collision check is run against a state where the
        # RIGHT arm is still at its seed (usually HOME) but the bar --
        # attached to the LEFT tool0 -- has already moved to the LEFT
        # target. That intermediate state can put the bar into the
        # still-at-seed right arm, so the LEFT IK is rejected even though
        # the FINAL merged state (both arms at their targets, bar held
        # consistently between them) is collision-free. We defer the
        # collision check to the merged state below.
        per_ik_opts = _ik_options(False)
        seed = seed_state if seed_state is not None else start_state
        try:
            conf_L = planner.inverse_kinematics(target_L, seed, left_group, per_ik_opts)
        except _IK_FAIL as e:
            return None, f"LEFT FAIL: {getattr(e, 'message', e)}"
        st = seed.copy()
        st.robot_configuration = conf_L
        try:
            conf_LR = planner.inverse_kinematics(target_R, st, right_group, per_ik_opts)
        except _IK_FAIL as e:
            return None, f"RIGHT FAIL: {getattr(e, 'message', e)}"
        gs = seed.copy()
        gs.robot_configuration = conf_LR
        if check_collision:
            cc_opts = {"verbose": verbose}
            if skip_env_collisions:
                cc_opts["_skip_cc3"] = True
                cc_opts["_skip_cc4"] = True
                cc_opts["_skip_cc5"] = True
            try:
                planner.check_collision(gs, cc_opts)
            except CollisionCheckError as e:
                return None, f"GOAL COLLISION: {(e.message or '').splitlines()[0] if e.message else ''}"
        return gs, None

    # * Build a list of seed configurations to try across outer attempts.
    # Trac_ik's descent is deterministic in the neighbourhood of its seed,
    # so retrying with only different numpy state does not escape a
    # colliding local minimum. This list gives trac_ik a shot at each of:
    # the caller-supplied seed (usually the "live" pose, often HOME with
    # extended arms), the known-good dual-arm home, an optional user-
    # supplied `alt_seed_conf12` (typically the movement's own
    # start_conf -- a bar-holding pose that FK-produces the target frames
    # by construction), and small random perturbations of the caller seed.
    from husky_assembly_teleop.utils import (
        HUSKY_DUAL_UR5e_JOINT_NAMES as _JN,
        HUSKY_DUAL_ARM_HOME_CONF_12 as _HOME12,
    )
    _all_names = list(_JN[0]) + list(_JN[1])

    def _seed_with_conf12(base_state, conf12):
        s = base_state.copy()
        for n, v in zip(_all_names, conf12):
            s.robot_configuration[n] = float(v)
        return s

    np.random.seed(random_seed)
    caller_seed_12 = np.array(
        [float(start_state.robot_configuration[n]) for n in _all_names], dtype=float
    )
    seed_states = [start_state]                                  # 1. caller-supplied
    seed_states.append(_seed_with_conf12(start_state, _HOME12))  # 2. known-good home
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
        seed_states.append(                                     # 3. movement start
            _seed_with_conf12(start_state, np.asarray(alt_seed_conf12, dtype=float))
        )
    while len(seed_states) < max_outer_attempts:
        perturb = np.random.normal(0.0, 0.05, size=12)
        seed_states.append(_seed_with_conf12(start_state, perturb_center + perturb))

    goal_state = None
    last_err = None
    for attempt in range(1, max_outer_attempts + 1):
        gs, err = _solve_pair(check_collision=True,
                              seed_state=seed_states[attempt - 1])
        if gs is not None:
            goal_state = gs
            print(f"[goal IK] attempt {attempt}/{max_outer_attempts}: OK")
            break
        last_err = err
        print(f"[goal IK] attempt {attempt}/{max_outer_attempts}: {err}")

    if goal_state is None:
        # Diagnostic: try without collision check. If THIS succeeds, the
        # target is reachable and the failure was ACM/collision rejection
        # — usually a missing touch-link on the held bar or a stale ACM.
        gs_nc, err_nc = _solve_pair(check_collision=False)
        if gs_nc is not None:
            print(
                "[goal IK] DIAGNOSTIC: IK is reachable WITHOUT collision check "
                "but rejected WITH collision check. Last with-CC error: "
                f"{last_err}. Likely missing touch-link on the held bar or "
                "stale ACM. Inspect monitor.cfab.planner state, the bar's "
                "rigid_body_states[...].touch_links, and the start_state "
                "passed to IK."
            )
        else:
            print(
                f"[goal IK] DIAGNOSTIC: IK ALSO fails without collision check "
                f"({err_nc}); the EE targets are unreachable from the current "
                "base. Move the base closer to the goal-ghost base pose."
            )
        # * Fallback: when the caller provided an alt seed (typically the
        # movement's own start_conf, whose FK produces the target frames by
        # construction), accept it as the goal state. This trades a small
        # tool0 world-frame error (equal to the base offset applied on top
        # of the authored base) for a usable goal_conf so Button 2's
        # composite free plan can still plan a motion to a known bar-holding
        # pose. Only kicks in when trac_ik + collision-check couldn't find
        # a non-colliding branch on their own.
        if alt_seed_conf12 is not None:
            fallback_state = _seed_with_conf12(
                start_state, np.asarray(alt_seed_conf12, dtype=float),
            )
            try:
                cc_opts = {"verbose": verbose}
                if skip_env_collisions:
                    cc_opts["_skip_cc3"] = True
                    cc_opts["_skip_cc4"] = True
                    cc_opts["_skip_cc5"] = True
                planner.check_collision(fallback_state, cc_opts)
            except CollisionCheckError as e:
                print(
                    f"[goal IK] fallback (alt_seed_conf12 verbatim) also "
                    f"collides: {(e.message or '').splitlines()[0] if e.message else ''}. "
                    f"Giving up."
                )
                return None
            print(
                "[goal IK] FALLBACK: accepting alt_seed_conf12 verbatim as "
                "goal (tool0 world-frame error ~ base_offset). Composite free "
                "plan will land the arms on this bar-holding pose."
            )
            monitor.movement_goal_state = fallback_state
            return np.asarray(alt_seed_conf12, dtype=float)
        return None

    monitor.movement_goal_state = goal_state
    conf_LR = goal_state.robot_configuration
    left_names = HUSKY_DUAL_UR5e_JOINT_NAMES[0]
    right_names = HUSKY_DUAL_UR5e_JOINT_NAMES[1]
    return np.array([conf_LR[n] for n in list(left_names) + list(right_names)])


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

def execute_linear_cartesian_move(robot, hi, start_time, cartesian_trajectory, index):
    time_elapsed = time.time() - start_time
    
    if time_elapsed > cartesian_trajectory[2] + cartesian_trajectory[3] + PROBE_END_WAIT_TIME:
        return False
    
    world_from_arm_base = pp.get_link_pose(robot, pp.link_from_name(robot, 'left_ur_arm_base_link' if index == 0 else 'right_ur_arm_base_link'))
    
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

def switch_dual_arm_controller(monitor, to_ctrl):
    """Switch both arms from `from_ctrl` to `to_ctrl`; yield until ack."""
    hi: HuskyRobotInterface = monitor.huskies[monitor.selected_robot_id].interface
    hi.switch_controller(hi.active_controller[0], to_ctrl, 0)
    hi.switch_controller(hi.active_controller[1], to_ctrl, 1)
    while hi.active_controller[0] != to_ctrl or hi.active_controller[1] != to_ctrl:
        yield


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
    start_time = time.time()

    def _step():
        l = execute_linear_cartesian_move(robot, hi, start_time,
                                          cartesian_trajectories[0], 0)
        r = execute_linear_cartesian_move(robot, hi, start_time,
                                          cartesian_trajectories[1], 1)
        return l or r

    cont = should_continue if should_continue is not None else (lambda: True)
    while cont() and _step():
        if on_tick is not None:
            on_tick(hi, robot)
        yield


def _scaffolding_m2_stalled(hi, index):
    """v3 stall read: ScaffoldingToolStatus.state_m2 == 'STALLED'. None until first msg."""
    s = hi.scaffolding_status[index]
    return s is not None and s.state_m2 == 'STALLED'


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

    yield from switch_dual_arm_controller(
        monitor,
        'cartesian_compliance_controller',
    )

    # v3 screw motor: clear residual then TIGHTEN M2 on both arms.
    hi.send_scaffolding_cmd(0, 2, 0)
    hi.send_scaffolding_cmd(0, 2, 1)
    hi.send_scaffolding_cmd(1, 2, 0)
    hi.send_scaffolding_cmd(1, 2, 1)

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
            _scaffolding_m2_stalled(hi, 0) and _scaffolding_m2_stalled(hi, 1)),
    )

    # STOP M2 once insertion completes (by stall or by time budget).
    hi.send_scaffolding_cmd(0, 2, 0)
    hi.send_scaffolding_cmd(0, 2, 1)

    motor_stalled_left = _scaffolding_m2_stalled(hi, 0)
    motor_stalled_right = _scaffolding_m2_stalled(hi, 1)
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
        hi.send_scaffolding_cmd(0, 2, 0)
        hi.send_scaffolding_cmd(0, 2, 1)
        yield from switch_dual_arm_controller(
            monitor,
            'scaled_joint_trajectory_controller',
        )
        return

    # LOOSEN M2 during retreat.
    hi.send_scaffolding_cmd(-1, 2, 0)
    hi.send_scaffolding_cmd(-1, 2, 1)

    yield from execute_cartesian_linear_dual(
        monitor, retreat_trajectories_cartesian)

    # STOP M2 after retreat.
    hi.send_scaffolding_cmd(0, 2, 0)
    hi.send_scaffolding_cmd(0, 2, 1)

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

    yield from switch_dual_arm_controller(
        monitor,
        'scaled_joint_trajectory_controller',
    )


def execute_planned_trajectory_compliant(monitor):
    """Execute the loaded planned trajectory's endpoints as a single linear
    cartesian segment per arm under `cartesian_compliance_controller`. Only
    M2 / M3 movements are accepted (linear in TCP space).

    Safety contract: the live arms must already be at the planned start conf;
    otherwise `send_arm_cmd_cartesian` rejects targets (>5 cm from current TCP)
    and the motion will not run.
    """
    if monitor.current_movement is None:
        monitor.get_logger().warn(
            "No movement loaded; click 'Load Movement' first.")
        return
    role = monitor._match_movement_role(monitor.current_movement)
    if role not in ('M2', 'M3'):
        monitor.get_logger().warn(
            f"Compliant exec only supports M2/M3; current is {role!r}")
        return
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

    # M2 holds the end pose under compliance while the Joint (M2) motor
    # tightens against the scaffold; the loop must NOT terminate on
    # motion-time budget alone. Inflate t_wait so execute_linear_cartesian_move
    # keeps publishing the end pose, and exit only when both M2 motors report
    # STALLED. Large t_wait is a hard ceiling fallback if firmware never
    # reports stall.
    HOLD_FOR_STALL_TIMEOUT_S = 300.0
    if role == 'M2':
        t_wait = HOLD_FOR_STALL_TIMEOUT_S
        should_continue_fn = lambda: not (
            _scaffolding_m2_stalled(hi, 0) and _scaffolding_m2_stalled(hi, 1))
    else:
        t_wait = 0.0
        should_continue_fn = None

    cartesian_trajectories = [
        [L_start, L_end, t_total, t_wait],
        [R_start, R_end, t_total, t_wait],
    ]

    def _stop_all_both_arms():
        # mirror of 'L/R Stop All' buttons: STOP M1 and M2 on both arms.
        print('[scaffolding] L Stop All: M1 + M2 (arm 0)')
        hi.send_scaffolding_cmd(0, 1, 0)
        hi.send_scaffolding_cmd(0, 2, 0)
        print('[scaffolding] R Stop All: M1 + M2 (arm 1)')
        hi.send_scaffolding_cmd(0, 1, 1)
        hi.send_scaffolding_cmd(0, 2, 1)

    # ! important: DO NOT zero when the robot is holding the bar
    # the only good time to zero is when the robot is holding nothing but the tool

    # Pre-exec scaffolding tool commands:
    #   M2 -> Stop All + TIGHTEN Joint (M2) on both arms (bar tightened against scaffold).
    #   M3 -> Stop All + LOOSEN Gripper (M1) on both arms (release bar before retreat).
    _stop_all_both_arms()
    if role == 'M2':
        print('[scaffolding] M2: TIGHTEN Joint on L arm (arm 0)')
        hi.send_scaffolding_cmd(1, 2, 0)
        print('[scaffolding] M2: TIGHTEN Joint on R arm (arm 1)')
        hi.send_scaffolding_cmd(1, 2, 1)
    elif role == 'M3':
        print('[scaffolding] M3: LOOSEN Gripper on L arm (arm 0)')
        hi.send_scaffolding_cmd(-1, 1, 0)
        print('[scaffolding] M3: LOOSEN Gripper on R arm (arm 1)')
        hi.send_scaffolding_cmd(-1, 1, 1)

    try:
        # wait until the switch is completely (yield will go back to top level monitor to get updated state)
        yield from switch_dual_arm_controller(
            monitor,
            'cartesian_compliance_controller',
        )

        yield from execute_cartesian_linear_dual(
            monitor, cartesian_trajectories, should_continue=should_continue_fn)
    finally:
        # Always stop motors first, then restore the joint controller.
        _stop_all_both_arms()

        yield from switch_dual_arm_controller(
            monitor,
            'scaled_joint_trajectory_controller',
        )

    monitor.get_logger().info(
        f"Compliant exec done for {monitor.current_movement.movement_id} "
        f"(role {role})")


MOVE_TO_MOVEMENT_START_MAX_DELTA_RAD = np.pi / 3.0


def move_arms_to_movement_start(monitor):
    """Send a 2-waypoint joint trajectory (current -> target) on both arms,
    taking the live arms to `current_movement.start_state.robot_configuration`.

    Safety guard: refuses if either arm's per-joint max |delta| exceeds
    pi/3 rad. Protects against large unintended sweeps when the live arms
    are far from the planned start.
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
        target_left = np.array(
            [rc[n] for n in HUSKY_DUAL_UR5e_JOINT_NAMES[0]], dtype=float)
        target_right = np.array(
            [rc[n] for n in HUSKY_DUAL_UR5e_JOINT_NAMES[1]], dtype=float)
    except KeyError as e:
        monitor.get_logger().warn(
            f"start_state missing joint key {e}; cannot build target conf.")
        return

    hi: HuskyRobotInterface = monitor.huskies[monitor.selected_robot_id].interface
    current_left = np.asarray(hi.arm_joint_pose[0], dtype=float)
    current_right = np.asarray(hi.arm_joint_pose[1], dtype=float)

    delta_left = float(np.max(np.abs(target_left - current_left)))
    delta_right = float(np.max(np.abs(target_right - current_right)))
    limit = MOVE_TO_MOVEMENT_START_MAX_DELTA_RAD
    if delta_left > limit or delta_right > limit:
        monitor.get_logger().warn(
            f"Refusing move to {mv.movement_id!r} start: max |delta_q| "
            f"L={delta_left:.3f} R={delta_right:.3f} rad exceeds pi/3 "
            f"({limit:.3f} rad)."
        )
        return

    t_total = float(monitor.trajectory_time) or 5.0
    multi_arm_trajectory = [
        ([current_left, target_left], None, t_total, None),
        ([current_right, target_right], None, t_total, None),
    ]
    monitor.get_logger().info(
        f"Moving arms to {mv.movement_id!r} start "
        f"(max |delta_q| L={delta_left:.3f} R={delta_right:.3f} rad, "
        f"t={t_total:.1f}s)"
    )
    hi.send_dual_arm_cmd(multi_arm_trajectory)


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
