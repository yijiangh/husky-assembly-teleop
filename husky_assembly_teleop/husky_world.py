"""
This module contains the world definition and high level actions or sequences of actions for the huskies.
"""

import os, time
import asyncio.runners
import asyncio
from matplotlib.pyplot import bar
import numpy as np
import copy
import rclpy

import pybullet as p
import pybullet_planning as pp

from husky_assembly_teleop import DATA_DIRECTORY
from husky_assembly_teleop.common import Husky, TrackedObject, AssemblyObject
import husky_assembly_teleop.husky_planning as planning
import husky_assembly_teleop.husky_control as control
from husky_assembly_teleop.utils import HUSKY_DUAL_UR5e_JOINT_NAMES, UR5E_JOINT_NAMES, get_custom_limits, notify, plan_transit_motion
from husky_assembly_teleop.scaffolding import parse_mt_geometric, create_collision_bodies, create_couplers, flatten_list
import json
from datetime import datetime

from compas_fab.robots import RobotCellState

import matplotlib.pyplot as plt
import compas

MT_FILE_NAME = "one_tet_MT_contact.json"
# huskies = []
assembly_objects = []

# Use the centralized DATA_DIRECTORY from the package
DATA_DIR = DATA_DIRECTORY

CALIB_DATA_DIR = os.path.join(DATA_DIR, "calibration_data")
BAR_HOLDING_ACC_DATA_DIR = os.path.join(DATA_DIR, "bar_holding_acc_data")
DUAL_ARM_ACC_DATA_DIR = os.path.join(DATA_DIR, "dual_arm_acc_data")

def create_husky_with_end_effectors(monitor, name, mocap_id=None, pos=np.zeros(3), rot=np.array((0, 0, 0, 1)), 
                                   connect_arm=True, connect_gripper=True, base_calibration_file=None, 
                                   calibration=False, dual_arm=False, ee_types=None, force_regenerate=False):
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
                 - "victor_gripper": Victor gripper
                 - "robotiq_gripper": Robotiq gripper  
                 - "custom_gripper": Custom gripper (example)
                 - "validation_tool_pair": Validation tool pair (PointTool and BoardTool)
                 - "calib_tip": Calibration tip
                 For dual-arm robots, provide a list of two types.
                 For single-arm robots, provide a list of one type.
                 If None, defaults to victor_gripper or calib_tip based on calibration flag.
        force_regenerate: Force regeneration of URDF cache (only used for validation_tool_pair)
    """
    if ee_types is None:
        if calibration:
            ee_types = ["calib_tip"]
        else:
            ee_types = ["victor_gripper"]
     
    return Husky(monitor, name=name, mocap_id=mocap_id, pos=pos, rot=rot,
                connect_arm=connect_arm, connect_gripper=connect_gripper,
                base_calibration_file=base_calibration_file, calibration=calibration,
                dual_arm=dual_arm, ee_types=ee_types, force_regenerate=force_regenerate)

def init(monitor):
    # * add robots
    # 1004 - Example of creating a dual-arm robot with victor grippers
    create_husky_with_end_effectors(
        monitor, 
        name='/a200_0806', 
        mocap_id=4617, 
        pos=np.array((0,0,0)), 
        connect_arm=not monitor.FAKE_HARDWARE, 
        connect_gripper=False and not monitor.FAKE_HARDWARE, 
        calibration=monitor.CALIBRATION,
        dual_arm=True,
        # ee_types=["victor_gripper", "victor_gripper"],  # Mixed end effectors
        # ee_types=["validation_tool_pair"],  # Specify end effectors for both arms
        ee_types=["custom_gripper", "custom_gripper"],  # Specify end effector for single arm
        # base_calibration_file=os.path.join(CALIB_DATA_DIR, '20260126', 'calibrated_transformation_0806.json'),
        force_regenerate=False
    )
    
    # Example of creating a single-arm robot with robotiq gripper (commented out)
    # create_husky_with_end_effectors(
    #     monitor, 
    #     name='/a200_0804', 
    #     mocap_id=4615, 
    #     pos=np.array((0,0,0)), 
    #     connect_arm=not monitor.FAKE_HARDWARE, 
    #     connect_gripper=not monitor.FAKE_HARDWARE, 
    #     calibration=monitor.CALIBRATION,
    #     dual_arm=False,
    #     ee_types=["custom_gripper"],  # Specify end effector for single arm
    #     base_calibration_file=os.path.join(CALIB_DATA_DIR, 'calibrated_transformation_0804.json')
    # )

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
        ee_types=["custom_gripper", "victor_gripper"]  # Mixed end effectors
    )"""

    # * add static obstacles
    monitor.add_static_obstacles(pp.create_plane(color=(0.9, 0.9, 0.9, 1)), 'base_plane')
    
    wall_right = pp.create_box(10, 0.4, 3)
    pp.set_color(wall_right, pp.GREY)
    pp.set_pose(wall_right, pp.Pose(pp.Point(0, 2.6, 0)))

    wall_left = pp.create_box(10, 0.4, 3)
    pp.set_pose(wall_left, pp.Pose(pp.Point(0, -3.0, 0)))
    pp.set_color(wall_left, pp.GREY)

    monitor.add_static_obstacles(wall_left, 'wall_left')
    monitor.add_static_obstacles(wall_right, 'wall_right')

    # * add tracked obstacles
    # TODO use one tracked box to indicate where to put the assembly
    if monitor.CALIBRATION:
        left_tool_name = 'calib_tool_left'
        TrackedObject(monitor, left_tool_name, 4616, np.zeros(3), np.array((0, 0, 0, 1)), 0.2)
        monitor.assign_calibration_tool_to_robot(0, 0, left_tool_name)

        # right_tool_name = 'calib_tool_right'
        # TrackedObject(monitor, right_tool_name, 4573, np.zeros(3), np.array((0, 0, 0, 1)), 0.2)
        # monitor.assign_calibration_tool_to_robot(0, 1, right_tool_name)

    if monitor.BAR_HOLDING_ACCURACY_TEST:
        bar_rig = TrackedObject(monitor, 'bar_rig', 4570, np.zeros(3), np.array((0, 0, 0, 1)), 0.2)
        bar_rig.body = pp.create_cylinder(radius=0.01, height=1, color=(1, 0, 0, 0.2))
        bar_rig.model_base_pose = pp.Pose(euler=pp.Euler(roll=np.pi/2))
        
    if monitor.DUAL_ARM_ACCURACY_TEST:
        left_EE = TrackedObject(monitor, 'left_EE', 4572, np.zeros(3), np.array((0, 0, 0, 1)), 0.2)
        left_EE.body = pp.create_box(0.1, 0.1, 0.1)
        right_EE = TrackedObject(monitor, 'right_EE', 4573, np.zeros(3), np.array((0, 0, 0, 1)), 0.2)
        right_EE.body = pp.create_box(0.1, 0.1, 0.1)

    #boxes.append(TrackedObject(monitor, 'box1', 4457, np.zeros(3), np.array((0, 0, 0, 1)), 0.2, 'cube.obj'))
    #boxes.append(TrackedObject(monitor, 'box2', 4484, np.zeros(3), np.array((0, 0, 0, 1)), 0.2, 'cube.obj'))
    #boxes.append(TrackedObject(monitor, 'box3', 1031, np.zeros(3), np.array((0, 0, 0, 1)), 0.2, 'cube.obj'))

    # * add assembly objects
    if monitor.ASSEMBLY_MODE:
        line_pt_pairs, contact_id_pairs, bar_radius = parse_mt_geometric(MT_FILE_NAME)
        line_pts_flattened = flatten_list(np.array(line_pt_pairs))
        radius_per_edge = [bar_radius] * int(len(line_pts_flattened)/2)

        # TODO: set in rhino
        line_pts_flattened += np.array([-1.5, -0.5, 0.11])

        element_bodies = create_collision_bodies(line_pts_flattened, radius_per_edge, viewer=True)
        # TODO make coupler appear with the substructure
        half_coupler_from_contact_pair = create_couplers(line_pts_flattened, contact_id_pairs)

        far_away_pose = pp.Pose(pp.Point(0,0,100))
        goal_poses = {}
        for i, e in enumerate(element_bodies):
            goal_poses[i] = pp.get_pose(e)

        # TODO use parsed sequence here
        assembly_objects.append([
            AssemblyObject(monitor, 'b{}'.format(i), body, far_away_pose, goal_poses[i]) for i, body in enumerate(element_bodies)
        ])
    
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
    obstacles = [monitor.assembly_objects[i].body for i in range(monitor.current_seq_index)] + list(monitor.static_obstacles.values())
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

def plan_arm_to_transfer_element(monitor, grasp=None):
    obstacles = [monitor.assembly_objects[i].body for i in range(monitor.current_seq_index)] + list(monitor.static_obstacles.values())
    transfer_element = monitor.assembly_objects[monitor.current_seq_index]
    full_traj, free_traj, linear_traj = planning.plan_arm_to_transfer_element(
        monitor.huskies[monitor.selected_robot_id], 
        transfer_element, 
        obstacles, 
        monitor.trajectory_time, 
        grasp=grasp
        )
    monitor.set_arm_trajectory(full_traj, index=monitor.selected_arm_index)
    monitor.free_arm_trajectory = free_traj
    monitor.linear_arm_trajectory = linear_traj

def plan_arm_to_retract_to_home(monitor):
    obstacles = [monitor.assembly_objects[i].body for i in range(monitor.current_seq_index)] + list(monitor.static_obstacles.values())
    transfer_element = monitor.assembly_objects[monitor.current_seq_index]
    monitor.set_arm_trajectory(
        planning.plan_arm_to_retract_to_home(monitor.huskies[monitor.selected_robot_id], transfer_element, obstacles, monitor.trajectory_time), 
        index=monitor.selected_arm_index)

def compute_ik_for_bar(monitor, world_from_bar, theta_index, grasp_dist):
    obstacles = list(monitor.static_obstacles.values())
    # [monitor.assembly_objects[i].body for i in range(monitor.current_seq_index)] + 
    monitor.goal_element.set_pose(world_from_bar)

    # gripper_from_object
    grasp = planning.compute_grasp(theta_index, monitor.GRASP_PARTITION, grasp_dist)
    world_from_tool0 = pp.multiply(world_from_bar, pp.invert(grasp))

    # pp.draw_pose(world_from_bar)
    # pp.draw_pose(world_from_tool0)

    husky = monitor.huskies[monitor.selected_robot_id]
    robot = husky.object.robot
    attachments = [husky.object.ee_list[monitor.selected_arm_index][1], pp.Attachment(robot, pp.link_from_name(robot, 'ur_arm_tool0'), grasp, monitor.goal_element.body)]

    arm_conf = planning.arm_ik(monitor.huskies[monitor.selected_robot_id], world_from_tool0, attachments, obstacles)
    if arm_conf is None:
        monitor.get_logger().warn("IK failed!")
        return None, None
    
    return arm_conf, grasp

def randomize_bar_location_for_ik_and_transfer(monitor, bar_goal_axis=None, target_grasp_index=None):
    LOC_ATTEMPTS = 10
    MP_ATTEMPTS = 3
    TRAJ_MAX_LENGTH = 100

    BOUNDING_BOX_RANGE = [[0.2, 1.0], [-1.0,1.0], [0.3, 1.4]]
    AXIS_OPTIONS = [
        pp.quat_from_euler(np.array([0, np.pi/2, 0])), # global x axis
        pp.quat_from_euler(np.array([np.pi/2, 0, 0])), # global y axis
        pp.quat_from_euler(np.array([0, 0, 0])), # global z axis
    ]
    # default longitudinal axis of the bar is aligned with the z axis

    # disabled_collisions = disabled_collisions or {}
    robot = monitor.huskies[monitor.selected_robot_id].object.robot
    def check_body_robot_collision(target_body, target_link=pp.BASE_LINK):
        _, robot_links = pp.expand_links(robot)
        for rlink in robot_links:
            if pp.pairwise_link_collision(robot, rlink, target_body, target_link):
                return True

    world_from_base_link = monitor.goal_model.get_link_pose_from_name("base_footprint")
    obstacles = list(monitor.static_obstacles.values())
    if target_grasp_index is not None:
        candidate_grasps = [target_grasp_index]
    else:
        candidate_grasps = list(range(monitor.GRASP_PARTITION))

    # [monitor.assembly_objects[i].body for i in range(monitor.current_seq_index)] + 
    for i in range(LOC_ATTEMPTS):
        monitor.get_logger().info(f"Randomizing bar location {i+1}/{LOC_ATTEMPTS}...")

        # * randomize the bar location in the footprint frame of the robot, only keep its world orientation
        rand_pos = np.array([
            np.random.uniform(BOUNDING_BOX_RANGE[0][0], BOUNDING_BOX_RANGE[0][1]),
            np.random.uniform(BOUNDING_BOX_RANGE[1][0], BOUNDING_BOX_RANGE[1][1]),
            np.random.uniform(BOUNDING_BOX_RANGE[2][0], BOUNDING_BOX_RANGE[2][1])
        ])
        # rand_pos = pp.Point(0.8, 0, 1.3)

        # Randomize the bar quaternion to align with one of the global x, y, z axes
        if bar_goal_axis is None:
            bar_goal_quat = AXIS_OPTIONS[np.random.randint(0, len(AXIS_OPTIONS))]
        else:
            assert bar_goal_axis in range(len(AXIS_OPTIONS)), f"Invalid bar goal axis: {bar_goal_axis}"
            bar_goal_quat = AXIS_OPTIONS[bar_goal_axis]

        # Keep the world orientation from bar_goal_quat
        world_from_bar = pp.multiply(world_from_base_link, (rand_pos, bar_goal_quat))[0], bar_goal_quat
        # pp.draw_pose(world_from_bar)

        # check if bar is in collision with teh robot body, if so reject immediately
        # with pp.WorldSaver():
        pp.set_pose(monitor.goal_element.body, world_from_bar)
        if check_body_robot_collision(monitor.goal_element.body):
            monitor.get_logger().warn("Bar in collision with robot body, reject immediately!")
            continue

        # * enumerate grasp parameters
        for theta_index in candidate_grasps:
            monitor.get_logger().info(f"Grasping bar with id {theta_index+1}/{monitor.GRASP_PARTITION}...")

            grasp_dist = 0.0

            # Compute IK for this configuration
            arm_conf, grasp = compute_ik_for_bar(monitor, world_from_bar, theta_index, grasp_dist)

            if arm_conf is not None:
                # plan transit path
                for _ in range(MP_ATTEMPTS):
                    # * plan arm motion
                    traj = planning.plan_arm_motion(monitor.huskies[monitor.selected_robot_id], arm_conf, obstacles, monitor.trajectory_time,
                                                    grasped_element=monitor.goal_element, grasp=grasp)
                    if traj[0] is not None:
                        if len(traj[0]) < TRAJ_MAX_LENGTH:
                            traj[3].goal_pose = world_from_bar
                            traj[3].grasp = grasp
                            monitor.get_logger().info(f"Arm motion planning succeeded with {len(traj[0])} points!")
                            return traj, rand_pos, bar_goal_quat, theta_index, grasp_dist
                        else:
                            monitor.get_logger().warn(f"Arm motion planning trajectory too long {len(traj[0])}!")
                    else:
                        monitor.get_logger().warn("Arm motion planning failed!")
                else:
                    monitor.get_logger().warn(f"Motion planning failed after {MP_ATTEMPTS} attempts!")

    return None, None, None, None, None

def update_goal_gripper_model_pose(monitor, world_from_bar, theta_index, grasp_dist):
    tool0_from_object = planning.compute_grasp(theta_index, monitor.GRASP_PARTITION, grasp_dist)
    world_from_tool0 = pp.multiply(world_from_bar, pp.invert(tool0_from_object), pp.Pose(euler=pp.Euler(yaw=-np.pi/2)))
    # pp.draw_pose(world_from_tool0)
    # print("world_from_tool0", world_from_tool0)
    pp.set_pose(monitor.goal_gripper_model, world_from_tool0)
    # print('finished updating goal gripper model pose')

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
        if tool_mocap_name in monitor._mocap_rigidbody_cache:
            flange_mocap_pose = monitor._mocap_rigidbody_cache[tool_mocap_name]
    else:
        pass
        # base_mocap_pose = ho.get_link_pose_from_name("base_footprint")
        # flange_mocap_pose = ho.get_link_pose_from_name(tool0_link_name)

    tool0_fk_pose = ho.get_link_pose_from_name(tool0_link_name)

    # Visualization for debugging mocap poses
    DEBUG_MOCAP_POSES = True  # Toggle this to enable/disable mocap pose visualization

    if DEBUG_MOCAP_POSES:
        # Make all robot links transparent for easier visualization
        robot = ho.robot
        for link_id in range(pp.get_num_joints(robot)):
            pp.set_color(robot, [1, 1, 1, 0.2], link=link_id)  # Use RGBA where A<1 for transparency
        # Also set the base link transparent
        pp.set_color(robot, [1, 1, 1, 0.2], link=-1)

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

        # Draw the poses with annotations
        if base_mocap_pose is not None:
            pp.draw_pose(base_mocap_pose, length=0.15)
            pp.add_text("base_mocap", position=base_mocap_pose[0])

        if flange_mocap_pose is not None:
            pp.draw_pose(flange_mocap_pose, length=0.15)
            pp.add_text("flange_mocap (calib_tool)", position=flange_mocap_pose[0])

        pp.draw_pose(base_footprint_pose, length=0.15)
        pp.add_text("base_footprint_link", position=base_footprint_pose[0])

        pp.draw_pose(arm_base_link_pose, length=0.15)
        pp.add_text(f"{arm_base_link_name}", position=arm_base_link_pose[0])

        # Visualize all link poses between arm_base_link_inertia and tool0
        arm_link_names = [
            f"{arm_prefix}ur_arm_shoulder_link",
            f"{arm_prefix}ur_arm_upper_arm_link",
            f"{arm_prefix}ur_arm_forearm_link",
            f"{arm_prefix}ur_arm_wrist_1_link",
            f"{arm_prefix}ur_arm_wrist_2_link",
            f"{arm_prefix}ur_arm_wrist_3_link",
            f"{arm_prefix}ur_arm_tool0"
        ]

        for link_name in arm_link_names:
            try:
                link_pose = ho.get_link_pose_from_name(link_name)
                pp.draw_pose(link_pose, length=0.1)
                pp.add_text(link_name, position=[p + 0.015 for p in link_pose[0]])
            except:
                pass  # Skip if link doesn't exist

    if flange_mocap_pose is None:
        if monitor.CALIBRATION:
            monitor.get_logger().warn(f'Mocap {tool_mocap_name} not found!')
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
        pp.draw_pose(flange_mocap_pose)
        monitor.append_calibration_data({
                'robot_id' : int(monitor.selected_robot_id),
                'arm_index' : int(monitor.selected_arm_index),
                'joint_conf' : list(hi.arm_joint_pose[monitor.selected_arm_index]), 
                'base_mocap_pose' : [list(v) for v in base_mocap_pose],
                "flange_mocap_pose" : [list(v) for v in flange_mocap_pose],
                'tool0_fk_pose' : [list(v) for v in tool0_fk_pose],
                'tool0_fk_from_mocap' : [list(v) for v in tool_0_fk_from_mocap],
             })

def save_calibration(monitor, filename_suffix=""):
    # print(monitor.calibration_data)
    # save monitor.calibration_data to json, file name with time stamp
    # save to data/calibration_data
    timestamp = datetime.now().strftime("%Y%m%d_%H%M")
    # Create a date subfolder (format: YYYYMMDD)
    date_subfolder = datetime.now().strftime("%Y%m%d")
    subfolder_path = os.path.join(CALIB_DATA_DIR, date_subfolder)

    # Create the subfolder if it doesn't exist
    if not os.path.exists(subfolder_path):
        os.makedirs(subfolder_path)
        monitor.get_logger().info(f"Created subfolder: {subfolder_path}")

    # Save the file in the date subfolder
    if filename_suffix:
        filename = os.path.join(subfolder_path, f"calibration_{timestamp}_{filename_suffix}.json")
    else:
        filename = os.path.join(subfolder_path, f"calibration_{timestamp}.json")

    with open(filename, 'w') as f:
        json.dump({'raw_data' : monitor.calibration_data}, f, indent=4)

    monitor.get_logger().info(f"Calibration data saved to {filename}")

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

        bar_pose = monitor.get_world_from_bar_goal_pose()
        monitor.marker_set_data.append(
            {'joint_conf' : list(hi.arm_joint_pose[monitor.selected_arm_index]), 
             'base_mocap_pose' : [list(v) for v in base_mocap_pose],
             'footprint_base_link_pose' : base_link_pose,
             'world_from_bar_pose' : bar_pose,
             'bar_euler_angles' : list(pp.euler_from_quat(bar_pose[1])),
            # needs to make sure the marker set data is not pointing to the same object, so later new data will override the previously saved ones
             rb_mocap_name : copy.deepcopy(labeled_marker_data),
             'theta_index' : copy.copy(monitor.grasp_theta_index),
             'theta_partition': copy.copy(monitor.GRASP_PARTITION),
             })

def save_markerset_data(monitor, filename_suffix=""):
    print(monitor.calibration_data)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M")
    # Create a date subfolder (format: YYYYMMDD)
    date_subfolder = datetime.now().strftime("%Y%m%d")
    subfolder_path = os.path.join(BAR_HOLDING_ACC_DATA_DIR, date_subfolder+f'{filename_suffix}')

    # Create the subfolder if it doesn't exist
    if not os.path.exists(subfolder_path):
        os.makedirs(subfolder_path)
        monitor.get_logger().info(f"Created subfolder: {subfolder_path}")

    # Save the file in the date subfolder
    filename = os.path.join(subfolder_path, f"bar_holding_acc_{timestamp}.json")
    with open(filename, 'w') as f:
        json.dump({'raw_data' : monitor.marker_set_data}, f, indent=4)

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

def save_dual_arm_E_mocap(monitor, filename_suffix=""):
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
    with open(filename, 'w') as f:
        json.dump({'raw_data' : monitor.dual_arm_EE_mocap_data}, f, indent=4)

    monitor.get_logger().info(f"Dual arm acc data saved to {filename}")

def execute_and_log_mocap(monitor):
    global bar_pose, next_bar_pose
    bar_pose = next_bar_pose
    execute_arm_trajectory_both(monitor)
    while monitor.huskies[monitor.selected_robot_id].interface.is_arm_executing[0] or monitor.huskies[monitor.selected_robot_id].interface.is_arm_executing[1]:
        record_dual_arm_E_mocap(monitor)
        yield
    save_dual_arm_E_mocap(monitor)

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
    settle_time = 6
    hi = monitor.huskies[monitor.selected_robot_id].interface
    # last_conf = hi.arm_joint_pose[index]
    # print(transit_traj)
    # execute_arm_trajectory(monitor, transit_traj, index=index)

    total_num_confs = len(calib_traj[0])
    calibrate_button(monitor, monitor.active_calib_tool_name)
    monitor.get_logger().info(f'Saved calibration data {i}/{total_num_confs}.')

    # ! there seems to be a delay in arm conf, resulting in a one-step lag between the conf and the mocap data
    # TODO investigate
    for i, conf in enumerate(calib_traj[0]):
        monitor.get_logger().info(f'Executing arm conf {i+1}/{len(calib_traj[0])}...')
        # print('last conf:', last_conf, 'conf:', conf)
        hi.send_arm_cmd(
            # [hi.arm_joint_pose[monitor.selected_arm_index], conf], 
            [conf], 
            None, 
            time_between_confs,
            index=index
            )

        # wait until it finishes
        time.sleep(time_between_confs + settle_time)

        calibrate_button(monitor, monitor.active_calib_tool_name)
        monitor.get_logger().info(f'Saved calibration data {i}/{total_num_confs}.')

        # ! since the joint state is updated in the main thread and is blocked when running this function, 
        # we need to manually update the last conf here
        # Todo: change to Jakob's task system to avoid blocking the main thread
        hi.arm_joint_pose[monitor.selected_arm_index] = conf
        # last_conf = conf

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

def sample_dual_arm_configuration(monitor, tool0_to_tool0_transform, max_attempts=50, ik_attempts=10, attachments=None):
    """
    Sample a dual-arm configuration with the following steps:
    1. Sample a left arm configuration, reject if collision with static obstacles
    2. Get left arm tool0 pose in world coordinates
    3. Apply tool0_to_tool0 transform to get right arm tool0 pose
    4. Compute IK for right arm, reject if collision with left arm or static obstacles
    5. Plan transition paths for both arms, reject if path too long or no plan found
    6. If any step fails, restart from sampling left arm configuration
    
    Parameters:
    -----------
    monitor : HuskyMonitor
        The monitor instance containing robot and world state
    tool0_to_tool0_transform : pp.Pose
        Transformation from left arm tool0 to right arm tool0
    max_attempts : int
        Maximum number of attempts to find a valid configuration
    ik_attempts : int
        Maximum number of IK attempts for right arm with random initial guesses
    max_path_length : float
        Maximum allowed path length for transition trajectories
        
    Returns:
    --------
    tuple or None
        (left_arm_trajectory, right_arm_trajectory) if successful, None if failed
    """
    MAX_TRAJECTORY_POINTS = 180
    PLAN_SEPARATE_TRAJECTORIES = True

    husky = monitor.huskies[monitor.selected_robot_id]
    robot = husky.object.robot
    
    # Get joint names for both arms
    left_joint_names = HUSKY_DUAL_UR5e_JOINT_NAMES[0]
    right_joint_names = HUSKY_DUAL_UR5e_JOINT_NAMES[1]
    
    # Get joint indices
    left_joints = pp.joints_from_names(robot, left_joint_names)
    right_joints = pp.joints_from_names(robot, right_joint_names)
    
    # Get joint limits
    left_limits = [pp.get_joint_limits(robot, j) for j in left_joints]
    right_limits = [pp.get_joint_limits(robot, j) for j in right_joints]
    
    # Create sample functions
    left_sample_fn = pp.get_sample_fn(robot, left_joints)
    right_sample_fn = pp.get_sample_fn(robot, right_joints)
    
    # Create collision functions for both arms
    left_attachments = [attachments[0]] if attachments is not None else []
    right_attachments = [attachments[1], attachments[2]] if attachments is not None else []

    if attachments is not None:
        left_extra_disabled_collisions = [
        ((robot, pp.link_from_name(robot, 'left_ur_arm_wrist_3_link')), 
         (attachments[0].child, pp.BASE_LINK)), 
        ]
        right_extra_disabled_collisions = [
        ((robot, pp.link_from_name(robot, 'right_ur_arm_wrist_3_link')), 
         (attachments[1].child, pp.BASE_LINK)), 
        ]
    else:
        left_extra_disabled_collisions = []
        right_extra_disabled_collisions = []

    left_collision_fn = pp.get_collision_fn(robot, left_joints, obstacles=list(monitor.static_obstacles.values()),
                                              attachments=left_attachments, 
                                              self_collisions=True,
                                              disabled_collisions={}, 
                                              extra_disabled_collisions=left_extra_disabled_collisions,
                                              custom_limits={}, 
                                              max_distance=0)
    right_collision_fn = pp.get_collision_fn(robot, right_joints, obstacles=list(monitor.static_obstacles.values()),
                                              attachments=right_attachments, 
                                              self_collisions=True,
                                              disabled_collisions={}, 
                                              extra_disabled_collisions=right_extra_disabled_collisions,
                                              custom_limits={}, 
                                              max_distance=0)

    diagnose = False
    # save the current joint configuration here to use as starting conf for transit planning
    # start_left_conf = list(pp.get_joint_positions(robot, left_joints))
    # start_right_conf = list(pp.get_joint_positions(robot, right_joints))
    current_left_conf = np.copy(husky.interface.arm_joint_pose[0])
    current_right_conf = np.copy(husky.interface.arm_joint_pose[1])
    
    for attempt in range(max_attempts):
        if attempt % 10 == 0:
            print(f"Attempt {attempt}/{max_attempts}")
            
        # Step 1: Sample left arm configuration
        left_conf = left_sample_fn()
        
        # Check collision for left arm
        if left_collision_fn(left_conf, diagnosis=diagnose):
            continue
            
        # Step 2: Get left arm tool0 pose in world coordinates
        # Set left arm to sampled configuration
        pp.set_joint_positions(robot, left_joints, left_conf)
        
        # Get left arm tool0 pose
        left_tool0_pose = pp.get_link_pose(robot, pp.link_from_name(robot, 'left_ur_arm_tool0'))
        # pp.draw_pose(left_tool0_pose, length=0.2)
        # pp.wait_if_gui('left tool0 pose')
        
        # Step 3: Apply transform to get right arm tool0 pose
        right_tool0_pose = pp.multiply(left_tool0_pose, tool0_to_tool0_transform)
        # pp.draw_pose(right_tool0_pose, length=0.2)
        # pp.wait_if_gui('right tool0 pose')
        
        # Step 4: Compute IK for right arm
        right_conf = None
        for ik_attempt in range(ik_attempts):
            # Use random initial guess for IK
            if ik_attempt == 0:
                qinit = pp.get_joint_positions(robot, right_joints)
            else:
                qinit = right_sample_fn()
            
            # Use the IK solver from planning module
            from husky_assembly_teleop.husky_planning import IK_SOLVER_DUAL
            ik_solver = IK_SOLVER_DUAL[1]  # Right arm solver

            # Get right arm base pose
            right_arm_base_pose = pp.get_link_pose(robot, pp.link_from_name(robot, ik_solver.base_link))

            # Compute IK
            right_arm_base_from_tool0 = pp.multiply(pp.invert(right_arm_base_pose), right_tool0_pose)

            conf = ik_solver.ik(pp.tform_from_pose(right_arm_base_from_tool0), qinit=qinit)
            
            if conf is not None:
                # Check collision for right arm with static obstacles
                if not right_collision_fn(conf, diagnosis=diagnose):
                    right_conf = conf
                    print(f"Found valid right arm configuration on IK attempt {ik_attempt + 1}")
                    # pp.wait_if_gui('right arm conf')

                    break
        
        if right_conf is None:
            continue
            
        if PLAN_SEPARATE_TRAJECTORIES:
            # Step 5: Plan transition paths for both arms
            pp.set_joint_positions(robot, left_joints, current_left_conf)
        
            # Plan left arm transition
            left_trajectory = planning.plan_arm_motion(
                husky, left_conf, list(monitor.static_obstacles.values()), monitor.trajectory_time, arm_index=0
            )
            if left_trajectory[0] is None:
                continue
        
            # Plan right arm transition
            pp.set_joint_positions(robot, left_joints, left_trajectory[0][-1])
            pp.set_joint_positions(robot, right_joints, current_right_conf)
            right_trajectory = planning.plan_arm_motion(
                husky, right_conf, list(monitor.static_obstacles.values()), monitor.trajectory_time, arm_index=1
            )
        else:
            # Plan in the composite space of both arms
            # Set both arms to their current configurations
            pp.set_joint_positions(robot, left_joints, current_left_conf)
            pp.set_joint_positions(robot, right_joints, current_right_conf)

            # Concatenate the left and right arm goal configurations
            composite_goal = np.concatenate([left_conf, right_conf])

            # Plan a path in the composite space
            from husky_assembly_teleop.husky_planning import plan_transit_motion
            composite_start = np.concatenate([current_left_conf, current_right_conf])
            composite_path = plan_transit_motion(
                robot,
                composite_goal,
                attachments,
                list(monitor.static_obstacles.values()),
                debug=False,
                disabled_collisions=None,
                dual_arm_index="both",
                # collision_fn=composite_collision_fn
            )

            if composite_path is None:
                continue

            # Split the composite path into left and right arm trajectories
            left_trajectory = (np.array([q[:len(left_joints)] for q in composite_path]), None, monitor.trajectory_time, None)
            right_trajectory = (np.array([q[len(left_joints):] for q in composite_path]), None, monitor.trajectory_time, None)
        
        if right_trajectory[0] is None:
            continue
            
        # Check path length
        # Use the number of trajectory points as the path length
        left_path_length = len(left_trajectory[0])
        right_path_length = len(right_trajectory[0])
        if left_path_length > MAX_TRAJECTORY_POINTS or right_path_length > MAX_TRAJECTORY_POINTS:
            continue
            
        # Success! Return the trajectories
        return left_trajectory, right_trajectory
    
    # If we get here, no valid configuration was found
    print(f"Failed to find valid dual-arm configuration after {max_attempts} attempts")
    return None

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

def plan_both_arms_to_goal(monitor, use_composite=False, debug=False):
    """
    Plan motions for both arms from current to goal joint configurations.
    If use_composite is False, plan left then right sequentially.
    If True, plan in the composite joint space.
    Sets the resulting trajectories in the monitor.
    """
    husky = monitor.huskies[monitor.selected_robot_id]
    robot = husky.object.robot
    
    left_joint_names = HUSKY_DUAL_UR5e_JOINT_NAMES[0]
    right_joint_names = HUSKY_DUAL_UR5e_JOINT_NAMES[1]
    left_joints = pp.joints_from_names(robot, left_joint_names)
    right_joints = pp.joints_from_names(robot, right_joint_names)
    
    current_left_conf = pp.get_joint_positions(robot, left_joints)
    current_right_conf = pp.get_joint_positions(robot, right_joints)
    left_conf = np.array(monitor.goal_arm_pose[0])
    right_conf = np.array(monitor.goal_arm_pose[1])
    # Print current joint configuration for both arms
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

    print(f"target left_conf: {left_conf}")
    print(f"target right_conf: {right_conf}")
    attachments = [ee[1] for ee in husky.object.ee_list]

    left_trajectory = None
    right_trajectory = None

    if not use_composite:
        # Sequential planning: left arm, then right arm
        pp.set_joint_positions(robot, left_joints, current_left_conf)
        left_trajectory = planning.plan_arm_motion(
            husky, left_conf, list(monitor.static_obstacles.values()), monitor.trajectory_time, arm_index=0, debug=debug
        )
        if left_trajectory[0] is None:
            monitor.get_logger().warn('Left arm planning failed!')
            return
        # Set left arm to end conf, right arm to current
        pp.set_joint_positions(robot, left_joints, left_trajectory[0][-1])
        pp.set_joint_positions(robot, right_joints, current_right_conf)
        right_trajectory = planning.plan_arm_motion(
            husky, right_conf, list(monitor.static_obstacles.values()), monitor.trajectory_time, arm_index=1, debug=debug
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
        # Composite planning: plan in the joint space of both arms
        pp.set_joint_positions(robot, left_joints, current_left_conf)
        pp.set_joint_positions(robot, right_joints, current_right_conf)
        composite_goal = np.concatenate([left_conf, right_conf])
        from husky_assembly_teleop.husky_planning import plan_transit_motion
        composite_start = np.concatenate([current_left_conf, current_right_conf])
        composite_path = plan_transit_motion(
            robot,
            composite_goal,
            attachments,
            list(monitor.static_obstacles.values()),
            debug=debug,
            disabled_collisions=None,
            dual_arm_index="both",
        )
        if composite_path is None:
            monitor.get_logger().warn('Composite planning failed!')
            return
        left_trajectory = (np.array([q[:len(left_joints)] for q in composite_path]), None, monitor.trajectory_time, None)
        right_trajectory = (np.array([q[len(left_joints):] for q in composite_path]), None, monitor.trajectory_time, None)

    # Set the trajectories for both arms
    monitor.set_arm_trajectory(left_trajectory, index=0)
    monitor.set_arm_trajectory(right_trajectory, index=1)
    monitor.set_to_show_traj_state()
    print("Successfully planned both arms to goal ({} mode)!".format('composite' if use_composite else 'sequential'))

