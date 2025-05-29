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

import pybullet_planning as pp

from husky_assembly_teleop import DATA_DIRECTORY
from husky_assembly_teleop.common import Husky, TrackedObject, AssemblyObject, HUSKY_UR5e_JOINT_NAMES
import husky_assembly_teleop.husky_planning as planning
import husky_assembly_teleop.husky_control as control
import husky_assembly_teleop.utils as utils
from husky_assembly_teleop.scaffolding import parse_mt_geometric, create_collision_bodies, create_couplers, flatten_list
import json
from datetime import datetime

MT_FILE_NAME = "one_tet_MT_contact.json"
# huskies = []
assembly_objects = []

DATA_DIR = "/home/yijiangh/ros2_ws/src/husky_assembly_teleop/data"
if not os.path.exists(DATA_DIR):
    DATA_DIR = "/home/yijiangh/ros2_ws/src/husky-asembly-teleop/data"

CALIB_DATA_DIR = os.path.join(DATA_DIR, "calibration_data")
BAR_HOLDING_ACC_DATA_DIR = os.path.join(DATA_DIR, "bar_holding_acc_data")

def init(monitor): 
    # * add robots
    # 1004
    Husky(monitor, name='/a200_0804', mocap_id=4568, pos=np.array((0,0,0)), 
          connect_arm=not monitor.FAKE_HARDWARE, connect_gripper=not monitor.FAKE_HARDWARE, 
        #   calibration=monitor.CALIBRATION)
          calibration=monitor.CALIBRATION,
          base_calibration_file=os.path.join(CALIB_DATA_DIR, 'calibrated_transformation_0804.json'))

    # Husky(monitor, name='/a200_0805', mocap_id=1033, pos=np.array((0,1,0)), connect_gripper=False)

    # * add static obstacles
    monitor.add_static_obstacles(pp.create_plane(color=(0.9, 0.9, 0.9, 1)))
    
    wall_right = pp.create_box(10, 0.4, 3)
    pp.set_color(wall_right, pp.GREY)
    pp.set_pose(wall_right, pp.Pose(pp.Point(0, 2.6, 0)))

    wall_left = pp.create_box(10, 0.4, 3)
    pp.set_pose(wall_left, pp.Pose(pp.Point(0, -3.0, 0)))
    pp.set_color(wall_left, pp.GREY)

    monitor.add_static_obstacles(pp.create_plane(color=(0.9, 0.9, 0.9, 1)))
    monitor.add_static_obstacles(wall_left)
    monitor.add_static_obstacles(wall_right)

    # * add tracked obstacles
    # TODO use one tracked box to indicate where to put the assembly
    if monitor.CALIBRATION:
        TrackedObject(monitor, 'calib_tool', 4569, np.zeros(3), np.array((0, 0, 0, 1)), 0.2)

    if monitor.BAR_HOLDING_ACCURACY_TEST:
        bar_rig = TrackedObject(monitor, 'bar_rig', 4570, np.zeros(3), np.array((0, 0, 0, 1)), 0.2)
        bar_rig.body = pp.create_cylinder(radius=0.01, height=1, color=(1, 0, 0, 0.2))
        bar_rig.model_base_pose = pp.Pose(euler=pp.Euler(roll=np.pi/2))

    #boxes.append(TrackedObject(monitor, 'box1', 4457, np.zeros(3), np.array((0, 0, 0, 1)), 0.2, 'cube.obj'))
    #boxes.append(TrackedObject(monitor, 'box2', 4484, np.zeros(3), np.array((0, 0, 0, 1)), 0.2, 'cube.obj'))
    #boxes.append(TrackedObject(monitor, 'box3', 1031, np.zeros(3), np.array((0, 0, 0, 1)), 0.2, 'cube.obj'))

    # * add assembly objects
    line_pt_pairs, contact_id_pairs, bar_radius = parse_mt_geometric(MT_FILE_NAME)
    line_pts_flattened = flatten_list(np.array(line_pt_pairs))
    radius_per_edge = [bar_radius] * int(len(line_pts_flattened)/2)

    # # compute the centroid of the line_pts_flattened
    # centroid = np.mean(line_pts_flattened, axis=0)
    # # move the line_pts_flattened to the origin
    # line_pts_flattened -= centroid
    # line_pts_flattened += [1.5,0,0.5]

    # TODO: set in rhino
    # line_pts_flattened += np.array([1.5, -0.5, 0.11])
    line_pts_flattened += np.array([-1.5, -0.5, 0.11])

    element_bodies = create_collision_bodies(line_pts_flattened, radius_per_edge, viewer=True)
    half_coupler_from_contact_pair = create_couplers(line_pts_flattened, contact_id_pairs)

    far_away_pose = pp.Pose(pp.Point(0,0,100))
    goal_poses = {}
    for i, e in enumerate(element_bodies):
        goal_poses[i] = pp.get_pose(e)

    # TODO use parsed sequence here
    assembly_objects.append([
        AssemblyObject(monitor, 'b{}'.format(i), body, far_away_pose, goal_poses[i]) for i, body in enumerate(element_bodies)
    ])

def update(monitor):
    pass

def plan_base_to_goal(monitor):
    base = planning.plan_base_motion(monitor.huskies[monitor.selected_robot_id], monitor.goal_pose, [])
    monitor.set_base_trajectry(base)

def plan_arm_wave(monitor):
    monitor.set_arm_trajectory(planning.plan_arm_wave(monitor.huskies[monitor.selected_robot_id], monitor.trajectory_time))

def plan_arm_to_goal(monitor):
    obstacles = [monitor.assembly_objects[i].body for i in range(monitor.current_seq_index)] + monitor.static_obstacles
    monitor.set_arm_trajectory(planning.plan_arm_motion(monitor.huskies[monitor.selected_robot_id], monitor.goal_arm_pose, obstacles, monitor.trajectory_time,
                                                        grasped_element=monitor.goal_element, grasp=monitor.goal_bar_grasp))

def plan_arm_to_transfer_element(monitor, grasp=None):
    obstacles = [monitor.assembly_objects[i].body for i in range(monitor.current_seq_index)] + monitor.static_obstacles
    transfer_element = monitor.assembly_objects[monitor.current_seq_index]
    full_traj, free_traj, linear_traj = planning.plan_arm_to_transfer_element(
        monitor.huskies[monitor.selected_robot_id], 
        transfer_element, 
        obstacles, 
        monitor.trajectory_time, 
        grasp=grasp
        )
    monitor.set_arm_trajectory(full_traj)
    monitor.free_arm_trajectory = free_traj
    monitor.linear_arm_trajectory = linear_traj

def plan_arm_to_retract_to_home(monitor):
    obstacles = [monitor.assembly_objects[i].body for i in range(monitor.current_seq_index)] + monitor.static_obstacles
    transfer_element = monitor.assembly_objects[monitor.current_seq_index]
    monitor.set_arm_trajectory(planning.plan_arm_to_retract_to_home(monitor.huskies[monitor.selected_robot_id], transfer_element, obstacles, monitor.trajectory_time))

def compute_ik_for_bar(monitor, world_from_bar, theta_index, grasp_dist):
    obstacles = monitor.static_obstacles
    # [monitor.assembly_objects[i].body for i in range(monitor.current_seq_index)] + 
    monitor.goal_element.set_pose(world_from_bar)

    # gripper_from_object
    grasp = planning.compute_grasp(theta_index, monitor.GRASP_PARTITION, grasp_dist)
    world_from_tool0 = pp.multiply(world_from_bar, pp.invert(grasp))

    # pp.draw_pose(world_from_bar)
    # pp.draw_pose(world_from_tool0)

    husky = monitor.huskies[monitor.selected_robot_id]
    robot = husky.object.robot
    attachments = [husky.object.ee_list[monitor.arm_index][1], pp.Attachment(robot, pp.link_from_name(robot, 'ur_arm_tool0'), grasp, monitor.goal_element.body)]

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
    obstacles = monitor.static_obstacles
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

def calibrate_button(monitor, tool_mocap_name):
    # record current joint conf and add to record
    h = monitor.huskies[monitor.selected_robot_id]
    hi = h.interface
    ho = h.object
    # fetch calibration mocap set frame
    flange_mocap_pose = None
    base_mocap_pose = None
    if monitor.USE_MOCAP:
        # need to get the raw data from mocap
        if h.name in monitor._mocap_rigidbody_cache:
            base_mocap_pose = monitor._mocap_rigidbody_cache[h.name]
        if tool_mocap_name in monitor._mocap_rigidbody_cache:
            flange_mocap_pose = monitor._mocap_rigidbody_cache[tool_mocap_name]
    else:
        base_mocap_pose = ho.get_link_pose_from_name("base_footprint")
        flange_mocap_pose = ho.get_link_pose_from_name("ur_arm_tool0")

    tool0_fk_pose = ho.get_link_pose_from_name("ur_arm_tool0")

    if flange_mocap_pose is None:
        if monitor.CALIBRATION:
            monitor.get_logger().warn(f'Mocap {tool_mocap_name} not found!')
            return
        else:
            pp.draw_pose(base_mocap_pose)
            monitor.append_calibration_data(
                {'joint_conf' : list(hi.arm_joint_pose[monitor.arm_index]), 
                 'base_mocap_pose' : [list(v) for v in base_mocap_pose],
                 "flange_mocap_pose" : [],
                 'tool0_fk_pose' : [list(v) for v in tool0_fk_pose],
                 'tool0_fk_from_mocap' : [],
                 })
    else:
        tool_0_fk_from_mocap = pp.multiply(pp.invert(tool0_fk_pose), flange_mocap_pose)
        pp.draw_pose(flange_mocap_pose)
        monitor.append_calibration_data(
            {'joint_conf' : list(hi.arm_joint_pose[monitor.arm_index]), 
             'base_mocap_pose' : [list(v) for v in base_mocap_pose],
             "flange_mocap_pose" : [list(v) for v in flange_mocap_pose],
             'tool0_fk_pose' : [list(v) for v in tool0_fk_pose],
             'tool0_fk_from_mocap' : [list(v) for v in tool_0_fk_from_mocap],
             })

def save_calibration(monitor, filename_suffix=""):
    print(monitor.calibration_data)
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
    filename = os.path.join(subfolder_path, f"calibration_{timestamp}_{filename_suffix}.json")

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
            {'joint_conf' : list(hi.arm_joint_pose[monitor.arm_index]), 
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
 
def calibrate_joint(monitor, joint_id, tool_mocap_name):
    global calibration_running, calibration_confirm
    hi = monitor.huskies[monitor.selected_robot_id].interface
    ho = monitor.huskies[monitor.selected_robot_id].object
    current_conf = hi.arm_joint_pose[monitor.arm_index]
    goal_conf = np.copy(monitor.goal_arm_pose)
    # check if values are close between current conf and goal conf, except for the joint id
    diff_vec = np.abs(np.array(current_conf) - np.array(goal_conf))
    diff_vec[joint_id] = 0
    if not np.all(diff_vec < 1e-4):
        monitor.get_logger().warn(f'Current conf and goal conf differs in axes other than the target joint {joint_id}: {diff_vec}!')
        return
   
    # linearly interpolate joint 0 from joint conf from -np.pi/2 to np.pi/2 different from the current joint 0
    joint_limit = pp.get_joint_limits(ho.robot, pp.joint_from_name(ho.robot, HUSKY_UR5e_JOINT_NAMES[joint_id]))

    steps = 20
    # interpolate between current conf and goal conf
    joint_confs = []
    for i in range(steps):
        joint_conf = np.array(current_conf) + (i+1)/steps * (np.array(goal_conf) - np.array(current_conf))
        joint_confs.append(joint_conf)

    monitor.set_arm_trajectory((joint_confs, None, monitor.trajectory_time, None))
    
def execute_arm_conf(monitor, conf):
    hi = monitor.huskies[monitor.selected_robot_id].interface
    monitor.huskies[monitor.selected_robot_id].interface.send_arm_cmd([hi.arm_joint_pose[monitor.arm_index], conf], 
                                                                      None, monitor.trajectory_time)

#################################

def execute_arm_trajectory(monitor, trajectory, index=0):
    if trajectory is None:
        monitor.get_logger().warn('Arm trajectory must be planed before executing!')
        return
    # trajectory confs, velocity, total time
    monitor.huskies[monitor.selected_robot_id].interface.send_arm_cmd(trajectory[index][0], trajectory[index][1], monitor.trajectory_time, index=index)

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

    obstacles = monitor.static_obstacles
    
    hi = monitor.huskies[monitor.selected_robot_id].interface
    ho = monitor.huskies[monitor.selected_robot_id].object

    # get ideal tool0 pose, not related to mocap obs
    # TODO this should be generalized to any world_from_tool0 and attachment
    transfer_element = trajectory[3]
    world_from_tool0 = pp.multiply(transfer_element.goal_pose, pp.invert(transfer_element.grasp))
    attachments = [ho.ee_list[monitor.arm_index][1], pp.Attachment(ho.robot, pp.link_from_name(ho.robot, 'ur_arm_tool0'), transfer_element.grasp, transfer_element.body)]

    for iter_i in range(num_iters):
        monitor.get_logger().info(f'Servoing arm trajectory {iter_i+1}/{num_iters}...')

        data[iter_i]['before_exe_footprint_pose'] = copy.copy(hi.position), copy.copy(hi.rotation)

        # execute the trajectory
        if iter_i != 0:
            traj_time = 2
        else:
            traj_time = trajectory[2] 

        monitor.huskies[monitor.selected_robot_id].interface.send_arm_cmd(trajectory[0], trajectory[1], traj_time)

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
        hi.arm_joint_pose[monitor.arm_index] = trajectory[0][-1]
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
    monitor.huskies[monitor.selected_robot_id].interface.send_gripper_cmd(0.6, 0.1)

def set_gripper(monitor):
    monitor.huskies[monitor.selected_robot_id].interface.send_gripper_cmd(monitor.goal_gripper, 0.1)