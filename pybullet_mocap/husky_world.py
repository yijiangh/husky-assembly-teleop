"""
This module contains the world definition and high level actions or sequences of actions for the huskies.
"""

import os
import asyncio.runners
import asyncio
import numpy as np

import pybullet_planning as pp

from pybullet_mocap import DATA_DIRECTORY
from pybullet_mocap.common import Husky, TrackedObject, AssemblyObject, HUSKY_UR5e_JOINT_NAMES
import pybullet_mocap.husky_planning as planning
import pybullet_mocap.husky_control as control
from pybullet_mocap.scaffolding import parse_mt_geometric, create_collision_bodies, create_couplers, flatten_list
import json
from datetime import datetime

MT_FILE_NAME = "one_tet_MT_contact.json"
# huskies = []
assembly_objects = []

# CALIB_DATA_DIR = "/home/yijiangh/ros2_ws/src/pybullet_mocap/data/calibration_data"
CALIB_DATA_DIR = "/home/yijiangh/ros2_ws/src/husky-asembly-teleop/data/calibration_data"

def init(monitor): 
    # * add robots
    # 1004
    Husky(monitor, name='/a200_0804', mocap_id=4568, pos=np.array((0,0,0)), 
          connect_arm=not monitor.FAKE_HARDWARE, connect_gripper=not monitor.FAKE_HARDWARE, 
          calibration=monitor.CALIBRATION)
        #   calibration=monitor.CALIBRATION,
        #   base_calibration_file=os.path.join(CALIB_DATA_DIR, 'calibrated_transformation_0804.json'))

    # Husky(monitor, name='/a200_0805', mocap_id=1033, pos=np.array((0,1,0)), connect_gripper=False)

    # * add static obstacles
    monitor.add_static_obstacles(pp.create_plane(color=(0.9, 0.9, 0.9, 1)))

    # * add tracked obstacles
    # TODO use one tracked box to indicate where to put the assembly
    if monitor.CALIBRATION:
        TrackedObject(monitor, 'calib_tool', 4569, np.zeros(3), np.array((0, 0, 0, 1)), 0.2)

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
    line_pts_flattened += np.array([1, -0.5, 0.11])

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
    monitor.set_arm_trajectory(planning.plan_arm_motion(monitor.huskies[monitor.selected_robot_id], monitor.goal_arm_pose, obstacles, monitor.trajectory_time))

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

# calibration_running = False
# calibration_confirm = False
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
    tool_0_fk_from_mocap = pp.multiply(pp.invert(tool0_fk_pose), flange_mocap_pose)

    if flange_mocap_pose is None:
        monitor.get_logger().warn(f'Mocap {tool_mocap_name} not found!')
        return
    pp.draw_pose(flange_mocap_pose)
    monitor.append_calibration_data(
        {'joint_conf' : list(hi.arm_joint_pose), 
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
    filename = os.path.join(CALIB_DATA_DIR, f"calibration_{timestamp}_{filename_suffix}.json")

    with open(filename, 'w') as f:
        json.dump({'raw_data' : monitor.calibration_data}, f, indent=4)

    monitor.get_logger().info(f"Calibration data saved to {filename}")
 
def calibrate_joint(monitor, joint_id, tool_mocap_name):
    global calibration_running, calibration_confirm
    hi = monitor.huskies[monitor.selected_robot_id].interface
    ho = monitor.huskies[monitor.selected_robot_id].object
    current_conf = hi.arm_joint_pose
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
        # monitor.goal_arm_pose(joint_conf)
        # monitor.huskies[monitor.selected_robot_id].interface.send_arm_cmd([hi.arm_joint_pose, joint_conf], None, monitor.trajectory_time)

        # print(f'Executing: {hi.is_arm_executing}')
        # execute the trajectory and wait for extra 2 secs after its done
        # if hi.is_arm_executing:
        #     while hi.is_arm_executing:
        #         pass
        #     else:
        #         pp.wait_for_duration(1)
        # else:
        # pp.wait_for_duration(monitor.trajectory_time + 1)
        # await asyncio.sleep(monitor.trajectory_time + 1)

        # pp.wait_for_user('The tool has stabilize?')
        # calibrate_button(monitor, tool_mocap_name)
    # save_calibration(monitor, f"joint_{joint_id}")

    monitor.set_arm_trajectory((joint_confs, None, monitor.trajectory_time, None))
    
def execute_arm_conf(monitor, conf):
    hi = monitor.huskies[monitor.selected_robot_id].interface
    monitor.huskies[monitor.selected_robot_id].interface.send_arm_cmd([hi.arm_joint_pose, conf], 
                                                                      None, monitor.trajectory_time)

def execute_arm_trajectory(monitor, trajectory):
    if trajectory is None:
        monitor.get_logger().warn('Arm trajectory must be planed before executing!')
        return
    # trajectory confs, velocity, total time
    monitor.huskies[monitor.selected_robot_id].interface.send_arm_cmd(trajectory[0], trajectory[1], monitor.trajectory_time)
     
def move_base_to_goal(monitor):
    if monitor.planned_base_trajectory[0] is None:
        monitor.get_logger().warn('Base trajectory must be planed before executing!')
        return
    monitor.tasks.append(control.execute_base_trajectory(monitor, monitor.huskies[0], monitor.planned_base_trajectory))
    
def open_gripper_full(monitor):
    monitor.huskies[monitor.selected_robot_id].interface.send_gripper_cmd(0.426, 0.1)

def close_gripper_for_bar(monitor):
    monitor.huskies[monitor.selected_robot_id].interface.send_gripper_cmd(0.6, 0.1)

def set_gripper(monitor):
    monitor.huskies[monitor.selected_robot_id].interface.send_gripper_cmd(monitor.goal_gripper, 0.1)