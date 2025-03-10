"""
This module contains the world definition and high level actions or sequences of actions for the huskies.
"""

import os
import asyncio.runners
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
    Husky(monitor, name='/a200_0804', mocap_id=1004, pos=np.array((0,0,0)), 
          connect_arm=not monitor.FAKE_HARDWARE, connect_gripper=not monitor.FAKE_HARDWARE)
    # Husky(monitor, name='/a200_0805', mocap_id=1033, pos=np.array((0,1,0)), connect_gripper=False)

    # * add static obstacles
    monitor.add_static_obstacles(pp.create_plane(color=(0.9, 0.9, 0.9, 1)))

    # * add tracked obstacles
    # TODO use one tracked box to indicate where to put the assembly
    TrackedObject(monitor, 'calib_tool', 4497, np.zeros(3), np.array((0, 0, 0, 1)), 0.2)

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
    line_pts_flattened += np.array([2.8, -0.5, 0.1])

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

def plan_arm_to_transfer_element(monitor):
    obstacles = [monitor.assembly_objects[i].body for i in range(monitor.current_seq_index)] + monitor.static_obstacles
    transfer_element = monitor.assembly_objects[monitor.current_seq_index]
    monitor.set_arm_trajectory(planning.plan_arm_to_transfer_element(monitor.huskies[monitor.selected_robot_id], transfer_element, obstacles, monitor.trajectory_time))

def plan_arm_to_retract_to_home(monitor):
    obstacles = [monitor.assembly_objects[i].body for i in range(monitor.current_seq_index)] + monitor.static_obstacles
    transfer_element = monitor.assembly_objects[monitor.current_seq_index]
    monitor.set_arm_trajectory(planning.plan_arm_to_retract_to_home(monitor.huskies[monitor.selected_robot_id], transfer_element, obstacles, monitor.trajectory_time))

# calibration_running = False
# calibration_confirm = False
def calibrate_button(monitor, tool_mocap_name):
    # record current joint conf and add to record
    hi = monitor.huskies[monitor.selected_robot_id].interface
    ho = monitor.huskies[monitor.selected_robot_id].object
    # fetch calibration mocap set frame
    mocap_pose = None
    if monitor.USE_MOCAP:
        for i, o in enumerate(monitor.tracked_objects):
            if o.name == tool_mocap_name:
                mocap_pose = (o.pos, o.rot)
    else:
        mocap_pose = ho.get_link_pose_from_name("ur_arm_tool0")

    if mocap_pose is None:
        monitor.get_logger().warn(f'Mocap {tool_mocap_name} not found!')
        return
    pp.draw_pose(mocap_pose)
    monitor.append_calibration_data({'joint_conf' : list(hi.arm_joint_pose), "mocap_pose" : [list(v) for v in mocap_pose]})

def save_calibration(monitor):
    print(monitor.calibration_data)
    # save monitor.calibration_data to json, file name with time stamp
    # save to data/calibration_data
    timestamp = datetime.now().strftime("%Y%m%d_%H%M")
    filename = os.path.join(CALIB_DATA_DIR, f"calibration_{timestamp}.json")

    with open(filename, 'w') as f:
        json.dump(monitor.calibration_data, f, indent=4)

    monitor.get_logger().info(f"Calibration data saved to {filename}")
 
def task_calibrate(monitor):
    global calibration_running, calibration_confirm
    hi = monitor.huskies[monitor.selected_robot_id].interface
    ho = monitor.huskies[monitor.selected_robot_id].object
    # to get goal_ee_pose as husky[0] pose pp.multiply((hi.position, hi.rotation), pp.invert(monitor.goal_pose), monitor.goal_model.get_ee_pose())
    current_conf = hi.arm_joint_pose
   
    # linearly interpolate joint 0 from joint conf from -np.pi/2 to np.pi/2 different from the current joint 0
    joint_0_limit = pp.get_joint_limits(ho.robot, pp.joint_from_name(ho.robot, HUSKY_UR5e_JOINT_NAMES[0]))
    joint_1_limit = pp.get_joint_limits(ho.robot, pp.joint_from_name(ho.robot, HUSKY_UR5e_JOINT_NAMES[1]))

    DELTA = np.pi/3
    steps = 10
    joint_confs = []
    for i in range(steps):
        joint_0_value = current_conf[0] + (i+1) * DELTA/steps
        if joint_0_value < joint_0_limit[1]:
            new_conf = np.copy(current_conf)
            new_conf[0] = joint_0_value
            joint_confs.append(new_conf)

    print(joint_confs)

    # draw_list = []
    for conf in joint_confs:
        # pp.remove_handles(draw_list)
        # draw_list = pp.draw_pose(pose)
        while True:
            # print('is arm executing', hi.is_arm_executing)
            if hi.is_arm_executing:
                break

            # print('caliberation confirm is', calibration_confirm)
            if calibration_confirm:
                calibration_confirm = False
                
                # arm_joint_pose = planning.arm_ik(monitor.huskies[monitor.selected_robot_id], pose)
                # if arm_joint_pose is None:
                #     monitor.get_logger().warn('Ik for calibration failed!')
                #     monitor.set_arm_trajectory((None, None, 2, None))
                # else:

                monitor.set_arm_trajectory(([hi.arm_joint_pose, conf], None, 5, None))

            yield
        
        while hi.is_arm_executing:
            yield # wait for execution to finish
            
    # pp.remove_handles(draw_list)
    
    monitor.get_logger().info('Calibration squence finished!')
    calibration_running = False
    
def execute_arm_trajectory(monitor):
    if monitor.planned_arm_trajectory[0] is None:
        monitor.get_logger().warn('Arm trajectory must be planed before executing!')
        return
    monitor.huskies[monitor.selected_robot_id].interface.send_arm_cmd(monitor.planned_arm_trajectory[0], monitor.planned_arm_trajectory[1], monitor.trajectory_time)
     
def move_base_to_goal(monitor):
    if monitor.planned_base_trajectory[0] is None:
        monitor.get_logger().warn('Base trajectory must be planed before executing!')
        return
    monitor.tasks.append(control.execute_base_trajectory(monitor, monitor.huskies[0], monitor.planned_base_trajectory))
    
def open_gripper_full(monitor):
    monitor.huskies[monitor.selected_robot_id].interface.send_gripper_cmd(0.0, 0.1)

def close_gripper_for_bar(monitor):
    monitor.huskies[monitor.selected_robot_id].interface.send_gripper_cmd(0.6, 0.1)

def set_gripper(monitor):
    monitor.huskies[monitor.selected_robot_id].interface.send_gripper_cmd(monitor.goal_gripper, 0.1)