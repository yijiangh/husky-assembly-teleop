""" 
This module contains functions for planning the motion of the Husky robot.
"""

from typing import Tuple
import os
import numpy as np
import random
from scipy.spatial.transform import Rotation as R

from tracikpy import TracIKSolver

import pybullet as p
import pybullet_planning as pp

from husky_assembly_teleop.common import Husky, lerp, quat_lerp
from husky_assembly_teleop.base_planner import RRTStar, fill_yaw_angle
from husky_assembly_teleop.utils import plan_transit_motion, plan_transfer_motion, plan_retract_to_home_motion, TOOL0_FROM_GRIPPER_TCP, get_arm_ik_for_grasp_bar
from husky_assembly_teleop import DATA_DIRECTORY

IK_SOLVER = TracIKSolver(
    os.path.join(DATA_DIRECTORY,'husky_urdf/mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint.urdf'),
    'ur_arm_base_link',
    'ur_arm_tool0'
)

IK_SOLVER_DUAL = [TracIKSolver(
                    os.path.join(DATA_DIRECTORY,'husky_urdf/mt_husky_dual_ur5_e_moveit_config/urdf/husky_dual_ur5_e.urdf'),
                    'left_ur_arm_base_link',
                    'left_ur_arm_tool0',
                    solve_type='Speed'
                ), 
                TracIKSolver(
                    os.path.join(DATA_DIRECTORY,'husky_urdf/mt_husky_dual_ur5_e_moveit_config/urdf/husky_dual_ur5_e.urdf'),
                    'right_ur_arm_base_link',
                    'right_ur_arm_tool0',
                    solve_type='Speed'
                )]

def compute_grasp(theta_index, grasp_partition=4, longitudinal_offset=0.0):
    theta = (theta_index % grasp_partition) * (2*np.pi/grasp_partition)
    longitude_x = pp.Pose(euler=pp.Euler(pitch=np.pi/2))
    rotate_around_x_axis = pp.Pose(euler=pp.Euler(theta, 0, 0))
    rotate_around_z = pp.Pose(euler=[0, 0, np.pi/2])

    height = 1.0
    assert longitudinal_offset <= height/2, \
    'safety margin length must be smaller than half of the bounding cylinder\'s height {}'.format(height/2)
    # longitudinal_offset = random.uniform(-height/2+longitudinal_offset, height/2-longitudinal_offset)
    translate_along_x_axis = pp.Pose(point=pp.Point(longitudinal_offset,0,0))

    object_from_tool0 = pp.multiply(longitude_x, translate_along_x_axis, rotate_around_x_axis, rotate_around_z, pp.invert(TOOL0_FROM_GRIPPER_TCP))
    # pp.get_side_cylinder_grasps
    return pp.invert(object_from_tool0)

def arm_ik(husky: Husky, world_from_tool0, attachments, obstacles, hint_conf=None):
    return get_arm_ik_for_grasp_bar(husky.object.robot, IK_SOLVER, world_from_tool0, attachments, obstacles, hint_conf=hint_conf)

def plan_arm_motion(husky: Husky, arm_goal_pose, obstacles, traj_time, grasped_element=None, grasp=None, arm_index=0):
    attachments = [husky.object.ee_list[arm_index][1]]
    if grasped_element is not None and grasp is not None:
        robot = husky.object.robot
        attachments.append(pp.Attachment(robot, pp.link_from_name(robot, 'ur_arm_tool0'), grasp, grasped_element.body))

    trajectory = plan_transit_motion(
                husky.object.robot,
                arm_goal_pose,
                attachments,
                obstacles,
                debug=0,
                disabled_collisions=False,
                dual_arm_index=None if not husky.dual_arm else arm_index
            )

    if trajectory is None:
        return (None, None, None, None)

    planned_arm_trajectory = [np.array(p) for p in trajectory]

    if grasped_element is not None and grasp is not None:
        grasped_element.update_grasp(grasp)
        return (planned_arm_trajectory, None, traj_time, grasped_element)
    else:
        return (planned_arm_trajectory, None, traj_time, None)

def plan_arm_to_transfer_element(husky: Husky, transfer_element, obstacles, traj_time, grasp=None, arm_index=0):
    free_path, linear_path, grasp = plan_transfer_motion(
        husky.object.robot,
        IK_SOLVER, 
        transfer_element, 
        [husky.object.ee_list[arm_index][1]],
        obstacles, 
        grasp=grasp,
        debug=False, 
        disabled_collisions=None
        )

    if free_path is None or linear_path is None:
        return (None, None, None)

    planned_arm_trajectory = [np.array(p) for p in free_path + linear_path]
    transfer_element.update_grasp(grasp)

    fm_time = len(free_path) / len(planned_arm_trajectory)
    lm_time = len(linear_path) / len(planned_arm_trajectory)

    return (planned_arm_trajectory, None, traj_time, transfer_element), \
           (np.array(free_path), None, fm_time, transfer_element), \
           (np.array(linear_path), None, lm_time, transfer_element)

def plan_arm_to_retract_to_home(husky: Husky, transfer_element, obstacles, traj_time, arm_index=0):
    trajectory = plan_retract_to_home_motion(
        husky.object.robot,
        IK_SOLVER, 
        transfer_element.body, 
        [husky.object.ee_list[arm_index][1]],
        obstacles, 
        debug=False, 
        disabled_collisions=None
        )
    if trajectory is None:
        return (None, None, None, None)
    planned_arm_trajectory = [np.array(p) for p in trajectory]
    return (planned_arm_trajectory, None, traj_time, None)

def plan_base_motion(husky: Husky, goal_pose, obstacles):    
    x_range = (-3, 3)
    y_range = (-3, 3)
    
    ob_x_list = [np.inf] # what is this?
    ob_y_list = [np.inf]
    
    for o in obstacles:
        ob_x_list.append(o.pos[0])
        ob_y_list.append(o.pos[1])
    
    rrt_star = RRTStar(
                0.2, *x_range, *y_range, robot_size=0.1, avoid_dist=0.5
            )
    start_point, start_ori = pp.get_pose(husky.object.robot)
    start_pose_2d = (
        start_point[0],
        start_point[1],
        R.from_quat(start_ori).as_euler("zyx")[0],
    )
    goal_point, goal_ori = goal_pose
    goal_pose_2d = (
        goal_point[0],
        goal_point[1],
        R.from_quat(goal_ori).as_euler("zyx")[0],
    )
    x_list, y_list = rrt_star.plan(
                ob_x_list, ob_y_list, *(start_pose_2d[:2]), *(goal_pose_2d[:2])
            )
    yaw_list = fill_yaw_angle(start_pose_2d[-1], goal_pose_2d[-1], x_list, y_list)
    
    planned_base_trajectory_rrt = [
        (np.array((x, y, 0)), R.from_euler('z', yaw).as_quat()) for x, y, yaw in zip(x_list, y_list, yaw_list)
    ]
     
    # compute timestamps using max velocities   
    time_stamps = []
    t = 0
    for i in range(len(planned_base_trajectory_rrt)-1):
        pos_i, rot_i = planned_base_trajectory_rrt[i]
        pos_i_plus, rot_i_plus = planned_base_trajectory_rrt[i+1]
        
        dp = np.linalg.norm(pos_i_plus - pos_i)
        drz = np.abs((R.from_quat(rot_i).inv() * R.from_quat(rot_i_plus)).as_euler("zxy")[0])
        
        dt = max(dp / 0.5, drz / (0.05 * 2 * np.pi))
        time_stamps.append(t)
        t += dt
    time_stamps.append(t)
    
    # resample with 0.1s timestep
    planned_base_trajectory = []
    i = 0
    for t2 in np.arange(0, t, 0.1):
        while time_stamps[i] <= t2:
            i += 1
        
        dt_norm = (t2 - time_stamps[i-1]) / (time_stamps[i] - time_stamps[i-1])
        inter_pos = lerp(planned_base_trajectory_rrt[i-1][0], planned_base_trajectory_rrt[i][0], dt_norm)
        inter_rot = quat_lerp(planned_base_trajectory_rrt[i-1][1], planned_base_trajectory_rrt[i][1], dt_norm)
        planned_base_trajectory.append((inter_pos, inter_rot))
        
    return planned_base_trajectory, time_stamps[-1]

def plan_arc(husky: Husky, radius=1, angle=np.pi):
    """plans a circular arc trajectory"""
    hi = husky.interface
    
    start_pos = hi.position
    start_rot = R.from_quat(hi.rotation)
    
    N = 200
    arc_trajectory = [(np.array([np.sin(i/N * angle) * radius, np.cos(i/N * angle) * radius - radius, 0]), R.from_euler("z", -i/N * angle)) for i in range(N+1)]
    arc_trajectory = [(start_pos + start_rot.apply(pos), (start_rot * rot).as_quat()) for pos, rot in arc_trajectory]
        
    return arc_trajectory
    
def plan_corner(husky: Husky, distance1=1.0, angle=0.75 * np.pi, distance2=1.0):
    """plans a corner trajectory (straight, turn, straight)"""
    hi = husky.interface
    
    start_pos = hi.position
    start_rot = R.from_quat(hi.rotation)
    
    N = 200
    discrete_trajectory = (
        [(np.array([i/N * distance1, 0, 0]), R.identity()) for i in range(N+1)] +
        [(np.array([distance1, 0, 0]), R.from_euler("z", -i/N * angle)) for i in range(N+1)] + 
        [(np.array([distance1 + np.cos(angle) * i/N * distance2, -np.sin(angle) * i/N * distance2, 0]), R.from_euler("z", -angle)) for i in range(N+1)]
    )
    discrete_trajectory = [(start_pos + start_rot.apply(pos), (start_rot * rot).as_quat()) for pos, rot in discrete_trajectory]
        
    return discrete_trajectory

def plan_arm_wave(husky: Husky, trajectory_time):
    N = 20 # number of waypoints
    
    ts = list(np.linspace(0, trajectory_time, N))[0:]
    time_scaling = lambda t: t/trajectory_time*2*np.pi
    
    traj_pos = [np.array([0, -np.pi/2, -np.sin(time_scaling(t)), -np.pi/2 + np.sin(time_scaling(t)), 0, 0]) for t in ts]
    traj_vel = [1 / trajectory_time * 2*np.pi * np.array([0, 0, -np.cos(time_scaling(t)), np.cos(time_scaling(t)), 0, 0]) for t in ts]

    return traj_pos, traj_vel, trajectory_time, None

def dual_arm_bar_arc(start_pose, end_pose, trajectory_time):
    N = 20
    ts = list(np.linspace(0, trajectory_time, N))[0:]
    
    return [(np.array(start_pose[0]) + (np.array(end_pose[0]) - np.array(start_pose[0])) * t / trajectory_time, quat_lerp(start_pose[1], end_pose[1], t / trajectory_time)) for t in ts]

def plan_dual_arm_motion(husky: Husky, bar_trajectory, obstacles):
    N = 20
    trajectory_time = 5.0
    ts = list(np.linspace(0, trajectory_time, N))[0:]

    # generate bar trajectory (could be input)

    base_pose = pp.get_pose(husky.object.robot)

    translate_along_pos_axis = pp.Pose(point=pp.Point(0, 0.25,0))
    translate_along_neg_axis = pp.Pose(point=pp.Point(0, -0.25,0))

    # generate EE trajectories

    ee_trajectories = [[pp.multiply(p, translate_along_pos_axis) for p in bar_trajectory], 
                       [pp.multiply(p, translate_along_neg_axis) for p in bar_trajectory]]

    # solve

    def try_ik(qinit):
        arm_joint_trajectories = [([], None, trajectory_time, None), ([], None, trajectory_time, None)]

        for i in range(0,2):
            for (j, pose) in enumerate(ee_trajectories[i]):
                attachments = [husky.object.ee_list[i][1]]
                ik = get_arm_ik_for_grasp_bar(husky.object.robot, IK_SOLVER_DUAL[i], pose, attachments, obstacles, hint_conf=qinit)
                if ik is None:
                    print('Dual arm IK failed!')
                    return None
                # why does IK sometimes produce weird solutions which are quite far away from qinit?
                ik = np.mod(ik+np.pi-qinit, 2*np.pi)-np.pi+qinit
                if j > 0 and np.max(np.abs(ik-qinit)) > 0.25:
                    print("Dual arm IK failed because of discontinuity!")
                    return None
                arm_joint_trajectories[i][0].append(ik)
                qinit = ik
        
        return arm_joint_trajectories
        
    for j in range(0, 2):
        qinit = husky.interface.arm_joint_pose[j]
        arm_joint_trajectories = try_ik(qinit)
        if arm_joint_trajectories is not None:
            print(arm_joint_trajectories)
            return arm_joint_trajectories
        print('Dual arm IK retrying with new random init...')
        qinit = np.random.random((6))
    
    print('Dual arm IK does not find a solution!')
    return None