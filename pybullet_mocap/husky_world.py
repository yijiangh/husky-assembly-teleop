"""
This module contains the world definition and high level actions or sequences of actions for the huskies.
"""

import asyncio
import asyncio.runners
import numpy as np

import pybullet_planning as pp

from pybullet_mocap.common import Husky, TrackedObject
import pybullet_mocap.husky_planning as planning
import pybullet_mocap.husky_control as control


boxes = []
huskies = []

def init(monitor):
    #boxes.append(TrackedObject(monitor, 'box1', 4457, np.zeros(3), np.array((0, 0, 0, 1)), 0.2, 'cube.obj'))
    #boxes.append(TrackedObject(monitor, 'box2', 4484, np.zeros(3), np.array((0, 0, 0, 1)), 0.2, 'cube.obj'))
    #boxes.append(TrackedObject(monitor, 'box3', 1031, np.zeros(3), np.array((0, 0, 0, 1)), 0.2, 'cube.obj'))
    
    huskies.append(Husky(monitor, name='/a200_0804', mocap_id=1004, pos=np.array((0,0,0)), connect_gripper=True))
    huskies.append(Husky(monitor, name='/a200_0805', mocap_id=1033, pos=np.array((0,1,0)), connect_gripper=False))

def update(monitor):
    pass

def plan_to_goal(monitor):
    base = planning.plan_base_motion(huskies[0], monitor.goal_pose, boxes)
    monitor.set_base_trajectry(base)

def plan_arm_wave(monitor):
    monitor.set_arm_trajectory(planning.plan_arm_wave(huskies[0]))

def plan_arm_to_goal(monitor):
    hi = huskies[0].interface
    monitor.set_arm_trajectory(([hi.arm_joint_pose, monitor.goal_arm_pose], None, 10))
    # monitor.set_arm_trajectory(planning.plan_arm_motion(huskies[0], monitor.goal_arm_pose, boxes))

calibration_running = False
calibration_confirm = False
def calibrate_button(monitor):
    global calibration_running, calibration_confirm
    if not calibration_running:
        calibration_running = True
        calibration_confirm = False
        monitor.tasks.append(task_calibrate(monitor))
    else:
        calibration_confirm = True

def task_calibrate(monitor):
    global calibration_running, calibration_confirm
    hi = huskies[0].interface
    # to get goal_ee_pose as husky[0] pose pp.multiply((hi.position, hi.rotation), pp.invert(monitor.goal_pose), monitor.goal_model.get_ee_pose())
    ee_pose_0 = huskies[0].object.get_ee_pose()
    ee_pose_x = pp.multiply(ee_pose_0, pp.Pose(point=pp.Point(x=0.1)))
    ee_pose_y = pp.multiply(ee_pose_0, pp.Pose(point=pp.Point(y=0.1)))
    # local frame: [ee_pose_0, ee_pose_x, ee_pose_y, ee_pose_0]
    
    world_ee_poses = [
        pp.Pose(point=pp.Point(2, -1, 1), euler=pp.Euler(roll=np.pi/2, yaw=-np.pi/2)),
        pp.Pose(point=pp.Point(2.5, -1, 1.2)),
        pp.Pose(point=pp.Point(2, 0, 1), euler=pp.Euler(roll=np.pi/2, yaw=-np.pi/2)),
        pp.Pose(point=pp.Point(2.5, 0, 1.2)),
        pp.Pose(point=pp.Point(2, 1, 1), euler=pp.Euler(roll=np.pi/2, yaw=-np.pi/2)),
        pp.Pose(point=pp.Point(2.5, 1, 1.2))
        ]
    
    draw_list = []
    for pose in world_ee_poses:
        pp.remove_handles(draw_list)
        draw_list = pp.draw_pose(pose)
        while True:
            if hi.is_arm_executing:
                break
            if calibration_confirm:
                calibration_confirm = False
                
                arm_joint_pose = planning.arm_ik(huskies[0], pose)
                if arm_joint_pose is None:
                    monitor.get_logger().warn('Ik for calibration failed!')
                    monitor.set_arm_trajectory((None, None, 2))
                else:
                    monitor.set_arm_trajectory(([hi.arm_joint_pose, arm_joint_pose], None, 5))
            yield
        
        while hi.is_arm_executing:
            yield # wait for execution to finish
            
    pp.remove_handles(draw_list)
    
    monitor.get_logger().info('Calibration squence finished!')
    calibration_running = False
    
def execute_arm_trajectory(monitor):
    if monitor.planned_arm_trajectory[0] is None:
        monitor.get_logger().warn('Arm trajectory must be planed before executing!')
        return
    huskies[0].interface.send_arm_cmd(*monitor.planned_arm_trajectory)
    
    # TESTING SIMULTANEOUS ARM WAVE
    huskies[1].interface.send_arm_cmd(*monitor.planned_arm_trajectory)
    
def move_to_goal(monitor):
    if monitor.planned_base_trajectory[0] is None:
        monitor.get_logger().warn('Base trajectory must be planed before executing!')
        return
    monitor.tasks.append(control.execute_base_trajectory(monitor, huskies[0], monitor.planned_base_trajectory))
    

def set_gripper(monitor):
    huskies[0].interface.send_gripper_cmd(monitor.goal_gripper, 0.1)