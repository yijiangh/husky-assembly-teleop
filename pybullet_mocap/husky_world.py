"""
This module contains the world definition and high level actions or sequences of actions for the huskies.
"""

import numpy as np

from pybullet_mocap.common import Husky, TrackedObject
import pybullet_mocap.husky_planning as planning
import pybullet_mocap.husky_control as control


boxes = []
huskies = []

def init(monitor):
    boxes.append(TrackedObject(monitor, 'box1', 4457, np.zeros(3), np.array((0, 0, 0, 1)), 0.2, 'cube.obj'))
    boxes.append(TrackedObject(monitor, 'box2', 4484, np.zeros(3), np.array((0, 0, 0, 1)), 0.2, 'cube.obj'))
    boxes.append(TrackedObject(monitor, 'box3', 1031, np.zeros(3), np.array((0, 0, 0, 1)), 0.2, 'cube.obj'))
    
    huskies.append(Husky(monitor, name='/a200_0804', mocap_id=1004, pos=np.array((0,0,0))))
    #husky_iterfaces.append(Husky(monitor, name='/a200_0805', mocap_id=1033, pos=np.array((0,1,0))))

def update(monitor):
    pass

def plan_to_goal(monitor):
    base = planning.plan_base_motion(huskies[0], (monitor.goal_pos, monitor.goal_rot), monitor.goal_arm_pose, boxes)
    monitor.set_base_trajectry(base)

def plan_arm_wave(monitor):
    arm_pos_traj, _ = planning.plan_arm_wave(huskies[0])
    monitor.set_arm_trajectory(arm_pos_traj)

def plan_arm_to_goal(monitor):
    hi = huskies[0].interface
    monitor.set_arm_trajectory(planning.plan_arm_motion(huskies[0], monitor.goal_arm_pose, boxes))
    # monitor.set_arm_trajectory([hi.arm_joint_pose, monitor.goal_arm_pose])

def execute_arm_trajectory(monitor):
    huskies[0].interface.send_arm_cmd(monitor.planned_arm_trajectory, dt=0.1) # TODO: get correct time information!
    
def move_to_goal(monitor):
    control.execute_base_trajectory(monitor, huskies[0], monitor.base_trajectory) 

def set_gripper(monitor):
    huskies[0].interface.set_gripper(monitor.goal_gripper)