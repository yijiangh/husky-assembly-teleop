from pybullet_mocap.common import Husky, TrackedObject
from pybullet_mocap.husky_planning import plan_base_motion
from pybullet_mocap.husky_robot import HuskyRobotInterface

import numpy as np


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
    base, arm = plan_base_motion(huskies[0], (monitor.goal_pos, monitor.goal_rot), monitor.goal_arm_pose, boxes)
    monitor.set_base_trajectry(base)
    monitor.planned_arm_trajectory = arm
    
def move_to_goal(monitor):
    monitor.execute_base_trajectory()