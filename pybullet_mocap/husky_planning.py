
import numpy as np
from scipy.spatial.transform import Rotation as R
import pybullet as p
import pybullet_planning as pp

from pybullet_mocap.common import Husky, HuskyObject
from pybullet_mocap.controller import Stanley, State
from pybullet_mocap.planner import RRTStar, fill_yaw_angle
from pybullet_mocap.utils import plan_transit_motion
from pybullet_planning.utils import RED

def lerp(a, b, t):
    return a + t * (b - a)

def quat_lerp(q1, q2, t):
    if np.dot(q1,q2) < 0:
        q2 = -q2
    
    res = lerp(q1, q2, t)
    res /= np.linalg.norm(res)
    
    return res



def plan_base_motion(husky: Husky, goal_pose, arm_goal_pose, obstacles):
    planned_arm_trajectory = plan_transit_motion(
                husky.object.robot,
                arm_goal_pose,
                [husky.object.ee_attachment],
                [],
                debug=True,
                disabled_collisions=False,
            )
    
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
    
    points = [(x, y, 0.0) for x, y in zip(x_list, y_list)]
    with pp.LockRenderer():
        pp.add_segments(points)
        
    
    planned_base_trajectory_rrt = [
        (np.array((x, y, 0)), R.from_euler('z', yaw).as_quat()) for x, y, yaw in zip(x_list, y_list, yaw_list)
    ]
        
    planned_pos_yaw = [
        (np.array((x, y, 0)), yaw)
        for x, y, yaw in zip(x_list, y_list, yaw_list)
    ]
        
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
    
    planned_base_trajectory = []
    i = 0
    for t2 in np.arange(0, t, 0.1):
        while time_stamps[i] <= t2:
            i += 1
        
        dt_norm = (t2 - time_stamps[i-1]) / (time_stamps[i] - time_stamps[i-1])
        inter_pos = lerp(planned_base_trajectory_rrt[i-1][0], planned_base_trajectory_rrt[i][0], dt_norm)
        inter_rot = quat_lerp(planned_base_trajectory_rrt[i-1][1], planned_base_trajectory_rrt[i][1], dt_norm)
        planned_base_trajectory.append((inter_pos, inter_rot))
    
    points = [
        pos for pos, _ in planned_base_trajectory
    ]
    with pp.LockRenderer():
        pp.add_segments(points, color=RED)
        
    return planned_base_trajectory, planned_arm_trajectory

def plan_arc(husky: Husky):
    hi = husky.interface
    
    start_pos = hi.position
    start_rot = R.from_quat(hi.rotation)
    
    N = 200
    radius = 1
    angle = np.pi
    arc_trajectory = [(np.array([np.sin(i/N * angle) * radius, np.cos(i/N * angle) * radius - radius, 0]), R.from_euler("z", -i/N * angle)) for i in range(N+1)]
    arc_trajectory = [(start_pos + start_rot.apply(pos), (start_rot * rot).as_quat()) for pos, rot in arc_trajectory]
        
    return arc_trajectory
    
def plan_corner(husky: Husky):
    hi = husky.interface
    
    start_pos = hi.position
    start_rot = R.from_quat(hi.rotation)
    
    N = 200
    angle = 0.75 * np.pi
    distance = 1.0
    discrete_trajectory = (
        [(np.array([i/N * distance, 0, 0]), R.identity()) for i in range(N+1)] +
        [(np.array([distance, 0, 0]), R.from_euler("z", -i/N * angle)) for i in range(N+1)] + 
        [(np.array([distance + np.cos(angle) * i/N * distance, -np.sin(angle) * i/N * distance, 0]), R.from_euler("z", -angle)) for i in range(N+1)]
    )
    discrete_trajectory = [(start_pos + start_rot.apply(pos), (start_rot * rot).as_quat()) for pos, rot in discrete_trajectory]
        
    return discrete_trajectory