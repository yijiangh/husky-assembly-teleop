"""
This module contains control logic for the husky robot.
Most of it is handled on the robot itself, but the base closed loop control is implemented here as it needs mocap data.
"""

import time
import numpy as np
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R

from typing import List, Tuple

from pybullet_mocap.common import Husky

def execute_base_trajectory(monitor, husky: Husky, trajectory: Tuple[List[Tuple[np.ndarray, np.ndarray]], float]):
    actual_trajectory = []
    actual_rots = []
    target_rots = []
    ortho_error_list = []
    para_error_list = []
    rot_error_list = []
    rot_vel_error_list = []
    ortho_vel_error_list = []
    para_vel_error_list = []
    
    vel_list = []
    target_vel_list = []
    ang_vel_list = []
    target_ang_vel_list = []
    
    hi = husky.interface
    
    k_p = np.array([3.0, 5.0])
    k_d = 0.5 * np.array([0.2, 0.5])
    k_p_ortho = 5 # 5 #p.readUserDebugParameter(self.pid_sliders[0])
    
    time_start = time.time()
    total_time = trajectory[1]
    
    yield
    
    while True:
        exec_time = time.time() - time_start
        
        N = len(trajectory[0])    
        dt = (total_time / (N - 1))

        # get trajectory data
        base_traj_idx = min(int(exec_time / total_time * (N - 1)), N-1)
        base_traj_next_idx = min(int(exec_time / total_time * (N - 1))+1, N-1)
        
        target_pos, target_rot  = trajectory[0][base_traj_idx]
        next_target_pos, next_target_rot = trajectory[0][base_traj_next_idx]
        
        target_vel = np.linalg.norm((next_target_pos - target_pos) / dt)
        target_rot_vel = ((R.from_quat(target_rot).inv() * R.from_quat(next_target_rot)).as_euler("zxy")[0]) / dt
        
        # get current data
        current_pos, current_rot = (hi.position, hi.rotation)
        current_vel, current_rot_vel = (hi.velocity, hi.angular_velocity[2])
        
        # split pos error into parallel and orthogonal components
        pos_error = target_pos - current_pos
        pos_error_local = R.from_quat(current_rot).inv().apply(pos_error)
        pos_err_para, pos_err_ortho = pos_error_local[0], pos_error_local[1]
        
        vel_local = R.from_quat(current_rot).inv().apply(current_vel)
        vel_para, vel_ortho = vel_local[0], vel_local[1]
        
        vel_err_para = target_vel - vel_para
        vel_err_ortho = 0 - vel_ortho
        
        # adjust target angle to reduce ortho pos error
        # rot_error = np.arctan2(pos_err_ortho, pos_err_para) # actual error to drive directly to next waypoint
        rot_offset = k_p_ortho * np.arctan2(pos_err_ortho, 1.0) * target_vel
        rot_err = ((R.from_quat(current_rot).inv() * R.from_quat(target_rot)).as_euler("zxy")[0]) + rot_offset
        
        rot_vel_err = target_rot_vel - current_rot_vel
        
        target_vel_list.append(target_vel)
        target_ang_vel_list.append(target_rot_vel)
        vel_list.append(np.linalg.norm(current_vel))
        ang_vel_list.append(current_rot_vel)
        
        actual_trajectory.append(current_pos)
        actual_rots.append(R.from_quat(current_rot).as_euler("zxy")[0])
        target_rots.append(R.from_quat(target_rot).as_euler("zxy")[0])
        
        para_error_list.append(pos_err_para)
        ortho_error_list.append(pos_err_ortho)
        rot_error_list.append(rot_err)
        
        para_vel_error_list.append(vel_err_para)
        ortho_vel_error_list.append(vel_err_ortho)
        rot_vel_error_list.append(rot_vel_err)
        
        twist = k_p * np.array([pos_err_para, rot_err]) + k_d * np.array([vel_err_para, rot_vel_err]) + np.array([target_vel, target_rot_vel])

        hi.send_base_twist_cmd(twist[0], twist[1])
        
        converged = np.abs(pos_err_para) < 0.05 and np.abs(rot_err) < 0.05
        if exec_time < 2*total_time and (exec_time < total_time or not converged):
            yield
        else:
            break
        
    points = np.array([pos for pos, _ in trajectory[0]])
    actual_trajectory = np.array(actual_trajectory)
    
    fig, ((ax_traj, ax_traj_rot), (ax_vel, ax_rot_vel), (ax_err, ax_vel_err)) = plt.subplots(3, 2)
    ax_traj.plot(points[:,0], points[:,1])
    ax_traj.plot(actual_trajectory[:,0], actual_trajectory[:,1])
    ax_traj.set_aspect(1.0)
    ax_traj_rot.plot(target_rots, label='target')
    ax_traj_rot.plot(actual_rots, label='actual')
    ax_traj_rot.legend()
    ax_err.plot(para_error_list, label='para')
    ax_err.plot(ortho_error_list, label='ortho')
    ax_err.plot(rot_error_list, label='rot')
    ax_err.legend()
    ax_vel_err.plot(para_vel_error_list, label='v para')
    ax_vel_err.plot(ortho_vel_error_list, label='v ortho')
    ax_vel_err.plot(rot_vel_error_list, label='w rot')
    ax_vel_err.legend()
    ax_vel.plot(vel_list, label='v real')
    ax_vel.plot(target_vel_list, label='v target')
    ax_vel.legend()
    ax_rot_vel.plot(ang_vel_list, label='w real')
    ax_rot_vel.plot(target_ang_vel_list, label='w target')
    ax_rot_vel.legend()
    fig.set_dpi(300)
    fig.savefig("trajectory.png")
    plt.close(fig)