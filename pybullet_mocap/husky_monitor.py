import time
import rclpy
from rclpy.node import Node

import os
from matplotlib import pyplot as plt
import numpy as np
import pybullet_planning as pp
import pybullet as p
from scipy.spatial.transform import Rotation as R

from pybullet_mocap.husky_robot import HuskyRobotInterface
from pybullet_mocap import DATA_DIRECTORY
import pybullet_mocap.husky_world as world
from pybullet_mocap.common import Husky, TrackedObject, HuskyObject

from pybullet_mocap.utils import plan_transit_motion
from pybullet_mocap.planner import RRTStar, fill_yaw_angle
from pybullet_mocap.controller import Stanley, State
from pybullet_planning.utils import RED

from pybullet_mocap.optitrack.NatNetClient import NatNetClient

HUSKY_JOINT_NAMES = ['x', 'y', 'theta']
HUSKY_UR5e_JOINT_NAMES = ["ur_arm_shoulder_pan_joint", 
                      "ur_arm_shoulder_lift_joint",
                      "ur_arm_elbow_joint", 
                      "ur_arm_wrist_1_joint", 
                      "ur_arm_wrist_2_joint", 
                      "ur_arm_wrist_3_joint" ]

TOOL0_FROM_TIP = pp.Pose(point=(0, 0, 73.65 * 1e-3))

DEFAULT_GREY = [0.2, 0.2, 0.2, 0.7]
GOAL_BLUE = [0, 0.2, 0.5, 0.7]
TRAJECTORY_GREEN = [0, 0.5, 0.2, 0.7]

CLIENT_IP = '192.168.0.7' # Set to your own IP
MOCAP_IP = '192.168.0.117' # set to the mocap PC's IP, get this from Motive Settings>Streaming pane->Local interface

class Button:
    def __init__(self, name, action):
        self.dbg_param = p.addUserDebugParameter(name, 1.0, 0.0, 0.0)
        self.prev_value = p.readUserDebugParameter(self.dbg_param)
        self.action = action
        
    def update(self):
        new_value = p.readUserDebugParameter(self.dbg_param)
        if new_value != self.prev_value:
            self.prev_value = new_value
            self.action()
        
class HuskyMonitor(Node):
    def __init__(self):
        super().__init__('husky_monitor')
        
        self.husky_interfaces = []
        self.timer = self.create_timer(0.05, self.update_pybullet)
        
        self.tasks = []
        
        self.husky_objects = []
        self.tracked_objects = []
        self.name_from_mocap_id = {}
        
        self.buttons = []
        self.state_sliders = []
        self.joint_state_sliders = []
        self.pid_sliders = []
        self.planned_arm_trajectory = None
        self.plan_traj_seg = None
        self.planned_base_trajectory = None
        self.goal_pos = np.zeros(3)
        self.goal_rot = R.identity().as_quat()
        self.goal_arm_pose = np.zeros(6)
        self.start_pybullet()
        self.start_mocap()
        
        world.init(self)

        self.build_ui()
        
    def add_tracked_object(self, obstacle: TrackedObject):
        self.tracked_objects.append(obstacle)
        self.name_from_mocap_id[obstacle.mocap_id] = obstacle.name
        
    def add_husky(self, husky: Husky):
        self.husky_interfaces.append(husky.interface)
        self.husky_objects.append(husky.object)
        self.name_from_mocap_id[husky.mocap_id] = husky.name
        
    def set_base_trajectry(self, base_trajectory):
            self.planned_base_trajectory = base_trajectory
            
            # draw
            points = [
                pos for pos, _ in self.planned_base_trajectory
            ]
            with pp.LockRenderer():
                with pp.HideOutput():
                    if self.plan_traj_seg is not None:
                       pp.remove_all_debug()
                    self.plan_traj_seg = pp.add_segments(points)
    
    def execute_base_trajectory(self):
        if self.planned_base_trajectory is None:
            self.get_logger().warn('Trajectory must be planned before executing!')
            return
        
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
        
        hi = self.husky_interfaces[0]
        
        k_p = np.array([3.0, 5.0])
        k_d = 0.5 * np.array([0.2, 0.5])
        k_p_ortho = 5 # 5 #p.readUserDebugParameter(self.pid_sliders[0])
        
        time_start = time.time()
        total_time = 10
        def exec():
            nonlocal time_start
            nonlocal actual_trajectory
            nonlocal actual_rots
            nonlocal target_rots
            nonlocal ortho_error_list
            nonlocal para_error_list
            nonlocal rot_error_list
            nonlocal vel_list
            nonlocal ang_vel_list
            nonlocal target_ang_vel_list
            exec_time = time.time() - time_start
            
            N = len(self.planned_base_trajectory)    
            dt = (total_time / (N - 1))

            # get trajectory data
            base_traj_idx = min(int(exec_time / total_time * (N - 1)), N-1)
            base_traj_next_idx = min(int(exec_time / total_time * (N - 1))+1, N-1)
            
            target_pos, target_rot  = self.planned_base_trajectory[base_traj_idx]
            next_target_pos, next_target_rot = self.planned_base_trajectory[base_traj_next_idx]
            
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
                return True
            
            points = np.array([pos for pos, _ in self.planned_base_trajectory])
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
            
            return False
        
        self.tasks.append(exec)
    
    def send_motion_command(self):
        for hi in self.husky_interfaces:
            hi.send_base_twist_cmd(0.1, 0.0)
        self.get_logger().info('Sent motion command!')
        
    def send_arm_command(self):
        joint_state_slider_values = np.array([p.readUserDebugParameter(ps) for ps in self.joint_state_sliders])
        for hi in self.husky_interfaces:
            hi.send_arm_cmd([joint_state_slider_values], dt=5)
        self.get_logger().info('Sent arm command!')
        
    def send_arm_wave(self):
        for hi in self.husky_interfaces:
            if not np.isclose(hi.arm_joint_states, [0, -np.pi/2, 0, -np.pi/2, 0, 0], atol=0.1).all():
                self.get_logger().warn(f'Husky {hi} is not in correct wave start pose!')
                return
        
        N = 20 # number of waypoints (N+1 with starting point)
        TIME = 20
        
        ts = list(np.linspace(0, TIME, N+1))[1:]
        time_scaling = lambda t: t/TIME*2*np.pi
        
        traj_pos = [np.array([0, -np.pi/2, -np.sin(time_scaling(t)), -np.pi/2 + np.sin(time_scaling(t)), 0, 0]) for t in ts]
        traj_vel = [1 / TIME * 2*np.pi * np.array([0, 0, -np.cos(time_scaling(t)), np.cos(time_scaling(t)), 0, 0]) for t in ts]
        
        for hi in self.husky_interfaces:
            hi.send_arm_cmd(traj_pos, traj_vel, dt=TIME/N)
        self.get_logger().info('Sent arm command!')
        
    def send_gripper_command(self):
        for hi in self.husky_interfaces:
            hi.send_gripper_cmd(p.readUserDebugParameter(self.gripper_slider), 0.1)
        self.get_logger().info('Sent gripper command!')
        
    def reset_ui(self):
        # reset all sliders to default value by recreating them...
        # pybullet seems to lack a setUserDebugParameter() method :(
        p.removeAllUserParameters()
        self.buttons.clear()
        self.state_sliders.clear()
        self.joint_state_sliders.clear()
        self.build_ui()
    
    # --- --- --- --- --- SETUP PYBULLET --- --- --- --- ---
    def start_pybullet(self):
        # start pybullet simulator
        pp.connect(use_gui=True, shadows=True, color=[0.9, 0.9, 1.0])
        # turn on the GUI panels
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1, physicsClientId=pp.CLIENT)
        
        # draw world frame
        pp.draw_pose(pp.unit_pose(), 0.1)

        # load goal robot model
        with pp.LockRenderer():
            with pp.HideOutput():                
                self.goal_model = HuskyObject()
                self.goal_model.set_color(GOAL_BLUE)
        
    def build_ui(self):
        self.time_slider = p.addUserDebugParameter("time", 0.0, 1.0, 1.0)
        
        self.buttons.append(Button('Reset Goal State', self.reset_ui))
        
        self.state_sliders.append(p.addUserDebugParameter("x", -5.0, 5.0, 0))
        self.state_sliders.append(p.addUserDebugParameter("x", -5.0, 5.0, 0))
        self.state_sliders.append(p.addUserDebugParameter("yaw", -np.pi, np.pi, 0))
        
        self.pid_sliders.append(p.addUserDebugParameter("P", 0, 100, 24))
        
        self.buttons.append(Button('Plan', lambda: world.plan_to_goal(self)))
        self.buttons.append(Button('Exec', lambda: world.move_to_goal(self)))
        
        for i, j in enumerate(pp.joints_from_names(self.husky_objects[0].robot, HUSKY_UR5e_JOINT_NAMES)):
            lower, upper = pp.get_joint_limits(self.husky_objects[0].robot, j)
            self.joint_state_sliders.append(p.addUserDebugParameter(f'Joint {i}', lower, upper, self.husky_interfaces[0].arm_joint_states[i]))
            
        self.buttons.append(Button('Send Arm Command', self.send_arm_command))
        self.buttons.append(Button('Send Arm Wave', self.send_arm_wave))

        self.gripper_slider = p.addUserDebugParameter("gripper", 0, 1.0)
        self.buttons.append(Button('Gripper Set', self.send_gripper_command))
    
    # --- --- --- --- --- MOCAP --- --- --- --- --- 
    def start_mocap(self):
        print('Starting mocap!')
        mocap_client = NatNetClient()
        mocap_client.set_client_address(CLIENT_IP)
        mocap_client.set_server_address(MOCAP_IP)
        mocap_client.set_use_multicast(False)
        mocap_client.print_level = 1
        mocap_client.rigid_body_listener = self.receive_rigid_body_frame
        mocap_client.new_frame_listener = self.receive_mocap_frame
        
        if mocap_client.run():
            start_connect = time.time()
            while not mocap_client.connected():
                time.sleep(0.25)
                if time.time() - start_connect > 5:
                    break
            print(f"mocap client connected: {mocap_client.connected()}")
        else:
            print('Failed to run mocap client!')
    
    _mocap_rigidbody_cache = {}
    def receive_rigid_body_frame(self, id, pos, rot):
        if id not in self.name_from_mocap_id:
            return
        
        pos = np.array((pos[2], pos[0], pos[1]))
        rot = np.array((rot[2], rot[0], rot[1], rot[3]))       
        
        name = self.name_from_mocap_id[id]
        for hi in self.husky_interfaces:
            if hi.name == name:
                self._mocap_rigidbody_cache[name] = (pos, rot)
                
        for o in self.tracked_objects:
            if o.name == name:
                self._mocap_rigidbody_cache[name] = (pos, rot)
    
    def receive_mocap_frame(self, data):
        ts = data['timestamp']
        for hi in self.husky_interfaces:
            if hi.name not in self._mocap_rigidbody_cache:
                continue
            (pos, rot) = self._mocap_rigidbody_cache[hi.name]
            hi.mocap_callback(pos, rot, ts)
        for o in self.tracked_objects:
            if o.name not in self._mocap_rigidbody_cache:
                continue
            (pos, rot) = self._mocap_rigidbody_cache[o.name]
            o.mocap_callback(pos, rot, ts)
        self._mocap_rigidbody_cache.clear()
        
    # --- --- --- --- --- UPDATE --- --- --- --- --- 
    def update_pybullet(self):
        for b in self.buttons:
            b.update()
            
        # update tracked objects
        for i, o in enumerate(self.tracked_objects):
            o.set_pose((o.pos, o.rot))
        
        # update robot state
        for i, hi in enumerate(self.husky_interfaces):
            self.husky_objects[i].set_pose((hi.position, hi.rotation), hi.arm_joint_states)
        
        # update goal robot state
        state_slider_values = [p.readUserDebugParameter(ps) for ps in self.state_sliders]
        self.goal_pos = np.array((state_slider_values[0], state_slider_values[1], 0))
        self.goal_rot = R.from_euler("z", state_slider_values[2], degrees=False).as_quat()
        self.goal_arm_pose = np.array([p.readUserDebugParameter(ps) for ps in self.joint_state_sliders])
            
        preview_time = p.readUserDebugParameter(self.time_slider)
        goal_pose = (self.goal_pos, self.goal_rot)
        goal_arm_pose = self.goal_arm_pose
        if not np.isclose(preview_time, 1.0):
            if self.planned_base_trajectory:
                base_traj_idx = int(preview_time * (len(self.planned_base_trajectory) - 1))
                goal_pose = self.planned_base_trajectory[base_traj_idx]
            if self.planned_arm_trajectory:
                arm_traj_idx = int(preview_time * (len(self.planned_arm_trajectory) - 1))
                goal_arm_pose = self.planned_arm_trajectory[arm_traj_idx]
            
        self.goal_model.set_pose(goal_pose, goal_arm_pose)
        
        # run tasks
        i = 0
        while i < len(self.tasks):
            task = self.tasks[i]
            if task():
                i += 1
            else:
                self.tasks.remove(task)
                
        world.update(self)
                


# --- --- --- --- --- MAIN --- --- --- --- --- 
def main(args=None):
    rclpy.init(args=args)

    husky_monitor = HuskyMonitor()

    rclpy.spin(husky_monitor)

    husky_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':     
    main()
