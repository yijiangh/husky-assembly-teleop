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

ROBOT_START_POS = [-np.array((0,0,0)), -np.array((0,1,0)), -np.array((0,2,0))]

CLIENT_IP = '192.168.0.7' # Set to your own IP
MOCAP_IP = '192.168.0.117' # set to the mocap PC's IP, get this from Motive Settings>Streaming pane->Local interface

# the dictionary key is the rigid body id you find in the Motive software, the name doesn't matter
name_from_mocap_id = {
    1004 : '/a200_0804',
    1033 : '/a200_0805',
}

def load_robot(ik_from_arm_base=True, load_calib_tip=False):
    # robot_srdf = os.path.join(DATA_DIRECTORY, 'husky_urdf/mt_husky_moveit_config/config/husky.srdf')
    robot_urdf = os.path.join(DATA_DIRECTORY,'husky_urdf/mt_husky_moveit_config/urdf/husky_ur5_e.urdf')
    # robot_urdf = os.path.join(DATA_DIRECTORY,'husky_urdf/mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint.urdf')

    if load_calib_tip:
        gripper_obj = os.path.join(DATA_DIRECTORY,'calibration_tip.stl')
        gripper_scale = 1
    else:
        gripper_obj = os.path.join(DATA_DIRECTORY,'husky_urdf/robotiq_85/meshes/static/robotiq_85_close_20mm.obj')
        gripper_scale = 1

    assert os.path.exists(robot_urdf)
    assert os.path.exists(gripper_obj)

    robot = pp.load_pybullet(robot_urdf, fixed_base=False, cylinder=False)
    robot_pose = pp.get_pose(robot)

    tool0_pose = pp.get_link_pose(robot, pp.link_from_name(robot, 'ur_arm_tool0'))
    ee = pp.create_obj(gripper_obj, scale=gripper_scale) 
    pp.set_pose(ee, pp.multiply(tool0_pose, pp.Pose(euler=pp.Euler(yaw=-np.pi/2))))
    
    ee_attachment = pp.create_attachment(robot, pp.link_from_name(robot, 'ur_arm_tool0'), ee)

    return HuskyModel(robot, ee, ee_attachment)

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
            
class HuskyModel():
    def __init__(self, robot, ee, ee_attachment):
       self.robot = robot
       self.ee = ee
       self.ee_attachment = ee_attachment
       self.old_color = None
       
    def set_pose(self, base_pose, arm_joint_states):
        pp.set_pose(self.robot, base_pose)
        arm_joints = pp.joints_from_names(self.robot, HUSKY_UR5e_JOINT_NAMES)
        pp.set_joint_positions(self.robot, arm_joints, arm_joint_states)
        self.ee_attachment.assign()
        
    def set_color(self, new_color):
        if self.old_color != new_color:
            self.old_color = new_color
            pp.set_color(self.robot, new_color)
            pp.set_color(self.ee, new_color)
        
class HuskyMonitor(Node):
    def __init__(self):
        super().__init__('husky_monitor')
        
        #self.huskyInterfaces = [HuskyRobotInterface(self, name='/a200_0804', use_odom=False), HuskyRobotInterface(self, name='/a200_0805', use_odom=False)]
        self.huskyInterfaces = [HuskyRobotInterface(self, name='/a200_0804', use_odom=False)]
        self.timer = self.create_timer(0.05, self.update_pybullet)
        
        self.tasks = []
        
        self.huskyModels = []
        
        self.buttons = []
        self.state_sliders = []
        self.joint_state_sliders = []
        self.pid_sliders = []
        self.planned_arm_trajectory = None
        self.plan_traj_seg = None
        self.planned_base_trajectory = None
        self.goal_pos = np.zeros(3)
        self.goal_rot = R.identity()
        self.goal_arm_pose = np.zeros(6)
        self.start_pybullet()
        self.start_mocap()
    
    def reset_odom(self):
        for i, hi in enumerate(self.huskyInterfaces):
            hi.odom_offset = ROBOT_START_POS[i] + hi.raw_odom_position
    
    def plan(self):
        hi = self.huskyInterfaces[0]
        
        start_pos = hi.position
        start_rot = R.from_quat(hi.rotation)
        
        N = 200
        radius = 1
        angle = np.pi
        arc_trajectory = [(np.array([np.sin(i/N * angle) * radius, np.cos(i/N * angle) * radius - radius, 0]), R.from_euler("z", -i/N * angle)) for i in range(N+1)]
        arc_trajectory = [(start_pos + start_rot.apply(pos), (start_rot * rot).as_quat()) for pos, rot in arc_trajectory]
        
        points = [pos for pos, rot in arc_trajectory]
        with pp.LockRenderer():
            if self.plan_traj_seg is not None:
                pp.remove_handles(self.plan_traj_seg)
            self.plan_traj_seg = pp.add_segments(points)
            
        self.planned_base_trajectory = arc_trajectory
    
    def execute_base_trajectory(self):
        if self.planned_base_trajectory is None:
            self.get_logger().warn('Trajectory must be planned before executing!')
            return
        
        fig, (ax_traj, ax_ortho) = plt.subplots(2, 1)
        actual_trajectory = []
        ortho_error_list = []
        para_error_list = []
        rot_error_list = []
        
        hi = self.huskyInterfaces[0]
        
        k_p = np.array([2.0, 1.0])
        k_p_ortho = 24 #p.readUserDebugParameter(self.pid_sliders[0])
        
        time_start = time.time()
        total_time = 10
        def exec():
            nonlocal time_start
            nonlocal actual_trajectory
            nonlocal ortho_error_list
            nonlocal para_error_list
            nonlocal rot_error_list
            exec_time = time.time() - time_start
            
            N = len(self.planned_base_trajectory)    
            dt = (total_time / (N - 1))
        
            base_traj_idx = min(int(exec_time / total_time * (N - 1)), N-1)
            base_traj_next_idx = min(int(exec_time / total_time * (N - 1))+1, N-1)
            target_pos, target_rot  = self.planned_base_trajectory[base_traj_idx]
            next_target_pos, next_target_rot = self.planned_base_trajectory[base_traj_next_idx]
            target_vel = np.linalg.norm((next_target_pos - target_pos)) / dt
            target_rot_vel = ((R.from_quat(target_rot).inv() * R.from_quat(next_target_rot)).as_euler("zxy")[0]) / dt
            
            current_pos, current_rot = (hi.position, hi.rotation)
            
            actual_trajectory.append(current_pos)
            
            # split pos error into parallel and orthogonal components
            pos_error = target_pos - current_pos
            pos_error_local = R.from_quat(current_rot).inv().apply(pos_error)
            pos_err_para, pos_err_ortho = pos_error_local[0], pos_error_local[1]
            para_error_list.append(pos_err_para)
            ortho_error_list.append(pos_err_ortho)
            
            # adjust target angle to reduce ortho pos error
            rot_offset = k_p_ortho * np.arctan2(pos_err_ortho, 1.0) * target_vel
            print(f'pos_err_ortho {pos_err_ortho} pos_err_para {pos_err_para} rot_offset {rot_offset}')
            rot_error = ((R.from_quat(current_rot).inv() * R.from_quat(target_rot)).as_euler("zxy")[0]) + rot_offset
            rot_error_list.append(rot_error)
            
            twist = k_p * np.array([pos_err_para, rot_error]) + np.array([target_vel, target_rot_vel])

            hi.send_base_twist_cmd(twist[0], twist[1])
            
            converged = np.abs(pos_err_para) < 0.05 and np.abs(rot_error) < 0.05
            if exec_time < 2*total_time and (exec_time < total_time or not converged):
                return True
            
            points = np.array([pos for pos, _ in self.planned_base_trajectory])
            actual_trajectory = np.array(actual_trajectory)
            
            ax_traj.plot(points[:,0], points[:,1])
            ax_traj.plot(actual_trajectory[:,0], actual_trajectory[:,1])
            ax_traj.set_aspect(1.0)
            ax_ortho.plot(para_error_list, label='para')
            ax_ortho.plot(ortho_error_list, label='ortho')
            ax_ortho.plot(rot_error_list, label='rot')
            ax_ortho.legend()
            fig.savefig("trajectory.png")
            plt.close(fig)
            
            return False
        
        self.tasks.append(exec)
    
    def send_motion_command(self):
        for hi in self.huskyInterfaces:
            hi.send_base_twist_cmd(0.1, 0.0)
        self.get_logger().info('Sent motion command!')
        
    def send_arm_command(self):
        joint_state_slider_values = np.array([p.readUserDebugParameter(ps) for ps in self.joint_state_sliders])
        for hi in self.huskyInterfaces:
            hi.send_arm_cmd([joint_state_slider_values], dt=5)
        self.get_logger().info('Sent arm command!')
        
    def send_arm_wave(self):
        for hi in self.huskyInterfaces:
            if not np.isclose(hi.arm_joint_states, [0, -np.pi/2, 0, -np.pi/2, 0, 0], atol=0.1).all():
                self.get_logger().warn(f'Husky {hi} is not in correct wave start pose!')
                return
        
        N = 20 # number of waypoints (N+1 with starting point)
        TIME = 20
        
        ts = list(np.linspace(0, TIME, N+1))[1:]
        time_scaling = lambda t: t/TIME*2*np.pi
        
        traj_pos = [np.array([0, -np.pi/2, -np.sin(time_scaling(t)), -np.pi/2 + np.sin(time_scaling(t)), 0, 0]) for t in ts]
        traj_vel = [1 / TIME * 2*np.pi * np.array([0, 0, -np.cos(time_scaling(t)), np.cos(time_scaling(t)), 0, 0]) for t in ts]
        
        for hi in self.huskyInterfaces:
            hi.send_arm_cmd(traj_pos, traj_vel, dt=TIME/N)
        self.get_logger().info('Sent arm command!')
        
    def send_gripper_command(self):
        for hi in self.huskyInterfaces:
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
        
        # load robot models
        with pp.LockRenderer():
            with pp.HideOutput():
                for i, hi in enumerate(self.huskyInterfaces):
                    self.huskyModels.append(load_robot(load_calib_tip=False))
                    hi.odom_offset = ROBOT_START_POS[i]
                    hi.position = -ROBOT_START_POS[i]
            
                
                self.goalModel = load_robot(load_calib_tip=False)
                self.goalModel.set_color(GOAL_BLUE)
                
        # UI
        self.build_ui()
        
    def build_ui(self):
        self.time_slider = p.addUserDebugParameter("time", 0.0, 1.0, 1.0)
        
        self.buttons.append(Button('Reset Odom', self.reset_odom))
        self.buttons.append(Button('Reset Goal State', self.reset_ui))
        
        self.state_sliders.append(p.addUserDebugParameter("x", -5.0, 5.0, 0))
        self.state_sliders.append(p.addUserDebugParameter("x", -5.0, 5.0, 0))
        self.state_sliders.append(p.addUserDebugParameter("yaw", -np.pi, np.pi, 0))
        
        self.pid_sliders.append(p.addUserDebugParameter("P", 0, 100, 24))
        
        self.buttons.append(Button('Plan', self.plan))
        self.buttons.append(Button('Exec', self.execute_base_trajectory))
        
        for i, j in enumerate(pp.joints_from_names(self.huskyModels[0].robot, HUSKY_UR5e_JOINT_NAMES)):
            lower, upper = pp.get_joint_limits(self.huskyModels[0].robot, j)
            self.joint_state_sliders.append(p.addUserDebugParameter(f'Joint {i}', lower, upper, self.huskyInterfaces[0].arm_joint_states[i]))
            
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
        
        if mocap_client.run():
            start_connect = time.time()
            while not mocap_client.connected():
                time.sleep(0.25)
                if time.time() - start_connect > 5:
                    break
            print(f"mocap client connected: {mocap_client.connected()}")
        else:
            print('Failed to run mocap client!')
        
    def receive_rigid_body_frame(self, id, pos, rot):
        if id not in name_from_mocap_id:
            return
        
        name = name_from_mocap_id[id]
        for hi in self.huskyInterfaces:
            if hi.name == name:
                hi.position = np.array((pos[2], pos[0], pos[1]))
                hi.rotation = np.array((rot[2], rot[0], rot[1], rot[3]))
        
    # --- --- --- --- --- UPDATE --- --- --- --- --- 
    def update_pybullet(self):
        for b in self.buttons:
            b.update()
        
        # update robot state
        for i, hi in enumerate(self.huskyInterfaces):
            self.huskyModels[i].set_pose((hi.position, hi.rotation), hi.arm_joint_states)
        
        # update goal robot state
        state_slider_values = [p.readUserDebugParameter(ps) for ps in self.state_sliders]
        self.goal_pos = np.array((state_slider_values[0], state_slider_values[1], 0))
        self.goal_rot = R.from_euler("z", state_slider_values[2], degrees=False)
        self.goal_arm_pose = np.array([p.readUserDebugParameter(ps) for ps in self.joint_state_sliders])
            
        preview_time = p.readUserDebugParameter(self.time_slider)
        goal_pose = (self.goal_pos, self.goal_rot.as_quat())
        goal_arm_pose = self.goal_arm_pose
        if not np.isclose(preview_time, 1.0):
            if self.planned_base_trajectory:
                base_traj_idx = int(preview_time * (len(self.planned_base_trajectory) - 1))
                goal_pose = self.planned_base_trajectory[base_traj_idx]
            if self.planned_arm_trajectory:
                arm_traj_idx = int(preview_time * (len(self.planned_arm_trajectory) - 1))
                goal_arm_pose = self.planned_arm_trajectory[arm_traj_idx]
            
        self.goalModel.set_pose(goal_pose, goal_arm_pose)
        
        # run tasks
        i = 0
        while i < len(self.tasks):
            task = self.tasks[i]
            if task():
                i += 1
            else:
                self.tasks.remove(task)
                


# --- --- --- --- --- MAIN --- --- --- --- --- 
def main(args=None):
    rclpy.init(args=args)

    husky_monitor = HuskyMonitor()

    rclpy.spin(husky_monitor)

    husky_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':     
    main()
