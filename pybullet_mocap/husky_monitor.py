import rclpy
from rclpy.node import Node

import os
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
        
        self.huskyInterfaces = [HuskyRobotInterface(self, name='/a200_0804'), HuskyRobotInterface(self, name='/a200_0805')]
        self.timer = self.create_timer(0.05, self.update_pybullet)
        
        self.huskyModels = []
        
        self.buttons = []
        self.state_sliders = []
        self.joint_state_sliders = []
        self.planned_arm_trajectory = None
        self.planned_base_trajectory = None
        self.goal_pos = np.zeros(3)
        self.goal_rot = R.identity()
        self.goal_arm_pose = np.zeros(6)
        self.start_pybullet()
    
    def reset_odom(self):
        for i, hi in enumerate(self.huskyInterfaces):
            hi.odom_offset = ROBOT_START_POS[i] + hi.raw_odom_position
    
    def plan(self):
        joint_state_slider_values = np.array([p.readUserDebugParameter(ps) for ps in self.joint_state_sliders])
        self.planned_arm_trajectory = plan_transit_motion(
                    self.huskyModels.robot,
                    joint_state_slider_values,
                    [self.huskyModels.ee_attachment],
                    [],
                    debug=True,
                    disabled_collisions=False,
                )
        
        print(self.planned_arm_trajectory)
        
        x_range = (-3, 3)
        y_range = (-3, 3)
        
        ob_x_list = [np.inf] # what is this?
        ob_y_list = [np.inf]
        
        rrt_star = RRTStar(
                    0.2, *x_range, *y_range, robot_size=0.1, avoid_dist=0.25
                )
        start_point, start_ori = pp.get_pose(self.huskyModels.robot)
        start_pose = (
            start_point[0],
            start_point[1],
            R.from_quat(start_ori).as_euler("zyx")[0],
        )
        goal_point, goal_ori = pp.get_pose(self.goalModel.robot)
        goal_pose = (
            goal_point[0],
            goal_point[1],
            R.from_quat(goal_ori).as_euler("zyx")[0],
        )
        x_list, y_list = rrt_star.plan(
                    ob_x_list, ob_y_list, *(start_pose[:2]), *(goal_pose[:2])
                )
        yaw_list = fill_yaw_angle(start_pose[-1], goal_pose[-1], x_list, y_list)
        targets = [
                    State(x, y, yaw)
                    for x, y, yaw in zip(x_list, y_list, yaw_list)
                ]
        
        points = [(x, y, 0.0) for x, y in zip(x_list, y_list)]
        with pp.LockRenderer():
            pp.add_segments(points)
            
        controller = Stanley(
                targets[0],
                targets,
                dt=0.1,
                max_steps=2000,
                switch_distance=0.2,
                max_velocity=0.2,
                max_angle_velocity=np.pi,
                stanley_gain=0.75,
                position_tolerance=0.05,
                yaw_tolerance=0.1,
            )
        
        x_list_ctrl, y_list_ctrl, yaw_list_ctrl = controller.run()
        self.planned_base_trajectory = [
                State(x, y, yaw)
                for x, y, yaw in zip(x_list_ctrl, y_list_ctrl, yaw_list_ctrl)
            ]
        
        points = [(x, y, 0.0) for x, y in zip(x_list_ctrl, y_list_ctrl)]
        with pp.LockRenderer():
            pp.add_segments(points, color=RED)

    
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
        
        N = 10
        traj = [np.array([0, -np.pi/2, 0, -np.pi/2, np.sin((x+1)/N*2*np.pi), 0]) for x in range(N)]
        for hi in self.huskyInterfaces:
            hi.send_arm_cmd(traj, dt=10/N)
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
    
    def build_ui(self):
        self.time_slider = p.addUserDebugParameter("time", 0.0, 1.0, 1.0)
        
        self.buttons.append(Button('Reset Odom', self.reset_odom))
        self.buttons.append(Button('Reset Goal State', self.reset_ui))
        
        self.state_sliders.append(p.addUserDebugParameter("x", -5.0, 5.0, 0))
        self.state_sliders.append(p.addUserDebugParameter("x", -5.0, 5.0, 0))
        self.state_sliders.append(p.addUserDebugParameter("yaw", -np.pi, np.pi, 0))
        
        self.buttons.append(Button('Plan', self.plan))
        self.buttons.append(Button('Send Motion Command', self.send_motion_command))
        
        for i, j in enumerate(pp.joints_from_names(self.huskyModels[0].robot, HUSKY_UR5e_JOINT_NAMES)):
            lower, upper = pp.get_joint_limits(self.huskyModels[0].robot, j)
            self.joint_state_sliders.append(p.addUserDebugParameter(f'Joint {i}', lower, upper, self.huskyInterfaces[0].arm_joint_states[i]))
            
        self.buttons.append(Button('Send Arm Command', self.send_arm_command))
        self.buttons.append(Button('Send Arm Wave', self.send_arm_wave))

        self.gripper_slider = p.addUserDebugParameter("gripper", 0, 1.0)
        self.buttons.append(Button('Gripper Set', self.send_gripper_command))
    
    # --- --- --- --- --- SETUP --- --- --- --- --- 
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
        if self.planned_base_trajectory is None or np.isclose(preview_time, 1.0):
            self.goalModel.set_pose((self.goal_pos, self.goal_rot.as_quat()), self.goal_arm_pose)
        elif self.planned_base_trajectory is not None:
            arm_traj_idx = int(preview_time * (len(self.planned_arm_trajectory) - 1))
            arm_pose = self.planned_arm_trajectory[arm_traj_idx]
            
            base_traj_idx = int(preview_time * (len(self.planned_base_trajectory) - 1))
            pos = (self.planned_base_trajectory[base_traj_idx].x, self.planned_base_trajectory[base_traj_idx].y, 0.0)
            rot = R.from_euler("z", self.planned_base_trajectory[base_traj_idx].yaw, degrees=False)
            
            self.goalModel.set_pose((pos, rot.as_quat()), arm_pose)
            
        
        # arm test TODO
        #self.huskyModels.set_pose((self.goal_pos, self.goal_rot.as_quat()), self.huskyInterfaces[0].arm_joint_states)


# --- --- --- --- --- MAIN --- --- --- --- --- 
def main(args=None):
    rclpy.init(args=args)

    husky_monitor = HuskyMonitor()

    rclpy.spin(husky_monitor)

    husky_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':     
    main()
