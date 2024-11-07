from pybullet_mocap.utils import plan_transit_motion
import rclpy
from rclpy.node import Node

import os
import numpy as np
import pybullet_planning as pp
import pybullet as p
from scipy.spatial.transform import Rotation as R

from pybullet_mocap.husky_robot import HuskyRobotInterface
from pybullet_mocap import DATA_DIRECTORY

HUSKY_JOINT_NAMES = ['x', 'y', 'theta']
HUSKY_UR5e_JOINT_NAMES = ["ur_arm_shoulder_pan_joint", 
                      "ur_arm_shoulder_lift_joint",
                      "ur_arm_elbow_joint", 
                      "ur_arm_wrist_1_joint", 
                      "ur_arm_wrist_2_joint", 
                      "ur_arm_wrist_3_joint" ]

TOOL0_FROM_TIP = pp.Pose(point=(0, 0, 73.65 * 1e-3))

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

    return robot, ee, ee_attachment

def create_transformation_matrix(p0, p1, p2):
    p0 = np.array(p0)
    p1 = np.array(p1)
    p2 = np.array(p2)
    x = p1 - p0  # Direction along what you define as the X-axis
    y = p2 - p0  # Direction along what you define as the Z-axis
    x_normalized = x / np.linalg.norm(x)
    y_normalized = y / np.linalg.norm(y)
    z = np.cross(x_normalized, y_normalized)  # This ensures Y is up and the system remains right-handed
    z_normalized = z / np.linalg.norm(z)
    T = np.eye(4)
    T[:3, 0] = x_normalized
    T[:3, 1] = y_normalized
    T[:3, 2] = z_normalized
    #T[:3, 3] = pp.tform_point(ZUP_FROM_YUP,p0)
    T[:3, 3] = p0
    return T

# Calculate transformation from T_A to T_B
def find_t_bet(T_A, T_B):
    # Calculate the inverse of T_A
    T_A_inv = np.linalg.inv(T_A)
    # Correct order to find transformation from A to B
    T_A_to_B = np.dot(T_A_inv, T_B)
    return T_A_to_B

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
        
        self.husky = HuskyRobotInterface(self)  
        self.timer = self.create_timer(0.05, self.update_pybullet)
        
        self.buttons = []
        self.state_sliders = []
        self.planned_trajectory = None
        self.start_pybullet()
    
    def reset_odom(self):
        self.husky.odom_offset = self.husky.raw_position
        
    def plan(self):
        self.planned_trajectory = plan_transit_motion(
                    self.robot,
                    current_joint_slider_values,
                    [self.ee_attachment],
                    [],
                    debug=False,
                    disabled_collisions=False,
                )
    
    # --- --- --- --- --- SETUP --- --- --- --- --- 
    def start_pybullet(self):
        # start pybullet simulator
        pp.connect(use_gui=True, shadows=True, color=[0.9, 0.9, 1.0])
        # turn on the GUI panels
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1, physicsClientId=pp.CLIENT)
        
        # Build UI
        self.buttons.append(Button('Reset Odom', self.reset_odom))
        
        self.state_sliders.append(p.addUserDebugParameter("x", -5.0, 5.0, 0))
        self.state_sliders.append(p.addUserDebugParameter("x", -5.0, 5.0, 0))
        self.state_sliders.append(p.addUserDebugParameter("yaw", -np.pi, np.pi, 0))
        
        # draw world frame
        pp.draw_pose(pp.unit_pose(), 0.1)
        
        # load robot models
        with pp.LockRenderer():
            with pp.HideOutput():
                self.robot, self.ee, self.ee_attachment = load_robot(load_calib_tip=False)
                
                self.goal_robot, self.goal_ee, self.goal_ee_attachment = load_robot(load_calib_tip=False)
                pp.set_color(self.goal_robot, [0, 0.2, 0.5, 0.7])
        
    # --- --- --- --- --- UPDATE --- --- --- --- --- 
    def update_pybullet(self):
        for b in self.buttons:
            b.update()
        
        # update robot state
        pp.set_pose(self.robot, (self.husky.position, self.husky.rotation))
        arm_joints = pp.joints_from_names(self.robot, HUSKY_UR5e_JOINT_NAMES)
        pp.set_joint_positions(self.robot, arm_joints, self.husky.arm)
        self.ee_attachment.assign()
        
        # update goal robot state
        state_slider_values = [p.readUserDebugParameter(ps) for ps in self.state_sliders]
        self.goal_pos = np.array((state_slider_values[0], state_slider_values[1], 0))
        self.goal_rot = R.from_euler("z", state_slider_values[2], degrees=False)
        pp.set_pose(self.goal_robot, (self.goal_pos, self.goal_rot.as_quat()))
        self.goal_ee_attachment.assign()


# --- --- --- --- --- MAIN --- --- --- --- --- 
def main(args=None):
    rclpy.init(args=args)

    husky_monitor = HuskyMonitor()

    rclpy.spin(husky_monitor)

    husky_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':     
    main()
