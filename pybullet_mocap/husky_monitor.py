import rclpy
from rclpy.node import Node

import os
import numpy as np
import pybullet_planning as pp
import pybullet as p

from pybullet_mocap.husky_robot import HuskyRobotInterface
from pybullet_mocap.lib import DATA_DIRECTORY

HUSKYU_JOINT_NAMES = [
                    #   'x', 'y', 'theta', 
                      "ur_arm_shoulder_pan_joint", 
                      "ur_arm_shoulder_lift_joint",
                      "ur_arm_elbow_joint", 
                      "ur_arm_wrist_1_joint", 
                      "ur_arm_wrist_2_joint", 
                      "ur_arm_wrist_3_joint" ]

TOOL0_FROM_TIP = pp.Pose(point=(0, 0, 73.65 * 1e-3))

def load_robot(ik_from_arm_base=True, load_calib_tip=False):
    # robot_urdf = os.path.join(DATA_DIRECTORY,'husky_urdf/mt_husky_moveit_config/urdf/husky_ur5_e.urdf')
    robot_urdf = os.path.join(DATA_DIRECTORY,'husky_urdf/mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint.urdf')
    # robot_srdf = os.path.join(DATA_DIRECTORY, 'husky_urdf/mt_husky_moveit_config/config/husky.srdf')

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

    # # Convert your YUP_TFORM to a pose
    # y_up_quaternion = p.getQuaternionFromEuler([-np.pi/2, np.pi/2, 0])
    # base_position, _ = p.getBasePositionAndOrientation(robot)
    # p.resetBasePositionAndOrientation(robot, base_position, y_up_quaternion)

    tool0_pose = pp.get_link_pose(robot, pp.link_from_name(robot, 'ur_arm_tool0'))
    # pp.draw_pose(tool0_pose)
    ee = pp.create_obj(gripper_obj, scale=gripper_scale) 
    pp.set_pose(ee, pp.multiply(tool0_pose, pp.Pose(euler=pp.Euler(yaw=-np.pi/2))))
    
    ee_attachment = pp.create_attachment(robot, pp.link_from_name(robot, 'ur_arm_tool0'), ee)

    # tool0_from_ee = pp.Pose(euler=pp.Euler(yaw=-np.pi/2), point=[0,0,0.138])
    # tcp_pose = pp.multiply(tool0_pose, tool0_from_ee)
    # tcp_pose = pp.get_link_pose(robot, pp.link_from_name(robot, 'central_tcp'))
    # pp.draw_pose(tcp_pose)

    return robot, ee_attachment

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

class HuskyMonitor(Node):
    def __init__(self):
        super().__init__('husky_monitor')
        
        self.husky = HuskyRobotInterface(self)  
        self.timer = self.create_timer(0.05, self.update_pybullet)
        self.start_pybullet()
        
    # --- --- --- --- --- SETUP --- --- --- --- --- 
    def start_pybullet(self):
        # start pybullet simulator
        pp.connect(use_gui=True, shadows=True, color=[0.9, 0.9, 1.0])
        # y-up to be consistent with mocap
        p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP, 1, physicsClientId=pp.CLIENT)
        # turn on the GUI panels
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1, physicsClientId=pp.CLIENT)

        self.param_slider = p.addUserDebugParameter("a slider", 0.0, 1.0, 0.0)
        
        pp.draw_pose(pp.unit_pose(), 0.1)
        with pp.LockRenderer():
            with pp.HideOutput():
                robot, ee_attachment = load_robot(load_calib_tip=True)
            pp.set_color(robot, pp.apply_alpha(pp.GREY, 0.3))
            
            box = pp.create_box(0.25, 0.20, 0.01, color=pp.apply_alpha(pp.GREY, 0.3))
        
        world_from_tool0 = pp.get_link_pose(robot, pp.link_from_name(robot, 'ur_arm_tool0'))
        pts = np.array([[602.47,1047.38,-1019.17],
                    [597.30,997.52,-1012.96],
                    [609.12,1038.14,-1069.54]]) * 1e-3
        world_from_tip = pp.pose_from_tform(create_transformation_matrix(*pts))
        arm_joints = pp.joints_from_names(robot, HUSKYU_JOINT_NAMES)
        
        arm_conf = np.deg2rad(np.array(
            [12.56, -102.02, -74.55, 0.34, 355.99, 77.03]
        ))
        pp.set_joint_positions(robot, arm_joints, arm_conf)
        ee_attachment.assign()
        
        # an example for getting the relative transformation between two links of the robot
        world_from_base_link = pp.get_link_pose(robot, pp.link_from_name(robot, 'world_link'))
        world_from_tool0 = pp.get_link_pose(robot, pp.link_from_name(robot, 'ur_arm_tool0'))

        # arm_base_from_tool0 = pp.get_relative_pose(robot, pp.link_from_name(robot, 'ur_arm_base_link'), pp.link_from_name(robot, 'ur_arm_tool0'))
        tool0_from_base_link = pp.multiply(pp.invert(world_from_tool0), world_from_base_link)
        tip_from_tool0 = pp.invert(TOOL0_FROM_TIP)
        world_from_base_link = pp.multiply(world_from_tip, tip_from_tool0, tool0_from_base_link)

        pp.set_pose(robot, world_from_base_link)
        world_from_tool0 = pp.get_link_pose(robot, pp.link_from_name(robot, 'ur_arm_tool0'))
        ee_attachment.assign()
        #print(world_from_base_link)
        pp.draw_pose(world_from_tool0)
        pp.draw_pose(world_from_tip)
        mocap_pose_training = \
        ((-0.137288898229599, 0.0006755590438842773, -0.7249376177787781), (0.0009945170022547245, -0.5800670385360718, 0.004748028237372637, -0.8145542740821838))
        
        correction_matrix=find_t_bet(pp.tform_from_pose(mocap_pose_training),pp.tform_from_pose(world_from_base_link))
        print(correction_matrix.tolist())
        
    # --- --- --- --- --- UPDATE --- --- --- --- --- 
    def update_pybullet(self):
        p.addUserDebugPoints(pointPositions=[[self.husky.xpos*100,0,0]],pointColorsRGB=[[0.9, 0.9, 0.0]],lifeTime=1.1,pointSize=10)




# --- --- --- --- --- MAIN --- --- --- --- --- 
def main(args=None):
    rclpy.init(args=args)

    husky_monitor = HuskyMonitor()

    rclpy.spin(husky_monitor)

    husky_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':     
    main()
