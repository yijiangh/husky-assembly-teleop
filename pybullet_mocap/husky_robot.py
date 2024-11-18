import time
from rclpy.node import Node
from rclpy.action import ActionClient

# base
from std_msgs.msg._header import Header
from tf2_msgs.msg._tf_message import TFMessage
from geometry_msgs.msg._transform import Transform
from geometry_msgs.msg._twist import Twist
from sensor_msgs.msg._joy import Joy

# gripper
from control_msgs.action._gripper_command import GripperCommand

# arm
from sensor_msgs.msg._joint_state import JointState
from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance

import numpy as np  
import pybullet as p
from scipy.spatial.transform import Rotation as R

UR5e_HOME_STATE = np.array([0, -np.pi/2, 0, -np.pi/2, 0, 0])
ARM_JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
#ARM_JOINT_NAMES = ['/a200_0804/ur5e/' + x for x in ARM_JOINT_NAMES]
#ARM_JOINT_NAMES = ['/' + x for x in ARM_JOINT_NAMES]


class HuskyRobotInterface:
    position = np.zeros(3)
    raw_odom_position = np.zeros(3)
    rotation = R.as_quat(R.identity())
    
    odom_offset = np.zeros(3)
    
    arm_joint_states = UR5e_HOME_STATE
    
    def __init__(self, node: Node, name='/a200_0804'):
        self.node = node
        
        # Listeners --- --- --- --- ---
        self.sub_tf = self.node.create_subscription(
            TFMessage,
            name + '/tf',
            self.tf_callback,
            10)
        
        self.sub_joy = self.node.create_subscription(
            Joy,
            name + '/joy_teleop/joy',
            self.joy_callback,
            10)
        
        self.seb_arm = self.node.create_subscription(
            JointState,
            name + '/ur5e/joint_states',
            self.arm_callback,
            10)
        
        # Publishers --- --- --- --- ---
        self.pub_cmd_vel = self.node.create_publisher(Twist, name + '/cmd_vel', 10)
        
        # Action Clients
        self.act_gripper = ActionClient(
            self.node,
            GripperCommand,
            name + '/gripper/robotiq_gripper_controller/gripper_cmd',
        )
        self.act_gripper.wait_for_server(timeout_sec=2.5)
        self.node.get_logger().info(f'Gripper Action Server {self.act_gripper.server_is_ready()}')
        
        self.act_arm = ActionClient(
            self.node,
            FollowJointTrajectory,
            name + '/ur5e/scaled_joint_trajectory_controller/follow_joint_trajectory',
        )
        self.act_arm.wait_for_server(timeout_sec=2.5)
        self.node.get_logger().info(f'Arm Action Server {self.act_arm.server_is_ready()}')
        
        # done --- --- --- --- ---
        self.node.get_logger().info(f'Husky Monitor startet on "{name}"!')

    def tf_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            header: Header = transform.header
            if header.frame_id == 'odom':
                ts: Transform = transform.transform
                self.raw_odom_position =  np.array((ts.translation.x, ts.translation.y, ts.translation.z))
                self.position = self.raw_odom_position - self.odom_offset
                self.rotation = np.array((ts.rotation.x, ts.rotation.y, ts.rotation.z, ts.rotation.w))
                #self.node.get_logger().info(f'Position {np.around(self.position, decimals=2)}')
    
    # for debugging intermittent joy control
    # joy node sometimes periodically sends zero values even tough stick is held continuously...
    def joy_callback(self, msg: Joy):
        pass
        #self.node.get_logger().info(f'Velocity {msg}')
        
    def arm_callback(self, msg: JointState):
        arm_pos = msg.position
        reorder = []
        for name in ARM_JOINT_NAMES:
            reorder.append(msg.name.index(name))
        self.arm_joint_states = np.array(arm_pos)[reorder]
    
    def send_base_twist_cmd(self, x_dot, theta_dot):
        msg = Twist()
        msg.linear.x = x_dot
        msg.angular.z = theta_dot
        self.pub_cmd_vel.publish(msg)
        
    def send_gripper_cmd(self, pos, effort):
        goal = GripperCommand.Goal()
        goal.command.position = pos
        goal.command.max_effort = effort
        self.act_gripper.send_goal_async(goal)
    
    def send_arm_cmd(self, arm_joint_positions, dt=10):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        
        goal.trajectory.joint_names = ARM_JOINT_NAMES
        for i, waypoint in enumerate(arm_joint_positions):
            point = JointTrajectoryPoint()
            point.positions = list(waypoint)
            #point.velocities = [0., 0., 0., 0., 0., 0.]
            time_from_start = dt*(i+1)
            sec = np.floor(time_from_start)
            nano = time_from_start - sec
            point.time_from_start = Duration(sec=int(sec), nanosec=int(nano*1000000))
            goal.trajectory.points.append(point)

        goal.goal_time_tolerance = Duration(sec=0, nanosec=500000000)
        goal.goal_tolerance = [
            JointTolerance(position=0.01, velocity=0.01, name=joint_name) for joint_name in ARM_JOINT_NAMES
        ]
        send_goal_future = self.act_arm.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Goal rejected :(")
            return

        self.node.get_logger().info("Goal accepted :)")

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.node.get_logger().info(f"Done with result: {self.status_to_str(status)}")
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().error(
                f"Done with result: {self.error_code_to_str(result.error_code)}"
            )

    @staticmethod
    def error_code_to_str(error_code):
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return "SUCCESSFUL"
        if error_code == FollowJointTrajectory.Result.INVALID_GOAL:
            return "INVALID_GOAL"
        if error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
            return "INVALID_JOINTS"
        if error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
            return "OLD_HEADER_TIMESTAMP"
        if error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
            return "PATH_TOLERANCE_VIOLATED"
        if error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            return "GOAL_TOLERANCE_VIOLATED"

    @staticmethod
    def status_to_str(error_code):
        if error_code == GoalStatus.STATUS_UNKNOWN:
            return "UNKNOWN"
        if error_code == GoalStatus.STATUS_ACCEPTED:
            return "ACCEPTED"
        if error_code == GoalStatus.STATUS_EXECUTING:
            return "EXECUTING"
        if error_code == GoalStatus.STATUS_CANCELING:
            return "CANCELING"
        if error_code == GoalStatus.STATUS_SUCCEEDED:
            return "SUCCEEDED"
        if error_code == GoalStatus.STATUS_CANCELED:
            return "CANCELED"
        if error_code == GoalStatus.STATUS_ABORTED:
            return "ABORTED"