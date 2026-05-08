"""
The husky robot inteface handling:

- ROS2 communication
- State estimation
"""

import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import pybullet_planning as pp

# ROS
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.client import Client

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
from control_msgs.msg._dynamic_joint_state import DynamicJointState
from control_msgs.msg._interface_value import InterfaceValue
from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from ur_msgs.msg._io_states import IOStates
from std_srvs.srv._trigger import Trigger
from geometry_msgs.msg import PoseStamped, Point, Quaternion, WrenchStamped
from crl_husky_msgs.msg import MultiArmTrajectory
from ur_msgs.srv._set_force_mode import SetForceMode
from ur_msgs.msg._io_states import IOStates
from std_srvs.srv._trigger import Trigger

# Controller Manager
from controller_manager_msgs.srv._switch_controller import SwitchController

# SetIO service for gripper and screw control
from ur_msgs.srv import SetIO

# Controller Manager
from controller_manager_msgs.srv._switch_controller import SwitchController

# Scaffolding tool RS485 driver client (replaces SetIO-based gripper/screw)
from husky_assembly_teleop.scaffolding_tool_client import ScaffoldingToolClient

from rclpy.qos import QoSProfile

UR5e_HOME_STATE = np.array([0, -np.pi/2, 0, -np.pi/2, 0, np.pi/2])
ARM_JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

USE_TRAJECTORY_TOPIC_INTERFACE = 1
ARM_NOT_EXECUTING_TIME = 1

def quaterinion_2_angular_velocity(q1, q2, dt):
    return (2 / dt) * np.array([
        q1[3]*q2[0] - q1[0]*q2[3] - q1[1]*q2[2] + q1[2]*q2[1],
        q1[3]*q2[1] + q1[0]*q2[2] - q1[1]*q2[3] - q1[2]*q2[0],
        q1[3]*q2[2] - q1[0]*q2[1] + q1[1]*q2[0] - q1[2]*q2[3]])

class HuskyRobotInterface:
    position = np.zeros(3)
    rotation = R.as_quat(R.identity())
    
    velocity = np.zeros(3)
    angular_velocity = np.zeros(3)
    
    arm_joint_pose = [UR5e_HOME_STATE]
    arm_tcp_pose = [pp.Pose()] # TODO: This value reported bz UR5e does not correspond to world position nor to local position (relative to base link). Its something weird in between, probably accounting for mounting orientation set in ur5e.
    arm_ft_sensor = [[0, 0, 0, 0, 0, 0]]
    is_arm_executing = [False]
    last_arm_movement = [0]
    io_states = [[False for x in range(0,18)]]
    active_controller = [""]
    
    # Gripper and screw states for toggle functionality
    gripper_states = [False]  # False = open, True = closed
    screw_states = [False]    # False = not actuated, True = actuated
    
    odom_offset = np.zeros(3)
    _odom_position = np.zeros(3)
    
    def __init__(self, node: Node, name='/a200_0804', use_odom=True, connect_arm=True, connect_gripper=True, dual_arm=False):
        self.node = node
        self.name = name
        self.dual_arm = dual_arm
        
        if dual_arm:
            self.arm_joint_pose.append(UR5e_HOME_STATE)
            self.arm_tcp_pose.append(pp.Pose())
            self.arm_ft_sensor.append([0, 0, 0, 0, 0, 0])
            self.is_arm_executing.append(False)
            self.last_arm_movement.append(0)
            self.io_states.append([False for x in range(0,18)])
            self.active_controller.append("")
            self.gripper_states.append(False)
            self.screw_states.append(False)
        
        q = QoSProfile(depth=10)
        print(q)
        
        # Listeners --- --- --- --- ---
        if use_odom:
            self.sub_tf = self.node.create_subscription(
                TFMessage,
                name + '/tf',
                self.tf_callback,
                10)

        self.sub_arms = []
        if dual_arm:
            self.sub_arms.append(self.node.create_subscription(
                JointState,
                name + '/left_ur5e/rate_limiter/joint_states',
                lambda msg: self.arm_callback(0, msg),
                10))
            self.sub_arms.append(self.node.create_subscription(
                JointState,
                name + '/right_ur5e/rate_limiter/joint_states',
                lambda msg: self.arm_callback(1, msg),
                10))
        else:
            self.sub_arms.append(self.node.create_subscription(
                JointState,
                name + '/ur5e/rate_limiter/joint_states',
                lambda msg: self.arm_callback(0, msg),
                10))
            
        self.sub_dynamic_arm = []
        if dual_arm:
            self.sub_dynamic_arm.append(self.node.create_subscription(
                DynamicJointState,
                name + '/left_ur5e/rate_limiter/dynamic_joint_states',
                lambda msg: self.dynamic_arm_callback(0, msg),
                10))
            self.sub_dynamic_arm.append(self.node.create_subscription(
                DynamicJointState,
                name + '/right_ur5e/rate_limiter/dynamic_joint_states',
                lambda msg: self.dynamic_arm_callback(1, msg),
                10))
        else:
            self.sub_dynamic_arm.append(self.node.create_subscription(
                DynamicJointState,
                name + '/ur5e/rate_limiter/dynamic_joint_states',
                lambda msg: self.dynamic_arm_callback(0, msg),
                10))
            
        self.sub_io_states = []
        if dual_arm:
            self.sub_io_states.append(self.node.create_subscription(
                IOStates,
                name + '/left_ur5e/rate_limiter/io_and_status_controller/io_states',
                lambda msg: self.io_state_callback(0, msg),
                10))
            self.sub_io_states.append(self.node.create_subscription(
                IOStates,
                name + '/right_ur5e/rate_limiter/io_and_status_controller/io_states',
                lambda msg: self.io_state_callback(1, msg),
                10))
        else:
            self.sub_io_states.append(self.node.create_subscription(
                IOStates,
                name + '/ur5e/rate_limiter/io_and_status_controller/io_states',
                lambda msg: self.io_state_callback(0, msg),
                10))
            
        self.sub_ft_sensor = []
        if dual_arm:
            self.sub_ft_sensor.append(self.node.create_subscription(
                WrenchStamped,
                name + '/left_ur5e/rate_limiter/ft_sensor_wrench',
                lambda msg: self.ft_sensor_callback(0, msg),
                10))
            self.sub_ft_sensor.append(self.node.create_subscription(
                WrenchStamped,
                name + '/right_ur5e/rate_limiter/ft_sensor_wrench',
                lambda msg: self.ft_sensor_callback(1, msg),
                10))
        else:
            self.sub_ft_sensor.append(self.node.create_subscription(
                WrenchStamped,
                name + '/ur5e/rate_limiter/ft_sensor_wrench',
                lambda msg: self.ft_sensor_callback(0, msg),
                10))

        
        # Publishers --- --- --- --- ---
        self.pub_cmd_vel = self.node.create_publisher(Twist, name + '/cmd_vel', 10)
        self.pub_cmd_arm = []
        if dual_arm:
            self.pub_cmd_arm.append(self.node.create_publisher(JointTrajectory, name + '/left_ur5e/scaled_joint_trajectory_controller/joint_trajectory', 10))
            self.pub_cmd_arm.append(self.node.create_publisher(JointTrajectory, name + '/right_ur5e/scaled_joint_trajectory_controller/joint_trajectory', 10))
            self.pub_cmd_multi_arm = self.node.create_publisher(MultiArmTrajectory, name + '/multi_arm_joint_trajectory', 10)
        else:
            self.pub_cmd_arm.append(self.node.create_publisher(JointTrajectory, name + '/ur5e/scaled_joint_trajectory_controller/joint_trajectory', 10))
        
        self.pub_cmd_arm_cartesian = []
        if dual_arm:
            self.pub_cmd_arm_cartesian.append(self.node.create_publisher(PoseStamped, name + '/left_ur5e/target_frame', 10))
            self.pub_cmd_arm_cartesian.append(self.node.create_publisher(PoseStamped, name + '/right_ur5e/target_frame', 10))
        else:
            self.pub_cmd_arm_cartesian.append(self.node.create_publisher(PoseStamped, name + '/ur5e/target_frame', 10))
            
        self.pub_cmd_arm_cartesian_force = []
        if dual_arm:
            self.pub_cmd_arm_cartesian_force.append(self.node.create_publisher(WrenchStamped, name + '/left_ur5e/target_wrench', 10))
            self.pub_cmd_arm_cartesian_force.append(self.node.create_publisher(WrenchStamped, name + '/right_ur5e/target_wrench', 10))
        else:
            self.pub_cmd_arm_cartesian_force.append(self.node.create_publisher(WrenchStamped, name + '/ur5e/target_wrench', 10))
        
        # Service Clients
        
        self.force_services = []
        if dual_arm:
            self.force_services.append(node.create_client(SetForceMode, name + '/left_ur5e/force_mode_controller/start_force_mode'))
            self.force_services.append(node.create_client(SetForceMode, name + '/right_ur5e/force_mode_controller/start_force_mode'))
        else:
            self.force_services.append(node.create_client(SetForceMode, name + '/ur5e/force_mode_controller/start_force_mode'))
        
        for fs in self.force_services:
            fs.wait_for_service(timeout_sec=2.5)
            self.node.get_logger().info(f'Force Service Client {fs.service_is_ready()}')
            
        self.zero_ft_sensor_client = []
        if dual_arm:
            self.zero_ft_sensor_client.append(node.create_client(Trigger, name + '/left_ur5e/io_and_status_controller/zero_ftsensor'))
            self.zero_ft_sensor_client.append(node.create_client(Trigger, name + '/right_ur5e/io_and_status_controller/zero_ftsensor'))
        else:
            self.zero_ft_sensor_client.append(node.create_client(Trigger, name + '/ur5e/io_and_status_controller/zero_ftsensor')) 
        
        for fs in self.zero_ft_sensor_client:
            fs.wait_for_service(timeout_sec=2.5)
            self.node.get_logger().info(f'Zero FT Sensor Service Client {fs.service_is_ready()}')
            
        self.controller_change_service_client = []
        if dual_arm:
            self.controller_change_service_client.append(node.create_client(SwitchController, name + '/left_ur5e/controller_manager/switch_controller'))
            self.controller_change_service_client.append(node.create_client(SwitchController, name + '/right_ur5e/controller_manager/switch_controller'))
        else:
            self.controller_change_service_client.append(node.create_client(SwitchController, name + '/ur5e/controller_manager/switch_controller')) 
        
        for fs in self.controller_change_service_client:
            fs.wait_for_service(timeout_sec=2.5)
            self.node.get_logger().info(f'Switch Controller Service Client {fs.service_is_ready()}')

        
        # Action Clients
        # TODO support dual arm
        self.act_grippers = []
        if dual_arm:
            self.act_grippers.append(ActionClient(
                self.node,
                GripperCommand,
                name + '/left_gripper/robotiq_gripper_controller/gripper_cmd',
            ))
            self.act_grippers.append(ActionClient(
                self.node,
                GripperCommand,
                name + '/right_gripper/robotiq_gripper_controller/gripper_cmd',
            ))
        else:
            self.act_grippers.append(ActionClient(
                self.node,
                GripperCommand,
                name + '/gripper/robotiq_gripper_controller/gripper_cmd',
            ))
    
        if connect_gripper:
            for act_gripper in self.act_grippers:
                act_gripper.wait_for_server(timeout_sec=2.5)
                self.node.get_logger().info(f'Gripper Action Server {act_gripper.server_is_ready()}')
        
        self.act_arms = []
        if dual_arm:
            self.act_arms.append(ActionClient(
                self.node,
                FollowJointTrajectory,
                name + '/left_ur5e/scaled_joint_trajectory_controller/follow_joint_trajectory',
            ))
            self.act_arms.append(ActionClient(
                self.node,
                FollowJointTrajectory,
                name + '/right_ur5e/scaled_joint_trajectory_controller/follow_joint_trajectory',
            ))
        else:
            self.act_arms.append(ActionClient(
                self.node,
                FollowJointTrajectory,
                name + '/ur5e/scaled_joint_trajectory_controller/follow_joint_trajectory',
            ))
        if connect_arm and not USE_TRAJECTORY_TOPIC_INTERFACE:
            for act_arm in self.act_arms:
                act_arm.wait_for_server(timeout_sec=2.5)
                self.node.get_logger().info(f'Arm Action Server {act_arm.server_is_ready()}')
        
        # SetIO Service Clients for gripper and screw control
        self.setio_clients = []
        if dual_arm:
            self.setio_clients.append(self.node.create_client(SetIO, name + '/left_ur5e/io_and_status_controller/set_io'))
            self.setio_clients.append(self.node.create_client(SetIO, name + '/right_ur5e/io_and_status_controller/set_io'))
        else:
            self.setio_clients.append(self.node.create_client(SetIO, name + '/ur5e/io_and_status_controller/set_io'))
        
        # Wait for SetIO services to be available
        for i, client in enumerate(self.setio_clients):
            if client.wait_for_service(timeout_sec=2.5):
                self.node.get_logger().info(f'SetIO Service {i} is ready!')
            else:
                self.node.get_logger().warn(f'SetIO Service {i} not available!')

        # Zero FT Sensor service clients
        self.zero_ft_sensor_client = []
        if dual_arm:
            self.zero_ft_sensor_client.append(self.node.create_client(Trigger, name + '/left_ur5e/io_and_status_controller/zero_ftsensor'))
            self.zero_ft_sensor_client.append(self.node.create_client(Trigger, name + '/right_ur5e/io_and_status_controller/zero_ftsensor'))
        else:
            self.zero_ft_sensor_client.append(self.node.create_client(Trigger, name + '/ur5e/io_and_status_controller/zero_ftsensor'))

        for fs in self.zero_ft_sensor_client:
            fs.wait_for_service(timeout_sec=2.5)
            self.node.get_logger().info(f'Zero FT Sensor Service Client {fs.service_is_ready()}')

        # Switch Controller service clients
        self.controller_change_service_client = []
        if dual_arm:
            self.controller_change_service_client.append(self.node.create_client(SwitchController, name + '/left_ur5e/controller_manager/switch_controller'))
            self.controller_change_service_client.append(self.node.create_client(SwitchController, name + '/right_ur5e/controller_manager/switch_controller'))
        else:
            self.controller_change_service_client.append(self.node.create_client(SwitchController, name + '/ur5e/controller_manager/switch_controller'))

        for fs in self.controller_change_service_client:
            fs.wait_for_service(timeout_sec=2.5)
            self.node.get_logger().info(f'Switch Controller Service Client {fs.service_is_ready()}')

        # Scaffolding tool RS485 clients (replaces SetIO-based gripper/screw control).
        # Indexing matches setio_clients: 0 = left/single, 1 = right.
        self.tool_clients = []
        if dual_arm:
            self.tool_clients.append(ScaffoldingToolClient(node, name, 'left_tool'))
            self.tool_clients.append(ScaffoldingToolClient(node, name, 'right_tool'))
        else:
            self.tool_clients.append(ScaffoldingToolClient(node, name, 'tool'))

        # done --- --- --- --- ---
        self.node.get_logger().info(f'Husky "{name}" is ready!')

    def switch_controller(self, from_ctrl, to_ctrl, arm_index=0):
        msg = SwitchController.Request()
        
        msg.start_asap = True
        msg.deactivate_controllers = [from_ctrl]
        msg.activate_controllers = [to_ctrl]
        msg.strictness = SwitchController.Request.STRICT
        
        print(f"switching from {from_ctrl} to {to_ctrl} on arm {arm_index}")
        fut = self.controller_change_service_client[arm_index].call_async(msg)
        
        def controller_switched_callback(self, new_controller, arm_index, fut):
            msg = fut.result()
            if msg.ok:
                self.active_controller[arm_index] = new_controller
                print(f"Controller switched to {new_controller}")
            else:
                print("Failed to switch controller!")
                
        fut.add_done_callback(lambda fut: controller_switched_callback(self, to_ctrl, arm_index, fut))
        

    def tf_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            header: Header = transform.header
            if header.frame_id == 'odom':
                ts: Transform = transform.transform
                self._odom_position =  np.array((ts.translation.x, ts.translation.y, ts.translation.z))
                self.position = self._odom_position - self.odom_offset
                self.rotation = np.array((ts.rotation.x, ts.rotation.y, ts.rotation.z, ts.rotation.w))
                #self.node.get_logger().info(f'Position {np.around(self.position, decimals=2)}')
    
    def send_base_twist_cmd(self, x_dot, theta_dot):
        msg = Twist()
        msg.linear.x = x_dot
        msg.angular.z = theta_dot
        self.pub_cmd_vel.publish(msg)
    
    _last_mocap_data = 0
    _velocity_samples = []
    _angular_velocity_samples = []
    _velocity_samples_time = []
    velocity_filter_time = 0.2
    def mocap_callback(self, pos, rot, ts):
        dt = ts - self._last_mocap_data
        self._last_mocap_data = ts
        dp = pos - self.position
        
        v = dp / dt
        w = quaterinion_2_angular_velocity(self.rotation, rot, dt)
        
        # drop too old samples
        if len(self._velocity_samples_time) > 0:
            i = 0
            while i < len(self._velocity_samples_time) and ts - self._velocity_samples_time[i] > self.velocity_filter_time:
                i += 1
            self._velocity_samples = self._velocity_samples[i:]
            self._angular_velocity_samples = self._angular_velocity_samples[i:]
            self._velocity_samples_time = self._velocity_samples_time[i:]
            
        # add new sample
        self._velocity_samples.append(v)
        self._angular_velocity_samples.append(w)
        self._velocity_samples_time.append(ts)
        
        # take mean of all samples
        self.velocity = np.mean(self._velocity_samples, axis=0)
        self.angular_velocity = np.mean(self._angular_velocity_samples, axis=0)
        
        self.position = pos
        self.rotation = rot
        
    def arm_callback(self, index, msg: JointState):
        arm_pos = msg.position
        reorder = []
        for name in ARM_JOINT_NAMES:
            reorder.append(msg.name.index(name))
        old_pose = self.arm_joint_pose[index]
        self.arm_joint_pose[index] = np.array(arm_pos)[reorder]
        if USE_TRAJECTORY_TOPIC_INTERFACE == 1:
            if not np.isclose(old_pose, self.arm_joint_pose[index], atol=1e-2).all():
                if not self.is_arm_executing[index]:
                    print(f"{index} STARTED MOVING")
                self.is_arm_executing[index] = True
                self.last_arm_movement[index] = time.time()
            elif time.time() - self.last_arm_movement[index] > ARM_NOT_EXECUTING_TIME:
                if self.is_arm_executing[index]:
                    print(f"{index} FINISHED MOVING")
                self.is_arm_executing[index] = False
    
    def dynamic_arm_callback(self, index, msg: DynamicJointState):
        tcp_interface_value: InterfaceValue = msg.interface_values[msg.joint_names.index("tcp_pose")]
        reorder = []
        for name in ['position.x', 'position.y', 'position.z', 'orientation.x', 'orientation.y', 'orientation.z', 'orientation.w']:
            reorder.append(tcp_interface_value._interface_names.index(name))
        
        correction_transform = pp.Pose(pp.Point(), pp.Euler(0, 0, np.deg2rad(-180 if self.dual_arm else -90)))
        
        data = np.array(tcp_interface_value.values)[reorder]
        self.arm_tcp_pose[index] = pp.multiply(correction_transform, pp.Pose(data[0:3], pp.euler_from_quat(data[3:7])))
        
    def io_state_callback(self, index, msg: IOStates):
        in_states = msg.digital_in_states
        for i in range(0, 18):
            pin = in_states[i].pin
            state = in_states[i].state
            self.io_states[index][pin] = state
            
    def ft_sensor_callback(self, index, msg: WrenchStamped):
        self.arm_ft_sensor[index] = [
                msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z
            ]

    def zero_ft_sensor(self, index=0):
        msg = Trigger.Request()
        self.zero_ft_sensor_client[index].call_async(msg)
        
    def send_force_mode_command(self, force, index=0):
        msg = SetForceMode.Request()
        msg.task_frame.header.frame_id = 'tool0'
        msg.task_frame.pose.position.x = 0.0
        msg.task_frame.pose.position.y = 0.0
        msg.task_frame.pose.position.z = 0.0
        
        msg.task_frame.pose.orientation.x = 0.0
        msg.task_frame.pose.orientation.y = 0.0
        msg.task_frame.pose.orientation.z = 0.0
        msg.task_frame.pose.orientation.w = 1.0
        
        msg.selection_vector_x = False
        msg.selection_vector_y = True
        msg.selection_vector_z = False
        
        msg.selection_vector_rx = False
        msg.selection_vector_ry = False
        msg.selection_vector_rz = False
        
        msg.speed_limits.linear.x = 0.1
        msg.speed_limits.linear.y = 0.1
        msg.speed_limits.linear.z = 0.1
        
        msg.speed_limits.angular.x = 0.1
        msg.speed_limits.angular.y = 0.1
        msg.speed_limits.angular.z = 0.1
        
        msg.type = 2
        
        msg.wrench.force.x = force[0]
        msg.wrench.force.y = force[1]
        msg.wrench.force.z = force[2]
        self.force_services[index].call_async(msg)
        
    def send_arm_cmd_cartesian(self, pose_arm_local, index=0):
        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        
        if (not np.isclose(self.arm_tcp_pose[index][0], pose_arm_local[0], atol=0.05).all()) or (not np.isclose(self.arm_tcp_pose[index][1], pose_arm_local[1], atol=0.05).all()):
            self.node.get_logger().warn(f'Arm {index} of husky {self.name} is not in correct start pose!')
            self.node.get_logger().warn(f'{self.arm_tcp_pose[index]} vs {pose_arm_local}')
            return
        
        point = pp.point_from_pose(pose_arm_local)
        quat = pp.quat_from_pose(pose_arm_local)
        msg.pose.position.x = point[0]
        msg.pose.position.y = point[1]
        msg.pose.position.z = point[2]
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        self.pub_cmd_arm_cartesian[index].publish(msg)
        
    def send_arm_cmd_cartesian_force(self, force_arm_local, index=0):
        msg = WrenchStamped()
        msg.header.frame_id = "base_link"
        
        msg.wrench.force.x = force_arm_local[0]
        msg.wrench.force.y = force_arm_local[1]
        msg.wrench.force.z = force_arm_local[2]
        
        self.pub_cmd_arm_cartesian_force[index].publish(msg)
        
    def send_gripper_cmd(self, pos, effort, index=0):
        goal = GripperCommand.Goal()
        goal.command.position = pos
        goal.command.max_effort = effort
        self.act_grippers[index].send_goal_async(goal)
        
    def send_dual_arm_cmd(self, multi_arm_trajectory):
        # raise NotImplementedError("Multi-arm trajectory control is not implemented in this interface.")

        multitrajectory = MultiArmTrajectory()
        
        multitrajectory.trajectory1 = self.to_trajectory_msg(*multi_arm_trajectory[0][0:3], 0)
        multitrajectory.trajectory2 = self.to_trajectory_msg(*multi_arm_trajectory[1][0:3], 1)
        
        if multitrajectory.trajectory1 is None or multitrajectory.trajectory2 is None:
            return
        
        print("0 SENT MOVING")
        print("1 SENT MOVING")
        # assume execution starts immediately
        self.last_arm_movement[0] = time.time()
        self.is_arm_executing[0] = True 
        self.last_arm_movement[1] = time.time()
        self.is_arm_executing[1] = True
        self.pub_cmd_multi_arm.publish(multitrajectory)
    
    def to_trajectory_msg(self, arm_joint_positions, arm_joint_velocities=None, time=10.0, index=0):
        if arm_joint_velocities is not None:
            if len(arm_joint_positions) != len(arm_joint_velocities):
                self.node.get_logger().error("trajectory must have equal number of position and velocity entries!")
                return None
            
        if not np.isclose(self.arm_joint_pose[index], arm_joint_positions[0], atol=0.1).all():
            self.node.get_logger().warn(f'Arm of husky {self.name} is not in correct start pose!')
            self.node.get_logger().warn(f'{self.arm_joint_pose[index]} vs {arm_joint_positions[0]}')
            return
        
        dt = time / (len(arm_joint_positions) - 1)
        
        trajectory = JointTrajectory()
        trajectory.joint_names = ARM_JOINT_NAMES
        for i, waypoint in enumerate(arm_joint_positions):
            point = JointTrajectoryPoint()
            point.positions = list(waypoint)
            if arm_joint_velocities is not None:
                point.velocities = list(arm_joint_velocities[i])
            time_from_start = dt*i
            sec = np.floor(time_from_start)
            nano = time_from_start - sec
            point.time_from_start = Duration(sec=int(sec), nanosec=int(nano*1e9))
            trajectory.points.append(point)
        
        return trajectory
    
    def send_arm_cmd(self, arm_joint_positions, arm_joint_velocities=None, traj_time=10.0, index=0):
        """
        Send a joint trajectory to the arm
        
        Important: The arm must be in the correct start pose as the first waypoint of the trajectory has timestep 0!
        """
        if arm_joint_velocities is not None:
            if len(arm_joint_positions) != len(arm_joint_velocities):
                self.node.get_logger().error("trajectory must have equal number of position and velocity entries!")
                return
            
        if not np.isclose(self.arm_joint_pose[index], arm_joint_positions[0], atol=0.1).all():
            self.node.get_logger().warn(f'Arm of husky {self.name} is not in correct start pose!')
            self.node.get_logger().warn(f'{self.arm_joint_pose[index]} vs {arm_joint_positions[0]}')
            return
        
        dt = traj_time / (len(arm_joint_positions) - 1)

        print('monitor trajectory time:', traj_time)
        print('dt:', dt)

        # return
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        
        goal.trajectory.joint_names = ARM_JOINT_NAMES
        for i, waypoint in enumerate(arm_joint_positions):
            point = JointTrajectoryPoint()
            point.positions = list(waypoint)
            if arm_joint_velocities is not None:
                point.velocities = list(arm_joint_velocities[i])
            time_from_start = dt*i
            sec = np.floor(time_from_start)
            nano = time_from_start - sec
            point.time_from_start = Duration(sec=int(sec), nanosec=int(nano*1e9))
            goal.trajectory.points.append(point)
  
        goal.path_tolerance = [
            JointTolerance(position=1.0, velocity=1.0, name=joint_name) for joint_name in ARM_JOINT_NAMES
        ]
        goal.goal_time_tolerance = Duration(sec=10, nanosec=0)
        goal.goal_tolerance = [
            JointTolerance(position=0.01, velocity=0.01, name=joint_name) for joint_name in ARM_JOINT_NAMES
        ]
        if USE_TRAJECTORY_TOPIC_INTERFACE:
            # assume execution starts immediately
            print(f"{index} SENT MOVING")
            self.last_arm_movement[index] = time.time()
            self.is_arm_executing[index] = True
            
            self.pub_cmd_arm[index].publish(goal.trajectory)
        else:
            self.is_arm_executing[index] = True
            send_goal_future = self.act_arms[index].send_goal_async(goal)
            send_goal_future.add_done_callback(lambda fut: self.goal_response_callback(index, fut))
    
    def goal_response_callback(self, index, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Goal rejected :(")
            self.is_arm_executing[index] = False
            return

        self.node.get_logger().info("Goal accepted :)")
        self.is_arm_executing[index] = True

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(lambda fut: self.get_result_callback(index, fut))
    
    def get_result_callback(self, index, future):
        result = future.result().result
        status = future.result().status
        self.node.get_logger().info(f"Done with result: {self.status_to_str(status)}")
        self.is_arm_executing[index] = False
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().error(
                f"Done with result: {self.error_code_to_str(result.error_code)}"
            )

    # DEPRECATED: SetIO-based gripper/screw control. Replaced by RS485 tool driver
    # (see self.tool_clients and tighten_tool/loosen_tool/stop_tool below).
    # Kept as no-ops (with a one-time warning) so any leftover callers don't crash.
    _deprecation_warned = {'toggle_gripper': False, 'toggle_screw': False}

    def _warn_deprecated_once(self, name):
        if not HuskyRobotInterface._deprecation_warned.get(name):
            self.node.get_logger().warn(
                f'{name}() is deprecated; use tighten_tool/loosen_tool/stop_tool instead')
            HuskyRobotInterface._deprecation_warned[name] = True

    def toggle_gripper(self, index=0):
        self._warn_deprecated_once('toggle_gripper')

    def set_screw(self, state, index=0):
        """
        Set screw actuation state for the specified arm.
        Uses SetIO service with PIN_TOOL_DOUT0.
        """
        if index >= len(self.setio_clients):
            self.node.get_logger().error(f'Invalid arm index: {index}')
            return

        self.screw_states[index] = state
        new_state = SetIO.Request.STATE_ON if self.screw_states[index] else SetIO.Request.STATE_OFF

        # Create and send the request
        req = SetIO.Request()
        req.fun = SetIO.Request.FUN_SET_DIGITAL_OUT
        req.pin = SetIO.Request.PIN_TOOL_DOUT0
        req.state = float(new_state)

        future = self.setio_clients[index].call_async(req)
        self.node.get_logger().info(f'Screw {index} {"actuated" if self.screw_states[index] else "deactivated"}')

        return future
    
    def set_screw(self, state, index=0):
        """
        Set screw actuation state for the specified arm.
        Uses SetIO service with PIN_TOOL_DOUT0.
        """
        return self.set_screw(not self.screw_states[index], index)

    # ---------- new RS485 scaffolding tool wrappers --------------------------
    def tighten_tool(self, index=0, motor='M1'):
        if index >= len(self.tool_clients):
            self.node.get_logger().error(f'Invalid arm index: {index}')
            return
        
        self.screw_states[index] = state
        new_state = SetIO.Request.STATE_ON if self.screw_states[index] else SetIO.Request.STATE_OFF
        
        # Create and send the request
        req = SetIO.Request()
        req.fun = SetIO.Request.FUN_SET_DIGITAL_OUT
        req.pin = SetIO.Request.PIN_TOOL_DOUT0
        req.state = float(new_state)
        
        future = self.setio_clients[index].call_async(req)
        self.node.get_logger().info(f'Screw {index} {"actuated" if self.screw_states[index] else "deactivated"}')
        
        return future
    
    def toggle_screw(self, index=0):
        """
        Toggle screw actuation state for the specified arm.
        Uses SetIO service with PIN_TOOL_DOUT0.
        """
        return self.set_screw(not self.screw_states[index], index)

    def loosen_tool(self, index=0, motor='M1'):
        if index >= len(self.tool_clients):
            self.node.get_logger().error(f'Invalid arm index: {index}')
            return None
        return self.tool_clients[index].loosen(motor)

    def stop_tool(self, index=0):
        """Stop both motors on the given arm's tool."""
        if index >= len(self.tool_clients):
            self.node.get_logger().error(f'Invalid arm index: {index}')
            return None
        return self.tool_clients[index].stop()

    def stop_all_tools(self):
        """Panic-stop: stop motors on every connected tool."""
        return [c.stop() for c in self.tool_clients]

    def tool_status(self, index=0):
        if index >= len(self.tool_clients):
            return None
        return self.tool_clients[index].status

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