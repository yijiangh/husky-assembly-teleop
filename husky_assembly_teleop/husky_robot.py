"""
The husky robot inteface handling:

- ROS2 communication
- State estimation
"""

import time
import numpy as np  
from scipy.spatial.transform import Rotation as R

# ROS
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
try:
    from crl_husky_msgs.msg import MultiArmTrajectory
except ImportError:
    print("MultiArmTrajectory not found, using single arm interface only.")

from rclpy.qos import QoSProfile

UR5e_HOME_STATE = np.array([0, -np.pi/2, 0, -np.pi/2, 0, np.pi/2])
ARM_JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

USE_TRAJECTORY_TOPIC_INTERFACE = 0
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
    is_arm_executing = [False]
    last_arm_movement = [0]
    
    odom_offset = np.zeros(3)
    _odom_position = np.zeros(3)
    
    def __init__(self, node: Node, name='/a200_0804', use_odom=True, connect_arm=True, connect_gripper=True, dual_arm=False):
        self.node = node
        self.name = name
        self.dual_arm = dual_arm
        
        if dual_arm:
            self.arm_joint_pose.append(UR5e_HOME_STATE)
            self.is_arm_executing.append(False)
            self.last_arm_movement.append(0)
        
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
                name + '/left_ur5e/joint_states',
                lambda msg: self.arm_callback(0, msg),
                10))
            self.sub_arms.append(self.node.create_subscription(
                JointState,
                name + '/right_ur5e/joint_states',
                lambda msg: self.arm_callback(1, msg),
                10))
        else:
            self.sub_arms.append(self.node.create_subscription(
                JointState,
                name + '/ur5e/joint_states',
                lambda msg: self.arm_callback(0, msg),
                10))
        
        # Publishers --- --- --- --- ---
        self.pub_cmd_vel = self.node.create_publisher(Twist, name + '/cmd_vel', 10)
        self.pub_cmd_arm = []
        if dual_arm:
            self.pub_cmd_arm.append(self.node.create_publisher(JointTrajectory, name + '/left_ur5e/scaled_joint_trajectory_controller/joint_trajectory', 10))
            self.pub_cmd_arm.append(self.node.create_publisher(JointTrajectory, name + '/right_ur5e/scaled_joint_trajectory_controller/joint_trajectory', 10))
            # self.pub_cmd_multi_arm = self.node.create_publisher(MultiArmTrajectory, name + '/multi_arm_joint_trajectory', 10)
        else:
            self.pub_cmd_arm.append(self.node.create_publisher(JointTrajectory, name + '/ur5e/scaled_joint_trajectory_controller/joint_trajectory', 10))
        
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
        
        # done --- --- --- --- ---
        self.node.get_logger().info(f'Husky "{name}" is ready!')

    def tf_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            header: Header = transform.header
            if header.frame_id == 'odom':
                ts: Transform = transform.transform
                self._odom_position =  np.array((ts.translation.x, ts.translation.y, ts.translation.z))
                self.position = self._odom_position - self.odom_offset
                self.rotation = np.array((ts.rotation.x, ts.rotation.y, ts.rotation.z, ts.rotation.w))
                #self.node.get_logger().info(f'Position {np.around(self.position, decimals=2)}')
    
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
        if not np.isclose(old_pose, self.arm_joint_pose[index], atol=1e-2).all():
            if not self.is_arm_executing[index]:
                print(f"{index} STARTED MOVING")
            self.is_arm_executing[index] = True
            self.last_arm_movement[index] = time.time()
        elif time.time() - self.last_arm_movement[index] > ARM_NOT_EXECUTING_TIME:
            if self.is_arm_executing[index]:
                print(f"{index} FINISHED MOVING")
            self.is_arm_executing[index] = False
    
    def send_base_twist_cmd(self, x_dot, theta_dot):
        msg = Twist()
        msg.linear.x = x_dot
        msg.angular.z = theta_dot
        self.pub_cmd_vel.publish(msg)
        
    def send_gripper_cmd(self, pos, effort, index=0):
        goal = GripperCommand.Goal()
        goal.command.position = pos
        goal.command.max_effort = effort
        self.act_grippers[index].send_goal_async(goal)
        
    def send_dual_arm_cmd(self, multi_arm_trajectory):
        raise NotImplementedError("Multi-arm trajectory control is not implemented in this interface.")

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