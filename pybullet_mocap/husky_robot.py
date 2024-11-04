from rclpy.node import Node

from std_msgs.msg._header import Header
from tf2_msgs.msg._tf_message import TFMessage
from geometry_msgs.msg._transform import Transform
from geometry_msgs.msg._twist import Twist
from sensor_msgs.msg._joy import Joy

import numpy as np

class HuskyRobotInterface:
    position = np.zeros(3)
    rotation = np.zeros(4)
    
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
        
        # Publishers --- --- --- --- ---
        self.pub_cmd_vel = self.node.create_publisher(Twist, name + '/cmd_vel', 10)
        
        # done --- --- --- --- ---
        self.node.get_logger().info(f'Husky Monitor startet on "{name}"!')

    def tf_callback(self, msg: TFMessage):
        header: Header = msg.transforms[0].header
        if header.frame_id == 'odom':
            ts: Transform = msg.transforms[0].transform
            self.position = np.array((ts.translation.x, ts.translation.y, ts.translation.z))
            self.rotation = np.array((ts.rotation.x, ts.rotation.y, ts.rotation.z, ts.rotation.w))
            #self.get_logger().info(f'Position {np.around(pos, decimals=2)}')
    
    # for debugging intermittent joy control
    # joy node sometimes periodically sends zero values even tough stick is held continuously...
    def joy_callback(self, msg: Joy):
        pass
        #self.node.get_logger().info(f'Velocity {msg}')
        
    def send_cmd_vel(self, x_dot, theta_dot):
        msg = Twist()
        msg.linear.x = x_dot
        msg.angular.z = theta_dot
        self.pub_cmd_vel.publish(msg)