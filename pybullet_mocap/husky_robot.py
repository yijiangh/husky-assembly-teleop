import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from tf2_msgs.msg._tf_message import TFMessage
from geometry_msgs.msg._transform import Transform
from geometry_msgs.msg._twist import Twist
from sensor_msgs.msg._joy import Joy

import numpy as np
import pybullet_mocap.husky_robot as husky_robot

class HuskyRobotInterface(Node):
    def __init__(self, name='/a200_0804'):
        super().__init__('husky_robot_interface')
        
        # Listeners --- --- --- --- ---
        self.sub_tf = self.create_subscription(
            TFMessage,
            name + '/tf',
            self.tf_callback,
            10)
        
        self.sub_joy = self.create_subscription(
            Joy,
            name + '/joy_teleop/joy',
            self.joy_callback,
            10)
        
        # Publishers --- --- --- --- ---
        self.pub_cmd_vel = self.create_publisher(Twist, name + '/cmd_vel', 10)
        
        # timers --- --- --- --- ---
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        # done --- --- --- --- ---
        self.get_logger().info(f'Husky Monitor startet on "{name}"!')
        
        husky_robot.do()

    def tf_callback(self, msg: TFMessage):
        ts: Transform = msg.transforms[0].transform
        pos = np.array((ts.translation.x, ts.translation.y, ts.translation.z))
        self.get_logger().info(f'Position {np.around(pos, decimals=2)}')
    
    # for debugging intermittent joy control
    # joy node sometimes periodically sends zero values even tough stick is held continuously...
    def joy_callback(self, msg: Joy):
        pass
        #self.get_logger().info(f'Velocity {msg}')
    
    def timer_callback(self):
        pass
        #self.send_cmd_vel(0.0, 0.1)
        
    def send_cmd_vel(self, x_dot, theta_dot):
        msg = Twist()
        msg.linear.x = x_dot
        msg.angular.z = theta_dot
        self.pub_cmd_vel.publish(msg)