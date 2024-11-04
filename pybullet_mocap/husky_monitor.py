import rclpy
from rclpy.node import Node

import numpy as np
import pybullet_planning as pp
import pybullet as p

from pybullet_mocap.husky_robot import HuskyRobotInterface
from pybullet_mocap.lib import DATA_DIRECTORY
    
class HuskyMonitor(Node):
    def __init__(self):
        super().__init__('husky_monitor')
        
        self.husky = HuskyRobotInterface(self)
                
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        self.start_pybullet()
        
    def start_pybullet(self):
        # start pybullet simulator
        pp.connect(use_gui=True, shadows=True, color=[0.9, 0.9, 1.0])
        # y-up to be consistent with mocap
        p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP, 1, physicsClientId=pp.CLIENT)
        # turn on the GUI panels
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1, physicsClientId=pp.CLIENT)

        self.param_slider = p.addUserDebugParameter("a slider", 0.0, 1.0, 0.0)
    
    def timer_callback(self):
        #p.addUserDebugText(text="AAAA", textPosition=[0,100,100])
        p.addUserDebugPoints(pointPositions=[[self.husky.xpos*100,0,0]],pointColorsRGB=[[0.9, 0.9, 0.0]],lifeTime=1.1,pointSize=10)

def main(args=None):
    rclpy.init(args=args)

    husky_monitor = HuskyMonitor()

    rclpy.spin(husky_monitor)

    husky_monitor.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':     
    main()
