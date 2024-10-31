import rclpy

import numpy as np

from pybullet_mocap.husky_robot import HuskyRobotInterface

def main(args=None):
    rclpy.init(args=args)

    husky_robot = HuskyRobotInterface()

    rclpy.spin(husky_robot)

    husky_robot.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
