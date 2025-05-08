from ur_msgs.srv import SetIO
import rclpy
from rclpy.node import Node

# ros2 service call /a200_0804/ur5e/io_and_status_controller/set_io ur_msgs/srv/SetIO "{fun: 1, pin: 17, state: 0}"
class IOClient(Node):

    def __init__(self):
        super().__init__('test_setio')
        # self.cli = self.create_client(SetIO, '/a200_0804/ur5e/io_and_status_controller/set_io')
        # self.cli = self.create_client(SetIO, '/a200_0806/left_ur5e/io_and_status_controller/set_io')
        self.cli = self.create_client(SetIO, '/a200_0806/right_ur5e/io_and_status_controller/set_io')

        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = SetIO.Request()

    def send_request(self):
        self.req.fun = SetIO.Request.FUN_SET_DIGITAL_OUT # Prefer using constants instead of writing the constant's value
        # self.req.pin = SetIO.Request.PIN_TOOL_DOUT0
        self.req.pin = SetIO.Request.PIN_TOOL_DOUT1
        self.req.state = float(SetIO.Request.STATE_ON)
        # self.req.state = float(SetIO.Request.STATE_OFF)

        self.future = self.cli.call_async(self.req)
        self.get_logger().info('request sent...')
        rclpy.spin_until_future_complete(self, self.future)
        
def main():
    rclpy.init()

    IOclient = IOClient()
    response = IOclient.send_request()
    IOclient.get_logger().info('response: %s' % response)

    IOclient.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()