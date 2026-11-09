"""
A fixed mocap pose for a demo without Motive: rigid bodies published standing still, where the demo assumes them.

The monitor takes a robot's base only from the relay's topic, and every planner and compliant part refuses without a
live pose. This node publishes each body of `bodies` at `pose`, valid and fresh, on that topic, so the monitor sees the
robot standing there (the insertion demo: Cindy at the origin, her jig on her base).

    ros2 run husky_assembly_teleop husky_fixed_base                                             # Cindy at the origin
    ros2 run husky_assembly_teleop husky_fixed_base --ros-args -p bodies:="[1850]" -p pose:="1.0 0.5 0 90"

- ! Never drive a base from it: the pose never changes, whatever the robot does.
- ! Never while the relay runs: two publishers of one body.
"""

from __future__ import annotations

from typing import List, Optional

import rclpy
from builtin_interfaces.msg import Time
from crl_husky_msgs.msg import MocapRigidBodyPose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose

from ..robot_interface.mocap import mocap_topic

#: Cindy's rigid body: the demo robot.
DEFAULT_BODIES = [1860]
DEFAULT_RATE_HZ = 20.0
#: The relay's frame id of a calibrated pose.
LAB_FRAME = "lab"


def parse_pose(text: str) -> Pose:
    """A pose from "x y z yaw_deg" (metres, degrees; fewer values are 0), e.g. "0 0 0 0" for the origin.

    Raises:
        ValueError: For more than four values, or one that is not a number.
    """
    values = [float(value) for value in text.replace(",", " ").split()]
    if len(values) > 4:
        raise ValueError(f"pose {text!r}: at most x y z yaw_deg, not {len(values)} values")
    x, y, z, yaw = values + [0.0] * (4 - len(values))
    return Pose((x, y, z), tuple(Rotation.from_euler("z", yaw, degrees=True).as_quat()))


def fixed_pose_message(pose: Pose, stamp: Time) -> MocapRigidBodyPose:
    """The relay's message for a body standing at `pose`, valid, captured at `stamp`, in the lab frame, no markers."""
    message = MocapRigidBodyPose()
    message.header.stamp = stamp
    message.header.frame_id = LAB_FRAME
    message.pose.position.x, message.pose.position.y, message.pose.position.z = (float(v) for v in pose.position)
    (message.pose.orientation.x, message.pose.orientation.y,
     message.pose.orientation.z, message.pose.orientation.w) = (float(v) for v in pose.orientation)
    message.pose_valid = message.tracking_valid = True
    message.marker_error = 0.0
    return message


class FixedBase(Node):
    """Publishes every body of `bodies` at `pose` at `rate_hz`, stamped now."""

    def __init__(self):
        """Read the parameters and start the timer; says loudly what it fakes."""
        super().__init__("fixed_base")
        self.declare_parameter("bodies", DEFAULT_BODIES)
        self.declare_parameter("pose", "0 0 0 0")
        self.declare_parameter("rate_hz", DEFAULT_RATE_HZ)
        self.bodies = [int(body) for body in self.get_parameter("bodies").value]
        self.pose = parse_pose(str(self.get_parameter("pose").value))
        rate = float(self.get_parameter("rate_hz").value)
        if rate <= 0.0:
            raise ValueError(f"rate_hz must be positive, not {rate}")
        # ! Not named `_publishers`: that is rclpy's own list, which `destroy_node` walks.
        self._by_body = {body: self.create_publisher(MocapRigidBodyPose, mocap_topic(body), 10)
                         for body in self.bodies}
        self.create_timer(1.0 / rate, self._publish)
        x, y, z = self.pose.position
        yaw = Rotation.from_quat(self.pose.orientation).as_euler("xyz", degrees=True)[2]
        self.get_logger().warn(f"FIXED mocap pose: rigid bodies {self.bodies} stand at x {x:g} y {y:g} z {z:g} m, "
                               f"yaw {yaw:g}°, {rate:g} Hz, whatever the robots do. Never drive a base from it, "
                               f"and never run the mocap relay at the same time.")

    def _publish(self) -> None:
        """One message per body, stamped now."""
        stamp = self.get_clock().now().to_msg()
        for body, publisher in self._by_body.items():
            publisher.publish(fixed_pose_message(self.pose, stamp))


def main(args: Optional[List[str]] = None) -> None:
    """Run until Ctrl-C; exit 1 on a bad parameter."""
    rclpy.init(args=args)
    try:
        node = FixedBase()
    except ValueError as error:
        rclpy.logging.get_logger("fixed_base").error(str(error))
        rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ! Nothing to clean up: on Ctrl-C rclpy's own handler shuts the context down, and a second shutdown raises.
        pass


if __name__ == "__main__":
    main()
