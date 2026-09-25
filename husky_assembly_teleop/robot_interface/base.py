"""
The mobile base of one husky: its pose from mocap, and driving it.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from crl_husky_msgs.msg import MocapRigidBodyPose
from geometry_msgs.msg import Twist
from rclpy.node import Node

from ..config import RobotConfig
from .controller_manager import ControllerManagerInterface, ControllerManagerState

#: The base's only controller for now. cmd_vel goes through it.
PLATFORM_VELOCITY_CONTROLLER = "platform_velocity_controller"


@dataclass
class BaseState:
    """Measured state of the base.

    Attributes:
        position: Position in world frame, metres.
        orientation: Orientation in world frame, quaternion (x, y, z, w).
        tracked: Whether the pose is a live mocap fix. False means stale or never
            set, and callers must not read the pose as a measurement.
        tracking_valid: Whether NatNet itself tracked the body in the last
            sample. False usually means too many markers are hidden.
        marker_error: Mean marker error of the last sample, metres, as NatNet
            reports it. None before the first sample.
        controllers: The base controller manager's state.
        last_update_time: ROS time of the last mocap message, valid or not, seconds.
        last_fix_time: ROS time of the last *valid* pose, seconds. 0 means the
            pose above is still the default and has never been measured.

    ! Which pose to trust, in short: `has_fix` says whether the pose was ever
      measured (show dashes until then); `tracked` says whether it is being
      measured right now (grey it out, and do not plan from it, when not).
    """

    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    orientation: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))
    tracked: bool = False
    tracking_valid: bool = False
    marker_error: float | None = None
    controllers: ControllerManagerState = field(default_factory=ControllerManagerState)
    last_update_time: float = 0.0
    last_fix_time: float = 0.0

    @property
    def has_fix(self) -> bool:
        """bool: Whether mocap has ever sent a valid pose. Until then `position` is only the default."""
        return self.last_fix_time > 0.0


class BaseInterface:
    """ROS2 interface to one husky base.

    ! Mocap is the only source of the base pose. The odometry TF the old code
      also listened to is gone: two sources that disagree are worse than one.
    """

    def __init__(self, node: Node, config: RobotConfig):
        """Subscribe to mocap and create the velocity publisher.

        Args:
            node: The monitor node.
            config: The robot this base belongs to.
        """
        self._node = node
        self.state = BaseState()
        namespace = f"/{config.ros_namespace}"
        self.controllers = ControllerManagerInterface(
            node, namespace, (PLATFORM_VELOCITY_CONTROLLER,), self.state.controllers)
        self._cmd_vel = node.create_publisher(Twist, f"{namespace}/cmd_vel", 10)

        # * crl_husky's mocap_relay publishes this pose already Z-up and
        #   calibrated (see crl-husky/MOCAP_SETUP.md). Use it as is; calibrating
        #   again here would apply the transform twice.
        if config.mocap_id is None:
            node.get_logger().warning(f"robot {config.serial} has no mocap id; its base "
                                      f"pose will never be tracked")
        else:
            self._mocap = node.create_subscription(
                MocapRigidBodyPose, f"/mocap/rigid_body/id_{config.mocap_id}/pose",
                self._on_mocap, 10)

    def send_twist(self, linear_x: float, angular_z: float) -> bool:
        """Drive the base.

        Args:
            linear_x: Forward speed, metres per second.
            angular_z: Turn rate, radians per second.

        Returns:
            bool: True if sent. False if the velocity controller is not running.
        """
        if not self.controllers.is_active(PLATFORM_VELOCITY_CONTROLLER):
            self._node.get_logger().warning(
                f"{self.controllers.namespace}: not driving, {PLATFORM_VELOCITY_CONTROLLER} "
                f"is not active", throttle_duration_sec=2.0)
            return False
        message = Twist()
        message.linear.x = float(linear_x)
        message.angular.z = float(angular_z)
        self._cmd_vel.publish(message)
        return True

    def _on_mocap(self, message: MocapRigidBodyPose) -> None:
        """Store a mocap sample. An invalid sample only clears `tracked`.

        ? `pose_valid` is the relay's combined verdict (fresh, tracked by
          NatNet, marker error under threshold), so it is the only flag read.
          The pose of an invalid sample is not stored, so readers that ignore
          `tracked` still see the last good pose rather than garbage.
        """
        now = self._node.get_clock().now().nanoseconds * 1e-9
        self.state.tracked = bool(message.pose_valid)
        # * Kept for the health panel, so it can say *why* a pose is not valid.
        self.state.tracking_valid = bool(message.tracking_valid)
        self.state.marker_error = float(message.marker_error)
        if self.state.tracked:
            p, q = message.pose.position, message.pose.orientation
            self.state.position = np.array([p.x, p.y, p.z])
            self.state.orientation = np.array([q.x, q.y, q.z, q.w])
            self.state.last_fix_time = now
        self.state.last_update_time = now
