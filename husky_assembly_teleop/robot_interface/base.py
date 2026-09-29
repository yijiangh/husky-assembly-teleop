"""
The mobile base of one husky: its pose from mocap, and driving it.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from crl_husky_msgs.msg import MocapRigidBodyPose
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

from ..config import RobotConfig
from .connections import RosConnections
from .controller_manager import ControllerManagerInterface, ControllerManagerState

#: The base's only controller for now. cmd_vel goes through it.
PLATFORM_VELOCITY_CONTROLLER = "platform_velocity_controller"


@dataclass
class BaseState:
    """Measured state of the base.

    Attributes:
        position: Position in world frame, metres. None until the first valid
            fix, then the last valid one.
        orientation: Orientation in world frame, quaternion (x, y, z, w). Set
            together with `position`.
        tracked: Whether the latest mocap sample was valid. Implies `position`
            is set. False means stale or lost: show the pose greyed out, but do
            not plan or control from it.
        tracking_valid: Whether NatNet itself tracked the body in the last
            sample, or None before any sample. False usually means hidden markers.
        marker_error: Mean marker error of the last sample, metres, as NatNet
            reports it, or None before any sample.
        estopped: Whether the platform's emergency stop is engaged, or None
            before the first message.
        battery_percentage: Charge from the battery management system, 0 to 1,
            or None before the first message. NaN if the BMS does not report it.
        battery_voltage: Battery voltage, volts, or None before the first message.
        battery_charging: Whether the battery reports it is charging.
        battery_health: The BMS's `power_supply_health`, a BatteryState
            POWER_SUPPLY_HEALTH_* constant, or None before the first message.
        battery_update_time: ROS time of the last battery message, seconds, or None.
        controllers: The base controller manager's state.
        last_update_time: ROS time of the last mocap message, valid or not,
            seconds, or None before any.
        last_fix_time: ROS time of the last *valid* pose, seconds, or None before any.

    ! Which pose to trust, in short: `position is None` -- never measured, show
      dashes; `tracked` False -- measured before, not now, grey it out.
    """

    position: np.ndarray | None = None
    orientation: np.ndarray | None = None
    tracked: bool = False
    tracking_valid: bool | None = None
    marker_error: float | None = None
    estopped: bool | None = None
    battery_percentage: float | None = None
    battery_voltage: float | None = None
    battery_charging: bool = False
    battery_health: int | None = None
    battery_update_time: float | None = None
    controllers: ControllerManagerState = field(default_factory=ControllerManagerState)
    last_update_time: float | None = None
    last_fix_time: float | None = None


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
        self._config = config
        self.state = BaseState()
        self._ros = RosConnections(node)
        self.controllers = ControllerManagerInterface(
            node, f"/{config.ros_namespace}", (PLATFORM_VELOCITY_CONTROLLER,), self.state.controllers)
        if config.mocap_id is None:
            node.get_logger().warning(f"robot {config.serial} has no mocap id; its base "
                                      f"pose will never be tracked")
        self._connect()

    def _connect(self) -> None:
        """Create the velocity publisher and the mocap, e-stop and battery subscriptions."""
        namespace = f"/{self._config.ros_namespace}"
        self._cmd_vel = self._ros.publisher(Twist, f"{namespace}/cmd_vel")
        # ? Best effort, which receives from reliable and best-effort publishers
        #   alike. The Clearpath platform's QoS for these is not pinned down here.
        self._ros.subscription(Bool, f"{namespace}/platform/emergency_stop", self._on_estop,
                               qos_profile_sensor_data)
        self._ros.subscription(BatteryState, f"{namespace}/platform/bms/state", self._on_battery,
                               qos_profile_sensor_data)
        # * crl_husky's mocap_relay publishes this pose already calibrated and in
        #   the Z-up 'rhino' world frame that Rhino designs and PyBullet share
        #   (see crl-husky/MOCAP_SETUP.md). Use it as is; converting axes or
        #   calibrating again here would apply the transform twice.
        if self._config.mocap_id is not None:
            self._ros.subscription(MocapRigidBodyPose,
                                   f"/mocap/rigid_body/id_{self._config.mocap_id}/pose", self._on_mocap)

    def reconnect(self) -> None:
        """Destroy this base's topics and clients and create them again. Keeps `state`."""
        self._ros.destroy_all()
        self._connect()
        self.controllers.reconnect()

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

    def stop(self) -> bool:
        """Soft stop: one zero twist, then switch the velocity controller off.

        ? Why the controller, not just a zero twist. twist_mux listens to
          other sources too (joystick, RC, interactive marker), and a plugin
          could send again the next tick. With the controller off nothing
          drives the wheels until someone switches it on again. The Clearpath
          stack offers no software e-stop: its twist_mux lock listens to the
          MCU's own emergency_stop status, which is not ours to publish.

        Returns:
            bool: True if the deactivation went out. False if the controller
                manager is not reachable; the zero twist was sent anyway.
        """
        # * Sent whether or not the controller runs: it costs nothing, and
        #   brakes the base while the deactivation is on its way.
        self._cmd_vel.publish(Twist())
        return self.controllers.deactivate_all()

    def _on_mocap(self, message: MocapRigidBodyPose) -> None:
        """Store a mocap sample. An invalid sample only clears `tracked`.

        ? `pose_valid` is the relay's combined verdict (fresh, tracked by
          NatNet, marker error under threshold), so it decides `tracked`. The
          pose of an invalid sample is not stored: `position` keeps the last
          valid one, or stays None if there never was one.
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

    def _on_estop(self, message: Bool) -> None:
        """Store whether the platform's emergency stop is engaged."""
        self.state.estopped = bool(message.data)

    def _on_battery(self, message: BatteryState) -> None:
        """Store the battery charge, voltage and health."""
        self.state.battery_percentage = float(message.percentage)
        self.state.battery_voltage = float(message.voltage)
        self.state.battery_charging = message.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
        self.state.battery_health = int(message.power_supply_health)
        self.state.battery_update_time = self._node.get_clock().now().nanoseconds * 1e-9
