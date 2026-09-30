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
from ..mocap import mocap_topic, store_sample
from .connections import RosConnections
from .controller_manager import ControllerManagerInterface, ControllerManagerState

#: The base's only controller for now. cmd_vel goes through it.
PLATFORM_VELOCITY_CONTROLLER = "platform_velocity_controller"


@dataclass
class BaseState:
    """Measured state of the base. None means no message received yet.

    Attributes:
        position: Last valid position in world frame, metres.
        orientation: Last valid orientation, quaternion (x, y, z, w).
        tracked: Whether the latest mocap sample was valid.
        tracking_valid: Whether NatNet tracked the body in the last sample.
            False usually means hidden markers.
        marker_error: Mean marker error of the last sample, metres.
        estopped: Whether the platform's emergency stop is engaged.
        battery_percentage: Charge, 0 to 1. NaN if the BMS does not report it.
        battery_voltage: Battery voltage, volts.
        battery_charging: Whether the battery reports it is charging.
        battery_health: BatteryState POWER_SUPPLY_HEALTH_* constant.
        battery_update_time: ROS time of the last battery message, seconds.
        controllers: The base controller manager's state.
        last_update_time: ROS time of the last mocap message, valid or not, seconds.
        last_fix_time: ROS time of the last valid pose, seconds.

    ! `position is None`: never measured, show dashes. `tracked` False: pose is
      stale, grey it out and do not plan or control from it.
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

    ! Mocap is the only source of the base pose; do not add a second one.
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

    @property
    def mocap_id(self) -> int | None:
        """int | None: Rigid-body id of the base in the mocap system, or None if untracked."""
        return self._config.mocap_id

    def _connect(self) -> None:
        """Create the velocity publisher and the mocap, e-stop and battery subscriptions."""
        namespace = f"/{self._config.ros_namespace}"
        self._cmd_vel = self._ros.publisher(Twist, f"{namespace}/cmd_vel")
        # ? Best effort QoS, since the platform's publisher QoS is unknown.
        self._ros.subscription(Bool, f"{namespace}/platform/emergency_stop", self._on_estop,
                               qos_profile_sensor_data)
        self._ros.subscription(BatteryState, f"{namespace}/platform/bms/state", self._on_battery,
                               qos_profile_sensor_data)
        if self._config.mocap_id is not None:
            self._ros.subscription(MocapRigidBodyPose, mocap_topic(self._config.mocap_id), self._on_mocap)

    def reconnect(self) -> None:
        """Recreate this base's topics and clients. Keeps `state`."""
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

        ? A zero twist alone is not enough: other sources or a plugin can drive
          again. With the controller off, nothing drives until it is switched on.

        Returns:
            bool: True if the deactivation went out. False if the controller
                manager is unreachable; the zero twist was sent anyway.
        """
        # * Always sent: it brakes the base while the deactivation is on its way.
        self._cmd_vel.publish(Twist())
        return self.controllers.deactivate_all()

    def _on_mocap(self, message: MocapRigidBodyPose) -> None:
        """Store a mocap sample; an invalid one only clears `tracked`, keeping the last valid pose."""
        store_sample(self.state, message, self._node.get_clock().now().nanoseconds * 1e-9)

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
