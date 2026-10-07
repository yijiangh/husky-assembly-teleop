"""The mobile base of one husky: its pose from mocap, and driving it."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import Bool

from ..config import RobotConfig
from .connections import RosConnections
from .controller_manager import ControllerManagerInterface, ControllerManagerState
from .mocap import subscribe_mocap
from .stream_stats import StreamStats

#: The base's only controller for now. cmd_vel goes through it.
PLATFORM_VELOCITY_CONTROLLER = "platform_velocity_controller"
#: Rate of the platform's joint_states, Hz. ? Measured on Alice: ~30 Hz, though its controller manager is set to 50.
JOINT_STATES_RATE = 28.0


@dataclass
class BaseState:
    """Measured state of the base. None means no message received yet.

    Attributes:
        position: Last valid position in world frame, metres.
        orientation: Last valid orientation, quaternion (x, y, z, w).
        tracked: Whether the latest mocap sample was valid.
        tracking_valid: Whether NatNet tracked the body in the last sample (False: usually hidden markers).
        marker_errors: (ROS time, marker error) of the valid samples in the last MARKER_ERROR_WINDOW, oldest first.
        marker_error: Mean marker error of the last sample, metres.
        estopped: Whether the platform's emergency stop is engaged.
        battery_percentage: Charge, 0 to 1. NaN if the BMS does not report it.
        battery_voltage: Battery voltage, volts.
        battery_charging: Whether the battery reports it is charging.
        battery_health: BatteryState POWER_SUPPLY_HEALTH_* constant.
        battery_update_time: ROS time of the last battery message, seconds.
        controllers: The base controller manager's state.
        last_update_time: ROS time of the last mocap message, valid or not, seconds.
        last_fix_time: ROS time the last valid pose was captured, seconds.
        wheel_positions: Wheel joint angles from the platform's joint_states, radians, by joint name.
        wheel_velocities: Wheel joint speeds from the platform's joint_states, rad/s, by joint name.
        joint_states_update_time: ROS time of the last platform joint_states, seconds.
        joint_states_stats: Rate, gaps and delay of the platform's joint_states, to judge the wifi link and the
            robot's clock. Published from boot, also with the velocity controller off; none in the simulator.

    ! `tracked` False means the pose is stale: grey it out and don't plan or control from it.
    """

    position: np.ndarray | None = None
    orientation: np.ndarray | None = None
    tracked: bool = False
    tracking_valid: bool | None = None
    marker_error: float | None = None
    marker_errors: deque[tuple[float, float]] = field(default_factory=deque)
    estopped: bool | None = None
    battery_percentage: float | None = None
    battery_voltage: float | None = None
    battery_charging: bool = False
    battery_health: int | None = None
    battery_update_time: float | None = None
    controllers: ControllerManagerState = field(default_factory=ControllerManagerState)
    last_update_time: float | None = None
    last_fix_time: float | None = None
    wheel_positions: dict[str, float] = field(default_factory=dict)
    wheel_velocities: dict[str, float] = field(default_factory=dict)
    joint_states_update_time: float | None = None
    joint_states_stats: StreamStats = field(default_factory=StreamStats)


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
        """Rigid-body id of the base in the mocap system, or None if untracked."""
        return self._config.mocap_id

    def _connect(self) -> None:
        """Create the velocity publisher and the mocap, e-stop, battery and joint_states subscriptions."""
        namespace = f"/{self._config.ros_namespace}"
        self._cmd_vel = self._ros.publisher(Twist, f"{namespace}/cmd_vel")
        # ? Best effort QoS, since the platform's publisher QoS is unknown.
        self._ros.subscription(Bool, f"{namespace}/platform/emergency_stop", self._on_estop,
                               qos_profile_sensor_data)
        self._ros.subscription(BatteryState, f"{namespace}/platform/bms/state", self._on_battery,
                               qos_profile_sensor_data)
        self._ros.subscription(JointState, f"{namespace}/platform/joint_states", self._on_joint_states,
                               qos_profile_sensor_data)
        if self._config.mocap_id is not None:
            subscribe_mocap(self._ros, self._config.mocap_id, self.state,
                            lambda: self._node.get_clock().now().nanoseconds * 1e-9)

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
        """Soft stop: one zero twist to brake, then switch the velocity controller off so nothing drives.

        Returns:
            bool: True if the deactivation went out. False if the controller manager is unreachable.
        """
        self._cmd_vel.publish(Twist())
        return self.controllers.deactivate_all()

    def _on_estop(self, message: Bool) -> None:
        """Store whether the platform's emergency stop is engaged."""
        self.state.estopped = bool(message.data)

    def _on_joint_states(self, message: JointState) -> None:
        """Store the wheel joints, and note the message for the stream's rate, gaps and delay."""
        now = self._node.get_clock().now().nanoseconds * 1e-9
        state = self.state
        state.wheel_positions.update(zip(message.name, map(float, message.position)))
        state.wheel_velocities.update(zip(message.name, map(float, message.velocity)))
        state.joint_states_update_time = now
        state.joint_states_stats.add(now, message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec)

    def _on_battery(self, message: BatteryState) -> None:
        """Store the battery charge, voltage and health."""
        self.state.battery_percentage = float(message.percentage)
        self.state.battery_voltage = float(message.voltage)
        self.state.battery_charging = message.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
        self.state.battery_health = int(message.power_supply_health)
        self.state.battery_update_time = self._node.get_clock().now().nanoseconds * 1e-9
