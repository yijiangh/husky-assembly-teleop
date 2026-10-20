"""The mobile base of one husky: its pose from mocap, driving it, and its onboard path follower."""

from __future__ import annotations

import itertools
import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable

import numpy as np
from crl_husky_msgs.msg import BaseFollowerState, BasePath
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import Bool

from ..config import RobotConfig
from .connections import RosConnections
from .controller_manager import ControllerManagerInterface, ControllerManagerState
from .event import MESSAGE_QUEUE, Event
from .mocap import subscribe_mocap
from .mocap_state import RawMarkers, pose_lost
from .simulated_parts import SimulatedPart
from .stream_stats import StreamStats

#: The base's only controller for now. cmd_vel goes through it.
PLATFORM_VELOCITY_CONTROLLER = "platform_velocity_controller"
#: Rate of the platform's joint_states, Hz. ? Measured on Alice: ~30 Hz, though its controller manager is set to 50.
JOINT_STATES_RATE = 28.0

#: BaseFollowerState.state codes, by name.
FOLLOWER_STATES = ("idle", "following", "done", "aborted")

#: A follower silent for this long, seconds, is not running.
FOLLOWER_SILENT_AFTER = 1.0


@dataclass
class FollowerState:
    """The onboard path follower's last report (crl_husky_msgs/BaseFollowerState).

    Attributes:
        controller: The follower's node name in the robot's namespace, e.g. "pure_pursuit".
        state: "idle", "following", "done" or "aborted".
        reason: Why it aborted or went idle, else "".
        path_id: Id of the current or last path, as returned by `send_path`.
        timed: Whether that path is timed.
        piece: Current piece: a drive along a curve or a turn on the spot.
        piece_count: Pieces in the path.
        turning: Whether the current piece is a turn on the spot.
        progress: 0 to 1 along the whole path.
        reference: The path pose it compares against (x, y, yaw), metres and radians.
        lookahead: The point it steers toward (x, y), NaN while turning.
        cross_track_error: Metres, robot left of the path positive; NaN while turning.
        along_track_error: Metres behind schedule on a timed path, else NaN.
        yaw_error: Reference yaw minus measured, radians.
        command: Linear (m/s) and angular (rad/s) velocity it sent.
        update_time: ROS time the monitor handled the report, seconds.
        stamp: Time the follower sent the report, and the command in it, seconds on the robot's clock (time synced).
    """

    controller: str
    state: str
    reason: str
    path_id: int
    timed: bool
    piece: int
    piece_count: int
    turning: bool
    progress: float
    reference: np.ndarray
    lookahead: np.ndarray
    cross_track_error: float
    along_track_error: float
    yaw_error: float
    command: np.ndarray
    update_time: float
    stamp: float


@dataclass
class WheelOdometry:
    """The platform's last wheel odometry (nav_msgs/Odometry): its velocity in the base frame.

    Attributes:
        velocity: Linear (m/s) and angular (rad/s) velocity.
        update_time: ROS time the monitor handled the message, seconds.
        stamp: Time the platform stamped it, seconds on the robot's clock (time synced).
    """

    velocity: np.ndarray
    update_time: float
    stamp: float


@dataclass(frozen=True)
class SentTwist:
    """A velocity command the monitor sent the base: the record of `BaseInterface.twists_sent`.

    Attributes:
        command: Linear (m/s) and angular (rad/s) velocity.
        update_time: ROS time it was sent, seconds.
        stamp: The same; the monitor's clock.
    """

    command: np.ndarray
    update_time: float
    stamp: float


@dataclass(frozen=True)
class SentPath:
    """A path the monitor sent the onboard follower: the record of `BaseInterface.paths_sent`.

    Attributes:
        path_id: Its id, as in the follower's reports; the stopped path's for a stop.
        waypoints: Poses in it; 0 for a stop (an empty path).
        timed: 1 for a timed path, else 0.
        speed_caps: Linear (m/s) and angular (rad/s) caps sent with it; 0 keeps the follower's own.
        update_time: ROS time it was sent, seconds.
        stamp: The same; the monitor's clock, also in the message's header.
    """

    path_id: int
    waypoints: int
    timed: int
    speed_caps: np.ndarray
    update_time: float
    stamp: float


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
        follower: The onboard path follower's last report.
        odometry: The platform's last wheel odometry; none in the simulator.
        last_update_time: ROS time of the last mocap message, valid or not, seconds.
        last_fix_time: ROS time the last valid pose was captured, seconds.
        markers: Raw mocap markers of the last mocap message, valid or not, or None before one.
        wheel_positions: Wheel joint angles from the platform's joint_states, radians, by joint name.
        wheel_velocities: Wheel joint speeds from the platform's joint_states, rad/s, by joint name.
        joint_states_update_time: ROS time of the last platform joint_states, seconds.
        joint_states_stats: Rate, gaps and delay of the platform's joint_states, to judge the wifi link and the
            robot's clock. Published from boot, also with the velocity controller off; none in the simulator.

    ! `tracked` False means the pose is stale: grey it out. Drive from it only while `world.mocap.pose_lost` is "":
      `tracked` stays True when the relay goes silent.
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
    follower: FollowerState | None = None
    odometry: WheelOdometry | None = None
    last_update_time: float | None = None
    last_fix_time: float | None = None
    markers: RawMarkers | None = None
    wheel_positions: dict[str, float] = field(default_factory=dict)
    wheel_velocities: dict[str, float] = field(default_factory=dict)
    joint_states_update_time: float | None = None
    joint_states_stats: StreamStats = field(default_factory=StreamStats)


class BaseInterface:
    """ROS2 interface to one husky base.

    Attributes:
        follower_reports: Fired with each `FollowerState`, for message signals.
        odometry_messages: Fired with each `WheelOdometry`, for message signals.
        mocap_messages: Fired with each `MocapSample` of the base's rigid body, valid or not.
        twists_sent: Fired with each `SentTwist`, the stop's zero twist included.
        paths_sent: Fired with each `SentPath`, the empty path of a stop included.

    ! Mocap is the only source of the base pose; do not add a second one.
    """

    def __init__(self, node: Node, config: RobotConfig,
                 simulated: Callable[[], SimulatedPart | None] = lambda: None):
        """Subscribe to mocap and create the velocity publisher.

        Args:
            node: The monitor node.
            config: The robot this base belongs to.
            simulated: The base's simulator announcement while it is simulated, else None (`SimulatedParts.get`).
        """
        self._node = node
        self._config = config
        self._simulated = simulated
        self.state = BaseState()
        self._ros = RosConnections(node)
        # * Path ids start somewhere new each run, so a report on an earlier run's path never matches a new one.
        self._path_ids = itertools.count(int(time.time()) % 1_000_000 * 1000)
        self._path_id = 0
        #: (list, get) parameter clients per node, made on first use.
        self._parameter_clients: dict[str, tuple] = {}
        self.follower_reports = Event(f"{config.serial}_follower")
        self.odometry_messages = Event(f"{config.serial}_odometry")
        self.mocap_messages = Event(f"{config.serial}_mocap")
        self.twists_sent = Event(f"{config.serial}_twist")
        self.paths_sent = Event(f"{config.serial}_path")
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
        """Create the velocity and path publishers, and subscribe to the base's topics."""
        namespace = f"/{self._config.ros_namespace}"
        self._cmd_vel = self._ros.publisher(Twist, f"{namespace}/cmd_vel")
        self._base_path = self._ros.publisher(BasePath, f"{namespace}/base_path")
        self._ros.subscription(BaseFollowerState, f"{namespace}/base_follower/state", self._on_follower,
                               MESSAGE_QUEUE)
        # ? Best effort QoS, since the platform's publisher QoS is unknown.
        self._ros.subscription(Bool, f"{namespace}/platform/emergency_stop", self._on_estop,
                               qos_profile_sensor_data)
        self._ros.subscription(BatteryState, f"{namespace}/platform/bms/state", self._on_battery,
                               qos_profile_sensor_data)
        # ? The velocity controller's odometry from the wheels (Clearpath's topic); none in the simulator.
        self._ros.subscription(Odometry, f"{namespace}/platform/odom", self._on_odometry,
                               QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                                          depth=MESSAGE_QUEUE))
        self._ros.subscription(JointState, f"{namespace}/platform/joint_states", self._on_joint_states,
                               qos_profile_sensor_data)
        if self._config.mocap_id is not None:
            subscribe_mocap(self._ros, self._config.mocap_id, self.state,
                            lambda: self._node.get_clock().now().nanoseconds * 1e-9, self.mocap_messages)

    def reconnect(self) -> None:
        """Recreate this base's topics and clients. Keeps `state`."""
        self._parameter_clients.clear()
        self._ros.destroy_all()
        self._connect()
        self.controllers.reconnect()

    @property
    def simulation(self) -> SimulatedPart | None:
        """The base simulator's announcement (`SimulatedPart`) while this base is simulated, else None: a real base."""
        return self._simulated()

    def follow_problem(self) -> str:
        """Why the onboard follower may not be handed a path now, or "": no live mocap pose, e-stop, or no follower.

        ! Check it before a path goes out and while one is followed; `send_path` refuses a path while it holds.
        """
        now = self._node.get_clock().now().nanoseconds * 1e-9
        if lost := pose_lost(self.mocap_id, self.state, now):
            return f"no mocap pose: {lost}"
        if self.state.estopped:
            return "e-stopped"
        if live_report(self.state.follower, now) is None:
            return "no follower: start crl_husky pure_pursuit"
        return ""

    def send_twist(self, linear_x: float, angular_z: float, require_controller: bool = True) -> bool:
        """Drive the base.

        Args:
            linear_x: Forward speed, metres per second.
            angular_z: Turn rate, radians per second.
            require_controller: Send only while the velocity controller runs; False for the simulator (it has none),
                or for a stop.

        Returns:
            bool: True if sent. False if a speed is not finite, or (with `require_controller`) the velocity controller
                is not running or its manager has not answered lately.
        """
        if not (math.isfinite(linear_x) and math.isfinite(angular_z)):
            self._node.get_logger().error(f"{self.controllers.namespace}: not driving, the twist is not finite "
                                          f"({linear_x}, {angular_z})")
            return False
        problem = self.controllers.running_problem(PLATFORM_VELOCITY_CONTROLLER) if require_controller else None
        if problem is not None:
            self._node.get_logger().warning(f"{self.controllers.namespace}: not driving, {problem}",
                                            throttle_duration_sec=2.0)
            return False
        self._publish_twist(float(linear_x), float(angular_z))
        return True

    def stop(self) -> bool:
        """Soft stop: stop the path follower and brake with one zero twist, then switch the velocity controller off.

        Returns:
            bool: True if the deactivation went out. False if the controller manager is unreachable.
        """
        self.stop_path()
        self._publish_twist(0.0, 0.0)
        return self.controllers.deactivate_all()

    def send_path(self, x, y, yaw, t=None, max_linear_speed: float = 0.0, max_angular_speed: float = 0.0) -> int:
        """Hand a world-frame floor path to the onboard follower, replacing the one it follows.

        ! The follower drives through cmd_vel, so the base only moves while the velocity controller is active.

        Args:
            x: Positions, metres.
            y: Positions, metres.
            yaw: Yaw per pose, radians, unwrapped: the step between poses gives the turn direction.
            t: Seconds from now per pose for a timed path, or None for a geometric one.
            max_linear_speed: Cap for this path, m/s; 0 keeps the follower's own.
            max_angular_speed: Cap for this path, rad/s; 0 keeps the follower's own.

        Returns:
            int: The path's id, as in `state.follower.path_id`.

        Raises:
            ValueError: If x, y, yaw (and t) differ in length or hold a value that is not finite, or the follower may
                not take a path now (`follow_problem`); nothing is sent.
        """
        columns = [[float(v) for v in values] for values in (x, y, yaw) + (() if t is None else (t,))]
        caps = [float(max_linear_speed), float(max_angular_speed)]
        if len({len(column) for column in columns}) != 1:
            raise ValueError("x, y, yaw and t differ in length")
        if not all(math.isfinite(v) for v in caps + [v for column in columns for v in column]):
            raise ValueError("the path holds a value that is not finite")
        if problem := self.follow_problem():
            raise ValueError(problem)
        message = BasePath()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.header.frame_id = "mocap_world"
        message.id = self._path_id = next(self._path_ids)
        message.x, message.y, message.yaw = columns[:3]
        message.t = columns[3] if t is not None else []
        message.max_linear_speed, message.max_angular_speed = caps
        self._publish_path(message, len(columns[0]), t is not None)
        return message.id

    def stop_path(self) -> None:
        """Tell the onboard follower to stop and drop its path (an empty path)."""
        message = BasePath()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.id = self._path_id
        self._publish_path(message, 0, False)

    def _publish_twist(self, linear: float, angular: float) -> None:
        """Publish a twist, unchecked, then fire `twists_sent`."""
        message = Twist()
        message.linear.x, message.angular.z = linear, angular
        self._cmd_vel.publish(message)
        now = self._node.get_clock().now().nanoseconds * 1e-9
        self.twists_sent.fire(SentTwist(np.array([linear, angular]), now, now))

    def _publish_path(self, message: BasePath, waypoints: int, timed: bool) -> None:
        """Publish a path message, then fire `paths_sent`."""
        self._base_path.publish(message)
        sent = message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec
        self.paths_sent.fire(SentPath(int(message.id), waypoints, int(timed),
                                      np.array([message.max_linear_speed, message.max_angular_speed]), sent, sent))

    def _on_estop(self, message: Bool) -> None:
        """Store whether the platform's emergency stop is engaged."""
        self.state.estopped = bool(message.data)

    def node_names(self) -> list[str]:
        """Names of the ROS nodes running in this robot's namespace, from the ROS graph, e.g. ["pure_pursuit"]."""
        namespace = f"/{self._config.ros_namespace}"
        return sorted(name for name, node_namespace in self._node.get_node_names_and_namespaces()
                      if node_namespace == namespace)

    def list_parameters(self, node: str):
        """Ask a node in this robot's namespace for its parameter names.

        Returns:
            The rclpy future of the ListParameters call (await with `ctx.ros`), or None if the node does not answer.
        """
        clients = self._clients_for(node)
        if not clients[0].service_is_ready():
            return None
        return clients[0].call_async(ListParameters.Request())

    def get_parameters(self, node: str, names: list[str]):
        """Ask a node in this robot's namespace for parameter values; convert them with `parameter_value`.

        Returns:
            The rclpy future of the GetParameters call (await with `ctx.ros`), or None if the node does not answer.
        """
        clients = self._clients_for(node)
        if not clients[1].service_is_ready():
            return None
        return clients[1].call_async(GetParameters.Request(names=list(names)))

    def _clients_for(self, node: str) -> tuple:
        """The parameter clients of `node`, made on first use."""
        if node not in self._parameter_clients:
            prefix = f"/{self._config.ros_namespace}/{node}"
            self._parameter_clients[node] = (self._ros.client(ListParameters, f"{prefix}/list_parameters"),
                                             self._ros.client(GetParameters, f"{prefix}/get_parameters"))
        return self._parameter_clients[node]

    def _on_follower(self, message: BaseFollowerState) -> None:
        """Store the follower's report, then fire `follower_reports`."""
        self.state.follower = FollowerState(
            controller=message.controller,
            state=FOLLOWER_STATES[message.state] if message.state < len(FOLLOWER_STATES) else "unknown",
            reason=message.reason, path_id=int(message.path_id), timed=bool(message.timed),
            piece=int(message.piece), piece_count=int(message.piece_count), turning=bool(message.turning),
            progress=float(message.progress),
            reference=np.array([message.reference_x, message.reference_y, message.reference_yaw]),
            lookahead=np.array([message.lookahead_x, message.lookahead_y]),
            cross_track_error=float(message.cross_track_error), along_track_error=float(message.along_track_error),
            yaw_error=float(message.yaw_error),
            command=np.array([message.linear_velocity, message.angular_velocity]),
            update_time=self._node.get_clock().now().nanoseconds * 1e-9,
            stamp=message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec)
        self.follower_reports.fire(self.state.follower)

    def _on_odometry(self, message: Odometry) -> None:
        """Store the wheel odometry's velocities, then fire `odometry_messages`."""
        self.state.odometry = WheelOdometry(
            velocity=np.array([message.twist.twist.linear.x, message.twist.twist.angular.z]),
            update_time=self._node.get_clock().now().nanoseconds * 1e-9,
            stamp=message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec)
        self.odometry_messages.fire(self.state.odometry)

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


def parameter_value(value):
    """A ROS ParameterValue as plain Python: bool, int, float, str, a list of those, or None if not set."""
    by_type = {
        ParameterType.PARAMETER_BOOL: "bool_value", ParameterType.PARAMETER_INTEGER: "integer_value",
        ParameterType.PARAMETER_DOUBLE: "double_value", ParameterType.PARAMETER_STRING: "string_value",
        ParameterType.PARAMETER_BYTE_ARRAY: "byte_array_value", ParameterType.PARAMETER_BOOL_ARRAY: "bool_array_value",
        ParameterType.PARAMETER_INTEGER_ARRAY: "integer_array_value",
        ParameterType.PARAMETER_DOUBLE_ARRAY: "double_array_value",
        ParameterType.PARAMETER_STRING_ARRAY: "string_array_value",
    }
    field_name = by_type.get(value.type)
    if field_name is None:
        return None
    result = getattr(value, field_name)
    return list(result) if not isinstance(result, (bool, int, float, str)) else result


def live_report(follower: FollowerState | None, now: float) -> FollowerState | None:
    """The follower's last report if it came within FOLLOWER_SILENT_AFTER seconds, else None: no follower runs."""
    if follower is None or now - follower.update_time >= FOLLOWER_SILENT_AFTER:
        return None
    return follower
