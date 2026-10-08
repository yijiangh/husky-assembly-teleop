"""
One UR arm: its measured state, its controllers, and commanding it.

Each command checks the running controller and its inputs itself before sending.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import numpy as np
from builtin_interfaces.msg import Duration
from crl_husky_msgs.msg import ArmStatus
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_msgs.msg import IOStates

from ..config import ArmConfig
from .stream_stats import StreamStats
from .connections import RosConnections
from .controller_manager import ControllerManagerInterface, ControllerManagerState
from .end_effectors import EndEffector, EndEffectorState, make_end_effector
from bar_assembly_core.design_io.pose import Pose, compose, invert
from .ur_frames import BASE_LINK_FROM_UR_BASE

# ? qos_profile_sensor_data but deeper, to hold one tick of samples: 100 covers 500 Hz for 200 ms.
RECORDED_QOS = QoSProfile(depth=100, reliability=qos_profile_sensor_data.reliability,
                          durability=qos_profile_sensor_data.durability,
                          history=qos_profile_sensor_data.history)

#: joint_states per second the robot's rate limiter sends (crl_husky rate_limiter DEFAULT_RATE).
JOINT_STATES_RATE = 50.0

#: Joint names in the UR driver's order. The URDF has the same names with the arm's prefix.
UR_JOINT_NAMES = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")

SCALED_JOINT_TRAJECTORY_CONTROLLER = "scaled_joint_trajectory_controller"
CARTESIAN_COMPLIANCE_CONTROLLER = "cartesian_compliance_controller"
#: UR freedrive: the arm can be pushed by hand, only while it keeps hearing from us (FREE_DRIVE_HEARTBEAT).
#: ? The tool's button chord runs free drive on the robot (crl_husky onboard/free_drive_buttons.py), not here.
FREE_DRIVE_CONTROLLER = "free_drive_controller"
#: The controllers an arm switches between, only one at a time.
ARM_CONTROLLERS = (SCALED_JOINT_TRAJECTORY_CONTROLLER, CARTESIAN_COMPLIANCE_CONTROLLER, FREE_DRIVE_CONTROLLER)
#: Seconds between "keep free drive on" messages while the free drive controller runs. It ends free drive
#: after 1 s without one (inactive_timeout in crl_husky ur_controllers.yaml), so a lost link or a dead
#: monitor ends it too.
FREE_DRIVE_HEARTBEAT = 0.2
#: multi_arm_safety_sync status older than this, seconds, counts as unknown.
SYNC_STATUS_MAX_AGE = 3.0

#: Largest gap, radians per joint, between the arm and a trajectory's first waypoint (it runs at time 0).
START_TOLERANCE = 0.1
#: Largest gap, metres, between the TCP and a cartesian target.
CARTESIAN_START_TOLERANCE = 0.05
#: Largest turn, radians, between the TCP and a cartesian target; more means a wrong frame.
CARTESIAN_START_ANGLE_TOLERANCE = np.radians(5.0)

# ! Our own guards: the trajectory controller does not check the start, speeds or joint limits.
#: Joint limits of the UR5e, radians, UR_JOINT_NAMES order. +-360 deg except the elbow.
UR_JOINT_LIMITS = np.radians([360.0, 360.0, 180.0, 360.0, 360.0, 360.0])
#: Fastest any joint may be asked to move, rad/s; catches a wrong duration or units.
TRAJECTORY_MAX_JOINT_SPEED = np.radians(60.0)
#: Joint state older than this, seconds, is too old to trust.
JOINT_STATE_MAX_AGE = 0.5
#: Largest joint acceleration, rad/s^2.
TRAJECTORY_MAX_JOINT_ACCELERATION = np.radians(120.0)
#: Upper bounds on one trajectory.
TRAJECTORY_MAX_DURATION = 300.0
TRAJECTORY_MAX_WAYPOINTS = 10_000
#: A trajectory is refused if the arm moved within this many seconds (it would jump in velocity).
MOVING_WITHIN = 0.5
#: Controller state older than this, seconds, is too old to trust "active".
CONTROLLER_STATE_MAX_AGE = 3.0
#: Largest step, metres and radians, between Cartesian targets sent within CARTESIAN_STEP_WINDOW seconds.
CARTESIAN_MAX_STEP = 0.01
CARTESIAN_MAX_STEP_ANGLE = np.radians(3.0)
CARTESIAN_STEP_WINDOW = 0.5
#: Rough UR5e reach, metres, from the shoulder (this high above base_link). Targets beyond it are refused.
UR5E_REACH = 0.85
UR5E_SHOULDER_HEIGHT = 0.163
#: Frame of a target wrench. The controller always reads it in the tool frame (`hand_frame_control`).
TARGET_WRENCH_FRAME = "tool0"
#: Largest target force (N) and torque (N m) `send_target_wrench` sends.
MAX_TARGET_FORCE = 50.0
MAX_TARGET_TORQUE = 5.0
#: How long `hold` takes to brake to a standstill, seconds.
HOLD_DURATION = 0.5
#: A joint that moved further than this, radians, since motion was last seen counts as motion.
MOTION_THRESHOLD = 1e-2
#: Seconds without motion after which the arm counts as done executing.
STILL_AFTER = 1.0

#: Defaults for `joint_move`: never faster than this on any joint, rad/s ...
JOINT_MOVE_MAX_SPEED = np.radians(20.0)
#: ... and never quicker than this, seconds, however small the move.
JOINT_MOVE_MIN_DURATION = 5.0
#: Seconds between waypoints of a joint move.
JOINT_MOVE_WAYPOINT_SPACING = 0.2


#: Defaults for `cartesian_move`: TCP speed, m/s ...
CARTESIAN_MOVE_MAX_SPEED = 0.05
#: ... TCP rotation speed, rad/s ...
CARTESIAN_MOVE_MAX_ROTATION_SPEED = np.radians(15.0)
#: ... and never quicker than this, seconds.
CARTESIAN_MOVE_MIN_DURATION = 5.0


def cartesian_move(start_position: Sequence[float], start_orientation: Sequence[float],
                   target_position: Sequence[float], target_orientation: Sequence[float],
                   max_speed: float = CARTESIAN_MOVE_MAX_SPEED,
                   max_rotation_speed: float = CARTESIAN_MOVE_MAX_ROTATION_SPEED,
                   min_duration: float = CARTESIAN_MOVE_MIN_DURATION):
    """A smooth straight-line TCP move with an even turn (slerp), from rest to rest, on `joint_move`'s cosine profile.

    ! No collision checking, and joints can still move fast near a singularity. For watched moves only.

    Args:
        start_position: TCP position now, metres, in the arm's base_link frame.
        start_orientation: TCP orientation now, quaternion (x, y, z, w).
        target_position: Where the TCP should end, same frame.
        target_orientation: How it should end up turned, quaternion (x, y, z, w).
        max_speed: Peak TCP speed, m/s.
        max_rotation_speed: Peak TCP rotation speed, rad/s.
        min_duration: Shortest allowed duration, seconds.

    Returns:
        tuple[float, Callable]: The duration T in seconds, and `sample(t)`, the
            (position, quaternion) t seconds after the start (the target beyond T).
    """
    start_position = np.asarray(start_position, dtype=float)
    delta = np.asarray(target_position, dtype=float) - start_position
    rotations = Rotation.from_quat([start_orientation, target_orientation])
    angle = float((rotations[0].inv() * rotations[1]).magnitude())
    duration = max(min_duration, np.pi / 2 * float(np.linalg.norm(delta)) / max_speed,
                   np.pi / 2 * angle / max_rotation_speed)
    turn = Slerp([0.0, 1.0], rotations)

    def sample(t: float) -> tuple[np.ndarray, np.ndarray]:
        """(position, quaternion) at t seconds after the start."""
        progress = (1.0 - np.cos(np.pi * min(max(t / duration, 0.0), 1.0))) / 2.0
        return start_position + progress * delta, turn([progress]).as_quat()[0]

    return duration, sample


def joint_move(start: Sequence[float], target: Sequence[float],
               max_speed: float = JOINT_MOVE_MAX_SPEED,
               min_duration: float = JOINT_MOVE_MIN_DURATION) -> tuple[np.ndarray, np.ndarray, float]:
    """A smooth straight-line joint-space move, from rest to rest; all joints start and stop together.

    Every joint follows s(t) = (1 - cos(pi t / T)) / 2.

    ! No collision checking. For watched moves only.

    Args:
        start: Joint angles now, radians, UR_JOINT_NAMES order.
        target: Joint angles to end at, same order.
        max_speed: Peak speed allowed on any joint, rad/s.
        min_duration: Shortest allowed duration, seconds.

    Returns:
        tuple[np.ndarray, np.ndarray, float]: Evenly spaced waypoint positions (N x 6), their velocities
            (N x 6), and the duration T in seconds.
    """
    start, target = np.asarray(start, dtype=float), np.asarray(target, dtype=float)
    delta = target - start
    duration = max(min_duration, np.pi / 2 * float(np.max(np.abs(delta))) / max_speed)
    count = max(2, int(np.ceil(duration / JOINT_MOVE_WAYPOINT_SPACING)) + 1)
    phase = np.linspace(0.0, np.pi, count)[:, None]           # pi * t / T
    positions = start + (1.0 - np.cos(phase)) / 2.0 * delta
    velocities = np.pi / (2.0 * duration) * np.sin(phase) * delta
    return positions, velocities, duration


@dataclass
class ArmState:
    """Measured state of one arm.

    Attributes:
        joint_positions: URDF joint name to position, radians. Empty until the first message.
        tcp_position: TCP position, metres, in the compliance controller's `base_link`, the frame commands use.
        tcp_orientation: TCP quaternion (x, y, z, w), same frame.
        tcp_raw_position: The TCP as the driver reports it, in the UR Base frame.
        tcp_raw_orientation: Its quaternion (x, y, z, w).
        tcp_update_time: ROS time of the last valid TCP pose, seconds.
        sent_target_position: Last Cartesian target sent (commanded, not measured), same frame as `tcp_position`.
        sent_target_orientation: Its quaternion (x, y, z, w).
        sent_target_time: ROS time it was sent, seconds.
            ! In `cartesian_test_mode` these hold the target that *would* have been sent.
        wrench: Force/torque reading (fx, fy, fz, tx, ty, tz).
        digital_in: UR digital inputs by pin.
        is_executing: Whether the arm is moving under a trajectory we sent.
            ? Guessed from motion: True from sending until the joints have been still for STILL_AFTER.
        controllers: The arm controller manager's state.

        * The fields below come from multi_arm_safety_sync (`on_sync_status`).
        status_update_time: ROS time of the last sync status, seconds. Older than SYNC_STATUS_MAX_AGE
            means the fields below are unknown.
        dashboard_up: Whether the arm's dashboard_client runs (False with fake hardware).
        dashboard_connected: Whether the dashboard is connected to the arm.
        safety_mode: UR SafetyMode constant, or None if unknown.
        robot_mode: UR RobotMode constant, or None if unknown. Only RUNNING means the arm can move.
        program_playing: Whether the pendant program plays; the arm follows ROS only while ros_control.urp plays.
        program_name: The loaded program's file name, or "".
        operational: The sync's verdict (connected, safety NORMAL, robot RUNNING); if any arm is not,
            the sync stops every arm.
        problem: What the sync says is wrong with this arm, or "".
        stop_reason: Why the sync last stopped every arm, or "".
        end_effector: State of the mounted tool, or None for a bare arm.
        last_motion_time: ROS time the joints last moved or a trajectory was sent.
        last_update_time: ROS time of the last joint_states.
        joint_states_stats: Rate and gaps of joint_states, to judge the wifi link.
    """

    joint_positions: dict[str, float] = field(default_factory=dict)
    tcp_position: np.ndarray | None = None
    tcp_orientation: np.ndarray | None = None
    tcp_raw_position: np.ndarray | None = None
    tcp_raw_orientation: np.ndarray | None = None
    tcp_update_time: float | None = None
    sent_target_position: np.ndarray | None = None
    sent_target_orientation: np.ndarray | None = None
    sent_target_time: float | None = None
    wrench: np.ndarray | None = None
    digital_in: list[bool] | None = None
    is_executing: bool = False
    controllers: ControllerManagerState = field(default_factory=ControllerManagerState)
    status_update_time: float | None = None
    dashboard_up: bool = False
    dashboard_connected: bool = False
    safety_mode: int | None = None
    robot_mode: int | None = None
    program_playing: bool = False
    program_name: str = ""
    operational: bool = False
    problem: str = ""
    stop_reason: str = ""
    end_effector: EndEffectorState | None = None
    last_motion_time: float | None = None
    last_update_time: float | None = None
    joint_states_stats: StreamStats = field(default_factory=StreamStats)


class ArmInterface:
    """ROS2 interface to one UR arm and the tool mounted on it.

    Attributes:
        config: This arm's configuration.
        state: Measured state, written only by callbacks.
        controllers: Tracks and switches this arm's controllers.
        end_effector: The mounted tool, or None for a bare arm.
    """

    def __init__(self, node: Node, robot_namespace: str, config: ArmConfig, base_in_husky: Pose,
                 urdf_problem: str | None = None):
        """Create every subscription, publisher and client of one arm.

        Args:
            node: The monitor node.
            robot_namespace: The robot's namespace, e.g. "a200_0806".
            config: This arm.
            base_in_husky: This arm's base_link in the husky frame.
            urdf_problem: Why the URDF breaks the stock UR frames, or None. If set, Cartesian targets are refused.
        """
        self._node = node
        self.config = config
        #: This arm's base_link (the controller's target frame) in the husky frame.
        self.base_in_husky = base_in_husky
        self.urdf_problem = urdf_problem
        self.state = ArmState()
        #: When each test-mode banner was last logged, ROS time.
        self._test_mode_logged: dict[str, float] = {}
        #: Joint positions, by URDF name, where motion was last detected.
        self._motion_reference: dict[str, float] = {}
        #: Whether free drive was being kept on at the last heartbeat, to log the change.
        self._free_drive_on = False
        self._namespace = f"/{robot_namespace}/{config.ros_namespace}"
        self._ros = RosConnections(node)

        self.controllers = ControllerManagerInterface(
            node, self._namespace, ARM_CONTROLLERS, self.state.controllers)
        self.end_effector: EndEffector | None = make_end_effector(node, robot_namespace, config)
        self.state.end_effector = self.end_effector.state if self.end_effector else None
        self._connect()

    def _connect(self) -> None:
        """Create the arm's subscriptions, publishers and clients."""
        namespace = self._namespace
        # * Driver topics come through a rate limiter on the robot, to spare the wifi.
        measured = f"{namespace}/rate_limiter"
        sensor = qos_profile_sensor_data

        # --- --- measurements --- ---
        self._ros.subscription(JointState, f"{measured}/joint_states", self._on_joint_state, RECORDED_QOS)
        # * The rate limiter extracts "tcp_pose" from dynamic_joint_states, which is large and slow to decode.
        self._ros.subscription(PoseStamped, f"{measured}/tcp_pose", self._on_tcp_pose, RECORDED_QOS)
        self._ros.subscription(IOStates, f"{measured}/io_and_status_controller/io_states",
                               self._on_io_states, sensor)
        self._ros.subscription(WrenchStamped, f"{measured}/ft_sensor_wrench", self._on_wrench, RECORDED_QOS)

        # --- --- commands --- ---
        self._trajectory = self._ros.publisher(
            JointTrajectory, f"{namespace}/{SCALED_JOINT_TRAJECTORY_CONTROLLER}/joint_trajectory")
        self._target_frame = self._ros.publisher(PoseStamped, f"{namespace}/target_frame")
        self._target_wrench = self._ros.publisher(WrenchStamped, f"{namespace}/target_wrench")
        self._zero_ft = self._ros.client(Trigger, f"{namespace}/io_and_status_controller/zero_ftsensor")
        self._free_drive = self._ros.publisher(Bool, f"{namespace}/{FREE_DRIVE_CONTROLLER}/enable_freedrive_mode")
        self._ros.timer(FREE_DRIVE_HEARTBEAT, self._keep_free_drive)
        # Safety and program state arrive via HuskyRobotInterface._on_sync_status.

    def reconnect(self) -> None:
        """Destroy this arm's and its tool's ROS entities and create them again. Keeps `state`."""
        self._ros.destroy_all()
        self._connect()
        self.controllers.reconnect()
        if self.end_effector is not None:
            self.end_effector.reconnect()

    # --- --- --- --- --- COMMANDS --- --- --- --- ---
    # ! Main thread only. Never from a viser callback.

    def build_trajectory(self, positions: Sequence[Sequence[float]], duration: float,
                         velocities: Sequence[Sequence[float]] | None = None) -> JointTrajectory | None:
        """Turn waypoints into a trajectory message, after checking it is safe.

        Args:
            positions: At least two waypoints of six joint angles each, radians,
                UR_JOINT_NAMES order. The first must match where the arm is now.
            duration: Total time, seconds. Waypoints are spread evenly across it.
            velocities: Joint velocities per waypoint, same shape, or None.

        Returns:
            JointTrajectory | None: The message, or None (reason logged) if a check fails.
        """
        refusal = self._check_trajectory(positions, duration, velocities)
        if refusal is not None:
            self._node.get_logger().error(f"arm {self.config.name}: not sending a trajectory: {refusal}")
            return None

        step = duration / (len(positions) - 1)
        trajectory = JointTrajectory()
        trajectory.joint_names = list(UR_JOINT_NAMES)
        for index, waypoint in enumerate(positions):
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in waypoint]
            if velocities is not None:
                point.velocities = [float(value) for value in velocities[index]]
            seconds = step * index
            point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9))
            trajectory.points.append(point)
        return trajectory

    def _check_trajectory(self, positions: Sequence[Sequence[float]], duration: float,
                          velocities: Sequence[Sequence[float]] | None) -> str | None:
        """Run every check a trajectory must pass before it is sent.

        Args:
            positions: Waypoints, as for `build_trajectory`.
            duration: Total time, seconds.
            velocities: Waypoint velocities, or None.

        Returns:
            str | None: Why the trajectory is refused, or None if it may be sent.
        """
        refusal = self._motion_refusal(SCALED_JOINT_TRAJECTORY_CONTROLLER)
        if refusal is not None:
            return refusal

        # --- the input itself
        try:
            positions = np.asarray(positions, dtype=float)
        except (TypeError, ValueError):
            return "waypoints are not a list of equal-length lists of numbers"
        if positions.ndim != 2 or positions.shape[1] != len(UR_JOINT_NAMES) or len(positions) < 2:
            return f"need at least two waypoints of {len(UR_JOINT_NAMES)} joints, got shape {positions.shape}"
        if not (np.isfinite(duration) and 0.0 < duration <= TRAJECTORY_MAX_DURATION):
            return f"duration must be positive and at most {TRAJECTORY_MAX_DURATION:.0f} s, got {duration}"
        if len(positions) > TRAJECTORY_MAX_WAYPOINTS:
            return f"{len(positions)} waypoints, over the limit of {TRAJECTORY_MAX_WAYPOINTS}"
        if not np.all(np.isfinite(positions)):
            return "waypoints contain NaN or infinity"
        if velocities is not None:
            try:
                velocities = np.asarray(velocities, dtype=float)
            except (TypeError, ValueError):
                return "velocities are not a list of equal-length lists of numbers"
            if velocities.shape != positions.shape or not np.all(np.isfinite(velocities)):
                return f"velocities must be finite and shaped like the waypoints {positions.shape}"
        outside = np.abs(positions) > UR_JOINT_LIMITS
        if np.any(outside):
            joint = UR_JOINT_NAMES[int(np.argwhere(outside)[0][1])]
            return f"a waypoint puts {joint} past its limit"

        # --- speed: between waypoints, and in the given velocities
        step = duration / (len(positions) - 1)
        speed = float(np.max(np.abs(np.diff(positions, axis=0)))) / step
        if velocities is not None:
            speed = max(speed, float(np.max(np.abs(velocities))))
        if speed > TRAJECTORY_MAX_JOINT_SPEED:
            return (f"needs {np.degrees(speed):.0f} deg/s on some joint, over the "
                    f"{np.degrees(TRAJECTORY_MAX_JOINT_SPEED):.0f} deg/s limit; make it slower")

        # --- acceleration: from the given velocities, and from the waypoint spacing
        acceleration = 0.0
        if velocities is not None:
            acceleration = float(np.max(np.abs(np.diff(velocities, axis=0)))) / step
        if len(positions) >= 3:
            acceleration = max(acceleration, float(np.max(np.abs(np.diff(positions, n=2, axis=0)))) / step ** 2)
        if acceleration > TRAJECTORY_MAX_JOINT_ACCELERATION:
            return (f"needs {np.degrees(acceleration):.0f} deg/s^2 on some joint, over the "
                    f"{np.degrees(TRAJECTORY_MAX_JOINT_ACCELERATION):.0f} deg/s^2 limit")
        # ! Must end at rest, or the arm would coast.
        if velocities is not None and float(np.max(np.abs(velocities[-1]))) > 1e-6:
            return "the last waypoint's velocity is not zero; a trajectory must end at rest"

        # --- the start must be where the arm is now, by a recent measurement
        current = self.joint_vector()
        if current is None:
            return "no joint state yet"
        age = self._now() - self.state.last_update_time
        if age > JOINT_STATE_MAX_AGE:
            return f"joint state is {age:.1f} s old; cannot check the start against it"
        if self.state.last_motion_time is not None and self._now() - self.state.last_motion_time < MOVING_WITHIN:
            return "the arm is still moving; Hold first, then send once it has stopped"
        if not np.allclose(current, positions[0], atol=START_TOLERANCE):
            return (f"arm is not at the first waypoint: {np.round(current, 3)} "
                    f"vs {np.round(positions[0], 3)}")
        return None

    def _motion_refusal(self, controller: str) -> str | None:
        """Run the checks every motion command shares.

        Args:
            controller: The controller the command is for.

        Returns:
            str | None: Why it is refused, or None if it may go ahead.
        """
        state = self.state.controllers
        if not self.controllers.is_active(controller):
            return f"{controller} is not active (active: {state.active!r})"
        if state.last_update_time is None or self._now() - state.last_update_time > CONTROLLER_STATE_MAX_AGE:
            return ("the controller manager has not answered for a while, so which controller "
                    "runs is not known")
        updated = self.state.status_update_time
        if updated is None or self._now() - updated > SYNC_STATUS_MAX_AGE or not self.state.dashboard_up:
            # ? Not refused: fake hardware has no dashboard and the robot enforces safety itself.
            self._node.get_logger().warning(
                f"arm {self.config.name}: no dashboard state from multi_arm_safety_sync, safety not checked",
                throttle_duration_sec=5.0)
            return None
        if not self.state.operational:
            return f"the arm is not operational: {self.state.problem}"
        return None

    def hold(self) -> bool:
        """Stop the arm at its measured joints or TCP, under whichever controller is running.

        - Under Cartesian compliance, the caller must stop sending new targets.
        - Refused on stale measurements (an old position could mean a jump); use the teach pendant then.
        ! A target wrench stays applied; zero it separately.

        Returns:
            bool: True if sent. False (reason logged) if neither controller runs or the measurement is stale.
        """
        log = self._node.get_logger()
        if self.controllers.is_active(CARTESIAN_COMPLIANCE_CONTROLLER):
            if self.state.tcp_position is None or \
                    self._now() - self.state.tcp_update_time > JOINT_STATE_MAX_AGE:
                log.error(f"arm {self.config.name}: HOLD NOT SENT, TCP pose missing or stale; "
                          f"stop the arm from the teach pendant")
                return False
            sent = self.send_cartesian_target(self.state.tcp_position, self.state.tcp_orientation, hold=True)
            if sent:
                log.info(f"arm {self.config.name}: hold sent")
            return sent
        if not self.controllers.is_active(SCALED_JOINT_TRAJECTORY_CONTROLLER):
            log.warning(f"arm {self.config.name}: hold not sent, neither motion controller is active")
            return False
        here = self.joint_vector()
        if here is None or self._now() - self.state.last_update_time > JOINT_STATE_MAX_AGE:
            log.error(f"arm {self.config.name}: HOLD NOT SENT, joint state missing or stale; "
                      f"stop the arm from the teach pendant")
            return False
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in here]
        point.velocities = [0.0] * len(UR_JOINT_NAMES)
        point.time_from_start = Duration(sec=int(HOLD_DURATION), nanosec=int((HOLD_DURATION % 1.0) * 1e9))
        trajectory = JointTrajectory(joint_names=list(UR_JOINT_NAMES), points=[point])
        self._trajectory.publish(trajectory)
        log.info(f"arm {self.config.name}: hold sent")
        return True

    def send_joint_trajectory(self, positions: Sequence[Sequence[float]], duration: float,
                              velocities: Sequence[Sequence[float]] | None = None) -> bool:
        """Send a joint trajectory. Arguments as for `build_trajectory`.

        Returns:
            bool: True if sent. On False nothing was sent; the log says why.
        """
        trajectory = self.build_trajectory(positions, duration, velocities)
        if trajectory is None:
            return False
        self._trajectory.publish(trajectory)
        self.mark_executing()
        return True

    def send_joint_move(self, target: Sequence[float], max_speed: float = JOINT_MOVE_MAX_SPEED,
                        min_duration: float = JOINT_MOVE_MIN_DURATION) -> float | None:
        """Move smoothly from the current joints to `target`. See `joint_move`.

        Args:
            target: Joint angles to end at, radians, UR_JOINT_NAMES order.
            max_speed: Peak speed allowed on any joint, rad/s.
            min_duration: Shortest allowed duration, seconds.

        Returns:
            float | None: The duration in seconds, or None if nothing was sent (reason logged).
        """
        here = self.joint_vector()
        if here is None:
            self._node.get_logger().error(f"arm {self.config.name}: not moving, no joint state yet")
            return None
        positions, velocities, duration = joint_move(here, target, max_speed, min_duration)
        if not self.send_joint_trajectory(positions, duration, velocities):
            return None
        return duration

    def send_cartesian_target(self, position: Sequence[float], orientation: Sequence[float],
                              hold: bool = False) -> bool:
        """Send a TCP target to the compliance controller.

        Args:
            position: Target position in the arm's base_link frame, metres.
            orientation: Target orientation, quaternion (x, y, z, w).
            hold: True only from `hold`; its target is the reported TCP, so a URDF problem does not stop it.

        ! The controller does not interpolate: send bigger moves as many close targets, one per tick
          (`cartesian_move`).

        Returns:
            bool: True if sent. False (reason logged) if a check fails.
        """
        log = self._node.get_logger()
        position = np.asarray(position, dtype=float)
        orientation = np.asarray(orientation, dtype=float)
        refusal = self._motion_refusal(CARTESIAN_COMPLIANCE_CONTROLLER)
        if refusal is not None:
            pass
        elif self.urdf_problem is not None and not hold:
            refusal = "the URDF breaks the stock UR frames (logged at startup, doc/ur_frames.md)"
        elif position.shape != (3,) or orientation.shape != (4,) or not (
                np.all(np.isfinite(position)) and np.all(np.isfinite(orientation))):
            refusal = "position must be 3 finite numbers and orientation a finite quaternion"
        elif abs(np.linalg.norm(orientation) - 1.0) > 1e-3:
            refusal = "orientation is not a unit quaternion"
        elif self.state.tcp_position is None:
            refusal = "no TCP pose yet"
        elif self._now() - self.state.tcp_update_time > JOINT_STATE_MAX_AGE:
            refusal = f"TCP pose is {self._now() - self.state.tcp_update_time:.1f} s old"
        elif not np.allclose(self.state.tcp_position, position, atol=CARTESIAN_START_TOLERANCE):
            refusal = (f"target too far from the TCP: {np.round(self.state.tcp_position, 3)} "
                       f"vs {np.round(position, 3)}")
        elif (turn := (Rotation.from_quat(self.state.tcp_orientation).inv()
                       * Rotation.from_quat(orientation)).magnitude()) > CARTESIAN_START_ANGLE_TOLERANCE:
            refusal = (f"target is turned {np.degrees(turn):.1f} deg from the TCP, over "
                       f"{np.degrees(CARTESIAN_START_ANGLE_TOLERANCE):.0f} deg")
        elif np.linalg.norm(position - [0.0, 0.0, UR5E_SHOULDER_HEIGHT]) > UR5E_REACH:
            refusal = f"target is out of reach ({UR5E_REACH} m from the shoulder)"
        else:
            refusal = self._cartesian_step_refusal(position, orientation)
        if refusal is not None:
            log.error(f"arm {self.config.name}: not sending a cartesian target: {refusal}")
            return False
        if self.state.sent_target_time is None or self._now() - self.state.sent_target_time > CARTESIAN_STEP_WINDOW:
            # First target of a move: log how it compares with the driver's TCP.
            log.info(self.target_vs_reported(position, orientation))
        if self.config.cartesian_test_mode:
            # Recorded anyway, so the 3D view can show it.
            self.state.sent_target_position, self.state.sent_target_orientation = position, orientation
            self.state.sent_target_time = self._now()
            self._log_test_mode("target frame", f"p {np.round(position, 4)} m  q {np.round(orientation, 4)}")
            return False
        message = PoseStamped()
        message.header.frame_id = "base_link"
        message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, position)
        (message.pose.orientation.x, message.pose.orientation.y,
         message.pose.orientation.z, message.pose.orientation.w) = map(float, orientation)
        self._target_frame.publish(message)
        self.state.sent_target_position, self.state.sent_target_orientation = position, orientation
        self.state.sent_target_time = self._now()
        return True

    def to_husky(self, position: Sequence[float], orientation: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
        """Convert a pose from this arm's base_link to the husky frame.

        Args:
            position: Metres, in the arm's base_link.
            orientation: Quaternion (x, y, z, w).

        Returns:
            tuple[np.ndarray, np.ndarray]: Position and quaternion in the husky frame.
        """
        pose = compose(self.base_in_husky, Pose.from_arrays(position, orientation))
        return np.array(pose.position), np.array(pose.orientation)

    def from_husky(self, position: Sequence[float], orientation: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
        """Convert a pose from the husky frame to this arm's base_link. Inverse of `to_husky`."""
        pose = compose(invert(self.base_in_husky), Pose.from_arrays(position, orientation))
        return np.array(pose.position), np.array(pose.orientation)

    def target_vs_reported(self, position: np.ndarray, orientation: np.ndarray) -> str:
        """Compare a target with the raw reported TCP, both in UR Base, as one log line.

        For a move's first target, which starts at the TCP, any difference is a frame error.

        Args:
            position: The target, metres, in the compliance controller's base_link.
            orientation: Its quaternion (x, y, z, w).
        """
        raw_p = self.state.tcp_raw_position
        target_p = BASE_LINK_FROM_UR_BASE.inv().apply(position)
        target_q = BASE_LINK_FROM_UR_BASE.inv() * Rotation.from_quat(orientation)
        distance = float(np.linalg.norm(target_p - raw_p)) * 1e3
        angle = float(np.degrees((Rotation.from_quat(self.state.tcp_raw_orientation).inv() * target_q).magnitude()))
        return (f"arm {self.config.name}: first target vs reported TCP, both in UR Base: "
                f"target p {np.round(target_p * 1e3, 1)} mm, reported p {np.round(raw_p * 1e3, 1)} mm, "
                f"difference {distance:.2f} mm {angle:.3f} deg")

    def _cartesian_step_refusal(self, position: np.ndarray, orientation: np.ndarray) -> str | None:
        """Return why a target jumps too far from the one sent just before it, or None."""
        state = self.state
        if state.sent_target_time is None or self._now() - state.sent_target_time > CARTESIAN_STEP_WINDOW:
            return None  # nothing sent just now: the TCP check covers it
        step = float(np.linalg.norm(position - state.sent_target_position))
        turn = float((Rotation.from_quat(state.sent_target_orientation).inv() * Rotation.from_quat(orientation))
                     .magnitude())
        if step > CARTESIAN_MAX_STEP or turn > CARTESIAN_MAX_STEP_ANGLE:
            return (f"jumps {step * 1e3:.1f} mm / {np.degrees(turn):.1f} deg from the target sent "
                    f"just before; at most {CARTESIAN_MAX_STEP * 1e3:.0f} mm / "
                    f"{np.degrees(CARTESIAN_MAX_STEP_ANGLE):.0f} deg per step")
        return None

    def _log_test_mode(self, what: str, detail: str) -> None:
        """Log a banner, at most once a second per command, that a command passed but was not sent.

        Args:
            what: Which command, e.g. "target frame".
            detail: Its values, for the log.
        """
        # ! Throttled by hand: rclpy throttles per call site, so one banner would silence the other.
        now = self._now()
        if now - self._test_mode_logged.get(what, -np.inf) < 1.0:
            return
        self._test_mode_logged[what] = now
        rule = "#" * 78
        self._node.get_logger().warning(
            f"\n{rule}\n"
            f"##  TEST MODE -- arm {self.config.name}: {what} passed every check, NOT SENT\n"
            f"##  {detail}\n"
            f"##  Send for real (ArmConfig.cartesian_test_mode=False) only once the startup\n"
            f"##  log shows no URDF frame error for this arm (doc/ur_frames.md).\n"
            f"{rule}")

    def send_target_wrench(self, force: Sequence[float],
                           torque: Sequence[float] = (0.0, 0.0, 0.0)) -> bool:
        """Send a target wrench to the compliance controller.

        ! In the tool frame (tool0), not the arm's base, so it turns with the tool.
        ! Stays applied until another wrench is sent, even through a hold. Send zeros to remove it.

        Args:
            force: Force in tool0, newtons.
            torque: Torque in tool0, newton-metres.

        Returns:
            bool: True if sent. False (reason logged) if the controller is not
                running or the wrench is not finite or over the limits.
        """
        force, torque = np.asarray(force, dtype=float), np.asarray(torque, dtype=float)
        refusal = self._motion_refusal(CARTESIAN_COMPLIANCE_CONTROLLER)
        if refusal is not None:
            pass
        elif force.shape != (3,) or torque.shape != (3,) or not (
                np.all(np.isfinite(force)) and np.all(np.isfinite(torque))):
            refusal = "force and torque must be 3 finite numbers each"
        elif np.linalg.norm(force) > MAX_TARGET_FORCE or np.linalg.norm(torque) > MAX_TARGET_TORQUE:
            refusal = f"over the limit of {MAX_TARGET_FORCE:.0f} N / {MAX_TARGET_TORQUE:.0f} N m"
        if refusal is not None:
            self._node.get_logger().error(f"arm {self.config.name}: not sending a wrench: {refusal}")
            return False
        if self.config.cartesian_test_mode:
            self._log_test_mode("target force", f"F {np.round(force, 2)} N  T {np.round(torque, 3)} N m (tool0)")
            return False
        message = WrenchStamped()
        message.header.frame_id = TARGET_WRENCH_FRAME
        message.wrench.force.x, message.wrench.force.y, message.wrench.force.z = map(float, force)
        message.wrench.torque.x, message.wrench.torque.y, message.wrench.torque.z = map(float, torque)
        self._target_wrench.publish(message)
        return True

    def zero_ft_sensor(self) -> bool:
        """Re-zero the force/torque sensor, taring the current reading.

        ! Only when the arm holds nothing but its tool: zeroing while gripping a bar tares away its weight.

        Returns:
            bool: True if the request went out. False if the service is not up.
        """
        if not self._zero_ft.service_is_ready():
            self._node.get_logger().error(f"arm {self.config.name}: zero_ftsensor is not available")
            return False
        self._zero_ft.call_async(Trigger.Request())
        return True

    def _keep_free_drive(self) -> None:
        """While the free drive controller runs because we switched to it, keep free drive on; log on and off.

        ! Only after our own switch: a controller left active by another tool or an earlier monitor run must not
          start free drive by itself. Switching away, the soft stop or a switch from outside ends it.
        """
        state = self.controllers.state
        on = self.controllers.is_active(FREE_DRIVE_CONTROLLER) and state.requested == FREE_DRIVE_CONTROLLER
        if on != self._free_drive_on:
            self._node.get_logger().info(f"arm {self.config.name}: free drive {'on' if on else 'off'}")
            self._free_drive_on = on
        if on:
            self._free_drive.publish(Bool(data=True))

    def mark_executing(self) -> None:
        """Record that a trajectory was just sent, so a waiter checking at once doesn't see it as finished."""
        self.state.is_executing = True
        self.state.last_motion_time = self._now()

    # --- --- --- --- --- QUERIES --- --- --- --- ---

    def joint_vector(self) -> np.ndarray | None:
        """Current joint angles in UR_JOINT_NAMES order, or None before all six are known."""
        prefix = f"{self.config.name}_"
        values = [self.state.joint_positions.get(prefix + name) for name in UR_JOINT_NAMES]
        return None if any(value is None for value in values) else np.array(values)

    # --- --- --- --- --- CALLBACKS (main thread) --- --- --- --- ---
    # ! Only write into self.state here: no planning, drawing or file IO.

    def _on_joint_state(self, message: JointState) -> None:
        """Store joint positions under URDF names and update `is_executing`."""
        now = self._now()
        prefix = f"{self.config.name}_"
        moved = False
        for name, position in zip(message.name, message.position):
            key = prefix + name
            self.state.joint_positions[key] = float(position)
            reference = self._motion_reference.setdefault(key, float(position))
            if abs(position - reference) > MOTION_THRESHOLD:
                moved = True
        # ? Compared with where motion was last seen, not the previous message, so slow moves count too.
        if moved:
            self._motion_reference = {key: self.state.joint_positions[key] for key in self._motion_reference}
            self.state.last_motion_time = now
        elif self.state.last_motion_time is None or now - self.state.last_motion_time > STILL_AFTER:
            self.state.is_executing = False
        self.state.last_update_time = now
        stamp = message.header.stamp
        self.state.joint_states_stats.add(now, stamp.sec + stamp.nanosec * 1e-9)

    def _on_tcp_pose(self, message: PoseStamped) -> None:
        """Store the driver's "tcp_pose", raw (UR Base frame) and in the controller's `base_link`."""
        p, o = message.pose.position, message.pose.orientation
        position = np.array([p.x, p.y, p.z])
        quaternion = np.array([o.x, o.y, o.z, o.w])
        # ! An all-zero quaternion means "no pose yet" (startup, fake hardware): keep the last good one.
        if not np.linalg.norm(quaternion) > 1e-6:
            return
        self.state.tcp_raw_position, self.state.tcp_raw_orientation = position, quaternion / np.linalg.norm(quaternion)
        self.state.tcp_position = BASE_LINK_FROM_UR_BASE.apply(position)
        self.state.tcp_orientation = (BASE_LINK_FROM_UR_BASE * Rotation.from_quat(quaternion)).as_quat()
        self.state.tcp_update_time = self._now()

    def _on_io_states(self, message: IOStates) -> None:
        """Store the digital inputs by pin."""
        digital_in = [False] * len(message.digital_in_states)
        for pin in message.digital_in_states:
            if 0 <= pin.pin < len(digital_in):
                digital_in[pin.pin] = bool(pin.state)
        self.state.digital_in = digital_in

    def on_sync_status(self, status: ArmStatus, stop_reason: str) -> None:
        """Store this arm's entry of a multi_arm_safety_sync status.

        Args:
            status: This arm's entry of the status.
            stop_reason: Why the sync last stopped every arm, or "".
        """
        state = self.state
        state.dashboard_up = status.dashboard_up
        state.dashboard_connected = status.connected
        state.safety_mode = int(status.safety_mode) if status.safety_mode_known else None
        state.robot_mode = int(status.robot_mode) if status.robot_mode_known else None
        state.program_playing = status.program_playing
        state.program_name = status.program_name
        state.operational = status.operational
        state.problem = status.problem
        state.stop_reason = stop_reason
        state.status_update_time = self._now()

    def _on_wrench(self, message: WrenchStamped) -> None:
        """Store the force/torque reading."""
        f, t = message.wrench.force, message.wrench.torque
        self.state.wrench = np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def _now(self) -> float:
        """ROS time in seconds (follows bag playback)."""
        return self._node.get_clock().now().nanoseconds * 1e-9
