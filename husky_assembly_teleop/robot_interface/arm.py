"""
One UR arm: its measured state, its controllers, and commanding it.

Every arm gets the same set of topics and services, whichever controller it is
running. Which commands are allowed depends on that controller, which is runtime
state, so each command checks it instead of the constructor deciding what exists.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import numpy as np
from builtin_interfaces.msg import Duration
from control_msgs.msg import DynamicJointState
from crl_husky_msgs.msg import ArmStatus
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_msgs.msg import IOStates

from ..config import ArmConfig
from .connections import RosConnections
from .controller_manager import ControllerManagerInterface, ControllerManagerState
from .end_effectors import EndEffector, EndEffectorState, make_end_effector
from .frames import Pose, compose, invert
from .ur_frames import BASE_LINK_FROM_UR_BASE

#: Joint names as the UR driver publishes and expects them, in its order. The
#: URDF has the same joints with the arm's prefix in front.
UR_JOINT_NAMES = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")

SCALED_JOINT_TRAJECTORY_CONTROLLER = "scaled_joint_trajectory_controller"
CARTESIAN_COMPLIANCE_CONTROLLER = "cartesian_compliance_controller"
#: The controllers an arm switches between, only one at a time.
ARM_CONTROLLERS = (SCALED_JOINT_TRAJECTORY_CONTROLLER, CARTESIAN_COMPLIANCE_CONTROLLER)
#: The status from multi_arm_safety_sync older than this, seconds, counts as
#: unknown. The sync publishes it every second and on every change.
SYNC_STATUS_MAX_AGE = 3.0

#: How far, in radians per joint, the arm may be from a trajectory's first
#: waypoint. The first waypoint runs at time 0, so a bigger gap is a jump.
START_TOLERANCE = 0.1
#: How far, in metres, the TCP may be from a cartesian target. Same reason.
CARTESIAN_START_TOLERANCE = 0.05
#: How far, in radians, the TCP may be turned from a cartesian target. A
#: streamed move turns at most CARTESIAN_MOVE_MAX_ROTATION_SPEED, a degree
#: per tick; a bigger turn means the target is in the wrong frame.
CARTESIAN_START_ANGLE_TOLERANCE = np.radians(5.0)

# * Cartesian frames: see BASE_LINK_FROM_UR_BASE in ur_frames.py and doc/ur_frames.md.

# ! Our own guards. The trajectory controller checks only the message's form
#   (joint names, points present, times increasing, array sizes). It does not
#   check the start against the arm, speeds, or joint limits -- so we do,
#   before anything is sent. See build_trajectory.
#: Joint limits of the UR5e, radians, UR_JOINT_NAMES order. +-360 deg except the elbow.
UR_JOINT_LIMITS = np.radians([360.0, 360.0, 180.0, 360.0, 360.0, 360.0])
#: No joint may be asked to move faster than this, rad/s -- between waypoints
#: or in a waypoint's velocity. A sanity limit well above what the panel sends
#: (JOINT_MOVE_MAX_SPEED), to catch a wrong duration or a unit mix-up.
TRAJECTORY_MAX_JOINT_SPEED = np.radians(60.0)
#: Joint state older than this, seconds, is too old to check a start against.
JOINT_STATE_MAX_AGE = 0.5
#: No joint may be asked to speed up or slow down faster than this, rad/s^2.
TRAJECTORY_MAX_JOINT_ACCELERATION = np.radians(120.0)
#: Upper bounds on one trajectory, to catch a runaway caller.
TRAJECTORY_MAX_DURATION = 300.0
TRAJECTORY_MAX_WAYPOINTS = 10_000
#: A trajectory is refused while the arm moved within this many seconds:
#: sent to a moving arm it replaces the motion with a jump in velocity.
MOVING_WITHIN = 0.5
#: Controller state older than this, seconds, is too old to trust "active".
#: It is polled every second (controller_manager.REFRESH_PERIOD).
CONTROLLER_STATE_MAX_AGE = 3.0
#: Two Cartesian targets sent within CARTESIAN_STEP_WINDOW seconds of each other
#: may be at most this far apart, metres and radians. A streamed move steps a
#: few mm per tick; a bigger step means the sender went wrong.
CARTESIAN_MAX_STEP = 0.01
CARTESIAN_MAX_STEP_ANGLE = np.radians(3.0)
CARTESIAN_STEP_WINDOW = 0.5
#: Rough UR5e reach, metres, measured from the shoulder, which sits this high
#: above the arm's base_link. Targets beyond it are refused.
UR5E_REACH = 0.85
UR5E_SHOULDER_HEIGHT = 0.163
#: The frame a target wrench is applied in: the compliance controller's
#: end_effector_link. Its `hand_frame_control` parameter defaults to true and
#: crl-husky's ur_controllers.yaml does not set it, so the controller reads the
#: wrench in the tool frame -- and ignores the message's frame_id altogether.
#: Sent as the frame_id anyway, so the message says what it means.
TARGET_WRENCH_FRAME = "tool0"
#: Largest target force `send_target_wrench` will send, newtons, and torque, N m.
MAX_TARGET_FORCE = 50.0
MAX_TARGET_TORQUE = 5.0
#: How long `hold` takes to brake to a standstill, seconds.
HOLD_DURATION = 0.5
#: A joint that has moved further than this, radians, since motion was last
#: seen counts as motion. Measured from where it last moved, not from the
#: previous message, so a slow move still adds up to motion.
MOTION_THRESHOLD = 1e-2
#: Seconds without motion after which the arm counts as done executing.
STILL_AFTER = 1.0

#: Defaults for `joint_move`: never faster than this on any joint, rad/s ...
JOINT_MOVE_MAX_SPEED = np.radians(20.0)
#: ... and never quicker than this, seconds, however small the move.
JOINT_MOVE_MIN_DURATION = 5.0
#: Seconds between waypoints of a joint move. Fine enough that the controller's
#: interpolation between them follows the smooth profile closely.
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
    """A smooth move of the TCP: a straight line, turning evenly, from rest to rest.

    The same cosine profile as `joint_move`, s(t) = (1 - cos(pi t / T)) / 2, on
    both the distance along the line and the angle of the turn (a slerp), so
    both start and stop together at zero speed. The duration is chosen from
    whichever needs longer:

        T = max(min_duration, pi / 2 * distance / max_speed,
                pi / 2 * angle / max_rotation_speed)

    ! No collision checking, and a straight line for the TCP says nothing about
      the joints: near a singularity they can still move fast. For moves an
      operator sets up by hand and watches.

    Args:
        start_position: TCP position now, metres, in the arm's base_link frame.
        start_orientation: TCP orientation now, quaternion (x, y, z, w).
        target_position: Where the TCP should end, same frame.
        target_orientation: How it should end up turned, quaternion (x, y, z, w).
        max_speed: Peak TCP speed, m/s.
        max_rotation_speed: Peak TCP rotation speed, rad/s.
        min_duration: Shortest allowed duration, seconds.

    Returns:
        tuple[float, Callable]: The duration T in seconds, and `sample(t)`,
            which gives the (position, quaternion) the TCP should be at t
            seconds after the start. Beyond T it gives the target.
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
    """A smooth straight-line move in joint space, from rest to rest.

    Every joint follows the same cosine profile, s(t) = (1 - cos(pi t / T)) / 2,
    so all joints start and stop together and at zero speed. Its peak speed is
    pi / 2 times the average, reached halfway, so the duration is chosen from
    the joint that moves furthest:

        T = max(min_duration, pi / 2 * max|target - start| / max_speed)

    ! No collision checking. This is a straight line in joint space, for moves
      an operator sets up by hand and watches.

    Args:
        start: Joint angles now, radians, UR_JOINT_NAMES order.
        target: Joint angles to end at, same order.
        max_speed: Peak speed allowed on any joint, rad/s.
        min_duration: Shortest allowed duration, seconds.

    Returns:
        tuple[np.ndarray, np.ndarray, float]: Waypoint positions (N x 6), their
            velocities (N x 6), and the duration T in seconds. Waypoints are
            evenly spaced in time, as `build_trajectory` expects.
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
        joint_positions: Joint name to position, radians. Keyed by URDF name
            (e.g. "left_ur_arm_elbow_joint"), so RobotScene and the viewer can
            use it directly. Absent until the first message.
        tcp_position: TCP position, metres, in the compliance controller's
            `base_link` (BASE_LINK_FROM_UR_BASE) -- the frame every Cartesian
            command is sent in. None before the first valid pose.
        tcp_orientation: TCP orientation, quaternion (x, y, z, w), same frame.
            Set together with `tcp_position`.
        tcp_raw_position: The TCP exactly as the driver reports it, in the UR
            controller's Base frame, for checking. Set together with the above.
        tcp_raw_orientation: Its orientation, quaternion (x, y, z, w).
        tcp_update_time: ROS time of the last valid TCP pose, seconds, or None
            before any. The TCP comes on its own topic, so it has its own age.
        sent_target_position: The last Cartesian target we sent the compliance
            controller, metres, same frame as `tcp_position`, or None if none
            was. Commanded, not measured -- kept so it can be shown and checked.
        sent_target_orientation: Its orientation, quaternion (x, y, z, w).
        sent_target_time: ROS time it was sent, seconds, or None.
            ! In test mode (ArmConfig.cartesian_test_mode) these hold the target
              that *would* have been sent, so it can still be shown.
        wrench: Force/torque sensor reading (fx, fy, fz, tx, ty, tz), or None
            before the first one.
        digital_in: UR digital inputs, by pin, or None before the first io_states.
        is_executing: Whether the arm is moving under a trajectory we sent.
            ? Guessed from motion, because trajectories go over a topic (more
              reliable than the action on busy wifi, and not every controller
              has an action) and a topic has no "done". True from the moment
              one is sent until the joints have been still for STILL_AFTER.
        controllers: The arm controller manager's state.

        * The fields below come from multi_arm_safety_sync on the robot, the only
          node that talks to the UR dashboard. See `on_sync_status`.
        status_update_time: ROS time the last sync status arrived, seconds, or
            None before any. Older than SYNC_STATUS_MAX_AGE, the fields below
            count as unknown.
        dashboard_up: Whether the arm's dashboard_client runs. False with fake
            hardware, which has none.
        dashboard_connected: Whether the dashboard is connected to the arm.
        safety_mode: The UR safety mode, a ur_dashboard_msgs SafetyMode constant
            (NORMAL, PROTECTIVE_STOP, ROBOT_EMERGENCY_STOP, ...), or None if unknown.
        robot_mode: The UR robot mode, a ur_dashboard_msgs RobotMode constant
            (POWER_OFF, IDLE, RUNNING, ...), or None if unknown.
            Only RUNNING means the brakes are released and the arm can move.
        program_playing: Whether the program on the teach pendant is playing.
            The arm only follows ROS commands while ros_control.urp plays.
        program_name: The loaded program's file name, "" before it is known.
        operational: The sync's verdict: dashboard connected, safety NORMAL and
            robot RUNNING. When any arm is not, the sync stops every arm.
        problem: What the sync says is wrong with this arm, or "".
        stop_reason: Why the sync last stopped every arm, or "".
        end_effector: State of the mounted tool, or None for a bare arm.
        last_motion_time: ROS time the joints last moved or a trajectory was
            sent, seconds, or None before either.
        last_update_time: ROS time of the last joint_states, seconds, or None before any.
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
            base_in_husky: Where this arm's base_link sits in the husky frame
                (frames.HUSKY_FRAME), from the URDF's fixed mount chain.
            urdf_problem: Why the URDF breaks the stock UR frames for this arm
                (ur_frames.stock_frame_problem), or None. If set, Cartesian
                targets are refused: `base_in_husky` would put them in the
                wrong frame -- the arm looks right in the viewer, moves wrong.
        """
        self._node = node
        self.config = config
        #: This arm's base_link -- the frame its controller takes targets in -- in the husky frame.
        self.base_in_husky = base_in_husky
        self.urdf_problem = urdf_problem
        self.state = ArmState()
        #: When each kind of test-mode banner was last logged, ROS time. See _log_test_mode.
        self._test_mode_logged: dict[str, float] = {}
        #: Joint positions, by URDF name, where motion was last detected. See _on_joint_state.
        self._motion_reference: dict[str, float] = {}
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
        # * The driver's topics come through a rate limiter on the robot, so the
        #   monitor's wifi link is not flooded at the driver's 500 Hz.
        # ? Subscribed best effort: only the newest sample matters, so lost ones
        #   are not resent over wifi. The rate limiter publishes reliable, for
        #   older subscribers; a best-effort subscription receives from it too.
        measured = f"{namespace}/rate_limiter"
        sensor = qos_profile_sensor_data

        # --- --- measurements --- ---
        self._ros.subscription(JointState, f"{measured}/joint_states", self._on_joint_state, sensor)
        self._ros.subscription(DynamicJointState, f"{measured}/dynamic_joint_states",
                               self._on_dynamic_joint_state, sensor)
        self._ros.subscription(IOStates, f"{measured}/io_and_status_controller/io_states",
                               self._on_io_states, sensor)
        self._ros.subscription(WrenchStamped, f"{measured}/ft_sensor_wrench", self._on_wrench, sensor)

        # --- --- commands --- ---
        self._trajectory = self._ros.publisher(
            JointTrajectory, f"{namespace}/{SCALED_JOINT_TRAJECTORY_CONTROLLER}/joint_trajectory")
        self._target_frame = self._ros.publisher(PoseStamped, f"{namespace}/target_frame")
        self._target_wrench = self._ros.publisher(WrenchStamped, f"{namespace}/target_wrench")
        self._zero_ft = self._ros.client(Trigger, f"{namespace}/io_and_status_controller/zero_ftsensor")
        # Safety mode, robot mode and program come from multi_arm_safety_sync,
        # through the robot's one subscription (HuskyRobotInterface._on_sync_status).

    def reconnect(self) -> None:
        """Destroy this arm's and its tool's ROS entities and create them again. Keeps `state`."""
        self._ros.destroy_all()
        self._connect()
        self.controllers.reconnect()
        if self.end_effector is not None:
            self.end_effector.reconnect()

    # --- --- --- --- --- COMMANDS --- --- --- --- ---
    # ! ROS thread only: from a plugin's update, a job step, or an intent
    #   drained at the start of that plugin's step. Never from a viser callback.

    def build_trajectory(self, positions: Sequence[Sequence[float]], duration: float,
                         velocities: Sequence[Sequence[float]] | None = None) -> JointTrajectory | None:
        """Turn waypoints into a trajectory message, after checking it is safe.

        Used by `send_joint_trajectory`, and by the robot for a multi-arm
        trajectory, which is why it is separate from sending.

        Args:
            positions: At least two waypoints, each six joint angles in
                UR_JOINT_NAMES order, radians. The first must match where the arm
                is now, because it runs at time 0.
            duration: Total time, seconds. Waypoints are spread evenly across it.
            velocities: Joint velocities per waypoint, same shape, or None.

        Returns:
            JointTrajectory | None: The message, or None (with the reason logged)
                if any check in `_check_trajectory` fails.
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
        """Every check a trajectory must pass before it is sent.

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
        # ! Must end at rest: the controller rejects a moving end anyway, and a
        #   trajectory that ended moving would leave the arm coasting.
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
        """Checks every motion command shares, whichever controller it is for.

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
            # ? Not refused: with fake hardware there is no dashboard, and the
            #   robot enforces its own safety either way. But it is said.
            self._node.get_logger().warning(
                f"arm {self.config.name}: no dashboard state from multi_arm_safety_sync, safety not checked",
                throttle_duration_sec=5.0)
            return None
        if not self.state.operational:
            return f"the arm is not operational: {self.state.problem}"
        return None

    def hold(self) -> bool:
        """Stop the arm where it is now, under whichever controller is running.

        ? How, per controller.
          - Joint trajectory: an empty trajectory would be simplest, but the
            controller rejects it ("Empty trajectory received.") and carries
            on. A new trajectory replaces the running one instead, and the
            controller blends from its current state into the new first point.
            So: one point, at the measured joints, at rest, HOLD_DURATION ahead.
          - Cartesian compliance: the target becomes the measured TCP. The
            controller follows its target and nothing else, so once the target
            stops moving -- the caller also stops sending new ones -- so does
            the arm. A target wrench stays applied; zero it separately.

        ! It holds where the arm was *measured*, which is a wifi trip behind
          where it is. At 20 deg/s that is about a degree of back-motion.

        ! Refused when the measurement is stale: holding at an old position
          could mean a jump. Use the teach pendant then.

        Returns:
            bool: True if the hold was sent. False (with the reason logged) if
                neither controller is running -- so nothing of ours is moving --
                or the measurement is missing or stale.
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
        """Move smoothly from where the arm is now to `target`. See `joint_move`.

        Args:
            target: Joint angles to end at, radians, UR_JOINT_NAMES order.
            max_speed: Peak speed allowed on any joint, rad/s.
            min_duration: Shortest allowed duration, seconds.

        Returns:
            float | None: The move's duration in seconds, or None if nothing was
                sent (no joint state yet, or `send_joint_trajectory` refused; the
                log says why).
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
            hold: True only from `hold`, whose target is the reported TCP and
                never passes through the URDF, so a URDF problem does not stop it.

        ! The compliance controller pulls the TCP straight towards whatever
          target it is given, with no interpolation of its own. So a target far
          from the TCP is a jump; bigger moves are sent as many close targets,
          one per tick (see `cartesian_move`).

        Returns:
            bool: True if sent. False (with the reason logged) if the compliance
                controller is not running, the input is not finite, there is no
                recent TCP measurement, or the target is too far from the TCP.
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
            # The first target of a move: say exactly how it compares with the
            # TCP the driver reports, in the driver's own frame.
            log.info(self.target_vs_reported(position, orientation))
        if self.config.cartesian_test_mode:
            # Recorded anyway, so the 3D view shows where it would have gone.
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
        """A pose in this arm's base_link (the controller's frame), in the husky frame.

        Args:
            position: Metres, in the arm's base_link.
            orientation: Quaternion (x, y, z, w).

        Returns:
            tuple[np.ndarray, np.ndarray]: Position and quaternion in the husky frame.
        """
        p, r = compose(self.base_in_husky, (np.asarray(position, dtype=float), Rotation.from_quat(orientation)))
        return p, r.as_quat()

    def from_husky(self, position: Sequence[float], orientation: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
        """A pose in the husky frame, in this arm's base_link -- ready to send. Inverse of `to_husky`."""
        p, r = compose(invert(self.base_in_husky), (np.asarray(position, dtype=float), Rotation.from_quat(orientation)))
        return p, r.as_quat()

    def target_vs_reported(self, position: np.ndarray, orientation: np.ndarray) -> str:
        """One line: a target, turned back into the UR's Base frame, against the raw reported TCP.

        The first target of every move starts at the TCP, so here the two must
        match to the digit. Any difference is a frame or conversion error.

        Args:
            position: The target, metres, in the compliance controller's base_link.
            orientation: Its quaternion (x, y, z, w).

        Returns:
            str: The comparison, for the log.
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
        """Refuse a target that jumps away from the one sent just before it.

        Args:
            position: The new target's position, metres.
            orientation: The new target's quaternion.

        Returns:
            str | None: Why it is refused, or None.
        """
        state = self.state
        if state.sent_target_time is None or self._now() - state.sent_target_time > CARTESIAN_STEP_WINDOW:
            return None  # nothing sent just now: the TCP check above is the one that counts
        step = float(np.linalg.norm(position - state.sent_target_position))
        turn = float((Rotation.from_quat(state.sent_target_orientation).inv() * Rotation.from_quat(orientation))
                     .magnitude())
        if step > CARTESIAN_MAX_STEP or turn > CARTESIAN_MAX_STEP_ANGLE:
            return (f"jumps {step * 1e3:.1f} mm / {np.degrees(turn):.1f} deg from the target sent "
                    f"just before; at most {CARTESIAN_MAX_STEP * 1e3:.0f} mm / "
                    f"{np.degrees(CARTESIAN_MAX_STEP_ANGLE):.0f} deg per step")
        return None

    def _log_test_mode(self, what: str, detail: str) -> None:
        """Say, in a banner at most once a second, that a Cartesian command passed but was not sent.

        Args:
            what: Which command, e.g. "target frame".
            detail: Its values, for the log.
        """
        # ! Throttled per command, not with rclpy's throttle: that one is per
        #   line of code, so a target-frame banner would silence the force one.
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

        ! In the TOOL frame (TARGET_WRENCH_FRAME, tool0), not the arm's base.
          That is how the controller is configured (see TARGET_WRENCH_FRAME),
          and it turns with the tool: a force along tool x stays along tool x
          as the tool rotates.

        Args:
            force: Force in tool0, newtons.
            torque: Torque in tool0, newton-metres.

        ! It stays applied until another wrench is sent, whatever else happens
          -- including a hold. Send zeros to remove it.

        Returns:
            bool: True if sent. False (with the reason logged) if the compliance
                controller is not running, or the wrench is not finite or over
                MAX_TARGET_FORCE / MAX_TARGET_TORQUE.
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

        ! Only correct when the arm holds nothing but its own tool. Zeroing
          while a bar is gripped tares away the bar's weight, which is exactly
          the load the compliant execution needs to measure.

        Returns:
            bool: True if the request went out. False if the service is not up.
        """
        if not self._zero_ft.service_is_ready():
            self._node.get_logger().error(f"arm {self.config.name}: zero_ftsensor is not available")
            return False
        self._zero_ft.call_async(Trigger.Request())
        return True

    def mark_executing(self) -> None:
        """Record that a trajectory was just sent, so `is_executing` is True now.

        ? Without this, `is_executing` would stay False until the first joint
          motion arrives, and a waiter checking it right after sending would
          think the trajectory had already finished.
        """
        self.state.is_executing = True
        self.state.last_motion_time = self._now()

    # --- --- --- --- --- QUERIES --- --- --- --- ---

    def joint_vector(self) -> np.ndarray | None:
        """Current joint angles in UR_JOINT_NAMES order, or None before all six are known."""
        prefix = f"{self.config.name}_"
        values = [self.state.joint_positions.get(prefix + name) for name in UR_JOINT_NAMES]
        return None if any(value is None for value in values) else np.array(values)

    # --- --- --- --- --- CALLBACKS (executor thread) --- --- --- --- ---
    # ! Each only writes into self.state. No planning, drawing or file IO here.

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
        # ? Compared with where the joints were when motion was last seen, not
        #   with the previous message. A slow move -- 20 deg/s is 0.007 rad per
        #   message at 50 Hz -- never crosses the threshold between two
        #   messages, and would count as finished while still moving.
        if moved:
            self._motion_reference = {key: self.state.joint_positions[key] for key in self._motion_reference}
            self.state.last_motion_time = now
        elif self.state.last_motion_time is None or now - self.state.last_motion_time > STILL_AFTER:
            self.state.is_executing = False
        self.state.last_update_time = now

    def _on_dynamic_joint_state(self, message: DynamicJointState) -> None:
        """Store the TCP pose the UR driver publishes as the "tcp_pose" interface.

        ? The driver reports it in the UR controller's Base frame. It is kept
          as reported (`tcp_raw_*`) and turned into the compliance controller's
          `base_link` (BASE_LINK_FROM_UR_BASE), the frame commands are sent in.
        """
        if "tcp_pose" not in message.joint_names:
            return
        interface = message.interface_values[message.joint_names.index("tcp_pose")]
        values = dict(zip(interface.interface_names, interface.values))
        position = np.array([values["position.x"], values["position.y"], values["position.z"]])
        quaternion = np.array([values["orientation.x"], values["orientation.y"],
                               values["orientation.z"], values["orientation.w"]])
        # ! The driver publishes an all-zero quaternion until it knows the TCP
        #   (at startup, and with fake hardware). That is "no pose", not a pose:
        #   skip it and keep the last good one.
        if not np.linalg.norm(quaternion) > 1e-6:
            return
        self.state.tcp_raw_position, self.state.tcp_raw_orientation = position, quaternion / np.linalg.norm(quaternion)
        self.state.tcp_position = BASE_LINK_FROM_UR_BASE.apply(position)
        self.state.tcp_orientation = (BASE_LINK_FROM_UR_BASE * Rotation.from_quat(quaternion)).as_quat()
        self.state.tcp_update_time = self._now()

    def _on_io_states(self, message: IOStates) -> None:
        """Store the digital inputs by pin."""
        # A fresh list per message, sized by what the driver sends.
        digital_in = [False] * len(message.digital_in_states)
        for pin in message.digital_in_states:
            if 0 <= pin.pin < len(digital_in):
                digital_in[pin.pin] = bool(pin.state)
        self.state.digital_in = digital_in

    def on_sync_status(self, status: ArmStatus, stop_reason: str) -> None:
        """Store this arm's part of a multi_arm_safety_sync status.

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
        """ROS time in seconds, so timing behaves under bag playback."""
        return self._node.get_clock().now().nanoseconds * 1e-9
