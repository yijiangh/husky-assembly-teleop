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
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_msgs.msg import IOStates

from ..config import ArmConfig
from .controller_manager import ControllerManagerInterface, ControllerManagerState
from .end_effectors import EndEffector, EndEffectorState, make_end_effector

#: Joint names as the UR driver publishes and expects them, in its order. The
#: URDF has the same joints with the arm's prefix in front.
UR_JOINT_NAMES = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")

SCALED_JOINT_TRAJECTORY_CONTROLLER = "scaled_joint_trajectory_controller"
CARTESIAN_COMPLIANCE_CONTROLLER = "cartesian_compliance_controller"
#: The controllers an arm switches between, only one at a time.
ARM_CONTROLLERS = (SCALED_JOINT_TRAJECTORY_CONTROLLER, CARTESIAN_COMPLIANCE_CONTROLLER)

#: How far, in radians per joint, the arm may be from a trajectory's first
#: waypoint. The first waypoint runs at time 0, so a bigger gap is a jump.
START_TOLERANCE = 0.1
#: How far, in metres, the TCP may be from a cartesian target. Same reason.
CARTESIAN_START_TOLERANCE = 0.05
#: A joint change bigger than this between two joint_states counts as motion.
MOTION_THRESHOLD = 1e-2
#: Seconds without motion after which the arm counts as done executing.
STILL_AFTER = 1.0


@dataclass
class ArmState:
    """Measured state of one arm.

    Attributes:
        joint_positions: Joint name to position, radians. Keyed by URDF name
            (e.g. "left_ur_arm_elbow_joint"), so RobotScene and the viewer can
            use it directly. Absent until the first message.
        tcp_position: TCP position from the UR driver, metres, or None.
        tcp_orientation: TCP orientation, quaternion (x, y, z, w), or None.
        wrench: Force/torque sensor reading (fx, fy, fz, tx, ty, tz), or None.
        digital_in: UR digital inputs, by pin.
        is_executing: Whether the arm is moving under a trajectory we sent.
            ? Guessed from motion, because trajectories go over a topic (more
              reliable than the action on busy wifi, and not every controller
              has an action) and a topic has no "done". True from the moment
              one is sent until the joints have been still for STILL_AFTER.
        controllers: The arm controller manager's state.
        end_effector: State of the mounted tool, or None for a bare arm.
        last_motion_time: ROS time the joints last moved, seconds.
        last_update_time: ROS time of the last joint_states, seconds.
    """

    joint_positions: dict[str, float] = field(default_factory=dict)
    tcp_position: np.ndarray | None = None
    tcp_orientation: np.ndarray | None = None
    wrench: np.ndarray | None = None
    digital_in: list[bool] = field(default_factory=lambda: [False] * 18)
    is_executing: bool = False
    controllers: ControllerManagerState = field(default_factory=ControllerManagerState)
    end_effector: EndEffectorState | None = None
    last_motion_time: float = 0.0
    last_update_time: float = 0.0


class ArmInterface:
    """ROS2 interface to one UR arm and the tool mounted on it.

    Attributes:
        config: This arm's configuration.
        state: Measured state, written only by callbacks.
        controllers: Tracks and switches this arm's controllers.
        end_effector: The mounted tool, or None for a bare arm.
    """

    def __init__(self, node: Node, robot_namespace: str, config: ArmConfig):
        """Create every subscription, publisher and client of one arm.

        Args:
            node: The monitor node.
            robot_namespace: The robot's namespace, e.g. "a200_0806".
            config: This arm.
        """
        self._node = node
        self.config = config
        self.state = ArmState()
        namespace = f"/{robot_namespace}/{config.ros_namespace}"
        # * The driver's topics come through a rate limiter on the robot, so the
        #   monitor's wifi link is not flooded at the driver's 500 Hz.
        measured = f"{namespace}/rate_limiter"
        self._tcp_correction = Rotation.from_euler("z", config.tcp_yaw_correction)

        self.controllers = ControllerManagerInterface(
            node, namespace, ARM_CONTROLLERS, self.state.controllers)
        self.end_effector: EndEffector | None = make_end_effector(node, robot_namespace, config)
        self.state.end_effector = self.end_effector.state if self.end_effector else None

        # --- --- measurements --- ---
        node.create_subscription(JointState, f"{measured}/joint_states", self._on_joint_state, 10)
        node.create_subscription(DynamicJointState, f"{measured}/dynamic_joint_states",
                                 self._on_dynamic_joint_state, 10)
        node.create_subscription(IOStates, f"{measured}/io_and_status_controller/io_states",
                                 self._on_io_states, 10)
        node.create_subscription(WrenchStamped, f"{measured}/ft_sensor_wrench", self._on_wrench, 10)

        # --- --- commands --- ---
        self._trajectory = node.create_publisher(
            JointTrajectory, f"{namespace}/{SCALED_JOINT_TRAJECTORY_CONTROLLER}/joint_trajectory", 10)
        self._target_frame = node.create_publisher(PoseStamped, f"{namespace}/target_frame", 10)
        self._target_wrench = node.create_publisher(WrenchStamped, f"{namespace}/target_wrench", 10)
        self._zero_ft = node.create_client(Trigger, f"{namespace}/io_and_status_controller/zero_ftsensor")

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
                if the joint trajectory controller is not running, the input is
                malformed, or the arm is not at the first waypoint.
        """
        log = self._node.get_logger()
        if not self.controllers.is_active(SCALED_JOINT_TRAJECTORY_CONTROLLER):
            log.error(f"arm {self.config.name}: not sending a trajectory, "
                      f"{SCALED_JOINT_TRAJECTORY_CONTROLLER} is not active "
                      f"(active: {self.state.controllers.active!r})")
            return None
        if len(positions) < 2:
            log.error(f"arm {self.config.name}: a trajectory needs at least two waypoints")
            return None
        if velocities is not None and len(velocities) != len(positions):
            log.error(f"arm {self.config.name}: {len(positions)} waypoints but "
                      f"{len(velocities)} velocities")
            return None
        current = self.joint_vector()
        if current is None:
            log.error(f"arm {self.config.name}: not sending a trajectory, no joint state yet")
            return None
        if not np.allclose(current, positions[0], atol=START_TOLERANCE):
            log.error(f"arm {self.config.name}: not at the trajectory's start: "
                      f"{np.round(current, 3)} vs {np.round(positions[0], 3)}")
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

    def send_cartesian_target(self, position: Sequence[float], orientation: Sequence[float]) -> bool:
        """Send a TCP target to the compliance controller.

        Args:
            position: Target position in the arm's base_link frame, metres.
            orientation: Target orientation, quaternion (x, y, z, w).

        Returns:
            bool: True if sent. False if the compliance controller is not running,
                there is no TCP measurement yet, or the target is too far from it.
        """
        log = self._node.get_logger()
        if not self.controllers.is_active(CARTESIAN_COMPLIANCE_CONTROLLER):
            log.error(f"arm {self.config.name}: not sending a cartesian target, "
                      f"{CARTESIAN_COMPLIANCE_CONTROLLER} is not active")
            return False
        if self.state.tcp_position is None:
            log.error(f"arm {self.config.name}: not sending a cartesian target, no TCP pose yet")
            return False
        if not np.allclose(self.state.tcp_position, position, atol=CARTESIAN_START_TOLERANCE):
            log.error(f"arm {self.config.name}: cartesian target too far from the TCP: "
                      f"{np.round(self.state.tcp_position, 3)} vs {np.round(position, 3)}")
            return False
        message = PoseStamped()
        message.header.frame_id = "base_link"
        message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, position)
        (message.pose.orientation.x, message.pose.orientation.y,
         message.pose.orientation.z, message.pose.orientation.w) = map(float, orientation)
        self._target_frame.publish(message)
        return True

    def send_target_wrench(self, force: Sequence[float],
                           torque: Sequence[float] = (0.0, 0.0, 0.0)) -> bool:
        """Send a target wrench to the compliance controller.

        Args:
            force: Force in the arm's base_link frame, newtons.
            torque: Torque in the same frame, newton-metres.

        Returns:
            bool: True if sent. False if the compliance controller is not running.
        """
        if not self.controllers.is_active(CARTESIAN_COMPLIANCE_CONTROLLER):
            self._node.get_logger().error(f"arm {self.config.name}: not sending a wrench, "
                                          f"{CARTESIAN_COMPLIANCE_CONTROLLER} is not active")
            return False
        message = WrenchStamped()
        message.header.frame_id = "base_link"
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
            previous = self.state.joint_positions.get(key)
            if previous is not None and abs(position - previous) > MOTION_THRESHOLD:
                moved = True
            self.state.joint_positions[key] = float(position)
        # ? Compared with the previous message, as the old code did. At the rate
        #   limiter's rate that catches any trajectory worth waiting for.
        if moved:
            self.state.last_motion_time = now
        elif now - self.state.last_motion_time > STILL_AFTER:
            self.state.is_executing = False
        self.state.last_update_time = now

    def _on_dynamic_joint_state(self, message: DynamicJointState) -> None:
        """Store the TCP pose the UR driver publishes as the "tcp_pose" interface.

        ? The driver reports the TCP in a frame that includes the arm's mounting
          rotation, so `tcp_yaw_correction` rotates it back into the arm's
          base_link frame.
        """
        if "tcp_pose" not in message.joint_names:
            return
        interface = message.interface_values[message.joint_names.index("tcp_pose")]
        values = dict(zip(interface.interface_names, interface.values))
        position = np.array([values["position.x"], values["position.y"], values["position.z"]])
        orientation = Rotation.from_quat([values["orientation.x"], values["orientation.y"],
                                          values["orientation.z"], values["orientation.w"]])
        self.state.tcp_position = self._tcp_correction.apply(position)
        self.state.tcp_orientation = (self._tcp_correction * orientation).as_quat()

    def _on_io_states(self, message: IOStates) -> None:
        """Store the digital inputs by pin."""
        for pin in message.digital_in_states:
            if 0 <= pin.pin < len(self.state.digital_in):
                self.state.digital_in[pin.pin] = bool(pin.state)

    def _on_wrench(self, message: WrenchStamped) -> None:
        """Store the force/torque reading."""
        f, t = message.wrench.force, message.wrench.torque
        self.state.wrench = np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def _now(self) -> float:
        """ROS time in seconds, so timing behaves under bag playback."""
        return self._node.get_clock().now().nanoseconds * 1e-9
