"""
One physical robot: its base, its arms, and the state they measure together.

HuskyRobotInterface only composes. Every topic belongs to the part it concerns
-- BaseInterface, ArmInterface, the end effectors -- and the configuration
decides which parts exist. The exceptions concern all arms at once and so live
here: the multi-arm trajectory, and multi_arm_safety_sync -- its status (one
message for every arm, handed to each), unlocking a protective stop, and the
soft stop of the whole robot and resuming its arms.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import numpy as np
from crl_husky_msgs.msg import MultiArmTrajectory, SafetySyncStatus
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from ..config import RobotConfig
from .arm import ArmInterface, ArmState
from .base import BaseInterface, BaseState
from .frames import HUSKY_FRAME, MOCAP_FRAME, Pose, compose, fixed_transform, invert
from .ur_frames import stock_frame_problem
from .connections import RosConnections

#: The on-board node that runs or stops all arms together, under the robot's namespace.
SAFETY_SYNC = "MultiArmSafetySync"


@dataclass
class RobotState:
    """Measured state of one robot, gathered in one place for the core.

    ! Plugins do not read through this. They go through the parts --
      `robot.base.state`, `robot.arms[name].state` -- which are the same
      objects, so there is one way to reach anything. This bundle exists for
      the scene and the viewer, which pose every robot from it each tick.

    ! Every field is per-instance, through the dataclass. The old interface
      declared these at class scope and mutated them in place, so two robots
      shared one list -- and holding N robots is the whole point of WorldState.

    ! The base and arm states here are the *same objects* their interfaces write
      into, not copies. Reading through RobotState is always current.

    Attributes:
        serial: Which robot this describes.
        base: Base pose and base controllers.
        arms: Per-arm state, keyed by ArmConfig.name.
    """

    serial: str
    base: BaseState = field(default_factory=BaseState)
    arms: dict[str, ArmState] = field(default_factory=dict)

    @property
    def joint_positions(self) -> dict[str, float]:
        """dict[str, float]: Every arm's joints, by URDF name. What the scene and viewer pose from."""
        merged: dict[str, float] = {}
        for arm in self.arms.values():
            merged.update(arm.joint_positions)
        return merged


class HuskyRobotInterface:
    """ROS2 interface to one husky: a base, its arms, and their tools.

    ! Threading. Every subscription callback here runs on the node's
      single-threaded executor, the same thread as the tick, so writes to
      `self.state` are already serialised against every reader and need no
      locking. Mocap arrives the same way, as an ordinary subscription. This
      holds only while the node keeps one executor and the default
      mutually-exclusive callback group; if that changes, this note was wrong.

    * How a plugin reaches a robot, always the same way:

          robot.config                  fixed for the run: serial, arms, tools
          robot.base.state              measured base, read only
          robot.arms[name].state        measured arm, read only
          robot.base.<command>(...)     commands, ROS thread only
          robot.arms[name].<command>(...)

    Attributes:
        config: Identity, URDF, arms and mocap id of this robot.
        base: The mobile base.
        arms: The arms, keyed by ArmConfig.name, in configuration order.
        state: Everything measured, gathered from the parts, for the core's
            scene and viewer. Plugins read the parts instead.
    """

    def __init__(self, node: Node, config: RobotConfig):
        """Build the base and one ArmInterface per configured arm.

        Nothing here waits for the robot: clients are created, and whether a
        service is up is checked when it is called.

        Args:
            node: The monitor node, used to create subscriptions, publishers and
                clients. This class does not own a node of its own.
            config: The robot to connect to.
        """
        self._node = node
        self.config = config
        self.base = BaseInterface(node, config)
        # ! Every arm's URDF frames are checked first. Silent when stock; if not,
        #   the error is logged and the arm refuses Cartesian targets
        #   (doc/ur_frames.md): its URDF would look right and move wrong.
        self.arms = {}
        for arm in config.arms:
            problem = stock_frame_problem(config.urdf_file, arm.name)
            if problem is not None:
                node.get_logger().error(problem)
            self.arms[arm.name] = ArmInterface(
                node, config.ros_namespace, arm,
                fixed_transform(config.urdf_file, HUSKY_FRAME, f"{arm.name}_base_link"), urdf_problem=problem)
        #: The husky frame in the frame mocap tracks (URDF, fixed).
        self._husky_in_mocap_frame = fixed_transform(config.urdf_file, MOCAP_FRAME, HUSKY_FRAME)
        self.state = RobotState(serial=config.serial, base=self.base.state,
                                arms={name: arm.state for name, arm in self.arms.items()})
        self._ros = RosConnections(node)
        self._connect()

    def _connect(self) -> None:
        """Create the robot-level publisher, subscription and clients. The parts create their own."""
        sync = f"/{self.config.ros_namespace}/{SAFETY_SYNC}"
        # ! Transient local, like the sync's publisher, so the last status arrives
        #   at once instead of after the next change.
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._ros.subscription(SafetySyncStatus, f"{sync}/status", self._on_sync_status, status_qos)
        self._unlock = self._ros.client(Trigger, f"{sync}/unlock_protective_stop")
        self._stop_arms = self._ros.client(Trigger, f"{sync}/stop")
        self._resume_arms = self._ros.client(Trigger, f"{sync}/resume")

        # * Only a robot with two arms has the on-board node that splits this
        #   message into two synchronised trajectories.
        self._multi_arm = None
        if len(self.arms) == 2:
            self._multi_arm = self._ros.publisher(
                MultiArmTrajectory, f"/{self.config.ros_namespace}/multi_arm_joint_trajectory")

    def reconnect(self) -> None:
        """Tear down every topic, service and action of this robot and create them again.

        For when a robot stopped answering, e.g. after a driver restart or a
        discovery hiccup on the wifi. The objects and their state stay the same,
        so anything holding `robot.base`, `robot.arms[name]` or their state keeps
        working. Old measurements stay until fresh ones arrive, so their age
        still tells the truth.

        ! ROS thread only, like every command.
        """
        self._node.get_logger().info(f"robot {self.config.serial}: reconnecting")
        self._ros.destroy_all()
        self._connect()
        self.base.reconnect()
        for arm in self.arms.values():
            arm.reconnect()

    # --- --- --- --- --- FRAMES --- --- --- --- ---

    def husky_in_world(self) -> Pose | None:
        """The husky frame in the world, from the last valid mocap fix, or None before one."""
        base = self.base.state
        if base.position is None:
            return None
        return compose((base.position, Rotation.from_quat(base.orientation)), self._husky_in_mocap_frame)

    def to_world(self, position, orientation) -> tuple[np.ndarray, np.ndarray] | None:
        """A pose in the husky frame, in the world; None before the first mocap fix.

        ! Uses the last valid mocap pose, which may be stale: check
          `state.base.tracked` before acting on the result.
        """
        husky = self.husky_in_world()
        if husky is None:
            return None
        p, r = compose(husky, (np.asarray(position, dtype=float), Rotation.from_quat(orientation)))
        return p, r.as_quat()

    def from_world(self, position, orientation) -> tuple[np.ndarray, np.ndarray] | None:
        """A pose in the world, in the husky frame; None before the first mocap fix. Inverse of `to_world`."""
        husky = self.husky_in_world()
        if husky is None:
            return None
        p, r = compose(invert(husky), (np.asarray(position, dtype=float), Rotation.from_quat(orientation)))
        return p, r.as_quat()

    # --- --- --- --- --- COMMANDS --- --- --- --- ---
    # ! ROS thread only, like the commands on the parts.

    def unlock_protective_stop(self) -> bool:
        """Clear the protective stop of every arm in one, like "Enable robot" on the pendant.

        Done by multi_arm_safety_sync, which then restarts ros_control.urp on
        every arm once all are operational.

        ! Clear the cause first. The robot refuses it for the first 5 s after
          the stop, and only accepts it in remote control mode. Whether it
          worked shows in the safety mode.

        Returns:
            bool: True if the request went out. False if the sync is not reachable.
        """
        return self._call_sync(self._unlock, "unlock protective stop")

    def soft_stop(self) -> None:
        """Stop the whole robot, and keep it stopped until an operator resumes it.

        - Arms: multi_arm_safety_sync stops ros_control.urp on every arm and
          does not restart it until `resume_arms`. The driver's
          controller_stopper then deactivates the arms' motion controllers.
        - Base: see BaseInterface.stop. Resumed by switching its controller on.

        ! Not an emergency stop. It needs the wifi, the monitor and the on-board
          nodes all working. The teach pendant and the platform e-stop remain
          the real stops.
        """
        self._node.get_logger().warning(f"robot {self.config.serial}: SOFT STOP")
        self.base.stop()
        if self.arms:
            self._call_sync(self._stop_arms, "stop arms")

    def resume_arms(self) -> bool:
        """Lift the soft stop of the arms. The sync restarts them once all are operational.

        Returns:
            bool: True if the request went out. False if the sync is not reachable.
        """
        return self._call_sync(self._resume_arms, "resume arms")

    def _call_sync(self, client, what: str) -> bool:
        """Call one of multi_arm_safety_sync's Trigger services and log its answer.

        Args:
            client: The service client.
            what: What the call does, for the log.

        Returns:
            bool: True if the request went out. False if the sync is not reachable.
        """
        log = self._node.get_logger()
        if not client.service_is_ready():
            log.error(f"robot {self.config.serial}: cannot {what}, {SAFETY_SYNC} not reachable")
            return False
        log.warning(f"robot {self.config.serial}: {what}")
        client.call_async(Trigger.Request()).add_done_callback(
            lambda future: self._on_sync_answer(what, future))
        return True

    def _on_sync_answer(self, what: str, future) -> None:
        """Log the sync's answer to a `_call_sync` request."""
        try:
            response = future.result()
        except Exception as error:
            response, message = None, str(error)
        else:
            message = response.message if response is not None else "no answer"
        # ! Two separate calls, never one line choosing info or error: rclpy
        #   ties each log call site to one severity and raises if it changes.
        log = self._node.get_logger()
        if response is not None and response.success:
            log.info(f"robot {self.config.serial}: {what}: {message}")
        else:
            log.error(f"robot {self.config.serial}: {what}: {message}")

    def send_multi_arm_trajectory(self, positions: dict[str, Sequence[Sequence[float]]],
                                  duration: float) -> bool:
        """Send one trajectory to each arm, started together on the robot.

        Each arm's trajectory is checked exactly as `ArmInterface.build_trajectory`
        checks it, and nothing is sent unless both pass.

        Args:
            positions: Waypoints per arm name, for both arms. See
                ArmInterface.build_trajectory for the waypoint format.
            duration: Total time for both, seconds.

        Returns:
            bool: True if sent. False if this robot does not have two arms, an
                arm is missing from `positions`, or either trajectory fails its checks.
        """
        if self._multi_arm is None:
            self._node.get_logger().error(f"robot {self.config.serial}: multi-arm trajectory "
                                          f"needs two arms, it has {len(self.arms)}")
            return False
        if set(positions) != set(self.arms):
            self._node.get_logger().error(f"robot {self.config.serial}: multi-arm trajectory needs "
                                          f"waypoints for {sorted(self.arms)}, got {sorted(positions)}")
            return False
        # The message's trajectory1 / trajectory2 follow the configured arm order.
        first, second = self.arms.values()
        # Built before the message: a message field refuses None.
        first_trajectory = first.build_trajectory(positions[first.config.name], duration)
        second_trajectory = second.build_trajectory(positions[second.config.name], duration)
        if first_trajectory is None or second_trajectory is None:
            return False
        message = MultiArmTrajectory()
        message.trajectory1 = first_trajectory
        message.trajectory2 = second_trajectory
        self._multi_arm.publish(message)
        first.mark_executing()
        second.mark_executing()
        return True

    # --- --- --- --- --- CALLBACKS (executor thread) --- --- --- --- ---

    def _on_sync_status(self, message: SafetySyncStatus) -> None:
        """Hand each arm its entry of the sync's status, matched by namespace."""
        by_namespace = {status.name: status for status in message.arms}
        for arm in self.arms.values():
            status = by_namespace.get(arm.config.ros_namespace)
            if status is not None:
                arm.on_sync_status(status, message.stop_reason)
