"""
One physical robot: its base, its arms, and the state they measure together.

HuskyRobotInterface only composes the parts. Only what concerns all arms at
once lives here: the multi-arm trajectory and the multi_arm_safety_sync calls.
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

#: On-board node that runs or stops all arms together.
SAFETY_SYNC = "MultiArmSafetySync"


@dataclass
class RobotState:
    """Measured state of one robot, for the scene and viewer.

    ! Plugins should read `robot.base.state` and `robot.arms[name].state`
      instead. They are the same live objects, not copies.

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
        """dict[str, float]: Every arm's joints, by URDF name."""
        merged: dict[str, float] = {}
        for arm in self.arms.values():
            merged.update(arm.joint_positions)
        return merged


class HuskyRobotInterface:
    """ROS2 interface to one husky: a base, its arms, and their tools.

    ! Callbacks run on the node's single executor thread, the same as the tick,
      so no locking is needed. This breaks if the node gets more than one
      executor thread or a non-default callback group.

    * Plugins reach a robot through `robot.config`, `robot.base.state`,
      `robot.arms[name].state` (read only) and `robot.base.<command>()` /
      `robot.arms[name].<command>()` (ROS thread only).

    Attributes:
        config: Identity, URDF, arms and mocap id of this robot.
        base: The mobile base.
        arms: The arms, keyed by ArmConfig.name, in configuration order.
        state: Everything measured, for the scene and viewer.
    """

    def __init__(self, node: Node, config: RobotConfig):
        """Build the base and one ArmInterface per configured arm.

        Does not wait for the robot; service availability is checked per call.

        Args:
            node: The monitor node that owns the subscriptions, publishers and clients.
            config: The robot to connect to.
        """
        self._node = node
        self.config = config
        self.base = BaseInterface(node, config)
        # ! A non-stock URDF frame is logged as an error and the arm refuses
        #   Cartesian targets: it would look right but move wrong (doc/ur_frames.md).
        self.arms = {}
        for arm in config.arms:
            problem = stock_frame_problem(config.urdf_file, arm.name)
            if problem is not None:
                node.get_logger().error(problem)
            self.arms[arm.name] = ArmInterface(
                node, config.ros_namespace, arm,
                fixed_transform(config.urdf_file, HUSKY_FRAME, f"{arm.name}_base_link"), urdf_problem=problem)
        #: The husky frame in the mocap-tracked frame (fixed, from the URDF).
        self._husky_in_mocap_frame = fixed_transform(config.urdf_file, MOCAP_FRAME, HUSKY_FRAME)
        self.state = RobotState(serial=config.serial, base=self.base.state,
                                arms={name: arm.state for name, arm in self.arms.items()})
        self._ros = RosConnections(node)
        self._connect()

    def _connect(self) -> None:
        """Create the robot-level publisher, subscription and clients."""
        sync = f"/{self.config.ros_namespace}/{SAFETY_SYNC}"
        # ! Transient local, matching the sync's publisher, so the last status arrives at once.
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._ros.subscription(SafetySyncStatus, f"{sync}/status", self._on_sync_status, status_qos)
        self._unlock = self._ros.client(Trigger, f"{sync}/unlock_protective_stop")
        self._stop_arms = self._ros.client(Trigger, f"{sync}/stop")
        self._resume_arms = self._ros.client(Trigger, f"{sync}/resume")

        # * Only a two-arm robot has the on-board node that splits this message.
        self._multi_arm = None
        if len(self.arms) == 2:
            self._multi_arm = self._ros.publisher(
                MultiArmTrajectory, f"/{self.config.ros_namespace}/multi_arm_joint_trajectory")

    def reconnect(self) -> None:
        """Recreate every topic, service and action of this robot, for when it stopped answering.

        Objects and state are kept, so held references stay valid; old
        measurements stay until fresh ones arrive, so their age is still true.

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
        """The husky frame in the world from the last mocap fix, or None before one."""
        base = self.base.state
        if base.position is None:
            return None
        return compose((base.position, Rotation.from_quat(base.orientation)), self._husky_in_mocap_frame)

    def to_world(self, position, orientation) -> tuple[np.ndarray, np.ndarray] | None:
        """Convert a husky-frame pose to the world; None before the first mocap fix.

        ! The mocap pose may be stale: check `state.base.tracked` before acting.
        """
        husky = self.husky_in_world()
        if husky is None:
            return None
        p, r = compose(husky, (np.asarray(position, dtype=float), Rotation.from_quat(orientation)))
        return p, r.as_quat()

    def from_world(self, position, orientation) -> tuple[np.ndarray, np.ndarray] | None:
        """Convert a world pose to the husky frame; None before the first mocap fix."""
        husky = self.husky_in_world()
        if husky is None:
            return None
        p, r = compose(invert(husky), (np.asarray(position, dtype=float), Rotation.from_quat(orientation)))
        return p, r.as_quat()

    # --- --- --- --- --- COMMANDS --- --- --- --- ---
    # ! ROS thread only, like the commands on the parts.

    def unlock_protective_stop(self) -> bool:
        """Clear the protective stop of every arm, like "Enable robot" on the pendant.

        ! Clear the cause first. The robot refuses for 5 s after the stop and
          only in remote control mode; check the safety mode to see if it worked.

        Returns:
            bool: True if the request went out. False if the sync is not reachable.
        """
        return self._call_sync(self._unlock, "unlock protective stop")

    def soft_stop(self) -> None:
        """Stop the whole robot until an operator resumes it.

        - Arms: stay stopped until `resume_arms`.
        - Base: see BaseInterface.stop; resume by switching its controller on.

        ! Not an emergency stop: it needs wifi, the monitor and the on-board
          nodes. Use the teach pendant or platform e-stop for that.
        """
        self._node.get_logger().warning(f"robot {self.config.serial}: SOFT STOP")
        self.base.stop()
        if self.arms:
            self._call_sync(self._stop_arms, "stop arms")

    def resume_arms(self) -> bool:
        """Lift the arms' soft stop; they restart once all are operational.

        Returns:
            bool: True if the request went out. False if the sync is not reachable.
        """
        return self._call_sync(self._resume_arms, "resume arms")

    def _call_sync(self, client, what: str) -> bool:
        """Call a multi_arm_safety_sync service and log its answer.

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
        # ! Keep two separate log calls: rclpy raises if one call site changes severity.
        log = self._node.get_logger()
        if response is not None and response.success:
            log.info(f"robot {self.config.serial}: {what}: {message}")
        else:
            log.error(f"robot {self.config.serial}: {what}: {message}")

    def send_multi_arm_trajectory(self, positions: dict[str, Sequence[Sequence[float]]],
                                  duration: float) -> bool:
        """Send one trajectory to each arm, started together; nothing is sent unless both pass `ArmInterface.build_trajectory`'s checks.

        Args:
            positions: Waypoints per arm name, for both arms (format as in
                ArmInterface.build_trajectory).
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
        # trajectory1 / trajectory2 follow the configured arm order.
        first, second = self.arms.values()
        # Built first: a message field refuses None.
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
