"""
One physical robot: its base, its arms, and the state they measure together.

Only what concerns all arms at once lives here: the multi-arm trajectory and the multi_arm_safety_sync calls.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from functools import partial
from typing import Callable, Sequence

import numpy as np
from crl_husky_msgs.msg import MultiArmTrajectory, SafetySyncStatus, SimulatedPart
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from ..config import RobotConfig
from .arm import ArmInterface, ArmState
from .base import PLATFORM_VELOCITY_CONTROLLER, BaseInterface, BaseState
from .frames import HUSKY_FRAME, MOCAP_FRAME, fixed_transform
from scaffolding_assembly_core.geometry import Pose, compose, invert
from .ur_frames import stock_frame_problem
from .connections import RosConnections
from .simulated_parts import TOPIC as SIMULATED_TOPIC
from .simulated_parts import SimulatedParts
from .robot_lease import LeaseSlot
from .soft_stop import SoftStopLatch

#: On-board node that runs or stops all arms together.
SAFETY_SYNC = "MultiArmSafetySync"


@dataclass
class RobotState:
    """Measured state of one robot, for the scene and viewer.

    ! Plugins should read `robot.base.state` and `robot.arms[name].state` instead (the same live objects).

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


@dataclass(frozen=True)
class Lease:
    """Who holds a robot for an automatic motion; other plugins refuse to command it meanwhile.

    - ! Set and cleared only through `shared/execution/interlock.py`; robot_interface does not enforce it.
    - Hold is always allowed, and calls `stop`, which cancels the holder's work.

    Attributes:
        holder: The plugin holding it, e.g. "movement_exec".
        label: What it runs, e.g. "B20_R_home".
        since: ROS time it was taken, seconds.
        stop: Cancels the holder's work. Main thread.
    """

    holder: str
    label: str
    since: float
    stop: Callable[[], None]


class HuskyRobotInterface:
    """ROS2 interface to one husky: a base, its arms, and their tools.

    ! Callbacks run on the main thread at the start of each tick, never alongside plugin code;
      don't add a multi-threaded executor.
    ! Plugins read `robot.base.state` / `robot.arms[name].state` and call commands on the main thread only.

    Attributes:
        config: Identity, URDF, arms and mocap id of this robot.
        base: The mobile base.
        arms: The arms, keyed by ArmConfig.name, in configuration order.
        state: Everything measured, for the scene and viewer.
        soft_stop_latch: Set from `soft_stop` until `resume`; while set, every part refuses commands that could move.
        lease_slot: Who holds the robot's lease (`robot_lease.py`); every part refuses motion and controller switches
            sent without it.
        lease: Who holds the robot for an automatic motion, or None (`Lease`). ! Not `lease_slot`: this one is the
            one-motion-at-a-time lock across every robot (`shared/execution/interlock.py`); movement_exec also takes
            the robot's `lease_slot` for the same time.
    """

    lease: Lease | None = None

    def __init__(self, node: Node, config: RobotConfig):
        """Build the base and one ArmInterface per configured arm, without waiting for the robot.

        Args:
            node: The monitor node.
            config: The robot to connect to.
        """
        self._node = node
        self.config = config
        self.soft_stop_latch = SoftStopLatch(config.serial)
        self.lease_slot = LeaseSlot(config.serial)
        #: The parts of this robot announced as simulated (`simulated_parts.py`).
        self.simulated = SimulatedParts(lambda: node.get_clock().now().nanoseconds * 1e-9)
        self.base = BaseInterface(node, config, simulated=partial(self.simulated.get, "base"),
                                  latch=self.soft_stop_latch, lease_slot=self.lease_slot,
                                  mocap_simulated=partial(self.simulated.get, "mocap"))
        # * The model already has each arm's UR calibration applied; an out-of-date URDF file is loud, not fatal.
        for check in config.ur_calibration:
            if check.out_of_date:
                node.get_logger().error(f"!!! robot {config.serial} {check.describe()}")
            elif check.file is None:
                node.get_logger().warning(f"robot {config.serial} {check.describe()}")
            else:
                node.get_logger().info(f"robot {config.serial} {check.describe()}")
        # ! A non-stock URDF frame makes the arm refuse Cartesian targets: it would look right but move wrong.
        self.arms = {}
        for arm in config.arms:
            problem = stock_frame_problem(config.model.urdf, arm.name)
            if problem is not None:
                node.get_logger().error(problem)
            self.arms[arm.name] = ArmInterface(
                node, config.ros_namespace, arm,
                fixed_transform(config.model.urdf, HUSKY_FRAME, f"{arm.name}_base_link"), urdf_problem=problem,
                urdf=config.model.urdf, simulated=partial(self.simulated.get, arm.ros_namespace),
                latch=self.soft_stop_latch, lease_slot=self.lease_slot)
        #: The husky frame in the mocap-tracked frame (fixed, from the URDF).
        self._husky_in_mocap_frame = fixed_transform(config.model.urdf, MOCAP_FRAME, HUSKY_FRAME)
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
        self._ros.subscription(SimulatedPart, f"/{self.config.ros_namespace}/{SIMULATED_TOPIC}", self.simulated.heard)
        self._unlock = self._ros.client(Trigger, f"{sync}/unlock_protective_stop")
        self._stop_arms = self._ros.client(Trigger, f"{sync}/stop")
        self._resume_arms = self._ros.client(Trigger, f"{sync}/resume")

        # * Only a two-arm robot has the on-board node that splits this message.
        self._multi_arm = None
        if len(self.arms) == 2:
            self._multi_arm = self._ros.publisher(
                MultiArmTrajectory, f"/{self.config.ros_namespace}/multi_arm_joint_trajectory")

    def reconnect(self) -> None:
        """Recreate every topic, service and action of this robot, keeping objects and state.

        ! Main thread only, like every command.
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
        return compose(Pose.from_arrays(base.position, base.orientation), self._husky_in_mocap_frame)

    def to_world(self, position, orientation) -> tuple[np.ndarray, np.ndarray] | None:
        """Convert a husky-frame pose to the world; None before the first mocap fix.

        ! The mocap pose may be stale: check `state.base.tracked` before acting.
        """
        husky = self.husky_in_world()
        if husky is None:
            return None
        pose = compose(husky, Pose.from_arrays(position, orientation))
        return np.array(pose.position), np.array(pose.orientation)

    def from_world(self, position, orientation) -> tuple[np.ndarray, np.ndarray] | None:
        """Convert a world pose to the husky frame; None before the first mocap fix."""
        husky = self.husky_in_world()
        if husky is None:
            return None
        pose = compose(invert(husky), Pose.from_arrays(position, orientation))
        return np.array(pose.position), np.array(pose.orientation)

    # --- --- --- --- --- COMMANDS --- --- --- --- ---
    # ! main thread only, like the commands on the parts.

    def unlock_protective_stop(self) -> bool:
        """Clear the protective stop of every arm, like "Enable robot" on the pendant.

        ! Clear the cause first. The robot refuses for 5 s after the stop and outside remote control mode;
          check the safety mode to see if it worked.

        Returns:
            bool: True if the request went out. False if the sync is not reachable.
        """
        return self._call_sync(self._unlock, "unlock protective stop")

    @property
    def soft_stopped(self) -> bool:
        """bool: Whether the robot is soft stopped: its motion commands raise RobotStopped until `resume`."""
        return self.soft_stop_latch.stopped

    def soft_stop(self, reason: str = "") -> None:
        """Stop the whole robot, and refuse every command that could move it, until an operator resumes it.

        `reason` is kept in the latch for the panels, e.g. "Stop all" or what a clearance came too close to.

        - Arms: stay stopped until `resume`.
        - Base: see BaseInterface.stop; `resume` switches its velocity controller on again.
        - Tools: every motor stops (`EndEffector.stop`); nothing restarts them.
        - Commands that could move any part raise RobotStopped until `resume` (`soft_stop.py`).

        ! Not an emergency stop: it needs wifi and the on-board nodes. Use the pendant or platform e-stop.
        ! Each part is stopped even if another raises; the error is logged.
        """
        self.soft_stop_latch.stopped, self.soft_stop_latch.reason = True, reason
        log = self._node.get_logger()
        log.warning(f"robot {self.config.serial}: SOFT STOP" + (f": {reason}" if reason else ""))
        try:
            self.base.stop()
        except Exception as error:
            log.error(f"robot {self.config.serial}: SOFT STOP of the base failed: {error!r}")
        if self.arms:
            try:
                self._call_sync(self._stop_arms, "stop arms")
            except Exception as error:
                log.error(f"robot {self.config.serial}: SOFT STOP of the arms failed: {error!r}")
        for arm in self.arms.values():
            if arm.end_effector is not None:
                try:
                    arm.end_effector.stop()
                except Exception as error:
                    log.error(f"robot {self.config.serial}: SOFT STOP of {arm.config.name}'s tool failed: {error!r}")

    def resume(self) -> bool:
        """Undo a soft stop: accept commands again, switch the base's velocity controller on, and lift the arms' stop.

        The arms restart once all are operational; the base moves again only when sent a twist or a path.

        Returns:
            bool: True if every request went out. False if the controller manager or the sync is not reachable.
        """
        self.soft_stop_latch.stopped, self.soft_stop_latch.reason = False, ""
        # * Afresh: started as it was, the velocity controller brakes from its speed at the stop, so the base creeps.
        base = self.base.controllers.restart(PLATFORM_VELOCITY_CONTROLLER)
        arms = self._call_sync(self._resume_arms, "resume arms") if self.arms else True
        return base and arms

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

    def send_multi_arm_trajectory(self, positions: dict[str, Sequence[Sequence[float]]], duration: float,
                                  velocities: dict[str, Sequence[Sequence[float]]] | None = None) -> bool:
        """Send one trajectory to each arm, started together; nothing is sent unless both pass their checks.

        Args:
            positions: Waypoints per arm name, as for `ArmInterface.build_trajectory`.
            duration: Total time for both, seconds.
            velocities: Waypoint velocities per arm name, the same arms as `positions`, or None.

        Returns:
            bool: True if sent. False (reason logged) if the robot lacks two arms or a trajectory is refused.

        Raises:
            RobotStopped: If the robot is soft stopped.
            NotLeased: If the running code does not hold the robot's lease.
        """
        self.soft_stop_latch.check("a multi-arm trajectory")
        self.lease_slot.check("a multi-arm trajectory")
        if self._multi_arm is None:
            self._node.get_logger().error(f"robot {self.config.serial}: multi-arm trajectory "
                                          f"needs two arms, it has {len(self.arms)}")
            return False
        for given, what in ((positions, "waypoints"), (velocities, "velocities")):
            if given is not None and set(given) != set(self.arms):
                self._node.get_logger().error(f"robot {self.config.serial}: multi-arm trajectory needs "
                                              f"{what} for {sorted(self.arms)}, got {sorted(given)}")
                return False
        # trajectory1 / trajectory2 follow the configured arm order.
        first, second = self.arms.values()
        speeds = velocities or {}
        first_trajectory = first.build_trajectory(positions[first.config.name], duration,
                                                  speeds.get(first.config.name))
        second_trajectory = second.build_trajectory(positions[second.config.name], duration,
                                                    speeds.get(second.config.name))
        if first_trajectory is None or second_trajectory is None:
            return False
        message = MultiArmTrajectory()
        message.trajectory1 = first_trajectory
        message.trajectory2 = second_trajectory
        self._multi_arm.publish(message)
        # * Each arm's `trajectories_sent` fires as for a trajectory of its own, so recordings see two-arm motions too.
        first.trajectory_sent(first_trajectory, duration)
        second.trajectory_sent(second_trajectory, duration)
        first.mark_executing()
        second.mark_executing()
        return True

    # --- --- --- --- --- CALLBACKS (main thread) --- --- --- --- ---

    def _on_sync_status(self, message: SafetySyncStatus) -> None:
        """Hand each arm its entry of the sync's status, matched by namespace."""
        by_namespace = {status.name: status for status in message.arms}
        for arm in self.arms.values():
            status = by_namespace.get(arm.config.ros_namespace)
            if status is not None:
                arm.on_sync_status(status, message.stop_reason)
