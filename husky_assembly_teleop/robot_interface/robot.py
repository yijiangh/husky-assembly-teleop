"""
One physical robot: its base, its arms, and the state they measure together.

HuskyRobotInterface only composes. Every topic belongs to the part it concerns
-- BaseInterface, ArmInterface, the end effectors -- and the configuration
decides which parts exist. The one exception is the multi-arm trajectory, which
concerns two arms at once and so lives here.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

from crl_husky_msgs.msg import MultiArmTrajectory
from rclpy.node import Node

from ..config import RobotConfig
from .arm import ArmInterface, ArmState
from .base import BaseInterface, BaseState


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
        self.arms = {arm.name: ArmInterface(node, config.ros_namespace, arm) for arm in config.arms}
        self.state = RobotState(serial=config.serial, base=self.base.state,
                                arms={name: arm.state for name, arm in self.arms.items()})

        # * Only a robot with two arms has the on-board node that splits this
        #   message into two synchronised trajectories.
        self._multi_arm = None
        if len(self.arms) == 2:
            self._multi_arm = node.create_publisher(
                MultiArmTrajectory, f"/{config.ros_namespace}/multi_arm_joint_trajectory", 10)

    # --- --- --- --- --- COMMANDS --- --- --- --- ---
    # ! ROS thread only, like the commands on the parts.

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
