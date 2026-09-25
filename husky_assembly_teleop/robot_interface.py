"""
One physical robot: its URDF-defined configuration, its ROS2 I/O, and the
measured state that comes back from it.

The only module that talks to a robot. Everything above reads RobotState and
calls the command methods; nothing above knows a topic name.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from rclpy.node import Node

from .config import RobotConfig


@dataclass
class RobotState:
    """Measured state of one robot. Written by ROS callbacks, read by everyone else.

    ! Every field is per-instance, through the dataclass. The old interface
      declared these at class scope and mutated them in place, so two robots
      shared one list -- and holding N robots is the whole point of WorldState.

    Attributes:
        serial: Which robot this describes.
        base_position: Base position in world frame, metres.
        base_orientation: Base orientation in world frame, quaternion.
        base_tracked: Whether the base pose is a live mocap fix. False means
            stale or never set, and callers must not read it as a measurement.
        joint_positions: Joint name to measured position, radians.
        joint_velocities: Joint name to measured velocity, radians per second.
        wrenches: Force/torque sensor readings, keyed by sensor name.
        active_controllers: Currently running controller name per arm group.
        is_executing: Whether a trajectory is running, per arm group.
        last_update_time: ROS time of the most recent message, seconds. Lets a
            reader notice that a robot has gone quiet.
    """

    serial: str
    base_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    base_orientation: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))
    base_tracked: bool = False
    joint_positions: dict[str, float] = field(default_factory=dict)
    joint_velocities: dict[str, float] = field(default_factory=dict)
    wrenches: dict[str, np.ndarray] = field(default_factory=dict)
    active_controllers: dict[str, str] = field(default_factory=dict)
    is_executing: dict[str, bool] = field(default_factory=dict)
    last_update_time: float = 0.0


class HuskyRobotInterface:
    """ROS2 interface to one husky: its topics, services and measured state.

    ! Threading. Every subscription callback here runs on the node's
      single-threaded executor, the same thread as the tick, so writes to
      `self.state` are already serialised against every reader and need no
      locking. Mocap arrives the same way, as an ordinary subscription. This
      holds only while the node keeps one executor and the default
      mutually-exclusive callback group; if that changes, this note was wrong.
    """

    def __init__(self, node: Node, config: RobotConfig):
        """Connect to one robot and start listening.

        Args:
            node: The monitor node, used to create subscriptions, publishers and
                clients. This class does not own a node of its own.
            config: Identity and URDF for this robot.
        """
        self._node = node
        self.config = config
        self.state = RobotState(serial=config.serial)

        # ? No parsed URDF here yet.
        #   An earlier draft loaded one for joint names, link names and which
        #   end effectors are mounted -- the things the old code hardcoded in
        #   per-configuration constant tables. Nothing read it: RobotScene parses
        #   the URDF for the scene, and the subscriptions below will learn joint
        #   names from the messages they receive. Add a model back when a caller
        #   actually needs one, and it will be clear then what type it should be.

        # TODO subscriptions: joint states, dynamic joint states, IO states,
        #      force/torque, tool status, controller list, and the mocap topic
        #      for this robot's rigid body. All of them write into self.state and
        #      do nothing else -- no planning, no drawing, no file IO in a
        #      callback.
        # TODO publishers: cmd_vel, joint trajectory, target frame, target wrench,
        #      tool command.
        # TODO service and action clients: SetIO, SwitchController,
        #      FollowJointTrajectory, GripperCommand.
        # TODO integrate crl_husky.config_resolver for the mocap calibration.
        #      ! Use only the explicit per-serial lookups
        #      (get_mocap_ids_for_robot_serial, get_mocap_calibration_for_id).
        #      Its get_robot_sn / get_robot_name / get_primary_mocap_id read
        #      environment variables set by Clearpath systemd *on board a robot*;
        #      the monitor runs off-board and watches several, so that identity
        #      is meaningless here.

    # --- --- --- --- --- COMMANDS --- --- --- --- ---
    # ! ROS thread only: from a plugin's update, a job step, or an intent
    #   drained at the start of that plugin's step. Never from a viser callback.

    def send_joint_trajectory(self, group: str, trajectory: list[dict[str, float]],
                              duration: float) -> None:
        """Send a joint trajectory to one arm group.

        Args:
            group: Arm group name as it appears in the URDF, e.g. "ur_arm".
            trajectory: Waypoints, each mapping joint name to position in radians.
            duration: Total execution time in seconds.
        """
        raise NotImplementedError

    def switch_controller(self, group: str, controller: str) -> None:
        """Request a controller switch for one arm group.

        Asynchronous; completion shows up in state.active_controllers, which
        callers poll.

        Args:
            group: Arm group name.
            controller: Name of the controller to activate.
        """
        raise NotImplementedError
