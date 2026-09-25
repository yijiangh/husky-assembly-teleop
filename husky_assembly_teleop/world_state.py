"""
The measured world: where things *actually* are.

! Ownership. WorldState is a registry, not a second copy: per-robot measurements
  live in RobotState, owned by that robot's HuskyRobotInterface. If it ever
  caches a robot pose of its own, there are two answers to "where is the arm"
  and they will drift.

! Direction. State flows real -> planning, never back, and nothing outside a ROS
  callback may write here. Once the planner can write into the measurement, a
  reading no longer means anything. The old code broke this in three places; see
  doc/refactor_rationale.md.

! Threading. Written only by ROS callbacks, read by the tick and by plugins, all
  on the single-threaded executor, so no locking. viser callbacks run elsewhere
  and must go through PluginContext.submit.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .robot_interface import HuskyRobotInterface, RobotState


@dataclass
class TrackedObject:
    """A non-robot rigid body observed by mocap.

    ! Measurement only: no PyBullet body id, no mesh. Geometry belongs to
      whichever plugin put it in the scene, and the name is the link between the
      two. Mixing them is what made "real" and "simulated" impossible to pull
      apart in the old code.

    Attributes:
        name: Stable identifier, matching the CellObject it corresponds to.
        position: Position in world frame, metres.
        orientation: Orientation in world frame, quaternion.
        tracked: Whether the pose is a live fix. False means stale or never set.
        last_update_time: ROS time of the most recent observation, seconds.
    """

    name: str
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    orientation: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))
    tracked: bool = False
    last_update_time: float = 0.0


@dataclass
class WorldState:
    """Source of truth for everything measured.

    Attributes:
        robots: Connected robots, keyed by serial. Each owns its own RobotState.
        tracked_objects: Mocap-observed non-robot bodies, keyed by name.
    """

    robots: dict[str, HuskyRobotInterface] = field(default_factory=dict)
    tracked_objects: dict[str, TrackedObject] = field(default_factory=dict)

    def add_robot(self, robot: HuskyRobotInterface) -> None:
        """Register a robot.

        ? Explicit, rather than done by the robot's constructor, so constructing
          an object does not mutate a global scene as a side effect and the two
          can be separated for a test.

        Args:
            robot: The interface to register. Its serial must be unique.

        Raises:
            ValueError: If a robot with the same serial is already registered.
        """
        serial = robot.config.serial
        if serial in self.robots:
            raise ValueError(f"robot {serial} is already registered")
        self.robots[serial] = robot

    def robot_state(self, serial: str) -> RobotState:
        """Look up one robot's measured state.

        Args:
            serial: Robot serial, e.g. "a200-0806".

        Returns:
            RobotState: The live state object. Treat it as read-only.

        Raises:
            KeyError: If no such robot is registered.
        """
        return self.robots[serial].state

    def robot_states(self) -> dict[str, RobotState]:
        """Every robot's measured state, keyed by serial.

        Lets a consumer that only needs measurements -- RobotScene.sync_real --
        take those instead of the whole world.

        Returns:
            dict[str, RobotState]: The live state objects. Treat as read-only.
        """
        return {serial: robot.state for serial, robot in self.robots.items()}
