"""
The measured world: where things actually are, as reported by sensors.

! Registry only: per-robot measurements live in RobotState, owned by each
  HuskyRobotInterface. Don't cache robot poses here; two copies will drift.

! Data flows real -> planning, never back. Only ROS callbacks may write here, on
  the main thread (run by the tick's ROS pump), so there is no locking. viser callbacks must go
  through PluginContext.submit.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .robot_interface import HuskyRobotInterface, RobotState


@dataclass
class TrackedObject:
    """A non-robot rigid body observed by mocap (measurement only, no geometry).

    Attributes:
        name: Stable identifier, matching the CellObject it corresponds to.
        position: World-frame position, metres. None until the first valid fix,
            then the last valid one.
        orientation: World-frame quaternion (x, y, z, w). Set with `position`.
        tracked: Whether the latest sample was valid. Implies `position` is set.
        last_update_time: ROS time of the latest observation, seconds, or None.
    """

    name: str
    position: np.ndarray | None = None
    orientation: np.ndarray | None = None
    tracked: bool = False
    last_update_time: float | None = None


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
        """Register a robot; kept separate from its constructor so tests can build one alone.

        Args:
            robot: The interface to register. Its serial must be unique.

        Raises:
            ValueError: If a robot with the same serial is already registered.
        """
        serial = robot.config.serial
        if serial in self.robots:
            raise ValueError(f"robot {serial} is already registered")
        self.robots[serial] = robot

    def robot_states(self) -> dict[str, RobotState]:
        """Every robot's measured state, keyed by serial.

        Returns:
            dict[str, RobotState]: The live state objects. Treat as read-only.
        """
        return {serial: robot.state for serial, robot in self.robots.items()}
