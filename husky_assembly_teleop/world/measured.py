"""
The measured world: where things actually are, as reported by sensors.

- ! Don't cache robot poses here: they live in each robot's RobotState, and two copies drift.
- ! Only ROS callbacks write here, on the main thread, so there is no locking; viser callbacks
  must go through PluginContext.submit.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

from ..robot_interface.robot import HuskyRobotInterface


@dataclass
class TrackedObject:
    """A non-robot rigid body observed by mocap (measurement only, no geometry); judged like BaseState.

    Attributes:
        name: Stable identifier, matching the CellObject it corresponds to.
        mocap_id: Rigid-body id in the mocap system it is tracked by.
        position: World-frame position of the last valid fix, metres, or None before one.
        orientation: World-frame quaternion (x, y, z, w). Set with `position`.
        tracked: Whether the latest sample was valid. Implies `position` is set.
        tracking_valid: Whether NatNet tracked the body in the latest sample.
            False usually means hidden markers.
        marker_errors: (ROS time, marker error) of the valid samples in the last MARKER_ERROR_WINDOW, oldest first.
        marker_error: Mean marker error of the latest sample, metres.
        last_update_time: ROS time of the latest observation, seconds, or None.
        last_fix_time: ROS time of the latest valid observation, seconds, or None.
    """

    name: str
    mocap_id: int
    position: np.ndarray | None = None
    orientation: np.ndarray | None = None
    tracked: bool = False
    tracking_valid: bool | None = None
    marker_error: float | None = None
    marker_errors: deque[tuple[float, float]] = field(default_factory=deque)
    last_update_time: float | None = None
    last_fix_time: float | None = None


@dataclass
class WorldState:
    """Source of truth for everything measured.

    Attributes:
        robots: Connected robots, keyed by serial. Each owns its own RobotState.
        tracked_objects: Mocap-observed non-robot bodies, keyed by name. Changed only through
            PluginContext.track_object / untrack_object.
    """

    robots: dict[str, HuskyRobotInterface] = field(default_factory=dict)
    tracked_objects: dict[str, TrackedObject] = field(default_factory=dict)

    def add_robot(self, robot: HuskyRobotInterface) -> None:
        """Register a robot.

        Args:
            robot: The interface to register. Its serial must be unique.

        Raises:
            ValueError: If a robot with the same serial is already registered.
        """
        serial = robot.config.serial
        if serial in self.robots:
            raise ValueError(f"robot {serial} is already registered")
        self.robots[serial] = robot
