"""
RRT-Connect over one arm's six joints, avoiding the robot itself, the other robots and every scene body.

! Worker thread only: sync, search and close on the worker.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Sequence

import numpy as np

from ..planning.path import TimedPath
from ..planning.search import PlanResult, connect
from ...robot_interface.arm import JOINT_MOVE_MAX_SPEED, UR_JOINT_LIMITS, UR_JOINT_NAMES
from bar_assembly_core.mirrors.compas_fab import CompasFabMirror
from bar_assembly_core.scene import Scene
from bar_assembly_core.ids import robot_id

#: Collision-checking resolution along a move, radians per joint.
JOINT_STEP = 0.05
#: Joint speed of the preview, rad/s: the same as robot_control's joint moves.
PREVIEW_SPEED = JOINT_MOVE_MAX_SPEED


def arm_joint_names(arm_name: str) -> list[str]:
    """The URDF names of one arm's six joints (e.g. arm "left_ur_arm"), in the UR driver's order."""
    return [f"{arm_name}_{name}" for name in UR_JOINT_NAMES]


@dataclass(frozen=True)
class ArmPath(TimedPath):
    """A joint path for one arm, timed at a constant joint speed for the preview.

    `points` are joint values in radians, shape (n, 6), in `joint_names` order.

    Attributes:
        joint_names: The six joints, in the order of each waypoint.
    """

    joint_names: tuple[str, ...]

    @classmethod
    def at_preview_speed(cls, joint_names: Sequence[str], waypoints: np.ndarray) -> ArmPath:
        """Time each segment by its largest joint change at PREVIEW_SPEED.

        Args:
            joint_names: The six joints.
            waypoints: (n, 6) joint values, radians; the first is the start.

        Returns:
            ArmPath: The timed path.
        """
        steps = np.max(np.abs(np.diff(waypoints, axis=0)), axis=1) / PREVIEW_SPEED
        return cls(times=np.concatenate([[0.0], np.cumsum(steps)]), points=waypoints, joint_names=tuple(joint_names))


class ArmPlanningWorld:
    """One compas_fab mirror per robot planned for, and the robot shown in the window. Worker thread only."""

    def __init__(self):
        """Start empty; a robot's mirror is made on its first sync (slow: loads the models)."""
        self._mirrors: dict[str, CompasFabMirror] = {}
        #: The robot synced last; the PyBullet window shows its world.
        self.serial: str | None = None
        #: Whether the PyBullet window is open.
        self.gui = False
        # Display text for our ids, from the last synced snapshot.
        self.label = Scene().label
        #: Given to each mirror: logs each full rebuild of its cell, and why. Set by the plugin.
        self.log: Callable[[str], None] | None = None

    def sync(self, snapshot: Scene, serial: str) -> CompasFabMirror:
        """Make one robot's planning world match a snapshot.

        Args:
            snapshot: The tick's copy of the world, taken on the main thread.
            serial: The robot to plan for.

        Returns:
            CompasFabMirror: Its mirror, synced.
        """
        mirror = self._mirror(serial)
        if self.gui and serial != self.serial:
            # ? One window per process: it follows the robot being planned for.
            self._mirrors[self.serial].set_gui(False)
            mirror.set_gui(True)
        self.serial = serial
        mirror.sync(snapshot)
        self.label = snapshot.label
        return mirror

    def set_gui(self, gui: bool, snapshot: Scene, serial: str) -> None:
        """Open or close PyBullet's own window on a robot's planning world, for debugging. Worker thread.

        Args:
            gui: Whether the window should be open.
            snapshot: The world now, shown even before the first plan.
            serial: The robot whose world to show.

        Raises:
            RuntimeError: If there is no X display.
            pybullet.error: If another PyBullet window is open in this process.
        """
        if self.serial is not None:
            self._mirrors[self.serial].set_gui(False)
        self.gui = False
        if gui:
            self._mirror(serial).set_gui(True)  # ? before sync, so the cell is built once
            self.gui = True
            self.sync(snapshot, serial)

    def _mirror(self, serial: str) -> CompasFabMirror:
        """A robot's mirror, made on first use."""
        if serial not in self._mirrors:
            self._mirrors[serial] = CompasFabMirror(robot_id(serial), log=self.log)
        return self._mirrors[serial]

    def close(self) -> None:
        """Disconnect every world. On the worker, once no search is running."""
        for mirror in self._mirrors.values():
            mirror.close()
        self._mirrors.clear()


def _extend(q1: Sequence[float], q2: Sequence[float]) -> list[tuple[float, ...]]:
    """Points from q1 (excluded) to q2 (included), at most JOINT_STEP apart on every joint."""
    q1, q2 = np.asarray(q1, dtype=float), np.asarray(q2, dtype=float)
    count = max(1, math.ceil(float(np.max(np.abs(q2 - q1))) / JOINT_STEP))
    return [tuple(q1 + (q2 - q1) * i / count) for i in range(1, count + 1)]


def _distance(q1: Sequence[float], q2: Sequence[float]) -> float:
    """Straight-line distance in joint space, radians."""
    return float(np.linalg.norm(np.subtract(q1, q2)))


def plan_arm(world: ArmPlanningWorld, mirror: CompasFabMirror, arm_name: str, goal: Sequence[float],
             abort: threading.Event) -> PlanResult[ArmPath]:
    """Plan a collision-free joint path for one arm, from where it is in the synced world to `goal`.

    Args:
        world: The planning world, for labels.
        mirror: The robot's mirror, already synced.
        arm_name: Which arm, e.g. "ur_arm".
        goal: Six joint values, radians, UR driver order.
        abort: Set from the main thread to end the search early.

    Returns:
        PlanResult[ArmPath]: The path, or why there is none.
    """
    started = time.time()
    names = arm_joint_names(arm_name)
    start = tuple(float(mirror.state.robot_configuration[name]) for name in names)
    goal = tuple(float(value) for value in goal)

    def reason(where: str, pair: tuple[str, str]) -> str:
        return f"{where} is in collision: {world.label(pair[0])} with {world.label(pair[1])}"

    # * Check the start in full once; the fast `search_check` skips pairs the arm does not move.
    hits = mirror.collisions()
    if hits:
        return PlanResult(None, reason("start", hits[0]), time.time() - started)
    check = mirror.search_check(names)
    hit = check(goal)
    if hit is not None:
        return PlanResult(None, reason("target", hit), time.time() - started)

    limits = np.array(UR_JOINT_LIMITS)
    corners = connect(start, goal, _distance, lambda: tuple(np.random.uniform(-limits, limits)), _extend,
                      lambda q: check(q) is not None, abort)
    if corners.states is None:
        return PlanResult(None, corners.reason, time.time() - started)
    path = ArmPath.at_preview_speed(names, np.array(corners.states, dtype=float))
    return PlanResult(path, seconds=time.time() - started, direct=corners.direct)
