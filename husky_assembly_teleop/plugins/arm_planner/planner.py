"""
The arm planner: an RRT-Connect search (`planning.search.connect`) over one arm's six joints, avoiding
the robot itself, the other robots and every scene body, with compas_fab's rules.

The search runs on the plugin's one worker thread, in a compas_fab world
(`CompasFabMirror`, one per robot) synced on that thread from the tick's scene snapshot.

* The start is checked once in full (`collisions`); the search then uses the mirror's
  fast `search_check`, which only looks at what the arm moves.
! Worker thread only: sync, search and close on the worker.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from ...planning.path import TimedPath
from ...planning.search import PlanResult, connect
from ...robot_interface.arm import JOINT_MOVE_MAX_SPEED, UR_JOINT_LIMITS, UR_JOINT_NAMES
from ...world.mirrors.compas_fab import CompasFabMirror
from ...world.scene import SceneSnapshot

#: Collision-checking resolution along a move, radians per joint.
JOINT_STEP = 0.05
#: Joint speed of the preview, rad/s: the same as robot_control's joint moves.
PREVIEW_SPEED = JOINT_MOVE_MAX_SPEED


def arm_joint_names(arm_name: str) -> list[str]:
    """The URDF names of one arm's joints, in the UR driver's order.

    Args:
        arm_name: E.g. "left_ur_arm".

    Returns:
        list[str]: Six joint names.
    """
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
        # Display text for our ids, from the last synced snapshot.
        self.label = SceneSnapshot().label

    def sync(self, snapshot: SceneSnapshot, serial: str) -> CompasFabMirror:
        """Make one robot's planning world match a snapshot.

        Args:
            snapshot: The tick's copy of the world, taken on the main thread.
            serial: The robot to plan for.

        Returns:
            CompasFabMirror: Its mirror, synced.
        """
        mirror = self._mirrors.get(serial)
        if mirror is None:
            mirror = self._mirrors[serial] = CompasFabMirror(serial)
        gui = self.serial is not None and self._mirrors[self.serial].gui
        if serial != self.serial and gui:
            # ? One window per process: it follows the robot being planned for.
            self._mirrors[self.serial].set_gui(False)
        self.serial = serial
        mirror.sync(snapshot)
        if gui:
            mirror.set_gui(True)
        self.label = snapshot.label
        return mirror

    def set_gui(self, gui: bool, snapshot: SceneSnapshot, serial: str) -> tuple[bool, str]:
        """Open or close PyBullet's own window on a robot's planning world. Worker thread.

        Opening builds the robot's world from `snapshot` if it has none yet (slow the
        first time: loads the models), so the window works before the first plan.

        Args:
            gui: Whether the window should be open.
            snapshot: The world now.
            serial: The robot whose world to show.

        Returns:
            tuple[bool, str]: Whether it is open now, and why not if it was asked for.
        """
        if not gui:
            mirror = self._mirrors.get(self.serial)
            return (False, "") if mirror is None else (mirror.set_gui(False), "")
        if self.serial not in (None, serial):
            self._mirrors[self.serial].set_gui(False)  # ? one window per process
        mirror = self._mirrors.get(serial)
        if mirror is None:
            mirror = self._mirrors[serial] = CompasFabMirror(serial)
        self.serial = serial
        # ? Window first, then sync: the other order would build the cell twice.
        is_open = mirror.set_gui(True)
        mirror.sync(snapshot)
        self.label = snapshot.label
        return is_open, mirror.window_problem

    def window(self) -> tuple[bool, str]:
        """Whether the window is open, and why not. Worker thread, e.g. after a search."""
        mirror = self._mirrors.get(self.serial)
        return (False, "") if mirror is None else (mirror.gui, mirror.window_problem)

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
        mirror: The robot's mirror, already synced. Owned by this call until it returns.
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

    # * A full check of the start, static pairs included; the search check leaves those out.
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
