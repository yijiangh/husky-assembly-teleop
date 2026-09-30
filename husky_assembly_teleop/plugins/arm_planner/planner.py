"""
The arm planner: an RRT-Connect search (`birrt`) over one arm's six joints, avoiding
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
from dataclasses import dataclass, field
from typing import Sequence

import numpy as np
from pybullet_planning.motion_planners.rrt_connect import birrt
from pybullet_planning.motion_planners.utils import waypoints_from_path

from ...robot_interface.arm import JOINT_MOVE_MAX_SPEED, UR_JOINT_LIMITS, UR_JOINT_NAMES
from ...world.mirrors.compas_fab import CompasFabMirror
from ...world.scene import SceneSnapshot

#: Give up searching after this long, seconds. The first plan for a robot also loads its models.
TIME_LIMIT = 10.0
#: RRT-Connect iterations per attempt, extra attempts, and smoothing iterations.
MAX_ITERATIONS = 2000
RESTARTS = 2
SMOOTHING = 100
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


@dataclass
class ArmPath:
    """A joint path for one arm, timed at a constant joint speed for the preview.

    Attributes:
        joint_names: The six joints, in the order of each waypoint.
        waypoints: (n, 6) joint values, radians; the first is the start.
        times: Time at each waypoint, seconds from the start.
    """

    joint_names: list[str]
    waypoints: np.ndarray
    times: np.ndarray = field(init=False)

    def __post_init__(self):
        """Time each segment by its largest joint change at PREVIEW_SPEED."""
        steps = np.max(np.abs(np.diff(self.waypoints, axis=0)), axis=1) / PREVIEW_SPEED
        self.times = np.concatenate([[0.0], np.cumsum(steps)])

    @property
    def duration(self) -> float:
        """float: Seconds from start to goal."""
        return float(self.times[-1])

    @property
    def start(self) -> np.ndarray:
        """np.ndarray: The first waypoint."""
        return self.waypoints[0]

    @property
    def goal(self) -> np.ndarray:
        """np.ndarray: The last waypoint."""
        return self.waypoints[-1]

    def sample(self, t: float) -> np.ndarray:
        """Joint values at time `t`, straight between waypoints; clamped to the ends.

        Args:
            t: Seconds from the start.

        Returns:
            np.ndarray: Six joint values.
        """
        return np.array([np.interp(t, self.times, column) for column in self.waypoints.T])


class PlanningAborted(Exception):
    """Raised inside a search to end it early."""


@dataclass
class PlanResult:
    """What a search came back with.

    Attributes:
        path: The path, or None if none was found.
        reason: Why not, when `path` is None; empty otherwise.
        seconds: How long it took, loading and syncing included.
        direct: Whether the straight move was already free.
    """

    path: ArmPath | None
    reason: str = ""
    seconds: float = 0.0
    direct: bool = False


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
             abort: threading.Event) -> PlanResult:
    """Plan a collision-free joint path for one arm, from where it is in the synced world to `goal`.

    Args:
        world: The planning world, for labels.
        mirror: The robot's mirror, already synced. Owned by this call until it returns.
        arm_name: Which arm, e.g. "ur_arm".
        goal: Six joint values, radians, UR driver order.
        abort: Set from the main thread to end the search early.

    Returns:
        PlanResult: The path, or why there is none.
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

    def sample_fn():
        return tuple(np.random.uniform(-limits, limits))

    def collision_fn(q, **_kwargs):
        # birrt may pass a `diagnosis` flag; ignored.
        if abort.is_set():
            raise PlanningAborted()
        return check(q) is not None

    try:
        direct = all(not collision_fn(q) for q in _extend(start, goal))
        waypoints = [start, goal] if direct else \
            birrt(start, goal, _distance, sample_fn, _extend, collision_fn, max_time=TIME_LIMIT,
                  max_iterations=MAX_ITERATIONS, restarts=RESTARTS, smooth=SMOOTHING)
    except PlanningAborted:
        return PlanResult(None, "cancelled", time.time() - started)
    if waypoints is None:
        return PlanResult(None, f"no path found in {TIME_LIMIT:.0f} s", time.time() - started)

    # ! birrt returns every small step, maybe without the start: keep only the corners,
    #   ending exactly at the goal (interpolated steps are off by rounding).
    if tuple(waypoints[0]) != start:
        waypoints = [start] + list(waypoints)
    waypoints = waypoints_from_path(waypoints)[:-1] + [goal]
    return PlanResult(ArmPath(names, np.array(waypoints, dtype=float)), seconds=time.time() - started, direct=direct)
