"""
Clearances: while a task drives a robot, stop it once it comes too close to anything in the scene.

Optional, like freezes. Whoever puts bodies into the scene (obstacles, process, tracked objects) need not know.

- Checked against the whole tick snapshot, other robots included; allowed contacts (`Body.touches`, what a robot
  holds) are skipped. Other robots not measured are checked as their stand-ins (`stand_ins.py`).
- ! What cannot be checked counts as too close: a lost base pose, or a robot the mirror cannot place. Only arms named
  in `Clearance.unknown_arms` may be unknown (manual moves): they are checked where last seen, or stowed. A robot
  that cannot be checked when a clearance starts refuses it (`ClearanceRefused`), so nothing is sent.
- One worker thread checks one snapshot at a time, a tick or two behind the robot. Not an emergency stop.
- On a trip, `PluginContext.clearance` soft stops the robot (Resume needed) and cancels the task. The monitor keeps
  the last TRIPS_KEPT trips with what came too close (`ClearanceTrip`), for the collisions plugin.
"""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, replace
from typing import Any, Collection, Hashable, Iterable, Literal

from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Scene

from ..robot_interface.soft_stop import MotionRefused
from .base_collision import BaseCollisionWorld
from .collision_hit import CollisionHit
from .stand_ins import with_stand_ins

#: Default distance, metres, below which a clearance trips.
STOP_DISTANCE = 0.1
#: How many trips the monitor keeps.
TRIPS_KEPT = 10
#: For `unknown_arms`: every arm of the robot.
ALL_ARMS = "all"


@dataclass(eq=False)
class Clearance:
    """Robots a task keeps clear of everything, and why it tripped.

    Attributes:
        serials: The robots, by serial.
        margin: Distance below which it trips, metres.
        problem: Why it tripped, or None while it holds.
        hit: What came too close, once tripped; None while it holds, or when it could not be checked.
        unknown_arms: Arms whose joints may be unknown, checked where last seen or stowed, or ALL_ARMS; none by
            default.
            The base pose must always be known.
        stand_ins: Other robots not measured when it started, and how they are checked (`StandIns.warnings`).
    """

    serials: tuple[str, ...]
    margin: float
    problem: str | None = None
    hit: CollisionHit | None = None
    unknown_arms: Collection[str] | Literal["all"] = ()
    stand_ins: tuple[str, ...] = ()


class ClearanceRefused(MotionRefused):
    """A clearance could not check its robot when it started (`clearance_problem`): nothing was sent."""

    label = "cannot check"


def clearance_problem(robot: RobotObject | None, assumable: frozenset[str] = frozenset()) -> str:
    """Why a clearance cannot check a robot, or "": absent, base not tracked, or joints not known it may not assume.

    Args:
        robot: The robot in the snapshot, or None if it is not there.
        assumable: Joints that may be unknown (`assumable_joints`).
    """
    if robot is None:
        return "it is not in the scene"
    return "; ".join(replace(robot, unmeasured=robot.unmeasured - assumable).acting_problems())


def assumable_joints(joints: Iterable[str], unknown_arms: Collection[str] | Literal["all"]) -> frozenset[str]:
    """The joints among `joints` that a clearance may take at their stow pose: those of `unknown_arms`.

    Args:
        joints: A robot's movable joints, e.g. `RobotModel.movable_joints`.
        unknown_arms: Arm names (a joint "<arm>_<name>" belongs to arm "<arm>"), or ALL_ARMS for every joint.
    """
    if unknown_arms == ALL_ARMS:
        return frozenset(joints)
    return frozenset(joint for joint in joints if any(joint.startswith(f"{arm}_") for arm in unknown_arms))


@dataclass(frozen=True)
class ClearanceTrip:
    """One trip, as the monitor keeps it.

    Attributes:
        time: When it tripped, ROS time in seconds.
        serial: The robot stopped.
        task: The task cancelled, e.g. "base_exp: run path".
        problem: Why, as in `Clearance.problem`.
        hit: What came too close, with the scene as checked; None when the robot could not be checked.
    """

    time: float
    serial: str
    task: str
    problem: str
    hit: CollisionHit | None


class ClearanceChecker:
    """Checks robots against a scene snapshot on its own worker thread, one snapshot at a time."""

    def __init__(self):
        """Start idle; the worker's collision world loads on the first check."""
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="clearance")
        self._world = BaseCollisionWorld()
        self._pending: Future | None = None

    def check(self, snapshot: Scene, asked: list[tuple[Hashable, str, float, frozenset[str]]]) -> bool:
        """Start checking each (key, robot id, margin, joints that may be unknown) in `snapshot`, unless one runs.

        Returns:
            bool: Whether the check started; False while the last one runs (this snapshot is skipped).
        """
        if self._pending is not None:
            return False
        self._pending = self._executor.submit(self._check, snapshot, list(asked))
        return True

    def finished(self) -> list[tuple[Any, str, CollisionHit | None]] | None:
        """The (key, why, hit) of each robot the last check found too close, once; None while it runs or none started.

        Raises:
            Exception: Whatever the check raised.
        """
        if self._pending is None or not self._pending.done():
            return None
        done, self._pending = self._pending, None
        return done.result()

    def close(self) -> None:
        """Close the collision world and stop the worker, after the running check."""
        self._executor.submit(self._world.close)
        self._executor.shutdown(wait=True)

    def _check(self, snapshot: Scene, asked: list[tuple[Hashable, str, float, frozenset[str]]]
               ) -> list[tuple[Any, str, CollisionHit | None]]:
        """Sync the world to `snapshot` and say why each robot that is too close, or cannot be checked, is. Worker."""
        found, checked = [], []
        for key, own, margin, assumable in asked:
            if problem := clearance_problem(snapshot.robots.get(own), assumable):
                found.append((key, f"cannot be checked: {problem}", None))
            else:
                checked.append((key, own, margin))
        # * The unknown joints left may be: take them as known, where the kinematics has them (last seen, or stowed).
        #   A strict clearance on the same robot failed above already.
        lenient = {own for _key, own, _margin in checked if snapshot.robots.get(own) is not None
                   and snapshot.robots[own].unmeasured}
        if lenient:
            snapshot = replace(snapshot, robots={object_id: replace(robot, unmeasured=frozenset())
                                                 if object_id in lenient else robot
                                                 for object_id, robot in snapshot.robots.items()})
        snapshot = with_stand_ins(snapshot, {own for _key, own, _margin in checked}).scene
        self._world.sync(snapshot)
        for key, own, margin in checked:
            try:
                hit = self._world.collision_hit(own, margin, "clearance")
            except (ValueError, KeyError) as error:
                found.append((key, f"cannot be checked: {error}", None))
                continue
            if hit is not None:
                found.append((key, f"came within {margin * 100:.0f} cm of {snapshot.label(hit.pairs[0][1])}", hit))
        return found
