"""
Clearances: while a task drives a robot, stop it once it comes too close to anything in the scene.

Optional, like freezes. Whoever puts bodies into the scene (obstacles, process, tracked objects) need not know.

- Checked against the whole tick snapshot, other robots included; allowed contacts (`Body.touches`, what a robot
  holds) are skipped.
- ! Other robots must be measured too, except those in `Clearance.unknown_robots` (unknown when it was committed,
  checked as their stand-ins, `stand_ins.py`): any other robot not measured trips it, and refuses it at the start.
- ! What cannot be checked counts as too close: a lost base pose, or a robot the mirror cannot place. Only arms named
  in `Clearance.unknown_arms` may be unknown (manual moves): they are checked where last seen, or stowed. A robot
  that cannot be checked when a clearance starts refuses it (`ClearanceRefused`), so nothing is sent.
- A robot is also checked against itself, within SELF_DISTANCE: links, tools and what it holds; pairs that touch by
  design (joined links, the SRDF's, a tool's flange links) are left out (`BaseCollisionWorld.self_hit`).
- One worker thread checks one snapshot at a time, a tick or two behind the robot. Not an emergency stop.
- On a trip, `PluginContext.clearance` soft stops the robot (Resume needed) and cancels the task. The monitor keeps
  the last TRIPS_KEPT trips with what came too close (`ClearanceTrip`), for the collisions plugin.
- A watch (`ClearanceWatch`, `PluginContext.watch_clearance`) is checked the same way, every WATCH_EVERY, and only
  reports: e.g. to show before a press that a drive would start too close.
"""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field, replace
from typing import Any, Collection, Hashable, Iterable, Literal

from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Scene

from ..robot_interface.soft_stop import MotionRefused
from .base_collision import BaseCollisionWorld
from .collision_hit import CollisionHit
from .stand_ins import unknown_parts, unknown_robots, with_stand_ins

#: Default distance, metres, below which a clearance trips.
STOP_DISTANCE = 0.1
#: Distance kept while an arm moves (jogs, planned paths), metres: tight, so it can still reach close to things.
ARM_STOP_DISTANCE = 0.01
#: How many trips the monitor keeps.
TRIPS_KEPT = 10
#: A robot keeps this far from itself (links, tools, what it holds), metres; the SRDF's and tools' allowed pairs aside.
SELF_DISTANCE = ARM_STOP_DISTANCE
#: For `unknown_arms`: every arm of the robot.
ALL_ARMS = "all"
#: Seconds between checks of the watches (`ClearanceWatch`).
WATCH_EVERY = 0.5


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
        unknown_robots: Other robots that may be unknown, by id: checked as their stand-ins, live once measured again.
        stand_ins: Other robots not measured when it started, and how they are checked (`StandIns.warnings`).
        refuse_first: Too close at the first check refuses it instead: no soft stop, `refused` set. ! Then send
            nothing until `checked`.
        checked: Whether a check found it clear yet.
        refused: Whether it ended at its first check, too close, without a soft stop (`refuse_first`).
    """

    serials: tuple[str, ...]
    margin: float
    problem: str | None = None
    hit: CollisionHit | None = None
    unknown_arms: Collection[str] | Literal["all"] = ()
    unknown_robots: frozenset[str] = frozenset()
    stand_ins: tuple[str, ...] = ()
    refuse_first: bool = False
    checked: bool = False
    refused: bool = False


@dataclass(eq=False)
class ClearanceWatch:
    """Robots a plugin wants to know about without driving them: what each is too close to, as a clearance would see.

    Never trips: nothing is stopped or cancelled. The other robots not measured are checked as stand-ins, as at the
    start of a clearance; a robot that cannot be checked is left out of `found`.

    Attributes:
        serials: The robots, by serial.
        margin: Distance below which a robot counts as too close, metres.
        unknown_arms: Arms whose joints may be unknown, checked where last seen or stowed, or ALL_ARMS.
        found: Per serial, what the last check found it too close to, e.g. "came within 10 cm of wall"; empty if clear.
        checked_at: ROS time of the last finished check, or None before the first.
    """

    serials: tuple[str, ...]
    margin: float
    unknown_arms: Collection[str] | Literal["all"] = ()
    found: dict[str, str] = field(default_factory=dict)
    checked_at: float | None = None


class ClearanceRefused(MotionRefused):
    """A clearance could not check its robot when it started (`clearance_problem`): nothing was sent."""

    label = "cannot check"


def clearance_problem(robot: RobotObject | None, assumable: frozenset[str] = frozenset()) -> str:
    """Why a clearance cannot check a robot, or "": absent, base not tracked, or joints not known it may not assume.

    Args:
        robot: The robot in the snapshot, or None if it is not there.
        assumable: Joints that may be unknown (`assumable_joints`).
    """
    if robot is None or not robot.enabled:
        return "it is not in the scene"
    parts = unknown_parts(replace(robot, unmeasured=robot.unmeasured - assumable))
    return f"{parts} not measured" if parts else ""


def others_problem(snapshot: Scene, own: Collection[str], allowed: Collection[str]) -> str:
    """Why a clearance cannot check its robots against the others: those not measured it may not stand in; or "".

    Args:
        snapshot: The tick's snapshot.
        own: Ids of the robots the clearance keeps clear.
        allowed: Ids of the other robots that may be unknown (`Clearance.unknown_robots`).
    """
    return "; ".join(f"{key.split('/', 1)[1]}: {unknown_parts(snapshot.robots[key])} not measured"
                     for key in sorted(unknown_robots(snapshot, own) - set(allowed)))


def self_text(snapshot: Scene, pair: tuple[str, str]) -> str:
    """A pair of one robot's parts, short, e.g. "left_ur_arm_wrist_3_link with right_ur_arm tool"."""
    def short(object_id: str) -> str:
        if object_id in snapshot.bodies:
            return snapshot.label(object_id)
        kind, *_robot, name = object_id.split("/")
        return f"{name} tool" if kind == "tools" else name
    return f"{short(pair[0])} with {short(pair[1])}"


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

    def check(self, snapshot: Scene, asked: list[tuple[Hashable, str, float, frozenset[str], frozenset[str]]]) -> bool:
        """Start checking each (key, robot id, margin, joints and other robots that may be unknown) in `snapshot`.

        Skipped while the last check runs.

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

    def _check(self, snapshot: Scene, asked: list[tuple[Hashable, str, float, frozenset[str], frozenset[str]]]
               ) -> list[tuple[Any, str, CollisionHit | None]]:
        """Sync the world to `snapshot` and say why each robot that is too close, or cannot be checked, is. Worker."""
        found, checked = [], []
        for key, own, margin, assumable, allowed in asked:
            problem = clearance_problem(snapshot.robots.get(own), assumable) or others_problem(snapshot, {own}, allowed)
            if problem:
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
                continue
            # * Itself only within SELF_DISTANCE: links of one robot sit close by design.
            self_margin = min(margin, SELF_DISTANCE)
            hit = self._world.self_hit(own, self_margin, "clearance")
            if hit is not None:
                why = f"came within {self_margin * 100:.0f} cm of itself: {self_text(snapshot, hit.pairs[0])}"
                found.append((key, why, hit))
        return found
