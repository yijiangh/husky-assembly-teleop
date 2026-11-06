"""
What planning a movement gave (`RouteResult`), what checking its goal gave (`GoalCheck`), and pose helpers for goals.

`goal_line` puts a goal check into one log line.

Pure: any thread.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Mapping

import numpy as np

from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import split_link_id
from scaffolding_assembly_core.process import LineSpec, Movement, Process
from scaffolding_assembly_core.process.solutions import MovementResult

from .collision import SweptReport
from .ik import IkResult
from .movement import MovementSpec, Routine

if TYPE_CHECKING:
    from ...world.collision_hit import CollisionHit


@dataclass(frozen=True)
class RouteResult:
    """One movement planned (or not), with what the planner and the checks found.

    Attributes:
        result: The core's result, in process ids: status "solved", "failed" or "not_planned".
        routine: How the movement was planned; None: it failed before it was classified.
        planner: Which planner ran, e.g. "tamp.plan_free_dual_arm" or "single_arm.plan_linear/ssik".
        attempts: How many tries ran.
        seconds: Time taken: scenes, syncs, IK, searches and checks.
        waypoints: Rows of the path; 0 without one.
        end_error: Process flange id -> how far the path's end puts it from its goal pose: metres, radians.
        goals: Process flange id -> the world pose its tool0 was planned to end at; empty for a joint goal.
        joint_error: The largest gap between the path's end and a joint goal, radians; None for a pose goal.
        goal_joints: The arm joints planned to, where the goal was in joints (given, or solved by IK).
        colliding: Colliding pairs found on the way, in process ids where mapped.
        hits: The collisions found on the way, in scene ids, for the collisions plugin.
        swept: The swept check of the path, where one ran.
        ik: The goal's IK, where one ran.
        warnings: Robots left out of the scene, a colliding start of a linear move, a mismatched grasp.
    """

    result: MovementResult
    routine: Routine | None
    planner: str = ""
    attempts: int = 0
    seconds: float = 0.0
    waypoints: int = 0
    end_error: dict[str, tuple[float, float]] = field(default_factory=dict)
    joint_error: float | None = None
    goal_joints: dict[str, float] | None = None
    colliding: tuple[tuple[str, str], ...] = ()
    swept: SweptReport | None = None
    ik: IkResult | None = None
    warnings: tuple[str, ...] = ()
    hits: tuple[CollisionHit, ...] = ()
    goals: dict[str, Pose] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        """bool: Whether the movement was solved."""
        return self.result.status == "solved"


@dataclass(frozen=True)
class GoalCheck:
    """A movement's goal solved by IK at the live base, without planning to it.

    Attributes:
        movement: The movement's id.
        routine: How the movement would be planned; None: it failed before it was classified.
        ok: Whether IK found a collision-free goal within the joint limits.
        reason: Why not, when not ok.
        acting: The acting robot's live id.
        base: Its base the goal was solved at.
        start: Its arm joints now, the start the goal is turned next to.
        ik: The IK result, where IK ran.
        goal_joints: The goal a plan would go to: IK's, turned by whole turns next to the start where the routine does.
        authored: The movement's target joints, the reference IK stays near.
        sources: Live id of each other robot -> where its base and joints came from (`scene_for`).
        warnings: Robots and bodies left out of the scene, and why.
        seconds: Time taken: scenes, syncs and IK.
        hits: What the goal collides with, where IK found only colliding solutions, for the collisions plugin.
    """

    movement: str
    routine: Routine | None
    ok: bool
    reason: str = ""
    acting: str = ""
    base: Pose | None = None
    start: dict[str, float] | None = None
    ik: IkResult | None = None
    goal_joints: dict[str, float] | None = None
    authored: dict[str, float] | None = None
    sources: dict[str, str] = field(default_factory=dict)
    warnings: tuple[str, ...] = ()
    seconds: float = 0.0
    hits: tuple[CollisionHit, ...] = ()

    @property
    def max_from_authored(self) -> float | None:
        """The largest gap between IK's joints (before any turn) and the authored ones, radians; None without both."""
        if self.ik is None or self.ik.joints is None or not self.authored:
            return None
        gaps = [abs(value - self.authored[name]) for name, value in self.ik.joints.items() if name in self.authored]
        return max(gaps) if gaps else None

    def as_record(self) -> dict:
        """The check as JSON values, for a run's events: enough to solve it again offline."""
        ik = self.ik
        return {"movement": self.movement, "routine": self.routine.value if self.routine is not None else None,
                "ok": self.ok, "reason": self.reason, "acting": self.acting,
                "base": [*self.base.position, *self.base.orientation] if self.base is not None else None,
                "start": self.start, "goal_joints": self.goal_joints, "authored": self.authored,
                "max_from_authored": self.max_from_authored,
                "ik": None if ik is None else {"joints": ik.joints, "max_change": ik.max_change, "rank": ik.rank,
                                               "combinations": ik.combinations, "backend": ik.backend,
                                               "solvers": list(ik.solvers),
                                               "colliding": list(ik.colliding) if ik.colliding else None,
                                               "reason": ik.reason},
                "sources": dict(self.sources), "warnings": list(self.warnings), "seconds": self.seconds}


def goal_line(check: GoalCheck) -> str:
    """A goal check as one readable line: the base and the largest gap to the authored goal.

    The joints are in its record (`as_record`).

    Args:
        check: The answer of `request_ik`.

    Returns:
        str: E.g. "B20_J_transfer at the live base 1.234 -0.567 m 90.0 deg: goal within 0.0004 rad of the authored
            one".
    """
    if not check.ok:
        return f"{check.movement}: no goal at the live base: {check.reason}"
    x, y, _z = check.base.position
    qx, qy, qz, qw = check.base.orientation
    yaw = math.degrees(math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))
    gap = check.max_from_authored
    found = "goal found (no authored goal to compare)" if gap is None else \
        f"goal within {gap:.4f} rad of the authored one"
    return f"{check.movement} at the live base {x:.3f} {y:.3f} m {yaw:.1f} deg: {found}"


def pose_error(a: Pose, b: Pose) -> tuple[float, float]:
    """How far apart two poses are: metres, and radians of turn (a quaternion and its negative are the same turn).

    Args:
        a: One pose.
        b: The other.

    Returns:
        tuple[float, float]: Distance and angle.
    """
    distance = float(np.linalg.norm(np.subtract(a.position, b.position)))
    dot = abs(float(np.dot(a.orientation, b.orientation)))
    return distance, 2.0 * math.acos(min(1.0, dot))


def line_goal(start: Pose, line: LineSpec) -> Pose:
    """The end of a linear move: the start moved `distance` along `direction`, turned the same.

    Args:
        start: Where the flange starts, world.
        line: The movement's line for it.

    Returns:
        Pose: The goal, world.
    """
    return Pose.from_arrays(np.add(start.position, np.multiply(line.direction, line.distance)), start.orientation)


#: A live tool axis this far from the export's retreat axis is said loudly: the tool sits turned on the bar, or the
#: export's axis is wrong.
RETREAT_AXIS_WARN = math.radians(20.0)


def live_line_goal(start: Pose, distance: float) -> Pose:
    """A retreat's goal from the live tool: the start moved `distance` back along its own -z, turned the same.

    The way a tool pulls off its joint is its own axis as the encoders read it now (the old monitor since 2026-10-22),
    not the export's; the export's distance is kept.
    """
    axis = start.matrix()[:3, 2]
    return Pose.from_arrays(np.subtract(start.position, np.multiply(axis, distance)), start.orientation)


def axis_angle(start: Pose, line: LineSpec) -> float:
    """Radians between a tool's live -z and its line's direction (the export's retreat axis)."""
    axis = -start.matrix()[:3, 2]
    cos = float(np.dot(axis, np.asarray(line.direction, dtype=float)))
    return math.acos(min(1.0, max(-1.0, cos / max(np.linalg.norm(line.direction), 1e-12))))


def live_retreat_goals(movement_id: str, spec: MovementSpec, ends: Mapping[str, Pose],
                       flanges: Mapping[str, Pose]) -> tuple[dict[str, Pose], str, list[str]]:
    """Every moving flange's goal along its live -z, the log line saying so, and a warning per tool turned far.

    Args:
        movement_id: The retreat, for the lines.
        spec: Its classification (the moving arms and their lines).
        ends: Flange id -> the export's goal pose, for how far the live goal is from it (missing: not said).
        flanges: Flange id -> the live tool0 pose (FK on the mocap base).

    Returns:
        tuple: (goals by flange id, the "[retreat live] ..." line, the warnings over RETREAT_AXIS_WARN).

    Raises:
        KeyError: A moving flange without a live pose.
        ValueError: A moving flange without a line.
    """
    goals, parts, warnings = {}, [], []
    for part in spec.arms:
        if part.line is None:
            raise ValueError(f"{movement_id}: {part.flange} has no retreat line")
        start = flanges[part.flange]
        goals[part.flange] = live_line_goal(start, part.line.distance)
        turn = axis_angle(start, part.line)
        axis = np.round(-start.matrix()[:3, 2], 2) + 0.0  # + 0.0 turns -0.00 into 0.00
        text = (f"{part.side} along ({axis[0]:+.2f}, {axis[1]:+.2f}, {axis[2]:+.2f}), {math.degrees(turn):.1f} deg "
                f"from the Rhino axis")
        if part.flange in ends:
            text += f", {pose_error(goals[part.flange], ends[part.flange])[0] * 1e3:.1f} mm from the exported goal"
        parts.append(text)
        if turn > RETREAT_AXIS_WARN:
            warnings.append(f"[retreat live] {movement_id}: the {part.side} tool axis is {math.degrees(turn):.1f} deg "
                            f"from the Rhino retreat axis; check the tool before Exec.")
    distance = max((part.line.distance for part in spec.arms), default=0.0)
    line = (f"[retreat live] {movement_id}: goals = the live tool0 poses moved {distance * 1e3:.0f} mm back along "
            f"their own -z (encoders): {'; '.join(parts)}.")
    return goals, line, warnings


def aims_at_line(process: Process, movement: Movement, spec: MovementSpec) -> bool:
    """Whether a linear move leaves a contact, so its goal is the line from where the flange really is.

    Its target poses are kept for a move into a contact (a grasp), where the body sets the goal.

    Args:
        process: The process, for the tools on the moving flanges.
        movement: The movement.
        spec: Its classification.

    Returns:
        bool: True when a tool on a moving flange is on a body at the start.
    """
    for part in spec.arms:
        robot, link = split_link_id(part.flange)
        tool = process.robots[robot].tools.get(link)
        state = movement.start.tools.get(tool) if tool is not None else None
        if state is not None and state.on is not None:
            return True
    return False
