"""
What planning a movement gave (`RouteResult`), what checking its goal gave (`GoalCheck`), and pose helpers for goals.

`goal_line` puts a goal check into one log line.

Pure: any thread.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np

from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import split_link_id
from scaffolding_assembly_core.process import LineSpec, Movement, Process
from scaffolding_assembly_core.process.solutions import MovementResult
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES

from .collision import SweptReport
from .ik import IkResult
from .joints import arm_side
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
    """A goal check as one log line: the largest gap to the authored goal, IK's goal per arm (radians) and the base.

    Args:
        check: The answer of `request_ik`.

    Returns:
        str: E.g. "B20_J_transfer at the live base: max |goal - authored goal| = 0.0004 rad; goal (rad) left [...]
            right [...]; base 1.234 -0.567 m 90.0 deg".
    """
    if not check.ok:
        return f"{check.movement}: no goal at the live base: {check.reason}"
    arms: dict[str, list[float]] = {}
    for name, value in check.ik.joints.items():
        arm = next(name[:-len(joint) - 1] for joint in UR_JOINT_NAMES if name.endswith(f"_{joint}"))
        arms.setdefault(arm_side(arm), []).append(value)
    goal = " ".join(f"{side} [{', '.join(f'{value:+.4f}' for value in joints)}]" for side, joints in arms.items())
    x, y, _z = check.base.position
    qx, qy, qz, qw = check.base.orientation
    yaw = math.degrees(math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))
    gap = check.max_from_authored
    return (f"{check.movement} at the live base: max |goal - authored goal| = "
            f"{'-' if gap is None else f'{gap:.4f}'} rad; goal (rad) {goal}; base {x:.3f} {y:.3f} m {yaw:.1f} deg")


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
