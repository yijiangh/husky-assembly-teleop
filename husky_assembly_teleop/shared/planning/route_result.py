"""
What planning one process movement gave (`RouteResult`), and the pure pose helpers its goals are made with.

Pure: any thread.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import split_link_id
from scaffolding_assembly_core.process import LineSpec, Movement, Process
from scaffolding_assembly_core.process.solutions import MovementResult

from .collision import SweptReport
from .ik import IkResult
from .movement import MovementSpec, Routine


@dataclass(frozen=True)
class RouteResult:
    """One movement planned (or not), with what the planner and the checks found.

    Attributes:
        result: The core's result, in process ids: status "solved", "failed" or "not_planned".
        routine: How the movement was planned.
        planner: Which planner ran, e.g. "tamp.plan_free_dual_arm" or "single_arm.plan_linear/ssik".
        attempts: How many tries ran.
        seconds: Time taken: scenes, syncs, IK, searches and checks.
        waypoints: Rows of the path; 0 without one.
        end_error: Process flange id -> how far the path's end puts it from its goal pose: metres, radians.
        joint_error: The largest gap between the path's end and a joint goal, radians; None for a pose goal.
        goal_joints: The arm joints planned to, where the goal was in joints (given, or solved by IK).
        colliding: Colliding pairs found on the way, in process ids where mapped.
        swept: The swept check of the path, where one ran.
        ik: The goal's IK, where one ran.
        warnings: Robots left out of the scene, a colliding start of a linear move, a mismatched grasp.
    """

    result: MovementResult
    routine: Routine
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

    @property
    def ok(self) -> bool:
        """bool: Whether the movement was solved."""
        return self.result.status == "solved"


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
