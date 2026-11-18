"""
What a step's motion plan is asked from, and how its answer is timed: one copy for every plugin that plans steps.

`live_goal_poses` gives the goals a step's primary movement is planned to from the live robot instead of its
authored ones: an insert from where the transfer's servo left the flanges, a retreat along each tool's live
axis. `timed_route` times a found route for the arms.

! Main thread: the snapshot is taken by the caller there.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Callable

from scaffolding_assembly_core.robot import RobotObject

from ..execution.retime import RetimeError, retime
from ..execution.servo import carried_insert_goals, flange_poses, flange_side
from ..planner.timed_path import TimedPath
from .movement import Routine, classify
from .route_result import live_retreat_goals, pose_error

if TYPE_CHECKING:
    from scaffolding_assembly_core.geometry import Pose

    from ..movement_exec_contract import Carried
    from .process_steps import LoadedProcess, Step
    from .route_result import RouteResult


def live_goal_poses(loaded: LoadedProcess, action: str, step: Step, routine: Routine | None, live: RobotObject, *,
                    transfer_as_free: bool, carried: Carried | None, retreat_live_axis: bool,
                    log_info: Callable[[str], None], log_warn: Callable[[str], None]) -> dict[str, Pose] | None:
    """The flange goals a step's primary movement is planned to from the live robot, or None for the authored ones.

    Args:
        loaded: The loaded process.
        action: The action's id.
        step: The primary movement's step (`loaded.steps`).
        routine: The primary movement's routine.
        live: The acting robot as measured now.
        transfer_as_free: The process plugin's transfer workaround.
        carried: What the last transfer's servo left (`MovementExecContract.carried`), or None.
        retreat_live_axis: The run's "Retreat live axis": a retreat backs each tool out along its own live -z.
        log_info, log_warn: Where the carried correction and the retreat's axes are said.

    Returns:
        dict[str, Pose] | None: Process flange id -> world goal (`MotionPlanRequest.goal_poses`), or None.
    """
    if (routine is Routine.INSERT and carried is not None and carried.loaded is loaded
            and (carried.action, carried.insert) == (action, step.movement_index)):
        # * The transfer's servo left the flanges off the authored start: the insert goes from there.
        goals = carried_insert_goals(loaded, action, carried.insert, live)
        flange, ends = next(iter(goals)), step.movement.target.robots[step.action.robot].link_poses
        distance, turn = pose_error(goals[flange], ends[flange])
        log_info(f"[insert plan] servo correction carried: {step.label} {flange_side(flange)} goal "
                 f"{distance * 1e3:.2f} mm / {math.degrees(turn):.3f}° from the authored end")
        return goals
    if routine is Routine.RETREAT_LINEAR and retreat_live_axis:
        # * The retreat backs each tool out along its own live -z.
        spec = classify(loaded.process, step.action, step.movement, transfer_as_free)
        own = step.movement.target.robots.get(step.action.robot) if step.movement.target else None
        goals, line, warnings = live_retreat_goals(step.movement.id, spec, own.link_poses if own is not None else {},
                                                   flange_poses(live, [part.flange for part in spec.arms]))
        log_info(line)
        for warning in warnings:
            log_warn(warning)
        return goals
    return None


def timed_route(route: RouteResult, seconds: float) -> tuple[TimedPath, float] | str:
    """A route timed for the arms to take `seconds` (`retime`), and the shortest time they allow; or why it can't be."""
    try:
        timed = retime(route.result.trajectory.positions, seconds)
    except RetimeError as error:
        rows = route.result.trajectory.positions
        if rows[0] == rows[-1]:
            return "the arms are already at the goal: nothing to move"
        return f"planned, but it can't be timed for the arms: {error}"
    return TimedPath(timed.times, timed.positions), timed.minimum
