"""
The motion planner's work, run on its one worker thread, which owns the mirrors: plan, IK and collision checks.

- ! `run_plan`, `run_ik` and `run_collisions` run only on the worker, after the request's checks (`_acting`).
- They never raise: any error, a cancel or a robot that can't be planned for gives a failed answer with the reason.
"""

from __future__ import annotations

import threading
import time
from typing import Callable

from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.process.solutions import MovementResult

from ...shared.assembly_planning.collision import collision_hit, in_process_terms
from ...shared.assembly_planning.mirrors import MirrorPool
from ...shared.assembly_planning.route_result import GoalCheck, RouteResult
from ...shared.assembly_planning.scene_for import scene_for
from ...shared.assembly_planning.tamp_route import check_goal, plan_movement
from ...shared.motion_planner_contract import CollisionReport, MotionPlanRequest
from ...shared.process_scene.scene_bodies import live_ids

Log = Callable[[str], None]


def run_plan(pool: MirrorPool, request: MotionPlanRequest, abort: threading.Event, log: Log) -> RouteResult:
    """Plan a movement on the acting robot's mirror. Worker thread.

    Args:
        pool: The worker's mirrors.
        request: The question.
        abort: Set to stop between tries.
        log: Gets the planners' fallback lines.

    Returns:
        RouteResult: The plan, or a failed result with the reason (routine None if it failed before classifying).
    """
    started = time.monotonic()
    try:
        serial = _acting(request, abort)
        return plan_movement(pool.mirror(serial), request.loaded, request.step, request.snapshot, request.robots,
                             transfer_as_free=request.transfer_as_free, beliefs=request.beliefs,
                             ignore_built=request.ignore_built, obstacles_collide=request.obstacles_collide,
                             goal_joints=request.goal_joints, abort=abort, log=log)
    except Exception as error:
        return RouteResult(MovementResult("failed", _reason(error)), None, seconds=time.monotonic() - started)


def run_ik(pool: MirrorPool, request: MotionPlanRequest, abort: threading.Event, log: Log) -> GoalCheck:
    """Solve a movement's goal by IK at the live base, on the acting robot's mirror. Worker thread.

    Args:
        pool: The worker's mirrors.
        request: The question.
        abort: Set to answer "cancelled" before starting.
        log: Gets the IK fallback line.

    Returns:
        GoalCheck: The goal, or a failed check with the reason.
    """
    started = time.monotonic()
    try:
        serial = _acting(request, abort)
        return check_goal(pool.mirror(serial), request.loaded, request.step, request.snapshot, request.robots,
                          transfer_as_free=request.transfer_as_free, beliefs=request.beliefs,
                          ignore_built=request.ignore_built, obstacles_collide=request.obstacles_collide, log=log)
    except Exception as error:
        return GoalCheck(request.step.movement.id, None, False, _reason(error), seconds=time.monotonic() - started)


def run_collisions(pool: MirrorPool, request: MotionPlanRequest, abort: threading.Event, log: Log) -> CollisionReport:
    """Check what the acting robot collides with in the movement's planning scene (end contacts allowed). Worker thread.

    Args:
        pool: The worker's mirrors.
        request: The question; `joints` changes the acting robot's joints for the check.
        abort: Set to answer "cancelled" before starting.
        log: Unused; the same signature as the other work.

    Returns:
        CollisionReport: Every colliding pair and its hit, or why the check did not run.
    """
    started = time.monotonic()
    try:
        mirror = pool.mirror(_acting(request, abort))
        planning = scene_for(request.loaded, request.step, request.snapshot, request.robots, beliefs=request.beliefs,
                             ignore_built=request.ignore_built, obstacles_collide=request.obstacles_collide,
                             end_contacts=True)
        mirror.sync(planning.scene)
        joints = dict(request.joints) if request.joints is not None else None
        hit = collision_hit(mirror, planning.scene, "start" if joints is None else "the given joints", joints)
        pairs = hit.pairs if hit is not None else ()
        return CollisionReport(tuple(in_process_terms(pair, planning.process_ids) for pair in pairs), "",
                               dict(planning.sources), planning.warnings, time.monotonic() - started, hit)
    except Exception as error:
        return CollisionReport(reason=_reason(error), seconds=time.monotonic() - started)


class _Refused(Exception):
    """A request the worker does not start; the message says why."""


def _acting(request: MotionPlanRequest, abort: threading.Event) -> str:
    """The serial of the robot the request plans for, once it may start.

    Args:
        request: The question; its movement's action names the acting robot.
        abort: Set to refuse with "cancelled" before anything else is looked at.

    Returns:
        str: The acting robot's serial, e.g. "a200-0806".

    Raises:
        _Refused: If cancelled, no configured robot acts in the movement, or the acting robot can't be planned for.
    """
    if abort.is_set():
        raise _Refused("cancelled")
    ids, _ = live_ids(request.loaded.process, request.robots, "process/")
    action = request.step.action
    live = ids.get(action.robot) if action.robot else None
    config = next((config for config in request.robots if robot_id(config.serial) == live), None)
    if config is None:
        raise _Refused("no configured robot acts")
    robot = request.snapshot.robots.get(live)
    problems = robot.acting_problems() if robot is not None else ["it is not in the snapshot"]
    if problems:
        raise _Refused(f"{config.serial} can't be planned for: {'; '.join(problems)}")
    return config.serial


def _reason(error: Exception) -> str:
    """A failure's reason: a refusal's own text, anything else with its type."""
    return str(error) if isinstance(error, _Refused) else f"the planner raised {error!r}"
