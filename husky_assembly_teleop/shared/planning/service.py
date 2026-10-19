"""
The motion planner service: what other plugins ask it (`PlanRequest`), what it answers, and the work its worker runs.

A request carries a snapshot taken on the main thread; the planning scene is made on the worker. Answers come back as
asyncio futures (`MotionPlannerPluginApi`).

- ! `run_plan`, `run_ik` and `run_collisions` run on the planner's one worker thread, which owns the mirrors.
- They never raise: any error, a cancel or a robot that can't be planned for gives a failed answer with the reason.
"""

from __future__ import annotations

import asyncio
import threading
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Callable, Mapping, Protocol

from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.process.solutions import MovementResult
from scaffolding_assembly_core.scene import Scene

from ..process import LoadedProcess, Step, live_ids
from .collision import collisions, in_process_terms
from .mirrors import MirrorPool
from .route_result import GoalCheck, RouteResult
from .scene_for import RobotBelief, scene_for
from .tamp_route import check_goal, plan_movement

if TYPE_CHECKING:
    from ...config import RobotConfig

Log = Callable[[str], None]


@dataclass(frozen=True)
class PlanRequest:
    """One question for the motion planner, built on the main thread.

    Attributes:
        loaded: The loaded process.
        step: The movement.
        snapshot: `ctx.scene.snapshot`, taken on the main thread: the acting robot's base and joints are the start.
        robots: The configured robots, `ctx.config.robots`.
        beliefs: Live robot id -> where it was last recorded, for robots not measured (`scene_for`).
        transfer_as_free: The transfer workaround (`classify`).
        ignore_built: Plan as if the built bars were not there (`scene_for`).
        goal_joints: Arm joints to plan to instead of solving the goal (`plan_movement`).
        joints: Acting robot joints to check collisions at (`request_collisions`); None: as in the snapshot.
        label: Names the request in the planner's panel and log; empty: the movement's id.
    """

    loaded: LoadedProcess
    step: Step
    snapshot: Scene
    robots: tuple[RobotConfig, ...]
    beliefs: Mapping[str, RobotBelief] = field(default_factory=dict)
    transfer_as_free: bool = False
    ignore_built: bool = False
    goal_joints: Mapping[str, float] | None = None
    joints: Mapping[str, float] | None = None
    label: str = ""


@dataclass(frozen=True)
class CollisionReport:
    """What the acting robot collides with at the start of a movement (or at the request's joints).

    Attributes:
        pairs: Colliding pairs, in process ids where mapped; empty if clear.
        reason: Why the check did not run, or ""; then `pairs` says nothing.
        sources: Live id of each other robot -> where its base and joints came from (`scene_for`).
        warnings: Robots and bodies left out of the scene, and why.
        seconds: Time taken.
    """

    pairs: tuple[tuple[str, str], ...] = ()
    reason: str = ""
    sources: dict[str, str] = field(default_factory=dict)
    warnings: tuple[str, ...] = ()
    seconds: float = 0.0


class MotionPlannerPluginApi(Protocol):
    """What the motion_planner plugin offers: type `ctx.require("motion_planner")` as this.

    Requests run one at a time, in order. ! Call them on the main thread, and await the future inside
    `async_timeout.timeout(...)`: a timeout or a cancel stops the search between its tries.
    """

    @property
    def busy(self) -> bool:
        """bool: Whether a request is running or queued."""

    @property
    def running(self) -> str:
        """str: The label of the request running now, or ""."""

    def request_plan(self, request: PlanRequest) -> asyncio.Future[RouteResult]:
        """Plan the movement from the snapshot's start (`plan_movement`)."""

    def request_ik(self, request: PlanRequest) -> asyncio.Future[GoalCheck]:
        """Solve the movement's goal by IK at the live base (`check_goal`)."""

    def request_collisions(self, request: PlanRequest) -> asyncio.Future[CollisionReport]:
        """Check what the acting robot collides with in the movement's planning scene."""

    def request_cancel(self) -> None:
        """Cancel every request: queued ones answer "cancelled" unrun, the running one stops at its next try."""


def run_plan(pool: MirrorPool, request: PlanRequest, abort: threading.Event, log: Log) -> RouteResult:
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
                             ignore_built=request.ignore_built, goal_joints=request.goal_joints, abort=abort, log=log)
    except Exception as error:
        return RouteResult(MovementResult("failed", _reason(error)), None, seconds=time.monotonic() - started)


def run_ik(pool: MirrorPool, request: PlanRequest, abort: threading.Event, log: Log) -> GoalCheck:
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
                          ignore_built=request.ignore_built, log=log)
    except Exception as error:
        return GoalCheck(request.step.movement.id, None, False, _reason(error), seconds=time.monotonic() - started)


def run_collisions(pool: MirrorPool, request: PlanRequest, abort: threading.Event, log: Log) -> CollisionReport:
    """Check what the acting robot collides with in the movement's planning scene (end contacts allowed). Worker thread.

    Args:
        pool: The worker's mirrors.
        request: The question; `joints` changes the acting robot's joints for the check.
        abort: Set to answer "cancelled" before starting.
        log: Unused; the same signature as the other work.

    Returns:
        CollisionReport: Every colliding pair, or why the check did not run.
    """
    started = time.monotonic()
    try:
        mirror = pool.mirror(_acting(request, abort))
        planning = scene_for(request.loaded, request.step, request.snapshot, request.robots, beliefs=request.beliefs,
                             ignore_built=request.ignore_built, end_contacts=True)
        mirror.sync(planning.scene)
        pairs = collisions(mirror, dict(request.joints) if request.joints is not None else None, full_report=True)
        return CollisionReport(tuple(in_process_terms(pair, planning.process_ids) for pair in pairs), "",
                               dict(planning.sources), planning.warnings, time.monotonic() - started)
    except Exception as error:
        return CollisionReport(reason=_reason(error), seconds=time.monotonic() - started)


class _Refused(Exception):
    """A request the worker does not start; the message says why."""


def _acting(request: PlanRequest, abort: threading.Event) -> str:
    """The serial of the robot the request plans for, once it may start.

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
