"""
What the motion_planner plugin offers others: plans, IK and collision checks for a movement (`MotionPlanRequest`).

Answers come as asyncio futures. Users type `ctx.require("motion_planner")` with it:
`planner: MotionPlannerContract = ctx.require("motion_planner")`. A request carries a snapshot taken on the main
thread; the planning scene is made on the planner's worker.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Mapping, Protocol

if TYPE_CHECKING:
    from scaffolding_assembly_core.scene import Scene

    from ..config import RobotConfig
    from ..world.collision_hit import CollisionHit
    from .assembly_planning.process_steps import LoadedProcess, Step
    from .assembly_planning.route_result import GoalCheck, RouteResult


@dataclass(frozen=True)
class MotionPlanRequest:
    """One question for the motion planner, built on the main thread.

    Attributes:
        loaded: The loaded process.
        step: The movement.
        snapshot: `ctx.scene.snapshot`, taken on the main thread: the acting robot's base and joints are the start.
        robots: The configured robots, `ctx.config.robots`.
        transfer_as_free: The transfer workaround (`classify`).
        ignore_built: Plan as if the built bars were not there (`scene_for`).
        obstacles_collide: Whether the process's obstacles from Rhino count (`scene_for`); the process plugin's box.
        goal_joints: Arm joints to plan to instead of solving the goal (`plan_movement`).
        joints: Acting robot joints to check collisions at (`request_collisions`); None: as in the snapshot.
        label: Names the request in the planner's panel and log; empty: the movement's id.
    """

    loaded: LoadedProcess
    step: Step
    snapshot: Scene
    robots: tuple[RobotConfig, ...]
    transfer_as_free: bool = False
    ignore_built: bool = False
    obstacles_collide: bool = True
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
        hit: The same collision in scene ids, for the collisions plugin; None if clear.
    """

    pairs: tuple[tuple[str, str], ...] = ()
    reason: str = ""
    sources: dict[str, str] = field(default_factory=dict)
    warnings: tuple[str, ...] = ()
    seconds: float = 0.0
    hit: CollisionHit | None = None


class MotionPlannerContract(Protocol):
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

    def request_plan(self, request: MotionPlanRequest) -> asyncio.Future[RouteResult]:
        """Plan the movement from the snapshot's start (`plan_movement`)."""

    def request_ik(self, request: MotionPlanRequest) -> asyncio.Future[GoalCheck]:
        """Solve the movement's goal by IK at the live base (`check_goal`)."""

    def request_collisions(self, request: MotionPlanRequest) -> asyncio.Future[CollisionReport]:
        """Check what the acting robot collides with in the movement's planning scene."""

    def request_cancel(self) -> None:
        """Cancel every request: queued ones answer "cancelled" unrun, the running one stops at its next try."""
