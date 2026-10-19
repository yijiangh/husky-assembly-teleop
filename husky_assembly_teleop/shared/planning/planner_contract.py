"""
What planner plugins offer others: request a plan, cancel, clear, commit, and read the latest plan.

A plan goes planning -> ready -> stale or sent, or planning -> failed or cancelled; every change replaces it.
Each planner has its own request type; its users type `ctx.require("<planner>")` with its contract, e.g.
`planner: ArmPlannerContract = ctx.require("arm_planner")`.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Any, Generic, Protocol, TypeVar, runtime_checkable

from .timed_path import TimedPath

#: Every plan state.
STATES = ("planning", "ready", "stale", "sent", "failed", "cancelled")


class PlanRequest(Protocol):
    """What every planner's request has; each planner defines its own frozen request type in `shared/planning/`."""

    @property
    def serial(self) -> str:
        """str: The robot to plan for."""


R = TypeVar("R", bound=PlanRequest)


@dataclass(frozen=True)
class Plan(Generic[R]):
    """One plan as it stands; replaced, never edited, on every change.

    Attributes:
        id: Unique per planner, counting from 1; `commit` takes it.
        request: What was asked.
        state: One of STATES.
        path: The path once found, else None.
        message: What happens or happened: progress, what was found, why it failed or went stale.
        warning: Whether `message` is a warning, e.g. planned from a guessed robot pose.
    """

    id: int
    request: R
    state: str
    path: TimedPath | None = None
    message: str = ""
    warning: bool = False


@runtime_checkable
class PlannerContract(Protocol[R]):
    """What a planner plugin offers others: request, cancel, clear and commit plans, and read the latest one."""

    @property
    def plan(self) -> Plan[R] | None:
        """Plan | None: The latest plan, replaced on every change; None before the first request or after `clear`."""

    def request_plan(self, request: R) -> asyncio.Future[Plan[R]]:
        """Plan for `request`, ending a search in progress; resolves once the plan is ready, failed or cancelled."""

    def cancel(self) -> None:
        """End a search in progress; its plan becomes "cancelled". Nothing else changes."""

    def clear(self) -> None:
        """End a search in progress and forget the plan. ! A sent plan's motion goes on: stop it on the robot."""

    def commit(self, plan_id: int) -> asyncio.Future[tuple[bool, str]]:
        """Send plan `plan_id` to the robot if it is still the latest and ready; resolves with (sent, message)."""


@dataclass(frozen=True)
class ArmPlanRequest:
    """Plan one arm from where it is now to a joint goal.

    Attributes:
        serial: The robot.
        arm: Which of its arms, e.g. "left_ur_arm".
        goal: The six joint values to reach, radians, in the UR driver's order.
    """

    serial: str
    arm: str
    goal: tuple[float, ...]


#: What `ctx.require("arm_planner")` offers.
ArmPlannerContract = PlannerContract[ArmPlanRequest]


@dataclass(frozen=True)
class BasePlanRequest:
    """A base path for robot `serial` from where it is now to `goal`, and whether to record the attempt.

    Attributes:
        serial: The robot to plan for.
        goal: (x, y, yaw) to end at, metres and radians, world frame.
        algorithm: The algorithm's label, as in the panel's Algorithm dropdown; "" for the default.
        settings: (name, value) pairs that replace the algorithm's default settings.
        debug: Whether the search keeps a record of how it worked, for the debug view.
        task: The experiment task chosen, by name; the attempt counts for it only from its start to its target.
        record: Whether to save the attempt, and the run once committed, to the Drive.
        experiment: The experiment the recordings are saved under.
    """

    serial: str
    goal: tuple[float, float, float]
    algorithm: str = ""
    settings: tuple[tuple[str, Any], ...] = ()
    debug: bool = False
    task: str | None = None
    record: bool = False
    experiment: str = "default"


#: What `ctx.require("base_planner")` offers.
BasePlannerContract = PlannerContract[BasePlanRequest]
