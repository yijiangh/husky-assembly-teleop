"""
A plan, and the generic contract every planner plugin offers: request a plan, cancel, clear, commit, read the latest.

A plan goes planning -> ready -> stale or sent, or planning -> failed or cancelled; every change replaces it.
Each planner's own contract file in `shared/` fixes the request type, e.g. `arm_planner_contract.ArmPlannerContract`.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Generic, Protocol, TypeVar, runtime_checkable

from .timed_path import TimedPath

#: Every plan state.
STATES = ("planning", "ready", "stale", "sent", "failed", "cancelled")


class PlanRequest(Protocol):
    """What every planner's request has; each planner defines its own frozen request type in its contract file."""

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
