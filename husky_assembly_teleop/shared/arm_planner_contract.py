"""
What the arm planner plugin offers others: plans for one arm to a joint goal, and checks of a given joint path.

Users type `ctx.require("arm_planner")` with it: `planner: ArmPlannerContract = ctx.require("arm_planner")`.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Protocol, Sequence

from .planner.plan import PlannerContract


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


class ArmPlannerContract(PlannerContract[ArmPlanRequest], Protocol):
    """What `ctx.require("arm_planner")` offers: the planner contract, and path checks."""

    def check_path(self, serial: str, arm: str, waypoints: Sequence[Sequence[float]]) -> asyncio.Future[str | None]:
        """Check a joint path for one arm against the robot itself, the other robots and every scene body.

        Each waypoint and the straight joint move between two are checked, in the world as it is now; the robot's
        other joints stay where they are. Runs after any search in progress; the first check loads the models.

        Args:
            serial: The robot.
            arm: Which of its arms, e.g. "left_ur_arm".
            waypoints: (n, 6) joint values, radians, UR driver order.

        Returns:
            asyncio.Future[str | None]: Resolves with the first collision, or None if the whole path is clear.
        """
