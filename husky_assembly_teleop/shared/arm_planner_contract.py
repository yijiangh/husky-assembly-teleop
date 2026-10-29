"""
What the arm planner plugin offers others: plans for one arm to a joint goal, and checks of a given joint path.

Users type `ctx.require("arm_planner")` with it: `planner: ArmPlannerContract = ctx.require("arm_planner")`.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Protocol, Sequence

from ..world.collision_hit import CollisionHit
from .planner.plan import PlannerContract

#: Slowest timing a plan may be stretched to (`set_duration`): this many times its fastest.
MAX_SLOWDOWN = 5.0


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
    """What `ctx.require("arm_planner")` offers: the planner contract, re-timing, and path checks."""

    def set_duration(self, plan_id: int, seconds: float) -> None:
        """Re-time the latest plan to take `seconds`, without replanning: the same path, its fastest timing stretched.

        Clamped to its fastest timing up to MAX_SLOWDOWN times that; only a ready plan changes. Play and Commit follow
        it. The plan is replaced, keeping its id.
        """

    def check_path(self, serial: str, arm: str,
                   waypoints: Sequence[Sequence[float]]) -> asyncio.Future[CollisionHit | None]:
        """Check a joint path for one arm against the robot itself, the other robots and every scene body.

        Each waypoint and the straight joint move between two are checked, in the world as it is now; the robot's
        other joints stay where they are. Runs after any search in progress; the first check loads the models.

        Args:
            serial: The robot.
            arm: Which of its arms, e.g. "left_ur_arm".
            waypoints: (n, 6) joint values, radians, UR driver order.

        Returns:
            asyncio.Future[CollisionHit | None]: Resolves with the first configuration that collides and every pair
                that blocks it, or None if the whole path is clear. Raises ValueError if the robot can't be checked
                (absent), RuntimeError if the check itself failed.
        """
