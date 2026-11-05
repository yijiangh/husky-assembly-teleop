"""
What the arm planner plugin offers others: plans for one arm to a joint goal, and checks of a given joint path.

Users type `ctx.require("arm_planner")` with it: `planner: ArmPlannerContract = ctx.require("arm_planner")`.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Protocol, Sequence

from scaffolding_assembly_core.geometry import Pose

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
    """What `ctx.require("arm_planner")` offers: the planner contract, re-timing, and path checks.

    `commit(..., clearance=True)` runs the path under an ARM_MOVE clearance (`stop_distance`: 5 mm, or half the path's
    margin when its start or target is closer than 1 cm); False (Clearance unticked): nothing soft stops it.
    """

    def set_duration(self, plan_id: int, seconds: float) -> None:
        """Re-time the latest plan to take `seconds`, without replanning: the same path, its fastest timing stretched.

        Clamped to its fastest timing up to MAX_SLOWDOWN times that; only a ready plan changes. Play and Commit follow
        it. The plan is replaced, keeping its id.
        """

    def solve_goal(self, serial: str, arm: str, tool0: Pose,
                   reference: Sequence[float] | None = None) -> asyncio.Future[tuple[tuple[float, ...] | None, str]]:
        """Joints that put `arm`'s tool0 at world pose `tool0`, by exact IK at the robot's live base, collision-free.

        The branch nearest `reference` (the arm's joints now if None) wins. Runs after any search in progress.

        Returns:
            asyncio.Future: (six joint values in the UR driver's order, or None; one line saying how or why not).
        """

    def check_path(self, serial: str, arm: str,
                   waypoints: Sequence[Sequence[float]], margin: float = 0.0) -> asyncio.Future[CollisionHit | None]:
        """Check a joint path for one arm against the robot itself, the other robots and every scene body.

        Each waypoint and the straight joint move between two are checked, in the world as it is now; the robot's
        other joints stay where they are. Runs after any search in progress; the first check loads the models.

        Args:
            serial: The robot.
            arm: Which of its arms, e.g. "left_ur_arm".
            waypoints: (n, 6) joint values, radians, UR driver order.
            margin: Closer than this to anything counts as a hit, metres (0: contact only); against the robot
                itself at most SELF_DISTANCE, as a clearance checks it.

        Returns:
            asyncio.Future[CollisionHit | None]: Resolves with the first configuration that collides and every pair
                that blocks it, or None if the whole path is clear. Raises ValueError if the robot can't be checked
                (absent), RuntimeError if the check itself failed.
        """
