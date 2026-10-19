"""
What the arm planner plugin offers others: plans for one arm to a joint goal (`ArmPlanRequest`).

Users type `ctx.require("arm_planner")` with it: `planner: ArmPlannerContract = ctx.require("arm_planner")`.
"""

from __future__ import annotations

from dataclasses import dataclass

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


#: What `ctx.require("arm_planner")` offers.
ArmPlannerContract = PlannerContract[ArmPlanRequest]
