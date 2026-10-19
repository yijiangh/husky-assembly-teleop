"""What the arm planner is asked and offers: its request and its `ArmPlannerPluginApi`."""

from __future__ import annotations

from dataclasses import dataclass

from .service import PlannerPluginApi


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
ArmPlannerPluginApi = PlannerPluginApi[ArmPlanRequest]
