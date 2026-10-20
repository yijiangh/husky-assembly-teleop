"""
The arm planner's planning worlds (`assembly_planning.mirrors.MirrorPool`) and its search (`assembly_planning.arm`).

! Worker thread only: sync, search and close on the worker.
"""

from __future__ import annotations

import threading
from typing import Sequence

from ...shared.assembly_planning.arm import ArmPath, _extend
from ...shared.assembly_planning.arm import plan_arm as search_arm
from ...shared.assembly_planning.joints import arm_joint_names
from ...shared.assembly_planning.mirrors import MirrorPool
from ...shared.planner.planner_search import PlanResult
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror

__all__ = ["ArmPath", "ArmPlanningWorld", "_extend", "arm_joint_names", "plan_arm"]

#: The arm planner's planning worlds: one compas_fab mirror per robot (`MirrorPool`).
ArmPlanningWorld = MirrorPool


def plan_arm(world: ArmPlanningWorld, mirror: CompasFabMirror, arm_name: str, goal: Sequence[float],
             abort: threading.Event) -> PlanResult[ArmPath]:
    """Plan a collision-free joint path for one arm (`assembly_planning.arm.plan_arm`), named with the world's labels.

    Args:
        world: The planning world, for labels.
        mirror: The robot's mirror, already synced.
        arm_name: Which arm, e.g. "ur_arm".
        goal: Six joint values, radians, UR driver order.
        abort: Set from the main thread to end the search early.

    Returns:
        PlanResult[ArmPath]: The path, or why there is none.
    """
    return search_arm(mirror, arm_name, goal, abort, label=world.label)
