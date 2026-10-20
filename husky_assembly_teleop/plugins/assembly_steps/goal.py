"""
Check goal: a step's goal solved by IK at the live base, kept for the target ghost and the step's next plan.

A step is keyed (loaded process, action, primary movement index, ...); the process compares by identity (`same_step`).

! Main thread.
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING

from async_timeout import timeout

from ...shared.execution.basis import STALE_BASE
from ...shared.assembly_planning.route_result import GoalCheck, goal_line, pose_error

if TYPE_CHECKING:
    from scaffolding_assembly_core.geometry import Pose

    from ...plugin_api.context import PluginContext
    from ...shared.motion_planner_contract import MotionPlannerContract, MotionPlanRequest

#: Seconds to wait for a goal check; the first of a robot loads its models.
GOAL_TIMEOUT = 120.0


def same_step(a: tuple | None, b: tuple | None) -> bool:
    """Whether two step keys (loaded process, action, primary, ...) are the same step: the process by identity.

    ? Never `==` on the process: a frozen dataclass compares every field of the whole process.
    """
    return a is not None and b is not None and a[0] is b[0] and a[1:] == b[1:]


class StepGoal:
    """The last goal check and the step it is for; one check at a time.

    Attributes:
        task: The check running, or None.
    """

    def __init__(self):
        """No check yet."""
        self._check: GoalCheck | None = None
        self._step: tuple | None = None
        self.task: asyncio.Task | None = None

    @property
    def running(self) -> bool:
        """bool: Whether a check runs."""
        return self.task is not None and not self.task.done()

    def of(self, step: tuple | None) -> GoalCheck | None:
        """The check made for `step` (loaded, action, primary), or None."""
        return self._check if same_step(self._step, step) else None

    def joints_near(self, step: tuple | None, base: Pose) -> dict[str, float] | None:
        """The goal joints of `step`'s check, if it found one and the base stayed within 2 mm / 0.5° since; else None.

        Args:
            step: The step key (loaded, action, primary).
            base: The acting robot's base now.
        """
        check = self.of(step)
        if check is None or not check.ok or check.base is None or check.goal_joints is None:
            return None
        distance, turn = pose_error(base, check.base)
        return dict(check.goal_joints) if distance <= STALE_BASE[0] and turn <= STALE_BASE[1] else None

    async def solve(self, ctx: PluginContext, step: tuple, request: MotionPlanRequest) -> GoalCheck | None:
        """Ask the motion planner for the goal, keep the answer for `step`, and log it.

        Args:
            ctx: The plugin's context.
            step: The step key (loaded, action, primary) the check is for.
            request: The question, built on the main thread.

        Returns:
            GoalCheck | None: The answer, or None without one within GOAL_TIMEOUT (logged).
        """
        planner: MotionPlannerContract = ctx.require("motion_planner")
        try:
            async with timeout(GOAL_TIMEOUT):
                check = await planner.request_ik(request)
        except asyncio.TimeoutError:
            ctx.log_warn(f"Check goal: no answer within {GOAL_TIMEOUT:.0f} s")
            return None
        self._check, self._step = check, step
        (ctx.log_info if check.ok else ctx.log_warn)(goal_line(check))
        return check
