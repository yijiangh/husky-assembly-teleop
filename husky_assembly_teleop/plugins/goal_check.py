"""
Check the selected movement's goal at the live base: IK only, nothing planned or moved.

Check goal asks `motion_planner` to solve the goal of the step selected in `process` where its robot stands now
(`request_ik`, with the run's beliefs and transfer workaround), logs how far IK's goal is from the authored one, shows
it as a dark blue ghost until the selection changes, and records it in the run (`assembly_run`). It is the old
monitor's "Transfer start: Confirm manual pose (IK check)", to compare the two in the lab.

! EXPERIMENTAL and temporary (milestone 4): the assembly steps plugin's Check goal replaces it.
"""

from __future__ import annotations

import asyncio
import math
from html import escape
from typing import TYPE_CHECKING

from async_timeout import timeout
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES

from ..plugin_api.context import PluginContext
from ..plugin_api.plugin import HuskyPlugin, register
from ..shared.planning.joints import arm_side
from ..shared.planning.route_result import GoalCheck
from ..shared.planning.service import PlanRequest
from ..ui.ghost import robot_ghosts
from ..ui.style import BUSY, FAIL, NONE, OK, block, chip

if TYPE_CHECKING:
    from ..shared.assembly_run import AssemblyRunPluginApi
    from ..shared.planning.service import MotionPlannerPluginApi
    from ..shared.process import ProcessPluginApi

#: Seconds to wait for the answer; the first check of a robot loads its models first.
GOAL_TIMEOUT = 120.0


def goal_line(check: GoalCheck) -> str:
    """A check as one log line: the largest gap to the authored goal, IK's goal per arm (radians) and the base.

    Args:
        check: The answer of `request_ik`.

    Returns:
        str: E.g. "B20_J_transfer at the live base: max |goal - authored goal| = 0.0004 rad; goal (rad) left [...]
            right [...]; base 1.234 -0.567 m 90.0 deg".
    """
    if not check.ok:
        return f"{check.movement}: no goal at the live base: {check.reason}"
    arms: dict[str, list[float]] = {}
    for name, value in check.ik.joints.items():
        arm = next(name[:-len(joint) - 1] for joint in UR_JOINT_NAMES if name.endswith(f"_{joint}"))
        arms.setdefault(arm_side(arm), []).append(value)
    goal = " ".join(f"{side} [{', '.join(f'{value:+.4f}' for value in joints)}]" for side, joints in arms.items())
    x, y, _z = check.base.position
    qx, qy, qz, qw = check.base.orientation
    yaw = math.degrees(math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))
    gap = check.max_from_authored
    return (f"{check.movement} at the live base: max |goal - authored goal| = "
            f"{'-' if gap is None else f'{gap:.4f}'} rad; goal (rad) {goal}; base {x:.3f} {y:.3f} m {yaw:.1f} deg")


@register
class GoalCheckPlugin(HuskyPlugin):
    """One button: solve the selected step's goal at the live base, log it, show it, record it."""

    name = "goal_check"
    requires = ("process", "assembly_run", "motion_planner")
    experimental = True

    def __init__(self):
        """Start with no check."""
        self._task: asyncio.Task | None = None
        #: The last answer, and the (loaded process, step index) it was for; its ghost shows while that is selected.
        self._check: GoalCheck | None = None
        self._checked_at: tuple | None = None
        self._message, self._failed = "", False
        self._ghosts = {}
        self._serials: dict[str, str] = {}

    def setup(self, ctx: PluginContext) -> None:
        """Build the button, the status and a goal ghost per robot."""
        self._serials = {robot_id(config.serial): config.serial for config in ctx.config.robots}
        with ctx.view.ui() as gui:
            self._status = gui.add_html("")
            button = gui.add_button("Check goal", hint="Solve the selected step's goal by IK where its robot stands "
                                                       "now, and show it; nothing moves (the first check loads the "
                                                       "models, up to a minute)")
            # ! Keep changing text BELOW the button.
            self._details = gui.add_html("")
        button.on_click(ctx.defer("check goal", lambda: self._start(ctx)))
        self._ghosts = robot_ghosts(ctx, ctx.config.robots, "goal")

    def _start(self, ctx: PluginContext) -> None:
        """Start a check, unless one is running."""
        if self._task is not None and not self._task.done():
            ctx.log_info("Check goal ignored: a check is running")
            return
        self._task = ctx.spawn("check goal", self.check(ctx))

    async def check(self, ctx: PluginContext) -> GoalCheck | None:
        """Solve the selected step's goal at the live base, then log, show and record it.

        Returns:
            GoalCheck | None: The answer, or None without a process or an answer in time.
        """
        process: ProcessPluginApi = ctx.require("process")
        run: AssemblyRunPluginApi = ctx.require("assembly_run")
        planner: MotionPlannerPluginApi = ctx.require("motion_planner")
        selection = process.selection
        if selection is None:
            self._say("no process loaded", failed=True)
            return None
        step, view = selection.step, run.view
        # * The snapshot is taken here, on the main thread; the planner's worker makes the scene from it.
        request = PlanRequest(selection.loaded, step, ctx.scene.snapshot, ctx.config.robots, beliefs=view.beliefs,
                              transfer_as_free=view.settings.transfer_as_free, label=f"check goal {step.label}")
        self._check = None
        self._say(f"solving the goal of {step.label}…")
        try:
            async with timeout(GOAL_TIMEOUT):
                check = await planner.request_ik(request)
        except asyncio.TimeoutError:
            self._say(f"no answer within {GOAL_TIMEOUT:.0f} s", failed=True)
            return None
        self._check, self._checked_at = check, (selection.loaded, selection.index)
        line = goal_line(check)
        (ctx.log_info if check.ok else ctx.log_warn)(line)
        self._say(line, failed=not check.ok)
        run.request_record("check_goal", action=step.action.id, movement=step.movement.id, data=check.as_record())
        return check

    def draw(self, ctx: PluginContext) -> None:
        """Show the goal ghost while its step is selected, and the last answer."""
        process: ProcessPluginApi = ctx.require("process")
        selection, check, at = process.selection, self._check, self._checked_at
        shown = check is not None and check.ok and selection is not None and at is not None \
            and selection.loaded is at[0] and selection.index == at[1]
        serial = self._serials.get(check.acting) if shown else None
        for key, ghost in self._ghosts.items():
            if key == serial:
                ghost.show(check.base, check.ik.joints)  # * IK's own joints, as the old monitor's blue ghost
            else:
                ghost.hide()
        if self._task is not None and not self._task.done():
            chips = chip("checking…", BUSY)
        elif check is not None:
            chips = chip("goal found" if check.ok else "no goal", OK if check.ok else FAIL)
        else:
            chips = chip("no check yet", NONE)
        self._status.content = block(chips)
        # * One line, cut with "…"; the whole line on hover and in the log.
        self._details.content = block(
            f'<div title="{escape(self._message, quote=True)}" style="color:{FAIL if self._failed else NONE};'
            f'font:11px/1.5 monospace;white-space:nowrap;overflow:hidden;text-overflow:ellipsis">'
            f'{escape(self._message) or "—"}</div>')

    def _say(self, message: str, failed: bool = False) -> None:
        """Set the line under the button."""
        self._message, self._failed = message, failed
