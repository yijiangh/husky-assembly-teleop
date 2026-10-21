"""
A movement's tool steps on the real tools: a Robotiq move, and a scaffolding tool's grasp, each watched until done.

- Robotiq: refused while an earlier goal still runs (its late answer would end this wait early); a timeout or a cancel
  stops the fingers where they are.
- Scaffolding grasp: STOP (clears a left-over stall), wait until no tool reports STALLED, close (the M1 gripper screw
  tightens) until every tool reports STALLED. Every tool is stopped on the way out, whatever happened.
- A command the robot refuses by raising (soft stopped, or the robot's lease not held: `MotionRefused`) is REFUSED.
- ! Main thread, from a plugin task.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Callable, Mapping

from ...plugin_api.concurrency import WaitTimeout
from ...robot_interface.end_effectors import ScaffoldingV3
from ...robot_interface.soft_stop import MotionRefused

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext
    from ...robot_interface.end_effectors import RobotiqGripper

#: Robotiq knuckle angles (rad) and force of the old monitor's buttons and gripper steps; seconds for its answer.
ROBOTIQ_OPEN = 0.426
ROBOTIQ_CLOSE = 0.8
ROBOTIQ_EFFORT = 0.1
ROBOTIQ_TIMEOUT = 5.0
#: A Robotiq move this close to its target, radians, reached it.
REACHED_TOLERANCE = 0.02
#: Seconds for every gripper screw to stall on the bar; for the stall to clear after a STOP.
GRASP_TIMEOUT = 30.0
CLEAR_TIMEOUT = 2.0
#: A scaffolding tool status older than this, seconds, means its driver is silent.
STATUS_MAX_AGE = 1.0

#: How a tool step ended.
REACHED, STALLED, FELL_SHORT, TIMEOUT, REFUSED, DRY_RUN = "reached", "stalled", "fell_short", "timeout", "refused", \
    "dry_run"


@dataclass(frozen=True)
class ToolOutcome:
    """How a tool step ended.

    Attributes:
        status: REACHED, STALLED (a grasp done), FELL_SHORT, TIMEOUT, REFUSED (nothing sent) or DRY_RUN.
        reason: One line saying what happened.
        seconds: From the first command to the end.
        detail: What the tools reported at the end, as JSON values.
    """

    status: str
    reason: str = ""
    seconds: float = 0.0
    detail: dict = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        """bool: The tool did what was asked, or (dry run) would have been asked."""
        return self.status in (REACHED, STALLED, DRY_RUN)

    def as_record(self) -> dict:
        """The outcome as JSON values."""
        return {"status": self.status, "reason": self.reason, "seconds": self.seconds, "detail": dict(self.detail)}


async def robotiq_move(ctx: PluginContext, gripper: RobotiqGripper, position: float, *, effort: float = ROBOTIQ_EFFORT,
                       timeout_s: float = ROBOTIQ_TIMEOUT, dry_run: bool = False) -> ToolOutcome:
    """Move a Robotiq's fingers and wait for its answer; stop them on a timeout or a cancel.

    Args:
        ctx: The plugin's context.
        gripper: The gripper.
        position: Knuckle angle to move to, radians.
        effort: Force limit, a fraction of full force.
        timeout_s: Seconds to wait for the answer.
        dry_run: Run the checks and send nothing.

    Returns:
        ToolOutcome: REACHED within REACHED_TOLERANCE with a success answer, else FELL_SHORT; TIMEOUT; REFUSED;
            DRY_RUN.

    Raises:
        asyncio.CancelledError: When cancelled; the fingers were stopped first.
    """
    name = gripper.namespace
    if not gripper.server_is_ready():
        return ToolOutcome(REFUSED, f"{name}: the gripper's action server is not available")
    if gripper.state.moving:
        return ToolOutcome(REFUSED, f"{name}: an earlier gripper command still runs; wait for it, or Stop")
    if dry_run:
        ctx.log_warn(f"DRY RUN {name}: gripper move to {position:.3f} rad passed every check, NOT SENT")
        return ToolOutcome(DRY_RUN, "every check passed; nothing sent (dry run)")
    started = ctx.now()
    try:
        gripper.move(position, effort)
    except MotionRefused as refused:
        return ToolOutcome(REFUSED, f"{name}: {refused}")
    try:
        await ctx.wait_until(lambda: not gripper.state.moving, timeout_s=timeout_s, description=f"{name} to answer")
    except WaitTimeout:
        gripper.stop()
        return ToolOutcome(TIMEOUT, f"no answer from the gripper in {timeout_s:.0f} s; fingers stopped",
                           ctx.now() - started, _robotiq_detail(gripper, position))
    except asyncio.CancelledError:
        gripper.stop()
        raise
    state, seconds = gripper.state, ctx.now() - started
    detail = _robotiq_detail(gripper, position)
    if state.last_result_ok and state.position is not None and abs(state.position - position) <= REACHED_TOLERANCE:
        return ToolOutcome(REACHED, f"at {state.position:.3f} rad", seconds, detail)
    where = "unknown" if state.position is None else f"{state.position:.3f} rad"
    answer = "reported success" if state.last_result_ok else "did not report success"
    return ToolOutcome(FELL_SHORT, f"fell short: at {where} for {position:.3f} rad; the gripper {answer}", seconds,
                       detail)


async def scaffolding_grasp(ctx: PluginContext, tools: Mapping[str, ScaffoldingV3], *,
                            timeout_s: float = GRASP_TIMEOUT, dry_run: bool = False,
                            ready: Callable[[], str] = lambda: "") -> ToolOutcome:
    """Clamp a bar in scaffolding tools: STOP, wait for the stall to clear, tighten the gripper screws until stalled.

    Args:
        ctx: The plugin's context.
        tools: Arm name -> its tool.
        timeout_s: Seconds for every screw to stall.
        dry_run: Run the checks and send nothing.
        ready: Why the screws may not tighten now, "" if they may; asked again right before they tighten, after
            the wait for the stall to clear (up to CLEAR_TIMEOUT).

    Returns:
        ToolOutcome: STALLED (clamped), TIMEOUT, REFUSED (a driver silent, `ready` said no, or the robot refused
            the close: nothing tightened) or DRY_RUN.

    Raises:
        asyncio.CancelledError: When cancelled; every tool was stopped first.
    """
    now = ctx.now()
    for arm, tool in tools.items():
        updated = tool.state.last_update_time
        if updated is None or now - updated > STATUS_MAX_AGE:
            return ToolOutcome(REFUSED, f"{arm}: no tool status in the last {STATUS_MAX_AGE:.0f} s; is its driver "
                                        f"running?")
    if dry_run:
        ctx.log_warn(f"DRY RUN grasp on {', '.join(tools)}: passed every check, NOT SENT")
        return ToolOutcome(DRY_RUN, "every check passed; nothing sent (dry run)")

    def every(since: float, stalled: bool) -> bool:
        """Whether every tool reported since `since`, its gripper motor STALLED or (stalled False) not."""
        return all(tool.state.last_update_time is not None and tool.state.last_update_time > since
                   and (tool.state.gripper_motor == ScaffoldingV3.STALLED) == stalled for tool in tools.values())

    started = now
    try:
        # ! STOP first: a stalled motor refuses to start until stopped (`ScaffoldingV3._drive`).
        for tool in tools.values():
            tool.stop()
        stopped = ctx.now()
        try:
            await ctx.wait_until(lambda: every(stopped, False), timeout_s=CLEAR_TIMEOUT,
                                 description="the gripper screws' stall to clear")
        except WaitTimeout:
            return ToolOutcome(TIMEOUT, f"a gripper screw still reports STALLED {CLEAR_TIMEOUT:.0f} s after STOP; "
                                        f"nothing tightened", ctx.now() - started, _grasp_detail(tools))
        # ! Asked again here, with no wait before the close: the wait above may have lasted seconds.
        problem = ready()
        if problem:
            return ToolOutcome(REFUSED, f"{problem}; nothing tightened", ctx.now() - started, _grasp_detail(tools))
        try:
            for tool in tools.values():
                tool.close()
        except MotionRefused as refused:
            return ToolOutcome(REFUSED, f"{refused}; nothing tightened", ctx.now() - started, _grasp_detail(tools))
        closed = ctx.now()
        try:
            await ctx.wait_until(lambda: every(closed, True), timeout_s=timeout_s,
                                 description="every gripper screw to stall")
        except WaitTimeout:
            return ToolOutcome(TIMEOUT, f"not every gripper screw STALLED within {timeout_s:.0f} s: check the grip",
                               ctx.now() - started, _grasp_detail(tools))
        return ToolOutcome(STALLED, "every gripper screw STALLED: the bar is clamped", ctx.now() - started,
                           _grasp_detail(tools))
    finally:
        # ! Always: a screw motor must never be left running.
        for tool in tools.values():
            tool.stop()


def _robotiq_detail(gripper: RobotiqGripper, target: float) -> dict:
    """What the gripper reported, as JSON values."""
    state = gripper.state
    return {"target": target, "position": state.position, "result_ok": state.last_result_ok}


def _grasp_detail(tools: Mapping[str, ScaffoldingV3]) -> dict:
    """Each tool's gripper motor state, as JSON values."""
    return {arm: tool.state.gripper_motor for arm, tool in tools.items()}
