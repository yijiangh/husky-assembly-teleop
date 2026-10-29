"""
A movement's tool steps on the real tools: a Robotiq move, and a scaffolding tool's grasp, each watched until done.

- Robotiq: refused while an earlier goal still runs (its late answer would end this wait early); a timeout or a cancel
  stops the fingers where they are.
- Scaffolding tools: `stop_and_clear` before a screw motor starts (a stalled one refuses to start), a `StallGate` to
  tell when the screws have stalled, `stop_all` on the way out. The grasp clamps a bar with them.
- A command the robot refuses by raising (soft stopped, or no lease: `MotionRefused`) is REFUSED.
- ! Main thread, from a plugin task.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field, replace
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
#: A scaffolding tool's motors, for log lines.
MOTOR_NAMES = {ScaffoldingV3.GRIPPER_MOTOR: "gripper screw", ScaffoldingV3.JOINT_MOTOR: "joint screw"}

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


# * Scaffolding tools

class StallGate:
    """Tells when every watched tool's motor has stalled, for a part that ends on the stall (W:4181-4226).

    - A tool counts once its motor was seen not STALLED (in a fresh status when the gate was made, or since), and
      reports STALLED now: a stall left over from before never ends a part.
    - A status older than STATUS_MAX_AGE never counts; it is logged once per tool.
    - ! Make the gate when the motors start: `stall_times` count from then.

    Attributes:
        tools: Arm name -> its tool, the ones watched.
        motor: ScaffoldingV3.GRIPPER_MOTOR or JOINT_MOTOR.
        stall_times: Arm name -> seconds from the gate's making to its tool's first counted stall.
    """

    def __init__(self, ctx: PluginContext, tools: Mapping[str, ScaffoldingV3], motor: int):
        """Watch `motor` on `tools` from now; `ctx` gives the time and takes the log lines."""
        self._ctx, self.tools, self.motor = ctx, dict(tools), motor
        self.stall_times: dict[str, float] = {}
        self._made = ctx.now()
        #: The arms whose motor was seen not STALLED; those whose silence was logged.
        # * Seeded now, so a motor that stalls before the first poll still counts.
        self._ran = {arm for arm, tool in self.tools.items() if _fresh(tool, self._made)
                     and _motor_state(tool, motor) != ScaffoldingV3.STALLED}
        self._silent: set[str] = set()

    @property
    def text(self) -> str:
        """str: The watched motors for log lines, e.g. "left_ur_arm joint screw"."""
        plural = "s" if len(self.tools) > 1 else ""
        return f"{' + '.join(self.tools)} {MOTOR_NAMES[self.motor]}{plural}"

    def all_stalled(self) -> bool:
        """Read every watched tool's status once; whether each one counts as stalled (never with no tool)."""
        now, stalled = self._ctx.now(), 0
        for arm, tool in self.tools.items():
            if not _fresh(tool, now):
                if arm not in self._silent:
                    self._silent.add(arm)
                    self._ctx.log_warn(f"{arm}: no tool status in the last {STATUS_MAX_AGE:.0f} s; its "
                                       f"{MOTOR_NAMES[self.motor]} does not count as stalled until it reports again")
            elif _motor_state(tool, self.motor) != ScaffoldingV3.STALLED:
                self._ran.add(arm)
            elif arm in self._ran:
                stalled += 1
                self.stall_times.setdefault(arm, now - self._made)
        return bool(self.tools) and stalled == len(self.tools)


def status_problem(ctx: PluginContext, tools: Mapping[str, ScaffoldingV3]) -> str:
    """Why the tools' statuses can't be trusted now; "" when each one reported within STATUS_MAX_AGE."""
    now = ctx.now()
    for arm, tool in tools.items():
        if not _fresh(tool, now):
            return f"{arm}: no tool status in the last {STATUS_MAX_AGE:.0f} s; is its driver running?"
    return ""


async def stop_and_clear(ctx: PluginContext, tools: Mapping[str, ScaffoldingV3], motor: int) -> ToolOutcome | None:
    """STOP every tool, then wait until each reports `motor` not STALLED after the STOP, so that it may start.

    Args:
        ctx: The plugin's context.
        tools: Arm name -> its tool.
        motor: ScaffoldingV3.GRIPPER_MOTOR or JOINT_MOTOR, the one about to start.

    Returns:
        ToolOutcome | None: None once every stall cleared; TIMEOUT when one did not within CLEAR_TIMEOUT: start
            nothing then.
    """
    stop_all(tools)
    stopped = ctx.now()
    try:
        await ctx.wait_until(lambda: all(_reports(tool, motor, stopped, False) for tool in tools.values()),
                             timeout_s=CLEAR_TIMEOUT, description=f"the {MOTOR_NAMES[motor]}s' stall to clear")
    except WaitTimeout:
        stuck = ", ".join(arm for arm, tool in tools.items() if not _reports(tool, motor, stopped, False))
        return ToolOutcome(TIMEOUT, f"{stuck}: the {MOTOR_NAMES[motor]} still reports STALLED, or nothing, "
                                    f"{CLEAR_TIMEOUT:.0f} s after STOP", ctx.now() - stopped,
                           _motor_detail(tools, motor))
    return None


def stop_all(tools: Mapping[str, ScaffoldingV3]) -> None:
    """STOP every tool: both motors halt and their stalls clear.

    ! Call it in the `finally` of everything that starts a screw motor: one must never be left running.
    """
    for tool in tools.values():
        tool.stop()


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
    problem = status_problem(ctx, tools)
    if problem:
        return ToolOutcome(REFUSED, problem)
    if dry_run:
        ctx.log_warn(f"DRY RUN grasp on {', '.join(tools)}: passed every check, NOT SENT")
        return ToolOutcome(DRY_RUN, "every check passed; nothing sent (dry run)")
    gripper = ScaffoldingV3.GRIPPER_MOTOR
    started = ctx.now()
    try:
        not_cleared = await stop_and_clear(ctx, tools, gripper)
        if not_cleared is not None:
            return replace(not_cleared, reason=f"{not_cleared.reason}; nothing tightened")
        # ? Asked again here, with no wait before the close: the wait above may have lasted seconds.
        problem = ready()
        if problem:
            return ToolOutcome(REFUSED, f"{problem}; nothing tightened", ctx.now() - started,
                               _motor_detail(tools, gripper))
        try:
            for tool in tools.values():
                tool.close()
        except MotionRefused as refused:
            return ToolOutcome(REFUSED, f"{refused}; nothing tightened", ctx.now() - started,
                               _motor_detail(tools, gripper))
        closed = ctx.now()
        try:
            await ctx.wait_until(lambda: all(_reports(tool, gripper, closed, True) for tool in tools.values()),
                                 timeout_s=timeout_s, description="every gripper screw to stall")
        except WaitTimeout:
            return ToolOutcome(TIMEOUT, f"not every gripper screw STALLED within {timeout_s:.0f} s: check the grip",
                               ctx.now() - started, _motor_detail(tools, gripper))
        return ToolOutcome(STALLED, "every gripper screw STALLED: the bar is clamped", ctx.now() - started,
                           _motor_detail(tools, gripper))
    finally:
        stop_all(tools)


def _robotiq_detail(gripper: RobotiqGripper, target: float) -> dict:
    """What the gripper reported, as JSON values."""
    state = gripper.state
    return {"target": target, "position": state.position, "result_ok": state.last_result_ok}


def _motor_state(tool: ScaffoldingV3, motor: int) -> str | None:
    """What the tool last reported for `motor`."""
    return tool.state.gripper_motor if motor == ScaffoldingV3.GRIPPER_MOTOR else tool.state.joint_motor


def _fresh(tool: ScaffoldingV3, now: float) -> bool:
    """Whether the tool reported within STATUS_MAX_AGE of `now`."""
    updated = tool.state.last_update_time
    return updated is not None and now - updated <= STATUS_MAX_AGE


def _reports(tool: ScaffoldingV3, motor: int, since: float, stalled: bool) -> bool:
    """Whether the tool reported after `since`, with `motor` STALLED or (stalled False) not."""
    updated = tool.state.last_update_time
    return updated is not None and updated > since and (_motor_state(tool, motor) == ScaffoldingV3.STALLED) == stalled


def _motor_detail(tools: Mapping[str, ScaffoldingV3], motor: int) -> dict:
    """Each tool's `motor` state, as JSON values."""
    return {arm: _motor_state(tool, motor) for arm, tool in tools.items()}
