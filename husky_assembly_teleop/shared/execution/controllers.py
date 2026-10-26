"""
Switch a robot's arms between the joint trajectory controller and the Cartesian compliance controller, and back.

- `switch_arms` asks once and waits for every arm; it reports a failure and never falls back (W:4037-4114).
- `leave_compliance` is the way out of every compliant part: hold, ask for joint control at once, then wait for it in a
  task that neither a second cancel nor Stop all ends. A switch to compliance answered late is switched back
  (W:6841-6858).
- ! Main thread, from a plugin task.
"""

from __future__ import annotations

import asyncio
import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Collection

from ...plugin_api.concurrency import WaitTimeout
from ...robot_interface.arm import CARTESIAN_COMPLIANCE_CONTROLLER, SCALED_JOINT_TRAJECTORY_CONTROLLER

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext
    from ...robot_interface.arm import ArmInterface

#: Seconds for every arm to answer a switch (W:4031); for the controller manager to say what runs, first (W:4034).
SWITCH_TIMEOUT = 5.0
QUERY_TIMEOUT = 2.0
#: Seconds the way back to joint control is watched before the arms are reported as maybe left compliant.
RESTORE_TIMEOUT = 10.0
#: Seconds between two requests to switch the same arm back, so a controller manager that is down is not flooded.
RETRY_PERIOD = 1.0


@dataclass(frozen=True)
class SwitchOutcome:
    """How a controller switch ended.

    Attributes:
        ok: Every arm runs the controller asked for.
        reason: One line saying what happened.
        seconds: From the start to the end of the wait.
        active: Arm name -> the controller it runs at the end, as last reported.
    """

    ok: bool
    reason: str
    seconds: float
    active: dict[str, str | None]


async def switch_arms(ctx: PluginContext, arms: Collection[ArmInterface], controller: str, *,
                      timeout_s: float = SWITCH_TIMEOUT) -> SwitchOutcome:
    """Switch every arm to `controller` and wait until each runs it; a failure is reported, never worked around.

    - Refused, nothing asked, while any arm has a switch in flight.
    - The controller manager is first asked what runs (up to QUERY_TIMEOUT, then its last answer counts); only arms
      not running `controller` are switched.
    - ! A failure may leave some arms switched or still switching: end the part with `leave_compliance` all the same.

    Args:
        ctx: The plugin's context.
        arms: The arms to switch.
        controller: The controller to run.
        timeout_s: Seconds for every arm to answer the switch.

    Returns:
        SwitchOutcome: ok once every arm runs `controller`; else why not (a switch in flight, a switch refused or
            rejected, no answer within `timeout_s`), also logged as an error.
    """
    started = ctx.now()
    names = _names(arms)

    def outcome(ok: bool, reason: str) -> SwitchOutcome:
        """Log the ending and say what each arm runs."""
        (ctx.log_info if ok else ctx.log_error)(f"{names}: {reason}")
        return SwitchOutcome(ok, reason, ctx.now() - started,
                             {arm.config.name: arm.state.controllers.active for arm in arms})

    busy = [f"{arm.config.name} to {arm.state.controllers.switch_in_flight}" for arm in arms
            if arm.state.controllers.switch_in_flight is not None]
    if busy:
        return outcome(False, f"not switching to {controller}: still switching {', '.join(busy)}")
    asked = [arm for arm in arms if arm.controllers.refresh()]
    try:
        await ctx.wait_until(lambda: all(_answered_since(arm, started) for arm in asked), timeout_s=QUERY_TIMEOUT,
                             description="the controller manager to say what runs")
    except WaitTimeout:
        ctx.log_warn(f"{names}: the controller manager did not say what runs within {QUERY_TIMEOUT:.0f} s; "
                     f"going by its last answer")
    switching = [arm for arm in arms if not arm.controllers.is_active(controller)]
    if not switching:
        return outcome(True, f"already under {controller}")
    for arm in switching:
        if not arm.controllers.switch(controller):
            return outcome(False, f"switch to {controller} FAILED: {arm.state.controllers.switch_error}")

    def errors() -> list[str]:
        """Why the switches asked for failed, if any did."""
        return [arm.state.controllers.switch_error for arm in switching if arm.state.controllers.switch_error]

    try:
        await ctx.wait_until(lambda: bool(errors()) or all(_runs(arm, controller) for arm in arms),
                             timeout_s=timeout_s, description=f"the switch to {controller}")
    except WaitTimeout:
        running = ", ".join(f"{arm.config.name} runs {arm.state.controllers.active}" for arm in arms)
        return outcome(False, f"switch to {controller} FAILED: no answer within {timeout_s:.0f} s ({running}); "
                              f"is the controller manager running?")
    if errors():
        return outcome(False, f"switch to {controller} FAILED: {'; '.join(errors())}")
    return outcome(True, f"{controller} running ({ctx.now() - started:.1f} s)")


def request_joint_control(ctx: PluginContext, arms: Collection[ArmInterface]) -> tuple[str, list[str]]:
    """Hold every arm, then ask each one not under joint control to switch back; sends at once, waits for nothing.

    - An arm with a switch in flight is not asked: `restore_joint_control` asks once that switch is answered.
    - The switch back needs no lease, as a hold: it goes out even after the caller's lease is given back.

    Args:
        ctx: The plugin's context, for its log.
        arms: The arms to hold and bring back.

    Returns:
        tuple[str, list[str]]: What was asked of which arm, as logged; the names of the arms that refused their hold
            (a missing or stale measurement: the arm's log says, and the pendant stops it).
    """
    joint = SCALED_JOINT_TRAJECTORY_CONTROLLER
    not_held, requested, in_flight, not_sent = [], [], [], []
    for arm in arms:
        name, pending = arm.config.name, arm.state.controllers.switch_in_flight
        if not arm.hold():
            not_held.append(name)
        if pending is not None:
            in_flight.append(f"{name} (to {pending})")
        elif arm.controllers.is_active(joint):
            continue
        elif arm.controllers.switch(joint, unguarded=True):
            requested.append(name)
        else:
            not_sent.append(name)
    lines = []
    if not_held:
        lines.append(f"the hold was refused on {', '.join(not_held)} (its log says why)")
    if requested:
        lines.append(f"{', '.join(requested)}: switch back to {joint} requested")
    if in_flight:
        lines.append(f"not asked, a switch in flight: {', '.join(in_flight)}")
    if not_sent:
        lines.append(f"{', '.join(not_sent)}: switch back to {joint} NOT SENT (the log says why)")
    summary = "; ".join(lines) or f"{_names(arms)}: already under {joint}"
    (ctx.log_warn if not_held or in_flight or not_sent else ctx.log_info)(summary)
    return summary, not_held


async def restore_joint_control(ctx: PluginContext, arms: Collection[ArmInterface], *, label: str,
                                timeout_s: float = RESTORE_TIMEOUT) -> bool:
    """Wait until every arm runs the joint controller, asking again for each arm that doesn't and has no switch pending.

    A switch to compliance answered late lands here and is switched back (W:6841-6858).

    Args:
        ctx: The plugin's context.
        arms: The arms to bring back.
        label: Who brings them back, for the log, e.g. "Cindy (0806): B21 insert".
        timeout_s: Seconds to wait.

    Returns:
        bool: True once every arm is back; False after `timeout_s`, with a loud error to switch them by hand.

    Raises:
        asyncio.CancelledError: When cancelled; the arms not back yet get the same loud error first.
    """
    joint = SCALED_JOINT_TRAJECTORY_CONTROLLER
    started = ctx.now()
    deadline = started + timeout_s
    asked: dict[str, float] = {}
    try:
        while not all(_runs(arm, joint) for arm in arms):
            now = ctx.now()
            if now >= deadline:
                _report_left(ctx, arms, label, f"within {timeout_s:.0f} s")
                return False
            for arm in arms:
                name, running = arm.config.name, arm.state.controllers.active
                if not arm.controllers.is_active(joint) and arm.state.controllers.switch_in_flight is None \
                        and now - asked.get(name, -math.inf) >= RETRY_PERIOD:
                    asked[name] = now
                    sent = arm.controllers.switch(joint, unguarded=True)
                    ctx.log_warn(f"{label}: {name} runs {running}; switch back to {joint} "
                                 + ("requested" if sent else "NOT SENT (the log says why)"))
            await ctx.next_tick()
    except asyncio.CancelledError:
        _report_left(ctx, arms, label, "when the wait for them was cancelled")
        raise
    ctx.log_info(f"{label}: {_names(arms)} back under {joint} ({ctx.now() - started:.1f} s)")
    return True


async def leave_compliance(ctx: PluginContext, arms: Collection[ArmInterface], *, label: str) -> bool:
    """End a compliant part: hold and ask for joint control at once, then wait for it in a task of its own.

    - ! Call it in the part's `finally`: the request goes out before the first wait, so within the shutdown grace.
    - The wait is shielded and survives Stop all: a second cancel ends the caller, not the restore. While the plugin
      is closing no task can start; the request then stays unconfirmed, said loudly.

    Args:
        ctx: The plugin's context.
        arms: The arms of the part.
        label: Who leaves, for the log, e.g. "Cindy (0806): B21 insert".

    Returns:
        bool: Whether every hold went out and every arm is back under joint control (`restore_joint_control`).

    Raises:
        asyncio.CancelledError: If the caller is cancelled while it waits; the restore goes on.
    """
    asked, not_held = request_joint_control(ctx, arms)
    if all(_runs(arm, SCALED_JOINT_TRAJECTORY_CONTROLLER) for arm in arms):
        return not not_held
    try:
        # * Survives Stop all: it only asks for the joint controller, unguarded, which never moves an arm.
        restore = ctx.spawn(f"restore joint control: {label}", restore_joint_control(ctx, arms, label=label),
                            survives_soft_stop=True)
    except RuntimeError:
        ctx.log_error(f"{label}: back to {SCALED_JOINT_TRAJECTORY_CONTROLLER} not confirmed, the plugin is closing "
                      f"({asked}); check {_names(arms)} in robot_control before anything else")
        return False
    return await ctx.join(restore) and not not_held


def _runs(arm: ArmInterface, controller: str) -> bool:
    """Whether `arm` runs `controller`, as last reported, with no switch pending."""
    return arm.controllers.is_active(controller) and arm.state.controllers.switch_in_flight is None


def _answered_since(arm: ArmInterface, since: float) -> bool:
    """Whether the arm's controller manager answered at or after ROS time `since`."""
    updated = arm.state.controllers.last_update_time
    return updated is not None and updated >= since


def _report_left(ctx: PluginContext, arms: Collection[ArmInterface], label: str, when: str) -> None:
    """Log loudly the arms not back under joint control, if any: they may still be compliant."""
    left = []
    for arm in arms:
        state = arm.state.controllers
        if not _runs(arm, SCALED_JOINT_TRAJECTORY_CONTROLLER):
            pending = "" if state.switch_in_flight is None else f", switching to {state.switch_in_flight}"
            left.append(f"{arm.config.name} (runs {state.active}{pending})")
    if left:
        ctx.log_error(f"{label}: ARMS MAY BE LEFT UNDER {CARTESIAN_COMPLIANCE_CONTROLLER}: {', '.join(left)} not "
                      f"back under {SCALED_JOINT_TRAJECTORY_CONTROLLER} {when}; switch them in robot_control before "
                      f"anything else")


def _names(arms: Collection[ArmInterface]) -> str:
    """The arms' names, for the log."""
    return ", ".join(arm.config.name for arm in arms)
