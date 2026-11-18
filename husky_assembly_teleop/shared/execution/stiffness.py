"""
Setting the compliance controller's stiffness for one part, and setting the one it had back afterwards.

- Only an arm whose stiffness could be read is set, so it can always be set back; one that could not is said loudly and
  runs with what its controller has.
- ! A set is recorded the moment it goes out, before any answer: `send_back` then sets back every arm a set went to,
  however the set ended (taken, late, a cancel while waiting).
- ! Main thread, from a plugin task.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Collection, Mapping, Sequence

from ...robot_interface.arm import parameters_taken, stiffness_values

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext
    from ...robot_interface.arm import ArmInterface

#: Seconds an answer of the controller's parameter services is waited for.
ANSWER_TIMEOUT = 1.0


async def set_stiffness(ctx: PluginContext, arms: Collection[ArmInterface], stiffness: Sequence[float],
                        record: dict[str, dict], *, label: str) -> None:
    """Read each arm's translational stiffness, then set `stiffness` (N/m along tool0's x, y, z) where it was read.

    Args:
        ctx: The plugin's context.
        arms: The part's arms.
        stiffness: The three values.
        record: Gets, per arm a set went to, {"before": the values read, "set": whether the controller said it took
            them, None while waiting}; filled as each set goes out, so a cancel meanwhile still finds it.
        label: Who, for the log.
    """
    read = await _answers(ctx, {arm.config.name: arm.request_stiffness_values() for arm in arms})
    unread, refused, sets = [], [], {}
    for arm in arms:
        name = arm.config.name
        before = stiffness_values(read[name])
        if before is None:
            unread.append(name)
            continue
        future = arm.request_stiffness(stiffness)
        if future is None:
            refused.append(name)
            continue
        record[name] = {"before": before, "set": None}
        sets[name] = future
    for name, answer in (await _answers(ctx, sets)).items():
        record[name]["set"] = parameters_taken(answer)
    values = "/".join(f"{value:g}" for value in stiffness)
    late = [name for name in sets if not record[name]["set"]]
    if unread:
        ctx.log_warn(f"{label}: stiffness NOT set on {', '.join(unread)}: its stiffness could not be read to set back; "
                     f"it runs with what its compliance controller has")
    if refused:
        ctx.log_warn(f"{label}: stiffness {values} N/m NOT sent to {', '.join(refused)} (refused or no service)")
    if late:
        ctx.log_warn(f"{label}: stiffness {values} N/m sent to {', '.join(late)}, not confirmed; it is set back at "
                     f"the end all the same")
    taken = [name for name in sets if record[name]["set"]]
    if taken:
        ctx.log_info(f"{label}: stiffness {values} N/m (tool0 x/y/z) on {', '.join(taken)}, was " + ", ".join(
            "/".join(f"{value:g}" for value in record[name]["before"]) for name in taken))


def send_back(arms: Collection[ArmInterface], record: Mapping[str, dict]) -> dict[str, Any]:
    """Send every arm in `record` its stiffness from before; never raises, never waits: call it first in a finally.

    Returns:
        dict[str, Any]: Arm name -> the call's future, or None when it could not be sent (no lease, no service).
    """
    sent = {}
    for arm in arms:
        kept = record.get(arm.config.name)
        if kept:
            try:
                sent[arm.config.name] = arm.request_stiffness(kept["before"], restore=True)
            except Exception:  # noqa: BLE001 - in a finally: said by `check_back`, never raised
                sent[arm.config.name] = None
    return sent


async def check_back(ctx: PluginContext, sent: Mapping[str, Any], *, label: str) -> None:
    """Wait for `send_back`'s answers; warn for each arm not confirmed back."""
    answers = await _answers(ctx, sent)
    failed = [name for name, answer in answers.items() if not parameters_taken(answer)]
    if failed:
        ctx.log_warn(f"{label}: the stiffness of {', '.join(failed)} was NOT confirmed set back; check the compliance "
                     f"controller's stiffness parameters before the next compliant part")


async def _answers(ctx: PluginContext, futures: Mapping[str, Any]) -> dict[str, Any]:
    """Each future's answer, waited for up to ANSWER_TIMEOUT in all; None for a call not sent, failed or late."""
    done: dict[str, Any] = {}
    for name, future in futures.items():
        if future is not None:
            future.add_done_callback(lambda answered, name=name: done.__setitem__(name, answered))
    waiting = [name for name, future in futures.items() if future is not None]
    deadline = ctx.now() + ANSWER_TIMEOUT
    while any(name not in done for name in waiting) and ctx.now() < deadline:
        await ctx.next_tick()
    answers = {}
    for name in futures:
        try:
            answers[name] = done[name].result() if name in done else None
        except Exception:  # noqa: BLE001 - a failed call reads as no answer, said by the caller
            answers[name] = None
    return answers
