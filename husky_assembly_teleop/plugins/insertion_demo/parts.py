"""
The demo's parts on the robot, each from the live joints.

The steps of an action, a path from where the arms are, the check that they stand at the insert's start, the gripper
screws, the compliant hold, and the removal.

`run_unscrew_retreat` is the retreat with the bar kept in the tools: the joint screws run in reverse for a set time
under compliance, then the tools back out along the insert line to where the insert started. Built from the compliant
parts' blocks (`shared/execution/compliant_part.py`), as `retreat.run_retreat` is.

! The coroutines: main thread, from a plugin task, the robot leased.
"""

from __future__ import annotations

import asyncio
import math
from functools import partial
from typing import TYPE_CHECKING, Callable, Collection, Iterable, Mapping, Sequence

import numpy as np

from scaffolding_assembly_core.geometry import Pose, compose

from ...plugin_api.concurrency import WaitTimeout
from ...robot_interface.end_effectors import ScaffoldingV3
from ...robot_interface.soft_stop import MotionRefused
from ...shared.assembly_planning.joints import arm_joint_names
from ...shared.assembly_planning.movement import Routine, routine_or_none
from ...shared.assembly_planning.route_result import pose_error
from ...shared.execution.basis import STALE_ARM
from ...shared.execution.compliant_part import (check_part, enter_compliance, phase_keeper, started_part,
                                                stop_then_leave)
from ...shared.execution.force_sensors import zero_force_sensors
from ...shared.execution.stream import (ARRIVAL, FAILED, REFUSED, RETREATED, PartOutcome, Recorder, Segment,
                                        arm_base_in_world, measured_tool0, progress, stream)
from ...shared.execution.tools import StallGate, stop_all, stop_and_clear
from ...world.tcp import tool0_in_base_link

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext
    from ...robot_interface.arm import ArmInterface
    from ...robot_interface.robot import HuskyRobotInterface
    from ...shared.assembly_planning.process_steps import LoadedProcess

#: Every tool0 this close to the insert's start, metres and radians, stands at it: the insert may run.
AT_START = (0.005, math.radians(1.0))
#: Seconds a gripper screw gets to stall after its close or open.
GRIP_TIMEOUT = 15.0
JOINT_MOTOR, GRIPPER_MOTOR = ScaffoldingV3.JOINT_MOTOR, ScaffoldingV3.GRIPPER_MOTOR


# --- --- --- --- --- THE PROCESS --- --- --- --- ---

def demo_steps(loaded: LoadedProcess, action_id: str) -> tuple[int, int] | str:
    """The indices into `loaded.steps` of an action's transfer and insert; or why the action has no such pair.

    The insert is the action's first movement that runs as an insert; the transfer the last free transfer before it
    (the transfer workaround: a free move with the bar hidden).
    """
    action = loaded.process.actions[action_id]
    transfer = insert = None
    for index, movement in enumerate(action.movements):
        routine = routine_or_none(action, movement, transfer_as_free=True)
        if routine is Routine.TRANSFER_FREE and insert is None:
            transfer = index
        elif routine is Routine.INSERT:
            insert = index
            break
    if insert is None:
        return f"{action_id} has no insert"
    if transfer is None:
        return f"{action_id} has no transfer before its insert"
    return loaded.index_of(action_id, transfer), loaded.index_of(action_id, insert)


# --- --- --- --- --- WHERE THE ARMS ARE --- --- --- --- ---

def live_rows(robot: HuskyRobotInterface, names: Sequence[str],
              rows: Sequence[Sequence[float]]) -> tuple[list[str], np.ndarray] | str:
    """A planned path with its first row replaced by the arms' joints now; or why it can't be.

    Returns:
        tuple[list[str], np.ndarray] | str: (names, rows), or the refusal (a joint without a state, the arms over
            STALE_ARM from the path's start: plan again).
    """
    names, rows = list(names), np.array(rows, dtype=float)
    live: dict[str, float] = {}
    for name, arm in robot.arms.items():
        here = arm.joint_vector()
        if here is not None:
            live.update(zip(arm_joint_names(name), here.tolist()))
    missing = [name for name in names if name not in live]
    if missing:
        return f"no joint state for {', '.join(missing)}"
    now = np.array([live[name] for name in names])
    gap = float(np.max(np.abs(now - rows[0])))
    if gap > STALE_ARM:
        return (f"the arms are {math.degrees(gap):.1f}° from the plan's start (over {math.degrees(STALE_ARM):.0f}°): "
                f"plan again")
    # * Row 0 is where the arms are, so the trajectory starts without a jump.
    rows[0] = now
    return names, rows


def tool0_at(robot: HuskyRobotInterface, joints: Mapping[str, float], arms: Iterable[str]) -> dict[str, Pose] | None:
    """Each arm's tool0 in the world at `joints`, the base where mocap sees it now; None before a base pose."""
    poses = {}
    for arm in arms:
        base = arm_base_in_world(robot, arm)
        if base is None:
            return None
        poses[arm] = compose(base, tool0_in_base_link(robot.config.model.urdf, arm, joints))
    return poses


def off_start(robot: HuskyRobotInterface, joints: Mapping[str, float], arms: Iterable[str]) -> str:
    """Why the arms do not stand at `joints` (tool0 over AT_START away by the URDF's kinematics), or "" if they do."""
    urdf = robot.config.model.urdf
    for arm in arms:
        state = robot.arms[arm].state
        if robot.arms[arm].joint_vector() is None:
            return f"{arm}: no joint state yet"
        distance, turn = pose_error(tool0_in_base_link(urdf, arm, state.joint_positions),
                                    tool0_in_base_link(urdf, arm, joints))
        if distance > AT_START[0] or turn > AT_START[1]:
            return (f"{arm}'s tool0 is {distance * 1e3:.1f} mm, {math.degrees(turn):.1f}° from the insert's start "
                    f"(limit {AT_START[0] * 1e3:.0f} mm, {math.degrees(AT_START[1]):.0f}°)")
    return ""


# --- --- --- --- --- THE PARTS --- --- --- --- ---

async def run_grip(ctx: PluginContext, tools: Mapping[str, ScaffoldingV3], close: bool, *,
                   timeout_s: float = GRIP_TIMEOUT) -> tuple[bool, str]:
    """Close (or open) every gripper screw and wait until each stalls; every motor is stopped at the end.

    Args:
        ctx: The plugin's context.
        tools: Arm name -> its scaffolding tool.
        close: Tighten the gripper screws (the bar clamped); False loosens them.
        timeout_s: Seconds for every screw to stall.

    Returns:
        tuple[bool, str]: Whether every screw stalled, and one line saying what happened.
    """
    verb = "close" if close else "open"
    not_cleared = await stop_and_clear(ctx, tools, GRIPPER_MOTOR)
    if not_cleared is not None:
        return False, f"{not_cleared.reason}; nothing {verb}d"
    try:
        for tool in tools.values():
            (tool.close if close else tool.open)()
    except MotionRefused as refused:
        stop_all(tools)
        return False, f"{refused}; not every gripper screw {verb}s"
    gate = StallGate(ctx, tools, GRIPPER_MOTOR)
    ctx.log_info(f"{gate.text} {verb.upper()}")
    try:
        await ctx.wait_until(gate.all_stalled, timeout_s=timeout_s, description="the gripper screws to stall")
    except WaitTimeout:
        return False, f"not every gripper screw stalled within {timeout_s:.0f} s of its {verb}; motors stopped"
    finally:
        stop_all(tools)
    times = ", ".join(f"{arm} {seconds:.1f} s" for arm, seconds in gate.stall_times.items())
    return True, f"gripper screws {verb}d: stalled after {times}"


async def run_compliant_hold(ctx: PluginContext, arms: Collection[ArmInterface], released: Callable[[], bool], *,
                             label: str) -> str:
    """Put the arms under the compliance controller, zero target wrench, until `released`; then bring them back.

    The force sensors are zeroed first, so the arms give way to a push only. A cancel (Cancel, Stop all) only holds:
    the arms stay compliant, said loudly.

    Returns:
        str: Why the arms are not under compliance, or not back under joint control; "" when all went well.
    """
    record: dict = {}
    await zero_force_sensors(ctx, arms, label)
    failed = await enter_compliance(ctx, arms, record)
    cancelled = False
    try:
        if failed:
            return failed
        ctx.log_info(f"{label}: COMPLIANT: push the arms; Rigid brings them back under joint control")
        await ctx.wait_until(released, description="Rigid")
    except asyncio.CancelledError:
        cancelled = True
        raise
    finally:
        restored = await stop_then_leave(ctx, lambda: None, arms, label=label, record=record, cancelled=cancelled)
    return "" if restored else "not every arm is confirmed back under joint control: the log says; press Rigid again"


async def run_unscrew_retreat(ctx: PluginContext, robot: HuskyRobotInterface, tools: Mapping[str, ScaffoldingV3],
                              ends: Mapping[str, Pose], *, loosen_s: float, seconds: float,
                              recorder: Recorder | None = None) -> PartOutcome:
    """Loosen the joint screws for `loosen_s` under compliance, then back the tools out to `ends`, the bar still held.

    Refused, nothing sent, unless every check of `part_problem` passes. The force sensors are zeroed while the bar
    sits in its joints, so the back-out reacts only to the forces that appear as the tools leave.

    Args:
        ctx: The plugin's context.
        robot: The robot.
        tools: Arm name -> its scaffolding tool, one per moving arm.
        ends: Arm name -> the world pose its tool0 backs out to: the insert's start.
        loosen_s: Seconds the joint screws run in reverse; they do not stall.
        seconds: Seconds the back-out line takes.
        recorder: Gets the samples, made at the start; None: its own.

    Returns:
        PartOutcome: RETREATED, REFUSED (nothing sent) or FAILED; its record keeps the phases, the switches, the force
            zero, the end errors and the samples.

    Raises:
        asyncio.CancelledError: When cancelled; every tool was stopped and the arms held first (left compliant).
    """
    label = f"{robot.config.serial}: unscrew retreat"
    names = tuple(ends)
    arms = [robot.arms[name] for name in names]
    record: dict = {"loosen_s": loosen_s, "seconds": seconds}
    problem = check_part(ctx, robot, names, tools, record)
    if problem:
        ctx.log_warn(f"{label} refused: {problem}; nothing sent")
        return PartOutcome(REFUSED, f"{problem}; nothing sent", record=record)
    started, recorder = started_part(ctx, robot, recorder)
    phase = phase_keeper(ctx, started, record.setdefault("phases", []))
    #: Whether the arms may be under compliance, so the `finally` must bring them back.
    compliant = False

    async def steps() -> tuple[str, str]:
        """Clear the screws, zero the sensors, go compliant, loosen, stop, back out; how it ended."""
        nonlocal compliant
        not_cleared = await stop_and_clear(ctx, tools, JOINT_MOTOR)
        if not_cleared is not None:
            return REFUSED, f"{not_cleared.reason}; nothing sent"
        since = ctx.now()
        record["ft_zero"] = await zero_force_sensors(ctx, arms, label)
        phase("force zero", since, "done")
        since, compliant = ctx.now(), True
        failed = await enter_compliance(ctx, arms, record, phase=phase)
        if failed:
            return FAILED, f"{failed}; no target sent"
        try:
            for tool in tools.values():
                tool.drive_screw(-1)
        except MotionRefused as refused:
            return FAILED, f"{refused}; the screws did not loosen"
        since, recorder.phase = ctx.now(), "loosen"
        ctx.log_info(f"{label}: the joint screws of {' + '.join(tools)} LOOSEN for {loosen_s:g} s")
        while ctx.now() - since < loosen_s:
            recorder.tick(ctx.now())
            await ctx.next_tick()
        # * The motors off before the arms move: one running would keep pushing the bar.
        stop_all(tools)
        phase("loosen", since, "stopped")
        since = ctx.now()
        start = {name: measured_tool0(robot, name) for name in names}
        if any(pose is None for pose in start.values()):
            return FAILED, "no measured TCP after the loosen"
        segment = Segment(start, dict(ends), seconds, 0.0)

        def going(elapsed: float) -> bool:
            """On during the line's time, then until every tool0 arrived."""
            return elapsed < segment.seconds or not all(
                each is not None and each["remaining_mm"] <= ARRIVAL * 1e3
                for each in progress(robot, segment).values())

        recorder.phase = "back-out"
        ended = await stream(ctx, robot, segment, keep_going=going, recorder=recorder)
        reached = record["end_error_mm"] = progress(robot, segment)
        phase("back-out", since, ended)
        travelled = ", ".join("no TCP" if each is None else f"{name} {each['achieved_mm']:.1f} of "
                              f"{each['travel_mm']:.1f} mm, {each['remaining_mm']:.1f} mm short"
                              for name, each in reached.items())
        ctx.log_info(f"{label}: back-out: {travelled}")
        if ended == "time":
            return RETREATED, f"backed out, but not to the insert's start within the time: {travelled}"
        if ended != "ended":
            return FAILED, f"the back-out stopped: {ended}"
        return RETREATED, "the screws loosened and the tools backed out to the insert's start, the bar still held"

    cancelled = False
    try:
        status, reason = await steps()
    except asyncio.CancelledError:
        cancelled = True
        raise
    finally:
        restored = await stop_then_leave(ctx, partial(stop_all, tools), arms if compliant else [], label=label,
                                         record=record, cancelled=cancelled)
    record.update(recorder.as_record())
    outcome = PartOutcome(status, reason, ctx.now() - started, record)
    if outcome.ok and not restored:
        outcome = PartOutcome(FAILED, f"{reason}; but the hold or the switch back to joint control was refused: "
                                      f"check the arms in robot_control", outcome.seconds, record)
    forces = ", ".join(f"{name} {force:.1f} N" for name, force in recorder.peak_force.items())
    (ctx.log_info if outcome.ok else ctx.log_warn)(
        f"{label}: {outcome.status}: {outcome.reason}; peak |F| {forces or 'not measured'}")
    return outcome
