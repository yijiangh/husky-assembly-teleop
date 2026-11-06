"""
The insert: a held bar into its joint, rigid under joint control, then compliant while the mating screws tighten.

! Its mode follows from the bar, never from a setting:
- a ground bar: the whole path rigid; no screw tightens; it ends with the motion.
- every other bar: rigid up to the split, then compliant to the end on a straight line or the spiral, the mating screws
  tightening from the start; it ends when they stall. At a one-mate bar's stall (one arm of two mating), only the
  mating arm goes back at once: the non-mating arm first finishes at its line's end under compliance and settles
  against the force there (SETTLE_CEILING at most).

! Every ending, a cancel included, stops every tool and brings the arms back under joint control. Main thread, from a
plugin task.
"""

from __future__ import annotations

import asyncio
import math
from dataclasses import dataclass
from functools import partial
from typing import TYPE_CHECKING, Callable, Collection, Mapping, Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose, compose

from ...robot_interface.arm import SCALED_JOINT_TRAJECTORY_CONTROLLER
from ...robot_interface.end_effectors import ScaffoldingV3
from ...robot_interface.soft_stop import MotionRefused
from ...world.tcp import tool0_in_base_link
from ..assembly_planning.joints import arm_joint_names
from .compliant_part import (Phase, check_part, enter_compliance, first_refused_target, moving_arms, phase_keeper,
                             started_part, stop_then_leave, tool0_by_arm)
from .controllers import leave_compliance
from .send import ARRIVED, ENDED, STOPPED, MotionCommand, motion_command, send_motion
from .send import DRY_RUN as SENT_DRY
from .send import HOLD_NOT_SENT as SENT_NOT_HELD
from .send import REFUSED as SENT_REFUSED
from .spiral import SPIRAL_RADIUS, SPIRAL_RETURN, Spiral, run_spiral, spiral_fits, spiral_problem
from .stream import (ARRIVAL, CANCELLED, DRY_RUN, FAILED, HOLD_FOR_STALL, HOLD_NOT_SENT, NO_STALL, REFUSED,
                     RIGID_DONE, SETTLE_CEILING, SETTLE_WINDOW, STALLED, PartOutcome, Recorder, Segment, Settling,
                     line_target, measured_tool0, progress, split_rows, split_seconds, stream)
from .tools import StallGate, stop_all, stop_and_clear

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext
    from ...robot_interface.robot import HuskyRobotInterface
    from .spiral import Gate, Operator

#: Arms whose tool0 travels sum to less than this, metres, give no axis to spiral around: the straight line runs.
MIN_AXIS = 1e-6
#: The end of every insert prompt but the ground bar's.
WATCH = "Watch the bar enter its joint and the held bars stay in their jaws."
JOINT_MOTOR = ScaffoldingV3.JOINT_MOTOR


@dataclass(frozen=True)
class InsertPlan:
    """An insert worked out from its planned path: its mode, its rigid part, its compliant line, its prompt.

    Attributes:
        serial: The robot.
        arms: The moving arms, in the robot's arm order.
        names: The path's joint names.
        rows: The path (N x joints); row 0 is where the arms were.
        mating: The arms whose joint screw tightens; none for a ground bar; one of two arms is a one-mate bar (the
            other arm finishes its line after the stall).
        grounded: A ground bar: the whole path rigid, no tighten.
        split_mm: The split asked for: the last millimetres compliant.
        achieved_mm: The compliant part's length as cut on the path; 0 when the insert is not compliant.
        total_mm: The path's length: the furthest any arm's tool0 travels.
        rigid: The rigid part, timed; None when the whole path is compliant.
        compliant_seconds: The straight compliant line's seconds; 0 when the insert is not compliant.
        ends: Arm name -> its tool0 at the path's end, in its base_link.
        axis: The planned insert's direction in the husky frame, metres: every arm's tool0 travel, summed.
        spiral: The spiral's (seconds, turns), or None for the straight line.
        text: The prompt after "<robot> (<serial>): <movement> — ".
    """

    serial: str
    arms: tuple[str, ...]
    names: tuple[str, ...]
    rows: np.ndarray
    mating: tuple[str, ...]
    grounded: bool
    split_mm: float
    achieved_mm: float
    total_mm: float
    rigid: MotionCommand | None
    compliant_seconds: float
    ends: Mapping[str, Pose]
    axis: np.ndarray
    spiral: tuple[float, int] | None
    text: str

    @property
    def compliant(self) -> bool:
        """bool: Whether the insert ends under compliance: every bar but a ground bar."""
        return not self.grounded

    def as_record(self) -> dict:
        """The plan as JSON values; the rows are left out (the rigid command's record has them)."""
        return {"serial": self.serial, "arms": list(self.arms), "mating": list(self.mating),
                "grounded": self.grounded, "split_mm": self.split_mm,
                "achieved_mm": self.achieved_mm, "total_mm": self.total_mm,
                "rigid_s": 0.0 if self.rigid is None else self.rigid.duration,
                "compliant_s": self.compliant_seconds, "axis": self.axis.tolist(),
                "spiral": None if self.spiral is None else Spiral(self.axis, *self.spiral).as_record()}


def plan_insert(robot: HuskyRobotInterface, names: Sequence[str], rows: Sequence[Sequence[float]], *,
                mating: Collection[str], grounded: bool, split_mm: float, spiral: tuple[float, int] | None,
                traj_time: float) -> InsertPlan | str:
    """Work out an insert from its planned path: its mode, where it turns compliant, each part's time, its prompt.

    - A ground bar ignores the split and the spiral.
    - ! A spiral over SPIRAL_MAX_SPEED is refused here, before anything is sent.

    Args:
        robot: The robot; its URDF gives each arm's tool0.
        names: The path's joint names.
        rows: The path (N x joints); row 0 is where the arms are.
        mating: The arms whose joint screw tightens (the kept tighten drives); none for a ground bar.
        grounded: The bar stands on the ground.
        split_mm: The run setting: the last millimetres compliant.
        spiral: (seconds, turns) to spiral the compliant part, or None for the straight line.
        traj_time: Seconds for the whole path.

    Returns:
        InsertPlan | str: The plan, or why not (a spiral too fast, a path the arms can't run).
    """
    spiral = None if grounded else spiral
    line = "straight line"
    if spiral is not None and not spiral_fits(mating):
        spiral, line = None, "straight line: no spiral without a mating joint"
    problem = "" if spiral is None else spiral_problem(*spiral)
    if problem:
        return problem
    serial, urdf = robot.config.serial, robot.config.model.urdf
    names, rows = tuple(names), np.asarray(rows, dtype=float)
    arms = moving_arms(robot, names)
    if not arms:
        return f"the path moves no arm of {serial}"
    tool0 = partial(tool0_by_arm, urdf, arms, names)

    def in_husky(arm: str, pose: Pose) -> np.ndarray:
        """A position in `arm`'s base_link, in the husky frame."""
        return np.asarray(compose(robot.arms[arm].base_in_husky, pose).position)

    starts, ends = tool0(rows[0]), tool0(rows[-1])
    if grounded:
        leading, achieved, rigid_s, compliant_s = rows, 0.0, traj_time, 0.0
        total = 1e3 * max(float(np.linalg.norm(np.subtract(ends[arm].position, starts[arm].position))) for arm in arms)
    else:
        leading, _trailing, achieved, total = split_rows(
            rows, lambda row: [pose.position for pose in tool0(row).values()], split_mm)
        rigid_s, compliant_s = split_seconds(traj_time, achieved, total)
    try:
        rigid = None if leading is None else motion_command(serial, list(robot.arms), names, leading, rigid_s)
    except ValueError as error:
        return f"the insert can't be sent: {error}"
    axis = np.sum([in_husky(arm, ends[arm]) - in_husky(arm, starts[arm]) for arm in arms], axis=0)
    if spiral is not None and np.linalg.norm(axis) < MIN_AXIS:
        spiral, line = None, "straight line: the planned insert has no travel to spiral around"
    screws = " + ".join(mating)
    if grounded:
        text = (f"RIGID insert of a ground bar, no tighten, {rigid.duration:.1f} s, largest joint travel "
                f"{math.degrees(rigid.travel):.1f}°.")
    else:
        if spiral is not None:
            line = (f"spiral {SPIRAL_RADIUS * 1e3:.0f} mm, {spiral[0]:g} s, {spiral[1]} turns, pitch "
                    f"{SPIRAL_RADIUS * 1e3 / spiral[1]:.3g} mm")
        text = (f"INSERT: rigid {total - achieved:.1f} mm ({0.0 if rigid is None else rigid.duration:.1f} s), then "
                f"COMPLIANT {achieved:.1f} mm ({line}); the joint screw of {screws} TIGHTENS from the start until it "
                f"stalls (≤ {HOLD_FOR_STALL:.0f} s), then STOP and back to joint control. {WATCH}")
    return InsertPlan(serial, arms, names, rows, tuple(mating), grounded, float(split_mm), achieved, total, rigid,
                      compliant_s, ends, axis, spiral, text)


async def run_insert(ctx: PluginContext, robot: HuskyRobotInterface, plan: InsertPlan,
                     tools: Mapping[str, ScaffoldingV3], *, operator: Operator, dry_run: bool,
                     watch: Callable[[], str] | None = None, recorder: Recorder | None = None) -> PartOutcome:
    """Run an insert: checks, the screws, the rigid part, then the compliant part.

    - Refused, nothing sent, unless every check of `part_problem` passes and the arms would take every compliant
      target.
    - The compliant line runs from each arm's measured TCP at the handover to its planned end, fixed in the world with
      the base where it stood when the insert started.

    Args:
        ctx: The plugin's context.
        robot: The robot.
        plan: The insert (`plan_insert`), from the arms' joints now.
        tools: Arm name -> its scaffolding tool, every one of the robot's: all are stopped, the mating ones tighten.
        operator: Asks the warning at the spiral's end.
        dry_run: Build the rigid part's trajectory and check every compliant target, as the arms would; send nothing.
        watch: Checked every tick of the rigid part: why it must stop (e.g. another robot moves), or "".
        recorder: Gets the samples (e.g. for a live readout), made at the start; None: its own.

    Returns:
        PartOutcome: STALLED, RIGID_DONE (a ground bar); NO_STALL, CANCELLED (Cancel at the spiral's end), REFUSED
            (the arms did not move), FAILED, HOLD_NOT_SENT (the rigid part's) or DRY_RUN. Its record: the plan, the
            phases, the switches, stall times, peak |F|, the base drift, the samples, the end errors.

    Raises:
        asyncio.CancelledError: When cancelled; every tool was stopped and the arms brought back first.
    """
    label = f"{plan.serial}: insert"
    arms = [robot.arms[name] for name in plan.arms]
    record: dict = {"plan": plan.as_record()}
    problem = check_part(ctx, robot, plan.arms, tools, record)
    if problem:
        ctx.log_warn(f"{label} refused: {problem}; nothing sent")
        return PartOutcome(REFUSED, f"{problem}; nothing sent", record=record)
    if dry_run:
        return await _dry_run(ctx, robot, plan, record)
    refused = _compliant_refusal(robot, plan)
    if refused:
        ctx.log_warn(f"{label} refused: {refused}; nothing sent")
        return PartOutcome(REFUSED, f"{refused}; nothing sent", record=record)
    (started, recorder), husky = started_part(ctx, robot, recorder), robot.husky_in_world()
    phase = phase_keeper(ctx, started, record.setdefault("phases", []))
    gate: StallGate | None = None
    #: The arms confirmed back under joint control before the end (a one-mate bar's mating arm).
    back_early: set[str] = set()

    def stalled() -> bool:
        """Keep a sample, then say whether every mating screw stalled: the rigid part ends early then."""
        recorder.tick(ctx.now())
        return gate.all_stalled()

    async def steps() -> tuple[str, str]:
        """Clear and start the screws, the rigid part, then the compliant part; how it ended."""
        nonlocal gate
        not_cleared = await stop_and_clear(ctx, tools, JOINT_MOTOR)
        if not_cleared is not None:
            return REFUSED, f"{not_cleared.reason}; the arms did not move"
        mating = {name: tools[name] for name in plan.mating}
        try:
            for tool in mating.values():
                tool.drive_screw(1)
        except MotionRefused as refused:
            return REFUSED, f"{refused}; the arms did not move"
        # * Made in the tick the screws start: the stall times count from here.
        gate = StallGate(ctx, mating, JOINT_MOTOR)
        if mating:
            ctx.log_info(f"{label}: {gate.text} TIGHTENING")
        sent = None
        if plan.rigid is not None:
            since, recorder.phase = ctx.now(), "rigid"
            sent = await send_motion(ctx, robot, plan.rigid, watch=watch, until=stalled)
            phase("rigid", since, sent.status)
            if sent.status not in (ARRIVED, STOPPED, ENDED) or (plan.grounded and sent.status != ARRIVED):
                status = {SENT_REFUSED: REFUSED, SENT_NOT_HELD: HOLD_NOT_SENT}.get(sent.status, FAILED)
                return status, f"rigid part: {sent.reason}"
        if plan.grounded:
            gap = record["end_gap_mm"] = _measured_end_gap(robot, plan) * 1e3
            return RIGID_DONE, f"ground bar in, no tighten; the flanges stopped {gap:.1f} mm from the planned end"
        failed = await enter_compliance(ctx, arms, record, phase=phase)
        if failed:
            return FAILED, f"{failed}; no target sent"
        ended = await _compliant(ctx, robot, plan, husky, gate, operator, recorder, record, phase, back_early)
        if ended == "stalled":
            return STALLED, f"{gate.text} STALLED"
        if ended == "no_stall":
            return NO_STALL, f"no {gate.text} stall within {HOLD_FOR_STALL:.0f} s of the line's end"
        if ended == "cancelled":
            return CANCELLED, "Cancel at the spiral's end"
        return FAILED, f"compliant part: {ended}"

    cancelled = False
    try:
        status, reason = await steps()
    except asyncio.CancelledError:
        cancelled = True
        raise
    finally:
        # ! An arm already back must not be held again: under the joint controller a hold sends a trajectory.
        restored = await stop_then_leave(ctx, partial(stop_all, tools),
                                         [arm for arm in arms if arm.config.name not in back_early], label=label,
                                         record=record, cancelled=cancelled)
    record.update(recorder.as_record(), stall_times=dict(gate.stall_times) if gate is not None else {})
    outcome = PartOutcome(status, reason, ctx.now() - started, record)
    if outcome.ok and not restored:
        outcome = PartOutcome(FAILED, f"{reason}; but the hold or the switch back to "
                                      f"{SCALED_JOINT_TRAJECTORY_CONTROLLER} was refused: check the arms in "
                                      f"robot_control", outcome.seconds, record)
    # * A refused hold was said loudly by `send_motion`; once is enough.
    if outcome.status != HOLD_NOT_SENT:
        forces = ", ".join(f"{name} {force:.1f} N" for name, force in recorder.peak_force.items())
        (ctx.log_info if outcome.ok else ctx.log_warn)(
            f"{label}: {outcome.status}: {outcome.reason}; peak |F| {forces or 'not measured'}")
    return outcome


# --- --- --- --- --- PARTS --- --- --- --- ---

async def _dry_run(ctx: PluginContext, robot: HuskyRobotInterface, plan: InsertPlan, record: dict) -> PartOutcome:
    """Build the rigid part's trajectory, and check every compliant target as the arm would; send nothing."""
    if plan.rigid is not None:
        built = await send_motion(ctx, robot, plan.rigid, dry_run=True)
        if built.status != SENT_DRY:
            return PartOutcome(REFUSED, f"dry run: rigid part: {built.reason}", record=record)
    if plan.compliant:
        refused = _compliant_refusal(robot, plan)
        if refused:
            return PartOutcome(REFUSED, f"dry run: {refused}", record=record)
        ctx.log_warn(f"DRY RUN {plan.serial}: insert: every compliant target passed the arm's checks, NOT SENT")
    return PartOutcome(DRY_RUN, "every check passed; nothing sent (dry run)", record=record)


def _compliant_refusal(robot: HuskyRobotInterface, plan: InsertPlan) -> str:
    """Why an arm would refuse a compliant target (`first_refused_target`) from tool0 at the handover; "" if none."""
    if not plan.compliant:
        return ""
    husky = robot.husky_in_world()
    handover = dict(zip(plan.names, plan.rows[0])) if plan.rigid is None else {
        name: value for motion in plan.rigid.arms
        for name, value in zip(arm_joint_names(motion.arm), motion.positions[-1])}
    start = {name: compose(_arm_base(robot, name, husky),
                           tool0_in_base_link(robot.config.model.urdf, name, handover)) for name in plan.arms}
    end = _line_ends(robot, plan, husky)
    spiral = _spiral(plan, husky)
    if spiral is None:
        lines = [(Segment(start, end, plan.compliant_seconds, 0.0), None)]
    else:
        back = spiral.offset(spiral.seconds)
        out = {name: Pose.from_arrays(np.add(pose.position, back), pose.orientation) for name, pose in end.items()}
        lines = [(Segment(start, end, spiral.seconds, 0.0), spiral.offset),
                 (Segment(out, end, SPIRAL_RETURN, 0.0), None)]
    refused = first_refused_target(robot, lines)
    if refused is None:
        return ""
    name, at, problem = refused
    return f"{name}'s compliant target at {at:.1f} s: {problem}"


async def _compliant(ctx: PluginContext, robot: HuskyRobotInterface, plan: InsertPlan, husky: Pose, gate: Gate,
                     operator: Operator, recorder: Recorder, record: dict, phase: Phase, back_early: set[str]) -> str:
    """Stream the compliant part from each measured TCP to its end, straight or spiralling, until the screws stall.

    The arms run the compliance controller. Keeps its phases, the spiral's state and each arm's end error in the
    record. A one-mate bar stalled before its line's end, or out on the spiral, goes on per arm (`_finish_holding`).

    Returns:
        str: "stalled", "no_stall", "cancelled" (the operator at the spiral's end) or why a stream stopped.
    """
    since = ctx.now()
    start = {name: measured_tool0(robot, name) for name in plan.arms}
    if any(pose is None for pose in start.values()):
        ended = "no measured TCP at the handover"
        phase("spiral" if plan.spiral else "compliant", since, ended)
        return ended
    segment = Segment(start, _line_ends(robot, plan, husky), plan.compliant_seconds, HOLD_FOR_STALL)
    holding = [name for name in plan.arms if name not in plan.mating]
    ends = {name: segment.end[name] for name in holding}
    spiral = _spiral(plan, husky)
    if spiral is None:
        stalled_at = math.inf

        def going(elapsed: float) -> bool:
            """No stall yet; keeps when it came, since `stream` does not return its elapsed."""
            nonlocal stalled_at
            if gate.all_stalled():
                stalled_at = elapsed
                return False
            return True

        recorder.phase = "compliant"
        ended = await stream(ctx, robot, segment, keep_going=going, recorder=recorder)
        ended = {"ended": "stalled", "time": "no_stall"}.get(ended, ended)
        phase("compliant", since, ended)
        # ! Only a stall after the first compliant target and before the line's end: earlier, the non-mating arm would
        #   push its whole line against a closed joint; later, both arms stand at the end already.
        if ended == "stalled" and holding and 0.0 < stalled_at < segment.seconds:
            rest = Segment({name: line_target(segment.start[name], segment.end[name], segment.seconds, stalled_at)
                            for name in holding}, ends, segment.seconds - stalled_at, SETTLE_CEILING)
            ended = await _finish_holding(ctx, robot, plan, rest, f"at {stalled_at:.1f} of {segment.seconds:.1f} s",
                                          gate, recorder, record, phase, back_early)
    else:
        stall_targets: dict[str, Pose] = {}
        ended = await run_spiral(ctx, robot, segment, spiral, gate, operator, recorder, stall_targets)
        record["spiral_text"] = spiral.text(ctx.now() - since)
        phase("spiral", since, ended)
        if ended == "stalled" and holding and stall_targets:
            # * Long enough for the way back from the full radius and the whole line, wherever it stalled.
            rest = Segment({name: stall_targets[name] for name in holding}, ends,
                           SPIRAL_RETURN + segment.seconds, SETTLE_CEILING)
            ended = await _finish_holding(ctx, robot, plan, rest, "in the spiral", gate, recorder, record, phase,
                                          back_early)
    # * Measured against the straight line's end: a spiral stopped part way reads short by up to its radius.
    errors = record["end_error_mm"] = progress(robot, segment)
    known = {name: error for name, error in errors.items() if error is not None}
    travelled = ", ".join(f"{name} {error['achieved_mm']:.1f} of {error['travel_mm']:.1f} mm, "
                          f"{error['remaining_mm']:.1f} mm short" for name, error in known.items())
    if max((error["remaining_mm"] for error in known.values()), default=math.inf) > ARRIVAL * 1e3:
        ctx.log_warn(f"{plan.serial}: insert: the arms did NOT reach the line's end: {travelled or 'no TCP'}")
    else:
        ctx.log_info(f"{plan.serial}: insert: tool0 travelled {travelled}")
    return ended


async def _finish_holding(ctx: PluginContext, robot: HuskyRobotInterface, plan: InsertPlan, rest: Segment, when: str,
                          gate: Gate, recorder: Recorder, record: dict, phase: Phase, back_early: set[str]) -> str:
    """End a one-mate bar's stall per arm: the mating arm back under joint control now, the non-mating arm's line on.

    Switched at the stall, the non-mating arm, still settling against the force, left the bar skewed: it streams the
    rest of its line under compliance, then keeps its end target until its TCP stands still (`Settling`; SETTLE_CEILING
    after the end target at most, said loudly when it is still moving then); the caller's finally then holds it and
    brings it back. The mating arm goes into `back_early` once confirmed back, so that finally does not hold it again.

    Args:
        rest: The non-mating arm's line, from its target at the stall to its planned end.
        when: Where the stall came, for the log.

    Returns:
        str: "stalled" (the non-mating arm settled or the ceiling passed; or the mating arm's way back failed and the
            caller asks every arm again), or why the rest's stream stopped.
    """
    label = f"{plan.serial}: insert"
    holding = list(rest.start)
    mating, finishing = " + ".join(plan.mating), " + ".join(holding)
    since = ctx.now()
    back = await leave_compliance(ctx, [robot.arms[name] for name in plan.mating], label=label)
    phase(f"{mating} back", since, "back" if back else "not back")
    if not back:
        return "stalled"
    back_early.update(plan.mating)
    settling = Settling(robot, holding)
    line_done: float | None = None
    settled: bool | None = None

    def keep_going(elapsed: float) -> bool:
        """The line to its end, then the end target again each tick until the arm settled or the ceiling passed.

        ! The ceiling ends the stream here: its "time" comes END_WAIT later than the Segment's wait.
        """
        nonlocal line_done, settled
        if elapsed < rest.seconds:
            return True
        if line_done is None:
            line_done, recorder.phase = ctx.now(), f"settle {finishing}"
        settled = settling.tick(ctx.now())
        return settled is None

    since, recorder.phase = ctx.now(), f"compliant {finishing}"
    ctx.log_info(f"{label}: {gate.text} STALLED {when}; {mating} back under joint control, {finishing} finish the "
                 f"line under compliance ({rest.seconds:.1f} s)")
    ended = await stream(ctx, robot, rest, keep_going=keep_going, recorder=recorder)
    phase(f"compliant {finishing}", since, ended)
    if line_done is not None:
        # * A stream failure during the settle keeps its reason as the settle's end.
        phase(f"settle {finishing}", line_done, {True: "settled", False: "not settled"}.get(settled, ended))
    if settled is not None:
        record["settle"] = {"arms": list(holding), "seconds": settling.seconds, "moved_mm": settling.moved_mm,
                            "settled": settled}
        if settled:
            ctx.log_info(f"{label}: {finishing} settled after {settling.seconds:.1f} s (moved {settling.moved_mm:.2f} "
                         f"mm in the last {SETTLE_WINDOW:g} s), held")
        else:
            moving = ("its move unknown (no TCP)" if settling.moved_mm is None
                      else f"still moving {settling.moved_mm:.2f} mm per {SETTLE_WINDOW:g} s")
            ctx.log_warn(f"{label}: {finishing} NOT settled {SETTLE_CEILING:g} s after its last target, {moving}; "
                         f"held anyway")
    return "stalled" if ended == "ended" else ended


def _measured_end_gap(robot: HuskyRobotInterface, plan: InsertPlan) -> float:
    """The furthest any moving arm's measured TCP is from its planned end, metres; infinite without a TCP.

    * The measured TCP, so the URDF's error shows; the servo's gate (`servo.insert_end_gap`) compares the URDF's tool0
      at the live joints instead.
    """
    gaps = [math.inf if robot.arms[name].state.tcp_position is None else
            float(np.linalg.norm(np.subtract(robot.arms[name].state.tcp_position, plan.ends[name].position)))
            for name in plan.arms]
    return max(gaps)


def _arm_base(robot: HuskyRobotInterface, name: str, husky: Pose) -> Pose:
    """An arm's base_link in the world, with the husky frame at `husky`."""
    return compose(husky, robot.arms[name].base_in_husky)


def _line_ends(robot: HuskyRobotInterface, plan: InsertPlan, husky: Pose) -> dict[str, Pose]:
    """Each arm's planned end in the world, with the husky frame at `husky`: E_s = B·H·A_s·FK_s(q_N)."""
    return {name: compose(_arm_base(robot, name, husky), plan.ends[name]) for name in plan.arms}


def _spiral(plan: InsertPlan, husky: Pose) -> Spiral | None:
    """The plan's spiral around its axis turned into the world with the husky frame at `husky`, or None."""
    if plan.spiral is None:
        return None
    return Spiral(Rotation.from_quat(husky.orientation).apply(plan.axis), *plan.spiral)
