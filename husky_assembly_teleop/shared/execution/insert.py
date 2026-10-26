"""
The insert: a held bar into its joint, rigid under joint control, then compliant while the mating screws tighten.

Its mode, as the old monitor's (W:5385-5437):
- a ground bar: the whole path rigid; no screw tightens; it ends with the motion.
- rigid-only (a run setting): the whole path rigid with the mating screws tightening, then the joint controller holds
  the end until they stall; never compliant.
- otherwise: rigid up to the split, then compliant to the end on a straight line or the spiral, the mating screws
  tightening from the start; it ends when they stall.

! Every ending, a cancel included, stops every tool and brings the arms back under joint control. Main thread, from a
plugin task.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Collection, Mapping, Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose, compose

from ...robot_interface.arm import CARTESIAN_COMPLIANCE_CONTROLLER, SCALED_JOINT_TRAJECTORY_CONTROLLER
from ...robot_interface.end_effectors import ScaffoldingV3
from ...robot_interface.soft_stop import MotionRefused
from ..assembly_planning.joints import arm_joint_names
from .controllers import leave_compliance, switch_arms
from .precheck import part_problem
from .send import ARRIVED, ENDED, STOPPED, MotionCommand, motion_command, send_motion
from .send import DRY_RUN as SENT_DRY
from .send import HOLD_NOT_SENT as SENT_NOT_HELD
from .send import REFUSED as SENT_REFUSED
from .spiral import SPIRAL_RADIUS, SPIRAL_RETURN, Spiral, run_spiral, spiral_problem
from .stream import (ARRIVAL, CANCELLED, DRY_RUN, FAILED, HOLD_FOR_STALL, HOLD_NOT_SENT, MAX_CLOCK_STEP, NO_STALL,
                     REFUSED, RIGID_DONE, STALLED, PartOutcome, Recorder, Segment, fk_tcp_gap, gap_record,
                     measured_tool0, plan_stream, progress, split_rows, split_seconds, stream, target_problem,
                     tool0_fk, zero_wrench)
from .tools import StallGate, stop_all, stop_and_clear

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext
    from ...robot_interface.robot import HuskyRobotInterface
    from .spiral import Gate, Operator

#: A ground bar's insert is finished when every flange stands this close to its planned end, metres (W:1659); only
#: then may the servo after it (M7) run.
GROUNDED_END_TOL = 0.003
#: Arms whose tool0 travels sum to less than this, metres, give no axis to spiral around: the straight line runs.
MIN_AXIS = 1e-6
#: The end of every insert prompt but the ground bar's.
WATCH = "Watch the bar enter its joint and the held bars stay in their jaws. Confirm to start."
JOINT_MOTOR = ScaffoldingV3.JOINT_MOTOR


@dataclass(frozen=True)
class InsertPlan:
    """An insert worked out from its planned path: its mode, its rigid part, its compliant line, its prompt.

    Attributes:
        serial: The robot.
        arms: The moving arms, in the robot's arm order.
        names: The path's joint names.
        rows: The path (N x joints); row 0 is where the arms were.
        mating: The arms whose joint screw tightens; none for a ground bar.
        grounded: A ground bar: the whole path rigid, no tighten.
        rigid_only: The whole path rigid with the mating screws tightening, then held until they stall.
        split_mm: The split asked for: the last millimetres compliant.
        achieved_mm: The compliant part's length as cut on the path; 0 when the insert is not compliant.
        total_mm: The path's length: the furthest any arm's tool0 travels.
        rigid: The rigid part, timed; None when the whole path is compliant.
        compliant_seconds: The straight compliant line's seconds; 0 when the insert is not compliant.
        ends: Arm name -> its tool0 at the path's end, in its base_link.
        axis: The planned insert's direction in the husky frame, metres: every arm's tool0 travel, summed (W:5441).
        spiral: The spiral's (seconds, turns), or None for the straight line.
        text: The prompt after "<robot> (<serial>): <movement> — ".
    """

    serial: str
    arms: tuple[str, ...]
    names: tuple[str, ...]
    rows: np.ndarray
    mating: tuple[str, ...]
    grounded: bool
    rigid_only: bool
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
        """bool: Whether the insert ends under compliance: neither a ground bar nor rigid-only."""
        return not (self.grounded or self.rigid_only)

    def as_record(self) -> dict:
        """The plan as JSON values; the rows are left out (the rigid command's record has them)."""
        return {"serial": self.serial, "arms": list(self.arms), "mating": list(self.mating),
                "grounded": self.grounded, "rigid_only": self.rigid_only, "split_mm": self.split_mm,
                "achieved_mm": self.achieved_mm, "total_mm": self.total_mm,
                "rigid_s": 0.0 if self.rigid is None else self.rigid.duration,
                "compliant_s": self.compliant_seconds, "axis": self.axis.tolist(),
                "spiral": None if self.spiral is None else Spiral(self.axis, *self.spiral).as_record()}


def plan_insert(robot: HuskyRobotInterface, names: Sequence[str], rows: Sequence[Sequence[float]], *,
                mating: Collection[str], grounded: bool, split_mm: float, rigid_only: bool,
                spiral: tuple[float, int] | None, traj_time: float) -> InsertPlan | str:
    """Work out an insert from its planned path: its mode, where it turns compliant, each part's time, its prompt.

    - A ground bar ignores rigid-only; both ignore the split and the spiral.
    - ! A spiral over SPIRAL_MAX_SPEED is refused here, before anything is sent.

    Args:
        robot: The robot; its URDF gives each arm's tool0.
        names: The path's joint names.
        rows: The path (N x joints); row 0 is where the arms are.
        mating: The arms whose joint screw tightens (the kept tighten drives); none for a ground bar.
        grounded: The bar stands on the ground.
        split_mm: The run setting: the last millimetres compliant.
        rigid_only: The run setting: the whole insert under joint control.
        spiral: (seconds, turns) to spiral the compliant part, or None for the straight line.
        traj_time: Seconds for the whole path.

    Returns:
        InsertPlan | str: The plan, or why not (a spiral too fast, a path the arms can't run).
    """
    rigid_only = rigid_only and not grounded
    spiral = None if grounded or rigid_only else spiral
    problem = "" if spiral is None else spiral_problem(*spiral)
    if problem:
        return problem
    serial, urdf = robot.config.serial, robot.config.model.urdf
    names, rows = tuple(names), np.asarray(rows, dtype=float)
    arms = tuple(arm for arm in robot.arms if set(arm_joint_names(arm)) <= set(names))
    if not arms:
        return f"the path moves no arm of {serial}"

    def tool0(row: np.ndarray) -> dict[str, Pose]:
        """Each moving arm's tool0 at a row, in its base_link."""
        joints = dict(zip(names, row))
        return {arm: tool0_fk(urdf, arm, joints) for arm in arms}

    def in_husky(arm: str, pose: Pose) -> np.ndarray:
        """A position in `arm`'s base_link, in the husky frame."""
        return np.asarray(compose(robot.arms[arm].base_in_husky, pose).position)

    starts, ends = tool0(rows[0]), tool0(rows[-1])
    if grounded or rigid_only:
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
    line = "straight line"
    if spiral is not None and np.linalg.norm(axis) < MIN_AXIS:
        spiral, line = None, "straight line: the planned insert has no travel to spiral around"
    screws = " + ".join(mating)
    if grounded:
        text = (f"RIGID insert of a ground bar, no tighten, {rigid.duration:.1f} s, largest joint travel "
                f"{math.degrees(rigid.travel):.1f}°.")
    elif rigid_only:
        text = (f"RIGID-ONLY insert: the whole {total:.1f} mm under joint control ({rigid.duration:.1f} s); the joint "
                f"screw of {screws} TIGHTENS from the start; the arms hold the end until it stalls "
                f"(≤ {HOLD_FOR_STALL:.0f} s), then STOP. Compliance is never engaged; the split and the spiral are "
                f"ignored. {WATCH}")
    else:
        if spiral is not None:
            line = (f"spiral {SPIRAL_RADIUS * 1e3:.0f} mm, {spiral[0]:g} s, {spiral[1]} turns, pitch "
                    f"{SPIRAL_RADIUS * 1e3 / spiral[1]:.3g} mm")
        text = (f"INSERT: rigid {total - achieved:.1f} mm ({0.0 if rigid is None else rigid.duration:.1f} s), then "
                f"COMPLIANT {achieved:.1f} mm ({line}); the joint screw of {screws} TIGHTENS from the start until it "
                f"stalls (≤ {HOLD_FOR_STALL:.0f} s), then STOP and back to joint control. {WATCH}")
    return InsertPlan(serial, arms, names, rows, tuple(mating), grounded, rigid_only, float(split_mm), achieved, total,
                      rigid, compliant_s, ends, axis, spiral, text)


async def run_insert(ctx: PluginContext, robot: HuskyRobotInterface, plan: InsertPlan,
                     tools: Mapping[str, ScaffoldingV3], *, operator: Operator, dry_run: bool,
                     watch: Callable[[], str] | None = None, recorder: Recorder | None = None) -> PartOutcome:
    """Run an insert (port of W:5268-5775): checks, the screws, the rigid part, then the hold or the compliant part.

    - Refused, nothing sent, unless every check of `part_problem` passes.
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
    record: dict = {"plan": plan.as_record(),
                    "fk_tcp_gap": {name: gap_record(fk_tcp_gap(robot, name)) for name in plan.arms}}
    problem = part_problem(ctx, robot, plan.arms, tools)
    if problem:
        ctx.log_warn(f"{label} refused: {problem}; nothing sent")
        return PartOutcome(REFUSED, f"{problem}; nothing sent", record=record)
    if plan.rigid_only:
        ctx.log_info(f"{label}: RIGID-ONLY: the whole path under joint control; the split and the spiral are ignored")
    if dry_run:
        return await _dry_run(ctx, robot, plan, record)
    started, husky = ctx.now(), robot.husky_in_world()
    recorder = Recorder(robot, started) if recorder is None else recorder
    phases: list[dict] = []
    record.update(phases=phases, switch={})
    gate: StallGate | None = None

    def phase(name: str, since: float, end: str) -> None:
        """Keep a phase in the record, its times from the insert's start."""
        phases.append({"name": name, "t0": round(since - started, 3), "t1": round(ctx.now() - started, 3),
                       "end": end})

    def stalled() -> bool:
        """Keep a sample, then say whether every mating screw stalled: the rigid part ends early then."""
        recorder.tick(ctx.now())
        return gate.all_stalled()

    async def steps() -> tuple[str, str]:
        """Clear and start the screws, the rigid part, then the hold or the compliant part; how it ended."""
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
            gap = record["end_gap_mm"] = _end_gap(robot, plan) * 1e3
            ctx.log_info(f"{plan.serial}: servo after the insert: M7, not run")
            return RIGID_DONE, f"ground bar in, no tighten; the flanges stopped {gap:.1f} mm from the planned end"
        if plan.rigid_only:
            if sent.status == ENDED:
                return STALLED, f"{gate.text} STALLED during the motion"
            since, recorder.phase = ctx.now(), "hold"
            held = await _hold_for_stall(ctx, gate, recorder)
            phase("hold", since, "stalled" if held else "no stall")
            if held:
                return STALLED, f"{gate.text} STALLED; the rigid-only hold is complete"
            return NO_STALL, f"no {gate.text} stall within {HOLD_FOR_STALL:.0f} s of the end, held under joint control"
        since = ctx.now()
        switch = await switch_arms(ctx, arms, CARTESIAN_COMPLIANCE_CONTROLLER)
        record["switch"].update(to_s=switch.seconds, ok=switch.ok)
        phase("switch", since, switch.reason)
        if not switch.ok:
            return FAILED, f"{switch.reason}; no target sent"
        refused = zero_wrench(arms)
        if refused:
            return FAILED, f"{refused}; no target sent"
        since = ctx.now()
        ended = await _compliant(ctx, robot, plan, husky, gate, operator, recorder, record)
        phase("compliant" if plan.spiral is None else "spiral", since, ended)
        if ended == "stalled":
            return STALLED, f"{gate.text} STALLED"
        if ended == "no_stall":
            return NO_STALL, f"no {gate.text} stall within {HOLD_FOR_STALL:.0f} s of the line's end"
        if ended == "cancelled":
            return CANCELLED, "Cancel at the spiral's end"
        return FAILED, f"compliant part: {ended}"

    try:
        status, reason = await steps()
    finally:
        try:
            stop_all(tools)
        finally:
            since = ctx.now()
            restored = await leave_compliance(ctx, arms, label=label)
            record["switch"].update(back_s=ctx.now() - since, back_ok=restored)
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
    """Build the rigid part's trajectory, and check every compliant target as the arm would; send nothing.

    The compliant line starts at tool0 at the handover row; each target is checked against the one before it (where the
    TCP follows to), one MAX_CLOCK_STEP apart: the largest step the stream may make.
    """
    if plan.rigid is not None:
        built = await send_motion(ctx, robot, plan.rigid, dry_run=True)
        if built.status != SENT_DRY:
            return PartOutcome(REFUSED, f"dry run: rigid part: {built.reason}", record=record)
    if plan.compliant:
        husky = robot.husky_in_world()
        handover = dict(zip(plan.names, plan.rows[0])) if plan.rigid is None else {
            name: value for motion in plan.rigid.arms
            for name, value in zip(arm_joint_names(motion.arm), motion.positions[-1])}
        start = {name: compose(_arm_base(robot, name, husky), tool0_fk(robot.config.model.urdf, name, handover))
                 for name in plan.arms}
        end = _line_ends(robot, plan, husky)
        spiral = _spiral(plan, husky)
        if spiral is None:
            lines = [(Segment(start, end, plan.compliant_seconds, 0.0), None)]
        else:
            back = spiral.offset(spiral.seconds)
            out = {name: Pose.from_arrays(np.add(pose.position, back), pose.orientation) for name, pose in end.items()}
            lines = [(Segment(start, end, spiral.seconds, 0.0), spiral.offset),
                     (Segment(out, end, SPIRAL_RETURN, 0.0), None)]
        previous: dict[str, Pose] = {}
        for segment, lateral in lines:
            for k, goals in enumerate(plan_stream(segment, MAX_CLOCK_STEP, lateral)):
                for name, goal in goals.items():
                    arm = robot.arms[name]
                    target = Pose.from_arrays(*arm.from_husky(*robot.from_world(goal.position, goal.orientation)))
                    problem = target_problem(previous.get(name), target, previous.get(name, target))
                    if problem:
                        return PartOutcome(REFUSED, f"dry run: {name}'s compliant target at {k * MAX_CLOCK_STEP:.1f} "
                                                    f"s: {problem}", record=record)
                    previous[name] = target
        ctx.log_warn(f"DRY RUN {plan.serial}: insert: every compliant target passed the arm's checks, NOT SENT")
    return PartOutcome(DRY_RUN, "every check passed; nothing sent (dry run)", record=record)


async def _hold_for_stall(ctx: PluginContext, gate: Gate, recorder: Recorder) -> bool:
    """Wait while the joint controller holds the end, until every mating screw stalls (W:4928-4958).

    Returns:
        bool: True once they stalled; False after HOLD_FOR_STALL.
    """
    held = ctx.now()
    while not gate.all_stalled():
        if ctx.now() - held >= HOLD_FOR_STALL:
            return False
        recorder.tick(ctx.now())
        await ctx.next_tick()
    return True


async def _compliant(ctx: PluginContext, robot: HuskyRobotInterface, plan: InsertPlan, husky: Pose, gate: Gate,
                     operator: Operator, recorder: Recorder, record: dict) -> str:
    """Stream the compliant part from each measured TCP to its end, straight or spiralling, until the screws stall.

    The arms run the compliance controller. Keeps the spiral's state and each arm's end error in `record`.

    Returns:
        str: "stalled", "no_stall", "cancelled" (the operator at the spiral's end) or why a stream stopped.
    """
    start = {name: measured_tool0(robot, name) for name in plan.arms}
    if any(pose is None for pose in start.values()):
        return "no measured TCP at the handover"
    segment = Segment(start, _line_ends(robot, plan, husky), plan.compliant_seconds, HOLD_FOR_STALL)
    spiral = _spiral(plan, husky)
    since = ctx.now()
    if spiral is None:
        recorder.phase = "compliant"
        ended = await stream(ctx, robot, segment, keep_going=lambda _elapsed: not gate.all_stalled(),
                             recorder=recorder)
        ended = {"ended": "stalled", "time": "no_stall"}.get(ended, ended)
    else:
        ended = await run_spiral(ctx, robot, segment, spiral, gate, operator, recorder)
        record["spiral_text"] = spiral.text(ctx.now() - since)
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


def _end_gap(robot: HuskyRobotInterface, plan: InsertPlan) -> float:
    """The furthest any moving arm's measured TCP is from its planned end, metres; infinite without a TCP."""
    gaps = [math.inf if robot.arms[name].state.tcp_position is None else
            float(np.linalg.norm(np.subtract(robot.arms[name].state.tcp_position, plan.ends[name].position)))
            for name in plan.arms]
    return max(gaps)


def _arm_base(robot: HuskyRobotInterface, name: str, husky: Pose) -> Pose:
    """An arm's base_link in the world, with the husky frame at `husky`."""
    return compose(husky, robot.arms[name].base_in_husky)


def _line_ends(robot: HuskyRobotInterface, plan: InsertPlan, husky: Pose) -> dict[str, Pose]:
    """Each arm's planned end in the world, with the husky frame at `husky`: E_s = B·H·A_s·FK_s(q_N) (§0.3)."""
    return {name: compose(_arm_base(robot, name, husky), plan.ends[name]) for name in plan.arms}


def _spiral(plan: InsertPlan, husky: Pose) -> Spiral | None:
    """The plan's spiral around its axis turned into the world with the husky frame at `husky`, or None."""
    if plan.spiral is None:
        return None
    return Spiral(Rotation.from_quat(husky.orientation).apply(plan.axis), *plan.spiral)
