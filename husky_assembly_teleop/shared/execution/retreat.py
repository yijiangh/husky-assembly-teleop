"""
The retreat: a bar's tools let go of it and back out of their joints, stiff (the default) or compliant first.

Stiff (`plan_stiff_retreat`, `run_stiff_retreat`; the old monitor's default since 2026-10-28, W:5645-5732): under the
joint controller the whole way, both gripper screws LOOSEN while the tools back straight out the first 12 mm of the
planned path at the screws' own rate (2.4 mm/s), so the screws neither push nor pull the bar, then the rest of the path
runs rigid; the path checks only report. Compliant (`plan_retreat`, `run_retreat`; the run setting "Retreat compliant",
kept for tests): the gripper screws open, each tool0 is streamed under the compliance controller from where it is to
the split point, then back under joint control the rest is replanned from where the tools ended and runs at once
(W:5268-5775); in the lab the compliant tools pushed the bar away (2026-10-27).

- ! Every ending, a cancel included, stops every tool and (the compliant variant) brings the arms back under joint
  control.
- ! The replan comes from the caller (`run_retreat`'s `replan`): this layer never imports a plugin.
- ! Main thread, from a plugin task.
"""

from __future__ import annotations

import asyncio
import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Awaitable, Callable, Collection, Mapping, Sequence

import numpy as np

from scaffolding_assembly_core.geometry import Pose, compose, invert
from scaffolding_assembly_core.ids import split_link_id
from scaffolding_assembly_core.robot import FLANGE_SUFFIX

from ...plugin_api.concurrency import WaitTimeout
from ...robot_interface.arm import (CARTESIAN_COMPLIANCE_CONTROLLER, MOVING_WITHIN,
                                    SCALED_JOINT_TRAJECTORY_CONTROLLER)
from ...robot_interface.end_effectors import ScaffoldingV3
from ...robot_interface.soft_stop import MotionRefused
from ..assembly_planning.joints import arm_joint_names
from ..assembly_planning.route_result import pose_error
from .controllers import leave_compliance, switch_arms
from .precheck import part_problem
from .send import HOLD_NOT_SENT as SENT_NOT_HELD
from .send import motion_command, send_motion
from .stream import (ARRIVAL, DRY_RUN, END_WAIT, FAILED, HOLD_NOT_SENT, MAX_CLOCK_STEP, REFUSED, RETREATED,
                     PartOutcome, Recorder, Segment, arm_base_in_world, fk_tcp_gap, gap_record, measured_tool0,
                     plan_stream, progress, split_rows, split_seconds, stream, target_problem, tool0_fk, zero_wrench)
from .tools import stop_all, stop_and_clear

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext
    from ...robot_interface.arm import ArmInterface
    from ...robot_interface.robot import HuskyRobotInterface
    from ..assembly_planning.route_result import RouteResult

# * The gripper screws (W:5403-5427): the thread's pitch, how much of it is engaged at a full grasp, how much further
#   the tools back out so the thread is clear, and the motor's rate (measured 2026-10-28).
SCREW_PITCH_MM = 2.0
SCREW_ENGAGEMENT_MM = 8.0
SCREW_OUT_MARGIN_MM = 4.0
SCREW_RPM = 72.0
#: How far the tools back out while the screws loosen, and how fast: 12 mm at 2.4 mm/s = 5.0 s. No faster (the tools
#: would pull the bar), no slower (the screws would push it away).
SCREW_OUT_MM = SCREW_ENGAGEMENT_MM + SCREW_OUT_MARGIN_MM
SCREW_RATE_MM_S = SCREW_PITCH_MM * SCREW_RPM / 60.0
#: The live joints may be this far from a stiff retreat's planned start, radians (W:5737): no jump holding a bar.
START_TOL = 0.01
#: Seconds to wait for the arms to stand still between the screw-out and the rest: an arm refuses a trajectory within
#: MOVING_WITHIN of its last motion.
STILL_WAIT = MOVING_WITHIN + 1.0
#: The tail's replanned line moves in steps of this, metres and radians (M:257-258).
TAIL_STEP = 0.001
TAIL_ANGLE = 0.05
#: Seconds the force sensors get after their zero, before the break-out.
FT_SETTLE = 0.5

# * The retreat checks (W:5056-5065): a report before the tail, which runs whatever they say.
#: The tail's first row may be this far from the live joints, radians.
CHECK_START = 0.01
#: After the break-out every tool0 is this close to its end, and the arms travelled this alike, metres.
CHECK_ARRIVAL = 0.003
CHECK_SPREAD = 0.003
#: The largest force left on an arm after the break-out, newtons.
CHECK_FORCE = 20.0
#: Seconds to wait for a gripper screw still opening before its check.
CHECK_GRIPPER_WAIT = 3.0
#: Along the tail, per arm: its length against the distance to its goal, and its end to the goal, metres; how far it
#: comes back towards the structure, metres; how far tool0 turns, radians.
CHECK_LENGTH = 0.005
CHECK_BACKTRACK = 0.002
CHECK_TURN = math.radians(3.0)
#: The furthest one joint travels along the tail, radians; the fastest a tool0 moves on it, metres per second.
CHECK_JOINT_TRAVEL = 0.35
CHECK_SPEED = 0.05

GRIPPER_MOTOR = ScaffoldingV3.GRIPPER_MOTOR


@dataclass(frozen=True)
class RetreatPlan:
    """A retreat worked out from its planned path: the compliant break-out from the start, then the rigid tail.

    Attributes:
        serial: The robot.
        arms: The moving arms, in the robot's arm order.
        split_mm: The split asked for: the first millimetres compliant.
        achieved_mm: Where the path was cut, millimetres from its end: the tail's length.
        total_mm: The path's length: the furthest any arm's tool0 travels.
        compliant_seconds: The break-out's seconds; 0 without one (split under 0.5 mm).
        tail_seconds: The tail's seconds; 0 without one (the break-out runs to the path's end).
        ends: Arm name -> its tool0 at the cut, in its base_link: the break-out's end; empty without one.
        goals: Arm name -> the world pose its tool0 retreats to, from the plan; the tail's checks measure against it.
        text: The prompt after "<robot> (<serial>): <movement> — ".
    """

    serial: str
    arms: tuple[str, ...]
    split_mm: float
    achieved_mm: float
    total_mm: float
    compliant_seconds: float
    tail_seconds: float
    ends: Mapping[str, Pose]
    goals: Mapping[str, Pose]
    text: str

    @property
    def compliant_mm(self) -> float:
        """float: The break-out's length, millimetres."""
        return self.total_mm - self.achieved_mm

    def as_record(self) -> dict:
        """The plan as JSON values."""
        return {"serial": self.serial, "arms": list(self.arms), "split_mm": self.split_mm,
                "achieved_mm": self.achieved_mm, "total_mm": self.total_mm, "compliant_mm": self.compliant_mm,
                "compliant_s": self.compliant_seconds, "tail_s": self.tail_seconds}


def plan_retreat(robot: HuskyRobotInterface, names: Sequence[str], rows: Sequence[Sequence[float]],
                 goals: Mapping[str, Pose], *, split_mm: float, traj_time: float) -> RetreatPlan | str:
    """Work out a retreat from its planned path: where the break-out ends, each part's time, its prompt (§0.2).

    The break-out is the first `split_mm` (cut as the insert's split, counted from the start); under 0.5 mm there is
    none, and past the path's end there is no tail.

    Args:
        robot: The robot; its URDF gives each arm's tool0.
        names: The path's joint names.
        rows: The path (N x joints); row 0 is where the arms are.
        goals: Process flange id -> the world pose its tool0 was planned to end at (`RouteResult.goals`).
        split_mm: The run setting: the first millimetres compliant.
        traj_time: Seconds for the whole path.

    Returns:
        RetreatPlan | str: The plan, or why not (no arm or tool moves, a moving arm without its goal).
    """
    serial, urdf = robot.config.serial, robot.config.model.urdf
    rows = np.asarray(rows, dtype=float)
    arms = tuple(arm for arm in robot.arms if set(arm_joint_names(arm)) <= set(names))
    if not arms:
        return f"the path moves no arm of {serial}"

    def tool0(row: np.ndarray) -> dict[str, Pose]:
        """Each moving arm's tool0 at a row, in its base_link."""
        joints = dict(zip(names, row))
        return {arm: tool0_fk(urdf, arm, joints) for arm in arms}

    leading, trailing, achieved, total = split_rows(
        rows, lambda row: [pose.position for pose in tool0(row).values()], split_mm, from_start=True)
    if total <= 0.0:
        return "the path does not move the tools"
    by_arm = {split_link_id(flange)[1].removesuffix(FLANGE_SUFFIX): pose for flange, pose in goals.items()}
    missing = [arm for arm in arms if arm not in by_arm]
    if trailing is not None and missing:
        return f"no retreat goal for {', '.join(missing)} in the plan: Plan again"
    compliant_s, tail_s = split_seconds(traj_time, achieved, total, from_start=True)
    compliant_s, tail_s = (0.0 if leading is None else compliant_s), (0.0 if trailing is None else tail_s)
    steps = [f"the M1 screws of {' + '.join(arms)} OPEN"]
    if leading is not None:
        steps += ["force sensors zeroed", f"COMPLIANT break-out {total - achieved:.1f} mm ({compliant_s:.1f} s)",
                  "joint control"]
    if trailing is not None:
        steps.append(f"the rest replanned from where the tools are and run at once ({tail_s:.1f} s)")
    both = len(arms) > 1
    text = (f"RETREAT: {'; '.join(steps)}. Watch {'both tools' if both else 'the tool'} let go and back off along "
            f"{'their lines' if both else 'its line'}.")
    ends = {} if leading is None else tool0(leading[-1])
    return RetreatPlan(serial, arms, float(split_mm), achieved, total, compliant_s, tail_s, ends,
                       {arm: by_arm[arm] for arm in arms if arm in by_arm}, text)


def retreat_checks(*, names: Sequence[str], tail: Sequence[Sequence[float]], live: Mapping[str, float],
                   reached: Mapping[str, Mapping[str, float] | None] | None, forces: Mapping[str, float | None],
                   grippers: Mapping[str, str | None], paths: Mapping[str, Sequence[Pose]],
                   goals: Mapping[str, Pose], seconds: float, path_only: bool = False) -> list[str]:
    """The retreat checks before its tail, one "PASS …" or "FAIL …" line each (W:5068-5199); a report only.

    ! A check without its measurement fails. With `path_only` (the stiff retreat, checked before anything is sent)
      only each arm's path, the joints and the speed are checked.

    Args:
        names: The tail's joint names.
        tail: The tail (N x joints).
        live: The arms' joints now, by name.
        reached: Arm name -> how far the break-out got (`progress`), or None for an arm not measured; None without
            a break-out.
        forces: Arm name -> |F| now, newtons, or None without a reading.
        grippers: Arm name -> its gripper screw's state, or None without a status.
        paths: Arm name -> its tool0 at each row of the tail, world.
        goals: Arm name -> its tool0's goal, world.
        seconds: The tail's duration.

    Returns:
        list[str]: The lines, in order: start, break-out, contact, grippers, each arm's path, joints, speed.
    """
    lines = []

    def note(ok: bool, text: str) -> None:
        """Keep one check's line."""
        lines.append(f"{'PASS' if ok else 'FAIL'} {text}")

    tail = np.asarray(tail, dtype=float)
    gap = max(abs(value - live[name]) for name, value in zip(names, tail[0]))
    if not path_only:
        note(gap <= CHECK_START, f"start: first row {gap:.4f} rad from the live joints (limit {CHECK_START})")
    if reached is not None and not path_only:
        if any(each is None for each in reached.values()):
            note(False, "break-out: a tool0 was not measured")
        else:
            worst = max(each["remaining_mm"] for each in reached.values())
            done = [each["achieved_mm"] for each in reached.values()]
            note(worst <= CHECK_ARRIVAL * 1e3 and max(done) - min(done) <= CHECK_SPREAD * 1e3,
                 f"break-out: worst tool0 {worst:.1f} mm short of its end (limit {CHECK_ARRIVAL * 1e3:.0f}), "
                 f"travelled {' / '.join(f'{value:.1f}' for value in done)} mm (spread limit "
                 f"{CHECK_SPREAD * 1e3:.0f})")
    if not path_only:
        readings = ", ".join(f"{arm} {_newtons(force)}" for arm, force in forces.items())
        note(all(force is not None and force <= CHECK_FORCE for force in forces.values()),
             f"contact: |F| {readings} (limit {CHECK_FORCE:.0f} N)")
        states = ", ".join(f"{arm} {state or 'no status'}" for arm, state in grippers.items())
        note(all(state == ScaffoldingV3.STALLED for state in grippers.values()),
             f"grippers: {states} (want {ScaffoldingV3.STALLED} = fully open)")
    lengths = []
    for arm, path in paths.items():
        points, goal = np.array([pose.position for pose in path]), np.asarray(goals[arm].position)
        length = float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))
        remaining, to_goal = float(np.linalg.norm(goal - points[0])), float(np.linalg.norm(goal - points[-1]))
        # * How far a later row falls behind the furthest one so far, along start -> goal.
        ahead = (points - points[0]) @ ((goal - points[0]) / remaining if remaining > 1e-9 else np.zeros(3))
        backtrack = float(np.max(np.maximum.accumulate(ahead) - ahead))
        turn = max(pose_error(path[0], pose)[1] for pose in path)
        lengths.append(length)
        note(abs(length - remaining) <= CHECK_LENGTH and to_goal <= CHECK_LENGTH and backtrack <= CHECK_BACKTRACK
             and turn <= CHECK_TURN,
             f"{arm} tool0 path: {length * 1e3:.1f} mm for {remaining * 1e3:.1f} mm to the goal, ends "
             f"{to_goal * 1e3:.1f} mm from it (limits {CHECK_LENGTH * 1e3:.0f} mm), backtracks "
             f"{backtrack * 1e3:.1f} mm (limit {CHECK_BACKTRACK * 1e3:.0f}), turns {math.degrees(turn):.1f}° (limit "
             f"{math.degrees(CHECK_TURN):.0f})")
    travel = np.sum(np.abs(np.diff(tail, axis=0)), axis=0)
    furthest = int(np.argmax(travel))
    note(travel[furthest] <= CHECK_JOINT_TRAVEL, f"joints: largest total travel {travel[furthest]:.3f} rad on "
                                                 f"{names[furthest]} (limit {CHECK_JOINT_TRAVEL})")
    speed = max(lengths, default=0.0) / seconds
    note(speed <= CHECK_SPEED, f"speed: tool0 {speed * 1e3:.1f} mm/s over {seconds:.1f} s (limit "
                               f"{CHECK_SPEED * 1e3:.0f})")
    return lines


@dataclass(frozen=True)
class StiffRetreatPlan:
    """A stiff retreat worked out from its planned path: the screw-out lead, the rigid rest, its prompt.

    Attributes:
        serial: The robot.
        arms: The moving arms, in the robot's arm order.
        names: The path's joint names.
        lead: The first SCREW_OUT_MM of the path (rows), row 0 where the arms are.
        rest: The rest of the path (rows), its first row the cut.
        screw_out_mm: The lead's length (a touch over 12 mm along a bent line).
        screw_out_seconds: The lead's time at SCREW_RATE_MM_S.
        rest_mm: The rest's length, millimetres.
        rest_seconds: The rest's share of the operator's time.
        total_mm: The path's length: the furthest any arm's tool0 travels.
        goals: Arm name -> the world pose its tool0 retreats to, from the plan; the checks measure against it.
        text: The prompt after "<robot> (<serial>): <movement> — ".
    """

    serial: str
    arms: tuple[str, ...]
    names: tuple[str, ...]
    lead: np.ndarray
    rest: np.ndarray
    screw_out_mm: float
    screw_out_seconds: float
    rest_mm: float
    rest_seconds: float
    total_mm: float
    goals: Mapping[str, Pose]
    text: str

    def as_record(self) -> dict:
        """The plan as JSON values."""
        return {"serial": self.serial, "arms": list(self.arms), "stiff": True, "screw_out_mm": self.screw_out_mm,
                "screw_out_s": self.screw_out_seconds, "rate_mm_s": SCREW_RATE_MM_S, "rest_mm": self.rest_mm,
                "rest_s": self.rest_seconds, "total_mm": self.total_mm}


def plan_stiff_retreat(robot: HuskyRobotInterface, names: Sequence[str], rows: Sequence[Sequence[float]],
                       goals: Mapping[str, Pose], *, traj_time: float) -> StiffRetreatPlan | str:
    """Work out a stiff retreat from its planned path: the 12 mm screw-out lead at the screw rate, the rest rigid.

    Args:
        robot: The robot; its URDF gives each arm's tool0.
        names: The path's joint names.
        rows: The path (N x joints); row 0 is where the arms are.
        goals: Process flange id -> the world pose its tool0 was planned to end at (`RouteResult.goals`).
        traj_time: Seconds the operator asked for the whole path; the rest takes its share by distance.

    Returns:
        StiffRetreatPlan | str: The plan, or why not: no arm moves, the path not longer than the screw-out (W:5663),
        a moving arm without its goal.
    """
    serial, urdf = robot.config.serial, robot.config.model.urdf
    rows = np.asarray(rows, dtype=float)
    arms = tuple(arm for arm in robot.arms if set(arm_joint_names(arm)) <= set(names))
    if not arms:
        return f"the path moves no arm of {serial}"

    def tool0(row: np.ndarray) -> dict[str, Pose]:
        """Each moving arm's tool0 at a row, in its base_link."""
        joints = dict(zip(names, row))
        return {arm: tool0_fk(urdf, arm, joints) for arm in arms}

    lead, rest, achieved, total = split_rows(
        rows, lambda row: [pose.position for pose in tool0(row).values()], SCREW_OUT_MM, from_start=True)
    if total <= 0.0:
        return "the path does not move the tools"
    if rest is None or lead is None:
        return (f"stiff screw-out: the planned retreat ({total:.0f} mm) is not longer than the {SCREW_OUT_MM:.0f} mm "
                f"screw-out: Plan again")
    by_arm = {split_link_id(flange)[1].removesuffix(FLANGE_SUFFIX): pose for flange, pose in goals.items()}
    missing = [arm for arm in arms if arm not in by_arm]
    if missing:
        return f"no retreat goal for {', '.join(missing)} in the plan: Plan again"
    screw_out_mm = total - achieved
    seconds = screw_out_mm / SCREW_RATE_MM_S
    rest_seconds = split_seconds(traj_time, achieved, total, from_start=True)[1]
    both = len(arms) > 1
    tools = "both tools" if both else "the tool"
    text = (f"RETREAT, stiff: the M1 screws of {' + '.join(arms)} LOOSEN while {tools} back straight out "
            f"{screw_out_mm:.1f} mm at {SCREW_RATE_MM_S:.2f} mm/s ({seconds:.1f} s, joint controller), then "
            f"{achieved:.0f} mm rigid ({rest_seconds:.1f} s). Watch {tools} let go and back off along "
            f"{'their lines' if both else 'its line'}.")
    return StiffRetreatPlan(serial, arms, tuple(names), lead, rest, screw_out_mm, seconds, achieved, rest_seconds,
                            total, {arm: by_arm[arm] for arm in arms}, text)


async def run_stiff_retreat(ctx: PluginContext, robot: HuskyRobotInterface, plan: StiffRetreatPlan,
                            tools: Mapping[str, ScaffoldingV3], *, dry_run: bool = False,
                            watch: Callable[[], str] | None = None, recorder: Recorder | None = None) -> PartOutcome:
    """Run a stiff retreat (W:5645-5732): both jaws LOOSEN as the lead goes out at the screw rate, then the rest.

    Refused, nothing sent, unless every check of `part_problem` passes; the path checks only report. The lead and the
    LOOSEN go out in one tick; the rest is sent once the arms stand still, from the joints then (over START_TOL from
    the cut: FAILED, nothing more sent). Every tool is STOPped at the end, however it ends; a cancel during the lead
    holds every arm and sends no rest.

    Args:
        ctx: The plugin's context.
        robot: The robot; its serial is the plan's.
        plan: The retreat (`plan_stiff_retreat`), its lead's row 0 at the live joints.
        tools: Arm name -> the scaffolding tool on each moving arm.
        dry_run: Build the lead's trajectory, every check, and send nothing.
        watch: Checked every tick while the arms move: why the retreat must stop (e.g. another robot moves), or "".
        recorder: Gets the samples (e.g. for a live readout), made at the start; None: its own.

    Returns:
        PartOutcome: RETREATED, REFUSED (nothing sent), FAILED, HOLD_NOT_SENT or DRY_RUN; its record: the plan, the
            checks, the phases "screw-out" and "rigid", the motor states as they changed, each send's outcome.

    Raises:
        asyncio.CancelledError: When cancelled; every arm was held and every tool stopped first.
    """
    label = f"{plan.serial}: retreat"
    arms = [robot.arms[name] for name in plan.arms]
    record = {"plan": plan.as_record(),
              "fk_tcp_gap": {name: gap_record(fk_tcp_gap(robot, name)) for name in plan.arms}}
    problem = part_problem(ctx, robot, plan.arms, tools)
    if problem:
        ctx.log_error(f"{label} REFUSED: {problem}; nothing sent")
        return _outcome(REFUSED, problem, 0.0, record)
    try:
        lead = motion_command(plan.serial, list(robot.arms), plan.names, plan.lead, plan.screw_out_seconds,
                              constant_speed=True)
    except ValueError as error:
        return _outcome(REFUSED, f"the screw-out can't be timed: {error}", 0.0, record)
    rows = np.vstack([plan.lead, plan.rest[1:]])
    lines = record["checks"] = _checks(robot, plan, tools, plan.names, rows, None,
                                       plan.screw_out_seconds + plan.rest_seconds, path_only=True)
    for line in lines:
        (ctx.log_info if line.startswith("PASS") else ctx.log_warn)(f"{label}: check {line}")
    failures = sum(line.startswith("FAIL") for line in lines)
    ctx.log_info(f"{label}: stiff screw-out: both jaws LOOSEN while both tools back straight out "
                 f"{plan.screw_out_mm:.1f} mm at {SCREW_RATE_MM_S:.2f} mm/s ({lead.duration:.1f} s, joint controller), "
                 f"then {plan.rest_mm:.0f} mm rigid ({plan.rest_seconds:.1f} s); "
                 + (f"{failures} check(s) FAILED on the planned path" if failures else "every check passed"))
    if dry_run:
        sent = await send_motion(ctx, robot, lead, dry_run=True)
        if not sent.ok:
            return _outcome(REFUSED, sent.reason, 0.0, record)
        ctx.log_warn(f"DRY RUN {label}: the screw-out passed every check, NOT SENT; the rest is built from where it "
                     f"ends when it runs")
        return _outcome(DRY_RUN, "every check passed; nothing sent (dry run)", 0.0, record)
    started = ctx.now()
    recorder = Recorder(robot, started) if recorder is None else recorder
    phases = record["phases"] = []
    states = record["motor_states"] = []
    short = {name: name.split("_")[0][0].upper() for name in plan.arms}

    def phase(name: str, since: float, end: str) -> None:
        """Keep a phase's times in the record, seconds from the start."""
        phases.append({"name": name, "t0": round(since - started, 3), "t1": round(ctx.now() - started, 3),
                       "end": end})

    def motors() -> dict[str, str | None]:
        """Each moving arm's gripper motor state."""
        return {name: tools[name].state.gripper_motor for name in plan.arms if name in tools}

    def motors_text(now: Mapping[str, str | None]) -> str:
        return "/".join(short[name] for name in now) + " " + "/".join(str(state) for state in now.values())

    def sample() -> str:
        """Keep a sample each tick, log the motors when their states change; why it must stop (`watch`), or ""."""
        recorder.tick(ctx.now())
        now = motors()
        if not states or states[-1]["states"] != now:
            states.append({"t": round(ctx.now() - started, 1), "states": dict(now)})
            ctx.log_info(f"{label}: screw-out t={ctx.now() - started:.1f}s: gripper motors {motors_text(now)}")
        return watch() if watch is not None else ""

    async def steps() -> tuple[str, str]:
        """Run every step; the status and the reason it ended with."""
        not_cleared = await stop_and_clear(ctx, tools, GRIPPER_MOTOR)
        if not_cleared is not None:
            return FAILED, f"{not_cleared.reason}; nothing loosened"
        # * The LOOSEN and the lead in one tick: the tools back out as the screws turn (W:5700-5712).
        try:
            for tool in tools.values():
                tool.open()
        except MotionRefused as refused:
            return FAILED, f"{refused}; not every gripper screw loosened"
        ctx.log_info(f"{label}: LOOSEN the gripper screws of {' + '.join(tools)}")
        since, recorder.phase = ctx.now(), "screw-out"
        try:
            sent = await send_motion(ctx, robot, lead, watch=sample)
        except asyncio.CancelledError:
            ctx.log_warn(f"{label}: cancelled during the screw-out: the rest of the way out is not sent; every arm "
                         f"held, the gripper motors stop now")
            raise
        phase("screw-out", since, sent.status)
        record["screw_out"] = {"rows": len(lead.arms[0].positions), "seconds": lead.duration,
                               "mm": plan.screw_out_mm, "outcome": sent.as_record()}
        if not sent.ok:
            status = HOLD_NOT_SENT if sent.status == SENT_NOT_HELD else FAILED
            return status, f"the screw-out ended {sent.status}: {sent.reason}; the rest is not sent"
        ctx.log_info(f"{label}: screw-out done after {ctx.now() - since:.1f} s (gripper motors "
                     f"{motors_text(motors())}, still loosening); now the rest of the way out.")

        def still() -> bool:
            """No arm moved within MOVING_WITHIN: the rest would be refused otherwise."""
            now = ctx.now()
            return all(arm.state.last_motion_time is None or now - arm.state.last_motion_time >= MOVING_WITHIN
                       for arm in arms)

        try:
            await ctx.wait_until(still, timeout_s=STILL_WAIT, description="the arms to stand still after the screw-out")
        except WaitTimeout:
            pass
        rest = np.array(plan.rest, dtype=float)
        live = {}
        for name, arm in robot.arms.items():
            live.update(zip(arm_joint_names(name), arm.joint_vector().tolist()))
        now = np.array([live[name] for name in plan.names])
        gap = float(np.max(np.abs(now - rest[0])))
        if gap > START_TOL:
            return FAILED, (f"after the screw-out the arms are {math.degrees(gap):.2f}° from the cut (over "
                            f"{math.degrees(START_TOL):.2f}°); the rest is not sent")
        rest[0] = now
        try:
            command = motion_command(plan.serial, list(robot.arms), plan.names, rest, plan.rest_seconds)
        except ValueError as error:
            return FAILED, f"the rest can't be sent: {error}"
        ctx.log_info(f"{label}: rigid chunk: {len(command.arms[0].positions)} rows over {command.duration:.1f} s")
        since, recorder.phase = ctx.now(), "rigid"
        sent = await send_motion(ctx, robot, command, watch=sample)
        phase("rigid", since, sent.status)
        record["rigid"] = {"rows": len(command.arms[0].positions), "seconds": command.duration, "mm": plan.rest_mm,
                           "outcome": sent.as_record()}
        if not sent.ok:
            status = HOLD_NOT_SENT if sent.status == SENT_NOT_HELD else FAILED
            return status, f"the rest ended {sent.status}: {sent.reason}"
        return RETREATED, (f"retreated {plan.total_mm:.0f} mm; {failures} check(s) FAILED on the planned path"
                           if failures else f"retreated {plan.total_mm:.0f} mm; every check passed")

    try:
        status, reason = await steps()
    finally:
        stop_all(tools)
        record["grippers_at_end"] = motors()
    # * A refused hold was said loudly by `send_motion`; once is enough.
    if status != HOLD_NOT_SENT:
        (ctx.log_info if status == RETREATED else ctx.log_error)(
            f"{label}: {status}: {reason}; the gripper motors stopped ({motors_text(motors())} at the end)")
    return _outcome(status, reason, ctx.now() - started, {**record, **recorder.as_record()})


async def run_retreat(ctx: PluginContext, robot: HuskyRobotInterface, plan: RetreatPlan,
                      tools: Mapping[str, ScaffoldingV3], *, replan: Callable[[], Awaitable[RouteResult | None]],
                      dry_run: bool = False, watch: Callable[[], str] | None = None,
                      recorder: Recorder | None = None) -> PartOutcome:
    """Run a retreat (W:5268-5775): open the gripper screws, break out compliantly, replan the rest and run it rigid.

    Refused, nothing sent, unless every check of `part_problem` passes; the checks before the tail only report. Not
    back under joint control, or no tail from the replan (A4): FAILED, nothing more is sent.

    Args:
        ctx: The plugin's context.
        robot: The robot; its serial is the plan's.
        plan: The retreat (`plan_retreat`), with row 0 at the live joints.
        tools: Arm name -> the scaffolding tool on each moving arm.
        replan: Plans the rest of the retreat from the live joints to the plan's goals in TAIL_STEP steps; None, or a
            result not solved, when it can't.
        dry_run: Run the checks and send nothing.
        watch: Checked every tick of the tail: why it must stop (e.g. another robot moves), or "".
        recorder: Gets the samples (e.g. for a live readout), made at the start; None: its own.

    Returns:
        PartOutcome: RETREATED, REFUSED (nothing sent), FAILED, HOLD_NOT_SENT (the tail's) or DRY_RUN; its record as
            the spec's §1.3.

    Raises:
        asyncio.CancelledError: When cancelled; every tool was stopped and the arms brought back first.
    """
    label = f"{plan.serial}: retreat"
    arms = [robot.arms[name] for name in plan.arms]
    record = {"plan": plan.as_record(),
              "fk_tcp_gap": {name: gap_record(fk_tcp_gap(robot, name)) for name in plan.arms}}
    problem = part_problem(ctx, robot, plan.arms, tools)
    if problem:
        ctx.log_error(f"{label} REFUSED: {problem}; nothing sent")
        return _outcome(REFUSED, problem, 0.0, record)
    ends = {name: compose(arm_base_in_world(robot, name), end) for name, end in plan.ends.items()}
    if dry_run:
        return _dry_run(ctx, robot, plan, ends, label, record)
    started = ctx.now()
    recorder = Recorder(robot, started) if recorder is None else recorder
    recorder.phase = "open"
    phases = record["phases"] = []
    #: Whether the arms may be under compliance, so the `finally` must bring them back.
    compliant = False

    def phase(name: str, since: float, end: str) -> None:
        """Keep a phase's times in the record, seconds from the start."""
        phases.append({"name": name, "t0": round(since - started, 3), "t1": round(ctx.now() - started, 3),
                       "end": end})

    async def steps() -> tuple[str, str]:
        """Run every step; the status and the reason it ended with."""
        nonlocal compliant
        not_cleared = await stop_and_clear(ctx, tools, GRIPPER_MOTOR)
        if not_cleared is not None:
            return FAILED, f"{not_cleared.reason}; nothing opened"
        try:
            for tool in tools.values():
                tool.open()
        except MotionRefused as refused:
            return FAILED, f"{refused}; not every gripper screw opened"
        ctx.log_info(f"{label}: the gripper screws of {' + '.join(tools)} OPEN")
        phase("open", started, "sent")
        if ends:
            since = ctx.now()
            record["ft_zero"] = await _zero_force_sensors(ctx, arms, label)
            phase("force zero", since, "done")
            since, compliant = ctx.now(), True
            failed = await _break_out(ctx, robot, plan, ends, recorder, record, label)
            phase("break-out", since, failed or "done")
            since, compliant = ctx.now(), False
            back = await leave_compliance(ctx, arms, label=label)
            phase("joint control", since, "back" if back else "not confirmed")
            record["switch"].update(back_s=ctx.now() - since, back_ok=back)
            if failed:
                return FAILED, failed
            if not back:
                return FAILED, (f"not back under {SCALED_JOINT_TRAJECTORY_CONTROLLER}, or a hold was refused (the "
                                f"log says); nothing more sent")
        if not plan.tail_seconds:
            return RETREATED, "the break-out ran to the retreat's end: no tail"
        since = ctx.now()
        route = await replan()
        phase("replan", since, "solved" if route is not None and route.ok else "failed")
        if route is None or not route.ok:
            why = "no answer" if route is None else route.result.reason
            return FAILED, f"the tail's replan failed ({why}); the arms hold where the break-out left them"
        trajectory = route.result.trajectory

        def settled() -> bool:
            """No gripper screw still opening, and no arm moved within MOVING_WITHIN (it would refuse the tail)."""
            now = ctx.now()
            return all(tool.state.gripper_motor != ScaffoldingV3.LOOSENING for tool in tools.values()) and all(
                arm.state.last_motion_time is None or now - arm.state.last_motion_time >= MOVING_WITHIN
                for arm in arms)

        try:
            await ctx.wait_until(settled, timeout_s=CHECK_GRIPPER_WAIT,
                                 description="the gripper screws to open and the arms to stand still")
        except WaitTimeout:
            pass
        try:
            command = motion_command(plan.serial, list(robot.arms), trajectory.joint_names, trajectory.positions,
                                     plan.tail_seconds)
        except ValueError as error:
            return FAILED, f"the tail can't be sent: {error}"
        lines = record["checks"] = _checks(robot, plan, tools, trajectory.joint_names, trajectory.positions,
                                           record.get("end_error_mm"), command.duration)
        for line in lines:
            (ctx.log_info if line.startswith("PASS") else ctx.log_warn)(f"{label}: check {line}")
        failures = sum(line.startswith("FAIL") for line in lines)
        ctx.log_info(f"{label}: running the tail now ({len(trajectory.positions)} rows, {command.duration:.1f} s); "
                     + (f"{failures} check(s) FAILED" if failures else "every check passed"))

        def sample() -> str:
            """Keep a sample of the tail each tick; why it must stop (`watch`), or ""."""
            recorder.tick(ctx.now())
            return watch() if watch is not None else ""

        since, recorder.phase = ctx.now(), "tail"
        sent = await send_motion(ctx, robot, command, watch=sample)
        phase("tail", since, sent.status)
        record["tail"] = {"planner": route.planner, "waypoints": route.waypoints, "seconds": command.duration,
                          "outcome": sent.as_record()}
        if not sent.ok:
            status = HOLD_NOT_SENT if sent.status == SENT_NOT_HELD else FAILED
            return status, f"the tail ended {sent.status}: {sent.reason}"
        return RETREATED, (f"retreated; {failures} check(s) FAILED" if failures else "retreated; every check passed")

    try:
        status, reason = await steps()
    finally:
        try:
            stop_all(tools)
        finally:
            if compliant:
                await leave_compliance(ctx, arms, label=label)
    # * A refused hold was said loudly by `send_motion`; once is enough.
    if status != HOLD_NOT_SENT:
        (ctx.log_info if status == RETREATED else ctx.log_error)(f"{label}: {status}: {reason}")
    return _outcome(status, reason, ctx.now() - started, {**record, **recorder.as_record()})


async def _zero_force_sensors(ctx: PluginContext, arms: Collection[ArmInterface], label: str) -> dict:
    """Zero every arm's force sensor and give it FT_SETTLE (W:5203-5265); |F| before and after, newtons, per arm.

    Zeroed on purpose while the tools still touch the bar: the break-out then reacts only to the forces that appear
    as it backs off, not to the insert's preload. A zero that can't be sent is logged; the break-out goes on.
    """
    before = {arm.config.name: _force(arm) for arm in arms}
    for arm in arms:
        if not arm.zero_ft_sensor():
            ctx.log_warn(f"{label}: {arm.config.name}'s force sensor not zeroed; the break-out reacts to its reading "
                         f"as it is")
    await ctx.sleep(FT_SETTLE)
    after = {arm.config.name: _force(arm) for arm in arms}
    ctx.log_info(f"{label}: force sensors zeroed: |F| " + ", ".join(
        f"{name} {_newtons(before[name])} -> {_newtons(after[name])}" for name in before))
    return {name: [before[name], after[name]] for name in before}


async def _break_out(ctx: PluginContext, robot: HuskyRobotInterface, plan: RetreatPlan, ends: Mapping[str, Pose],
                     recorder: Recorder, record: dict, label: str) -> str:
    """Switch to compliance, zero the target wrench, stream every tool0 from its TCP to its end; why it failed, or "".

    The stream runs the plan's compliant seconds, then on until every tool0 is within ARRIVAL of its end, at most
    END_WAIT more (W:5509-5522); not arriving only warns.
    """
    arms = [robot.arms[name] for name in plan.arms]
    switched = await switch_arms(ctx, arms, CARTESIAN_COMPLIANCE_CONTROLLER)
    record["switch"] = {"to_s": switched.seconds, "ok": switched.ok}
    if not switched.ok:
        return switched.reason
    refused = zero_wrench(arms)
    if refused:
        return refused
    segment = Segment({name: measured_tool0(robot, name) for name in ends}, ends, plan.compliant_seconds, 0.0)

    def going(elapsed: float) -> bool:
        """On during the line's time, then until every tool0 arrived."""
        return elapsed < segment.seconds or not all(
            each is not None and each["remaining_mm"] <= ARRIVAL * 1e3 for each in progress(robot, segment).values())

    recorder.phase = "break-out"
    ended = await stream(ctx, robot, segment, keep_going=going, recorder=recorder)
    reached = record["end_error_mm"] = progress(robot, segment)
    ctx.log_info(f"{label}: break-out " + ", ".join(
        "no TCP" if each is None else f"{name} travelled {each['achieved_mm']:.1f} of {each['travel_mm']:.1f} mm, "
                                      f"{each['remaining_mm']:.1f} mm short" for name, each in reached.items()))
    if ended == "time":
        ctx.log_warn(f"{label}: the tools did not reach the break-out's end within {END_WAIT:.0f} s of its time")
    elif ended != "ended":
        return f"the break-out stopped: {ended}"
    return ""


def _checks(robot: HuskyRobotInterface, plan: RetreatPlan | StiffRetreatPlan, tools: Mapping[str, ScaffoldingV3],
            names: Sequence[str], tail: Sequence[Sequence[float]], reached: Mapping | None, seconds: float,
            path_only: bool = False) -> list[str]:
    """`retreat_checks` on the tail with what the robot and its tools measure now."""
    urdf, rows = robot.config.model.urdf, np.asarray(tail, dtype=float)
    live = {}
    for name, arm in robot.arms.items():
        live.update(zip(arm_joint_names(name), arm.joint_vector().tolist()))
    paths = {}
    for name in plan.arms:
        base = arm_base_in_world(robot, name)
        paths[name] = [compose(base, tool0_fk(urdf, name, dict(zip(names, row)))) for row in rows]
    return retreat_checks(names=names, tail=rows, live=live, reached=reached,
                          forces={name: _force(robot.arms[name]) for name in plan.arms},
                          grippers={name: tool.state.gripper_motor for name, tool in tools.items()},
                          paths=paths, goals=plan.goals, seconds=seconds, path_only=path_only)


def _dry_run(ctx: PluginContext, robot: HuskyRobotInterface, plan: RetreatPlan, ends: Mapping[str, Pose], label: str,
             record: dict) -> PartOutcome:
    """Check every break-out target as the arm would, at the stream's largest step, the TCP following; send nothing."""
    starts = {name: measured_tool0(robot, name) for name in ends}
    previous: dict[str, Pose] = {}
    for targets in plan_stream(Segment(starts, ends, plan.compliant_seconds, 0.0), MAX_CLOCK_STEP):
        for name, goal in targets.items():
            state = robot.arms[name].state
            target = compose(invert(arm_base_in_world(robot, name)), goal)
            tcp = previous.get(name, Pose.from_arrays(state.tcp_position, state.tcp_orientation))
            problem = target_problem(previous.get(name), target, tcp)
            if problem:
                ctx.log_error(f"DRY RUN {label}: {name} would refuse a break-out target: {problem}")
                return _outcome(REFUSED, f"{name} would refuse a break-out target: {problem}", 0.0, record)
            previous[name] = target
    ctx.log_warn(f"DRY RUN {label}: passed every check, NOT SENT; its tail is replanned when it runs")
    return _outcome(DRY_RUN, "every check passed; nothing sent (dry run)", 0.0, record)


def _outcome(status: str, reason: str, seconds: float, record: dict) -> PartOutcome:
    """The outcome, its record starting with the status, the reason and the seconds."""
    return PartOutcome(status, reason, seconds, {"status": status, "reason": reason, "seconds": seconds, **record})


def _force(arm: ArmInterface) -> float | None:
    """The arm's |F|, newtons; None without a reading."""
    wrench = arm.state.wrench
    return None if wrench is None else float(np.linalg.norm(wrench[:3]))


def _newtons(force: float | None) -> str:
    """A force for the log."""
    return "no reading" if force is None else f"{force:.1f} N"
