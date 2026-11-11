"""
The demo's parts on the robot, each from the live joints.

The steps of an action, the carry saved for it, a path from where the arms are, a path timed for the preview, the
check that they stand at the insert's start, the checks of the way back, the gripper screws, the compliant hold, and
the removal.

`run_unscrew_retreat` is the retreat with the bar kept in the tools: the joint screws run in reverse for a set time
under compliance, then the tools back out along the insert line to where the insert started. Built from the compliant
parts' blocks (`shared/execution/compliant_part.py`), as `retreat.run_retreat` is.

! The coroutines: main thread, from a plugin task, the robot leased.
"""

from __future__ import annotations

import asyncio
import math
import os
from dataclasses import replace
from functools import partial
from pathlib import Path
from typing import TYPE_CHECKING, Callable, Collection, Iterable, Mapping, Sequence

import numpy as np

from scaffolding_assembly_core.geometry import Pose, compose, invert
from scaffolding_assembly_core.ids import split_link_id
from scaffolding_assembly_core.kinematics import link_poses
from scaffolding_assembly_core.process import ProcessError, validate
from scaffolding_assembly_core.process.solutions import is_stale, read_solutions
from scaffolding_assembly_core.process.types import Process
from scaffolding_assembly_core.process.version import writer_info
# ? The core writes whole processes only; its one-action serialiser is private.
from scaffolding_assembly_core.process.write import _action, _write_json, _writer
from scipy.spatial.transform import Rotation

from ...plugin_api.concurrency import WaitTimeout
from ...robot_interface.end_effectors import ScaffoldingV3
from ...robot_interface.soft_stop import MotionRefused
from ...robot_interface.tool_step import tool_step_target
from ...shared.assembly_planning.joints import arm_joint_names
from ...shared.assembly_planning.movement import Routine, routine_or_none
from ...shared.assembly_planning.route_result import pose_error
from ...shared.execution.basis import STALE_ARM
from ...shared.execution.compliant_part import (check_part, enter_compliance, phase_keeper, started_part,
                                                stop_then_leave, tool0_by_arm)
from ...shared.execution.force_sensors import zero_force_sensors
from ...shared.execution.retime import RetimeError, retime
from ...shared.execution.stream import (ARRIVAL, FAILED, REFUSED, RETREATED, PartOutcome, Recorder, Segment,
                                        arm_base_in_world, measured_tool0, progress, stream)
from ...shared.execution.tools import MOTOR_NAMES, StallGate, stop_all, stop_and_clear
from ...shared.planner.timed_path import TimedPath
from ...world.tcp import tool0_in_base_link

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext
    from ...robot_interface.arm import ArmInterface
    from ...robot_interface.robot import HuskyRobotInterface
    from ...shared.assembly_planning.process_steps import LoadedProcess

#: Every tool0 this close to the insert's start, metres and radians, stands at it: the insert may run. The angle is
#: loose because the grasp twists about 1° over a compliant insert and retreat, and the way back steers one arm only.
AT_START = (0.005, math.radians(3.0))
#: Along the way back, one tool0 seen from the other may drift this far from where it starts: metres, radians.
COUPLING = (0.002, math.radians(1.0))
#: The largest step of a joint between two waypoints of the way back, radians.
JOINT_STEP = math.radians(5.0)
#: A start this near the authored one along the insert's line, metres, is the authored one.
SAME_LENGTH = 5e-5
#: The largest tool0 step solved at once along the insert's line, metres.
LINE_STEP = 0.005
#: What plans and saves an action's carry, for the refusals that ask for it.
CARRY_SCRIPT = "husky_assembly_tamp's scripts/plan_process_carry.py --goal joints --write-solution"
#: Seconds a gripper screw gets to stall after its close or open.
GRIP_TIMEOUT = 15.0
JOINT_MOTOR, GRIPPER_MOTOR = ScaffoldingV3.JOINT_MOTOR, ScaffoldingV3.GRIPPER_MOTOR
#: Each motor's verb and its past tense, tightening then loosening, for the log and the outcome line.
SCREW_VERBS = {GRIPPER_MOTOR: (("close", "closed"), ("open", "opened")),
               JOINT_MOTOR: (("tighten", "tightened"), ("loosen", "loosened"))}


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


def load_step(loaded: LoadedProcess, action_id: str) -> int | str:
    """The index into `loaded.steps` of an action's travel to load, the free move with empty tools; or why none."""
    action = loaded.process.actions[action_id]
    for index, movement in enumerate(action.movements):
        if routine_or_none(action, movement) is Routine.FREE_TO_LOAD:
            return loaded.index_of(action_id, index)
    return f"{action_id} has no travel to load"


def saved_carry(process: Process, action_id: str, movement_id: str) -> tuple[list[str], np.ndarray] | str:
    """The carry saved for an action's transfer, from the loading pose to the insert's start; or why there is none.

    Planned offline with the bar held and kept in the process folder's `solutions/<action id>.json` (CARRY_SCRIPT).

    Returns:
        tuple[list[str], np.ndarray] | str: (joint names, rows), the loading pose first; or the refusal: no file, a
            process or action changed since it was planned, no path for the movement.
    """
    try:
        solution = read_solutions(process).get(action_id)
        if solution is None:
            return f"no carry is saved for {action_id} (solutions/{action_id}.json): plan it with {CARRY_SCRIPT}"
        if is_stale(process, solution):
            return (f"the saved carry is stale: the process or {action_id} changed since it was planned (Load the "
                    f"process again, or plan the carry again with {CARRY_SCRIPT})")
    except (ProcessError, OSError, ValueError) as error:
        return f"the saved carry can't be read: {error}"
    result = solution.movements.get(movement_id)
    if result is None or result.status != "solved" or result.trajectory is None \
            or len(result.trajectory.positions) < 2:
        return f"solutions/{action_id}.json holds no path for {movement_id}: plan it with {CARRY_SCRIPT}"
    return list(result.trajectory.joint_names), np.array(result.trajectory.positions, dtype=float)


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


def back_out_rows(robot: HuskyRobotInterface, joints: Mapping[str, float],
                  arms: Iterable[str]) -> tuple[list[str], np.ndarray] | str:
    """The removal's back-out as a joint path for the preview: the arms' joints now, then `joints`; or why not.

    Returns:
        tuple[list[str], np.ndarray] | str: (names, two rows), or the refusal (an arm without a joint state).
    """
    names, now = [], []
    for arm in arms:
        here = robot.arms[arm].joint_vector()
        if here is None:
            return f"{arm}: no joint state yet"
        names += arm_joint_names(arm)
        now += here.tolist()
    return names, np.array([now, [joints[name] for name in names]], dtype=float)


def timed_preview(rows: Sequence[Sequence[float]], seconds: float,
                  limits: bool = True) -> tuple[TimedPath, float] | str:
    """A joint path timed for the preview, as its motion is timed when it runs; or why it can't be.

    Args:
        rows: The joint path (N x D), radians.
        seconds: The motion time asked for.
        limits: Time it as a joint motion (`retime`: the arms' limits may need longer); False: straight from the
            first row to the last in `seconds`, as the compliant back-out streams its line.

    Returns:
        tuple[TimedPath, float] | str: The timed path and the shortest time the arms' limits allow (NaN without
            `limits`); or the refusal.
    """
    rows = np.asarray(rows, dtype=float)
    if not limits:
        return TimedPath(np.array([0.0, float(seconds)]), rows[[0, -1]]), math.nan
    try:
        timed = retime(rows, seconds)
    except RetimeError as error:
        if np.array_equal(rows[0], rows[-1]):
            return "the arms are already there: nothing to move"
        return f"planned, but it can't be timed for the arms: {error}"
    return TimedPath(timed.times, timed.positions), timed.minimum


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


def line_length(urdf: Path, arm: str, start: Mapping[str, float], end: Mapping[str, float]) -> float:
    """Metres between an arm's tool0 at the joints `start` and at `end`: the insert's authored length."""
    return float(np.linalg.norm(np.subtract(tool0_in_base_link(urdf, arm, start).position,
                                            tool0_in_base_link(urdf, arm, end).position)))


def line_start(urdf: Path, arms: Iterable[str], start: Mapping[str, float], end: Mapping[str, float],
               distance: float) -> dict[str, float] | str:
    """The joints that put every arm's tool0 `distance` before the insert's end, on the line the authored start is on.

    Solved from the authored start in tool0 steps of LINE_STEP on the URDF, the orientation kept; no collision check.

    Args:
        urdf: The robot's URDF.
        arms: The insert's moving arms.
        start: The insert's authored start joints, by URDF name.
        end: Its authored end joints.
        distance: Metres from the end to the wanted start.

    Returns:
        dict[str, float] | str: `start` with the arms' joints there (the authored ones within SAME_LENGTH of the
            authored length); or why there are none (an arm too near a singularity on the way).
    """
    joints = dict(start)
    for arm in arms:
        names = arm_joint_names(arm)
        at_start, at_end = tool0_in_base_link(urdf, arm, start), tool0_in_base_link(urdf, arm, end)
        along = np.subtract(at_start.position, at_end.position)
        length = float(np.linalg.norm(along))
        if length < 1e-6:
            return f"{arm}: the insert's authored start and end coincide: no line to move along"
        more = distance - length
        if abs(more) <= SAME_LENGTH:
            continue
        # * The same shift in tool0's own frame all the way: the orientation never changes.
        unit = Rotation.from_quat(at_start.orientation).inv().apply(along / length)
        steps = math.ceil(abs(more) / LINE_STEP)
        values = np.array([start[name] for name in names], dtype=float)
        try:
            for _ in range(steps):
                values = tool_step_target(urdf, arm, values, Pose(tuple(unit * more / steps), (0.0, 0.0, 0.0, 1.0)))
        except ValueError as error:
            return f"{arm}: no joints {distance * 1e3:.0f} mm before the insert's end on its line: {error}"
        joints.update(zip(names, values.tolist()))
    return joints


def with_insert_start(process: Process, action_id: str, transfer_id: str, insert_id: str,
                      joints: Mapping[str, float], distance: float) -> Process:
    """Copy a process with the insert starting at `joints`, `distance` metres before its end; validated.

    The insert's start joints and line distances change, and the transfer's target with them (its joints, and its
    link poses by the URDF), so each movement still ends where the next starts.

    Raises:
        ProcessError: If the edited process breaks a format rule.
    """
    action = process.actions[action_id]
    robot, urdf = action.robot, process.robots[action.robot].urdf
    movements = []
    for movement in action.movements:
        if movement.id == insert_id:
            state = movement.start.robots[robot]
            start = replace(movement.start, robots={**movement.start.robots, robot: replace(
                state, joints={**(state.joints or {}), **joints})})
            movement = replace(movement, start=start, line={
                flange: replace(line, distance=float(distance)) for flange, line in movement.line.items()})
        elif movement.id == transfer_id and movement.target is not None:
            target, state = movement.target, movement.start.robots[robot]
            base = target.bases.get(robot, state.base)
            ends = {**(state.joints or {}), **target.joints.get(robot, {}), **joints}
            links = [link for link in target.link_poses if split_link_id(link)[0] == robot]
            poses = link_poses(urdf, base, ends, [split_link_id(link)[1] for link in links])
            movement = replace(movement, target=target.having(
                joints={**target.joints, robot: {**target.joints.get(robot, {}), **joints}},
                link_poses={**target.link_poses, **{link: poses[split_link_id(link)[1]] for link in links}}))
        movements.append(movement)
    edited = replace(process, actions={**process.actions, action_id: replace(action, movements=tuple(movements))})
    validate(edited, check_robot_meshes=False)
    return edited


def save_action(process: Process, action_id: str) -> Path:
    """Write one action of `process` over its file in the process's folder, as the core's writer would; the file.

    ! It overwrites an authored file (on the Drive, for everyone): only on the operator's own confirmed press.
    Nothing else in the folder is touched; the file is swapped in whole, never left half written.
    """
    path = Path(process.folder) / "actions" / f"{action_id}.json"
    scratch = path.with_name(f".{path.name}.tmp")
    _write_json(scratch, _action(process.actions[action_id], _writer(writer_info())))
    if path.is_file() and b"\r\n" in path.read_bytes():
        # * Keep the file's Windows line ends, so only the changed lines differ.
        scratch.write_bytes(scratch.read_bytes().replace(b"\n", b"\r\n"))
    os.replace(scratch, path)
    return path


def return_problem(robot: HuskyRobotInterface, names: Sequence[str], rows: Sequence[Sequence[float]],
                   arms: Sequence[str]) -> str:
    """Why a planned way back may not run, or "" if it may.

    The planner keeps the bar's grasp by construction; this checks that its IK did: along `rows` the second arm's
    tool0 seen from the first stays within COUPLING of where it starts, and no joint steps over JOINT_STEP between
    two waypoints.
    """
    rows = np.asarray(rows, dtype=float)
    if len(rows) < 2:
        return "the path has fewer than two waypoints"
    steps = np.abs(np.diff(rows, axis=0)).max(axis=1)
    worst = int(np.argmax(steps))
    if steps[worst] > JOINT_STEP:
        return (f"a joint jumps {math.degrees(steps[worst]):.1f}° between waypoints {worst} and {worst + 1} "
                f"(over {math.degrees(JOINT_STEP):.0f}°)")
    if len(arms) < 2:
        return ""
    first, second = arms[0], arms[1]
    bases = {arm: robot.arms[arm].base_in_husky for arm in (first, second)}

    def relative(row: np.ndarray) -> Pose:
        """The second arm's tool0 seen from the first's, both through their base_link in the husky frame."""
        tool0 = tool0_by_arm(robot.config.model.urdf, (first, second), names, row)
        in_husky = {arm: compose(bases[arm], tool0[arm]) for arm in (first, second)}
        return compose(invert(in_husky[first]), in_husky[second])

    held = relative(rows[0])
    for k, row in enumerate(rows[1:], start=1):
        distance, turn = pose_error(relative(row), held)
        if distance > COUPLING[0] or turn > COUPLING[1]:
            return (f"the arms' relative pose drifts {distance * 1e3:.1f} mm, {math.degrees(turn):.1f}° at waypoint "
                    f"{k} (over {COUPLING[0] * 1e3:.0f} mm, {math.degrees(COUPLING[1]):.0f}°)")
    return ""


# --- --- --- --- --- THE PARTS --- --- --- --- ---

async def run_grip(ctx: PluginContext, tools: Mapping[str, ScaffoldingV3], close: bool, *,
                   timeout_s: float = GRIP_TIMEOUT) -> tuple[bool, str]:
    """Close (or open) every gripper screw and wait until each stalls; every motor is stopped at the end."""
    return await run_screws(ctx, tools, GRIPPER_MOTOR, close, timeout_s=timeout_s)


async def run_screws(ctx: PluginContext, tools: Mapping[str, ScaffoldingV3], motor: int, tighten: bool, *,
                     seconds: float | None = None, timeout_s: float = GRIP_TIMEOUT) -> tuple[bool, str]:
    """Run one screw motor of every tool one way until each stalls, or for `seconds`; every motor is stopped at the end.

    Args:
        ctx: The plugin's context.
        tools: Arm name -> its scaffolding tool.
        motor: GRIPPER_MOTOR or JOINT_MOTOR.
        tighten: Tighten the screws (a gripper screw closes); False loosens them.
        seconds: Run this long, then stop (a loosening joint screw never stalls); None: until every screw stalls.
        timeout_s: Seconds for every screw to stall, when `seconds` is None.

    Returns:
        tuple[bool, str]: Whether the screws ran as asked, and one line saying what happened.
    """
    name = MOTOR_NAMES[motor]
    verb, past = SCREW_VERBS[motor][0 if tighten else 1]
    not_cleared = await stop_and_clear(ctx, tools, motor)
    if not_cleared is not None:
        return False, f"{not_cleared.reason}; nothing {past}"
    try:
        for tool in tools.values():
            if motor == GRIPPER_MOTOR:
                (tool.close if tighten else tool.open)()
            else:
                tool.drive_screw(1 if tighten else -1)
    except MotionRefused as refused:
        stop_all(tools)
        return False, f"{refused}; not every {name} {verb}s"
    gate = StallGate(ctx, tools, motor)
    ctx.log_info(f"{gate.text} {verb.upper()}" + ("" if seconds is None else f" for {seconds:g} s"))
    try:
        if seconds is None:
            await ctx.wait_until(gate.all_stalled, timeout_s=timeout_s, description=f"the {name}s to stall")
        else:
            await ctx.sleep(seconds)
    except WaitTimeout:
        return False, f"not every {name} stalled within {timeout_s:.0f} s of its {verb}; motors stopped"
    finally:
        stop_all(tools)
    if seconds is not None:
        return True, f"{name}s {past} for {seconds:g} s; motors stopped"
    times = ", ".join(f"{arm} {seconds:.1f} s" for arm, seconds in gate.stall_times.items())
    return True, f"{name}s {past}: stalled after {times}"


async def run_compliant_hold(ctx: PluginContext, arms: Collection[ArmInterface], released: Callable[[], bool], *,
                             label: str, recorder: Recorder | None = None) -> str:
    """Put the arms under the compliance controller, zero target wrench, until `released`; then bring them back.

    The force sensors are zeroed first, so the arms give way to a push only. A cancel (Cancel, Stop all) only holds:
    the arms stay compliant, said loudly. `recorder` gets a sample each tick of the hold, for a live plot.

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

        def done() -> bool:
            if recorder is not None:
                recorder.tick(ctx.now())
            return released()

        await ctx.wait_until(done, description="Rigid")
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
        return RETREATED, "the screws loosened and the tools backed out, the bar still held"

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
