"""
A step's parts, run one at a time: a motion along its plan, a tool step, or a step done by hand.

Each part asks the operator first, the robot named, and checks the world again after Confirm. A motion is then rebuilt
from the live joints and sent in the same tick, so nothing can change between the last check and the send; the other
robots are frozen (`ctx.freeze`) until it ends. An insert, a retreat (both motions along the plan) and the Robotiq
handoff run partly compliant (`shared/execution/`), with the run's settings read once when the part starts. The servo
part (`servo_part.py`) runs on the same `Job`, `PartRun` and checks; the plugin dispatches it.

! Main thread, inside the plugin's task.
"""

from __future__ import annotations

import asyncio
import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Awaitable, Callable, Mapping, Sequence

import numpy as np
from async_timeout import timeout

from scaffolding_assembly_core.ids import robot_id

from ...robot_interface.end_effectors import RobotiqGripper, ScaffoldingV3
from ...shared.assembly_planning.joints import arm_joint_names
from ...shared.assembly_planning.movement import Routine
from ...shared.assembly_run_contract import is_live
from ...shared.execution.basis import STALE_ARM, stale_reason
from ...shared.execution.handoff import handoff_text, run_handoff
from ...shared.execution.insert import plan_insert, run_insert
from ...shared.execution.moving import base_moving, moving_reason
from ...shared.execution.retreat import TAIL_STEP, plan_retreat, run_retreat
from ...shared.execution.send import ARRIVED, LARGE_TRAVEL, SendProgress, motion_command, send_motion
from ...shared.execution.send import DRY_RUN as SENT_DRY, HOLD_NOT_SENT as SENT_NOT_HELD, REFUSED as SENT_REFUSED
from ...shared.execution.stream import CANCELLED as PART_CANCELLED, DRY_RUN as PART_DRY, REFUSED as PART_REFUSED
from ...shared.execution.stream import HOLD_NOT_SENT as PART_NOT_HELD
from ...shared.execution.stream import Recorder
from ...shared.execution.tools import (FELL_SHORT, GRASP_TIMEOUT, REACHED, ROBOTIQ_CLOSE, ROBOTIQ_EFFORT, ROBOTIQ_OPEN,
                                       STALLED, robotiq_move, scaffolding_grasp)
from ...shared.execution.tools import DRY_RUN as TOOL_DRY, REFUSED as TOOL_REFUSED
from ...shared.motion_planner_contract import MotionPlanRequest
from ...shared.movement_exec_contract import (BASE_PART, BY_HAND_PART, CANCELLED, DONE, DRY_RUN, FAILED, GRASP_PART,
                                              HANDOFF_PART, HOLD_NOT_SENT, INSERT_PART, MOTION_KINDS, MOTION_PART,
                                              OPEN_PART, REFUSED, REFUSED_PART, RETREAT_PART, WITH_MOTION_PART,
                                              current_primary)
from ...shared.runs.runs import SPLIT_RANGE
from ...world.freeze import WorldMoved, take_basis

if TYPE_CHECKING:
    from scaffolding_assembly_core.geometry import Pose

    from ...plugin_api.context import PluginContext
    from ...robot_interface.robot import HuskyRobotInterface
    from ...shared.assembly_planning.route_result import RouteResult
    from ...shared.assembly_run_contract import AssemblyRunContract
    from ...shared.execution.insert import InsertPlan
    from ...shared.execution.operator import Prompt, PromptBox
    from ...shared.execution.retreat import RetreatPlan
    from ...shared.execution.send import MotionCommand
    from ...shared.execution.stream import PartOutcome
    from ...shared.execution.tools import ToolOutcome
    from ...shared.execution.basis import PlanBasis
    from ...shared.motion_planner_contract import MotionPlannerContract
    from ...shared.movement_exec_contract import Carried, ExecRequest, PartPlan
    from ...world.freeze import Basis, Freeze

#: Seconds the arms stand still after the travel to load before the force sensors are zeroed (old W:5781).
FT_ZERO_SETTLE = 2.0
#: Seconds the retreat's tail may take to plan; without a plan by then the retreat fails (A4).
REPLAN_TIMEOUT = 60.0


@dataclass
class Job:
    """One request being run, and what its parts need.

    Attributes:
        ctx: The plugin's context.
        request: The request.
        robot: The acting robot.
        run: The assembly_run plugin, for its view.
        prompts: The plugin's prompt box.
        dry_run: Whether this request sends nothing (fixed when it starts).
        event: Records a run event: (name, movement id, data).
        moved_at: Serial -> ROS time its arms last arrived (a motion, a servo pass); the plugin's dict, shared.
        show: Writes the panel's servo line.
        opened: The arms whose gripper screws a retreat of this request opened (its ungrasp is then done).
        recorder: The samples of the compliant part running (insert, retreat, handoff), for the panel; else None.
    """

    ctx: PluginContext
    request: ExecRequest
    robot: HuskyRobotInterface
    run: AssemblyRunContract
    prompts: PromptBox
    dry_run: bool
    event: Callable[[str, str, Mapping[str, Any]], None]
    moved_at: dict[str, float] = field(default_factory=dict)
    show: Callable[[str], None] = lambda _line: None
    opened: set[str] = field(default_factory=set)
    recorder: Recorder | None = None

    @property
    def who(self) -> str:
        """str: The robot as prompts name it, e.g. "alice (a200-0804)"."""
        return f"{self.request.robot_name} ({self.request.serial})"


@dataclass
class PartRun:
    """What one part did, filled in as it goes, so a part cancelled midway is still recorded.

    Attributes:
        part: The part.
        result: DONE, DRY_RUN, REFUSED, CANCELLED or FAILED; "" while running.
        status: The outcome's own status, e.g. "ARRIVED" or "reached".
        reason: One line saying what happened.
        command: What was (or would be) sent, as JSON values.
        outcome: The send's or tool's outcome, as JSON values.
        end: The acting arms' measured joints after a motion that arrived, by name.
        base: The acting robot's base then.
        progress: How far a motion's send got, for a cancel's record; None for other parts.
        freeze: The other robots' freeze while a motion runs; its `problem` says why it broke. None for other parts.
        servo_file: The servo record's path in the run, once the servo part wrote one.
        carry: What a transfer's bar servo leaves for its insert's plan, once it did.
    """

    part: PartPlan
    result: str = ""
    status: str = ""
    reason: str = ""
    command: dict | None = None
    outcome: dict | None = None
    end: dict[str, float] | None = None
    base: Pose | None = None
    progress: SendProgress | None = None
    freeze: Freeze | None = None
    servo_file: str | None = None
    carry: Carried | None = None

    def ended(self, result: str, status: str, reason: str) -> None:
        """Say how the part ended."""
        self.result, self.status, self.reason = result, status, reason


async def run_part(job: Job, run: PartRun) -> None:
    """Run one part, asking the operator first; `run` says how it ended.

    Args:
        job: The request being run.
        run: The part, filled in as it goes.

    Raises:
        asyncio.CancelledError: When cancelled; a motion has held every arm, a tool has been stopped.
    """
    kind = run.part.kind
    if kind == REFUSED_PART:
        run.ended(REFUSED, "refused", run.part.text)
        return
    problem = world_problem(job, motion=kind in MOTION_KINDS)
    if problem:
        run.ended(REFUSED, "refused", problem)
        return
    runs = {MOTION_PART: _motion, INSERT_PART: _insert, RETREAT_PART: _retreat, OPEN_PART: _robotiq,
            HANDOFF_PART: _handoff, GRASP_PART: _grasp, WITH_MOTION_PART: _with_motion, BY_HAND_PART: _by_hand,
            BASE_PART: _by_hand}
    await runs[kind](job, run)


def world_problem(job: Job, motion: bool, basis: PlanBasis | None = None) -> str:
    """Why a part may not run now; "" if it may.

    Every part: the process and the current action are still the request's, the robot is not soft stopped (Stop all,
    or a clearance tripped: its commands are refused until Resume), and nothing else moves (`others_moving`). A motion
    also: the robot is live, the safety sync vouches for each of its arms (`ArmInterface.safety_problem`:
    else the arm refuses the trajectory anyway), the plan was made after the robot's arms last moved (`Job.moved_at`)
    and is not stale (`stale_reason`).

    Args:
        job: The request being run.
        motion: Whether the part moves the arms.
        basis: What the motion's plan was made from; None: the request's.
    """
    request, view, ctx = job.request, job.run.view, job.ctx
    if view.process is not request.loaded:
        return "the process was loaded again since the request"
    if view.current is None or view.current[0] != request.action:
        return f"the current action is no longer {request.action}"
    if job.robot.soft_stopped:
        return f"{job.who} is soft stopped; press Resume first"
    moving = others_moving(job)
    if moving or not motion:
        return moving
    snapshot = ctx.scene.snapshot
    robot = snapshot.robots.get(robot_id(request.serial))
    if robot is None or not is_live(robot, ctx.now()):
        return f"{job.who} is not live (base tracked and fresh within 1 s, joints within 0.5 s)"
    for name, arm in job.robot.arms.items():
        if problem := arm.safety_problem():
            return f"{job.who} {name}: {problem}"
    basis = basis if basis is not None else request.basis
    if basis is None:
        return "the plan has no basis to check it against"
    if basis.time < job.moved_at.get(request.serial, -math.inf):
        return "the arms moved since this plan was made: replan"
    transfer_as_free = request.transfer_as_free()
    return stale_reason(basis, snapshot, loaded=view.process,
                        current=current_primary(view, transfer_as_free), transfer_as_free=transfer_as_free,
                        ignore_built=request.ignore_built(), statuses=view.statuses,
                        obstacles_collide=request.obstacles_collide())


def others_moving(job: Job) -> str:
    """Why the part must not move now: another robot moves, or this robot's base moves; else "".

    ! One motion at a time across every robot: checked before the prompt, after Confirm and, for a motion, every tick
      while the arms move (`send_motion`'s watch, next to the freeze). The request leases every robot, so only motion
      this monitor did not send (the pendant, a node on the robot) can show up here.
    """
    serial, now = job.request.serial, job.ctx.now()
    return moving_reason(job.ctx.world, now, besides=serial) or base_moving(job.robot.base.state, f"{serial} base", now)


def others_basis(job: Job) -> Basis:
    """What a motion relies on standing still: the other robots' arms and bases, measured now, for `ctx.freeze`.

    Arms that never sent joint states and bases never fixed by mocap this session are left out, as `moving_reason`
    leaves them out (that robot is not running, or not tracked: the plan did not check against it).

    Raises:
        WorldMoved: If one of them can no longer be measured (stale joints, mocap lost).
    """
    others = [(serial, robot) for serial, robot in job.ctx.world.robots.items() if serial != job.request.serial]
    return take_basis(job.ctx.world, job.ctx.now(),
                      arms=[(serial, name) for serial, robot in others for name, arm in robot.arms.items()
                            if arm.state.last_update_time is not None],
                      bases=[serial for serial, robot in others if robot.base.state.last_fix_time is not None])


def live_path(job: Job) -> tuple[list[str], np.ndarray] | str:
    """The request's plan with its first row replaced by the arms' joints now (`live_path_of`); or why it can't be.

    Args:
        job: The request being run.

    Returns:
        tuple[list[str], np.ndarray] | str: (names, rows), or the refusal (no plan, arms over STALE_ARM from its
            start).
    """
    route = job.request.route
    trajectory = route.result.trajectory if route is not None and route.ok else None
    if trajectory is None:
        return "no plan for this motion: Plan first"
    return live_path_of(job, list(trajectory.joint_names), trajectory.positions)


def live_path_of(job: Job, names: Sequence[str], rows: Sequence[Sequence[float]]) -> tuple[list[str], np.ndarray] | str:
    """A path with its first row replaced by the arms' joints now; or why it can't be.

    Args:
        job: The request being run.
        names: The path's joint names.
        rows: The path, radians, its first row where it was planned from.

    Returns:
        tuple[list[str], np.ndarray] | str: (names, rows), or the refusal (arms over STALE_ARM from the path's
            start, a joint without a state).
    """
    names, rows = list(names), np.array(rows, dtype=float)
    live = {}
    for name, arm in job.robot.arms.items():
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
                f"replan")
    # * Row 0 is where the arms are, so the trajectory starts without a jump.
    rows[0] = now
    return names, rows


def live_command_of(job: Job, names: Sequence[str], rows: Sequence[Sequence[float]],
                    traj_time: float) -> MotionCommand | str:
    """A path as a command taking `traj_time`, from the arms' joints now (`live_path_of`); or why it can't be."""
    path = live_path_of(job, names, rows)
    if isinstance(path, str):
        return path
    try:
        return motion_command(job.request.serial, list(job.robot.arms), *path, traj_time)
    except ValueError as error:
        return f"the plan can't be sent: {error}"


def live_command(job: Job) -> MotionCommand | str:
    """The request's plan as a command at its motion time (`live_command_of`); or why it can't be (no plan, ...)."""
    route = job.request.route
    trajectory = route.result.trajectory if route is not None and route.ok else None
    if trajectory is None:
        return "no plan for this motion: Plan first"
    return live_command_of(job, list(trajectory.joint_names), trajectory.positions, job.request.traj_time)


def run_settings(job: Job) -> dict:
    """The run's settings a compliant part runs with, read once as it starts, the split clamped to SPLIT_RANGE."""
    settings = job.run.view.settings
    return {"split_mm": min(max(float(settings.insert_split_mm), SPLIT_RANGE[0]), SPLIT_RANGE[1]),
            "rigid_only": settings.rigid_only, "spiral": settings.spiral,
            "spiral_seconds": settings.spiral_seconds, "spiral_turns": settings.spiral_turns}


def _ask(job: Job, movement: str, text: str, warn: bool) -> Prompt:
    """Open a prompt after the robot's name ("DRY RUN — " first in a dry run), log it and record it."""
    prompt = job.prompts.ask(job.who, ("DRY RUN — " if job.dry_run else "") + text, job.ctx.now(), warn)
    (job.ctx.log_warn if warn else job.ctx.log_info)(f"waiting for Confirm: {prompt.line}")
    job.event("prompt", movement, {"text": prompt.line, "warn": warn})
    return prompt


@dataclass(frozen=True)
class PartOperator:
    """The operator as a compliant part asks it without waiting (`spiral.Operator`): prompts as `confirmed`'s.

    Attributes:
        job: The request being run.
        movement: The part's movement id: every prompt reads "<robot> (<serial>): <movement> — <text>".
    """

    job: Job
    movement: str

    def open(self, text: str, warn: bool = False) -> Prompt:
        """Ask `text`, as a warning if `warn`, and record it; returns the prompt to poll."""
        return _ask(self.job, self.movement, f"{self.movement} — {text}", warn)

    def poll(self, prompt: Prompt) -> bool | None:
        """The answer, recorded once it came: True for Confirm, False for Cancel; None while there is none."""
        answer = self.job.prompts.poll(prompt)
        if answer:
            self.job.event("confirm", self.movement, {"text": prompt.line})
        elif answer is not None:
            self.job.event("cancel", self.movement, {"reason": "operator", "at": "prompt"})
        return answer

    def close(self, prompt: Prompt) -> None:
        """Take the prompt away unanswered."""
        self.job.prompts.close(prompt)


async def confirmed(job: Job, run: PartRun, text: str, warn: bool = False) -> bool:
    """Ask the operator and wait (no time limit); a Cancel ends the part as cancelled.

    Args:
        job: The request being run.
        run: The part.
        text: The question, after the robot's name.
        warn: Ask as a warning.

    Returns:
        bool: True on Confirm.
    """
    ctx, movement = job.ctx, run.part.movement
    prompt = _ask(job, movement, text, warn)
    if await job.prompts.wait(ctx, prompt):
        job.event("confirm", movement, {"text": prompt.line})
        return True
    job.event("cancel", movement, {"reason": "operator", "at": "prompt"})
    run.ended(CANCELLED, "cancelled", "cancelled at the prompt; nothing sent")
    return False


# --- --- --- --- --- PARTS --- --- --- --- ---

async def _motion(job: Job, run: PartRun) -> None:
    """Move the arms along the plan: prompt with its time and largest joint travel, check again, send, watch."""
    ctx, part = job.ctx, run.part
    command = live_command(job)
    if isinstance(command, str):
        run.ended(REFUSED, "refused", command)
        return
    run.command = command.as_record()
    ctx.log_info(f"{job.who}: the teach pendant's speed slider must be at 100 %: slower, the motion times out and "
                 f"is held mid-path")
    arms = ", ".join(motion.arm for motion in command.arms)
    basis = job.request.basis
    ignored = " (built bars ignored)" if basis is not None and basis.ignore_built else ""
    ignored += " (Rhino obstacles not checked)" if basis is not None and not basis.obstacles_collide else ""
    text = (f"{part.movement} — move {arms} along the planned path{ignored}, {command.duration:.0f} s, largest joint "
            f"travel {math.degrees(command.travel):.0f}° (Play it first). Confirm to move.")
    if not await confirmed(job, run, text, warn=command.travel > LARGE_TRAVEL):
        return
    # ! Checked again after Confirm; the command is rebuilt from the joints now and sent in this same tick.
    command = world_problem(job, motion=True) or live_command(job)
    if isinstance(command, str):
        run.ended(REFUSED, "refused", command)
        return
    run.command = command.as_record()
    try:
        basis = others_basis(job)
    except WorldMoved as unmeasured:
        run.ended(REFUSED, "refused", f"can't tell whether the other robots stand still: {unmeasured}")
        return
    run.progress = SendProgress()
    # * Both hold every arm: the watch at another robot's first activity, the freeze once one was displaced (a push
    #   too) or lost to mocap, which cancels this task.
    with ctx.freeze(basis) as run.freeze:
        outcome = await send_motion(ctx, job.robot, command, dry_run=job.dry_run, watch=lambda: others_moving(job),
                                    progress=run.progress)
    run.outcome = outcome.as_record()
    result = {ARRIVED: DONE, SENT_DRY: DRY_RUN, SENT_REFUSED: REFUSED, SENT_NOT_HELD: HOLD_NOT_SENT}.get(
        outcome.status, FAILED)
    run.ended(result, outcome.status, outcome.reason)
    (ctx.log_info if outcome.ok else ctx.log_warn)(f"{job.who}: {part.movement}: {outcome.reason}")
    if outcome.status != ARRIVED:
        return
    _note_end(job, run, outcome.end or {})
    if part.routine is Routine.SUPPORT_LEAVE:
        ctx.log_info(f"{job.who}: {part.movement}: done; drive the base away by hand")
    elif part.routine is Routine.FREE_TO_LOAD:
        # * Tare the force sensors standing still, on the empty tools (old W:5807-5849).
        await ctx.sleep(FT_ZERO_SETTLE)
        zeroed = [arm.zero_ft_sensor() for arm in job.robot.arms.values()]
        (ctx.log_info if all(zeroed) else ctx.log_warn)(
            f"{job.who}: travel to load done; force sensors zeroed on {sum(zeroed)} of {len(zeroed)} arms")


async def _insert(job: Job, run: PartRun) -> None:
    """Insert the held bar by the run's settings (`run_insert`): rigid, then compliant while the mating screws tighten.

    A ground bar (no mating arm) runs rigid, no tool driven.
    """
    part, robot = run.part, job.robot
    tools = {name: arm.end_effector for name, arm in robot.arms.items() if isinstance(arm.end_effector, ScaffoldingV3)}
    missing = [arm for arm in part.tools if arm not in tools]
    if missing:
        run.ended(REFUSED, "refused", f"{job.who} has no scaffolding tool on {', '.join(missing)}")
        return
    settings = run_settings(job)
    spiral = (settings["spiral_seconds"], settings["spiral_turns"]) if settings["spiral"] else None

    def plan_of(names: list[str], rows: np.ndarray) -> InsertPlan | str:
        return plan_insert(robot, names, rows, mating=part.tools, grounded=not part.tools,
                           split_mm=settings["split_mm"], rigid_only=settings["rigid_only"], spiral=spiral,
                           traj_time=job.request.traj_time)

    def start(plan: InsertPlan, recorder: Recorder) -> Awaitable[PartOutcome]:
        return run_insert(job.ctx, robot, plan, tools if part.tools else {}, operator=PartOperator(job, part.movement),
                          dry_run=job.dry_run, watch=lambda: others_moving(job), recorder=recorder)

    await _along_plan(job, run, settings, plan_of, start)


async def _retreat(job: Job, run: PartRun) -> None:
    """Back the tools out of the bar (`run_retreat`); done, its arms count as opened for the ungrasp it runs.

    The gripper screws open, a compliant break-out, then the rest replanned (`_replan`) and run rigid.
    """
    part, robot = run.part, job.robot
    tools = {name: robot.arms[name].end_effector for name in part.tools if name in robot.arms}
    if len(tools) != len(part.tools) or not all(isinstance(tool, ScaffoldingV3) for tool in tools.values()):
        run.ended(REFUSED, "refused", f"{job.who} has no scaffolding tool on {', '.join(part.tools)}")
        return
    settings = run_settings(job)
    goals = job.request.route.goals if job.request.route is not None else {}

    def plan_of(names: list[str], rows: np.ndarray) -> RetreatPlan | str:
        return plan_retreat(robot, names, rows, goals, split_mm=settings["split_mm"], traj_time=job.request.traj_time)

    def start(plan: RetreatPlan, recorder: Recorder) -> Awaitable[PartOutcome]:
        return run_retreat(job.ctx, robot, plan, tools, replan=lambda: _replan(job, part), dry_run=job.dry_run,
                           watch=lambda: others_moving(job), recorder=recorder)

    outcome = await _along_plan(job, run, settings, plan_of, start)
    if outcome is not None and outcome.ok:
        job.opened.update(part.tools)


async def _along_plan(job: Job, run: PartRun, settings: dict, plan_of: Callable[[list[str], np.ndarray], Any],
                      start: Callable[[Any, Recorder], Awaitable[PartOutcome]]) -> PartOutcome | None:
    """Run a compliant part along the plan as `_motion` runs a motion: plan, prompt, check and plan again, run.

    Args:
        job: The request being run.
        run: The part.
        settings: The run's settings it runs with (`run_settings`), recorded.
        plan_of: The plan's names and rows from the live joints -> the part's plan, or why not.
        start: Runs the plan under the other robots' freeze, its samples into the recorder given.

    Returns:
        PartOutcome | None: How the part ran; None if it did not start.
    """
    ctx, part = job.ctx, run.part

    def planned() -> Any:
        path = live_path(job)
        return path if isinstance(path, str) else plan_of(*path)

    plan = planned()
    if isinstance(plan, str):
        run.ended(REFUSED, "refused", plan)
        return None
    run.command = {"settings": settings, "plan": plan.as_record()}
    ctx.log_info(f"{job.who}: the teach pendant's speed slider must be at 100 %: slower, the motion times out and "
                 f"is held mid-path")
    if not await confirmed(job, run, f"{part.movement} — {plan.text}"):
        return None
    # ! Checked again after Confirm; planned again from the joints now and started in this same tick.
    plan = world_problem(job, motion=True) or planned()
    if isinstance(plan, str):
        run.ended(REFUSED, "refused", plan)
        return None
    run.command["plan"] = plan.as_record()
    try:
        basis = others_basis(job)
    except WorldMoved as unmeasured:
        run.ended(REFUSED, "refused", f"can't tell whether the other robots stand still: {unmeasured}")
        return None
    job.recorder = Recorder(job.robot, ctx.now())
    try:
        with ctx.freeze(basis) as run.freeze:
            outcome = await start(plan, job.recorder)
    finally:
        job.recorder = None
    _part_ended(run, outcome, settings)
    if outcome.ok:
        _note_end(job, run, {name: job.robot.arms[name].joint_vector() for name in plan.arms})
    return outcome


async def _replan(job: Job, part: PartPlan) -> RouteResult | None:
    """Plan the retreat's tail again, from the live robot to the plan's goals in TAIL_STEP steps.

    Returns:
        RouteResult | None: The plan, or None without an answer within REPLAN_TIMEOUT.
    """
    ctx, request = job.ctx, job.request
    loaded = request.loaded
    asked = MotionPlanRequest(loaded, loaded.steps[loaded.index_of(request.action, part.index)], ctx.scene.snapshot,
                              ctx.config.robots, beliefs=job.run.view.beliefs,
                              transfer_as_free=request.transfer_as_free(), ignore_built=request.ignore_built(),
                              obstacles_collide=request.obstacles_collide(), goal_poses=request.route.goals,
                              linear_step=TAIL_STEP, label=f"retreat tail {part.movement}")
    planner: MotionPlannerContract = ctx.require("motion_planner")
    try:
        async with timeout(REPLAN_TIMEOUT):
            return await planner.request_plan(asked)
    except asyncio.TimeoutError:
        ctx.log_warn(f"{job.who}: {part.movement}: no tail planned within {REPLAN_TIMEOUT:.0f} s")
        return None


async def _handoff(job: Job, run: PartRun) -> None:
    """Close a Robotiq on a bar another robot holds, its arm compliant meanwhile (`run_handoff`), after the prompt."""
    ctx, part = job.ctx, run.part
    gripper = _robotiq_of(job, run)
    if gripper is None:
        return
    arm = part.tools[0]
    run.command = {"tool": "robotiq", "arm": arm, "position": ROBOTIQ_CLOSE, "effort": ROBOTIQ_EFFORT,
                   "compliant": True}
    if not await confirmed(job, run, f"{part.movement} — {handoff_text(arm)}"):
        return
    problem = world_problem(job, motion=False)
    if problem:
        run.ended(REFUSED, "refused", problem)
        return
    job.recorder = Recorder(job.robot, ctx.now())
    try:
        outcome = await run_handoff(ctx, job.robot, arm, gripper, dry_run=job.dry_run, recorder=job.recorder)
    finally:
        job.recorder = None
    _part_ended(run, outcome)
    (ctx.log_info if outcome.ok else ctx.log_warn)(f"{job.who}: {part.movement}: {outcome.status}: {outcome.reason}")


async def _with_motion(job: Job, run: PartRun) -> None:
    """An ungrasp its retreat runs: done once that retreat opened the gripper screws of the same arms."""
    part = run.part
    run.command = {"with_motion": part.text}
    if job.dry_run:
        run.ended(DRY_RUN, "dry_run", "the retreat opens these gripper screws (dry run: nothing sent)")
    elif set(part.tools) <= job.opened:
        run.ended(DONE, "opened", f"the retreat opened the gripper screws of {', '.join(part.tools)}")
    else:
        run.ended(FAILED, "not opened", f"no retreat of this Exec opened the gripper screws of {', '.join(part.tools)}")


async def _robotiq(job: Job, run: PartRun) -> None:
    """Open the Robotiq to 0.426 after the prompt."""
    part = run.part
    gripper = _robotiq_of(job, run)
    if gripper is None:
        return
    run.command = {"tool": "robotiq", "arm": part.tools[0], "position": ROBOTIQ_OPEN, "effort": ROBOTIQ_EFFORT}
    if not await confirmed(job, run, f"{part.movement} — OPEN the gripper ({ROBOTIQ_OPEN}). Confirm to send."):
        return
    problem = world_problem(job, motion=False)
    if problem:
        run.ended(REFUSED, "refused", problem)
        return
    _tool_ended(job, run, await robotiq_move(job.ctx, gripper, ROBOTIQ_OPEN, dry_run=job.dry_run))


def _robotiq_of(job: Job, run: PartRun) -> RobotiqGripper | None:
    """The Robotiq on the part's arm; None, the part ended refused, if it has none."""
    arm = run.part.tools[0]
    gripper = job.robot.arms[arm].end_effector if arm in job.robot.arms else None
    if isinstance(gripper, RobotiqGripper):
        return gripper
    run.ended(REFUSED, "refused", f"{job.who} has no Robotiq on {arm}")
    return None


async def _grasp(job: Job, run: PartRun) -> None:
    """Clamp the bar in the scaffolding tools: STOP, tighten the M1 screws until STALLED, STOP."""
    part = run.part
    tools = {arm: job.robot.arms[arm].end_effector for arm in part.tools if arm in job.robot.arms}
    if len(tools) != len(part.tools) or not all(isinstance(tool, ScaffoldingV3) for tool in tools.values()):
        run.ended(REFUSED, "refused", f"{job.who} has no scaffolding tool on {', '.join(part.tools)}")
        return
    run.command = {"tool": "scaffolding_v3", "arms": list(tools), "motor": "gripper (M1) tighten",
                   "timeout": GRASP_TIMEOUT}
    text = (f"{part.movement} — GRASP: STOP, then the M1 screw of {', '.join(tools)} TIGHTENS until STALLED "
            f"(≤{GRASP_TIMEOUT:.0f} s), then STOP. Confirm to send.")
    if not await confirmed(job, run, text):
        return
    problem = world_problem(job, motion=False)
    if problem:
        run.ended(REFUSED, "refused", problem)
        return
    _tool_ended(job, run, await scaffolding_grasp(job.ctx, tools, dry_run=job.dry_run,
                                                  ready=lambda: world_problem(job, motion=False)))


async def _by_hand(job: Job, run: PartRun) -> None:
    """A step done by hand, every robot still leased: done once the operator confirms it.

    ? Leased, so nothing else moves meanwhile (and the step panel keeps its done parts): the monitor's panels refuse
      while the step has the leases; the prompt says so, and how to drive a base without them.
    """
    part = run.part
    run.command = {"by_hand": part.text}
    if part.kind == BASE_PART:
        text = (f"{part.movement} — drive {job.who}'s base there with its joystick or the command line (the D-pad and "
                f"the base planner refuse while this step has the robot's lease; Cancel to use them), then Confirm.")
    else:
        text = (f"{part.movement} — {part.text}, then Confirm (the robot panels and their free drive refuse while "
                f"this step has the robots' leases; Cancel to use them).")
    if await confirmed(job, run, text):
        run.ended(DRY_RUN if job.dry_run else DONE, "confirmed", "done by hand, confirmed by the operator")


def _part_ended(run: PartRun, outcome: PartOutcome, settings: dict | None = None) -> None:
    """Say how a compliant part ended and keep its record.

    NO_STALL, STIFF and a failed tail replan are FAILED (A3, A4); a rigid part's refused hold stays HOLD_NOT_SENT, as a
    motion's.

    Args:
        run: The part.
        outcome: How it ended.
        settings: The run's settings it ran with, recorded; None for none.
    """
    run.outcome = {"status": outcome.status, "reason": outcome.reason, "seconds": outcome.seconds,
                   **({} if settings is None else {"settings": settings}), **outcome.record}
    result = DONE if outcome.ok else {PART_DRY: DRY_RUN, PART_REFUSED: REFUSED, PART_CANCELLED: CANCELLED,
                                      PART_NOT_HELD: HOLD_NOT_SENT}.get(outcome.status, FAILED)
    run.ended(result, outcome.status, outcome.reason)


def _note_end(job: Job, run: PartRun, joints: Mapping[str, Sequence[float] | None]) -> None:
    """Keep where the moving arms ended (arm name -> joints, UR order; None: not measured) and the base then."""
    run.end = {name: float(value) for arm, values in joints.items() if values is not None
               for name, value in zip(arm_joint_names(arm), values)}
    snapshot = job.ctx.scene.snapshot.robots.get(robot_id(job.request.serial))
    run.base = snapshot.base if snapshot is not None else None


def _tool_ended(job: Job, run: PartRun, outcome: ToolOutcome) -> None:
    """Say how a tool part ended, a fell-short loudly."""
    run.outcome = outcome.as_record()
    result = DONE if outcome.status in (REACHED, STALLED) else {TOOL_DRY: DRY_RUN, TOOL_REFUSED: REFUSED}.get(
        outcome.status, FAILED)
    run.ended(result, outcome.status, outcome.reason)
    if outcome.status == FELL_SHORT:
        job.ctx.log_warn(f"{job.who}: {run.part.movement}: {outcome.reason}; check the fingers")
    else:
        (job.ctx.log_info if outcome.ok else job.ctx.log_warn)(f"{job.who}: {run.part.movement}: {outcome.reason}")
