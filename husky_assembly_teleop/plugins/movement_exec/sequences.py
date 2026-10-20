"""
A step's parts, run one at a time: a motion along its plan, a tool step, or a step done by hand.

Each part asks the operator first, the robot named, and checks the world again after Confirm. A motion is then rebuilt
from the live joints and sent in the same tick, so nothing can change between the last check and the send.

! Main thread, inside the plugin's task.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Callable, Mapping

import numpy as np

from scaffolding_assembly_core.ids import robot_id

from ...robot_interface.end_effectors import RobotiqGripper, ScaffoldingV3
from ...shared.assembly_run import is_live
from ...shared.execution.basis import STALE_ARM, stale_reason
from ...shared.execution.interlock import base_moving, busy_reason
from ...shared.execution.send import ARRIVED, LARGE_TRAVEL, SendProgress, motion_command, send_motion
from ...shared.execution.send import DRY_RUN as SENT_DRY, HOLD_NOT_SENT as SENT_NOT_HELD, REFUSED as SENT_REFUSED
from ...shared.execution.tools import (FELL_SHORT, GRASP_TIMEOUT, REACHED, ROBOTIQ_CLOSE, ROBOTIQ_EFFORT, ROBOTIQ_OPEN,
                                       STALLED, robotiq_move, scaffolding_grasp)
from ...shared.execution.tools import DRY_RUN as TOOL_DRY, REFUSED as TOOL_REFUSED
from ...shared.movement_exec import (BASE_PART, BY_HAND_PART, CANCELLED, CLOSE_PART, DONE, DRY_RUN, FAILED,
                                     GRASP_PART, HOLD_NOT_SENT, MOTION_PART, OPEN_PART, REFUSED, REFUSED_PART,
                                     current_primary)
from ...shared.planning.joints import arm_joint_names
from ...shared.planning.movement import Routine

if TYPE_CHECKING:
    from scaffolding_assembly_core.geometry import Pose

    from ...plugin_api.context import PluginContext
    from ...robot_interface.robot import HuskyRobotInterface
    from ...shared.assembly_run import AssemblyRunPluginApi
    from ...shared.execution.operator import PromptBox
    from ...shared.execution.send import MotionCommand
    from ...shared.execution.tools import ToolOutcome
    from ...shared.movement_exec import ExecRequest, PartPlan

#: Seconds the arms stand still after the travel to load before the force sensors are zeroed (old W:5781).
FT_ZERO_SETTLE = 2.0


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
    """

    ctx: PluginContext
    request: ExecRequest
    robot: HuskyRobotInterface
    run: AssemblyRunPluginApi
    prompts: PromptBox
    dry_run: bool
    event: Callable[[str, str, Mapping[str, Any]], None]

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
    problem = world_problem(job, motion=kind == MOTION_PART)
    if problem:
        run.ended(REFUSED, "refused", problem)
        return
    runs = {MOTION_PART: _motion, OPEN_PART: _robotiq, CLOSE_PART: _robotiq, GRASP_PART: _grasp,
            BY_HAND_PART: _by_hand, BASE_PART: _by_hand}
    await runs[kind](job, run)


def world_problem(job: Job, motion: bool) -> str:
    """Why a part may not run now; "" if it may.

    Every part: the process and the current action are still the request's, and nothing else moves (`others_moving`).
    A motion also: the robot is live and the plan is not stale (`stale_reason`).

    Args:
        job: The request being run.
        motion: Whether the part moves the arms.
    """
    request, view, ctx = job.request, job.run.view, job.ctx
    if view.process is not request.loaded:
        return "the process was loaded again since the request"
    if view.current is None or view.current[0] != request.action:
        return f"the current action is no longer {request.action}"
    moving = others_moving(job)
    if moving or not motion:
        return moving
    snapshot = ctx.scene.snapshot
    robot = snapshot.robots.get(robot_id(request.serial))
    if robot is None or not is_live(robot, ctx.now()):
        return f"{job.who} is not live (base tracked and fresh within 1 s, joints within 0.5 s)"
    if request.basis is None:
        return "the plan has no basis to check it against"
    return stale_reason(request.basis, snapshot, loaded=view.process, current=current_primary(view),
                        transfer_as_free=view.settings.transfer_as_free, ignore_built=request.ignore_built(),
                        statuses=view.statuses)


def others_moving(job: Job) -> str:
    """Why the part must not move now: another robot is held or moves, or this robot's base moves; else "".

    ! One motion at a time across every robot: checked before the prompt, after Confirm and, for a motion, every tick
      while the arms move (`send_motion`'s watch).
    """
    serial, now = job.request.serial, job.ctx.now()
    return busy_reason(job.ctx.world, now, besides=serial) or base_moving(job.robot.base.state, f"{serial} base", now)


def live_command(job: Job) -> MotionCommand | str:
    """The plan as a command, its first row replaced by the arms' joints now; or why it can't be.

    Args:
        job: The request being run.

    Returns:
        MotionCommand | str: The command, or the refusal (no plan, arms over STALE_ARM from its start, bad joints).
    """
    route = job.request.route
    trajectory = route.result.trajectory if route is not None and route.ok else None
    if trajectory is None:
        return "no plan for this motion: Plan first"
    names, rows = list(trajectory.joint_names), np.array(trajectory.positions, dtype=float)
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
    try:
        return motion_command(job.request.serial, list(job.robot.arms), names, rows, job.request.traj_time)
    except ValueError as error:
        return f"the plan can't be sent: {error}"


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
    prompt = job.prompts.ask(job.who, ("DRY RUN — " if job.dry_run else "") + text, ctx.now(), warn)
    (ctx.log_warn if warn else ctx.log_info)(f"waiting for Confirm: {prompt.line}")
    job.event("prompt", movement, {"text": prompt.line, "warn": warn})
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
    ignored = " (built bars ignored)" if job.request.basis is not None and job.request.basis.ignore_built else ""
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
    run.progress = SendProgress()
    outcome = await send_motion(ctx, job.robot, command, dry_run=job.dry_run, watch=lambda: others_moving(job),
                                progress=run.progress)
    run.outcome = outcome.as_record()
    result = {ARRIVED: DONE, SENT_DRY: DRY_RUN, SENT_REFUSED: REFUSED, SENT_NOT_HELD: HOLD_NOT_SENT}.get(
        outcome.status, FAILED)
    run.ended(result, outcome.status, outcome.reason)
    (ctx.log_info if outcome.ok else ctx.log_warn)(f"{job.who}: {part.movement}: {outcome.reason}")
    if outcome.status != ARRIVED:
        return
    run.end = {name: value for arm, joints in (outcome.end or {}).items() if joints is not None
               for name, value in zip(arm_joint_names(arm), joints)}
    snapshot = ctx.scene.snapshot.robots.get(robot_id(job.request.serial))
    run.base = snapshot.base if snapshot is not None else None
    if part.routine in (Routine.TRANSFER_FREE, Routine.SUPPORT_APPROACH):
        ctx.log_info(f"{job.who}: {part.movement}: servo correction: M7, not run")
    elif part.routine is Routine.SUPPORT_LEAVE:
        ctx.log_info(f"{job.who}: {part.movement}: done; drive the base away by hand")
    elif part.routine is Routine.FREE_TO_LOAD:
        # * Tare the force sensors standing still, on the empty tools (old W:5807-5849).
        await ctx.sleep(FT_ZERO_SETTLE)
        zeroed = [arm.zero_ft_sensor() for arm in job.robot.arms.values()]
        (ctx.log_info if all(zeroed) else ctx.log_warn)(
            f"{job.who}: travel to load done; force sensors zeroed on {sum(zeroed)} of {len(zeroed)} arms")


async def _robotiq(job: Job, run: PartRun) -> None:
    """Open the Robotiq to 0.426, or close it stiffly to 0.8 (the compliant handoff is M6) after a warning prompt."""
    part = run.part
    arm = part.tools[0]
    gripper = job.robot.arms[arm].end_effector if arm in job.robot.arms else None
    if not isinstance(gripper, RobotiqGripper):
        run.ended(REFUSED, "refused", f"{job.who} has no Robotiq on {arm}")
        return
    opening = part.kind == OPEN_PART
    position = ROBOTIQ_OPEN if opening else ROBOTIQ_CLOSE
    run.command = {"tool": "robotiq", "arm": arm, "position": position, "effort": ROBOTIQ_EFFORT}
    text = (f"{part.movement} — OPEN the gripper ({ROBOTIQ_OPEN}). Confirm to send." if opening else
            f"{part.movement} — STIFF close ({ROBOTIQ_CLOSE}; the compliant handoff comes in M6): jaws empty, no bar "
            f"held by another robot. Confirm to send.")
    if not await confirmed(job, run, text, warn=not opening):
        return
    problem = world_problem(job, motion=False)
    if problem:
        run.ended(REFUSED, "refused", problem)
        return
    _tool_ended(job, run, await robotiq_move(job.ctx, gripper, position, dry_run=job.dry_run))


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
    """A step done by hand, the robot still held: done once the operator confirms it.

    ? Held, so nothing else moves meanwhile (and the step panel keeps its done parts): the monitor's panels stay
      locked; the prompt says so, and how to drive a base without them.
    """
    part = run.part
    run.command = {"by_hand": part.text}
    if part.kind == BASE_PART:
        text = (f"{part.movement} — drive {job.who}'s base there with its joystick or the command line (the D-pad and "
                f"the base planner are locked while this step runs; Cancel to use them), then Confirm.")
    else:
        text = (f"{part.movement} — {part.text}, then Confirm (the robot panels are locked until this step ends; "
                f"Cancel to use them).")
    if await confirmed(job, run, text):
        run.ended(DRY_RUN if job.dry_run else DONE, "confirmed", "done by hand, confirmed by the operator")


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
