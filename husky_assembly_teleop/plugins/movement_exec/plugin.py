"""
Run one step of the assembly on the real robot: its motion along the plan, its tool steps, its steps done by hand.

Another plugin asks (`request_execute`, `shared.movement_exec.MovementExecPluginApi`); each part asks the operator to
Confirm first, on the laptop or the phone, and is recorded in the run (`assembly_run`). The robot is held from the
request to its end (`shared/execution/interlock.py`), so other plugins' commands to it are refused; Cancel here, Hold in
robot_control, or Stop all ends it, every arm held.

- Refused at once: while another request runs, a refused part (a later milestone's routine), a request for another
  robot, a plan for another robot, movement or base.
- Before every motion, and again after Confirm: the robot is live, the plan is not stale, the process and the current
  action are the request's; then the command is rebuilt from the live joints and sent in the same tick.
- Dry run: every check, every trajectory built, nothing sent; nothing is ever done by it.
- ! EXPERIMENTAL, the first plugin that moves robots: keep the e-stops in hand.
"""

from __future__ import annotations

import asyncio
import math
import traceback
from typing import TYPE_CHECKING, Any, Mapping

from scaffolding_assembly_core.ids import robot_id

from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...shared.execution.basis import STALE_BASE
from ...shared.execution.interlock import Busy, give_back, take
from ...shared.execution.operator import PromptBox
from ...shared.execution.send import motion_command
from ...shared.movement_exec import (CANCELLED, DONE, DRY_RUN, FAILED, HOLD_NOT_SENT, MAX_TRAJ_TIME, MOTION_PART,
                                     REFUSED, REFUSED_PART, ExecRequest, ExecResult, serial_of)
from ...shared.planning.route_result import pose_error
from .panel import ExecPanel
from .sequences import Job, PartRun, run_part

if TYPE_CHECKING:
    from ...robot_interface.robot import Lease
    from ...shared.execution.operator import Prompt

#: Why a request was cancelled when this plugin did not cancel it: the soft stop, or the plugin closing.
STOP_ALL, CLOSED = "Stop all (the soft stop)", "the plugin stop or shutdown"


def cancelled_by(ctx: PluginContext) -> str:
    """Who cancelled a task this plugin did not: the plugin closing (stopped or shut down), else the soft stop."""
    return CLOSED if ctx.closed else STOP_ALL


@register
class MovementExecPlugin(HuskyPlugin):
    """Runs one step's parts at a time on its robot, each after the operator's Confirm, and records them."""

    name = "movement_exec"
    requires = ("motion_planner", "assembly_run")
    experimental = True

    def __init__(self):
        """Idle, the dry run off."""
        self._ctx: PluginContext | None = None
        self._panel: ExecPanel | None = None
        self._prompts = PromptBox()
        self._dry_run = False
        #: The running request, its task and its label for the panel.
        self._task: asyncio.Task | None = None
        self._active = ""
        #: "operator" once Cancel (or a Hold) cancelled the running task; else a cancel came from outside.
        self._cancel_reason = ""
        #: The last result as one line, and whether it failed; how many dry-run changes were refused.
        self._last, self._last_failed = "", False
        self._dry_run_refused = 0

    # --- --- --- --- --- WHAT OTHER PLUGINS CALL (MovementExecPluginApi) --- --- --- --- ---

    @property
    def active(self) -> str:
        """str: "<robot> · <label>" while a request runs, else ""."""
        return self._active if self._task is not None and not self._task.done() else ""

    @property
    def prompt(self) -> Prompt | None:
        """Prompt | None: The question waiting for Confirm or Cancel."""
        return self._prompts.current

    @property
    def dry_run(self) -> bool:
        """bool: Whether requests run every check and send nothing."""
        return self._dry_run

    def request_execute(self, request: ExecRequest) -> asyncio.Future[ExecResult]:
        """Run a step's parts, holding its robot until done; a refusal resolves the future at once. Main thread."""
        ctx = self._ctx
        future = asyncio.get_running_loop().create_future()
        refusal = self._refusal(ctx, request)
        if not refusal:
            try:
                lease = take(ctx.world, request.serial, self.name, request.label, self.request_cancel, ctx.now())
            except Busy as busy:
                refusal = f"busy: {busy}"
        if refusal:
            result = ExecResult(REFUSED, refusal)
            self._finished(ctx, request, result)
            future.set_result(result)
            return future
        job = Job(ctx, request, ctx.world.robots[request.serial], ctx.require("assembly_run"), self._prompts,
                  self._dry_run, lambda name, movement, data: self._event(job, name, movement, data))
        self._active, self._cancel_reason = f"{request.robot_name} · {request.label}", ""
        ctx.log_info(f"Exec {request.label} on {job.who}: {', '.join(part.movement for part in request.parts)}"
                     + (" (DRY RUN)" if job.dry_run else ""))
        self._task = ctx.spawn(f"exec {request.label}", self._run(job, lease, future))
        # * A safety net for a task cancelled before it started: its own `finally` never ran.
        self._task.add_done_callback(lambda task: self._ended(ctx, task, lease, future))
        return future

    def request_confirm(self) -> None:
        """Confirm the open prompt; a click too soon after it opened is ignored, with the reason logged."""
        why = self._prompts.answer(True, self._ctx.now())
        if why:
            self._ctx.log_info(why)
            self._last, self._last_failed = why, True

    def request_cancel(self) -> None:
        """Cancel the open prompt, else the running request (every arm is held); also the robot lease's stop."""
        if self._prompts.current is not None:
            self._prompts.answer(False, self._ctx.now())
            self._ctx.log_warn("Cancel: the prompt is answered Cancel")
        elif self._task is not None and not self._task.done():
            self._cancel_reason = "operator"
            self._ctx.log_warn(f"Cancel: stopping {self._active}")
            self._task.cancel()
        else:
            self._ctx.log_info("Cancel: nothing to cancel")

    def request_dry_run(self, on: bool) -> None:
        """Turn the dry run on or off; refused while a request runs (it keeps the setting it started with)."""
        if bool(on) == self._dry_run:
            return
        if self.active:
            self._dry_run_refused += 1
            self._ctx.log_warn(f"Dry run not changed: {self._active} runs; change it once it is done")
            return
        self._dry_run = bool(on)
        self._ctx.log_warn(f"dry run {'ON: every check, nothing sent' if on else 'OFF: Exec moves the robots'}")

    # --- --- --- --- --- HOOKS --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Keep the context for the requests; build the panel and the phone's Exec tab."""
        self._ctx = ctx
        self._panel = ExecPanel(ctx, self)

    def draw(self, ctx: PluginContext) -> None:
        """Fill the panel."""
        if self._panel is not None:
            self._panel.draw(self.active, self.prompt, self._dry_run, (self._last, self._last_failed),
                             self._dry_run_refused)

    # --- --- --- --- --- RUNNING A REQUEST --- --- --- --- ---

    def _refusal(self, ctx: PluginContext, request: ExecRequest) -> str:
        """Why a request may not start; "" if it may."""
        if self.active:
            return f"busy: {self._active} runs"
        refused = next((part for part in request.parts if part.kind == REFUSED_PART), None)
        if refused is not None:
            return f"{refused.movement}: {refused.text}"
        if not request.parts:
            return "nothing to run in this step"
        serial = serial_of(request.loaded, request.action, ctx.config.robots)
        if serial != request.serial:
            return f"{request.action} is for {serial or 'no configured robot'}, not {request.serial}"
        if request.serial not in ctx.world.robots:
            return f"{request.serial} is not connected"
        if not 0.0 < request.traj_time <= MAX_TRAJ_TIME:
            return f"the motion time must be over 0 and at most {MAX_TRAJ_TIME:.0f} s, not {request.traj_time}"
        motions = [part for part in request.parts if part.kind == MOTION_PART]
        return self._plan_refusal(ctx, request) if motions else ""

    @staticmethod
    def _plan_refusal(ctx: PluginContext, request: ExecRequest) -> str:
        """Why the request's plan is not the step's motion on this robot at its base now; "" if it is."""
        motions = [part.index for part in request.parts if part.kind == MOTION_PART]
        route, basis, primary = request.route, request.basis, request.step.primary
        if motions != [primary]:
            return f"the only motion of a step must be its primary, movement {primary}"
        trajectory = route.result.trajectory if route is not None and route.ok else None
        if trajectory is None:
            return "no plan for the motion: Plan first"
        acting = request.loaded.process.actions[request.action].robot
        if trajectory.robot != acting:
            return f"the plan is for {trajectory.robot}, the step is {acting}'s"
        if request.route_movement != primary:
            return f"the plan is for movement {request.route_movement}, the step moves in movement {primary}"
        planned_for = None if basis is None else (basis.serial, basis.action, basis.movement_index)
        if planned_for != (request.serial, request.action, primary):
            return "the plan was made for another robot or step"
        live = ctx.scene.snapshot.robots.get(robot_id(request.serial))
        if live is None:
            return f"{request.serial} is not in the scene"
        distance, turn = pose_error(live.base, basis.base)
        if distance > STALE_BASE[0] or turn > STALE_BASE[1]:
            return (f"{request.serial}'s base is {distance * 1e3:.1f} mm, {math.degrees(turn):.2f}° from where the "
                    f"plan was made: replan")
        try:
            motion_command(request.serial, list(ctx.world.robots[request.serial].arms), trajectory.joint_names,
                           trajectory.positions, request.traj_time)
        except ValueError as error:
            return f"the plan can't be sent to {request.serial}: {error}"
        return ""

    async def _run(self, job: Job, lease: Lease, future: asyncio.Future) -> None:
        """Run the parts in order, stopping at the first not done; give the robot back and resolve the future after."""
        ctx, request = job.ctx, job.request
        done: list[int] = []
        end = base = None
        result = ExecResult(FAILED, "ended before its first part")
        try:
            for part in request.parts:
                run = PartRun(part)
                try:
                    await run_part(job, run)
                except asyncio.CancelledError:
                    result = self._cancelled(job, run, tuple(done), end, base)
                    raise
                except Exception as error:
                    result = self._raised(job, run, error, tuple(done), end, base)
                    raise
                self._record_part(job, run)
                if run.end is not None:
                    end, base = run.end, run.base
                if run.result == DONE:
                    done.append(part.index)
                elif run.result != DRY_RUN:
                    result = ExecResult(run.result, f"{part.movement}: {run.reason}", tuple(done), end, base)
                    break
            else:
                result = ExecResult(DRY_RUN, "dry run: every check passed, nothing sent", (), end, base) \
                    if job.dry_run else ExecResult(DONE, "every part done", tuple(done), end, base)
        finally:
            give_back(ctx.world, lease)
            self._prompts.current = None
            if not future.done():
                self._finished(ctx, request, result)
                future.set_result(result)

    def _cancelled(self, job: Job, run: PartRun, done: tuple[int, ...], end, base) -> ExecResult:
        """Record a part cut short by a cancel (where the arms stopped, the holds) and say who cancelled it.

        Returns:
            ExecResult: CANCELLED, or HOLD_NOT_SENT when an arm refused its hold.
        """
        part, progress = run.part, run.progress
        reason = self._cancel_reason or cancelled_by(job.ctx)
        refused = progress.hold_not_sent if progress is not None else []
        if progress is not None:
            run.outcome = {"seconds": progress.seconds, "end": progress.end, "holds": dict(progress.holds)}
            if refused:
                # ! A hold is refused on stale joints: then "end" is where they were last seen, not where it stopped.
                run.outcome.update(end_stale=progress.end_stale, end_age=dict(progress.end_age))
        if refused:
            run.ended(HOLD_NOT_SENT, HOLD_NOT_SENT, f"HOLD NOT SENT on {', '.join(refused)}: stop {job.request.serial} "
                                                    f"with the teach pendant or the e-stop (cancelled: {reason})")
        else:
            run.ended(CANCELLED, "cancelled", f"cancelled ({reason})")
        job.event("cancel", part.movement, {"reason": reason})
        self._record_part(job, run)
        if reason == STOP_ALL:
            job.event("soft_stop", part.movement, {"part": part.index})
        return ExecResult(run.result, f"{part.movement}: {run.reason}", done, end, base)

    def _raised(self, job: Job, run: PartRun, error: Exception, done: tuple[int, ...], end, base) -> ExecResult:
        """Hold every arm after a part raised, record it with the error, and let the monitor count the failure.

        ! A bug must not leave an arm moving.

        Returns:
            ExecResult: FAILED, or HOLD_NOT_SENT when an arm refused its hold.
        """
        holds = {name: bool(arm.hold()) for name, arm in job.robot.arms.items()}
        refused = [name for name, sent in holds.items() if not sent]
        job.ctx.log_error(f"Exec {job.request.label} raised; holding every arm of {job.who}:\n"
                          f"{traceback.format_exc()}")
        run.outcome = {"holds": holds}
        if refused:
            run.ended(HOLD_NOT_SENT, HOLD_NOT_SENT, f"HOLD NOT SENT on {', '.join(refused)}: stop "
                                                    f"{job.request.serial} with the teach pendant or the e-stop "
                                                    f"(the executor raised {error!r})")
        else:
            run.ended(FAILED, "raised", f"the executor raised {error!r}; every arm held")
        self._record_part(job, run)
        job.event("error", run.part.movement, {"error": repr(error)})
        return ExecResult(run.result, f"{run.part.movement}: {run.reason}", done, end, base)

    def _ended(self, ctx: PluginContext, task: asyncio.Task, lease: Lease, future: asyncio.Future) -> None:
        """Give the robot back once the task is over, however it ended; resolve a future it never got to."""
        give_back(ctx.world, lease)
        if self._task is task:
            self._prompts.current = None
        if not future.done():
            result = ExecResult(CANCELLED, f"cancelled before it started ({cancelled_by(ctx)})")
            self._finished(ctx, None, result)
            future.set_result(result)

    def _finished(self, ctx: PluginContext, request: ExecRequest | None, result: ExecResult) -> None:
        """Log a request's end and show it on the panel."""
        label = request.label if request is not None else "a request"
        self._last = f"{label}: {result.status} — {result.reason}"
        self._last_failed = result.status not in (DONE, DRY_RUN)
        if result.status == HOLD_NOT_SENT:
            ctx.log_error(f"Exec {self._last}")
        else:
            (ctx.log_warn if self._last_failed else ctx.log_info)(f"Exec {self._last}")

    # --- --- --- --- --- RECORDS --- --- --- --- ---

    def _event(self, job: Job, name: str, movement: str, data: Mapping[str, Any]) -> None:
        """Record a prompt, confirm or cancel in the run."""
        job.run.request_record(name, action=job.request.action, movement=movement, data=dict(data))

    def _record_part(self, job: Job, run: PartRun) -> None:
        """Record one part in the run: what it was, what it was planned from, what was sent, and how it ended."""
        request, part = job.request, run.part
        motion = part.kind == MOTION_PART
        route = request.route if motion else None
        trajectory = route.result.trajectory if route is not None else None
        plan = None if trajectory is None else {
            "planner": route.planner, "attempts": route.attempts, "seconds": route.seconds,
            "joint_names": list(trajectory.joint_names), "rows": [list(row) for row in trajectory.positions],
            "goal_joints": route.goal_joints, "end_error": {flange: list(error) for flange, error in
                                                            route.end_error.items()},
            "warnings": list(route.warnings)}
        payload = {
            "movement": part.movement, "routine": part.routine.value if part.routine is not None else None,
            "robot": request.robot_name, "serial": request.serial,
            "step": {"primary": request.step.primary, "indices": list(request.step.indices)},
            "part": {"index": part.index, "kind": part.kind, "tools": list(part.tools), "text": part.text},
            "basis": request.basis.as_record() if motion and request.basis is not None else None,
            "plan": plan, "command": run.command,
            "outcome": {**(run.outcome or {}), "result": run.result, "status": run.status, "reason": run.reason,
                        "dry_run": job.dry_run}}
        job.run.request_record("exec", action=request.action, movement=part.movement,
                               data={"status": run.status, "result": run.result}, file=payload)
