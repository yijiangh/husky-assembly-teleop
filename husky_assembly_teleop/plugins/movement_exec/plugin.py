"""
Run one step of the assembly on the real robot: its motion along the plan, its tool steps, its steps done by hand.

Requests come through `MovementExecContract`; a motion runs on Exec, a tool step or a step by hand after the
operator's Confirm, and each is recorded in the assembly run, the servo part (`servo_part.py`) pass by pass. Every
robot is leased from the request to its end, so nothing else here moves a robot, and every check runs again right
before a send, with the command rebuilt from the live joints in the same tick. Cancel or Stop all ends it with every
arm held; so does another robot starting to move, or one displaced (`ctx.freeze`, pushes too) or lost to mocap while
a motion runs.

- ! EXPERIMENTAL, the first plugin that runs process movements on the robots: keep the e-stops in hand.
"""

from __future__ import annotations

import asyncio
import math
import traceback
from typing import TYPE_CHECKING, Any, Mapping

import viser
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id

from ...config import short_serial
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...robot_interface.robot_lease import NotLeased
from ...shared.assembly_planning.movement import Routine
from ...shared.assembly_planning.route_result import pose_error
from ...shared.execution.basis import STALE_BASE
from ...shared.execution.moving import moving_reason
from ...shared.execution.operator import PromptBox
from ...shared.execution.send import INTERRUPTED, motion_command
from ...shared.movement_exec_contract import (CANCELLED, DONE, DRY_RUN, FAILED, HANDOFF_PART, HOLD_NOT_SENT,
                                              INSERT_PART, MAX_TRAJ_TIME, MOTION_KINDS, MOTION_PART, REFUSED,
                                              REFUSED_PART, RETREAT_PART, SERVO_INDEX, SERVO_PART, WITH_MOTION_PART,
                                              Carried, ExecRequest, ExecResult, serial_of)
from ...shared.assembly_tab.layout import ASSEMBLY_TAB, EXEC_PLOTS_X, plots_window
from ...shared.runs.as_built import KIND as AS_BUILT_KIND, bar_lines
from .as_built import as_built_record
from .panel import ExecPanel
from .plots import ExecPlots
from .sequences import Job, PartRun, run_part, unknown_serials
from .servo_part import run_servo

if TYPE_CHECKING:
    from ...robot_interface.robot_lease import Lease
    from ...shared.execution.operator import Prompt

#: The routines whose motion part (and servo) start a transfer anew: the carry of an earlier servo is dropped.
TRANSFERS = (Routine.TRANSFER_FREE, Routine.TRANSFER_HELD)

#: Why a request was cancelled when this plugin did not cancel it: the soft stop, or the plugin closing.
STOP_ALL, CLOSED = "Stop all (the soft stop)", "the plugin stop or shutdown"


def cancelled_by(ctx: PluginContext) -> str:
    """Who cancelled a task this plugin did not: the plugin closing (stopped or shut down), else the soft stop."""
    return CLOSED if ctx.closed else STOP_ALL


#: Parts no as-built record is taken around: nothing moves in them.
AS_BUILT_SKIPPED = frozenset({WITH_MOTION_PART, REFUSED_PART})
#: The parts that run partly under the compliance controller, whose force and torque the Exec plots window shows.
COMPLIANT_KINDS = (INSERT_PART, RETREAT_PART, HANDOFF_PART)


@register
class MovementExecPlugin(HuskyPlugin):
    """Runs one step's parts in order on its robot (a motion on Exec, the rest after Confirm) and records them."""

    name = "movement_exec"
    requires = ("motion_planner", "assembly_run", "mocap_bars")
    experimental = True
    tab_path = (ASSEMBLY_TAB,)
    icon = viser.Icon.PLAYER_PLAY

    def __init__(self):
        """Idle, the dry run off."""
        self._ctx: PluginContext | None = None
        self._panel: ExecPanel | None = None
        self._prompts = PromptBox()
        self._dry_run = False
        #: The running request, its task and its label for the panel.
        self._job: Job | None = None
        self._task: asyncio.Task | None = None
        self._active = ""
        #: "operator" once Cancel cancelled the running task; else a cancel came from outside.
        self._cancel_reason = ""
        #: The last result as one line, whether it failed and whether it left a hold not sent; the run's current step;
        #: how many dry-run changes were refused.
        self._last, self._last_failed, self._hold_lost = "", False, False
        self._step: tuple[str, int] | None = None
        self._dry_run_refused = 0
        #: Serial -> ROS time its arms last arrived (a motion or a servo pass): an older plan is refused.
        self._moved_at: dict[str, float] = {}
        #: Serial -> {at, reason} of its force sensors' last zero, for the as-built records of later requests.
        self._ft_zero: dict[str, dict] = {}
        #: What the last transfer's servo left for its insert's plan; the panel's servo line: the running servo's
        #: last reading, or how the last one ended, until the next request.
        self._carried: Carried | None = None
        self._servo_line = ""
        #: Action id -> the loaded process and run folder, and the (step primary, movement index) finished for real
        #: there (`finished`): a new run or another process starts with none.
        self._finished_moves: dict[str, tuple[object, object, set[tuple[int, int]]]] = {}
        #: The Exec plots window's plots, and the time of the compliant sample last plotted.
        self._plots: ExecPlots | None = None
        #: What was said once about nothing to record (the process without markers, no plugin), not to repeat it.
        self._no_bars_said = None
        self._plotted_t: float | None = None

    # --- --- --- --- --- WHAT OTHER PLUGINS CALL (MovementExecContract) --- --- --- --- ---

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

    @property
    def carried(self) -> Carried | None:
        """Carried | None: What the last transfer's servo left for its insert's plan; dropped on a reload."""
        if self._carried is not None and self._ctx.require("assembly_run").view.process is not self._carried.loaded:
            self._carried = None
        return self._carried

    def finished(self, loaded, action: str) -> frozenset[tuple[int, int]]:
        """(step primary, movement index) of `action` finished for real by any request on `loaded`."""
        kept = self._finished_moves.get(action)
        folder = self._ctx.require("assembly_run").view.folder
        return frozenset(kept[2]) if kept is not None and kept[0] is loaded and kept[1] == folder else frozenset()

    def request_forget(self, action: str, done=None) -> None:
        """Forget that `action`'s movements (only the (primary, index) pairs in `done`, if given) were finished."""
        kept = self._finished_moves.get(action)
        if kept is None:
            return
        if done is None:
            del self._finished_moves[action]
        else:
            kept[2].difference_update(done)

    def request_execute(self, request: ExecRequest) -> asyncio.Future[ExecResult]:
        """Run a step's parts, leasing every robot until done; a refusal resolves the future at once.

        ! Main thread.
        """
        ctx = self._ctx
        future = asyncio.get_running_loop().create_future()
        refusal = self._refusal(ctx, request)
        if not refusal and (moving := moving_reason(ctx.world, ctx.now(),
                                                    unknown=unknown_serials(ctx.world, request))):
            refusal = f"busy: {moving}"
        if not refusal:
            try:
                # * Every robot's lease, for the whole request (prompts and parts done by hand included): its sends
                #   need it, and no other plugin moves any robot meanwhile (one motion at a time).
                lease = ctx.take_lease(ctx.world.robots, f"exec {request.label}")
            except NotLeased as refused:
                refusal = f"busy: {refused}"
        if refusal:
            result = ExecResult(REFUSED, refusal)
            self._finished(ctx, request, result)
            future.set_result(result)
            return future
        try:
            job = Job(ctx, request, ctx.world.robots[request.serial], ctx.require("assembly_run"), self._prompts,
                      self._dry_run or request.dry_run,
                      lambda name, movement, data: self._event(job, name, movement, data),
                      self._moved_at, lambda line: setattr(self, "_servo_line", line),
                      measured=self._plots.servo_reading if self._plots is not None else (lambda _reading: None),
                      status=self._plots.status if self._plots is not None else (lambda _text, _color: None),
                      ft_zero=self._ft_zero)
            self._job, self._active, self._cancel_reason = job, f"{request.robot_name} · {request.label}", ""
            self._last, self._last_failed, self._hold_lost = "", False, False
            ctx.log_info(f"Exec {request.label} on {job.who}: {', '.join(part.movement for part in request.parts)}"
                         + (" (DRY RUN)" if job.dry_run else "") + f"; {_safety_line(request)}")
            self._task = ctx.spawn(f"exec {request.label}", self._run(job, lease, future))
        except BaseException:
            # ! No task runs to give them back (e.g. the plugin is closing, so spawn refuses): every other plugin
            #   would be refused every lease until a restart.
            lease.release()
            self._active = ""
            raise
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
        """Cancel the open prompt, else the running request (every arm is held)."""
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
        """Keep the context for the requests; build the panel, the phone's Exec tab and the Exec plots window."""
        self._ctx = ctx
        self._panel = ExecPanel(ctx, self)
        # * A window at the bottom left, beside the plan plots, minimized to its bar until a servo or a compliant
        #   part starts (`ExecPlots`).
        with plots_window(ctx.view, "Exec plots", viser.Icon.CHART_DOTS, EXEC_PLOTS_X) as panel:
            self._plots = ExecPlots(ctx.view.gui, panel)

    def update(self, ctx: PluginContext) -> None:
        """Drop a failure once the step changes; a hold not sent stays until the next request: an arm may move."""
        current = ctx.require("assembly_run").view.current
        if current != self._step:
            self._step = current
            if self._last_failed and not self._hold_lost:
                self._last, self._last_failed = "", False

    def draw(self, ctx: PluginContext) -> None:
        """Fill the panel, and the Exec plots with the running compliant part's newest sample."""
        if self._panel is not None:
            recorder = self._job.recorder if self.active and self._job is not None else None
            self._panel.draw(self.active, self.prompt, self._dry_run, (self._last, self._last_failed),
                             self._dry_run_refused, recorder, self._servo_line)
            if self._plots is not None:
                if recorder is not None and recorder.samples and recorder.samples[-1]["t"] != self._plotted_t:
                    self._plotted_t = recorder.samples[-1]["t"]
                    self._plots.compliant_sample(recorder.samples[-1])
                self._plots.draw()

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
            return (f"{request.action} is for {short_serial(serial) if serial else 'no configured robot'}, not "
                    f"{short_serial(request.serial)}")
        if request.serial not in ctx.world.robots:
            return f"{short_serial(request.serial)} is not connected"
        if not 0.0 < request.traj_time <= MAX_TRAJ_TIME:
            return f"the motion time must be over 0 and at most {MAX_TRAJ_TIME:.0f} s, not {request.traj_time}"
        motions = [part for part in request.parts if part.kind in MOTION_KINDS]
        return self._plan_refusal(ctx, request) if motions else ""

    def _plan_refusal(self, ctx: PluginContext, request: ExecRequest) -> str:
        """Why the request's plan is not the step's motion on this robot at its base now; "" if it is."""
        motions = [part.index for part in request.parts if part.kind in MOTION_KINDS]
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
        if basis.time < self._moved_at.get(request.serial, -math.inf):
            return "the arms moved since this plan was made: replan"
        live = ctx.scene.snapshot.robots.get(robot_id(request.serial))
        if live is None:
            return f"{short_serial(request.serial)} is not in the scene"
        distance, turn = pose_error(live.base, basis.base)
        if distance > STALE_BASE[0] or turn > STALE_BASE[1]:
            return (f"{short_serial(request.serial)}'s base is {distance * 1e3:.1f} mm, {math.degrees(turn):.2f}° from "
                    f"where the plan was made: replan")
        try:
            motion_command(request.serial, list(ctx.world.robots[request.serial].arms), trajectory.joint_names,
                           trajectory.positions, request.traj_time)
        except ValueError as error:
            return f"the plan can't be sent to {short_serial(request.serial)}: {error}"
        return ""

    async def _run(self, job: Job, lease: Lease, future: asyncio.Future) -> None:
        """Run the parts in order, stopping at the first not done; give the robots back and resolve the future after.

        Args:
            job: The request being run.
            lease: Every robot's lease, which this task has while it runs: its commands need it.
            future: Resolved with the result.
        """
        ctx, request = job.ctx, job.request
        done: list[int] = []
        end = base = None
        result = ExecResult(FAILED, "ended before its first part")
        if self._carried is not None and self._carried.action != request.action:
            self._carried = None
        self._servo_line = ""
        first = True
        try:
            with lease.use():
                for part in request.parts:
                    run = PartRun(part)
                    if first and part.kind not in AS_BUILT_SKIPPED:
                        # * The moment before anything moves; every later start is the end of the part before.
                        self._as_built(job, run, "start")
                        first = False
                    if part.routine in TRANSFERS and part.kind in (MOTION_PART, SERVO_PART):
                        self._carried = None  # the transfer runs anew: its servo sets the carry again when due
                    if self._plots is not None and part.kind == SERVO_PART:
                        self._plots.start_servo(f"{part.movement} on {short_serial(request.serial)}")
                    elif self._plots is not None and part.kind in COMPLIANT_KINDS:
                        self._plots.start_compliant(list(job.robot.arms),
                                                    f"{part.movement} on {short_serial(request.serial)}")
                        self._plotted_t = None
                    try:
                        await (run_servo if part.kind == SERVO_PART else run_part)(job, run)
                    except asyncio.CancelledError:
                        result = self._cancelled(job, run, tuple(done), end, base)
                        raise
                    except Exception as error:
                        result = self._raised(job, run, error, tuple(done), end, base)
                        raise
                    finally:
                        # * However the servo ended: it moved the flanges, so the insert plans from where it left them.
                        if run.carry is not None:
                            self._carried = run.carry
                    self._record_part(job, run)
                    if run.end is not None:
                        end, base = run.end, run.base
                        self._moved_at[request.serial] = ctx.now()
                    if run.result == DONE:
                        done.extend((part.index, *part.also))
                        if part.kind in MOTION_KINDS:
                            request.preview(None, None, 0.0)  # * the plan has run: it is no longer what comes next
                    elif run.result != DRY_RUN:
                        result = ExecResult(run.result, f"{part.movement}: {run.reason}", tuple(done), end, base)
                        break
                else:
                    result = ExecResult(DRY_RUN, "dry run: every check passed, nothing sent", (), end, base) \
                        if job.dry_run else ExecResult(DONE, "every part done", tuple(done), end, base)
        finally:
            # ! Given back even after HOLD_NOT_SENT, when an arm may still move: the operator recovers it with
            #   robot_control, which needs the lease.
            lease.release()
            self._prompts.current = None
            if not future.done():
                self._finished(ctx, request, result)
                future.set_result(result)

    def _cancelled(self, job: Job, run: PartRun, done: tuple[int, ...], end, base) -> ExecResult:
        """Record a part cut short by a cancel (where the arms stopped, the holds) and say who cancelled it.

        Returns:
            ExecResult: CANCELLED; FAILED when the freeze broke (another robot moved) or the clearance tripped;
                REFUSED when the clearance found the start too close, nothing sent; HOLD_NOT_SENT when an arm refused
                its hold.
        """
        part, progress = run.part, run.progress
        clearance = None if self._cancel_reason else run.clearance
        if clearance is not None and clearance.refused:
            run.ended(REFUSED, "refused", f"nothing sent: {clearance.problem}; untick Clearance to move out")
            job.event("cancel", part.movement, {"reason": "clearance refused the start"})
            self._record_part(job, run)
            return ExecResult(run.result, f"{part.movement}: {run.reason}", done, end, base)
        moved = None if self._cancel_reason or run.freeze is None else run.freeze.problem
        reason = self._cancel_reason or (f"freeze broken: {moved}" if moved else cancelled_by(job.ctx))
        if moved is None and clearance is not None and clearance.problem:
            moved = f"soft stopped: {clearance.problem}"
            reason = f"clearance tripped: {clearance.problem}"
        refused = progress.hold_not_sent if progress is not None else []
        if progress is not None:
            run.outcome = {"seconds": progress.seconds, "end": progress.end, "holds": dict(progress.holds)}
            if refused:
                # ? A hold is refused on stale joints: then "end" is where they were last seen, not where it stopped.
                run.outcome.update(end_stale=progress.end_stale, end_age=dict(progress.end_age))
        if refused:
            run.ended(HOLD_NOT_SENT, HOLD_NOT_SENT, f"HOLD NOT SENT on {', '.join(refused)}: stop "
                                                    f"{short_serial(job.request.serial)} with the teach pendant or the "
                                                    f"e-stop (cancelled: {reason})")
        elif moved:
            run.ended(FAILED, INTERRUPTED, f"{moved}; every arm held")
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
                                                    f"{short_serial(job.request.serial)} with the teach pendant or the "
                                                    f"e-stop (the executor raised {error!r})")
        else:
            run.ended(FAILED, "raised", f"the executor raised {error!r}; every arm held")
        self._record_part(job, run)
        job.event("error", run.part.movement, {"error": repr(error)})
        return ExecResult(run.result, f"{run.part.movement}: {run.reason}", done, end, base)

    def _ended(self, ctx: PluginContext, task: asyncio.Task, lease: Lease, future: asyncio.Future) -> None:
        """Give the robots back once the task is over, however it ended; resolve a future it never got to."""
        lease.release()
        if self._task is task:
            self._prompts.current = None
        if not future.done():
            result = ExecResult(CANCELLED, f"cancelled before it started ({cancelled_by(ctx)})")
            self._finished(ctx, None, result)
            future.set_result(result)

    def _finished(self, ctx: PluginContext, request: ExecRequest | None, result: ExecResult) -> None:
        """Log a request's end and show it on the panel."""
        if request is not None and result.done:
            folder = ctx.require("assembly_run").view.folder
            kept = self._finished_moves.get(request.action)
            if kept is None or kept[0] is not request.loaded or kept[1] != folder:
                kept = self._finished_moves[request.action] = (request.loaded, folder, set())
            kept[2].update((request.step.primary, index) for index in result.done)
        label = request.label if request is not None else "a request"
        self._last = f"{label}: {result.status} — {result.reason}"
        self._last_failed = result.status not in (DONE, DRY_RUN)
        self._hold_lost = result.status == HOLD_NOT_SENT
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
        motion = part.kind in MOTION_KINDS
        route = request.route if motion else None
        trajectory = route.result.trajectory if route is not None else None
        plan = None if trajectory is None else {
            "planner": route.planner, "attempts": route.attempts, "seconds": route.seconds,
            "joint_names": list(trajectory.joint_names), "rows": [list(row) for row in trajectory.positions],
            "goal_joints": route.goal_joints, "end_error": {flange: list(error) for flange, error in
                                                            route.end_error.items()},
            "warnings": list(route.warnings)}
        # * A folded part (the bar placed and clamped) is one record, named after its first movement.
        movement = request.loaded.process.actions[request.action].movements[part.index].id if part.also \
            else part.movement
        payload = {
            "movement": movement, "routine": part.routine.value if part.routine is not None else None,
            "robot": request.robot_name, "serial": request.serial,
            "step": {"primary": request.step.primary, "indices": list(request.step.indices)},
            "part": {"index": part.index, "also": list(part.also), "kind": part.kind, "tools": list(part.tools),
                     "text": part.text},
            "basis": request.basis.as_record() if motion and request.basis is not None else None,
            "plan": plan, "command": run.command, "safety": _safety(request),
            # * Where the robot stood and when, at the part's start and end, as measured then.
            "exec": {"ros_time_start": run.started_at, "ros_time_end": run.ended_at,
                     "base_at_start": _pose_record(run.base_at_start), "base_at_end": _pose_record(run.base)},
            "outcome": {**(run.outcome or {}), "result": run.result, "status": run.status, "reason": run.reason,
                        "dry_run": job.dry_run}}
        if part.kind == SERVO_PART:
            payload["servo_file"] = run.servo_file
        job.run.request_record("exec", action=request.action, movement=movement,
                               data={"status": run.status, "result": run.result}, file=payload)
        if part.kind not in AS_BUILT_SKIPPED:
            self._as_built(job, run, "end")

    # --- --- --- --- --- THE AS-BUILT RECORD --- --- --- --- ---

    def request_as_built(self) -> None:
        """Measure every MoCap bar now and append an as-built record (phase "manual"); the current step's context."""
        ctx = self._ctx
        view = ctx.require("assembly_run").view
        if view.process is None:
            ctx.log_warn("[as-built] no process loaded: nothing recorded")
            return
        if self._job is not None and self.active:
            job = self._job
            self._as_built(job, None, "manual")
            return
        action_id, movement_index = view.current if view.current is not None else (None, None)
        serial = serial_of(view.process, action_id, ctx.config.robots) if action_id else None
        if serial is None or serial not in ctx.world.robots:
            serial = next(iter(ctx.world.robots), None)
        if serial is None:
            ctx.log_warn("[as-built] no robot: nothing recorded")
            return
        self._record_as_built(ctx, view, "manual", action_id, movement_index, serial, ctx.world.robots[serial],
                              None, None, False, {})

    def _as_built(self, job: Job, run: PartRun | None, phase: str) -> None:
        """An as-built record at a part's start or end (or a click mid-request); never raises, the step goes on."""
        request = job.request
        part = run.part if run is not None else None
        movement_index = None if part is None else (request.step.primary if part.index == SERVO_INDEX
                                                    else part.index)
        self._record_as_built(job.ctx, job.run.view, phase, request.action, movement_index, request.serial,
                              job.robot, request, run, job.dry_run, job.ft_zero)

    def _record_as_built(self, ctx: PluginContext, view, phase: str, action_id: str | None,
                         movement_index: int | None, serial: str, robot, request: ExecRequest | None,
                         run: PartRun | None, dry_run: bool, ft_zero: Mapping[str, Mapping]) -> None:
        """Build the record and hand it to assembly_run; an error is one log line."""
        movement = run.part.movement if run is not None else "(no step loaded)"
        if request is not None and run is None:
            movement = request.label
        ended = f" ({run.result or 'running'})" if run is not None and phase == "end" else ""
        head = f"[as-built] {movement} {phase}{ended}"
        try:
            bars_plugin = ctx.require("mocap_bars")
        except KeyError:
            if self._no_bars_said != "no plugin":
                self._no_bars_said = "no plugin"
                ctx.log_info("[as-built] the mocap_bars plugin is not loaded: nothing recorded")
            return
        try:
            record = as_built_record(ctx, bars_plugin, view.process, view, action_id=action_id,
                                     movement_index=movement_index, serial=serial, robot=robot, phase=phase,
                                     request=request, run=run, dry_run=dry_run, ft_zero=ft_zero)
            if record is None:
                if self._no_bars_said != view.process.source:
                    self._no_bars_said = view.process.source
                    ctx.log_info("[as-built] no bar with MoCap markers in this process: nothing recorded")
                return
            for line in bar_lines(record):
                ctx.log_info(f"{head}: {line}")
            ctx.require("assembly_run").request_record(
                AS_BUILT_KIND, action=action_id, movement=movement if run is not None else None,
                data={"phase": phase, "outcome": (record.get("step") or {}).get("outcome"),
                      "bars": len(record["bars"]),
                      "milestones": {bar: entry["milestone"] for bar, entry in record["bars"].items()}},
                file=record, kind=AS_BUILT_KIND)
        except Exception as error:
            ctx.log_error(f"[as-built] {phase} of {movement} NOT recorded: {error!r}")


def _pose_record(pose: Pose | None) -> list[float] | None:
    """A pose as [x, y, z, qx, qy, qz, qw] for the record, or None."""
    return None if pose is None else [*map(float, pose.position), *map(float, pose.orientation)]


def _safety(request: ExecRequest) -> dict:
    """The operator's safety choices for a request, as its records keep them."""
    return {"clearance": request.clearance, "ignore_built": request.ignore_built(),
            "unknown_robots": sorted(request.unknown_robots)}


def _safety_line(request: ExecRequest) -> str:
    """The operator's safety choices for a request, for its log line."""
    unknown = ", ".join(sorted(request.unknown_robots)) or "none"
    return (f"clearance {'on' if request.clearance else 'off'}, ignore built bars "
            f"{'on' if request.ignore_built() else 'off'}, robots let be unknown: {unknown}")
