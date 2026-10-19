"""
Remember how far an assembly got: which actions are done, where each robot was left, and the run's settings.

When the process plugin loads a process, this reads the runs in its folder (`shared/runs.py`). It continues the newest
run when that run was made on exactly these process files. Otherwise the operator chooses: Continue the newest run (in
a new run, carried over by action id), Continue an older run made on these files, or start a New run; with no run yet,
the first write starts one. Then the first pending action becomes current, and its first step is selected in the
process plugin once the action's robot is live (at most 15 s later, unless the operator picks a step first).

Mark done records every live robot where it is. When the action's own robot is not live, the operator confirms and
its end is the export's, marked assumed. Reopen puts a done action back to pending. Other plugins read `view` and send
requests (`shared.assembly_run.AssemblyRunPluginApi`).

- ! Nothing is written until the runs were read: an open that fails or is stopped is tried again.
- ! Runs are written into the process folder (`runs/`), in order on a writer thread; the state here changes first.
- ! EXPERIMENTAL: the run format and the requests may still change.
"""

from __future__ import annotations

import asyncio
from concurrent.futures import ThreadPoolExecutor
from dataclasses import fields, replace
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable, Coroutine, Mapping

from ...plugin_api.concurrency import WaitTimeout
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...shared.assembly_run import RunView, beliefs_of, exported_end, first_pending, is_live, live_record
from ...shared.planning.steps import first_step
from ...shared.process import LoadedProcess, live_ids
from ...shared.runs import (DONE, PENDING, ActionRecord, Progress, RobotRecord, Run, RunError, RunSettings,
                            continue_run, file_hashes, find_runs, mark_done, new_run, record_event, record_movement,
                            reopen, save_info, save_progress, stamp, with_current)
from .panel import PanelStatus, RunPanel

if TYPE_CHECKING:
    from scaffolding_assembly_core.ids import IdMap

    from ...shared.process import ProcessPluginApi

#: Seconds to wait for the current action's robot to be live before selecting its step anyway.
LIVE_WAIT = 15.0
#: Seconds the operator has to answer a prompt.
CONFIRM_TIMEOUT = 120.0
#: Seconds (ROS time) before reading the runs again after reading them failed.
RETRY_AFTER = 5.0
#: The insert split's range, millimetres.
SPLIT_RANGE = (0.0, 30.0)
#: How far the loaded process's runs were read; only OPEN allows writes.
NO_PROCESS, OPENING, OPEN, NOT_OPENED = "", "opening", "open", "not opened"


def _scan(process) -> tuple[tuple[str, dict], tuple[list[Run], list[str]]]:
    """The process's file hashes and the runs in its folder. Writer thread."""
    return file_hashes(process), find_runs(process.folder)


def _record_file(folder: Path, event: str, ros_time: float, action: str, movement: str,
                 payload: Mapping[str, Any], data: Mapping[str, Any] | None) -> None:
    """Write a movement record, then an event naming it. Writer thread.

    Args:
        folder: The run's folder.
        event: The event's name.
        ros_time: The monitor's clock.
        action: The action id.
        movement: The movement id.
        payload: The record's JSON values.
        data: The event's data, or None.
    """
    path = record_movement(folder, action, movement, payload)
    record_event(folder, event, ros_time, action=action, movement=movement, file=str(path.relative_to(folder)),
                 **({"data": dict(data)} if data else {}))


def _age(at: str) -> str:
    """How long ago a recorded local ISO time was, e.g. "12 min"; "?" if it can't be read."""
    try:
        seconds = (datetime.now().astimezone() - datetime.fromisoformat(at)).total_seconds()
    except (TypeError, ValueError):
        return "?"
    for size, unit in ((86400.0, "d"), (3600.0, "h"), (60.0, "min")):
        if seconds >= size:
            return f"{seconds // size:.0f} {unit}"
    return "under a minute"


@register
class AssemblyRunPlugin(HuskyPlugin):
    """Tracks which actions of the loaded process are done, and where the robots were left."""

    name = "assembly_run"
    requires = ("process",)
    experimental = True

    def __init__(self):
        """Start with no process."""
        self._ctx: PluginContext | None = None
        self._panel: RunPanel | None = None
        #: The process the run is for, as the process plugin loaded it, and its process ids -> live ids.
        self._loaded: LoadedProcess | None = None
        self._ids: IdMap | None = None
        #: How far its runs were read (NO_PROCESS, OPENING, OPEN, NOT_OPENED), and when to read them again.
        self._state, self._retry_at = NO_PROCESS, 0.0
        #: The run being written; else the newest run, offered, and an older run made on these files, offered too.
        self._run: Run | None = None
        self._offer: Run | None = None
        self._match: Run | None = None
        self._settings = RunSettings()
        #: (action id, movement index) the operator is at.
        self._current: tuple[str, int] | None = None
        #: The question waiting for Confirm or Cancel, and the answer once given.
        self._prompt = ""
        self._answer: bool | None = None
        #: One line of feedback, whether it is a failure, and the last write that failed.
        self._message, self._failed, self._save_error = "", False, ""
        #: How many settings requests were refused, so the panel puts its widgets back.
        self._settings_refused = 0
        self._revision = 0
        self._view: RunView | None = None
        # ! Every file read and write, in order: the state here changes first, the files follow.
        self._writer = ThreadPoolExecutor(1, "run-writer")
        # The run being started on the writer, so a second request waits for the same one.
        self._starting: asyncio.Future | None = None
        # This process's tasks (opening, prompts, starting a run), cancelled when another process loads.
        self._tasks: set[asyncio.Task] = set()
        # The process plugin's step index while the first pending action is still to be selected for the operator
        # (None once anything else selects), and the task waiting for its robot to be live.
        self._auto_from: int | None = None
        self._go_task: asyncio.Task | None = None

    # --- --- --- --- --- WHAT OTHER PLUGINS READ AND ASK (AssemblyRunPluginApi) --- --- --- --- ---

    @property
    def view(self) -> RunView:
        """RunView: The run now; the same object until something changes."""
        if self._view is None or self._view.revision != self._revision:
            progress = self._progress()
            schedule = self._loaded.process.schedule if self._loaded is not None else ()
            self._view = RunView(
                self._run.folder if self._run is not None else None, self._loaded,
                {action_id: progress.actions.get(action_id, ActionRecord()).status for action_id in schedule},
                frozenset(action_id for action_id, record in progress.actions.items()
                          if record.status == DONE and record.assumed),
                self._current, beliefs_of(progress), self._settings,
                self._offer.info.name if self._offer is not None else "",
                self._match.info.name if self._match is not None else "", self._revision)
        return self._view

    def request_set_current(self, action_id: str, movement_index: int | None = None) -> None:
        """Make an action current (at a movement it has, else its first step) and select it; never starts a run."""
        if self._refused("Select", self._unknown(action_id)):
            return
        self._stop_auto()
        self._set_current(self._ctx, action_id, movement_index)

    def request_mark_done(self, action_id: str | None = None) -> None:
        """Mark an action (the current one if None) done; asks to confirm when its robot is not live."""
        ctx = self._ctx
        action_id = action_id or self._current_action()
        if self._refused("Mark done", self._mark_refusal(ctx, action_id)):
            return
        self._stop_auto()
        acting = self._acting(ctx, action_id)
        ends = self._live_ends(ctx)
        if acting is None or acting in ends:
            if self._run is not None:
                self._apply_mark(ctx, action_id, ends, False)
            else:
                self._spawn(ctx, f"mark {action_id} done", self._mark(ctx, action_id, ends, False))
        else:
            robot = self._loaded.process.robots[self._loaded.process.actions[action_id].robot].name
            self._ask(f"{action_id} is {robot}'s; {robot} is not live: mark done with the export's end (assumed)?")
            self._spawn(ctx, f"confirm marking {action_id}", self._confirm_mark(ctx, action_id, acting, robot))

    def request_reopen(self, action_id: str | None = None) -> None:
        """Put a done action (the current one if None) back to pending, and make it current."""
        ctx = self._ctx
        action_id = action_id or self._current_action()
        if self._refused("Reopen", self._unknown(action_id) or self._busy() or self._no_run()):
            return
        try:
            run = reopen(self._run, action_id)
        except RunError as refused:
            self._refused("Reopen", str(refused))
            return
        self._stop_auto()
        self._run = run
        self._set_current(ctx, action_id)
        self._write(ctx, "record the reopen", record_event, run.folder, "reopen", ctx.now(), action=action_id)
        self._say(f"{action_id} reopened: pending again")
        ctx.log_info(f"{action_id} reopened in run {run.info.name}")

    def request_new_run(self) -> None:
        """Start a new run after the operator confirms; every action pending, the old run kept."""
        if not self._refused("New run", self._busy()):
            self._stop_auto()
            self._ask("Start a new run? Every action starts pending; the old run is kept.")
            self._spawn(self._ctx, "new run", self._new_run(self._ctx))

    def request_continue(self, run_name: str | None = None) -> None:
        """Continue an offered run: the newest if None; one made on another export goes on in a new run."""
        offered = [run for run in (self._offer, self._match) if run is not None]
        target = self._offer if run_name is None else next((run for run in offered if run.info.name == run_name), None)
        missing = "" if target is not None else (f"no offered run is named {run_name!r}" if run_name
                                                 else "no run is offered to continue")
        if not self._refused("Continue", self._busy() or missing):
            self._stop_auto()
            self._spawn(self._ctx, "continue run", self._continue(self._ctx, target))

    def request_settings(self, **changes: Any) -> None:
        """Change run settings (`RunSettings` fields); saved with the run, which this starts if there is none."""
        refusal = self._settings_problem(changes) or self._not_open() or self._offered_first()
        if refusal:
            self._settings_refused += 1
            self._refused("Settings", refusal)
            return
        settings = replace(self._settings, **{name: float(value) if name == "insert_split_mm" else value
                                              for name, value in changes.items()})
        if settings == self._settings:
            return
        self._settings = settings
        self._changed()
        self._spawn(self._ctx, "save settings", self._save_settings(self._ctx, changes))

    def request_record(self, event: str, *, action: str | None = None, movement: str | None = None,
                       data: Mapping[str, Any] | None = None, file: Mapping[str, Any] | None = None) -> None:
        """Record an event, `data` under its "data" key; `file` is written as a movement record the event names."""
        refusal = self._not_open() or (
            "a file needs its action and movement" if file is not None and None in (action, movement) else "")
        if self._refused(f"Record {event}", refusal):
            return
        if self._run is not None:
            self._queue_record(self._ctx, event, action, movement, data, file)
        else:
            self._spawn(self._ctx, f"record {event}", self._record_later(self._ctx, event, action, movement, data,
                                                                         file))

    # --- --- --- --- --- HOOKS --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Keep the context for the requests, and build the panel."""
        self._ctx = ctx
        self._panel = RunPanel(ctx, self)

    def update(self, ctx: PluginContext) -> None:
        """Open the run of a newly loaded process; open it again if that failed; notice a step picked meanwhile."""
        process: ProcessPluginApi = ctx.require("process")
        selection = process.selection
        loaded = selection.loaded if selection is not None else None
        if loaded is not self._loaded:
            self._reset(loaded)
            if loaded is not None:
                self._auto_from = selection.index
                self._open_now(ctx)
            return
        if loaded is None:
            return
        if self._auto_from is not None and selection.index != self._auto_from:
            self._stop_auto()
            ctx.log_info("a step was picked in the process panel: the first pending action is not selected")
        if self._state == NOT_OPENED and ctx.now() >= self._retry_at:
            self._open_now(ctx)

    def draw(self, ctx: PluginContext) -> None:
        """Fill the panel."""
        if self._panel is not None:
            status = PanelStatus(self._run.info.name if self._run is not None else "", self._prompt,
                                 (self._message, self._failed), self._save_error, tuple(self.robot_lines(ctx)),
                                 self._settings_refused)
            self._panel.draw(self.view, status)

    def teardown(self, ctx: PluginContext) -> None:
        """Wait for the queued writes."""
        self._writer.shutdown(wait=True)

    # --- --- --- --- --- FOR THE PANEL --- --- --- --- ---

    def answer(self, confirmed: bool) -> None:
        """Answer the prompt: Confirm or Cancel; ignored without one."""
        if not self._prompt:
            self._ctx.log_info(f"{'Confirm' if confirmed else 'Cancel'} ignored: nothing to answer")
            return
        self._answer = confirmed

    def robot_lines(self, ctx: PluginContext) -> list[str]:
        """One line per configured robot: live now, or when and where it was last recorded."""
        snapshot, now, progress = ctx.scene.snapshot, ctx.now(), self._progress()
        names = {}
        if self._loaded is not None:
            names = {self._live(ctx).get(planned): spec.name for planned, spec in self._loaded.process.robots.items()}
        lines = []
        for live in sorted(snapshot.robots):
            record: RobotRecord | None = progress.robots.get(live)
            if is_live(snapshot.robots[live], now):
                text = "live now"
            elif record is not None:
                where = f" at {record.after_action}" if record.after_action else ""
                text = f"not live; last recorded {record.source}{where}, {_age(record.at)} ago"
            else:
                text = "not live; nothing recorded"
            lines.append(f"{names.get(live) or live.split('/', 1)[-1]:<9} {text}")
        return lines

    # --- --- --- --- --- OPENING A PROCESS'S RUN --- --- --- --- ---

    def _reset(self, loaded: LoadedProcess | None) -> None:
        """Forget the old process's run and tasks, for another process."""
        for task in list(self._tasks):
            task.cancel()
        self._loaded, self._ids, self._run, self._offer, self._match = loaded, None, None, None, None
        self._state, self._current, self._starting, self._auto_from, self._go_task = NO_PROCESS, None, None, None, None
        self._settings, self._prompt, self._answer, self._save_error = RunSettings(), "", None, ""
        self._say("")
        self._changed()

    def _open_now(self, ctx: PluginContext) -> None:
        """Start reading the loaded process's runs; however that task ends without opening, it is tried again."""
        loaded = self._loaded
        self._state = OPENING
        self._say("reading the runs")
        self._changed()
        task = self._spawn(ctx, "open run", self._open(ctx, loaded))
        task.add_done_callback(lambda done: self._open_ended(ctx, loaded, done))

    def _open_ended(self, ctx: PluginContext, loaded: LoadedProcess, task: asyncio.Task) -> None:
        """After an open: if it did not open (stopped, failed), read the runs again soon; stopped, at the next tick."""
        if loaded is not self._loaded or self._state != OPENING:
            return
        self._state = NOT_OPENED
        self._retry_at = ctx.now() + (0.0 if task.cancelled() else RETRY_AFTER)
        if task.cancelled():
            self._say("reading the runs was stopped: reading them again", failed=True)
        self._changed()

    async def _open(self, ctx: PluginContext, loaded: LoadedProcess) -> None:
        """Continue the newest run if made on these files, else offer the runs; then go to the first pending action."""
        try:
            # * On the writer, so the runs are read after every write queued before.
            hashes, (runs, problems) = await asyncio.wrap_future(self._writer.submit(_scan, loaded.process))
        except Exception as error:
            self._say(f"could not read the runs ({error}): trying again in {RETRY_AFTER:.0f} s", failed=True)
            ctx.log_error(f"could not read the runs of {loaded.source}: {error!r}")
            return
        for problem in problems:
            ctx.log_warn(f"skipped a run folder: {problem}")
        newest, matching = (runs[0] if runs else None), continue_run(runs, hashes)
        if matching is not None and matching is newest:
            self._adopt(ctx, matching)
        elif newest is not None:
            self._offer, self._match, self._settings = newest, matching, newest.info.settings
            older = f"; run {matching.info.name}, older, was made on these files" if matching is not None else ""
            self._say(f"run {newest.info.name} was made on another export{older}: Continue, or start a New run")
        else:
            self._say("no run yet: the first mark starts one")
        ctx.log_info(self._message)
        self._state = OPEN
        self._changed()
        if self._auto_from is not None:
            self._go_task = self._spawn(ctx, "go to the first pending action", self._go_to(ctx, self._resume_point()))

    async def _go_to(self, ctx: PluginContext, point: tuple[str, int | None] | None) -> None:
        """Make an action current once its robot is live, or after LIVE_WAIT seconds anyway."""
        if point is None:
            self._say("every action is done")
            return
        acting = self._acting(ctx, point[0])
        if acting is not None:
            try:
                await ctx.wait_until(lambda: is_live(ctx.scene.snapshot.robots[acting], ctx.now()),
                                     timeout_s=LIVE_WAIT, description=f"{acting} to be live")
            except WaitTimeout:
                ctx.log_warn(f"{acting} is not live after {LIVE_WAIT:.0f} s: selecting {point[0]} anyway")
        self._auto_from, self._go_task = None, None
        self._set_current(ctx, *point)

    def _stop_auto(self) -> None:
        """Stop selecting the first pending action for the operator: something else was selected."""
        self._auto_from = None
        if self._go_task is not None:
            self._go_task.cancel()
            self._go_task = None

    def _resume_point(self) -> tuple[str, int | None] | None:
        """Where to carry on: the saved current action if still pending, else the first pending; None if all done."""
        saved = self._run.progress.current if self._run is not None else None
        if saved is not None and self.view.statuses.get(saved[0]) == PENDING:
            return saved
        action_id = first_pending(self._progress(), self._loaded.process.schedule)
        return (action_id, None) if action_id is not None else None

    def _adopt(self, ctx: PluginContext, run: Run) -> None:
        """Write on in a run made on these process files."""
        self._run, self._offer, self._match, self._settings = run, None, None, run.info.settings
        self._write(ctx, "record the session", record_event, run.folder, "session", ctx.now())
        self._say(f"continuing run {run.info.name}")
        self._changed()

    # --- --- --- --- --- MARKING --- --- --- --- ---

    def _mark_refusal(self, ctx: PluginContext, action_id: str | None) -> str:
        """Why an action can't be marked done now, or ""."""
        refusal = self._unknown(action_id) or self._busy() or self._offered_first()
        if refusal:
            return refusal
        if self.view.statuses[action_id] == DONE:
            return f"{action_id} is already done"
        acting = self._acting(ctx, action_id)
        if acting is not None and self._moving(ctx, acting):
            return f"an arm of {acting} is still moving: wait until it stops"
        return ""

    async def _confirm_mark(self, ctx: PluginContext, action_id: str, acting: str, robot: str) -> None:
        """Once confirmed, and still allowed, mark an action whose robot is not live with the export's end, assumed."""
        if not await self._confirmed(ctx):
            self._say(f"{action_id} not marked")
            return
        if self._refused("Mark done", self._mark_refusal(ctx, action_id)):
            return  # ? e.g. an arm started moving while the prompt waited
        end = exported_end(self._loaded, action_id, at=stamp())
        if end is None:
            self._say(f"{action_id} not marked: the export leaves {robot}'s base or joints open", failed=True)
            return
        await self._mark(ctx, action_id, {**self._live_ends(ctx), acting: end}, True)

    async def _mark(self, ctx: PluginContext, action_id: str, ends: Mapping[str, RobotRecord],
                    assumed: bool) -> None:
        """Mark an action done once there is a run to write into."""
        if await self._ensure_run(ctx) is not None:
            self._apply_mark(ctx, action_id, ends, assumed)

    def _apply_mark(self, ctx: PluginContext, action_id: str, ends: Mapping[str, RobotRecord], assumed: bool) -> None:
        """Mark an action done, make the first pending one current, then write both."""
        try:
            run = mark_done(self._run, action_id, ends, assumed)
        except RunError as refused:
            self._refused("Mark done", str(refused))
            return
        self._run = run
        following = first_pending(run.progress, self._loaded.process.schedule)
        if following is not None:
            self._set_current(ctx, following)
        else:
            self._save_current(ctx, None)
        self._write(ctx, "record the mark", record_event, run.folder, "mark_done", ctx.now(), action=action_id,
                    assumed=assumed, ends=sorted(ends))
        moving = sorted(live for live in ctx.scene.snapshot.robots if self._moving(ctx, live))
        left_out = f"; not recorded while moving: {', '.join(moving)}" if moving else ""
        self._say(f"{action_id} done{' (end assumed)' if assumed else ''}; next: {following or 'none, all done'}"
                  f"{left_out}")
        ctx.log_info(f"{action_id} marked done in run {run.info.name}{' (end assumed)' if assumed else ''}; "
                     f"recorded {', '.join(sorted(ends)) or 'no robot'}{left_out}")

    def _live_ends(self, ctx: PluginContext) -> dict[str, RobotRecord]:
        """Where every live robot is now, by live id; a robot whose arm moves is left out."""
        snapshot, now, at = ctx.scene.snapshot, ctx.now(), stamp()
        return {live: live_record(robot, at) for live, robot in snapshot.robots.items()
                if is_live(robot, now) and not self._moving(ctx, live)}

    def _moving(self, ctx: PluginContext, live: str) -> bool:
        """Whether an arm of a robot runs a trajectory we sent."""
        robot = ctx.world.robots.get(live.split("/", 1)[-1])
        return robot is not None and any(arm.state.is_executing for arm in robot.arms.values())

    # --- --- --- --- --- STARTING RUNS --- --- --- --- ---

    async def _ensure_run(self, ctx: PluginContext) -> Run | None:
        """The run to write into, started if there is none yet; None (said why) before the open or during an offer."""
        if self._run is not None or self._loaded is None:
            return self._run
        if self._refused("Writing", self._not_open() or self._offered_first()):
            return None
        return await self._start(ctx, None)

    async def _start(self, ctx: PluginContext, continued_from: Run | None) -> Run | None:
        """Start a run on the writer thread; a second caller waits for the same one. None if it failed."""
        if self._starting is None:
            loaded = self._loaded
            self._starting = asyncio.wrap_future(self._writer.submit(
                new_run, loaded.process, ctx.config.robots, self._settings, continued_from=continued_from,
                root=ctx.config.drive_root))
            self._starting.add_done_callback(lambda done: self._started(ctx, loaded, done))
        try:
            await asyncio.shield(self._starting)
        except Exception:
            return None  # `_started` said why
        return self._run

    def _started(self, ctx: PluginContext, loaded: LoadedProcess, done: asyncio.Future) -> None:
        """Take a run the writer started, unless another process was loaded meanwhile."""
        if self._starting is done:
            self._starting = None
        if done.cancelled() or loaded is not self._loaded:
            return
        if done.exception() is not None:
            self._save_error = f"could not start a run: {done.exception()}"
            self._say(self._save_error, failed=True)
            ctx.log_error(f"could not start a run in {loaded.source}: {done.exception()!r}")
            return
        self._run, self._offer, self._match = done.result(), None, None
        self._changed()
        info = self._run.info
        since = f"; changed since {info.continued_from}: {', '.join(info.changed_since) or 'nothing'}" \
            if info.continued_from else ""
        ctx.log_info(f"started run {self._run.folder}{since}")

    async def _new_run(self, ctx: PluginContext) -> None:
        """Start a new run once the operator confirms; then the first action is current."""
        if not await self._confirmed(ctx):
            self._say("no new run")
            return
        self._starting = None
        if await self._start(ctx, None) is not None:
            self._say(f"new run {self._run.info.name}")
            self._set_current(ctx, self._loaded.process.schedule[0])

    async def _continue(self, ctx: PluginContext, run: Run) -> None:
        """Write on in an offered run made on these files, or carry another export's run into a new one; then go on."""
        if run is self._match:
            self._adopt(ctx, run)
        elif await self._start(ctx, run) is None:
            return
        else:
            self._say(f"run {self._run.info.name} continues {run.info.name}")
            was, now = run.progress.current, self._run.progress.current
            if was is not None and now is not None and was != now:
                ctx.log_warn(f"{was[0]} has no movement {was[1]} in this export: it starts at its first step")
        point = self._resume_point()
        if point is not None:
            self._set_current(ctx, *point)

    async def _save_settings(self, ctx: PluginContext, changes: Mapping[str, Any]) -> None:
        """Write the settings into the run, starting one if needed."""
        run = await self._ensure_run(ctx)
        if run is None:
            return
        self._run = replace(run, info=replace(run.info, settings=self._settings))
        self._write(ctx, "save the settings", save_info, self._run)
        self._write(ctx, "record the settings", record_event, run.folder, "settings", ctx.now(), changes=dict(changes))

    async def _record_later(self, ctx: PluginContext, event: str, action: str | None, movement: str | None,
                            data: Mapping[str, Any] | None, file: Mapping[str, Any] | None) -> None:
        """Record once there is a run."""
        if await self._ensure_run(ctx) is not None:
            self._queue_record(ctx, event, action, movement, data, file)

    def _queue_record(self, ctx: PluginContext, event: str, action: str | None, movement: str | None,
                      data: Mapping[str, Any] | None, file: Mapping[str, Any] | None) -> None:
        """Queue a record's writes."""
        folder = self._run.folder
        if file is not None:
            self._write(ctx, f"record {event}", _record_file, folder, event, ctx.now(), action, movement, file, data)
        else:
            self._write(ctx, f"record {event}", record_event, folder, event, ctx.now(), action=action,
                        movement=movement, **({"data": dict(data)} if data else {}))

    # --- --- --- --- --- STATE --- --- --- --- ---

    def _set_current(self, ctx: PluginContext, action_id: str, movement_index: int | None = None) -> None:
        """Make an action's movement current (its first step if None or one it lacks), save it, and select it."""
        movement = self._movement(ctx, action_id, movement_index)
        self._save_current(ctx, (action_id, movement))
        selector: ProcessPluginApi = ctx.require("process")
        selector.request_select(self._loaded.index_of(action_id, movement))

    def _movement(self, ctx: PluginContext, action_id: str, movement_index: int | None) -> int:
        """A movement index the action has: the one given, else its first step (warned when one was given)."""
        process = self._loaded.process
        action = process.actions[action_id]
        if isinstance(movement_index, int) and 0 <= movement_index < len(action.movements):
            return movement_index
        first = first_step(process, action, self._settings.transfer_as_free)
        if movement_index is not None:
            ctx.log_warn(f"{action_id} has no movement {movement_index} (it has {len(action.movements)}): "
                         f"it starts at its first step, {action.movements[first].id}")
        return first

    def _save_current(self, ctx: PluginContext, current: tuple[str, int] | None) -> None:
        """Make `current` current, and write the run's progress with it."""
        self._current = current
        if self._run is not None:
            self._run = with_current(self._run, current)
            self._write(ctx, "save the progress", save_progress, self._run)
        self._changed()

    def _progress(self) -> Progress:
        """The run's progress, or an empty one (every action pending) before there is a run."""
        return self._run.progress if self._run is not None else Progress({}, None, {}, "")

    def _current_action(self) -> str | None:
        """The current action's id, or None."""
        return self._current[0] if self._current is not None else None

    def _acting(self, ctx: PluginContext, action_id: str) -> str | None:
        """The live id of the configured robot an action is for, or None."""
        robot = self._loaded.process.actions[action_id].robot
        return self._live(ctx).get(robot) if robot else None

    def _live(self, ctx: PluginContext) -> IdMap:
        """The loaded process's ids -> live ids (`live_ids`), worked out once per process."""
        if self._ids is None:
            self._ids = live_ids(self._loaded.process, ctx.config.robots, "")[0]
        return self._ids

    # --- --- --- --- --- REFUSALS --- --- --- --- ---

    def _not_open(self) -> str:
        """Why nothing may be written yet: no process, or its runs not read; else ""."""
        if self._loaded is None:
            return "no process loaded"
        if self._state == OPENING:
            return "the run is not open yet: its folder is still being read"
        if self._state == NOT_OPENED:
            return "the run is not open: reading its folder failed or was stopped, it is read again"
        return ""

    def _busy(self) -> str:
        """Why nothing that writes may start now: the run is not open, or a prompt waits; else ""."""
        return self._not_open() or ("answer the prompt first" if self._prompt else "")

    def _offered_first(self) -> str:
        """Why nothing is written while runs are offered; else ""."""
        if self._run is not None or self._offer is None:
            return ""
        return f"run {self._offer.info.name} was made on another export: Continue or New run first"

    def _no_run(self) -> str:
        """Why there is nothing done to reopen without a run; else ""."""
        return "" if self._run is not None else self._offered_first() or "no run yet: nothing is done"

    def _settings_problem(self, changes: Mapping[str, Any]) -> str:
        """What is wrong with requested settings, or ""."""
        names = {field.name for field in fields(RunSettings)}
        for name, value in changes.items():
            if name not in names:
                return f"there is no setting {name!r}"
            if name == "insert_split_mm":
                if isinstance(value, bool) or not isinstance(value, (int, float)) \
                        or not SPLIT_RANGE[0] <= value <= SPLIT_RANGE[1]:
                    return f"the insert split must be {SPLIT_RANGE[0]:.0f} to {SPLIT_RANGE[1]:.0f} mm, not {value!r}"
            elif not isinstance(value, bool):
                return f"{name} must be True or False, not {value!r}"
        return ""

    def _unknown(self, action_id: str | None) -> str:
        """Why an action id names no scheduled action, or ""."""
        if self._loaded is None:
            return "no process loaded"
        if action_id not in self._loaded.process.schedule:
            return f"no action {action_id!r} in the schedule" if action_id else "no current action"
        return ""

    def _refused(self, what: str, refusal: str) -> bool:
        """Say and log why something was refused; False if `refusal` is empty."""
        if refusal:
            self._say(f"{what} refused: {refusal}", failed=True)
            self._ctx.log_warn(self._message)
        return bool(refusal)

    # --- --- --- --- --- PLUMBING --- --- --- --- ---

    def _write(self, ctx: PluginContext, what: str, function: Callable[..., Any], *args: Any, **kwargs: Any) -> None:
        """Queue a write after every earlier one; a failure shows in the panel and the log."""
        done = asyncio.wrap_future(self._writer.submit(function, *args, **kwargs))
        done.add_done_callback(lambda job: self._written(ctx, what, job))

    def _written(self, ctx: PluginContext, what: str, job: asyncio.Future) -> None:
        """Report a write that failed."""
        if not job.cancelled() and job.exception() is not None:
            self._save_error = f"could not {what}: {job.exception()}"
            ctx.log_error(f"could not {what}: {job.exception()!r}")
            self._changed()

    def _ask(self, prompt: str) -> None:
        """Put a question to the operator, at once, so a second click is refused until it is answered."""
        self._prompt, self._answer = prompt, None
        self._changed()

    async def _confirmed(self, ctx: PluginContext) -> bool:
        """The answer to the question asked: True on Confirm, False on Cancel or none within CONFIRM_TIMEOUT."""
        try:
            await ctx.wait_until(lambda: self._answer is not None, timeout_s=CONFIRM_TIMEOUT,
                                 description="Confirm or Cancel")
            return bool(self._answer)
        except WaitTimeout:
            ctx.log_warn(f"no answer in {CONFIRM_TIMEOUT:.0f} s to: {self._prompt}")
            return False
        finally:
            self._prompt, self._answer = "", None
            self._changed()

    def _spawn(self, ctx: PluginContext, label: str, work: Coroutine) -> asyncio.Task:
        """Start a task of this process's run, cancelled when another process loads."""
        task = ctx.spawn(label, work)
        self._tasks.add(task)
        task.add_done_callback(self._tasks.discard)
        return task

    def _say(self, message: str, failed: bool = False) -> None:
        """Set the one line of feedback."""
        self._message, self._failed = message, failed

    def _changed(self) -> None:
        """Note a change, so `view` is rebuilt on its next read."""
        self._revision += 1
