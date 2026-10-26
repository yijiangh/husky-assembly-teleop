"""
Remember how far an assembly got: which actions are done, where each robot was left, and the run's settings.

On a process load it continues the assembly run used last when it was made on exactly these process files, else the
operator chooses (`shared/runs/runs.py`); then the first pending action becomes current, and its first step is
selected once the action's robot is live. Other plugins read and change it through `AssemblyRunContract`.

Its part of the Assembly tab is Mark done and Reopen, the settings and the run. The run follows the process panel: a
movement picked there becomes current, except while movement_exec has a robot's lease for a step; each action's mark
shows in the panel's rows. New run, and Mark done with the action's robot not live, take a second click within 10 s.

- ! Nothing is written until the runs were read: an open that fails or is stopped is tried again.
- ! Runs are written into the process folder (`runs/`), in order on a writer thread; the state here changes first.
- ! EXPERIMENTAL: the run format and the requests may still change.
"""

from __future__ import annotations

import asyncio
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable, Mapping

import viser

from ...plugin_api.concurrency import WaitTimeout
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...shared.assembly_run_contract import RunView, beliefs_of, first_pending, is_live
from ...shared.assembly_planning.process_steps import LoadedProcess, loaded_process
from ...shared.execution.spiral import spiral_fits
from ...shared.movement_exec_contract import INSERT_PART, current_step, part_plan
from ...shared.process_contract import MARK_ASSUMED, MARK_DONE, MARK_PENDING
from ...shared.runs.runs import (DONE, PENDING, ActionRecord, RobotRecord, Run, RunError, RunSettings, continue_run,
                                 file_hashes, find_runs, made_after, record_event, reopen, save_progress, with_current)
from ...ui.assembly_tab import ASSEMBLY_TAB
from .panel import PanelStatus, RunPanel
from .writes import (AGAIN_MARK, AGAIN_NEW_RUN, CONFIRM_AGAIN, NO_PROCESS, NOT_OPENED, OPEN, OPENING, SETTING_TYPES,
                     RunWrites)

if TYPE_CHECKING:
    from scaffolding_assembly_core.ids import IdMap
    from scaffolding_assembly_core.process import Process

    from ...shared.process_contract import ProcessContract

#: Seconds to wait for the current action's robot to be live before selecting its step anyway.
LIVE_WAIT = 15.0
#: Seconds (ROS time) before reading the runs again after reading them failed.
RETRY_AFTER = 5.0
#: Seconds (ROS time) without another pick before a pick followed in the process panel is written.
FOLLOW_SAVE_AFTER = 2.0


def _scan(process: Process) -> tuple[tuple[str, dict], tuple[list[Run], list[tuple[str, str]]]]:
    """The process's file hashes and the runs in its folder. Writer thread."""
    return file_hashes(process), find_runs(process.folder)


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
class AssemblyRunPlugin(RunWrites, HuskyPlugin):
    """Tracks which actions of the loaded process are done, and where the robots were left."""

    name = "assembly_run"
    requires = ("process",)
    experimental = True
    tab_path = (ASSEMBLY_TAB,)
    icon = viser.Icon.CHECKLIST

    def __init__(self):
        """Start with no process."""
        self._ctx: PluginContext | None = None
        self._panel: RunPanel | None = None
        #: The process the run is for, as the process plugin loaded it, and its process ids -> live ids.
        self._loaded: LoadedProcess | None = None
        #: The process plugin's `Process` that `_loaded` wraps.
        #: ! Wrapped once per process: plans and requests compare `_loaded` by identity, so a new wrapper would make
        #:   every plan stale and reset the run.
        self._source: Process | None = None
        self._ids: IdMap | None = None
        #: How far its runs were read (NO_PROCESS, OPENING, OPEN, NOT_OPENED), and when to read them again.
        self._state, self._retry_at = NO_PROCESS, 0.0
        #: The run being written; else the run used last, offered, and an older run made on these files, offered too
        #: (both the same run when a newer run folder can't be read).
        self._run: Run | None = None
        self._offer: Run | None = None
        self._match: Run | None = None
        self._settings = RunSettings()
        #: (action id, movement index) the operator is at.
        self._current: tuple[str, int] | None = None
        #: The process panel's authored index the run last selected or followed, so a new pick there is noticed.
        self._selected: int | None = None
        #: Whether movement_exec had a robot's lease for a step at the last update, to notice the step ending.
        self._step_ran = False
        #: The click waiting for its second one (`again`): (AGAIN_NEW_RUN or "mark <action id>", ROS time it ends).
        self._again: tuple[str, float] | None = None
        #: When the current followed from the process panel was last changed without writing it; None: written.
        self._unsaved_at: float | None = None
        #: The view's revision whose marks the process panel was last given.
        self._marks_revision = -1
        #: One line of feedback, whether it is a failure, and the last write that failed.
        self._message, self._failed, self._save_error = "", False, ""
        #: How many settings requests were refused, so the panel puts its widgets back.
        self._settings_refused = 0
        self._revision = 0
        self._view: RunView | None = None
        # ! Every file read and write, in order: the state here changes first, the files follow.
        self._writer = ThreadPoolExecutor(1, "run-writer")
        # The run being started on the writer, so a second request waits for the same one; and a Continue running.
        self._starting: asyncio.Future | None = None
        self._continuing: asyncio.Task | None = None
        # This process's tasks (opening, prompts, starting a run), cancelled when another process loads.
        self._tasks: set[asyncio.Task] = set()
        # The process plugin's step index while the first pending action is still to be selected for the operator
        # (None once anything else selects), and the task waiting for its robot to be live.
        self._auto_from: int | None = None
        self._go_task: asyncio.Task | None = None

    # --- --- --- --- --- WHAT OTHER PLUGINS READ AND ASK (AssemblyRunContract) --- --- --- --- ---

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
        self._again = None
        if self._refused("Select", self._unknown(action_id) or self._step_running(self._ctx)):
            return
        self._stop_auto()
        self._set_current(self._ctx, action_id, movement_index)

    def request_mark_done(self, action_id: str | None = None) -> None:
        """Mark an action (the current one if None) done; its robot not live, on a second click within 10 s."""
        ctx = self._ctx
        action_id = action_id or self._current_action()
        if self._refused("Mark done", self._mark_refusal(ctx, action_id)):
            self._again = None  # ! a refused click never waits for a second one
            return
        self._stop_auto()
        acting = self._acting(ctx, action_id)
        ends = self._live_ends(ctx)
        key = f"{AGAIN_MARK} {action_id}"
        if acting is None or acting in ends:
            self._again = None
            if self._run is not None:
                self._apply_mark(ctx, action_id, ends, False)
            else:
                self._spawn(ctx, f"mark {action_id} done", self._mark(ctx, action_id, ends, False))
            return
        robot = self._loaded.process.robots[self._loaded.process.actions[action_id].robot].name
        if not self._clicked_again(ctx, key, f"{action_id}: {robot} is not live; click Mark done again within "
                                             f"{CONFIRM_AGAIN:.0f} s to record the export's end (assumed)"):
            return
        self._spawn(ctx, f"mark {action_id} done", self._mark_assumed(ctx, action_id, acting, robot))

    def request_reopen(self, action_id: str | None = None) -> None:
        """Put a done action (the current one if None) back to pending, and make it current."""
        ctx = self._ctx
        self._again = None
        action_id = action_id or self._current_action()
        if self._refused("Reopen", self._unknown(action_id) or self._busy() or self._no_run()
                         or self._step_running(ctx)):
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
        """Start a new run on a second click within 10 s; every action pending, the old run kept."""
        if self._refused("New run", self._busy() or self._step_running(self._ctx)):
            self._again = None  # ! a refused click never waits for a second one
            return
        if not self._clicked_again(self._ctx, AGAIN_NEW_RUN, f"New run: click New run again within "
                                                             f"{CONFIRM_AGAIN:.0f} s to confirm; every action starts "
                                                             f"pending, the old run is kept"):
            return
        self._stop_auto()
        self._spawn(self._ctx, "new run", self._new_run(self._ctx))

    def request_continue(self, run_name: str | None = None) -> None:
        """Continue an offered run: the one used last if None; one made on another export goes on in a new run."""
        self._again = None
        offered = [run for run in (self._offer, self._match) if run is not None]
        target = self._offer if run_name is None else next((run for run in offered if run.info.name == run_name), None)
        missing = "" if target is not None else (f"no offered run is named {run_name!r}" if run_name
                                                 else "no run made on these files is offered" if run_name == ""
                                                 else "no run is offered to continue")
        if not self._refused("Continue", self._busy() or missing or self._step_running(self._ctx)):
            self._stop_auto()
            self._continuing = self._spawn(self._ctx, "continue run", self._continue(self._ctx, target))

    def request_settings(self, **changes: Any) -> None:
        """Change run settings (`RunSettings` fields); saved with the run, which this starts if there is none."""
        refusal = (self._settings_problem(changes) or self._not_open() or self._offered_first()
                   or self._step_running(self._ctx))
        if refusal:
            self._settings_refused += 1
            self._refused("Settings", refusal)
            return
        settings = replace(self._settings, **{name: SETTING_TYPES[name](value) for name, value in changes.items()})
        if settings == self._settings:
            return
        self._again = None
        self._settings = settings
        self._changed()
        self._spawn(self._ctx, "save settings", self._save_settings(self._ctx, changes))

    def request_record(self, event: str, *, action: str | None = None, movement: str | None = None,
                       data: Mapping[str, Any] | None = None, file: Mapping[str, Any] | None = None,
                       kind: str = "movements",
                       figure: Callable[[Mapping[str, Any], Path], None] | None = None) -> None:
        """Record an event, `data` under its "data" key; `file` is written as a `kind` record the event names.

        `figure(payload, png path)` draws the record's plot beside it; a plot that fails is noted in the event's data
        as `figure_error`, the record kept.
        """
        refusal = self._not_open() or (
            "a file needs its action and movement" if file is not None and None in (action, movement) else "")
        if self._refused(f"Record {event}", refusal):
            return
        if self._run is not None:
            self._queue_record(self._ctx, event, action, movement, data, file, kind, figure)
        else:
            self._spawn(self._ctx, f"record {event}", self._record_later(self._ctx, event, action, movement, data,
                                                                         file, kind, figure))

    # --- --- --- --- --- HOOKS --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Keep the context for the requests, and build the panel."""
        self._ctx = ctx
        self._panel = RunPanel(ctx, self)

    def update(self, ctx: PluginContext) -> None:
        """Open the run of a newly loaded process; open it again if that failed; notice a step picked meanwhile.

        It also follows a pick in the process panel (not one made while a step runs), writes it once the picking stops,
        puts the panel back on the run's movement when a step ends, and gives the panel the actions' marks.
        """
        process: ProcessContract = ctx.require("process")
        selection = process.selection
        source = selection.process if selection is not None else None
        if source is not self._source:
            self._source = source
            loaded = loaded_process(source) if source is not None else None
            self._reset(loaded)
            if loaded is not None:
                self._auto_from = self._selected = selection.index
                self._open_now(ctx)
            return
        if source is None:
            return
        if self._auto_from is not None and selection.index != self._auto_from:
            self._stop_auto()
            ctx.log_info("a step was picked in the process panel: the first pending action is not selected")
        running = bool(self._step_running(ctx))
        if self._step_ran and not running and self._current is not None:
            # * The step ended: the panel goes back to the run's movement, whatever was picked during it. Read back,
            #   as in `_set_current`, so this select is not taken for a pick.
            process.request_select(self._loaded.index_of(*self._current))
            selection = process.selection
            self._selected = selection.index
        self._step_ran = running
        if selection.index != self._selected:
            # * Follow a pick in the process panel. ! Never one made while a step runs, then or later, so the step being
            #   run can't change: such a pick only shows that movement until the step ends.
            self._selected = selection.index
            if not running:
                # A click waiting for its second one was for what was current before: it no longer waits.
                step = self._loaded.steps[selection.index]
                self._again = None
                # ? progress.json once the picking stops, not at every step of a dragged slider.
                self._save_current(ctx, (step.action.id, step.movement_index), write=False)
                self._unsaved_at = ctx.now()
        if self._unsaved_at is not None and ctx.now() - self._unsaved_at >= FOLLOW_SAVE_AFTER:
            self._unsaved_at = None
            if self._run is not None:
                self._write(ctx, "save the progress", save_progress, self._run)
        if self._again is not None and ctx.now() > self._again[1]:
            self._say(f"not confirmed: no second click within {CONFIRM_AGAIN:.0f} s")
            self._again = None
        if self._state == NOT_OPENED and ctx.now() >= self._retry_at:
            self._open_now(ctx)
        view = self.view
        if view.revision != self._marks_revision:
            # * Here, not in draw: draw is skipped while the panels are frozen.
            self._marks_revision = view.revision
            process.request_marks({action: MARK_ASSUMED if action in view.assumed
                                   else MARK_DONE if status == DONE else MARK_PENDING
                                   for action, status in view.statuses.items()})

    def draw(self, ctx: PluginContext) -> None:
        """Fill the panel."""
        if self._panel is not None:
            status = PanelStatus(self._run.info.name if self._run is not None else "", self.again,
                                 (self._message, self._failed), self._save_error, tuple(self.robot_lines(ctx)),
                                 self._settings_refused, self._spiral_fits(ctx))
            self._panel.draw(self.view, status)

    def teardown(self, ctx: PluginContext) -> None:
        """Write a current not written yet, and wait for the queued writes."""
        if self._unsaved_at is not None and self._run is not None:
            self._writer.submit(save_progress, self._run)
        self._writer.shutdown(wait=True)

    # --- --- --- --- --- FOR THE PANEL --- --- --- --- ---

    @property
    def again(self) -> str:
        """str: The click waiting for its second one, AGAIN_NEW_RUN or "mark <action id>"; "" after CONFIRM_AGAIN s."""
        waiting = self._again is not None and self._ctx.now() <= self._again[1]
        return self._again[0] if waiting else ""

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

    def _spiral_fits(self, ctx: PluginContext) -> bool:
        """Whether the current step's insert has two mating joints, so the Spiral box is live (`spiral_fits`)."""
        transfer = ctx.require("process").transfer_as_free
        found = current_step(self.view, transfer)
        if found is None:
            return False
        action, steps, index = found
        part = part_plan(self.view.process, action, steps[index].primary, ctx.config.robots, transfer)
        return part.kind == INSERT_PART and spiral_fits(part.tools)

    # --- --- --- --- --- OPENING A PROCESS'S RUN --- --- --- --- ---

    def _reset(self, loaded: LoadedProcess | None) -> None:
        """Forget the old process's run and tasks, for another process."""
        for task in list(self._tasks):
            task.cancel()
        self._loaded, self._ids, self._run, self._offer, self._match = loaded, None, None, None, None
        self._state, self._current, self._starting, self._auto_from, self._go_task = NO_PROCESS, None, None, None, None
        self._continuing = None
        self._settings, self._again, self._save_error, self._unsaved_at = RunSettings(), None, "", None
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
        elif task.exception() is not None:
            self._say(f"could not open the run ({task.exception()}): trying again in {RETRY_AFTER:.0f} s",
                      failed=True)
        self._changed()

    async def _open(self, ctx: PluginContext, loaded: LoadedProcess) -> None:
        """Continue the run used last if made on these files, else offer the runs; then go to the first pending one."""
        try:
            # * On the writer, so the runs are read after every write queued before.
            hashes, (runs, problems) = await asyncio.wrap_future(self._writer.submit(_scan, loaded.process))
        except Exception as error:
            self._say(f"could not read the runs ({error}): trying again in {RETRY_AFTER:.0f} s", failed=True)
            ctx.log_error(f"could not read the runs of {loaded.source}: {error!r}")
            return
        for name, problem in problems:
            ctx.log_warn(f"skipped a run folder: {name}: {problem}")
        last, matching = (runs[0] if runs else None), continue_run(runs, hashes)
        # ? A run that can't be read may be the one used last (e.g. half synced): never write on past it unasked.
        unread = matching is not None and any(made_after(name, matching) for name, _problem in problems)
        if matching is not None and matching is last and not unread:
            self._adopt(ctx, matching)
        elif last is not None:
            self._offer, self._match, self._settings = last, matching, last.info.settings
            self._say(f"{self._offer_line()}: Continue, or start a New run")
        else:
            self._say("no run yet: the first mark starts one")
        ctx.log_info(self._message)
        self._state = OPEN
        self._changed()
        if self._offer is not None:
            self._auto_from = None  # * the operator chooses a run first; Continue or New run selects
        elif self._auto_from is not None:
            point = self._resume_point()
            self._go_task = self._spawn(ctx, "go to the first pending action", self._go_to(ctx, point))
            self._go_task.add_done_callback(lambda done: self._go_ended(ctx, point, done))

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

    def _go_ended(self, ctx: PluginContext, point: tuple[str, int | None] | None, task: asyncio.Task) -> None:
        """After `_go_to`: if a soft stop cancelled it, make the action current now, or none would be.

        ? Not after `_stop_auto`, another process or the plugin's own stop: they forget the task first, or close the
          context.
        """
        if task.cancelled() and self._go_task is task and point is not None and not ctx.closed:
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
        """Write on in a run made on these process files; it becomes the run used last."""
        self._offer, self._match, self._settings = None, None, run.info.settings
        self._run = with_current(run, run.progress.current)
        self._write(ctx, "save the progress", save_progress, self._run)
        self._write(ctx, "record the session", record_event, run.folder, "session", ctx.now())
        self._say(f"continuing run {run.info.name}")
        self._changed()
