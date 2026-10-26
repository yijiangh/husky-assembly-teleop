"""
The assembly_run plugin's writing half: marking, starting runs, the run's state, refusals, and the writer thread.

`RunWrites` is mixed into `AssemblyRunPlugin` (plugin.py), which holds every attribute used here (its `__init__`).
It is split off only to keep each file short, and is never used on its own.
"""

from __future__ import annotations

import asyncio
from dataclasses import fields, replace
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable, Coroutine, Mapping

from ...plugin_api.context import PluginContext
from ...shared.assembly_run_contract import exported_end, first_pending, is_live, live_record
from ...shared.execution.operator import PROMPT_ARM_S
from ...shared.assembly_planning.steps import first_step
from ...shared.movement_exec_contract import step_running
from ...shared.process_scene.scene_bodies import live_ids
from ...shared.runs.runs import (DONE, SPIRAL_SECONDS_RANGE, SPIRAL_TURNS_RANGE, SPLIT_RANGE, Progress, RobotRecord,
                                 Run, RunError, RunSettings, mark_done, new_run, record_event, record_movement,
                                 save_info, save_progress, stamp, with_current)

if TYPE_CHECKING:
    from scaffolding_assembly_core.ids import IdMap

    from ...shared.assembly_planning.process_steps import LoadedProcess
    from ...shared.process_contract import ProcessContract

#: Seconds (ROS time) in which a second click confirms New run, or Mark done with the action's robot not live.
CONFIRM_AGAIN = 10.0
#: What waits for its second click (`AssemblyRunPlugin.again`): a new run, or marking an action ("mark <action id>").
AGAIN_NEW_RUN, AGAIN_MARK = "new run", "mark"
#: Each number setting: what a refusal calls it, its range and its unit.
NUMBER_SETTINGS = {"insert_split_mm": ("the insert split", SPLIT_RANGE, "mm"),
                   "spiral_seconds": ("the spiral's duration", SPIRAL_SECONDS_RANGE, "s"),
                   "spiral_turns": ("the spiral's turns", SPIRAL_TURNS_RANGE, "whole turns")}
#: Each setting's type, its default's: a request is converted to it.
SETTING_TYPES = {each.name: type(each.default) for each in fields(RunSettings)}
#: How far the loaded process's runs were read; only OPEN allows writes.
NO_PROCESS, OPENING, OPEN, NOT_OPENED = "", "opening", "open", "not opened"


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


class RunWrites:
    """Marking, starting runs, the current action, refusals and writes of `AssemblyRunPlugin` (module docstring)."""

    # --- --- --- --- --- MARKING --- --- --- --- ---

    def _mark_refusal(self, ctx: PluginContext, action_id: str | None) -> str:
        """Why an action can't be marked done now, or ""."""
        refusal = self._unknown(action_id) or self._busy() or self._offered_first()
        if refusal:
            return refusal
        if self.view.statuses[action_id] == DONE:
            return f"{action_id} is already done"
        refusal = self._step_running(ctx)
        if refusal:
            return refusal
        acting = self._acting(ctx, action_id)
        if acting is not None and self._moving(ctx, acting):
            return f"an arm of {acting} is still moving: wait until it stops"
        return ""

    async def _mark_assumed(self, ctx: PluginContext, action_id: str, acting: str, robot: str) -> None:
        """Mark an action whose robot is not live done with the export's end, assumed (the second click)."""
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
        if self._refused("Mark done", self._step_running(ctx)):
            return  # ? a step started while the run was being started
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
                root=ctx.config.drive_root, transfer_as_free=ctx.require("process").transfer_as_free))
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
        self._say(f"run {info.name} started")
        ctx.log_info(f"started run {self._run.folder}{since}")

    async def _new_run(self, ctx: PluginContext) -> None:
        """Start a new run (the second click); then the first action is current."""
        if self._refused("New run", self._step_running(ctx)):
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
        """Make an action's movement current (its first step if None or one it lacks), save it, and select it.

        ? Refused while a step runs: a prompt, a run starting or the wait for a live robot may have let one start.
        """
        if self._refused("Select", self._step_running(ctx)):
            return
        movement = self._movement(ctx, action_id, movement_index)
        self._save_current(ctx, (action_id, movement))
        selector: ProcessContract = ctx.require("process")
        selector.request_select(self._loaded.index_of(action_id, movement))
        # * Read back, not the index asked for: a movement the panel doesn't list selects another, not to be followed.
        self._selected = selector.selection.index

    def _movement(self, ctx: PluginContext, action_id: str, movement_index: int | None) -> int:
        """A movement index the action has: the one given, else its first step (warned when one was given)."""
        process = self._loaded.process
        action = process.actions[action_id]
        if type(movement_index) is int and 0 <= movement_index < len(action.movements):  # ? not True, not "1"
            return movement_index
        first = first_step(process, action, ctx.require("process").transfer_as_free)
        if movement_index is not None:
            ctx.log_warn(f"{action_id} has no movement {movement_index} (it has {len(action.movements)}): "
                         f"it starts at its first step, {action.movements[first].id}")
        return first

    def _save_current(self, ctx: PluginContext, current: tuple[str, int] | None, write: bool = True) -> None:
        """Make `current` current, and write the run's progress with it.

        Args:
            ctx: The plugin's context.
            current: (action id, movement index), or None.
            write: Write progress.json now; False leaves it to the caller (a pick followed in the process panel is
                written once the picking stops, `FOLLOW_SAVE_AFTER`).
        """
        self._current = current
        if self._run is not None:
            self._run = with_current(self._run, current)
            if write:
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

    @staticmethod
    def _step_running(ctx: PluginContext) -> str:
        """Why nothing that moves the current step or the robots may change now: a step runs on them; else "".

        ? A step runs while movement_exec has the robots' leases (`step_running`).
        """
        return step_running(ctx.world.robots)

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
        """Why nothing that writes may start now: the run is not open, or a run is being started; else "".

        ? Being started includes a Continue still running: a second Continue would race the first.
        """
        starting = self._starting is not None or (self._continuing is not None and not self._continuing.done())
        return self._not_open() or ("a run is being started: wait a moment" if starting else "")

    def _offered_first(self) -> str:
        """Why nothing is written while runs are offered; else ""."""
        if self._run is not None or self._offer is None:
            return ""
        return f"{self._offer_line()}: Continue or New run first"

    def _offer_line(self) -> str:
        """What is offered, as a line: another export's run (and one made on these files), or one on these files.

        The second while a newer run folder can't be read: it may be the run used last.
        """
        if self._offer is self._match:
            return (f"a newer run folder can't be read (see the log); run {self._match.info.name} was made on these "
                    f"files")
        older = f"; run {self._match.info.name}, older, was made on these files" if self._match is not None else ""
        return f"run {self._offer.info.name} was made on another export{older}"

    def _no_run(self) -> str:
        """Why there is nothing done to reopen without a run; else ""."""
        return "" if self._run is not None else self._offered_first() or "no run yet: nothing is done"

    def _settings_problem(self, changes: Mapping[str, Any]) -> str:
        """What is wrong with requested settings, or "": a number out of its range or not whole where it must be."""
        for name, value in changes.items():
            if name not in SETTING_TYPES:
                return f"there is no setting {name!r}"
            if name in NUMBER_SETTINGS:
                what, (low, high), unit = NUMBER_SETTINGS[name]
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not low <= value <= high \
                        or (SETTING_TYPES[name] is int and value != int(value)):
                    return f"{what} must be {low:g} to {high:g} {unit}, not {value!r}"
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

    def _arm(self, ctx: PluginContext, key: str, message: str) -> None:
        """Wait CONFIRM_AGAIN s for the second click of `key` (`again`), and say what it will do.

        Args:
            ctx: The plugin's context, for the clock.
            key: What waits: AGAIN_NEW_RUN, or AGAIN_MARK and the action id.
            message: What the second click does, for the message line.
        """
        self._again = (key, ctx.now() + CONFIRM_AGAIN)
        self._say(message)

    def _clicked_again(self, ctx: PluginContext, key: str, message: str) -> bool:
        """Whether this click is the second one `key` waits for; the first click arms it and says `message`.

        ! A second click within PROMPT_ARM_S of the first is a double click: ignored, the first one still waits.

        Args:
            ctx: The plugin's context, for the clock.
            key: What waits: AGAIN_NEW_RUN, or AGAIN_MARK and the action id.
            message: What a second click does, for the message line.

        Returns:
            bool: True when it confirms (and the wait ends); False when it armed the wait or was too soon.
        """
        if self.again != key:
            self._arm(ctx, key, message)
            return False
        if ctx.now() - (self._again[1] - CONFIRM_AGAIN) < PROMPT_ARM_S:
            self._say(f"ignored: clicked again within {PROMPT_ARM_S} s (a double click?). {message}")
            return False
        self._again = None
        return True

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
