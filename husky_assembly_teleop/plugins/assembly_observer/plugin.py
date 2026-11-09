"""
Record everything that happens while the monitor runs, whoever does it: the robots at 20 Hz, and every event.

Recorded on this PC only, kept KEPT_DAYS days; the panel copies a time range to the Drive after an experiment, in
the helper script's own process (`scripts/observer_logs.py copy`, logic in `export.py`), and opens one in the Rerun
viewer the same way (`observer_logs.py rerun`, `rerun_view.py`). One session per monitor
start, a folder `~/.ros/assembly_observer/<YYYYMMDD>/<stamp>_session/`:

- experiment.json: the robots and their tools, the commits, sim or real, the names' FORMAT
  (`shared/recordings/runs.py`, `names.py`).
- chunk_001.npz, chunk_002.npz, ...: every continuous value each tick (`columns.py`) for CHUNK_SECONDS each, or
  until the tracked objects change; compressed, values float32; read one with `load_recording` (pass the file).
  Each opens with a "state" event per robot, the relay and the objects: every state then.
- events.jsonl: one JSON line per event (`watch.py`): every switching state (leases, stops, controllers, modes, motor
  states, tracking), commands sent, motions with their cause, streams lost, clearance trips, force sensor zeroing,
  the relay's calibration and drift, and every log line of every plugin (the guide's, the executor's, the run's),
  each with its ROS time and local time.
- commands/<n>_<event>_<robot>.json: a trajectory or path exactly as sent, named by its event's "file".

- ! Copies go to `<Drive>/data_experiment/assembly_observer/<name>/<stamp>/`: make that folder in Drive first; it is
  never made here (`drive.py`).
- Loaded by default (config.DEFAULT_PLUGINS). It only records, never commands, so a soft stop leaves it running.
  Files are written on its own writer thread.
- Joins the assembly run's log (`runs/<run>/events.jsonl`) by ROS time: both keep it on every line.
"""

from __future__ import annotations

import json
import math
import subprocess
import sys
import tempfile
import time
from concurrent.futures import Future, ThreadPoolExecutor
from datetime import date, datetime, timedelta
from importlib.util import find_spec
from pathlib import Path
from typing import Any

import viser
from rclpy.logging import LoggingSeverity
from rosidl_runtime_py.convert import message_to_ordereddict

from ...drive import drive_folder, environment, folder_name
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...shared.recordings.runs import run_folder, write_description
from ...shared.runs.runs import commits
from ...ui.css import side_by_side_css
from ...ui.style import BUSY, FAIL, NONE, OK, SECTION_SENSOR, block, chip, chip_line, section, values
from .columns import monitor_signals, object_signals, relay_signals, robot_signals
from . import export
from .export import DAY_FORMAT, KEPT_DAYS, NOW, RECORDING_FOLDER, days, prune, size, wall_time
from .names import FORMAT, LOG, LOG_LINES_LOST, OBSERVER_ERROR, SESSION_END, SESSION_START
from .watch import FILE_KEY, Watcher

#: Where the sessions are recorded (tests point it elsewhere).
LOCAL_ROOT = export.LOCAL_ROOT
#: The Range buttons.
LAST_15, LAST_HOUR, WHOLE_DAY = "Last 15 min", "Last hour", "Whole day"
#: The helper script a copy runs in, a process of its own: memory and the GIL stay the monitor's.
SCRIPT = Path(__file__).resolve().parents[3] / "scripts" / "observer_logs.py"
#: A copy taking longer is stopped, seconds.
COPY_TIMEOUT = 15 * 60.0
#: Sending a range to the Rerun viewer taking longer is stopped, seconds; the viewer stays open.
RERUN_TIMEOUT = 10 * 60.0
#: What Open in Rerun says when the venv lacks the viewer's package.
NO_RERUN = "Rerun is not installed: pip install rerun-sdk==0.39.0 in the venv"
#: Seconds per chunk file: a crash loses at most this much of the ticks. The trace holds twice as many ticks.
CHUNK_SECONDS = 60.0
#: The events file, and the folder of the commands sent in full.
EVENTS, COMMANDS = "events.jsonl", "commands"
#: Log lines kept from this level up, but not from these sources: health's chip colours repeat the events.
LOG_LEVEL = LoggingSeverity.INFO
SKIPPED_SOURCES = ("health/chips",)


@register
class AssemblyObserverPlugin(HuskyPlugin):
    """Records the robots at every tick and every event of the session on this PC; copies ranges to the Drive."""

    name = "assembly_observer"
    experimental = True
    hidden = True
    icon = viser.Icon.HISTORY

    def __init__(self):
        """Nothing recorded yet."""
        self._folder: Path | None = None
        self._problem = ""
        self._writer: ThreadPoolExecutor | None = None
        self._trace = None
        self._watcher: Watcher | None = None
        self._chunk = 0
        self._chunk_started = 0.0
        self._log_seq = 0
        #: The console's count of lines pushed out (`ConsoleLog.dropped`) at the last look.
        self._log_dropped = 0
        self._events = 0
        self._write_error = ""
        self._look_error = ""
        self._commands = 0
        self._objects: tuple[str, ...] = ()
        self._simulated = False
        self._pruned: date | None = None
        #: Log lines from the writer thread, logged at the next tick: (line, whether a warning).
        self._pending_logs: list[tuple[str, bool]] = []
        #: The log line repeating: its run (`Entry.first_seq`), the count last written, its newest entry (`_log_lines`).
        self._repeat_key: tuple | None = None
        self._repeat_written = 0
        self._repeat_seen = None
        self._rows = 0
        self._chunk_started_wall = 0.0
        self._kept_bytes = 0
        #: The last copy's and the last Open in Rerun's outcome for the panel (text, colour), and their tasks.
        self._copy_line = ("", NONE)
        self._copying = None
        self._rerun_line = ("", NONE)
        self._opening = None

    # --- --- --- --- --- HOOKS --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Drop days older than KEPT_DAYS, make the session's folder, start recording and listening, build the panel."""
        robots = dict(ctx.world.robots)
        relay = getattr(ctx.world, "mocap_relay", None)
        simulated = self._simulated = any(robot.base.simulation is not None for robot in robots.values()) or \
            bool(relay is not None and relay.state.simulated)
        self._root = LOCAL_ROOT
        self._writer = ThreadPoolExecutor(1, "observer-writer")
        self._rows = 2 * math.ceil(CHUNK_SECONDS / getattr(ctx.config, "tick_period", 0.05))
        try:
            self._root.mkdir(parents=True, exist_ok=True)
        except OSError as error:
            ctx.log_warn(f"no recordings folder {self._root}: {error}")
        self._prune()
        self._log_pending(ctx)
        try:
            self._folder = run_folder(self._root, time.strftime(DAY_FORMAT), "session", *(["sim"] if simulated else []))
        except OSError as error:
            self._problem = str(error)
            ctx.log_warn(f"nothing recorded: {error}")
        self._build_panel(ctx)
        if self._folder is None:
            return
        description = {
            # * FORMAT: the meaning of the names in this session (`names.py`); the viewer reads older ones its way.
            "kind": "observer session", "format": FORMAT,
            "started_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "ros_time": ctx.now(), "wall_time": time.time(), "chunk_seconds": CHUNK_SECONDS,
            "tick_period": getattr(ctx.config, "tick_period", None),
            "robots": {serial: {"arms": {name: type(arm.end_effector).__name__ if arm.end_effector else None
                                         for name, arm in robot.arms.items()}} for serial, robot in robots.items()},
            "plugins": list(getattr(ctx.config, "enabled_plugins", ())), **environment(simulated)}
        # * On the writer: the commits ask git.
        self._write(lambda: write_description(self._folder, {**description, "commits": commits()}))
        self._write(self._measure)
        self._watcher = Watcher(robots, relay, ctx.now, ctx.world.tracked_objects)
        self._watcher.attach()
        console = getattr(ctx, "console_log", None)
        self._signals = [signal for robot in robots.values()
                         for signal in robot_signals(robot, ctx.now, self._watcher.twists, self._watcher.take_peak)]
        self._signals += relay_signals(relay, ctx.now)
        if console is not None:
            self._signals += monitor_signals(console.late_share)
        self._queue(ctx, [{"event": SESSION_START, "ros_time": ctx.now(), "folder": str(self._folder)}])
        self._start_chunk(ctx)
        ctx.log_info(f"recording the session into {self._folder}")

    def update(self, ctx: PluginContext) -> None:
        """Write this tick's events, start the next chunk when this one is full, and log a copy that ended."""
        self._log_pending(ctx)
        if self._watcher is None:
            return
        self._queue(ctx, self._look(ctx) + self._log_lines(ctx))
        # * A new chunk also when the tracked objects change: a chunk's columns are fixed.
        if ctx.now() - self._chunk_started >= CHUNK_SECONDS or tuple(ctx.world.tracked_objects) != self._objects:
            self._end_chunk(ctx)
            self._start_chunk(ctx)

    def _look(self, ctx: PluginContext) -> list[dict]:
        """The watcher's events; an error is one "observer error" event, never a failed tick.

        ! A plugin whose hooks fail too often is stopped, and the recording with it.
        """
        try:
            events, errors = self._watcher.look(getattr(ctx, "clearance_trips", ()))
        except Exception as error:
            events, errors = [], [repr(error)]
        return events + self._errors(ctx, errors)

    def _snapshot(self, ctx: PluginContext) -> list[dict]:
        """Every state now (`Watcher.snapshot`); an error is one "observer error" event, as in `_look`."""
        try:
            events, errors = self._watcher.snapshot()
        except Exception as error:
            events, errors = [], [repr(error)]
        return events + self._errors(ctx, errors)

    def _errors(self, ctx: PluginContext, errors: list[str]) -> list[dict]:
        """The first error as an "observer error" event, unless it is the one told last."""
        events = []
        if errors and errors[0] != self._look_error:
            self._look_error = errors[0]
            events.append({"event": OBSERVER_ERROR, "ros_time": ctx.now(), "error": errors[0]})
        return events

    def draw(self, ctx: PluginContext) -> None:
        """Whether it records, where, how much is kept, and the last copy's and Open in Rerun's outcomes."""
        self._copied.content = "".join(block(chip_line(chip(text, color))) for text, color in
                                       (self._copy_line, self._rerun_line) if text)
        if self._folder is None:
            self._chips.content = chip_line(chip("not recording", FAIL, self._problem))
            self._lines.content = block(values(self._problem or "—"))
            return
        state = chip("write failed", FAIL, self._write_error) if self._write_error else \
            chip("recording", OK, f"every tick and every event, on this PC for {KEPT_DAYS} days")
        self._chips.content = chip_line(state + chip(f"chunk {self._chunk}", NONE, "the chunk being recorded")
                                        + chip(f"{self._events} events", NONE, "events written this session")
                                        + chip(f"{self._kept_bytes / 1e6:.0f} MB kept", NONE,
                                               f"every recording on this PC, the last {KEPT_DAYS} days"))
        # * One line: the copy's controls below never move (a write error is in the chip's hover).
        self._lines.content = block(values(str(self._folder)))

    def teardown(self, ctx: PluginContext) -> None:
        """Save the chunk being recorded and the last events, then wait for the writer."""
        try:
            if self._watcher is not None:
                self._watcher.detach()
                self._queue(ctx, self._look(ctx) + self._log_lines(ctx) + self._repeat_end(ctx)
                            + [{"event": SESSION_END, "ros_time": ctx.now()}])
        finally:
            self._end_chunk(ctx)
            if self._writer is not None:
                self._writer.shutdown(wait=True)

    # --- --- --- --- --- CHUNKS --- --- --- --- ---

    def _start_chunk(self, ctx: PluginContext) -> None:
        """Start recording the next chunk, with every state as it is now; a new day drops the days past KEPT_DAYS."""
        self._chunk += 1
        self._chunk_started, self._chunk_started_wall = ctx.now(), time.time()
        if date.today() != self._pruned:
            # * On the writer: a whole day's files take a while to delete.
            self._pruned = date.today()
            self._write(self._prune)
            self._write(self._measure)
        self._objects = tuple(ctx.world.tracked_objects)
        objects = object_signals({name: ctx.world.tracked_objects[name] for name in self._objects})
        try:
            self._trace = ctx.trace(*self._signals, *objects, max_samples=self._rows)
        except ValueError as error:
            # * Not one tick failed for it: the next chunk tries again.
            self._trace = None
            ctx.log_error(f"chunk {self._chunk} not recorded: {error}")
        # * Each chunk opens with every state, so a copy from any time knows where things stood.
        self._queue(ctx, self._snapshot(ctx))

    def _log_pending(self, ctx: PluginContext) -> None:
        """Log the lines the writer left (`_pending_logs`)."""
        while self._pending_logs:
            line, failed = self._pending_logs.pop(0)
            (ctx.log_warn if failed else ctx.log_info)(line)

    def _prune(self) -> None:
        """Drop the day folders past KEPT_DAYS but this session's; logged at the next tick, a failure too."""
        self._pruned = date.today()
        keep = self._folder.parent.name if self._folder is not None else ""
        try:
            for day in prune(self._root, self._pruned, keep=keep):
                self._pending_logs.append((f"recordings of {day} deleted: kept {KEPT_DAYS} days", False))
        except OSError as error:
            self._pending_logs.append((f"old recordings not deleted: {error}", True))

    def _end_chunk(self, ctx: PluginContext) -> None:
        """Stop the chunk being recorded and save it on the writer."""
        trace, self._trace = self._trace, None
        if trace is None:
            return
        ctx.untrace(trace)
        if not len(trace):
            return
        path = self._folder / f"chunk_{self._chunk:03d}.npz"
        self._write(_save_chunk, trace, path)
        # * A running total, not a rescan of the week per chunk; measured in full at the start and after a prune.
        self._write(self._grow, path)

    # --- --- --- --- --- PANEL AND COPIES --- --- --- --- ---

    def _build_panel(self, ctx: PluginContext) -> None:
        """The recording's chips and folder, then the range: day, from, to, name, the two buttons, their outcomes."""
        today = time.strftime(DAY_FORMAT)
        with ctx.view.ui() as gui:
            gui.add_html(section("observer", SECTION_SENSOR))
            self._chips = gui.add_html(chip_line(""))
            self._lines = gui.add_html("")
            gui.add_html(section("copy or open a range", SECTION_SENSOR))
            self._day = gui.add_dropdown("Day", options=sorted(set(days(self._root)) | {today}, reverse=True),
                                         initial_value=today, hint=f"Recordings stay on this PC {KEPT_DAYS} days")
            self._from = gui.add_text("From", initial_value=time.strftime("%H:%M"), hint="HH:MM or HH:MM:SS, that day")
            self._to = gui.add_text("To", initial_value=NOW, hint=f"HH:MM or HH:MM:SS, or {NOW}")
            ranges = gui.add_button_group("Range", [LAST_15, LAST_HOUR, WHOLE_DAY])
            self._name = gui.add_text("Name", initial_value="experiment",
                                      hint="The experiment's folder under data_experiment/assembly_observer/")
            # * Draws nothing: Copy to Drive and Open in Rerun side by side, once they exist.
            pair = gui.add_html("")
            copy = gui.add_button("Copy to Drive", hint="Copy what was recorded from From to To into a new folder on "
                                                        "the Drive: its events, the ticks' values, the commands sent")
            viewer = gui.add_button("Open in Rerun", hint="Show what was recorded from From to To in the Rerun viewer "
                                                          "on this PC: plots, state timelines, events, robots in 3D")
            pair.content = f"<style>{side_by_side_css([copy._impl.uuid, viewer._impl.uuid])}</style>"
            self._copied = gui.add_html("")
        ranges.on_click(ctx.defer_value("range", self._pick_range))
        copy.on_click(ctx.defer("copy to the Drive", lambda: self._copy(ctx)))
        viewer.on_click(ctx.defer("open in rerun", lambda: self._open_rerun(ctx)))

    def _pick_range(self, label: str) -> None:
        """Fill Day, From and To from a Range button."""
        now = datetime.now()
        if label == WHOLE_DAY:
            self._from.value, self._to.value = "00:00", "23:59:59"
            return
        self._day.value = now.strftime(DAY_FORMAT)
        since = now - timedelta(minutes=15 if label == LAST_15 else 60)
        self._from.value, self._to.value = since.strftime("%H:%M:%S"), NOW

    def _refusal(self, ctx: PluginContext, running) -> str:
        """Why the range chosen cannot go to the helper script now, "" if it can.

        When it can and the range reaches into the chunk being recorded, that chunk is saved first.

        Args:
            ctx: The plugin's context.
            running: The task of the last run of this button, None before the first.
        """
        try:
            start, end = wall_time(self._day.value, self._from.value), wall_time(self._day.value, self._to.value)
        except ValueError as error:
            return str(error)
        if end <= start:
            return f"To ({self._to.value}) is not after From ({self._from.value})"
        if running is not None and not running.done():
            return "the last one is still running"
        if not SCRIPT.is_file():
            return f"no helper script {SCRIPT} (the monitor installed without --symlink-install?)"
        if self._trace is not None and end >= self._chunk_started_wall:
            # * This tick's changes first, so the new chunk's "state" lines never swallow one (this intent runs before
            #   `update`'s look).
            if self._watcher is not None:
                self._queue(ctx, self._look(ctx) + self._log_lines(ctx))
            self._end_chunk(ctx)
            self._start_chunk(ctx)
        return ""

    def _copy(self, ctx: PluginContext) -> None:
        """Copy the range chosen to a new folder on the Drive, in the helper script's own process (SCRIPT)."""
        try:
            drive_folder(ctx.config.drive_root, RECORDING_FOLDER)
            why = self._refusal(ctx, self._copying)
        except OSError as error:
            why = str(error)
        if why:
            self._copy_line = (f"not copied: {why}", FAIL)
            ctx.log_warn(f"copy to the Drive refused: {why}")
            return
        command = [sys.executable, str(SCRIPT), "--root", str(self._root), "copy", self._from.value, self._to.value,
                   "--day", self._day.value, "--name", folder_name(self._name.value, "experiment"),
                   "--drive", str(ctx.config.drive_root), *(["--sim"] if self._simulated else [])]
        self._copy_line = (f"copying {self._day.value} {self._from.value} to {self._to.value}...", BUSY)
        # * Only records: a soft stop leaves the copy running.
        work = self._run_helper(ctx, command, COPY_TIMEOUT, "_copy_line", copy_line)
        self._copying = ctx.spawn("copy to the Drive", work, survives_soft_stop=True)

    def _open_rerun(self, ctx: PluginContext) -> None:
        """Open the range chosen in the Rerun viewer, sent from the helper script's own process (SCRIPT)."""
        why = NO_RERUN if find_spec("rerun") is None else self._refusal(ctx, self._opening)
        if why:
            self._rerun_line = (f"not opened: {why}", FAIL)
            ctx.log_warn(f"open in Rerun refused: {why}")
            return
        command = [sys.executable, str(SCRIPT), "--root", str(self._root), "rerun", self._from.value, self._to.value,
                   "--day", self._day.value]
        self._rerun_line = (f"opening {self._day.value} {self._from.value} to {self._to.value} in Rerun...", BUSY)
        work = self._run_helper(ctx, command, RERUN_TIMEOUT, "_rerun_line", rerun_line)
        self._opening = ctx.spawn("open in rerun", work, survives_soft_stop=True)

    async def _run_helper(self, ctx: PluginContext, command: list[str], timeout: float, shown: str, read) -> None:
        """Wait for the writer (the chunk just saved), run the helper script, then say what it did on the panel and log.

        ! The script is killed after `timeout` and when this task is cancelled (the plugin stopping): the monitor never
          waits on it. Stop all leaves it running: it only reads and copies. The Rerun viewer it opened stays open.
        ! Its output goes to files, never a pipe: a viewer it starts inherits them and outlives it.

        Args:
            ctx: The plugin's context.
            command: The script's command line.
            timeout: Seconds it may take.
            shown: The attribute holding the panel's line for it (`_copy_line`, `_rerun_line`).
            read: The script's last JSON line -> the line to show and whether it failed.
        """
        await ctx.run_in_thread(self._writer.submit(lambda: None).result)
        try:
            with tempfile.TemporaryFile("w+") as output, tempfile.TemporaryFile("w+") as errors:
                process = subprocess.Popen(command, stdout=output, stderr=errors, text=True)
                started = time.monotonic()
                try:
                    while process.poll() is None:
                        if time.monotonic() - started > timeout:
                            raise TimeoutError(f"no end after {timeout / 60:.0f} min; stopped")
                        await ctx.sleep(0.5)
                finally:
                    await _stop(ctx, process)
                output.seek(0)
                errors.seek(0)
                stdout, stderr = output.read(), errors.read()
            outcome = json.loads(stdout.strip().splitlines()[-1]) if stdout.strip() else \
                {"error": (stderr.strip().splitlines() or ["no answer"])[-1]}
        except (OSError, ValueError, TimeoutError) as error:
            outcome = {"error": str(error)}
        line, failed = read(outcome)
        setattr(self, shown, (line, FAIL if failed else OK))
        (ctx.log_warn if failed else ctx.log_info)(line)

    def _measure(self) -> None:
        """Note how much is kept on this PC. Writer thread."""
        self._kept_bytes = size(self._root)

    def _grow(self, path: Path) -> None:
        """Add a saved chunk to what is kept. Writer thread.

        The command files and events count at the next full measure (`_measure`).
        """
        try:
            self._kept_bytes += path.stat().st_size
        except OSError:
            return  # * the save failed: its own error stays on the panel

    # --- --- --- --- --- EVENTS --- --- --- --- ---

    def _log_lines(self, ctx: PluginContext) -> list[dict]:
        """The log lines added since the last call, from INFO up.

        A line repeating is written again each time its count passes 2, 10, 100, ... since it was last written, and
        with its final count once another line follows.
        """
        console = getattr(ctx, "console_log", None)
        if console is None:
            return []
        # * Read before `since`; a line pushed out between the two reads is lost untold (a window of microseconds).
        dropped, dropped_seq = console.dropped, console.dropped_seq
        lost = dropped_seq > self._log_seq and dropped > self._log_dropped
        lost_events = [{"event": LOG_LINES_LOST, "ros_time": ctx.now(), "at_most": dropped - self._log_dropped}
                       ] if lost else []
        self._log_dropped = dropped
        entries = console.since(self._log_seq)
        if entries:
            self._log_seq = entries[-1].seq
        lines = []
        for entry in entries:
            if entry.level < LOG_LEVEL or entry.source in SKIPPED_SOURCES:
                continue
            # * A run of repeats is one console entry, known by its first line: the same text back is another run.
            key = entry.first_seq
            if key != self._repeat_key:
                if self._repeat_seen is not None and self._repeat_seen.count > self._repeat_written:
                    lines.append(self._repeat_seen)
                self._repeat_key, self._repeat_written = key, entry.count
                lines.append(entry)
            elif _passed(self._repeat_written, entry.count):
                self._repeat_written = entry.count
                lines.append(entry)
            self._repeat_seen = entry
        return lost_events + [{"event": LOG, "ros_time": ctx.now(), "time": _local(entry.time),
                               "level": entry.level.name, "source": entry.source, "text": entry.text,
                               "count": entry.count} for entry in lines]

    def _repeat_end(self, ctx: PluginContext) -> list[dict]:
        """The final count of the line repeating, if not written yet: at the end of the session."""
        seen = self._repeat_seen
        if seen is None or seen.count <= self._repeat_written:
            return []
        self._repeat_written = seen.count
        return [{"event": LOG, "ros_time": ctx.now(), "time": _local(seen.time), "level": seen.level.name,
                 "source": seen.source, "text": seen.text, "count": seen.count}]

    def _queue(self, ctx: PluginContext, events: list[dict]) -> None:
        """Write events, stamped with the local time now unless they carry one."""
        if not events or self._folder is None:
            return
        now = _local(time.time())
        for event in events:
            if FILE_KEY in event:
                self._commands += 1
                parts = [f"{self._commands:05d}", event["event"].replace(" ", "_"), event.get("robot", ""),
                         event.get("arm", "")]
                name = f"{COMMANDS}/{'_'.join(part for part in parts if part)}.json"
                self._write(_write_message, self._folder / name, event.pop(FILE_KEY))
                event["file"] = name
        lines = [json.dumps(_plain({"time": now, **event}), allow_nan=False) for event in events]
        self._events += len(lines)
        self._write(_append, self._folder / EVENTS, lines)

    def _write(self, function, *args: Any) -> Future:
        """Run a write on the writer thread; a failure is kept for the panel."""
        future = self._writer.submit(function, *args)
        future.add_done_callback(self._written)
        return future

    def _written(self, future: Future) -> None:
        """Keep the last write's failure, if any."""
        error = future.exception()
        if error is not None:
            self._write_error = f"a write failed: {error!r}"


# --- --- --- --- --- FILES (writer thread) --- --- --- --- ---

async def _stop(ctx: PluginContext, process: subprocess.Popen) -> None:
    """End a helper script still running: ask, then kill it after 2 s; polls, never blocks the loop."""
    if process.poll() is not None:
        return
    process.terminate()
    try:
        for _ in range(20):
            await ctx.sleep(0.1)
            if process.poll() is not None:
                return
    finally:
        if process.poll() is None:
            process.kill()  # * also when cancelled again meanwhile: reaped by the poll below or by Popen later
    while process.poll() is None:
        await ctx.sleep(0.05)


def copy_line(outcome: dict) -> tuple[str, bool]:
    """The line a copy's outcome (the helper script's last JSON line) shows, and whether it failed."""
    if "error" in outcome:
        return f"copy failed: {outcome['error']}", True
    copied = outcome.get("copied") or {}
    if not copied:
        return "nothing was recorded in that range", True
    events = sum(each["events"] for each in copied.values())
    rows = sum(each["rows"] for each in copied.values())
    skipped = sum(each.get("skipped", 0) for each in copied.values())
    line = f"copied {len(copied)} session(s), {events} events, {rows} ticks to {outcome['destination']}"
    return line + (f"; {skipped} damaged line(s) or chunk(s) left out" if skipped else ""), False


def rerun_line(outcome: dict) -> tuple[str, bool]:
    """The line Open in Rerun's outcome (the helper script's last JSON line) shows, and whether it failed."""
    if "error" in outcome:
        return f"not opened: {outcome['error']}", True
    if not outcome.get("sessions"):
        return (outcome.get("notes") or ["nothing was recorded in that range"])[0], True
    line = f"opened {len(outcome['sessions'])} session(s), {outcome['events']} events, {outcome['rows']} ticks in Rerun"
    return line + "".join(f"; {note}" for note in outcome.get("notes", ())), False


def _save_chunk(trace, path: Path) -> Path:
    """Save a chunk's trace, compact, under a hidden name first: a chunk file is never half written."""
    hidden = path.with_name("." + path.name)
    trace.save(hidden, compact=True)
    hidden.replace(path)
    return path


def _write_message(path: Path, message: Any) -> None:
    """Write a ROS message sent as a JSON file, making its folder."""
    path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(_plain(message_to_ordereddict(message)), indent=1, allow_nan=False) + "\n")


def _append(path: Path, lines: list[str]) -> None:
    """Append lines to a file, flushed to disk."""
    with open(path, "a", encoding="utf-8") as handle:
        handle.write("".join(line + "\n" for line in lines))
        handle.flush()


def _local(wall: float) -> str:
    """A wall time as a local ISO time to the millisecond."""
    return datetime.fromtimestamp(wall).astimezone().isoformat(timespec="milliseconds")


def _passed(written: int, count: int) -> bool:
    """Whether a repeat count went past 2, 10, 100, 1000, ... since `written` was written."""
    return (written < 2 <= count) or math.floor(math.log10(count)) > math.floor(math.log10(written))


def _plain(value: Any) -> Any:
    """JSON-safe: tuples and arrays as lists, numbers as floats, NaN and infinities as None, anything else as text."""
    if hasattr(value, "tolist") and not isinstance(value, (bool, int, float)):
        value = value.tolist()  # numpy arrays and numbers, a message's array.array
    if isinstance(value, dict):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return value
    if isinstance(value, int):
        return value
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    return number if math.isfinite(number) else None
