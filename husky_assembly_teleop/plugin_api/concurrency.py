"""
Plugin concurrency: asyncio on the main thread, driven by the monitor's tick.

The tick, ROS callbacks and every task (`ctx.spawn`) share one thread, so between two `await`s a task needs
no locks. For timeouts use `ctx.wait_until(..., timeout_s=)` (ROS clock) or `async_timeout.timeout` (wall clock).

- ! Never block in a task: await `ctx.wait_until` / `ctx.sleep`, or use `ctx.run_in_thread` for heavy work.
- ! Never swallow `asyncio.CancelledError` (bare `except:`, `except BaseException:`): cancelling is how jobs,
  soft stop and shutdown end a task.
- ! Never `await` inside `with mirror.active()`: another task would run with pybullet_planning on the wrong client.
"""

from __future__ import annotations

import asyncio
import gc
import sys
import threading
import time
import traceback
from contextlib import contextmanager
from typing import TYPE_CHECKING, Callable, Iterator

if TYPE_CHECKING:
    from rclpy.task import Future as RosFuture


class WaitTimeout(asyncio.TimeoutError):
    """Raised by `ctx.wait_until` when a condition does not come true in time; an `asyncio.TimeoutError`."""


def ros_future(future: RosFuture) -> asyncio.Future:
    """Wrap an rclpy future (`call_async`, `send_goal_async`) as an asyncio future.

    ? Awaiting an rclpy future directly busy-spins under asyncio; its callbacks run on this thread.

    ! Cancelling the awaiting task does not cancel the ROS request or goal.
    """
    loop = asyncio.get_running_loop()
    result = loop.create_future()

    def copy(done: RosFuture) -> None:
        if result.done():
            return  # the awaiter was cancelled
        if done.cancelled():
            result.cancel()
        elif done.exception() is not None:
            result.set_exception(done.exception())
        else:
            result.set_result(done.result())

    future.add_done_callback(copy)
    return result


def describe_exception(error: BaseException) -> str:
    """Format an exception with its traceback, for the log."""
    return "".join(traceback.format_exception(type(error), error, error.__traceback__))


#: Functions in which a thread waits without working, e.g. the loop polling for events.
_WAITS = ("select", "poll", "wait", "sleep", "wait_for_ready_callbacks")


class LoopWatchdog:
    """Logs the main thread's stack when a plugin hook or task holds the thread too long.

    A thread watches a heartbeat the loop bumps every few ms. The monitor wraps each plugin hook in `watching`,
    so a slow hook is timed on its own and named in the report; otherwise the report names the running task.
    Each stall is reported twice: once when it passes the limit, with the stack, and once when it ends, with
    its full length.

    ? The watchdog thread needs the GIL, so it can wake late while the loop runs numpy or C code: the first
      report's time is then past the limit, and its stack is wherever the main thread happened to be.
    ? A stall is not always the main thread's own work: garbage collection (on any thread) and other threads
      holding the GIL stall it too. So the first report also lists the other threads, and the second the time
      spent collecting garbage during the stall.
    """

    #: Seconds between heartbeats, and between checks.
    PERIOD = 0.005

    def __init__(self, loop: asyncio.AbstractEventLoop, limit: float, repeat_after: float,
                 log_warn: Callable[[str], None]):
        """Start watching.

        Args:
            loop: The monitor's loop. Must be running in the calling thread.
            limit: Seconds without a heartbeat before a stall is reported.
            repeat_after: Seconds between two reports with the same name.
            log_warn: Receives each report. Called from the watchdog thread.
        """
        self._loop = loop
        self._limit = limit
        self._repeat_after = repeat_after
        self._log_warn = log_warn
        self._beat = time.monotonic()
        # Seconds spent collecting garbage so far, on any thread; and that total at the last heartbeat.
        self._gc_total = 0.0
        self._gc_at_beat = 0.0
        self._gc_started = 0.0
        gc.callbacks.append(self._time_gc)
        # Set by `watching`; None names the running task instead.
        self._label: str | None = None
        self._last_report: dict[str, float] = {}
        self._main_thread = threading.get_ident()
        self._stopped = threading.Event()
        self._heartbeat = loop.call_soon(self._bump)
        self._thread = threading.Thread(target=self._watch, name="loop-watchdog", daemon=True)
        self._thread.start()

    @contextmanager
    def watching(self, label: str) -> Iterator[None]:
        """Time the block on its own clock and name it `label` in a report, e.g. "plugin 'health' update"."""
        self._label = label
        self._beat = time.monotonic()
        try:
            yield
        finally:
            self._label = None
            self._beat = time.monotonic()

    def stop(self) -> None:
        """Stop watching. Safe to call twice."""
        self._stopped.set()
        self._heartbeat.cancel()
        if self._time_gc in gc.callbacks:
            gc.callbacks.remove(self._time_gc)

    def _time_gc(self, phase: str, _info: dict) -> None:
        """Add up the time spent collecting garbage. Called by `gc` before and after each collection."""
        if phase == "start":
            self._gc_started = time.monotonic()
        else:
            self._gc_total += time.monotonic() - self._gc_started

    def _bump(self) -> None:
        """Record that the loop is alive, and schedule the next bump. On the loop."""
        self._beat = time.monotonic()
        self._gc_at_beat = self._gc_total
        self._heartbeat = self._loop.call_later(self.PERIOD, self._bump)

    def _watch(self) -> None:
        """Poll the heartbeat and report each stall once. On the watchdog thread."""
        reported_beat, reported_name, gc_before = None, "", 0.0
        while not self._stopped.wait(self.PERIOD):
            beat = self._beat
            if reported_beat is not None and beat != reported_beat:
                # ? `beat` is the newest heartbeat after the stall: late only by however long this thread slept past it.
                self._log_warn(f"{reported_name} released the main thread after {(beat - reported_beat) * 1e3:.0f} ms"
                               f"; {(self._gc_total - gc_before) * 1e3:.0f} ms of it collecting garbage")
                reported_beat = None
            stalled = time.monotonic() - beat
            if stalled < self._limit or beat == reported_beat:
                continue
            # ? Read from another thread; a race only mislabels one report.
            name = self._label
            if name is None:
                task = asyncio.current_task(self._loop)
                name = f"task {task.get_name()!r}" if task is not None else "a loop callback"
            now = time.monotonic()
            if now - self._last_report.get(name, -float("inf")) < self._repeat_after:
                continue
            reported_beat, reported_name, gc_before = beat, name, self._gc_at_beat
            self._last_report[name] = now
            frames = sys._current_frames()
            frame = frames.get(self._main_thread)
            stack = "".join(traceback.format_stack(frame)[-6:]) if frame is not None else ""
            if frame is not None and frame.f_code.co_name in _WAITS:
                stack += ("! The main thread is only waiting here: another thread (below) or garbage collection "
                          "holds the GIL.\n")
            self._log_warn(f"{name} is holding the main thread ({stalled * 1e3:.0f} ms so far), blocking every "
                           f"robot and plugin; move slow work to `ctx.run_in_thread`. Now at:\n{stack}"
                           f"Other threads now at:\n{self._other_threads(frames)}")

    def _other_threads(self, frames: dict) -> str:
        """One line per thread other than the main thread and this one: its name and innermost line.

        ? A thread in a wait (`wait`, `select`, `poll`, `sleep`) is idle; one anywhere else may be what holds the GIL.
        """
        names = {thread.ident: thread.name for thread in threading.enumerate()}
        lines = []
        for ident, frame in frames.items():
            if ident in (self._main_thread, threading.get_ident()):
                continue
            where = traceback.extract_stack(frame)[-1]
            lines.append(f"  {names.get(ident, ident)}: {where.filename}:{where.lineno} in {where.name}\n")
        return "".join(lines)
