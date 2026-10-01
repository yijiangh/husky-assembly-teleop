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
import time
import traceback
from collections.abc import Coroutine as CoroutineABC
from contextlib import contextmanager
from typing import TYPE_CHECKING, Any, Callable, Coroutine, Iterator

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


class TickTimer:
    """Times every tick's parts and every plugin task's steps; logs where the time went when a tick starts late.

    The monitor wraps each part of a tick in `part`, and `ctx.spawn` wraps each task in `timed_task`, which times
    each step the task runs between two `await`s. A late tick's report lists everything measured since the
    previous tick started, slowest first: a full account, not a sample.

    ? Each part is wall time, so it includes waiting for the GIL while another thread runs Python.
    ? "idle" is the loop blocked waiting for events; "other" is what no part covers: loop callbacks (ROS futures,
      viser events, thread results) and the loop itself.
    ! Main thread only.
    """

    #: Parts shorter than this (seconds) are added up into one line of the report.
    SMALL_PART = 0.001

    def __init__(self, loop: asyncio.AbstractEventLoop, late_after: float, repeat_after: float,
                 log_warn: Callable[[str], None]):
        """Start measuring.

        Args:
            loop: The monitor's loop; the time it waits for events counts as idle.
            late_after: Seconds after it was due that a tick counts as late.
            repeat_after: Seconds between two reports; the worst late tick in between is reported next.
            log_warn: Receives each report.
        """
        self._late_after = late_after
        self._repeat_after = repeat_after
        self._log_warn = log_warn
        # Seconds per part since the current tick started, and a note per part (e.g. a message count).
        self._parts: dict[str, float] = {}
        self._notes: dict[str, str] = {}
        # When the current tick started (perf_counter), and seconds the loop was idle since.
        self._started: float | None = None
        self._idle = 0.0
        self._gc_started = 0.0
        # Late ticks since the last report, and the worst of them as (seconds late, report).
        self._late_count = 0
        self._worst: tuple[float, str] | None = None
        self._last_report = -float("inf")
        gc.callbacks.append(self._time_gc)
        # * Idle is the time the loop blocks in its selector waiting for events: time that call.
        #   ? `_selector` is private to asyncio's selector loops; without one, idle time shows as "other".
        self._selector = getattr(loop, "_selector", None)
        if self._selector is not None:
            select = self._selector.select

            def timed_select(timeout: float | None = None) -> list:
                started = time.perf_counter()
                try:
                    return select(timeout)
                finally:
                    self._idle += time.perf_counter() - started

            self._selector.select = timed_select

    @contextmanager
    def part(self, name: str) -> Iterator[None]:
        """Time the block as part `name` of the current tick, e.g. "plugin 'health' update"."""
        started = time.perf_counter()
        try:
            yield
        finally:
            self.add(name, time.perf_counter() - started)

    def add(self, name: str, seconds: float) -> None:
        """Add `seconds` to part `name` of the current tick."""
        self._parts[name] = self._parts.get(name, 0.0) + seconds

    def note(self, name: str, text: str) -> None:
        """Show `text` next to part `name` in a report, e.g. how many messages it handled."""
        self._notes[name] = text

    def timed_task(self, name: str, work: Coroutine[Any, Any, Any]) -> Coroutine[Any, Any, Any]:
        """Wrap a task's coroutine so each of its steps adds to part "task `name`"."""
        return _TimedCoroutine(work, lambda seconds: self.add(f"task {name!r}", seconds))

    def start_tick(self, index: int, late: float) -> None:
        """Close the previous tick's account, reporting it if this tick is late, and start a new one.

        Args:
            index: This tick's number.
            late: Seconds this tick started after it was due.
        """
        now = time.perf_counter()
        if self._started is not None and late > self._late_after:
            self._late_count += 1
            if self._worst is None or late > self._worst[0]:
                self._worst = (late, self._describe(index, late, now - self._started))
        if self._worst is not None and now - self._last_report >= self._repeat_after:
            more = (f"\n{self._late_count - 1} other late ticks since the last report, this was the worst."
                    if self._late_count > 1 else "")
            self._log_warn(self._worst[1] + more)
            self._last_report, self._late_count, self._worst = now, 0, None
        self._started, self._idle = now, 0.0
        self._parts.clear()
        self._notes.clear()

    def stop(self) -> None:
        """Stop timing garbage collection and idle time. Safe to call twice."""
        if self._time_gc in gc.callbacks:
            gc.callbacks.remove(self._time_gc)
        if self._selector is not None and "select" in vars(self._selector):
            del self._selector.select  # back to the class's own

    def _describe(self, index: int, late: float, cycle: float) -> str:
        """Write the report for one late tick: every part since the previous tick started, slowest first."""
        parts = sorted(self._parts.items(), key=lambda item: -item[1])
        lines = [f"  {seconds * 1e3:6.1f} ms  {name}" + (f" ({self._notes[name]})" if name in self._notes else "")
                 for name, seconds in parts if seconds >= self.SMALL_PART]
        small = [seconds for _, seconds in parts if seconds < self.SMALL_PART]
        if small:
            lines.append(f"  {sum(small) * 1e3:6.1f} ms  {len(small)} parts under {self.SMALL_PART * 1e3:.0f} ms each")
        lines.append(f"  {self._idle * 1e3:6.1f} ms  idle")
        other = cycle - sum(self._parts.values()) - self._idle
        if other >= self.SMALL_PART:
            lines.append(f"  {other * 1e3:6.1f} ms  other")
        hint = ""
        if parts and parts[0][0].startswith(("plugin", "task")):
            hint = ("\n! A slow plugin hook or task blocks every robot and plugin: "
                    "move slow work to `ctx.run_in_thread`.")
        return (f"tick {index} started {late * 1e3:.0f} ms late; the {cycle * 1e3:.0f} ms since the previous tick "
                f"started went to:\n" + "\n".join(lines) + hint)

    def _time_gc(self, phase: str, _info: dict) -> None:
        """Add garbage collection (on any thread) as a part. Called by `gc` before and after each collection.

        ? The one call from other threads; a race with the main thread only misplaces one collection's time.
        """
        if phase == "start":
            self._gc_started = time.perf_counter()
        else:
            self.add("garbage collection", time.perf_counter() - self._gc_started)


class _TimedCoroutine(CoroutineABC):
    """A coroutine that reports how long each of its steps took: each run between two `await`s.

    ? asyncio drives a task only through `send` and `throw`, so wrapping them times every step.
    """

    def __init__(self, work: Coroutine[Any, Any, Any], on_step: Callable[[float], None]):
        """Wrap `work`; `on_step(seconds)` is called after each step."""
        self._work = work
        self._on_step = on_step

    def send(self, value: Any) -> Any:
        """Run one step of the wrapped coroutine."""
        started = time.perf_counter()
        try:
            return self._work.send(value)
        finally:
            self._on_step(time.perf_counter() - started)

    def throw(self, *args: Any) -> Any:
        """Raise inside the wrapped coroutine (e.g. a cancel) and run it to its next `await`."""
        started = time.perf_counter()
        try:
            return self._work.throw(*args)
        finally:
            self._on_step(time.perf_counter() - started)

    def close(self) -> None:
        """Close the wrapped coroutine."""
        self._work.close()

    def __await__(self) -> Iterator[Any]:
        """Iterate the steps, for `await`."""
        return self

    def __iter__(self) -> Iterator[Any]:
        """Iterate the steps."""
        return self

    def __next__(self) -> Any:
        """Run one step."""
        return self.send(None)
