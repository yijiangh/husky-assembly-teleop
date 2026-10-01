"""
Plugin concurrency: asyncio on the main thread, driven by the monitor's tick.

Plugin work longer than one tick is a task: an `async def` started with
`ctx.spawn`. The tick, ROS callbacks, PyBullet and every task share one thread,
so between two `await`s a task has the thread to itself (no locks needed).

- ! Never block in a task: `time.sleep`, a busy loop or a long computation
  freezes the whole node. Await `ctx.wait_until`, `ctx.sleep`, or push heavy
  work off the thread with `ctx.run_in_thread`.
- ! Never catch `asyncio.CancelledError` without re-raising, and never use a bare
  `except:` / `except BaseException:` that swallows it. Cancelling is how
  jobs, soft stop and shutdown end a task.
- ! Never `await` inside `with <other client>.active()`: another task would run
  with pybullet_planning pointed at the wrong client.

For timeouts, `ctx.wait_until(..., timeout_s=)` counts on the ROS clock. For a
block of awaits use `timeout` (the 3.11 `asyncio.timeout` backport, wall
clock); both raise `asyncio.TimeoutError`.

    async with timeout(10):
        await ctx.ros(client.call_async(request))
        await ctx.wait_until(arrived, description="the arm to arrive")
"""

from __future__ import annotations

import asyncio
import sys
import threading
import time
import traceback
from typing import TYPE_CHECKING, Any, Callable

from async_timeout import timeout  # noqa: F401  (re-exported for plugins)

if TYPE_CHECKING:
    from rclpy.task import Future as RosFuture


class WaitTimeout(asyncio.TimeoutError):
    """Raised by `ctx.wait_until` when a condition does not come true in time.

    A subclass of `asyncio.TimeoutError`, so one `except asyncio.TimeoutError`
    also catches the timeouts of `timeout(...)` and `asyncio.wait_for`.
    """


def ros_future(future: RosFuture) -> asyncio.Future:
    """Wrap an rclpy future (`call_async`, `send_goal_async`) as an asyncio future.

    ? rclpy futures are awaitable, but only inside rclpy's own executor; under
      asyncio they would busy-spin. Their done callbacks run inside the monitor's
      ROS pump, on this thread, so the result can be set directly.

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


class LoopWatchdog:
    """Logs where the loop is stuck when a task holds the thread too long.

    ? asyncio cannot time each task step cheaply, so a thread watches a
      heartbeat that the loop bumps every few milliseconds. When it goes quiet
      for longer than `limit`, the watchdog logs the main thread's stack: the
      line that blocks, not just the plugin.

    ! Stalls inside `ignore` (the tick task) are not reported here; the monitor
      times plugin hooks itself.
    """

    #: Seconds between heartbeats, and between checks.
    PERIOD = 0.005

    def __init__(self, loop: asyncio.AbstractEventLoop, limit: float, repeat_after: float,
                 log_warn: Callable[[str], None], ignore: Callable[[], Any]):
        """Start watching.

        Args:
            loop: The monitor's loop. Must be running in the calling thread.
            limit: Seconds without a heartbeat before a stall is reported.
            repeat_after: Seconds between two reports of the same task.
            log_warn: Receives each report. Called from the watchdog thread.
            ignore: Returns the task whose stalls are not reported, or None.
        """
        self._loop = loop
        self._limit = limit
        self._repeat_after = repeat_after
        self._log_warn = log_warn
        self._ignore = ignore
        self._beat = time.monotonic()
        self._last_report: dict[str, float] = {}
        self._main_thread = threading.get_ident()
        self._stopped = threading.Event()
        self._heartbeat = loop.call_soon(self._bump)
        self._thread = threading.Thread(target=self._watch, name="loop-watchdog", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop watching. Safe to call twice."""
        self._stopped.set()
        self._heartbeat.cancel()

    def _bump(self) -> None:
        """Record that the loop is alive, and schedule the next bump. On the loop."""
        self._beat = time.monotonic()
        self._heartbeat = self._loop.call_later(self.PERIOD, self._bump)

    def _watch(self) -> None:
        """Poll the heartbeat and report each stall once. On the watchdog thread."""
        reported_beat = None
        while not self._stopped.wait(self.PERIOD):
            beat = self._beat
            stalled = time.monotonic() - beat
            if stalled < self._limit or beat == reported_beat:
                continue
            # ? Reads asyncio's current-task table from another thread; a race
            #   only mislabels one report.
            task = asyncio.current_task(self._loop)
            if task is None or task is self._ignore():
                continue
            name = task.get_name()
            now = time.monotonic()
            if now - self._last_report.get(name, -float("inf")) < self._repeat_after:
                continue
            reported_beat = beat
            self._last_report[name] = now
            frame = sys._current_frames().get(self._main_thread)
            stack = "".join(traceback.format_stack(frame)[-6:]) if frame is not None else ""
            self._log_warn(f"task {name!r} has held the thread for {stalled * 1e3:.0f} ms "
                           f"without awaiting; it blocks every robot and plugin. Now at:\n{stack}")
