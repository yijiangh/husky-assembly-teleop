"""
Helpers for plugin code that waits without blocking.

! Never wait in a loop: the tick, ROS callbacks and PyBullet share one thread, so
  it would freeze the node. Yield instead; the monitor advances the code one step
  per tick, and between yields it has the thread to itself (no locks needed).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Callable, Iterator

if TYPE_CHECKING:
    from .context import PluginContext

#: A unit of cooperative work: a generator that yields to give up the thread.
Task = Iterator[None]


class Cancelled(Exception):
    """Thrown into a job at its current yield point, to unwind it.

    - ! Let it propagate unless you are cleaning up; catching it and carrying on defeats cancellation.
    - ! Cleanup gets only a few more ticks at shutdown, so waits in it need a short timeout.
    """


class WaitTimeout(Exception):
    """Raised by wait_until when a condition does not come true in time."""


def wait_until(
    ctx: "PluginContext",
    predicate: Callable[[], bool],
    timeout_s: float | None = None,
    description: str = "condition",
) -> Task:
    """Yield until a predicate becomes true.

    Args:
        ctx: The plugin's context, used for the clock.
        predicate: Checked once per tick on the ROS thread. Keep it cheap.
        timeout_s: Give up after this many seconds, or None to wait forever.
            ! Prefer a timeout: a wait on silent hardware otherwise hangs unnoticed.
        description: Names the wait in the timeout message.

    Yields:
        None: Once per tick until the predicate holds.

    Raises:
        WaitTimeout: If timeout_s elapses first.
    """
    deadline = None if timeout_s is None else ctx.now() + timeout_s
    while not predicate():
        if deadline is not None and ctx.now() >= deadline:
            raise WaitTimeout(f"timed out after {timeout_s}s waiting for {description}")
        yield


def wait_seconds(ctx: "PluginContext", seconds: float) -> Task:
    """Yield for a fixed duration.

    Args:
        ctx: The plugin's context, used for the clock.
        seconds: How long to wait.

    Yields:
        None: Once per tick until the time has passed.
    """
    until = ctx.now() + seconds
    while ctx.now() < until:
        yield


class Job:
    """A cancellable unit of plugin work, advanced one step per tick.

    ! Create it with PluginContext.spawn, not directly, so the plugin's jobs are cancelled with it.

    Attributes:
        label: Human-readable name, used in logs.
        done: Whether the job has finished, failed or been cancelled.
        error: What ended the job: the exception, a Cancelled if cancelled, or
            None if it finished cleanly or is still running.
    """

    def __init__(self, label: str, task: Task):
        """Wrap a generator as a job.

        Args:
            label: Human-readable name.
            task: The generator to advance.
        """
        self.label = label
        self.done = False
        self.error: BaseException | None = None
        self._task = task
        self._cancel_requested = False

    def cancel(self) -> None:
        """Ask the job to stop: Cancelled is thrown in on the next tick. Does nothing once done."""
        self._cancel_requested = True

    def step(self) -> None:
        """Advance the job one step; PluginContext calls this once per tick.

        Raises:
            RuntimeError: If the task raised. The job is marked done first, so it is not retried.
        """
        if self.done:
            return
        try:
            if self._cancel_requested:
                self._task.throw(Cancelled(f"job {self.label!r} cancelled"))
            else:
                next(self._task)
        except StopIteration:
            # Finished, or cleanup swallowed the cancellation and returned.
            self.done = True
        except Cancelled as cancelled:
            # Cancelling is not a failure, so it is not re-raised.
            self.done = True
            self.error = cancelled
        except Exception as failure:
            self.done = True
            self.error = failure
            raise RuntimeError(f"job {self.label!r} failed") from failure
