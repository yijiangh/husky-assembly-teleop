"""
Tooling for writing plugin code that waits without blocking.

? Why generators.
  The tick, every ROS callback, all planning and all PyBullet share one thread,
  so a plugin that waits in a loop freezes the node -- including the
  subscriptions that would have told it the wait was over. Plugin code yields
  instead, and the monitor advances it one step per tick. Between two yields a
  plugin has the thread to itself and needs no locks.

  The payoff is that the logic stays in the order it happens:

      def _execute(self, ctx):
          robot = ctx.world.robots["a200-0806"]
          robot.switch_controller("ur_arm", "scaled_joint_trajectory_controller")
          yield from wait_until(ctx, lambda: robot.state.active_controllers.get("ur_arm")
                                             == "scaled_joint_trajectory_controller",
                                timeout_s=5.0, description="controller switch")
          robot.send_joint_trajectory("ur_arm", self.trajectory, duration=8.0)
          yield from wait_until(ctx, lambda: not robot.state.is_executing["ur_arm"],
                                timeout_s=30.0, description="trajectory execution")
          self.log_result()
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Callable, Iterator

if TYPE_CHECKING:
    from .context import PluginContext

#: A unit of cooperative work: a generator that yields to give up the thread.
Task = Iterator[None]


class Cancelled(Exception):
    """Thrown into a job at its current yield point, to unwind it.

    ! Let it propagate unless you have cleanup to do.
      try/finally and with blocks around that yield run as normal. Catching it
      to release a gripper is right; catching it and carrying on defeats
      cancellation.

    ! Cleanup gets only a few more ticks.
      A torn-down plugin's jobs are pumped a bounded number of times and then
      dropped, so cleanup that waits on hardware needs a short timeout.
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
        predicate: Checked once per tick on the ROS thread, so it may read world
            state, cell state and the scene freely. Keep it cheap and pure.
        timeout_s: Give up after this many seconds, or None to wait forever.
            ! Prefer a timeout. A job waiting forever on hardware that never
            answers is invisible; a WaitTimeout names the wait that failed.
        description: What is being waited for, for the timeout message.

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

    Created by PluginContext.spawn, never directly. A job belongs to exactly one
    plugin: disabling or tearing down that plugin cancels its jobs.

    Attributes:
        label: Human-readable name, used in logs and error messages.
        done: Whether the job has finished, failed or been cancelled.
        error: What ended the job, or None if it finished cleanly or is still
            running; a Cancelled instance if it was cancelled. Plugins holding a
            Job handle read this to find out how their work ended.
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
        """Ask the job to stop at its next step.

        Cooperative, so it takes effect on the next tick: Cancelled is thrown in
        at the current yield point and the job's cleanup runs. Calling this on a
        finished job does nothing.
        """
        self._cancel_requested = True

    def step(self) -> None:
        """Advance the job by one step. Called by the monitor, once per tick.

        Raises:
            RuntimeError: If the task raised. The job is marked done first, so it
                is dropped rather than retried. Reporting is left to the
                monitor's `_guard`, the one place that decides what a plugin
                failure costs; the wrapper keeps the label in the message.
        """
        if self.done:
            return
        try:
            if self._cancel_requested:
                self._task.throw(Cancelled(f"job {self.label!r} cancelled"))
            else:
                next(self._task)
        except StopIteration:
            # Ran to completion, or its cleanup swallowed the cancellation and
            # returned. Either way it is finished.
            self.done = True
        except Cancelled as cancelled:
            # Being cancelled is not a failure, so it is not reported as one.
            self.done = True
            self.error = cancelled
        except Exception as failure:
            self.done = True
            self.error = failure
            raise RuntimeError(f"job {self.label!r} failed") from failure
