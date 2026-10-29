"""
One procedure at a time (a collision check, a take, a reload): started, waiting for Go, cancelled, and how it ended.

A procedure is a coroutine run as a task. It says what it does now in `doing`, and may wait for the operator's Go
(`ask_go`), with a path to scrub first (`preview_go`); on `auto` it skips the Go it makes optional. No outcome is
kept when it went well: the step's checks say so. Steps and their helpers also leave an outcome for what they refused.
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING, Coroutine

import numpy as np

from ...plugin_api.concurrency import WaitTimeout
from ...robot_interface.soft_stop import MotionRefused
from ...world.checks import BAD, WARN, Check

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext
    from ..planner.timed_path import TimedPath

#: Seconds to wait for the operator's Go.
GO_TIMEOUT = 600.0


class Procedure:
    """The guide's one running procedure, its question for Go, and how the last one ended.

    Attributes:
        doing: What the running procedure does now; "" when none runs.
        outcome: How the last one ended, or what was last refused, as a chip, e.g. Check("take failed", BAD, "the
            arm refused the move"); None for nothing to say.
        prompt: The question it waits on for Go; "" when it does not wait.
        go_label: What Go then does, e.g. "move".
        preview_path: What the operator scrubs before Go (a planned move or a sweep), or None.
        preview: Where Preview stands along it, 0 to 1.
        auto: Whether procedures go on without asking for the Go they make optional.
    """

    def __init__(self):
        """Nothing runs."""
        self._task: asyncio.Task | None = None
        self._go = False
        self.doing = ""
        self.outcome: Check | None = None
        self.prompt = ""
        self.go_label = ""
        self.preview_path: TimedPath | None = None
        self.preview = 0.0
        self.auto = False

    @property
    def running(self) -> bool:
        """bool: Whether a procedure runs now."""
        return self._task is not None and not self._task.done()

    def start(self, ctx: PluginContext, label: str, work: Coroutine, task_name: str) -> None:
        """Run `work` as the procedure, unless one runs already: then say so and drop it.

        Args:
            ctx: The plugin's context.
            label: What it is, for its outcome and the log, e.g. "take".
            work: The procedure's coroutine.
            task_name: The task's name, e.g. "take a200-0805 ur_arm".
        """
        if self.running:
            work.close()
            ctx.log_info(f"{label} not started: {self.doing or 'a procedure'} runs")
            return
        self.outcome, self.prompt, self._go = None, "", False
        self._task = ctx.spawn(task_name, self._run(ctx, label, work))

    async def _run(self, ctx: PluginContext, label: str, work: Coroutine) -> None:
        """Run a procedure and keep how it ended."""
        try:
            await work
        except MotionRefused as refused:
            self.outcome = Check(f"{label} refused", BAD, str(refused))
        except (WaitTimeout, RuntimeError, OSError, ValueError) as error:
            self.outcome = Check(f"{label} failed", BAD, str(error))
            ctx.log_warn(f"{label}: {error}")
        except asyncio.CancelledError:
            self.outcome = Check(f"{label} cancelled", WARN)
            raise
        finally:
            self.doing, self.prompt = "", ""

    def go(self) -> None:
        """The operator pressed Go: the procedure goes on if it waits."""
        if self.prompt:
            self._go = True

    def cancel(self) -> None:
        """Cancel the running procedure."""
        if self.running:
            self._task.cancel()

    async def ask_go(self, ctx: PluginContext, prompt: str, go: str = "try again") -> None:
        """Wait for the operator's Go; `go` says on the button what it does."""
        self.prompt, self._go, self.doing, self.go_label = prompt, False, "waiting for Go", go
        await ctx.wait_until(lambda: self._go, timeout_s=GO_TIMEOUT, description="Go")
        self.prompt = ""

    async def preview_go(self, ctx: PluginContext, path: TimedPath, prompt: str, go: str) -> None:
        """Let the operator scrub along `path` with Preview, then wait for Go."""
        self.preview_path, self.preview = path, 0.0
        try:
            await self.ask_go(ctx, prompt, go)
        finally:
            self.preview_path = None

    def previewed(self) -> np.ndarray | None:
        """The previewed path's point where Preview stands; None without a preview."""
        path = self.preview_path
        return None if path is None else path.sample(self.preview * path.duration)
