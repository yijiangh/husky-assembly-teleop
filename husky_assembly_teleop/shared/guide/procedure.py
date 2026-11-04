"""
One procedure at a time (a collision check, a take, a reload): started, waiting for Go, cancelled, and how it ended.

A procedure is a coroutine run as a task. It says what it does now in `doing`, and may wait for the operator's Go
(`ask_go`), with a path to scrub first (`preview_go`); on `auto` it skips the Go it makes optional (`optional_go`). No
outcome is kept when it went well: the step's checks say so. Steps and their helpers also leave an outcome for what
they refused.
- A Go that moves robots (`acting`) takes a second press while other robots are not measured, as the planners' Commit
  (`CommitConfirm`); the move may then have those robots unknown (`unknown_allowed`). Robots the move cannot reach
  (`out_of_reach`) may be unknown without asking.
- Go's Clearance box (`clearance`, below Go) says whether what Go moves keeps a clearance; the steps read it.
- Cancel ends the procedure, except in a part run with `pausable`: there it stops that part (the arm holds) and the
  procedure goes on, e.g. to ask Go to go on; Cancel then ends it.
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING, Callable, Coroutine, Iterable

import numpy as np

from ...plugin_api.concurrency import WaitTimeout
from ...robot_interface.soft_stop import MotionRefused
from ...world.checks import BAD, PENDING, WARN, Check
from ...world.stand_ins import unknown_robots
from ..planner.planner_panel import CommitConfirm
from ..teleop.clearance_toggle import ClearanceToggle

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext
    from ..planner.timed_path import TimedPath

#: Seconds to wait for the operator's Go.
GO_TIMEOUT = 600.0
#: The outcome's label while Go waits for its second press.
GO_AGAIN = "press Go again"
#: The hover text of Go's Clearance box.
GO_CLEARANCE_HINT = ("While ticked, what Go moves soft stops once the robot comes too close to anything, and a move "
                     "starting too close is refused. Untick to move out of a tight spot.")


class Procedure:
    """The guide's one running procedure, its question for Go, and how the last one ended.

    Attributes:
        doing: What the running procedure does now; "" when none runs.
        outcome: How the last one ended, or what was last refused, as a chip, e.g. Check("take failed", BAD, "the
            arm refused the move"); None for nothing to say.
        prompt: The question it waits on for Go; "" when it does not wait.
        go_label: What Go then does, e.g. "move".
        preview_path: What the operator scrubs before Go (a planned move or a sweep), or None.
        preview: Where Preview stands along it, seconds from its start.
        auto: Whether procedures go on without asking for the Go they make optional.
        confirm: Go's second press while other robots are not measured; the panel sets its `button`.
        unknown_allowed: Ids of the other robots the last Go let be unknown during its move, for its clearance.
        clearance: Go's Clearance box: whether what Go moves keeps a clearance.
        go_margin: The clearance of what the awaited Go moves, metres, for its watch; None for the arm's usual one.
        out_of_reach: Ids of the other robots not measured that what Go moves cannot reach; the plugin sets it.
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
        self.confirm = CommitConfirm()
        self.unknown_allowed: frozenset[str] = frozenset()
        self.clearance = ClearanceToggle(GO_CLEARANCE_HINT)
        self.go_margin: float | None = None
        self.out_of_reach: Callable[[PluginContext], frozenset[str]] = lambda _ctx: frozenset()
        #: Ids of the robots the awaited Go moves; none for a Go that moves nothing.
        self._acting: frozenset[str] = frozenset()
        #: Whether a `pausable` part runs, and whether Cancel stopped it (rather than the procedure).
        self._pausable = self._paused = False

    @property
    def running(self) -> bool:
        """bool: Whether a procedure runs now."""
        return self._task is not None and not self._task.done()

    @property
    def waiting(self) -> bool:
        """bool: Whether the running procedure only waits for Go: a new one replaces it (`start`)."""
        return self.running and bool(self.prompt)

    def start(self, ctx: PluginContext, label: str, work: Coroutine, task_name: str) -> None:
        """Run `work` as the procedure. One that only waits for Go is replaced; one that works refuses it.

        A replaced one is cancelled, nothing of it having moved since it asked; e.g. Record take with another take
        chosen plans that one instead.

        Args:
            ctx: The plugin's context.
            label: What it is, for its outcome and the log, e.g. "take".
            work: The procedure's coroutine.
            task_name: The task's name, e.g. "take a200-0805 ur_arm".
        """
        replaced = None
        if self.running:
            if not self.waiting:
                work.close()
                ctx.log_info(f"{label} not started: {self.doing or 'a procedure'} runs")
                return
            replaced = self._task
            replaced.cancel()
            ctx.log_info(f"{label} replaces the procedure waiting for Go")
        self.outcome, self.prompt, self._go = None, "", False
        self.confirm.press.forget()
        self._task = ctx.spawn(task_name, self._run(ctx, label, work, replaced))

    async def _run(self, ctx: PluginContext, label: str, work: Coroutine, replaced: asyncio.Task | None = None) -> None:
        """Run a procedure and keep how it ended; first let the one it replaces end, and forget its outcome."""
        try:
            if replaced is not None:
                try:
                    await asyncio.wait({replaced})
                except asyncio.CancelledError:
                    work.close()  # * never started: closing it avoids a "never awaited" warning
                    raise
                self.outcome = None
            await work
        except MotionRefused as refused:
            self.outcome = Check(f"{label} refused", BAD, str(refused))
            ctx.log_warn(f"{label} refused: {refused}")
        except (WaitTimeout, RuntimeError, OSError, ValueError) as error:
            self.outcome = Check(f"{label} failed", BAD, str(error))
            ctx.log_warn(f"{label}: {error}")
        except asyncio.CancelledError:
            self.outcome = Check(f"{label} cancelled", PENDING, "Stopped before it finished; run it again when ready.")
            raise
        finally:
            self.doing, self.prompt = "", ""

    def go(self) -> None:
        """The procedure goes on if it waits for Go."""
        if self.prompt:
            self._go = True

    def press_go(self, ctx: PluginContext) -> None:
        """The operator pressed Go: go on; a Go that moves robots asks first while other robots are not measured."""
        if not self.prompt:
            return
        unknown = frozenset()
        if self._acting:
            asked, unknown = self.confirm.asks(ctx, self._acting, "Go", far=self.out_of_reach(ctx))
            if asked:
                self.outcome = Check(GO_AGAIN, WARN, asked)
                return
        if self.outcome is not None and self.outcome.label == GO_AGAIN:
            self.outcome = None
        self.unknown_allowed = unknown
        self.go()

    def cancel(self) -> None:
        """Cancel the running procedure, or only its `pausable` part while one moves; one waiting for Go ends."""
        if self.running:
            self._paused = self._pausable and not self.waiting
            self._task.cancel()

    async def pausable(self, work: Coroutine) -> bool:
        """Run `work` so that Cancel stops it alone and the procedure goes on; whether it finished (False: cancelled).

        ! Python 3.10: the procedure's task goes on after a swallowed cancellation; `work`'s own cleanup (holding the
          arm, giving the lease back) has run by then.
        """
        self._pausable = True
        try:
            await work
            return True
        except asyncio.CancelledError:
            if not self._paused:
                raise
            return False
        finally:
            self._pausable = self._paused = False

    async def ask_go(self, ctx: PluginContext, prompt: str, go: str = "try again", acting: Iterable[str] = ()
                     ) -> frozenset[str]:
        """Wait for the operator's Go; `go` says on the button what it does.

        Args:
            ctx: The plugin's context.
            prompt: The question.
            go: What Go does, on the button.
            acting: Ids of the robots Go moves: other robots not measured then make Go ask twice (`press_go`).

        Returns:
            frozenset[str]: Ids of the other robots Go let be unknown during the move (`unknown_allowed`).
        """
        self.prompt, self._go, self.doing, self.go_label = prompt, False, "waiting for Go", go
        self.confirm.press.forget()  # * a Go pressed for an earlier question never counts for this one
        self._acting, self.unknown_allowed = frozenset(acting), frozenset()
        await ctx.wait_until(lambda: self._go, timeout_s=GO_TIMEOUT, description="Go")
        self.prompt, self._acting = "", frozenset()
        return self.unknown_allowed

    async def preview_go(self, ctx: PluginContext, path: TimedPath, prompt: str, go: str,
                         acting: Iterable[str] = ()) -> frozenset[str]:
        """Let the operator scrub along `path` with Preview, then wait for Go (`ask_go`)."""
        self.preview_path, self.preview = path, 0.0
        try:
            return await self.ask_go(ctx, prompt, go, acting)
        finally:
            self.preview_path = None

    async def optional_go(self, ctx: PluginContext, path: TimedPath, prompt: str, go: str,
                          acting: Iterable[str]) -> frozenset[str]:
        """`preview_go`, skipped on `auto` while every robot besides `acting` is measured or out of reach.

        Returns:
            frozenset[str]: Ids of the other robots Go let be unknown during the move; skipped, those out of reach.
        """
        acting = frozenset(acting)
        unknown = unknown_robots(ctx.scene.snapshot, acting)
        if self.auto and unknown <= self.out_of_reach(ctx):
            return unknown
        return await self.preview_go(ctx, path, prompt, go, acting)

    def previewed(self) -> np.ndarray | None:
        """The previewed path's point where Preview stands; None without a preview."""
        path = self.preview_path
        return None if path is None else path.sample(self.preview)
