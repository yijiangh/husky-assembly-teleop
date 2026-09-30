"""
A "PyBullet window" checkbox for a plugin's own PyBullet mirror, for debugging:
ticking it shows the mirror's world in PyBullet's own window.

* Build one in `setup`, inside `ctx.view.ui()`. Hand `show` the result of every
  check or search, so the box unticks when the window was closed by hand.

! The mirror belongs to one worker thread, so the window is opened and closed
  there: the checkbox queues `set_gui` on the plugin's own executor.
* `set_gui` also gets the scene snapshot of the tick the box was ticked in, so a
  planner can fill its world before it has ever planned.
"""

from __future__ import annotations

import asyncio
from concurrent.futures import Executor
from typing import Callable

import viser

from ..plugin_api.context import PluginContext
from ..world.scene import SceneSnapshot


class PyBulletWindowToggle:
    """The checkbox, and what it last heard from the mirror. Main thread only."""

    def __init__(self, ctx: PluginContext, gui: viser.GuiApi, executor: Executor,
                 set_gui: Callable[[bool, SceneSnapshot], tuple[bool, str]], report: Callable[[str], None]):
        """Add the checkbox.

        Args:
            ctx: The owning plugin's context.
            gui: GUI api to add the checkbox to.
            executor: The one worker thread that owns the mirror.
            set_gui: Opens or closes the window; runs on `executor`. Gets whether it
                should be open and the snapshot of the tick the box was changed in.
                Returns whether it is open now and why not, e.g. `PyBulletMirror.set_gui`
                and `window_problem`.
            report: Called on the main thread with the reason when the window asked
                for isn't open (another is open, no display, closed by hand), and
                with "" once it does open, to clear an old reason.
        """
        self._ctx = ctx
        self._executor = executor
        self._set_gui = set_gui
        self._report = report
        self._checkbox = gui.add_checkbox("PyBullet window", False,
                                          hint="Show this plugin's PyBullet world in PyBullet's own window")
        self._checkbox.on_update(ctx.defer_value("pybullet window", self._toggled))

    def _toggled(self, wanted: object) -> None:
        """Open or close the window on the worker (intent). A no-op if it already is so."""
        self._ctx.spawn("pybullet window", self._apply(bool(wanted)))

    async def _apply(self, wanted: bool) -> None:
        """Run `set_gui` on the worker, after any check or search still running there.

        Args:
            wanted: Whether the window should be open.
        """
        # * Taken here, on the main thread; the worker only reads it.
        snapshot = self._ctx.scene.snapshot
        is_open, problem = await asyncio.get_running_loop().run_in_executor(self._executor, self._set_gui,
                                                                            wanted, snapshot)
        if wanted and is_open:
            self._report("")
        self.show(is_open, problem)

    def show(self, is_open: bool, problem: str) -> None:
        """Take the window's state after a check or search; untick the box if it isn't open.

        Args:
            is_open: Whether the mirror has a window now.
            problem: Why the window asked for isn't open, or "".
        """
        if self._checkbox.value and not is_open:
            self._report(f"no PyBullet window: {problem}" if problem else "no PyBullet window")
            # ? Fires the checkbox callback once more, which asks the worker to close: nothing to do.
            self._checkbox.value = False
