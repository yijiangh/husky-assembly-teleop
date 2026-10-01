"""
A "PyBullet window" checkbox that shows a plugin's PyBullet mirror in PyBullet's own window, for debugging.

Add it in `setup`, inside `ctx.view.ui()`. If the window can't open, it logs why and unticks itself.
"""

from __future__ import annotations

import asyncio
from concurrent.futures import Executor
from typing import Callable

import viser

from ..plugin_api.context import PluginContext
from ..world.scene import SceneSnapshot


def add_pybullet_window_toggle(ctx: PluginContext, gui: viser.GuiApi, executor: Executor,
                               set_gui: Callable[[bool, SceneSnapshot], None]) -> None:
    """Add the checkbox.

    Args:
        ctx: The owning plugin's context.
        gui: GUI api to add the checkbox to.
        executor: The one worker thread that owns the mirror; `set_gui` runs there.
        set_gui: Opens or closes the window and fills the world from the snapshot; raises if it can't.
    """
    checkbox = gui.add_checkbox("PyBullet window", False,
                                hint="Show this plugin's PyBullet world in PyBullet's own window")

    async def apply(wanted: bool) -> None:
        snapshot = ctx.scene.snapshot  # main thread
        try:
            await asyncio.get_running_loop().run_in_executor(executor, set_gui, wanted, snapshot)
        except Exception as error:
            ctx.log_warn(f"no PyBullet window: {error}")
            checkbox.value = False  # ? fires again, asking the worker to close: harmless

    checkbox.on_update(ctx.defer_value("pybullet window",
                                       lambda wanted: ctx.spawn("pybullet window", apply(bool(wanted)))))
