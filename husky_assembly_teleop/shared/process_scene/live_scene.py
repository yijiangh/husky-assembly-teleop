"""
Plan a process movement in the live scene, only once it shows that movement: `async with showing(...)`.

The process plugin puts the selected movement's bars and allowed contacts into the live scene a tick after the
selection (`ProcessContract.scene_shows`); planners plan in the live scene and know nothing of the process.

- ! Take the snapshot for the plan inside the block, after the wait: one taken before may show another movement.
- ! Every caller asking a planner for a process movement (plan, IK, servo pass, tail) goes through `showing`.
"""

from __future__ import annotations

from contextlib import asynccontextmanager
from typing import TYPE_CHECKING, AsyncIterator

from ...plugin_api.concurrency import WaitTimeout

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext
    from ..process_contract import ProcessContract

#: How long a plan waits for the live scene to show its movement, seconds (two ticks are enough when it is selected).
SCENE_WAIT = 5.0


class SceneNotShown(RuntimeError):
    """The live scene did not show the movement to plan, or stopped showing it before the plan came back."""


@asynccontextmanager
async def showing(ctx: PluginContext, process: ProcessContract, action_id: str, movement_id: str,
                  timeout_s: float = SCENE_WAIT) -> AsyncIterator[None]:
    """Wait until the live scene shows the movement; on leaving the block, refuse if it no longer does.

    Args:
        ctx: The caller's context.
        process: The process plugin.
        action_id: The action.
        movement_id: The movement selected for the plan: the run's current step's primary.
        timeout_s: How long to wait.

    Raises:
        SceneNotShown: Not shown within `timeout_s` (another movement stays selected), or another one shown when the
            block ends: whatever the block planned is not for this movement. An error in the block passes through.
    """
    key = (action_id, movement_id)
    try:
        await ctx.wait_until(lambda: process.scene_shows == key, timeout_s=timeout_s,
                             description=f"the live scene to show {movement_id}")
    except WaitTimeout as late:
        raise SceneNotShown(f"the live scene does not show {movement_id} (it shows "
                            f"{_named(process.scene_shows)}): select it, then plan again") from late
    yield
    if process.scene_shows != key:
        raise SceneNotShown(f"the live scene changed while {movement_id} was planned (it shows "
                            f"{_named(process.scene_shows)}): plan again")


def _named(shown: tuple[str, str] | None) -> str:
    """The shown movement's id, or what a tick between two movements is."""
    return shown[1] if shown is not None else "nothing yet, its bodies being put"
