"""
What the obstacles plugin offers others: the lab as it stands in the scene (`Lab`), read through `ObstaclesContract`.

Users type `ctx.require("obstacles")` with it: `obstacles: ObstaclesContract = ctx.require("obstacles")`.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol

if TYPE_CHECKING:
    from ..world.lab import Lab


class ObstaclesContract(Protocol):
    """What the obstacles plugin offers other plugins: type `ctx.require("obstacles")` as this."""

    @property
    def lab(self) -> Lab:
        """Lab: The lab's obstacles and border as in the scene now, edits included; replaced on every change.

        `border` is None while the lab has none. Temporary obstacles are not in it; they are only in the scene.
        """
