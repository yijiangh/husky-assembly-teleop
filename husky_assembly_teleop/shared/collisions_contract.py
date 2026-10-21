"""
What the collisions plugin offers others: a panel listing collision hits per source, drawing the ticked ones.

Users type `ctx.require("collisions")` with it: `collisions: CollisionsContract = ctx.require("collisions")`.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Collection, Mapping, Protocol

if TYPE_CHECKING:
    from ..world.collision_hit import CollisionHit


class CollisionsContract(Protocol):
    """What the collisions plugin offers other plugins: type `ctx.require("collisions")` as this."""

    def replace(self, source: str, hits: Mapping[str, CollisionHit], shown: Collection[str] = ()) -> None:
        """Make these a source's hits, dropping its earlier ones. Main thread.

        Args:
            source: Who reports, e.g. the plugin's name; one group in the panel.
            hits: By label, e.g. "take 3". Empty: the source shows that nothing collides.
            shown: Labels to tick (draw) right away; the operator ticks the others.
        """

    def clear(self, source: str) -> None:
        """Drop a source and its hits from the panel. Main thread."""
