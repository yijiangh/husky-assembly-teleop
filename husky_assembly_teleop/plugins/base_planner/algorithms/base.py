"""
What every base planning algorithm provides, and `Debug`, the record of a search's inner workings.

! `Algorithm.add_settings` and `settings` run on the main thread, `plan` on the planner's worker thread.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    import threading

    import viser

    from ...planning.search import PlanResult
    from ..path import BasePath
    from ..world import PlanningWorld


@dataclass
class Layer:
    """One kind of thing a search draws on the floor, in one colour.

    Attributes:
        color: (r, g, b).
        segments: Line segments, ((x0, y0), (x1, y1)) each, metres.
        points: Points, (x, y) each, metres.
    """

    color: tuple[int, int, int]
    segments: list = field(default_factory=list)
    points: list = field(default_factory=list)


class Debug:
    """What a search shows of its inner workings: floor drawings in named layers, and lines of text.

    - Off, drawing does nothing, so algorithms call it freely; notes are kept (cheap, and saved with recorded plans).
    - ! Filled on the worker during one search; read it on the main thread only after the search returned.
    """

    def __init__(self, on: bool = False):
        """Start empty; record only if `on`."""
        self.on = on
        #: Layers by name, in the order declared or first drawn to.
        self.layers: dict[str, Layer] = {}
        self.notes: list[str] = []

    def layer(self, name: str, color: tuple[int, int, int]) -> None:
        """Declare layer `name` before anything is drawn into it; layers draw and list in the order declared."""
        if self.on:
            self._layer(name, color)

    def path(self, name: str, poses, color: tuple[int, int, int]) -> None:
        """Draw a path through some (x, y, ...) poses into layer `name`."""
        if self.on and len(poses) > 1:
            xy = np.asarray(poses, dtype=float)[:, :2]
            self.lines(name, np.stack([xy[:-1], xy[1:]], axis=1), color)

    def lines(self, name: str, segments, color: tuple[int, int, int]) -> None:
        """Draw line segments, ((x0, y0), (x1, y1)) each, into layer `name`."""
        if self.on:
            self._layer(name, color).segments.extend(np.asarray(segments, dtype=float).reshape(-1, 2, 2))

    def points(self, name: str, xy, color: tuple[int, int, int]) -> None:
        """Draw points, (x, y, ...) each, into layer `name`."""
        if self.on:
            layer = self._layer(name, color)
            if len(xy):
                layer.points.extend(np.asarray(xy, dtype=float).reshape(len(xy), -1)[:, :2])

    def note(self, text: str) -> None:
        """Add a line of text, e.g. how long a phase took; kept even when off, for recorded plans."""
        self.notes.append(text)

    def _layer(self, name: str, color: tuple[int, int, int]) -> Layer:
        """Layer `name`, made in `color` on first use."""
        return self.layers.setdefault(name, Layer(color))


class Algorithm:
    """One way to plan a base path: its settings in the panel, and its search. Add subclasses to ALGORITHMS."""

    #: Name in the panel's Algorithm dropdown.
    label = ""

    def add_settings(self, gui: viser.GuiApi) -> None:
        """Add the settings' widgets; the panel shows them only while this algorithm is chosen. Main thread."""

    def settings(self) -> Any:
        """The settings now, as plain values for one search; the defaults before `add_settings`. Main thread."""
        return None

    def plan(self, world: PlanningWorld, own: str, start, goal, abort: threading.Event, settings: Any,
             debug: Debug) -> PlanResult[BasePath]:
        """Plan a collision-free path from start to goal. Worker thread.

        Args:
            world: The planning world, already synced.
            own: The robot to plan for, by id.
            start: (x, y, yaw) where it is.
            goal: (x, y, yaw) where it should end up.
            abort: Set from the main thread to end the search early.
            settings: From `settings`, taken when planning started.
            debug: Where to record the search's inner workings.

        Returns:
            PlanResult[BasePath]: The path, or why there is none.
        """
        raise NotImplementedError
