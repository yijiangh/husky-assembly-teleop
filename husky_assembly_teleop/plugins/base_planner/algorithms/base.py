"""
What every base planning algorithm provides, and `Debug`, the record of a search's inner workings.

! `Algorithm.add_settings` and `settings` run on the main thread, `plan` on the planner's worker thread.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
from ament_index_python.packages import get_package_share_directory

from ..follow import XICR_RANGE
from ..world import COLLISION_MARGIN

if TYPE_CHECKING:
    import threading

    import viser

    from ...planning.search import PlanResult
    from ..path import BasePath
    from ..world import PlanningWorld


#: Largest gap between the xICR values a path is checked with, metres.
XICR_STEP = 0.025
#: The robot model the onboard follower runs with (its `model:=`), from crl_husky config/base_models.
FOLLOWER_MODEL = "alice_tiles"
#: Straight drive onto the target (and off the start, for the RRT), metres: no turn on the spot there.
LEAD = 0.5
#: Collision-checking resolution along a move: metres driven, radians turned.
POSITION_STEP = 0.05
YAW_STEP = 0.05


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


def follower_models() -> list[str]:
    """Names of the robot models in crl_husky config/base_models, "ideal" first."""
    try:
        folder = Path(get_package_share_directory("crl_husky")) / "config" / "base_models"
    except LookupError:
        return ["ideal"]
    return sorted((path.stem for path in folder.glob("*.yaml")), key=lambda name: (name != "ideal", name))


def add_follower_settings(gui: viser.GuiApi) -> tuple:
    """Add the xICR range and follower model widgets every algorithm checks its path with, in that order."""
    models = follower_models()
    return (gui.add_multi_slider("xICR m", min=-0.30, max=0.10, step=0.01, initial_value=XICR_RANGE,
                                 hint="Where the turning centre may really be, as crl_husky's xICR (negative: ahead "
                                      "of base_footprint); the path is checked for all of them"),
            gui.add_dropdown("Follower model", options=models,
                             initial_value=FOLLOWER_MODEL if FOLLOWER_MODEL in models else models[0],
                             hint="The robot model the onboard follower runs with (its model:=), from crl_husky "
                                  "config/base_models; the path is checked as that follower drives it"))


def add_margin_setting(gui: viser.GuiApi):
    """Add the margin slider: how far every algorithm keeps from everything else."""
    return gui.add_slider("Margin m", min=0.0, max=0.3, step=0.01, initial_value=COLLISION_MARGIN,
                          hint="Keep at least this far from the other robots and bodies")
