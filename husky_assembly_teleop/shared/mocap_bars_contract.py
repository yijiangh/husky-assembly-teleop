"""
What the mocap_bars plugin offers other plugins: every bar's latest MoCap fit, and a fresh fit on demand.

Type `ctx.require("mocap_bars")` as `MocapBarsContract`. A `BarMeasurement` is one bar as the raw markers near where
it should be last showed it: the fit (ok or not), the markers it came from and, when the process places the bar this
tick, the deviation from that planned pose.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Mapping, Protocol

import numpy as np

from scaffolding_assembly_core.geometry import Pose

if TYPE_CHECKING:
    from ..world.marker_match import BarFit, Sensed
    from .bars.layouts import BarLayout
    from .bars.pose import PoseDeviation

#: The side panel tab the mocap plugins' tabs go in: `tab_path = (MOCAP_TAB, label)`.
MOCAP_TAB = "Mocap"


@dataclass(frozen=True)
class BarMeasurement:
    """One bar as the raw markers near where it should be last showed it.

    Attributes:
        bar: The bar's body id, e.g. "bars/B20".
        layout: Its designed markers.
        fit: The fit, ok or not (`BarFit.ok`, `reason`).
        pose: The fitted bar pose (the process frame: origin at the bar's middle), None unless the fit is ok.
        sensed: The markers near the bar the fit came from, or None without a sample.
        source: Where the markers came from ("cloud", "fake"), "" without a sample.
        measured_at: ROS time the cameras captured the markers.
        tracked: The fit came from the quick match around the last pose, not the full search.
        planned: Where the process places the bar this tick (its assembly pose, or on the robot's flange), if known.
        deviation: The fit against `planned`, at the bar's middle; None without both.
    """

    bar: str
    layout: BarLayout
    fit: BarFit
    pose: Pose | None
    sensed: Sensed | None
    source: str
    measured_at: float
    tracked: bool
    planned: Pose | None
    deviation: PoseDeviation | None

    def to_json(self) -> dict:
        """The measurement as plain lists, dicts and numbers; poses as [x, y, z, qx, qy, qz, qw]."""
        sensed = self.sensed
        return {
            "bar": self.bar, "fit": self.fit.to_json(),
            "pose": None if self.pose is None else [*map(float, self.pose.position),
                                                    *map(float, self.pose.orientation)],
            "sensed": None if sensed is None else {
                "points": np.asarray(sensed.points).tolist(), "ids": list(sensed.ids),
                "residuals": None if sensed.residuals is None else np.asarray(sensed.residuals).tolist(),
                "age": sensed.age, "source": sensed.source, "dropped": sensed.dropped, "occluded": sensed.occluded,
                "notes": list(sensed.notes)},
            "source": self.source, "measured_at": self.measured_at, "tracked": self.tracked,
            "planned": None if self.planned is None else [*map(float, self.planned.position),
                                                          *map(float, self.planned.orientation)],
            "deviation": None if self.deviation is None else self.deviation.to_json(),
            "layout": {"names": list(self.layout.names), "points": np.asarray(self.layout.points).tolist(),
                       "length": self.layout.length},
        }


class MocapBarsContract(Protocol):
    """What the mocap_bars plugin offers: type `ctx.require("mocap_bars")` as this. Main thread."""

    @property
    def fits(self) -> Mapping[str, BarMeasurement]:
        """Mapping[str, BarMeasurement]: Bar id -> its latest measurement, every bar that had a sample."""

    def measure_now(self, hints: Mapping[str, Pose] | None = None) -> dict[str, BarMeasurement]:
        """Fit every bar afresh from the newest marker cloud, and keep the result as `fits`.

        Args:
            hints: Bar id -> a pose to match around first (~1 ms); a bar without one is matched around where it
                should be, then gets the full search over its nearby markers (~20 ms).

        Returns:
            dict[str, BarMeasurement]: Bar id -> its measurement, every bar; empty before the first cloud.
        """
