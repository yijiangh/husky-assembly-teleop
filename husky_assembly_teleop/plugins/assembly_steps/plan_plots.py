"""
The plan's two plots in the Assembly tab: every joint along the preview, and the largest joint step between plan rows.

Built once, empty; `show` fills them from a plan or empties them. Joints are labelled short, "L pan" … "R w3", each
joint in its own colour, the right arm's dashed.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import viser
import viser.uplot
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES

from ...shared.assembly_planning.joints import arm_side
from ...ui.trace_plot import COLORS, MAX_POINTS

if TYPE_CHECKING:
    from scaffolding_assembly_core.process.solutions import Trajectory

    from ...shared.planner.timed_path import TimedPath

#: Short names of the UR joints, in `UR_JOINT_NAMES` order, for the legend.
SHORT_NAMES = dict(zip(UR_JOINT_NAMES, ("pan", "lift", "elbow", "w1", "w2", "w3")))
#: Each plot's height, px.
PLOT_HEIGHT = 180
#: An "empty" plot's data: one blank point.
#: ! Never zero rows: a uPlot that viser 1.1.1 built, or was sent, with no rows never draws later data in an open page.
BLANK = (np.zeros(1), np.full(1, np.nan))


def joint_label(name: str) -> str:
    """A joint's short label: "L pan" for "left_ur_arm_shoulder_pan_joint", "pan" for a one-arm robot's joint.

    Args:
        name: The URDF joint name.

    Returns:
        str: The label; the name itself when it is no UR joint.
    """
    for joint, short in SHORT_NAMES.items():
        if name.endswith(f"_{joint}"):
            side = arm_side(name[:-len(joint) - 1])
            return short if side == "arm" else f"{side[0].upper()} {short}"
    return name


def _thinned(*columns: np.ndarray) -> tuple[np.ndarray, ...]:
    """The columns cut to at most MAX_POINTS rows, picked evenly: viser sends the whole plot on each change.

    Args:
        columns: Equally long columns, x first.

    Returns:
        tuple[np.ndarray, ...]: The same columns, at most MAX_POINTS rows each.
    """
    if len(columns[0]) <= MAX_POINTS:
        return columns
    pick = np.linspace(0, len(columns[0]) - 1, MAX_POINTS).astype(int)
    return tuple(column[pick] for column in columns)


def _line(index: int, label: str) -> viser.uplot.Series:
    """One joint's line: coloured by its place in the arm, dashed for the right arm.

    Args:
        index: The joint's column in the plan.
        label: Its short label (`joint_label`).

    Returns:
        viser.uplot.Series: The line's look.
    """
    line = viser.uplot.Series(label=label, stroke=COLORS[index % len(UR_JOINT_NAMES)], width=1.5)
    if label.startswith("R "):
        line["dash"] = (4.0, 3.0)
    return line


class PlanPlots:
    """The joint values along the preview, and the joint step between the plan's rows; main thread."""

    def __init__(self, gui: viser.GuiApi):
        """Add both plots to `gui`, empty.

        Args:
            gui: Where to add them, e.g. a band of the Assembly tab.
        """
        # * x in plain numbers, not times of day; the legend shows the labels only (12 lines in a narrow panel).
        options = {"scales": {"x": viser.uplot.Scale(time=False)}, "legend": viser.uplot.Legend(live=False),
                   "height": PLOT_HEIGHT}
        #: The joint values plot's series as last sent: the x axis, then one line per joint.
        self._series = (viser.uplot.Series(label="t s"), viser.uplot.Series(label="joints"))
        self._values = gui.add_uplot(data=BLANK, series=self._series,
                                     title="joint values (deg)", **options)
        self._steps = gui.add_uplot(data=BLANK,
                                    series=(viser.uplot.Series(label="row"),
                                            viser.uplot.Series(label="max |Δq|", stroke=COLORS[0], width=1.5)),
                                    title="joint step (deg)", **options)

    def show(self, path: TimedPath | None, trajectory: Trajectory | None) -> None:
        """Fill both plots from a plan, or empty them.

        Args:
            path: The preview as it will run, its columns in `trajectory.joint_names` order; None empties the plots.
            trajectory: The plan as planned, radians; None empties the plots.
        """
        if path is None or trajectory is None:
            self._values.data = (BLANK[0], *(BLANK[1] for _ in self._series[1:]))
            self._steps.data = BLANK
            return
        series = (self._series[0], *(_line(index, joint_label(name))
                                     for index, name in enumerate(trajectory.joint_names)))
        if series != self._series:
            self._series = self._values.series = series
        self._values.data = _thinned(np.asarray(path.times, dtype=float), *np.degrees(path.points).T)
        rows = np.asarray(trajectory.positions, dtype=float)
        steps = np.degrees(np.abs(np.diff(rows, axis=0)).max(axis=1))
        self._steps.data = _thinned(np.arange(1, len(rows), dtype=float), steps)
