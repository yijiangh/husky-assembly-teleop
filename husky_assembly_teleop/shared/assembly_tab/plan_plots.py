"""
The Plan plots window's plots: every joint along the preview, and the largest joint step between plan rows.

Built once, empty; `show` fills them from a plan or empties them. Joints are labelled short, "L pan" … "R w3", each
joint in its own colour, the right arm's dashed. With `grasp`, two more for a path with a bar held in two tools: how
far one tool0 seen from the other moves from where it is at the first waypoint, in position and per axis.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, NamedTuple

import numpy as np
import viser
import viser.uplot
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES

from ..assembly_planning.joints import arm_side
from ...ui.trace_plot import BLANK, COLORS, thinned

if TYPE_CHECKING:
    from scaffolding_assembly_core.process.solutions import Trajectory

    from ..planner.timed_path import TimedPath

#: Short names of the UR joints, in `UR_JOINT_NAMES` order, for the legend.
SHORT_NAMES = dict(zip(UR_JOINT_NAMES, ("pan", "lift", "elbow", "w1", "w2", "w3")))
#: Each plot's height, px.
PLOT_HEIGHT = 180


class GraspDrift(NamedTuple):
    """How far one tool0 seen from the other moves along a path, from where it is at the first waypoint.

    Attributes:
        waypoints: Where each sample is along the path, in waypoints; a half is between two.
        mm: The distance, millimetres (N).
        deg: The angle of its x, y and z axis to where each pointed, degrees (N x 3).
    """

    waypoints: np.ndarray
    mm: np.ndarray
    deg: np.ndarray


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
    """The joint values along the preview, the joint step between the plan's rows, the held grasp; main thread."""

    def __init__(self, gui: viser.GuiApi, grasp: bool = False):
        """Add the plots to `gui`, empty.

        Args:
            gui: Where to add them, e.g. the window's tab.
            grasp: Also add the two plots of the tools' relative pose.
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
        #: The tools' relative pose against the first waypoint's: its position, and its three axes; None without.
        self._grasp_position = self._grasp_axes = None
        if grasp:
            waypoint = viser.uplot.Series(label="waypoint")
            self._grasp_position = gui.add_uplot(
                data=BLANK, series=(waypoint, viser.uplot.Series(label="|Δp|", stroke=COLORS[0], width=1.5)),
                title="tools' relative pose vs first waypoint: position (mm)", **options)
            self._grasp_axes = gui.add_uplot(
                data=(BLANK[0], *(BLANK[1] for _ in "xyz")),
                series=(waypoint, *(viser.uplot.Series(label=f"{axis} axis", stroke=COLORS[index], width=1.5)
                                    for index, axis in enumerate("xyz"))),
                title="tools' relative pose vs first waypoint: axis angles (deg)", **options)

    def show(self, path: TimedPath | None, trajectory: Trajectory | None, drift: GraspDrift | None = None) -> None:
        """Fill the plots from a plan, or empty them.

        Args:
            path: The preview as it will run, its columns in `trajectory.joint_names` order; None empties the plots.
            trajectory: The plan as planned, radians; None empties the plots.
            drift: The tools' relative pose along the plan, for a bar held in two tools; None empties its two plots.
        """
        if self._grasp_position is not None:
            if drift is None or path is None or trajectory is None:
                self._grasp_position.data = BLANK
                self._grasp_axes.data = (BLANK[0], *(BLANK[1] for _ in "xyz"))
            else:
                self._grasp_position.data = thinned(drift.waypoints, drift.mm)
                self._grasp_axes.data = thinned(drift.waypoints, *drift.deg.T)
        if path is None or trajectory is None:
            self._values.data = (BLANK[0], *(BLANK[1] for _ in self._series[1:]))
            self._steps.data = BLANK
            return
        series = (self._series[0], *(_line(index, joint_label(name))
                                     for index, name in enumerate(trajectory.joint_names)))
        if series != self._series:
            self._series = self._values.series = series
        self._values.data = thinned(np.asarray(path.times, dtype=float), *np.degrees(path.points).T)
        rows = np.asarray(trajectory.positions, dtype=float)
        steps = np.degrees(np.abs(np.diff(rows, axis=0)).max(axis=1))
        self._steps.data = thinned(np.arange(1, len(rows), dtype=float), steps)
