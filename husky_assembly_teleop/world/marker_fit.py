"""
A bar's marker layout from the process, and how far a fitted bar sits from its planned pose.

`bar_markers` collects the markers of the connector halves mounted on each bar, in the bar's frame: the layout
`marker_layouts.fit_bar` matches sensed markers to. `bar_deviation` splits a fitted pose's offset from the planned
one into across, along and tilt.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from scaffolding_assembly_core.geometry import Pose, compose, invert

if TYPE_CHECKING:
    from scaffolding_assembly_core.process import Process


def bar_markers(process: Process) -> dict[str, tuple[tuple[str, ...], np.ndarray]]:
    """Each bar's mocap markers, from the halves mounted on it, in the bar's frame; bars with fewer than 3 left out.

    Args:
        process: The process; `BodySpec.markers` of each half, its `mount` bar and both assembly poses.

    Returns:
        dict[str, tuple[tuple[str, ...], np.ndarray]]: Bar id -> (marker names "<half id>/<label>", positions in
            metres, shape (m, 3)), in a fixed order.
    """
    names: dict[str, list[str]] = {}
    points: dict[str, list[np.ndarray]] = {}
    for half_id in sorted(process.bodies):
        half = process.bodies[half_id]
        if not half.markers or half.mount not in process.bodies:
            continue
        # * Mounted halves stay where they sit on their bar, so the assembly poses give the offset.
        bar_from_half = compose(invert(process.bodies[half.mount].assembly_pose), half.assembly_pose).matrix()
        for label in sorted(half.markers):
            names.setdefault(half.mount, []).append(f"{half_id}/{label}")
            points.setdefault(half.mount, []).append(bar_from_half[:3, :3] @ half.markers[label] + bar_from_half[:3, 3])
    return {bar: (tuple(names[bar]), np.array(points[bar])) for bar in sorted(names) if len(names[bar]) >= 3}


@dataclass(frozen=True)
class BarDeviation:
    """How far a fitted bar is from where it should be, in the planned bar's frame (its axis is z).

    ? Roll about the bar axis is left out: markers a few centimetres off the axis leave it uncertain.

    Attributes:
        across: Distance of the fitted origin from the planned axis, metres.
        along: Offset of the fitted origin along the planned axis, metres.
        tilt: Angle between the fitted and the planned axis, radians.
    """

    across: float
    along: float
    tilt: float


def bar_deviation(fitted: Pose, planned: Pose) -> BarDeviation:
    """The fitted bar's offset and tilt from the planned bar."""
    offset = invert(planned).matrix()[:3, :3] @ (np.asarray(fitted.position) - np.asarray(planned.position))
    fitted_axis, planned_axis = fitted.matrix()[:3, 2], planned.matrix()[:3, 2]
    tilt = float(np.arccos(np.clip(fitted_axis @ planned_axis, -1.0, 1.0)))
    return BarDeviation(across=float(np.hypot(offset[0], offset[1])), along=float(offset[2]), tilt=tilt)
