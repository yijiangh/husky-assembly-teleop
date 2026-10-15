"""
Fit a known marker layout, e.g. a bar's CAD markers, to raw mocap markers without Motive's labels.

Which model marker each measured marker is comes from pairwise distances, so the measured markers may be any
subset of the model's. Of all matchings that keep the distances, the one with the smallest fit error wins.
A bar's layout comes from the process: the markers of the connector halves mounted on it (`bar_markers`).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from pybullet_planning.utils.transformations import superimposition_matrix

from scaffolding_assembly_core.geometry import Pose, compose, invert

if TYPE_CHECKING:
    from scaffolding_assembly_core.process import Process

#: Two pairwise distances match when they differ by less than this, metres: CAD and marker placement tolerance.
DISTANCE_TOLERANCE = 0.01


@dataclass(frozen=True)
class MarkerFit:
    """A rigid fit of model markers onto measured ones.

    Attributes:
        world_from_model: 4x4 transform from the model frame to the world frame.
        model_index: Model marker matched to each measured marker, shape (n,).
        residuals: Distance of each measured marker to its fitted model marker, metres, shape (n,).
    """

    world_from_model: np.ndarray
    model_index: np.ndarray
    residuals: np.ndarray

    @property
    def rms(self) -> float:
        """float: Root mean square of the residuals, metres."""
        return float(np.sqrt(np.mean(self.residuals ** 2)))

    @property
    def pose(self) -> Pose:
        """Pose: The model frame in the world, e.g. the fitted bar's pose."""
        return Pose.from_matrix(self.world_from_model)

    def transform(self, points: np.ndarray) -> np.ndarray:
        """Model-frame points, shape (k, 3), in the world frame."""
        return points @ self.world_from_model[:3, :3].T + self.world_from_model[:3, 3]


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


def fit_markers(model: np.ndarray, measured: np.ndarray, tolerance: float = DISTANCE_TOLERANCE) -> MarkerFit | None:
    """The best rigid fit of `model` onto `measured`, or None with fewer than 3 measured markers or no matching.

    Args:
        model: Model markers, shape (m, 3).
        measured: Measured markers, shape (n, 3), n <= m; each must be one of the model's.
        tolerance: See DISTANCE_TOLERANCE.
    """
    if len(measured) < 3 or len(measured) > len(model):
        return None
    model_distances = np.linalg.norm(model[:, None] - model[None], axis=-1)
    measured_distances = np.linalg.norm(measured[:, None] - measured[None], axis=-1)
    best: MarkerFit | None = None

    def extend(matched: list[int]) -> None:
        nonlocal best
        i = len(matched)
        if i == len(measured):
            fit = _fit(model, measured, np.array(matched))
            if best is None or fit.rms < best.rms:
                best = fit
            return
        for j in range(len(model)):
            if j not in matched and all(abs(model_distances[j, k] - measured_distances[i, a]) < tolerance
                                        for a, k in enumerate(matched)):
                extend(matched + [j])

    extend([])
    return best


def _fit(model: np.ndarray, measured: np.ndarray, model_index: np.ndarray) -> MarkerFit:
    """The least-squares rigid fit for one matching; rotations only, never mirrored."""
    world_from_model = superimposition_matrix(model[model_index].T, measured.T)
    fit = MarkerFit(world_from_model, model_index, np.zeros(len(measured)))
    residuals = np.linalg.norm(fit.transform(model[model_index]) - measured, axis=1)
    return MarkerFit(world_from_model, model_index, residuals)


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
