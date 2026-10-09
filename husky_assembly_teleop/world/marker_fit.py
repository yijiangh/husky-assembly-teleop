"""
Fit a known marker layout, e.g. a bar's CAD markers, to raw mocap markers without Motive's labels.

Which model marker each measured marker is comes from pairwise distances, so the measured markers may be any
subset of the model's. Of all matchings that keep the distances, the one with the smallest fit error wins.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from pybullet_planning.utils.transformations import superimposition_matrix

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

    def transform(self, points: np.ndarray) -> np.ndarray:
        """Model-frame points, shape (k, 3), in the world frame."""
        return points @ self.world_from_model[:3, :3].T + self.world_from_model[:3, 3]


def bar_markers_from_prefab(data: dict) -> dict[str, np.ndarray]:
    """The mocap markers of each bar in a prefab file that has any, in the bar frame, metres, shape (m, 3).

    Raises:
        ValueError: If `data` is not a prefab file (no bars list).
    """
    if not isinstance(data, dict) or not isinstance(data.get("bars"), list):
        raise ValueError("not a prefab file (no bars list)")
    bars = {}
    for bar in data["bars"]:
        markers = [position for joint in bar.get("joints", [])
                   for position in joint.get("mocap", {}).get("markers_mm", {}).values()]
        if markers:
            bars[str(bar["bar_id"])] = np.array(markers, dtype=float) * 1e-3
    return bars


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
