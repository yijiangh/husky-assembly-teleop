"""
Fit a known marker layout, e.g. a bar's CAD markers, to raw mocap markers without Motive's labels.

Only measured markers near where a prior pose expects the model count; which model marker each is, or whether it
is a stray, comes from pairwise distances. Of all matchings that keep the distances, the one matching the most
markers wins, then the one with the smallest fit error.
A bar's layout comes from the process: the markers of the connector halves mounted on it (`bar_markers`).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from scaffolding_assembly_core.geometry import Pose, compose, invert

if TYPE_CHECKING:
    from scaffolding_assembly_core.process import Process

#: Two pairwise distances match when they differ by less than this, metres: CAD and marker placement tolerance.
DISTANCE_TOLERANCE = 0.01
#: A fit with a residual above this is refused, metres, so a stray never counts as a match by bending the fit.
RESIDUAL_LIMIT = 0.004
#: Measured markers farther than this from every model marker where the prior puts it are ignored, metres.
SEARCH_RADIUS = 0.08
#: Steps of the matching search at most per fit, so a crowded cloud never stalls the caller; the best so far wins.
MAX_SEARCH_STEPS = 20000


@dataclass(frozen=True)
class MarkerFit:
    """A rigid fit of model markers onto measured ones.

    Attributes:
        world_from_model: 4x4 transform from the model frame to the world frame.
        model_index: Model marker matched to each measured marker, shape (n,).
        residuals: Distance of each measured marker to its fitted model marker, metres, shape (n,).
        points: The matched measured markers, shape (n, 3).
    """

    world_from_model: np.ndarray
    model_index: np.ndarray
    residuals: np.ndarray
    points: np.ndarray

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


def fit_near(model: np.ndarray, cloud: np.ndarray, prior: np.ndarray, radius: float = SEARCH_RADIUS,
             tolerance: float = DISTANCE_TOLERANCE, limit: float = RESIDUAL_LIMIT) -> MarkerFit | None:
    """The best rigid fit of `model` onto the measured markers near where `prior` puts it, or None below 3 matches.

    Each measured marker within `radius` of a model marker placed by `prior` is one of those model markers or a
    stray; a matching must keep every pairwise distance within `tolerance`. The most matched markers win, then the
    smallest error. The search then starts again from the fit while that matches more markers, so a prior a few
    degrees off still reaches the far end of a long bar. Labels play no part.

    Args:
        model: Model markers in the model frame, shape (m, 3).
        cloud: Every measured marker, shape (n, 3), in any order, strays included.
        prior: 4x4 transform from the model frame to the world where the model is expected, e.g. the planned bar.
        radius: See SEARCH_RADIUS.
        tolerance: See DISTANCE_TOLERANCE.
        limit: See RESIDUAL_LIMIT.
    """
    fit = _match_near(model, cloud, prior, radius, tolerance, limit)
    while fit is not None:
        again = _match_near(model, cloud, fit.world_from_model, radius, tolerance, limit)
        if again is None or len(again.residuals) <= len(fit.residuals):
            return fit
        fit = again
    return None


def _match_near(model: np.ndarray, cloud: np.ndarray, prior: np.ndarray, radius: float, tolerance: float,
                limit: float) -> MarkerFit | None:
    """One search of `fit_near`, from one prior: each model marker in turn gets a nearby measured marker or none."""
    expected = model @ prior[:3, :3].T + prior[:3, 3]
    distances = np.linalg.norm(cloud[:, None] - expected[None], axis=-1).reshape(len(cloud), len(model))
    near = np.flatnonzero(distances.min(axis=1, initial=np.inf) <= radius)
    points, distances = cloud[near], distances[near]
    # * Nearest first, so the first matchings found are good ones and prune the rest.
    options = [[int(j) for j in np.argsort(distances[:, k]) if distances[j, k] <= radius] for k in range(len(model))]
    model_distances = np.linalg.norm(model[:, None] - model[None], axis=-1)
    point_distances = np.linalg.norm(points[:, None] - points[None], axis=-1)
    best: MarkerFit | None = None
    steps = 0

    def extend(k: int, matched: list[tuple[int, int]], fit: MarkerFit | None) -> None:
        """Match model markers k onwards, given `matched` (point, model marker) and its fit from 3 on."""
        nonlocal best, steps
        steps += 1
        least = max(3, len(best.residuals) if best is not None else 0)
        if steps > MAX_SEARCH_STEPS or len(matched) + len(model) - k < least:
            return
        if k == len(model):
            if fit is not None and (best is None or len(matched) > len(best.residuals) or fit.rms < best.rms):
                best = fit
            return
        choices = options[k]
        if fit is not None:
            # * From 3 matches on, the fit says where this marker is: only markers there can be it.
            choices = [j for j in choices if np.linalg.norm(points[j] - fit.transform(model[k])) < tolerance]
        for j in choices:
            if all(j != used and abs(point_distances[j, used] - model_distances[k, i]) < tolerance
                   for used, i in matched):
                grown = matched + [(j, k)]
                grown_fit = (_fit(model, points[[p for p, _ in grown]], np.array([i for _, i in grown]))
                             if len(grown) >= 3 else None)
                if grown_fit is None or grown_fit.residuals.max() < limit:
                    extend(k + 1, grown, grown_fit)
        extend(k + 1, matched, fit)  # * Or it is hidden.

    extend(0, [], None)
    return best


def _fit(model: np.ndarray, measured: np.ndarray, model_index: np.ndarray) -> MarkerFit:
    """The least-squares rigid fit for one matching (Kabsch); rotations only, never mirrored."""
    source = model[model_index]
    source_mean, measured_mean = source.mean(axis=0), measured.mean(axis=0)
    u, _, vt = np.linalg.svd((source - source_mean).T @ (measured - measured_mean))
    rotation = vt.T @ np.diag([1.0, 1.0, np.sign(np.linalg.det(vt.T @ u.T))]) @ u.T
    world_from_model = np.eye(4)
    world_from_model[:3, :3], world_from_model[:3, 3] = rotation, measured_mean - rotation @ source_mean
    fitted = source @ rotation.T + world_from_model[:3, 3]
    return MarkerFit(world_from_model, model_index, np.linalg.norm(fitted - measured, axis=1), measured)


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
