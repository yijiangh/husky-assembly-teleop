"""
Each bar's marker layout from the process, and its fit to sensed markers.

- `bar_layouts(process)`: the markers of the connector halves mounted on each bar, in the bar's frame, with the bar's
  length.
- `fit_in_cloud(cloud, layout, where)`: fits a bar to the relay's raw markers near where it should be; Motive's
  labels and assets are never used (it mixes up bars that look alike).
- `fit_bar(sensed, layout)`: matches a sample (`world.marker_match.sensed_near`) to a layout.
- `drawn_bar(process, bar)`: the bar's and its mounted halves' visual shapes in the bar's frame, no collision shapes.
- `fake_sensed_markers`: a made-up sample for the tests and a desk rehearsal.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

import numpy as np

from ...world.marker_match import (MARKER_SIZE_RANGE_M, MATCH_TOL_M, MAX_MARKER_ERROR_M, MIN_TRIANGLE_HEIGHT_M,
                                   TRACK_GATE_M, BarFit, Sensed, design_triangles, failed_fit, fit_near_hint,
                                   match_and_fit, sensed_near, transform_points)
from .pose import bar_slide_roll
from scaffolding_assembly_core.geometry import Geometry, Pose, Shape, TriMesh, compose, invert

if TYPE_CHECKING:
    from scaffolding_assembly_core.process import Process

    from ...robot_interface.mocap_state import MarkerCloud

#: Raw markers farther than this from every marker of a bar where it should be are not its own, metres: well over the
#: few millimetres a bar sits off its plan, under the gap to most other bars.
SEARCH_RADIUS = 0.05
#: The full search takes at most this many markers near a bar: its 8, and its neighbours' halves at each joint.
MAX_NEAR = 32

# What a fake marker reports as its Motive residual, and how far the fake stray marker sits from the bar.
_FAKE_MARKER_ERROR_M = 0.0005
_FAKE_STRAY_DISTANCE_M = 0.3


@dataclass(frozen=True)
class BarLayout:
    """One bar's designed marker balls, in the bar's frame (origin at the MIDDLE, z along the bar).

    Attributes:
        bar: The bar's body id, e.g. "bars/B20".
        names: One per marker, "<half id>/<label>", in `points` order.
        points: (M, 3) ball centres in the bar frame, metres.
        length: The bar's length, metres (its collision cylinder's height).
        warnings: Plain-language problems that do not stop the fit, e.g. markers on one line.
    """

    bar: str
    names: tuple[str, ...]
    points: np.ndarray
    length: float
    warnings: tuple[str, ...] = ()


# * ================================================================ the layouts of a process

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


def bar_layouts(process: Process) -> dict[str, BarLayout]:
    """Each bar's marker layout from the halves mounted on it (`bar_markers`), with the bar's length.

    Args:
        process: The process; a bar is the `mount` of halves with `markers`, its length its collision cylinder's.

    Returns:
        dict[str, BarLayout]: Bar id -> layout, bars with fewer than 3 markers left out. A layout whose markers lie
        on one line (its roll cannot be measured) carries a warning.
    """
    layouts = {}
    for bar, (names, points) in bar_markers(process).items():
        warnings = ()
        if not design_triangles(points, MIN_TRIANGLE_HEIGHT_M):
            warnings = (f"{bar}: its markers are collinear (no three of them form a triangle "
                        f"{MIN_TRIANGLE_HEIGHT_M * 1000:.0f} mm high), so its roll cannot be measured",)
        length = float(process.bodies[bar].geometry.collision[0].height)
        layouts[bar] = BarLayout(bar, names, points, length, warnings)
    return layouts


def drawn_bar(process: Process, bar: str) -> Geometry:
    """A bar and the connector halves mounted on it as visual shapes in the bar's frame: drawn, never collides.

    A body without visual shapes is drawn with its collision ones.

    Args:
        process: The process.
        bar: The bar's body id.
    """
    body = process.bodies[bar]
    shapes = list(body.geometry.visual or body.geometry.collision)
    for half_id in process.halves_of(bar):
        half = process.bodies[half_id]
        # * Mounted halves stay where they sit on their bar, so the assembly poses give the offset.
        offset = compose(invert(body.assembly_pose), half.assembly_pose)
        shapes += [_moved(shape, offset) for shape in half.geometry.visual or half.geometry.collision]
    return Geometry(tuple(shapes), ())


def _moved(shape: Shape, offset: Pose) -> Shape:
    """A shape moved by `offset` in its body's frame."""
    if isinstance(shape, TriMesh):
        T = offset.matrix()
        vertices = shape.vertices @ T[:3, :3].T + T[:3, 3]
        vertices.setflags(write=False)
        return TriMesh(vertices, shape.faces, shape.convex)
    return replace(shape, origin=compose(offset, shape.origin))


# * ================================================================ the fit

def fit_in_cloud(cloud: MarkerCloud, layout: BarLayout, where: np.ndarray, now: float, *,
                 radius: float = SEARCH_RADIUS, hint: np.ndarray | None = None, **kwargs) -> tuple[Sensed, BarFit]:
    """Fit a bar to the cloud's markers within `radius` of where its markers should be.

    Args:
        cloud: The relay's newest marker cloud.
        layout: The bar's designed markers.
        where: 4x4 pose the bar should have (its live-scene pose, the FK bar, ...): the markers are looked for there.
        now: The current ROS time, seconds.
        radius: How far a marker may be from where one of the bar's should be, metres.
        hint: 4x4 pose to match around first; None = `where`.
        **kwargs: Passed on to `fit_bar` (``max_age``, ``full_search``, ``tol_m``, ...).

    Returns:
        tuple[Sensed, BarFit]: The markers near the bar, and the fit.
    """
    where = np.asarray(where, dtype=float)
    sensed = sensed_near(cloud, transform_points(where, layout.points), radius, now)
    fit = fit_bar(sensed, layout, hint=where if hint is None else hint, **{"max_sensed": MAX_NEAR, **kwargs})
    if not fit.ok:
        fit.reason += f" (markers looked for within {radius * 1e3:.0f} mm of where the bar should be)"
    return sensed, fit


def fit_bar(sensed: Sensed, layout: BarLayout, *, max_age: float | None = None, hint: np.ndarray | None = None,
            gate_m: float = TRACK_GATE_M, full_search: bool = True, hint_all_markers: bool = True,
            **kwargs) -> BarFit:
    """Fit a bar's layout to one sample of sensed markers.

    * With `hint` (following live: the bar's last good pose, or the FK bar) the markers are first matched around that
      pose (`fit_near_hint`, ~1 ms); when the bar moved too far for that, the full search (`match_and_fit`) runs.

    Args:
        sensed: The sample, from `sensed_near` or `fake_sensed_markers`.
        layout: The bar's designed markers.
        max_age: Refuse a sample older than this, seconds; None to never refuse.
        hint: 4x4 bar pose to match around first; None = the full search only.
        gate_m: With `hint`: how far a marker may be from where that pose puts it.
        hint_all_markers: Whether the match around `hint` must find every marker; False only when `hint` is a fit
            the full search found moments ago (`fit_near_hint`).
        full_search: Run the full search when there is no hint or the match around it fails; False gives a failed
            fit saying the search waits its turn (a plugin following several bars runs one search per tick).
        **kwargs: Passed on to `match_and_fit` (``tol_m``, ``min_inliers``, ...).

    Returns:
        BarFit: The fit, named after the bar; ``sensed_ids[i]`` is the marker id of sensed row ``i``; ``tracked``
        True when the match around `hint` was used. A failed fit's reason also counts the markers dropped and names
        the checks that could not run.
    """
    points = layout.points
    if max_age is not None and sensed.age > max_age:
        fit = failed_fit(f"the marker snapshot is {sensed.age:.1f} s old (limit {max_age:.1f} s); "
                         f"is Motive streaming?", 0, len(points))
        fit.bar_name = layout.bar
        return fit
    fit = None
    if hint is not None:
        fit = fit_near_hint(sensed.points, points, np.asarray(hint, dtype=float), gate_m=gate_m,
                            tol_m=kwargs.get("tol_m", MATCH_TOL_M), min_inliers=kwargs.get("min_inliers"),
                            all_markers=hint_all_markers)
    if fit is None and not full_search:
        fit = failed_fit("no match around the last pose (the full search waits its turn)", len(sensed.points),
                         len(points))
    elif fit is None:
        fit = match_and_fit(sensed.points, points, **kwargs)
    fit.bar_name = layout.bar
    fit.sensed_ids = list(sensed.ids)
    if not fit.ok and sensed.occluded:
        fit.reason += f" ({sensed.occluded} marker(s) occluded in Motive)"
    if not fit.ok and sensed.dropped:
        fit.reason += (f" ({sensed.dropped} marker(s) dropped: size outside {MARKER_SIZE_RANGE_M[0] * 1000:.0f}-"
                       f"{MARKER_SIZE_RANGE_M[1] * 1000:.0f} mm or error above {MAX_MARKER_ERROR_M * 1000:.0f} mm)")
    if not fit.ok:
        fit.reason += "".join(f" ({note})" for note in sensed.notes)
    return fit


# * ================================================================ desk rehearsal

def fake_sensed_markers(T_world_from_bar: np.ndarray, design_points: np.ndarray, *, slide_m: float = 0.03,
                        roll_deg: float = 2.0, noise_m: float = 0.0005, seed: int = 0) -> Sensed:
    """A made-up sample of a bar that sits a known bit off its pose.

    * Its fit should come out as ``T_world_from_bar @ bar_slide_roll(slide_m, roll_deg)``: slid ``slide_m`` along
      the bar and rolled ``roll_deg`` about it (up to the noise), so `pose_deviation` against ``T_world_from_bar``
      reads ``dpos_bar_mm ~ (0, 0, slide_m * 1000)`` and ``dang_deg ~ roll_deg``.

    One stray marker is added 0.3 m from the bar (along the bar's x, in world) and the rows are shuffled, so the
    matcher has to work for it. Every marker counts as measured, a 19 mm ball with a 0.5 mm residual.

    Args:
        T_world_from_bar: 4x4 pose the offset starts from, e.g. the designed one.
        design_points: (M, 3) design markers in the bar frame, metres.
        slide_m: Slide along the bar, metres.
        roll_deg: Roll about the bar, degrees.
        noise_m: Standard deviation of the Gaussian noise added to each coordinate.
        seed: Random seed for the noise and the shuffle.

    Returns:
        Sensed: ``M + 1`` rows with ids ``1..M+1``, source "fake".
    """
    T_world_from_bar = np.asarray(T_world_from_bar, dtype=float)
    rng = np.random.default_rng(seed)
    moved = transform_points(T_world_from_bar @ bar_slide_roll(slide_m, roll_deg), design_points)
    moved = moved + rng.normal(0.0, noise_m, moved.shape)
    stray = moved.mean(axis=0) + _FAKE_STRAY_DISTANCE_M * T_world_from_bar[:3, 0]
    rows = np.vstack([moved, stray])[rng.permutation(len(moved) + 1)]
    return Sensed(points=rows, ids=tuple(range(1, len(rows) + 1)), residuals=np.full(len(rows), _FAKE_MARKER_ERROR_M),
                  age=0.0, source="fake")
