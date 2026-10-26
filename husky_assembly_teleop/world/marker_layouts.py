"""
Each bar's marker layout from the process, its sensed markers from the Motive asset, and the fit of one to the other.

The old monitor's `fit_bar_from_cache` (BMF:850-915) without its marker cache.

- `bar_layouts(process)`: the markers of the connector halves mounted on each bar, with the bar's length;
  `bar_asset_id` names the Motive asset that streams them (Bxx -> 10xx).
- `sensed_from_raw(raw, now)`: the relay's markers with the untrusted ones dropped; `fit_bar` matches them to a layout.
- `fake_sensed_markers`: a made-up sample for the tests and a desk rehearsal.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from ..robot_interface.mocap_state import RawMarkers
from .bar_fit import (MARKER_SIZE_RANGE_M, MATCH_TOL_M, MAX_MARKER_ERROR_M, MIN_TRIANGLE_HEIGHT_M, TRACK_GATE_M,
                      BarFit, _design_triangles, _failed_fit, fit_near_hint, match_and_fit)
from .bar_pose import bar_slide_roll, transform_points
from .marker_fit import bar_markers

if TYPE_CHECKING:
    from scaffolding_assembly_core.process import Process

#: The lab convention for a bar's own Motive asset: bar Bxx streams as rigid body 1000 + xx (W:81-95).
BAR_ASSET_BASE = 1000

# What a fake marker reports as its Motive residual, and how far the fake stray marker sits from the bar (BMF:81-84).
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


@dataclass(frozen=True)
class Sensed:
    """The markers of one sample that may be fitted, after the untrusted ones were dropped.

    Attributes:
        points: (N, 3) marker centres in world, metres.
        ids: The marker id of each row (Motive's member number; 1.. for a fake).
        residuals: (N,) Motive's fit residual of each row, metres; None when the relay did not send them.
        age: How old the sample is, seconds (0.0 when the relay sent no capture time).
        source: Where the markers came from: "asset" (the bar's Motive asset) or "fake".
        dropped: Markers dropped for their residual or size.
        occluded: Markers Motive did not see (filled in from its own pose), dropped.
        notes: Checks that could not run, said once in a failed fit's reason.
    """

    points: np.ndarray
    ids: tuple[int, ...]
    residuals: np.ndarray | None
    age: float
    source: str
    dropped: int = 0
    occluded: int = 0
    notes: tuple[str, ...] = ()


# * ================================================================ the layouts of a process

def bar_layouts(process: Process) -> dict[str, BarLayout]:
    """Each bar's marker layout from the halves mounted on it (`marker_fit.bar_markers`), with the bar's length.

    Args:
        process: The process; a bar is the `mount` of halves with `markers`, its length its collision cylinder's.

    Returns:
        dict[str, BarLayout]: Bar id -> layout, bars with fewer than 3 markers left out. A layout whose markers lie
        on one line (its roll cannot be measured) carries a warning.
    """
    layouts = {}
    for bar, (names, points) in bar_markers(process).items():
        warnings = ()
        if not _design_triangles(points, MIN_TRIANGLE_HEIGHT_M):  # (BMF:325-327)
            warnings = (f"{bar}: its markers are collinear (no three of them form a triangle "
                        f"{MIN_TRIANGLE_HEIGHT_M * 1000:.0f} mm high), so its roll cannot be measured",)
        length = float(process.bodies[bar].geometry.collision[0].height)
        layouts[bar] = BarLayout(bar, names, points, length, warnings)
    return layouts


def bar_asset_id(bar: str) -> int | None:
    """The Motive streaming id of a bar's own asset by the lab convention: "bars/B20" -> 1020 (W:85-95).

    Args:
        bar: The bar's body id, or a bare "B20".

    Returns:
        int | None: `BAR_ASSET_BASE` + the bar number; None when the id has no bar number.
    """
    name = bar.rsplit("/", 1)[-1]
    number = name[1:] if name[:1] == "B" else ""
    return BAR_ASSET_BASE + int(number) if number.isdigit() else None


# * ================================================================ the sensed markers, and the fit

def sensed_from_raw(raw: RawMarkers, now: float) -> Sensed:
    """The relay's markers of a bar's asset, ready to fit: measured ones only, residual <= 3 mm, size 12-26 mm.

    ! A hidden marker of an asset still streams, at Motive's guess: it is never fitted (BMF:891-894). Residuals,
      sizes or a capture time the relay did not send leave that check out, which a failed fit's reason then says.

    Args:
        raw: The markers Motive labeled to the bar's asset in one sample.
        now: The current ROS time, seconds, for the sample's age.

    Returns:
        Sensed: The markers kept, their ids and residuals, the counts dropped, and the sample's age.
    """
    measured = np.asarray(raw.measured, dtype=bool)
    residuals = getattr(raw, "residuals", None)
    sizes = getattr(raw, "sizes", None)
    capture_time = getattr(raw, "capture_time", None)
    keep = measured.copy()
    notes = []
    if residuals is None:
        notes.append("marker residuals not relayed: residual check skipped")
    else:
        residuals = np.asarray(residuals, dtype=float)
        keep &= residuals <= MAX_MARKER_ERROR_M  # (BMF:896)
    if sizes is None:
        notes.append("marker sizes not relayed: size check skipped")
    else:
        sizes = np.asarray(sizes, dtype=float)
        keep &= (MARKER_SIZE_RANGE_M[0] <= sizes) & (sizes <= MARKER_SIZE_RANGE_M[1])  # (BMF:896)
    if capture_time is None:
        notes.append("no capture time relayed: age check skipped")
    occluded = int(np.count_nonzero(~measured))
    return Sensed(points=np.asarray(raw.positions, dtype=float).reshape(-1, 3)[keep],
                  ids=tuple(int(marker_id) for marker_id in np.asarray(raw.ids)[keep]),
                  residuals=None if residuals is None else residuals[keep],
                  age=0.0 if capture_time is None else now - capture_time, source="asset",
                  dropped=int(len(keep) - np.count_nonzero(keep) - occluded), occluded=occluded,  # (BMF:906)
                  notes=tuple(notes))


def fit_bar(sensed: Sensed, layout: BarLayout, *, max_age: float | None = None, hint: np.ndarray | None = None,
            gate_m: float = TRACK_GATE_M, **kwargs) -> BarFit:
    """Fit a bar's layout to one sample of sensed markers (BMF:850-915).

    * With `hint` (following live: the bar's last good pose, or the FK bar) the markers are first matched around that
      pose (`fit_near_hint`, ~1 ms); when the bar moved too far for that, the full search (`match_and_fit`) runs.

    Args:
        sensed: The sample, from `sensed_from_raw` or `fake_sensed_markers`.
        layout: The bar's designed markers.
        max_age: Refuse a sample older than this, seconds; None to never refuse.
        hint: 4x4 bar pose to match around first; None = the full search only.
        gate_m: With `hint`: how far a marker may be from where that pose puts it.
        **kwargs: Passed on to `match_and_fit` (``tol_m``, ``min_inliers``, ...).

    Returns:
        BarFit: The fit, named after the bar; ``sensed_ids[i]`` is the marker id of sensed row ``i``; ``tracked``
        True when the match around `hint` was used. A failed fit's reason also counts the markers dropped and names
        the checks that could not run.
    """
    points = layout.points
    if max_age is not None and sensed.age > max_age:
        fit = _failed_fit(f"the marker snapshot is {sensed.age:.1f} s old (limit {max_age:.1f} s); "
                          f"is Motive streaming?", 0, len(points))
        fit.bar_name = layout.bar
        return fit
    fit = None
    if hint is not None:
        fit = fit_near_hint(sensed.points, points, np.asarray(hint, dtype=float), gate_m=gate_m,
                            tol_m=kwargs.get("tol_m", MATCH_TOL_M), min_inliers=kwargs.get("min_inliers"))
    if fit is None:
        fit = match_and_fit(sensed.points, points, **kwargs)
    fit.bar_name = layout.bar
    fit.sensed_ids = list(sensed.ids)
    if not fit.ok and sensed.occluded:  # (BMF:906-913)
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
    """A made-up sample of a bar that sits a known bit off its pose (BMF:920-952).

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
    moved = transform_points(T_world_from_bar @ bar_slide_roll(slide_m, roll_deg), design_points)  # (BMF:945)
    moved = moved + rng.normal(0.0, noise_m, moved.shape)
    stray = moved.mean(axis=0) + _FAKE_STRAY_DISTANCE_M * T_world_from_bar[:3, 0]  # (BMF:948)
    rows = np.vstack([moved, stray])[rng.permutation(len(moved) + 1)]
    return Sensed(points=rows, ids=tuple(range(1, len(rows) + 1)), residuals=np.full(len(rows), _FAKE_MARKER_ERROR_M),
                  age=0.0, source="fake")
