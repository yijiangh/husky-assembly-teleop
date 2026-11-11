"""
The old monitor's readings of a bar's fit: what the mocap_bars panel, its console lines and the as-built record show.

- `marker_rows`: each marker's state and residuals.
- `plate_geometry`: each plate's ball pair, the spacing between plates, the bar's bow.
- `bar_report`: the three `[mocap bar]` console lines.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from ...world.marker_match import transform_points
from .pose import PoseDeviation, bar_ends

if TYPE_CHECKING:
    from ...world.marker_match import BarFit, Sensed
    from .layouts import BarLayout

#: A marker row's state: matched to its design marker, a design marker no sensed one matched, a sensed marker
#: matched to nothing.
MATCHED, MISSING, STRAY = "matched", "missing", "stray"


@dataclass(frozen=True)
class MarkerRow:
    """One line of the per-marker report: a design marker (matched or missing) or a stray sensed marker.

    Attributes:
        name: The design marker's name ("joints/J20-21_mocap/M1"), or "stray" for a sensed marker matched to nothing.
        state: MATCHED, MISSING or STRAY.
        sensed_id: The sensed marker's id (Motive's member number), None for a missing design marker.
        motive_mm: Motive's own residual of the sensed marker, mm; None when not relayed or missing.
        fit_mm: The sensed marker's distance from its design marker at the fitted pose, mm; matched rows only.
    """

    name: str
    state: str
    sensed_id: int | None
    motive_mm: float | None
    fit_mm: float | None


def marker_rows(fit: BarFit, sensed: Sensed, layout: BarLayout) -> list[MarkerRow]:
    """Every design marker in layout order, matched or missing, then every stray sensed marker."""
    sensed_of = {d: s for s, d in fit.correspondences.items()}
    rows = []
    for d, name in enumerate(layout.names):
        s = sensed_of.get(d)
        if s is None:
            rows.append(MarkerRow(name, MISSING, None, None, None))
            continue
        motive = None if sensed.residuals is None or s >= len(sensed.residuals) else float(sensed.residuals[s]) * 1e3
        rows.append(MarkerRow(name, MATCHED, int(sensed.ids[s]) if s < len(sensed.ids) else None, motive,
                              float(fit.residuals_m[s]) * 1e3 if s in fit.residuals_m else None))
    for s in fit.unmatched_sensed:
        motive = None if sensed.residuals is None or s >= len(sensed.residuals) else float(sensed.residuals[s]) * 1e3
        rows.append(MarkerRow("stray", STRAY, int(sensed.ids[s]) if s < len(sensed.ids) else None, motive, None))
    return rows


@dataclass(frozen=True)
class PlateRow:
    """One MoCap joint's plate (two balls, M1 and M2) read in the fitted bar's frame.

    Attributes:
        plate: The plate's name, the joint half's id ("joints/J20-21_mocap").
        n_seen: How many of its balls were matched.
        z_mm: Where the plate sits along the bar, by design.
        length_mm: The sensed distance between its two balls; None unless both were seen.
        length_err_mm: That distance minus the design's (48.3 mm on a T20 plate). It does not change when the bar
            bends: an error over ~2 mm is a ball mislabelled, occluded or knocked loose.
        shift_mm: Where its balls sit against where the rigid fit puts them, bar frame (x, y across, z along), the
            mean over its matched balls; None when none was seen.
        roll_err_deg: The plate's turn about the bar against the design; None unless both balls were seen.
        bend_mm: Of the shift, the part across the bar the straight line through the outer plates' shifts does not
            explain: ~0 on a straight bar, the sag on a bowed one; None on the outer plates.
        bow_mm: The same as a direction (x, y) across the bar; None on the outer plates.
    """

    plate: str
    n_seen: int
    z_mm: float
    length_mm: float | None
    length_err_mm: float | None
    shift_mm: tuple[float, float, float] | None
    roll_err_deg: float | None
    bend_mm: float | None
    bow_mm: tuple[float, float] | None


@dataclass(frozen=True)
class SpacingRow:
    """The distance between two neighbouring plates' middles, sensed against the design."""

    from_plate: str
    to_plate: str
    mm: float
    err_mm: float


@dataclass(frozen=True)
class PlateGeometry:
    """A bar's plates along it, the spacing between neighbours, its largest bend and its worst ball pair."""

    plates: tuple[PlateRow, ...]
    spacing: tuple[SpacingRow, ...]
    bend_mm: float | None
    worst_length_err_mm: float | None

    def to_json(self) -> dict:
        """The geometry as plain lists, dicts and numbers (the old `pair_geometry` keys)."""
        return {"joints": [{"joint": row.plate, "n_seen": row.n_seen, "z_mm": row.z_mm, "length_mm": row.length_mm,
                            "length_err_mm": row.length_err_mm, "shift_mm": row.shift_mm,
                            "roll_err_deg": row.roll_err_deg, "bend_mm": row.bend_mm, "bow_mm": row.bow_mm}
                           for row in self.plates],
                "spacing": [{"from": row.from_plate, "to": row.to_plate, "mm": row.mm, "err_mm": row.err_mm}
                            for row in self.spacing],
                "bend_mm": self.bend_mm, "worst_length_err_mm": self.worst_length_err_mm}


EMPTY_GEOMETRY = PlateGeometry((), (), None, None)


def plate_geometry(fit: BarFit, sensed: Sensed, layout: BarLayout) -> PlateGeometry:
    """Each plate's ball pair, the spacing between plates and the bar's bow, from the matched balls.

    A plate is the design markers sharing a name up to the last "/"; its balls are their labels (M1, M2). Empty
    without a fitted pose.
    """
    if fit.T_world_from_bar is None:
        return EMPTY_GEOMETRY
    T_bar_from_world = np.linalg.inv(np.asarray(fit.T_world_from_bar, dtype=float))
    sensed_of = {d: s for s, d in fit.correspondences.items()}
    # * Group the design balls by plate; read each matched ball in the fitted bar frame.
    plates: dict[str, dict] = {}
    for d, name in enumerate(layout.names):
        plate, label = name.rsplit("/", 1) if "/" in name else (name, name)
        entry = plates.setdefault(plate, {"design": {}, "sensed": {}})
        entry["design"][label] = np.asarray(layout.points[d], dtype=float)
        s = sensed_of.get(d)
        if s is not None:
            entry["sensed"][label] = transform_points(T_bar_from_world, sensed.points[s])
    rows = []
    for plate, entry in plates.items():
        design_pts, sensed_pts = entry["design"], entry["sensed"]
        design_mid = np.mean(list(design_pts.values()), axis=0)
        shift = (np.mean([sensed_pts[label] - design_pts[label] for label in sensed_pts], axis=0) * 1e3
                 if sensed_pts else None)
        row = {"plate": plate, "n_seen": len(sensed_pts), "z_mm": float(design_mid[2] * 1e3), "length_mm": None,
               "length_err_mm": None, "roll_err_deg": None, "shift_mm": None if shift is None else tuple(shift),
               "bend_mm": None, "bow_mm": None, "mid": None, "design_mid": design_mid}
        if len(design_pts) == 2 and len(sensed_pts) == 2:
            a, b = sorted(design_pts)
            length = float(np.linalg.norm(sensed_pts[a] - sensed_pts[b]))
            row["length_mm"] = length * 1e3
            row["length_err_mm"] = (length - float(np.linalg.norm(design_pts[a] - design_pts[b]))) * 1e3
            mid = (sensed_pts[a] + sensed_pts[b]) / 2
            turn = np.degrees(np.arctan2(mid[1], mid[0]) - np.arctan2(design_mid[1], design_mid[0]))
            row["roll_err_deg"] = float((turn + 180.0) % 360.0 - 180.0)
            row["mid"] = mid
        rows.append(row)
    rows.sort(key=lambda row: row["z_mm"])
    # * Bending: the cross-bar shift of the inner plates off the line through the outer two.
    shifted = [row for row in rows if row["shift_mm"] is not None]
    if len(shifted) >= 3:
        first, last = shifted[0], shifted[-1]
        span = last["z_mm"] - first["z_mm"]
        for row in shifted[1:-1]:
            t = (row["z_mm"] - first["z_mm"]) / span if span else 0.0
            line = np.asarray(first["shift_mm"]) + t * (np.asarray(last["shift_mm"]) - np.asarray(first["shift_mm"]))
            bow = (np.asarray(row["shift_mm"]) - line)[:2]
            row["bow_mm"] = (float(bow[0]), float(bow[1]))
            row["bend_mm"] = float(np.linalg.norm(bow))
    bends = [row["bend_mm"] for row in rows if row["bend_mm"] is not None]
    # * Spacing between consecutive plates (both balls seen on each), against the design.
    spacing = []
    for prev, row in zip(rows, rows[1:]):
        if prev["mid"] is None or row["mid"] is None:
            continue
        mm = float(np.linalg.norm(row["mid"] - prev["mid"]) * 1e3)
        nominal = float(np.linalg.norm(row["design_mid"] - prev["design_mid"]) * 1e3)
        spacing.append(SpacingRow(prev["plate"], row["plate"], mm, mm - nominal))
    lengths = [abs(row["length_err_mm"]) for row in rows if row["length_err_mm"] is not None]
    return PlateGeometry(tuple(PlateRow(row["plate"], row["n_seen"], row["z_mm"], row["length_mm"],
                                        row["length_err_mm"], row["shift_mm"], row["roll_err_deg"], row["bend_mm"],
                                        row["bow_mm"]) for row in rows),
                         tuple(spacing), max(bends) if bends else None, max(lengths) if lengths else None)


def bar_report(bar: str, fit: BarFit, sensed: Sensed | None, layout: BarLayout, planned: np.ndarray | None,
               deviation: PoseDeviation | None) -> tuple[str, str, str]:
    """The old monitor's three `[mocap bar]` lines: the verdict, the residuals, the bar's ends.

    Args:
        bar: The bar's body id.
        fit: Its fit, ok or not.
        sensed: The sample it came from, or None without one.
        layout: The bar's designed markers.
        planned: 4x4 pose the bar should be at (the Rhino pose), or None.
        deviation: The fit against `planned`, or None.

    Returns:
        tuple[str, str, str]: (1) source, age, matched count, rms and max, the deviation also in the bar frame, ok
        or why not; (2) each matched marker's residual by name, the missing names, the stray count; (3) the bar's
        two ends in world, mm, Rhino and MoCap.
    """
    source = sensed.source if sensed is not None else "none"
    age = sensed.age if sensed is not None else 0.0
    parts = [f"[mocap bar] {bar}", f"source {source} (age {age * 1e3:.0f} ms)",
             f"{fit.n_matched}/{fit.n_design} matched"]
    if fit.rms_m is not None:
        parts.append(f"rms {fit.rms_m * 1e3:.1f} mm, max {(fit.max_residual_m or 0.0) * 1e3:.1f} mm")
    if deviation is not None:
        dx, dy, dz = deviation.dpos_bar_mm
        parts.append(f"dpos {deviation.dpos_mm:.1f} mm, dang {deviation.dang_deg:.2f} deg")
        parts.append(f"in the bar frame: dx {dx:.1f}, dy {dy:.1f}, dz(along) {dz:.1f} mm")
    parts.append("ok" if fit.ok else f"FAILED: {fit.reason}")
    if fit.ambiguous:
        parts.append("AMBIGUOUS")
    names = [name[len("joints/"):] if name.startswith("joints/") else name for name in layout.names]
    by_design = sorted(fit.correspondences.items(), key=lambda pair: pair[1])
    residuals = ", ".join(f"{names[d]} {fit.residuals_m[s] * 1e3:.1f}" for s, d in by_design if s in fit.residuals_m)
    missing = ", ".join(names[d] for d in fit.missing_design)
    second = (f"[mocap bar] residuals (mm): {residuals or 'none'} | missing: {missing or 'none'} | stray sensed: "
              f"{len(fit.unmatched_sensed)}")

    def ends(T: np.ndarray | None) -> str:
        if T is None:
            return "-"
        start, end = (" ".join(str(int(round(v * 1e3))) for v in point)  # int: no "-0"
                      for point in bar_ends(np.asarray(T, dtype=float), layout.length))
        return f"start [{start}] end [{end}]"
    third = (f"[mocap bar] bar ends (world, mm): Rhino {ends(planned)} | MoCap "
             f"{ends(fit.T_world_from_bar) if fit.ok else '-'}")
    return " | ".join(parts), second, third
