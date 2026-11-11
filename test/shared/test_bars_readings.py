"""Checks for the old monitor's readings of a bar's fit: each marker, each plate's ball pair, the console lines."""

from __future__ import annotations

import numpy as np
import pytest

from husky_assembly_teleop.shared.bars.layouts import BarLayout, fake_sensed_markers, fit_bar
from husky_assembly_teleop.shared.bars.pose import bar_slide_roll, pose_deviation
from husky_assembly_teleop.shared.bars.readings import MATCHED, MISSING, STRAY, bar_report, marker_rows, plate_geometry
from husky_assembly_teleop.world.marker_match import Sensed, transform_points

#: A made-up 2 m bar with four T20 plates, two balls 48.3 mm apart on each, the plates rolled differently.
PLATE_Z = (0.075, 0.430, 1.135, 1.670)
PAIR_MM = 48.3


def plated_layout() -> BarLayout:
    """Four plates along the bar (from its start), each a ball pair across the bar, named as the process names them."""
    names, points = [], []
    for k, (z, roll) in enumerate(zip(PLATE_Z, (0, 90, 180, 45))):
        block = np.array([[0.05, -PAIR_MM / 2e3, 0.0], [0.05, PAIR_MM / 2e3, 0.0]])
        for label, point in zip(("M1", "M2"), transform_points(bar_slide_roll(z - 1.0, roll), block)):
            names.append(f"joints/J{k + 1}_mocap/{label}")
            points.append(point)
    return BarLayout("bars/B9", tuple(names), np.array(points), 2.0)


def test_marker_rows_name_each_design_marker_and_the_strays():
    """Every design marker in layout order with both residuals, the fake's stray last; a hidden ball reads missing."""
    layout = plated_layout()
    sensed = fake_sensed_markers(np.eye(4), layout.points, noise_m=0.0)
    fit = fit_bar(sensed, layout)
    rows = marker_rows(fit, sensed, layout)
    assert [row.name for row in rows[:8]] == list(layout.names) and rows[8].name == "stray"
    assert [row.state for row in rows] == [MATCHED] * 8 + [STRAY]
    assert all(row.motive_mm == pytest.approx(0.5) for row in rows) and all(row.fit_mm < 0.01 for row in rows[:8])
    sensed_of = {d: s for s, d in fit.correspondences.items()}
    assert [row.sensed_id for row in rows[:8]] == [sensed.ids[sensed_of[d]] for d in range(8)]
    hidden = Sensed(np.delete(sensed.points, 0, axis=0), sensed.ids[1:], sensed.residuals[1:], 0.0, "fake")
    fit = fit_bar(hidden, layout)
    rows = marker_rows(fit, hidden, layout)
    assert [row.state for row in rows].count(MISSING) == 1 and fit.n_matched == 7


def test_plate_geometry_reads_pair_lengths_spacing_and_bow():
    """Exact markers: pairs 48.3, spacings as designed, no bend; an inner plate pushed 2 mm across bows by 2 mm."""
    layout = plated_layout()
    sensed = fake_sensed_markers(np.eye(4), layout.points, noise_m=0.0)
    geometry = plate_geometry(fit_bar(sensed, layout), sensed, layout)
    assert [row.plate for row in geometry.plates] == [f"joints/J{k}_mocap" for k in (1, 2, 3, 4)]
    assert all(row.length_mm == pytest.approx(PAIR_MM, abs=0.01) for row in geometry.plates)
    assert geometry.worst_length_err_mm == pytest.approx(0.0, abs=0.01)
    assert geometry.bend_mm == pytest.approx(0.0, abs=0.01)
    middles = [layout.points[2 * k:2 * k + 2].mean(axis=0) for k in range(4)]
    designed = [float(np.linalg.norm(b - a)) * 1e3 for a, b in zip(middles, middles[1:])]
    assert [row.mm for row in geometry.spacing] == pytest.approx(designed, abs=0.01)
    assert all(abs(row.err_mm) < 0.01 for row in geometry.spacing)
    assert geometry.to_json()["joints"][0]["joint"] == "joints/J1_mocap"
    # Push plate J2's two balls 2 mm across the bar (world = bar frame here): the fit stays, the plate bows.
    bent = layout.points.copy()
    bent[2:4, 0] += 0.002
    sensed = fake_sensed_markers(np.eye(4), bent, noise_m=0.0)
    geometry = plate_geometry(fit_bar(sensed, layout), sensed, layout)
    assert geometry.bend_mm == pytest.approx(2.0, abs=0.3) and geometry.plates[1].bend_mm == geometry.bend_mm
    assert geometry.worst_length_err_mm == pytest.approx(0.0, abs=0.05)


def test_bar_report_has_three_lines_in_the_old_words():
    """The verdict with the deviation in the bar frame, the residuals by name, the two ends, Rhino and MoCap."""
    layout = plated_layout()
    planned = np.eye(4)
    sensed = fake_sensed_markers(planned, layout.points, slide_m=0.03, roll_deg=2.0, noise_m=0.0)
    fit = fit_bar(sensed, layout)
    deviation = pose_deviation(fit.T_world_from_bar, planned)
    one, two, three = bar_report("bars/B9", fit, sensed, layout, planned, deviation)
    assert one.startswith("[mocap bar] bars/B9 | source fake (age 0 ms) | 8/8 matched | rms 0.0 mm, max 0.0 mm | "
                          "dpos 30.0 mm, dang 2.00 deg | in the bar frame: dx ")
    assert "dz(along) 30.0 mm | ok" in one and "AMBIGUOUS" not in one
    assert two.startswith("[mocap bar] residuals (mm): J1_mocap/M1 0.0, J1_mocap/M2 0.0, ")
    assert two.endswith("| missing: none | stray sensed: 1")
    assert three.startswith("[mocap bar] bar ends (world, mm): Rhino start [0 0 -1000] end [0 0 1000] | MoCap start "
                            "[0 0 -970] end [0 0 1030]")
    failed = fit_bar(Sensed(sensed.points[:2], sensed.ids[:2], None, 0.0, "fake"), layout)
    one, two, three = bar_report("bars/B9", failed, None, layout, None, None)
    assert "| source none (age 0 ms) | 0/8 matched | FAILED: " in one and three.endswith("Rhino - | MoCap -")
