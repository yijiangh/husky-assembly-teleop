"""Checks for the bars' marker layouts from the process, their Motive assets, and the fit of a sample to a layout.

Most checks build markers in memory (`bar_fixtures`). The checks on the purged process, on the old 2ModulesDemo
export and on the Oct-1 take of the old marker rig need the Drive and are skipped without it.
"""

from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from bar_fixtures import (BAR_LENGTH, NOW, b20_design, b20_layout, random_rigid, sensed_of, t20_design)

from husky_assembly_teleop.drive import drive_root
from husky_assembly_teleop.robot_interface.mocap_state import MarkerCloud, RawMarkers
from husky_assembly_teleop.shared.bars.layouts import (
    SEARCH_RADIUS, BarLayout, bar_layouts, bar_markers, drawn_bar, fake_sensed_markers, fit_bar, fit_in_cloud)
from husky_assembly_teleop.shared.bars.pose import bar_slide_roll, pose_deviation
from husky_assembly_teleop.world.marker_match import sensed_from_raw, sensed_near, transform_points
from planning_fixtures import purged_process
from process_fixtures import build_process
from scaffolding_assembly_core.geometry import shape_mesh

#: Under the Drive root: the old Rhino export B20's layout came from, and an Oct-1 take of the OLD 4-marker rig.
OLD_EXPORT = "data_design_study/261016_2ModulesDemo_YH/MoCapMarkers.json"
OLD_RIG_TAKE = "data_experiment/bar_holding_acc_data/20261001/bar_holding_acc_20261001_1514.json"


def drive_file(relative: str) -> Path:
    """A file under the Drive root; skips the test without it."""
    root = drive_root()
    if root is None or not (root / relative).is_file():
        pytest.skip(f"needs HUSKY_DRIVE_ROOT with {relative}")
    return root / relative


# * ------------------------------------------------- the layouts from the process

def test_bar_layouts_from_the_purged_process():
    """The purged process gives B20, B21, B34 and B35 with 8 named markers each; B20 is 1.8 m and the table above."""
    layouts = bar_layouts(purged_process().process)
    assert {bar: len(layout.names) for bar, layout in layouts.items()} == {
        "bars/B20": 8, "bars/B21": 8, "bars/B34": 8, "bars/B35": 8}
    b20 = layouts["bars/B20"]
    assert b20.bar == "bars/B20" and b20.length == BAR_LENGTH and b20.warnings == ()
    assert b20.names == b20_layout().names
    assert np.abs(b20.points - b20_design()).max() < 5e-5


def test_bar_layouts_match_the_old_export():
    """The process B20 layout is the old ``MoCapMarkers.json`` B20 table shifted by L/2 (< 0.05 mm)."""
    with open(drive_file(OLD_EXPORT), "r") as f:
        old = json.load(f)
    assert old["schema_version"] == 2 and old["units"] == "m"
    table = {marker["name"]: np.array(marker["bar_m"]) for marker in old["bars"]["bar_B20"]["markers"]}
    assert len(table) == 8

    b20 = bar_layouts(purged_process().process)["bars/B20"]
    for name, point in zip(b20.names, b20.points):
        # "joints/J20-21_mocap/M1" was "J20-21_mocap_M1" in the export.
        half, label = name.rsplit("/", 1)
        old_point = table[f"{half.split('/')[-1]}_{label}"] - [0.0, 0.0, b20.length / 2.0]
        assert np.abs(old_point - point).max() < 5e-5, name


def test_bars_with_too_few_markers_are_left_out(tmp_path):
    """A bar whose halves carry fewer than 3 markers has no layout; with 3 it has, with its cylinder's length."""
    process = build_process(tmp_path)
    assert bar_layouts(process) == {}
    female = replace(process.bodies["joints/J1_female"],
                     markers={"a": (0.01, 0.0, 0.02), "b": (-0.01, 0.0, 0.02), "c": (0.0, 0.03, 0.0)})
    process = replace(process, bodies={**process.bodies, female.id: female})
    layout = bar_layouts(process)["bars/B1"]
    assert layout.names == ("joints/J1_female/a", "joints/J1_female/b", "joints/J1_female/c")
    assert layout.length == 0.9 and layout.warnings == ()


def test_bar_layout_comes_from_its_mounted_halves(tmp_path):
    """Each marker of a half lands where the half sits on its bar; bars with under 3 markers are left out."""
    process = build_process(tmp_path)
    # * Three markers on B1's female half, which sits 0.85 m up the bar; B2's half gets two, too few to fit.
    female = replace(process.bodies["joints/J1_female"],
                     markers={"a": (0.01, 0.0, 0.02), "b": (-0.01, 0.0, 0.02), "c": (0.0, 0.03, 0.0)})
    male = replace(process.bodies["joints/J1_male"], markers={"a": (0.0, 0.0, 0.0), "b": (0.0, 0.0, 0.1)})
    process = replace(process, bodies={**process.bodies, female.id: female, male.id: male})
    layouts = bar_markers(process)
    assert list(layouts) == ["bars/B1"]
    names, points = layouts["bars/B1"]
    assert names == ("joints/J1_female/a", "joints/J1_female/b", "joints/J1_female/c")
    np.testing.assert_allclose(points, [[0.01, 0.0, 0.87], [-0.01, 0.0, 0.87], [0.0, 0.03, 0.85]], atol=1e-6)


def test_a_collinear_layout_warns(tmp_path):
    """Balls on one line on B1's female half (0.85 m up the bar): the layout carries the warning, the fit fails."""
    process = build_process(tmp_path)
    female = replace(process.bodies["joints/J1_female"],
                     markers={f"m{k}": (0.0255, 0.0, z) for k, z in enumerate((0.0, 0.1, 0.2, 0.3))})
    process = replace(process, bodies={**process.bodies, female.id: female})
    layout = bar_layouts(process)["bars/B1"]
    assert len(layout.warnings) == 1
    assert "bars/B1" in layout.warnings[0] and "collinear" in layout.warnings[0]
    assert fit_bar(sensed_of(transform_points(random_rigid(9), layout.points)), layout).ok is False


def test_a_drawn_bar_is_its_and_its_halves_visual_shapes_where_they_sit_and_never_collides():
    """B20's drawing: its own shapes, then each mounted half's moved by where it sits on the bar; no collision."""
    process = purged_process().process
    geometry = drawn_bar(process, "bars/B20")
    bar = process.bodies["bars/B20"]
    halves = process.halves_of("bars/B20")
    assert not geometry.collision and halves
    halves_shapes = sum(len(process.bodies[half].geometry.visual) for half in halves)
    assert len(geometry.visual) == len(bar.geometry.visual) + halves_shapes
    # * Placed at the bar's assembly pose, every shape lands where the process has it.
    half = process.bodies[halves[0]]
    placed = transform_points(bar.assembly_pose.matrix(),
                              shape_mesh(geometry.visual[len(bar.geometry.visual)]).vertices)
    np.testing.assert_allclose(placed, transform_points(half.assembly_pose.matrix(),
                                                        shape_mesh(half.geometry.visual[0]).vertices), atol=1e-9)


def test_fit_in_cloud_uses_only_markers_near_where_the_bar_should_be():
    """B20 slid 30 mm among 60 far markers: only its own 8 are fitted; looked for elsewhere, nothing is found."""
    layout = bar_layouts(purged_process().process)["bars/B20"]
    seat = purged_process().process.bodies["bars/B20"].assembly_pose.matrix()
    near = transform_points(seat @ bar_slide_roll(0.03, 2.0), layout.points)
    far = seat[:3, 3] + np.random.default_rng(1).uniform(SEARCH_RADIUS + 0.3, 2.0, (60, 3))
    positions = np.concatenate([far, near])
    cloud = MarkerCloud(positions, np.full(len(positions), 0.019), np.full(len(positions), 5e-4),
                        np.arange(len(positions)), capture_time=NOW, arrival_time=NOW)
    sensed, fit = fit_in_cloud(cloud, layout, seat, NOW)
    assert fit.ok and len(sensed.points) == 8 and sensed.source == "cloud" and set(sensed.ids) == set(range(60, 68))
    np.testing.assert_allclose(fit.T_world_from_bar, seat @ bar_slide_roll(0.03, 2.0), atol=1e-9)
    elsewhere = seat.copy()
    elsewhere[:3, 3] += [0.0, 0.0, 3.0]
    sensed, fit = fit_in_cloud(cloud, layout, elsewhere, NOW)
    assert not fit.ok and len(sensed.points) == 0
    assert fit.reason.endswith(f"(markers looked for within {SEARCH_RADIUS * 1e3:.0f} mm of where the bar should be)")


def test_sensed_near_drops_noisy_and_odd_sized_markers_and_keeps_unknown_sizes():
    """Residual over 3 mm or size outside 12-26 mm: dropped and counted; a size of 0 (not reported) is kept."""
    positions = np.zeros((4, 3))
    cloud = MarkerCloud(positions, np.array([0.019, 0.0, 0.019, 0.04]), np.array([5e-4, 5e-4, 0.004, 5e-4]),
                        np.arange(4), capture_time=NOW - 0.1, arrival_time=NOW)
    sensed = sensed_near(cloud, np.zeros((1, 3)), 0.01, NOW)
    assert sensed.ids == (0, 1) and sensed.dropped == 2 and sensed.age == pytest.approx(0.1)


# * ------------------------------------------------- fitting a sample

def test_old_rig_take_does_not_fit_a_t20_bar():
    """Every take of the old 82 mm rig is refused against a T20 bar, without an exception."""
    with open(drive_file(OLD_RIG_TAKE), "r") as f:
        takes = json.load(f)["raw_data"]
    for take in takes:
        markers = list(take["bar_rig"].values())
        sample = sensed_of([marker["pos"] for marker in markers],
                           residuals=[marker["error"] for marker in markers],
                           sizes=[np.ravel(marker["size"])[0] for marker in markers])
        for layout in (BarLayout("bars/T20", (), t20_design(), BAR_LENGTH), b20_layout()):
            result = fit_bar(sample, layout)
            assert result.ok is False, result.reason


def test_fake_markers_round_trip():
    """The fake sample fits 30 mm along and 2 deg about the bar, the stray unmatched."""
    layout = b20_layout()
    T_design = random_rigid(14)
    # With the default 0.5 mm noise the roll about the bar wanders ~0.3 deg (one sigma).
    for noise_m, tol_mm, tol_deg in ((0.0, 1e-6, 1e-6), (0.0005, 1.5, 1.5)):
        sample = fake_sensed_markers(T_design, layout.points, noise_m=noise_m)
        assert sample.source == "fake" and sorted(sample.ids) == list(range(1, 10))
        result = fit_bar(sample, layout)
        assert result.ok, result.reason
        assert result.bar_name == "bars/B20"
        assert result.n_matched == 8
        assert len(result.unmatched_sensed) == 1
        # The stray is the one marker far from every design ball.
        placed = transform_points(T_design, layout.points)
        assert np.linalg.norm(placed - sample.points[result.unmatched_sensed[0]], axis=1).min() > 0.2
        dev = pose_deviation(result.T_world_from_bar, T_design)
        assert dev.dpos_bar_mm == pytest.approx([0.0, 0.0, 30.0], abs=tol_mm)
        assert dev.dang_deg == pytest.approx(2.0, abs=tol_deg)
        json.dumps(result.to_json())
        json.dumps(dev.to_json())
    # Exact markers come back exactly at the documented pose.
    exact = fit_bar(fake_sensed_markers(T_design, layout.points, noise_m=0.0), layout)
    assert np.allclose(exact.T_world_from_bar, T_design @ bar_slide_roll(0.03, 2.0))


def test_sensed_from_raw_filters():
    """Trusted markers fit with their ids; a blob and a bad residual are dropped, an old sample refused."""
    layout = BarLayout("bars/T20", (), t20_design(), BAR_LENGTH)
    T_true = random_rigid(15)
    moved = transform_points(T_true, layout.points)
    result = fit_bar(sensed_of(moved), layout)
    assert result.ok, result.reason
    assert np.allclose(result.T_world_from_bar, T_true)
    assert {result.sensed_ids[s] for s in result.correspondences} == set(range(1, 9))

    # A tiny blob and a marker Motive does not trust are dropped before matching.
    sizes, residuals = np.full(8, 0.0188), np.full(8, 0.0008)
    sizes[0], residuals[1] = 0.005, 0.01
    sample = sensed_of(moved, sizes=sizes, residuals=residuals)
    assert sample.dropped == 2 and sample.occluded == 0 and sample.notes == ()
    result = fit_bar(sample, layout)
    assert result.ok, result.reason
    assert result.n_matched == 6
    assert 1 not in result.sensed_ids and 2 not in result.sensed_ids

    old = sensed_of(moved, capture_time=NOW - 2.0)
    assert old.age == pytest.approx(2.0)
    refused = fit_bar(old, layout, max_age=0.5)
    assert refused.ok is False and "old" in refused.reason and refused.bar_name == "bars/T20"
    assert fit_bar(old, layout, max_age=None).ok


def test_unmeasured_markers_are_dropped():
    """A hidden marker of a Motive asset still streams, at Motive's guess, not measured: never fitted."""
    layout = BarLayout("bars/T20", (), t20_design(), BAR_LENGTH)
    moved = transform_points(random_rigid(16), layout.points)
    measured = np.ones(8, dtype=bool)
    # One unmeasured marker (Motive's guess sits exactly on the design spot): it is still missing.
    measured[0] = False
    result = fit_bar(sensed_of(moved, measured=measured), layout)
    assert result.ok and result.n_matched == 7 and 1 not in result.sensed_ids
    # Three unmeasured: 5 of 8 left, and the reason says why.
    measured[1:3] = False
    sample = sensed_of(moved, measured=measured)
    assert sample.occluded == 3 and sample.dropped == 0
    result = fit_bar(sample, layout)
    assert result.ok is False and "5 of 8" in result.reason and "3 marker(s) occluded" in result.reason


def test_markers_without_sizes_skip_the_size_check():
    """A relay that sends no sizes: every measured marker is kept, and a failed fit's reason says so once."""
    layout = BarLayout("bars/T20", (), t20_design(), BAR_LENGTH)
    points = transform_points(random_rigid(18), layout.points)
    raw = RawMarkers(ids=np.arange(1, 9), positions=points, measured=np.ones(8, dtype=bool))
    sample = sensed_from_raw(raw, NOW)
    assert len(sample.points) == 8 and sample.dropped == 0 and sample.age == 0.0
    note = "marker sizes not relayed: size check skipped"
    assert note in sample.notes
    # An ok fit says nothing; a failed one names the skipped check once.
    assert fit_bar(sample, layout).ok and fit_bar(sample, layout).reason == ""
    failed = fit_bar(sensed_from_raw(RawMarkers(np.arange(1, 3), points[:2], np.ones(2, dtype=bool)), NOW), layout)
    assert failed.ok is False and "only 2 markers" in failed.reason
    assert failed.reason.count(note) == 1


# * ------------------------------------------------- following live: the match around the last pose

# * Since the hint (the last fit), the bar slid this far along itself and rolled this much.
TRACK_SLIDE_M = 0.015
TRACK_ROLL_DEG = 1.0


def _moved_since_hint(seed: int) -> tuple:
    """A B20 pose (the hint) and the B20 markers after the bar moved a bit from it.

    Returns:
        tuple: ``(T_hint, T_now, sample)``: the old pose, the pose now (slid 15 mm, rolled 1 deg), and the markers
        at the pose now as a relay sample.
    """
    T_hint = random_rigid(seed)
    T_now = T_hint @ bar_slide_roll(TRACK_SLIDE_M, TRACK_ROLL_DEG)
    points = transform_points(T_now, b20_design())
    points = points + np.random.default_rng(seed).normal(0.0, 0.0005, points.shape)
    return T_hint, T_now, sensed_of(points)


def test_tracked_fit_matches_the_full_search():
    """A hint near the truth: the quick match is used and gives the full search's pose."""
    layout = b20_layout()
    T_hint, T_now, sample = _moved_since_hint(20)
    full = fit_bar(sample, layout)
    tracked = fit_bar(sample, layout, hint=T_hint)
    # * The quick match tries no guesses; the full search tries many.
    assert full.ok and not full.tracked and full.n_hypotheses > 0
    assert tracked.ok and tracked.tracked, tracked.reason
    assert tracked.n_matched == 8 and tracked.correspondences == full.correspondences
    assert np.abs(tracked.T_world_from_bar - full.T_world_from_bar).max() < 1e-6
    assert pose_deviation(tracked.T_world_from_bar, T_now).dpos_mm < 1.5
    assert tracked.ambiguous is False and tracked.second_best is None and tracked.n_hypotheses == 0
    assert tracked.bar_name == "bars/B20" and tracked.reason == ""
    assert json.loads(json.dumps(tracked.to_json()))["tracked"] is True


def test_tracked_fit_falls_back_when_the_hint_is_far():
    """A hint 100 mm off: no marker within 20 mm, so the full search runs (still ok, not tracked)."""
    layout = b20_layout()
    _T_hint, T_now, sample = _moved_since_hint(21)
    far = np.eye(4)
    far[:3, 3] = [0.1, 0.0, 0.0]
    result = fit_bar(sample, layout, hint=far @ T_now)
    assert result.ok, result.reason
    assert result.tracked is False and result.n_hypotheses > 0
    assert pose_deviation(result.T_world_from_bar, T_now).dpos_mm < 1.5


def test_fit_bar_without_the_full_search_says_it_waits():
    """A hint that misses with the full search withheld: a failed fit naming the wait, not a 20 ms search."""
    layout = b20_layout()
    _T_hint, T_now, sample = _moved_since_hint(21)
    far = np.eye(4)
    far[:3, 3] = [0.1, 0.0, 0.0]
    waiting = fit_bar(sample, layout, hint=far @ T_now, full_search=False)
    assert not waiting.ok and "waits its turn" in waiting.reason and waiting.bar_name == "bars/B20"
    assert fit_bar(sample, layout, hint=far @ T_now, full_search=True).ok


def test_tracked_fit_with_an_occluded_marker():
    """A hidden ball (not measured): never fitted, and the full search runs (7 of 8, not tracked)."""
    layout = b20_layout()
    T_hint, _T_now, sample = _moved_since_hint(22)
    measured = np.ones(8, dtype=bool)
    measured[3] = False
    sample = sensed_of(sample.points, measured=measured)
    result = fit_bar(sample, layout, hint=T_hint)
    assert result.ok and not result.tracked, result.reason
    assert result.n_matched == 7 and result.missing_design == [3]
    assert 4 not in result.sensed_ids


def test_tracked_fit_never_stays_on_a_partial_match():
    """A hint on a wrong pose that matches only some balls: the full search takes over and finds the truth."""
    layout = b20_layout()
    _T_hint, T_now, sample = _moved_since_hint(24)
    # The hint turned half a turn about the bar's middle: at most part of the balls line up.
    T_wrong = T_now @ bar_slide_roll(0.0, 180.0)
    result = fit_bar(sample, layout, hint=T_wrong)
    assert result.ok and result.n_matched == 8, result.reason
    assert pose_deviation(result.T_world_from_bar, T_now).dpos_mm < 1.5
