"""The mocap probe's floor calibration: plane fit, its quality, the lab frame it places, and points files."""

from __future__ import annotations

import json
from dataclasses import asdict

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose, compose

from husky_assembly_teleop.world.coverage_map import CoverageMap
from husky_assembly_teleop.world.floor_calibration import (ANCHOR_LABEL, PLANE_MIN_POINTS, Sample, fit_plane,
                                                           floor_calibration_json, floor_from_raw, samples_from_json,
                                                           solve_floor)

pytestmark = pytest.mark.safety

#: A floor tilted by 0.5° about x and lifted 3 cm, as mocap might see it.
TILT = Rotation.from_euler("x", 0.5, degrees=True)
LIFT = np.array([0.0, 0.0, 0.03])


def floor_points(count: int, noise: float = 0.0, seed: int = 0) -> np.ndarray:
    """`count` points spread over a 6 m square of the tilted floor, with `noise` metres of height noise."""
    rng = np.random.default_rng(seed)
    flat = np.column_stack((rng.uniform(-3, 3, (count, 2)), rng.normal(0.0, noise, count)))
    return TILT.apply(flat) + LIFT


def sample(name: str, position) -> Sample:
    """A sample named "<label>_<n>" at `position`."""
    return Sample(id=0, name=name, label=name.rpartition("_")[0], position=list(map(float, position)),
                  orientation=[0.0, 0.0, 0.0, 1.0], mocap_id=1863, marker_error=1e-4, stamp=0.0)


def test_fit_needs_four_points():
    """Three points always fit exactly, so they say nothing about the noise; a point that is not a number is skipped."""
    assert fit_plane(floor_points(3)) is None
    plane = fit_plane(floor_points(4, noise=5e-4))
    assert np.isfinite(plane.normal_std) and plane.rms > 0
    assert not plane.is_good()
    assert fit_plane(np.vstack((floor_points(3), [np.nan, 0.0, 0.0]))) is None
    assert fit_plane(np.vstack((floor_points(4), [np.inf, 0.0, 0.0]))).residuals.shape == (4,)


def test_fit_finds_a_tilted_floor():
    """The normal points up, at the floor's tilt; residuals report the noise."""
    plane = fit_plane(floor_points(40, noise=5e-4))
    assert plane.normal == pytest.approx(TILT.apply([0.0, 0.0, 1.0]), abs=1e-3)
    assert np.degrees(plane.tilt) == pytest.approx(0.5, abs=0.02)
    assert plane.rms == pytest.approx(5e-4, rel=0.3)
    assert plane.max_error > plane.rms
    assert plane.height(LIFT + [0.0, 0.0, 0.1]) == pytest.approx(0.1, abs=1e-3)


def test_normal_spread_shrinks_with_more_points():
    """More points, spread out, pin the normal down: the quality chip turns good."""
    few, many = (fit_plane(floor_points(count, noise=5e-4)) for count in (PLANE_MIN_POINTS, 200))
    assert many.normal_std < few.normal_std
    assert many.is_good()


def test_points_on_a_line_give_no_normal():
    """Collinear points leave the plane's turn about that line unknown."""
    line = np.column_stack((np.linspace(0, 3, 5), np.zeros(5), np.zeros(5)))
    assert fit_plane(line).normal_std == np.inf
    assert not fit_plane(line).is_usable()


def test_a_wall_is_not_a_floor():
    """A well-fitted plane tilted far from mocap z is refused, so placing the lab frame never divides by zero."""
    wall = floor_points(10) @ Rotation.from_euler("y", 90, degrees=True).as_matrix().T
    plane = fit_plane(wall)
    assert plane.normal_std < 1e-6
    assert not plane.is_usable()
    solution = solve_floor([sample(f"floor_{i}", point) for i, point in enumerate(wall)]
                           + [sample("origin_1", wall[0])], {})
    with pytest.raises(ValueError, match="unusable"):
        floor_calibration_json(solution, "main_frame.json")


def test_the_lab_frame_on_the_floor():
    """The base point lands at the origin, the floor at z 0, and mocap x stays along floor x."""
    plane = fit_plane(floor_points(10))
    origin = TILT.apply([1.0, 2.0, 0.004]) + LIFT  # probed 4 mm above the floor
    transform = floor_from_raw(plane, origin)
    to_floor = lambda points: (np.c_[points, np.ones(len(points))] @ transform.T)[:, :3]  # noqa: E731
    assert to_floor([origin])[0] == pytest.approx([0.0, 0.0, 0.004], abs=1e-9)
    assert to_floor(floor_points(10, seed=1))[:, 2] == pytest.approx(0.0, abs=1e-9)
    x_axis = transform[:3, :3] @ [1.0, 0.0, 0.0]
    assert x_axis[1:] == pytest.approx(0.0, abs=1e-9)
    assert np.linalg.det(transform[:3, :3]) == pytest.approx(1.0)


def test_export_file():
    """The file holds the transform as matrix and as position plus quaternion, and the samples it came from."""
    floor = [sample(f"floor_{i}", point) for i, point in enumerate(floor_points(8, noise=3e-4), start=1)]
    origin = sample("origin_1", TILT.apply([0.5, 0.5, 0.0]) + LIFT)
    data = json.loads(json.dumps(floor_calibration_json(solve_floor([*floor, origin], {}), "main_frame.json")))
    calibration = data["floor_from_raw"]
    matrix = np.array(calibration["matrix"])
    assert Rotation.from_quat(calibration["orientation"]).as_matrix() == pytest.approx(matrix[:3, :3])
    assert calibration["position"] == pytest.approx(matrix[:3, 3])
    assert data["plane"]["points"] == 9  # the origin is on the floor too
    assert [item["name"] for item in data["samples"]][-1] == "origin_1"
    assert data["marks"]["origin"]["floor_xy"] == pytest.approx([0.0, 0.0], abs=1e-9)
    assert data["lab_frame_file"] is None
    with pytest.raises(ValueError, match="four floor or mark points"):
        floor_calibration_json(solve_floor([*floor[:2], origin], {}), "main_frame.json")


#: Marks and anchors as the lab frame file stores them.
MARKS = {"origin": np.array([0.0, 0.0]), "mark1": np.array([2.5, -1.0]), "mark2": np.array([-2.0, 2.0])}
ANCHORS = {"A1": np.array([-2.8, -3.4, 0.03]), "A2": np.array([3.4, -1.7, 0.02]), "A3": np.array([0.5, 4.0, 0.03])}


def pointed(anchors: dict) -> list[Sample]:
    """An anchor sample for each anchor (name: raw position of the tip), as "Anchor" records it."""
    return [Sample(id=0, name=f"{name}_1", label=ANCHOR_LABEL, position=list(map(float, position)),
                   orientation=[0.0, 0.0, 0.0, 1.0], mocap_id=0, marker_error=0.0, stamp=0.0)
            for name, position in anchors.items()]


def in_raw(floor_point) -> np.ndarray:
    """A lab frame point as Motive sees it: on the tilted, lifted floor, turned 10° and shifted."""
    turn = Rotation.from_euler("z", 10.0, degrees=True)
    return TILT.apply(turn.apply(floor_point) + [0.4, -0.2, 0.0]) + LIFT


def test_the_frame_is_fitted_to_marks_and_anchors_alike_and_the_two_compared():
    """Both agree: the joint fit lands every mark and anchor, the two alone place the lab together, heights match."""
    floor = [sample(f"floor_{i}", in_raw([x, y, 0.0])) for i, (x, y) in enumerate(
        np.random.default_rng(3).uniform(-3, 3, (8, 2)), start=1)]
    marks = [sample(f"{name}_1", in_raw([*xy, 0.0])) for name, xy in MARKS.items()]
    seen = {name: in_raw(xyz) for name, xyz in ANCHORS.items()}
    solution = solve_floor([*floor, *marks, *pointed(seen)], MARKS, ANCHORS, seen)
    assert sorted(solution.fits) == ["anchors", "both", "marks"] and solution.fit is solution.fits["both"]
    assert max(solution.fit.residuals.values()) < 1e-9 and solution.apart < 1e-9
    assert all(abs(off) < 1e-9 for off in solution.anchor_heights.values())
    data = json.loads(json.dumps(floor_calibration_json(solution, "main_frame.json")))
    assert data["anchors_in_floor"]["A2"]["floor_xyz"] == pytest.approx(ANCHORS["A2"].tolist(), abs=1e-9)
    assert data["marks_vs_anchors"] < 1e-9 and data["fits"]["anchors"]["rms"] < 1e-9


def test_marks_and_anchors_disagreeing_shows_how_far_apart():
    """The anchors seen 5 mm off in x where the marks say: the two alone place the lab about 5 mm apart."""
    floor = [sample(f"floor_{i}", in_raw([x, y, 0.0])) for i, (x, y) in enumerate(
        np.random.default_rng(3).uniform(-3, 3, (8, 2)), start=1)]
    marks = [sample(f"{name}_1", in_raw([*xy, 0.0])) for name, xy in MARKS.items()]
    seen = {name: in_raw(xyz + [0.005, 0.0, 0.0]) for name, xyz in ANCHORS.items()}
    solution = solve_floor([*floor, *marks, *pointed(seen)], MARKS, ANCHORS, seen)
    assert solution.apart == pytest.approx(0.005, abs=2e-4) and sorted(solution.moved) == sorted(ANCHORS)


def test_the_marks_alone_say_how_far_a_frame_is_off():
    """The frame the marks place is 0 mm off itself, 2 mm off one shifted 2 mm; no marks probed: nothing to say."""
    floor = [sample(f"floor_{i}", in_raw([x, y, 0.0])) for i, (x, y) in enumerate(
        np.random.default_rng(3).uniform(-3, 3, (8, 2)), start=1)]
    marks = [sample(f"{name}_1", in_raw([*xy, 0.0])) for name, xy in MARKS.items()]
    solution = solve_floor([*floor, *marks], MARKS)
    true = Pose.from_matrix(solution.fits["marks"].floor_from_raw)
    assert solution.marks_off(true) < 1e-9
    assert solution.marks_off(compose(Pose((0.002, 0.0, 0.0)), true)) == pytest.approx(0.002)
    assert solve_floor(floor, MARKS).marks_off(true) is None


def test_anchors_alone_place_the_frame_without_marks():
    """No mark probed: two anchors seen are enough."""
    floor = [sample(f"floor_{i}", in_raw([x, y, 0.0])) for i, (x, y) in enumerate(
        np.random.default_rng(3).uniform(-3, 3, (8, 2)), start=1)]
    seen = {name: in_raw(xyz) for name, xyz in ANCHORS.items()}
    solution = solve_floor([*floor, *pointed(seen)], MARKS, ANCHORS, seen)
    assert solution.transform is not None and "marks" not in solution.fits and solution.apart is None


def test_the_tips_only_point_at_the_anchors_the_markers_are_fitted_and_one_unread_is_left_out():
    """Tips 1 cm off each marker: the fit lands the markers; A3 not in the reading: left out, still a sample."""
    floor = [sample(f"floor_{i}", in_raw([x, y, 0.0])) for i, (x, y) in enumerate(
        np.random.default_rng(3).uniform(-3, 3, (8, 2)), start=1)]
    marks = [sample(f"{name}_1", in_raw([*xy, 0.0])) for name, xy in MARKS.items()]
    tips = pointed({name: in_raw(xyz + [0.01, 0.0, 0.0]) for name, xyz in ANCHORS.items()})
    read = {name: in_raw(xyz) for name, xyz in ANCHORS.items() if name != "A3"}
    solution = solve_floor([*floor, *marks, *tips], MARKS, ANCHORS, read)
    assert sorted(solution.anchors) == ["A1", "A2"] and max(solution.fit.residuals.values()) < 1e-9
    assert [s.name for s in solution.samples if s.label == ANCHOR_LABEL] == ["A1_1", "A2_1", "A3_1"]


def test_an_anchor_counts_in_3d_and_one_over_2_mm_off_is_left_out_as_moved():
    """A2 seen 3 mm higher than stored: on the floor it fits, in 3D it is 3 mm off; left out, exported at its place."""
    floor = [sample(f"floor_{i}", in_raw([x, y, 0.0])) for i, (x, y) in enumerate(
        np.random.default_rng(3).uniform(-3, 3, (8, 2)), start=1)]
    marks = [sample(f"{name}_1", in_raw([*xy, 0.0])) for name, xy in MARKS.items()]
    seen = {name: in_raw(xyz + ([0.0, 0.0, 0.003] if name == "A2" else 0.0)) for name, xyz in ANCHORS.items()}
    solution = solve_floor([*floor, *marks, *pointed(seen)], MARKS, ANCHORS, seen)
    assert solution.moved == {"A2": pytest.approx(0.003, abs=1e-6)} and "A2" not in solution.fit.residuals
    assert solution.worst[1] < 1e-9 and "A2" in solution.fits["anchors"].residuals
    data = json.loads(json.dumps(floor_calibration_json(solution, "main_frame.json")))
    assert data["anchors_in_floor"]["A2"]["floor_xyz"] == pytest.approx([*ANCHORS["A2"][:2], 0.023], abs=1e-9)
    in_floor = data["anchors_in_floor"]
    assert in_floor["A2"]["moved"] == pytest.approx(0.003) and in_floor["A1"]["moved"] is None


def test_an_anchor_moved_on_the_floor_leaves_the_frame_on_the_others():
    """A2 seen 5 mm off in x: left out as moved, the others land exactly, and its floor_xyz is 5 mm off its stored."""
    floor = [sample(f"floor_{i}", in_raw([x, y, 0.0])) for i, (x, y) in enumerate(
        np.random.default_rng(3).uniform(-3, 3, (8, 2)), start=1)]
    marks = [sample(f"{name}_1", in_raw([*xy, 0.0])) for name, xy in MARKS.items()]
    seen = {name: in_raw(xyz + ([0.005, 0.0, 0.0] if name == "A2" else 0.0)) for name, xyz in ANCHORS.items()}
    solution = solve_floor([*floor, *marks, *pointed(seen)], MARKS, ANCHORS, seen)
    assert list(solution.moved) == ["A2"] and max(solution.fit.residuals.values()) < 1e-9
    data = json.loads(json.dumps(floor_calibration_json(solution, "main_frame.json")))
    assert data["anchors_in_floor"]["A2"]["floor_xyz"] == pytest.approx((ANCHORS["A2"] + [0.005, 0, 0]).tolist())


def test_points_round_trip():
    """Samples read back from an export equal the exported ones, apart from the id."""
    sample = Sample(id=7, name="table_2", label="table", position=[1.0, 2.0, 0.5], orientation=[0.0, 0.0, 0.0, 1.0],
                    mocap_id=1863, marker_error=4e-4, stamp=12.5)
    data = json.loads(json.dumps({"samples": [asdict(sample)]}))
    assert asdict(samples_from_json(data)[0]) == {**asdict(sample), "id": 0}


def test_points_refuse_other_files():
    """A coverage map is not a points file, and a broken sample is reported."""
    coverage = CoverageMap()
    coverage.add((0.0, 0.0, 1.0), 5e-4, 1.0)
    with pytest.raises(ValueError, match="coverage"):
        samples_from_json(coverage.to_json())
    with pytest.raises(ValueError):
        samples_from_json({"samples": [{"name": "p_1"}]})
