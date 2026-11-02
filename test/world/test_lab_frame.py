"""The lab frame: fitted to its marks and anchors, and its file."""

from __future__ import annotations

import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from repo_paths import DATA
from husky_assembly_teleop.world.lab_frame import (FORMAT, MARK_WARN, fit_floor_points, floor_xy,
                                                   lab_frame_file, read_lab_frame)

pytestmark = pytest.mark.safety

#: The marks' floor (x, y), metres, as a file would store them.
KNOWN = {"origin": np.array([0.0, 0.0]), "mark1": np.array([3.5, -0.4]), "mark2": np.array([-2.8, 2.1]),
         "mark3": np.array([0.6, -4.2])}
#: Motive's frame against the floor: tilted 0.3°, turned 12° and shifted.
MOCAP_FROM_FLOOR = Rotation.from_euler("zx", [12.0, 0.3], degrees=True)
SHIFT = np.array([0.8, -1.3, 0.02])


def in_mocap(xy, height: float = 0.0) -> np.ndarray:
    """A floor point (x, y) at `height` above the floor, in mocap coordinates."""
    return MOCAP_FROM_FLOOR.apply([*xy, height]) + SHIFT


def plane() -> tuple[np.ndarray, np.ndarray]:
    """The floor plane's normal and a point on it, mocap frame."""
    return MOCAP_FROM_FLOOR.apply([0.0, 0.0, 1.0]), SHIFT


def test_fit_finds_the_frame_the_points_were_stored_in():
    """Probed exactly, every mark lands on its stored (x, y), floor z stays the normal, any height drops away."""
    measured = {name: in_mocap(xy, height=0.004) for name, xy in KNOWN.items()}
    fit = fit_floor_points(measured, KNOWN, *plane())
    assert max(fit.residuals.values()) == pytest.approx(0.0, abs=1e-9)
    for name, xy in KNOWN.items():
        assert floor_xy(fit.floor_from_raw, measured[name]) == pytest.approx(xy, abs=1e-9)
    assert fit.floor_from_raw[2, :3] == pytest.approx(plane()[0])
    assert np.linalg.det(fit.floor_from_raw[:3, :3]) == pytest.approx(1.0)


def test_noise_spreads_over_all_points_and_a_moved_mark_stands_out():
    """With 0.5 mm of probing noise the frame holds to about a millimetre at 7 m; a mark moved 5 mm is flagged."""
    rng = np.random.default_rng(3)
    measured = {name: in_mocap(xy + rng.normal(0.0, 5e-4, 2)) for name, xy in KNOWN.items()}
    fit = fit_floor_points(measured, KNOWN, *plane())
    assert floor_xy(fit.floor_from_raw, in_mocap((5.0, 5.0))) == pytest.approx([5.0, 5.0], abs=1.5e-3)
    measured["mark2"] = in_mocap(KNOWN["mark2"] + [0.005, 0.0])
    name, off = fit_floor_points(measured, KNOWN, *plane()).worst
    assert name == "mark2" and off > MARK_WARN


def test_two_points_are_enough_and_unknown_ones_are_left_out():
    """Two stored points place the frame; a probed point not in the file takes no part."""
    measured = {"mark1": in_mocap(KNOWN["mark1"]), "mark3": in_mocap(KNOWN["mark3"]), "mark9": in_mocap((9.0, 9.0))}
    fit = fit_floor_points(measured, KNOWN, *plane())
    assert set(fit.residuals) == {"mark1", "mark3"}
    assert floor_xy(fit.floor_from_raw, in_mocap(KNOWN["origin"])) == pytest.approx([0.0, 0.0], abs=1e-9)
    assert fit_floor_points({"mark1": measured["mark1"]}, KNOWN, *plane()) is None


def test_file_round_trip(tmp_path):
    """Next to the lab file: marks as (x, y), anchors as (x, y, z); a missing file is none, another file refused."""
    path = lab_frame_file(tmp_path, "main")
    assert path == tmp_path / "lab" / "main_frame.json"
    assert read_lab_frame(path).marks == {} and read_lab_frame(path).anchors == {}
    path.parent.mkdir()
    path.write_text(json.dumps({"format": FORMAT, "marks": {name: xy.tolist() for name, xy in KNOWN.items()},
                                "anchors": {"A1": [1.0, 2.0, 0.03]}}))
    frame = read_lab_frame(path)
    assert {name: xy.tolist() for name, xy in frame.marks.items()} == {name: xy.tolist() for name, xy in KNOWN.items()}
    assert frame.anchors["A1"].tolist() == [1.0, 2.0, 0.03]
    path.write_text(json.dumps({"format": FORMAT, "marks": {"A1": [0, 0]}, "anchors": {"A1": [0, 0, 0]}}))
    with pytest.raises(ValueError, match="share names"):
        read_lab_frame(path)
    path.write_text('{"format": "husky_lab"}')
    with pytest.raises(ValueError, match="not a lab frame file"):
        read_lab_frame(path)


def test_the_labs_frame_file_reads():
    """The committed file: the four marks, the origin at (0, 0)."""
    frame = read_lab_frame(DATA / "lab" / "main_frame.json")
    assert sorted(frame.marks) == ["mark1", "mark2", "mark3", "origin"] and frame.marks["origin"].tolist() == [0, 0]
