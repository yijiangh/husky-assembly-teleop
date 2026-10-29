"""Checks for the measured MoCap floor (mocap_floor.py) and the collision floor laid on it.

Two kinds of checks:

* A made-up floor with a known tilt and height, written to a temporary file:
  the plane maths (height above, z under a point, the grid) must reproduce it
  exactly. Runs on any machine.
* The real calibration file (MOCAP_FLOOR_CALIBRATION_FILE, on the shared drive):
  its own probe points must sit on the plane read back from it with the rms the
  file states, and a design problem's collision floor must lie on that plane.
  Skipped where the shared drive is not mounted.
"""
import json
import os

import numpy as np
import pytest

from husky_assembly_teleop import DESIGN_DATA_DIRECTORY, MOCAP_FLOOR_CALIBRATION_FILE
from husky_assembly_teleop.cfab_session import (
    GROUND_SLAB_THICKNESS, _slab_mesh_from_polygon, _walkable_ground_slabs,
)
from husky_assembly_teleop.mocap_floor import load_mocap_floor

# * The made-up floor: tilted 1 deg about the MoCap x axis (rising towards +y) and
# * 10 mm above the MoCap origin.
TILT_DEG = 1.0
HEIGHT_AT_ORIGIN = 0.010
# The problem the monitor runs today; its WalkableGround patch is ~17 x 17 m.
GROUND_PROBLEM = '261006_3bar_holding_test'

needs_floor_file = pytest.mark.skipif(
    not os.path.exists(MOCAP_FLOOR_CALIBRATION_FILE),
    reason='the floor calibration file is not on this machine',
)


def _write_made_up_floor(folder: str, probe_xy: list) -> tuple:
    """Write a floor calibration file for the made-up floor.

    Args:
        folder (str): Where to write it.
        probe_xy (list): ``(x, y)`` of the probe points to put on the floor.

    Returns:
        tuple: ``(path, probe_points)``, the file and the (N, 3) points written into it.
    """
    angle = np.radians(TILT_DEG)
    # The floor's up, and the floor height under (x, y): rises with y.
    normal = np.array([0.0, -np.sin(angle), np.cos(angle)])
    z_on_floor = lambda x, y: HEIGHT_AT_ORIGIN + np.tan(angle) * y  # noqa: E731
    probe_points = np.array([[x, y, z_on_floor(x, y)] for x, y in probe_xy])
    # floor_from_mocap: rotate the floor's up onto z (a turn about x), then put
    # the floor point under the origin at the floor origin.
    rotation = np.array([[1.0, 0.0, 0.0],
                         [0.0, np.cos(angle), np.sin(angle)],
                         [0.0, -np.sin(angle), np.cos(angle)]])
    assert np.allclose(rotation[2], normal)
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = -rotation @ np.array([0.0, 0.0, HEIGHT_AT_ORIGIN])
    raw = {
        'kind': 'mocap_floor_calibration',
        'floor_from_mocap': {'matrix': matrix.tolist()},
        'plane': {'points': len(probe_points), 'rms': 0.0, 'tilt_deg': TILT_DEG},
        'samples': [{'name': f'floor_{i}', 'label': 'floor', 'position': p.tolist()}
                    for i, p in enumerate(probe_points)],
    }
    path = os.path.join(folder, 'made_up_mocap_floor_calibration.json')
    with open(path, 'w') as f:
        json.dump(raw, f)
    return path, probe_points


def test_made_up_floor_is_read_back_exactly(tmp_path):
    path, probes = _write_made_up_floor(str(tmp_path), [(0, 0), (3, -4), (-2, 1), (1, 2)])
    floor = load_mocap_floor(path)
    assert floor is not None
    # Points on the floor are 0 above it; the floor under the origin is 10 mm up.
    assert np.allclose(floor.height_above(probes), 0.0, atol=1e-12)
    assert floor.z_at(0.0, 0.0) == pytest.approx(HEIGHT_AT_ORIGIN, abs=1e-12)
    for x, y, z in probes:
        assert floor.z_at(x, y) == pytest.approx(z, abs=1e-12)
    # A point 50 mm straight above the floor (along its up) reads 50 mm.
    assert floor.height_above(probes[1] + 0.05 * floor.normal) == pytest.approx(0.05, abs=1e-12)
    # The floor's own up leans 0 deg from it; the MoCap z axis leans by the tilt.
    assert floor.angle_to_normal_deg(floor.normal) == pytest.approx(0.0, abs=1e-6)
    assert floor.angle_to_normal_deg([0, 0, 1]) == pytest.approx(TILT_DEG, abs=1e-9)


def test_grid_lies_on_the_floor_and_covers_the_probes(tmp_path):
    path, probes = _write_made_up_floor(str(tmp_path), [(0, 0), (3, -4), (-2, 1)])
    floor = load_mocap_floor(path)
    ends = np.array([p for segment in floor.grid_segments(spacing=1.0, margin=1.0) for p in segment])
    assert np.allclose(floor.height_above(ends), 0.0, atol=1e-12)
    # It reaches past every probe point (the floor frame is ~MoCap x / y here).
    assert ends[:, 0].min() <= probes[:, 0].min() - 0.99
    assert ends[:, 0].max() >= probes[:, 0].max() + 0.99
    assert ends[:, 1].min() <= probes[:, 1].min() - 0.99
    assert ends[:, 1].max() >= probes[:, 1].max() + 0.99


def test_slab_top_lies_on_a_tilted_floor(tmp_path):
    path, _ = _write_made_up_floor(str(tmp_path), [(0, 0)])
    floor = load_mocap_floor(path)
    ring = [(-5, -5), (5, -5), (5, 5), (-5, 5)]
    slab = _slab_mesh_from_polygon(ring, top_z=[floor.z_at(x, y) for x, y in ring])
    vertices = np.array([slab.vertex_coordinates(k) for k in slab.vertices()])
    top, bottom = vertices[:4], vertices[4:]
    assert np.allclose(floor.height_above(top), 0.0, atol=1e-12)
    # The bottom is the slab thickness straight down from each top corner.
    assert np.allclose(top[:, 2] - bottom[:, 2], GROUND_SLAB_THICKNESS)
    # A level floor (one height) still works as before.
    level = _slab_mesh_from_polygon(ring, top_z=-0.01555)
    assert {round(level.vertex_coordinates(k)[2], 6) for k in range(4)} == {-0.01555}


def test_missing_or_wrong_file_falls_back(tmp_path):
    assert load_mocap_floor(os.path.join(str(tmp_path), 'nothing_here.json')) is None
    wrong = os.path.join(str(tmp_path), 'wrong.json')
    with open(wrong, 'w') as f:
        json.dump({'kind': 'something_else'}, f)
    assert load_mocap_floor(wrong) is None


@needs_floor_file
def test_real_floor_fits_its_own_probe_points():
    floor = load_mocap_floor()
    assert floor is not None and len(floor.samples) >= 3
    heights = floor.height_above(floor.samples)
    # * The plane read back must give exactly the rms the probe session reported:
    # * this proves the file's matrix is read the right way round (floor <- MoCap).
    assert np.sqrt(np.mean(heights ** 2)) == pytest.approx(floor.rms_m, abs=1e-6)
    # Every probe point is within a few mm of the floor (the floor is not perfectly flat).
    assert np.abs(heights).max() < 0.005
    # Its tilt matches the angle between its up and the MoCap z axis.
    assert floor.angle_to_normal_deg([0, 0, 1]) == pytest.approx(floor.tilt_deg, abs=1e-6)


@needs_floor_file
@pytest.mark.skipif(not os.path.isdir(os.path.join(DESIGN_DATA_DIRECTORY, GROUND_PROBLEM)),
                    reason='the design-study export is not on this machine')
def test_real_collision_floor_lies_on_the_measured_floor():
    floor = load_mocap_floor()
    slabs = _walkable_ground_slabs(GROUND_PROBLEM, floor)
    assert slabs
    for slab in slabs:
        vertices = np.array([slab.vertex_coordinates(k) for k in slab.vertices()])
        top = vertices[:len(vertices) // 2]
        assert np.allclose(floor.height_above(top), 0.0, atol=1e-9)
    # Without the floor, the patch keeps its exported height (-15.55 mm).
    level = _walkable_ground_slabs(GROUND_PROBLEM, None)[0]
    assert level.vertex_coordinates(0)[2] == pytest.approx(-0.01555, abs=1e-6)
