"""Where the cameras see a plane: each camera's footprint, and the plane split by how many cameras see each point."""

from __future__ import annotations

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from repo_paths import DATA
from husky_assembly_teleop.robot_interface.mocap_definitions import MocapCamera, read_cameras
from husky_assembly_teleop.world.camera_coverage import (aim_point, camera_fov, coverage_faces, coverage_regions,
                                                         footprint, outlines, plane_point, rectangle, segments,
                                                         triangles, view_edges)

#: A big plane to cut views with, metres.
BOUNDS = rectangle((-20.0, -20.0), (40.0, 40.0))


def looking_down(x: float, y: float, height: float, name: str = "PrimeX 22 #1") -> MocapCamera:
    """A camera at (x, y, height) looking straight down, its image's up along +y (turned as the relay announces)."""
    # * Announced frame: ahead +y, up +z. Down ahead and +y up: turn -90 degrees about x.
    return MocapCamera(name, (x, y, height), tuple(Rotation.from_euler("x", -90.0, degrees=True).as_quat()))


def half_widths(name: str, distance: float) -> np.ndarray:
    """Half the footprint's width and depth for a camera `distance` above the plane."""
    return distance * np.tan(np.radians(camera_fov(name)) / 2)


@pytest.mark.parametrize("height", [0.0, 1.0])
def test_a_camera_looking_down_sees_a_rectangle_its_field_of_view_wide(height):
    """At 4 m, a PrimeX 22: 79 x 47 degrees on the plane, centred under it; nearer the plane, smaller."""
    shape = footprint(looking_down(1.0, 2.0, 4.0), height, BOUNDS)
    width, depth = half_widths("PrimeX 22 #1", 4.0 - height)
    minx, miny, maxx, maxy = shape.bounds
    np.testing.assert_allclose([minx, maxx, miny, maxy], [1 - width, 1 + width, 2 - depth, 2 + depth], atol=1e-9)
    assert shape.area == pytest.approx(4 * width * depth)


def test_a_camera_above_looking_up_or_below_the_plane_sees_nothing_of_it():
    """The view is a one-sided pyramid: nothing behind the camera."""
    up = MocapCamera("Prime 13 #1", (0.0, 0.0, 3.0), tuple(Rotation.from_euler("x", 90.0, degrees=True).as_quat()))
    assert footprint(up, 0.0, BOUNDS).is_empty and footprint(looking_down(0.0, 0.0, 3.0), 4.0, BOUNDS).is_empty


def test_overlapping_views_split_the_plane_by_how_many_see_each_point():
    """Two equal footprints half overlapping: a third of their union seen by both; the rest of the bounds by none."""
    bounds = rectangle((-10.0, -10.0), (20.0, 20.0))
    width, depth = half_widths("PrimeX 22 #1", 4.0)
    shapes = [footprint(looking_down(x, 0.0, 4.0), 0.0, bounds) for x in (-width / 2, width / 2)]
    regions = coverage_regions(shapes, bounds)
    single = 4 * width * depth
    assert regions[2].area == pytest.approx(single / 2) and regions[1].area == pytest.approx(single)
    assert regions[0].area == pytest.approx(400.0 - 1.5 * single) and sum(r.area for r in regions.values()) == \
        pytest.approx(400.0)


def test_a_region_becomes_triangles_of_its_area():
    """Holes included: the plane seen by none around a footprint."""
    bounds = rectangle((-10.0, -10.0), (20.0, 20.0))
    regions = coverage_regions([footprint(looking_down(0.0, 0.0, 4.0), 0.0, bounds)], bounds)
    for region in regions.values():
        points, faces = triangles(region)
        a, b, c = (points[faces[:, k]] for k in range(3))
        area = 0.5 * np.abs((b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (c[:, 0] - a[:, 0]) * (b[:, 1] - a[:, 1]))
        assert area.sum() == pytest.approx(region.area)


def test_the_lab_floor_is_seen_by_at_least_one_camera_and_its_middle_by_all():
    """The lab's ten cameras (23 Oct 2026) over the 15 x 15 m map: nothing unseen at floor height."""
    cameras = read_cameras((DATA / "sim" / "motive_cameras.json").read_text())
    bounds = rectangle((-7.5, -7.5), (15.0, 15.0))
    shapes = [footprint(camera, 0.0, bounds) for camera in cameras.values()]
    regions = coverage_regions(shapes, bounds)
    assert 0 not in regions and max(regions) == 10
    assert sum(shape.covers(rectangle((-0.05, -0.05), (0.1, 0.1)).centroid) for shape in shapes) >= 9


def test_each_model_gets_its_field_of_view():
    """By how the name starts; an unknown model the default."""
    assert camera_fov("PrimeX 22 #72318") == (79.0, 47.0) and camera_fov("Prime 13 #31321") == (56.0, 46.0)
    assert camera_fov("Slim 3U #1") == (56.0, 46.0)


def test_the_view_edges_reach_the_plane_at_the_footprints_corners():
    """Looking down, the four edge rays meet the plane at the footprint's corners; the aim point lies under it."""
    camera = looking_down(1.0, 2.0, 4.0)
    width, depth = half_widths(camera.name, 3.0)
    ends = np.array([plane_point(camera.position, edge, 1.0) for edge in view_edges(camera)])
    np.testing.assert_allclose(sorted(map(tuple, ends[:, :2].round(9))),
                               sorted((1 + sx * width, 2 + sy * depth) for sx in (-1, 1) for sy in (-1, 1)), atol=1e-9)
    np.testing.assert_allclose(aim_point(camera, 1.0), (1.0, 2.0, 1.0), atol=1e-9)


def test_a_ray_away_from_the_plane_never_meets_it():
    """Pointing up, or level with it."""
    assert plane_point((0.0, 0.0, 3.0), (0.0, 0.0, 1.0), 0.0) is None
    assert plane_point((0.0, 0.0, 3.0), (1.0, 0.0, 0.0), 0.0) is None


def test_the_faces_count_their_cameras_and_their_outlines_are_segments():
    """Two half-overlapping views: faces seen by 0, 1 and 2, adding up to the bounds; every outline a straight piece."""
    bounds = rectangle((-10.0, -10.0), (20.0, 20.0))
    width, _ = half_widths("PrimeX 22 #1", 4.0)
    shapes = [footprint(looking_down(x, 0.0, 4.0), 0.0, bounds) for x in (-width / 2, width / 2)]
    faces = coverage_faces(shapes, bounds)
    assert sorted({count for count, _ in faces}) == [0, 1, 2]
    assert sum(face.area for _, face in faces) == pytest.approx(400.0)
    lines = segments(outlines(shapes, bounds))
    assert lines.shape[1:] == (2, 2) and len(lines) >= 4 + 8
