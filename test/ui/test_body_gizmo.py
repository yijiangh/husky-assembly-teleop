"""Tests for ui.body_gizmo's pure parts: a click ray against shapes, and resizing by a corner."""

from __future__ import annotations

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.ui.body_gizmo import pick, resize_from_corner
from scaffolding_assembly_core.geometry import Pose, box_geometry
from scaffolding_assembly_core.scene import Body, Scene


def _scene(*bodies: tuple[str, Pose]) -> Scene:
    """A snapshot of 1 m boxes at these poses."""
    return Scene(bodies={i: Body(i, box_geometry((1.0, 1.0, 1.0)), pose) for i, pose in bodies},
                 world_poses={i: pose for i, pose in bodies})


def test_pick_takes_the_nearest_candidate_hit():
    """Looking down -x through two boxes picks the nearer; a body not offered is skipped; a miss gives None."""
    scene = _scene(("near", Pose((2.0, 0.0, 0.0))), ("far", Pose((5.0, 0.0, 0.0))))
    origin, direction = np.array([10.0, 0.0, 0.0]), np.array([-1.0, 0.0, 0.0])
    assert pick(scene, origin, direction, ["near", "far"]) == "far"
    assert pick(scene, origin, direction, ["near"]) == "near"
    assert pick(scene, origin, np.array([0.0, 0.0, 1.0]), ["near", "far"]) is None


def test_resize_keeps_the_opposite_corner():
    """Dragging the +x +y +z corner of a turned box out by 0.5 along its x grows it, the anchor stays put."""
    pose = Pose((1.0, 2.0, 0.5), tuple(Rotation.from_euler("z", 30, degrees=True).as_quat()))
    rotation, half = pose.matrix()[:3, :3], np.array([0.5, 0.5, 0.5])
    anchor = rotation @ -half + np.array(pose.position)
    corner = rotation @ (half + [0.5, 0.0, 0.0]) + np.array(pose.position)
    new_pose, size = resize_from_corner(pose, anchor, corner)
    assert size == pytest.approx((1.5, 1.0, 1.0))
    assert rotation @ -np.array(size) / 2 + np.array(new_pose.position) == pytest.approx(anchor)


def test_resize_square_uses_the_larger_side():
    """A cylinder keeps its x and y equal."""
    _, size = resize_from_corner(Pose(), np.zeros(3), np.array([0.2, 0.6, 1.0]), square=True)
    assert size == pytest.approx((0.6, 0.6, 1.0))
