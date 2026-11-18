"""A base path on the floor is drawn by speed: its own colour at full speed, amber where it drives slowest."""

import math

import numpy as np
from crl_husky.follower_path import FollowerPath

from husky_assembly_teleop.ui.base_markers import (PREVIEW_COLOR, SLOW_COLOR, SLOWEST, floor_segments,
                                                   polyline_scales, speed_colors)


def test_each_drawn_pose_gets_its_speed_scale():
    """Drive 2 m with the middle slowed, turn on the spot slowed: one scale per polyline pose, slow where it is."""
    x = np.r_[np.linspace(0.0, 2.0, 41), np.full(5, 2.0)]
    yaw = np.r_[np.zeros(41), np.linspace(0.0, math.pi / 2, 6)[1:]]
    scale = np.r_[np.where((x[:41] > 0.8) & (x[:41] < 1.2), 0.25, 1.0), np.full(5, 0.5)]
    path = FollowerPath.from_poses(x, 0 * x, yaw, None, scale)
    poses, scales = path.polyline(), polyline_scales(path)
    assert len(scales) == len(poses)
    middle = np.abs(poses[:, 0] - 1.0) < 0.1
    assert np.all(scales[middle & (np.abs(poses[:, 2]) < 1e-6)] == 0.25)
    assert scales[0] == 1.0 and scales[-1] == 0.5


def test_a_path_without_scales_is_all_full_speed():
    """A path sent without scales draws all at full speed."""
    x = np.linspace(0.0, 1.0, 11)
    assert np.all(polyline_scales(FollowerPath.from_poses(x, 0 * x, 0 * x)) == 1.0)


def test_colours_go_from_the_path_colour_to_amber():
    """Full speed is the path's colour, SLOWEST and below amber, between a mix."""
    colors = speed_colors(np.array([1.0, 0.5, SLOWEST, 0.05]), PREVIEW_COLOR)
    assert colors.shape == (4, 2, 3)
    assert tuple(colors[0, 0]) == PREVIEW_COLOR and tuple(colors[2, 1]) == SLOW_COLOR == tuple(colors[3, 0])
    assert all(min(a, b) <= c <= max(a, b) for a, b, c in zip(PREVIEW_COLOR, SLOW_COLOR, colors[1, 0]))


def test_the_kept_mask_matches_the_segments():
    """The mask says which steps became segments, so colours line up with them."""
    xy = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
    segments, kept = floor_segments(xy, keep=True)
    assert len(segments) == 2 and kept.tolist() == [True, False, True]
