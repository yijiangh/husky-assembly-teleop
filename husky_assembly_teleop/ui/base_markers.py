"""
A base path on the floor as the onboard follower drives it: lines, heading ticks, trail, and the follower's points.

- A path slowed in parts (narrow passages) is drawn by speed: its own colour at full speed, shading to SLOW_COLOR
  (amber) at the slowest (`path.MIN_SCALE`); the heading ticks keep its own colour.

! Main thread only, like every scene call; build in `setup`, update in `draw`.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np

from ..colors import PLAN, PREVIEW, TARGET, TRAIL, WARN, rgb

if TYPE_CHECKING:
    from crl_husky.follower_path import FollowerPath

    from ..plugin_api.context import PluginContext
    from ..robot_interface.base import FollowerState

#: Colours (r, g, b).
PREVIEW_COLOR = rgb(PREVIEW)       # blue: what would be sent
SENT_COLOR = rgb(PLAN)             # the path being followed
TRAIL_COLOR = rgb(TRAIL)           # amber: where the robot went
LOOKAHEAD_COLOR = rgb(TARGET)       # the point on the path the follower aims at
SLOW_COLOR = rgb(WARN)             # amber: where a path drives at its slowest
#: Speed scale drawn fully in SLOW_COLOR (the planner's slowest, `plugins/base_planner/path.MIN_SCALE`).
SLOWEST = 0.15
FLOOR = 0.005                      # m, just above the grid
LINE = 0.012                       # m
TICK = 0.08                        # m, heading ticks
TICK_EVERY = 0.15                  # m of path, or every drawn pose on a turn
#: Most trail points drawn; longer trails are thinned evenly.
TRAIL_POINTS = 500


class Markers:
    """The plugin's scene nodes, rebuilt only when what they show changes."""

    def __init__(self, ctx: PluginContext):
        """Add the follower's points, hidden until it follows a path."""
        self._ctx = ctx
        root = ctx.view.scene_root
        self._lines: dict[str, object] = {}
        #: (preview, sent) last drawn.
        self._paths: tuple = (None, None)
        #: Trail samples last drawn.
        # The trail drawn: its length and newest point, so a full trace that still moves is redrawn.
        self._trail_shown: tuple | None = None
        self._lookahead = ctx.view.scene.add_icosphere(f"{root}/lookahead", radius=0.025, color=LOOKAHEAD_COLOR,
                                                       visible=False)
        self._reference = ctx.view.scene.add_icosphere(f"{root}/reference", radius=0.02, color=SENT_COLOR,
                                                       visible=False)

    def paths(self, preview: FollowerPath | None, sent: FollowerPath | None) -> None:
        """Show the preview in blue and the path sent in green; None removes one."""
        if preview is self._paths[0] and sent is self._paths[1]:
            return
        self._paths = (preview, sent)
        self._path("preview", preview, PREVIEW_COLOR)
        self._path("sent", sent, SENT_COLOR)

    def trail(self, xy: np.ndarray | None) -> None:
        """Show where the robot went (n x 2, NaN rows skipped); None removes it."""
        shown = None if xy is None else (len(xy), xy[-1].tobytes() if len(xy) else b"")
        if shown == self._trail_shown:
            return
        self._trail_shown = shown
        self._remove("trail")
        if xy is None:
            return
        xy = xy[np.isfinite(xy).all(axis=1)]
        if len(xy) > TRAIL_POINTS:
            xy = xy[np.linspace(0, len(xy) - 1, TRAIL_POINTS).astype(int)]
        self._add("trail", floor_segments(xy, FLOOR * 2), TRAIL_COLOR, LINE)

    def follower(self, follower: FollowerState | None) -> None:
        """Show the follower's lookahead and reference points while it follows; None hides them."""
        following = follower is not None and follower.state == "following"
        look = following and np.isfinite(follower.lookahead).all()
        self._lookahead.visible = bool(look)
        if look:
            self._lookahead.position = (*follower.lookahead, FLOOR * 4)
        ref = following and np.isfinite(follower.reference[:2]).all()
        self._reference.visible = bool(ref)
        if ref:
            self._reference.position = (*follower.reference[:2], FLOOR * 4)

    def _path(self, key: str, path: FollowerPath | None, color) -> None:
        """Replace one path's line and heading ticks, or remove them."""
        self._remove(key)
        self._remove(f"{key}_ticks")
        if path is None:
            return
        poses = path.polyline()
        segments, keep = floor_segments(poses[:, :2], keep=True)
        scales = polyline_scales(path)
        if len(scales) == len(poses) and np.any(scales < 1.0):
            self._add(key, segments, speed_colors(np.minimum(scales[:-1], scales[1:])[keep], color), LINE)
        else:
            self._add(key, segments, color, LINE)
        self._add(f"{key}_ticks", ticks(poses), color, LINE / 2)

    def _add(self, key: str, segments: np.ndarray, color, thickness: float) -> None:
        """Add a set of line segments under `key`; nothing if there are none (turning on the spot).

        `color`: one (r, g, b), or one per segment end (n x 2 x 3), 0..255.
        """
        if len(segments):
            self._lines[key] = self._ctx.view.scene.add_line_segments(
                f"{self._ctx.view.scene_root}/{key}", segments, color, thickness=thickness)

    def _remove(self, key: str) -> None:
        """Remove the lines under `key`, if any."""
        line = self._lines.pop(key, None)
        if line is not None:
            line.remove()


def floor_segments(xy: np.ndarray, height: float = FLOOR, keep: bool = False):
    """Consecutive points as (n - 1) x 2 x 3 line segments at `height`, skipping zero-length ones.

    With `keep`, also which of the n - 1 steps were kept, as a boolean mask.
    """
    points = np.column_stack([xy, np.full(len(xy), height)])
    segments = np.stack([points[:-1], points[1:]], axis=1)
    kept = np.linalg.norm(segments[:, 1] - segments[:, 0], axis=1) > 1e-6
    return (segments[kept], kept) if keep else segments[kept]


def polyline_scales(path: FollowerPath, turn_step: float = math.radians(10.0)) -> np.ndarray:
    """The speed scale at each pose of `path.polyline(turn_step)`, walking its pieces the same way; 1 without scales."""
    scales = []
    for piece in path.pieces:
        if piece.turning:
            count = max(2, int(math.ceil(abs(piece.yaw1 - piece.yaw0) / turn_step)) + 1)
            scales += [getattr(piece, "scale", 1.0)] * count
        else:
            at = getattr(piece, "scale_at", None)
            scales += [1.0 if at is None else at(arc) for arc in piece.arc]
    return np.array(scales, dtype=float)


def speed_colors(scales: np.ndarray, color) -> np.ndarray:
    """Per-segment colours (n x 2 x 3) for speed scales: `color` at full speed, SLOW_COLOR at SLOWEST or below."""
    slow = np.clip((1.0 - np.asarray(scales, dtype=float)) / (1.0 - SLOWEST), 0.0, 1.0)[:, None]
    mixed = (1.0 - slow) * np.asarray(color, dtype=float) + slow * np.asarray(SLOW_COLOR, dtype=float)
    return np.repeat(mixed[:, None, :], 2, axis=1).astype(np.uint8)


def ticks(poses: np.ndarray) -> np.ndarray:
    """Short heading lines along a path drawing (n x 3): every TICK_EVERY metres, and every pose of a turn."""
    picked, travelled = [poses[0]], 0.0
    for previous, pose in zip(poses[:-1], poses[1:]):
        step = math.hypot(pose[0] - previous[0], pose[1] - previous[1])
        travelled += step
        if travelled >= TICK_EVERY or step < 1e-6:
            picked.append(pose)
            travelled = 0.0
    picked = np.array(picked + [poses[-1]])
    start = np.column_stack([picked[:, :2], np.full(len(picked), FLOOR)])
    end = start + TICK * np.column_stack([np.cos(picked[:, 2]), np.sin(picked[:, 2]), np.zeros(len(picked))])
    return np.stack([start, end], axis=1)
