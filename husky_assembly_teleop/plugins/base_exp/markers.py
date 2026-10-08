"""
The base experiment's 3D view: path lines with heading ticks, the robot's trail, and the follower's points.

! Main thread only, like every scene call; build in `setup`, update in `draw`.
"""

from __future__ import annotations

import math

import numpy as np

from ...plugin_api.context import PluginContext
from ...robot_interface.base import FollowerState
from .plan import Plan

#: Colours (r, g, b).
PREVIEW_COLOR = (66, 99, 235)      # blue: what Start would send
SENT_COLOR = (47, 158, 68)         # green: the path being followed
TRAIL_COLOR = (232, 89, 12)        # orange: where the robot went
LOOKAHEAD_COLOR = (224, 49, 49)
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
        self._trail_count = 0
        self._lookahead = ctx.view.scene.add_icosphere(f"{root}/lookahead", radius=0.025, color=LOOKAHEAD_COLOR,
                                                       visible=False)
        self._reference = ctx.view.scene.add_icosphere(f"{root}/reference", radius=0.02, color=SENT_COLOR,
                                                       visible=False)

    def paths(self, preview: Plan | None, sent: Plan | None) -> None:
        """Show the preview in blue and the path sent in green; None removes one."""
        if preview is self._paths[0] and sent is self._paths[1]:
            return
        self._paths = (preview, sent)
        self._path("preview", preview, PREVIEW_COLOR)
        self._path("sent", sent, SENT_COLOR)

    def trail(self, xy: np.ndarray | None) -> None:
        """Show where the robot went (n x 2, NaN rows skipped); None removes it."""
        count = 0 if xy is None else len(xy)
        if count == self._trail_count:
            return
        self._trail_count = count
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

    def _path(self, key: str, plan: Plan | None, color) -> None:
        """Replace one path's line and heading ticks, or remove them."""
        self._remove(key)
        self._remove(f"{key}_ticks")
        if plan is None:
            return
        poses = plan.path.polyline()
        self._add(key, floor_segments(poses[:, :2]), color, LINE)
        self._add(f"{key}_ticks", ticks(poses), color, LINE / 2)

    def _add(self, key: str, segments: np.ndarray, color, thickness: float) -> None:
        """Add a set of line segments under `key`; nothing if there are none (turning on the spot)."""
        if len(segments):
            self._lines[key] = self._ctx.view.scene.add_line_segments(
                f"{self._ctx.view.scene_root}/{key}", segments, color, thickness=thickness)

    def _remove(self, key: str) -> None:
        """Remove the lines under `key`, if any."""
        line = self._lines.pop(key, None)
        if line is not None:
            line.remove()


def floor_segments(xy: np.ndarray, height: float = FLOOR) -> np.ndarray:
    """Consecutive points as (n - 1) x 2 x 3 line segments at `height`, skipping zero-length ones."""
    points = np.column_stack([xy, np.full(len(xy), height)])
    segments = np.stack([points[:-1], points[1:]], axis=1)
    keep = np.linalg.norm(segments[:, 1] - segments[:, 0], axis=1) > 1e-6
    return segments[keep]


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
