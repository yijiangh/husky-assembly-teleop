"""Small matplotlib helpers shared by the run reports (`plugins/base_exp/report.py`, `experiments/`). No ROS."""

from __future__ import annotations

import numpy as np

#: Heading arrows drawn per line in a top view.
ARROWS = 15


def heading_arrows(axis, poses: np.ndarray, color: str) -> None:
    """About ARROWS heading arrows along `poses` (n x 3)."""
    pick = np.unique(np.linspace(0, len(poses) - 1, min(ARROWS, len(poses))).astype(int))
    span = np.ptp(poses[:, :2], axis=0).max() if len(poses) > 1 else 0.0
    length = max(0.05, 0.06 * span)
    axis.quiver(poses[pick, 0], poses[pick, 1], length * np.cos(poses[pick, 2]), length * np.sin(poses[pick, 2]),
                color=color, alpha=0.6, angles="xy", scale_units="xy", scale=1.0, width=0.004)


def time_spans(t: np.ndarray, flags: np.ndarray) -> list[tuple[float, float]]:
    """Time spans where `flags` holds."""
    spans, start = [], None
    for time, flag in zip(t, flags):
        if flag and start is None:
            start = time
        elif not flag and start is not None:
            spans.append((start, time))
            start = None
    if start is not None:
        spans.append((start, t[-1]))
    return spans
