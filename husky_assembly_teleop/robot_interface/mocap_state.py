"""
What every mocap-tracked body stores (bases and tracked objects alike), without ROS.

`mocap.py` fills it from the relay; `world/mocap.py` judges it.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Protocol

import numpy as np

#: Seconds of valid samples whose mean marker error the chip judges.
MARKER_ERROR_WINDOW = 1.0


@dataclass(frozen=True)
class RawMarkers:
    """The raw markers Motive labeled to a rigid body in one sample: world frame (Z-up), metres, not calibrated.

    Attributes:
        ids: 1-based member numbers in the Motive asset, shape (n,).
        positions: Shape (n, 3).
        measured: False where the marker was occluded and filled in from Motive's own pose, shape (n,).
    """

    ids: np.ndarray
    positions: np.ndarray
    measured: np.ndarray


class MocapBody(Protocol):
    """The measured fields every mocap-tracked body has (BaseState, TrackedObject).

    Attributes:
        position: Last valid world-frame position, metres, or None before one.
        orientation: Last valid orientation, quaternion (x, y, z, w).
        tracked: Whether the latest sample was valid.
        tracking_valid: Whether NatNet tracked the body in the latest sample.
            False usually means hidden markers.
        marker_error: Mean marker error of the latest sample, metres.
        marker_errors: (ROS time, marker error) of the valid samples in the last MARKER_ERROR_WINDOW, oldest first.
        last_update_time: ROS time of the latest sample, valid or not, seconds.
        last_fix_time: ROS time the latest valid sample was captured, seconds.
        markers: Raw markers of the latest sample, valid or not, or None before one.
    """

    position: np.ndarray | None
    orientation: np.ndarray | None
    tracked: bool
    tracking_valid: bool | None
    marker_error: float | None
    marker_errors: deque[tuple[float, float]]
    last_update_time: float | None
    last_fix_time: float | None
    markers: RawMarkers | None
