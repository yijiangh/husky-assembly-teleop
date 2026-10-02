"""
Judging a mocap fix, the same for robot bases and tracked objects.

Only reads stored fields, so it needs no ROS; receiving samples is robot_interface/mocap.py's job.
"""

from __future__ import annotations

from typing import Protocol

import numpy as np
from .checks import BAD, GOOD, STALE_AFTER, WARN, Check

#: Mean marker error, metres, above which the mocap chip turns amber.
#: ? A guess from typical OptiTrack numbers. The relay's own looser threshold
#:   (`marker_error_valid_threshold`) marks the pose invalid.
MARKER_ERROR_WARN = 2e-3


class MocapBody(Protocol):
    """The measured fields every mocap-tracked body has (BaseState, TrackedObject).

    Attributes:
        position: Last valid world-frame position, metres, or None before one.
        orientation: Last valid orientation, quaternion (x, y, z, w).
        tracked: Whether the latest sample was valid.
        tracking_valid: Whether NatNet tracked the body in the latest sample.
            False usually means hidden markers.
        marker_error: Mean marker error of the latest sample, metres.
        last_update_time: ROS time of the latest sample, valid or not, seconds.
        last_fix_time: ROS time of the latest valid sample, seconds.
    """

    position: np.ndarray | None
    orientation: np.ndarray | None
    tracked: bool
    tracking_valid: bool | None
    marker_error: float | None
    last_update_time: float | None
    last_fix_time: float | None


def mocap_check(label: str, mocap_id: int | None, body: MocapBody, now: float) -> Check:
    """Judge whether a body's pose is live and how good the fix is: GOOD safe, WARN imprecise, BAD unusable.

    Args:
        label: Chip text, e.g. "mocap" or the object's name.
        mocap_id: The body's rigid-body id, or None if it has none configured.
        body: The body's measured state.
        now: Current ROS time, seconds.

    Returns:
        Check: The mocap chip.
    """
    if mocap_id is None:
        return Check(label, WARN, "no mocap id configured; the pose is never measured")
    if body.last_update_time is None:
        return Check(label, BAD, f"no message for rigid body {mocap_id} yet; is mocap_relay running?")
    if now - body.last_update_time > STALE_AFTER:
        return Check(label, BAD, f"relay silent for over {STALE_AFTER:g}s (rigid body {mocap_id})")

    # ! No marker error value in the text (it changes every sample); colour carries it.
    warn_mm = MARKER_ERROR_WARN * 1e3
    # * Reasons the relay marks a pose invalid: body lost, error past its threshold, or stale.
    if not body.tracking_valid:
        return Check(label, BAD, f"rigid body {mocap_id} lost by mocap; markers hidden?")
    if not body.tracked:
        return Check(label, BAD, "relay marks the pose invalid (marker error past its threshold, or stale)")
    if body.marker_error > MARKER_ERROR_WARN:
        return Check(label, WARN, f"marker error above {warn_mm:g} mm; "
                                  f"check the markers and the rigid body definition")
    return Check(label, GOOD, f"rigid body {mocap_id}, marker error under {warn_mm:g} mm")
