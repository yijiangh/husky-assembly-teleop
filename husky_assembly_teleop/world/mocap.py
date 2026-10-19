"""
Judging a mocap fix, the same for robot bases and tracked objects.

Only reads stored fields, so it needs no ROS; receiving samples is robot_interface/mocap.py's job.
"""

from __future__ import annotations

from ..robot_interface.mocap_state import MARKER_ERROR_WINDOW, MocapBody
from .checks import BAD, GOOD, STALE_AFTER, WARN, Check

#: Mean marker error, metres, above which the mocap chip turns amber.
#: ? A guess from typical OptiTrack numbers. The relay's own looser threshold
#:   (`marker_error_valid_threshold`) marks the pose invalid.
MARKER_ERROR_WARN = 2e-3
#: Seconds without a valid pose after which nothing may drive from it; the onboard follower's `mocap_timeout_sec`.
POSE_TIMEOUT = 0.3


def mocap_check(label: str, mocap_id: int | None, body: MocapBody, now: float) -> Check:
    """Judge whether a body's pose is live and how good the fix is: GOOD safe, WARN imprecise, BAD unusable.

    Loss turns the chip red at once; amber needs the mean marker error over MARKER_ERROR_WINDOW above the threshold.

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
    mean = mean_marker_error(body, now)
    if mean is not None and mean > MARKER_ERROR_WARN:
        return Check(label, WARN, f"mean marker error over {MARKER_ERROR_WINDOW:g}s above {warn_mm:g} mm; "
                                  f"check the markers and the rigid body definition")
    return Check(label, GOOD, f"rigid body {mocap_id}, mean marker error under {warn_mm:g} mm")


def pose_lost(mocap_id: int | None, body: MocapBody, now: float) -> str:
    """Why a body's pose is too old to drive from (no valid sample for POSE_TIMEOUT, or none yet), or "".

    ! Check this, not `tracked`, before and while driving from the pose: `tracked` stays True if the relay goes silent.

    Args:
        mocap_id: The body's rigid-body id, or None if it has none configured.
        body: The body's measured state.
        now: Current ROS time, seconds.

    Returns:
        str: The reason, or "" while the pose is live.
    """
    # * marker_errors holds the arrival time of each valid sample, so its newest entry is the last valid one.
    if mocap_id is not None and body.marker_errors and now - body.marker_errors[-1][0] <= POSE_TIMEOUT:
        return ""
    check = mocap_check("mocap", mocap_id, body, now)
    return check.detail if check.level != GOOD else f"no valid mocap pose for over {POSE_TIMEOUT:g}s"


def mean_marker_error(body: MocapBody, now: float) -> float | None:
    """Mean marker error, metres, of the valid samples in the last MARKER_ERROR_WINDOW, or None without one."""
    recent = [error for time, error in body.marker_errors if time > now - MARKER_ERROR_WINDOW]
    return sum(recent) / len(recent) if recent else None
