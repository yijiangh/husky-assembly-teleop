"""
Judging a mocap fix, the same for robot bases and tracked objects.

Only reads stored fields, so it needs no ROS; receiving samples is robot_interface/mocap.py's job.
"""

from __future__ import annotations

from ..robot_interface.mocap_state import MARKER_ERROR_WINDOW, MocapBody, mocap_problem
from .checks import BAD, GOOD, WARN, Check

#: Mean marker error, metres, above which the mocap chip turns amber.
#: ? A guess from typical OptiTrack numbers. The relay's own looser threshold
#:   (`marker_error_valid_threshold`) marks the pose invalid.
MARKER_ERROR_WARN = 2e-3


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
    problem = mocap_problem(mocap_id, body, now)
    if problem:
        # * No rigid body configured is amber: the robot is fine, it is only never measured.
        return Check(label, WARN if mocap_id is None else BAD, problem)
    # ! No marker error value in the text (it changes every sample); colour carries it.
    warn_mm = MARKER_ERROR_WARN * 1e3
    mean = mean_marker_error(body, now)
    if mean is not None and mean > MARKER_ERROR_WARN:
        return Check(label, WARN, f"mean marker error over {MARKER_ERROR_WINDOW:g}s above {warn_mm:g} mm; "
                                  f"check the markers and the rigid body definition")
    return Check(label, GOOD, f"rigid body {mocap_id}, mean marker error under {warn_mm:g} mm")


def mean_marker_error(body: MocapBody, now: float) -> float | None:
    """Mean marker error, metres, of the valid samples in the last MARKER_ERROR_WINDOW, or None without one."""
    recent = [error for time, error in body.marker_errors if time > now - MARKER_ERROR_WINDOW]
    return sum(recent) / len(recent) if recent else None
