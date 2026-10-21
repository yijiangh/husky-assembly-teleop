"""
What every mocap-tracked body stores (bases and tracked objects alike), without ROS.

`mocap.py` fills it from the relay; `mocap_problem` and `pose_lost` say when a pose cannot be used, and
`world/mocap.py` grades it for the chips.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Protocol

import numpy as np

#: Seconds of valid samples whose mean marker error the chip judges.
MARKER_ERROR_WINDOW = 1.0
#: A relay silent for this long, seconds, is down.
RELAY_SILENT_AFTER = 1.0
#: Seconds without a valid pose after which nothing may drive from it; the onboard follower's `mocap_timeout_sec`.
POSE_TIMEOUT = 0.3


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


@dataclass(frozen=True)
class MocapSample:
    """One relay message of a rigid body, as a mocap event hands it on.

    Attributes:
        valid: Whether the pose is valid (`tracked`); else position and orientation are NaN.
        position: World-frame position, metres.
        orientation: Quaternion (x, y, z, w).
        marker_error: Mean marker error, metres.
        update_time: ROS time the monitor handled the message, seconds.
        stamp: ROS time the cameras captured it, seconds (the relay PC's clock, time synced).
    """

    valid: bool
    position: np.ndarray
    orientation: np.ndarray
    marker_error: float
    update_time: float
    stamp: float


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


def mocap_problem(mocap_id: int | None, body: MocapBody, now: float) -> str:
    """Why a body's pose is unusable now: no rigid body, no message yet, a silent relay, lost or invalid; else "".

    Args:
        mocap_id: The body's rigid-body id, or None if it has none configured.
        body: The body's measured state.
        now: Current ROS time, seconds.
    """
    if mocap_id is None:
        return "no mocap id configured; the pose is never measured"
    if body.last_update_time is None:
        return f"no message for rigid body {mocap_id} yet; is mocap_relay running?"
    if now - body.last_update_time > RELAY_SILENT_AFTER:
        return f"relay silent for over {RELAY_SILENT_AFTER:g}s (rigid body {mocap_id})"
    # * Reasons the relay marks a pose invalid: body lost, error past its threshold, or stale.
    if not body.tracking_valid:
        return f"rigid body {mocap_id} lost by mocap; markers hidden?"
    if not body.tracked:
        return "relay marks the pose invalid (marker error past its threshold, or stale)"
    return ""


def pose_lost(mocap_id: int | None, body: MocapBody, now: float) -> str:
    """Why a body's pose is too old to drive from (none valid arrived, or was captured, within POSE_TIMEOUT), or "".

    ! Check this, not `tracked`, before and while driving from the pose: `tracked` stays True if the relay goes silent.
    ! Relies on synced clocks (chrony) for the capture time, as the onboard follower does.

    Args:
        mocap_id: The body's rigid-body id, or None if it has none configured.
        body: The body's measured state.
        now: Current ROS time, seconds.

    Returns:
        str: The reason, or "" while the pose is live.
    """
    # * marker_errors holds the arrival time of each valid sample, so its newest entry is the last valid one.
    if mocap_id is not None and body.marker_errors and now - body.marker_errors[-1][0] <= POSE_TIMEOUT:
        # * The relay republishes its last frame as valid for 1 s after the stream stops; its capture time shows it.
        age = now - body.last_fix_time
        if age <= POSE_TIMEOUT:
            return ""
        return f"the newest mocap pose was captured {age:.1f}s ago (stream stopped, or clocks out of sync?)"
    return mocap_problem(mocap_id, body, now) or f"no valid mocap pose for over {POSE_TIMEOUT:g}s"
