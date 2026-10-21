"""
Mocap on the ROS side: the relay topic of a rigid body, and how a sample is stored (bases and objects alike).

! Keep this the only reader of MocapRigidBodyPose, so bases and objects agree on what "tracked" means.
"""

from __future__ import annotations

from typing import Callable

import numpy as np
from crl_husky_msgs.msg import MocapRigidBodyPose

from .connections import RosConnections
from .event import MESSAGE_QUEUE, Event
from .mocap_state import MARKER_ERROR_WINDOW, MocapBody, MocapSample, RawMarkers


def mocap_topic(mocap_id: int) -> str:
    """The relay topic of one rigid body.

    * Its pose is already calibrated and in the Z-up world frame; don't transform it again.
    """
    return f"/mocap/rigid_body/id_{mocap_id}/pose"


def subscribe_mocap(ros: RosConnections, mocap_id: int, body: MocapBody,
                    now: Callable[[], float], event: Event | None = None) -> None:
    """Keep `body` updated from rigid body `mocap_id`.

    Args:
        ros: The connections the subscription belongs to.
        mocap_id: Rigid-body id in the mocap system.
        body: The measured state to write into.
        now: Current ROS time in seconds, for when a sample arrived.
        event: Fired with each sample's `MocapSample` once `body` holds it, or None.
    """
    def on_sample(message: MocapRigidBodyPose) -> None:
        sample = store_sample(body, message, now())
        if event is not None:
            event.fire(sample)

    ros.subscription(MocapRigidBodyPose, mocap_topic(mocap_id), on_sample, MESSAGE_QUEUE)


def store_sample(body: MocapBody, message: MocapRigidBodyPose, now: float) -> MocapSample:
    """Write one mocap sample into `body`; an invalid one only clears `tracked`, keeping the last valid pose.

    Args:
        body: The body's measured state, changed in place.
        message: The relay's sample, stamped with when the cameras captured it.
        now: ROS time of arrival, seconds; also the capture time of a sample without a stamp.

    Returns:
        MocapSample: The sample, for the body's event.
    """
    body.tracked = bool(message.pose_valid)
    # * Kept so the mocap chip can say why a pose is invalid.
    body.tracking_valid = bool(message.tracking_valid)
    body.marker_error = float(message.marker_error)
    body.markers = RawMarkers(ids=np.array(message.marker_ids, dtype=int),
                              positions=np.array([[p.x, p.y, p.z] for p in message.marker_positions]).reshape(-1, 3),
                              measured=np.array(message.marker_measured, dtype=bool))
    stamp = message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec
    # ! Capture time, so speeds and tracking errors use when the pose was true, not when it arrived.
    captured = stamp if stamp > 0.0 else now
    position, orientation = np.full(3, np.nan), np.full(4, np.nan)
    if body.tracked:
        p, q = message.pose.position, message.pose.orientation
        body.position = position = np.array([p.x, p.y, p.z])
        body.orientation = orientation = np.array([q.x, q.y, q.z, q.w])
        body.last_fix_time = captured
        body.marker_errors.append((now, body.marker_error))
    while body.marker_errors and body.marker_errors[0][0] < now - MARKER_ERROR_WINDOW:
        body.marker_errors.popleft()
    body.last_update_time = now
    return MocapSample(body.tracked, position, orientation, body.marker_error, now, captured)
