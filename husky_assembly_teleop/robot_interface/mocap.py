"""
Mocap on the ROS side: the relay topic of a rigid body, and how a sample is stored (bases and objects alike).

! Keep this the only reader of MocapRigidBodyPose, so bases and objects agree on what "tracked" means.
"""

from __future__ import annotations

from typing import Callable

import numpy as np
from crl_husky_msgs.msg import MocapRigidBodyPose

from ..world.mocap import MARKER_ERROR_WINDOW, MocapBody
from .connections import RosConnections


def mocap_topic(mocap_id: int) -> str:
    """The relay topic of one rigid body.

    * Its pose is already calibrated and in the Z-up world frame; don't transform it again.
    """
    return f"/mocap/rigid_body/id_{mocap_id}/pose"


def subscribe_mocap(ros: RosConnections, mocap_id: int, body: MocapBody,
                    now: Callable[[], float]) -> None:
    """Keep `body` updated from rigid body `mocap_id`.

    Args:
        ros: The connections the subscription belongs to.
        mocap_id: Rigid-body id in the mocap system.
        body: The measured state to write into.
        now: Current ROS time in seconds, for when a sample arrived.
    """
    ros.subscription(MocapRigidBodyPose, mocap_topic(mocap_id),
                     lambda message: store_sample(body, message, now()))


def store_sample(body: MocapBody, message: MocapRigidBodyPose, now: float) -> None:
    """Write one mocap sample into `body`; an invalid one only clears `tracked`, keeping the last valid pose.

    Args:
        body: The body's measured state, changed in place.
        message: The relay's sample, stamped with when the cameras captured it.
        now: ROS time of arrival, seconds; also the capture time of a sample without a stamp.
    """
    body.tracked = bool(message.pose_valid)
    # * Kept so the mocap chip can say why a pose is invalid.
    body.tracking_valid = bool(message.tracking_valid)
    body.marker_error = float(message.marker_error)
    if body.tracked:
        p, q = message.pose.position, message.pose.orientation
        body.position = np.array([p.x, p.y, p.z])
        body.orientation = np.array([q.x, q.y, q.z, q.w])
        stamp = message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec
        # ! Capture time, so speeds and tracking errors use when the pose was true, not when it arrived.
        body.last_fix_time = stamp if stamp > 0.0 else now
        body.marker_errors.append((now, body.marker_error))
    while body.marker_errors and body.marker_errors[0][0] < now - MARKER_ERROR_WINDOW:
        body.marker_errors.popleft()
    body.last_update_time = now
