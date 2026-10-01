"""
Mocap on the ROS side: where the relay publishes a rigid body, and how a sample is
stored. The same for robot bases and tracked objects.

! Keep this the only place that reads MocapRigidBodyPose, so a base and an
  object can never disagree about what "tracked" means. Judging a fix is
  `world.mocap.mocap_check`.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Callable

import numpy as np
from crl_husky_msgs.msg import MocapRigidBodyPose

from .connections import RosConnections

if TYPE_CHECKING:
    from ..world.mocap import MocapBody


def mocap_topic(mocap_id: int) -> str:
    """The relay topic of one rigid body.

    * The relay's pose is already calibrated and in the Z-up 'rhino' world
      frame. Use it as is; transforming again would apply it twice.
    """
    return f"/mocap/rigid_body/id_{mocap_id}/pose"


def subscribe_mocap(ros: RosConnections, mocap_id: int, body: "MocapBody",
                    now: Callable[[], float]) -> None:
    """Keep `body` updated from rigid body `mocap_id`.

    Args:
        ros: The connections the subscription belongs to (destroyed with them).
        mocap_id: Rigid-body id in the mocap system.
        body: The measured state to write into.
        now: Current ROS time in seconds, stamped on each sample.
    """
    ros.subscription(MocapRigidBodyPose, mocap_topic(mocap_id),
                     lambda message: store_sample(body, message, now()))


def store_sample(body: "MocapBody", message: MocapRigidBodyPose, now: float) -> None:
    """Write one mocap sample into `body`; an invalid one only clears `tracked`, keeping the last valid pose.

    Args:
        body: The body's measured state, changed in place.
        message: The relay's sample.
        now: ROS time of arrival, seconds.
    """
    body.tracked = bool(message.pose_valid)
    # * Kept so the mocap chip can say why a pose is invalid.
    body.tracking_valid = bool(message.tracking_valid)
    body.marker_error = float(message.marker_error)
    if body.tracked:
        p, q = message.pose.position, message.pose.orientation
        body.position = np.array([p.x, p.y, p.z])
        body.orientation = np.array([q.x, q.y, q.z, q.w])
        body.last_fix_time = now
    body.last_update_time = now
