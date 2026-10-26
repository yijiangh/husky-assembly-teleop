"""The raw markers a relay message leaves on a body: residuals, sizes and the capture time ride with them."""

from __future__ import annotations

import numpy as np
import pytest
from crl_husky_msgs.msg import MocapRigidBodyPose
from geometry_msgs.msg import Point

from husky_assembly_teleop.robot_interface.mocap import store_sample
from husky_assembly_teleop.world.measured import TrackedObject

# * The bar fit steers the servo on these markers: a wrong residual, size or age would pass a bad fit.
pytestmark = pytest.mark.safety

#: Two markers, the second filled in by Motive.
IDS, RESIDUALS, MEASURED = [1, 2], [0.001, 0.002], [True, False]
POSITIONS = [Point(x=1.0, y=0.0, z=0.5), Point(x=0.0, y=2.0, z=0.5)]


class MessageWithSizes(MocapRigidBodyPose):
    """The message as a newer relay fills it: `marker_sizes` beside the fields the message has today."""

    def __init__(self, sizes: list[float], **fields):
        """Build the message from `fields`, then add the sizes the real class has no slot for yet."""
        super().__init__(**fields)
        self.marker_sizes = sizes


def _stored(message: MocapRigidBodyPose, now: float) -> TrackedObject:
    """Store one sample arriving at `now` on a fresh body and give the body back."""
    body = TrackedObject(name="probe", mocap_id=1)
    store_sample(body, message, now)
    return body


def test_residuals_sizes_and_capture_time_are_kept_with_the_markers():
    """Every per-marker list lands as a float array in marker order; the capture time is the header stamp."""
    message = MessageWithSizes([0.019, 0.021], marker_ids=IDS, marker_positions=POSITIONS,
                               marker_residuals=RESIDUALS, marker_measured=MEASURED)
    message.header.stamp.sec = 10
    markers = _stored(message, 10.2).markers
    assert markers.ids.tolist() == IDS and markers.measured.tolist() == MEASURED
    assert markers.positions.shape == (2, 3) and markers.positions[1].tolist() == [0.0, 2.0, 0.5]
    assert np.allclose(markers.residuals, RESIDUALS, atol=1e-7)
    assert np.allclose(markers.sizes, [0.019, 0.021], atol=1e-7)
    assert markers.capture_time == 10.0


def test_an_unstamped_sample_is_captured_when_it_arrived():
    """Without a header stamp the arrival time stands in for the capture time."""
    message = MocapRigidBodyPose(marker_ids=IDS, marker_positions=POSITIONS, marker_residuals=RESIDUALS,
                                 marker_measured=MEASURED)
    assert _stored(message, 12.5).markers.capture_time == 12.5


def test_a_list_of_another_length_is_dropped():
    """A relay that sends fewer residuals or sizes than markers (an older one) leaves None, not a short array."""
    message = MessageWithSizes([0.019], marker_ids=IDS, marker_positions=POSITIONS, marker_residuals=[0.001],
                               marker_measured=MEASURED)
    markers = _stored(message, 10.0).markers
    assert markers.residuals is None and markers.sizes is None
    assert markers.ids.tolist() == IDS


def test_a_message_without_sizes_leaves_them_none():
    """Today's message has no `marker_sizes`: the residuals are kept and the sizes are None, nothing raises."""
    message = MocapRigidBodyPose(marker_ids=IDS, marker_positions=POSITIONS, marker_residuals=RESIDUALS,
                                 marker_measured=MEASURED)
    markers = _stored(message, 10.0).markers
    assert markers.sizes is None
    assert np.allclose(markers.residuals, RESIDUALS, atol=1e-7)
