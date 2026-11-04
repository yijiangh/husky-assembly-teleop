"""The fixed base pose: what it publishes, and where."""

import numpy as np
import pytest
from builtin_interfaces.msg import Time

from husky_assembly_teleop.mocap_relay.fixed_base import DEFAULT_BODIES, LAB_FRAME, fixed_pose_message, parse_pose
from husky_assembly_teleop.robot_interface.mocap import mocap_topic, store_sample
from husky_assembly_teleop.robot_interface.mocap_state import pose_lost
from husky_assembly_teleop.world.measured import TrackedObject


def test_the_pose_text_reads_metres_and_a_yaw_in_degrees():
    """Fewer values are 0; a yaw turns about z; commas work too."""
    origin = parse_pose("0 0 0 0")
    np.testing.assert_allclose(origin.position, (0.0, 0.0, 0.0))
    np.testing.assert_allclose(origin.orientation, (0.0, 0.0, 0.0, 1.0))
    turned = parse_pose("1.0, 0.5, 0, 90")
    np.testing.assert_allclose(turned.position, (1.0, 0.5, 0.0))
    np.testing.assert_allclose(turned.orientation, (0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)), atol=1e-12)
    assert parse_pose("").position == (0.0, 0.0, 0.0)
    with pytest.raises(ValueError):
        parse_pose("1 2 3 4 5")


def test_the_message_is_a_valid_tracked_pose_the_monitor_takes_as_live():
    """Valid, tracked, no marker error, in the lab frame, stamped: stored, the base counts as live, not lost."""
    message = fixed_pose_message(parse_pose("1 2 0 90"), Time(sec=100, nanosec=0))
    assert message.pose_valid and message.tracking_valid and message.marker_error == 0.0
    assert message.header.frame_id == LAB_FRAME and message.header.stamp.sec == 100
    assert (message.pose.position.x, message.pose.position.y, message.pose.position.z) == (1.0, 2.0, 0.0)
    assert list(message.marker_ids) == [] and list(message.marker_positions) == []
    body = TrackedObject(name="cindy", mocap_id=DEFAULT_BODIES[0])
    store_sample(body, message, 100.1)
    assert body.tracked and pose_lost(DEFAULT_BODIES[0], body, 100.2) == ""
    assert mocap_topic(DEFAULT_BODIES[0]) == "/mocap/rigid_body/id_1860/pose"
