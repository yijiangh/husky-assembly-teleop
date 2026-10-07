"""The mocap chip: loss turns it red at once, amber follows the rolling mean marker error."""

from __future__ import annotations

from crl_husky_msgs.msg import MocapRigidBodyPose

from husky_assembly_teleop.robot_interface.mocap import store_sample
from husky_assembly_teleop.world.checks import BAD, GOOD, WARN
from husky_assembly_teleop.world.measured import TrackedObject
from husky_assembly_teleop.world.mocap import MARKER_ERROR_WARN, MARKER_ERROR_WINDOW, mocap_check


def _level(body: TrackedObject, valid: bool, marker_error: float, now: float) -> int:
    """Store one sample arriving at `now`, then judge the body."""
    message = MocapRigidBodyPose(pose_valid=valid, tracking_valid=valid, marker_error=marker_error)
    store_sample(body, message, now)
    return mocap_check("probe", body.mocap_id, body, now).level


def test_warning_follows_the_rolling_mean():
    """A lone spike stays green; amber once the mean over MARKER_ERROR_WINDOW is high, green once it drops."""
    body = TrackedObject(name="probe", mocap_id=1)
    low, high = MARKER_ERROR_WARN / 2, MARKER_ERROR_WARN * 3
    step = MARKER_ERROR_WINDOW / 10
    for i in range(10):
        assert _level(body, True, low, 10.0 + i * step) == GOOD
    assert _level(body, True, high, 11.0) == GOOD
    t = 11.0
    for _ in range(10):
        t += step
        level = _level(body, True, high, t)
    assert level == WARN
    for _ in range(10):
        t += step
        level = _level(body, True, low, t)
    assert level == GOOD


def test_loss_turns_red_at_once():
    """A lost body is red on its first sample and back as soon as it is tracked."""
    body = TrackedObject(name="probe", mocap_id=1)
    assert _level(body, True, 0.0, 10.0) == GOOD
    assert _level(body, False, 0.0, 10.01) == BAD
    assert _level(body, True, 0.0, 10.02) == GOOD
