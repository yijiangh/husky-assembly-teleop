"""The mocap chip (loss turns it red at once, amber follows the mean marker error), and when a pose is lost."""

from __future__ import annotations

import pytest
from crl_husky_msgs.msg import MocapRigidBodyPose

from husky_assembly_teleop.robot_interface.mocap import store_sample
from husky_assembly_teleop.robot_interface.mocap_state import MARKER_ERROR_WINDOW, POSE_TIMEOUT, pose_lost
from husky_assembly_teleop.world.checks import BAD, GOOD, WARN
from husky_assembly_teleop.world.measured import TrackedObject
from husky_assembly_teleop.world.mocap import MARKER_ERROR_WARN, mocap_check

pytestmark = pytest.mark.safety


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


def _store(body: TrackedObject, valid: bool, now: float) -> None:
    """Store one sample arriving at `now`."""
    store_sample(body, MocapRigidBodyPose(pose_valid=valid, tracking_valid=valid), now)


def test_pose_lost_after_the_timeout_without_a_valid_sample():
    """A dropped frame is forgiven; invalid samples or a silent relay for over POSE_TIMEOUT lose the pose."""
    body = TrackedObject(name="probe", mocap_id=1)
    assert "no message" in pose_lost(1, body, 10.0)
    _store(body, True, 10.0)
    assert pose_lost(1, body, 10.0) == ""
    _store(body, False, 10.01)
    assert pose_lost(1, body, 10.01) == ""
    _store(body, False, 10.0 + POSE_TIMEOUT + 0.01)
    assert "lost by mocap" in pose_lost(1, body, 10.0 + POSE_TIMEOUT + 0.01)
    _store(body, True, 11.0)
    assert body.tracked and "no valid mocap pose" in pose_lost(1, body, 11.0 + POSE_TIMEOUT + 0.01)


def test_pose_lost_once_the_newest_pose_was_captured_too_long_ago():
    """The relay republishes a frozen frame as valid after the stream stops: arriving now, it is still lost."""
    body = TrackedObject(name="probe", mocap_id=1)
    message = MocapRigidBodyPose(pose_valid=True, tracking_valid=True)
    message.header.stamp.sec = 10
    store_sample(body, message, 10.0 + POSE_TIMEOUT - 0.01)
    assert pose_lost(1, body, 10.0 + POSE_TIMEOUT - 0.01) == ""
    store_sample(body, message, 10.0 + POSE_TIMEOUT + 0.01)
    assert "captured" in pose_lost(1, body, 10.0 + POSE_TIMEOUT + 0.01)


def test_pose_lost_without_a_mocap_id():
    """A body with no rigid body configured never has a pose to drive from."""
    body = TrackedObject(name="probe", mocap_id=None)
    _store(body, True, 10.0)
    assert "no mocap id" in pose_lost(None, body, 10.0)
