"""The relay's frames, calibrations, stamps and validity, without ROS or NatNet: what a robot's pose means."""

from __future__ import annotations

import threading
import time
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
from robot_fakes import Log, Publisher
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.world.calibration_sessions import Calibration
from husky_assembly_teleop.mocap_relay.node import (MARKER_OCCLUDED, MARKER_POINT_CLOUD_SOLVED, MocapRelay,
                                                    apply_base_calibration)

pytestmark = pytest.mark.safety

#: The rotation NatNet's Y-up frame takes to the Z-up world (+90 degrees about x).
Y_UP_TO_Z_UP = Rotation.from_euler("x", 90.0, degrees=True)
STALE_AFTER = 1.0


def relay(calibrations=None, exclude=(), frequency: int = 0) -> MocapRelay:
    """A relay without ROS or NatNet; `frequency` is NatNet's hi-res clock rate (0: no hi-res stamps)."""
    node = MocapRelay.__new__(MocapRelay)
    log = Log()
    node.get_logger = lambda: log
    node.create_publisher = lambda _type, topic, _depth: Publisher()
    node.exclude_ids, node._calibrations, node._logged_uncalibrated = set(exclude), dict(calibrations or {}), set()
    node._pending_samples, node._samples, node._samples_lock, node._rb_publishers = {}, {}, threading.Lock(), {}
    node._warned_no_markers = node._warned_no_hires_stamps = node._logged_latency = False
    node.mocap_client = SimpleNamespace(get_high_res_clock_frequency=lambda: frequency)
    node.topic_prefix, node.frame_id = "/mocap/rigid_body", "mocap_world"
    node.streaming_stale_timeout_sec, node.marker_error_valid_threshold = STALE_AFTER, 0.005
    node._last_frame_mono_sec = time.monotonic()
    return node


def frame(node: MocapRelay, rigid_body_id: int, position, quaternion, tracking: bool = True, error: float = 0.001,
          markers=None, **stamps) -> None:
    """One NatNet frame with one rigid body (Y-up), its labeled markers, then the frame's end."""
    node._receive_rigid_body_frame(rigid_body_id, tuple(position), tuple(quaternion), tracking, error)
    if markers is not None:
        node._receive_labeled_markers({rigid_body_id: markers})
    node._receive_mocap_frame({"labeled_marker_count": len(markers or {}), **stamps})


def published(node: MocapRelay, rigid_body_id: int):
    """Publish once and return the last message of `rigid_body_id`."""
    node._publish_cached_poses()
    return node._rb_publishers[rigid_body_id].sent[-1]


def pose_of(message) -> tuple[np.ndarray, np.ndarray]:
    """Position and quaternion (x, y, z, w) of a MocapRigidBodyPose."""
    p, q = message.pose.position, message.pose.orientation
    return np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])


def same_rotation(q1, q2) -> bool:
    """Whether two quaternions turn the same way, to a microradian."""
    return (Rotation.from_quat(q1).inv() * Rotation.from_quat(q2)).magnitude() < 1e-6


@pytest.mark.parametrize("seed", range(5))
def test_y_up_poses_are_turned_into_the_z_up_world(seed):
    """Position and orientation equal a +90 degree turn about x of NatNet's Y-up pose."""
    rng = np.random.default_rng(seed)
    position, rotation = rng.normal(size=3), Rotation.random(random_state=seed)
    node = relay()
    frame(node, 7, position, rotation.as_quat())
    got_position, got_quaternion = pose_of(published(node, 7))
    np.testing.assert_allclose(got_position, Y_UP_TO_Z_UP.apply(position), atol=1e-12)
    assert same_rotation(got_quaternion, (Y_UP_TO_Z_UP * rotation * Y_UP_TO_Z_UP.inv()).as_quat())


@pytest.mark.parametrize("seed", range(5))
def test_a_calibrated_body_is_published_as_its_base_footprint(seed):
    """world_from_base = world_from_mocap * base_mocap_from_base, for arbitrary poses and calibrations."""
    rng = np.random.default_rng(seed)
    body_p, body_r = rng.normal(size=3), Rotation.random(random_state=seed)
    offset_p, offset_r = rng.normal(size=3) * 0.2, Rotation.random(random_state=seed + 100)
    calibration = Calibration(mocap_id=3, robot_serial="a200-0804", position=tuple(offset_p),
                              orientation=tuple(offset_r.as_quat()), source="test", sha256="", measured=())
    position, quaternion = apply_base_calibration(calibration, tuple(body_p), tuple(body_r.as_quat()))
    np.testing.assert_allclose(position, body_p + body_r.apply(offset_p), atol=1e-12)
    assert same_rotation(quaternion, (body_r * offset_r).as_quat())


def test_only_the_calibrated_body_is_moved_to_its_base():
    """A robot's body gets its calibration; another body is published as Motive gives it, logged once."""
    calibration = Calibration(mocap_id=3, robot_serial="a200-0804", position=(0.1, 0.0, -0.3),
                              orientation=(0.0, 0.0, 0.0, 1.0), source="test", sha256="", measured=())
    node = relay({3: calibration})
    for _ in range(2):
        frame(node, 3, (0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        frame(node, 9, (0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    np.testing.assert_allclose(pose_of(published(node, 3))[0], (0.1, 0.0, 0.7), atol=1e-12)
    np.testing.assert_allclose(pose_of(published(node, 9))[0], (0.0, 0.0, 1.0), atol=1e-12)
    assert node.get_logger().text("info").count("rigid body 9: no robot's") == 1


def test_excluded_bodies_are_never_published():
    """The simulated robot's own body (exclude_ids) gives no pose and no markers."""
    node = relay(exclude=(5,))
    frame(node, 5, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), markers={1: {"pos": (0, 0, 0), "error": 0.0, "param": 2}})
    node._publish_cached_poses()
    assert 5 not in node._rb_publishers


@pytest.mark.parametrize("tracking, error, age, valid", [
    (True, 0.001, 0.0, True),
    (False, 0.001, 0.0, False),
    (True, 0.010, 0.0, False),
    (True, 0.001, STALE_AFTER + 0.1, False),
])
def test_pose_valid_needs_tracking_a_small_error_and_a_recent_frame(tracking, error, age, valid):
    """Lost by Motive, a marker error over the threshold, or no frame for STALE_AFTER: published invalid."""
    node = relay()
    frame(node, 7, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), tracking, error)
    node._samples[7] = replace(node._samples[7], received_mono_sec=time.monotonic() - age)
    assert published(node, 7).pose_valid is valid


def test_the_stamp_is_the_capture_time_and_stays_when_republished():
    """Arrival time less Motive's latency from its hi-res stamps; a pose republished keeps its stamp."""
    node = relay(frequency=1_000_000)
    arrival = 1_700_000_000_123_000_000
    frame(node, 7, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), receive_time_ns=arrival,
          stamp_camera_mid_exposure=1_000_000, stamp_transmit=1_008_000)
    first = published(node, 7).header.stamp
    second = published(node, 7).header.stamp
    expected = arrival - 8_000_000
    assert (first.sec, first.nanosec) == (second.sec, second.nanosec) == (expected // 10**9, expected % 10**9)


@pytest.mark.parametrize("stamps", [
    {},
    {"stamp_camera_mid_exposure": 1_000_000, "stamp_transmit": 1_900_000},
    {"stamp_camera_mid_exposure": 1_000_000, "stamp_transmit": 900_000},
])
def test_without_plausible_hi_res_stamps_the_arrival_time_is_used(stamps):
    """No hi-res stamps, or a latency over half a second or negative: stamped with the arrival time."""
    node = relay(frequency=1_000_000 if stamps else 0)
    arrival = 1_700_000_000_000_000_000
    frame(node, 7, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), receive_time_ns=arrival, **stamps)
    stamp = published(node, 7).header.stamp
    assert stamp.sec * 10**9 + stamp.nanosec == arrival


def test_markers_are_turned_to_z_up_and_flagged_when_filled_in():
    """Raw markers get the same remap as poses (no calibration); occluded or unsolved ones are not measured."""
    node = relay()
    markers = {2: {"pos": (0.0, 1.0, 2.0), "error": 0.0004, "param": MARKER_POINT_CLOUD_SOLVED},
               1: {"pos": (1.0, 0.0, 0.0), "error": 0.0, "param": MARKER_POINT_CLOUD_SOLVED | MARKER_OCCLUDED},
               3: {"pos": (0.0, 0.0, 0.0), "error": 0.0, "param": 0}}
    frame(node, 7, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), markers=markers)
    message = published(node, 7)
    assert list(message.marker_ids) == [1, 2, 3]
    assert [(p.x, p.y, p.z) for p in message.marker_positions][1] == (0.0, -2.0, 1.0)
    assert list(message.marker_measured) == [False, True, False]


def test_a_simulated_body_is_calibrated_and_stamped_like_motives():
    """source:=sim: a raw Z-up body from the simulator gets its robot's calibration and keeps the simulator's stamp."""
    from builtin_interfaces.msg import Time
    from crl_husky_msgs.msg import MocapRigidBodyPose
    calibration = Calibration(mocap_id=3, robot_serial="a200-0804", position=(0.1, 0.0, -0.3),
                              orientation=(0.0, 0.0, 0.0, 1.0), source="test", sha256="", measured=())
    node = relay({3: calibration})
    message = MocapRigidBodyPose(pose_valid=True, tracking_valid=True, marker_error=0.0003)
    message.header.stamp = Time(sec=1_700_000_000, nanosec=5)
    message.pose.position.x, message.pose.position.y, message.pose.position.z = 1.0, 2.0, 0.5
    node._receive_sim_body(3, message)
    sent = published(node, 3)
    position, quaternion = pose_of(sent)
    np.testing.assert_allclose(position, (1.1, 2.0, 0.2), atol=1e-12)
    assert sent.pose_valid and (sent.header.stamp.sec, sent.header.stamp.nanosec) == (1_700_000_000, 5)
