"""The relay's frames, calibrations, stamps, validity and sources, without ROS or NatNet: what a pose means."""

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
from husky_assembly_teleop.mocap_relay.node import (FAKE_MOTIVE, MARKER_OCCLUDED, MARKER_POINT_CLOUD_SOLVED, MOTIVE,
                                                    SIM_MOTIVE_PREFIX, TRUTH, TRUTH_PREFIX, MocapRelay,
                                                    apply_base_calibration, remove_base_calibration)

pytestmark = pytest.mark.safety

#: The rotation NatNet's Y-up frame takes to the Z-up world (+90 degrees about x).
Y_UP_TO_Z_UP = Rotation.from_euler("x", 90.0, degrees=True)
STALE_AFTER = 1.0


def relay(calibrations=None, sim_calibrations=None, frequency: int = 0, defaults=None) -> MocapRelay:
    """A relay without ROS or NatNet, outside the robots' domain.

    `frequency` is NatNet's hi-res clock rate (0: no hi-res stamps).
    """
    node = MocapRelay.__new__(MocapRelay)
    log = Log()
    node.get_logger = lambda: log
    node.create_publisher = lambda _type, topic, _depth: Publisher()
    node._calibrations = {MOTIVE: dict(calibrations or {}), FAKE_MOTIVE: dict(sim_calibrations or {})}
    node._logged_uncalibrated, node._defaults = set(), dict(defaults or {})
    node._pending_samples, node._samples, node._samples_lock, node._rb_publishers = {}, {}, threading.Lock(), {}
    node._sim_samples, node._sim_topics, node._sources = {FAKE_MOTIVE: {}, TRUTH: {}}, set(), {}
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


def calibration(position) -> Calibration:
    """A calibration of rigid body 3 that only moves base_footprint by `position` in the body's frame."""
    return Calibration(mocap_id=3, robot_serial="a200-0804", position=tuple(position),
                       orientation=(0.0, 0.0, 0.0, 1.0), source="test", sha256="", measured=())


def sim_body(node: MocapRelay, source: str, rigid_body_id: int, position, stamp=(1_700_000_000, 5)) -> None:
    """One simulated Z-up body from `source`, unturned, stamped with when it was true."""
    from builtin_interfaces.msg import Time
    from crl_husky_msgs.msg import MocapRigidBodyPose
    message = MocapRigidBodyPose(pose_valid=True, tracking_valid=True, marker_error=0.0003)
    message.header.stamp = Time(sec=stamp[0], nanosec=stamp[1])
    message.pose.position.x, message.pose.position.y, message.pose.position.z = position
    message.pose.orientation.w = 1.0
    node._receive_sim_body(source, rigid_body_id, message)


def age(samples: dict, rigid_body_id: int, seconds: float) -> None:
    """Make a body's sample `seconds` older than now."""
    samples[rigid_body_id] = replace(samples[rigid_body_id], received_mono_sec=time.monotonic() - seconds)


@pytest.mark.parametrize("seed", range(5))
def test_removing_a_calibration_undoes_applying_it(seed):
    """The body back from its base_footprint, for arbitrary poses and calibrations."""
    rng = np.random.default_rng(seed)
    body_p, body_r = tuple(rng.normal(size=3)), Rotation.random(random_state=seed)
    offset = Calibration(3, "a200-0804", tuple(rng.normal(size=3) * 0.2),
                         tuple(Rotation.random(random_state=seed + 100).as_quat()), "test", "", ())
    position, quaternion = remove_base_calibration(offset, *apply_base_calibration(offset, body_p, body_r.as_quat()))
    np.testing.assert_allclose(position, body_p, atol=1e-12)
    assert same_rotation(quaternion, body_r.as_quat())


def test_a_fake_motives_base_gets_the_simulated_calibration_over_the_default_and_keeps_its_stamp():
    """A fake Motive's base: the default off, the simulated session's on (never the robots'), the truth's stamp kept."""
    node = relay({3: calibration((0.5, 0.5, 0.5))}, {3: calibration((0.1, 0.0, -0.3))},
                 defaults={3: calibration((0.1, 0.0, -0.2))})
    sim_body(node, FAKE_MOTIVE, 3, (1.0, 2.0, 0.5))
    sent = published(node, 3)
    np.testing.assert_allclose(pose_of(sent)[0], (1.0, 2.0, 0.4), atol=1e-12)
    assert sent.pose_valid and (sent.header.stamp.sec, sent.header.stamp.nanosec) == (1_700_000_000, 5)


def test_with_the_default_applied_a_fake_motives_base_is_unchanged():
    """No simulated session: the default comes off and goes on again, so the base stays where the fake Motive put it."""
    default = calibration((0.06, 0.01, -0.48))
    node = relay(sim_calibrations={3: default}, defaults={3: default})
    sim_body(node, FAKE_MOTIVE, 3, (1.0, 2.0, 0.0))
    np.testing.assert_allclose(pose_of(published(node, 3))[0], (1.0, 2.0, 0.0), atol=1e-12)


def test_a_simulators_truth_is_published_as_it_is():
    """A simulator's true base_footprint needs no calibration, whatever is applied to the robot's body."""
    node = relay({3: calibration((0.5, 0.5, 0.5))}, {3: calibration((0.1, 0.0, -0.3))})
    sim_body(node, TRUTH, 3, (1.0, 2.0, 0.0))
    np.testing.assert_allclose(pose_of(published(node, 3))[0], (1.0, 2.0, 0.0), atol=1e-12)


def test_a_simulated_body_stands_in_for_motives_while_it_streams():
    """Fake Motive over truth over Motive, each only while it streams; then the next comes back, logged."""
    node = relay()
    frame(node, 3, (5.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert pose_of(published(node, 3))[0][0] == 5.0
    sim_body(node, TRUTH, 3, (1.0, 0.0, 0.0))
    assert pose_of(published(node, 3))[0][0] == 1.0
    sim_body(node, FAKE_MOTIVE, 3, (2.0, 0.0, 0.0))
    assert pose_of(published(node, 3))[0][0] == 2.0
    age(node._sim_samples[FAKE_MOTIVE], 3, STALE_AFTER + 0.1)
    assert pose_of(published(node, 3))[0][0] == 1.0
    age(node._sim_samples[TRUTH], 3, STALE_AFTER + 0.1)
    frame(node, 3, (5.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    message = published(node, 3)
    assert pose_of(message)[0][0] == 5.0 and message.pose_valid
    assert "rigid body 3: now from the Motive, was the simulator's truth" in node.get_logger().text("info")


def test_with_every_source_quiet_the_last_heard_is_published_invalid():
    """No source streams: the one heard last is published, marked invalid."""
    node = relay()
    frame(node, 3, (5.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    sim_body(node, FAKE_MOTIVE, 3, (2.0, 0.0, 0.0))
    age(node._samples, 3, STALE_AFTER + 0.5)
    age(node._sim_samples[FAKE_MOTIVE], 3, STALE_AFTER + 0.2)
    message = published(node, 3)
    assert pose_of(message)[0][0] == 2.0 and not message.pose_valid
    assert node._get_streaming_ids(time.monotonic()) == []


def test_a_simulated_body_never_commits_a_half_received_motive_frame():
    """Motive's bodies wait for their frame's end, also while simulated bodies arrive in between."""
    node = relay()
    node._receive_rigid_body_frame(7, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), True, 0.001)
    sim_body(node, TRUTH, 3, (1.0, 0.0, 0.0))
    assert 7 not in node._samples and 7 in node._pending_samples
    node._receive_mocap_frame({"labeled_marker_count": 0})
    assert 7 in node._samples and node._get_streaming_ids(time.monotonic()) == [3, 7]


def test_simulated_inputs_are_found_by_their_topics_once_each():
    """Every truth and fake Motive topic is subscribed to once, with its source and id; other topics are left."""
    node = relay()
    subscribed = []
    node.create_subscription = lambda _type, topic, callback, _depth: subscribed.append((topic, callback))
    topics = [f"{TRUTH_PREFIX}/id_1840/pose", f"{SIM_MOTIVE_PREFIX}/id_1866/pose", "/mocap/rigid_body/id_1840/pose",
              f"{TRUTH_PREFIX}/id_x/pose"]
    node.get_topic_names_and_types = lambda: [(topic, ["crl_husky_msgs/msg/MocapRigidBodyPose"]) for topic in topics]
    node._look_for_simulated_inputs()
    node._look_for_simulated_inputs()
    assert [topic for topic, _ in subscribed] == topics[:2]
    sim_message = SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=0.0, z=0.0),
                                                       orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)),
                                  header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
                                  tracking_valid=True, marker_error=0.0)
    for _, callback in subscribed:
        callback(sim_message)
    assert set(node._sim_samples[TRUTH]) == {1840} and set(node._sim_samples[FAKE_MOTIVE]) == {1866}
