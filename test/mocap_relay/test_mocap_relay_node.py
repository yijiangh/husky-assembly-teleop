"""The relay's frames, calibrations, stamps, validity and sources, without ROS or NatNet: what a pose means."""

from __future__ import annotations

import json
import time
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
from body_definition_fixtures import MARKERS, body, moved
from mocap_frame_fixtures import (LAB_ANCHORS, LAB_CAMERAS, LOOSE, SHIFT, TURN, anchors_after, cameras_after, export,
                                  seen_after)
from robot_fakes import Log, Publisher
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose, compose
from husky_assembly_teleop.mocap_relay.body_definitions import Reply
from husky_assembly_teleop.mocap_relay.frame_log import FrameLog
from husky_assembly_teleop.robot_interface.mocap_definitions import (MocapCamera, cameras_json, definitions_json,
                                                                     read_cameras, read_definitions)
from husky_assembly_teleop.robot_interface.mocap_relay import MocapRelayState, floor_calibration_state
from husky_assembly_teleop.world.calibration_sessions import (SESSIONS_FOLDER, Calibration, default_calibration,
                                                              sessions_folder)
from husky_assembly_teleop.world.frame_follower import FrameFollower
from husky_assembly_teleop.world.mocap_frame import FLOOR_SUFFIX, TRUSS_CAMERAS, truss_names
from husky_assembly_teleop.mocap_relay.node import (FAKE_MOTIVE, MARKER_OCCLUDED, MARKER_POINT_CLOUD_SOLVED,
                                                    MARKER_UNLABELED, SIM_MOTIVE_PREFIX, TRUTH, TRUTH_PREFIX,
                                                    MocapRelay, apply_base_calibration, remove_base_calibration,
                                                    simulated_mode, z_up_body)

pytestmark = pytest.mark.safety

#: The rotation NatNet's Y-up frame takes to the Z-up world (+90 degrees about x).
Y_UP_TO_Z_UP = Rotation.from_euler("x", 90.0, degrees=True)
STALE_AFTER = 1.0


def relay(calibrations=None, simulated: bool = False, frequency: int = 0, defaults=None,
          drive_root=None) -> MocapRelay:
    """A relay without ROS or NatNet, reading Motive or with `simulated` a simulation (sim:=true), as constructed.

    `frequency` is NatNet's hi-res clock rate (0: no hi-res stamps); `drive_root` the Drive's folder, or None.
    """
    node = MocapRelay.__new__(MocapRelay)
    log = Log()
    node.get_logger = lambda: log
    node.create_publisher = lambda _type, topic, _depth: Publisher()
    node.get_fully_qualified_name = lambda: "/mocap_relay"
    node.topic_prefix, node.streaming_stale_timeout_sec, node.marker_error_valid_threshold = (
        "/mocap/rigid_body", STALE_AFTER, 0.005)
    node.drive_root, node.simulated, node._sessions = drive_root, simulated, sessions_folder(simulated, 70)
    node._start_state(TRUSS_CAMERAS)
    node._calibrations = dict(calibrations or {})
    node._defaults = dict(defaults or {})
    node.mocap_client = SimpleNamespace(get_high_res_clock_frequency=lambda: frequency)
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
    assert node.get_logger().text("info").count("rigid body 9: no robot's, uncalibrated") == 1


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
    node = relay({3: calibration((0.1, 0.0, -0.3))}, simulated=True, defaults={3: calibration((0.1, 0.0, -0.2))})
    sim_body(node, FAKE_MOTIVE, 3, (1.0, 2.0, 0.5))
    sent = published(node, 3)
    np.testing.assert_allclose(pose_of(sent)[0], (1.0, 2.0, 0.4), atol=1e-12)
    assert sent.pose_valid and (sent.header.stamp.sec, sent.header.stamp.nanosec) == (1_700_000_000, 5)


def test_with_the_default_applied_a_fake_motives_base_is_unchanged():
    """No simulated session: the default comes off and goes on again, so the base stays where the fake Motive put it."""
    default = calibration((0.06, 0.01, -0.48))
    node = relay({3: default}, simulated=True, defaults={3: default})
    sim_body(node, FAKE_MOTIVE, 3, (1.0, 2.0, 0.0))
    np.testing.assert_allclose(pose_of(published(node, 3))[0], (1.0, 2.0, 0.0), atol=1e-12)


def test_a_simulators_truth_is_published_as_it_is():
    """A simulator's true base_footprint needs no calibration, whatever is applied to the robot's body."""
    node = relay({3: calibration((0.1, 0.0, -0.3))}, simulated=True)
    sim_body(node, TRUTH, 3, (1.0, 2.0, 0.0))
    np.testing.assert_allclose(pose_of(published(node, 3))[0], (1.0, 2.0, 0.0), atol=1e-12)


def test_in_a_simulation_a_fake_motive_stands_in_for_the_truth_while_it_streams():
    """Fake Motive over truth, each only while it streams; then the truth comes back, logged."""
    node = relay(simulated=True)
    sim_body(node, TRUTH, 3, (1.0, 0.0, 0.0))
    assert pose_of(published(node, 3))[0][0] == 1.0
    sim_body(node, FAKE_MOTIVE, 3, (2.0, 0.0, 0.0))
    assert pose_of(published(node, 3))[0][0] == 2.0
    age(node._sim_samples[FAKE_MOTIVE], 3, STALE_AFTER + 0.1)
    sim_body(node, TRUTH, 3, (1.0, 0.0, 0.0))
    message = published(node, 3)
    assert pose_of(message)[0][0] == 1.0 and message.pose_valid
    assert "rigid body 3: now from the simulator's truth, was the fake Motive" in node.get_logger().text("info")


@pytest.mark.parametrize("domain, sim", [("80", False), ("70", False), ("70", True)])
def test_a_simulations_relay_is_refused_in_the_robots_domain(monkeypatch, domain, sim):
    """sim:=true only in a domain of its own: there nothing may stand in for a robot."""
    monkeypatch.setenv("ROS_DOMAIN_ID", domain)
    assert simulated_mode(sim) is sim
    monkeypatch.setenv("ROS_DOMAIN_ID", "80")
    with pytest.raises(RuntimeError, match="sim:=true refused"):
        simulated_mode(True)


@pytest.mark.parametrize("simulated", [False, True])
def test_a_relay_never_mixes_motive_with_a_simulation(simulated):
    """Motive's relay publishes only Motive's bodies, a simulation's only simulated ones; announced which it is."""
    node = relay(simulated=simulated)
    frame(node, 3, (5.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    sim_body(node, FAKE_MOTIVE, 4, (2.0, 0.0, 0.0))
    assert node._get_streaming_ids(time.monotonic()) == ([4] if simulated else [3])
    node._announce_calibrations()
    announced = json.loads(node._calibrations_publisher.sent[-1].data)
    assert announced["simulated"] is simulated and MocapRelayState().simulated is False


def test_with_every_source_quiet_the_last_heard_is_published_invalid():
    """No source streams: the one heard last is published, marked invalid."""
    node = relay(simulated=True)
    sim_body(node, TRUTH, 3, (5.0, 0.0, 0.0))
    sim_body(node, FAKE_MOTIVE, 3, (2.0, 0.0, 0.0))
    age(node._sim_samples[TRUTH], 3, STALE_AFTER + 0.5)
    age(node._sim_samples[FAKE_MOTIVE], 3, STALE_AFTER + 0.2)
    message = published(node, 3)
    assert pose_of(message)[0][0] == 2.0 and not message.pose_valid
    assert node._get_streaming_ids(time.monotonic()) == []


def test_simulated_inputs_are_found_by_their_topics_once_each():
    """Every truth and fake Motive topic is subscribed to once, with its source and id; other topics are left."""
    node = relay(simulated=True)
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


# * Motive's body definitions and cameras.

#: A camera as Motive gives it (Y-up).
CAMERA = MocapCamera("PrimeX 22 #72318", (-3.286, 2.188, 1.852), (-0.186, -0.646, -0.148, 0.725))


def camera_moved_up(metres: float) -> MocapCamera:
    """CAMERA moved up (Motive's +y) by `metres`."""
    return replace(CAMERA, position=(CAMERA.position[0], CAMERA.position[1] + metres, CAMERA.position[2]))


def test_motives_definitions_and_cameras_are_published_once_and_z_up():
    """The first reply is published on both topics, turned to Z-up like the poses; the same reply again is not."""
    node = relay()
    reply = Reply(1.0, [body()], [CAMERA])
    node._take_definitions(reply)
    node._take_definitions(reply)
    assert len(node._definitions_publisher.sent) == len(node._cameras_publisher.sent) == 1
    published_body = read_definitions(node._definitions_publisher.sent[0].data)[1866]
    np.testing.assert_allclose(published_body.array(), Y_UP_TO_Z_UP.apply(body().array()), atol=1e-12)
    camera = read_cameras(node._cameras_publisher.sent[0].data)[CAMERA.name]
    np.testing.assert_allclose(camera.position, Y_UP_TO_Z_UP.apply(CAMERA.position), atol=1e-12)
    rotation = Y_UP_TO_Z_UP * Rotation.from_quat(CAMERA.orientation) * Y_UP_TO_Z_UP.inv()
    assert same_rotation(camera.orientation, rotation.as_quat())
    assert "Motive defines 1 rigid bodies: [1866]" in node.get_logger().text("info")


def test_a_refined_body_is_published_and_warned_about():
    """A refine is a data change: a warning naming the body, and the new definition published."""
    node = relay()
    node._take_definitions(Reply(1.0, [body()]))
    node._take_definitions(Reply(3.0, [body(moved(noise=0.3e-3))]))
    assert len(node._definitions_publisher.sent) == 2
    assert "rigid body id 1866: refined" in node.get_logger().text("warn")


def test_a_renamed_body_is_published_and_logged_as_info():
    """A rename changes no data: info only."""
    node = relay()
    node._take_definitions(Reply(1.0, [body()]))
    node._take_definitions(Reply(3.0, [body(name="tool")]))
    assert len(node._definitions_publisher.sent) == 2 and "rigid body id 1866: renamed" in node.get_logger().text(
        "info") and node.get_logger().text("warn") == ""


def test_a_cameras_small_moves_are_published_and_warned_about_once_they_add_up():
    """Each reading publishes; a warning names the camera once it moved over a millimetre since it was last warned."""
    node = relay()
    for index in range(4):
        node._take_definitions(Reply(float(index), [], [camera_moved_up(0.0004 * index)]))
    assert len(node._cameras_publisher.sent) == 4
    assert node.get_logger().text("warn").count("BMR #72318: camera moved") == 1


def test_description_problems_are_warned_about_once():
    """The same problem in every reply is logged once."""
    node = relay()
    for index in range(3):
        node._take_definitions(Reply(float(index), [body()], problems=["camera 'x': read 20 of 28 bytes"]))
    assert node.get_logger().text("warn").count("read 20 of 28 bytes") == 1


def test_motive_is_asked_only_while_connected_and_its_newest_reply_is_taken():
    """Each poll takes the replies since the last and asks again, unless the client is not connected."""
    requests, connected = [], [False]
    replies = [[Reply(1.0, [body()]), Reply(2.0, [body(MARKERS * 1.5, streaming_id=1850)])]]
    node = relay()
    node.mocap_client = SimpleNamespace(take_replies=lambda: replies.pop() if replies else [],
                                        connected=lambda: connected[0], request=lambda: requests.append(True))
    node._poll_definitions()
    assert requests == [] and set(read_definitions(node._definitions_publisher.sent[-1].data)) == {1850}
    connected[0] = True
    node._poll_definitions()
    assert requests == [True] and len(node._definitions_publisher.sent) == 1


def built_for(definition) -> Calibration:
    """Rigid body 1866's calibration, built for `definition` (as Motive gives it, Y-up)."""
    return replace(calibration((0.1, 0.0, 0.0)), mocap_id=1866, body_definition=z_up_body(definition))


def announced_check(node: MocapRelay) -> dict:
    """The definition check of rigid body 1866 in the relay's last calibrations announcement."""
    entry, = json.loads(node._calibrations_publisher.sent[-1].data)["calibrations"]
    return entry["definition_check"]


def test_a_calibration_for_motives_body_definition_is_announced_as_matching():
    """Checked once Motive's definitions arrive: logged once, announced with the calibration."""
    node = relay(calibrations={1866: built_for(body())})
    node._take_definitions(Reply(1.0, [body()]))
    node._take_definitions(Reply(3.0, [body()]))
    assert announced_check(node) == {"status": "matches", "detail": "unchanged"}
    assert len(node._calibrations_publisher.sent) == 1
    assert node.get_logger().text("info").count("its calibration matches the body definition") == 1


def test_a_calibration_for_an_earlier_body_definition_is_reported_once_and_still_applied():
    """A refine since: a warning naming the body, announced wrong; the pose is still calibrated."""
    node = relay(calibrations={1866: built_for(body())})
    for index in range(2):
        node._take_definitions(Reply(float(index), [body(moved(noise=0.3e-3))]))
    assert announced_check(node)["status"] == "out_of_date" and "refined" in announced_check(node)["detail"]
    assert node.get_logger().text("warn").count("CALIBRATION FOR AN EARLIER BODY") == 1
    frame(node, 1866, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    np.testing.assert_allclose(pose_of(published(node, 1866))[0], (0.1, 0.0, 0.0), atol=1e-12)


def fake_announcement(bodies=(), cameras=()):
    """A fake Motive's announcements, as it publishes them (Z-up)."""
    return (SimpleNamespace(data=definitions_json(list(bodies), [], 1.0)),
            SimpleNamespace(data=cameras_json(list(cameras), 1.0)))


def test_a_simulations_relay_publishes_only_the_fake_motives_bodies_and_cameras():
    """Never Motive's, even if a reply came: the fake Motive's definitions and cameras, as they are."""
    node = relay(simulated=True)
    node._motive_definitions = ({1850: z_up_body(body(MARKERS * 1.5, 1850))}, {CAMERA.name: CAMERA}, ())
    fake_body = replace(z_up_body(body(MARKERS * 1.3)), name="sim body 1866")
    fake_camera = MocapCamera("Prime 13 #1", (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))
    definitions, cameras = fake_announcement([fake_body], [fake_camera])
    node._receive_sim_definitions(definitions)
    node._receive_sim_cameras(cameras)
    assert read_definitions(node._definitions_publisher.sent[-1].data) == {1866: fake_body}
    assert read_cameras(node._cameras_publisher.sent[-1].data) == {fake_camera.name: fake_camera}


def test_a_fake_motive_alone_is_enough():
    """A laptop simulation: the fake's announcements are published as they are; no cameras yet."""
    node = relay(simulated=True)
    fake_body = z_up_body(body())
    definitions, _cameras = fake_announcement([fake_body])
    node._receive_sim_definitions(definitions)
    assert read_definitions(node._definitions_publisher.sent[-1].data) == {1866: fake_body}
    assert read_cameras(node._cameras_publisher.sent[-1].data) == {}


def test_a_fake_motives_calibration_is_checked_against_the_fakes_body():
    """A simulated session built for the fake's body: announced matching, then wrong after the fake's refine."""
    built = replace(built_for(body()), mocap_id=1866)
    node = relay({1866: replace(built, body_definition=z_up_body(body()))}, simulated=True)
    node._receive_sim_definitions(fake_announcement([z_up_body(body())])[0])
    entry, = json.loads(node._calibrations_publisher.sent[-1].data)["calibrations"]
    assert entry["definition_check"]["status"] == "matches"
    node._receive_sim_definitions(fake_announcement([z_up_body(body(moved(noise=0.3e-3)))])[0])
    entry, = json.loads(node._calibrations_publisher.sent[-1].data)["calibrations"]
    assert entry["definition_check"]["status"] == "out_of_date"
    assert "rigid body 1866 of the fake Motive" in node.get_logger().text("warn")


# * The lab frame: the relay applies the floor calibration and the refit in use; the frame follower decides
# * (test/world/test_frame_follower.py).


def z_up_camera_inverse(camera: MocapCamera) -> MocapCamera:
    """A Z-up camera as Motive sends it (Y-up), so the relay turns it back."""
    x, y, z = camera.position
    qx, qy, qz, qw = camera.orientation
    return MocapCamera(camera.name, (x, z, -y), (qx, qz, -qy, qw))


def raw_cameras_after(motions) -> list:
    """Every lab camera as Motive sends it (Y-up) after its frame moved by each (shift, degrees about z) in turn."""
    return [z_up_camera_inverse(camera) for camera in cameras_after(motions).values()]


def see_markers(node: MocapRelay, points) -> None:
    """One NatNet frame's unlabeled markers at `points` (Z-up), then the relay's look."""
    node._receive_labeled_markers({0: {index + 1: {"pos": tuple(Y_UP_TO_Z_UP.inv().apply(point)), "error": 0.0,
                                                   "param": MARKER_POINT_CLOUD_SOLVED}
                                       for index, point in enumerate(points)}})
    node._check_anchors()


def floor_relay(tmp_path, anchors: dict | None = None, floor_from_raw=None) -> MocapRelay:
    """A relay started with a floor calibration made with the lab's cameras where they are; cameras heard, one look."""
    export(tmp_path, cameras=LAB_CAMERAS, floor_from_raw=floor_from_raw, anchors=anchors)
    node = relay(drive_root=tmp_path)
    node._load_floor_calibration("start")
    node._take_definitions(Reply(1.0, [], raw_cameras_after([])))
    see_markers(node, list((anchors or {}).values()))
    return node


def recalibrated(node: MocapRelay, motions=((SHIFT, TURN),)) -> None:
    """Motive recalibrated: its cameras and the anchors seen from the moved frame, one look."""
    node._take_definitions(Reply(2.0, [], raw_cameras_after(motions)))
    see_markers(node, anchors_after(motions))


def announced_frame(node: MocapRelay) -> dict:
    """The relay's last floor calibration announcement."""
    return json.loads(node._frame_publisher.sent[-1].data)


def use_refit(node: MocapRelay):
    """Call ~/use_refit."""
    return node._handle_use_refit(None, SimpleNamespace(success=None, message=""))


def reload_floor(node: MocapRelay):
    """Call ~/reload_floor_calibration."""
    return node._handle_reload_floor(None, SimpleNamespace(success=None, message=""))


def published_point(node: MocapRelay, point) -> np.ndarray:
    """Where the relay publishes rigid body 7 (no robot's) that Motive reports at `point` (Z-up)."""
    frame(node, 7, Y_UP_TO_Z_UP.inv().apply(point), (0.0, 0.0, 0.0, 1.0))
    return pose_of(published(node, 7))[0]


@pytest.mark.parametrize("seed", range(3))
def test_poses_are_put_in_the_lab_frame_as_the_core_composes_them(tmp_path, seed):
    """The relay's own vector maths (fast, on NatNet's thread) equals lab_from_raw composed with the raw pose."""
    floor = Pose(tuple(np.random.default_rng(seed).normal(size=3) * 0.1),
                 tuple(Rotation.random(random_state=seed).as_quat()))
    node = floor_relay(tmp_path, floor_from_raw=(floor.position, floor.orientation))
    raw = Pose((1.0, 2.0, 0.5), tuple(Rotation.random(random_state=seed + 10).as_quat()))
    position, quaternion = node._in_lab_frame(raw.position, raw.orientation)
    expected = compose(floor, raw)
    np.testing.assert_allclose(position, expected.position, atol=1e-12)
    assert same_rotation(quaternion, expected.orientation)


def test_poses_say_lab_with_a_floor_calibration_and_mocap_raw_without(tmp_path):
    """header.frame_id names the frame the pose is in."""
    node = relay()
    frame(node, 7, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert published(node, 7).header.frame_id == "mocap_raw"
    node = floor_relay(tmp_path)
    frame(node, 7, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert published(node, 7).header.frame_id == "lab"


def test_a_recalibration_changes_no_pose_until_the_refit_is_used_and_a_restart_starts_from_the_file(tmp_path):
    """Pending: poses in Motive's moved frame; used: a fixed point back, no file written; a restart: pending again."""
    node = floor_relay(tmp_path, LAB_ANCHORS)
    assert announced_frame(node)["status"] == "held" and announced_frame(node)["decides"] == "anchors"
    recalibrated(node)
    assert announced_frame(node)["status"] == "refit_pending"
    assert "REFIT PENDING by the anchors" in node.get_logger().text("warn")
    point = np.array([1.0, 2.0, 0.5])
    moved = seen_after(point, [(SHIFT, TURN)])
    np.testing.assert_allclose(published_point(node, moved), moved, atol=1e-9)
    response = use_refit(node)
    assert response.success and announced_frame(node)["status"] == "held"
    assert "REFIT USED" in node.get_logger().text("info")
    np.testing.assert_allclose(published_point(node, moved), point, atol=1e-6)
    see_markers(node, anchors_after([(SHIFT, TURN)]))
    sent = json.loads(node._anchors_publisher.sent[-1].data)["anchors"]
    np.testing.assert_allclose(sent["A5"], LAB_ANCHORS["A5"], atol=1e-6)
    assert [path.name for path in (tmp_path / SESSIONS_FOLDER / "20261024").iterdir()] == [
        f"20261024_090000{FLOOR_SUFFIX}"]
    restarted = floor_relay(tmp_path, LAB_ANCHORS)
    recalibrated(restarted)
    assert announced_frame(restarted)["status"] == "refit_pending" and use_refit(restarted).success


def test_nothing_pending_is_refused_by_the_service(tmp_path):
    """Held: the service says so."""
    response = use_refit(floor_relay(tmp_path))
    assert not response.success and "no refit pending" in response.message


def test_cameras_and_markers_are_published_corrected_like_every_pose(tmp_path):
    """After a refit, the truss cameras and a fixed labeled marker are where the floor calibration has them."""
    node = floor_relay(tmp_path)
    recalibrated(node)
    assert use_refit(node).success
    published_cameras = read_cameras(node._cameras_publisher.sent[-1].data)
    for name in truss_names(LAB_CAMERAS):
        np.testing.assert_allclose(published_cameras[name].position, LAB_CAMERAS[name].position, atol=1e-6)
    point = np.array([1.0, 2.0, 0.5])
    marker = {1: {"pos": tuple(Y_UP_TO_Z_UP.inv().apply(seen_after(point, [(SHIFT, TURN)]))), "error": 0.0,
                  "param": MARKER_POINT_CLOUD_SOLVED}}
    frame(node, 7, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), markers=marker)
    p = published(node, 7).marker_positions[0]
    np.testing.assert_allclose((p.x, p.y, p.z), point, atol=1e-6)


def test_a_floor_calibration_made_after_a_refit_reloads_without_a_jump_and_holds_through_the_next(tmp_path):
    """It has lab_from_raw as now and the cameras raw now: the same point stays put; a second refit brings it home."""
    first, second = (SHIFT, TURN), (np.array([-0.03, 0.02, 0.0]), -1.2)
    node = floor_relay(tmp_path)
    recalibrated(node, [first])
    assert use_refit(node).success
    point = np.array([1.0, 2.0, 0.5])
    before = published_point(node, seen_after(point, [first]))
    export(tmp_path, "20261024_120000", cameras_after([first]), announced_frame(node)["lab_from_raw"])
    node._load_floor_calibration("reload")
    assert announced_frame(node)["status"] == "unchecked" and announced_frame(node)["refit"] is None
    np.testing.assert_allclose(published_point(node, seen_after(point, [first])), before, atol=1e-6)
    recalibrated(node, [first, second])
    assert announced_frame(node)["status"] == "refit_pending" and use_refit(node).success
    np.testing.assert_allclose(published_point(node, seen_after(point, [first, second])), point, atol=1e-6)


@pytest.mark.parametrize("break_it", ["half-synced file", "Drive gone", "floor folder gone"])
def test_a_reload_that_cannot_read_the_drive_keeps_the_frame_in_use(tmp_path, break_it):
    """A half-synced floor calibration, a Drive or its floor folder gone: an error, every pose stays corrected."""
    node = floor_relay(tmp_path)
    recalibrated(node)
    assert use_refit(node).success
    point = np.array([1.0, 2.0, 0.5])
    if break_it == "half-synced file":
        export(tmp_path).write_text('{"cameras": [{"name": ')
    elif break_it == "Drive gone":
        node.drive_root = tmp_path / "unmounted"
    else:
        (tmp_path / SESSIONS_FOLDER).rename(tmp_path / "moved_away")
    assert not reload_floor(node).success
    assert "not read" in node.get_logger().text("error") and announced_frame(node)["status"] == "held"
    np.testing.assert_allclose(published_point(node, seen_after(point, [(SHIFT, TURN)])), point, atol=1e-6)


def test_a_start_that_cannot_read_the_frame_says_so(tmp_path):
    """No frame to keep: the status is error, so every base's mocap chip turns amber."""
    export(tmp_path).write_text('{"cameras": [{"name": ')
    node = relay(drive_root=tmp_path)
    node._load_floor_calibration("start")
    assert announced_frame(node)["status"] == "error" and "not read" in announced_frame(node)["error"]


def test_a_start_with_a_half_synced_newest_floor_calibration_applies_the_one_before(tmp_path):
    """The newest does not read: the one before is applied, the newest passed over with a warning; poses not raw."""
    export(tmp_path, floor_from_raw=((0.01, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
    export(tmp_path, "20261027_120000").write_text('{"cameras": [{"name": ')
    node = relay(drive_root=tmp_path)
    node._load_floor_calibration("start")
    assert announced_frame(node)["floor_calibration"].startswith("20261024_090000")
    assert announced_frame(node)["status"] == "unchecked"
    assert "passed over 20261027_120000" in node.get_logger().text("warn")
    np.testing.assert_allclose(published_point(node, (1.0, 2.0, 0.5)), (1.01, 2.0, 0.5), atol=1e-9)


def test_a_first_start_without_a_floor_folder_is_no_floor_calibration(tmp_path):
    """No frame in use and no folder yet (e.g. a new simulation domain): "none", not an error."""
    node = relay(drive_root=tmp_path)
    node._load_floor_calibration("start")
    assert announced_frame(node)["status"] == "none" and node.get_logger().text("error") == ""


def test_a_simulations_relay_announces_its_own_floor_calibrations_folder(tmp_path):
    """With sim:=true, the simulation's sessions of its domain: where the probe writes new floor calibrations."""
    node = relay(simulated=True, drive_root=tmp_path)
    node._load_floor_calibration("start")
    announced = announced_frame(node)
    assert announced["folder"] == "data_experiment/sim/calibration_data/domain_70"
    assert floor_calibration_state(json.dumps(announced)).folder == announced["folder"]


def test_a_new_floor_calibration_is_announced_and_applied_only_on_command(tmp_path):
    """The probe exports again: warned about once, announced, poses unchanged; reload_floor_calibration applies it."""
    node = floor_relay(tmp_path)
    before = published_point(node, (1.0, 2.0, 0.5))
    export(tmp_path, "20261027_120000", floor_from_raw=((0.003, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
    for _ in range(3):
        node._watch_for_newer()
    warned = " ".join(node.get_logger().text("warn").split())
    assert warned.count("newer floor calibration on the Drive, not applied") == 1
    state = floor_calibration_state(node._frame_publisher.sent[-1].data)
    assert state.floor_calibration.startswith("20261024_090000") and state.newer.startswith("20261027_120000")
    assert state.newer_moves_poses and state.poses_off
    np.testing.assert_allclose(published_point(node, (1.0, 2.0, 0.5)), before, atol=1e-9)
    response = reload_floor(node)
    assert response.success and "20261027_120000" in response.message and "moved up to" in response.message
    assert announced_frame(node)["floor_calibration"].startswith("20261027_120000")
    assert announced_frame(node)["newer"] is None


def test_a_newer_floor_calibration_within_a_millimetre_is_info_and_moves_no_pose(tmp_path):
    """0.5 mm from the one in use: logged as info, announced, and the mocap chips stay as they are."""
    node = floor_relay(tmp_path)
    export(tmp_path, "20261027_120000", floor_from_raw=((0.0005, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
    node._watch_for_newer()
    assert "newer floor calibration" in node.get_logger().text("info")
    assert "newer floor calibration" not in node.get_logger().text("warn")
    state = floor_calibration_state(node._frame_publisher.sent[-1].data)
    assert state.newer and not state.newer_moves_poses and not state.poses_off


def test_the_base_calibrations_reload_leaves_the_floor_calibration_alone(tmp_path):
    """~/reload_base_calibrations is the robots' base calibrations only: a newer floor calibration stays announced."""
    node = floor_relay(tmp_path)
    node._load_calibrations = lambda why, only=None: [f"calibrations at {why}"]
    export(tmp_path, "20261027_120000", floor_from_raw=((0.003, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
    node._watch_for_newer()
    node._handle_reload(None, SimpleNamespace(success=None, message=""))
    assert announced_frame(node)["floor_calibration"].startswith("20261024_090000")


def test_a_failing_callback_is_logged_and_the_relay_goes_on():
    """A timer's error is logged with its traceback, not raised into rclpy.spin; a service answers it failed."""
    node = relay()

    def look():
        raise ValueError("bad look")

    assert node._survives(look)() is None
    assert "look failed, the relay goes on: ValueError('bad look')" in node.get_logger().text("error")
    answer = node._survives(lambda _request, _response: look())(None, SimpleNamespace(success=None, message=""))
    assert answer.success is False and "bad look" in answer.message


def test_one_robots_reload_leaves_the_others_on_their_calibration(tmp_path):
    """~/reload_base_calibration/id_1850 applies Belle's newer session; Alice keeps hers and is still warned about."""
    node = relay(drive_root=tmp_path)
    node._load_calibrations("start")
    for mocap_id, digits in ((1840, "0804"), (1850, "0805")):
        default = default_calibration(mocap_id)
        path = tmp_path / SESSIONS_FOLDER / "20261101" / f"calibrated_transformation_{digits}_rhino.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({
            "base_mocap_from_ur_arm_base_link_inertia": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
            "base_mocap_from_base_footprint": [[default.position[0] + 0.01, *default.position[1:]],
                                               list(default.orientation)]}))
    response = node._handle_reload(None, SimpleNamespace(success=None, message=""), 1850)
    assert response.success and "20261101" in node._calibrations[1850].source
    assert node._calibrations[1840].session == ""
    node._watch_for_newer()
    warned = node.get_logger().text("warn")
    assert "rigid body 1840" in warned and "reload_base_calibration/id_1840" in warned
    assert "rigid body 1850" not in warned


def test_a_first_floor_calibration_is_applied_on_command_while_the_relay_runs(tmp_path):
    """No floor calibration yet, then the probe exports one: announced, applied by the reload, no restart needed."""
    (tmp_path / SESSIONS_FOLDER).mkdir(parents=True)
    node = relay(drive_root=tmp_path)
    node._load_floor_calibration("start")
    node._take_definitions(Reply(1.0, [], raw_cameras_after([])))
    export(tmp_path, "20261027_120000")
    node._watch_for_newer()
    assert announced_frame(node)["status"] == "none" and announced_frame(node)["newer"]
    assert reload_floor(node).success and announced_frame(node)["status"] == "unchecked"
    see_markers(node, [LOOSE])
    assert announced_frame(node)["status"] == "held"


def test_a_half_synced_new_file_is_tried_again(tmp_path):
    """Not readable yet: nothing announced or warned about; readable at the next look, announced."""
    node = floor_relay(tmp_path)
    export(tmp_path, "20261027_120000").write_text('{"cameras": [{"name": ')
    node._watch_for_newer()
    assert announced_frame(node)["newer"] is None
    export(tmp_path, "20261027_120000")
    node._watch_for_newer()
    assert announced_frame(node)["newer"]["floor_calibration"].startswith("20261027_120000")


def test_anchors_lost_without_cameras_are_untrusted_not_held(tmp_path):
    """Six anchors stored, no cameras from Motive, its frame moved 22 mm: untrusted, never held; unchecked before."""
    export(tmp_path, anchors=LAB_ANCHORS)
    node = relay(drive_root=tmp_path)
    node._load_floor_calibration("start")
    assert announced_frame(node)["status"] == "unchecked"
    see_markers(node, [np.add(position, (0.022, 0.0, 0.0)) for position in LAB_ANCHORS.values()])
    assert announced_frame(node)["status"] == "untrusted"
    assert floor_calibration_state(node._frame_publisher.sent[-1].data).poses_off


def test_the_anchors_are_averaged_and_published_in_the_lab_frame(tmp_path):
    """Seen 0.1 mm either side in turn: published at the middle, in the lab frame like every marker.

    floor_from_raw is turned 30° and shifted.
    """
    floor, shift = Rotation.from_euler("z", 30.0, degrees=True), np.array([1.0, -2.0, 0.0])
    node = floor_relay(tmp_path, LAB_ANCHORS, (tuple(shift), tuple(floor.as_quat())))
    for offset in (-0.0001, 0.0001):
        node._receive_labeled_markers({0: {index + 1: {"pos": tuple(Y_UP_TO_Z_UP.inv().apply(np.add(
            position, (offset, 0.0, 0.0)))), "error": 0.0, "param": MARKER_POINT_CLOUD_SOLVED}
            for index, position in enumerate([*LAB_ANCHORS.values(), LOOSE])}})
    node._check_anchors()
    sent = json.loads(node._anchors_publisher.sent[-1].data)
    assert sorted(sent["anchors"]) == sorted(LAB_ANCHORS) and sent["missing"] == []
    np.testing.assert_allclose(sent["anchors"]["A5"], floor.apply(LAB_ANCHORS["A5"]) + shift, atol=1e-9)
    markers = json.loads(node._markers_publisher.sent[-1].data)["markers"]
    np.testing.assert_allclose(markers[-1], floor.apply(LOOSE) + shift, atol=1e-9)


def test_an_anchor_whose_unlabeled_id_spilled_into_the_model_id_is_still_found(tmp_path):
    """Motive's unlabeled ids past 65535 arrive as model 3 (flagged unlabeled): found, and no marker of body 3."""
    node = floor_relay(tmp_path, LAB_ANCHORS)
    names = list(LAB_ANCHORS)
    node._receive_rigid_body_frame(3, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), True, 0.001)
    by_model = {0: {}, 3: {}}
    for index, name in enumerate(names):
        model, marker_id = (3, 48797) if name == "A2" else (0, index + 1)
        by_model[model][marker_id] = {"pos": tuple(Y_UP_TO_Z_UP.inv().apply(LAB_ANCHORS[name])), "error": 0.0,
                                      "param": MARKER_POINT_CLOUD_SOLVED | MARKER_UNLABELED}
    by_model[0][48797] = {"pos": tuple(Y_UP_TO_Z_UP.inv().apply(LOOSE)), "error": 0.0, "param": MARKER_UNLABELED}
    node._receive_labeled_markers(by_model)
    assert node._pending_samples[3].markers == ()
    node._check_anchors()
    assert json.loads(node._anchors_publisher.sent[-1].data)["missing"] == []
    assert len(json.loads(node._markers_publisher.sent[-1].data)["markers"]) == len(names) + 1


def test_without_a_floor_calibration_no_anchors_are_looked_for_and_the_markers_still_published():
    """No floor calibration: no anchors announced, nothing logged; the unlabeled markers go out in any case."""
    node = relay()
    see_markers(node, [LOOSE])
    assert node._anchors_publisher.sent == [] and node.get_logger().text("warn") == ""
    np.testing.assert_allclose(json.loads(node._markers_publisher.sent[-1].data)["markers"], [LOOSE], atol=1e-9)


def test_every_look_publishes_both_fits_on_the_metrics_topic(tmp_path):
    """After a recalibration: one sample per look, with both fits, the state and which fit decides."""
    node = floor_relay(tmp_path, LAB_ANCHORS)
    recalibrated(node)
    sample = json.loads(node._metrics_publisher.sent[-1].data)
    assert sample["status"] == "refit_pending" and sample["decides"] == "anchors" and sample["floor_calibration"]


def test_each_anchors_noise_is_its_rms_distance_from_its_mean_over_the_look(tmp_path):
    """Seen 0.1 mm either side of A5 in x and 0.3 mm of A1 in z: noise 0.1 and 0.3 mm, from the sums of squares."""
    node = floor_relay(tmp_path, LAB_ANCHORS)
    for sign in (-1.0, 1.0, -1.0, 1.0):
        shifted = {**LAB_ANCHORS, "A5": np.add(LAB_ANCHORS["A5"], (sign * 0.0001, 0.0, 0.0)),
                   "A1": np.add(LAB_ANCHORS["A1"], (0.0, 0.0, sign * 0.0003))}
        node._receive_labeled_markers({0: {index + 1: {"pos": tuple(Y_UP_TO_Z_UP.inv().apply(position)),
                                                       "error": 0.0, "param": MARKER_POINT_CLOUD_SOLVED}
                                           for index, position in enumerate(shifted.values())}})
    node._check_anchors()
    quality = json.loads(node._metrics_publisher.sent[-1].data)["anchor_quality"]
    assert quality["A5"]["noise_mm"] == pytest.approx(0.1, abs=1e-3) and quality["A5"]["frames"] == 4
    assert quality["A1"]["noise_mm"] == pytest.approx(0.3, abs=1e-3) and quality["A2"]["noise_mm"] < 1e-3
    assert {entry["status"] for entry in quality.values()} == {"used"}


def test_every_look_is_kept_in_the_frame_log_as_published(tmp_path):
    """One line per look in the day's file, the sample published; also without a floor calibration."""
    node = floor_relay(tmp_path / "drive", LAB_ANCHORS)
    node._frame_log = FrameLog(tmp_path / "log", 30)
    see_markers(node, list(LAB_ANCHORS.values()))
    see_markers(node, list(LAB_ANCHORS.values()))
    files = list((tmp_path / "log").iterdir())
    assert [path.name for path in files] == [f"{time.strftime('%Y%m%d')}.jsonl"]
    lines = files[0].read_text().splitlines()
    assert len(lines) == 2 and lines[-1] == node._metrics_publisher.sent[-1].data
    bare = relay()
    bare._frame_log = FrameLog(tmp_path / "bare", 30)
    see_markers(bare, [LOOSE])
    assert json.loads((tmp_path / "bare" / files[0].name).read_text())["status"] == "none"


def test_a_frame_log_that_cannot_be_written_is_an_error_not_a_crash(tmp_path):
    """Its folder is a file: each look still published, the error logged."""
    (tmp_path / "log").write_text("")
    node = relay()
    node._frame_log = FrameLog(tmp_path / "log", 30)
    see_markers(node, [LOOSE])
    see_markers(node, [LOOSE])
    assert len(node._metrics_publisher.sent) == 2 and "frame log" in node.get_logger().text("error")


# * Motive's frame and cameras moving while the relay runs (between two readings, not against the floor calibration).

def test_without_a_floor_calibration_a_move_is_still_warned_about(tmp_path):
    """No frame to hold: nothing refitted, but cameras moving while the relay runs are warned about."""
    node = relay(drive_root=tmp_path)
    node._load_floor_calibration("start")
    node._take_definitions(Reply(1.0, [], raw_cameras_after([])))
    node._take_definitions(Reply(2.0, [], raw_cameras_after([(SHIFT, TURN)])))
    node._check_anchors()
    assert "MOVED while the relay runs" in node.get_logger().text("warn") and announced_frame(node)["status"] == "none"


def test_a_slow_creep_is_warned_about_once_it_adds_up():
    """0.3 mm steps of Motive's frame, each under the 1 mm threshold: warned at 1.2 mm, then at 2.4 mm (1.2 since)."""
    node = relay()
    for step in range(0, 9):
        node._take_definitions(Reply(float(step), [], raw_cameras_after([((0.0003 * step, 0.0, 0.0), 0.0)])))
    log = node.get_logger().text("warn")
    assert log.count("truss cameras moved together") == 2 and "camera moved" not in log


def test_cameras_moving_while_the_anchors_hold_the_frame_are_info_not_a_warning(monkeypatch):
    """Continuous calibration re-solving the cameras: while the anchors hold the lab frame, poses did not move."""
    node = relay()
    monkeypatch.setattr(FrameFollower, "anchors_hold", property(lambda follower: True))
    node._take_definitions(Reply(1.0, [], raw_cameras_after([])))
    node._take_definitions(Reply(2.0, [], raw_cameras_after([(SHIFT, TURN)])))
    assert "MOVED" not in node.get_logger().text("warn")
    assert "the anchors hold the lab frame: poses unaffected" in node.get_logger().text("info")


def test_a_frame_move_is_told_apart_from_a_camera_moved_against_it():
    """Motive's frame moved 20 mm, a tripod 5 mm on its own: the frame's move, then only the tripod against it."""
    node = relay()
    node._take_definitions(Reply(1.0, [], raw_cameras_after([])))
    tripod = LAB_CAMERAS["PrimeX 22 #72318"]
    lab = {**LAB_CAMERAS, tripod.name: replace(tripod, position=tuple(np.add(tripod.position, (0.005, 0.0, 0.0))))}
    inverse = Rotation.from_euler("z", TURN, degrees=True).inv()
    now = {name: MocapCamera(name, tuple(inverse.apply(np.subtract(camera.position, SHIFT))),
                             tuple((inverse * Rotation.from_quat(camera.orientation)).as_quat()))
           for name, camera in lab.items()}
    node._take_definitions(Reply(2.0, [], [z_up_camera_inverse(camera) for camera in now.values()]))
    log = node.get_logger().text("warn")
    assert "CAMERAS MOVED while" in log and "the anchors do not hold the lab frame (frame none)" in log
    assert "the 6 truss cameras moved together 22.6 mm" in log and "each camera below moved against them" in log
    assert "BMR #72318: camera moved 5.0 mm" in log and log.count("camera moved") == 1


def test_after_a_frame_move_a_camera_creeping_against_it_is_still_warned_once_it_adds_up():
    """The cameras kept as seen from the moved frame: a tripod's 0.4 mm steps after it are warned about at 1.2 mm."""
    node = relay()
    node._take_definitions(Reply(1.0, [], raw_cameras_after([])))
    node._take_definitions(Reply(2.0, [], raw_cameras_after([(SHIFT, TURN)])))
    name = "PrimeX 22 #72318"
    for step in range(1, 4):
        cameras = {camera.name: camera for camera in raw_cameras_after([(SHIFT, TURN)])}
        tripod = cameras[name]
        cameras[name] = replace(tripod, position=tuple(np.add(tripod.position, (0.0004 * step, 0.0, 0.0))))
        node._take_definitions(Reply(2.0 + step, [], list(cameras.values())))
    log = node.get_logger().text("warn")
    assert log.count("MOVED while the relay runs") == 2 and log.count("BMR #72318: camera moved 1.2 mm") == 1


@pytest.mark.parametrize("name, label", [("PrimeX 22 #72318", "BMR #72318"),
                                         ("PrimeX 22 #72657", "BFL #72657 (truss)")])
def test_every_camera_move_is_warned_about_and_truss_cameras_are_marked(name, label):
    """A tripod or a truss camera moved 5 mm: one warning, the lab's name and serial, "(truss)" for a truss camera."""
    node = relay()
    node._take_definitions(Reply(1.0, [], raw_cameras_after([])))
    camera = LAB_CAMERAS[name]
    moved = {**LAB_CAMERAS, name: replace(camera, position=tuple(np.add(camera.position, (0.005, 0.0, 0.0))))}
    node._take_definitions(Reply(2.0, [], [z_up_camera_inverse(c) for c in moved.values()]))
    log = node.get_logger().text("warn")
    assert log.count("CAMERAS MOVED") == 1 and f"{label}: camera moved" in log


def test_a_camera_gone_is_warned_about():
    """Unplugged or switched off: a warning naming it."""
    node = relay()
    node._take_definitions(Reply(1.0, [], raw_cameras_after([])))
    gone = "PrimeX 22 #72318"
    node._take_definitions(Reply(2.0, [], [z_up_camera_inverse(c) for n, c in LAB_CAMERAS.items() if n != gone]))
    assert "BMR #72318: camera gone" in node.get_logger().text("warn")


@pytest.mark.parametrize("receive", ["_receive_sim_definitions", "_receive_sim_cameras"])
def test_a_fake_announcement_that_does_not_read_is_skipped_not_fatal(receive):
    """The relay keeps running: a warning, nothing published from it."""
    node = relay()
    getattr(node, receive)(SimpleNamespace(data="not json"))
    assert "skipped" in node.get_logger().text("warn") and node._definitions_publisher.sent == []


def test_an_unreadable_reply_is_no_news():
    """A reply that did not read keeps Motive's bodies and cameras: nothing gone, nothing announced again."""
    node = relay()
    node._take_definitions(Reply(1.0, [body()], [CAMERA]))
    sent = len(node._definitions_publisher.sent), len(node._cameras_publisher.sent)
    node._take_definitions(Reply(3.0, problems=["could not read the descriptions: unpack requires a buffer"]))
    assert (len(node._definitions_publisher.sent), len(node._cameras_publisher.sent)) == sent
    assert "body gone" not in node.get_logger().text("info")


def test_each_bodys_source_is_announced_when_it_changes():
    """Announced with the calibrations once per change: a fake Motive's body is calibrated, a simulator's truth not."""
    node = relay(simulated=True)
    sim_body(node, TRUTH, 3, (1.0, 0.0, 0.0))
    published(node, 3)
    sim_body(node, FAKE_MOTIVE, 3, (2.0, 0.0, 0.0))
    published(node, 3)
    published(node, 3)
    announced = [json.loads(message.data)["sources"] for message in node._calibrations_publisher.sent]
    assert announced == [{"3": TRUTH}, {"3": FAKE_MOTIVE}]
    state = MocapRelayState(simulated=True, sources={3: TRUTH})
    assert state.source(3) == TRUTH and state.source(7) == FAKE_MOTIVE and MocapRelayState().source(7) == "Motive"
