"""
ROS2 node that relays OptiTrack rigid body data into ROS topics.

It consumes NatNet rigid body callbacks at mocap frequency and republishes the latest pose of each rigid body at a
lower, timer-driven rate, with the raw markers Motive labeled to it in the same frame. Each pose is stamped with when
the cameras captured it, on this PC's clock, and a robot's rigid body is turned into its base_footprint pose with
its base calibration (`world/calibration_sessions.py`).

- Services: `/mocap/get_streaming_ids` (which ids stream), `~/reload_calibration` (apply the newest calibrations).
- `~/calibrations` (latched): what is applied to each rigid body, with its source and hash.
- ! Calibrations are applied at start and on `~/reload_calibration` only. A newer one appearing on the Drive is
  logged, never applied by itself: a robot driving its path onboard would jump.
- Outside the robots' domain it also reads simulated inputs (SIM_SOURCES), found by their topics: per rigid body, a
  fake Motive's body wins over a simulator's true pose, which wins over Motive's, while each streams. A truth needs
  no calibration. A fake Motive's robot base comes relative to crl-husky's default calibration: that is taken off,
  then the simulated sessions' calibration applied. Motive's bodies get the robots' sessions.
"""

from __future__ import annotations

import dataclasses
import json
import re
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Time
from crl_husky_msgs.msg import MocapRigidBodyPose
from crl_husky_msgs.srv import GetStreamingMocapIds
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from crl_husky import config_resolver
from crl_husky.sim.husky_sim import TRUTH_PREFIX
from crl_husky.simulated_parts import fleet_domain_problem
from ..drive import drive_root
from ..world.calibration_sessions import (DESCRIBE_HEADER, SESSIONS_FOLDER, SIM_SESSIONS_FOLDER, Calibration,
                                          default_calibration_or_none, explain, newest_session_file, resolve,
                                          robot_mocap_ids)
from .log_text import box, good, wrap
from .natnet.NatNetClient import NatNetClient

#: The relay's service; whoever serves it is a running relay.
STREAMING_IDS_SERVICE = "/mocap/get_streaming_ids"
#: Seconds to look for another relay on the network before starting.
#: ? The ROS graph fills in shortly after a node starts, so an immediate look can miss it.
RELAY_DISCOVERY_SEC = 2.0
#: Where a fake Motive (`sim_motive.py`) publishes its bodies: <prefix>/id_<n>/pose, Z-up world; a robot's base
#: relative to crl-husky's default calibration.
SIM_MOTIVE_PREFIX = "/mocap_sim/motive"
#: Where each body comes from, highest priority first: a fake Motive, a simulator's true base_footprint
#: (crl-husky's husky_sim, on TRUTH_PREFIX), Motive itself.
FAKE_MOTIVE, TRUTH, MOTIVE = "fake Motive", "simulator's truth", "Motive"
#: The simulated sources by topic prefix.
SIM_SOURCES = {SIM_MOTIVE_PREFIX: FAKE_MOTIVE, TRUTH_PREFIX: TRUTH}
SIM_TOPIC = re.compile(rf"^({'|'.join(re.escape(prefix) for prefix in SIM_SOURCES)})/id_(\d+)/pose$")
#: Seconds between looks for new simulated inputs.
SIM_DISCOVERY_SEC = 1.0
#: Motive latencies outside [0, this] are treated as garbage; the frame is stamped with its arrival instead.
MAX_PLAUSIBLE_LATENCY_SEC = 0.5
#: NatNet labeled marker flags: hidden from the cameras, and position measured by the cameras.
MARKER_OCCLUDED = 0x01
MARKER_POINT_CLOUD_SOLVED = 0x02

Vec3 = Tuple[float, float, float]
QuatXyzw = Tuple[float, float, float, float]


@dataclasses.dataclass(frozen=True)
class Marker:
    """One raw marker Motive labeled to a rigid body, in the Z-up world frame (not calibrated).

    Attributes:
        member_id: 1-based member number in the Motive asset.
        pos: Position, metres.
        residual: Residual of the reconstruction, metres.
        measured: False when occluded and filled in from Motive's own rigid body pose.
    """

    member_id: int
    pos: Vec3
    residual: float
    measured: bool


@dataclasses.dataclass(frozen=True)
class BodySample:
    """One rigid body in one mocap frame, in the Z-up world frame and calibrated if possible.

    Attributes:
        pos: Position, metres.
        quat: Orientation (x, y, z, w).
        tracking_valid: Whether Motive tracked the body.
        marker_error: Mean marker error, metres.
        markers: Raw markers of this frame, by member id.
        capture_time_ns: Mid-exposure time, ns on this PC's wall clock; set once the frame completes.
        received_mono_sec: time.monotonic() when the frame arrived, for staleness.
    """

    pos: Vec3
    quat: QuatXyzw
    tracking_valid: bool
    marker_error: float
    markers: Tuple[Marker, ...] = ()
    capture_time_ns: int = 0
    received_mono_sec: float = 0.0


def quat_multiply_xyzw(q1: QuatXyzw, q2: QuatXyzw) -> QuatXyzw:
    """The product q1 * q2 of two quaternions (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def rotate_vec_by_quat_xyzw(v: Vec3, q: QuatXyzw) -> Vec3:
    """A vector turned by a quaternion (x, y, z, w)."""
    x, y, z, w = q
    r00, r01, r02 = 1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)
    r10, r11, r12 = 2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)
    r20, r21, r22 = 2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)
    return (r00 * v[0] + r01 * v[1] + r02 * v[2],
            r10 * v[0] + r11 * v[1] + r12 * v[2],
            r20 * v[0] + r21 * v[1] + r22 * v[2])


def apply_base_calibration(calibration: Calibration, pos: Vec3, quat: QuatXyzw) -> Tuple[Vec3, QuatXyzw]:
    """A rigid body pose turned into its robot's base_footprint pose: world_from_mocap * base_mocap_from_base."""
    offset = rotate_vec_by_quat_xyzw(calibration.position, quat)
    return ((pos[0] + offset[0], pos[1] + offset[1], pos[2] + offset[2]),
            quat_multiply_xyzw(quat, calibration.orientation))


def remove_base_calibration(calibration: Calibration, pos: Vec3, quat: QuatXyzw) -> Tuple[Vec3, QuatXyzw]:
    """A robot's base_footprint pose turned back into its rigid body's: the inverse of `apply_base_calibration`."""
    cx, cy, cz, cw = calibration.orientation
    body_quat = quat_multiply_xyzw(quat, (-cx, -cy, -cz, cw))
    offset = rotate_vec_by_quat_xyzw(calibration.position, body_quat)
    return (pos[0] - offset[0], pos[1] - offset[1], pos[2] - offset[2]), body_quat


class MocapRelay(Node):
    """Relays mocap poses, serves the streaming rigid body ids, and applies and reports base calibrations."""

    def __init__(self) -> None:
        """Declare the parameters, load the calibrations and start the NatNet client.

        Raises:
            RuntimeError: If another relay already runs in this ROS domain.
        """
        super().__init__("mocap_relay")

        self._refuse_if_relay_on_network()

        self.declare_parameter("client_ip", "192.168.0.7")
        self.declare_parameter("server_ip", "192.168.0.117")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("topic_prefix", "/mocap/rigid_body")
        self.declare_parameter("frame_id", "mocap_world")
        self.declare_parameter("streaming_stale_timeout_sec", 1.0)
        self.declare_parameter("marker_error_valid_threshold", 1.0)
        self.declare_parameter("use_multicast", False)
        self.declare_parameter("multicast_address", "239.255.42.99")
        self.declare_parameter("frame_timeout_sec", 2.0)
        self.declare_parameter("reconnect_min_interval_sec", 2.0)
        # * Calibrations: the Drive root (empty: HUSKY_DRIVE_ROOT), whether to look there, how often to check.
        self.declare_parameter("drive_root", "")
        self.declare_parameter("use_drive", True)
        self.declare_parameter("watch_period_sec", 30.0)

        self.client_ip = self.get_parameter("client_ip").value
        self.server_ip = self.get_parameter("server_ip").value
        self.publish_rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        self.topic_prefix = str(self.get_parameter("topic_prefix").value).rstrip("/")
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.streaming_stale_timeout_sec = max(float(self.get_parameter("streaming_stale_timeout_sec").value), 0.01)
        self.marker_error_valid_threshold = max(float(self.get_parameter("marker_error_valid_threshold").value), 0.0)
        self.use_multicast = bool(self.get_parameter("use_multicast").value)
        self.multicast_address = str(self.get_parameter("multicast_address").value)
        self.frame_timeout_sec = max(float(self.get_parameter("frame_timeout_sec").value), 0.1)
        self.reconnect_min_interval_sec = max(float(self.get_parameter("reconnect_min_interval_sec").value), 0.1)
        self.drive_root = (drive_root(str(self.get_parameter("drive_root").value))
                           if bool(self.get_parameter("use_drive").value) else None)
        # ! Simulated inputs only outside the robots' domain: there nothing may stand in for a robot's body.
        self.simulating = not fleet_domain_problem()
        #: The sessions folder of each source whose bodies are calibrated.
        self.sessions_folders = {MOTIVE: SESSIONS_FOLDER}
        if self.simulating:
            self.sessions_folders[FAKE_MOTIVE] = SIM_SESSIONS_FOLDER
        if self.simulating and self.drive_root is not None and self.drive_root.is_dir():
            (self.drive_root / SIM_SESSIONS_FOLDER).mkdir(parents=True, exist_ok=True)

        # Latest sample per rigid body id: Motive's, written by the NatNet thread, and the simulated sources', written
        # by the executor; all read by the executor.
        self._samples_lock = threading.Lock()
        self._samples: Dict[int, BodySample] = {}
        self._sim_samples: Dict[str, Dict[int, BodySample]] = {source: {} for source in SIM_SOURCES.values()}
        # Simulated topics subscribed to, and the source each rigid body was last published from (executor only).
        self._sim_topics: set[str] = set()
        self._sources: Dict[int, str] = {}
        # Bodies of the frame being received; NatNet sends the frame's timestamps after its bodies.
        self._pending_samples: Dict[int, BodySample] = {}
        self._warned_no_hires_stamps = False
        self._warned_no_markers = False
        self._logged_latency = False
        # Avoid clashing with rclpy Node internals (Node already uses _publishers).
        self._rb_publishers: Dict[int, object] = {}
        self._last_frame_mono_sec = time.monotonic()
        self._last_reconnect_attempt_mono = 0.0
        # Reconnects since frames last arrived, so recovery can be reported.
        self._reconnect_attempts = 0
        self._mocap_client_lock = threading.Lock()
        # ! Read by the NatNet thread: always replaced whole (one assignment), never edited in place. Per source in
        # ! `sessions_folders`, by rigid body.
        self._calibrations: Dict[str, Dict[int, Calibration]] = {source: {} for source in self.sessions_folders}
        #: crl-husky's default calibration by robot's rigid body, which a fake Motive's bases are relative to.
        self._defaults: Dict[int, Calibration] = {}
        # Rigid bodies of no robot already logged as published uncalibrated (NatNet thread only).
        self._logged_uncalibrated: set[int] = set()
        # Newer Drive files already reported by the watch, so each is logged once.
        self._reported_newer: set[str] = set()

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # TODO: a proper message in crl_husky_msgs; a JSON string for now (temporary).
        self._calibrations_publisher = self.create_publisher(String, "~/calibrations", latched)
        self._load_calibrations("start")
        self._reload_service = self.create_service(Trigger, "~/reload_calibration", self._handle_reload)
        watch_period = float(self.get_parameter("watch_period_sec").value)
        if watch_period > 0 and self.drive_root is not None:
            self._watch_timer = self.create_timer(watch_period, self._watch_for_newer)

        self._service = self.create_service(GetStreamingMocapIds, STREAMING_IDS_SERVICE,
                                            self._handle_get_streaming_ids)
        self._publish_timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_cached_poses)
        if self.simulating:
            self.get_logger().info(f"Outside the robots' domain: also reading simulated inputs "
                                   f"({', '.join(f'{prefix}/id_<n>/pose' for prefix in SIM_SOURCES)}) as they appear")
            self._sim_timer = self.create_timer(SIM_DISCOVERY_SEC, self._look_for_simulated_inputs)

        self._health_timer = self.create_timer(0.5, self._check_mocap_health)
        self.mocap_client = NatNetClient()
        self._configure_mocap_client(self.mocap_client)
        self._start_mocap_client()

    # --- --- --- --- --- SIMULATED INPUTS --- --- --- --- ---

    def _look_for_simulated_inputs(self) -> None:
        """Subscribe to every simulated input topic that appeared since the last look."""
        for topic, _types in self.get_topic_names_and_types():
            match = SIM_TOPIC.match(topic)
            if match is None or topic in self._sim_topics:
                continue
            self._sim_topics.add(topic)
            source, rigid_body_id = SIM_SOURCES[match[1]], int(match[2])
            self.create_subscription(MocapRigidBodyPose, topic,
                                     lambda message, s=source, i=rigid_body_id: self._receive_sim_body(s, i, message),
                                     10)
            self.get_logger().info(f"rigid body {rigid_body_id}: reading the {source} on {topic}")

    def _receive_sim_body(self, source: str, rigid_body_id: int, message: MocapRigidBodyPose) -> None:
        """Keep one simulated body, Z-up and stamped with when it was true; a fake Motive's gets calibrated.

        ! Stored directly, never through the pending frame: that may hold half of a NatNet frame.
        """
        p, q = message.pose.position, message.pose.orientation
        pos, quat = (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)
        if source == FAKE_MOTIVE:
            default = self._defaults.get(rigid_body_id)
            if default is not None:
                pos, quat = remove_base_calibration(default, pos, quat)
            pos, quat = self._calibrated(rigid_body_id, pos, quat, self._calibrations[FAKE_MOTIVE])
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        sample = BodySample(pos, quat, bool(message.tracking_valid), float(message.marker_error),
                            capture_time_ns=stamp_ns or time.time_ns(), received_mono_sec=time.monotonic())
        with self._samples_lock:
            self._sim_samples[source][rigid_body_id] = sample

    def _simulated_streaming(self) -> bool:
        """Whether any simulated input streams now."""
        cutoff = time.monotonic() - self.streaming_stale_timeout_sec
        with self._samples_lock:
            return any(sample.received_mono_sec >= cutoff
                       for samples in self._sim_samples.values() for sample in samples.values())

    # --- --- --- --- --- CALIBRATIONS --- --- --- --- ---

    def _load_calibrations(self, why: str) -> List[str]:
        """Resolve every robot's body for each calibrated source, apply, log and publish it; return the lines logged."""
        log = self.get_logger()
        root = self.drive_root if self.drive_root is not None and self.drive_root.is_dir() else None
        if self.drive_root is None:
            where = "no Drive (HUSKY_DRIVE_ROOT and drive_root unset, or use_drive off): crl-husky defaults only"
        elif root is None:
            where = f"the Drive root is not a folder, crl-husky defaults only: {self.drive_root}"
        else:
            where = f"Drive root {root}"
        config_resolver.reload_robot_configs()
        calibrations: Dict[str, Dict[int, Calibration]] = {}
        lines = [f"calibrations at {why}, {where}",
                 "crl-husky: config/robots/; only each robot's newest Drive session is looked at"]
        problems = 0
        for source, folder in self.sessions_folders.items():
            calibrations[source] = {}
            lines += ["", f"{source}'s bodies, sessions in {folder}:", DESCRIBE_HEADER]
            for mocap_id, serial in sorted(robot_mocap_ids().items()):
                calibration, warnings = resolve(mocap_id, serial, root, folder)
                problems += len(warnings)
                lines += explain(mocap_id, serial, calibration, warnings, root, sessions=folder)
                if calibration is None:
                    log.error(box(f"rigid body {mocap_id} ({serial}): NO CALIBRATION for the {source}",
                                  "Its poses are published uncalibrated, as Motive gives them: not its "
                                  "base_footprint."))
                    continue
                calibrations[source][mocap_id] = calibration
        if problems:
            log.warn(box(f"calibrations at {why}: {problems} newer calibration(s) refused, see '! refused'",
                         "\n".join(lines)))
        else:
            log.info("\n".join([good(wrap(lines[0])), *(f"  {line}" for line in lines[1:])]))
        self._calibrations = calibrations
        self._defaults = {mocap_id: default for mocap_id in robot_mocap_ids()
                          if (default := default_calibration_or_none(mocap_id)) is not None}
        # * Every newest session looked at now counts as seen, applied or refused: only later ones are reported.
        self._reported_newer = {str(path.relative_to(root)) for folder in self.sessions_folders.values()
                                for path in (newest_session_file(root, serial, folder)
                                             for serial in set(robot_mocap_ids().values()))
                                if path is not None} if root is not None else set()
        self._calibrations_publisher.publish(String(data=json.dumps({
            "applied": why, "time": time.time(), "drive_root": str(root) if root else None,
            "calibrations": [c.as_dict() for c in calibrations[MOTIVE].values()],
            "sim_calibrations": [c.as_dict() for c in calibrations.get(FAKE_MOTIVE, {}).values()],
        })))
        return lines

    def _handle_reload(self, request, response):
        """Apply the newest calibrations now (service ~/reload_calibration)."""
        del request
        lines = self._load_calibrations("reload")
        response.success = True
        response.message = "\n".join(lines)
        return response

    def _watch_for_newer(self) -> None:
        """Log each newer Drive calibration once; ! never apply it (see the module docstring)."""
        if self.drive_root is None or not self.drive_root.is_dir():
            return
        for source, folder in self.sessions_folders.items():
            applied = self._calibrations[source]
            for mocap_id, serial in robot_mocap_ids().items():
                path = newest_session_file(self.drive_root, serial, folder)
                if path is None:
                    continue
                newer = str(path.relative_to(self.drive_root))
                current = applied.get(mocap_id)
                if current is not None and current.source == newer or newer in self._reported_newer:
                    continue
                self._reported_newer.add(newer)
                self.get_logger().warn(box(
                    f"rigid body {mocap_id} ({serial}): newer calibration on the Drive for the {source}, not applied",
                    f"newer:     {newer}\napplying:  {current.source if current else 'none'}\napply it with:\n"
                    f"  ros2 service call {self.get_fully_qualified_name()}/reload_calibration std_srvs/srv/Trigger"))

    # --- --- --- --- --- NATNET --- --- --- --- ---

    def _configure_mocap_client(self, client: NatNetClient) -> None:
        """Configure NatNet client transport and callbacks."""
        client.set_client_address(self.client_ip)
        client.set_server_address(self.server_ip)
        client.set_use_multicast(self.use_multicast)
        client.multicast_address = self.multicast_address
        client.print_level = 0
        client.rigid_body_listener = self._receive_rigid_body_frame
        client.labeled_marker_listener = self._receive_labeled_markers
        client.new_frame_listener = self._receive_mocap_frame

    def _refuse_if_relay_on_network(self) -> None:
        """Abort when another relay already runs in this ROS domain, on any PC.

        Two relays would publish the same topics interleaved, with different delays and possibly
        different calibrations. The ROS graph is per domain, so relays in other domains do not count.
        ? Two relays started within RELAY_DISCOVERY_SEC of each other can miss each other.

        Raises:
            RuntimeError: If another relay serves STREAMING_IDS_SERVICE.
        """
        deadline = time.monotonic() + RELAY_DISCOVERY_SEC
        while time.monotonic() < deadline:
            names = {name for name, _types in self.get_service_names_and_types()}
            if STREAMING_IDS_SERVICE in names:
                raise RuntimeError(
                    "Another mocap relay is already running in this ROS domain "
                    f"({STREAMING_IDS_SERVICE} is served). Not starting a second one; use the running relay.")
            time.sleep(0.1)

    def _start_mocap_client(self) -> None:
        """Start the NatNet client and wait briefly for connection."""
        self.get_logger().info(
            f"Starting mocap relay\n  NatNet: client {self.client_ip}, server {self.server_ip}, "
            f"{'multicast' if self.use_multicast else 'unicast'}\n  publishing at {self.publish_rate_hz:.0f} Hz, "
            f"a rigid body stops streaming after {self.streaming_stale_timeout_sec:.2f} s")
        # A failed start is not fatal: the health check keeps retrying.
        try:
            started = self.mocap_client.run()
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"Failed to start NatNet client: {exc}")
            return
        if not started:
            self.get_logger().error("Failed to start NatNet client threads.")
            return
        start_time = time.monotonic()
        while (time.monotonic() - start_time) < 5.0 and not self.mocap_client.connected():
            time.sleep(0.05)
        if self.mocap_client.connected():
            self._last_frame_mono_sec = time.monotonic()
            self.get_logger().info(good("NatNet client connected."))
        else:
            self.get_logger().warn(wrap("NatNet client started, but its connection handshake did not finish in time."))

    def _receive_mocap_frame(self, data_dict) -> None:
        """Commit the frame's bodies, stamped with the frame's capture time."""
        now_mono = time.monotonic()
        self._last_frame_mono_sec = now_mono
        if self._pending_samples and not data_dict.get("labeled_marker_count") and not self._warned_no_markers:
            self._warned_no_markers = True
            self.get_logger().warn(wrap(
                "Motive streams no labeled markers; poses are published without raw markers. "
                "Enable 'Labeled Markers' in Motive's streaming settings to get them."))
        self._commit_frame(self._capture_time_ns(data_dict), now_mono)

    def _commit_frame(self, capture_time_ns: int, now_mono: float) -> None:
        """Make the held bodies the latest samples, stamped with their frame's capture time."""
        pending, self._pending_samples = self._pending_samples, {}
        with self._samples_lock:
            for rb_id, sample in pending.items():
                self._samples[rb_id] = dataclasses.replace(
                    sample, capture_time_ns=capture_time_ns, received_mono_sec=now_mono)

    def _capture_time_ns(self, data_dict) -> int:
        """Mid-exposure time of a frame on this PC's wall clock, in ns.

        Motive's hi-res ticks run on the mocap PC's own counter, not its wall clock, so they only give
        the latency from mid-exposure to transmit; that is subtracted from when the packet arrived here.
        The network hop left in the stamp is well under a millisecond on the wired LAN.
        ! Comparable with stamps from other machines only while this PC is time synced (crl-husky TIME_SYNC.md).
        """
        receive_time_ns = int(data_dict.get("receive_time_ns") or time.time_ns())
        frequency = self.mocap_client.get_high_res_clock_frequency()
        mid_exposure = data_dict.get("stamp_camera_mid_exposure", -1)
        transmit = data_dict.get("stamp_transmit", -1)
        if frequency <= 0 or mid_exposure < 0 or transmit < 0:
            if not self._warned_no_hires_stamps:
                self._warned_no_hires_stamps = True
                self.get_logger().warn(wrap(
                    "NatNet sends no hi-res timestamps (needs NatNet 3+). Stamping poses with their arrival time, "
                    "which is late by Motive's latency."))
            return receive_time_ns
        latency_sec = (transmit - mid_exposure) / frequency
        if not 0.0 <= latency_sec <= MAX_PLAUSIBLE_LATENCY_SEC:
            self.get_logger().warn(
                f"Implausible Motive latency {latency_sec * 1e3:.1f} ms; stamping frame with its arrival time.",
                throttle_duration_sec=10.0)
            return receive_time_ns
        if not self._logged_latency:
            self._logged_latency = True
            self.get_logger().info(f"Motive latency (mid-exposure to transmit): {latency_sec * 1e3:.1f} ms")
        return receive_time_ns - int(latency_sec * 1e9)

    def _check_mocap_health(self) -> None:
        """Reconnect while no frames arrive, retrying until they do.

        ! Runs on the executor thread, like publishing. Nothing in here may
          block, or the relay goes silent instead of publishing invalid poses.
        """
        now = time.monotonic()
        if now - self._last_frame_mono_sec <= self.frame_timeout_sec:
            if self._reconnect_attempts > 0:
                self.get_logger().info(good(f"NatNet frames resumed after {self._reconnect_attempts} reconnect(s)."))
                self._reconnect_attempts = 0
            return
        if now - self._last_reconnect_attempt_mono < self.reconnect_min_interval_sec:
            return
        self._last_reconnect_attempt_mono = now
        self._reconnect_attempts += 1
        # Throttled: during a long outage this fires every reconnect interval. Only info while a simulation streams:
        # away from the lab Motive never comes. ! rclpy refuses to change a log call's severity: one call each.
        message = (f"No NatNet frames for {now - self._last_frame_mono_sec:.2f} s; "
                   f"reconnecting (attempt {self._reconnect_attempts}).")
        if self._simulated_streaming():
            self.get_logger().info(message, throttle_duration_sec=10.0)
        else:
            self.get_logger().warn(message, throttle_duration_sec=10.0)
        self._restart_mocap_client()

    def _restart_mocap_client(self) -> None:
        """Replace the NatNet client with a fresh one, without waiting for it.

        Whether it worked shows up as frames arriving; if they do not, the next health check simply tries again.
        """
        with self._mocap_client_lock:
            try:
                self.mocap_client.shutdown()
            except Exception:  # pylint: disable=broad-except
                pass
            self.mocap_client = NatNetClient()
            self._configure_mocap_client(self.mocap_client)
            # ! While the cable is out the local IP can be gone, and socket setup
            #   raises. Uncaught, that would end rclpy.spin and the relay with it.
            try:
                self.mocap_client.run()
            except Exception as exc:  # pylint: disable=broad-except
                message = wrap(f"NatNet client restart failed: {exc}")
                if self._simulated_streaming():
                    self.get_logger().info(message, throttle_duration_sec=10.0)
                else:
                    self.get_logger().warn(message, throttle_duration_sec=10.0)

    def _receive_rigid_body_frame(self, rigid_body_id: int, pos_xyz_y_up: Vec3, quat_xyzw_y_up: QuatXyzw,
                                  tracking_valid: bool = True, marker_error: float = 0.0) -> None:
        """Hold a rigid body pose from NatNet until its frame completes (NatNet thread).

        NatNet data is Y-up; this remaps to the Z-up world frame shared with
        Rhino and PyBullet ('rhino' convention, a +90 deg rotation about mocap x):
        (x, y, z, qx, qy, qz, qw) -> (x, -z, y, qx, -qz, qy, qw)

        Calibration files must be expressed in this convention too.
        """
        pos = (pos_xyz_y_up[0], -pos_xyz_y_up[2], pos_xyz_y_up[1])
        quat = (quat_xyzw_y_up[0], -quat_xyzw_y_up[2], quat_xyzw_y_up[1], quat_xyzw_y_up[3])
        self._hold_body(int(rigid_body_id), pos, quat, tracking_valid, marker_error)

    def _hold_body(self, rigid_body_id: int, pos: Vec3, quat: QuatXyzw, tracking_valid: bool,
                   marker_error: float) -> None:
        """Hold one of Motive's Z-up poses, calibrated if it is a robot's, until its frame is committed."""
        pos, quat = self._calibrated(rigid_body_id, pos, quat, self._calibrations[MOTIVE])
        # Committed once the frame's timestamps arrive, in _receive_mocap_frame.
        self._pending_samples[rigid_body_id] = BodySample(pos, quat, bool(tracking_valid), float(marker_error))

    def _calibrated(self, rigid_body_id: int, pos: Vec3, quat: QuatXyzw,
                    calibrations: Dict[int, Calibration]) -> Tuple[Vec3, QuatXyzw]:
        """A raw body's pose as its robot's base_footprint, or unchanged (logged once) if it is no robot's."""
        calibration = calibrations.get(rigid_body_id)
        if calibration is not None:
            return apply_base_calibration(calibration, pos, quat)
        if rigid_body_id not in self._logged_uncalibrated:
            self._logged_uncalibrated.add(rigid_body_id)
            self.get_logger().info(f"rigid body {rigid_body_id}: no robot's, published as Motive gives it")
        return pos, quat

    def _receive_labeled_markers(self, labeled_marker_from_model_id) -> None:
        """Attach the frame's labeled markers to its rigid bodies held until the frame completes.

        NatNet sends them after the rigid bodies; markers of bodies not held are dropped.
        Positions get the same Y-up to Z-up remap as poses, but no calibration.
        """
        for rigid_body_id, markers in labeled_marker_from_model_id.items():
            sample = self._pending_samples.get(int(rigid_body_id))
            if sample is None:
                continue
            converted = tuple(
                Marker(member_id=int(member_id),
                       pos=(marker["pos"][0], -marker["pos"][2], marker["pos"][1]),
                       residual=float(marker["error"]),
                       measured=bool(marker["param"] & MARKER_POINT_CLOUD_SOLVED)
                       and not marker["param"] & MARKER_OCCLUDED)
                for member_id, marker in sorted(markers.items()))
            self._pending_samples[int(rigid_body_id)] = dataclasses.replace(sample, markers=converted)

    # --- --- --- --- --- PUBLISHING --- --- --- --- ---

    def _current_samples(self, now_mono: float) -> Dict[int, Tuple[str, BodySample]]:
        """Per rigid body, the sample to publish and its source.

        The first source by priority that still streams, else the one heard last.
        """
        cutoff = now_mono - self.streaming_stale_timeout_sec
        with self._samples_lock:
            by_priority = [(FAKE_MOTIVE, dict(self._sim_samples[FAKE_MOTIVE])), (TRUTH, dict(self._sim_samples[TRUTH])),
                           (MOTIVE, dict(self._samples))]
        chosen: Dict[int, Tuple[str, BodySample]] = {}
        for source, samples in by_priority:
            for rb_id, sample in samples.items():
                current = chosen.get(rb_id)
                if current is None or (current[1].received_mono_sec < cutoff
                                       and sample.received_mono_sec > current[1].received_mono_sec):
                    chosen[rb_id] = (source, sample)
        return chosen

    def _get_streaming_ids(self, now_mono: float) -> List[int]:
        cutoff = now_mono - self.streaming_stale_timeout_sec
        return sorted(rb_id for rb_id, (_source, sample) in self._current_samples(now_mono).items()
                      if sample.received_mono_sec >= cutoff)

    def _publish_cached_poses(self) -> None:
        now = time.monotonic()
        cutoff = now - self.streaming_stale_timeout_sec
        for rb_id, (source, sample) in self._current_samples(now).items():
            if self._sources.get(rb_id, source) != source:
                self.get_logger().info(f"rigid body {rb_id}: now from the {source}, was the {self._sources[rb_id]}")
            self._sources[rb_id] = source
            pose_valid = (sample.received_mono_sec >= cutoff and sample.tracking_valid
                          and sample.marker_error <= self.marker_error_valid_threshold)
            publisher = self._rb_publishers.get(rb_id)
            if publisher is None:
                # ROS tokens cannot start with a number, so prefix rigid-body IDs.
                topic = f"{self.topic_prefix}/id_{rb_id}/pose"
                publisher = self.create_publisher(MocapRigidBodyPose, topic, 10)
                self._rb_publishers[rb_id] = publisher
                self.get_logger().info(f"Created mocap publisher for rigid body id={rb_id} on topic: {topic}, "
                                       f"from the {source}")
            msg = MocapRigidBodyPose()
            # Capture time, so a pose republished while no new frame arrived keeps its age.
            msg.header.stamp = Time(sec=sample.capture_time_ns // 1_000_000_000,
                                    nanosec=sample.capture_time_ns % 1_000_000_000)
            msg.header.frame_id = self.frame_id
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in sample.pos)
            (msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w) = (float(v) for v in sample.quat)
            msg.pose_valid = bool(pose_valid)
            msg.tracking_valid = sample.tracking_valid
            msg.marker_error = sample.marker_error
            msg.marker_ids = [marker.member_id for marker in sample.markers]
            msg.marker_positions = [Point(x=float(m.pos[0]), y=float(m.pos[1]), z=float(m.pos[2]))
                                    for m in sample.markers]
            msg.marker_residuals = [marker.residual for marker in sample.markers]
            msg.marker_measured = [marker.measured for marker in sample.markers]
            publisher.publish(msg)

    def _handle_get_streaming_ids(self, request, response):
        del request
        ids = self._get_streaming_ids(time.monotonic())
        response.ids = ids
        response.count = len(ids)
        response.query_time = self.get_clock().now().to_msg()
        return response

    def destroy_node(self) -> None:
        """Shut the NatNet client down, then the node."""
        if hasattr(self, "mocap_client") and self.mocap_client is not None:
            with self._mocap_client_lock:
                try:
                    self.mocap_client.shutdown()
                except BaseException as exc:  # pylint: disable=broad-except
                    # During Ctrl-C shutdown, thread joins can be interrupted; keep teardown best-effort.
                    if rclpy.ok():
                        self.get_logger().warn(f"Exception during NatNet shutdown: {exc}")
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    """Run the relay until Ctrl-C; exit 1 if another relay already runs in this domain."""
    rclpy.init(args=args)
    node = None
    try:
        node = MocapRelay()
    except RuntimeError as exc:
        rclpy.logging.get_logger("mocap_relay").error(box("mocap relay NOT STARTED", str(exc)))
        if rclpy.ok():
            rclpy.shutdown()
        # Non-zero, so launch reports the relay as died rather than finished.
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C, or SIGTERM through rclpy's signal handler
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass  # a second Ctrl-C during teardown: launch forwards one on top of the terminal's


if __name__ == "__main__":
    main()
