"""
ROS2 node that relays OptiTrack rigid body data into ROS topics.

It consumes NatNet rigid body callbacks at mocap frequency and republishes the latest pose of each rigid body at a
lower, timer-driven rate, with the raw markers Motive labeled to it in the same frame. Each pose is stamped with when
the cameras captured it, on this PC's clock, and a robot's rigid body is turned into its base_footprint pose with
its base calibration (`world/calibration_sessions.py`).

- Services: `/mocap/get_streaming_ids` (which ids stream), `~/reload_base_calibrations` (apply the newest base
  calibrations), `~/reload_base_calibration/id_<n>` (the newest of one robot's rigid body only, the others kept),
  `~/reload_floor_calibration` (apply the newest floor calibration).
- ! Threads: NatNet's, and two executor threads. Publishing poses has its own callback group, so a slow look or reload
  never holds the poses back; it only reads what the others lock or replace whole. Every other callback runs one at a
  time, as on a single thread.
- ! One mode for the relay's life, never mixed (`sim`): off, it reads Motive over NatNet only, with the robots'
  sessions; on (refused in the robots' domain), only simulated inputs (SIM_SOURCES), with the simulation's sessions of
  its domain (`calibration_sessions.sessions_folder`), base and floor calibrations alike. Every calibrations
  announcement says which (`simulated`), so whatever is measured through the relay is all real or all simulated.
- `~/calibrations` (latched): what is applied to each rigid body, with its source, hash, and whether the body
  definition it was built for still matches Motive's (a wrong one is warned about, and still applied); and where each
  body streams from now (Motive, or with `sim` a fake Motive or a simulator's truth).
- `~/body_definitions`, `~/cameras` (latched): Motive's rigid body definitions and camera poses, asked for every
  `definitions_period_sec` and published when they change; each change is logged. With `sim` a fake Motive's
  announcements (SIM_DEFINITIONS_TOPIC, SIM_CAMERAS_TOPIC) instead.
- Layers: every pose, marker and camera of Motive (or a fake Motive) is turned Z-up ("raw"), then put in the lab frame
  by `lab_from_raw`: the newest floor calibration's floor_from_raw and the refit in use (`world/mocap_frame.py`); a
  robot's base calibration goes on top.
- `~/unlabeled_markers`, `~/anchors` (latched, every `definitions_period_sec`): Motive's unlabeled markers, each
  averaged since the last look, and among them the floor calibration's anchors, found where it saw them.
- Following Motive's frame (`world/frame_follower.py`, which also decides what is logged): every look fits the anchors
  and the truss cameras against the floor calibration's; `~/frame` announces the state, and only `~/use_refit` takes a
  pending refit into lab_from_raw, in memory only. `~/frame_metrics` carries both fits every look, with the look's
  raw inputs and each anchor's quality (`scripts/mocap_frame_plot.py`); `frame_log` keeps them, a file per day.
- ! Calibrations are applied at start and on their reload service only. A newer one appearing on the Drive is
  logged, never applied by itself: a robot driving its path onboard would jump.
- Simulated inputs are found by their topics: per rigid body, a fake Motive's body wins over a simulator's true pose
  while it streams. A truth needs no calibration. A fake Motive's robot base comes relative to crl-husky's default
  calibration: that is taken off, then the simulation's calibration applied. `~/frame` names the sessions folder,
  where the probe writes new floor calibrations.
"""

from __future__ import annotations

import dataclasses
import json
import re
import threading
import time
import traceback
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from crl_husky_msgs.msg import MocapRigidBodyPose
from crl_husky_msgs.srv import GetStreamingMocapIds
from geometry_msgs.msg import Point
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from std_srvs.srv import Trigger

from crl_husky import config_resolver
from crl_husky.sim.husky_sim import TRUTH_PREFIX
from crl_husky.simulated_parts import fleet_domain_problem
from scaffolding_assembly_core.geometry import Pose, invert
from ..drive import drive_root
from ..robot_interface.mocap_definitions import (MATCHES, OUT_OF_DATE, UNKNOWN, BodyDefinition, MocapAnchor,
                                                 MocapCamera, anchors_json, camera_changes, cameras_json,
                                                 definition_check, definitions_json, match, read_cameras,
                                                 read_definitions)
from ..robot_interface.mocap_relay import FAKE_MOTIVE, MOTIVE, TRUTH
from ..world.calibration_sessions import (DESCRIBE_HEADER, Calibration, default_calibration_or_none, explain,
                                          newest_session_file, resolve, robot_mocap_ids, sessions_folder)
from ..world.frame_follower import FrameEvent, FrameFollower
from ..world.mocap_frame import (TRUSS_CAMERAS, VOLUME_MOVED, camera_label, frame_and_cameras, frame_moved,
                                 moved_camera, newest_floor_calibration, truss_names, volume_moved)
from .body_definitions import DescriptionClient, Reply
from .frame_log import DEFAULT_FOLDER, FrameLog, frame_log_folder
from .log_text import box, good, wrap

#: The relay's service; whoever serves it is a running relay.
STREAMING_IDS_SERVICE = "/mocap/get_streaming_ids"
#: Seconds to look for another relay on the network before starting.
#: ? The ROS graph fills in shortly after a node starts, so an immediate look can miss it.
RELAY_DISCOVERY_SEC = 2.0
#: Where a fake Motive (`sim_motive.py`) publishes its bodies: <prefix>/id_<n>/pose, Z-up world; a robot's base
#: relative to crl-husky's default calibration.
SIM_MOTIVE_PREFIX = "/mocap_sim/motive"
#: The simulated sources by topic prefix, read only with `sim`, highest priority first: a fake Motive, a simulator's
#: true base_footprint (crl-husky's husky_sim, on TRUTH_PREFIX).
SIM_SOURCES = {SIM_MOTIVE_PREFIX: FAKE_MOTIVE, TRUTH_PREFIX: TRUTH}
SIM_TOPIC = re.compile(rf"^({'|'.join(re.escape(prefix) for prefix in SIM_SOURCES)})/id_(\d+)/pose$")
#: Where a fake Motive announces, latched, its bodies' definitions and its cameras, Z-up, as the relay announces them.
SIM_DEFINITIONS_TOPIC, SIM_CAMERAS_TOPIC = f"{SIM_MOTIVE_PREFIX}/body_definitions", f"{SIM_MOTIVE_PREFIX}/cameras"
#: Seconds between looks for new simulated inputs.
SIM_DISCOVERY_SEC = 1.0
#: Motive latencies outside [0, this] are treated as garbage; the frame is stamped with its arrival instead.
MAX_PLAUSIBLE_LATENCY_SEC = 0.5
#: The poses' header.frame_id: in the lab frame (by a floor calibration, or a simulator's truth), or raw (Motive's,
#: Z-up).
LAB_FRAME, RAW_FRAME = "lab", "mocap_raw"
#: The relay's announcements: the last one reaches every subscriber, also one that comes later.
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
#: NatNet labeled marker flags: hidden from the cameras, position measured by the cameras, and on no asset.
MARKER_OCCLUDED = 0x01
MARKER_POINT_CLOUD_SOLVED = 0x02
MARKER_UNLABELED = 0x10

Vec3 = Tuple[float, float, float]
QuatXyzw = Tuple[float, float, float, float]


@dataclasses.dataclass(frozen=True)
class Marker:
    """One raw marker Motive labeled to a rigid body, in the lab frame (no robot calibration).

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
    """One rigid body in one mocap frame, in the lab frame and calibrated if possible.

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


def z_up(point: Vec3) -> Vec3:
    """A point or vector in NatNet's Y-up frame turned into the Z-up world: (x, y, z) -> (x, -z, y)."""
    return point[0], -point[2], point[1]


def z_up_quat(quat: QuatXyzw) -> QuatXyzw:
    """An orientation (x, y, z, w) in NatNet's Y-up frame turned into the Z-up world: (x, -z, y, w)."""
    return quat[0], -quat[2], quat[1], quat[3]


def z_up_body(body: BodyDefinition) -> BodyDefinition:
    """A rigid body's definition with its offsets in the frame the relay's poses give it."""
    return dataclasses.replace(body, offsets=tuple(z_up(offset) for offset in body.offsets))


def z_up_camera(camera: MocapCamera) -> MocapCamera:
    """A camera's pose turned into the Z-up world."""
    return MocapCamera(camera.name, z_up(camera.position), z_up_quat(camera.orientation))


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


def simulated_mode(sim: bool) -> bool:
    """The `sim` parameter, checked: a simulation's relay (simulated inputs only) rather than Motive's.

    Raises:
        RuntimeError: If `sim` is on in the robots' domain, where nothing may stand in for a robot.
    """
    problem = fleet_domain_problem()
    if sim and problem:
        raise RuntimeError(f"sim:=true refused: {problem}")
    return bool(sim)


class MocapRelay(Node):
    """Relays mocap poses, serves the streaming rigid body ids, and applies and reports base calibrations."""

    def __init__(self) -> None:
        """Declare the parameters, load the calibrations and start the NatNet client.

        Raises:
            RuntimeError: If another relay already runs in this ROS domain, or `sim` is on in the robots' domain.
        """
        super().__init__("mocap_relay")

        self._refuse_if_relay_on_network()

        self.declare_parameter("client_ip", "192.168.0.7")
        self.declare_parameter("server_ip", "192.168.0.117")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("topic_prefix", "/mocap/rigid_body")
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
        # * What the relay reads, for its whole life: Motive over NatNet, or with `sim` a simulation's inputs only.
        self.declare_parameter("sim", False)
        # * How often to ask Motive for its body definitions and cameras, seconds; 0 never.
        self.declare_parameter("definitions_period_sec", 2.0)
        # * The cameras bolted to the truss, by serial, that tell how Motive's frame moved (`world/mocap_frame.py`).
        self.declare_parameter("truss_cameras", ",".join(TRUSS_CAMERAS))
        # * Where every `~/frame_metrics` sample is kept, a file per day (`frame_log.py`), and for how many days;
        # * empty: not kept.
        self.declare_parameter("frame_log", DEFAULT_FOLDER)
        self.declare_parameter("frame_log_days", 30)

        self.client_ip = self.get_parameter("client_ip").value
        self.server_ip = self.get_parameter("server_ip").value
        self.publish_rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        self.topic_prefix = str(self.get_parameter("topic_prefix").value).rstrip("/")
        self.streaming_stale_timeout_sec = max(float(self.get_parameter("streaming_stale_timeout_sec").value), 0.01)
        self.marker_error_valid_threshold = max(float(self.get_parameter("marker_error_valid_threshold").value), 0.0)
        self.use_multicast = bool(self.get_parameter("use_multicast").value)
        self.multicast_address = str(self.get_parameter("multicast_address").value)
        self.frame_timeout_sec = max(float(self.get_parameter("frame_timeout_sec").value), 0.1)
        self.reconnect_min_interval_sec = max(float(self.get_parameter("reconnect_min_interval_sec").value), 0.1)
        self.drive_root = (drive_root(str(self.get_parameter("drive_root").value))
                           if bool(self.get_parameter("use_drive").value) else None)
        #: Whether the relay reads a simulation's inputs, not Motive. ! Never in the robots' domain.
        self.simulated = simulated_mode(bool(self.get_parameter("sim").value))
        #: The base and floor calibrations' sessions folder below the Drive root.
        self._sessions = sessions_folder(self.simulated)
        if self.simulated and self.drive_root is not None and self.drive_root.is_dir():
            (self.drive_root / self._sessions).mkdir(parents=True, exist_ok=True)

        truss = tuple(serial.strip() for serial in str(self.get_parameter("truss_cameras").value).split(",")
                      if serial.strip())
        self._start_state(truss)
        folder = frame_log_folder(str(self.get_parameter("frame_log").value), self.simulated)
        if folder is not None:
            self._frame_log = FrameLog(folder, int(self.get_parameter("frame_log_days").value))
            self.get_logger().info(wrap(f"frame log: every look's frame metrics in {folder}/<YYYYMMDD>.jsonl, kept "
                                        f"{self._frame_log.days} days"))
        self._load_calibrations("start")
        self._load_floor_calibration("start")
        self._reload_service = self.create_service(Trigger, "~/reload_base_calibrations", self._handle_reload)
        self._reload_one_services = [
            self.create_service(Trigger, f"~/reload_base_calibration/id_{mocap_id}",
                                lambda request, response, mocap_id=mocap_id: self._handle_reload(request, response,
                                                                                                 mocap_id))
            for mocap_id in robot_mocap_ids()]
        self._reload_floor_service = self.create_service(Trigger, "~/reload_floor_calibration",
                                                         self._handle_reload_floor)
        self._use_refit_service = self.create_service(Trigger, "~/use_refit", self._handle_use_refit)
        watch_period = float(self.get_parameter("watch_period_sec").value)
        if watch_period > 0 and self.drive_root is not None:
            self._watch_timer = self.create_timer(watch_period, self._watch_for_newer)

        self._service = self.create_service(GetStreamingMocapIds, STREAMING_IDS_SERVICE,
                                            self._handle_get_streaming_ids)
        # * Its own group: a look can take half a second, longer than a robot may go without a pose.
        self._publish_timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_cached_poses,
                                                callback_group=MutuallyExclusiveCallbackGroup())
        if self.simulated:
            inputs = ", ".join(f"{prefix}/id_<n>/pose" for prefix in SIM_SOURCES)
            self.get_logger().info(wrap(f"sim:=true: reading only simulated inputs ({inputs}) as they appear, never "
                                        f"Motive; calibrations from {self._sessions}"))
            self.mocap_client = None
            self._sim_timer = self.create_timer(SIM_DISCOVERY_SEC, self._look_for_simulated_inputs)
            self.create_subscription(String, SIM_DEFINITIONS_TOPIC, self._receive_sim_definitions, LATCHED)
            self.create_subscription(String, SIM_CAMERAS_TOPIC, self._receive_sim_cameras, LATCHED)
            return
        self._health_timer = self.create_timer(0.5, self._check_mocap_health)
        definitions_period = float(self.get_parameter("definitions_period_sec").value)
        if definitions_period > 0:
            self._definitions_timer = self.create_timer(definitions_period, self._poll_definitions)
        self.mocap_client = DescriptionClient()
        self._configure_mocap_client(self.mocap_client)
        self._start_mocap_client()

    # * Every callback rclpy runs goes through `_survives`: an uncaught error would end rclpy.spin, and with no respawn
    #   the relay would stay down, every robot's pose with it.
    def create_timer(self, timer_period_sec, callback, *args, **kwargs):
        """Node.create_timer, with `callback` guarded (`_survives`)."""
        return super().create_timer(timer_period_sec, self._survives(callback), *args, **kwargs)

    def create_service(self, srv_type, srv_name, callback, *args, **kwargs):
        """Node.create_service, with `callback` guarded (`_survives`)."""
        return super().create_service(srv_type, srv_name, self._survives(callback), *args, **kwargs)

    def create_subscription(self, msg_type, topic, callback, *args, **kwargs):
        """Node.create_subscription, with `callback` guarded (`_survives`)."""
        return super().create_subscription(msg_type, topic, self._survives(callback), *args, **kwargs)

    def _survives(self, callback):
        """`callback`, logging an error it raises with its traceback instead of stopping the relay.

        A service's answer (its second argument) is returned failed, with the error.
        """
        def run(*args):
            try:
                return callback(*args)
            except Exception as error:  # * any error, by design: the relay must go on relaying poses
                self.get_logger().error(f"{getattr(callback, '__name__', 'a callback')} failed, the relay goes on: "
                                        f"{error!r}\n{traceback.format_exc()}", throttle_duration_sec=10.0)
                if len(args) != 2:
                    return None
                response = args[1]
                if hasattr(response, "success"):
                    response.success, response.message = False, f"failed: {error!r} (the relay's log has more)"
                return response
        return run

    def _start_state(self, truss_serials: Tuple[str, ...]) -> None:
        """Set up what the relay keeps and its publishers, from the parameters read; no services, timers or NatNet."""
        # Latest sample per rigid body id: Motive's, written by the NatNet thread, and the simulated sources', written
        # by the executor; all read by the executor.
        self._samples_lock = threading.Lock()
        self._samples: Dict[int, BodySample] = {}
        self._sim_samples: Dict[str, Dict[int, BodySample]] = {source: {} for source in SIM_SOURCES.values()}
        # Simulated topics subscribed to, and the source each rigid body was last published from.
        # ! The sources are written by publishing only and read by the announcements: replaced whole.
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
        # ! Read by the NatNet thread: always replaced whole (one assignment), never edited in place. By rigid body,
        # ! for Motive's bodies, or with `sim` the fake Motive's.
        self._calibrations: Dict[int, Calibration] = {}
        #: crl-husky's default calibration by robot's rigid body, which a fake Motive's bases are relative to.
        self._defaults: Dict[int, Calibration] = {}
        # Newer Drive files already reported by the watch, so each is logged once.
        self._reported_newer: set[str] = set()
        # Motive's body definitions and cameras as last published (None before Motive's first reply), the cameras as
        # last logged (a camera's move is logged once it adds up), and the description problems last logged.
        self._definitions: Optional[Dict[int, BodyDefinition]] = None
        self._cameras: Optional[Dict[str, MocapCamera]] = None
        self._cameras_logged: Dict[str, MocapCamera] = {}
        self._definition_problems: Tuple[str, ...] = ()
        # Motive's newest reply, turned Z-up: (bodies, cameras, problems), or None; a fake Motive's announcements.
        self._motive_definitions: Optional[tuple] = None
        self._fake_definitions: Dict[int, BodyDefinition] = {}
        self._fake_cameras: Dict[str, MocapCamera] = {}
        # How each calibration compares with the body definition now, by rigid body (`definition_check`), and what the
        # last announcement said besides the calibrations.
        self._definition_checks: Dict[int, Tuple[str, str]] = {}
        self._announced: Dict[str, object] = {}
        # The frame follower: the floor calibration applied, its fits, its state.
        self._frame = FrameFollower(truss_serials, f"Use refit in the mocap probe, or:\n  ros2 service call "
                                                   f"{self.get_fully_qualified_name()}/use_refit std_srvs/srv/Trigger")
        # ! Read by the NatNet thread: replaced whole. lab_from_raw as (rotation matrix, shift, quaternion), or None.
        self._lab_from_raw: Optional[Tuple[np.ndarray, np.ndarray, QuatXyzw]] = None
        # Motive's unlabeled markers since the last look, summed by Motive's id as [x, y, z, frames, x² + y² + z²]
        # (raw; the NatNet thread adds under its lock).
        self._loose_sums: Dict[int, List[float]] = {}
        self._loose_lock = threading.Lock()
        # Where each look's metrics sample is kept (executor only), or None.
        self._frame_log: Optional[FrameLog] = None

        latched = LATCHED
        # TODO: a proper message in crl_husky_msgs; a JSON string for now (temporary).
        self._calibrations_publisher = self.create_publisher(String, "~/calibrations", latched)
        self._definitions_publisher = self.create_publisher(String, "~/body_definitions", latched)
        self._cameras_publisher = self.create_publisher(String, "~/cameras", latched)
        self._frame_publisher = self.create_publisher(String, "~/frame", latched)
        self._metrics_publisher = self.create_publisher(String, "~/frame_metrics", 10)
        self._anchors_publisher = self.create_publisher(String, "~/anchors", latched)
        self._markers_publisher = self.create_publisher(String, "~/unlabeled_markers", latched)

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
            self.get_logger().info(wrap(f"rigid body {rigid_body_id}: reading the {source} on {topic}"))

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
            pos, quat = self._calibrated(rigid_body_id, *self._in_lab_frame(pos, quat), self._calibrations)
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        sample = BodySample(pos, quat, bool(message.tracking_valid), float(message.marker_error),
                            capture_time_ns=stamp_ns or time.time_ns(), received_mono_sec=time.monotonic())
        with self._samples_lock:
            self._sim_samples[source][rigid_body_id] = sample

    def _receive_sim_definitions(self, message: String) -> None:
        """Take a fake Motive's body definitions: they win over Motive's per body. One that does not read is skipped."""
        try:
            self._fake_definitions = read_definitions(message.data)
        except ValueError as error:
            self.get_logger().warn(wrap(f"fake Motive's body definitions skipped: {error}"), throttle_duration_sec=10.0)
            return
        self._use_definitions()

    def _receive_sim_cameras(self, message: String) -> None:
        """Take a fake Motive's cameras: they replace Motive's. One that does not read is skipped."""
        try:
            self._fake_cameras = read_cameras(message.data)
        except ValueError as error:
            self.get_logger().warn(wrap(f"fake Motive's cameras skipped: {error}"), throttle_duration_sec=10.0)
            return
        self._use_definitions()

    # --- --- --- --- --- CALIBRATIONS --- --- --- --- ---

    def _load_calibrations(self, why: str, only: Optional[int] = None) -> List[str]:
        """Resolve every robot's body, or with `only` that rigid body's alone, apply, log and publish; return the lines.

        With `only`, the other robots keep the calibration they have, even if a newer one is on the Drive.
        """
        log = self.get_logger()
        root = self.drive_root if self.drive_root is not None and self.drive_root.is_dir() else None
        if self.drive_root is None:
            where = "no Drive (HUSKY_DRIVE_ROOT and drive_root unset, or use_drive off): crl-husky defaults only"
        elif root is None:
            where = f"the Drive root is not a folder, crl-husky defaults only: {self.drive_root}"
        else:
            where = f"Drive root {root}"
        config_resolver.reload_robot_configs()
        calibrations: Dict[int, Calibration] = {} if only is None else {
            mocap_id: calibration for mocap_id, calibration in self._calibrations.items() if mocap_id != only}
        source, folder = (FAKE_MOTIVE if self.simulated else MOTIVE), self._sessions
        lines = [f"calibrations at {why}, {where}",
                 "crl-husky: config/robots/; only each robot's newest Drive session is looked at",
                 "", f"{source}'s bodies, sessions in {folder}:", DESCRIBE_HEADER]
        problems = 0
        resolved = {mocap_id: serial for mocap_id, serial in robot_mocap_ids().items() if only in (None, mocap_id)}
        for mocap_id, serial in sorted(resolved.items()):
            calibration, warnings = resolve(mocap_id, serial, root, folder)
            problems += len(warnings)
            lines += explain(mocap_id, serial, calibration, warnings, root, sessions=folder)
            if calibration is None:
                log.error(box(f"rigid body {mocap_id} ({serial}): NO CALIBRATION for the {source}",
                              "Its poses are published uncalibrated, as Motive gives them: not its base_footprint."))
                continue
            calibrations[mocap_id] = calibration
        if problems:
            log.warn(box(f"calibrations at {why}: {problems} newer calibration(s) refused, see '! refused'",
                         "\n".join(lines)))
        else:
            log.info("\n".join([good(wrap(lines[0])), *(f"  {line}" for line in lines[1:])]))
        self._calibrations = calibrations
        self._defaults = {mocap_id: default for mocap_id in robot_mocap_ids()
                          if (default := default_calibration_or_none(mocap_id)) is not None}
        # * Every newest session looked at now counts as seen, applied or refused: only later ones are reported.
        seen = {str(path.relative_to(root)) for path in (newest_session_file(root, serial, folder)
                                                         for serial in set(resolved.values()))
                if path is not None} if root is not None else set()
        self._reported_newer = seen if only is None else self._reported_newer | seen
        self._announced = {"applied": why, "time": time.time(), "drive_root": str(root) if root else None}
        # * Checked afresh, so a calibration still wrong after a reload is reported again.
        self._definition_checks = {}
        if not self._check_calibrations():
            self._announce_calibrations()
        return lines

    def _check_calibrations(self) -> bool:
        """Compare each calibration with the body definition now; log and announce what changed.

        Returns:
            bool: Whether anything changed (and was announced).
        """
        checks = {mocap_id: definition_check(calibration.body_definition, mocap_id, self._definitions)
                  for mocap_id, calibration in self._calibrations.items()}
        if checks == self._definition_checks:
            return False
        log = self.get_logger()
        for mocap_id, (status, detail) in sorted(checks.items()):
            before = self._definition_checks.get(mocap_id, ("", ""))[0]
            calibration = self._calibrations[mocap_id]
            body = f"rigid body {mocap_id}" + (f" of the {FAKE_MOTIVE}" if self.simulated else "")
            if status == OUT_OF_DATE and before != OUT_OF_DATE:
                log.warn(box(f"{body} ({calibration.robot_serial}): CALIBRATION FOR AN EARLIER BODY",
                             f"{detail}\napplied: {calibration.source}\nIts base pose is a few mm or degrees off: "
                             f"calibrate the robot again (calibration plugin). Still applied."))
            elif status == MATCHES and before != MATCHES:
                log.info(wrap(f"{body}: its calibration matches the body definition"))
            elif before == OUT_OF_DATE:
                log.info(wrap(f"{body}: its calibration is no longer known to be wrong: {detail}"))
        self._definition_checks = checks
        self._announce_calibrations()
        return True

    def _announce_calibrations(self) -> None:
        """Publish what is applied to each rigid body, with its definition check, and the mode (`~/calibrations`)."""
        entries = []
        for calibration in self._calibrations.values():
            status, detail = self._definition_checks.get(calibration.mocap_id, (UNKNOWN, "not checked"))
            entries.append({**calibration.as_dict(), "definition_check": {"status": status, "detail": detail}})
        self._calibrations_publisher.publish(String(data=json.dumps({
            **self._announced, "calibrations": entries, "simulated": self.simulated,
            "sources": {str(rb_id): source for rb_id, source in self._sources.items()}})))

    def _handle_reload(self, request, response, only: Optional[int] = None):
        """Apply the newest base calibrations now (~/reload_base_calibrations), or one rigid body's (`only`)."""
        del request
        lines = self._load_calibrations("reload" if only is None else f"reload of rigid body {only}", only)
        response.success = True
        response.message = "\n".join(lines)
        return response

    def _handle_reload_floor(self, request, response):
        """Apply the newest floor calibration now (service ~/reload_floor_calibration): every pose moves by it."""
        del request
        before = self._frame.lab_from_raw()
        self._load_floor_calibration("reload")
        after, error = self._frame.floor, self._frame.error
        response.success = not error
        if error:
            response.message = error
        elif after is None:
            response.message = "no floor calibration on the Drive: poses stay raw"
        else:
            moves = volume_moved(after.floor_from_raw, before if before is not None else Pose())
            response.message = (f"floor calibration {after.floor_file.name} applied: points of the lab's volume moved "
                                f"up to {moves * 1e3:.1f} mm")
        return response

    def _watch_for_newer(self) -> None:
        """Log each newer base or floor calibration once, ! never applying it (`_watch_floor` for the floor)."""
        if self.drive_root is None or not self.drive_root.is_dir():
            return
        self._watch_floor()
        for mocap_id, serial in robot_mocap_ids().items():
            path = newest_session_file(self.drive_root, serial, self._sessions)
            if path is None:
                continue
            newer = str(path.relative_to(self.drive_root))
            current = self._calibrations.get(mocap_id)
            if current is not None and current.source == newer or newer in self._reported_newer:
                continue
            self._reported_newer.add(newer)
            self.get_logger().warn(box(
                f"rigid body {mocap_id} ({serial}): newer calibration on the Drive, not applied",
                f"newer:     {newer}\napplying:  {current.source if current else 'none'}\napply it with:\n"
                f"  ros2 service call {self.get_fully_qualified_name()}/reload_base_calibration/id_{mocap_id} "
                f"std_srvs/srv/Trigger"))

    def _watch_floor(self) -> None:
        """Warn once about a newer floor calibration, how far it would move the lab's volume, and announce it.

        ! Never applied by itself: every pose would jump; `~/reload_floor_calibration` applies it. One that does not
        read yet (half-synced) is tried again at the next look.
        """
        follower = self._frame
        newer, _skipped = newest_floor_calibration(self.drive_root / self._sessions)
        current = follower.floor
        if newer is None or (current is not None and newer.floor_file.name <= current.floor_file.name):
            return
        if follower.newer is not None and follower.newer[0] == newer.floor_file.name:
            return
        moves = volume_moved(newer.floor_from_raw, follower.lab_from_raw() or Pose())
        follower.newer = (newer.floor_file.name, moves)
        title = (f"newer floor calibration on the Drive, not applied: it would move points of the lab's volume up to "
                 f"{moves * 1e3:.1f} mm")
        text = (f"newer:  {newer.floor_file.name}\nusing:  {current.floor_file.name if current else 'none'}\napply it, "
                f"while nothing moves, with Apply floor calibration in the mocap probe, or:\n"
                f"  ros2 service call {self.get_fully_qualified_name()}/reload_floor_calibration std_srvs/srv/Trigger")
        # * Within VOLUME_MOVED it changes no pose that matters: no warning, the mocap chips stay as they are.
        if current is not None and moves <= VOLUME_MOVED:
            self.get_logger().info("\n".join([wrap(title), *(f"  {line}" for line in text.splitlines())]))
        else:
            self.get_logger().warn(box(title, text))
        self._announce_frame_if_news()

    # --- --- --- --- --- NATNET --- --- --- --- ---

    def _configure_mocap_client(self, client: DescriptionClient) -> None:
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
            wrap(f"Starting mocap relay\n  NatNet: client {self.client_ip}, server {self.server_ip}, "
                 f"{'multicast' if self.use_multicast else 'unicast'}\n  publishing at {self.publish_rate_hz:.0f} Hz, "
                 f"a rigid body stops streaming after {self.streaming_stale_timeout_sec:.2f} s"))
        # A failed start is not fatal: the health check keeps retrying.
        try:
            started = self.mocap_client.run()
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(wrap(f"Failed to start NatNet client: {exc}"))
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
            # * Asked now, so the first look already has the cameras: the frame's first message then has both fits.
            try:
                self.mocap_client.request()
            except OSError as exc:
                self.get_logger().warn(wrap(f"Could not ask Motive for its body definitions: {exc}"))
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
                wrap(f"Implausible Motive latency {latency_sec * 1e3:.1f} ms; stamping frame with its arrival time."),
                throttle_duration_sec=10.0)
            return receive_time_ns
        if not self._logged_latency:
            self._logged_latency = True
            self.get_logger().info(wrap(f"Motive latency (mid-exposure to transmit): {latency_sec * 1e3:.1f} ms"))
        return receive_time_ns - int(latency_sec * 1e9)

    def _check_mocap_health(self) -> None:
        """Reconnect while no frames arrive, retrying until they do.

        ! Nothing in here may block: the looks, reloads and services wait for it.
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
        # Throttled: during a long outage this fires every reconnect interval.
        self.get_logger().warn(wrap(f"No NatNet frames for {now - self._last_frame_mono_sec:.2f} s; "
                                    f"reconnecting (attempt {self._reconnect_attempts})."), throttle_duration_sec=10.0)
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
            self.mocap_client = DescriptionClient()
            self._configure_mocap_client(self.mocap_client)
            # ! While the cable is out the local IP can be gone, and socket setup
            #   raises. Uncaught, that would end rclpy.spin and the relay with it.
            try:
                self.mocap_client.run()
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().warn(wrap(f"NatNet client restart failed: {exc}"), throttle_duration_sec=10.0)

    def _receive_rigid_body_frame(self, rigid_body_id: int, pos_xyz_y_up: Vec3, quat_xyzw_y_up: QuatXyzw,
                                  tracking_valid: bool = True, marker_error: float = 0.0) -> None:
        """Hold a rigid body pose from NatNet until its frame completes (NatNet thread).

        NatNet data is Y-up; this remaps it to raw Z-up, the convention shared with
        Rhino and PyBullet ('rhino' convention, a +90 deg rotation about mocap x):
        (x, y, z, qx, qy, qz, qw) -> (x, -z, y, qx, -qz, qy, qw)

        Calibration files must be expressed in this convention too.
        """
        self._hold_body(int(rigid_body_id), z_up(pos_xyz_y_up), z_up_quat(quat_xyzw_y_up), tracking_valid,
                        marker_error)

    def _hold_body(self, rigid_body_id: int, pos: Vec3, quat: QuatXyzw, tracking_valid: bool,
                   marker_error: float) -> None:
        """Hold one of Motive's Z-up poses, in the lab frame and calibrated if a robot's, until its frame commits."""
        pos, quat = self._calibrated(rigid_body_id, *self._in_lab_frame(pos, quat), self._calibrations)
        # Committed once the frame's timestamps arrive, in _receive_mocap_frame.
        self._pending_samples[rigid_body_id] = BodySample(pos, quat, bool(tracking_valid), float(marker_error))

    def _calibrated(self, rigid_body_id: int, pos: Vec3, quat: QuatXyzw,
                    calibrations: Dict[int, Calibration]) -> Tuple[Vec3, QuatXyzw]:
        """A raw body's pose as its robot's base_footprint, or unchanged if it is no robot's."""
        calibration = calibrations.get(rigid_body_id)
        return apply_base_calibration(calibration, pos, quat) if calibration is not None else (pos, quat)

    def _receive_labeled_markers(self, labeled_marker_from_model_id) -> None:
        """Attach the frame's labeled markers to its rigid bodies held until the frame completes.

        NatNet sends them after the rigid bodies; markers of bodies not held are dropped.
        Positions get the same Y-up to Z-up remap as poses, but no calibration. The unlabeled markers (model 0 or
        flagged unlabeled) are summed, averaged over every frame between two looks (`_check_anchors`).
        ! Take unlabeled markers by their flag too, keyed by the whole id: once Motive's unlabeled ids pass 65535 they
        spill into the model id half of NatNet's id.
        """
        with self._loose_lock:
            for model_id, markers in labeled_marker_from_model_id.items():
                for marker_id, marker in markers.items():
                    if (model_id == 0 or marker["param"] & MARKER_UNLABELED) and not marker["param"] & MARKER_OCCLUDED:
                        x, y, z = z_up(marker["pos"])
                        entry = self._loose_sums.setdefault(int(model_id) << 16 | int(marker_id),
                                                            [0.0, 0.0, 0.0, 0, 0.0])
                        entry[0] += x
                        entry[1] += y
                        entry[2] += z
                        entry[3] += 1
                        entry[4] += x * x + y * y + z * z
        for rigid_body_id, markers in labeled_marker_from_model_id.items():
            sample = self._pending_samples.get(int(rigid_body_id))
            if sample is None:
                continue
            converted = tuple(
                Marker(member_id=int(member_id),
                       pos=self._in_lab_frame(z_up(marker["pos"]))[0],
                       residual=float(marker["error"]),
                       measured=bool(marker["param"] & MARKER_POINT_CLOUD_SOLVED)
                       and not marker["param"] & MARKER_OCCLUDED)
                for member_id, marker in sorted(markers.items()) if not marker["param"] & MARKER_UNLABELED)
            self._pending_samples[int(rigid_body_id)] = dataclasses.replace(sample, markers=converted)

    # --- --- --- --- --- BODY DEFINITIONS AND CAMERAS --- --- --- --- ---

    def _poll_definitions(self) -> None:
        """Take Motive's newest reply, then ask again while connected; the reply comes on NatNet's thread."""
        with self._mocap_client_lock:
            client = self.mocap_client
            replies = client.take_replies()
            if client.connected():
                try:
                    client.request()
                except OSError as exc:
                    self.get_logger().warn(wrap(f"Could not ask Motive for its body definitions: {exc}"),
                                           throttle_duration_sec=10.0)
        # * The frame is checked at each look, with both the cameras and the anchors new: one message for both.
        if replies:
            self._take_definitions(replies[-1])
        self._check_anchors()

    def _take_definitions(self, reply: Reply) -> None:
        """Keep Motive's newest reply, turned Z-up, and use it (`_use_definitions`); log new problems reading it."""
        if tuple(reply.problems) != self._definition_problems:
            self._definition_problems = tuple(reply.problems)
            for problem in reply.problems:
                self.get_logger().warn(wrap(f"Motive's model descriptions: {problem}"))
        # * A reply that did not read is no news: Motive did not lose its bodies and cameras.
        if reply.problems and not reply.bodies and not reply.cameras and self._motive_definitions is not None:
            return
        self._motive_definitions = ({body.streaming_id: z_up_body(body) for body in reply.bodies},
                                    {camera.name: z_up_camera(camera) for camera in reply.cameras},
                                    tuple(reply.problems))
        self._use_definitions()

    def _use_definitions(self) -> None:
        """Log what changed in the body definitions and cameras, and publish them when they changed.

        Motive's, or with `sim` a fake Motive's; the frame is checked with the cameras at the next look
        (`_check_anchors`).
        """
        log, now = self.get_logger(), time.time()
        bodies, cameras, problems = ((self._fake_definitions, self._fake_cameras, ()) if self.simulated else
                                     self._motive_definitions or ({}, {}, ()))
        if bodies != self._definitions:
            if self._definitions is None:
                log.info(wrap(f"Motive defines {len(bodies)} rigid bodies: {sorted(bodies) or 'none'}"))
            else:
                for event in match(self._definitions, bodies):
                    if event.change is not None and event.change.kind == "unchanged":
                        continue
                    if event.change is not None and event.change.data_change:
                        log.warn(wrap(f"rigid body {event.describe()}: data measured with it is out of date"))
                    else:
                        log.info(wrap(f"rigid body {event.describe()}"))
            self._definitions = bodies
            self._definitions_publisher.publish(String(data=definitions_json(
                [bodies[key] for key in sorted(bodies)], problems, now)))
            self._check_calibrations()
        if cameras != self._cameras:
            # * The first cameras heard are where they are, not new: a fake Motive may announce them after its bodies.
            if not self._cameras:
                if cameras:
                    log.info(wrap(f"Motive has {len(cameras)} cameras"))
                self._cameras_logged = dict(cameras)
            # * Motive's frame from the truss cameras, and each camera against it, both since they were last warned
            # * about, so a slow creep is warned about once it adds up.
            fit, seen_before = frame_and_cameras(self._cameras_logged, cameras, self._frame.truss_serials)
            moved_frame = frame_moved(fit)
            changes = camera_changes(self._cameras_logged, seen_before)
            if moved_frame or changes:
                truss = set(truss_names({**self._cameras_logged, **cameras}, self._frame.truss_serials))
                lines = sorted(f"{camera_label(name)}{' (truss)' if name in truss else ''}: {line}"
                               for name, line in changes.items())
                if moved_frame:
                    metres, turned = fit.moved
                    lines.insert(0, f"the {len(fit.used)} truss cameras moved together {metres * 1e3:.1f} mm, turned "
                                    f"{np.degrees(turned):.2f}°, the lab's volume up to "
                                    f"{volume_moved(fit.raw_cal_from_raw_now) * 1e3:.1f} mm, since last reported"
                                    + ("; each camera below moved against them" if changes else ""))
                why = "continuous calibration re-solved them, Motive recalibrated, or a camera was bumped"
                # * The cameras alone cannot tell whether poses moved; while the anchors hold the lab frame, none did.
                if self._frame.anchors_hold:
                    log.info(box(f"cameras moved in Motive's solution ({why}); the anchors hold the lab frame: poses "
                                 f"unaffected", "\n".join(lines)))
                else:
                    log.warn(box(f"CAMERAS MOVED while the relay runs ({why}); the anchors do not hold the lab frame "
                                 f"(frame {self._frame.status}): poses may have moved with them", "\n".join(lines)))
            # * Once the frame move is warned about, the cameras not warned about are kept as seen from the new frame.
            if moved_frame:
                back = invert(fit.raw_cal_from_raw_now)
                self._cameras_logged = {name: moved_camera(back, camera)
                                        for name, camera in self._cameras_logged.items()}
            latest = cameras if moved_frame else seen_before
            for name in changes:
                if name in latest:
                    self._cameras_logged[name] = latest[name]
                else:
                    del self._cameras_logged[name]
            self._cameras = self._frame.cameras = cameras
            self._publish_cameras()

    # --- --- --- --- --- LAB FRAME --- --- --- --- ---

    def _check_anchors(self) -> None:
        """Publish Motive's unlabeled markers, look for the floor calibration's anchors among them, check the frame."""
        with self._loose_lock:
            sums, self._loose_sums = self._loose_sums, {}
        # * Each marker's mean, if seen in most frames since the last look: one frame's noise tilts the fit too much.
        frames = max((entry[3] for entry in sums.values()), default=0)
        kept = [entry for entry in sums.values() if entry[3] >= frames / 2]
        markers = [(entry[0] / entry[3], entry[1] / entry[3], entry[2] / entry[3]) for entry in kept]
        # * How far each moved within the look: the RMS distance from its mean, from the sum of squares.
        noise = [(float(np.sqrt(max(entry[4] / entry[3] - float(np.dot(mean, mean)), 0.0))), int(entry[3]))
                 for entry, mean in zip(kept, markers)]
        now = time.time()
        self._markers_publisher.publish(String(data=json.dumps(
            {"time": now, "markers": [list(self._in_lab_frame(marker)[0]) for marker in markers]})))
        follower = self._frame
        events = follower.look(markers, now, noise)
        if follower.floor is not None and follower.floor.anchors and markers:
            self._anchors_publisher.publish(String(data=anchors_json(
                [MocapAnchor(name, self._in_lab_frame(anchor.position)[0]) for name, anchor in
                 sorted(follower.anchors.items())], follower.anchors_unseen, now)))
        self._log_frame_events(events)
        line = json.dumps(follower.metrics(now))
        self._metrics_publisher.publish(String(data=line))
        self._write_frame_log(line, now)
        self._announce_frame_if_news()

    def _write_frame_log(self, line: str, now: float) -> None:
        """Append a metrics sample to the frame log, if on; an error is logged (throttled), never raised."""
        if self._frame_log is None:
            return
        try:
            self._frame_log.write(line, now)
        except OSError as error:
            self.get_logger().error(wrap(f"frame log {self._frame_log.folder} not written: {error}; the relay goes on"),
                                    throttle_duration_sec=600.0)

    def _log_frame_events(self, events: List[FrameEvent]) -> None:
        """Log what the frame follower reports: info with its table indented, a warning or an error boxed.

        ! One call per level: rclpy refuses one call site logging at two levels.
        """
        log = self.get_logger()
        for event in events:
            if event.level == "info":
                log.info("\n".join([wrap(event.title), *(f"  {line}" for line in event.lines)]))
            elif event.level == "warn":
                log.warn(box(event.title, "\n".join(event.lines)) if event.lines else wrap(event.title))
            else:
                log.error(box(event.title, "\n".join(event.lines)) if event.lines else wrap(event.title))

    def _in_lab_frame(self, pos: Vec3, quat: QuatXyzw = (0.0, 0.0, 0.0, 1.0)) -> Tuple[Vec3, QuatXyzw]:
        """A raw pose (Motive's, Z-up) in the lab frame by `lab_from_raw`; unchanged without a floor calibration.

        Hand-written vector maths: it runs for every body and marker of every frame, on NatNet's thread.
        """
        lab_from_raw = self._lab_from_raw
        if lab_from_raw is None:
            return pos, quat
        rotation, shift, turn = lab_from_raw
        return tuple(float(v) for v in rotation @ np.asarray(pos, dtype=float) + shift), quat_multiply_xyzw(turn, quat)

    def _publish_cameras(self) -> None:
        """Publish Motive's cameras on `~/cameras`, in the lab frame like every pose."""
        pose, cameras = self._frame.lab_from_raw(), self._cameras or {}
        if pose is not None:
            cameras = {name: moved_camera(pose, camera) for name, camera in cameras.items()}
        self._cameras_publisher.publish(String(data=cameras_json([cameras[name] for name in sorted(cameras)],
                                                                 time.time())))

    def _use_lab_from_raw(self) -> None:
        """Put every pose in the lab frame by the follower's `lab_from_raw` from now on; raw without one."""
        pose = self._frame.lab_from_raw()
        self._lab_from_raw = None if pose is None else (Rotation.from_quat(pose.orientation).as_matrix(),
                                                        np.asarray(pose.position, dtype=float),
                                                        tuple(float(v) for v in pose.orientation))
        if self._cameras is not None:
            self._publish_cameras()

    def _load_floor_calibration(self, why: str) -> None:
        """Read the newest floor calibration with cameras and apply it, unchecked and without a refit; log, announce.

        ! A Drive that does not read keeps the frame in use (every robot's pose would jump): only an error.
        """
        log, follower = self.get_logger(), self._frame
        folder = self.drive_root / self._sessions if self.drive_root is not None else None
        kept = follower.floor
        try:
            if folder is not None and not self.drive_root.is_dir():
                raise ValueError("the Drive root is not a folder (not synced?)")
            # * A missing folder is no floor calibration yet, unless a frame is in use: then the Drive is not synced.
            if folder is not None and not folder.is_dir() and kept is not None:
                raise ValueError(f"<Drive>/{self._sessions} is missing (not synced, or moved?)")
            frame, skipped = newest_floor_calibration(folder) if folder is not None else (None, [])
            # * A newer file that does not read (half-synced): the frame in use is kept; without one, the newest that
            # * reads is applied.
            if skipped and (frame is None or kept is not None and frame.floor_file.name <= kept.floor_file.name):
                raise ValueError("; ".join(skipped))
        except ValueError as error:
            follower.error = f"floor calibration at {why} not read: {error}"
            log.error(wrap(f"{follower.error}; " + (f"keeping {kept.floor_file.name}'s frame" if kept else
                                                    "poses stay raw (Motive's, Z-up), which is not the lab frame")))
            self._announce_frame_if_news()
            return
        for problem in skipped:
            log.warn(wrap(f"floor calibration at {why}: passed over {problem}"))
        follower.use_floor_calibration(frame)
        self._use_lab_from_raw()
        if frame is None:
            where = f"<Drive>/{self._sessions}" if folder is not None else "no Drive"
            log.info(wrap(f"floor calibration at {why}: none with cameras in {where}; poses stay raw "
                          f"(Motive's, Z-up), not in the lab frame (Export floor calibration in the mocap probe)"))
        else:
            log.info(wrap(f"floor calibration at {why}: {frame.floor_file.name}\n  it puts raw poses up to "
                          f"{volume_moved(frame.floor_from_raw) * 1e3:.1f} mm away, in the lab frame\n  Motive's frame "
                          f"followed by {len(frame.anchors)} anchors and "
                          f"{len(truss_names(frame.cameras, self._frame.truss_serials))} truss cameras; unchecked "
                          f"until the first look"))
        self._announce_frame_if_news()

    def _handle_use_refit(self, request, response):
        """Use the pending refit for every pose from now on, in memory only (service ~/use_refit).

        It is fitted against the floor calibration's anchors and cameras, as every later one is: a restart or another
        floor calibration starts from the file's floor_from_raw again.
        """
        del request
        response.success, response.message, events = self._frame.use_refit()
        self._log_frame_events(events)
        if response.success:
            self._use_lab_from_raw()
            self._announce_frame_if_news()
        return response

    def _announce_frame_if_news(self) -> None:
        """Publish the frame's state on `~/frame` when it says something new (`FrameFollower.news`)."""
        if self._frame.news():
            self._frame_publisher.publish(String(data=json.dumps({
                **self._frame.announcement(time.time()), "folder": self._sessions.as_posix()})))

    # --- --- --- --- --- PUBLISHING --- --- --- --- ---

    def _current_samples(self, now_mono: float) -> Dict[int, Tuple[str, BodySample]]:
        """Per rigid body, the sample to publish and its source.

        Motive's; with `sim` the first source by priority that still streams, else the one heard last.
        """
        cutoff = now_mono - self.streaming_stale_timeout_sec
        with self._samples_lock:
            by_priority = ([(FAKE_MOTIVE, dict(self._sim_samples[FAKE_MOTIVE])),
                            (TRUTH, dict(self._sim_samples[TRUTH]))] if self.simulated else
                           [(MOTIVE, dict(self._samples))])
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
        sources_changed = False
        for rb_id, (source, sample) in self._current_samples(now).items():
            if self._sources.get(rb_id) != source:
                if rb_id in self._sources:
                    self.get_logger().info(wrap(f"rigid body {rb_id}: now from the {source}, was the "
                                                f"{self._sources[rb_id]}"))
                self._sources = {**self._sources, rb_id: source}
                sources_changed = True
            pose_valid = (sample.received_mono_sec >= cutoff and sample.tracking_valid
                          and sample.marker_error <= self.marker_error_valid_threshold)
            publisher = self._rb_publishers.get(rb_id)
            if publisher is None:
                # ROS tokens cannot start with a number, so prefix rigid-body IDs.
                topic = f"{self.topic_prefix}/id_{rb_id}/pose"
                publisher = self.create_publisher(MocapRigidBodyPose, topic, 10)
                self._rb_publishers[rb_id] = publisher
                calibration = self._calibrations.get(rb_id) if source != TRUTH else None
                what = (f"{calibration.robot_serial}'s base_footprint" if calibration is not None else
                        "a robot's base_footprint" if source == TRUTH else "no robot's, uncalibrated")
                self.get_logger().info(wrap(f"rigid body {rb_id}: {what}, from the {source}, on {topic}"))
            msg = MocapRigidBodyPose()
            # Capture time, so a pose republished while no new frame arrived keeps its age.
            msg.header.stamp = Time(sec=sample.capture_time_ns // 1_000_000_000,
                                    nanosec=sample.capture_time_ns % 1_000_000_000)
            # * A simulator's truth is the lab frame itself.
            msg.header.frame_id = LAB_FRAME if self._lab_from_raw is not None or source == TRUTH else RAW_FRAME
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
        if sources_changed:
            # * Announced with the calibrations: a fake Motive's body is calibrated, a simulator's truth not.
            self._announce_calibrations()

    def _handle_get_streaming_ids(self, request, response):
        del request
        ids = self._get_streaming_ids(time.monotonic())
        response.ids = ids
        response.count = len(ids)
        response.query_time = self.get_clock().now().to_msg()
        return response

    def destroy_node(self) -> None:
        """Shut the NatNet client down, close the frame log, then the node."""
        if hasattr(self, "mocap_client") and self.mocap_client is not None:
            with self._mocap_client_lock:
                try:
                    self.mocap_client.shutdown()
                except BaseException as exc:  # pylint: disable=broad-except
                    # During Ctrl-C shutdown, thread joins can be interrupted; keep teardown best-effort.
                    if rclpy.ok():
                        self.get_logger().warn(wrap(f"Exception during NatNet shutdown: {exc}"))
        if getattr(self, "_frame_log", None) is not None:
            self._frame_log.close()
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
    # * Two threads: one for publishing poses, one for every other callback.
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        rclpy.spin(node, executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C, or SIGTERM through rclpy's signal handler
    finally:
        try:
            executor.shutdown()
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass  # a second Ctrl-C during teardown: launch forwards one on top of the terminal's


if __name__ == "__main__":
    main()
