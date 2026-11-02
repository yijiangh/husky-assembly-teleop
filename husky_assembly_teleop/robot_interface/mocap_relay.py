"""
The mocap relay of the monitor's ROS domain, as one device: what it announces for every rigid body, and requests to it.

Each rigid body's poses stay with its body (`mocap.py`); this holds what belongs to the relay as a whole: the base
calibrations it applies, Motive's body definitions and cameras, the floor calibration it applies, and asking it to
reload the base or floor calibrations or use a refit of the floor calibration.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Callable

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from scaffolding_assembly_core.geometry import Pose, compose, invert

from .connections import RosConnections
# * The FRAME_* states are imported from here by the monitor's side.
from .mocap_definitions import (FRAME_ERROR, FRAME_HELD, FRAME_NONE, FRAME_REFIT_PENDING,  # noqa: F401
                                FRAME_UNCHECKED, FRAME_UNTRUSTED, UNKNOWN, BodyDefinition, MocapAnchor, MocapCamera,
                                read_anchors, read_cameras, read_definitions)

#: The relay's latched announcement of the base calibrations it applies (JSON in a String), and its service reloading
#: one robot's rigid body (by id).
CALIBRATIONS_TOPIC = "/mocap_relay/calibrations"
RELOAD_BASE_SERVICE = "/mocap_relay/reload_base_calibration/id_{}"
#: The relay's latched announcements of Motive's rigid body definitions and cameras (JSON in a String).
DEFINITIONS_TOPIC, CAMERAS_TOPIC = "/mocap_relay/body_definitions", "/mocap_relay/cameras"
#: The anchors it finds among Motive's unlabeled markers, and those markers (JSON in a String, latched).
ANCHORS_TOPIC, MARKERS_TOPIC = "/mocap_relay/anchors", "/mocap_relay/unlabeled_markers"
#: The announcement's key for the calibration itself, as [[x, y, z], [qx, qy, qz, qw]].
BASE_KEY = "base_mocap_from_base_footprint"
#: Where the relay says a rigid body streams from (`MocapRelayState.source`): the real Motive, or a simulation's fake
#: Motive or simulator's truth (only a relay with sim:=true reads those).
MOTIVE, FAKE_MOTIVE, TRUTH = "Motive", "fake Motive", "simulator's truth"
#: Its floor calibration announcement (status: the FRAME_* states), the service that uses the pending refit
#: (`world/mocap_frame.py`), and the one that applies the newest floor calibration on the Drive.
FRAME_TOPIC, USE_REFIT_SERVICE = "/mocap_relay/frame", "/mocap_relay/use_refit"
RELOAD_FLOOR_SERVICE = "/mocap_relay/reload_floor_calibration"
#: How Motive's frame moved, every look (JSON in a String, not latched; `FrameFollower.metrics`).
METRICS_TOPIC = "/mocap_relay/frame_metrics"


@dataclass(frozen=True)
class FloorCalibrationState:
    """The floor calibration as the relay last announced it.

    Attributes:
        status: One of the FRAME_* states.
        floor_calibration: The floor calibration file it applies, or "".
        refit: The refit in use, for people ("" if none).
        pending: The deciding fit while the frame is not held (a refit pending, or untrusted), for people ("" if
            none): how far poses are off now.
        lab_from_raw: What the relay applies to every raw (Z-up) pose: the floor calibration's floor_from_raw and
            the refit in use, ((x, y, z), (qx, qy, qz, qw)), or None for nothing.
        error: Why the relay could not read the floor calibration at its last start or reload, or "".
        anchor_fit: How Motive's frame moved by the anchors since the floor calibration, for people ("" if none).
        camera_fit: The same by the truss cameras.
        newer: A newer floor calibration on the Drive, not applied until asked (`reload_floor_calibration`), or "".
        newer_moves: How far applying it would move points of the lab's volume, metres, or None.
        newer_moves_poses: Whether that is far enough to matter (over 1 mm, or no floor calibration applied yet).
        folder: Its floor calibrations' sessions folder below the Drive root, where new ones go ("" from older relays).
        decides: Which fit decides: "anchors", "truss cameras", or "" before one did.
        time: When the relay announced it, on the relay's clock, seconds; None from older relays.
    """

    status: str
    floor_calibration: str = ""
    refit: str = ""
    pending: str = ""
    lab_from_raw: tuple | None = None
    error: str = ""
    anchor_fit: str = ""
    camera_fit: str = ""
    newer: str = ""
    newer_moves: float | None = None
    newer_moves_poses: bool = False
    folder: str = ""
    decides: str = ""
    time: float | None = None

    @property
    def poses_off(self) -> bool:
        """Whether poses may be off the lab frame: unchecked, untrusted, a refit pending, none read, or a newer one."""
        return self.status in (FRAME_UNCHECKED, FRAME_REFIT_PENDING, FRAME_UNTRUSTED, FRAME_ERROR) or \
            self.newer_moves_poses


def floor_calibration_state(text: str) -> FloorCalibrationState:
    """Read the relay's floor calibration announcement.

    Raises:
        ValueError: If the text is not the relay's format.
    """
    try:
        entry = json.loads(text)
        lab_from_raw, newer = entry.get("lab_from_raw"), entry.get("newer") or {}
        return FloorCalibrationState(str(entry["status"]), entry.get("floor_calibration") or "",
                                     (entry.get("refit") or {}).get("describe", ""),
                                     (entry.get("pending") or {}).get("describe", ""),
                                     (tuple(map(float, lab_from_raw[0])), tuple(map(float, lab_from_raw[1])))
                                     if lab_from_raw else None, str(entry.get("error") or ""),
                                     (entry.get("anchor_fit") or {}).get("describe", ""),
                                     (entry.get("camera_fit") or {}).get("describe", ""),
                                     str(newer.get("floor_calibration") or ""),
                                     float(newer["moves_volume"]) if "moves_volume" in newer else None,
                                     bool(newer.get("moves_poses", bool(newer))), str(entry.get("folder") or ""),
                                     str(entry.get("decides") or ""),
                                     float(entry["time"]) if entry.get("time") is not None else None)
    except (KeyError, TypeError, AttributeError, json.JSONDecodeError) as error:
        raise ValueError(f"not the relay's floor calibration announcement: {error!r}") from None


@dataclass(frozen=True)
class AppliedCalibration:
    """The base calibration the relay applies to a robot's rigid body, as it last announced it.

    Attributes:
        position: base_footprint in the rigid body's frame, metres.
        orientation: Its quaternion (x, y, z, w).
        sha256: Hash of the calibration's content, to tell versions apart.
        source: Where it came from: crl-husky's config file, or the session file below the Drive root.
        session: The Drive session's YYYYMMDD, or "" for crl-husky's default.
        definition_status: Whether the body definition it was built for matches Motive's now: "matches",
            "out_of_date" or "unknown" (`mocap_definitions.definition_check`).
        definition_detail: Why, for people.
    """

    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    sha256: str
    source: str
    session: str
    definition_status: str = UNKNOWN
    definition_detail: str = ""


@dataclass
class MocapRelayState:
    """What the relay last announced.

    Attributes:
        calibrations: The base calibration it applies, by rigid body (a fake Motive's in a simulation); a body it
            publishes uncalibrated is missing. A simulator's true pose gets none.
        simulated: Whether the relay reads a simulation's inputs only (`sim:=true`), else Motive's only: whatever is
            measured through it is all simulated or all real.
        calibrations_time: ROS time the last announcement arrived, seconds, or None before one.
        body_definitions: Motive's rigid body definitions, by streaming id (a fake Motive's in a simulation).
        definitions_time: ROS time they last arrived, seconds, or None before.
        cameras: Motive's cameras, by name (a fake Motive's in a simulation).
        cameras_time: ROS time they last arrived, seconds, or None before.
        anchors: The anchors of Motive's continuous calibration the relay sees, by the lab's name, in the frame it
            publishes poses in.
        anchors_missing: The floor calibration's anchors it does not see.
        unlabeled_markers: Motive's unlabeled markers, each averaged over the relay's last look, in the frame it
            publishes poses in.
        markers_time: ROS time they last arrived, seconds, or None before.
        markers_stamp: When the relay took them, on its clock (as `FloorCalibrationState.time`), seconds, or None.
        frame: The floor calibration the relay applies, or None before it announced one.
        fit_now: The deciding fit's raw_cal_from_raw_now from the relay's last look, ((x, y, z), (qx, qy, qz, qw)),
            or None while it is not trusted (`lab_from_raw_now`).
        refit_now: The refit in use in that look, the same way, or None.
        marker_noise: That look's unlabeled markers, raw (Motive's, Z-up), each with its noise: the RMS distance of its
            frames from its mean, mm, or None.
        metrics_time: ROS time that look last arrived, seconds, or None before.
        sources: Where each rigid body streams from now, by id: MOTIVE, FAKE_MOTIVE or TRUTH (`source`).
    """

    calibrations: dict[int, AppliedCalibration] = field(default_factory=dict)
    simulated: bool = False
    calibrations_time: float | None = None
    body_definitions: dict[int, BodyDefinition] = field(default_factory=dict)
    definitions_time: float | None = None
    cameras: dict[str, MocapCamera] = field(default_factory=dict)
    cameras_time: float | None = None
    anchors: dict[str, MocapAnchor] = field(default_factory=dict)
    anchors_missing: tuple[str, ...] = ()
    unlabeled_markers: tuple[tuple[float, float, float], ...] = ()
    markers_time: float | None = None
    markers_stamp: float | None = None
    frame: FloorCalibrationState | None = None
    fit_now: tuple | None = None
    refit_now: tuple | None = None
    marker_noise: tuple[tuple[tuple[float, float, float], float | None], ...] = ()
    metrics_time: float | None = None
    sources: dict[int, str] = field(default_factory=dict)

    def source(self, mocap_id: int) -> str:
        """Where a rigid body streams from now: MOTIVE, FAKE_MOTIVE or TRUTH; one not heard of counts as the mode's.

        Only Motive's and a fake Motive's bodies get a base calibration and the floor calibration.
        """
        return self.sources.get(mocap_id, FAKE_MOTIVE if self.simulated else MOTIVE)

    @property
    def lab_from_raw_now(self) -> Pose | None:
        """Where the relay's trusted fit puts the lab frame now: the frame it applies, its refit undone, the fit done.

        None without a floor calibration applied or a trusted fit.
        """
        if self.frame is None or self.frame.lab_from_raw is None or self.fit_now is None:
            return None
        floor_from_raw = compose(Pose(*self.frame.lab_from_raw), invert(Pose(*self.refit_now)) if self.refit_now
                                 else Pose())
        return compose(floor_from_raw, Pose(*self.fit_now))


def applied_calibrations(text: str) -> dict[int, AppliedCalibration]:
    """Read the relay's announcement: the calibration it applies, by rigid body.

    Args:
        text: The announcement.

    Raises:
        ValueError: If the text is not the relay's format.
    """
    try:
        announcement = json.loads(text)
        entries = announcement["calibrations"]
        return {int(entry["mocap_id"]): AppliedCalibration(
            tuple(float(v) for v in entry[BASE_KEY][0]), tuple(float(v) for v in entry[BASE_KEY][1]),
            str(entry["sha256"]), str(entry["source"]), str(entry.get("session", "")),
            str(entry.get("definition_check", {}).get("status", UNKNOWN)),
            str(entry.get("definition_check", {}).get("detail", "an older relay: not checked")))
            for entry in entries}
    except (KeyError, TypeError, IndexError, json.JSONDecodeError) as error:
        raise ValueError(f"not the relay's calibration announcement: {error!r}") from None


class MocapRelayInterface:
    """ROS2 interface to the mocap relay: its announcements in `state`, and requests to it."""

    def __init__(self, node: Node, now: Callable[[], float]):
        """Subscribe to the relay's announcements.

        Args:
            node: The monitor's node, in the relay's domain.
            now: Current ROS time in seconds.
        """
        self._ros = RosConnections(node)
        self._now = now
        self._use_refit_client = self._reload_floor_client = None
        self._reload_base_clients = {}
        self.state = MocapRelayState()
        self._connect()

    def _connect(self) -> None:
        """Subscribe to the latched announcements: the last of each arrives at once, then each change."""
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._ros.subscription(String, CALIBRATIONS_TOPIC, self._on_calibrations, latched)
        self._ros.subscription(String, DEFINITIONS_TOPIC, self._on_definitions, latched)
        self._ros.subscription(String, CAMERAS_TOPIC, self._on_cameras, latched)
        self._ros.subscription(String, FRAME_TOPIC, self._on_frame, latched)
        self._ros.subscription(String, ANCHORS_TOPIC, self._on_anchors, latched)
        self._ros.subscription(String, MARKERS_TOPIC, self._on_markers, latched)
        self._ros.subscription(String, METRICS_TOPIC, self._on_metrics)

    def _on_calibrations(self, message: String) -> None:
        """Keep the relay's latest announcement."""
        self.state.calibrations = applied_calibrations(message.data)
        announcement = json.loads(message.data)
        self.state.simulated = bool(announcement.get("simulated"))
        self.state.sources = {int(rb_id): str(source) for rb_id, source in (announcement.get("sources") or {}).items()}
        self.state.calibrations_time = self._now()

    def _on_definitions(self, message: String) -> None:
        """Keep Motive's latest body definitions."""
        self.state.body_definitions = read_definitions(message.data)
        self.state.definitions_time = self._now()

    def _on_cameras(self, message: String) -> None:
        """Keep Motive's latest cameras."""
        self.state.cameras = read_cameras(message.data)
        self.state.cameras_time = self._now()

    def _on_anchors(self, message: String) -> None:
        """Keep the anchors the relay last saw."""
        self.state.anchors, self.state.anchors_missing = read_anchors(message.data)

    def _on_markers(self, message: String) -> None:
        """Keep the unlabeled markers the relay last saw; an announcement that does not read is skipped."""
        try:
            announcement = json.loads(message.data)
            markers = tuple(tuple(float(v) for v in marker) for marker in announcement["markers"])
            stamp = float(announcement["time"]) if announcement.get("time") is not None else None
        except (KeyError, TypeError, ValueError):
            return
        self.state.unlabeled_markers, self.state.markers_stamp = markers, stamp
        self.state.markers_time = self._now()

    def _on_metrics(self, message: String) -> None:
        """Keep the deciding fit, if trusted, the refit in use and the markers' noise from the relay's last look.

        A look that does not read is skipped.
        """
        def pose(fit: dict | None) -> tuple | None:
            """A fit's raw_cal_from_raw_now as ((x, y, z), (qx, qy, qz, qw)), or None for no fit."""
            if not fit:
                return None
            position, orientation = fit["raw_cal_from_raw_now"]
            return tuple(map(float, position)), tuple(map(float, orientation))

        try:
            sample = json.loads(message.data)
            decides = sample.get("decides")
            fit = sample.get(decides.replace(" ", "_")) if decides else None
            self.state.fit_now = pose(fit) if fit and fit.get("trusted") else None
            self.state.refit_now = pose(sample.get("refit"))
            markers = sample.get("markers") or {}
            self.state.marker_noise = tuple(
                (tuple(map(float, raw)), None if noise is None else float(noise))
                for raw, noise in zip(markers.get("raw") or [], markers.get("noise_mm") or []))
        except (KeyError, TypeError, ValueError, AttributeError):
            return
        self.state.metrics_time = self._now()

    def _on_frame(self, message: String) -> None:
        """Keep the relay's latest floor calibration announcement."""
        self.state.frame = floor_calibration_state(message.data)

    def use_refit(self):
        """Ask the relay to use the pending refit of the floor calibration: every pose it publishes moves by it.

        ! Like a reload, it changes the poses every robot drives from: only while none drives.

        Returns:
            The rclpy future of the Trigger call (await with `ctx.ros`), or None if the relay does not answer.
        """
        if self._use_refit_client is None:
            self._use_refit_client = self._ros.client(Trigger, USE_REFIT_SERVICE)
        if not self._use_refit_client.service_is_ready():
            return None
        return self._use_refit_client.call_async(Trigger.Request())

    def reload_floor_calibration(self):
        """Ask the relay to apply the newest floor calibration on the Drive: every pose it publishes moves by it.

        ! Like a refit, it changes the poses every robot drives from: only while none drives.

        Returns:
            The rclpy future of the Trigger call (await with `ctx.ros`), or None if the relay does not answer.
        """
        if self._reload_floor_client is None:
            self._reload_floor_client = self._ros.client(Trigger, RELOAD_FLOOR_SERVICE)
        if not self._reload_floor_client.service_is_ready():
            return None
        return self._reload_floor_client.call_async(Trigger.Request())

    def reload_base_calibration(self, mocap_id: int):
        """Ask the relay to apply the newest base calibration of one robot's rigid body; the others keep theirs.

        ! It changes the pose that robot drives from, its onboard follower's too: only while it stands still.

        Returns:
            The rclpy future of the Trigger call (await with `ctx.ros`), or None if the relay does not answer.
        """
        if mocap_id not in self._reload_base_clients:
            self._reload_base_clients[mocap_id] = self._ros.client(Trigger, RELOAD_BASE_SERVICE.format(mocap_id))
        client = self._reload_base_clients[mocap_id]
        return client.call_async(Trigger.Request()) if client.service_is_ready() else None
