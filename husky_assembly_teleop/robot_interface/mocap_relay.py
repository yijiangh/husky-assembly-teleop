"""
The mocap relay of the monitor's ROS domain, as one device: what it announces for every rigid body, and requests to it.

Each rigid body's poses stay with its body (`mocap.py`); this holds what belongs to the relay as a whole: the base
calibrations it applies, and asking it to reload them.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Callable

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .connections import RosConnections

#: The relay's latched announcement of the base calibrations it applies (JSON in a String), and its reload service.
CALIBRATIONS_TOPIC = "/mocap_relay/calibrations"
RELOAD_SERVICE = "/mocap_relay/reload_calibration"
#: The announcement's key for the calibration itself, as [[x, y, z], [qx, qy, qz, qw]].
BASE_KEY = "base_mocap_from_base_footprint"


@dataclass(frozen=True)
class AppliedCalibration:
    """The base calibration the relay applies to a robot's rigid body, as it last announced it.

    Attributes:
        position: base_footprint in the rigid body's frame, metres.
        orientation: Its quaternion (x, y, z, w).
        sha256: Hash of the calibration's content, to tell versions apart.
        source: Where it came from: crl-husky's config file, or the session file below the Drive root.
        session: The Drive session's YYYYMMDD, or "" for crl-husky's default.
    """

    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    sha256: str
    source: str
    session: str


@dataclass
class MocapRelayState:
    """What the relay last announced.

    Attributes:
        calibrations: The base calibration it applies, by rigid body; a body it publishes uncalibrated is missing.
        calibrations_time: ROS time the last announcement arrived, seconds, or None before one.
    """

    calibrations: dict[int, AppliedCalibration] = field(default_factory=dict)
    calibrations_time: float | None = None


def applied_calibrations(text: str) -> dict[int, AppliedCalibration]:
    """Read the relay's announcement: the calibration it applies, by rigid body.

    Raises:
        ValueError: If the text is not the relay's format.
    """
    try:
        entries = json.loads(text)["calibrations"]
        return {int(entry["mocap_id"]): AppliedCalibration(
            tuple(float(v) for v in entry[BASE_KEY][0]), tuple(float(v) for v in entry[BASE_KEY][1]),
            str(entry["sha256"]), str(entry["source"]), str(entry.get("session", ""))) for entry in entries}
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
        self._reload_client = None
        self.state = MocapRelayState()
        self._connect()

    def _connect(self) -> None:
        """Subscribe to the latched calibrations: the last announcement arrives at once, then each reload's."""
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._ros.subscription(String, CALIBRATIONS_TOPIC, self._on_calibrations, latched)

    def _on_calibrations(self, message: String) -> None:
        """Keep the relay's latest announcement."""
        self.state.calibrations = applied_calibrations(message.data)
        self.state.calibrations_time = self._now()

    def reload_calibrations(self):
        """Ask the relay to apply the newest base calibrations now; its next announcement says what it applies.

        ! It changes the poses every robot drives from, the onboard followers' too: only while none drives.

        Returns:
            The rclpy future of the Trigger call (await with `ctx.ros`), or None if the relay does not answer.
        """
        if self._reload_client is None:
            self._reload_client = self._ros.client(Trigger, RELOAD_SERVICE)
        if not self._reload_client.service_is_ready():
            return None
        return self._reload_client.call_async(Trigger.Request())
