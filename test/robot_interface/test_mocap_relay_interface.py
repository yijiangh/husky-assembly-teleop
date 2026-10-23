"""The mocap relay as the monitor sees it: its announcement of the base calibrations it applies, and reloading them."""

import json
from types import SimpleNamespace

import pytest
from std_msgs.msg import String

from robot_fakes import Node
from husky_assembly_teleop.robot_interface.mocap_relay import (CALIBRATIONS_TOPIC, RELOAD_SERVICE, MocapRelayInterface,
                                                               applied_calibrations)
from husky_assembly_teleop.world.calibration_sessions import Calibration

pytestmark = pytest.mark.safety

#: What the relay publishes for Belle, built with its own `Calibration.as_dict`; Alice's body goes uncalibrated.
BELLE = Calibration(1850, "a200-0805", (0.056, 0.0108, -0.4787), (0.0045, 0.0024, -0.7099, -0.7043),
                    "data_experiment/calibration_data/20261013/calibrated_transformation_0805_rhino.json",
                    "ab12cd34ef56", ("base_mocap_from_ur_arm_base_link_inertia",), "20261013")
ANNOUNCEMENT = json.dumps({"applied": "start", "time": 1.0, "drive_root": "/drive",
                           "calibrations": [BELLE.as_dict()]})


class RelayNode(Node):
    """A node that keeps its subscriptions' callbacks and its clients, by topic or service name."""

    def __init__(self):
        """No subscriptions yet; the relay's reload service answers."""
        super().__init__()
        self.callbacks, self.clients = {}, {}

    def create_subscription(self, _kind, topic, callback, _qos):
        """Keep the callback."""
        self.callbacks[topic] = callback
        return topic

    def create_client(self, _kind, name):
        """A client whose service is ready and keeps its requests."""
        client = SimpleNamespace(requests=[], service_is_ready=lambda: True)
        client.call_async = lambda request: client.requests.append(request) or "future"
        self.clients[name] = client
        return client


def test_the_relays_announcement_is_read():
    """Each announced rigid body gets its transform, hash, source and session."""
    applied = applied_calibrations(ANNOUNCEMENT)[1850]
    assert applied.position == BELLE.position and applied.orientation == BELLE.orientation
    assert (applied.sha256, applied.source, applied.session) == (BELLE.sha256, BELLE.source, "20261013")


def test_the_interface_keeps_the_latest_announcement():
    """Before one, nothing is known; after it, Belle's calibration and when it came; Alice's body has none."""
    node = RelayNode()
    relay = MocapRelayInterface(node, lambda: node.now)
    assert relay.state.calibrations_time is None
    node.callbacks[CALIBRATIONS_TOPIC](String(data=ANNOUNCEMENT))
    assert relay.state.calibrations[1850].sha256 == BELLE.sha256 and 1840 not in relay.state.calibrations
    assert relay.state.calibrations_time == node.now


def test_reload_asks_the_relays_service():
    """A reload is one Trigger request to the relay."""
    node = RelayNode()
    assert MocapRelayInterface(node, lambda: node.now).reload_calibrations() == "future"
    assert len(node.clients[RELOAD_SERVICE].requests) == 1


def test_another_format_is_refused():
    """Anything but the relay's format raises, so the guarded callback drops it with a log line."""
    with pytest.raises(ValueError, match="not the relay's calibration announcement"):
        applied_calibrations(json.dumps({"calibrations": [{"mocap_id": 1}]}))
