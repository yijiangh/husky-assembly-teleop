"""The mocap relay as the monitor sees it: its announcement of the base calibrations it applies, and reloading them."""

import json
from types import SimpleNamespace

import pytest
from std_msgs.msg import String

from body_definition_fixtures import body
from robot_fakes import Node
from husky_assembly_teleop.robot_interface.mocap_definitions import MocapCamera, cameras_json, definitions_json
from husky_assembly_teleop.robot_interface.mocap_relay import (USE_REFIT_SERVICE, CALIBRATIONS_TOPIC, CAMERAS_TOPIC,
                                                               DEFINITIONS_TOPIC, FRAME_TOPIC, MARKERS_TOPIC,
                                                               METRICS_TOPIC, RELOAD_BASE_SERVICE, MocapRelayInterface,
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


def test_reload_asks_the_relays_service_of_that_rigid_body_only():
    """A reload is one Trigger request to the relay's service of that robot's rigid body."""
    node = RelayNode()
    assert MocapRelayInterface(node, lambda: node.now).reload_base_calibration(1850) == "future"
    assert len(node.clients[RELOAD_BASE_SERVICE.format(1850)].requests) == 1
    assert RELOAD_BASE_SERVICE.format(1840) not in node.clients


def test_another_format_is_refused():
    """Anything but the relay's format raises, so the guarded callback drops it with a log line."""
    with pytest.raises(ValueError, match="not the relay's calibration announcement"):
        applied_calibrations(json.dumps({"calibrations": [{"mocap_id": 1}]}))


def test_a_simulations_relay_says_so_with_its_calibrations():
    """sim:=true announces simulated, with the simulated sessions' calibrations; an older relay counts as Motive's."""
    node = RelayNode()
    relay = MocapRelayInterface(node, lambda: node.now)
    sim = Calibration(1850, "a200-0805", (0.05, 0.01, -0.48), (0.0, 0.0, 0.0, 1.0),
                      "data_experiment/sim/calibration_data/domain_70/20261022/"
                      "calibrated_transformation_0805_rhino.json",
                      "99aa", ("base_mocap_from_ur_arm_base_link_inertia",), "20261022")
    announcement = {**json.loads(ANNOUNCEMENT), "calibrations": [sim.as_dict()], "simulated": True}
    node.callbacks[CALIBRATIONS_TOPIC](String(data=json.dumps(announcement)))
    assert relay.state.simulated and relay.state.calibrations[1850].sha256 == "99aa"
    node.callbacks[CALIBRATIONS_TOPIC](String(data=ANNOUNCEMENT))
    assert not relay.state.simulated and relay.state.calibrations[1850].sha256 == BELLE.sha256


def test_the_interface_keeps_motives_definitions_and_cameras():
    """Each announcement replaces the last, with when it came; before one, nothing is known."""
    node = RelayNode()
    relay = MocapRelayInterface(node, lambda: node.now)
    assert relay.state.definitions_time is None and relay.state.cameras_time is None
    camera = MocapCamera("PrimeX 22 #72318", (-3.286, -1.852, 2.188), (-0.186, 0.148, -0.646, 0.725))
    node.callbacks[DEFINITIONS_TOPIC](String(data=definitions_json([body()], [], 1.0)))
    node.callbacks[CAMERAS_TOPIC](String(data=cameras_json([camera], 1.0)))
    assert relay.state.body_definitions == {1866: body()} and relay.state.cameras == {camera.name: camera}
    assert relay.state.definitions_time == relay.state.cameras_time == node.now


def test_the_definition_check_is_read_and_unknown_from_an_older_relay():
    """The relay's check of each calibration's body definition; an older relay announces none."""
    checked = json.dumps({"calibrations": [{**BELLE.as_dict(), "definition_check": {
        "status": "out_of_date", "detail": "made for an earlier body: refined"}}]})
    assert applied_calibrations(checked)[1850].definition_status == "out_of_date"
    assert applied_calibrations(ANNOUNCEMENT)[1850].definition_status == "unknown"


def test_the_floor_calibration_is_kept_and_a_refit_adopted_through_the_relays_service():
    """A pending refit puts poses off; Use refit calls the relay's service."""
    node = RelayNode()
    relay = MocapRelayInterface(node, lambda: node.now)
    assert relay.state.frame is None
    node.callbacks[FRAME_TOPIC](String(data=json.dumps({"status": "refit_pending", "floor_calibration": "f.json",
                                                        "pending": {"describe": "frame moved 20.0 mm"}})))
    assert relay.state.frame.poses_off and relay.state.frame.pending == "frame moved 20.0 mm"
    assert relay.use_refit() == "future" and node.clients[USE_REFIT_SERVICE].requests
    node.callbacks[FRAME_TOPIC](String(data=json.dumps({"status": "held", "floor_calibration": "f.json", "time": 7.0,
                                                        "anchor_fit": {"describe": "frame moved 0.2 mm"},
                                                        "camera_fit": {"describe": "frame moved 0.4 mm"}})))
    frame = relay.state.frame
    assert not frame.poses_off and (frame.anchor_fit, frame.camera_fit) == ("frame moved 0.2 mm", "frame moved 0.4 mm")
    assert frame.time == 7.0


def test_the_unlabeled_markers_are_kept_and_an_unreadable_announcement_skipped():
    """The relay's markers with when they came; one that does not read leaves them as they were."""
    node = RelayNode()
    relay = MocapRelayInterface(node, lambda: node.now)
    node.callbacks[MARKERS_TOPIC](String(data=json.dumps({"time": 1.0, "markers": [[1.0, 2.0, 0.03]]})))
    assert relay.state.unlabeled_markers == ((1.0, 2.0, 0.03),) and relay.state.markers_time == node.now
    assert relay.state.markers_stamp == 1.0
    node.callbacks[MARKERS_TOPIC](String(data="not json"))
    assert relay.state.unlabeled_markers == ((1.0, 2.0, 0.03),)


def test_the_markers_noise_from_the_relays_last_look_is_kept():
    """Each marker of the look raw with its noise, mm; a marker without frames has none."""
    node = RelayNode()
    relay = MocapRelayInterface(node, lambda: node.now)
    sample = {"decides": None, "markers": {"raw": [[1.0, 2.0, 0.03], [0.0, 0.0, 1.0]], "noise_mm": [0.04, None]}}
    node.callbacks[METRICS_TOPIC](String(data=json.dumps(sample)))
    assert relay.state.marker_noise == (((1.0, 2.0, 0.03), 0.04), ((0.0, 0.0, 1.0), None))


def test_where_the_relays_last_look_puts_the_lab_frame_is_kept():
    """The frame applied, its refit undone, the trusted deciding fit done; untrusted: none; unreadable: skipped."""
    node = RelayNode()
    relay = MocapRelayInterface(node, lambda: node.now)
    node.callbacks[FRAME_TOPIC](String(data=json.dumps({"status": "held", "lab_from_raw": [[1, 0, 0], [0, 0, 0, 1]]})))
    fit = {"raw_cal_from_raw_now": [[0.003, 0, 0], [0, 0, 0, 1]], "trusted": True}
    sample = {"decides": "truss cameras", "truss_cameras": fit, "anchors": None,
              "refit": {"raw_cal_from_raw_now": [[0.001, 0, 0], [0, 0, 0, 1]], "trusted": True}}
    node.callbacks[METRICS_TOPIC](String(data=json.dumps(sample)))
    assert relay.state.lab_from_raw_now.position == pytest.approx((1.002, 0.0, 0.0))
    assert relay.state.metrics_time == node.now
    node.callbacks[METRICS_TOPIC](String(data="not json"))
    assert relay.state.lab_from_raw_now.position == pytest.approx((1.002, 0.0, 0.0))
    node.callbacks[METRICS_TOPIC](String(data=json.dumps({**sample, "truss_cameras": {**fit, "trusted": False}})))
    assert relay.state.lab_from_raw_now is None
