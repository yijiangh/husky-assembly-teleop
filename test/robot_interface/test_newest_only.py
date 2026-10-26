"""Newest-only subscriptions: every message is taken and its stamp noted, only the newest decoded, unless recorded."""

from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Twist
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState

from husky_assembly_teleop.monitor import HuskyMonitor
from husky_assembly_teleop.robot_interface.connections import RosConnections, header_stamp
from husky_assembly_teleop.robot_interface.event import Event


class _Node:
    """Keeps what `create_subscription` was given, as a subscription the monitor's drain can read."""

    def __init__(self):
        self.subscriptions = []

    def create_subscription(self, msg_type, topic, callback, qos, raw=False):
        subscription = SimpleNamespace(msg_type=msg_type, topic_name=topic, callback=callback, raw=raw, queue=[])
        subscription.handle = _Handle(subscription.queue)
        self.subscriptions.append(subscription)
        return subscription

    def get_logger(self):
        return SimpleNamespace(error=print)


class _Handle:
    """A subscription handle whose queue the test fills with serialized messages."""

    def __init__(self, queue: list):
        self.queue = queue

    def __enter__(self):
        return self

    def __exit__(self, *_):
        return False

    def take_message(self, _msg_type, raw):
        assert raw, "newest-only subscriptions take raw messages"
        return (self.queue.pop(0), {}) if self.queue else None


def _joint_state(sec: int, position: float) -> bytes:
    message = JointState(name=["a"], position=[position])
    message.header.stamp.sec, message.header.stamp.nanosec = sec, 250_000_000
    return serialize_message(message)


def _drain(node: _Node) -> int:
    monitor = SimpleNamespace(subscriptions=node.subscriptions, _undecodable=set(), log_error=print)
    return HuskyMonitor._drain_subscriptions(monitor)


def test_header_stamp_is_read_from_the_bytes():
    """The stamp of any message starting with a header, without decoding it."""
    assert header_stamp(_joint_state(1792773484, 0.0)) == pytest.approx(1792773484.25)


@pytest.mark.parametrize("recording", [False, True])
def test_newest_only_decodes_the_newest_unless_its_event_is_listened(recording):
    """Three queued messages: all stamps noted; one callback, or three while a recording listens."""
    node, event, positions, stamps = _Node(), Event("x"), [], []
    if recording:
        event.attach(lambda _record: None)
    RosConnections(node).subscription(JointState, "j", lambda m: positions.append(m.position[0]), newest_only=event,
                                      on_stamp=stamps.append)
    node.subscriptions[0].queue.extend(_joint_state(10 + i, float(i)) for i in range(3))
    assert _drain(node) == 3
    assert stamps == pytest.approx([10.25, 11.25, 12.25])
    assert positions == ([0.0, 1.0, 2.0] if recording else [2.0])


def test_a_message_the_executor_takes_is_stamped_and_decoded():
    """One that arrives between the drain and the executor still counts for the rates, and updates the state."""
    node, positions, stamps = _Node(), [], []
    RosConnections(node).subscription(JointState, "j", lambda m: positions.append(m.position[0]), newest_only=True,
                                      on_stamp=stamps.append)
    node.subscriptions[0].callback(_joint_state(10, 1.0))
    assert (stamps, positions) == ([pytest.approx(10.25)], [1.0])


def test_a_garbled_message_is_dropped_not_fatal():
    """Too short to hold a stamp: both steps fail inside their guards, and the drain goes on."""
    node, stamps = _Node(), []
    RosConnections(node).subscription(JointState, "j", print, newest_only=True, on_stamp=stamps.append)
    node.subscriptions[0].queue.append(b"\x00\x01")
    assert _drain(node) == 1 and stamps == []


def test_on_stamp_needs_a_header_and_newest_only():
    """Refused for a message without a header, and for a subscription that decodes everything anyway."""
    ros = RosConnections(_Node())
    with pytest.raises(ValueError):
        ros.subscription(Twist, "t", print, newest_only=True, on_stamp=print)
    with pytest.raises(ValueError):
        ros.subscription(JointState, "j", print, on_stamp=print)
