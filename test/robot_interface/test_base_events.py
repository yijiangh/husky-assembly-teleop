"""The base's message events: each follower report and wheel odometry message is stored, then fired as its record."""

from types import SimpleNamespace

import numpy as np

from crl_husky_msgs.msg import BaseFollowerState
from nav_msgs.msg import Odometry
from robot_fakes import Node

from husky_assembly_teleop.robot_interface.base import BaseInterface, BaseState
from husky_assembly_teleop.robot_interface.event import Event


def _base() -> BaseInterface:
    """A base at ROS time 100 s with its two events, and nothing else."""
    base = BaseInterface.__new__(BaseInterface)
    base._node, base.state = Node(100.0), BaseState()
    base.follower_reports, base.odometry_messages = Event("x_follower"), Event("x_odometry")
    return base


def test_follower_report_is_stored_before_it_is_fired():
    """A listener gets the report the state already holds, with the time it was handled and its stamp."""
    base, seen = _base(), []
    base.follower_reports.attach(lambda record: seen.append((record, base.state.follower)))
    message = BaseFollowerState(controller="pure_pursuit", state=1, linear_velocity=0.2, angular_velocity=-0.1)
    message.header.stamp.sec, message.header.stamp.nanosec = 99, 500_000_000
    base._on_follower(message)
    (record, stored), = seen
    assert record is stored and record.state == "following"
    assert record.command.tolist() == [0.2, -0.1] and (record.update_time, record.stamp) == (100.0, 99.5)


def test_a_mocap_offset_is_put_on_the_followers_points():
    """The follower's reference and lookahead come in its own world: the offset is added so they match the pose."""
    base = _base()
    base.mocap_offset = np.array([0.1, 0.2, 0.0])
    message = BaseFollowerState(controller="pure_pursuit", state=1, reference_x=1.0, reference_y=1.0,
                                reference_yaw=0.5, lookahead_x=2.0, lookahead_y=2.0, cross_track_error=0.01)
    base._on_follower(message)
    report = base.state.follower
    assert report.reference.tolist() == [1.1, 1.2, 0.5] and report.lookahead.tolist() == [2.1, 2.2]
    assert report.cross_track_error == 0.01


def test_odometry_is_stored_and_fired():
    """Wheel odometry becomes `state.odometry`, then a record for `odometry_messages`."""
    base, seen = _base(), []
    base.odometry_messages.attach(seen.append)
    message = Odometry()
    message.twist.twist.linear.x, message.twist.twist.angular.z = 0.3, 0.4
    message.header.stamp.sec = 99
    base._on_odometry(message)
    record, = seen
    assert record is base.state.odometry and record.velocity.tolist() == [0.3, 0.4]
    assert (record.update_time, record.stamp) == (100.0, 99.0)


def test_event_detaches_and_ignores_unknown_listeners():
    """A detached listener hears nothing more; detaching one never attached is fine."""
    event, seen = Event("e"), []
    event.attach(seen.append)
    event.fire(SimpleNamespace())
    event.detach(seen.append)
    event.detach(print)
    event.fire(SimpleNamespace())
    assert len(seen) == 1
