"""The arm's force/torque events: every reading while something listens, and each zeroing with what it tares."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
from geometry_msgs.msg import WrenchStamped
from robot_fakes import fake_arm


def _reading(force_z: float) -> WrenchStamped:
    """A reading of `force_z` newtons along z, stamped at 99.5 s."""
    message = WrenchStamped()
    message.wrench.force.z = force_z
    message.header.stamp.sec, message.header.stamp.nanosec = 99, 500_000_000
    return message


def test_every_reading_is_fired_only_while_something_listens():
    """Unheard, a reading only updates the state; heard, it is fired with its stamp."""
    arm, seen = fake_arm(), []
    arm._on_wrench(_reading(1.0))
    arm.wrench_messages.attach(seen.append)
    arm._on_wrench(_reading(2.0))
    assert arm.state.wrench[2] == 2.0
    assert [(reading.wrench[2], reading.stamp) for reading in seen] == [(2.0, 99.5)]


def test_a_zeroing_is_fired_with_the_reading_it_tares():
    """The zero request goes out and `ft_zeros_sent` gets the reading just before."""
    arm, seen = fake_arm(), []
    arm._zero_ft = SimpleNamespace(service_is_ready=lambda: True, calls=[])
    arm._zero_ft.call_async = arm._zero_ft.calls.append
    arm.ft_zeros_sent.attach(seen.append)
    arm._on_wrench(_reading(7.0))
    assert arm.zero_ft_sensor()
    assert len(arm._zero_ft.calls) == 1 and np.array_equal(seen[0].wrench, [0.0, 0.0, 7.0, 0.0, 0.0, 0.0])
