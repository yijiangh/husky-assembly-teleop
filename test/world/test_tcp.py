"""Tests for world.tcp.tcp_check: the reported tool0 against the kinematics' tool0, judged while the arm is still."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from husky_assembly_teleop.world.tcp import TCP_POSITION_TOLERANCE, tcp_check
from husky_assembly_teleop.robot_interface.arm import STILL_AFTER
from husky_assembly_teleop.world.checks import GOOD, WARN
from scaffolding_assembly_core.geometry import Pose

pytestmark = pytest.mark.safety

#: tool0 where the kinematics put it, in the arm's base_link.
EXPECTED = Pose((0.4, -0.2, 0.3))
NOW = 100.0


def arm(position=EXPECTED.position, updated=NOW, moved=NOW - 2 * STILL_AFTER):
    """The smallest stand-in for an ArmInterface: its TCP state and its tool's TCP."""
    state = SimpleNamespace(tcp_position=None if position is None else np.array(position, dtype=float),
                            tcp_orientation=np.array([0.0, 0.0, 0.0, 1.0]), tcp_update_time=updated,
                            last_motion_time=moved)
    return SimpleNamespace(state=state, pendant_tcp=Pose((0.0, 0.0, 0.15)))


def test_agreeing_tcp_is_good():
    """Reported and computed tool0 agree."""
    assert tcp_check(arm(), EXPECTED, NOW).level == GOOD


def test_an_offset_is_amber_and_names_the_tools_tcp():
    """5 mm off: amber, with the size and the TCP the pendant should have."""
    check = tcp_check(arm(position=(0.405, -0.2, 0.3)), EXPECTED, NOW)
    assert check.level == WARN and "5 mm" in check.detail and "150.0" in check.detail
    assert 5e-3 > TCP_POSITION_TOLERANCE


def test_no_verdict_while_the_arm_moves():
    """Joints and TCP come in separate messages: a moving arm is not judged."""
    assert tcp_check(arm(position=(1.0, 0.0, 0.0), moved=NOW - STILL_AFTER / 2), EXPECTED, NOW) is None


def test_missing_or_old_tcp_is_amber():
    """No report yet, or none for a while."""
    assert tcp_check(arm(position=None), EXPECTED, NOW).level == WARN
    assert tcp_check(arm(updated=NOW - 10.0), EXPECTED, NOW).level == WARN
