"""The base refuses commands that are not finite, before anything goes out."""

import itertools
import math
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time

from husky_assembly_teleop.robot_interface.base import BaseInterface

pytestmark = pytest.mark.safety


class _Publisher:
    """Keeps what is published."""

    def __init__(self):
        """Nothing published yet."""
        self.sent = []

    def publish(self, message):
        """Keep the message."""
        self.sent.append(message)


def _base() -> BaseInterface:
    """A base with recording publishers and a running velocity controller, without ROS."""
    base = BaseInterface.__new__(BaseInterface)
    clock = SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time))
    base._node = SimpleNamespace(get_logger=lambda: SimpleNamespace(error=lambda *_a, **_k: None),
                                 get_clock=lambda: clock)
    base.controllers = SimpleNamespace(namespace="alice", is_active=lambda _name: True)
    base._cmd_vel, base._base_path, base._path_ids = _Publisher(), _Publisher(), itertools.count(1)
    return base


@pytest.mark.parametrize("twist", [(math.nan, 0.0), (0.0, math.inf), (-math.inf, math.nan)])
def test_twist_not_finite_is_refused(twist):
    """send_twist returns False and publishes nothing."""
    base = _base()
    assert not base.send_twist(*twist)
    assert not base._cmd_vel.sent


@pytest.mark.parametrize("path, why", [
    (([0.0, math.nan], [0.0, 0.0], [0.0, 0.0], None, 0.3, 0.5), "not finite"),
    (([0.0, 1.0], [0.0, 0.0], [0.0, 0.0], [0.0, math.inf], 0.3, 0.5), "not finite"),
    (([0.0, 1.0], [0.0, 0.0], [0.0, 0.0], None, math.nan, 0.5), "not finite"),
    (([0.0, 1.0], [0.0], [0.0, 0.0], None, 0.3, 0.5), "differ in length"),
    (([0.0, 1.0], [0.0, 0.0], [0.0, 0.0], [0.0], 0.3, 0.5), "differ in length"),
])
def test_path_not_finite_or_ragged_is_refused(path, why):
    """send_path raises before it takes a path id or publishes."""
    base = _base()
    with pytest.raises(ValueError, match=why):
        base.send_path(*path)
    assert not base._base_path.sent and next(base._path_ids) == 1


def test_finite_commands_go_out():
    """A finite twist and a finite path are published."""
    base = _base()
    assert base.send_twist(0.2, -0.1)
    assert base.send_path([0.0, 1.0], [0.0, 0.0], [0.0, 0.0], [0.0, 2.0], 0.3, 0.5) == 1
    assert len(base._cmd_vel.sent) == 1 and list(base._base_path.sent[0].t) == [0.0, 2.0]
