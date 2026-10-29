"""The mocap offset of robot_control's Base tab: Apply shifts the base's mocap pose, Reset puts it back."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
from plugin_fakes import FakeContext
from robot_fakes import Node

from husky_assembly_teleop.plugins.robot_control.plugin import BASE, RobotControlPlugin
from husky_assembly_teleop.plugins.robot_control.status import base_status
from husky_assembly_teleop.robot_interface.base import BaseInterface, BaseState

SERIAL = "a200-0804"


def _base() -> BaseInterface:
    """A base with a tracked pose at ROS time 100 s, and nothing else."""
    base = BaseInterface.__new__(BaseInterface)
    base._node, base._config = Node(100.0), SimpleNamespace(serial=SERIAL, mocap_id=1)
    base.state = BaseState(tracked=True, tracking_valid=True, last_update_time=100.0, last_fix_time=100.0)
    base.state.position, base.state.orientation = np.array([1.0, 2.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])
    base.state.controllers.last_update_time = 100.0
    base.controllers = SimpleNamespace(state=base.state.controllers, is_active=lambda _c: False)
    return base


def test_apply_shifts_the_base_and_shows_a_chip_and_reset_clears_both():
    """Typed 0.05, -0.02, 0 and Apply: the pose moves, a chip says so, a log line too; Reset zeroes inputs and pose."""
    ctx, base, plugin = FakeContext(name="robot_control"), _base(), RobotControlPlugin()
    widgets = SimpleNamespace(serial=SERIAL, base=base,
                              offset=tuple(SimpleNamespace(value=value) for value in (0.05, -0.02, 0.0)))
    plugin._set_offset(ctx, widgets, apply=True)
    np.testing.assert_allclose(base.mocap_offset, [0.05, -0.02, 0.0])
    np.testing.assert_allclose(base.state.position, [1.05, 1.98, 0.0])
    assert any("mocap offset applied: 0.050 -0.020 0.000 m" in line for _level, line in ctx.logs)
    assert "offset 0.050 -0.020 0.000 m" in base_status(base, 100.0)
    plugin._set_offset(ctx, widgets, apply=False)
    assert [number.value for number in widgets.offset] == [0.0, 0.0, 0.0]
    np.testing.assert_allclose(base.mocap_offset, 0.0)
    np.testing.assert_allclose(base.state.position, [1.0, 2.0, 0.0])
    assert "offset " not in base_status(base, 100.0) and (SERIAL, BASE) not in plugin._refusals
