"""The F/T watch of robot_control: the raw force and torque traced every tick into two live plots per arm."""

from __future__ import annotations

import numpy as np
from plugin_fakes import FakeContext, fake_arm, fake_node, fake_robot

from husky_assembly_teleop.plugin_api.signals import force, torque
from husky_assembly_teleop.plugins.robot_control.plugin import FT_SAMPLES, RobotControlPlugin
from husky_assembly_teleop.ui.trace_plot import BLANK, TracePlot

ARM, SERIAL = "ur_arm", "a200-0804"
JOINTS = np.radians([10.0, -100.0, 90.0, -80.0, -90.0, 30.0])


class FakeUplot:
    """What `gui.add_uplot` gives back: keeps the data and scales set on it."""

    def __init__(self, **fields):
        """Keep the data and scales the plot was built with."""
        self.data, self.scales = fields.get("data"), fields.get("scales")


def _watched(wrench) -> tuple[FakeContext, object, list[TracePlot]]:
    """A context with one arm reporting `wrench`, its F/T trace recording, and the two plots on a fake GUI."""
    ctx = FakeContext(name="robot_control")
    node = fake_node(ctx.now, ctx.logs)
    arm = fake_arm(node, ARM, JOINTS)
    arm.state.wrench = wrench
    ctx.world.robots[SERIAL] = fake_robot(node, SERIAL, [arm])
    gui = ctx.gui
    gui.add_uplot = lambda **fields: FakeUplot(**fields)
    signals = (force(arm), torque(arm))
    trace = ctx.trace(*signals, max_samples=FT_SAMPLES)
    plots = [TracePlot(gui, trace, signal, seconds=30.0, min_span=span) for signal, span in zip(signals, (1.0, 0.1))]
    return ctx, arm, plots


def test_the_force_and_torque_reach_the_plots_tick_by_tick():
    """Three ticks of a steady wrench: three rows per plot, the force on one, the torque on the other."""
    ctx, _arm, plots = _watched(np.array([1.0, 2.0, 3.0, 0.1, 0.2, 0.3]))
    for plot in plots:
        assert len(plot.handle.data[0]) == 1 and np.isnan(plot.handle.data[1][0]), "built with one blank row"
    import asyncio
    asyncio.run(_ticks(ctx, 3))
    for plot in plots:
        plot.draw()
    assert len(plots[0].handle.data[0]) == 3
    np.testing.assert_allclose(np.array(plots[0].handle.data[1:]).T[-1], [1.0, 2.0, 3.0])
    np.testing.assert_allclose(np.array(plots[1].handle.data[1:]).T[-1], [0.1, 0.2, 0.3])


def test_without_a_wrench_the_plots_keep_a_row_and_a_flat_axis():
    """Fake hardware reports no wrench: the rows are NaN, the plot never has zero rows, the axis stays readable."""
    ctx, _arm, plots = _watched(None)
    import asyncio
    asyncio.run(_ticks(ctx, 2))
    for plot in plots:
        plot.draw()
        assert len(plot.handle.data[0]) == 2 and np.all(np.isnan(plot.handle.data[1]))
    assert plots[0].handle.scales["y"]["range"] == (-1.0, 1.0)


async def _ticks(ctx: FakeContext, count: int) -> None:
    for _ in range(count):
        await ctx.next_tick()


def test_the_plugin_builds_the_watch_last_in_the_arms_tab(monkeypatch):
    """`_build_arm` on the fake GUI ends with the Live plots folder: two uplots and the note under them."""
    import husky_assembly_teleop.plugins.robot_control.plugin as robot_control_module
    for name in ("add_frame_markers", "add_force_arrows"):
        monkeypatch.setattr(robot_control_module, name, lambda *_args: {})
    monkeypatch.setattr(robot_control_module, "add_target_gizmo", lambda *_args: None)
    ctx, arm, _plots = _watched(None)
    ctx.gui.add_uplot = lambda **fields: FakeUplot(**fields)
    widgets = RobotControlPlugin()._build_arm(ctx, ctx.gui, SERIAL, arm)
    assert len(widgets.ft_plots) == 2 and widgets.ft_trace is not None and widgets.ft_note is not None
    assert BLANK[0].shape == (1,)
