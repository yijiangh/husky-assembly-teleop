"""The Force plots window: both arms' force and torque recorded tick after tick, over moves, until Clear."""

from __future__ import annotations

import asyncio

import numpy as np
from test_demo_preview import Panel
from test_unscrew_retreat import INSERTED, START


def test_force_plots_record_both_arms_all_along_and_clear_starts_again(monkeypatch):
    """Readings before, during and after a transit all stay; the peak readout keeps a push; Clear forgets."""
    panel = Panel(monkeypatch, np.linspace(INSERTED, START, 5))
    left, right = panel.lab.robot.arms.values()
    readout = panel.ctx.gui.of_kind("html")[-1]
    force, torque = panel.ctx.gui.of_kind("uplot")[-2:]
    assert [plot.options["title"] for plot in (force, torque)] == ["force [N]", "torque [Nm]"]

    async def ticks(count: int) -> None:
        for _ in range(count):
            await panel.ctx.next_tick()
            panel.draw()
            panel.ctx.t += 0.5  # past the redraw interval

    async def scenario():
        assert "—" in readout.content
        left.state.wrench, right.state.wrench = np.array([3.0, 4.0, 0, 0, 0, 0.1]), np.array([0, 0, -2.0, 0, 0, 0])
        await ticks(3)
        assert "+5.0" in readout.content and "-2.0" not in readout.content and "+2.0" in readout.content
        left.state.wrench = np.array([0, 0, 30.0, 0, 0, 0])
        await ticks(1)
        left.state.wrench = np.zeros(6)
        await panel.click("Transit")
        await panel.click("Execute")
        await ticks(2)
        plotted = force.data
        assert len(plotted) == 7 and len(plotted[0]) >= 20, "every tick of the move kept"
        assert np.nanmax(plotted[3]) == 30.0 and np.nanmin(plotted[6]) == -2.0
        assert len(torque.data[0]) == len(plotted[0]) and np.nanmax(torque.data[3]) == 0.1
        assert "+30.0" in readout.content.split("peak")[1] and "+0.0" in readout.content.split("peak")[0]
        panel.widget("Clear").callback()
        await ticks(1)
        assert len(force.data[0]) == 1 and "+30.0" not in readout.content

    asyncio.run(scenario())
