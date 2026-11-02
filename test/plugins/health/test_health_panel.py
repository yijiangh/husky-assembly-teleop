"""Health's panel as drawn: the refusal line under a robot's buttons, and the held signal chip in rows and log."""

from __future__ import annotations

from contextlib import nullcontext
from types import SimpleNamespace

import pytest
from plugin_fakes import FakeContext

from husky_assembly_teleop.plugins import health as health_module
from husky_assembly_teleop.plugins.health import LOG_HOLD, HealthPlugin, render_line
from husky_assembly_teleop.robot_interface.soft_stop import SoftStopLatch
from husky_assembly_teleop.ui.style import REFUSAL_SHOWN, chip_line
from husky_assembly_teleop.world.checks import GOOD, WARN, Check

SERIAL = "a200-0805"


@pytest.fixture
def panel(monkeypatch):
    """Health set up for one robot whose base line has only the signal chip, at the level in `signal` (a list)."""
    signal = [Check("signal", GOOD, "on time")]
    monkeypatch.setattr(health_module, "robot_checks", lambda *_args: [("base", [signal[0]])])
    clock = SimpleNamespace(t=0.0)
    monkeypatch.setattr(health_module.time, "monotonic", lambda: clock.t)
    ctx = FakeContext()
    ctx.view.header = ctx.view.phone_header = lambda: nullcontext(ctx.gui)
    ctx.view.folder = lambda **_options: nullcontext(ctx.gui)
    robot = SimpleNamespace(config=SimpleNamespace(serial=SERIAL, mocap_id=None), arms={},
                            soft_stop_latch=SoftStopLatch(SERIAL), lease_slot=SimpleNamespace(lease=None))
    ctx.world.robots, ctx.world.mocap_relay = {SERIAL: robot}, None
    health = HealthPlugin()
    health.setup(ctx)
    return SimpleNamespace(health=health, ctx=ctx, robot=robot, signal=signal, clock=clock)


def tick(panel, t: float) -> None:
    """One monitor tick at wall clock `t`: update, then draw."""
    panel.clock.t = t
    panel.health.update(panel.ctx)
    panel.health.draw(panel.ctx)


def test_a_refused_button_shows_under_the_buttons_for_refusal_shown(panel):
    """Unlock while leased: a red chip on the line under the buttons, gone REFUSAL_SHOWN later; the line stays."""
    panel.robot.lease_slot.lease = SimpleNamespace(holder="assembly_run")
    tick(panel, 0.0)
    line = panel.health._refusal_lines[SERIAL]
    assert line.content == chip_line("")
    panel.ctx.gui.of_kind("button_group")[0].callback("Unlock")
    tick(panel, 0.1)
    assert "Unlock refused" in line.content and "leased by assembly_run" in line.content
    panel.ctx.t += REFUSAL_SHOWN + 0.1
    tick(panel, 0.2)
    assert line.content == chip_line("")


def test_the_held_signal_level_is_what_is_drawn_and_logged(panel):
    """Signal flapping green and amber: the row shows amber steadily, and the log says it turned amber, once."""
    warn = Check("signal", WARN, "a gap over 100 ms")
    tick(panel, 0.0)
    for step in range(1, round((LOG_HOLD + 1.0) / 0.1)):  # long enough for the change to be logged
        panel.signal[0] = warn if step % 2 else Check("signal", GOOD, "on time")
        tick(panel, step * 0.1)
        assert render_line("base", [warn]) in panel.health._robot_rows[SERIAL].content
    assert [line for line in panel.ctx.logs if "signal" in line[1]] == [
        ("warn", f"{SERIAL} base: signal green -> amber: a gap over 100 ms")]
