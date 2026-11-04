"""Health's panel as drawn: the refusal line under a robot's buttons, and the held signal chip in rows and log."""

from __future__ import annotations

import math
from contextlib import nullcontext
from types import SimpleNamespace

import pytest
from plugin_fakes import FakeContext
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Scene

from husky_assembly_teleop.plugins import health as health_module
from husky_assembly_teleop.plugins.health import LOG_HOLD, HealthPlugin, moved_since, render_line
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
                            soft_stop_latch=SoftStopLatch(SERIAL), lease_slot=SimpleNamespace(lease=None), parked=False)
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


def standing(x: float = 0.0, joint: float = 0.0, base_tracked: bool = True, unmeasured=frozenset()) -> RobotObject:
    """The robot as the snapshot holds it: its base at `x`, its one arm joint at `joint` (radians)."""
    model = SimpleNamespace(movable_joints=("ur_arm_shoulder_pan_joint",))
    return RobotObject(f"robots/{SERIAL}", model, Pose((x, 0.0, 0.0)), {"ur_arm_shoulder_pan_joint": joint},
                       base_tracked=base_tracked, unmeasured=frozenset(unmeasured))


def test_the_parked_button_toggles_the_mark_and_is_filled_while_on(panel):
    """Parked, on one row with the robot's buttons: a click parks the robot, filled; another unparks it."""
    panel.ctx.scene.snapshot = Scene(robots={f"robots/{SERIAL}": standing()})
    buttons = panel.ctx.gui.of_kind("button_group")[0]
    style = panel.health._park_styles[SERIAL]
    buttons.callback("Parked")
    assert panel.robot.parked
    filled = "button:nth-child(4) { background"
    assert "display: none" in style.content and filled in style.content, "one row, Parked filled"
    buttons.callback("Parked")
    assert not panel.robot.parked
    assert "display: none" in style.content and filled not in style.content
    assert style.content.count("::after { display: block") == 4, "a tip per button"


def test_a_parked_robot_seen_moving_is_unparked_with_a_warning(panel):
    """Standing still or not measured, it stays parked; its base measured 3 cm away, it is unparked."""
    panel.ctx.scene.snapshot = Scene(robots={f"robots/{SERIAL}": standing()})
    panel.ctx.gui.of_kind("button_group")[0].callback("Parked")
    for still in (standing(x=0.01, joint=0.01), standing(x=1.0, base_tracked=False)):
        panel.ctx.scene.snapshot = Scene(robots={f"robots/{SERIAL}": still})
        tick(panel, 0.0)
        assert panel.robot.parked
    panel.ctx.scene.snapshot = Scene(robots={f"robots/{SERIAL}": standing(x=0.03)})
    tick(panel, 0.1)
    assert not panel.robot.parked
    assert panel.ctx.lines("warn") == [f"{SERIAL} no longer parked: base moved 3.0 cm, 0.0°"]


def test_a_robot_unparked_elsewhere_shows_unparked(panel):
    """Leased to move, the robot is unparked outside Health: the next tick shows Parked unfilled."""
    panel.ctx.scene.snapshot = Scene(robots={f"robots/{SERIAL}": standing()})
    panel.ctx.gui.of_kind("button_group")[0].callback("Parked")
    panel.robot.parked = False
    tick(panel, 0.0)
    assert "button:nth-child(4) { background" not in panel.health._park_styles[SERIAL].content


@pytest.mark.parametrize("now, moved", [
    (standing(joint=math.radians(2.0)), "ur_arm_shoulder_pan_joint turned 2.0°"),
    (standing(joint=math.radians(2.0), unmeasured={"ur_arm_shoulder_pan_joint"}), ""),
    (standing(x=0.5, base_tracked=False), ""),
], ids=["a joint turned", "a joint not measured", "a base not tracked"])
def test_only_what_is_measured_can_be_seen_moving(now, moved):
    """A joint turned 2° counts; the same joint not measured, or a base not tracked, cannot be seen to move."""
    assert moved_since(standing(), now) == moved
