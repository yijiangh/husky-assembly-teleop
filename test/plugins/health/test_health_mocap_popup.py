"""Health's warning window for an amber mocap chip: when it opens, what its buttons do, and that it needs no draw."""

from __future__ import annotations

import asyncio
from contextlib import nullcontext
from types import SimpleNamespace

import pytest
from plugin_fakes import FakeContext

from husky_assembly_teleop.plugins import health as health_module
from husky_assembly_teleop.plugins.health import ACKNOWLEDGE, USE_REFIT, HealthPlugin
from husky_assembly_teleop.robot_interface.soft_stop import SoftStopLatch
from husky_assembly_teleop.shared.execution import moving
from husky_assembly_teleop.ui.popup import POPUP_X
from husky_assembly_teleop.world.base_mocap import ASK_AFTER
from husky_assembly_teleop.world.checks import GOOD, WARN, Check

ALICE, CINDY = "a200-0804", "a200-0806"
FRAME = "Motive's frame moved: poses off until Use refit (Mocap probe → Floor calibration)"


@pytest.fixture
def lab(monkeypatch):
    """Health with two robots whose mocap chips are in `chips`, on a clock of its own; nothing moves."""
    chips = {ALICE: Check("mocap", GOOD, "fine"), CINDY: Check("mocap", GOOD, "fine")}
    monkeypatch.setattr(health_module, "robot_checks",
                        lambda robot, *_args: [("base", [chips[robot.config.serial]])])
    clock = SimpleNamespace(t=0.0)
    monkeypatch.setattr(health_module.time, "monotonic", lambda: clock.t)
    moving_now = SimpleNamespace(why="")
    monkeypatch.setattr(moving, "moving_reason", lambda *_args: moving_now.why)
    ctx = FakeContext()
    ctx.view.header = ctx.view.phone_header = lambda: nullcontext(ctx.gui)
    ctx.view.folder = lambda **_options: nullcontext(ctx.gui)
    state = SimpleNamespace(frame=None, source=lambda _id: "motive", calibrations={})
    refits = []
    relay = SimpleNamespace(state=state, use_refit=lambda: refits.append(1) or relay.future)
    relay.future = None
    ctx.world.mocap_relay = relay
    ctx.world.robots = {serial: SimpleNamespace(config=SimpleNamespace(serial=serial, mocap_id=1840), arms={},
                                                soft_stop_latch=SoftStopLatch(serial),
                                                lease_slot=SimpleNamespace(lease=None), parked=False)
                        for serial in (ALICE, CINDY)}
    health = HealthPlugin()
    health.setup(ctx)
    return SimpleNamespace(health=health, ctx=ctx, chips=chips, clock=clock, state=state, relay=relay, refits=refits,
                           moving=moving_now)


def tick(lab, t: float) -> None:
    """One monitor tick while the panels are frozen: update only, no draw."""
    lab.clock.t = t
    lab.health.update(lab.ctx)


def amber_for_a_while(lab, *serials: str) -> None:
    """Turn these chips amber, the frame moved, long enough to ask."""
    lab.state.frame = SimpleNamespace(status="refit_pending", poses_off=True)
    for serial in serials:
        lab.chips[serial] = Check("mocap", WARN, FRAME)
    tick(lab, 0.0)
    tick(lab, ASK_AFTER)


def press(lab, label: str) -> None:
    """Press one of the window's buttons."""
    lab.health._mocap_popup.buttons[label].callback(None)


def test_the_window_starts_hidden_beside_the_main_panel(lab):
    """It floats past the main panel, so Stop all stays in reach; green chips never open it."""
    tick(lab, 0.0)
    tick(lab, 100.0)
    assert not lab.health._mocap_popup.is_open and lab.ctx.panels[-1].placed["x"] == POPUP_X


def test_one_window_names_every_robot_with_the_same_reason(lab):
    """Alice and Cindy trip together, and a refit is one action for both: one line, without a draw."""
    amber_for_a_while(lab, ALICE, CINDY)
    text = lab.health._mocap_popup._text.content
    assert lab.health._mocap_popup.is_open and f"{ALICE}, {CINDY}" in text and "frame moved" in text
    assert text.count("frame moved") == 1 and not lab.health._mocap_popup.buttons[USE_REFIT].disabled


def test_use_refit_is_greyed_when_a_refit_is_not_what_helps(lab):
    """A marker error: only Acknowledge."""
    lab.state.frame = SimpleNamespace(status="held", poses_off=False)
    lab.chips[CINDY] = Check("mocap", WARN, "mean marker error over 1s above 1 mm")
    tick(lab, 0.0)
    tick(lab, ASK_AFTER)
    assert lab.health._mocap_popup.is_open and lab.health._mocap_popup.buttons[USE_REFIT].disabled


def test_a_robot_without_a_rigid_body_never_asks(lab):
    """Its chip is amber for good, and nothing is wrong: it is only never measured."""
    lab.ctx.world.robots[CINDY].config.mocap_id = None
    lab.chips[CINDY] = Check("mocap", WARN, "no rigid body configured")
    tick(lab, 0.0)
    tick(lab, ASK_AFTER)
    assert not lab.health._mocap_popup.is_open


def test_acknowledge_closes_it_and_it_stays_closed_while_amber(lab):
    """The operator decides what to do: the chips stay amber, the window stays away, the log says who saw what."""
    amber_for_a_while(lab, CINDY)
    press(lab, ACKNOWLEDGE)
    assert not lab.health._mocap_popup.is_open
    tick(lab, ASK_AFTER + 10)
    assert not lab.health._mocap_popup.is_open and not lab.refits
    assert any("acknowledged" in line and CINDY in line for line in lab.ctx.lines("info"))


def test_the_window_closes_itself_once_the_chip_is_green(lab):
    """The frame came back on its own: nothing left to decide."""
    amber_for_a_while(lab, CINDY)
    lab.chips[CINDY] = Check("mocap", GOOD, "fine")
    tick(lab, ASK_AFTER + 1)
    assert not lab.health._mocap_popup.is_open


def test_use_refit_is_refused_while_anything_moves(lab):
    """Every pose changes with it: the relay is not asked, the window stays open and says why."""
    amber_for_a_while(lab, CINDY)
    lab.moving.why = "a200-0806 left arm moves"
    press(lab, USE_REFIT)
    tick(lab, ASK_AFTER + 1)
    assert not lab.refits and lab.health._mocap_popup.is_open
    assert "left arm moves" in lab.health._mocap_popup._text.content


def test_use_refit_says_so_when_the_relay_does_not_answer(lab):
    """It asked, nobody answers: the window says that rather than closing."""
    amber_for_a_while(lab, CINDY)
    press(lab, USE_REFIT)
    tick(lab, ASK_AFTER + 1)
    assert lab.refits == [1] and "does not answer" in lab.health._mocap_popup._text.content


def test_use_refit_shows_the_relay_s_answer_then_closes_with_the_chips(lab):
    """A refit used: the frame is held again, the chips turn green, and the window goes by itself."""
    amber_for_a_while(lab, CINDY)
    lab.relay.future = object()

    async def answered(_future):
        return SimpleNamespace(success=True, message="refit used")

    async def press_and_wait():
        press(lab, USE_REFIT)
        await asyncio.gather(*lab.ctx._tasks)

    lab.ctx.ros = answered
    asyncio.run(press_and_wait())
    tick(lab, ASK_AFTER + 1)
    assert lab.refits == [1] and "refit used" in lab.health._mocap_popup._text.content
    lab.chips[CINDY] = Check("mocap", GOOD, "fine")
    tick(lab, ASK_AFTER + 2)
    assert not lab.health._mocap_popup.is_open
