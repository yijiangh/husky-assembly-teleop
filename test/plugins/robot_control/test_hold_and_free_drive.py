"""Hold to drive and free drive while held: a base drives at its page's speed, an arm frees only while held."""

from __future__ import annotations

from types import SimpleNamespace

import pytest
from robot_fakes import lease_context

from husky_assembly_teleop.plugins.robot_control.plugin import FREE_DRIVE_RETRY, RobotControlPlugin, _BaseWidgets
from husky_assembly_teleop.robot_interface.arm import FREE_DRIVE_CONTROLLER, SCALED_JOINT_TRAJECTORY_CONTROLLER
from husky_assembly_teleop.shared.teleop.dpad import HOLD_TIMEOUT, MAX_LINEAR_SPEED, Hold

pytestmark = pytest.mark.safety

SERIAL = "a200-0804"


def test_phone_hold_drives_at_the_phone_speed():
    """A hold drives at the speed slider of the page it came from, and stops once after the timeout."""
    sent = []
    base = SimpleNamespace(controllers=SimpleNamespace(is_active=lambda _name: True),
                           send_twist=lambda linear, angular: sent.append((linear, angular)), soft_stopped=False)
    ctx = lease_context(now=lambda: clock[0], log_warn=lambda _text: None)
    clock = [0.0]
    plugin = RobotControlPlugin()
    plugin._bases = [_BaseWidgets(serial=SERIAL, base=base, panel=None, status=None,
                                  speed=SimpleNamespace(value=0.5), phone_speed=SimpleNamespace(value=1.0))]

    plugin._on_hold(SERIAL, Hold(1.0, 0.0, plugin._bases[0].phone_speed, ctx.now()))
    plugin.update(ctx)
    plugin._on_hold(SERIAL, Hold(1.0, 0.0, plugin._bases[0].speed, ctx.now()))
    plugin.update(ctx)
    assert sent == [(MAX_LINEAR_SPEED, 0.0), (0.5 * MAX_LINEAR_SPEED, 0.0)]

    # Released: one stop after the timeout, then nothing.
    clock[0] = 2 * HOLD_TIMEOUT
    plugin.update(ctx)
    plugin.update(ctx)
    assert sent[2:] == [(0.0, 0.0)]


class _FakeControllers:
    """A controller manager whose switches answer at once, or are refused while `refuse` is set."""

    def __init__(self):
        self.state = SimpleNamespace(switch_in_flight=None, requested=None)
        self.active = SCALED_JOINT_TRAJECTORY_CONTROLLER
        self.switches = []
        self.refuse = False

    def is_active(self, controller):
        return self.active == controller

    def switch(self, controller, unguarded=False):
        self.switches.append(controller)
        if self.refuse:
            return False
        self.active = self.state.requested = controller
        return True


def _free_drive_plugin():
    """robot_control with one arm on a fake controller manager, and a clock to move by hand.

    The arm counts its free drive requests in `requests`; the laptop's Keep on is `plugin._arms[0].free_drive_keep`.
    """
    controllers = _FakeControllers()
    clock = [0.0]
    ctx = lease_context(now=lambda: clock[0], log_warn=lambda _text: None)
    plugin = RobotControlPlugin()
    arm = SimpleNamespace(controllers=controllers, config=SimpleNamespace(name="ur_arm"), requests=[],
                          soft_stopped=False)
    arm.request_free_drive = lambda: arm.requests.append(clock[0])
    plugin._arms = [SimpleNamespace(serial=SERIAL, arm=arm, free_drive_keep=SimpleNamespace(value=False))]
    return plugin, ctx, clock, controllers


def test_free_drive_while_held_then_joint_holds_the_arm():
    """Holding switches to free drive once; releasing switches to the joint controller."""
    plugin, ctx, clock, controllers = _free_drive_plugin()
    key = (SERIAL, "ur_arm")
    for t in (0.0, 0.05, 0.1):
        clock[0] = t
        plugin._on_free_drive_hold(ctx, key, True)
        plugin._update_free_drive(ctx, t)
    assert controllers.switches == [FREE_DRIVE_CONTROLLER]
    plugin._update_free_drive(ctx, 0.1 + 2 * HOLD_TIMEOUT)
    plugin._update_free_drive(ctx, 0.2 + 2 * HOLD_TIMEOUT)
    assert controllers.switches == [FREE_DRIVE_CONTROLLER, SCALED_JOINT_TRAJECTORY_CONTROLLER]


def test_free_drive_release_after_stop_all_starts_nothing():
    """After Stop all ended free drive, letting go must not start a controller again."""
    plugin, ctx, clock, controllers = _free_drive_plugin()
    plugin._on_free_drive_hold(ctx, (SERIAL, "ur_arm"), True)
    plugin._update_free_drive(ctx, 0.0)
    controllers.active = None  # the soft stop deactivated every controller
    plugin._update_free_drive(ctx, 2 * HOLD_TIMEOUT)
    assert controllers.switches == [FREE_DRIVE_CONTROLLER]
    assert not plugin._free_driving


def test_free_drive_refused_switch_waits_before_asking_again():
    """A refused switch is asked again after FREE_DRIVE_RETRY, not every tick."""
    plugin, ctx, clock, controllers = _free_drive_plugin()
    controllers.refuse = True
    for t in (0.0, 0.05, 0.1):
        clock[0] = t
        plugin._on_free_drive_hold(ctx, (SERIAL, "ur_arm"), True)
        plugin._update_free_drive(ctx, t)
    assert controllers.switches == [FREE_DRIVE_CONTROLLER]
    clock[0] = FREE_DRIVE_RETRY
    plugin._on_free_drive_hold(ctx, (SERIAL, "ur_arm"), True)
    plugin._update_free_drive(ctx, FREE_DRIVE_RETRY)
    assert controllers.switches == [FREE_DRIVE_CONTROLLER] * 2


def test_the_phone_button_switches_and_frees_the_arm_while_held():
    """Every tick the button is held asks the arm for free drive; once released, none."""
    plugin, ctx, clock, controllers = _free_drive_plugin()
    arm = plugin._arms[0].arm
    for t in (0.0, 0.05, 0.1):
        clock[0] = t
        plugin._on_free_drive_hold(ctx, (SERIAL, "ur_arm"), True)
        plugin._update_free_drive(ctx, t)
    plugin._update_free_drive(ctx, 0.1 + 2 * HOLD_TIMEOUT)
    assert arm.requests == [0.0, 0.05, 0.1] and controllers.switches[0] == FREE_DRIVE_CONTROLLER


def test_the_laptop_button_frees_the_arm_without_switching():
    """On the laptop the Free controller button switches; holding the button in its folder only frees the arm."""
    plugin, ctx, clock, controllers = _free_drive_plugin()
    controllers.switch(FREE_DRIVE_CONTROLLER)
    for t in (0.0, 0.05):
        clock[0] = t
        plugin._on_free_drive_hold(ctx, (SERIAL, "ur_arm"), False)
        plugin._update_free_drive(ctx, t)
    plugin._update_free_drive(ctx, 0.05 + 2 * HOLD_TIMEOUT)
    assert plugin._arms[0].arm.requests == [0.0, 0.05] and controllers.switches == [FREE_DRIVE_CONTROLLER]


def test_keep_on_frees_the_arm_every_tick_until_another_controller_runs():
    """Ticked, every tick asks for free drive; choosing another controller unticks it, so it is never left on."""
    plugin, ctx, clock, controllers = _free_drive_plugin()
    widgets = plugin._arms[0]
    controllers.switch(FREE_DRIVE_CONTROLLER)
    widgets.free_drive_keep.value = True
    for t in (0.0, 1.0, 2.0, 3.0):
        if t == 2.0:
            controllers.switch(SCALED_JOINT_TRAJECTORY_CONTROLLER)
        elif t == 3.0:
            controllers.switch(FREE_DRIVE_CONTROLLER)
        clock[0] = t
        plugin._update_free_drive(ctx, t)
    assert widgets.arm.requests == [0.0, 1.0] and not widgets.free_drive_keep.value
