"""Tests for the phone page: pinned ports, a plugin's tab and header, Stop all, the D-pad and free drive while held."""

from __future__ import annotations

import socket
import threading
from types import SimpleNamespace

import pytest

from husky_assembly_teleop.plugins.robot_control.dpad import HOLD_TIMEOUT, MAX_LINEAR_SPEED
from husky_assembly_teleop.plugins.robot_control.plugin import FREE_DRIVE_RETRY, RobotControlPlugin, _BaseWidgets
from husky_assembly_teleop.robot_interface.arm import FREE_DRIVE_CONTROLLER, SCALED_JOINT_TRAJECTORY_CONTROLLER
from husky_assembly_teleop.ui.visualization import Visualization

SERIAL = "a200-0804"


@pytest.fixture
def viz():
    """Both servers on free ports."""
    viz = Visualization(port=0, phone_port=0)
    yield viz
    viz.stop()


@pytest.fixture
def taken_port():
    """A port some other program is listening on."""
    with socket.socket() as sock:
        sock.bind(("0.0.0.0", 0))
        sock.listen()
        yield sock.getsockname()[1]


@pytest.mark.parametrize("which", ["desktop", "phone"])
def test_taken_port_refuses_to_start(taken_port, which):
    """A taken port stops the monitor from starting, instead of viser quietly moving to the next port."""
    threads = threading.active_count()
    ports = {"port": 0, "phone_port": 0, ("port" if which == "desktop" else "phone_port"): taken_port}
    with pytest.raises(RuntimeError, match=f"Port {taken_port} is taken"):
        Visualization(**ports)
    # Neither server is left running, also when the second one failed.
    assert threading.active_count() == threads


def test_phone_tab_on_first_use_and_removed_by_clear(viz):
    """A plugin gets one phone tab, made when first used; teardown removes the tab and both headers."""
    tabs = viz._phone.tabs
    view = viz.view_for("demo")
    assert tabs._tab_labels == ()
    with view.phone("Demo") as gui:
        gui.add_button("Go")
    with view.phone() as gui:
        gui.add_button("Again")
    with view.phone_header() as gui:
        gui.add_html("status")
    with view.header() as gui:
        gui.add_html("desktop status")
    assert tabs._tab_labels == ("Demo",)
    headers = (view._phone_header, view._header)
    view.clear()
    assert tabs._tab_labels == ()
    assert all(header._impl.removed for header in headers)


def test_phone_stop_all_raises_the_soft_stop(viz):
    """The phone's Stop all raises the same flag as the desktop's."""
    assert not viz.take_stop_request()
    for callback in viz._phone.stop._impl.update_cb:
        callback(None)
    assert viz.take_stop_request()


def _free_robot() -> SimpleNamespace:
    """The context robot_control keeps from setup, with SERIAL held by nobody."""
    return SimpleNamespace(world=SimpleNamespace(robots={SERIAL: SimpleNamespace(lease=None)}), log_warn=print)


def test_phone_hold_drives_at_the_phone_speed():
    """A hold drives at the speed slider of the page it came from, and stops once after the timeout."""
    sent = []
    base = SimpleNamespace(controllers=SimpleNamespace(is_active=lambda _name: True),
                           send_twist=lambda linear, angular: sent.append((linear, angular)))
    ctx = SimpleNamespace(now=lambda: clock[0])
    clock = [0.0]
    plugin = RobotControlPlugin()
    plugin._ctx = _free_robot()
    plugin._bases = [_BaseWidgets(serial=SERIAL, base=base, panel=None, status=None,
                                  speed=SimpleNamespace(value=0.5), phone_speed=SimpleNamespace(value=1.0))]

    plugin._on_hold(ctx, SERIAL, True, 1.0, 0.0)
    plugin.update(ctx)
    plugin._on_hold(ctx, SERIAL, False, 1.0, 0.0)
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
        self.state = SimpleNamespace(switch_in_flight=None)
        self.active = SCALED_JOINT_TRAJECTORY_CONTROLLER
        self.switches = []
        self.refuse = False

    def is_active(self, controller):
        return self.active == controller

    def switch(self, controller):
        self.switches.append(controller)
        if self.refuse:
            return False
        self.active = controller
        return True


def _free_drive_plugin():
    """robot_control with one arm on a fake controller manager, and a clock to move by hand."""
    controllers = _FakeControllers()
    clock = [0.0]
    ctx = SimpleNamespace(now=lambda: clock[0])
    plugin = RobotControlPlugin()
    plugin._ctx = _free_robot()
    arm = SimpleNamespace(controllers=controllers, config=SimpleNamespace(name="ur_arm"))
    plugin._arms = [SimpleNamespace(serial=SERIAL, arm=arm)]
    return plugin, ctx, clock, controllers


def test_free_drive_while_held_then_joint_holds_the_arm():
    """Holding switches to free drive once; releasing switches to the joint controller."""
    plugin, ctx, clock, controllers = _free_drive_plugin()
    key = (SERIAL, "ur_arm")
    for t in (0.0, 0.05, 0.1):
        clock[0] = t
        plugin._on_free_drive_hold(ctx, key)
        plugin._update_free_drive(t)
    assert controllers.switches == [FREE_DRIVE_CONTROLLER]
    plugin._update_free_drive(0.1 + 2 * HOLD_TIMEOUT)
    plugin._update_free_drive(0.2 + 2 * HOLD_TIMEOUT)
    assert controllers.switches == [FREE_DRIVE_CONTROLLER, SCALED_JOINT_TRAJECTORY_CONTROLLER]


def test_free_drive_release_after_stop_all_starts_nothing():
    """After Stop all ended free drive, letting go must not start a controller again."""
    plugin, ctx, clock, controllers = _free_drive_plugin()
    plugin._on_free_drive_hold(ctx, (SERIAL, "ur_arm"))
    plugin._update_free_drive(0.0)
    controllers.active = None  # the soft stop deactivated every controller
    plugin._update_free_drive(2 * HOLD_TIMEOUT)
    assert controllers.switches == [FREE_DRIVE_CONTROLLER]
    assert not plugin._free_driving


def test_free_drive_refused_switch_waits_before_asking_again():
    """A refused switch is asked again after FREE_DRIVE_RETRY, not every tick."""
    plugin, ctx, clock, controllers = _free_drive_plugin()
    controllers.refuse = True
    for t in (0.0, 0.05, 0.1):
        clock[0] = t
        plugin._on_free_drive_hold(ctx, (SERIAL, "ur_arm"))
        plugin._update_free_drive(t)
    assert controllers.switches == [FREE_DRIVE_CONTROLLER]
    clock[0] = FREE_DRIVE_RETRY
    plugin._on_free_drive_hold(ctx, (SERIAL, "ur_arm"))
    plugin._update_free_drive(FREE_DRIVE_RETRY)
    assert controllers.switches == [FREE_DRIVE_CONTROLLER] * 2
