"""Tests for the phone page: pinned ports, a plugin's tab and header, Stop all, the D-pad and free drive while held."""

from __future__ import annotations

import socket
import threading
from types import SimpleNamespace

import pytest
from robot_fakes import lease_context

from husky_assembly_teleop.shared.teleop.dpad import HOLD_TIMEOUT, MAX_LINEAR_SPEED, Hold
from husky_assembly_teleop.plugins.robot_control.plugin import FREE_DRIVE_RETRY, RobotControlPlugin, _BaseWidgets
from husky_assembly_teleop.robot_interface.arm import FREE_DRIVE_CONTROLLER, SCALED_JOINT_TRAJECTORY_CONTROLLER
from husky_assembly_teleop.ui.phone import (SHEET, big_buttons, button_group_color_css, css_string, group_buttons,
                                            group_tooltips_css, one_row_css,
                                            side_by_side_css, slim_labels_css)
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


def test_big_buttons_on_the_phone_sheet_or_the_whole_page():
    """By default only the phone sheet's buttons grow; an empty scope makes the rule apply on the whole page."""
    assert big_buttons(["check"], 40, 18).startswith(f"{SHEET} button:has(.icon-tabler-check) {{ height: 40px")
    assert big_buttons(["check", "x"], 40, 18, scope="").startswith(
        "button:has(.icon-tabler-check), button:has(.icon-tabler-x) { height: 40px")


def test_slim_labels_shrink_the_label_box_of_each_input():
    """One rule per input, found by its label's `for`, overriding viser's inline width."""
    rule = 'div:has(> p > label[for="{}"]) {{ width: 3.0em !important; }}'
    assert slim_labels_css(["a1", "b2"], 3.0) == rule.format("a1") + rule.format("b2")


def test_side_by_side_buttons_share_a_row_by_their_ids():
    """Each button's box, found by the button's id, takes its share of the row, less viser's side margins."""
    css = side_by_side_css(["u1", "u2"])
    assert css.count("display: inline-block; width: calc(50% - 2 * var(--mantine-spacing-xs))") == 2
    assert 'div:has(> button[id="u1"])' in css and 'div:has(> button[id="u2"])' in css


def test_a_button_group_is_found_by_its_label_coloured_and_kept_on_one_row():
    """Its buttons by the label's `for`, one of them by its place from 0; a colour overrides viser's blue."""
    group = 'div:has(> div > p > label[for="g1"])'
    assert group_buttons("g1") == f"{group} button" and group_buttons("g1", 2) == f"{group} button:nth-child(3)"
    assert button_group_color_css("g1", "#f08c00") == (f"{group} button {{ color: #f08c00 !important; "
                                                       "border-color: #f08c00 !important; }")
    css = one_row_css("g1")
    assert f"{group} > div:last-child {{ min-width: 0; }}" in css and "flex-wrap: nowrap !important" in css


def test_a_button_groups_tooltips_hang_off_its_row_one_per_hovered_button():
    """Each tip is the row's `::after` while its button is hovered, as a CSS string with its line breaks; None: none."""
    group = 'div:has(> div > p > label[for="g1"])'
    css = group_tooltips_css("g1", ["J_M3\nTravel to the bar", None, 'say "hi"'])
    assert css.startswith(f"{group} {{ position: relative; }}{group}::after {{ display: none; position: absolute;")
    assert (f'{group}:has(button:nth-child(1):hover)::after {{ display: block; content: "J_M3\\A Travel to the bar"; }}'
            in css)
    assert "nth-child(2)" not in css
    assert css.endswith('nth-child(3):hover)::after { display: block; content: "say \\"hi\\""; }')
    assert css_string("a\\b") == '"a\\\\b"'
