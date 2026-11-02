"""Tests for the phone page: pinned ports, a plugin's tab and header, Stop all, and its button styles."""

from __future__ import annotations

import socket
import threading

import pytest

from husky_assembly_teleop.ui.phone import (SHEET, big_buttons, button_group_color_css, css_string, group_buttons,
                                            group_tooltips_css, one_row_css,
                                            side_by_side_css, slim_labels_css)
from husky_assembly_teleop.ui.visualization import Visualization


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
