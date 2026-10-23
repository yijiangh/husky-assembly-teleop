"""Tests for the side panels: plugins' tabs at their tab path, made in order on first use, removed with their plugin."""

from __future__ import annotations

import warnings

import pytest
import viser
from viser._icons import svg_from_icon

from husky_assembly_teleop.ui.style import PATH_TABS_MARKER
from husky_assembly_teleop.ui.visualization import Visualization


@pytest.fixture
def viz():
    """Both servers on free ports."""
    viz = Visualization(port=0, phone_port=0)
    yield viz
    viz.stop()


def test_plugins_share_parent_tabs_and_clear_only_their_own(viz):
    """Control, then Plan with Arm and Base inside; clearing the arm panel leaves Plan and Base."""
    control = viz.view_for("robot_control", tab_path=("Control",), icon=viser.Icon.DEVICE_GAMEPAD_2)
    arm = viz.view_for("arm_panel", tab_path=("Plan", "Arm"), icon=viser.Icon.ROBOT)
    base = viz.view_for("base_panel", tab_path=("Plan", "Base"), icon=viser.Icon.CAR)
    for view in (control, arm, base, arm):
        with view.ui() as gui:
            gui.add_button("Go")
    side = viz._side
    assert side._panel._tab_labels == ("Control", "Plan")
    assert side._groups[("Plan",)]._tab_labels == ("Arm", "Base")
    arm.clear()
    assert side._panel._tab_labels == ("Control", "Plan")
    assert side._groups[("Plan",)]._tab_labels == ("Base",)
    control.clear()
    assert side._panel._tab_labels == ("Plan",)


def test_own_tabs_carry_the_plugin_icon_and_shared_parents_none(viz):
    """The plugin's tab and the tab named after it carry its icon; Plan carries none; a nested tab picks its own."""
    arm = viz.view_for("arm_panel", tab_path=("Plan", "Arm"), icon=viser.Icon.ROBOT)
    probe = viz.view_for("mocap_probe", hidden=True, tab_path=("Mocap probe", "Points"), icon=viser.Icon.CROSSHAIR)
    with arm.ui():
        pass
    with probe.ui():
        pass
    with probe.tab("Status", viser.Icon.EYE, hidden=True):
        pass
    plan_icon, = viz._side._panel._tab_icons_html
    probe_icon, = viz._hidden_side._panel._tab_icons_html
    arm_icon, = viz._side._groups[("Plan",)]._tab_icons_html
    assert plan_icon is None and arm_icon and probe_icon
    assert viz._hidden_side._groups[("Mocap probe",)]._tab_labels == ("Points", "Status")


def marked(tab: viser.GuiTabHandle) -> bool:
    """Whether the tab holds the path marker."""
    return any(getattr(child, "content", None) == PATH_TABS_MARKER for child in tab._children.values())


def test_only_tabs_shared_by_plugins_mark_their_tab_bar_as_a_path(viz):
    """Plan holds Arm and Base of two plugins, so its bar is a path; the probe's own tab holds its sections."""
    for name, path in (("arm_panel", ("Plan", "Arm")), ("base_panel", ("Plan", "Base"))):
        with viz.view_for(name, tab_path=path).ui():
            pass
    probe = viz.view_for("mocap_probe", tab_path=("Mocap probe", "Points"))
    with probe.ui():
        pass
    with probe.tab("Status"):
        pass
    assert marked(viz._side._tabs[("Plan",)]) and not marked(viz._side._tabs[("Mocap probe",)])


def test_extra_tabs_nest_under_the_plugin_name_in_the_minimized_panel(viz):
    """Hidden tabs go in the second panel under a tab named after the plugin; it shows only while it holds a tab."""
    obstacles, calibration = viz.view_for("obstacles", hidden=True), viz.view_for("calibration")
    with obstacles.ui() as gui:
        gui.add_button("Add")
    with calibration.ui() as gui:
        gui.add_button("Start")
    with calibration.tab("Live", hidden=True) as gui:
        gui.add_button("Plot")
    with calibration.tab("Results", hidden=True) as gui:
        gui.add_button("Figure")
    side, hidden = viz._side, viz._hidden_side
    assert side._panel._tab_labels == ("Calibration",)
    assert hidden._panel._tab_labels == ("Obstacles", "Calibration") and hidden._panel.visible
    assert hidden._groups[("Calibration",)]._tab_labels == ("Live", "Results")
    obstacles.clear()
    calibration.clear()
    assert hidden._panel._tab_labels == () and not hidden._panel.visible


def test_a_folder_goes_in_the_main_panel_not_the_tabs(viz):
    """`folder()` makes no side panel; clearing the plugin removes the folder."""
    health = viz.view_for("health")
    with health.folder() as gui:
        gui.add_html("row")
    assert not viz._side._panel.visible and health._folder is not None
    health.clear()
    assert health._folder._impl.removed


@pytest.mark.parametrize("maker", ["assembly_run", "assembly_steps"])
def test_plugins_share_a_tab_through_bands_and_clear_only_their_own(viz, maker):
    """Two plugins at tab path ("Assembly",) get their bands in band order, whichever starts first; its maker's icon.

    Clearing the other plugin removes only its bands; the maker cleared first takes a third plugin's bands with it,
    and clearing that one then warns of nothing.
    """
    icons = {"assembly_run": viser.Icon.LIST_CHECK, "assembly_steps": viser.Icon.STAIRS_UP}
    bands = {"assembly_run": 400.0, "assembly_steps": 100.0}
    views = {name: viz.view_for(name, tab_path=("Assembly",), icon=icon) for name, icon in icons.items()}
    other = next(name for name in views if name != maker)
    for name in (maker, other, "assembly_steps"):
        with views[name].ui(band=bands[name]) as gui:
            gui.add_button(name)
    side = viz._side
    tab = side._tabs[("Assembly",)]
    assert sorted((child.order, len(child._children)) for child in tab._children.values()) == [(100.0, 2), (400.0, 1)]
    assert side._panel._tab_labels == ("Assembly",) and side._panel._tab_icons_html == (svg_from_icon(icons[maker]),)
    (band,) = views[other]._bands.values()
    views[other].clear()
    assert side._panel._tab_labels == ("Assembly",) and band._impl.removed
    assert [child.order for child in tab._children.values()] == [bands[maker]]
    exec_ = viz.view_for("movement_exec", tab_path=("Assembly",), icon=viser.Icon.PLAYER_PLAY)
    with exec_.ui(band=300.0) as gui:
        gui.add_button("Confirm")
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        views[maker].clear()
        exec_.clear()
    assert side._panel._tab_labels == ()
