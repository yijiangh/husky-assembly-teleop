"""Tests for the side panel: plugins' tabs found by path, made in order on first use, removed with their plugin."""

from __future__ import annotations

import pytest
import viser

from husky_assembly_teleop.ui.visualization import Visualization

CONTROL = ("Control", viser.Icon.DEVICE_GAMEPAD_2)
PLAN = ("Plan", viser.Icon.ROUTE)


@pytest.fixture
def viz():
    """Both servers on free ports."""
    viz = Visualization(port=0, phone_port=0)
    yield viz
    viz.stop()


def test_plugins_share_parent_tabs_and_clear_only_their_own(viz):
    """Control, then Plan with Arm and Base inside; clearing the arm panel leaves Plan and Base."""
    control, arm, base = viz.view_for("robot_control"), viz.view_for("arm_panel"), viz.view_for("base_panel")
    with control.tab(CONTROL) as gui:
        gui.add_button("Drive")
    with arm.tab(PLAN, ("Arm", viser.Icon.ROBOT)) as gui:
        gui.add_button("Plan arm")
    with base.tab(PLAN, ("Base", viser.Icon.CAR)) as gui:
        gui.add_button("Plan base")
    with arm.tab(PLAN, ("Arm", viser.Icon.ROBOT)) as gui:
        gui.add_button("Again")
    side = viz._side
    assert side._panel._tab_labels == ("Control", "Plan")
    assert side._groups[("Plan",)]._tab_labels == ("Arm", "Base")
    arm.clear()
    assert side._panel._tab_labels == ("Control", "Plan")
    assert side._groups[("Plan",)]._tab_labels == ("Base",)
    control.clear()
    assert side._panel._tab_labels == ("Plan",)
