"""Clearance below every D-pad and tool pad: one setting, ticked at start, whichever box the operator ticks."""

from types import SimpleNamespace

import pytest
from plugin_fakes import FakeGui

from husky_assembly_teleop.shared.teleop.clearance_toggle import ClearanceToggle
from husky_assembly_teleop.shared.teleop.dpad import build_velocity_inputs
from husky_assembly_teleop.shared.teleop.tool_pad import build_tool_step_inputs

pytestmark = pytest.mark.safety

#: A context whose callbacks run at once.
CTX = SimpleNamespace(defer=lambda _label, run, operator_only=False: lambda *_event: run(),
                      defer_value=lambda _label, run, operator_only=False: run, now=lambda: 0.0)


@pytest.mark.parametrize("build", [
    lambda gui, clearance: build_velocity_inputs(CTX, gui, "0805", lambda _hold: None, clearance),
    lambda gui, clearance: build_tool_step_inputs(CTX, gui, "0805 ur_arm", lambda _step: None, clearance),
], ids=["dpad", "tool pad"])
def test_every_pad_has_a_ticked_clearance_box_below_its_buttons(build):
    """No D-pad or tool pad without its Clearance: the box comes after the last button, ticked."""
    gui, clearance = FakeGui(), ClearanceToggle("hint")
    build(gui, clearance)
    kinds = [widget.kind for widget in gui.widgets]
    box = kinds.index("checkbox")
    assert kinds.count("checkbox") == 1 and box > max(i for i, kind in enumerate(kinds) if kind == "button")
    assert gui.widgets[box].label == "Clearance" and gui.widgets[box].value is True and clearance.value is True


def test_a_tick_on_any_box_sets_the_one_setting_and_every_box():
    """The laptop's and the phone's D-pad show one Clearance: unticking either unticks both, and back."""
    laptop, phone, clearance = FakeGui(), FakeGui(), ClearanceToggle("hint")
    for gui in (laptop, phone):
        build_velocity_inputs(CTX, gui, "0805", lambda _hold: None, clearance)
    (first,), (second,) = laptop.of_kind("checkbox"), phone.of_kind("checkbox")
    second.value = False
    second.callback(False)
    assert clearance.value is False and first.value is False
    clearance.value = True
    assert first.value is True and second.value is True
