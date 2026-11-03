"""The tool pad: each button's move in its frame, at each step size; the robot's frame unless Tool is chosen."""

from types import SimpleNamespace

import numpy as np
import pytest
from plugin_fakes import FakeGui
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.shared.teleop.clearance_toggle import ClearanceToggle
from husky_assembly_teleop.shared.teleop.tool_pad import (ROBOT_FRAME, SHIFTS, STEP_SIZES, TOOL_FRAME, TURNS,
                                                          build_tool_step_inputs, tool_step)

pytestmark = pytest.mark.safety


@pytest.mark.parametrize("size", STEP_SIZES)
def test_each_shift_moves_along_its_axis_by_the_step_and_does_not_turn(size):
    """−X to +Z: one axis, signed, the size's distance."""
    for index, button in enumerate(SHIFTS):
        step = tool_step(button, size)
        expected = np.eye(3)[index // 2] * STEP_SIZES[size][0] * (1 if index % 2 else -1)
        np.testing.assert_allclose(step.position, expected)
        assert Rotation.from_quat(step.orientation).magnitude() == 0.0


@pytest.mark.parametrize("size", STEP_SIZES)
def test_each_turn_turns_about_its_axis_by_the_step_and_does_not_shift(size):
    """−RX to +RZ: one axis, signed, the size's angle."""
    for index, button in enumerate(TURNS):
        step = tool_step(button, size)
        expected = np.eye(3)[index // 2] * np.radians(STEP_SIZES[size][1]) * (1 if index % 2 else -1)
        np.testing.assert_allclose(Rotation.from_quat(step.orientation).as_rotvec(), expected, atol=1e-12)
        assert step.position == (0.0, 0.0, 0.0)


def test_on_the_phone_the_step_size_is_a_row_of_buttons_the_chosen_one_filled_and_used_by_the_pad():
    """No dropdown on the phone (it opens the keyboard): a click on a size fills it, and the pad then steps by it."""
    gui, steps = FakeGui(), []
    ctx = SimpleNamespace(defer=lambda _label, run, operator_only=False: lambda *_event: run(),
                          defer_value=lambda _label, run, operator_only=False: run)
    size = build_tool_step_inputs(ctx, gui, "0805 ur_arm", lambda step, frame: steps.append((step, frame)),
                                  ClearanceToggle("hint"), phone=True)
    assert not [widget for widget in gui.widgets if widget.kind == "dropdown"] and size.value == "0.5 mm"
    group, frame = [widget for widget in gui.widgets if widget.kind == "button_group"]
    style = size._style.content
    group.callback("5 mm")
    assert size.value == "5 mm" and size._style.content != style
    plus_x, = [widget for widget in gui.widgets if widget.label == "+X"]
    plus_x.callback()
    np.testing.assert_allclose(steps[-1][0].position, (5e-3, 0.0, 0.0))
    assert steps[-1][1] == ROBOT_FRAME, "the robot's frame at start"
    frame.callback(TOOL_FRAME)
    plus_x.callback()
    assert steps[-1][1] == TOOL_FRAME
