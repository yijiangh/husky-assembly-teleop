"""
The tool pad: one click shifts an arm's tool0 along, or turns it about, one axis of the chosen frame by the step.

The frame is the robot's (x forward, y left, z up: the same whatever the flange's turn) or tool0's own. A row per
axis, coloured as the scene draws axes (x red, y green, z blue): shift −/+, then turn −/+; Clearance below.
"""

from __future__ import annotations

from typing import Callable

import numpy as np
import viser
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose

from ...colors import AXIS_X, AXIS_Y, AXIS_Z, rgb
from ...plugin_api.context import PluginContext
from ...ui.phone import columns_css
from .choice_row import ChoiceRow
from .clearance_toggle import ClearanceToggle

#: Step sizes, fine to coarse: label -> (shift in metres, turn in degrees). Labels short: four fit one phone row.
STEP_SIZES = {"0.1 mm": (1e-4, 0.2), "0.5 mm": (5e-4, 1.0), "2 mm": (2e-3, 2.0), "5 mm": (5e-3, 5.0)}
#: The frames a step is given in: the robot's (its URDF root, from the joints only) or tool0's.
ROBOT_FRAME, TOOL_FRAME = "Robot", "Tool"
FRAMES = (ROBOT_FRAME, TOOL_FRAME)
#: Each frame's axes in words, for the hints.
AXIS_WORDS = {ROBOT_FRAME: {"X": "the robot's x (forward)", "Y": "the robot's y (left)", "Z": "the robot's z (up)"},
              TOOL_FRAME: {"X": "tool0's x", "Y": "tool0's y", "Z": "tool0's z (out of the flange)"}}
#: The buttons: a sign and one of the frame's axes.
SHIFTS = ("−X", "+X", "−Y", "+Y", "−Z", "+Z")
TURNS = ("−RX", "+RX", "−RY", "+RY", "−RZ", "+RZ")
#: Each axis' colour, as the scene draws tool0's axes.
AXIS_COLORS = {"X": rgb(AXIS_X), "Y": rgb(AXIS_Y), "Z": rgb(AXIS_Z)}
#: Icons: a shift's direction; a turn's sense by the right-hand rule (+ counterclockwise, looking down the axis).
#: ! Used by no other grid (`ui.phone.grid_css` places buttons by icon); ARROW_NARROW_RIGHT marks the pad's folder.
ICONS = {"−": viser.Icon.ARROW_NARROW_LEFT, "+": viser.Icon.ARROW_NARROW_RIGHT,
         "−R": viser.Icon.ROTATE_CLOCKWISE_2, "+R": viser.Icon.ROTATE_2}
_PAD = f"div:has(> div > button .icon-tabler-{viser.Icon.ARROW_NARROW_RIGHT})"
#: Lays the pad out as a row per axis (shift −, shift +, turn −, turn +) under a header across all four columns.
#: ? viser gives every widget side margins; in four narrow columns they would leave a button room for its icon only.
PAD_STYLE = (f"<style>{columns_css(viser.Icon.ARROW_NARROW_RIGHT, 4)}"
             f"{_PAD} {{ column-gap: 4px; padding-inline: 6px; }}"
             f"{_PAD} > div {{ margin-inline: 0 !important; }}"
             f"{_PAD} > div:has(.tool-pad-header) {{ grid-column: 1 / -1; }}"
             f"{_PAD} button {{ width: 100%; padding: 0 2px; }}</style>")
#: The pad's first row: which columns shift and which turn.
PAD_HEADER = ('<div class="tool-pad-header" style="display:grid;grid-template-columns:1fr 1fr;text-align:center;'
              'font-size:0.85em">'
              '<span>shift</span><span>turn</span></div>')


def tool_step(button: str, size: str) -> Pose:
    """The move one click of a pad button asks for, in the pad's frame.

    Args:
        button: One of SHIFTS or TURNS.
        size: One of STEP_SIZES.
    """
    shift, turn = STEP_SIZES[size]
    axis = np.eye(3)["XYZ".index(button[-1])] * (1.0 if button[0] == "+" else -1.0)
    if button in SHIFTS:
        return Pose(tuple(axis * shift))
    return Pose(orientation=tuple(Rotation.from_rotvec(axis * np.radians(turn)).as_quat()))


def build_tool_step_inputs(ctx: PluginContext, gui: viser.GuiApi, label: str, on_step: Callable[[Pose, str], None],
                           clearance: ClearanceToggle, phone: bool = False) -> viser.GuiDropdownHandle | ChoiceRow:
    """Add the step size, the frame, the pad (a row per axis) and a box of `clearance`; a click calls `on_step`.

    Args:
        ctx: The plugin's context.
        gui: The GUI api.
        label: What the pad moves, for intent names.
        on_step: Called with each click's `tool_step` and the frame it is in (one of FRAMES).
        clearance: Whether the steps keep their clearance.
        phone: For the phone page: the step size as a row of buttons, where a dropdown opens the keyboard.

    Returns:
        viser.GuiDropdownHandle | ChoiceRow: The step size, its `value` a key of STEP_SIZES.
    """
    turns = ", ".join(f"{turn:g}°" for _shift, turn in STEP_SIZES.values())
    hint = f"How far one click shifts tool0; it turns {turns} with them."
    size = (ChoiceRow(ctx, gui, "Step size", tuple(STEP_SIZES), "0.5 mm", hint) if phone
            else gui.add_dropdown("Step size", tuple(STEP_SIZES), initial_value="0.5 mm", hint=hint))
    frame = ChoiceRow(ctx, gui, "Frame", FRAMES, ROBOT_FRAME,
                      "Robot: x forward, y left, z up, whatever the flange's turn. Tool: tool0's own axes.")
    gui.add_html(PAD_STYLE)
    with gui.add_folder("Tool pad"):
        gui.add_html(PAD_HEADER)
        for axis in "XYZ":
            for button in (f"−{axis}", f"+{axis}", f"−R{axis}", f"+R{axis}"):
                turn = button in TURNS
                words = " or ".join(f"{name}: {AXIS_WORDS[name][axis]}" for name in FRAMES)
                hint = (f"Turn {'+' if button[0] == '+' else '−'} about the frame's {axis.lower()} axis through the "
                        f"tip ({words})" if turn else f"Shift {button[0]} along the frame's {axis.lower()} axis "
                        f"({words})")
                handle = gui.add_button(button, color=AXIS_COLORS[axis], icon=ICONS[button[:2 if turn else 1]],
                                        hint=f"{hint}, by the step size.")
                # ! Bind the loop value as a default, or every button moves like the last.
                handle.on_click(ctx.defer(f"tool step {button} {label}",
                                          lambda button=button: on_step(tool_step(button, size.value), frame.value)))
    clearance.add(ctx, gui, label)
    return size
