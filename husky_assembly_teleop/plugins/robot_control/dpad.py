"""The base's D-pad: four hold buttons that drive while pressed, and a speed slider."""

from __future__ import annotations

from typing import Callable

import viser

from ...plugin_api.context import PluginContext

#: Base speed at a Speed of 1. Kept low: this is a test panel.
MAX_LINEAR_SPEED = 0.3   # m/s
MAX_ANGULAR_SPEED = 0.5  # rad/s
#: Hold buttons (Forward, Left, Back, Right): label, icon, sign of linear and angular speed.
HOLD_BUTTONS = (("", viser.Icon.ARROW_UP, 1.0, 0.0),
                ("", viser.Icon.ROTATE, 0.0, 1.0),
                ("", viser.Icon.ARROW_DOWN, -1.0, 0.0),
                ("", viser.Icon.ROTATE_CLOCKWISE, 0.0, -1.0))
#: Each hold button's D-pad cell, (row, column), in HOLD_BUTTONS order.
DPAD_CELLS = ((1, 2), (2, 1), (2, 2), (2, 3))


def _dpad_css() -> str:
    """CSS that lays out the hold buttons as a D-pad in every folder holding all four of them.

    ? viser has no grid container; buttons are found by their icon class (`icon-tabler-<name>`).

    Returns:
        str: A <style> element, for an html widget outside the Drive folder (inside, it takes a grid cell).
    """
    def button(icon: str) -> str:
        return f"button .icon-tabler-{icon}"

    folder = "div" + "".join(f":has(> div > {button(icon)})" for _, icon, _, _ in HOLD_BUTTONS)
    grid = f"{folder} {{ display: grid; grid-template-columns: repeat(3, 1fr); }}"
    # ! Give every button an explicit cell, or Left fills the empty cell beside Forward.
    cells = "".join(f"div:has(> {button(icon)}) {{ grid-area: {row} / {column}; }}"
                    for (_, icon, _, _), (row, column) in zip(HOLD_BUTTONS, DPAD_CELLS))
    return f"<style>{grid}{cells}</style>"


DPAD_STYLE = _dpad_css()
#: How often the browser reports a held button, Hz. Matches the 20 Hz tick.
HOLD_CALLBACK_HZ = 20.0
#: A hold counts as released after this long without a report, seconds.
#: ! viser never reports a release: this timeout is what stops the base, also on disconnect.
HOLD_TIMEOUT = 0.25


def build_velocity_inputs(ctx: PluginContext, gui: viser.GuiApi, serial: str,
                          on_hold: Callable[[float, float], None]) -> viser.GuiSliderHandle:
    """Add the inputs for platform_velocity_controller: a speed slider and the D-pad.

    Args:
        ctx: The plugin's context.
        gui: The GUI api, inside the base's tab.
        serial: The robot's serial, for intent names.
        on_hold: Called every report of a held button, with its signs of linear and angular speed.

    Returns:
        viser.GuiSliderHandle: The speed slider, a fraction of the full speed.
    """
    speed = gui.add_slider("Speed", min=0.05, max=1.0, step=0.05, initial_value=0.5,
                           hint=f"Fraction of the full speed ({MAX_LINEAR_SPEED} m/s, {MAX_ANGULAR_SPEED} rad/s).")
    grid = gui.add_html("")
    with gui.add_folder("Drive"):
        buttons = [gui.add_button(label, icon=icon, hint="Drives while held.") for label, icon, _, _ in HOLD_BUTTONS]
    grid.content = DPAD_STYLE
    for button, (label, _, linear, angular) in zip(buttons, HOLD_BUTTONS):
        # ! Bind loop values as defaults, or every button drives like the last.
        button.on_hold(ctx.defer(f"hold {label} {serial}",
                                 lambda linear=linear, angular=angular: on_hold(linear, angular)),
                       callback_hz=HOLD_CALLBACK_HZ)
    return speed
