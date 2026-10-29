"""The base's D-pad: four hold buttons that drive while pressed, and a speed slider."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import viser

from ...plugin_api.context import PluginContext
from ...ui.phone import big_buttons, grid_css

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

_ICONS = [icon for _, icon, _, _ in HOLD_BUTTONS]
_GRID = tuple((icon, row, column) for icon, (row, column) in zip(_ICONS, DPAD_CELLS))
#: Lays out the hold buttons as a D-pad, for an html widget outside the Drive folder (inside, it takes a grid cell).
DPAD_STYLE = f"<style>{grid_css(_GRID, 3)}</style>"
#: Add to DPAD_STYLE on the phone page: big, icon-only buttons.
PHONE_DPAD_STYLE = f"<style>{big_buttons(_ICONS, 84, 40, labels=False)}</style>"
#: How often the browser reports a held button, Hz. Matches the 20 Hz tick.
HOLD_CALLBACK_HZ = 20.0
#: D-pad colours: gray while Clearance would refuse a drive (too close already), orange while Clearance is off.
#: ! Only a colour: the buttons stay pressable, and a refused press says why.
BLOCKED_COLOR, UNGUARDED_COLOR = "gray", "orange"
#: The D-pad buttons' hover text.
DPAD_HINT = "Drives while held. Gray: too close to start with Clearance on. Orange: Clearance is off."
#: A hold counts as released after this long without a report, seconds.
#: ! viser never reports a release: this timeout is what stops the base, also on disconnect.
HOLD_TIMEOUT = 0.25


@dataclass(frozen=True)
class Hold:
    """A D-pad button held: its signs of linear and angular speed, its D-pad's speed slider, and its last report.

    Attributes:
        linear: Sign of the linear speed asked for.
        angular: Sign of the angular speed asked for.
        speed: The speed slider of the D-pad it is on.
        time: When the browser last reported it held, `ctx.now()`.
    """

    linear: float
    angular: float
    speed: viser.GuiSliderHandle
    time: float

    def released(self, now: float) -> bool:
        """Whether no report came for HOLD_TIMEOUT: the button counts as released."""
        return now - self.time > HOLD_TIMEOUT

    def twist(self) -> tuple[float, float]:
        """The twist it asks for: linear (m/s) and angular (rad/s) speed."""
        return self.linear * self.speed.value * MAX_LINEAR_SPEED, self.angular * self.speed.value * MAX_ANGULAR_SPEED


def build_velocity_inputs(ctx: PluginContext, gui: viser.GuiApi, serial: str, on_hold: Callable[[Hold], None]
                          ) -> tuple[viser.GuiSliderHandle, list[viser.GuiButtonHandle]]:
    """Add the inputs for platform_velocity_controller: a speed slider and the D-pad.

    Args:
        ctx: The plugin's context.
        gui: The GUI api, inside the base's tab.
        serial: The robot's serial, for intent names.
        on_hold: Called with a `Hold` every report of a held button.

    Returns:
        tuple[viser.GuiSliderHandle, list[viser.GuiButtonHandle]]: The speed slider, a fraction of the full speed,
            and the D-pad's buttons (`color_dpad`).
    """
    speed = gui.add_slider("Speed", min=0.05, max=1.0, step=0.05, initial_value=0.5,
                           hint=f"Fraction of the full speed ({MAX_LINEAR_SPEED} m/s, {MAX_ANGULAR_SPEED} rad/s).")
    grid = gui.add_html("")
    with gui.add_folder("Drive"):
        buttons = [gui.add_button(label, icon=icon, hint=DPAD_HINT) for label, icon, _, _ in HOLD_BUTTONS]
    grid.content = DPAD_STYLE
    for button, (label, _, linear, angular) in zip(buttons, HOLD_BUTTONS):
        # ! Bind loop values as defaults, or every button drives like the last.
        button.on_hold(ctx.defer(f"hold {label} {serial}",
                                 lambda linear=linear, angular=angular: on_hold(
                                     Hold(linear, angular, speed, ctx.now()))),
                       callback_hz=HOLD_CALLBACK_HZ)
    return speed, buttons


def color_dpad(buttons: list[viser.GuiButtonHandle], color: str | None) -> None:
    """Colour the D-pad's buttons, e.g. BLOCKED_COLOR; None for the theme's own. Sends only a change."""
    for button in buttons:
        if button.color != color:
            button.color = color
