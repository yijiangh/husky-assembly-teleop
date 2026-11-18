"""
A row of buttons choosing the robot a panel works on: icon and short id, as Control's tabs, the chosen one highlighted.

For a panel with one set of widgets for all robots: viser's tabs do not tell a plugin which one is open, so Control
builds its widgets once per robot, and this gives the others the same look.
"""

from __future__ import annotations

from typing import Sequence

import viser

from ..colors import CURRENT, NONE, rgb
from ..config import short_serial
from .css import side_by_side_css


class RobotButtons:
    """One button per robot, on one row; always the same height, whichever is chosen.

    Attributes:
        buttons: Serial -> its button; connect each one's `on_click`.
    """

    def __init__(self, gui: viser.GuiApi, serials: Sequence[str], chosen: str):
        """Add the row where the panel's widgets are being added.

        Args:
            gui: The GUI api, with the panel as parent.
            serials: The robots, left to right.
            chosen: The robot highlighted at start.
        """
        # * Draws nothing: the buttons side by side, once they exist.
        row = gui.add_html("")
        self.buttons = {serial: gui.add_button(short_serial(serial), icon=viser.Icon.ROBOT, color=rgb(NONE))
                        for serial in serials}
        row.content = f"<style>{side_by_side_css([button._impl.uuid for button in self.buttons.values()])}</style>"
        self.show(chosen)

    def show(self, chosen: str) -> None:
        """Highlight this robot; call it on every draw, as the panel may switch robot by itself.

        Args:
            chosen: The robot's serial.
        """
        for serial, button in self.buttons.items():
            button.color = rgb(CURRENT if serial == chosen else NONE)
