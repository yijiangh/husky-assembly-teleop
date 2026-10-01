"""
Example 1: widgets, buttons and intents.

A counter showing the three steps of every panel:

  1. setup    build the widgets once and keep the handles on `self`
  2. click    viser calls back on its own thread; `ctx.defer` / `ctx.defer_value`
              turn that into an intent run on the ROS thread
  3. draw     copy the plugin's state into the widgets, once per tick

! Keep state on `self` (here `self.count`); widgets only display it. Operator
  settings (step size, "Below 0") may be read from widgets.

Run with:  -p plugins:="['example_ui']"
"""

from __future__ import annotations

import viser

from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...ui.style import NONE, OK, SECTION_CTRL, block, chip, section, values


@register
class ExampleUiPlugin(HuskyPlugin):
    """A counter driven by buttons, a slider and a checkbox."""

    name = "example_ui"

    def __init__(self):
        """Create the plugin state."""
        self.count = 0
        self.presses = 0

    def setup(self, ctx: PluginContext) -> None:
        """Build the widgets once and route their callbacks through `ctx.defer` / `defer_value`.

        Args:
            ctx: This plugin's context.
        """
        with ctx.view.ui() as gui:
            self._status: viser.GuiHtmlHandle = gui.add_html("")

            gui.add_html(section("counter", SECTION_CTRL))
            # * One callback serves the whole button row; it gets the clicked label.
            self._buttons = gui.add_button_group("Count", ["−", "+", "Reset"])
            self._step = gui.add_slider("Step", min=1, max=10, step=1, initial_value=1)
            self._allow_negative = gui.add_checkbox("Below 0", initial_value=False)

        # ! Never do the work inside the callback: it runs on a viser thread, where plugin
        #   state and robots are not safe to touch. defer_value passes the new value; defer nothing.
        self._buttons.on_click(ctx.defer_value("count button", self._on_button))
        # ! A setting that limits the state must act when changed, or a negative count stays after unticking.
        self._allow_negative.on_update(ctx.defer("below zero", self._apply_limits))

    def _on_button(self, clicked: str) -> None:
        """Apply one button press (intent, on the ROS thread).

        Args:
            clicked: The label of the button that was pressed.
        """
        self.presses += 1
        if clicked == "Reset":
            self.count = 0
        else:
            step = int(self._step.value)
            self.count += step if clicked == "+" else -step
        self._apply_limits()

    def _apply_limits(self) -> None:
        """Keep the count within what the settings allow (intent)."""
        if not self._allow_negative.value:
            self.count = max(self.count, 0)

    def draw(self, ctx: PluginContext) -> None:
        """Copy the state into the widgets every tick (viser sends only real changes).

        Args:
            ctx: This plugin's context.
        """
        state = chip("zero", NONE) if self.count == 0 else chip("counting", OK)
        self._status.content = block(state + values(f"count   {self.count:+5d}",
                                                    f"presses {self.presses:5d}"))
