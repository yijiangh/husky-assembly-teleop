"""
Example 1: widgets, buttons and intents.

A counter, and nothing else, so the three steps every panel goes through are
easy to follow:

  1. setup    build the widgets once and keep the handles on `self`
  2. click    viser calls back on *its* thread; `ctx.defer` or
              `ctx.defer_value` turns that into an intent, which runs on the
              ROS thread at this plugin's next step
  3. draw     copy the plugin's state into the widgets, once per tick

! The plugin's state is `self.count`, not a widget's value. Widgets only show
  it. That way the state has one owner, and anything else in the plugin can
  read or change it without touching the UI. Settings the operator picks (the
  step size, "Below 0") are fine to read straight from their widget.

* No need to check whether a value changed before assigning it. viser only
  sends real changes to the browser, so `draw` can simply assign every tick.

Run with:  -p plugins:="['example_ui']"
"""

from __future__ import annotations

import viser

from ...context import PluginContext
from ...plugin import HuskyPlugin, register
from ...ui_style import NONE, OK, SECTION_CTRL, block, chip, section, values


@register
class ExampleUiPlugin(HuskyPlugin):
    """A counter driven by buttons, a slider and a checkbox."""

    name = "example_ui"

    def __init__(self):
        """Plugin state lives here, before any widget exists."""
        self.count = 0
        self.presses = 0

    def setup(self, ctx: PluginContext) -> None:
        """Build every widget once and wire each callback through `ctx.defer`/`defer_value`.

        Args:
            ctx: This plugin's context.
        """
        with ctx.view.ui() as gui:
            self._status: viser.GuiHtmlHandle = gui.add_html("")

            gui.add_html(section("counter", SECTION_CTRL))
            # * A button group is a row of small buttons. One callback serves
            #   the whole row; defer_value hands it the label that was clicked.
            self._buttons = gui.add_button_group("Count", ["−", "+", "Reset"])
            self._step = gui.add_slider("Step", min=1, max=10, step=1, initial_value=1)
            self._allow_negative = gui.add_checkbox("Below 0", initial_value=False)

        # ! Never do the work inside the callback. It runs on a viser thread,
        #   where the plugin's state, the robots and PyBullet are not safe to
        #   touch. defer queues it, and the monitor runs it on the ROS thread.
        #
        #   defer_value     the work needs the widget's new value (which
        #                   button of a group, a slider's position)
        #   defer           it does not (a plain button, "something changed")
        self._buttons.on_click(ctx.defer_value("count button", self._on_button))
        # ! A setting that constrains the state must act when it changes, not
        #   only on the next button press -- otherwise unticking "Below 0" would
        #   leave a negative count standing until someone clicks.
        self._allow_negative.on_update(ctx.defer("below zero", self._apply_limits))

    def _on_button(self, clicked: str) -> None:
        """Apply one button press. Runs as an intent, on the ROS thread.

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
        """Keep the count within what the settings allow. Runs as an intent."""
        if not self._allow_negative.value:
            self.count = max(self.count, 0)

    def draw(self, ctx: PluginContext) -> None:
        """Show the state. Runs every tick; viser only sends what changed.

        Args:
            ctx: This plugin's context.
        """
        state = chip("zero", NONE) if self.count == 0 else chip("counting", OK)
        self._status.content = block(state + values(f"count   {self.count:+5d}",
                                                    f"presses {self.presses:5d}"))
