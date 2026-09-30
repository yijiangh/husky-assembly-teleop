"""
Example 6: live plots with viser's uPlot widget.

Two scrolling plots of the last few seconds, one sample per tick:

  signal   a sine wave, frequency set by a slider, plus a little noise
  tick     the measured time between ticks, to see how steady the loop runs

  1. setup    `gui.add_uplot` once, with empty data and the series styling
  2. update   append one sample per series to a fixed-length history
  3. draw     assign `handle.data`; viser sends the new arrays to the browser

! `data` is a tuple of 1-D arrays of equal length: x first, then one per series,
  matching `series` in order. Assign a new tuple; editing the arrays in place
  is not sent.

Run with:  -p plugins:="['example_plot']"
"""

from __future__ import annotations

from collections import deque

import numpy as np
import viser
import viser.uplot

from ...context import PluginContext
from ...plugin import HuskyPlugin, register
from ...ui_style import SECTION_SENSOR, section

#: How much history each plot shows, in samples (ticks).
HISTORY = 200


@register
class ExamplePlotPlugin(HuskyPlugin):
    """A sine wave and the tick period, plotted live."""

    name = "example_plot"

    def __init__(self):
        """Create empty histories."""
        self._t: deque[float] = deque(maxlen=HISTORY)
        self._signal: deque[float] = deque(maxlen=HISTORY)
        self._tick_ms: deque[float] = deque(maxlen=HISTORY)
        self._start = 0.0
        self._last = 0.0
        self._phase = 0.0

    def setup(self, ctx: PluginContext) -> None:
        """Build the slider and both plots.

        Args:
            ctx: This plugin's context.
        """
        self._start = self._last = ctx.now()
        empty = (np.zeros(0), np.zeros(0))
        with ctx.view.ui() as gui:
            gui.add_html(section("signal", SECTION_SENSOR))
            self._frequency = gui.add_slider("Frequency Hz", min=0.1, max=3.0, step=0.1, initial_value=0.5)
            self._signal_plot: viser.GuiUplotHandle = gui.add_uplot(
                data=empty,
                series=(viser.uplot.Series(label="t s"),
                        viser.uplot.Series(label="sin", stroke="#1098ad", width=2)),
                scales={"x": viser.uplot.Scale(time=False), "y": viser.uplot.Scale(range=(-1.5, 1.5))},
                aspect=2.0,
            )

            gui.add_html(section("tick period", SECTION_SENSOR, f"expected {ctx.config.tick_period * 1e3:.0f} ms"))
            self._tick_plot: viser.GuiUplotHandle = gui.add_uplot(
                data=empty,
                series=(viser.uplot.Series(label="t s"),
                        viser.uplot.Series(label="ms", stroke="#f08c00", width=1)),
                scales={"x": viser.uplot.Scale(time=False), "y": viser.uplot.Scale(auto=True)},
                aspect=2.0,
            )

    def update(self, ctx: PluginContext) -> None:
        """Take one sample per series.

        Args:
            ctx: This plugin's context.
        """
        now = ctx.now()
        dt = now - self._last
        self._last = now
        # * Integrate the phase, so moving the slider does not make the wave jump.
        self._phase += 2 * np.pi * float(self._frequency.value) * dt

        self._t.append(now - self._start)
        self._signal.append(np.sin(self._phase) + np.random.normal(scale=0.05))
        self._tick_ms.append(dt * 1e3)

    def draw(self, ctx: PluginContext) -> None:
        """Send the histories to the plots.

        Args:
            ctx: This plugin's context.
        """
        t = np.array(self._t)
        self._signal_plot.data = (t, np.array(self._signal))
        self._tick_plot.data = (t, np.array(self._tick_ms))
