"""
A live uPlot of one signal of a trace: one line per channel, x in seconds since the trace started.

Build it in `setup` inside `ctx.view.ui()`, call `draw` from the plugin's `draw`.

? Only the last `seconds` and at most MAX_POINTS points are sent: viser resends the whole plot on each change.
"""

from __future__ import annotations

import numpy as np
import viser
import viser.uplot

from ..plugin_api.trace import Trace

#: Line colour per channel, in order; x, y, z are red, green, blue as in the 3D view.
COLORS = ("#e03131", "#2f9e44", "#4263eb", "#f08c00", "#ae3ec9", "#1098ad", "#868e96")

#: Most points sent per line; longer spans are thinned evenly.
MAX_POINTS = 1000


class TracePlot:
    """One signal of a trace, plotted; redrawn only when the trace has new samples."""

    def __init__(self, gui: viser.GuiApi, trace: Trace, name: str, seconds: float | None = 30.0,
                 y_range: tuple[float, float] | None = None, aspect: float = 2.0):
        """Add the plot to `gui`.

        Args:
            gui: Where to add it, e.g. from `with ctx.view.ui() as gui`.
            trace: The trace to show.
            name: Which of its signals.
            seconds: How far back to show; None shows everything kept.
            y_range: Fixed y axis, or None to fit the data.
            aspect: Width over height.
        """
        self._trace = trace
        self._name = name
        self._seconds = seconds
        signal = trace.signal(name)
        unit = f" [{signal.unit}]" if signal.unit else ""
        series = (viser.uplot.Series(label="t s"),
                  *(viser.uplot.Series(label=label, stroke=COLORS[i % len(COLORS)], width=1.5)
                    for i, label in enumerate(signal.channels)))
        y_scale = viser.uplot.Scale(range=y_range) if y_range else viser.uplot.Scale(auto=True)
        self.handle: viser.GuiUplotHandle = gui.add_uplot(
            data=tuple(np.zeros(0) for _ in series), series=series, title=f"{name}{unit}",
            scales={"x": viser.uplot.Scale(time=False), "y": y_scale}, aspect=aspect)
        # (count, newest time) last sent, to skip unchanged redraws.
        self._shown: tuple[int, float | None] = (0, None)

    def draw(self) -> None:
        """Send the trace's newest samples to the browser, if it changed."""
        t = self._trace.times()
        shown = (len(t), float(t[-1]) if len(t) else None)
        if shown == self._shown:
            return
        self._shown = shown
        values = self._trace.values(self._name)
        if self._seconds is not None and len(t):
            keep = t >= t[-1] - self._seconds
            t, values = t[keep], values[keep]
        if len(t) > MAX_POINTS:
            pick = np.linspace(0, len(t) - 1, MAX_POINTS).astype(int)
            t, values = t[pick], values[pick]
        start = self._trace.start or 0.0
        self.handle.data = (t - start, *values.T)
