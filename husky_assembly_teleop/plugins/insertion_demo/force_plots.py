"""
The Force plots window: both arms' force and torque all along, over every move and pause, until Clear.

Read each tick from the arms' force sensors as they report (raw, or as zeroed by Zero F/T); the last KEEP_SAMPLES
ticks are kept. The plots thin a long record to what the browser is sent keeping each line's highs and lows, so a
short push still shows. Drawn twice a second: viser resends the whole plot on each change.
"""

from __future__ import annotations

import math
from functools import partial
from typing import TYPE_CHECKING

import numpy as np
import viser

from ...plugin_api.trace import Signal
from ...shared.execution.servo_record import FIRST_ARM_COLORS, SECOND_ARM_COLORS
from ...ui.style import numbers, values
from ...ui.trace_plot import TracePlot

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext

#: Ticks kept: about half an hour at 20 Hz; past it the oldest go.
KEEP_SAMPLES = 36000
#: Seconds between two redraws.
DRAW_EVERY = 0.5
#: The first arm's three axes, then the second's.
CHANNELS = tuple(f"{side} {axis}" for side in "LR" for axis in "xyz")
COLORS = FIRST_ARM_COLORS[:3] + SECOND_ARM_COLORS[:3]


def _both(ctx: PluginContext, serial: str, start: int) -> list[float]:
    """Three values of each arm's wrench from `start` (0 force, 3 torque), NaN for an arm without a reading."""
    robot = ctx.world.robots.get(serial) if serial else None
    arms = list(robot.arms.values()) if robot is not None else []
    row: list[float] = []
    for index in range(2):
        wrench = arms[index].state.wrench if index < len(arms) else None
        row += [math.nan] * 3 if wrench is None else [float(value) for value in wrench[start:start + 3]]
    return row


class ForcePlots:
    """Both arms' force and torque since the monitor started or the last Clear, with |F| now and its peak."""

    def __init__(self, ctx: PluginContext, gui: viser.GuiApi, serial: str):
        """Start recording `serial`'s two arms and build the plots, Clear and the readout inside `gui`."""
        self._trace = ctx.trace(Signal("force", partial(_both, ctx, serial, 0), CHANNELS, "N"),
                                Signal("torque", partial(_both, ctx, serial, 3), CHANNELS, "Nm"),
                                max_samples=KEEP_SAMPLES)
        self._plots = tuple(TracePlot(gui, self._trace, name, seconds=None, min_span=span, colors=COLORS, peaks=True)
                            for name, span in (("force", 1.0), ("torque", 0.1)))
        clear = gui.add_button("Clear", icon=viser.Icon.ERASER, hint="Start the record again from now")
        clear.on_click(ctx.defer("clear force plots", self._trace.clear))
        self._readout = gui.add_html(self._text(None))
        self._drawn = -math.inf

    def draw(self, now: float) -> None:
        """Send the new samples to the browser, at most every DRAW_EVERY seconds."""
        if now - self._drawn < DRAW_EVERY:
            return
        self._drawn = now
        for plot in self._plots:
            plot.draw()
        force = self._trace.values("force")
        self._readout.content = self._text(force if len(force) else None)

    @staticmethod
    def _text(force: np.ndarray | None) -> str:
        """|F| of each arm now and its largest since the record started; dashes before any reading."""
        if force is None:
            return values("|F| now  N " + numbers(None, 2, 7, 1), "|F| peak N " + numbers(None, 2, 7, 1))
        size = np.linalg.norm(force.reshape(len(force), 2, 3), axis=2)
        known = np.isfinite(size).any(axis=0)
        now = [value if math.isfinite(value) else math.nan for value in size[-1]]
        peak = [float(np.nanmax(size[:, arm])) if known[arm] else math.nan for arm in range(2)]
        return values("|F| now  N " + numbers(now, 2, 7, 1) + "   L, R",
                      "|F| peak N " + numbers(peak, 2, 7, 1) + "   since the start or Clear")
