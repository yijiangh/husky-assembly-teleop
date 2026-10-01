"""
Example 5: live plots and recordings of signals, one sample per tick.

  * A live plot of the first arm's joints: `ctx.trace` samples them every tick, `TracePlot` draws them.
  * "Record 5 s" records joints, tool0 by forward kinematics, and the force/torque sensor for five
    seconds, then saves them to an .npz file in RECORDING_FOLDER. Load it with `np.load(path)`.

  * A signal is any function of live state, so derived values (FK, errors) record like measurements.
  * `with ctx.record(...)` stops when the block ends, also when the task is cancelled.

! One sample per tick (20 Hz): for faster dynamics (contact, controller tuning) `ros2 bag record` on the robot.

Run with:  -p plugins:="['example_recording']" -p robots:="['0806']"
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import viser

from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...ui.style import BUSY, NONE, OK, SECTION_SENSOR, block, chip, section, values
from ...ui.trace_plot import TracePlot
from ...world import signals

RECORD_SECONDS = 5.0
RECORDING_FOLDER = Path("/tmp/husky_recordings")
#: Samples the live plot keeps: 30 s at 20 Hz.
LIVE_SAMPLES = 600


@register
class ExampleRecordingPlugin(HuskyPlugin):
    """A live joint plot, and a button that records the first arm for a few seconds."""

    name = "example_recording"

    def __init__(self):
        """Start idle."""
        self._task = None
        self._result = ""
        self._plot: TracePlot | None = None

    def setup(self, ctx: PluginContext) -> None:
        """Pick the first arm of the first robot, start its live trace and build the panel.

        Args:
            ctx: This plugin's context.
        """
        self._robot = next(iter(ctx.world.robots.values()), None)
        self._arm = next(iter(self._robot.arms.values()), None) if self._robot else None
        with ctx.view.ui() as gui:
            self._status: viser.GuiHtmlHandle = gui.add_html("")
            gui.add_html(section("record", SECTION_SENSOR))
            button = gui.add_button(f"Record {RECORD_SECONDS:.0f} s")
            if self._arm is not None:
                live = ctx.trace(signals.joints(self._arm), max_samples=LIVE_SAMPLES)
                self._plot = TracePlot(gui, live, signals.joints(self._arm).name)
        if self._arm is None:
            ctx.log_warn("example_recording: no robot with an arm, nothing to record")
            return
        # * The intent spawns the async def as a task.
        button.on_click(ctx.defer("record", lambda: self._start(ctx)))

    def _start(self, ctx: PluginContext) -> None:
        """Start a recording unless one is running (intent).

        Args:
            ctx: This plugin's context.
        """
        if self._task is not None and not self._task.done():
            ctx.log_info("recording ignored: one is already running")
            return
        self._task = ctx.spawn("record arm", self._record(ctx))

    async def _record(self, ctx: PluginContext) -> None:
        """Record for RECORD_SECONDS, then save.

        Args:
            ctx: This plugin's context.
        """
        arm, serial = self._arm, self._robot.config.serial
        self._result = "recording..."
        with ctx.record(signals.joints(arm),
                        signals.link_position(ctx.kinematics, serial, f"{arm.config.name}_tool0"),
                        signals.force(arm), signals.torque(arm)) as rec:
            await ctx.sleep(RECORD_SECONDS)

        joints = rec.values(signals.joints(arm).name)
        if np.isnan(joints).all():
            self._result = "no joint states: is the arm connected?"
            return
        path = rec.save(RECORDING_FOLDER / f"arm_{time.strftime('%Y%m%d_%H%M%S')}.npz")
        self._result = f"{len(rec)} samples -> {path}"
        ctx.log_info(f"recording saved: {self._result}")

    def draw(self, ctx: PluginContext) -> None:
        """Show whether a recording runs, the last result and the live plot.

        Args:
            ctx: This plugin's context.
        """
        if self._arm is None:
            self._status.content = block(chip("no arm", NONE))
            return
        running = self._task is not None and not self._task.done()
        state = chip("recording", BUSY) if running else chip("idle", OK)
        self._status.content = block(state + values(self._result))
        self._plot.draw()
