"""
Example 5: recording an arm's joint states at the full ROS rate.

! EXPERIMENTAL: see recording.py; how recording works is not decided yet.

"Record 5 s" collects every JointState message for five seconds, saves the
stamps and positions to an .npz file and shows the sample count and rate.

  * The subscription queue is deep enough for one tick of samples, and
    `record` listens to every message, so the 20 Hz tick does not thin the data.
  * Samples are stamped with the message's `header.stamp`. Callbacks run in
    batches at tick time, so the node clock would give the drain time instead.
  * `with record(...)` detaches when the block ends, also when the task is
    cancelled (soft stop, plugin stop), so no listener is left behind.

! If the loop stalls for longer than the queue covers (about 200 ms at 500 Hz),
  DDS drops samples. A recording that must never lose one: use `ros2 bag record`.

Files go to /tmp/husky_recordings (change RECORDING_FOLDER).

Run with:  -p plugins:="['example_recording']" -p robots:="['0806']"
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import viser

from ...context import PluginContext
from ...plugin import HuskyPlugin, register
from ...recording import record
from ...ui_style import BUSY, NONE, OK, SECTION_SENSOR, block, chip, section, values

RECORD_SECONDS = 5.0
RECORDING_FOLDER = Path("/tmp/husky_recordings")


@register
class ExampleRecordingPlugin(HuskyPlugin):
    """A button that records the first arm's joint states for a few seconds."""

    name = "example_recording"
    experimental = True

    def __init__(self):
        """Start idle."""
        self._task = None
        self._result = ""

    def setup(self, ctx: PluginContext) -> None:
        """Pick the first arm of the first robot and build the button.

        Args:
            ctx: This plugin's context.
        """
        self._arm = None
        for robot in ctx.world.robots.values():
            self._arm = next(iter(robot.arms.values()), None)
            break
        with ctx.view.ui() as gui:
            self._status: viser.GuiHtmlHandle = gui.add_html("")
            gui.add_html(section("record", SECTION_SENSOR))
            button = gui.add_button("Record 5 s")
        if self._arm is None:
            ctx.log_warn("example_recording: no robot with an arm, nothing to record")
            return
        # * `defer` runs the async def as an intent, which spawns it as a task.
        button.on_click(ctx.defer("record", lambda: self._start(ctx)))

    def _start(self, ctx: PluginContext) -> None:
        """Start a recording unless one is running. Runs as an intent.

        Args:
            ctx: This plugin's context.
        """
        if self._task is not None and not self._task.done():
            ctx.log_info("recording ignored: one is already running")
            return
        self._task = ctx.spawn("record joint states", self._record(ctx))

    async def _record(self, ctx: PluginContext) -> None:
        """Record for RECORD_SECONDS, then save.

        Args:
            ctx: This plugin's context.
        """
        self._result = "recording..."
        with record(self._arm.samples.joint_states) as rec:
            await ctx.sleep(RECORD_SECONDS)

        if not rec.samples:
            self._result = "no samples: is the arm connected?"
            return
        stamps = rec.stamps()
        positions = np.array([message.position for _, message in rec.samples])
        RECORDING_FOLDER.mkdir(parents=True, exist_ok=True)
        path = RECORDING_FOLDER / f"joint_states_{time.strftime('%Y%m%d_%H%M%S')}.npz"
        np.savez(path, stamps=stamps, positions=positions)
        # * Rate from the sender's stamps, so it is the true sample rate.
        rate = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 and stamps[-1] > stamps[0] else 0.0
        self._result = f"{len(stamps)} samples, {rate:.0f} Hz -> {path}"
        ctx.log_info(f"recording saved: {self._result}")

    def draw(self, ctx: PluginContext) -> None:
        """Show whether a recording runs and the last result.

        Args:
            ctx: This plugin's context.
        """
        if self._arm is None:
            self._status.content = block(chip("no arm", NONE))
            return
        running = self._task is not None and not self._task.done()
        state = chip("recording", BUSY) if running else chip("idle", OK)
        self._status.content = block(state + values(self._result))
