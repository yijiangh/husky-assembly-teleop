"""
Example 5: recording an arm's joint states at the full ROS rate.

! EXPERIMENTAL: see robot_interface/recording.py; how recording works is not decided yet.

"Record 5 s" saves every JointState message for five seconds (stamps and positions)
to an .npz file in RECORDING_FOLDER, and shows the sample count and rate.

  * `record` sees every message, not one per tick, and uses each message's `header.stamp`:
    callbacks run in batches at tick time, so the node clock would be wrong.
  * `with record(...)` detaches when the block ends, also when the task is cancelled.

! If the loop stalls longer than the queue covers (about 200 ms at 500 Hz), DDS drops
  samples. To never lose one, use `ros2 bag record`.

Run with:  -p plugins:="['example_recording']" -p robots:="['0806']"
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import viser

from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...robot_interface.recording import record
from ...ui.style import BUSY, NONE, OK, SECTION_SENSOR, block, chip, section, values

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
        # * Rate from the sender's stamps: the true sample rate.
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
