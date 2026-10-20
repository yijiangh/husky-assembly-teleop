"""
Example 5: live plots and recordings of signals, ready-made and your own, per tick and per message.

A signal is a name and a function that reads a few numbers. Once per tick, `read()` sees the monitor's state after
every plugin's update. With `on=event`, `read(record)` gets every message of an interface's event instead, e.g. every
mocap sample of the base. `plugin_api/signals.py` has ready-made ones of both kinds.

  * A live plot of the first arm's joints: `ctx.trace` keeps them, `TracePlot` draws them.
  * "Record 5 s" records the joints and tool0's height (our own signal) every tick, and the base's floor pose and
    marker error (our own) on every mocap sample, then saves the run on the Drive. Load it with
    `load_recording(folder)`: "t" are the tick's times, "mocap.t" and "mocap.stamp" the samples'.

! A tick signal shows only the newest value of a faster stream (mocap 50 Hz, the tick 20 Hz): read it per message.

Run with:  -p plugins:="['example_recording']" -p robots:="['0806']"
"""

from __future__ import annotations

from pathlib import Path

from ...drive import drive_folder
from ...plugin_api import signals
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...plugin_api.trace import Signal
from ...shared.recordings.runs import run_folder, save_recording, write_description
from ...ui.style import BUSY, NONE, OK, SECTION_SENSOR, block, chip, section, values
from ...ui.trace_plot import TracePlot

RECORD_SECONDS = 5.0
#: Runs, under the Drive root (`drive.py`); ! create it in Drive, it is never created here.
RECORDING_FOLDER = Path("data_experiment/example_recording")
#: Samples the live plot keeps: 30 s at 20 Hz.
LIVE_SAMPLES = 600


@register
class ExampleRecordingPlugin(HuskyPlugin):
    """A live joint plot, and a button that records the first arm and its base for a few seconds."""

    name = "example_recording"

    def __init__(self):
        """Start idle."""
        self._task = None
        self._result = ""

    def setup(self, ctx: PluginContext) -> None:
        """Pick the first robot with an arm, make its signals once, start the live plot and build the panel."""
        self._robot = next((robot for robot in ctx.world.robots.values() if robot.arms), None)
        try:
            self._recordings, problem = drive_folder(ctx.config.drive_root, RECORDING_FOLDER), ""
        except FileNotFoundError as error:
            self._recordings, problem = None, str(error)
        with ctx.view.ui() as gui:
            self._status = gui.add_html("")
            gui.add_html(section("record", SECTION_SENSOR))
            button = gui.add_button(f"Record {RECORD_SECONDS:.0f} s", disabled=bool(problem), hint=problem or None)
            if self._robot is None:
                return
            robot = self._robot
            arm = next(iter(robot.arms.values()))
            serial, tool0 = robot.config.serial, f"{arm.config.name}_tool0"
            joints = signals.joints(arm)
            self._signals = [
                joints,
                # * Our own tick signal: any function of the monitor's state, here forward kinematics.
                Signal(f"{tool0}_height", lambda: ctx.kinematics.link_pose(serial, tool0).position[2], unit="m"),
                # * Ready-made, on every mocap sample instead of once per tick.
                signals.base_floor_pose(robot, every_message=True),
                # * Our own message signal: it reads the sample it is handed.
                Signal("marker_error", lambda sample: sample.marker_error, unit="m", on=robot.base.mocap_messages),
            ]
            self._plot = TracePlot(gui, ctx.trace(joints, max_samples=LIVE_SAMPLES), joints)
        button.on_click(ctx.defer("record", lambda: self._start(ctx)))

    def _start(self, ctx: PluginContext) -> None:
        """Start a recording unless one is running (intent)."""
        if self._task is None or self._task.done():
            self._task = ctx.spawn("record", self._record(ctx))

    async def _record(self, ctx: PluginContext) -> None:
        """Record for RECORD_SECONDS, then save the run."""
        self._result = "recording..."
        with ctx.record(*self._signals) as rec:
            await ctx.sleep(RECORD_SECONDS)
        serial = self._robot.config.serial
        folder = run_folder(self._recordings, "default", serial, "example")
        write_description(folder, {"robot": serial, "seconds": RECORD_SECONDS})
        path = save_recording(folder, rec, serial, file_format=ctx.config.recording_format)
        self._result = f"{len(rec)} samples -> {path}"
        ctx.log_info(f"recording saved: {self._result}")

    def draw(self, ctx: PluginContext) -> None:
        """Show whether a recording runs, the last result and the live plot."""
        if self._robot is None:
            self._status.content = block(chip("no robot with an arm", NONE))
            return
        running = self._task is not None and not self._task.done()
        self._status.content = block((chip("recording", BUSY) if running else chip("idle", OK)) + values(self._result))
        self._plot.draw()
