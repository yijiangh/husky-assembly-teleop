"""
Example 4: a long-running sequence that waits, can be cancelled, and asks the
operator to continue.

The sequence is a generator: it reads top to bottom, and each `yield` hands the
thread back until the next tick. Start spawns it as a job (`ctx.spawn(...)`);
Cancel calls `job.cancel()`, which raises `Cancelled` at the current `yield` and
runs the `finally` block, the place to stop motors or open a gripper.

Each cycle, with fake waits in place of robot work (three cycles, then finish):
  1. wait 3 s                              (wait_seconds)
  2. wait for the operator to press Next   (wait_until on a flag, with a timeout)
  3. wait for a fake sensor to fill up     (wait_until on a reading, with a timeout)

! Ignore (and log) a button click that does not apply now, such as Next while
  nothing waits for it: a stored click would fire long after the operator forgot it.

! Catch the failures you expect, such as a timeout. Repeated failing ticks stop
  the plugin, so report them from the sequence instead of raising.

Run with:  -p plugins:="['example_sequence']"
"""

from __future__ import annotations

import viser

from ...concurrency import Cancelled, Job, Task, WaitTimeout, wait_seconds, wait_until
from ...context import PluginContext
from ...plugin import HuskyPlugin, register
from ...ui_style import BUSY, FAIL, NONE, OK, SECTION_CTRL, block, chip, section

CYCLES = 3
TIMER_SECONDS = 3.0
SENSOR_FILL_SECONDS = 4.0     # time for the fake sensor to reach 1.0
NEXT_TIMEOUT = 30.0           # give up if nobody presses Next
SENSOR_TIMEOUT = 10.0


@register
class ExampleSequencePlugin(HuskyPlugin):
    """A three-cycle fake automation with Start, Next and Cancel."""

    name = "example_sequence"

    def __init__(self):
        """Start idle."""
        #: The last sequence started, running or finished; None before the first Start.
        self._job: Job | None = None
        #: Set by the Next button, and only while the sequence waits for it.
        self._next_pressed = False
        #: What the operator sees: where the sequence is and how the last one ended.
        self.cycle = 0
        self.step = "idle"
        self.outcome = ""
        #: Start time and expected length of the current step, for the progress
        #: bar; the length is None if unset.
        self._step_started = 0.0
        self._step_seconds: float | None = None

    @property
    def running(self) -> bool:
        """bool: Whether a sequence is running right now."""
        return self._job is not None and not self._job.done

    def setup(self, ctx: PluginContext) -> None:
        """Build the status, a progress bar and the three buttons.

        Args:
            ctx: This plugin's context.
        """
        with ctx.view.ui() as gui:
            self._status: viser.GuiHtmlHandle = gui.add_html("")
            self._progress = gui.add_progress_bar(0.0, color="blue")
            gui.add_html(section("sequence", SECTION_CTRL))
            buttons = gui.add_button_group("Seq", ["Start", "Next", "Cancel"])
        buttons.on_click(ctx.defer_value("sequence button", lambda clicked: self._on_button(ctx, clicked)))

    # --- --- --- --- --- BUTTONS (an intent, on the ROS thread) --- --- --- --- ---

    def _on_button(self, ctx: PluginContext, clicked: str) -> None:
        """Act on a button, or ignore it if it does not apply right now.

        Args:
            ctx: This plugin's context.
            clicked: The label of the button that was pressed.
        """
        if clicked == "Start" and not self.running:
            self._job = ctx.spawn("example sequence", self._sequence(ctx))
        elif clicked == "Next" and self.step == "press Next":
            self._next_pressed = True
        elif clicked == "Cancel" and self.running:
            self._job.cancel()
        else:
            ctx.log_info(f"{clicked} ignored: the sequence is {self.step}")

    # --- --- --- --- --- THE SEQUENCE (a job) --- --- --- --- ---

    def _sequence(self, ctx: PluginContext) -> Task:
        """Run the automation, written in the order things happen.

        Args:
            ctx: This plugin's context.

        Yields:
            None: Once per tick while it waits.
        """
        self.outcome = ""
        try:
            for cycle in range(1, CYCLES + 1):
                self.cycle = cycle

                self._begin_step(ctx, "timer", TIMER_SECONDS)
                yield from wait_seconds(ctx, TIMER_SECONDS)

                self._begin_step(ctx, "press Next")
                yield from wait_until(ctx, lambda: self._next_pressed,
                                      timeout_s=NEXT_TIMEOUT, description="the operator to press Next")
                self._next_pressed = False

                self._begin_step(ctx, "sensor", SENSOR_FILL_SECONDS)
                yield from wait_until(ctx, lambda: self._fake_sensor(ctx) >= 1.0,
                                      timeout_s=SENSOR_TIMEOUT, description="the fake sensor")
            self.outcome = "done"
        except WaitTimeout as timeout:
            # Expected: report it, do not raise.
            self.outcome = "timed out"
            ctx.log_warn(str(timeout))
        except Cancelled:
            self.outcome = "cancelled"
            raise  # ! Re-raise, so the job ends as cancelled
        except Exception:
            self.outcome = "failed"
            raise  # a bug: let it reach the log
        finally:
            # * Runs however the sequence ends: stop motors and open grippers here.
            self.step, self._step_seconds, self._next_pressed = "idle", None, False
            ctx.log_info(f"sequence ended: {self.outcome}")

    def _begin_step(self, ctx: PluginContext, step: str, seconds: float | None = None) -> None:
        """Move on to the next step.

        Args:
            ctx: This plugin's context.
            step: The step's name, shown to the operator.
            seconds: How long the step should take, for the progress bar.
        """
        self.step = step
        self._step_started = ctx.now()
        self._step_seconds = seconds

    def _fake_sensor(self, ctx: PluginContext) -> float:
        """Return a fake reading that rises from 0 to 1 over the sensor step."""
        return (ctx.now() - self._step_started) / SENSOR_FILL_SECONDS

    # --- --- --- --- --- DRAW --- --- --- --- ---

    def draw(self, ctx: PluginContext) -> None:
        """Show the step, cycle and how the last run ended.

        Args:
            ctx: This plugin's context.
        """
        waiting = self.step == "press Next"
        if self.running:
            chips = chip(f"cycle {self.cycle}/{CYCLES}", OK) + chip(self.step, BUSY if waiting else OK)
        else:
            chips = chip("idle", NONE)
        if self.outcome:
            chips += chip(self.outcome, OK if self.outcome == "done" else FAIL)
        self._status.content = block(chips)

        progress = 0.0
        if self._step_seconds:
            progress = min((ctx.now() - self._step_started) / self._step_seconds, 1.0)
        self._progress.value = progress * 100
