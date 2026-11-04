"""
The calibration's plots: live marker errors and base drift, and the old pipeline's figures of Solve and Validate.

Two tabs under Calibration in the minimized panel at the right edge. A figure shows only while the solution it
came from is current.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Callable

import numpy as np
import viser
from PIL import Image as PILImage

from ....plugin_api.trace import Signal
from ....ui.style import SECTION_CTRL, indented, note, section, wrapped
from ....ui.trace_plot import TracePlot
from ....world.mocap import MARKER_ERROR_WARN
from ..analysis.checks import MARKER_ERROR_LIMIT, pose_change
from ..analysis.report import summary_lines

if TYPE_CHECKING:
    from ....plugin_api.context import PluginContext
    from ..run import Run, Solved

#: Live diagnostics kept: a minute at the 20 Hz tick.
LIVE_SAMPLES = 1200
#: The two tabs, in the minimized panel.
LIVE_TAB = ("Live", viser.Icon.ACTIVITY)
RESULTS_TAB = ("Results", viser.Icon.CHART_DOTS)


class ResultsPanel:
    """The Live and Results tabs."""

    def __init__(self, run: Callable[[], Run | None], tool_error: Callable[[], float | None]):
        """Show the run `run()` gives, and the tool body's marker error `tool_error()` gives (metres)."""
        self._run, self._tool_error = run, tool_error
        #: Each figure's image by file name, and the run and solution it came from.
        self._images: dict[str, tuple[viser.GuiImageHandle, Run, Solved | None]] = {}
        self._summary_for: tuple[Run, Solved] | None = None

    def build(self, ctx: PluginContext) -> None:
        """Add the Live and Results tabs, hidden by default."""
        markers = Signal("marker_error", self._marker_errors, labels=("base", "tool"), unit="mm")
        drift = Signal("base_drift", lambda: self._drift(ctx, 0), unit="mm")
        turn = Signal("base_turn", lambda: self._drift(ctx, 1), unit="°")
        self._live = ctx.trace(markers, drift, turn, max_samples=LIVE_SAMPLES)
        with ctx.view.tab(*LIVE_TAB, hidden=True) as gui:
            self._live_html = gui.add_html("")
            limits = f"amber over {MARKER_ERROR_WARN * 1e3:g} mm, red over {MARKER_ERROR_LIMIT * 1e3:g} mm"
            gui.add_html(section("marker error", SECTION_CTRL, limits))
            self._plots = [TracePlot(gui, self._live, markers, seconds=60.0, y_range=(0.0, 3.0))]
            gui.add_html(section("base drift", SECTION_CTRL, "from where collisions were checked"))
            self._plots.append(TracePlot(gui, self._live, drift, seconds=60.0, min_span=5.0))
            self._plots.append(TracePlot(gui, self._live, turn, seconds=60.0, min_span=0.5))
        with ctx.view.tab(*RESULTS_TAB, hidden=True) as gui:
            self._summary_html = gui.add_html(self._no_summary())

    @staticmethod
    def _no_summary() -> str:
        return note("Solve's, Validate's and Punch's figures show here, as the old pipeline drew them.")

    def _marker_errors(self) -> tuple[float, float] | None:
        """Both bodies' marker errors now, mm (NaN while unknown); None without a run."""
        run = self._run()
        if run is None:
            return None
        errors = (run.robot.base.state.marker_error, self._tool_error())
        return tuple(math.nan if error is None else error * 1e3 for error in errors)

    def _drift(self, ctx: PluginContext, part: int) -> float | None:
        """How far the base is from where collisions were checked: part 0 moved in mm, 1 turned in °; None before."""
        run = self._run()
        if run is None or run.collision_base is None:
            return None
        metres, degrees = pose_change(run.collision_base, ctx.kinematics.base_pose(run.serial))
        return metres * 1e3 if part == 0 else degrees

    def draw(self) -> None:
        """Draw the live plots and readout; hide what came from a solution no longer current."""
        for plot in self._plots:
            plot.draw()
        markers, drift = self._live.latest("marker_error"), self._live.latest("base_drift")
        turn = self._live.latest("base_turn")
        self._live_html.content = indented(
            "marker error now: " + (f"base {markers[0]:.2f} mm, tool {markers[1]:.2f} mm" if markers is not None
                                    else "-"),
            f"base drift: {drift[0]:.1f} mm, {turn[0]:.2f}°" if drift is not None and not math.isnan(drift[0])
            and turn is not None else "base drift: collisions not checked yet")
        run = self._run()
        solved = run.current_solved() if run is not None else None
        if self._summary_for is not None and (self._summary_for[0] is not run or self._summary_for[1] is not solved):
            self._summary_for, self._summary_html.content = None, self._no_summary()
        for handle, figure_run, figure_solved in self._images.values():
            current = figure_run is run and figure_solved is solved
            if handle.visible != current:
                handle.visible = current

    async def write_and_show(self, ctx: PluginContext, run: Run, write, *args) -> None:
        """Write the old pipeline's files on a worker thread, log the headline numbers, show the figures.

        Args:
            ctx: The plugin's context.
            run: The run they belong to.
            write: Writes the files, e.g. `old_outputs.write_solve_outputs`; returns the files written.
            *args: Its arguments.
        """
        solved = run.current_solved()
        try:
            written = await ctx.run_in_thread(write, *args)
        except (OSError, ValueError) as error:
            ctx.log_warn(f"calibration outputs failed: {error}")
            return
        figures = [path for path in written if path.suffix == ".png"]
        if solved is not None and solved is run.current_solved():
            lines = summary_lines(solved.solution, run.applied_calibration(),
                                  solved.solution.footprint, figures,
                                  run.validation().spread, solved.mount[1] if solved.mount else None)
            for line in lines:
                ctx.log_info(f"calibration {run.serial} {run.arm}: {line}")
            self._summary_for, self._summary_html.content = (run, solved), wrapped(*lines)
        for path in figures:
            image = await ctx.run_in_thread(lambda p=path: np.asarray(PILImage.open(p).convert("RGB")))
            entry = self._images.get(path.name)
            if entry is None:
                with ctx.view.tab(*RESULTS_TAB, hidden=True) as gui:
                    handle = gui.add_image(image, label=path.name)
            else:
                handle = entry[0]
                handle.image = image
            handle.visible = True
            self._images[path.name] = (handle, run, solved)
