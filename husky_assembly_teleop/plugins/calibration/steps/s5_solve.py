"""Solve: the old pipeline's math on the good current takes; the session's files where the relay finds them."""

from __future__ import annotations

from dataclasses import replace
from typing import TYPE_CHECKING

import viser

from ....world.checks import BAD, GOOD, PENDING, Check
from ....ui.style import indented
from ..analysis.checks import SOLVE_LABELS, mount_checks, solve_checks, take_count_check, tcp_offset_check
from ..analysis.old_outputs import write_solve_outputs
from .base import CalibrationStep

if TYPE_CHECKING:
    from ....plugin_api.context import PluginContext


class Solve(CalibrationStep):
    """The solution and its written output, built from the good current takes; needs nothing."""

    name = "Solve"
    instructions = "Press Solve and write. Nothing changes on the robot until Apply."
    on_laptop = "Solve on the laptop."

    def build(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """Solve and write; the readout below it."""
        button = gui.add_button("Solve and write", icon=viser.Icon.MATH_FUNCTION)
        # * The readout last: its lines come and go, and must not move the buttons.
        self._html = gui.add_html("")
        button.on_click(ctx.defer("solve", lambda: self._solve(ctx)))

    def _solve(self, ctx: PluginContext) -> None:
        """Solve from this run's good current takes, and write the session's outputs where the relay finds them.

        This run's out-of-date takes and validation files go to archive/ first (`Run.tidy`).
        """
        run, procedure = self.run, self.cal.procedure
        if self.busy("solve not started"):
            return
        if (refusal := self.refusal(ctx)) is not None:
            procedure.outcome = Check("solve not started", BAD, refusal)
            return
        try:
            run.solve_and_write()
            run.save(start=True)
        except (ValueError, KeyError, OSError) as error:
            procedure.outcome = Check("solve failed", BAD, str(error))
            return
        self.write_outputs(ctx)

    def write_outputs(self, ctx: PluginContext) -> None:
        """Write the analysis and, but for a mount check, the old pipeline's files and figures of the solution."""
        run = self.run
        run.write_analysis()
        solved = run.current_solved()
        if run.mount_check or solved is None:
            # ? The old pipeline's files are per session, for the arm that calibrates the base: a mount check would
            # ? overwrite them.
            return
        offset = run.punch_offset if tcp_offset_check(run.punch_offset).level == GOOD else None
        ctx.spawn("calibration outputs", self.cal.results.write_and_show(
            ctx, run, write_solve_outputs, run.session, run.digits, run.arm, run.urdf, solved.solution,
            list(solved.files), offset), survives_soft_stop=True)

    def none_yet(self, ctx: PluginContext) -> str | None:
        """Not solved yet."""
        return "press Solve and write" if self.run.solved is None else None

    def out_of_date(self, ctx: PluginContext) -> str | None:
        """Solved from other takes than the good current ones."""
        return "the takes changed since: Solve again" if self.run.current_solved() is None else None

    def checks(self, ctx: PluginContext) -> list[Check]:
        """A current solution: its fits and axes, and whether the relay would take it (a mount check: the mount)."""
        run = self.run
        solved = run.current_solved()
        if solved is None or solved.output is None:
            # * Red only out of date: not solved yet is grey, as the step's state.
            solve = (Check("solve", BAD, "the takes changed since: Solve again") if run.solved is not None else
                     Check("solve", PENDING, "press Solve and write"))
            labels = ("mount",) if run.mount_check else SOLVE_LABELS
            return [solve] + [Check(label, PENDING, "not solved yet") for label in labels]
        solve = Check("solve", GOOD, f"solved from {len(solved.files)} takes; wrote {solved.output.name}",
                      f"{len(solved.files)} takes")
        if solved.mount is not None:
            return [solve] + mount_checks(solved.mount[1])
        applied = run.applied_calibration()
        return [solve] + solve_checks(solved.solution, applied, solved.sanity)

    def ready(self, ctx: PluginContext) -> list[Check]:
        """Enough good takes per joint to solve from: red under 2 a joint, as Record's takes chip."""
        run = self.run
        counts = [sum(run.sweeps.sweeps[index].joint == joint for index in run.good_takes()) for joint in (0, 1)]
        return [replace(take_count_check(counts, "pass their checks"), label="good takes")]

    def draw(self, ctx: PluginContext) -> None:
        """How many takes are good, and what was written."""
        run = self.run
        solved = run.current_solved()
        self._html.content = indented(f"{len(run.good_takes())} of {len(run.current_takes())} takes good",
                                      f"wrote {solved.output.name}" if solved is not None and solved.output
                                      else "nothing written yet")
