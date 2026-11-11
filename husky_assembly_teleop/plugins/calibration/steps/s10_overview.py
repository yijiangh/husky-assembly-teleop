"""Overview: the run's result in one place: every step's verdict and chips, the headline numbers, the files written."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import viser

from ....ui.style import SECTION_CTRL, check_chip, section, wrapped
from ....world.checks import BAD, GOOD, PENDING, WARN, Check
from ..analysis.report import summary_lines
from ..storage.session_files import BATCHES, PUNCH_BATCH, VALIDATION_BATCH
from ..views.takes_view import chip_row
from .base import CalibrationStep

if TYPE_CHECKING:
    from ....plugin_api.context import PluginContext

#: The session's folders holding figures, "" for the session itself.
FIGURE_FOLDERS = ("", *BATCHES, VALIDATION_BATCH, PUNCH_BATCH)
#: The steps whose `write_outputs` Write figures again calls, in order.
WRITERS = ("Solve", "Validate", "Punch")
#: The steps that never fail the run: their worst is amber.
OPTIONAL = ("Tip offset", "Punch")
#: Seconds between looks at the files on the Drive, while the step shows.
FILES_PERIOD = 2.0


class Overview(CalibrationStep):
    """Every other step's state and chips, the headline numbers and the session's figures; done once Apply is."""

    name = "Overview"
    instructions = ("Check that no step is red. The figures show in the Results tab at the screen's right edge; press "
                    "Write figures again if one is missing or old.")
    on_laptop = "The overview and Write figures again are on the laptop."

    def __init__(self, cal):
        """No files looked at yet."""
        super().__init__(cal)
        self._files: list[str] = []
        self._looked = -FILES_PERIOD

    def build(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """Write figures again; the overview below it."""
        button = gui.add_button("Write figures again", icon=viser.Icon.CHART_DOTS,
                                hint="Solve's, Validate's and Punch's figures and the analysis, from the current "
                                     "solution and takes.")
        # * The readout last: it grows as the run does, and must not move the button.
        self._html = gui.add_html("")
        button.on_click(ctx.defer("write figures", lambda: self._write(ctx)))

    def _write(self, ctx: PluginContext) -> None:
        if self.busy("figures not written"):
            return
        for name in WRITERS:
            self.cal.steps[name].write_outputs(ctx)
        self._looked = -FILES_PERIOD

    def _others(self) -> list[CalibrationStep]:
        return [step for step in self.cal.steps.values() if step is not self]

    def none_yet(self, ctx: PluginContext) -> str | None:
        """Not applied yet (a mount check: not written yet)."""
        return None if self.run.finished else "the calibration is not applied yet: Apply"

    def checks(self, ctx: PluginContext) -> list[Check]:
        """The required steps' worst state, and the optional ones' (amber at worst: they never fail the run)."""
        states = [(step.name, step.state(ctx)) for step in self._others()]
        return [_worst("required", [(name, state) for name, state in states if name not in OPTIONAL]),
                _worst("optional", [(name, state) for name, state in states if name in OPTIONAL], WARN)]

    def draw(self, ctx: PluginContext) -> None:
        """A row per step with its chips, the headline numbers, and the files below the session."""
        run = self.run
        if ctx.now() - self._looked >= FILES_PERIOD:
            self._looked, self._files = ctx.now(), self._list_files(run.session)
        # * Each step's own chips: its state is in the step list above.
        rows = "".join(chip_row(step.name, "".join(check_chip(check) for check in step.checks(ctx)), wrap=True)
                       for step in self._others())
        solved = run.current_solved()
        numbers = (summary_lines(solved.solution, run.applied_calibration(), solved.solution.footprint, (),
                                 run.validation().spread, solved.mount[1] if solved.mount else None)
                   if solved is not None else ["not solved yet"])
        if any(run.punch_offset):
            numbers.append("TCP offset: (" + ", ".join(f"{value * 1e3:.2f}" for value in run.punch_offset) + ") mm")
        session = str(run.session) if run.session is not None else "not written yet"
        self._html.content = (section("steps", SECTION_CTRL) + rows + section("numbers", SECTION_CTRL)
                              + wrapped(*numbers) + section("files", SECTION_CTRL) + wrapped(session, *self._files))

    @staticmethod
    def _list_files(session: Path | None) -> list[str]:
        """Each figure folder below `session` with its figures, e.g. "validation/: verification_validation.png"."""
        if session is None:
            return []
        lines = []
        for folder in FIGURE_FOLDERS:
            figures = sorted(path.name for path in (session / folder).glob("*.png"))
            if figures:
                lines.append(f"{folder or '.'}/: " + ", ".join(figures))
        return lines or ["no figures yet"]


def _worst(label: str, states: list[tuple[str, Check]], cap: str = BAD) -> Check:
    """The worst of the named states as one check, no worse than `cap`: a short value, the steps in its hover."""
    order = (GOOD, PENDING, WARN, BAD)
    worst = max((state.level for _name, state in states), key=order.index, default=GOOD)
    if worst == GOOD:
        return Check(label, GOOD, "every step passes")
    names = [(name, state.label) for name, state in states if state.level == worst]
    level = min(worst, cap, key=order.index)
    # * The chip stays short: one step by name, several by count; the hover lists each with its state.
    value = names[0][0] if len(names) == 1 else f"{len(names)} steps"
    return Check(label, level, "; ".join(f"{name} {state}" for name, state in names), value)
