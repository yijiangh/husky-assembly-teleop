"""Marker error: drive with the D-pad, judging both bodies' marker errors while moving; refine in Motive if large."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import viser

from ....colors import rgb
from ....shared.guide.panel import set_changed
from ....ui.style import BUSY, FAIL, indented, warning, wrapped
from ....world.checks import BAD, Check
from ..analysis.checks import DRIVE_SECONDS, PRECONDITION_HOLDS, MARKER_ERROR_LIMIT, marker_checks
from .base import CalibrationStep

if TYPE_CHECKING:
    from ....plugin_api.context import PluginContext

#: What a refine of each body makes out of date (`Run.take_current`, `validation_current`, `punch_current`).
REFINE_EFFECT = {"base": "takes (and with them the solution and Apply), validation samples, punch takes",
                 "tool": "validation samples"}
#: Shown while the base body is refined, or would need to be.
BASE_REFINE_WARNING = ("Refining the base body shifts the base pose every robot consumer gets from the relay "
                       "(monitors, the onboard follower) until this run is applied: refine it only if you finish the "
                       "run.")


class MarkerError(CalibrationStep):
    """Precondition: both bodies' largest marker error while driving, within the hour and not red; needs Setup.

    Red holds once accepted (`Run.marker_expired`). A refine changes the body's definition in Motive, which puts results
    out of date (`REFINE_EFFECT`) and ends the refine here by itself (`Run.follow`); Recheck ends it by hand.
    """

    name = "Marker error"
    instructions = ("Drive around with the D-pad, turning both ways, so Motive sees the rigid bodies from new angles; "
                    "their marker errors must stay small. If one does not: look at its markers and Recheck; if it "
                    "stays large, Refine that body in Motive while driving; it ends here once Motive applied it. A "
                    "measurement holds for "
                    f"{PRECONDITION_HOLDS / 60:.0f} min.")
    drives = moves_robot = precondition = True
    needs = ("Setup",)
    on_laptop = "Refining a body: on the laptop, beside Motive."

    def build(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """The D-pad, Recheck, and the refine buttons; the readout below them."""
        self.build_movers(ctx, gui)
        self.build_robot_buttons(ctx, gui)
        # * A body's Refine only shows once its marker error is too large (`draw`).
        self._refine_buttons = {body: gui.add_button(
            f"Refine {body} body", color=rgb(BUSY), icon=viser.Icon.ADJUSTMENTS, visible=False,
            hint=f"Refine the {body} rigid body in Motive while driving; not measured. Puts this run's "
                 f"{REFINE_EFFECT[body]} out of date.") for body in ("base", "tool")}
        # * The readout last: its lines come and go, and must not move the buttons.
        self._html = gui.add_html("")
        for body, button in self._refine_buttons.items():
            button.on_click(ctx.defer(f"refine {body}", lambda body=body: self._start_refine(body)))

    def build_robot_buttons(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """Recheck: used next to the robot, on both pages."""
        recheck = gui.add_button("Recheck", icon=viser.Icon.REFRESH,
                                 hint=f"Measure again from zero: {DRIVE_SECONDS:g} s of driving. Also ends a refine.")
        recheck.on_click(ctx.defer("recheck markers", self._recheck))

    def enter(self, ctx: PluginContext) -> None:
        """Drive, and judge the marker errors on every tick driven."""
        super().enter(ctx)
        self.cal.driving.on_driven = self._note_marker_errors

    def leave(self, ctx: PluginContext) -> None:
        """Stop judging the marker errors."""
        self.cal.driving.on_driven = None

    def _note_marker_errors(self, ctx: PluginContext) -> None:
        """Count a tick of driving, and keep each body's largest marker error; not while refining."""
        run, tool = self.run, self.cal.tool
        base = run.robot.base
        if run.refining or tool is None or base.state.marker_error is None or tool.marker_error is None:
            return
        # * Errors measured with an earlier definition of either body do not count for the new one.
        if run.marker_made is not None and run.changes(run.marker_made):
            run.reset_markers()
        errors = (base.state.marker_error, tool.marker_error)
        run.marker_max = list(errors) if run.marker_max is None else list(np.maximum(run.marker_max, errors))
        run.drive_seconds += ctx.config.tick_period
        run.marker_time, run.marker_made = self.cal.wall_now(), run.made

    def _recheck(self) -> None:
        """Measure the marker errors again from zero, e.g. after cleaning or reseating a marker; ends a refine."""
        self.run.refining = self.run.refining_from = None
        self.run.reset_markers()
        self.run.save()

    def _start_refine(self, body: str) -> None:
        self.run.start_refine(body)

    def _too_large(self) -> list[str]:
        """The bodies whose largest marker error while driving is over MARKER_ERROR_LIMIT (red): "base", "tool"."""
        if self.run.marker_max is None:
            return []
        return [name for name, error in zip(("base", "tool"), self.run.marker_max) if error > MARKER_ERROR_LIMIT]

    def checks(self, ctx: PluginContext) -> list[Check]:
        """Enough driving, both bodies' largest marker error small, measured within PRECONDITION_HOLDS; not refining."""
        checks = marker_checks(self.run.marker_max, self.run.drive_seconds, self.run.marker_age(self.cal.wall_now()))
        if self.run.refining:
            checks[0] = Check("drive", BAD, f"refining the {self.run.refining} body in Motive: ends once Motive "
                                            f"applied it, or Recheck")
        return checks

    def ready(self, ctx: PluginContext) -> list[Check]:
        """Setup's checklist and the safety boxes, for the D-pad."""
        return self.needs_checks(ctx)

    def none_yet(self, ctx: PluginContext) -> str | None:
        """Not measured yet: drive with the D-pad."""
        return "not measured: drive with the D-pad" if self.run.marker_max is None else None

    def expired(self, ctx: PluginContext) -> str | None:
        """Why the marker errors have expired for Record and Validate: not measured, too old, red, or refining."""
        if self.run.refining:
            return f"refining the {self.run.refining} body"
        return self.run.marker_expired(self.cal.wall_now())

    def draw(self, ctx: PluginContext) -> None:
        """How long the base was driven and the largest errors; how to refine once one is too large."""
        run, too_large = self.run, self._too_large()
        for body, button in self._refine_buttons.items():
            set_changed(button, visible=body in too_large and not run.refining)
        if run.refining:
            self._html.content = wrapped(
                f"Refining the {run.refining} body: in Motive select it → Builder → Refine → Start; drive around with "
                f"the D-pad, turning both ways; Apply in Motive. This ends by itself once the relay announces the new "
                f"definition; without one, Recheck.",
                f"A refine changes the rigid body: this run's {REFINE_EFFECT[run.refining]} go out of date.") + (
                warning(BASE_REFINE_WARNING) if run.refining == "base" else "")
            return
        lines = [f"{run.drive_seconds:.1f} of {DRIVE_SECONDS:g} s driven"]
        if run.marker_max is not None:
            base, tool = np.array(run.marker_max) * 1e3
            lines.append(f"largest marker error: base {base:.2f} mm, tool {tool:.2f} mm")
        html = indented(*lines)
        if too_large:
            html += wrapped(
                f"The {' and '.join(too_large)} marker error is over {MARKER_ERROR_LIMIT * 1e3:g} mm. Look at the "
                f"markers first (loose, dirty, hidden), then Recheck; if it stays large, Refine that body.")
            if "base" in too_large:
                html += warning(BASE_REFINE_WARNING, FAIL)
        self._html.content = html
