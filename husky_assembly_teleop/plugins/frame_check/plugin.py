"""
Frame check: how far each measured bar of the built frame is from its design, live, for straightening it by hand.

Reads every bar's MoCap fit from `mocap_bars` and compares both ends with the bar's assembly pose in the loaded
process (placed). The phone tab and the desktop tab show the same: a picture from the side, the front or above with
the errors magnified, and below it the chosen bar's "move by" per end (design minus measured, world axes), or every
bar's worse end.

- * Only the bars the live scene shows built at the selected movement count: select a movement after them. Bars
    without markers, not built yet or held by a robot are left out.
- * A fit older than mocap_bars' marker age (MAX_AGE without one) counts as none: a hand over the markers greys the
    bar out rather than freezing it.

Run with:  plugins:="frame_check"  (loads `process` and `mocap_bars`)
"""

from __future__ import annotations

from html import escape
from typing import TYPE_CHECKING

import numpy as np
import viser

from ...colors import INK, NONE
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...shared.bars.layouts import bar_layouts
from ...shared.bars.pose import bar_ends, bar_gap, gap_color
from ...shared.mocap_bars_contract import MOCAP_TAB
from ...ui.choice_row import ChoiceRow
from ...ui.phone import phone_text
from ...ui.style import block, chip, chip_line
from .picture import VIEWS, BarError, ordered_ends, render

if TYPE_CHECKING:
    from scaffolding_assembly_core.process import Process

    from ...shared.bars.layouts import BarLayout
    from ...shared.mocap_bars_contract import BarMeasurement, MocapBarsContract
    from ...shared.process_contract import ProcessContract

#: The readout refreshes this often, seconds.
REFRESH_PERIOD = 0.25
#: The picture is sent again only once an end moved more than this since it was drawn, mm: mocap noise
#: (~0.04 mm) alone sends nothing.
REDRAW_MM = 0.3
#: A bar counts as built when the live scene puts it this close to its assembly pose, metres.
BUILT_TOL = 1e-3
#: A fit older than this counts as none, seconds, unless mocap_bars says its own (its "Marker age s").
MAX_AGE = 1.0
#: Magnifications of the errors in the picture.
GAINS = ("1", "10", "30")
DEFAULT_GAIN = "10"
#: The bar row's first option: every bar alike, the readout a line per bar.
ALL = "All"


def bar_errors(process: Process, layouts: dict[str, BarLayout], fits: dict[str, BarMeasurement],
               now: float, max_age: float = MAX_AGE) -> list[BarError]:
    """Every built bar with markers, its design ends and its fitted ends when fresh and good, by name.

    A bar is built when its fit's `planned` pose (the live scene's) is its assembly pose; a bar not in the scene
    or held by a robot is left out.

    Args:
        process: The loaded process, placed.
        layouts: `bar_layouts(process)`: the bars with markers.
        fits: `MocapBarsContract.fits`.
        now: ROS time, seconds.
        max_age: A fit older than this is stale, seconds.

    Returns:
        list[BarError]: Sorted by bar name.
    """
    errors = []
    for bar, layout in layouts.items():
        found = fits.get(bar)
        seat = process.bodies[bar].assembly_pose
        if found is None or found.planned is None or \
                np.linalg.norm(np.subtract(found.planned.position, seat.position)) > BUILT_TOL:
            continue
        design, measured, gap, why = bar_ends(seat.matrix(), layout.length), None, None, "no fit"
        if found.pose is not None:
            if now - found.measured_at <= max_age:
                gap, why = bar_gap(found.pose.matrix(), seat.matrix(), layout.length), ""
                design, measured = gap.design[[0, 2]], gap.measured[[0, 2]]
            else:
                why = "stale"
        design, measured, names = ordered_ends(design, measured)
        errors.append(BarError(bar.rsplit("/", 1)[-1], design, measured, names, why, gap))
    return sorted(errors, key=lambda e: e.bar)


def _cell(text: str, color: str = INK, bold: bool = False) -> str:
    """A right-aligned table cell."""
    weight = "font-weight:700;" if bold else ""
    return f'<td style="text-align:right;padding:0 6px;color:{color};{weight}">{escape(text)}</td>'


def _name(text: str) -> str:
    """A left-aligned table cell."""
    return f'<td style="padding:0 6px;color:{NONE}">{escape(text)}</td>'


def _table(header: list[str], rows: list[str]) -> str:
    """A table big enough to read at arm's length; `header` names its columns."""
    head = "".join(f'<th style="text-align:right;padding:0 6px;color:{NONE};font-weight:400">{escape(h)}</th>'
                   for h in header)
    body = "".join(f"<tr>{row}</tr>" for row in rows)
    return block(f'<table style="font:18px/1.5 monospace;border-collapse:collapse"><tr>{head}</tr>{body}</table>')


def bar_readout(error: BarError) -> str:
    """One bar: per end, which way and how far to move it to its design (world axes, mm), and its tilt."""
    if error.measured is None:
        return block(f'<div style="font-size:18px;color:{NONE}">{escape(error.bar)}: {error.why}</div>')
    rows = []
    for name, moved in zip(error.end_names, -error.errors_mm):
        size = float(np.linalg.norm(moved))
        rows.append(_name(name) + "".join(_cell(f"{v:+.1f}") for v in moved)
                    + _cell(f"{size:.1f}", gap_color(size), bold=True))
    rows.append(_name("tilt") + _cell("") * 3 + _cell(f"{error.tilt_deg:.2f}°"))
    return _table(["move", "x", "y", "z", "mm"], rows)


def frame_readout(errors: list[BarError]) -> str:
    """Every bar's worse end: its name, distance from the design (mm) and the bar's tilt."""
    rows = []
    for e in errors:
        if e.measured is None:
            rows.append(_name(e.bar) + _cell(e.why, NONE) + _cell("") * 2)
            continue
        ends = np.linalg.norm(e.errors_mm, axis=1)
        worst = int(np.argmax(ends))
        rows.append(_name(e.bar) + _cell(e.end_names[worst], NONE)
                    + _cell(f"{ends[worst]:.1f}", gap_color(ends[worst]), bold=True) + _cell(f"{e.tilt_deg:.2f}°"))
    return _table(["bar", "worse end", "mm", "tilt"], rows)


def summary_chip(errors: list[BarError]) -> str:
    """One chip: the worst end of all measured bars and how many built bars are not measured."""
    if not errors:
        return chip("no built bar with markers", NONE, "Select a movement after the bars to check: only the bars "
                                                       "the scene shows built at it count.")
    measured = [e for e in errors if e.measured is not None]
    if not measured:
        return chip("no bar measured", NONE, "No fresh MoCap fit of any built bar: are its markers seen?")
    worst = max(measured, key=lambda e: e.worst_mm)
    end = worst.end_names[int(np.argmax(np.linalg.norm(worst.errors_mm, axis=1)))]
    missing = len(errors) - len(measured)
    tail = f", {missing} not measured" if missing else ""
    return chip(f"worst {worst.bar} {end} {worst.worst_mm:.1f} mm{tail}", gap_color(worst.worst_mm),
                f"The end furthest from its design of the {len(measured)} measured built bars.")


@register
class FrameCheckPlugin(HuskyPlugin):
    """Shows how far the built frame's measured bars are from their design, on the phone and the desktop."""

    name = "frame_check"
    tab_path = (MOCAP_TAB, "Frame")
    icon = viser.Icon.RULER_MEASURE
    requires = ("process", "mocap_bars")
    experimental = True

    def __init__(self):
        """Start with the side view, errors x10, every bar."""
        self._view, self._gain, self._bar = "side", DEFAULT_GAIN, ALL
        self._errors: list[BarError] = []
        #: The process the layouts were read from, and its bars with markers.
        self._layouts_of: Process | None = None
        self._layouts: dict[str, BarLayout] = {}
        self._refreshed_at = float("-inf")
        #: The choices and the measured ends (bar -> (2, 3) or None) the picture last showed.
        self._drawn: tuple | None = None
        self._drawn_ends: dict[str, np.ndarray | None] = {}
        self._pages: list[dict] = []

    def setup(self, ctx: PluginContext) -> None:
        """Build the same view in the desktop tab and the phone tab.

        Args:
            ctx: This plugin's context.
        """
        with ctx.view.ui() as gui:
            self._pages.append(self._page(ctx, gui, phone=False))
        with ctx.view.phone("Frame", self.icon) as phone:
            self._pages.append(self._page(ctx, phone, phone=True))

    def _page(self, ctx: PluginContext, gui: viser.GuiApi, phone: bool) -> dict:
        """Chip line, picture, the view, magnification and bar rows, then the readout (it changes height)."""
        wrap = phone_text if phone else (lambda html: html)
        page = {"wrap": wrap, "chips": gui.add_html(wrap(chip_line("")))}
        page["picture"] = gui.add_image(render([], self._view, float(self._gain), None), format="png")
        page["view"] = ChoiceRow(ctx, gui, "View", tuple(VIEWS), self._view, on_choose=self._choose_view,
                                 hint="Side: seen from +x. Front: seen from -y. Top: seen from above.")
        page["gain"] = ChoiceRow(ctx, gui, "Errors", GAINS, self._gain, on_choose=self._choose_gain,
                                 labels=tuple(f"×{g}" for g in GAINS),
                                 hint="How much the picture magnifies each bar's error from its design.")
        page["bar"] = ChoiceRow(ctx, gui, "Bar", (ALL,), self._bar, on_choose=self._choose_bar,
                                hint="A bar's ends: how far to move each to its design, world axes, mm.")
        page["readout"] = gui.add_html("")
        return page

    def _choose_view(self, view: str) -> None:
        self._view = view

    def _choose_gain(self, gain: str) -> None:
        self._gain = gain

    def _choose_bar(self, bar: str) -> None:
        self._bar = bar

    def _moved(self) -> bool:
        """Whether any bar's ends moved more than REDRAW_MM since the picture was drawn, or one came or went."""
        ends = {e.bar: e.measured for e in self._errors}
        if ends.keys() != self._drawn_ends.keys():
            return True
        for bar, now in ends.items():
            then = self._drawn_ends[bar]
            if (now is None) != (then is None):
                return True
            if now is not None and np.abs(now - then).max() * 1e3 > REDRAW_MM:
                return True
        return False

    def update(self, ctx: PluginContext) -> None:
        """Compare every bar with its design every REFRESH_PERIOD.

        Args:
            ctx: This plugin's context.
        """
        now = ctx.now()
        # * abs: ROS time can jump back (bag playback).
        if abs(now - self._refreshed_at) < REFRESH_PERIOD:
            return
        self._refreshed_at = now
        process: ProcessContract = ctx.require("process")
        bars: MocapBarsContract = ctx.require("mocap_bars")
        selection = process.selection
        loaded = selection.process if selection is not None else None
        if loaded is not self._layouts_of:
            self._layouts_of, self._layouts = loaded, bar_layouts(loaded) if loaded is not None else {}
        max_age = getattr(bars, "max_marker_age", MAX_AGE)
        self._errors = bar_errors(loaded, self._layouts, bars.fits, now, max_age) if loaded is not None else []

    def draw(self, ctx: PluginContext) -> None:
        """Copy the chips, picture, rows and readout into both pages; viser sends only what changed.

        Args:
            ctx: This plugin's context.
        """
        names = (ALL, *(e.bar for e in self._errors))
        if self._bar not in names:
            self._bar = ALL
        chosen = next((e for e in self._errors if e.bar == self._bar), None)
        key = (self._view, self._gain, self._bar)
        picture = None
        if key != self._drawn or self._moved():
            self._drawn, self._drawn_ends = key, {e.bar: e.measured for e in self._errors}
            picture = render(self._errors, self._view, float(self._gain), chosen.bar if chosen else None)
        chips = chip_line(summary_chip(self._errors))
        readout = bar_readout(chosen) if chosen is not None else frame_readout(self._errors)
        for page in self._pages:
            page["bar"].set_options(names)
            page["view"].value, page["gain"].value, page["bar"].value = self._view, self._gain, self._bar
            page["chips"].content = page["wrap"](chips)
            page["readout"].content = readout
            if picture is not None:
                page["picture"].image = picture
