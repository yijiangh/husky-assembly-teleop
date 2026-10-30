"""
The console: a window at the bottom right of the page with the newest log lines, newest first.

It starts minimized to its bar; the control at the bar's right end opens it.

- Show "Plugins" (the default) leaves out health's colour changes and the late-tick reports, "Warnings" shows warnings
  and errors from every source but the ticks, "All" every line. Errors always show. Source narrows to one source;
  Hide leaves out the sources ticked in it (DEFAULT_HIDDEN at start), whatever Show says.
- The list is rewritten when a line comes in: pause the page (Live) to read or select it.
- stdout and the ROS log still get every line; a message's first line wraps, a tick report or traceback shows in full
  on hover.

Loaded by default (config.DEFAULT_PLUGINS).
"""

from __future__ import annotations

import time
from html import escape

import viser
from rclpy.logging import LoggingSeverity

from ..plugin_api.console_log import RUNTIME, TICKS, Entry
from ..plugin_api.context import PluginContext
from ..plugin_api.plugin import HuskyPlugin, register
from ..ui.style import BUSY, FAIL, INFO, NONE, chip, chip_line

SHOW_PLUGINS, SHOW_WARNINGS, SHOW_ALL = "Plugins", "Warnings", "All"
#: The Source choice that shows every source.
EVERY_SOURCE = "all"
#: Sources "Plugins" leaves out: health's colour changes (its "chips" topic) and the late-tick reports.
NOISY_SOURCES = ("health/chips", TICKS)
#: Sources Hide leaves out at start: the late-tick reports and the runtime's own lines.
DEFAULT_HIDDEN = (TICKS, RUNTIME)
#: How Hide marks a source it leaves out, and one it shows; picking either turns it to the other.
HIDDEN_MARK, SHOWN_MARK = "☑ ", "☐ "
HIDE_HINT = ("Pick a source to hide it (☑), pick it again to show it; several can be hidden. Hidden whatever Show "
             "says; a name without a / also hides its topics (health hides health/chips)")

#: The window's size, and the height of the list in it, pixels.
WIDTH, HEIGHT, LIST_HEIGHT = 900, 360, 220
#: Lines in the list; older ones are only in stdout and the ROS log.
SHOWN_LINES = 200
#: Share of late ticks in the last minute above which their chip turns amber.
LATE_TICKS_WARN = 0.05

#: Severity -> (short name, colour).
LEVEL_STYLES = {LoggingSeverity.DEBUG: ("dbg", NONE), LoggingSeverity.INFO: ("info", INFO),
                LoggingSeverity.WARN: ("warn", BUSY), LoggingSeverity.ERROR: ("err", FAIL),
                LoggingSeverity.FATAL: ("fatal", FAIL)}

LIST_STYLE = (
    '<style>.console {overflow-y:auto; font:11.5px/1.45 ui-monospace,Menlo,monospace; padding:2px 0; '
    'border:1px solid var(--mantine-color-default-border); border-radius:4px} '
    '.console .line {display:grid; grid-template-columns:70px 36px 110px 1fr; gap:6px; padding:0 6px; '
    'white-space:nowrap} '
    '.console .line:hover {background:var(--mantine-color-default-hover)} '
    '.console .text {overflow:hidden; text-overflow:ellipsis} '
    '.console .message {white-space:pre-wrap; overflow-wrap:anywhere} '
    '.console .dim {color:var(--mantine-color-dimmed)} '
    '.console .count {margin-left:6px; padding:0 4px; border-radius:3px; font-size:10px; '
    'background:var(--mantine-color-default-hover); color:var(--mantine-color-dimmed)}</style>')


@register
class ConsolePlugin(HuskyPlugin):
    """The newest log lines in a window of their own, filtered down to what the plugins say."""

    name = "console"
    icon = viser.Icon.TERMINAL_2

    def __init__(self):
        """Start with no widgets; setup builds them."""
        self._show = SHOW_PLUGINS
        self._source: viser.GuiDropdownHandle | None = None
        self._hide: viser.GuiDropdownHandle | None = None
        #: The sources Hide leaves out.
        self._hidden_names: tuple[str, ...] = DEFAULT_HIDDEN
        self._status: viser.GuiHtmlHandle | None = None
        self._list: viser.GuiHtmlHandle | None = None
        #: The sources offered under Source, what the list was last drawn from, and how many lines it hid.
        self._sources: tuple[str, ...] = ()
        self._drawn: tuple | None = None
        self._hidden = 0

    def setup(self, ctx: PluginContext) -> None:
        """Build the window: the filters, a status line and the list.

        Args:
            ctx: This plugin's context.
        """
        panel = ctx.view.panel()
        with panel.add_tab("Console", self.icon):
            gui = ctx.view.gui
            show = gui.add_button_group("Show", [SHOW_PLUGINS, SHOW_WARNINGS, SHOW_ALL],
                                        hint="Plugins: plugin messages only. Warnings: warnings from all but ticks. "
                                             "All: everything. Errors always show.")
            show.on_click(ctx.defer_value("console show", self._set_show))
            self._source = gui.add_dropdown("Source", [EVERY_SOURCE], initial_value=EVERY_SOURCE)
            # * A dropdown whose first choice sums up what is hidden; picking a source ticks or unticks it.
            self._hide = gui.add_dropdown("Hide", [hide_summary(self._hidden_names)],
                                          initial_value=hide_summary(self._hidden_names), hint=HIDE_HINT)
            self._hide.on_update(ctx.defer_value("console hide", self._toggle_hidden))
            self._offer_hide()
            self._status = gui.add_html("")
            self._list = gui.add_html("")
        panel.float(x=-15, y=-15, width=WIDTH, height=HEIGHT)
        panel.minimize()

    def _set_show(self, show: str) -> None:
        self._show = show

    def _toggle_hidden(self, choice: str) -> None:
        """Hide the source picked under Hide, or show it again; the summary picked (or written back) changes nothing."""
        for mark in (HIDDEN_MARK, SHOWN_MARK):
            if choice.startswith(mark):
                name = choice[len(mark):]
                names = self._hidden_names
                self._hidden_names = tuple(each for each in names if each != name) if name in names else (*names, name)
                self._offer_hide()
                return

    def _offer_hide(self) -> None:
        """Write Hide's choices: the summary first (shown when closed), then every source marked hidden or shown."""
        summary = hide_summary(self._hidden_names)
        self._hide.options = [summary, *(f"{HIDDEN_MARK if name in self._hidden_names else SHOWN_MARK}{name}"
                                         for name in sorted(set(self._sources) | set(self._hidden_names)))]
        self._hide.value = summary

    def draw(self, ctx: PluginContext) -> None:
        """Redraw the list and the Source choices when a line came in or a filter changed, and the status line.

        Args:
            ctx: This plugin's context.
        """
        log = ctx.console_log
        source, hidden = self._source.value, self._hidden_names
        # * The version before the lines: a line added in between only costs one more redraw, never a missed one.
        drawn = (log.version, self._show, source, hidden)
        if drawn != self._drawn:
            self._drawn = drawn
            entries = log.entries()
            sources = tuple(sorted({entry.source for entry in entries} | {source} - {EVERY_SOURCE}))
            if sources != self._sources:
                self._sources = sources
                self._source.options = [EVERY_SOURCE, *sources]
                self._offer_hide()
            kept = [entry for entry in entries if shown(entry, self._show, source, hidden)]
            self._hidden = len(entries) - len(kept)
            self._list.content = render(kept[-SHOWN_LINES:])
        late = log.late_share()
        self._status.content = chip_line(
            chip(f"showing {self._show.lower()}", INFO, "Which lines show; Show below changes it, Hide leaves out "
                                                        "sources.")
            + chip(f"late ticks {late:.0%}", BUSY if late > LATE_TICKS_WARN else NONE,
                   "Share of the last minute's tick periods that were late, a freeze counting for every period it "
                   "lasted; Show All lists where the time went. Often the power profile: run the monitor under "
                   "`powerprofilesctl launch -p performance`.")
            + f'<span style="font-size:11px;color:{NONE}">{self._hidden} hidden</span>')


def hide_summary(hidden: tuple[str, ...]) -> str:
    """Hide's first choice, shown while it is closed: what it leaves out."""
    return f"hiding {', '.join(hidden)}" if hidden else "hiding nothing"


def shown(entry: Entry, show: str, source: str, hidden: tuple[str, ...] = ()) -> bool:
    """Whether the list shows `entry` with Show at `show`, Source at `source` and Hide naming `hidden`."""
    if source != EVERY_SOURCE and entry.source != source:
        return False
    if any(entry.source == name or entry.source.startswith(name + "/") for name in hidden):
        return False
    if entry.level >= LoggingSeverity.ERROR or show == SHOW_ALL:
        return True
    if show == SHOW_WARNINGS:
        return entry.level >= LoggingSeverity.WARN and entry.source != TICKS
    return entry.level >= LoggingSeverity.INFO and entry.source not in NOISY_SOURCES


def render(entries: list[Entry]) -> str:
    """The list, newest line on top, always LIST_HEIGHT high so nothing below it moves."""
    lines = "".join(render_line(entry) for entry in reversed(entries))
    if not lines:
        lines = '<div class="dim" style="padding:6px">nothing to show with these filters</div>'
    return f'{LIST_STYLE}<div class="console" style="height:{LIST_HEIGHT}px">{lines}</div>'


def render_line(entry: Entry) -> str:
    """One entry: time, severity, source and the whole message, wrapped, with its own line breaks; also on hover."""
    stamp = time.strftime("%H:%M:%S", time.localtime(entry.time)) + f".{int(entry.time * 10) % 10}"
    name, color = LEVEL_STYLES.get(entry.level, ("?", NONE))
    count = f'<span class="count">×{entry.count}</span>' if entry.count > 1 else ""
    text_color = color if entry.level >= LoggingSeverity.WARN else (
        "var(--mantine-color-dimmed)" if entry.level < LoggingSeverity.INFO else "inherit")
    return (f'<div class="line" title="{escape(entry.text, quote=True)}"><span class="dim">{stamp}</span>'
            f'<span style="color:{color};font-weight:600">{name}</span>'
            f'<span class="text" style="font-weight:600">{escape(entry.source)}</span>'
            f'<span class="message" style="color:{text_color}">{escape(entry.text)}{count}</span></div>')
