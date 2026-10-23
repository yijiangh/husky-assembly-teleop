"""
The console: a window at the bottom right of the page with the newest log lines, newest first.

It starts minimized to its bar; the control at the bar's right end opens it.

- Show "Plugins" (the default) leaves out health's colour changes and the late-tick reports, "Warnings" shows warnings
  and errors from every source but the ticks, "All" every line. Errors always show. Source narrows to one source.
- The list is rewritten when a line comes in: pause the page (Live) to read or select it.
- stdout and the ROS log still get every line; a tick report or traceback shows in full on hover.

Loaded by default (config.DEFAULT_PLUGINS).
"""

from __future__ import annotations

import time
from html import escape

import viser
from rclpy.logging import LoggingSeverity

from ..plugin_api.console_log import TICKS, Entry
from ..plugin_api.context import PluginContext
from ..plugin_api.plugin import HuskyPlugin, register
from ..ui.style import BUSY, FAIL, INFO, NONE, chip, chip_line

SHOW_PLUGINS, SHOW_WARNINGS, SHOW_ALL = "Plugins", "Warnings", "All"
#: The Source choice that shows every source.
EVERY_SOURCE = "all"
#: Sources "Plugins" leaves out: health's colour changes (its "chips" topic) and the late-tick reports.
NOISY_SOURCES = ("health/chips", TICKS)

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
                                        hint="Plugins: what the plugins say, without health's colour changes and "
                                             "the late-tick reports. Warnings: warnings from every source but the "
                                             "ticks. All: every line. Errors always show.")
            show.on_click(ctx.defer_value("console show", self._set_show))
            self._source = gui.add_dropdown("Source", [EVERY_SOURCE], initial_value=EVERY_SOURCE)
            self._status = gui.add_html("")
            self._list = gui.add_html("")
        panel.float(x=-15, y=-15, width=WIDTH, height=HEIGHT)
        panel.minimize()

    def _set_show(self, show: str) -> None:
        self._show = show

    def draw(self, ctx: PluginContext) -> None:
        """Redraw the list and the Source choices when a line came in or a filter changed, and the status line.

        Args:
            ctx: This plugin's context.
        """
        log = ctx.console_log
        source = self._source.value
        # * The version before the lines: a line added in between only costs one more redraw, never a missed one.
        drawn = (log.version, self._show, source)
        if drawn != self._drawn:
            self._drawn = drawn
            entries = log.entries()
            sources = tuple(sorted({entry.source for entry in entries} | {source} - {EVERY_SOURCE}))
            if sources != self._sources:
                self._sources = sources
                self._source.options = [EVERY_SOURCE, *sources]
            kept = [entry for entry in entries if shown(entry, self._show, source)]
            self._hidden = len(entries) - len(kept)
            self._list.content = render(kept[-SHOWN_LINES:])
        late = log.late_share()
        self._status.content = chip_line(
            chip(f"showing {self._show.lower()}", INFO)
            + chip(f"late ticks {late:.0%}", BUSY if late > LATE_TICKS_WARN else NONE,
                   "Share of the last minute's tick periods that were late, a freeze counting for every period it "
                   "lasted; Show All lists where the time went. Often the power profile: run the monitor under "
                   "`powerprofilesctl launch -p performance`.")
            + f'<span style="font-size:11px;color:{NONE}">{self._hidden} hidden</span>')


def shown(entry: Entry, show: str, source: str) -> bool:
    """Whether the list shows `entry` with Show at `show` and Source at `source`."""
    if source != EVERY_SOURCE and entry.source != source:
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
    """One line: time, severity, source and the message's first line; the whole message on hover."""
    stamp = time.strftime("%H:%M:%S", time.localtime(entry.time)) + f".{int(entry.time * 10) % 10}"
    name, color = LEVEL_STYLES.get(entry.level, ("?", NONE))
    first, *rest = entry.text.splitlines() or [""]
    more = f' <span class="dim">(+{len(rest)} lines)</span>' if rest else ""
    count = f'<span class="count">×{entry.count}</span>' if entry.count > 1 else ""
    text_color = color if entry.level >= LoggingSeverity.WARN else (
        "var(--mantine-color-dimmed)" if entry.level < LoggingSeverity.INFO else "inherit")
    return (f'<div class="line" title="{escape(entry.text, quote=True)}"><span class="dim">{stamp}</span>'
            f'<span style="color:{color};font-weight:600">{name}</span>'
            f'<span class="text" style="font-weight:600">{escape(entry.source)}</span>'
            f'<span class="text" style="color:{text_color}">{escape(first)}{more}{count}</span></div>')
