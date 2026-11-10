"""
What Motive streams right now, at a glance: a row per rigid body the monitor expects or the relay lists.

Expected are the robots' bases and every tracked object: `mocap_bars` tracks each bar with markers of the loaded
process as its Motive asset, so bars appear once a process is. Anything else the relay lists is "extra", added below
them and kept once seen, so no row ever moves.

- * Read-only: commands nothing, takes no lease, and subscribes to nothing of its own: it reads the bodies the robots
  and `mocap_bars` keep, so an unexpected row has no numbers.
- ! Judge a body by the capture time of its last sample, not by messages arriving: the relay keeps republishing a
  stopped body's last frame, marked invalid, for as long as it runs.
- Nothing waits on the main thread: the relay's answer lands in its state a few ticks after `ask_streaming_ids`.

Run with:  plugins:="mocap_streams"  (loads `process` and `mocap_bars`)
"""

from __future__ import annotations

from dataclasses import dataclass
from html import escape
from typing import TYPE_CHECKING, Callable

import viser

from ..config import robot_name
from ..plugin_api.context import PluginContext
from ..plugin_api.plugin import HuskyPlugin, register
from ..robot_interface.mocap_definitions import ScaledPose
from ..robot_interface.mocap_relay import MocapRelayState
from ..robot_interface.mocap_state import RELAY_SILENT_AFTER, MocapBody
from ..shared.bars.layouts import bar_layouts
from ..shared.mocap_bars_contract import MOCAP_TAB
from ..ui.phone import phone_text
from ..ui.style import BUSY, LEVEL_COLORS, NONE, block, check_chip, chip, note
from ..world.checks import BAD, INFO, PENDING, WARN, Check
from ..world.mocap import mean_marker_error, mocap_check, valid_sample_rate
from ..world.mocap_frame import volume_moved

if TYPE_CHECKING:
    from ..shared.process_contract import ProcessContract

#: The rows' states, each a chip's text, short so the table fits the panel.
STREAMING, STALE, INVALID, MISSING, UNEXPECTED = "live", "stale", "invalid", "missing", "extra"
#: Seconds between refreshes of the rows; each replaces the table's text, sent to the browser and the phone.
REFRESH_PERIOD = 1.0
#: Seconds between questions to the relay, and how old its last answer may be before it counts as not answering.
ASK_PERIOD = 2.0
ANSWER_MAX_AGE = 3 * ASK_PERIOD
#: How old the relay's last look at Motive's frame may be, seconds; it looks every 2 s by default.
LOOK_MAX_AGE = 3 * ASK_PERIOD
#: Chip colour of a row not streaming: a bar not yet in the rig is only amber, one that stopped red.
STATE_LEVELS = {STALE: BAD, INVALID: BAD, MISSING: WARN, UNEXPECTED: INFO}
#: Hover text of a chip whose state says more than `mocap_check` does.
STATE_DETAILS = {STALE: "no new sample captured: the asset stopped, Motive stopped, or clocks out of sync",
                 MISSING: "no sample since the monitor started: is the asset streaming in Motive?",
                 UNEXPECTED: "the relay lists it, nothing here expects it"}
#: Keeps the table one line per row, narrow enough for the panel.
#: ! Never in a sideways scroller: the rows' numbers change every refresh, viser then rebuilds the HTML, and the scroll
#:   jumps back to the left.
TABLE_STYLE = ('<style>.streams {border-collapse:collapse; font:11px/1.3 ui-monospace,Menlo,monospace; '
               'white-space:nowrap} .streams th {font-weight:400; color:gray} .streams th, .streams td '
               '{padding:0 5px 0 0; text-align:right} .streams .left {text-align:left} .streams .name '
               '{max-width:5em; overflow:hidden; text-overflow:ellipsis}</style>')
#: The table's columns: rigid body id, what it is, state, valid samples per second, mean marker error in mm, measured
#: markers of those in the layout, how long in this state.
COLUMNS = ('<th class="left">id</th><th class="left">what</th><th class="left">state</th><th>Hz</th>'
           '<th>mm</th><th>n/N</th><th>for</th>')


@dataclass(frozen=True)
class StreamRow:
    """One rigid body as the table shows it.

    Attributes:
        mocap_id: Its id in Motive.
        name: What it is, e.g. "Alice", "B20".
        expected: Whether the monitor expects it.
        state: One of the states above.
        level: Its chip's colour (`world.checks`).
        detail: Its chip's hover text.
        since: ROS time it entered `state`, seconds.
        rate: Valid samples per second reaching the monitor, Hz; None when nothing here subscribes to it.
        error: Mean marker error over the last second, metres, or None without a valid sample in it.
        markers: (measured now, in the layout) for a bar, measured None without a sample; None for anything else.
    """

    mocap_id: int
    name: str
    expected: bool
    state: str
    level: int
    detail: str
    since: float
    rate: float | None = None
    error: float | None = None
    markers: tuple[int | None, int] | None = None


def capture_time(body: MocapBody) -> float | None:
    """ROS time the body's last sample was captured, seconds, or None before one."""
    if body.markers is not None and body.markers.capture_time is not None:
        return body.markers.capture_time
    return body.last_update_time


def stream_state(body: MocapBody | None, listed: bool, now: float) -> str:
    """A rigid body's state now.

    Args:
        body: Its measured state if the monitor expects it, else None.
        listed: Whether the relay's last answer says it streams.
        now: Current ROS time, seconds.
    """
    if body is None:
        return UNEXPECTED if listed else STALE
    captured = capture_time(body)
    if captured is None:
        return MISSING
    if now - captured > RELAY_SILENT_AFTER or now - body.last_update_time > RELAY_SILENT_AFTER:
        return STALE
    return STREAMING if body.tracked else INVALID


def state_since(clocks: dict[int, tuple[str, float]], mocap_id: int, state: str, captured: float | None,
                now: float) -> float:
    """When a body entered `state`, ROS time, seconds; keeps it in `clocks` until the state changes.

    A stale body counts from its last capture, so a dropout noticed late, or from before this monitor, reads true.

    Args:
        clocks: Rigid body id -> (state, since); changed in place.
        mocap_id: The body.
        state: Its state now.
        captured: When its last sample was captured, or None.
        now: Current ROS time, seconds.
    """
    known = clocks.get(mocap_id)
    if known is not None and known[0] == state:
        return known[1]
    since = min(captured, now) if state == STALE and captured is not None else now
    clocks[mocap_id] = (state, since)
    return since


def stream_row(mocap_id: int, name: str, body: MocapBody | None, listed: bool, clocks: dict[int, tuple[str, float]],
               now: float, layout_markers: int | None = None) -> StreamRow:
    """One row from what is known of a rigid body now, judged as the health panel judges it.

    Args:
        mocap_id: The body.
        name: What it is.
        body: Its measured state if the monitor expects it, else None.
        listed: Whether the relay's last answer says it streams.
        clocks: As `state_since`.
        now: Current ROS time, seconds.
        layout_markers: How many markers its layout has, for a bar.
    """
    state = stream_state(body, listed, now)
    if body is None:
        level = STATE_LEVELS[UNEXPECTED] if listed else PENDING
        detail = STATE_DETAILS[UNEXPECTED] if listed else "the relay listed it earlier, not now"
        return StreamRow(mocap_id, name, False, state, level, detail, state_since(clocks, mocap_id, state, None, now))
    check = mocap_check(name, mocap_id, body, now)
    captured = capture_time(body)
    measured = int(body.markers.measured.sum()) if body.markers is not None else None
    return StreamRow(mocap_id, name, True, state,
                     check.level if state == STREAMING else STATE_LEVELS[state],
                     STATE_DETAILS.get(state, check.detail),
                     state_since(clocks, mocap_id, state, captured, now),
                     valid_sample_rate(body, now), mean_marker_error(body, now),
                     None if layout_markers is None else (measured, layout_markers))


def summary(rows: list[StreamRow]) -> tuple[int, str, str]:
    """The table in short: the worst expected level, "9 of 12 expected streaming", and "missing: B25, B34" or ""."""
    expected = [row for row in rows if row.expected]
    streaming = sum(row.state == STREAMING for row in expected)
    parts = []
    for state in (STALE, INVALID, MISSING):
        names = [row.name for row in expected if row.state == state]
        if names:
            parts.append(f"{state}: {', '.join(names)}")
    extra = [row.name for row in rows if row.state == UNEXPECTED]
    if extra:
        parts.append(f"{UNEXPECTED}: {', '.join(extra)}")
    return (max((row.level for row in expected), default=PENDING), f"{streaming} of {len(expected)} expected streaming",
            "; ".join(parts))


def relay_answers(relay: MocapRelayState, now: float) -> bool:
    """Whether the relay's last answer on what streams is recent."""
    return relay.streaming_ids_time is not None and now - relay.streaming_ids_time <= ANSWER_MAX_AGE


def relay_line(relay: MocapRelayState, now: float) -> str:
    """One line on the relay: whether it answers, its frame's status, and how far Motive's frame moved."""
    parts = [] if relay_answers(relay, now) else ["relay does not answer"]
    parts.append(f"frame {relay.frame.status}" if relay.frame is not None else "no frame announced")
    if relay.metrics_time is None or now - relay.metrics_time > LOOK_MAX_AGE:
        parts.append("no recent look")
    elif relay.fit_now is None:
        parts.append("no trusted fit")
    else:
        fit, in_use = relay.fit_now, relay.refit_now or ScaledPose()
        parts += [f"poses off {volume_moved(fit, in_use) * 1e3:.1f} mm",
                  f"moved {volume_moved(fit) * 1e3:.1f} mm since the floor calibration"]
    return "; ".join(parts)


def duration(seconds: float) -> str:
    """Seconds for people, at a scale that suits them, e.g. "20 ms", "12 s", "33 min", "2.1 h"."""
    seconds = max(seconds, 0.0)
    if seconds < 1.0:
        return f"{seconds * 1e3:.0f} ms"
    if seconds < 60.0:
        return f"{seconds:.0f} s"
    if seconds < 3600.0:
        return f"{seconds / 60.0:.0f} min"
    return f"{seconds / 3600.0:.1f} h"


def table(rows: list[StreamRow], now: float) -> str:
    """The rows as HTML, one line each; "—" where there is no number."""
    def shown(value: float | int | None, text: Callable[[float], str] = str) -> str:
        return "—" if value is None else text(value)

    lines = []
    for row in rows:
        marked = "—" if row.markers is None else f"{shown(row.markers[0])}/{row.markers[1]}"
        lines.append(
            f'<tr><td class="left">{row.mocap_id}</td><td class="left name">{escape(row.name)}</td>'
            f'<td class="left">{check_chip(Check(row.state, row.level, row.detail))}</td>'
            f'<td>{shown(row.rate, "{:.0f}".format)}</td>'
            f'<td>{shown(row.error, lambda error: f"{error * 1e3:.2f}")}</td>'
            f'<td>{marked}</td><td>{duration(now - row.since)}</td></tr>')
    return block(f'{TABLE_STYLE}<table class="streams"><tr>{COLUMNS}</tr>{"".join(lines)}</table>')


@register
class MocapStreamsPlugin(HuskyPlugin):
    """Shows the operator which rigid bodies Motive streams, which it should, and for how long each has been so."""

    name = "mocap_streams"
    tab_path = (MOCAP_TAB, "Streams")
    icon = viser.Icon.ANTENNA_BARS_5
    requires = ("process", "mocap_bars")

    def __init__(self):
        """Start with no rows; the first tick makes them."""
        self._clocks: dict[int, tuple[str, float]] = {}
        #: The ids the relay listed that nothing expected, ever: their rows stay, so the rows above never move.
        self._seen: set[int] = set()
        #: Bar name ("B20") -> how many markers its layout has, and the process they were read from.
        self._layout_markers: dict[str, int] = {}
        self._layouts_of = None
        self._asked_at = self._refreshed_at = float("-inf")
        self._summary_html = self._relay_html = self._table_html = ""

    def setup(self, ctx: PluginContext) -> None:
        """Build its tab, beside `mocap_bars`'s, and the phone's tab.

        Args:
            ctx: This plugin's context.
        """
        # * No buttons in this tab, so its lines may wrap and its rows grow.
        with ctx.view.ui() as gui:
            self._summary = gui.add_html("")
            self._relay = gui.add_html("")
            self._table = gui.add_html("")
        with ctx.view.phone("Streams", self.icon) as phone:
            self._phone = phone.add_html("")

    def update(self, ctx: PluginContext) -> None:
        """Ask the relay every ASK_PERIOD, and refresh the rows every REFRESH_PERIOD; both skip every other tick.

        Args:
            ctx: This plugin's context.
        """
        now, relay = ctx.now(), ctx.world.mocap_relay
        # * abs: ROS time can jump back (bag playback).
        if relay is not None and abs(now - self._asked_at) >= ASK_PERIOD:
            self._asked_at = now
            relay.ask_streaming_ids()
        if abs(now - self._refreshed_at) >= REFRESH_PERIOD:
            self._refreshed_at = now
            self._refresh(ctx, relay.state if relay is not None else MocapRelayState(), now)

    def _refresh(self, ctx: PluginContext, relay: MocapRelayState, now: float) -> None:
        """Rebuild the rows and their HTML from the bodies the robots and `mocap_bars` keep, and the relay's answer."""
        process: ProcessContract = ctx.require("process")
        selection = process.selection
        loaded = selection.process if selection is not None else None
        if loaded is not self._layouts_of:
            self._layouts_of = loaded
            layouts = bar_layouts(loaded) if loaded is not None else {}
            self._layout_markers = {bar.rsplit("/", 1)[-1]: len(layout.points) for bar, layout in layouts.items()}
        listed = relay.streaming_ids if relay_answers(relay, now) else frozenset()
        bodies = self._expected_bodies(ctx)
        self._seen |= listed - bodies.keys()
        self._seen -= bodies.keys()
        rows = [stream_row(mocap_id, name, body, mocap_id in listed, self._clocks, now, self._layout_markers.get(name))
                for mocap_id, (name, body) in bodies.items()]
        for mocap_id in sorted(self._seen):
            definition = relay.body_definitions.get(mocap_id)
            rows.append(stream_row(mocap_id, definition.name if definition is not None else "unknown", None,
                                   mocap_id in listed, self._clocks, now))
        level, head, rest = summary(rows)
        self._summary_html = block(chip(escape(head), LEVEL_COLORS[level])) + (note(escape(rest)) if rest else "")
        self._relay_html = note(escape(relay_line(relay, now)),
                                BUSY if relay.frame is not None and relay.frame.poses_off else NONE)
        self._table_html = table(rows, now)

    @staticmethod
    def _expected_bodies(ctx: PluginContext) -> dict[int, tuple[str, MocapBody]]:
        """Rigid body id -> (what it is, its measured state): the robots' bases, then the tracked objects."""
        bodies: dict[int, tuple[str, MocapBody]] = {}
        for serial, robot in ctx.world.robots.items():
            if robot.config.mocap_id is not None:
                bodies.setdefault(robot.config.mocap_id, (robot_name(serial), robot.base.state))
        for name, obj in ctx.world.tracked_objects.items():
            # * "fitted_bar/B20" -> "B20".
            bodies.setdefault(obj.mocap_id, (name.rsplit("/", 1)[-1], obj))
        return bodies

    def draw(self, ctx: PluginContext) -> None:
        """Copy the rows' HTML into the panel and the phone; viser sends only what changed.

        Args:
            ctx: This plugin's context.
        """
        self._summary.content = self._summary_html
        self._relay.content = self._relay_html
        self._table.content = self._table_html
        self._phone.content = phone_text(self._summary_html + self._relay_html + self._table_html)
