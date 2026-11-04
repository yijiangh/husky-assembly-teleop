"""
The shared look of plugin panels: coloured chips, section bars, number rows.

Chips are green = ok, blue = an expected end state (e.g. a stalled screw), amber = moving, in progress or stale,
red = failed, grey = no data; the colours and what else they mean are in `colors.py`. Every helper returns
HTML for viser's `add_html`; assigning it in every `draw` is cheap (viser only sends changes).

! Nothing above a button may change height while a view shows (AGENTS.md, "Panels: nothing jumps"):
  - a chip that can appear, e.g. the refusal chip saying why a button did nothing (gone after REFUSAL_SHOWN), goes on
    a `chip_line`, which is always there and one line high;
  - never hide buttons; grey one out only while a toggle can lift the block (the D-pad under Clearance), else ignore
    and log inapplicable actions, with such a chip.
! A hover text is only readable while its whole `add_html` handle stays the same: viser rebuilds the handle's HTML
  on any change, which drops the tooltip. Keep chips apart from live numbers (their own handle), and keep numbers
  that change every tick out of chip texts and titles. A chip that must change every tick says so in a comment,
  `# ! Live: …`, with where the operator reads what its hover would have said.
"""

from __future__ import annotations

from html import escape
from typing import Sequence

import viser
from viser import _messages

from ..colors import ACTION_SHADES, CURRENT, FAIL, INFO, INK, NONE, OK, WHITE
from ..colors import SECTION_CTRL, SECTION_SENSOR, SECTION_TOOL  # noqa: F401 (for the plugins' section bars)
from ..colors import WARN as BUSY
from ..world.checks import BAD, GOOD, PENDING, STALE_AFTER, WARN, Check
from ..world.checks import INFO as INFO_LEVEL

#: The ring that marks the current one of a row (the process panel's movement buttons).
RING = CURRENT

#: Chip colour for each severity.
LEVEL_COLORS = {PENDING: NONE, INFO_LEVEL: INFO, GOOD: OK, WARN: BUSY, BAD: FAIL}

#: Seconds the chip saying why a button did nothing stays up.
REFUSAL_SHOWN = 3.0
#: Longest hover text kept whole, characters (`hover`): about three lines of a chip's tooltip.
HOVER_MAX = 160


def configure_theme(gui: viser.GuiApi, control_width: str = "medium", show_logo: bool = True,
                    show_share_button: bool = True, dark_mode: bool = False) -> None:
    """Configure the viser theme with ACTION blue as its colour, the same blue in the light and the dark theme.

    ? viser 1.1.1 drops a 10-shade `brand_color` and derives its own shades from one colour, which makes the light
      theme's buttons lighter than ACTION; so the theme message is sent here, with `colors.ACTION_SHADES`.
    """
    gui._websock_interface.queue_message(_messages.ThemeConfigurationMessage(
        titlebar_content=None, control_layout="floating", control_width=control_width, show_logo=show_logo,
        show_share_button=show_share_button, dark_mode=dark_mode, colors=ACTION_SHADES))


def few(names: Sequence[str], shown: int = 3) -> str:
    """Up to `shown` names, then how many more, e.g. "A1, A2, A3 +2"."""
    return ", ".join(names[:shown]) + (f" +{len(names) - shown}" if len(names) > shown else "")


def hover(text: str, where: str = "the console") -> str:
    """Hover text of at most HOVER_MAX characters: a longer `text` is cut at a word and says where the rest is.

    Args:
        text: The full text.
        where: Where the full text can be read, e.g. "the console" (then make sure it is logged).
    """
    if len(text) <= HOVER_MAX:
        return text
    suffix = f"… full text in {where}"
    return f"{text[:HOVER_MAX - len(suffix)].rsplit(' ', 1)[0]}{suffix}"


def _title(title: str) -> str:
    """`title` as a hover attribute, cut at HOVER_MAX: a backstop; a text that may be long goes through `hover`."""
    if len(title) > HOVER_MAX:
        title = f"{title[:HOVER_MAX - 1].rsplit(' ', 1)[0]}…"
    return escape(title, quote=True)


def chip(text: str, color: str, title: str = "") -> str:
    """A small coloured label; `title` is hover text for extra detail (cut at HOVER_MAX)."""
    # nowrap: rows wrap between chips, never inside one.
    return (f'<span title="{_title(title)}" style="display:inline-block;white-space:nowrap;'
            f'background:{color};color:{WHITE};border-radius:4px;padding:0 5px;'
            f'margin:0 4px 2px 0;font-size:11px;font-weight:600">{text}</span>')


def swatch_chip(text: str, fill: str, border: str, title: str = "", fill_to: str = "", dashed: bool = False) -> str:
    """A chip in a diagram's own colours: dark text on a light fill, with a border.

    Args:
        text: The label.
        fill: Background colour.
        border: Border colour.
        title: Hover text.
        fill_to: A second background colour the fill fades into, left to right; "" for a plain fill.
        dashed: Dash the border.
    """
    background = f"linear-gradient(90deg,{fill},{fill_to})" if fill_to else fill
    return (f'<span title="{_title(title)}" style="display:inline-block;white-space:nowrap;'
            f'background:{background};color:{INK};border:1px {"dashed" if dashed else "solid"} {border};'
            f'border-radius:4px;padding:0 5px;margin:0 4px 2px 0;font-size:11px;font-weight:600">{text}</span>')


def check_chip(check: Check) -> str:
    """A chip coloured by the check's level, its value after its label, its detail as hover text."""
    text = f"{check.label} {check.value}" if check.value else check.label
    return chip(escape(text), LEVEL_COLORS[check.level], check.detail)


def section(label: str, color: str, detail: str = "") -> str:
    """A thin coloured bar with an uppercase label; `detail` is optional small gray text after it."""
    extra = (f' <span style="color:{NONE};font-weight:400;text-transform:none;'
             f'letter-spacing:0">{detail}</span>') if detail else ""
    return (f'<div style="border-left:3px solid {color};padding-left:6px;margin:6px 0 2px;'
            f'font-size:11px;font-weight:700;letter-spacing:.05em;color:{color};'
            f'text-transform:uppercase">{label}{extra}</div>')


def block(html: str) -> str:
    """Indent content to line up with the button-row labels."""
    return f'<div style="padding:0 12px">{html}</div>'


#: Lets `chip_line` keep its chips on one line, scrolling sideways when they don't fit.
CHIP_LINE_STYLE = ('<style>.chip-line {display:flex; flex-wrap:nowrap; overflow-x:auto; scrollbar-width:none; '
                   'min-height:19px} .chip-line::-webkit-scrollbar {display:none} .chip-line > span {flex:none}'
                   '</style>')

#: Put right before the tab group inside a tab shared by several plugins (Plan), so its bar shows as a path.
PATH_TABS_MARKER = '<span class="path-tabs"></span>'

#: The tab bar right after `PATH_TABS_MARKER`, not the bars inside the plugins' tabs below it.
_PATH_BAR = ('[data-dock-group] div:has(> .path-tabs) + * [data-dock-strip]'
             ':not(div:has(> .path-tabs) + * [data-dock-group] [data-dock-group] [data-dock-strip])')

#: Tabs nested in a tab look like a bar of pills, so a root tab row that wraps never reads as nested tabs: tinted
#: inside a plugin, outlined after `PATH_TABS_MARKER`, where they choose between plugins.
#: ! viser sets the dock tabs' style inline, hence `!important`; the path rules win by being more specific.
NESTED_TABS_STYLE = (
    '<style>[data-dock-group] [data-dock-group] [data-dock-strip] {background-image:none !important; '
    'background-color:var(--mantine-color-default-hover); border-radius:6px; margin:6px 8px; padding:2px; '
    'font-size:0.8em !important} '
    '[data-dock-group] [data-dock-group] [data-dock-tab] {height:2em !important; border-radius:4px; '
    'box-shadow:none !important} '
    '[data-dock-group] [data-dock-group] [data-dock-tab][aria-selected="true"] '
    '{background-color:var(--mantine-color-body) !important} '
    f'{_PATH_BAR} {{background-color:transparent; box-shadow:inset 0 0 0 1px var(--mantine-color-default-border)}} '
    f'{_PATH_BAR} > [data-dock-tab][aria-selected="true"] '
    '{background-color:var(--mantine-color-default-hover) !important}</style>')


def chip_line(html: str) -> str:
    """A line for chips that come and go: always there, even empty, and one line high, so nothing below it moves."""
    return block(f'{CHIP_LINE_STYLE}<div class="chip-line">{html}</div>')


#: The width of a chip row's name, px: every row's chips start at the same place.
ROW_NAME_WIDTH = 68


def chip_row(name: str, chips: str, wrap: bool = False) -> str:
    """A row of chips after a small grey name in a column of its own: one line high, more chips scroll sideways.

    Args:
        name: The row's name; "" leaves the column empty.
        chips: The chips' HTML.
        wrap: Let the chips wrap onto more lines instead, for rows whose chips are fixed while a step shows.
    """
    label = (f'<span title="{escape(name, quote=True)}" style="flex:0 0 {ROW_NAME_WIDTH}px;font-size:0.85em;'
             f'color:{NONE};white-space:nowrap;overflow:hidden;text-overflow:ellipsis">{escape(name)}</span>')
    body = chips if wrap else f'{CHIP_LINE_STYLE}<div class="chip-line">{chips}</div>'
    row = f'<div style="display:flex;align-items:baseline">{label}<div style="flex:1;min-width:0">{body}</div></div>'
    return f'<div style="margin-left:8px">{block(row)}</div>'


def note_line(text: str, color: str = NONE) -> str:
    """Small text that may come and go, on one line that is always there: cut with "…", the full text on hover.

    Args:
        text: The line.
        color: Gray by default; FAIL (red) only for a failure, BUSY (amber) for a warning or question.
    """
    return block(f'<div title="{escape(text, quote=True)}" style="font-size:11px;color:{color};white-space:nowrap;'
                 f'overflow:hidden;text-overflow:ellipsis">{escape(text) or "—"}</div>')


def note(text: str, color: str = NONE) -> str:
    """Small gray text, for placeholders; another `color` for a message (FAIL only for a failure)."""
    return block(f'<div style="font-size:11px;color:{color}">{text}</div>')


def warning(text: str, color: str = BUSY) -> str:
    """One line of warning text with a warning sign; the sign and a bar on the left are coloured, the text is not.

    Args:
        text: The warning.
        color: Amber by default; FAIL (red) when the control is blocked.
    """
    # Amber text is too light on a white panel, hence the plain text.
    return block(f'<div style="font-size:11px;font-weight:600;border-left:3px solid {color};padding-left:5px">'
                 f'<span style="color:{color}">⚠</span> {text}</div>')


def values(*lines: str, dim: bool = False) -> str:
    """Rows of numbers in a small monospace font.

    Args:
        lines: One string per row, already padded to fixed width.
        dim: Gray the numbers out (stale or untracked) but still show them.
    """
    rows = "".join(f"<div>{line}</div>" for line in lines)
    # ! white-space:pre keeps the padding spaces that HTML would collapse.
    color = f"color:{NONE};" if dim else ""
    return f'<div class="values" style="font:11px/1.5 monospace;white-space:pre;margin-top:2px;{color}">{rows}</div>'


def wrapped(*lines: str) -> str:
    """Sentences that wrap, with the readouts' margin."""
    return "".join(f'<div style="font-size:0.85em;margin:0 8px 4px">{escape(line)}</div>' for line in lines)


def indented(*lines: str) -> str:
    """Readout lines with a small margin from the folder's border."""
    return f'<div style="margin-left:8px">{values(*lines)}</div>' if lines else ""


def numbers(items, count: int, width: int, digits: int) -> str:
    """Signed numbers at fixed width and precision, so the text does not jump.

    Args:
        items: The numbers, or None when there is no measurement yet.
        count: How many numbers to expect (sets the placeholder count).
        width: Characters per number, sign and point included.
        digits: Digits after the point.
    """
    if items is None:
        return " ".join("—".rjust(width) for _ in range(count))
    return " ".join(f"{value:+{width}.{digits}f}" for value in items)


def freshness_chip(label: str, last_update_time: float | None, now: float) -> str:
    """Green if the last message is recent, amber if stale, gray if none yet (None); the hover text never ticks."""
    if last_update_time is None:
        return chip(label, NONE, "no data yet")
    if now - last_update_time < STALE_AFTER:
        return chip(label, OK, "messages arriving")
    return chip(label, BUSY, f"no message for over {STALE_AFTER:g} s")
