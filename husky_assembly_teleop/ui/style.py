"""
The shared look of plugin panels: coloured chips, section bars, number rows.

Chips are green = ok, blue = an expected end state (e.g. a stalled screw), amber = in progress or stale,
red = failed, gray = no data. Every helper returns
HTML for viser's `add_html`; assigning it in every `draw` is cheap (viser only sends changes).

! Nothing above a button may change height while a view shows (AGENTS.md, "Panels: nothing jumps"):
  - a chip that can appear, e.g. the refusal chip saying why a button did nothing (gone after REFUSAL_SHOWN), goes on
    a `chip_line`, which is always there and one line high;
  - never hide or grey out buttons: ignore and log inapplicable actions instead, with such a chip.
"""

from __future__ import annotations

from html import escape

from ..world.checks import BAD, GOOD, STALE_AFTER, WARN, Check

#: The ring that marks the current chip of a row (`swatch_chip`); no movement kind uses it.
RING = "#212529"

# Colours: chips put white text on these, readable in light and dark theme.
OK, BUSY, FAIL, NONE = "#2f9e44", "#e8590c", "#e03131", "#868e96"
INFO = "#1c7ed6"
SECTION_CTRL, SECTION_SENSOR, SECTION_TOOL = "#4263eb", "#1098ad", "#f08c00"

#: Chip colour for each severity.
LEVEL_COLORS = {GOOD: OK, WARN: BUSY, BAD: FAIL}

#: Seconds the chip saying why a button did nothing stays up.
REFUSAL_SHOWN = 3.0


def chip(text: str, color: str, title: str = "") -> str:
    """A small coloured label; `title` is hover text for extra detail."""
    # nowrap: rows wrap between chips, never inside one.
    return (f'<span title="{escape(title, quote=True)}" style="display:inline-block;white-space:nowrap;'
            f'background:{color};color:#fff;border-radius:4px;padding:0 5px;'
            f'margin:0 4px 2px 0;font-size:11px;font-weight:600">{text}</span>')


def swatch_chip(text: str, fill: str, border: str, title: str = "", fill_to: str = "", dashed: bool = False,
                current: bool = False) -> str:
    """A chip in a diagram's own colours: dark text on a light fill, with a border.

    Args:
        text: The label.
        fill: Background colour.
        border: Border colour.
        title: Hover text.
        fill_to: A second background colour the fill fades into, left to right; "" for a plain fill.
        dashed: Dash the border.
        current: Mark it as the current one of a row: a dark ring inside the border, which takes no room and is
            never cut off by its row, so nothing beside it moves when the mark moves.
    """
    background = f"linear-gradient(90deg,{fill},{fill_to})" if fill_to else fill
    ring = f"box-shadow:inset 0 0 0 2px {RING};" if current else ""
    return (f'<span title="{escape(title, quote=True)}" style="display:inline-block;white-space:nowrap;'
            f'background:{background};color:#212529;border:1px {"dashed" if dashed else "solid"} {border};{ring}'
            f'border-radius:4px;padding:0 5px;margin:0 4px 2px 0;font-size:11px;font-weight:600">{text}</span>')


def check_chip(check: Check) -> str:
    """A chip coloured by the check's level, its detail as hover text."""
    return chip(check.label, LEVEL_COLORS[check.level], check.detail)


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


def note_line(text: str) -> str:
    """Small gray text that may come and go, on one line that is always there: cut with "…", the full text on hover."""
    return block(f'<div title="{escape(text, quote=True)}" style="font-size:11px;color:{NONE};white-space:nowrap;'
                 f'overflow:hidden;text-overflow:ellipsis">{escape(text) or "—"}</div>')


def note(text: str) -> str:
    """Small gray text, for placeholders."""
    return block(f'<div style="font-size:11px;color:{NONE}">{text}</div>')


def warning(text: str, color: str = BUSY) -> str:
    """One line of warning text with a warning sign.

    Args:
        text: The warning.
        color: Amber by default; FAIL (red) when the control is blocked.
    """
    return block(f'<div style="font-size:11px;font-weight:600;color:{color}">⚠ {text}</div>')


def values(*lines: str, dim: bool = False) -> str:
    """Rows of numbers in a small monospace font.

    Args:
        lines: One string per row, already padded to fixed width.
        dim: Gray the numbers out (stale or untracked) but still show them.
    """
    rows = "".join(f"<div>{line}</div>" for line in lines)
    # ! white-space:pre keeps the padding spaces that HTML would collapse.
    color = f"color:{NONE};" if dim else ""
    return f'<div style="font:11px/1.5 monospace;white-space:pre;margin-top:2px;{color}">{rows}</div>'


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
    """Green if the last message is recent, amber if stale, gray if none yet (None)."""
    if last_update_time is None:
        return chip(label, NONE, "no data yet")
    if now - last_update_time < STALE_AFTER:
        return chip(label, OK)
    return chip(label, BUSY, f"last message {now - last_update_time:.0f}s ago")
