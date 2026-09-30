"""
The shared look of plugin panels: coloured chips, section bars, number rows.

Chips are green = ok, amber = in progress or stale, red = failed, gray = no data.
Every helper returns an HTML string for viser's `add_html`; assign it to the
widget's `content` each tick in `draw` (viser only sends real changes).

! Never hide or grey out buttons, since that shifts the layout. Ignore actions
  that do not apply in their handler and log it.
"""

from __future__ import annotations

from dataclasses import dataclass
from html import escape

# Colours: chips put white text on these, readable in light and dark theme.
OK, BUSY, FAIL, NONE = "#2f9e44", "#e8590c", "#e03131", "#868e96"
SECTION_CTRL, SECTION_SENSOR, SECTION_TOOL = "#4263eb", "#1098ad", "#f08c00"

#: Age in seconds after which a measurement counts as stale.
STALE_AFTER = 1.0

# Severity levels of a Check, ordered so the worst is `max`.
GOOD, WARN, BAD = 0, 1, 2
#: Chip colour for each severity.
LEVEL_COLORS = {GOOD: OK, WARN: BUSY, BAD: FAIL}


@dataclass(frozen=True)
class Check:
    """The outcome of one status check, shown as one chip (see `check_chip`).

    Attributes:
        label: Short chip text, e.g. "mocap" or "left_ur_arm joints".
        level: GOOD, WARN or BAD.
        detail: What is wrong, or a short fact when all is well. Shown as the
            chip's tooltip.
    """

    label: str
    level: int
    detail: str = ""


def chip(text: str, color: str, title: str = "") -> str:
    """A small coloured label; `title` is hover text for extra detail."""
    # inline-block + nowrap: rows wrap between chips, never inside one.
    # Hover text is escaped because it can carry any message.
    return (f'<span title="{escape(title, quote=True)}" style="display:inline-block;white-space:nowrap;'
            f'background:{color};color:#fff;border-radius:4px;padding:0 5px;'
            f'margin:0 4px 2px 0;font-size:11px;font-weight:600">{text}</span>')


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
