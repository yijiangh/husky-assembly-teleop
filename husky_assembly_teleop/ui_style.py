"""
The shared look of plugin panels: coloured chips, section bars, number rows.

Plugins build their panels from these, so every panel reads the same way:

    chips     green = ok / running, amber = in progress or stale,
              red = failed, gray = unknown or no data
    sections  a thin coloured bar with a short uppercase label
    numbers   small monospace rows at fixed width and precision, so text does
              not jump; a gray dash where there is no measurement yet
    actions   rows of small buttons (viser button groups); a coloured
              full-width button only for the one action that must stand out
              ! Buttons stay put and look the same whatever the state: no
                hiding (it shifts the layout) and no greying out. An action
                that does not apply right now is ignored by its handler, with
                a log line; the status chips tell the operator what is
                expected. See examples/sequence.py.

All of these return HTML strings for viser's `add_html`. Assign them to the
widget's `content` every tick in `draw`: viser only sends real changes.
"""

from __future__ import annotations

# Colours. Chips are white text on these, so they read in light and dark theme.
OK, BUSY, FAIL, NONE = "#2f9e44", "#e8590c", "#e03131", "#868e96"
SECTION_CTRL, SECTION_SENSOR, SECTION_TOOL = "#4263eb", "#1098ad", "#f08c00"

#: A measurement older than this, in seconds, is shown as stale.
STALE_AFTER = 1.0


def chip(text: str, color: str, title: str = "") -> str:
    """A small coloured label. `title` shows on hover, for detail that would not fit."""
    return (f'<span title="{title}" style="background:{color};color:#fff;border-radius:4px;'
            f'padding:0 5px;margin-right:4px;font-size:11px;font-weight:600">{text}</span>')


def section(label: str, color: str, detail: str = "") -> str:
    """A thin coloured bar with a short uppercase label, to start a section.

    `detail` is an optional word in small gray after the label, e.g. the tool kind.
    """
    extra = (f' <span style="color:{NONE};font-weight:400;text-transform:none;'
             f'letter-spacing:0">{detail}</span>') if detail else ""
    return (f'<div style="border-left:3px solid {color};padding-left:6px;margin:6px 0 2px;'
            f'font-size:11px;font-weight:700;letter-spacing:.05em;color:{color};'
            f'text-transform:uppercase">{label}{extra}</div>')


def block(html: str) -> str:
    """Indent content to line up with the labels of the button rows below it."""
    return f'<div style="padding:0 12px">{html}</div>'


def note(text: str) -> str:
    """Small gray text, for placeholders."""
    return block(f'<div style="font-size:11px;color:{NONE}">{text}</div>')


def values(*lines: str, dim: bool = False) -> str:
    """Rows of numbers in a small monospace font.

    Args:
        lines: One string per row, already padded to fixed width.
        dim: Gray the numbers out, for values that are stale or not tracked.
            They are still shown: the last known value beats no value.
    """
    rows = "".join(f"<div>{line}</div>" for line in lines)
    # ! white-space:pre keeps the padding spaces, which HTML would otherwise
    #   collapse, so columns stay put.
    color = f"color:{NONE};" if dim else ""
    return f'<div style="font:11px/1.5 monospace;white-space:pre;margin-top:2px;{color}">{rows}</div>'


def numbers(items, count: int, width: int, digits: int) -> str:
    """Signed numbers at fixed width and precision, so text does not jump around.

    Args:
        items: The numbers, or None when there is no measurement yet.
        count: How many there should be; None fills that many placeholders.
        width: Characters per number, sign and point included.
        digits: Digits after the point.
    """
    if items is None:
        return " ".join("—".rjust(width) for _ in range(count))
    return " ".join(f"{value:+{width}.{digits}f}" for value in items)


def freshness_chip(label: str, last_update_time: float, now: float) -> str:
    """Green if the last message is recent, amber if stale, gray if none came yet."""
    if last_update_time == 0.0:
        return chip(label, NONE, "no data yet")
    if now - last_update_time < STALE_AFTER:
        return chip(label, OK)
    return chip(label, BUSY, f"last message {now - last_update_time:.0f}s ago")
