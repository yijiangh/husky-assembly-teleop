"""
Log text for the relay: lines wrapped to a terminal's width, and boxes for what must not be missed.

ROS colours whole messages by severity (warnings yellow, errors red); the launch file turns that on. `good` adds
green for what went well, in info messages, which ROS leaves uncoloured. Escape codes are added only where ROS colours
too, so plain logs stay free of them.
"""

from __future__ import annotations

import os
import sys
import textwrap

#: Characters per line of a message, without ROS's `[WARN] [time] [mocap_relay]: ` prefix.
WIDTH = 100


def _colour() -> bool:
    """Whether ROS colours this process's output (rcutils' rule: forced by the variable, else a terminal)."""
    forced = os.environ.get("RCUTILS_COLORIZED_OUTPUT")
    return forced == "1" if forced in ("0", "1") else sys.stdout.isatty()


def bold(text: str) -> str:
    """Text in bold where the output is coloured; it keeps the severity colour around it."""
    return f"\033[1m{text}\033[22m" if _colour() else text


def good(text: str) -> str:
    """Text in bold green where the output is coloured: something that worked. ! Only in info messages."""
    return f"\033[1;32m{text}\033[22;39m" if _colour() else text


def wrap(text: str, width: int = WIDTH, indent: str = "  ") -> str:
    """Wrap each line of text to width, indenting the continuations; long words such as paths stay whole."""
    return "\n".join(textwrap.fill(line, width, subsequent_indent=indent, break_long_words=False,
                                   break_on_hyphens=False) or line for line in text.splitlines())


def box(title: str, text: str) -> str:
    """A bold title on the log line, and text below it in a box, so a warning stands out among routine lines."""
    lines = wrap(text, WIDTH - 2).splitlines()
    rule = "─" * WIDTH
    return "\n".join([bold(title), f"┌{rule}", *(f"│ {line}" for line in lines), f"└{rule}"])
