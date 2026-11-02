"""Coloured text for messages printed to the terminal, such as startup errors that say what to fix.

Colours follow ROS's rule (rcutils): RCUTILS_COLORIZED_OUTPUT forces them on or off, else they show on a terminal.
"""

from __future__ import annotations

import difflib
import os
import sys


def colour(code: str, text: str) -> str:
    """`text` in the ANSI colour `code` (e.g. "1;31") where the error output is coloured."""
    forced = os.environ.get("RCUTILS_COLORIZED_OUTPUT")
    on = forced == "1" if forced in ("0", "1") else sys.stderr.isatty()
    return f"\033[{code}m{text}\033[0m" if on else text


def wrong(text: str) -> str:
    """Bold red: the value that is wrong."""
    return colour("1;31", text)


def suggested(text: str) -> str:
    """Bold green: what to write instead."""
    return colour("1;32", text)


def dim(text: str) -> str:
    """Dim: a list of choices or a hint, behind the message."""
    return colour("2", text)


def did_you_mean(name: str, choices: list[str] | tuple[str, ...]) -> str:
    """A "did you mean" line for the choice closest to `name`, or "" if none is close."""
    close = difflib.get_close_matches(name, choices, n=1)
    return f"\n    did you mean   {suggested(close[0])}" if close else ""
