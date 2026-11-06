"""
A command a robot part did not send, and why.

Each part keeps its newest as `last_refusal`, so a panel can show it next to the button that did nothing.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .soft_stop import MotionRefused


@dataclass(frozen=True)
class Refusal:
    """One refused command.

    Attributes:
        label: Short enough for a chip, e.g. "jog refused" or "lease refused".
        reason: The whole reason, for the chip's hover text.
        time: ROS time it was refused, seconds.
    """

    label: str
    reason: str
    time: float


def newest_reason(*parts: Any) -> str:
    """Why one of these parts just refused a command, for a panel: its newest `last_refusal`, as its log says."""
    refusals = [part.last_refusal for part in parts if part.last_refusal is not None]
    return max(refusals, key=lambda refusal: refusal.time).reason if refusals else "see the log"


def refusal_from(error: MotionRefused, now: float) -> Refusal:
    """The refusal a raised MotionRefused stands for, labelled by its kind (`MotionRefused.label`)."""
    return Refusal(error.label, str(error), now)
