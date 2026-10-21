"""
A command a robot part did not send, and why.

Each part keeps its newest as `last_refusal`, so a panel can show it next to the button that did nothing.
"""

from __future__ import annotations

from dataclasses import dataclass

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


def refusal_from(error: MotionRefused, now: float) -> Refusal:
    """The refusal a raised MotionRefused stands for, labelled by its kind (`MotionRefused.label`)."""
    return Refusal(error.label, str(error), now)
