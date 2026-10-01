"""
Status checks: a verdict on one thing (a sensor, a controller, a mocap fix) as data.

Judging lives here, drawing lives in ui/style.py (`check_chip`), so the world can
judge its own state without depending on how it is shown.
"""

from __future__ import annotations

from dataclasses import dataclass

#: Age in seconds after which a measurement counts as stale.
STALE_AFTER = 1.0

# Severity levels of a Check, ordered so the worst is `max`.
GOOD, WARN, BAD = 0, 1, 2


@dataclass(frozen=True)
class Check:
    """The outcome of one status check, shown as one chip (see `ui.style.check_chip`).

    Attributes:
        label: Short chip text, e.g. "mocap" or "left_ur_arm joints".
        level: GOOD, WARN or BAD.
        detail: What is wrong, or a short fact when all is well. Shown as the
            chip's tooltip.
    """

    label: str
    level: int
    detail: str = ""
