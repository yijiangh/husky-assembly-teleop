"""
Status checks: a verdict on one thing (a sensor, a controller, a mocap fix) as data.

Drawing lives in ui/style.py (`check_chip`), so judging needs no UI.
"""

from __future__ import annotations

from dataclasses import dataclass

#: Age in seconds after which a measurement counts as stale.
STALE_AFTER = 1.0

# Severity levels of a Check, ordered so the worst is `max`. INFO is a fact to know, no problem; PENDING: nothing to
# judge yet (grey; blocks nothing). Both are below GOOD, so they never make a row or the banner worse.
PENDING, INFO, GOOD, WARN, BAD = -2, -1, 0, 1, 2


@dataclass(frozen=True)
class Check:
    """The outcome of one status check, shown as one chip (see `ui.style.check_chip`).

    Attributes:
        label: Short chip text, e.g. "mocap" or "left_ur_arm joints".
        level: INFO, GOOD, WARN or BAD; PENDING while there is nothing to judge yet.
        detail: What is wrong, or a short fact when all is well; the chip's tooltip.
        value: A short figure after the label, e.g. "6/15" or "0.8 mm". ! Only one that changes on events, not every
            tick: a chip whose text keeps changing loses its tooltip under the pointer.
    """

    label: str
    level: int
    detail: str = ""
    value: str = ""
