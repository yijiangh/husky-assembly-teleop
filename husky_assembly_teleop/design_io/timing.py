"""
Lap times of a slow job (loading, converting), summarised in one log line.
"""

from __future__ import annotations

from time import perf_counter
from typing import List, Tuple


class Stopwatch:
    """Time since start, split into named laps. Laps with the same name add up."""

    def __init__(self):
        """Start now."""
        self._start = self._last = perf_counter()
        #: (name, seconds) in the order first seen.
        self.laps: List[Tuple[str, float]] = []

    def lap(self, name: str) -> float:
        """End the current lap and name it; a name used before adds to that lap.

        Returns:
            float: Seconds of this lap.
        """
        now = perf_counter()
        seconds, self._last = now - self._last, now
        for index, (known, total) in enumerate(self.laps):
            if known == name:
                self.laps[index] = (known, total + seconds)
                return seconds
        self.laps.append((name, seconds))
        return seconds

    @property
    def total(self) -> float:
        """float: Seconds since start."""
        return perf_counter() - self._start

    def summary(self) -> str:
        """The total and every lap, e.g. "4.2 s: read design 0.1 s, cells 4.1 s"."""
        laps = ", ".join(f"{name} {seconds:.1f} s" for name, seconds in self.laps)
        return f"{self.total:.1f} s" + (f": {laps}" if laps else "")
