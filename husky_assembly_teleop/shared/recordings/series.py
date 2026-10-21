"""Time series helpers for recorded runs: samples within a time span, and a value held until the next one."""

from __future__ import annotations

import numpy as np


def within(times: np.ndarray, start: float, end: float) -> np.ndarray:
    """Which of `times` lie in [start, end], as a mask."""
    return (times >= start) & (times <= end)


def held(times: np.ndarray, values: np.ndarray, at: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The latest value sent at or before each of `at`, as a command is held until the next one.

    Args:
        times: When each value was sent, ascending.
        values: One row per time.
        at: When to look.

    Returns:
        tuple[np.ndarray, np.ndarray]: The value at each of `at` (the first one before it is sent), and which were
            sent by then.
    """
    index = np.searchsorted(times, at, side="right") - 1
    return values[np.clip(index, 0, None)], index >= 0


def finite_mean(values: np.ndarray) -> float:
    """Mean of the finite values, NaN if there are none."""
    finite = values[np.isfinite(values)]
    return float(finite.mean()) if finite.size else float("nan")


def span(times: np.ndarray) -> float:
    """Seconds from the first to the last time, 0 for fewer than two."""
    return float(times[-1] - times[0]) if len(times) > 1 else 0.0
