"""
The mocap probe's coverage map: every probe message while mapping, and the floor map smoothed from them.

Each bin weighs its samples, at most BIN_FULL, and the weights spread to the neighbours with a Gaussian of
COVERAGE_SIGMA; the map shows the mean marker error, greyer where the probe was often lost. Pure data and maths; no UI.
"""

from __future__ import annotations

import colorsys
from dataclasses import dataclass
from functools import lru_cache

import numpy as np
from scipy.ndimage import gaussian_filter

from .mocap import MARKER_ERROR_LIMIT, MARKER_ERROR_WARN

#: Coverage map: size (x, y), centred on the mocap origin, and the side of its bins, metres.
COVERAGE_SIZE = (15.0, 15.0)
COVERAGE_BIN = 0.25
#: Lower corner (x, y) of the coverage map in the lab frame, metres.
COVERAGE_ORIGIN = (-COVERAGE_SIZE[0] / 2, -COVERAGE_SIZE[1] / 2)
#: Smoothing: each bin spreads over its neighbours with a Gaussian of COVERAGE_SIGMA metres. A bin weighs at most
#: BIN_FULL samples (they come once per tick), so standing still does not drown out walking past.
COVERAGE_SIGMA = 0.3
BIN_FULL = 10
#: The map shows only where the smoothed weight reaches COVERAGE_REACH of what a lone full bin leaves on itself:
#: about 1.5 sigma around a lone bin, further along a walk.
COVERAGE_REACH = 0.3
#: Colour of coverage cells without samples.
UNCOVERED_COLOR = (200, 200, 200)
#: The map by mean marker error: GREENS from 0 to MARKER_ERROR_WARN, ORANGES from there to MARKER_ERROR_LIMIT, red
#: past it; UNTRACKED_COLOR where never tracked. Lighter than the phone map's bodies.
ERROR_GREENS = ((25, 120, 55), (150, 205, 70))
ERROR_ORANGES = ((250, 190, 40), (225, 95, 15))
ERROR_RED = (200, 30, 30)
UNTRACKED_COLOR = (120, 120, 120)

#: Columns of CoverageMap.rows: probe position x, y, z, metres; mean marker error, metres, NaN when not tracked; stamp,
#: seconds.
COVERAGE_COLUMNS = ("x", "y", "z", "error", "stamp")


@dataclass(frozen=True)
class SmoothedMap:
    """The coverage map smoothed over its bins, indexed [x bin, y bin].

    Attributes:
        seen: Whether samples are near enough to show the bin.
        tracked: Fraction of the nearby samples that were tracked.
        error: Mean marker error of the nearby tracked samples, metres; NaN where none were tracked.
    """

    seen: np.ndarray
    tracked: np.ndarray
    error: np.ndarray

    def at(self, x: float, y: float) -> tuple[float, float] | None:
        """The (tracked fraction, mean marker error) of the bin at world (x, y), or None outside or unseen."""
        i = int(np.floor((x - COVERAGE_ORIGIN[0]) / COVERAGE_BIN))
        j = int(np.floor((y - COVERAGE_ORIGIN[1]) / COVERAGE_BIN))
        if not (0 <= i < self.seen.shape[0] and 0 <= j < self.seen.shape[1]) or not self.seen[i, j]:
            return None
        return float(self.tracked[i, j]), float(self.error[i, j])


class CoverageMap:
    """Every probe message counted while mapping, one row per COVERAGE_COLUMNS, and the floor map smoothed from them.

    Each bin weighs its samples, at most BIN_FULL, and the weights spread to the neighbours with a Gaussian of
    COVERAGE_SIGMA. Samples outside the map are kept but not drawn.
    """

    def __init__(self, rows: np.ndarray | None = None):
        """Start with `rows`, or empty."""
        self._rows = np.empty((1024, len(COVERAGE_COLUMNS)))
        self.count = 0
        self._smoothed: tuple[int, SmoothedMap] | None = None
        for row in () if rows is None else rows:
            self.add(row[:3], None if np.isnan(row[3]) else row[3], row[4])

    @property
    def rows(self) -> np.ndarray:
        """The samples so far; a view, so copy it to keep it."""
        return self._rows[:self.count]

    def add(self, position, error: float | None, stamp: float) -> None:
        """Keep one mocap message: the probe position, its mean marker error (None if not tracked) and its stamp."""
        if self.count == len(self._rows):
            self._rows = np.concatenate([self._rows, np.empty_like(self._rows)])
        self._rows[self.count] = (*position[:3], np.nan if error is None else error, stamp)
        self.count += 1

    def tracked_rate(self) -> float | None:
        """The tracked fraction of every sample, or None without samples."""
        return float(np.mean(~np.isnan(self.rows[:, 3]))) if self.count else None

    def total_error(self) -> tuple[float, float] | None:
        """The (mean, standard deviation) marker error over all tracked samples, metres, or None."""
        errors = self.rows[:, 3][~np.isnan(self.rows[:, 3])]
        return (float(errors.mean()), float(errors.std())) if len(errors) else None

    def smoothed(self) -> SmoothedMap:
        """The map smoothed from the samples; computed again only after new ones."""
        if self._smoothed is None or self._smoothed[0] != self.count:
            self._smoothed = (self.count, smooth(self.rows))
        return self._smoothed[1]

    def to_json(self) -> dict:
        """dict: The samples and how the map is made from them, as written by Export."""
        rows = np.round(self.rows, 6).tolist()
        for row in rows:
            row[3] = None if np.isnan(row[3]) else row[3]
        return {"kind": "mocap_coverage", "columns": list(COVERAGE_COLUMNS),
                "map": {"origin": list(COVERAGE_ORIGIN), "size": list(COVERAGE_SIZE), "bin": COVERAGE_BIN,
                        "sigma": COVERAGE_SIGMA, "bin_full": BIN_FULL, "reach": COVERAGE_REACH},
                "samples": rows}

    @classmethod
    def from_json(cls, data: dict) -> CoverageMap:
        """The map written by `to_json`.

        Raises:
            ValueError: If `data` is not a coverage map, or one exported before the samples were kept.
        """
        if data.get("kind") != "mocap_coverage":
            raise ValueError("not a mocap coverage file (no kind: mocap_coverage)")
        if data.get("columns") != list(COVERAGE_COLUMNS):
            raise ValueError("an older coverage file with counts per cell only, no samples to smooth")
        rows = np.array([[np.nan if value is None else value for value in row] for row in data["samples"]], float)
        return cls(rows.reshape(-1, len(COVERAGE_COLUMNS)))


def smooth(rows: np.ndarray) -> SmoothedMap:
    """Kernel-smooth coverage samples (rows as CoverageMap.rows) over the map's bins."""
    shape = tuple(round(size / COVERAGE_BIN) for size in COVERAGE_SIZE)
    i = np.floor((rows[:, 0] - COVERAGE_ORIGIN[0]) / COVERAGE_BIN).astype(int)
    j = np.floor((rows[:, 1] - COVERAGE_ORIGIN[1]) / COVERAGE_BIN).astype(int)
    inside = (i >= 0) & (i < shape[0]) & (j >= 0) & (j < shape[1])
    tracked = inside & ~np.isnan(rows[:, 3])
    index = i * shape[1] + j

    def per_bin(where: np.ndarray, weights: np.ndarray | None = None) -> np.ndarray:
        """Sum `weights` (or count) of the samples `where` per bin."""
        return np.bincount(index[where], None if weights is None else weights[where],
                           minlength=shape[0] * shape[1]).reshape(shape)

    samples, hits, error_sum = per_bin(inside), per_bin(tracked), per_bin(tracked, rows[:, 3])
    with np.errstate(invalid="ignore", divide="ignore"):
        weight = np.minimum(samples, BIN_FULL) / BIN_FULL
        hit_weight = weight * np.where(samples > 0, hits / np.maximum(samples, 1), 0.0)
        mean = np.where(hits > 0, error_sum / np.maximum(hits, 1), 0.0)
        spread = [gaussian_filter(a, COVERAGE_SIGMA / COVERAGE_BIN, mode="constant")
                  for a in (weight, hit_weight, hit_weight * mean)]
        seen = spread[0] >= COVERAGE_REACH * _lone_bin_peak()
        return SmoothedMap(seen, np.where(seen, spread[1] / spread[0], 0.0),
                           np.where(seen & (spread[1] > 1e-9), spread[2] / spread[1], np.nan))


@lru_cache(maxsize=1)
def _lone_bin_peak() -> float:
    """The smoothed weight a single full bin leaves on itself."""
    size = 2 * int(np.ceil(4 * COVERAGE_SIGMA / COVERAGE_BIN)) + 1
    impulse = np.zeros((size, size))
    impulse[size // 2, size // 2] = 1.0
    return float(gaussian_filter(impulse, COVERAGE_SIGMA / COVERAGE_BIN, mode="constant")[size // 2, size // 2])


def coverage_color(rate: float | None) -> tuple[int, int, int]:
    """Red (0) through yellow to green (1) for a fraction of the cameras wanted; UNCOVERED_COLOR for None."""
    if rate is None:
        return UNCOVERED_COLOR
    return tuple(round(c * 255) for c in colorsys.hsv_to_rgb(rate / 3, 0.8, 0.85))


def error_colors(errors: np.ndarray) -> np.ndarray:
    """RGB 0-255 per mean marker error, metres, on the ERROR_ scale, UNTRACKED_COLOR for NaN; shape (..., 3)."""
    error = np.asarray(errors, dtype=float)[..., None]

    def along(band: tuple, fraction: np.ndarray) -> np.ndarray:
        low, high = np.array(band, dtype=float)
        return low + (high - low) * np.clip(fraction, 0.0, 1.0)

    with np.errstate(invalid="ignore"):
        colors = np.where(error <= MARKER_ERROR_WARN, along(ERROR_GREENS, error / MARKER_ERROR_WARN),
                          np.where(error <= MARKER_ERROR_LIMIT,
                                   along(ERROR_ORANGES, (error - MARKER_ERROR_WARN)
                                         / (MARKER_ERROR_LIMIT - MARKER_ERROR_WARN)), ERROR_RED))
    return np.rint(np.where(np.isnan(error), UNTRACKED_COLOR, colors)).astype(np.uint8)


def error_color(error: float | None) -> tuple[int, int, int]:
    """One colour of `error_colors`; UNTRACKED_COLOR for None."""
    return tuple(int(c) for c in error_colors(np.nan if error is None else error))


def map_colors(smoothed: SmoothedMap) -> np.ndarray:
    """RGB 0-255 per bin, indexed [x, y]: the error colour, greyer the more were lost; UNCOVERED_COLOR unseen."""
    tracked = smoothed.tracked[..., None]
    colors = error_colors(smoothed.error) * tracked + np.array(UNTRACKED_COLOR) * (1.0 - tracked)
    colors[~smoothed.seen] = UNCOVERED_COLOR
    return np.rint(colors).astype(np.uint8)
