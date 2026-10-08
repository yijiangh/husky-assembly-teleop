"""
Named values sampled once per tick, for live plots and recordings.

A `Signal` reads a few numbers from live state; a `Trace` keeps a time series of some signals. `ctx.trace`
and `ctx.record` sample one every tick, after every plugin's update and task step, so a signal sees this
tick's measurements, link poses and commands alike. `world/signals.py` has the common ones.

! One sample per tick, stamped with the tick's ROS time: a late tick leaves a gap, and a stream faster than
  the tick shows only its newest value. For anything faster, `ros2 bag record` on the robot.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Sequence

import numpy as np

#: Default cap on a trace's samples, oldest dropped first: an hour at 20 Hz.
MAX_SAMPLES = 72_000

#: Rows a trace allocates up front; it doubles up to its cap as it fills.
_INITIAL_ROWS = 256


@dataclass(frozen=True)
class Signal:
    """A named value of one or more channels, read from live state.

    Attributes:
        name: Unique within a trace; the key in a saved file.
        read: Returns the channels (a number or a sequence of `len(channels)`), or None while unknown.
            ! Main thread only, called every tick: keep it cheap, and copy dicts edited in place.
        labels: One name per channel, e.g. ("x", "y", "z"); empty for a single channel named after the signal.
        unit: Unit of every channel, e.g. "m"; give values of different units their own signal.
    """

    name: str
    read: Callable[[], Any]
    labels: tuple[str, ...] = ()
    unit: str = ""

    @property
    def channels(self) -> tuple[str, ...]:
        """tuple[str, ...]: The channel names, at least one."""
        return self.labels or (self.name,)

    def coerce(self, value: Any) -> np.ndarray:
        """One sample as a float row: None becomes NaN.

        Raises:
            ValueError: If the value has the wrong number of channels.
        """
        width = len(self.channels)
        if value is None:
            return np.full(width, np.nan)
        row = np.asarray(value, dtype=float).reshape(-1)
        if row.size != width:
            raise ValueError(f"signal {self.name!r} gave {row.size} values for {width} channels")
        return row


class Trace:
    """A time series of some signals, filled by `sample` (from live state) or `add` (computed values).

    Attributes:
        signals: The signals, in column order.
        start: Time of the first sample since the last `clear`, or None; plots count seconds from it.
    """

    def __init__(self, signals: Sequence[Signal], max_samples: int = MAX_SAMPLES):
        """Start empty.

        Args:
            signals: What to keep; names must be unique.
            max_samples: Cap; past it the oldest sample is dropped for each new one.

        Raises:
            ValueError: If two signals share a name, one is called "t", or none are given.
        """
        names = [signal.name for signal in signals]
        if not names or len(set(names)) != len(names) or "t" in names:
            raise ValueError(f"a trace needs signals with unique names other than 't', got {names}")
        self.signals = tuple(signals)
        self.start: float | None = None
        # Column slice of each signal; column 0 is the time.
        self._columns: dict[str, slice] = {}
        column = 1
        for signal in self.signals:
            self._columns[signal.name] = slice(column, column + len(signal.channels))
            column += len(signal.channels)
        self._max_samples = max_samples
        self._rows = np.empty((min(max_samples, _INITIAL_ROWS), column))
        # A ring once full: row `_first` is the oldest. 0 until then.
        self._first = 0
        self._count = 0
        self._dropped = 0

    # --- --- --- --- --- FILLING --- --- --- --- ---

    def sample(self, t: float) -> None:
        """Read every signal now and keep the values at time `t`."""
        self.add(t, *(signal.read() for signal in self.signals))

    def add(self, t: float, *values: Any) -> None:
        """Keep one value per signal, in `signals` order, at time `t`; None means unknown.

        Raises:
            ValueError: If the count or a signal's width is wrong.
        """
        if len(values) != len(self.signals):
            raise ValueError(f"trace has {len(self.signals)} signals, got {len(values)} values")
        row = np.concatenate([[t], *(signal.coerce(value) for signal, value in zip(self.signals, values))])
        if self.start is None:
            self.start = t
        size = len(self._rows)
        if self._count == size and size < self._max_samples:
            grown = np.empty((min(2 * size, self._max_samples), self._rows.shape[1]))
            grown[:size] = self._rows
            self._rows, size = grown, len(grown)
        if self._count < size:
            self._rows[self._count] = row
            self._count += 1
        else:
            self._rows[self._first] = row
            self._first = (self._first + 1) % size
            self._dropped += 1

    def clear(self) -> None:
        """Forget every sample."""
        self.start = None
        self._first = self._count = self._dropped = 0

    # --- --- --- --- --- READING --- --- --- --- ---

    def __len__(self) -> int:
        """Samples kept."""
        return self._count

    @property
    def dropped(self) -> int:
        """int: Oldest samples dropped since the last `clear`, because the trace was full."""
        return self._dropped

    def signal(self, name: str) -> Signal:
        """The signal called `name`; KeyError if there is none."""
        if name not in self._columns:
            raise KeyError(f"trace has no signal {name!r}; it has {list(self._columns)}")
        return next(signal for signal in self.signals if signal.name == name)

    def times(self) -> np.ndarray:
        """np.ndarray: Sample times, seconds, oldest first. A copy."""
        return self._ordered(slice(0, 1))[:, 0]

    def values(self, name: str) -> np.ndarray:
        """np.ndarray: One signal's samples, (samples x channels), oldest first. A copy."""
        self.signal(name)
        return self._ordered(self._columns[name])

    def latest(self, name: str) -> np.ndarray | None:
        """np.ndarray | None: One signal's newest sample, or None before the first."""
        if not self._count:
            return None
        self.signal(name)
        newest = (self._first + self._count - 1) % len(self._rows)
        return self._rows[newest, self._columns[name]].copy()

    def save(self, path: str | Path, extra: dict[str, Any] | None = None) -> Path:
        """Write every sample to an .npz file, creating its folder.

        Keys: "t" (seconds), each signal's name (samples x channels), and "<name>.labels", "<name>.unit".

        Args:
            path: The file to write.
            extra: More arrays to store, e.g. the commanded path; keys must not clash with the above.

        Returns:
            Path: The file written.
        """
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        arrays = {"t": self.times(), **(extra or {})}
        for signal in self.signals:
            arrays[signal.name] = self.values(signal.name)
            arrays[f"{signal.name}.labels"] = np.array(signal.channels)
            arrays[f"{signal.name}.unit"] = np.array(signal.unit)
        if extra and len(arrays) != 1 + 3 * len(self.signals) + len(extra):
            raise ValueError(f"extra keys {sorted(extra)} clash with the trace's own")
        np.savez(path, **arrays)
        return path

    def _ordered(self, columns: slice) -> np.ndarray:
        """The given columns of every kept row, oldest first."""
        if self._first == 0:
            return self._rows[:self._count, columns].copy()
        return np.concatenate((self._rows[self._first:, columns], self._rows[:self._first, columns]))
