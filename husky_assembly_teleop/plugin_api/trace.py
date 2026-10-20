"""
Named values recorded over time, for live plots and recordings.

A `Signal` reads a few numbers, and its trigger says when: once per tick, or once per message of an interface's
`Event`. A `Trace` keeps one time series per trigger; `ctx.trace` fills it until untraced, `ctx.record` for one
`with` block. `signals.py` has the common ones.

- Tick signals: `read()` runs once per tick (`MonitorConfig.tick_period`, 20 Hz by default), after every plugin's
  update and task step, so it sees this tick's measurements, link poses and commands alike. Stamped with the tick's
  ROS time.
- Message signals: `read(record)` gets each message's record. Stamped with the record's `update_time`; its `stamp`
  is kept beside.

! A tick signal shows only the newest value of a stream faster than the tick: record such a stream on its event.
! `update_time` is when the monitor handled the message, early in the tick after it arrived, so messages queued between
  ticks get nearly the same time: time within a stream by its `stamp`.
! Faster than what reaches the monitor (the arms' driver rate): `ros2 bag record` on the robot.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable, Sequence

import numpy as np

if TYPE_CHECKING:
    from ..robot_interface.event import Event

#: Default cap on each time series' samples, oldest dropped first: an hour at 20 Hz.
MAX_SAMPLES = 72_000

#: Rows a time series allocates up front; it doubles up to its cap as it fills.
_INITIAL_ROWS = 256
#: Names a signal may not have: the keys of each time series' own columns.
_RESERVED = ("t", "stamp")
#: The file formats `Trace.save` writes, by suffix: compact binary (the default for runs), or readable text.
FORMATS = ("npz", "json")


@dataclass(frozen=True)
class Signal:
    """A named value of one or more channels, read once per tick or once per message of an event.

    Attributes:
        name: Unique among the signals of its trigger in a trace; no ".".
        read: Returns the channels (a number or a sequence of `len(channels)`), or None while unknown. A tick signal's
            takes no arguments; a message signal's gets the message's record.
            ! Main thread only, called every tick or message: keep it cheap, and copy dicts edited in place.
        labels: One name per channel, e.g. ("x", "y", "z"); empty for a single channel named after the signal.
        unit: One unit for every channel, e.g. "m", or one per channel, e.g. ("m", "m", "rad"); "" for none (a code,
            a flag, a quaternion part).
        on: The event it is read on, or None for every tick.
    """

    name: str
    read: Callable[..., Any]
    labels: tuple[str, ...] = ()
    unit: str | tuple[str, ...] = ""
    on: Event | None = None

    def __post_init__(self):
        """Check that a unit per channel has one for every channel.

        Raises:
            ValueError: If it has another count.
        """
        if not isinstance(self.unit, str) and len(self.unit) != len(self.channels):
            raise ValueError(f"signal {self.name!r} has {len(self.unit)} units for {len(self.channels)} channels")

    @property
    def channels(self) -> tuple[str, ...]:
        """tuple[str, ...]: The channel names, at least one."""
        return self.labels or (self.name,)

    @property
    def units(self) -> tuple[str, ...]:
        """tuple[str, ...]: The unit of each channel, repeats included."""
        return (self.unit,) * len(self.channels) if isinstance(self.unit, str) else tuple(self.unit)

    @property
    def key(self) -> str:
        """str: Its key in a trace and a saved file: the name, after the event's name for a message signal."""
        return self.name if self.on is None else f"{self.on.name}.{self.name}"

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
            raise ValueError(f"signal {self.key!r} gave {row.size} values for {width} channels")
        return row


class _Series:
    """One trigger's samples: rows of the time (and a message's stamp), then the signals' channels; a ring when full."""

    def __init__(self, signals: Sequence[Signal], head: tuple[str, ...], max_samples: int):
        """Start empty, with `head` (e.g. ("t", "stamp")) as the first columns."""
        self.signals = tuple(signals)
        self.head = head
        self.columns: dict[str, slice] = {name: slice(i, i + 1) for i, name in enumerate(head)}
        column = len(head)
        for signal in self.signals:
            self.columns[signal.key] = slice(column, column + len(signal.channels))
            column += len(signal.channels)
        self._max_samples = max_samples
        self._rows = np.empty((min(max_samples, _INITIAL_ROWS), column))
        # Row `_first` is the oldest once the ring is full; 0 until then.
        self._first = 0
        self.count = 0
        self.dropped = 0

    def add(self, head: Sequence[float], values: Sequence[Any]) -> None:
        """Keep one row: the head columns, then one value per signal (None is unknown)."""
        if len(values) != len(self.signals):
            raise ValueError(f"{len(self.signals)} signals, got {len(values)} values")
        row = np.concatenate([np.asarray(head, dtype=float),
                              *(signal.coerce(value) for signal, value in zip(self.signals, values))])
        size = len(self._rows)
        if self.count == size and size < self._max_samples:
            grown = np.empty((min(2 * size, self._max_samples), self._rows.shape[1]))
            grown[:size] = self._rows
            self._rows, size = grown, len(grown)
        if self.count < size:
            self._rows[self.count] = row
            self.count += 1
        else:
            self._rows[self._first] = row
            self._first = (self._first + 1) % size
            self.dropped += 1

    def clear(self) -> None:
        """Forget every row."""
        self._first = self.count = self.dropped = 0

    def ordered(self, key: str) -> np.ndarray:
        """One column group of every kept row, oldest first. A copy."""
        columns = self.columns[key]
        if self._first == 0:
            return self._rows[:self.count, columns].copy()
        return np.concatenate((self._rows[self._first:, columns], self._rows[:self._first, columns]))

    def newest(self, key: str) -> np.ndarray | None:
        """One column group of the newest row, or None before the first."""
        if not self.count:
            return None
        return self._rows[(self._first + self.count - 1) % len(self._rows), self.columns[key]].copy()


class Trace:
    """Time series of some signals, one per trigger: the tick's and each event's, each on its own time axis.

    Filled by `sample` (tick signals, from live state), `add` (tick signals, computed values) and `take` (one
    event's signals, from a message's record).

    Attributes:
        signals: The signals, in the order given.
        events: The events of its message signals, in order of first use.
        start: Time of the earliest sample since the last `clear`, or None; plots count seconds from it.
    """

    def __init__(self, signals: Sequence[Signal], max_samples: int = MAX_SAMPLES):
        """Start empty.

        Args:
            signals: What to keep, tick and message signals in any mix.
            max_samples: Cap of each trigger's samples; past it the oldest is dropped for each new one.

        Raises:
            ValueError: If none are given, a name is "t", "stamp" or has a ".", two signals share a key, or two events
                share a name.
        """
        self.signals = tuple(signals)
        keys = [signal.key for signal in self.signals]
        if not keys or len(set(keys)) != len(keys) or any(
                signal.name in _RESERVED or "." in signal.name for signal in self.signals):
            raise ValueError(f"a trace needs signals with unique keys, none named {_RESERVED} or with a '.', "
                             f"got {keys}")
        self.events = tuple(dict.fromkeys(signal.on for signal in self.signals if signal.on is not None))
        names = [event.name for event in self.events]
        if len(set(names)) != len(names) or any("." in name for name in names):
            raise ValueError(f"a trace's events need unique names without a '.', got {names}")
        self.start: float | None = None
        ticked = [signal for signal in self.signals if signal.on is None]
        # Each trigger's samples: None for the tick (only with tick signals), else the event.
        self._series: dict[Event | None, _Series] = {} if not ticked else {None: _Series(ticked, ("t",), max_samples)}
        for event in self.events:
            self._series[event] = _Series([s for s in self.signals if s.on is event], ("t", "stamp"), max_samples)

    # --- --- --- --- --- FILLING --- --- --- --- ---

    def sample(self, t: float) -> None:
        """Read every tick signal now and keep the values at time `t`. Nothing without tick signals."""
        series = self._series.get(None)
        if series is not None:
            self._keep(series, (t,), [signal.read() for signal in series.signals])

    def add(self, t: float, *values: Any) -> None:
        """Keep one value per tick signal, in `signals` order, at time `t`; None means unknown.

        Raises:
            ValueError: If the trace has no tick signals, or the count or a signal's width is wrong.
        """
        if None not in self._series:
            raise ValueError("this trace has no tick signals")
        self._keep(self._series[None], (t,), values)

    def take(self, event: Event, record: Any) -> None:
        """Read `event`'s signals from a message's record, at its `update_time` and with its `stamp`."""
        series = self._series[event]
        self._keep(series, (record.update_time, record.stamp), [signal.read(record) for signal in series.signals])

    def clear(self) -> None:
        """Forget every sample."""
        self.start = None
        for series in self._series.values():
            series.clear()

    # --- --- --- --- --- READING --- --- --- --- ---

    def __len__(self) -> int:
        """Samples kept, of every trigger."""
        return sum(series.count for series in self._series.values())

    @property
    def dropped(self) -> int:
        """int: Oldest samples dropped since the last `clear` because a time series was full, of every trigger."""
        return sum(series.dropped for series in self._series.values())

    def signal(self, key: str) -> Signal:
        """The signal with `key` (see `Signal.key`); KeyError if there is none."""
        for signal in self.signals:
            if signal.key == key:
                return signal
        raise KeyError(f"trace has no signal {key!r}; it has {[s.key for s in self.signals]}")

    def times(self, key: str | None = None) -> np.ndarray:
        """np.ndarray: Times of the samples of `key`'s trigger (the tick's if None), seconds, oldest first. A copy."""
        series = self._series.get(None if key is None else self.signal(key).on)
        return np.empty(0) if series is None else series.ordered("t")[:, 0]

    def stamps(self, key: str) -> np.ndarray:
        """np.ndarray: The message stamps of the message signal `key`'s samples, seconds, oldest first. A copy."""
        return self._series[self._event(key)].ordered("stamp")[:, 0]

    def values(self, key: str) -> np.ndarray:
        """np.ndarray: One signal's samples, (samples x channels), oldest first. A copy."""
        return self._series[self.signal(key).on].ordered(key)

    def latest(self, key: str) -> np.ndarray | None:
        """np.ndarray | None: One signal's newest sample, or None before the first."""
        return self._series[self.signal(key).on].newest(key)

    def save(self, path: str | Path, extra: dict[str, Any] | None = None) -> Path:
        """Write every sample to an .npz or a .json file, by its suffix, creating its folder.

        - .npz keys: the tick's "t", and per event "<event>.t" and "<event>.stamp" (seconds); per signal its key
          (samples x channels), "<key>.labels" and "<key>.units" (one per channel); then `extra`.
        - .json: the same, grouped: {"tick": {"t", "signals"}, "events": {<event>: {"t", "stamp", "signals"}},
          "extra"}, each signal as {"labels", "units", "values"}; unknown values are null. `load_arrays` reads both
          back to the .npz keys.

        Args:
            path: The file to write.
            extra: More arrays to store, e.g. the commanded path; keys must not clash with the above.

        Returns:
            Path: The file written.

        Raises:
            ValueError: If an extra key clashes with the trace's own, or the suffix is neither.
        """
        path = Path(path)
        if path.suffix.lstrip(".") not in FORMATS:
            raise ValueError(f"a trace is saved as {' or '.join(FORMATS)}, not {path.name}")
        arrays: dict[str, Any] = {}
        for event, series in self._series.items():
            group = "" if event is None else f"{event.name}."
            for column in series.head:
                arrays[group + column] = series.ordered(column)[:, 0]
            for signal in series.signals:
                arrays[signal.key] = series.ordered(signal.key)
                arrays[f"{signal.key}.labels"] = np.array(signal.channels)
                arrays[f"{signal.key}.units"] = np.array(signal.units)
        clash = sorted(set(extra or {}) & set(arrays))
        if clash:
            raise ValueError(f"extra keys {clash} clash with the trace's own")
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.suffix == ".npz":
            np.savez(path, **arrays, **(extra or {}))
        else:
            text = json.dumps(self._document(extra or {}), indent=1, allow_nan=False)
            # * One line per row: a list without lists or objects inside goes on one line. Only the indent's line
            #   breaks are touched: JSON strings hold none.
            text = re.sub(r"\[\n[^\[\]{}]*\]", lambda m: re.sub(r"\[\s+|\s+\]|\s*\n\s*",
                                                                lambda w: w.group(0).strip() or " ", m.group(0)), text)
            path.write_text(text + "\n")
        return path

    def _document(self, extra: dict[str, Any]) -> dict:
        """The .json form of `save`: groups by trigger, signals with their labels and units, NaN as null."""
        def signals(series: _Series) -> dict:
            return {signal.name: {"labels": list(signal.channels), "units": list(signal.units),
                                  "values": _jsonable(series.ordered(signal.key))} for signal in series.signals}

        document: dict[str, Any] = {}
        for event, series in self._series.items():
            group = {column: _jsonable(series.ordered(column)[:, 0]) for column in series.head}
            group["signals"] = signals(series)
            if event is None:
                document["tick"] = group
            else:
                document.setdefault("events", {})[event.name] = group
        document["extra"] = {key: _jsonable(value) for key, value in extra.items()}
        return document

    def _event(self, key: str) -> Event:
        """The event of the message signal `key`."""
        event = self.signal(key).on
        if event is None:
            raise KeyError(f"{key!r} is a tick signal: it has no message stamps")
        return event

    def _keep(self, series: _Series, head: tuple[float, ...], values: Sequence[Any]) -> None:
        """Add one row to `series`, and move `start` back to its time if it is the earliest."""
        series.add(head, values)
        if self.start is None or head[0] < self.start:
            self.start = head[0]


def load_arrays(path: str | Path) -> dict[str, np.ndarray]:
    """A file `Trace.save` wrote, .npz or .json, as its arrays by the .npz keys."""
    path = Path(path)
    if path.suffix == ".npz":
        with np.load(path) as data:
            return {key: data[key] for key in data.files}
    document = json.loads(path.read_text())
    arrays: dict[str, np.ndarray] = {}
    groups = [("", document["tick"])] if "tick" in document else []
    groups += [(f"{name}.", group) for name, group in document.get("events", {}).items()]
    for prefix, group in groups:
        for column in ("t", "stamp"):
            if column in group:
                arrays[prefix + column] = np.array(group[column], dtype=float)
        for name, signal in group["signals"].items():
            key = prefix + name
            arrays[key] = np.array(signal["values"], dtype=float).reshape(-1, len(signal["labels"]))
            arrays[f"{key}.labels"] = np.array(signal["labels"])
            arrays[f"{key}.units"] = np.array(signal["units"])
    for key, value in document.get("extra", {}).items():
        array = np.array(value)
        arrays[key] = np.array(value, dtype=float) if array.dtype == object else array
    return arrays


def _jsonable(value: Any) -> Any:
    """An array or number as plain JSON: lists, with NaN as None; text stays text."""
    array = np.asarray(value)
    if array.dtype.kind != "f":
        return array.tolist()
    if array.ndim == 0:
        return None if np.isnan(array) else float(array)
    plain = array.astype(object)
    plain[np.isnan(array)] = None
    return plain.tolist()
