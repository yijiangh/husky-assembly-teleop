"""
Recording every message of a stream at the full ROS rate.

! EXPERIMENTAL: not decided yet. This taps raw ROS messages only. The aim is
  that any interface state (joint positions, TCP pose, a mocap fix, a derived
  value) can be asked for at its full update rate just as easily, so expect
  this API to change.

A `SampleSource` is fed with each raw message by its subscription callback;
`record(source)` collects them, stamped by the sender, for as long as its `with` block runs.
"""

from __future__ import annotations

from typing import Any, Callable

import numpy as np

#: Default cap on one recording's samples, so a forgotten recording cannot grow forever.
MAX_SAMPLES = 1_000_000


class SampleSource:
    """Every raw message of one stream, passed on to whoever listens.

    ! Listeners run inside the subscription callback, on the main thread. Keep them
      cheap (an append).
    """

    def __init__(self):
        """Start with no listeners."""
        self._listeners: list[Callable[[Any], None]] = []

    def emit(self, message: Any) -> None:
        """Hand one message to every listener.

        Args:
            message: The raw ROS message.
        """
        for listener in list(self._listeners):
            listener(message)

    def listen(self, callback: Callable[[Any], None]) -> Callable[[], None]:
        """Call `callback(message)` for every message from now on.

        Args:
            callback: Called with each raw message.

        Returns:
            Callable[[], None]: A function that removes the listener again.
        """
        self._listeners.append(callback)
        return lambda: self._listeners.remove(callback) if callback in self._listeners else None


class Recording:
    """Collects `(stamp_seconds, message)` pairs from a source while a `with` block runs.

    * The stamp is the message's `header.stamp`, the sender's sample time. The node
      clock would give the drain time, and callbacks run in batches once per tick.
    ! A driver that leaves the header empty gives stamp 0.0; it is recorded as is.

    Attributes:
        samples: The `(stamp_seconds, message)` pairs, in arrival order.
        truncated: True if `max_samples` was reached and later messages were dropped.
    """

    def __init__(self, source: SampleSource, max_samples: int = MAX_SAMPLES):
        """Prepare a recording; nothing is collected until the `with` block starts.

        Args:
            source: The stream to record.
            max_samples: Stop appending after this many samples.
        """
        self._source = source
        self._max_samples = max_samples
        self._stop: Callable[[], None] | None = None
        self.samples: list[tuple[float, Any]] = []
        self.truncated = False

    def __enter__(self) -> Recording:
        """Start listening."""
        self._stop = self._source.listen(self._add)
        return self

    def __exit__(self, *exc_info) -> None:
        """Stop listening, also when the block ends by an error or a cancelled task."""
        self._stop()

    def _add(self, message: Any) -> None:
        """Keep one message with its header stamp."""
        if len(self.samples) >= self._max_samples:
            self.truncated = True
            return
        stamp = message.header.stamp
        self.samples.append((stamp.sec + stamp.nanosec * 1e-9, message))

    def stamps(self) -> np.ndarray:
        """np.ndarray: The sample times, seconds, one per sample."""
        return np.array([stamp for stamp, _ in self.samples])


def record(source: SampleSource, max_samples: int = MAX_SAMPLES) -> Recording:
    """Record `source` inside a `with` block, so it always detaches, even if cancelled.

    Args:
        source: The stream to record.
        max_samples: Stop appending after this many samples.

    Returns:
        Recording: Use as `with record(source) as rec:`.
    """
    return Recording(source, max_samples)
