"""
How well a high-rate stream gets through: its arrival rate, largest gap and delay, over the last few seconds.

Used to spot an overloaded wifi (lost best-effort samples lower the rate and open gaps) and clocks out of sync.

* Gaps come from `header.stamp` (sender time): callbacks run in batches once per tick, so arrival
  times are only good to a tick. The rate counts arrivals, where a tick of blur hardly matters.
* The delay is the smallest arrival minus stamp: network latency plus how far the monitor's clock is ahead of the
  sender's. The smallest, since a message handled right after it arrived carries no tick blur. Negative: the
  sender's clock is ahead (certain); large: its clock is behind, or the network slow (the two look the same).
! A driver that leaves the header empty gives stamp 0.0; gaps and delay are then unknown.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

#: Seconds of history the rate and the largest gap are taken over.
WINDOW = 2.0


@dataclass(frozen=True)
class StreamQuality:
    """One stream's rate, largest gap and delay over the last WINDOW seconds.

    Attributes:
        rate: Messages received per second, or None while the stream is younger than the window.
        max_gap: Largest time between consecutive sender stamps, seconds, or None if unknown
            (fewer than two messages, or unstamped).
        delay: Smallest arrival time minus sender stamp, seconds (see the module), or None if unknown.
        seen: Whether any message ever arrived.
    """

    rate: float | None
    max_gap: float | None
    delay: float | None = None
    seen: bool = True


class StreamStats:
    """Arrival times and sender stamps of one stream's recent messages."""

    def __init__(self, window: float = WINDOW):
        """Start empty.

        Args:
            window: Seconds of history kept.
        """
        self._window = window
        #: (arrival time, sender stamp) per message, oldest first, seconds.
        self._samples: deque[tuple[float, float]] = deque()
        #: Arrival time of the first message ever, seconds.
        self._first_arrival: float | None = None

    def add(self, arrival_time: float, stamp: float) -> None:
        """Note one message and forget the ones older than the window.

        Args:
            arrival_time: When its callback ran, ROS time, seconds.
            stamp: Its `header.stamp`, seconds.
        """
        if self._first_arrival is None:
            self._first_arrival = arrival_time
        self._samples.append((arrival_time, stamp))
        while self._samples[0][0] < arrival_time - self._window:
            self._samples.popleft()

    def quality(self, now: float) -> StreamQuality:
        """The rate, largest gap and delay over the window ending at `now`.

        Args:
            now: Current ROS time, seconds.
        """
        samples = [(arrival, stamp) for arrival, stamp in self._samples if arrival >= now - self._window]
        recent = [stamp for _arrival, stamp in samples]
        seen = self._first_arrival is not None
        # ? None until a full window has passed: a part-filled one would read as a low rate.
        rate = len(recent) / self._window if not seen or now - self._first_arrival >= self._window else None
        if not recent or min(recent) <= 0.0:
            return StreamQuality(rate, None, None, seen)
        delay = min(arrival - stamp for arrival, stamp in samples)
        # ? Clamped at 0: a reordered sample is no gap.
        max_gap = (max(max(later - earlier, 0.0) for earlier, later in zip(recent, recent[1:]))
                   if len(recent) >= 2 else None)
        return StreamQuality(rate, max_gap, delay, seen)
