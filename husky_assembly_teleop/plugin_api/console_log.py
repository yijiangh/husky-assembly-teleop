"""
The monitor's newest log lines, each with its source, for the console plugin; stdout and the ROS log get them all.

Sources: a plugin's name, `RUNTIME` (the monitor), `ROBOTS` (the robot interfaces, through the node's logger) and
`TICKS` (late-tick reports).

! Add from any thread; read with `entries`.
"""

from __future__ import annotations

import os
import threading
import time
from collections import deque
from dataclasses import dataclass, replace

from rclpy.impl import rcutils_logger
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.logging import LoggingSeverity

RUNTIME, ROBOTS, TICKS = "runtime", "robots", "ticks"

#: Lines kept; older ones are only in stdout and the ROS log.
KEPT = 500

#: Seconds over which `late_share` counts.
LATE_WINDOW = 60.0


@dataclass(frozen=True)
class Entry:
    """One log line; `count` > 1 when the same message came several times in a row.

    Attributes:
        time: Wall time of its last repeat, seconds.
        level: Its `LoggingSeverity`.
        source: Who logged it: a plugin's name, `RUNTIME`, `ROBOTS` or `TICKS`.
        text: The message, without the "[source]" prefix stdout shows.
        count: How many times it came in a row.
    """

    time: float
    level: LoggingSeverity
    source: str
    text: str
    count: int = 1


class ConsoleLog:
    """The newest `KEPT` log lines, and the tick periods of the last `LATE_WINDOW` seconds, how many of them late."""

    def __init__(self):
        """Nothing logged yet."""
        self._entries: deque[Entry] = deque(maxlen=KEPT)
        #: (monotonic time, periods it stood for, how many of them late) per tick of the last LATE_WINDOW seconds,
        #: and the sums of both counts.
        self._ticks: deque[tuple[float, int, int]] = deque()
        self._periods = self._late_periods = 0
        self._lock = threading.Lock()
        #: Goes up with every line, so a view redraws only when something is new.
        self.version = 0

    def add(self, level: LoggingSeverity, source: str, text: str) -> None:
        """Add a line; the same line right after itself only raises the last one's count."""
        now = time.time()
        with self._lock:
            last = self._entries[-1] if self._entries else None
            if last is not None and (last.level, last.source, last.text) == (level, source, text):
                self._entries[-1] = replace(last, time=now, count=last.count + 1)
            else:
                self._entries.append(Entry(now, level, source, text))
            self.version += 1

    def entries(self) -> list[Entry]:
        """The kept lines, oldest first."""
        with self._lock:
            return list(self._entries)

    def count_tick(self, late: float, period: float) -> None:
        """Count a tick that started `late` seconds late (0 when on time), for `late_share`.

        A late tick counts as late for every period it missed and its own: a freeze weighs as long as it lasted.
        """
        now = time.monotonic()
        late_periods = 1 + int(late / period) if late > 0 else 0
        with self._lock:
            self._ticks.append((now, max(1, late_periods), late_periods))
            self._periods += max(1, late_periods)
            self._late_periods += late_periods
            self._forget(now)

    def late_share(self) -> float:
        """The share (0 to 1) of the tick periods in the last `LATE_WINDOW` seconds that were late; 0 before any."""
        with self._lock:
            self._forget(time.monotonic())
            return self._late_periods / self._periods if self._periods else 0.0

    def _forget(self, now: float) -> None:
        """Drop the ticks older than LATE_WINDOW. ! Hold the lock."""
        while self._ticks and now - self._ticks[0][0] > LATE_WINDOW:
            _, periods, late_periods = self._ticks.popleft()
            self._periods -= periods
            self._late_periods -= late_periods


class ConsoleLogger(RcutilsLogger):
    """The node's logger, which also adds every line it logs to a ConsoleLog under one source."""

    def __init__(self, name: str, source: str, log: ConsoleLog):
        """Log under the node's logger `name`."""
        super().__init__(name)
        self._source = source
        self._console = log

    def log(self, message, severity, **kwargs) -> bool:
        """Log as rclpy does, then keep the line if it was logged."""
        logged = super().log(message, severity, **kwargs)
        if logged:
            self._console.add(LoggingSeverity(severity), self._source, str(message))
        return logged


# * rclpy keys its throttle and once filters by the first caller outside its own files: count this one as its own,
#   so the filters still see the real caller.
rcutils_logger._internal_callers.append(os.path.realpath(__file__))
