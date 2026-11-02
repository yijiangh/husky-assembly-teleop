"""
The relay's frame log: every `~/frame_metrics` sample as one JSON line, a file per day, kept a number of days.

Files are `<folder>/<YYYYMMDD>.jsonl` by the look's local date, appended to; a simulation's relay writes to
`<folder>/sim/domain_<id>/`, so sim and real never mix. Day files older than `days` are deleted at the first write
and at each day change; no other file is touched.

- ! Written on the relay's executor thread: each line is flushed, never fsynced, so a look stays quick.
"""

from __future__ import annotations

import re
from datetime import date, datetime, timedelta
from pathlib import Path
from typing import TextIO

from ..drive import ros_domain

#: The relay's default frame log folder: local and per user, beside ROS's own logs, never on the Drive.
DEFAULT_FOLDER = "~/.ros/mocap_frame"
#: A day file's name: the local date of its looks.
DAY_FILE = re.compile(r"(\d{8})\.jsonl")


def frame_log_folder(folder: str, simulated: bool, domain: int | None = None) -> Path | None:
    """The folder a relay logs to: `folder` (`~` expanded), a simulation's below it; None when `folder` is empty.

    Args:
        folder: The `frame_log` parameter.
        simulated: Whether the relay reads a simulation (`sim`).
        domain: The simulation's ROS domain, or None for this process's.
    """
    if not folder.strip():
        return None
    root = Path(folder.strip()).expanduser()
    return root / "sim" / f"domain_{ros_domain() if domain is None else domain}" if simulated else root


class FrameLog:
    """Appends lines to the day file of their time; deletes day files older than `days`.

    Attributes:
        folder: Where the day files are; made at the first write.
        days: How many days of files are kept, today's included.
    """

    def __init__(self, folder: Path, days: int) -> None:
        """Open nothing yet: the first write makes the folder and its day file."""
        self.folder, self.days = folder, max(int(days), 1)
        self._day = ""
        self._file: TextIO | None = None

    def write(self, line: str, when: float) -> None:
        """Append `line` (one JSON object, no newline) to the day file of `when` (seconds since the epoch).

        Raises:
            OSError: If the folder or the file cannot be written; the next write tries again.
        """
        day = datetime.fromtimestamp(when).strftime("%Y%m%d")
        try:
            if self._file is None or day != self._day:
                self.close()
                self.folder.mkdir(parents=True, exist_ok=True)
                self.prune(datetime.fromtimestamp(when).date())
                self._file, self._day = (self.folder / f"{day}.jsonl").open("a", encoding="utf-8"), day
            self._file.write(line + "\n")
            self._file.flush()
        except OSError:
            self.close()
            raise

    def prune(self, today: date) -> list[Path]:
        """Delete the day files older than `days` before `today`; return those deleted."""
        oldest = today - timedelta(days=self.days - 1)
        deleted = []
        for path in self.folder.glob("*.jsonl"):
            match = DAY_FILE.fullmatch(path.name)
            try:
                old = match is not None and datetime.strptime(match.group(1), "%Y%m%d").date() < oldest
            except ValueError:
                old = False
            if old:
                path.unlink(missing_ok=True)
                deleted.append(path)
        return deleted

    def close(self) -> None:
        """Close the day file; the next write opens it again."""
        if self._file is not None:
            try:
                self._file.close()
            except OSError:
                pass
            self._file = None
