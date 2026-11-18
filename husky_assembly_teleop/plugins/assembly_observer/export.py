"""
The observer's local recordings: the days kept, dropping old ones, and copying a time range out (to the Drive).

Local layout: `<root>/<YYYYMMDD>/<stamp>_session[_sim]/` with experiment.json, events.jsonl, chunk_NNN.npz and
commands/ (`plugin.py`). A copy is `<destination>/<stamp>_session[_sim]/`, one folder per session the range touches,
with only what lies in the range: the events (from the newest states before it, and the process loaded before them),
the chunks' rows, the command files those events name; `copied_sessions` finds them.

Each reader takes `environment`: SIM or REAL sessions only, or BOTH, by their experiment.json ("environment"; a
session without one is real, `drive.marked_sim`).

! Plain files, no ROS. `copy_range` and `events_in` read whole days: run them in a process of their own
  (`scripts/observer_logs.py`); `days`, `prune` and `size` are quick enough for a worker thread.
"""

from __future__ import annotations

import json
import shutil
import time
from stat import S_ISREG
from datetime import date, datetime, timedelta
from pathlib import Path
from typing import Iterator

import numpy as np

from ...drive import REAL, SIM, marked_sim
from ...plugin_api.trace import load_arrays
from ...shared.recordings.runs import DESCRIPTION, load_description, write_description
from .names import LOADED, LOG, PROCESS, STATES

#: Where the sessions are recorded, on this PC.
LOCAL_ROOT = Path("~/.ros/assembly_observer").expanduser()
#: Where copies go, under the Drive root; ! made in Drive, never here.
RECORDING_FOLDER = Path("data_experiment/assembly_observer")
#: What a time may say instead of a clock time: the moment it is read.
NOW = "now"
#: The events file, the folder of commands sent in full, and how the chunk files are named.
EVENTS, COMMANDS, CHUNKS = "events.jsonl", "commands", "chunk_*.npz"
#: A chunk saved this long before a copy's range is not opened, seconds: a margin for clocks and late saves.
SAVED_MARGIN = 3600.0
#: Days kept locally, today included.
KEPT_DAYS = 7
#: How a day folder is named.
DAY_FORMAT = "%Y%m%d"
#: What `environment` takes besides SIM and REAL: every session.
BOTH = "both"
ENVIRONMENTS = (REAL, SIM, BOTH)


def wall_time(day: str, text: str) -> float:
    """A day (YYYYMMDD) and a time of it (HH:MM, HH:MM:SS, or NOW) as wall time, seconds since the epoch.

    Raises:
        ValueError: If either does not read.
    """
    text = text.strip()
    if text.lower() == NOW:
        return time.time()
    for form in ("%H:%M:%S", "%H:%M"):
        try:
            return datetime.strptime(f"{day} {text}", f"{DAY_FORMAT} {form}").timestamp()
        except ValueError:
            continue
    raise ValueError(f"{text!r} is not a time (HH:MM or HH:MM:SS, or {NOW})")


def days(root: Path) -> list[str]:
    """The days with recordings, newest first, as their folder names (YYYYMMDD)."""
    if not root.is_dir():
        return []
    return sorted((folder.name for folder in root.iterdir() if folder.is_dir() and _day(folder.name)), reverse=True)


def prune(root: Path, today: date, kept: int = KEPT_DAYS, keep: str = "") -> list[str]:
    """Delete the day folders older than the last `kept` days (today included), but `keep`; returns the days deleted.

    ! Only folders named as days, directly under `root`: nothing else there is touched.
    """
    oldest = today - timedelta(days=kept - 1)
    gone = [name for name in days(root) if _day(name) < oldest and name != keep]
    for name in gone:
        shutil.rmtree(root / name)
    return gone


def size(root: Path) -> int:
    """Bytes of every file under `root`; 0 without it."""
    return sum(path.stat().st_size for path in root.rglob("*") if path.is_file()) if root.is_dir() else 0


def sessions(root: Path, environment: str = BOTH) -> list[dict]:
    """Every session kept, oldest first: folder, start and end (wall time; end = last write), size, environment."""
    return [found for name in reversed(days(root))
            for folder in sorted(path for path in (root / name).iterdir() if path.is_dir())
            if (found := _session(folder)) is not None and environment in (BOTH, found["environment"])]


def copied_sessions(folder: Path, environment: str = BOTH) -> list[dict]:
    """The sessions in a copy (`copy_range`'s destination), one of its sessions, or a folder of copies; oldest first.

    As `sessions`, but a copied session starts and ends with the range it was copied from, not at its last write.
    """
    # * Down to a folder of copies: `<to>/<name>/<stamp>/<session>`.
    for pattern in (DESCRIPTION, f"*/{DESCRIPTION}", f"*/*/{DESCRIPTION}", f"*/*/*/{DESCRIPTION}"):
        found = [session for path in sorted(folder.glob(pattern)) if (session := _session(path.parent)) is not None]
        if found:
            return sorted((session for session in found if environment in (BOTH, session["environment"])),
                          key=lambda session: session["start"])
    return []


def _session(folder: Path) -> dict | None:
    """One session's folder, start, end, size and sim (`sessions`); None without a usable description."""
    try:
        description = load_description(folder)
        started = float(description["wall_time"])
    except (ValueError, KeyError, TypeError):
        return None
    stats = []
    for path in folder.rglob("*"):
        try:
            stats.append(path.stat())
        except OSError:
            continue  # * renamed or deleted meanwhile: a chunk being saved
    found = {"folder": folder, "start": started, "end": max((stat.st_mtime for stat in stats), default=started),
             "bytes": sum(stat.st_size for stat in stats if S_ISREG(stat.st_mode)),
             "environment": SIM if marked_sim(description) else REAL}
    copied = description.get("copied")
    if isinstance(copied, dict):
        try:
            found["start"], found["end"] = max(started, float(copied["from"])), float(copied["to"])
        except (KeyError, TypeError, ValueError):
            pass
    return found


def events_in(root: Path, start: float, end: float, environment: str = BOTH) -> Iterator[tuple[Path, dict]]:
    """Every event recorded from wall time `start` to `end`, session by session, read line by line."""
    for session in _sessions_touching(root, start, end, environment):
        offset = _offset(session)
        if offset is None:
            continue
        for line in _lines(session / EVENTS):
            event, when = _parsed(line)
            if event is not None and start - offset <= when <= end - offset:
                yield session, event


def copy_range(root: Path, start: float, end: float, destination: Path, about: dict,
               environment: str = BOTH) -> dict:
    """Copy what was recorded from wall time `start` to `end` (seconds since the epoch) into `destination`.

    Each session copied starts with its newest states before `start` ("state" events) and every event since them,
    each marked `"before_range": true`, so it says where things stood at `start`; the newest process loaded before
    them (its log line, `LOADED`) is kept too, for the runs and bar shapes. Everything is read line by line and
    chunk by chunk: memory stays flat however long the session. Damaged lines and chunks (a crash mid-write) are left
    out and counted.

    ! Run it in a process of its own (`scripts/observer_logs.py copy`), never in the monitor: it reads whole days.

    Args:
        root: The local recordings.
        start: The range's start, wall time.
        end: Its end, wall time.
        destination: An existing, empty folder.
        about: What the copy is, written into each session's experiment.json under "copied", e.g. its name.
        environment: Copy only SIM or REAL sessions, or BOTH.

    Returns:
        dict: Per session copied, its events, chunk rows, command files and damaged parts left out ("skipped").
    """
    copied = {}
    for session in _sessions_touching(root, start, end, environment):
        counts = _copy_session(session, start, end, destination / session.name, about)
        if counts is not None:
            copied[session.name] = counts
    return copied


def _sessions_touching(root: Path, start: float, end: float, environment: str = BOTH) -> list[Path]:
    """The session folders of the days a range touches, a day early too: a session may run past midnight."""
    first = datetime.fromtimestamp(start).date() - timedelta(days=1)
    last = datetime.fromtimestamp(end).date()
    return [folder for name in reversed(days(root)) if first <= _day(name) <= last
            for folder in sorted(path for path in (root / name).iterdir() if path.is_dir())
            if environment == BOTH or environment == (SIM if marked_sim(_description(folder)) else REAL)]


def _description(folder: Path) -> dict | None:
    """A session's experiment.json, None if it has none or it does not read."""
    try:
        return load_description(folder)
    except ValueError:
        return None


def _offset(session: Path) -> float | None:
    """Wall time minus ROS time in a session (a constant: the monitor's clock is the system clock); None if unknown."""
    try:
        description = load_description(session)
        return float(description["wall_time"]) - float(description["ros_time"])
    except (ValueError, KeyError, TypeError):
        return None


def _parsed(line: str) -> tuple[dict | None, float]:
    """An events line and its ROS time; (None, NaN) for a line that does not read (cut by a crash)."""
    try:
        event = json.loads(line)
        return event, float(event["ros_time"])
    except (ValueError, KeyError, TypeError):
        return None, float("nan")


def _copy_session(session: Path, start: float, end: float, target: Path, about: dict) -> dict | None:
    """Copy one session's part of the range into `target`; None (nothing made) when none of it lies in the range."""
    offset = _offset(session)
    if offset is None:
        return None  # * not a session, or one without a usable description
    low, high = start - offset, end - offset
    # * First pass, times only: where the newest states and process loaded before the range are, whether any event
    # * lies in it.
    since, loaded, in_range, skipped, cut = low, -1, False, 0, False
    for index, line in enumerate(_lines(session / EVENTS)):
        event, when = _parsed(line)
        # * A line that does not read counts as damaged unless it is the last: the monitor may be writing it now.
        skipped += cut
        cut = event is None
        if event is None:
            continue
        if when < low and event.get("event") in STATES:
            since = when
        elif when < low and loaded_folder(event) is not None:
            loaded = index
        elif low <= when <= high:
            in_range = True
    made = False

    def make_target() -> None:
        target.mkdir(parents=True)
        write_description(target, {**load_description(session), "copied": {**about, "from": start, "to": end}})

    rows = 0
    for chunk in sorted(session.glob(CHUNKS)):
        if chunk.stat().st_mtime < start - SAVED_MARGIN:
            continue  # * saved well before the range began: none of its ticks can lie in it
        try:
            arrays = load_arrays(chunk)
            inside = (arrays["t"] >= low) & (arrays["t"] <= high)
        except Exception:
            skipped += 1
            continue
        if inside.any():
            if not made:
                make_target()
                made = True
            ticks = len(arrays["t"])
            sliced = {key: value[inside] if _per_tick(key, value, ticks) else value for key, value in arrays.items()}
            np.savez_compressed(target / chunk.name, **sliced)
            rows += int(inside.sum())
    if not in_range and not made:
        return None
    if not made:
        make_target()
    # * Second pass: the process loaded and the lines from the newest states on, those before the range marked.
    events, files = 0, set()
    with open(target / EVENTS, "w", encoding="utf-8") as out:
        for index, line in enumerate(_lines(session / EVENTS)):
            event, when = _parsed(line)
            if event is None or not (since <= when <= high or index == loaded):
                continue
            if when < low:
                line = json.dumps({**event, "before_range": True}, allow_nan=False)
            out.write(line + "\n")
            events += 1
            if event.get("file"):
                files.add(event["file"])
    for file in sorted(files):
        if (session / file).is_file():
            (target / file).parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(session / file, target / file)
    return {"events": events, "rows": rows, "files": len(files), "skipped": skipped}


def loaded_folder(event: dict) -> Path | None:
    """The process folder an event says the monitor loaded (the process plugin's log line, `LOADED`), or None."""
    found = LOADED.match(str(event.get("text"))) if event.get("event") == LOG and \
        event.get("source") == PROCESS else None
    return Path(found.group(1)) if found else None


def _per_tick(key: str, value: np.ndarray, ticks: int) -> bool:
    """Whether an array of a chunk has one row per tick (the times and every signal), not labels or units."""
    return value.ndim > 0 and len(value) == ticks and not key.endswith((".labels", ".units"))


def _lines(path: Path) -> Iterator[str]:
    """A file's lines one by one, without their line ends; none without it."""
    if not path.is_file():
        return
    with open(path, encoding="utf-8") as lines:
        for line in lines:
            yield line.rstrip("\n")


def _day(name: str) -> date | None:
    """The date a day folder's name says, or None for another name."""
    try:
        return datetime.strptime(name, DAY_FORMAT).date()
    except ValueError:
        return None
