"""
Which movements the executor finished in a run, read back from the run's log (`finished_moves`), e.g. after a restart.

The log, events.jsonl, is replayed in order: an `exec` event whose result is "done" adds its (step primary, movement
index) keys; a `forget` event (the executor forgetting, on Redo from here or a reopened action) removes the keys it
names, or the whole action. Records are never moved or changed: an abandoned one stays where it is, only later in the
log than the forget that outdates it.

- ! Reads files: call it on a worker thread.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Callable, Iterable

from .runs import read_events

#: An `exec` event's result when its part ran to its end. ! The executor's DONE (`movement_exec_contract`).
FINISHED = "done"
#: The executor's events: a part ended (its record named in `file`), and what it forgot.
EXEC_EVENT, FORGET_EVENT = "exec", "forget"


def finished_moves(folder: Path, log: Callable[[str], None] | None = None) -> dict[str, set[tuple[int, int]]]:
    """Every action's movements finished for real in the run at `folder`, as the executor keys them.

    Args:
        folder: The run's folder.
        log: Told about lines of the log it cannot read.

    Returns:
        dict[str, set[tuple[int, int]]]: Action id -> its (step primary, movement index) pairs still finished.
    """
    found: dict[str, set[tuple[int, int]]] = {}
    for event in read_events(folder, log):
        name, action, data = event.get("event"), event.get("action"), event.get("data") or {}
        if not action:
            continue
        if name == EXEC_EVENT and data.get("result") == FINISHED:
            found.setdefault(action, set()).update(_keys(Path(folder), event, data))
        elif name == FORGET_EVENT:
            done = data.get("done")
            if done is None:
                found.pop(action, None)
            elif action in found:
                found[action].difference_update(_pairs(done))
    return {action: keys for action, keys in found.items() if keys}


def _keys(folder: Path, event: dict, data: dict) -> set[tuple[int, int]]:
    """An `exec` event's keys: written in it, or, in older logs, read from the record it names."""
    if "keys" in data:
        return _pairs(data["keys"])
    try:
        record = json.loads((folder / event["file"]).read_text(encoding="utf-8"))
        step, part = record["step"], record["part"]
        if (record.get("outcome") or {}).get("dry_run"):
            return set()
        return {(int(step["primary"]), int(index)) for index in (part["index"], *part.get("also", ()))}
    except (KeyError, TypeError, ValueError, OSError):
        return set()


def _pairs(values: Iterable) -> set[tuple[int, int]]:
    """[[primary, index], ...] as a set of int pairs."""
    return {(int(primary), int(index)) for primary, index in values}
