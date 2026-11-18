"""
The old monitor's as-built records (schema 1, keys "bar_B20") brought to the new form, so one collector reads both.

- The old bar frame has its origin at the bar's start, the new one at its middle: every pose and marker is shifted.
- `actions_of_records` lists a day's actions from the records, as the old process folder cannot be read as a process.
- `old_bar_lengths` reads each bar's length from the old export's `RobotCell.json`, the only place that has it.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Mapping

import numpy as np

from ...shared.bars.pose import bar_ends

#: What the old movement names say a step did, in the new routine's words; the first match counts.
OLD_ROUTINES = (("transfer", "transfer_free"), ("insert", "insert"), ("retreat", "retreat_linear"),
                ("fix_foundation", "operator"), ("free_to_approach", "support_approach"),
                ("to_grasp", "support_linear"), ("gripper", "tool_only"))
#: A block in the pretty-printed `RobotCell.json`: its name, at this indent; a bar's is "bar_B20".
_CELL_BLOCK = re.compile(r'^ {12}"([^"]+)": \{', re.MULTILINE)
_CELL_Z = re.compile(r'"z":\s*(-?\d+\.?\d*(?:[eE]-?\d+)?)')


def is_old(record: Mapping) -> bool:
    """Whether a record has the old monitor's form."""
    return int(record.get("schema_version") or 0) < 2


def old_bar_lengths(cell_file: Path) -> dict[str, float]:
    """Each bar's length in an old export: its collision meshes' extent along z, {bar id: metres}.

    Reads the whole file (hundreds of MB) once; it is only needed for days of the old monitor.
    """
    text = Path(cell_file).read_text()
    marks = list(_CELL_BLOCK.finditer(text))
    out = {}
    for mark, after in zip(marks, [*marks[1:], None]):
        if not mark.group(1).startswith("bar_"):
            continue
        heights = [float(z) for z in _CELL_Z.findall(text, mark.end(), after.start() if after else len(text))]
        if heights:
            out[mark.group(1)[len("bar_"):]] = max(heights) - min(heights)
    return out


def _shifted(T, half: float) -> list | None:
    """A recorded 4x4 pose in the old frame as the same pose in the new one (its origin half a bar further along z)."""
    if T is None:
        return None
    moved = np.asarray(T, dtype=float).copy()
    moved[:3, 3] += moved[:3, 2] * half
    return moved.tolist()


def _sensed(markers: Mapping) -> dict:
    """The old marker dictionary {id: {pos, error}} as the new sample, ordered as the fit's `sensed_ids`."""
    ids = list(markers)
    return {"points": [markers[each]["pos"] for each in ids], "ids": [int(each) for each in ids],
            "residuals": [markers[each].get("error") for each in ids], "age": 0.0, "source": "asset", "dropped": 0,
            "occluded": 0, "notes": []}


def upgrade_entry(entry: Mapping, length: float) -> dict:
    """One old bar entry in the new form, its frames moved to the bar's middle."""
    half = length / 2.0
    key = entry.get("markerset_key")
    fit = dict(entry.get("bar_fit") or {})
    fit["T_world_from_bar"] = _shifted(fit.get("T_world_from_bar"), half)
    designed = entry.get("design_markers") or []
    points = np.asarray([marker["bar_m"] for marker in designed], dtype=float).reshape(-1, 3) - [0.0, 0.0, half]
    link = entry.get("T_link_from_bar")
    return {**{k: v for k, v in entry.items() if k != key}, "bar": f"bars/{entry.get('bar_id')}",
            "sensed": _sensed(entry.get(key) or {}), "bar_fit": fit,
            "layout": {"names": [marker["name"] for marker in designed], "points": points.tolist(), "length": length},
            "T_link_from_bar": _shifted(link, half), "T_reference": _shifted(entry.get("T_reference"), half),
            "T_assembled": _shifted(entry.get("T_assembled"), half)}


def routine_of(movement_id: str | None) -> str | None:
    """The new routine an old movement's name stands for, or None."""
    name = (movement_id or "").lower()
    return next((routine for word, routine in OLD_ROUTINES if word in name), None)


def upgrade_record(record: Mapping, lengths: Mapping[str, float]) -> dict:
    """One old record in the new form; a bar whose length is not known is left out."""
    action, step = dict(record.get("action") or {}), dict(record.get("step") or {})
    action["id"] = action.get("action_id")
    step["routine"] = routine_of(step.get("movement_id"))
    bars = {entry.get("bar_id"): upgrade_entry(entry, lengths[entry.get("bar_id")])
            for entry in (record.get("bars") or {}).values() if entry.get("bar_id") in lengths}
    return {**record, "action": action, "step": step, "bars": bars}


def actions_of_records(records: list[Mapping]) -> list[dict]:
    """The actions a day's records name, in the schedule's order, shaped as `collect.timeline` shapes them."""
    seen = {}
    for record in records:
        action = record.get("action") or {}
        if action.get("id") is not None:
            seen.setdefault(action["id"], action)
    ordered = sorted(seen.values(), key=lambda action: action.get("index") or 0)
    return [{"index": k, "action_id": action["id"], "kind": action.get("kind") or "?",
             "bar_id": action.get("bar_id"), "robot": (action.get("robot") or "").rsplit("/", 1)[-1].capitalize(),
             "exists": [], "held": {}} for k, action in enumerate(ordered)]


def seats_of_records(records: list[Mapping], lengths: Mapping[str, float]) -> dict[str, dict]:
    """Each bar's seat and length from the upgraded records themselves (the first record that names it)."""
    seats: dict[str, dict] = {}
    for record in records:
        for bar_id, entry in (record.get("bars") or {}).items():
            T = entry.get("T_assembled")
            if bar_id in seats or T is None:
                continue
            T = np.asarray(T, dtype=float)
            seats[bar_id] = {"T": T, "length": lengths[bar_id], "ends": bar_ends(T, lengths[bar_id]).tolist()}
    return seats
