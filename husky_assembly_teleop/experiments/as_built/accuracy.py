"""
The numbers of the accuracy report: one lab day's as-built records and servo files, measured and never interpreted.

Every placement number comes from `collect.shape_records`, the 3D page's own code, so the two pages agree. `read_day`
reads a day of either monitor (`old_records.py`); the other functions are pure and work on what it returns.
"""

from __future__ import annotations

import json
import re
import statistics
from datetime import datetime
from pathlib import Path
from typing import Mapping

import numpy as np

from scaffolding_assembly_core.process import read

from ...shared.runs.as_built import AS_BUILT_FOLDER, as_built_path, read_records
from .collect import FIT_RMS_TOL_MM, _three_places, assembled_bars, shape_records, timeline
from .old_records import _shifted, actions_of_records, is_old, old_bar_lengths, seats_of_records, upgrade_record

#: The kinds of step in the report's order: their titles, and how each pair of records is compared.
GROUPS = {
    "servo": ("Servo", "the servo file: the first reading to the last"),
    "wait": ("Waiting between steps", "one step's last record to the next step's first, no step running between"),
    "insert": ("Insert", "the insert's start record to its end record"),
    "retreat": ("Release and retreat", "the retreat's start record to its end record"),
    "support": ("Support robot", "the part's start record to its end record"),
    "transfer": ("Transfer", "the previous record to the transfer's end record"),
    "operator": ("By hand", "the previous record to the record once the operator confirmed"),
    "manual": ("Records taken by a click", "two records taken by hand, one after the other"),
    "other": ("Other", "the previous record to this one"),
}
GROUP_OF_ROUTINE = {"insert": "insert", "retreat_linear": "retreat", "support_approach": "support",
                    "support_linear": "support", "support_leave": "support", "tool_only": "support",
                    "transfer_free": "transfer", "transfer_held": "transfer", "operator": "operator"}
#: A bystander measured this close to its seat in its first record of the day counts as in place already.
IN_PLACE_MM = 50.0


def read_day(drive_root: Path, day: str, problem: str, process_folder: Path | None = None) -> dict:
    """Read one day's records, of either monitor, with the 3D page's numbers.

    Args:
        drive_root: The Drive root.
        day: YYYYMMDD.
        problem: The process folder's name.
        process_folder: The process folder; default `<drive>/data_design_study/<problem>`.

    Returns:
        dict: {day, problem, process_folder, source_file, old, raw (the records in the new form), actions (with their
        shaped `records`), seats, lengths}.

    Raises:
        SystemExit: The day's file holds no record.
    """
    path = as_built_path(drive_root, problem, day)
    raw = sorted(read_records(path), key=lambda record: record.get("stamp_epoch") or 0.0)
    if not raw:
        raise SystemExit(f"{path} holds no record.")
    process_folder = Path(process_folder) if process_folder else Path(drive_root) / "data_design_study" / problem
    old = is_old(raw[0])
    if old:
        lengths = old_bar_lengths(process_folder / "RobotCell.json")
        raw = [upgrade_record(record, lengths) for record in raw]
        actions, seats = actions_of_records(raw), seats_of_records(raw, lengths)
    else:
        process = read(process_folder)
        actions, seats = timeline(process), assembled_bars(process)
        lengths = {bar: seat["length"] for bar, seat in seats.items()}
    shape_records(raw, actions, seats)
    return {"day": day, "problem": problem, "process_folder": process_folder, "source_file": path, "old": old,
            "raw": raw, "actions": actions, "seats": seats, "lengths": lengths}


def flat_records(actions: list[Mapping]) -> list[tuple[Mapping, Mapping]]:
    """Every shaped record with its action, oldest first."""
    pairs = [(action, record) for action in actions for record in action.get("records", [])]
    return sorted(pairs, key=lambda pair: pair[1].get("epoch") or 0.0)


def _bar_of(record: Mapping, bar_id: str | None) -> Mapping | None:
    """A shaped record's entry of one bar, or None."""
    return next((bar for bar in record["bars"] if bar["bar"] == bar_id), None)


# * ================================================================ what each step did to the bar being built

def group_of_pair(previous: Mapping, record: Mapping) -> str | None:
    """Which kind of step lies between two shaped records of one action; None for a pair the servo files cover."""
    step = record["step"]
    if step["phase"] == "manual" or previous["step"]["phase"] == "manual":
        return "manual"
    if step["phase"] == "start":
        return "wait"
    if step.get("part") == "servo":
        return None
    return GROUP_OF_ROUTINE.get(step.get("kind"), "other")


def step_changes(actions: list[Mapping]) -> list[dict]:
    """Before, after and change of the placement error across each two neighbouring records of one action, of its bar.

    Returns:
        list[dict]: Rows {group, bar, before, after, change, from, to, movement, outcome, seconds, epoch}, oldest
        first; a pair is left out when either record has no placement (no fit, or no reference then).
    """
    rows = []
    flat = flat_records(actions)
    for (previous_action, previous), (action, record) in zip(flat, flat[1:]):
        # ! Next to each other in the day: with another action's records between them, it was not one step or a wait.
        if previous_action is not action or previous.get("run_id") != record.get("run_id"):
            continue
        group = group_of_pair(previous, record)
        before, after = _bar_of(previous, action.get("bar_id")), _bar_of(record, action.get("bar_id"))
        if group is None or before is None or after is None:
            continue
        if before.get("placement") is None or after.get("placement") is None:
            continue
        rows.append({"group": group, "bar": action["bar_id"], "before": before["placement"],
                     "after": after["placement"], "change": after["placement"] - before["placement"],
                     "from": previous["time"], "to": record["time"], "movement": record["step"]["movement_id"],
                     "outcome": record["step"]["outcome"], "epoch": record.get("epoch"),
                     "seconds": (record.get("epoch") or 0.0) - (previous.get("epoch") or 0.0)})
    return number_tries(rows)


def number_tries(rows: list[dict]) -> list[dict]:
    """Say which try each row is when a bar went through the same kind of step several times: `try` (1 up), `tries`."""
    totals: dict = {}
    for row in rows:
        totals[(row["group"], row["bar"])] = totals.get((row["group"], row["bar"]), 0) + 1
    seen: dict = {}
    for row in rows:
        key = (row["group"], row["bar"])
        seen[key] = seen.get(key, 0) + 1
        row["try"], row["tries"] = seen[key], totals[key]
    return rows


def servo_step_rows(runs: list[Mapping]) -> list[dict]:
    """The servo runs as step rows: first reading to last, for the runs whose bar was measured in both."""
    rows = []
    for run in runs:
        readings = [reading["bar_mm"] for reading in run["passes"]]
        if not readings or readings[0] is None or readings[-1] is None or not run["bar"]:
            continue
        rows.append({"group": "servo", "bar": run["bar"], "before": readings[0], "after": readings[-1],
                     "change": readings[-1] - readings[0], "from": run["time"], "to": run["time"],
                     "movement": run["movement"], "outcome": run["outcome"],
                     "epoch": run["epoch"], "seconds": None})
    return number_tries(rows)


# * ================================================================ the servo files

def servo_files(drive_root: Path, day: str, process_folder: Path, run_ids: list[str], old: bool) -> list[tuple]:
    """The day's servo files, oldest first: (path, epoch or None).

    The new monitor keeps them in `<run>/servo/<action>/<movement>.<n>.json` (their time in the run's events), the old
    one in `<day>-servoing/<movement>_servoing_data_<day>_<HHMMSS>.json` (their time in the name).
    """
    if old:
        folder = Path(drive_root) / AS_BUILT_FOLDER / f"{day}-servoing"
        found = [(path, _stamp_of_old_name(path.stem)) for path in sorted(folder.glob("*_servoing_data_*.json"))]
    else:
        found = []
        for run_id in run_ids:
            run = process_folder / "runs" / run_id
            times = _servo_times(run / "events.jsonl")
            found += [(path, times.get(str(path.relative_to(run)))) for path in sorted(run.glob("servo/*/*.json"))]
    return sorted(found, key=lambda pair: pair[1] or 0.0)


def _stamp_of_old_name(stem: str) -> float | None:
    """The time in an old servo file's name, epoch seconds."""
    try:
        return datetime.strptime("_".join(stem.rsplit("_", 2)[-2:]), "%Y%m%d_%H%M%S").timestamp()
    except ValueError:
        return None


def _servo_times(events: Path) -> dict[str, float]:
    """When a run's events say each servo file was saved: {file relative to the run: epoch}."""
    times: dict[str, float] = {}
    if not events.exists():
        return times
    for line in events.read_text().splitlines():
        try:
            event = json.loads(line)
        except ValueError:
            continue
        if event.get("event") == "servo" and event.get("file"):
            times[event["file"]] = datetime.fromisoformat(event["time"]).timestamp()
    return times


def bar_error_mm(reading: Mapping, length: float | None, old: bool) -> float | None:
    """One servo reading's bar error in the placement form, or None when the bar was not usable or has no length."""
    bar = reading.get("bar") or {}
    T_mocap, T_design = bar.get("T_mocap"), bar.get("T_design")
    if not bar.get("usable") or T_mocap is None or T_design is None or length is None:
        return None
    half = length / 2.0 if old else 0.0
    return _three_places(np.asarray(_shifted(T_mocap, half)), np.asarray(_shifted(T_design, half)), length)["worst"]


def servo_run(data: Mapping, epoch: float | None, length: float | None, old: bool, file: str = "") -> dict:
    """One servo file as a run: every reading's bar error, flange errors and wrist forces, and its outcome.

    Returns:
        dict: {file, bar, movement, kind, feedback, epoch, time, passes [{iter, bar_mm, flanges {side: mm},
        forces {side: N}, note}], corrections, outcome, reason, ended}. `ended` is the last reading's state in words,
        for an old file that names no outcome.
    """
    passes = []
    for reading in data.get("servoing_data") or []:
        bar = reading.get("bar") or {}
        passes.append({"iter": reading.get("iter"), "bar_mm": bar_error_mm(reading, length, old),
                       "flanges": {side: arm.get("pos_norm_mm") for side, arm in (reading.get("tool0") or {}).items()},
                       "forces": {side: arm.get("force_N") for side, arm in (reading.get("wrench") or {}).items()},
                       "note": "" if bar.get("usable", True) else (bar.get("reason") or "")})
    outcome = data.get("outcome") or {}
    name = data.get("bar_name") or ""
    return {"file": file, "bar": name.rsplit("/", 1)[-1].replace("bar_", ""), "movement": data.get("movement_id") or "",
            "kind": data.get("kind") or "", "feedback": data.get("servo_feedback") or "",
            "epoch": epoch, "time": datetime.fromtimestamp(epoch).strftime("%H:%M:%S") if epoch else "",
            "passes": passes, "corrections": max(len(passes) - 1, 0), "outcome": outcome.get("status"),
            "reason": outcome.get("reason") or "", "ended": "" if outcome else "outcome not in the old file",
            "dry_run": bool(data.get("dry_run"))}


def read_servo_runs(day_data: Mapping, drive_root: Path) -> list[dict]:
    """The servo runs of a day, oldest first; an unreadable file is left out with a printed note."""
    run_ids = list(dict.fromkeys(record.get("run_id") for record in day_data["raw"] if record.get("run_id")))
    runs = []
    for path, epoch in servo_files(drive_root, day_data["day"], day_data["process_folder"], run_ids, day_data["old"]):
        try:
            data = json.loads(path.read_text())
        except (OSError, ValueError) as error:
            print(f"[assembly] ! {path}: not read: {error}")
            continue
        bar = (data.get("bar_name") or "").rsplit("/", 1)[-1].replace("bar_", "")
        runs.append(servo_run(data, epoch, day_data["lengths"].get(bar), day_data["old"], path.name))
    return runs


# * ================================================================ bars already in place

def placed_motion(actions: list[Mapping]) -> dict:
    """How far each bar in place moved at each later event: the start's and the end's distance, in mm.

    A bar is in place from its release (or from its first record, when it is first seen at its seat) until an action
    builds it again. An event is a run of records of one movement of one action, measured from the record before it.

    Returns:
        dict: {bars: their ids in the order they came into place; events: [{title, action, robot, from, to, outcome,
        moved {bar: [start, end]}}]; since: {bar: {from, to, moved [start, end], error_from, error_to}}; still: of
        the pairs of neighbouring records with no step running between them, the one where the bars moved least, or
        None}.
    """
    placed: dict[str, np.ndarray] = {}
    last: dict[str, np.ndarray] = {}
    since: dict[str, dict] = {}
    events: list[dict] = []
    still = None
    previous_record, previous_key = None, None
    for action, record in flat_records(actions):
        step = record["step"]
        key = (record.get("run_id"), action["action_id"], step["movement_id"])
        # * A bar being built again is not in place any more, from this record on.
        if action.get("kind") == "J" and action.get("bar_id") in placed:
            placed.pop(action["bar_id"])
        current = {bar["bar"]: np.asarray(bar["fitted"], dtype=float) for bar in record["bars"]
                   if bar["bar"] in placed and bar.get("ok") and bar.get("fitted") is not None}
        if current:
            if key != previous_key or not events:
                events.append({"title": step["movement_id"] or action["action_id"], "action": action["action_id"],
                               "robot": record.get("robot"), "from": previous_record["time"] if previous_record else "",
                               "to": record["time"], "outcome": step["outcome"], "moved": {}, "base": dict(last)})
                previous_key = key
            event = events[-1]
            event["to"], event["outcome"] = record["time"], step["outcome"]
            event["moved"].update({bar: _ends_moved(now, event["base"].setdefault(bar, last[bar]))
                                   for bar, now in current.items()})
            if previous_record is not None and step["phase"] == "start" and previous_record["step"]["phase"] == "end":
                moved = {bar: _ends_moved(now, last[bar]) for bar, now in current.items()}
                if still is None or _worst(moved) < _worst(still["moved"]):
                    still = {"from": previous_record["time"], "to": record["time"], "moved": moved,
                             "seconds": record["epoch"] - previous_record["epoch"]}
            last.update(current)
            for bar in record["bars"]:
                if bar["bar"] in current:
                    since[bar["bar"]].update(to=record["time"], error_to=bar.get("placement"),
                                             moved=_ends_moved(current[bar["bar"]], placed[bar["bar"]]))
        previous_record = record
        # * Who is in place from the next record on.
        for bar in record["bars"]:
            bar_id = bar["bar"]
            released = bar.get("milestone") == "after_retreat" and step["outcome"] in (None, "ok")
            at_seat = (bar_id not in since and bar.get("milestone") == "bystander"
                       and (bar.get("placement_seat") or np.inf) < IN_PLACE_MM)
            if (released or at_seat) and bar.get("ok") and bar.get("fitted") is not None:
                placed[bar_id] = last[bar_id] = np.asarray(bar["fitted"], dtype=float)
                since[bar_id] = {"from": record["time"], "to": record["time"], "moved": [0.0, 0.0],
                                 "error_from": bar.get("placement"), "error_to": bar.get("placement")}
    for event in events:
        del event["base"]
    return {"bars": list(since), "events": events, "since": since, "still": still}


def _worst(moved: Mapping[str, list[float]]) -> float:
    """The largest distance any bar's end moved, mm."""
    return max(max(ends) for ends in moved.values())


def _ends_moved(now: np.ndarray, before: np.ndarray) -> list[float]:
    """How far a bar's two ends moved, mm."""
    return [float(np.linalg.norm(now[k] - before[k])) * 1e3 for k in (0, 1)]


# * ================================================================ settings and fits

def step_options(actions: list[Mapping]) -> list[dict]:
    """What each step that ended used: time, action, movement, part, outcome and the 3D page's tick boxes."""
    rows = []
    for action, record in flat_records(actions):
        if record["step"]["phase"] != "end":
            continue
        rows.append({"time": record["time"], "action": action["action_id"], "robot": record.get("robot"),
                     "movement": record["step"]["movement_id"], "part": record["step"].get("part"),
                     "routine": record["step"].get("kind"), "outcome": record["step"]["outcome"],
                     "boxes": [box for box in record["options"] if box["shown"]]})
    return rows


def settings_in_force(raw: list[Mapping]) -> list[dict]:
    """Each setting and scalar option over the day, by when it held which value: [{key, spans [{value, from, to}]}]."""
    spans: dict[str, list[dict]] = {}
    for record in raw:
        time = (record.get("stamp") or "T")[11:19]
        scalars = {k: v for k, v in (record.get("options") or {}).items() if not isinstance(v, dict)}
        for key, value in {**(record.get("settings") or {}), **scalars}.items():
            rows = spans.setdefault(key, [])
            if rows and rows[-1]["value"] == value:
                rows[-1]["to"] = time
            else:
                rows.append({"value": value, "from": time, "to": time})
    return [{"key": key, "spans": rows} for key, rows in spans.items()]


def fit_quality(raw: list[Mapping]) -> list[dict]:
    """Each bar's fits over the day: matched n of N, rms, and which records fall under N or above the rms limit.

    Returns:
        list[dict]: Per bar {bar, records, ok, failed, matched (fewest..most), design, rms_median, rms_max, under_n,
        over_rms, reasons}, by bar id.
    """
    bars: dict[str, dict] = {}
    for record in raw:
        for bar_id, entry in (record.get("bars") or {}).items():
            fit = entry.get("bar_fit") or {}
            slot = bars.setdefault(bar_id, {"bar": bar_id, "records": 0, "ok": 0, "failed": 0, "matched": [],
                                            "design": fit.get("n_design"), "rms": [], "under_n": 0, "over_rms": 0,
                                            "reasons": {}})
            slot["records"] += 1
            if not fit.get("ok"):
                slot["failed"] += 1
                reason = re.sub(r"\d+(\.\d+)?", "N", fit.get("reason") or "no reason given")
                slot["reasons"][reason] = slot["reasons"].get(reason, 0) + 1
                continue
            slot["ok"] += 1
            slot["matched"].append(fit.get("n_matched") or 0)
            slot["under_n"] += int((fit.get("n_matched") or 0) < (fit.get("n_design") or 0))
            if fit.get("rms_m") is not None:
                rms = fit["rms_m"] * 1e3
                slot["rms"].append(rms)
                slot["over_rms"] += int(rms > FIT_RMS_TOL_MM)
    out = []
    for bar_id in sorted(bars):
        slot = bars[bar_id]
        rms = slot.pop("rms")
        slot["rms_median"], slot["rms_max"] = (statistics.median(rms), max(rms)) if rms else (None, None)
        out.append(slot)
    return out


def bars_without_markers(raw: list[Mapping]) -> list[str]:
    """The bars the day's actions were about that no record lists as carrying markers."""
    # * The new records list {bar, bar_id} (some with mocap_id), the old ones "bar_B20".
    with_markers = {each["bar_id"] if isinstance(each, dict) else str(each).replace("bar_", "") for record in raw
                    for each in record.get("instrumented_bars") or []}
    acted = {(record.get("action") or {}).get("bar_id") for record in raw}
    return sorted(bar for bar in acted if bar and bar not in with_markers)


def bars_measured(raw: list[Mapping]) -> list[str]:
    """The bars the day's actions were about that have at least one good fit in the records."""
    acted = {(record.get("action") or {}).get("bar_id") for record in raw}
    return sorted(bar for bar in acted if any(((record.get("bars") or {}).get(bar, {}).get("bar_fit") or {}).get("ok")
                                              for record in raw))
