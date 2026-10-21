"""
Convert base_exp and base_planner runs, once, to the layout they are saved in now (`shared/recordings/runs.py`).

1. Message logs. Recordings kept the base's message logs as raw arrays beside the trace:
   - "commands", (received, stamp, v, w, state code) per follower report, becomes "<serial>_follower.t", ".stamp",
     ".command" (v, w) and ".state". In a constant-command run (base_exp template Twist) it held the monitor's own
     sends instead; the tick signals hold every one of them too (one send per tick), so it is dropped.
   - "wheel_odometry", (received, stamp, v, w) per message, becomes "<serial>_odometry.t", ".stamp" and ".velocity".
2. Descriptions. What describes the run moves out of recording.npz into experiment.json, and the recording gets
   the robot's serial ("robot"):
   - base_exp: "experiment" (the setup) with "path_label" and "outcome" become experiment.json; "speed_caps" and
     "expected_duration" are in the setup already. The old experiment.json's "metrics" go to report.json.
   - base_planner: "outcome" is in experiment.json's run already; the oldest runs' "run" becomes experiment.json.
3. Units. Each signal's "<key>.unit" (one text, e.g. "m, rad") becomes "<key>.units" (one per channel, e.g. "m",
   "m", "rad"): split where it lists one per channel, repeated where it is one for all, from UNITS_BY_TEXT otherwise.

A recording without either is left alone, so the script can run again. Each converted file is read back and checked
before it replaces the old one: every other array unchanged, each group equal to the columns it came from, every
value moved out found in the description.

Run with the venv and ROS sourced (it imports crl_husky), on the experiments' folder:

    python3 src/husky-assembly-teleop/scripts/convert_recordings.py [--recordings <folder>] [--dry-run]
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import numpy as np

from husky_assembly_teleop.drive import drive_folder, drive_root
from husky_assembly_teleop.plugins.base_exp.twist import TWIST
from husky_assembly_teleop.shared.recordings.runs import load_description, write_description, write_report

#: The experiments' folder under the Drive root: base_exp and base_planner keep their recordings under it.
EXPERIMENTS = Path("data_experiment")
#: The old arrays, and the units of the groups made from them.
COMMANDS, ODOMETRY = "commands", "wheel_odometry"
VELOCITY = (("v", "w"), ("m/s", "rad/s"))
#: Old unit texts that do not split into one unit per channel: x, y and yaw; no unit for a flag, a 0-1 share, a code
#: or a quaternion part.
UNITS_BY_TEXT = {"m, rad": ("m", "m", "rad"), "m, quat": ("m",) * 3 + ("",) * 4,
                 "flag, 1/m, 0-1, m/s, rad/s, 0 sent 1 following 2 settling": ("", "1/m", "", "m/s", "rad/s", "")}
#: Arrays that described a run, now in its experiment.json.
DESCRIBING = ("experiment", "path_label", "outcome", "speed_caps", "expected_duration", "run")


def described(data: dict[str, np.ndarray], description: dict | None) -> tuple[dict, dict, dict | None] | None:
    """A recording's arrays without what describes the run, its new description, and its report; None if done.

    Raises:
        ValueError: If a value moved out disagrees with the description, or the serial cannot be found.
    """
    if "robot" in data:
        return None
    pose = next((key for key in data if key.endswith("_floor_pose")), None)
    if pose is None:
        raise ValueError("no <serial>_floor_pose")
    arrays = {key: value for key, value in data.items() if key not in DESCRIBING}
    arrays["robot"] = np.array(pose[:-len("_floor_pose")])
    report = None
    if "experiment" in data:
        setup = json.loads(str(data["experiment"]))
        caps = [setup["speed_caps"]["linear"], setup["speed_caps"]["angular"]]
        if caps != data["speed_caps"].tolist() or setup["expected_duration"] != float(data["expected_duration"]):
            raise ValueError("speed caps or expected duration differ from the setup")
        new = {**setup, "path_label": str(data["path_label"]), "outcome": str(data["outcome"])}
        if description is not None:
            if description.get("outcome") != new["outcome"]:
                raise ValueError("the outcome differs from experiment.json")
            report = {**new, "metrics": description["metrics"]} if "metrics" in description else None
        return arrays, new, report
    if description is None:
        if "run" not in data:
            raise ValueError("no experiment.json and no description in the recording")
        description = json.loads(str(data["run"]))
    outcome = (description.get("run") or description).get("outcome")
    if "outcome" in data and outcome != str(data["outcome"]):
        raise ValueError("the outcome differs from experiment.json")
    return arrays, description, report


def converted(data: dict[str, np.ndarray]) -> dict[str, np.ndarray] | None:
    """A recording's arrays with its message logs as groups, or None if it has none to convert.

    Raises:
        ValueError: If it has no "<serial>_floor_pose" to take the serial from.
    """
    if COMMANDS not in data and ODOMETRY not in data:
        return None
    pose = next((key for key in data if key.endswith("_floor_pose")), None)
    if pose is None:
        raise ValueError("no <serial>_floor_pose")
    prefix = pose[:-len("floor_pose")]
    arrays = {key: value for key, value in data.items() if key not in (COMMANDS, ODOMETRY)}
    commands = data.get(COMMANDS)
    if commands is not None and not _twist(data):
        commands = commands.reshape(-1, 5)
        arrays.update(_group(f"{prefix}follower", commands, {"command": (commands[:, 2:4], *VELOCITY),
                                                             "state": (commands[:, 4:5], ("state",), ("",))}))
    odometry = data.get(ODOMETRY)
    if odometry is not None:
        odometry = odometry.reshape(-1, 4)
        arrays.update(_group(f"{prefix}odometry", odometry, {"velocity": (odometry[:, 2:4], *VELOCITY)}))
    return arrays


def check(old: dict[str, np.ndarray], new: dict[str, np.ndarray]) -> None:
    """Raise ValueError unless `new` is `old` converted: the rest unchanged, every logged column in a group."""
    for key, value in old.items():
        if key in (COMMANDS, ODOMETRY):
            continue
        if key not in new or not _same(value, new[key]):
            raise ValueError(f"{key} changed")
    prefix = next(key for key in old if key.endswith("_floor_pose"))[:-len("floor_pose")]
    if COMMANDS in old and not _twist(old):
        log = old[COMMANDS].reshape(-1, 5)
        group = f"{prefix}follower"
        joined = np.column_stack([new[f"{group}.t"], new[f"{group}.stamp"], new[f"{group}.command"],
                                  new[f"{group}.state"]])
        if not _same(log, joined):
            raise ValueError("the follower group differs from commands")
    if ODOMETRY in old:
        log, group = old[ODOMETRY].reshape(-1, 4), f"{prefix}odometry"
        joined = np.column_stack([new[f"{group}.t"], new[f"{group}.stamp"], new[f"{group}.velocity"]])
        if not _same(log, joined):
            raise ValueError("the odometry group differs from wheel_odometry")


def with_units(data: dict[str, np.ndarray]) -> dict[str, np.ndarray] | None:
    """A recording's arrays with one unit per channel ("<key>.units"), or None if it has them already.

    Raises:
        ValueError: If a unit text does not give one unit per channel.
    """
    texts = [key for key in data if key.endswith(".unit")]
    if not texts:
        return None
    arrays = {key: value for key, value in data.items() if not key.endswith(".unit")}
    for key in texts:
        signal, text = key[:-len(".unit")], str(data[key])
        count = len(data[f"{signal}.labels"])
        units = UNITS_BY_TEXT.get(text) or (tuple(text.split(", ")) if ", " in text else (text,) * count)
        if len(units) != count:
            raise ValueError(f"{signal}: unit {text!r} for {count} channels")
        arrays[f"{signal}.units"] = np.array(units)
    return arrays


def convert(path: Path, dry_run: bool = False) -> str:
    """Convert one run folder's recording.npz (and its descriptions) in place; returns what was done.

    Each step is checked against the arrays before it, and the file written against the last step's arrays.
    """
    with np.load(path) as data:
        old = {key: data[key] for key in data.files}
    folder, parts, arrays = path.parent, [], old
    logs = converted(arrays)
    if logs is not None:
        check(arrays, logs)
        parts += (["twist commands dropped" if _twist(old) else "follower"] if COMMANDS in old else []) + \
            (["odometry"] if ODOMETRY in old else [])
        arrays = logs
    moved = described(arrays, load_description(folder))
    if moved is not None:
        _check_moved(arrays, moved[0])
        parts.append("description")
        arrays = moved[0]
    units = with_units(arrays)
    if units is not None:
        _check_units(arrays, units)
        parts.append("units")
        arrays = units
    if not parts:
        return "already converted"
    what = ", ".join(parts)
    if dry_run:
        return f"would convert ({what})"
    written = path.with_name(path.name + ".converting")
    with open(written, "wb") as file:
        np.savez(file, **arrays)
    with np.load(written) as data:
        back = {key: data[key] for key in data.files}
    if set(back) != set(arrays) or not all(_same(arrays[key], back[key]) for key in arrays):
        raise ValueError("the file written differs from the converted arrays")
    if moved is not None:
        write_description(folder, moved[1])
        if moved[2] is not None:
            report = dict(moved[2])
            metrics = report.pop("metrics")
            write_report(folder, report, metrics)
    os.replace(written, path)
    return f"converted ({what})"


def _check_units(old: dict[str, np.ndarray], new: dict[str, np.ndarray]) -> None:
    """Raise ValueError unless `new` is `old` with each unit text replaced by one unit per channel."""
    kept = {key for key in old if not key.endswith(".unit")}
    texts = {key[:-len(".unit")] for key in old if key.endswith(".unit")}
    if set(new) != kept | {f"{signal}.units" for signal in texts} or \
            not all(_same(old[key], new[key]) for key in kept) or \
            any(len(new[f"{signal}.units"]) != len(new[f"{signal}.labels"]) for signal in texts):
        raise ValueError("the units do not match the channels, or another array changed")


def _check_moved(old: dict[str, np.ndarray], new: dict[str, np.ndarray]) -> None:
    """Raise ValueError unless `new` is `old` without the describing arrays, plus "robot"."""
    kept = {key for key in old if key not in DESCRIBING}
    if set(new) != kept | {"robot"} or not all(_same(old[key], new[key]) for key in kept):
        raise ValueError("the recording's arrays changed")


def _group(name: str, log: np.ndarray, signals: dict[str, tuple]) -> dict[str, np.ndarray]:
    """A message group as `Trace.save` writes it: times, stamps, and per signal its values, labels and units."""
    arrays = {f"{name}.t": log[:, 0].copy(), f"{name}.stamp": log[:, 1].copy()}
    for signal, (values, labels, units) in signals.items():
        arrays[f"{name}.{signal}"] = values.copy()
        arrays[f"{name}.{signal}.labels"] = np.array(labels)
        arrays[f"{name}.{signal}.units"] = np.array(units)
    return arrays


def _twist(data: dict[str, np.ndarray]) -> bool:
    """Whether the recording is of a base_exp constant-command run."""
    if "experiment" not in data:
        return False
    return (json.loads(str(data["experiment"])).get("template") or {}).get("name") == TWIST


def _same(a: np.ndarray, b: np.ndarray) -> bool:
    """Equal shape and values, NaN equal to NaN."""
    a, b = np.asarray(a), np.asarray(b)
    if a.dtype.kind in "fc" and b.dtype.kind in "fc":
        return a.shape == b.shape and np.array_equal(a, b, equal_nan=True)
    return a.shape == b.shape and np.array_equal(a, b)


def main() -> None:
    """Convert every recording.npz under the experiments' folder."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--recordings", type=Path, default=None,
                        help=f"Folder searched for recording.npz files (default: {EXPERIMENTS} under the Drive root)")
    parser.add_argument("--dry-run", action="store_true", help="Only say what would be converted")
    args = parser.parse_args()
    try:
        folder = args.recordings or drive_folder(drive_root(), EXPERIMENTS)
    except FileNotFoundError as error:
        raise SystemExit(str(error)) from None
    counts: dict[str, int] = {}
    failed = 0
    for path in sorted(folder.rglob("recording.npz")):
        try:
            done = convert(path, args.dry_run)
        except (ValueError, OSError) as error:
            failed += 1
            print(f"FAILED {path.relative_to(folder)}: {error}")
            continue
        counts[done] = counts.get(done, 0) + 1
    summary = [f"{n} {what}" for what, n in sorted(counts.items())] + ([f"{failed} failed"] if failed else [])
    print(f"{folder}: {', '.join(summary)}")
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
