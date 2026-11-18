#!/usr/bin/env python3
"""Look into the observer's recordings on this PC, and copy a time range of them (to the Drive by default).

The `assembly_observer` plugin records every session in `~/.ros/assembly_observer/<YYYYMMDD>/<stamp>_session/` and
keeps a week; its panel's Copy to Drive runs `copy` below. Times are HH:MM or HH:MM:SS of `--day` (today by default),
or `now`. Run from the workspace root with the venv active:

    python src/husky-assembly-teleop/scripts/observer_logs.py list
    python src/husky-assembly-teleop/scripts/observer_logs.py events 14:00 14:15 --grep refused
    python src/husky-assembly-teleop/scripts/observer_logs.py events 14:00 now --robot a200-0806 --event "arm moving"
    python src/husky-assembly-teleop/scripts/observer_logs.py copy 14:00 14:30 --name b20_insert
    python src/husky-assembly-teleop/scripts/observer_logs.py rerun 14:00 14:30
    python src/husky-assembly-teleop/scripts/observer_logs.py rerun --folder <a copy's folder>

`rerun` opens the range in the Rerun viewer (`pip install rerun-sdk==0.39.0`): plots, state timelines, the events and
the robots in 3D on one timeline (`plugins/assembly_observer/rerun_view.py`); `--save` writes an .rrd file instead.
With `--folder` it opens a copy (on the Drive, say) whole, or From to To of it.

`copy` writes `<Drive>/data_experiment/assembly_observer/<name>/<stamp>/` (or under `--to`), one folder per session the
range touches, and ends with one JSON line saying what it copied. The folder is `<stamp>_sim/` when every session
copied is a simulation.

Every command takes `--env real|sim|both` (default both): only the real sessions, only the simulated ones, or all.
A session's environment is in its experiment.json, and a simulated one's folder ends in "_sim".
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path

from husky_assembly_teleop.drive import drive_folder, drive_root, folder_name
from husky_assembly_teleop.drive import SIM
from husky_assembly_teleop.plugins.assembly_observer.export import (BOTH, DAY_FORMAT, ENVIRONMENTS, LOCAL_ROOT,
                                                                    RECORDING_FOLDER, copied_sessions, copy_range,
                                                                    events_in, sessions, wall_time)
from husky_assembly_teleop.shared.recordings.runs import run_folder

#: The teleop data folder: the robots' URDFs, for the arms' mounts in `rerun`.
DATA = Path(__file__).resolve().parents[1] / "data"
#: Keys an event line leaves out when printed: the times it starts with, and what is too long for a line.
QUIET = ("event", "time", "ros_time", "status", "flags", "hit")


def main(argv: list[str] | None = None) -> int:
    """List the sessions, print a range's events, or copy a range."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--root", type=Path, default=LOCAL_ROOT, help=f"the recordings (default {LOCAL_ROOT})")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("list", help="every session kept: when, how long, how big, sim or real")
    for name, text in (("events", "print the events of a time range"), ("copy", "copy a time range"),
                       ("rerun", "open a time range in the Rerun viewer")):
        command = commands.add_parser(name, help=text)
        # * A copy opened in Rerun needs no range: it opens whole.
        optional = {"nargs": "?"} if name == "rerun" else {}
        command.add_argument("start", help="HH:MM or HH:MM:SS", **optional)
        command.add_argument("end", help="HH:MM or HH:MM:SS, or now", **optional)
        command.add_argument("--day", help="YYYYMMDD (default today, or a copy's first day)")
    events = commands.choices["events"]
    events.add_argument("--grep", help="only lines holding this text")
    events.add_argument("--robot", help="only this robot's events, e.g. a200-0806")
    events.add_argument("--event", help="only events of this name, e.g. 'arm moving' or log")
    events.add_argument("--json", action="store_true", help="each event as its JSON line")
    copy = commands.choices["copy"]
    copy.add_argument("--name", required=True, help="the experiment's folder name")
    copy.add_argument("--to", type=Path, help=f"copy under this folder (default <Drive>/{RECORDING_FOLDER})")
    copy.add_argument("--drive", type=Path, help="the Drive root (default HUSKY_DRIVE_ROOT)")
    viewer = commands.choices["rerun"]
    viewer.add_argument("--save", type=Path, help="write this .rrd file instead of opening the viewer")
    viewer.add_argument("--folder", type=Path, help="open this copy (`copy`'s folder, one of its sessions, or a "
                                                    "folder of copies) instead of the recordings on this PC")
    for command in commands.choices.values():
        command.add_argument("--env", choices=ENVIRONMENTS, default=BOTH,
                             help="only real sessions, only simulated ones, or both (default)")
    args = parser.parse_args(argv)
    if args.command == "list":
        return _list(args.root, args.env)
    if args.command == "rerun" and args.folder is not None:
        return _rerun_copy(args)
    if args.start is None or args.end is None:
        return _fail("a start and an end are needed (only a copy opens without them: --folder)")
    args.day = args.day or time.strftime(DAY_FORMAT)
    try:
        start, end = wall_time(args.day, args.start), wall_time(args.day, args.end)
        if end <= start:
            raise ValueError(f"the end ({args.end}) is not after the start ({args.start})")
    except ValueError as error:
        return _fail(str(error))
    if args.command == "events":
        return _events(args, start, end)
    if args.command == "rerun":
        return _rerun(args, start, end)
    return _copy(args, start, end)


def _list(root: Path, environment: str) -> int:
    """Print every session kept, oldest first."""
    found = sessions(root, environment)
    for session in found:
        print(f"{_clock(session['start'], '%Y-%m-%d %H:%M')}  to {_clock(session['end'], '%H:%M')}  "
              f"{session['bytes'] / 1e6:7.1f} MB  {session['environment']:4}  {session['folder']}")
    if not found:
        print(f"no recordings in {root}" + ("" if environment == BOTH else f" ({environment} only)"))
    return 0


def _events(args: argparse.Namespace, start: float, end: float) -> int:
    """Print the range's events, filtered, one per line."""
    for _session, event in events_in(args.root, start, end, args.env):
        if args.robot and event.get("robot") != args.robot or args.event and event.get("event") != args.event:
            continue
        line = json.dumps(event) if args.json else _readable(event)
        if args.grep and args.grep not in line:
            continue
        print(line)
    return 0


def _copy(args: argparse.Namespace, start: float, end: float) -> int:
    """Copy the range into a new folder; the last line printed is JSON: the folder and what was copied, or why not."""
    name = folder_name(args.name, "experiment")
    try:
        if args.to is not None:
            args.to.mkdir(parents=True, exist_ok=True)  # * a folder of the operator's; the Drive's is never made here
        under = args.to if args.to is not None else drive_folder(args.drive or drive_root(), RECORDING_FOLDER)
        destination = run_folder(under, name)
        about = {"name": name, "day": args.day, "from_text": args.start, "to_text": args.end}
        copied = copy_range(args.root, start, end, destination, about, args.env)
        if not copied:
            destination.rmdir()
        elif all(session["environment"] == SIM for session in copied_sessions(destination)):
            destination = _renamed(destination, f"{destination.name}_sim")
    except Exception as error:
        return _fail(repr(error))
    print(json.dumps({"destination": str(destination) if copied else None, "copied": copied}))
    return 0


def _renamed(folder: Path, name: str) -> Path:
    """`folder` renamed to `name` beside it, "_2", "_3", … if taken."""
    target, count = folder.with_name(name), 2
    while target.exists():
        target, count = folder.with_name(f"{name}_{count}"), count + 1
    folder.rename(target)
    return target


def _rerun_copy(args: argparse.Namespace) -> int:
    """Open a copy in Rerun: whole, or from `start` to `end` (of `--day`, by default the copy's first day)."""
    found = copied_sessions(args.folder, args.env)
    if not found:
        return _fail(f"no copied sessions in {args.folder}" + ("" if args.env == BOTH else f" ({args.env} only)"))
    start, end = min(session["start"] for session in found), max(session["end"] for session in found)
    day = args.day or time.strftime(DAY_FORMAT, time.localtime(start))
    try:
        start = wall_time(day, args.start) if args.start is not None else start
        end = wall_time(day, args.end) if args.end is not None else end
        if end <= start:
            raise ValueError(f"the end ({args.end}) is not after the start ({args.start})")
    except ValueError as error:
        return _fail(str(error))
    return _rerun(args, start, end, args.folder, copy=True)


def _rerun(args: argparse.Namespace, start: float, end: float, root: Path | None = None, copy: bool = False) -> int:
    """Open a range of the recordings (or of the copy at `root`) in Rerun, or save it; the last line printed is JSON."""
    try:
        import rerun  # noqa: F401
    except ImportError:
        return _fail("rerun is not installed: pip install rerun-sdk==0.39.0 (in the venv)")
    from husky_assembly_teleop.plugins.assembly_observer.rerun_view import log_range
    try:
        shown = log_range(root or args.root, start, end, save=args.save, data=DATA, copy=copy,
                          environment=args.env)
    except Exception as error:
        return _fail(repr(error))
    for note in shown["notes"]:
        print(note, file=sys.stderr)
    print(json.dumps({**shown, "saved": str(args.save) if args.save else None}))
    return 0


def _readable(event: dict) -> str:
    """One event as a line: its local time, its name, its robot and arm, then what it says."""
    clock = event.get("time", "")[11:23]
    if event.get("event") == "log":
        return f"{clock}  log  [{event.get('source')}] {event.get('level', '')} {event.get('text')}" + \
            (f" (x{event['count']})" if event.get("count", 1) > 1 else "")
    rest = " ".join(f"{key}={json.dumps(value)}" for key, value in event.items() if key not in QUIET)
    return f"{clock}  {event.get('event')}  {rest}"


def _clock(wall: float, form: str) -> str:
    """A wall time as local text."""
    return datetime.fromtimestamp(wall).strftime(form)


def _fail(why: str) -> int:
    """Say why on stderr and as the last JSON line; exit status 1."""
    print(why, file=sys.stderr)
    print(json.dumps({"error": why}))
    return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except BrokenPipeError:  # * printed into `head` or a pager closed early: no traceback, no flush error at exit
        os.dup2(os.open(os.devnull, os.O_WRONLY), sys.stdout.fileno())
        sys.exit(0)
