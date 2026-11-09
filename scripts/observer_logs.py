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

`rerun` opens the range in the Rerun viewer (`pip install rerun-sdk`): plots, state timelines, the events and the
robots in 3D on one timeline (`plugins/assembly_observer/rerun_view.py`); `--save` writes an .rrd file instead.

`copy` writes `<Drive>/data_experiment/assembly_observer/<name>/<stamp>/` (or under `--to`), one folder per session the
range touches, and ends with one JSON line saying what it copied.
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
from husky_assembly_teleop.plugins.assembly_observer.export import (DAY_FORMAT, LOCAL_ROOT, RECORDING_FOLDER,
                                                                    copy_range, events_in, sessions, wall_time)
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
    commands.add_parser("list", help="every session kept: when, how long, how big")
    for name, text in (("events", "print the events of a time range"), ("copy", "copy a time range"),
                       ("rerun", "open a time range in the Rerun viewer")):
        command = commands.add_parser(name, help=text)
        command.add_argument("start", help="HH:MM or HH:MM:SS")
        command.add_argument("end", help="HH:MM or HH:MM:SS, or now")
        command.add_argument("--day", default=time.strftime(DAY_FORMAT), help="YYYYMMDD (default today)")
    events = commands.choices["events"]
    events.add_argument("--grep", help="only lines holding this text")
    events.add_argument("--robot", help="only this robot's events, e.g. a200-0806")
    events.add_argument("--event", help="only events of this name, e.g. 'arm moving' or log")
    events.add_argument("--json", action="store_true", help="each event as its JSON line")
    copy = commands.choices["copy"]
    copy.add_argument("--name", required=True, help="the experiment's folder name")
    copy.add_argument("--to", type=Path, help=f"copy under this folder (default <Drive>/{RECORDING_FOLDER})")
    copy.add_argument("--drive", type=Path, help="the Drive root (default HUSKY_DRIVE_ROOT)")
    copy.add_argument("--sim", action="store_true", help="mark the copy's folder as from a simulation")
    viewer = commands.choices["rerun"]
    viewer.add_argument("--save", type=Path, help="write this .rrd file instead of opening the viewer")
    args = parser.parse_args(argv)
    if args.command == "list":
        return _list(args.root)
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


def _list(root: Path) -> int:
    """Print every session kept, oldest first."""
    found = sessions(root)
    for session in found:
        print(f"{_clock(session['start'], '%Y-%m-%d %H:%M')}  to {_clock(session['end'], '%H:%M')}  "
              f"{session['bytes'] / 1e6:7.1f} MB  {session['environment'] or '?':4}  {session['folder']}")
    if not found:
        print(f"no recordings in {root}")
    return 0


def _events(args: argparse.Namespace, start: float, end: float) -> int:
    """Print the range's events, filtered, one per line."""
    for _session, event in events_in(args.root, start, end):
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
        destination = run_folder(under, name, *(["sim"] if args.sim else []))
        about = {"name": name, "day": args.day, "from_text": args.start, "to_text": args.end}
        copied = copy_range(args.root, start, end, destination, about)
    except Exception as error:
        return _fail(repr(error))
    if not copied:
        destination.rmdir()
    print(json.dumps({"destination": str(destination) if copied else None, "copied": copied}))
    return 0


def _rerun(args: argparse.Namespace, start: float, end: float) -> int:
    """Open the range in Rerun (or save it); the last line printed is JSON: what was shown, or why not."""
    try:
        import rerun  # noqa: F401
    except ImportError:
        return _fail("rerun is not installed: pip install rerun-sdk (in the venv)")
    from husky_assembly_teleop.plugins.assembly_observer.rerun_view import log_range
    try:
        shown = log_range(args.root, start, end, save=args.save, data=DATA)
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
