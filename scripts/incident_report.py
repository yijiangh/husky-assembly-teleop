#!/usr/bin/env python3
"""Build the incident report of a day or a date range: the crashes and failures in the lab's logs and the runs' events.

It lists what the lines say, with the lines, and gives no causes. Run from the workspace root with the venv active and
HUSKY_DRIVE_ROOT set (or `--drive`):

    python src/husky-assembly-teleop/scripts/incident_report.py                     # today
    python src/husky-assembly-teleop/scripts/incident_report.py 20261029 --to 20261104 --open

It writes `<drive>/data_experiment/bar_action_live_replan_exe/<last>-asbuilt-result/incidents_<first>[-<last>].html`,
rebuilt in place on every run.
"""

from __future__ import annotations

import argparse
import sys
import webbrowser
from pathlib import Path

from husky_assembly_teleop.drive import drive_root
from husky_assembly_teleop.experiments.incidents.page import DEFAULT_LOGS, build_report


def main(argv: list[str] | None = None) -> int:
    """Build the report of the days asked for."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("day", nargs="?", help="the first day, YYYYMMDD (default: today)")
    parser.add_argument("--to", help="the last day, YYYYMMDD (default: the first)")
    parser.add_argument("--logs", default=str(DEFAULT_LOGS), help=f"the log folder (default: {DEFAULT_LOGS})")
    parser.add_argument("--drive", help="the Drive root, for the runs' events (default: HUSKY_DRIVE_ROOT)")
    parser.add_argument("--out", help="output .html (default: beside the last day's as-built records on the drive)")
    parser.add_argument("--open", action="store_true", help="open the page in the browser")
    args = parser.parse_args(argv)
    out = build_report(Path(args.logs), args.day, args.to, drive_root(args.drive), Path(args.out) if args.out else None)
    print(f"[assembly] wrote {out} ({out.stat().st_size / 1e6:.1f} MB)")
    if args.open:
        webbrowser.open(out.resolve().as_uri())
    return 0


if __name__ == "__main__":
    sys.exit(main())
