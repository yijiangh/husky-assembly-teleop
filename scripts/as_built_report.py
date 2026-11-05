#!/usr/bin/env python3
"""Build the accuracy report of one lab day: the as-built records' numbers per step, per bar and per servo run.

Numbers only, without conclusions, from the code behind the 3D page (`scripts/as_built_page.py`), so the two agree. It
reads a day of the new monitor (the records and the runs' servo files) or of the old one (the records and the day's
`-servoing` folder). Run from the workspace root with the venv active and HUSKY_DRIVE_ROOT set (or `--drive`):

    python src/husky-assembly-teleop/scripts/as_built_report.py            # the newest day
    python src/husky-assembly-teleop/scripts/as_built_report.py 20261028 --open

It writes `<drive>/data_experiment/bar_action_live_replan_exe/<day>-asbuilt-result/accuracy_<day>.html`, rebuilt in
place on every run.
"""

from __future__ import annotations

import argparse
import sys
import webbrowser
from pathlib import Path

from husky_assembly_teleop.drive import drive_root
from husky_assembly_teleop.experiments.as_built.accuracy_page import build_report


def main(argv: list[str] | None = None) -> int:
    """Build the report of one lab day."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("day", nargs="?", help="YYYYMMDD (default: the newest day with as-built records)")
    parser.add_argument("--problem", help="the process folder's name (default: the day's newest file's)")
    parser.add_argument("--process", help="the process folder (default: <drive>/data_design_study/<problem>)")
    parser.add_argument("--drive", help="the Drive root (default: HUSKY_DRIVE_ROOT)")
    parser.add_argument("--out", help="output .html (default: <day>-asbuilt-result/accuracy_<day>.html on the drive)")
    parser.add_argument("--open", action="store_true", help="open the page in the browser")
    args = parser.parse_args(argv)
    root = drive_root(args.drive)
    if root is None:
        parser.error("no Drive root: set HUSKY_DRIVE_ROOT or pass --drive")
    out = build_report(root, args.day, args.problem, Path(args.process) if args.process else None,
                       Path(args.out) if args.out else None)
    print(f"[assembly] wrote {out} ({out.stat().st_size / 1e6:.1f} MB)")
    if args.open:
        webbrowser.open(out.resolve().as_uri())
    return 0


if __name__ == "__main__":
    sys.exit(main())
