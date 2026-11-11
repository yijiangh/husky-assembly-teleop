#!/usr/bin/env python3
"""Build the 3D assembly page of one lab day from its as-built records (the old monitor's `4_assembly_viewer.py`).

The monitor appends one as-built record at the start and the end of every part it runs
(`husky_assembly_teleop/shared/runs/as_built.py`): every bar with MoCap markers, its fit, where it should be, the
robot's base, joints, tool0 and load, the servo, the tick boxes. This script reads one lab day of them and writes one
offline page (three.js inside, no server): a slider over the schedule's actions, each record's bars where they were
measured and where they should be, their markers, and a card per bar with the placement, rotation, plates, bow, fit,
tool0, servo and load numbers. Manual: `doc/lab/real_lab_test_manual.md`, the as-built part.

Run from the workspace root with the venv active and HUSKY_DRIVE_ROOT set (or `--drive`):

    python src/husky-assembly-teleop/scripts/as_built_page.py            # the newest day
    python src/husky-assembly-teleop/scripts/as_built_page.py 20261030 --open

It writes `<drive>/data_experiment/bar_action_live_replan_exe/<day>-asbuilt-result/assembly_<day>.html`, rebuilt in
place on every run. The three.js files come from `scripts/fetch_three_js.sh`, once per checkout.
"""

from __future__ import annotations

import argparse
import sys
import webbrowser
from pathlib import Path

from husky_assembly_teleop.drive import drive_root
from husky_assembly_teleop.experiments.as_built.build import build_page


def main(argv: list[str] | None = None) -> int:
    """Build the page of one lab day."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("day", nargs="?", help="YYYYMMDD (default: the newest day with as-built records)")
    parser.add_argument("--problem", help="the process folder's name (default: the day's newest file's)")
    parser.add_argument("--process", help="the process folder (default: <drive>/data_design_study/<problem>)")
    parser.add_argument("--drive", help="the Drive root (default: HUSKY_DRIVE_ROOT)")
    parser.add_argument("--out", help="output .html (default: <day>-asbuilt-result/assembly_<day>.html on the drive)")
    parser.add_argument("--env-3dm", help="a Rhino file: its 'env' layer is the room, its camera layer the rig")
    parser.add_argument("--lab", help="a lab file (data/lab/<name>.json) whose obstacles are drawn as the room")
    parser.add_argument("--no-robots", action="store_true", help="skip the robots (no URDF load, smaller page)")
    parser.add_argument("--no-cameras", action="store_true", help="skip the mocap cameras")
    parser.add_argument("--vendor", choices=("inline", "embed", "sibling"), default="inline",
                        help="how three.js is carried (default inline: works from disk and when served)")
    parser.add_argument("--open", action="store_true", help="open the page in the browser")
    args = parser.parse_args(argv)
    root = drive_root(args.drive)
    if root is None:
        parser.error("no Drive root: set HUSKY_DRIVE_ROOT or pass --drive")
    lab = Path(args.lab) if args.lab else None
    if lab is not None and not lab.suffix:
        lab = Path(__file__).resolve().parent.parent / "data" / "lab" / f"{lab}.json"
    out = build_page(root, args.day, args.problem, Path(args.process) if args.process else None,
                     Path(args.out) if args.out else None, args.vendor, Path(args.env_3dm) if args.env_3dm else None,
                     lab, with_robots=not args.no_robots, with_cameras=not args.no_cameras)
    print(f"[assembly] wrote {out} ({out.stat().st_size / 1e6:.1f} MB)")
    if args.open:
        webbrowser.open(out.resolve().as_uri())
    return 0


if __name__ == "__main__":
    sys.exit(main())
