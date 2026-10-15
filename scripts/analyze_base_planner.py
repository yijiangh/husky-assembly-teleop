"""
Report one base planner experiment (doc/base_planner_manual.md): the planners side by side on the standard tasks.

Reads every attempt folder of the experiment under --recordings (default: the base planner's RECORDING_FOLDER under the
Drive root, HUSKY_DRIVE_ROOT) and writes its `_analysis/` (`plugins/base_planner/analysis.py`): summary.md, one CSV
per table, tasks.png, metrics.png and results_prompt.md; and per attempt plan.png, per run overview.png and report.json.

Run with the venv and ROS sourced (it imports crl_husky):

    python3 src/husky-assembly-teleop/scripts/analyze_base_planner.py --experiment benchmark_1015
"""

from __future__ import annotations

import argparse
from pathlib import Path

from husky_assembly_teleop.drive import drive_folder, drive_root
from husky_assembly_teleop.plugins.base_planner.analysis import analyse
from husky_assembly_teleop.plugins.base_planner.recording import RECORDING_FOLDER


def main() -> None:
    """Parse the arguments and write the experiment's report."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--experiment", required=True, help="Experiment name: the folder of its attempts")
    parser.add_argument("--recordings", type=Path, default=None,
                        help=f"Folder of the experiments (default: {RECORDING_FOLDER} under the Drive root)")
    args = parser.parse_args()
    try:
        recordings = args.recordings or drive_folder(drive_root(), RECORDING_FOLDER)
    except FileNotFoundError as error:
        raise SystemExit(str(error)) from None
    experiment = recordings / args.experiment
    if not experiment.is_dir():
        raise SystemExit(f"no experiment {args.experiment!r} under {recordings}")
    print(f"written {analyse(experiment)}")


if __name__ == "__main__":
    main()
