"""
Analyse an experiment's base runs: the controllers, the robot, and its response over speed and turn rate.

Reads every run folder under --recordings (default: record.RECORDING_FOLDER under the Drive root, HUSKY_DRIVE_ROOT;
folders starting with "_", such as _analysis and _archive, are skipped) and writes to the experiment's _analysis
folder (`plugins/base_exp/analysis/`):

  summary.md             the controllers, from the path runs: the standard-path headline, by path and template,
                         failures
  robot_model.md         the robot, from the path runs: its model, stability, prediction, simulator against real,
                         checks, the model for crl_husky
  constant_commands.md   the robot over speed and turn rate, from the constant-command runs (if any)
  results_prompt.md      the prompt for results.md, written by a subagent
  *.csv, *.png           the tables and plots behind them

Run with the venv and ROS sourced (it imports crl_husky):

    python3 src/husky-assembly-teleop/scripts/analyze_base_exp.py --experiment my_collection
"""

from __future__ import annotations

import argparse
from pathlib import Path

from husky_assembly_teleop.drive import drive_folder, drive_root
from husky_assembly_teleop.plugins.base_exp.analysis import commands, controller, robot_model
from husky_assembly_teleop.plugins.base_exp.analysis.common import COMMANDS, PATHS, Experiment, load_runs
from husky_assembly_teleop.plugins.base_exp.record import RECORDING_FOLDER


def main() -> None:
    """Parse the arguments, load the runs, and write the reports their runs allow."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--recordings", type=Path, default=None,
                        help=f"Folder of the run folders (default: {RECORDING_FOLDER} under the Drive root)")
    parser.add_argument("--experiment", default=None, help="Only this experiment name (default: all)")
    parser.add_argument("--out", type=Path, default=None,
                        help="Where to write (default: <recordings>/<experiment>/_analysis, or "
                             "<recordings>/_analysis/all without --experiment)")
    args = parser.parse_args()
    if args.recordings is None:
        try:
            args.recordings = drive_folder(drive_root(), RECORDING_FOLDER)
        except FileNotFoundError as error:
            raise SystemExit(str(error)) from None

    runs, profiles, left_out = load_runs(args.recordings, args.experiment)
    if runs.empty:
        raise SystemExit(f"no runs following the protocol under {args.recordings}"
                         + (f" for experiment {args.experiment}" if args.experiment else "") + f" ({left_out})")
    # * Only inside existing folders: the experiment's, or the recordings' _analysis for all of them.
    out = args.out or (args.recordings / args.experiment / "_analysis" if args.experiment
                       else args.recordings / "_analysis" / "all")
    out.mkdir(parents=True, exist_ok=True)
    experiment = Experiment(args.experiment or "all", runs, profiles, left_out, out)
    written = []
    if (runs.kind == PATHS).any():
        controller.write(experiment.only(PATHS))
        robot_model.write(experiment.only(PATHS))
        written += ["summary.md", "robot_model.md"]
    if (runs.kind == COMMANDS).any():
        commands.write(experiment.only(COMMANDS))
        written.append("constant_commands.md")
    print(f"{len(runs)} runs ({int(runs.success.sum())} done; left out: {left_out}) -> {out}: {', '.join(written)}")


if __name__ == "__main__":
    main()
