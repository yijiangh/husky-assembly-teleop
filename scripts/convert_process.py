"""Convert an export in the old compas_fab format into a schema 2.1 process folder, then run the plan checks on it.

The format is in doc/process_format.md; the plan checks' errors and warnings are printed.

    python scripts/convert_process.py <export folder> [<process folder>]

The process folder defaults to `<export>_process` next to the export, where the cell plugin also
puts and finds it. Robot files come from this repository's `data/` (scaffolding_assembly_core.legacy.conversion).
"""

from __future__ import annotations

import sys
from argparse import ArgumentParser
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from scaffolding_assembly_core.process import read  # noqa: E402
from scaffolding_assembly_core.process.plan_check import check_plan  # noqa: E402
from scaffolding_assembly_core.legacy.conversion import convert_export, converted_folder  # noqa: E402
from scaffolding_assembly_core.legacy.timing import Stopwatch  # noqa: E402


def main() -> None:
    """Convert the export given on the command line."""
    parser = ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("export", type=Path, help="folder with ActionSchedule.json and RobotCell*.json")
    parser.add_argument("process", type=Path, nargs="?", help="folder to write the process into (replaced)")
    options = parser.parse_args()
    destination = options.process or converted_folder(options.export)
    watch = Stopwatch()
    convert_export(options.export, destination, REPO / "data", watch=watch)
    print(f"wrote {destination} in {watch.summary()}")
    report = check_plan(read(destination))
    for line in (*report.errors, *report.warnings):
        print(line)
    print(f"plan checks: {len(report.errors)} errors, {len(report.warnings)} warnings")


if __name__ == "__main__":
    main()
