"""Convert an export in the old compas_fab format into a schema 1 design folder (doc/design_format.md).

    python scripts/convert_design.py <export folder> [<design folder>]

The design folder defaults to `<export>_design` next to the export, where the cell plugin also
puts and finds it. Robot files come from this repository's `data/` (bar_assembly_core.design_io.conversion).
"""

from __future__ import annotations

import sys
from argparse import ArgumentParser
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from bar_assembly_core.design_io.conversion import convert_export, converted_folder  # noqa: E402
from bar_assembly_core.design_io.timing import Stopwatch  # noqa: E402


def main() -> None:
    """Convert the export given on the command line."""
    parser = ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("export", type=Path, help="folder with ActionSchedule.json and RobotCell*.json")
    parser.add_argument("design", type=Path, nargs="?", help="folder to write the design into (replaced)")
    options = parser.parse_args()
    destination = options.design or converted_folder(options.export)
    watch = Stopwatch()
    convert_export(options.export, destination, REPO / "data", watch=watch)
    print(f"wrote {destination} in {watch.summary()}")


if __name__ == "__main__":
    main()
