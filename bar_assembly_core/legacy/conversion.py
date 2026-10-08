"""
Convert an export in the old compas_fab format into a schema 1 design, written next to it in `<export>_design`.

The robots are the calibrated URDFs and SRDFs from the data directory, copied into the design with their meshes.
! Slow (~15 s, the cells are ~350 MB each): call it off the main thread.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable, Optional

from ..design.write import write
from .export import from_export, load_export
from .timing import Stopwatch
from ..design.types import Design

#: A folder holding this is an export in the old compas_fab format.
OLD_EXPORT_FILE = "ActionSchedule.json"
#: A folder holding this is a schema 1 design.
DESIGN_FILE = "design.json"
#: Added to an export's folder name for its converted copy.
CONVERTED_SUFFIX = "_design"

#: Robot name in the schedule -> (URDF, SRDF) under the data directory, as the Rhino workflow's
#: `core/config.py` uses them. ! The calibrated files, not the monitor's `_StockUrFrames` variants.
ROBOT_FILES = {
    "Cindy": ("husky_urdf/mt_husky_dual_ur5_e_moveit_config/urdf/husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf",
              "husky_urdf/mt_husky_dual_ur5_e_moveit_config/config/dual_arm_husky.srdf"),
    "Alice": ("husky_urdf/mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint_Alice_Calibrated.urdf",
              "husky_urdf/mt_husky_moveit_config/config/husky.srdf"),
    "Belle": ("husky_urdf/mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint_Belle_Calibrated.urdf",
              "husky_urdf/mt_husky_moveit_config/config/belle.srdf"),
}

#: Robot name -> hardware serial (config._ROBOTS_BY_SERIAL).
SERIALS = {"Cindy": "0806", "Alice": "0804", "Belle": "0805"}


def is_old_export(folder: Path) -> bool:
    """Whether a folder is an old compas_fab export and not a schema 1 design."""
    folder = Path(folder)
    return (folder / OLD_EXPORT_FILE).is_file() and not (folder / DESIGN_FILE).is_file()


def converted_folder(export: Path) -> Path:
    """The sibling folder `<export>_design` an export is converted into."""
    export = Path(export).expanduser().resolve()
    return export.with_name(export.name + CONVERTED_SUFFIX)


def is_up_to_date(export: Path) -> bool:
    """Whether the export's converted copy exists and is newer than every JSON file of the export.

    Args:
        export: The export folder.
    """
    design_file = converted_folder(export) / DESIGN_FILE
    if not design_file.is_file():
        return False
    written = design_file.stat().st_mtime
    return all(path.stat().st_mtime <= written for path in Path(export).rglob("*.json"))


def robot_files(data_directory: Path) -> dict:
    """ROBOT_FILES as absolute paths under a data directory.

    Args:
        data_directory: The monitor's data directory, holding `husky_urdf/`.

    Returns:
        dict[str, tuple[Path, Path]]: Robot name -> (URDF, SRDF).
    """
    return {name: (data_directory / urdf, data_directory / srdf) for name, (urdf, srdf) in ROBOT_FILES.items()}


def convert_in_memory(export: Path, data_directory: Path, report: Callable[[str], None] = print,
                      watch: Optional[Stopwatch] = None) -> Design:
    """Load an export and convert it into an in-memory design (`folder` None), writing nothing.

    Args:
        export: The export folder, holding ActionSchedule.json.
        data_directory: The monitor's data directory, holding `husky_urdf/`.
        report: Told what is being done.
        watch: Gets a lap per step, if given.
    """
    watch = watch if watch is not None else Stopwatch()
    loaded = load_export(export, report, watch)
    design = from_export(loaded, robot_files(data_directory), serials=SERIALS, report=report)
    watch.lap("convert")
    return design


def convert_export(export: Path, destination: Path, data_directory: Path,
                   report: Callable[[str], None] = print, watch: Optional[Stopwatch] = None) -> Path:
    """Convert an export into a schema 1 design folder, replacing what is there.

    Args:
        export: The export folder, holding ActionSchedule.json.
        destination: The design folder to write.
        data_directory: The monitor's data directory, holding `husky_urdf/`.
        report: Told what is being done.
        watch: Gets a lap per step, if given.

    Returns:
        Path: The design folder.
    """
    watch = watch if watch is not None else Stopwatch()
    design = convert_in_memory(export, data_directory, report, watch)
    report(f"writing {destination}")
    write(design, destination, overwrite=True, package_dirs=[data_directory / "husky_urdf"])
    watch.lap("write (copy robots, read back, validate)")
    return Path(destination)
