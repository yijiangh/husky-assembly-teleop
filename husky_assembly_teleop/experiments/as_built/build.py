"""
Build one lab day's as-built page: the records collected (`collect.py`), the room added, the .html written (`page.py`).

`build_page` is what `scripts/as_built_page.py` and the monitor's "Build as-built page" button run.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

from ...world.lab import read_lab
from .collect import (EXTRA_FORMULAS, METRICS, PAGE_CONTROLS, PAGE_CSS, as_built_days, as_built_problems,
                      collect_assembly, finite)
from .page import CAMERA_CONE_HALF_ANGLE_DEG, CAMERA_CONE_M, cameras_3dm, environment_3dm, lab_environment
from .page import viewer_script, write_page
from ...shared.runs.as_built import AS_BUILT_FOLDER, DIR_SUFFIX

#: Where a day's page goes, beside its records: `<drive>/.../<day>-asbuilt-result/assembly_<day>.html`.
RESULT_SUFFIX = "-asbuilt-result"


def result_folder(drive_root: Path, day: str) -> Path:
    """The folder a day's page is written to, beside the day's records."""
    return drive_root / AS_BUILT_FOLDER / f"{day}{RESULT_SUFFIX}"


def build_page(drive_root: Path, day: str | None = None, problem: str | None = None,
               process_folder: Path | None = None, out: Path | None = None, vendor: str = "inline",
               env_3dm: Path | None = None, lab: Path | None = None, with_robots: bool = True,
               with_cameras: bool = True) -> Path:
    """Build the page of one lab day and return its path.

    Args:
        drive_root: The Drive root the records are under.
        day: YYYYMMDD; default the newest day with records.
        problem: The process folder's name; default the day's newest file's.
        process_folder: The process folder; default `<drive>/data_design_study/<problem>`.
        out: The .html to write; default `<drive>/.../<day>-asbuilt-result/assembly_<day>.html`.
        vendor: How the page carries three.js: "inline" (one script, works served or from disk), "embed", "sibling".
        env_3dm: A Rhino file whose `env` layer is the room and whose camera layer the Motive rig, or None.
        lab: The lab file (`data/lab/<lab>.json`) whose obstacles are drawn as the room, or None.
        with_robots: Draw the robots (loads the URDF meshes).
        with_cameras: Draw the Motive cameras (from `env_3dm`).

    Raises:
        SystemExit: No records, or the 3D library files are missing.
    """
    drive_root = Path(drive_root)
    if day is None:
        days = as_built_days(drive_root)
        if not days:
            raise SystemExit(f"no as-built records under {drive_root / AS_BUILT_FOLDER}")
        day = days[-1]
    if problem is None:
        problems = as_built_problems(drive_root, day)
        if not problems:
            raise SystemExit(f"no as-built file in {drive_root / AS_BUILT_FOLDER / (day + DIR_SUFFIX)}")
        problem = problems[0]
    scene = collect_assembly(day, problem, drive_root, process_folder, with_robots=with_robots)
    env = environment_3dm(Path(env_3dm)) if env_3dm else None
    if env is None and lab is not None:
        env = lab_environment(read_lab(Path(lab)))
    scene.update({
        "title": f"As-built {problem} · {day}", "generated": datetime.now().isoformat(timespec="seconds"),
        "metrics": METRICS, "extra_formulas": EXTRA_FORMULAS, "env": env,
        "cameras": cameras_3dm(Path(env_3dm)) if env_3dm and with_cameras else [],
        "cone": {"length": CAMERA_CONE_M, "half_angle_deg": CAMERA_CONE_HALF_ANGLE_DEG},
    })
    out = Path(out) if out else result_folder(drive_root, day) / f"assembly_{day}.html"
    return write_page(finite(scene), out, vendor, viewer_script(), extra_css=PAGE_CSS, controls=PAGE_CONTROLS)
