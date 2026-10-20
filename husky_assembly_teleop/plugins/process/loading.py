"""
Loading a process folder for the process plugin; an old compas_fab export is converted first.

! Runs on a worker thread: nothing here may touch viser, PyBullet or a PluginContext.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable

from scaffolding_assembly_core.legacy.conversion import (OLD_EXPORT_FILE, PROCESS_FILE, convert_export,
                                                         converted_folder, is_old_export, is_up_to_date)
from scaffolding_assembly_core.legacy.timing import Stopwatch
from scaffolding_assembly_core.process import Process, read


def load_process(folder: Path, data_directory: Path, report: Callable[[str], None] = lambda _text: None,
                 watch: Stopwatch | None = None) -> Process:
    """Read a process folder; an old compas_fab export is first converted into `<export>_process`.

    The converted copy is reused while it is newer than the export.

    Args:
        folder: The process folder (holding process.json), or an old export (holding ActionSchedule.json).
        data_directory: The monitor's data directory, for converting an old export's robots.
        report: Told what is being done, for the panel. Called from the loading thread.
        watch: Gets a lap per step, if given.

    Returns:
        Process: The process; its `folder` is the converted copy for an old export.

    Raises:
        FileNotFoundError: If the folder is neither a process nor an old export.
    """
    folder = Path(folder).expanduser()
    watch = watch if watch is not None else Stopwatch()
    if is_old_export(folder):
        export, folder = folder, converted_folder(folder)
        if not is_up_to_date(export):
            convert_export(export, folder, data_directory, report, watch)
    if not (folder / PROCESS_FILE).is_file():
        raise FileNotFoundError(f"no {PROCESS_FILE} or {OLD_EXPORT_FILE} in {folder}")
    report(f"reading {folder.name}")
    process = read(folder)
    watch.lap("read process")
    return process
