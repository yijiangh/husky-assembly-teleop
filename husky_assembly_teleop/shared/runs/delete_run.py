"""
Deleting an assembly run: its folder, its rows in the as-built files, and the day's report pages that showed them.

As-built files and report pages are per day and shared between runs, so rows are taken out one by one and a page is
built again; a page goes only with the last records of its day. Incident pages stay: they read the day's logs.

! Everything here is on the shared Drive, and may not be undoable: only on the operator's say, after `plan_deletion`.
! Blocking: call it on a thread.
"""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
from dataclasses import dataclass
from os import replace as replace_file
from pathlib import Path
from typing import Mapping

from .as_built import AS_BUILT_FOLDER, DIR_SUFFIX, FILE_PREFIX
from .runs import RUN_FILE, RUNS

#: A day's result folder, beside its records (`experiments/as_built/build.py`).
RESULT_SUFFIX = "-asbuilt-result"
#: The pages made from a day's as-built records, and the script that builds each, by file name with "{day}".
PAGE_SCRIPTS = {"assembly_{day}.html": "as_built_page.py", "accuracy_{day}.html": "as_built_report.py"}
#: The scripts, beside the package.
SCRIPTS = Path(__file__).resolve().parents[3] / "scripts"
#: How often a rewrite tries again when a line was appended while it wrote.
REWRITE_TRIES = 3


@dataclass(frozen=True)
class Page:
    """A report page of one day."""

    path: Path
    day: str
    script: str


@dataclass(frozen=True)
class RunDeletion:
    """What deleting a run would take, to show before it happens.

    Attributes:
        folder: The run's folder.
        file_count: Files in it.
        size: Their bytes.
        rows: The run's rows in each as-built file that has any, by file.
    """

    folder: Path
    file_count: int
    size: int
    rows: Mapping[Path, int]

    @property
    def pages(self) -> tuple[Page, ...]:
        """tuple[Page, ...]: The pages that exist of the days those rows are in."""
        return tuple(page for path in self.rows for page in _day_pages(path))


@dataclass(frozen=True)
class RunDeleted:
    """What deleting a run did.

    Attributes:
        rows: How many as-built rows went.
        pages_removed: Pages deleted with the last records of their day.
        to_rebuild: Pages that showed the run, of days with other rows left: build them again (`rebuild_page`).
        kept: Pages left alone, of days whose records left are another process's: a page does not say whose it is.
    """

    rows: int
    pages_removed: tuple[Path, ...]
    to_rebuild: tuple[Page, ...]
    kept: tuple[Path, ...]


def plan_deletion(folder: Path, drive_root: Path | None) -> RunDeletion:
    """Find what deleting the run in `folder` would take; touches nothing.

    Args:
        folder: The run's folder, `<process>/runs/<name>`.
        drive_root: The Drive root, or None: then no as-built files are looked for.

    Raises:
        ValueError: If `folder` is not a run's folder.
    """
    folder = Path(folder)
    if folder.parent.name != RUNS or not (folder / RUN_FILE).is_file():
        raise ValueError(f"{folder} is not a run's folder")
    files = [path for path in folder.rglob("*") if path.is_file()]
    rows = {path: count for path in _as_built_files(drive_root, folder.parent.parent.name)
            if (count := _count_rows(path, folder.name))}
    return RunDeletion(folder, len(files), sum(path.stat().st_size for path in files), rows)


def delete_run(deletion: RunDeletion) -> RunDeleted:
    """Take the run's as-built rows out, then delete its folder; a failure leaves the run listed, to try again.

    A file left without rows goes, and a day left without records goes with its pages and empty folders.
    """
    name = deletion.folder.name
    removed, rebuild, kept, rows = [], [], [], 0
    for path, _count in deletion.rows.items():
        pages = _day_pages(path)
        rows += _remove_rows(path, name)
        if path.exists():
            rebuild.extend(pages)
        elif any(path.parent.glob(f"{FILE_PREFIX}*.jsonl")):
            kept.extend(page.path for page in pages)
        else:
            for page in pages:
                page.path.unlink()
                removed.append(page.path)
            _remove_if_empty(path.parent)
            _remove_if_empty(_result_folder(path))
    shutil.rmtree(deletion.folder)
    return RunDeleted(rows, tuple(removed), tuple(rebuild), tuple(kept))


def rebuild_page(page: Page, drive_root: Path, process_folder: Path) -> str:
    """Build a page again by its script and return what went wrong, or "".

    Args:
        page: The page.
        drive_root: The Drive root.
        process_folder: The process's folder.
    """
    script = SCRIPTS / page.script
    if not script.is_file():
        return f"{page.path.name} not rebuilt: no script {page.script}"
    result = subprocess.run([sys.executable, str(script), page.day, "--drive", str(drive_root), "--problem",
                             process_folder.name, "--process", str(process_folder)], capture_output=True, text=True)
    if result.returncode == 0:
        return ""
    lines = (result.stdout + result.stderr).strip().splitlines()
    return f"{page.path.name} not rebuilt: {lines[-1] if lines else f'exit code {result.returncode}'}"


def _as_built_files(drive_root: Path | None, problem: str) -> list[Path]:
    """The process's as-built file of every day; a run may go on past midnight, so all of them."""
    if drive_root is None:
        return []
    return sorted((Path(drive_root) / AS_BUILT_FOLDER).glob(f"*{DIR_SUFFIX}/{FILE_PREFIX}{problem}.jsonl"))


def _is_row_of(line: bytes, run: str) -> bool:
    """Whether a line of an as-built file is a record of `run`; a line that can't be read is nobody's, and stays."""
    if run.encode() not in line:
        return False  # most lines: no need to parse them
    try:
        return json.loads(line).get("run_id") == run
    except (ValueError, AttributeError):
        return False


def _count_rows(path: Path, run: str) -> int:
    """How many rows of an as-built file are `run`'s."""
    with open(path, "rb") as file:
        return sum(_is_row_of(line, run) for line in file)


def _remove_rows(path: Path, run: str) -> int:
    """Take `run`'s rows out of an as-built file, atomically, and return how many; a file left without rows goes.

    ! A monitor may be appending to this file. A line appended while this writes makes it start over; one in the
    last instant, between that look and the replace, would be lost.

    Raises:
        OSError: If lines kept being appended.
    """
    temporary = path.with_name(path.name + ".tmp")
    for _ in range(REWRITE_TRIES):
        content = path.read_bytes()
        lines = content.splitlines(keepends=True)
        kept = [line for line in lines if not _is_row_of(line, run)]
        if not kept:
            if path.stat().st_size == len(content):
                path.unlink()
                return len(lines)
            continue
        temporary.write_bytes(b"".join(kept))
        if path.stat().st_size == len(content):
            replace_file(temporary, path)
            return len(lines) - len(kept)
    temporary.unlink(missing_ok=True)
    raise OSError(f"{path} kept changing while its rows were taken out: try again")


def _result_folder(path: Path) -> Path:
    """The result folder of the day of an as-built file."""
    return path.parent.with_name(path.parent.name[:-len(DIR_SUFFIX)] + RESULT_SUFFIX)


def _day_pages(path: Path) -> list[Page]:
    """The report pages that exist of the day of an as-built file."""
    day = path.parent.name[:-len(DIR_SUFFIX)]
    pages = [Page(_result_folder(path) / name.format(day=day), day, script) for name, script in PAGE_SCRIPTS.items()]
    return [page for page in pages if page.path.is_file()]


def _remove_if_empty(folder: Path) -> None:
    """Remove a folder holding nothing; one that holds something, or is gone, stays as it is."""
    if folder.is_dir() and not any(folder.iterdir()):
        folder.rmdir()
