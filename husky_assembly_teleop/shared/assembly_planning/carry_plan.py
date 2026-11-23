"""
A bar's carry planned ahead by tamp and kept beside the bar's roadmap: `<roadmap root>/prm/<process>__<bar>/carry.json`.

With the Transfer workaround off, the travel to load ends on its first row (the loading posture the robot can drive in
with), and the held transfer is planned from there with its path as a seed. Written by husky_assembly_tamp's
`scripts/plan_process_carry.py --save-carry`; read here, and planned again by that script (`CarryReplan`).
"""

from __future__ import annotations

import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from importlib.util import find_spec
from pathlib import Path

from scaffolding_assembly_core.process import Process
from scaffolding_assembly_core.process.solutions import solved_against

from .route_search import Refused

CARRY_FILE = "carry.json"
CARRY_FORMAT = "husky_carry/1"
#: The command that plans a bar's carry and keeps it, with the process folder, the bar and the station filled in.
PLAN_COMMAND = ("husky_assembly_tamp's scripts/plan_process_carry.py <process folder> --bar {bar} --cdfm-backend prm "
                "--goal frames --station X Y YAW_DEG --save-carry (HUSKY_BENCH_ROOT set)")
#: The classes of loading pose a carry can be planned for, as tamp names them, each in the operator's words;
#: ANY_CLASS lets the planner choose.
ANY_CLASS = "all"
LOADING_CLASSES = {"horizontal": "the bar across the front", "vertical": "the bar upright in front",
                   "back": "the bar along the robot, over its back"}
#: Where a carry is planned again: tamp's scene files, plans and logs go here, never into the roadmap root.
REPLAN_ROOT = Path.home() / ".cache" / "husky_assembly_teleop" / "carry_replan"
#: The roadmap's files tamp reads, copied there.
ROADMAP_FILE = "roadmap.json.gz"
ROADMAP_FILES = (ROADMAP_FILE, "config.json", "tracked_*mm.json.gz")
#: Worker processes of a carry planned again, at lowered priority: half the cores, so the monitor keeps ticking.
REPLAN_WORKERS = min(8, max(1, (os.cpu_count() or 2) // 2))
REPLAN_NICE = 10
#: Why tamp found no carry, in the operator's words, by its failure label.
REPLAN_FAILURES = {
    "prm_no_start_of_class": "no loading pose of this class is reached from the insert start",
    "prm_starts_rejected": "every loading pose was refused: it collides, or the base cannot drive in with it",
    "prm_no_start_free": "no loading pose on the roadmap is free in this scene",
    "prm_goal_unholdable": "the arms cannot hold the bar at the insert start from this base",
    "dense_check_collision": "the route found touches something in the final check",
    "goal_in_collision": "the bar collides at the insert start",
}
_FAILED = re.compile(r"FAILED: (\w+)")


@dataclass(frozen=True)
class SavedCarry:
    """A carry read from its file.

    Attributes:
        file: The file.
        joint_names: The twelve arm joints, in the rows' order.
        rows: The path, one tuple of joint values per waypoint; the first is the loading posture.
        station: (x, y, yaw) in metres and radians of the base where the bar is mounted, as the drive in was checked
            from; None when it was not checked.
        base: The base pose the carry was planned at, (x, y, z, qx, qy, qz, qw).
        loading_class: The loading pose's class, "horizontal", "vertical" or "back"; "" when the file does not say.
    """

    file: Path
    joint_names: tuple[str, ...]
    rows: tuple[tuple[float, ...], ...]
    station: tuple[float, float, float] | None
    base: tuple[float, ...]
    loading_class: str = ""

    @property
    def start(self) -> dict[str, float]:
        """dict[str, float]: The loading posture, by joint name."""
        return dict(zip(self.joint_names, self.rows[0]))

    @property
    def end(self) -> dict[str, float]:
        """dict[str, float]: The posture the carry ends in, by joint name."""
        return dict(zip(self.joint_names, self.rows[-1]))


def roadmap_folder(roadmap_root: str, process: Process, bar: str) -> Path:
    """The folder of a bar's roadmap and saved carry: `<roadmap root>/prm/<process folder name>__<bar>/`.

    Args:
        roadmap_root: The folder holding `prm/` (`FreeSearch.roadmap_root`).
        process: The process, read from a folder.
        bar: The bar's id, "bars/B21" or "B21".

    Raises:
        Refused: Without a roadmap root, or a process read from no folder.
    """
    if not roadmap_root:
        raise Refused("no roadmap root: set drive_root or Advanced planning's Roadmap root")
    if process.folder is None:
        raise Refused("the process was read from no folder, so its roadmaps have no name")
    return Path(roadmap_root) / "prm" / f"{process.folder.name}__{bar.removeprefix('bars/')}"


def read_carry(roadmap_root: str, process: Process, action_id: str) -> SavedCarry:
    """The saved carry of an action's bar.

    Args:
        roadmap_root: The folder holding `prm/`.
        process: The process.
        action_id: The jointing action, e.g. "B21_J".

    Raises:
        Refused: No roadmap root, no file (with the command that makes it), not a carry of this action, or planned
            for an older export of the process.
    """
    bar = process.actions[action_id].bar.removeprefix("bars/")
    file = roadmap_folder(roadmap_root, process, bar) / CARRY_FILE
    again = f"plan it with {PLAN_COMMAND.format(bar=bar)}"
    if not file.is_file():
        raise Refused(f"no saved carry for {bar} at {file}: {again}")
    try:
        data = json.loads(file.read_text())
        if data.get("format") != CARRY_FORMAT or data.get("action") != action_id:
            raise ValueError(f"format {data.get('format')!r}, action {data.get('action')!r}")
        names = tuple(str(name) for name in data["joint_names"])
        rows = tuple(tuple(float(value) for value in row) for row in data["path"])
        if not rows or any(len(row) != len(names) for row in rows):
            raise ValueError("its path is empty or does not fit its joints")
        station = data.get("station")
        station = None if station is None else (float(station[0]), float(station[1]), math.radians(float(station[2])))
        against = data.get("solved_against") or {}
        base = tuple(float(value) for value in data["base"])
        if len(base) != 7:
            raise ValueError("its base is not a pose")
    except (OSError, KeyError, IndexError, TypeError, ValueError) as error:
        raise Refused(f"{file} is not the saved carry of {action_id} ({error}): {again}") from error
    now = solved_against(process, action_id)
    if (against.get("process"), against.get("action")) != (now.process, now.action):
        raise Refused(f"the saved carry of {bar} was planned for another export of the process ({file}): {again}")
    return SavedCarry(file, names, rows, station, base, str(data.get("loading_class") or ""))


def carry_script() -> Path:
    """The script of tamp that plans a carry and keeps it.

    Raises:
        Refused: If husky_assembly_tamp is not installed from its source tree (the script is not part of the package).
    """
    spec = find_spec("husky_assembly_tamp")
    script = Path(spec.origin).resolve().parents[1] / "scripts" / "plan_process_carry.py" if spec and spec.origin \
        else None
    if script is None or not script.is_file():
        raise Refused("husky_assembly_tamp's scripts/plan_process_carry.py was not found: install tamp from its "
                      "source tree (pip install -e)")
    return script


class CarryReplan:
    """One bar's carry planned again by tamp's script, as a process of its own; `adopt` keeps what it found.

    - ! tamp writes scene files and plans beside the roadmap it reads, so it works on a copy under `work_root`; the
      roadmap root (often the shared Drive) only gets the new carry.json, from `adopt`.
    - `start`, then `poll` until it ended; `stop` ends it early. Main thread; a reader thread keeps its output.

    Attributes:
        bar: The bar, e.g. "B21".
        loading_class: The class of loading pose asked for, or ANY_CLASS.
        target: The saved carry `adopt` replaces.
        log_file: Everything the script printed.
        command: The script's command line.
        last: Its last progress line.
    """

    def __init__(self, roadmap_root: str, process: Process, action_id: str, station: tuple[float, float, float],
                 loading_class: str = ANY_CLASS, workers: int = REPLAN_WORKERS, work_root: Path = REPLAN_ROOT,
                 script: Path | None = None):
        """Check that the bar has a roadmap and the script exists; nothing is copied or started yet.

        Args:
            roadmap_root: The folder holding `prm/`.
            process: The process, read from a folder.
            action_id: The jointing action, e.g. "B21_J".
            station: (x, y, yaw) in metres and radians of the base where the bar is mounted: the drive in is
                checked from there.
            loading_class: A key of LOADING_CLASSES, or ANY_CLASS.
            workers: Worker processes.
            work_root: The local folder tamp works in.
            script: tamp's script; None finds it (`carry_script`).

        Raises:
            Refused: No roadmap root, no roadmap of the bar, an unknown class, or no script.
        """
        if loading_class != ANY_CLASS and loading_class not in LOADING_CLASSES:
            raise Refused(f"no loading pose class {loading_class!r}")
        self.bar = process.actions[action_id].bar.removeprefix("bars/")
        self.action, self.loading_class, self._process, self._root = action_id, loading_class, process, Path(work_root)
        self._source = roadmap_folder(roadmap_root, process, self.bar)
        if not (self._source / ROADMAP_FILE).is_file():
            raise Refused(f"no roadmap of {self.bar} at {self._source}")
        script = script or carry_script()
        self.target = self._source / CARRY_FILE
        self._work = self._root / "prm" / self._source.name
        self.log_file = self._root / "logs" / f"{self._source.name}__{loading_class}.log"
        self.command = (
            "nice", "-n", str(REPLAN_NICE), sys.executable, str(script), str(process.folder), "--bar", self.bar,
            "--cdfm-backend", "prm", "--goal", "frames", "--prm-root", str(self._root), "--prm-workers", str(workers),
            "--cdfm-home-anchor", loading_class, "--station", f"{station[0]:.4f}", f"{station[1]:.4f}",
            f"{math.degrees(station[2]):.2f}", "--save-carry", "--no-plot", "--save-dir",
            str(self._root / "plans" / self._source.name), "--save-tag", loading_class)
        self._cwd = script.parent.parent
        self.last = ""
        self._reason = ""
        self._child: subprocess.Popen | None = None
        self._started = self._ended = 0.0
        self._stopped = False

    def start(self) -> None:
        """Copy the roadmap's files to the work folder and start the script.

        Raises:
            Refused: If the files cannot be copied or the script cannot be started.
        """
        try:
            self._work.mkdir(parents=True, exist_ok=True)
            self.log_file.parent.mkdir(parents=True, exist_ok=True)
            for pattern in ROADMAP_FILES:
                for file in self._source.glob(pattern):
                    shutil.copyfile(file, self._work / file.name)
            # * A carry left there by an earlier run is not this run's result.
            (self._work / CARRY_FILE).unlink(missing_ok=True)
            env = {**os.environ, "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1",
                   "PYTHONUNBUFFERED": "1"}
            # * A session of its own: `stop` ends the script and its workers together, and a Ctrl-C in the monitor's
            #   terminal is the monitor's alone.
            self._child = subprocess.Popen(self.command, cwd=self._cwd, env=env, stdout=subprocess.PIPE,
                                           stderr=subprocess.STDOUT, text=True, errors="replace",
                                           start_new_session=True)
        except OSError as error:
            raise Refused(f"the carry of {self.bar} could not be planned again: {error}") from error
        self._started = time.monotonic()
        threading.Thread(target=self._read, name="carry-replan", daemon=True).start()

    def _read(self) -> None:
        """Keep the script's output in the log file, and its last progress line and failure label."""
        with open(self.log_file, "w") as log:
            log.write(" ".join(self.command) + "\n")
            for text in self._child.stdout:
                log.write(text)
                log.flush()
                line = text.strip()
                # * tamp tags its own lines, "[plan] ...", "[track] 4/8 chunks": the last one is the progress.
                if line.startswith("[") and not line.startswith("[ssik"):
                    self.last = line
                if found := _FAILED.search(line):
                    self._reason = found.group(1)

    def poll(self) -> int | None:
        """The script's exit code once it ended; None while it runs or before `start`."""
        code = self._child.poll() if self._child is not None else None
        if code is not None and not self._ended:
            self._ended = time.monotonic()
        return code

    @property
    def running(self) -> bool:
        """bool: Whether the script runs."""
        return self._child is not None and self.poll() is None

    @property
    def seconds(self) -> float:
        """float: How long it has run, or ran."""
        return ((self._ended or time.monotonic()) - self._started) if self._started else 0.0

    def stop(self) -> None:
        """End the script and its workers; nothing is kept."""
        if self.running:
            self._stopped = True
            try:
                os.killpg(self._child.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

    def adopt(self) -> SavedCarry:
        """Replace the bar's saved carry by the one the script found, once it ended.

        Returns:
            SavedCarry: The new carry, as read from its file beside the roadmap.

        Raises:
            Refused: It was stopped, it found no carry (with tamp's reason in plain words), or the carry it wrote is
                not this action's; the saved carry is then as it was.
        """
        code = self.poll()
        if self._stopped:
            raise Refused("stopped")
        if code is None:
            raise Refused("still planning")
        if not (self._work / CARRY_FILE).is_file():
            why = REPLAN_FAILURES.get(self._reason, "") or self.last or f"the script ended with code {code}"
            label = f" ({self._reason})" if self._reason in REPLAN_FAILURES else ""
            raise Refused(f"{why}{label}; log: {self.log_file}")
        read_carry(str(self._root), self._process, self.action)
        try:
            shutil.copyfile(self._work / CARRY_FILE, self.target)
        except OSError as error:
            raise Refused(f"the new carry could not be written to {self.target}: {error}") from error
        return read_carry(str(self._source.parents[1]), self._process, self.action)
