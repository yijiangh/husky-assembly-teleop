"""
Recorded plan attempts and runs of the base planner: one folder per attempt, under its experiment.

- Folder: <Drive root>/RECORDING_FOLDER/<experiment>/<date>_<time>_<robot>_<task or free>_<real|sim|offline>/
- experiment.json: the attempt's setup (experiment, task, robot, environment, conditions, algorithm and settings,
  start, target), its plan (found or not, why, time, length, clearance, notes) and its run once committed (outcome,
  controller), or null.
- plan.npz: the path and the floor around it; recording.npz: the run's samples (the plugin); simulated.npz: an
  offline attempt's follower traces, with the exact and deviated models (`save_simulated`).

! Record raw, decide later: the analysis recomputes every metric from these files.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np

from ..planning.search import PlanResult
from .path import densify
from .world import PlanningWorld

#: Recorded attempts, under the Drive root (`drive.py`); ! create it in Drive, it is never created here.
RECORDING_FOLDER = Path("data_experiment/base_planner")
#: The floor map saved with a plan: the box around start, target and path grown by FLOOR_PADDING metres, in cells of
#: FLOOR_CELL metres, of things from FLOOR_BOTTOM metres up.
FLOOR_PADDING = 1.5
FLOOR_CELL = 0.05
FLOOR_BOTTOM = 0.05
#: The path's clearance is measured at poses this far apart, metres and radians.
CLEARANCE_SPACING = 0.05
#: The description file of an attempt.
DESCRIPTION = "experiment.json"


def attempt_folder(recordings: Path, experiment: str, serial: str, task: str | None, environment: str) -> Path:
    """A new folder for an attempt (not yet created): under the experiment, named by time, robot, task and where."""
    name = f"{time.strftime('%Y%m%d_%H%M%S')}_{serial}_{task or 'free'}_{environment}"
    folder, count = recordings / experiment / name, 2
    while folder.exists():
        folder, count = recordings / experiment / f"{name}_{count}", count + 1
    return folder


def save_plan(folder: Path, world: PlanningWorld | None, own: str, setup: dict, result: PlanResult,
              notes: list[str]) -> None:
    """Measure the path's clearance and the floor around it, and write experiment.json and plan.npz. Worker thread.

    Args:
        folder: The attempt's folder, created here.
        world: The planning world as the search left it; None if the search never synced it (nothing measured).
        own: The robot planned for, by id.
        setup: What the main thread knew: experiment, task, robot, environment, conditions, algorithm, settings,
            start, target ("goal").
        result: The search's result.
        notes: The search's notes (`Debug.notes`).

    Raises:
        OSError: If the files cannot be written.
    """
    path = result.path
    poses = np.empty((0, 3)) if path is None else np.asarray(path.points, dtype=float)
    plan = {"found": path is not None, "reason": result.reason, "seconds": result.seconds, "direct": result.direct,
            "length": None if path is None else path.length, "duration": None if path is None else path.duration,
            "waypoints": len(poses), "clearance": None, "clearance_at": None, "notes": notes}
    arrays = {"path_poses": poses, "path_t": np.empty(0) if path is None else np.asarray(path.times)}
    if world is not None and world.mirror is not None:
        if path is not None:
            started = time.time()
            points = [tuple(poses[0])] + densify([tuple(pose) for pose in poses], CLEARANCE_SPACING, CLEARANCE_SPACING)
            plan["clearance"], plan["clearance_at"] = world.clearance(own, points)
            plan["clearance_seconds"] = time.time() - started
        corners = np.vstack([np.array([setup["start"][:2], setup["goal"][:2]]), poses[:, :2]])
        low = corners.min(axis=0) - FLOOR_PADDING
        shape = tuple(int(math.ceil(size / FLOOR_CELL)) + 1 for size in corners.max(axis=0) + FLOOR_PADDING - low)
        arrays.update(floor_heights=world.floor_heights(own, low, FLOOR_CELL, shape, FLOOR_BOTTOM).astype(np.float32),
                      floor_low=low, floor_cell=np.float64(FLOOR_CELL))
    folder.mkdir(parents=True)
    np.savez_compressed(folder / "plan.npz", **arrays)
    write_description(folder, {**setup, "plan": plan, "run": None})


def save_simulated(folder: Path, follower_model: str, drives: list) -> None:
    """Write an attempt's simulated drives (`follow.model_groups`) to simulated.npz and its experiment.json.

    Args:
        folder: The attempt's folder, with its experiment.json.
        follower_model: The follower's robot model, by name.
        drives: (group, true xICR, trace, why not finished) per drive.
    """
    np.savez_compressed(folder / "simulated.npz", **{f"trace_{k}": drive[2] for k, drive in enumerate(drives)})
    description = load_description(folder)
    description["simulated"] = {"follower_model": follower_model,
                                "drives": [{"group": group, "xicr": x_icr, "not_finished": why}
                                           for group, x_icr, _, why in drives]}
    write_description(folder, description)


def write_description(folder: Path, description: dict) -> None:
    """Write an attempt's experiment.json."""
    (folder / DESCRIPTION).write_text(json.dumps(description, indent=2, default=_plain) + "\n")


def load_description(folder: Path) -> dict | None:
    """An attempt's experiment.json; None if the folder has none."""
    path = folder / DESCRIPTION
    return json.loads(path.read_text()) if path.is_file() else None


def load_plan_arrays(folder: Path) -> dict | None:
    """An attempt's plan.npz arrays; None if it has none."""
    if not (folder / "plan.npz").is_file():
        return None
    with np.load(folder / "plan.npz") as data:
        return {key: data[key] for key in data.files}


def _plain(value):
    """JSON fallback: numpy numbers and arrays as plain Python."""
    return value.item() if isinstance(value, np.generic) else np.asarray(value).tolist()
