"""
Experiment A of doc/base_planner_manual.md: every task with every planner, offline, recorded like the panel's attempts.

For each task of the standard set (`plugins/base_planner/tasks.py`) and each planner in the Algorithm dropdown (default
settings), plans REPEATS times (once for a planner without randomness) and saves each attempt under the experiment, as
the panel does, with environment "offline". Each path found is then driven by the onboard follower in simulation
(`follow.model_groups`): with the robot exactly as the follower's model says, and with deviated models. The traces go
into the attempt's simulated.npz. --resimulate only drives the recorded paths again. Then report it with
scripts/analyze_base_planner.py.

Run with the venv and ROS sourced:

    python3 src/husky-assembly-teleop/scripts/benchmark_base_planner.py --experiment benchmark_1015 \
        [--resimulate] [task ...]
"""

from __future__ import annotations

import argparse
import threading
from dataclasses import asdict
from pathlib import Path

from crl_husky.base_model import load_model

from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.drive import drive_folder, drive_root
from husky_assembly_teleop.plugins.base_planner.algorithms import ALGORITHMS, Debug
from husky_assembly_teleop.plugins.base_planner.follow import model_groups
from husky_assembly_teleop.plugins.base_planner.path import MAX_ANGULAR_SPEED, MAX_LINEAR_SPEED
from husky_assembly_teleop.plugins.base_planner.recording import (RECORDING_FOLDER, attempt_folder,
                                                                  load_description, load_plan_arrays, save_plan,
                                                                  save_simulated)
from husky_assembly_teleop.plugins.base_planner.tasks import PROCESS, TASK_ORIGIN, TASKS
from husky_assembly_teleop.plugins.base_planner.world import PlanningWorld
from husky_assembly_teleop.plugins.planning.panel import with_guesses
from husky_assembly_teleop.plugins.process.steps import load_process
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Scene

#: The robot planned for (Alice), and the repo's data folder with its description.
SERIAL = "0804"
DATA = Path(__file__).resolve().parent.parent / "data"
#: Attempts per task for a planner with randomness (the RRT); a deterministic one (the lattice) plans once.
REPEATS = 5
DETERMINISTIC = ("Lattice (hybrid A*)",)


def main() -> None:
    """Plan every task with every planner, record the attempts, and simulate the follower on each path."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--experiment", required=True, help="Experiment name: attempts are saved under it")
    parser.add_argument("--recordings", type=Path, default=None,
                        help=f"Folder of the experiments (default: {RECORDING_FOLDER} under the Drive root)")
    parser.add_argument("--repeats", type=int, default=REPEATS, help="Attempts per task for a random planner")
    parser.add_argument("--resimulate", action="store_true",
                        help="Only drive the experiment's recorded paths again (all offline attempts with a path)")
    parser.add_argument("tasks", nargs="*", help="Task names; all if none")
    args = parser.parse_args()
    root = drive_root()
    recordings = args.recordings or drive_folder(root, RECORDING_FOLDER)
    if args.resimulate:
        for folder in sorted((recordings / args.experiment).iterdir()):
            description = load_description(folder) if folder.is_dir() else None
            if description and description.get("environment") == "offline" and description["plan"]["found"] \
                    and (not args.tasks or description.get("task") in args.tasks):
                simulate(folder)
                print(f"simulated {folder.name}")
        return
    config = robot_config_from_serial(SERIAL, DATA)
    world = PlanningWorld()
    try:
        for task in TASKS:
            if args.tasks and task.name not in args.tasks:
                continue
            snapshot, own = _scene(task, config, root)
            for kind in ALGORITHMS:
                algorithm = kind()
                for repeat in range(1 if algorithm.label in DETERMINISTIC else args.repeats):
                    world.sync(snapshot)  # the last search moved the robot around
                    folder = _attempt(world, own, task, algorithm, args.experiment, recordings)
                    print(f"{task.name:16s} {algorithm.label:20s} {repeat + 1}: {folder.name}")
    finally:
        world.close()


def _scene(task, config, root: Path) -> tuple[Scene, str]:
    """The task's world and the planned robot's id: Alice with the virtual obstacles, or the process step's scene."""
    if task.virtual:
        own = robot_id(config.serial)
        bodies = task.bodies("task/")
        robots = {own: RobotObject(own, config.model, Pose((*task.start_in_lab()[:2], 0.0)), {})}
        return Scene(bodies={body.id: body for body in bodies},
                     world_poses={body.id: body.placement for body in bodies}, robots=robots), own
    loaded = load_process(root / PROCESS, DATA)
    own = "robots/alice"
    scene, _ = with_guesses(loaded.process.scene_at(loaded.steps[task.process_step].movement), own)
    return scene, own


def _attempt(world: PlanningWorld, own: str, task, algorithm, experiment: str, recordings: Path) -> Path:
    """Plan the task once, record the attempt, and simulate the follower on the path found; returns its folder."""
    settings, debug = algorithm.settings(), Debug()
    start, goal = task.start_in_lab(), task.target_in_lab()
    result = algorithm.plan(world, own, start, goal, threading.Event(), settings, debug)
    setup = {"experiment": experiment, "task": task.name, "robot": "a200-0804", "environment": "offline",
             "conditions": {"task_origin": list(TASK_ORIGIN), "process_step": task.process_step,
                            "speed_caps": {"linear": MAX_LINEAR_SPEED, "angular": MAX_ANGULAR_SPEED}},
             "algorithm": algorithm.label, "settings": asdict(settings), "start": list(start), "goal": list(goal),
             "guessed": []}
    folder = attempt_folder(recordings, experiment, "a200-0804", task.name, "offline")
    save_plan(folder, world, own, setup, result, debug.notes)
    if result.path is not None:
        simulate(folder)
    return folder


def simulate(folder: Path) -> None:
    """Drive an attempt's path with the follower's model exact and deviated (`follow.model_groups`), and save it."""
    description = load_description(folder)
    model_name = description.get("settings", {}).get("follower_model", "alice_tiles")
    poses = load_plan_arrays(folder)["path_poses"]
    save_simulated(folder, model_name,
                   model_groups(poses, load_model(model_name), MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED))


if __name__ == "__main__":
    main()
