"""Recorded plan attempts (plugins/base_planner/recording.py), and the experiment's tasks (tasks.py)."""

import math
import threading

import numpy as np
import pytest
from crl_husky.base_model import load_model

from husky_assembly_teleop.plugins.base_planner.algorithms import ALGORITHMS, Debug
from husky_assembly_teleop.plugins.base_planner.algorithms.lattice import Settings, plan
from husky_assembly_teleop.experiments.base_planner.analysis import analyse
from husky_assembly_teleop.plugins.base_planner.algorithms.base import FOLLOWER_MODEL
from husky_assembly_teleop.plugins.base_planner.follow import DEVIATED, EXACT, XICR_SPREAD, model_groups
from husky_assembly_teleop.plugins.base_planner.recording import (attempt_folder, load_description,
                                                                  load_plan_arrays, save_plan)
from husky_assembly_teleop.plugins.base_planner.tasks import BY_NAME, TASKS, to_lab
from test_base_planner_rrt import _snapshot, alice, robots, world  # noqa: F401  (fixtures)


def _setup(start, goal) -> dict:
    """What the plugin knows of an attempt on the main thread."""
    return {"experiment": "trial", "task": None, "robot": "alice", "environment": "sim",
            "algorithm": "Lattice (hybrid A*)", "settings": {"margin": Settings().margin},
            "start": list(start), "goal": list(goal), "guessed": []}


def test_found_plan_is_saved_with_its_path_clearance_and_floor(tmp_path, world, robots, alice):  # noqa: F811
    """A plan found: its time, length, clearance and notes in experiment.json, path and floor in plan.npz."""
    world.sync(_snapshot(robots))
    start, goal = (0.0, 0.0, 0.0), (0.0, -2.0, 1.0)
    result = plan(world, alice, start, goal, threading.Event())
    folder = attempt_folder(tmp_path, "trial", "alice", None, "sim")
    save_plan(folder, world, alice, _setup(start, goal), result, ["one note"])

    description, arrays = load_description(folder), load_plan_arrays(folder)
    assert folder.parent.name == "trial" and folder.name.endswith("_alice_free_sim")
    found = description["plan"]
    assert found["found"] and found["notes"] == ["one note"] and description["run"] is None
    assert found["seconds"] == pytest.approx(result.seconds) and found["length"] == pytest.approx(result.path.length)
    assert 0.0 < found["clearance"] <= 0.30
    assert arrays["path_poses"].shape == result.path.points.shape
    assert arrays["floor_heights"].ndim == 2 and arrays["floor_cell"] == pytest.approx(0.05)


def test_failed_plan_is_saved_with_why(tmp_path, world, robots, alice):  # noqa: F811
    """A target inside Cindy: no path, the reason kept, no clearance, the floor still mapped."""
    world.sync(_snapshot(robots))
    start, goal = (0.0, 0.0, 0.0), (2.0, 0.2, 0.0)
    result = plan(world, alice, start, goal, threading.Event())
    folder = attempt_folder(tmp_path, "trial", "alice", "passage", "real")
    save_plan(folder, world, alice, _setup(start, goal), result, [])

    found = load_description(folder)["plan"]
    assert not found["found"] and "target" in found["reason"] and found["clearance"] is None
    assert load_plan_arrays(folder)["path_poses"].shape == (0, 3)


def test_attempt_folders_never_clash(tmp_path):
    """Two attempts in the same second get two folders, both made at once."""
    first = attempt_folder(tmp_path, "trial", "alice", "free", "real")
    second = attempt_folder(tmp_path, "trial", "alice", "free", "real")
    assert first.is_dir() and second.is_dir() and second != first


def test_tasks_are_unique_and_placed_in_the_lab():
    """Task names are unique; the task frame turns and moves poses as the origin says."""
    assert len({task.name for task in TASKS}) == len(TASKS)
    x, y, yaw = to_lab((1.0, 0.0, 0.0), (2.0, 3.0, math.pi / 2))
    np.testing.assert_allclose((x, y, yaw), (2.0, 4.0, math.pi / 2), atol=1e-12)
    for task in TASKS:
        assert task.virtual == bool(task.obstacles) or task.name == "free"


@pytest.mark.slow
def test_report_compares_the_planners_on_a_task(tmp_path, world, robots, alice):  # noqa: F811
    """Two planners on the free task: the report has every section, both in the overview, one CSV per table."""
    world.sync(_snapshot(robots))
    task = BY_NAME["free"]
    for kind in ALGORITHMS:
        algorithm, debug = kind(), Debug()
        start, goal = task.start_in_lab(), task.target_in_lab()
        result = algorithm.plan(world, alice, start, goal, threading.Event(), algorithm.settings(), debug)
        setup = {**_setup(start, goal), "task": "free", "environment": "offline", "algorithm": algorithm.label}
        save_plan(attempt_folder(tmp_path, "trial", "alice", "free", "offline"), world, alice, setup, result, [])

    summary = analyse(tmp_path / "trial").read_text()
    for section in ("## Why", "## What we measured", "## Overview", "## Planning", "## Failures", "## Checks"):
        assert section in summary, section
    for algorithm in ALGORITHMS:
        assert f"| {algorithm.label} |" in summary
    assert (tmp_path / "trial" / "_analysis" / "attempts.csv").read_text().count("\n") == 1 + len(ALGORITHMS)
    assert (tmp_path / "trial" / "_analysis" / "results_prompt.md").is_file()


def test_model_groups_drive_the_exact_model_first_then_the_deviated_ones():
    """One drive with the follower's own xICR, then the spread either side of it; a straight drive finishes in all."""
    model = load_model(FOLLOWER_MODEL)
    drives = model_groups([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)], model, 0.2, 0.3)
    assert [(group, x_icr) for group, x_icr, _, _ in drives] == \
        [(EXACT, model["xICR"]), (DEVIATED, model["xICR"] - XICR_SPREAD), (DEVIATED, model["xICR"] + XICR_SPREAD)]
    assert all(why == "" and trace[-1][0] > 0.9 for _, _, trace, why in drives)
