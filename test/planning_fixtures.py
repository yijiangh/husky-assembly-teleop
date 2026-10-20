"""Shared data for the assembly_planning tests: the purged two-module process from the Drive, and the robot configs.

The process needs HUSKY_DRIVE_ROOT (the local "2025-03 Husky Assembly" folder); tests using it are skipped without.
Both are loaded once per test run; treat them as read-only.
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import pytest

from husky_assembly_teleop.config import RobotConfig, robot_config_from_serial
from husky_assembly_teleop.drive import drive_root
from husky_assembly_teleop.plugins.process.loading import load_process
from husky_assembly_teleop.shared.assembly_planning.process_steps import LoadedProcess, Step, loaded_process
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.process import Action, Movement
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Scene

DATA = Path(__file__).resolve().parent.parent / "data"
#: The purged process, under the Drive root.
PURGED = "data_design_study/261008_2ModulesDemo_purged_process"
ALICE, BELLE, CINDY = "0804", "0805", "0806"


@lru_cache(maxsize=1)
def _load_purged(folder: Path) -> LoadedProcess:
    """The purged process, read once."""
    return loaded_process(load_process(folder, DATA))


def purged_process() -> LoadedProcess:
    """The purged process; skips the test without the Drive."""
    root = drive_root()
    if root is None or not (root / PURGED).is_dir():
        pytest.skip(f"needs HUSKY_DRIVE_ROOT with {PURGED}")
    return _load_purged(root / PURGED)


@lru_cache(maxsize=None)
def robot_config(serial: str) -> RobotConfig:
    """One robot's config by its serial digits, e.g. "0806", built once."""
    return robot_config_from_serial(serial, DATA)


def action(loaded: LoadedProcess, action_id: str) -> Action:
    """One action of a loaded process by id."""
    return loaded.process.actions[action_id]


def movement(loaded: LoadedProcess, movement_id: str) -> tuple[Action, Movement]:
    """One movement of a loaded process by id ("B21_J_insert"), with its action."""
    return next((each, moving) for each, moving in loaded.process.movements() if moving.id == movement_id)


def step(loaded: LoadedProcess, movement_id: str) -> Step:
    """The step of one movement by id."""
    return next(each for each in loaded.steps if each.movement.id == movement_id)


def design_scene(loaded: LoadedProcess, movement_id: str, serials: tuple[str, ...] = (ALICE, BELLE, CINDY)) -> Scene:
    """A snapshot of the real robots measured where the design has them at a movement's start.

    Robots absent from the start, or without a base there, are left out; null joints are the acting robot's assumed
    start, else empty.
    """
    current = step(loaded, movement_id)
    planned = {spec.serial: robot for robot, spec in loaded.process.robots.items()}
    robots = {}
    for serial in serials:
        config = robot_config(serial)
        state = current.movement.start.robots.get(planned.get(serial))
        if state is None or state.base is None:
            continue
        assumed = current.assumed_start if planned[serial] == current.action.robot else None
        robots[robot_id(config.serial)] = RobotObject(robot_id(config.serial), config.model, state.base,
                                                      dict(state.joints or assumed or {}))
    return Scene(robots=robots)
