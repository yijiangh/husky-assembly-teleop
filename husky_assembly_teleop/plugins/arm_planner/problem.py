"""
A failed arm search saved to a folder, and loaded again to replay it offline (`scripts/replay_arm_plan.py`).

A folder holds the scene planned in (pickled), the request, a readable summary, and each robot's URDF and SRDF: the
monitor writes those to a temporary folder that goes with it. Mesh paths stay absolute: replay on the same checkout.
! Load only folders the monitor wrote: unpickling runs code.
"""

from __future__ import annotations

import json
import math
import pickle
import shutil
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Sequence

from ...config import short_serial
from ...shared.assembly_planning.joints import arm_joint_names
from ...shared.planner.planner_search import PlanResult
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.scene import Scene

#: Where failed searches are saved, a folder per day.
PROBLEM_ROOT = Path("~/.ros/arm_planner").expanduser()
PICKLE, SUMMARY = "problem.pkl", "problem.json"


@dataclass
class Problem:
    """One arm search as it was asked for: the arguments of `plan_arms`, and how it ended.

    Attributes:
        scene: The scene planned in, stand-ins included.
        serial: The robot planned for.
        arms: Its arms searched together.
        goal: Six joint values per arm, radians, the arms after each other.
        reason: Why it failed.
        detail: What blocked it (`PlanResult.detail`).
    """

    scene: Scene
    serial: str
    arms: tuple[str, ...]
    goal: tuple[float, ...]
    reason: str = ""
    detail: str = ""

    @property
    def start(self) -> tuple[float, ...]:
        """The searched joints where the robot was, radians, in the order of `goal`."""
        joints = self.scene.robots[robot_id(self.serial)].joints
        return tuple(float(joints[name]) for arm in self.arms for name in arm_joint_names(arm))


def save_problem(problem: Problem, root: Path | None = None) -> Path:
    """Write a problem to a new folder `<root>/<YYYYMMDD>/<HHMMSS>_<robot>`, root PROBLEM_ROOT by default. Any thread.

    Returns:
        Path: The folder.

    Raises:
        OSError: If it cannot be written.
        pickle.PicklingError: If the scene holds something that cannot be pickled.
    """
    stamp = time.strftime("%H%M%S")
    folder = (root or PROBLEM_ROOT) / time.strftime("%Y%m%d") / f"{stamp}_{short_serial(problem.serial)}"
    for count in range(2, 100):
        if not folder.exists():
            break
        folder = folder.with_name(f"{stamp}_{short_serial(problem.serial)}_{count}")
    folder.mkdir(parents=True)
    for robot in problem.scene.robots.values():
        name = robot.id.replace("/", "_")
        shutil.copyfile(robot.model.urdf, folder / f"{name}.urdf")
        if robot.model.srdf is not None:
            shutil.copyfile(robot.model.srdf, folder / f"{name}.srdf")
    (folder / PICKLE).write_bytes(pickle.dumps(problem, protocol=pickle.HIGHEST_PROTOCOL))
    names = [name for arm in problem.arms for name in arm_joint_names(arm)]
    summary = {"serial": problem.serial, "arms": list(problem.arms), "reason": problem.reason,
               "detail": problem.detail, "saved": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
               "joints_deg": {name: {"start": round(math.degrees(start), 2), "goal": round(math.degrees(goal), 2)}
                              for name, start, goal in zip(names, problem.start, problem.goal)}}
    (folder / SUMMARY).write_text(json.dumps(summary, indent=2) + "\n")
    return folder


def load_problem(folder: Path) -> Problem:
    """Read a problem `save_problem` wrote, each robot's model pointed at the URDF and SRDF copied next to it."""
    folder = Path(folder).expanduser()
    problem: Problem = pickle.loads((folder / PICKLE).read_bytes())
    for robot in problem.scene.robots.values():
        name = robot.id.replace("/", "_")
        srdf = folder / f"{name}.srdf"
        robot.model = replace(robot.model, urdf=folder / f"{name}.urdf", srdf=srdf if srdf.exists() else None)
    return problem


def problem_of(scene: Scene, serial: str, arms: Sequence[str], goal: Sequence[float], result: PlanResult) -> Problem:
    """The problem of a search that ended with `result`."""
    return Problem(scene, serial, tuple(arms), tuple(float(value) for value in goal), result.reason, result.detail)
