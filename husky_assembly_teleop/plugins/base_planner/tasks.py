"""
The base planner experiment's tasks: a start and a target for Alice, and the obstacles in the way. No ROS.

- Virtual tasks put their obstacles into the scene as virtual bodies, so they run in a clean lab as well as offline.
  They are defined in a task frame placed in the lab at TASK_ORIGIN, around a passage 17 degrees off its axes.
- The process task drives into the bar structure of a process step: its scene comes from the process plugin (live)
  or from the process folder (offline); its start and target are in the lab frame.

`TASKS` is the standard set, in report order. doc/base_planner_manual.md says what each tests.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from scaffolding_assembly_core.geometry import Pose, box_geometry
from scaffolding_assembly_core.scene import Body

#: Where the task frame lies in the lab: (x, y, yaw), metres and radians. ! Set it to the middle of a clear area
#: about 5 m by 3.5 m before collecting; it is recorded with every attempt.
TASK_ORIGIN = (0.5, -0.6, 0.0)
#: The passages run along PASSAGE_ANGLE in the task frame.
PASSAGE_ANGLE = math.radians(17.0)
#: Alice's width, the same at every height with her arm stowed (as she is planned), and the planners' default
#: margin, metres: the passages are sized from them.
WHEELS = 0.70
MARGIN = 0.05
#: The process task: its process folder (under the Drive root), step, and start and target in the lab frame.
PROCESS = "data_design_study/261008_2ModulesDemo_purged_process"
PROCESS_STEP = 110
PROCESS_START = (-2.75, -3.32, math.radians(76.61))
PROCESS_TARGET = (0.43, -2.785, 0.0)
#: Colour of virtual obstacles, (r, g, b, a) from 0 to 1.
VIRTUAL_COLOR = (0.85, 0.45, 0.85, 0.6)


@dataclass(frozen=True)
class Task:
    """One task of the standard set.

    Attributes:
        name: Short id, used in folder names and reports.
        title: What it asks, in a few words.
        goal: Which experiment goal it serves: "passages", "obstacles" or "process".
        start: (x, y, yaw) where Alice starts: in the task frame for a virtual task, the lab frame for the process.
        target: (x, y, yaw) to reach, likewise.
        obstacles: Virtual obstacles as (size (x, y, z), pose (x, y, yaw) of their centre at floor level) in the task
            frame; empty for the process task.
        room: Room each side of the narrowest place beyond the margin, metres (negative: too narrow); None if open.
        solvable: Whether a path exists.
        process_step: The process step whose scene the task drives in; None for a virtual task.
    """

    name: str
    title: str
    goal: str
    start: tuple[float, float, float]
    target: tuple[float, float, float]
    obstacles: tuple = field(default=())
    room: float | None = None
    solvable: bool = True
    process_step: int | None = None

    @property
    def virtual(self) -> bool:
        """Whether its obstacles are virtual bodies (else the process gives the scene)."""
        return self.process_step is None

    def start_in_lab(self, origin=TASK_ORIGIN) -> tuple[float, float, float]:
        """The start in the lab frame."""
        return to_lab(self.start, origin) if self.virtual else self.start

    def target_in_lab(self, origin=TASK_ORIGIN) -> tuple[float, float, float]:
        """The target in the lab frame."""
        return to_lab(self.target, origin) if self.virtual else self.target

    def bodies(self, prefix: str, origin=TASK_ORIGIN) -> list[Body]:
        """The virtual obstacles as scene bodies "<prefix><n>", placed in the lab."""
        out = []
        for number, (size, pose) in enumerate(self.obstacles):
            x, y, yaw = to_lab(pose, origin)
            out.append(Body(f"{prefix}{number}", box_geometry(size),
                            Pose((x, y, size[2] / 2.0), (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))),
                            label=f"{self.name} {number}", color=VIRTUAL_COLOR))
        return out


def to_lab(pose, origin=TASK_ORIGIN) -> tuple[float, float, float]:
    """A task-frame pose in the lab frame."""
    ox, oy, oyaw = origin
    x, y, yaw = pose
    return (ox + x * math.cos(oyaw) - y * math.sin(oyaw), oy + x * math.sin(oyaw) + y * math.cos(oyaw), yaw + oyaw)


def _along(along: float, across: float, yaw: float = PASSAGE_ANGLE) -> tuple[float, float, float]:
    """A task-frame pose given along and across the passage axis from its centre."""
    u = np.array([math.cos(PASSAGE_ANGLE), math.sin(PASSAGE_ANGLE)])
    v = np.array([-u[1], u[0]])
    x, y = along * u + across * v
    return float(x), float(y), yaw


def _passage(gap: float, depth: float = 1.0, height: float = 0.2, length: float = 5.0) -> tuple:
    """Two blocks `length` long across the way, `gap` apart, along the passage axis."""
    return tuple(((depth, length, height), _along(0.0, side * (gap + length) / 2.0)) for side in (1, -1))


#: Gaps, by the room each side beyond the margin.
LOW_4CM = WHEELS + 2 * MARGIN + 2 * 0.04
LOW_3CM = WHEELS + 2 * MARGIN + 2 * 0.03
TOO_NARROW_FOR_MARGIN = WHEELS + 2 * MARGIN - 0.01
#: A parked Husky without its arm, (x, y, z) metres.
PARKED = (1.0, 0.7, 0.5)

TASKS = (
    Task("free", "Nothing in the way", "obstacles", (-1.0, 0.8, 0.0), (1.0, -0.8, 1.0)),
    Task("around_robot", "A parked robot in the straight line", "obstacles", (-2.0, 0.0, 0.0), (2.0, 0.0, 0.0),
         ((PARKED, (0.0, 0.0, 0.0)),)),
    Task("beside_wall", "Target facing a wall, 5 cm short of it", "obstacles", (-2.0, 1.25, 0.0),
         (0.0, -0.25, -math.pi / 2), (((2.0, 0.3, 1.0), (0.0, -1.25, 0.0)),)),
    Task("looks_wide", "Gap 1 cm too narrow for the margin; walls short enough to go around", "obstacles",
         _along(-2.0, 0.4, 0.0), _along(2.0, 0.0), _passage(TOO_NARROW_FOR_MARGIN, depth=0.6, length=2.0), -0.005),
    Task("passage", "Through a passage 1 m deep, 4 cm to spare", "passages", _along(-2.0, 0.3, 0.0),
         _along(2.0, 0.0), _passage(LOW_4CM), 0.04),
    Task("narrow_passage", "Through a passage 1.5 m deep, 3 cm to spare, starting beside it facing across",
         "passages", _along(-2.0, 1.0, math.pi / 2), _along(2.2, -0.5, PASSAGE_ANGLE - 0.5),
         _passage(LOW_3CM, depth=1.5), 0.03),
    Task("tall_walls", "Between walls taller than Alice, 4 cm to spare, starting off-centre", "passages",
         _along(-2.2, -0.27, 0.0), _along(2.2, 0.0), _passage(LOW_4CM, height=1.2), 0.04),
    Task("into_passage", "Into the middle of the narrow passage", "passages", _along(-2.0, 1.0, math.pi / 2),
         _along(0.2, 0.0), _passage(LOW_3CM, depth=1.5), 0.03),
    Task("into_facing_out", "Into the narrow passage, ending facing back out", "passages",
         _along(-2.0, 1.0, math.pi / 2), _along(0.2, 0.0, PASSAGE_ANGLE + math.pi), _passage(LOW_3CM, depth=1.5),
         0.03),
    Task("out_of_passage", "Out of the middle of the narrow passage", "passages", _along(0.2, 0.0),
         _along(2.2, -0.5, PASSAGE_ANGLE - 0.5), _passage(LOW_3CM, depth=1.5), 0.03),
    Task("too_narrow", "Passage narrower than Alice, no way around", "passages", _along(-2.0, 0.0), _along(2.0, 0.0),
         _passage(WHEELS - 0.1), -0.1, solvable=False),
    Task(f"step_{PROCESS_STEP}", f"Into the bar structure at process step {PROCESS_STEP}", "process", PROCESS_START,
         PROCESS_TARGET, process_step=PROCESS_STEP),
)

#: Tasks by name.
BY_NAME = {task.name: task for task in TASKS}
