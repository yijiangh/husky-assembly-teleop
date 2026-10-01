"""
A design (doc/design_format.md) as the cell plugin steps through it: every movement as a step, and the joints
to draw each robot at. A movement without authored start joints is drawn where its robot last was.

! Runs on a worker thread: nothing here may touch viser, PyBullet or a PluginContext.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from ...design_io.conversion import (DESIGN_FILE, OLD_EXPORT_FILE, convert_export, converted_folder, is_old_export,
                                     is_up_to_date)
from ...design_io import Action, Design, Movement, read
from ...design_io.carry import assumed_start_all
from ...design_io.timing import Stopwatch


@dataclass(frozen=True)
class Step:
    """One movement of one action: the unit the viewer steps through.

    Attributes:
        action: The action it belongs to.
        action_index: Position of the action in the schedule, from 0.
        movement_index: Position of the movement in its action, from 0.
        movement: The movement, as authored.
        assumed_start: The acting robot's joints to draw when the start has none, or None.
        assumed_from: Where `assumed_start` came from (a movement id, "own target", "… (later)").
    """

    action: Action
    action_index: int
    movement_index: int
    movement: Movement
    assumed_start: dict | None = None
    assumed_from: str = ""

    @property
    def label(self) -> str:
        """str: Short name for the panel, e.g. "B3_H_M1_gripper_open"."""
        return self.movement.id


@dataclass(frozen=True)
class CellDesign:
    """A loaded design and its steps.

    Attributes:
        source: The folder it was loaded from.
        design: The design, as read.
        steps: Every movement of every action, in schedule order.
    """

    source: Path
    design: Design
    steps: tuple[Step, ...]

    @property
    def action_count(self) -> int:
        """int: How many actions the schedule has."""
        return len(self.design.schedule)

    def first_step_of(self, action_index: int) -> int:
        """Index into `steps` of the first movement of the action at `action_index` in the schedule."""
        for index, step in enumerate(self.steps):
            if step.action_index == action_index:
                return index
        raise IndexError(f"no action {action_index} in the schedule")


def load_design(folder: Path, data_directory: Path, report: Callable[[str], None] = lambda _text: None,
                watch: Stopwatch | None = None) -> CellDesign:
    """Read a design folder; an old compas_fab export is first converted into `<export>_design`.

    The converted copy is reused while it is newer than the export.

    Args:
        folder: The design folder (holding design.json), or an old export (holding ActionSchedule.json).
        data_directory: The monitor's data directory, for converting an old export's robots.
        report: Told what is being done, for the panel. Called from the loading thread.
        watch: Gets a lap per step, if given.

    Returns:
        CellDesign: The design and its steps; `source` is the converted copy for an old export.

    Raises:
        FileNotFoundError: If the folder is neither a design nor an old export.
    """
    folder = Path(folder).expanduser()
    watch = watch if watch is not None else Stopwatch()
    if is_old_export(folder):
        export, folder = folder, converted_folder(folder)
        if not is_up_to_date(export):
            convert_export(export, folder, data_directory, report, watch)
    if not (folder / DESIGN_FILE).is_file():
        raise FileNotFoundError(f"no {DESIGN_FILE} or {OLD_EXPORT_FILE} in {folder}")
    report(f"reading {folder.name}")
    design = read(folder)
    watch.lap("read design")
    steps = _steps(design)
    watch.lap("steps")
    return CellDesign(source=folder, design=design, steps=steps)


def _steps(design: Design) -> tuple[Step, ...]:
    """Every movement of the schedule as a step, in order, with its assumed start joints."""
    assumed = assumed_start_all(design)
    steps = []
    for action_index, action_id in enumerate(design.schedule):
        action = design.actions[action_id]
        for movement_index, movement in enumerate(action.movements):
            joints, source = assumed.get((action_id, movement.id), (None, ""))
            steps.append(Step(action, action_index, movement_index, movement, joints, source))
    return tuple(steps)


def displayed_joints(step: Step, at_target: bool) -> dict[str, dict[str, float]]:
    """The joint values to draw each robot of a step's start state at.

    Authored joints, else `step.assumed_start` for the acting robot; with `at_target`, the target's joints
    override. Robots not in the state are left out.

    Args:
        step: The step.
        at_target: Use the movement's target joints.

    Returns:
        dict[str, dict[str, float]]: Robot id -> joint name -> value.
    """
    joints = {}
    for robot_id, robot_state in step.movement.start.robots.items():
        if robot_state is None:
            continue
        given = robot_state.joints
        if given is None and robot_id == step.action.robot:
            given = step.assumed_start
        joints[robot_id] = dict(given or {})
    target = step.movement.target
    if at_target and target is not None:
        for robot_id, values in target.joints.items():
            joints.setdefault(robot_id, {}).update(values)
    return joints
