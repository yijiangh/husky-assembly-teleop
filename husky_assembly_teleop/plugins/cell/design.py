"""
A design (doc/design_format.md) as the cell plugin steps through it: every movement of the
schedule, and the joints to draw each robot at.

* Everything shown comes from the `design_io.Design`; nothing here builds compas_fab cells or
  states. Planners build those in their own mirrors (world/mirrors/compas_fab.py).
* An export in the old compas_fab format is read with compas and converted in memory
  (`load_old_export`), then shown the same way.
* A movement without authored start joints is drawn where its robot last was
  (`design_io.carry`); the step says so.
! Runs on a worker thread: nothing here touches viser, PyBullet or a PluginContext.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from ...design_conversion import SERIALS, robot_files
from ...design_io import Action, Design, Movement, read
from ...design_io.carry import assumed_start_all
from ...design_io.timing import Stopwatch

#: A design folder has this file; an export in the old compas_fab format has ActionSchedule.json instead.
DESIGN_FILE = "design.json"


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
        design: The design: as read, or converted in memory from an old export.
        steps: Every movement of every action, in schedule order.
        is_old_export: Whether it came from an export in the old compas_fab format.
    """

    source: Path
    design: Design
    steps: tuple[Step, ...]
    is_old_export: bool = False

    @property
    def folder(self) -> Path:
        """Path: Where the design was loaded from."""
        return self.source

    @property
    def action_count(self) -> int:
        """int: How many actions the schedule has."""
        return len(self.design.schedule)

    def first_step_of(self, action_index: int) -> int:
        """Index into `steps` of the first movement of an action.

        Args:
            action_index: Position in the schedule.

        Returns:
            int: The step index.
        """
        for index, step in enumerate(self.steps):
            if step.action_index == action_index:
                return index
        raise IndexError(f"no action {action_index} in the schedule")


def load_design(folder: Path, report: Callable[[str], None] = lambda _text: None,
                watch: Stopwatch | None = None) -> CellDesign:
    """Read a design folder.

    Args:
        folder: The design folder, holding design.json.
        report: Told what is being done, for the panel. Called from the loading thread.
        watch: Gets a lap per step, if given.

    Returns:
        CellDesign: The design and its steps.

    Raises:
        FileNotFoundError: If the folder has no design.json; an old export is named as such.
    """
    folder = Path(folder).expanduser()
    if not (folder / DESIGN_FILE).is_file():
        if (folder / "ActionSchedule.json").is_file():
            raise FileNotFoundError(f"{folder} is an export in the old compas_fab format: use load_old_export")
        raise FileNotFoundError(f"no {DESIGN_FILE} in {folder}")
    watch = watch if watch is not None else Stopwatch()
    report(f"reading {folder.name}")
    design = read(folder)
    watch.lap("read design")
    steps = _steps(design)
    watch.lap("steps")
    return CellDesign(source=folder, design=design, steps=steps)


def load_old_export(folder: Path, data_directory: Path,
                    report: Callable[[str], None] = lambda _text: None,
                    watch: Stopwatch | None = None) -> CellDesign:
    """Read an export in the old compas_fab format and convert it in memory, without writing anything.

    Args:
        folder: The export folder, holding ActionSchedule.json.
        data_directory: The monitor's data directory, for the robots' URDF and SRDF files.
        report: Told what is being done, for the panel. Called from the loading thread.
        watch: Gets a lap per step, if given.

    Returns:
        CellDesign: With `is_old_export` set.
    """
    # ? Imported here: reading the old format needs compas and rs_data_structure, and only this does.
    from ...design_io.legacy import from_export, load_export

    folder = Path(folder).expanduser()
    watch = watch if watch is not None else Stopwatch()
    export = load_export(folder, report, watch)
    report("converting in memory")
    design = from_export(export, robot_files(data_directory), serials=SERIALS, report=lambda _line: None)
    watch.lap("convert in memory")
    steps = _steps(design)
    watch.lap("steps")
    return CellDesign(source=folder, design=design, steps=steps, is_old_export=True)


def _steps(design: Design) -> tuple[Step, ...]:
    """Every movement of the schedule as a step, with its assumed start joints.

    Args:
        design: The design.

    Returns:
        tuple[Step, ...]: In schedule order.
    """
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

    Authored joints where the state has them; for the acting robot without any, `step.assumed_start`;
    with `at_target`, the target's joints replace those they name. Robots not in the state are left
    out, and joints nobody gives are drawn at zero.

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
