"""
A process (doc/process_format.md) as the assembly's planners step through it: every movement is a step.

A step carries the acting robot's joints to start from when the movement has none authored.

`loaded_process` wraps the core's `Process`, as the process plugin serves it (`ProcessContract`), into a
`LoadedProcess`. Its steps are in `Process.movements()` order, so a step's index is the process plugin's index too.

! Any thread: nothing here may touch viser, PyBullet or a PluginContext.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from scaffolding_assembly_core.process import Action, Movement, Process
from scaffolding_assembly_core.process.carry import assumed_start_all


@dataclass(frozen=True)
class Step:
    """One movement of one action: the unit the planners step through.

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
        """str: Short name for the panel, e.g. "B3_HR_open"."""
        return self.movement.id


@dataclass(frozen=True)
class LoadedProcess:
    """A loaded process and its steps.

    Attributes:
        source: The folder it was loaded from.
        process: The process, as read.
        steps: Every movement of every action, in schedule order.
    """

    source: Path
    process: Process
    steps: tuple[Step, ...]

    @property
    def action_count(self) -> int:
        """int: How many actions the schedule has."""
        return len(self.process.schedule)

    def first_step_of(self, action_index: int) -> int:
        """Index into `steps` of the first movement of the action at `action_index` in the schedule."""
        for index, step in enumerate(self.steps):
            if step.action_index == action_index:
                return index
        raise IndexError(f"no action {action_index} in the schedule")

    def index_of(self, action_id: str, movement_index: int = 0) -> int:
        """Index into `steps` of an action's movement.

        Raises:
            ValueError: If the action is not scheduled.
            IndexError: If the action has no such movement.
        """
        count = len(self.process.actions[action_id].movements)
        if not 0 <= movement_index < count:
            raise IndexError(f"{action_id} has no movement {movement_index}: it has {count}")
        return self.first_step_of(self.process.schedule.index(action_id)) + movement_index


def loaded_process(process: Process) -> LoadedProcess:
    """A process with its steps; `source` is the folder it was read from.

    ! Build it once per `Process` object and keep it: plans and requests compare it by identity, so a new wrapper
      for the same process makes them stale.

    Args:
        process: The process, as the process plugin serves it (`Selection.process`), read from a folder.

    Returns:
        LoadedProcess: The process and its steps.
    """
    return LoadedProcess(source=Path(process.folder), process=process, steps=_steps(process))


def _steps(process: Process) -> tuple[Step, ...]:
    """Every movement of the schedule as a step, in order, with its assumed start joints."""
    assumed = assumed_start_all(process)
    steps = []
    for action_index, action_id in enumerate(process.schedule):
        action = process.actions[action_id]
        for movement_index, movement in enumerate(action.movements):
            joints, source = assumed.get((action_id, movement.id), (None, ""))
            steps.append(Step(action, action_index, movement_index, movement, joints, source))
    return tuple(steps)
