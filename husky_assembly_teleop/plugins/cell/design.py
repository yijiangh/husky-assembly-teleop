"""
The design as authored: robot cells, the action schedule and every movement's
cell state, loaded once from a design folder as read-only data.

The folder holds ActionSchedule.json (action order and robot), BarActions/*.json
(one Action per file) and RobotCell*.json (one cell per robot; the other robots
appear in it as tools named ObstacleRobot<Name>).

! Nothing is computed from the design: each movement's `start_state` is used as authored.

! Runs on a worker thread (see plugin.py), so never touch viser, PyBullet or a
  PluginContext here: loading takes seconds.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Callable

from compas.data import json_load
from compas_fab.robots import RobotCell, RobotCellState
from compas_robots import Configuration

# ! Importing rs_data_structure registers the Action/Movement types; json_load needs them.
from rs_data_structure import Action, Movement

#: The schedule file every design folder must have.
SCHEDULE_FILE = "ActionSchedule.json"

#: Robot cell files: "RobotCell.json" and "RobotCell_<Name>.json".
CELL_FILE_PATTERN = "RobotCell*.json"


@dataclass(frozen=True)
class ScheduledAction:
    """One entry of the action schedule, with its action file loaded.

    Attributes:
        index: Position in the schedule, from 0.
        action_id: As authored, e.g. "B3_H_hold".
        robot: Short robot name from the schedule, e.g. "Alice".
        robot_id: Full id, e.g. "single-arm_husky_Alice"; matches the robot model name of its RobotCell.
        action: The loaded rs_data_structure Action.
    """

    index: int
    action_id: str
    robot: str
    robot_id: str
    action: Action


@dataclass(frozen=True)
class Step:
    """One movement of one action: the unit the viewer steps through.

    Attributes:
        action: The scheduled action the movement belongs to.
        movement_index: Position of the movement in its action, from 0.
        movement: The movement, with its authored start state and target.
        assumed_start: Guessed joint values to draw when the start state has no
            configuration (set by `carry_forward`), else None. Display only.
        assumed_from: Source of `assumed_start` for the panel: a movement id or "own target".
    """

    action: ScheduledAction
    movement_index: int
    movement: Movement
    assumed_start: dict[str, float] | None = None
    assumed_from: str = ""

    @property
    def label(self) -> str:
        """str: Short name for the panel, e.g. "B3_H_M1_gripper_open"."""
        return self.movement.movement_id


@dataclass(frozen=True)
class Design:
    """A whole loaded design folder.

    Attributes:
        folder: Where it was loaded from.
        cells: Robot cell per robot id (the cell's robot model name).
        actions: The schedule, in execution order.
        steps: Every movement of every action, flattened in execution order.
    """

    folder: Path
    cells: dict[str, RobotCell]
    actions: tuple[ScheduledAction, ...]
    steps: tuple[Step, ...]

    def cell_for(self, step: Step) -> RobotCell:
        """The robot cell a step's state belongs to.

        Args:
            step: A step of this design.

        Returns:
            RobotCell: The cell of the robot doing that step's action.
        """
        return self.cells[step.action.robot_id]

    def first_step_of(self, action_index: int) -> int:
        """Index into `steps` of the first movement of an action.

        Args:
            action_index: Position in the schedule.

        Returns:
            int: The step index.
        """
        for index, step in enumerate(self.steps):
            if step.action.index == action_index:
                return index
        raise IndexError(f"no action {action_index} in the schedule")


# --- --- --- --- --- LOADING --- --- --- --- ---

def load_design(folder: Path, report: Callable[[str], None] = lambda _text: None) -> Design:
    """Load a design folder: every robot cell, the schedule and its actions.

    Args:
        folder: The design folder, holding ActionSchedule.json.
        report: Told what is being loaded, for the panel. Called from the
            loading thread, so it should do no more than store the string.

    Returns:
        Design: The loaded design.

    Raises:
        FileNotFoundError: If the folder, its schedule, a robot cell or an
            action file is missing.
        ValueError: If the schedule names a robot that has no cell, or an
            action file belongs to a different robot than the schedule says.
    """
    folder = Path(folder).expanduser()
    schedule_file = folder / SCHEDULE_FILE
    if not schedule_file.is_file():
        raise FileNotFoundError(f"no {SCHEDULE_FILE} in {folder}")
    schedule = json.loads(schedule_file.read_text())

    # * Robot cells, keyed by robot model name (not file name).
    cells: dict[str, RobotCell] = {}
    cell_files = sorted(folder.glob(CELL_FILE_PATTERN))
    if not cell_files:
        raise FileNotFoundError(f"no {CELL_FILE_PATTERN} in {folder}")
    for number, cell_file in enumerate(cell_files, start=1):
        report(f"loading {cell_file.name} ({number}/{len(cell_files)})")
        cell = json_load(str(cell_file))
        cells[cell.robot_model.name] = cell

    # * Load each scheduled action file and check its robot.
    robots = schedule["robots"]
    actions = []
    for entry in schedule["schedule"]:
        report(f"loading {entry['file']}")
        robot_id = robots[entry["robot"]]["robot_id"]
        if robot_id not in cells:
            raise ValueError(f"schedule entry {entry['action_id']!r} is for {robot_id!r}, which has "
                             f"no robot cell; cells found for {', '.join(sorted(cells))}")
        action = json_load(str(folder / entry["file"]))
        if action.robot_id != robot_id:
            raise ValueError(f"{entry['file']} is for {action.robot_id!r}, but the schedule "
                             f"gives it to {entry['robot']!r} ({robot_id!r})")
        actions.append(ScheduledAction(index=len(actions), action_id=entry["action_id"],
                                       robot=entry["robot"], robot_id=robot_id, action=action))

    steps = carry_forward([Step(action=scheduled, movement_index=index, movement=movement)
                           for scheduled in actions
                           for index, movement in enumerate(scheduled.action.movements)])
    return Design(folder=folder, cells=cells, actions=tuple(actions), steps=steps)


# --- --- --- --- --- STATES --- --- --- --- ---

def _joints(configuration: Configuration | None) -> dict[str, float]:
    """A configuration as joint name -> value; empty for None."""
    return dict(zip(configuration.joint_names, configuration.joint_values)) if configuration else {}


def carry_forward(steps: list[Step]) -> tuple[Step, ...]:
    """Give each step without an authored start configuration the robot's last known pose.

    ? Free movements have no start configuration, and drawing them at zero
      joints stretches the arm through the structure.

    - Per robot, an authored start sets the pose, then the step's target moves it (only the joints it names).
    - Before any known pose, a step uses its own target, else the robot's next known start.
    - A robot with no known pose anywhere stays at zero.

    Args:
        steps: Every step, in schedule order.

    Returns:
        tuple[Step, ...]: The same steps, with `assumed_start` / `assumed_from` filled in.
    """
    # Robot id -> (last known joint values, id of the movement they came from).
    last: dict[str, tuple[dict[str, float], str]] = {}
    result = []
    for step in steps:
        movement = step.movement
        robot_id = step.action.robot_id
        authored = _joints(movement.start_state.robot_configuration)
        target = _joints(movement.target_configuration)
        assumed, source = None, ""
        if authored:
            start = authored
        elif robot_id in last:
            assumed, source = last[robot_id]
            start = assumed
        elif target:
            assumed, source = target, "own target"
            start = assumed
        else:
            start = {}
        result.append(replace(step, assumed_start=assumed, assumed_from=source))

        # Where the robot is after this step.
        if start or target:
            last[robot_id] = ({**start, **target}, movement.movement_id)

    # * Steps with no pose and no target: take the robot's next known start (walk backwards).
    upcoming: dict[str, tuple[dict[str, float], str]] = {}
    for index in reversed(range(len(result))):
        step = result[index]
        robot_id = step.action.robot_id
        known = _joints(step.movement.start_state.robot_configuration) or step.assumed_start
        if known:
            upcoming[robot_id] = (known, step.movement.movement_id)
        elif robot_id in upcoming:
            joints, source = upcoming[robot_id]
            result[index] = replace(step, assumed_start=joints, assumed_from=f"{source} (later)")
    return tuple(result)


def full_configuration(cell: RobotCell, configuration: Configuration | None) -> Configuration:
    """Fill a partial or missing configuration up to every joint, with zero for missing ones.

    ? Forward kinematics needs a value for every configurable joint.

    Args:
        cell: The robot cell whose robot the configuration is for.
        configuration: Authored joint values, or None.

    Returns:
        Configuration: Every configurable joint, authored values where given.
    """
    full = cell.zero_full_configuration()
    given = dict(zip(configuration.joint_names, configuration.joint_values)) if configuration else {}
    values = [given.get(name, value) for name, value in zip(full.joint_names, full.joint_values)]
    return Configuration(values, full.joint_types, full.joint_names)


def displayed_state(cell: RobotCell, step: Step, at_target: bool) -> RobotCellState:
    """The state to show for a step: its start, or its start with the target configuration.

    Returns a new state with a full configuration and object frames filled in;
    the authored state is not modified.

    Args:
        cell: The step's robot cell.
        step: The step; `assumed_start` is used when it has no authored start.
        at_target: Use the movement's `target_configuration` instead of the
            start; joints it does not name keep their start value.

    Returns:
        RobotCellState: The state to draw.
    """
    movement = step.movement
    start = movement.start_state
    configuration = full_configuration(cell, start.robot_configuration)
    if start.robot_configuration is None and step.assumed_start:
        configuration = Configuration([step.assumed_start.get(name, value) for name, value
                                       in zip(configuration.joint_names, configuration.joint_values)],
                                      configuration.joint_types, configuration.joint_names)
    if at_target and movement.target_configuration is not None:
        target = dict(zip(movement.target_configuration.joint_names, movement.target_configuration.joint_values))
        configuration = Configuration([target.get(name, value) for name, value
                                       in zip(configuration.joint_names, configuration.joint_values)],
                                      configuration.joint_types, configuration.joint_names)
    # ? Shares the authored tool/body states on purpose: compute_attach_objects_frames
    #   already deep-copies (~40 ms), so a second copy would double the cost.
    state = RobotCellState(robot_base_frame=start.robot_base_frame, robot_configuration=configuration,
                           tool_states=start.tool_states, rigid_body_states=start.rigid_body_states)
    return cell.compute_attach_objects_frames(state)
