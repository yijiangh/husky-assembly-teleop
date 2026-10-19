"""
A process (doc/process_format.md) as the process plugin steps through it, and what it offers other plugins.

Every movement is a step, with the joints to draw each robot at. A movement without authored start joints is
drawn where its robot last was.
`scene_bodies` turns a step's process scene (`Process.scene_at`) into scene bodies: the core draws them, and planners
avoid them. Bodies the plan holds follow the configured real robot. `live_ids` maps process ids to scene ids and back.

! Runs on a worker thread: nothing here may touch viser, PyBullet or a PluginContext.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Callable, Protocol

from scaffolding_assembly_core.legacy.conversion import (OLD_EXPORT_FILE, PROCESS_FILE, convert_export,
                                                         converted_folder, is_old_export, is_up_to_date)
from scaffolding_assembly_core.process import Action, Holder, Movement, Process, State, read
from scaffolding_assembly_core.process.carry import assumed_start_all
from scaffolding_assembly_core.process.relations import is_present, placement
from scaffolding_assembly_core.legacy.timing import Stopwatch
from scaffolding_assembly_core.ids import IdMap
from scaffolding_assembly_core.scene import retarget
from scaffolding_assembly_core.scene import Attachment, Body
from scaffolding_assembly_core.ids import robot_id as scene_robot_id

if TYPE_CHECKING:
    from ..config import RobotConfig

#: Colours of bodies nobody holds, RGB 0-255.
BAR_COLOR = (205, 170, 110)
JOINT_COLOR = (120, 120, 140)
GROUND_COLOR = (170, 180, 170)


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


@dataclass(frozen=True)
class Selection:
    """The process plugin's selected step, as other plugins read it; replaced whenever any field changes, never edited.

    Attributes:
        loaded: The loaded process.
        index: Position of `step` in `loaded.steps`.
        step: The selected movement.
        ids: Process id -> live scene id ("bars/B1" -> "process/bars/B1", planned robot -> real robot), or None
            until the step's scene bodies are put.
        process_ids: Live scene id -> process id, or None likewise.
    """

    loaded: LoadedProcess
    index: int
    step: Step
    ids: IdMap | None = None
    process_ids: IdMap | None = None


class ProcessSource(Protocol):
    """What the process plugin offers: type `ctx.require("process")` as this."""

    @property
    def selection(self) -> Selection | None:
        """Selection | None: The selected step, or None while no process with steps is loaded."""


def load_process(folder: Path, data_directory: Path, report: Callable[[str], None] = lambda _text: None,
                 watch: Stopwatch | None = None) -> LoadedProcess:
    """Read a process folder; an old compas_fab export is first converted into `<export>_process`.

    The converted copy is reused while it is newer than the export.

    Args:
        folder: The process folder (holding process.json), or an old export (holding ActionSchedule.json).
        data_directory: The monitor's data directory, for converting an old export's robots.
        report: Told what is being done, for the panel. Called from the loading thread.
        watch: Gets a lap per step, if given.

    Returns:
        LoadedProcess: The process and its steps; `source` is the converted copy for an old export.

    Raises:
        FileNotFoundError: If the folder is neither a process nor an old export.
    """
    folder = Path(folder).expanduser()
    watch = watch if watch is not None else Stopwatch()
    if is_old_export(folder):
        export, folder = folder, converted_folder(folder)
        if not is_up_to_date(export):
            convert_export(export, folder, data_directory, report, watch)
    if not (folder / PROCESS_FILE).is_file():
        raise FileNotFoundError(f"no {PROCESS_FILE} or {OLD_EXPORT_FILE} in {folder}")
    report(f"reading {folder.name}")
    process = read(folder)
    watch.lap("read process")
    steps = _steps(process)
    watch.lap("steps")
    return LoadedProcess(source=folder, process=process, steps=steps)


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


def body_color(body_id: str) -> tuple[int, int, int]:
    """The colour of a body nobody holds, RGB 0-255: bar, ground, or anything else (joint halves, obstacles)."""
    if body_id.startswith("bars/"):
        return BAR_COLOR
    if body_id.startswith("ground/"):
        return GROUND_COLOR
    return JOINT_COLOR


def stands(process: Process, state: State, body_id: str) -> bool:
    """Whether a body stands in a state on its own: present, and not an attached, unbuilt bar or one of its halves.

    * Standing bodies go into the scene enabled; the plugin's overlay (`drawing.py`) draws the rest.
    """
    return is_present(process, state, body_id) and not isinstance(placement(process, state, body_id), Holder)


def live_ids(process: Process, robots: tuple[RobotConfig, ...], prefix: str) -> tuple[IdMap, IdMap]:
    """Map process ids to live scene ids, and back.

    A body becomes "<prefix><body id>"; a planned robot (and its links) becomes the configured robot with its serial,
    and each planned tool the real tool on the same flange. Robots not configured are not mapped.

    Args:
        process: The process.
        robots: The configured robots, `ctx.config.robots`.
        prefix: Id prefix, the owning plugin's "<name>/".

    Returns:
        tuple[IdMap, IdMap]: Process id -> live id, and live id -> process id.

    Raises:
        ValueError: If two process ids map to the same live id (e.g. two planned robots with one serial).
    """
    configured = {config.serial[-4:]: config for config in robots if config.model is not None}
    pairs = {body_id: f"{prefix}{body_id}" for body_id in process.bodies}
    for planned, spec in process.robots.items():
        config = configured.get(spec.serial or "")
        if config is None:
            continue
        pairs[planned] = scene_robot_id(config.serial)
        for flange, tool_id in spec.tools.items():
            if flange in config.model.tools:
                pairs[process.tools[tool_id].id] = config.model.tools[flange].id
    back: dict[str, str] = {}
    for own, live in pairs.items():
        if back.setdefault(live, own) != own:
            raise ValueError(f"{back[live]!r} and {own!r} both map to {live!r}: the way back would be ambiguous")
    return IdMap(pairs), IdMap(back)


def scene_bodies(loaded: LoadedProcess, step: Step, robots: tuple[RobotConfig, ...], prefix: str) -> list[Body]:
    """Every body of a step's process scene, as a scene body "<prefix><body id>".

    The scene is edited for the live world:
    - a held body is attached to the configured robot with the planned robot's serial (`retarget`); held by a robot
      not configured, it is disabled where the plan has it;
    - `touches` name the real robots, their tools and the prefixed ids; those of robots not configured are dropped.

    * Every step gives the same ids, geometry objects and colours, so stepping never rebuilds a body in a mirror or
      the 3D view: it only moves, enables and disables them.

    Args:
        loaded: The loaded process.
        step: The step; its movement's start is the state.
        robots: The configured robots, `ctx.config.robots`.
        prefix: Id prefix, the owning plugin's "<name>/".

    Returns:
        list[Body]: One scene body per process body.

    Raises:
        ValueError: If a configured robot lacks a link the plan holds a body by, or two process ids map to one
            live id (`live_ids`).
    """
    scene = loaded.process.scene_at(step.movement)
    ids, _ = live_ids(loaded.process, robots, prefix)
    models = {scene_robot_id(config.serial): config.model for config in robots if config.model is not None}
    held = {body_id: body.placement for body_id, body in scene.bodies.items()
            if isinstance(body.placement, Attachment)}
    moved = retarget({body_id: held[body_id] for body_id in held if held[body_id].parent in ids}, ids, models)

    bodies = []
    for body_id, body in scene.bodies.items():
        placement = moved.get(body_id) or scene.world_poses.get(body_id,
                                                                loaded.process.bodies[body_id].assembly_pose)
        enabled = body.enabled and (body_id not in held or body_id in moved)
        bodies.append(Body(ids(body_id), body.geometry, placement,
                           touches=tuple(ids(other) for other in body.touches if other in ids),
                           label=body.label or body_id, color=tuple(c / 255 for c in body_color(body_id)) + (1.0,),
                           enabled=enabled))
    return bodies
