"""
Read an old compas_fab export into a schema 2 `Design`.

The export is RobotCell*.json, BarActions/, ActionSchedule.json and WalkableGround.json.

Robots come from the given URDF and SRDF files, checked joint by joint against the models in the cells.
! Slow (~15 s for three ~350 MB cells). Imports compas and rs_data_structure.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Mapping, Optional, Tuple

import numpy as np
from compas.data import json_load
from compas_fab.robots import RobotCell, RobotCellState
from compas_robots import RobotModel

# ! Registers the dtypes of the action and movement classes, so json_load can rebuild them.
import rs_data_structure  # noqa: F401

from ..design.types import (Action, BodySpec, Carried, Design, LineSpec, Movement, Producer, RobotSpec, RobotState,
                            State, Target)
from ..design.version import writer_info
from ..design.vocabulary import ON, released
from ..geometry import Geometry, Pose, TriMesh
from ..ids import link_id
from ..kinematics import ForwardKinematics
from ..mirrors.compas import PARKED_POSITION, pose_from_frame
from ..robot import Tool
from ..urdf import movable_joints, urdf_joints, urdf_links
from .timing import Stopwatch

#: Movement class -> (schema 1 movement type, coupled).
MOVEMENT_KINDS: Dict[str, Tuple[str, bool]] = {
    "IndependentDualArmFreeMovement": ("free", False),
    "EndEffectorConstrainedDualArmFreeMovement": ("free", True),
    "EndEffectorConstrainedDualArmLinearMovement": ("linear", True),
    "IndependentDualArmLinearMovement": ("linear", False),
    "SingleArmFreeMovement": ("free", False),
    "SingleArmLinearMovement": ("linear", False),
    "ManualMovement": ("manual", False),
    "ScaffoldingToolMovement": ("tool", False),
    "GripperToolMovement": ("tool", False),
}

#: Action class -> action type.
ACTION_KINDS: Dict[str, str] = {
    "BarAssemblyJointingAction": "bar_jointing",
    "BarAssemblyReleaseAction": "bar_release",
    "BarHoldingAction": "bar_holding",
    "BarHoldingReleaseAction": "bar_holding_release",
}

#: Notes that became fields, or are planner status that belongs in `solutions/` (format §5.6); the rest stay notes.
_CONSUMED_NOTES = ("bar_pose_is_placeholder", "lm_axis", "lm_distance_mm", "retreat_axes_world", "ends_on",
                   "constraint", "bar_arm_side", "planner_fills", "start_config_is_none", "unplanned_offline",
                   "goal_backfilled_from")

#: Schema 1 controller -> schema 2 controller (None: no arm moves).
_CONTROLLERS = {"joint_tracking": "position", "cartesian_compliant": "compliant", "none": None}

#: Schema 1 tool action -> (channel, value) it ends in.
_TOOL_ACTIONS = {"grasp": ("grip", "closed"), "close": ("grip", "closed"), "ungrasp": ("grip", "open"),
                 "open": ("grip", "open"), "tighten": ("joint", "tight"), "untighten": ("joint", "loose")}

#: Depth of the slab the walkable ground polygons are extruded into, downward, metres.
GROUND_THICKNESS = 0.05

#: Coordinates beyond this are millimetres (WalkableGround.json is written in Rhino's units).
_MILLIMETRE_THRESHOLD = 50.0

#: How far outside a ground body's floor area a ground connector may stand and still rest on it, metres.
_GROUND_MARGIN = 0.05

#: Poses closer than this are the same, metres or quaternion units.
_SAME = 1e-6


def default_tool_kind(name: str) -> str:
    """The end effector kind of a Rhino tool name, e.g. "AT3L" or "SupportGripper".

    Raises:
        KeyError: For a name this rule does not know.
    """
    if name.startswith("AT3"):
        return "scaffolding_v3"
    if name == "SupportGripper":
        return "robotiq"
    raise KeyError(f"no tool kind known for tool {name!r}; pass tool_kind=")


# --- --- --- --- --- ENTRY POINT --- --- --- --- ---

@dataclass(frozen=True)
class Export:
    """An export folder as compas_fab and rs_data_structure objects, as loaded.

    Attributes:
        folder: The export folder.
        schedule: ActionSchedule.json, as plain JSON.
        cells: Robot model name ("dual-arm_husky_Cindy") -> its RobotCell.
        actions: (schedule entry, loaded action) in schedule order.
    """

    folder: Path
    schedule: dict
    cells: Dict[str, RobotCell]
    actions: Tuple[tuple, ...]

    def robot_id(self, name: str) -> str:
        """Our id of a robot named in the schedule: "Cindy" -> "robots/cindy"."""
        return f"robots/{name.lower()}"


def load_export(folder, report: Callable[[str], None] = print, watch: Optional[Stopwatch] = None) -> Export:
    """Load an export's cells, schedule and scheduled actions with compas.

    Args:
        folder: The export folder, holding ActionSchedule.json.
        report: Told what is being loaded.
        watch: Gets a lap per cell file and one for the actions, if given.
    """
    folder = Path(folder)
    schedule = json.loads((folder / "ActionSchedule.json").read_text())
    cells = {}
    for path in sorted(folder.glob("RobotCell*.json")):
        report(f"loading {path.name}")
        cell = json_load(str(path))
        cells[cell.robot_model.name] = cell
        if watch is not None:
            watch.lap(f"load {path.name}")
    report("loading the actions")
    actions = tuple((entry, json_load(str(folder / entry["file"]))) for entry in schedule["schedule"])
    if watch is not None:
        watch.lap("load actions")
    return Export(folder=folder, schedule=schedule, cells=cells, actions=actions)


def read_legacy(folder, robot_files: Mapping[str, Tuple[Path, Path]],
                serials: Optional[Mapping[str, str]] = None,
                tool_kind: Callable[[str], str] = default_tool_kind,
                report: Callable[[str], None] = print) -> Design:
    """Read an export folder into an in-memory design (`folder` None): `load_export`, then `from_export`."""
    report(f"reading {folder}")
    return from_export(load_export(folder, report), robot_files, serials, tool_kind, report)


def from_export(export: Export, robot_files: Mapping[str, Tuple[Path, Path]],
                serials: Optional[Mapping[str, str]] = None,
                tool_kind: Callable[[str], str] = default_tool_kind,
                report: Callable[[str], None] = print) -> Design:
    """Convert a loaded export into an in-memory design (`folder` None); the export is not modified.

    Args:
        export: From `load_export`.
        robot_files: Robot name as in the schedule ("Cindy") -> (URDF, SRDF) the cells were built from.
        serials: Robot name -> hardware serial ("0806").
        tool_kind: Rhino tool name -> end effector kind.
        report: Told about everything filled in or dropped, one line each.

    Raises:
        ValueError: If a robot's URDF does not match the model in its cell (joint names or origins), or the
            export holds data a design cannot carry (`_refuse_unsupported`, a tool with moving joints).
    """
    schedule, cells, actions = export.schedule, export.cells, export.actions
    names = {entry["robot_id"]: name for name, entry in schedule["robots"].items()}   # robot_id -> "Cindy"
    robot_ids = {name: export.robot_id(name) for name in schedule["robots"]}         # "Cindy" -> robots/cindy

    unscheduled = sorted({path.name for path in (export.folder / "BarActions").glob("*.json")}
                         - {Path(entry["file"]).name for entry in schedule["schedule"]})
    if unscheduled:
        report(f"{len(unscheduled)} BarActions file(s) are not in the schedule and are left out: "
               f"{', '.join(unscheduled)}")
    converter = _Converter(cells, names, robot_ids, robot_files, serials or {}, tool_kind, report)
    converter.collect(actions)
    ground = _walkable_ground(export.folder / "WalkableGround.json", report)
    converter.bodies.update(ground)
    # * Pass 1: every movement as read; pass 2: schema 2, following the schedule.
    raw = [converter.raw_action(entry, action, tuple(ground)) for entry, action in actions]
    connections = converter.connections(raw)
    design_actions = converter.actions(raw, connections)
    if converter.filled:
        report(f"{converter.filled} configuration(s) listed only some joints; the others were taken from the "
               f"robot's previous state, or zero")
    if converter.mate_bar_contacts:
        report(f"{converter.mate_bar_contacts} contact(s) of a joint half with its mate's bar are not connections "
               f"(mates connect parts, never merge them); the mate contact rule still allows them")
    if converter.no_line:
        report(f"{converter.no_line} linear movement(s) without a line: neither notes nor start and target give one")
    report(f"{len(converter.robots)} robots, {len(converter.tools)} tools, {len(converter.bodies)} bodies, "
           f"{len(connections)} connections, {len(design_actions)} actions, "
           f"{sum(len(a.movements) for a in design_actions.values())} movements")
    writer = writer_info()
    return Design(folder=None, writer=writer, robots=converter.robots, tools=converter.tools,
                  bodies=converter.bodies, schedule=tuple(entry["action_id"] for entry, _ in actions),
                  actions=design_actions, connections=connections,
                  producer=Producer("husky-assembly-teleop", writer.commit, writer.dirty, "legacy.export.from_export"))


# --- --- --- --- --- CONVERSION --- --- --- --- ---

@dataclass
class _RawMovement:
    """One export movement as read, before schema 2: its start state, target and schema 1 fields."""

    id: str
    kind: str
    coupled: bool
    controller: str
    arms: Tuple[str, ...]
    tools: Tuple[str, ...]
    tool_action: Optional[str]
    overlaps_next: bool
    robots: Dict[str, Optional[RobotState]]
    present: set
    poses: Dict[str, Pose]
    carried: Dict[str, Carried]
    touches: set
    joints: Dict[str, Dict[str, float]]
    links: Dict[str, Pose]
    label: str
    notes: dict


@dataclass
class _RawAction:
    """One export action as read, its movements before schema 2."""

    id: str
    type: str
    robot: str
    bar: str
    label: str
    ground: Tuple[str, ...]
    supports_until: Tuple[str, ...]
    movements: Tuple[_RawMovement, ...]


def body_id(name: str) -> str:
    """Our id of an export's rigid body, e.g. "bar_B1" and "env_bar_B1" -> "bars/B1"; unknown prefixes -> obstacles."""
    name = name[len("env_"):] if name.startswith("env_") else name
    for prefix, group in (("bar_", "bars"), ("joint_", "joints"), ("obstacle_", "obstacles")):
        if name.startswith(prefix):
            return f"{group}/{name[len(prefix):]}"
    return f"obstacles/{name}"


class _Converter:
    """The state of one conversion: ids and specs found so far, and each robot's last joints."""

    def __init__(self, cells: Dict[str, RobotCell], names: Dict[str, str], robot_ids: Dict[str, str],
                 robot_files: Mapping[str, Tuple[Path, Path]], serials: Mapping[str, str],
                 tool_kind: Callable[[str], str], report: Callable[[str], None]):
        self.cells, self.names, self.robot_ids, self.report = cells, names, robot_ids, report
        self.robots: Dict[str, RobotSpec] = {}
        self.tools: Dict[str, Tool] = {}
        self.bodies: Dict[str, BodySpec] = {}
        self._tool_kind = tool_kind
        # Robot id -> (URDF, SRDF, movable joints in URDF order).
        self._files: Dict[str, Tuple[Path, Path, Tuple[str, ...]]] = {}
        # (robot id, cell tool name) -> tool id.
        self._tool_ids: Dict[Tuple[str, str], str] = {}
        # Robot id -> joints of its last known state, for filling partial configurations.
        self._last: Dict[str, Dict[str, float]] = {}
        #: How many configurations listed only some joints.
        self.filled = 0
        #: How many linear movements got no line.
        self.no_line = 0
        #: How many export contacts of a joint half with its mate's bar were not made connections.
        self.mate_bar_contacts = 0
        self._serials = {self.robot_ids[name]: serial for name, serial in serials.items()}
        for robot_id, name in names.items():
            urdf, srdf = (Path(path) for path in robot_files[name])
            self._files[self.robot_ids[name]] = (urdf, srdf, tuple(movable_joints(urdf, srdf)))
            if robot_id in cells:
                _check_model(cells[robot_id], urdf)

    # --- --- robots, tools and bodies, from every movement --- ---

    def collect(self, actions) -> None:
        """Find robots, tools, bodies and each body's design pose, from (schedule entry, action) pairs."""
        tool_names: Dict[str, set] = {}    # cell tool name -> robots that carry it
        mounts: Dict[str, Dict[str, str]] = {}
        tool_touches: Dict[Tuple[str, str], set] = {}
        for entry, action in actions:
            robot = self.robot_ids[entry["robot"]]
            cell = self.cells[action.robot_id]
            for movement in action.movements:
                _refuse_unsupported(movement)
                for name, tool_state in movement.start_state.tool_states.items():
                    if name.startswith("ObstacleRobot") or not tool_state.attached_to_group:
                        continue
                    tool_names.setdefault(name, set()).add(robot)
                    flange = cell.get_end_effector_link_name(tool_state.attached_to_group)
                    mounts.setdefault(robot, {})[flange] = name
                    tool_touches.setdefault((robot, name), set()).update(
                        link_id(robot, link) for link in tool_state.touch_links)

        for (robot, name), touches in tool_touches.items():
            # ? One Rhino tool name may be on several robots (SupportGripper): the robot goes in the id.
            tool = f"tools/{name}" if len(tool_names[name]) == 1 else f"tools/{robot.split('/')[1]}/{name}"
            self._tool_ids[(robot, name)] = tool
            cell = next(cell for cell_id, cell in self.cells.items() if self.robot_ids[self.names[cell_id]] == robot)
            model = cell.tool_models[name]
            self.tools[tool] = Tool(id=tool, geometry=_model_geometry(model), tcp=pose_from_frame(model.frame),
                                    kind=self._tool_kind(name), mount_contacts=tuple(sorted(touches)))

        for robot, (urdf, srdf, _) in self._files.items():
            flanges = mounts.get(robot, {})
            # ? The export let every robot's wheel links touch the ground: those are the links it stands on.
            self.robots[robot] = RobotSpec(id=robot, urdf=urdf, srdf=srdf, serial=self._serials.get(robot),
                                           tools={flange: self._tool_ids[(robot, name)]
                                                  for flange, name in sorted(flanges.items())},
                                           ground_links=tuple(sorted(link for link in urdf_links(urdf)
                                                                     if "wheel" in link)))

        # * Bodies: geometry from the first cell that has them, pose from the last movement where present and unheld.
        poses: Dict[str, Pose] = {}
        for cell in self.cells.values():
            for name, rigid_body in cell.rigid_body_models.items():
                key = body_id(name)
                if key not in self.bodies:
                    self.bodies[key] = BodySpec(id=key, pose=Pose(), geometry=_body_geometry(rigid_body))
        for _, action in actions:
            for movement in action.movements:
                for name, body_state in movement.start_state.rigid_body_states.items():
                    if not body_state.is_hidden and not body_state.attached_to_link and body_state.frame is not None:
                        poses[body_id(name)] = pose_from_frame(body_state.frame)
        for key, body in list(self.bodies.items()):
            if key in poses:
                self.bodies[key] = BodySpec(id=key, pose=poses[key], geometry=body.geometry)
            else:
                self.report(f"{key}: never present unheld; design pose left at the origin")

    # --- --- pass 1: every movement as read --- ---

    def raw_action(self, entry: dict, action, ground: Tuple[str, ...]) -> "_RawAction":
        """One schedule entry, its movements read but not yet in schema 2.

        Args:
            entry: Its ActionSchedule.json entry.
            action: The loaded rs_data_structure action.
            ground: Ids of the ground bodies, present in every state.
        """
        robot = self.robot_ids[entry["robot"]]
        cell = self.cells[action.robot_id]
        return _RawAction(
            id=entry["action_id"], type=ACTION_KINDS[type(action).__name__], robot=robot,
            bar=f"bars/{action.active_bar_id}", label=action.tag or "",
            ground=tuple(f"ground/{name}" for name in getattr(action, "walkable_ground_ids", []) or []),
            supports_until=tuple(f"bars/{bar}" for bar in getattr(action, "supported_until", []) or []),
            movements=tuple(self.raw_movement(robot, cell, movement, action.active_bar_id, ground)
                            for movement in action.movements))

    def raw_movement(self, robot: str, cell: RobotCell, movement, bar: str, ground: Tuple[str, ...]) -> "_RawMovement":
        """One rs_data_structure movement as read: its start state, target and schema 1 fields."""
        kind, coupled = MOVEMENT_KINDS[type(movement).__name__]
        flanges = tuple(link_id(robot, flange) for flange in self.robots[robot].tools)
        placeholder = {f"bars/{bar}"} if (movement.notes or {}).get("bar_pose_is_placeholder") else set()
        robots, present, poses, carried, touches = self.raw_state(robot, movement.start_state, ground)
        # ? Schema 2: a body at a placeholder pose is not present.
        present -= placeholder
        for body in placeholder:
            poses.pop(body, None)
        joints, links = {}, {}
        if movement.target_configuration is not None:
            joints[robot] = dict(zip(movement.target_configuration.joint_names,
                                     map(float, movement.target_configuration.joint_values)))
        for key, frame in (movement.target_ee_frames or {}).items():
            links[self._flange(flanges, key, movement.movement_id)] = pose_from_frame(frame)
        return _RawMovement(
            id=movement.movement_id, kind=kind, coupled=coupled, controller=movement.controller,
            arms=flanges if kind in ("free", "linear") else (),
            tools=tuple(self._tool_ids[(robot, name)] for name in getattr(movement, "tool_names", []) or []),
            tool_action=getattr(movement, "tool_action", None) if kind == "tool" else None,
            overlaps_next=bool(getattr(movement, "overlaps_next", False)), robots=robots, present=present,
            poses=poses, carried=carried, touches=touches, joints=joints, links=links, label=movement.tag or "",
            notes=dict(movement.notes or {}))

    @staticmethod
    def _flange(flanges: Tuple[str, ...], key: str, where: str) -> str:
        """The flange a schema 1 side key names: "arm" on a single-arm robot, "left"/"right" on the dual-arm one."""
        matches = [flange for flange in flanges if key == "arm" or f"/{key}_" in flange]
        if len(matches) != 1:
            raise ValueError(f"{where}: cannot tell which flange {key!r} is")
        return matches[0]

    def raw_state(self, robot: str, legacy: RobotCellState, ground: Tuple[str, ...]):
        """One compas_fab start state: robots, present bodies, poses of every free one, carried, contacts."""
        robots: Dict[str, Optional[RobotState]] = {other: None for other in self.robots}
        configuration = legacy.robot_configuration
        robots[robot] = RobotState(base=pose_from_frame(legacy.robot_base_frame),
                                   joints=None if configuration is None else self._joints(robot, configuration))
        for name, tool_state in legacy.tool_states.items():
            if not name.startswith("ObstacleRobot"):
                continue
            other = self.robot_ids[name[len("ObstacleRobot"):]]
            if tool_state.frame is None or np.allclose(list(tool_state.frame.point), PARKED_POSITION, atol=1e-3):
                continue
            robots[other] = RobotState(base=pose_from_frame(tool_state.frame),
                                       joints=self._joints(other, tool_state.configuration))
        present = set(ground)
        poses = {key: self.bodies[key].pose for key in ground}
        carried, touches = {}, set()
        for name, body_state in legacy.rigid_body_states.items():
            key = body_id(name)
            if body_state.is_hidden:
                continue
            present.add(key)
            if body_state.attached_to_link:
                carried[key] = Carried(to=link_id(robot, body_state.attached_to_link),
                                       offset=pose_from_frame(body_state.attachment_frame))
            elif body_state.attached_to_tool:
                raise ValueError(f"{key}: attached to a tool; the converter does not handle that yet")
            else:
                poses[key] = (pose_from_frame(body_state.frame) if body_state.frame is not None
                              else self.bodies[key].pose)
            for link in body_state.touch_links:
                touches.add(tuple(sorted((key, link_id(robot, link)))))
            for other in body_state.touch_bodies:
                touches.add(tuple(sorted((key, self._touch_id(robot, other)))))
        return robots, present, poses, carried, touches

    def _touch_id(self, robot: str, name: str) -> str:
        """Our id of a compas_fab touch_bodies entry: a body, a tool of the acting robot, or another robot."""
        if name.startswith("ObstacleRobot"):
            return self.robot_ids[name[len("ObstacleRobot"):]]
        if (robot, name) in self._tool_ids:
            return self._tool_ids[(robot, name)]
        return body_id(name)

    # --- --- pass 2: schema 2 --- ---

    def connections(self, raw: List["_RawAction"]) -> frozenset:
        """Bodies joined in the design: each joint half and its bar, mated halves, ground connectors and their ground.

        - A half's bar is the one it is carried with ("held by the same link, its joint halves included"), else the
          one bar it touches in the export.
        - A half also touching its mate's bar is not connected to it: mates connect parts, never merge them
          (counted in `mate_bar_contacts`).
        - A ground connector is connected to the ground bodies under its design position (by their floor area),
          else to the ground of the action that places its bar.
        """
        parent: Dict[str, str] = {}
        for action in raw:
            for movement in action.movements:
                if action.bar in movement.carried:
                    for body in movement.carried:
                        if body.startswith("joints/"):
                            parent.setdefault(body, action.bar)
        contacts = {pair for action in raw for movement in action.movements for pair in movement.touches
                    if all(side in self.bodies for side in pair)}
        for a, b in sorted(contacts):
            for half, bar in ((a, b), (b, a)):
                if half.startswith("joints/") and bar.startswith("bars/"):
                    parent.setdefault(half, bar)
        pairs = {tuple(sorted(pair)) for pair in parent.items()}
        for a, b in contacts:
            if a.startswith("joints/") and b.startswith("joints/"):
                pairs.add((a, b))
            elif {a.split("/")[0], b.split("/")[0]} == {"joints", "bars"} and (a, b) not in pairs:
                self.mate_bar_contacts += 1
        grounds = {key: self._floor_area(body) for key, body in self.bodies.items() if key.startswith("ground/")}
        for action in raw:
            for half, bar in parent.items():
                if bar == action.bar and half.endswith("_ground"):
                    x, y = self.bodies[half].pose.position[:2]
                    under = [key for key, (low, high) in grounds.items()
                             if low[0] - _GROUND_MARGIN <= x <= high[0] + _GROUND_MARGIN
                             and low[1] - _GROUND_MARGIN <= y <= high[1] + _GROUND_MARGIN]
                    pairs.update(tuple(sorted((half, ground))) for ground in (under or action.ground))
        return frozenset(pairs)

    @staticmethod
    def _floor_area(body: BodySpec) -> Tuple[np.ndarray, np.ndarray]:
        """The (x, y) bounding box of a ground body's collision shapes in world, as (low, high)."""
        points = np.vstack([shape.vertices for shape in body.geometry.collision])
        points = points @ body.pose.matrix()[:3, :3].T + body.pose.position
        return points[:, :2].min(axis=0), points[:, :2].max(axis=0)

    def actions(self, raw: List["_RawAction"], connections: frozenset) -> Dict[str, Action]:
        """Every action in schema 2, following the schedule: tool states replayed, tighten and insert merged."""
        tool_kinds = {tool_id: tool.kind for tool_id, tool in self.tools.items()}
        # Tool id -> its current state (channels and "on"), from the start.
        current = {tool_id: {**released(kind), ON: None} for tool_id, kind in tool_kinds.items()}
        mounted = {tool: robot_id for robot_id, robot in self.robots.items() for tool in robot.tools.values()}
        fk = ForwardKinematics()
        # Robot id -> what it carried at the start of its last own movement.
        last_carried: Dict[str, Dict[str, Carried]] = {}
        result = {}
        for action in raw:
            movements, index = [], 0
            last_links: Dict[str, Pose] = {}
            while index < len(action.movements):
                move = action.movements[index]
                merged = move.overlaps_next and index + 1 < len(action.movements)
                arm_move = action.movements[index + 1] if merged else move
                self._on_from_contacts(action, move, current, tool_kinds)
                # * A support gripper sits on the bar once its approach ends, before it closes.
                if move.tool_action == "close":
                    for tool in move.tools:
                        if tool_kinds[tool] == "robotiq":
                            current[tool][ON] = action.bar
                # ? The export leaves out what another robot holds (Alice's states lack the bar Cindy holds in place):
                #   a schema 2 state is complete, so another present robot keeps carrying what it last carried.
                carried, present, poses = dict(move.carried), set(move.present), dict(move.poses)
                for other, bodies in last_carried.items():
                    if other != action.robot and move.robots.get(other) is not None:
                        for body, value in bodies.items():
                            if body not in present:
                                carried[body] = value
                                present.add(body)
                                poses.pop(body, None)
                if action.type == "bar_holding_release":
                    # ? The export hides the supported bar from its own support robot; it is built, so present.
                    for body in (action.bar, *(half for pair in connections for half in pair
                                               if action.bar in pair and half.startswith("joints/"))):
                        if body not in present and body not in carried:
                            present.add(body)
                            poses[body] = self.bodies[body].pose
                last_carried[action.robot] = dict(move.carried)
                start = State(robots=dict(move.robots), present=frozenset(present), poses=poses, carried=carried,
                              tools={tool: (dict(current[tool]) if move.robots.get(mounted[tool]) is not None
                                            else None) for tool in sorted(current)})
                changes = {tool: dict([_TOOL_ACTIONS[move.tool_action]]) for tool in move.tools} \
                    if move.tool_action else {}
                path = arm_move.kind if arm_move.kind in ("free", "linear") else None
                notes = {**move.notes, **arm_move.notes} if merged else move.notes
                movement = Movement(
                    id=move.id, start=start, arms=arm_move.arms, path=path,
                    coupled=arm_move.coupled if path else False,
                    controller=_CONTROLLERS[arm_move.controller] if path else None,
                    line=self._line(action.robot, arm_move, start, last_links, fk) if path == "linear" else {},
                    ends_on="tools" if merged else "operator" if move.kind == "manual" else "target",
                    target=(Target(joints=dict(arm_move.joints), links=dict(arm_move.links), tools=changes)
                            if arm_move.joints or arm_move.links or changes else None),
                    label=arm_move.label or move.label,
                    notes={key: value for key, value in notes.items()
                           if key not in _CONSUMED_NOTES and isinstance(value, (str, int, float, bool))})
                movements.append(movement)
                # * After the movement: tool channels change, and a tool leaves its body once an arm motion that
                #   started open ends (the retreat).
                for tool, channels in changes.items():
                    current[tool].update(channels)
                for flange in movement.arms:
                    tool = self.robots[action.robot].tools.get(flange.rsplit("/", 1)[1])
                    if tool and start.tools[tool] and start.tools[tool].get("grip") == "open":
                        current[tool][ON] = None
                last_links.update(arm_move.links)
                index += 2 if merged else 1
            result[action.id] = Action(id=action.id, type=action.type, robot=action.robot, bar=action.bar,
                                       movements=tuple(movements), ground=action.ground,
                                       supports_until=action.supports_until, label=action.label)
        return result

    def _on_from_contacts(self, action: "_RawAction", move: "_RawMovement", current: Dict[str, dict],
                          kinds: Dict[str, str]) -> None:
        """Set where the acting robot's scaffolding tools sit from the export's contacts: the male or ground half."""
        for tool in self.robots[action.robot].tools.values():
            if kinds[tool] != "scaffolding_v3":
                continue
            touched = {side for pair in move.touches if tool in pair for side in pair if side != tool}
            halves = sorted(body for body in touched if body.startswith("joints/")
                            and body.endswith(("_male", "_ground")))
            if len(halves) > 1:
                raise ValueError(f"{move.id}: {tool} touches {halves}; cannot tell which it sits on")
            current[tool][ON] = halves[0] if halves else None

    def _line(self, robot: str, move: "_RawMovement", start: State, last_links: Dict[str, Pose],
              fk: ForwardKinematics) -> Dict[str, LineSpec]:
        """Each moving flange's line: the notes' world axes and distance, or start to target; empty if neither."""
        axes = move.notes.get("retreat_axes_world")
        distance = move.notes.get("lm_distance_mm")
        lines = {}
        for flange in move.arms:
            direction, length = None, None
            if isinstance(axes, dict):
                side = next((key for key in axes if f"/{key}_" in flange or key == "arm"), None)
                if side is not None:
                    direction = np.asarray(axes[side], dtype=float)
                    length = float(distance) / 1000.0 if distance else None
            if (direction is None or length is None) and flange in move.links:
                begin = self._flange_pose(robot, flange, start, last_links, fk)
                if begin is not None:
                    step = np.subtract(move.links[flange].position, begin.position)
                    direction = step if direction is None else direction
                    length = float(np.linalg.norm(step)) if length is None else length
            if direction is None or not length or np.linalg.norm(direction) == 0.0:
                continue
            unit = direction / np.linalg.norm(direction)
            lines[flange] = LineSpec(tuple(float(v) for v in unit), length)
        if len(lines) != len(move.arms):
            self.no_line += 1
            return {}
        return lines

    def _flange_pose(self, robot: str, flange: str, start: State, last_links: Dict[str, Pose],
                     fk: ForwardKinematics) -> Optional[Pose]:
        """Where a flange is at a movement's start: forward kinematics of the start joints, else the previous target."""
        robot_state = start.robots.get(robot)
        if robot_state is not None and robot_state.joints is not None and robot_state.base is not None:
            return fk.link_pose(self._files[robot][0], robot_state.base, robot_state.joints, flange.rsplit("/", 1)[1])
        return last_links.get(flange)

    def _joints(self, robot: str, configuration) -> Dict[str, float]:
        """Every movable joint from a possibly partial configuration; the rest from the robot's last state or zero."""
        given = dict(zip(configuration.joint_names, map(float, configuration.joint_values))) if configuration else {}
        last = self._last.get(robot, {})
        movable = self._files[robot][2]
        missing = [name for name in movable if name not in given]
        if missing:
            self.filled += 1
        joints = {name: given.get(name, last.get(name, 0.0)) for name in movable}
        self._last[robot] = joints
        return joints


# --- --- --- --- --- GEOMETRY --- --- --- --- ---

def _same(a: Pose, b: Pose) -> bool:
    """Whether two poses are equal within _SAME (quaternion sign ignored)."""
    q_a, q_b = np.asarray(a.orientation), np.asarray(b.orientation)
    return (np.allclose(a.position, b.position, atol=_SAME)
            and min(np.abs(q_a - q_b).max(), np.abs(q_a + q_b).max()) < _SAME)


def _tri(mesh, matrix: Optional[np.ndarray] = None, scale: float = 1.0) -> TriMesh:
    """A compas mesh as one of ours, scaled, then transformed by a 4x4 `matrix` if given."""
    vertices, faces = mesh.to_vertices_and_faces(triangulated=True)
    points = np.asarray(vertices, dtype=float) * scale
    if matrix is not None:
        points = points @ matrix[:3, :3].T + matrix[:3, 3]
    return TriMesh.from_arrays(points, faces)


def _shared(geometry: Geometry) -> Geometry:
    """The same geometry, its visual shapes replaced by the collision ones when equal, so the writer omits `visual`."""
    same = len(geometry.visual) == len(geometry.collision) and all(
        np.array_equal(v.vertices, c.vertices) and np.array_equal(v.faces, c.faces)
        for v, c in zip(geometry.visual, geometry.collision))
    return Geometry(geometry.collision, geometry.collision) if same else geometry


def _body_geometry(rigid_body) -> Geometry:
    """A rigid body's geometry, visual shared with collision when equal."""
    return _shared(Geometry.from_rigid_body(rigid_body))


def _refuse_unsupported(movement) -> None:
    """Raise if a movement holds data a design cannot carry, instead of dropping it.

    Raises:
        ValueError: For a trajectory, or a tool state that is hidden, configured or attached off its flange.
    """
    where = movement.movement_id
    if getattr(movement, "trajectory", None) is not None:
        raise ValueError(f"{where}: has a trajectory; planned results are not part of a design (format §8)")
    for name, tool_state in movement.start_state.tool_states.items():
        if name.startswith("ObstacleRobot"):
            continue
        if tool_state.is_hidden:
            raise ValueError(f"{where}: tool {name} is hidden; a design has no hidden tools")
        if tool_state.configuration is not None:
            raise ValueError(f"{where}: tool {name} has a configuration; design tools have no joints")
        frame = tool_state.attachment_frame
        if frame is not None and not np.allclose(np.asarray(frame.to_transformation().matrix), np.eye(4), atol=_SAME):
            raise ValueError(f"{where}: tool {name} is attached off its flange; a design mounts tools on the flange")


def _model_geometry(model) -> Geometry:
    """A compas ToolModel's meshes in its base frame.

    Raises:
        ValueError: If the tool has a joint that moves: a design tool is one rigid shape.
    """
    moving = [joint.name for joint in model.joints if joint.type != joint.FIXED]
    if moving:
        raise ValueError(f"tool {model.name} has moving joints {moving}; a design tool is one rigid shape")
    transformations = model.compute_transformations(model.zero_configuration())

    def shapes(which: str):
        result = []
        for link in model.links:
            joint = link.parent_joint
            link_frame = joint.current_origin.transformed(transformations[joint.name]) if joint else None
            link_matrix = np.asarray(link_frame.to_transformation().matrix) if joint else np.eye(4)
            for item in getattr(link, which):
                shape = item.geometry.shape
                origin = np.asarray(item.origin.to_transformation().matrix) if item.origin else np.eye(4)
                scale = np.diag([*getattr(shape, "scale", (1.0, 1.0, 1.0)), 1.0])
                for mesh in getattr(shape, "meshes", None) or []:
                    result.append(_tri(mesh, link_matrix @ origin @ scale))
        return tuple(result)

    visual, collision = shapes("visual"), shapes("collision")
    return _shared(Geometry(visual or collision, collision))


def _walkable_ground(path: Path, report: Callable[[str], None]) -> Dict[str, BodySpec]:
    """The walkable ground patches as `ground/<id>` slab bodies in metres; empty if the file is missing.

    Args:
        path: WalkableGround.json.
        report: Told about the conversion.
    """
    if not path.is_file():
        report("no WalkableGround.json: no ground bodies")
        return {}
    bodies = {}
    for name, mesh in json_load(str(path))["grounds"].items():
        vertices, faces = mesh.to_vertices_and_faces()
        points = np.asarray(vertices, dtype=float)
        if np.abs(points).max() > _MILLIMETRE_THRESHOLD:
            points = points * 0.001
            report(f"ground/{name}: millimetres converted to metres")
        slabs = tuple(_slab(points[face]) for face in faces)
        key = f"ground/{name}"
        bodies[key] = BodySpec(id=key, pose=Pose(), geometry=Geometry(slabs, slabs))
    return bodies


def _slab(polygon: np.ndarray) -> TriMesh:
    """Extrude an (n, 3) polygon downward by GROUND_THICKNESS into a closed, triangulated slab."""
    n = len(polygon)
    top = polygon
    bottom = polygon - np.array([0.0, 0.0, GROUND_THICKNESS])
    vertices = np.vstack([top, bottom])
    faces: List[List[int]] = []
    faces += [[0, i, i + 1] for i in range(1, n - 1)]                      # top, fan
    faces += [[n, n + i + 1, n + i] for i in range(1, n - 1)]              # bottom, reversed
    for i in range(n):
        j = (i + 1) % n
        faces += [[i, n + i, n + j], [i, n + j, j]]                        # sides
    return TriMesh.from_arrays(vertices, faces)


def _check_model(cell: RobotCell, urdf: Path) -> None:
    """Check that a URDF has the same joints and joint origins (the calibration) as the robot in a cell.

    Raises:
        ValueError: If joint names differ, or an origin differs by more than _SAME.
    """
    embedded = {joint.name: joint for joint in cell.robot_model.joints}
    in_file = set(urdf_joints(urdf))
    if set(embedded) != in_file:
        raise ValueError(f"{urdf.name} does not match the model in the cell of {cell.robot_model.name}: "
                         f"only in the cell {sorted(set(embedded) - in_file)}, "
                         f"only in the file {sorted(in_file - set(embedded))}")
    # Joints only: parsing without meshes is fast and needs no package paths.
    for joint in RobotModel.from_urdf_file(str(urdf)).joints:
        a = np.asarray(joint.origin.to_transformation().matrix)
        b = np.asarray(embedded[joint.name].origin.to_transformation().matrix)
        if not np.allclose(a, b, atol=_SAME):
            raise ValueError(f"{urdf.name}: joint {joint.name} has another origin than in the cell of "
                             f"{cell.robot_model.name}; is it the same calibration?")
