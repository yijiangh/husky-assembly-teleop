"""
Read an old compas_fab export into a schema 2 `Design`.

The export is RobotCell*.json, BarActions/, ActionSchedule.json and WalkableGround.json. Robots come from the given
URDF and SRDF files, checked joint by joint against the models in the cells. The bars' `attached` and `built` flags
and the tool states are replayed along the schedule (format Appendix B), not copied from the export's states.
! Slow (~15 s for three ~350 MB cells). Imports compas and rs_data_structure.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Callable, Dict, List, Mapping, Optional, Set, Tuple

import numpy as np
from compas.data import json_load
from compas_fab.robots import RobotCell, RobotCellState
from compas_robots import RobotModel

# ! Registers the dtypes of the action and movement classes, so json_load can rebuild them.
import rs_data_structure  # noqa: F401

from ..design.plan_check import PART_SEATS
from ..design.types import (Action, BodySpec, Design, Holder, LineSpec, Movement, PartSpec, Producer, RobotSpec,
                            RobotState, State, Target, ToolState)
from ..design.version import writer_info
from ..geometry import Geometry, Pose, TriMesh, compose, invert
from ..ids import link_id, split_link_id
from ..kinematics import link_pose
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

#: Notes that became typed fields (`line`, `ends_on`), say what other fields already say, or are planner status that
#: belongs in `solutions/` (proposal, Notes). The rest stay notes, for people.
_CONSUMED_NOTES = ("lm_axis", "lm_distance_mm", "retreat_axes_world", "ends_on", "constraint", "bar_arm_side",
                   "planner_fills", "start_config_is_none", "unplanned_offline", "goal_backfilled_from",
                   "bar_pose_is_placeholder")

#: Schema 1 controller -> schema 2 controller (None: no arm moves).
_CONTROLLERS = {"joint_tracking": "position", "cartesian_compliant": "compliant", "none": None}

#: Schema 1 tool action -> the grip it ends with; "tighten" becomes a drive, "untighten" a drive of the next ungrasp.
_GRIPS = {"grasp": "closed", "close": "closed", "ungrasp": "open", "open": "open"}

#: The converter's name and version, written as the design's `producer.command`. ! Bump the number whenever the
#: output changes: converted copies made by another version are converted again (`conversion.is_up_to_date`).
CONVERTER = "legacy.export.from_export/2"

#: Action type -> the code in its id, e.g. "B10_J" (format §3.1).
ACTION_CODES = {"bar_jointing": "J", "bar_release": "R", "bar_holding": "H", "bar_holding_release": "HR"}

#: Joint half id -> (bar pair or ground name, subtype), e.g. "joints/J3-10_male", "joints/G1-T20Ground-0_ground".
_HALF_ID = re.compile(r"joints/(?P<pair>J\d+-\d+|G\d+-.*)_(?P<subtype>male|female|ground)")
#: The catalogue type of every half in the exports (the ids name only the bar pair).
_JOINT_TYPE = "T20"

#: Depth of the slab the walkable ground polygons are extruded into, downward, metres.
GROUND_THICKNESS = 0.05

#: Coordinates beyond this are millimetres (WalkableGround.json is written in Rhino's units).
_MILLIMETRE_THRESHOLD = 50.0

#: How far a tool's TCP may be from the half it is on, metres.
_TCP_REACH = 0.01

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
    # * Pass 1: every movement as read; then mounts and mates; pass 2: schema 2, replayed along the schedule.
    raw = [converter.raw_action(entry, action) for entry, action in actions]
    converter.mount_halves(raw)
    mates = converter.mates()
    design_actions = converter.actions(raw)
    for line in converter.notes:
        report(line)
    report(f"{len(converter.robots)} robots, {len(converter.tools)} tools, {len(converter.bodies)} bodies, "
           f"{len(mates)} mates, {len(design_actions)} actions, "
           f"{sum(len(a.movements) for a in design_actions.values())} movements")
    writer = writer_info()
    parts = {body.part for body in converter.bodies.values()}
    return Design(folder=None, writer=writer, robots=converter.robots, tools=converter.tools,
                  bodies=converter.bodies, schedule=tuple(design_actions), actions=design_actions, mates=mates,
                  producer=Producer("bar_assembly_core", writer.commit, writer.dirty, CONVERTER),
                  parts={name: PartSpec(seat) for name, seat in PART_SEATS.items() if name in parts})


# --- --- --- --- --- CONVERSION --- --- --- --- ---

@dataclass
class _RawMovement:
    """One export movement as read, before schema 2: its start state, target and schema 1 fields.

    Attributes:
        carried: Body -> (link id, the body's pose in that link's frame), as the export attaches it.
        poses: World pose of every present body the export does not attach.
    """

    id: str
    kind: str
    coupled: bool
    controller: str
    arms: Tuple[str, ...]
    tools: Tuple[str, ...]
    tool_action: Optional[str]
    overlaps_next: bool
    robots: Dict[str, Optional[RobotState]]
    carried: Dict[str, Tuple[str, Pose]]
    poses: Dict[str, Pose]
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
        #: Lines for the report: what was derived, dropped or found inconsistent.
        self.notes: List[str] = []
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

    def raw_action(self, entry: dict, action) -> "_RawAction":
        """One schedule entry, its movements read but not yet in schema 2.

        Args:
            entry: Its ActionSchedule.json entry.
            action: The loaded rs_data_structure action.
        """
        robot = self.robot_ids[entry["robot"]]
        cell = self.cells[action.robot_id]
        return _RawAction(
            id=entry["action_id"], type=ACTION_KINDS[type(action).__name__], robot=robot,
            bar=f"bars/{action.active_bar_id}", label=action.tag or "",
            ground=tuple(f"ground/{name}" for name in getattr(action, "walkable_ground_ids", []) or []),
            supports_until=tuple(f"bars/{bar}" for bar in getattr(action, "supported_until", []) or []),
            movements=tuple(self.raw_movement(robot, cell, movement) for movement in action.movements))

    def raw_movement(self, robot: str, cell: RobotCell, movement) -> "_RawMovement":
        """One rs_data_structure movement as read: its start state, target and schema 1 fields."""
        kind, coupled = MOVEMENT_KINDS[type(movement).__name__]
        flanges = tuple(link_id(robot, flange) for flange in self.robots[robot].tools)
        robots, carried, poses = self.raw_state(robot, movement.start_state)
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
            overlaps_next=bool(getattr(movement, "overlaps_next", False)), robots=robots, carried=carried,
            poses=poses, joints=joints, links=links, label=movement.tag or "", notes=dict(movement.notes or {}))

    @staticmethod
    def _flange(flanges: Tuple[str, ...], key: str, where: str) -> str:
        """The flange a schema 1 side key names: "arm" on a single-arm robot, "left"/"right" on the dual-arm one."""
        matches = [flange for flange in flanges if key == "arm" or f"/{key}_" in flange]
        if len(matches) != 1:
            raise ValueError(f"{where}: cannot tell which flange {key!r} is")
        return matches[0]

    def raw_state(self, robot: str, legacy: RobotCellState):
        """One compas_fab start state: every robot, the bodies the acting robot attaches, the others' poses."""
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
        carried, poses = {}, {}
        for name, body_state in legacy.rigid_body_states.items():
            key = body_id(name)
            if body_state.is_hidden:
                continue
            if body_state.attached_to_link:
                carried[key] = (link_id(robot, body_state.attached_to_link),
                                pose_from_frame(body_state.attachment_frame))
            elif body_state.attached_to_tool:
                raise ValueError(f"{key}: attached to a tool; the converter does not handle that yet")
            elif body_state.frame is not None:
                poses[key] = pose_from_frame(body_state.frame)
        return robots, carried, poses

    # --- --- mounts and mates --- ---

    def mount_halves(self, raw: List["_RawAction"]) -> None:
        """Give each half its `mount`, the bar it is carried with in the jointing actions, and its `part` (from its id).

        Raises:
            ValueError: If a half is carried with two bars, or with none.
        """
        mounts: Dict[str, str] = {}
        for action in raw:
            if action.type != "bar_jointing":
                continue
            for movement in action.movements:
                if action.bar not in movement.carried:
                    continue
                for body in movement.carried:
                    if body.startswith("joints/") and mounts.setdefault(body, action.bar) != action.bar:
                        raise ValueError(f"{body} is carried with {mounts[body]} and with {action.bar}")
        for key, body in list(self.bodies.items()):
            if not key.startswith("joints/"):
                continue
            if key not in mounts:
                raise ValueError(f"{key} is never carried with a bar in a jointing action: no mount")
            match = _HALF_ID.fullmatch(key)
            part = f"{_JOINT_TYPE}/{match['subtype'].capitalize()}" if match else ""
            self.bodies[key] = replace(body, mount=mounts[key], part=part)

    def mates(self) -> frozenset:
        """The joints of the finished structure, from the half ids.

        - `J<a>-<b>_male` mates `J<a>-<b>_female` when both are in the design.
        - A ground half (`G<n>-…_ground`) mates the ground body it stands on: the one whose walkable polygons are
          nearest its design position seen from above (0 when it is above one; the nearest in height wins a tie).
        """
        pairs: Set[Tuple[str, str]] = set()
        footprints = {key: _footprints(body) for key, body in self.bodies.items() if key.startswith("ground/")}
        unmated = []
        for key in sorted(self.bodies):
            match = _HALF_ID.fullmatch(key)
            if match is None:
                continue
            other = f"joints/{match['pair']}_{'female' if match['subtype'] == 'male' else 'male'}"
            if match["subtype"] in ("male", "female"):
                if other in self.bodies:
                    pairs.add(tuple(sorted((key, other))))
                else:
                    unmated.append(key)
            elif footprints:
                position = np.asarray(self.bodies[key].pose.position)
                ground = min(footprints, key=lambda name: _footprint_distance(position, footprints[name]))
                pairs.add(tuple(sorted((key, ground))))
                gap = _footprint_distance(position, footprints[ground])[0]
                if gap > 0.0:
                    self.notes.append(f"{key} stands {gap * 1000:.0f} mm beside {ground}, its nearest ground")
        if unmated:
            self.notes.append(f"{len(unmated)} half(s) without a partner in the design (it sits on a bar the design "
                              f"leaves out): {', '.join(unmated)}")
        return frozenset(pairs)

    # --- --- pass 2: schema 2, replayed along the schedule --- ---

    def actions(self, raw: List["_RawAction"]) -> Dict[str, Action]:
        """Every action in schema 2, in schedule order: `attached`, `built` and tool states replayed along the schedule.

        - Ids name the bar, the action and each movement's role (`B10_J`, `B10_J_insert`), as Rhino writes them.
        - A manual mount attaches the bar to every flange carrying it or its halves, and ends with the scaffolding
          tools on the male or ground half at their TCP. A support gripper is on its bar from the end of the movement
          before its close, and the bar is attached to it when the grip closes.
        - Tighten (with `overlaps_next`) and the insert become one movement that builds the bar.
        - The untighten opening every release becomes a `loosen` drive of the ungrasp that follows it: it backs the
          jointing screw off, and the bar stays built.
        - An ungrasp or open releases what that flange holds; a tool leaves the body its arm no longer holds at the
          end of the next arm motion (the retreat).
        """
        flange_of = {tool: link_id(robot_id, flange) for robot_id, robot in self.robots.items()
                     for flange, tool in robot.tools.items()}
        grounds = frozenset(key for key in self.bodies if key.startswith("ground/"))
        tools = {tool: ToolState() for tool in self.tools}
        attached: Dict[str, Tuple[Holder, ...]] = {}
        built: Set[str] = set()
        folded: List[str] = []
        # Largest gaps between the export and the replay: built bars' poses, held bars' grasps.
        gaps = {"pose": 0.0, "grasp": 0.0}
        result: Dict[str, Action] = {}
        for action in raw:
            action_id = _unique(f"{action.bar.split('/', 1)[1]}_{ACTION_CODES[action.type]}", result)
            movements: List[Movement] = []
            last_links: Dict[str, Pose] = {}
            loosen: Dict[str, str] = {}
            mounted = False
            index = 0
            while index < len(action.movements):
                move = action.movements[index]
                merged = move.tool_action == "tighten" and move.overlaps_next and index + 1 < len(action.movements)
                arm_move = action.movements[index + 1] if merged else move
                index += 2 if merged else 1
                grips = {tool: _GRIPS[move.tool_action] for tool in move.tools} if move.tool_action in _GRIPS else {}
                if move.tool_action == "untighten":
                    loosen.update((tool, "loosen") for tool in move.tools)
                    folded.append(move.id)
                    continue
                on: Dict[str, Optional[str]] = {}
                if move.tool_action == "close":
                    # * The gripper arrives on its bar at the end of the movement before the close.
                    previous = movements[-1] if movements else None
                    arrive = {tool: action.bar for tool in move.tools}
                    if previous is None:
                        on.update(arrive)
                    else:
                        if previous.target is not None:
                            movements[-1] = replace(previous, target=replace(previous.target,
                                                                             on={**previous.target.on, **arrive}))
                        for tool in move.tools:
                            tools[tool] = replace(tools[tool], on=action.bar)
                start = State(robots=dict(move.robots), present=grounds | built | set(attached),
                              attached=dict(attached), built=frozenset(built), tools=dict(tools))
                self._compare_export(move, start, gaps)

                # * What the movement changes.
                after = dict(attached)
                if move.kind == "manual":
                    after[action.bar] = self._mounted_holders(action, move)
                    for holder in after[action.bar]:
                        tool = self.robots[action.robot].tools.get(split_link_id(holder.to)[1])
                        if tool is not None:
                            on[tool] = self._half_at_tcp(holder, tool, action.bar)
                for tool, grip in sorted(grips.items()):
                    if grip == "closed" and self.tools[tool].kind == "robotiq":
                        holder = self._support_holder(move, flange_of[tool], action.bar, built)
                        after[action.bar] = (*after.get(action.bar, ()), holder)
                    elif grip == "open":
                        after = {bar: tuple(holder for holder in holders if holder.to != flange_of[tool])
                                 for bar, holders in after.items()}
                        after = {bar: holders for bar, holders in after.items() if holders}
                for arm in arm_move.arms:
                    tool = self.robots[action.robot].tools.get(split_link_id(arm)[1])
                    body = tools[tool].on if tool is not None else None
                    if body is not None and not any(holder.to == arm for holder in after.get(self._bar(body), ())):
                        on[tool] = None
                built_after = built | {action.bar} if merged else built
                path = arm_move.kind if arm_move.kind in ("free", "linear") else None
                notes = {**move.notes, **arm_move.notes} if merged else move.notes
                drives = {tool: "tighten" for tool in move.tools} if merged else {}
                if grips and loosen:
                    drives.update(loosen)
                    loosen = {}
                target = Target(joints=dict(arm_move.joints), links=dict(arm_move.links), tools=grips, on=on,
                                attached=after if after != attached else None,
                                built=frozenset(built_after) if built_after != built else None)
                role = _role(action.type, arm_move.kind if merged else move.kind, grips, merged, mounted)
                movement = Movement(
                    id=_unique(f"{action_id}_{role}", {m.id for m in movements}), start=start, arms=arm_move.arms,
                    path=path, coupled=arm_move.coupled if path else False,
                    controller=_CONTROLLERS[arm_move.controller] if path else None,
                    line=self._line(action.robot, arm_move, start, last_links) if path == "linear" else {},
                    drives=drives,
                    ends_on="operator" if move.kind == "manual" else "tools" if move.kind == "tool" else "target",
                    target=target if target != Target() else None,
                    label=arm_move.label or move.label,
                    notes={key: value for key, value in notes.items()
                           if key not in _CONSUMED_NOTES and isinstance(value, (str, int, float, bool))})
                movements.append(movement)

                # * After the movement.
                attached, built = after, set(built_after)
                mounted = mounted or move.kind == "manual"
                for tool, grip in grips.items():
                    tools[tool] = replace(tools[tool], grip=grip)
                for tool, body in on.items():
                    tools[tool] = replace(tools[tool], on=body)
                last_links.update(arm_move.links)
            if loosen:
                self.notes.append(f"{action_id}: an untighten with no ungrasp after it was dropped")
            result[action_id] = Action(id=action_id, type=action.type, robot=action.robot, bar=action.bar,
                                       movements=tuple(movements), ground=action.ground,
                                       supports_until=action.supports_until, label=action.label)
        self.notes.append(f"folded {len(folded)} untighten movement(s) into the ungrasp after them, as a loosen drive")
        self.notes.append(f"export against the replay: built bars at most {gaps['pose'] * 1000:.3g} mm from their "
                          f"design pose, held bars' grasps at most {gaps['grasp'] * 1000:.3g} mm from the export's")
        if self.filled:
            self.notes.append(f"{self.filled} configuration(s) listed only some joints; the others were taken from "
                              f"the robot's previous state, or zero")
        if self.no_line:
            self.notes.append(f"{self.no_line} linear movement(s) without a line: neither notes nor start and target "
                              f"give one")
        return result

    def _bar(self, body: str) -> Optional[str]:
        """The bar a body belongs to: itself, or a half's mount."""
        return body if body.startswith("bars/") else self.bodies[body].mount

    def _mount(self, half: str) -> Pose:
        """A half's pose in its bar's frame, from the two design poses."""
        return compose(invert(self.bodies[self.bodies[half].mount].pose), self.bodies[half].pose)

    def _mounted_holders(self, action: "_RawAction", move: "_RawMovement") -> Tuple[Holder, ...]:
        """The holders of a bar the operator mounts, each with the bar's pose in its flange's frame.

        First the flange the export attaches the bar to, then every other flange carrying one of its halves.
        """
        link, grasp = move.carried[action.bar]
        holders = [Holder(link, grasp)]
        for body, (other, offset) in sorted(move.carried.items()):
            if self.bodies.get(body) and self.bodies[body].mount == action.bar \
                    and other not in {holder.to for holder in holders}:
                holders.append(Holder(other, compose(offset, invert(self._mount(body)))))
        return tuple(holders)

    def _support_holder(self, move: "_RawMovement", flange: str, bar: str, built: Set[str]) -> Holder:
        """A support gripper's hold on a built bar: the bar's design pose in the flange frame, by forward kinematics."""
        robot, link = split_link_id(flange)
        robot_state = move.robots.get(robot)
        if bar not in built or robot_state is None or robot_state.joints is None:
            raise ValueError(f"{move.id}: {robot} closes on {bar}, which is not built, or its joints are unknown")
        world = link_pose(self._files[robot][0], robot_state.base, robot_state.joints, link)
        return Holder(flange, compose(invert(world), self.bodies[bar].pose))

    def _half_at_tcp(self, holder: Holder, tool: str, bar: str) -> Optional[str]:
        """The male or ground half of a bar at a scaffolding tool's TCP, from the holder's grasp; None for a gripper.

        Raises:
            ValueError: If no such half is within _TCP_REACH of the TCP.
        """
        if self.tools[tool].kind != "scaffolding_v3":
            return None
        tcp = np.asarray(self.tools[tool].tcp.position)
        halves = [key for key, body in self.bodies.items() if body.mount == bar and key.endswith(("_male", "_ground"))]
        distance, half = min((float(np.linalg.norm(np.subtract(compose(holder.grasp, self._mount(key)).position, tcp))),
                              key) for key in halves)
        if distance > _TCP_REACH:
            raise ValueError(f"{tool}: no male or ground half of {bar} at its TCP (nearest {half}, "
                             f"{distance * 1000:.1f} mm away)")
        return half

    def _compare_export(self, move: "_RawMovement", start: State, gaps: Dict[str, float]) -> None:
        """Track how far the export's own state is from the replayed one: built bars' poses, attached bars' grasps."""
        for bar in start.built:
            if bar in move.poses and bar not in start.attached:
                gaps["pose"] = max(gaps["pose"], _gap(move.poses[bar], self.bodies[bar].pose))
        for bar, holders in start.attached.items():
            if bar in move.carried:
                link, offset = move.carried[bar]
                for holder in holders:
                    if holder.to == link:
                        gaps["grasp"] = max(gaps["grasp"], _gap(offset, holder.grasp))

    def _line(self, robot: str, move: "_RawMovement", start: State, last_links: Dict[str, Pose]) -> Dict[str, LineSpec]:
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
                begin = self._flange_pose(robot, flange, start, last_links)
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

    def _flange_pose(self, robot: str, flange: str, start: State, last_links: Dict[str, Pose]) -> Optional[Pose]:
        """Where a flange is at a movement's start: forward kinematics of the start joints, else the previous target."""
        robot_state = start.robots.get(robot)
        if robot_state is not None and robot_state.joints is not None and robot_state.base is not None:
            return link_pose(self._files[robot][0], robot_state.base, robot_state.joints, flange.rsplit("/", 1)[1])
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


def _role(action_type: str, kind: str, grips: Dict[str, str], merged: bool, mounted: bool) -> str:
    """A movement's role in its id, as Rhino names them (format §3.1), e.g. "insert" or "to_grasp"."""
    if merged:
        return "insert"
    if kind == "manual":
        return "mount"
    if kind == "tool":
        closing = "closed" in grips.values()
        if action_type in ("bar_jointing", "bar_release"):
            return "grasp" if closing else "ungrasp"
        return "close" if closing else "open"
    if kind == "linear":
        return "to_grasp" if action_type == "bar_holding" else "retreat"
    return {"bar_jointing": "transfer" if mounted else "load", "bar_release": "home", "bar_holding": "approach",
            "bar_holding_release": "leave"}[action_type]


def _unique(name: str, taken) -> str:
    """`name`, or `name_2`, `name_3`, … if it is taken."""
    found, number = name, 1
    while found in taken:
        number += 1
        found = f"{name}_{number}"
    return found


# --- --- --- --- --- GEOMETRY --- --- --- --- ---

def _same(a: Pose, b: Pose) -> bool:
    """Whether two poses are equal within _SAME (quaternion sign ignored)."""
    return _gap(a, b) < _SAME


def _gap(a: Pose, b: Pose) -> float:
    """The larger of two poses' position distance and quaternion difference (sign ignored)."""
    q_a, q_b = np.asarray(a.orientation), np.asarray(b.orientation)
    return max(float(np.linalg.norm(np.subtract(a.position, b.position))),
               float(min(np.abs(q_a - q_b).max(), np.abs(q_a + q_b).max())))


def _footprints(body: BodySpec) -> List[np.ndarray]:
    """The walkable polygons of a ground body in world, each (n, 3): the top faces of its slabs (`_slab`)."""
    matrix = body.pose.matrix()
    return [shape.vertices[:len(shape.vertices) // 2] @ matrix[:3, :3].T + matrix[:3, 3]
            for shape in body.geometry.collision]


def _footprint_distance(point: np.ndarray, polygons: List[np.ndarray]) -> Tuple[float, float]:
    """How far a point is from a ground seen from above (0 above a polygon), then its height above that polygon."""
    best = (np.inf, np.inf)
    for polygon in polygons:
        xy, x, y = polygon[:, :2], point[0], point[1]
        inside = False
        for (x1, y1), (x2, y2) in zip(xy, np.roll(xy, -1, axis=0)):
            if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
                inside = not inside
        if inside:
            distance = 0.0
        else:
            starts, ends = xy, np.roll(xy, -1, axis=0)
            edges = ends - starts
            t = np.clip(np.einsum("ij,ij->i", point[:2] - starts, edges) / np.maximum(
                np.einsum("ij,ij->i", edges, edges), 1e-18), 0.0, 1.0)
            distance = float(np.min(np.linalg.norm(starts + t[:, None] * edges - point[:2], axis=1)))
        best = min(best, (distance, abs(float(point[2] - polygon[:, 2].mean()))))
    return best


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
