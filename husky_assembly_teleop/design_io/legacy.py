"""
Read an old compas_fab export into a schema 1 `Design`.

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

from .compas_fab import PARKED_POSITION, pose_from_frame
from .geometry import Geometry, TriMesh
from .pose import Pose
from .robot_files import movable_joints, urdf_joints, urdf_links
from .types import (Action, Attached, BodySpec, Design, Movement, RobotSpec, RobotState, State, Target,
                    ToolSpec, link_id)
from .timing import Stopwatch
from .version import writer_info

#: Movement class -> (movement type, coupled).
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

#: Notes that became fields; every other note is passed through as it is.
_CONSUMED_NOTES = ("bar_pose_is_placeholder",)

#: Depth of the slab the walkable ground polygons are extruded into, downward, metres.
GROUND_THICKNESS = 0.05

#: Coordinates beyond this are millimetres (WalkableGround.json is written in Rhino's units).
_MILLIMETRE_THRESHOLD = 50.0

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
    ground = _walkable_ground(export.folder / "WalkableGround.json", converter, report)
    converter.bodies.update(ground)
    design_actions = {entry["action_id"]: converter.action(entry, action, tuple(ground)) for entry, action in actions}
    if converter.filled:
        report(f"{converter.filled} configuration(s) listed only some joints; the others were taken from the "
               f"robot's previous state, or zero")
    report(f"{len(converter.robots)} robots, {len(converter.tools)} tools, {len(converter.bodies)} bodies, "
           f"{len(design_actions)} actions, {sum(len(a.movements) for a in design_actions.values())} movements")
    return Design(folder=None, writer=writer_info(), robots=converter.robots, tools=converter.tools,
                  bodies=converter.bodies, schedule=tuple(entry["action_id"] for entry, _ in actions),
                  actions=design_actions)


# --- --- --- --- --- CONVERSION --- --- --- --- ---

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
        self.tools: Dict[str, ToolSpec] = {}
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
            self.tools[tool] = ToolSpec(id=tool, geometry=_model_geometry(model), tcp=pose_from_frame(model.frame),
                                        kind=self._tool_kind(name), touches=tuple(sorted(touches)))

        for robot, (urdf, srdf, _) in self._files.items():
            flanges = mounts.get(robot, {})
            self.robots[robot] = RobotSpec(id=robot, urdf=urdf, srdf=srdf, serial=self._serials.get(robot),
                                           tools={flange: self._tool_ids[(robot, name)]
                                                  for flange, name in sorted(flanges.items())})

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

    # --- --- actions and states --- ---

    def action(self, entry: dict, action, ground: Tuple[str, ...]) -> Action:
        """One schedule entry as an Action.

        Args:
            entry: Its ActionSchedule.json entry.
            action: The loaded rs_data_structure action.
            ground: Ids of the ground bodies, present in every state.
        """
        robot = self.robot_ids[entry["robot"]]
        cell = self.cells[action.robot_id]
        movements = tuple(self.movement(robot, cell, movement, action.active_bar_id, ground)
                          for movement in action.movements)
        kind = ACTION_KINDS[type(action).__name__]
        return Action(id=entry["action_id"], type=kind, robot=robot, bar=f"bars/{action.active_bar_id}",
                      movements=movements,
                      ground=tuple(f"ground/{name}" for name in getattr(action, "walkable_ground_ids", []) or []),
                      supports_until=tuple(f"bars/{bar}" for bar in getattr(action, "supported_until", []) or []),
                      label=action.tag or "")

    def movement(self, robot: str, cell: RobotCell, movement, bar: str, ground: Tuple[str, ...]) -> Movement:
        """One rs_data_structure movement, with its start state and target.

        Args:
            robot: The acting robot's id.
            cell: Its cell.
            movement: The rs_data_structure movement.
            bar: The action's bar name ("B3").
            ground: Ids of the ground bodies.
        """
        kind, coupled = MOVEMENT_KINDS[type(movement).__name__]
        flanges = tuple(link_id(robot, flange) for flange in self.robots[robot].tools)
        tools = tuple(self._tool_ids[(robot, name)] for name in getattr(movement, "tool_names", []) or [])
        notes = {key: value for key, value in (movement.notes or {}).items() if key not in _CONSUMED_NOTES}
        is_placeholder = (movement.notes or {}).get("bar_pose_is_placeholder")
        placeholder = frozenset({f"bars/{bar}"}) if is_placeholder else frozenset()
        start = self.state(robot, cell, movement.start_state, ground, placeholder)
        return Movement(id=movement.movement_id, type=kind, controller=movement.controller, start=start,
                        arms=flanges if kind in ("free", "linear") else (), coupled=coupled, tools=tools,
                        tool_action=getattr(movement, "tool_action", None) if kind == "tool" else None,
                        overlaps_next=bool(getattr(movement, "overlaps_next", False)),
                        target=self.target(robot, movement, flanges), label=movement.tag or "", notes=notes)

    def target(self, robot: str, movement, flanges: Tuple[str, ...]) -> Optional[Target]:
        """A movement's target configuration and flange frames, or None when it has neither.

        Args:
            robot: The acting robot's id.
            movement: The rs_data_structure movement.
            flanges: The acting robot's flange link ids.
        """
        joints = {}
        if movement.target_configuration is not None:
            joints[robot] = dict(zip(movement.target_configuration.joint_names,
                                     map(float, movement.target_configuration.joint_values)))
        links = {}
        for key, frame in (movement.target_ee_frames or {}).items():
            # "arm" on a single-arm robot, "left"/"right" on the dual-arm one.
            matches = [flange for flange in flanges if key == "arm" or f"/{key}_" in flange]
            if len(matches) != 1:
                raise ValueError(f"{movement.movement_id}: cannot tell which flange target {key!r} is for")
            links[matches[0]] = pose_from_frame(frame)
        return Target(joints=joints, links=links) if joints or links else None

    def state(self, robot: str, cell: RobotCell, legacy: RobotCellState, ground: Tuple[str, ...],
              placeholder: frozenset) -> State:
        """One compas_fab start state as a State.

        Args:
            robot: The acting robot's id.
            cell: Its cell.
            legacy: The compas_fab state.
            ground: Ids of the ground bodies, always present.
            placeholder: Ids whose pose is a placeholder.
        """
        robots: Dict[str, Optional[RobotState]] = {other: None for other in self.robots}
        configuration = legacy.robot_configuration
        robots[robot] = RobotState(base=pose_from_frame(legacy.robot_base_frame),
                                   joints=None if configuration is None else self._joints(robot, configuration))
        touches = set()
        for name, tool_state in legacy.tool_states.items():
            if not name.startswith("ObstacleRobot"):
                continue
            other = self.robot_ids[name[len("ObstacleRobot"):]]
            if tool_state.frame is None or np.allclose(list(tool_state.frame.point), PARKED_POSITION, atol=1e-3):
                continue
            robots[other] = RobotState(base=pose_from_frame(tool_state.frame),
                                       joints=self._joints(other, tool_state.configuration))

        present, poses, attached = set(ground), {}, {}
        for name, body_state in legacy.rigid_body_states.items():
            key = body_id(name)
            if body_state.is_hidden:
                continue
            present.add(key)
            if body_state.attached_to_link:
                attached[key] = Attached(to=link_id(robot, body_state.attached_to_link),
                                         grasp=pose_from_frame(body_state.attachment_frame))
            elif body_state.attached_to_tool:
                raise ValueError(f"{key}: attached to a tool; the converter does not handle that yet")
            elif body_state.frame is not None and not _same(pose_from_frame(body_state.frame), self.bodies[key].pose):
                poses[key] = pose_from_frame(body_state.frame)
            for link in body_state.touch_links:
                touches.add(tuple(sorted((key, link_id(robot, link)))))
            for other in body_state.touch_bodies:
                touches.add(tuple(sorted((key, self._touch_id(robot, other)))))
        return State(robots=robots, present=frozenset(present), poses=poses, attached=attached,
                     touches=frozenset(touches), placeholder=placeholder)

    def _touch_id(self, robot: str, name: str) -> str:
        """Our id of a compas_fab touch_bodies entry: a body, a tool of the acting robot, or another robot."""
        if name.startswith("ObstacleRobot"):
            return self.robot_ids[name[len("ObstacleRobot"):]]
        if (robot, name) in self._tool_ids:
            return self._tool_ids[(robot, name)]
        return body_id(name)

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


def _walkable_ground(path: Path, converter: _Converter, report: Callable[[str], None]) -> Dict[str, BodySpec]:
    """The walkable ground patches as `ground/<id>` slab bodies in metres; empty if the file is missing.

    Args:
        path: WalkableGround.json.
        converter: For the robots' wheel links, allowed to touch the ground.
        report: Told about the conversion.
    """
    if not path.is_file():
        report("no WalkableGround.json: no ground bodies")
        return {}
    wheels = tuple(sorted(link_id(robot, link) for robot, (urdf, _, _) in converter._files.items()
                          for link in urdf_links(urdf) if "wheel" in link))
    bodies = {}
    for name, mesh in json_load(str(path))["grounds"].items():
        vertices, faces = mesh.to_vertices_and_faces()
        points = np.asarray(vertices, dtype=float)
        if np.abs(points).max() > _MILLIMETRE_THRESHOLD:
            points = points * 0.001
            report(f"ground/{name}: millimetres converted to metres")
        slabs = tuple(_slab(points[face]) for face in faces)
        key = f"ground/{name}"
        bodies[key] = BodySpec(id=key, pose=Pose(), geometry=Geometry(slabs, slabs), touches=wheels)
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
