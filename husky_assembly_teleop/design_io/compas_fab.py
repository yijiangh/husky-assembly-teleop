"""
A design as compas_fab objects: one `RobotCell` per acting robot (format App. A), and its `RobotCellState`s.

Every other robot is a `ToolModel` of its whole URDF with its tools welded on; cell keys are our ids unless
`names` maps them. A robot absent from a state is parked at PARKED_POSITION, since compas_fab needs every tool.
! Importing this module imports compas; `design_io` itself never imports it.
"""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
from typing import Callable, Dict, List, Mapping, Optional

import numpy as np
import trimesh
from compas.datastructures import Mesh
from compas.geometry import Frame
from compas_fab.robots import RigidBody, RigidBodyState, RobotCell, RobotCellState, RobotSemantics, ToolState
from compas_robots import Configuration, RobotModel, ToolModel
from compas_robots.model import Joint

from .geometry import Geometry, TriMesh, shape_mesh
from .pose import Pose, compose
from .robot_files import resolved_urdf_text
from .types import ROBOT_PREFIX, Design, LinkPose, State, split_link_id

#: Where a robot absent from a state is put: far from any cell, on the floor.
PARKED_POSITION = (50.0, 50.0, 0.0)

# (URDF path, with visuals) -> loaded model (loading takes seconds). ! Shared: copy before changing (`model.copy()`).
_MODELS: Dict[tuple, RobotModel] = {}

# SHA-1 of a mesh file's bytes -> its compas meshes; robots share many mesh files. ! Shared: never change them.
_MESHES: Dict[str, List[Mesh]] = {}

#: Rows per write when saving an OBJ file. Each write holds the GIL; short ones keep the main thread responsive.
_OBJ_ROWS = 5000


class FileMesh(Mesh):
    """A compas mesh read from a mesh file (`_file_meshes`); saves itself as OBJ fast, and once per path.

    ? compas_fab's `set_robot_cell` writes every robot mesh to "<its guid>.obj" each time, in pure Python:
      ~10 s per robot, holding the GIL and stalling the monitor. These meshes never change, so a file
      this mesh already wrote is still right.

    Attributes:
        arrays: Vertices (V, 3) and triangles (F, 3), as read; None on a copy, which writes the compas way.
        written: Paths this mesh wrote; set together with `arrays`.
    """

    arrays: Optional[tuple] = None
    written: Optional[set] = None

    def to_obj(self, filepath, precision=None, unweld=False, **kwargs) -> None:
        """Write the mesh as an OBJ file at `filepath`, unless this mesh already wrote it there.

        Args:
            filepath: Where to write.
            precision: Ignored when written from `arrays` (9 decimals).
            unweld: Passed on when written the compas way.
        """
        if self.arrays is None:
            super().to_obj(filepath, precision=precision, unweld=unweld, **kwargs)
            return
        if filepath in self.written and os.path.exists(filepath):
            return
        vertices, faces = self.arrays
        with open(filepath, "w") as file:
            for start in range(0, len(vertices), _OBJ_ROWS):
                rows = vertices[start:start + _OBJ_ROWS]
                file.write(("v %.9f %.9f %.9f\n" * len(rows)) % tuple(rows.ravel().tolist()))
            for start in range(0, len(faces), _OBJ_ROWS):
                rows = faces[start:start + _OBJ_ROWS] + 1  # OBJ counts vertices from 1
                file.write(("f %d %d %d\n" * len(rows)) % tuple(rows.ravel().tolist()))
        self.written.add(filepath)


# --- --- --- --- --- CONVERSIONS --- --- --- --- ---

def frame_from_pose(pose: Pose) -> Frame:
    """A compas frame from one of our poses."""
    x, y, z, w = pose.orientation
    return Frame.from_quaternion([w, x, y, z], point=list(pose.position))


def pose_from_frame(frame: Frame) -> Pose:
    """One of our poses from a compas frame."""
    w, x, y, z = frame.quaternion.wxyz
    return Pose.from_arrays(frame.point, (x, y, z, w))


def compas_mesh(mesh: TriMesh) -> Mesh:
    """A compas mesh from one of ours."""
    return Mesh.from_vertices_and_faces(mesh.vertices.tolist(), mesh.faces.tolist())


def _joined(shapes) -> Optional[Mesh]:
    """All shapes as one compas mesh, or None if there are none."""
    meshes = [shape_mesh(shape) for shape in shapes]
    if not meshes:
        return None
    offsets = np.cumsum([0] + [len(mesh.vertices) for mesh in meshes[:-1]])
    vertices = np.vstack([mesh.vertices for mesh in meshes])
    faces = np.vstack([mesh.faces + offset for mesh, offset in zip(meshes, offsets)])
    return Mesh.from_vertices_and_faces(vertices.tolist(), faces.tolist())


def rigid_body(geometry: Geometry, draw_visual: bool = False) -> RigidBody:
    """A compas_fab rigid body of a geometry, one triangle mesh per shape, in metres.

    Args:
        geometry: Our geometry; it is not kept.
        draw_visual: Use the visual shapes as visual meshes; by default they are the collision ones,
            so a PyBullet window shows what is checked.
    """
    collision = [compas_mesh(shape_mesh(shape)) for shape in geometry.collision]
    visual = [compas_mesh(shape_mesh(shape)) for shape in geometry.visual] if draw_visual else collision
    return RigidBody(visual_meshes=visual, collision_meshes=collision, native_scale=1.0)


def load_model(urdf_file, visual: bool = True) -> RobotModel:
    """Parse a URDF and load its meshes, cached per file; slow (seconds) the first time.

    Args:
        urdf_file: The URDF.
        visual: Keep the visual shapes. Without them (for planning) loading is much faster: the visual
            meshes are the large ones.

    Returns:
        RobotModel: With geometry. ! Shared: copy before changing it.
    """
    path = Path(urdf_file).resolve()
    model = _MODELS.get((path, visual))
    if model is None:
        model = RobotModel.from_urdf_string(resolved_urdf_text(path))
        if not visual:
            for link in model.links:
                link.visual = []
        # ? Not `model.load_geometry()`: that parses every OBJ in pure Python (~3 s per robot), once per link.
        for link in model.links:
            for item in (*link.visual, *link.collision):
                shape = item.geometry.shape
                filename = getattr(shape, "filename", None)
                if filename:
                    shape.meshes = _file_meshes(filename)
        _MODELS[path, visual] = model
    return model


def _file_meshes(filename: str) -> List[Mesh]:
    """A mesh file (absolute path, maybe `file://`) as one compas mesh, read once per file content.

    Returns:
        list[Mesh]: One mesh. ! Shared between every link using it.
    """
    path = Path(filename[len("file://"):] if filename.startswith("file://") else filename)
    key = hashlib.sha1(path.read_bytes()).hexdigest()
    meshes = _MESHES.get(key)
    if meshes is None:
        mesh = trimesh.load(str(path), force="mesh", process=False)
        file_mesh = FileMesh.from_vertices_and_faces(mesh.vertices.tolist(), mesh.faces.tolist())
        file_mesh.arrays = (np.asarray(mesh.vertices, dtype=float), np.asarray(mesh.faces, dtype=np.int64))
        file_mesh.written = set()
        meshes = [file_mesh]
        _MESHES[key] = meshes
    return meshes


def subtree(model: RobotModel, link_name: str) -> set:
    """The names of a link and every link below it."""
    names, todo = set(), [model.get_link_by_name(link_name)]
    while todo:
        link = todo.pop()
        names.add(link.name)
        todo.extend(model.get_link_by_name(joint.child.link) for joint in link.joints)
    return names


def filled(configuration: Configuration, joints: Optional[Mapping[str, float]]) -> Configuration:
    """A copy of a configuration with the values in `joints` for the joints it has.

    Args:
        configuration: Joint names, types and default values.
        joints: Values by joint name; unknown names are ignored. None keeps the defaults.
    """
    joints = joints or {}
    values = [float(joints.get(name, value))
              for name, value in zip(configuration.joint_names, configuration.joint_values)]
    return Configuration(values, configuration.joint_types, configuration.joint_names)


# --- --- --- --- --- CELLS --- --- --- --- ---

def _name(names: Optional[Mapping[str, str]], our_id: str) -> str:
    """The cell's key for one of our ids."""
    return names.get(our_id, our_id) if names else our_id


def _tool_model(design: Design, tool_id: str, name: str) -> ToolModel:
    """A tool of the acting robot, keyed `name`: its shapes as one mesh, its TCP as the tool frame."""
    tool = design.tools[tool_id]
    return ToolModel(_joined(tool.geometry.visual), frame_from_pose(tool.tcp),
                     collision=_joined(tool.geometry.collision), name=name)


def _robot_as_tool(design: Design, robot_id: str, name: str) -> ToolModel:
    """Another robot as one articulated tool keyed `name`: its whole URDF, each tool a link fixed at its flange."""
    robot = design.robots[robot_id]
    # ? from_robot_model builds new links, so the cached model is not changed; a deep copy took ~6 s per robot.
    tool = ToolModel.from_robot_model(load_model(robot.urdf), Frame.worldXY())
    for link_name, tool_id in robot.tools.items():
        geometry = design.tools[tool_id].geometry
        visual, collision = _joined(geometry.visual), _joined(geometry.collision)
        child = tool.add_link(f"{link_name}_tool", visual_meshes=[visual] if visual else None,
                              collision_meshes=[collision] if collision else None)
        tool.add_joint(f"{link_name}_tool_joint", Joint.FIXED, tool.get_link_by_name(link_name), child)
    tool.name = name
    return tool


def to_robot_cell(design: Design, robot_id: str, names: Optional[Mapping[str, str]] = None,
                  include: Optional[Callable[[str], bool]] = None) -> RobotCell:
    """A compas_fab cell: the acting robot with its SRDF and tools, every other robot, and the bodies.

    Args:
        design: The design.
        robot_id: The acting robot.
        names: Our id -> key in the cell, for ids that should keep another name.
        include: Which bodies to put in the cell, by id; all by default.
    """
    robot = design.robots[robot_id]
    model = load_model(robot.urdf)
    semantics = RobotSemantics.from_srdf_file(str(robot.srdf), model)
    tools = {_name(names, tool_id): _tool_model(design, tool_id, _name(names, tool_id))
             for tool_id in robot.tools.values()}
    for other in design.robots:
        if other != robot_id:
            tools[_name(names, other)] = _robot_as_tool(design, other, _name(names, other))
    bodies = {_name(names, body_id): rigid_body(body.geometry, draw_visual=True)
              for body_id, body in design.bodies.items() if include is None or include(body_id)}
    return RobotCell(model, semantics, tool_models=tools, rigid_body_models=bodies)


def planning_group(cell: RobotCell, link: str) -> str:
    """The SRDF group ending at a flange link whose base link is nearest the URDF root (targets in the base frame).

    ? E.g. `base_left_arm_manipulator` (from base_footprint) over `Left arm` (from the arm's base link); the
      URDF root itself (world_link) starts no group. Ties keep the SRDF order.

    Args:
        cell: A cell of the robot.
        link: Flange link name, e.g. "left_ur_arm_tool0".

    Raises:
        KeyError: If no group ends at that link.
    """
    groups = [group for group in cell.group_names if cell.get_end_effector_link_name(group) == link]
    if not groups:
        raise KeyError(f"no SRDF group of {cell.robot_model.name} ends at link {link!r}")
    return min(groups, key=lambda group: _depth(cell.robot_model, cell.get_base_link_name(group)))


def _depth(model: RobotModel, link_name: str) -> int:
    """How many joints lie between the URDF root and a link."""
    depth, link = 0, model.get_link_by_name(link_name)
    while link.parent_joint is not None:
        link = model.get_link_by_name(link.parent_joint.parent.link)
        depth += 1
    return depth


def to_cell_state(design: Design, robot_id: str, state: State, cell: RobotCell,
                  names: Optional[Mapping[str, str]] = None,
                  joints: Optional[Mapping[str, float]] = None) -> RobotCellState:
    """A compas_fab state for a cell made by `to_robot_cell`.

    Args:
        design: The design.
        robot_id: The acting robot.
        state: The state.
        cell: The acting robot's cell; bodies it leaves out are left out here too.
        names: As given to `to_robot_cell`.
        joints: Acting robot joints to use where the state has none (e.g. `carry.assumed_joints`).
    """
    robot = design.robots[robot_id]
    acting = state.robots[robot_id]
    acting_joints = acting.joints if acting.joints is not None else joints
    link_pose = compas_link_pose(design)
    own_links = {link.name for link in cell.robot_model.links}

    # * Every allowed contact, both ways: design-level touches of bodies and tools, and this state's.
    contacts: Dict[str, set] = {}
    pairs = set(state.touches)
    pairs |= {(body_id, other) for body_id, body in design.bodies.items() for other in body.touches}
    pairs |= {(tool_id, other) for tool_id, tool in design.tools.items() for other in tool.touches}
    for a, b in pairs:
        contacts.setdefault(a, set()).add(b)
        contacts.setdefault(b, set()).add(a)

    def split(touching) -> tuple:
        """Allowed contacts as compas_fab's (touch_links of the acting robot, touch_bodies)."""
        links, bodies = [], []
        for other in sorted(touching):
            if other.startswith(robot_id + "/"):
                links.append(split_link_id(other)[1])
            elif other == robot_id:
                links.extend(sorted(own_links))
            elif other.startswith(ROBOT_PREFIX):
                # Another robot is one tool: any of its links means the whole robot.
                bodies.append(_name(names, "/".join(other.split("/")[:2])))
            else:
                bodies.append(_name(names, other))
        return links, bodies

    # * Tools of the acting robot: attached to the group ending at their flange.
    tool_states = {}
    for link_name, tool_id in robot.tools.items():
        touch_links, _ = split(contacts.get(tool_id, ()))
        tool_states[_name(names, tool_id)] = ToolState(
            frame=None, attached_to_group=planning_group(cell, link_name), touch_links=touch_links,
            attachment_frame=Frame.worldXY())
    # * Every other robot: a tool at its base with its joints, or parked when absent.
    for other in design.robots:
        if other == robot_id:
            continue
        key = _name(names, other)
        other_state = state.robots.get(other)
        zero = cell.tool_models[key].zero_configuration()
        if other_state is None:
            tool_states[key] = ToolState(frame=Frame(PARKED_POSITION), configuration=zero)
        else:
            tool_states[key] = ToolState(frame=frame_from_pose(other_state.base),
                                         configuration=filled(zero, other_state.joints))

    # * Bodies: hidden when absent, attached when held by the acting robot, else stationary.
    body_states = {}
    for body_id, body in design.bodies.items():
        key = _name(names, body_id)
        if key not in cell.rigid_body_models:
            continue
        touch_links, touch_bodies = split(contacts.get(body_id, ()))
        body_state = RigidBodyState(frame=frame_from_pose(body.pose), touch_links=touch_links,
                                    touch_bodies=touch_bodies)
        attached = state.attached.get(body_id)
        if body_id not in state.present:
            body_state.is_hidden = True
        elif attached is not None and attached.to.startswith(robot_id + "/"):
            body_state.attached_to_link = split_link_id(attached.to)[1]
            body_state.attachment_frame = frame_from_pose(attached.grasp)
            body_state.frame = None
        elif attached is not None:
            # Held by another robot: stationary where that robot holds it, allowed to touch it.
            holder, link = split_link_id(attached.to)
            holder_state = state.robots.get(holder)
            if holder_state is not None:
                pose = compose(link_pose(holder, link, holder_state.joints or {}, holder_state.base),
                               attached.grasp)
                body_state.frame = frame_from_pose(pose)
            body_state.touch_bodies.append(_name(names, holder))
        else:
            body_state.frame = frame_from_pose(state.poses.get(body_id, body.pose))
        body_states[key] = body_state

    configuration = filled(cell.zero_full_configuration(), acting_joints) if acting_joints is not None else None
    return RobotCellState(robot_base_frame=frame_from_pose(acting.base), robot_configuration=configuration,
                          tool_states=tool_states, rigid_body_states=body_states)


# --- --- --- --- --- KINEMATICS --- --- --- --- ---

def compas_link_pose(design: Design) -> LinkPose:
    """Forward kinematics from each robot's URDF, for `types.world_pose`; joints not given are zero."""
    def link_pose(robot_id: str, link: str, joints: Mapping[str, float], base: Pose) -> Pose:
        model = load_model(design.robots[robot_id].urdf)
        frame = model.forward_kinematics(filled(model.zero_configuration(), joints), link_name=link)
        return compose(base, pose_from_frame(frame))
    return link_pose
