"""
A design as compas_fab objects: one `RobotCell` per acting robot, one `RobotCellState` per state.

    to_robot_cell(design, "robots/cindy")          -> RobotCell   (format App. A)
    to_cell_state(design, "robots/cindy", state)   -> RobotCellState for that cell

* compas_fab plans one robot per cell, so each acting robot gets its own cell: every other robot
  is a `ToolModel` of its whole URDF, with its tools welded to its flanges, keyed by its robot id.
* Keys in the cell are our ids (`bars/B1`, `tools/AT3L`, `robots/alice`), or other names via
  `names` (our id -> cell name) for code that expects the old ones.
* The helpers here (`frame_from_pose`, `rigid_body`, `load_model`, `subtree`, `filled`) are shared
  with the monitor's `world/mirrors/compas_fab.py`.
! Importing this module imports compas; `design_io` itself never imports it.
! A robot that is not in a state is parked far away (PARKED_POSITION): compas_fab needs every tool
  in every state. The producer's own cells do the same.
"""

from __future__ import annotations

import hashlib
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

#: Where a robot that is not in a state is put in a compas_fab state: far from any cell, on the floor.
PARKED_POSITION = (50.0, 50.0, 0.0)

# URDF path -> loaded model. Loading a robot with its meshes takes seconds; every cell shares one.
# ! Shared: never change a cached model; copy it first (`model.copy()`).
_MODELS: Dict[Path, RobotModel] = {}

# SHA-1 of a mesh file's bytes -> its compas meshes. Robots share mesh files: the same UR arm and
# wheels on every husky, the wheel four times per robot. ! Shared: never change these meshes.
_MESHES: Dict[str, List[Mesh]] = {}


# --- --- --- --- --- CONVERSIONS --- --- --- --- ---

def frame_from_pose(pose: Pose) -> Frame:
    """A compas frame from one of our poses.

    Args:
        pose: Position and (x, y, z, w) quaternion.

    Returns:
        Frame: The same pose.
    """
    x, y, z, w = pose.orientation
    return Frame.from_quaternion([w, x, y, z], point=list(pose.position))


def pose_from_frame(frame: Frame) -> Pose:
    """One of our poses from a compas frame.

    Args:
        frame: The frame.

    Returns:
        Pose: The same pose, quaternion as (x, y, z, w).
    """
    w, x, y, z = frame.quaternion.wxyz
    return Pose.from_arrays(frame.point, (x, y, z, w))


def compas_mesh(mesh: TriMesh) -> Mesh:
    """A compas mesh from one of ours.

    Args:
        mesh: Our triangle mesh.

    Returns:
        Mesh: The same triangles.
    """
    return Mesh.from_vertices_and_faces(mesh.vertices.tolist(), mesh.faces.tolist())


def _joined(shapes) -> Optional[Mesh]:
    """All shapes as one compas mesh, or None if there are none.

    Args:
        shapes: Our shapes.

    Returns:
        Mesh | None: Their triangles in one mesh.
    """
    meshes = [shape_mesh(shape) for shape in shapes]
    if not meshes:
        return None
    offsets = np.cumsum([0] + [len(mesh.vertices) for mesh in meshes[:-1]])
    vertices = np.vstack([mesh.vertices for mesh in meshes])
    faces = np.vstack([mesh.faces + offset for mesh, offset in zip(meshes, offsets)])
    return Mesh.from_vertices_and_faces(vertices.tolist(), faces.tolist())


def rigid_body(geometry: Geometry, draw_visual: bool = False) -> RigidBody:
    """A compas_fab rigid body of a geometry, as triangle meshes in metres.

    Args:
        geometry: Our geometry; it is not kept.
        draw_visual: Use the visual shapes as visual meshes. By default the collision meshes
            are also the visual ones, so what a PyBullet window shows is what is checked.

    Returns:
        RigidBody: One mesh per shape.
    """
    collision = [compas_mesh(shape_mesh(shape)) for shape in geometry.collision]
    visual = [compas_mesh(shape_mesh(shape)) for shape in geometry.visual] if draw_visual else collision
    return RigidBody(visual_meshes=visual, collision_meshes=collision, native_scale=1.0)


def load_model(urdf_file) -> RobotModel:
    """Parse a URDF and load its meshes, cached per file. Slow (seconds) the first time.

    ? Relative mesh paths are made absolute first: compas_robots resolves a plain path
      against the working directory, not the URDF's folder.

    Args:
        urdf_file: A URDF; its mesh paths relative to it or absolute.

    Returns:
        RobotModel: With geometry. ! Shared: copy before changing it.
    """
    path = Path(urdf_file).resolve()
    model = _MODELS.get(path)
    if model is None:
        model = RobotModel.from_urdf_string(resolved_urdf_text(path))
        # ? Instead of `model.load_geometry()`, which parses every OBJ in pure Python (~3 s per robot)
        #   and parses a file again for every link using it: read with trimesh, once per file content.
        for link in model.links:
            for item in (*link.visual, *link.collision):
                shape = item.geometry.shape
                filename = getattr(shape, "filename", None)
                if filename:
                    shape.meshes = _file_meshes(filename)
        _MODELS[path] = model
    return model


def _file_meshes(filename: str) -> List[Mesh]:
    """A mesh file as compas meshes, read once per file content.

    Args:
        filename: Absolute path, possibly with `file://`.

    Returns:
        list[Mesh]: One mesh (all of the file's geometry). ! Shared between every link using it.
    """
    path = Path(filename[len("file://"):] if filename.startswith("file://") else filename)
    key = hashlib.sha1(path.read_bytes()).hexdigest()
    meshes = _MESHES.get(key)
    if meshes is None:
        mesh = trimesh.load(str(path), force="mesh", process=False)
        meshes = [Mesh.from_vertices_and_faces(mesh.vertices.tolist(), mesh.faces.tolist())]
        _MESHES[key] = meshes
    return meshes


def subtree(model: RobotModel, link_name: str) -> set:
    """A link and every link below it.

    Args:
        model: The robot.
        link_name: Where to start.

    Returns:
        set[str]: Link names, `link_name` included.
    """
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
        joints: Values by joint name; others are ignored. None keeps the defaults.

    Returns:
        Configuration: The new configuration.
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
    """A tool of the acting robot: its shapes as one mesh, its TCP as the tool frame.

    Args:
        design: The design.
        tool_id: The tool.
        name: Its key in the cell.

    Returns:
        ToolModel: The tool, its base at the flange link.
    """
    tool = design.tools[tool_id]
    return ToolModel(_joined(tool.geometry.visual), frame_from_pose(tool.tcp),
                     collision=_joined(tool.geometry.collision), name=name)


def _robot_as_tool(design: Design, robot_id: str, name: str) -> ToolModel:
    """Another robot as one articulated tool: its whole URDF, its tools welded to its flanges.

    ? As the producer's frozen-robot obstacles (`robot_obstacles.py`): each tool becomes a link
      on a fixed joint at its flange, so it follows the arm and collides with it.

    Args:
        design: The design.
        robot_id: The robot.
        name: Its key in the cell.

    Returns:
        ToolModel: Its base at the robot's URDF root.
    """
    robot = design.robots[robot_id]
    # ? from_robot_model builds new links (sharing the meshes, which are never changed), so the
    #   tools are welded onto the result and the cached model stays as it is. No deep copy: that
    #   copied every mesh and took ~6 s per robot.
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
    """A compas_fab cell for one acting robot (format App. A).

    Args:
        design: The design.
        robot_id: The acting robot.
        names: Our id -> key in the cell, for ids that should keep another name.
        include: Which bodies to put in the cell, by id; all by default.

    Returns:
        RobotCell: The acting robot with its SRDF, its tools, every other robot and the bodies.
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
    """The SRDF group that ends at a flange link, preferring one rooted at the URDF root.

    ? Arm-only and base-rooted groups move the same joints (format R17); the base-rooted one
      takes targets in the robot base frame, as the producer plans the assembly robot.

    Args:
        cell: A cell of the robot.
        link: Flange link name, e.g. "left_ur_arm_tool0".

    Returns:
        str: The group name.

    Raises:
        KeyError: If no group ends at that link.
    """
    groups = [group for group in cell.group_names if cell.get_end_effector_link_name(group) == link]
    if not groups:
        raise KeyError(f"no SRDF group of {cell.robot_model.name} ends at link {link!r}")
    root = cell.robot_model.root.name
    rooted = [group for group in groups if cell.get_base_link_name(group) == root]
    return (rooted or groups)[0]


def to_cell_state(design: Design, robot_id: str, state: State, cell: RobotCell,
                  names: Optional[Mapping[str, str]] = None,
                  joints: Optional[Mapping[str, float]] = None) -> RobotCellState:
    """A compas_fab state for a cell made by `to_robot_cell` (format App. A).

    Args:
        design: The design.
        robot_id: The acting robot.
        state: The state.
        cell: The acting robot's cell; bodies it leaves out are left out here too.
        names: As given to `to_robot_cell`.
        joints: Joint values to use for the acting robot where the state has none (e.g. from
            `carry.assumed_joints`). None keeps compas_fab's "no configuration".

    Returns:
        RobotCellState: The state.
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
    """Forward kinematics from each robot's URDF, for `types.world_pose`.

    Args:
        design: The design whose robots are used.

    Returns:
        LinkPose: (robot id, link, joints, base) -> link pose in world. Joints not given are zero.
    """
    def link_pose(robot_id: str, link: str, joints: Mapping[str, float], base: Pose) -> Pose:
        model = load_model(design.robots[robot_id].urdf)
        frame = model.forward_kinematics(filled(model.zero_configuration(), joints), link_name=link)
        return compose(base, pose_from_frame(frame))
    return link_pose
