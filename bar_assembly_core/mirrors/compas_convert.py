"""
compas_fab conversions the mirrors build from: frames, meshes, rigid bodies, and URDF models cached per file.

! Importing this module imports compas.
"""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
from typing import Dict, List, Mapping, Optional

import numpy as np
import trimesh
from compas.datastructures import Mesh
from compas.geometry import Frame
from compas_fab.robots import RigidBody, RobotCell
from compas_robots import Configuration, RobotModel, ToolModel
from compas_robots.model import Joint

from ..design_io.geometry import Geometry, TriMesh, shape_mesh
from ..design_io.pose import Pose
from ..design_io.robot_files import resolved_urdf_text
from ..design_io.types import ToolSpec

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


def joined_mesh(shapes) -> Optional[Mesh]:
    """All shapes as one compas mesh, or None if there are none."""
    meshes = [shape_mesh(shape) for shape in shapes]
    if not meshes:
        return None
    offsets = np.cumsum([0] + [len(mesh.vertices) for mesh in meshes[:-1]])
    vertices = np.vstack([mesh.vertices for mesh in meshes])
    faces = np.vstack([mesh.faces + offset for mesh, offset in zip(meshes, offsets)])
    return Mesh.from_vertices_and_faces(vertices.tolist(), faces.tolist())


def tool_model(tool: ToolSpec, name: str) -> ToolModel:
    """A mounted tool as a compas_fab tool keyed `name`: its shapes as one mesh, its TCP as the tool frame."""
    return ToolModel(joined_mesh(tool.geometry.visual), frame_from_pose(tool.tcp),
                     collision=joined_mesh(tool.geometry.collision), name=name)


def robot_as_tool(urdf, tools: Mapping[str, ToolSpec], name: str, visual: bool = True) -> ToolModel:
    """A whole robot as one articulated tool keyed `name`, each mounted tool a link fixed at its flange.

    Args:
        urdf: The robot's URDF without tools.
        tools: Flange link -> mounted tool.
        name: The tool's key in the cell.
        visual: Keep the visual shapes; without them loading is faster.
    """
    # ? from_robot_model builds new links, so the cached model is not changed; a deep copy took ~6 s per robot.
    model = ToolModel.from_robot_model(load_model(urdf, visual=visual), Frame.worldXY())
    for link_name, tool in tools.items():
        shapes, collision = joined_mesh(tool.geometry.visual), joined_mesh(tool.geometry.collision)
        child = model.add_link(f"{link_name}_tool", visual_meshes=[shapes] if shapes and visual else None,
                               collision_meshes=[collision] if collision else None)
        model.add_joint(f"{link_name}_tool_joint", Joint.FIXED, model.get_link_by_name(link_name), child)
    model.name = name
    return model


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
