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
from compas_fab.robots import RigidBody
from compas_robots import Configuration, RobotModel

from ..design_io.geometry import Geometry, TriMesh, shape_mesh
from ..design_io.pose import Pose
from ..design_io.robot_files import resolved_urdf_text

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
