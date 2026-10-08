"""
Poses and shapes, the values every layer shares: a `Pose` in a parent frame, and a body's `Geometry`.

Backends draw primitives natively where they can and use `shape_mesh` for the rest.
! Never change a `Geometry` or shape after building it: mirrors cache per object and only rebuild for a new one.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from typing import TYPE_CHECKING, Union

import numpy as np
from scipy.spatial.transform import Rotation
from trimesh import Trimesh
from trimesh.creation import box, cylinder

if TYPE_CHECKING:
    from compas.datastructures import Mesh
    from compas_fab.robots import RigidBody


# --- --- --- --- --- POSES --- --- --- --- ---

@dataclass(frozen=True)
class Pose:
    """A pose in the world, or in a parent frame.

    Attributes:
        position: (x, y, z), metres.
        orientation: Quaternion (x, y, z, w), the ROS and PyBullet order.
    """

    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    @classmethod
    def from_arrays(cls, position, orientation) -> Pose:
        """Build a pose from any sequences (numpy arrays included), stored as plain floats.

        Args:
            position: (x, y, z), metres.
            orientation: Quaternion (x, y, z, w).
        """
        return cls(tuple(float(v) for v in position), tuple(float(v) for v in orientation))

    @classmethod
    def from_matrix(cls, matrix: np.ndarray) -> Pose:
        """Build a pose from a 4x4 homogeneous transform."""
        # ! Copy: some callers (yourdfpy) hand out read-only matrices, which this scipy version refuses.
        return cls.from_arrays(matrix[:3, 3], Rotation.from_matrix(np.array(matrix[:3, :3])).as_quat())

    def matrix(self) -> np.ndarray:
        """Return this pose as a 4x4 homogeneous transform."""
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat(self.orientation).as_matrix()
        matrix[:3, 3] = self.position
        return matrix


def compose(a: Pose, b: Pose) -> Pose:
    """Chain two poses: `b` is given in the frame of `a`; the result is in the frame `a` is in.

    Args:
        a: The parent frame's pose.
        b: The pose inside that frame.
    """
    rotation = Rotation.from_quat(a.orientation)
    return Pose.from_arrays(np.asarray(a.position) + rotation.apply(b.position),
                            (rotation * Rotation.from_quat(b.orientation)).as_quat())


def invert(a: Pose) -> Pose:
    """The inverse pose: where the parent frame is, seen from inside `a`."""
    inverse = Rotation.from_quat(a.orientation).inv()
    return Pose.from_arrays(-inverse.apply(a.position), inverse.as_quat())


#: Sides around the circle when a cylinder is turned into triangles.
CYLINDER_SECTIONS = 32


# --- --- --- --- --- SHAPES --- --- --- --- ---

@dataclass(frozen=True, eq=False)
class TriMesh:
    """A read-only triangle mesh, in metres, in the body's frame.

    Attributes:
        vertices: (n, 3) float64 vertex positions.
        faces: (m, 3) int32 vertex indices, one row per triangle.
        convex: Whether the mesh is convex.
    """

    vertices: np.ndarray
    faces: np.ndarray
    convex: bool

    @classmethod
    def from_arrays(cls, vertices, faces) -> TriMesh:
        """Build a mesh from read-only copies of any vertex and triangle sequences, with `convex` computed.

        Args:
            vertices: (n, 3) vertex positions, metres.
            faces: (m, 3) vertex indices of triangles.
        """
        vertices = np.array(vertices, dtype=np.float64).reshape(-1, 3)
        faces = np.array(faces, dtype=np.int32).reshape(-1, 3)
        vertices.setflags(write=False)
        faces.setflags(write=False)
        convex = bool(Trimesh(vertices, faces, process=False).is_convex)
        return cls(vertices, faces, convex)


@dataclass(frozen=True)
class BoxShape:
    """A box centred at `origin`.

    Attributes:
        size: Side lengths (x, y, z) in the origin's frame, metres.
        origin: Its pose in the body's frame.
    """

    size: tuple[float, float, float]
    origin: Pose = Pose()

    @property
    def convex(self) -> bool:
        """bool: Always True."""
        return True


@dataclass(frozen=True)
class CylinderShape:
    """An upright cylinder along the Z axis of `origin`, centred at it.

    Attributes:
        radius: Radius, metres.
        height: Length along Z, metres.
        origin: Its pose in the body's frame.
    """

    radius: float
    height: float
    origin: Pose = Pose()

    @property
    def convex(self) -> bool:
        """bool: Always True."""
        return True


#: Anything a Geometry is made of.
Shape = Union[TriMesh, BoxShape, CylinderShape]


@lru_cache(maxsize=None)
def shape_mesh(shape: Shape) -> TriMesh:
    """The triangles of a shape in the body's frame, its origin applied; a TriMesh is returned as is.

    ? Cached, so equal primitives share one mesh.
    """
    if isinstance(shape, TriMesh):
        return shape
    if isinstance(shape, BoxShape):
        mesh = box(extents=shape.size)
    else:
        mesh = cylinder(radius=shape.radius, height=shape.height, sections=CYLINDER_SECTIONS)
    mesh.apply_transform(shape.origin.matrix())
    return TriMesh.from_arrays(mesh.vertices, mesh.faces)


# --- --- --- --- --- GEOMETRY --- --- --- --- ---

@dataclass(frozen=True, eq=False)
class Geometry:
    """The shape of a body: shapes to draw and shapes to check collisions with.

    Attributes:
        visual: What the 3D view draws.
        collision: What collision checks use. ! Empty means the body never collides (as in compas_fab).
    """

    visual: tuple[Shape, ...]
    collision: tuple[Shape, ...]

    @classmethod
    def from_rigid_body(cls, rigid_body: RigidBody) -> Geometry:
        """Import a compas_fab rigid body as triangle meshes in metres. ! Slow: call it on a loading thread.

        Args:
            rigid_body: The source body; it is not modified or kept.
        """
        visual = tuple(_from_compas(mesh, rigid_body.native_scale) for mesh in rigid_body.visual_meshes)
        # ? No fallback to the visual meshes: compas_fab treats a body without collision meshes as never colliding.
        collision = tuple(_from_compas(mesh, rigid_body.native_scale) for mesh in rigid_body.collision_meshes)
        return cls(visual, collision)


def _from_compas(mesh: Mesh, scale: float) -> TriMesh:
    """Triangulate a compas mesh and scale its vertices to metres (e.g. by a rigid body's native_scale)."""
    vertices, faces = mesh.to_vertices_and_faces(triangulated=True)
    return TriMesh.from_arrays(np.asarray(vertices, dtype=np.float64) * scale, faces)


def box_geometry(size: tuple[float, float, float]) -> Geometry:
    """One box centred at the body's origin, for both drawing and collisions.

    Args:
        size: Side lengths (x, y, z), metres.
    """
    shapes = (BoxShape(tuple(float(v) for v in size)),)
    return Geometry(shapes, shapes)


def cylinder_geometry(radius: float, height: float) -> Geometry:
    """One cylinder along Z, centred at the body's origin, for both drawing and collisions.

    Args:
        radius: Radius, metres.
        height: Length along Z, metres.
    """
    shapes = (CylinderShape(float(radius), float(height)),)
    return Geometry(shapes, shapes)
