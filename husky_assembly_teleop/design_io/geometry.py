"""
Shapes of scene and design bodies: triangle meshes and primitives (box, cylinder), in the body's frame.

* Each backend uses what it supports natively (viser and PyBullet draw and check
  primitives exactly) and turns the rest into triangles with `shape_mesh`.
* `Geometry.from_rigid_body` imports a compas_fab body. It is slow: call it on a loading thread.
! Never change a `Geometry` or a shape after building it. Mirrors cache what
  they build per object and only rebuild when they get a different one.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from typing import TYPE_CHECKING, Union

import numpy as np
from trimesh import Trimesh
from trimesh.creation import box, cylinder

from .pose import Pose

if TYPE_CHECKING:
    from compas.datastructures import Mesh
    from compas_fab.robots import RigidBody

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
        """Build a mesh from any vertex and triangle sequences.

        Args:
            vertices: (n, 3) vertex positions, metres.
            faces: (m, 3) vertex indices of triangles.

        Returns:
            TriMesh: Read-only copies of the arrays, with `convex` computed.
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
    """The triangles of a shape, in the body's frame, for backends without primitives.

    ? Cached: primitives compare by value, so equal boxes share one mesh.

    Args:
        shape: Any shape. A TriMesh is returned as is.

    Returns:
        TriMesh: The shape as triangles, its origin applied.
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
        collision: What collision checks use. ! Empty means the body never collides
            (as in compas_fab); `ctx.scene.put` warns about such bodies.
    """

    visual: tuple[Shape, ...]
    collision: tuple[Shape, ...]

    @classmethod
    def from_rigid_body(cls, rigid_body: RigidBody) -> Geometry:
        """Import a compas_fab rigid body as triangle meshes, scaled to metres. Slow: call on a loading thread.

        Args:
            rigid_body: The source body; it is not modified or kept.

        Returns:
            Geometry: Triangulated meshes.
        """
        visual = tuple(_from_compas(mesh, rigid_body.native_scale) for mesh in rigid_body.visual_meshes)
        # ? No fallback to the visual meshes: compas_fab's PyBullet backend treats a body
        #   without collision meshes as never colliding, and every mirror must agree with it.
        collision = tuple(_from_compas(mesh, rigid_body.native_scale) for mesh in rigid_body.collision_meshes)
        return cls(visual, collision)


def _from_compas(mesh: Mesh, scale: float) -> TriMesh:
    """Triangulate a compas mesh and scale its vertices.

    Args:
        mesh: The compas mesh; it is not modified.
        scale: Uniform scale to metres, e.g. a rigid body's native_scale.

    Returns:
        TriMesh: The scaled triangles.
    """
    vertices, faces = mesh.to_vertices_and_faces(triangulated=True)
    return TriMesh.from_arrays(np.asarray(vertices, dtype=np.float64) * scale, faces)


def box_geometry(size: tuple[float, float, float]) -> Geometry:
    """A box centred at the body's origin, drawn and checked exactly.

    Args:
        size: Side lengths (x, y, z), metres.

    Returns:
        Geometry: One box, for both drawing and collisions.
    """
    shapes = (BoxShape(tuple(float(v) for v in size)),)
    return Geometry(shapes, shapes)


def cylinder_geometry(radius: float, height: float) -> Geometry:
    """An upright cylinder along Z, centred at the body's origin, drawn and checked exactly.

    Args:
        radius: Radius, metres.
        height: Length along Z, metres.

    Returns:
        Geometry: One cylinder, for both drawing and collisions.
    """
    shapes = (CylinderShape(float(radius), float(height)),)
    return Geometry(shapes, shapes)
