"""Tests for `geometry`: primitives, read-only triangle meshes, and importing compas_fab bodies."""

import numpy as np
import pytest
from compas.datastructures import Mesh
from compas_fab.robots import RigidBody

from husky_assembly_teleop.world.geometry import (BoxShape, CylinderShape, Geometry, TriMesh, box_geometry,
                                            cylinder_geometry, shape_mesh)
from husky_assembly_teleop.world.scene import Pose


def _extents(mesh: TriMesh) -> np.ndarray:
    """Return the side lengths of a mesh's axis-aligned bounding box."""
    return mesh.vertices.max(axis=0) - mesh.vertices.min(axis=0)


def _quad_cube(size: float) -> Mesh:
    """Build a cube from six quad faces, with one corner at the origin."""
    vertices = [[x, y, z] for x in (0, size) for y in (0, size) for z in (0, size)]
    faces = [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1], [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]]
    return Mesh.from_vertices_and_faces(vertices, faces)


def test_box_and_cylinder_are_primitives():
    """box_geometry and cylinder_geometry hold one exact shape, the same for drawing and collisions."""
    box = box_geometry((1, 2, 3))
    assert box.visual == box.collision == (BoxShape((1.0, 2.0, 3.0)),)
    cylinder = cylinder_geometry(0.5, 2)
    assert cylinder.visual == cylinder.collision == (CylinderShape(0.5, 2.0),)
    assert box.collision[0].convex and cylinder.collision[0].convex


def test_box_mesh():
    """A box as triangles: 12 convex triangles, centred at its origin."""
    mesh = shape_mesh(BoxShape((1.0, 2.0, 3.0), origin=Pose((5.0, 0.0, 0.0))))
    assert mesh.faces.shape == (12, 3)
    assert mesh.vertices.dtype == np.float64 and mesh.faces.dtype == np.int32
    np.testing.assert_allclose(_extents(mesh), (1.0, 2.0, 3.0))
    np.testing.assert_allclose(mesh.vertices.mean(axis=0), (5.0, 0.0, 0.0), atol=1e-12)
    assert mesh.convex


def test_cylinder_mesh():
    """A cylinder as triangles stands along Z with the requested radius and height."""
    mesh = shape_mesh(CylinderShape(0.5, 2.0))
    np.testing.assert_allclose(_extents(mesh), (1.0, 1.0, 2.0), atol=1e-6)
    assert mesh.vertices[:, 2].min() == pytest.approx(-1.0)
    assert mesh.convex


def test_shape_mesh_is_cached():
    """Equal primitives give the very same mesh; a mesh is returned as is."""
    assert shape_mesh(BoxShape((1.0, 1.0, 1.0))) is shape_mesh(BoxShape((1.0, 1.0, 1.0)))
    mesh = shape_mesh(BoxShape((1.0, 1.0, 1.0)))
    assert shape_mesh(mesh) is mesh


def test_arrays_read_only():
    """Writing to a mesh's arrays raises."""
    mesh = shape_mesh(BoxShape((1.0, 1.0, 1.0)))
    with pytest.raises(ValueError):
        mesh.vertices[0, 0] = 5.0
    with pytest.raises(ValueError):
        mesh.faces[0, 0] = 1


def test_from_arrays_copies():
    """Changing the source arrays afterwards does not change the mesh."""
    vertices = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=float)
    mesh = TriMesh.from_arrays(vertices, [[0, 1, 2]])
    vertices[0, 0] = 9.0
    assert mesh.vertices[0, 0] == 0.0


def test_concave():
    """Two separate boxes merged into one mesh are not convex."""
    a = shape_mesh(BoxShape((1.0, 1.0, 1.0)))
    b_vertices = a.vertices + (3.0, 0.0, 0.0)
    mesh = TriMesh.from_arrays(np.vstack([a.vertices, b_vertices]), np.vstack([a.faces, a.faces + len(a.vertices)]))
    assert not mesh.convex


def test_from_rigid_body():
    """Quads are triangulated and native_scale is applied; the input is left unchanged."""
    visual, collision = _quad_cube(1000.0), _quad_cube(500.0)
    rigid_body = RigidBody([visual], [collision], native_scale=0.001)
    before = visual.to_vertices_and_faces()

    geometry = Geometry.from_rigid_body(rigid_body)

    assert geometry.visual[0].faces.shape == (12, 3)
    np.testing.assert_allclose(_extents(geometry.visual[0]), (1.0, 1.0, 1.0))
    np.testing.assert_allclose(_extents(geometry.collision[0]), (0.5, 0.5, 0.5))
    assert geometry.visual[0].convex
    assert visual.to_vertices_and_faces() == before
    assert rigid_body.native_scale == 0.001
    assert rigid_body.visual_meshes == [visual] and rigid_body.collision_meshes == [collision]


def test_from_rigid_body_without_collision_meshes():
    """With no collision meshes the body is drawn but never collides, as in compas_fab."""
    geometry = Geometry.from_rigid_body(RigidBody([_quad_cube(1.0)], [], native_scale=2.0))
    assert geometry.collision == ()
    assert len(geometry.visual) == 1
