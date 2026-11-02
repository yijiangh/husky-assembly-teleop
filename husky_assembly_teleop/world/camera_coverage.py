"""
Where Motive's cameras see a horizontal plane: each camera's footprint, and the plane split by how many see each point.

Pure geometry in the Z-up world, on shapely polygons. A camera sees the inside of the pyramid its field of view spans
from it; on a plane that is a convex polygon, clipped here to a rectangle of interest.
? Range and occlusion are not modelled: a camera sees all of its view, however far and whatever stands in it.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np
import shapely
from scipy.spatial.transform import Rotation
from shapely.geometry import Polygon, box

from ..robot_interface.mocap_definitions import MocapCamera

#: Each camera model's field of view, horizontal and vertical, degrees, by how its name in Motive starts; others get
#: CAMERA_FOV. ? From OptiTrack's spec sheets as remembered (wide-angle "W" lenses differ): check against the lab's.
CAMERA_FOVS = {"PrimeX 22": (79.0, 47.0), "PrimeX 13": (56.0, 46.0), "Prime 13": (56.0, 46.0)}
CAMERA_FOV = (56.0, 46.0)
#: A camera's frame as the relay announces it looks along +y with z up (Motive's -z ahead, y up, turned Z-up); this
#: turns it into a view frame looking along +z with -y up, as viser's frustums and pinhole cameras do.
VIEW_FROM_CAMERA = Rotation.from_euler("x", -90.0, degrees=True)


def camera_fov(name: str) -> tuple[float, float]:
    """A camera's field of view, horizontal and vertical, degrees, by its model (`CAMERA_FOVS`)."""
    return next((fov for model, fov in CAMERA_FOVS.items() if name.startswith(model)), CAMERA_FOV)


def view_rotation(camera: MocapCamera) -> Rotation:
    """The camera's view frame in the world: +z ahead, -y up."""
    return Rotation.from_quat(camera.orientation) * VIEW_FROM_CAMERA


def footprint(camera: MocapCamera, height: float, bounds: Polygon) -> Polygon:
    """The part of the plane z = `height` within `bounds` the camera sees: its view's four sides clip `bounds`."""
    tan_x, tan_y = np.tan(np.radians(camera_fov(camera.name)) / 2)
    rotation, centre = view_rotation(camera), np.asarray(camera.position, dtype=float)
    points = list(bounds.exterior.coords)[:-1]
    # * Inward normals of the view's sides, in the view frame: a point p is inside where every n . p >= 0.
    for normal in rotation.apply([(-1.0, 0.0, tan_x), (1.0, 0.0, tan_x), (0.0, -1.0, tan_y), (0.0, 1.0, tan_y)]):
        offset = normal[2] * (height - centre[2]) - normal[0] * centre[0] - normal[1] * centre[1]
        points = _clip(points, normal[0], normal[1], offset)
        if len(points) < 3:
            return Polygon()
    return Polygon(points)


def _clip(points: Sequence, a: float, b: float, c: float) -> list:
    """A convex polygon's part where a x + b y + c >= 0 (Sutherland-Hodgman, one edge)."""
    kept = []
    for (x0, y0), (x1, y1) in zip(points, [*points[1:], points[0]]):
        s0, s1 = a * x0 + b * y0 + c, a * x1 + b * y1 + c
        if s0 >= 0:
            kept.append((x0, y0))
        if (s0 >= 0) != (s1 >= 0):
            t = s0 / (s0 - s1)
            kept.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
    return kept


def outlines(footprints: Sequence[Polygon], bounds: Polygon) -> shapely.Geometry:
    """Every footprint's edge and `bounds`' edge, cut where they cross: the outlines of the faces they make."""
    return shapely.union_all([bounds.boundary, *(shape.boundary.intersection(bounds) for shape in footprints
                                                 if not shape.is_empty)])


def coverage_faces(footprints: Sequence[Polygon], bounds: Polygon) -> list[tuple[int, Polygon]]:
    """`bounds` cut by the footprints' edges into faces (`shapely.polygonize`), each with how many cover it."""
    faces = []
    for face in shapely.get_parts(shapely.polygonize([outlines(footprints, bounds)])):
        inside = face.representative_point()
        faces.append((sum(shape.covers(inside) for shape in footprints), face))
    return faces


def coverage_regions(footprints: Sequence[Polygon], bounds: Polygon) -> dict[int, shapely.Geometry]:
    """`bounds` split by how many footprints cover each point: the region seen by exactly n cameras, by n."""
    faces: dict[int, list] = {}
    for count, face in coverage_faces(footprints, bounds):
        faces.setdefault(count, []).append(face)
    return {count: shapely.union_all(parts) for count, parts in sorted(faces.items())}


def segments(lines: shapely.Geometry) -> np.ndarray:
    """Lines as (n, 2, 2) straight segments, for drawing."""
    found = [pair for line in shapely.get_parts(lines) if not line.is_empty
             for pair in zip(line.coords[:-1], line.coords[1:])]
    return np.array(found, dtype=float).reshape(-1, 2, 2)


def view_edges(camera: MocapCamera) -> np.ndarray:
    """The four edges of the camera's view, unit directions in the world: its pyramid's corner rays."""
    tan_x, tan_y = np.tan(np.radians(camera_fov(camera.name)) / 2)
    corners = np.array([(sx * tan_x, sy * tan_y, 1.0) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
    return view_rotation(camera).apply(corners / np.linalg.norm(corners, axis=1, keepdims=True))


def plane_point(origin: Sequence[float], direction: Sequence[float], height: float) -> np.ndarray | None:
    """Where a ray from `origin` along `direction` meets the plane z = `height`; None if it never does."""
    if abs(direction[2]) < 1e-9:
        return None
    t = (height - origin[2]) / direction[2]
    return np.add(origin, t * np.asarray(direction)) if t > 0 else None


def aim_point(camera: MocapCamera, height: float) -> np.ndarray | None:
    """Where the middle of the camera's view meets the plane z = `height`; None if it looks away from it."""
    return plane_point(camera.position, view_rotation(camera).apply((0.0, 0.0, 1.0)), height)


def triangles(region: shapely.Geometry) -> tuple[np.ndarray, np.ndarray]:
    """A region of the plane as triangles: (n, 2) vertices and (m, 3) vertex indices, for a mesh."""
    vertices, faces, index = [], [], {}
    for triangle in shapely.get_parts(shapely.constrained_delaunay_triangles(region)):
        corners = []
        for point in list(triangle.exterior.coords)[:3]:
            corners.append(index.setdefault(point, len(vertices)))
            if corners[-1] == len(vertices):
                vertices.append(point)
        faces.append(corners)
    return np.array(vertices, dtype=float).reshape(-1, 2), np.array(faces, dtype=np.int32).reshape(-1, 3)


def rectangle(origin: Sequence[float], size: Sequence[float]) -> Polygon:
    """The rectangle from `origin` (x, y) spanning `size` (x, y), metres."""
    return box(origin[0], origin[1], origin[0] + size[0], origin[1] + size[1])
