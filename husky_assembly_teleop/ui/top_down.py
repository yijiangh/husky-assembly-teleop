"""
A top-down picture of the scene, for the phone: floor cells, footprints of bodies and robot links, and dots.

World +y points up the picture, +x to the right.

A footprint is the convex hull of a collision shape's corners projected on the floor, so a robot shows as the union
of its links' outlines. The picture is an RGB array for viser's `add_image`, which sends it as a PNG.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from typing import Iterable

import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial import ConvexHull, QhullError

from scaffolding_assembly_core.geometry import Pose, Shape, shape_mesh
from scaffolding_assembly_core.scene import Scene, scene_parts

#: Footprint colours: bodies (obstacles, parts) and robots with their tools.
BODY_COLOR = (70, 70, 70)
ROBOT_COLOR = (40, 70, 150)
#: Thin lines between floor cells.
GRID_LINE_COLOR = (255, 255, 255)


@dataclass(frozen=True)
class Dot:
    """A round marker: a recorded point or the probe.

    Attributes:
        x: World x, metres.
        y: World y, metres.
        color: Fill, RGB 0-255.
        radius: Pixels.
        outline: Ring colour, RGB 0-255.
    """

    x: float
    y: float
    color: tuple[int, int, int]
    radius: int = 4
    outline: tuple[int, int, int] = (255, 255, 255)


@dataclass(frozen=True)
class FloorGrid:
    """The area drawn: square floor cells, world frame.

    Attributes:
        origin: (x, y) of the lower-left corner, metres.
        cell_size: Side of a cell, metres.
        shape: Cells along (x, y).
        cell_pixels: Side of a cell in the picture.
    """

    origin: tuple[float, float]
    cell_size: float
    shape: tuple[int, int]
    cell_pixels: int = 32

    def pixel(self, x: float, y: float) -> tuple[float, float]:
        """The picture's (column, row) of world (x, y)."""
        scale = self.cell_pixels / self.cell_size
        return (x - self.origin[0]) * scale, (self.shape[1] * self.cell_size - (y - self.origin[1])) * scale


@lru_cache(maxsize=None)
def _corners(shape: Shape) -> np.ndarray:
    """The corners of a shape's convex hull in its body's frame; few, so projecting them every frame is cheap."""
    vertices = np.asarray(shape_mesh(shape).vertices, dtype=float)
    try:
        return vertices[ConvexHull(vertices).vertices]
    except QhullError:  # flat or degenerate: keep every vertex
        return vertices


def footprint(shape: Shape, pose: Pose) -> np.ndarray:
    """The outline of `shape` at world `pose`, seen from above: (n, 2) world x y, in order around it."""
    matrix = pose.matrix()
    points = (_corners(shape) @ matrix[:3, :3].T + matrix[:3, 3])[:, :2]
    try:
        return points[ConvexHull(points).vertices]
    except QhullError:  # seen edge-on (a wall, a thin plate): its two ends along its longer side
        axis = int(np.ptp(points[:, 1]) > np.ptp(points[:, 0]))
        return points[[points[:, axis].argmin(), points[:, axis].argmax()]]


def scene_footprints(scene: Scene) -> Iterable[tuple[np.ndarray, tuple[int, int, int]]]:
    """Every footprint in `scene` with its colour: enabled bodies, then enabled robots' links and tools.

    A part the 3D view does not draw (no visual shapes, as the floor) is left out.
    """
    for part_id, geometry, pose in scene_parts(scene):
        if not geometry.visual:
            continue
        color = ROBOT_COLOR if part_id.startswith(("robots/", "tools/")) else BODY_COLOR
        for shape in geometry.collision or geometry.visual:
            yield footprint(shape, pose), color


def render_top_down(grid: FloorGrid, cell_colors: np.ndarray,
                    footprints: Iterable[tuple[np.ndarray, tuple[int, int, int]]], dots: Iterable[Dot]) -> np.ndarray:
    """Draw the floor, then the footprints, then the dots.

    Args:
        grid: The area drawn.
        cell_colors: RGB 0-255 per cell, shape (*grid.shape, 3), indexed [x cell, y cell].
        footprints: (outline, colour) pairs; outlines in world x y, as `footprint` gives them.
        dots: Drawn last, in order, so the last one is on top.

    Returns:
        np.ndarray: The picture, (rows, columns, 3) uint8.
    """
    # * Rows run down the picture while y runs up: transpose to [y, x], then flip the rows.
    floor = np.asarray(cell_colors, dtype=np.uint8).transpose(1, 0, 2)[::-1]
    floor = floor.repeat(grid.cell_pixels, axis=0).repeat(grid.cell_pixels, axis=1)
    image = Image.fromarray(np.ascontiguousarray(floor))
    draw = ImageDraw.Draw(image)
    width, height = image.size
    for k in range(1, grid.shape[0]):
        draw.line([(k * grid.cell_pixels, 0), (k * grid.cell_pixels, height)], fill=GRID_LINE_COLOR)
    for k in range(1, grid.shape[1]):
        draw.line([(0, k * grid.cell_pixels), (width, k * grid.cell_pixels)], fill=GRID_LINE_COLOR)
    for outline, color in footprints:
        corners = [grid.pixel(x, y) for x, y in outline]
        if len(corners) >= 3:
            draw.polygon(corners, fill=color)
        else:
            draw.line(corners, fill=color, width=3)
    for dot in dots:
        u, v = grid.pixel(dot.x, dot.y)
        r = dot.radius
        draw.ellipse([u - r, v - r, u + r, v + r], fill=dot.color, outline=dot.outline, width=2)
    return np.asarray(image)
