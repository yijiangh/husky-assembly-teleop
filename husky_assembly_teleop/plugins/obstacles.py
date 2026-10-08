"""
Fixed lab obstacles put into the scene: tables, cabinets and border walls as boxes, tripods as cylinders, and trusses.

Ids are "obstacles/boxes/<name>", "obstacles/cylinders/<name>" and "obstacles/trusses/<name>"; planners see them in
`ctx.scene.snapshot`. Fine-tune placements with the `debug_gizmo` plugin.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass

from ..plugin_api.context import PluginContext
from bar_assembly_core.design_io.geometry import BoxShape, Geometry, Shape, box_geometry, cylinder_geometry
from ..plugin_api.plugin import HuskyPlugin, register
from ..world.scene import Body
from bar_assembly_core.design_io.pose import Pose

#: Muted grey, so the obstacles read as furniture.
BOX_COLOR = (150, 150, 160)
#: Warning orange, so the border reads as floor tape.
BORDER_COLOR = (230, 140, 40)


@dataclass(frozen=True)
class Box:
    """One box standing on the floor.

    Attributes:
        name: Label, shown in collision messages.
        center: (x, y, z) of the box's middle, metres, world frame.
        size: (x, y, z) side lengths, metres, before turning.
        yaw: Turn about Z, radians.
        visual: Shapes to draw instead of the box, in its frame; it still collides as the box. None: draw the box.
    """

    name: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]
    yaw: float = 0.0
    visual: tuple[Shape, ...] | None = None

    @property
    def orientation(self) -> tuple[float, float, float, float]:
        """tuple[float, float, float, float]: `yaw` as an (x, y, z, w) quaternion."""
        return (0.0, 0.0, math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0))

    @property
    def geometry(self) -> Geometry:
        """Geometry: The box for collisions, drawn as `visual` if set."""
        geometry = box_geometry(self.size)
        return geometry if self.visual is None else Geometry(self.visual, geometry.collision)

    @classmethod
    def from_top_points(cls, name: str, points: tuple[tuple[float, float, float], ...]) -> Box:
        """Fit an axis-aligned box from the floor up to points measured on its top surface.

        The footprint is the points' bounding rectangle (so take opposite corners); the height is their mean Z.

        Args:
            name: Label, shown in collision messages.
            points: (x, y, z) points on the top surface, metres, world frame.

        Returns:
            Box: The fitted box, with no yaw.
        """
        xs, ys, zs = zip(*points)
        height = sum(zs) / len(zs)
        return cls(name, center=((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, height / 2),
                   size=(max(xs) - min(xs), max(ys) - min(ys), height))


@dataclass(frozen=True)
class Cylinder:
    """One upright cylinder standing on the floor.

    Attributes:
        name: Label, shown in collision messages.
        center: (x, y, z) of the cylinder's middle, metres, world frame.
        radius: Metres.
        height: Metres.
    """

    name: str
    center: tuple[float, float, float]
    radius: float
    height: float

    #: Upright, as an (x, y, z, w) quaternion like `Box.orientation`.
    orientation = (0.0, 0.0, 0.0, 1.0)

    @property
    def geometry(self) -> Geometry:
        """Geometry: The cylinder."""
        return cylinder_geometry(self.radius, self.height)

    @classmethod
    def around_points(cls, name: str, points: tuple[tuple[float, float, float], ...], height: float) -> Cylinder:
        """Fit the upright cylinder whose circle passes through three floor points, e.g. tripod feet.

        Args:
            name: Label, shown in collision messages.
            points: Three (x, y, z) points, metres, world frame. Z is ignored.
            height: Metres, from the floor.

        Returns:
            Cylinder: The fitted cylinder.
        """
        (ax, ay, _), (bx, by, _), (cx, cy, _) = points
        d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
        a2, b2, c2 = ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy
        x = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d
        y = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d
        return cls(name, center=(x, y, height / 2), radius=math.hypot(ax - x, ay - y), height=height)


@dataclass(frozen=True)
class Truss:
    """A square truss column standing on a square ground plate, axis aligned.

    Attributes:
        name: Label, shown in collision messages.
        center: (x, y, z) of the column's middle, metres, world frame; z is half the height.
        column: Side of the column, metres.
        plate: Side of the ground plate, metres.
        plate_thickness: Metres.
        height: Column height from the floor, metres.
    """

    name: str
    center: tuple[float, float, float]
    column: float = 0.3
    plate: float = 0.8
    plate_thickness: float = 0.02
    height: float = 2.0

    #: Upright and unturned, as an (x, y, z, w) quaternion like `Box.orientation`.
    orientation = (0.0, 0.0, 0.0, 1.0)

    @property
    def geometry(self) -> Geometry:
        """Geometry: Column and plate, for drawing and collisions."""
        shapes = (BoxShape((self.column, self.column, self.height)),
                  BoxShape((self.plate, self.plate, self.plate_thickness),
                           Pose((0.0, 0.0, (self.plate_thickness - self.height) / 2))))
        return Geometry(shapes, shapes)


#: Lab tables, metres; long side along Y.
TABLE_SIZE = (0.8, 1.6, 0.73)
#: The same tables with the long side along X.
TABLE_SIZE_X = (1.6, 0.8, 0.73)
#: Tripods and trusses are about this tall; covers any camera head.
STAND_HEIGHT = 2.0

#: The lab's furniture, from mocap probe points, or placed by hand with `debug_gizmo` (2026-10-06). All axis aligned.
BOXES = (
    Box.from_top_points("building cabinet", ((3.793, 2.004, 1.139), (-0.414, 2.014, 1.094), (-0.411, 2.612, 1.099))),
    # * Three corners probed; long side along Y.
    Box("main pc table", (-1.927, 1.251, TABLE_SIZE[2] / 2), TABLE_SIZE),
    # * Placed by hand.
    Box("pc table 2", (-1.218, 2.404, TABLE_SIZE[2] / 2), TABLE_SIZE_X),
    Box("pc table 3", (-2.816, 2.400, TABLE_SIZE[2] / 2), TABLE_SIZE_X),
    # * Only the room-side long edge probed; the tables stand against the -x wall.
    Box("bar table 1", (-3.232, -3.332, TABLE_SIZE[2] / 2), TABLE_SIZE),
    Box("bar table 2", (-3.258, -0.675, TABLE_SIZE[2] / 2), TABLE_SIZE),
)

#: Tripods, as cylinders up to STAND_HEIGHT.
_ADA = Cylinder.around_points("tripod ada", ((3.825, -1.347, 0.051), (3.825, -2.212, 0.034), (3.093, -1.787, 0.041)),
                              height=STAND_HEIGHT)
CYLINDERS = (
    _ADA,
    # * Placed by hand, with the ada tripod's radius.
    Cylinder("tripod net", (-3.499, -1.857, STAND_HEIGHT / 2), _ADA.radius, STAND_HEIGHT),
    Cylinder("tripod 3", (-3.389, -4.580, STAND_HEIGHT / 2), _ADA.radius, STAND_HEIGHT),
)

#: Placed by hand.
TRUSSES = (
    Truss("truss 1", (3.844, -4.974, STAND_HEIGHT / 2), height=STAND_HEIGHT),
    Truss("truss 2", (3.928, 5.744, STAND_HEIGHT / 2), height=STAND_HEIGHT),
)


@dataclass(frozen=True)
class Border:
    """Walls just outside a floor rectangle, drawn as a shaded band on the floor but colliding up to `height`.

    Attributes:
        x: (min, max) of the free area, metres, world frame.
        y: (min, max) of the free area, metres, world frame.
        width: Band width, metres, outward from the free area.
        height: Collision height, metres, from the floor.
        shade: Drawn thickness of the band, metres.
    """

    x: tuple[float, float]
    y: tuple[float, float]
    width: float = 0.2
    height: float = 2.0
    shade: float = 0.005

    def walls(self) -> tuple[Box, ...]:
        """tuple[Box, ...]: One wall per side, drawn `shade` high; the x walls cover the corners."""
        (x0, x1), (y0, y1), w, h = self.x, self.y, self.width, self.height
        sides = (("x min", (x0 - w / 2, (y0 + y1) / 2), (w, y1 - y0 + 2 * w)),
                 ("x max", (x1 + w / 2, (y0 + y1) / 2), (w, y1 - y0 + 2 * w)),
                 ("y min", ((x0 + x1) / 2, y0 - w / 2), (x1 - x0, w)),
                 ("y max", ((x0 + x1) / 2, y1 + w / 2), (x1 - x0, w)))
        return tuple(Box(f"border {side}", (cx, cy, h / 2), (sx, sy, h),
                         visual=(BoxShape((sx, sy, self.shade), Pose((0.0, 0.0, (self.shade - h) / 2))),))
                     for side, (cx, cy), (sx, sy) in sides)


#: The lab's usable floor area (2026-10-06).
BORDER = Border(x=(-3.7, 3.8), y=(-4.8, 5.6))


def _slug(name: str) -> str:
    """Turn a display name into an id segment, e.g. "main pc table" -> "main_pc_table"."""
    return re.sub(r"[^A-Za-z0-9_.\-]+", "_", name)


@register
class ObstaclesPlugin(HuskyPlugin):
    """Puts the fixed boxes, cylinders, trusses and border walls into the scene."""

    name = "obstacles"

    def __init__(self):
        """Keep the obstacle lists."""
        self.boxes: tuple[Box, ...] = BOXES
        self.cylinders: tuple[Cylinder, ...] = CYLINDERS
        self.trusses: tuple[Truss, ...] = TRUSSES
        self.border: Border | None = BORDER

    def setup(self, ctx: PluginContext) -> None:
        """Put one scene body per box, cylinder, truss and border wall; the core removes them when the plugin closes.

        Args:
            ctx: This plugin's context.
        """
        color = tuple(c / 255 for c in BOX_COLOR) + (1.0,)
        border_color = tuple(c / 255 for c in BORDER_COLOR) + (1.0,)
        walls = self.border.walls() if self.border else ()
        ctx.scene.put_many(
            [self._body("boxes", box, color) for box in self.boxes]
            + [self._body("cylinders", cylinder, color) for cylinder in self.cylinders]
            + [self._body("trusses", truss, color) for truss in self.trusses]
            + [self._body("boxes", wall, border_color) for wall in walls])

    def _body(self, group: str, obstacle: Box | Cylinder | Truss, color: tuple[float, float, float, float]) -> Body:
        """The scene body of one obstacle, with id "<plugin>/<group>/<name>"."""
        return Body(f"{self.name}/{group}/{_slug(obstacle.name)}", obstacle.geometry,
                    Pose(obstacle.center, obstacle.orientation), label=obstacle.name, color=color)
