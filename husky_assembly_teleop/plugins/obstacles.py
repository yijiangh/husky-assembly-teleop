"""
Fixed lab obstacles (tables, cabinets as boxes; tripods as cylinders), put into the scene.

* Ids: "obstacles/boxes/<name>" and "obstacles/cylinders/<name>". The core draws them.
* Planners get them from the scene snapshot (`ctx.scene.snapshot`), like every other body.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass

from ..plugin_api.context import PluginContext
from ..world.geometry import box_geometry, cylinder_geometry
from ..plugin_api.plugin import HuskyPlugin, register
from ..world.scene import Body, Pose

#: Muted grey, so the obstacles read as furniture.
BOX_COLOR = (150, 150, 160)


@dataclass(frozen=True)
class Box:
    """One box standing on the floor.

    Attributes:
        name: Label, shown in collision messages.
        center: (x, y, z) of the box's middle, metres, world frame.
        size: (x, y, z) side lengths, metres, before turning.
        yaw: Turn about Z, radians.
    """

    name: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]
    yaw: float = 0.0

    @property
    def orientation(self) -> tuple[float, float, float, float]:
        """tuple[float, float, float, float]: `yaw` as an (x, y, z, w) quaternion."""
        return (0.0, 0.0, math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0))

    @classmethod
    def from_top_points(cls, name: str, points: tuple[tuple[float, float, float], ...]) -> Box:
        """Fit an axis-aligned box from floor to points measured on its top surface.

        The XY footprint is the points' bounding rectangle, so take them at opposite corners.
        The top height is their mean Z.

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

    #: Upright; the (x, y, z, w) quaternion shared with `Box`.
    orientation = (0.0, 0.0, 0.0, 1.0)

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


#: The lab's static obstacles, fitted from mocap pointer samples (2026-09-30) on each top surface.
BOXES = (
    Box.from_top_points("building cabinet", ((-0.412, 2.012, 1.094), (3.788, 2.001, 1.136), (-0.410, 2.612, 1.096))),
    Box.from_top_points("operator table A", ((-1.584, 0.462, 1.339), (-2.287, 0.456, 1.274), (-1.519, 2.116, 1.369))),
    Box.from_top_points("operator table B", ((-0.400, 2.756, 1.265), (-1.427, 1.421, 1.329), (-0.481, 1.562, 1.305))),
)

#: Tripods, as a circle through their feet (mocap pointer, 2026-09-30).
# ? Height is not measured; 2 m covers any head position.
CYLINDERS = (
    Cylinder.around_points("tripod", ((3.807, -1.303, 0.112), (3.035, -1.781, 0.114), (3.803, -2.258, 0.082)),
                           height=2.0),
)


def _slug(name: str) -> str:
    """Turn a display name into an id segment, e.g. "operator table A" -> "operator_table_A"."""
    return re.sub(r"[^A-Za-z0-9_.\-]+", "_", name)


@register
class ObstaclesPlugin(HuskyPlugin):
    """Puts the fixed boxes and cylinders into the scene."""

    name = "obstacles"

    def __init__(self):
        """Keep the obstacle lists; other code reads `boxes` and `cylinders`."""
        self.boxes: tuple[Box, ...] = BOXES
        self.cylinders: tuple[Cylinder, ...] = CYLINDERS

    def setup(self, ctx: PluginContext) -> None:
        """Put one scene body per box and cylinder. They are removed automatically when the plugin closes.

        Args:
            ctx: This plugin's context.
        """
        color = tuple(c / 255 for c in BOX_COLOR) + (1.0,)
        ctx.scene.put_many(
            [Body(f"{self.name}/boxes/{_slug(box.name)}", box_geometry(box.size),
                  Pose(box.center, box.orientation), label=box.name, color=color) for box in self.boxes]
            + [Body(f"{self.name}/cylinders/{_slug(cylinder.name)}",
                    cylinder_geometry(cylinder.radius, cylinder.height),
                    Pose(cylinder.center, cylinder.orientation), label=cylinder.name, color=color)
               for cylinder in self.cylinders])
