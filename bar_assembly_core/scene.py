"""
A scene: the world at one moment, as `Body` and `RobotObject` objects, for planners, mirrors and the 3D view.

Whoever owns a scene may edit it, e.g. disable the held bar for a plan. Mirrors compare each scene with the one they
applied last: poses, joints, `enabled` and attachments by value; `Geometry` and `RobotModel` by identity.

- ! To change a shape or a robot model, assign a new object; never edit one in place.
- ! Hand a scene to another thread or a mirror only once nobody edits it any more.
- ! Ids under "tracked/" are measured objects that move every tick: mirrors never build them concave.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Callable, Dict, Mapping, Optional, Tuple, Union

from .design_io.pose import Pose, compose
from .robot import RobotObject

if TYPE_CHECKING:
    from .design_io.geometry import Geometry

#: First id segments of robots and of measured objects.
ROBOTS, TRACKED = "robots", "tracked"

#: Forward kinematics given by the scene's owner: (robot, link name) -> the link's world pose.
LinkPose = Callable[[RobotObject, str], Pose]


def robot_id(name: str) -> str:
    """The id of a robot: "robots/<name>", e.g. "robots/a200-0806"."""
    return f"{ROBOTS}/{name}"


def tracked_id(name: str) -> str:
    """The id of a measured object: "tracked/<name>"."""
    return f"{TRACKED}/{name}"


@dataclass(frozen=True)
class Attachment:
    """A body held by a robot link, or fixed to another body; it moves with its parent.

    Attributes:
        parent: A robot id ("robots/cindy") or a body id ("tracked/probe").
        link: URDF link name for a robot, or None for its base (and for a body).
        grasp: The body's pose in the parent link's (or body's) frame.
    """

    parent: str
    link: Optional[str]
    grasp: Pose


@dataclass(eq=False)
class Body:
    """One collision object in the scene. Its owner may change its fields in place.

    Attributes:
        id: Unique path, e.g. "bars/B1" or "<owning plugin>/<group…>/<name>". Never changes.
        geometry: Its shape. ! Assign a new one to change it; never edit it in place.
        placement: A world pose, or an attachment to a robot link or another body.
        touches: Ids allowed to touch it: bodies, "robots/<name>" (whole robot), "robots/<name>/<link>",
            mounted tools ("tools/AT3L"). Either side may list the other.
        label: Display text, e.g. in collision messages. Empty: use the id.
        color: (r, g, b, a) from 0 to 1 for the 3D view, or None for grey.
        enabled: False: never collides and is not drawn, but mirrors keep it built, so turning it back on is
            cheap. * Prefer this over removing and re-adding bodies that come and go.
    """

    id: str
    geometry: Geometry
    placement: Union[Pose, Attachment]
    touches: Tuple[str, ...] = ()
    label: str = ""
    color: Optional[Tuple[float, float, float, float]] = None
    enabled: bool = True

    def copy(self) -> Body:
        """A copy that later changes to this body don't reach. Shares the geometry.

        ? Built by hand: about 4x faster than `copy.copy`.
        """
        return Body(self.id, self.geometry, self.placement, self.touches, self.label, self.color, self.enabled)


@dataclass(eq=False)
class SceneSnapshot:
    """The world at one moment: bodies with their resolved world poses, and robots.

    Attributes:
        tick: Index of the monitor tick it was taken in, or -1.
        time: Time it was taken, or 0.
        bodies: Every body, by id. ! Disabled bodies are included: check `Body.enabled` before treating one as an
            obstacle.
        world_poses: The world pose of every body in `bodies`, attachments resolved (`world_poses`).
            ! Recompute it after moving a robot or changing a placement.
        robots: Every robot, by id; absent ones disabled.
    """

    tick: int = -1
    time: float = 0.0
    bodies: Dict[str, Body] = field(default_factory=dict)
    world_poses: Dict[str, Pose] = field(default_factory=dict)
    robots: Dict[str, RobotObject] = field(default_factory=dict)

    def copy(self) -> SceneSnapshot:
        """A copy whose bodies and robots can be edited without changing this one. Shares geometry and models."""
        return SceneSnapshot(self.tick, self.time, {key: body.copy() for key, body in self.bodies.items()},
                             dict(self.world_poses), {key: robot.copy() for key, robot in self.robots.items()})

    def label(self, object_id: str) -> str:
        """Display text for any id: a body's or robot's label, else the id itself."""
        found = self.bodies.get(object_id) or self.robots.get(object_id)
        return found.label if found is not None and found.label else object_id


def same_source(new: object, old: object) -> bool:
    """Whether a mirror may keep what it built from `old` for `new`: the same `Geometry` or `RobotModel` object.

    ! The one test every mirror uses: by identity, never by value, so an unchanged object is never rebuilt and
      comparing never costs more than a pointer.
    """
    return new is old


def world_poses(bodies: Mapping[str, Body], robots: Mapping[str, RobotObject],
                link_pose: LinkPose) -> Dict[str, Pose]:
    """The world pose of every body whose parent has one: its own pose, or its parent's composed with the grasp.

    Args:
        bodies: The bodies, by id.
        robots: The robots, by id.
        link_pose: Forward kinematics for bodies held by a robot link.

    Returns:
        dict[str, Pose]: By body id. Bodies attached to a missing robot or body are left out.
    """
    poses: Dict[str, Optional[Pose]] = {}

    def resolve(body_id: str, seen: frozenset) -> Optional[Pose]:
        if body_id in poses:
            return poses[body_id]
        placement = bodies[body_id].placement
        if isinstance(placement, Pose):
            pose = placement
        elif placement.parent in robots:
            robot = robots[placement.parent]
            parent = robot.base if placement.link is None else link_pose(robot, placement.link)
            pose = compose(parent, placement.grasp)
        elif placement.parent in bodies and placement.parent not in seen:
            parent = resolve(placement.parent, seen | {body_id})
            pose = None if parent is None else compose(parent, placement.grasp)
        else:
            pose = None
        poses[body_id] = pose
        return pose

    for body_id in bodies:
        resolve(body_id, frozenset())
    return {body_id: pose for body_id, pose in poses.items() if pose is not None}
