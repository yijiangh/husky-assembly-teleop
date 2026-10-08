"""
A scene: everything in the world at one moment, as plain data for planners, mirrors and the 3D view.

- ! Treat a snapshot as read-only once handed to another thread or a mirror.
- ! To change a body's shape, assign a new `Geometry`, never edit one in place: mirrors rebuild only
  when the geometry object changes.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

from .design_io.pose import Pose

if TYPE_CHECKING:
    from .design_io.geometry import Geometry

#: What `RobotEntry.config` holds: the monitor's `RobotConfig`; mirrors read its `serial`, `urdf_file`,
#: `srdf_file` and `arms`. ? Replaced by a core `RobotModel`.
RobotDescription = Any

#: First id segments that belong to the core, never to a plugin.
ROBOTS, TRACKED = "robots", "tracked"


# --- --- --- --- --- IDS --- --- --- --- ---

def robot_id(serial: str) -> str:
    """The id of a robot: "robots/<serial>"."""
    return f"{ROBOTS}/{serial}"


def tracked_id(name: str) -> str:
    """The id of a tracked object: "tracked/<name>"."""
    return f"{TRACKED}/{name}"


# --- --- --- --- --- WHAT PLUGINS PUT IN --- --- --- --- ---

@dataclass(frozen=True)
class Attachment:
    """A body held by a robot link or a tracked object; it moves with it.

    Attributes:
        parent: "robots/<serial>" or "tracked/<name>".
        link: URDF link name for a robot, or None for its base (and for a tracked object).
        grasp: The body's pose in the parent link's frame.
    """

    parent: str
    link: str | None
    grasp: Pose


@dataclass(eq=False)
class Body:
    """One collision object in the scene. Plugins may change its fields in place.

    Attributes:
        id: Unique path, "<owning plugin>/<group…>/<name>". Never changes.
        geometry: Its shape. ! Assign a new one to change it; never edit it in place.
        placement: A world pose, or an attachment to a robot link or tracked object.
        touches: Ids allowed to touch it: bodies, "robots/<serial>" (whole robot),
            "robots/<serial>/<link>", "tracked/<name>". Either side may list the other.
        label: Display text, e.g. in collision messages. Empty: use the id.
        color: (r, g, b, a) from 0 to 1 for the 3D view, or None for grey.
        enabled: False: never collides and is not drawn, but mirrors keep it built, so turning it back on is
            cheap. * Prefer this over removing and re-adding bodies that come and go.
    """

    id: str
    geometry: Geometry
    placement: Pose | Attachment
    touches: tuple[str, ...] = ()
    label: str = ""
    color: tuple[float, float, float, float] | None = None
    enabled: bool = True

    def copy(self) -> Body:
        """A copy that later changes to this body don't reach. Shares the geometry.

        ? Built by hand: about 4x faster than `copy.copy`.
        """
        return Body(self.id, self.geometry, self.placement, self.touches, self.label, self.color, self.enabled)


@dataclass(eq=False)
class TrackedDescription:
    """What a tracked object is, given when tracking starts.

    Attributes:
        geometry: Its shape, or None for a frame only that is not an obstacle (e.g. the mocap probe).
        touches: Ids allowed to touch it, as for `Body.touches`.
        label: Display text. Empty: use the id.
    """

    geometry: Geometry | None = None
    touches: tuple[str, ...] = ()
    label: str = ""


# --- --- --- --- --- THE COPY --- --- --- --- ---

@dataclass(eq=False)
class RobotEntry:
    """One robot, as measured at the start of the tick.

    Attributes:
        config: The robot's configuration. ! A different config object makes mirrors reload the model.
        base: Last valid mocap pose, or the configured default before any.
        base_tracked: Whether the latest mocap sample was valid.
        joints: Last measured value per actuated joint.
        unmeasured: Joints never measured; their value in `joints` is the arm's stow pose, or 0.
        base_time: ROS time of the mocap fix behind `base`, or None.
        joints_time: ROS time of the oldest arm's latest joint state, or None if an arm never reported.
            ! Base and arms are measured at different instants; while moving, `base_time - joints_time`
            is the error.
    """

    config: RobotDescription
    base: Pose
    base_tracked: bool
    joints: dict[str, float]
    unmeasured: frozenset[str]
    base_time: float | None
    joints_time: float | None


@dataclass(eq=False)
class TrackedEntry:
    """One tracked object with a fix, as measured at the start of the tick.

    Attributes:
        name: Tracking name; its id is "tracked/<name>".
        description: Its geometry and touches.
        pose: Last valid fix.
        tracked: Whether the latest sample was valid.
        time: ROS time of `pose`, or None.
    """

    name: str
    description: TrackedDescription
    pose: Pose
    tracked: bool
    time: float | None


@dataclass(eq=False)
class SceneSnapshot:
    """The whole world at the start of one tick. Readable from any thread; never change it.

    Attributes:
        tick: Index of the tick it was taken in.
        time: ROS time it was taken.
        bodies: Copies of every body, by id, except attached ones whose parent has no pose yet.
            ! Disabled bodies are included: check `Body.enabled` before treating one as an obstacle.
        world_poses: The world pose of every body in `bodies`, attachments resolved.
        robots: Every robot, by serial.
        tracked: Every tracked object that has had a fix, by name.
    """

    tick: int = -1
    time: float = 0.0
    bodies: dict[str, Body] = field(default_factory=dict)
    world_poses: dict[str, Pose] = field(default_factory=dict)
    robots: dict[str, RobotEntry] = field(default_factory=dict)
    tracked: dict[str, TrackedEntry] = field(default_factory=dict)

    def label(self, object_id: str) -> str:
        """Display text for any id: a body's or tracked object's label, else the id itself."""
        body = self.bodies.get(object_id)
        if body is not None and body.label:
            return body.label
        if object_id.startswith(f"{TRACKED}/"):
            entry = self.tracked.get(object_id.split("/", 1)[1])
            if entry is not None and entry.description.label:
                return entry.description.label
        return object_id
