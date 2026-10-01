"""
The scene: every collision object we don't measure, plus one copy of the whole
world per tick for planners and the 3D view.

    tick:  pump ROS → kinematics.update → take_snapshot → plugins → draw
                                           └─ planners and the 3D view read this copy

* Plugins change bodies freely, in place or with `put`, during their step. The
  copy is taken before any plugin runs, so it always holds one ROS pump's
  measurements and every plugin's complete writes of the previous tick.

! Main thread only: the scene, `Body` objects in it and `take_snapshot`.
  A snapshot can be read from any thread; treat it as read-only.
! To change a body's shape, assign a new `Geometry`. Never change one in place:
  mirrors rebuild only when the geometry object is a different one.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Callable, Iterable

# Pose, compose and ids live in design_io, shared with design files; re-exported here.
from ..design_io.pose import ID_PATTERN, Pose, check_id, compose  # noqa: F401

if TYPE_CHECKING:
    from ..config import RobotConfig
    from .geometry import Geometry
    from .kinematics import Kinematics
    from .measured import WorldState

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
        touches: Ids allowed to touch it: bodies, "robots/<serial>" (the whole
            robot), "robots/<serial>/<link>" (one link), "tracked/<name>".
            Symmetric: it doesn't matter which side lists the other.
        label: Display text, e.g. in collision messages. Empty: use the id.
        color: (r, g, b, a) from 0 to 1 for the 3D view, or None for grey.
    """

    id: str
    geometry: Geometry
    placement: Pose | Attachment
    touches: tuple[str, ...] = ()
    label: str = ""
    color: tuple[float, float, float, float] | None = None

    def copy(self) -> Body:
        """A copy that later changes to this body don't reach. Shares the geometry.

        ? Built by hand: about four times faster than `copy.copy` for a dataclass.
        """
        return Body(self.id, self.geometry, self.placement, self.touches, self.label, self.color)


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
        config: The robot's configuration. Its `urdf_file` is the stitched URDF.
            ! A different config object means a different robot model: mirrors reload it.
        base: Last valid mocap pose, or the configured default before any.
        base_tracked: Whether the latest mocap sample was valid.
        joints: Last measured value per actuated joint.
        unmeasured: Joints never measured; their value in `joints` is 0.
        base_time: ROS time of the mocap fix behind `base`, or None.
        joints_time: ROS time of the oldest arm's latest joint state, or None
            if an arm has never reported.
            ! Base and arms are measured by different sensors at different
              instants. While the robot moves, `base_time - joints_time` is the error.
    """

    config: RobotConfig
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
        bodies: Copies of every body, by id. An attached body whose parent has no
            pose (a tracked object never seen) is left out.
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


# --- --- --- --- --- THE STORE --- --- --- --- ---

class Scene:
    """Every body plugins put in, the tracked objects' descriptions, and the latest copy. One instance, core.

    ! Main thread only.
    """

    def __init__(self):
        """Start empty, with an empty snapshot."""
        #: Live bodies by id. Plugins change them during their step.
        self.bodies: dict[str, Body] = {}
        #: Tracked objects' descriptions, by tracking name. Written by track_object.
        self.tracked: dict[str, TrackedDescription] = {}
        self._snapshot = SceneSnapshot()

    def put(self, body: Body) -> None:
        """Add a body, or replace the one with the same id.

        Raises:
            ValueError: If the id or the attachment's parent is invalid.
        """
        check_id(body.id)
        if body.id.split("/", 1)[0] in (ROBOTS, TRACKED):
            raise ValueError(f"{body.id!r}: ids under '{ROBOTS}/' and '{TRACKED}/' belong to the core")
        if isinstance(body.placement, Attachment) and \
                body.placement.parent.split("/", 1)[0] not in (ROBOTS, TRACKED):
            raise ValueError(f"{body.id!r}: attached to {body.placement.parent!r}, "
                             f"which is neither 'robots/<serial>' nor 'tracked/<name>'")
        self.bodies[body.id] = body

    def remove(self, body_id: str) -> None:
        """Remove one body. Unknown ids are ignored."""
        self.bodies.pop(body_id, None)

    def remove_prefix(self, prefix: str) -> None:
        """Remove every body whose id starts with `prefix`, e.g. "cell/" when the cell plugin closes."""
        for body_id in [body_id for body_id in self.bodies if body_id.startswith(prefix)]:
            del self.bodies[body_id]

    @property
    def snapshot(self) -> SceneSnapshot:
        """SceneSnapshot: The copy taken at the start of this tick."""
        return self._snapshot

    def take_snapshot(self, world: WorldState, kinematics: Kinematics, tick: int, time: float) -> SceneSnapshot:
        """Copy the whole world. The monitor calls this once per tick, after kinematics.update.

        Args:
            world: Measured state, for tracked objects and timestamps.
            kinematics: Robot bases, joints and link poses, updated this tick.
            tick: The tick's index.
            time: ROS time now.

        Returns:
            SceneSnapshot: The new copy, also kept as `snapshot`.
        """
        robots = {}
        for serial, robot in world.robots.items():
            arm_times = [arm.state.last_update_time for arm in robot.arms.values()]
            robots[serial] = RobotEntry(
                config=robot.config, base=kinematics.base_pose(serial), base_tracked=robot.base.state.tracked,
                joints=dict(kinematics.joints(serial)), unmeasured=kinematics.unmeasured(serial),
                base_time=robot.base.state.last_fix_time,
                joints_time=None if None in arm_times else min(arm_times, default=None))

        tracked = {}
        for name, obj in world.tracked_objects.items():
            if obj.position is None:
                continue  # never seen: no pose to give it
            tracked[name] = TrackedEntry(name=name, description=self.tracked.get(name, TrackedDescription()),
                                         pose=Pose.from_arrays(obj.position, obj.orientation),
                                         tracked=obj.tracked, time=obj.last_fix_time)

        bodies, world_poses = {}, {}
        for body_id, body in self.bodies.items():
            pose = self._world_pose(body.placement, kinematics, tracked)
            if pose is not None:
                bodies[body_id] = body.copy()
                world_poses[body_id] = pose

        self._snapshot = SceneSnapshot(tick=tick, time=time, bodies=bodies, world_poses=world_poses,
                                       robots=robots, tracked=tracked)
        return self._snapshot

    @staticmethod
    def _world_pose(placement: Pose | Attachment, kinematics: Kinematics,
                    tracked: dict[str, TrackedEntry]) -> Pose | None:
        """Resolve a placement to a world pose, or None if its parent has no pose."""
        if isinstance(placement, Pose):
            return placement
        kind, name = placement.parent.split("/", 1)
        if kind == ROBOTS:
            parent = kinematics.base_pose(name) if placement.link is None else kinematics.link_pose(name, placement.link)
        else:
            entry = tracked.get(name)
            if entry is None:
                return None
            parent = entry.pose
        return compose(parent, placement.grasp)


class PluginScene:
    """A plugin's handle on the scene (`ctx.scene`): it may add and remove only ids under its own name.

    ! Bodies in `bodies` are the live ones: changing a field changes the scene.
      Change only your own; nothing stops you from changing another plugin's.
    """

    def __init__(self, scene: Scene, owner: str, log_warn: Callable[[str], None]):
        """Wrap the scene for one plugin.

        Args:
            scene: The one scene.
            owner: The plugin's name; its ids must start with "<owner>/".
            log_warn: Reports bodies that are drawn but never collide.
        """
        self._scene = scene
        self._prefix = f"{owner}/"
        self._log_warn = log_warn
        # Ids already warned about, so each is reported once.
        self._warned: set[str] = set()

    @property
    def bodies(self) -> dict[str, Body]:
        """dict[str, Body]: Every live body, by id. Don't add or remove through this dict."""
        return self._scene.bodies

    @property
    def snapshot(self) -> SceneSnapshot:
        """SceneSnapshot: The copy taken at the start of this tick. Hand this to worker threads."""
        return self._scene.snapshot

    def put(self, body: Body) -> None:
        """Add a body, or replace yours with the same id.

        Raises:
            ValueError: If the id doesn't start with "<plugin name>/", or is invalid.
        """
        self._check_owner(body.id)
        self._scene.put(body)
        if not body.geometry.collision and body.id not in self._warned:
            self._warned.add(body.id)
            self._log_warn(f"{body.id!r} has no collision meshes: it is drawn but never collides")

    def put_many(self, bodies: Iterable[Body]) -> None:
        """`put` each body."""
        for body in bodies:
            self.put(body)

    def remove(self, body_id: str) -> None:
        """Remove one of your bodies. Unknown ids are ignored.

        Raises:
            ValueError: If the id isn't yours.
        """
        self._check_owner(body_id)
        self._scene.remove(body_id)

    def remove_prefix(self, prefix: str) -> None:
        """Remove every one of your bodies whose id starts with `prefix`.

        Raises:
            ValueError: If `prefix` doesn't start with "<plugin name>/".
        """
        self._check_owner(prefix)
        self._scene.remove_prefix(prefix)

    def _check_owner(self, body_id: str) -> None:
        """Refuse an id outside this plugin's own prefix."""
        if not body_id.startswith(self._prefix):
            raise ValueError(f"{body_id!r} must start with {self._prefix!r}: plugins own only their own ids")
