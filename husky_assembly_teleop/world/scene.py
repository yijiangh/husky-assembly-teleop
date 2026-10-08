"""
The monitor's live scene (collision objects we don't measure), and its per-tick snapshot for planners and the 3D view.

The data types (`Body`, `RobotObject`, `SceneSnapshot`) are the core's (`bar_assembly_core.scene`). The snapshot is
taken before plugins run, so it holds one ROS pump's measurements and every plugin's complete writes of the previous
tick. Each measured robot is a `RobotObject` "robots/<serial>"; each tracked object with a fix, a `Body`
"tracked/<name>".

- ! Main thread only: the scene, its `Body` objects and `take_snapshot`. A snapshot can be read from
  any thread; treat it as read-only.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Iterable

from bar_assembly_core.design_io.geometry import Geometry
from bar_assembly_core.design_io.pose import Pose, check_id
from bar_assembly_core.robot import RobotObject
from bar_assembly_core.scene import ROBOTS, TRACKED, Attachment, Body, SceneSnapshot, robot_id, tracked_id, world_poses

if TYPE_CHECKING:
    from .kinematics import Kinematics
    from .measured import WorldState

#: The geometry of a tracked object that is a frame only: drawn, never collides.
NO_GEOMETRY = Geometry((), ())


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


# --- --- --- --- --- THE STORE --- --- --- --- ---

class Scene:
    """Every body plugins put in, the tracked objects' descriptions, and the latest snapshot.

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
        """Copy the whole world; called once per tick, after kinematics.update.

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
            model = robot.config.model
            arm_times = [arm.state.last_update_time for arm in robot.arms.values()]
            # ! Only movable joints count as unmeasured: wheels and gripper fingers are never measured.
            robots[robot_id(serial)] = RobotObject(
                id=robot_id(serial), model=model, base=kinematics.base_pose(serial),
                joints=dict(kinematics.joints(serial)), base_tracked=robot.base.state.tracked,
                unmeasured=kinematics.unmeasured(serial).intersection(model.movable_joints),
                base_time=robot.base.state.last_fix_time,
                joints_time=None if None in arm_times else min(arm_times, default=None))

        live = dict(self.bodies)
        for name, obj in world.tracked_objects.items():
            if obj.position is None:
                continue  # never seen: no pose to give it
            description = self.tracked.get(name, TrackedDescription())
            live[tracked_id(name)] = Body(tracked_id(name), description.geometry or NO_GEOMETRY,
                                          Pose.from_arrays(obj.position, obj.orientation), description.touches,
                                          description.label)

        def link_pose(robot: RobotObject, link: str) -> Pose:
            """A measured robot's link pose, from this tick's kinematics (keyed by serial)."""
            return kinematics.link_pose(robot.id.split("/", 1)[1], link)

        poses = world_poses(live, robots, link_pose)
        bodies = {body_id: body.copy() for body_id, body in live.items() if body_id in poses}
        self._snapshot = SceneSnapshot(tick=tick, time=time, bodies=bodies, world_poses=poses, robots=robots)
        return self._snapshot


class PluginScene:
    """A plugin's handle on the scene (`ctx.scene`): it may add and remove only ids under its own name.

    ! `bodies` holds the live ones: change only your own; nothing stops you from changing another plugin's.
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
