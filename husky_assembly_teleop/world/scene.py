"""
The monitor's live scene, and the copy of it each tick that planners and the 3D view read.

Both are `Scene`s (`bar_assembly_core.scene`): plugins edit the live one through `PluginScene`; `take_snapshot`
copies it.

The copy is taken before plugins run, so it holds one ROS pump's measurements and every plugin's complete writes of
the previous tick. Each measured robot is a `RobotObject` "robots/<serial>"; each tracked object a `Body`
"tracked/<name>", disabled until its first mocap fix.

- ! Main thread only: the live scene, its `Body` objects and `take_snapshot`. A copy can be read from any thread;
  treat it as read-only.
- ! Never hand the live scene to a plugin or a thread: plugins get a `PluginScene`, threads its `snapshot`.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Callable, Iterable

from bar_assembly_core.geometry import Geometry, Pose
from bar_assembly_core.ids import ROBOTS, TRACKED, robot_id, tracked_id
from bar_assembly_core.robot import RobotObject
from bar_assembly_core.scene import Attachment, Body, Scene, world_poses

if TYPE_CHECKING:
    from .kinematics import Kinematics
    from .measured import WorldState

#: The geometry of a tracked object that is a frame only: drawn, never collides.
NO_GEOMETRY = Geometry((), ())


def tracked_body(name: str, geometry: Geometry | None = None, touches: tuple[str, ...] = (), label: str = "") -> Body:
    """The body of a tracked object "tracked/<name>", disabled until `take_snapshot` gives it a mocap fix.

    Args:
        name: Tracking name.
        geometry: Its shape, or None for a frame only that is not an obstacle (e.g. the mocap probe).
        touches: Ids allowed to touch it, as for `Body.touches`.
        label: Display text. Empty: use the id.
    """
    return Body(tracked_id(name), geometry or NO_GEOMETRY, Pose(), tuple(touches), label, enabled=False)


def take_snapshot(scene: Scene, world: WorldState, kinematics: Kinematics, tick: int, time: float) -> Scene:
    """Bring the live scene up to this tick's measurements, and copy it; called once per tick, after kinematics.

    Sets every robot from the measurements, moves each tracked object with a fix to it, and resolves world poses.

    Args:
        scene: The live scene.
        world: Measured state, for robots, tracked objects and timestamps.
        kinematics: Robot bases, joints and link poses, updated this tick.
        tick: The tick's index.
        time: ROS time now.

    Returns:
        Scene: The copy for this tick. Bodies whose parent has no pose are left out of it.
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
    scene.robots = robots

    for name, obj in world.tracked_objects.items():
        body = scene.bodies.get(tracked_id(name))
        if body is not None and obj.position is not None:
            body.placement, body.enabled = Pose.from_arrays(obj.position, obj.orientation), True

    def link_pose(robot: RobotObject, link: str) -> Pose:
        """A measured robot's link pose, from this tick's kinematics (keyed by serial)."""
        return kinematics.link_pose(robot.id.split("/", 1)[1], link)

    scene.tick, scene.time = tick, time
    scene.world_poses = world_poses(scene.bodies, robots, link_pose)
    return Scene(tick, time, {body_id: body.copy() for body_id, body in scene.bodies.items()
                              if body_id in scene.world_poses},
                 dict(scene.world_poses), {key: robot.copy() for key, robot in robots.items()})


class PluginScene:
    """A plugin's handle on the scene (`ctx.scene`): it may add and remove only ids under its own name.

    ! `bodies` holds the live ones: change only your own; nothing stops you from changing another plugin's.
    """

    def __init__(self, scene: Scene, owner: str, log_warn: Callable[[str], None], snapshot: Callable[[], Scene]):
        """Wrap the live scene for one plugin.

        Args:
            scene: The live scene.
            owner: The plugin's name; its ids must start with "<owner>/".
            log_warn: Reports bodies that are drawn but never collide.
            snapshot: Returns this tick's copy.
        """
        self._scene = scene
        self._snapshot = snapshot
        self._prefix = f"{owner}/"
        self._log_warn = log_warn
        # Ids already warned about, so each is reported once.
        self._warned: set[str] = set()

    @property
    def bodies(self) -> dict[str, Body]:
        """dict[str, Body]: Every live body, by id. Don't add or remove through this dict."""
        return self._scene.bodies

    @property
    def snapshot(self) -> Scene:
        """Scene: The copy taken at the start of this tick. Hand this to worker threads."""
        return self._snapshot()

    def put(self, body: Body) -> None:
        """Add a body, or replace yours with the same id.

        Raises:
            ValueError: If the id doesn't start with "<plugin name>/", or is invalid; or the body is attached to
                something other than a robot or a tracked object.
        """
        self._check_owner(body.id)
        if body.id.split("/", 1)[0] in (ROBOTS, TRACKED):
            raise ValueError(f"{body.id!r}: ids under '{ROBOTS}/' and '{TRACKED}/' belong to the monitor")
        if isinstance(body.placement, Attachment) and \
                body.placement.parent.split("/", 1)[0] not in (ROBOTS, TRACKED):
            raise ValueError(f"{body.id!r}: attached to {body.placement.parent!r}, "
                             f"which is neither 'robots/<serial>' nor 'tracked/<name>'")
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
