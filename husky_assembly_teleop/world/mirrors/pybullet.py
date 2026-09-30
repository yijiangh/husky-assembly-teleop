"""
A private PyBullet world, filled from a `SceneSnapshot`: robots, scene bodies and
tracked objects with geometry.

    worker thread:  mirror.sync(snapshot) → collision checks in mirror.client_id

* `sync` only does what changed: it builds new or reshaped objects, removes gone
  ones and moves a body only when its pose changed. Robots are re-posed every sync.
! A planner that moves a body (not a robot) while searching must put it back
  itself: `sync` compares with the pose it last applied, not with PyBullet.
* Every call is a raw `p.*` call with `physicsClientId`, so other PyBullet worlds
  in the process are never touched.

! One thread only: create, sync and query a mirror on the same thread.
! `pp.CLIENT` (pybullet_planning) is one global for the whole process. Use `pp`
  only inside `mirror.active()`, and only one `pp` planner thread at a time.
! PyBullet reuses body ids after removal. Never keep a PyBullet id outside the
  mirror; translate results to our ids with `id_of`.
"""

from __future__ import annotations

from contextlib import contextmanager
from os import environ
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Iterable, Iterator

import pybullet as p
import pybullet_planning as pp

from ..geometry import BoxShape, CylinderShape
from ..scene import ROBOTS, Pose, SceneSnapshot, robot_id, tracked_id

if TYPE_CHECKING:
    from ...config import RobotConfig
    from ..geometry import Geometry, Shape


@dataclass(eq=False)
class _Built:
    """What the mirror built for one of our ids.

    Attributes:
        source: The `Geometry` or `RobotConfig` object it was built from.
        concave: Whether non-convex meshes were built concave (free bodies only).
        pose: The pose last applied.
        bodies: PyBullet body ids, one per collision shape (one for a robot).
        joints: Robots only: joint name -> joint index.
        links: Robots only: link name per link index + 1 (index -1, the base, first).
    """

    source: Geometry | RobotConfig
    concave: bool
    pose: Pose
    bodies: list[int]
    joints: dict[str, int] = field(default_factory=dict)
    links: list[str] = field(default_factory=list)


def _text(name: bytes) -> str:
    """Decode a PyBullet joint or link name (bytes) to str."""
    return name.decode("utf-8")


def _matches(entry: str, object_id: str) -> bool:
    """Whether a `touches` entry names an id: exactly, or "robots/<serial>" for any of its links."""
    if entry == object_id:
        return True
    return entry.startswith(f"{ROBOTS}/") and entry.count("/") == 1 and object_id.startswith(f"{entry}/")


class PyBulletMirror:
    """A private PyBullet DIRECT world that follows the scene snapshots given to `sync`."""

    def __init__(self, gui: bool = False) -> None:
        """Connect a new world, empty until the first `sync`.

        Args:
            gui: Open PyBullet's own window on it, to watch what a planner sees.
                ! Only one GUI world per process: a second raises `pybullet.error`.
                  The window belongs to the mirror's thread, like everything else here.

        Raises:
            RuntimeError: If a window is asked for but there is no display.
        """
        # ! Without an X display PyBullet exits the whole process instead of raising, so
        #   check first. Its window is X11 only (on Wayland, XWayland sets DISPLAY).
        if gui and not environ.get("DISPLAY"):
            raise RuntimeError("there is no display to open a PyBullet window on")
        self.client_id: int = p.connect(p.GUI if gui else p.DIRECT)
        #: Whether this world has a window.
        self.gui = gui
        # Our id -> what was built for it.
        self._built: dict[str, _Built] = {}
        # PyBullet body id -> our id.
        self._owner: dict[int, str] = {}
        # Our id -> its `touches`, from the last synced snapshot.
        self._touches: dict[str, tuple[str, ...]] = {}
        # (shape, built concave) -> collision shape id, shared between bodies.
        # ? Keyed by the shape itself, never `id(obj)`: Python reuses ids of freed
        #   objects. Meshes compare by identity, primitives by value (equal boxes share one).
        self._shapes: dict[tuple[Shape, bool], int] = {}

    # --- --- --- --- --- SYNC --- --- --- --- ---

    def sync(self, snapshot: SceneSnapshot) -> None:
        """Make this world match a snapshot: remove, (re)build and move only what changed.

        Args:
            snapshot: The world to copy. It is not modified.
        """
        # Our id -> (source, concave, pose) for everything that should exist.
        wanted: dict[str, tuple[Geometry | RobotConfig, bool, Pose]] = {}
        touches: dict[str, tuple[str, ...]] = {}
        for serial, entry in snapshot.robots.items():
            wanted[robot_id(serial)] = (entry.config, False, entry.base)
        for body_id, body in snapshot.bodies.items():
            # ! Concave only when free: Bullet can't collide two concave meshes,
            #   and an attached body may meet another concave body.
            concave = isinstance(body.placement, Pose) and any(not mesh.convex for mesh in body.geometry.collision)
            wanted[body_id] = (body.geometry, concave, snapshot.world_poses[body_id])
            touches[body_id] = body.touches
        for name, entry in snapshot.tracked.items():
            if entry.description.geometry is not None:
                wanted[tracked_id(name)] = (entry.description.geometry, False, entry.pose)
                touches[tracked_id(name)] = entry.description.touches
        self._touches = touches

        # * Remove everything gone or to be rebuilt first, so no freed PyBullet id is still in our tables.
        for object_id, built in list(self._built.items()):
            target = wanted.get(object_id)
            if target is None or target[0] is not built.source or target[1] != built.concave:
                self._remove(object_id)

        for object_id, (source, concave, pose) in wanted.items():
            built = self._built.get(object_id)
            if built is None:
                self._build(object_id, source, concave, pose)
            # ? Robots are re-posed every sync (cheap): planners move them around while searching.
            elif pose != built.pose or object_id.startswith(f"{ROBOTS}/"):
                for body in built.bodies:
                    p.resetBasePositionAndOrientation(body, pose.position, pose.orientation,
                                                      physicsClientId=self.client_id)
                built.pose = pose

        for serial, entry in snapshot.robots.items():
            built = self._built[robot_id(serial)]
            for name, value in entry.joints.items():
                # ? Joints the URDF lacks are skipped silently; Kinematics already warns about them.
                joint = built.joints.get(name)
                if joint is not None:
                    p.resetJointState(built.bodies[0], joint, value, physicsClientId=self.client_id)

        self._drop_unused_shapes()

    def _build(self, object_id: str, source: Geometry | RobotConfig, concave: bool, pose: Pose) -> None:
        """Build one object at a pose and record it.

        Args:
            object_id: Our id; "robots/<serial>" means `source` is a RobotConfig.
            source: What to build.
            concave: Whether to build non-convex meshes concave.
            pose: Its world pose.
        """
        if object_id.startswith(f"{ROBOTS}/"):
            body = p.loadURDF(str(source.urdf_file), useFixedBase=False, physicsClientId=self.client_id)
            p.resetBasePositionAndOrientation(body, pose.position, pose.orientation, physicsClientId=self.client_id)
            infos = [p.getJointInfo(body, i, physicsClientId=self.client_id)
                     for i in range(p.getNumJoints(body, physicsClientId=self.client_id))]
            built = _Built(source, concave, pose, [body],
                           joints={_text(info[1]): info[0] for info in infos},
                           links=[_text(p.getBodyInfo(body, physicsClientId=self.client_id)[0])]
                           + [_text(info[12]) for info in infos])
        else:
            bodies = [p.createMultiBody(baseMass=0,
                                        baseCollisionShapeIndex=self._shape(mesh, concave and not mesh.convex),
                                        basePosition=pose.position, baseOrientation=pose.orientation,
                                        physicsClientId=self.client_id)
                      for mesh in source.collision]
            built = _Built(source, concave, pose, bodies)
        self._built[object_id] = built
        for body in built.bodies:
            self._owner[body] = object_id

    def _remove(self, object_id: str) -> None:
        """Remove one object's PyBullet bodies and forget it.

        Args:
            object_id: Our id.
        """
        for body in self._built.pop(object_id).bodies:
            p.removeBody(body, physicsClientId=self.client_id)
            del self._owner[body]

    def _shape(self, shape: Shape, concave: bool) -> int:
        """The PyBullet collision shape of one of our shapes, created on first use and then shared.

        Args:
            shape: A box, a cylinder or triangles, in the body's frame.
            concave: For a mesh: build it as a concave triangle mesh; otherwise as the
                convex hull of its vertices. Primitives are always exact.

        Returns:
            int: The collision shape id.
        """
        key = (shape, concave)
        shape_id = self._shapes.get(key)
        if shape_id is None:
            client = self.client_id
            if isinstance(shape, BoxShape):
                shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[v / 2 for v in shape.size],
                                                  collisionFramePosition=shape.origin.position,
                                                  collisionFrameOrientation=shape.origin.orientation,
                                                  physicsClientId=client)
            elif isinstance(shape, CylinderShape):
                # ? PyBullet's cylinder stands along Z, centred, as ours does.
                shape_id = p.createCollisionShape(p.GEOM_CYLINDER, radius=shape.radius, height=shape.height,
                                                  collisionFramePosition=shape.origin.position,
                                                  collisionFrameOrientation=shape.origin.orientation,
                                                  physicsClientId=client)
            elif concave:
                shape_id = p.createCollisionShape(p.GEOM_MESH, vertices=shape.vertices, indices=shape.faces.flatten(),
                                                  flags=p.GEOM_FORCE_CONCAVE_TRIMESH, physicsClientId=client)
            else:
                # ! Vertices only. Given triangles, PyBullet builds a concave mesh even
                #   without the concave flag, and two of those never collide.
                shape_id = p.createCollisionShape(p.GEOM_MESH, vertices=shape.vertices, physicsClientId=client)
            self._shapes[key] = shape_id
        return shape_id

    def _drop_unused_shapes(self) -> None:
        """Forget cached shapes no body uses any more, so their meshes can be freed.

        ! PyBullet (3.2.x) refuses `removeCollisionShape` for a shape any body has
          ever used, even after that body is removed; it only prints a warning. So
          the PyBullet side of a dropped shape stays allocated until `close`.
        """
        used = {(mesh, built.concave and not mesh.convex)
                for object_id, built in self._built.items() if not object_id.startswith(f"{ROBOTS}/")
                for mesh in built.source.collision}
        self._shapes = {key: shape for key, shape in self._shapes.items() if key in used}

    # --- --- --- --- --- LOOKUP --- --- --- --- ---

    def robot(self, serial: str) -> int:
        """The PyBullet body id of a robot.

        Raises:
            KeyError: If no such robot was synced.
        """
        return self._built[robot_id(serial)].bodies[0]

    @property
    def robots(self) -> dict[str, int]:
        """Serial -> PyBullet body id of every robot, as a new dict."""
        return {object_id.split("/", 1)[1]: built.bodies[0]
                for object_id, built in self._built.items() if object_id.startswith(f"{ROBOTS}/")}

    def body_ids(self, object_id: str) -> list[int]:
        """The PyBullet bodies of a body, tracked object or "robots/<serial>".

        Raises:
            KeyError: If the id is not in this world.
        """
        return list(self._built[object_id].bodies)

    def id_of(self, body: int) -> str:
        """Our id of a PyBullet body: a body's or tracked object's id, or "robots/<serial>".

        Raises:
            KeyError: If the mirror did not build that body (e.g. a planner's own).
        """
        return self._owner[body]

    def obstacle_ids(self) -> list[str]:
        """Our ids of every body and tracked object that can collide, robots excluded.

        ? One without collision meshes has no PyBullet body, so it is left out.
        """
        return [object_id for object_id, built in self._built.items()
                if built.bodies and not object_id.startswith(f"{ROBOTS}/")]

    # --- --- --- --- --- COLLISIONS --- --- --- --- ---

    def allowed(self, a: str, b: str) -> bool:
        """Whether two ids may touch: either lists the other in its `touches`.

        An entry "robots/<serial>" allows every link "robots/<serial>/<link>";
        "robots/<serial>/<link>" allows only that link.

        Args:
            a: An id: body, tracked object, robot or robot link.
            b: Another id.

        Returns:
            bool: True if the pair may touch. Symmetric.
        """
        return (any(_matches(entry, b) for entry in self._touches.get(a, ()))
                or any(_matches(entry, a) for entry in self._touches.get(b, ())))

    def collisions(self, serial: str, margin: float = 0.0, candidates: Iterable[str] | None = None) -> list[str]:
        """Everything within `margin` of a robot, as it stands now, except allowed pairs.

        Args:
            serial: The robot to check.
            margin: Distance below which two objects count as colliding, metres.
            candidates: Only check these ids (e.g. after a cheap pre-check), or None for everything.

        Returns:
            list[str]: Our ids of the other robots, bodies and tracked objects hit; sorted, unique.
        """
        own = robot_id(serial)
        robot = self._built[own]
        hits: set[str] = set()
        for object_id in self._built if candidates is None else candidates:
            if object_id == own:
                continue
            other = self._built[object_id]
            for body in other.bodies:
                for contact in p.getClosestPoints(robot.bodies[0], body, margin, physicsClientId=self.client_id):
                    # ? contact[3] / contact[4]: link index on each side; -1 is the base, links[0].
                    link = f"{own}/{robot.links[contact[3] + 1]}"
                    target = f"{object_id}/{other.links[contact[4] + 1]}" if other.links else object_id
                    if not self.allowed(link, target):
                        hits.add(object_id)
                        break
                if object_id in hits:
                    break
        return sorted(hits)

    # --- --- --- --- --- CLIENT --- --- --- --- ---

    @contextmanager
    def active(self) -> Iterator[None]:
        """Point pybullet_planning's free functions at this world, and restore the previous one after.

        Yields:
            None: Inside the block, `pp` functions act on this world.
        """
        previous = pp.CLIENT
        pp.CLIENT = self.client_id
        try:
            yield
        finally:
            pp.CLIENT = previous

    @property
    def connected(self) -> bool:
        """bool: Whether the world still exists. False once closed, or once its window was closed."""
        return bool(p.isConnected(physicsClientId=self.client_id))

    def close(self) -> None:
        """Disconnect this world, unless already gone. The mirror can't be used afterwards."""
        if self.connected:
            p.disconnect(physicsClientId=self.client_id)
        self._built, self._owner, self._shapes = {}, {}, {}
