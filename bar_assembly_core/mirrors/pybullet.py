"""
A private PyBullet world filled from a `Scene`: robots with their tools, and the bodies with geometry.

`sync` only rebuilds or moves what changed; robots are re-posed every sync. A robot is loaded from a URDF the
mirror writes once per `RobotModel`, each mounted tool a fixed link at its flange. `set_gui(True)` shows the world
in PyBullet's own window (one per process) for debugging; it shows collision shapes only, as nothing visual
is loaded.

- ! One thread only: create, sync and query a mirror on the same thread.
- ! A planner that moves a body (not a robot) must put it back: `sync` compares with the pose it last applied.
- ! Use `pp` (pybullet_planning) only inside `mirror.active()`, one `pp` thread at a time: its client is global.
- ! Never keep a PyBullet id outside the mirror: ids are reused after removal and change on `set_gui`;
  translate with `id_of`.
"""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import TYPE_CHECKING, Iterable, Iterator, List, Union
from xml.etree.ElementTree import ElementTree, SubElement, fromstring

import pybullet as p
import trimesh
from scipy.spatial.transform import Rotation

from ..geometry import BoxShape, CylinderShape
from ..geometry import Pose
from ..urdf import resolved_urdf_text
from ..robot import RobotModel
from ..ids import ROBOTS, TRACKED
from ..scene import Scene, same_source
from . import HIDDEN_POSITION, check_display
from .pp_client import pp_client

if TYPE_CHECKING:
    from ..geometry import Geometry, Shape


@dataclass(eq=False)
class _Built:
    """What the mirror built for one of our ids.

    Attributes:
        source: The `Geometry` or `RobotModel` object it was built from.
        concave: Whether non-convex meshes were built concave (free bodies only).
        pose: The pose last applied.
        bodies: PyBullet body ids, one per collision shape (one for a robot).
        joints: Robots only: joint name -> joint index.
        links: Robots only: link name per link index + 1 (index -1, the base, first).
        tools: Robots only: link name -> id of the mounted tool it is.
    """

    source: Union[Geometry, RobotModel]
    concave: bool
    pose: Pose
    bodies: List[int]
    joints: dict = field(default_factory=dict)
    links: List[str] = field(default_factory=list)
    tools: dict = field(default_factory=dict)


def _text(name: bytes) -> str:
    """Decode a PyBullet joint or link name (bytes) to str."""
    return name.decode("utf-8")


def _tool_link(flange: str) -> str:
    """The name of the link a mounted tool becomes in the URDF the mirror writes."""
    return f"{flange}_tool"


def _names(object_id: str, built: _Built, link: int) -> List[str]:
    """The ids `touches` may name one side of a contact by: the body, or the robot link and its tool's id.

    Args:
        object_id: The body's or robot's id.
        built: What was built for it.
        link: PyBullet link index of the contact; -1 is the base.
    """
    if not built.links:
        return [object_id]
    name = built.links[link + 1]
    tool = built.tools.get(name)
    return [f"{object_id}/{name}"] + ([tool] if tool is not None else [])


def _matches(entry: str, object_id: str) -> bool:
    """Whether a `touches` entry names an id: exactly, or "robots/<name>" for any of its links."""
    if entry == object_id:
        return True
    return entry.startswith(f"{ROBOTS}/") and entry.count("/") == 1 and object_id.startswith(f"{entry}/")


class PyBulletMirror:
    """A private PyBullet world that follows the scene snapshots given to `sync`."""

    def __init__(self) -> None:
        """Connect a new world without a window, empty until the first `sync`."""
        self.client_id: int | None = None
        self._connect(gui=False)

    def _connect(self, gui: bool) -> None:
        """Replace the world with a new, empty one, with PyBullet's own window if asked for.

        Raises:
            RuntimeError: If a window is asked for without an X display.
            pybullet.error: If another PyBullet window is open in this process. The old world stays.
        """
        if gui:
            check_display()
        client_id = p.connect(p.GUI if gui else p.DIRECT)
        if self.client_id is not None:
            self.close()
        self.client_id = client_id
        #: Whether PyBullet's own window shows this world.
        self.gui = gui
        # Our id -> what was built for it.
        self._built: dict[str, _Built] = {}
        # PyBullet body id -> our id.
        self._owner: dict[int, str] = {}
        # Our id -> its `touches`, from the last synced snapshot.
        self._touches: dict[str, tuple[str, ...]] = {}
        # Ids of disabled bodies and robots in the last synced snapshot: built, but never collide.
        self._disabled: set[str] = set()
        # Robot id -> why it can't be planned for (`RobotObject.acting_problems`), from the last synced snapshot.
        self._problems: dict[str, list[str]] = {}
        # (shape, built concave) -> collision shape id, shared between bodies.
        # ? Keyed by the shape, not `id(obj)` (reused after free); equal primitives share one.
        self._shapes: dict[tuple[Shape, bool], int] = {}
        # RobotModel -> the URDF written for it with its tools, in `_files` (made on first use).
        self._urdfs: dict[RobotModel, Path] = {}
        self._files: TemporaryDirectory | None = None

    def set_gui(self, gui: bool) -> None:
        """Open or close PyBullet's own window, for debugging. The world is empty until the next `sync`.

        - ! Closing the window by hand ends the world; `set_gui(False)` brings it back.

        Raises:
            RuntimeError: If a window is asked for without an X display.
            pybullet.error: If another PyBullet window is open in this process.
        """
        if gui != self.gui:
            self._connect(gui)

    # --- --- --- --- --- SYNC --- --- --- --- ---

    def sync(self, snapshot: Scene) -> None:
        """Make this world match a snapshot: remove, (re)build and move only what changed.

        Args:
            snapshot: The world to copy. It is not modified.
        """
        # Our id -> (source, concave, pose) for everything that should exist.
        wanted: dict[str, tuple[Geometry | RobotModel, bool, Pose]] = {}
        touches: dict[str, tuple[str, ...]] = {}
        for object_id, robot in snapshot.robots.items():
            wanted[object_id] = (robot.model, False, robot.base)
        for body_id, body in snapshot.bodies.items():
            # ! Concave only when free and unmeasured: Bullet can't collide two concave meshes, and those others move.
            concave = (isinstance(body.placement, Pose) and not body_id.startswith(f"{TRACKED}/")
                       and any(not mesh.convex for mesh in body.geometry.collision))
            wanted[body_id] = (body.geometry, concave, snapshot.world_poses[body_id])
            touches[body_id] = body.touches
        self._disabled = ({body_id for body_id, body in snapshot.bodies.items() if not body.enabled}
                          | {object_id for object_id, robot in snapshot.robots.items() if not robot.enabled})
        self._problems = {object_id: robot.acting_problems() for object_id, robot in snapshot.robots.items()}
        self._touches = touches

        # * Remove everything gone or to be rebuilt first, so no freed PyBullet id is still in our tables.
        for object_id, built in list(self._built.items()):
            target = wanted.get(object_id)
            if target is None or not same_source(target[0], built.source) or target[1] != built.concave:
                self._remove(object_id)

        hidden = Pose(HIDDEN_POSITION)
        for object_id, (source, concave, pose) in wanted.items():
            # * A disabled object leaves the world: queries skip it too, but the window and `pp` users would see it.
            pose = hidden if object_id in self._disabled else pose
            built = self._built.get(object_id)
            if built is None:
                self._build(object_id, source, concave, pose)
            # ? Robots are re-posed every sync (cheap): planners move them around while searching.
            elif pose != built.pose or object_id in snapshot.robots:
                for body in built.bodies:
                    p.resetBasePositionAndOrientation(body, pose.position, pose.orientation,
                                                      physicsClientId=self.client_id)
                built.pose = pose

        for object_id, robot in snapshot.robots.items():
            built = self._built[object_id]
            for name, value in robot.joints.items():
                # ? Joints the URDF lacks are skipped silently; whoever measures them warns.
                joint = built.joints.get(name)
                if joint is not None:
                    p.resetJointState(built.bodies[0], joint, value, physicsClientId=self.client_id)

        self._drop_unused_shapes()

    def _build(self, object_id: str, source: Geometry | RobotModel, concave: bool, pose: Pose) -> None:
        """Build one object at a pose and record it.

        Args:
            object_id: Our id.
            source: What to build: a robot's model, or a body's geometry.
            concave: Whether to build non-convex meshes concave.
            pose: Its world pose.
        """
        if isinstance(source, RobotModel):
            # * Collision shapes only: the visual meshes are large and slow to load (~0.6 s per robot), and unused.
            body = p.loadURDF(str(self._robot_urdf(source)), useFixedBase=False, flags=p.URDF_IGNORE_VISUAL_SHAPES,
                              physicsClientId=self.client_id)
            p.resetBasePositionAndOrientation(body, pose.position, pose.orientation, physicsClientId=self.client_id)
            infos = [p.getJointInfo(body, i, physicsClientId=self.client_id)
                     for i in range(p.getNumJoints(body, physicsClientId=self.client_id))]
            built = _Built(source, concave, pose, [body],
                           joints={_text(info[1]): info[0] for info in infos},
                           links=[_text(p.getBodyInfo(body, physicsClientId=self.client_id)[0])]
                           + [_text(info[12]) for info in infos],
                           tools={_tool_link(flange): tool.id for flange, tool in source.tools.items()})
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
            concave: For a mesh, build a concave triangle mesh instead of the convex hull. Ignored for primitives.

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
                # ! Vertices only: given triangles, PyBullet builds a concave mesh, and two of those never collide.
                shape_id = p.createCollisionShape(p.GEOM_MESH, vertices=shape.vertices, physicsClientId=client)
            self._shapes[key] = shape_id
        return shape_id

    def _drop_unused_shapes(self) -> None:
        """Forget cached shapes no body uses any more, so their meshes can be freed.

        ? PyBullet (3.2.x) won't remove a shape any body ever used, so its PyBullet side stays until `close`.
        """
        used = {(mesh, built.concave and not mesh.convex)
                for built in self._built.values() if not isinstance(built.source, RobotModel)
                for mesh in built.source.collision}
        self._shapes = {key: shape for key, shape in self._shapes.items() if key in used}

    def _robot_urdf(self, model: RobotModel) -> Path:
        """The URDF of a robot model with each tool a fixed link at its flange; written once per model.

        Args:
            model: The robot's model.

        Returns:
            Path: The URDF, in this mirror's own folder, with absolute mesh paths.
        """
        path = self._urdfs.get(model)
        if path is not None:
            return path
        if self._files is None:
            self._files = TemporaryDirectory(prefix="pybullet_mirror_")
        folder = Path(self._files.name)
        stem = f"{model.name}_{len(self._urdfs)}"
        root = fromstring(resolved_urdf_text(model.urdf))
        for flange, tool in model.tools.items():
            link = SubElement(root, "link", name=_tool_link(flange))
            for index, shape in enumerate(tool.geometry.collision):
                collision = SubElement(link, "collision")
                geometry = SubElement(collision, "geometry")
                if isinstance(shape, BoxShape):
                    SubElement(geometry, "box", size=" ".join(str(v) for v in shape.size))
                elif isinstance(shape, CylinderShape):
                    SubElement(geometry, "cylinder", radius=str(shape.radius), length=str(shape.height))
                else:
                    mesh_file = folder / f"{stem}_{flange}_{index}.obj"
                    trimesh.Trimesh(shape.vertices, shape.faces, process=False).export(str(mesh_file))
                    SubElement(geometry, "mesh", filename=str(mesh_file))
                origin = getattr(shape, "origin", Pose())
                SubElement(collision, "origin", xyz=" ".join(str(v) for v in origin.position),
                           rpy=" ".join(str(v) for v in Rotation.from_quat(origin.orientation).as_euler("xyz")))
            joint = SubElement(root, "joint", name=f"{_tool_link(flange)}_joint", type="fixed")
            SubElement(joint, "parent", link=flange)
            SubElement(joint, "child", link=_tool_link(flange))
        path = self._urdfs[model] = folder / f"{stem}.urdf"
        ElementTree(root).write(str(path), encoding="utf-8", xml_declaration=True)
        return path

    # --- --- --- --- --- LOOKUP --- --- --- --- ---

    def robot(self, object_id: str) -> int:
        """The PyBullet body id of a robot, by robot id.

        Raises:
            KeyError: If no such robot was synced.
        """
        return self._built[object_id].bodies[0]

    @property
    def robots(self) -> dict[str, int]:
        """Robot id -> PyBullet body id of every robot, as a new dict."""
        return {object_id: built.bodies[0] for object_id, built in self._built.items()
                if isinstance(built.source, RobotModel)}

    def body_ids(self, object_id: str) -> list[int]:
        """The PyBullet bodies of a body or robot.

        Raises:
            KeyError: If the id is not in this world.
        """
        return list(self._built[object_id].bodies)

    def id_of(self, body: int) -> str:
        """Our id of a PyBullet body: a body's or a robot's.

        Raises:
            KeyError: If the mirror did not build that body (e.g. a planner's own).
        """
        return self._owner[body]

    def obstacle_ids(self) -> list[str]:
        """Our ids of every enabled body that can collide, robots excluded.

        ? One without collision meshes has no PyBullet body, so it is left out.
        """
        return [object_id for object_id, built in self._built.items()
                if built.bodies and not isinstance(built.source, RobotModel) and object_id not in self._disabled]

    # --- --- --- --- --- COLLISIONS --- --- --- --- ---

    def allowed(self, a: str, b: str) -> bool:
        """Whether two ids may touch: either lists the other in its `touches` ("robots/<name>" covers all links).

        Args:
            a: An id: body, robot, robot link or mounted tool.
            b: Another id.

        Returns:
            bool: True if the pair may touch. Symmetric.
        """
        return (any(_matches(entry, b) for entry in self._touches.get(a, ()))
                or any(_matches(entry, a) for entry in self._touches.get(b, ())))

    def collisions(self, object_id: str, margin: float = 0.0, candidates: Iterable[str] | None = None) -> list[str]:
        """Everything within `margin` of a robot, as it stands now, except allowed pairs.

        Args:
            object_id: The robot to check, by id.
            margin: Distance below which two objects count as colliding, metres.
            candidates: Only check these ids (e.g. after a cheap pre-check), or None for everything.

        Returns:
            list[str]: Our ids of the other robots and bodies hit; sorted, unique.

        Raises:
            ValueError: If the robot can't be planned for (`RobotObject.acting_problems`).
        """
        if self._problems.get(object_id):
            raise ValueError(f"cannot check {object_id}: {'; '.join(self._problems[object_id])}")
        robot = self._built[object_id]
        hits: set[str] = set()
        for other_id in self._built if candidates is None else candidates:
            if other_id == object_id or other_id in self._disabled:
                continue
            other = self._built[other_id]
            for body in other.bodies:
                for contact in p.getClosestPoints(robot.bodies[0], body, margin, physicsClientId=self.client_id):
                    # ? contact[3] / contact[4]: link index on each side; -1 is the base, links[0].
                    own = _names(object_id, robot, contact[3])
                    target = _names(other_id, other, contact[4])
                    if not any(self.allowed(a, b) for a in own for b in target):
                        hits.add(other_id)
                        break
                if other_id in hits:
                    break
        return sorted(hits)

    # --- --- --- --- --- CLIENT --- --- --- --- ---

    @contextmanager
    def active(self) -> Iterator[None]:
        """Point pybullet_planning's free functions at this world, and restore the previous one after (`pp_client`).

        Yields:
            None: Inside the block, `pp` functions act on this world.
        """
        with pp_client(self.client_id):
            yield

    @property
    def connected(self) -> bool:
        """bool: Whether the world still exists. False once closed, or once its window was closed."""
        return bool(p.isConnected(physicsClientId=self.client_id))

    def close(self) -> None:
        """Disconnect this world and delete its files, unless already gone. The mirror can't be used afterwards."""
        if self.connected:
            p.disconnect(physicsClientId=self.client_id)
        if self._files is not None:
            self._files.cleanup()
        self._built, self._owner, self._shapes, self._urdfs, self._files = {}, {}, {}, {}, None
