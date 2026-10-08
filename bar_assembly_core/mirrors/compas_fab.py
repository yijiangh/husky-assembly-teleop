"""
A compas_fab planning world for one acting robot, filled from a `SceneSnapshot`.

The acting robot is the cell's robot, each of its mounted tools a `ToolModel` attached to the SRDF group ending at
its flange (as the Rhino plugin and tamp build cells). Every other robot is one `ToolModel` keyed by its robot id,
parked far away while absent; bodies are `RigidBody`s keyed by our id, attached when held by the acting robot.
`sync` rebuilds the cell (~2 s) only when a model, body id or geometry object changed. `collisions` is compas_fab's
own check; `search_check` is a fast copy of it for searches. `set_gui(True)` shows the world in PyBullet's
own window (one per process) for debugging.

- ! One thread only: create, sync and query a mirror on the same thread.
- ! Other robots' tool/body pairs already touching at `sync` are allowed for that snapshot (`static_contacts`).
- ! compas_fab builds each collision mesh as its convex hull.
- ? Robots are loaded without their visual shapes (`load_model(visual=False)`), and bodies with their collision
  shapes as visuals: loading is faster, and PyBullet's window shows exactly what is checked.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from itertools import combinations
from typing import TYPE_CHECKING, Callable, Dict, Mapping, Optional, Sequence, Tuple, Union

import pybullet as p
from compas.geometry import Frame
from compas_fab.backends import CollisionCheckError, PyBulletClient, PyBulletPlanner
from compas_fab.backends.pybullet.conversions import pose_from_frame
from compas_fab.robots import RigidBody, RigidBodyState, RobotCell, RobotCellState, RobotSemantics, ToolState
from compas_robots import Configuration, ToolModel

from ..design_io.pose import Pose
from ..robot import RobotModel
from ..scene import ROBOTS, Attachment, SceneSnapshot, same_source
from . import check_display
from .compas_convert import (PARKED_POSITION, filled, frame_from_pose, load_model, planning_group, rigid_body,
                             robot_as_tool, subtree, tool_model)

if TYPE_CHECKING:
    from compas_robots import RobotModel as CompasRobotModel
    from compas_robots.model import Link

    from ..design_io.geometry import Geometry


@dataclass(eq=False)
class _Acting:
    """The acting robot as compas_fab objects, built once per `RobotModel`.

    Attributes:
        source: The model they were built from.
        model: Its URDF, collision shapes only.
        semantics: Its SRDF.
        tools: Tool id -> its compas_fab tool, one per mounted tool.
        flanges: Tool id -> the flange it is mounted on.
    """

    source: RobotModel
    model: CompasRobotModel
    semantics: RobotSemantics
    tools: Dict[str, ToolModel]
    flanges: Dict[str, str]


class CompasFabMirror:
    """A compas_fab PyBullet world planning for one robot, following the snapshots given to `sync`."""

    def __init__(self, robot_id: str, log: Callable[[str], None] | None = None) -> None:
        """Connect an empty world without a window; the first `sync` loads everything (slow).

        Args:
            robot_id: The acting robot, e.g. "robots/a200-0806"; every other robot becomes an obstacle.
            log: Called on this mirror's thread with one line per full rebuild of the cell: why, and how long.
        """
        self.robot_id = robot_id
        self._log = log
        #: compas_fab's client and planner. Use them only on the owning thread.
        self.client: PyBulletClient | None = None
        # * Cached for the mirror's lifetime: loading and converting is the slow part.
        self._acting: _Acting | None = None
        # (our id, model) of other robots -> their tool models.
        self._others: Dict[Tuple[str, RobotModel], ToolModel] = {}
        # Geometry -> rigid body. ? Keyed by the object itself (identity), never `id(obj)`.
        self._bodies: Dict[Geometry, RigidBody] = {}
        self._connect(gui=False)

    def _connect(self, gui: bool) -> None:
        """Replace the world with a new, empty one, with PyBullet's own window if asked for.

        Raises:
            RuntimeError: If a window is asked for without an X display.
            pybullet.error: If another PyBullet window is open in this process. The old world stays.
        """
        if gui:
            check_display()
        client = PyBulletClient("gui" if gui else "direct", verbose=False)
        client.__enter__()
        if self.client is not None:
            self.close()
        #: Whether PyBullet's own window shows this world.
        self.client, self.gui = client, gui
        self.planner = PyBulletPlanner(self.client)
        #: The cell as last built, and the state as last synced (None before the first sync).
        self.cell: RobotCell | None = None
        self.state: RobotCellState | None = None
        #: (tool, body) pairs of our ids already touching at the last sync, and so allowed.
        self.static_contacts: list[tuple[str, str]] = []
        # What the cell was built from: acting model, other models by id, geometry by id.
        self._built: tuple | None = None
        # Tool id -> SRDF group its tool is attached to, for the cell as built.
        self._groups: Dict[str, str] = {}
        # id() of each tool and rigid body model in the cell -> our id. ? Safe: the cell keeps them alive.
        self._names: dict[int, str] = {}

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

    def sync(self, snapshot: SceneSnapshot) -> None:
        """Make the world match a snapshot: rebuild the cell if its models changed, then set the state.

        Args:
            snapshot: The world to copy. It is not modified.

        Raises:
            KeyError: If the acting robot is not in the snapshot.
            ValueError: If it can't be planned for (`RobotObject.acting_problems`), or has no SRDF.
        """
        acting = snapshot.robots[self.robot_id]
        problems = acting.acting_problems()
        if problems:
            raise ValueError(f"cannot plan for {self.robot_id}: {'; '.join(problems)}")
        others = {key: robot for key, robot in snapshot.robots.items() if key != self.robot_id}
        # Our id -> (geometry, (world pose, placement, touches, enabled)), for everything that can collide.
        # ? Disabled bodies are built too, but hidden: switching them on and off never rebuilds the cell.
        bodies = {body_id: (body.geometry, (snapshot.world_poses[body_id], body.placement, body.touches, body.enabled))
                  for body_id, body in snapshot.bodies.items() if body.geometry.collision}

        built = (acting.model, {key: robot.model for key, robot in others.items()},
                 {key: value[0] for key, value in bodies.items()})
        if not self._same(built):
            why = self._rebuild_reason(built)
            started = time.monotonic()
            self._build(built)
            if self._log is not None:
                self._log(f"rebuilt the planning cell for {self.robot_id} in {time.monotonic() - started:.1f} s; "
                          f"why: {why}")

        model = acting.model
        tool_states = {key: ToolState(frame=None, attached_to_group=self._groups[key],
                                      touch_links=list(model.tool_touches.get(self._acting.flanges[key], ())),
                                      attachment_frame=Frame.worldXY())
                       for key in self._acting.tools}
        for key, robot in others.items():
            zero = self.cell.tool_models[key].zero_configuration()
            # ? compas_fab needs every tool in every state: an absent robot is parked far away.
            tool_states[key] = (ToolState(frame=frame_from_pose(robot.base), configuration=filled(zero, robot.joints))
                                if robot.enabled else ToolState(frame=Frame(PARKED_POSITION), configuration=zero))
        state = RobotCellState(
            robot_base_frame=frame_from_pose(acting.base),
            robot_configuration=filled(self.cell.zero_full_configuration(), acting.joints),
            tool_states=tool_states,
            rigid_body_states={key: self._body_state(*placed) for key, (_, placed) in bodies.items()})
        self.planner.set_robot_cell_state(state)
        self.state = state
        self._allow_static_contacts()

    def _same(self, built: tuple) -> bool:
        """Whether the cell was built from these models: the same model and geometry objects, by `same_source`."""
        if self._built is None:
            return False
        acting, others, geometries = built
        old_acting, old_others, old_geometries = self._built
        return same_source(acting, old_acting) and all(
            new.keys() == old.keys() and all(same_source(value, old[key]) for key, value in new.items())
            for new, old in ((others, old_others), (geometries, old_geometries)))

    def _rebuild_reason(self, built: tuple) -> str:
        """Why the cell must be rebuilt for `built`, in the terms `_same` compares; for the log."""
        if self._built is None:
            return "first build"
        acting, others, geometries = built
        old_acting, old_others, old_geometries = self._built
        reasons = [] if same_source(acting, old_acting) else ["the acting robot's model changed"]
        reasons += _changes("robots", others, old_others)
        reasons += _changes("bodies", geometries, old_geometries)
        return "; ".join(reasons)

    def _build(self, built: tuple) -> None:
        """Build the cell and hand it to compas_fab. Slow: loads models the first time, writes OBJ files.

        Args:
            built: (acting model, other models by our id, geometry by our id).

        Raises:
            ValueError: If the acting robot has no SRDF.
        """
        acting, others, geometries = built
        if self._acting is None or not same_source(acting, self._acting.source):
            if acting.srdf is None:
                raise ValueError(f"{self.robot_id} has no SRDF (RobotModel.srdf); compas_fab needs one")
            model = load_model(acting.urdf, visual=False)
            self._acting = _Acting(acting, model, RobotSemantics.from_srdf_file(str(acting.srdf), model),
                                   {tool.id: tool_model(tool, tool.id) for tool in acting.tools.values()},
                                   {tool.id: flange for flange, tool in acting.tools.items()})

        self._others = {(key, model): self._others.get((key, model))
                        or robot_as_tool(model.urdf, model.tools, key, visual=False)
                        for key, model in others.items()}
        self._bodies = {geometry: self._bodies.get(geometry) or rigid_body(geometry)
                        for geometry in geometries.values()}
        tools = {**self._acting.tools, **{key: self._others[key, model] for key, model in others.items()}}
        rigid_bodies = {key: self._bodies[geometry] for key, geometry in geometries.items()}

        self.cell = RobotCell(self._acting.model, self._acting.semantics, tool_models=tools,
                              rigid_body_models=rigid_bodies)
        self.planner.set_robot_cell(self.cell)
        self._built = built
        self._groups = {key: planning_group(self.cell, flange) for key, flange in self._acting.flanges.items()}
        self._names = {id(value): key for key, value in [*tools.items(), *rigid_bodies.items()]}

    def _body_state(self, pose: Pose, placement: Union[Pose, Attachment], touches: Tuple[str, ...],
                    enabled: bool) -> RigidBodyState:
        """A body's compas_fab state: attached to one of our links, or stationary at its world pose.

        Args:
            pose: Its world pose.
            placement: As in the scene.
            touches: As in the scene.
            enabled: As in the scene; a disabled body is hidden, so nothing collides with it.

        Returns:
            RigidBodyState: The state.
        """
        own = self.robot_id
        touch_links, touch_bodies = [], []
        for entry in touches:
            if entry.startswith(f"{own}/"):
                touch_links.append(entry.split("/", 2)[2])
            elif entry == own:
                # The whole robot: every link and every mounted tool.
                touch_links.extend(link.name for link in self.cell.robot_model.links)
                touch_bodies.extend(self._acting.tools)
            elif entry.startswith(f"{ROBOTS}/"):
                # Another robot is one tool: any of its links means the whole robot.
                touch_bodies.append("/".join(entry.split("/", 2)[:2]))
            else:
                touch_bodies.append(entry)

        state = RigidBodyState(frame=frame_from_pose(pose), touch_links=touch_links, touch_bodies=touch_bodies,
                               is_hidden=not enabled)
        if isinstance(placement, Attachment):
            if placement.parent == own and placement.link is not None:
                state.attached_to_link = placement.link
                state.attachment_frame = frame_from_pose(placement.grasp)
            elif placement.parent != own:
                # Held by another robot or fixed to a body: stationary here, but it may touch its holder.
                state.touch_bodies.append(placement.parent)
            # ? Held by our base: stationary, since the base does not move while an arm plans.
        return state

    def _allow_static_contacts(self) -> None:
        """Allow other robots' tool/body pairs already touching now; they can't change while this robot plans.

        ! Never the acting robot's own tools: they move with it, so their contacts are real.
        """
        self.static_contacts = []
        client = self.client
        for tool, tool_body in client.tools_puids.items():
            if self.state.tool_states[tool].attached_to_group is not None:
                continue
            for body, parts in client.rigid_bodies_puids.items():
                state = self.state.rigid_body_states[body]
                # ? A hidden body is not moved by compas_fab, so its parts may sit anywhere: skip it.
                if state.is_hidden or state.attached_to_link or tool in state.touch_bodies:
                    continue
                if any(p.getClosestPoints(tool_body, part, 0.0, physicsClientId=client.client_id) for part in parts):
                    state.touch_bodies.append(tool)
                    self.static_contacts.append((tool, body))

    # --- --- --- --- --- QUERIES --- --- --- --- ---

    def configuration(self, joints: Mapping[str, float]) -> Configuration:
        """The acting robot's full configuration: `joints` where given, else the synced value.

        Args:
            joints: Values by joint name.

        Returns:
            Configuration: Every configurable joint.
        """
        return filled(self.state.robot_configuration, joints)

    def state_at(self, joints: Mapping[str, float]) -> RobotCellState:
        """The synced state with some joints changed; shares everything else with `state`.

        Args:
            joints: Values by joint name.

        Returns:
            RobotCellState: A new state. ! Treat its tool and body states as read-only.
        """
        return RobotCellState(robot_base_frame=self.state.robot_base_frame,
                              robot_configuration=self.configuration(joints),
                              tool_states=self.state.tool_states, rigid_body_states=self.state.rigid_body_states)

    def collisions(self, joints: Mapping[str, float] | None = None, full_report: bool = False) -> list[tuple[str, str]]:
        """What collides with the acting robot or its tools at the synced state, some joints changed.

        Args:
            joints: Values by joint name to check at, or None for the synced state.
            full_report: Find every pair; otherwise stop at the first.

        Returns:
            list[tuple[str, str]]: Colliding pairs as our ids, e.g. ("robots/a200-0806/left_ur_arm_wrist_3_link",
                "obstacles/tables/A"); a mounted tool by its tool id. Empty if clear.
        """
        state = self.state if joints is None else self.state_at(joints)
        try:
            self.planner.check_collision(state, {"full_report": full_report})
        except CollisionCheckError as error:
            return [(self._our_id(a), self._our_id(b)) for a, b in error.collision_pairs]
        return []

    def search_check(self, joint_names: Sequence[str]) -> SearchCheck:
        """A fast collision check for a search that moves only these joints, from the synced state.

        Args:
            joint_names: The joints the search changes, e.g. one arm's six.

        Returns:
            SearchCheck: Call it with joint values in `joint_names` order.
        """
        return SearchCheck(self, joint_names)

    def _our_id(self, model: Link | ToolModel | RigidBody) -> str:
        """Our id of one side of a compas_fab collision pair."""
        name = self._names.get(id(model))
        return name if name is not None else f"{self.robot_id}/{model.name}"

    @property
    def connected(self) -> bool:
        """bool: Whether the world still exists. False once closed, or once its window was closed."""
        return self.client.client_id is not None and bool(p.isConnected(physicsClientId=self.client.client_id))

    def close(self) -> None:
        """Disconnect this world and delete its files, unless already gone."""
        if self.client.client_id is not None:
            if self.connected:
                self.client.__exit__()
            else:
                self.client._cache_dir.cleanup()
            self.client.client_id = None


def _changes(kind: str, new: Mapping, old: Mapping) -> list[str]:
    """What differs between two maps of our id -> model, for the rebuild log.

    Args:
        kind: What the ids are, e.g. "bodies".
        new: The models now.
        old: The models the cell was built from.

    Returns:
        list[str]: Up to three parts, e.g. "bodies added: bars/3, bars/4".
    """
    parts = []
    for label, ids in (("added", new.keys() - old.keys()), ("removed", old.keys() - new.keys()),
                       ("changed", {key for key in new.keys() & old.keys() if not same_source(new[key], old[key])})):
        if ids:
            ids = sorted(ids)
            shown = ", ".join(ids[:5]) + (f" and {len(ids) - 5} more" if len(ids) > 5 else "")
            parts.append(f"{kind} {label}: {shown}")
    return parts


# --- --- --- --- --- FAST CHECKS FOR SEARCHES --- --- --- --- ---

@dataclass(frozen=True)
class _Part:
    """One side of a pair to check: a PyBullet body, or one link of it.

    Attributes:
        body: PyBullet body id.
        link: Link index, or None for the whole body.
        name: Our id, for reports.
    """

    body: int
    link: Optional[int]
    name: str


def _offset(client_id: int, robot: int, link: int, body: int) -> tuple:
    """Where a body sits in a robot link's frame now, as (position, quaternion)."""
    state = p.getLinkState(robot, link, computeForwardKinematics=True, physicsClientId=client_id)
    inverse = p.invertTransform(state[4], state[5])
    return p.multiplyTransforms(*inverse, *p.getBasePositionAndOrientation(body, physicsClientId=client_id))


class SearchCheck:
    """compas_fab's collision rules for a search over some joints, resolved once and checked fast.

    * Why ours: compas_fab's `check_collision` resets the whole cell and walks every pair on each call
      (3.6 ms vs 0.16 ms here; arm plans ran ~12x slower and hit the search time limit). Replace it once
      compas_fab offers a fast repeated check.

    - ! Only pairs with a moving side are checked: check the start once with `collisions`.
    - ! It leaves the world at the last configuration checked; `sync` and `collisions` put it back.
    - ! Build it after each `sync`.
    """

    def __init__(self, mirror: CompasFabMirror, joint_names: Sequence[str]):
        """Resolve the pairs to check for a search over `joint_names`.

        Args:
            mirror: A synced mirror; its current state is the one searched from.
            joint_names: The joints the search changes.
        """
        client, state, model = mirror.client, mirror.state, mirror.cell.robot_model
        self._client_id = client.client_id
        self._robot = client.robot_puid
        self._joints = [client.robot_joint_puids[name] for name in joint_names]
        own = mirror.robot_id

        # * What moves: every link below a searched joint, and the bodies and tools attached to those links.
        moving = set().union(*(subtree(model, model.get_joint_by_name(name).child.link) for name in joint_names))
        bodies = {name: body for name, body in state.rigid_body_states.items()
                  if not body.is_hidden and name in client.rigid_bodies_puids}
        carried = {name for name, body in bodies.items() if body.attached_to_link in moving}
        #: (PyBullet parts, link index, attachment as (position, quaternion)) of each carried body.
        self._carried = [(client.rigid_bodies_puids[name], client.robot_link_puids[bodies[name].attached_to_link],
                          pose_from_frame(bodies[name].attachment_frame)) for name in carried]
        # ? compas_fab placed the acting robot's tools at `sync`: keep each where it sits in its flange's frame.
        flanges = {name: mirror.cell.get_end_effector_link_name(tool.attached_to_group)
                   for name, tool in state.tool_states.items() if tool.attached_to_group is not None}
        carried_tools = {name for name, flange in flanges.items()
                         if flange in moving and not state.tool_states[name].is_hidden}
        self._carried += [([client.tools_puids[name]], client.robot_link_puids[flanges[name]],
                           _offset(self._client_id, self._robot, client.robot_link_puids[flanges[name]],
                                   client.tools_puids[name])) for name in carried_tools]

        # ? Links without collision shapes (tool0, flange, …) never hit anything: leave them out.
        links = {name: _Part(self._robot, index, f"{own}/{name}") for name, index in client.robot_link_puids.items()
                 if p.getCollisionShapeData(self._robot, index, physicsClientId=self._client_id)}
        tools = {name: _Part(body, None, name) for name, body in client.tools_puids.items()
                 if not state.tool_states[name].is_hidden}
        parts = {name: [_Part(body, None, name) for body in client.rigid_bodies_puids[name]] for name in bodies}

        # * The pairs compas_fab's check_collision walks (CC.1 to CC.5), with a moving side only.
        disabled = client.unordered_disabled_collisions
        pairs = [(links[a], links[b]) for a, b in combinations(links, 2)
                 if (a in moving or b in moving) and frozenset((a, b)) not in disabled]
        pairs += [(links[link], tool) for link in links for name, tool in tools.items()
                  if (link in moving or name in carried_tools) and link not in state.tool_states[name].touch_links]
        pairs += [(links[link], part) for link in links for name, body in bodies.items()
                  if (link in moving or name in carried) and link not in body.touch_links for part in parts[name]]
        for name in carried:
            for other, body in bodies.items():
                # ? Two carried bodies once, not twice.
                if other == name or (other in carried and other < name) or \
                        name in body.touch_bodies or other in bodies[name].touch_bodies:
                    continue
                pairs += [(a, b) for a in parts[name] for b in parts[other]]
        pairs += [(tool, part) for tool_name, tool in tools.items() for name, body in bodies.items()
                  if (name in carried or tool_name in carried_tools)
                  and body.attached_to_tool != tool_name and tool_name not in body.touch_bodies for part in parts[name]]
        #: (body A, body B, getClosestPoints link arguments, names) per pair.
        self._pairs = [(a.body, b.body, {**({"linkIndexA": a.link} if a.link is not None else {}),
                                         **({"linkIndexB": b.link} if b.link is not None else {})}, (a.name, b.name))
                       for a, b in dict.fromkeys(pairs)]

    def __call__(self, values: Sequence[float]) -> tuple[str, str] | None:
        """Set the searched joints and report the first colliding pair.

        Args:
            values: Joint values, in the order of the joint names given.

        Returns:
            tuple[str, str] | None: The first pair hit, as our ids; None if clear.
        """
        client_id = self._client_id
        for joint, value in zip(self._joints, values):
            p.resetJointState(self._robot, joint, float(value), physicsClientId=client_id)
        for bodies, link, (position, orientation) in self._carried:
            state = p.getLinkState(self._robot, link, computeForwardKinematics=True, physicsClientId=client_id)
            world = p.multiplyTransforms(state[4], state[5], position, orientation)
            for body in bodies:
                p.resetBasePositionAndOrientation(body, *world, physicsClientId=client_id)
        for body_a, body_b, links, names in self._pairs:
            if p.getClosestPoints(body_a, body_b, 0.0, physicsClientId=client_id, **links):
                return names
        return None
