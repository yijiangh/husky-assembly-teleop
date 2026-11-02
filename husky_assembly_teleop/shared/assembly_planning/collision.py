"""
Collision checks on a synced compas_fab mirror: one state, or a path swept between its waypoints.

A check that hits becomes a `CollisionHit` (`collision_hit`), which the collisions plugin draws. Pairs can be named in
process terms; `world_frame_fk` fixes compas_fab's forward kinematics off the world origin.

! Worker thread only, on the thread that owns the mirror, after `mirror.sync`.
"""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass, field
from types import MethodType
from typing import Iterator, Mapping, Sequence

import numpy as np
import pybullet as p
from compas.geometry import Frame
from compas_fab.backends import PyBulletPlanner
from compas_fab.backends.exceptions import PlanningGroupNotExistsError
from compas_fab.robots import RobotCellState, TargetMode

from scaffolding_assembly_core.ids import IdMap, link_id
from scaffolding_assembly_core.mirrors.compas import subtree
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror
from scaffolding_assembly_core.scene import Scene

from ...world.collision_hit import CollisionHit, Contact, scene_at

#: Largest joint step between the samples of a swept check, radians.
SWEPT_STEP = 0.01


@contextmanager
def world_frame_fk(planner: PyBulletPlanner) -> Iterator[None]:
    """Give one planner compas_fab's `forward_kinematics` without its second robot-base transform, while it runs.

    ? compas_fab (pinned 995122d, pybullet_forward_kinematics.py:99-102) multiplies the PyBullet link pose, already in
      the world, by the robot base again: off the world origin every pose it gives is wrong. Set on the instance only
      and removed on exit, so no other planner sees a changed compas_fab.

    Args:
        planner: The planner to fix, e.g. the one `CompasFabMirror.lend` yields.
    """
    had_own = "forward_kinematics" in vars(planner)
    previous = vars(planner).get("forward_kinematics")
    planner.forward_kinematics = MethodType(_forward_kinematics_world, planner)
    try:
        yield
    finally:
        if had_own:
            planner.forward_kinematics = previous
        else:
            del planner.forward_kinematics


def _forward_kinematics_world(planner: PyBulletPlanner, robot_cell_state: RobotCellState, target_mode: TargetMode,
                              group: str | None = None, native_scale: float | None = None,
                              options: dict | None = None) -> Frame:
    """compas_fab's `PyBulletForwardKinematics.forward_kinematics`, the same steps without the second base transform.

    Raises:
        PlanningGroupNotExistsError: If the cell has no such group.
    """
    client, cell = planner.client, planner.client.robot_cell
    group = group or cell.main_group_name
    robot_cell_state.assert_target_mode_match(target_mode, group)
    if group not in cell.group_names:
        raise PlanningGroupNotExistsError(f"Planning group '{group}' is not supported by PyBullet planner.")
    planner.set_robot_cell_state(robot_cell_state)
    link = client.robot_link_puids[cell.get_end_effector_link_name(group)]
    frame = cell.pcf_to_target_frames(robot_cell_state, client._get_link_frame(link, client.robot_puid), target_mode,
                                      group)
    if native_scale:
        frame.scale(1 / native_scale)
    return frame


def collisions(mirror: CompasFabMirror, joints: Mapping[str, float] | None = None,
               full_report: bool = False) -> list[tuple[str, str]]:
    """What the acting robot, its tools or what they hold collide with at the synced state, some joints changed.

    - Its tools against other robots and each other (CC.6) included: the core's mirror checks them.
    - Pairs without the acting robot's side (another robot, as a tool, in a table) are left out: they never block it.

    Args:
        mirror: A synced mirror.
        joints: Values by joint name to check at, or None for the synced state.
        full_report: Find every pair; otherwise stop at the first.

    Returns:
        list[tuple[str, str]]: Colliding pairs as scene ids; empty if clear.
    """
    pairs = mirror.collisions(joints, full_report)
    if not pairs:
        return []
    own = _ids_on(mirror, subtree(mirror.cell.robot_model, mirror.cell.robot_model.root.name))
    if not full_report and (pairs[0][0] in own or pairs[0][1] in own):
        return pairs
    # * The first pair was one of the others': only a full report shows whether one of ours hides behind it.
    if not full_report:
        pairs = mirror.collisions(joints, full_report=True)
    pairs = [pair for pair in pairs if pair[0] in own or pair[1] in own]
    return pairs if full_report else pairs[:1]


def collision_hit(mirror: CompasFabMirror, scene: Scene, where: str, joints: Mapping[str, float] | None = None,
                  moved: Sequence[str] | None = None, first: tuple[str, str] | None = None) -> CollisionHit | None:
    """Every pair that blocks a configuration, with its contacts and the scene as checked.

    ! Leaves the mirror at that configuration: sync it before the next search check.

    Args:
        mirror: The robot's mirror, synced to `scene`.
        scene: The scene the mirror was synced to; the hit's scene is a copy at `joints`.
        where: Which configuration, for the message, e.g. "start" or "waypoint 3".
        joints: The joints changed from the synced state, by name; None: the synced state.
        moved: The joints a search moved: only pairs with a moving side block it, as in `search_check`, and that side
            comes first. None: every pair of the acting robot blocks (a state checked in full, such as the start).
        first: The pair the check stopped on, listed first even if the full check misses it.

    Returns:
        CollisionHit | None: The hit, or None if nothing blocks the configuration.
    """
    joints = dict(joints or {})
    pairs = collisions(mirror, joints, full_report=True)  # ! poses the whole world at `joints`, for the contacts
    if moved is not None:
        # * The moving side first: what moved into what.
        moving = _moving_ids(mirror, moved)
        pairs = [(a, b) if a in moving else (b, a) for a, b in pairs if a in moving or b in moving]
        first = first if first is None or first[0] in moving else (first[1], first[0])
    unique = {frozenset(first): first} if first is not None else {}
    for pair in pairs:
        unique.setdefault(frozenset(pair), pair)
    if not unique:
        return None
    pairs = list(unique.values())
    contacts = tuple(_contact(mirror, scene, pair) for pair in pairs)
    return CollisionHit(scene_at(scene, mirror.robot_id, joints), mirror.robot_id, where, tuple(pairs), contacts)


# * Core candidate: `_moving_ids` and `_contact` belong next to `CompasFabMirror.collisions`, as `contacts(joints)`.
#   They repeat the cell's naming here (links as "<robot id>/<link>", tools and bodies by our id, other robots as one
#   tool with "<flange>_tool" links) and read compas_fab's client maps, as `SearchCheck` does.

def _moving_ids(mirror: CompasFabMirror, joint_names: Sequence[str]) -> set[str]:
    """Our ids of what moves with these joints: links below them, the tools on those links and the bodies held."""
    model = mirror.cell.robot_model
    return _ids_on(mirror, set().union(*(subtree(model, model.get_joint_by_name(name).child.link)
                                         for name in joint_names)))


def _ids_on(mirror: CompasFabMirror, links: set[str]) -> set[str]:
    """Our ids of these links of the acting robot, the tools on them and the bodies they hold."""
    state, cell = mirror.state, mirror.cell
    tools = {name for name, tool in state.tool_states.items()
             if tool.attached_to_group is not None and cell.get_end_effector_link_name(tool.attached_to_group) in links}
    bodies = {name for name, body in state.rigid_body_states.items()
              if body.attached_to_link in links or body.attached_to_tool in tools}
    return {link_id(mirror.robot_id, link) for link in links} | tools | bodies


def _contact(mirror: CompasFabMirror, scene: Scene, pair: tuple[str, str]) -> Contact | None:
    """The deepest contact of a pair in the world as it stands, or None if PyBullet finds no point."""
    client_id = mirror.client.client_id
    deepest = None
    for body_a, link_a in _bodies(mirror, pair[0]):
        for body_b, link_b in _bodies(mirror, pair[1]):
            links = {**({"linkIndexA": link_a} if link_a is not None else {}),
                     **({"linkIndexB": link_b} if link_b is not None else {})}
            for point in p.getClosestPoints(body_a, body_b, 0.0, physicsClientId=client_id, **links):
                # ? Point fields: 1, 2 bodies; 3, 4 links; 5, 6 positions on A and B; 8 signed distance.
                if deepest is None or point[8] < deepest[8]:
                    deepest = point
    if deepest is None:
        return None
    narrowed = (_touched(mirror, scene, pair[0], deepest[1], deepest[3]),
                _touched(mirror, scene, pair[1], deepest[2], deepest[4]))
    return Contact(narrowed, tuple(deepest[5]), tuple(deepest[6]), float(deepest[8]))


def _bodies(mirror: CompasFabMirror, our_id: str) -> list[tuple[int, int | None]]:
    """The PyBullet (body, link index or None for all) behind one of our ids in a collision pair."""
    client, prefix = mirror.client, f"{mirror.robot_id}/"
    if our_id.startswith(prefix):
        return [(client.robot_puid, client.robot_link_puids[our_id[len(prefix):]])]
    if our_id in client.tools_puids:
        return [(client.tools_puids[our_id], None)]
    return [(body, None) for body in client.rigid_bodies_puids.get(our_id, ())]


def _touched(mirror: CompasFabMirror, scene: Scene, our_id: str, body: int, link: int) -> str:
    """Narrow a whole other robot to the link (or mounted tool) a contact touched; any other id stays."""
    robot = scene.robots.get(our_id)
    if robot is None:
        return our_id
    client_id = mirror.client.client_id
    name = (p.getBodyInfo(body, physicsClientId=client_id)[0] if link < 0
            else p.getJointInfo(body, link, physicsClientId=client_id)[12]).decode()
    flange = name.removesuffix("_tool")
    if flange != name and flange in robot.model.tools:
        return robot.model.tools[flange].id
    return link_id(our_id, name)


@dataclass(frozen=True)
class SweptReport:
    """What a swept check found.

    Attributes:
        ok: Nothing was hit, and the check ran to the end.
        samples: How many states between the waypoints were checked.
        bad_segments: (segment index, colliding pairs as scene ids) per segment that hits something, in order.
        error: Why the check could not run, if it raised; then `ok` is False.
        colliding_joints: The joints of the first state that hits, by name; None if none did.
    """

    ok: bool
    samples: int
    bad_segments: list[tuple[int, list[tuple[str, str]]]] = field(default_factory=list)
    error: str = ""
    colliding_joints: dict[str, float] | None = None


def swept_check(mirror: CompasFabMirror, joint_names: Sequence[str], path: Sequence[Sequence[float]],
                step: float = SWEPT_STEP) -> SweptReport:
    """Check a path between its waypoints, which the search already checked, at most `step` apart in every joint.

    The pairs of a segment that hits are named once (`collisions` with a full report).
    ! Fails closed: a check that raises gives `ok` False, never a path accepted unchecked.

    Args:
        mirror: A mirror synced to the state the path was planned from.
        joint_names: The joints the path moves.
        path: Waypoints, each with one value per joint name (else the check fails); the first is the start.
        step: Largest joint step between samples, radians.

    Returns:
        SweptReport: What was hit, segment by segment.
    """
    samples, bad, first = 0, [], None
    try:
        points = np.asarray(path, dtype=float)
        if points.ndim != 2 or points.shape[1] != len(joint_names):
            raise ValueError(f"each waypoint must give {len(joint_names)} values, one per joint name; the path is "
                             f"{points.shape}")
        check = mirror.search_check(joint_names)
        for index, (start, end) in enumerate(zip(points[:-1], points[1:])):
            count = max(1, int(np.ceil(np.max(np.abs(end - start)) / step)))
            pairs = None
            for k in range(1, count):
                values = start + (end - start) * (k / count)
                samples += 1
                if check(values) is not None and pairs is None:
                    joints = dict(zip(joint_names, values.tolist()))
                    first = first or joints
                    pairs = collisions(mirror, joints, full_report=True)
            if pairs is not None:
                bad.append((index, sorted(pairs)))
    except Exception as error:
        # ! Whatever went wrong, the path is not verified.
        return SweptReport(False, samples, bad, repr(error), first)
    return SweptReport(not bad, samples, bad, colliding_joints=first)


def in_process_terms(pair: tuple[str, str], process_ids: IdMap) -> tuple[str, str]:
    """A colliding pair of scene ids in process ids, where mapped ("robots/a200-0806/…" -> "robots/cindy/…")."""
    return tuple(process_ids.get(side) or side for side in pair)
