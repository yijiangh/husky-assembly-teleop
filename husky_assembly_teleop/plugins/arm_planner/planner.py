"""
The arm planner's planning worlds (`assembly_planning.mirrors.MirrorPool`), its search, and the collisions it finds.

The search is `assembly_planning.arm`'s; a collision comes as a `CollisionHit` (every blocking pair with its
contacts), for the collisions plugin.

! Worker thread only: sync, search, check and close on the worker.
"""

from __future__ import annotations

import threading
from functools import partial
from typing import Mapping, Sequence

import numpy as np
import pybullet as p

from ...shared.assembly_planning.arm import ArmPath, _extend
from ...shared.assembly_planning.arm import plan_arm as search_arm
from ...shared.assembly_planning.joints import arm_joint_names
from ...shared.assembly_planning.mirrors import MirrorPool
from ...shared.planner.planner_search import PlanResult
from ...world.collision_hit import CollisionHit, Contact, scene_at
from scaffolding_assembly_core.ids import link_id
from scaffolding_assembly_core.mirrors.compas import subtree
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror
from scaffolding_assembly_core.scene import Scene

__all__ = ["ArmPath", "ArmPlanningWorld", "_extend", "arm_joint_names", "collision_hit", "path_hit", "plan_arm"]

#: The arm planner's planning worlds: one compas_fab mirror per robot (`MirrorPool`).
ArmPlanningWorld = MirrorPool


def plan_arm(world: ArmPlanningWorld, mirror: CompasFabMirror, arm_name: str, goal: Sequence[float],
             abort: threading.Event) -> PlanResult[ArmPath]:
    """Plan a collision-free joint path for one arm (`assembly_planning.arm.plan_arm`), named with the world's labels.

    A start or target in collision comes with its `CollisionHit` (`PlanResult.hit`).

    Args:
        world: The planning world, for the snapshot.
        mirror: The robot's mirror, already synced.
        arm_name: Which arm, e.g. "ur_arm".
        goal: Six joint values, radians, UR driver order.
        abort: Set from the main thread to end the search early.

    Returns:
        PlanResult[ArmPath]: The path, or why there is none.
    """
    return search_arm(mirror, arm_name, goal, abort, label=world.label, hit=partial(collision_hit, world, mirror))


def path_hit(world: ArmPlanningWorld, mirror: CompasFabMirror, arm_name: str,
             waypoints: np.ndarray) -> CollisionHit | None:
    """Why a given joint path for one arm collides: its first waypoint, or straight joint move between two, that does.

    Args:
        world: The planning world, for the snapshot.
        mirror: The robot's mirror, already synced; only this arm's joints change.
        arm_name: Which arm, e.g. "ur_arm".
        waypoints: (n, 6) joint values, radians, UR driver order; checked as given, moves at JOINT_STEP.

    Returns:
        CollisionHit | None: The first configuration that collides, or None if the whole path is clear.
    """
    names = arm_joint_names(arm_name)
    check = mirror.search_check(names)
    previous = None
    for index, waypoint in enumerate(np.asarray(waypoints, dtype=float)):
        for point in [tuple(waypoint)] if previous is None else _extend(previous, waypoint):
            hit = check(point)
            if hit is not None:
                where = f"waypoint {index + 1}" if np.allclose(point, waypoint) else f"the move to waypoint {index + 1}"
                return collision_hit(world, mirror, where, dict(zip(names, point)), hit, names)
        previous = waypoint
    return None


def collision_hit(world: ArmPlanningWorld, mirror: CompasFabMirror, where: str, joints: Mapping[str, float],
                  first: tuple[str, str], moved: Sequence[str] | None) -> CollisionHit:
    """Every pair that blocks a configuration, with its contacts and the scene as checked. On the worker.

    Leaves the world at that configuration; every worker job syncs first anyway.

    Args:
        world: The planning world, synced.
        mirror: The robot's mirror, synced.
        where: Which configuration, for the message.
        joints: The joints changed from the synced state, by name.
        first: The pair the check stopped on.
        moved: The joints a search moved: only pairs with a moving side block it, as in `search_check`, and that side
            comes first. None: every pair blocks (a state checked in full, such as the start).

    Returns:
        CollisionHit: The hit, `first` first.
    """
    pairs = mirror.collisions(joints, full_report=True)  # ! poses the whole world at `joints`, for the contacts
    if moved is not None:
        # * The moving side first: what moved into what.
        moving = _moving_ids(mirror, moved)
        pairs = [(a, b) if a in moving else (b, a) for a, b in pairs if a in moving or b in moving]
        first = first if first[0] in moving else (first[1], first[0])
    unique = {frozenset(first): first}
    for pair in pairs:
        unique.setdefault(frozenset(pair), pair)
    pairs = list(unique.values())
    contacts = tuple(_contact(mirror, world.snapshot, pair) for pair in pairs)
    return CollisionHit(scene_at(world.snapshot, mirror.robot_id, joints), mirror.robot_id, where, tuple(pairs),
                        contacts)


# * Core candidate: `_moving_ids` and `_contact` belong next to `CompasFabMirror.collisions`, as `contacts(joints)`.
#   They repeat the cell's naming here (links as "<robot id>/<link>", tools and bodies by our id, other robots as one
#   tool with "<flange>_tool" links) and read compas_fab's client maps, as `SearchCheck` does.

def _moving_ids(mirror: CompasFabMirror, joint_names: Sequence[str]) -> set[str]:
    """Our ids of what moves with these joints: links below them, the tools on those links and the bodies held."""
    model, state, cell = mirror.cell.robot_model, mirror.state, mirror.cell
    links = set().union(*(subtree(model, model.get_joint_by_name(name).child.link) for name in joint_names))
    tools = {name for name, tool in state.tool_states.items()
             if tool.attached_to_group is not None and cell.get_end_effector_link_name(tool.attached_to_group) in links}
    bodies = {name for name, body in state.rigid_body_states.items()
              if body.attached_to_link in links or body.attached_to_tool in tools}
    return {link_id(mirror.robot_id, link) for link in links} | tools | bodies


def _contact(mirror: CompasFabMirror, snapshot: Scene, pair: tuple[str, str]) -> Contact | None:
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
    narrowed = (_touched(mirror, snapshot, pair[0], deepest[1], deepest[3]),
                _touched(mirror, snapshot, pair[1], deepest[2], deepest[4]))
    return Contact(narrowed, tuple(deepest[5]), tuple(deepest[6]), float(deepest[8]))


def _bodies(mirror: CompasFabMirror, our_id: str) -> list[tuple[int, int | None]]:
    """The PyBullet (body, link index or None for all) behind one of our ids in a collision pair."""
    client, prefix = mirror.client, f"{mirror.robot_id}/"
    if our_id.startswith(prefix):
        return [(client.robot_puid, client.robot_link_puids[our_id[len(prefix):]])]
    if our_id in client.tools_puids:
        return [(client.tools_puids[our_id], None)]
    return [(body, None) for body in client.rigid_bodies_puids.get(our_id, ())]


def _touched(mirror: CompasFabMirror, snapshot: Scene, our_id: str, body: int, link: int) -> str:
    """Narrow a whole other robot to the link (or mounted tool) a contact touched; any other id stays."""
    robot = snapshot.robots.get(our_id)
    if robot is None:
        return our_id
    client_id = mirror.client.client_id
    name = (p.getBodyInfo(body, physicsClientId=client_id)[0] if link < 0
            else p.getJointInfo(body, link, physicsClientId=client_id)[12]).decode()
    flange = name.removesuffix("_tool")
    if flange != name and flange in robot.model.tools:
        return robot.model.tools[flange].id
    return link_id(our_id, name)
