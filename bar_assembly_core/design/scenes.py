"""
Scenes from a design: the world at one movement (`scene_at`), or once a bar is built (`scene_after`).

A scene shares nothing mutable with the design: bodies, robots, joints and touches are new; only `Geometry` and
`RobotModel` objects are shared, and a robot's model is the same object for every scene of the same robot.
"""

from __future__ import annotations

from functools import lru_cache
from typing import Dict, Optional, Set, Tuple

from ..geometry import Pose
from ..ids import split_link_id
from ..kinematics import link_pose
from ..robot import RobotModel, RobotObject, Tool, robot_model
from ..scene import Attachment, Body, Scene, world_poses
from .relations import allowed_contacts, is_present, placement
from .types import Design, Holder, Movement, State


def design_model(design: Design, robot_id: str) -> RobotModel:
    """The model of a design robot: its URDF and SRDF, its tools, and the links each tool may touch.

    ! The same object for an unchanged robot, also across `dataclasses.replace(design, …)`: mirrors compare models
      by identity.

    Args:
        design: The design.
        robot_id: E.g. "robots/cindy".

    Returns:
        RobotModel: Cached by the robot's files and tool objects.
    """
    spec = design.robots[robot_id]
    tools = tuple(sorted((flange, design.tools[tool_id]) for flange, tool_id in spec.tools.items()))
    return _model(robot_id, spec.urdf, spec.srdf, tools)


@lru_cache(maxsize=64)
def _model(robot_id: str, urdf, srdf, tools: Tuple[Tuple[str, Tool], ...]) -> RobotModel:
    """`design_model`, cached: a tool's `Geometry` hashes by identity, so a new design object is a new model."""
    touches = {flange: tuple(split_link_id(entry)[1] for entry in tool.mount_contacts
                             if entry.startswith(f"{robot_id}/"))
               for flange, tool in tools}
    return robot_model(robot_id.split("/", 1)[1], urdf, srdf, dict(tools), touches)


def scene_at(design: Design, movement: Movement) -> Scene:
    """The world at the start of a movement, as the design gives it.

    - A robot absent from the state is disabled; one with `joints: null` has every movable joint `unmeasured`
      (at 0), one with `base: null` is not `base_tracked`, so planners refuse it until its owner fills them in.
    - Bodies follow the pose rule (`relations.placement`): an attached, unbuilt bar and its halves are attached to
      the first holder's link; every other body has a world pose. An absent body is disabled.
    - `touches` hold every allowed contact both ways, derived (`relations.allowed_contacts`); mount contacts are on
      the robot model.

    Args:
        design: The design.
        movement: One of its movements.

    Returns:
        Scene: A new scene, `tick` -1.
    """
    return _scene(design, movement.start)


def scene_after(design: Design, bar: str) -> Scene:
    """The world once a bar is built, following the schedule.

    That is the start of the first movement after the last action of that bar; after the schedule's last action, its
    last movement's start with the target joints applied.

    Args:
        design: The design.
        bar: A bar id, e.g. "bars/B3".

    Returns:
        Scene: A new scene.

    Raises:
        KeyError: If no action places that bar.
    """
    indices = [index for index, action_id in enumerate(design.schedule) if design.actions[action_id].bar == bar]
    if not indices:
        raise KeyError(f"no action in the schedule places {bar!r}")
    following = indices[-1] + 1
    if following < len(design.schedule):
        return _scene(design, design.actions[design.schedule[following]].movements[0].start)
    last = design.actions[design.schedule[indices[-1]]].movements[-1]
    return _scene(design, last.start, last.target.joints if last.target is not None else None)


def _scene(design: Design, state: State, target_joints: Optional[Dict[str, Dict[str, float]]] = None) -> Scene:
    """A new scene of one design state; `target_joints` (robot id -> joints) override the state's."""
    robots: Dict[str, RobotObject] = {}
    for robot_id, spec in design.robots.items():
        model = design_model(design, robot_id)
        robot_state = state.robots.get(robot_id)
        if robot_state is None:
            robots[robot_id] = RobotObject(robot_id, model, Pose(), {}, enabled=False, label=spec.name)
            continue
        joints = dict(robot_state.joints or {})
        joints.update((target_joints or {}).get(robot_id, {}))
        unknown = frozenset() if robot_state.joints is not None else frozenset(model.movable_joints) - set(joints)
        # ? A base the design leaves open counts as not tracked: planners refuse it until its owner fills it in.
        robots[robot_id] = RobotObject(robot_id, model, robot_state.base or Pose(), joints,
                                       base_tracked=robot_state.base is not None, unmeasured=unknown, label=spec.name)

    # * Allowed contacts are derived, never stored: both ways, on every body that takes part.
    contacts: Dict[str, Set[str]] = {}
    for a, b in allowed_contacts(design, state):
        contacts.setdefault(a, set()).add(b)
        contacts.setdefault(b, set()).add(a)

    bodies: Dict[str, Body] = {}
    for body_id, spec in design.bodies.items():
        where = placement(design, state, body_id)
        if isinstance(where, Holder):
            robot_id, link = split_link_id(where.to)
            where = Attachment(robot_id, link, where.grasp)
        bodies[body_id] = Body(body_id, spec.geometry, where, tuple(sorted(contacts.get(body_id, ()))),
                               spec.label, enabled=is_present(design, state, body_id))

    poses = world_poses(bodies, robots, lambda robot, link: link_pose(robot.model.urdf, robot.base, robot.joints,
                                                                      link))
    return Scene(bodies=bodies, world_poses=poses, robots=robots)
