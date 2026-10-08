"""
Scenes from a design: the world at one movement (`scene_at`), or once a bar is built (`scene_after`).

A scene shares nothing mutable with the design: bodies, robots, joints and touches are new; only `Geometry` and
`RobotModel` objects are shared, and a robot's model is the same object for every scene of the same robot.

! Needs yourdfpy (forward kinematics for held bodies): `Design.scene_at` imports this module on first use.
"""

from __future__ import annotations

import threading
from functools import lru_cache
from typing import Dict, Optional, Set, Tuple

from ..kinematics import ForwardKinematics
from ..robot import RobotModel, RobotObject, robot_model
from ..scene import Attachment, Body, SceneSnapshot, world_poses
from .pose import Pose
from .types import Design, Movement, State, ToolSpec, split_link_id

# * One forward kinematics per thread: a parsed URDF keeps the joints it was last set to.
_local = threading.local()


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
def _model(robot_id: str, urdf, srdf, tools: Tuple[Tuple[str, ToolSpec], ...]) -> RobotModel:
    """`design_model`, cached: a tool's `Geometry` hashes by identity, so a new design object is a new model."""
    touches = {flange: tuple(split_link_id(entry)[1] for entry in tool.touches if entry.startswith(f"{robot_id}/"))
               for flange, tool in tools}
    return robot_model(robot_id.split("/", 1)[1], urdf, srdf, dict(tools), touches)


def scene_at(design: Design, movement: Movement) -> SceneSnapshot:
    """The world at the start of a movement, as the design gives it.

    - A robot absent from the state is disabled; one with `joints: null` has every movable joint `unmeasured`
      (at 0), so planners refuse it until its owner fills them in (e.g. `carry.assumed_joints`).
    - A held body is attached to its robot link; an absent body is disabled. Placeholder poses are kept as given.
    - `touches` hold every allowed contact both ways: the design's, its tools' and the state's.

    Args:
        design: The design.
        movement: One of its movements.

    Returns:
        SceneSnapshot: A new scene, `tick` -1.
    """
    return _scene(design, movement.start)


def scene_after(design: Design, bar: str) -> SceneSnapshot:
    """The world once a bar is built, following the schedule.

    That is the start of the first movement after the last action of that bar; after the schedule's last action, its
    last movement's start with the target joints applied.

    Args:
        design: The design.
        bar: A bar id, e.g. "bars/B3".

    Returns:
        SceneSnapshot: A new scene.

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


def _scene(design: Design, state: State, target_joints: Optional[Dict[str, Dict[str, float]]] = None) -> SceneSnapshot:
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
        robots[robot_id] = RobotObject(robot_id, model, robot_state.base, joints, unmeasured=unknown, label=spec.name)

    # * Every allowed contact, both ways: design-level touches of bodies and tools, and this state's.
    contacts: Dict[str, Set[str]] = {}
    pairs = set(state.touches)
    pairs |= {(body_id, other) for body_id, body in design.bodies.items() for other in body.touches}
    pairs |= {(tool_id, other) for tool_id, tool in design.tools.items() for other in tool.touches}
    for a, b in pairs:
        contacts.setdefault(a, set()).add(b)
        contacts.setdefault(b, set()).add(a)

    bodies: Dict[str, Body] = {}
    for body_id, spec in design.bodies.items():
        attached = state.attached.get(body_id)
        if attached is not None:
            robot_id, link = split_link_id(attached.to)
            placement = Attachment(robot_id, link, attached.grasp)
        else:
            placement = state.poses.get(body_id, spec.pose)
        bodies[body_id] = Body(body_id, spec.geometry, placement, tuple(sorted(contacts.get(body_id, ()))),
                               spec.label, enabled=body_id in state.present)

    fk = getattr(_local, "fk", None)
    if fk is None:
        fk = _local.fk = ForwardKinematics()
    poses = world_poses(bodies, robots, lambda robot, link: fk.link_pose(robot.model.urdf, robot.base, robot.joints,
                                                                         link))
    return SceneSnapshot(bodies=bodies, world_poses=poses, robots=robots)
