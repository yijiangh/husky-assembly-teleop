"""
The process scene at a movement, as live scene bodies: process ids mapped to the plugin's ids and the real robots.

`scene_bodies` puts every body of the movement's process scene into the live scene; bodies the plan holds follow the
configured real robot with the planned robot's serial. `live_ids` maps process ids to scene ids and back.

! Any thread: nothing here may touch viser, PyBullet or a PluginContext.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from scaffolding_assembly_core.ids import IdMap
from scaffolding_assembly_core.ids import robot_id as scene_robot_id
from scaffolding_assembly_core.process import Movement, Process
from scaffolding_assembly_core.scene import Attachment, Body, retarget

from .colors import body_color

if TYPE_CHECKING:
    from ...config import RobotConfig


def live_ids(process: Process, robots: tuple[RobotConfig, ...], prefix: str) -> tuple[IdMap, IdMap]:
    """Map process ids to live scene ids, and back.

    A body becomes "<prefix><body id>"; a planned robot (and its links) becomes the configured robot with its serial,
    and each planned tool the real tool on the same flange. Robots not configured are not mapped.

    Args:
        process: The process.
        robots: The configured robots, `ctx.config.robots`.
        prefix: Id prefix, the owning plugin's "<name>/".

    Returns:
        tuple[IdMap, IdMap]: Process id -> live id, and live id -> process id.

    Raises:
        ValueError: If two process ids map to the same live id (e.g. two planned robots with one serial).
    """
    configured = {config.serial[-4:]: config for config in robots if config.model is not None}
    pairs = {body_id: f"{prefix}{body_id}" for body_id in process.bodies}
    for planned, spec in process.robots.items():
        config = configured.get(spec.serial or "")
        if config is None:
            continue
        pairs[planned] = scene_robot_id(config.serial)
        for flange, tool_id in spec.tools.items():
            if flange in config.model.tools:
                pairs[process.tools[tool_id].id] = config.model.tools[flange].id
    back: dict[str, str] = {}
    for own, live in pairs.items():
        if back.setdefault(live, own) != own:
            raise ValueError(f"{back[live]!r} and {own!r} both map to {live!r}: the way back would be ambiguous")
    return IdMap(pairs), IdMap(back)


def scene_bodies(process: Process, movement: Movement, robots: tuple[RobotConfig, ...], prefix: str) -> list[Body]:
    """Every body of the process scene at a movement's start, as a scene body "<prefix><body id>".

    The scene is edited for the live world:
    - a held body is attached to the configured robot with the planned robot's serial (`retarget`); held by a robot
      not configured, it is disabled where the plan has it;
    - `touches` name the real robots, their tools and the prefixed ids; those of robots not configured are dropped.

    * Every movement gives the same ids, geometry objects and colours, so changing the movement never rebuilds a body
      in a mirror or the 3D view: it only moves, enables and disables them.

    Args:
        process: The process.
        movement: The movement; its start is the state.
        robots: The configured robots, `ctx.config.robots`.
        prefix: Id prefix, the owning plugin's "<name>/".

    Returns:
        list[Body]: One scene body per process body.

    Raises:
        ValueError: If a configured robot lacks a link the plan holds a body by, or two process ids map to one
            live id (`live_ids`).
    """
    scene = process.scene_at(movement)
    ids, _ = live_ids(process, robots, prefix)
    models = {scene_robot_id(config.serial): config.model for config in robots if config.model is not None}
    held = {body_id: body.placement for body_id, body in scene.bodies.items()
            if isinstance(body.placement, Attachment)}
    moved = retarget({body_id: held[body_id] for body_id in held if held[body_id].parent in ids}, ids, models)

    bodies = []
    for body_id, body in scene.bodies.items():
        placement = moved.get(body_id) or scene.world_poses.get(body_id,
                                                                process.bodies[body_id].assembly_pose)
        enabled = body.enabled and (body_id not in held or body_id in moved)
        bodies.append(Body(ids(body_id), body.geometry, placement,
                           touches=tuple(ids(other) for other in body.touches if other in ids),
                           label=body.label or body_id, color=tuple(c / 255 for c in body_color(body_id)) + (1.0,),
                           enabled=enabled))
    return bodies
