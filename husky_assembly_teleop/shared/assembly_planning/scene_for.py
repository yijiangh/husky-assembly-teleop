"""
The planning scene of one process step on the real robots: live robots, the snapshot's obstacles, the step's bodies.

Other robots that are not measured are filled in from a belief or the design, else parked.

- ! Pure: take the snapshot on the main thread, run `scene_for` on the planner's worker.
- Ids and Geometry objects are the process plugin's (`scene_bodies`), so a mirror only moves bodies, never rebuilds.
- ! Parked and unconfigured robots are no obstacles: each one is named in `warnings`.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Mapping

from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import IdMap, split_link_id
from scaffolding_assembly_core.ids import robot_id as scene_robot_id
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.process import RobotState
from scaffolding_assembly_core.process.plan_check import end_state
from scaffolding_assembly_core.process.scenes import seeded
from scaffolding_assembly_core.scene import Attachment, Scene, world_poses

from ...world.kinematics import stow_joints
from ..process_scene.scene_bodies import live_ids, scene_bodies
from .process_steps import LoadedProcess, Step

if TYPE_CHECKING:
    from ...config import RobotConfig

#: Where a robot's base and joints come from, from measured to guessed; a robot's source is the most guessed it uses.
SOURCES = ("live", "belief", "design", "design (stowed arms)", "parked")
LIVE, BELIEF, DESIGN, STOWED, PARKED = SOURCES


@dataclass(frozen=True)
class RobotBelief:
    """Where a planner believes a robot is that is not measured, e.g. where it was last seen; None: no belief.

    Attributes:
        base: World pose of its base.
        joints: Values by joint name; may cover some joints only.
    """

    base: Pose | None = None
    joints: Mapping[str, float] | None = None


@dataclass(frozen=True)
class PlanningScene:
    """A scene to plan one process step in, and how it was made.

    Attributes:
        scene: The scene, world poses resolved; hand it to the acting robot's mirror.
        acting: The acting robot's live id, e.g. "robots/a200-0806".
        ids: Process id -> live id (`live_ids`).
        process_ids: Live id -> process id.
        sources: Live id of each other robot -> where its base and joints come from (`SOURCES`).
        warnings: Robots and bodies left out of the checks, and why.
    """

    scene: Scene
    acting: str
    ids: IdMap
    process_ids: IdMap
    sources: dict[str, str]
    warnings: tuple[str, ...]


def scene_for(loaded: LoadedProcess, step: Step, snapshot: Scene, robots: tuple[RobotConfig, ...], *,
              prefix: str = "process/", beliefs: Mapping[str, RobotBelief] | None = None, hide_held: bool = False,
              ignore_built: bool = False, obstacles_collide: bool = True, keep: tuple[str, ...] = ("obstacles/",),
              end_contacts: bool = False) -> PlanningScene:
    """The planning scene of one step: the snapshot's robots and kept bodies, plus the step's process bodies.

    Another robot that is not measured gets, field by field, its live value, else the belief, else the design (the
    step's start state; stowed arms on a design base without joints). One left with an unknown is parked: it and
    the bodies it holds are no obstacles.

    Args:
        loaded: The loaded process.
        step: The step; its movement's start is the state.
        snapshot: This tick's snapshot; it is not changed.
        robots: The configured robots, `ctx.config.robots`.
        prefix: The process plugin's id prefix.
        beliefs: Live robot id -> belief, for robots not measured.
        hide_held: Disable the bodies the acting robot holds (plan without the bar).
        ignore_built: Disable the built bars and their halves, except the action's own bar and those the acting
            robot holds.
        obstacles_collide: Whether the process's obstacles from Rhino count (`scene_bodies`); the measured ones
            kept from the snapshot always do.
        keep: Prefixes of the snapshot bodies to keep; the process bodies replace the rest.
        end_contacts: Also let each tool touch the body it is on at the movement's end (a linear move into a grasp):
            the tool with that body only, not the rest of its part.

    Returns:
        PlanningScene: A new scene; it shares only Geometry and RobotModel objects.

    Raises:
        ValueError: If the acting robot is not configured or not in the snapshot; or, from `scene_bodies`, if a
            configured robot lacks a link the plan holds a body by, or two process ids map to one live id.
    """
    process, start = loaded.process, step.movement.start
    ids, process_ids = live_ids(process, robots, prefix)
    acting = ids.get(step.action.robot) if step.action.robot else None
    if acting is None or acting not in snapshot.robots:
        raise ValueError(f"{step.label}: its robot {step.action.robot or '(none)'} is not configured or not in the "
                         f"scene")
    scene = Scene(snapshot.tick, snapshot.time, robots={key: robot.copy() for key, robot in snapshot.robots.items()})
    for body_id, body in snapshot.bodies.items():
        if body_id.startswith(tuple(keep)):
            scene.bodies[body_id] = body.copy()
    for body in scene_bodies(process, step.movement, robots, prefix, obstacles_collide):
        scene.put(body)
    warnings = [f"{planned} (serial {spec.serial}) is not configured: left out of the planning scene"
                for planned, spec in process.robots.items() if planned not in ids and start.robots.get(planned)]

    # * Other robots: fill in what is not measured, or park them.
    sources = {}
    configs = {scene_robot_id(config.serial): config for config in robots if config.model is not None}
    for live, robot in scene.robots.items():
        if live == acting:
            continue
        planned = process_ids.get(live)
        design = start.robots.get(planned) if planned is not None else None
        sources[live] = _fill(scene, live, configs.get(live), design, (beliefs or {}).get(live))
        if sources[live] == PARKED:
            warnings.append(f"{live} is not measured and has no belief or design to stand in: parked, so it is "
                            f"no obstacle ({'; '.join(robot.acting_problems())})")

    parked = {live for live, source in sources.items() if source == PARKED}
    # * Never the action's own bar: the robot works on it, built or not.
    kept = {step.action.bar} | {bar for bar, holders in start.attached.items()
                                if any(split_link_id(holder.to)[0] == step.action.robot for holder in holders)}
    built = {ids(body_id) for bar in start.built - kept for body_id in (bar, *process.halves_of(bar))}
    for body_id, body in scene.bodies.items():
        holder = body.placement.parent if isinstance(body.placement, Attachment) else None
        if holder in parked and body.enabled:
            warnings.append(f"{body_id} is held by parked {holder}: disabled")
            body.enabled = False
        if (hide_held and holder == acting) or (ignore_built and body_id in built):
            body.enabled = False

    if end_contacts:
        # * The old monitor's per-movement contacts: the gripper closing on its bar touches it all along the line.
        for tool, tool_state in end_state(step.movement).tools.items():
            on = tool_state.on if tool_state is not None else None
            body = scene.bodies.get(ids(on)) if on is not None and on in ids and tool in ids else None
            if body is not None and ids(tool) not in body.touches:
                body.touches = (*body.touches, ids(tool))

    scene.world_poses = world_poses(scene.bodies, scene.robots,
                                    lambda robot, link: link_pose(robot.model.urdf, robot.base, robot.joints, link))
    for body_id in [body_id for body_id in scene.bodies if body_id not in scene.world_poses]:
        # ? Without a world pose the compas_fab cell refuses the whole scene (KeyError).
        warnings.append(f"{body_id} has no world pose: removed")
        scene.remove(body_id)
    return PlanningScene(scene, acting, ids, process_ids, sources, tuple(warnings))


def _fill(scene: Scene, robot_id: str, config: RobotConfig | None, design: RobotState | None,
          belief: RobotBelief | None) -> str:
    """Fill in a robot's unmeasured base and joints in place (`seeded`), if every one gets a value; its source."""
    robot = scene.robots[robot_id]
    if not robot.acting_problems():
        return LIVE
    if not robot.enabled:
        return PARKED
    used = [LIVE]
    base = None
    if not robot.base_tracked:
        if belief is not None and belief.base is not None:
            base, source = belief.base, BELIEF
        elif design is not None and design.base is not None:
            base, source = design.base, DESIGN
        else:
            return PARKED
        used.append(source)
    stowed = stow_joints(config, robot.unmeasured) if config is not None and design is not None \
        and design.base is not None and design.joints is None else {}
    joints = {}
    for name in robot.unmeasured:
        for values, source in ((belief.joints if belief is not None else None, BELIEF),
                               (design.joints if design is not None else None, DESIGN), (stowed, STOWED)):
            if values and name in values:
                joints[name] = values[name]
                used.append(source)
                break
        else:
            return PARKED
    seeded(scene, robot_id, joints, base)
    return max(used, key=SOURCES.index)
