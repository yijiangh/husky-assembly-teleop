"""
How far an arm can reach whatever its joints: a sphere about its shoulder holding its links, its tool and what it holds.

Two arms whose spheres are apart cannot touch, so a robot whose only unknown parts are arms that far from the moving
ones may stay unknown without asking (`out_of_reach`).

- The radius is a bound, not the reach: each link's distance from the shoulder is at most the sum of the joint offsets
  on the way (each offset's length is fixed, whatever the joints), plus its collision shapes' farthest point.
- Pure: runs on any thread.
"""

from __future__ import annotations

from typing import Iterable, Mapping

import numpy as np

from scaffolding_assembly_core.geometry import Geometry, Pose, shape_mesh
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Attachment, Body, Scene
from scaffolding_assembly_core.urdf import load_urdf

#: Gap kept between a moving arm's reach and an unknown one's, metres: beyond the bound's own slack.
REACH_MARGIN = 0.05
#: The joint an arm's name prefixes: "<arm>_shoulder_pan_joint".
PAN_JOINT = "shoulder_pan_joint"


def robot_arms(robot: RobotObject) -> tuple[str, ...]:
    """The robot's arms by name, e.g. ("left_ur_arm", "right_ur_arm"): every "<arm>_shoulder_pan_joint"."""
    return tuple(joint.name[:-len(PAN_JOINT) - 1] for joint in load_urdf(robot.model.urdf).robot.joints
                 if joint.name.endswith(f"_{PAN_JOINT}"))


def extent(geometry: Geometry | None, offset: Pose = Pose()) -> float:
    """The farthest collision point of `geometry` placed at `offset`, from the frame it is placed in, metres."""
    if geometry is None or not geometry.collision:
        return 0.0
    matrix = offset.matrix()
    points = np.vstack([shape_mesh(shape).vertices for shape in geometry.collision])
    return float(np.max(np.linalg.norm(points @ matrix[:3, :3].T + matrix[:3, 3], axis=1)))


def arm_sphere(robot: RobotObject, arm: str, bodies: Mapping[str, Body] = {}) -> tuple[np.ndarray, float]:
    """Where arm `arm` can reach whatever its joints, with its tools and the bodies held on its links.

    Args:
        robot: The robot, at its base pose.
        arm: The arm's name, e.g. "ur_arm".
        bodies: The scene's bodies: those attached to the arm's links count.

    Returns:
        tuple[np.ndarray, float]: The centre in the world (the shoulder, on the pan axis) and the radius, metres.
    """
    model = robot.model
    joints = load_urdf(model.urdf).robot.joints
    shoulder = next(joint.child for joint in joints if joint.name == f"{arm}_{PAN_JOINT}")
    # * Every link below the shoulder: its distance bound, the joint offsets from the shoulder down to it summed.
    reach = {shoulder: 0.0}
    grown = True
    while grown:
        grown = False
        for joint in joints:
            if joint.parent in reach and joint.child not in reach:
                offset = 0.0 if joint.origin is None else float(np.linalg.norm(np.asarray(joint.origin)[:3, 3]))
                reach[joint.child] = reach[joint.parent] + offset
                grown = True
    geometry = model.link_geometry
    radius = max(distance + extent(geometry.get(link)) for link, distance in reach.items())
    for flange, tool in model.tools.items():
        if flange in reach:
            radius = max(radius, reach[flange] + extent(tool.geometry))
    for body in bodies.values():
        placement = body.placement
        if isinstance(placement, Attachment) and placement.parent == robot.id and placement.link in reach:
            radius = max(radius, reach[placement.link] + extent(body.geometry, placement.offset))
    return np.asarray(link_pose(model.urdf, robot.base, {}, shoulder).position, dtype=float), radius


def out_of_reach(snapshot: Scene, acting: str, arms: Iterable[str], margin: float = REACH_MARGIN) -> frozenset[str]:
    """Other robots not measured that `acting`'s moving `arms` cannot reach: base tracked, only arms unknown, far.

    Every unknown arm's sphere (`arm_sphere`) must be `margin` clear of every moving arm's; joints outside the arms
    (wheels) do not count. A robot with its base not tracked is never out of reach: it may stand anywhere.
    """
    candidates = {key: robot for key, robot in snapshot.robots.items()
                  if key != acting and robot.enabled and robot.acting_problems() and robot.base_tracked}
    if not candidates:
        return frozenset()
    moving = [arm_sphere(snapshot.robots[acting], arm, snapshot.bodies) for arm in arms]
    far = set()
    for key, robot in candidates.items():
        unknown = [arm for arm in robot_arms(robot) if any(name.startswith(f"{arm}_") for name in robot.unmeasured)]
        theirs = [arm_sphere(robot, arm, snapshot.bodies) for arm in unknown]
        if all(np.linalg.norm(centre - other) > radius + reach + margin
               for centre, radius in moving for other, reach in theirs):
            far.add(key)
    return frozenset(far)
