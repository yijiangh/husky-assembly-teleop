"""
Whether two models of one robot agree, e.g. the robot a process planned with and the configured real robot.

Compared by what matters for planning, never by file: links, joints and flanges, the pose of every link with shapes
and of every flange by forward kinematics at a few joint values (URDF variants differ in single frames), and each
tool's kind, TCP and collision bounds. Pure functions on core types; any thread.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose, shape_mesh
from scaffolding_assembly_core.kinematics import link_poses
from scaffolding_assembly_core.robot import RobotModel, Tool
from scaffolding_assembly_core.urdf import load_urdf

#: Largest link or TCP offset, metres, and turn, radians, that still counts as the same.
#: ? Guesses: calibrated URDFs differ by millimetres, and that is what an operator needs to see.
POSITION_TOLERANCE = 1e-3
ANGLE_TOLERANCE = np.radians(0.1)
#: Largest difference of a tool's collision bounds, metres, that still counts as the same shape.
GEOMETRY_TOLERANCE = 5e-3
#: Joint settings compared, and how far each joint turns from zero in them, radians.
SAMPLES = 3
JOINT_RANGE = 1.5
#: Names listed at most in one line; more are counted.
MAX_NAMES = 3


def model_differences(planned: RobotModel, live: RobotModel,
                      planned_name: str = "the process", live_name: str = "configured") -> list[str]:
    """Where two models of one robot disagree, with sizes; empty if they agree.

    Args:
        planned: One model, e.g. the process's (`process_model`).
        live: The other, e.g. the configured robot's (`RobotConfig.model`).
        planned_name: How messages name the first, e.g. "the process".
        live_name: How messages name the second.

    Returns:
        list[str]: One short line per difference, e.g. "left_ur_arm_tool0 off by 3.2 mm / 0.40°".
    """
    differences = []
    for kind, ours, theirs in (("links", planned.links, live.links),
                               ("movable joints", set(planned.movable_joints), set(live.movable_joints)),
                               ("flanges", set(planned.flanges), set(live.flanges))):
        differences += _only_in(kind, ours - theirs, planned_name) + _only_in(kind, theirs - ours, live_name)
    worst = _worst_link(planned, live)
    if worst is not None:
        differences.append(worst)
    for flange in sorted(set(planned.tools) | set(live.tools)):
        differences += _tool_differences(flange, planned.tools.get(flange), live.tools.get(flange),
                                         planned_name, live_name)
    return differences


def _only_in(kind: str, names: set[str], where: str) -> list[str]:
    """One line naming what only one model has, or none."""
    if not names:
        return []
    shown = sorted(names)[:MAX_NAMES]
    more = f" (+{len(names) - MAX_NAMES} more)" if len(names) > MAX_NAMES else ""
    return [f"{kind} only in {where}: {', '.join(shown)}{more}"]


def _worst_link(planned: RobotModel, live: RobotModel) -> str | None:
    """The link whose pose differs most between the models over SAMPLES joint settings, if over tolerance.

    Only links with shapes, and the flanges: what planners collide with and mount tools on. ? A frame without
    either, such as a non-stock `<arm>_base_link`, does not change a plan.
    """
    links = sorted((_shaped_links(planned) & _shaped_links(live)) | (set(planned.flanges) & set(live.flanges)))
    joints = sorted(set(planned.movable_joints) & set(live.movable_joints))
    if not links:
        return None
    rng = np.random.default_rng(0)
    off: dict[str, tuple[float, float]] = {}
    for _ in range(SAMPLES):
        values = dict(zip(joints, rng.uniform(-JOINT_RANGE, JOINT_RANGE, len(joints))))
        ours, theirs = link_poses(planned.urdf, Pose(), values, links), link_poses(live.urdf, Pose(), values, links)
        for link in links:
            distance, angle = _offset(ours[link], theirs[link])
            known = off.get(link, (0.0, 0.0))
            off[link] = (max(known[0], distance), max(known[1], angle))
    beyond = {link: value for link, value in off.items()
              if value[0] > POSITION_TOLERANCE or value[1] > ANGLE_TOLERANCE}
    if not beyond:
        return None
    link, (distance, angle) = max(beyond.items(), key=lambda item: (item[1][0], item[1][1]))
    return f"{link} off by {_size(distance, angle)} ({len(beyond)} of {len(links)} links differ)"


def _shaped_links(model: RobotModel) -> set[str]:
    """The model's links with a visual or collision element."""
    return {link.name for link in load_urdf(model.urdf).robot.links if link.visuals or link.collisions}


def _tool_differences(flange: str, planned: Tool | None, live: Tool | None,
                      planned_name: str, live_name: str) -> list[str]:
    """How the tools on one flange differ: missing, kind, TCP, collision bounds."""
    if planned is None or live is None:
        where = planned_name if live is None else live_name
        return [f"{flange}: a tool only in {where}"]
    if planned.kind != live.kind:
        return [f"{flange}: {planned.kind} in {planned_name}, {live.kind} {live_name}"]
    differences = []
    distance, angle = _offset(planned.tcp, live.tcp)
    if distance > POSITION_TOLERANCE or angle > ANGLE_TOLERANCE:
        differences.append(f"{flange}: TCP off by {_size(distance, angle)}")
    ours, theirs = _bounds(planned), _bounds(live)
    if (ours is None) != (theirs is None):
        differences.append(f"{flange}: collision shapes only in {planned_name if theirs is None else live_name}")
    elif ours is not None and (gap := float(np.max(np.abs(ours - theirs)))) > GEOMETRY_TOLERANCE:
        differences.append(f"{flange}: collision bounds differ by {gap * 1e3:.1f} mm")
    return differences


def _bounds(tool: Tool) -> np.ndarray | None:
    """The tool's collision shapes' bounding box in the flange frame, (2 x 3), or None without shapes."""
    shapes = tool.geometry.collision
    if not shapes:
        return None
    vertices = np.vstack([shape_mesh(shape).vertices for shape in shapes])
    return np.array([vertices.min(axis=0), vertices.max(axis=0)])


def _offset(a: Pose, b: Pose) -> tuple[float, float]:
    """Distance, metres, and turn, radians, between two poses."""
    distance = float(np.linalg.norm(np.subtract(a.position, b.position)))
    angle = float((Rotation.from_quat(a.orientation).inv() * Rotation.from_quat(b.orientation)).magnitude())
    return distance, angle


def _size(distance: float, angle: float) -> str:
    """An offset as "3.2 mm / 0.40°"."""
    return f"{distance * 1e3:.1f} mm / {np.degrees(angle):.2f}°"
