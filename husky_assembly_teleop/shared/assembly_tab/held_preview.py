"""
A move with a bar held in two tools, for its preview: what the robot holds, and how well the two tools keep it.

`held_bodies` is what the preview robot carries (`RobotGhost.hold`); `held_drift` and `grasp_drift` are what the
Plan plots draw (`PlanPlots`): how far one tool0 seen from the other moves from where it is at the first waypoint.
Pure: any thread.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Sequence

import numpy as np
from scaffolding_assembly_core.geometry import Geometry, Pose, compose, invert
from scaffolding_assembly_core.ids import split_link_id
from scaffolding_assembly_core.kinematics import link_poses
from scaffolding_assembly_core.process import Holder
from scaffolding_assembly_core.process.relations import is_present, placement
from scipy.spatial.transform import Rotation

from ..execution.compliant_part import tool0_by_arm
from .plan_plots import GraspDrift

if TYPE_CHECKING:
    from ...robot_interface.robot import HuskyRobotInterface
    from ..assembly_planning.process_steps import LoadedProcess

#: Points checked between two waypoints of a timed path (`held_drift`).
BETWEEN = 3
#: The robot's root at the origin: the tools' relative pose is the same wherever the base stands.
IDENTITY = Pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def held_bodies(loaded: LoadedProcess, index: int) -> dict[str, tuple[str, Pose, Geometry]]:
    """What the acting robot's tools hold at a step's start: the bar and the joint halves mounted on it.

    Returns:
        dict[str, tuple[str, Pose, Geometry]]: Body id -> (the holding link's name, the body's pose in that link's
            frame, its shapes).
    """
    step = loaded.steps[index]
    process, state = loaded.process, step.movement.start
    held = {}
    for body_id, body in process.bodies.items():
        where = placement(process, state, body_id)
        if isinstance(where, Holder) and is_present(process, state, body_id):
            holder, link = split_link_id(where.to)
            if holder == step.action.robot:
                held[body_id] = (link, where.grasp, body.geometry)
    return held


def relative_tool0(robot: HuskyRobotInterface, names: Sequence[str], rows: Sequence[Sequence[float]],
                   arms: Sequence[str]) -> list[Pose]:
    """The second arm's tool0 seen from the first's at every row, both through their base_link in the husky frame."""
    first, second = arms[0], arms[1]
    bases = {arm: robot.arms[arm].base_in_husky for arm in (first, second)}
    seen = []
    for row in rows:
        tool0 = tool0_by_arm(robot.config.model.urdf, (first, second), names, row)
        in_husky = {arm: compose(bases[arm], tool0[arm]) for arm in (first, second)}
        seen.append(compose(invert(in_husky[first]), in_husky[second]))
    return seen


def with_midpoints(rows: Sequence[Sequence[float]]) -> np.ndarray:
    """A joint path with the row halfway between each two put in between: 2 N - 1 rows."""
    rows = np.asarray(rows, dtype=float)
    dense = np.empty((2 * len(rows) - 1, rows.shape[1]))
    dense[::2] = rows
    dense[1::2] = (rows[:-1] + rows[1:]) / 2
    return dense


def grasp_drift(relative: Sequence[Pose], per_waypoint: int = 1) -> GraspDrift:
    """How far the tools' relative pose moves along a path from where it is at the first row.

    Args:
        relative: One tool0 seen from the other at every row (`relative_tool0`).
        per_waypoint: Rows per waypoint: 2 for a path given `with_midpoints`.

    Returns:
        GraspDrift: Per row, the distance in millimetres and the angle of each axis in degrees.
    """
    positions = np.array([pose.position for pose in relative], dtype=float)
    turns = Rotation.from_quat([pose.orientation for pose in relative]).as_matrix()
    # * Each axis against where it pointed at the first row: the columns' dot products.
    cosines = np.einsum("ji,nji->ni", turns[0], turns)
    return GraspDrift(np.arange(len(relative), dtype=float) / per_waypoint,
                      np.linalg.norm(positions - positions[0], axis=1) * 1e3,
                      np.degrees(np.arccos(np.clip(cosines, -1.0, 1.0))))


def held_drift(urdf: Path, names: Sequence[str], times: Sequence[float], rows: Sequence[Sequence[float]],
               arms: Sequence[str], between: int = BETWEEN) -> GraspDrift | None:
    """How far the tools' relative pose moves along a timed path, at its waypoints and between them, over its seconds.

    On the robot's model alone, as the planner holds the bar. Between two waypoints the joints move straight, as the
    arms are sent them; `between` points are checked there.

    Args:
        urdf: The robot's URDF.
        names: The joints of each row.
        times: Seconds at which each row is reached.
        rows: The path, radians.
        arms: The two arms holding the bar, e.g. ("left_ur_arm", "right_ur_arm").
        between: Points checked between each two waypoints.

    Returns:
        GraspDrift | None: Over `times`: the distance at each waypoint, the largest between it and its neighbours
            (`between_mm`), and each axis's largest angle there; None for one arm or fewer than two rows.
    """
    rows = np.asarray(rows, dtype=float)
    if len(arms) < 2 or len(rows) < 2:
        return None
    count = len(rows)
    steps = [rows[:-1] + (rows[1:] - rows[:-1]) * (index + 1) / (between + 1) for index in range(between)]
    first, second = (f"{arm}_tool0" for arm in arms[:2])
    seen = []
    # * The waypoints first, so the first row is what everything is measured against.
    for row in np.concatenate([rows, *steps]):
        flanges = link_poses(urdf, IDENTITY, dict(zip(names, row)), (first, second))
        seen.append(compose(invert(flanges[first]), flanges[second]))
    dense = grasp_drift(seen)

    def around(values: np.ndarray) -> np.ndarray:
        """Per waypoint, the largest of its own value and those between it and its neighbours."""
        inside = values[count:].reshape(between, count - 1, *values.shape[1:]).max(axis=0)
        out = values[:count].copy()
        out[:-1] = np.maximum(out[:-1], inside)
        out[1:] = np.maximum(out[1:], inside)
        return out

    return GraspDrift(np.asarray(times, dtype=float), dense.mm[:count], around(dense.deg), around(dense.mm))
