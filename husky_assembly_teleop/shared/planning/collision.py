"""
Collision checks on a synced compas_fab mirror: one state, or a path swept between its waypoints.

Hits can be named in process terms; `world_frame_fk` fixes compas_fab's forward kinematics off the world origin.

! Worker thread only, on the thread that owns the mirror, after `mirror.sync`.
"""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass, field
from types import MethodType
from typing import Iterator, Mapping, Sequence

import numpy as np
from compas.geometry import Frame
from compas_fab.backends import PyBulletPlanner
from compas_fab.backends.exceptions import PlanningGroupNotExistsError
from compas_fab.robots import RobotCellState, TargetMode

from scaffolding_assembly_core.ids import IdMap
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror

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
    """What the acting robot or its tools collide with at the synced state, some joints changed (`mirror.collisions`).

    ? A tool against another robot's tool (CC.6) is found only once the core checks it; until then it is missed.

    Args:
        mirror: A synced mirror.
        joints: Values by joint name to check at, or None for the synced state.
        full_report: Find every pair; otherwise stop at the first.

    Returns:
        list[tuple[str, str]]: Colliding pairs as scene ids; empty if clear.
    """
    return mirror.collisions(joints, full_report)


@dataclass(frozen=True)
class SweptReport:
    """What a swept check found.

    Attributes:
        ok: Nothing was hit, and the check ran to the end.
        samples: How many states between the waypoints were checked.
        bad_segments: (segment index, colliding pairs as scene ids) per segment that hits something, in order.
        error: Why the check could not run, if it raised; then `ok` is False.
    """

    ok: bool
    samples: int
    bad_segments: list[tuple[int, list[tuple[str, str]]]] = field(default_factory=list)
    error: str = ""


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
    samples, bad = 0, []
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
                    pairs = collisions(mirror, dict(zip(joint_names, values.tolist())), full_report=True)
            if pairs is not None:
                bad.append((index, sorted(pairs)))
    except Exception as error:
        # ! Whatever went wrong, the path is not verified.
        return SweptReport(False, samples, bad, repr(error))
    return SweptReport(not bad, samples, bad)


def in_process_terms(pair: tuple[str, str], process_ids: IdMap) -> tuple[str, str]:
    """A colliding pair of scene ids in process ids, where mapped ("robots/a200-0806/…" -> "robots/cindy/…")."""
    return tuple(process_ids.get(side) or side for side in pair)
