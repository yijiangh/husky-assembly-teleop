"""
Freezes: what one task relies on standing still, measured once, and whether it still does. Optional, unlike leases.

A freeze judges measurements, so it also sees what this monitor did not send: another monitor, a joystick, a tool's
buttons, a pushed base, a shifted bar. Leases (`robot_interface/robot_lease.py`) only keep this monitor's code apart.

- ! A measurement too old to judge counts as moved: a lost mocap pose or a silent arm breaks the freeze too.
- `take_basis` measures; `basis_problem` compares. `PluginContext.freeze` checks every tick and cancels the task.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Mapping

import numpy as np
from scipy.spatial.transform import Rotation

from ..robot_interface.arm import JOINT_STATE_MAX_AGE, ArmInterface
from ..robot_interface.mocap_state import MocapBody, pose_lost
from ..robot_interface.soft_stop import MotionRefused
from .measured import WorldState


class WorldMoved(MotionRefused):
    """Something a motion relies on moved past its tolerance, or can no longer be measured: nothing more was sent."""

    label = "freeze broken"


@dataclass(frozen=True)
class Tolerance:
    """How far a frozen thing may drift before the freeze breaks.

    ? Guesses, not measured: an arm at rest reads well under 0.1 deg, mocap jitter is under 1 mm.

    Attributes:
        joint: Any one joint, radians.
        position: A base or tracked object, metres.
        angle: A base or tracked object, radians.
    """

    joint: float = np.radians(1.0)
    position: float = 0.01
    angle: float = np.radians(1.0)


@dataclass(frozen=True)
class Basis:
    """The measured state of what must stand still, at `time`.

    Attributes:
        time: ROS time it was measured, seconds.
        joints: Joint angles per (robot serial, arm name), UR_JOINT_NAMES order, radians.
        bases: Mocap pose per robot serial, as (position, quaternion).
        objects: Mocap pose per tracked object name, as (position, quaternion).
    """

    time: float
    joints: Mapping[tuple[str, str], np.ndarray]
    bases: Mapping[str, tuple[np.ndarray, np.ndarray]]
    objects: Mapping[str, tuple[np.ndarray, np.ndarray]]


@dataclass(eq=False)
class Freeze:
    """A basis being watched for one task, and why it broke.

    Attributes:
        basis: What must stand still.
        tolerance: How far it may drift.
        problem: Why it no longer holds, or None while it does.
    """

    basis: Basis
    tolerance: Tolerance
    problem: str | None = None


def take_basis(world: WorldState, now: float, robots: Iterable[str] = (), bases: Iterable[str] = (),
               arms: Iterable[tuple[str, str]] = (), objects: Iterable[str] = ()) -> Basis:
    """Measure what must stand still: whole robots, single bases or arms, and tracked objects.

    Args:
        world: The measured world.
        now: Current ROS time, seconds.
        robots: Serials whose base and every arm must stand still, e.g. the robots a plan only avoids.
        bases: Serials whose base alone must stand still, e.g. the robot whose arm moves.
        arms: (serial, arm name) pairs that must stand still.
        objects: Names in `world.tracked_objects`.

    Returns:
        Basis: Their poses and joints now.

    Raises:
        WorldMoved: If one of them is not measured now: there is nothing to compare against.
        KeyError: If a robot, arm or object is not known.
    """
    robots = tuple(robots)
    arm_keys = [(serial, name) for serial in robots for name in world.robots[serial].arms] + list(arms)
    joints = {}
    for serial, name in dict.fromkeys(arm_keys):
        joints[(serial, name)] = _joints(world.robots[serial].arms[name], f"robot {serial} arm {name}", now)
    base_poses = {serial: _pose(world.robots[serial].base.mocap_id, world.robots[serial].base.state,
                                f"robot {serial} base", now)
                  for serial in dict.fromkeys(robots + tuple(bases))}
    object_poses = {name: _pose(world.tracked_objects[name].mocap_id, world.tracked_objects[name], name, now)
                    for name in objects}
    return Basis(now, joints, base_poses, object_poses)


def basis_problem(basis: Basis, world: WorldState, now: float, tolerance: Tolerance) -> str | None:
    """Why `basis` no longer holds: the first thing that moved past `tolerance` or can no longer be measured; else None.

    Args:
        basis: What must stand still, as measured by `take_basis`.
        world: The measured world.
        now: Current ROS time, seconds.
        tolerance: How far each thing may drift.
    """
    try:
        for (serial, name), then in basis.joints.items():
            what = f"robot {serial} arm {name}"
            turned = float(np.max(np.abs(_joints(world.robots[serial].arms[name], what, now) - then)))
            if turned > tolerance.joint:
                return f"{what} moved {np.degrees(turned):.1f} deg"
        for serial, then in basis.bases.items():
            base = world.robots[serial].base
            moved = _moved(f"robot {serial} base", then, _pose(base.mocap_id, base.state, f"robot {serial} base", now),
                           tolerance)
            if moved:
                return moved
        for name, then in basis.objects.items():
            body = world.tracked_objects.get(name)
            if body is None:
                return f"{name} is no longer tracked"
            moved = _moved(name, then, _pose(body.mocap_id, body, name, now), tolerance)
            if moved:
                return moved
    except WorldMoved as unmeasured:
        return str(unmeasured)
    return None


def _joints(arm: ArmInterface, what: str, now: float) -> np.ndarray:
    """An arm's joints now; WorldMoved if they are missing or older than JOINT_STATE_MAX_AGE."""
    joints = arm.joint_vector()
    if joints is None:
        raise WorldMoved(f"{what}: no joint state")
    age = now - arm.state.last_update_time
    if age > JOINT_STATE_MAX_AGE:
        raise WorldMoved(f"{what}: joint state is {age:.1f} s old")
    return joints


def _pose(mocap_id: int | None, body: MocapBody, what: str, now: float) -> tuple[np.ndarray, np.ndarray]:
    """A mocap body's pose now; WorldMoved if it is lost (`pose_lost`)."""
    lost = pose_lost(mocap_id, body, now)
    if lost:
        raise WorldMoved(f"{what}: {lost}")
    return np.array(body.position, dtype=float), np.array(body.orientation, dtype=float)


def _moved(what: str, then: tuple[np.ndarray, np.ndarray], now: tuple[np.ndarray, np.ndarray],
           tolerance: Tolerance) -> str | None:
    """How `what` moved from `then` to `now` if past `tolerance`, else None."""
    distance = float(np.linalg.norm(now[0] - then[0]))
    angle = float((Rotation.from_quat(then[1]).inv() * Rotation.from_quat(now[1])).magnitude())
    if distance > tolerance.position or angle > tolerance.angle:
        return f"{what} moved {distance * 1e3:.0f} mm, {np.degrees(angle):.1f} deg"
    return None
