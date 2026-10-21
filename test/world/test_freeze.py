"""A freeze's basis: measured once, broken by a move past its tolerance or by a measurement too old to judge."""

from __future__ import annotations

from collections import deque
from types import SimpleNamespace

import numpy as np
import pytest
from robot_fakes import fake_arm

from husky_assembly_teleop.robot_interface.arm import JOINT_STATE_MAX_AGE
from husky_assembly_teleop.robot_interface.base import BaseState
from husky_assembly_teleop.robot_interface.mocap_state import POSE_TIMEOUT
from husky_assembly_teleop.world.freeze import Tolerance, WorldMoved, basis_problem, take_basis
from husky_assembly_teleop.world.measured import TrackedObject, WorldState

NOW = 100.0
TOLERANCE = Tolerance()
QUARTER_TURN = np.array([0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)])


def _live(body, now: float = NOW):
    """Give a mocap body a live pose at the origin, measured at `now`."""
    body.position, body.orientation = np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0])
    body.tracked = body.tracking_valid = True
    body.marker_errors, body.last_update_time, body.last_fix_time = deque([(now, 0.0)]), now, now
    return body


def _world() -> WorldState:
    """Robot 0805 with one arm and a live base, and a tracked bar, all measured at NOW."""
    world = WorldState()
    world.robots["0805"] = SimpleNamespace(arms={"ur_arm": fake_arm(now=NOW)},
                                           base=SimpleNamespace(mocap_id=1, state=_live(BaseState())))
    world.tracked_objects["bar"] = _live(TrackedObject("bar", mocap_id=7))
    return world


def _turn_joint(world: WorldState, degrees: float) -> None:
    """Turn the arm's first joint by `degrees`."""
    joints = world.robots["0805"].arms["ur_arm"].state.joint_positions
    joints["ur_arm_shoulder_pan_joint"] += np.radians(degrees)


def test_nothing_moved_keeps_the_basis():
    """A whole robot and a bar, still where they were: no problem."""
    world = _world()
    basis = take_basis(world, NOW, robots=["0805"], objects=["bar"])
    assert set(basis.joints) == {("0805", "ur_arm")} and set(basis.bases) == {"0805"}
    assert basis_problem(basis, world, NOW, TOLERANCE) is None


@pytest.mark.parametrize("move, problem", [
    (lambda world: _turn_joint(world, 0.5), None),
    (lambda world: _turn_joint(world, 2.0), "robot 0805 arm ur_arm moved 2.0 deg"),
    (lambda world: setattr(world.robots["0805"].base.state, "position", np.array([0.005, 0.0, 0.0])), None),
    (lambda world: setattr(world.robots["0805"].base.state, "position", np.array([0.015, 0.0, 0.0])),
     "robot 0805 base moved 15 mm, 0.0 deg"),
    (lambda world: setattr(world.robots["0805"].base.state, "orientation", QUARTER_TURN),
     "robot 0805 base moved 0 mm, 90.0 deg"),
    (lambda world: setattr(world.tracked_objects["bar"], "position", np.array([0.0, 0.0, 0.02])),
     "bar moved 20 mm, 0.0 deg"),
    (lambda world: world.tracked_objects.pop("bar"), "bar is no longer tracked"),
])
def test_a_move_past_the_tolerance_breaks_the_basis(move, problem):
    """Joints, base and object each break it past the tolerance and keep it within."""
    world = _world()
    basis = take_basis(world, NOW, robots=["0805"], objects=["bar"])
    move(world)
    assert basis_problem(basis, world, NOW, TOLERANCE) == problem


def test_a_measurement_too_old_to_judge_counts_as_moved():
    """Silent joints break it once older than JOINT_STATE_MAX_AGE; a mocap pose once older than POSE_TIMEOUT."""
    world = _world()
    arms_only = take_basis(world, NOW, arms=[("0805", "ur_arm")])
    base_only = take_basis(world, NOW, bases=["0805"])
    later = NOW + max(JOINT_STATE_MAX_AGE, POSE_TIMEOUT) + 0.1
    assert "robot 0805 arm ur_arm: joint state is" in basis_problem(arms_only, world, later, TOLERANCE)
    assert basis_problem(base_only, world, later, TOLERANCE).startswith("robot 0805 base: ")


def test_what_is_not_measured_cannot_be_frozen():
    """A lost base pose refuses the basis at once."""
    world = _world()
    world.robots["0805"].base.state.marker_errors.clear()
    with pytest.raises(WorldMoved, match="robot 0805 base: "):
        take_basis(world, NOW, bases=["0805"])
