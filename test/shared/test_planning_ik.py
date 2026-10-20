"""Tests for shared.assembly_planning.ik: which ssik solver fits which URDF, exact branches, and solving on a mirror."""

from __future__ import annotations

from math import cos, sin

import numpy as np
import pytest
from planning_fixtures import ALICE, BELLE, CINDY, purged_process, robot_config

from husky_assembly_teleop.shared.assembly_planning import ik
from husky_assembly_teleop.shared.assembly_planning.ik import (MAX_BRANCHES, PYBULLET_TOLERANCE, branches,
                                                               fitting_solver, solve_ik, ssik_unavailable)
from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names
from husky_assembly_teleop.world.kinematics import stow_joints
from scaffolding_assembly_core.geometry import Pose, box_geometry
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Body, Scene
from scipy.spatial.transform import Rotation

CINDY_ID = robot_id(f"a200-{CINDY}")
#: (robot, arm) of every ssik solver.
ARMS = ((ALICE, "ur_arm"), (BELLE, "ur_arm"), (CINDY, "left_ur_arm"), (CINDY, "right_ur_arm"))
BASE = Pose((2.0, 1.0, 0.0), (0.0, 0.0, sin(0.35), cos(0.35)))


@pytest.fixture(scope="module", autouse=True)
def ssik():
    """Skip when ssik can't run in this Python."""
    why = ssik_unavailable()
    if why is not None:
        pytest.skip(why)


@pytest.fixture(scope="module")
def mirror():
    """One mirror planning for Cindy, shared by the tests."""
    mirror = CompasFabMirror(CINDY_ID)
    yield mirror
    mirror.close()


def _cindy() -> RobotObject:
    """Cindy at BASE, arms stowed."""
    config = robot_config(CINDY)
    return RobotObject(CINDY_ID, config.model, BASE, stow_joints(config, config.model.movable_joints))


def _tool0_targets(robot: RobotObject) -> dict[str, Pose]:
    """Each arm's tool0 where the robot has it now."""
    return {arm.name: link_pose(robot.model.urdf, robot.base, robot.joints, f"{arm.name}_tool0")
            for arm in robot_config(CINDY).arms}


def test_each_live_robot_fits_its_own_solver():
    """Alice's URDF fits the alice solver, Belle's belle, Cindy's arms left and right."""
    assert [fitting_solver(robot_config(serial).model.urdf, arm) for serial, arm in ARMS] == [
        "alice", "belle", "left", "right"]
    assert fitting_solver(robot_config(CINDY).model.urdf, "ur_arm") is None


def test_process_alice_fits_no_solver():
    """The purged process's Alice URDF fits no solver: always solve on the live model."""
    assert fitting_solver(purged_process().process.robots["robots/alice"].urdf, "ur_arm") is None


def test_branches_round_trip():
    """At 20 random bases per arm: the first branch is the joints the target came from, every branch reaches it."""
    generator = np.random.default_rng(7)
    for serial, arm in ARMS:
        urdf = robot_config(serial).model.urdf
        names, solver = arm_joint_names(arm), fitting_solver(urdf, arm)
        for _ in range(20):
            yaw = generator.uniform(-np.pi, np.pi)
            base = Pose((*generator.uniform(-3.0, 3.0, 2), 0.0), (0.0, 0.0, sin(yaw / 2), cos(yaw / 2)))
            q = generator.uniform(-np.pi, np.pi, 6) * np.array([1.0, 1.0, 0.9, 1.0, 1.0, 1.0])
            target = link_pose(urdf, base, dict(zip(names, q)), f"{arm}_tool0")
            found = branches(urdf, base, arm, solver, target, q)
            assert 1 <= len(found) <= MAX_BRANCHES
            assert np.allclose(found[0], q, atol=1e-6)
            moves = [float(np.max(np.abs(branch - q))) for branch in found]
            assert moves == sorted(moves)
            for branch in found:
                reached = link_pose(urdf, base, dict(zip(names, branch)), f"{arm}_tool0")
                assert np.allclose(reached.position, target.position, atol=1e-6)
                assert abs(abs(np.dot(reached.orientation, target.orientation)) - 1.0) < 1e-9


@pytest.mark.slow
def test_solve_ik_at_the_stowed_pose(mirror):
    """Cindy stowed, targets where her tool0s are: rank 1, no move."""
    cindy = _cindy()
    mirror.sync(Scene(robots={CINDY_ID: cindy}))
    result = solve_ik(mirror, cindy, _tool0_targets(cindy))
    assert (result.rank, result.backend, result.solvers, result.colliding) == (1, "ssik", ("left", "right"), None)
    assert result.max_change < 1e-6 and result.combinations > 1
    assert set(result.joints) == set(arm_joint_names("left_ur_arm") + arm_joint_names("right_ur_arm"))


@pytest.mark.slow
@pytest.mark.parametrize("link", ["left_ur_arm_wrist_3_link", "left_ur_arm_forearm_link"])
def test_solve_ik_steps_past_a_box(mirror, link):
    """A box at a stowed left arm link: the nearest combination hits it, so a later one wins, or none does.

    Every branch puts the wrist where the tool0 target is, so none clears a box there; a forearm can go elsewhere.
    """
    cindy = _cindy()
    where = link_pose(cindy.model.urdf, BASE, cindy.joints, link)
    box = Body("obstacles/box", box_geometry((0.05, 0.05, 0.05)), Pose(where.position))
    mirror.sync(Scene(bodies={box.id: box}, world_poses={box.id: box.placement}, robots={CINDY_ID: cindy}))
    result = solve_ik(mirror, cindy, _tool0_targets(cindy))
    assert "obstacles/box" in result.colliding
    assert result.rank > 1 or (result.joints is None and "collide" in result.reason)
    if link == "left_ur_arm_forearm_link":
        assert result.rank > 1 and result.joints is not None


@pytest.mark.slow
def test_pybullet_fallback(mirror, monkeypatch):
    """Without ssik, PyBullet's IK solves it and a "!" line says so."""
    monkeypatch.setattr(ik, "ssik_unavailable", lambda: "ssik switched off for the test")
    monkeypatch.setattr(ik, "_told", set())
    cindy = _cindy()
    mirror.sync(Scene(robots={CINDY_ID: cindy}))
    lines = []
    result = solve_ik(mirror, cindy, _tool0_targets(cindy), log=lines.append)
    assert result.backend == "pybullet" and result.joints is not None, result.reason
    assert lines and all(line.startswith("! IK falls back to PyBullet") for line in lines)
    solve_ik(mirror, cindy, _tool0_targets(cindy), log=lines.append)
    assert len(lines) == 2, "told once per URDF and arm"


@pytest.mark.slow
def test_pybullet_fallback_reaches_a_moved_target(mirror, monkeypatch):
    """Without ssik, a left target 0.4 rad of shoulder pan from the stowed reference is reached.

    ? PyBullet's IK stops as soon as it is within PYBULLET_TOLERANCE (1 mm, 0.01 rad), so its result may sit just
      inside it: asserted with 20 % margin. Measured: 0.57 mm, 2.8e-4 rad.
    """
    monkeypatch.setattr(ik, "ssik_unavailable", lambda: "ssik switched off for the test")
    cindy = _cindy()
    mirror.sync(Scene(robots={CINDY_ID: cindy}))
    pan = arm_joint_names("left_ur_arm")[0]
    moved = RobotObject(CINDY_ID, cindy.model, BASE, {**cindy.joints, pan: cindy.joints[pan] + 0.4})
    targets = _tool0_targets(moved)
    result = solve_ik(mirror, cindy, targets, log=lambda _line: None)
    assert result.backend == "pybullet" and result.joints is not None, result.reason
    assert result.max_change > 0.3
    reached = {**cindy.joints, **result.joints}
    for arm, target in targets.items():
        pose = link_pose(cindy.model.urdf, BASE, reached, f"{arm}_tool0")
        assert np.linalg.norm(np.subtract(pose.position, target.position)) < 1.2 * PYBULLET_TOLERANCE[0]
        turn = Rotation.from_quat(pose.orientation).inv() * Rotation.from_quat(target.orientation)
        assert turn.magnitude() < 1.2 * PYBULLET_TOLERANCE[1]
