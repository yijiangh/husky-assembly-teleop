"""Tests for shared.assembly_planning.single_arm: the poses along a line, and one arm's straight move on a mirror."""

from __future__ import annotations

import math

import numpy as np
import pytest
from planning_fixtures import ALICE, BELLE, CINDY, design_scene, purged_process, robot_config, step
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.shared.assembly_planning import single_arm
from husky_assembly_teleop.shared.assembly_planning.ik import ssik_unavailable
from husky_assembly_teleop.shared.assembly_planning.scene_for import scene_for
from husky_assembly_teleop.shared.assembly_planning.single_arm import MAX_JOINT_STEP, interpolate_poses, plan_linear
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror

ALICE_ID = robot_id(f"a200-{ALICE}")
FLANGE = "robots/alice/ur_arm_tool0"
START = Pose((1.0, 2.0, 0.5), tuple(Rotation.from_euler("xyz", (0.3, -0.2, 0.1)).as_quat()))


def _turn(pose: Pose, axis: str, angle: float) -> Pose:
    """A pose turned about one of its own axes."""
    rotation = Rotation.from_quat(pose.orientation) * Rotation.from_euler(axis, angle)
    return Pose(pose.position, tuple(rotation.as_quat()))


def _angle(a: Pose, b: Pose) -> float:
    """The turn between two poses' orientations, radians."""
    return (Rotation.from_quat(a.orientation).inv() * Rotation.from_quat(b.orientation)).magnitude()


@pytest.mark.parametrize("goal", [Pose((1.0, 2.1, 0.47), START.orientation), _turn(START, "z", 0.4),
                                  _turn(Pose((1.02, 2.0, 0.5), START.orientation), "x", 0.07)])
def test_poses_along_a_line(goal):
    """Exactly start and goal at the ends; ceil of the larger step count + 1 poses; every step within both limits."""
    poses = interpolate_poses(START, goal)
    distance = float(np.linalg.norm(np.subtract(goal.position, START.position)))
    assert poses[0] is START and poses[-1] is goal
    assert len(poses) == math.ceil(max(distance / 0.005, _angle(START, goal) / 0.05)) + 1
    for a, b in zip(poses[:-1], poses[1:]):
        assert np.linalg.norm(np.subtract(b.position, a.position)) <= 0.005 + 1e-12
        assert _angle(a, b) <= 0.05 + 1e-12
    middle = poses[len(poses) // 2]
    direction = np.subtract(goal.position, START.position)
    offset = np.subtract(middle.position, START.position)
    assert np.allclose(np.cross(offset, direction), 0.0, atol=1e-12)


def test_no_move_gives_the_two_ends():
    """A goal equal to the start: just the two poses."""
    assert interpolate_poses(START, START) == [START, START]


@pytest.fixture(scope="module")
def alice_grasp():
    """Alice's mirror, B21_H_to_grasp's step and its scene with and without the end contacts."""
    if ssik_unavailable() is not None:
        pytest.skip(ssik_unavailable())
    loaded = purged_process()
    current = step(loaded, "B21_H_to_grasp")
    configs = tuple(robot_config(serial) for serial in (ALICE, BELLE, CINDY))
    snapshot = design_scene(loaded, "B21_H_to_grasp")
    scenes = {contacts: scene_for(loaded, current, snapshot, configs, end_contacts=contacts).scene
              for contacts in (True, False)}
    mirror = CompasFabMirror(ALICE_ID)
    yield mirror, current, scenes
    mirror.close()


def _end_error(robot, path, goal: Pose) -> tuple[float, float]:
    """How far the path's last step puts tool0 from the goal: metres, radians."""
    end = link_pose(robot.model.urdf, robot.base, {**robot.joints, **dict(zip(path.joint_names, path.points[-1]))},
                    "ur_arm_tool0")
    return float(np.linalg.norm(np.subtract(end.position, goal.position))), _angle(end, goal)


@pytest.mark.slow
def test_alice_closes_in_on_her_bar(alice_grasp):
    """B21_H_to_grasp, the bar allowed to touch the gripper at the end: 0.1 m in 21 exact steps, no joint jump."""
    mirror, current, scenes = alice_grasp
    mirror.sync(scenes[True])
    robot = scenes[True].robots[ALICE_ID]
    goal = current.movement.target.link_poses[FLANGE]
    path = plan_linear(mirror, robot, "ur_arm", goal)
    assert path.backend == "ssik" and path.reason == "" and len(path.points) == 21
    assert np.allclose(path.points[0], [robot.joints[name] for name in path.joint_names])
    assert np.max(np.abs(np.diff(path.points, axis=0))) <= MAX_JOINT_STEP
    distance, angle = _end_error(robot, path, goal)
    assert distance < 1e-6 and angle < 1e-6


@pytest.mark.slow
def test_without_end_contacts_the_gripper_hits_the_bar(alice_grasp):
    """Without the end contacts the line stops near its end, where the gripper reaches B21."""
    mirror, current, scenes = alice_grasp
    mirror.sync(scenes[False])
    path = plan_linear(mirror, scenes[False].robots[ALICE_ID], "ur_arm", current.movement.target.link_poses[FLANGE])
    assert path.points is None and path.waypoint > 10
    assert set(path.colliding) == {"tools/a200-0804/ur_arm", "process/bars/B21"}
    assert "collides" in path.reason


@pytest.mark.slow
def test_compas_fab_runs_without_ssik(alice_grasp, monkeypatch):
    """Without ssik compas_fab's planner moves the arm, within its 1 mm, and says so."""
    mirror, current, scenes = alice_grasp
    monkeypatch.setattr(single_arm, "ssik_unavailable", lambda: "ssik turned off by the test")
    mirror.sync(scenes[True])
    robot = scenes[True].robots[ALICE_ID]
    goal = current.movement.target.link_poses[FLANGE]
    told = []
    path = plan_linear(mirror, robot, "ur_arm", goal, log=told.append)
    assert path.backend == "compas_fab" and path.points is not None
    assert told and told[0].startswith("!")
    assert np.allclose(path.points[0], [robot.joints[name] for name in path.joint_names])
    distance, angle = _end_error(robot, path, goal)
    assert distance < 2e-3 and angle < 2e-3


def _line_from_the_start(alice_grasp, offset):
    """Alice's line from her B21_H_to_grasp start, its goal moved by `offset` (metres, world), turned the same."""
    mirror, _current, scenes = alice_grasp
    mirror.sync(scenes[True])
    robot = scenes[True].robots[ALICE_ID]
    start = link_pose(robot.model.urdf, robot.base, robot.joints, "ur_arm_tool0")
    return plan_linear(mirror, robot, "ur_arm", Pose(tuple(np.add(start.position, offset)), start.orientation))


@pytest.mark.slow
def test_a_line_out_of_reach_stops_where_ik_ends(alice_grasp):
    """1.5 m out at 45° and 0.4 m down: no branch reaches a step part way; the step is named, no pair."""
    path = _line_from_the_start(alice_grasp, (1.5 * math.cos(math.pi / 4), 1.5 * math.sin(math.pi / 4), -0.4))
    assert path.points is None and path.reason.endswith(": unreachable") and path.colliding is None
    assert 1 < path.waypoint < 311 and path.reason.startswith(f"waypoint {path.waypoint}/311")


@pytest.mark.slow
def test_a_line_through_a_flip_stops_at_the_joint_jump(alice_grasp):
    """1 m straight up: the nearest branch turns a joint by about π in one 5 mm step; refused as a joint jump."""
    path = _line_from_the_start(alice_grasp, (0.0, 0.0, 1.0))
    assert path.points is None and "joint jump of" in path.reason and path.colliding is None
    assert 1 < path.waypoint < 200 and float(path.reason.split("joint jump of ")[1].split()[0]) > MAX_JOINT_STEP
