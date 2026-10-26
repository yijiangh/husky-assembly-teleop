"""Tests for shared.assembly_planning.servo_route: servo passes planned on the purged process with the real planners.

All slow: ssik IK, tamp's straight bar-held line and the swept check on Cindy's and Alice's mirrors.
"""

from __future__ import annotations

import numpy as np
import pytest
from planning_fixtures import ALICE, BELLE, CINDY, design_scene, movement, purged_process, robot_config, step

from husky_assembly_teleop.shared.assembly_planning import servo_route
from husky_assembly_teleop.shared.assembly_planning.ik import ssik_unavailable
from husky_assembly_teleop.shared.assembly_planning.servo_route import TWO_ROWS, plan_servo_step
from husky_assembly_teleop.shared.motion_planner_contract import ServoAsk
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror
from scaffolding_assembly_core.scene import Scene

pytestmark = pytest.mark.slow

#: Corrections under this plan nothing, metres (TOOL0_TOL; ? read it from shared/execution/servo.py once that lands).
STOP_BELOW = 0.0002
#: The last row of a correction puts every flange within this of its target, metres and radians.
END_TOL = (0.0002, 1e-3)
CINDY_ID, ALICE_ID = robot_id(f"a200-{CINDY}"), robot_id(f"a200-{ALICE}")
LINEAR, SAMPLING = "tamp.plan_constrained_dual_arm_linear", "tamp.plan_constrained_dual_arm"


@pytest.fixture(scope="module")
def purged():
    """The purged process."""
    return purged_process()


@pytest.fixture(scope="module")
def configs():
    """The three configured robots."""
    return tuple(robot_config(serial) for serial in (ALICE, BELLE, CINDY))


@pytest.fixture(scope="module")
def mirrors():
    """One mirror per robot for every test here, made on first use; closed at the end."""
    if ssik_unavailable() is not None:
        pytest.skip(ssik_unavailable())
    made = {}
    yield made
    for mirror in made.values():
        mirror.close()


def mirror_of(mirrors: dict, live: str) -> CompasFabMirror:
    """A robot's mirror of the shared set."""
    return mirrors.setdefault(live, CompasFabMirror(live))


def shifted(scene: Scene, live: str, offset: tuple[float, float, float]) -> Scene:
    """The scene with one robot's base moved by `offset`, metres: the live base off the design one."""
    robot = scene.robots[live]
    robot.base = Pose.from_arrays(np.add(robot.base.position, offset), robot.base.orientation)
    return scene


def flanges(scene: Scene, live: str, planned: str, arms: tuple[str, ...]) -> dict[str, Pose]:
    """Each arm's world tool0 pose where the scene has its robot, by process flange id."""
    robot = scene.robots[live]
    return {f"{planned}/{arm}_tool0": link_pose(robot.model.urdf, robot.base, robot.joints, f"{arm}_tool0")
            for arm in arms}


def moved(poses: dict[str, Pose], offset: tuple[float, float, float]) -> dict[str, Pose]:
    """Every pose moved by `offset`, metres, turned the same."""
    return {flange: Pose.from_arrays(np.add(pose.position, offset), pose.orientation) for flange, pose in poses.items()}


def reaches(answer, targets: dict[str, Pose]) -> None:
    """The route's last row puts every flange of the targets within END_TOL of them."""
    assert set(answer.route.end_error) == set(targets)
    for flange, (distance, angle) in answer.route.end_error.items():
        assert distance <= END_TOL[0] and angle <= END_TOL[1], (flange, distance, angle)


def cindy_held(purged, configs, mirrors, snapshot: Scene, targets: dict[str, Pose], **ask):
    """A held servo pass of Cindy at B20's insert, from the snapshot, to the targets."""
    return plan_servo_step(mirror_of(mirrors, CINDY_ID), purged, step(purged, "B20_J_insert"), snapshot, configs,
                           ServoAsk(targets, STOP_BELOW, ask.get("linear_only", False), True),
                           transfer_as_free=True, log=lambda _line: None)


CINDY_ARMS = ("left_ur_arm", "right_ur_arm")


def test_held_correction_at_the_insert_start(purged, configs, mirrors):
    """Cindy 10 mm off her base at the insert's start: a straight bar-held line onto the transfer's authored targets.

    IK at the live base; 10 mm at both flanges; the live joints are the first row.
    """
    snapshot = shifted(design_scene(purged, "B20_J_insert"), CINDY_ID, (0.01, 0.0, 0.0))
    targets = dict(movement(purged, "B20_J_transfer")[1].target.link_poses)
    answer = cindy_held(purged, configs, mirrors, snapshot, targets)
    route = answer.route
    assert answer.ok and route.ok and not answer.nothing_left, route.result.reason
    assert route.planner == LINEAR and route.attempts == 1 and route.ik.backend == "ssik"
    assert set(answer.correction) == set(flanges(snapshot, CINDY_ID, "robots/cindy", CINDY_ARMS))
    assert all(abs(metres - 0.01) <= 1e-5 for metres in answer.correction.values()), answer.correction
    trajectory = route.result.trajectory
    assert len(trajectory.joint_names) == 12 and route.waypoints == len(trajectory.positions) >= 2
    live = snapshot.robots[CINDY_ID].joints
    assert np.allclose(trajectory.positions[0], [live[name] for name in trajectory.joint_names], atol=1e-9)
    assert route.goal_joints is not None and set(route.goal_joints) == set(trajectory.joint_names)
    reaches(answer, targets)
    assert not any("touches" in warning for warning in route.warnings)


def test_a_correction_under_the_tolerance_plans_nothing(purged, configs, mirrors):
    """Targets 0.1 mm from the live flanges: IK runs, nothing is planned, the step is done ("nothing left")."""
    snapshot = shifted(design_scene(purged, "B20_J_insert"), CINDY_ID, (0.01, 0.0, 0.0))
    targets = moved(flanges(snapshot, CINDY_ID, "robots/cindy", CINDY_ARMS), (0.0001, 0.0, 0.0))
    answer = cindy_held(purged, configs, mirrors, snapshot, targets)
    assert answer.nothing_left and answer.ok and not answer.route.ok
    assert (answer.route.result.status, answer.route.result.reason) == ("not_planned", "nothing left to plan")
    assert answer.route.result.trajectory is None and answer.route.attempts == 0 and answer.route.ik.joints
    assert all(abs(metres - 0.0001) <= 1e-5 for metres in answer.correction.values()), answer.correction


def test_a_correction_at_the_insert_end_plans_straight_on_the_floor(purged, configs, mirrors):
    """The KEY check (spec risk 3): at the insert's end a 1 mm sideways correction plans as a straight bar-held line.

    B20's ground halves stand on the floor, straight lines only, env collisions on.
    """
    snapshot = design_scene(purged, "B20_R_ungrasp")  # Cindy at the insert's end: the bar stands on the floor
    targets = moved(flanges(snapshot, CINDY_ID, "robots/cindy", CINDY_ARMS), (0.0, 0.001, 0.0))
    answer = cindy_held(purged, configs, mirrors, snapshot, targets, linear_only=True)
    route = answer.route
    assert answer.ok and route.ok and route.planner == LINEAR, route.result.reason
    assert route.waypoints >= 2 and route.attempts == 1
    assert all(abs(metres - 0.001) <= 1e-5 for metres in answer.correction.values()), answer.correction
    reaches(answer, targets)


def test_linear_only_refuses_when_there_is_no_line(purged, configs, mirrors, monkeypatch):
    """No straight line: with straight lines only the pass fails saying so, and the sampling search never runs."""
    sampled = []
    monkeypatch.setattr(servo_route, "plan_constrained_dual_arm_linear", lambda *_args, **_options: None)
    monkeypatch.setattr(servo_route, "plan_constrained_dual_arm", lambda *args, **options: sampled.append(1))
    snapshot = shifted(design_scene(purged, "B20_J_insert"), CINDY_ID, (0.01, 0.0, 0.0))
    targets = dict(movement(purged, "B20_J_transfer")[1].target.link_poses)
    answer = cindy_held(purged, configs, mirrors, snapshot, targets, linear_only=True)
    assert not answer.ok and answer.route.planner == LINEAR and sampled == []
    assert answer.route.result.reason == (f"{LINEAR}: no path; straight lines only after a grounded insert: nothing "
                                          f"is planned")
    assert all(abs(metres - 0.01) <= 1e-5 for metres in answer.correction.values())


def test_no_line_falls_back_to_one_sampling_search(purged, configs, mirrors, monkeypatch):
    """No straight line and sampling allowed: one sampling search to the IK goal, seeded 0, its result warned about."""
    calls = []

    def sampling(_planner, state, **options):
        calls.append(options)
        rows = [[float(state.robot_configuration[name]) for name in options["goal_conf"]],
                [float(options["goal_conf"][name]) for name in options["goal_conf"]]]
        return rows, {}

    monkeypatch.setattr(servo_route, "plan_constrained_dual_arm_linear", lambda *_args, **_options: None)
    monkeypatch.setattr(servo_route, "plan_constrained_dual_arm", sampling)
    snapshot = shifted(design_scene(purged, "B20_J_insert"), CINDY_ID, (0.01, 0.0, 0.0))
    targets = dict(movement(purged, "B20_J_transfer")[1].target.link_poses)
    answer = cindy_held(purged, configs, mirrors, snapshot, targets)
    route = answer.route
    assert answer.ok and route.planner == SAMPLING and route.attempts == 2, route.result.reason
    assert len(calls) == 1 and calls[0]["random_seed"] == 0 and calls[0]["active_bar_id"] == "process/bars/B20"
    assert any(warning.startswith("no straight line (") and "sampling search" in warning for warning in route.warnings)
    reaches(answer, targets)


def test_a_missing_target_is_refused(purged, configs, mirrors):
    """Targets without the right flange: refused naming it, before IK."""
    snapshot = design_scene(purged, "B20_J_insert")
    targets = dict(movement(purged, "B20_J_transfer")[1].target.link_poses)
    del targets["robots/cindy/right_ur_arm_tool0"]
    answer = cindy_held(purged, configs, mirrors, snapshot, targets)
    assert not answer.ok and answer.correction == {} and answer.route.ik is None
    assert answer.route.result.reason == "no target for robots/cindy/right_ur_arm_tool0"


def test_one_arm_correction_is_two_rows_swept_checked(purged, configs, mirrors):
    """Alice 10 mm off her base at her approach's end: her own solver's IK, the rows [live, goal], swept clear."""
    snapshot = shifted(design_scene(purged, "B21_H_to_grasp"), ALICE_ID, (0.01, 0.0, 0.0))
    targets = dict(movement(purged, "B21_H_approach")[1].target.link_poses)
    answer = plan_servo_step(mirror_of(mirrors, ALICE_ID), purged, step(purged, "B21_H_approach"), snapshot, configs,
                             ServoAsk(targets, STOP_BELOW, False, False), log=lambda _line: None)
    route = answer.route
    assert answer.ok and route.ok and route.planner == TWO_ROWS, route.result.reason
    assert route.ik.solvers == ("alice",) and route.swept is not None and route.swept.ok and route.waypoints == 2
    trajectory = route.result.trajectory
    assert len(trajectory.joint_names) == 6 and all(name.startswith("ur_arm_") for name in trajectory.joint_names)
    live = snapshot.robots[ALICE_ID].joints
    assert np.allclose(trajectory.positions[0], [live[name] for name in trajectory.joint_names], atol=1e-9)
    assert list(answer.correction) == ["robots/alice/ur_arm_tool0"]
    assert abs(answer.correction["robots/alice/ur_arm_tool0"] - 0.01) <= 1e-5
    reaches(answer, targets)
