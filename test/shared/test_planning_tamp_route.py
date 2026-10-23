"""Tests for shared.assembly_planning.tamp_route: the goal helpers, and every routine planned on the purged process."""

from __future__ import annotations

import math
import re
import threading
from contextlib import suppress
from types import SimpleNamespace

import numpy as np
import pytest
from planning_fixtures import ALICE, BELLE, CINDY, design_scene, movement, purged_process, robot_config, step
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.experiments.assembly_plan_check import perturbed, run_check, select_actions, write_solutions
from husky_assembly_teleop.shared.assembly_planning import route_search, tamp_route
from husky_assembly_teleop.shared.assembly_planning.ik import ssik_unavailable
from husky_assembly_teleop.shared.assembly_planning.movement import Routine, classify
from husky_assembly_teleop.shared.assembly_planning.route_result import aims_at_line, line_goal, pose_error
from husky_assembly_teleop.shared.assembly_planning.tamp_route import check_goal, plan_movement
from husky_assembly_teleop.world.kinematics import stow_joints
from scaffolding_assembly_core.geometry import Pose, box_geometry
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror
from scaffolding_assembly_core.process import LineSpec, read
from scaffolding_assembly_core.process.solutions import is_stale, read_solutions
from scaffolding_assembly_core.scene import Body, Scene

POSE = Pose((0.4, -1.2, 0.9), tuple(Rotation.from_euler("xyz", (0.2, -0.4, 1.1)).as_quat()))


@pytest.fixture(scope="module")
def purged():
    """The purged process."""
    return purged_process()


def test_pose_error():
    """Identical poses and a sign-flipped quaternion are 0 apart; a quarter turn about z is π/2."""
    assert pose_error(POSE, POSE) == pytest.approx((0.0, 0.0), abs=1e-7)
    flipped = Pose(POSE.position, tuple(-value for value in POSE.orientation))
    assert pose_error(POSE, flipped) == pytest.approx((0.0, 0.0), abs=1e-7)
    quarter = Rotation.from_quat(POSE.orientation) * Rotation.from_euler("z", 90, degrees=True)
    turned = Pose((0.4, -1.2, 1.9), tuple(quarter.as_quat()))
    distance, angle = pose_error(POSE, turned)
    assert distance == pytest.approx(1.0) and angle == pytest.approx(math.pi / 2)


def test_line_goal_moves_along_the_line_and_keeps_the_turn():
    """The goal is exactly d·u from the start, turned the same."""
    goal = line_goal(POSE, LineSpec((0.6, -0.8, 0.0), 0.05))
    assert np.allclose(np.subtract(goal.position, POSE.position), (0.03, -0.04, 0.0), atol=1e-15)
    assert goal.orientation == POSE.orientation


def test_aims_at_line(purged):
    """A move into a grasp aims at its target poses; a move leaving the bar it is on aims along its line."""
    for movement_id, along in (("B21_H_to_grasp", False), ("B21_HR_retreat", True)):
        action, moving = movement(purged, movement_id)
        assert aims_at_line(purged.process, moving, classify(purged.process, action, moving)) is along


def test_check_goal_refuses_a_movement_without_an_ik_goal(purged):
    """An insert's goal is its line: refused as not solved by IK, before any mirror or scene is made."""
    check = check_goal(None, purged, step(purged, "B20_J_insert"), Scene(), ())
    assert (check.ok, check.routine, check.reason, check.ik) == (False, Routine.INSERT, "goal not solved by IK", None)


class _Captured(Exception):
    """Raised by a stand-in `scene_for` once it has kept what it was asked."""


@pytest.mark.parametrize("entry", [plan_movement, check_goal])
def test_the_rhino_obstacles_box_reaches_both_scenes(purged, monkeypatch, entry):
    """The workaround's transfer makes a planning scene and a search scene without the bar: both leave them out."""
    acting = robot_id(f"a200-{CINDY}")
    calls = []

    def scene_for(*_args, **options) -> SimpleNamespace:
        calls.append(options)
        if len(calls) == 2:
            raise _Captured()
        return SimpleNamespace(acting=acting, scene=Scene())

    monkeypatch.setattr(tamp_route, "scene_for", scene_for)
    with suppress(_Captured):
        entry(SimpleNamespace(robot_id=acting), purged, step(purged, "B20_J_transfer"), Scene(),
              (robot_config(CINDY),), transfer_as_free=True, obstacles_collide=False)
    asked = [(call["obstacles_collide"], call.get("hide_held", False)) for call in calls]
    assert asked == [(False, False), (False, True)]


# --- --- --- --- --- PLANNED ON THE PURGED PROCESS (slow) --- --- --- --- ---

#: Tolerances: a route solved by ssik reaches its pose within these (metres, radians); a joint goal within JOINT_TOL.
POSE_TOL, JOINT_TOL = (1e-4, 1e-3), 1e-9
CINDY_ID = robot_id(f"a200-{CINDY}")


@pytest.fixture(scope="module")
def configs():
    """The three configured robots."""
    return tuple(robot_config(serial) for serial in (ALICE, BELLE, CINDY))


@pytest.fixture(scope="module")
def mirrors():
    """One mirror per robot for every slow test here; closed at the end."""
    if ssik_unavailable() is not None:
        pytest.skip(ssik_unavailable())
    made = {}
    yield made
    for mirror in made.values():
        mirror.close()


@pytest.fixture(scope="module")
def check(purged, configs, mirrors):
    """`run_check` over the 18 actions with the shared mirrors; each `only` planned once, rows by movement id."""
    done = {}

    def rows(*only: str, **options) -> dict:
        key = (only, tuple(sorted(options.items())))
        if key not in done:
            found = run_check(purged, configs, select_actions(purged, "18"), only=only, mirrors=mirrors,
                              verbose=False, log=lambda _line: None, **options)
            done[key] = {row.movement: row for row in found}
        return done[key]
    return rows


def _flanges(loaded, row) -> dict[str, tuple[Pose, Pose]]:
    """Each moving flange's world pose at the start and end of a row's path, at its robot's planned base."""
    route, robot = row.route, loaded.process.robots
    planned, = route.result.end
    config = robot_config(robot[planned].serial)
    trajectory = route.result.trajectory
    base = perturbed(movement(loaded, row.movement)[1].start.robots[planned].base, row.offset)
    arms = [part.arm for part in classify(loaded.process, *movement(loaded, row.movement)).arms]
    first, last = (dict(zip(trajectory.joint_names, trajectory.positions[index])) for index in (0, -1))
    return {f"{planned}/{arm}_tool0": tuple(link_pose(config.model.urdf, base, joints, f"{arm}_tool0")
                                            for joints in (first, last)) for arm in arms}


def _reaches(loaded, row) -> None:
    """The path ends on the movement's target link poses within POSE_TOL, each flange."""
    targets = movement(loaded, row.movement)[1].target.link_poses
    for flange, (_start, end) in _flanges(loaded, row).items():
        distance, angle = pose_error(end, targets[flange])
        assert distance <= POSE_TOL[0] and angle <= POSE_TOL[1], (row.movement, flange, distance, angle)


@pytest.mark.slow
def test_transfer_without_the_bar(purged, configs, check):
    """B20's transfer, the workaround: from the stowed arms to the insertion start, by ssik IK, swept clear."""
    row = check("B20_J_transfer", "B20_J_insert")["B20_J_transfer"]
    route = row.route
    assert route.ok and route.routine is Routine.TRANSFER_FREE and route.planner == "tamp.plan_free_dual_arm"
    assert route.ik.backend == "ssik" and route.swept.ok
    trajectory = route.result.trajectory
    assert len(trajectory.joint_names) == 12
    stowed = stow_joints(robot_config(CINDY), trajectory.joint_names)
    assert np.allclose(trajectory.positions[0], [stowed[name] for name in trajectory.joint_names], atol=JOINT_TOL)
    _reaches(purged, row)


@pytest.mark.slow
def test_insert_chained_on_the_transfer(purged, check):
    """B20's insert starts exactly where the transfer ended and ends on its target poses in a few steps."""
    rows = check("B20_J_transfer", "B20_J_insert")
    transfer, insert = rows["B20_J_transfer"].route.result, rows["B20_J_insert"].route
    assert insert.ok and insert.routine is Routine.INSERT
    start = insert.result.start["robots/cindy"]
    assert max(abs(start[name] - value) for name, value in transfer.end["robots/cindy"].items()) <= JOINT_TOL
    assert 2 <= insert.waypoints <= 10
    _reaches(purged, rows["B20_J_insert"])


@pytest.mark.slow
def test_retreat_moves_each_flange_along_its_line(purged, check):
    """B20's retreat: each flange 0.05 m along (0, -1, 0), turned the same."""
    row = check("B20_R_retreat")["B20_R_retreat"]
    assert row.route.ok and row.route.routine is Routine.RETREAT_LINEAR
    for flange, (start, end) in _flanges(purged, row).items():
        assert np.allclose(np.subtract(end.position, start.position), (0.0, -0.05, 0.0), atol=1e-4), flange
        assert pose_error(start, end)[1] <= 1e-3


@pytest.mark.slow
def test_home_ends_on_the_target_joints(purged, check):
    """B20's way home ends exactly on the process's target joints, within 3 tries, swept clear."""
    route = check("B20_R_home")["B20_R_home"].route
    assert route.ok and route.routine is Routine.FREE_HOME and route.attempts <= 3 and route.swept.ok
    target = movement(purged, "B20_R_home")[1].target.joints["robots/cindy"]
    end = route.result.end["robots/cindy"]
    assert max(abs(end[name] - value) for name, value in target.items()) <= JOINT_TOL
    assert route.joint_error <= JOINT_TOL


@pytest.mark.slow
def test_support_approach_and_grasp(purged, check):
    """Alice's approach solved by her own ssik solver; the line to the grasp in 21 ± 1 steps onto its targets."""
    rows = check("B21_H_approach", "B21_H_to_grasp")
    approach, grasp = rows["B21_H_approach"], rows["B21_H_to_grasp"]
    assert approach.route.ok and approach.route.ik.solvers == ("alice",)
    _reaches(purged, approach)
    assert grasp.route.ok and grasp.route.planner == "single_arm.plan_linear/ssik"
    assert abs(grasp.route.waypoints - 21) <= 1
    _reaches(purged, grasp)


@pytest.mark.slow
def test_leave_without_the_chain_stows_the_arm(purged, check):
    """B21_HR_leave alone starts at its authored joints and ends on Alice's stowed arm."""
    route = check("B21_HR_leave")["B21_HR_leave"].route
    assert route.ok and route.routine is Routine.SUPPORT_LEAVE
    authored = movement(purged, "B21_HR_leave")[1].start.robots["robots/alice"].joints
    assert route.result.start["robots/alice"] == {name: authored[name] for name in route.result.start["robots/alice"]}
    stowed = stow_joints(robot_config(ALICE), route.result.end["robots/alice"])
    assert max(abs(route.result.end["robots/alice"][name] - value) for name, value in stowed.items()) <= JOINT_TOL


@pytest.mark.slow
def test_a_box_at_home_fails_the_way_home(purged, configs, mirrors):
    """A box where home puts Cindy's left tool0: the way home fails at its first try, naming the box and Cindy.

    A 0.3 m box also covers her upper arm at the start (the retreat's end): refused before any try.
    """
    home = movement(purged, "B20_R_home")[1]
    cindy = home.start.robots["robots/cindy"]
    wrist = link_pose(robot_config(CINDY).model.urdf, cindy.base, {**cindy.joints, **home.target.joints[
        "robots/cindy"]}, "left_ur_arm_tool0")

    def way_home(size: float):
        box = Body("obstacles/box", box_geometry((size, size, size)), wrist)
        row, = run_check(purged, configs, ["B20_R"], only=["B20_R_home"], extra_bodies=(box,), mirrors=mirrors,
                         verbose=False, log=lambda _line: None)
        assert row.status == "failed" and any(
            "obstacles/box" in pair and any(side.startswith(("robots/cindy/", "tools/cindy/")) for side in pair)
            for pair in row.route.colliding)
        return row
    at_goal = way_home(0.1)
    assert at_goal.route.attempts == 1 and "the goal collides" in at_goal.reason
    at_start = way_home(0.3)
    assert at_start.route.attempts == 0 and "the start collides" in at_start.reason


@pytest.mark.slow
def test_perturbed_bases(purged, check):
    """Alice 2 cm / 2° off her design base: the approach and the grasp still end on their world targets."""
    rows = check("B21_H_approach", "B21_H_to_grasp", perturb=(0.02, 2.0), seed=0)
    for row in rows.values():
        assert row.route.ok and any(row.offset)
        _reaches(purged, row)


@pytest.mark.slow
def test_write_solutions_into_a_copy(purged, check, tmp_path):
    """Solutions go into a copy of the process, which reads them back fresh; the original gets none."""
    rows = list(check("B21_H_approach", "B21_H_to_grasp").values())
    copy = write_solutions(purged, rows, tmp_path, {"seed": 0})
    assert not (purged.source / "solutions").exists()
    copied = read(copy)
    solutions = read_solutions(copied)
    assert set(solutions) == {"B21_H"} and not is_stale(copied, solutions["B21_H"])
    assert solutions["B21_H"].movements["B21_H_to_grasp"].status == "solved"
    with pytest.raises(ValueError, match="inside the process"):
        write_solutions(purged, rows, purged.source / "out", {})


def _cindys(mirrors: dict) -> CompasFabMirror:
    """Cindy's mirror of the shared set, made on first use."""
    return mirrors.setdefault(CINDY_ID, CompasFabMirror(CINDY_ID))


@pytest.mark.slow
def test_an_insert_out_of_reach_names_its_waypoint(purged, configs, mirrors):
    """B37's insert with Cindy 4.6/−4.7 mm and +1.99° off her base: the left target is out of reach, at a waypoint."""
    snapshot = design_scene(purged, "B37_J_insert")
    cindy = snapshot.robots[CINDY_ID]
    cindy.base = perturbed(cindy.base, (0.0046, -0.0047, 0.0347))
    route = plan_movement(_cindys(mirrors), purged, step(purged, "B37_J_insert"), snapshot, configs,
                          transfer_as_free=True, log=lambda _line: None)
    assert route.result.status == "failed" and route.routine is Routine.INSERT and route.attempts == 1
    assert re.search(r"left_unreachable at waypoint \d+/3$", route.result.reason), route.result.reason


@pytest.mark.slow
def test_a_retreat_into_a_box_names_the_waypoint_and_the_pair(purged, configs, mirrors):
    """A 2 cm box behind Cindy's left tool, clear at the start: the retreat stops at the step that reaches it."""
    cindy = movement(purged, "B20_R_retreat")[1].start.robots["robots/cindy"]
    tool0 = link_pose(robot_config(CINDY).model.urdf, cindy.base, cindy.joints, "left_ur_arm_tool0")
    behind = Pose(tuple(np.add(tool0.position, (0.0, -0.08, -0.06))))
    box = Body("obstacles/box", box_geometry((0.02, 0.02, 0.02)), behind)
    row, = run_check(purged, configs, ["B20_R"], only=["B20_R_retreat"], extra_bodies=(box,), mirrors=mirrors,
                     verbose=False, log=lambda _line: None)
    found = re.search(r"no_candidate at waypoint (\d+)/11, the nearest step collides: (.*)$", row.reason)
    assert row.status == "failed" and found and int(found.group(1)) > 1, row.reason
    assert ("tools/cindy/AT3L", "obstacles/box") in row.route.colliding
    assert not any("touches" in warning for warning in row.route.warnings)


@pytest.mark.slow
def test_an_error_fails_closed_and_syncs_the_mirror_again(purged, configs, mirrors, monkeypatch):
    """An error inside tamp's borrow: the way home fails with the error's repr, and the mirror is synced after it."""
    mirror, events = _cindys(mirrors), []
    real_sync = mirror.sync

    def boom(*_args, **_options):
        events.append("raised")
        raise RuntimeError("boom")

    def sync(scene):
        events.append("sync")
        real_sync(scene)

    monkeypatch.setattr(route_search, "plan_free_dual_arm", boom)
    monkeypatch.setattr(mirror, "sync", sync)
    row, = run_check(purged, configs, ["B20_R"], only=["B20_R_home"], mirrors=mirrors, verbose=False,
                     log=lambda _line: None)
    assert row.status == "failed" and "RuntimeError('boom')" in row.reason
    assert "raised" in events and events[-1] == "sync"


@pytest.mark.slow
def test_abort_cancels_before_a_try(purged, configs, mirrors):
    """With abort set, the way home stops before its first try: failed, "cancelled", no tries."""
    abort = threading.Event()
    abort.set()
    route = plan_movement(_cindys(mirrors), purged, step(purged, "B20_R_home"), design_scene(purged, "B20_R_home"),
                          configs, abort=abort, log=lambda _line: None)
    assert (route.result.status, route.result.reason, route.attempts) == ("failed", "cancelled", 0)


@pytest.mark.slow
def test_the_workaround_searches_without_the_bar(purged, configs, mirrors, monkeypatch):
    """B20's transfer: the goal is solved with the bar held, the search runs with it and its halves hidden."""
    bar = "process/bars/B20"
    held = [bar, *(f"process/{half}" for half in purged.process.halves_of("bars/B20"))]
    real_ik, seen = tamp_route.solve_ik, {}

    def solve(mirror, robot, targets, **options):
        state = mirror.state.rigid_body_states[bar]
        seen["ik"] = (state.is_hidden, state.attached_to_link)
        return real_ik(mirror, robot, targets, **options)

    def search(_planner, start_state, _goal, **_settings):
        seen["search"] = [start_state.rigid_body_states[body].is_hidden for body in held]
        return None, {"failure_reason": "birrt_failed"}

    monkeypatch.setattr(tamp_route, "solve_ik", solve)
    monkeypatch.setattr(route_search, "plan_free_dual_arm", search)
    row, = run_check(purged, configs, ["B20_J"], only=["B20_J_transfer"], mirrors=mirrors, verbose=False,
                     log=lambda _line: None)
    assert seen["ik"] == (False, "left_ur_arm_tool0")
    assert len(seen["search"]) > 1 and all(seen["search"])
    assert row.status == "failed" and row.route.attempts == 3 and row.route.ik.joints is not None


@pytest.mark.slow
def test_check_goal_of_a_support_approach(purged, configs, mirrors):
    """Alice's approach: her goal by her own ssik solver at her base, reaching the target pose, nothing planned."""
    alice = robot_id(f"a200-{ALICE}")
    mirror = mirrors.setdefault(alice, CompasFabMirror(alice))
    check = check_goal(mirror, purged, step(purged, "B21_H_approach"), design_scene(purged, "B21_H_approach"), configs,
                       log=lambda _line: None)
    assert check.ok and check.routine is Routine.SUPPORT_APPROACH and check.acting == alice, check.reason
    assert check.ik.solvers == ("alice",) and check.ik.backend == "ssik"
    assert check.goal_joints == check.ik.joints and set(check.authored) == set(check.goal_joints)
    target, = movement(purged, "B21_H_approach")[1].target.link_poses.values()
    reached = link_pose(robot_config(ALICE).model.urdf, check.base, check.goal_joints, "ur_arm_tool0")
    assert pose_error(reached, target)[0] <= POSE_TOL[0] and pose_error(reached, target)[1] <= POSE_TOL[1]


@pytest.mark.slow
def test_a_support_goal_outside_the_urdf_limits_is_refused(purged, configs):
    """Alice's approach to given joints with the pan past its limit: refused before any search, naming the joint."""
    alice = robot_id(f"a200-{ALICE}")
    mirror = CompasFabMirror(alice)
    snapshot = design_scene(purged, "B21_H_approach")
    goal = {**snapshot.robots[alice].joints, "ur_arm_shoulder_pan_joint": 7.0}
    try:
        route = plan_movement(mirror, purged, step(purged, "B21_H_approach"), snapshot, configs, goal_joints=goal,
                              log=lambda _line: None)
    finally:
        mirror.close()
    assert route.result.status == "failed" and route.attempts == 0, route.result.reason
    assert "outside the URDF limits: goal ur_arm_shoulder_pan_joint=+7.000" in route.result.reason
