"""The servo's math on hand-made poses, and its plan and carried goal on the purged process (Drive; skipped without).

Every formula of spec §0 to 1e-12 (or the float rounding of the inputs): the errors, the base drift, the excursion
and timing, the bar's deviation at its START, each gate at its threshold, the stop tests in the old order, the
carried insert goal and the insert-end gap.
"""

from __future__ import annotations

from dataclasses import replace
from math import radians, sin, sqrt

import numpy as np
import pytest
from planning_fixtures import design_scene, purged_process
from plugin_fakes import fake_arm, fake_node, fake_robot
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names
from husky_assembly_teleop.shared.execution.servo import (
    AFTER_INSERT, APPROACH, DONE, FAILED, MAX_FORCE_CHANGE, MIN_TIME, TRANSFER, ServoPlan, bar_on_target, bar_reading,
    bar_targets, base_drift, carried_insert_goals, deviation_text, excursion, flange_poses, flange_side, force_vectors,
    insert_end_gap, per_axis_angles_deg, recommended_time, servo_kind, servo_plan, stop_after_move, tool0_errors,
    tracking_gaps, wrench_magnitudes)
from husky_assembly_teleop.shared.runs.runs import RunSettings
from husky_assembly_teleop.world.bar_fit import BarFit
from husky_assembly_teleop.world.bar_pose import apply_authored_move, bar_start, pose_deviation, transform_points
from husky_assembly_teleop.world.marker_layouts import BarLayout, Sensed, bar_layouts, fit_bar
from scaffolding_assembly_core.geometry import Pose, compose, invert
from scaffolding_assembly_core.kinematics import link_pose

pytestmark = pytest.mark.safety

LEFT, RIGHT = "robots/cindy/left_ur_arm_tool0", "robots/cindy/right_ur_arm_tool0"
ALICE_ARM = "robots/alice/ur_arm_tool0"
CINDY = "robots/a200-0806"
ARMS = ("left_ur_arm", "right_ur_arm")
LENGTH = 1.8
#: B20's grasp in its left tool (the process's), the bar pose in the flange frame.
GRASP = Pose((-0.495, 0.0, 0.08), (0.0, -0.7071067811865476, 0.0, -0.7071067811865476))
#: Eight markers along a bar in the process frame (origin at the MIDDLE), no two-fold symmetry: B20's table.
POINTS = np.array([
    (0.02727, 0.02214, 0.2453), (0.02358, -0.02602, 0.2453), (-0.0255, 0.02415, -0.48), (-0.0255, -0.02415, -0.48),
    (-0.02735, -0.02203, -0.8151), (-0.02348, 0.02612, -0.8151), (-0.0255, -0.02415, 0.78), (-0.0255, 0.02415, 0.78)])
#: A layout a half turn about the bar's axis maps onto itself: two marker assignments fit equally well.
SYMMETRIC = np.array([(0.025, 0.024, z) for z in (-0.8, -0.5, 0.25, 0.78)]
                     + [(-0.025, -0.024, z) for z in (-0.8, -0.5, 0.25, 0.78)])


# * ------------------------------------------------- helpers

def pose(x: float = 0.0, y: float = 0.0, z: float = 0.0, rotvec_deg: tuple = (0.0, 0.0, 0.0)) -> Pose:
    """A pose at (x, y, z) turned by a rotation vector given in degrees."""
    return Pose((x, y, z), tuple(Rotation.from_rotvec(np.radians(rotvec_deg)).as_quat()))


def layout(points: np.ndarray = POINTS) -> BarLayout:
    """A B20 layout of `points`."""
    return BarLayout("bars/B20", tuple(f"m{index}" for index in range(len(points))), points, LENGTH)


def held_plan(**changes) -> ServoPlan:
    """A transfer's servo steering on the bar: Cindy's flanges 1 m apart, the bar in the left tool."""
    targets = {LEFT: pose(1.0, 0.0, 1.0), RIGHT: pose(1.0, -1.0, 1.0)}
    plan = ServoPlan(TRANSFER, "B20_J", 3, "B20_J_transfer", 4, (LEFT, RIGHT), ARMS, targets, True, "bars/B20",
                     (LEFT, GRASP), layout(), compose(targets[LEFT], GRASP), True, True, "", False, None)
    return replace(plan, **changes)


def good_fit(T: np.ndarray, rms: float = 0.001) -> BarFit:
    """A fit of all eight markers at `T` with the given rms."""
    return BarFit(True, "", "bars/B20", np.asarray(T, dtype=float), rms_m=rms, n_matched=8, n_design=8)


def sensed(T: np.ndarray, points: np.ndarray = POINTS, count: int = 8) -> Sensed:
    """The first `count` markers of `points` placed at `T`, exact."""
    return Sensed(transform_points(T, points[:count]), tuple(range(1, count + 1)), None, 0.0, "asset")


def shifted(T: np.ndarray, dx: float = 0.0, turn_deg: float = 0.0) -> np.ndarray:
    """`T` turned about its own x axis THROUGH THE BAR START (the start stays put), then moved `dx` along world x."""
    start = np.eye(4)
    start[2, 3] = -LENGTH / 2
    turn = np.eye(4)
    turn[:3, :3] = Rotation.from_rotvec([radians(turn_deg), 0.0, 0.0]).as_matrix()
    moved = T @ start @ turn @ np.linalg.inv(start)
    moved[0, 3] += dx
    return moved


def reading_with(tool0_mm: tuple = (0.1, 0.1), bar: dict | None = None, force: dict | None = None) -> dict:
    """A reading with the given tool0 position errors by side, a bar reading and force changes when given."""
    reading = {"iter": 1, "tool0": {side: {"pos_norm_mm": mm} for side, mm in zip(("left", "right"), tool0_mm)}}
    if bar is not None:
        reading["bar"] = bar
    if force is not None:
        reading["force_change_N"] = force
    return reading


def bar_dict(dpos_mm: float = 0.1, dang_deg: float = 0.01, usable: bool = True, reason: str = "") -> dict:
    """The part of a bar reading the stop tests read."""
    return {"usable": usable, "reason": reason,
            "mocap_vs_design": {"dpos_mm": dpos_mm, "dang_deg": dang_deg} if usable else None}


# * ------------------------------------------------- one reading

def test_per_axis_angles():
    """Each world axis' angle between two orientations: a 10 deg yaw turns x and y by 10 deg, z not at all."""
    assert per_axis_angles_deg((0, 0, 0, 1), (0, 0, 0, 1)) == [0.0, 0.0, 0.0]
    turned = pose(rotvec_deg=(0.0, 0.0, 10.0)).orientation
    assert np.allclose(per_axis_angles_deg((0, 0, 0, 1), turned), [10.0, 10.0, 0.0], atol=1e-12)


def test_tool0_errors_by_side():
    """Position error per axis and its norm in mm, the per-axis angles in degrees, keyed by the flange's side."""
    targets = {LEFT: pose(1.0, 0.0, 1.0), RIGHT: pose(1.0, -1.0, 1.0, rotvec_deg=(0.0, 0.0, 90.0))}
    flanges = {LEFT: pose(1.001, 0.002, 1.003), RIGHT: pose(1.0, -1.0, 1.0, rotvec_deg=(0.0, 0.0, 91.0))}
    errors = tool0_errors(flanges, targets)
    assert list(errors) == ["left", "right"]
    assert np.allclose(errors["left"]["pos_err_mm"], [1.0, 2.0, 3.0], atol=1e-9)
    assert abs(errors["left"]["pos_norm_mm"] - sqrt(14.0)) < 1e-9
    assert np.allclose(errors["right"]["rot_err_deg"], [1.0, 1.0, 0.0], atol=1e-9)
    assert errors["right"]["pos_norm_mm"] < 1e-12
    assert errors["right"]["observed_pose"] == (flanges[RIGHT].position, flanges[RIGHT].orientation)
    assert flange_side(ALICE_ARM) == "arm"


def test_base_drift():
    """The base 2 mm along x and 1 deg about z from the start: the drift per axis and its norm."""
    drift = base_drift(pose(0.002, 0.0, 0.0, rotvec_deg=(0.0, 0.0, 1.0)), pose())
    assert np.allclose(drift["pos_diff_mm"], [2.0, 0.0, 0.0], atol=1e-9) and abs(drift["pos_norm_mm"] - 2.0) < 1e-9
    assert np.allclose(drift["rot_diff_deg"], [1.0, 1.0, 0.0], atol=1e-9)


def test_flange_poses_are_the_live_fk_by_link_name():
    """A process flange id names the live robot's link; at the design pose the FK meets the authored target."""
    loaded = purged_process()
    cindy = design_scene(loaded, "B20_J_insert").robots[CINDY]
    poses = flange_poses(cindy, (LEFT, RIGHT))
    assert poses[LEFT] == link_pose(cindy.model.urdf, cindy.base, cindy.joints, "left_ur_arm_tool0")
    authored = loaded.process.actions["B20_J"].movements[3].target.robots["robots/cindy"].link_poses
    for flange in (LEFT, RIGHT):
        assert np.linalg.norm(np.subtract(poses[flange].position, authored[flange].position)) < 1e-6


def test_wrench_magnitudes_and_force_vectors_by_side():
    """None while any arm has no F/T reading (the guard off); else magnitudes and the force vectors by side."""
    node = fake_node(lambda: 10.0, [])
    arms = [fake_arm(node, name, [0.0] * 6) for name in ARMS]
    robot = fake_robot(node, "0806", arms)
    assert wrench_magnitudes(robot, ARMS) is None and force_vectors(robot, ARMS) is None
    arms[0].state.wrench = np.array([3.0, 4.0, 0.0, 0.0, 0.0, 2.0])
    assert force_vectors(robot, ARMS) is None
    arms[1].state.wrench = np.array([0.0, 0.0, 1.0, 1.0, 0.0, 0.0])
    assert wrench_magnitudes(robot, ARMS) == {"left": {"force_N": 5.0, "torque_Nm": 2.0},
                                              "right": {"force_N": 1.0, "torque_Nm": 1.0}}
    vectors = force_vectors(robot, ARMS)
    assert np.array_equal(vectors["left"], [3.0, 4.0, 0.0]) and np.array_equal(vectors["right"], [0.0, 0.0, 1.0])


# * ------------------------------------------------- the move between passes

def test_excursion_reads_the_whole_path():
    """An out-and-back path has no net travel but a full excursion; an empty path none."""
    rest = np.zeros(12)
    out = rest.copy()
    out[2] = 0.3
    assert excursion([rest, out, rest]) == (3, 0.0, 0.3)
    assert excursion([rest, out]) == (2, 0.3, 0.3)
    assert excursion([]) == (0, 0.0, 0.0)


@pytest.mark.parametrize("travel, seconds", [(radians(3.0), MIN_TIME), (radians(60.0), 20.0), (radians(600.0), 90.0)])
def test_recommended_time_clamps(travel: float, seconds: float):
    """3 deg per second, at least 3 s, at most the cap."""
    assert abs(recommended_time(travel, 90.0) - seconds) < 1e-9


# * ------------------------------------------------- the held bar by MoCap

def test_bar_reading_measures_at_the_bar_start():
    """A 0.1 deg turn about the bar's MIDDLE moves its START by 1.57 mm: the deviation is the old frame's."""
    plan = held_plan()
    flanges = dict(plan.targets)
    T_design = plan.design_bar.matrix()
    turn = np.eye(4)
    turn[:3, :3] = Rotation.from_rotvec([0.0, radians(0.1), 0.0]).as_matrix()
    T_mocap = T_design @ turn
    reading = bar_reading(plan, flanges, good_fit(T_mocap), "asset", plan.design_bar)
    assert reading["usable"] and reading["reason"] == ""
    # The old monitor's bar frame sat at the START (T . Trans(0, 0, -L/2)): its deviation, unchanged.
    old_mocap, old_design = bar_start(T_mocap, LENGTH), bar_start(T_design, LENGTH)
    assert reading["mocap_vs_design"] == pose_deviation(old_mocap, old_design).to_json()
    assert abs(reading["mocap_vs_design"]["dpos_mm"] - 2.0 * (LENGTH / 2) * sin(radians(0.05)) * 1e3) < 1e-9
    assert abs(reading["mocap_vs_design"]["dang_deg"] - 0.1) < 1e-9
    assert np.allclose(np.asarray(reading["T_corr"]) @ T_mocap, T_design, atol=1e-12)
    assert np.allclose(reading["T_fk"], T_design, atol=1e-12)
    assert reading["fk_vs_design"]["dpos_mm"] < 1e-9 and reading["mocap_vs_fk"] == reading["mocap_vs_design"]
    assert reading["bar"] == "bars/B20" and reading["grounded"] and reading["fit"]["ok"]
    assert deviation_text(reading["mocap_vs_design"]) == "1.57 mm/0.100°" and deviation_text(None) == "n/a"


@pytest.mark.parametrize("fit, source, usable, words", [
    (lambda T: good_fit(T, rms=0.0029), "asset", True, ""),
    (lambda T: good_fit(T, rms=0.0031), "asset", False, "fit rms 3.1 mm (not under 3 mm)"),
    (lambda T: good_fit(shifted(T, dx=0.029)), "asset", True, ""),
    (lambda T: good_fit(shifted(T, dx=0.031)), "asset", False, "beyond the 30 mm / 3 deg gate"),
    (lambda T: good_fit(shifted(T, turn_deg=2.9)), "asset", True, ""),
    (lambda T: good_fit(shifted(T, turn_deg=3.1)), "asset", False, "beyond the 30 mm / 3 deg gate"),
    (lambda T: good_fit(T), "fake", False, "fake markers"),
    (lambda T: good_fit(T), None, False, "unknown"),
    (lambda T: None, "asset", False, "could not be measured"),
    (lambda T: fit_bar(sensed(T, SYMMETRIC), layout(SYMMETRIC)), "asset", False,
     "fit failed: two different marker assignments fit equally well"),
    (lambda T: fit_bar(sensed(T, count=5), layout()), "asset", False, "fit failed: only 5 of 8"),
])
def test_bar_reading_gates_at_their_thresholds(fit, source: str | None, usable: bool, words: str):
    """The rms, the 30 mm / 3 deg gate, fake or unknown markers, no sample, an ambiguous or a thin fit."""
    plan = held_plan()
    reading = bar_reading(plan, dict(plan.targets), fit(plan.design_bar.matrix()), source, plan.design_bar)
    assert reading["usable"] is usable and words in reading["reason"]


def test_bar_targets_keep_the_flange_to_flange_transform():
    """Both flanges move by the same rigid correction, so the grasp actually held is kept."""
    flanges = {LEFT: pose(1.002, 0.001, 0.999, rotvec_deg=(0.1, 0.0, 0.0)),
               RIGHT: pose(1.0, -1.003, 1.001, rotvec_deg=(0.0, 0.2, 0.0))}
    T_corr = pose(0.003, -0.002, 0.001, rotvec_deg=(0.0, 0.0, 0.3)).matrix()
    targets = bar_targets(T_corr, flanges)
    for flange in (LEFT, RIGHT):
        assert np.allclose(targets[flange].matrix(), T_corr @ flanges[flange].matrix(), atol=1e-12)
    before = compose(invert(flanges[LEFT]), flanges[RIGHT]).matrix()
    after = compose(invert(targets[LEFT]), targets[RIGHT]).matrix()
    assert np.allclose(before, after, atol=1e-12)


def test_bar_on_target_at_the_tolerance():
    """Under 0.5 mm and 0.1 deg, and usable."""
    assert bar_on_target(bar_dict(0.49, 0.09))
    assert not bar_on_target(bar_dict(0.5, 0.09)) and not bar_on_target(bar_dict(0.49, 0.1))
    assert not bar_on_target(bar_dict(usable=False, reason="fit failed"))


# * ------------------------------------------------- the stop tests

AFTER = held_plan(kind=AFTER_INSERT, linear_only=True, max_force_change=MAX_FORCE_CHANGE)
TOOL0 = held_plan(by_bar=False, why_tool0="'Servo by bar' is off")


@pytest.mark.parametrize("plan, reading, engaged, status, words", [
    # 1. the load guard, after a grounded insert only, beats a converged bar
    (AFTER, reading_with(bar=bar_dict(), force={"left": 20.1, "right": 1.0}), True, FAILED, "changed by 20.1 N"),
    (AFTER, reading_with(bar=bar_dict(), force={"left": 19.9, "right": 1.0}), True, DONE, "converged on the MoCap bar"),
    (held_plan(), reading_with(bar=bar_dict(), force={"left": 25.0, "right": 1.0}), True, DONE, "on the MoCap bar"),
    # 2. steering on the bar: its fit lost, converged at the tolerance, or not yet (tool0 does not count, D2)
    (held_plan(), reading_with(bar=bar_dict(usable=False, reason="fit rms 3.1 mm")), True, FAILED,
     "fit is lost (fit rms 3.1 mm)"),
    (held_plan(), reading_with(), True, FAILED, "fit is lost (not measured)"),
    (held_plan(), reading_with(bar=bar_dict(0.49, 0.09)), True, DONE, "dpos 0.49 mm, dang 0.090 deg"),
    (held_plan(), reading_with(tool0_mm=(0.1, 0.1), bar=bar_dict(0.51, 0.09)), True, None, ""),
    (held_plan(), reading_with(tool0_mm=(0.1, 0.1), bar=bar_dict(0.49, 0.11)), True, None, ""),
    # 3. tool0: the fit never usable, or by tool0 from the start
    (held_plan(), reading_with(tool0_mm=(0.19, 0.1), bar=bar_dict(usable=False)), False, DONE, "aimed tool0"),
    (TOOL0, reading_with(tool0_mm=(0.19, 0.1)), False, DONE, "converged on tool0 (< 0.2 mm)"),
    (TOOL0, reading_with(tool0_mm=(0.19, 0.21)), False, None, ""),
])
def test_stop_after_move_in_the_old_order(plan: ServoPlan, reading: dict, engaged: bool, status: str | None,
                                          words: str):
    """Load guard, then the bar (lost or converged), then tool0."""
    stop = stop_after_move(reading, plan, engaged)
    if status is None:
        assert stop is None
    else:
        assert stop[0] == status and words in stop[1]


# * ------------------------------------------------- which servo, and what it aims at (the purged process)

def test_servo_kind_by_routine():
    """A transfer (either routine), a grounded insert and an approach get a servo; a free move or a retreat none."""
    loaded = purged_process()
    assert servo_kind(loaded, "B20_J", 3, True) == TRANSFER and servo_kind(loaded, "B20_J", 3, False) == TRANSFER
    assert servo_kind(loaded, "B20_J", 4, True) == AFTER_INSERT
    assert servo_kind(loaded, "B21_H", 0, True) == APPROACH
    assert servo_kind(loaded, "B20_J", 0, True) is None and servo_kind(loaded, "B20_R", 1, True) is None


def test_servo_plan_after_the_transfer():
    """Aims the transfer's targets in the insert's scene; B* is the assembly pose lifted 15 mm; by the bar if on."""
    loaded = purged_process()
    layouts = bar_layouts(loaded.process)
    plan = servo_plan(loaded, "B20_J", 3, RunSettings(servo_by_bar=True), layouts)
    assert isinstance(plan, ServoPlan)
    assert (plan.kind, plan.action, plan.primary, plan.target) == (TRANSFER, "B20_J", 3, "B20_J_transfer")
    assert plan.step_index == loaded.index_of("B20_J", 4)
    assert loaded.steps[plan.step_index].movement.id == "B20_J_insert"
    assert plan.flanges == (LEFT, RIGHT) and plan.arms == ARMS
    transfer = loaded.process.actions["B20_J"].movements[3]
    assert plan.targets == transfer.target.robots["robots/cindy"].link_poses
    assert plan.held and plan.bar == "bars/B20" and plan.holder[0] == LEFT and plan.grounded
    assert plan.layout is layouts["bars/B20"] and plan.layout.length == LENGTH
    assembly = loaded.process.bodies["bars/B20"].assembly_pose
    assert np.allclose(plan.design_bar.position, np.add(assembly.position, (0.0, 0.0, 0.015)), atol=1e-9)
    assert np.allclose(plan.design_bar.matrix()[:3, :3], assembly.matrix()[:3, :3], atol=1e-9)
    assert plan.by_bar and plan.why_tool0 == "" and not plan.linear_only and plan.max_force_change is None
    off = servo_plan(loaded, "B20_J", 3, RunSettings(servo_by_bar=False), layouts)
    assert not off.by_bar and off.why_tool0 == "'Servo by bar' is off"
    bare = servo_plan(loaded, "B20_J", 3, RunSettings(servo_by_bar=True), {})
    assert not bare.by_bar and bare.why_tool0 == "bars/B20 carries no MoCap markers" and bare.layout is None


def test_servo_plan_after_the_insert_and_the_approach():
    """After the insert: straight lines only under the load guard, B* the assembly pose; the approach: one arm."""
    loaded = purged_process()
    layouts = bar_layouts(loaded.process)
    after = servo_plan(loaded, "B20_J", 4, RunSettings(servo_by_bar=True), layouts)
    assert after.kind == AFTER_INSERT and after.target == "B20_J_insert"
    assert after.step_index == loaded.index_of("B20_J", 4)
    assert after.linear_only and after.max_force_change == MAX_FORCE_CHANGE and after.held and after.by_bar
    assembly = loaded.process.bodies["bars/B20"].assembly_pose
    assert np.allclose(after.design_bar.position, assembly.position, atol=1e-9)
    approach = servo_plan(loaded, "B21_H", 0, RunSettings(servo_by_bar=True), layouts)
    assert approach.kind == APPROACH and approach.flanges == (ALICE_ARM,) and approach.arms == ("ur_arm",)
    assert approach.step_index == loaded.index_of("B21_H", 0) and not approach.held and approach.bar is None
    assert approach.holder is None and approach.design_bar is None and not approach.grounded
    assert not approach.by_bar and approach.why_tool0 == "this loop plans free moves (no bar held)"
    assert approach.max_force_change is None and not approach.linear_only
    assert servo_plan(loaded, "B20_J", 0, RunSettings(), layouts) == "B20_J_load: no servo after it"


# * ------------------------------------------------- carried into the insert, and the gate after it

def test_carried_insert_goals_make_the_authored_move_from_the_live_flanges():
    """On the authored start the goal is the authored end; with the base 10 mm off, the goal moves with the flanges."""
    loaded = purged_process()
    cindy = design_scene(loaded, "B20_J_insert").robots[CINDY]
    action = loaded.process.actions["B20_J"]
    starts = action.movements[3].target.robots["robots/cindy"].link_poses
    ends = action.movements[4].target.robots["robots/cindy"].link_poses
    goals = carried_insert_goals(loaded, "B20_J", 4, cindy)
    for flange in (LEFT, RIGHT):
        assert np.linalg.norm(np.subtract(goals[flange].position, ends[flange].position)) < 1e-6
    cindy.base = Pose(tuple(np.add(cindy.base.position, (0.01, 0.0, 0.0))), cindy.base.orientation)
    live = flange_poses(cindy, (LEFT, RIGHT))
    goals = carried_insert_goals(loaded, "B20_J", 4, cindy)
    for flange in (LEFT, RIGHT):
        expected = apply_authored_move(starts[flange].matrix(), ends[flange].matrix(), live[flange].matrix())
        assert np.allclose(goals[flange].matrix(), expected, atol=1e-9)
        assert abs(np.linalg.norm(np.subtract(goals[flange].position, ends[flange].position)) - 0.01) < 1e-6


def test_insert_end_gap_is_the_flange_gap_to_the_last_row():
    """Zero on the planned end; a left elbow 0.01 rad short gaps the left flange only, by its FK difference."""
    loaded = purged_process()
    cindy = design_scene(loaded, "B20_J_insert").robots[CINDY]
    names = [name for arm in ARMS for name in arm_joint_names(arm)]
    live = [cindy.joints[name] for name in names]
    assert insert_end_gap(cindy, names, live, ARMS) == 0.0
    end = list(live)
    end[2] += 0.01
    moved = {**cindy.joints, names[2]: end[2]}
    expected = np.linalg.norm(np.subtract(
        link_pose(cindy.model.urdf, cindy.base, cindy.joints, "left_ur_arm_tool0").position,
        link_pose(cindy.model.urdf, cindy.base, moved, "left_ur_arm_tool0").position))
    gaps = tracking_gaps(cindy, names, end, ARMS)
    assert abs(gaps["left"] - expected) < 1e-12 and gaps["right"] == 0.0 and expected > 0.003
    assert insert_end_gap(cindy, names, end, ARMS) == gaps["left"]
