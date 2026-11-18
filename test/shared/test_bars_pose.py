"""Checks for a bar's pose as a matrix: its deviation from the design, its start and ends, the servo's correction."""

from __future__ import annotations

import json

import numpy as np
import pytest
from bar_fixtures import BAR_LENGTH, random_rigid, small_rigid
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.shared.bars.pose import (
    OK_MM, WARN_MM, apply_authored_move, bar_ends, bar_gap, bar_servo_correction, bar_slide_roll, bar_start, gap_color,
    pose_deviation)
from husky_assembly_teleop.colors import FAIL, NONE, OK, WARN
from scaffolding_assembly_core.geometry import Pose


# * ------------------------------------------------- pose deviation, and the bar's start and ends

def test_pose_deviation():
    """5 mm along the bar reads (0, 0, 5) in the bar frame; a 1 deg roll reads 1 deg."""
    T_design = random_rigid(12)
    slid = pose_deviation(T_design @ bar_slide_roll(0.005, 0.0), T_design)
    assert slid.dpos_bar_mm == pytest.approx([0.0, 0.0, 5.0], abs=1e-9)
    assert slid.dpos_world_mm == pytest.approx(T_design[:3, 2] * 5.0, abs=1e-9)
    assert slid.dpos_mm == pytest.approx(5.0) and slid.dang_deg == pytest.approx(0.0, abs=1e-9)

    rolled = pose_deviation(T_design @ bar_slide_roll(0.0, 1.0), T_design)
    assert rolled.dang_deg == pytest.approx(1.0)
    assert rolled.rotvec_bar_deg == pytest.approx([0.0, 0.0, 1.0], abs=1e-9)
    assert rolled.dpos_mm == pytest.approx(0.0, abs=1e-9)

    # A bar flipped end for end (wrong z sign) reads 180 deg.
    flipped = pose_deviation(T_design @ np.diag([1.0, -1.0, -1.0, 1.0]), T_design)
    assert flipped.dang_deg == pytest.approx(180.0)
    json.dumps(flipped.to_json())


def test_deviation_splits_along_and_across_the_axis():
    """3 mm along the planned axis, 4 mm across it, and a 2° tilt, from a planned bar lying along world y."""
    planned = Pose((1.0, 2.0, 0.5), tuple(Rotation.from_euler("x", -90, degrees=True).as_quat()))
    axis, side = planned.matrix()[:3, 2], planned.matrix()[:3, 0]
    tilted = Rotation.from_euler("x", -88, degrees=True).as_quat()
    fitted = Pose(tuple(np.array(planned.position) + 0.003 * axis + 0.004 * side), tuple(tilted))
    deviation = pose_deviation(fitted.matrix(), planned.matrix())
    assert deviation.along_mm == pytest.approx(3.0, abs=1e-6)
    assert deviation.across_mm == pytest.approx(4.0, abs=1e-6)
    assert deviation.tilt_deg == pytest.approx(2.0, abs=1e-6)
    # * A roll about the bar is no tilt.
    assert pose_deviation(fitted.matrix() @ bar_slide_roll(0.0, 5.0), planned.matrix()).tilt_deg == \
        pytest.approx(2.0, abs=1e-6)


def test_bar_ends():
    """The drawn bar line runs L/2 each way from the bar's middle along its z."""
    T = random_rigid(13)
    ends = bar_ends(T, BAR_LENGTH)
    assert ends[0] == pytest.approx(T[:3, 3] - 0.9 * T[:3, 2])
    assert ends[1] == pytest.approx(T[:3, 3] + 0.9 * T[:3, 2])


def test_the_old_origin_is_the_bar_start():
    """Two process-frame poses deviate at `bar_start` exactly as the old-frame poses (origin at the start) did."""
    X_old, Y_old = random_rigid(25), random_rigid(26)
    up = np.eye(4)
    up[2, 3] = BAR_LENGTH / 2.0
    X, Y = X_old @ up, Y_old @ up  # the same bars, their frames moved to the middle
    assert bar_start(X, BAR_LENGTH) == pytest.approx(X_old, abs=1e-12)
    old, new = pose_deviation(X_old, Y_old), pose_deviation(bar_start(X, BAR_LENGTH), bar_start(Y, BAR_LENGTH))
    assert new.dpos_mm == pytest.approx(old.dpos_mm, abs=1e-9)
    assert new.dang_deg == pytest.approx(old.dang_deg, abs=1e-9)
    assert new.dpos_world_mm == pytest.approx(old.dpos_world_mm, abs=1e-9)
    assert new.dpos_bar_mm == pytest.approx(old.dpos_bar_mm, abs=1e-9)
    assert new.rotvec_bar_deg == pytest.approx(old.rotvec_bar_deg, abs=1e-9)
    # At the middle the same tilt moves the origin: the two frames do not agree there.
    assert pose_deviation(X, Y).dpos_mm != pytest.approx(old.dpos_mm, abs=1e-6)


# * ------------------------------------------------- servo on the mocap bar

@pytest.mark.safety
def test_bar_servo_correction_carries_the_measured_bar_onto_the_design():
    """``T_corr @ T_mocap`` is the design pose."""
    T_mocap, T_design = random_rigid(21), random_rigid(22)
    T_corr = bar_servo_correction(T_mocap, T_design)
    assert T_corr @ T_mocap == pytest.approx(T_design, abs=1e-12)


@pytest.mark.safety
def test_apply_authored_move():
    """From the authored start it lands on the authored end; a pure translation stays one."""
    T_start, T_end = random_rigid(31), random_rigid(32)
    assert apply_authored_move(T_start, T_end, T_start) == pytest.approx(T_end, abs=1e-12)

    # The insert: 15 mm straight down in world, from a flange 3 mm / 0.1 deg off the authored start.
    down = np.eye(4)
    down[2, 3] = -0.015
    T_actual = small_rigid(3.0, 0.1, 33) @ T_start
    moved = apply_authored_move(T_start, down @ T_start, T_actual)
    assert moved[:3, :3] == pytest.approx(T_actual[:3, :3], abs=1e-12)
    assert moved[:3, 3] - T_actual[:3, 3] == pytest.approx([0.0, 0.0, -0.015], abs=1e-12)


@pytest.mark.safety
def test_one_servo_step_closes_on_the_bar():
    """One step: flanges moved by ``T_corr`` put the held bar on its design pose, the grasp kept.

    The model (FK) is off by an unknown flange error E per arm (tool mount, grasp): the real flange is ``T_fk @ E``,
    so mocap reads the bar at ``T_fk_left @ E_left @ A``. The IK puts the model flanges on ``T_corr @ T_fk``; the
    real bar then lands on the design.
    """
    T_design = random_rigid(41)
    A = random_rigid(42)                                      # left tool0 -> bar (the grasp)
    E_left, E_right = small_rigid(3.0, 0.1, 43), small_rigid(2.0, 0.2, 44)
    # Where the model puts the flanges now: 20 mm / 1 deg off the design, the right one 1.4 m away.
    T_fk_left = small_rigid(20.0, 1.0, 45) @ T_design @ np.linalg.inv(A)
    T_fk_right = T_fk_left @ random_rigid(46)
    T_mocap = T_fk_left @ E_left @ A
    assert pose_deviation(T_mocap, T_design).dpos_mm > 5.0

    T_corr = bar_servo_correction(T_mocap, T_design)
    new_left, new_right = T_corr @ T_fk_left, T_corr @ T_fk_right
    after = pose_deviation(new_left @ E_left @ A, T_design)
    assert after.dpos_mm == pytest.approx(0.0, abs=1e-9) and after.dang_deg == pytest.approx(0.0, abs=1e-6)
    # The real right flange seen from the real left one: unchanged (the bar is not strained).
    held_before = np.linalg.inv(T_fk_left @ E_left) @ (T_fk_right @ E_right)
    held_after = np.linalg.inv(new_left @ E_left) @ (new_right @ E_right)
    assert held_after == pytest.approx(held_before, abs=1e-9)


def test_bar_gap_measures_the_start_middle_and_end_and_the_tilt():
    """Leaning 1 mrad about the bottom: 0 at the start, half the length mrad in mm at the middle, all at the end."""
    T_design = random_rigid(3)
    # * About the bar's x axis through its start (z = -L/2).
    lean = Rotation.from_rotvec([1e-3, 0.0, 0.0]).as_matrix()
    move = np.eye(4)
    move[:3, :3] = lean
    move[:3, 3] = (np.eye(3) - lean) @ [0.0, 0.0, -BAR_LENGTH / 2]
    gap = bar_gap(T_design @ move, T_design, BAR_LENGTH)
    assert gap.distances_mm == pytest.approx([0.0, BAR_LENGTH / 2, BAR_LENGTH], rel=1e-3, abs=1e-9)
    assert gap.worst_mm == pytest.approx(BAR_LENGTH, rel=1e-3)
    assert gap.design[[0, 2]] == pytest.approx(bar_ends(T_design, BAR_LENGTH))
    assert gap.tilt_deg == pytest.approx(np.degrees(1e-3))
    assert gap.gaps_mm[1] == pytest.approx((gap.measured[1] - gap.design[1]) * 1e3)


def test_gap_color_is_green_amber_red_by_the_shared_limits():
    """Up to OK_MM green, up to WARN_MM amber, beyond red, grey without a gap."""
    assert [gap_color(v) for v in (0.0, OK_MM, OK_MM + 0.1, WARN_MM, WARN_MM + 0.1, None)] == \
        [OK, OK, WARN, WARN, FAIL, NONE]
