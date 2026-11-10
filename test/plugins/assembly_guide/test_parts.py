"""The guide's parts on the purged process: the executor's split with a drive and Mark done, and the tolerances."""

from __future__ import annotations

import math

import pytest
from planning_fixtures import ALICE, BELLE, CINDY, purged_process, robot_config

from husky_assembly_teleop.plugins.assembly_guide.parts import (DRIVE, GRIPPER, HAND, INSERT, MARK, MOVE, PARK,
                                                                RETREAT, SERVO, action_parts, arm_arrived,
                                                                base_arrived)
from scaffolding_assembly_core.geometry import Pose


@pytest.fixture(scope="module")
def loaded():
    """The purged process."""
    return purged_process()


def parts(loaded, action: str):
    """An action's parts with the transfer workaround."""
    return action_parts(loaded, action, tuple(robot_config(serial) for serial in (ALICE, BELLE, CINDY)), True)


def test_a_ground_bar_drives_moves_waits_for_hands_servos_inserts_and_marks(loaded):
    """B20_J: the servo after the transfer on its own, the one after the ground bar's insert with the insert."""
    assert [(part.kind, part.label) for part in parts(loaded, "B20_J")] == [
        (DRIVE, "drive to B20_J's start"), (MOVE, "B20_J_transfer: transfer, empty tools"),
        (HAND, "B20_J_place_bar: by hand"), (SERVO, "B20_J_transfer: servo"),
        (INSERT, "B20_J_insert: insert, then its servo"), (MARK, "mark B20_J done")]


def test_a_hold_and_a_release(loaded):
    """B21_H's gripper steps and straight line; B20_R's retreat with its ungrasp, in one request, and no drive.

    Cindy holds B20 at B20_R's start: she stays at the bar (no drive part), as Alice holding nothing drives for B21_H.
    """
    assert [part.kind for part in parts(loaded, "B21_H")] == [DRIVE, MOVE, SERVO, GRIPPER, MOVE, GRIPPER, MARK]
    release = parts(loaded, "B20_R")
    assert [part.kind for part in release] == [RETREAT, MOVE, MARK]
    assert [plan.kind for plan in release[0].plans] == ["retreat", "with_motion"], "the executor's own order"


def test_each_part_selects_its_movement_and_says_what_finishes_it(loaded):
    """The servo selects its step's primary and is finished as (primary, SERVO_INDEX); the drive goes to the start."""
    b20 = parts(loaded, "B20_J")
    action = loaded.process.actions["B20_J"]
    assert all(action.movements[part.movement_index].id == part.movement for part in b20)
    assert b20[3].done_keys == {(3, -1)} and b20[4].done_keys == {(4, 4), (4, -1)}
    assert b20[2].done_keys == {(3, 1), (3, 2)}
    x, y, yaw = b20[0].base
    assert (round(x, 3), round(y, 3), round(math.degrees(yaw), 1)) == (0.527, -3.067, 89.6)


def test_a_part_is_at_its_end_within_the_followers_tolerance_and_a_degree_per_joint():
    """6 cm and 6° for a base, yaw across ±180°; 1° per joint for an arm."""
    goal = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
    assert arm_arrived([value + math.radians(0.9) for value in goal], goal)
    assert not arm_arrived([value + math.radians(1.1) for value in goal], goal)
    assert not arm_arrived(None, goal)
    half = math.radians(179.0) / 2
    pose = Pose((1.05, 2.0, 0.0), (0.0, 0.0, math.sin(half), math.cos(half)))
    assert base_arrived(pose, (1.0, 2.0, math.radians(-179.0)))
    assert not base_arrived(pose, (1.12, 2.0, math.radians(179.0)))
    assert not base_arrived(pose, (1.0, 2.0, math.radians(172.0)))


def _parked(loaded, action: str):
    return action_parts(loaded, action, tuple(robot_config(serial) for serial in (ALICE, BELLE, CINDY)), True,
                        park=True)


def test_robots_an_action_does_not_use_are_parked_first(loaded):
    """Every robot an action does not use, holding no bar, parks first; with `park` off, none.

    B25_J (Cindy, nothing held): Alice and Belle; B21_H (Alice, Cindy holds): Belle only; B35_R (Cindy, Alice and Belle
    hold): none. Each parks at its park.yaml pose with its arms' stow joints.
    """
    first = _parked(loaded, "B25_J")
    alice, belle = robot_config(ALICE), robot_config(BELLE)
    assert [(part.kind, part.robot) for part in first[:3]] == [(PARK, alice.serial), (PARK, belle.serial), (DRIVE, "")]
    assert first[0].label == "park alice (0804): out of B25_J's way"
    assert first[0].base == tuple(alice.park_pose)
    assert first[0].stow == (("ur_arm", tuple(alice.arms[0].stow_joints)),)
    assert first[0].movement == first[2].movement, "selects the action's first step, as its drive"
    assert [part.robot for part in _parked(loaded, "B21_H") if part.kind == PARK] == [belle.serial]
    assert not [part for part in _parked(loaded, "B35_R") if part.kind == PARK]
    assert not [part for part in parts(loaded, "B25_J") if part.kind == PARK]
