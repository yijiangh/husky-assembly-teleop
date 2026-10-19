"""Tests for shared.planning.movement: routines by parts, goals, arm sides and the drives really sent.

The golden table is the purged two-module process (planning_fixtures.PURGED). The old smoke test's
DISPATCH_EXPECTED maps onto it as: J0 load, J1/J2 place_bar/grasp, J3 transfer, J4 tighten (now on the insert),
J5 insert; R0 untighten (now the dropped loosen), R1/R2 ungrasp folded into the retreat, R3 home.
"""

from __future__ import annotations

from dataclasses import replace

import pytest
from planning_fixtures import action, movement, purged_process

from husky_assembly_teleop.shared.planning.movement import (GROUND_HALF, LOOSEN_NEVER_SENT, NO_MATE, ON_NOTHING,
                                                            RECEIVER_NOT_BUILT, GoalSource, Routine, UnknownMovement,
                                                            classify, classify_action, mating_drives, next_start)
from scaffolding_assembly_core.process import ToolState

R = Routine
#: The routines of each action type in the purged process, in movement order.
GOLDEN = {
    "bar_jointing": (R.FREE_TO_LOAD, R.OPERATOR, R.TOOL_ONLY, R.TRANSFER_HELD, R.INSERT),
    "bar_release": (R.TOOL_ONLY, R.RETREAT_LINEAR, R.FREE_HOME),
    "bar_holding": (R.SUPPORT_APPROACH, R.TOOL_ONLY, R.SUPPORT_LINEAR, R.TOOL_ONLY),
    "bar_holding_release": (R.TOOL_ONLY, R.SUPPORT_LINEAR, R.SUPPORT_LEAVE),
}
#: The movement id suffix of each routine, per action type: a cross-check in this test only, never in the code.
SUFFIXES = {
    "bar_jointing": ("load", "place_bar", "grasp", "transfer", "insert"),
    "bar_release": ("ungrasp", "retreat", "home"),
    "bar_holding": ("approach", "open", "to_grasp", "close"),
    "bar_holding_release": ("open", "retreat", "leave"),
}
AT3L, AT3R = "tools/cindy/AT3L", "tools/cindy/AT3R"


@pytest.fixture(scope="module")
def purged():
    """The purged process."""
    return purged_process()


def _first_actions(loaded) -> list:
    """The first 18 actions of the schedule, B20_J to B35_HR: those of bars B20-B22 and B34-B37."""
    return [action(loaded, action_id) for action_id in loaded.process.schedule[:18]]


def _spec(loaded, movement_id: str, **options):
    """The classified movement of one id."""
    return classify(loaded.process, *movement(loaded, movement_id), **options)


def test_golden_table(purged):
    """The 70 movements of B20-B37 classify as the table says, without warnings; ids agree with the routines."""
    actions = _first_actions(purged)
    assert sorted({each.bar for each in actions}) == [f"bars/B{n}" for n in (20, 21, 22, 34, 35, 36, 37)]
    assert sum(len(each.movements) for each in actions) == 70
    for each in actions:
        specs = classify_action(purged.process, each)
        assert tuple(spec.routine for spec in specs) == GOLDEN[each.type], each.id
        assert [spec.movement for spec in specs] == [f"{each.id}_{suffix}" for suffix in SUFFIXES[each.type]]
        assert all(spec.warnings == () for spec in specs), [spec.warnings for spec in specs]


def test_every_movement_classifies(purged):
    """All 306 movements of the purged process match a routine expected for their action type."""
    specs = [spec for each in purged.process.actions.values() for spec in classify_action(purged.process, each)]
    assert len(specs) == 306
    assert not [spec.warnings for spec in specs if spec.warnings]


def test_transfer_workaround_changes_only_the_transfer(purged):
    """With the workaround only J_transfer changes, to TRANSFER_FREE."""
    for each in _first_actions(purged):
        plain = classify_action(purged.process, each)
        workaround = classify_action(purged.process, each, transfer_as_free=True)
        changed = [(a.movement, b.routine) for a, b in zip(plain, workaround) if a.routine != b.routine]
        assert changed == ([(f"{each.id}_transfer", R.TRANSFER_FREE)] if each.type == "bar_jointing" else [])


def test_goals(purged):
    """Where each goal comes from: next start, link poses, line, joints, stow."""
    load = _spec(purged, "B20_J_load")
    assert (load.goal, load.goal_joints, load.goal_from) == (GoalSource.NEXT_START, None, "B20_J_transfer")
    transfer = _spec(purged, "B20_J_transfer")
    assert transfer.goal is GoalSource.LINK_POSES and all(arm.target is not None for arm in transfer.arms)
    insert = _spec(purged, "B20_J_insert")
    assert insert.goal is GoalSource.LINE and all(arm.line is not None for arm in insert.arms)
    home_action, home = movement(purged, "B20_R_home")
    spec = _spec(purged, "B20_R_home")
    assert (spec.goal, spec.goal_joints) == (GoalSource.JOINTS, home.target.joints[home_action.robot])
    assert _spec(purged, "B21_H_approach").goal is GoalSource.LINK_POSES
    leave = _spec(purged, "B21_HR_leave")
    assert (leave.goal, leave.goal_joints) == (GoalSource.STOW, None)


def test_next_start(purged):
    """Skips tool and operator movements with null joints; stops at an arm movement, or where the robot is absent."""
    assert next_start(purged.process, *movement(purged, "B20_J_load")) == (None, "B20_J_transfer")
    joints, source = next_start(purged.process, *movement(purged, "B21_J_transfer"))
    insert = movement(purged, "B21_J_insert")[1]
    assert (joints, source) == (insert.start.robots["robots/cindy"].joints, "B21_J_insert")
    assert next_start(purged.process, *movement(purged, "B21_HR_leave")) == (None, "B35_HR_open")


def test_support_leave_goes_to_the_next_start_with_joints(purged):
    """A leave without a target goes where the next start puts the robot, when that start gives its joints."""
    process = purged.process
    alice = movement(purged, "B21_HR_leave")[1].start.robots["robots/alice"]
    following = process.actions["B35_HR"]
    first = following.movements[0]
    known = replace(first, start=replace(first.start, robots={**first.start.robots, "robots/alice": alice}))
    changed = replace(process, actions={**process.actions,
                                        "B35_HR": replace(following, movements=(known, *following.movements[1:]))})
    leave = classify(changed, *movement(purged, "B21_HR_leave"))
    assert (leave.routine, leave.goal) == (R.SUPPORT_LEAVE, GoalSource.NEXT_START)
    assert (leave.goal_joints, leave.goal_from) == (alice.joints, "B35_HR_open")


def test_arm_sides(purged):
    """Cindy's arms are left and right, with six joints each; Alice's one arm is "arm"."""
    cindy = _spec(purged, "B20_J_transfer").arms
    assert [(arm.arm, arm.side) for arm in cindy] == [("left_ur_arm", "left"), ("right_ur_arm", "right")]
    assert cindy[0].joints[0] == "left_ur_arm_shoulder_pan_joint" and len(cindy[0].joints) == 6
    alice = _spec(purged, "B21_H_approach").arms
    assert [(arm.flange, arm.arm, arm.side) for arm in alice] == [("robots/alice/ur_arm_tool0", "ur_arm", "arm")]


def test_unknown_shapes_raise(purged):
    """A linear coupled movement that neither tightens nor builds, and one moving both base and arms, raise."""
    each, insert = movement(purged, "B21_J_insert")
    unbuilt = replace(insert, drives={}, target=replace(insert.target, built=None))
    with pytest.raises(UnknownMovement, match="coupled True"):
        classify(purged.process, each, unbuilt)
    each, load = movement(purged, "B21_J_load")
    with pytest.raises(UnknownMovement, match="base"):
        classify(purged.process, each, replace(load, base=("robots/cindy",)))


def test_mating_drives_from_the_mates(purged):
    """Ground halves never tighten; a one-sided bar tightens one side; a normal bar both; a release never loosens."""
    grounded = mating_drives(purged.process, movement(purged, "B20_J_insert")[1])
    assert (grounded.kept, grounded.dropped, grounded.grounded) == ({}, {AT3L: GROUND_HALF, AT3R: GROUND_HALF}, True)
    one_sided = mating_drives(purged.process, movement(purged, "B21_J_insert")[1])
    assert (one_sided.kept, one_sided.dropped, one_sided.grounded) == ({AT3L: "tighten"}, {AT3R: NO_MATE}, False)
    both = mating_drives(purged.process, movement(purged, "B22_J_insert")[1])
    assert (both.kept, both.dropped) == ({AT3L: "tighten", AT3R: "tighten"}, {})
    release = mating_drives(purged.process, movement(purged, "B20_R_ungrasp")[1])
    assert (release.kept, release.dropped) == ({}, {AT3L: LOOSEN_NEVER_SENT, AT3R: LOOSEN_NEVER_SENT})
    assert _spec(purged, "B21_J_insert").drives == one_sided


def test_correct_drives_pass_through(purged):
    """An export whose B21 insert already tightens only the mating side is left as it is."""
    insert = movement(purged, "B21_J_insert")[1]
    corrected = replace(insert, drives={AT3L: "tighten"})
    drives = mating_drives(purged.process, corrected)
    assert (drives.kept, drives.dropped) == (corrected.drives, {})


def test_receiver_not_built_and_tool_on_nothing(purged):
    """B22's left half mates B21's: without B21 built it is not tightened; a tool on nothing is not either."""
    insert = movement(purged, "B22_J_insert")[1]
    start = insert.start
    no_b21 = replace(insert, start=replace(start, built=start.built - {"bars/B21"}))
    assert mating_drives(purged.process, no_b21).dropped == {AT3L: RECEIVER_NOT_BUILT}
    bare = replace(insert, start=replace(start, tools={**start.tools, AT3R: ToolState("closed", None)}))
    assert mating_drives(purged.process, bare).dropped == {AT3R: ON_NOTHING}


def test_action_type_mismatch_warns(purged):
    """A routine the action type does not expect is classified anyway, with a warning."""
    each, load = movement(purged, "B21_J_load")
    spec = classify(purged.process, replace(each, type="bar_release"), load)
    assert spec.routine is R.FREE_TO_LOAD and "free_to_load" in spec.warnings[0]
