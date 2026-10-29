"""Tests for shared.assembly_planning.steps: an action's movements grouped into the operator's steps."""

from __future__ import annotations

from dataclasses import replace
from itertools import chain
from pathlib import Path
from types import SimpleNamespace

import pytest
from planning_fixtures import action, purged_process
from process_fixtures import build_process, joints, with_movement

from husky_assembly_teleop.shared.assembly_planning.movement import Routine, classify, routine_or_none
from husky_assembly_teleop.shared.assembly_planning.process_steps import LoadedProcess, Step
from husky_assembly_teleop.shared.assembly_planning.steps import (TRANSIT_LABEL, OperatorStep, as_run, operator_steps,
                                                                  step_index_of)
from scaffolding_assembly_core.process import Action, Movement, State, Target
from scaffolding_assembly_core.process.plan_check import end_state


@pytest.fixture(scope="module")
def purged():
    """The purged process."""
    return purged_process()


def test_jointing_steps(purged):
    """J: one step per movement; with the workaround, the transfer, then mount and grasp, and no travel to load.

    The workaround's transfer runs with empty tools; the operator places and grasps the bar after it.
    """
    jointing = action(purged, "B21_J")
    assert operator_steps(purged.process, jointing) == [OperatorStep(index) for index in range(5)]
    workaround = operator_steps(purged.process, jointing, transfer_as_free=True)
    assert workaround == [OperatorStep(3, after_motion=(1, 2), skipped=(0,)), OperatorStep(4)]
    assert workaround[0].indices == (3, 1, 2)


def test_release_ungrasp_runs_with_the_retreat(purged):
    """R: the ungrasp is folded into the retreat; home is its own step."""
    assert operator_steps(purged.process, action(purged, "B20_R")) == [OperatorStep(1, with_motion=(0,)),
                                                                       OperatorStep(2)]


def test_support_steps(purged):
    """H and HR: every movement is a step; the opening before a one-arm retreat is not folded."""
    assert operator_steps(purged.process, action(purged, "B21_H")) == [OperatorStep(index) for index in range(4)]
    assert operator_steps(purged.process, action(purged, "B21_HR")) == [OperatorStep(index) for index in range(3)]


def test_bar_placing_is_one_step(purged):
    """A bar put in by hand: one operator movement, one step."""
    start = action(purged, "B20_J").movements[0].start
    placing = Action("B99_P", "bar_placing", "", "bars/B99", (
        Movement("B99_P_place", start=State(start.robots, start.present), ends_on="operator",
                 target=Target(built=frozenset({"bars/B99"}))),))
    assert operator_steps(purged.process, placing) == [OperatorStep(0)]


def test_fixing_the_foundation_is_its_own_step(purged):
    """A ground bar built by the operator after its insert: an operator movement, its own step after the insert."""
    jointing = action(purged, "B20_J")
    insert = jointing.movements[-1]
    fix = Movement("B20_J_fix_foundation", start=insert.start, ends_on="operator",
                   target=Target(built=frozenset({*insert.start.built, "bars/B20"})))
    fixed = replace(jointing, movements=(*jointing.movements, fix))
    assert classify(purged.process, fixed, fix).routine is Routine.OPERATOR
    assert operator_steps(purged.process, fixed)[-2:] == [OperatorStep(4), OperatorStep(5)]
    assert operator_steps(purged.process, fixed, transfer_as_free=True)[-2:] == [OperatorStep(4), OperatorStep(5)]


def test_every_index_once(purged):
    """In every action, with and without the workaround, each movement is run or skipped by exactly one step."""
    for each in purged.process.actions.values():
        for transfer_as_free in (False, True):
            steps = operator_steps(purged.process, each, transfer_as_free)
            indices = list(chain.from_iterable((*step.indices, *step.skipped) for step in steps))
            assert sorted(indices) == list(range(len(each.movements))), (each.id, transfer_as_free)
            assert [step.primary for step in steps] == sorted(step.primary for step in steps)


def test_step_index_of(purged):
    """Which step holds a movement, as primary, run with or after it, or skipped; None for none."""
    steps = operator_steps(purged.process, action(purged, "B20_R"))
    assert [step_index_of(steps, index) for index in (0, 1, 2, 3, None)] == [0, 0, 1, None, None]
    assert steps[0].indices == (1, 0)
    paused = operator_steps(purged.process, action(purged, "B20_J"), transfer_as_free=True)
    assert [step_index_of(paused, index) for index in range(5)] == [0, 0, 0, 0, 1]


def test_index_of_an_actions_movement():
    """A movement's step index: its action's first step plus its index in the action; none for a movement it lacks."""
    steps = tuple(Step(None, action, movement, None) for action, movement in ((0, 0), (0, 1), (1, 0), (1, 1), (1, 2)))
    actions = {"A": SimpleNamespace(movements=(0, 1)), "B": SimpleNamespace(movements=(0, 1, 2))}
    loaded = LoadedProcess(Path("process"), SimpleNamespace(schedule=("A", "B"), actions=actions), steps)
    assert (loaded.index_of("A"), loaded.index_of("B"), loaded.index_of("B", 2)) == (0, 2, 4)
    with pytest.raises(IndexError, match="A has no movement 2"):
        loaded.index_of("A", 2)


def test_as_run_shows_a_jointing_action_as_the_workaround_runs_it(purged):
    """B22_J: the transit from the action's start (no bar, tools open); place and grasp at the insert start; insert.

    Releases keep their objects; without the workaround the process itself comes back.
    """
    process = purged.process
    shown = as_run(process, True)
    load, place_bar, grasp, transfer, insert = process.actions["B22_J"].movements
    jointing = shown.actions["B22_J"]
    assert [each.id for each in jointing.movements] == [transfer.id, place_bar.id, grasp.id, insert.id]
    transit, place = jointing.movements[:2]
    assert "bars/B22" not in transit.start.present and transit.label == TRANSIT_LABEL
    assert {tools.grip for tool, tools in transit.start.tools.items() if tool.startswith("tools/cindy/")} == {"open"}
    assert routine_or_none(jointing, transit, True) is Routine.TRANSFER_FREE
    assert place.start.robots["robots/cindy"].joints == transfer.target.joints["robots/cindy"]
    assert "bars/B22" in end_state(place).attached and jointing.movements[3] is insert
    assert all(shown.actions[action_id] is each for action_id, each in process.actions.items()
               if each.type == "bar_release")
    assert as_run(process, False) is process


def test_as_run_without_the_drive(tmp_path):
    """The small process, B2's transfer coupled: the transit from the place's start, then place and grasp at its end."""
    process = with_movement(build_process(tmp_path), "B2_J_joint", 2, coupled=True)
    shown = as_run(process, True)
    place, grasp, transfer, insert = process.actions["B2_J_joint"].movements
    transit, *rest = shown.actions["B2_J_joint"].movements
    assert [each.id for each in (transit, *rest)] == [transfer.id, place.id, grasp.id, insert.id]
    assert transit.start is place.start and transit.target.joints == {"robots/cindy": joints(0.3)}
    assert rest[0].start.robots["robots/cindy"].joints == joints(0.3) and rest[2] is insert
    assert shown.actions["B1_J_joint"] is process.actions["B1_J_joint"]
