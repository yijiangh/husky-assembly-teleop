"""Tests for shared.planning.steps: an action's movements grouped into the operator's steps."""

from __future__ import annotations

from dataclasses import replace
from itertools import chain

import pytest
from planning_fixtures import action, purged_process

from husky_assembly_teleop.shared.planning.movement import Routine, classify
from husky_assembly_teleop.shared.planning.steps import OperatorStep, operator_steps, step_index_of
from scaffolding_assembly_core.process import Action, Movement, State, Target


@pytest.fixture(scope="module")
def purged():
    """The purged process."""
    return purged_process()


def test_jointing_steps(purged):
    """J: one step per movement; with the workaround, the transfer, then mount and grasp, and no travel to load.

    The workaround's transfer runs with empty tools; the operator mounts and grasps the bar after it.
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
