"""Checks for the floor's allowed contacts (state only, no cell file, no planner).

The floor body (``obstacle_ground``) is added by the monitor, not by the export,
so its ``touch_bodies`` are computed here. These checks read one real BarAction
of the multi-robot export and are skipped where the shared drive is not mounted.
! Never load a RobotCell*.json here: the cell is stood in by a tiny object
! holding only the two fields ``inject_ground_rigid_body_state`` reads.
"""
import os
import types

import pytest

from husky_assembly_teleop import DESIGN_DATA_DIRECTORY
from husky_assembly_teleop.bar_action_io import parse_bar_action
from husky_assembly_teleop.cfab_session import (
    GROUND_RIGID_BODY_NAME, GROUND_TOUCH_LINKS, ground_touch_bodies,
    inject_ground_rigid_body_state,
)

# The multi-robot export: Cindy's __J/__R plus the support robots' __H/__HR.
SCHEDULE_PROBLEM = '260920_RobArch_demo_revamp_backup'
ACTIONS_DIR = os.path.join(DESIGN_DATA_DIRECTORY, SCHEDULE_PROBLEM, 'BarActions')
# * B1 is a ground bar: both of its ground joints ride with it while it is inserted.
B1_GROUND_JOINTS = ('joint_G1-T20Ground-0_ground', 'joint_G1-T20Ground-1_ground')

needs_schedule_export = pytest.mark.skipif(
    not os.path.isdir(ACTIONS_DIR),
    reason='the multi-robot design-study export is not on this machine',
)


def _b1_insert_start_state():
    """Start state of ``B1_J_M5_LM_insert`` (movement 5 of ``B1__J.json``).

    Returns:
        RobotCellState: The exported state, without a floor entry.
    """
    action = parse_bar_action(os.path.join(ACTIONS_DIR, 'B1__J.json'))
    mv = action.movements[5]
    assert mv.movement_id == 'B1_J_M5_LM_insert'
    return mv.start_state


def _stand_in_cell(obstacle_tools: tuple = ()) -> types.SimpleNamespace:
    """A cell stand-in that has the floor body and the given obstacle-robot tools.

    Args:
        obstacle_tools (tuple): ``ObstacleRobot<Name>`` tool names to put in the cell.

    Returns:
        types.SimpleNamespace: Object with ``rigid_body_models`` and ``tool_models``.
    """
    return types.SimpleNamespace(
        rigid_body_models={GROUND_RIGID_BODY_NAME: None},
        tool_models={name: None for name in obstacle_tools},
    )


@needs_schedule_export
def test_touch_list_holds_the_ground_joints():
    """The pure touch-list: ground joints and obstacle robots, each once, nothing else."""
    state = _b1_insert_start_state()
    touch = ground_touch_bodies(state.rigid_body_states, ['ObstacleRobotAlice'],
                                existing=['ObstacleRobotAlice'])
    for name in B1_GROUND_JOINTS:
        assert name in touch, name
    assert 'ObstacleRobotAlice' in touch
    assert len(touch) == len(set(touch))
    assert 'bar_B1' not in touch


@needs_schedule_export
def test_new_floor_entry_allows_the_ground_joints():
    """Injecting the floor into the B1 insert start state allows both ground joints."""
    state = _b1_insert_start_state()
    assert GROUND_RIGID_BODY_NAME not in state.rigid_body_states
    inject_ground_rigid_body_state(_stand_in_cell(('ObstacleRobotAlice',)), state)
    ground = state.rigid_body_states[GROUND_RIGID_BODY_NAME]
    for name in B1_GROUND_JOINTS:
        assert name in ground.touch_bodies, name
    assert 'ObstacleRobotAlice' in ground.touch_bodies
    assert sorted(ground.touch_links) == sorted(GROUND_TOUCH_LINKS)


@needs_schedule_export
def test_existing_floor_entry_gains_the_ground_joints():
    """An older floor entry (no ground joints) gets them added, without duplicates."""
    state = _b1_insert_start_state()
    cell = _stand_in_cell()
    inject_ground_rigid_body_state(cell, state)
    ground = state.rigid_body_states[GROUND_RIGID_BODY_NAME]
    ground.touch_bodies = ['ObstacleRobotBelle']
    inject_ground_rigid_body_state(cell, state)
    inject_ground_rigid_body_state(cell, state)
    for name in B1_GROUND_JOINTS:
        assert ground.touch_bodies.count(name) == 1, name
    assert 'ObstacleRobotBelle' in ground.touch_bodies
