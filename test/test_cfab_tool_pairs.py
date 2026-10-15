"""CC.6: the planning robot's own tools against the other robots (CfabSession's check_collision).

compas_fab checks robot links against tools (CC.2) and tools against rigid bodies
(CC.5), never a tool against a tool. The other robots stand in a cell as tools
(``ObstacleRobot<Name>``), so ``cfab_session._check_collision_with_tool_pairs``
adds that step. These checks load Cindy's real 261006 cell (a few seconds) and
are skipped where the shared drive is not mounted.
"""
import os

import numpy as np
import pybullet as p
import pytest
from compas.geometry import Frame
from compas_fab.backends import CollisionCheckError

from husky_assembly_teleop import DESIGN_DATA_DIRECTORY
from husky_assembly_teleop.bar_action_io import parse_bar_action
from husky_assembly_teleop.cfab_session import CfabSession, inject_ground_rigid_body_state

PROBLEM = '261006_3bar_holding_test'
B36_J = os.path.join(DESIGN_DATA_DIRECTORY, PROBLEM, 'BarActions', 'B36__J.json')

needs_holding_test_export = pytest.mark.skipif(
    not os.path.isfile(B36_J), reason='the 261006 multi-robot export is not on this machine')


@pytest.fixture(scope='module')
def session():
    """Cindy's planning cell of the 261006 export.

    Yields:
        CfabSession: Closed after the module's tests.
    """
    s = CfabSession(PROBLEM, cell_filename='RobotCell.json', connection_type='direct')
    yield s
    s.close()


def _insert_start(session: CfabSession):
    """B36's insert start: Cindy at the insertion start, her tools AT3L / AT3R on, Belle by B35.

    Args:
        session (CfabSession): Cindy's planning cell.

    Returns:
        RobotCellState: The exported state, with the floor added.
    """
    mv = [m for m in parse_bar_action(B36_J).movements if m.movement_id == 'B36_J_M5_LM_insert'][0]
    state = mv.start_state.copy()
    inject_ground_rigid_body_state(session.robot_cell, state)
    return state


def _tool_pair_lines(session: CfabSession, state, **options) -> list:
    """The CC.6 lines of a full collision report.

    Args:
        session (CfabSession): Cindy's planning cell.
        state (RobotCellState): The state to check.
        **options: Extra check_collision options (e.g. ``_skip_cc2=True``).

    Returns:
        list: Every ``CC.6 ...`` line of the report (empty when none).
    """
    try:
        session.planner.check_collision(state, {'full_report': True, **options})
    except CollisionCheckError as e:
        return [line for line in e.message.splitlines() if line.startswith('CC.6')]
    return []


def _belle_gripper_into_at3l(session: CfabSession, state) -> None:
    """Shift Belle's obstacle robot (no turn) so her gripper's middle sits on AT3L's middle.

    Args:
        session (CfabSession): Cindy's planning cell.
        state (RobotCellState): Edited in place.
    """
    client = session.client
    session.planner.set_robot_cell_state(state)
    belle = client.tools_puids['ObstacleRobotBelle']
    gripper = [j for j in range(p.getNumJoints(belle, physicsClientId=client.client_id))
               if p.getJointInfo(belle, j, physicsClientId=client.client_id)[12].decode()
               == 'ur_arm_tool0_obstacle_tool'][0]
    middle = lambda aabb: np.add(aabb[0], aabb[1]) / 2   # noqa: E731
    shift = (middle(p.getAABB(client.tools_puids['AT3L'], physicsClientId=client.client_id))
             - middle(p.getAABB(belle, gripper, physicsClientId=client.client_id)))
    tool_state = state.tool_states['ObstacleRobotBelle']
    frame = tool_state.frame
    tool_state.frame = Frame(list(np.add(list(frame.point), shift)), frame.xaxis, frame.yaxis)


@needs_holding_test_export
def test_exported_insert_start_has_no_tool_pair(session):
    """As exported, Cindy's tools at B36's insertion start touch no other robot."""
    assert _tool_pair_lines(session, _insert_start(session)) == []


@needs_holding_test_export
def test_own_tool_inside_another_robot_is_a_collision(session):
    """Belle's gripper on Cindy's AT3L: CC.6 reports the pair (compas_fab alone did not)."""
    state = _insert_start(session)
    _belle_gripper_into_at3l(session, state)
    lines = _tool_pair_lines(session, state)
    assert "CC.6 between tool 'AT3L' and tool 'ObstacleRobotBelle' - COLLISION" in lines
    # Without a full report the first hit raises too.
    with pytest.raises(CollisionCheckError):
        session.planner.check_collision(state, {})


@needs_holding_test_export
def test_tool_pairs_skipped_with_cc2_or_hidden(session):
    """No CC.6 when the caller skips robot-vs-tool checks, or when the other robot is hidden."""
    state = _insert_start(session)
    _belle_gripper_into_at3l(session, state)
    assert _tool_pair_lines(session, state, _skip_cc2=True) == []
    state.tool_states['ObstacleRobotBelle'].is_hidden = True
    assert _tool_pair_lines(session, state) == []
