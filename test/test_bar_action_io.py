"""Checks for the bar action loader helpers (no cell, no planner).

The kind and sibling checks are pure python. The loader checks read the REAL
exports, so they also catch the exporter changing shape under us; they are
skipped where the shared drive is not mounted.
"""
import os

import pytest

import rs_data_structure.bar_action as bar_action_module
from compas.geometry import Frame
from compas_fab.robots import RigidBodyState, RobotCellState
from rs_data_structure.bar_action import (
    BarAssemblyJointingAction,
    BarAssemblyReleaseAction,
    EndEffectorConstrainedDualArmFreeMovement,
    EndEffectorConstrainedDualArmLinearMovement,
    GripperToolMovement,
    IndependentDualArmFreeMovement,
    IndependentDualArmLinearMovement,
    ManualMovement,
    ScaffoldingToolMovement,
    SingleArmFreeMovement,
    SingleArmLinearMovement,
    ToolMovement,
)
from rs_data_structure.hold_action import BarHoldingAction

from husky_assembly_teleop import DESIGN_DATA_DIRECTORY
from husky_assembly_teleop.bar_action_io import (
    BarAssemblyAction, sibling_action_path, list_bar_actions,
    load_action_cycle, slot_of_index, cycle_start_ee_sources,
    parse_bar_action, MovementKind, movement_kind, kind_fits_robot,
    step_kind, tool_event, default_trajectory_time, FREE_HOME_TRAJECTORY_TIME_S,
    COMPLIANT_KINDS, is_free_home, check_action_kinds,
    ARM_KINDS, STATIONARY_KINDS, LIVE_SOLVED_TAG, clean_action_path, _clean_action_path,
    sidecar_action_path, preferred_action_path, write_path_for,
    BUILT_ASSEMBLY_RB_PREFIXES, is_built_assembly_body, bar_body_name, find_bar_body,
    bar_id_of_body, is_ground_joint_body, held_ground_joints,
    tool_runs_with_next_motion, OperatorStep, operator_steps, step_index_of,
)

# The two exports under test, one per schema. 260715 writes one file per bar
# holding M0..M4; 260929 splits each bar into a jointing and a release half.
LEGACY_PROBLEM = '260715_phase1_test'
SPLIT_PROBLEM = '260929_phase1_retest'
# The multi-robot export: Cindy's __J/__R plus the support robots' __H/__HR.
SCHEDULE_PROBLEM = '260920_RobArch_demo_revamp_backup'
# The newer multi-robot export: its grounded bars have no tighten step and end
# with a manual foundation fix instead.
HOLDING_TEST_PROBLEM = '261006_3bar_holding_test'

# * Three bars, picked for how they sit across the two exports:
# *   B1  -- in both, and its poses and configurations are identical, so the
# *          reference pose must come out the same through either schema.
# *   B4  -- in both by name, but re-planned (its base moved 823 mm and its M3
# *          start turned 90 deg), so the old file's data lives only there.
# *   B84 -- only in the new export (the model grew from 81 bars to 90).
BAR_SAME_IN_BOTH = 'B1'
BAR_OLD_ONLY = 'B4'
BAR_NEW_ONLY = 'B84'


def _actions_dir(problem: str) -> str:
    """The BarActions folder of a design problem.

    Args:
        problem (str): Design problem folder name.

    Returns:
        str: Absolute path, which need not exist on this machine.
    """
    return os.path.join(DESIGN_DATA_DIRECTORY, problem, 'BarActions')


def _bars_in(problem: str) -> set:
    """Active bar ids that have an action in a design problem.

    Args:
        problem (str): Design problem folder name.

    Returns:
        set: Bar ids, with any ``__J`` / ``__R`` half suffix stripped.
    """
    return {name.split('.', 1)[0].split('__', 1)[0]
            for name in list_bar_actions(_actions_dir(problem))}


needs_exports = pytest.mark.skipif(
    not (os.path.isdir(_actions_dir(LEGACY_PROBLEM))
         and os.path.isdir(_actions_dir(SPLIT_PROBLEM))),
    reason='the design-study exports are not mounted on this machine',
)


# * ------------------------------------------------- file names, legacy class

def test_sibling_paths():
    assert sibling_action_path('/p/B6__J.json') == '/p/B6__R.json'
    assert sibling_action_path('/p/B6__R.solved_keyframe.json') == '/p/B6__J.solved_keyframe.json'
    assert sibling_action_path('/p/B6.json') is None


def test_legacy_class_registered_for_compas():
    """compas looks the legacy dtype up on the rs_data_structure module."""
    assert bar_action_module.BarAssemblyAction is BarAssemblyAction


# * ------------------------------------- loading whole cycles, real exports

@needs_exports
def test_either_half_opens_the_same_cycle():
    """Picking the release half must not reorder the cycle."""
    folder = _actions_dir(SPLIT_PROBLEM)
    from_j = load_action_cycle(os.path.join(folder, f'{BAR_SAME_IN_BOTH}__J.json'))
    from_r = load_action_cycle(os.path.join(folder, f'{BAR_SAME_IN_BOTH}__R.json'))
    assert ([os.path.basename(p) for _a, p in from_j]
            == [os.path.basename(p) for _a, p in from_r])
    assert ([mv.movement_id for a, _p in from_j for mv in a.movements]
            == [mv.movement_id for a, _p in from_r for mv in a.movements])


@needs_exports
def test_slot_of_index_maps_to_the_owning_half():
    """Movements keep pointing at the file they were read from."""
    slots = load_action_cycle(
        os.path.join(_actions_dir(SPLIT_PROBLEM), f'{BAR_SAME_IN_BOTH}__J.json'))
    _action, path, local = slot_of_index(slots, 5)       # the insert, last of __J
    assert os.path.basename(path) == f'{BAR_SAME_IN_BOTH}__J.json' and local == 5
    _action, path, local = slot_of_index(slots, 6)       # first of __R
    assert os.path.basename(path) == f'{BAR_SAME_IN_BOTH}__R.json' and local == 0
    assert slot_of_index(slots, 99) is None


@needs_exports
def test_listing_folds_release_halves():
    """One slider entry per bar, not per file."""
    new_all = list_bar_actions(_actions_dir(SPLIT_PROBLEM))
    new_cycles = list_bar_actions(_actions_dir(SPLIT_PROBLEM), cycle_only=True)
    assert len(new_cycles) == len(_bars_in(SPLIT_PROBLEM)) < len(new_all)
    assert all(name.endswith('__J.json') for name in new_cycles)
    # The legacy export has no release halves, so nothing is folded away.
    legacy_all = list_bar_actions(_actions_dir(LEGACY_PROBLEM))
    assert list_bar_actions(_actions_dir(LEGACY_PROBLEM), cycle_only=True) == legacy_all


@needs_exports
def test_every_legacy_bar_still_exported():
    """Every bar of the old export still has an action in the new one.

    Keeps BAR_OLD_ONLY honest: it is old-only in its DATA, not in its existence.
    A re-export that drops a bar fails here rather than at the robot.
    """
    assert _bars_in(LEGACY_PROBLEM) <= _bars_in(SPLIT_PROBLEM)


# * ================================================ support robots (hold export)

needs_schedule_export = pytest.mark.skipif(
    not os.path.isdir(_actions_dir(SCHEDULE_PROBLEM)),
    reason='the multi-robot design-study export is not on this machine',
)
needs_holding_test_export = pytest.mark.skipif(
    not os.path.isdir(_actions_dir(HOLDING_TEST_PROBLEM)),
    reason='the 261006 multi-robot export is not on this machine',
)


def _load_schedule_action(name: str):
    """Load one action file of the multi-robot export.

    Args:
        name (str): File stem, e.g. ``'B3__H'``.

    Returns:
        BarSceneAction: The loaded action.
    """
    return parse_bar_action(os.path.join(_actions_dir(SCHEDULE_PROBLEM), f'{name}.json'))


# * ------------------------------------------------- movement kinds

_KIND_OF_CLASS = {
    IndependentDualArmFreeMovement: MovementKind.DUAL_FREE,
    EndEffectorConstrainedDualArmFreeMovement: MovementKind.DUAL_CONSTRAINED_FREE,
    EndEffectorConstrainedDualArmLinearMovement: MovementKind.DUAL_CONSTRAINED_LINEAR,
    IndependentDualArmLinearMovement: MovementKind.DUAL_INDEPENDENT_LINEAR,
    SingleArmFreeMovement: MovementKind.SINGLE_FREE,
    SingleArmLinearMovement: MovementKind.SINGLE_LINEAR,
    GripperToolMovement: MovementKind.GRIPPER_TOOL,
    ScaffoldingToolMovement: MovementKind.SCAFFOLDING_TOOL,
    ManualMovement: MovementKind.MANUAL,
}


def test_movement_kind_for_every_class():
    """Every concrete movement class maps to its own MovementKind."""
    for cls, kind in _KIND_OF_CLASS.items():
        assert movement_kind(cls(movement_id='x')) is kind, cls.__name__


def test_movement_kind_rejects_unknown_classes():
    """The bare tool base and any new subclass fail loudly instead of guessing."""
    class NewSingleArmMovement(SingleArmFreeMovement):
        pass

    with pytest.raises(TypeError):
        movement_kind(ToolMovement(movement_id='x'))
    with pytest.raises(TypeError):
        movement_kind(NewSingleArmMovement(movement_id='x'))


def test_kind_groups_and_robot_fit():
    """Arm and stationary kinds split all kinds; arm kinds fit only their robot type."""
    assert ARM_KINDS | STATIONARY_KINDS == set(MovementKind)
    assert not ARM_KINDS & STATIONARY_KINDS
    assert kind_fits_robot(MovementKind.DUAL_FREE, dual_arm=True)
    assert not kind_fits_robot(MovementKind.DUAL_CONSTRAINED_LINEAR, dual_arm=False)
    assert kind_fits_robot(MovementKind.SINGLE_LINEAR, dual_arm=False)
    assert not kind_fits_robot(MovementKind.SINGLE_FREE, dual_arm=True)
    for kind in STATIONARY_KINDS:
        assert kind_fits_robot(kind, dual_arm=True) and kind_fits_robot(kind, dual_arm=False)


def test_step_kind():
    """Each movement class maps to the right step label (arm / gripper / scaffold / manual)."""
    assert step_kind(SingleArmLinearMovement()) == 'arm'
    assert step_kind(IndependentDualArmFreeMovement()) == 'arm'
    assert step_kind(GripperToolMovement()) == 'gripper'
    assert step_kind(ScaffoldingToolMovement()) == 'scaffold'
    assert step_kind(ManualMovement()) == 'manual'


def test_tool_event():
    """tool_event returns the (action, tool names, overlaps_next) of a tool movement."""
    close = GripperToolMovement(tool_action='close', tool_names=['SupportGripper'])
    assert tool_event(close) == ('close', ['SupportGripper'], False)
    tighten = ScaffoldingToolMovement(tool_action='tighten', tool_names=['AT3L', 'AT3R'],
                                      overlaps_next=True)
    assert tool_event(tighten) == ('tighten', ['AT3L', 'AT3R'], True)


def test_default_trajectory_time():
    """The per-kind default; Cindy's free move home is shorter than her travel out."""
    free = IndependentDualArmFreeMovement()
    assert default_trajectory_time(free) == 30.0
    assert default_trajectory_time(free, free_home=True) == FREE_HOME_TRAJECTORY_TIME_S == 10.0
    transfer = EndEffectorConstrainedDualArmFreeMovement()
    assert default_trajectory_time(transfer) == 10.0
    # The flag only changes a free move.
    assert default_trajectory_time(transfer, free_home=True) == 10.0
    assert default_trajectory_time(EndEffectorConstrainedDualArmLinearMovement()) == 5.0
    assert default_trajectory_time(IndependentDualArmLinearMovement()) == 5.0
    assert default_trajectory_time(SingleArmFreeMovement()) == 15.0
    assert default_trajectory_time(SingleArmLinearMovement()) == 5.0
    for stationary in (GripperToolMovement(), ManualMovement()):
        assert default_trajectory_time(stationary) is None
        assert default_trajectory_time(stationary, free_home=True) is None


def test_compliant_kinds():
    """Only the insert and the retreat run under the compliance controller."""
    assert COMPLIANT_KINDS == {MovementKind.DUAL_CONSTRAINED_LINEAR,
                               MovementKind.DUAL_INDEPENDENT_LINEAR}


# * ------------------------------------------------- free move home, kind counts

def test_is_free_home_in_memory():
    """The action decides: release = home, jointing = travel out, legacy = the last free move."""
    out = IndependentDualArmFreeMovement(movement_id='out')
    assert not is_free_home(BarAssemblyJointingAction(movements=[out]), out)

    home = IndependentDualArmFreeMovement(movement_id='home')
    retreat = IndependentDualArmLinearMovement(movement_id='retreat')
    release = BarAssemblyReleaseAction(movements=[retreat, home])
    assert is_free_home(release, home)
    assert not is_free_home(release, retreat)

    free0 = IndependentDualArmFreeMovement(movement_id='free0')
    free1 = IndependentDualArmFreeMovement(movement_id='free1')
    legacy = BarAssemblyAction(movements=[
        free0,
        EndEffectorConstrainedDualArmFreeMovement(movement_id='transfer'),
        EndEffectorConstrainedDualArmLinearMovement(movement_id='insert'),
        IndependentDualArmLinearMovement(movement_id='retreat'),
        free1,
    ])
    assert not is_free_home(legacy, free0)
    assert is_free_home(legacy, free1)

    approach = SingleArmFreeMovement(movement_id='approach')
    assert not is_free_home(BarHoldingAction(movements=[approach]), approach)


@needs_schedule_export
def test_is_free_home_on_fixture():
    """B1__J only travels out; in B1__R exactly the free move home is home."""
    jointing = _load_schedule_action('B1__J')
    assert not any(is_free_home(jointing, mv) for mv in jointing.movements)
    release = _load_schedule_action('B1__R')
    assert [mv.movement_id for mv in release.movements if is_free_home(release, mv)] == [
        'B1_R_M3_free_home']


@needs_exports
def test_is_free_home_on_legacy_export():
    """In a legacy single file only the last movement is the free move home."""
    action = parse_bar_action(os.path.join(_actions_dir(LEGACY_PROBLEM), 'B1.json'))
    flags = [is_free_home(action, mv) for mv in action.movements]
    assert flags == [False] * (len(flags) - 1) + [True]


@pytest.mark.parametrize('problem, name', [
    pytest.param(SCHEDULE_PROBLEM, 'B1__J', marks=needs_schedule_export),
    pytest.param(SCHEDULE_PROBLEM, 'B1__R', marks=needs_schedule_export),
    pytest.param(SCHEDULE_PROBLEM, 'B3__H', marks=needs_schedule_export),
    pytest.param(SCHEDULE_PROBLEM, 'B3__HR', marks=needs_schedule_export),
    pytest.param(LEGACY_PROBLEM, 'B1', marks=needs_exports),
    pytest.param(LEGACY_PROBLEM, 'B4', marks=needs_exports),
])
def test_check_action_kinds_accepts_real_exports(problem, name):
    """Real exports hold one transfer / insert / retreat each, a legacy file two free moves."""
    check_action_kinds(parse_bar_action(os.path.join(_actions_dir(problem), f'{name}.json')))


def test_check_action_kinds_rejects_bad_counts():
    """A missing or doubled transfer / insert / retreat raises ValueError naming the file."""
    two_transfers = BarAssemblyJointingAction(movements=[
        EndEffectorConstrainedDualArmFreeMovement(movement_id='transfer_a'),
        EndEffectorConstrainedDualArmFreeMovement(movement_id='transfer_b'),
    ])
    with pytest.raises(ValueError, match='B9__J.json') as excinfo:
        check_action_kinds(two_transfers, 'B9__J.json')
    assert 'transfer' in str(excinfo.value) and 'insert' in str(excinfo.value)

    no_retreat = BarAssemblyReleaseAction(action_id='B9_R_release', movements=[
        ScaffoldingToolMovement(movement_id='untighten'),
        IndependentDualArmFreeMovement(movement_id='home'),
    ])
    with pytest.raises(ValueError, match='B9__R.json'):
        check_action_kinds(no_retreat, 'B9__R.json')
    with pytest.raises(ValueError, match='B9_R_release'):   # no source: the action id
        check_action_kinds(no_retreat)

    one_free_move = BarAssemblyAction(movements=[
        EndEffectorConstrainedDualArmFreeMovement(movement_id='transfer'),
        EndEffectorConstrainedDualArmLinearMovement(movement_id='insert'),
        IndependentDualArmLinearMovement(movement_id='retreat'),
        IndependentDualArmFreeMovement(movement_id='home'),
    ])
    with pytest.raises(ValueError, match='B9.json'):
        check_action_kinds(one_free_move, 'B9.json')

    # A support robot's action is not checked, whatever it holds.
    check_action_kinds(BarHoldingAction(movements=[
        SingleArmFreeMovement(movement_id='approach'), GripperToolMovement(movement_id='close'),
    ]), 'B9__H.json')


# * ------------------------------------------------- start poses for one arm

def test_start_ee_sources_single_arm_carry_and_clear():
    """Same rules as Cindy's two sides, with the support robot's one 'arm' side."""
    free = SingleArmFreeMovement(movement_id='free', target_ee_frames={'arm': Frame.worldXY()})
    opened = GripperToolMovement(movement_id='open')
    linear = SingleArmLinearMovement(movement_id='linear', target_ee_frames={'arm': Frame.worldXY()})
    closed = GripperToolMovement(movement_id='close')
    joint_goal = SingleArmFreeMovement(movement_id='joint_goal')      # moves, authors nothing
    manual = ManualMovement(movement_id='manual')

    sources = cycle_start_ee_sources([free, opened, linear, closed, joint_goal, manual],
                                     side_keys=('arm',))
    assert [s['arm'] for s in sources] == [None, free, free, linear, linear, None]
    assert all(set(s) == {'arm'} for s in sources)


# * ------------------------------------------------- clean export vs sidecar

def test_action_path_helpers(tmp_path):
    """Clean export and live-solved sidecar paths map to each other; clean files are never overwritten."""
    clean = str(tmp_path / 'B3__J.json')
    sidecar = str(tmp_path / f'B3__J.{LIVE_SOLVED_TAG}.json')
    assert LIVE_SOLVED_TAG == 'live-solved'

    assert _clean_action_path is clean_action_path
    assert clean_action_path(sidecar) == clean
    assert clean_action_path(clean) == clean
    assert sidecar_action_path(clean) == sidecar
    assert sidecar_action_path(sidecar) == sidecar          # calling twice changes nothing
    assert sidecar_action_path(clean, tag='solved_keyframe') == str(
        tmp_path / 'B3__J.solved_keyframe.json')

    # Preferred = the sidecar only once it is on disk.
    assert preferred_action_path(clean) == clean
    assert preferred_action_path(sidecar) == clean
    open(sidecar, 'w').close()
    assert preferred_action_path(clean) == sidecar

    # A clean export is never written over; a tagged file is written in place.
    assert write_path_for(clean) == sidecar
    assert write_path_for(sidecar) == sidecar


# * ------------------------------------------------- rigid-body naming

def test_rigid_body_naming():
    """Bar / joint body names with and without the support prefix, and back to the bar id."""
    assert BUILT_ASSEMBLY_RB_PREFIXES == ('bar_', 'joint_', 'env_bar_', 'env_joint_')
    for name in ('bar_B3', 'joint_B3_male', 'env_bar_B3', 'env_joint_B3_female'):
        assert is_built_assembly_body(name), name
    for name in ('obstacle_env1', 'obstacle_ground', 'ObstacleRobotAlice'):
        assert not is_built_assembly_body(name), name

    assert bar_body_name('B3') == 'bar_B3'
    assert bar_body_name('B3', 'env_') == 'env_bar_B3'
    assert find_bar_body(['bar_B3', 'joint_B3_male'], 'B3') == 'bar_B3'
    assert find_bar_body({'env_bar_B3'}, 'B3') == 'env_bar_B3'
    assert find_bar_body(['bar_B31'], 'B3') is None

    assert bar_id_of_body('bar_B12') == 'B12'
    assert bar_id_of_body('env_bar_B12') == 'B12'
    assert bar_id_of_body('joint_B12_male') is None
    assert bar_id_of_body('env_joint_B12_male') is None
    assert bar_id_of_body('obstacle_env1') is None


def test_ground_joint_naming():
    """Ground joints with and without the support prefix; other bodies are not."""
    for name in ('joint_G1-T20Ground-1_ground', 'env_joint_G1-T20Ground-0_ground'):
        assert is_ground_joint_body(name), name
    for name in ('joint_J1-12_female', 'bar_B1', 'obstacle_ground'):
        assert not is_ground_joint_body(name), name


# * ------------------------------------------------- grounded bars

# * The two grounded bars of both exports and the ground joints they are grasped
# * on at the insert. Every other bar is grasped on its male joints.
GROUNDED_BAR_JOINTS = {
    'B1': ['joint_G1-T20Ground-0_ground', 'joint_G1-T20Ground-1_ground'],
    'B5': ['joint_G5-T20Ground-0_ground', 'joint_G5-T20Ground-1_ground'],
}


def _insert_start_state(problem: str, bar: str):
    """The start state of a bar's insert, found by kind (not by index).

    Args:
        problem (str): Design problem folder name.
        bar (str): Bar id, e.g. ``'B1'``.

    Returns:
        RobotCellState: The start state of the jointing action's one
        DUAL_CONSTRAINED_LINEAR movement.
    """
    action = parse_bar_action(os.path.join(_actions_dir(problem), f'{bar}__J.json'))
    inserts = [mv for mv in action.movements
               if movement_kind(mv) is MovementKind.DUAL_CONSTRAINED_LINEAR]
    assert len(inserts) == 1, [mv.movement_id for mv in action.movements]
    return inserts[0].start_state


def test_held_ground_joints_in_memory():
    """Only a ground joint attached to a link or a tool is held; no state holds nothing."""
    assert held_ground_joints(None) == []
    ground = 'joint_G1-T20Ground-0_ground'
    state = RobotCellState(rigid_body_states={
        # Ground joints standing on the floor (built), with and without the support prefix.
        ground: RigidBodyState(Frame.worldXY()),
        'env_joint_G2-T20Ground-0_ground': RigidBodyState(Frame.worldXY()),
        # A normal bar's male joint in a tool is not a ground joint.
        'joint_J1-12_male': RigidBodyState(Frame.worldXY(), attached_to_link='left_ur_arm_tool0'),
    })
    assert held_ground_joints(state) == []
    state.rigid_body_states[ground].attached_to_tool = 'AT3L'
    assert held_ground_joints(state) == [ground]


@pytest.mark.parametrize('problem', [
    pytest.param(SCHEDULE_PROBLEM, marks=needs_schedule_export),
    pytest.param(HOLDING_TEST_PROBLEM, marks=needs_holding_test_export),
])
def test_held_ground_joints_on_exports(problem):
    """At the insert B1 and B5 hold their two ground joints (grounded); B3 holds none."""
    for bar, joints in GROUNDED_BAR_JOINTS.items():
        assert held_ground_joints(_insert_start_state(problem, bar)) == joints, bar
    assert held_ground_joints(_insert_start_state(problem, 'B3')) == []


@needs_schedule_export
def test_kinds_and_tool_events_of_real_hold_actions():
    """The hold export's classes and tool fields, as the exporter writes them."""
    hold = _load_schedule_action('B3__H')
    assert [movement_kind(mv) for mv in hold.movements] == [
        MovementKind.SINGLE_FREE, MovementKind.GRIPPER_TOOL,
        MovementKind.SINGLE_LINEAR, MovementKind.GRIPPER_TOOL]
    assert tool_event(hold.movements[1]) == ('open', ['SupportGripper'], False)
    assert tool_event(hold.movements[3]) == ('close', ['SupportGripper'], False)
    jointing = _load_schedule_action('B3__J')
    assert tool_event(jointing.movements[4]) == ('tighten', ['AT3L', 'AT3R'], True)


# * ------------------------------------------------- operator steps

def _screw_step(movement_id: str, tool_action: str, overlaps_next: bool = False):
    """One of Cindy's scaffolding-tool steps, built in memory.

    Args:
        movement_id (str): Its id.
        tool_action (str): e.g. ``'tighten'``.
        overlaps_next (bool): Whether it overlaps the next movement.

    Returns:
        ScaffoldingToolMovement: The step.
    """
    return ScaffoldingToolMovement(movement_id=movement_id, tool_action=tool_action,
                                   tool_names=['AT3L', 'AT3R'], overlaps_next=overlaps_next)


def test_tool_runs_with_next_motion():
    """Only the overlapping tighten, the ungrasp and the untighten of the screw tool run with the next motion."""
    assert tool_runs_with_next_motion(_screw_step('tighten', 'tighten', overlaps_next=True))
    assert tool_runs_with_next_motion(_screw_step('ungrasp', 'ungrasp'))
    assert tool_runs_with_next_motion(_screw_step('untighten', 'untighten'))
    assert not tool_runs_with_next_motion(_screw_step('grasp', 'grasp'))
    assert not tool_runs_with_next_motion(_screw_step('tighten', 'tighten', overlaps_next=False))
    # The rule is about the screw tool only: a support gripper step never runs with the next motion.
    assert not tool_runs_with_next_motion(GripperToolMovement(tool_action='ungrasp', overlaps_next=True))
    assert not tool_runs_with_next_motion(ManualMovement(movement_id='mount'))
    assert not tool_runs_with_next_motion(EndEffectorConstrainedDualArmLinearMovement(movement_id='insert'))
    assert not tool_runs_with_next_motion(ToolMovement(movement_id='bare'))   # class with no kind


def test_operator_steps_in_memory():
    """Absorbed tool steps join the next compliant arm movement; anything else ends the wait."""
    jointing = [
        IndependentDualArmFreeMovement(movement_id='free_to_load'),
        ManualMovement(movement_id='mount'),
        _screw_step('grasp', 'grasp'),
        EndEffectorConstrainedDualArmFreeMovement(movement_id='transfer'),
        _screw_step('tighten', 'tighten', overlaps_next=True),
        EndEffectorConstrainedDualArmLinearMovement(movement_id='insert'),
    ]
    assert operator_steps(jointing) == [
        OperatorStep(0), OperatorStep(1), OperatorStep(2), OperatorStep(3), OperatorStep(5, (4,))]

    retreat = IndependentDualArmLinearMovement(movement_id='retreat')
    home = IndependentDualArmFreeMovement(movement_id='home')
    untighten = _screw_step('untighten', 'untighten')
    ungrasp = _screw_step('ungrasp', 'ungrasp')
    assert operator_steps([untighten, ungrasp, retreat, home]) == [
        OperatorStep(2, (0, 1)), OperatorStep(3)]
    # * After the re-export without the untighten (Rhino note D7).
    assert operator_steps([ungrasp, retreat, home]) == [OperatorStep(1, (0,)), OperatorStep(2)]
    # A manual step between them: the waiting untighten becomes its own step first.
    assert operator_steps([untighten, ManualMovement(movement_id='mount'), retreat]) == [
        OperatorStep(0), OperatorStep(1), OperatorStep(2)]
    # Nothing follows: a trailing tool step stays its own step.
    insert = EndEffectorConstrainedDualArmLinearMovement(movement_id='insert')
    assert operator_steps([insert, _screw_step('tighten', 'tighten', overlaps_next=True)]) == [
        OperatorStep(0), OperatorStep(1)]
    # ! Never hidden behind a movement that would not send it: a tighten before a
    # ! free move (not compliant) stays its own step.
    assert operator_steps([_screw_step('tighten', 'tighten', overlaps_next=True),
                           IndependentDualArmFreeMovement(movement_id='free')]) == [
        OperatorStep(0), OperatorStep(1)]
    assert operator_steps([]) == []


@needs_schedule_export
def test_operator_steps_on_fixture():
    """B1__J folds its tighten into the insert, B1__R its screw steps into the retreat.

    The support robots' hold and hold release keep one step per movement.
    """
    jointing = _load_schedule_action('B1__J').movements
    assert operator_steps(jointing) == [
        OperatorStep(0), OperatorStep(1), OperatorStep(2), OperatorStep(3), OperatorStep(5, (4,))]

    # * The release's screw steps may change with a re-export (Rhino note D7), so
    # * the layout is read from the file: every step before the retreat joins it.
    release = _load_schedule_action('B1__R').movements
    n = len(release)
    assert all(tool_runs_with_next_motion(mv) for mv in release[:n - 2])
    assert operator_steps(release) == [OperatorStep(n - 2, tuple(range(n - 2))), OperatorStep(n - 1)]

    for name in ('B3__H', 'B3__HR'):
        movements = _load_schedule_action(name).movements
        assert operator_steps(movements) == [OperatorStep(i) for i in range(len(movements))], name


def test_step_index_of():
    """A movement maps to the step holding it, as primary or absorbed; anything else to None."""
    steps = [OperatorStep(0), OperatorStep(1), OperatorStep(2), OperatorStep(3), OperatorStep(5, (4,))]
    assert [step_index_of(steps, i) for i in range(6)] == [0, 1, 2, 3, 4, 4]
    assert step_index_of(steps, 6) is None
    assert step_index_of(steps, None) is None
    assert step_index_of([], 0) is None
