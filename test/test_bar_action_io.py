"""Checks for the bar action loader helpers (no cell, no planner).

The role and sibling checks are pure python. The loader checks read the REAL
exports, so they also catch the exporter changing shape under us; they are
skipped where the shared drive is not mounted.
"""
import os
import shutil
import types

import pytest

import rs_data_structure.bar_action as bar_action_module
from compas.geometry import Frame
from rs_data_structure.bar_action import (
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

from husky_assembly_teleop import DESIGN_DATA_DIRECTORY
from husky_assembly_teleop.bar_action_io import (
    BarAssemblyAction, movement_role, sibling_action_path, list_bar_actions,
    load_action_cycle, cycle_roles, slot_of_index, cycle_start_ee_sources,
    parse_bar_action, roles_for_action, MovementKind, movement_kind, kind_fits_robot,
    step_kind, movement_controller, tool_event, default_trajectory_time,
    ARM_KINDS, STATIONARY_KINDS, LIVE_SOLVED_TAG, clean_action_path, _clean_action_path,
    sidecar_action_path, preferred_action_path, write_path_for,
    BUILT_ASSEMBLY_RB_PREFIXES, is_built_assembly_body, bar_body_name, find_bar_body,
    bar_id_of_body, is_ground_joint_body,
)

# The two exports under test, one per schema. 260715 writes one file per bar
# holding M0..M4; 260929 splits each bar into a jointing and a release half.
LEGACY_PROBLEM = '260715_phase1_test'
SPLIT_PROBLEM = '260929_phase1_retest'
# The multi-robot export: Cindy's __J/__R plus the support robots' __H/__HR.
SCHEDULE_PROBLEM = '260920_RobArch_demo_revamp_backup'

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


def _role_index(roles: list, role: str) -> int:
    """Where a classic role sits in a loaded cycle.

    Args:
        roles (list): Output of ``cycle_roles``.
        role (str): 'M0'..'M4'.

    Returns:
        int: Its index in the cycle.
    """
    assert roles.count(role) == 1, f"expected exactly one {role}, got {roles}"
    return roles.index(role)


def _start_ee_source(slots: list, role: str, side: str):
    """Which movement authors ``side``'s flange pose at the start of ``role``.

    Mirrors how the monitor reads the cycle's start-EE map
    (``HuskyMonitor._start_ee_source``): build it once, then index it.

    Args:
        slots (list): Output of ``load_action_cycle``.
        role (str): The classic role being measured, e.g. 'M3'.
        side (str): ``'left'`` or ``'right'``.

    Returns:
        Movement | None: The authoring movement.
    """
    movements = [mv for action, _p in slots for mv in action.movements]
    sources = cycle_start_ee_sources(movements)
    return sources[_role_index(cycle_roles(slots), role)][side]


needs_exports = pytest.mark.skipif(
    not (os.path.isdir(_actions_dir(LEGACY_PROBLEM))
         and os.path.isdir(_actions_dir(SPLIT_PROBLEM))),
    reason='the design-study exports are not mounted on this machine',
)


def _mv(movement_id):
    return types.SimpleNamespace(movement_id=movement_id)


# * ------------------------------------------------- roles read off the ids

def test_roles_legacy_ids():
    """The single-file export tags the classic role directly."""
    ids = ['B3_M0_free_to_M1_start', 'B3_M1_CDFM_bar_loading_to_approach',
           'B3_M2_LM_mate', 'B3_M3_LM_retreat', 'B3_M4_free_home']
    assert [movement_role(_mv(i)) for i in ids] == ['M0', 'M1', 'M2', 'M3', 'M4']


def test_roles_split_ids():
    """The jointing/release export numbers movements per file; translate them."""
    jointing = ['B6_J_M0_free_to_load', 'B6_J_M1_manual_mount_bar', 'B6_J_M2_tool_grasp_bar',
                'B6_J_M3_CDFM_transfer_to_approach', 'B6_J_M4_tool_tighten_joint',
                'B6_J_M5_LM_insert']
    release = ['B6_R_M0_tool_untighten_joint', 'B6_R_M1_tool_ungrasp_bar',
               'B6_R_M2_LM_retreat', 'B6_R_M3_free_home']
    assert [movement_role(_mv(i)) for i in jointing] == ['M0', None, None, 'M1', None, 'M2']
    assert [movement_role(_mv(i)) for i in release] == [None, None, 'M3', 'M4']
    assert movement_role(_mv('')) is None


def test_sibling_paths():
    assert sibling_action_path('/p/B6__J.json') == '/p/B6__R.json'
    assert sibling_action_path('/p/B6__R.solved_keyframe.json') == '/p/B6__J.solved_keyframe.json'
    assert sibling_action_path('/p/B6.json') is None


def test_legacy_class_registered_for_compas():
    """compas looks the legacy dtype up on the rs_data_structure module."""
    assert bar_action_module.BarAssemblyAction is BarAssemblyAction


# * ------------------------------------- loading whole cycles, real exports

@needs_exports
def test_unchanged_bar_gives_the_same_reference_pose_in_both_exports():
    """B1 is identical in the two exports, so both must resolve to one pose.

    This is the check that matters for the accuracy test: the pose stamped into
    a take is the authored flange target of the movement before the retreat,
    composed with the grasp. The legacy export finds that movement right above
    the retreat; the split export has to walk back over two screw events into
    the other file. Same bar, so the answer has to be the same.
    """
    legacy = load_action_cycle(
        os.path.join(_actions_dir(LEGACY_PROBLEM), f'{BAR_SAME_IN_BOTH}.json'))
    split = load_action_cycle(
        os.path.join(_actions_dir(SPLIT_PROBLEM), f'{BAR_SAME_IN_BOTH}__J.json'))

    legacy_src = _start_ee_source(legacy, 'M3', 'left')
    split_src = _start_ee_source(split, 'M3', 'left')

    assert legacy_src.movement_id == f'{BAR_SAME_IN_BOTH}_M2_LM_mate'
    assert split_src.movement_id == f'{BAR_SAME_IN_BOTH}_J_M5_LM_insert'
    assert (list(split_src.target_ee_frames['left'].point)
            == pytest.approx(list(legacy_src.target_ee_frames['left'].point)))


@needs_exports
def test_legacy_only_bar_loads_as_one_cycle():
    """B4's authored data lives only in the old export (it was re-planned since).

    A legacy action has no sibling half, so the whole cycle is the one file.
    """
    slots = load_action_cycle(
        os.path.join(_actions_dir(LEGACY_PROBLEM), f'{BAR_OLD_ONLY}.json'))
    assert len(slots) == 1
    movements = [mv for action, _p in slots for mv in action.movements]
    assert len(movements) == 5
    assert cycle_roles(slots) == ['M0', 'M1', 'M2', 'M3', 'M4']
    src = _start_ee_source(slots, 'M3', 'left')
    assert src.movement_id == f'{BAR_OLD_ONLY}_M2_LM_mate'


@needs_exports
def test_new_only_bar_loads_both_halves():
    """B84 exists only in the new export; picking one half opens the cycle."""
    slots = load_action_cycle(
        os.path.join(_actions_dir(SPLIT_PROBLEM), f'{BAR_NEW_ONLY}__J.json'))
    assert [os.path.basename(p) for _action, p in slots] == [
        f'{BAR_NEW_ONLY}__J.json', f'{BAR_NEW_ONLY}__R.json']
    movements = [mv for action, _p in slots for mv in action.movements]
    roles = cycle_roles(slots)
    assert len(movements) == 10
    assert roles == ['M0', None, None, 'M1', None, 'M2', None, None, 'M3', 'M4']
    src = _start_ee_source(slots, 'M3', 'left')
    assert src.movement_id == f'{BAR_NEW_ONLY}_J_M5_LM_insert'


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
def test_start_ee_source_is_cleared_by_an_arm_movement():
    """A movement that MOVED the arms must not be carried past.

    Before the transfer (M1) sit the manual mount and the grasp -- no arm moves
    in those -- and then M0, which does move both arms but authors no EE target
    (it is a joint-space goal). Carrying anything past M0 would describe a flange
    pose the arm has since left, so the source is cleared and the caller fails
    loudly instead.
    """
    slots = load_action_cycle(
        os.path.join(_actions_dir(SPLIT_PROBLEM), f'{BAR_SAME_IN_BOTH}__J.json'))
    movements = [mv for action, _p in slots for mv in action.movements]
    sources = cycle_start_ee_sources(movements)
    assert _start_ee_source(slots, 'M1', 'left') is None
    assert sources[0] == {'left': None, 'right': None}   # nothing precedes M0
    # Both sides are tracked, and both are filled by the insert for the retreat.
    assert set(sources[_role_index(cycle_roles(slots), 'M3')]) == {'left', 'right'}
    assert all(src is not None
               for src in sources[_role_index(cycle_roles(slots), 'M3')].values())


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
def test_sidecar_falls_back_to_the_clean_release_half(tmp_path):
    """Step A saves only the jointing half; reloading it must still give M3.

    Adopting the manual start writes M0 and M1, which both live in the jointing
    file, so no release sidecar is ever written. Reloading the jointing sidecar
    therefore has to pick up the clean release export next to it, or Step B
    would have no retreat to measure at.
    """
    folder = _actions_dir(SPLIT_PROBLEM)
    shutil.copy(os.path.join(folder, f'{BAR_SAME_IN_BOTH}__J.json'),
                tmp_path / f'{BAR_SAME_IN_BOTH}__J.live-solved.json')
    shutil.copy(os.path.join(folder, f'{BAR_SAME_IN_BOTH}__R.json'),
                tmp_path / f'{BAR_SAME_IN_BOTH}__R.json')

    slots = load_action_cycle(str(tmp_path / f'{BAR_SAME_IN_BOTH}__J.live-solved.json'))
    assert [os.path.basename(p) for _a, p in slots] == [
        f'{BAR_SAME_IN_BOTH}__J.live-solved.json', f'{BAR_SAME_IN_BOTH}__R.json']
    assert 'M3' in cycle_roles(slots)


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


def _load_schedule_action(name: str):
    """Load one action file of the multi-robot export.

    Args:
        name (str): File stem, e.g. ``'B3__H'``.

    Returns:
        BarSceneAction: The loaded action.
    """
    return parse_bar_action(os.path.join(_actions_dir(SCHEDULE_PROBLEM), f'{name}.json'))


# * ------------------------------------------------- hold-id guard on roles

def test_hold_ids_have_no_role():
    """H / HR ids must not fall through to the legacy '_M<n>_' pattern."""
    for mid in ('B3_H_M0_free_to_approach', 'B3_H_M2_LM_to_grasp',
                'B3_H_M3_gripper_close', 'B3_HR_M0_gripper_open', 'B3_HR_M1_LM_retreat'):
        assert movement_role(_mv(mid)) is None, mid
    # Cindy's ids are unchanged.
    assert movement_role(_mv('B3_J_M5_LM_insert')) == 'M2'
    assert movement_role(_mv('B3_R_M2_LM_retreat')) == 'M3'
    assert movement_role(_mv('B3_M2_LM_mate')) == 'M2'


@needs_schedule_export
def test_roles_for_real_actions(capsys):
    """Cindy's halves keep their roles; a support robot's actions have none, silently."""
    assert roles_for_action(_load_schedule_action('B3__J')) == ['M0', None, None, 'M1', None, 'M2']
    assert roles_for_action(_load_schedule_action('B3__R')) == [None, None, 'M3', 'M4']
    hold, hold_release = _load_schedule_action('B3__H'), _load_schedule_action('B3__HR')
    assert roles_for_action(hold) == [None] * 4
    assert roles_for_action(hold_release) == [None] * 2
    # The multi-file form must not print a role mismatch for them either.
    assert cycle_roles([(hold, None), (hold_release, None)]) == [None] * 6
    assert capsys.readouterr().out == ''


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


def test_movement_controller_warns_on_disagreement(capsys):
    """Cindy's M2 runs compliant, every other role joint tracking."""
    insert = EndEffectorConstrainedDualArmLinearMovement(
        movement_id='B3_J_M5_LM_insert', controller='cartesian_compliant')
    assert movement_controller(insert, 'M2') == 'cartesian_compliant'
    assert movement_controller(insert) == 'cartesian_compliant'
    assert capsys.readouterr().out == ''

    retreat = IndependentDualArmLinearMovement(
        movement_id='B3_R_M2_LM_retreat', controller='cartesian_compliant')
    assert movement_controller(retreat, 'M3') == 'cartesian_compliant'   # export wins
    out = capsys.readouterr().out
    assert out.count('\n') == 1 and 'B3_R_M2_LM_retreat' in out
    # Support movements have no role: nothing to compare, nothing printed.
    assert movement_controller(SingleArmLinearMovement(controller='cartesian_compliant')) \
        == 'cartesian_compliant'
    assert capsys.readouterr().out == ''


def test_tool_event():
    """tool_event returns the (action, tool names, overlaps_next) of a tool movement."""
    close = GripperToolMovement(tool_action='close', tool_names=['SupportGripper'])
    assert tool_event(close) == ('close', ['SupportGripper'], False)
    tighten = ScaffoldingToolMovement(tool_action='tighten', tool_names=['AT3L', 'AT3R'],
                                      overlaps_next=True)
    assert tool_event(tighten) == ('tighten', ['AT3L', 'AT3R'], True)


def test_default_trajectory_time():
    """The monitor's role table wins for Cindy; otherwise the per-kind default."""
    role_table = {'M0': 30.0, 'M4': 10.0}
    home = IndependentDualArmFreeMovement()
    assert default_trajectory_time(home) == 30.0
    assert default_trajectory_time(home, role='M4', role_table=role_table) == 10.0
    assert default_trajectory_time(home, role='M9', role_table=role_table) == 30.0
    assert default_trajectory_time(EndEffectorConstrainedDualArmFreeMovement()) == 10.0
    assert default_trajectory_time(SingleArmFreeMovement()) == 15.0
    assert default_trajectory_time(SingleArmLinearMovement()) == 5.0
    assert default_trajectory_time(GripperToolMovement()) is None
    assert default_trajectory_time(ManualMovement(), role='M1', role_table=role_table) is None


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
