"""Checks for the old M0..M4 roles (``legacy_bar_action_io``), no cell, no planner.

Moved here unchanged from ``test_bar_action_io.py`` when the monitor stopped
using roles: only offline tools that read old takes still need them. The id
checks are pure python; the loader checks read the REAL exports and are skipped
where the shared drive is not mounted.
"""
import os
import shutil
import types

import pytest

from rs_data_structure.bar_action import BarAssemblyReleaseAction, IndependentDualArmLinearMovement

from husky_assembly_teleop import DESIGN_DATA_DIRECTORY
from husky_assembly_teleop.bar_action_io import (
    cycle_start_ee_sources, load_action_cycle, parse_bar_action,
)
from husky_assembly_teleop.legacy_bar_action_io import (
    cycle_roles, movement_role, roles_for_action,
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


needs_schedule_export = pytest.mark.skipif(
    not os.path.isdir(_actions_dir(SCHEDULE_PROBLEM)),
    reason='the multi-robot design-study export is not on this machine',
)


def _mv(movement_id):
    return types.SimpleNamespace(movement_id=movement_id)


def _load_schedule_action(name: str):
    """Load one action file of the multi-robot export.

    Args:
        name (str): File stem, e.g. ``'B3__H'``.

    Returns:
        BarSceneAction: The loaded action.
    """
    return parse_bar_action(os.path.join(_actions_dir(SCHEDULE_PROBLEM), f'{name}.json'))


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


def test_hold_ids_have_no_role():
    """H / HR ids must not fall through to the legacy '_M<n>_' pattern."""
    for mid in ('B3_H_M0_free_to_approach', 'B3_H_M2_LM_to_grasp',
                'B3_H_M3_gripper_close', 'B3_HR_M0_gripper_open', 'B3_HR_M1_LM_retreat'):
        assert movement_role(_mv(mid)) is None, mid
    # Cindy's ids are unchanged.
    assert movement_role(_mv('B3_J_M5_LM_insert')) == 'M2'
    assert movement_role(_mv('B3_R_M2_LM_retreat')) == 'M3'
    assert movement_role(_mv('B3_M2_LM_mate')) == 'M2'


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


# * ------------------------------------------------- roles from the classes

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


def test_no_mismatch_print_for_split_ids(capsys):
    """A split id's number is its position in the file, so it is not compared with the class.

    ``B1_R_M1_LM_retreat`` reads as no role by its id (release position 1 has
    none in the translation table), but its class is the retreat: the class
    wins, and nothing is printed.
    """
    retreat = IndependentDualArmLinearMovement(movement_id='B1_R_M1_LM_retreat')
    assert movement_role(retreat) is None
    assert cycle_roles([(BarAssemblyReleaseAction(movements=[retreat]), None)]) == ['M3']
    assert capsys.readouterr().out == ''
