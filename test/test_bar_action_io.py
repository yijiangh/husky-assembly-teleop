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

from husky_assembly_teleop import DESIGN_DATA_DIRECTORY
from husky_assembly_teleop.bar_action_io import (
    BarAssemblyAction, movement_role, sibling_action_path, list_bar_actions,
    load_action_cycle, cycle_roles, slot_of_index, cycle_start_ee_sources,
)

# The two exports under test, one per schema. 260715 writes one file per bar
# holding M0..M4; 260929 splits each bar into a jointing and a release half.
LEGACY_PROBLEM = '260715_phase1_test'
SPLIT_PROBLEM = '260929_phase1_retest'

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
