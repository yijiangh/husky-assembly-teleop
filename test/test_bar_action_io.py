"""Pure-python checks for the bar action loader helpers (no cell, no planner)."""
import types

import rs_data_structure.bar_action as bar_action_module

from husky_assembly_teleop.bar_action_io import (
    BarAssemblyAction, movement_role, sibling_action_path,
)


def _mv(movement_id):
    return types.SimpleNamespace(movement_id=movement_id)


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
