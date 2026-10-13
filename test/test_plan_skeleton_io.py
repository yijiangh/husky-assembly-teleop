"""Checks for the PlanSkeleton reader (no cell, no planner, no ROS).

Most checks read the REAL multi-robot export (``ActionSchedule.json`` and a few
``BarActions/*.json``, never the 340 MB ``RobotCell*.json``), so they also catch
the exporter changing shape. Anything that writes works on a copy in ``tmp_path``.
"""
import json
import os
import shutil

import pytest

from husky_assembly_teleop import DESIGN_DATA_DIRECTORY
from husky_assembly_teleop.bar_action_io import LIVE_SOLVED_TAG, MovementKind
from husky_assembly_teleop.robot_registry import ROBOTS
from husky_assembly_teleop.plan_skeleton_io import (
    SKELETON_FILENAME, PlanSkeleton, SkeletonAction, is_executable_by, load_skeleton_action,
    load_plan_skeleton, problem_root, skeleton_path,
)

SKELETON_PROBLEM = '260920_RobArch_demo_revamp_backup'
FIXTURE_ROOT = os.path.join(DESIGN_DATA_DIRECTORY, SKELETON_PROBLEM)

needs_fixture = pytest.mark.skipif(
    not os.path.isfile(os.path.join(FIXTURE_ROOT, SKELETON_FILENAME)),
    reason='the multi-robot design-study export is not on this machine',
)


@pytest.fixture(scope='module')
def skeleton():
    """The fixture problem's plan skeleton, loaded once."""
    return load_plan_skeleton(FIXTURE_ROOT)


def _skeleton_dict() -> dict:
    """The fixture's raw ActionSchedule.json.

    Returns:
        dict: Parsed JSON.
    """
    with open(os.path.join(FIXTURE_ROOT, SKELETON_FILENAME)) as handle:
        return json.load(handle)


def _copy_problem(tmp_path, names: list) -> str:
    """Copy the plan skeleton and some action files into a scratch problem folder.

    Args:
        tmp_path: pytest's temporary folder.
        names (list): Action file names to copy, e.g. ``['B3__H.json']``.

    Returns:
        str: The scratch problem folder.
    """
    root = tmp_path / SKELETON_PROBLEM
    (root / 'BarActions').mkdir(parents=True)
    shutil.copy(os.path.join(FIXTURE_ROOT, SKELETON_FILENAME), root / SKELETON_FILENAME)
    for name in names:
        shutil.copy(os.path.join(FIXTURE_ROOT, 'BarActions', name), root / 'BarActions' / name)
    return str(root)


# * ------------------------------------------------------------- pure python

def test_action_kind_comes_from_the_type_not_the_id():
    """An action's kind follows its action type, even when the id suggests another kind."""
    action = SkeletonAction(index=0, action_id='B3_J_looks_like_jointing', type='BarHoldingAction',
                            bar_id='B3', robot='Alice', file='BarActions/B3__H.json')
    assert action.kind == 'H' and action.is_support
    assert SkeletonAction.from_dict(action.to_dict()) == action
    assert is_executable_by(action, 'Alice') and not is_executable_by(action, 'Cindy')


def test_paths_and_missing_skeleton(tmp_path):
    """A problem without ActionSchedule.json is a legacy problem: None, not an error."""
    assert skeleton_path('/p/prob') == os.path.join('/p/prob', 'ActionSchedule.json')
    assert problem_root('prob', design_dir='/d') == os.path.join('/d', 'prob')
    assert problem_root(SKELETON_PROBLEM) == FIXTURE_ROOT
    assert load_plan_skeleton(str(tmp_path)) is None


def _action_dict(index: int, bar_id: str, type_name: str, robot: str, suffix: str) -> dict:
    """One synthetic plan skeleton item.

    Args:
        index (int): Plan skeleton index.
        bar_id (str): e.g. ``'B3'``.
        type_name (str): Action class name, e.g. ``'BarHoldingAction'``.
        robot (str): Short robot name.
        suffix (str): File suffix, e.g. ``'H'`` for ``BarActions/B3__H.json``.

    Returns:
        dict: The item as the file would hold it.
    """
    return {'index': index, 'action_id': f'{bar_id}_{suffix}', 'type': type_name,
            'bar_id': bar_id, 'robot': robot, 'file': f'BarActions/{bar_id}__{suffix}.json'}


def test_hold_without_any_end_is_rejected_at_load():
    """A hold with no HR action and no R action for its last bar has no end: ValueError at load."""
    data = {
        'schema_version': 1,
        'robots': {name: {'robot_id': ROBOTS[name].robot_id, 'role': role}
                   for name, role in (('Cindy', 'assembly'), ('Alice', 'support'))},
        'assembly_seq': ['B1', 'B3', 'B9'],
        'holds': [{'bar_id': 'B3', 'robot': 'Alice', 'release_after_bar_id': 'B9'}],
        'schedule': [
            _action_dict(0, 'B3', 'BarAssemblyJointingAction', 'Cindy', 'J'),
            _action_dict(1, 'B3', 'BarHoldingAction', 'Alice', 'H'),
            _action_dict(2, 'B9', 'BarAssemblyJointingAction', 'Cindy', 'J'),
        ],
    }
    with pytest.raises(ValueError, match="no HR action and no R action for 'B9'"):
        PlanSkeleton.from_dict(data, '/p/prob')

    # With B9's release in the plan skeleton the hold ends right after it.
    data['schedule'].append(_action_dict(3, 'B9', 'BarAssemblyReleaseAction', 'Cindy', 'R'))
    synthetic = PlanSkeleton.from_dict(data, '/p/prob')
    assert synthetic.hold_end_index(synthetic.hold_window('B3')) == 4


def test_open_hold_of_a_range_export():
    """A range export's hold released after a bar beyond the range is on until the end.

    Same shape as the exporter's range bundles: ``assembly_seq`` stops at the
    range's last bar, and the hold names a later release bar with no release action.
    """
    data = {
        'schema_version': 1,
        'robots': {name: {'robot_id': ROBOTS[name].robot_id, 'role': role}
                   for name, role in (('Cindy', 'assembly'), ('Alice', 'support'))},
        'assembly_seq': ['B20', 'B21', 'B22'],
        'export_range': {'from_bar_id': 'B20', 'until_bar_id': 'B22'},
        'holds': [{'bar_id': 'B21', 'robot': 'Alice', 'release_after_bar_id': 'B37'}],
        'schedule': [
            _action_dict(0, 'B20', 'BarAssemblyJointingAction', 'Cindy', 'J'),
            _action_dict(1, 'B21', 'BarAssemblyJointingAction', 'Cindy', 'J'),
            _action_dict(2, 'B21', 'BarHoldingAction', 'Alice', 'H'),
            _action_dict(3, 'B21', 'BarAssemblyReleaseAction', 'Cindy', 'R'),
            _action_dict(4, 'B22', 'BarAssemblyJointingAction', 'Cindy', 'J'),
        ],
    }
    skeleton = PlanSkeleton.from_dict(data, '/p/prob')
    hold = skeleton.hold_window('B21')
    assert hold.release_after_seq is None and hold.release_action_index is None
    assert skeleton.hold_end_index(hold) == 5                       # past the last action
    assert skeleton.held_bar_ids(hold) == ['B21', 'B22']
    assert skeleton.robots_holding_at(skeleton.action(4)) == {'Alice': 'B21'}
    assert 'Alice' not in skeleton.robots_holding_at(skeleton.action(2))   # she is acting

    # Without export_range the same file is an export error, as before.
    del data['export_range']
    with pytest.raises(ValueError, match="names 'B37', which is not in assembly_seq"):
        PlanSkeleton.from_dict(data, '/p/prob')


# * ------------------------------------------------------------- the real plan skeleton

@needs_fixture
def test_skeleton_table(skeleton):
    """The fixture's robots, sequence and actions, as the exporter wrote them."""
    assert skeleton.problem_name == SKELETON_PROBLEM
    assert skeleton.schema_version == 1
    assert sorted(skeleton.robots) == ['Alice', 'Belle', 'Cindy']
    assert len(skeleton.assembly_seq) == 20 and len(skeleton.actions) == 48
    assert [e.action_id for e in skeleton.actions[:9]] == [
        'B1_J_joint', 'B1_R_release', 'B3_J_joint', 'B3_H_hold', 'B3_R_release',
        'B4_J_joint', 'B4_R_release', 'B5_J_joint', 'B5_R_release']
    assert [e.kind for e in skeleton.actions[:9]] == ['J', 'R', 'J', 'H', 'R', 'J', 'R', 'J', 'R']
    assert skeleton.action(16).action_id == 'B3_HR_hold_release' and skeleton.action(16).kind == 'HR'
    assert [e.index for e in skeleton.actions_for_robot('Alice')] == [3, 16, 21, 40]
    assert [e.index for e in skeleton.actions_for_robot('Belle')] == [10, 17, 26, 41]
    assert [e.index for e in skeleton.actions_for_bar('B3')] == [2, 3, 4, 16]
    assert skeleton.find_action('B3', 'HR').index == 16
    assert skeleton.find_action('B1', 'H') is None
    assert skeleton.seq_position('B3') == 1


@needs_fixture
def test_predecessors(skeleton):
    """R starts from its J, HR from its H; J and H have no predecessor."""
    assert skeleton.predecessor(skeleton.action(4)).index == 2      # B3_R <- B3_J
    assert skeleton.predecessor(skeleton.action(16)).index == 3     # B3_HR <- B3_H
    assert skeleton.predecessor(skeleton.action(2)) is None
    assert skeleton.predecessor(skeleton.action(3)) is None


@needs_fixture
def test_hold_windows(skeleton):
    """Hold windows resolve to sequence positions, action indices and the bars built meanwhile."""
    assert [(h.bar_id, h.robot, h.release_after_bar_id) for h in skeleton.holds] == [
        ('B3', 'Alice', 'B9'), ('B7', 'Belle', 'B9'),
        ('B12', 'Alice', 'B21'), ('B15', 'Belle', 'B21')]
    hold = skeleton.hold_window('B3')
    assert (hold.hold_start_seq, hold.release_after_seq) == (1, 6)
    assert (hold.hold_action_index, hold.release_action_index) == (3, 16)
    assert skeleton.hold_end_index(hold) == 16
    assert skeleton.held_bar_ids(hold) == ['B3', 'B4', 'B5', 'B7', 'B8', 'B9']
    with pytest.raises(KeyError):
        skeleton.hold_window('B1')


@needs_fixture
def test_who_is_holding(skeleton):
    """Alice holds B3 from after her hold action until her hold release action."""
    holding_alice = [i for i in range(len(skeleton.actions))
                     if skeleton.robots_holding_at(skeleton.action(i)).get('Alice') == 'B3']
    assert holding_alice == list(range(4, 16))
    assert 'Alice' not in skeleton.robots_holding_at(skeleton.action(3))    # she is acting
    assert 'Alice' not in skeleton.robots_holding_at(skeleton.action(16))   # she is releasing
    assert skeleton.robots_holding_at(skeleton.action(11)) == {'Alice': 'B3', 'Belle': 'B7'}
    assert skeleton.robots_holding_at(skeleton.action(0)) == {}


@needs_fixture
def test_executable_by(skeleton):
    """A robot runs only its own actions; an unknown robot is a KeyError."""
    assert [e.index for e in skeleton.executable_by('Alice')] == [3, 16, 21, 40]
    assert len(skeleton.executable_by('Cindy')) == 40
    with pytest.raises(KeyError):
        skeleton.executable_by('Dora')


@needs_fixture
def test_action_path(skeleton):
    """Without a sidecar on disk both path choices give the clean export."""
    expected = os.path.join(FIXTURE_ROOT, 'BarActions', 'B3__H.json')
    assert skeleton.action_path(skeleton.action(3)) == expected   # no sidecar in the fixture
    assert skeleton.action_path(skeleton.action(3), prefer_sidecar=False) == expected


# * ------------------------------------------------------------- loaded actions

@needs_fixture
def test_release_starts_where_the_jointing_left_the_flanges(skeleton):
    """B3_R's tool steps and retreat all start at B3_J's insert targets."""
    loaded = load_skeleton_action(skeleton, skeleton.action(4))
    assert loaded.predecessor.action.index == 2 and loaded.predecessor.predecessor is None
    # * The layout is read off the file: tool steps, then the retreat and the
    # * free move home. A re-export may add or drop a tool step.
    kinds = loaded.kinds
    assert kinds[-2:] == [MovementKind.DUAL_INDEPENDENT_LINEAR, MovementKind.DUAL_FREE]
    assert all(kind is MovementKind.SCAFFOLDING_TOOL for kind in kinds[:-2])
    for idx in range(len(kinds) - 1):
        for side in ('left', 'right'):
            assert loaded.start_ee_source(idx, side).movement_id == 'B3_J_M5_LM_insert'
    retreat = kinds.index(MovementKind.DUAL_INDEPENDENT_LINEAR)
    assert set(loaded.start_ee_frames(retreat)) == {'left', 'right'}
    assert loaded.missing_start_sides(retreat) == []


@needs_fixture
def test_hold_release_starts_at_the_hold_approach(skeleton):
    """B3_HR's movements start where B3_H's linear approach left Alice's flange."""
    loaded = load_skeleton_action(skeleton, skeleton.action(16))
    assert loaded.spec.name == 'Alice'
    assert loaded.kinds == [MovementKind.GRIPPER_TOOL, MovementKind.SINGLE_LINEAR]
    for idx in (0, 1):
        assert loaded.start_ee_source(idx, 'arm').movement_id == 'B3_H_M2_LM_to_grasp'


@needs_fixture
def test_hold_starts_unknown_then_from_its_own_approach(skeleton):
    """B3_H's first movement has no known start; later ones start at its own approach."""
    loaded = load_skeleton_action(skeleton, skeleton.action(3))
    assert loaded.predecessor is None
    assert loaded.start_ee_source(0, 'arm') is None
    assert loaded.missing_start_sides(0) == ['arm'] and loaded.start_ee_frames(0) == {}
    assert loaded.start_ee_source(2, 'arm').movement_id == 'B3_H_M0_free_to_approach'
    assert len(loaded.movements) == 4


@needs_fixture
def test_without_predecessor(skeleton):
    """Skipping the predecessor leaves a release's start poses unknown."""
    loaded = load_skeleton_action(skeleton, skeleton.action(4), with_predecessor=False)
    assert loaded.predecessor is None
    assert loaded.start_ee_source(2, 'left') is None


# * ------------------------------------------------------------- edited copies

@needs_fixture
def test_hold_end_without_a_hold_release_action():
    """Without HR actions, a hold ends right after the release of its last bar."""
    data = _skeleton_dict()
    kept = [e for e in data['schedule'] if e['type'] != 'BarHoldingReleaseAction']
    for i, e in enumerate(kept):
        e['index'] = i
    data['schedule'] = kept
    synthetic = PlanSkeleton.from_dict(data, FIXTURE_ROOT)

    hold = synthetic.hold_window('B3')
    assert hold.release_action_index is None
    b9_release = synthetic.find_action('B9', 'R').index
    assert synthetic.hold_end_index(hold) == b9_release + 1
    assert 'Alice' in synthetic.robots_holding_at(synthetic.action(b9_release))
    assert 'Alice' not in synthetic.robots_holding_at(synthetic.action(b9_release + 1))


@needs_fixture
def test_invalid_skeletons_are_rejected():
    """Each broken rule of the file raises ValueError at load."""
    def broken(edit):
        """Apply ``edit`` to a fresh copy of the file's dict and expect ValueError."""
        data = _skeleton_dict()
        edit(data)
        with pytest.raises(ValueError):
            PlanSkeleton.from_dict(data, FIXTURE_ROOT)

    broken(lambda d: d.update(schema_version=2))
    broken(lambda d: d['schedule'][5].update(index=99))
    broken(lambda d: d['schedule'][5].update(type='BarTeleportAction'))
    broken(lambda d: d['schedule'][5].update(robot='Dora'))
    broken(lambda d: d['holds'][0].update(release_after_bar_id='B999'))


@needs_fixture
def test_sidecar_is_preferred(tmp_path):
    """A live-solved sidecar on disk wins over the clean export, unless asked otherwise."""
    root = _copy_problem(tmp_path, ['B3__H.json'])
    clean = os.path.join(root, 'BarActions', 'B3__H.json')
    sidecar = os.path.join(root, 'BarActions', f'B3__H.{LIVE_SOLVED_TAG}.json')
    with open(clean) as handle:
        data = json.load(handle)
    data['data']['tag'] = 'written by the monitor'
    with open(sidecar, 'w') as handle:
        json.dump(data, handle)

    copy = load_plan_skeleton(root)
    action = copy.action(3)
    assert copy.action_path(action) == sidecar
    assert copy.action_path(action, prefer_sidecar=False) == clean
    assert copy.load_bar_action(action).tag == 'written by the monitor'
    assert copy.load_bar_action(action, prefer_sidecar=False).tag != 'written by the monitor'
    assert load_skeleton_action(copy, action).path == sidecar


@needs_fixture
def test_file_that_does_not_match_its_action(tmp_path):
    """Wrong action type, or right type for the wrong robot -> ValueError."""
    root = _copy_problem(tmp_path, ['B3__HR.json', 'B7__H.json'])
    actions_dir = os.path.join(root, 'BarActions')
    copy = load_plan_skeleton(root)

    shutil.copy(os.path.join(actions_dir, 'B3__HR.json'), os.path.join(actions_dir, 'B3__H.json'))
    with pytest.raises(ValueError, match='BarHoldingReleaseAction'):
        copy.load_bar_action(copy.action(3))

    shutil.copy(os.path.join(actions_dir, 'B7__H.json'), os.path.join(actions_dir, 'B3__H.json'))
    with pytest.raises(ValueError, match='Belle'):
        copy.load_bar_action(copy.action(3))


@needs_fixture
def test_load_skeleton_action_refuses_two_transfers(tmp_path):
    """A jointing file whose insert became a second transfer is refused, naming the file."""
    root = _copy_problem(tmp_path, ['B1__J.json'])
    path = os.path.join(root, 'BarActions', 'B1__J.json')
    with open(path) as handle:
        text = handle.read()
    # Exactly one insert in the file, and both classes have the same fields.
    assert text.count('EndEffectorConstrainedDualArmLinearMovement') == 1
    with open(path, 'w') as handle:
        handle.write(text.replace('EndEffectorConstrainedDualArmLinearMovement',
                                  'EndEffectorConstrainedDualArmFreeMovement'))

    copy = load_plan_skeleton(root)
    with pytest.raises(ValueError, match='B1__J.json'):
        load_skeleton_action(copy, copy.action(0))
