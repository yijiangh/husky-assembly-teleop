"""Checks for the as-built record file (numpy + pytest only: no ROS, no pybullet, no compas)."""
import json
from datetime import datetime

import numpy as np
import pytest

from husky_assembly_teleop.as_built_io import (
    AS_BUILT_SCHEMA_VERSION, append_as_built_record, as_built_path, build_as_built_record, json_safe,
    milestone_of, read_as_built,
)

# * The movement kinds of the 261016_2ModulesDemo_YH actions, in file order.
J_GROUNDED = ['dual_free', 'manual', 'scaffolding_tool', 'dual_constrained_free', 'dual_constrained_linear', 'manual']
J_MATING = ['dual_free', 'manual', 'scaffolding_tool', 'dual_constrained_free', 'scaffolding_tool',
            'dual_constrained_linear']
R = ['scaffolding_tool', 'dual_independent_linear', 'dual_free']
H = ['single_free', 'gripper_tool', 'single_linear', 'gripper_tool']
HR = ['gripper_tool', 'single_linear']


class _HasToJson:
    """Stands in for BarFit / PoseDeviation: anything with a to_json()."""

    def to_json(self):
        return {'ok': True, 'pose': np.eye(4)}


def _record(**changes):
    """A small record with numpy inside, as the monitor hands it over."""
    pieces = dict(
        problem='P', run_id='Cindy-20261020T100000', robot='Cindy',
        action={'index': 0, 'action_id': 'B20_J_joint', 'kind': 'J', 'bar_id': 'B20'},
        action_file='B20__J.json',
        step={'index': 3, 'movement_id': 'B20_J_M3_CDFM_transfer_to_approach', 'kind': 'dual_constrained_free',
              'phase': 'end', 'outcome': 'ok', 'arms_off_end_rad': np.float64(0.001), 'commanded_end': None},
        bars={'bar_B20': {'T_reference': np.eye(4), 'fit': _HasToJson(), 'ids': {np.int64(3): (1.0, 2.0)}}},
        robot_state={'base_pose': (np.zeros(3), np.array([0, 0, 0, 1.0]))},
        beliefs={}, instrumented_bars=['bar_B20', 'bar_B21'], mocap_markers_path='/x/MoCapMarkers.json',
        now=datetime(2026, 10, 20, 10, 0, 5),
    )
    pieces.update(changes)
    return build_as_built_record(**pieces)


def test_paths():
    """The file of a day sits in <root>/<YYYYMMDD>-asbuilt/, named after the problem."""
    path = as_built_path('/x', 'P', day=datetime(2026, 10, 20))
    assert path == '/x/20261020-asbuilt/as_built_P.jsonl'


def test_record_is_json_safe():
    """numpy arrays, numpy scalars and keys, tuples and to_json() objects all become plain JSON."""
    record = _record()
    text = json.dumps(record)
    assert record['schema_version'] == AS_BUILT_SCHEMA_VERSION
    assert record['stamp'] == '2026-10-20T10:00:05'
    assert isinstance(record['stamp_epoch'], float)
    bar = record['bars']['bar_B20']
    assert bar['T_reference'] == np.eye(4).tolist()
    assert bar['fit'] == {'ok': True, 'pose': np.eye(4).tolist()}
    assert bar['ids'] == {3: [1.0, 2.0]}
    assert '"ids": {"3": [1.0, 2.0]}' in text
    assert json_safe((np.int64(2), np.float32(0.5))) == [2, 0.5]


def test_bad_phase_or_outcome_raises():
    """A phase or outcome the page would not understand is refused at once."""
    with pytest.raises(ValueError, match='phase'):
        _record(step={'phase': 'middle', 'outcome': None})
    with pytest.raises(ValueError, match='outcome'):
        _record(step={'phase': 'end', 'outcome': 'done'})
    assert _record(step=None)['step'] == {'phase': 'manual', 'outcome': None}


def test_append_and_read_round_trip(tmp_path):
    """Two appended records read back in order, unchanged (integer keys come back as strings, as JSON does)."""
    path = str(tmp_path / '20261020-asbuilt' / 'as_built_P.jsonl')
    first, second = _record(), _record(step={'phase': 'manual', 'outcome': None})
    append_as_built_record(path, first)
    append_as_built_record(path, second)
    assert read_as_built(path) == [json.loads(json.dumps(first)), json.loads(json.dumps(second))]


def test_read_tolerates_a_torn_last_line_only(tmp_path):
    """A last line cut short is dropped; a broken line in the middle raises; a missing file is empty."""
    path = tmp_path / 'as_built_P.jsonl'
    good = json.dumps(_record())
    path.write_text(good + '\n' + good[:40])
    assert len(read_as_built(str(path))) == 1
    path.write_text(good[:40] + '\n' + good + '\n')
    with pytest.raises(ValueError, match='line 1'):
        read_as_built(str(path))
    assert read_as_built(str(tmp_path / 'missing.jsonl')) == []


@pytest.mark.parametrize('action_kind, kinds, index, phase, expected', [
    ('J', J_GROUNDED, 3, 'end', 'before_insert'),
    ('J', J_GROUNDED, 3, 'start', None),
    ('J', J_GROUNDED, 4, 'end', 'after_insert'),
    ('J', J_GROUNDED, 5, 'end', 'after_fix'),
    ('J', J_GROUNDED, 1, 'end', None),             # the manual mount comes before the insert
    ('J', J_GROUNDED, 0, 'end', None),
    ('J', J_MATING, 3, 'end', 'before_insert'),
    ('J', J_MATING, 5, 'end', 'after_insert'),
    ('R', R, 1, 'start', 'before_retreat'),
    ('R', R, 1, 'end', 'after_retreat'),
    ('R', R, 2, 'end', None),
    ('H', H, 0, 'end', 'hold_approach'),
    ('H', H, 1, 'end', None),                      # the gripper opens before the move in
    ('H', H, 2, 'end', 'hold_at_grasp'),
    ('H', H, 3, 'end', 'hold_grasped'),
    ('HR', HR, 0, 'end', 'hold_released'),
    ('HR', HR, 1, 'end', 'hold_retreated'),
    ('J', J_GROUNDED, 4, 'manual', None),
    (None, [], None, 'manual', None),
])
def test_milestones_of_the_active_bar(action_kind, kinds, index, phase, expected):
    """The action's own bar gets the moments the page puts first."""
    assert milestone_of(action_kind, kinds, index, phase, is_active_bar=True) == expected


def test_every_other_bar_is_a_bystander():
    """A bar measured while another bar's action runs is followed as a bystander, whatever the step."""
    for phase in ('start', 'end', 'manual'):
        assert milestone_of('J', J_MATING, 5, phase, is_active_bar=False) == 'bystander'
