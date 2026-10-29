"""Checks for the as-built assembly page's numbers (4_assembly_viewer.py), on the 2-modules export.

Skipped where the shared drive (the design problem) is not on this machine.
"""
import importlib.util
import json
import os

import numpy as np
import pytest

from husky_assembly_teleop import DESIGN_DATA_DIRECTORY
from husky_assembly_teleop.bar_mocap_fit import fit_bar_from_cache, load_mocap_markers, transform_points
from husky_assembly_teleop.plan_skeleton_io import load_plan_skeleton

PROBLEM = '261016_2ModulesDemo_YH'
PROBLEM_DIR = os.path.join(DESIGN_DATA_DIRECTORY, PROBLEM)
VIEWER = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'data', 'bar_holding_acc_data', '4_assembly_viewer.py')

pytestmark = pytest.mark.skipif(not os.path.exists(os.path.join(PROBLEM_DIR, 'MoCapMarkers.json')),
                                reason=f'{PROBLEM} is not on this machine')


@pytest.fixture(scope='module')
def viewer():
    """The page script as a module (its file name starts with a digit, so it is loaded by path)."""
    spec = importlib.util.spec_from_file_location('assembly_viewer', VIEWER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_timeline_follows_the_plan_skeleton(viewer):
    """Which bars exist and who holds them, at the actions the user asked about."""
    actions = viewer.timeline(load_plan_skeleton(PROBLEM_DIR))
    by_id = {a['action_id']: a for a in actions}
    # * Action 5 (B22_J): B20 placed, B21 held by Alice, B22 being built.
    assert by_id['B22_J_joint']['exists'] == ['B20', 'B21', 'B22']
    assert by_id['B22_J_joint']['held'] == {'B21': 'Alice'}
    # * Action 6 (B22_R): Cindy releases B22, Alice still holds B21.
    assert by_id['B22_R_release']['exists'] == ['B20', 'B21', 'B22']
    assert by_id['B22_R_release']['held'] == {'B21': 'Alice'}
    # * Action 3 (B21_H): Alice grabs B21 while Cindy still holds it (its R comes next).
    assert by_id['B21_H_hold']['held'] == {'B21': 'Cindy'}
    # * Action 16 (B21_HR): Alice lets go (her own action); Belle still holds B35.
    assert by_id['B21_HR_hold_release']['held'] == {'B35': 'Belle'}
    assert by_id['B20_J_joint']['exists'] == ['B20']


def test_history_leaves_out_records_before_the_bar_was_placed(viewer):
    """The bar on its loading spot or standing by before its first milestone is counted, not listed."""
    history = {}
    kept = [viewer.add_history(history, {'bar': 'B20', 'milestone': m}, {'milestone': m})
            for m in (None, 'bystander', 'before_insert', None, 'bystander', 'after_retreat')]
    assert kept == [False, False, True, True, True, True]
    assert history['B20']['skipped'] == 2
    assert [e['milestone'] for e in history['B20']['entries']] == ['before_insert', None, 'bystander', 'after_retreat']


def test_history_leaves_out_a_cancelled_step_before_the_bar_was_placed(viewer):
    """A cancelled transfer's milestone does not place the bar (B20 on 2026-10-23 was not mounted yet)."""
    history = {}
    kept = [viewer.add_history(history, {'bar': 'B20', 'milestone': m}, {'milestone': m, 'outcome': o})
            for m, o in (('before_insert', 'cancelled'), (None, None), ('before_insert', 'ok'), (None, None))]
    assert kept == [False, False, True, True]
    assert history['B20']['skipped'] == 2


def test_a_release_run_under_another_bars_r_goes_to_the_held_bar(viewer):
    """23 Oct 18:12: Cindy ran "B20_R" while she held B34; the record belongs to B34's R."""
    actions = [{'index': 0, 'action_id': 'B20_R_release', 'kind': 'R', 'bar_id': 'B20'},
               {'index': 1, 'action_id': 'B34_R_release', 'kind': 'R', 'bar_id': 'B34'}]
    record = {'robot': 'Cindy', 'action': {'index': 0},
              'bars': {'bar_B20': {'bar_id': 'B20', 'milestone': 'after_retreat'},
                       'bar_B34': {'bar_id': 'B34', 'milestone': 'bystander'}}}
    k, moved, note = viewer.release_as_run(record, actions, {'Cindy': 'B34'})
    assert k == 1 and 'B34' in note
    assert moved['bars']['bar_B34']['milestone'] == 'after_retreat'
    assert moved['bars']['bar_B20']['milestone'] == 'bystander'
    assert record['bars']['bar_B20']['milestone'] == 'after_retreat'    # the read record is left as it was
    # * The R of the held bar itself stays where it is.
    assert viewer.release_as_run(record, actions, {'Cindy': 'B20'}) == (0, record, None)


def test_step_options_say_which_tick_box_each_step_used(viewer):
    """The monitor's log lines and the servo file decide each tick box, the way 2026-10-23 ran."""
    events = [(0.0, 'spiral_box', ('ON',)), (0.0, 'servo_box', ('MoCap bar',)),
              (10.0, 'servo_box', ('MoCap bar',)),    # the transfer reads the box at its start
              (12.0, 'mount_pause', ('B20_J_M3_CDFM_transfer_to_approach',)),
              (40.0, 'servo_box_insert', ('B20_J_M4_LM_insert', 'MoCap bar')),
              (60.0, 'spiral_ran', ('B21_J_M5_LM_insert',))]

    def options(log, kind, movement_id, step_kind, t0, t1, servo):
        """The step's boxes by key."""
        out = viewer.step_options(log, kind, {'movement_id': movement_id, 'kind': step_kind}, t0, t1, servo)
        return {o['key']: o for o in out}

    transfer = options(events, 'J', 'B20_J_M3_CDFM_transfer_to_approach', 'dual_constrained_free', 10.0, 20.0,
                       {'feedback': 'mocap_bar'})
    assert transfer['free_move']['used'] and transfer['servo_bar']['used']
    # * A grounded insert: the servo after it aims the bar; the ticked spiral box does nothing (rigid insert).
    grounded = options(events, 'J', 'B20_J_M4_LM_insert', 'dual_constrained_linear', 38.0, 45.0,
                       {'feedback': 'mocap_bar'})
    assert grounded['servo_bar']['used']
    assert grounded['spiral']['box'] and grounded['spiral']['used'] is False and grounded['spiral']['shown']
    screwed = options(events, 'J', 'B21_J_M5_LM_insert', 'dual_constrained_linear', 59.0, 70.0, None)
    assert screwed['spiral']['used'] and not screwed['servo_bar']['used']
    # * A retreat never spirals, but a ticked box is still listed, as not used.
    retreat = options(events, 'R', 'B20_R_M1_LM_retreat', 'dual_independent_linear', 80.0, 90.0, None)
    assert retreat['spiral']['shown'] and retreat['spiral']['used'] is False and not retreat['free_move']['shown']
    support = options([(0.0, 'servo_box', ('tool0',))], 'H', 'B21_H_M0_free_to_approach', 'single_free', 5.0, 9.0,
                      {'feedback': 'tool0'})
    assert support['servo_bar']['shown'] and not support['servo_bar']['used']
    assert 'aims its flange' in support['servo_bar']['why']
    # * No monitor log: nothing is claimed.
    unknown = options(None, 'J', 'B21_J_M5_LM_insert', 'dual_constrained_linear', 0.0, 1.0, None)
    assert unknown['spiral']['used'] is None and unknown['spiral']['box'] is None


def _plate(joint, z, err, shift, bow=None):
    """One plate row as `pair_geometry` writes it (only the fields the verdict reads)."""
    return {'joint': joint, 'n_seen': 2, 'z_mm': z, 'length_mm': 48.3 + err, 'length_err_mm': err,
            'shift_mm': shift, 'bow_mm': bow, 'bend_mm': None if bow is None else float(np.linalg.norm(bow))}


def _pairs(err_far=0.0, shift_near=(0.0, 0.0, 0.0), bow=(0.0, 0.0)):
    """A 3-plate bar's plates: the far plate's ball distance off by err_far, the near plate shifted, the middle bowed."""
    return {'joints': [_plate('joint_A_mocap', 0.0, 0.1, list(shift_near)),
                       _plate('joint_B_mocap', 500.0, -0.1, [0.0, 0.0, 0.0], list(bow)),
                       _plate('joint_C_mocap', 1000.0, err_far, [0.0, 0.0, 0.0])], 'spacing': []}


def test_ball_verdict_tells_a_bad_ball_a_plate_offset_and_bending_apart(viewer):
    """The verdict's four cases, each from the numbers that pin it down."""
    def bar(pairs, rms=0.4):
        return {'ok': True, 'pairs': pairs, 'fit_rms': rms, 'ambiguous': False}

    # * Nothing wrong: noise only.
    day = viewer.ball_check([_pairs(), _pairs(), _pairs()])
    assert viewer.ball_findings(bar(_pairs()), day) == ['balls good, bar straight, fit tight']
    assert day['noise_mm'] == pytest.approx(0.0)

    # * A bad ball: the far plate's two balls 0.7 mm short in every record.
    day = viewer.ball_check([_pairs(err_far=-0.7), _pairs(err_far=-0.75), _pairs(err_far=-0.68)])
    lines = viewer.ball_findings(bar(_pairs(err_far=-0.7)), day)
    assert len(lines) == 1 and lines[0].startswith('bad ball on plate C') and 'in every record' in lines[0]

    # * A plate off the design table: the near plate 0.9 mm off, the same in every record; it explains the rms.
    shifted = _pairs(shift_near=(0.0, 0.9, 0.0))
    day = viewer.ball_check([shifted, shifted, shifted])
    lines = viewer.ball_findings(bar(shifted, rms=1.4), day)
    assert any(line.startswith('plates off the design table') and 'A 0.9' in line for line in lines)
    assert any('mostly the constant plate offsets' in line for line in lines)

    # * Bending: the middle plate's bow 1 mm away from its usual in one record.
    day = viewer.ball_check([_pairs(), _pairs(), _pairs(bow=(1.0, 0.0))])
    lines = viewer.ball_findings(bar(_pairs(bow=(1.0, 0.0))), day)
    assert any(line.startswith('bending at plate B') for line in lines)

    # * A large mismatch nothing explains: the fit itself is doubtful.
    lines = viewer.ball_findings(bar(_pairs(), rms=1.6), viewer.ball_check([_pairs()] * 3))
    assert any('the fit itself is doubtful' in line for line in lines)


def test_load_names_its_zero(viewer):
    """The load says what its readings are relative to, when the record saved it."""
    ft = {'left_force_N': 26.0, 'right_force_N': 24.0}
    assert viewer.load_numbers({'robot_state': {'tool_ft': ft}})['zero'] is None
    zero = {'at': '2026-10-21T10:00:00', 'note': 'at the mount pause (tools empty)'}
    load = viewer.load_numbers({'robot_state': {'tool_ft': dict(ft, zero=zero)}})
    assert load['zero'] == zero and load['imbalance'] == pytest.approx(2.0)


def test_bar_numbers_from_a_record(viewer):
    """A B20 fitted 2 mm off its seat (exact balls) reads 2 mm at the start, middle and end, nothing else."""
    markers = load_mocap_markers(PROBLEM_DIR)
    design = markers.bars['bar_B20']
    frame = viewer._find_populated_cell_state(PROBLEM_DIR)['bar_frames']['bar_B20']
    T_seat = viewer.matrix_from_frame(frame)
    seat = {'T': T_seat, 'length': 1.8,
            'ends': viewer.design_bar_axis_endpoints(T_seat, 1.8).tolist()}
    T_real = T_seat.copy()
    T_real[:3, 3] += [0.002, 0.0, 0.0]
    cache = {k + 1: {'pos': p.tolist(), 'size': 0.019, 'error': 0.0003, 'flags': 0}
             for k, p in enumerate(transform_points(T_real, design.points()))}
    fit = fit_bar_from_cache(cache, design)
    assert fit.ok, fit.reason
    entry = json.loads(json.dumps({
        'markerset_key': 'bar_B20', 'bar_B20': cache, 'bar_fit': fit.to_json(), 'bar_id': 'B20',
        'milestone': 'after_retreat', 'held_by': None, 'reference_kind': 'resting', 'T_reference': T_seat.tolist(),
        'T_link_from_bar': None, 'marker_source': 'asset', 'marker_age_s': 0.01}))
    bar = viewer.shape_bar(entry, {'robot_state': {}}, seat, design)
    for key in ('tip_start', 'tip_middle', 'tip_end', 'placement', 'placement_seat'):
        assert bar[key] == pytest.approx(2.0, abs=1e-6), key
    assert bar['rotation'] == pytest.approx(0.0, abs=1e-6)
    assert bar['bend'] == pytest.approx(0.0, abs=1e-6)
    assert bar['pair_length'] == pytest.approx(0.0, abs=1e-6)
    assert bar['spacing'] == pytest.approx(0.0, abs=1e-6)
    assert [b['state'] for b in bar['markers_sensed']] == ['matched'] * 8
    # * Exact balls: every ball sits on its design spot once the bar is fitted.
    assert all(b['residual_mm'] == pytest.approx(0.0, abs=1e-6) for b in bar['markers_sensed'])
    assert 'fk' not in bar     # nothing holds it: no robot-side numbers
    assert 'tip_mean' not in bar     # the card shows start / middle / end only
    # * The two frames drawn as axes (three.js order: the origin is numbers 12-14), no turn between them.
    assert bar['frame_fit'][12:15] == pytest.approx(T_real[:3, 3], abs=1e-5)
    assert bar['frame_ref'][12:15] == pytest.approx(T_seat[:3, 3], abs=1e-5)
    assert bar['turn_ref_deg'] == pytest.approx([0.0, 0.0, 0.0], abs=1e-6)
    # * The bow's direction on the inner plates only (straight bar: zero).
    joints = bar['pairs']['joints']
    assert joints[0]['bow_mm'] is None and joints[-1]['bow_mm'] is None
    assert all(j['bow_mm'] == pytest.approx([0.0, 0.0], abs=1e-6) for j in joints[1:-1])
    assert viewer.ball_findings(bar) == ['balls good, bar straight, fit tight']
    json.dumps(viewer.finite(bar))


def test_metrics_follow_the_card(viewer):
    """The pickers list the measurements in the card's order, each one a key the page can read."""
    assert [m['key'] for m in viewer.METRICS] == [
        'placement', 'rotation', 'tool0', 'servo', 'pair_length', 'spacing', 'bend',
        'fit_residual', 'load']


def test_finite_turns_nan_into_none(viewer):
    """The page's json must never hold a bare NaN (the browser would refuse the whole page)."""
    assert viewer.finite({'a': float('nan'), 'b': [1.0, np.float64('inf')], 'c': np.array([2.0])}) == \
        {'a': None, 'b': [1.0, None], 'c': [2.0]}
