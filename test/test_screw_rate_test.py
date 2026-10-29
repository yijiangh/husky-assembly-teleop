"""The release test's analysis (screw_rate_test) on synthetic runs with a known answer."""

import json
import os

import numpy as np
import pytest

from husky_assembly_teleop.screw_rate_test import (
    SPEED_MARGIN, analyse_run, analyse_tool, main, plot_run, save_run, summarise, summary_line,
    tool_axis_from_quat, tool_lines, travel_at_tool_mm,
)

# * The synthetic run: 20 Hz samples, LOOSEN at 0.5 s, Stop at 8.5 s (epoch-like times).
T0 = 1000.0
DT = 0.05
T_LOOSEN = T0 + 0.5
T_STOP = T0 + 8.5
DELAY_S = 0.3        # LOOSEN -> the screw starts pushing
PUSH_S = 3.5         # how long it pushes, until the thread is out
TRAVEL_MM = 7.5      # how far the bar moves at each tool, along the tool axis
# Tool0 z pointing along world +y: a -90 deg turn about x, as a PyBullet (x, y, z, w) quaternion.
QUAT_Y = (-np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5))
TOL_S = DT + 1e-6


def _tool_signals(t: np.ndarray, push_n: float = 10.0, release: bool = True) -> tuple:
    """One tool's wrench, current and state over the run.

    Args:
        t (np.ndarray): Sample times.
        push_n (float): How much the axial force changes while the screw pushes.
        release (bool): The tool lets go of the bar at the end of the push (False: it never does).

    Returns:
        tuple: ``(wrench, current, state)`` lists, one entry per sample.
    """
    t_push, t_free = T_LOOSEN + DELAY_S, T_LOOSEN + DELAY_S + PUSH_S
    wrench, current, state = [], [], []
    for ti in t:
        if ti < t_push or (ti >= t_free and not release):
            w = [47.0, 0.0, 4.0, 0.0, 0.0, 0.0]          # the mount preload, tool still on the bar
        elif ti < t_free:
            w = [47.0, 0.0, 4.0 - push_n, 0.0, 0.0, 0.0]  # + the screw pushing the bar off
        else:
            w = [2.0, 0.0, 1.0, 0.0, 0.0, 0.0]           # empty tool
        wrench.append(w)
        if ti < T_LOOSEN or ti > T_STOP:
            current.append(0.0)
        else:
            current.append(200.0 if ti < t_free else 60.0)
        state.append('LOOSENING' if T_LOOSEN + 0.1 <= ti <= T_STOP else 'IDLE')
    return wrench, current, state


def _run(push_n: float = 10.0, with_fits: bool = True, release: bool = True) -> dict:
    """A whole synthetic release test, as ``HuskyMonitor._release_test_task`` saves it.

    Args:
        push_n (float): The axial force change while pushing (0 = no push).
        with_fits (bool): Give the before / after MoCap fits.
        release (bool): The tools let go of the bar at the end of the push.

    Returns:
        dict: The run.
    """
    t = T0 + DT * np.arange(int(round(9.5 / DT)) + 1)
    samples = {'t': t.tolist()}
    for side in ('left', 'right'):
        wrench, current, state = _tool_signals(t, push_n, release)
        samples[side] = {'wrench': wrench, 'current': current, 'state': state, 'pwm': [None] * len(t),
                         'joints': [[0.1, -1.0, 1.2, 0.0, 1.5, 0.0]] * len(t)}
    T_before = np.eye(4)
    T_after = np.eye(4)
    T_after[1, 3] = TRAVEL_MM / 1000.0              # the bar pushed 7.5 mm along +y
    return {
        'stamp': '2026-10-28T12:00:00', 'bar': 'bar_B20', 't_loosen': T_LOOSEN, 't_stop': T_STOP,
        'samples': samples,
        'before': {'T_world_from_bar': T_before.tolist() if with_fits else None,
                   'tool0_fk': {'left': [[0.5, -0.97, 0.16], list(QUAT_Y)],
                                'right': [[1.7, -0.97, 0.16], list(QUAT_Y)]}},
        'after': {'T_world_from_bar': T_after.tolist() if with_fits else None},
    }


def test_tool_axis_from_quat():
    """Tool0 z of a -90 deg turn about x is world +y; the identity gives world +z."""
    assert np.allclose(tool_axis_from_quat(QUAT_Y), [0.0, 1.0, 0.0])
    assert np.allclose(tool_axis_from_quat((0.0, 0.0, 0.0, 1.0)), [0.0, 0.0, 1.0])


def test_travel_at_tool_follows_a_turn_of_the_bar():
    """A bar turned about world z moves more at a tool far from the turn axis."""
    T_before = np.eye(4)
    ang = np.radians(0.5)
    T_after = np.eye(4)
    T_after[:3, :3] = [[np.cos(ang), -np.sin(ang), 0.0], [np.sin(ang), np.cos(ang), 0.0], [0.0, 0.0, 1.0]]
    near = travel_at_tool_mm(T_before, T_after, [[0.1, 0.0, 0.0], list(QUAT_Y)])
    far = travel_at_tool_mm(T_before, T_after, [[1.2, 0.0, 0.0], list(QUAT_Y)])
    assert near == pytest.approx(100.0 * np.sin(ang), abs=1e-6)
    assert far == pytest.approx(1200.0 * np.sin(ang), abs=1e-6)
    assert travel_at_tool_mm(None, T_after, [[0.0, 0.0, 0.0], list(QUAT_Y)]) is None


def test_analyse_tool_finds_the_push():
    """Delay, push time by force and by current, and the motor start, within one sample."""
    run = _run()
    s = run['samples']
    r = analyse_tool(s['t'], s['left']['wrench'], s['left']['current'], s['left']['state'],
                     T_LOOSEN, T_STOP)
    assert r['delay_s'] == pytest.approx(DELAY_S, abs=TOL_S)
    assert r['push_force_s'] == pytest.approx(PUSH_S, abs=TOL_S)
    assert r['push_current_s'] == pytest.approx(PUSH_S, abs=TOL_S)
    assert r['push_s'] == pytest.approx(PUSH_S, abs=TOL_S)
    assert r['motor_start_s'] == pytest.approx(0.1, abs=TOL_S)
    assert r['note'] == ''


def test_analyse_tool_without_a_push_says_so():
    """No axial force change after the LOOSEN at all: no delay, no push time, and a note."""
    s = _run(push_n=0.0, release=False)['samples']
    r = analyse_tool(s['t'], s['left']['wrench'], s['left']['current'], s['left']['state'],
                     T_LOOSEN, T_STOP)
    assert r['delay_s'] is None and r['push_s'] is None
    assert 'no push seen' in r['note']


def test_analyse_tool_release_without_a_push_is_no_rate():
    """The force drops straight from the hold to the empty tool: not a push, so no rate."""
    run = _run(push_n=0.0)
    s = run['samples']
    r = analyse_tool(s['t'], s['left']['wrench'], s['left']['current'], s['left']['state'],
                     T_LOOSEN, T_STOP)
    assert r['delay_s'] is None and r['push_s'] is None
    assert 'no clear push' in r['note']
    assert analyse_run(run)['tools']['left']['rate_mm_s'] is None


def test_analyse_run_rate_and_summary():
    """Travel from the MoCap fits, rate = travel / push time, and the suggested speed."""
    result = analyse_run(_run())
    for side in ('left', 'right'):
        r = result['tools'][side]
        assert r['travel_source'] == 'MoCap'
        assert r['travel_mm'] == pytest.approx(TRAVEL_MM, abs=1e-6)
        assert r['rate_mm_s'] == pytest.approx(TRAVEL_MM / PUSH_S, rel=0.02)
    assert result['arms_moved_deg'] == pytest.approx(0.0)
    assert result['sample_hz'] == pytest.approx(1.0 / DT, rel=1e-3)
    summary = summarise([result, result], pitch_mm=2.0)
    assert summary['n_runs'] == 2 and summary['n_rates'] == 4
    assert summary['speed_mm_s'] == pytest.approx(SPEED_MARGIN * summary['slowest_mm_s'])
    assert summary['rpm'] == pytest.approx(summary['speed_mm_s'] / 2.0 * 60.0)
    assert 'RETREAT_SCREW_RPM' in summary_line(summary)
    assert len(tool_lines(result)) == 2


def test_analyse_run_without_mocap_uses_the_design_travel():
    """Without MoCap fits the travel is the design's, and the line says so."""
    result = analyse_run(_run(with_fits=False), design_travel_mm=8.0)
    r = result['tools']['left']
    assert r['travel_mm'] == 8.0 and 'design' in r['travel_source']
    assert r['rate_mm_s'] == pytest.approx(8.0 / PUSH_S, rel=0.02)


def test_save_plot_and_reprint(tmp_path, capsys):
    """A saved run re-prints the same summary from the command line; the plot is written."""
    run = _run()
    result = analyse_run(run)
    path = os.path.join(tmp_path, 'release_test_120000.json')
    save_run(path, run, result)
    plot_run(os.path.join(tmp_path, 'release_test_120000.png'), run, result)
    assert os.path.getsize(os.path.join(tmp_path, 'release_test_120000.png')) > 0
    with open(path) as f:
        assert json.load(f)['kind'] == 'release_test'
    main([path])
    out = capsys.readouterr().out
    assert summary_line(summarise([result], 2.0)) in out
