"""The one-mate insert's per-arm ending (husky_world): the pure pieces.

* ``segment_pose_at``: the straight line a compliant stream follows, so the holding
  arm can restart its line from the point it had reached at the stall.
* ``_SettleWatch``: "the tool stopped moving" (less than 0.5 mm over the last 0.5 s),
  with the 3 s ceiling; a fake clock drives it.
* ``switch_arm_controllers``: asks controller_manager only for the arms that are
  not on the wanted controller yet (a mixed state after the mating arm went back).

husky_world imports the ROS stack, so these tests skip when it is not sourced.
"""

from types import SimpleNamespace

import numpy as np
import pytest

husky_world = pytest.importorskip('husky_assembly_teleop.husky_world')
from husky_assembly_teleop.robot_registry import robot_by_name  # noqa: E402

JOINT_CTRL = 'scaled_joint_trajectory_controller'
COMPLIANCE_CTRL = 'cartesian_compliance_controller'


# * --- segment_pose_at ---

def _segment():
    """A 10 mm line along x over 2 s, turning 90 deg about z on the way."""
    start = ((1.0, 0.0, 0.5), (0.0, 0.0, 0.0, 1.0))
    end = ((1.01, 0.0, 0.5), (0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)))
    return [start, end, 2.0, 30.0]


def test_segment_pose_at_runs_from_start_point_to_end_point():
    """t = 0 is the start point, t = t_move the end point, halfway the midpoint; after t_move it holds the end."""
    seg = _segment()
    p, q = husky_world.segment_pose_at(seg, 0.0)
    assert np.allclose(p, seg[0][0]) and np.allclose(q, seg[0][1])
    p, q = husky_world.segment_pose_at(seg, 2.0)
    assert np.allclose(p, seg[1][0]) and np.allclose(np.abs(np.dot(q, seg[1][1])), 1.0)
    p, _ = husky_world.segment_pose_at(seg, 1.0)
    assert np.allclose(p, (1.005, 0.0, 0.5))
    p_late, q_late = husky_world.segment_pose_at(seg, 7.0)
    assert np.allclose(p_late, seg[1][0]) and np.allclose(np.abs(np.dot(q_late, seg[1][1])), 1.0)


def test_segment_pose_at_turns_halfway_at_half_time():
    """The orientation slerps: halfway along, half the turn (45 deg about z)."""
    _, q = husky_world.segment_pose_at(_segment(), 1.0)
    angle = 2.0 * np.degrees(np.arccos(min(1.0, abs(float(q[3])))))
    assert angle == pytest.approx(45.0, abs=1e-6)


# * --- _SettleWatch ---

class _Clock:
    """A clock the test advances by hand."""

    def __init__(self):
        self.t = 100.0

    def __call__(self) -> float:
        return self.t


def _watch(clock):
    return husky_world._SettleWatch(tol_m=0.0005, window_s=0.5, ceiling_s=3.0, clock=clock)


def test_settle_watch_says_nothing_before_a_full_window():
    """With less than a window of samples the answer is None and moved_mm unknown, even for a still tool."""
    clock = _Clock()
    watch = _watch(clock)
    for _ in range(4):
        assert watch.sample([1.0, 2.0, 3.0]) is None
        clock.t += 0.1
    assert watch.moved_mm is None


def test_settle_watch_settles_a_still_tool_right_after_one_window():
    """A tool that does not move is 'settled' at the first sample a full window after the first."""
    clock = _Clock()
    watch = _watch(clock)
    verdicts = []
    while len(verdicts) < 8 and (not verdicts or verdicts[-1] is None):
        verdicts.append(watch.sample([1.0, 2.0, 3.0]))
        clock.t += 0.125       # exact in binary: 4 ticks = one 0.5 s window
    assert verdicts == [None] * 4 + ['settled']
    assert watch.moved_mm == pytest.approx(0.0)
    assert watch.elapsed_s == pytest.approx(0.5)


def test_settle_watch_hits_the_ceiling_on_a_creeping_tool():
    """A tool creeping 1 mm per 0.5 s never settles; at 3 s the verdict is 'ceiling' with that movement."""
    clock = _Clock()
    watch = _watch(clock)
    t0 = clock.t
    verdict = None
    while verdict is None:
        verdict = watch.sample([0.002 * (clock.t - t0), 0.0, 0.0])    # 2 mm/s = 1 mm per window
        clock.t += 0.125
    assert verdict == 'ceiling'
    assert watch.elapsed_s == pytest.approx(3.0)
    assert watch.moved_mm == pytest.approx(1.0, abs=1e-6)


def test_settle_watch_window_fills_on_jittered_ticks():
    """Uneven tick times (0.07 / 0.13 s) still give a compared pair a full window apart, so it settles."""
    clock = _Clock()
    watch = _watch(clock)
    verdict = None
    for k in range(40):
        verdict = watch.sample([0.0, 0.0, 0.0])
        if verdict is not None:
            break
        clock.t += 0.07 if k % 2 else 0.13
    assert verdict == 'settled'
    assert 0.5 <= watch.elapsed_s < 0.7


# * --- switch_arm_controllers: only the arms not there yet ---

def _monitor(active):
    """A stand-in Cindy monitor whose interface runs ``active`` (per arm) and records switch requests."""
    hi = SimpleNamespace(active_controller=list(active), controller_switch_error=[None, None],
                         controller_query_pending=[False, False], requests=[])

    def switch_controller(from_ctrl, to_ctrl, index):
        hi.requests.append((from_ctrl, to_ctrl, index))
        hi.active_controller[index] = to_ctrl      # acknowledged at once
        return True

    hi.switch_controller = switch_controller
    log = SimpleNamespace(error=lambda text: None, info=lambda text: None, warn=lambda text: None)
    return SimpleNamespace(huskies=[SimpleNamespace(interface=hi)], selected_robot_id=0,
                           _connected_robot=lambda: robot_by_name('Cindy'), get_logger=lambda: log), hi


def _run(gen):
    """Drive a generator to its end and return its return value."""
    try:
        while True:
            next(gen)
    except StopIteration as done:
        return done.value


def test_switch_asks_only_for_the_arm_still_under_the_other_controller():
    """Left already on the joint controller, right under compliance: one request, for the right arm."""
    mon, hi = _monitor([JOINT_CTRL, COMPLIANCE_CTRL])
    ok = _run(husky_world.switch_arm_controllers(mon, JOINT_CTRL))
    assert ok is True
    assert hi.requests == [(COMPLIANCE_CTRL, JOINT_CTRL, 1)]
    assert hi.active_controller == [JOINT_CTRL, JOINT_CTRL]


def test_switch_with_both_arms_already_there_sends_nothing():
    """Nothing to do: True without a request (STRICT would reject deactivate == activate)."""
    mon, hi = _monitor([JOINT_CTRL, JOINT_CTRL])
    assert _run(husky_world.switch_arm_controllers(mon, JOINT_CTRL)) is True
    assert hi.requests == []


def test_switch_of_one_listed_arm_leaves_the_other_alone():
    """arm_indices=[0] with both under compliance: only arm 0 is switched."""
    mon, hi = _monitor([COMPLIANCE_CTRL, COMPLIANCE_CTRL])
    assert _run(husky_world.switch_arm_controllers(mon, JOINT_CTRL, arm_indices=[0])) is True
    assert hi.requests == [(COMPLIANCE_CTRL, JOINT_CTRL, 0)]
    assert hi.active_controller == [JOINT_CTRL, COMPLIANCE_CTRL]
