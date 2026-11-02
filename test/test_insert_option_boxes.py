"""The insert's two option boxes follow the loaded step (HuskyMonitor._default_insert_options).

* Spiral search: ticked and live for an insert mating on BOTH arms; unticked and
  greyed for a ground bar, a one-mate bar, or a step that is no insert.
* Servo feedback (MoCap bar): a reminder, never a choice, so ALWAYS greyed:
  ticked for a GROUND bar that carries markers, unticked for every other bar,
  markers or not (a given rule: only a ground bar is servoed by its markers,
  every other bar aims tool0).
* A greyed box's tick never overwrites the flag; a click that still gets through
  to the servo box changes nothing and warns.

Each test stands in for the monitor with a SimpleNamespace and calls the real
methods on it. husky_monitor imports the ROS stack, so these tests skip when it
is not sourced.
"""

from types import SimpleNamespace

import pytest

husky_monitor = pytest.importorskip('husky_assembly_teleop.husky_monitor')
from compas.geometry import Frame  # noqa: E402
from compas_fab.robots import RigidBodyState, RobotCellState  # noqa: E402

from husky_assembly_teleop.bar_action_io import MovementKind  # noqa: E402

HuskyMonitor = husky_monitor.HuskyMonitor


class _FakeToggle:
    """A checkbox that records what the monitor writes into it."""

    def __init__(self, ticked: bool = False, enabled: bool = True):
        self.value = ticked
        self.enabled = enabled
        self.calls = []

    def set_value(self, value: bool) -> None:
        self.value = bool(value)
        self.calls.append(('value', bool(value)))

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = bool(enabled)
        self.calls.append(('enabled', bool(enabled)))


class _Log:
    """Collects what the monitor logs, by level."""

    def __init__(self):
        self.lines = []

    def info(self, text: str):
        self.lines.append(('info', text))

    def warn(self, text: str):
        self.lines.append(('warn', text))

    def error(self, text: str):
        self.lines.append(('error', text))

    def of(self, level: str) -> list:
        return [text for lvl, text in self.lines if lvl == level]


def _state(mating: str) -> RobotCellState:
    """A start state holding a bar: ``'two'`` mates on both arms, ``'one'`` on the left only, ``'ground'`` holds a ground joint."""
    bodies = {
        'joint_J1-3_male': RigidBodyState(Frame.worldXY(), attached_to_link='left_ur_arm_tool0'),
        'joint_J2-3_male': RigidBodyState(Frame.worldXY(), attached_to_link='right_ur_arm_tool0'),
    }
    if mating in ('one', 'two'):
        bodies['joint_J1-3_female'] = RigidBodyState(Frame.worldXY())
    if mating == 'two':
        bodies['joint_J2-3_female'] = RigidBodyState(Frame.worldXY())
    if mating == 'ground':
        bodies['joint_G1_ground'] = RigidBodyState(Frame.worldXY(), attached_to_link='left_ur_arm_tool0')
    return RobotCellState(rigid_body_states=bodies)


def _movement(kind, mating: str = 'two'):
    """A stand-in movement of ``kind`` with a start state."""
    return SimpleNamespace(movement_id=f'B3_J_{kind.value}', start_state=_state(mating), kind=kind)


def _monitor(*, markers: bool, mocap: bool = True, spiral_box=True, servo_box=True) -> SimpleNamespace:
    """A stand-in monitor with the real option-box methods bound.

    Args:
        markers (bool): Whether the active bar is in the MoCap marker table.
        mocap (bool): Whether MoCap is in use at all.
        spiral_box, servo_box: A ``_FakeToggle`` each (or None for a hidden box).
    """
    log = _Log()
    mon = SimpleNamespace(
        log=log, get_logger=lambda: log, USE_MOCAP=mocap, MOCAP_BAR_FAKE_MARKERS=False,
        active_bar_name='bar_B3', current_movement=None, m2_spiral_search=False, _servo_bar_feedback=False,
        _loaded_movements=[],
        m2_spiral_toggle=_FakeToggle() if spiral_box else None,
        servo_bar_feedback_toggle=_FakeToggle() if servo_box else None,
        _ANSI_GREEN='', _ANSI_YELLOW='', _ANSI_RESET='')
    mon._kind_of = lambda mv: getattr(mv, 'kind', None)
    mon._mocap_bar_design = lambda name: object() if (markers and name) else None
    for name in ('_spiral_fits', '_bar_has_mocap_markers', '_action_bar_is_grounded', '_servo_can_aim_bar',
                 '_loaded_index_of', '_loaded_movement_of', '_default_insert_options',
                 '_apply_insert_option_boxes', '_m2_spiral_search', 'servo_bar_feedback_ticked',
                 '_log_spiral_mode', '_log_servo_feedback_mode', '_set_servo_bar_feedback',
                 '_set_m2_spiral_search'):
        setattr(mon, name, getattr(HuskyMonitor, name).__get__(mon))
    return mon


def _load(mon, mv):
    """What load_selected_movement does for the boxes: load the action (here: the one step), seed the boxes."""
    mon._loaded_movements = [mv]
    mon.current_movement = mv
    mon._default_insert_options(mv)


# * --- the spiral box ---

def test_two_mate_insert_ticks_the_spiral_box_and_keeps_it_live():
    mon = _monitor(markers=True)
    _load(mon, _movement(MovementKind.DUAL_CONSTRAINED_LINEAR, 'two'))
    assert mon.m2_spiral_search is True
    assert mon.m2_spiral_toggle.value is True and mon.m2_spiral_toggle.enabled is True
    assert any('insert spiral search: ON' in t for t in mon.log.of('info'))


@pytest.mark.parametrize('mating, kind', [
    ('one', MovementKind.DUAL_CONSTRAINED_LINEAR),      # a one-mate bar
    ('ground', MovementKind.DUAL_CONSTRAINED_LINEAR),   # a ground bar
    ('two', MovementKind.DUAL_INDEPENDENT_LINEAR),      # the retreat: no insert
])
def test_other_steps_grey_the_spiral_box_unticked(mating, kind):
    mon = _monitor(markers=True)
    _load(mon, _movement(kind, mating))
    assert mon.m2_spiral_search is False
    assert mon.m2_spiral_toggle.value is False and mon.m2_spiral_toggle.enabled is False
    assert any('insert spiral search: n/a' in t for t in mon.log.of('info'))


def test_a_greyed_spiral_box_never_writes_the_flag():
    """A greyed box showing a stale tick leaves the flag alone; a live one is copied."""
    mon = _monitor(markers=True)
    mon.m2_spiral_toggle = _FakeToggle(ticked=True, enabled=False)
    mon.m2_spiral_search = False
    assert mon._m2_spiral_search() is False
    mon.m2_spiral_toggle.enabled = True
    assert mon._m2_spiral_search() is True


def test_nothing_loaded_says_the_spiral_is_not_applicable_not_on():
    mon = _monitor(markers=True)
    mon.m2_spiral_search = True      # a stale flag must not be announced as ON
    mon._log_spiral_mode()
    assert any('insert spiral search: n/a' in t and 'no step loaded' in t for t in mon.log.of('info'))
    assert not any('insert spiral search: ON' in t for t in mon.log.of('info'))


# * --- the servo box: a reminder of the rule, never a choice ---

def test_a_ground_bar_with_markers_ticks_the_servo_box_greyed():
    mon = _monitor(markers=True)
    _load(mon, _movement(MovementKind.DUAL_CONSTRAINED_LINEAR, 'ground'))
    assert mon._servo_bar_feedback is True
    assert mon.servo_bar_feedback_toggle.value is True and mon.servo_bar_feedback_toggle.enabled is False
    assert any('servo feedback: MoCap bar' in t for t in mon.log.of('info'))


def test_the_transfer_of_a_ground_bar_ticks_the_servo_box():
    """The transfer holds no ground joint itself: the action's insert says the bar is grounded."""
    mon = _monitor(markers=True)
    transfer = _movement(MovementKind.DUAL_CONSTRAINED_FREE, 'two')
    insert = _movement(MovementKind.DUAL_CONSTRAINED_LINEAR, 'ground')
    mon._loaded_movements = [transfer, insert]
    mon.current_movement = transfer
    mon._default_insert_options(transfer)
    assert mon._servo_bar_feedback is True and mon.servo_bar_feedback_toggle.value is True


@pytest.mark.parametrize('mating', ['two', 'one'])
def test_a_screwed_bar_with_markers_greys_the_servo_box_unticked(mating):
    """Only a ground bar is servoed by its markers: B21, B22 ... aim tool0 even with markers."""
    mon = _monitor(markers=True)
    _load(mon, _movement(MovementKind.DUAL_CONSTRAINED_LINEAR, mating))
    assert mon._servo_bar_feedback is False
    assert mon.servo_bar_feedback_toggle.value is False and mon.servo_bar_feedback_toggle.enabled is False
    assert any('servo feedback: tool0' in t and 'is not a ground bar' in t for t in mon.log.of('info'))


def test_a_bar_without_markers_greys_the_servo_box_unticked():
    mon = _monitor(markers=False)
    _load(mon, _movement(MovementKind.DUAL_CONSTRAINED_LINEAR, 'two'))
    assert mon._servo_bar_feedback is False
    assert mon.servo_bar_feedback_toggle.value is False and mon.servo_bar_feedback_toggle.enabled is False
    assert any('servo feedback: tool0' in t and 'carries no MoCap markers' in t for t in mon.log.of('info'))


def test_without_mocap_the_servo_flag_stays_off():
    mon = _monitor(markers=True, mocap=False, servo_box=False)
    _load(mon, _movement(MovementKind.DUAL_CONSTRAINED_LINEAR, 'two'))
    assert mon._servo_bar_feedback is False


@pytest.mark.parametrize('markers, mating, ticked, rule', [
    (True, 'ground', False, True),    # unticking a ground bar: stays MoCap bar
    (True, 'two', True, False),       # ticking a screwed bar with markers: stays tool0
    (False, 'two', True, False),      # ticking a bar without markers: stays tool0
])
def test_a_click_on_the_servo_box_changes_nothing(markers, mating, ticked, rule):
    """A click that gets through (PyBullet cannot grey a box) puts the flag and the box back to the rule."""
    mon = _monitor(markers=markers)
    _load(mon, _movement(MovementKind.DUAL_CONSTRAINED_LINEAR, mating))
    mon._set_servo_bar_feedback(ticked)
    assert mon._servo_bar_feedback is rule and mon.servo_bar_feedback_ticked() is rule
    assert mon.servo_bar_feedback_toggle.value is rule and mon.servo_bar_feedback_toggle.enabled is False
    warns = mon.log.of('warn')
    assert len(warns) == 1 and 'a reminder, not a choice' in warns[0]


def test_the_servo_flag_is_never_read_from_the_widget():
    mon = _monitor(markers=False)
    mon.servo_bar_feedback_toggle = _FakeToggle(ticked=True, enabled=True)
    mon._servo_bar_feedback = False
    assert mon.servo_bar_feedback_ticked() is False


def test_hidden_boxes_do_not_break_the_load():
    """No MoCap and no assembly knobs (both boxes None): the flags are still set."""
    mon = _monitor(markers=True, spiral_box=False, servo_box=False)
    _load(mon, _movement(MovementKind.DUAL_CONSTRAINED_LINEAR, 'two'))
    assert mon.m2_spiral_search is True
