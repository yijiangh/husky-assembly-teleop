"""Checks for the schedule panel's text and rules (no UI toolkit, no cell, no ROS).

The counts are pinned on the REAL multi-robot export (``ActionSchedule.json`` and
a few ``BarActions/*.json``, never the 340 MB ``RobotCell*.json``). Anything that
writes works on a copy in ``tmp_path``.
"""
import os
import shutil

import pytest
from compas.data import json_dump
from compas_fab.robots import JointTrajectory, JointTrajectoryPoint
from rs_data_structure.bar_action import IndependentDualArmLinearMovement, ScaffoldingToolMovement

from husky_assembly_teleop import DESIGN_DATA_DIRECTORY
from husky_assembly_teleop.bar_action_io import (
    operator_steps, parse_bar_action, sidecar_action_path, step_kind,
)
from husky_assembly_teleop.robot_registry import robot_by_name
from husky_assembly_teleop.schedule_io import SCHEDULE_FILENAME, load_entry, load_schedule
from husky_assembly_teleop.schedule_ui import (
    ROW_COLOR_DONE, ROW_COLOR_OTHER_ROBOT, ROW_COLOR_PENDING, ROW_COLOR_SELECTED,
    EntryFlags, absorbed_tool_note, entry_row_text, knobs_for_assembly_robot,
    missing_action_files, now_line_text, row_color, scan_entry_flags, step_button_label,
    step_readout_text, visible_row_window,
)

SCHEDULE_PROBLEM = '260920_RobArch_demo_revamp_backup'
FIXTURE_ROOT = os.path.join(DESIGN_DATA_DIRECTORY, SCHEDULE_PROBLEM)

needs_fixture = pytest.mark.skipif(
    not os.path.isfile(os.path.join(FIXTURE_ROOT, SCHEDULE_FILENAME)),
    reason='the multi-robot design-study export is not on this machine',
)


@pytest.fixture(scope='module')
def schedule():
    """The fixture problem's schedule, loaded once."""
    return load_schedule(FIXTURE_ROOT)


def _action_file(name: str) -> str:
    """Path of one of the fixture's clean action exports.

    Args:
        name (str): e.g. ``'B1__J.json'``.

    Returns:
        str: The absolute path.
    """
    return os.path.join(FIXTURE_ROOT, 'BarActions', name)


# * ------------------------------------------------------------- file flags
@needs_fixture
@pytest.mark.parametrize('name, expected', [
    # B1__J: M0 free, M3 transfer, M5 insert drive the arms; only M3 / M5 have a goal conf.
    ('B1__J.json', EntryFlags(n_arm=3, n_ik=2, n_traj=0, sidecar=False)),
    ('B1__R.json', EntryFlags(n_arm=2, n_ik=2, n_traj=0, sidecar=False)),
    ('B3__H.json', EntryFlags(n_arm=2, n_ik=2, n_traj=0, sidecar=False)),
    ('B3__HR.json', EntryFlags(n_arm=1, n_ik=1, n_traj=0, sidecar=False)),
])
def test_scan_entry_flags_on_clean_exports(name, expected):
    """The clean exports scan to the pinned arm / IK / trajectory counts, no sidecar."""
    assert scan_entry_flags(_action_file(name)) == expected


@needs_fixture
def test_scan_entry_flags_on_a_sidecar_with_one_trajectory(tmp_path):
    """A sidecar with one planned trajectory counts it and is flagged as a sidecar."""
    clean = tmp_path / 'B3__H.json'
    shutil.copy(_action_file('B3__H.json'), clean)
    action = parse_bar_action(str(clean))
    names = list(robot_by_name('Alice').arm_joint_names[0])
    action.movements[0].trajectory = JointTrajectory(
        trajectory_points=[JointTrajectoryPoint([0.0] * 6, [0] * 6),
                           JointTrajectoryPoint([0.1] * 6, [0] * 6)],
        joint_names=names)
    sidecar = sidecar_action_path(str(clean))
    json_dump(action, sidecar)

    assert scan_entry_flags(sidecar) == EntryFlags(n_arm=2, n_ik=2, n_traj=1, sidecar=True)
    assert scan_entry_flags(str(clean)).sidecar is False


@needs_fixture
def test_missing_action_files(schedule, tmp_path):
    """The full fixture misses no file; a partial copy lists every missing entry."""
    assert missing_action_files(schedule) == []

    # A scratch problem with only two of the 48 files: the other 46 are missing.
    root = tmp_path / SCHEDULE_PROBLEM
    (root / 'BarActions').mkdir(parents=True)
    shutil.copy(os.path.join(FIXTURE_ROOT, SCHEDULE_FILENAME), root / SCHEDULE_FILENAME)
    for name in ('B1__J.json', 'B1__R.json'):
        shutil.copy(_action_file(name), root / 'BarActions' / name)
    partial = load_schedule(str(root))
    missing = missing_action_files(partial)
    assert len(missing) == 46
    assert [e.index for e in missing[:3]] == [2, 3, 4]


# * ------------------------------------------------------------- step buttons
@needs_fixture
@pytest.mark.parametrize('index, kinds', [
    (0, ['arm', 'manual', 'scaffold', 'arm', 'scaffold', 'arm']),   # B1_J
    (1, None),                                                      # B1_R: checked by structure
    (3, ['arm', 'gripper', 'arm', 'gripper']),                      # B3_H (Alice)
    (16, ['gripper', 'arm']),                                       # B3_HR (Alice)
])
def test_step_kind_sequences(schedule, index, kinds):
    """Each entry's movements map to the expected step kinds, in order."""
    loaded = load_entry(schedule, schedule.entry(index))
    steps = [step_kind(mv) for mv in loaded.movements]
    if kinds is None:
        # * A release ends with the retreat and the free move home; the screw
        # * steps before them may be added or dropped by a re-export.
        assert steps[-2:] == ['arm', 'arm']
        assert all(step == 'scaffold' for step in steps[:-2])
    else:
        assert steps == kinds


@needs_fixture
def test_step_button_labels(schedule):
    """The step button label of every movement of B1_J, B1_R, B3_H and B3_HR."""
    labels = {i: [step_button_label(mv) for mv in load_entry(schedule, schedule.entry(i)).movements]
              for i in (0, 1, 3, 16)}
    assert labels[0] == [
        None,
        'Operator done (manual step) -> then Confirm Exec',
        'Tool step: grasp',
        None,
        'Mark tool step done (tighten runs with the next movement)',
        None,
    ]
    assert labels[1] == [
        # R_M0 untighten is mark-only: the schedule never reverses the joint motor.
        'Mark tool step done (untighten: use Loosen Joint by hand if needed)',
        'Mark tool step done (ungrasp runs with the next movement)',
        None,
        None,
    ]
    assert labels[3] == [
        None,
        'Exec gripper step: OPEN',
        None,
        'Exec gripper step: CLOSE + compliant handoff',
    ]
    assert labels[16] == ['Exec gripper step: OPEN', None]
    assert step_button_label(None) is None


def test_step_button_label_of_an_unknown_movement_class_is_none():
    """A movement class the monitor does not know gets no step button."""
    class Odd:
        movement_id = 'X_M0_odd'

    assert step_button_label(Odd()) is None


# * ------------------------------------------------------------- operator steps
MARK_ONLY_PREFIX = 'Mark tool step done'


@needs_fixture
@pytest.mark.parametrize('index', [0, 1, 3, 16])
def test_step_primaries_never_get_a_mark_only_button(schedule, index):
    """No step the operator lands on is mark-only; the absorbed steps are exactly the mark-only ones."""
    movements = load_entry(schedule, schedule.entry(index)).movements
    steps = operator_steps(movements)
    for step in steps:
        label = step_button_label(movements[step.primary])
        assert label is None or not label.startswith(MARK_ONLY_PREFIX), movements[step.primary].movement_id
    absorbed = {j for step in steps for j in step.absorbed}
    mark_only = {i for i, mv in enumerate(movements)
                 if (step_button_label(mv) or '').startswith(MARK_ONLY_PREFIX)}
    assert absorbed == mark_only


@needs_fixture
def test_step_readout_text(schedule):
    """The step readout: id and kind, or id and the tool steps that run with it."""
    movements = load_entry(schedule, schedule.entry(0)).movements
    steps = operator_steps(movements)
    texts = [step_readout_text(k, len(steps), movements[step.primary],
                               tuple(movements[j] for j in step.absorbed))
             for k, step in enumerate(steps)]
    assert texts[3] == 'step 4/5: B1_J_M3_CDFM_transfer_to_approach  (dual_constrained_free)'
    assert texts[4] == 'step 5/5: B1_J_M5_LM_insert  (+ B1_J_M4_tool_tighten_joint runs with it)'

    # Two absorbed steps (the release before the D7 re-export), and an unknown class.
    untighten = ScaffoldingToolMovement(movement_id='untighten', tool_action='untighten')
    ungrasp = ScaffoldingToolMovement(movement_id='ungrasp', tool_action='ungrasp')
    retreat = IndependentDualArmLinearMovement(movement_id='retreat')
    assert (step_readout_text(0, 2, retreat, (untighten, ungrasp))
            == "step 1/2: retreat  (+ ungrasp runs with it; untighten not sent: "
               "'Loosen Joint' by hand if needed)")

    class Odd:
        movement_id = 'odd'

    assert step_readout_text(0, 1, Odd()) == 'step 1/1: odd  (Odd)'


def test_absorbed_tool_note():
    """The exec log line names the absorbed step and who sends its tool action."""
    tighten = ScaffoldingToolMovement(movement_id='B1_J_M4_tool_tighten_joint',
                                      tool_action='tighten', overlaps_next=True)
    assert absorbed_tool_note(tighten) == (
        "B1_J_M4_tool_tighten_joint ('tighten') runs with this movement: "
        "the compliant insert sends it (joint motors TIGHTENING).")
    ungrasp = ScaffoldingToolMovement(movement_id='B1_R_M1_tool_ungrasp_bar', tool_action='ungrasp')
    assert absorbed_tool_note(ungrasp) == (
        "B1_R_M1_tool_ungrasp_bar ('ungrasp') runs with this movement: "
        "the compliant retreat sends it (gripper motors LOOSENING).")
    untighten = ScaffoldingToolMovement(movement_id='B1_R_M0_tool_untighten_joint',
                                        tool_action='untighten')
    assert absorbed_tool_note(untighten) == (
        "B1_R_M0_tool_untighten_joint ('untighten') runs with this movement: "
        "nothing is sent (use the manual 'Loosen Joint' button if the tool must back off).")
    # The tool action decides, not overlaps_next: an ungrasp exported with it is still an ungrasp.
    ungrasp_overlapping = ScaffoldingToolMovement(movement_id='ungrasp', tool_action='ungrasp',
                                                  overlaps_next=True)
    assert 'gripper motors LOOSENING' in absorbed_tool_note(ungrasp_overlapping)


# * ------------------------------------------------------------- rows / now line
@needs_fixture
def test_entry_row_text(schedule):
    """Row text: marker, fixed-width columns, status, sidecar and other-robot suffixes."""
    flags = EntryFlags(n_arm=2, n_ik=2, n_traj=0, sidecar=False)
    entry3 = schedule.entry(3)
    assert (entry_row_text(entry3, flags, 'done', executable=True, selected=True)
            == '> [03] H   B3   Alice   IK 2/2  TRJ 0/2  done')
    assert (entry_row_text(entry3, flags, 'pending', executable=False, selected=False)
            == '  [03] H   B3   Alice   IK 2/2  TRJ 0/2  pending (other robot)')
    solved = EntryFlags(n_arm=3, n_ik=3, n_traj=3, sidecar=True)
    assert (entry_row_text(schedule.entry(0), solved, 'pending', executable=True, selected=False)
            == '  [00] J   B1   Cindy   IK 3/3  TRJ 3/3  pending [sidecar]')
    # Fixed widths: every row of the fixture lines up.
    rows = [entry_row_text(e, flags, 'pending', executable=True, selected=False)
            for e in schedule.entries]
    assert len({row.index('IK ') for row in rows}) == 1


def test_row_color():
    """Row colour priority: selected, then done, then other robot, else pending."""
    assert row_color('pending', executable=True, selected=True) == ROW_COLOR_SELECTED
    assert row_color('done', executable=False, selected=True) == ROW_COLOR_SELECTED
    assert row_color('done', executable=False, selected=False) == ROW_COLOR_DONE
    assert row_color('pending', executable=False, selected=False) == ROW_COLOR_OTHER_ROBOT
    assert row_color('pending', executable=True, selected=False) == ROW_COLOR_PENDING


@needs_fixture
def test_now_line_text(schedule):
    """The 'Now:' line counts steps; without a movement; with no entry loaded."""
    loaded = load_entry(schedule, schedule.entry(3))
    assert (now_line_text(schedule.entry(3), 2, 4, loaded.movements[2])
            == 'Now: entry 3 -- H B3 by Alice -- step 3/4 B3_H_M2_LM_to_grasp '
               '[single_linear] ctrl=joint_tracking')
    # Cindy's jointing half: the insert (movement 6 of 6) is step 5 of 5.
    jointing = load_entry(schedule, schedule.entry(0)).movements
    assert (now_line_text(schedule.entry(0), 4, len(operator_steps(jointing)), jointing[5])
            == 'Now: entry 0 -- J B1 by Cindy -- step 5/5 B1_J_M5_LM_insert '
               '[dual_constrained_linear] ctrl=cartesian_compliant')
    assert (now_line_text(schedule.entry(3), None, 4, None)
            == 'Now: entry 3 -- H B3 by Alice -- no movement loaded')
    assert now_line_text(None, None, 0, None).startswith('Now: no entry loaded')


@pytest.mark.parametrize('n, selected, size, expected', [
    (48, 0, 12, (0, 12)),
    (48, 47, 12, (36, 48)),
    (48, 20, 12, (14, 26)),
    (5, 3, 12, (0, 5)),
    (12, 11, 12, (0, 12)),
])
def test_visible_row_window(n, selected, size, expected):
    """The shown row window stays inside the schedule and contains the selected entry."""
    assert visible_row_window(n, selected, size) == expected


# * ------------------------------------------------------------- Cindy's knobs
@needs_fixture
def test_knobs_for_assembly_robot(schedule):
    """Cindy's M1 / M2 knobs show only on Cindy while none of another robot's entries is loaded."""
    cindy, alice = robot_by_name('Cindy'), robot_by_name('Alice')
    assert knobs_for_assembly_robot(None, cindy) is True
    assert knobs_for_assembly_robot(None, alice) is False
    assert knobs_for_assembly_robot(schedule.entry(3), cindy) is False
    assert knobs_for_assembly_robot(schedule.entry(0), cindy) is True
    assert knobs_for_assembly_robot(schedule.entry(3), alice) is False
