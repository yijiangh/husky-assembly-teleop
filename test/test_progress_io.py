"""Checks for progress.json and the robot beliefs (no cell, no planner, no ROS).

The belief checks read the REAL multi-robot export (``ActionSchedule.json`` and
a few ``BarActions/*.json``, never ``RobotCell*.json``). Every write goes to a
copy in ``tmp_path``.
"""
import os
import shutil
from datetime import datetime

import pytest

from compas_robots import Configuration

from husky_assembly_teleop import DESIGN_DATA_DIRECTORY
from husky_assembly_teleop.progress_io import (
    ARCHIVE_DIRNAME, BELIEF_ACTION_END, BELIEF_EXPORTED, BELIEF_LIVE, BELIEF_PARKED, HOLD_HOLDING, HOLD_PENDING,
    HOLD_RELEASED, PARKED_BASE_FRAME, PROGRESS_FILENAME, STATUS_DONE, STATUS_PENDING,
    STATUS_SKIPPED, archive_progress_and_saved_plans, arm_configuration, belief_after, belief_from_exported, belief_from_live,
    load_progress, new_progress, new_run_id, obstacle_sources, obstacle_sources_line,
    obstacle_tool_states, parked_belief, progress_path, recompute_belief, save_progress,
    schedule_fingerprint,
)
from husky_assembly_teleop.robot_registry import ROBOTS
from husky_assembly_teleop.schedule_io import SCHEDULE_FILENAME, load_schedule

SCHEDULE_PROBLEM = '260920_RobArch_demo_revamp_backup'
FIXTURE_ROOT = os.path.join(DESIGN_DATA_DIRECTORY, SCHEDULE_PROBLEM)
ALICE, BELLE, CINDY = ROBOTS['Alice'], ROBOTS['Belle'], ROBOTS['Cindy']

# * Alice's end state after holding B3 (entry 3), as exported in B4__J.
ALICE_HOLD_B3_BASE = (-4.5075, 1.7620, -0.0156)
ALICE_HOLD_B3_CONF = [1.035, -1.612, 2.393, -0.793, 0.331, 0.012]

pytestmark = pytest.mark.skipif(
    not os.path.isfile(os.path.join(FIXTURE_ROOT, SCHEDULE_FILENAME)),
    reason='the multi-robot design-study export is not on this machine',
)


@pytest.fixture(scope='module')
def schedule():
    """The fixture problem's schedule, loaded once."""
    return load_schedule(FIXTURE_ROOT)


@pytest.fixture
def tmp_problem(tmp_path) -> str:
    """A scratch copy of the problem folder holding only ActionSchedule.json.

    Named like the real problem, so progress files match it.
    """
    root = tmp_path / SCHEDULE_PROBLEM
    root.mkdir()
    shutil.copy(os.path.join(FIXTURE_ROOT, SCHEDULE_FILENAME), root / SCHEDULE_FILENAME)
    return str(root)


def _mark_done_up_to(progress, schedule, last_index: int):
    """Mark entries 0..last_index done, each with its authored end state as belief.

    Args:
        progress (Progress): Progress to update.
        schedule (ActionSchedule): The schedule.
        last_index (int): Last entry to mark.
    """
    for i in range(last_index + 1):
        entry = schedule.entry(i)
        belief = belief_after(schedule.load_action(entry), ROBOTS[entry.robot])
        progress.mark_done(entry, 'Cindy', 'Cindy-test', belief=belief)


# * ------------------------------------------------------------- the file

def test_new_progress(schedule):
    """A fresh progress: every entry pending, no beliefs, every hold pending."""
    progress = new_progress(schedule)
    assert progress.problem == SCHEDULE_PROBLEM and progress.schema_version == 1
    assert progress.schedule_fingerprint == schedule_fingerprint(schedule)
    assert progress.schedule_fingerprint.startswith('48:')
    assert progress.current_index == 0 and progress.next_pending_index() == 0
    assert len(progress.entries) == 48
    assert all(progress.status(i) == STATUS_PENDING for i in range(48))
    assert progress.robots == {}
    assert {bar: h.state for bar, h in progress.holds.items()} == {
        'B3': HOLD_PENDING, 'B7': HOLD_PENDING, 'B12': HOLD_PENDING, 'B15': HOLD_PENDING}


def test_missing_file_gives_a_new_progress(schedule, tmp_problem):
    """No progress.json on disk means a fresh progress, not an error."""
    assert not os.path.exists(progress_path(tmp_problem))
    assert load_progress(tmp_problem, schedule).current_index == 0


def test_save_and_load_round_trip(schedule, tmp_problem):
    """Statuses, holds and beliefs survive a save and load; no temporary file is left."""
    progress = new_progress(schedule)
    _mark_done_up_to(progress, schedule, 3)
    path = save_progress(progress, tmp_problem)

    assert path == os.path.join(tmp_problem, PROGRESS_FILENAME)
    # Written through a temporary file that is renamed into place: none is left.
    assert set(os.listdir(tmp_problem)) == {SCHEDULE_FILENAME, PROGRESS_FILENAME}

    loaded = load_progress(tmp_problem, load_schedule(tmp_problem))
    assert loaded.current_index == 4
    assert [loaded.status(i) for i in range(5)] == [STATUS_DONE] * 4 + [STATUS_PENDING]
    assert loaded.entries[3].marked_by_robot == 'Cindy' and loaded.entries[3].run_id == 'Cindy-test'
    assert loaded.holds['B3'].state == HOLD_HOLDING and loaded.holds['B3'].since_index == 3

    belief = loaded.belief('Alice')
    assert belief.source == BELIEF_ACTION_END and belief.after_entry == 3
    assert list(belief.base_frame.point) == pytest.approx(
        list(progress.belief('Alice').base_frame.point), abs=1e-9)
    assert belief.configuration.joint_names == ALICE.all_arm_joint_names
    assert belief.configuration.joint_values == pytest.approx(
        progress.belief('Alice').configuration.joint_values, abs=1e-9)
    assert len(loaded.belief('Cindy').configuration.joint_values) == 12


def test_file_of_another_problem_is_refused(schedule, tmp_problem):
    """A progress.json written for another design problem is a ValueError."""
    progress = new_progress(schedule)
    progress.problem = 'some_other_problem'
    save_progress(progress, tmp_problem)
    with pytest.raises(ValueError, match='some_other_problem'):
        load_progress(tmp_problem, schedule)


def test_changed_schedule_warns_once_and_keeps_statuses(schedule, tmp_problem, capsys):
    """A re-exported schedule prints one warning, keeps the statuses and adopts the new fingerprint."""
    progress = new_progress(schedule)
    progress.mark_done(schedule.entry(0), 'Cindy', 'Cindy-test')
    progress.schedule_fingerprint = '47:not-this-schedule'
    save_progress(progress, tmp_problem)

    loaded = load_progress(tmp_problem, schedule)
    out = capsys.readouterr().out
    assert out.count('\n') == 1 and '47:not-this-schedule' in out
    assert loaded.is_done(0) and loaded.current_index == 1
    # The current fingerprint is adopted, so the next save records it.
    assert loaded.schedule_fingerprint == schedule_fingerprint(schedule)


# * ------------------------------------------------------------- statuses and holds

def test_archive_progress_and_saved_plans(schedule, tmp_problem):
    """'Reset schedule to the Rhino export': progress and saved plans move aside, nothing else."""
    progress = new_progress(schedule)
    _mark_done_up_to(progress, schedule, 3)
    save_progress(progress, tmp_problem)
    actions = os.path.join(tmp_problem, 'BarActions')
    os.makedirs(actions)
    for name in ('B1__J.json', 'B1__J.live-solved.json', 'B3__H.live-solved.json'):
        with open(os.path.join(actions, name), 'w') as handle:
            handle.write('{}')
    older = os.path.join(tmp_problem, ARCHIVE_DIRNAME, 'older', 'BarActions')
    os.makedirs(older)
    with open(os.path.join(older, 'B1__R.live-solved.json'), 'w') as handle:
        handle.write('{}')

    folder, moved = archive_progress_and_saved_plans(tmp_problem, stamp='reset1')

    assert folder == os.path.join(tmp_problem, ARCHIVE_DIRNAME, 'reset1')
    assert sorted(moved) == sorted([PROGRESS_FILENAME,
                                    os.path.join('BarActions', 'B1__J.live-solved.json'),
                                    os.path.join('BarActions', 'B3__H.live-solved.json')])
    for rel in moved:
        assert os.path.isfile(os.path.join(folder, rel)), rel
        assert not os.path.exists(os.path.join(tmp_problem, rel)), rel
    # The clean export, the schedule and an earlier archive stay where they are.
    assert os.path.isfile(os.path.join(actions, 'B1__J.json'))
    assert os.path.isfile(os.path.join(tmp_problem, SCHEDULE_FILENAME))
    assert os.path.isfile(os.path.join(older, 'B1__R.live-solved.json'))

    # Loading again starts over: every entry pending, no beliefs.
    fresh = load_progress(tmp_problem, schedule)
    assert all(fresh.status(e.index) == STATUS_PENDING for e in schedule.entries)
    assert fresh.current_index == 0 and not fresh.robots

    # Nothing left to move: nothing moved, no folder created.
    folder2, moved2 = archive_progress_and_saved_plans(tmp_problem, stamp='reset2')
    assert moved2 == [] and not os.path.exists(folder2)


def test_mark_skip_reopen_and_holds(schedule):
    """Marking, skipping and reopening entries moves the current index and the hold states."""
    progress = new_progress(schedule)
    _mark_done_up_to(progress, schedule, 3)
    assert progress.current_index == 4
    assert progress.holding_bars() == {'Alice': 'B3'}
    assert progress.released_robots() == set()
    assert progress.hold_state('B3').since_index == 3
    assert progress.hold_state('B1') is None

    progress.skip(schedule.entry(4), 'Cindy', 'Cindy-test')
    assert progress.status(4) == STATUS_SKIPPED and progress.current_index == 5
    progress.reopen(schedule.entry(4))
    assert progress.status(4) == STATUS_PENDING and progress.current_index == 4

    # Alice releases B3 (entry 16) -- out of order is allowed, the pointer stays at 4.
    hr_belief = belief_after(schedule.load_action(schedule.entry(16)), ALICE)
    progress.mark_done(schedule.entry(16), 'Alice', 'Alice-test', belief=hr_belief)
    assert progress.current_index == 4
    assert progress.hold_state('B3').state == HOLD_RELEASED
    assert progress.hold_state('B3').released_at_index == 16
    assert progress.holding_bars() == {} and progress.released_robots() == {'Alice'}
    assert progress.belief('Alice').after_entry == 16

    # Reopening the release: holding again, and the belief it wrote is dropped.
    progress.reopen(schedule.entry(16))
    assert progress.hold_state('B3').state == HOLD_HOLDING
    assert progress.hold_state('B3').released_at_index is None
    assert progress.belief('Alice') is None

    # Reopening the hold: pending, pointer back to 3.
    progress.reopen(schedule.entry(3))
    assert progress.hold_state('B3').state == HOLD_PENDING
    assert progress.hold_state('B3').since_index is None
    assert progress.current_index == 3


def test_reopen_keeps_a_belief_from_another_entry(schedule):
    """Reopening drops a belief only when it came from the reopened entry."""
    progress = new_progress(schedule)
    _mark_done_up_to(progress, schedule, 3)
    assert progress.belief('Cindy').after_entry == 2   # her last done entry, B3_J
    progress.reopen(schedule.entry(1))                  # an older entry: belief stays
    assert progress.belief('Cindy').after_entry == 2
    progress.reopen(schedule.entry(2))                  # the entry it came from: dropped
    assert progress.belief('Cindy') is None
    assert progress.belief('Alice').after_entry == 3


def test_recompute_belief(schedule):
    """After a reopen, the belief is rebuilt from the robot's last done entry."""
    progress = new_progress(schedule)
    _mark_done_up_to(progress, schedule, 3)
    progress.mark_done(schedule.entry(16), 'Alice', 'Alice-test',
                       belief=belief_after(schedule.load_action(schedule.entry(16)), ALICE))
    progress.reopen(schedule.entry(16))
    assert progress.belief('Alice') is None

    rebuilt = recompute_belief(progress, schedule, 'Alice')
    assert rebuilt.after_entry == 3 and rebuilt.source == BELIEF_ACTION_END
    assert rebuilt.configuration.joint_values == pytest.approx(ALICE_HOLD_B3_CONF, abs=1e-3)
    assert recompute_belief(progress, schedule, 'Belle') is None


def test_new_run_id():
    """A run id is the robot name plus a compact timestamp."""
    assert new_run_id('Cindy', now=datetime(2026, 9, 30, 13, 55, 1)) == 'Cindy-20260930T135501'
    assert new_run_id('Alice').startswith('Alice-')


# * ------------------------------------------------------------- beliefs

def test_belief_after_hold_matches_the_next_export(schedule):
    """Alice's end state after B3_H is exactly what B4_J exports for her."""
    belief = belief_after(schedule.load_action(schedule.entry(3)), ALICE, entry_index=3)
    exported = schedule.load_action(schedule.entry(5)).movements[0] \
        .start_state.tool_states['ObstacleRobotAlice']

    assert belief.source == BELIEF_ACTION_END and belief.after_entry == 3
    assert belief.configuration.joint_names == ALICE.all_arm_joint_names
    assert belief.configuration.joint_values == pytest.approx(
        list(exported.configuration.joint_values), abs=1e-6)
    assert list(belief.base_frame.point) == pytest.approx(list(exported.frame.point), abs=1e-6)
    assert list(belief.base_frame.xaxis) == pytest.approx(list(exported.frame.xaxis), abs=1e-6)
    assert list(belief.base_frame.point) == pytest.approx(ALICE_HOLD_B3_BASE, abs=1e-4)
    assert belief.configuration.joint_values == pytest.approx(ALICE_HOLD_B3_CONF, abs=1e-3)


def test_belief_after_release_is_cindys_twelve_joints(schedule):
    """Cindy's belief after a release is the last movement's 12-joint target."""
    release = schedule.load_action(schedule.entry(4))
    belief = belief_after(release, CINDY)
    assert belief.configuration.joint_names == CINDY.all_arm_joint_names
    assert len(belief.configuration.joint_values) == 12
    assert belief.configuration.joint_values == pytest.approx(
        list(release.movements[-1].target_configuration.joint_values))


def test_arm_configuration_matches_joints_by_name():
    """Right arm listed first (as Cindy's URDF does) still comes out left first."""
    names = CINDY.arm_joint_names[1] + CINDY.arm_joint_names[0]
    conf = Configuration.from_revolute_values(list(range(12)), joint_names=list(names))
    ordered = arm_configuration(conf, CINDY)
    assert ordered.joint_names == CINDY.all_arm_joint_names
    assert ordered.joint_values == [6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5]
    with pytest.raises(ValueError):
        arm_configuration(conf, ALICE)


def test_belief_from_live():
    """A live belief turns the mocap quaternion into a frame and joins the arms' joints in side order."""
    # A 90 degree turn about z: quaternion (x, y, z, w) = (0, 0, sin 45, cos 45).
    half = 2 ** -0.5
    belief = belief_from_live(ALICE, ((1.0, 2.0, 0.0), (0.0, 0.0, half, half)),
                              [[0.1, 0.2, 0.3, 0.4, 0.5, 0.6]], entry_index=3)
    assert belief.source == BELIEF_LIVE and belief.after_entry == 3
    assert list(belief.base_frame.point) == pytest.approx([1.0, 2.0, 0.0])
    assert list(belief.base_frame.xaxis) == pytest.approx([0.0, 1.0, 0.0], abs=1e-9)
    assert belief.configuration.joint_names == ALICE.all_arm_joint_names
    assert belief.configuration.joint_values == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

    two_arms = belief_from_live(CINDY, ((0, 0, 0), (0, 0, 0, 1)), [[1.0] * 6, [2.0] * 6])
    assert two_arms.configuration.joint_names == CINDY.all_arm_joint_names
    assert two_arms.configuration.joint_values == [1.0] * 6 + [2.0] * 6


def test_belief_from_exported_and_parked(schedule):
    """Exported beliefs come from the obstacle tool states; parked is far away with zero joints."""
    # ! B3_R parks Alice even though she is holding B3 (exporter defect D1);
    # ! this is exactly why the persisted belief has to win over the export.
    exported = belief_from_exported(schedule.load_action(schedule.entry(4)), ALICE)
    assert exported.source == BELIEF_EXPORTED
    assert list(exported.base_frame.point) == pytest.approx([50.0, 50.0, 0.0])
    # Cindy's own action does not carry Cindy as an obstacle.
    assert belief_from_exported(schedule.load_action(schedule.entry(4)), CINDY) is None

    parked = parked_belief(BELLE)
    assert parked.source == BELIEF_PARKED
    assert list(parked.base_frame.point) == pytest.approx([50.0, 50.0, 0.0])
    assert parked.base_frame is not PARKED_BASE_FRAME
    assert parked.configuration.joint_names == BELLE.all_arm_joint_names
    assert parked.configuration.joint_values == [0.0] * 6


def test_obstacle_robots_after_the_hold(schedule):
    """While Cindy runs B3_R: Alice drawn from her belief, Belle from the export (parked)."""
    progress = new_progress(schedule)
    _mark_done_up_to(progress, schedule, 3)
    release = schedule.load_action(schedule.entry(4))

    states = obstacle_tool_states(progress, 'Cindy', exported_action=release)
    assert set(states) == {'ObstacleRobotAlice', 'ObstacleRobotBelle'}
    alice_frame, alice_conf = states['ObstacleRobotAlice']
    assert list(alice_frame.point) == pytest.approx(ALICE_HOLD_B3_BASE, abs=1e-4)
    assert alice_conf.joint_values == pytest.approx(ALICE_HOLD_B3_CONF, abs=1e-3)
    belle_frame, belle_conf = states['ObstacleRobotBelle']
    assert list(belle_frame.point) == pytest.approx([50.0, 50.0, 0.0])
    assert belle_conf.joint_names == BELLE.all_arm_joint_names

    assert obstacle_sources(progress, 'Cindy', exported_action=release) == {
        'ObstacleRobotAlice': BELIEF_ACTION_END, 'ObstacleRobotBelle': BELIEF_EXPORTED}
    assert obstacle_sources(progress, 'Cindy') == {
        'ObstacleRobotAlice': BELIEF_ACTION_END, 'ObstacleRobotBelle': BELIEF_PARKED}
    # From Alice's monitor, Cindy is drawn from her belief after B3_J (entry 2).
    assert obstacle_sources(progress, 'Alice')['ObstacleRobotCindy'] == BELIEF_ACTION_END


def test_obstacle_sources_line(schedule):
    """The header's 'others' line: each other robot's source, with the entry of a stamped belief."""
    progress = new_progress(schedule)
    assert obstacle_sources_line(progress, 'Cindy') == 'Alice <- parked | Belle <- parked'

    _mark_done_up_to(progress, schedule, 3)
    release = schedule.load_action(schedule.entry(4))
    assert obstacle_sources_line(progress, 'Cindy', exported_action=release) == \
        'Alice <- action_end_state (entry 3) | Belle <- exported'
    assert obstacle_sources_line(progress, 'Alice') == \
        'Cindy <- action_end_state (entry 2) | Belle <- parked'


def test_released_robot_is_parked(schedule):
    """A robot that released its hold is drawn parked, whatever its belief."""
    progress = new_progress(schedule)
    _mark_done_up_to(progress, schedule, 3)
    progress.mark_done(schedule.entry(16), 'Alice', 'Alice-test',
                       belief=belief_after(schedule.load_action(schedule.entry(16)), ALICE))
    release = schedule.load_action(schedule.entry(4))
    assert obstacle_sources(progress, 'Cindy', exported_action=release)['ObstacleRobotAlice'] \
        == BELIEF_PARKED
    frame, conf = obstacle_tool_states(progress, 'Cindy', exported_action=release)['ObstacleRobotAlice']
    assert list(frame.point) == pytest.approx([50.0, 50.0, 0.0])
    assert conf.joint_values == [0.0] * 6
