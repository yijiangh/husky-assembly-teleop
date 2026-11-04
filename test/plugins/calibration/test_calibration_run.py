"""A calibration run's results know what they were made from: out of date once that changes, and resumed as saved."""

import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from body_definition_fixtures import MARKERS, body, moved
from calibration_fixtures import SESSION, record_take
from repo_paths import DATA
from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.plugins.calibration.analysis.checks import (CHECKLIST_BOXES, DRIVE_SECONDS,
                                                                       PRECONDITION_HOLDS, TILT, TOOL_BOX)
from husky_assembly_teleop.plugins.calibration.storage.progress import Progress
from husky_assembly_teleop.plugins.calibration.run import Made, Run
from husky_assembly_teleop.plugins.calibration.robot.sampling import Reading, Sampler
from husky_assembly_teleop.plugins.calibration.robot.sweeps import named
from husky_assembly_teleop.plugins.calibration.storage.session_files import (pose_list, punch_take, read_punch,
                                                                             read_takes)
from husky_assembly_teleop.robot_interface.mocap_relay import (FRAME_HELD, FRAME_NONE, FloorCalibrationState,
                                                               MocapRelayState)
from husky_assembly_teleop.robot_interface.tool_step import tool_step_target
from husky_assembly_teleop.world.calibration_sessions import SESSIONS_FOLDER, read_session_file, sessions_folder
from husky_assembly_teleop.shared.guide.step import fails
from husky_assembly_teleop.world.checks import BAD, PENDING
from scaffolding_assembly_core.geometry import Pose, compose
from scaffolding_assembly_core.kinematics import link_pose

pytestmark = pytest.mark.safety

ARM = "ur_arm"
#: The punch's TCP offset, metres.
OFFSET = np.array([0.0, 0.0, 0.1])
#: The calibration the relay applied during the punch takes: base_footprint in the base rigid body.
CALIBRATION = Pose((0.01, -0.02, 0.3), tuple(Rotation.from_euler("z", 3.0, degrees=True).as_quat()))


def _robot() -> SimpleNamespace:
    """Belle, real base and arm."""
    return SimpleNamespace(config=robot_config_from_serial("0805", DATA), base=SimpleNamespace(simulation=None),
                           arms={ARM: SimpleNamespace(simulation=None)})


def _run(root: Path) -> Run:
    """A new run of Belle's arm with its run file, the Drive at `root`."""
    (root / SESSIONS_FOLDER).mkdir(parents=True, exist_ok=True)
    run = Run.new(_robot(), ARM, DATA, root)
    run.start_file()
    return run


def _record_all(run: Run, count: int = 3) -> None:
    for joint in (0, 1):
        for take in range(count):
            record_take(run, joint, take)


def _sample(x: float):
    """A still validation sample: base and tool where Belle's first J0 take began."""
    base, tool = SESSION["j0_base"][0], SESSION["j0_tool"][0]
    sampler = Sampler()
    sampler.add(Reading(Pose(tuple(base[:3] + (x, 0, 0)), tuple(base[3:])), Pose(tuple(tool[:3] + (x, 0, 0)),
                        tuple(tool[3:])), SESSION["j0_joints"][0], 1e-4, 1e-4))
    return sampler.sample(Pose(), "", 1.0)


def _validate(run: Run, *xs: float) -> list[Path]:
    """Validation samples at these x offsets, a file each, as Validate writes them; their files."""
    return [run.add_validation_sample(_sample(x), pose) for pose, x in enumerate(xs, start=1)]


def _punch(run: Run, x: float, offset: np.ndarray = OFFSET) -> dict:
    """A punch take of the stowed arm, base_footprint at `x` on the world x axis, under CALIBRATION."""
    base, tool0 = Pose((x, 0.0, 0.0)), f"{ARM}_tool0"
    tip = compose(link_pose(run.urdf, base, named(ARM, run.stow), tool0), Pose(tuple(offset)))
    return punch_take("single", 0, tool0, run.stow, base, tip, offset, CALIBRATION, "aaa")


def _defined(run: Run, base: str = "", tool: str = "") -> dict:
    """Motive's definitions of the run's base and tool bodies: as first defined, or "refined", "renamed" or "pivot"."""
    def definition(streaming_id: int, points, change: str):
        changed = {"": points, "renamed": points, "refined": moved(points, noise=0.3e-3, seed=streaming_id),
                   "pivot": moved(points, shift=(0.002, 0.0, 0.0))}[change]
        return body(changed, streaming_id, f"body {streaming_id}" + (" renamed" if change == "renamed" else ""))

    base_id = run.robot.config.mocap_id
    return {base_id: definition(base_id, MARKERS * 1.6, base), run.tool_body: definition(run.tool_body, MARKERS, tool)}


def _tips(takes: list[dict]) -> np.ndarray:
    """The takes' tips as stored: under the calibration applied then and the TCP offset used."""
    return np.array([take["world_from_punch_tip"]["position"] for take in takes])


def test_takes_are_current_only_with_the_tool_body_they_were_measured_with(tmp_path):
    """Another tool body makes every take out of date, to record again; the one before brings them back."""
    run = _run(tmp_path)
    run.collisions = {index: "" for index in range(len(run.sweeps.sweeps))}
    _record_all(run)
    body = run.tool_body
    assert len(run.good_takes()) == 6 and len(run.pending()) == 6
    assert all(take.made == Made(body) for take in run.takes.values())
    run.tool_body = 99
    assert run.current_takes() == {} and run.good_takes() == [] and len(run.pending()) == 12
    run.tool_body = body
    assert len(run.good_takes()) == 6


def test_a_solution_is_current_only_for_the_takes_it_came_from(tmp_path):
    """Solve writes the relay's file in progress; a new take puts it out of date until Solve runs again."""
    run = _run(tmp_path)
    _record_all(run)
    solved = run.solve_and_write()
    assert run.current_solved() is solved and solved.output.name.endswith("_in_progress.json")
    assert solved.sanity is None
    record_take(run, 0, 3)
    assert run.current_solved() is None and not run.finished
    assert run.solve_and_write() is run.current_solved()


def test_applied_is_finished_only_for_the_current_output(tmp_path):
    """The relay applying the output finishes the run, until the takes change."""
    run = _run(tmp_path)
    _record_all(run)
    run.applied_sha = run.solve_and_write().output_sha
    assert run.finished
    record_take(run, 1, 4)
    assert not run.finished


def test_a_mount_check_is_finished_once_written(tmp_path):
    """A mount check has nothing to apply: it counts as applied once Solve wrote it, until the takes change."""
    run = _run(tmp_path)
    run.mount_check = True
    _record_all(run)
    assert not run.finished
    run.solve_and_write()
    run.save()
    assert run.finished and run.applied_sha is None and Progress.load(run.progress.file).applied
    record_take(run, 0, 3)
    assert not run.finished


def test_the_run_file_starts_only_when_asked_for_the_first_thing_recorded(tmp_path):
    """A plain save of a new run writes nothing; a save that starts it makes the run file."""
    (tmp_path / SESSIONS_FOLDER).mkdir(parents=True)
    run = Run.new(_robot(), ARM, DATA, tmp_path)
    run.notes = "first"
    run.save()
    assert run.progress is None and run.session is None
    run.save(start=True)
    assert Progress.load(run.progress.file).notes == "first"


def test_a_base_refine_puts_takes_validation_and_punch_takes_out_of_date_and_archives_them(tmp_path):
    """The base body is the frame the takes are in: after a refine in Motive, nothing measured before holds."""
    run = _run(tmp_path)
    run.follow(_defined(run))
    run.marker_max, run.drive_seconds, run.marker_time, run.marker_made = [1e-4, 1e-4], 6.0, 1e9, run.made
    _record_all(run, 2)
    run.applied_sha = run.solve_and_write().output_sha
    spot = _validate(run, 0.0, 0.001)
    run.punch_offset = OFFSET
    first = run.add_punch_take(_punch(run, 0.0), "single")
    assert len(run.validation().bases) == 2 and run.validation().spread is not None and run.finished
    moved = run.follow(_defined(run, base="refined"))
    assert len(moved) == 6 and all(path.parent.name == "archive" for path in moved)
    # * Kept, out of date: their files moved to archive/, so an offline solve of the session leaves them out.
    assert len(run.takes) == 4 and run.current_takes() == {} and len(run.validation_files) == 2
    assert not any(path.exists() for path in spot)
    assert run.current_validation_files() == []
    assert run.validation().spread is None and run.current_solved() is None and run.current_punch_takes() == []
    assert run.marker_expired(1e9) == "base body refined since measured" and not run.finished
    # * The punch take stays until the next, which starts a new file.
    assert len(run.punch_takes) == 1 and first.is_file()
    run.add_punch_take(_punch(run, 1.0), "single")
    assert (first.parent / "archive" / first.name).is_file() and len(run.current_punch_takes()) == 1


def test_a_tool_refine_puts_only_validation_out_of_date(tmp_path):
    """A tool-body refine moves the tool point rigidly on the flange: the takes and the solution stay current."""
    run = _run(tmp_path)
    run.follow(_defined(run))
    _record_all(run, 2)
    solved = run.solve_and_write()
    spot = _validate(run, 0.0, 0.001)
    run.punch_offset = OFFSET
    run.add_punch_take(_punch(run, 0.0), "single")
    moved = run.follow(_defined(run, tool="refined"))
    assert moved == [path.parent / "archive" / path.name for path in spot] and list(run.validation_files) == moved
    assert run.current_validation_files() == []
    assert len(run.current_takes()) == 4 and run.current_solved().output_sha == solved.output_sha
    assert len(run.current_punch_takes()) == 1


def test_validation_belongs_to_the_tool_body_not_to_the_solution(tmp_path):
    """Another tool body puts validation out of date; a new solution judges the same samples again."""
    run = _run(tmp_path)
    _record_all(run, 2)
    run.solve_and_write()
    _validate(run, 0.0, 0.001)
    body, run.tool_body = run.tool_body, 99
    assert run.current_validation_files() == [] and run.validation().bases == []
    run.tool_body = body
    record_take(run, 0, 2)
    assert run.current_solved() is None and run.validation().spread is None
    run.solve_and_write()
    assert len(run.validation().spots) == 1 and run.validation().spread is not None


def test_the_tip_offset_holds_through_refines(tmp_path):
    """Nothing the monitor sees changes the tip offset's poses: refines of either body keep them and the offset."""
    run = _run(tmp_path)
    start = np.radians([90.0, -100.0, 90.0, -80.0, -90.0, 0.0])
    for joints in [start, tilted(run, start, "x", 20), tilted(run, start, "x", -20), tilted(run, start, "y", 20)]:
        run.add_tip_pose(joints)
    assert run.commit_tip_offset()
    offset = run.punch_offset.copy()
    run.follow(_defined(run, base="refined", tool="refined"))
    assert len(run.current_tip()) == 4 and np.array_equal(run.punch_offset, offset) and run.tip_offset_used()
    assert not fails(run.tip_checks())


def test_a_resumed_run_has_what_was_saved_current_or_not(tmp_path):
    """Takes with what they were measured with, the solution (found by its output's hash) and validation come back."""
    run = _run(tmp_path)
    _record_all(run)
    solved = run.solve_and_write()
    _validate(run, 0.0)
    body, run.tool_body = run.tool_body, 77
    old = record_take(run, 0, 4)
    run.tool_body = body
    run.save()
    resumed = Run.resume(_robot(), Progress.load(run.progress.file), DATA, tmp_path)
    assert resumed.takes.keys() == run.takes.keys() and resumed.takes[old].made == Made(77)
    assert old not in resumed.current_takes() and len(resumed.good_takes()) == 6
    # * The out-of-date take is not among Solve's inputs, so the solution is still current.
    assert resumed.current_solved().output_sha == solved.output_sha
    assert len(resumed.validation().bases) == 1 and resumed.validation().spread is not None
    record_take(run, 1, 4)
    run.save()
    assert Run.resume(_robot(), Progress.load(run.progress.file), DATA, tmp_path).current_solved() is None


def test_an_older_run_file_without_stamps_resumes_with_everything_current(tmp_path):
    """A run file from before the stamps: its takes count as measured with its tool body, definitions not known."""
    run = _run(tmp_path)
    _record_all(run, 2)
    run.save()
    data = json.loads(run.progress.file.read_text())
    for key in ("made", "marker_made", "acceptances"):
        del data[key]
    run.progress.file.write_text(json.dumps(data))
    resumed = Run.resume(_robot(), Progress.load(run.progress.file), DATA, tmp_path)
    assert len(resumed.good_takes()) == 4


def test_an_older_run_file_without_marker_time_asks_for_the_marker_errors_again(tmp_path):
    """Marker errors from before their time was kept do not count: when they were measured is not known."""
    run = _run(tmp_path)
    run.marker_max, run.drive_seconds, run.marker_time = [1e-4, 1e-4], 6.0, 1_000_000.0
    run.save()
    data = json.loads(run.progress.file.read_text())
    del data["marker_time"]
    run.progress.file.write_text(json.dumps(data))
    resumed = Run.resume(_robot(), Progress.load(run.progress.file), DATA, tmp_path)
    assert resumed.marker_max == [1e-4, 1e-4]
    assert resumed.marker_expired(1_000_000.0) == "measured, but when is not known"


@pytest.mark.parametrize("made", [lambda made: [made["tool_body"], 1], lambda made: [made["tool_body"], 2, 1]])
def test_an_older_run_file_counting_refines_has_its_definitions_not_known(tmp_path, made):
    """Refine counts (one for both bodies, or one each) say nothing about Motive's definitions: never out of date."""
    run = _run(tmp_path)
    _record_all(run, 2)
    run.save()
    data = json.loads(run.progress.file.read_text())
    data["refines"] = [2, 1]
    data["made"] = {name: made(entry) for name, entry in data["made"].items()}
    run.progress.file.write_text(json.dumps(data))
    resumed = Run.resume(_robot(), Progress.load(run.progress.file), DATA, tmp_path)
    resumed.follow(_defined(resumed, base="refined"))
    assert len(resumed.good_takes()) == 4
    assert all(take.made == Made(run.tool_body) for take in resumed.takes.values())


def test_measurements_keep_the_definitions_they_were_made_with_through_a_resume(tmp_path):
    """Takes, validation, punch takes and the marker errors: saved with both bodies' definitions, judged on resume."""
    run = _run(tmp_path)
    run.follow(_defined(run))
    run.marker_max, run.drive_seconds, run.marker_time, run.marker_made = [1e-4, 1e-4], 6.0, 1e9, run.made
    _record_all(run, 2)
    _validate(run, 0.0)
    run.punch_offset = OFFSET
    run.add_punch_take(_punch(run, 0.0), "single")
    run.save()
    resumed = Run.resume(_robot(), Progress.load(run.progress.file), DATA, tmp_path)
    assert all(take.made == run.made for take in resumed.takes.values()) and resumed.marker_made == run.made
    resumed.follow(_defined(resumed, base="pivot"))
    assert resumed.current_takes() == {} and resumed.current_validation_files() == []
    assert resumed.current_punch_takes() == [] and "base body pivot moved" in resumed.marker_expired(1e9)


def test_a_renamed_body_changes_nothing_and_putting_a_definition_back_makes_its_results_current(tmp_path):
    """A rename is no data change; undoing a refine (the old definition back) brings the takes back from archive/."""
    run = _run(tmp_path)
    run.follow(_defined(run))
    _record_all(run, 2)
    assert run.follow(_defined(run, base="renamed")) == [] and len(run.current_takes()) == 4
    assert len(run.follow(_defined(run, base="refined"))) == 4 and run.current_takes() == {}
    run.follow(_defined(run))
    assert len(run.current_takes()) == 4 and all(take.file.parent.name != "archive" for take in run.takes.values())


def test_without_definitions_nothing_goes_out_of_date(tmp_path):
    """A simulation's relay announces none: measurements made without them hold whatever Motive later says."""
    run = _run(tmp_path)
    _record_all(run, 2)
    run.follow(_defined(run, base="refined", tool="refined"))
    assert len(run.current_takes()) == 4


def test_a_refine_ends_once_its_body_definition_changes_and_the_marker_errors_are_measured_again(tmp_path):
    """Refining the tool body: a change to the base does not end it; the tool's new definition does."""
    run = _run(tmp_path)
    run.follow(_defined(run))
    run.marker_max, run.drive_seconds, run.marker_time, run.marker_made = [5e-4, 1e-3], 6.0, 1e9, run.made
    run.start_refine("tool")
    run.follow(_defined(run, base="renamed"))
    assert run.refining == "tool" and run.marker_max is not None
    run.follow(_defined(run, tool="refined"))
    assert run.refining is None and run.marker_max is None and run.marker_expired(1e9) == "not measured"


def test_solve_writes_the_base_body_definition_the_takes_were_recorded_with(tmp_path):
    """The relay reads it back from the calibration file, to check it against Motive's (Z-up, as announced)."""
    run = _run(tmp_path)
    run.follow(_defined(run))
    _record_all(run, 2)
    output = run.solve_and_write().output
    calibration = read_session_file(output, tmp_path, run.robot.config.mocap_id, run.robot.config.serial)
    assert calibration.body_definition == run.made.base


def test_preconditions_hold_for_an_hour_and_the_checklist_only_for_its_tool_model(tmp_path):
    """Marker errors and each box's tick expire after PRECONDITION_HOLDS; Tool on flange holds for its tool only.

    A resume keeps all of it.
    """
    run, now = _run(tmp_path), 1_000_000.0
    assert run.marker_expired(now) == "not measured" and "Area clear not ticked" in run.checklist_expired(now)
    run.marker_time, run.drive_seconds = now, DRIVE_SECONDS
    for label in CHECKLIST_BOXES:
        run.tick(label, True, now)
    run.tick("Area clear", True, now + 60)
    assert run.marker_expired(now + PRECONDITION_HOLDS) is None
    assert run.checklist_expired(now + PRECONDITION_HOLDS) is None
    assert "min ago" in run.marker_expired(now + PRECONDITION_HOLDS + 1)
    expired = run.checklist_expired(now + PRECONDITION_HOLDS + 1)
    assert "Tool on flange ticked" in expired and "min ago" in expired and "Area clear" not in expired
    run.tool_ticked_for = "calibration_marker"
    assert run.checklist_expired(now) == "Tool on flange ticked for calibration_marker"
    run.save()
    resumed = Run.resume(_robot(), Progress.load(run.progress.file), DATA, tmp_path)
    assert (resumed.marker_time, resumed.ticks, resumed.tool_ticked_for) == (now, run.ticks, "calibration_marker")
    resumed.tick("Area clear", False, now)
    assert resumed.box_expired("Area clear", now) == "Area clear not ticked"


def test_an_older_run_file_with_one_checklist_time_resumes_its_ticks(tmp_path):
    """One time for the boxes but Tool on flange, and that box's [model, time]: each box gets its time."""
    run = _run(tmp_path)
    run.save()
    data = json.loads(run.progress.file.read_text())
    del data["ticks"], data["tool_ticked_for"]
    data["checklist_time"], data["tool_checked"] = 100.0, ["none", 200.0]
    run.progress.file.write_text(json.dumps(data))
    resumed = Run.resume(_robot(), Progress.load(run.progress.file), DATA, tmp_path)
    assert resumed.ticks == {**{label: 100.0 for label in CHECKLIST_BOXES}, TOOL_BOX: 200.0}
    assert resumed.tool_ticked_for == "none"


def test_another_run_of_the_arm_takes_the_newer_ticks(tmp_path):
    """The checklist is about the arm: a new run adopts each box's tick of the run before, unless its own is newer."""
    old, new = _run(tmp_path), Run.new(_robot(), ARM, DATA, tmp_path)
    old.ticks, old.tool_ticked_for = {"Area clear": 100.0, TOOL_BOX: 100.0}, "none"
    new.ticks = {"Area clear": 150.0}
    new.adopt_checklist(old)
    assert new.ticks == {"Area clear": 150.0, TOOL_BOX: 100.0} and new.tool_ticked_for == "none"


def tilted(run: Run, joints: np.ndarray, axis: str, degrees: float, tip=(0.0, 0.0, 0.11)) -> np.ndarray:
    """The arm turned about `tip` by `degrees` about tool0's `axis`, in tool steps of at most 5°."""
    for _ in range(int(abs(degrees) // 5)):
        turn = Pose(orientation=tuple(Rotation.from_euler(axis, np.copysign(5.0, degrees), degrees=True).as_quat()))
        joints = tool_step_target(run.urdf, ARM, joints, turn, tip)
    return joints


def _tip_poses(run: Run) -> list[np.ndarray]:
    """Four poses of the tip 0.11 m out of tool0 on one point, tilted about two axes."""
    start = np.radians([90.0, -100.0, 90.0, -80.0, -90.0, 0.0])
    return [start, tilted(run, start, "x", 20), tilted(run, start, "x", -20), tilted(run, start, "y", 20)]


def test_the_tip_fit_becomes_the_tcp_offset_only_once_committed_and_resumes_as_saved(tmp_path):
    """The fit is a calculation: adding poses never changes the TCP offset, committing does; clearing keeps it.

    A resume has the poses and the committed offset.
    """
    run = _run(tmp_path)
    run.punch_offset = np.array([0.0, 0.0, 0.1])
    poses = _tip_poses(run)
    for joints in poses[:3]:
        run.add_tip_pose(joints)
    assert run.tip_pivot() is not None and fails(run.tip_checks())
    run.add_tip_pose(poses[3])
    assert not fails(run.tip_checks()) and np.allclose(run.punch_offset, [0.0, 0.0, 0.1])
    assert not run.tip_offset_used() and run.commit_tip_offset() and run.tip_offset_used()
    np.testing.assert_allclose(run.punch_offset, (0.0, 0.0, 0.11), atol=1e-6)
    run.save()
    resumed = Run.resume(_robot(), Progress.load(run.progress.file), DATA, tmp_path)
    np.testing.assert_allclose(resumed.tip_joints, poses)
    assert len(resumed.current_tip()) == 4 and np.allclose(resumed.punch_offset, run.punch_offset)
    assert resumed.tip_offset_used()
    resumed.clear_tip()
    assert resumed.tip_joints == [] and resumed.tip_pivot() is None and not resumed.commit_tip_offset()
    assert np.allclose(resumed.punch_offset, run.punch_offset)


def test_a_remount_puts_the_tip_poses_and_the_punch_takes_out_of_date(tmp_path):
    """Poses and takes from before the remount count no more; the committed offset stays; the next pose starts anew.

    An older run file without the counts reads them as zero, with everything current.
    """
    run = _run(tmp_path)
    for joints in _tip_poses(run):
        run.add_tip_pose(joints)
    run.commit_tip_offset()
    offset = run.punch_offset.copy()
    run.add_punch_take(_punch(run, 0.0), "single")
    run.save()
    data = json.loads(run.progress.file.read_text())
    for key in ("punch_mount", "punch_target", "punch_mounts", "punch_targets", "tip_mounts"):
        del data[key]
    run.progress.file.write_text(json.dumps(data))
    older = Run.resume(_robot(), Progress.load(run.progress.file), DATA, tmp_path)
    assert len(older.current_tip()) == 4 and len(older.current_punch_takes()) == 1
    run.remounted()
    assert run.current_tip() == [] and run.tip_pivot() is None and run.current_punch_takes() == []
    assert np.array_equal(run.punch_offset, offset) and len(run.tip_joints) == 4
    run.add_tip_pose(_tip_poses(run)[0])
    assert len(run.tip_joints) == 1 and len(run.current_tip()) == 1
    run.save()
    resumed = Run.resume(_robot(), Progress.load(run.progress.file), DATA, tmp_path)
    assert resumed.punch_mount == 1 and len(resumed.current_tip()) == 1 and resumed.current_punch_takes() == []


def test_a_tip_fit_with_a_red_check_is_not_used_on_its_own(tmp_path):
    """Poses tilted about one axis only fit, but their tilt is red: the offset entered stays."""
    run = _run(tmp_path)
    run.punch_offset = np.array([0.0, 0.0, 0.1])
    start = np.radians([90.0, -100.0, 90.0, -80.0, -90.0, 0.0])
    for joints in [start, tilted(run, start, "x", 5), tilted(run, start, "x", 10), tilted(run, start, "x", -5)]:
        run.add_tip_pose(joints)
    assert run.tip_pivot() is not None and {c.label: c.level for c in run.tip_checks()}[TILT] == BAD
    assert np.allclose(run.punch_offset, [0.0, 0.0, 0.1])


def test_punch_takes_stay_current_through_solutions_offsets_and_apply(tmp_path):
    """Takes are raw measurements: a new solution, another TCP offset or Apply keeps them; judged anew under each."""
    run = _run(tmp_path)
    run.punch_offset = OFFSET
    file = run.add_punch_take(_punch(run, 0.0), "single")
    run.add_punch_take(_punch(run, 1.0), "single")
    _record_all(run, 2)
    run.applied_sha = run.solve_and_write().output_sha
    assert len(run.current_punch_takes()) == 2 and run.finished
    # * Under the calibration applied then, with the offset then, the tips are those stored.
    tips, bases = run.punch_tips(CALIBRATION)
    np.testing.assert_allclose(tips, _tips(run.punch_takes), atol=1e-9)
    np.testing.assert_allclose([base.position for base in bases], [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], atol=1e-9)
    run.punch_offset = OFFSET + 0.001
    moved = np.linalg.norm(run.punch_tips(CALIBRATION)[0] - _tips(run.punch_takes), axis=1)
    assert len(run.current_punch_takes()) == 2 and np.allclose(moved, np.sqrt(3) * 0.001)
    run.add_punch_take(_punch(run, 2.0, run.punch_offset), "single")
    assert len(run.current_punch_takes()) == 3 and file.is_file()
    # * The punch file keeps the old monitor's keys, each take with the offset it was taken with.
    stored = read_punch(file)
    assert {"base_pose", "world_from_punch_tip", "tool0_from_punch_tip", "joint_conf", "base_mocap_pose",
            "applied_calibration"} <= set(stored[0])
    assert stored[0]["tool0_from_punch_tip"]["position"] == list(OFFSET)


def test_the_punch_verdict_is_judged_under_the_calibration_given(tmp_path):
    """A calibration moving base_footprint 1 cm up in the base body moves every tip 1 cm up."""
    run = _run(tmp_path)
    run.punch_offset = OFFSET
    for x in (0.0, 1.0):
        run.add_punch_take(_punch(run, x), "single")
    tips, _ = run.punch_tips(compose(CALIBRATION, Pose((0.0, 0.0, 0.01))))
    np.testing.assert_allclose(tips, _tips(run.punch_takes) + [0.0, 0.0, 0.01], atol=1e-5)


def test_punch_takes_go_out_of_date_when_the_target_moves(tmp_path):
    """The next take starts a new file; a resume keeps the count."""
    run = _run(tmp_path)
    first = run.add_punch_take(_punch(run, 0.0), "single")
    run.target_moved()
    assert run.current_punch_takes() == []
    run.add_punch_take(_punch(run, 1.0), "single")
    assert (first.parent / "archive" / first.name).is_file() and len(run.current_punch_takes()) == 1
    run.save()
    resumed = Run.resume(_robot(), Progress.load(run.progress.file), DATA, tmp_path)
    assert resumed.punch_target == 1 and len(resumed.current_punch_takes()) == 1


def test_punch_takes_go_out_of_date_when_the_lab_frame_moves(tmp_path):
    """A floor calibration applied or a refit used moves the frame the tips are compared in; within 1 mm it holds.

    A frame not known, then (an older take) or now (no announcement yet), changes nothing; the frame back, the take
    is current again, also after a resume.
    """
    run = _run(tmp_path)
    turned = tuple(Rotation.from_euler("z", 0.1, degrees=True).as_quat())
    still = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    state = MocapRelayState(frame=FloorCalibrationState(FRAME_HELD, "floor.json", lab_from_raw=still))
    run.relay = SimpleNamespace(state=state)
    take = _punch(run, 0.0)
    take["lab_from_raw"] = pose_list(run.lab_frame())
    run.add_punch_take(take, "single")
    run.add_punch_take({key: value for key, value in _punch(run, 1.0).items() if key != "lab_from_raw"}, "single")
    state.frame = replace(state.frame, lab_from_raw=((0.0005, 0.0, 0.0), (0, 0, 0, 1)))
    assert len(run.current_punch_takes()) == 2
    state.frame = replace(state.frame, lab_from_raw=((0.0, 0.0, 0.0), turned))
    assert run.frame_moved(Pose()) == "lab frame moved 7.4 mm" and run.current_punch_takes() == [run.punch_takes[1]]
    state.frame = None
    assert len(run.current_punch_takes()) == 2
    state.frame = FloorCalibrationState(FRAME_NONE)
    assert run.lab_frame() == Pose() and len(run.current_punch_takes()) == 2
    run.save()
    resumed = Run.resume(_robot(), Progress.load(run.progress.file), DATA, tmp_path, run.relay)
    state.frame = FloorCalibrationState(FRAME_HELD, "floor.json", lab_from_raw=((0.0, 0.0, 0.0), turned))
    assert resumed.current_punch_takes() == [resumed.punch_takes[1]]


def test_an_older_punch_take_is_judged_under_the_calibration_applied_then(tmp_path):
    """Without the base body's own pose, a take's base is the base_footprint it stored, whatever the calibration."""
    run = _run(tmp_path)
    run.punch_offset = OFFSET
    take = _punch(run, 1.0)
    del take["base_mocap_pose"], take["applied_calibration"]
    run.add_punch_take(take, "single")
    tips, bases = run.punch_tips(Pose((0.0, 0.0, 0.5)))
    np.testing.assert_allclose(tips, _tips([take]), atol=1e-9)
    assert np.allclose(bases[0].position, (1.0, 0.0, 0.0))


def test_validation_may_be_sampled_before_solve_and_is_pending_until_then(tmp_path):
    """Samples need no solution: their spread is pending until one is current, then judged under it."""
    run = _run(tmp_path)
    _validate(run, 0.0, 0.001)
    levels = {check.label: check.level for check in run.validation_verdict()}
    assert levels["position"] == levels["orientation"] == PENDING and not run.validation_passes()
    _record_all(run, 2)
    run.solve_and_write()
    assert {check.label: check.level for check in run.validation_verdict()}["position"] != PENDING


def test_an_acceptance_holds_while_its_inputs_stay_the_same(tmp_path):
    """Validate's acceptance is stamped on the solution and the sample files, Record's on the take files.

    A resume keeps it; older run files' overrides are read but accept nothing.
    """
    run = _run(tmp_path)
    _record_all(run, 2)
    run.solve_and_write()
    _validate(run, 0.0)
    assert not run.validation_passes() and not run.accepted("Validate")
    run.accept("Validate")
    run.accept("Record")
    assert run.accepted("Validate") and run.accepted("Record")
    run.save()
    assert Run.resume(_robot(), Progress.load(run.progress.file), DATA, tmp_path).accepted("Validate")
    run.clear_validation()
    assert not run.accepted("Validate") and run.accepted("Record")
    run.accept("Validate")
    record_take(run, 0, 2)
    assert not run.accepted("Validate") and not run.accepted("Record")
    data = json.loads(run.progress.file.read_text())
    del data["acceptances"]
    data["overrides"] = [{"step": "Validate", "checks": ["Validate: samples"], "time": "10:00:00"}]
    run.progress.file.write_text(json.dumps(data))
    older = Run.resume(_robot(), Progress.load(run.progress.file), DATA, tmp_path)
    assert older.progress.overrides and not older.accepted("Validate")


def test_red_marker_errors_expire_the_markers_precondition_unless_accepted(tmp_path):
    """Measured within the hour is not enough: a red body expires it, until accepted for that measurement."""
    run, now = _run(tmp_path), 1_000_000.0
    run.marker_max, run.drive_seconds, run.marker_time = [1e-4, 3e-3], 6.0, now
    assert run.marker_age(now) is None and run.marker_expired(now) == "tool marker error over 2 mm"
    run.accept("Marker error")
    assert run.marker_expired(now) is None
    run.marker_max = [1e-4, 3.5e-3]
    assert run.marker_expired(now) == "tool marker error over 2 mm"
    run.marker_max = [1e-4, 1.5e-3]
    assert run.marker_expired(now) is None


def test_marker_errors_driven_too_short_expire_the_precondition(tmp_path):
    """A tick of driving is not a measurement: the precondition expires until DRIVE_SECONDS were driven."""
    run, now = _run(tmp_path), 1_000_000.0
    run.marker_max, run.drive_seconds, run.marker_time = [1e-4, 1e-4], 0.05, now
    assert run.marker_expired(now) == "driven under 5 s"
    run.drive_seconds = DRIVE_SECONDS
    assert run.marker_expired(now) is None


def test_a_validation_verdict_is_good_only_passing_or_failing_and_accepted(tmp_path):
    """No samples or no solution: nothing to accept, whatever stamp the run holds."""
    run = _run(tmp_path)
    run.accept("Validate")
    assert not run.validation_good()
    _record_all(run, 2)
    _validate(run, 0.0)
    run.accept("Validate")
    assert not run.validation_good()  # * pending: no solution
    run.solve_and_write()
    assert not run.validation_good()  # * the stamp was for no solution
    run.accept("Validate")
    assert run.validation_good()


def test_punchs_acceptance_covers_the_calibration_it_was_judged_under(tmp_path):
    """Stamped with the base_footprint judged under: another solution or relay calibration ends it."""
    run = _run(tmp_path)
    run.punch_offset = OFFSET
    run.add_punch_take(_punch(run, 0.0), "single")
    run.accept("Punch", CALIBRATION)
    assert run.accepted("Punch", CALIBRATION) and not run.accepted("Punch", Pose())


def test_measurements_put_out_of_date_are_kept_and_current_again_once_their_input_is_back(tmp_path):
    """Solve archives the files of out-of-date takes and validation spots but keeps them; back, they count again.

    The session's folders then hold what Solve used, as an offline solve reads every file there.
    """
    run = _run(tmp_path)
    _record_all(run, 2)
    spot, = _validate(run, 0.0)
    body = run.tool_body
    run.tool_body = body + 1
    assert run.tidy() and len(run.takes) == 4 and run.current_takes() == {} and len(run.validation_files) == 1
    assert all(take.file.parent.name == "archive" for take in run.takes.values()) and not spot.exists()
    run.tool_body = body
    run.solve_and_write()
    assert len(run.current_takes()) == 4 and run.current_validation_files() == [spot]
    assert all(take.file.parent.name != "archive" for take in run.takes.values()) and spot.exists()
    assert len(read_takes(run.session)[0]) == 4
    run.save()
    resumed = Run.resume(_robot(), Progress.load(run.progress.file), DATA, tmp_path)
    assert len(resumed.current_takes()) == 4 and resumed.current_validation_files() == [spot]


def test_on_stand_and_the_notes_are_kept_in_the_run_file(tmp_path):
    """A resume brings them back; an older run file without them reads as on wheels, no notes."""
    run = _run(tmp_path)
    run.on_stand, run.notes = True, "marker plate re-glued before take 3"
    run.save()
    resumed = Run.resume(_robot(), Progress.load(run.progress.file), DATA, tmp_path)
    assert (resumed.on_stand, resumed.notes) == (True, "marker plate re-glued before take 3")
    data = json.loads(run.progress.file.read_text())
    del data["on_stand"], data["notes"]
    run.progress.file.write_text(json.dumps(data))
    older = Run.resume(_robot(), Progress.load(run.progress.file), DATA, tmp_path)
    assert (older.on_stand, older.notes) == (False, "")


def test_a_run_reads_the_calibration_its_relay_applies(tmp_path):
    """Motive's relay or a simulation's (sim:=true) alike: its one set of calibrations; none without a relay."""
    run = _run(tmp_path)
    assert run.applied() is None
    run.relay = SimpleNamespace(state=MocapRelayState(calibrations={run.robot.config.mocap_id: "applied"}))
    assert run.applied() == "applied"


def test_a_run_through_a_simulations_relay_goes_to_its_domains_sessions_marked_sim(tmp_path, monkeypatch):
    """Never the robots' sessions: the simulation's of its ROS domain, and its files say it was simulated."""
    monkeypatch.setenv("ROS_DOMAIN_ID", "72")
    robot = _robot()
    robot.base.simulation = robot.arms[ARM].simulation = SimpleNamespace(node="husky_sim")
    (tmp_path / SESSIONS_FOLDER).mkdir(parents=True)
    run = Run.new(robot, ARM, DATA, tmp_path, SimpleNamespace(state=MocapRelayState(simulated=True,
                                                                                    calibrations_time=1.0)))
    run.start_file()
    assert run.session.parent == tmp_path / sessions_folder(True, 72)
    assert not list((tmp_path / SESSIONS_FOLDER).iterdir())
    saved = json.loads(run.progress.file.read_text())
    assert (saved["environment"], saved["ros_domain"]) == ("sim", 72)
    assert Progress.load(run.progress.file).environment == "sim"
    _record_all(run, 2)
    assert json.loads(run.solve_and_write().output.read_text())["environment"] == "sim"


@pytest.mark.parametrize("relay_simulated, base_simulated, arm_simulated, why", [
    (False, False, True, "the robot's arm is simulated, but the relay reads Motive"),
    (False, True, True, "the robot's base and arm are simulated, but the relay reads Motive"),
    (True, True, False, "the relay reads a simulation, but the robot's arm is real")])
def test_a_run_mixing_a_simulation_and_motive_writes_nothing(tmp_path, relay_simulated, base_simulated,
                                                             arm_simulated, why):
    """A mock arm on a real base measured through Motive's relay would land where that relay applies it: refused."""
    robot = _robot()
    robot.base.simulation = SimpleNamespace(node="husky_sim") if base_simulated else None
    robot.arms[ARM].simulation = SimpleNamespace(node="mock") if arm_simulated else None
    (tmp_path / SESSIONS_FOLDER).mkdir(parents=True)
    run = Run.new(robot, ARM, DATA, tmp_path, SimpleNamespace(state=MocapRelayState(simulated=relay_simulated,
                                                                                    calibrations_time=1.0)))
    with pytest.raises(FileNotFoundError, match=why):
        run.start_file()
    assert run.progress is None and not list(tmp_path.rglob("*.json"))
