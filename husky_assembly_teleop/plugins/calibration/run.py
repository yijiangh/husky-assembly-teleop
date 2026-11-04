"""
One calibration run: the chosen arm, what it measured and found, and which of it is current.

Results are built from data; a result is current while that is unchanged, else out of date, and so is everything built
on it. Putting the input back makes it current again; time never ages data.

  measurement          made with                                            current while (`*_current`)
  take                 tool body, base body definition (`Made`)             the same tool body, no data change
                                                                            to the definition (`changes`)
  validation sample    tool body, base and tool body definitions            the same, no data change to either
  tip pose             punch mount count                                    no remount since (`remounted`)
  punch take           base body definition, punch mount and target         no data change, the same counts, the
                       counts, the lab frame; it stores the base body's     lab frame within VOLUME_MOVED; new
                       own pose, joints, TCP offset used and the            solutions, TCP offsets and Apply keep
                       calibration applied                                  it

Body definitions are Motive's, as the relay announces them (`follow`); a data change is a refine, a body made again,
a moved pivot or another body on the id (`mocap_definitions.compare`). A definition not known, then or now (a
simulation, an older run), changes nothing. The lab frame is the relay's lab_from_raw (`lab_frame`): a floor
calibration applied or a refit used moves it. Only punch takes compare poses in it across measurements; a take, a
validation sample and a tip pose each use the tool or arm relative to the base, which no frame move changes.

  calculation          worked out again from what is current: pending, passes or fails, never out of date
  solution             the good current takes (`current_solved`)
  validation verdict   the current samples under the current solution; pending without one (`validation_verdict`)
  tip fit              the current tip poses (`tip_pivot`, `tip_checks`)
  punch verdict        the current punch takes under the current solution (else the relay's calibration) and the
                       TCP offset now (`punch_tips`); as taken: their stored tips

  effect               out of date while it differs from what the current data gives
  calibration file     written by Solve with the solution (`Solved.output`)
  applied              the relay applies the output's hash (`finished`; a mount check: once its file is written)
  TCP offset           the tip fit once committed (`commit_tip_offset`), or entered (`tip_offset_used`)

  acceptance           a failing result accepted by Done pressed twice, stamped on the inputs it was calculated from
                       and what it is judged under (`inputs`, `accept`); holds while they match (`accepted`). A
                       result missing or out of date is never accepted.

Preconditions are checked when a step acts (`CalibrationStep.needs`); they hold or have expired, and never put a
result out of date:

  Setup's checklist    each box ticked within PRECONDITION_HOLDS, Tool on flange for this tool model
                       (`checklist_expired`); needed by Marker error, Record, Validate, Tip offset and Punch
  Area clear, E-stop   their boxes ticked within PRECONDITION_HOLDS (`safety_expired`); else every robot move is refused
  marker errors        measured within PRECONDITION_HOLDS, no data change to either body definition since, driven
                       DRIVE_SECONDS and not red unless accepted (`marker_expired`); needed by Record and Validate
  collisions           every take checked, the base still where it stood then (`collisions_expired`); needed by Record
  validation verdict   passes, or fails and is accepted (`validation_good`); needed by Apply

- Out-of-date results stay in the run, are shown as such and are left out of what uses them. `tidy` moves their files
  to archive/ and current ones back (a definition change and Solve call it), so an offline solve of the session reads
  what Solve used.
- The run file (`progress.py`) is written from here only (`save`), started by the first thing recorded. Plain files;
  main thread.
"""

from __future__ import annotations

import hashlib
import json
import math
import shutil
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import TYPE_CHECKING, Sequence

import numpy as np

from scaffolding_assembly_core.geometry import Pose, compose
from scaffolding_assembly_core.kinematics import link_pose

from ...drive import drive_folder
from ...robot_interface.mocap_definitions import BodyDefinition, body_dict, body_from_dict, compare
from ...world.calibration_sessions import (Calibration, default_calibration, read_session_file, sanity_problem,
                                           sessions_folder)
from ...world.checks import BAD, GOOD, PENDING, Check
from ...world.mocap_frame import VOLUME_MOVED, volume_moved
from .analysis.checks import (CHECKED_ANGLE, CHECKED_POSITION, CHECKLIST_BOXES, DRIVE_SECONDS, MARKER_ERROR_LIMIT,
                              SAFETY_BOXES, TOOL_BOX, expiry, pose_change, spot_groups, take_checks, tcp_offset_check,
                              tip_checks, validation_checks)
from .analysis.fit import Circle, Solution, Take, fit_circle, mount_correction, offset_spread, solve, tool_offsets
from .analysis.pivot import PIVOT_MIN_POSES, Pivot, pivot_fit
from .storage.progress import Progress, new_run
from .analysis.report import write_analysis
from .robot.sampling import Sample, mean_pose
from .storage.session_files import (ARCHIVE, BATCHES, PUNCH_BATCH, VALIDATION_BATCH, archive, calibration_file,
                                    mount_check_file, pose_from, pose_list, read_marker_errors, read_punch,
                                    read_samples, read_take, restore, write_mount_check, write_punch, write_solution,
                                    write_take)
from .robot.sweeps import (DEFAULT_TOOL_BODY, ArmSweeps, joint_limits, load_sweeps, load_validation_poses, named,
                           sweep_problems)

if TYPE_CHECKING:
    from ...robot_interface.mocap_relay import AppliedCalibration, MocapRelayInterface
    from ...robot_interface.robot import HuskyRobotInterface


@dataclass(frozen=True)
class Made:
    """What a measurement was made with besides the arm: the tool body, and both bodies' definitions in Motive.

    Attributes:
        tool_body: The Motive rigid body on the flange.
        base: The base body's definition then, or None if not known (a simulation, an older run).
        tool: The tool body's definition then, or None if not known.
    """

    tool_body: int
    base: BodyDefinition | None = None
    tool: BodyDefinition | None = None

    def as_dict(self) -> dict:
        """For the run file."""
        return {"tool_body": self.tool_body, "base": body_dict(self.base) if self.base else None,
                "tool": body_dict(self.tool) if self.tool else None}

    @classmethod
    def from_dict(cls, entry: dict) -> Made:
        """From `as_dict`."""
        return cls(int(entry["tool_body"]), body_from_dict(entry["base"]) if entry.get("base") else None,
                   body_from_dict(entry["tool"]) if entry.get("tool") else None)


@dataclass(frozen=True)
class RecordedTake:
    """A take of this run, as read back from its file.

    Attributes:
        file: Its take file.
        take: Its points in the base rigid body.
        circle: Its circle.
        calibration: The calibration the relay applied while it was recorded.
        made: What it was measured with.
        marker_errors: The marker errors stored with each of its points ({"base": m, "tool": m}), or None.
    """

    file: Path
    take: Take
    circle: Circle
    calibration: Pose
    made: Made
    marker_errors: tuple[dict | None, ...] = ()

    @classmethod
    def read(cls, file: Path, joint: int, made: Made) -> RecordedTake:
        """Read a take file and fit its circle."""
        take, calibration = read_take(file, joint)
        return cls(file, take, fit_circle(take), calibration, made, tuple(read_marker_errors(file)))

    @property
    def good(self) -> bool:
        """bool: Whether its checks all pass: Solve uses it."""
        return all(check.level == GOOD for check in take_checks("", self.circle))


@dataclass(frozen=True)
class Solved:
    """A solution, the takes it came from, and what Solve wrote from it.

    Attributes:
        solution: The solved calibration.
        takes: The takes it was solved from, J0 first.
        files: Their files: what the solution was made from.
        applied: The calibration the relay applied while measuring (the first take's): the base frame a mount check
            measures the arm in.
        sanity: Why the relay would refuse it, or None; None for a mount check.
        mount: For a mount check, the measured arm base in base_footprint and its correction from the URDF's.
        output: What Solve wrote (the relay's calibration file, or the mount check), or None before writing.
        output_sha: Its hash as the relay gives it (a mount check: the file's).
    """

    solution: Solution
    takes: tuple[Take, ...]
    files: tuple[Path, ...]
    applied: Pose
    sanity: str | None
    mount: tuple[Pose, Pose] | None
    output: Path | None = None
    output_sha: str | None = None


@dataclass(frozen=True)
class Validation:
    """The current validation samples, a file each, and the tool body's spread in tool0 under the solution.

    Attributes:
        bases: Each sample's base rigid body in the world.
        tools: Each sample's tool body in the world.
        spread: Each sample's distance from the mean position (metres) and axis angles (degrees); None without a
            current solution or samples.
        spots: Each base spot's mean base rigid body in the world, by where the base stood (`spot_groups`).
        spot_samples: Each spot's samples, as indices into `bases`.
        marker_errors: The marker errors stored with each sample ({"base": m, "tool": m}), or None.
    """

    bases: list[Pose]
    tools: list[Pose]
    spread: tuple[np.ndarray, np.ndarray] | None
    spots: list[Pose]
    spot_samples: list[list[int]]
    marker_errors: list[dict | None]

    @property
    def spot_sizes(self) -> list[int]:
        """list[int]: Each spot's number of samples."""
        return [len(samples) for samples in self.spot_samples]


class Run:
    """The chosen arm's run: its sweeps, its measurements and results, and its run file once something is recorded.

    Attributes:
        robot: The robot, live.
        arm: The arm whose first two joints sweep.
        sweeps: The takes' sweeps.
        validation_poses: The arm's validation poses.
        problems: Why the sweeps cannot calibrate this arm; empty if they can.
        limits: The arm's joint limits, lower and upper, UR driver order.
        stow: The arm's stow joints.
        mount_check: Whether the arm only checks its mount: only the robot's first arm calibrates the base.
        progress: The run file, None until the first thing is recorded (`save`).
        session: The session folder, None until first needed.
        tool_body: The Motive rigid body on the flange.
        definitions: Motive's body definitions now, by streaming id (`follow`); empty when not known.
        refining: The body ("base" or "tool") being refined in Motive now, or None; refining_from: its definition when
            the refine began.
        marker_max: (base, tool) largest marker error while driving, metres, or None.
        marker_time: When the marker errors were last measured (Unix time), or None.
        marker_made: What they were measured with, or None.
        ticks: When each box of Setup's checklist was ticked (Unix time), by label; unticked boxes are missing.
        tool_ticked_for: The tool model Tool on flange was ticked for ("none" for a bare arm), or None.
        drive_seconds: How long the base was driven in Marker error.
        collisions: Per sweep index: "" clear, else why not; missing until checked.
        collision_base: The base pose when collisions were checked, or None.
        takes: Per sweep index, its recorded take.
        solved: The last solution, current or not.
        validation_files: The validation files, one per base spot, and what each was measured with.
        punch_takes: The punch takes (`session_files.punch_take`), in `punch_file`; what each was taken with: the
            base body's definition, the punch mount and target counts (`punch_current`).
        punch_offset: The TCP offset of the punch, tool0 to tip, metres: committed from Tip offset, or entered.
        punch_mount: How often the punch was remounted (`remounted`); punch_target: how often the target moved.
        tip_joints: The arm's joints at each pose of the tip offset.
        tip_mounts: The punch mount count at each of them.
        acceptances: Per step name, the stamp of the inputs its accepted result was calculated from (`accept`).
        on_stand: Whether the base stands on its stand rather than its wheels; a record for people only.
        notes: The operator's notes on the run, for people only.
        applied_sha: The output hash the relay was seen applying, or None.
        relay: The mocap relay (`ctx.world.mocap_relay`), live, or None without one: what it applies and reads.
    """

    def __init__(self, robot: HuskyRobotInterface, arm: str, sweeps: ArmSweeps, validation_poses: list[np.ndarray],
                 drive_root: Path | None, tool_body: int = DEFAULT_TOOL_BODY, relay: MocapRelayInterface | None = None):
        """A new run that measured nothing yet."""
        urdf = robot.config.model.urdf
        self.robot, self.arm, self.sweeps, self.validation_poses = robot, arm, sweeps, validation_poses
        self.relay = relay
        self._drive_root = drive_root
        self.problems = sweep_problems(sweeps, urdf, arm)
        self.limits = joint_limits(urdf, arm)
        self.stow = np.array(next(config.stow_joints for config in robot.config.arms if config.name == arm))
        self.mount_check = arm != robot.config.arms[0].name
        self.progress: Progress | None = None
        self.session: Path | None = None
        self.tool_body, self.refining = tool_body, None
        self.definitions: dict[int, BodyDefinition] = {}
        self.refining_from: BodyDefinition | None = None
        self.marker_max: list[float] | None = None
        self.marker_time: float | None = None
        self.marker_made: Made | None = None
        self.ticks: dict[str, float] = {}
        self.tool_ticked_for: str | None = None
        self.drive_seconds = 0.0
        self.collisions: dict[int, str] = {}
        self.collision_base: Pose | None = None
        self.takes: dict[int, RecordedTake] = {}
        self.solved: Solved | None = None
        self.validation_files: dict[Path, Made] = {}
        self.punch_takes: list[dict] = []
        self.punch_file: Path | None = None
        self.punch_bases: list[BodyDefinition | None] = []
        self.punch_mounts: list[int] = []
        self.punch_targets: list[int] = []
        self.punch_offset = np.zeros(3)
        self.punch_mount = self.punch_target = 0
        self.tip_joints: list[np.ndarray] = []
        self.tip_mounts: list[int] = []
        self.acceptances: dict[str, str] = {}
        self.on_stand, self.notes = False, ""
        self._tip_pivot: tuple[tuple, Pivot | None] | None = None
        self._punch_judged: tuple[tuple, tuple[np.ndarray, list[Pose]]] | None = None
        #: The sessions folder, or why there is none, for the relay simulated or not (`sessions`, `refresh_sessions`).
        self._sessions: tuple[bool, Path | FileNotFoundError] | None = None
        self.applied_sha: str | None = None
        self._validation: tuple[tuple, Validation] | None = None
        self._validation_solved: Solved | None = None

    @classmethod
    def new(cls, robot: HuskyRobotInterface, arm: str, data_directory: Path, drive_root: Path | None,
            relay: MocapRelayInterface | None = None) -> Run:
        """A new run of a robot's arm with its predefined sweeps and validation poses.

        Raises:
            KeyError: If the sweeps file has no sweeps for the arm.
        """
        digits = robot.config.serial[-4:]
        return cls(robot, arm, load_sweeps(data_directory, digits, arm),
                   load_validation_poses(data_directory, digits, arm), drive_root, relay=relay)

    @classmethod
    def resume(cls, robot: HuskyRobotInterface, progress: Progress, data_directory: Path,
               drive_root: Path | None, relay: MocapRelayInterface | None = None) -> Run:
        """Rebuild a run from its file and the files it lists; the solution only if its written output matches.

        Raises:
            OSError: If a file it lists is missing.
            ValueError: If a file it lists does not read.
        """
        run = cls(robot, progress.arm, progress.arm_sweeps,
                  load_validation_poses(data_directory, robot.config.serial[-4:], progress.arm), drive_root,
                  progress.tool_body, relay)
        run.progress, run.session = progress, progress.session
        run.marker_max, run.drive_seconds, run.marker_time = progress.marker_max, progress.drive_seconds, \
            progress.marker_time
        run.marker_made = Made.from_dict(progress.marker_made) if progress.marker_made else None
        run.ticks, run.tool_ticked_for = dict(progress.ticks), progress.tool_ticked_for
        run.collisions = dict(progress.collisions)
        run.collision_base = pose_from(progress.collision_base) if progress.collision_base else None

        def made(name: str) -> Made:
            return Made.from_dict(progress.made[name]) if name in progress.made else Made(progress.tool_body)

        run.takes = {index: RecordedTake.read(run.session / name, run.sweeps.sweeps[index].joint, made(name))
                     for index, name in progress.takes.items()}
        if progress.output_sha is not None:
            solved = run.compute_solution()
            output = next((path for path in (run.output_file(False), run.output_file(True))
                           if path.is_file() and run.output_sha(path) == progress.output_sha), None)
            if output is not None:
                run.solved = replace(solved, output=output, output_sha=progress.output_sha)
                run.applied_sha = progress.output_sha if progress.applied else None
        run.validation_files = {run.session / name: made(name) for name in progress.validations}
        run.punch_mount, run.punch_target = progress.punch_mount, progress.punch_target
        if progress.punch_file:
            run.punch_file = run.session / progress.punch_file
            run.punch_takes = read_punch(run.punch_file)
            # * Older run files lack these: their takes count as taken with an unknown definition, before any remount
            # * or target move.
            count = len(run.punch_takes)
            run.punch_bases = [body_from_dict(base) if base else None for base in progress.punch_bases] or \
                [None] * count
            run.punch_mounts = progress.punch_mounts or [0] * count
            run.punch_targets = progress.punch_targets or [0] * count
        run.punch_offset = np.array(progress.punch_offset, dtype=float)
        run.tip_joints = [np.array(joints, dtype=float) for joints in progress.tip_joints]
        run.tip_mounts = progress.tip_mounts or [0] * len(run.tip_joints)
        run.acceptances = dict(progress.acceptances)
        run.on_stand, run.notes = progress.on_stand, progress.notes
        return run

    # --- --- --- --- --- FILES --- --- --- --- ---

    @property
    def serial(self) -> str:
        """str: The robot's serial, e.g. "a200-0805"."""
        return self.robot.config.serial

    @property
    def digits(self) -> str:
        """str: The serial's last four digits, as file names use them."""
        return self.serial[-4:]

    @property
    def urdf(self) -> Path:
        """Path: The robot's URDF."""
        return self.robot.config.model.urdf

    @property
    def tool_model(self) -> str:
        """str: The tool the monitor models on the arm, as launched: an end effector kind, or "none" for a bare arm."""
        return next(arm.end_effector for arm in self.robot.config.arms if arm.name == self.arm) or "none"

    def simulated(self) -> bool:
        """Whether the run measures a simulation: the relay reads one (`sim:=true`), so all it measures is simulated."""
        return self.relay is not None and self.relay.state.simulated

    def mixed(self) -> str | None:
        """Why the run must not write: the robot and the relay mix real and simulated; None if they agree.

        A simulated base or arm with the relay reading Motive would write where that relay applies it.
        """
        simulated = self.simulated()
        parts = {"base": self.robot.base.simulation, "arm": self.robot.arms[self.arm].simulation}
        odd = [name for name, simulation in parts.items() if (simulation is not None) != simulated]
        if not odd:
            return None
        odd_are = f"{' and '.join(odd)} {'is' if len(odd) == 1 else 'are'}"
        if simulated:
            return f"the relay reads a simulation, but the robot's {odd_are} real: run it with a simulated robot"
        known = self.relay is not None and self.relay.state.calibrations_time is not None
        motive = "the relay reads Motive" + ("" if known else " (or has not said yet)")
        return (f"the robot's {odd_are} simulated, but {motive}: the run would go to the robots' sessions, which the "
                f"relay applies; start the relay with sim:=true")

    def sessions(self) -> Path:
        """The folder of dated sessions on the Drive: the robots', or this domain's simulated ones (`simulated`).

        Looked up again once the relay's source changes, or after `refresh_sessions`.

        Raises:
            FileNotFoundError: If the Drive's folder is missing, or the robot and the relay mix real and simulated
                (`mixed`).
        """
        if (why := self.mixed()) is not None:
            raise FileNotFoundError(f"no sessions folder: {why}")
        simulated = self.simulated()
        if self._sessions is None or self._sessions[0] != simulated:
            try:
                folder = sessions_folder(simulated)
                if simulated:
                    # * Only a relay with sim:=true reads them, for a fake Motive's bodies.
                    (drive_folder(self._drive_root, Path(".")) / folder).mkdir(parents=True, exist_ok=True)
                self._sessions = (simulated, drive_folder(self._drive_root, folder))
            except FileNotFoundError as error:
                self._sessions = (simulated, error)
        if isinstance(self._sessions[1], FileNotFoundError):
            raise self._sessions[1]
        return self._sessions[1]

    def refresh_sessions(self) -> None:
        """Look up the sessions folder again on next use."""
        self._sessions = None

    @property
    def drive_root(self) -> Path | None:
        """Path | None: The Drive folder this run writes its sessions in; None without one."""
        return self._drive_root

    def session_folder(self) -> Path:
        """Today's session folder, made on first use, with its batch folders.

        Raises:
            FileNotFoundError: If the Drive's folder is missing.
        """
        if self.session is None:
            self.session = self.sessions() / time.strftime("%Y%m%d")
        for batch in (*BATCHES, VALIDATION_BATCH):
            (self.session / batch).mkdir(parents=True, exist_ok=True)
        return self.session

    def start_file(self) -> None:
        """Start the run file in today's session folder.

        Raises:
            OSError: If the session folder cannot be made.
        """
        self.progress = new_run(self.session_folder(), self.serial, self.arm, self.sweeps, time.strftime("%H%M%S"),
                                self.tool_body, self.simulated())

    def save(self, start: bool = False) -> None:
        """Write the run file, if the run has one; `start` it first if not: for the first thing recorded.

        Raises:
            OSError: If the run file cannot be started.
        """
        if self.progress is None and start:
            self.start_file()
        progress = self.progress
        if progress is None:
            return

        def relative(path: Path) -> str:
            return str(path.relative_to(progress.session))

        solved = self.current_solved()
        progress.tool_body = self.tool_body
        progress.drive_seconds, progress.marker_max, progress.marker_time = self.drive_seconds, self.marker_max, \
            self.marker_time
        progress.marker_made = self.marker_made.as_dict() if self.marker_made else None
        progress.ticks, progress.tool_ticked_for = dict(self.ticks), self.tool_ticked_for
        progress.collisions = dict(self.collisions)
        progress.collision_base = pose_list(self.collision_base) if self.collision_base else None
        progress.takes = {index: relative(take.file) for index, take in self.takes.items()}
        progress.validations = [relative(path) for path in self.validation_files]
        made = [(take.file, take.made) for take in self.takes.values()] + list(self.validation_files.items())
        progress.made = {relative(path): m.as_dict() for path, m in made}
        # * Only a current solution's output: a resume solves again from the takes and must find that file.
        progress.output_sha = solved.output_sha if solved is not None else None
        progress.punch_file = relative(self.punch_file) if self.punch_file else None
        progress.punch_offset = [float(v) for v in self.punch_offset]
        progress.punch_mount, progress.punch_target = self.punch_mount, self.punch_target
        progress.punch_bases = [body_dict(base) if base else None for base in self.punch_bases]
        progress.punch_mounts, progress.punch_targets = list(self.punch_mounts), list(self.punch_targets)
        progress.tip_joints = [[float(v) for v in joints] for joints in self.tip_joints]
        progress.tip_mounts = list(self.tip_mounts)
        progress.acceptances = dict(self.acceptances)
        progress.on_stand, progress.notes = self.on_stand, self.notes
        progress.applied = self.finished
        progress.save()

    def tidy(self) -> list[Path]:
        """Move this run's out-of-date take and validation files to archive/, current ones back; return those archived.

        The run keeps every measurement, so putting its input back makes it current again; only the files move, so the
        session's folders hold what Solve uses (a take failing its checks stays archived).
        """
        moved = []
        for index, take in self.takes.items():
            current, archived = self.take_current(take), take.file.parent.name == ARCHIVE
            if not current and not archived:
                self.takes[index] = replace(take, file=archive(take.file))
                moved.append(self.takes[index].file)
            elif current and archived and take.good:
                self.takes[index] = replace(take, file=restore(take.file))
        files = {}
        for path, made in self.validation_files.items():
            current, archived = self.validation_current(made), path.parent.name == ARCHIVE
            if not current and not archived:
                path = archive(path)
                moved.append(path)
            elif current and archived:
                path = restore(path)
            files[path] = made
        self.validation_files = files
        return moved

    # --- --- --- --- --- MEASUREMENTS --- --- --- --- ---

    @property
    def made(self) -> Made:
        """Made: What a measurement taken now is made with."""
        return Made(self.tool_body, self.definitions.get(self.robot.config.mocap_id),
                    self.definitions.get(self.tool_body))

    def changes(self, made: Made, bodies: Sequence[str] = ("base", "tool")) -> list[str]:
        """How each of `bodies` ("base", "tool") changed since `made` by a data change, e.g. "base body refined".

        A definition not known, then or now, counts as unchanged.
        """
        now = self.made
        found = []
        for body in bodies:
            then, current = getattr(made, body), getattr(now, body)
            if then is not None and current is not None and (change := compare(then, current)).data_change:
                found.append(f"{body} body {change.kind}")
        return found

    def take_current(self, take: RecordedTake) -> bool:
        """Whether a take holds: the same tool body, no data change to the base body's definition since.

        ? A tool-body refine moves the tool point rigidly on the flange, so it circles the same axes; to confirm with
          calib_bench.
        """
        return take.made.tool_body == self.tool_body and not self.changes(take.made, ("base",))

    def follow(self, definitions: dict[int, BodyDefinition], tidy: bool = True) -> list[Path]:
        """Take Motive's body definitions now: archive what went out of date (`tidy`, unless not) and return where.

        A refine in progress ends once its body's definition changed; the marker errors are then measured again.
        """
        self.definitions = definitions
        live = self.made.base if self.refining == "base" else self.made.tool
        if self.refining and self.refining_from is not None and live is not None and \
                compare(self.refining_from, live).data_change:
            self.refining = self.refining_from = None
            self.reset_markers()
        return self.tidy() if tidy else []

    def lab_frame(self) -> Pose | None:
        """The lab frame the relay applies now, its lab_from_raw; None before it announced one.

        Without a floor calibration it is the identity: poses are Motive's own.
        """
        frame = self.relay.state.frame if self.relay is not None else None
        if frame is None:
            return None
        return Pose(*frame.lab_from_raw) if frame.lab_from_raw is not None else Pose()

    def frame_moved(self, then: Pose | None) -> str | None:
        """How the lab frame moved since `then`, if by over VOLUME_MOVED at a point of the lab; else None.

        A frame not known, then or now, counts as unmoved.
        """
        now = self.lab_frame()
        if then is None or now is None or (moved := volume_moved(then, now)) <= VOLUME_MOVED:
            return None
        return f"lab frame moved {moved * 1e3:.1f} mm"

    def start_refine(self, body: str) -> None:
        """The operator refines the "base" or "tool" body in Motive: it ends once its definition changes (`follow`)."""
        self.refining = body
        self.refining_from = self.made.base if body == "base" else self.made.tool

    def reset_markers(self) -> None:
        """Measure the marker errors again from zero."""
        self.marker_max, self.drive_seconds, self.marker_time, self.marker_made = None, 0.0, None, None

    # --- --- --- --- --- PRECONDITIONS --- --- --- --- ---

    def marker_age(self, now: float) -> str | None:
        """Why the marker errors measured have expired by age (not measured, or too long ago); None if they hold."""
        if self.marker_time is None:
            return "not measured" if self.marker_max is None else "measured, but when is not known"
        return expiry("measured", self.marker_time, now)

    def marker_expired(self, now: float) -> str | None:
        """Why the marker errors have expired: too old (`marker_age`); unless accepted, driven too short or red."""
        why = self.marker_age(now)
        if why is None and self.marker_made is not None and (changed := self.changes(self.marker_made)):
            why = f"{' and '.join(changed)} since measured"
        if why is None and not self.accepted("Marker error"):
            red = [name for name, error in zip(("base", "tool"), self.marker_max or ()) if error > MARKER_ERROR_LIMIT]
            if self.drive_seconds < DRIVE_SECONDS:
                # * No seconds: the overview shows this in every step; Marker error's readout counts them.
                why = f"driven under {DRIVE_SECONDS:g} s"
            elif red:
                why = f"{' and '.join(red)} marker error over {MARKER_ERROR_LIMIT * 1e3:g} mm"
        return why

    def box_expired(self, label: str, now: float) -> str | None:
        """Why a box of Setup's checklist does not hold: unticked, too old, or for another tool; None if it does."""
        if label not in self.ticks:
            return f"{label} not ticked"
        if label == TOOL_BOX and self.tool_ticked_for != self.tool_model:
            return f"{label} ticked for {self.tool_ticked_for}"
        return expiry(f"{label} ticked", self.ticks[label], now)

    def safety_expired(self, now: float) -> str | None:
        """Why Area clear or E-stop at hand do not hold, which every robot move needs; None if they do."""
        return "; ".join(why for label in SAFETY_BOXES if (why := self.box_expired(label, now))) or None

    def checklist_expired(self, now: float, boxes: Sequence[str] = CHECKLIST_BOXES) -> str | None:
        """Why Setup's checklist, or these of its `boxes`, does not hold: each box that does not and why; else None."""
        return "; ".join(why for label in boxes if (why := self.box_expired(label, now))) or None

    def tick(self, label: str, ticked: bool, now: float) -> None:
        """The operator ticked (or unticked) a box of Setup's checklist; Tool on flange for the tool model now."""
        if not ticked:
            self.ticks.pop(label, None)
            return
        self.ticks[label] = now
        if label == TOOL_BOX:
            self.tool_ticked_for = self.tool_model

    def adopt_checklist(self, other: Run) -> None:
        """Take another run's ticks where they are newer: they are about the arm, not the run."""
        for label, at in other.ticks.items():
            if at > self.ticks.get(label, -math.inf):
                self.ticks[label] = at
                if label == TOOL_BOX:
                    self.tool_ticked_for = other.tool_ticked_for

    def collisions_expired(self, base: Pose) -> str | None:
        """Why the collision checks do not hold with the base at `base`: not all checked, or moved; None if they do."""
        if self.collision_base is None or len(self.collisions) < len(self.sweeps.sweeps):
            return "collisions not checked"
        metres, degrees = pose_change(self.collision_base, base)
        if metres > CHECKED_POSITION or math.radians(degrees) > CHECKED_ANGLE:
            # * No distance: the overview shows this in every step; the Live tab plots the base drift, and a take
            # * refused under the lease logs it (`collision_base_moved`).
            return (f"the base moved over {CHECKED_POSITION * 1e2:g} cm or {math.degrees(CHECKED_ANGLE):g}° since the "
                    f"collision check: check again")
        return None

    def collision_base_moved(self, base: Pose) -> str | None:
        """How far the base at `base` is from where the collisions were checked, e.g. "32 mm, 2.1°"; None unchecked."""
        if self.collision_base is None:
            return None
        metres, degrees = pose_change(self.collision_base, base)
        return f"{metres * 1e3:.0f} mm, {degrees:.1f}°"

    def usable(self, joint: int | None = None) -> list[int]:
        """Indices of the sweeps checked clear, of one joint or all."""
        return [index for index, sweep in enumerate(self.sweeps.sweeps)
                if self.collisions.get(index) == "" and (joint is None or sweep.joint == joint)]

    def built_for(self) -> BodyDefinition | None:
        """The base body definition the current takes were recorded with (the newest known), for Solve's file."""
        found = [take.made.base for _, take in sorted(self.current_takes().items()) if take.made.base is not None]
        return found[-1] if found else None

    def current_takes(self) -> dict[int, RecordedTake]:
        """The takes that hold (`take_current`)."""
        return {index: take for index, take in self.takes.items() if self.take_current(take)}

    def pending(self) -> list[int]:
        """The clear takes still to record: not recorded, or out of date."""
        current = self.current_takes()
        return [index for index in self.usable() if index not in current]

    def good_takes(self) -> list[int]:
        """The current takes whose checks all pass: the ones Solve uses."""
        return [index for index, take in sorted(self.current_takes().items()) if take.good]

    def keep_take(self, index: int, file: Path) -> RecordedTake:
        """Keep a take just written, read back from its file: what is used is what is saved.

        The take it replaces goes to archive/, and so does a new one that fails its checks: left out of Solve, so no
        offline solve of the session reads it either.
        """
        if index in self.takes:
            archive(self.takes[index].file)
        recorded = RecordedTake.read(file, self.sweeps.sweeps[index].joint, self.made)
        if not recorded.good:
            recorded = replace(recorded, file=archive(file))
        self.takes[index] = recorded
        return recorded

    # --- --- --- --- --- SOLUTION --- --- --- --- ---

    def solve_inputs(self) -> tuple[Path, ...]:
        """The files Solve would use now: the good current takes, J0 first."""
        ordered = sorted(self.good_takes(), key=lambda index: self.takes[index].take.joint)
        return tuple(self.takes[index].file for index in ordered)

    def current_solved(self) -> Solved | None:
        """The solution, if solved from the takes Solve would use now; else None.

        Compared by file name: a take's file may move to archive/ and back (`tidy`).
        """
        solved = self.solved
        if solved is None or [path.name for path in solved.files] != [path.name for path in self.solve_inputs()]:
            return None
        return solved

    def compute_solution(self) -> Solved:
        """Solve from the good current takes, and judge it as the relay would; writes nothing.

        Raises:
            ValueError: If no take passes its checks, or the takes cannot be solved.
        """
        good = self.good_takes()
        if not good:
            raise ValueError("no take passes its checks: record takes again")
        ordered = sorted(good, key=lambda index: self.takes[index].take.joint)
        takes = tuple(self.takes[index].take for index in ordered)
        solution = solve([t for t in takes if t.joint == 0], [t for t in takes if t.joint == 1], self.urdf, self.arm)
        applied = self.takes[good[0]].calibration
        files = tuple(self.takes[index].file for index in ordered)
        if self.mount_check:
            return Solved(solution, takes, files, applied, None,
                          mount_correction(applied, solution, self.urdf, self.arm))
        mocap_id = self.robot.config.mocap_id
        default = default_calibration(mocap_id)
        candidate = Calibration(mocap_id, self.serial, solution.footprint.position, solution.footprint.orientation, "",
                                "", ())
        sanity = sanity_problem(candidate, default) if default is not None else None
        return Solved(solution, takes, files, applied, sanity, None)

    def solve_and_write(self) -> Solved:
        """Tidy, solve, and write the result where the relay finds it: in progress, until Apply approves it.

        A result Solve wrote before goes to archive/; an approved one stays until Apply.

        Raises:
            ValueError: If the takes cannot be solved.
            OSError: If the session cannot be written.
        """
        self.tidy()
        solved = self.compute_solution()
        session = self.session_folder()
        outputs = [self.output_file()] if self.mount_check else [session / "base_frame_calibration.json"]
        for existing in (path for path in outputs if path.is_file()):
            # * The relay ignores archive/: keep the session's earlier result there.
            (session / ARCHIVE).mkdir(exist_ok=True)
            shutil.move(existing, session / ARCHIVE / f"{existing.stem}_{time.strftime('%H%M%S')}.json")
        if solved.mount is not None:
            output = write_mount_check(session, self.digits, self.arm, *solved.mount, solved.applied)
        else:
            output = write_solution(session, self.digits, self.arm, self.robot.config.mocap_id, solved.solution,
                                    approved=False, body_definition=self.built_for(), simulated=self.simulated())
        self.solved = replace(solved, output=output, output_sha=self.output_sha(output))
        return self.solved

    def output_file(self, approved: bool = False) -> Path:
        """What Solve writes: the relay's calibration file, in progress until Apply approves it; or the mount check."""
        if self.mount_check:
            return mount_check_file(self.session, self.digits, self.arm)
        return calibration_file(self.session, self.digits, approved)

    def output_sha(self, output: Path) -> str:
        """The hash the relay gives a calibration file; a mount check's file hash."""
        if self.mount_check:
            return hashlib.sha256(output.read_bytes()).hexdigest()
        return read_session_file(output, output.parent, self.robot.config.mocap_id, self.serial).sha256

    def applied(self) -> AppliedCalibration | None:
        """The relay's entry for the calibration it applies to the robot now (pose, hash, source); None if none.

        A simulation's relay (sim:=true) applies those of the simulated sessions.
        """
        if self.relay is None:
            return None
        return self.relay.state.calibrations.get(self.robot.config.mocap_id)

    def applied_calibration(self) -> Pose | None:
        """The calibration the relay applies to the robot now, base_footprint in the base rigid body; None if none."""
        applied = self.applied()
        return Pose(applied.position, applied.orientation) if applied is not None else None

    def write_analysis(self) -> None:
        """Write the plugin's own analysis of the current solution to `analysis/<digits>_<arm>/`, if there is one.

        It compares the solution with the calibration the relay applies now.
        """
        solved = self.current_solved()
        if solved is not None:
            write_analysis(self.session_folder(), self.digits, self.arm, solved.solution, solved.files,
                           self.applied_calibration(), self.validation().spread,
                           solved.mount[1] if solved.mount else None)

    @property
    def finished(self) -> bool:
        """bool: Whether the relay applied the current solution's output (a mount check: once it is written)."""
        solved = self.current_solved()
        if solved is None or solved.output_sha is None:
            return False
        return self.mount_check or self.applied_sha == solved.output_sha

    # --- --- --- --- --- VALIDATION --- --- --- --- ---

    def validation_current(self, made: Made) -> bool:
        """Whether validation samples hold: the same tool body, no data change to either body; any solution."""
        return made.tool_body == self.tool_body and not self.changes(made)

    def current_validation_files(self) -> list[Path]:
        """The validation files that hold (`validation_current`)."""
        return [path for path, made in self.validation_files.items() if self.validation_current(made)]

    def add_validation_sample(self, sample: Sample, pose: int) -> Path:
        """Write one validation sample, at validation pose number `pose`, and keep the file: nothing is lost later."""
        name = f"{time.strftime('%H%M%S')}_{self.digits}_{self.arm}_validation_pose{pose}"
        path = write_take(self.session_folder() / VALIDATION_BATCH, name, [sample.record()])
        self.validation_files[path] = self.made
        return path

    def clear_validation(self) -> None:
        """Start the validation again: every validation file goes to archive/."""
        for path in self.validation_files:
            archive(path)
        self.validation_files = {}

    def validation(self) -> Validation:
        """The current validation files' samples, grouped by base spot, judged under the solution.

        Kept until the files or the solution change: the step's checks ask every draw.
        """
        files, solved = tuple(self.current_validation_files()), self.current_solved()
        key = (files, id(solved))
        if self._validation is not None and self._validation[0] == key:
            return self._validation[1]
        bases, tools, joints, errors = [], [], [], []
        for file in files:
            more = read_samples(file)
            bases, tools, joints = bases + more[0], tools + more[1], joints + list(more[2])
            errors += read_marker_errors(file)
        groups = spot_groups(bases)
        spots = [mean_pose([bases[index] for index in group]) for group in groups]
        spread = None
        if solved is not None and bases:
            spread = offset_spread(tool_offsets(bases, tools, np.array(joints).reshape(-1, 6),
                                                solved.solution.footprint, self.urdf, self.arm))
        validation = Validation(bases, tools, spread, spots, groups, errors)
        # * The key holds the solution's id: keep the solution with it, so the id is not reused meanwhile.
        self._validation, self._validation_solved = (key, validation), solved
        return validation

    def validation_verdict(self) -> list[Check]:
        """Validate's verdict: enough samples and spots, their spread under the current solution (pending without)."""
        validation = self.validation()
        checks = validation_checks(*(validation.spread or (None, None)), validation.bases)
        if validation.bases and self.current_solved() is None:
            checks[2:] = [Check(label, PENDING, "judged under the current solution: Solve first")
                          for label in ("position", "orientation")]
        return checks

    def validation_passes(self) -> bool:
        """Whether the validation verdict passes: no check red or pending."""
        return all(check.level not in (BAD, PENDING) for check in self.validation_verdict())

    def validation_good(self) -> bool:
        """Whether the validation verdict passes, or fails and is accepted: Apply may act.

        A verdict without current samples or a current solution cannot be accepted.
        """
        if self.validation_passes():
            return True
        return bool(self.current_validation_files()) and self.current_solved() is not None and \
            self.accepted("Validate")

    def footprint(self) -> Pose:
        """base_footprint in the base rigid body to judge under: the current solution's, else the relay's, else none."""
        solved = self.current_solved()
        if solved is not None:
            return solved.solution.footprint
        return self.applied_calibration() or Pose()

    # --- --- --- --- --- PUNCH --- --- --- --- ---

    def punch_current(self, index: int) -> bool:
        """Whether a punch take is current: no base body data change, the same punch mount and target, the lab frame."""
        frame = self.punch_takes[index].get("lab_from_raw")
        return (not self.changes(Made(self.tool_body, self.punch_bases[index]), ("base",))
                and self.punch_mounts[index] == self.punch_mount and self.punch_targets[index] == self.punch_target
                and self.frame_moved(pose_from(frame) if frame else None) is None)

    def current_punch_takes(self) -> list[dict]:
        """The punch takes that are current (`punch_current`)."""
        return [take for index, take in enumerate(self.punch_takes) if self.punch_current(index)]

    def add_punch_take(self, take: dict, arm_name: str) -> Path:
        """Keep a punch take and write the punch file; return it.

        Once a take is out of date (`punch_current`), the next starts a new file: the old one goes to archive/.
        """
        if not all(self.punch_current(index) for index in range(len(self.punch_takes))):
            self.clear_punch()
        self.punch_takes.append(take)
        self.punch_bases.append(self.made.base)
        self.punch_mounts.append(self.punch_mount)
        self.punch_targets.append(self.punch_target)
        if self.punch_file is None:
            self.punch_file = (self.session_folder() / PUNCH_BATCH
                               / f"punch_validation_{arm_name}_{time.strftime('%Y%m%d_%H%M')}.json")
        write_punch(self.punch_file, self.punch_takes)
        return self.punch_file

    def clear_punch(self) -> None:
        """Forget the punch takes; their file goes to archive/."""
        if self.punch_file is not None and self.punch_file.is_file():
            archive(self.punch_file)
        self.punch_takes, self.punch_bases, self.punch_mounts, self.punch_targets = [], [], [], []
        self.punch_file = None

    def target_moved(self) -> None:
        """The punch target moved: the punch takes so far are out of date."""
        self.punch_target += 1

    def punch_tips(self, footprint: Pose) -> tuple[np.ndarray, list[Pose]]:
        """The current punch takes judged under `footprint` and the TCP offset now: their tips and bases in the world.

        A take from an older file, without the base body's own pose, is judged under the calibration applied then.
        Kept until the takes, `footprint` or the offset change: the step's checks ask every draw.

        Args:
            footprint: base_footprint in the base rigid body (`footprint`).
        """
        takes = self.current_punch_takes()
        key = (self.punch_file, tuple(take["timestamp"] for take in takes), tuple(footprint.position),
               tuple(footprint.orientation), tuple(self.punch_offset))
        if self._punch_judged is None or self._punch_judged[0] != key:
            tips, bases = [], []
            for take in takes:
                if "base_mocap_pose" in take:
                    base = compose(pose_from(take["base_mocap_pose"]), footprint)
                else:
                    base = Pose(tuple(take["base_pose"]["position"]), tuple(take["base_pose"]["quaternion"]))
                tool0 = link_pose(self.urdf, base, named(self.arm, take["joint_conf"]), f"{self.arm}_tool0")
                tips.append(compose(tool0, Pose(tuple(self.punch_offset))).position)
                bases.append(base)
            self._punch_judged = (key, (np.array(tips, dtype=float).reshape(-1, 3), bases))
        return self._punch_judged[1]

    # --- --- --- --- --- TIP OFFSET --- --- --- --- ---

    def current_tip(self) -> list[np.ndarray]:
        """The tip offset's current poses, those taken since the punch was last remounted: their joints."""
        return [joints for joints, mount in zip(self.tip_joints, self.tip_mounts) if mount == self.punch_mount]

    def tip_tool0s(self) -> list[Pose]:
        """Tool0 in the robot's frame at each current pose of the tip offset, from the URDF."""
        link = f"{self.arm}_tool0"
        return [link_pose(self.urdf, Pose(), named(self.arm, joints), link) for joints in self.current_tip()]

    def tip_pivot(self) -> Pivot | None:
        """The tip fit, the offset solved from the current poses; None with too few. Solved once per set of poses."""
        key = (len(self.tip_joints), self.punch_mount)
        if self._tip_pivot is None or self._tip_pivot[0] != key:
            tool0s = self.tip_tool0s()
            self._tip_pivot = (key, pivot_fit(tool0s) if len(tool0s) >= PIVOT_MIN_POSES else None)
        return self._tip_pivot[1]

    def tip_checks(self) -> list[Check]:
        """The tip fit's checks: its poses, their tilt spread and fit, the offset plausible."""
        return tip_checks(len(self.current_tip()), self.tip_pivot())

    def add_tip_pose(self, joints: np.ndarray) -> None:
        """Keep a pose of the tip offset, its joints; poses from before a remount are dropped first."""
        self.tip_joints = [*self.current_tip(), np.asarray(joints, dtype=float)]
        self.tip_mounts = [self.punch_mount] * len(self.tip_joints)

    def commit_tip_offset(self) -> bool:
        """Make the tip fit the punch's TCP offset; whether there was a fit to commit."""
        pivot = self.tip_pivot()
        if pivot is None:
            return False
        self.punch_offset = np.array(pivot.offset, dtype=float)
        return True

    def tip_offset_used(self) -> bool:
        """Whether the punch's TCP offset is the tip fit, to a micrometre."""
        pivot = self.tip_pivot()
        return pivot is not None and np.allclose(pivot.offset, self.punch_offset, rtol=0.0, atol=1e-6)

    def clear_tip(self) -> None:
        """Forget the tip offset's poses and fit; the punch keeps its TCP offset."""
        self.tip_joints, self.tip_mounts, self._tip_pivot = [], [], None

    def remounted(self) -> None:
        """The punch was remounted: the tip poses and the punch takes so far are out of date."""
        self.punch_mount += 1

    def tip_in_tool0(self) -> np.ndarray:
        """The punch tip in tool0, for tool steps to turn about: the punch offset if plausible, else tool0 itself."""
        return self.punch_offset if tcp_offset_check(self.punch_offset).level == GOOD else np.zeros(3)

    # --- --- --- --- --- ACCEPTANCES --- --- --- --- ---

    def inputs(self, step: str, under=None) -> str:
        """A stamp of a step's inputs, by step name, and of `under`: an acceptance holds while it matches."""
        def sha() -> str | None:
            solved = self.current_solved()
            return solved.output_sha if solved is not None else None

        built_from = {
            "Setup": lambda: [self.ticks, self.tool_ticked_for, self.tool_body],
            "Marker error": lambda: [self.marker_max, self.marker_made.as_dict() if self.marker_made else None],
            "Sweeps": lambda: [sorted(self.collisions.items()),
                               pose_list(self.collision_base) if self.collision_base else None],
            "Record": lambda: [take.file.name for take in self.current_takes().values()],
            "Solve": lambda: [path.name for path in self.solve_inputs()],
            "Validate": lambda: [sha(), [path.name for path in self.current_validation_files()]],
            "Apply": lambda: [sha()],
            "Tip offset": lambda: [self.current_tip()],
            "Punch": lambda: [self.punch_file.name if self.punch_file else None,
                              [take["timestamp"] for take in self.current_punch_takes()], self.punch_offset],
        }
        stamped = [built_from[step](), pose_list(under) if isinstance(under, Pose) else under]
        text = json.dumps(stamped, sort_keys=True, default=lambda value: np.asarray(value).tolist())
        return hashlib.sha256(text.encode()).hexdigest()[:16]

    def accept(self, step: str, under=None) -> None:
        """The operator went on past the step's failing result: accept it, as long as its inputs stay the same.

        Args:
            step: The step's name.
            under: What its verdict is judged under besides its inputs (`CalibrationStep.judged_under`), e.g.
                Punch's base_footprint; None for nothing.
        """
        self.acceptances[step] = self.inputs(step, under)

    def accepted(self, step: str, under=None) -> bool:
        """Whether the step's result is accepted: an acceptance stamped on the inputs it has now (`accept`)."""
        return step in self.acceptances and self.acceptances[step] == self.inputs(step, under)
