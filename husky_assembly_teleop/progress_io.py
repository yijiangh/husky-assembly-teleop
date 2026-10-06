"""Persist how far the assembly got: ``<problem>/progress.json`` next to ``ActionSchedule.json``.

Only one robot is connected per monitor run, and the operator restarts the
monitor on the next robot. What must survive that restart lives here:

- each schedule entry's status (pending / done / skipped) and who marked it;
- each robot's last known state (a ``RobotBelief``: base frame + arm joints),
  used to draw and collision-check the robots that are NOT connected;
- each support robot's hold (pending / holding / released).

File layout (schema v1)::

    {"schema_version": 1, "problem": "260920_RobArch_demo_revamp_backup",
     "schedule_fingerprint": "48:<sha1 of the action ids>", "updated_at": "...", "current_index": 4,
     "entries": {"0": {"status": "done", "marked_at": "...", "marked_by_robot": "Cindy", "run_id": "..."}},
     "robots": {"Alice": {"base_frame": <compas Frame>, "configuration": <compas Configuration>,
                          "source": "action_end_state", "at": "...", "after_entry": 3}},
     "holds": {"B3": {"bar_id": "B3", "robot": "Alice", "state": "holding",
                      "since_index": 3, "released_at_index": null}}}

* Where a non-connected robot is drawn (``obstacle_tool_states``): a robot that
* has released its hold is parked out of the way; otherwise its persisted
* belief; otherwise what the loaded action's export says; otherwise parked.
* (Live mocap for the base is layered on top by the monitor.)
"""

from __future__ import annotations

import os
from dataclasses import asdict, dataclass, field, replace
from datetime import datetime
from hashlib import sha1
from shutil import move
from typing import Optional

from compas.data import json_dumps, json_load
from compas.geometry import Frame
from compas_robots import Configuration
from rs_data_structure.bar_action import BarSceneAction

from husky_assembly_teleop.bar_action_io import LIVE_SOLVED_TAG
from husky_assembly_teleop.robot_registry import RobotSpec, other_robots, robot_by_name
from husky_assembly_teleop.schedule_io import ActionSchedule, ScheduleEntry

PROGRESS_FILENAME = 'progress.json'
# Where 'Reset schedule to the Rhino export' moves the progress and the saved plans.
ARCHIVE_DIRNAME = 'archive'
SCHEMA_VERSION = 1

# * Entry statuses.
STATUS_PENDING = 'pending'
STATUS_DONE = 'done'
STATUS_SKIPPED = 'skipped'

# * Where a robot belief came from.
BELIEF_LIVE = 'live'                    # mocap base + joints read from the robot
BELIEF_ACTION_END = 'action_end_state'  # end state of the last action it finished
BELIEF_ASSUMED = 'assumed'              # set by the operator without evidence
BELIEF_EXPORTED = 'exported'            # the obstacle tool_state the exporter wrote
BELIEF_PARKED = 'parked'                # moved out of the way (see PARKED_BASE_FRAME)

# * Hold states.
HOLD_PENDING = 'pending'
HOLD_HOLDING = 'holding'
HOLD_RELEASED = 'released'

# Where the exporter puts a robot that is out of the scene: far away, arm at zeros.
PARKED_BASE_FRAME = Frame((50.0, 50.0, 0.0), (1, 0, 0), (0, 1, 0))


@dataclass
class EntryStatus:
    """Status of one schedule entry.

    Attributes:
        status (str): ``'pending'``, ``'done'`` or ``'skipped'``.
        marked_at (str | None): When it was marked (ISO time).
        marked_by_robot (str | None): The robot whose monitor marked it.
        run_id (str | None): The monitor run that marked it.
    """

    status: str = STATUS_PENDING
    marked_at: Optional[str] = None
    marked_by_robot: Optional[str] = None
    run_id: Optional[str] = None


@dataclass
class RobotBelief:
    """Best knowledge of where a robot is.

    Attributes:
        base_frame (Frame): Robot base in the world (meters).
        configuration (Configuration): Arm joints only, in the robot's
            ``all_arm_joint_names`` order.
        source (str): One of the ``BELIEF_*`` values.
        at (str): When the belief was made (ISO time).
        after_entry (int | None): The schedule entry whose end this describes.
    """

    base_frame: Frame
    configuration: Configuration
    source: str
    at: str
    after_entry: Optional[int] = None

    def to_dict(self) -> dict:
        """The belief as a dict (Frame / Configuration kept as compas objects).

        Returns:
            dict: Ready for ``compas.data.json_dumps``.
        """
        return {'base_frame': self.base_frame, 'configuration': self.configuration,
                'source': self.source, 'at': self.at, 'after_entry': self.after_entry}

    @classmethod
    def from_dict(cls, data: dict) -> 'RobotBelief':
        """Build a belief from ``to_dict``'s output (after ``json_load``).

        Args:
            data (dict): The belief dict.

        Returns:
            RobotBelief: The belief.
        """
        return cls(**data)


@dataclass
class HoldState:
    """Where a support robot's hold of one bar stands.

    Attributes:
        bar_id (str): The held bar.
        robot (str): The support robot.
        state (str): ``'pending'``, ``'holding'`` or ``'released'``.
        since_index (int | None): The hold entry that started the hold.
        released_at_index (int | None): The hold release entry that ended it.
    """

    bar_id: str
    robot: str
    state: str
    since_index: Optional[int] = None
    released_at_index: Optional[int] = None


@dataclass
class Progress:
    """Everything ``progress.json`` holds.

    Attributes:
        problem (str): Design problem name.
        schema_version (int): File schema version (1).
        schedule_fingerprint (str): Identifies the schedule it was written for.
        current_index (int): The entry the operator is at (the first pending one).
        updated_at (str): Last save (ISO time).
        entries (dict): ``{index: EntryStatus}`` for every schedule entry.
        robots (dict): ``{robot name: RobotBelief}``.
        holds (dict): ``{bar_id: HoldState}``.
    """

    problem: str
    schema_version: int
    schedule_fingerprint: str
    current_index: int
    updated_at: str
    entries: dict = field(default_factory=dict)
    robots: dict = field(default_factory=dict)
    holds: dict = field(default_factory=dict)

    # * --------------------------------------------------------- entries
    def status(self, i: int) -> str:
        """Status of entry ``i``.

        Args:
            i (int): Schedule index.

        Returns:
            str: ``'pending'``, ``'done'`` or ``'skipped'``.
        """
        return self.entries[i].status

    def is_done(self, i: int) -> bool:
        """Whether entry ``i`` is done.

        Args:
            i (int): Schedule index.

        Returns:
            bool: True when done.
        """
        return self.status(i) == STATUS_DONE

    def next_pending_index(self) -> Optional[int]:
        """The first entry still pending.

        Returns:
            int | None: Its index, or None when nothing is pending.
        """
        pending = [i for i, s in self.entries.items() if s.status == STATUS_PENDING]
        return min(pending) if pending else None

    def mark_done(self, entry: ScheduleEntry, robot: str, run_id: str,
                  belief: Optional[RobotBelief] = None) -> None:
        """Mark an entry done and record what that changes.

        Args:
            entry (ScheduleEntry): The finished entry.
            robot (str): The robot whose monitor marks it (may differ from the
                entry's robot when the operator marks another robot's step).
            run_id (str): The monitor run that marks it.
            belief (RobotBelief | None): The acting robot's state at the end of
                the entry; stored for ``entry.robot``, stamped with this entry.
        """
        self._mark(entry, STATUS_DONE, robot, run_id)
        if belief is not None:
            self.robots[entry.robot] = replace(belief, after_entry=entry.index)
        if entry.kind == 'H':
            self.holds[entry.bar_id] = HoldState(entry.bar_id, entry.robot, HOLD_HOLDING,
                                                 since_index=entry.index)
        elif entry.kind == 'HR':
            hold = self.holds.setdefault(
                entry.bar_id, HoldState(entry.bar_id, entry.robot, HOLD_RELEASED))
            hold.state = HOLD_RELEASED
            hold.released_at_index = entry.index
        self._advance()

    def skip(self, entry: ScheduleEntry, robot: str, run_id: str) -> None:
        """Mark an entry skipped (no belief or hold change).

        Args:
            entry (ScheduleEntry): The skipped entry.
            robot (str): The robot whose monitor marks it.
            run_id (str): The monitor run that marks it.
        """
        self._mark(entry, STATUS_SKIPPED, robot, run_id)
        self._advance()

    def reopen(self, entry: ScheduleEntry) -> None:
        """Put an entry back to pending and undo what marking it recorded.

        A reopened hold goes back to pending, a reopened hold release back to
        holding. The acting robot's belief is dropped when it came from this
        entry (``recompute_belief`` can rebuild it from the entry before).

        Args:
            entry (ScheduleEntry): The entry to reopen.
        """
        self.entries[entry.index] = EntryStatus()
        hold = self.holds.get(entry.bar_id)
        if hold is not None and entry.kind == 'H':
            hold.state = HOLD_PENDING
            hold.since_index = None
            hold.released_at_index = None
        elif hold is not None and entry.kind == 'HR':
            hold.state = HOLD_HOLDING
            hold.released_at_index = None
        self.current_index = min(self.current_index, entry.index)
        belief = self.robots.get(entry.robot)
        if belief is not None and belief.after_entry == entry.index:
            del self.robots[entry.robot]

    def _mark(self, entry: ScheduleEntry, status: str, robot: str, run_id: str) -> None:
        """Stamp an entry with a status.

        Args:
            entry (ScheduleEntry): The entry.
            status (str): The new status.
            robot (str): The robot whose monitor marks it.
            run_id (str): The monitor run that marks it.
        """
        self.entries[entry.index] = EntryStatus(status, now_iso(), robot, run_id)

    def _advance(self) -> None:
        """Move ``current_index`` to the first pending entry (past the end when none)."""
        pending = self.next_pending_index()
        self.current_index = pending if pending is not None else len(self.entries)

    # * --------------------------------------------------------- robots / holds
    def set_belief(self, robot: str, belief: RobotBelief) -> None:
        """Store a robot's belief.

        Args:
            robot (str): Short robot name.
            belief (RobotBelief): The belief.
        """
        self.robots[robot] = belief

    def belief(self, robot: str) -> Optional[RobotBelief]:
        """A robot's stored belief.

        Args:
            robot (str): Short robot name.

        Returns:
            RobotBelief | None: The belief, or None when there is none.
        """
        return self.robots.get(robot)

    def hold_state(self, bar_id: str) -> Optional[HoldState]:
        """The hold of one bar.

        Args:
            bar_id (str): e.g. ``'B3'``.

        Returns:
            HoldState | None: Its state, or None when the bar is not held.
        """
        return self.holds.get(bar_id)

    def holding_bars(self) -> dict:
        """The bars support robots are holding right now.

        Returns:
            dict: ``{robot: bar_id}``.
        """
        return {h.robot: h.bar_id for h in self.holds.values() if h.state == HOLD_HOLDING}

    def released_robots(self) -> set:
        """Support robots that released a hold and have not started holding again.

        Returns:
            set: Robot names.
        """
        released = {h.robot for h in self.holds.values() if h.state == HOLD_RELEASED}
        return released - set(self.holding_bars())

    # * --------------------------------------------------------- (de)serialize
    def to_dict(self) -> dict:
        """The progress as the ``progress.json`` dict.

        Returns:
            dict: Frame / Configuration kept as compas objects, for
            ``compas.data.json_dumps``.
        """
        return {
            'schema_version': self.schema_version,
            'problem': self.problem,
            'schedule_fingerprint': self.schedule_fingerprint,
            'updated_at': self.updated_at,
            'current_index': self.current_index,
            'entries': {str(i): asdict(s) for i, s in sorted(self.entries.items())},
            'robots': {name: b.to_dict() for name, b in self.robots.items()},
            'holds': {bar: asdict(h) for bar, h in self.holds.items()},
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'Progress':
        """Build a Progress from ``to_dict``'s output (after ``json_load``).

        Args:
            data (dict): The progress dict.

        Returns:
            Progress: The progress.
        """
        return cls(
            problem=data['problem'],
            schema_version=data['schema_version'],
            schedule_fingerprint=data['schedule_fingerprint'],
            current_index=data['current_index'],
            updated_at=data['updated_at'],
            entries={int(i): EntryStatus(**s) for i, s in data.get('entries', {}).items()},
            robots={name: RobotBelief.from_dict(b) for name, b in data.get('robots', {}).items()},
            holds={bar: HoldState(**h) for bar, h in data.get('holds', {}).items()},
        )


# * ------------------------------------------------------------- file I/O
def progress_path(problem_root: str) -> str:
    """Where a problem's progress file lives.

    Args:
        problem_root (str): The design problem folder.

    Returns:
        str: ``<problem_root>/progress.json``.
    """
    return os.path.join(problem_root, PROGRESS_FILENAME)


def schedule_fingerprint(schedule: ActionSchedule) -> str:
    """Short id of a schedule's content, to notice a re-exported schedule.

    Args:
        schedule (ActionSchedule): The schedule.

    Returns:
        str: ``'<number of entries>:<sha1 of the action ids in order>'``.
    """
    ids = [e.action_id for e in schedule.entries]
    digest = sha1('\n'.join(ids).encode('utf-8')).hexdigest()
    return f"{len(ids)}:{digest}"


def new_progress(schedule: ActionSchedule) -> Progress:
    """A fresh progress: every entry pending, no beliefs, every hold pending.

    Args:
        schedule (ActionSchedule): The schedule.

    Returns:
        Progress: The new progress (not saved).
    """
    return Progress(
        problem=schedule.problem_name,
        schema_version=SCHEMA_VERSION,
        schedule_fingerprint=schedule_fingerprint(schedule),
        current_index=0,
        updated_at=now_iso(),
        entries={e.index: EntryStatus() for e in schedule.entries},
        robots={},
        holds={h.bar_id: HoldState(h.bar_id, h.robot, HOLD_PENDING) for h in schedule.holds},
    )


def load_progress(problem_root: str, schedule: ActionSchedule) -> Progress:
    """Load a problem's progress, or start a new one when there is none.

    Args:
        problem_root (str): The design problem folder.
        schedule (ActionSchedule): The problem's schedule.

    Returns:
        Progress: The progress. Entries / holds the file does not mention are
        added as pending.

    Raises:
        ValueError: The file belongs to another design problem.
    """
    path = progress_path(problem_root)
    if not os.path.isfile(path):
        return new_progress(schedule)
    progress = Progress.from_dict(json_load(path))
    if progress.problem != schedule.problem_name:
        raise ValueError(f"{path} is for problem {progress.problem!r}, "
                         f"not {schedule.problem_name!r}")
    if progress.schedule_fingerprint != schedule_fingerprint(schedule):
        # ! The schedule was re-exported since this was written; the statuses are
        # ! kept by index, so the operator must check they still make sense.
        print(f"[Progress] {PROGRESS_FILENAME} was written for another version of "
              f"{schedule.problem_name}'s schedule ({progress.schedule_fingerprint} vs "
              f"{schedule_fingerprint(schedule)}); keeping its statuses -- check them.")
        # Adopt the current schedule's fingerprint, so the warning shows once and
        # the next save records which schedule the statuses now refer to.
        progress.schedule_fingerprint = schedule_fingerprint(schedule)
    for e in schedule.entries:
        progress.entries.setdefault(e.index, EntryStatus())
    for h in schedule.holds:
        progress.holds.setdefault(h.bar_id, HoldState(h.bar_id, h.robot, HOLD_PENDING))
    return progress


def save_progress(progress: Progress, problem_root: str) -> str:
    """Write the progress file atomically (a reader never sees half a file).

    Args:
        progress (Progress): The progress; its ``updated_at`` is set to now.
        problem_root (str): The design problem folder.

    Returns:
        str: The written path.
    """
    progress.updated_at = now_iso()
    path = progress_path(problem_root)
    tmp = path + '.tmp'
    with open(tmp, 'w') as handle:
        handle.write(json_dumps(progress.to_dict(), pretty=True))
    os.replace(tmp, path)
    return path


def archive_progress_and_saved_plans(problem_root: str,
                                     stamp: Optional[str] = None) -> tuple[str, list[str]]:
    """Move a problem's progress and saved plans aside, so it starts again from the Rhino export.

    Moves ``progress.json`` and every saved plan (``*.live-solved.json``, written
    next to the clean exports) into ``<problem_root>/archive/<stamp>/``, keeping
    their paths relative to the problem folder. Nothing is deleted: moving the
    files back restores the previous state. The clean Rhino exports and
    ``ActionSchedule.json`` are not touched, and earlier archives are left alone.

    Args:
        problem_root (str): The design problem folder.
        stamp (str | None): Name of the archive subfolder; None = the current
            time, e.g. ``'20261006T143012'``.

    Returns:
        tuple[str, list[str]]: The archive folder and the moved files (paths
        relative to ``problem_root``). Nothing is created when there is
        nothing to move (empty list).
    """
    archive_root = os.path.join(problem_root, ARCHIVE_DIRNAME)
    to_move = []
    if os.path.isfile(progress_path(problem_root)):
        to_move.append(PROGRESS_FILENAME)
    saved_plan_ending = f'.{LIVE_SOLVED_TAG}.json'
    for here, subfolders, files in os.walk(problem_root):
        if os.path.abspath(here) == os.path.abspath(problem_root) and ARCHIVE_DIRNAME in subfolders:
            subfolders.remove(ARCHIVE_DIRNAME)  # earlier archives stay where they are
        for name in sorted(files):
            if name.endswith(saved_plan_ending):
                to_move.append(os.path.relpath(os.path.join(here, name), problem_root))
    folder = os.path.join(archive_root, stamp or datetime.now().strftime('%Y%m%dT%H%M%S'))
    for rel in to_move:
        target = os.path.join(folder, rel)
        os.makedirs(os.path.dirname(target), exist_ok=True)
        move(os.path.join(problem_root, rel), target)
    return folder, to_move


def new_run_id(robot: str, now: Optional[datetime] = None) -> str:
    """Id of one monitor run, e.g. ``'Cindy-20260930T135501'``.

    Args:
        robot (str): The connected robot's short name.
        now (datetime | None): The start time (now by default).

    Returns:
        str: The run id.
    """
    return f"{robot}-{(now or datetime.now()):%Y%m%dT%H%M%S}"


def now_iso() -> str:
    """The current local time, to the second.

    Returns:
        str: ISO 8601 time, e.g. ``'2026-09-30T13:55:01'``.
    """
    return datetime.now().isoformat(timespec='seconds')


# * ------------------------------------------------------------- beliefs
def arm_configuration(conf: Configuration, spec: RobotSpec) -> Configuration:
    """Only a robot's arm joints out of a configuration, in the robot's order.

    Joints are matched by NAME, never by position (Cindy's URDF declares the
    right arm first, the exported configurations list the left arm first).

    Args:
        conf (Configuration): A configuration that names every arm joint.
        spec (RobotSpec): The robot.

    Returns:
        Configuration: ``spec.all_arm_joint_names`` with their values and types.

    Raises:
        ValueError: An arm joint is missing from ``conf``.
    """
    values, types = conf.joint_dict, conf.type_dict
    names = spec.all_arm_joint_names
    missing = [n for n in names if n not in values]
    if missing:
        raise ValueError(f"configuration has no value for {spec.name}'s joints {missing}")
    return Configuration([values[n] for n in names], [types[n] for n in names], names)


def belief_after(action: BarSceneAction, spec: RobotSpec, entry_index: Optional[int] = None,
                 source: str = BELIEF_ACTION_END) -> RobotBelief:
    """Where the acting robot is at the end of an action, as authored.

    Args:
        action (BarSceneAction): The loaded action.
        spec (RobotSpec): The robot that runs it.
        entry_index (int | None): The schedule entry it belongs to.
        source (str): Belief source to record.

    Returns:
        RobotBelief: The last movement's target configuration (or, for a step
        where no arm moves, its start configuration) and its base frame.

    Raises:
        ValueError: The last movement has neither configuration, or no base frame.
    """
    last = action.movements[-1]
    state = last.start_state
    conf = last.target_configuration
    if conf is None and state is not None:
        conf = state.robot_configuration
    base = state.robot_base_frame if state is not None else None
    if conf is None or base is None:
        raise ValueError(f"{last.movement_id}: no configuration or base frame to "
                         f"take {spec.name}'s end state from")
    return RobotBelief(base.copy(), arm_configuration(conf, spec), source, now_iso(), entry_index)


def belief_from_live(spec: RobotSpec, base_pose: tuple, arm_joint_values: list,
                     entry_index: Optional[int] = None) -> RobotBelief:
    """A belief from the live robot: mocap base pose and measured arm joints.

    Args:
        spec (RobotSpec): The robot.
        base_pose (tuple): ``(position xyz, quaternion xyzw)`` of the base.
        arm_joint_values (list): One 6-value list per arm, in ``spec.side_keys`` order.
        entry_index (int | None): The schedule entry this describes the end of.

    Returns:
        RobotBelief: Source ``'live'``.
    """
    position, (qx, qy, qz, qw) = base_pose
    # compas quaternions are ordered w, x, y, z.
    frame = Frame.from_quaternion([qw, qx, qy, qz], point=list(position))
    values = [float(v) for arm in arm_joint_values for v in arm]
    conf = Configuration.from_revolute_values(values, joint_names=spec.all_arm_joint_names)
    return RobotBelief(frame, conf, BELIEF_LIVE, now_iso(), entry_index)


def belief_from_exported(action: BarSceneAction, spec: RobotSpec) -> Optional[RobotBelief]:
    """Where the exporter put a robot in ANOTHER robot's action.

    Each action's states carry the other robots as obstacle tools
    (``ObstacleRobot<Name>``); this reads movement 0's.

    Args:
        action (BarSceneAction): An action of another robot.
        spec (RobotSpec): The robot to look up.

    Returns:
        RobotBelief | None: Source ``'exported'``, or None when the action does
        not carry that robot.
    """
    tool_state = action.movements[0].start_state.tool_states.get(spec.obstacle_tool_name)
    if tool_state is None or tool_state.frame is None or tool_state.configuration is None:
        return None
    return RobotBelief(tool_state.frame.copy(), arm_configuration(tool_state.configuration, spec),
                       BELIEF_EXPORTED, now_iso())


def parked_belief(spec: RobotSpec) -> RobotBelief:
    """A robot moved out of the scene: far-away base, arm at zeros.

    Args:
        spec (RobotSpec): The robot.

    Returns:
        RobotBelief: Source ``'parked'``.
    """
    names = spec.all_arm_joint_names
    conf = Configuration.from_revolute_values([0.0] * len(names), joint_names=names)
    return RobotBelief(PARKED_BASE_FRAME.copy(), conf, BELIEF_PARKED, now_iso())


def recompute_belief(progress: Progress, schedule: ActionSchedule, robot: str) -> Optional[RobotBelief]:
    """Rebuild a robot's belief from the last entry it has DONE (e.g. after a reopen).

    Args:
        progress (Progress): The progress.
        schedule (ActionSchedule): The schedule.
        robot (str): Short robot name.

    Returns:
        RobotBelief | None: The end state of its last done entry, or None when
        it has done nothing yet.
    """
    done = [e for e in schedule.entries_for_robot(robot) if progress.is_done(e.index)]
    if not done:
        return None
    last = done[-1]
    return belief_after(schedule.load_action(last), robot_by_name(robot), entry_index=last.index)


def _obstacle_beliefs(progress: Progress, active_robot: str,
                      exported_action: Optional[BarSceneAction] = None) -> dict:
    """The belief used to draw each robot other than the connected one.

    Args:
        progress (Progress): The progress.
        active_robot (str): The connected robot's short name.
        exported_action (BarSceneAction | None): The loaded action of the
            connected robot, whose obstacle tool states are the last resort.

    Returns:
        dict: ``{obstacle_tool_name: RobotBelief}``.
    """
    released = progress.released_robots()
    beliefs = {}
    for spec in other_robots(active_robot):
        belief = None
        if spec.name not in released:
            belief = progress.belief(spec.name)
            if belief is None and exported_action is not None:
                belief = belief_from_exported(exported_action, spec)
        beliefs[spec.obstacle_tool_name] = belief if belief is not None else parked_belief(spec)
    return beliefs


def obstacle_tool_states(progress: Progress, active_robot: str,
                         exported_action: Optional[BarSceneAction] = None) -> dict:
    """Base frame and arm joints to pose each other robot's obstacle tool with.

    Order of preference per robot: released its hold -> parked; else its stored
    belief; else the exported tool state in ``exported_action``; else parked.

    Args:
        progress (Progress): The progress.
        active_robot (str): The connected robot's short name.
        exported_action (BarSceneAction | None): The connected robot's loaded action.

    Returns:
        dict: ``{obstacle_tool_name: (Frame, Configuration)}``.
    """
    beliefs = _obstacle_beliefs(progress, active_robot, exported_action)
    return {name: (b.base_frame, b.configuration) for name, b in beliefs.items()}


def obstacle_sources(progress: Progress, active_robot: str,
                     exported_action: Optional[BarSceneAction] = None) -> dict:
    """Which source won for each other robot (for the UI readout).

    Args:
        progress (Progress): The progress.
        active_robot (str): The connected robot's short name.
        exported_action (BarSceneAction | None): The connected robot's loaded action.

    Returns:
        dict: ``{obstacle_tool_name: source}`` (one of the ``BELIEF_*`` values).
    """
    beliefs = _obstacle_beliefs(progress, active_robot, exported_action)
    return {name: b.source for name, b in beliefs.items()}


def obstacle_sources_line(progress: Progress, active_robot: str,
                          exported_action: Optional[BarSceneAction] = None) -> str:
    """Where each other robot's pose comes from, as one readout line.

    Same rule as ``obstacle_sources``; a belief stamped with an entry names it.

    Args:
        progress (Progress): The progress.
        active_robot (str): The connected robot's short name.
        exported_action (BarSceneAction | None): The connected robot's loaded action.

    Returns:
        str: e.g. ``'Alice <- live (entry 3) | Belle <- parked'``, robots in
        registry order.
    """
    beliefs = _obstacle_beliefs(progress, active_robot, exported_action)
    parts = []
    for spec in other_robots(active_robot):
        belief = beliefs[spec.obstacle_tool_name]
        entry = f" (entry {belief.after_entry})" if belief.after_entry is not None else ''
        parts.append(f"{spec.name} <- {belief.source}{entry}")
    return ' | '.join(parts)
