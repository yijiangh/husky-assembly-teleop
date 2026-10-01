"""Read a design problem's ``ActionSchedule.json``: which robot runs which action, in what order.

The Rhino exporter writes, next to ``BarActions/``, one ``ActionSchedule.json``
per problem. It lists every action of the assembly in execution order::

    {"schema_version": 1,
     "robots": {"Cindy": {"robot_id": "dual-arm_husky_Cindy", "role": "assembly"}, ...},
     "assembly_seq": ["B1", "B3", ...],
     "holds": [{"bar_id": "B3", "robot": "Alice", "release_after_bar_id": "B9"}, ...],
     "schedule": [{"index": 0, "action_id": "B1_J_joint", "type": "BarAssemblyJointingAction",
                   "bar_id": "B1", "robot": "Cindy", "file": "BarActions/B1__J.json"}, ...]}

One schedule entry = one action file = the monitor's unit of work. Entry kinds:
``J`` jointing and ``R`` release (Cindy), ``H`` hold and ``HR`` hold release
(a support robot). A release's movements START where its jointing / hold left
the arm, so ``load_entry`` also loads that predecessor to know where each flange
starts.

! A problem without ``ActionSchedule.json`` is a legacy problem: ``load_schedule``
! returns None and the caller keeps using ``bar_action_io.load_action_cycle``.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from json import load as json_load_std
from typing import Optional

from rs_data_structure.bar_action import BarSceneAction, Movement

from husky_assembly_teleop import DESIGN_DATA_DIRECTORY
from husky_assembly_teleop.bar_action_io import (
    check_action_kinds,
    cycle_start_ee_sources,
    movement_kind,
    parse_bar_action,
    preferred_action_path,
)
from husky_assembly_teleop.robot_registry import ROBOTS, RobotSpec, robot_by_name

SCHEDULE_FILENAME = 'ActionSchedule.json'
SCHEMA_VERSION = 1

# * The entry kind comes from the action TYPE, never from ids or file names.
KIND_BY_TYPE = {
    'BarAssemblyJointingAction': 'J',
    'BarAssemblyReleaseAction': 'R',
    'BarHoldingAction': 'H',
    'BarHoldingReleaseAction': 'HR',
}
# The action whose end state a release starts from (same bar, same robot).
PREDECESSOR_KIND = {'R': 'J', 'HR': 'H'}
SUPPORT_KINDS = ('H', 'HR')


@dataclass(frozen=True)
class ScheduleEntry:
    """One line of the schedule: one action file run by one robot.

    Attributes:
        index (int): Position in the schedule (0-based, contiguous).
        action_id (str): e.g. ``'B3_H_hold'``.
        type (str): The action class name, e.g. ``'BarHoldingAction'``.
        bar_id (str): The bar the action is about.
        robot (str): Short robot name, e.g. ``'Alice'``.
        file (str): Action file, relative to the problem folder.
    """

    index: int
    action_id: str
    type: str
    bar_id: str
    robot: str
    file: str

    @property
    def kind(self) -> str:
        """``'J'``, ``'R'``, ``'H'`` or ``'HR'``, from the action type.

        Returns:
            str: The entry kind.
        """
        return KIND_BY_TYPE[self.type]

    @property
    def is_support(self) -> bool:
        """Whether a support robot runs this entry (a hold or hold release).

        Returns:
            bool: True for ``H`` / ``HR`` entries.
        """
        return self.kind in SUPPORT_KINDS

    @classmethod
    def from_dict(cls, data: dict) -> 'ScheduleEntry':
        """Build an entry from its JSON dict.

        Args:
            data (dict): One item of the file's ``schedule`` list.

        Returns:
            ScheduleEntry: The entry.
        """
        return cls(index=int(data['index']), action_id=data['action_id'], type=data['type'],
                   bar_id=data['bar_id'], robot=data['robot'], file=data['file'])

    def to_dict(self) -> dict:
        """The entry as its JSON dict.

        Returns:
            dict: Same keys as the file.
        """
        return {'index': self.index, 'action_id': self.action_id, 'type': self.type,
                'bar_id': self.bar_id, 'robot': self.robot, 'file': self.file}


@dataclass(frozen=True)
class HoldWindow:
    """When a support robot holds a bar: from its hold entry until its release.

    Attributes:
        bar_id (str): The held bar.
        robot (str): The support robot holding it.
        release_after_bar_id (str): The last bar that must be built before release.
        hold_start_seq (int): Position of ``bar_id`` in ``assembly_seq``.
        release_after_seq (int): Position of ``release_after_bar_id`` in ``assembly_seq``.
        hold_entry_index (int): Schedule index of the hold (``H``) entry.
        release_entry_index (int | None): Schedule index of the hold release
            (``HR``) entry, or None when the schedule has none.
    """

    bar_id: str
    robot: str
    release_after_bar_id: str
    hold_start_seq: int
    release_after_seq: int
    hold_entry_index: int
    release_entry_index: Optional[int]


@dataclass
class ActionSchedule:
    """A loaded ``ActionSchedule.json``.

    Attributes:
        problem_root (str): The design problem folder.
        schema_version (int): File schema version (1).
        robots (dict): ``{name: {'robot_id': ..., 'role': ...}}`` as in the file.
        assembly_seq (list): Bar ids in assembly order.
        holds (list): One ``HoldWindow`` per hold, in file order.
        entries (list): ``ScheduleEntry`` per action, in execution order.
    """

    problem_root: str
    schema_version: int
    robots: dict
    assembly_seq: list
    holds: list = field(default_factory=list)
    entries: list = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: dict, problem_root: str) -> 'ActionSchedule':
        """Build and validate a schedule from the file's JSON dict.

        Args:
            data (dict): The parsed ``ActionSchedule.json``.
            problem_root (str): The design problem folder the file came from.

        Returns:
            ActionSchedule: The schedule, with its hold windows resolved.

        Raises:
            ValueError: The file breaks one of the rules checked here (schema
                version, contiguous indices, known types / robots, hold bars in
                the assembly sequence with a hold entry each, and a known end
                for every hold).
        """
        if data.get('schema_version') != SCHEMA_VERSION:
            raise ValueError(f"{SCHEDULE_FILENAME}: schema_version "
                             f"{data.get('schema_version')!r}, expected {SCHEMA_VERSION}")
        schedule = cls(problem_root=problem_root, schema_version=data['schema_version'],
                       robots=dict(data['robots']), assembly_seq=list(data['assembly_seq']),
                       entries=[ScheduleEntry.from_dict(d) for d in data['schedule']])
        schedule._check_entries()
        schedule.holds = [schedule._hold_window_from_dict(h) for h in data.get('holds', [])]
        return schedule

    def _check_entries(self) -> None:
        """Validate the robots table and the entries (see ``from_dict``).

        Raises:
            ValueError: On the first broken rule.
        """
        for name, info in self.robots.items():
            if name not in ROBOTS or ROBOTS[name].robot_id != info.get('robot_id'):
                raise ValueError(f"{SCHEDULE_FILENAME}: robot {name!r} ({info}) is not in "
                                 f"the robot registry {sorted(ROBOTS)}")
        for position, e in enumerate(self.entries):
            if e.index != position:
                raise ValueError(f"{SCHEDULE_FILENAME}: entry #{position} has index "
                                 f"{e.index}; indices must be 0, 1, 2, ... in order")
            if e.type not in KIND_BY_TYPE:
                raise ValueError(f"{SCHEDULE_FILENAME}: entry {e.index} has unknown type "
                                 f"{e.type!r}; known: {sorted(KIND_BY_TYPE)}")
            if e.robot not in self.robots:
                raise ValueError(f"{SCHEDULE_FILENAME}: entry {e.index} names robot "
                                 f"{e.robot!r}, not in robots {sorted(self.robots)}")

    def _hold_window_from_dict(self, hold: dict) -> HoldWindow:
        """Resolve one ``holds`` item of the file into a HoldWindow.

        Args:
            hold (dict): ``{'bar_id', 'robot', 'release_after_bar_id'}``.

        Returns:
            HoldWindow: With positions and entry indices filled in.

        Raises:
            ValueError: A bar is not in the assembly sequence, the bar has no
                hold entry for that robot, or the hold has no end (neither a
                hold release entry nor a release of ``release_after_bar_id``).
        """
        for key in ('bar_id', 'release_after_bar_id'):
            if hold[key] not in self.assembly_seq:
                raise ValueError(f"{SCHEDULE_FILENAME}: hold {hold} names {hold[key]!r}, "
                                 f"which is not in assembly_seq")
        hold_entry = self.find_entry(hold['bar_id'], 'H')
        if hold_entry is None or hold_entry.robot != hold['robot']:
            raise ValueError(f"{SCHEDULE_FILENAME}: hold {hold} has no H entry by that robot")
        release_entry = self.find_entry(hold['bar_id'], 'HR')
        # * hold_end_index needs one of the two to know when the hold is over
        if release_entry is None and self.find_entry(hold['release_after_bar_id'], 'R') is None:
            raise ValueError(f"{SCHEDULE_FILENAME}: hold {hold} has no HR entry and no R entry "
                             f"for {hold['release_after_bar_id']!r}, so its end is unknown")
        return HoldWindow(
            bar_id=hold['bar_id'],
            robot=hold['robot'],
            release_after_bar_id=hold['release_after_bar_id'],
            hold_start_seq=self.seq_position(hold['bar_id']),
            release_after_seq=self.seq_position(hold['release_after_bar_id']),
            hold_entry_index=hold_entry.index,
            release_entry_index=release_entry.index if release_entry is not None else None,
        )

    # * --------------------------------------------------------- lookups
    @property
    def problem_name(self) -> str:
        """The design problem's folder name.

        Returns:
            str: e.g. ``'260920_RobArch_demo_revamp_backup'``.
        """
        return os.path.basename(os.path.normpath(self.problem_root))

    def entry(self, i: int) -> ScheduleEntry:
        """The entry at schedule index ``i``.

        Args:
            i (int): Schedule index.

        Returns:
            ScheduleEntry: The entry.
        """
        return self.entries[i]

    def entries_for_robot(self, robot: str) -> list:
        """Every entry one robot runs, in order.

        Args:
            robot (str): Short robot name.

        Returns:
            list: ScheduleEntries.
        """
        return [e for e in self.entries if e.robot == robot]

    def entries_for_bar(self, bar_id: str) -> list:
        """Every entry about one bar, in order.

        Args:
            bar_id (str): e.g. ``'B3'``.

        Returns:
            list: ScheduleEntries.
        """
        return [e for e in self.entries if e.bar_id == bar_id]

    def find_entry(self, bar_id: str, kind: str) -> Optional[ScheduleEntry]:
        """The entry of one kind for one bar.

        Args:
            bar_id (str): e.g. ``'B3'``.
            kind (str): ``'J'``, ``'R'``, ``'H'`` or ``'HR'``.

        Returns:
            ScheduleEntry | None: The first match, or None.
        """
        for e in self.entries:
            if e.bar_id == bar_id and e.kind == kind:
                return e
        return None

    def predecessor(self, entry: ScheduleEntry) -> Optional[ScheduleEntry]:
        """The action a release starts from: ``R`` -> its ``J``, ``HR`` -> its ``H``.

        Args:
            entry (ScheduleEntry): Any entry.

        Returns:
            ScheduleEntry | None: The predecessor, or None for ``J`` / ``H``
            (they start from wherever the robot is).
        """
        kind = PREDECESSOR_KIND.get(entry.kind)
        return self.find_entry(entry.bar_id, kind) if kind else None

    def seq_position(self, bar_id: str) -> int:
        """Position of a bar in the assembly sequence.

        Args:
            bar_id (str): e.g. ``'B3'``.

        Returns:
            int: 0-based position.
        """
        return self.assembly_seq.index(bar_id)

    def hold_window(self, bar_id: str) -> HoldWindow:
        """The hold of one bar.

        Args:
            bar_id (str): A held bar, e.g. ``'B3'``.

        Returns:
            HoldWindow: Its hold.

        Raises:
            KeyError: The bar is not held.
        """
        for hold in self.holds:
            if hold.bar_id == bar_id:
                return hold
        raise KeyError(f"bar {bar_id!r} is not held; held bars: {[h.bar_id for h in self.holds]}")

    def hold_end_index(self, hold: HoldWindow) -> int:
        """Schedule index at which the hold ends (the robot is no longer holding).

        Args:
            hold (HoldWindow): A hold.

        Returns:
            int: The hold release entry's index when there is one; otherwise the
            entry right after the release (``R``) of ``release_after_bar_id``.
        """
        if hold.release_entry_index is not None:
            return hold.release_entry_index
        return self.find_entry(hold.release_after_bar_id, 'R').index + 1

    def robots_holding_at(self, entry: ScheduleEntry) -> dict:
        """Which support robots are frozen holding a bar while ``entry`` runs.

        The robot running the hold entry itself is not "holding" yet (it is the
        one acting), and neither is the robot running the hold release.

        Args:
            entry (ScheduleEntry): The entry being run.

        Returns:
            dict: ``{robot: bar_id}``.
        """
        return {h.robot: h.bar_id for h in self.holds
                if h.hold_entry_index < entry.index < self.hold_end_index(h)}

    def held_bar_ids(self, hold: HoldWindow) -> list:
        """The bars built while (and including) the held bar is supported.

        Args:
            hold (HoldWindow): A hold.

        Returns:
            list: ``assembly_seq`` from the held bar to ``release_after_bar_id``,
            both included.
        """
        return self.assembly_seq[hold.hold_start_seq:hold.release_after_seq + 1]

    def executable_by(self, robot: str) -> list:
        """The entries the given robot can run (only its own).

        Args:
            robot (str): Short robot name.

        Returns:
            list: ScheduleEntries.

        Raises:
            KeyError: The robot is not in this schedule.
        """
        if robot not in self.robots:
            raise KeyError(f"robot {robot!r} is not in the schedule; valid: {sorted(self.robots)}")
        return [e for e in self.entries if is_executable_by(e, robot)]

    # * --------------------------------------------------------- action files
    def action_path(self, entry: ScheduleEntry, prefer_sidecar: bool = True) -> str:
        """Absolute path of an entry's action file.

        Args:
            entry (ScheduleEntry): The entry.
            prefer_sidecar (bool): Use the ``.live-solved.json`` sidecar when it
                exists on disk.

        Returns:
            str: The path to load.
        """
        path = os.path.normpath(os.path.join(self.problem_root, entry.file))
        return preferred_action_path(path) if prefer_sidecar else path

    def load_action(self, entry: ScheduleEntry, prefer_sidecar: bool = True) -> BarSceneAction:
        """Load an entry's action and check it matches the schedule.

        Args:
            entry (ScheduleEntry): The entry.
            prefer_sidecar (bool): See ``action_path``.

        Returns:
            BarSceneAction: The loaded action.

        Raises:
            ValueError: The file's action type or robot differs from the entry.
        """
        return _load_checked(entry, self.action_path(entry, prefer_sidecar))


def _load_checked(entry: ScheduleEntry, path: str) -> BarSceneAction:
    """Load an action file and check its type and robot against the entry.

    Args:
        entry (ScheduleEntry): The schedule entry the file belongs to.
        path (str): The file to load.

    Returns:
        BarSceneAction: The loaded action.

    Raises:
        ValueError: The type or robot does not match.
    """
    action = parse_bar_action(path)
    if type(action).__name__ != entry.type:
        raise ValueError(f"{os.path.basename(path)} holds a {type(action).__name__}, but "
                         f"schedule entry {entry.index} expects {entry.type}")
    expected_id = robot_by_name(entry.robot).robot_id
    if action.robot_id != expected_id:
        raise ValueError(f"{os.path.basename(path)} is for robot {action.robot_id!r}, but "
                         f"schedule entry {entry.index} expects {expected_id!r}")
    return action


def is_executable_by(entry: ScheduleEntry, robot_name: str) -> bool:
    """Whether the given robot runs this entry.

    Args:
        entry (ScheduleEntry): The entry.
        robot_name (str): Short robot name.

    Returns:
        bool: True when the entry is assigned to that robot.
    """
    return entry.robot == robot_name


def schedule_path(problem_root: str) -> str:
    """Where a problem's schedule file lives.

    Args:
        problem_root (str): The design problem folder.

    Returns:
        str: ``<problem_root>/ActionSchedule.json``.
    """
    return os.path.join(problem_root, SCHEDULE_FILENAME)


def problem_root(problem_name: str, design_dir: str = DESIGN_DATA_DIRECTORY) -> str:
    """The folder of a design problem.

    Args:
        problem_name (str): e.g. ``'260920_RobArch_demo_revamp_backup'``.
        design_dir (str): The design-study data folder.

    Returns:
        str: ``<design_dir>/<problem_name>``.
    """
    return os.path.join(design_dir, problem_name)


def load_schedule(problem_root: str) -> Optional[ActionSchedule]:
    """Load and validate a problem's ``ActionSchedule.json``.

    Args:
        problem_root (str): The design problem folder.

    Returns:
        ActionSchedule | None: The schedule, or None when the problem has no
        schedule file (a legacy problem).

    Raises:
        ValueError: The file exists but is not a valid schedule.
    """
    path = schedule_path(problem_root)
    if not os.path.isfile(path):
        return None
    with open(path) as handle:
        data = json_load_std(handle)
    return ActionSchedule.from_dict(data, problem_root)


# * ------------------------------------------------------------- loaded entries
@dataclass
class LoadedEntry:
    """An entry with its action loaded and everything the monitor derives from it.

    Attributes:
        entry (ScheduleEntry): The schedule entry.
        action: The loaded action (BarSceneAction subclass).
        path (str): The file it was loaded from.
        spec (RobotSpec): The robot running it.
        predecessor (LoadedEntry | None): For ``R`` / ``HR``: the loaded ``J`` / ``H``.
        kinds (list): ``MovementKind`` per movement.
        start_ee_sources (list): Per movement, ``{side: Movement | None}`` -- the
            movement whose authored target is where that flange starts.
    """

    entry: ScheduleEntry
    action: object
    path: str
    spec: RobotSpec
    predecessor: Optional['LoadedEntry']
    kinds: list
    start_ee_sources: list

    @property
    def movements(self) -> list:
        """The action's movements.

        Returns:
            list: Movements, in order.
        """
        return self.action.movements

    def start_ee_source(self, idx: int, side: str) -> Optional[Movement]:
        """The movement whose authored target is where ``side``'s flange starts movement ``idx``.

        Args:
            idx (int): Movement index in this action.
            side (str): A key of ``spec.side_keys``.

        Returns:
            Movement | None: The authoring movement, or None when unknown (the
            arm moved without an authored target, or nothing came before).
        """
        return self.start_ee_sources[idx][side]

    def start_ee_frames(self, idx: int) -> dict:
        """Where each flange starts movement ``idx``, for the sides that are known.

        Args:
            idx (int): Movement index in this action.

        Returns:
            dict: ``{side: Frame}`` (sides with no source are left out).
        """
        return {side: src.target_ee_frames[side]
                for side, src in self.start_ee_sources[idx].items() if src is not None}

    def missing_start_sides(self, idx: int) -> list:
        """The sides whose start pose is unknown for movement ``idx``.

        Args:
            idx (int): Movement index in this action.

        Returns:
            list: Side keys, in ``spec.side_keys`` order.
        """
        return [side for side, src in self.start_ee_sources[idx].items() if src is None]


def load_entry(schedule: ActionSchedule, entry: ScheduleEntry, *,
               prefer_sidecar: bool = True, with_predecessor: bool = True) -> LoadedEntry:
    """Load an entry's action (and its predecessor) and derive kinds, start poses.

    A release starts where its jointing / hold left the arm (e.g. ``B3_R``'s
    retreat starts at ``B3_J``'s insert targets, ``B3_HR``'s retreat at
    ``B3_H``'s linear approach), so the predecessor's movements are walked first
    and only this action's part of the start-pose table is kept.

    Args:
        schedule (ActionSchedule): The loaded schedule.
        entry (ScheduleEntry): The entry to load.
        prefer_sidecar (bool): Load ``.live-solved.json`` sidecars when present.
        with_predecessor (bool): Also load the predecessor (``R`` / ``HR`` only).

    Returns:
        LoadedEntry: The loaded entry.

    Raises:
        ValueError: A file does not match its entry, the predecessor is for
            another robot, or an action holds two (or no) transfer / insert /
            retreat movements.
    """
    spec = robot_by_name(entry.robot)
    path = schedule.action_path(entry, prefer_sidecar)
    action = _load_checked(entry, path)
    # * The predecessor goes through this same call, so it is checked too.
    check_action_kinds(action, os.path.basename(path))

    pred = None
    pred_entry = schedule.predecessor(entry) if with_predecessor else None
    if pred_entry is not None:
        pred = load_entry(schedule, pred_entry, prefer_sidecar=prefer_sidecar,
                          with_predecessor=False)
        if pred.action.robot_id != action.robot_id:
            raise ValueError(f"entry {entry.index} ({action.robot_id}) and its predecessor "
                             f"entry {pred_entry.index} ({pred.action.robot_id}) are "
                             f"different robots")
    pred_movements = pred.movements if pred is not None else []
    sources = cycle_start_ee_sources(pred_movements + action.movements, spec.side_keys)

    return LoadedEntry(
        entry=entry,
        action=action,
        path=path,
        spec=spec,
        predecessor=pred,
        kinds=[movement_kind(mv) for mv in action.movements],
        start_ee_sources=sources[len(pred_movements):],
    )
