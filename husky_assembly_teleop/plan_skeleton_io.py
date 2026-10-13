"""Read a design problem's ``ActionSchedule.json``: which robot runs which action, in what order.

The Rhino exporter writes, next to ``BarActions/``, one ``ActionSchedule.json``
per problem. It lists every action of the assembly in execution order::

    {"schema_version": 1,
     "robots": {"Cindy": {"robot_id": "dual-arm_husky_Cindy", "role": "assembly"}, ...},
     "assembly_seq": ["B1", "B3", ...],
     "holds": [{"bar_id": "B3", "robot": "Alice", "release_after_bar_id": "B9"}, ...],
     "schedule": [{"index": 0, "action_id": "B1_J_joint", "type": "BarAssemblyJointingAction",
                   "bar_id": "B1", "robot": "Cindy", "file": "BarActions/B1__J.json"}, ...]}

A RANGE export (``"export_range": {"from_bar_id", "until_bar_id"}``) covers only
part of the assembly: ``assembly_seq`` stops at ``until`` and ``holds`` lists the
holds under way during the range. A hold released after a bar beyond ``until``
names a bar that is not in ``assembly_seq`` and has no release action: it is an
OPEN hold, still on at the end of the bundle (see ``HoldWindow``).

One action = one action file = the monitor's unit of work. Action kinds:
``J`` jointing and ``R`` release (Cindy), ``H`` hold and ``HR`` hold release
(a support robot). A release's movements START where its jointing / hold left
the arm, so ``load_skeleton_action`` also loads that predecessor to know where each flange
starts.

! A problem without ``ActionSchedule.json`` is a legacy problem: ``load_plan_skeleton``
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

SKELETON_FILENAME = 'ActionSchedule.json'
SCHEMA_VERSION = 1

# * The action kind comes from the action TYPE, never from ids or file names.
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
class SkeletonAction:
    """One line of the plan skeleton: one action file run by one robot.

    Attributes:
        index (int): Position in the plan skeleton (0-based, contiguous).
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
            str: The action kind.
        """
        return KIND_BY_TYPE[self.type]

    @property
    def is_support(self) -> bool:
        """Whether a support robot runs this action (a hold or hold release).

        Returns:
            bool: True for ``H`` / ``HR`` actions.
        """
        return self.kind in SUPPORT_KINDS

    @classmethod
    def from_dict(cls, data: dict) -> 'SkeletonAction':
        """Build an action from its JSON dict.

        Args:
            data (dict): One item of the file's ``schedule`` list.

        Returns:
            SkeletonAction: The action.
        """
        return cls(index=int(data['index']), action_id=data['action_id'], type=data['type'],
                   bar_id=data['bar_id'], robot=data['robot'], file=data['file'])

    def to_dict(self) -> dict:
        """The action as its JSON dict.

        Returns:
            dict: Same keys as the file.
        """
        return {'index': self.index, 'action_id': self.action_id, 'type': self.type,
                'bar_id': self.bar_id, 'robot': self.robot, 'file': self.file}


@dataclass(frozen=True)
class HoldWindow:
    """When a support robot holds a bar: from its hold action until its release.

    Attributes:
        bar_id (str): The held bar.
        robot (str): The support robot holding it.
        release_after_bar_id (str): The last bar that must be built before release.
        hold_start_seq (int): Position of ``bar_id`` in ``assembly_seq``.
        release_after_seq (int | None): Position of ``release_after_bar_id`` in
            ``assembly_seq``; None for an OPEN hold of a range export (released
            after a bar beyond the range, so the hold is on until the end).
        hold_action_index (int): Plan skeleton index of the hold (``H``) action.
        release_action_index (int | None): Plan skeleton index of the hold release
            (``HR``) action, or None when the plan skeleton has none.
    """

    bar_id: str
    robot: str
    release_after_bar_id: str
    hold_start_seq: int
    release_after_seq: Optional[int]
    hold_action_index: int
    release_action_index: Optional[int]


@dataclass
class PlanSkeleton:
    """A loaded ``ActionSchedule.json``.

    Attributes:
        problem_root (str): The design problem folder.
        schema_version (int): File schema version (1).
        robots (dict): ``{name: {'robot_id': ..., 'role': ...}}`` as in the file.
        assembly_seq (list): Bar ids in assembly order.
        holds (list): One ``HoldWindow`` per hold, in file order.
        actions (list): ``SkeletonAction`` per action, in execution order.
        export_range (dict | None): ``{'from_bar_id', 'until_bar_id'}`` of a range
            export, as in the file; None for an export that does not say.
    """

    problem_root: str
    schema_version: int
    robots: dict
    assembly_seq: list
    holds: list = field(default_factory=list)
    actions: list = field(default_factory=list)
    export_range: Optional[dict] = None

    @classmethod
    def from_dict(cls, data: dict, problem_root: str) -> 'PlanSkeleton':
        """Build and validate a plan skeleton from the file's JSON dict.

        Args:
            data (dict): The parsed ``ActionSchedule.json``.
            problem_root (str): The design problem folder the file came from.

        Returns:
            PlanSkeleton: The plan skeleton, with its hold windows resolved.

        Raises:
            ValueError: The file breaks one of the rules checked here (schema
                version, contiguous indices, known types / robots, hold bars in
                the assembly sequence with a hold action each, and a known end
                for every hold -- an open hold of a range export counts as one).
        """
        if data.get('schema_version') != SCHEMA_VERSION:
            raise ValueError(f"{SKELETON_FILENAME}: schema_version "
                             f"{data.get('schema_version')!r}, expected {SCHEMA_VERSION}")
        skeleton = cls(problem_root=problem_root, schema_version=data['schema_version'],
                       robots=dict(data['robots']), assembly_seq=list(data['assembly_seq']),
                       actions=[SkeletonAction.from_dict(d) for d in data['schedule']],
                       export_range=data.get('export_range'))
        skeleton._check_actions()
        skeleton.holds = [skeleton._hold_window_from_dict(h) for h in data.get('holds', [])]
        return skeleton

    def _check_actions(self) -> None:
        """Validate the robots table and the actions (see ``from_dict``).

        Raises:
            ValueError: On the first broken rule.
        """
        for name, info in self.robots.items():
            if name not in ROBOTS or ROBOTS[name].robot_id != info.get('robot_id'):
                raise ValueError(f"{SKELETON_FILENAME}: robot {name!r} ({info}) is not in "
                                 f"the robot registry {sorted(ROBOTS)}")
        for position, e in enumerate(self.actions):
            if e.index != position:
                raise ValueError(f"{SKELETON_FILENAME}: action #{position} has index "
                                 f"{e.index}; indices must be 0, 1, 2, ... in order")
            if e.type not in KIND_BY_TYPE:
                raise ValueError(f"{SKELETON_FILENAME}: action {e.index} has unknown type "
                                 f"{e.type!r}; known: {sorted(KIND_BY_TYPE)}")
            if e.robot not in self.robots:
                raise ValueError(f"{SKELETON_FILENAME}: action {e.index} names robot "
                                 f"{e.robot!r}, not in robots {sorted(self.robots)}")

    def _hold_window_from_dict(self, hold: dict) -> HoldWindow:
        """Resolve one ``holds`` item of the file into a HoldWindow.

        Args:
            hold (dict): ``{'bar_id', 'robot', 'release_after_bar_id'}``.

        Returns:
            HoldWindow: With positions and action indices filled in.

        Raises:
            ValueError: A bar is not in the assembly sequence (except the release
                bar of an open hold, see below), the bar has no hold action for
                that robot, or the hold has no end (neither a hold release action
                nor a release of ``release_after_bar_id``).
        """
        # * A range export cuts assembly_seq at its last bar, so a hold released
        # * after a later bar names a bar that is not in it: an OPEN hold, still
        # * on at the end of the bundle (no release action). Only a file that says
        # * it is a range export may have one; elsewhere it is an export error.
        # ? The monitor cannot tell a bar beyond the range from a misspelt one;
        # ? the exporter's hold plan only names real bars.
        open_hold = (self.export_range is not None
                     and hold['release_after_bar_id'] not in self.assembly_seq)
        for key in ('bar_id', 'release_after_bar_id'):
            if hold[key] not in self.assembly_seq and not (open_hold and key == 'release_after_bar_id'):
                raise ValueError(f"{SKELETON_FILENAME}: hold {hold} names {hold[key]!r}, "
                                 f"which is not in assembly_seq")
        hold_action = self.find_action(hold['bar_id'], 'H')
        if hold_action is None or hold_action.robot != hold['robot']:
            raise ValueError(f"{SKELETON_FILENAME}: hold {hold} has no H action by that robot")
        release_action = self.find_action(hold['bar_id'], 'HR')
        # * hold_end_index needs one of the two to know when the hold is over
        if (not open_hold and release_action is None
                and self.find_action(hold['release_after_bar_id'], 'R') is None):
            raise ValueError(f"{SKELETON_FILENAME}: hold {hold} has no HR action and no R action "
                             f"for {hold['release_after_bar_id']!r}, so its end is unknown")
        return HoldWindow(
            bar_id=hold['bar_id'],
            robot=hold['robot'],
            release_after_bar_id=hold['release_after_bar_id'],
            hold_start_seq=self.seq_position(hold['bar_id']),
            release_after_seq=(None if open_hold
                               else self.seq_position(hold['release_after_bar_id'])),
            hold_action_index=hold_action.index,
            release_action_index=release_action.index if release_action is not None else None,
        )

    # * --------------------------------------------------------- lookups
    @property
    def problem_name(self) -> str:
        """The design problem's folder name.

        Returns:
            str: e.g. ``'260920_RobArch_demo_revamp_backup'``.
        """
        return os.path.basename(os.path.normpath(self.problem_root))

    def action(self, i: int) -> SkeletonAction:
        """The action at plan skeleton index ``i``.

        Args:
            i (int): Plan skeleton index.

        Returns:
            SkeletonAction: The action.
        """
        return self.actions[i]

    def actions_for_robot(self, robot: str) -> list:
        """Every action one robot runs, in order.

        Args:
            robot (str): Short robot name.

        Returns:
            list: SkeletonActions.
        """
        return [e for e in self.actions if e.robot == robot]

    def actions_for_bar(self, bar_id: str) -> list:
        """Every action about one bar, in order.

        Args:
            bar_id (str): e.g. ``'B3'``.

        Returns:
            list: SkeletonActions.
        """
        return [e for e in self.actions if e.bar_id == bar_id]

    def find_action(self, bar_id: str, kind: str) -> Optional[SkeletonAction]:
        """The action of one kind for one bar.

        Args:
            bar_id (str): e.g. ``'B3'``.
            kind (str): ``'J'``, ``'R'``, ``'H'`` or ``'HR'``.

        Returns:
            SkeletonAction | None: The first match, or None.
        """
        for e in self.actions:
            if e.bar_id == bar_id and e.kind == kind:
                return e
        return None

    def predecessor(self, action: SkeletonAction) -> Optional[SkeletonAction]:
        """The action a release starts from: ``R`` -> its ``J``, ``HR`` -> its ``H``.

        Args:
            action (SkeletonAction): Any action.

        Returns:
            SkeletonAction | None: The predecessor, or None for ``J`` / ``H``
            (they start from wherever the robot is).
        """
        kind = PREDECESSOR_KIND.get(action.kind)
        return self.find_action(action.bar_id, kind) if kind else None

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
        """Plan skeleton index at which the hold ends (the robot is no longer holding).

        Args:
            hold (HoldWindow): A hold.

        Returns:
            int: The hold release action's index when there is one; for an open
            hold (range export) one past the last action -- it is still on at the
            end; otherwise the action right after the release (``R``) of
            ``release_after_bar_id``.
        """
        if hold.release_action_index is not None:
            return hold.release_action_index
        if hold.release_after_seq is None:
            return len(self.actions)
        return self.find_action(hold.release_after_bar_id, 'R').index + 1

    def robots_holding_at(self, action: SkeletonAction) -> dict:
        """Which support robots are frozen holding a bar while ``action`` runs.

        The robot running the hold action itself is not "holding" yet (it is the
        one acting), and neither is the robot running the hold release.

        Args:
            action (SkeletonAction): The action being run.

        Returns:
            dict: ``{robot: bar_id}``.
        """
        return {h.robot: h.bar_id for h in self.holds
                if h.hold_action_index < action.index < self.hold_end_index(h)}

    def held_bar_ids(self, hold: HoldWindow) -> list:
        """The bars built while (and including) the held bar is supported.

        Args:
            hold (HoldWindow): A hold.

        Returns:
            list: ``assembly_seq`` from the held bar to ``release_after_bar_id``,
            both included (to the end of ``assembly_seq`` for an open hold).
        """
        end = (len(self.assembly_seq) if hold.release_after_seq is None
               else hold.release_after_seq + 1)
        return self.assembly_seq[hold.hold_start_seq:end]

    def executable_by(self, robot: str) -> list:
        """The actions the given robot can run (only its own).

        Args:
            robot (str): Short robot name.

        Returns:
            list: SkeletonActions.

        Raises:
            KeyError: The robot is not in this plan skeleton.
        """
        if robot not in self.robots:
            raise KeyError(f"robot {robot!r} is not in the plan skeleton; valid: {sorted(self.robots)}")
        return [e for e in self.actions if is_executable_by(e, robot)]

    # * --------------------------------------------------------- action files
    def action_path(self, action: SkeletonAction, prefer_sidecar: bool = True) -> str:
        """Absolute path of an action's file.

        Args:
            action (SkeletonAction): The action.
            prefer_sidecar (bool): Use the ``.live-solved.json`` sidecar when it
                exists on disk.

        Returns:
            str: The path to load.
        """
        path = os.path.normpath(os.path.join(self.problem_root, action.file))
        return preferred_action_path(path) if prefer_sidecar else path

    def load_bar_action(self, action: SkeletonAction, prefer_sidecar: bool = True) -> BarSceneAction:
        """Load an action's bar action and check it matches the plan skeleton.

        Args:
            action (SkeletonAction): The action.
            prefer_sidecar (bool): See ``action_path``.

        Returns:
            BarSceneAction: The loaded action.

        Raises:
            ValueError: The file's action type or robot differs from the action.
        """
        return _load_checked(action, self.action_path(action, prefer_sidecar))


def _load_checked(action: SkeletonAction, path: str) -> BarSceneAction:
    """Load an action file and check its type and robot against the action.

    Args:
        action (SkeletonAction): The action the file belongs to.
        path (str): The file to load.

    Returns:
        BarSceneAction: The loaded action.

    Raises:
        ValueError: The type or robot does not match.
    """
    bar_action = parse_bar_action(path)
    if type(bar_action).__name__ != action.type:
        raise ValueError(f"{os.path.basename(path)} holds a {type(bar_action).__name__}, but "
                         f"action {action.index} expects {action.type}")
    expected_id = robot_by_name(action.robot).robot_id
    if bar_action.robot_id != expected_id:
        raise ValueError(f"{os.path.basename(path)} is for robot {bar_action.robot_id!r}, but "
                         f"action {action.index} expects {expected_id!r}")
    return bar_action


def is_executable_by(action: SkeletonAction, robot_name: str) -> bool:
    """Whether the given robot runs this action.

    Args:
        action (SkeletonAction): The action.
        robot_name (str): Short robot name.

    Returns:
        bool: True when the action is assigned to that robot.
    """
    return action.robot == robot_name


def skeleton_path(problem_root: str) -> str:
    """Where a problem's plan skeleton file lives.

    Args:
        problem_root (str): The design problem folder.

    Returns:
        str: ``<problem_root>/ActionSchedule.json``.
    """
    return os.path.join(problem_root, SKELETON_FILENAME)


def problem_root(problem_name: str, design_dir: str = DESIGN_DATA_DIRECTORY) -> str:
    """The folder of a design problem.

    Args:
        problem_name (str): e.g. ``'260920_RobArch_demo_revamp_backup'``.
        design_dir (str): The design-study data folder.

    Returns:
        str: ``<design_dir>/<problem_name>``.
    """
    return os.path.join(design_dir, problem_name)


def load_plan_skeleton(problem_root: str) -> Optional[PlanSkeleton]:
    """Load and validate a problem's ``ActionSchedule.json``.

    Args:
        problem_root (str): The design problem folder.

    Returns:
        PlanSkeleton | None: The plan skeleton, or None when the problem has no
        plan skeleton file (a legacy problem).

    Raises:
        ValueError: The file exists but is not a valid plan skeleton.
    """
    path = skeleton_path(problem_root)
    if not os.path.isfile(path):
        return None
    with open(path) as handle:
        data = json_load_std(handle)
    return PlanSkeleton.from_dict(data, problem_root)


# * ------------------------------------------------------------- loaded actions
@dataclass
class LoadedAction:
    """An action with its bar action loaded and everything the monitor derives from it.

    Attributes:
        action (SkeletonAction): The action.
        action: The loaded action (BarSceneAction subclass).
        path (str): The file it was loaded from.
        spec (RobotSpec): The robot running it.
        predecessor (LoadedAction | None): For ``R`` / ``HR``: the loaded ``J`` / ``H``.
        kinds (list): ``MovementKind`` per movement.
        start_ee_sources (list): Per movement, ``{side: Movement | None}`` -- the
            movement whose authored target is where that flange starts.
    """

    action: SkeletonAction
    bar_action: object
    path: str
    spec: RobotSpec
    predecessor: Optional['LoadedAction']
    kinds: list
    start_ee_sources: list

    @property
    def movements(self) -> list:
        """The action's movements.

        Returns:
            list: Movements, in order.
        """
        return self.bar_action.movements

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


def load_skeleton_action(skeleton: PlanSkeleton, action: SkeletonAction, *,
                         prefer_sidecar: bool = True, with_predecessor: bool = True) -> LoadedAction:
    """Load an action's bar action (and its predecessor) and derive kinds, start poses.

    A release starts where its jointing / hold left the arm (e.g. ``B3_R``'s
    retreat starts at ``B3_J``'s insert targets, ``B3_HR``'s retreat at
    ``B3_H``'s linear approach), so the predecessor's movements are walked first
    and only this action's part of the start-pose table is kept.

    Args:
        skeleton (PlanSkeleton): The loaded plan skeleton.
        action (SkeletonAction): The action to load.
        prefer_sidecar (bool): Load ``.live-solved.json`` sidecars when present.
        with_predecessor (bool): Also load the predecessor (``R`` / ``HR`` only).

    Returns:
        LoadedAction: The loaded action.

    Raises:
        ValueError: A file does not match its action, the predecessor is for
            another robot, or an action holds two (or no) transfer / insert /
            retreat movements.
    """
    spec = robot_by_name(action.robot)
    path = skeleton.action_path(action, prefer_sidecar)
    bar_action = _load_checked(action, path)
    # * The predecessor goes through this same call, so it is checked too.
    check_action_kinds(bar_action, os.path.basename(path))

    pred = None
    pred_action = skeleton.predecessor(action) if with_predecessor else None
    if pred_action is not None:
        pred = load_skeleton_action(skeleton, pred_action, prefer_sidecar=prefer_sidecar,
                                    with_predecessor=False)
        if pred.bar_action.robot_id != bar_action.robot_id:
            raise ValueError(f"action {action.index} ({bar_action.robot_id}) and its predecessor "
                             f"action {pred_action.index} ({pred.bar_action.robot_id}) are "
                             f"different robots")
    pred_movements = pred.movements if pred is not None else []
    sources = cycle_start_ee_sources(pred_movements + bar_action.movements, spec.side_keys)

    return LoadedAction(
        action=action,
        bar_action=bar_action,
        path=path,
        spec=spec,
        predecessor=pred,
        kinds=[movement_kind(mv) for mv in bar_action.movements],
        start_ee_sources=sources[len(pred_movements):],
    )
