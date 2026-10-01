"""Text and rules behind the monitor's ActionSchedule panel (no UI toolkit here).

The monitor's schedule section lists every schedule entry as one line, names
the entry and movement the operator is on, and labels the one "run this step"
button. Everything that decides WHAT those widgets say lives here as plain
functions, so it can be tested without Dear PyGui, PyBullet or ROS:

- ``scan_entry_flags(path)``      -> how far an entry's action file is solved
  (arm movements with a goal configuration / a trajectory, sidecar or not)
- ``missing_action_files(schedule)`` -> entries whose action file is not on disk
- ``step_button_label(mv)``       -> the label of the step button, None for an arm move
- ``entry_row_text`` / ``row_color`` -> one row of the entry list
- ``now_line_text``               -> the "Now: entry k -- ... -- movement i/n ..." line
- ``visible_row_window``          -> which rows of a long schedule are shown
- ``knobs_for_assembly_robot``    -> whether Cindy's M1 / M2 tuning widgets apply

* Only ASCII goes into these strings: the DPG font is loaded with its default
* glyph range, so dashes like '—' would not draw.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from json import load as json_load_std
from typing import Optional

from rs_data_structure.bar_action import Movement

from husky_assembly_teleop.bar_action_io import (
    movement_kind,
    sidecar_action_path,
    step_kind,
    tool_event,
)
from husky_assembly_teleop.progress_io import STATUS_DONE
from husky_assembly_teleop.robot_registry import ROLE_ASSEMBLY, RobotSpec
from husky_assembly_teleop.schedule_io import ActionSchedule, ScheduleEntry, is_executable_by

# * Row colours (RGBA 0-255) of the entry list, see row_color.
ROW_COLOR_SELECTED = (240, 220, 80, 255)    # yellow: the entry the slider points at
ROW_COLOR_DONE = (90, 220, 90, 255)         # green: marked done
ROW_COLOR_OTHER_ROBOT = (130, 130, 130, 255)  # grey: another robot runs it
ROW_COLOR_PENDING = (215, 215, 215, 255)    # plain: still to do on this robot

# Movement classes whose serialized name marks a step where no arm moves.
_MANUAL_CLASS = 'ManualMovement'
_TOOL_CLASS_SUFFIX = 'ToolMovement'


@dataclass(frozen=True)
class EntryFlags:
    """How far one entry's action file is solved.

    Attributes:
        n_arm (int): Movements that drive an arm (not a manual / tool step).
        n_ik (int): Of those, how many carry a goal configuration.
        n_traj (int): Of those, how many carry a planned trajectory.
        sidecar (bool): Whether the scanned file is the ``.live-solved.json``
            sidecar (the monitor's own save) rather than the clean export.
    """

    n_arm: int
    n_ik: int
    n_traj: int
    sidecar: bool


def scan_entry_flags(action_path: str) -> EntryFlags:
    """Count an action file's solved arm movements, reading the JSON directly.

    A plain ``json.load`` (no compas objects are built), so all 48 files of a
    schedule can be scanned at start-up.

    Args:
        action_path (str): The action file to scan (the clean export or its sidecar).

    Returns:
        EntryFlags: The counts.
    """
    with open(action_path) as handle:
        movements = json_load_std(handle)['data']['movements']
    arm = [mv['data'] for mv in movements if _is_arm_dtype(mv['dtype'])]
    return EntryFlags(
        n_arm=len(arm),
        n_ik=sum(data.get('target_configuration') is not None for data in arm),
        n_traj=sum(data.get('trajectory') is not None for data in arm),
        sidecar=sidecar_action_path(action_path) == action_path,
    )


def _is_arm_dtype(dtype: str) -> bool:
    """Whether a serialized movement drives an arm.

    Args:
        dtype (str): The compas dtype, e.g. ``'rs_data_structure.bar_action/ManualMovement'``.

    Returns:
        bool: False for a manual step and for any tool step (gripper, scaffolding).
    """
    cls = dtype.rsplit('/', 1)[-1]
    return cls != _MANUAL_CLASS and not cls.endswith(_TOOL_CLASS_SUFFIX)


def missing_action_files(schedule: ActionSchedule) -> list:
    """The entries whose action file is not on disk (neither clean export nor sidecar).

    Args:
        schedule (ActionSchedule): A loaded schedule.

    Returns:
        list: ScheduleEntries, in schedule order.
    """
    return [e for e in schedule.entries if not os.path.isfile(schedule.action_path(e))]


def step_button_label(mv: Optional[Movement]) -> Optional[str]:
    """Label of the monitor's step button for a movement, or None when it has none.

    Arm movements are planned and run with 'Plan Movement' / 'Exec Selected Mv
    Traj (auto)', so they get no step button.

    Args:
        mv (Movement | None): The loaded movement.

    Returns:
        str | None: The label; None for an arm movement, for no movement, and for a
        movement class the monitor does not know.
    """
    if mv is None:
        return None
    try:
        kind = step_kind(mv)
    except TypeError:
        return None
    if kind == 'arm':
        return None
    if kind == 'manual':
        return 'Operator done (manual step) -> then Confirm Exec'
    tool_action, _names, overlaps_next = tool_event(mv)
    if kind == 'gripper':
        if tool_action == 'close':
            return 'Exec gripper step: CLOSE + compliant handoff'
        return f'Exec gripper step: {str(tool_action).upper()}'
    # * Scaffolding: J_M4 tighten (overlaps_next) and R_M1 ungrasp are sent by the
    # * compliant insert / retreat, so their button only marks them done.
    if overlaps_next or tool_action == 'ungrasp':
        return f'Mark tool step done ({tool_action} runs with the next movement)'
    # ! R_M0 untighten is mark-only too: the joint motor is never reversed by the
    # ! schedule (it may back the just-tightened screw off the bar).
    if tool_action == 'untighten':
        return 'Mark tool step done (untighten: use Loosen Joint by hand if needed)'
    return f'Tool step: {tool_action}'


def entry_row_text(entry: ScheduleEntry, flags: EntryFlags, status: str, *,
                   executable: bool, selected: bool) -> str:
    """One fixed-width line of the schedule entry list.

    Example: ``'> [03] H   B3   Alice   IK 2/2  TRJ 0/2  done (other robot)'``.

    Args:
        entry (ScheduleEntry): The entry.
        flags (EntryFlags): Its scanned file flags.
        status (str): Its progress status (``'pending'``, ``'done'``, ``'skipped'``).
        executable (bool): Whether the connected robot runs it.
        selected (bool): Whether the entry slider points at it (drawn with ``'>'``).

    Returns:
        str: The line.
    """
    marker = '>' if selected else ' '
    text = (f"{marker} [{entry.index:02d}] {entry.kind:<3} {entry.bar_id:<4} {entry.robot:<7} "
            f"IK {flags.n_ik}/{flags.n_arm}  TRJ {flags.n_traj}/{flags.n_arm}  {status}")
    if flags.sidecar:
        text += ' [sidecar]'
    if not executable:
        text += ' (other robot)'
    return text


def row_color(status: str, *, executable: bool, selected: bool) -> tuple:
    """Colour of one entry row.

    Selected wins, then done, then "another robot runs it"; a pending entry of
    the connected robot is plain.

    Args:
        status (str): The entry's progress status.
        executable (bool): Whether the connected robot runs it.
        selected (bool): Whether the entry slider points at it.

    Returns:
        tuple: RGBA 0-255.
    """
    if selected:
        return ROW_COLOR_SELECTED
    if status == STATUS_DONE:
        return ROW_COLOR_DONE
    if not executable:
        return ROW_COLOR_OTHER_ROBOT
    return ROW_COLOR_PENDING


def now_line_text(entry: Optional[ScheduleEntry], mv_idx: Optional[int], n_mv: int,
                  mv: Optional[Movement]) -> str:
    """The "where am I" line: loaded entry and movement.

    Example: ``'Now: entry 3 -- H B3 by Alice -- movement 3/4
    B3_H_M2_LM_to_grasp [single_linear] ctrl=joint_tracking'`` (movement numbers
    count from 1).

    Args:
        entry (ScheduleEntry | None): The loaded entry.
        mv_idx (int | None): The movement's index (0-based) in the entry's action.
        n_mv (int): How many movements the action has.
        mv (Movement | None): The movement.

    Returns:
        str: The line.
    """
    if entry is None:
        return 'Now: no entry loaded (pick one, then Load entry)'
    head = f"Now: entry {entry.index} -- {entry.kind} {entry.bar_id} by {entry.robot}"
    if mv is None or mv_idx is None:
        return f"{head} -- no movement loaded"
    try:
        kind = movement_kind(mv).value
    except TypeError:
        kind = type(mv).__name__
    return (f"{head} -- movement {mv_idx + 1}/{n_mv} {mv.movement_id} "
            f"[{kind}] ctrl={mv.controller}")


def visible_row_window(n: int, selected: int, size: int) -> tuple:
    """Which rows of an ``n``-entry list to show so that ``selected`` is in view.

    The window of ``size`` rows is centred on ``selected`` and pushed back inside
    ``[0, n)`` at both ends, e.g. ``(48, 20, 12) -> (14, 26)``.

    Args:
        n (int): Number of entries.
        selected (int): The selected entry.
        size (int): Rows shown at once.

    Returns:
        tuple: ``(lo, hi)``, the half-open range of shown entries.
    """
    if n <= size:
        return 0, n
    lo = max(0, min(selected - size // 2, n - size))
    return lo, lo + size


def knobs_for_assembly_robot(entry: Optional[ScheduleEntry], connected_spec: RobotSpec) -> bool:
    """Whether Cindy's M1 / M2 tuning widgets apply to what is loaded.

    They drive the assembly robot's own jointing cycle, so they are shown only
    on the assembly robot and only while no other robot's entry is loaded.

    Args:
        entry (ScheduleEntry | None): The loaded entry (None before any load).
        connected_spec (RobotSpec): The robot this monitor run drives.

    Returns:
        bool: True to build the widgets.
    """
    if connected_spec.role != ROLE_ASSEMBLY:
        return False
    return entry is None or is_executable_by(entry, connected_spec.name)
