"""As-built record of an assembly run: where every MoCap bar is at every executed step (no ROS, no compas).

While the plan skeleton runs, the monitor measures every bar that carries
MoCap markers (``MoCapMarkers.json``) at the start and at the end of every
executed step, and appends one line per measurement to a JSON-lines file::

    <EXPERIMENT_DATA_DIRECTORY>/bar_action_live_replan_exe/<YYYYMMDD>-asbuilt/as_built_<problem>.jsonl

next to that day's servo records (``<YYYYMMDD>-servoing/``). One file per lab
day and design problem; every robot's monitor (Cindy, Alice, Belle) appends to
the same file, so the whole run reads in time order. The offline page
``data/bar_holding_acc_data/4_assembly_viewer.py`` reads it back.

What is here:

- `as_built_dir`, `as_built_path` -> where the file of a day lives
- `milestone_of(...)` -> which moment of a bar's life a record is: before the
  insert, after the insert, after the retreat, the support robot's hold and
  release, or a bar standing by while another one is built
- `build_as_built_record(...)` -> one record from plain pieces (pure)
- `append_as_built_record`, `read_as_built` -> write one line, read them all

* A record stays raw on purpose: the markers as Motive sent them, the fit, the
* reference poses and the robot state. Every derived number (distances at the
* bar's start / middle / end, bending, the tool0 error) is computed offline from
* them, so the page and any later analysis cannot disagree with each other.
"""

from __future__ import annotations

import json
import os
from datetime import datetime
from typing import Optional

import numpy as np

AS_BUILT_SCHEMA_VERSION = 1
# * The day folder sits beside '<YYYYMMDD>-servoing' (husky_world._save_servoing_data).
AS_BUILT_DIR_SUFFIX = '-asbuilt'
AS_BUILT_FILE_PREFIX = 'as_built_'

# * When a record was taken: just before a step ran, right after it, or on a click.
PHASES = ('start', 'end', 'manual')
# * How the step ended (None for 'start' and 'manual').
OUTCOMES = (None, 'ok', 'cancelled', 'off_target')

# * The moments of a bar's life the page puts first (see `milestone_of`).
MILESTONES = (
    'before_insert',    # J: the transfer is done and its servo has corrected it
    'after_insert',     # J: the insert (with the joint tighten) is done
    'after_fix',        # J of a grounded bar: the manual foundation fix is done
    'before_retreat',   # R: the tools still hold the placed bar
    'after_retreat',    # R: the tools have let go and backed away
    'hold_approach',    # H: the support robot is at its approach pose
    'hold_at_grasp',    # H: its gripper is around the bar, still open
    'hold_grasped',     # H: its gripper has closed on the bar
    'hold_released',    # HR: its gripper has opened
    'hold_retreated',   # HR: it has backed away from the bar
    'bystander',        # any other bar measured while another action runs
)


def as_built_dir(root_dir: str, day: Optional[datetime] = None) -> str:
    """The folder of one day's as-built records.

    Args:
        root_dir (str): The assembly runs' folder
            (``husky_world.BAR_ACTION_LIVE_REPLAN_EXE_DIR``).
        day (datetime | None): The day (today by default).

    Returns:
        str: ``<root_dir>/<YYYYMMDD>-asbuilt``.
    """
    return os.path.join(root_dir, f"{(day or datetime.now()):%Y%m%d}{AS_BUILT_DIR_SUFFIX}")


def as_built_path(root_dir: str, problem: str, day: Optional[datetime] = None) -> str:
    """The as-built file of one day and one design problem.

    Args:
        root_dir (str): The assembly runs' folder.
        problem (str): The design problem name, e.g. ``'261016_2ModulesDemo_YH'``.
        day (datetime | None): The day (today by default).

    Returns:
        str: ``<root_dir>/<YYYYMMDD>-asbuilt/as_built_<problem>.jsonl``.
    """
    return os.path.join(as_built_dir(root_dir, day), f"{AS_BUILT_FILE_PREFIX}{problem}.jsonl")


def json_safe(value):
    """A copy of ``value`` that ``json.dumps`` accepts: numpy turned into plain numbers and lists.

    Handles numpy arrays and scalars, tuples, dicts keyed by numpy integers, and
    objects with a ``to_json()`` method (``BarFit``, ``PoseDeviation``), at any depth.

    Args:
        value: Anything built from dicts, lists, tuples, numbers, strings and numpy.

    Returns:
        The same content in plain Python types.
    """
    if hasattr(value, 'to_json') and callable(value.to_json):
        return json_safe(value.to_json())
    if isinstance(value, dict):
        return {(k.item() if isinstance(k, np.generic) else k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, np.generic):
        return value.item()
    return value


def milestone_of(action_kind: Optional[str], step_kinds: list, step_index: Optional[int], phase: str,
                 is_active_bar: bool) -> Optional[str]:
    """Which moment of a bar's life a record is, for the page to put first.

    The action's own bar gets a milestone at the step ends the user asked for;
    every other bar measured at that moment is a ``'bystander'`` (so a bar that
    nothing touches any more is followed through the rest of the assembly).

    | action | step (kind) | phase | milestone |
    |---|---|---|---|
    | J | the transfer (dual_constrained_free) | end | before_insert |
    | J | the insert (dual_constrained_linear; the tighten runs inside it) | end | after_insert |
    | J | a manual step after the insert (grounded bar: fix the foundation) | end | after_fix |
    | R | the retreat (dual_independent_linear; the ungrasp runs inside it) | start / end | before_retreat / after_retreat |
    | H | the free approach (single_free) | end | hold_approach |
    | H | the linear move to the grasp (single_linear) | end | hold_at_grasp |
    | H | the gripper step after that move (the close) | end | hold_grasped |
    | HR | the gripper step (the open) | end | hold_released |
    | HR | the linear retreat (single_linear) | end | hold_retreated |

    Args:
        action_kind (str | None): ``'J'``, ``'R'``, ``'H'`` or ``'HR'`` (None outside the plan skeleton).
        step_kinds (list): The ``MovementKind`` value (str) of every movement of
            the loaded action, in order (None where unknown).
        step_index (int | None): The movement the record belongs to (None = nothing loaded).
        phase (str): One of `PHASES`.
        is_active_bar (bool): The bar is the one the action is about.

    Returns:
        str | None: One of `MILESTONES`, or None for every other record of the action's bar.
    """
    if not is_active_bar:
        return 'bystander'
    if phase == 'manual' or step_index is None or not 0 <= step_index < len(step_kinds):
        return None
    kind = step_kinds[step_index]
    before = step_kinds[:step_index]
    if action_kind == 'R' and kind == 'dual_independent_linear':
        return 'before_retreat' if phase == 'start' else 'after_retreat'
    if phase != 'end':
        return None
    if action_kind == 'J':
        if kind == 'dual_constrained_free':
            return 'before_insert'
        if kind == 'dual_constrained_linear':
            return 'after_insert'
        if kind == 'manual' and 'dual_constrained_linear' in before:
            return 'after_fix'
    elif action_kind == 'H':
        if kind == 'single_free':
            return 'hold_approach'
        if kind == 'single_linear':
            return 'hold_at_grasp'
        if kind == 'gripper_tool' and 'single_linear' in before:
            return 'hold_grasped'
    elif action_kind == 'HR':
        if kind == 'gripper_tool':
            return 'hold_released'
        if kind == 'single_linear':
            return 'hold_retreated'
    return None


def build_as_built_record(*, problem: str, run_id: str, robot: str, action: Optional[dict],
                          action_file: Optional[str], step: Optional[dict], bars: dict, robot_state: dict,
                          beliefs: dict, instrumented_bars: list, mocap_markers_path: Optional[str],
                          world_from_design=None, now: Optional[datetime] = None) -> dict:
    """One as-built record from plain pieces, ready for `append_as_built_record`.

    Args:
        problem (str): The design problem name.
        run_id (str): The monitor run, e.g. ``'Cindy-20261020T135501'``.
        robot (str): The robot whose monitor took the record.
        action (dict | None): The plan skeleton action (``SkeletonAction.to_dict``
            plus its ``kind``); None outside the plan skeleton.
        action_file (str | None): The loaded action file's name.
        step (dict | None): ``{index, movement_id, kind, phase, outcome,
            arms_off_end_rad, commanded_end}``; None when nothing is loaded
            (a 'record now' click), then the phase is 'manual'.
        bars (dict): ``{rigid body name: entry}``, one entry per measured bar.
        robot_state (dict): The recording robot's base, joints, tool0 and wrench.
        beliefs (dict): The other robots' saved states (``progress.json``) and the holds.
        instrumented_bars (list): Every bar that carries markers (a bar missing
            from ``bars`` could not be measured at that moment).
        mocap_markers_path (str | None): The marker table the fits used.
        world_from_design (array-like | None): The 4x4 that placed the Rhino design on
            the measured floor for this run (``bar_action_io.get_world_from_design``),
            so the reference poses here are in its frame; None = the design as
            exported (every record before 2026-10-23).
        now (datetime | None): The time stamp (now by default).

    Returns:
        dict: The record, in plain JSON types.

    Raises:
        ValueError: An unknown phase or outcome.
    """
    phase = (step or {}).get('phase', 'manual')
    outcome = (step or {}).get('outcome')
    if phase not in PHASES:
        raise ValueError(f"unknown as-built phase {phase!r}; expected one of {PHASES}")
    if outcome not in OUTCOMES:
        raise ValueError(f"unknown as-built outcome {outcome!r}; expected one of {OUTCOMES}")
    now = now or datetime.now()
    return json_safe({
        'schema_version': AS_BUILT_SCHEMA_VERSION,
        'stamp': now.isoformat(timespec='seconds'),
        'stamp_epoch': now.timestamp(),
        'problem': problem,
        'run_id': run_id,
        'robot': robot,
        'action': action,
        'action_file': action_file,
        'step': step if step is not None else {'phase': 'manual', 'outcome': None},
        'instrumented_bars': list(instrumented_bars),
        'mocap_markers_path': mocap_markers_path,
        'world_from_design': world_from_design,
        'bars': bars,
        'robot_state': robot_state,
        'beliefs': beliefs,
    })


def append_as_built_record(path: str, record: dict) -> str:
    """Append one record as one line of the as-built file (the folder is made when missing).

    Args:
        path (str): The file (`as_built_path`).
        record (dict): The record (`build_as_built_record`).

    Returns:
        str: ``path``.
    """
    os.makedirs(os.path.dirname(path), exist_ok=True)
    line = json.dumps(json_safe(record))
    with open(path, 'a') as f:
        f.write(line + '\n')
        f.flush()
    return path


def read_as_built(path: str) -> list:
    """Every record of an as-built file, in file order.

    ! A last line cut short (a crash mid-write, or the drive copy caught while
    ! it syncs) is dropped with a printed note; a broken line anywhere else
    ! means the file was edited or damaged, and raises.

    Args:
        path (str): The file.

    Returns:
        list: The records (dicts); empty when the file does not exist.

    Raises:
        ValueError: A line other than the last cannot be read.
    """
    if not os.path.exists(path):
        return []
    with open(path, 'r') as f:
        lines = [line for line in f.read().split('\n') if line.strip()]
    records = []
    for number, line in enumerate(lines, start=1):
        try:
            records.append(json.loads(line))
        except json.JSONDecodeError as e:
            if number == len(lines):
                print(f"[as-built] {os.path.basename(path)}: its last line is cut short "
                      f"(line {number}); it is skipped.")
                break
            raise ValueError(f"{path}: line {number} is not a JSON record ({e})") from e
    return records
