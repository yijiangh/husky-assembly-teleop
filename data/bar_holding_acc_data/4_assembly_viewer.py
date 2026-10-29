#!/usr/bin/env python3
"""The as-built record of an assembly run as one 3D web page, with a slider over the actions.

The monitor appends one as-built record at the start and the end of every
executed step (``husky_assembly_teleop.as_built_io``): every bar of
``MoCapMarkers.json`` that Motive streams, its fit, where it should be, and the
robot's base, joints and tool0. This script reads one lab day of them and
writes a single offline page (three.js inside, no server):

- a slider over the plan skeleton's actions (J / R / H / HR), in the plan's
  order; at each action every bar that exists then is drawn as a thin line where
  it was designed (placed, held by a support robot, or the one being built), the
  MoCap bars see-through where they were measured (from their first milestone
  on: a bar still on its loading spot is not drawn), their markers sensed (filled)
  and authored (hollow), and the "data by" measurement drawn on them;
- under the slider, that action's records (``before insert``, ``after insert``,
  ``after retreat``, ``grasped``, ...), the tick boxes of the picked record's
  step (the free-move transfer, the MoCap-bar servo feedback, the spiral
  search: used or not, and why; read from the monitor's log, `step_options`)
  and a strip of every action's own bar's error at its last step (or its worst bar, or one bar);
- click a bar: its placement at the start / middle / end of the bar, rotation
  (with its axes drawn), the bar's shape and fit from its balls (ball pairs,
  plate spacing, bow, and a verdict: bad ball, plate off the table, bending, or
  a doubtful fit), the robot's tool0 error and the servo's last iteration, the
  wrist load, and the bar's history since it was placed.

Usage (from ``/home/su/ros2_ws`` with the venv active)::

    python src/husky-assembly-teleop/data/bar_holding_acc_data/4_assembly_viewer.py            # the newest day
    python src/husky-assembly-teleop/data/bar_holding_acc_data/4_assembly_viewer.py 20261021 --open

It writes ``<EXPERIMENT_DATA_DIRECTORY>/bar_action_live_replan_exe/<date>-asbuilt-result/assembly_<date>.html``,
one page per lab day, rebuilt in place on every run. Manual:
``doc/three_robot_assembly_test_manual.md`` section 9.
"""

import argparse
import glob
import json
import math
import os
import re
import sys
import webbrowser
from datetime import datetime

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(HERE))
# * The repo for the package, this folder for viewer_common (shared with 2_session_viewer.py).
for path in (REPO_ROOT, HERE):
    if path not in sys.path:
        sys.path.insert(0, path)

from husky_assembly_teleop import DESIGN_DATA_DIRECTORY, EXPERIMENT_DATA_DIRECTORY, MOCAP_MARKERS_PROBLEM_NAME  # noqa: E402
from husky_assembly_teleop.as_built_io import AS_BUILT_DIR_SUFFIX, AS_BUILT_FILE_PREFIX, read_as_built  # noqa: E402
from husky_assembly_teleop.bar_action_io import bar_id_of_body  # noqa: E402
from husky_assembly_teleop.bar_mocap_fit import (  # noqa: E402
    MOCAP_MARKERS_FILENAME, axis_angle_deg, design_bar_axis_endpoints, fit_bar_from_cache, load_mocap_markers,
    marker_match, mocap_markers_root, pair_geometry, pose_deviation,
)
from husky_assembly_teleop.mocap_experiment import (  # noqa: E402
    _bar_lengths_from_robot_cell, _find_populated_cell_state, env_3dm_for_bar_action, tip_distances,
)
from husky_assembly_teleop.plan_skeleton_io import load_plan_skeleton, problem_root  # noqa: E402
from husky_assembly_teleop.robot_registry import robot_by_name  # noqa: E402
from viewer_common import (  # noqa: E402
    CAMERA_CONE_HALF_ANGLE_DEG, CAMERA_CONE_M, build_cameras, build_environment, file_timestamp, flatten,
    footprint_size, load_kinematics, matrix_from_frame, matrix_from_pose, pack_link_meshes, robot_matrices,
    viewer_script, write_page,
)

# * Where the monitor writes (husky_world.BAR_ACTION_LIVE_REPLAN_EXE_DIR, not imported: it pulls pybullet in).
RUNS_DIR = os.path.join(EXPERIMENT_DATA_DIRECTORY, 'bar_action_live_replan_exe')

# * The lab room (its 'env' solids and the Motive camera rig) when the design's own Rhino file has none:
# * the 2-modules exports carry no 'env' layer. Same Motive frame, so it lines up as well as the room ever
# * does (its placement is approximate, see the bar-holding manual's placement TODO).
LAB_ENV_3DM = os.path.join(os.path.dirname(DESIGN_DATA_DIRECTORY), 'assembly - demo', '260929_phase1_retest.3dm')

# * A servo file belongs to a step when it was saved between the step's start and end records.
SERVO_MATCH_SLACK_S = 2.0
# * Records of one action further apart than this, or from another monitor run, are a redo ("run 2").
REDO_GAP_S = 600.0

# * The monitor's terminal log (scripts/husky/open_terminals.sh writes pane 1 into it). The as-built
# * records do not keep the panel's tick boxes; this log is where the monitor says how each was set
# * and what a step did with it (spiral or straight insert, which servo feedback).
MONITOR_LOG = os.path.expanduser('~/husky_real/monitor.log')
# * The tick boxes that change what a step does, as the panel names them.
OPTION_BOXES = {
    'free_move': 'Transfer as a free move (workaround)',
    'servo_bar': 'Servo feedback: MoCap bar markers',
    'spiral': 'Insert compliant part: SPIRAL search',
}
# * The monitor's log lines about those boxes: (kind, pattern). The groups end with the value.
OPTION_LINES = (
    # The spiral box, said at start-up and on every click.
    ('spiral_box', re.compile(r'insert spiral search: (ON|OFF)')),
    # The servo feedback box: the start-up / click line, and the one each transfer reads at its start.
    ('servo_box', re.compile(r'servo feedback: (MoCap bar|tool0)')),
    # The box as the servo after a grounded bar's insert reads it.
    ('servo_box_insert', re.compile(r'\[insert (\S+)\] servo to .*?, feedback: (MoCap bar|tool0)')),
    # What an insert's compliant part did.
    ('spiral_ran', re.compile(r'\[(\S+)\] SPIRAL search for the compliant part')),
    ('straight_ran', re.compile(r'\[(\S+)\] compliant part: STRAIGHT line')),
    # Why a servo with the box ticked aimed tool0 after all.
    ('servo_no_markers', re.compile(r'\[servo\] feedback: tool0 .*\((\S+ carries no MoCap markers)\)')),
    ('servo_unusable', re.compile(r'the MoCap bar is not usable \((.*?)\); the loop aims tool0')),
    # The transfer's pause with empty tools: only a transfer planned as a free move has it.
    ('mount_pause', re.compile(r'\[transfer (\S+)\] the arms stand at the insertion start with EMPTY tools')),
)
_ANSI = re.compile(r'\x1b\[[0-9;?]*[A-Za-z]')
_ROS_STAMP = re.compile(r'\[(\d{10})\.\d+\]')

# * The verdict on a bar's balls (`ball_findings`). Starting values from the first run (2026-10-20, B20):
# * a plate's two balls measured within +-0.3 mm of the design between records (Motive's noise), one
# * plate read 0.7 mm short in every record (a ball off its spot), and the plates sat 0.6-1.2 mm off the
# * design table the same way in every record (mounting), with a bow that never changed (not bending).
# A plate's two balls further from their design distance than this: one ball is off its spot.
BALL_PAIR_TOL_MM = 0.6
# A plate this far from the design table, the same in every record (it moves less than
# PLATE_STEADY_MM over the day): mounted off its spot, a constant offset, not bending.
PLATE_OFFSET_TOL_MM = 0.5
PLATE_STEADY_MM = 0.5
# An inner plate's bow this far from its usual (the day's mean) in one record: the bar bends there.
BOW_CHANGE_TOL_MM = 0.5
# A typical ball mismatch above this, with no constant offset to explain it: the fit itself is doubtful.
FIT_RMS_TOL_MM = 1.0
# Records of a bar needed before "the same in every record" or "its usual" mean anything.
MIN_RECORDS_FOR_DAY = 3

# * The measurements the page can colour and label by (same shape as 2_session_viewer.METRICS), in the
# * order the card shows them, so the pickers read top to bottom like the card.
METRICS = [
    {'key': 'placement', 'label': 'placement error', 'unit': 'mm', 'high': 5.0,
     'help': 'the worst of the distances at the start, the middle and the end of the bar, between the MoCap bar '
             'and where it should be at that moment (the step target)',
     'formula': 'max over start / middle / end of | MoCap - target |'},
    {'key': 'rotation', 'label': 'rotation error', 'unit': 'deg', 'high': 0.25,
     'help': 'the angle between the MoCap bar\'s centre line and the target\'s',
     'formula': 'arccos | z_MoCap . z_target |'},
    {'key': 'tool0', 'label': 'tool0 vs target', 'unit': 'mm', 'high': 1.5,
     'help': 'where the robot believes its flange is (mocap base + joints) against the authored target of the '
             'step, the worse arm; the part of the error the robot can see',
     'formula': 'max over arms of | FK(base, joints) - target |'},
    {'key': 'servo', 'label': 'servo residual', 'unit': 'mm', 'high': 1.5,
     'help': 'the servo\'s last iteration in this step (tool0 vs its aim), the worse arm',
     'formula': 'last iteration: max over arms of pos_norm_mm'},
    {'key': 'pair_length', 'label': 'plate length error', 'unit': 'mm', 'high': 2.0,
     'help': 'the worst | M1-M2 distance - design | over the bar\'s MoCap plates; large = a ball mislabelled, '
             'hidden or loose (not bending)',
     'formula': 'max over plates of | |M1 - M2| - 48.3 mm |'},
    {'key': 'spacing', 'label': 'spacing', 'unit': 'mm', 'high': 2.0,
     'help': 'the worst | distance between neighbouring plates (the midpoints of their two balls) - design |; the '
             'same in every record = plates mounted off the table, changing = a ball moved',
     'formula': 'max over neighbouring plates of | |mid_a - mid_b| - design |'},
    {'key': 'bend', 'label': 'bending', 'unit': 'mm', 'high': 1.0,
     'help': 'how far the inner MoCap joints sit off the straight line through the outer two, across the bar: '
             'the bow of the bar (Motive noise ~0.5 mm)',
     'formula': 'max over inner joints of | shift_xy - line through the outer joints\' shifts |'},
    {'key': 'fit_residual', 'label': 'fit residual', 'unit': 'mm', 'high': 1.5,
     'help': 'the worst distance between a sensed ball and its design ball once the bar is fitted',
     'formula': 'max over matched balls of | sensed - fitted design |'},
    {'key': 'load', 'label': 'load imbalance', 'unit': 'N', 'high': 2.0,
     'help': 'the gap between the two wrists\' force magnitudes (Cindy only)',
     'formula': '| |F_left| - |F_right| |'},
]
EXTRA_FORMULAS = [
    ('MoCap vs FK', 'MoCap bar against tool0 FK composed with the grasp: the part of the error the robot cannot '
                    'see (base calibration, arm, TCP, the bar in the jaws)'),
    ('reference', 'the step target: the seat for a resting bar; for a held bar the authored flange target of '
                  'the step\'s end composed with the grasp (never FK)'),
]

# * Short names for the records' pills.
MILESTONE_LABEL = {
    'before_insert': 'before insert', 'after_insert': 'after insert', 'after_fix': 'after fix',
    'before_retreat': 'before retreat', 'after_retreat': 'after retreat', 'hold_approach': 'approach',
    'hold_at_grasp': 'at grasp', 'hold_grasped': 'grasped', 'hold_released': 'released',
    'hold_retreated': 'backed off',
}


def finite(value):
    """A copy of ``value`` with NaN and infinities turned into None.

    ! ``json.dumps`` writes NaN as a bare ``NaN``, which the browser's
    ! ``JSON.parse`` refuses: one undefined number would leave the page blank.

    Args:
        value: Dicts, lists, numbers and strings (numpy included).

    Returns:
        The same content, every number finite or None.
    """
    if isinstance(value, dict):
        return {k: finite(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite(v) for v in value]
    if isinstance(value, np.ndarray):
        return finite(value.tolist())
    if isinstance(value, np.generic):
        return finite(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


# * ---------------------------------------------------------------------------
# * Finding the day's files
# * ---------------------------------------------------------------------------

def latest_as_built_day(runs_dir: str = RUNS_DIR) -> str:
    """The newest day that has as-built records.

    Args:
        runs_dir (str): The assembly runs' folder.

    Returns:
        str: ``YYYYMMDD``.

    Raises:
        SystemExit: No ``<date>-asbuilt`` folder exists.
    """
    days = sorted(name[:-len(AS_BUILT_DIR_SUFFIX)] for name in os.listdir(runs_dir)
                  if name.endswith(AS_BUILT_DIR_SUFFIX) and os.path.isdir(os.path.join(runs_dir, name)))
    if not days:
        raise SystemExit(f"no '<date>{AS_BUILT_DIR_SUFFIX}' folder under {runs_dir}: nothing was recorded yet.")
    return days[-1]


def as_built_file(day: str, problem: str = None, runs_dir: str = RUNS_DIR) -> str:
    """The as-built file of a day (of a problem, or the only one there).

    Args:
        day (str): ``YYYYMMDD``.
        problem (str | None): The design problem; None = the day's only file (or the newest).
        runs_dir (str): The assembly runs' folder.

    Returns:
        str: The ``.jsonl`` path.
    """
    folder = os.path.join(runs_dir, f"{day}{AS_BUILT_DIR_SUFFIX}")
    if problem:
        return os.path.join(folder, f"{AS_BUILT_FILE_PREFIX}{problem}.jsonl")
    files = sorted(glob.glob(os.path.join(folder, f"{AS_BUILT_FILE_PREFIX}*.jsonl")), key=os.path.getmtime)
    if not files:
        raise SystemExit(f"no as-built file in {folder}.")
    if len(files) > 1:
        print(f"[assembly] {len(files)} problems recorded on {day}; showing the newest "
              f"({os.path.basename(files[-1])}). Pass --problem to pick another.")
    return files[-1]


def servo_files(day: str, runs_dir: str = RUNS_DIR) -> list:
    """The day's servo records, each with the time in its name.

    Args:
        day (str): ``YYYYMMDD``.
        runs_dir (str): The assembly runs' folder.

    Returns:
        list: ``[(epoch seconds, path)]`` in time order.
    """
    out = []
    for path in glob.glob(os.path.join(runs_dir, f"{day}-servoing", '*_servoing_data_*.json')):
        stamp = file_timestamp(path)
        if stamp is not None:
            out.append((stamp.timestamp(), path))
    return sorted(out)


def option_events(day: str, log_path: str = MONITOR_LOG):
    """The day's lines about the tick boxes in the monitor's log, each with its time.

    * A line without a ROS stamp (a plain print) takes the time of the last
    * stamped line before it, the way the terminal shows them.

    Args:
        day (str): ``YYYYMMDD``.
        log_path (str): The monitor's terminal log (`MONITOR_LOG`).

    Returns:
        list | None: ``[(epoch seconds, kind, groups)]`` in time order (kinds and
        groups as in `OPTION_LINES`); None when there is no log.
    """
    if not log_path or not os.path.exists(log_path):
        return None
    with open(log_path, 'rb') as f:
        text = f.read().decode('utf-8', 'replace')
    events, last = [], None
    for line in text.split('\n'):
        line = _ANSI.sub('', line)
        stamp = _ROS_STAMP.search(line)
        if stamp:
            last = int(stamp.group(1))
        if last is None or datetime.fromtimestamp(last).strftime('%Y%m%d') != day:
            continue
        for kind, pattern in OPTION_LINES:
            hit = pattern.search(line)
            if hit:
                events.append((float(last), kind, hit.groups()))
                break
    return events


def markers_root(problem_dir: str) -> str:
    """The folder whose ``MoCapMarkers.json`` the monitor used: the problem's own, else the borrowed one.

    Args:
        problem_dir (str): The recorded design problem's folder.

    Returns:
        str: ``problem_dir``, or ``MOCAP_MARKERS_PROBLEM_NAME``'s folder when the problem has no file.
    """
    return mocap_markers_root(problem_dir, problem_root(MOCAP_MARKERS_PROBLEM_NAME, DESIGN_DATA_DIRECTORY))


def day_markers_root(day: str, problem_dir: str, runs_dir: str = RUNS_DIR) -> str:
    """The folder of the marker table for one lab day: the day's own, else the one the monitor used.

    * A day folder (``<day>-asbuilt``) holds its own ``MoCapMarkers.json`` when the physical bars
    * differed from the export that day. 28 Oct: B34 still carried J34-56_mocap, while the 261023
    * export had moved that plate to M34-T20-0_mocap, so the monitor's fits of B34 failed.

    Args:
        day (str): ``YYYYMMDD``.
        problem_dir (str): The recorded design problem's folder.
        runs_dir (str): The assembly runs' folder.

    Returns:
        str: The day folder when it holds the file, else `markers_root`.
    """
    folder = os.path.join(runs_dir, f"{day}{AS_BUILT_DIR_SUFFIX}")
    if os.path.isfile(os.path.join(folder, MOCAP_MARKERS_FILENAME)):
        return folder
    return markers_root(problem_dir)


def refit_on_new_layout(entry: dict, design) -> dict:
    """Fit a recorded bar again from its recorded balls when its layout differs from the monitor's.

    Args:
        entry (dict): ``record['bars'][body name]``, with the monitor's ``design_markers`` and fit.
        design (BarMarkers | None): The bar's markers in the table the page uses.

    Returns:
        dict: ``entry`` itself when the layouts agree (or nothing can be compared), else a copy
        with ``bar_fit`` and ``design_markers`` from ``design`` and ``refit`` True.
    """
    used = [m.get('bar_m') for m in entry.get('design_markers') or []]
    if design is None or not used:
        return entry
    if len(used) == len(design.markers) and np.allclose(np.asarray(used, dtype=float), design.points(), atol=1e-6):
        return entry
    snapshot = entry.get(entry.get('markerset_key')) or {}
    fit = fit_bar_from_cache(snapshot, design)
    authored = [{'name': m.name, 'joint': m.joint, 'label': m.label, 'bar_m': list(m.bar_m)} for m in design.markers]
    return dict(entry, bar_fit=fit.to_json(), design_markers=authored, refit=True)


def resolve_env_3dm(problem_dir: str, markers, override: str = None):
    """The Rhino file whose ``env`` layer is the cell: the problem's own, else the marker table's source.

    Args:
        problem_dir (str): The design problem folder.
        markers (MoCapMarkerSet | None): The marker table (its ``source`` names the Rhino file it came from).
        override (str | None): ``--env-3dm``.

    Returns:
        str | None: The path, or None (no environment drawn).
    """
    actions = sorted(glob.glob(os.path.join(problem_dir, 'BarActions', '*.json')))
    path = env_3dm_for_bar_action(actions[0], override) if actions else override
    if (not path or not os.path.exists(path)) and markers is not None and markers.source:
        path = os.path.join(os.path.dirname(DESIGN_DATA_DIRECTORY), 'assembly - demo', markers.source)
    if not path or not os.path.exists(path):
        print("[assembly] no Rhino file found for the environment; pass --env-3dm to draw it.")
        return None
    if os.path.exists(path + '.rhl'):
        print(f"[assembly] ! {os.path.basename(path)} is open in Rhino (a .rhl lock beside it): "
              "its meshes may be from a half-saved state.")
    return path


# * ---------------------------------------------------------------------------
# * What exists at each action (from the plan skeleton)
# * ---------------------------------------------------------------------------

def timeline(skeleton) -> list:
    """Which bars exist at each action, and who holds them.

    * A bar exists from its J action on. It is held by Cindy between its J and
    * its R (the insert is done, the tools have not let go), and by a support
    * robot during its hold window; the action's own bar is the active one.

    Args:
        skeleton (PlanSkeleton): The loaded ``ActionSchedule.json``.

    Returns:
        list: Per action ``{index, action_id, kind, bar_id, robot, exists [bar ids],
        held {bar id: robot}}``.
    """
    j_at = {a.bar_id: a.index for a in skeleton.actions if a.kind == 'J'}
    r_at = {a.bar_id: a.index for a in skeleton.actions if a.kind == 'R'}
    for bar_id in skeleton.assembly_seq:
        if bar_id not in j_at:
            print(f"[assembly] ! {bar_id} has no J action in the plan skeleton: it is never drawn.")
    out = []
    for action in skeleton.actions:
        k = action.index
        exists = [b for b in skeleton.assembly_seq if b in j_at and j_at[b] <= k]
        held = {b: 'Cindy' for b in exists if j_at[b] < k < r_at.get(b, len(skeleton.actions))}
        held.update({bar_id: robot for robot, bar_id in skeleton.robots_holding_at(action).items()})
        out.append({'index': k, 'action_id': action.action_id, 'kind': action.kind, 'bar_id': action.bar_id,
                    'robot': action.robot, 'exists': exists, 'held': held})
    return out


def assembled_bars(problem_dir: str, world_from_design=None) -> dict:
    """Every bar's assembled (seat) pose and length.

    Args:
        problem_dir (str): The design problem folder.
        world_from_design (array-like | None): The 4x4 a record's monitor placed the
            Rhino design with (the record's ``world_from_design``: the design on the
            measured floor), so the seats compare with that record's poses; None =
            as exported (records before 2026-10-23).

    Returns:
        dict: ``{bar id: {'T': 4x4, 'length': m, 'ends': [[start], [end]]}}``.
    """
    state = _find_populated_cell_state(problem_dir) or {}
    lengths = _bar_lengths_from_robot_cell(problem_dir)
    placement = np.eye(4) if world_from_design is None else np.asarray(world_from_design, dtype=float)
    out = {}
    for name, frame in (state.get('bar_frames') or {}).items():
        if name not in lengths:
            print(f"[assembly] ! {name}: no length in RobotCell.json; it is not drawn.")
            continue
        T = placement @ matrix_from_frame(frame)
        out[bar_id_of_body(name)] = {'T': T, 'length': lengths[name],
                                     'ends': design_bar_axis_endpoints(T, lengths[name]).tolist()}
    return out


# * ---------------------------------------------------------------------------
# * Numbers of one record
# * ---------------------------------------------------------------------------

def _mat(value):
    """A recorded 4x4 as an array.

    Args:
        value (list | None): A nested 4x4 list from a record.

    Returns:
        np.ndarray | None: The matrix; None when the record had none.
    """
    return None if value is None else np.asarray(value, dtype=float)


def _pose_mat(pose):
    """A recorded pose as a 4x4.

    Args:
        pose (list | None): ``[pos, quat_xyzw]`` from a record.

    Returns:
        np.ndarray | None: The matrix; None when the record had none.
    """
    return None if not pose else matrix_from_pose(pose[0], pose[1])


def _three_places(T_a, T_b, length: float) -> dict:
    """The distances between two bars at the start, the middle and the end (``tip_distances``), in mm.

    Args:
        T_a (np.ndarray): 4x4 of the measured bar.
        T_b (np.ndarray): 4x4 of the bar it is compared with.
        length (float): The bar length, metres.

    Returns:
        dict: ``{start, middle, end, worst, mean, spread}`` in mm.
    """
    a, b = design_bar_axis_endpoints(T_a, length), design_bar_axis_endpoints(T_b, length)
    tips = tip_distances({'fit_start': a[0], 'fit_end': a[1], 'goal_start': b[0], 'goal_end': b[1]})
    return {key[:-2]: value * 1000 for key, value in tips.items()}


def _deviation(T_a, T_b):
    """How far one pose is from another (``pose_deviation``), as plain numbers.

    Args:
        T_a (np.ndarray | None): 4x4 of the measured pose.
        T_b (np.ndarray | None): 4x4 of the pose it is compared with.

    Returns:
        dict | None: ``{dpos_mm, dang_deg, dpos_bar_mm, rotvec_bar_deg}``; None when a pose is missing.
    """
    if T_a is None or T_b is None:
        return None
    dev = pose_deviation(T_a, T_b)
    return {'dpos_mm': dev.dpos_mm, 'dang_deg': dev.dang_deg, 'dpos_bar_mm': np.asarray(dev.dpos_bar_mm).tolist(),
            'rotvec_bar_deg': np.asarray(dev.rotvec_bar_deg).tolist()}


def _side_of_link(link: str) -> str:
    """The arm side a held link belongs to.

    Args:
        link (str): e.g. ``'left_ur_arm_tool0'``.

    Returns:
        str: ``'left'``, ``'right'``, or ``'arm'`` on a single-arm robot.
    """
    for side in ('left', 'right'):
        if side in (link or ''):
            return side
    return 'arm'


def shape_bar(entry: dict, record: dict, seat: dict, design) -> dict:
    """One measured bar of a record, with every number the page shows.

    Args:
        entry (dict): ``record['bars'][body name]``.
        record (dict): The whole record (the robot state, for the FK bar).
        seat (dict): ``assembled_bars()[bar id]``.
        design (BarMarkers | None): The bar's design markers.

    Returns:
        dict: The bar as the page draws it (see the module's ``DATA`` note in ``collect_assembly``).
    """
    body = entry.get('markerset_key')
    fit = entry.get('bar_fit') or {}
    snapshot = entry.get(body) or {}
    length = seat['length']
    T_fit = _mat(fit.get('T_world_from_bar')) if fit.get('ok') else None
    T_ref, T_seat = _mat(entry.get('T_reference')), seat['T']
    names = design.names() if design is not None else []
    out = {
        'bar': entry.get('bar_id'), 'body': body, 'ok': bool(fit.get('ok')), 'reason': fit.get('reason') or '',
        'source': entry.get('marker_source'), 'age_s': entry.get('marker_age_s'),
        'milestone': entry.get('milestone'), 'held_by': entry.get('held_by'),
        'reference_kind': entry.get('reference_kind'),
        'fitted': design_bar_axis_endpoints(T_fit, length).tolist() if T_fit is not None else None,
        'reference': design_bar_axis_endpoints(T_ref, length).tolist() if T_ref is not None else None,
        'seat': seat['ends'],
        'n_matched': fit.get('n_matched'), 'n_design': fit.get('n_design'),
        'fit_rms': None if fit.get('rms_m') is None else fit['rms_m'] * 1000,
        'fit_residual': None if fit.get('max_residual_m') is None else fit['max_residual_m'] * 1000,
        'ambiguous': fit.get('ambiguous'), 'tracked': fit.get('tracked'),
        'missing': [names[int(d)] for d in fit.get('missing_design') or [] if int(d) < len(names)],
        'strays': len(fit.get('unmatched_sensed') or []),
    }
    if design is not None:
        sensed, authored = marker_match(fit, snapshot, design, T_ref if T_ref is not None else T_seat)
        out['markers_sensed'], out['markers_authored'] = sensed, authored
    if T_fit is None:
        return out

    # * Where the bar is against where it should be now (reference) and against its seat.
    if T_ref is not None:
        tips = _three_places(T_fit, T_ref, length)
        out.update({f'tip_{k}': v for k, v in tips.items() if k in ('start', 'middle', 'end')})
        out['placement'] = tips['worst']
        out['rotation'] = axis_angle_deg(T_fit, T_ref)
        out['vs_reference'] = _deviation(T_fit, T_ref)
    seat_tips = _three_places(T_fit, T_seat, length)
    out.update({f'seat_{k}': v for k, v in seat_tips.items() if k in ('start', 'middle', 'end')})
    out['placement_seat'] = seat_tips['worst']
    out['rotation_seat'] = axis_angle_deg(T_fit, T_seat)

    # * The two frames the page draws as x / y / z axes (16 numbers each, three.js order), and the
    # * measured bar's turn from the drawn reference, split about the reference's x / y / z (degrees;
    # * the reference is the step target, else the seat).
    T_axes = T_ref if T_ref is not None else T_seat
    out['frame_fit'], out['frame_ref'] = flatten(T_fit), flatten(T_axes)
    out['turn_ref_deg'] = _deviation(T_fit, T_axes)['rotvec_bar_deg']

    # * The plates: length, shift across the bar, bending.
    if design is not None:
        pairs = pair_geometry(fit, snapshot, design)
        out['pairs'] = {'joints': pairs['joints'], 'spacing': pairs['spacing']}
        out['bend'], out['pair_length'] = pairs['bend_mm'], pairs['worst_length_err_mm']
        gaps = [abs(gap['err_mm']) for gap in pairs['spacing']]
        out['spacing'] = max(gaps) if gaps else None

    # * Where the ROBOT believes the bar is (tool0 FK with the grasp), when an arm holds it.
    grasp = _mat(entry.get('T_link_from_bar'))
    side = _side_of_link(entry.get('held_by'))
    fk_pose = ((((record.get('robot_state') or {}).get('arms') or {}).get(side) or {}).get('tool0_fk'))
    if grasp is not None and fk_pose:
        T_fk = _pose_mat(fk_pose) @ grasp
        out['fk'] = {'side': side, 'fk_vs_reference': _deviation(T_fk, T_ref), 'mocap_vs_reference':
                     _deviation(T_fit, T_ref), 'mocap_vs_fk': _deviation(T_fit, T_fk),
                     'line': design_bar_axis_endpoints(T_fk, length).tolist()}
    return out


def tool0_numbers(record: dict) -> dict:
    """Per arm: where tool0 is by FK against the step's authored target.

    Args:
        record (dict): An as-built record.

    Returns:
        dict: ``{side: {'fk_vs_target': {dpos_mm, dang_deg} | None, 'pos': [x, y, z] | None}}``; ``pos`` is
        where FK puts the flange, so the page writes the arm's numbers there.
    """
    out = {}
    for side, arm in (((record.get('robot_state') or {}).get('arms')) or {}).items():
        fk = arm.get('tool0_fk')
        out[side] = {'fk_vs_target': _deviation(_pose_mat(fk), _pose_mat(arm.get('tool0_target_end'))),
                     'pos': [float(v) for v in fk[0]] if fk else None}
    return out


def servo_numbers(path: str) -> dict:
    """The last iteration of a servo record.

    Args:
        path (str): ``<label>_servoing_data_<ts>.json``.

    Returns:
        dict: ``{file, movement_id, feedback, planner_mode, iterations, arms {side: {pos_norm_mm,
        rot_max_deg}}, bar {mocap_vs_design, fk_vs_design, mocap_vs_fk} | None, worst}``.
    """
    with open(path, 'r') as f:
        data = json.load(f)
    readings = data.get('servoing_data') or []
    last = readings[-1] if readings else {}
    arms = {side: {'pos_norm_mm': v.get('pos_norm_mm'),
                   'rot_max_deg': max((abs(a) for a in v.get('rot_err_deg') or [0.0]), default=0.0)}
            for side, v in (last.get('tool0') or {}).items()}
    bar = last.get('bar') or None
    return {'file': os.path.basename(path), 'movement_id': data.get('movement_id'),
            'feedback': data.get('servo_feedback'), 'planner_mode': data.get('planner_mode'),
            'iterations': len(readings), 'arms': arms,
            'bar': None if not bar else {k: bar.get(k) for k in ('mocap_vs_design', 'fk_vs_design', 'mocap_vs_fk')},
            'worst': max((a['pos_norm_mm'] or 0.0 for a in arms.values()), default=None)}


def load_numbers(record: dict):
    """The wrists' force magnitudes and their gap (Cindy only), and what they are relative to.

    Args:
        record (dict): An as-built record.

    Returns:
        dict | None: ``{left_N, right_N, imbalance, zero}``; None without a reading. ``zero`` is
        ``{'at', 'note'}`` of the sensors' last zero, None in records from before it was saved.
    """
    ft = (record.get('robot_state') or {}).get('tool_ft')
    if not ft or ft.get('left_force_N') is None or ft.get('right_force_N') is None:
        return None
    return {'left_N': ft['left_force_N'], 'right_N': ft['right_force_N'],
            'imbalance': abs(ft['left_force_N'] - ft['right_force_N']), 'zero': ft.get('zero')}


def step_options(events, action_kind: str, step: dict, t0: float, t1: float, servo) -> list:
    """Each tick box that could change one executed step: how it was set, and whether the step used it.

    * What can act where: the free-move workaround on a transfer, the servo
    * feedback on the servo that ends a transfer or a grounded bar's insert (a
    * support robot's servo aims tool0 only), the spiral on a screwed bar's
    * insert (a grounded bar's insert is rigid, and a retreat never spirals). A
    * box ticked at a step it cannot act on is listed too, as not used, because
    * the monitor still prints it there (e.g. '[Exec] ... SPIRAL search (box
    * ticked)' before a retreat).

    Args:
        events (list | None): `option_events` of the day (None = no monitor log).
        action_kind (str): ``'J'``, ``'R'``, ``'H'`` or ``'HR'``.
        step (dict): The record's ``step`` (``movement_id``, ``kind``).
        t0 (float): The step's start record (epoch seconds).
        t1 (float): Its end record (``t0`` when it has none).
        servo (dict | None): `servo_numbers` of the servo that ran inside the step.

    Returns:
        list: ``[{key, label, box (True / False / None = unknown), applies, used (True / False / None),
        why, shown}]``, one per box in `OPTION_BOXES`; ``shown``: the page lists it at this step (it can
        act here, the step is an insert, or the box was ticked).
    """
    mv, kind = step.get('movement_id') or '', step.get('kind')
    inside = [e for e in events or [] if t0 - SERVO_MATCH_SLACK_S <= e[0] <= t1 + SERVO_MATCH_SLACK_S]

    def clicked(kinds: tuple, box, after: float) -> str:
        """A click on the box during the step, after the step had read it: ' (ticked at HH:MM:SS ...)'."""
        for e_t, e_kind, groups in inside:
            state = groups[-1] in ('ON', 'MoCap bar')
            if e_kind in kinds and e_t > after and box is not None and state != box:
                return (f" (box {'ticked' if state else 'unticked'} at {datetime.fromtimestamp(e_t):%H:%M:%S}, "
                        "after the step had read it)")
        return ''

    def box_at(kinds: tuple, t: float):
        """The box as its latest line at or before ``t`` says (None = no line yet)."""
        state = None
        for e_t, e_kind, groups in events or []:
            if e_t > t:
                break
            if e_kind in kinds:
                state = groups[-1] in ('ON', 'MoCap bar')
        return state

    def first(kinds: tuple, movement: str = None):
        """The step's first line of these kinds (for this movement, when one is given): ``(time, groups)``."""
        return next(((t, g) for t, k, g in inside if k in kinds and (movement is None or g[0] == movement)), None)

    is_transfer = action_kind == 'J' and kind == 'dual_constrained_free'
    is_insert = action_kind == 'J' and kind == 'dual_constrained_linear'
    out = []

    # * The free-move workaround: only its transfer pauses with empty tools for the mount.
    used = None if events is None else first(('mount_pause',), mv) is not None
    out.append({'key': 'free_move', 'box': used if is_transfer else None, 'applies': is_transfer,
                'used': used if is_transfer else None,
                'why': ('planned from the live arms without the bar; the bar is mounted by hand at the '
                        'insertion start' if used else 'the transfer carried a bar already in the tools')
                if is_transfer and used is not None else '', 'shown': is_transfer})

    # * The servo feedback: the transfer reads the box at its start, the servo after a grounded bar's
    # * insert reads it again ('[insert ...] servo to ...'); elsewhere the box as it stood at the start.
    said = (first(('servo_box',)) if is_transfer else first(('servo_box_insert',), mv) if is_insert else None)
    box = (said[1][-1] == 'MoCap bar') if said else box_at(('servo_box', 'servo_box_insert'), t0)
    runs_servo = servo is not None
    used = runs_servo and servo.get('feedback') == 'mocap_bar'
    reason = first(('servo_no_markers', 'servo_unusable'))
    if used:
        why = 'the servo aimed the bar\'s MoCap fit at its Rhino pose'
    elif runs_servo and action_kind == 'H':
        why = 'a support robot\'s servo aims its flange (tool0) only'
    elif runs_servo:
        why = ('the servo aimed tool0' if box else 'the servo aimed the flanges (tool0) at their Rhino targets') \
            + (f": {reason[1][0]}" if reason else '')
    else:
        why = 'no servo runs in this step'
    if said and runs_servo:
        why += clicked(('servo_box',), box, said[0])
    out.append({'key': 'servo_bar', 'box': box, 'applies': runs_servo, 'used': used if runs_servo else False,
                'why': why, 'shown': runs_servo or bool(box)})

    # * The spiral: said at every screwed bar's insert (spiral or straight); a grounded insert is rigid.
    if is_insert and first(('spiral_ran',), mv):
        spiral = {'box': True, 'applies': True, 'used': True,
                  'why': 'the compliant part circled the bar about the insertion line while the joint tightened'}
    elif is_insert and first(('straight_ran',), mv):
        spiral = {'box': False, 'applies': True, 'used': False, 'why': 'the compliant part drove a straight line'}
    else:
        box = None if events is None else bool(box_at(('spiral_box',), t0))
        why = ('this insert ran rigid all the way (a grounded bar): the box does not act on it' if is_insert
               else 'the box acts on a screwed bar\'s insert only')
        spiral = {'box': box, 'applies': False, 'used': False if events is not None else None, 'why': why}
    out.append({'key': 'spiral', **spiral, 'shown': is_insert or bool(spiral['box'])})
    for option in out:
        option['label'] = OPTION_BOXES[option['key']]
    return out


# * ---------------------------------------------------------------------------
# * A bar through the day: its history, and the verdict on its balls
# * ---------------------------------------------------------------------------

def add_history(history: dict, bar: dict, entry: dict) -> bool:
    """Add one measurement of a bar to its history, unless the bar was not placed yet.

    * The records before a bar's first milestone measure it somewhere else: on
    * its loading spot at the start of its J action, or standing by while other
    * bars are built. Kept, they sit metres from the seat and flatten the
    * history's plot, so they are only counted.

    Args:
        history (dict): ``{bar id: {'entries': [...], 'skipped': n}}``, filled in place.
        bar (dict): The bar as `shape_bar` made it (its fit must be ok).
        entry (dict): What the page lists for this measurement.

    Returns:
        bool: True when the entry was kept.
    """
    slot = history.setdefault(bar['bar'], {'entries': [], 'skipped': 0})
    # ! A cancelled step's milestone does not count: B20's first transfer on 2026-10-23 was cancelled
    # ! with the bar still on its loading spot, 880 mm from its seat.
    if not slot['entries'] and (bar.get('milestone') in (None, 'bystander')
                                or entry.get('outcome') not in (None, 'ok')):
        slot['skipped'] += 1
        return False
    slot['entries'].append(entry)
    return True


def ball_check(pairs_of_day: list) -> dict:
    """One bar's balls over a day: what stays the same between records and what wanders.

    What a plate's numbers mean, read over the day:

    * its two balls' distance (``length_err_mm``) wandering between records is
      Motive's noise; off by the same amount in every record is a ball sitting
      off its spot (or a wrong table entry);
    * its shift from where the rigid fit puts it (``shift_mm``), the same in every
      record, is the plate mounted off the design table: a constant offset that
      ends up in the fit's rms, not bending;
    * an inner plate's bow (``bow_mm``) that changes between records is the bar
      bending (its load changed); a bow that never changes is the same mounting
      offset seen from the straight line.

    Args:
        pairs_of_day (list): ``shape_bar(...)['pairs']`` of every record the bar is in
            its history (placed), in time order.

    Returns:
        dict: ``{'n_records', 'noise_mm', 'plates': {joint: {'n', 'pair_err_min', 'pair_err_max',
        'shift_mean' [3], 'shift_range', 'bow_mean' [2] | None, 'bow_range' | None}}}``. ``noise_mm``
        is half the widest range of a plate's ball distance (None with fewer than two records).
    """
    by_joint = {}
    for pairs in pairs_of_day:
        for j in (pairs or {}).get('joints') or []:
            by_joint.setdefault(j['joint'], []).append(j)
    plates = {}
    for joint, rows in by_joint.items():
        errs = [r['length_err_mm'] for r in rows if r['length_err_mm'] is not None]
        shifts = np.asarray([r['shift_mm'] for r in rows if r['shift_mm'] is not None], dtype=float).reshape(-1, 3)
        bows = np.asarray([r['bow_mm'] for r in rows if r.get('bow_mm') is not None], dtype=float).reshape(-1, 2)
        plates[joint] = {
            'n': len(rows),
            'pair_err_min': min(errs) if errs else None, 'pair_err_max': max(errs) if errs else None,
            'shift_mean': shifts.mean(axis=0).tolist() if len(shifts) else None,
            # The widest a coordinate of the shift moved over the day.
            'shift_range': float(np.ptp(shifts, axis=0).max()) if len(shifts) else None,
            'bow_mean': bows.mean(axis=0).tolist() if len(bows) else None,
            'bow_range': float(np.ptp(bows, axis=0).max()) if len(bows) else None,
        }
    ranges = [p['pair_err_max'] - p['pair_err_min'] for p in plates.values() if p['pair_err_min'] is not None]
    n = len(pairs_of_day)
    return {'n_records': n, 'noise_mm': max(ranges) / 2 if n >= 2 and ranges else None, 'plates': plates}


def _plate_name(joint: str) -> str:
    """A MoCap joint's short name, e.g. ``joint_J20-22_mocap`` -> ``J20-22``."""
    return joint.replace('joint_', '').replace('_mocap', '')


def ball_findings(bar: dict, day: dict = None) -> list:
    """The verdict on one measured bar, in plain words: a bad ball, a plate off the table, bending, a doubtful fit.

    Args:
        bar (dict): The bar as `shape_bar` made it.
        day (dict | None): `ball_check` of this bar over the day (None: judged from this record alone).

    Returns:
        list: One line per finding; one "all good" line when there is none; empty when the bar was not fitted.
    """
    if not bar.get('ok') or not bar.get('pairs'):
        return []
    lines = []
    joints = bar['pairs']['joints']
    plates = (day or {}).get('plates') or {}
    steady = day is not None and day['n_records'] >= MIN_RECORDS_FOR_DAY

    # * 1. A ball off its spot: its plate's two balls are not 48.3 mm apart (bending cannot change that).
    for j in joints:
        name = _plate_name(j['joint'])
        if j['length_mm'] is None:
            lines.append(f"plate {name}: only {j['n_seen']} of its 2 balls seen (hidden or not streamed)")
            continue
        if abs(j['length_err_mm']) <= BALL_PAIR_TOL_MM:
            continue
        p = plates.get(j['joint']) or {}
        always = (steady and p.get('pair_err_min') is not None
                  and min(abs(p['pair_err_min']), abs(p['pair_err_max'])) > BALL_PAIR_TOL_MM
                  and p['pair_err_min'] * p['pair_err_max'] > 0)
        design = j['length_mm'] - j['length_err_mm']
        lines.append(f"bad ball on plate {name}: its two balls are {j['length_mm']:.1f} mm apart, not "
                     f"{design:.1f}" + (" -- in every record this day: a ball mounted off its spot or a wrong "
                                        "table entry" if always else ": a ball loose, half hidden or mislabelled"))

    # * 2. Bending: an inner plate's bow away from its usual one (needs the day).
    if steady:
        for j in joints:
            p = plates.get(j['joint']) or {}
            if j.get('bow_mm') is None or p.get('bow_mean') is None:
                continue
            change = float(np.linalg.norm(np.asarray(j['bow_mm']) - np.asarray(p['bow_mean'])))
            if change > BOW_CHANGE_TOL_MM:
                lines.append(f"bending at plate {_plate_name(j['joint'])}: its bow is {change:.1f} mm away from its "
                             f"usual this day (the bar bends under a load here, or a ball moved)")

    # * 3. Constant offsets: plates off the design table the same way in every record (needs the day).
    offsets = []
    if steady:
        for joint, p in plates.items():
            if p['shift_mean'] is None or p['shift_range'] is None:
                continue
            size = float(np.linalg.norm(p['shift_mean']))
            if size > PLATE_OFFSET_TOL_MM and p['shift_range'] < PLATE_STEADY_MM:
                offsets.append(f"{_plate_name(joint)} {size:.1f}")
    if offsets:
        lines.append(f"plates off the design table by the same amount in every record (mm): {', '.join(offsets)} -- "
                     f"mounted off their table spots, a constant offset, not bending")

    # * 4. The fit itself: an assignment that fits as well, or a large mismatch nothing above explains.
    if bar.get('ambiguous'):
        lines.append("doubtful fit: another assignment of the balls fits as well")
    rms = bar.get('fit_rms')
    if rms is not None and rms > FIT_RMS_TOL_MM:
        lines.append(f"typical ball mismatch {rms:.1f} mm (above {FIT_RMS_TOL_MM:.1f}): "
                     + ("mostly the constant plate offsets above" if offsets
                        else "nothing above explains it: the fit itself is doubtful"))
    return lines or ["balls good, bar straight, fit tight"]


# * ---------------------------------------------------------------------------
# * Robots
# * ---------------------------------------------------------------------------

class RobotModels:
    """The robot meshes, loaded once per model, and each pose's link matrices."""

    def __init__(self):
        """Nothing is loaded until a robot is drawn."""
        self.models = {}      # 'dual' / 'single' -> {kinematics, geometry, meshes, footprint}

    def _model(self, robot: str) -> str:
        """Load a robot's model the first time it is drawn.

        Args:
            robot (str): Short robot name.

        Returns:
            str: ``'dual'`` or ``'single'``, the key of its model.
        """
        spec = robot_by_name(robot)
        key = 'dual' if spec.dual_arm else 'single'
        if key not in self.models:
            kinematics, geometry = load_kinematics(spec.urdf_path)
            self.models[key] = {'kinematics': kinematics, 'geometry': geometry,
                                'meshes': pack_link_meshes(geometry), 'footprint': footprint_size(geometry)}
        return key

    def pose(self, robot: str, base_pose, names: list, values: list, *, live: bool, source: str) -> dict:
        """One robot drawn at one pose.

        Args:
            robot (str): Short robot name.
            base_pose (list): ``[pos, quat_xyzw]`` of its base.
            names (list): Arm joint names.
            values (list): Their values.
            live (bool): Measured now (the recording robot) or a saved belief.
            source (str): Where the pose came from.

        Returns:
            dict: ``{name, model, live, source, base, links}``.
        """
        key = self._model(robot)
        model = self.models[key]
        drawn = robot_matrices(model['kinematics'], model['geometry'], matrix_from_pose(base_pose[0], base_pose[1]),
                               values, names)
        return {'name': robot, 'model': key, 'live': live, 'source': source, **drawn}

    def payload(self) -> dict:
        """The meshes the page needs.

        Returns:
            dict: ``{model: {meshes, footprint}}`` for every model drawn.
        """
        return {key: {'meshes': m['meshes'], 'footprint': m['footprint']} for key, m in self.models.items()}


def record_robots(record: dict, models: RobotModels) -> list:
    """The robots of one record: the recording one live, the others as last saved.

    Args:
        record (dict): An as-built record.
        models (RobotModels): The loaded models.

    Returns:
        list: Robot poses (`RobotModels.pose`); a robot whose base is not known is left out.
    """
    out = []
    state = record.get('robot_state') or {}
    if state.get('base_pose'):
        names, values = [], []
        for arm in (state.get('arms') or {}).values():
            names += arm.get('joint_names') or []
            values += arm.get('joints') or []
        out.append(models.pose(record['robot'], state['base_pose'], names, values, live=True, source='live'))
    for name, belief in (((record.get('beliefs') or {}).get('robots')) or {}).items():
        if name == record['robot'] or not belief.get('base_pose') or belief.get('source') == 'parked':
            continue
        conf = belief.get('configuration') or {}
        out.append(models.pose(name, belief['base_pose'], list(conf), list(conf.values()), live=False,
                               source=belief.get('source') or 'saved'))
    return out


# * ---------------------------------------------------------------------------
# * The whole day
# * ---------------------------------------------------------------------------

def release_as_run(record: dict, actions: list, holding: dict):
    """Which R action a release record really was: the one of the bar in the robot's tools.

    * An R lets go of the bar the same robot's last J left in its tools. The monitor loads
    * the next R it finds not done, so it can name another bar: on 23 Oct no R was marked
    * done and Cindy ran "B20_R" at 18:12 while she held B34. Such a record is moved to the
    * held bar's R, with the release moments handed from the named bar to the held one.

    Args:
        record (dict): An as-built record of an R action.
        actions (list): `timeline`'s actions.
        holding (dict): Robot name -> the bar its last J left in its tools.

    Returns:
        tuple: ``(action index, record, note)``; the record and index unchanged and the note
        None when the R names the held bar (or the held bar has no R action).
    """
    k = record['action']['index']
    named, held = actions[k]['bar_id'], holding.get(record.get('robot'))
    real = next((a for a in actions if a['kind'] == 'R' and a['bar_id'] == held), None)
    if held in (None, named) or real is None:
        return k, record, None
    record = dict(record, bars={key: dict(entry) for key, entry in (record.get('bars') or {}).items()})
    moment = None
    for entry in record['bars'].values():
        if entry.get('bar_id') == named:
            moment, entry['milestone'] = entry.get('milestone'), 'bystander'
    for entry in record['bars'].values():
        if entry.get('bar_id') == held:
            entry['milestone'] = moment
    note = (f"loaded as {actions[k]['action_id']}, but {record.get('robot')}'s tools held {held} "
            f"(the last bar it inserted): this is {held}'s release")
    return real['index'], record, note


def collect_assembly(day: str, problem: str = None, with_robots: bool = True, runs_dir: str = RUNS_DIR,
                     monitor_log: str = MONITOR_LOG) -> dict:
    """Read one day's as-built records and shape them for the page.

    Args:
        day (str): ``YYYYMMDD``.
        problem (str | None): The design problem (default: the records').
        with_robots (bool): Draw the robots (loads the URDF meshes).
        runs_dir (str): The assembly runs' folder (``--runs-dir``).
        monitor_log (str): The monitor's terminal log, for the tick boxes of each step
            (`step_options`); a missing file leaves them unknown.

    Returns:
        dict: The page's ``DATA`` without the environment, the cameras and the page text:
        ``{problem, day, assembled {bar id: {start, end, length}}, actions [{index, action_id, kind,
        bar_id, robot, exists, held, worst, records [...]}] (in the plan skeleton's order),
        history {bar id: {entries [...], skipped}}, ball_check {bar id: `ball_check`}, robot_models,
        monitor_log}``; each record has its step's ``options`` (`step_options`).
    """
    path = as_built_file(day, problem, runs_dir)
    records = sorted(read_as_built(path), key=lambda r: r.get('stamp_epoch') or 0.0)
    if not records:
        raise SystemExit(f"{path} holds no record.")
    problem = problem or records[0]['problem']
    problem_dir = problem_root(problem, DESIGN_DATA_DIRECTORY)
    skeleton = load_plan_skeleton(problem_dir)
    if skeleton is None:
        raise SystemExit(f"{problem_dir} has no ActionSchedule.json.")
    # * Only a day with its own table is fitted again: other days keep the monitor's fits.
    own_table = False
    try:
        markers = load_mocap_markers(day_markers_root(day, problem_dir, runs_dir))
        own_table = os.path.dirname(markers.path) == os.path.dirname(path)
        if own_table:
            print(f"[assembly] the day's own marker table: {markers.path}; bars whose layout differs from the "
                  "monitor's are fitted again from their recorded balls.")
    except Exception as e:  # the page still shows the authored bars
        print(f"[assembly] ! no marker table ({e}); markers and plates are not drawn.")
        markers = None
    # * The seats in each record's own frame: a monitor with the design on the measured
    # * floor wrote its reference poses there (its record's world_from_design). The page
    # * draws the day's last frame.
    seats_by_placement = {}

    def seats_of(record: dict) -> dict:
        """The seats in the frame ``record``'s monitor used (built once per frame)."""
        key = json.dumps(record.get('world_from_design'))
        if key not in seats_by_placement:
            seats_by_placement[key] = assembled_bars(problem_dir, record.get('world_from_design'))
        return seats_by_placement[key]

    seats = seats_of(records[-1])
    if len({json.dumps(r.get('world_from_design')) for r in records}) > 1:
        print("[assembly] ! the day mixes records with and without the design on the measured floor; "
              "each record is compared in its own frame, the page draws the last one's.")
    servos = servo_files(day, runs_dir)
    events = option_events(day, monitor_log)
    models = RobotModels() if with_robots else None
    print(f"[assembly] {len(records)} records from {path}; problem {problem}, {len(skeleton.actions)} actions, "
          f"{len(seats)} bars, {len(servos)} servo files.")
    print(f"[assembly] tick boxes: {len(events)} lines of {day} in {monitor_log}" if events is not None
          else f"[assembly] ! no monitor log at {monitor_log}: the tick boxes of each step are unknown.")

    actions = timeline(skeleton)
    history = {}
    pairs_of_day = {}    # bar id -> its plates in every record of its history (for ball_check)
    placed = set()    # bars past their first milestone (mounted on the robot): drawn from then on
    starts = {}    # (run_id, action, step) -> the step's start record's time
    start_records = {}    # the same key -> the start record as shaped for the page
    last_of_action = {}
    holding = {}    # robot -> the bar its last J left in its tools (until its R ends)
    for record in records:
        action = record.get('action') or {}
        k = action.get('index')
        if k is None or not 0 <= k < len(actions):
            continue    # outside the plan skeleton (a click with nothing loaded): not on the slider
        step = record.get('step') or {}
        # * Which bar an R really let go of (see `release_as_run`).
        note = None
        if actions[k]['kind'] == 'J':
            holding[record.get('robot')] = actions[k]['bar_id']
        elif actions[k]['kind'] == 'R':
            k, record, note = release_as_run(record, actions, holding)
            if note and step.get('phase') == 'start':
                print(f"[assembly] {record['stamp']}: {note}")
            if step.get('phase') == 'end' and step.get('outcome') in (None, 'ok'):
                holding.pop(record.get('robot'), None)
        act = actions[k]
        key = (record.get('run_id'), k, step.get('index'))
        epoch = record.get('stamp_epoch') or 0.0
        if step.get('phase') == 'start':
            starts[key] = epoch
        bars = []
        for entry in (record.get('bars') or {}).values():
            seat = seats_of(record).get(entry.get('bar_id'))
            if seat is None:
                continue
            design = markers.bars.get(f"bar_{entry.get('bar_id')}") if markers is not None else None
            if own_table:
                entry = refit_on_new_layout(entry, design)
            bar = shape_bar(entry, record, seat, design)
            # * A bar is drawn from its first milestone on (mounted on the robot, then placed), in a
            # * step that ended well. Before that it lies on its loading spot, metres away: not drawn,
            # * not in the action's worst error, only counted in its history (the same rule as `add_history`).
            if bar['milestone'] not in (None, 'bystander') and step.get('outcome') in (None, 'ok'):
                placed.add(bar['bar'])
            if bar['bar'] not in placed:
                if bar['ok']:
                    history.setdefault(bar['bar'], {'entries': [], 'skipped': 0})['skipped'] += 1
                continue
            bars.append(bar)
        # * The servo that ran inside this step: saved between its start and end records.
        servo = None
        if step.get('phase') == 'end' and key in starts:
            inside = [p for t, p in servos if starts[key] - SERVO_MATCH_SLACK_S <= t <= epoch + SERVO_MATCH_SLACK_S]
            if inside:
                servo = servo_numbers(inside[-1])
                if servo['movement_id'] != step.get('movement_id'):
                    servo['note'] = (f"saved as {servo['movement_id']} (the servo after a transfer or an insert "
                                     "is named after the step it aims at)")
        prev = last_of_action.get(k)
        redo = prev is not None and (prev[0] != record.get('run_id') or epoch - prev[1] > REDO_GAP_S)
        last_of_action[k] = (record.get('run_id'), epoch)
        tool0 = tool0_numbers(record)
        worst_tool0 = max((v['fk_vs_target']['dpos_mm'] for v in tool0.values() if v['fk_vs_target']), default=None)
        shaped = {
            'stamp': record.get('stamp'), 'time': (record.get('stamp') or 'T')[11:19], 'run_id': record.get('run_id'),
            'robot': record.get('robot'), 'redo': redo, 'note': note,
            'step': {'index': step.get('index'), 'movement_id': step.get('movement_id'), 'kind': step.get('kind'),
                     'phase': step.get('phase'), 'outcome': step.get('outcome')},
            'milestone': next((b['milestone'] for b in bars if b['milestone'] not in (None, 'bystander')), None),
            'bars': bars, 'tool0': tool0, 'tool0_worst': worst_tool0, 'servo': servo,
            'load': load_numbers(record), 'fk_note': (record.get('robot_state') or {}).get('fk_note'),
            'robots': record_robots(record, models) if models is not None else [],
        }
        shaped['label'] = (MILESTONE_LABEL.get(shaped['milestone'])
                           or f"{(step.get('movement_id') or 'click').split('_', 3)[-1]} · {step.get('phase')}")
        # * The step's tick boxes, worked out at its end from all of its log lines and given to its
        # * start record too; a start that never got an end keeps what its own moment says.
        shaped['options'] = step_options(events, act['kind'], step, starts.get(key, epoch), epoch, servo)
        if step.get('phase') == 'start':
            start_records[key] = shaped
        elif step.get('phase') == 'end' and key in start_records:
            start_records.pop(key)['options'] = shaped['options']
        act.setdefault('records', []).append(shaped)
        for bar in bars:
            if not bar['ok']:
                continue
            kept = add_history(history, bar, {
                'action': k, 'record': len(act['records']) - 1, 'time': shaped['time'],
                'kind': act['kind'], 'robot': act['robot'], 'milestone': bar['milestone'],
                'placement_seat': bar.get('placement_seat'), 'seat_start': bar.get('seat_start'),
                'seat_middle': bar.get('seat_middle'), 'seat_end': bar.get('seat_end'),
                'rotation_seat': bar.get('rotation_seat'), 'bend': bar.get('bend'),
                'outcome': step.get('outcome'), 'fitted': bar.get('fitted')})
            if kept and bar.get('pairs'):
                pairs_of_day.setdefault(bar['bar'], []).append(bar['pairs'])

    # * The verdict on every measured bar, against what its balls did over the whole day.
    checks = {bar_id: ball_check(pairs) for bar_id, pairs in pairs_of_day.items()}
    for action in actions:
        action.setdefault('records', [])
        for record in action['records']:
            for bar in record['bars']:
                bar['findings'] = ball_findings(bar, checks.get(bar['bar']))
        ok = [r for r in action['records'] if r['step']['outcome'] in (None, 'ok')] or action['records']
        values = [b.get('placement') for b in (ok[-1]['bars'] if ok else []) if b.get('placement') is not None]
        action['worst'] = max(values) if values else None
    return {
        'problem': problem, 'day': day, 'source_file': path,
        'assembled': {bar_id: {'start': s['ends'][0], 'end': s['ends'][1], 'length': s['length']}
                      for bar_id, s in seats.items()},
        'actions': actions, 'history': history, 'ball_check': checks,
        'robot_models': models.payload() if models is not None else {},
        'monitor_log': monitor_log if events is not None else None,
    }


def main(argv: list = None) -> int:
    """Build the page of one lab day.

    Args:
        argv (list | None): Command line (None = ``sys.argv[1:]``).

    Returns:
        int: 0 on success.
    """
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('day', nargs='?', help='YYYYMMDD (default: the newest day with as-built records)')
    parser.add_argument('--problem', help='design problem (default: the one the records name)')
    parser.add_argument('--env-3dm', help='the Rhino file of the environment (default: found by itself)')
    parser.add_argument('--no-robots', action='store_true', help='skip the robots (no URDF load, smaller page)')
    parser.add_argument('--no-cameras', action='store_true', help='skip the mocap cameras')
    parser.add_argument('--vendor', choices=('inline', 'embed', 'sibling'), default='inline',
                        help='how three.js is carried (default inline: works from disk and when served)')
    parser.add_argument('--out', help='output .html (default: <day>-asbuilt-result/assembly_<day>.html on the drive)')
    parser.add_argument('--open', action='store_true', help='open the page in the browser')
    parser.add_argument('--runs-dir', default=RUNS_DIR,
                        help='the folder holding <date>-asbuilt/ and <date>-servoing/ (default: the drive\'s '
                             'bar_action_live_replan_exe; a headless smoke\'s scratch folder for a rehearsal)')
    parser.add_argument('--monitor-log', default=MONITOR_LOG,
                        help='the monitor\'s terminal log, for which tick boxes each step used (default: '
                             '~/husky_real/monitor.log, written by scripts/husky/open_terminals.sh)')
    args = parser.parse_args(argv)

    day = args.day or latest_as_built_day(args.runs_dir)
    scene = collect_assembly(day, args.problem, with_robots=not args.no_robots, runs_dir=args.runs_dir,
                             monitor_log=args.monitor_log)
    problem_dir = problem_root(scene['problem'], DESIGN_DATA_DIRECTORY)
    try:
        markers = load_mocap_markers(markers_root(problem_dir))
    except Exception:
        markers = None
    env_3dm = resolve_env_3dm(problem_dir, markers, args.env_3dm)
    env = build_environment(env_3dm) if env_3dm else None
    if not env and not args.env_3dm and os.path.exists(LAB_ENV_3DM) and env_3dm != LAB_ENV_3DM:
        print(f"[assembly] {os.path.basename(env_3dm) if env_3dm else 'the design'} has no 'env' solids; the lab room "
              f"and the camera rig come from {os.path.basename(LAB_ENV_3DM)} (same Motive frame).")
        env_3dm = LAB_ENV_3DM
        env = build_environment(env_3dm)
    scene.update({
        'title': f"As-built {scene['problem']} · {day}",
        'generated': datetime.now().isoformat(timespec='seconds'),
        'metrics': METRICS, 'extra_formulas': EXTRA_FORMULAS,
        'env': env,
        'cameras': [] if args.no_cameras or not env_3dm else build_cameras(env_3dm),
        'cone': {'length': CAMERA_CONE_M, 'half_angle_deg': CAMERA_CONE_HALF_ANGLE_DEG},
    })
    out = args.out or os.path.join(args.runs_dir, f"{day}-asbuilt-result", f"assembly_{day}.html")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    write_page(finite(scene), out, args.vendor, viewer_script(os.path.join(HERE, '4_assembly_viewer.js')),
               extra_css=PAGE_CSS, controls=PAGE_CONTROLS)
    print(f"[assembly] wrote {out} ({os.path.getsize(out) / 1e6:.1f} MB)")
    if args.open:
        webbrowser.open('file://' + os.path.abspath(out))
    return 0


# * The step controls and their style, put into the shared page template (viewer_common.write_page).
PAGE_CSS = """#steps { left:364px; right:14px; bottom:14px; width:auto; padding:9px 12px; }
#steps-head { display:flex; align-items:center; gap:8px; }
#steps-head button { width:auto; margin:0; padding:1px 8px; }
#action { flex:1; accent-color:#1763b8; }
#action-label { margin:5px 0 2px; font-size:12px; }
#options { margin:2px 0 4px; font-size:11px; line-height:1.55; }
#options .opt { display:block; padding-left:8px; }
#options .opt b { display:inline-block; min-width:66px; white-space:nowrap; }
#options .opt.used b { color:#1a7f37; }
#options .opt.unused b { color:#6b7686; }
#options .opt.note { color:#9a6700; padding-left:0; }
#records { display:flex; flex-wrap:wrap; gap:4px; margin:4px 0; }
.rec { width:auto; margin:0; padding:2px 8px; font-size:11px; border-radius:11px; }
.rec.on { background:#1763b8; color:#fff; border-color:#1763b8; }
.rec.cancelled { text-decoration:line-through; color:#b45309; }
.rec.on.cancelled { color:#fff; }
#steps-foot { display:flex; align-items:center; gap:10px; margin-top:4px; font-size:11px; color:var(--dim); }
#steps-foot label { display:flex; align-items:center; gap:4px; cursor:pointer; white-space:nowrap; }
#strip-by { width:auto; padding:1px 4px; font-size:11px; }
#timeline { display:flex; gap:1px; height:14px; margin-top:6px; }
#timeline .cell { flex:1; min-width:3px; border-radius:2px; cursor:pointer; }
#timeline .cell.cur { outline:2px solid #1d2430; outline-offset:1px; }
"""
PAGE_CONTROLS = """    <div class="card" id="steps">
      <div id="steps-head"><span class="muted">action</span><button id="prev" title="previous action (left arrow)">&#9664;</button>
        <input type="range" id="action" min="0" max="0" value="0"><button id="next" title="next action (right arrow)">&#9654;</button></div>
      <div id="action-label"></div>
      <div id="records"></div>
      <div id="options"></div>
      <div id="steps-foot"><label><input type="checkbox" id="all-records"> all records</label>
        <span>[ and ] step through them</span><span style="margin-left:auto">strip by</span><select id="strip-by"></select></div>
      <div id="timeline"></div>
    </div>
"""


if __name__ == '__main__':
    sys.exit(main())
