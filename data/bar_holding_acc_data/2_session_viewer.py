"""Build one self-contained 3D web page for a bar-holding accuracy session.

Where `0_` fits the bars and `1_` compares them to the cell state one panel at a
time, this draws the whole session at once: every bar where mocap says it landed,
coloured by how far off it was, with the cell around it and the robot at each
parking spot. Click a bar to read its servo and load numbers.

The page is a single `.html` file with the 3D library inside it, so it opens in
any browser with no server, no install and no internet connection.

Run it with the session folder name, the same way as the other two scripts::

    python 2_session_viewer.py 20261001

which writes ``<session>-result/session_<session>.html`` next to the data.

What it reads:
- ``<batch>/bar_holding_acc_*.json``   -- the marker takes, one per bar
- ``<batch>-servoing/servoing_data_*.json`` -- how the arms converged
- the design problem's ``.3dm`` -- the environment solids on the ``env`` layer
- each bar's BarAction -- where the robot was told to stand

An ``<batch>-archive/`` folder, if there is one, is deliberately NOT read: it
holds records that were set aside on purpose.
"""

import argparse
import base64
import json
import os
import re
import sys
import webbrowser
from datetime import datetime

# ! Running this file directly puts only its OWN folder on the import path, not
# ! the repo, so the package import below fails with ModuleNotFoundError even
# ! when you are sitting in the repo root. Put the repo root on the path first.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))

import numpy as np

from husky_assembly_teleop import EXPERIMENT_DATA_DIRECTORY
from husky_assembly_teleop.bar_action_io import resolve_take_movement
from husky_assembly_teleop.mocap_experiment import (
    fit_bar_from_markerset,
    bar_deviation_from_goal,
    pair_fit_to_goal,
    tip_distances,
    make_axis_corrector,
    convert_markerset_axes,
    read_env_obstacles_3dm,
    read_mocap_cameras_3dm,
    _reroot_gdrive_path,
    latest_batch_folder,
    env_3dm_for_bar_action,
)

# ! Imported lazily inside build_robots(): loading the URDF pulls in pybullet and
# ! costs a couple of seconds, which a --no-robots run should not pay.

# * Where the three javascript files live. They are NOT in git (see .gitignore);
# * scripts/fetch_dashboard_vendor.sh downloads them once per checkout. They are
# * needed only to GENERATE the page -- the finished page carries its own copy.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
VENDOR_DIR = os.path.join(_REPO_ROOT, 'husky_assembly_teleop', 'dashboard',
                          'static', 'vendor', 'three-r185')
THREE_CORE = os.path.join(VENDOR_DIR, 'build', 'three.core.min.js')
THREE_MODULE = os.path.join(VENDOR_DIR, 'build', 'three.module.min.js')
ORBIT_CONTROLS = os.path.join(VENDOR_DIR, 'examples', 'jsm', 'controls',
                              'OrbitControls.js')

# * The Rhino layer holding the environment solids (desks, columns, walls), and
# * the one holding the Motive camera rig.
ENV_LAYER = 'env'
CAMERA_LAYER = 'Mocap::mocap_cameras'

# * How far to draw each camera's view cone, and how wide it opens. Motive does
# * not record a field of view, so this is a readable stand-in, not a measured
# * frustum -- it says where a camera points, not exactly what it can see.
CAMERA_CONE_M = 0.9
CAMERA_CONE_HALF_ANGLE_DEG = 22.0

# * A force reading below this means the sensor was zeroed while the bar was
# * already gripped, so the bar's own load was tared away. Such a record cannot
# * say anything about bending and is shown as "load not measured" instead of
# * being coloured as a perfect result.
FT_INVALID_BELOW_N = 1.5

# * How the four colour metrics are scaled. `high` is the top of the gradient;
# * anything above it gets the single warning colour rather than a gradient step,
# * so one bad bar cannot flatten the range the good ones live in.
METRICS = [
    {'key': 'placement', 'label': 'placement error', 'unit': 'mm', 'high': 5.0,
     'help': 'how far the bar ended up from where it was meant to be: the '
             'WORST of the three places along it that are measured',
     'formula': 'max over the start, middle and end of '
                '‖ detected − authored ‖',
     'detail': 'The detected and authored bars are compared at the same three '
               'points along each, and the largest of the three is the '
               'number -- a bar that is right at one end and out at the other '
               'is placed badly, and the start on its own would call it good. '
               'The fitted tips are paired to the authored ones first, so the '
               'number does not flip when a bar is built end-for-end. Both '
               'bars are straight, so the gap changes smoothly from one end to '
               'the other: that is what the shading along each bar shows.'},
    {'key': 'rotation', 'label': 'rotation error', 'unit': 'deg', 'high': 0.25,
     'help': 'angle between the bar axis as built and as authored',
     'formula': 'arccos( | fitted_axis · authored_axis | )',
     'detail': 'The absolute value makes it blind to which way along the bar '
               'each axis points, so the answer is always 0-90 deg.'},
    {'key': 'fit_residual', 'label': 'fit residual', 'unit': 'mm', 'high': 1.5,
     'help': 'how well the 8 markers lie on one straight line -- the QUALITY '
             'of the measurement, not an error of the robot',
     'formula': 'max perpendicular distance from a marker-pair midpoint '
                'to the fitted axis',
     'detail': 'Per take the worst of the four pairs, then averaged over the '
               'takes: deliberately a max, because one bad pair is the thing '
               'worth seeing and a mean would hide it behind three good ones. '
               'Across this session it does not correlate with placement error '
               '(r = -0.05), which is what says the placement error is real.'},
    {'key': 'servo', 'label': 'servo residual', 'unit': 'mm', 'high': 1.5,
     'help': 'how far each wrist still was from its target on the last servo '
             'iteration, before the markers were recorded. Both wrists are '
             'written on the bar; the colour follows the worse of the two',
     'formula': 'max( ‖tool0_left − target_left‖ , ‖tool0_right − target_right‖ )',
     'detail': 'Taken from the LAST iteration of the paired servo run. Both '
               'sides are computed by the robot from its own encoders and base '
               'pose, so this says whether the robot reached what it aimed at '
               '-- not whether it aimed at the right place.'},
    {'key': 'load', 'label': 'load imbalance', 'unit': 'N', 'high': 2.0,
     'help': 'gap between the two wrists force magnitudes. The two grippers '
             'pull against each other through the bar, so a BALANCED pair is a '
             'bar in clean axial load; the gap is the part the bar has to '
             'absorb sideways, i.e. bending',
     'formula': '| ‖F_left‖ − ‖F_right‖ |',
     'detail': 'Both wrists hold one rigid bar, so an equal and opposite pair '
               'is clean axial load. What is left over is what the bar must '
               'take sideways.'},
]

# * Measurements shown in the detail card that are not colour metrics.
EXTRA_FORMULAS = [
    {'label': 'tool0 left / right', 'unit': 'mm',
     'formula': '‖ FK(base_mocap, joint_encoders) − commanded_frame ‖',
     'detail': 'Per wrist, on the servo run final iteration. The left flange is '
               'driven exact and the right absorbs the grasp mismatch, so only '
               'the right one tracks the placement error (r = +0.51 against '
               '-0.18 for the left).'},
]


# ---------------------------------------------------------------------------
# * Matching each servo run to the bar it belongs to
# ---------------------------------------------------------------------------

def file_timestamp(path: str) -> datetime:
    """When a take or run was saved, read from its own filename.

    ! Deliberately NOT the file's modification time. Editing a take -- fixing a
    ! wrong label, say -- rewrites that, and the record would then sort to the
    ! end of the session and pair with the wrong run. The name is written once,
    ! when the file is saved, and never changes.

    Takes are stamped to the minute (``..._1902.json``) and runs to the second
    (``..._185931.json``).

    Args:
        path (str): The file.

    Returns:
        datetime: Its saved time, or None when the name does not carry one.
    """
    stem = os.path.splitext(os.path.basename(path))[0]
    date, _, clock = stem.rpartition('_')
    date = date.rpartition('_')[2]
    # ! Pick the format by how many digits the clock has, rather than trying
    # ! both. strptime accepts one OR two digits per field, so 'HHMM' also
    # ! matches an 'HHMMSS' pattern -- 1514 would come back as 15:01:04.
    formats = {6: '%Y%m%d%H%M%S', 4: '%Y%m%d%H%M'}
    if not date.isdigit() or not clock.isdigit() or len(clock) not in formats:
        return None
    try:
        return datetime.strptime(date + clock, formats[len(clock)])
    except ValueError:
        return None


def pair_runs_to_takes(take_paths: list, run_paths: list) -> dict:
    """Decide which servo run belongs to each marker take.

    A servo run file never names its bar -- there is no bar name anywhere in it
    -- while a take does. The session always goes "servo the arms into place,
    then record the markers", so each take belongs to the last run saved before
    it.

    ? A take's name only gives the minute, so the take could have been saved any
    ? time inside it. A run counts as "before" the take when it started before
    ? that minute was over -- otherwise a run saved at 16:05:31 would look later
    ? than the take named 16:05 that it actually fed.

    Args:
        take_paths (list): Take files, any order.
        run_paths (list): Servo run files, any order.

    Returns:
        dict: ``{take_path: run_path or None}``.
    """
    runs = sorted((file_timestamp(p), p) for p in run_paths if file_timestamp(p))
    pairing = {}
    for take in take_paths:
        when = file_timestamp(take)
        if when is None:
            pairing[take] = None
            continue
        end_of_minute = when.timestamp() + 60.0
        earlier = [p for t, p in runs if t.timestamp() < end_of_minute]
        pairing[take] = earlier[-1] if earlier else None
    return pairing


def check_pairing(pairing: dict, run_paths: list) -> None:
    """Print whether the run-to-take matching looks trustworthy.

    Two things have to hold. Every run should be used exactly once, because the
    session alternates one run per take. And the force the take recorded should
    match the force the run saw on its last iteration, because the two readings
    are seconds apart.

    Args:
        pairing (dict): Output of :func:`pair_runs_to_takes`.
        run_paths (list): Every run file found, to spot unused ones.
    """
    used = [p for p in pairing.values() if p]
    leftover = sorted(set(run_paths) - set(used))
    repeated = len(used) - len(set(used))

    print(f"\n[pairing] {len(used)} of {len(run_paths)} runs matched to "
          f"{len(pairing)} takes")
    if repeated:
        print(f"  ! {repeated} run(s) matched to more than one take -- the "
              f"session is not a clean one-run-per-take alternation")
    for path in leftover:
        print(f"  ! unused run: {os.path.basename(path)} (no take followed it)")

    worst = 0.0
    for take, run in pairing.items():
        if not run:
            print(f"  ! no run before {os.path.basename(take)}")
            continue
        take_force = _take_forces(_load_json(take))
        run_force = _run_last_forces(_load_json(run))
        if take_force is None or run_force is None:
            continue
        # ? A take whose sensor was zeroed while the bar was gripped reads near
        # ? nothing, so it cannot confirm or deny the match -- skip it rather
        # ? than report a disagreement that says nothing about the pairing.
        if max(take_force) < FT_INVALID_BELOW_N:
            print(f"  - {os.path.basename(take)}: load not measured, so the "
                  f"force cross-check is skipped for it")
            continue
        gap = max(abs(a - b) for a, b in zip(take_force, run_force))
        worst = max(worst, gap)
        if gap > 2.0:
            print(f"  ! {os.path.basename(take)} and "
                  f"{os.path.basename(run)} disagree on force by {gap:.1f} N "
                  f"-- they may not belong together")
    print(f"  worst force disagreement across the session: {worst:.2f} N")


def _load_json(path: str) -> dict:
    """Read a json file.

    Args:
        path (str): File to read.

    Returns:
        dict: Its contents.
    """
    with open(path, 'r') as handle:
        return json.load(handle)


def _take_forces(take: dict) -> tuple:
    """Average left/right force magnitude over a take file's repeats.

    Args:
        take (dict): A loaded take file.

    Returns:
        tuple: ``(left_N, right_N)``, or None when no force was recorded.
    """
    left, right = [], []
    for entry in take.get('raw_data') or []:
        wrench = entry.get('tool_ft') or {}
        if wrench.get('left_force_N') is None:
            continue
        left.append(abs(float(wrench['left_force_N'])))
        right.append(abs(float(wrench['right_force_N'])))
    if not left:
        return None
    return float(np.mean(left)), float(np.mean(right))


def _run_last_forces(run: dict) -> tuple:
    """Left/right force magnitude on a run's final iteration.

    Args:
        run (dict): A loaded servo run file.

    Returns:
        tuple: ``(left_N, right_N)``, or None when the run logged no wrench.
    """
    iterations = run.get('servoing_data') or []
    if not iterations:
        return None
    wrench = iterations[-1].get('wrench') or {}
    if 'left' not in wrench:
        return None
    return (abs(float(wrench['left'].get('force_N', 0.0))),
            abs(float(wrench['right'].get('force_N', 0.0))))


# ---------------------------------------------------------------------------
# * Measuring one bar, through the same helpers 1_compare_to_cell_state.py uses
# ---------------------------------------------------------------------------

def _pairs_along_bar(fit: dict, start_tip: list) -> list:
    """The marker pairs of one take, ordered from one end of the bar along it.

    ! The fit hands the pairs back in the order they were MATCHED -- closest to
    ! the nominal cross-bar distance first -- which is not the order they sit
    ! in along the bar and which changes from take to take. Averaging the takes
    ! without re-ordering them therefore mixes two different physical pairs
    ! together: their numbers come out identical and both land halfway between
    ! the two pairs in the 3D view.

    Args:
        fit (dict): One take's fit from ``fit_bar_from_markerset``.
        start_tip (list): The bar tip to measure from, so every take counts
            its pairs from the same end.

    Returns:
        list: Pair indices, the one nearest that tip first.
    """
    start = np.asarray(start_tip, dtype=float)
    centers = np.asarray(fit['pair_centers'], dtype=float)
    return [int(i) for i in np.argsort(np.linalg.norm(centers - start, axis=1))]


def measure_take(path: str) -> dict:
    """Fit the bar in one take file and compare it to its authored pose.

    Every number here comes from the helpers ``1_compare_to_cell_state.py``
    calls, so the page and that report cannot disagree.

    ! The stamped ``bar_start_position`` is the bar's LOWER TIP, not its middle
    ! (a known Rhino export bug). This function never does its own arithmetic on
    ! it -- ``pair_fit_to_goal`` pairs the fitted tips to the goal tips and hands
    ! back both, which is the only correct way to use that field.

    Args:
        path (str): A ``bar_holding_acc_*.json`` take file.

    Returns:
        dict: One bar's record, or None when it holds nothing measurable.
    """
    data = _load_json(path)
    correct = make_axis_corrector(data.get('mocap_axis_convention', 'rotated'))
    goal_position = data.get('bar_start_position')
    goal_quaternion = data.get('bar_start_quaternion')
    if goal_position is None or goal_quaternion is None:
        print(f"  SKIP {os.path.basename(path)}: no stamped bar pose")
        return None
    goal_pose = (list(goal_position), list(goal_quaternion))

    placements, rotations, residuals = [], [], []
    per_pair, pair_ids, pair_points = [], [], []
    tips, markers = [], []
    unfittable = None
    for entry in data.get('raw_data') or []:
        points = convert_markerset_axes(entry.get('bar_rig', {}), correct)
        try:
            fit = fit_bar_from_markerset(points)
        except Exception as error:
            # ? A take whose markers cannot be paired is usually one where the
            # ? markerset was knocked or partly hidden. Counted once at the end
            # ? rather than printed per take -- one file here has 31 of them.
            unfittable = (unfittable[0] + 1, str(error)) if unfittable else (1, str(error))
            continue
        deviation = bar_deviation_from_goal(fit, goal_pose)
        pairing = pair_fit_to_goal(fit, goal_pose)
        placements.append(pairing['start_dev_m'] * 1000.0)
        rotations.append(np.rad2deg(deviation['angle_rad']))
        residuals.append(fit['center_to_line_dist_max_m'] * 1000.0)
        order = _pairs_along_bar(fit, pairing['fit_start'])
        per_pair.append([fit['center_to_line_dists_m'][i] * 1000.0
                         for i in order])
        pair_ids.append([list(fit['pairs'][i]) for i in order])
        pair_points.append([list(map(float, fit['pair_centers'][i]))
                            for i in order])
        tips.append([list(pairing['fit_start']), list(pairing['fit_end'])])
        markers.append([list(map(float, m['pos'])) for m in points.values()])

    if unfittable:
        print(f"  {os.path.basename(path)}: {unfittable[0]} of "
              f"{len(data.get('raw_data') or [])} takes could not be fitted "
              f"({unfittable[1]})")
    if not placements:
        print(f"  SKIP {os.path.basename(path)}: nothing could be fitted")
        return None

    # * Repeat takes of one bar differ by a fraction of a millimetre, so the
    # * average is the bar's answer and the spread says how settled it was.
    goal_pairing = pair_fit_to_goal(fit, goal_pose)
    forces = _take_forces(data)
    return {
        'bar': (data.get('bar_name') or '?').replace('bar_', ''),
        'file': os.path.basename(path),
        'movement': data.get('movement_id'),
        'bar_action_path': data.get('bar_action_path'),
        'n_takes': len(placements),
        # ? How much the repeat takes of this bar disagreed at the start tip --
        # ? a fraction of a millimetre when the markers were seen cleanly. The
        # ? placement error itself is added by add_tip_distances().
        'placement_spread': float(np.ptp(placements)),
        'rotation': float(np.mean(rotations)),
        # * Worst all the way through: the worst marker pair in a take, then the
        # * worst take. One bad pair in one take is exactly the thing worth
        # * seeing, and any averaging hides it behind the good ones.
        'fit_residual': float(np.max(residuals)),
        'fit_residual_mean': float(np.mean(residuals)),
        # Each marker pair's own distance from the fitted axis, averaged over
        # the takes, so a single bad pair can be picked out by name.
        'pair_residuals': [float(v) for v in np.mean(per_pair, axis=0)],
        'pair_ids': pair_ids[0],
        'pair_points': [list(map(float, c)) for c in np.mean(pair_points, axis=0)],
        'bar_length': float(np.linalg.norm(
            np.array(tips[0][0]) - np.array(tips[0][1]))),
        'fitted': [list(np.mean([t[0] for t in tips], axis=0)),
                   list(np.mean([t[1] for t in tips], axis=0))],

        'authored': [list(map(float, goal_pairing['goal_start'])),
                     list(map(float, goal_pairing['goal_end']))],
        'markers': markers[0],
        'force_left': forces[0] if forces else None,
        'force_right': forces[1] if forces else None,
    }


def add_tip_distances(record: dict) -> None:
    """Measure the detected and authored bars against each other at three places.

    The headline ``placement`` number is the WORST of the three. A bar that
    sits right at one end and is out at the other is placed badly, and the
    start distance on its own -- which this number used to be -- would call it
    good. Their spread is the other half of the story: all three alike means
    the bar is shifted bodily, three different numbers mean it is tilted.

    Args:
        record (dict): A bar record, edited in place.
    """
    # * One shared helper with 1_compare_to_cell_state.py, so the page and that
    # * report cannot drift apart on the headline number.
    tips = tip_distances({
        'fit_start': np.asarray(record['fitted'][0], dtype=float),
        'fit_end': np.asarray(record['fitted'][1], dtype=float),
        'goal_start': np.asarray(record['authored'][0], dtype=float),
        'goal_end': np.asarray(record['authored'][1], dtype=float),
    })
    record['tip_start'] = tips['start_m'] * 1000.0
    record['tip_middle'] = tips['middle_m'] * 1000.0
    record['tip_end'] = tips['end_m'] * 1000.0
    record['tip_mean'] = tips['mean_m'] * 1000.0
    record['placement'] = tips['worst_m'] * 1000.0


def add_servo_numbers(record: dict, run_path: str) -> None:
    """Attach the paired run's last iteration and the load reading to a bar.

    Args:
        record (dict): A bar record from :func:`measure_take`, edited in place.
        run_path (str): The servo run matched to it, or None.
    """
    record['run'] = os.path.basename(run_path) if run_path else None
    record['servo'] = None
    record['iterations'] = None
    record['servo_left'] = record['servo_right'] = None
    record['rot_left'] = record['rot_right'] = None
    if run_path:
        iterations = _load_json(run_path).get('servoing_data') or []
        if iterations:
            tool0 = iterations[-1].get('tool0') or {}
            left = tool0.get('left', {})
            right = tool0.get('right', {})
            record['iterations'] = len(iterations)
            record['servo_left'] = left.get('pos_norm_mm')
            record['servo_right'] = right.get('pos_norm_mm')
            record['rot_left'] = _axis_max(left.get('rot_err_deg'))
            record['rot_right'] = _axis_max(right.get('rot_err_deg'))
            if left.get('pos_norm_mm') is not None:
                record['servo'] = max(float(left['pos_norm_mm']),
                                      float(right['pos_norm_mm']))

    # * The bending number. Both wrists pull against each other through a rigid
    # * bar, so an equal pair means clean axial load; the GAP is the unbalanced
    # * part the bar has to take sideways.
    left_force, right_force = record['force_left'], record['force_right']
    if left_force is None:
        record['load'] = None
        record['load_valid'] = False
    else:
        record['load'] = abs(left_force - right_force)
        record['load_valid'] = max(left_force, right_force) >= FT_INVALID_BELOW_N


def _axis_max(values: list) -> float:
    """Largest absolute value of a per-axis error triple.

    Args:
        values (list): Three per-axis numbers, or None.

    Returns:
        float: The largest magnitude, or None when nothing was given.
    """
    if not values:
        return None
    return float(max(abs(float(v)) for v in values))


# ---------------------------------------------------------------------------
# * The robot, drawn as boxes
# ---------------------------------------------------------------------------

def build_robots(records: list) -> dict:
    """Work out where each bar's robot stood, and the boxes that draw it.

    Takes record no robot pose at all, so it comes from the BarAction the take
    names: that movement's authored base frame and joint values. The link shapes
    are the same for every bar, so they are collected once and each robot only
    carries one matrix per link.

    Args:
        records (list): Bar records; each gains a ``robot`` entry when resolved.

    Returns:
        dict: ``{'links': {name: [min3, max3]}, 'footprint': [length, width,
        height]}`` -- the shared shapes the page needs.
    """
    from husky_assembly_teleop.cfab_session import HUSKY_DUAL_URDF_PATH
    from husky_assembly_teleop.dashboard.kinematics import SceneKinematics

    kinematics = SceneKinematics(HUSKY_DUAL_URDF_PATH)
    geometry = kinematics._link_geometry()
    # * Every robot in the session is the same machine, so the link shapes are
    # * sent ONCE and each robot adds only one matrix per link. That is what
    # * makes real geometry affordable here instead of crude boxes.
    link_meshes = _pack_link_meshes(geometry)

    resolved = 0
    for record in records:
        matrices = _robot_link_matrices(record, kinematics, geometry)
        record['robot'] = matrices
        record['robot_holds_bar'] = _wrist_to_bar_mm(record)
        resolved += bool(matrices)
        # ! A configuration that does not hold its own bar is a fault in the
        # ! export (B52 on 20261001 stores a horizontal carry pose where the
        # ! assembled pose should be). During the session the monitor ignored
        # ! those joint values and solved IK from the authored flange targets,
        # ! so drawing them here would show arms the robot never had. Keep the
        # ! base -- the parking spot is still right -- and drop the arm links;
        # ! the detail card says why they are missing.
        if matrices and not _robot_holds_bar(record):
            matrices['links'] = {}
    _warn_about_loose_robots(records)
    print(f"[robot] {resolved} of {len(records)} bars resolved their robot pose; "
          f"{len(geometry)} link shapes, "
          f"{sum(len(f) for _p, f in geometry.values())} triangles, sent once")
    return {'meshes': link_meshes, 'footprint': _footprint_size(geometry)}


def _pack_link_meshes(geometry: dict) -> dict:
    """The robot's link shapes, packed small enough to carry in the page.

    Each link keeps its own vertices and triangles in its own frame, so drawing
    it later is one matrix away.

    Args:
        geometry (dict): ``{link_name: (points, faces)}`` from SceneKinematics.

    Returns:
        dict: ``{link_name: {'positions': base64, 'indices': base64}}``.
    """
    packed = {}
    for name, (points, faces) in geometry.items():
        # ! uint16 indices only work while a link stays under 65536 vertices;
        # ! the heaviest here is the chassis at a few thousand.
        index_type = np.uint16 if len(points) <= 65535 else np.uint32
        packed[name] = {
            'positions': _pack(np.asarray(points, dtype=np.float32)),
            'indices': _pack(np.asarray(faces, dtype=index_type)),
            'bits': 16 if index_type is np.uint16 else 32,
        }
    return packed


# * How far a wrist may sit from the bar before the pose is not a grasp. The
# * grippers hold the bar 80 mm off its axis, so anything near that is right and
# * anything far past it means the configuration and the bar disagree.
GRASP_OFFSET_MM = 80.0
GRASP_OFFSET_TOLERANCE_MM = 250.0


def _closest_point_on_segment(point, start, end) -> tuple:
    """The nearest point on a line segment, and how far away it is.

    Args:
        point (numpy.ndarray): The point to measure from.
        start (numpy.ndarray): One end of the segment.
        end (numpy.ndarray): The other end.

    Returns:
        tuple: ``(point_on_the_segment, distance_m)``.
    """
    along = end - start
    length = float(np.linalg.norm(along))
    direction = along / length
    # ! Clamped to the segment, not to its infinite line, so a wrist beyond a
    # ! bar's end is measured against the end rather than past it.
    reach = float(np.clip(np.dot(point - start, direction), 0.0, length))
    foot = start + reach * direction
    return foot, float(np.linalg.norm(point - foot))


def _wrist_position(record: dict, side: str):
    """Where one authored wrist flange sits in the world.

    Args:
        record (dict): A bar record carrying ``robot``.
        side (str): ``'left'`` or ``'right'``.

    Returns:
        numpy.ndarray: The wrist origin, or None when no robot was resolved.
    """
    robot = record.get('robot')
    if not robot:
        return None
    matrix = robot['links'].get(f'{side}_ur_arm_wrist_3_link')
    if matrix is None:
        return None
    return np.asarray(matrix, dtype=float).reshape(4, 4).T[:3, 3]


def _wrist_to_bar_mm(record: dict) -> float:
    """How far the further wrist sits from the bar the robot is meant to hold.

    Both are authored, so this compares the export against itself: at the
    movement the take was recorded at, the arms should be gripping the bar.

    Args:
        record (dict): A bar record with ``robot`` and ``authored`` set.

    Returns:
        float: Distance in mm for the worse wrist, or None without a robot.
    """
    if not record.get('robot'):
        return None
    start = np.asarray(record['authored'][0], dtype=float)
    end = np.asarray(record['authored'][1], dtype=float)
    worst = 0.0
    for side in ('left', 'right'):
        wrist = _wrist_position(record, side)
        if wrist is None:
            continue
        worst = max(worst, _closest_point_on_segment(wrist, start, end)[1])
    return worst * 1000.0


# * Where the two grippers' numbers go when there is no robot to ask: a fifth
# * of the way in from each end of the bar. Which end is which is then only a
# * guess, so the page writes "L" and "R" in front of the numbers either way.
GRASP_FALLBACK_ALONG = {'left': 0.8, 'right': 0.2}


def add_grasp_points(record: dict) -> None:
    """Mark where along the bar each gripper holds it.

    The page writes the per-wrist numbers -- the servo residual and the force
    each tool reads -- beside the place on the bar they were measured at,
    rather than both in the middle. The wrists are authored, so each one is
    projected onto the bar mocap actually found.

    Args:
        record (dict): A bar record with ``fitted`` set, edited in place.
    """
    start = np.asarray(record['fitted'][0], dtype=float)
    end = np.asarray(record['fitted'][1], dtype=float)
    points = {}
    for side, fraction in GRASP_FALLBACK_ALONG.items():
        wrist = _wrist_position(record, side)
        on_bar = (start + (end - start) * fraction if wrist is None
                  else _closest_point_on_segment(wrist, start, end)[0])
        points[side] = [round(float(v), 5) for v in on_bar]
    record['grasp'] = points


def _warn_about_loose_robots(records: list) -> None:
    """Say which bars have an authored pose that does not hold their own bar.

    ! This catches a fault in the EXPORT, not in the measurement: the robot is
    ! drawn exactly as the BarAction describes it, so a wrist far from the bar
    ! means that file's movement and bar disagree with each other.

    Args:
        records (list): Bar records carrying ``robot_holds_bar``.
    """
    loose = [r for r in records
             if r.get('robot_holds_bar') is not None and not _robot_holds_bar(r)]
    for record in loose:
        print(f"  ! {record['bar']}: the authored robot pose does not hold its "
              f"own bar (worse wrist {record['robot_holds_bar']:.0f} mm away, "
              f"expected about {GRASP_OFFSET_MM:.0f}). Footprint kept, arms not "
              f"drawn, flagged in the viewer; the fault is in "
              f"{os.path.basename(record.get('bar_action_path') or '?')} -- "
              f"re-export it.")


def _robot_holds_bar(record: dict) -> bool:
    """Whether the authored joint values actually grip the authored bar.

    The one place the tolerance is applied, so the console warning and the
    decision to drop the arm links can never disagree.

    Args:
        record (dict): A bar record carrying ``robot_holds_bar`` (mm, or None).

    Returns:
        bool: True when the worse wrist is within tolerance of the grasp offset.
        A record with no robot at all counts as holding -- there is nothing to
        drop.
    """
    distance = record.get('robot_holds_bar')
    return distance is None or distance <= GRASP_OFFSET_MM + GRASP_OFFSET_TOLERANCE_MM


def _robot_link_matrices(record: dict, kinematics, geometry: dict) -> dict:
    """Per-link placement matrices for the robot behind one bar.

    Args:
        record (dict): A bar record carrying ``bar_action_path``/``movement``.
        kinematics (SceneKinematics): The loaded robot model.
        geometry (dict): Links that actually have a shape to draw.

    Returns:
        dict: ``{'base': 16 floats, 'links': {name: 16 floats}}``, or None.
    """
    path = _reroot_gdrive_path(record.get('bar_action_path'))
    if not path or not os.path.exists(path):
        print(f"  ! {record['bar']}: BarAction not found, robot not drawn")
        return None
    try:
        _idx, movement, _action, _p = resolve_take_movement(
            path, record.get('movement') or 'M3')
        state = movement.start_state
        base_frame, configuration = state.robot_base_frame, state.robot_configuration
        if base_frame is None or configuration is None:
            raise ValueError('the movement carries no base frame or joint values')
        world_from_base = _matrix_from_frame(base_frame)
        placements = kinematics.link_matrices(
            world_from_base, configuration.joint_values, configuration.joint_names)
    except Exception as error:
        print(f"  ! {record['bar']}: robot not drawn ({error})")
        return None
    return {
        'base': _flatten(world_from_base),
        'links': {name: _flatten(matrix) for name, matrix in placements.items()
                  if name in geometry},
    }


def _matrix_from_frame(frame) -> np.ndarray:
    """A 4x4 placement matrix from a compas frame's origin and two axes.

    The frame stores x and y; z is their cross product. The axes are
    re-squared first so small rounding in the export cannot skew the robot.

    Args:
        frame: A compas ``Frame`` with ``point``, ``xaxis`` and ``yaxis``.

    Returns:
        numpy.ndarray: The 4x4 matrix.
    """
    origin = np.asarray(frame.point, dtype=float)
    x_axis = np.asarray(frame.xaxis, dtype=float)
    y_axis = np.asarray(frame.yaxis, dtype=float)
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = y_axis - x_axis * float(np.dot(x_axis, y_axis))
    y_axis = y_axis / np.linalg.norm(y_axis)
    matrix = np.eye(4)
    matrix[:3, 0] = x_axis
    matrix[:3, 1] = y_axis
    matrix[:3, 2] = np.cross(x_axis, y_axis)
    matrix[:3, 3] = origin
    return matrix


def _footprint_size(geometry: dict) -> list:
    """Overall size of the husky chassis, for the overview rectangles.

    Args:
        geometry (dict): ``{link_name: (points, faces)}`` in link frames.

    Returns:
        list: ``[length, width, height]`` in metres.
    """
    chassis = [name for name in geometry
               if 'wheel' in name or name in ('base_link', 'top_chassis_link',
                                              'front_bumper_link',
                                              'rear_bumper_link')]
    points = np.vstack([geometry[name][0] for name in chassis])
    return (points.max(axis=0) - points.min(axis=0)).tolist()


def _flatten(matrix: np.ndarray) -> list:
    """A 4x4 matrix as the 16 numbers three.js reads, column by column.

    Args:
        matrix (numpy.ndarray): The 4x4 matrix.

    Returns:
        list: Sixteen floats.
    """
    return [round(float(v), 6) for v in np.asarray(matrix).T.reshape(-1)]


# ---------------------------------------------------------------------------
# * The environment, and packing geometry small
# ---------------------------------------------------------------------------

def build_environment(env_3dm: str) -> dict:
    """Read the cell's solids and merge them into one drawable mesh.

    Args:
        env_3dm (str): The ``.3dm`` to read, or None to skip the environment.

    Returns:
        dict: ``{'positions': base64, 'indices': base64, 'count': int}``.
    """
    if not env_3dm or not os.path.exists(env_3dm):
        print("[env] no .3dm given; the scene will have no environment")
        return None
    solids = read_env_obstacles_3dm(env_3dm, ENV_LAYER)
    if not solids:
        print(f"[env] layer {ENV_LAYER!r} held no solids")
        return None

    positions, triangles, offset = [], [], 0
    for solid in solids:
        vertices = solid['verts']
        positions.extend(vertices)
        for face in solid['faces']:
            # * Rhino gives three- or four-sided faces; a quad becomes 2 triangles.
            triangles.append([offset + face[0], offset + face[1], offset + face[2]])
            if len(face) == 4:
                triangles.append([offset + face[0], offset + face[2],
                                  offset + face[3]])
        offset += len(vertices)

    print(f"[env] {len(solids)} solids, {len(positions)} points, "
          f"{len(triangles)} triangles")
    return {
        'positions': _pack(np.array(positions, dtype=np.float32)),
        'indices': _pack(np.array(triangles, dtype=np.uint32)),
        'count': len(solids),
    }


def build_cameras(env_3dm: str) -> list:
    """The Motive camera rig, as a position and an orientation per camera.

    Args:
        env_3dm (str): The ``.3dm`` to read, or None to skip the cameras.

    Returns:
        list: One record per camera, ready for the page to draw a cone from.
    """
    if not env_3dm or not os.path.exists(env_3dm):
        return []
    cameras = read_mocap_cameras_3dm(env_3dm, CAMERA_LAYER)
    return [{
        'name': camera['name'] or 'camera',
        'position': [round(v, 4) for v in camera['position']],
        # * The three axes become one placement matrix, so the page can orient a
        # * cone with a single call instead of redoing this arithmetic.
        'matrix': _flatten(_camera_matrix(camera)),
    } for camera in cameras]


def _camera_matrix(camera: dict) -> np.ndarray:
    """A placement matrix that stands a cone up along a camera's line of sight.

    The cone three.js builds points along its own +Y, which is also the axis the
    camera looks down, so the camera's three axes can be used as the matrix
    directly.

    Args:
        camera (dict): One entry from ``read_mocap_cameras_3dm``.

    Returns:
        numpy.ndarray: The 4x4 matrix.
    """
    matrix = np.eye(4)
    matrix[:3, 0] = camera['right']
    matrix[:3, 1] = camera['direction']
    matrix[:3, 2] = camera['up']
    matrix[:3, 3] = camera['position']
    return matrix


def _pack(array: np.ndarray) -> str:
    """Encode a numeric array as text the page can turn back into numbers.

    Writing thousands of coordinates as plain json text would roughly triple
    the file; this keeps the raw bytes and encodes them once.

    Args:
        array (numpy.ndarray): A float32 or uint32 array.

    Returns:
        str: Base64 of the array's bytes.
    """
    return base64.b64encode(array.tobytes()).decode('ascii')


# ---------------------------------------------------------------------------
# * Writing the page
# ---------------------------------------------------------------------------

def _check_three_js() -> tuple:
    """Read the three library files, or explain how to fetch them.

    Returns:
        tuple: ``(core, module, controls)`` as text.

    Raises:
        SystemExit: When the library files have not been downloaded.
    """
    missing = [p for p in (THREE_CORE, THREE_MODULE, ORBIT_CONTROLS)
               if not os.path.exists(p)]
    if missing:
        raise SystemExit(
            "The 3D library files are missing:\n  "
            + "\n  ".join(missing)
            + "\n\nThey are not kept in git. Download them once with:\n"
              "  scripts/fetch_dashboard_vendor.sh")
    return (open(THREE_CORE).read(), open(THREE_MODULE).read(),
            open(ORBIT_CONTROLS).read())


def embed_three_js() -> str:
    """Put the 3D library inside the page, so it needs no network.

    The library is three files. ``three.core.min.js`` stands alone;
    ``three.module.min.js`` loads it by relative path, which an embedded module
    cannot follow, so those two references are rewritten to a name the import
    map below defines; ``OrbitControls.js`` (mouse rotate/zoom/pan) asks for
    "three", which the map also supplies.

    Returns:
        str: The ``<script type="importmap">`` block.
    """
    core, module, controls = _check_three_js()
    rewritten = module.replace('"./three.core.min.js"', '"three-core"')
    if rewritten == module:
        raise SystemExit("three.module.min.js no longer refers to its core file "
                         "the expected way; the embedding needs updating.")

    def as_url(text: str) -> str:
        """Carry one javascript file inside the page as an address of its own."""
        return ('data:text/javascript;base64,'
                + base64.b64encode(text.encode()).decode('ascii'))

    imports = {
        'three-core': as_url(core),
        'three': as_url(rewritten),
        'three/addons/controls/OrbitControls.js': as_url(controls),
    }
    return ('<script type="importmap">'
            + json.dumps({'imports': imports})
            + '</script>')


def _specifiers(block: str, importing: bool = False) -> list:
    """Split an import/export name list into ``(outside, inside)`` pairs.

    ! The two directions are mirror images and mixing them up is silent.
    ! ``export{a as Vector3}`` means the file calls it ``a`` and the world sees
    ! ``Vector3``, so the outside name is on the RIGHT. ``import{Vector3 as e}``
    ! means the world calls it ``Vector3`` and the file calls it ``e``, so the
    ! outside name is on the LEFT.

    Args:
        block (str): The text between the braces.
        importing (bool): True for an import list, False for an export list.

    Returns:
        list: ``[(outside_name, inside_name), ...]``.
    """
    pairs = []
    for piece in block.split(','):
        piece = ' '.join(piece.split())
        if not piece:
            continue
        if ' as ' in piece:
            left, right = (part.strip() for part in piece.split(' as ', 1))
            pairs.append((left, right) if importing else (right, left))
        else:
            pairs.append((piece, piece))
    return pairs


def _as_scoped_module(source: str, name: str, incoming: str = None,
                      from_object: str = None) -> str:
    """Rewrite one ES module as a self-contained expression.

    The file's ``import`` becomes a plain destructuring from an object the
    caller already built, and its ``export`` becomes a returned object. Wrapping
    each file in its OWN function keeps their short minified names apart -- the
    core and the renderer build both use names like ``e`` and ``t`` at the top
    level, so pasting them into one scope would collide.

    Args:
        source (str): The module's text.
        name (str): For error messages.
        incoming (str): Regex matching the import statement to replace, if any.
        from_object (str): The expression the imported names come from.

    Returns:
        str: ``(function(){ ... return {...}; })()``.
    """
    # ! Handle `export{...}from"other.js"` BEFORE the plain `export{...}`. It is
    # ! a re-export: the names are forwarded straight out of the other file and
    # ! never exist in this one. Matching only its `export{...}` half would
    # ! leave a dangling `from"..."` (a syntax error) and would also claim
    # ! local variables that were never declared.
    given = []
    for match in list(re.finditer(
            r'export\s*\{([^}]*)\}\s*from\s*[\'"][^\'"]*[\'"]\s*;?', source)):
        for outside, inside in _specifiers(match.group(1)):
            given.append((outside, f'{from_object}["{inside}"]'))
    source = re.sub(r'export\s*\{[^}]*\}\s*from\s*[\'"][^\'"]*[\'"]\s*;?',
                    '', source)

    for match in list(re.finditer(r'export\s*\{([^}]*)\}\s*;?', source)):
        for outside, inside in _specifiers(match.group(1)):
            given.append((outside, inside))
    if not given:
        raise SystemExit(f"{name}: found no export list to bundle.")
    source = re.sub(r'export\s*\{[^}]*\}\s*;?', '', source)

    if incoming:
        match = re.search(incoming, source)
        if not match:
            raise SystemExit(f"{name}: its import statement no longer looks the "
                             f"way the bundler expects.")
        taken = _specifiers(match.group(1), importing=True)
        pulled = ','.join(f'{outside}:{inside}' for outside, inside in taken)
        source = (source[:match.start()] + f'const {{{pulled}}}={from_object};'
                  + source[match.end():])

    body = ','.join(f'"{outside}":{expression}' for outside, expression in given)
    return f'(function(){{\n{source}\nreturn {{{body}}};\n}})()'


def bundle_three_js(viewer_js: str) -> str:
    """The 3D library and the viewer as ONE ordinary inline script.

    ! This is what a page needs when it is SERVED rather than opened from disk.
    ! A host may wrap the page inside a document of its own and serve it from a
    ! path where neither a ``data:`` URL nor a relative filename resolves, so
    ! an import map and sibling files both fail and the page comes up blank.
    ! Nothing here is imported, fetched or mapped: the library, its controls and
    ! the viewer are one script with no outside reference at all.

    Args:
        viewer_js (str): The viewer module, whose own imports are removed.

    Returns:
        str: A ``<script>`` body.
    """
    core, module, controls = _check_three_js()
    core_expr = _as_scoped_module(core, 'three.core.min.js')
    module_expr = _as_scoped_module(
        module, 'three.module.min.js',
        incoming=r'import\s*\{([^}]*)\}\s*from\s*"\./three\.core\.min\.js"\s*;?',
        from_object='__core')
    controls_expr = _as_scoped_module(
        controls, 'OrbitControls.js',
        incoming=r'import\s*\{([^}]*)\}\s*from\s*[\'"]three[\'"]\s*;?',
        from_object='THREE')

    # The viewer asks for the library by name; here it is already in scope.
    body = re.sub(r'^\s*import\s[^\n]*\n', '', viewer_js, flags=re.M)
    return (
        '(function(){\n'
        '"use strict";\n'
        f'const __core={core_expr};\n'
        f'const __three={module_expr};\n'
        # * The renderer build re-exports most of the core, but merging both is
        # * what guarantees every name the viewer reaches for is present.
        'const THREE=Object.assign({},__core,__three);\n'
        f'const __controls={controls_expr};\n'
        'const OrbitControls=__controls["OrbitControls"];\n'
        f'\n{body}\n'
        '})();'
    )


def sibling_three_js(out_path: str) -> str:
    """Write the 3D library next to the page and point the import map at it.

    ! Needed wherever the page is SERVED rather than opened from disk. A hosted
    ! page usually runs under a content-security policy, and those routinely
    ! refuse to execute a ``data:`` script -- which is how the embedded build
    ! carries the library, so the page comes up blank with the whole module
    ! blocked. Plain sibling files are fetched normally and pass.

    The page is then no longer a single file: the three ``.js`` files have to
    travel with it.

    Args:
        out_path (str): The ``.html`` being written; the library lands beside it.

    Returns:
        str: The ``<script type="importmap">`` block.
    """
    core, module, controls = _check_three_js()
    beside = os.path.dirname(os.path.abspath(out_path))
    for name, text in (('three.core.min.js', core),
                       ('three.module.min.js', module),   # keeps its own import
                       ('OrbitControls.js', controls)):
        with open(os.path.join(beside, name), 'w') as handle:
            handle.write(text)
    imports = {
        'three': './three.module.min.js',
        'three/addons/controls/OrbitControls.js': './OrbitControls.js',
    }
    return ('<script type="importmap">'
            + json.dumps({'imports': imports})
            + '</script>')


def write_page(scene: dict, out_path: str, vendor: str = 'embed') -> None:
    """Write the finished page.

    Args:
        scene (dict): Everything the page draws.
        out_path (str): Where to write the ``.html``.
        vendor (str): ``'embed'`` keeps the 3D library inside the file, so it
            travels alone; ``'sibling'`` writes the library beside it, which is
            what a hosted copy needs.
    """
    template_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 '2_session_viewer.js')
    with open(template_path, 'r') as handle:
        viewer_js = handle.read()

    if vendor == 'inline':
        # One ordinary script, nothing imported or fetched: the only form that
        # survives being served inside someone else's page.
        importmap, script_open = '', '<script>'
        viewer_js = bundle_three_js(viewer_js)
    else:
        importmap = (sibling_three_js(out_path) if vendor == 'sibling'
                     else embed_three_js())
        script_open = '<script type="module">'
    html = _PAGE_TEMPLATE.format(
        title=scene['title'],
        importmap=importmap,
        script_open=script_open,
        data=json.dumps(scene, separators=(',', ':')),
        viewer_js=viewer_js,
    )
    with open(out_path, 'w') as handle:
        handle.write(html)

    size_mb = os.path.getsize(out_path) / (1024 * 1024)
    # ? Count only what the browser would actually FETCH. Plain text matches are
    # ? no use now that the library is inlined unencoded: its source carries an
    # ? XML namespace URI and a citation in a shader comment, neither a request.
    external = len(re.findall(r'(?:src|href)\s*=\s*["\']https?://', html))
    print(f"\n[page] {out_path}")
    print(f"       {size_mb:.1f} MB, {external} external references "
          f"({'fully offline' if external == 0 else 'NOT offline!'})")
    if vendor == 'sibling':
        print("       the 3D library sits beside it as three .js files; they "
              "must travel with the page")


_PAGE_TEMPLATE = """<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>{title}</title>
{importmap}
<style>
:root {{ --bg:#ffffff; --panel:#f6f7f9; --line:#d9dee5; --text:#1d2430;
         --dim:#6b7686; --accent:#1763b8; }}
* {{ box-sizing:border-box; }}
html,body {{ margin:0; height:100%; background:var(--bg); color:var(--text);
  font:13px/1.5 ui-sans-serif,system-ui,-apple-system,Segoe UI,Roboto,sans-serif; }}
#app {{ display:flex; height:100%; }}
#scene {{ flex:1; position:relative; min-width:0; }}
#scene canvas {{ display:block; }}
#side {{ width:300px; flex:none; background:var(--panel);
  border-left:1px solid var(--line); overflow-y:auto; padding:16px; }}
h1 {{ font-size:15px; margin:0 0 2px; }}
h2 {{ font-size:11px; letter-spacing:.08em; text-transform:uppercase;
  color:var(--dim); margin:16px 0 6px; font-weight:600; }}
.sub {{ color:var(--dim); font-size:11px; margin-bottom:4px; }}
.row {{ display:flex; justify-content:space-between; gap:10px; padding:3px 0;
  border-bottom:1px solid rgba(0,0,0,.06); }}
.row span:last-child {{ font-variant-numeric:tabular-nums; }}
.muted {{ color:var(--dim); }}
.warn {{ color:#b45309; }}
select,button {{ background:#fff; color:var(--text);
  border:1px solid var(--line); border-radius:5px; padding:5px 8px;
  font:inherit; width:100%; }}
button {{ cursor:pointer; margin-top:6px; }}
button:hover {{ border-color:var(--accent); }}
label.chk {{ display:flex; align-items:center; gap:7px; padding:3px 0;
  cursor:pointer; }}
/* * The two cards floating over the scene: the colour key, and the detail card
   * that opens directly above it once a bar is selected. */
.card {{ position:absolute; left:14px; background:rgba(255,255,255,.96);
  border:1px solid var(--line); border-radius:8px; width:336px;
  box-shadow:0 2px 14px rgba(20,30,50,.13); }}
#legend {{ bottom:14px; padding:10px 12px; }}
/* * Tall enough to show every number without scrolling, while still clearing
   * the hint text at the top and the colour key below. */
#detail {{ padding:12px 14px; display:none;
  max-height:calc(100vh - 250px); overflow-y:auto; }}
#detail.open {{ display:block; }}
#detail h2:first-of-type {{ margin-top:0; }}
/* * The card is one block per measurement, and the one the "data by" picker
   * asks for is marked and set in bold -- so clicking a bar answers the
   * question the picker asked instead of leaving it among the others. */
.block h2 {{ margin-top:0; }}
.block {{ margin-top:16px; }}
.block.focus {{ margin:16px -9px 0; padding:8px 9px 7px; border-radius:6px;
  background:#eef4fc; border:1px solid #c7dbf4; }}
.pill {{ margin-left:7px; padding:1px 5px; border-radius:3px;
  background:var(--accent); color:#fff; font-size:9px; letter-spacing:.06em; }}
#detail-close {{ position:absolute; top:7px; right:9px; width:auto; margin:0;
  padding:0 7px 2px; font-size:16px; line-height:1.3; color:var(--dim);
  background:transparent; border-color:transparent; }}
#detail-close:hover {{ color:var(--text); border-color:var(--line); }}
#legend-head {{ display:flex; align-items:center; gap:8px; margin-bottom:2px; }}
/* ! The layer toggles live in this floating card, NOT in the side panel: the
   ! panel is a fixed strip at the right edge and falls off screen entirely in a
   ! window wider than the display, taking its controls with it. */
#layers {{ margin-top:9px; padding-top:8px; border-top:1px solid var(--line);
  display:flex; flex-wrap:wrap; gap:3px 10px; align-items:center; }}
#layers label {{ display:flex; align-items:center; gap:4px; cursor:pointer;
  font-size:11px; color:var(--dim); white-space:nowrap; }}
/* * Tick boxes in the environment's own blue, so the controls read as part of
   * the same drawing rather than borrowing the browser's accent colour. */
#layers input, #side input {{ margin:0; accent-color:#4a7fb5; }}
#reset {{ width:auto; margin:0 0 0 auto; padding:2px 8px; font-size:11px; }}
#legend-head span {{ color:var(--dim); font-size:10px; letter-spacing:.07em;
  text-transform:uppercase; white-space:nowrap; }}
#metric-legend {{ flex:1; padding:3px 6px; font-size:12px; }}
#ramp {{ height:11px; border-radius:3px; margin:6px 0 3px;
  border:1px solid rgba(0,0,0,.10); }}
#ticks {{ display:flex; justify-content:space-between; color:var(--dim);
  font-size:10px; font-variant-numeric:tabular-nums; }}
#bands {{ margin-top:7px; font-size:11px; color:var(--dim); }}
.swatch {{ display:inline-block; width:9px; height:9px; border-radius:2px;
  margin-right:5px; vertical-align:-1px; border:1px solid rgba(0,0,0,.12); }}
#hint {{ position:absolute; top:12px; left:14px; color:var(--dim);
  font-size:11px; line-height:1.7; }}
.formula {{ margin:9px 0 0; }}
.formula code {{ display:block; margin:3px 0 2px; padding:4px 7px;
  background:#eef2f7; border:1px solid #dde4ec; border-radius:4px;
  font-size:11.5px; color:#1d3f72; overflow-wrap:anywhere; }}
#formula-toggle {{ width:auto; padding:3px 9px; font-size:11px; }}
#detail {{ scrollbar-width:thin; scrollbar-color:#4a7fb5 transparent; }}
#detail::-webkit-scrollbar {{ width:9px; }}
#detail::-webkit-scrollbar-thumb {{ background:#4a7fb5; border-radius:5px; }}
#detail::-webkit-scrollbar-track {{ background:transparent; }}
.flag {{ margin:8px 0 0; padding:7px 9px; border-radius:6px;
  background:#fff6f5; border:1px solid #e2a79c; color:#7a2318; font-size:11.5px; }}
#boot {{ display:none; }}
#boot.boot-error {{ display:block; position:absolute; top:70px; left:14px;
  max-width:560px; background:#fff6f5; border:1px solid #e2a79c;
  border-radius:8px; padding:14px 16px; font-size:12.5px; line-height:1.6;
  color:#7a2318; }}
#boot code {{ background:#f3e2de; padding:1px 4px; border-radius:3px; }}
#status {{ margin-top:12px; padding-top:10px; border-top:1px solid var(--line);
  color:var(--dim); font-size:11px; }}
</style></head><body>
<div id="app">
  <div id="scene">
    <div id="hint">left drag to rotate, right drag to move, scroll to zoom<br>
      left click a bar to select and see detail, ESC or the card's &times;
      to unpin</div>
    <div class="card" id="detail"></div>
    <div class="card" id="legend">
      <div id="legend-head"><span>colour by</span><select id="metric-legend"></select></div>
      <div id="legend-head"><span style="min-width:52px">data by</span>
        <select id="data-legend"></select></div>
      <div id="legend-body"></div>
      <div id="layers">
        <label><input type="checkbox" id="show-env" checked> environment</label>
        <label><input type="checkbox" id="show-authored" checked> authored</label>
        <label><input type="checkbox" id="show-markers" checked> markers</label>
        <label><input type="checkbox" id="show-robots" checked> robots</label>
        <label><input type="checkbox" id="show-cameras"> mocap cameras</label>
        <label><input type="checkbox" id="show-labels" checked> labels</label>
        <button id="reset">reset view</button>
      </div>
    </div>
  </div>
  <div id="side">
    <h1>{title}</h1>
    <div class="sub" id="subtitle"></div>
    <div class="sub" id="metric-help" style="margin-top:10px"></div>
    <div id="status"></div>
  </div>
</div>
<div id="boot"></div>
<script>
/* ! A module that fails to load leaves a BLANK page -- no error, nothing. That
   ! is exactly what happens when a host's content-security policy refuses the
   ! script, so say so on the page instead of showing nothing. */
(function () {{
  var box = document.getElementById('boot');
  function fail(what) {{
    if (window.__viewerReady) return;
    box.className = 'boot-error';
    box.innerHTML = '<b>The 3D view did not start.</b><br>' + what
      + '<br><br>If this page is being served rather than opened from a file, '
      + 'its 3D library may have been blocked. Re-generate it with '
      + '<code>--vendor sibling</code> and keep the three .js files beside it.';
  }}
  window.addEventListener('error', function (e) {{
    fail(e.message ? ('<code>' + e.message + '</code>') : 'A script was blocked or failed to load.');
  }}, true);
  setTimeout(function () {{ fail('It did not finish starting.'); }}, 6000);
}})();
</script>
<script type="application/json" id="scene-data">{data}</script>
{script_open}
{viewer_js}
</script></body></html>
"""


# ---------------------------------------------------------------------------
# * Putting a session together
# ---------------------------------------------------------------------------

def collect_session(batch_dir: str, with_robots: bool) -> dict:
    """Read a whole session and shape it into what the page draws.

    The environment is added afterwards by the caller, because which ``.3dm``
    to read is only known once the takes have named their design problem.

    Args:
        batch_dir (str): The session folder, e.g. ``.../20261001``.
        with_robots (bool): Whether to work out the robot poses.

    Returns:
        dict: The scene, without its environment.
    """
    runs_dir = batch_dir + '-servoing'
    takes = sorted(os.path.join(batch_dir, f) for f in os.listdir(batch_dir)
                   if f.startswith('bar_holding_acc_') and f.endswith('.json'))
    runs = sorted(os.path.join(runs_dir, f) for f in os.listdir(runs_dir)
                  if f.startswith('servoing_data_') and f.endswith('.json')) \
        if os.path.isdir(runs_dir) else []
    if not takes:
        raise SystemExit(f"No take files in {batch_dir}")
    print(f"[session] {len(takes)} takes, {len(runs)} servo runs")

    pairing = pair_runs_to_takes(takes, runs)
    check_pairing(pairing, runs)

    print("\n[measure]")
    records = []
    for take in takes:
        record = measure_take(take)
        if record is None:
            continue
        add_tip_distances(record)
        add_servo_numbers(record, pairing[take])
        records.append(record)
        print(f"  {record['bar']:>5s}  placement {record['placement']:6.2f} mm "
              f"(worst of {record['tip_start']:.2f} / "
              f"{record['tip_middle']:.2f} / {record['tip_end']:.2f})  "
              f"rotation {record['rotation']:6.3f} deg  "
              f"load gap {_show(record['load'])}"
              f"{'' if record['load_valid'] else '  (load not measured)'}")

    shared = build_robots(records) if with_robots else None
    # ? After the robots, so a resolved wrist can be used; without one this
    # ? falls back to a fixed point near each end of the bar.
    for record in records:
        add_grasp_points(record)
    return {
        'title': f"bar-holding session {os.path.basename(batch_dir)}",
        'generated': datetime.now().strftime('%Y-%m-%d %H:%M'),
        'metrics': METRICS,
        'extra_formulas': EXTRA_FORMULAS,
        'bars': records,
        'robot': shared,
        'env': None,
        'cameras': [],
        'cone': {'length': CAMERA_CONE_M,
                 'half_angle_deg': CAMERA_CONE_HALF_ANGLE_DEG},
    }


def _show(value: float) -> str:
    """Format an optional number for the console.

    Args:
        value: A number or None.

    Returns:
        str: Right-aligned text.
    """
    return '    --' if value is None else f"{value:6.2f} N"


def resolve_env_3dm(records: list, override: str) -> str:
    """Find the ``.3dm`` whose ``env`` layer describes this session's cell.

    Args:
        records (list): Bar records, used to read the stamped problem name.
        override (str): An explicit path from the command line, or None.

    Returns:
        str: The chosen ``.3dm`` path, or None.
    """
    stamped = next((r.get('bar_action_path') for r in records
                    if r.get('bar_action_path')), None)
    chosen = env_3dm_for_bar_action(stamped, override)
    if chosen:
        print(f"[env] using {os.path.basename(chosen)}")
    return chosen


def main() -> None:
    """Read the session named on the command line and write its page."""
    parser = argparse.ArgumentParser(
        description='Build a 3D web page for one bar-holding accuracy session.')
    parser.add_argument('batch', nargs='?', default=None,
                        help='session folder name, e.g. 20261001 '
                             '(default: the newest session on disk)')
    parser.add_argument('--env-3dm', default=None,
                        help='.3dm to read the environment from '
                             '(default: the session design problem)')
    parser.add_argument('--no-robots', action='store_true',
                        help='skip the robot poses (faster)')
    parser.add_argument('--no-cameras', action='store_true',
                        help='skip the mocap camera rig')
    parser.add_argument('--vendor', choices=('inline', 'embed', 'sibling'), default='inline',
                        help="where the 3D library goes: 'inline' bundles it "
                             "into one ordinary script (default -- works both "
                             "from disk and when served); 'embed' uses an "
                             "import map of data: URLs; 'sibling' writes it "
                             "beside the page as three .js files")
    parser.add_argument('--out', default=None, help='output .html path')
    parser.add_argument('--open', action='store_true',
                        help='open the page when it is written')
    args = parser.parse_args()

    batch = args.batch or latest_batch_folder()
    if not batch:
        raise SystemExit("no session folder found; pass one, e.g. 20261001")
    if not args.batch:
        print(f"[session] no batch given; using the newest one: {batch}")

    batch_dir = os.path.join(EXPERIMENT_DATA_DIRECTORY, 'bar_holding_acc_data',
                             batch)
    if not os.path.isdir(batch_dir):
        raise SystemExit(f"No such session folder: {batch_dir}")

    # ? The environment needs the problem name, which only the takes know, so
    # ? the session is read first and the environment added once it is known.
    scene = collect_session(batch_dir, not args.no_robots)
    env_3dm = resolve_env_3dm(scene['bars'], args.env_3dm)
    scene['env'] = build_environment(env_3dm)
    scene['cameras'] = [] if args.no_cameras else build_cameras(env_3dm)

    out_path = args.out or os.path.join(batch_dir + '-result',
                                        f'session_{batch}.html')
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    write_page(scene, out_path, args.vendor)
    if args.open:
        webbrowser.open('file://' + os.path.abspath(out_path))


if __name__ == '__main__':
    main()
