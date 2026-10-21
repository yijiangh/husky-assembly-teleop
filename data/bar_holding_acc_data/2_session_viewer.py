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

* Everything a 3D page shares with the other viewers -- the environment, the
* camera rig, the robot's link shapes, the 3D library and the page itself --
* lives in ``viewer_common.py`` (and ``viewer_common.js`` for the browser
* half). This file keeps what is specific to a bar-holding session: reading
* the takes, pairing them to the servo runs, and measuring each bar.
"""

import argparse
import os
import sys
import webbrowser
from datetime import datetime

# ! Running this file directly puts only its OWN folder on the import path, not
# ! the repo, so the package import below fails with ModuleNotFoundError even
# ! when you are sitting in the repo root. Put the repo root on the path first.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
# ! And this folder itself, for the shared viewer module: a script loaded by
# ! path (as 3_session_report.py does with this one) does not get it for free.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

from husky_assembly_teleop import EXPERIMENT_DATA_DIRECTORY
from husky_assembly_teleop.legacy_bar_action_io import resolve_take_movement
from husky_assembly_teleop.mocap_experiment import (
    fit_bar_from_markerset,
    bar_deviation_from_goal,
    pair_fit_to_goal,
    tip_distances,
    make_axis_corrector,
    convert_markerset_axes,
    _reroot_gdrive_path,
    latest_batch_folder,
    env_3dm_for_bar_action,
)
from viewer_common import (
    CAMERA_CONE_M,
    CAMERA_CONE_HALF_ANGLE_DEG,
    file_timestamp,
    load_json,
    load_kinematics,
    robot_matrices,
    pack_link_meshes,
    _closest_point_on_segment,
    matrix_from_frame,
    footprint_size,
    build_environment,
    build_cameras,
    viewer_script,
    write_page,
)


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
        take_force = _take_forces(load_json(take))
        run_force = _run_last_forces(load_json(run))
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
    data = load_json(path)
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
        iterations = load_json(run_path).get('servoing_data') or []
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

    kinematics, geometry = load_kinematics(HUSKY_DUAL_URDF_PATH)
    # * Every robot in the session is the same machine, so the link shapes are
    # * sent ONCE and each robot adds only one matrix per link. That is what
    # * makes real geometry affordable here instead of crude boxes.
    link_meshes = pack_link_meshes(geometry)

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
    return {'meshes': link_meshes, 'footprint': footprint_size(geometry)}


# * How far a wrist may sit from the bar before the pose is not a grasp. The
# * grippers hold the bar 80 mm off its axis, so anything near that is right and
# * anything far past it means the configuration and the bar disagree.
GRASP_OFFSET_MM = 80.0
GRASP_OFFSET_TOLERANCE_MM = 250.0


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
        return robot_matrices(kinematics, geometry, matrix_from_frame(base_frame),
                              configuration.joint_values, configuration.joint_names)
    except Exception as error:
        print(f"  ! {record['bar']}: robot not drawn ({error})")
        return None


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
    write_page(scene, out_path, args.vendor, viewer_script('2_session_viewer.js'))
    if args.open:
        webbrowser.open('file://' + os.path.abspath(out_path))


if __name__ == '__main__':
    main()
