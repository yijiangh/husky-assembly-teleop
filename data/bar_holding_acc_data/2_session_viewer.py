"""Build one self-contained 3D web page for a bar-holding accuracy session.

Where `0_` fits the bars and `1_` compares them to the cell state one panel at a
time, this draws the whole session at once: every bar where mocap says it landed,
coloured by how far off it was, with the cell around it and the robot at each
parking spot. Click a bar to read its servo and load numbers.

The page is a single `.html` file with the 3D library inside it, so it opens in
any browser with no server, no install and no internet connection.

Run it with the session folder name, the same way as the other two scripts::

    python 2_session_viewer.py 20261001

which writes ``<session>-viz/session_<session>.html`` next to the data.

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
     'help': 'how far the bar ended up from where it was meant to be, '
             'measured by mocap against the authored pose'},
    {'key': 'rotation', 'label': 'rotation error', 'unit': 'deg', 'high': 0.25,
     'help': 'angle between the bar axis as built and as authored'},
    {'key': 'servo', 'label': 'servo residual', 'unit': 'mm', 'high': 1.5,
     'help': 'how far the worse wrist still was from its target on the last '
             'servo iteration, before the markers were recorded'},
    {'key': 'load', 'label': 'load imbalance', 'unit': 'N', 'high': 2.0,
     'help': 'gap between the two wrists force magnitudes. The two grippers '
             'pull against each other through the bar, so a BALANCED pair is a '
             'bar in clean axial load; the gap is the part the bar has to '
             'absorb sideways, i.e. bending'},
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
        'placement': float(np.mean(placements)),
        'placement_spread': float(np.ptp(placements)),
        'rotation': float(np.mean(rotations)),
        'fit_residual': float(np.mean(residuals)),
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


def add_servo_numbers(record: dict, run_path: str) -> None:
    """Attach the paired run's last iteration and the load reading to a bar.

    Args:
        record (dict): A bar record from :func:`measure_take`, edited in place.
        run_path (str): The servo run matched to it, or None.
    """
    record['run'] = os.path.basename(run_path) if run_path else None
    record['servo'] = None
    record['iterations'] = None
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
        resolved += bool(matrices)
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

def embed_three_js() -> str:
    """Put the 3D library inside the page, so it needs no network.

    The library is three files. ``three.core.min.js`` stands alone;
    ``three.module.min.js`` loads it by relative path, which an embedded module
    cannot follow, so those two references are rewritten to a name the import
    map below defines; ``OrbitControls.js`` (mouse rotate/zoom/pan) asks for
    "three", which the map also supplies.

    Returns:
        str: The ``<script type="importmap">`` block.

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

    core = open(THREE_CORE).read()
    module = open(THREE_MODULE).read()
    controls = open(ORBIT_CONTROLS).read()
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


def write_page(scene: dict, out_path: str) -> None:
    """Write the finished single-file page.

    Args:
        scene (dict): Everything the page draws.
        out_path (str): Where to write the ``.html``.
    """
    template_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 '2_session_viewer.js')
    with open(template_path, 'r') as handle:
        viewer_js = handle.read()

    html = _PAGE_TEMPLATE.format(
        title=scene['title'],
        importmap=embed_three_js(),
        data=json.dumps(scene, separators=(',', ':')),
        viewer_js=viewer_js,
    )
    with open(out_path, 'w') as handle:
        handle.write(html)

    size_mb = os.path.getsize(out_path) / (1024 * 1024)
    external = html.count('http://') + html.count('https://')
    print(f"\n[page] {out_path}")
    print(f"       {size_mb:.1f} MB, {external} external references "
          f"({'fully offline' if external == 0 else 'NOT offline!'})")


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
#status {{ margin-top:12px; padding-top:10px; border-top:1px solid var(--line);
  color:var(--dim); font-size:11px; }}
</style></head><body>
<div id="app">
  <div id="scene">
    <div id="hint">left drag to rotate, right drag to move, scroll to zoom<br>
      left click a bar to select and see detail, right click or ESC to unpin</div>
    <div class="card" id="detail"></div>
    <div class="card" id="legend">
      <div id="legend-head"><span>colour by</span><select id="metric-legend"></select></div>
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
<script type="application/json" id="scene-data">{data}</script>
<script type="module">
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
        add_servo_numbers(record, pairing[take])
        records.append(record)
        print(f"  {record['bar']:>5s}  placement {record['placement']:6.2f} mm  "
              f"rotation {record['rotation']:6.3f} deg  "
              f"load gap {_show(record['load'])}"
              f"{'' if record['load_valid'] else '  (load not measured)'}")

    shared = build_robots(records) if with_robots else None
    return {
        'title': f"bar-holding session {os.path.basename(batch_dir)}",
        'generated': datetime.now().strftime('%Y-%m-%d %H:%M'),
        'metrics': METRICS,
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

    out_path = args.out or os.path.join(batch_dir + '-viz',
                                        f'session_{batch}.html')
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    write_page(scene, out_path)
    if args.open:
        webbrowser.open('file://' + os.path.abspath(out_path))


if __name__ == '__main__':
    main()
