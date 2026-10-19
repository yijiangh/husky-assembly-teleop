"""Smoke test: the 'MoCap bar' panel through the real HuskyMonitor methods (no Motive needed).

No ROS, no hardware, no window. The monitor is built without its ``__init__``
(``headless_plan_skeleton_smoke.make_monitor``: a stub robot, the cfab session
in PyBullet DIRECT mode) on a scratch copy of the design problem, with the
desk-rehearsal flag ``MOCAP_BAR_FAKE_MARKERS = 1``. It loads action
``B20_J_joint`` the way 'Load action' does and then:

1. checks the load lines: ``[mocap bars] 2 instrumented bar(s): bar_B20 (8),
   bar_B21 (8) | ...`` and the marker-table check (< 0.05 mm over 8 markers);
   with a made-up Motive asset list (B20, B21, B34, B35) the line naming the
   bar assets without a marker table, once;
2. one bar (``measure_bar_pose``) on the made-up markers (B20's design markers
   at its Rhino pose, slid 30 mm along the bar, rolled 2 deg, plus a stray):
   the fit is ok with 8/8 matched and the stray unmatched, the deviation reads
   (0, 0, 30) mm in the bar frame and 2 deg, both copies of the bar + its 8
   joints are built, the pink one at the Rhino pose, the cyan one at the
   slid + rolled pose;
3. the same markers minus three, handed to the monitor as Motive would
   (``receive_labeled_marker``, model id 0 = unlabelled): the fit fails with
   "5 of 8", the cyan copy is hidden. Then all of them under model id 1020
   (B20's asset by the lab convention Bxx -> 10xx): cached as ``bar_B20``,
   fit ok with source 'asset'; then nothing at all: a plain "no markers from
   Motive yet" message;
4. 'MoCap bar: measure + draw (streaming bars)' (``measure_mocap_bars``) with
   B20's made-up markers under 1020 and B21's design markers at its Rhino pose
   slid 10 mm under 1021: both bars fit and are drawn, with the header line;
   'follow live' (``log=False``) twice: the fits start from the last pose
   (``tracked``), the time per call is printed; the 'Rhino bar lines (pink)'
   checkbox off and on (the pink Rhino lines of the MoCap drawings and the
   assembled-bar lines go and come back); B21's asset goes stale: its drawing
   is removed, B20's kept;
5. 'MoCap bar: record take' (one take per streaming bar, each with its own
   marker table and Rhino pose) and 'Save markerset data' (into the scratch
   folder);
6. renders the copies (PyBullet's TinyRenderer, the cell's own B20 and its
   joints moved out of the way) to a PNG. Debug lines and text do not show in
   DIRECT mode; the PNG is for the bar + joint copies.

! It ALWAYS runs on a scratch copy of the problem folder in a temp dir (the
! RobotCell files are symlinked and only read); the last check confirms the
! real design folder is untouched.

Usage:

    cd /home/yijiangh/Code/ros2_ws && source venv/bin/activate && source install/setup.bash
    INSYNC="/home/yijiangh/Insync/yijiang94817@gmail.com/Google Drive - Shared with me"
    export DESIGN_DATA_DIRECTORY="$INSYNC/2025-03 Husky Assembly/data_design_study"
    export DESIGN_PROBLEM_NAME=261016_2ModulesDemo_YH
    python src/husky-assembly-teleop/scripts/headless_mocap_bar_smoke.py [--out overlay.png] [--keep]

Exit code 0 when every check passes, 1 otherwise.
"""

import argparse
import glob
import json
import os
import re
import shutil
import sys
import tempfile
import threading
import time
import traceback
from types import SimpleNamespace

import numpy as np
import pybullet as p
from PIL import Image
from pybullet_planning import tform_from_pose

# The plan skeleton smoke in this same folder (importable because Python puts
# the script's folder on sys.path).
from headless_plan_skeleton_smoke import (
    Results, add_viz_huskies, call_keeping_prints, folder_snapshot, make_monitor, make_scratch_problem,
    point_package_at, select_movement,
)
from husky_assembly_teleop import DESIGN_DATA_DIRECTORY, DESIGN_PROBLEM_NAME
from husky_assembly_teleop import husky_world
from husky_assembly_teleop.bar_action_io import bar_id_of_body, joints_on_bar
from husky_assembly_teleop.bar_mocap_fit import MOCAP_MARKERS_FILENAME, bar_slide_roll, transform_points
from husky_assembly_teleop.mocap_bar_viz import bar_end_points
from husky_assembly_teleop.plan_skeleton_io import load_plan_skeleton, problem_root
from husky_assembly_teleop.robot_registry import robot_by_name

ACTION_ID = 'B20_J_joint'
BAR = 'bar_B20'
OTHER_BAR = 'bar_B21'
N_JOINTS = 8
# * Motive's asset list as seen 2026-10-16 (name, streaming id): B34 / B35 have no marker table.
MOTIVE_ASSETS = (('B20', 1020), ('B21', 1021), ('B34', 1034), ('B35', 1035))
# * What the made-up markers are moved by (fake_sensed_markers' defaults).
SLIDE_M = 0.03
ROLL_DEG = 2.0
# * Tolerances: the marker table vs the joint frames; the fitted deviation vs
# * the made-up offset (0.5 mm marker noise); the cyan copy's bar vs the
# * slid + rolled pose.
TABLE_TOL_MM = 0.05
DPOS_TOL_MM = 1.5
DANG_TOL_DEG = 0.5
GHOST_TOL_MM = 1.5
GHOST_TOL_DEG = 0.3
# How many matched markers the second measurement loses.
N_DROPPED = 3
# * B21's markers are its design markers at its Rhino pose, slid this far along the bar.
OTHER_SLIDE_M = 0.01
# How many 'follow live' calls the timing is the median of.
N_LIVE_TIMED = 20
# * The PNG: a camera this far from B20's middle, above and to the side.
CAMERA_DISTANCE_M = 1.2
IMAGE_SIZE = (960, 720)
DEFAULT_OUT = os.path.join(tempfile.gettempdir(), 'mocap_bar_overlay.png')
# What every take of 'MoCap bar: record take' holds, besides the bar's own marker snapshot.
TAKE_KEYS = {'markerset_key', 'bar_fit', 'deviation', 'marker_source', 'marker_age_s', 'tool_ft',
             'design_markers', 'design_T_world_from_bar'}


def body_pose_error(client: int, body: int, T: np.ndarray) -> tuple:
    """How far a PyBullet body is from a 4x4 pose.

    Args:
        client (int): PyBullet client.
        body (int): Body id.
        T (np.ndarray): 4x4 pose it should have.

    Returns:
        tuple: ``(distance mm, angle deg)``.
    """
    pos, quat = p.getBasePositionAndOrientation(body, physicsClientId=client)
    R = np.reshape(p.getMatrixFromQuaternion(quat), (3, 3))
    cos = np.clip((np.trace(T[:3, :3].T @ R) - 1.0) / 2.0, -1.0, 1.0)
    return 1000.0 * float(np.linalg.norm(np.asarray(pos) - T[:3, 3])), float(np.degrees(np.arccos(cos)))


def as_natnet_frame(markers: dict, model_id: int = 0) -> dict:
    """Marker snapshot (z-up, 'rhino' axes) as NatNet hands it to the listener: y-up, under one model id.

    Two models in one frame: ``{**as_natnet_frame(a, 1020), **as_natnet_frame(b, 1021)}``.

    Args:
        markers (dict): ``{id: {'pos', 'size', 'error'}}`` in the monitor's z-up world.
        model_id (int): 0 = Motive's unlabelled markers; else the asset's streaming id.

    Returns:
        dict: ``{model_id: {id: {'pos': y-up tuple, 'size': 1-tuple, 'error'}}}``.
    """
    # The 'rhino' convention reads (x, y, z) = (mocap x, -mocap z, mocap y).
    return {model_id: {marker_id: {'pos': (m['pos'][0], m['pos'][2], -m['pos'][1]), 'size': tuple(m['size']),
                                   'error': m['error']}
                       for marker_id, m in markers.items()}}


def render_overlay(monitor, T_design: np.ndarray, z_range: tuple, joint_names: list, out_png: str) -> np.ndarray:
    """Render the bar copies with TinyRenderer and save a PNG.

    The cell's own bar and joint bodies sit exactly where the pink copy is,
    so they are moved out of the way for the picture and put back.

    Args:
        monitor (HuskyMonitor): The headless monitor (its cfab session holds the scene).
        T_design (np.ndarray): 4x4 Rhino pose of the bar.
        z_range (tuple): The bar's ``(z_min, z_max)``.
        joint_names (list): The joints on the bar.
        out_png (str): Where to save.

    Returns:
        np.ndarray: (H, W, 3) the image.
    """
    client = monitor.cfab.client.client_id
    ends = bar_end_points(T_design, z_range)
    centre = ends.mean(axis=0)
    side = np.cross(ends[1] - ends[0], [0.0, 0.0, 1.0])
    view_dir = 0.6 * side / np.linalg.norm(side) + np.array([0.0, 0.0, 0.8])
    eye = centre + CAMERA_DISTANCE_M * view_dir / np.linalg.norm(view_dir)
    cell_bodies = [body for name in [BAR] + joint_names
                   for body in monitor.cfab.client.rigid_bodies_puids.get(name, [])]
    poses = [p.getBasePositionAndOrientation(body, physicsClientId=client) for body in cell_bodies]
    try:
        for body in cell_bodies:
            p.resetBasePositionAndOrientation(body, (0.0, 0.0, -50.0), (0.0, 0.0, 0.0, 1.0), physicsClientId=client)
        width, height = IMAGE_SIZE
        view = p.computeViewMatrix(eye.tolist(), centre.tolist(), [0.0, 0.0, 1.0])
        projection = p.computeProjectionMatrixFOV(fov=60.0, aspect=width / height, nearVal=0.02, farVal=20.0)
        _w, _h, rgba, _depth, _seg = p.getCameraImage(width, height, view, projection,
                                                      renderer=p.ER_TINY_RENDERER, physicsClientId=client)
    finally:
        for body, (pos, quat) in zip(cell_bodies, poses):
            p.resetBasePositionAndOrientation(body, pos, quat, physicsClientId=client)
    image = np.reshape(np.asarray(rgba, dtype=np.uint8), (height, width, 4))[:, :, :3]
    os.makedirs(os.path.dirname(os.path.abspath(out_png)), exist_ok=True)
    Image.fromarray(image).save(out_png)
    return image


def markers_at(T: np.ndarray, points: np.ndarray) -> dict:
    """Design markers placed at a bar pose, shaped like the monitor's marker cache.

    Args:
        T (np.ndarray): 4x4 bar pose.
        points (np.ndarray): (M, 3) design markers in the bar frame, metres.

    Returns:
        dict: ``{id: {'pos', 'size', 'error'}}`` with ids ``1..M``.
    """
    return {k + 1: {'pos': point.tolist(), 'size': [0.019], 'error': 0.0005}
            for k, point in enumerate(transform_points(T, points))}


def n_ghost_bodies(cell, bar: str, joint_names: list) -> int:
    """How many bodies one see-through copy of a bar + its joints has (one per visual mesh).

    Args:
        cell (RobotCell): The cfab cell.
        bar (str): The bar's rigid-body name.
        joint_names (list): The joints on the bar.

    Returns:
        int: The body count.
    """
    return sum(len(cell.rigid_body_models[name].visual_meshes) for name in [bar] + joint_names)


def run(results: Results, problem: str, root: str, out_png: str) -> None:
    """Load B20's J action, measure (one bar, then every streaming bar, live), record, save, render.

    Args:
        results (Results): Where the checks go.
        problem (str): Problem folder name.
        root (str): The scratch problem folder.
        out_png (str): Where the PNG goes.
    """
    skeleton = load_plan_skeleton(root)
    action = next(a for a in skeleton.actions if a.action_id == ACTION_ID)
    robot = robot_by_name(action.robot)
    base = skeleton.load_bar_action(action, prefer_sidecar=False).movements[-1].start_state.robot_base_frame
    monitor, _iface, _log = make_monitor(robot, base, problem)
    add_viz_huskies(monitor, robot)
    # * What __init__ sets for the MoCap bar panel (the harness skips it), and
    # * the desk-rehearsal flag.
    monitor.MOCAP_BAR_FAKE_MARKERS = 1
    monitor.marker_set_data = []
    monitor.calibration_data = []   # save_markerset_data prints it
    monitor.name_from_mocap_id = {}
    monitor._mocap_cache_lock = threading.Lock()
    monitor._mocap_labeled_marker_cache = {}
    monitor._mocap_labeled_marker_stamp = {}
    client = monitor.cfab.client.client_id
    try:
        # * --- 1. load: the marker table and its check against the joint frames
        print(f"\n{'=' * 30} LOAD {ACTION_ID} {'=' * 30}")
        _, out = call_keeping_prints(lambda: (monitor._load_skeleton_state(), monitor.load_action(action.index)))
        results.check("load: '[mocap bars] 2 instrumented bar(s): bar_B20 (8), bar_B21 (8) | ...'",
                      '[mocap bars] 2 instrumented bar(s): bar_B20 (8), bar_B21 (8) | ' in out)
        table = re.search(r"\[mocap bars\] bar_B20: the marker table agrees with this action's joint frames "
                          r"\(max ([0-9.]+) mm over (\d+) markers\)", out)
        results.check(f"load: B20's marker table agrees with the joint frames (< {TABLE_TOL_MM} mm, 8 markers)",
                      table is not None and float(table.group(1)) < TABLE_TOL_MM and int(table.group(2)) == 8,
                      table.group(0) if table else 'line not printed')
        select_movement(monitor, 0)
        results.check(f"load: the loaded bar is {BAR}", monitor.active_bar_name == BAR, str(monitor.active_bar_name))

        # * --- 1b. Motive's asset list (made up, as seen 2026-10-16): B34 / B35 stream without a marker table
        print(f"\n{'=' * 30} MOTIVE ASSETS (B20, B21, B34, B35) {'=' * 30}")
        assets = [SimpleNamespace(sz_name=name.encode('utf-8'), id_num=mocap_id) for name, mocap_id in MOTIVE_ASSETS]
        monitor.mocap_client = SimpleNamespace(
            get_latest_data_descriptions=lambda: SimpleNamespace(rigid_body_list=assets))
        try:
            _, out = call_keeping_prints(monitor._register_mocap_bar_assets)
            _, again = call_keeping_prints(monitor._register_mocap_bar_assets)
        finally:
            monitor.mocap_client = None
        no_table = (f"[mocap bar] Motive also streams B34 (id 1034), B35 (id 1035), but {MOCAP_MARKERS_FILENAME} "
                    "has no marker table for them (no T20_MoCap joints in the Rhino export): they cannot be measured.")
        results.check("assets: B34 / B35 named once as streaming without a marker table",
                      no_table in out and no_table not in again
                      and monitor._mocap_bar_asset_from_id == {1020: BAR, 1021: OTHER_BAR},
                      f"{monitor._mocap_bar_asset_from_id}")

        design_pose, state = monitor._designed_bar_frame(BAR)
        T_design = tform_from_pose(design_pose)
        joint_names = joints_on_bar(state.rigid_body_states, bar_id_of_body(BAR))

        # * --- 2. one bar (measure_bar_pose) on the made-up markers
        print(f"\n{'=' * 30} MEASURE ONE BAR (fake markers) {'=' * 30}")
        fit, out = call_keeping_prints(monitor.measure_bar_pose)
        last = monitor._mocap_bar_last[BAR]
        dev = last['deviation']
        results.check("measure: fit ok, 8/8 matched, the stray marker unmatched, source 'fake'",
                      fit is not None and fit.ok and fit.n_matched == 8 and fit.n_design == 8
                      and len(fit.unmatched_sensed) == 1 and last['source'] == 'fake',
                      f"ok={fit.ok if fit else None} {fit.n_matched if fit else '-'}/8, "
                      f"unmatched {fit.unmatched_sensed if fit else '-'}, source {last['source'] if last else '-'}")
        want = np.array([0.0, 0.0, 1000.0 * SLIDE_M])
        results.check(f"measure: dpos_bar ~ (0, 0, 30) mm (within {DPOS_TOL_MM} mm), "
                      f"dang ~ 2 deg (within {DANG_TOL_DEG})",
                      dev is not None and np.linalg.norm(dev.dpos_bar_mm - want) < DPOS_TOL_MM
                      and abs(dev.dang_deg - ROLL_DEG) < DANG_TOL_DEG,
                      f"dpos_bar {np.round(dev.dpos_bar_mm, 2).tolist()} mm, dang {dev.dang_deg:.3f} deg"
                      if dev is not None else 'no deviation')
        for prefix in ("[mocap bar] bar_B20 | source fake", "[mocap bar] residuals (mm):",
                       "[mocap bar] bar ends (world, mm):"):
            results.check(f"measure: log line '{prefix} ...'", prefix in out)
        draw = monitor._mocap_bar_draws[BAR]
        cell = monitor.cfab.robot_cell
        n_bodies = n_ghost_bodies(cell, BAR, joint_names)
        results.check(f"measure: both copies hold the bar + its {N_JOINTS} joints "
                      f"({n_bodies} bodies each)",
                      len(joint_names) == N_JOINTS and len(draw['rhino']) == n_bodies == len(draw['mocap']),
                      f"joints {joint_names}, pink {len(draw['rhino'])}, cyan {len(draw['mocap'])} bodies")
        err_rhino = body_pose_error(client, draw['rhino'][0][0], T_design)
        results.check("measure: the pink copy's bar sits at the Rhino pose",
                      err_rhino[0] < 1e-3 and err_rhino[1] < 1e-3, f"{err_rhino[0]:.2e} mm, {err_rhino[1]:.2e} deg")
        err_mocap = body_pose_error(client, draw['mocap'][0][0], T_design @ bar_slide_roll(SLIDE_M, ROLL_DEG))
        results.check(f"measure: the cyan copy's bar sits at Rhino slid 30 mm + rolled 2 deg "
                      f"(within {GHOST_TOL_MM} mm, {GHOST_TOL_DEG} deg)",
                      err_mocap[0] < GHOST_TOL_MM and err_mocap[1] < GHOST_TOL_DEG,
                      f"{err_mocap[0]:.2f} mm, {err_mocap[1]:.3f} deg")
        items = draw['items']
        n_sensed = len(fit.sensed_ids)
        results.check("measure: debug items = 2 centre lines + 2 labels, a cross per sensed marker, "
                      "a line per match, no missing label",
                      {'rhino_axis', 'rhino_label', 'mocap_axis', 'mocap_label'} <= set(items)
                      and sum(k.startswith('sensed_') for k in items) == 3 * n_sensed
                      and sum(k.startswith('match_') for k in items) == 8
                      and not any(k.startswith('missing_') for k in items),
                      f"{len(items)} items")
        b20_fake = last['snapshot']

        # * --- 3. the same markers minus three, through the Motive listener
        print(f"\n{'=' * 30} MEASURE ({N_DROPPED} markers fewer, via receive_labeled_marker) {'=' * 30}")
        dropped = [fit.sensed_ids[s] for s in sorted(fit.correspondences)[:N_DROPPED]]
        kept = {k: v for k, v in b20_fake.items() if k not in dropped}
        monitor.receive_labeled_marker(as_natnet_frame(kept))
        cached, age_s = monitor.labeled_markers_snapshot('_unlabeled')
        gap = max(np.linalg.norm(np.subtract(cached[k]['pos'], kept[k]['pos'])) for k in kept)
        results.check("listener: unlabelled markers cached under '_unlabeled', back in z-up, fresh",
                      set(cached) == set(kept) and gap < 1e-12 and age_s is not None and age_s < 0.5,
                      f"{len(cached)} markers, max gap {gap:.1e} m, age {age_s}")
        fit2, out = call_keeping_prints(monitor.measure_bar_pose)
        results.check("measure: 3 markers fewer -> not ok, reason mentions '5 of 8', source 'unlabeled'",
                      fit2 is not None and not fit2.ok and '5 of 8' in fit2.reason
                      and monitor._mocap_bar_last[BAR]['source'] == 'unlabeled',
                      f"reason: {fit2.reason if fit2 else '-'}")
        hidden_z = p.getBasePositionAndOrientation(draw['mocap'][0][0], physicsClientId=client)[0][2]
        results.check("measure: the cyan copy is hidden while the fit fails", hidden_z < -50.0, f"z = {hidden_z}")
        n_missing = sum(k.startswith('missing_') for k in monitor._mocap_bar_draws[BAR]['items'])
        results.check(f"measure: a red label at each of the {N_DROPPED} missing markers",
                      n_missing == N_DROPPED, f"{n_missing} labels")

        # * --- 3b. B20's own asset (lab convention Bxx -> 10xx: B20 = 1020)
        print(f"\n{'=' * 30} MEASURE (B20's asset, model id 1020) {'=' * 30}")
        ids = getattr(monitor, '_mocap_bar_asset_from_id', {})
        results.check("assets: the convention maps 1020 -> bar_B20 and 1021 -> bar_B21",
                      ids.get(1020) == BAR and ids.get(1021) == OTHER_BAR, f"{ids}")
        monitor._mocap_labeled_marker_cache.clear()
        monitor._mocap_labeled_marker_stamp.clear()
        monitor.receive_labeled_marker(as_natnet_frame(b20_fake, model_id=1020))
        cached, _ = monitor.labeled_markers_snapshot(BAR)
        fit3, out = call_keeping_prints(monitor.measure_bar_pose)
        results.check("measure: markers under model 1020 are cached as bar_B20 and fit ok, source 'asset'",
                      len(cached) == len(b20_fake) and fit3 is not None and fit3.ok
                      and fit3.n_matched == 8 and monitor._mocap_bar_last[BAR]['source'] == 'asset',
                      f"{len(cached)} cached, ok={getattr(fit3, 'ok', None)}, "
                      f"source {monitor._mocap_bar_last[BAR]['source']}")

        # * --- 3c. nothing from Motive at all (and no made-up markers): a plain message
        monitor._mocap_labeled_marker_cache.clear()
        monitor._mocap_labeled_marker_stamp.clear()
        monitor.MOCAP_BAR_FAKE_MARKERS = 0
        fit4, out = call_keeping_prints(monitor.measure_bar_pose)
        monitor.MOCAP_BAR_FAKE_MARKERS = 1
        results.check("measure: no markers ever -> None and '[mocap bar] no markers from Motive yet ...'",
                      fit4 is None and '[mocap bar] no markers from Motive yet' in out and '1020' in out)

        # * --- 4. every streaming bar: B20 (made-up markers) under 1020, B21 (design markers
        # * at its Rhino pose, slid 10 mm) under 1021, in one Motive frame
        print(f"\n{'=' * 30} MEASURE STREAMING BARS (B20 under 1020, B21 under 1021) {'=' * 30}")
        other_pose, other_state = monitor._designed_bar_frame(OTHER_BAR)
        T_other = tform_from_pose(other_pose)
        other_joints = joints_on_bar(other_state.rigid_body_states, bar_id_of_body(OTHER_BAR))
        b21_markers = markers_at(T_other @ bar_slide_roll(OTHER_SLIDE_M, 0.0),
                                 monitor._mocap_markers.bars[OTHER_BAR].points())
        two_bars = {**as_natnet_frame(b20_fake, model_id=1020), **as_natnet_frame(b21_markers, model_id=1021)}
        monitor.receive_labeled_marker(two_bars)
        start = time.perf_counter()
        fits, out = call_keeping_prints(monitor.measure_mocap_bars)
        click_ms = 1000.0 * (time.perf_counter() - start)
        header = ("[mocap bar] measuring 2 bar(s): bar_B20 (asset 1020), bar_B21 (asset 1021) | "
                  "instrumented but not streaming: none")
        results.check("streaming bars: header line names both bars and their assets", header in out)
        other_dev = monitor._mocap_bar_last.get(OTHER_BAR, {}).get('deviation')
        results.check(f"streaming bars: both fits ok, not tracked (a click searches in full); "
                      f"B21 dpos_bar ~ (0, 0, {OTHER_SLIDE_M * 1000:.0f}) mm",
                      set(fits) == {BAR, OTHER_BAR} and all(f.ok and not f.tracked for f in fits.values())
                      and other_dev is not None
                      and np.linalg.norm(other_dev.dpos_bar_mm - [0.0, 0.0, 1000.0 * OTHER_SLIDE_M]) < 1e-3,
                      f"fits {sorted(fits)}, B21 dpos_bar "
                      f"{np.round(other_dev.dpos_bar_mm, 3).tolist() if other_dev is not None else '-'} mm, "
                      f"click {click_ms:.0f} ms")
        draws = monitor._mocap_bar_draws
        n_other = n_ghost_bodies(cell, OTHER_BAR, other_joints)
        results.check(f"streaming bars: both drawn (B20 {n_bodies} bodies per copy, B21 {n_other} = bar + "
                      f"{len(other_joints)} joints)",
                      set(draws) == {BAR, OTHER_BAR}
                      and len(draws[BAR]['rhino']) == n_bodies == len(draws[BAR]['mocap'])
                      and len(draws[OTHER_BAR]['rhino']) == n_other == len(draws[OTHER_BAR]['mocap']),
                      f"B21 joints {other_joints}")

        # * --- 4b. 'follow live' (log=False): each fit starts from the last pose
        print(f"\n{'=' * 30} FOLLOW LIVE (2 bars) {'=' * 30}")
        live = []
        for _ in range(2):
            monitor.receive_labeled_marker(two_bars)
            live.append(monitor.measure_mocap_bars(log=False))
        results.check("live: the second call's fits are ok and tracked (started from the last pose)",
                      set(live[1]) == {BAR, OTHER_BAR} and all(f.ok and f.tracked for f in live[1].values()),
                      ", ".join(f"{bar} ok={f.ok} tracked={f.tracked}" for bar, f in live[1].items()))
        seconds = []
        for _ in range(N_LIVE_TIMED):
            monitor.receive_labeled_marker(two_bars)
            start = time.perf_counter()
            monitor.measure_mocap_bars(log=False)
            seconds.append(time.perf_counter() - start)
        live_ms = 1000.0 * float(np.median(seconds))
        print(f"[smoke] follow live, 2 bars (headless DIRECT): {live_ms:.1f} ms per call "
              f"(median of {N_LIVE_TIMED}); the click above (full search + report): {click_ms:.0f} ms")
        results.check("live: one call with 2 bars well within the 100 ms period",
                      live_ms < 50.0, f"{live_ms:.1f} ms per call")

        # * --- 4c. the 'Rhino bar lines (pink)' checkbox, also with 'follow live' off
        print(f"\n{'=' * 30} CHECKBOX 'Rhino bar lines (pink)' {'=' * 30}")
        call_keeping_prints(monitor._show_assembled_bar_ghost)
        lines = monitor._assembled_bar_lines
        results.check("checkbox: the loaded bar's pink assembled line is kept and drawn",
                      set(lines) == {BAR} and set(lines[BAR]['ids']) == {'axis', 'text'}, f"{sorted(lines)}")
        monitor.SHOW_RHINO_BAR_LINES = 0
        monitor._service_rhino_bar_lines()
        monitor.receive_labeled_marker(two_bars)
        monitor.measure_mocap_bars(log=False)
        monitor._redraw_assembled_bar_lines(force=True)
        rhino_items = {bar: sorted(k for k in d['items'] if k.startswith('rhino_')) for bar, d in draws.items()}
        results.check("checkbox off: no rhino_axis / rhino_label in any drawing (also after a live fit), "
                      "no assembled-line ids, the pink copies stay",
                      not any(rhino_items.values()) and not any(line['ids'] for line in lines.values())
                      and all(d['rhino'] for d in draws.values()),
                      f"rhino items {rhino_items}")
        monitor.SHOW_RHINO_BAR_LINES = 1
        monitor._service_rhino_bar_lines()
        rhino_items = {bar: sorted(k for k in d['items'] if k.startswith('rhino_')) for bar, d in draws.items()}
        results.check("checkbox on: rhino_axis + rhino_label back in every drawing, the assembled line back",
                      all(items == ['rhino_axis', 'rhino_label'] for items in rhino_items.values())
                      and all(set(line['ids']) == {'axis', 'text'} for line in lines.values()),
                      f"rhino items {rhino_items}")

        # * --- 4d. B21's asset goes stale: its drawing goes, B20's stays
        print(f"\n{'=' * 30} B21's ASSET STOPS STREAMING {'=' * 30}")
        n_before = p.getNumBodies(physicsClientId=client)
        monitor.receive_labeled_marker(as_natnet_frame(b20_fake, model_id=1020))
        monitor._mocap_labeled_marker_stamp[OTHER_BAR] -= 10.0
        stale, out = call_keeping_prints(lambda: monitor.measure_mocap_bars(log=False))
        _, again = call_keeping_prints(lambda: monitor.measure_mocap_bars(log=False))
        n_removed = n_before - p.getNumBodies(physicsClientId=client)
        gone = f"[mocap bar] {OTHER_BAR}: its Motive asset stopped streaming; its drawing is removed."
        results.check("stale: B21's drawing removed (both copies), B20 still measured and drawn, said once",
                      set(stale) == {BAR} and set(draws) == {BAR} and n_removed == 2 * n_other
                      and gone in out and gone not in again,
                      f"measured {sorted(stale)}, drawn {sorted(draws)}, {n_removed} bodies removed")

        # * --- 5. record a take per streaming bar and save them
        print(f"\n{'=' * 30} RECORD TAKE + SAVE {'=' * 30}")
        monitor.receive_labeled_marker(two_bars)
        _, out = call_keeping_prints(monitor.record_mocap_bar_take)
        takes = monitor.marker_set_data
        try:
            json.dumps(takes)
            serialisable = True
        except TypeError as e:
            serialisable = False
            print(f"takes not JSON-serialisable: {e}")
        designs = {BAR: T_design, OTHER_BAR: T_other}
        results.check("record take: one take per streaming bar, each with its snapshot, fit, deviation, source, "
                      "age, tool_ft, its 8 design markers and its Rhino pose",
                      [take.get('markerset_key') for take in takes] == [BAR, OTHER_BAR] and serialisable
                      and all(set(take) == TAKE_KEYS | {take['markerset_key']} and take['bar_fit']['ok']
                              and take['marker_source'] == 'asset' and take['deviation'] is not None
                              and len(take['design_markers']) == 8
                              and set(take['design_markers'][0]) == {'name', 'joint', 'label', 'bar_m'}
                              and np.allclose(take['design_T_world_from_bar'], designs[take['markerset_key']])
                              for take in takes),
                      f"{len(takes)} takes, keys {[sorted(take) for take in takes]}")
        for k, bar in enumerate((BAR, OTHER_BAR), start=1):
            line = f"[mocap bar] take {k} recorded: {bar} (unsaved; 'Save markerset data' writes it)"
            results.check(f"record take: log line '{line}'", line in out)
        husky_world.save_markerset_data(monitor, use_experiment_dir=True)
        saved = sorted(glob.glob(os.path.join(husky_world.BAR_HOLDING_ACC_EXPERIMENT_DIR, '*', '*.json')))
        payload = json.load(open(saved[-1])) if saved else {}
        results.check("save: the file names mocap_markers_file, holds both takes, and no file-level "
                      "mocap_markers_design / design_T_world_from_bar",
                      str(payload.get('mocap_markers_file', '')).endswith(MOCAP_MARKERS_FILENAME)
                      and 'mocap_markers_design' not in payload and 'design_T_world_from_bar' not in payload
                      and len(payload.get('raw_data', [])) == 2,
                      saved[-1] if saved else 'no file written')

        # * --- 6. the picture
        print(f"\n{'=' * 30} RENDER {'=' * 30}")
        image = render_overlay(monitor, T_design, monitor._mocap_bar_draws[BAR]['z_range'], joint_names, out_png)
        r, g, b = (image[:, :, k].astype(int) for k in range(3))
        n_cyan = int(np.sum((g > 120) & (b > 120) & (r < 90)))
        n_pink = int(np.sum((r > 150) & (b > 90) & (g < 110)))
        results.check("render: the PNG shows cyan and pink pixels", n_cyan > 200 and n_pink > 200,
                      f"{out_png}: {n_cyan} cyan, {n_pink} pink pixels")
    finally:
        monitor.cfab.close()
        # The bar copies' OBJ files (the live monitor keeps them until the machine restarts).
        if getattr(monitor, '_mocap_bar_mesh_dir', None):
            shutil.rmtree(monitor._mocap_bar_mesh_dir, ignore_errors=True)


def main(argv: list = None) -> int:
    """Build the scratch problem, run the checks, print the summary.

    Args:
        argv (list | None): Command line (None = ``sys.argv[1:]``).

    Returns:
        int: 0 when every check passed, else 1.
    """
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--out', default=DEFAULT_OUT, help="where the PNG of the bar copies goes")
    parser.add_argument('--keep', action='store_true', help="keep the temp dir (the saved take) for inspection")
    args = parser.parse_args(argv)

    problem = DESIGN_PROBLEM_NAME
    real_root = problem_root(problem, DESIGN_DATA_DIRECTORY)
    if not os.path.isfile(os.path.join(real_root, MOCAP_MARKERS_FILENAME)):
        print(f"{problem} has no {MOCAP_MARKERS_FILENAME}; this smoke needs 261016_2ModulesDemo_YH.")
        return 1
    before = folder_snapshot(real_root)
    scratch_design = tempfile.mkdtemp(prefix='husky_mocap_bar_smoke_')
    print(f"[smoke] real problem folder (read only): {real_root}")
    print(f"[smoke] scratch copy: {os.path.join(scratch_design, problem)}")
    results = Results()
    try:
        root = make_scratch_problem(real_root, scratch_design, problem)
        shutil.copy(os.path.join(real_root, MOCAP_MARKERS_FILENAME), os.path.join(root, MOCAP_MARKERS_FILENAME))
        point_package_at(scratch_design)
        # 'Save markerset data' writes under the scratch folder, not the experiment data.
        husky_world.BAR_HOLDING_ACC_EXPERIMENT_DIR = os.path.join(scratch_design, 'bar_holding_acc_data')
        try:
            run(results, problem, root, args.out)
        except Exception as e:  # keep going: the summary shows where it stopped
            traceback.print_exc()
            results.check('the smoke ran to the end', False, f"{type(e).__name__}: {e}")
        results.check('real design folder untouched', folder_snapshot(real_root) == before)
    finally:
        if args.keep:
            print(f"[smoke] kept the scratch copy: {scratch_design}")
        else:
            shutil.rmtree(scratch_design, ignore_errors=True)
            print("[smoke] removed the scratch copy (use --keep to inspect it)")

    results.print_summary()
    return 1 if results.n_failed() else 0


if __name__ == '__main__':
    sys.exit(main())
