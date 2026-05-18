"""Compare observed bar pose (from mocap) to the BarAction cell-state goal.

For each marker take in a batch folder:
1. Read the top-level `bar_action_path` + `movement_id` stamped by the monitor.
2. Load the BarAction; find the matching movement.
3. Compute the goal bar world pose from the movement's `target_ee_frames`
   composed with the bar's `attachment_frame`.
4. Run fit_bar_from_markerset on the take; compare via bar_deviation_from_goal.
5. Print per-take deviations and aggregate stats.
"""

import json
import os
import sys
import argparse

import numpy as np
import matplotlib.pyplot as plt
import pybullet_planning as pp

from husky_assembly_teleop import DESIGN_DATA_DIRECTORY, DESIGN_PROBLEM_NAME, EXPERIMENT_DATA_DIRECTORY
from husky_assembly_teleop.utils import pose_from_frame
from husky_assembly_teleop.bar_action_io import parse_bar_action, find_movement
from husky_assembly_teleop.mocap_experiment import (
    fit_bar_from_markerset,
    bar_deviation_from_goal,
)


def goal_bar_pose_from_movement(mv, bar_name):
    """world_from_bar at the movement's start_state.

    Two cases:
    - Bar attached to a tool/link (e.g. holding mid-air): compose
      ``target_ee_frames[side] ∘ attachment_frame``.
    - Bar already installed (retreat movement): use the bar's world ``frame``
      directly; side is reported as ``'installed'``.

    Returns ``(None, None)`` if neither is available.
    """
    bar_rb = mv.start_state.rigid_body_states[bar_name]
    if bar_rb is None:
        return None, None
    if bar_rb.attachment_frame is not None:
        attached_link = getattr(bar_rb, 'attached_to_link', '') or ''
        side = 'left' if 'left' in attached_link else 'right'
        target = (mv.target_ee_frames or {}).get(side)
        if target is None:
            return None, None
        world_from_tool = pose_from_frame(target)
        tool_from_bar = pose_from_frame(bar_rb.attachment_frame)
        return pp.multiply(world_from_tool, tool_from_bar), side
    if bar_rb.frame is not None:
        return pose_from_frame(bar_rb.frame), 'installed'
    return None, None


def _make_corrector(saved_convention):
    if saved_convention == 'rhino':
        return lambda p: list(p)
    if saved_convention == 'rotated':
        return lambda p: [p[1], -p[0], p[2]]
    raise ValueError(f"unknown mocap_axis_convention {saved_convention!r}")


def _convert_markerset(labeled_marker_dict, correct):
    out = {}
    for mid, info in labeled_marker_dict.items():
        new_info = dict(info)
        new_info['pos'] = correct(info['pos'])
        out[mid] = new_info
    return out


def _equal_axes_3d(ax, points, pad=0.05):
    arr = np.asarray(points)
    mins = arr.min(axis=0)
    maxs = arr.max(axis=0)
    centers = (mins + maxs) / 2
    span = max((maxs - mins).max(), 0.2) * (1 + pad)
    half = span / 2
    ax.set_xlim(centers[0] - half, centers[0] + half)
    ax.set_ylim(centers[1] - half, centers[1] + half)
    ax.set_zlim(centers[2] - half, centers[2] + half)


def _plot_compare(ax, fit, goal_bar_pose, take_label, dev, marker_pts=None):
    """Goal bar (green dashed) + fitted bar (red) with start/end deltas.

    TEMPORARY (rhino-export OCF-origin bug): goal_pos is the bar's LOWER
    tip, NOT the midpoint. So the goal bar is drawn from goal_pos extending
    in +goal_z for the full observed bar length. The fitted bar's lower
    tip is paired with goal_pos for the start-vs-start delta.

    `marker_pts` is the (already corrected) ``{marker_id: {'pos': [x,y,z], ...}}``
    dict for the take, plotted as small grey dots so we can sanity-check the
    fit against the raw mocap.
    """
    goal_pos = np.asarray(goal_bar_pose[0], dtype=float)
    goal_R = np.asarray(pp.matrix_from_quat(goal_bar_pose[1]), dtype=float)
    goal_z = goal_R[:, 2] / np.linalg.norm(goal_R[:, 2])

    fit_dir = np.asarray(fit['fitted_line']['direction'], dtype=float)
    fit_dir = fit_dir / np.linalg.norm(fit_dir)
    # Align fit_dir with goal_z so end-points pair up consistently.
    if np.dot(fit_dir, goal_z) < 0:
        fit_dir = -fit_dir

    ocf = np.asarray(fit['ocf_position'], dtype=float)
    tips = np.asarray(fit['bar_end_points'], dtype=float)
    bar_len = float(np.linalg.norm(tips[0] - tips[1]))
    # Order fitted tips: a = -dir (lower / start), b = +dir (upper / end).
    if np.dot(tips[0] - ocf, fit_dir) > np.dot(tips[1] - ocf, fit_dir):
        tips = tips[::-1]
    fit_a, fit_b = tips[0], tips[1]

    # Bug-compat: goal_pos = goal_a (bar's lower tip / start). Extend the
    # full bar length in +goal_z to reach the other end.
    goal_a = goal_pos
    goal_b = goal_pos + bar_len * goal_z

    angle_deg = float(np.degrees(dev['angle_rad']))
    lateral_mm = dev['lateral_dev_m'] * 1000
    start_dev_mm = float(np.linalg.norm(fit_a - goal_a)) * 1000
    end_dev_mm = float(np.linalg.norm(fit_b - goal_b)) * 1000

    raw_pts = None
    if marker_pts:
        raw_pts = np.asarray([info['pos'] for info in marker_pts.values()], dtype=float)
        ax.scatter(raw_pts[:, 0], raw_pts[:, 1], raw_pts[:, 2],
                   c='0.4', s=15, depthshade=False, label='raw mocap markers')

    # fitted bar (red, solid)
    ax.plot([fit_a[0], fit_b[0]], [fit_a[1], fit_b[1]], [fit_a[2], fit_b[2]],
            c='r', lw=2, label='fitted bar')
    ax.scatter([fit_a[0]], [fit_a[1]], [fit_a[2]],
               c='r', s=90, marker='*', depthshade=False, label='fitted start (lower tip)')
    ax.scatter([fit_b[0]], [fit_b[1]], [fit_b[2]],
               c='r', s=40, marker='^', depthshade=False)
    ax.scatter([ocf[0]], [ocf[1]], [ocf[2]],
               c='r', s=30, marker='o', depthshade=False, label='fitted OCF (midpoint, FYI)')

    # goal bar (green, dashed)
    ax.plot([goal_a[0], goal_b[0]], [goal_a[1], goal_b[1]], [goal_a[2], goal_b[2]],
            c='g', lw=2, ls='--', label='goal bar')
    ax.scatter([goal_a[0]], [goal_a[1]], [goal_a[2]],
               c='g', s=90, marker='*', depthshade=False, label='goal start (= goal_pos)')
    ax.scatter([goal_b[0]], [goal_b[1]], [goal_b[2]],
               c='g', s=40, marker='^', depthshade=False)

    # delta connectors: start-vs-start (primary) + end-vs-end.
    ax.plot([fit_a[0], goal_a[0]], [fit_a[1], goal_a[1]], [fit_a[2], goal_a[2]],
            c='m', lw=1, ls=':')
    ax.plot([fit_b[0], goal_b[0]], [fit_b[1], goal_b[1]], [fit_b[2], goal_b[2]],
            c='b', lw=1, ls=':')

    mid_a = (fit_a + goal_a) / 2
    mid_b = (fit_b + goal_b) / 2
    ax.text(mid_a[0], mid_a[1], mid_a[2], f"Δstart={start_dev_mm:.2f} mm",
            color='m', fontsize=8)
    ax.text(mid_b[0], mid_b[1], mid_b[2], f"Δend={end_dev_mm:.2f} mm",
            color='b', fontsize=8)

    ax.set_xlabel('X (rhino)')
    ax.set_ylabel('Y (rhino)')
    ax.set_zlabel('Z (rhino, up)')
    ax.set_title(
        f"{take_label}\n"
        f"angle(fit, goal) = {angle_deg:.3f}°  |  "
        f"Δstart = {start_dev_mm:.2f} mm  |  lateral = {lateral_mm:.2f} mm"
    )
    ax.legend(loc='upper left', fontsize=7)
    pts_for_bounds = [fit_a, fit_b, ocf, goal_a, goal_b]
    if raw_pts is not None and len(raw_pts):
        pts_for_bounds.extend(raw_pts)
    _equal_axes_3d(ax, np.vstack(pts_for_bounds))


def process_file(file_path, override_bar_action=None, override_movement=None):
    with open(file_path, 'r') as f:
        data = json.load(f)

    saved_convention = data.get('mocap_axis_convention', 'rotated')
    correct = _make_corrector(saved_convention)

    bar_action_path = override_bar_action or data.get('bar_action_path')
    movement_id = override_movement or data.get('movement_id')
    movement_index = data.get('movement_index')

    if not bar_action_path:
        print(f"  SKIP {os.path.basename(file_path)}: no bar_action_path in JSON")
        return []
    if not os.path.isabs(bar_action_path):
        print(f"  SKIP {os.path.basename(file_path)}: bar_action_path not absolute: {bar_action_path}")
        return []
    if not os.path.exists(bar_action_path):
        print(f"  SKIP {os.path.basename(file_path)}: bar_action_path not found: {bar_action_path}")
        return []

    action = parse_bar_action(bar_action_path)
    key = movement_id if movement_id else (movement_index if movement_index is not None else 2)
    idx, mv = find_movement(action, key)
    bar_name = f"bar_{action.active_bar_id}"

    goal_bar_pose, side = goal_bar_pose_from_movement(mv, bar_name)
    if goal_bar_pose is None:
        print(f"  SKIP {os.path.basename(file_path)}: bar {bar_name!r} not attached in mv[{idx}]={mv.movement_id} start_state")
        return [], []

    print(f"\n=== {os.path.basename(file_path)} (mocap_axis_convention={saved_convention}) ===")
    print(f"  action={os.path.basename(bar_action_path)} | mv[{idx}]={mv.movement_id} | bar={bar_name} | held_by={side}")
    print(f"  goal_bar_pos={[round(v,4) for v in goal_bar_pose[0]]} | goal_bar_quat={[round(v,4) for v in goal_bar_pose[1]]}")
    # TEMPORARY: rhino RobotCell export writes the bar's LOWER end (smallest
    # world z) as the OCF origin instead of the midpoint. Comparing OCF to
    # goal_pos would therefore mis-report pos_dev by ~half the bar length.
    # Until the rhino export is fixed, compare the fitted bar's lower tip
    # to the (bug-derived) goal_pos. `start_dev_m` below is that residual.
    print(
        "  WARN: goal_bar_pos interpreted as the bar's LOWER tip (rhino export "
        "OCF-origin bug). Comparing against fitted bar's lower tip; pos_dev is "
        "kept as OCF-vs-goal for reference but is NOT the true center error."
    )

    rows = []
    fits_for_plot = []
    for i, entry in enumerate(data['raw_data']):
        marker_pts_saved = entry.get('bar_rig', {})
        marker_pts = _convert_markerset(marker_pts_saved, correct)
        try:
            fit = fit_bar_from_markerset(marker_pts)
        except Exception as e:
            print(f"  take {i}: fit failed ({e})")
            continue
        dev = bar_deviation_from_goal(fit, goal_bar_pose)
        ocf = fit['ocf_position']
        # TEMPORARY: bug-compat — pick the fitted tip with the lower world z
        # as the "observed start" and compare it to goal_pos.
        tips = fit['bar_end_points']
        fitted_start = tips[0] if tips[0][2] <= tips[1][2] else tips[1]
        start_dev_m = float(np.linalg.norm(
            np.asarray(fitted_start, dtype=float) - np.asarray(goal_bar_pose[0], dtype=float)
        ))
        print(
            f"  take {i}: "
            f"start_dev={start_dev_m*1000:.2f} mm | "
            f"angle_dev={np.rad2deg(dev['angle_rad']):.3f} deg | "
            f"lateral_dev={dev['lateral_dev_m']*1000:.2f} mm | "
            f"center_to_line_dist_max={fit['center_to_line_dist_max_m']*1000:.2f} mm | "
            f"fitted_start=({fitted_start[0]:.4f}, {fitted_start[1]:.4f}, {fitted_start[2]:.4f}) | "
            f"ocf=({ocf[0]:.4f}, {ocf[1]:.4f}, {ocf[2]:.4f}) | "
            f"pos_dev(ocf↔goal)={dev['pos_dev_m']*1000:.2f} mm"
        )
        rows.append({
            'source_file': os.path.basename(file_path),
            'take_index': i,
            'bar_action_path': bar_action_path,
            'movement_id': mv.movement_id,
            'goal_bar_position': list(goal_bar_pose[0]),
            'goal_bar_quaternion': list(goal_bar_pose[1]),
            'ocf_position': fit['ocf_position'],
            'fitted_start_lower_tip': list(fitted_start),
            'fitted_line': fit['fitted_line'],
            'bar_end_points': fit['bar_end_points'],
            'bar_length_observed': fit['bar_length_observed'],
            'center_to_line_dist_max_m': fit['center_to_line_dist_max_m'],
            'center_to_line_dist_rms_m': fit['center_to_line_dist_rms_m'],
            'pos_dev_m': dev['pos_dev_m'],
            'start_dev_m': start_dev_m,
            'angle_dev_rad': dev['angle_rad'],
            'lateral_dev_m': dev['lateral_dev_m'],
        })
        fits_for_plot.append((f"{os.path.basename(file_path)}#{i}", fit, goal_bar_pose, dev, marker_pts))

    return rows, fits_for_plot


def _problem_name_from_bar_action_path(bar_action_path):
    """Derive cfab problem_name from a bar-action path; fall back to default.

    Expected layout: ``<DESIGN_DATA_DIRECTORY>/<problem>/BarActions/xxx.json``.
    """
    try:
        rel = os.path.relpath(bar_action_path, DESIGN_DATA_DIRECTORY)
        parts = rel.split(os.sep)
        if parts and parts[0] not in ('', '..'):
            return parts[0]
    except ValueError:
        pass
    return DESIGN_PROBLEM_NAME


def open_pp_viewer_for_goal(bar_action_path, movement_key, takes=None):
    """Open a cfab pybullet GUI, set the movement's start_state, draw the
    goal bar OCF, and (optionally) overlay each take's marker points + fitted
    bar line.

    takes: optional list of dicts with keys:
        - 'label'   : str (e.g. "<file>#<i>")
        - 'fit'     : output of fit_bar_from_markerset (has bar_end_points)
        - 'markers' : already-corrected ``{id: {'pos': [x,y,z], ...}}``
    """
    from husky_assembly_teleop.cfab_session import CfabSession

    problem_name = _problem_name_from_bar_action_path(bar_action_path)
    action = parse_bar_action(bar_action_path)
    idx, mv = find_movement(action, movement_key)
    bar_name = f"bar_{action.active_bar_id}"
    goal_bar_pose, side = goal_bar_pose_from_movement(mv, bar_name)

    print(f"\n[pp-viewer] problem={problem_name!r} | mv[{idx}]={mv.movement_id} | bar={bar_name} | held_by={side}")
    if goal_bar_pose is None:
        print("[pp-viewer] no goal_bar_pose to draw; aborting")
        return
    print(f"[pp-viewer] goal_bar_pos={[round(v,4) for v in goal_bar_pose[0]]}")
    if takes:
        print(f"[pp-viewer] overlaying {len(takes)} take(s): markers (red dots) + fitted line (blue)")

    session = CfabSession(problem_name, connection_type="gui", enable_debug_gui=True)
    try:
        session.planner.set_robot_cell_state(mv.start_state)
        pp.draw_pose(goal_bar_pose, length=0.3)
        for take in (takes or []):
            fit = take['fit']
            markers = take.get('markers') or {}
            for info in markers.values():
                pp.draw_point(info['pos'], size=0.01, color=[1, 0, 0])
            tips = fit.get('bar_end_points')
            if tips and len(tips) == 2:
                pp.add_line(tips[0], tips[1], color=[0, 0, 1], width=3)
        pp.wait_if_gui(f"goal bar pose for {bar_name} @ {mv.movement_id} — close to continue")
    finally:
        session.close()


def aggregate(rows):
    if not rows:
        print("\nno valid takes")
        return
    pos = np.array([r['pos_dev_m'] for r in rows]) * 1000
    start = np.array([r['start_dev_m'] for r in rows]) * 1000
    ang = np.rad2deg([r['angle_dev_rad'] for r in rows])
    lat = np.array([r['lateral_dev_m'] for r in rows]) * 1000
    st = np.array([r['center_to_line_dist_max_m'] for r in rows]) * 1000
    print(f"\n=== aggregate over {len(rows)} takes ===")
    print("  (start_dev = fitted lower tip vs goal_pos; TEMP bug-compat metric)")
    print(f"  start_dev_mm:      mean={start.mean():.2f}  std={start.std():.2f}  max={start.max():.2f}")
    print(f"  angle_dev_deg:     mean={ang.mean():.3f}  std={ang.std():.3f}  max={ang.max():.3f}")
    print(f"  lateral_dev_mm:    mean={lat.mean():.2f}  std={lat.std():.2f}  max={lat.max():.2f}")
    print(f"  center_to_line_dist_max_mm: mean={st.mean():.2f}  std={st.std():.2f}  max={st.max():.2f}")
    print(f"  pos_dev_mm(ocf↔goal): mean={pos.mean():.2f}  std={pos.std():.2f}  max={pos.max():.2f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('batch', help='batch folder name under EXPERIMENT_DATA_DIRECTORY/bar_holding_acc_data/')
    parser.add_argument('--bar-action', default=None, help='override stamped bar_action_path (absolute)')
    parser.add_argument('--movement', default=None, help='override movement_id (e.g. M2)')
    parser.add_argument('--export', action='store_true', help='dump compared_<batch>.json')
    parser.add_argument('--viewer', action='store_true', help='show matplotlib 3D compare plots')
    parser.add_argument('--pp-viewer', action='store_true',
                        help='open a cfab pybullet GUI, load the BarAction + start_state, '
                             'and draw_pose the goal bar OCF for visual sanity check')
    args = parser.parse_args()

    data_folder = os.path.join(EXPERIMENT_DATA_DIRECTORY, 'bar_holding_acc_data', args.batch)
    if not os.path.isdir(data_folder):
        sys.exit(f"data folder not found: {data_folder}")

    files = sorted(
        os.path.join(data_folder, f) for f in os.listdir(data_folder)
        if f.startswith('bar_holding_acc_') and f.endswith('.json')
    )
    if not files:
        sys.exit(f"no bar_holding_acc_*.json files in {data_folder}")

    all_rows = []
    all_fits = []
    for fp in files:
        rows, fits = process_file(fp, args.bar_action, args.movement)
        all_rows.extend(rows)
        all_fits.extend(fits)
    aggregate(all_rows)

    if args.pp_viewer:
        # Derive bar_action_path + movement key from the first file (or CLI overrides).
        with open(files[0], 'r') as f:
            first = json.load(f)
        ba_path = args.bar_action or first.get('bar_action_path')
        mv_key = args.movement or first.get('movement_id') or first.get('movement_index') or 2
        if not ba_path or not os.path.exists(ba_path):
            print(f"[pp-viewer] no usable bar_action_path (got {ba_path!r}); skipping")
        else:
            takes = [
                {'label': label, 'fit': fit, 'markers': marker_pts}
                for (label, fit, _goal, _dev, marker_pts) in all_fits
            ]
            open_pp_viewer_for_goal(ba_path, mv_key, takes=takes)

    if args.export:
        out_path = os.path.join(data_folder, 'compared_to_cell_state.json')
        with open(out_path, 'w') as f:
            json.dump(all_rows, f, indent=4)
        print(f"\nexported to {out_path}")

    if args.viewer and all_fits:
        n = len(all_fits)
        ncols = min(n, 3)
        nrows = (n + ncols - 1) // ncols
        fig = plt.figure(figsize=(6 * ncols, 5 * nrows))
        for idx, (label, fit, goal_pose, dev, marker_pts) in enumerate(all_fits, start=1):
            ax = fig.add_subplot(nrows, ncols, idx, projection='3d')
            _plot_compare(ax, fit, goal_pose, label, dev, marker_pts=marker_pts)
        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    main()
