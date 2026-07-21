"""OCF + axis estimation from saved bar-holding marker takes.

Pipeline per take:
  1. If saved file used the legacy 'rotated' mocap axis convention,
     correct to the rhino frame. Files written with the 'rhino' convention
     need no correction (the monitor already saves in rhino frame).
  2. Auto-pair markers by cross-bar distance and fit the bar axis through
     the pair midpoints (see fit_bar_from_markerset).
  3. Print OCF, axis, bar_len, angle-to-world-Z, and center_to_line_dist_max.
  4. Optionally show a 3D plot with world Z axis + fitted bar axis +
     angle annotation.

Frame note: the top-level `mocap_axis_convention` field (added by
HuskyMonitor.MOCAP_AXIS_CONVENTION) tells us which convention the saved
data uses. Files without the field are assumed to be legacy 'rotated'.

  rotated -> rhino correction: (x, y, z) -> (y, -x, z)
  rhino   -> rhino correction: identity

center_to_line_dist_max: max perpendicular distance from any pair midpoint
  to the fitted axis. Small → clean fit; large → bad pair match or noise.
"""

import json
import os
import sys
import argparse

import numpy as np
import matplotlib.pyplot as plt

from husky_assembly_teleop import EXPERIMENT_DATA_DIRECTORY, DESIGN_DATA_DIRECTORY, DEFAULT_ENV_3DM
from husky_assembly_teleop.mocap_experiment import (
    fit_bar_from_markerset,
    build_layout,
    draw_layout_2d,
    problem_dir_from_bar_action_path,
    enable_scroll_zoom,
    show_scrollable,
)


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


def _angle_to_world_z_rad(direction):
    d = np.asarray(direction, dtype=float)
    d = d / np.linalg.norm(d)
    return float(np.arccos(np.clip(abs(d[2]), 0.0, 1.0)))


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


def _plot_take(ax, fit, take_label, bar_name=None):
    pair_centers = np.asarray(fit['pair_centers'])
    tips = np.asarray(fit['bar_end_points'])
    ocf = np.asarray(fit['ocf_position'])
    direction = np.asarray(fit['fitted_line']['direction'])
    angle_to_z_deg = np.degrees(_angle_to_world_z_rad(direction))

    bar_len = float(np.linalg.norm(tips[0] - tips[1]))
    arrow_len = 0.6 * bar_len if bar_len > 0 else 0.3

    # pair midpoints
    ax.scatter(pair_centers[:, 0], pair_centers[:, 1], pair_centers[:, 2],
               c='b', s=40, depthshade=False, label='pair midpoints')
    # bar tips
    ax.scatter(tips[:, 0], tips[:, 1], tips[:, 2],
               c='k', s=60, marker='^', depthshade=False, label='bar tips')
    # fitted bar axis through OCF (red)
    fit_dir_unit = direction / np.linalg.norm(direction)
    fit_a = ocf - 0.5 * arrow_len * fit_dir_unit
    fit_b = ocf + 0.5 * arrow_len * fit_dir_unit
    ax.plot([fit_a[0], fit_b[0]], [fit_a[1], fit_b[1]], [fit_a[2], fit_b[2]],
            c='r', lw=2, label='fitted bar axis')
    # world Z axis through OCF (green, dashed)
    z_a = ocf - 0.5 * arrow_len * np.array([0, 0, 1])
    z_b = ocf + 0.5 * arrow_len * np.array([0, 0, 1])
    ax.plot([z_a[0], z_b[0]], [z_a[1], z_b[1]], [z_a[2], z_b[2]],
            c='g', lw=2, ls='--', label='world Z')
    # OCF marker
    ax.scatter([ocf[0]], [ocf[1]], [ocf[2]],
               c='m', s=80, marker='*', depthshade=False, label='OCF')

    ax.set_xlabel('X (rhino)')
    ax.set_ylabel('Y (rhino)')
    ax.set_zlabel('Z (rhino, up)')
    short = take_label.replace('bar_holding_acc_', '')
    bid = (bar_name or '?').replace('bar_', '')
    ax.set_title(
        f"{short}\n"
        f"bar = {bid} | bar_len = {bar_len:.4f} m\n"
        f"ang(fit,Z) = {angle_to_z_deg:.2f}° | "
        f"ctr→line max = {fit['center_to_line_dist_max_m']*1000:.2f} mm",
        fontsize=8,
    )
    ax.legend(loc='upper left', fontsize=8)
    _equal_axes_3d(ax,
                   np.vstack([pair_centers, tips, ocf,
                              z_a, z_b, fit_a, fit_b]))


def process_batch(data_folder, export=True, viewer=False, problem_override=None, env_3dm=None):
    if not os.path.isdir(data_folder):
        sys.exit(f"data folder not found: {data_folder}")

    json_files = sorted(
        f for f in os.listdir(data_folder)
        if f.startswith('bar_holding_acc_') and f.endswith('.json')
    )
    if not json_files:
        sys.exit(f"no bar_holding_acc_*.json files in {data_folder}")

    compiled = []
    fits_for_plot = []

    for file_name in json_files:
        file_path = os.path.join(data_folder, file_name)
        with open(file_path, 'r') as f:
            data = json.load(f)

        saved_convention = data.get('mocap_axis_convention', 'rotated')
        correct = _make_corrector(saved_convention)

        # Metadata the monitor stamps per save: which movement the bar was
        # held at, and the bar's authored dimensions. `nominal_bar_len` (the
        # longest AABB extent) is the design bar length we compare the fitted
        # `bar_length_observed` against below.
        movement_id = data.get('movement_id')
        bar_action_path = data.get('bar_action_path')
        bar_dims = data.get('bar_dimensions')
        bar_start_position = data.get('bar_start_position')
        bar_start_quaternion = data.get('bar_start_quaternion')
        nominal_bar_len = max(bar_dims) if bar_dims else None
        # Provenance for the layout diagram: which bar is tested.
        bar_name = data.get('bar_name')

        print(f"\n=== {file_name} (mocap_axis_convention={saved_convention}) ===")
        print(f"  movement_id={movement_id} | "
              f"bar_action={os.path.basename(bar_action_path) if bar_action_path else None} | "
              f"bar_dims={[round(v, 4) for v in bar_dims] if bar_dims else None} | "
              f"nominal_bar_len={round(nominal_bar_len, 4) if nominal_bar_len else None} m")
        if not data.get('raw_data'):
            print("  WARN: 0 takes recorded in this file — click "
                  "'Record + Fit + Viz (shared)' (once per pose) BEFORE "
                  "'Save markerset data'. Nothing to fit/plot for this file.")
        ref_ocf = None  # take-0 OCF in this file; baseline for diff print
        for i, entry in enumerate(data['raw_data']):
            marker_pts_saved = entry.get('bar_rig', {})
            marker_pts_rhino = _convert_markerset(marker_pts_saved, correct)
            try:
                fit = fit_bar_from_markerset(marker_pts_rhino)
            except Exception as e:
                print(f"  take {i}: fit failed ({e})")
                continue

            ocf = fit['ocf_position']
            axis = fit['fitted_line']['direction']
            angle_to_z_deg = np.degrees(_angle_to_world_z_rad(axis))
            ocf_arr = np.asarray(ocf, dtype=float)
            if ref_ocf is None:
                ref_ocf = ocf_arr
            d = (ocf_arr - ref_ocf) * 1000.0  # signed mm vs take 0
            # Observed-vs-design bar length (mm), only when dims were stamped.
            len_err_str = ""
            if nominal_bar_len is not None:
                len_err_mm = (fit['bar_length_observed'] - nominal_bar_len) * 1000.0
                len_err_str = f"bar_len_err_vs_nominal={len_err_mm:+.2f} mm | "
            print(
                f"  take {i}: "
                f"ocf=({ocf[0]:.4f}, {ocf[1]:.4f}, {ocf[2]:.4f}) m | "
                f"d_ocf_from_take0=({d[0]:+.2f}, {d[1]:+.2f}, {d[2]:+.2f}) mm | "
                f"axis=({axis[0]:+.4f}, {axis[1]:+.4f}, {axis[2]:+.4f}) | "
                f"angle_to_Z={angle_to_z_deg:.3f}° | "
                f"bar_len={fit['bar_length_observed']:.4f} m | "
                f"{len_err_str}"
                f"center_to_line_dist_max={fit['center_to_line_dist_max_m']*1000:.2f} mm | "
                f"center_to_line_dist_rms={fit['center_to_line_dist_rms_m']*1000:.2f} mm"
            )

            bar_len_err = (
                fit['bar_length_observed'] - nominal_bar_len
                if nominal_bar_len is not None else None
            )
            compiled.append({
                'source_file': file_name,
                'take_index': i,
                'movement_id': movement_id,
                'bar_action_path': bar_action_path,
                'bar_dimensions': bar_dims,
                'nominal_bar_length_m': nominal_bar_len,
                'bar_length_error_m': bar_len_err,
                'bar_start_position': bar_start_position,
                'bar_start_quaternion': bar_start_quaternion,
                'joint_conf': entry.get('joint_conf'),
                'footprint_base_link_pose': entry.get('footprint_base_link_pose'),
                'pairs': fit['pairs'],
                'pair_centers': fit['pair_centers'],
                'pair_is_end': fit['pair_is_end'],
                'fitted_line': fit['fitted_line'],
                'bar_end_points': fit['bar_end_points'],
                'ocf_position': fit['ocf_position'],
                'bar_length_observed': fit['bar_length_observed'],
                'angle_to_world_z_rad': float(_angle_to_world_z_rad(axis)),
                'center_to_line_dist_max_m': fit['center_to_line_dist_max_m'],
                'center_to_line_dist_rms_m': fit['center_to_line_dist_rms_m'],
            })
            fits_for_plot.append((f"{file_name}#{i}", fit, bar_action_path, bar_name))

    if export:
        out_path = os.path.join(data_folder, 'compiled_bar_holding_acc.json')
        with open(out_path, 'w') as f:
            json.dump(compiled, f, indent=4)
        print(f"\nexported {len(compiled)} takes to {out_path}")

    if viewer and fits_for_plot:
        # Per-take data plots (two per row, tall + scrollable). The layout diagram
        # is a SEPARATE figure (below) so it doesn't crowd these plots and can be
        # zoomed on its own.
        n = len(fits_for_plot)
        ncols = 2
        nrows = (n + ncols - 1) // ncols
        fig = plt.figure(figsize=(7.0 * ncols, 5.5 * nrows))
        for idx, (label, fit, bap, bn) in enumerate(fits_for_plot, start=1):
            ax = fig.add_subplot(nrows, ncols, idx, projection='3d')
            _plot_take(ax, fit, label, bar_name=bn)
        fig.subplots_adjust(left=0.03, right=0.99, top=0.97, bottom=0.03, hspace=0.25, wspace=0.1)
        enable_scroll_zoom(fig)   # mouse-wheel zoom on every subplot

        # Layout diagram — its OWN figure (2D top view): whole model + all tested
        # bars (red) + environment. Uses the first take's parked base.
        active_names = {bn for (_l, _f, _b, bn) in fits_for_plot if bn}
        _l0, fit0, bap0, _bn0 = fits_for_plot[0]
        problem_dir = problem_override or problem_dir_from_bar_action_path(bap0)
        tips0 = np.asarray(fit0['bar_end_points'], dtype=float)
        layout = build_layout(problem_dir, active_names,
                              tested_bar_endpoints=(tips0[0], tips0[1]), env_3dm=env_3dm,
                              tested_bar_action_path=bap0)
        if layout['source'] == 'degraded':
            print("  [layout] no solved cell-state in problem folder — "
                  "showing tested bar + origin only")
        layfig = plt.figure(figsize=(9, 8))
        draw_layout_2d(layfig.add_subplot(111), layout)
        enable_scroll_zoom(layfig)
        show_scrollable(fig)      # tall + scrollable window (layout shown too)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('batch', nargs='?', default='20260706',
                        help='batch folder name under EXPERIMENT_DATA_DIRECTORY/bar_holding_acc_data/ '
                             '(default: 20260706)')
    parser.add_argument('--no-export', action='store_true')
    parser.add_argument('--viewer', action='store_true')
    parser.add_argument('--problem', default=None,
                        help='override the layout problem folder: an absolute path '
                             'or a folder name under DESIGN_DATA_DIRECTORY')
    parser.add_argument('--env-3dm', dest='env_3dm', default=None,
                        help='Rhino .3dm whose "Environment Obstacles" layer is drawn '
                             'as the layout environment')
    args = parser.parse_args()

    problem_override = None
    if args.problem:
        problem_override = (args.problem if os.path.isdir(args.problem)
                            else os.path.join(DESIGN_DATA_DIRECTORY, args.problem))

    # Fall back to the default phase1 .3dm so the environment draws without the
    # flag; --env-3dm still overrides. Print a status line so it's never silently blank.
    env_3dm = args.env_3dm or (DEFAULT_ENV_3DM if os.path.exists(DEFAULT_ENV_3DM) else None)
    if env_3dm:
        print(f"[layout] environment from {os.path.basename(env_3dm)}")
    else:
        print("[layout] no environment .3dm found; pass --env-3dm <file.3dm> to draw obstacles")

    data_folder = os.path.join(EXPERIMENT_DATA_DIRECTORY, 'bar_holding_acc_data', args.batch)
    process_batch(data_folder, export=not args.no_export, viewer=args.viewer,
                  problem_override=problem_override, env_3dm=env_3dm)
