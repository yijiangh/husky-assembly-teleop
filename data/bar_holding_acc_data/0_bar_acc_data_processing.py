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

from husky_assembly_teleop import EXPERIMENT_DATA_DIRECTORY
from husky_assembly_teleop.mocap_experiment import fit_bar_from_markerset


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


def _plot_take(ax, fit, take_label):
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
    ax.set_title(
        f"{take_label}\n"
        f"angle(fit-axis, world-Z) = {angle_to_z_deg:.3f}°  |  "
        f"bar_len = {bar_len:.4f} m  |  "
        f"center_to_line_dist_max = {fit['center_to_line_dist_max_m']*1000:.2f} mm"
    )
    ax.legend(loc='upper left', fontsize=8)
    _equal_axes_3d(ax,
                   np.vstack([pair_centers, tips, ocf,
                              z_a, z_b, fit_a, fit_b]))


def process_batch(data_folder, export=True, viewer=False):
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
        print(f"\n=== {file_name} (mocap_axis_convention={saved_convention}) ===")
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
            print(
                f"  take {i}: "
                f"ocf=({ocf[0]:.4f}, {ocf[1]:.4f}, {ocf[2]:.4f}) m | "
                f"axis=({axis[0]:+.4f}, {axis[1]:+.4f}, {axis[2]:+.4f}) | "
                f"angle_to_Z={angle_to_z_deg:.3f}° | "
                f"bar_len={fit['bar_length_observed']:.4f} m | "
                f"center_to_line_dist_max={fit['center_to_line_dist_max_m']*1000:.2f} mm | "
                f"center_to_line_dist_rms={fit['center_to_line_dist_rms_m']*1000:.2f} mm"
            )

            compiled.append({
                'source_file': file_name,
                'take_index': i,
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
            fits_for_plot.append((f"{file_name}#{i}", fit))

    if export:
        out_path = os.path.join(data_folder, 'compiled_bar_holding_acc.json')
        with open(out_path, 'w') as f:
            json.dump(compiled, f, indent=4)
        print(f"\nexported {len(compiled)} takes to {out_path}")

    if viewer and fits_for_plot:
        n = len(fits_for_plot)
        ncols = min(n, 3)
        nrows = (n + ncols - 1) // ncols
        fig = plt.figure(figsize=(6 * ncols, 5 * nrows))
        for idx, (label, fit) in enumerate(fits_for_plot, start=1):
            ax = fig.add_subplot(nrows, ncols, idx, projection='3d')
            _plot_take(ax, fit, label)
        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('batch', help='batch folder name under EXPERIMENT_DATA_DIRECTORY/bar_holding_acc_data/')
    parser.add_argument('--no-export', action='store_true')
    parser.add_argument('--viewer', action='store_true')
    args = parser.parse_args()

    data_folder = os.path.join(EXPERIMENT_DATA_DIRECTORY, 'bar_holding_acc_data', args.batch)
    process_batch(data_folder, export=not args.no_export, viewer=args.viewer)
