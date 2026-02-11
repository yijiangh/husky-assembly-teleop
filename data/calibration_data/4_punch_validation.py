"""
Analyze punch tool calibration validation data.

This script processes the JSON data collected by the punch validation workflow:
1. Mount punch tool on UR5e arm
2. Jog robot so punch matches external target
3. Record world_from_punch_tip via FK (multiple takes from different base positions)

The script shows the mismatch among world_from_punch_tip across all collected
validation takes. If calibration is perfect, the punch tip should map to the
exact same world position regardless of the base pose.

Usage:
    python 4_punch_validation.py
    python 4_punch_validation.py --date 20260126
    python 4_punch_validation.py --file path/to/punch_validation_YYYYMMDD_HHMM.json
"""

import os
import sys
import json
import argparse
import glob
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from config_loader import load_config, HERE


def load_latest_validation_file(date_folder=None):
    """Find and load the most recently created punch validation JSON.

    If date_folder is given, look in HERE/date_folder/punch_validation/.
    Otherwise, scan all date folders for punch_validation_*.json files.
    """
    if date_folder:
        punch_dir = os.path.join(HERE, date_folder, "punch_validation")
        pattern = os.path.join(punch_dir, "punch_validation_*.json")
        files = sorted(glob.glob(pattern))
        if not files:
            print(f"No punch validation files found in {punch_dir}")
            sys.exit(1)
        return files[-1]

    # Scan all date folders
    pattern = os.path.join(HERE, "*", "punch_validation", "punch_validation_*.json")
    files = sorted(glob.glob(pattern))
    if not files:
        print(f"No punch validation files found under {HERE}/*/punch_validation/")
        sys.exit(1)
    return files[-1]


def load_validation_data(filepath):
    """Load validation data from JSON file."""
    with open(filepath, 'r') as f:
        data = json.load(f)
    return data


def analyze_position_mismatch(takes):
    """Analyze the position mismatch across all takes.

    Returns dict with positions array, mean, distances from mean, and stats.
    """
    positions = np.array([t['world_from_punch_tip']['position'] for t in takes])
    mean_pos = np.mean(positions, axis=0)

    # Distance of each take from the mean position
    distances_from_mean_mm = np.linalg.norm(positions - mean_pos, axis=1) * 1000

    # Pairwise distances
    n = len(positions)
    pairwise_mm = []
    for i in range(n):
        for j in range(i + 1, n):
            pairwise_mm.append(np.linalg.norm(positions[i] - positions[j]) * 1000)
    pairwise_mm = np.array(pairwise_mm)

    return {
        'positions': positions,
        'mean_pos': mean_pos,
        'distances_from_mean_mm': distances_from_mean_mm,
        'pairwise_mm': pairwise_mm,
    }


def analyze_orientation_mismatch(takes):
    """Analyze the orientation mismatch across all takes.

    Returns dict with quaternions, rotation matrices, and angular deviations.
    """
    quaternions = [t['world_from_punch_tip']['quaternion'] for t in takes]

    # Convert quaternions to rotation matrices
    # pybullet quaternion format: [x, y, z, w]
    rotation_matrices = []
    for q in quaternions:
        x, y, z, w = q
        R = np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
            [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
            [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
        ])
        rotation_matrices.append(R)

    # Compute angular deviations per axis from mean
    angular_deviations = {'X': [], 'Y': [], 'Z': []}
    for axis_idx, axis_name in enumerate(['X', 'Y', 'Z']):
        axes_data = np.array([rm[:, axis_idx] for rm in rotation_matrices])
        axis_mean = np.mean(axes_data, axis=0)
        axis_mean = axis_mean / np.linalg.norm(axis_mean)
        angular_deviations[axis_name] = np.array([
            np.rad2deg(np.arccos(np.clip(np.dot(a, axis_mean), -1, 1)))
            for a in axes_data
        ])

    return {
        'quaternions': quaternions,
        'rotation_matrices': rotation_matrices,
        'angular_deviations': angular_deviations,
    }


def analyze_base_diversity(takes):
    """Analyze the diversity of base poses across takes."""
    base_positions = np.array([t['base_pose']['position'] for t in takes])
    base_quats = [t['base_pose']['quaternion'] for t in takes]

    def quat_to_yaw(q):
        x, y, z, w = q
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return np.rad2deg(np.arctan2(siny_cosp, cosy_cosp))

    base_yaws = np.array([quat_to_yaw(q) for q in base_quats])

    return {
        'base_positions': base_positions,
        'base_yaws': base_yaws,
    }


def plot_results(takes, pos_analysis, ori_analysis, base_analysis, output_dir):
    """Generate analysis plots."""
    n_takes = len(takes)
    positions = pos_analysis['positions']
    mean_pos = pos_analysis['mean_pos']
    dist_mm = pos_analysis['distances_from_mean_mm']
    pairwise_mm = pos_analysis['pairwise_mm']
    ang_dev = ori_analysis['angular_deviations']
    base_pos = base_analysis['base_positions']
    base_yaws = base_analysis['base_yaws']

    # ========== Figure 1: Position Mismatch (2x2) ==========
    fig1, axes1 = plt.subplots(2, 2, figsize=(14, 10))
    fig1.suptitle(
        f'Punch Validation: Position Mismatch | {n_takes} takes',
        fontsize=13, fontweight='bold'
    )

    # Plot 1: Per-take distance from mean
    ax = axes1[0, 0]
    take_indices = np.arange(1, n_takes + 1)
    colors = plt.cm.coolwarm(dist_mm / max(dist_mm.max(), 1e-6))
    ax.bar(take_indices, dist_mm, color=colors, edgecolor='black', linewidth=0.5)
    ax.axhline(y=np.mean(dist_mm), color='red', linestyle='--', linewidth=1,
               label=f'Mean: {np.mean(dist_mm):.2f} mm')
    ax.set_xlabel('Take #')
    ax.set_ylabel('Distance from Mean (mm)')
    ax.set_title(f'Per-Take Position Error\n'
                 f'Max: {np.max(dist_mm):.2f} mm | Std: {np.std(dist_mm):.2f} mm')
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')

    # Plot 2: XYZ scatter of punch tip positions
    ax = axes1[0, 1]
    pos_mm = positions * 1000
    mean_mm = mean_pos * 1000
    for i, label in enumerate(['X', 'Y', 'Z']):
        ax.scatter(take_indices, pos_mm[:, i], alpha=0.8, s=30, label=f'{label}')
        ax.axhline(y=mean_mm[i], linestyle=':', alpha=0.5)
    ax.set_xlabel('Take #')
    ax.set_ylabel('Position (mm)')
    ax.set_title(f'Punch Tip Position per Take\n'
                 f'Mean: [{mean_mm[0]:.2f}, {mean_mm[1]:.2f}, {mean_mm[2]:.2f}] mm')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Plot 3: CDF of distance from mean
    ax = axes1[1, 0]
    sorted_dist = np.sort(dist_mm)
    cdf = np.arange(1, len(sorted_dist) + 1) / len(sorted_dist)
    ax.plot(sorted_dist, cdf, 'b-', linewidth=2)
    if n_takes > 1:
        p95 = np.percentile(dist_mm, 95)
        ax.axhline(y=0.95, color='r', linestyle='--', alpha=0.7, label='95%')
        ax.axvline(x=p95, color='r', linestyle='--', alpha=0.7)
        ax.plot(p95, 0.95, 'ro', markersize=8)
        ax.annotate(f'{p95:.2f} mm', xy=(p95, 0.95), xytext=(10, -20),
                    textcoords='offset points', fontsize=10, color='r',
                    arrowprops=dict(arrowstyle='->', color='r', alpha=0.7))
        ax.legend()
    ax.set_xlabel('Distance from Mean (mm)')
    ax.set_ylabel('CDF')
    ax.set_title('Position Error CDF')
    ax.grid(True, alpha=0.3)

    # Plot 4: Pairwise distance histogram
    ax = axes1[1, 1]
    if len(pairwise_mm) > 0:
        ax.hist(pairwise_mm, bins=max(10, n_takes), alpha=0.7, color='steelblue',
                edgecolor='black', linewidth=0.5)
        ax.axvline(x=np.mean(pairwise_mm), color='red', linestyle='--',
                   label=f'Mean: {np.mean(pairwise_mm):.2f} mm')
        ax.axvline(x=np.max(pairwise_mm), color='orange', linestyle='--',
                   label=f'Max: {np.max(pairwise_mm):.2f} mm')
        ax.legend()
    ax.set_xlabel('Pairwise Distance (mm)')
    ax.set_ylabel('Count')
    ax.set_title('Pairwise Distance Distribution')
    ax.grid(True, alpha=0.3, axis='y')

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    output_path = os.path.join(output_dir, 'punch_validation_position.png')
    plt.savefig(output_path, dpi=150)
    print(f'Position mismatch plot saved to: {output_path}')
    plt.show()
    plt.close(fig1)

    # ========== Figure 2: 3D scatter of punch tip positions ==========
    fig2 = plt.figure(figsize=(10, 8))
    ax3d = fig2.add_subplot(111, projection='3d')

    # Color by take index
    colors_3d = plt.cm.viridis(np.linspace(0.2, 0.9, n_takes))
    for i in range(n_takes):
        ax3d.scatter(*pos_mm[i], color=colors_3d[i], s=60, alpha=0.8,
                     edgecolors='black', linewidths=0.5)
        ax3d.text(pos_mm[i, 0], pos_mm[i, 1], pos_mm[i, 2],
                  f' {i+1}', fontsize=7, alpha=0.8)
        # Line from point to mean
        ax3d.plot([pos_mm[i, 0], mean_mm[0]],
                  [pos_mm[i, 1], mean_mm[1]],
                  [pos_mm[i, 2], mean_mm[2]],
                  color=colors_3d[i], linewidth=0.8, alpha=0.4)

    ax3d.scatter(*mean_mm, color='red', s=200, marker='*', zorder=5, label='Mean')

    ax3d.set_xlabel('X (mm)')
    ax3d.set_ylabel('Y (mm)')
    ax3d.set_zlabel('Z (mm)')
    ax3d.set_title(
        f'Punch Tip Positions in World Frame\n'
        f'Spread: {np.max(dist_mm):.2f} mm max | {np.std(dist_mm):.2f} mm std | '
        f'{n_takes} takes'
    )
    ax3d.legend()

    plt.tight_layout()
    output_path = os.path.join(output_dir, 'punch_validation_3d.png')
    plt.savefig(output_path, dpi=150)
    print(f'3D scatter plot saved to: {output_path}')
    plt.show()
    plt.close(fig2)

    # ========== Figure 3: Orientation + Base Diversity (1x2) ==========
    fig3, axes3 = plt.subplots(1, 2, figsize=(14, 5))
    fig3.suptitle(
        f'Punch Validation: Orientation & Base Diversity | {n_takes} takes',
        fontsize=13, fontweight='bold'
    )

    # Plot: Angular deviation per axis from mean
    ax = axes3[0]
    axis_colors = ['r', 'g', 'b']
    for axis_name, color in zip(['X', 'Y', 'Z'], axis_colors):
        ax.plot(take_indices, ang_dev[axis_name], '-o', color=color, alpha=0.8,
                markersize=5, label=f'{axis_name}-axis (max: {np.max(ang_dev[axis_name]):.3f} deg)')
    ax.set_xlabel('Take #')
    ax.set_ylabel('Angular Deviation from Mean (deg)')
    ax.set_title('Orientation Mismatch per Axis')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Plot: Base position scatter with yaw arrows
    ax = axes3[1]
    base_x_mm = base_pos[:, 0] * 1000
    base_y_mm = base_pos[:, 1] * 1000
    yaw_rad = np.deg2rad(base_yaws)
    arrow_dx = np.cos(yaw_rad)
    arrow_dy = np.sin(yaw_rad)

    ax.scatter(base_x_mm, base_y_mm, c=take_indices, cmap='viridis', s=40, zorder=2)
    arrow_scale = max(np.ptp(base_x_mm), np.ptp(base_y_mm), 200) * 0.08
    ax.quiver(base_x_mm, base_y_mm, arrow_dx * arrow_scale, arrow_dy * arrow_scale,
              angles='xy', scale_units='xy', scale=1, color='red', alpha=0.6,
              width=0.004, headwidth=3, headlength=4, zorder=3)

    # Label each point with take number
    for i in range(n_takes):
        ax.annotate(f'{i+1}', (base_x_mm[i], base_y_mm[i]),
                    textcoords='offset points', xytext=(5, 5), fontsize=7, alpha=0.8)

    ax.set_xlabel('Base X (mm)')
    ax.set_ylabel('Base Y (mm)')
    x_range = np.ptp(base_x_mm)
    y_range = np.ptp(base_y_mm)
    yaw_range = np.ptp(base_yaws)
    ax.set_title(f'Base Position & Yaw Diversity\n'
                 f'X range: {x_range:.1f} mm | Y range: {y_range:.1f} mm | '
                 f'Yaw range: {yaw_range:.1f} deg')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.92])
    output_path = os.path.join(output_dir, 'punch_validation_diversity.png')
    plt.savefig(output_path, dpi=150)
    print(f'Diversity plot saved to: {output_path}')
    plt.show()
    plt.close(fig3)


def print_summary(takes, pos_analysis, ori_analysis, base_analysis):
    """Print a text summary of the validation results."""
    n = len(takes)
    dist_mm = pos_analysis['distances_from_mean_mm']
    pairwise_mm = pos_analysis['pairwise_mm']
    mean_pos = pos_analysis['mean_pos']
    ang_dev = ori_analysis['angular_deviations']

    print('=' * 60)
    print(f'Punch Calibration Validation Summary ({n} takes)')
    print('=' * 60)

    print(f'\nMean punch tip position (m): '
          f'[{mean_pos[0]:.6f}, {mean_pos[1]:.6f}, {mean_pos[2]:.6f}]')

    print(f'\nPosition Mismatch (distance from mean):')
    print(f'  Mean:   {np.mean(dist_mm):.3f} mm')
    print(f'  Std:    {np.std(dist_mm):.3f} mm')
    print(f'  Max:    {np.max(dist_mm):.3f} mm')
    if n > 1:
        print(f'  95th %%: {np.percentile(dist_mm, 95):.3f} mm')

    if len(pairwise_mm) > 0:
        print(f'\nPairwise Distances:')
        print(f'  Mean:   {np.mean(pairwise_mm):.3f} mm')
        print(f'  Max:    {np.max(pairwise_mm):.3f} mm')

    print(f'\nOrientation Mismatch (angular deviation from mean):')
    for axis_name in ['X', 'Y', 'Z']:
        vals = ang_dev[axis_name]
        print(f'  {axis_name}-axis: max={np.max(vals):.3f} deg, '
              f'mean={np.mean(vals):.3f} deg')

    base_pos = base_analysis['base_positions']
    base_yaws = base_analysis['base_yaws']
    print(f'\nBase Diversity:')
    print(f'  X range: {np.ptp(base_pos[:, 0]) * 1000:.1f} mm')
    print(f'  Y range: {np.ptp(base_pos[:, 1]) * 1000:.1f} mm')
    print(f'  Yaw range: {np.ptp(base_yaws):.1f} deg')

    print('=' * 60)


def main():
    parser = argparse.ArgumentParser(description='Analyze punch tool calibration validation data.')
    parser.add_argument('--file', type=str, default=None,
                        help='Path to punch validation JSON file. If not specified, uses the latest.')
    parser.add_argument('--date', type=str, default=None,
                        help='Date folder name (e.g., 20260126). If not specified, uses config default.')
    args = parser.parse_args()

    if args.file:
        filepath = args.file
    else:
        date_folder = args.date
        if date_folder is None:
            config = load_config()
            date_folder = config['date_folder']
        filepath = load_latest_validation_file(date_folder=date_folder)

    print(f'Loading: {filepath}')
    data = load_validation_data(filepath)
    takes = data['takes']

    if len(takes) < 2:
        print(f'Only {len(takes)} take(s) found. Need at least 2 for meaningful analysis.')
        if len(takes) == 1:
            pos = takes[0]['world_from_punch_tip']['position']
            print(f'  Take 1 position: [{pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f}] m')
        return

    print(f'Loaded {len(takes)} takes.')

    pos_analysis = analyze_position_mismatch(takes)
    ori_analysis = analyze_orientation_mismatch(takes)
    base_analysis = analyze_base_diversity(takes)

    print_summary(takes, pos_analysis, ori_analysis, base_analysis)

    output_dir = os.path.dirname(filepath)
    plot_results(takes, pos_analysis, ori_analysis, base_analysis, output_dir)

    print(f'\nAll plots saved to: {output_dir}')


if __name__ == '__main__':
    main()
