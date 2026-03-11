"""Compare base_pose between two punch-validation JSON files.

This script compares takes from two JSON recordings (e.g., two mocap captures) and
quantifies base frame differences in translation and orientation.

Default input targets:
    data/calibration_data/20260303/base_by_Bob/*20260310*.json

It computes, per matched take index:
- Position delta (dx, dy, dz) in mm and translation norm
- Relative rotation (roll, pitch, yaw) in deg
- Total angular difference in deg (shortest quaternion distance)

It also saves plots to the same folder as the input files.
"""

import argparse
import glob
import json
import os
from dataclasses import dataclass
from typing import List

import matplotlib.pyplot as plt
import numpy as np


HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_FOLDER = os.path.join(HERE, '20260303', 'base_by_Bob')
DEFAULT_GLOB = '*20260310*.json'


@dataclass
class BasePose:
    take_index: int
    timestamp: str
    position: np.ndarray  # shape (3,), meters
    quaternion: np.ndarray  # shape (4,), [x, y, z, w]


def _normalize_quaternion(q: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise ValueError('Encountered zero-norm quaternion in input data.')
    return q / norm


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=float,
    )


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_to_rpy_deg(r: np.ndarray) -> np.ndarray:
    sy = np.sqrt(r[0, 0] ** 2 + r[1, 0] ** 2)
    singular = sy < 1e-8

    if not singular:
        roll = np.arctan2(r[2, 1], r[2, 2])
        pitch = np.arctan2(-r[2, 0], sy)
        yaw = np.arctan2(r[1, 0], r[0, 0])
    else:
        roll = np.arctan2(-r[1, 2], r[1, 1])
        pitch = np.arctan2(-r[2, 0], sy)
        yaw = 0.0

    return np.rad2deg(np.array([roll, pitch, yaw], dtype=float))


def wrap_deg(angle_deg: np.ndarray) -> np.ndarray:
    return (angle_deg + 180.0) % 360.0 - 180.0


def load_json(path: str) -> dict:
    with open(path, 'r') as f:
        return json.load(f)


def extract_base_poses(data: dict, source: str) -> List[BasePose]:
    takes = data.get('takes', [])
    if not takes:
        raise ValueError(f'No takes found in {source}')

    poses = []
    for i, take in enumerate(takes):
        if 'base_pose' not in take:
            raise ValueError(f'Missing base_pose in take {i + 1} of {source}')
        base_pose = take['base_pose']
        position = np.array(base_pose['position'], dtype=float)
        quat = _normalize_quaternion(np.array(base_pose['quaternion'], dtype=float))
        poses.append(
            BasePose(
                take_index=i + 1,
                timestamp=str(take.get('timestamp', '')),
                position=position,
                quaternion=quat,
            )
        )
    return poses


def choose_input_files(folder: str, pattern: str, file_a: str, file_b: str):
    if file_a and file_b:
        return file_a, file_b

    files = sorted(glob.glob(os.path.join(folder, pattern)))
    if len(files) < 2:
        raise FileNotFoundError(
            f'Need at least 2 files in {folder} matching {pattern}; found {len(files)}'
        )
    if len(files) > 2:
        print('More than 2 files matched. Using the first two sorted by filename:')
        print(f'  A: {os.path.basename(files[0])}')
        print(f'  B: {os.path.basename(files[1])}')
    return files[0], files[1]


def analyze(poses_a: List[BasePose], poses_b: List[BasePose]) -> dict:
    n = min(len(poses_a), len(poses_b))
    if n == 0:
        raise ValueError('No comparable takes found.')

    if len(poses_a) != len(poses_b):
        print(
            f'Warning: take count mismatch (A={len(poses_a)}, B={len(poses_b)}). '
            f'Comparing first {n} aligned-by-index takes.'
        )

    pos_a = np.array([p.position for p in poses_a[:n]], dtype=float)
    pos_b = np.array([p.position for p in poses_b[:n]], dtype=float)
    dpos_mm = (pos_b - pos_a) * 1000.0
    dpos_norm_mm = np.linalg.norm(dpos_mm, axis=1)

    rpy_a = []
    rpy_b = []
    drpy = []
    angle_diff_deg = []

    for i in range(n):
        qa = poses_a[i].quaternion
        qb = poses_b[i].quaternion

        ra = quat_to_matrix(qa)
        rb = quat_to_matrix(qb)
        r_rel = rb @ ra.T

        rpy_a.append(matrix_to_rpy_deg(ra))
        rpy_b.append(matrix_to_rpy_deg(rb))
        drpy.append(wrap_deg(matrix_to_rpy_deg(r_rel)))

        # Shortest quaternion angular distance between the two orientations.
        dot = np.clip(np.abs(np.dot(qa, qb)), -1.0, 1.0)
        angle_diff_deg.append(np.rad2deg(2.0 * np.arccos(dot)))

    return {
        'n': n,
        'dpos_mm': dpos_mm,
        'dpos_norm_mm': dpos_norm_mm,
        'rpy_a_deg': np.array(rpy_a),
        'rpy_b_deg': np.array(rpy_b),
        'drpy_deg': np.array(drpy),
        'angle_diff_deg': np.array(angle_diff_deg),
    }


def print_summary(file_a: str, file_b: str, result: dict):
    dpos_mm = result['dpos_mm']
    norm_mm = result['dpos_norm_mm']
    drpy_deg = result['drpy_deg']
    angle_diff_deg = result['angle_diff_deg']

    print('=' * 72)
    print('Base Pose Comparison Summary')
    print('=' * 72)
    print(f'File A: {file_a}')
    print(f'File B: {file_b}')
    print(f'Compared takes: {result["n"]}')
    print('-' * 72)

    for i in range(result['n']):
        dx, dy, dz = dpos_mm[i]
        droll, dpitch, dyaw = drpy_deg[i]
        print(
            f'Take {i + 1:02d} | '
            f'dpos [mm]=[{dx:+.3f}, {dy:+.3f}, {dz:+.3f}] '
            f'| |dpos|={norm_mm[i]:.3f} mm '
            f'| dRPY [deg]=[{droll:+.4f}, {dpitch:+.4f}, {dyaw:+.4f}] '
            f'| dAngle={angle_diff_deg[i]:.4f} deg'
        )

    print('-' * 72)
    print(
        'Position norm [mm]: '
        f'mean={np.mean(norm_mm):.3f}, std={np.std(norm_mm):.3f}, max={np.max(norm_mm):.3f}'
    )
    print(
        'Angular diff [deg]: '
        f'mean={np.mean(angle_diff_deg):.4f}, std={np.std(angle_diff_deg):.4f}, '
        f'max={np.max(angle_diff_deg):.4f}'
    )
    print(
        'Yaw diff [deg]: '
        f'mean={np.mean(drpy_deg[:, 2]):.4f}, std={np.std(drpy_deg[:, 2]):.4f}, '
        f'max_abs={np.max(np.abs(drpy_deg[:, 2])):.4f}'
    )
    print('=' * 72)


def save_csv(output_csv: str, result: dict):
    n = result['n']
    dpos_mm = result['dpos_mm']
    norm_mm = result['dpos_norm_mm']
    drpy_deg = result['drpy_deg']
    angle_diff_deg = result['angle_diff_deg']

    header = (
        'take_index,dx_mm,dy_mm,dz_mm,dpos_norm_mm,'
        'droll_deg,dpitch_deg,dyaw_deg,angle_diff_deg\n'
    )
    lines = [header]
    for i in range(n):
        lines.append(
            f'{i + 1},'
            f'{dpos_mm[i, 0]:.9f},{dpos_mm[i, 1]:.9f},{dpos_mm[i, 2]:.9f},'
            f'{norm_mm[i]:.9f},'
            f'{drpy_deg[i, 0]:.9f},{drpy_deg[i, 1]:.9f},{drpy_deg[i, 2]:.9f},'
            f'{angle_diff_deg[i]:.9f}\n'
        )

    with open(output_csv, 'w') as f:
        f.writelines(lines)


def plot_results(file_a: str, file_b: str, poses_a: List[BasePose], poses_b: List[BasePose], result: dict, output_dir: str, show: bool):
    idx = np.arange(1, result['n'] + 1)
    pos_a_mm = np.array([p.position for p in poses_a[: result['n']]]) * 1000.0
    pos_b_mm = np.array([p.position for p in poses_b[: result['n']]]) * 1000.0
    dpos_mm = result['dpos_mm']
    drpy_deg = result['drpy_deg']
    angle_diff_deg = result['angle_diff_deg']
    rpy_a = result['rpy_a_deg']
    rpy_b = result['rpy_b_deg']

    label_a = os.path.basename(file_a)
    label_b = os.path.basename(file_b)

    fig1, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig1.suptitle('Base Pose Translation Comparison', fontsize=13, fontweight='bold')

    ax = axes[0, 0]
    for c, name in enumerate(['X', 'Y', 'Z']):
        ax.plot(idx, pos_a_mm[:, c], '-o', label=f'{name} A', alpha=0.85)
        ax.plot(idx, pos_b_mm[:, c], '--s', label=f'{name} B', alpha=0.85)
    ax.set_xlabel('Take index')
    ax.set_ylabel('Position (mm)')
    ax.set_title('Base Position Components')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, ncol=2)

    ax = axes[0, 1]
    for c, name, color in zip(range(3), ['dX', 'dY', 'dZ'], ['tab:blue', 'tab:orange', 'tab:green']):
        ax.bar(idx + (c - 1) * 0.25, dpos_mm[:, c], width=0.25, label=f'{name} (B-A)', color=color)
    ax.set_xlabel('Take index')
    ax.set_ylabel('Delta position (mm)')
    ax.set_title('Per-Take Translation Delta')
    ax.grid(True, alpha=0.3, axis='y')
    ax.legend(fontsize=8)

    ax = axes[1, 0]
    ax.plot(idx, np.linalg.norm(dpos_mm, axis=1), '-o', color='tab:red')
    ax.set_xlabel('Take index')
    ax.set_ylabel('|B-A| (mm)')
    ax.set_title('Translation Norm Difference')
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.scatter(pos_a_mm[:, 0], pos_a_mm[:, 1], c=idx, cmap='Blues', label='A', s=50)
    ax.scatter(pos_b_mm[:, 0], pos_b_mm[:, 1], c=idx, cmap='Oranges', marker='x', label='B', s=60)
    for i in range(result['n']):
        ax.plot([pos_a_mm[i, 0], pos_b_mm[i, 0]], [pos_a_mm[i, 1], pos_b_mm[i, 1]], color='gray', alpha=0.5)
    ax.set_xlabel('Base X (mm)')
    ax.set_ylabel('Base Y (mm)')
    ax.set_title('Base XY Shift (A -> B)')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    fig1.tight_layout(rect=[0, 0, 1, 0.95])
    out1 = os.path.join(output_dir, 'base_pose_diff_translation.png')
    fig1.savefig(out1, dpi=150)

    fig2, axes2 = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig2.suptitle('Base Pose Orientation Comparison', fontsize=13, fontweight='bold')

    ax = axes2[0]
    ax.plot(idx, rpy_a[:, 2], '-o', label='Yaw A', color='tab:blue')
    ax.plot(idx, rpy_b[:, 2], '--s', label='Yaw B', color='tab:orange')
    ax.set_ylabel('Yaw (deg)')
    ax.set_title('Absolute Base Yaw per Take')
    ax.grid(True, alpha=0.3)
    ax.legend()

    ax = axes2[1]
    ax.plot(idx, drpy_deg[:, 0], '-o', label='dRoll (B-A)', color='tab:purple')
    ax.plot(idx, drpy_deg[:, 1], '-o', label='dPitch (B-A)', color='tab:green')
    ax.plot(idx, drpy_deg[:, 2], '-o', label='dYaw (B-A)', color='tab:blue')
    ax.plot(idx, angle_diff_deg, '-s', label='Total angle diff', color='tab:red', linewidth=2)
    ax.set_xlabel('Take index')
    ax.set_ylabel('Angle (deg)')
    ax.set_title('Relative Orientation Difference (A -> B)')
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=2)

    fig2.tight_layout(rect=[0, 0, 1, 0.95])
    out2 = os.path.join(output_dir, 'base_pose_diff_orientation.png')
    fig2.savefig(out2, dpi=150)

    print(f'Plot saved: {out1}')
    print(f'Plot saved: {out2}')

    if show:
        if os.environ.get('DISPLAY'):
            plt.show(block=True)
        else:
            print('DISPLAY is not set; skipping interactive pop-up and saving plots only.')

    plt.close(fig1)
    plt.close(fig2)

    print(f'Compared files:\n  A: {label_a}\n  B: {label_b}')


def parse_args():
    parser = argparse.ArgumentParser(description='Compare base_pose between two JSON files.')
    parser.add_argument('--folder', default=DEFAULT_FOLDER, help='Folder with JSON files.')
    parser.add_argument('--glob', dest='pattern', default=DEFAULT_GLOB, help='Glob pattern to select files.')
    parser.add_argument('--file-a', default='', help='Explicit path to file A.')
    parser.add_argument('--file-b', default='', help='Explicit path to file B.')
    parser.add_argument(
        '--show',
        action='store_true',
        help='Pop figures interactively (requires DISPLAY).',
    )
    return parser.parse_args()


def main():
    args = parse_args()
    file_a, file_b = choose_input_files(args.folder, args.pattern, args.file_a, args.file_b)

    data_a = load_json(file_a)
    data_b = load_json(file_b)
    poses_a = extract_base_poses(data_a, file_a)
    poses_b = extract_base_poses(data_b, file_b)

    result = analyze(poses_a, poses_b)
    print_summary(file_a, file_b, result)

    output_dir = os.path.dirname(file_a)
    csv_path = os.path.join(output_dir, 'base_pose_diff_summary.csv')
    save_csv(csv_path, result)
    print(f'CSV saved: {csv_path}')

    plot_results(file_a, file_b, poses_a, poses_b, result, output_dir, show=args.show)


if __name__ == '__main__':
    main()
