"""
Visualize robot joint configuration values per axis across all data takes.

For each data batch (j0, j1), plots each of the 6 joint axes as a subplot,
with dashed lines separating different data takes (files).

Usage:
    python visualize_joint_configs.py
"""

import os
import sys
import json
import glob
import numpy as np
import matplotlib.pyplot as plt

# Add parent directory to path for config_loader
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from config_loader import load_config, get_data_folder, get_joint_names

JOINT_SHORT_NAMES = [
    'shoulder_pan', 'shoulder_lift', 'elbow',
    'wrist_1', 'wrist_2', 'wrist_3',
]

# Use a colorblind-friendly qualitative palette (tab10)
COLORS = plt.cm.tab10.colors
MARKERS = ['o', 's', '^', 'D', 'v', '<', '>', 'p', '*', 'h']


def load_takes(batch_folder):
    """Load all calibration JSON files (takes) from a batch folder, sorted by name."""
    pattern = os.path.join(batch_folder, 'calibration_*.json')
    files = sorted(glob.glob(pattern))
    takes = []
    for fpath in files:
        with open(fpath, 'r') as f:
            data = json.load(f)
        takes.append({
            'file_name': os.path.basename(fpath),
            'raw_data': data['raw_data'],
        })
    return takes


def plot_joint_configs(batch_name, takes, joint_names):
    """Create a figure with 6 subplots (one per joint axis) showing joint values across all takes."""
    fig, axes = plt.subplots(6, 1, figsize=(14, 16), sharex=True)
    fig.suptitle(f'{batch_name} — Joint Configurations Across Takes ({len(takes)} takes)', fontsize=14)

    sample_offset = 0
    take_boundaries = []

    for take_idx, take in enumerate(takes):
        n_samples = len(take['raw_data'])
        joint_confs = np.array([entry['joint_conf'] for entry in take['raw_data']])  # (N, 6)
        x = np.arange(sample_offset, sample_offset + n_samples)

        color = COLORS[take_idx % len(COLORS)]
        marker = MARKERS[take_idx % len(MARKERS)]
        label = f"Take {take_idx + 1} ({n_samples} pts)"

        for ax_idx in range(6):
            axes[ax_idx].scatter(
                x, np.degrees(joint_confs[:, ax_idx]),
                c=[color], marker=marker, s=20, alpha=0.8, label=label if ax_idx == 0 else None,
                edgecolors='none',
            )

        take_boundaries.append(sample_offset + n_samples)
        sample_offset += n_samples

    # Draw dashed separators and label axes
    for ax_idx in range(6):
        ax = axes[ax_idx]
        for boundary in take_boundaries[:-1]:
            ax.axvline(x=boundary - 0.5, color='gray', linestyle='--', linewidth=1, alpha=0.7)
        ax.set_ylabel(f'{JOINT_SHORT_NAMES[ax_idx]}\n(deg)', fontsize=9)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Sample index')

    # Single shared legend at the top
    handles, labels = [], []
    for take_idx, take in enumerate(takes):
        n_samples = len(take['raw_data'])
        h = plt.Line2D([0], [0], marker=MARKERS[take_idx % len(MARKERS)],
                        color='w', markerfacecolor=COLORS[take_idx % len(COLORS)],
                        markersize=8, linestyle='None')
        handles.append(h)
        labels.append(f"Take {take_idx + 1} ({n_samples} pts)")
    fig.legend(handles, labels, loc='upper right', fontsize=8, ncol=2,
               bbox_to_anchor=(0.98, 0.98))

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    return fig


def main():
    config = load_config()
    data_folder = get_data_folder(config['date_folder'])
    joint_names = get_joint_names(config['robot_name'], config.get('arm', 'left'))

    for batch_name in config['data_batches']:
        batch_folder = os.path.join(data_folder, batch_name)
        takes = load_takes(batch_folder)
        if not takes:
            print(f"No calibration files found in {batch_folder}, skipping.")
            continue

        print(f"{batch_name}: loaded {len(takes)} takes, "
              f"{sum(len(t['raw_data']) for t in takes)} total samples")

        fig = plot_joint_configs(batch_name, takes, joint_names)

        out_path = os.path.join(batch_folder, f'{batch_name}_joint_configs.png')
        fig.savefig(out_path, dpi=150, bbox_inches='tight')
        print(f"  Saved: {out_path}")

        plt.show()


if __name__ == '__main__':
    main()
