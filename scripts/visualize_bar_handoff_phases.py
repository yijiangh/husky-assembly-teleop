#!/usr/bin/env python3
"""
Visualize bar handoff mocap logs by phase.

    python scripts/visualize_bar_handoff_phases.py --input <log.txt> [--output <fig.png>] [--no-show]

The input is the console output of crl-husky's `mocap_logger` launch, with `--- <PHASE> ---` lines
typed in between to mark the phases (first used for the 2026-04-29 Alice/Cindy bar handoff test).
! Offline analysis for the old bar-holding tests: port into the `mocap_accuracy` plugin's recordings
  (`ctx.record`, doc/plugin_roadmap.md) once that exists.

Requested plots:
1) Position magnitude per phase index (mean +/- std, single line)
2) Euler X/Y/Z over phase indices (mean +/- std)
3) Position delta to START phase (X/Y/Z bars around 0)
4) Rotation delta to START phase (Euler X/Y/Z bars around 0)
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np


@dataclass
class Sample:
    """One logged pose: its phase, position (m) and orientation quaternion (x, y, z, w)."""

    phase: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def normalize_phase_name(raw: str) -> str:
    """Phase name from a `--- NAME ---` marker line, without the dashes."""
    text = raw.strip()
    if text.startswith("---") and text.endswith("---"):
        text = text.strip("-").strip()
    return text


def quat_to_euler_xyz_deg(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """Convert quaternion to Euler XYZ (roll, pitch, yaw) in degrees."""
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return tuple(np.degrees([roll, pitch, yaw]))


def wrap_deg(angle: np.ndarray) -> np.ndarray:
    """Wrap angles to [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


def parse_log(file_path: Path) -> Tuple[List[str], List[Sample]]:
    """Parse phased mocap logger text file."""
    phases_in_order: List[str] = []
    samples: List[Sample] = []
    current_phase = "UNPHASED"

    with file_path.open("r", encoding="utf-8") as f:
        for line in f:
            text = line.strip()

            if text.startswith("---") and text.endswith("---"):
                current_phase = normalize_phase_name(text)
                if current_phase not in phases_in_order:
                    phases_in_order.append(current_phase)
                continue

            if "{\"event\":\"mocap_logger_record\"" not in text:
                continue

            json_start = text.find("{")
            if json_start < 0:
                continue

            payload = text[json_start:]
            try:
                data = json.loads(payload)
            except json.JSONDecodeError:
                continue

            if data.get("status") != "ok":
                continue

            try:
                sample = Sample(
                    phase=current_phase,
                    x=float(data["x"]),
                    y=float(data["y"]),
                    z=float(data["z"]),
                    qx=float(data["qx"]),
                    qy=float(data["qy"]),
                    qz=float(data["qz"]),
                    qw=float(data["qw"]),
                )
            except KeyError:
                continue

            samples.append(sample)

    if not samples:
        raise ValueError(f"No mocap samples parsed from {file_path}")

    # Ensure UNPHASED appears if needed.
    for s in samples:
        if s.phase not in phases_in_order:
            phases_in_order.insert(0, s.phase)

    return phases_in_order, samples


def compute_phase_stats(phases: List[str], samples: List[Sample]) -> Dict[str, np.ndarray]:
    """Compute per-phase means/std and deltas to first phase."""
    pos_mean = []
    pos_std = []
    pos_mag_mean = []
    pos_mag_std = []

    euler_mean = []
    euler_std = []

    counts = []

    for phase in phases:
        phase_samples = [s for s in samples if s.phase == phase]
        counts.append(len(phase_samples))

        pos = np.array([[s.x, s.y, s.z] for s in phase_samples], dtype=float)
        pos_mean.append(pos.mean(axis=0))
        pos_std.append(pos.std(axis=0, ddof=0))

        mag = np.linalg.norm(pos, axis=1)
        pos_mag_mean.append(mag.mean())
        pos_mag_std.append(mag.std(ddof=0))

        eulers = np.array([quat_to_euler_xyz_deg(s.qx, s.qy, s.qz, s.qw) for s in phase_samples], dtype=float)
        euler_mean.append(eulers.mean(axis=0))
        euler_std.append(eulers.std(axis=0, ddof=0))

    pos_mean_arr = np.vstack(pos_mean)
    euler_mean_arr = np.vstack(euler_mean)

    # Deltas relative to START (first phase).
    pos_delta = pos_mean_arr - pos_mean_arr[0]
    euler_delta = wrap_deg(euler_mean_arr - euler_mean_arr[0])

    return {
        "counts": np.array(counts),
        "pos_mean": pos_mean_arr,
        "pos_std": np.vstack(pos_std),
        "pos_mag_mean": np.array(pos_mag_mean),
        "pos_mag_std": np.array(pos_mag_std),
        "euler_mean": euler_mean_arr,
        "euler_std": np.vstack(euler_std),
        "pos_delta": pos_delta,
        "euler_delta": euler_delta,
    }


def plot_phase_analysis(phases: List[str], stats: Dict[str, np.ndarray], output_path: Path, show: bool) -> None:
    """Draw the four per-phase plots into one figure, save it to `output_path` and optionally show it."""
    idx = np.arange(len(phases))

    fig, axs = plt.subplots(2, 2, figsize=(15, 10))

    # 1) Single line with mean+std per phase index.
    ax = axs[0, 0]
    ax.errorbar(
        idx,
        stats["pos_mag_mean"],
        yerr=stats["pos_mag_std"],
        fmt="o-",
        color="#1f77b4",
        ecolor="#333333",
        linewidth=2.0,
        capsize=5,
        markersize=7,
        label="Position magnitude",
    )
    ax.set_title("Position Magnitude per Phase (Mean +/- Std)")
    ax.set_xlabel("Phase Index")
    ax.set_ylabel("Distance from origin [m]")
    ax.set_xticks(idx, phases, rotation=30, ha="right")
    ax.grid(alpha=0.3)
    ax.legend()

    # 2) Euler X/Y/Z over phase index (analogous mean+std lines).
    ax = axs[0, 1]
    labels = ["Euler X", "Euler Y", "Euler Z"]
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c"]
    for k in range(3):
        ax.errorbar(
            idx,
            stats["euler_mean"][:, k],
            yerr=stats["euler_std"][:, k],
            fmt="o-",
            linewidth=1.8,
            capsize=4,
            markersize=6,
            color=colors[k],
            label=labels[k],
        )
    ax.set_title("Euler Angles per Phase (Mean +/- Std)")
    ax.set_xlabel("Phase Index")
    ax.set_ylabel("Angle [deg]")
    ax.set_xticks(idx, phases, rotation=30, ha="right")
    ax.axhline(0.0, color="black", linewidth=0.8)
    ax.grid(alpha=0.3)
    ax.legend()

    # 3) Position delta to START as signed bars.
    ax = axs[1, 0]
    width = 0.25
    ax.bar(idx - width, stats["pos_delta"][:, 0], width=width, color="#1f77b4", label="dX")
    ax.bar(idx, stats["pos_delta"][:, 1], width=width, color="#ff7f0e", label="dY")
    ax.bar(idx + width, stats["pos_delta"][:, 2], width=width, color="#2ca02c", label="dZ")
    ax.set_title("Position Delta to START")
    ax.set_xlabel("Phase Index")
    ax.set_ylabel("Delta [m]")
    ax.set_xticks(idx, phases, rotation=30, ha="right")
    ax.axhline(0.0, color="black", linewidth=0.8)
    ax.grid(alpha=0.3, axis="y")
    ax.legend()

    # 4) Rotation delta to START as signed bars (Euler X/Y/Z).
    ax = axs[1, 1]
    ax.bar(idx - width, stats["euler_delta"][:, 0], width=width, color="#1f77b4", label="dEuler X")
    ax.bar(idx, stats["euler_delta"][:, 1], width=width, color="#ff7f0e", label="dEuler Y")
    ax.bar(idx + width, stats["euler_delta"][:, 2], width=width, color="#2ca02c", label="dEuler Z")
    ax.set_title("Rotation Delta to START (Euler)")
    ax.set_xlabel("Phase Index")
    ax.set_ylabel("Delta [deg]")
    ax.set_xticks(idx, phases, rotation=30, ha="right")
    ax.axhline(0.0, color="black", linewidth=0.8)
    ax.grid(alpha=0.3, axis="y")
    ax.legend()

    fig.tight_layout()
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    print(f"Saved figure: {output_path}")

    print("\nPhase summary (mean +/- std):")
    for i, phase in enumerate(phases):
        pm = stats["pos_mag_mean"][i]
        ps = stats["pos_mag_std"][i]
        ex, ey, ez = stats["euler_mean"][i]
        sx, sy, sz = stats["euler_std"][i]
        n = int(stats["counts"][i])
        print(
            f"{i}: {phase:30s} n={n:3d} | "
            f"|p|={pm:.6f}+/-{ps:.6f} m | "
            f"euler=({ex:.3f}+/-{sx:.3f}, {ey:.3f}+/-{sy:.3f}, {ez:.3f}+/-{sz:.3f}) deg"
        )

    if show:
        plt.show()
    else:
        plt.close(fig)


def main() -> None:
    """Parse the log given on the command line, print the per-phase summary and plot it."""
    parser = argparse.ArgumentParser(description="Visualize bar handoff phases from mocap logger text file")
    parser.add_argument(
        "--input",
        type=Path,
        required=True,
        help="Path to input log text file",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Path to output figure (default: next to the input, as <input>_phases.png)",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open plot window (save only)",
    )
    args = parser.parse_args()
    if args.output is None:
        args.output = args.input.with_name(args.input.stem + "_phases.png")

    phases, samples = parse_log(args.input)
    stats = compute_phase_stats(phases, samples)
    plot_phase_analysis(phases, stats, args.output, show=not args.no_show)


if __name__ == "__main__":
    main()
