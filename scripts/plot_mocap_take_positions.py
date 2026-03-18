#!/usr/bin/env python3
import argparse
import json
import math
import os
import re
import tempfile
from pathlib import Path


if "MPLCONFIGDIR" not in os.environ:
    os.environ["MPLCONFIGDIR"] = os.path.join(tempfile.gettempdir(), "matplotlib")

import matplotlib.pyplot as plt


DEFAULT_TAKE_PATHS = [
    # "/home/yijiangh/ros2_ws/src/husky-assembly-teleop/data/mocap_experiments/20260318/20260318_test_cover/takes/20260318_133937_ws_back_markers5_allcam_cover_take4--back--markers_5--cam_21--cover--take1.json",
    # "/home/yijiangh/ros2_ws/src/husky-assembly-teleop/data/mocap_experiments/20260318/20260318_test_cover/takes/20260318_134053_ws_back_markers5_allcam_cover_take4--back--markers_5--cam_21--cover--take1.json",
    "/home/yijiangh/ros2_ws/src/husky-assembly-teleop/data/mocap_experiments/20260318/20260318_test_cover/takes/20260318_141052_ws_back_markers5_allcam_cover_take4--back--markers_5--cam_21--cover--take1.json"
]


def _slugify(value):
    return re.sub(r"[^A-Za-z0-9._-]+", "_", value).strip("_") or "plot"


def resolve_input_file(path_str):
    path = Path(path_str).expanduser()
    if not path.is_absolute():
        path = (Path.cwd() / path).resolve()
    else:
        path = path.resolve()

    if path.is_dir():
        raise ValueError(
            f"Expected a JSON file but got a directory: {path}\n"
            "Pass the full take JSON path, not the parent takes directory."
        )
    if not path.is_file():
        raise ValueError(f"File does not exist: {path}")
    return path


def load_series(path, rigid_body_name=None):
    path = resolve_input_file(path)

    with open(path, "r") as f:
        payload = json.load(f)

    target_rigid_body = rigid_body_name or payload.get("target_rigid_body")
    if not target_rigid_body:
        raise ValueError(f"{path}: no rigid body requested and no target_rigid_body in payload")

    times = []
    xs = []
    ys = []
    zs = []
    marker_errors = []
    tracking_valids = []

    for frame in payload.get("frames", []):
        rigid_body = frame.get("rigid_bodies", {}).get(target_rigid_body)
        if rigid_body is None:
            continue
        position = rigid_body.get("position_m", [])
        if len(position) != 3:
            continue
        times.append(float(frame.get("elapsed_sec", 0.0)))
        xs.append(float(position[0]))
        ys.append(float(position[1]))
        zs.append(float(position[2]))

        marker_error = rigid_body.get("marker_error")
        marker_errors.append(float(marker_error) if marker_error is not None else math.nan)

        tracking_valid = rigid_body.get("tracking_valid")
        if tracking_valid is None:
            tracking_valids.append(math.nan)
        else:
            tracking_valids.append(1.0 if bool(tracking_valid) else 0.0)

    if not times:
        available = sorted(payload.get("available_rigid_bodies", []))
        raise ValueError(
            f"{path}: no samples found for rigid body '{target_rigid_body}'. "
            f"Available rigid bodies: {available}"
        )

    return {
        "path": str(path),
        "label": path.stem,
        "rigid_body": target_rigid_body,
        "times": times,
        "x": xs,
        "y": ys,
        "z": zs,
        "marker_error": marker_errors,
        "tracking_valid": tracking_valids,
        "frame_count": len(times),
    }


def _is_finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def _finite_values(values):
    return [value for value in values if _is_finite(value)]


def _tracking_stats(values):
    valid_values = [value for value in values if _is_finite(value)]
    if not valid_values:
        return None
    return {
        "valid_count": sum(1 for value in valid_values if value >= 0.5),
        "sample_count": len(valid_values),
    }


def _resolve_output_paths(series_list, output_arg):
    if output_arg:
        output_path = Path(output_arg).expanduser()
        if not output_path.is_absolute():
            output_path = (Path.cwd() / output_path).resolve()
        else:
            output_path = output_path.resolve()

        if len(series_list) == 1 and output_path.suffix:
            return [output_path]

        output_dir = output_path
    else:
        output_dir = Path.cwd()

    output_dir.mkdir(parents=True, exist_ok=True)
    return [output_dir / f"{_slugify(series['label'])}__xyz_quality.png" for series in series_list]


def _plot_single_take(series, output_path=None, show_plot=False):
    fig, axes = plt.subplots(5, 1, sharex=True, figsize=(14, 12))
    coord_specs = (
        ("x", "X [m]", "#1f77b4"),
        ("y", "Y [m]", "#ff7f0e"),
        ("z", "Z [m]", "#2ca02c"),
    )

    for ax, (coord_name, axis_label, color) in zip(axes[:3], coord_specs):
        ax.plot(series["times"], series[coord_name], linewidth=1.5, color=color)
        ax.set_ylabel(axis_label)
        ax.grid(True, alpha=0.3)

    marker_error_ax = axes[3]
    marker_error_values = _finite_values(series["marker_error"])
    if marker_error_values:
        marker_error_ax.plot(series["times"], series["marker_error"], linewidth=1.3, color="#d62728")
    else:
        marker_error_ax.text(
            0.5,
            0.5,
            "No marker_error samples in this take",
            ha="center",
            va="center",
            transform=marker_error_ax.transAxes,
        )
    marker_error_ax.set_ylabel("Marker Error")
    marker_error_ax.grid(True, alpha=0.3)

    tracking_ax = axes[4]
    tracking_stats = _tracking_stats(series["tracking_valid"])
    if tracking_stats:
        tracking_ax.step(series["times"], series["tracking_valid"], where="post", linewidth=1.2, color="#9467bd")
    else:
        tracking_ax.text(
            0.5,
            0.5,
            "No tracking_valid samples in this take",
            ha="center",
            va="center",
            transform=tracking_ax.transAxes,
        )
    tracking_ax.set_ylabel("Tracking")
    tracking_ax.set_xlabel("Elapsed time [s]")
    tracking_ax.set_yticks([0.0, 1.0])
    tracking_ax.set_yticklabels(["invalid", "valid"])
    tracking_ax.set_ylim(-0.2, 1.2)
    tracking_ax.grid(True, alpha=0.3)

    fig.suptitle(f"MoCap traces for rigid body '{series['rigid_body']}'\n{series['label']}")
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.97))

    if output_path is not None:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=150, bbox_inches="tight")

    if show_plot:
        plt.show()

    plt.close(fig)


def build_parser():
    parser = argparse.ArgumentParser(
        description="Plot per-take X/Y/Z position, marker error, and tracking validity from raw MoCap take JSON files."
    )
    parser.add_argument(
        "take_paths",
        nargs="*",
        help="Path(s) to take JSON files. If omitted, the baked-in DEFAULT_TAKE_PATHS are used.",
    )
    parser.add_argument(
        "--rigid-body",
        help="Rigid body name to plot. Defaults to each file's target_rigid_body.",
    )
    parser.add_argument(
        "--output",
        help="Optional output image path for a single take, or output directory for multiple takes.",
    )
    return parser


def main():
    args = build_parser().parse_args()
    take_paths = args.take_paths or DEFAULT_TAKE_PATHS
    series_list = [load_series(path, args.rigid_body) for path in take_paths]

    for series in series_list:
        print(f"{series['label']}:")
        print(f"  path: {series['path']}")
        print(f"  rigid_body: {series['rigid_body']}")
        print(f"  plotted_frames: {series['frame_count']}")
        print(
            "  start_xyz_m: "
            f"({series['x'][0]:.6f}, {series['y'][0]:.6f}, {series['z'][0]:.6f})"
        )
        print(
            "  end_xyz_m: "
            f"({series['x'][-1]:.6f}, {series['y'][-1]:.6f}, {series['z'][-1]:.6f})"
        )
        marker_error_values = _finite_values(series["marker_error"])
        if marker_error_values:
            print(
                "  marker_error: "
                f"min={min(marker_error_values):.6f}, "
                f"max={max(marker_error_values):.6f}, "
                f"mean={sum(marker_error_values)/len(marker_error_values):.6f}"
            )
        else:
            print("  marker_error: no samples")

        tracking_stats = _tracking_stats(series["tracking_valid"])
        if tracking_stats:
            print(
                "  tracking_valid: "
                f"{tracking_stats['valid_count']}/{tracking_stats['sample_count']} valid "
                f"({100.0 * tracking_stats['valid_count'] / tracking_stats['sample_count']:.2f}%)"
            )
        else:
            print("  tracking_valid: no samples")

    show_plot = bool(os.environ.get("DISPLAY")) and not args.output and len(series_list) == 1
    output_paths = _resolve_output_paths(series_list, None if show_plot else args.output)

    for series, output_path in zip(series_list, output_paths):
        _plot_single_take(series, None if show_plot else output_path, show_plot=show_plot)
        if not show_plot:
            print(f"Saved plot to {output_path}")


if __name__ == "__main__":
    main()
