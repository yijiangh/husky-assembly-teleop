"""
The old offline pipeline's files (`data/calibration_data/0_…` to `4_…`), written from the plugin's results.

Same names, folders, figures and log lines as the old scripts; like them, each call overwrites its files.
`write_solve_outputs` writes what scripts 0 to 2 wrote, `write_validation_outputs` script 3, `write_punch_outputs`
script 4.

! Uses matplotlib's `Figure`, never pyplot: safe on a worker thread, and no figure is left open.
"""

from __future__ import annotations

import json
import re
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Sequence

import matplotlib
import mpl_toolkits
import numpy as np
from matplotlib import projections
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from matplotlib.figure import Figure
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose, compose, invert
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES

from ....robot_interface.frames import MOCAP_FRAME, fixed_transform
from .fit import Circle, Solution, arm_base_link, fit_circle, tool_offsets
from ..storage.session_files import BATCHES, PUNCH_BATCH, VALIDATION_BATCH, calibration_file, read_samples, read_take

#: The old scripts' figure resolution; the base mocap deviation plots used matplotlib's default.
DPI, DEFAULT_DPI = 150, 100
#: Sample axes at least this far from the mean are drawn as outliers in the orientation debug plot, degrees.
OUTLIER_ANGLE = 170


def _register_axes3d() -> None:
    """Register matplotlib's own 3D axes; ! ROS's path puts the system's older `mpl_toolkits` first, which fails."""
    if "3d" in projections.get_projection_names():
        return
    own = str(Path(matplotlib.__file__).parent.parent / "mpl_toolkits")
    mpl_toolkits.__path__ = [own, *(path for path in mpl_toolkits.__path__ if path != own)]
    for name in [name for name in sys.modules if name.startswith("mpl_toolkits.mplot3d")]:
        del sys.modules[name]
    from mpl_toolkits.mplot3d import Axes3D  # * only after the path fix above
    projections.register_projection(Axes3D)


_register_axes3d()


class _Log:
    """Log lines in the old scripts' format, `<time> - <LEVEL> - <message>`, written at the end."""

    def __init__(self) -> None:
        self.lines: list[str] = []

    def info(self, message: str) -> None:
        self._add("INFO", message)

    def warning(self, message: str) -> None:
        self._add("WARNING", message)

    def _add(self, level: str, message: str) -> None:
        now = datetime.now()
        self.lines.append(f"{now:%Y-%m-%d %H:%M:%S},{now.microsecond // 1000:03d} - {level} - {message}")

    def write(self, path: Path) -> Path:
        path.write_text("\n".join(self.lines) + "\n")
        return path


def traj_label(file_name: str, fallback: int | str = 0) -> str:
    """The old plots' take label, "J<joint>_T<n>", from an old (`J0_traj3`) or plugin (`J0_take3`) file name."""
    match = re.search(r"J(\d+)_(?:traj|take)(\d+)", file_name)
    if match:
        return f"J{match.group(1)}_T{match.group(2)}"
    return fallback if isinstance(fallback, str) else f"T{fallback}"


def old_arm(arm: str) -> str:
    """The old config's arm name, "left" or "right", for a plugin arm such as "ur_arm" or "right_ur_arm"."""
    return "right" if arm.startswith("right") else "left"


# --- --- --- --- --- SOLVE: SCRIPTS 0, 1 AND 2 --- --- --- --- ---


def write_solve_outputs(session: Path, digits: str, arm: str, urdf: Path, solution: Solution, files: Sequence[Path],
                        punch_offset: Sequence[float] | None = None) -> list[Path]:
    """Write what the old scripts 0 to 2 wrote for a solution, and the session's config.yaml.

    Args:
        session: The session folder.
        digits: The robot's serial digits, e.g. "0805".
        arm: The measured arm, e.g. "ur_arm".
        urdf: The robot's URDF.
        solution: The solution, in the base rigid body's frame.
        files: The take files it was solved from, in the order given to `fit.solve`: J0 takes, then J1 takes.
        punch_offset: tool0 to punch tip, metres, for config.yaml; left out if None.

    Returns:
        list[Path]: Every file written.

    Raises:
        ValueError: If the files do not match the solution's circles.
    """
    count = len(solution.j0_circles)
    if len(files) != count + len(solution.j1_circles):
        raise ValueError(f"{len(files)} take files for {count} J0 and {len(solution.j1_circles)} J1 circles")
    batches = {"j0": (list(files[:count]), solution.j0_circles), "j1": (list(files[count:]), solution.j1_circles)}
    written = []
    raw = {}
    for joint, (batch, (paths, circles)) in enumerate(batches.items()):
        raw[batch] = [json.loads(Path(path).read_text())["raw_data"] for path in paths]
        written += _circle_fitting(session / batch, batch, joint, paths, raw[batch], circles)
    written += _calibration_analysis(session, arm, urdf, solution, batches, raw)
    written.append(_compute_tf_log(session, digits, arm, urdf, solution))
    written.append(write_config(session, digits, arm, punch_offset))
    return written


def write_config(session: Path, digits: str, arm: str, punch_offset: Sequence[float] | None = None) -> Path:
    """Write the session's config.yaml as the old scripts read it (`config_loader.py`), for this robot and arm."""
    side = old_arm(arm)
    batches = "".join(f'  - "{batch}"\n' for batch in BATCHES)
    text = f"""# Calibration Pipeline Configuration, written by the calibration plugin
# This file is shared across all calibration scripts (0-4)

data_batches:
{batches}
validation_data_batch: "{VALIDATION_BATCH}"

robot_name: "{digits}"  # Options: "0806" Cindy (dual arm), "0804" Alice / "0805" Belle (single arm)
arm: "{side}"         # Options: "left", "right" (only used for dual arm robot)

export: true

use_gui: false

# Punch validation analysis settings for 4_punch_validation.py
punch_validation:
  arm: {side}
"""
    if punch_offset is not None:
        xyz = ", ".join(repr(float(v)) for v in punch_offset)
        text += ("\n# Translation-only offset from tool0 flange to punch tip (meters), from the pendant's TCP\n"
                 f"punch_tool:\n  left:\n    offset_xyz: [{xyz}]\n  right:\n    offset_xyz: [{xyz}]\n")
    path = session / "config.yaml"
    path.write_text(text)
    return path


def _projected(points: np.ndarray, circle: Circle) -> np.ndarray:
    """Each point moved onto its circle, as 0_circle_fitting.py drew them."""
    normal = circle.normal / np.linalg.norm(circle.normal)
    on_plane = points - np.outer((points - circle.center) @ normal, normal) - circle.center
    return circle.center + on_plane / np.linalg.norm(on_plane, axis=1, keepdims=True) * circle.radius


def _ring(center: np.ndarray, normal: np.ndarray, radius: float) -> np.ndarray:
    """(100, 3) points around a circle, in the old scripts' way."""
    v1 = np.cross(normal, [0, 0, 1]) if abs(normal[2]) < 0.9 else np.cross(normal, [1, 0, 0])
    v1 = v1 / np.linalg.norm(v1)
    v2 = np.cross(normal, v1)
    v2 = v2 / np.linalg.norm(v2)
    theta = np.linspace(0, 2 * np.pi, 100)
    return center + radius * (np.outer(np.cos(theta), v1) + np.outer(np.sin(theta), v2))


def _circle_fitting(folder: Path, batch: str, joint: int, paths: list[Path], raws: list[list[dict]],
                    circles: Sequence[Circle]) -> list[Path]:
    """0_circle_fitting.py for one batch: its log, `<batch>_analysis.json` and two figures."""
    folder.mkdir(parents=True, exist_ok=True)
    log = _Log()
    log.info("=" * 60)
    log.info(f"Processing data batch: {batch}")
    log.info("=" * 60)
    takes = []
    for path, circle in zip(paths, circles):
        take = read_take(Path(path), joint)[0]
        log.info(f"Processing: {Path(path).name}")
        start = time.perf_counter()
        fit_circle(take)
        log.info(f"  Circle fitting time: {(time.perf_counter() - start) * 1000:.3f} ms")
        log.info(f"  Circle fit quality - max point distance: {circle.errors.max() * 1000:.3f} mm")
        takes.append((Path(path).name, take.points, circle))
    analysis = folder / f"{batch}_analysis.json"
    analysis.write_text(json.dumps({"takes": [
        {"file_name": name, "raw_data": raw, "center": circle.center.tolist(), "normal": circle.normal.tolist()}
        for (name, _, circle), raw in zip(takes, raws)]}, indent=4))
    log.info(f"Results exported to: {analysis}")
    log.info(f"Total takes processed: {len(takes)}")
    colors = matplotlib.colormaps["tab10"](np.linspace(0, 1, len(takes)))

    figure = Figure(figsize=(14, 12))
    axis = figure.add_subplot(projection="3d")
    for index, ((name, points, circle), color) in enumerate(zip(takes, colors)):
        label = traj_label(name, index)
        projected = _projected(points, circle) * 1000
        millimetres = points * 1000
        axis.scatter(*millimetres.T, c=[color], s=30, alpha=0.7, label=f"{label} points")
        axis.scatter(*projected.T, c=[color], s=10, marker="x", alpha=0.5)
        for point, on_circle, distance in zip(millimetres, projected, circle.errors):
            axis.plot(*np.array([point, on_circle]).T, color=color, linestyle="--", alpha=0.4, linewidth=0.8)
            axis.text(*(point + on_circle) / 2, f"{distance * 1000:.2f}", fontsize=6, alpha=0.6, color=color)
        ring = _ring(circle.center, circle.normal, circle.radius) * 1000
        axis.plot(*ring.T, color=color, linewidth=2, label=f"{label} circle (r={circle.radius * 1000:.1f} mm)")
        axis.scatter(*[[v] for v in circle.center * 1000], c=[color], s=80, marker="*")
    axis.set_xlabel("X (mm)")
    axis.set_ylabel("Y (mm)")
    axis.set_zlabel("Z (mm)")
    axis.set_title(f"{batch} - All Circle Fits ({len(takes)} takes)")
    axis.legend(loc="upper left", fontsize=8)
    written = [_save(figure, folder / f"{batch}_all_circles.png")]

    figure = Figure(figsize=(12, 6))
    axis = figure.add_subplot()
    markers = ["o", "s", "^", "D", "v", "<", ">", "p", "h", "*"]
    offset = 0
    for index, ((name, _, circle), color) in enumerate(zip(takes, colors)):
        errors = circle.errors * 1000
        axis.scatter(np.arange(offset, offset + len(errors)), errors, c=[color], s=40, marker=markers[index % 10],
                     label=f"{traj_label(name, index)} (max: {errors.max():.2f} mm)", alpha=0.8)
        if index > 0:
            axis.axvline(x=offset - 0.5, color="gray", linestyle="--", alpha=0.5, linewidth=1)
        offset += len(errors)
    axis.set_xlabel("Sample index")
    axis.set_ylabel("Projection error (mm)")
    axis.set_title(f"{batch} - Projection Errors Across All Circles ({len(takes)} takes)")
    axis.legend(loc="upper right", fontsize=8)
    axis.grid(True, alpha=0.3)
    figure.tight_layout()
    written.append(_save(figure, folder / f"{batch}_projection_errors.png"))
    return [log.write(folder / f"circle_fitting_log_{batch}.txt"), analysis, *written]


def _calibration_analysis(session: Path, arm: str, urdf: Path, solution: Solution, batches: dict,
                          raw: dict) -> list[Path]:
    """1_calibration_analysis.py: its log and figures, from the solution's circles and axis lines."""
    log = _Log()
    log.info("=" * 80)
    log.info("Starting calibration analysis")
    log.info("=" * 80)
    joint = f"{arm}_{UR_JOINT_NAMES[0]}"
    offset = solution.shoulder_height
    log.info(f"Parsing base offset from URDF: {urdf}")
    log.info(f'Parsed base offset from URDF joint "{joint}": z={offset}')
    log.info(f"Base offset: {offset} m ({offset * 1000:.4f} mm)")
    log.info("")
    for batch in batches:
        log.info(f"Loading {batch} data from: {session / batch / f'{batch}_analysis.json'}")
    for batch, (paths, _) in batches.items():
        log.info(f"{batch} has {len(paths)} takes")
    written = []
    for batch, (paths, _) in batches.items():
        _section(log, f"Analyzing {batch} base mocap variation")
        written += _base_mocap_variation(log, session, batch, paths, raw[batch])
    for batch, (_, circles) in batches.items():
        _section(log, f"Analyzing {batch} circle centers")
        written.append(_circle_centers(log, session, batch, circles, getattr(solution, batch)))
    for batch, (_, circles) in batches.items():
        _section(log, f"Fitting line to {batch} circle centers")
        line = getattr(solution, batch)
        log.info(f"{batch} line fitting:")
        log.info(f"  Average circle normal: {line.direction}")
        log.info(f"  Angle differences from mean normal (deg): {[f'{a:.2f}' for a in line.normal_angles]}")
        log.info(f"  Max angle difference: {line.normal_angles.max():.2f} degrees")
        log.info(f"  Fitted line point: {line.point * 1000} mm")
        log.info(f"  Fitted line direction: {line.direction}")
        log.info(f"  Max distance from centers to fitted line: {line.center_distances.max() * 1000:.2f} mm")
        log.info(f"  Average distance from centers to fitted line: {line.center_distances.mean() * 1000:.2f} mm")
    _section(log, "Computing robot base frame")
    crossing = _crossing(solution)
    matrix = solution.arm_base.matrix()
    x, y, z = matrix[:3, 0], matrix[:3, 1], matrix[:3, 2]
    log.info(f"  Intersection direction: {crossing['direction']}")
    log.info(f"  Intersection point (closest to p1): {crossing['p01'] * 1000} mm")
    log.info(f"  Intersection anchor: {crossing['anchor'] * 1000} mm")
    log.info("  Frame construction: Z=+J0, Y=-J1, X=Y×Z, Z=(recomputed)")
    log.info(f"  Base origin p_b (after offset): {matrix[:3, 3] * 1000} mm")
    log.info("  Base frame axes:")
    log.info(f"    x-axis: {x}")
    log.info(f"    y-axis: {y}")
    log.info(f"    z-axis: {z}")
    log.info(f"  Orthogonality check: |X·Y|={abs(x @ y):.6f}, |X·Z|={abs(x @ z):.6f}, |Y·Z|={abs(y @ z):.6f}")
    _section(log, "Final base frame transformation:")
    log.info(f"Origin: {np.array(solution.arm_base.position) * 1000} mm")
    log.info(f"Quaternion: {np.array(solution.arm_base.orientation)}")
    log.info(f"\nTransformation matrix:\n{matrix}")
    log.info("\nVisualizing results...")
    written.append(_analysis_result(session, arm, solution, crossing))
    log.info(f"\nResults saved to: {session / 'base_frame_calibration.json'}")
    log.info("=" * 80)
    log.info("Calibration analysis complete!")
    log.info("=" * 80)
    return [log.write(session / "calibration_analysis_log.txt"), *written]


def _section(log: _Log, title: str) -> None:
    log.info("\n" + "=" * 80)
    log.info(title)
    log.info("=" * 80)


def _base_mocap_variation(log: _Log, session: Path, batch: str, paths: list[Path], raws: list[list[dict]]
                          ) -> list[Path]:
    """The base rigid body's position and axes over a batch's samples: two figures."""
    positions, quaternions, boundaries, labels = [], [], [], []
    for index, (path, entries) in enumerate(zip(paths, raws)):
        boundaries.append(len(positions))
        labels.append(traj_label(Path(path).name, index))
        for entry in entries:
            if entry.get("base_mocap_pose"):
                positions.append(entry["base_mocap_pose"][0])
                quaternions.append(entry["base_mocap_pose"][1])
    positions = np.array(positions)
    mean = positions.mean(axis=0)
    distances = np.linalg.norm(positions - mean, axis=1)
    log.info("Base mocap position statistics:")
    log.info(f"  Mean position: {mean * 1000} mm")
    log.info(f"  Max distance from mean: {distances.max() * 1000:.2f} mm")
    log.info(f"  Min distance from mean: {distances.min() * 1000:.2f} mm")
    log.info(f"  Average distance from mean: {distances.mean() * 1000:.2f} mm")
    deviations = _axis_angles(Rotation.from_quat(quaternions).as_matrix())
    log.info("Base mocap orientation statistics:")
    for name, angles in zip("xyz", deviations):
        log.info(f"  Max angle deviation for {name}-axis: {angles.max():.2f} degrees (equiv. to "
                 f"{np.tan(np.radians(angles.max())) * 1000:.1f} mm over 1 m)")

    def separators(axis) -> None:
        for boundary, label in zip(boundaries[1:], labels[1:]):
            axis.axvline(x=boundary, color="gray", linestyle="--", alpha=0.7, linewidth=1)
            axis.text(boundary, axis.get_ylim()[1] * 0.95, label, fontsize=8, ha="center", va="top", color="gray",
                      rotation=90)

    figure = Figure(figsize=(10, 6))
    axis = figure.add_subplot()
    indices = np.arange(len(distances))
    for column, style, name in ((0, "r-", "X-axis"), (1, "g-", "Y-axis"), (2, "b-", "Z-axis")):
        axis.plot(indices, np.abs(positions[:, column] - mean[column]) * 1000, style, label=name)
    axis.plot(indices, distances * 1000, "k--", label="Total distance")
    separators(axis)
    axis.set_xlabel("Sample index")
    axis.set_ylabel("Distance from mean (mm)")
    axis.set_title(f"{batch} - Base Mocap Position Deviations by Axis")
    axis.legend()
    written = [_save(figure, session / f"{batch}_base_mocap_pos_deviation.png", DEFAULT_DPI)]

    figure = Figure()
    axis = figure.add_subplot()
    for angles, style, name in zip(deviations, ("r-", "g-", "b-"), ("X-axis", "Y-axis", "Z-axis")):
        axis.plot(indices, angles, style, label=name)
    separators(axis)
    axis.set_xlabel("Sample index")
    axis.set_ylabel("Angle deviation (degrees)")
    axis.set_title(f"{batch} - Base Mocap Orientation Deviations")
    axis.legend()
    written.append(_save(figure, session / f"{batch}_base_mocap_axis_deviation.png", DEFAULT_DPI))
    return written


def _axis_angles(rotations: np.ndarray) -> list[np.ndarray]:
    """Per axis X, Y, Z: each rotation's axis angle to the mean of that axis, degrees."""
    angles = []
    for column in range(3):
        axes = rotations[:, :, column]
        mean = axes.mean(axis=0)
        mean = mean / np.linalg.norm(mean)
        angles.append(np.degrees(np.arccos(np.clip(axes @ mean, -1.0, 1.0))))
    return angles


def _circle_centers(log: _Log, session: Path, batch: str, circles: Sequence[Circle], line) -> Path:
    """A batch's circle centres around its axis line: log lines and a 3D figure."""
    centers = np.array([circle.center for circle in circles])
    pairs = [np.linalg.norm(a - b) for k, a in enumerate(centers) for b in centers[k + 1:]]
    log.info(f"{batch} circle centers analysis:")
    log.info(f"  Number of centers: {len(centers)}")
    log.info(f"  Min distance between centers: {min(pairs) * 1000:.2f} mm")
    log.info(f"  Max distance between centers: {max(pairs) * 1000:.2f} mm")
    log.info(f"  Average distance between centers: {np.mean(pairs) * 1000:.2f} mm")
    if min(pairs) < 0.01:
        log.warning("  WARNING: Some circle centers are very close (< 10mm). Line fitting may be unreliable.")
    projections_ = line.point + np.outer((centers - line.point) @ line.direction, line.direction)

    figure = Figure(figsize=(12, 10))
    axis = figure.add_subplot(projection="3d")
    axis.scatter(*(centers * 1000).T, c="g", s=50, label="Circle centers", alpha=0.7)
    length = max(pairs) * 1.5
    ends = line.point + np.outer(np.linspace(-length / 2, length / 2, 100), line.direction)
    axis.plot(*(ends * 1000).T, "b-", linewidth=2, label="Fitted line")
    axis.scatter(*(projections_ * 1000).T, c="r", s=30, marker="x", label="Projections on line", alpha=0.7)
    for center, foot, distance in zip(centers, projections_, line.center_distances):
        axis.plot(*(np.array([center, foot]) * 1000).T, "k--", alpha=0.3, linewidth=1)
        axis.text(*(center + foot) / 2 * 1000, f"{distance * 1000:.2f}mm", fontsize=8, alpha=0.7)
    axis.set_xlabel("X (mm)")
    axis.set_ylabel("Y (mm)")
    axis.set_zlabel("Z (mm)")
    axis.set_title(f"{batch} - Circle Centers with Fitted Line")
    axis.legend()
    return _save(figure, session / f"{batch}_circle_centers.png")


def _unit(vector: np.ndarray) -> np.ndarray:
    return vector / np.linalg.norm(vector)


def _crossing(solution: Solution) -> dict:
    """Where J1 crosses J0's station, as 1_calibration_analysis.py logged and drew it (`fit.arm_base_frame`)."""
    j0, j1 = solution.j0, solution.j1
    normal1 = _unit(np.cross(j0.direction, j1.direction))
    direction = _unit(np.cross(j1.direction, normal1))
    anchor = np.linalg.lstsq(np.array([j1.direction, normal1]),
                             np.array([j1.direction @ j0.point, normal1 @ j1.point]), rcond=None)[0]
    p01 = anchor + ((j1.point - anchor) @ direction) * direction
    return {"anchor": anchor, "direction": direction, "p01": p01, "plane0": (j0.point, j1.direction),
            "plane1": (j1.point, normal1)}


def _plane(axis, point: np.ndarray, normal: np.ndarray, color: str, label: str, size: float = 1.0) -> None:
    """A square of a plane, in millimetres, as the old `plot_plane`."""
    v1 = np.cross(normal, [0, 0, 1]) if abs(normal[2]) < 0.9 else np.cross(normal, [1, 0, 0])
    v1 = v1 / np.linalg.norm(v1)
    v2 = np.cross(normal, v1)
    v2 = v2 / np.linalg.norm(v2)
    u, v = np.meshgrid(np.linspace(-size, size, 20), np.linspace(-size, size, 20))
    grid = [(point[k] + u * v1[k] + v * v2[k]) * 1000 for k in range(3)]
    axis.plot_surface(*grid, color=color, alpha=0.3, edgecolor="none", label=label)


def _analysis_result(session: Path, arm: str, solution: Solution, crossing: dict) -> Path:
    """calibration_analysis_result.png: centres, axis lines, the two planes, their crossing and the arm base frame."""
    figure = Figure(figsize=(12, 10))
    axis = figure.add_subplot(projection="3d")
    for name, circles, line, color in (("j0", solution.j0_circles, solution.j0, "b"),
                                       ("j1", solution.j1_circles, solution.j1, "r")):
        centers = np.array([circle.center for circle in circles]) * 1000
        axis.scatter(*centers.T, c=color, s=50, alpha=0.6, label=f"{name} centers")
        points = line.point + np.outer(np.linspace(-0.3, 0.3, 100), line.direction)
        axis.plot(*(points * 1000).T, f"{color}-", linewidth=2, label=f"{name} fitted line")
    points = crossing["anchor"] + np.outer(np.linspace(-0.5, 0.5, 100), crossing["direction"])
    axis.plot(*(points * 1000).T, color="m", linewidth=2, linestyle="--", alpha=0.8, label="Intersection line (c01)")
    p01 = crossing["p01"] * 1000
    axis.scatter(*[[v] for v in p01], c="orange", s=150, marker="o", label="p01", edgecolors="darkorange",
                 linewidths=2)
    axis.text(*p01, "  p01", fontsize=12, fontweight="bold", color="darkorange")
    _plane(axis, *crossing["plane0"], color="cyan", label="Plane0 (p0, v1)")
    _plane(axis, *crossing["plane1"], color="yellow", label="Plane1 (p1, v0×v1)")
    matrix = solution.arm_base.matrix()
    origin = matrix[:3, 3] * 1000
    for column, color, name in ((0, "r", "X"), (1, "g", "Y"), (2, "b", "Z")):
        axis.quiver(*origin, *matrix[:3, column], length=200, color=color, arrow_length_ratio=0.3, linewidth=2,
                    label=f"Base {name}")
    axis.scatter(*[[v] for v in origin], c="k", s=100, marker="*", label="Base origin")
    for column, color, name in ((0, (1, 0.7, 0.7, 0.6), "X"), (1, (0.7, 1, 0.7, 0.6), "Y"),
                                (2, (0.6, 0.8, 1.0, 0.6), "Z")):
        axis.quiver(0, 0, 0, *np.eye(3)[column], length=100, color=color, arrow_length_ratio=0.3, linewidth=2,
                    label=f"Unit {name} (ref)")
    axis.set_xlabel("X (mm)")
    axis.set_ylabel("Y (mm)")
    axis.set_zlabel("Z (mm)")
    axis.set_title(f"Calibration Analysis - Fitted Lines and base_mocap_from_{arm_base_link(arm)}")
    axis.legend()
    return _save(figure, session / "calibration_analysis_result.png")


def _pose_text(pose: Pose) -> str:
    return f"({tuple(float(v) for v in pose.position)}, {tuple(float(v) for v in pose.orientation)})"


def _compute_tf_log(session: Path, digits: str, arm: str, urdf: Path, solution: Solution) -> Path:
    """compute_tf_log.txt of 2_convert_and_visualize_transformation.py."""
    link = arm_base_link(arm)
    footprint_in_arm = invert(fixed_transform(urdf, MOCAP_FRAME, link))
    urdf_arm = compose(solution.footprint, invert(footprint_in_arm))
    log = _Log()
    log.info("=" * 60)
    log.info("Computing calibrated transformation")
    log.info("=" * 60)
    log.info(f"Loaded calibration data from: {session / 'base_frame_calibration.json'}")
    log.info(f"base_mocap_from_{link}: {_pose_text(solution.arm_base)}")
    log.info(f"Loading robot from: {urdf}")
    log.info(f"arm_base_link_from_base_footprint (from URDF): {_pose_text(footprint_in_arm)}")
    log.info(f"base_mocap_from_base_footprint: {_pose_text(solution.footprint)}")
    log.info(f"URDF arm_base_link pose: {_pose_text(urdf_arm)}")
    log.info(f"CALIB arm_base_link pose: {_pose_text(solution.arm_base)}")
    difference = (np.array(urdf_arm.position) - np.array(solution.arm_base.position)) * 1000
    log.info("Position difference (URDF - CALIB): %.3f, %.3f, %.3f mm" % tuple(difference))
    log.info(f"Position difference magnitude: {np.linalg.norm(difference):.3f} mm")
    log.info(f"Calibrated transformation saved to: {calibration_file(session, digits)}")
    log.info("=" * 60)
    log.info("Done!")
    log.info("=" * 60)
    return log.write(session / "compute_tf_log.txt")


# --- --- --- --- --- VALIDATION: SCRIPT 3 --- --- --- --- ---


def write_validation_outputs(session: Path, digits: str, arm: str, urdf: Path, footprint: Pose,
                             files: Sequence[Path]) -> list[Path]:
    """Write what 3_verify_calibration.py wrote into `validation/`: its log and five figures.

    Args:
        session: The session folder.
        digits: The robot's serial digits.
        arm: The validated arm, e.g. "ur_arm".
        urdf: The robot's URDF.
        footprint: The calibration validated: base_footprint in the base rigid body.
        files: The validation take files.

    Returns:
        list[Path]: verification_log.txt, then the figures.

    Raises:
        ValueError: If the files hold no complete sample.
    """
    batch = VALIDATION_BATCH
    folder = session / batch
    folder.mkdir(parents=True, exist_ok=True)
    tool0 = f"{arm}_tool0"
    log = _Log()
    log.info("=" * 60)
    log.info("Calibration Verification")
    log.info("=" * 60)
    log.info(f"Validation data batch: {batch}")
    log.info(f"Robot: {digits}")
    log.info(f"Joint names: {[f'{arm}_{name}' for name in UR_JOINT_NAMES]}")
    log.info(f"Tool0 link: {tool0}")
    log.info(f"Calibration base_mocap_from_base_footprint: {_pose_text(footprint)}")
    results = []
    for path in files:
        log.info(f"Processing: {Path(path).name}")
        bases, tools, joints = read_samples(Path(path))
        for index, (base, tool, angles, offset) in enumerate(zip(bases, tools, joints, tool_offsets(
                bases, tools, joints, footprint, urdf, arm))):
            world_tool0 = compose(tool, invert(offset))
            position = np.array(offset.position)
            norm = np.linalg.norm(position) * 1000
            distance = np.linalg.norm(np.array(world_tool0.position) - np.array(tool.position)) * 1000
            log.info(f"  Sample {index}: tool0_from_flange_mocap pos={position[0]:.3f}, {position[1]:.3f}, "
                     f"{position[2]:.3f} m (norm={norm:.1f} mm) | world dist={distance:.1f} mm")
            results.append({"offset": offset, "joints": angles, "tool0": np.array(world_tool0.position),
                            "flange": np.array(tool.position), "base": base, "file_name": Path(path).name})
    log.info(f"Processed {len(results)} samples")
    if not results:
        raise ValueError(f"no complete validation sample in {folder}")
    written = [_tool0_vs_flange(log, folder, batch, results)]
    written += _verification(log, folder, batch, results, session.name, digits, old_arm(arm))
    log.info("=" * 60)
    log.info("Verification complete!")
    log.info("=" * 60)
    return [log.write(folder / "verification_log.txt"), *written]


def _region_names(file_names: list[str]) -> list[str]:
    return [traj_label(name, name.replace("calibration_", "").replace(".json", "")) for name in file_names]


def _tool0_vs_flange(log: _Log, folder: Path, batch: str, results: list[dict]) -> Path:
    """debug_tool0_vs_flange: tool0 by FK against the tool body, in 3D and per sample."""
    tool0 = np.array([r["tool0"] for r in results])
    flange = np.array([r["flange"] for r in results])
    names = [r["file_name"] for r in results]
    distances = np.linalg.norm(tool0 - flange, axis=1) * 1000
    low, high = distances.min(), distances.max()
    scaled = (distances - low) / (high - low) if high > low else np.zeros_like(distances)
    cmap = matplotlib.colormaps["coolwarm"]

    figure = Figure(figsize=(20, 9))
    view = figure.add_subplot(121, projection="3d")
    flat = figure.add_subplot(122)
    for t0, fl, value, distance in zip(tool0, flange, scaled, distances):
        view.plot(*np.array([t0, fl]).T, color=cmap(value), linewidth=1.5, alpha=0.7)
        view.scatter(*t0, color="blue", s=20, alpha=0.6)
        view.scatter(*fl, color="red", s=20, alpha=0.6)
        view.text(*(t0 + fl) / 2, f"{distance:.1f}", fontsize=6, alpha=0.8)
    mappable = ScalarMappable(cmap=cmap, norm=Normalize(vmin=low, vmax=high))
    mappable.set_array([])
    figure.colorbar(mappable, ax=view, shrink=0.6, pad=0.1, label="Distance (mm)")
    view.scatter([], [], [], color="blue", s=40, label="tool0 (FK)")
    view.scatter([], [], [], color="red", s=40, label="flange_mocap")
    view.legend(loc="upper left")
    everything = np.vstack([tool0, flange])
    center = everything.mean(axis=0)
    extent = np.max(np.abs(everything - center)) * 1.3
    view.set_xlim(center[0] - extent, center[0] + extent)
    view.set_ylim(center[1] - extent, center[1] + extent)
    view.set_zlim(center[2] - extent, center[2] + extent)
    view.set_xlabel("X (m)")
    view.set_ylabel("Y (m)")
    view.set_zlabel("Z (m)")
    view.set_title(f"tool0 (FK) vs flange_mocap\nDistance range: {low:.1f} - {high:.1f} mm")

    scatter = flat.scatter(np.arange(len(results)), distances, c=distances, cmap=cmap, vmin=low, vmax=high, s=25,
                           alpha=0.8)
    figure.colorbar(scatter, ax=flat, label="Distance (mm)")
    boundaries = [i for i in range(1, len(names)) if names[i] != names[i - 1]]
    for boundary in boundaries:
        flat.axvline(x=boundary - 0.5, color="gray", linestyle="--", linewidth=1, alpha=0.7)
    regions = _region_names(names)
    for start, end in zip([0, *boundaries], [*boundaries, len(results)]):
        flat.text((start + end - 1) / 2, high * 1.05, regions[start], ha="center", va="bottom", fontsize=7,
                  alpha=0.7)
    flat.set_xlabel("Sample Index")
    flat.set_ylabel("Distance (mm)")
    flat.set_title(f"Per-Sample Distance\nMean: {np.mean(distances):.1f} mm | Median: {np.median(distances):.1f} mm")
    flat.grid(True, alpha=0.3)
    figure.suptitle(f"Debug: tool0 vs flange_mocap | {batch} | {len(results)} samples", fontsize=13,
                    fontweight="bold")
    figure.tight_layout(rect=[0, 0, 1, 0.95])
    path = _save(figure, folder / f"debug_tool0_vs_flange_{batch}.png")
    log.info(f"Debug 3D plot saved to: {path}")
    return path


def _verification(log: _Log, folder: Path, batch: str, results: list[dict], date: str, digits: str,
                  arm: str) -> list[Path]:
    """The rest of 3_verify_calibration.py: its analysis lines and the four other figures."""
    positions = np.array([r["offset"].position for r in results])
    mean = positions.mean(axis=0)
    # * As the old script: each offset's distance from tool0, less the mean of those distances.
    norms = np.linalg.norm(positions, axis=1) * 1000
    norms = norms - norms.mean()
    log.info("=" * 60)
    log.info("Position Analysis (tool0_from_flange_mocap offset)")
    log.info("=" * 60)
    log.info("  Mean position: [%.4f, %.4f, %.4f] m" % tuple(mean))
    log.info(f"  Max distance from mean: {norms.max():.3f} mm")
    log.info(f"  Mean distance from mean: {norms.mean():.3f} mm")
    log.info(f"  Std distance from mean: {norms.std():.3f} mm")
    rotations = Rotation.from_quat([r["offset"].orientation for r in results]).as_matrix()
    angles = dict(zip("XYZ", _axis_angles(rotations)))
    log.info("=" * 60)
    log.info("Orientation Analysis")
    log.info("=" * 60)
    for name, values in angles.items():
        log.info(f"  {name}-axis max angle from mean: {values.max():.3f} deg")
    names = [r["file_name"] for r in results]
    unique = list(dict.fromkeys(names))
    regions = dict(zip(names, _region_names(names)))
    tab10 = matplotlib.colormaps["tab10"].colors
    file_colors = {name: tab10[k % 10] for k, name in enumerate(unique)}

    figure = Figure(figsize=(12, 10))
    axis = figure.add_subplot(111, projection="3d")
    colors, labels = ["r", "g", "b"], ["X", "Y", "Z"]
    for column in range(3):
        axis.quiver(0, 0, 0, *np.eye(3)[column], color=colors[column], linewidth=4, arrow_length_ratio=0.1, alpha=1.0,
                    label=f"Unit {labels[column]}")
    outliers, legend = 0, set()
    for index, rotation in enumerate(rotations):
        for column in range(3):
            vector, angle = rotation[:, column], angles[labels[column]][index]
            if angle >= OUTLIER_ANGLE:
                axis.quiver(0, 0, 0, *vector, color="magenta", linewidth=3, alpha=0.9, arrow_length_ratio=0.08)
                axis.text(*vector * 1.05, f"#{index} {labels[column]} {angle:.0f}°", fontsize=6, color="magenta",
                          fontweight="bold")
                outliers += 1
            else:
                axis.quiver(0, 0, 0, *vector, color=colors[column], linewidth=0.5, alpha=0.15,
                            arrow_length_ratio=0.05)
        if names[index] not in legend:
            axis.scatter([], [], [], color=file_colors[names[index]], s=30, label=regions[names[index]])
            legend.add(names[index])
    for column in range(3):
        tips = rotations[:, :, column]
        for name in unique:
            mask = np.array([n == name for n in names])
            axis.scatter(*tips[mask].T, color=colors[column], s=8, alpha=0.3, edgecolors=file_colors[name],
                         linewidths=0.5)
    for setter in (axis.set_xlim, axis.set_ylim, axis.set_zlim):
        setter([-1.2, 1.2])
    axis.set_xlabel("X")
    axis.set_ylabel("Y")
    axis.set_zlabel("Z")
    if outliers:
        axis.plot([], [], color="magenta", linewidth=3, label=f"Outlier (>={OUTLIER_ANGLE}°)")
    ranges = " ".join(f"{name}=[{values.min():.1f}°,{values.max():.1f}°]" for name, values in angles.items())
    axis.set_title(f"tool0_from_flange_mocap Rotation Axes\nThick = unit axes | Thin = data | Magenta = outliers "
                   f"(>={OUTLIER_ANGLE}°, n={outliers})\nAngle range: {ranges}")
    axis.legend(fontsize=7, loc="upper left")
    figure.tight_layout()
    written = [_save(figure, folder / f"debug_orientation_{batch}.png")]
    log.info(f"Debug orientation plot saved to: {written[-1]}")

    title = f"Date: {date} | Robot: {digits} | Arm: {arm} | Batch: {batch} | Samples: {len(results)}"
    samples = np.arange(len(results))
    figure = Figure(figsize=(14, 10))
    axes = figure.subplots(2, 2)
    figure.suptitle(f"Calibration Verification | {title}", fontsize=12, fontweight="bold")
    axis = axes[0, 0]
    axis.scatter(samples, norms, alpha=0.7, s=20)
    axis.set_xlabel("Sample Index")
    axis.set_ylabel("Position Norm (mm)")
    axis.set_title(f"tool0_from_flange_mocap Position Norm\nMean: {norms.mean():.3f} mm")
    axis.grid(True, alpha=0.3)
    axis = axes[0, 1]
    for name, color in zip(labels, colors):
        axis.plot(samples, angles[name], c=color, alpha=0.8, linewidth=2, label=f"{name}-axis")
    axis.set_xlabel("Sample Index")
    axis.set_ylabel("Angular Deviation from Mean (deg)")
    axis.set_title("Angular Deviation per Axis from Mean\nMean: "
                   + ", ".join(f"{name}={angles[name].mean():.3f}°" for name in labels))
    axis.legend()
    axis.grid(True, alpha=0.3)
    axis = axes[1, 0]
    axis.plot(np.sort(norms), np.arange(1, len(norms) + 1) / len(norms), "b-", linewidth=2)
    p95 = np.percentile(norms, 95)
    axis.axhline(y=0.95, color="r", linestyle="--", label="95% cutoff")
    axis.axvline(x=p95, color="r", linestyle="--")
    axis.plot(p95, 0.95, "ro", markersize=8)
    axis.annotate(f"{p95:.2f} mm", xy=(p95, 0.95), xytext=(-40, -30), textcoords="offset points", fontsize=10,
                  color="r", arrowprops={"arrowstyle": "->", "color": "r", "alpha": 0.7})
    axis.set_xlabel("Position Offset (mm)")
    axis.set_ylabel("CDF")
    axis.set_title(f"Position Offset CDF\n95th percentile: {p95:.3f} mm | Goal 95% under 5mm")
    axis.legend()
    axis.grid(True, alpha=0.3)
    axis = axes[1, 1]
    for index, (name, color) in enumerate(zip(labels, colors)):
        values = np.sort(angles[name])
        a95 = np.percentile(angles[name], 95)
        axis.plot(values, np.arange(1, len(values) + 1) / len(values), color=color, linewidth=2,
                  label=f"{name}-axis (95%: {a95:.3f}°)")
        axis.plot(a95, 0.95, "o", color=color, markersize=6)
        axis.annotate(f"{a95:.2f}°", xy=(a95, 0.95), xytext=(10, -15 * (index + 1)), textcoords="offset points",
                      fontsize=9, color=color, arrowprops={"arrowstyle": "->", "color": color, "alpha": 0.7})
    axis.axhline(y=0.95, color="gray", linestyle="--", alpha=0.7, label="95% cutoff")
    axis.set_xlabel("Angular Deviation (deg)")
    axis.set_ylabel("CDF")
    axis.set_title("Angular Deviation CDF per Axis | Goal 95% under 0.3°")
    axis.legend()
    axis.grid(True, alpha=0.3)
    figure.tight_layout(rect=[0, 0, 1, 0.96])
    written.append(_save(figure, folder / f"verification_{batch}.png"))
    log.info(f"Verification plot saved to: {written[-1]}")

    millimetres, mean_mm = positions * 1000, mean * 1000
    figure = Figure(figsize=(12, 9))
    axis = figure.add_subplot(111, projection="3d")
    for k, name in enumerate(unique):
        mask = np.array([n == name for n in names])
        color = tab10[k % 10]
        points = millimetres[mask]
        axis.scatter(*points.T, c=[color], s=20, alpha=0.6, label=regions[name])
        for point in points:
            axis.plot(*np.array([point, mean_mm]).T, color=color, linewidth=0.5, alpha=0.3)
    axis.scatter(*mean_mm, color="black", s=200, marker="*", zorder=5, label="mean")
    axis.set_xlabel("X (mm)")
    axis.set_ylabel("Y (mm)")
    axis.set_zlabel("Z (mm)")
    axis.set_title(f"tool0_from_flange_mocap positions (in tool0 frame)\nMean: [{mean_mm[0]:.1f}, {mean_mm[1]:.1f}, "
                   f"{mean_mm[2]:.1f}] mm | Max dev: {norms.max():.1f} mm | {len(results)} samples")
    axis.legend(fontsize=8, loc="upper left")
    figure.tight_layout()
    written.append(_save(figure, folder / f"debug_offset_positions_{batch}.png"))
    log.info(f"Debug offset positions plot saved to: {written[-1]}")

    joints = np.degrees(np.array([r["joints"] for r in results]))
    bases = np.array([r["base"].position for r in results]) * 1000
    yaws = np.array([_yaw(r["base"].orientation) for r in results])
    figure = Figure(figsize=(14, 5))
    axes = figure.subplots(1, 2)
    figure.suptitle(f"Data Diversity | {title}", fontsize=12, fontweight="bold")
    axis = axes[0]
    names_ = [f"J{k + 1}" for k in range(joints.shape[1])]
    boxes = axis.boxplot(joints, tick_labels=names_, patch_artist=True)
    for patch, color in zip(boxes["boxes"], matplotlib.colormaps["viridis"](np.linspace(0.2, 0.8, len(names_)))):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    axis.set_xlabel("Joint")
    axis.set_ylabel("Joint Angle (deg)")
    ranges = ", ".join(f"{joints[:, k].min():.1f}~{joints[:, k].max():.1f}" for k in range(joints.shape[1]))
    axis.set_title(f"Joint Configuration Diversity\nRanges (deg): {ranges}")
    axis.grid(True, alpha=0.3, axis="y")
    axis = axes[1]
    axis.scatter(bases[:, 0], bases[:, 1], c="blue", alpha=0.5, s=20, zorder=2)
    axis.quiver(bases[:, 0], bases[:, 1], np.cos(np.radians(yaws)) * 300, np.sin(np.radians(yaws)) * 300, angles="xy",
                scale_units="xy", scale=1, color="red", alpha=0.6, width=0.003, headwidth=3, headlength=4, zorder=3)
    axis.set_xlabel("Base X (mm)")
    axis.set_ylabel("Base Y (mm)")
    axis.set_title(f"Base Position & Yaw Diversity\nX range: {np.ptp(bases[:, 0]):.1f}mm, Y range: "
                   f"{np.ptp(bases[:, 1]):.1f}mm, Yaw range: {np.ptp(yaws):.1f}°")
    axis.set_xlim(-4000, 4000)
    axis.set_ylim(-2000, 2000)
    axis.set_aspect("equal")
    axis.grid(True, alpha=0.3)
    figure.tight_layout(rect=[0, 0, 1, 0.92])
    written.append(_save(figure, folder / f"diversity_{batch}.png"))
    log.info(f"Diversity plot saved to: {written[-1]}")
    return written


def _yaw(quaternion: Sequence[float]) -> float:
    """Yaw of a quaternion (x, y, z, w), degrees, as the old scripts computed it."""
    x, y, z, w = quaternion
    return float(np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))


# --- --- --- --- --- PUNCH VALIDATION: SCRIPT 4 --- --- --- --- ---


def write_punch_outputs(session: Path, arm_index: int | None = None) -> list[Path]:
    """Write 4_punch_validation.py's three figures for every punch take of one arm in `punch_validation/`.

    Args:
        session: The session folder.
        arm_index: The arm's index in the punch files (0 single or left, 1 right); None if only one arm has takes.

    Returns:
        list[Path]: The figures; none with fewer than two takes, as the old script.

    Raises:
        ValueError: If several arms have takes and none is chosen, or the takes mix tool offsets.
    """
    folder = session / PUNCH_BATCH
    takes = []
    for path in sorted(folder.glob("punch_validation*.json")):
        data = json.loads(path.read_text())
        for take in data.get("takes", []):
            take = dict(take)
            take.setdefault("arm_index", data.get("arm_index", -1))
            take.setdefault("arm_name", data.get("arm_name") or ("left", "right")[int(take["arm_index"])])
            takes.append(take)
    groups: dict[int, list[dict]] = {}
    for take in takes:
        groups.setdefault(int(take["arm_index"]), []).append(take)
    if arm_index is None and len(groups) > 1:
        raise ValueError(f"punch takes of several arms in {folder}: choose one")
    chosen = groups.get(arm_index if arm_index is not None else next(iter(groups), -1), [])
    chosen = sorted(chosen, key=lambda take: take.get("timestamp", ""))
    if len(chosen) < 2:
        return []
    offsets = [take["tool0_from_punch_tip"] for take in chosen if take.get("tool0_from_punch_tip")]
    if any(not (np.allclose(o["position"], offsets[0]["position"])
                and np.allclose(o["quaternion"], offsets[0]["quaternion"])) for o in offsets):
        raise ValueError("Found multiple tool0_from_punch_tip transforms in the same analysis group. "
                         "Analyze each arm and TCP separately.")
    names = {take["arm_name"] for take in chosen}
    label = next(iter(names)) if len(names) == 1 else ("left", "right")[int(chosen[0]["arm_index"])]
    return _punch_figures(folder, chosen, label)


def _punch_figures(folder: Path, takes: list[dict], label: str) -> list[Path]:
    count = len(takes)
    positions = np.array([take["world_from_punch_tip"]["position"] for take in takes])
    mean = positions.mean(axis=0)
    distances = np.linalg.norm(positions - mean, axis=1) * 1000
    pairwise = np.array([np.linalg.norm(a - b) * 1000 for k, a in enumerate(positions) for b in positions[k + 1:]])
    angles = dict(zip("XYZ", _axis_angles(Rotation.from_quat(
        [take["world_from_punch_tip"]["quaternion"] for take in takes]).as_matrix())))
    bases = np.array([take["base_pose"]["position"] for take in takes]) * 1000
    yaws = np.array([_yaw(take["base_pose"]["quaternion"]) for take in takes])
    suffix = f" | {label} arm"
    numbers = np.arange(1, count + 1)
    millimetres, mean_mm = positions * 1000, mean * 1000

    figure = Figure(figsize=(14, 10))
    axes = figure.subplots(2, 2)
    figure.suptitle(f"Punch Validation: Position Mismatch | {count} takes{suffix}", fontsize=13, fontweight="bold")
    axis = axes[0, 0]
    axis.bar(numbers, distances, color=matplotlib.colormaps["coolwarm"](distances / max(distances.max(), 1e-6)),
             edgecolor="black", linewidth=0.5)
    axis.axhline(y=distances.mean(), color="red", linestyle="--", linewidth=1, label=f"Mean: {distances.mean():.2f} mm")
    axis.set_xlabel("Take #")
    axis.set_ylabel("Distance from Mean (mm)")
    axis.set_title(f"Per-Take Position Error\nMax: {distances.max():.2f} mm | Std: {distances.std():.2f} mm")
    axis.legend()
    axis.grid(True, alpha=0.3, axis="y")
    axis = axes[0, 1]
    for column, name in enumerate("XYZ"):
        axis.scatter(numbers, millimetres[:, column], alpha=0.8, s=30, label=name)
        axis.axhline(y=mean_mm[column], linestyle=":", alpha=0.5)
    axis.set_xlabel("Take #")
    axis.set_ylabel("Position (mm)")
    axis.set_title(f"Punch Tip Position per Take\nMean: [{mean_mm[0]:.2f}, {mean_mm[1]:.2f}, {mean_mm[2]:.2f}] mm")
    axis.legend()
    axis.grid(True, alpha=0.3)
    axis = axes[1, 0]
    axis.plot(np.sort(distances), np.arange(1, count + 1) / count, "b-", linewidth=2)
    p95 = np.percentile(distances, 95)
    axis.axhline(y=0.95, color="r", linestyle="--", alpha=0.7, label="95%")
    axis.axvline(x=p95, color="r", linestyle="--", alpha=0.7)
    axis.plot(p95, 0.95, "ro", markersize=8)
    axis.annotate(f"{p95:.2f} mm", xy=(p95, 0.95), xytext=(10, -20), textcoords="offset points", fontsize=10,
                  color="r", arrowprops={"arrowstyle": "->", "color": "r", "alpha": 0.7})
    axis.legend()
    axis.set_xlabel("Distance from Mean (mm)")
    axis.set_ylabel("CDF")
    axis.set_title("Position Error CDF")
    axis.grid(True, alpha=0.3)
    axis = axes[1, 1]
    axis.hist(pairwise, bins=max(10, count), alpha=0.7, color="steelblue", edgecolor="black", linewidth=0.5)
    axis.axvline(x=pairwise.mean(), color="red", linestyle="--", label=f"Mean: {pairwise.mean():.2f} mm")
    axis.axvline(x=pairwise.max(), color="orange", linestyle="--", label=f"Max: {pairwise.max():.2f} mm")
    axis.legend()
    axis.set_xlabel("Pairwise Distance (mm)")
    axis.set_ylabel("Count")
    axis.set_title("Pairwise Distance Distribution")
    axis.grid(True, alpha=0.3, axis="y")
    figure.tight_layout(rect=[0, 0, 1, 0.95])
    written = [_save(figure, folder / f"punch_validation_position_{label}.png")]

    figure = Figure(figsize=(10, 8))
    axis = figure.add_subplot(111, projection="3d")
    colors = matplotlib.colormaps["viridis"](np.linspace(0.2, 0.9, count))
    for k in range(count):
        axis.scatter(*millimetres[k], color=colors[k], s=60, alpha=0.8, edgecolors="black", linewidths=0.5)
        axis.text(*millimetres[k], f" {k + 1}", fontsize=7, alpha=0.8)
        axis.plot(*np.array([millimetres[k], mean_mm]).T, color=colors[k], linewidth=0.8, alpha=0.4)
    axis.scatter(*mean_mm, color="red", s=200, marker="*", zorder=5, label="Mean")
    axis.set_xlabel("X (mm)")
    axis.set_ylabel("Y (mm)")
    axis.set_zlabel("Z (mm)")
    axis.set_title(f"Punch Tip Positions in World Frame\nSpread: {distances.max():.2f} mm max | "
                   f"{distances.std():.2f} mm std | {count} takes{suffix}")
    axis.legend()
    figure.tight_layout()
    written.append(_save(figure, folder / f"punch_validation_3d_{label}.png"))

    figure = Figure(figsize=(14, 5))
    axes = figure.subplots(1, 2)
    figure.suptitle(f"Punch Validation: Orientation & Base Diversity | {count} takes{suffix}", fontsize=13,
                    fontweight="bold")
    axis = axes[0]
    for name, color in zip("XYZ", "rgb"):
        axis.plot(numbers, angles[name], "-o", color=color, alpha=0.8, markersize=5,
                  label=f"{name}-axis (max: {angles[name].max():.3f} deg)")
    axis.set_xlabel("Take #")
    axis.set_ylabel("Angular Deviation from Mean (deg)")
    axis.set_title("Orientation Mismatch per Axis")
    axis.legend()
    axis.grid(True, alpha=0.3)
    axis = axes[1]
    axis.scatter(bases[:, 0], bases[:, 1], c=numbers, cmap="viridis", s=40, zorder=2)
    scale = max(np.ptp(bases[:, 0]), np.ptp(bases[:, 1]), 200) * 0.08
    axis.quiver(bases[:, 0], bases[:, 1], np.cos(np.radians(yaws)) * scale, np.sin(np.radians(yaws)) * scale,
                angles="xy", scale_units="xy", scale=1, color="red", alpha=0.6, width=0.004, headwidth=3,
                headlength=4, zorder=3)
    for k in range(count):
        axis.annotate(f"{k + 1}", (bases[k, 0], bases[k, 1]), textcoords="offset points", xytext=(5, 5), fontsize=7,
                      alpha=0.8)
    axis.set_xlabel("Base X (mm)")
    axis.set_ylabel("Base Y (mm)")
    axis.set_title(f"Base Position & Yaw Diversity\nX range: {np.ptp(bases[:, 0]):.1f} mm | Y range: "
                   f"{np.ptp(bases[:, 1]):.1f} mm | Yaw range: {np.ptp(yaws):.1f} deg")
    axis.set_aspect("equal")
    axis.grid(True, alpha=0.3)
    figure.tight_layout(rect=[0, 0, 1, 0.92])
    written.append(_save(figure, folder / f"punch_validation_diversity_{label}.png"))
    return written


def _save(figure: Figure, path: Path, dpi: int = DPI) -> Path:
    figure.savefig(path, dpi=dpi)
    return path
