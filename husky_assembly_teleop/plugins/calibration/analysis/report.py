"""
The plugin's own analysis: the headline numbers for the log and the Results tab, and `analysis/<digits>_<arm>/`.

The old pipeline's figures and files are `old_outputs.py`'s; the analysis folder is where new math can go.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Sequence

import numpy as np

from scaffolding_assembly_core.geometry import Pose

from .fit import Solution, arm_base_link
from ..storage.session_files import analysis_folder, pose_list
from .checks import VALIDATION_ANGLE, VALIDATION_DISTANCE, pose_change


def summary_lines(solution: Solution, applied: Pose | None, solved: Pose, figures: Sequence[Path] = (),
                  validation: tuple[np.ndarray, np.ndarray] | None = None, mount: Pose | None = None) -> list[str]:
    """A few plain lines for the log: the headline numbers, and where the figures are.

    Args:
        solution: The solution.
        applied: The calibration the relay applies now (what Apply changes), or None if it applies none.
        solved: The solved calibration.
        figures: The figure files written.
        validation: The validation's distances (metres) and axis angles (degrees), if it ran.
        mount: The arm's mount correction, for an arm that does not calibrate the base.
    """
    if applied is None:
        lines = ["change vs applied: the relay applies no calibration to compare with"]
    else:
        metres, degrees = pose_change(applied, solved)
        lines = [f"change vs applied: {metres * 1e3:.2f} mm, {degrees:.3f}° from the calibration the relay applies "
                 f"now"]
    for name, circles, axis in (("J0", solution.j0_circles, solution.j0), ("J1", solution.j1_circles, solution.j1)):
        worst = max(float(circle.errors.max()) for circle in circles)
        lines.append(f"{name}: {len(circles)} takes, points within {worst * 1e3:.2f} mm of their circles, normals "
                     f"within {axis.normal_angles.max():.3f}°, centres within "
                     f"{axis.center_distances.max() * 1e3:.2f} mm of the axis")
    lines.append(f"axes {solution.axes_angle:.3f}° apart, {solution.skew * 1e3:.2f} mm skew")
    if mount is not None:
        metres, degrees = pose_change(Pose(), mount)
        lines.append(f"mount {metres * 1e3:.2f} mm, {degrees:.3f}° from the URDF's")
    if validation is not None:
        distances, angles = validation
        lines.append(f"validation: 95 % within {np.percentile(distances, 95) * 1e3:.2f} mm "
                     f"(limit {VALIDATION_DISTANCE * 1e3:g} mm), axes within {np.max(angles):.3f}° "
                     f"(target {VALIDATION_ANGLE:g}°)")
    if figures:
        folders = sorted({Path(figure).parent for figure in figures})
        lines.append(f"{len(figures)} figures in " + ", ".join(str(folder) for folder in folders))
    return lines


def write_analysis(session: Path, digits: str, arm: str, solution: Solution, files: Sequence[Path],
                   applied: Pose | None,
                   validation: tuple[np.ndarray, np.ndarray] | None = None, mount: Pose | None = None) -> Path:
    """Write the plugin's own analysis of a solution: `solution.json` (its numbers) and `summary.txt`.

    Args:
        session: The session folder.
        digits: The robot's four serial digits.
        arm: The measured arm.
        solution: The solution.
        files: The take files solved, in the solution's circle order (J0 first).
        applied: The calibration the relay applies now, or None if it applies none.
        validation: The validation's distances (metres) and axis angles (degrees), if it ran.
        mount: The arm's mount correction, for an arm that does not calibrate the base.

    Returns:
        Path: The analysis folder.
    """
    folder = analysis_folder(session, digits, arm)
    circles = solution.j0_circles + solution.j1_circles
    data = {
        "robot": digits, "arm": arm, "applied": pose_list(applied) if applied else None,
        "base_mocap_from_base_footprint": pose_list(solution.footprint),
        f"base_mocap_from_{arm_base_link(arm)}": pose_list(solution.arm_base),
        "axes": {name: {"point": axis.point.tolist(), "direction": axis.direction.tolist()}
                 for name, axis in (("j0", solution.j0), ("j1", solution.j1))},
        "axes_angle_deg": solution.axes_angle, "skew_m": solution.skew, "shoulder_height_m": solution.shoulder_height,
        "takes": [{"file": str(Path(file).relative_to(session)), "joint": 0 if k < len(solution.j0_circles) else 1,
                   "center": circle.center.tolist(), "normal": circle.normal.tolist(), "radius_m": circle.radius,
                   "max_error_m": float(circle.errors.max()), "turn": circle.turn}
                  for k, (file, circle) in enumerate(zip(files, circles))],
    }
    if validation is not None:
        data["validation"] = {"distances_m": np.asarray(validation[0]).tolist(),
                              "angles_deg": np.asarray(validation[1]).tolist()}
    if mount is not None:
        data["mount_correction"] = pose_list(mount)
    (folder / "solution.json").write_text(json.dumps(data, indent=4))
    lines = summary_lines(solution, applied, solution.footprint, validation=validation, mount=mount)
    (folder / "summary.txt").write_text("\n".join(lines) + "\n")
    return folder
