"""
Solve a base calibration session again with the calibration plugin's math, and print how good it is.

Reads the session's j0/ and j1/ take files (filed by the joint they turn), prints each circle, both axis lines and
the result, compares it with the session's own calibration file if there is one, and checks it on the session's
validation/ samples. Writes nothing unless --write is given; then it writes the session's outputs as the old
pipeline did (base_frame_calibration.json, calibrated_transformation_<digits>_rhino.json), replacing them.

- `--old-scripts`: also runs the old pipeline's scripts 0 to 4 (data/calibration_data/) on a copy of the session's
  takes, validation, punch takes and config.yaml, and compares their result and circles with the plugin's.
- `--truth-error dx,dy,dz,droll,dpitch,dyaw`: a simulated session's calibration error (the base's `offsets` entry given
  to sim_mocap.launch.py); both results are compared with the true calibration it gives.

Run with the venv and ROS sourced (it imports crl_husky):

    python3 src/husky-assembly-teleop/scripts/solve_calibration.py 20261013 --old-scripts
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import yaml

from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.drive import drive_root
from husky_assembly_teleop.world.calibration_sessions import SESSIONS_FOLDER, default_calibration
from husky_assembly_teleop.plugins.calibration.analysis.fit import Solution, Take, offset_spread, solve, tool_offsets
from husky_assembly_teleop.plugins.calibration.analysis.old_outputs import write_config
from husky_assembly_teleop.plugins.calibration.storage.session_files import (BATCHES, PUNCH_BATCH, VALIDATION_BATCH,
                                                                             calibration_file, pose_from, read_takes,
                                                                             read_validation, write_solution)
from husky_assembly_teleop.plugins.calibration.analysis.checks import pose_change
from scaffolding_assembly_core.geometry import Pose, compose

DATA = Path(__file__).resolve().parents[1] / "data"
#: The old pipeline's scripts, in the order they run; 3 needs validation takes, 4 punch takes.
OLD_SCRIPTS = DATA / "calibration_data"
OLD_STEPS = ("0_circle_fitting.py", "1_calibration_analysis.py", "2_convert_and_visualize_transformation.py",
             "3_verify_calibration.py", "4_punch_validation.py")
#: Runs one old script as `python <script>` would, but with matplotlib's own 3D axes first on the path: ROS's path
#: puts the system's older `mpl_toolkits` first, which fails to import.
_RUNNER = """
import runpy, sys
from pathlib import Path
import matplotlib, mpl_toolkits
own = str(Path(matplotlib.__file__).parent.parent / "mpl_toolkits")
mpl_toolkits.__path__ = [own] + [path for path in mpl_toolkits.__path__ if path != own]
sys.argv = sys.argv[1:]
sys.path.insert(0, str(Path(sys.argv[0]).parent))
runpy.run_path(sys.argv[0], run_name="__main__")
"""


def run_old_scripts(session: Path, takes: list[Take], digits: str, arm: str, folder: Path) -> Path:
    """Copy a session's inputs into `folder` and run the old scripts there; the session itself is never written.

    Args:
        session: The session folder.
        takes: The takes the plugin solved (`read_takes`): each is copied into the folder of the joint it turns, so
            the old scripts fit the same takes, also from a session where a take sits in the wrong folder.
        digits: The robot's four serial digits, for a session without config.yaml.
        arm: The measured arm, for a session without config.yaml.
        folder: Where the copy goes, as `<folder>/<session name>/`.

    Returns:
        Path: The copy, holding the old scripts' outputs.

    Raises:
        RuntimeError: If a script fails; the message ends with its output.
    """
    copy = folder / session.name
    for batch in (*BATCHES, VALIDATION_BATCH):
        (copy / batch).mkdir(parents=True, exist_ok=True)
    for take in takes:
        shutil.copy2(session / take.name, copy / BATCHES[take.joint])
    for path in (session / VALIDATION_BATCH).glob("calibration_*.json"):
        shutil.copy2(path, copy / VALIDATION_BATCH)
    punch = sorted((session / PUNCH_BATCH).glob("punch_validation*.json"))
    if punch:
        (copy / PUNCH_BATCH).mkdir(exist_ok=True)
        for path in punch:
            shutil.copy2(path, copy / PUNCH_BATCH)
    if (session / "config.yaml").is_file():
        shutil.copy2(session / "config.yaml", copy)
    else:
        write_config(copy, digits, arm)
    steps = list(OLD_STEPS[:3])
    steps += [OLD_STEPS[3]] if any((copy / VALIDATION_BATCH).iterdir()) else []
    steps += [OLD_STEPS[4]] if punch else []
    environment = {**os.environ, "CALIBRATION_DATA_DIR": str(folder), "CALIBRATION_DATE": session.name,
                   "MPLBACKEND": "Agg"}
    for step in steps:
        result = subprocess.run([sys.executable, "-c", _RUNNER, str(OLD_SCRIPTS / step)], cwd=OLD_SCRIPTS,
                                env=environment, capture_output=True, text=True)
        (copy / f"{Path(step).stem}.log").write_text(result.stdout + result.stderr)
        if result.returncode != 0:
            raise RuntimeError(f"{step} failed:\n{(result.stdout + result.stderr)[-2000:]}")
    return copy


def old_result(copy: Path, digits: str) -> tuple[Pose, dict[str, tuple[np.ndarray, np.ndarray]]]:
    """The old scripts' result in a session: base_mocap_from_base_footprint, and each take's circle by file name."""
    output = json.loads(calibration_file(copy, digits).read_text())
    circles = {}
    for batch in BATCHES:
        for take in json.loads((copy / batch / f"{batch}_analysis.json").read_text())["takes"]:
            circles[take["file_name"]] = (np.array(take["center"]), np.array(take["normal"]))
    return pose_from(output["base_mocap_from_base_footprint"]), circles


def circle_differences(takes: list[Take], solution: Solution,
                       circles: dict[str, tuple[np.ndarray, np.ndarray]]) -> tuple[float, float]:
    """The largest gaps between the plugin's and the old scripts' circles, over the takes both fitted.

    Returns:
        tuple: The largest distance between centres (metres), and angle between normals (degrees, either sign).
    """
    distances, angles = [0.0], [0.0]
    for take, circle in zip(takes, solution.j0_circles + solution.j1_circles):
        old = circles.get(Path(take.name).name)
        if old is not None:
            distances.append(float(np.linalg.norm(circle.center - old[0])))
            cosine = abs(float(np.dot(circle.normal, old[1]) / np.linalg.norm(old[1])))
            angles.append(float(np.degrees(np.arccos(min(cosine, 1.0)))))
    return max(distances), max(angles)


def true_calibration(mocap_id: int, error: list[float]) -> Pose:
    """A simulated robot's true calibration: crl-husky's default moved by the error, as sim_motive makes it."""
    from husky_assembly_teleop.mocap_relay.sim_motive import offset_pose  # * a ROS node's module: only when asked
    default = default_calibration(mocap_id)
    if default is None:
        raise ValueError(f"crl-husky has no default calibration for rigid body {mocap_id}")
    return compose(Pose(default.position, default.orientation), offset_pose(error))


def _against(label: str, reference: Pose, footprint: Pose) -> str:
    metres, degrees = pose_change(reference, footprint)
    return f"vs {label}: {metres * 1e3:.3f} mm, {degrees:.4f}°"


def main() -> None:
    """Parse the arguments, solve the session and print the report."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("session", help="The session folder: a YYYYMMDD name under the Drive's calibration data, "
                                        "or a path")
    parser.add_argument("--robot", help="Four serial digits, e.g. 0805 (default: the session's config.yaml)")
    parser.add_argument("--arm", help="The measured arm, e.g. left_ur_arm (default: the session's config.yaml)")
    parser.add_argument("--write", action="store_true", help="Write the session's outputs, replacing them")
    parser.add_argument("--old-scripts", action="store_true",
                        help="Also run the old scripts 0 to 4 on a copy of the session and compare")
    parser.add_argument("--truth-error", help="A simulated session's calibration error, dx,dy,dz,droll,dpitch,dyaw "
                                              "(metres, degrees), to compare with the true calibration")
    args = parser.parse_args()

    session = Path(args.session)
    if not session.is_dir():
        root = drive_root()
        if root is None:
            parser.error("no Drive root: set HUSKY_DRIVE_ROOT, or give the session's path")
        session = root / SESSIONS_FOLDER / args.session
    config_file = session / "config.yaml"
    config = yaml.safe_load(config_file.read_text()) if config_file.is_file() else {}
    digits = args.robot or str(config.get("robot_name", ""))
    if not digits:
        parser.error("no --robot, and the session has no config.yaml naming it")
    robot = robot_config_from_serial(digits, DATA)
    arm = args.arm or (f"{config.get('arm', 'left')}_ur_arm" if len(robot.arms) > 1 else robot.arms[0].name)

    takes, skipped = read_takes(session)
    for line in skipped:
        print(f"skipped {line}")
    solution = solve([t for t in takes if t.joint == 0], [t for t in takes if t.joint == 1], robot.model.urdf, arm)

    print(f"\n{session.name}: robot {digits}, arm {arm}")
    print(f"{'take':<62}{'points':>7}{'radius mm':>11}{'max err mm':>12}{'turn':>7}")
    for take, circle in zip(takes, solution.j0_circles + solution.j1_circles):
        print(f"J{take.joint} {take.name:<59}{len(take.points):>7}{circle.radius * 1e3:>11.1f}"
              f"{circle.errors.max() * 1e3:>12.2f}{circle.turn:>7.2f}")
    for name, axis in (("J0", solution.j0), ("J1", solution.j1)):
        print(f"{name} axis: normals within {axis.normal_angles.max():.3f}°, centres within "
              f"{axis.center_distances.max() * 1e3:.2f} mm of the line")
    print(f"axes: {solution.axes_angle:.3f}° apart, {solution.skew * 1e3:.2f} mm skew; "
          f"shoulder height {solution.shoulder_height * 1e3:.2f} mm")
    footprint = solution.footprint
    print(f"base_mocap_from_base_footprint: {np.round(footprint.position, 5).tolist()} "
          f"{np.round(footprint.orientation, 5).tolist()}")

    stored = calibration_file(session, digits)
    if stored.is_file():
        print(_against(stored.name, pose_from(json.loads(stored.read_text())["base_mocap_from_base_footprint"]),
                       footprint))

    references, tools, joints = read_validation(session)
    if len(joints) >= 2:
        distances, angles = offset_spread(tool_offsets(references, tools, joints, footprint, robot.model.urdf, arm))
        print(f"validation, {len(joints)} samples: tool body in tool0 within {distances.max() * 1e3:.2f} mm "
              f"(95 %: {np.percentile(distances, 95) * 1e3:.2f} mm) and {angles.max():.3f}° of its mean")
    else:
        print("validation: fewer than two samples in validation/")

    old = None
    if args.old_scripts:
        with tempfile.TemporaryDirectory(prefix="old_calibration_scripts_") as folder:
            old, circles = old_result(run_old_scripts(session, takes, digits, arm, Path(folder)), digits)
        centres, normals = circle_differences(takes, solution, circles)
        print(f"old scripts {_against('the plugin', footprint, old)}; circles: centres within {centres * 1e3:.4f} mm, "
              f"normals within {normals:.4f}°")
    if args.truth_error:
        truth = true_calibration(robot.mocap_id, [float(value) for value in args.truth_error.split(",")])
        print(f"plugin {_against('the truth', truth, footprint)}")
        if old is not None:
            print(f"old scripts {_against('the truth', truth, old)}")

    if args.write:
        print(f"wrote {write_solution(session, digits, arm, robot.mocap_id, solution)}")


if __name__ == "__main__":
    main()
