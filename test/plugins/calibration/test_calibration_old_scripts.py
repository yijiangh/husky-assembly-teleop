"""The old pipeline's scripts run on sessions the plugin wrote, and agree with the plugin and with the stored results.

Each session in `data/` (old real sessions, see test_calibration_fit.py) is written as the plugin writes takes; then
scripts/solve_calibration.py runs the old scripts 0 to 2 on it (data/calibration_data/), as `--old-scripts` does.
"""

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from repo_paths import DATA, REPO
from husky_assembly_teleop.plugins.calibration.analysis.fit import solve
from husky_assembly_teleop.plugins.calibration.storage.session_files import read_takes, write_take

# * The old scripts take ~25 s a session alone and up to twice that on a busy machine.
pytestmark = [pytest.mark.slow, pytest.mark.old_scripts, pytest.mark.timeout(300)]

SESSIONS = Path(__file__).parent / "data"
SCRIPT = REPO / "scripts" / "solve_calibration.py"


def _script():
    """scripts/solve_calibration.py as a module."""
    spec = importlib.util.spec_from_file_location("solve_calibration", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write_session(folder: Path, session) -> None:
    """The session's J0 and J1 takes, one file each, named and filed as the plugin does."""
    serial, arm = str(session["serial"]), str(session["arm"])
    for batch in ("j0", "j1"):
        (folder / batch).mkdir(parents=True)
        index = session[f"{batch}_take"]
        for k in range(index.max() + 1):
            rows = index == k
            samples = [{"joint_conf": joints.tolist(), "base_mocap_pose": [base[:3].tolist(), base[3:].tolist()],
                        "flange_mocap_pose": [tool[:3].tolist(), tool[3:].tolist()]}
                       for joints, base, tool in zip(session[f"{batch}_joints"][rows], session[f"{batch}_base"][rows],
                                                     session[f"{batch}_tool"][rows])]
            write_take(folder / batch, f"1015{k:02d}_{serial}_{arm}_J{batch[1]}_take{k + 1}", samples)


def _degrees(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.degrees((Rotation.from_quat(a).inv() * Rotation.from_quat(b)).magnitude()))


@pytest.mark.parametrize("name", ["20261013", "20260625"])
def test_old_scripts_agree_with_the_plugin(name, tmp_path):
    """The old scripts' circles and result equal the plugin's, and both equal the result stored on the Drive.

    One session per robot kind (Belle's single arm, Cindy's right arm): old script 0 alone takes ~30 s a session.
    """
    script, session = _script(), np.load(SESSIONS / f"{name}.npz")
    serial, arm = str(session["serial"]), str(session["arm"])
    folder = tmp_path / name
    _write_session(folder, session)
    takes, skipped = read_takes(folder)
    assert not skipped
    solution = solve([t for t in takes if t.joint == 0], [t for t in takes if t.joint == 1],
                     DATA / "husky_urdf" / str(session["urdf"]), arm)

    old, circles = script.old_result(script.run_old_scripts(folder, takes, serial, arm, tmp_path / "old"), serial)

    assert len(circles) == len(takes)
    centres, normals = script.circle_differences(takes, solution, circles)
    assert centres < 1e-6 and normals < 1e-4
    assert old.position == pytest.approx(solution.footprint.position, abs=1e-6)
    assert _degrees(old.orientation, solution.footprint.orientation) < 1e-4
    assert old.position == pytest.approx(session["footprint"][:3], abs=1e-6)
    assert _degrees(old.orientation, session["footprint"][3:]) < 1e-4
