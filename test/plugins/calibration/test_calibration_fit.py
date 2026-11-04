"""The base calibration's math: the old pipeline's results reproduced, and a known calibration recovered.

`data/<session>.npz` holds the J0 and J1 takes of old sessions on the Drive (`<batch>_analysis.json`) and what the
old scripts computed from them (`base_frame_calibration.json`, `calibrated_transformation_<serial>_rhino.json`):
Belle 20261013 (one arm), Cindy 20261016 (left arm) and 20260625 (right arm).
"""

from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from repo_paths import DATA
from husky_assembly_teleop.plugins.calibration.analysis.fit import (Take, fit_circle, offset_spread, solve,
                                                                    tool_offsets, tool_points)
from scaffolding_assembly_core.geometry import Pose, compose
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES

pytestmark = pytest.mark.safety

SESSIONS = Path(__file__).parent / "data"
#: A stock UR5e on a one-arm husky: J1 crosses J0 at right angles, so a calibration is recovered exactly.
STOCK_URDF = DATA / "husky_urdf" / "mt_husky_moveit_config" / "urdf" / "husky_ur5_e_no_base_joint.urdf"


def _poses(rows: np.ndarray) -> list[Pose]:
    return [Pose.from_arrays(row[:3], row[3:]) for row in rows]


def _session_takes(session, batch: str) -> list[Take]:
    joint = int(batch[1])
    references, tools = _poses(session[f"{batch}_base"]), _poses(session[f"{batch}_tool"])
    points = tool_points(references, tools)
    index = session[f"{batch}_take"]
    return [Take(joint, points[index == k], session[f"{batch}_joints"][index == k, joint], f"{batch} take {k}")
            for k in range(index.max() + 1)]


@pytest.mark.parametrize("name", ["20261013", "20261016", "20260625"])
def test_old_sessions_are_reproduced(name):
    """Circles, arm base and calibration match the old scripts' outputs to well under a micrometre."""
    session = np.load(SESSIONS / f"{name}.npz")
    j0, j1 = _session_takes(session, "j0"), _session_takes(session, "j1")
    for batch, takes in (("j0", j0), ("j1", j1)):
        for take, center, normal in zip(takes, session[f"{batch}_centers"], session[f"{batch}_normals"]):
            circle = fit_circle(take)
            assert circle.center == pytest.approx(center, abs=1e-6)
            assert circle.normal == pytest.approx(normal, abs=1e-6)
    solution = solve(j0, j1, DATA / "husky_urdf" / str(session["urdf"]), str(session["arm"]))
    assert solution.arm_base.matrix() == pytest.approx(session["arm_base"], abs=1e-6)
    assert solution.footprint.position == pytest.approx(session["footprint"][:3], abs=1e-6)
    turn = Rotation.from_quat(solution.footprint.orientation).inv() * Rotation.from_quat(session["footprint"][3:])
    assert np.degrees(turn.magnitude()) < 1e-4
    # * Every take swept its joint one way, and the fits are as good as the old logs said (about a millimetre).
    circles = solution.j0_circles + solution.j1_circles
    assert all(abs(circle.turn) > 0.9 for circle in circles)
    assert max(circle.errors.max() for circle in circles) < 1.5e-3


ARM = "ur_arm"
#: The tool body on the flange, in tool0: off-centre and turned, like a real marker tree.
TOOL_OFFSET = Pose((0.02, -0.01, 0.12), tuple(Rotation.from_euler("xyz", (10, -5, 30), degrees=True).as_quat()))
#: The calibration to recover: base_footprint in the base rigid body.
TRUTH = Pose((0.05, 0.01, -0.48), tuple(Rotation.from_euler("xyz", (0.4, -0.3, 91.0), degrees=True).as_quat()))


def _reference(i: int) -> Pose:
    """The base rigid body in the world at sample `i`: a few millimetres of sway and a fraction of a degree."""
    turn = Rotation.from_euler("xyz", (0.2 * np.sin(i), 0.1 * np.cos(i), 30 + 0.3 * np.sin(2 * i)), degrees=True)
    return Pose((1.0 + 0.003 * np.sin(i), -2.0 + 0.002 * np.cos(i), 0.47), tuple(turn.as_quat()))


def _measure(configurations, calibration: Pose = TRUTH):
    """Reference and tool poses for joint configurations, as mocap would measure them."""
    references, tools = [], []
    for i, angles in enumerate(configurations):
        reference = _reference(i)
        joints = {f"{ARM}_{name}": value for name, value in zip(UR_JOINT_NAMES, angles)}
        tool0 = link_pose(STOCK_URDF, compose(reference, calibration), joints, f"{ARM}_tool0")
        references.append(reference)
        tools.append(compose(tool0, TOOL_OFFSET))
    return references, tools


def _sweeps(reverse: bool = False):
    """J0 sweeps at three heights and J1 sweeps (J0 at 0) with three wrists, 12° apart; optionally swept backwards."""
    j0_angles = np.arange(-np.pi, 2.93, np.radians(12))
    j1_angles = np.linspace(-2.7, 0.43, 31)
    if reverse:
        j0_angles, j1_angles = j0_angles[::-1], j1_angles[::-1]
    heights = ((-0.5, -0.8), (-1.0, -1.2), (-1.4, 0.9))
    j0 = [[(a, lift, elbow, -1.2, 1.5, 0.3) for a in j0_angles] for lift, elbow in heights]
    j1 = [[(0.0, a, 0.0, wrist, 1.2, 0.0) for a in j1_angles] for wrist in (-1.0, 0.4, 1.3)]
    return j0, j1


def _takes(sweeps, joint: int, calibration: Pose = TRUTH, reference_of=lambda reference: reference):
    takes = []
    for k, sweep in enumerate(sweeps):
        references, tools = _measure(sweep, calibration)
        points = tool_points([reference_of(reference) for reference in references], tools)
        takes.append(Take(joint, points, np.array([angles[joint] for angles in sweep]), f"take {k}"))
    return takes


def _assert_same_pose(a: Pose, b: Pose, metres: float = 1e-6, degrees: float = 1e-4):
    assert a.position == pytest.approx(b.position, abs=metres)
    turn = Rotation.from_quat(a.orientation).inv() * Rotation.from_quat(b.orientation)
    assert np.degrees(turn.magnitude()) < degrees


@pytest.mark.parametrize("reverse", [False, True])
def test_a_known_calibration_is_recovered(reverse):
    """Perfect measurements of a stock arm give back the calibration, whichever way the joints were swept."""
    j0, j1 = _sweeps(reverse)
    solution = solve(_takes(j0, 0), _takes(j1, 1), STOCK_URDF, ARM)
    _assert_same_pose(solution.footprint, TRUTH)
    assert solution.skew == pytest.approx(0.0, abs=1e-6)
    assert solution.axes_angle == pytest.approx(90.0, abs=1e-4)
    assert all(circle.errors.max() < 1e-9 for circle in solution.j0_circles + solution.j1_circles)


def test_measuring_under_a_calibration_solves_the_true_one():
    """Measured in base_footprint under a wrong calibration (what the relay publishes), the solution corrects it."""
    applied = compose(TRUTH, Pose((0.004, -0.003, 0.002), tuple(Rotation.from_euler("z", 0.5, degrees=True).as_quat())))
    j0, j1 = _sweeps()
    published = lambda reference: compose(reference, applied)  # noqa: E731
    solution = solve(_takes(j0, 0, reference_of=published), _takes(j1, 1, reference_of=published), STOCK_URDF, ARM)
    _assert_same_pose(compose(applied, solution.footprint), TRUTH)


def test_tool_offsets_are_constant_only_under_the_right_calibration():
    """Validation: the tool body stays put in tool0 under the truth, and moves by millimetres under a wrong one."""
    configurations = [(0.3 * i - 1.0, -1.0 + 0.1 * i, 1.2 - 0.2 * i, -1.5, 1.4 + 0.1 * i, 0.2 * i) for i in range(8)]
    references, tools = _measure(configurations)
    distances, angles = offset_spread(tool_offsets(references, tools, configurations, TRUTH, STOCK_URDF, ARM))
    assert distances.max() < 1e-9 and angles.max() < 1e-5
    wrong = compose(TRUTH, Pose((0.0, 0.0, 0.0), tuple(Rotation.from_euler("z", 0.5, degrees=True).as_quat())))
    distances, angles = offset_spread(tool_offsets(references, tools, configurations, wrong, STOCK_URDF, ARM))
    assert distances.max() > 1e-3 and angles.max() > 0.1


def test_too_few_points_are_refused():
    """A circle needs three distinct points: repeated samples do not count."""
    with pytest.raises(ValueError, match="three distinct points"):
        fit_circle(Take(0, np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.0, 0.0]]), np.zeros(3)))
