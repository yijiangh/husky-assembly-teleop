"""Tool steps: small moves in tool0's frame solved on Belle's URDF, and the arm sending them as short joint moves."""

from __future__ import annotations

import numpy as np
import pytest
from robot_fakes import fake_arm
from repo_paths import DATA
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.robot_interface.arm import TOOL_STEP_MIN_DURATION
from husky_assembly_teleop.robot_interface.tool_step import (TOOL_JOG_LEAD, TOOL_JOG_SPEED_MAX, TOOL_STEP_MAX,
                                                             TOOL_STEP_MAX_ANGLE, _difference, tool0_pose,
                                                             tool_jog_target, tool_step_target)
from scaffolding_assembly_core.geometry import Pose, compose, invert
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES

pytestmark = pytest.mark.safety

ARM = "ur_arm"
#: Belle's URDF, whose arm is ur_arm.
URDF = robot_config_from_serial("0805", DATA).model.urdf
#: An arm pose well away from singularities.
JOINTS = np.radians([10.0, -100.0, 90.0, -80.0, -90.0, 30.0])
#: A punch tip in tool0.
TIP = (0.0, 0.0, 0.11)


def tool0(joints) -> Pose:
    """Tool0 in the URDF's root frame at these joints."""
    return link_pose(URDF, Pose(), {f"{ARM}_{name}": value for name, value in zip(UR_JOINT_NAMES, joints)},
                     f"{ARM}_tool0")


def turn(axis: str, degrees: float) -> Pose:
    """A pure turn about one of tool0's axes."""
    return Pose(orientation=tuple(Rotation.from_euler(axis, degrees, degrees=True).as_quat()))


@pytest.mark.parametrize("shift", [(1e-4, 0.0, 0.0), (0.0, -1e-3, 0.0), (0.0, 0.0, TOOL_STEP_MAX)])
def test_a_shift_moves_tool0_by_it_in_its_own_frame(shift):
    """The new tool0, seen from the old one, is the shift and no turn."""
    moved = compose(invert(tool0(JOINTS)), tool0(tool_step_target(URDF, ARM, JOINTS, Pose(shift))))
    np.testing.assert_allclose(moved.position, shift, atol=1e-7)
    assert Rotation.from_quat(moved.orientation).magnitude() < 1e-6


@pytest.mark.parametrize("axis", "xyz")
def test_a_turn_about_the_tip_keeps_the_tip_in_place(axis):
    """Turning 5° about the tip moves tool0 but not the tip, and turns tool0 by the step."""
    joints = tool_step_target(URDF, ARM, JOINTS, turn(axis, 5.0), TIP)
    before, after = compose(tool0(JOINTS), Pose(TIP)), compose(tool0(joints), Pose(TIP))
    np.testing.assert_allclose(after.position, before.position, atol=1e-7)
    turned = Rotation.from_quat(invert(tool0(JOINTS)).orientation) * Rotation.from_quat(tool0(joints).orientation)
    assert turned.magnitude() == pytest.approx(np.radians(5.0), abs=1e-6)


@pytest.mark.parametrize("step", [Pose((TOOL_STEP_MAX * 1.01, 0.0, 0.0)),
                                  turn("z", np.degrees(TOOL_STEP_MAX_ANGLE) * 1.01)])
def test_a_step_over_the_limit_is_refused(step):
    """A slipped digit cannot become a large move."""
    with pytest.raises(ValueError, match="over the limit"):
        tool_step_target(URDF, ARM, JOINTS, step)


@pytest.mark.parametrize("wrist_2", [0.0, np.radians(1.0)])
def test_a_turn_near_the_wrist_singularity_is_refused(wrist_2):
    """With wrist 2 near zero a tilt needs large wrist moves: refused instead of whipping the wrist round."""
    joints = JOINTS.copy()
    joints[4] = wrist_2
    with pytest.raises(ValueError, match="singularity"):
        tool_step_target(URDF, ARM, joints, turn("x", 1.0))


def test_the_arm_sends_a_tool_step_as_a_short_joint_move():
    """It ends at the solved joints, after the shortest tool step's duration."""
    arm = fake_arm(joints=JOINTS)
    arm.urdf = URDF
    assert arm.send_tool_step(Pose((0.0, 0.0, 1e-3))) == TOOL_STEP_MIN_DURATION
    trajectory, = arm._trajectory.sent
    expected = tool_step_target(URDF, ARM, JOINTS, Pose((0.0, 0.0, 1e-3)))
    np.testing.assert_allclose(trajectory.points[-1].positions, expected)


@pytest.mark.parametrize("urdf, step, why", [(None, Pose((1e-3, 0.0, 0.0)), "no URDF"),
                                             (URDF, Pose((0.1, 0.0, 0.0)), "over the limit")])
def test_a_tool_step_the_arm_cannot_solve_is_refused(urdf, step, why):
    """Nothing goes out, and the refusal says why."""
    arm = fake_arm(joints=JOINTS)
    arm.urdf = urdf
    assert arm.send_tool_step(step) is None
    assert not arm._trajectory.sent and why in arm.last_refusal.reason


# --- --- --- --- --- the tool0 jog --- --- --- --- ---

def jog_progress(joints, anchor: np.ndarray) -> np.ndarray:
    """Tool0 at `joints` seen from `anchor`: shift (m) then turn (rad), in the anchor's own axes."""
    return _difference(tool0_pose(URDF, ARM, joints), anchor, local=True)


def test_a_jog_point_lies_a_lead_ahead_on_the_anchor_line():
    """From the press, the first point is speed * lead along +x of tool0 and nothing else; the next, one more."""
    anchor = tool0_pose(URDF, ARM, JOINTS)
    first, travelled = tool_jog_target(URDF, ARM, JOINTS, anchor, 0, 5e-3, TOOL_JOG_LEAD)
    assert travelled == pytest.approx(0.0, abs=1e-9)
    np.testing.assert_allclose(jog_progress(first, anchor), [5e-3 * TOOL_JOG_LEAD, 0, 0, 0, 0, 0], atol=1e-7)
    second, travelled = tool_jog_target(URDF, ARM, first, anchor, 0, 5e-3, TOOL_JOG_LEAD, travelled)
    assert travelled == pytest.approx(5e-3 * TOOL_JOG_LEAD, abs=1e-7)
    np.testing.assert_allclose(jog_progress(second, anchor), [2 * 5e-3 * TOOL_JOG_LEAD, 0, 0, 0, 0, 0], atol=1e-7)


def test_a_lagging_arm_is_brought_back_onto_the_line_not_asked_to_jump():
    """An arm that slipped off the line sideways gets a point on the line, one lead ahead of its progress along it."""
    anchor = tool0_pose(URDF, ARM, JOINTS)
    off_line = tool_step_target(URDF, ARM, JOINTS, Pose((2e-3, 1e-3, 0.0)))  # 2 mm along, 1 mm beside
    point, travelled = tool_jog_target(URDF, ARM, off_line, anchor, 0, 5e-3, TOOL_JOG_LEAD)
    assert travelled == pytest.approx(2e-3, abs=1e-7)
    np.testing.assert_allclose(jog_progress(point, anchor), [2e-3 + 5e-3 * TOOL_JOG_LEAD, 0, 0, 0, 0, 0], atol=1e-7)


def test_a_turn_past_180_degrees_keeps_counting():
    """Wrist 3 turns tool0 about its own z: at 185° the progress reads 185°, not -175°, given last tick's 175°."""
    anchor = tool0_pose(URDF, ARM, JOINTS)
    sign = np.sign(jog_progress(JOINTS + np.radians([0, 0, 0, 0, 0, 10.0]), anchor)[5])
    far = JOINTS + np.radians([0, 0, 0, 0, 0, 185.0])
    _point, travelled = tool_jog_target(URDF, ARM, far, anchor, 5, sign * np.radians(2.0), TOOL_JOG_LEAD,
                                        travelled_before=sign * np.radians(175.0))
    assert travelled == pytest.approx(sign * np.radians(185.0), abs=1e-6)


def test_the_top_jog_speed_is_not_refused_as_a_large_step():
    """50 mm/s with a 0.2 s lead is a 10 mm point, the tool step's own limit: the jog allows twice its travel."""
    anchor = tool0_pose(URDF, ARM, JOINTS)
    point, _travelled = tool_jog_target(URDF, ARM, JOINTS, anchor, 2, TOOL_JOG_SPEED_MAX * 1e-3, TOOL_JOG_LEAD)
    np.testing.assert_allclose(jog_progress(point, anchor)[:3], [0, 0, TOOL_JOG_SPEED_MAX * 1e-3 * TOOL_JOG_LEAD],
                               atol=1e-7)


def test_a_jog_near_the_wrist_singularity_is_refused():
    """A turn about tool0's x needs large wrist moves with wrist 2 at zero: the solver refuses, so the jog stops."""
    joints = JOINTS.copy()
    joints[4] = 0.0
    with pytest.raises(ValueError, match="singularity"):
        tool_jog_target(URDF, ARM, joints, tool0_pose(URDF, ARM, joints), 3, np.radians(10.0), TOOL_JOG_LEAD)
