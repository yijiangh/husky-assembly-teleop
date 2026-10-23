"""Tool steps: small moves in tool0's frame solved on Belle's URDF, and the arm sending them as short joint moves."""

from __future__ import annotations

import numpy as np
import pytest
from robot_fakes import fake_arm
from repo_paths import DATA
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.robot_interface.arm import TOOL_STEP_MIN_DURATION
from husky_assembly_teleop.robot_interface.tool_step import TOOL_STEP_MAX, TOOL_STEP_MAX_ANGLE, tool_step_target
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
