"""Cartesian targets go out in the controllers' frame (HuskyRobotInterface._to_controller_frame).

husky_robot imports the ROS stack, so these tests skip when it is not sourced.
"""

import numpy as np
import pytest

husky_robot = pytest.importorskip('husky_assembly_teleop.husky_robot')

# * Measured on Alice 2026-10-12 at the same joints: tool0 in the monitor's arm
# * base frame (calibrated URDF, ur_arm_base_link) and in UR's base_link (what her
# * cartesian_compliance_controller reads). Same height, turned 90 deg about z.
ALICE_MONITOR_POS = (0.3816, 0.1036, 0.5319)
ALICE_CONTROLLER_POS = (0.1036, -0.3816, 0.5319)


def _interface(dual_arm: bool):
    """An interface with no ROS connection (only the frame bookkeeping)."""
    return husky_robot.HuskyRobotInterface(None, dual_arm=dual_arm, connect_ros=False)


def test_alice_target_is_turned_into_ur_base_link():
    """Single-arm: the monitor's pose is turned -90 deg about z, position and orientation."""
    pose = (ALICE_MONITOR_POS, (0.0, 0.0, 0.0, 1.0))
    point, quat = _interface(dual_arm=False)._to_controller_frame(pose)
    assert np.allclose(point, ALICE_CONTROLLER_POS, atol=1e-4)
    # The orientation turns with it: identity -> a -90 deg yaw.
    assert np.allclose(np.abs(quat), np.abs([0.0, 0.0, -np.sqrt(0.5), np.sqrt(0.5)]), atol=1e-6)


def test_cindy_target_is_sent_unchanged():
    """Dual-arm: the monitor's frame is already UR's base_link, the pose object is returned as is."""
    pose = ((0.1, 0.2, 0.3), (0.0, 0.0, 0.0, 1.0))
    assert _interface(dual_arm=True)._to_controller_frame(pose) is pose
