"""Tests for arm.flange_from_reported: tool0, the link the compliance controller moves, from the reported TCP."""

from __future__ import annotations

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.robot_interface.arm import flange_from_reported
from husky_assembly_teleop.robot_interface.ur_frames import BASE_LINK_FROM_UR_BASE
from scaffolding_assembly_core.geometry import Pose, compose

pytestmark = pytest.mark.safety

#: tool0 somewhere in front of the arm, in base_link, turned.
TOOL0 = Pose((0.4, -0.2, 0.3), tuple(Rotation.from_euler("xyz", (0.3, -0.5, 1.1)).as_quat()))
#: A tool's TCP in tool0: 15 cm out along Z, a quarter turn about Z.
TCP = Pose((0.0, 0.0, 0.15), tuple(Rotation.from_euler("z", np.pi / 2).as_quat()))


def reported(tcp: Pose) -> tuple[np.ndarray, np.ndarray]:
    """What the driver reports in UR Base for TOOL0 with this TCP set on the pendant."""
    in_base_link = compose(TOOL0, tcp)
    position = BASE_LINK_FROM_UR_BASE.inv().apply(in_base_link.position)
    quaternion = (BASE_LINK_FROM_UR_BASE.inv() * Rotation.from_quat(in_base_link.orientation)).as_quat()
    return position, quaternion


def assert_same_pose(position: np.ndarray, quaternion: np.ndarray, pose: Pose) -> None:
    """The position matches to a micrometre and the orientation to a microradian."""
    np.testing.assert_allclose(position, pose.position, atol=1e-6)
    angle = (Rotation.from_quat(quaternion).inv() * Rotation.from_quat(pose.orientation)).magnitude()
    assert angle < 1e-6


def test_tcp_at_tool0_is_only_turned_into_base_link():
    """With no tool TCP the report is tool0, turned from UR Base into base_link."""
    assert_same_pose(*flange_from_reported(*reported(Pose()), Pose()), TOOL0)


def test_the_tools_tcp_comes_off():
    """A pendant TCP 15 cm out still gives tool0, not the tool tip."""
    position, quaternion = flange_from_reported(*reported(TCP), TCP)
    assert_same_pose(position, quaternion, TOOL0)


def test_a_tcp_the_monitor_does_not_know_shows_as_an_offset():
    """A pendant TCP the monitor does not take off moves tool0 by exactly that TCP: what the health check sees."""
    position, _ = flange_from_reported(*reported(TCP), Pose())
    assert abs(np.linalg.norm(position - np.array(TOOL0.position)) - 0.15) < 1e-9
