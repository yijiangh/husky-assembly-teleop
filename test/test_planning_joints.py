"""Tests for shared.planning.joints: whole-turn unwrapping, URDF limits, the limit report, arm names and sides."""

from __future__ import annotations

from math import pi
from pathlib import Path

import numpy as np
import pytest

from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.shared.planning.joints import (arm_joint_names, arm_side, joint_limits, outside_limits,
                                                          unwrap_to_nearest)

DATA = Path(__file__).resolve().parent.parent / "data"


def test_a_whole_turn_is_taken_off():
    """A goal one turn and a bit away comes back to the bit."""
    assert unwrap_to_nearest([2 * pi + 0.1], [0.0]) == pytest.approx([0.1])


def test_elbow_keeps_its_in_range_value():
    """Start 2.26, goal -1.06 on a ±π elbow: the nearest value (5.22) is out of range, so -1.06 stays."""
    assert unwrap_to_nearest([-1.06], [2.26]) == pytest.approx([-1.06 + 2 * pi])
    assert unwrap_to_nearest([-1.06], [2.26], [-pi], [pi]) == pytest.approx([-1.06])


def test_no_in_range_equivalent_keeps_the_nearest():
    """No turn of 1.0 lies in [0, 0.5]: the goal keeps its value nearest the start."""
    assert unwrap_to_nearest([1.0 + 2 * pi], [3.0], [0.0], [0.5]) == pytest.approx([1.0])


def test_half_turn_ties_follow_numpy_round():
    """Exactly half a turn away is a tie: numpy rounds half to even, so 0.5 turns stays, 1.5 turns goes 2 turns."""
    assert unwrap_to_nearest([pi], [0.0]) == pytest.approx([pi])
    assert unwrap_to_nearest([3 * pi], [0.0]) == pytest.approx([-pi])


def test_a_goal_exactly_at_a_limit_is_in_range():
    """The nearest equivalent sits exactly on the upper limit (or a hair past it, by rounding): it is kept."""
    assert unwrap_to_nearest([-pi], [2.0], [-pi], [pi]) == pytest.approx([pi])
    assert unwrap_to_nearest([pi + 1e-10], [2.0], [-pi], [pi]) == pytest.approx([pi + 1e-10])


def test_each_joint_on_its_own():
    """Joints are unwrapped independently, in order."""
    goal = unwrap_to_nearest([2 * pi + 0.1, -0.2, 4.0], [0.0, 0.0, -2.0], [-2 * pi, -pi, -pi], [2 * pi, pi, pi])
    assert goal == pytest.approx([0.1, -0.2, 4.0 - 2 * pi])


def test_cindy_joint_limits():
    """Cindy's URDF: the elbow turns ±π, the shoulder pan ±2π."""
    urdf = robot_config_from_serial("0806", DATA).model.urdf
    lower, upper = joint_limits(urdf, ["left_ur_arm_elbow_joint", "left_ur_arm_shoulder_pan_joint"])
    assert lower == pytest.approx([-pi, -2 * pi])
    assert upper == pytest.approx([pi, 2 * pi])


def test_continuous_joint_has_no_limits(tmp_path):
    """A continuous joint gets ±inf."""
    urdf = tmp_path / "wheel.urdf"
    urdf.write_text("""<?xml version="1.0"?>
<robot name="wheel">
  <link name="base"/><link name="wheel"/>
  <joint name="spin" type="continuous"><parent link="base"/><child link="wheel"/><axis xyz="0 1 0"/></joint>
</robot>
""")
    lower, upper = joint_limits(urdf, ["spin"])
    assert np.isneginf(lower[0]) and np.isposinf(upper[0])


def test_outside_limits_text():
    """One line per joint outside its limits, labelled; a hair outside counts as inside."""
    names = ["a", "b"]
    lines = outside_limits([("start", [0.0, 3.5]), ("goal", [-4.0, pi + 1e-7])], names, [-pi, -pi], [pi, pi])
    assert lines == ["start b=+3.500 rad (limits [-3.142, +3.142])", "goal a=-4.000 rad (limits [-3.142, +3.142])"]


def test_arm_names_and_sides():
    """Six joints in the UR driver's order; Cindy's arms have sides, a one-arm robot's is "arm"."""
    assert arm_joint_names("left_ur_arm") == [
        "left_ur_arm_shoulder_pan_joint", "left_ur_arm_shoulder_lift_joint", "left_ur_arm_elbow_joint",
        "left_ur_arm_wrist_1_joint", "left_ur_arm_wrist_2_joint", "left_ur_arm_wrist_3_joint"]
    assert [arm_side(arm) for arm in ("left_ur_arm", "right_ur_arm", "ur_arm")] == ["left", "right", "arm"]
