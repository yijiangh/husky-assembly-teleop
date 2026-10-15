"""Tests for world.model_check and the process plugin's use of it: where two models of one robot disagree."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.plugins.process.plugin import robot_differences
from husky_assembly_teleop.world.model_check import model_differences
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.robot import robot_model
from process_fixtures import build_process

DATA = Path(__file__).resolve().parent.parent / "data"
#: Alice's URDF before the stock UR base frames were restored: only ur_arm_base_link is turned, by 90°.
NOT_STOCK_ALICE = DATA / "husky_urdf/mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint_Alice_Calibrated.urdf"


@pytest.fixture(scope="module")
def alice():
    """Alice as configured: one arm with the Robotiq."""
    return robot_config_from_serial("0804", DATA)


def test_a_model_agrees_with_itself(alice):
    """Nothing to report."""
    assert model_differences(alice.model, alice.model) == []


def test_another_tool_kind_is_named(alice):
    """A different tool on the same flange names both kinds."""
    other = robot_config_from_serial("0804", DATA, tools=("scaffolding_v3",))
    (line,) = model_differences(other.model, alice.model)
    assert line == "ur_arm_tool0: scaffolding_v3 in the process, robotiq configured"


def test_a_missing_tool_is_named(alice):
    """A bare arm against a tool on it."""
    bare = robot_config_from_serial("0804", DATA, tools=("none",))
    assert model_differences(alice.model, bare.model) == ["ur_arm_tool0: a tool only in the process"]


def test_a_tcp_offset_is_measured(alice):
    """A TCP 15 cm out is reported with its size."""
    tool = alice.model.tools["ur_arm_tool0"]
    moved = replace(alice.model, tools={"ur_arm_tool0": replace(tool, tcp=Pose((0.0, 0.0, 0.15)))})
    assert model_differences(moved, alice.model) == ["ur_arm_tool0: TCP off by 150.0 mm / 0.00°"]


def test_a_turned_frame_without_shapes_does_not_count(alice):
    """The non-stock URDF turns only ur_arm_base_link, which has no shapes: nothing a planner would see."""
    variant = robot_model("alice", NOT_STOCK_ALICE, alice.model.srdf, alice.model.tools)
    assert model_differences(variant, alice.model) == []


def test_a_moved_arm_mount_is_found_by_its_kinematics(alice, tmp_path):
    """The arm mounted 5 mm further out: the worst link and how far, by forward kinematics, not by file."""
    text = alice.model.urdf.read_text()
    mount = '<origin rpy="0 0 -1.57079628038" xyz="-0.105 0.0 0.0" />'
    assert text.count(mount) == 1
    moved_urdf = tmp_path / "moved.urdf"
    moved_urdf.write_text(text.replace(mount, mount.replace("-0.105", "-0.110")))
    moved = robot_model("alice", moved_urdf, alice.model.srdf, alice.model.tools)
    (line,) = model_differences(moved, alice.model)
    assert " off by 5.0 mm / 0.00° (" in line


def test_process_robots_are_paired_with_configured_ones_by_serial(tmp_path):
    """Only the process robot with a configured serial is compared; its line names both."""
    process = build_process(tmp_path)
    cindy = robot_config_from_serial("0806", DATA)
    lines = robot_differences(SimpleNamespace(process=process), (cindy,))
    assert len(lines) == 1 and lines[0].startswith("robots/cindy (a200-0806): ")
    assert robot_differences(SimpleNamespace(process=process), ()) == []
