"""Tests for tool_urdfs.tool_spec: a tool URDF read into the core's Tool, visual, collision and TCP apart."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from husky_assembly_teleop import tool_urdfs
from husky_assembly_teleop.tool_urdfs import TOOL_URDFS, tool_spec

DATA = Path(__file__).resolve().parent.parent / "data"

#: A tool with a box to draw, a smaller box to collide with, and its TCP 0.15 m out along Z, turned about Z.
TEST_TOOL = """<?xml version="1.0"?>
<robot name="test_tool">
  <link name="test_tool">
    <visual><geometry><box size="0.2 0.1 0.1"/></geometry></visual>
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <link name="tcp"/>
  <joint name="tcp_joint" type="fixed">
    <parent link="test_tool"/><child link="tcp"/>
    <origin xyz="0 0 0.15" rpy="0 0 1.5707963267948966"/>
  </joint>
</robot>
"""


@pytest.fixture
def test_tool(tmp_path, monkeypatch) -> Path:
    """A data directory holding TEST_TOOL as kind "test_tool"."""
    (tmp_path / tool_urdfs.TOOL_URDF_DIRECTORY).mkdir()
    (tmp_path / tool_urdfs.TOOL_URDF_DIRECTORY / "test_tool.urdf").write_text(TEST_TOOL)
    monkeypatch.setitem(TOOL_URDFS, "test_tool", "test_tool.urdf")
    return tmp_path


def test_visual_and_collision_are_read_apart(test_tool):
    """The drawn shape and the collision shape each come from their own URDF element."""
    tool = tool_spec("test_tool", "ur_arm", test_tool, "tools/a200-0804/ur_arm")
    (visual,), (collision,) = tool.geometry.visual, tool.geometry.collision
    np.testing.assert_allclose(np.ptp(visual.vertices, axis=0), (0.2, 0.1, 0.1), atol=1e-9)
    np.testing.assert_allclose(np.ptp(collision.vertices, axis=0), (0.1, 0.1, 0.1), atol=1e-9)
    assert tool.id == "tools/a200-0804/ur_arm" and tool.kind == "test_tool"


def test_tcp_is_the_tcp_link(test_tool):
    """The TCP is the pose of the link named TCP_LINK in the tool's root (tool0) frame."""
    tool = tool_spec("test_tool", "ur_arm", test_tool, "tools/a200-0804/ur_arm")
    np.testing.assert_allclose(tool.tcp.position, (0.0, 0.0, 0.15), atol=1e-9)
    np.testing.assert_allclose(np.abs(tool.tcp.orientation), (0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)), atol=1e-9)


@pytest.mark.parametrize("kind, arm", [("robotiq", "ur_arm"), ("scaffolding_v3", "left_ur_arm"),
                                       ("scaffolding_v3", "right_ur_arm")])
def test_mounted_tools_load(kind, arm):
    """Every tool model in data/ loads with shapes to draw and to collide with; no TCP link means tool0."""
    tool = tool_spec(kind, arm, DATA, f"tools/x/{arm}")
    assert tool.geometry.visual and tool.geometry.collision
    if f'"{tool_urdfs.TCP_LINK}"' not in tool_urdfs.tool_urdf(kind, arm, DATA).read_text():
        np.testing.assert_allclose(tool.tcp.position, (0.0, 0.0, 0.0))


def test_tool_without_a_model_is_none():
    """scaffolding_v1 has no model yet."""
    assert tool_spec("scaffolding_v1", "ur_arm", DATA, "tools/x/ur_arm") is None
