"""The cell plugin warns when a design's tools differ from the ones configured on the same robot."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

from husky_assembly_teleop.config import ArmConfig, RobotConfig
from husky_assembly_teleop.plugins.cell.plugin import tool_mismatches


def _design(kind_left: str, kind_right: str):
    """A stand-in for a loaded design: one dual-arm robot "0806" with two tools."""
    robots = {"robots/cindy": SimpleNamespace(id="robots/cindy", serial="0806",
                                              tools={"left_ur_arm_tool0": "tools/L", "right_ur_arm_tool0": "tools/R"}),
              "robots/ghost": SimpleNamespace(id="robots/ghost", serial=None, tools={})}
    tools = {"tools/L": SimpleNamespace(kind=kind_left), "tools/R": SimpleNamespace(kind=kind_right)}
    return SimpleNamespace(design=SimpleNamespace(robots=robots, tools=tools))


def _configured(left: str | None, right: str | None) -> tuple[RobotConfig, ...]:
    """The configured robot a200-0806 with these end effectors."""
    arms = (ArmConfig(name="left_ur_arm", ros_namespace="left_ur5e", end_effector=left),
            ArmConfig(name="right_ur_arm", ros_namespace="right_ur5e", end_effector=right))
    return (RobotConfig(serial="a200-0806", ros_namespace="a200_0806", urdf_file=Path("robot.urdf"), arms=arms),)


def test_same_tools_give_no_warning():
    """Equal kinds on every flange: nothing to report."""
    assert tool_mismatches(_design("scaffolding_v3", "scaffolding_v3"),
                           _configured("scaffolding_v3", "scaffolding_v3")) == []


def test_different_or_missing_tool_is_reported():
    """Another kind on one arm and a bare arm on the other: one line naming both."""
    lines = tool_mismatches(_design("scaffolding_v3", "scaffolding_v3"), _configured("robotiq", None))
    assert len(lines) == 1
    assert "robots/cindy (a200-0806)" in lines[0]
    assert "left_ur_arm scaffolding_v3 in the design, robotiq configured" in lines[0]
    assert "right_ur_arm scaffolding_v3 in the design, none configured" in lines[0]


def test_robots_not_configured_are_skipped():
    """A design robot without a serial, or not loaded, is not compared."""
    assert tool_mismatches(_design("robotiq", "robotiq"), ()) == []
