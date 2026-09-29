"""Every configured robot URDF must keep the stock UR base frames (doc/ur_frames.md).

A URDF that breaks this still looks right in the viewer, but Cartesian commands
go to the wrong place. Catch it here, before it reaches a robot.
"""

from pathlib import Path

import pytest

from husky_assembly_teleop.config import _ROBOTS_BY_SERIAL, robot_config_from_serial
from husky_assembly_teleop.robot_interface.ur_frames import stock_frame_problem

DATA = Path(__file__).resolve().parent.parent / "data"


@pytest.mark.parametrize("serial", sorted(_ROBOTS_BY_SERIAL))
def test_urdf_keeps_stock_ur_frames(serial):
    """No arm of any configured robot may have non-stock joints below its base_link."""
    config = robot_config_from_serial(serial, DATA)
    problems = [p for arm in config.arms if (p := stock_frame_problem(config.urdf_file, arm.name)) is not None]
    assert not problems, "\n".join(problems)
