"""Every configured robot URDF must keep the stock UR base frames (doc/ur_frames.md).

A URDF that breaks this still looks right in the viewer, but Cartesian commands
go to the wrong place. Catch it here, before it reaches a robot.
"""

import re

import pytest

from repo_paths import DATA
from husky_assembly_teleop.config import _ROBOTS_BY_SERIAL, robot_config_from_serial
from husky_assembly_teleop.robot_interface.ur_frames import stock_frame_problem

pytestmark = pytest.mark.safety


@pytest.mark.parametrize("serial", sorted(_ROBOTS_BY_SERIAL))
def test_urdf_keeps_stock_ur_frames(serial):
    """No arm of any configured robot may have non-stock joints below its base_link."""
    config = robot_config_from_serial(serial, DATA)
    problems = [p for arm in config.arms if (p := stock_frame_problem(config.model.urdf, arm.name)) is not None]
    assert not problems, "\n".join(problems)


#: Every flattened robot URDF in husky_urdf with a UR arm; ur_description's own files are left out.
ARM_URDFS = sorted(p for p in (DATA / "husky_urdf").rglob("*.urdf")
                   if "ur_description" not in p.parts and "_base_link_inertia" in p.read_text())


@pytest.mark.parametrize("urdf", ARM_URDFS, ids=lambda p: p.name)
def test_every_arm_urdf_keeps_stock_ur_frames(urdf):
    """No husky_urdf file may carry the old turned UR base frames: others copy from them (doc/ur_frames.md)."""
    arms = sorted({name[:-len("_base_link_inertia")] for name in re.findall(r'link name="([^"]*_base_link_inertia)"',
                                                                            urdf.read_text())})
    problems = [p for arm in arms if (p := stock_frame_problem(urdf, arm)) is not None]
    assert arms and not problems, "\n".join(problems)
