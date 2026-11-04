"""The base's mocap chip in Health: the relay's calibration shows in it (`world.base_mocap`), not in a chip apart."""

from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace

from husky_assembly_teleop.plugins.health import robot_checks
from husky_assembly_teleop.robot_interface.base import BaseState
from husky_assembly_teleop.robot_interface.mocap_relay import AppliedCalibration

#: Belle's calibration as the relay announces it, made for an earlier body.
WRONG = replace(AppliedCalibration((0.05, 0.01, -0.48), (0.0, 0.0, -0.71, 0.70), "ab12", "calibration_data/20261013",
                                   ""), definition_status="out_of_date", definition_detail="made for an earlier body")


def test_the_base_line_gets_no_chip_of_its_own():
    """The calibration shows in the mocap chip: the line keeps its chips; a lost body says so first."""
    robot = SimpleNamespace(config=SimpleNamespace(serial="a200-0805", mocap_id=1850), arms={},
                            base=SimpleNamespace(state=BaseState(), simulation=None))
    (_name, plain), = robot_checks(robot, 100.0, {}, None)
    (_name, wrong), = robot_checks(robot, 100.0, {}, WRONG)
    assert [chip.label for chip in wrong] == [chip.label for chip in plain]
    mocap = next(chip for chip in plain if chip.label == "mocap")
    assert next(chip for chip in wrong if chip.label == "mocap").detail == f"{mocap.detail} (+1 more)"
