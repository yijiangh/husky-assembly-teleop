"""The base's mocap chip: amber while the relay finds its calibration built for an earlier body, as for marker error."""

from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace

import pytest

from husky_assembly_teleop.plugins.health import applied_calibration, applied_frame, calibrated, in_frame, robot_checks
from husky_assembly_teleop.robot_interface.base import BaseState
from husky_assembly_teleop.robot_interface.mocap_relay import (FAKE_MOTIVE, MOTIVE, TRUTH, AppliedCalibration,
                                                               FloorCalibrationState, MocapRelayState)
from husky_assembly_teleop.world.checks import BAD, GOOD, WARN, Check

#: Belle's calibration as the relay announces it, not checked.
APPLIED = AppliedCalibration((0.05, 0.01, -0.48), (0.0, 0.0, -0.71, 0.70), "ab12", "calibration_data/20261013", "")
WRONG = replace(APPLIED, definition_status="out_of_date", definition_detail="made for an earlier body: refined")


@pytest.mark.parametrize("calibration", [None, replace(APPLIED, definition_status="matches"), APPLIED])
def test_the_mocap_chip_is_unchanged_unless_the_calibration_is_wrong(calibration):
    """Matching, or not known (no definition in the file, Motive not heard): the chip as tracking makes it."""
    mocap = Check("mocap", GOOD, "rigid body 1850, mean marker error under 1 mm")
    assert calibrated(mocap, calibration) is mocap


@pytest.mark.parametrize("level, expected", [(GOOD, WARN), (WARN, WARN), (BAD, BAD)])
def test_a_wrong_calibration_makes_the_mocap_chip_at_least_amber_and_says_why(level, expected):
    """Amber like a high marker error; a lost body stays red; the hover text gains the relay's reason."""
    chip = calibrated(Check("mocap", level, "tracking"), WRONG)
    assert chip.label == "mocap" and chip.level == expected and "refined" in chip.detail


def test_the_base_line_gets_no_chip_of_its_own():
    """The calibration shows in the mocap chip: the line keeps its chips."""
    robot = SimpleNamespace(config=SimpleNamespace(serial="a200-0805", mocap_id=1850), arms={},
                            base=SimpleNamespace(state=BaseState(), simulation=None))
    (_name, plain), = robot_checks(robot, 100.0, {}, None)
    (_name, wrong), = robot_checks(robot, 100.0, {}, WRONG)
    assert [chip.label for chip in wrong] == [chip.label for chip in plain]
    assert "refined" in next(chip for chip in wrong if chip.label == "mocap").detail


@pytest.mark.parametrize("source, expected, frame", [(MOTIVE, "applied", True), (FAKE_MOTIVE, "applied", True),
                                                     (TRUTH, None, False)])
def test_a_base_is_judged_by_what_the_relay_applies_to_its_source(source, expected, frame):
    """By the relay's word on the body: Motive's or a fake Motive's by its calibration; a truth none, nor floor's."""
    held = FloorCalibrationState("held", "f.json")
    relay = MocapRelayState(calibrations={1850: "applied"}, simulated=source != MOTIVE, frame=held,
                            sources={1850: source})
    robot = SimpleNamespace(config=SimpleNamespace(mocap_id=1850))
    assert applied_calibration(relay, robot) == expected and applied_calibration(None, robot) is None
    assert (applied_frame(relay, robot) is held) == frame and applied_frame(None, robot) is None


@pytest.mark.parametrize("status, level", [("held", GOOD), ("none", GOOD), ("unchecked", WARN), ("refit_pending", WARN),
                                           ("untrusted", WARN), ("error", WARN)])
def test_a_moved_frame_makes_the_mocap_chip_amber_until_the_refit_is_used(status, level):
    """While the truss cameras moved and no refit is adopted, every base's poses are off: amber, with why."""
    frame = FloorCalibrationState(status, "f.json", pending="frame moved 20 mm")
    chip = in_frame(Check("mocap", GOOD, "tracking"), frame)
    assert chip.level == level and (status not in ("refit_pending", "untrusted") or "frame moved 20 mm" in chip.detail)


@pytest.mark.parametrize("moves_poses, level", [(True, WARN), (False, GOOD)])
def test_a_newer_floor_calibration_not_applied_turns_the_chip_amber_if_it_moves_poses(moves_poses, level):
    """Held, but a newer export waits for Apply floor calibration: amber, saying so; one within 1 mm leaves it green."""
    frame = FloorCalibrationState("held", "f.json", newer="g.json", newer_moves_poses=moves_poses)
    chip = in_frame(Check("mocap", GOOD, "tracking"), frame)
    assert chip.level == level
    assert moves_poses == ("a newer floor calibration (g.json) is not applied yet" in chip.detail)
