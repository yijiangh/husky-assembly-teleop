"""The base's mocap chip: amber while the relay's calibration or floor frame is off, the worst reason on hover."""

from __future__ import annotations

from dataclasses import replace

import pytest

from husky_assembly_teleop.robot_interface.base import BaseState
from husky_assembly_teleop.robot_interface.mocap_relay import (FAKE_MOTIVE, MOTIVE, TRUTH, AppliedCalibration,
                                                               FloorCalibrationState, MocapRelayState)
from husky_assembly_teleop.world.base_mocap import ASK_AFTER, ASK_AGAIN_AFTER, MocapWatch  # the watch
from husky_assembly_teleop.world.base_mocap import applied_calibration, applied_frame, base_mocap_check, with_relay
from husky_assembly_teleop.world.checks import BAD, GOOD, WARN, Check

#: Belle's calibration as the relay announces it, not checked.
APPLIED = AppliedCalibration((0.05, 0.01, -0.48), (0.0, 0.0, -0.71, 0.70), "ab12", "calibration_data/20261013", "")
WRONG = replace(APPLIED, definition_status="out_of_date", definition_detail="made for an earlier body: refined")
#: A frame whose truss cameras moved, no refit used yet.
MOVED = FloorCalibrationState("refit_pending", "f.json", pending="frame moved 20 mm")


@pytest.mark.parametrize("calibration", [None, replace(APPLIED, definition_status="matches"), APPLIED])
def test_the_mocap_chip_is_unchanged_unless_the_calibration_is_wrong(calibration):
    """Matching, or not known (no definition in the file, Motive not heard): the chip as tracking makes it."""
    mocap = Check("mocap", GOOD, "rigid body 1850, mean marker error under 1 mm")
    assert with_relay(mocap, calibration, None) is mocap


@pytest.mark.parametrize("level, expected, why", [(GOOD, WARN, "base calibration"),
                                                  (WARN, WARN, "base calibration"), (BAD, BAD, "lost")])
def test_a_wrong_calibration_makes_the_mocap_chip_at_least_amber_and_says_the_worst(level, expected, why):
    """Amber like a high marker error, which comes after it; a lost body stays red and says so first."""
    chip = with_relay(Check("mocap", level, "lost" if level == BAD else "tracking"), WRONG, None)
    assert chip.label == "mocap" and chip.level == expected and chip.detail.startswith(why)
    assert chip.detail.endswith("(+1 more)") == (level != GOOD)


def test_only_the_worst_reason_is_on_hover():
    """Frame moved, an old calibration and a high marker error: the frame's reason, the others counted."""
    chip = with_relay(Check("mocap", WARN, "mean marker error above 1 mm"), WRONG, MOVED)
    assert chip.level == WARN and chip.detail.startswith("Motive's frame moved") and chip.detail.endswith("(+2 more)")
    assert len(chip.detail) < 120


@pytest.mark.parametrize("source, expected, frame", [(MOTIVE, "applied", True), (FAKE_MOTIVE, "applied", True),
                                                     (TRUTH, None, False)])
def test_a_base_is_judged_by_what_the_relay_applies_to_its_source(source, expected, frame):
    """By the relay's word on the body: Motive's or a fake Motive's by its calibration; a truth none, nor floor's."""
    held = FloorCalibrationState("held", "f.json")
    relay = MocapRelayState(calibrations={1850: "applied"}, simulated=source != MOTIVE, frame=held,
                            sources={1850: source})
    assert applied_calibration(relay, 1850) == expected and applied_calibration(None, 1850) is None
    assert (applied_frame(relay, 1850) is held) == frame and applied_frame(None, 1850) is None


@pytest.mark.parametrize("status, level", [("held", GOOD), ("none", GOOD), ("unchecked", WARN), ("refit_pending", WARN),
                                           ("untrusted", WARN), ("error", WARN)])
def test_a_moved_frame_makes_the_mocap_chip_amber_until_the_refit_is_used(status, level):
    """While the truss cameras moved and no refit is adopted, every base's poses are off: amber, with why."""
    frame = FloorCalibrationState(status, "f.json", pending="frame moved 20 mm")
    chip = with_relay(Check("mocap", GOOD, "tracking"), None, frame)
    assert chip.level == level and (status not in ("refit_pending", "untrusted") or "Use refit" in chip.detail)


@pytest.mark.parametrize("moves_poses, level", [(True, WARN), (False, GOOD)])
def test_a_newer_floor_calibration_not_applied_turns_the_chip_amber_if_it_moves_poses(moves_poses, level):
    """Held, but a newer export waits for Apply floor calibration: amber, saying so; one within 1 mm leaves it green."""
    frame = FloorCalibrationState("held", "f.json", newer="g.json", newer_moves_poses=moves_poses)
    chip = with_relay(Check("mocap", GOOD, "tracking"), None, frame)
    assert chip.level == level
    assert moves_poses == ("a newer floor calibration waits" in chip.detail)


@pytest.mark.parametrize("status", ["held", "untrusted"])
def test_motives_scale_changed_turns_the_chip_amber_saying_so_held_or_not(status):
    """The relay reports Motive's scale changed: amber, and the scale is the reason, not a refit."""
    frame = FloorCalibrationState(status, "f.json", scale="distances -1550 ppm (8.1 mm across the lab's volume), ...")
    chip = with_relay(Check("mocap", GOOD, "tracking"), None, frame)
    assert chip.level == WARN and chip.detail.startswith("Motive's scale changed (distances -1550 ppm): recalibrate")


def test_a_base_without_a_mocap_id_says_so_whatever_the_frame():
    """No rigid body configured: never measured, so the floor frame cannot put it off; the chip names the config."""
    relay = MocapRelayState(calibrations={}, simulated=False, frame=MOVED)
    chip = base_mocap_check(None, BaseState(), relay, 100.0)
    assert chip.level == WARN and chip.detail.startswith("no mocap id configured") and "more" not in chip.detail


FRAME_MOVED, MARKERS = "Motive's frame moved: poses off until Use refit", "mean marker error over 1s above 1 mm"
AMBER, GREEN = Check("mocap", WARN, FRAME_MOVED), Check("mocap", GOOD, "fine")


def test_an_amber_chip_asks_only_after_it_stayed_amber():
    """A flicker of amber asks nothing; "(+N more)" is left out of the reason."""
    watch = MocapWatch()
    assert watch.update({"a": Check("mocap", WARN, f"{FRAME_MOVED} (+1 more)")}, 0.0) == {}
    assert watch.update({"a": GREEN}, ASK_AFTER - 0.1) == {}
    assert watch.update({"a": AMBER}, ASK_AFTER) == {}
    assert watch.update({"a": Check("mocap", WARN, f"{FRAME_MOVED} (+1 more)")}, 2 * ASK_AFTER) == {"a": FRAME_MOVED}


def test_green_or_red_stops_asking():
    """It asks only while amber; a lost fix is health's red chip, not this question."""
    watch = MocapWatch()
    watch.update({"a": AMBER}, 0.0)
    assert watch.update({"a": AMBER}, ASK_AFTER) == {"a": FRAME_MOVED}
    assert watch.update({"a": Check("mocap", BAD, "lost")}, ASK_AFTER + 1) == {}
    assert watch.update({"a": GREEN}, ASK_AFTER + 2) == {}


def test_what_was_acknowledged_stays_quiet_through_flickers_of_green():
    """Belle's chip flickers: green for a moment must not ask again; green for ASK_AGAIN_AFTER does."""
    watch = MocapWatch()
    watch.update({"a": AMBER}, 0.0)
    watch.update({"a": AMBER}, ASK_AFTER)
    watch.acknowledge()
    assert watch.update({"a": AMBER}, ASK_AFTER + 1) == {}
    watch.update({"a": GREEN}, 10.0)
    watch.update({"a": AMBER}, 11.0)
    assert watch.update({"a": AMBER}, 11.0 + ASK_AFTER) == {}
    watch.update({"a": GREEN}, 20.0)
    watch.update({"a": GREEN}, 20.0 + ASK_AGAIN_AFTER)
    watch.update({"a": AMBER}, 100.0)
    assert watch.update({"a": AMBER}, 100.0 + ASK_AFTER) == {"a": FRAME_MOVED}


def test_another_reason_asks_though_the_first_was_acknowledged():
    """Acknowledging the marker error says nothing about the frame moving later; the marker error stays quiet after."""
    watch = MocapWatch()
    watch.update({"a": Check("mocap", WARN, MARKERS)}, 0.0)
    watch.update({"a": Check("mocap", WARN, MARKERS)}, ASK_AFTER)
    watch.acknowledge()
    assert watch.update({"a": AMBER}, ASK_AFTER + 1) == {"a": FRAME_MOVED}
    assert watch.update({"a": Check("mocap", WARN, MARKERS)}, ASK_AFTER + 2) == {}


def test_each_robot_is_followed_on_its_own():
    """Acknowledging covers what asked then; a robot that starts later still asks."""
    watch = MocapWatch()
    watch.update({"a": AMBER, "b": GREEN}, 0.0)
    assert watch.update({"a": AMBER, "b": AMBER}, ASK_AFTER) == {"a": FRAME_MOVED}
    watch.acknowledge()
    assert watch.update({"a": AMBER, "b": AMBER}, 2 * ASK_AFTER) == {"b": FRAME_MOVED}
