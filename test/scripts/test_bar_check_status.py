"""The bar check's bar status: green when every reading is good, yellow when only the triangles are."""

import sys
from pathlib import Path

import pytest

pytest.importorskip("tkinter")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))

from bar_check import (  # noqa: E402
    ACCEPTED, BAD, GOOD, NO_TRIANGLE, PARTIAL, TRIANGLE_WAITING, Bar, Joint, MeasureLog, Pair, in_triangle,
    required_pairs, waiting_state)

JOINTS = tuple(Joint(name, "T20/Male", low, low + 50.0, "B1") for name, low in
               (("a", 0.0), ("b", 200.0), ("c", 500.0), ("d", 900.0)))
BAR = Bar("B1", 1000.0, JOINTS, (Pair("a", "b", 3), Pair("c", "d", 2), Pair("a", "c", 4)))
TRIANGLE, GROUND, OTHER = ("a", "b"), ("c", "d"), ("a", "c")


def status(tmp_path: Path, *readings: tuple) -> str | None:
    """The bar's status after these (joints, error in mm) readings, with a tolerance of 1 mm."""
    log = MeasureLog(tmp_path / "log.csv", "process", "hash")
    for joints, error in readings:
        first, second = BAR.joint(joints[0]), BAR.joint(joints[1])
        log.add(BAR, joints, 3, max(first.high, second.high) - min(first.low, second.low) + error, 1.0)
    return log.state(BAR, 1.0)[2]


def test_triangles_are_the_required_pairs():
    """Triangles are the required pairs."""
    assert required_pairs(BAR) == [Pair("a", "b", 3)]


@pytest.mark.parametrize("readings, expected", [
    (((TRIANGLE, 0.5),), GOOD),
    (((TRIANGLE, 0.5), (OTHER, 0.9), (GROUND, -1.0)), GOOD),
    (((TRIANGLE, 0.5), (OTHER, 1.1)), ACCEPTED),
    (((TRIANGLE, 0.5), (OTHER, 4.9)), ACCEPTED),
    (((TRIANGLE, 0.5), (OTHER, 5.0), (GROUND, -5.0)), ACCEPTED),
    (((TRIANGLE, 0.5), (OTHER, 5.1)), BAD),
    (((TRIANGLE, 0.5), (GROUND, -6.0)), BAD),
    (((TRIANGLE, 1.5), (OTHER, 0.0)), BAD),
    (((OTHER, 3.0),), PARTIAL),
    (((OTHER, 0.5),), PARTIAL),
    (((OTHER, 6.0),), BAD),
])
def test_bar_status(tmp_path, readings, expected):
    """A bar's status follows from its readings."""
    assert status(tmp_path, *readings) == expected


def test_a_bar_without_pairs_is_bad_on_any_reading_out_of_tolerance(tmp_path):
    """A bar without pairs is bad on any reading out of tolerance."""
    bar = Bar("B4", 1000.0, JOINTS, ())
    log = MeasureLog(tmp_path / "log.csv", "process", "hash")
    log.add(bar, ("a", "b"), 0, 250.0 + 4.9, 1.0)
    assert log.state(bar, 1.0)[2] == BAD


def test_bars_waiting_to_be_measured_are_told_apart_by_their_triangles():
    """Bars waiting to be measured are told apart by their triangles."""
    assert waiting_state(BAR) == TRIANGLE_WAITING and in_triangle(BAR)
    assert waiting_state(Bar("B2", 1000.0, JOINTS, (Pair("c", "d", 2), Pair("a", "c", 4)))) == NO_TRIANGLE
    assert waiting_state(Bar("B3", 1000.0, JOINTS, ())) == NO_TRIANGLE


def test_a_bar_without_readings_has_no_status(tmp_path):
    """A bar without readings has no status."""
    assert status(tmp_path) is None
