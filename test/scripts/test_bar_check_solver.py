"""The bar check's solver: which joint is wrong, from readings between joints."""

import sys
from itertools import combinations
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))

from bar_check_solver import (  # noqa: E402
    CONTRADICTS, EITHER, FINE, NO_READINGS, SHARED, TOO_SMALL, UNDECIDED, WRONG, Reading, solve)

#: B20 as designed: joint -> its (low, high) edges along the bar, mm.
B20 = {"B22": (54.9, 114.9), "g0": (240.0, 290.0), "B21": (390.0, 450.0), "B42": (650.0, 710.0),
       "B169": (1115.3, 1175.3), "g1": (1300.0, 1350.0), "B44": (1470.0, 1530.0), "B25": (1650.0, 1710.0)}


def design(first: str, second: str, places: dict = B20) -> float:
    """The design's distance between two joints' outer edges, mm."""
    return max(places[first][1], places[second][1]) - min(places[first][0], places[second][0])


def read(first: str, second: str, wrong: dict, step: float = 1.0, places: dict = B20) -> Reading:
    """The reading a tape read to `step` gives when the joints in `wrong` are that far along the bar from design."""
    left, right = sorted((first, second), key=lambda joint: sum(places[joint]))
    true = design(first, second, places) + wrong.get(right, 0.0) - wrong.get(left, 0.0)
    return Reading(first, second, round(true / step) * step - design(first, second, places))


def test_no_readings_asks_for_one():
    """No readings asks for one."""
    assert solve(B20, []).status == NO_READINGS


def test_b20_real_readings_cannot_name_a_joint_and_ask_for_the_telling_reading():
    """B20 real readings cannot name a joint and ask for the telling reading."""
    readings = [Reading("B169", "B25", 595 - 594.702), Reading("B22", "B21", 394 - 395.098),
                Reading("g0", "g1", 0.0)]
    verdict = solve(B20, readings)
    assert verdict.status == TOO_SMALL
    assert {candidate.joints for candidate in verdict.candidates} == {("B21",), ("B22",)}
    assert set(verdict.next_pair) == {"g0", "B21"}


def test_clear_error_is_named_with_its_direction_and_confirmed_by_a_closed_triangle():
    """Clear error is named with its direction and confirmed by a closed triangle."""
    wrong = {"B21": 3.0}
    readings = [read(a, b, wrong) for a, b in (("B22", "B21"), ("g0", "B21"), ("B22", "g0"))]
    verdict = solve(B20, readings)
    assert verdict.status == WRONG and verdict.confirmed
    joint, error = verdict.candidates[0].errors[0]
    assert joint == "B21" and error == pytest.approx(3.0, abs=0.6)


def test_error_towards_the_start_has_the_opposite_sign():
    """Error towards the start has the opposite sign."""
    readings = [read(a, b, {"B21": -3.0}) for a, b in (("B22", "B21"), ("g0", "B21"), ("B22", "g0"))]
    assert solve(B20, readings).candidates[0].errors[0][1] == pytest.approx(-3.0, abs=0.6)


def test_two_readings_of_a_joint_are_only_likely_until_its_witnesses_are_read():
    """Two readings of a joint are only likely until its witnesses are read."""
    readings = [read(a, b, {"B21": 3.0}) for a, b in (("B22", "B21"), ("g0", "B21"))]
    verdict = solve(B20, readings)
    assert verdict.status == WRONG and not verdict.confirmed
    assert set(verdict.next_pair) == {"B22", "g0"}


def test_an_unrelated_good_reading_does_not_confirm():
    """An unrelated good reading does not confirm."""
    readings = [read("B22", "B21", {"B21": 3.0}), read("g0", "B21", {"B21": 3.0}), read("g0", "g1", {"B21": 3.0})]
    assert not solve(B20, readings).confirmed


def test_one_bad_reading_is_two_candidates_and_the_next_reading_tells_them_apart():
    """One bad reading is two candidates and the next reading tells them apart."""
    verdict = solve(B20, [read("B22", "B21", {"B21": 3.0})])
    assert verdict.status == UNDECIDED and verdict.can_settle
    assert {candidate.joints for candidate in verdict.candidates} == {("B21",), ("B22",)}


def test_a_loop_that_does_not_close_asks_for_the_odd_reading_again():
    """A loop that does not close asks for the odd reading again."""
    readings = [Reading("B22", "g0", 0.0), Reading("g0", "B21", 0.0), Reading("B22", "B21", 4.0),
                Reading("B22", "B42", 0.0), Reading("B21", "B42", 0.0)]
    verdict = solve(B20, readings)
    assert verdict.status == CONTRADICTS and set(verdict.suspect) == {"B22", "B21"}


def test_good_readings_are_fine_and_ask_to_reach_the_joints_not_read():
    """Good readings are fine and ask to reach the joints not read."""
    verdict = solve(B20, [Reading("B22", "g0", 0.3), Reading("g0", "B21", -0.5)])
    assert verdict.status == FINE
    assert "B42" in verdict.unchecked and verdict.next_pair is not None


def test_error_within_tolerance_needs_no_nudge():
    """Error within tolerance needs no nudge."""
    assert solve(B20, [Reading("B22", "B21", 2.5)], tolerance=3.0).status == FINE


def test_b20_error_split_over_two_gaps_is_never_called_fine():
    """The real readings: the pair is 1.1 mm short, half of it on each side of ground-0."""
    readings = [Reading("B169", "B25", 0.298), Reading("B22", "B21", -1.098), Reading("g0", "g1", 0.0),
                Reading("g0", "B21", -0.5), Reading("g0", "B22", -0.598)]
    verdict = solve(B20, readings, 0.5)
    assert verdict.status == SHARED
    assert {fix.errors for fix in verdict.fixes} == {(("B22", 1.0),), (("B21", -1.0),)}
    assert set(verdict.next_pair) == {"B21", "B42"}


def test_b20_joint_two_centimetres_out_is_named_despite_a_small_difference_elsewhere():
    """The real readings: B42 sits about 20 mm too close to B21 and ground-0, and B22 + B21 is also 1.1 mm short."""
    readings = [Reading("B169", "B25", 0.298), Reading("B22", "B21", -1.098), Reading("g0", "g1", 0.0),
                Reading("g0", "B21", -0.5), Reading("g0", "B22", -0.598), Reading("B21", "B42", -20.0),
                Reading("g0", "B42", -21.0)]
    for step in (1.0, 0.5):
        verdict = solve(B20, readings, step)
        assert verdict.status == WRONG and verdict.confirmed
        assert verdict.candidates[0].joints == ("B42",)
        assert verdict.candidates[0].errors[0][1] == pytest.approx(-20.5, abs=0.3)


@pytest.mark.parametrize("size", [5.0, 8.0, 20.0, -5.0, -20.0])
def test_a_gross_error_is_named_whatever_the_other_joints_do(size):
    """A gross error is named whatever the other joints do."""
    rng = np.random.default_rng(5)
    names = list(B20)
    named = 0
    for _ in range(200):
        culprit = names[int(rng.integers(len(names)))]
        wrong = {name: float(rng.normal(0.0, 0.5)) for name in names}
        wrong[culprit] = size
        others = [name for name in names if name != culprit]
        first, second = rng.choice(others, size=2, replace=False)
        pairs = [(culprit, str(first)), (culprit, str(second)), (str(first), str(second))]
        spare = rng.choice(len(others), size=3, replace=False)
        pairs += [(others[int(a)], others[int(b)]) for a, b in combinations(spare, 2)]
        verdict = solve(B20, [read(a, b, wrong, 0.5) for a, b in dict.fromkeys(pairs)], 0.5)
        named += verdict.status == WRONG and culprit in verdict.candidates[0].joints
    assert named >= 190


@pytest.mark.parametrize("wrong", [{"B21": 6.0, "B44": -9.0}, {"B22": 4.0, "B42": 20.0},
                                   {"g0": -5.0, "B42": 7.0, "B25": 12.0}])
def test_several_wrong_joints_at_once_are_all_named_with_their_errors(wrong):
    """Several wrong joints at once are all named with their errors."""
    readings = [read(a, b, wrong, 0.5) for a, b in combinations(B20, 2)]
    verdict = solve(B20, readings, 0.5)
    assert verdict.status == WRONG and verdict.confirmed
    found = dict(verdict.candidates[0].errors)
    assert set(found) == set(wrong)
    assert all(found[joint] == pytest.approx(error, abs=0.6) for joint, error in wrong.items())


def test_a_misread_reading_is_reported_before_any_joint_is_blamed():
    """B21 is 6 mm out by three joints, but its reading with B42 was misread: that reading is measured again first."""
    readings = [read(a, "B21", {"B21": 6.0}) for a in ("B22", "g0", "B169")]
    readings += [Reading("B21", "B42", 0.0), read("B22", "g0", {}), read("g0", "B42", {}), read("B22", "B42", {})]
    verdict = solve(B20, readings, 1.0)
    assert verdict.status == CONTRADICTS and set(verdict.suspect) == {"B21", "B42"}


def test_b34_disagreement_over_several_joints_names_no_joint_from_one_reading():
    """The real readings: B36 + B35 and B35 + B54 are both 2 mm long, and ground-1 is 1 mm off B54 and B42."""
    places = {"B36": (54.9, 114.9), "g0": (240.0, 290.0), "B35": (390.0, 450.0), "B54": (510.0, 570.0),
              "B42": (650.0, 710.0), "g1": (1300.0, 1350.0)}
    readings = [Reading("B36", "B35", 397.0 - 395.098), Reading("B35", "B54", 2.0), Reading("B54", "B42", 0.5),
                Reading("g0", "g1", 0.0), Reading("B54", "g1", -1.0), Reading("B42", "g1", -1.0)]
    for step in (1.0, 0.5):
        verdict = solve(places, readings, step)
        assert verdict.status == SHARED and not verdict.fixes
        assert set(verdict.next_pair) == {"B36", "g0"}


B34 = {"B36": (54.9, 114.9), "g0": (240.0, 290.0), "B35": (390.0, 450.0), "B54": (510.0, 570.0),
       "B42": (650.0, 710.0), "g1": (1300.0, 1350.0)}
B34_GOOD = [Reading("B35", "B54", 2.0), Reading("B54", "B42", 0.5), Reading("g0", "g1", 0.0),
            Reading("B54", "g1", -1.0), Reading("B42", "g1", -1.0), Reading("B36", "g0", -2.098)]


def test_b34_misread_by_five_mm_is_reported_as_a_loop_and_the_telling_reading_is_asked_for():
    """The real readings before the correction: B36 + B35 read 397 instead of 392."""
    verdict = solve(B34, B34_GOOD + [Reading("B36", "B35", 397.0 - 395.098)], 0.5)
    assert verdict.status == CONTRADICTS
    assert abs(verdict.closure) == pytest.approx(5.0, abs=0.3)
    assert frozenset(("B36", "B35")) in {frozenset(pair) for pair in verdict.suspects}
    assert set(verdict.next_pair) == {"g0", "B35"}


def test_b34_corrected_readings_close_and_name_b36():
    """B34 corrected readings close and name B36."""
    readings = B34_GOOD + [Reading("B36", "B35", -3.098), Reading("g0", "B35", -1.0), Reading("B36", "B54", -1.098)]
    for step in (1.0, 0.5):
        verdict = solve(B34, readings, step)
        assert verdict.status == WRONG and verdict.confirmed and verdict.candidates[0].joints == ("B36",)
        assert verdict.candidates[0].errors[0][1] == pytest.approx(2.1, abs=0.4)


def test_a_reading_with_its_sign_flipped_is_found():
    """A reading with its sign flipped is found."""
    wrong = {"B21": 2.5}
    readings = [read(a, b, wrong, 0.5) for a, b in combinations(["B22", "g0", "B21", "B42"], 2)]
    flipped = [Reading(r.first, r.second, -r.error) if {r.first, r.second} == {"g0", "B21"} else r for r in readings]
    verdict = solve(B20, flipped, 0.5)
    assert verdict.status == CONTRADICTS and set(verdict.suspect) == {"g0", "B21"}


def test_good_readings_rarely_report_an_open_loop():
    """Good readings rarely report an open loop."""
    rng = np.random.default_rng(3)
    names = list(B20)
    for step in (1.0, 0.5):
        false_alarms = 0
        for _ in range(300):
            wrong = {name: float(rng.normal(0.0, 1.0)) for name in names}
            pairs = list(combinations(names, 2))
            chosen = rng.choice(len(pairs), size=12, replace=False)
            readings = []
            for index in chosen:
                noisy = {name: value + rng.normal(0.0, 0.25 / np.sqrt(2)) for name, value in wrong.items()}
                readings.append(read(*pairs[index], noisy, step))
            false_alarms += solve(B20, readings, step).status == CONTRADICTS
        assert false_alarms <= 6


def test_b37_error_within_jig_precision_names_no_joint_but_offers_the_move():
    """The real readings: three joints, the two outer ones 1.9 mm too close, the middle one between them."""
    places = {"B21": (45.0, 95.0), "B172": (624.6, 684.6), "B35": (1085.4, 1135.4)}
    readings = [Reading("B21", "B172", -1.024), Reading("B172", "B35", -0.609), Reading("B21", "B35", -1.933)]
    for step in (1.0, 0.5):
        verdict = solve(places, readings, step)
        assert verdict.status == TOO_SMALL
        assert [fix.errors for fix in verdict.fixes] == [(("B21", 1.5),)]


B35 = {"B34": (75.0, 125.0), "B38": (366.0, 426.0), "B36": (558.4, 618.4), "B40": (690.0, 750.0),
       "J162": (925.2, 975.2), "B37": (1106.0, 1166.0), "B49": (1270.0, 1330.0)}
B35_READINGS = [("B34", "B36", -1.932), ("B36", "B40", 1.432), ("B34", "B38", -1.0), ("B38", "B36", 0.068),
                ("B34", "J162", -1.202), ("B34", "B40", 0.0), ("B36", "J162", 0.23), ("B40", "J162", -1.202),
                ("B34", "B37", -1.5), ("B34", "B49", -1.0), ("B38", "B40", 1.0), ("B40", "B37", -1.0)]


@pytest.mark.parametrize("step", [1.0, 0.5])
def test_b35_joints_many_readings_agree_on_are_named_and_no_more_readings_are_asked_for(step):
    """B35 joints many readings agree on are named and no more readings are asked for."""
    verdict = solve(B35, [Reading(*reading) for reading in B35_READINGS], step)
    assert verdict.status == WRONG and verdict.confirmed and verdict.next_pair is None
    found = dict(verdict.candidates[0].errors)
    assert set(found) == {"B34", "B40"}
    assert found["B34"] == pytest.approx(1.3, abs=0.3) and found["B40"] == pytest.approx(1.1, abs=0.3)


def test_b35_two_joints_off_alike_do_not_get_the_joint_between_them_confirmed_as_wrong():
    """The first four readings point at B36, but only because B34 and B40 are off by the same amount."""
    first = [Reading(*reading) for reading in B35_READINGS if {reading[0], reading[1]} in
             ({"B34", "B36"}, {"B36", "B40"}, {"B34", "B38"}, {"B34", "B40"})]
    verdict = solve(B35, first, 0.5)
    assert not verdict.confirmed and verdict.next_pair is not None


def test_b37_with_every_reading_taken_never_asks_for_a_reading_that_does_not_exist():
    """B37 with every reading taken never asks for a reading that does not exist."""
    places = {"B21": (45.0, 95.0), "B172": (624.6, 684.6), "B35": (1085.4, 1135.4)}
    readings = [Reading("B21", "B172", -1.624), Reading("B172", "B35", -0.809), Reading("B21", "B35", -1.433)]
    verdict = solve(places, readings, 0.5)
    assert verdict.status == WRONG and verdict.confirmed and verdict.next_pair is None
    assert verdict.candidates[0].joints == ("B21",)


def test_fine_always_means_every_reading_is_within_tolerance():
    """Fine always means every reading is within tolerance."""
    rng = np.random.default_rng(11)
    names = list(B20)
    for _ in range(400):
        pairs = list(combinations(names, 2))
        chosen = rng.choice(len(pairs), size=int(rng.integers(1, 9)), replace=False)
        wrong = {name: float(rng.normal(0.0, 0.6)) for name in names}
        readings = [read(*pairs[index], wrong, 0.5) for index in chosen]
        tolerance = float(rng.choice([0.5, 1.0, 2.0]))
        if solve(B20, readings, 0.5, tolerance).status == FINE:
            assert all(abs(reading.error) <= tolerance + 1e-6 for reading in readings)


def test_two_joint_bar_can_only_say_either():
    """Two joint bar can only say either."""
    places = {"a": (0.0, 50.0), "b": (500.0, 550.0)}
    verdict = solve(places, [Reading("a", "b", 3.0)])
    assert verdict.status == EITHER and verdict.next_pair is None


def test_three_joint_bar_names_the_odd_joint():
    """Three joint bar names the odd joint."""
    places = {"a": (0.0, 50.0), "b": (300.0, 350.0), "c": (700.0, 750.0)}
    readings = [read(x, y, {"b": 3.0}, places=places) for x, y in (("a", "b"), ("b", "c"), ("a", "c"))]
    verdict = solve(places, readings)
    assert verdict.status == WRONG and verdict.candidates[0].joints == ("b",)


@pytest.mark.parametrize("step, size, least_right", [(1.0, 2.0, 0.6), (0.5, 2.0, 0.7)])
def test_simulated_sessions_name_the_right_joint_and_rarely_a_good_one(step, size, least_right):
    """Follow the solver's own next readings on B20, with real scatter on joints and readings."""
    rng = np.random.default_rng(7)
    names, right, blamed, sessions = list(B20), 0, 0, 300
    for _ in range(sessions):
        scatter = {name: rng.normal(0.0, 0.15) for name in names}
        culprit = names[rng.integers(len(names))]
        scatter[culprit] += size * rng.choice([-1.0, 1.0])
        taken: dict = {}

        def take(first: str, second: str) -> None:
            noisy = {name: value + rng.normal(0.0, 0.25 / np.sqrt(2)) for name, value in scatter.items()}
            taken[frozenset((first, second))] = read(first, second, noisy, step)

        others = [name for name in names if name != culprit]
        take(culprit, others[int(rng.integers(len(others)))])
        verdict = solve(B20, list(taken.values()), step)
        for _step in range(6):
            if verdict.next_pair is None or frozenset(verdict.next_pair) in taken:
                break
            take(*verdict.next_pair)
            verdict = solve(B20, list(taken.values()), step)
        named = {joint for candidate in verdict.candidates for joint in candidate.joints}
        if verdict.status == WRONG:
            right += named == {culprit}
            blamed += bool(named - {culprit})
    assert right / sessions >= least_right
    assert blamed / sessions <= 0.05


def test_the_verdict_does_not_depend_on_the_order_of_the_readings():
    """The verdict does not depend on the order of the readings."""
    rng = np.random.default_rng(5)
    names, pairs = list(B20), list(combinations(B20, 2))
    for _ in range(200):
        wrong = {name: float(rng.normal(0.0, 3.0)) for name in rng.choice(names, size=3, replace=False)}
        chosen = rng.choice(len(pairs), size=int(rng.integers(3, 16)), replace=False)
        readings = [read(*pairs[index], wrong) for index in chosen]
        first = solve(B20, readings)
        for _shuffle in range(3):
            again = solve(B20, [readings[index] for index in rng.permutation(len(readings))])
            assert (again.status, again.candidates, again.loop) == (first.status, first.candidates, first.loop)


def test_two_joints_off_by_less_than_a_reading_can_show_still_name_both():
    """Two joints off by less than a reading can show still name both."""
    places = {"a": (0.0, 50.0), "b": (500.0, 550.0)}
    verdict = solve(places, [Reading("a", "b", 0.7)], tolerance=0.5)
    assert verdict.status == EITHER
    assert [candidate.errors for candidate in verdict.candidates] == [(("a", -0.7),), (("b", 0.7),)]
