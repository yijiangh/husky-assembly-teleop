"""The insert's spiral search: its geometry and speed limit; run on a fake Cindy to a stall, a Cancel or B's end."""

from __future__ import annotations

import asyncio
import math

import numpy as np
import pytest
from test_execution_stream import ARMS, Lab, assert_same_pose, world_target

from husky_assembly_teleop.shared.execution.spiral import (SPIRAL_END_TEXT, SPIRAL_MAX_SPEED, SPIRAL_RADIUS,
                                                           SPIRAL_RETURN, Spiral, lateral_basis, run_spiral,
                                                           spiral_problem)
from husky_assembly_teleop.shared.execution.stream import END_WAIT, HOLD_FOR_STALL, Recorder, fk_tcp_gap, line_target

pytestmark = pytest.mark.safety

#: The insert's direction for the runs: down, leaning towards y.
AXIS = (0.0, 0.3, -1.0)


# --- --- --- --- --- geometry --- --- --- --- ---

@pytest.mark.parametrize("axis", [(0.0, 0.3, -1.0), (1.0, 2.0, 0.5), (0.0, 0.0, 2.0), (0.0, 0.0, -1.0)])
def test_the_basis_is_orthonormal_across_the_axis(axis):
    """u, v unit, perpendicular to each other and to the axis, (u, v, axis) right-handed; a vertical axis too."""
    u, v = lateral_basis(axis)
    along = np.asarray(axis) / np.linalg.norm(axis)
    assert np.allclose([u @ u, v @ v, u @ v, u @ along, v @ along], [1.0, 1.0, 0.0, 0.0, 0.0], atol=1e-15)
    np.testing.assert_allclose(np.cross(u, v), along, atol=1e-15)


def test_a_vertical_axis_falls_back_to_x_and_a_zero_axis_is_refused():
    """â × z is zero for a vertical axis: u = â × x instead."""
    u, v = lateral_basis((0.0, 0.0, 2.0))
    np.testing.assert_allclose(u, [0.0, 1.0, 0.0], atol=1e-15)
    np.testing.assert_allclose(v, [-1.0, 0.0, 0.0], atol=1e-15)
    with pytest.raises(ValueError, match="zero"):
        lateral_basis((0.0, 0.0, 0.0))


def test_the_offset_grows_from_nothing_to_15_mm_across_the_axis():
    """offset(0) = 0, |offset(T)| = R, held after T; always across the axis."""
    spiral = Spiral(AXIS, 20.0, 4)
    along = np.asarray(AXIS) / np.linalg.norm(AXIS)
    np.testing.assert_array_equal(spiral.offset(0.0), np.zeros(3))
    assert np.linalg.norm(spiral.offset(20.0)) == pytest.approx(SPIRAL_RADIUS, abs=1e-15)
    np.testing.assert_allclose(spiral.offset(25.0), spiral.offset(20.0), atol=0.0)
    assert max(abs(spiral.offset(e) @ along) for e in np.linspace(0.0, 20.0, 101)) < 1e-15
    assert not spiral.done(19.99) and spiral.done(20.0)
    assert spiral.text(12.6) == "spiral 63% (12.6/20.0 s, radius 9.4 mm, 2.5 of 4 turns)"


@pytest.mark.parametrize("seconds, turns, pitch", [(20.0, 4, 0.00375), (30.0, 6, 0.0025)])
def test_the_pitch_is_the_radius_over_the_turns(seconds, turns, pitch):
    """Pitch R/n: 3.75 mm at 4 turns, 2.5 mm at 6; after each whole turn the radius has grown by it."""
    spiral = Spiral(AXIS, seconds, turns)
    assert spiral.pitch == pytest.approx(pitch, abs=1e-15)
    assert np.linalg.norm(spiral.offset(seconds / turns)) == pytest.approx(pitch, abs=1e-15)
    assert spiral.as_record()["pitch_mm"] == pytest.approx(pitch * 1e3)


def test_every_20_hz_step_of_the_default_spiral_is_under_a_millimetre():
    """20 s, 4 turns: the widest step is 0.94 mm, far under the arm's 1 cm per step."""
    spiral = Spiral(AXIS, 20.0, 4)
    steps = np.linalg.norm(np.diff([spiral.offset(e) for e in np.arange(0.0, 20.0 + 1e-9, 0.05)], axis=0), axis=1)
    assert 0.9e-3 < steps.max() < 1e-3


def test_the_speed_at_full_radius_is_the_formula_and_the_offsets_own():
    """(R/T)·√(1 + (2πn)²): 18.9 mm/s at 20 s, 4 turns; the offset moves that fast at its end."""
    spiral = Spiral(AXIS, 20.0, 4)
    assert spiral.speed == pytest.approx(0.015 / 20.0 * math.sqrt(1.0 + (8.0 * math.pi) ** 2), abs=1e-12)
    assert spiral.speed == pytest.approx(0.0189, abs=5e-5)
    dt = 1e-6
    assert np.linalg.norm(spiral.offset(20.0) - spiral.offset(20.0 - dt)) / dt == pytest.approx(spiral.speed, rel=1e-4)


@pytest.mark.parametrize("seconds, turns, refused", [(7.5, 4, True), (7.6, 4, False), (18.8, 10, True),
                                                     (18.9, 10, False), (20.0, 4, False), (1.9, 1, True)])
def test_a_spiral_over_50_mm_s_is_refused(seconds, turns, refused):
    """The shortest duration for the turns is T ≥ R √(1 + (2πn)²) / 50 mm/s: 7.55 s at 4 turns, 18.85 s at 10."""
    problem = spiral_problem(seconds, turns)
    assert bool(problem) == refused
    assert (Spiral(AXIS, seconds, turns).speed > SPIRAL_MAX_SPEED) == refused


def test_the_refusal_names_the_speed_and_the_shortest_duration_rounded_up():
    """5 s at 4 turns runs at 75 mm/s; 7.6 s is the shortest a tenth of a second allows."""
    assert spiral_problem(5.0, 4) == "spiral too fast: 75 mm/s at 15 mm, limit 50; 4 turns need ≥ 7.6 s"
    assert spiral_problem(3.0, 6).endswith("6 turns need ≥ 11.4 s") and not spiral_problem(11.4, 6)


# --- --- --- --- --- running it --- --- --- --- ---

class Gate:
    """Every mating screw stalls at ROS time `at` (never by default)."""

    text = "left_ur_arm joint screw"

    def __init__(self, lab: Lab, at: float = math.inf):
        """Stall at `at`."""
        self.lab, self.at = lab, at

    def all_stalled(self) -> bool:
        """Whether the time has come."""
        return self.lab.ctx.t >= self.at - 1e-9


class Operator:
    """Answers the prompt `answer` (None: never) `after` seconds after it opened; keeps what it was asked."""

    def __init__(self, lab: Lab, answer: bool | None = None, after: float = 1.0):
        """Answer `answer` after `after` s."""
        self.lab, self.answer, self.after = lab, answer, after
        self.opened, self.closed, self.answered = [], [], None

    def open(self, text: str, warn: bool = False) -> int:
        """Keep the prompt and when it opened."""
        self.opened.append((text, warn, self.lab.ctx.t))
        return len(self.opened)

    def poll(self, handle: int) -> bool | None:
        """The answer once due."""
        if self.answer is None or self.lab.ctx.t < self.opened[-1][2] + self.after - 1e-9:
            return None
        self.answered = self.lab.ctx.t
        return self.answer

    def close(self, handle: int) -> None:
        """Keep that it was closed unanswered."""
        self.closed.append(handle)


def run(lab: Lab, spiral: Spiral, gate: Gate, operator: Operator) -> tuple[str, list]:
    """Run the spiral from each arm's TCP down 5 mm; returns how it ended and (phase, arm, target) per target sent."""
    segment = lab.segment(seconds=99.0, wait=99.0)
    lab.recorder = Recorder(lab.robot, lab.ctx.t)
    sent = []
    for name, arm in lab.robot.arms.items():
        arm.targets_sent.attach(lambda target, name=name: sent.append((lab.recorder.phase, name, target)))
    ended = asyncio.run(run_spiral(lab.ctx, lab.robot, segment, spiral, gate, operator, lab.recorder))
    lab.spiral_segment = segment
    return ended, sent


def test_a_stall_during_the_spiral_ends_it_and_says_where():
    """A stall 1 s in: "stalled", the log says where the spiral stood; no prompt."""
    lab = Lab()
    spiral = Spiral(AXIS, 2.0, 1)
    operator = Operator(lab)
    ended, sent = run(lab, spiral, Gate(lab, at=11.0), operator)
    assert ended == "stalled" and not operator.opened
    assert len(sent) == 20 and {phase for phase, *_rest in sent} == {"spiral"}
    assert f"left_ur_arm joint screw STALLED at {spiral.text(1.0)}" in lab.ctx.lines("info")[-1]


def test_the_spiral_targets_are_the_line_plus_the_offset_on_both_arms():
    """Each tick: the line over the spiral's seconds plus the same offset for both arms, fixed in the world."""
    lab = Lab()
    spiral = Spiral(AXIS, 2.0, 1)
    ended, sent = run(lab, spiral, Gate(lab, at=11.0), Operator(lab))
    segment = lab.spiral_segment
    for _phase, name, target in sent:
        elapsed = target.update_time - 10.0
        expected = line_target(segment.start[name], segment.end[name], 2.0, elapsed, spiral.offset(elapsed))
        assert_same_pose(world_target(lab, name, target, lab.base()), expected, 1e-9)


def test_the_spiral_complete_opens_one_warning_and_cancel_ends_it_before_the_next_send():
    """At f = 1 one warning prompt, the end held meanwhile; Cancel 1 s later: "cancelled", nothing sent on that tick.

    The prompt is not closed again: an answered prompt closes itself.
    """
    lab = Lab()
    operator = Operator(lab, answer=False, after=1.0)
    ended, sent = run(lab, Spiral(AXIS, 2.0, 1), Gate(lab), operator)
    (text, warn, opened), = operator.opened
    assert ended == "cancelled" and text == SPIRAL_END_TEXT and warn and 12.0 - 1e-9 <= opened <= 12.1 + 1e-9
    assert max(target.update_time for *_rest, target in sent) < operator.answered - 1e-6
    assert not operator.closed and "Cancel at the spiral's end" in lab.ctx.lines("warn")[-1]
    held = [target.position for _phase, name, target in sent if name == "left_ur_arm"][-10:]
    np.testing.assert_allclose(held, [held[0]] * 10, atol=1e-12)


def test_a_stall_while_the_prompt_is_open_closes_it():
    """The screw stalls while the operator reads: "stalled", the prompt taken away."""
    lab = Lab()
    operator = Operator(lab)
    ended, _sent = run(lab, Spiral(AXIS, 2.0, 1), Gate(lab, at=12.5), operator)
    assert ended == "stalled" and len(operator.opened) == 1 and operator.closed == [1]


def test_confirm_returns_to_the_line_end_from_the_spirals_end_then_holds_until_the_ceiling():
    """B's first target is A's last; B ends the line at its end in 2 s, then holds; no stall in 33 s: "no_stall"."""
    lab = Lab()
    spiral = Spiral(AXIS, 2.0, 1)
    ended, sent = run(lab, spiral, Gate(lab), Operator(lab, answer=True, after=1.0))
    assert ended == "no_stall"
    for name in lab.robot.arms:
        mine = [(phase, target) for phase, arm, target in sent if arm == name]
        first_b = next(i for i, (phase, _target) in enumerate(mine) if phase == "spiral return")
        np.testing.assert_allclose(mine[first_b][1].position, mine[first_b - 1][1].position, atol=1e-12)
        np.testing.assert_allclose(mine[first_b][1].orientation, mine[first_b - 1][1].orientation, atol=1e-12)
        b_started, last = mine[first_b][1].update_time, mine[-1][1]
        assert_same_pose(world_target(lab, name, last, lab.base()), lab.spiral_segment.end[name], 1e-12)
        assert SPIRAL_RETURN + HOLD_FOR_STALL + END_WAIT - 0.05 < lab.ctx.t - b_started < 33.15
    assert "no stall within 30 s" in lab.ctx.lines("warn")[-1]
    for name, joints in ARMS.items():
        assert fk_tcp_gap(lab.robot, name)[0] < 1e-9
        assert np.max(np.abs(lab.robot.arms[name].joint_vector() - joints)) < 0.2  # the joints followed, smoothly


def test_a_stall_after_confirm_ends_the_return_stalled():
    """Confirm, then the screw stalls 3 s into B: "stalled"."""
    lab = Lab()
    ended, sent = run(lab, Spiral(AXIS, 2.0, 1), Gate(lab, at=16.0), Operator(lab, answer=True, after=1.0))
    assert ended == "stalled" and sent[-1][0] == "spiral return"
    assert "STALLED after the return" in lab.ctx.lines("info")[-1]
