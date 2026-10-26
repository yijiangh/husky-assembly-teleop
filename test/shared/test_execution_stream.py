"""Streamed Cartesian targets on a fake Cindy with her real frames: the line, base, clock, stops, and the split."""

from __future__ import annotations

import asyncio
import json
import math

import numpy as np
import pytest
from planning_fixtures import CINDY, robot_config
from plugin_fakes import ArmDriver, FakeContext, MocapDriver, fake_arm, fake_node, fake_robot, lease_to_test
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.robot_interface.arm import (CARTESIAN_COMPLIANCE_CONTROLLER,
                                                       SCALED_JOINT_TRAJECTORY_CONTROLLER)
from husky_assembly_teleop.robot_interface.base import BaseState
from husky_assembly_teleop.robot_interface.frames import HUSKY_FRAME, MOCAP_FRAME, fixed_transform
from husky_assembly_teleop.robot_interface.mocap_state import POSE_TIMEOUT
from husky_assembly_teleop.shared.execution.stream import (ARRIVAL, FK_TCP_TOL, MAX_SAMPLES, Recorder, Segment,
                                                           fk_tcp_gap, line_target, measured_tool0, plan_stream,
                                                           progress, split_rows, split_seconds, stream,
                                                           target_problem, tool0_fk)
from scaffolding_assembly_core.geometry import Pose, compose
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES

pytestmark = pytest.mark.safety

#: Cindy's arms, UR order, well away from a singularity (robot_lab's are next to the wrist singularity).
LEFT, RIGHT = [-1.54, -0.5, -2.2, -1.9, -1.2, 1.58], [1.52, -2.6, 2.2, -1.3, 1.2, -1.57]
ARMS = {"left_ur_arm": LEFT, "right_ur_arm": RIGHT}
#: Where Cindy's base (the mocap-tracked frame) stands.
BASE = Pose((3.0, 1.0, 0.0), tuple(Rotation.from_euler("z", 30.0, degrees=True).as_quat()))
#: Each tool0's line: 5 mm down, turning 1° about x.
DOWN, TURN = np.array([0.0, 0.0, -0.005]), Rotation.from_euler("x", 1.0, degrees=True)
#: Cindy's base's rigid body in the fake mocap.
MOCAP_ID = 6


def moved(pose: Pose, shift=(0.0, 0.0, 0.0), yaw_deg: float = 0.0) -> Pose:
    """`pose` shifted in the world and turned about the world z."""
    turn = Rotation.from_euler("z", yaw_deg, degrees=True)
    return Pose.from_arrays(np.add(pose.position, shift), (turn * Rotation.from_quat(pose.orientation)).as_quat())


def assert_same_pose(a: Pose, b: Pose, tolerance: float = 1e-12) -> None:
    """Positions and turns equal within `tolerance` (a quaternion and its negative are the same turn)."""
    np.testing.assert_allclose(a.position, b.position, rtol=0.0, atol=tolerance)
    sign = 1.0 if np.dot(a.orientation, b.orientation) >= 0.0 else -1.0
    np.testing.assert_allclose(a.orientation, sign * np.asarray(b.orientation), rtol=0.0, atol=tolerance)


class Lab:
    """Cindy with her URDF frames, under the compliance controller at her measured TCP, at BASE; leased to the test.

    `mocap` sees the base every tick (`MocapDriver`). `later` changes the world at a time (move the base, a late tick);
    `sent` keeps every target with the stream's elapsed time (None outside a stream) and the base it was sent on.
    """

    def __init__(self, lag: float = 0.5):
        """Build Cindy, measure her TCP by her kinematics, then switch her arms to compliance."""
        self.ctx = FakeContext(step=self.tick)
        node = fake_node(self.ctx.now, self.ctx.logs)
        config = robot_config(CINDY)
        urdf = config.model.urdf
        arms = [fake_arm(node, name, joints) for name, joints in ARMS.items()]
        for arm in arms:
            arm.urdf, arm.base_in_husky = urdf, fixed_transform(urdf, HUSKY_FRAME, f"{arm.config.name}_base_link")
        self.robot = lease_to_test(fake_robot(node, "a200-0806", arms))
        self.robot.config.model = config.model
        self.robot._husky_in_mocap_frame = fixed_transform(urdf, MOCAP_FRAME, HUSKY_FRAME)
        self.mocap = MocapDriver(self.robot, BASE, MOCAP_ID)
        self.drivers = [ArmDriver(arm, self.robot, lag=lag) for arm in arms]
        self.later: list = []
        self.elapsed: list[float] = []
        self.sent: list = []
        self.tick(self.ctx.t)
        for name, arm in self.robot.arms.items():
            arm.state.controllers.active = arm.state.controllers.requested = CARTESIAN_COMPLIANCE_CONTROLLER
            arm.targets_sent.attach(lambda target, name=name: self.sent.append(
                (self.elapsed[-1] if self.elapsed else None, name, target, self.base())))

    def base(self) -> Pose:
        """The base's measured pose."""
        state = self.robot.base.state
        return Pose.from_arrays(state.position, state.orientation)

    def move_base(self, pose: Pose) -> None:
        """The base moves to `pose`: mocap measures it there from now on."""
        self.mocap.pose = pose
        self.mocap.tick(self.ctx.t)

    def tick(self, t: float) -> None:
        """The changes due happen (they may move the clock), mocap sees the base, then every arm moves and reports."""
        while self.later and t >= self.later[0][0] - 1e-9:
            self.later.pop(0)[1]()
        self.mocap.tick(self.ctx.t)
        for driver in self.drivers:
            driver.tick(self.ctx.t)

    def segment(self, seconds: float = 1.0, wait: float = 0.5) -> Segment:
        """Each arm from its measured TCP, DOWN and turned by TURN."""
        start = {name: measured_tool0(self.robot, name) for name in ARMS}
        end = {name: Pose.from_arrays(np.add(pose.position, DOWN), (TURN * Rotation.from_quat(pose.orientation))
                                      .as_quat()) for name, pose in start.items()}
        return Segment(start, end, seconds, wait)

    def run(self, segment: Segment, keep_going=lambda _e: True, lateral=None) -> str:
        """Stream `segment` inside an event loop; every elapsed time `keep_going` was asked with is kept."""
        def going(elapsed: float) -> bool:
            self.elapsed.append(elapsed)
            return keep_going(elapsed)

        self.recorder = Recorder(self.robot, self.ctx.t, phase="test")
        return asyncio.run(stream(self.ctx, self.robot, segment, keep_going=going, lateral=lateral,
                                  recorder=self.recorder))

    def count(self, name: str) -> int:
        """How many targets `name` was sent."""
        return sum(arm == name for _e, arm, _target, _base in self.sent)


def world_target(lab: Lab, name: str, target, base: Pose) -> Pose:
    """A sent target back in the world, through the base it was sent on: B_t · H · A_s · target."""
    arm_base = compose(compose(base, lab.robot._husky_in_mocap_frame), lab.robot.arms[name].base_in_husky)
    return compose(arm_base, Pose.from_arrays(target.position, target.orientation))


# --- --- --- --- --- the line --- --- --- --- ---

def slerp(first, last, fraction: float) -> np.ndarray:
    """Quaternion slerp by its textbook formula, the short way round."""
    first, last = np.asarray(first, dtype=float), np.asarray(last, dtype=float)
    if np.dot(first, last) < 0.0:
        last = -last
    angle = math.acos(min(1.0, float(np.dot(first, last))))
    return (math.sin((1.0 - fraction) * angle) * first + math.sin(fraction * angle) * last) / math.sin(angle)


@pytest.mark.parametrize("elapsed", [0.0, 0.37 * 2.5, 2.5, 4.0])
def test_the_line_is_linear_in_time_and_turns_the_short_way(elapsed):
    """τ = min(e/T, 1): p = pS + τ(pE − pS) + Δ, R = slerp; an end as a negative quaternion turns 20°, not 340°."""
    start = Pose((0.4, -0.1, 0.3), tuple(Rotation.from_euler("xyz", [10.0, -5.0, 30.0], degrees=True).as_quat()))
    end_turn = Rotation.from_euler("xyz", [10.0, 15.0, 30.0], degrees=True).as_quat()
    end = Pose((0.41, -0.1, 0.28), tuple(-end_turn))
    lateral = np.array([0.002, -0.001, 0.0005])
    fraction = min(elapsed / 2.5, 1.0)
    target = line_target(start, end, 2.5, elapsed, lateral)
    expected = Pose.from_arrays(np.add(start.position, fraction * np.subtract(end.position, start.position)) + lateral,
                                slerp(start.orientation, end.orientation, fraction) if fraction > 0.0
                                else start.orientation)
    assert_same_pose(target, expected)


def test_plan_stream_runs_the_line_tick_by_tick_to_its_end():
    """Offline: one target per tick from the start to the end, the last one at the end."""
    lab = Lab()
    segment = lab.segment(seconds=1.0)
    ticks = plan_stream(segment, 0.05)
    assert len(ticks) == 21
    for name in ARMS:
        assert_same_pose(ticks[0][name], segment.start[name])
        assert_same_pose(ticks[-1][name], segment.end[name])
        assert_same_pose(ticks[7][name], line_target(segment.start[name], segment.end[name], 1.0, 0.35))


# --- --- --- --- --- streaming --- --- --- --- ---

def test_targets_stay_fixed_in_the_world_while_the_base_moves():
    """Each tick's target is (B_t·H·A)⁻¹·G with the base measured at that tick: a base moved 1.4 mm, 0.3° shifts it."""
    lab = Lab()
    segment = lab.segment()
    shifted = moved(BASE, (0.0012, -0.0008, 0.0), 0.3)
    lab.later.append((10.35, lambda: lab.move_base(shifted)))
    assert lab.run(segment, keep_going=lambda e: e < 1.25) == "ended"
    assert lab.count("left_ur_arm") == lab.count("right_ur_arm") == 13
    for elapsed, name, target, base in lab.sent:
        assert_same_pose(world_target(lab, name, target, base),
                         line_target(segment.start[name], segment.end[name], segment.seconds, elapsed))
    assert sum(base == shifted for *_rest, base in lab.sent) == 18


@pytest.mark.parametrize("shift, stops", [(0.0021, True), (0.0019, False)])
def test_the_stream_stops_once_the_base_moved_over_2_mm(shift, stops):
    """2.1 mm from where the base was at the start stops it before that tick's targets; 1.9 mm does not."""
    lab = Lab()
    lab.later.append((10.25, lambda: lab.move_base(moved(BASE, (shift, 0.0, 0.0)))))
    ended = lab.run(lab.segment(), keep_going=lambda e: e < 0.65)
    if stops:
        assert ended.startswith("base moved 2.1 mm") and lab.count("left_ur_arm") == lab.count("right_ur_arm") == 3
    else:
        assert ended == "ended" and lab.count("left_ur_arm") == lab.count("right_ur_arm") == 7


def test_a_late_tick_advances_the_stream_by_at_most_a_tenth_of_a_second():
    """A tick 0.6 s after the one before moves the line on by 0.1 s: it slows, it never jumps."""
    lab = Lab()
    lab.later.append((10.25, lambda: setattr(lab.ctx, "t", lab.ctx.t + 0.5)))
    assert lab.run(lab.segment(), keep_going=lambda e: e < 0.55) == "ended"
    steps = np.diff(lab.elapsed)
    assert np.allclose(steps, 0.1) and len(steps) == 6
    times = [target.update_time for _e, name, target, _base in lab.sent if name == "left_ur_arm"]
    assert np.isclose(max(np.diff(times)), 0.6)


def test_a_refused_target_ends_the_stream_and_nothing_more_is_sent():
    """The right arm leaves compliance at 10.3 s: its target is refused, the stream returns, no arm gets another."""
    lab = Lab()
    right = lab.robot.arms["right_ur_arm"]
    lab.later.append((10.25, lambda: setattr(right.state.controllers, "active", SCALED_JOINT_TRAJECTORY_CONTROLLER)))
    assert lab.run(lab.segment()).startswith("refused: right_ur_arm")
    assert (lab.count("left_ur_arm"), lab.count("right_ur_arm")) == (4, 3)
    assert "not active" in right.last_refusal.reason


def test_the_stop_test_runs_before_the_ticks_targets():
    """`keep_going` saying no at 0.3 s: nothing goes out at 0.3 s."""
    lab = Lab()
    assert lab.run(lab.segment(), keep_going=lambda e: e < 0.25) == "ended"
    assert lab.count("left_ur_arm") == lab.count("right_ur_arm") == 3
    assert max(elapsed for elapsed, *_rest in lab.sent) < 0.25


def test_the_stream_holds_the_end_until_its_time_is_up_and_the_tcp_arrives():
    """A 0.5 s line with a 0.3 s wait ends after 1.8 s (END_WAIT 1 s); the end is held, the TCP followed within 2 mm."""
    lab = Lab()
    segment = lab.segment(seconds=0.5, wait=0.3)
    assert lab.run(segment) == "time"
    assert 1.8 < lab.elapsed[-1] <= 1.9 + 1e-9 and max(elapsed for elapsed, *_rest in lab.sent) <= 1.8 + 1e-9
    *_rest, (elapsed, name, target, base) = lab.sent
    assert_same_pose(world_target(lab, name, target, base), segment.end[name])
    for each in progress(lab.robot, segment).values():
        assert each["travel_mm"] == pytest.approx(5.0) and each["remaining_mm"] < ARRIVAL * 1e3
        assert each["achieved_mm"] == pytest.approx(5.0, abs=ARRIVAL * 1e3)


def test_no_base_pose_sends_nothing():
    """Without a mocap fix there is no world to fix the targets in."""
    lab = Lab()
    segment = lab.segment()
    lab.mocap.live, lab.robot.base.state = False, BaseState()
    assert lab.run(segment) == f"base pose lost: no message for rigid body {MOCAP_ID} yet; is mocap_relay running?"
    assert not lab.sent


def test_the_stream_stops_once_mocap_loses_the_base():
    """The last fix at 10.2 s: targets go out while it is under POSE_TIMEOUT old, then the stream stops."""
    lab = Lab()
    lab.later.append((10.25, lambda: setattr(lab.mocap, "live", False)))
    assert lab.run(lab.segment()).startswith("base pose lost: no valid mocap pose")
    last = max(target.update_time for *_rest, target, _base in lab.sent)
    assert 10.2 + POSE_TIMEOUT - 0.1 - 1e-6 <= last <= 10.2 + POSE_TIMEOUT + 1e-6
    assert lab.ctx.t <= 10.2 + POSE_TIMEOUT + 0.1 + 1e-6


def test_a_line_of_no_time_is_at_its_end_at_once():
    """0 s: τ = 1 from the start, one tick offline."""
    lab = Lab()
    segment = lab.segment(seconds=0.0)
    for name in ARMS:
        assert_same_pose(line_target(segment.start[name], segment.end[name], 0.0, 0.0), segment.end[name])
    ticks = plan_stream(segment, 0.05)
    assert len(ticks) == 1 and all(ticks[0][name] == line_target(segment.start[name], segment.end[name], 0.0, 0.0)
                                   for name in ARMS)


def test_the_recorder_keeps_a_sample_per_tick_and_each_arms_peak_force():
    """Target, TCP and wrench per arm each tick, JSON values; the peak |F| per arm; only the last MAX_SAMPLES kept."""
    lab = Lab()
    lab.robot.arms["left_ur_arm"].state.wrench = np.array([3.0, 4.0, 0.0, 0.1, 0.0, 0.0])
    lab.run(lab.segment(), keep_going=lambda e: e < 0.25)
    record = json.loads(json.dumps(lab.recorder.as_record()))
    assert len(record["stream"]) == 3 and record["stream_dropped"] == 0 and record["peak_force_N"] == {
        "left_ur_arm": 5.0}
    sample = record["stream"][-1]
    assert sample["phase"] == "test" and sample["t"] == pytest.approx(0.2) and sample["motors"] == {}
    assert len(sample["arms"]["left_ur_arm"]["target"][1]) == 4 and sample["arms"]["right_ur_arm"]["wrench"] is None
    for _ in range(MAX_SAMPLES + 2):
        lab.recorder.tick(lab.ctx.t)
    assert len(lab.recorder.samples) == MAX_SAMPLES and lab.recorder.as_record()["stream_dropped"] == 5


# --- --- --- --- --- the fake arm's TCP --- --- --- --- ---

def test_the_fake_arm_follows_targets_under_compliance_and_nothing_jumps_on_the_switch_back():
    """The fake under compliance: the joints step half way to the target each tick, the TCP is their kinematics.

    Back under JTC nothing moves (the trajectory run before compliance is dropped); back under compliance, a target
    sent before the switch is not followed.
    """
    lab = Lab()
    left = lab.robot.arms["left_ur_arm"]

    def tick(active: str | None = None, times: int = 1) -> None:
        """Switch the controller if asked, then let `times` ticks pass; the TCP is always the joints' kinematics."""
        if active is not None:
            left.state.controllers.active = active
        for _ in range(times):
            lab.ctx.t += 0.1
            lab.tick(lab.ctx.t)
            distance, angle = fk_tcp_gap(lab.robot, "left_ur_arm")
            assert distance < 1e-9 and angle < 1e-6  # the angle by acos: no finer

    tick(SCALED_JOINT_TRAJECTORY_CONTROLLER)
    left.send_joint_move(left.joint_vector() + [0.0, 0.0, 0.0, 0.0, 0.0, 0.02], min_duration=1.0)
    tick(times=12)
    tick(CARTESIAN_COMPLIANCE_CONTROLLER)
    here, turn = np.array(left.state.tcp_position), np.array(left.state.tcp_orientation)
    joints = left.joint_vector()
    assert left.send_cartesian_target(here + [0.0, 0.0, 0.004], turn)
    tick()
    np.testing.assert_allclose(left.state.tcp_position, here + [0.0, 0.0, 0.002], atol=5e-5)  # one damped step
    assert np.max(np.abs(left.joint_vector() - joints)) > 1e-3
    tick(times=30)
    before, joints = np.array(left.state.tcp_position), left.joint_vector()
    np.testing.assert_allclose(before, here + [0.0, 0.0, 0.004], atol=1e-6)
    tick(SCALED_JOINT_TRAJECTORY_CONTROLLER, times=3)
    np.testing.assert_allclose(left.state.tcp_position, before, atol=1e-12)
    np.testing.assert_allclose(left.joint_vector(), joints, atol=1e-12)
    tick(CARTESIAN_COMPLIANCE_CONTROLLER)
    np.testing.assert_allclose(left.state.tcp_position, before, atol=1e-12)
    assert left.send_cartesian_target(before + [0.004, 0.0, 0.0], turn)
    tick()
    np.testing.assert_allclose(left.state.tcp_position, before + [0.002, 0.0, 0.0], atol=5e-5)
    assert left.state.tcp_update_time == pytest.approx(lab.ctx.t)


# --- --- --- --- --- frames --- --- --- --- ---

def test_fk_and_the_measured_tcp_disagreeing_by_11_mm_is_over_the_tolerance():
    """The measured TCP 11 mm off tool0 by the kinematics: the gap says 11 mm, over FK_TCP_TOL."""
    lab = Lab()
    left = lab.robot.arms["left_ur_arm"]
    left.state.tcp_position = np.add(left.state.tcp_position, [0.0, 0.011, 0.0])
    distance, angle = fk_tcp_gap(lab.robot, "left_ur_arm")
    assert distance == pytest.approx(0.011, abs=1e-9) and distance > FK_TCP_TOL[0] and angle < 1e-6
    left.state.joint_positions.clear()
    assert fk_tcp_gap(lab.robot, "left_ur_arm") is None


def test_tool0_fk_is_tool0_in_the_arms_base_link():
    """tool0 by the kinematics, seen from the arm's base_link: the URDF's root-frame poses composed."""
    urdf = robot_config(CINDY).model.urdf
    joints = {f"right_ur_arm_{name}": value for name, value in zip(UR_JOINT_NAMES, RIGHT)}
    base = link_pose(urdf, Pose(), joints, "right_ur_arm_base_link")
    assert_same_pose(compose(base, tool0_fk(urdf, "right_ur_arm", joints)),
                     link_pose(urdf, Pose(), joints, "right_ur_arm_tool0"), 1e-9)


# --- --- --- --- --- the arm's checks, offline --- --- --- --- ---

def _turned(pose_q, degrees: float) -> np.ndarray:
    """A quaternion turned by `degrees` about x."""
    return (Rotation.from_quat(pose_q) * Rotation.from_euler("x", degrees, degrees=True)).as_quat()


@pytest.mark.parametrize("case, why", [
    ("at the tcp", ""), ("4.9 cm off", ""), ("6 cm off", "too far from the TCP"),
    ("turned 6°", "turned 6.0 deg"), ("not unit", "not a unit quaternion"), ("out of reach", "out of reach"),
    ("9 mm step", ""), ("11 mm step", "jumps 11.0 mm"), ("3.5° step", "jumps 0.0 mm / 3.5 deg"),
])
def test_target_problem_refuses_what_the_arm_refuses(case, why):
    """The offline checks say what `send_cartesian_target` says, for the same TCP and target sent before."""
    lab = Lab()
    left = lab.robot.arms["left_ur_arm"]
    tcp_p, tcp_q = np.array(left.state.tcp_position), np.array(left.state.tcp_orientation)
    previous = None
    position, orientation = tcp_p.copy(), tcp_q.copy()
    if case == "4.9 cm off":
        position += [0.049, 0.0, 0.0]
    elif case == "6 cm off":
        position += [0.06, 0.0, 0.0]
    elif case == "turned 6°":
        orientation = _turned(tcp_q, 6.0)
    elif case == "not unit":
        orientation = tcp_q * 1.01
    elif case == "out of reach":
        tcp_p = position = np.array([0.0, 0.9, 0.163])
    elif case.endswith("step"):
        previous = Pose.from_arrays(tcp_p, tcp_q)
        if case == "3.5° step":
            orientation = _turned(tcp_q, 3.5)
        else:
            position += [0.0, float(case.split()[0]) * 1e-3, 0.0]
    left.state.tcp_position, left.state.tcp_orientation = tcp_p, tcp_q
    if previous is not None:
        left.state.sent_target_position, left.state.sent_target_orientation = tcp_p, tcp_q
        left.state.sent_target_time = lab.ctx.t
    problem = target_problem(previous, Pose.from_arrays(position, orientation), Pose.from_arrays(tcp_p, tcp_q))
    sent = left.send_cartesian_target(position, orientation)
    assert sent == (problem == "") and why in problem
    if not sent:
        assert problem in left.last_refusal.reason


# --- --- --- --- --- the split --- --- --- --- ---

def two_arms(row) -> list[np.ndarray]:
    """A row's two tool0 points: the first arm at x = row[0] mm, the second half as far along y."""
    return [np.array([row[0], 0.0, 0.0]) * 1e-3, np.array([0.0, row[1], 0.0]) * 1e-3]


#: A 15 mm path, unevenly spaced: the first arm at 0, 4, 9, 15 mm; the second goes half as far.
PATH = np.array([[0.0, 0.0], [4.0, 2.0], [9.0, 4.5], [15.0, 7.5]])


def test_a_split_5_mm_from_the_end_cuts_on_the_path_and_shares_the_time():
    """5 mm of 15: the cut at 10 mm, inside the third span; rigid 3.33 s, compliant 1.67 s of 5 s."""
    leading, trailing, achieved, total = split_rows(PATH, two_arms, 5.0)
    assert total == pytest.approx(15.0) and achieved == pytest.approx(5.0)
    np.testing.assert_allclose(leading, [[0.0, 0.0], [4.0, 2.0], [9.0, 4.5], [10.0, 5.0]])
    np.testing.assert_allclose(trailing, [[10.0, 5.0], [15.0, 7.5]])
    assert split_seconds(5.0, achieved, total) == pytest.approx((10.0 / 3.0, 5.0 / 3.0))


def test_a_retreat_split_counts_from_the_start():
    """5 mm from the start: the cut at 5 mm, 10 mm from the end; compliant 1.67 s, the tail 3.33 s."""
    leading, trailing, achieved, total = split_rows(PATH, two_arms, 5.0, from_start=True)
    assert achieved == pytest.approx(10.0)
    np.testing.assert_allclose(leading[-1], [5.0, 2.5])
    np.testing.assert_allclose(trailing[0], leading[-1])
    assert split_seconds(5.0, achieved, total, from_start=True) == pytest.approx((5.0 / 3.0, 10.0 / 3.0))
    assert split_seconds(5.0, 14.9, 15.0, from_start=True) == (0.5, pytest.approx(5.0 - 5.0 / 150.0))


@pytest.mark.parametrize("split_mm, from_start, parts, achieved", [
    (0.0, False, "leading", 0.0),  # all rigid
    (14.6, False, "trailing", 15.0),  # under 0.5 mm of rigid: all compliant
    (30.0, False, "trailing", 15.0),
    (0.4, True, "trailing", 15.0),  # a retreat: all rigid tail
    (15.0, True, "leading", 0.0),  # a retreat: all compliant
])
def test_a_split_at_either_end_runs_one_mode(split_mm, from_start, parts, achieved):
    """Nothing to cut: the whole path is one part, the other None."""
    leading, trailing, got, total = split_rows(PATH, two_arms, split_mm, from_start)
    whole = leading if parts == "leading" else trailing
    np.testing.assert_array_equal(whole, PATH)
    assert (trailing if parts == "leading" else leading) is None
    assert got == pytest.approx(achieved) and total == pytest.approx(15.0)


def test_a_path_under_two_rows_is_one_part():
    """One row: nothing to split, nothing to share."""
    leading, trailing, achieved, total = split_rows(PATH[:1], two_arms, 5.0)
    assert leading is None and np.array_equal(trailing, PATH[:1]) and (achieved, total) == (0.0, 0.0)
    assert split_seconds(5.0, achieved, total) == (0.0, 5.0)
