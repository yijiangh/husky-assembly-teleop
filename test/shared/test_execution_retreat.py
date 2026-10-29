"""The retreat on a fake Cindy with her real frames and two scaffolding tools: its steps, its endings, its checks."""

from __future__ import annotations

import asyncio
import json
import math
from dataclasses import replace

import numpy as np
import pytest
from planning_fixtures import CINDY, robot_config
from plugin_fakes import (ArmDriver, FakeContext, ScaffoldingDriver, fake_arm, fake_node, fake_robot,
                          fake_scaffolding, lease_to_test)
from robot_lab import LEFT, RIGHT
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.robot_interface.arm import CARTESIAN_COMPLIANCE_CONTROLLER as COMPLIANCE
from husky_assembly_teleop.robot_interface.arm import SCALED_JOINT_TRAJECTORY_CONTROLLER as JOINT
from husky_assembly_teleop.robot_interface.end_effectors import ScaffoldingV3
from husky_assembly_teleop.robot_interface.frames import HUSKY_FRAME, MOCAP_FRAME, fixed_transform
from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names
from husky_assembly_teleop.shared.assembly_planning.route_result import RouteResult
from husky_assembly_teleop.shared.execution.retreat import (CHECK_ARRIVAL, CHECK_BACKTRACK, CHECK_FORCE,
                                                            CHECK_JOINT_TRAVEL, CHECK_LENGTH, CHECK_SPEED,
                                                            CHECK_SPREAD, CHECK_START, CHECK_TURN, plan_retreat,
                                                            retreat_checks, run_retreat)
from husky_assembly_teleop.shared.execution.stream import (ARRIVAL, DRY_RUN, END_WAIT, FAILED, REFUSED, RETREATED,
                                                           arm_base_in_world, line_target, measured_tool0,
                                                           tool0_fk)
from scaffolding_assembly_core.geometry import Pose, compose
from scaffolding_assembly_core.process.solutions import MovementResult, Trajectory

pytestmark = pytest.mark.safety

ARMS = {"left_ur_arm": LEFT, "right_ur_arm": RIGHT}
NAMES = arm_joint_names("left_ur_arm") + arm_joint_names("right_ur_arm")
#: Each arm's retreat: the elbow opens, shoulder and wrist keep tool0's turn; about 24 mm, nearly straight.
DELTA = np.array([0.0, -0.03, 0.06, -0.03, 0.0, 0.0] * 2)
START = np.array(LEFT + RIGHT)
END = START + DELTA
PATH = np.linspace(START, END, 21)
#: Where Cindy's base (the mocap-tracked frame) stands.
BASE = Pose((3.0, 1.0, 0.0), tuple(Rotation.from_euler("z", 30.0, degrees=True).as_quat()))
#: Cindy's base's rigid body in the fake mocap.
MOCAP_ID = 6
#: The run's steps, each kind of command in the order it is first sent.
STEPS = ["STOP", "LOOSEN M1", "zero F/T", "switch to compliance", "zero wrench", "target", "hold", "switch to JTC",
         "replan", "check", "trajectory"]


class Lab:
    """Cindy under joint control with her URDF frames and her two tools clamped on a bar, at BASE; leased to the test.

    Mocap (rigid body MOCAP_ID) sees the base every tick while `mocap` is on. Every command sent goes into `ctx.logs`
    as ("event", "<arm>: <what>"), in order with the log lines; `targets` keeps each arm's Cartesian targets, holds
    left out. `on_tick` runs every tick after the drivers.
    """

    def __init__(self, lag: float = 0.5, **tool):
        """Build Cindy and her tools; `tool` options go to both ScaffoldingDrivers."""
        self.ctx = FakeContext(step=self.tick)
        node = fake_node(self.ctx.now, self.ctx.logs)
        config = robot_config(CINDY)
        self.urdf = config.model.urdf
        self.tools = {name: fake_scaffolding(node, f"/a200_0806/{name.split('_')[0]}_gripper") for name in ARMS}
        arms = [fake_arm(node, name, joints, end_effector=self.tools[name]) for name, joints in ARMS.items()]
        for arm in arms:
            arm.urdf = self.urdf
            arm.base_in_husky = fixed_transform(self.urdf, HUSKY_FRAME, f"{arm.config.name}_base_link")
            arm.state.wrench = np.zeros(6)
        self.robot = lease_to_test(fake_robot(node, "a200-0806", arms))
        self.robot.config.model = config.model
        self.robot._husky_in_mocap_frame = fixed_transform(self.urdf, MOCAP_FRAME, HUSKY_FRAME)
        self.robot.base.mocap_id, self.mocap = MOCAP_ID, True
        state = self.robot.base.state
        state.position, state.orientation = np.array(BASE.position), np.array(BASE.orientation)
        self.drivers = [ArmDriver(arm, self.robot, lag=lag) for arm in arms]
        options = {"motor": ScaffoldingV3.STALLED, "screw": ScaffoldingV3.STALLED, **tool}
        self.drivers += [ScaffoldingDriver(each, **options) for each in self.tools.values()]
        self.on_tick: list = []
        self.targets: dict[str, list] = {name: [] for name in ARMS}
        self.replans: list[np.ndarray] = []
        self.tick(self.ctx.t)
        for name, arm in self.robot.arms.items():
            self._spy(name, arm)

    def tick(self, t: float) -> None:
        """Mocap sees the base while `mocap` is on, every arm and tool moves and reports; then the test's changes."""
        base = self.robot.base.state
        if self.mocap:
            base.tracked = base.tracking_valid = True
            base.last_update_time = base.last_fix_time = t
            base.marker_errors.append((t, 0.0))
        for driver in self.drivers:
            driver.tick(t)
        for change in self.on_tick:
            change()

    def event(self, what: str) -> None:
        """Keep a command sent, in order with the log lines."""
        self.ctx.logs.append(("event", what))

    def _spy(self, name: str, arm) -> None:
        """Note every command sent to one arm and its tool."""
        tool, publish, switch = self.tools[name], self.tools[name]._command.publish, arm.controllers.switch

        def command(message) -> None:
            publish(message)
            verb = "STOP" if message.direction == 0 else "TIGHTEN" if message.direction > 0 else "LOOSEN"
            self.event(f"{name}: {verb}" + ("" if message.direction == 0 else f" M{message.motor}"))

        def switched(controller: str, unguarded: bool = False) -> bool:
            self.event(f"{name}: switch to {'compliance' if controller == COMPLIANCE else 'JTC'}")
            return switch(controller, unguarded)

        def target(sent) -> None:
            self.event(f"{name}: {'hold' if sent.hold else 'target'}")
            if not sent.hold:
                self.targets[name].append(sent)

        tool._command.publish, arm.controllers.switch = command, switched
        arm._zero_ft.call_async = lambda _request: self.event(f"{name}: zero F/T")
        arm.targets_sent.attach(target)
        arm.wrenches_sent.attach(lambda _wrench: self.event(f"{name}: zero wrench"))
        arm.trajectories_sent.attach(lambda sent: self.event(f"{name}: {'hold' if sent.hold else 'trajectory'}"))

    def events(self) -> list[str]:
        """Every command sent, "replan", and "check" for each check line, in order."""
        return [line if kind == "event" else "check" for kind, line in self.ctx.logs
                if kind == "event" or ": check " in line]

    def steps(self) -> list[str]:
        """Each kind of command, in the order it was first sent."""
        return list(dict.fromkeys(event.split(": ", 1)[-1] for event in self.events()))

    def joints(self) -> np.ndarray:
        """Both arms' measured joints, NAMES order."""
        return np.concatenate([self.robot.arms[name].joint_vector() for name in ARMS])

    def goals(self) -> dict[str, Pose]:
        """The route's goals: each tool0 at the path's end in the world, by process flange id."""
        joints = dict(zip(NAMES, END))
        return {f"robots/cindy/{name}_tool0": compose(arm_base_in_world(self.robot, name),
                                                      tool0_fk(self.urdf, name, joints)) for name in ARMS}

    def plan(self, split_mm: float = 5.0, traj_time: float = 5.0):
        """The retreat of PATH."""
        return plan_retreat(self.robot, NAMES, PATH, self.goals(), split_mm=split_mm, traj_time=traj_time)

    def replan(self, route: str = "solved"):
        """A planner answering at once: the rest of PATH from the live joints in 21 rows; or "failed", or "none"."""
        async def plan() -> RouteResult | None:
            self.event("replan")
            if route == "none":
                return None
            if route == "failed":
                return RouteResult(MovementResult("failed", "no linear path: collision"), None)
            rows = np.linspace(self.joints(), END, 21)
            self.replans.append(rows)
            return RouteResult(MovementResult("solved", trajectory=Trajectory(
                "robots/cindy", tuple(NAMES), tuple(map(tuple, rows.tolist())))), None, "stand-in", 1, 0.0, 21,
                goals=self.goals())
        return plan

    def run(self, plan, route: str = "solved", dry_run: bool = False):
        """Run the retreat to its end."""
        return asyncio.run(run_retreat(self.ctx, self.robot, plan, self.tools, replan=self.replan(route),
                                       dry_run=dry_run))

    def world_targets(self, name: str) -> list[Pose]:
        """An arm's targets back in the world, through the base (which stands still)."""
        base = arm_base_in_world(self.robot, name)
        return [compose(base, Pose.from_arrays(sent.position, sent.orientation)) for sent in self.targets[name]]


def assert_same_pose(a: Pose, b: Pose, tolerance: float = 1e-9) -> None:
    """Positions and turns equal within `tolerance` (a quaternion and its negative are the same turn)."""
    np.testing.assert_allclose(a.position, b.position, rtol=0.0, atol=tolerance)
    sign = 1.0 if np.dot(a.orientation, b.orientation) >= 0.0 else -1.0
    np.testing.assert_allclose(a.orientation, sign * np.asarray(b.orientation), rtol=0.0, atol=tolerance)


# --- --- --- --- --- the plan --- --- --- --- ---

def test_the_plan_cuts_the_break_out_from_the_start_and_shares_the_time():
    """Split 5 of ~24 mm: break-out 5 mm in T·c/D s, the tail the rest; its end is tool0 at a row on the path."""
    lab = Lab()
    plan = lab.plan()
    assert plan.arms == ("left_ur_arm", "right_ur_arm") and plan.compliant_mm == pytest.approx(5.0, abs=1e-3)
    total = plan.total_mm
    assert 24.0 < total < 24.5 and plan.achieved_mm == pytest.approx(total - 5.0)
    assert plan.compliant_seconds == pytest.approx(5.0 * plan.compliant_mm / total)
    assert plan.tail_seconds == pytest.approx(5.0 - plan.compliant_seconds)

    # * The cut is one row of the straight joint path, START + s DELTA: found on the left arm, it fits the right one.
    def tool0(name: str, s: float) -> Pose:
        return tool0_fk(lab.urdf, name, dict(zip(NAMES, START + s * DELTA)))

    def gone(name: str, pose: Pose) -> float:
        return float(np.linalg.norm(np.subtract(pose.position, tool0(name, 0.0).position)))

    assert max(gone(name, plan.ends[name]) for name in ARMS) == pytest.approx(0.005, abs=1e-5)
    low, high = 0.0, 1.0
    for _ in range(40):
        middle = (low + high) / 2.0
        low, high = (middle, high) if gone("left_ur_arm", tool0("left_ur_arm", middle)) < gone(
            "left_ur_arm", plan.ends["left_ur_arm"]) else (low, middle)
    for name in ARMS:
        assert_same_pose(tool0(name, low), plan.ends[name], 1e-8)
    assert set(plan.goals) == set(ARMS)
    assert plan.text == ("RETREAT: the M1 screws of left_ur_arm + right_ur_arm OPEN; force sensors zeroed; COMPLIANT "
                         f"break-out 5.0 mm ({plan.compliant_seconds:.1f} s); joint control; the rest replanned from "
                         f"where the tools are and run at once ({plan.tail_seconds:.1f} s). Watch both tools let go "
                         "and back off along their lines.")
    assert json.loads(json.dumps(plan.as_record()))["compliant_mm"] == pytest.approx(5.0, abs=1e-3)


def test_a_plan_without_a_goal_for_a_moving_arm_is_refused():
    """The tail is replanned to the goals: a moving arm without one can't retreat."""
    lab = Lab()
    goals = {flange: pose for flange, pose in lab.goals().items() if "right" not in flange}
    problem = plan_retreat(lab.robot, NAMES, PATH, goals, split_mm=5.0, traj_time=5.0)
    assert problem == "no retreat goal for right_ur_arm in the plan: Plan again"
    one = plan_retreat(lab.robot, NAMES[:6], PATH[:, :6], goals, split_mm=5.0, traj_time=5.0)
    assert one.arms == ("left_ur_arm",) and one.text.endswith("Watch the tool let go and back off along its line.")


@pytest.mark.parametrize("split_mm, compliant, tail", [(0.4, False, True), (30.0, True, False)])
def test_a_split_at_either_end_leaves_out_the_break_out_or_the_tail(split_mm, compliant, tail):
    """Split 0.4 mm: no break-out, the whole path rigid. Split past the end: all compliant, no tail."""
    plan = Lab().plan(split_mm)
    assert bool(plan.ends) == compliant and bool(plan.compliant_seconds) == compliant
    assert bool(plan.tail_seconds) == tail
    assert ("COMPLIANT" in plan.text) == compliant and ("replanned" in plan.text) == tail


# --- --- --- --- --- the run --- --- --- --- ---

def test_the_retreat_opens_breaks_out_comes_back_replans_checks_and_runs_its_tail():
    """STOP and LOOSEN M1 on both, F/T zero both, compliance, the stream to FK(q_split), JTC, replan, checks, tail.

    The stream starts at the measured TCP; the tail starts where the break-out left the arms.
    """
    lab = Lab()
    plan = lab.plan()
    starts = {name: measured_tool0(lab.robot, name) for name in ARMS}
    outcome = lab.run(plan)
    assert outcome.status == RETREATED and outcome.ok and outcome.reason == "retreated; every check passed"
    assert lab.steps() == STEPS
    events = lab.events()
    for name in ARMS:
        assert [events.count(f"{name}: {what}") for what in ("LOOSEN M1", "zero F/T", "switch to compliance")] == [
            1, 1, 1]
    sent = [i for i, event in enumerate(events) if event.endswith(": target")]
    compliance = [i for i, event in enumerate(events) if event.endswith("switch to compliance")]
    zeros = [i for i, event in enumerate(events) if event.endswith("zero F/T")]
    assert max(zeros) < min(compliance) and max(compliance) < sent[0] < sent[-1] < events.index(
        "left_ur_arm: switch to JTC")
    assert events[-2:] == ["left_ur_arm: STOP", "right_ur_arm: STOP"]
    # * One target a tick along the line from the measured TCP to tool0 at the cut, until the tools arrived.
    for name in ARMS:
        end = compose(arm_base_in_world(lab.robot, name), plan.ends[name])
        for k, target in enumerate(lab.world_targets(name)):
            assert_same_pose(target, line_target(starts[name], end, plan.compliant_seconds, 0.1 * k))
    # * The tail was planned from the live joints after the break-out, where its compliance left the tools.
    tail = lab.replans[-1]
    assert np.max(np.abs(tail[0] - START)) > 1e-3
    for name, (_start, end) in zip(ARMS, ((0, 6), (6, 12))):
        here = tool0_fk(lab.urdf, name, dict(zip(NAMES[_start:end], tail[0][_start:end])))
        assert np.linalg.norm(np.subtract(here.position, plan.ends[name].position)) < CHECK_ARRIVAL
    np.testing.assert_allclose(lab.joints(), END, atol=math.radians(0.5))
    record = json.loads(json.dumps(outcome.record))
    assert record["status"] == RETREATED and len(record["checks"]) == 8
    assert all(line.startswith("PASS") for line in record["checks"])
    assert record["tail"]["outcome"]["status"] == "ARRIVED"
    assert [each["name"] for each in record["phases"]] == ["open", "force zero", "break-out", "joint control",
                                                           "replan", "tail"]
    assert record["switch"]["ok"] and record["switch"]["back_ok"] and set(record["ft_zero"]) == set(ARMS)
    assert {each["phase"] for each in record["stream"]} == {"break-out", "tail"}
    assert all(each["remaining_mm"] <= ARRIVAL * 1e3 for each in record["end_error_mm"].values())


@pytest.mark.parametrize("lag", [0.5, 0.0])
def test_the_break_out_stops_once_every_tool_arrived_else_at_its_hard_end(lag):
    """Tools that follow end it within a tick of its time; tools that never move, at T_c + 1 s.

    Never arriving fails a check, and the tail runs all the same: the checks only report.
    """
    lab = Lab(lag=lag)
    plan = lab.plan()
    outcome = lab.run(plan)
    assert outcome.status == RETREATED
    streamed = 0.1 * (sum(each["phase"] == "break-out" for each in outcome.record["stream"]) - 1)
    breakout = next(line for line in outcome.record["checks"] if "break-out" in line)
    if lag:
        assert plan.compliant_seconds - 0.1 <= streamed < plan.compliant_seconds + 0.3
        assert breakout.startswith("PASS")
    else:
        assert plan.compliant_seconds + END_WAIT - 0.1 <= streamed <= plan.compliant_seconds + END_WAIT
        assert breakout.startswith("FAIL break-out: worst tool0 5.0 mm short") and "1 check(s) FAILED" in outcome.reason
        assert any("did not reach the break-out's end" in line for line in lab.ctx.lines("warn"))
    assert "trajectory" in lab.steps()


def test_the_record_keeps_the_furthest_the_base_moved():
    """The base moves 1.5 mm during the break-out: the record says so; under BASE_DRIFT, the retreat goes on."""
    lab = Lab()
    state = lab.robot.base.state

    def shift() -> None:
        """Once the first target went out, mocap sees the base 1.5 mm along x."""
        if lab.targets["left_ur_arm"]:
            state.position = np.array(BASE.position) + [0.0015, 0.0, 0.0]

    lab.on_tick.append(shift)
    outcome = lab.run(lab.plan())
    assert outcome.status == RETREATED and outcome.record["base_drift_max_mm"] == pytest.approx(1.5, abs=1e-6)


@pytest.mark.parametrize("route, why", [("none", "no answer"), ("failed", "no linear path: collision")])
def test_a_failed_replan_ends_failed_with_no_tail(route, why):
    """No tail from the planner (A4): FAILED once the arms are back under joint control; no check, no trajectory."""
    lab = Lab()
    outcome = lab.run(lab.plan(), route)
    assert outcome.status == FAILED and why in outcome.reason and "hold where the break-out left them" in outcome.reason
    assert lab.steps() == STEPS[:STEPS.index("replan") + 1]
    assert lab.events()[-2:] == ["left_ur_arm: STOP", "right_ur_arm: STOP"]
    assert all(arm.controllers.is_active(JOINT) for arm in lab.robot.arms.values())


@pytest.mark.parametrize("split_mm, steps", [
    (0.0, ["STOP", "LOOSEN M1", "replan", "check", "trajectory"]),
    (30.0, ["STOP", "LOOSEN M1", "zero F/T", "switch to compliance", "zero wrench", "target", "hold", "switch to JTC"]),
])
def test_without_a_break_out_or_a_tail_those_steps_are_left_out(split_mm, steps):
    """Split 0: open, then the whole path replanned and run, never compliant. Split past the end: no replan."""
    lab = Lab()
    plan = lab.plan(split_mm)
    outcome = lab.run(plan)
    assert outcome.status == RETREATED and lab.steps() == steps
    if not split_mm:
        assert not any("break-out" in line for line in outcome.record["checks"])
    else:
        assert outcome.reason == "the break-out ran to the retreat's end: no tail"
        for name in ARMS:
            goal = lab.goals()[f"robots/cindy/{name}_tool0"]
            assert_same_pose(lab.world_targets(name)[-1], goal)


@pytest.mark.parametrize("case, why", [
    ("compliant", "left_ur_arm is not under scaled_joint_trajectory_controller"),
    ("urdf", "the URDF breaks the stock UR frames"),
    ("test mode", "Cartesian test mode"),
    ("frames", "the measured TCP is 11.0 mm"),
    ("tool", "right_ur_arm: no tool status"),
    ("mocap", "base pose lost"),
])
def test_a_refused_retreat_sends_nothing(case, why):
    """Each precheck that fails: REFUSED, no command to an arm or a tool, no replan."""
    lab = Lab()
    plan = lab.plan()
    left = lab.robot.arms["left_ur_arm"]
    if case == "compliant":
        left.state.controllers.active = left.controllers.running = COMPLIANCE
    elif case == "urdf":
        left.urdf_problem = "base_link is turned"
    elif case == "test mode":
        left.config = replace(left.config, cartesian_test_mode=True)
    elif case == "frames":
        left.state.tcp_position = np.add(left.state.tcp_position, [0.0, 0.011, 0.0])
    elif case == "tool":
        lab.tools["right_ur_arm"].state.last_update_time = lab.ctx.t - 1.5
    else:
        lab.robot.base.state.marker_errors.clear()
    outcome = lab.run(plan)
    assert outcome.status == REFUSED and why in outcome.reason and lab.events() == []


def test_a_dry_run_sends_nothing():
    """Every break-out target checked offline; nothing sent, nothing replanned."""
    lab = Lab()
    outcome = lab.run(lab.plan(), dry_run=True)
    assert outcome.status == DRY_RUN and not outcome.ok and lab.events() == []
    assert any(line.startswith("DRY RUN a200-0806: retreat") for line in lab.ctx.lines("warn"))


def test_a_cancel_during_the_break_out_stops_the_tools_holds_and_restores_joint_control():
    """Cancelled after three targets: STOP both tools, hold each arm at its TCP, JTC; then CancelledError."""
    lab = Lab()
    plan = lab.plan()
    cut = []

    async def scenario():
        task = asyncio.ensure_future(run_retreat(lab.ctx, lab.robot, plan, lab.tools, replan=lab.replan(),
                                                 dry_run=False))
        while sum(event.endswith(": target") for event in lab.events()) < 6 and not task.done():
            await asyncio.sleep(0)
        cut.append(len(lab.events()))
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(scenario())
    after = [event for event in lab.events()[cut[0]:] if not event.endswith(": target")]
    assert not any(event.endswith(": target") for event in lab.events()[cut[0] + 2:])
    assert after == ["left_ur_arm: STOP", "right_ur_arm: STOP", "left_ur_arm: zero wrench", "left_ur_arm: hold",
                     "left_ur_arm: switch to JTC", "right_ur_arm: zero wrench", "right_ur_arm: hold",
                     "right_ur_arm: switch to JTC"]
    assert all(arm.controllers.is_active(JOINT) for arm in lab.robot.arms.values())


def test_arms_not_back_under_joint_control_end_failed_and_nothing_more_is_sent():
    """The switch back is rejected for 10 s: FAILED, the loud error once, no replan, no tail."""
    lab = Lab()
    arms = list(lab.robot.arms.values())
    lab.on_tick.append(lambda: [setattr(arm.controllers, "rejects", arm.controllers.running == COMPLIANCE
                                        or arm.controllers.rejects) for arm in arms])
    outcome = lab.run(lab.plan())
    assert outcome.status == FAILED and outcome.reason.startswith("not back under scaled_joint_trajectory_controller")
    assert "replan" not in lab.steps() and "trajectory" not in lab.steps()
    assert sum("ARMS MAY BE LEFT UNDER" in line for line in lab.ctx.lines("error")) == 1
    assert lab.events()[-2:] == ["left_ur_arm: STOP", "right_ur_arm: STOP"]


# --- --- --- --- --- the checks --- --- --- --- ---

#: A tail that passes every check: tool0 100 mm straight along x in 11 rows over 10 s, each joint 0.1 rad.
LINE = [Pose((0.01 * k, 0.0, 0.5)) for k in range(11)]
GOAL = Pose((0.1, 0.0, 0.5))
TAIL = np.linspace(np.zeros(12), np.full(12, 0.1), 11)
REACHED = {name: {"travel_mm": 5.0, "achieved_mm": 5.0, "remaining_mm": 0.5} for name in ARMS}
PASSING = {"names": NAMES, "tail": TAIL, "live": dict(zip(NAMES, TAIL[0])), "reached": REACHED,
           "forces": dict.fromkeys(ARMS, 1.0), "grippers": dict.fromkeys(ARMS, ScaffoldingV3.STALLED),
           "paths": dict.fromkeys(ARMS, LINE), "goals": dict.fromkeys(ARMS, GOAL), "seconds": 10.0}


def verdicts(**changes) -> dict[str, str]:
    """Each check's PASS or FAIL, by its name, with `changes` to the passing inputs."""
    lines = retreat_checks(**{**PASSING, **changes})
    return {line.split(" ", 1)[1].split(":", 1)[0]: line.split(" ", 1)[0] for line in lines}


def path_mm(*points) -> list[Pose]:
    """A tool0 path through points given in millimetres (x, y), at z = 0.5 m."""
    return [Pose((x * 1e-3, y * 1e-3, 0.5)) for x, y in points]


def longer(extra_mm: float) -> list[Pose]:
    """The line bent sideways at its middle, `extra_mm` longer."""
    side = math.sqrt(((100.0 + extra_mm) / 2.0) ** 2 - 50.0 ** 2)
    return path_mm((0.0, 0.0), (50.0, side), (100.0, 0.0))


def turned(degrees: float) -> list[Pose]:
    """The line, its last pose turned about z."""
    return LINE[:-1] + [Pose(LINE[-1].position, tuple(Rotation.from_euler("z", degrees, degrees=True).as_quat()))]


def travelled(radians: float) -> np.ndarray:
    """The tail, its first joint travelling `radians`."""
    tail = TAIL.copy()
    tail[:, 0] = np.linspace(0.0, radians, len(tail))
    return tail


CASES = {
    "start": ("start", lambda v: {"live": {**PASSING["live"], NAMES[0]: v}}, CHECK_START),
    "arrival": ("break-out", lambda v: {"reached": {**REACHED, "left_ur_arm": {
        "travel_mm": 5.0, "achieved_mm": 5.0, "remaining_mm": v * 1e3}}}, CHECK_ARRIVAL),
    "spread": ("break-out", lambda v: {"reached": {**REACHED, "left_ur_arm": {
        "travel_mm": 5.0, "achieved_mm": 5.0 + v * 1e3, "remaining_mm": 0.5}}}, CHECK_SPREAD),
    "force": ("contact", lambda v: {"forces": {"left_ur_arm": v, "right_ur_arm": 1.0}}, CHECK_FORCE),
    "length": ("left_ur_arm tool0 path", lambda v: {"paths": {**PASSING["paths"], "left_ur_arm": longer(v * 1e3)}},
               CHECK_LENGTH),
    "end": ("left_ur_arm tool0 path", lambda v: {"goals": {**PASSING["goals"], "left_ur_arm": Pose((0.1, v, 0.5))}},
            CHECK_LENGTH),
    "backtrack": ("left_ur_arm tool0 path", lambda v: {"paths": {**PASSING["paths"], "left_ur_arm": path_mm(
        (0.0, 0.0), (50.0, 0.0), (50.0 - v * 1e3, 0.0), (100.0, 0.0))}}, CHECK_BACKTRACK),
    "turn": ("left_ur_arm tool0 path", lambda v: {"paths": {**PASSING["paths"], "left_ur_arm": turned(
        math.degrees(v))}}, CHECK_TURN),
    "joint travel": ("joints", lambda v: {"tail": travelled(v), "live": dict(zip(NAMES, travelled(v)[0]))},
                     CHECK_JOINT_TRAVEL),
    "speed": ("speed", lambda v: {"seconds": 0.1 / v}, CHECK_SPEED),
}


def test_the_passing_inputs_pass_every_check():
    """Eight lines, in order, all PASS."""
    assert verdicts() == {"start": "PASS", "break-out": "PASS", "contact": "PASS", "grippers": "PASS",
                          "left_ur_arm tool0 path": "PASS", "right_ur_arm tool0 path": "PASS", "joints": "PASS",
                          "speed": "PASS"}


@pytest.mark.parametrize("case", list(CASES))
def test_each_retreat_check_fails_just_over_its_limit(case):
    """2% under the limit passes, 2% over fails that check only."""
    check, change, limit = CASES[case]
    under, over = verdicts(**change(0.98 * limit)), verdicts(**change(1.02 * limit))
    assert under[check] == "PASS" and over[check] == "FAIL"
    assert set(under.values()) == {"PASS"} and [name for name, verdict in over.items() if verdict == "FAIL"] == [check]


def test_a_check_without_its_measurement_fails():
    """No force reading, a gripper without a status or still LOOSENING, a TCP not measured: FAIL, said so."""
    lines = retreat_checks(**{**PASSING, "forces": {"left_ur_arm": None, "right_ur_arm": 1.0},
                              "grippers": {"left_ur_arm": None, "right_ur_arm": ScaffoldingV3.LOOSENING},
                              "reached": {**REACHED, "right_ur_arm": None}})
    assert "FAIL contact: |F| left_ur_arm no reading, right_ur_arm 1.0 N (limit 20 N)" in lines
    assert "FAIL grippers: left_ur_arm no status, right_ur_arm LOOSENING (want STALLED = fully open)" in lines
    assert "FAIL break-out: a tool0 was not measured" in lines
    assert not any(line.startswith("break-out") for line in retreat_checks(**{**PASSING, "reached": None}))
