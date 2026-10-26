"""The insert on a fake Cindy with her real frames and both scaffolding tools: its plan, its modes, its endings."""

from __future__ import annotations

import asyncio
import dataclasses
import json
import math
from types import SimpleNamespace

import numpy as np
import pytest
from planning_fixtures import CINDY, robot_config
from plugin_fakes import (ArmDriver, FakeContext, MocapDriver, ScaffoldingDriver, fake_arm, fake_node, fake_robot,
                          fake_scaffolding, lease_to_test)
from robot_lab import LEFT, RIGHT
from scipy.spatial.transform import Rotation
from test_execution_spiral import Operator
from test_execution_stream import BASE, MOCAP_ID, assert_same_pose, moved, world_target

from husky_assembly_teleop.robot_interface.arm import CARTESIAN_COMPLIANCE_CONTROLLER as COMPLIANCE
from husky_assembly_teleop.robot_interface.arm import SCALED_JOINT_TRAJECTORY_CONTROLLER as JOINT
from husky_assembly_teleop.robot_interface.end_effectors import ScaffoldingV3
from husky_assembly_teleop.robot_interface.frames import HUSKY_FRAME, MOCAP_FRAME, fixed_transform
from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names
from husky_assembly_teleop.shared.execution.insert import InsertPlan, plan_insert, run_insert
from husky_assembly_teleop.shared.execution.spiral import Spiral
from husky_assembly_teleop.shared.execution.stream import (DRY_RUN, END_WAIT, FAILED, HOLD_FOR_STALL, NO_STALL,
                                                           REFUSED, RIGID_DONE, STALLED, line_target, split_seconds,
                                                           tool0_fk)
from scaffolding_assembly_core.geometry import Pose, compose

pytestmark = pytest.mark.safety

ARMS = ("left_ur_arm", "right_ur_arm")
NAMES = [name for arm in ARMS for name in arm_joint_names(arm)]
#: The insert: both elbows turn 0.03 rad in 10 steps; each tool0 moves about 14.6 mm.
ROWS = np.array(LEFT + RIGHT) + np.outer(np.linspace(0.0, 1.0, 11), [0, 0, 0.03, 0, 0, 0, 0, 0, -0.03, 0, 0, 0])
STOP, TIGHTEN_SCREW = (0, 0), (ScaffoldingV3.JOINT_MOTOR, 1)
URDF = robot_config(CINDY).model.urdf


class Lab:
    """Cindy at BASE with her URDF frames, under joint control at the insert's start, a scaffolding tool on each arm.

    Leased to the test. Every tick the arms, their controller managers and the tools' firmware answer. `sent` keeps
    each arm's Cartesian targets with the TCP measured when each went out.
    """

    def __init__(self, left: dict | None = None, right: dict | None = None):
        """Build Cindy; `left` and `right` go to the tools' firmware (`ScaffoldingDriver`)."""
        self.ctx = FakeContext(step=self.tick)
        node = fake_node(self.ctx.now, self.ctx.logs)
        config = robot_config(CINDY)
        self.tools = {name: fake_scaffolding(node, f"/a200_0806/{name.split('_')[0]}_gripper") for name in ARMS}
        arms = [fake_arm(node, name, joints, end_effector=self.tools[name])
                for name, joints in zip(ARMS, (LEFT, RIGHT))]
        for arm in arms:
            arm.urdf, arm.base_in_husky = URDF, fixed_transform(URDF, HUSKY_FRAME, f"{arm.config.name}_base_link")
        self.robot = lease_to_test(fake_robot(node, "a200-0806", arms))
        self.robot.config.model = config.model
        self.robot._husky_in_mocap_frame = fixed_transform(URDF, MOCAP_FRAME, HUSKY_FRAME)
        self.mocap = MocapDriver(self.robot, BASE, MOCAP_ID)
        self.drivers = [ArmDriver(arm, self.robot) for arm in arms]
        self.screws = {"left_ur_arm": ScaffoldingDriver(self.tools["left_ur_arm"], **(left or {})),
                       "right_ur_arm": ScaffoldingDriver(self.tools["right_ur_arm"], **(right or {}))}
        self.sent: dict[str, list] = {name: [] for name in ARMS}
        for name, arm in self.robot.arms.items():
            arm.targets_sent.attach(lambda target, arm=arm, name=name: self.sent[name].append(
                (target, np.array(arm.state.tcp_position))))
        self.tick(self.ctx.t)

    def tick(self, t: float) -> None:
        """Mocap sees the base, the arms move and report, their controller managers answer, the tools run and report."""
        self.mocap.tick(t)
        for driver in self.drivers:
            driver.tick(t)
        for arm in self.robot.arms.values():
            arm.controllers.tick(t)
        for screw in self.screws.values():
            screw.tick(t)

    def plan(self, **settings) -> InsertPlan:
        """The insert of ROWS: by default B21-like (the left screw mates), split 5 mm, a straight line, 5 s."""
        defaults = dict(mating=("left_ur_arm",), grounded=False, split_mm=5.0, rigid_only=False, spiral=None,
                        traj_time=5.0)
        plan = plan_insert(self.robot, NAMES, ROWS, **{**defaults, **settings})
        assert isinstance(plan, InsertPlan), plan
        return plan

    def work(self, plan: InsertPlan, operator: Operator | None = None, dry_run: bool = False):
        """The insert's coroutine, on every tool."""
        return run_insert(self.ctx, self.robot, plan, self.tools, operator=operator or Operator(self), dry_run=dry_run)

    def run(self, plan: InsertPlan, **options):
        """Run the insert to its end."""
        return asyncio.run(self.work(plan, **options))

    def commands(self, arm: str) -> list[tuple[int, int]]:
        """(motor, direction) of every command sent to one arm's tool."""
        return [(command.motor, command.direction) for command in self.tools[arm]._command.messages]

    def streamed(self, arm: str) -> list:
        """The arm's streamed targets: every Cartesian target but a hold's."""
        return [target for target, _tcp in self.sent[arm] if not target.hold]

    def switches(self, arm: str) -> list[str]:
        """Every controller the arm was asked to switch to."""
        return self.robot.arms[arm].controllers.switches

    def running(self) -> list[str | None]:
        """What each arm's controller manager runs."""
        return [self.robot.arms[arm].controllers.running for arm in ARMS]

    def nothing_sent(self) -> bool:
        """No trajectory, target, wrench, switch or tool command went out."""
        arms = self.robot.arms.values()
        return not (self.robot._multi_arm.messages or any(
            arm._trajectory.messages or arm._target_frame.messages or arm._target_wrench.messages
            or arm.controllers.switches for arm in arms) or any(tool._command.messages for tool in self.tools.values()))


def world_tool0(lab: Lab, arm: str, row: np.ndarray) -> Pose:
    """An arm's tool0 at a row of ROWS' joints, in the world, with the base at BASE."""
    base = compose(compose(BASE, lab.robot._husky_in_mocap_frame), lab.robot.arms[arm].base_in_husky)
    return compose(base, tool0_fk(URDF, arm, dict(zip(NAMES, row))))


def insert_length() -> float:
    """The insert's length, millimetres: the furthest an arm's tool0 travels."""
    return 1e3 * max(float(np.linalg.norm(np.subtract(tool0_fk(URDF, arm, dict(zip(NAMES, ROWS[-1]))).position,
                                                      tool0_fk(URDF, arm, dict(zip(NAMES, ROWS[0]))).position)))
                     for arm in ARMS)


# --- --- --- --- --- the plan --- --- --- --- ---

def test_the_plan_splits_5_mm_off_the_end_and_says_so():
    """B21-like: rigid D − 5 mm, then compliant 5 mm in the rest of the 5 s; the prompt states both, the left screw."""
    lab = Lab()
    plan = lab.plan()
    total = insert_length()
    assert 14.0 < total < 15.5 and plan.total_mm == pytest.approx(total) and plan.achieved_mm == pytest.approx(5.0)
    rigid_s, compliant_s = split_seconds(5.0, plan.achieved_mm, plan.total_mm)
    assert plan.compliant_seconds == compliant_s == pytest.approx(5.0 * 5.0 / total)
    assert rigid_s - 1e-9 <= plan.rigid.duration < rigid_s + 0.2 and plan.arms == ARMS and plan.spiral is None
    left, right = plan.rigid.arms
    np.testing.assert_allclose(left.positions[0], LEFT, atol=1e-12)
    assert plan.text == (f"INSERT: rigid {total - 5.0:.1f} mm ({plan.rigid.duration:.1f} s), then COMPLIANT 5.0 mm "
                         f"(straight line); the joint screw of left_ur_arm TIGHTENS from the start until it stalls "
                         f"(≤ 30 s), then STOP and back to joint control. Watch the bar enter its joint and the held "
                         f"bars stay in their jaws. Confirm to start.")
    for arm in ARMS:
        assert_same_pose(plan.ends[arm], tool0_fk(URDF, arm, dict(zip(NAMES, ROWS[-1]))))


def test_a_spiral_too_fast_is_refused_at_plan():
    """5 s at 4 turns runs 75 mm/s at full radius: refused, with the shortest duration; nothing sent."""
    lab = Lab()
    refused = plan_insert(lab.robot, NAMES, ROWS, mating=("left_ur_arm",), grounded=False, split_mm=5.0,
                          rigid_only=False, spiral=(5.0, 4), traj_time=5.0)
    assert refused == "spiral too fast: 75 mm/s at 15 mm, limit 50; 4 turns need ≥ 7.6 s" and lab.nothing_sent()


def test_the_rigid_only_and_ground_bar_prompts():
    """Rigid-only ignores the split and the spiral; a ground bar ignores rigid-only too."""
    lab = Lab()
    total = insert_length()
    plan = lab.plan(rigid_only=True, spiral=(5.0, 4))
    assert plan.rigid_only and plan.spiral is None and plan.achieved_mm == 0.0 and plan.compliant_seconds == 0.0
    assert plan.text.startswith(f"RIGID-ONLY insert: the whole {total:.1f} mm under joint control "
                                f"({plan.rigid.duration:.1f} s); the joint screw of left_ur_arm TIGHTENS from the "
                                f"start")
    assert "Compliance is never engaged; the split and the spiral are ignored." in plan.text
    ground = lab.plan(mating=(), grounded=True, rigid_only=True, spiral=(30.0, 6))
    assert not ground.rigid_only and ground.spiral is None and ground.mating == ()
    assert ground.text == (f"RIGID insert of a ground bar, no tighten, {ground.rigid.duration:.1f} s, largest joint "
                           f"travel {math.degrees(0.03):.1f}°.")


# --- --- --- --- --- the split insert --- --- --- --- ---

def test_a_mating_insert_runs_rigid_then_streams_the_line_until_the_left_screw_stalls():
    """B21-like: STOP both, TIGHTEN the left screw only; rigid to the split; then each tick's target is the line.

    The line runs from the measured TCP at the handover (tool0 at the split) to tool0 at the path's end, linear in time
    over the compliant seconds, then holds; the stall 6 s after TIGHTEN ends it: STOP, hold, joint control.
    """
    lab = Lab(left={"screw_stall_after": 6.0})
    plan = lab.plan()
    outcome = lab.run(plan)
    assert outcome.status == STALLED and outcome.ok, outcome.reason
    assert lab.commands("left_ur_arm") == [STOP, TIGHTEN_SCREW, STOP] and lab.commands("right_ur_arm") == [STOP, STOP]
    assert [phase["name"] for phase in outcome.record["phases"]] == ["rigid", "switch", "compliant"]
    assert outcome.record["phases"][0]["end"] == "ARRIVED"
    # * The firmware takes the TIGHTEN and reports the stall a tick later each.
    assert list(outcome.record["stall_times"]) == ["left_ur_arm"]
    assert 6.0 <= outcome.record["stall_times"]["left_ur_arm"] <= 6.2 + 1e-9
    split = np.concatenate([motion.positions[-1] for motion in plan.rigid.arms])
    for arm in ARMS:
        streamed = lab.streamed(arm)
        first = streamed[0]
        start = world_target(lab, arm, first, BASE)
        assert_same_pose(start, world_tool0(lab, arm, split), 1e-9)
        end = world_tool0(lab, arm, ROWS[-1])
        for target in streamed:
            expected = line_target(start, end, plan.compliant_seconds, target.update_time - first.update_time)
            assert_same_pose(world_target(lab, arm, target, BASE), expected, 1e-9)
        assert lab.ctx.t >= streamed[-1].update_time > first.update_time + plan.compliant_seconds
        last, tcp = lab.sent[arm][-1]
        assert last.hold and np.allclose(last.position, tcp, atol=0.0)
        assert lab.switches(arm) == [COMPLIANCE, JOINT]
        wrenches = lab.robot.arms[arm]._target_wrench.messages
        assert wrenches and all(message.wrench.force.x == message.wrench.force.z == 0.0 for message in wrenches)
        assert outcome.record["end_error_mm"][arm]["remaining_mm"] < 2.0
    assert lab.running() == [JOINT, JOINT]
    json.dumps(outcome.record)


def test_an_early_stall_holds_the_rigid_part_then_switches():
    """The screw stalls 1 s into the rigid part: the motion ENDS, the arms are held, then switched; nothing streamed."""
    lab = Lab(left={"screw_stall_after": 1.0})
    outcome = lab.run(lab.plan())
    assert outcome.status == STALLED
    rigid, switch, compliant = outcome.record["phases"]
    assert rigid["end"] == "ENDED" and rigid["t1"] <= switch["t0"] and compliant["end"] == "stalled"
    for arm in lab.robot.arms.values():
        assert any(len(message.points) == 1 for message in arm._trajectory.messages)  # the hold
        assert arm.controllers.switches == [COMPLIANCE, JOINT]
    assert not lab.streamed("left_ur_arm") and not lab.streamed("right_ur_arm")


def test_no_stall_holds_the_end_until_the_ceiling():
    """The screw never stalls: the line's end is held until T_c + 30 + 1 s: NO_STALL, not ok; joint control again."""
    lab = Lab(left={"screw_stall_after": None})
    plan = lab.plan()
    outcome = lab.run(plan)
    assert outcome.status == NO_STALL and not outcome.ok and "30 s" in outcome.reason
    compliant = outcome.record["phases"][-1]
    assert compliant["end"] == "no_stall"
    seconds = plan.compliant_seconds + HOLD_FOR_STALL + END_WAIT
    assert seconds <= compliant["t1"] - compliant["t0"] <= seconds + 0.15
    assert lab.commands("left_ur_arm")[-1] == STOP and lab.running() == [JOINT, JOINT]


def test_cancel_during_compliance_stops_the_tools_holds_at_the_tcp_and_switches_back():
    """Cancelled while streaming: no more targets, STOP, each arm's last target a hold at its TCP, joint control."""
    lab = Lab(left={"screw_stall_after": None})

    async def scenario():
        task = asyncio.ensure_future(lab.work(lab.plan()))
        while len(lab.streamed("left_ur_arm")) < 5 and not task.done():
            await asyncio.sleep(0)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(scenario())
    assert len(lab.streamed("left_ur_arm")) == 5
    for arm in ARMS:
        last, tcp = lab.sent[arm][-1]
        assert last.hold and np.allclose(last.position, tcp, atol=0.0)
        assert lab.commands(arm)[-1] == STOP and lab.switches(arm) == [COMPLIANCE, JOINT]
    assert lab.running() == [JOINT, JOINT]


def test_a_switch_refused_sends_no_target():
    """The controller managers reject the switch: FAILED, no Cartesian target, the tools stopped."""
    lab = Lab()
    for arm in lab.robot.arms.values():
        arm.controllers.rejects = True
    outcome = lab.run(lab.plan())
    assert outcome.status == FAILED and "no target sent" in outcome.reason
    assert not lab.sent["left_ur_arm"] and not lab.sent["right_ur_arm"]
    assert lab.commands("left_ur_arm")[-1] == STOP and lab.running() == [JOINT, JOINT]


def test_the_record_keeps_the_furthest_the_base_moved():
    """The base moves 1.5 mm during the rigid part: the record says so; under BASE_DRIFT, the compliant part runs."""
    lab = Lab(left={"screw_stall_after": 6.0})
    shifted = moved(BASE, (0.0, 0.0015, 0.0))
    lab.drivers.append(SimpleNamespace(tick=lambda t: setattr(lab.mocap, "pose", shifted) if t > 11.0 else None))
    outcome = lab.run(lab.plan())
    assert outcome.status == STALLED and outcome.record["base_drift_max_mm"] == pytest.approx(1.5, abs=1e-6)


def test_a_hold_refused_as_the_insert_ends_fails_it():
    """The left arm refuses its hold after the stall (as on a stale TCP): joint control is asked all the same; FAILED.

    The one warn names the arm; the record says the way back was not ok.
    """
    lab = Lab(left={"screw_stall_after": 6.0})
    lab.robot.arms["left_ur_arm"].hold = lambda: False
    outcome = lab.run(lab.plan())
    assert outcome.status == FAILED and not outcome.ok
    assert outcome.reason.startswith("left_ur_arm joint screw STALLED; but the hold or the switch back to")
    assert lab.running() == [JOINT, JOINT] and not outcome.record["switch"]["back_ok"]
    assert sum(line.startswith("the hold was refused on left_ur_arm") for line in lab.ctx.lines("warn")) == 1


def test_a_tool_stop_that_raises_still_brings_the_arms_back():
    """The left tool's STOP raises as the compliant insert ends: the error comes out, joint control asked first."""
    lab = Lab(left={"screw_stall_after": 3.0})
    tool, left = lab.tools["left_ur_arm"], lab.robot.arms["left_ur_arm"]
    stop = tool.stop

    def broken() -> None:
        """The STOP at the start goes out; the one at the end, the arms compliant, raises."""
        if left.controllers.is_active(COMPLIANCE):
            raise RuntimeError("serial port gone")
        stop()

    tool.stop = broken
    with pytest.raises(RuntimeError, match="serial port gone"):
        lab.run(lab.plan())
    assert lab.switches("left_ur_arm") == [COMPLIANCE, JOINT] and lab.running() == [JOINT, JOINT]
    assert lab.sent["left_ur_arm"][-1][0].hold


def test_the_right_arm_failing_after_the_left_switched_brings_the_left_back():
    """Only the right arm's switch is rejected: FAILED; the left, already compliant, is held and switched back."""
    lab = Lab()
    lab.robot.arms["right_ur_arm"].controllers.rejects = True
    outcome = lab.run(lab.plan())
    assert outcome.status == FAILED and not lab.streamed("left_ur_arm") and not lab.streamed("right_ur_arm")
    assert lab.switches("left_ur_arm") == [COMPLIANCE, JOINT] and lab.running() == [JOINT, JOINT]
    assert lab.sent["left_ur_arm"][-1][0].hold


def test_the_spiral_settings_reach_the_spiral():
    """Spiral 30 s, 6 turns: the prompt says so; each tick's target is the line over 30 s plus that spiral's offset.

    The spiral turns around the planned insert's direction in the world: every arm's tool0 travel, summed.
    """
    lab = Lab(left={"screw_stall_after": 6.0})
    plan = lab.plan(spiral=(30.0, 6))
    assert "COMPLIANT 5.0 mm (spiral 15 mm, 30 s, 6 turns, pitch 2.5 mm)" in plan.text
    outcome = lab.run(plan)
    assert outcome.status == STALLED and outcome.record["plan"]["spiral"]["pitch_mm"] == pytest.approx(2.5)
    axis = sum(np.subtract(world_tool0(lab, arm, ROWS[-1]).position, world_tool0(lab, arm, ROWS[0]).position)
               for arm in ARMS)
    spiral = Spiral(axis, 30.0, 6)
    for arm in ARMS:
        streamed = lab.streamed(arm)
        first = streamed[0]
        start, end = world_target(lab, arm, first, BASE), world_tool0(lab, arm, ROWS[-1])
        assert len(streamed) > 20
        for target in streamed:
            elapsed = target.update_time - first.update_time
            assert_same_pose(world_target(lab, arm, target, BASE),
                             line_target(start, end, 30.0, elapsed, spiral.offset(elapsed)), 1e-9)
    assert outcome.record["spiral_text"].startswith("spiral ") and "of 6 turns" in outcome.record["spiral_text"]


# --- --- --- --- --- refusals and dry runs --- --- --- --- ---

def _tcp_off(lab: Lab) -> None:
    """The left TCP measured 11 mm off tool0 by the kinematics."""
    state = lab.robot.arms["left_ur_arm"].state
    state.tcp_position = np.add(state.tcp_position, [0.0, 0.011, 0.0])


def _test_mode(lab: Lab) -> None:
    arm = lab.robot.arms["right_ur_arm"]
    arm.config = dataclasses.replace(arm.config, cartesian_test_mode=True)


def _no_mocap(lab: Lab) -> None:
    """Mocap stops seeing the base, and its pose is gone."""
    lab.mocap.live = False
    lab.robot.base.state.position = None


@pytest.mark.parametrize("change, why", [
    (_tcp_off, "11.0 mm, 0.0° from tool0"),
    (_test_mode, "right_ur_arm is in Cartesian test mode"),
    (lambda lab: setattr(lab.robot.arms["left_ur_arm"], "urdf_problem", "moved"), "stock UR frames"),
    (lambda lab: setattr(lab.robot.arms["left_ur_arm"].state.controllers, "active", COMPLIANCE),
     f"runs {COMPLIANCE}"),
    (lambda lab: setattr(lab.robot.arms["left_ur_arm"].state.controllers, "switch_in_flight", COMPLIANCE),
     f"left_ur_arm is still switching to {COMPLIANCE}"),
    (lambda lab: setattr(lab.tools["right_ur_arm"].state, "last_update_time", lab.ctx.t - 2.0), "no tool status"),
    (_no_mocap, "no base pose from mocap"),
    (lambda lab: lab.robot.base.state.marker_errors.clear(), "base pose lost"),
])
def test_every_refusal_sends_nothing(change, why):
    """Wrong frames, test mode, URDF problem, not under JTC or still switching, silent tool, no mocap: REFUSED."""
    lab = Lab()
    plan = lab.plan()
    change(lab)
    outcome = lab.run(plan)
    assert outcome.status == REFUSED and why in outcome.reason and lab.nothing_sent()


@pytest.mark.parametrize("settings", [{}, {"spiral": (20.0, 4)}, {"rigid_only": True},
                                      {"mating": (), "grounded": True}])
def test_a_dry_run_publishes_nothing(settings):
    """Every check runs (the rigid trajectory built, each compliant target checked); nothing goes out."""
    lab = Lab()
    outcome = lab.run(lab.plan(**settings), dry_run=True)
    assert outcome.status == DRY_RUN and lab.nothing_sent()
    assert any("DRY RUN a200-0806 left_ur_arm" in line for line in lab.ctx.lines("warn"))
    if not settings:
        assert any("every compliant target passed" in line for line in lab.ctx.lines("warn"))


# --- --- --- --- --- rigid modes --- --- --- --- ---

def test_a_ground_bar_runs_rigid_without_tighten_or_switch():
    """No screw tightens, no switch, no Cartesian target, rigid-only ignored; it ends with the motion."""
    lab = Lab()
    outcome = lab.run(lab.plan(mating=(), grounded=True, rigid_only=True))
    assert outcome.status == RIGID_DONE and outcome.ok
    assert lab.commands("left_ur_arm") == lab.commands("right_ur_arm") == [STOP, STOP]
    assert lab.switches("left_ur_arm") == lab.switches("right_ur_arm") == []
    assert not lab.sent["left_ur_arm"] and not lab.sent["right_ur_arm"]
    assert outcome.record["end_gap_mm"] < 1e-6
    assert "a200-0806: servo after the insert: M7, not run" in lab.ctx.lines("info")


@pytest.mark.parametrize("stall_after, status, phase", [(1.0, STALLED, "rigid"), (8.0, STALLED, "hold"),
                                                        (None, NO_STALL, "hold")])
def test_rigid_only_tightens_runs_the_whole_path_and_holds_for_the_stall(stall_after, status, phase):
    """One joint motion over the whole path, the left screw tightening; then held until it stalls, or 30 s.

    Never compliant: no switch, no Cartesian target; the split and the spiral are ignored.
    """
    lab = Lab(left={"screw_stall_after": stall_after})
    outcome = lab.run(lab.plan(rigid_only=True, spiral=(5.0, 4)))
    assert outcome.status == status and outcome.record["phases"][-1]["name"] == phase
    message, = lab.robot._multi_arm.messages
    np.testing.assert_allclose(message.trajectory1.points[-1].positions, ROWS[-1, :6], atol=1e-12)
    np.testing.assert_allclose(message.trajectory2.points[-1].positions, ROWS[-1, 6:], atol=1e-12)
    assert lab.commands("left_ur_arm") == [STOP, TIGHTEN_SCREW, STOP] and lab.commands("right_ur_arm") == [STOP, STOP]
    assert lab.switches("left_ur_arm") == lab.switches("right_ur_arm") == []
    assert not lab.sent["left_ur_arm"] and not lab.sent["right_ur_arm"]
    if status == NO_STALL:
        hold = outcome.record["phases"][-1]
        assert not outcome.ok and HOLD_FOR_STALL <= hold["t1"] - hold["t0"] <= HOLD_FOR_STALL + 0.15


def test_the_axis_is_the_planned_travel_in_the_husky_frame():
    """plan.axis turned by the husky frame's world rotation is the arms' tool0 travel summed in the world."""
    lab = Lab()
    plan = lab.plan(spiral=(20.0, 4))
    husky = compose(BASE, lab.robot._husky_in_mocap_frame)
    world = sum(np.subtract(world_tool0(lab, arm, ROWS[-1]).position, world_tool0(lab, arm, ROWS[0]).position)
                for arm in ARMS)
    np.testing.assert_allclose(Rotation.from_quat(husky.orientation).apply(plan.axis), world, atol=1e-12)
