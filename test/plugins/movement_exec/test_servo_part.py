"""The servo part on fake robots: a stub planner solving by ssik at the live base, synthetic MoCap markers.

Spec §6 "test_servo_part": the tool0 and the MoCap-bar loops, the gates, the prompts, cancels, interruptions, the
record and the carry. Nothing is sent before Confirm; every arm is held on a cancel or an interruption.
"""

from __future__ import annotations

import asyncio
from dataclasses import replace
from math import radians

import numpy as np
import pytest
from planning_fixtures import ALICE, BELLE, CINDY, design_scene, purged_process, robot_config, step
from plugin_fakes import from_outside
from robot_lab import FakeRun, RobotLab

from husky_assembly_teleop.plugins.movement_exec.plugin import MovementExecPlugin
from husky_assembly_teleop.robot_interface.mocap_state import RawMarkers
from husky_assembly_teleop.shared.assembly_planning.ik import branches, fitting_solver, ssik_unavailable
from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names, joint_limits, unwrap_to_nearest
from husky_assembly_teleop.shared.assembly_planning.movement import Routine
from husky_assembly_teleop.shared.assembly_planning.route_result import RouteResult
from husky_assembly_teleop.shared.assembly_planning.servo_route import ServoStep
from husky_assembly_teleop.shared.assembly_planning.steps import operator_steps
from husky_assembly_teleop.shared.execution.basis import basis_of
from husky_assembly_teleop.shared.execution.send import DRY_RUN as SENT_DRY, INTERRUPTED
from husky_assembly_teleop.shared.execution.servo import flange_poses, insert_end_gap, servo_plan
from husky_assembly_teleop.shared.execution.servo_record import plot_servo
from husky_assembly_teleop.shared.movement_exec_contract import (CANCELLED, DONE, DRY_RUN, FAILED, REFUSED,
                                                                 SERVO_INDEX, SERVO_PART, ExecRequest, serial_of,
                                                                 servo_part, step_parts)
from husky_assembly_teleop.shared.runs.runs import RunSettings
from husky_assembly_teleop.world.bar_pose import transform_points
from husky_assembly_teleop.world.marker_layouts import bar_layouts
from scaffolding_assembly_core.geometry import Pose, compose
from scaffolding_assembly_core.ids import robot_id, split_link_id
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.process.solutions import MovementResult, Trajectory
from scaffolding_assembly_core.robot import FLANGE_SUFFIX

pytestmark = pytest.mark.skipif(ssik_unavailable() is not None, reason=ssik_unavailable() or "")

#: Wall-clock seconds a scenario may take: a guard missing makes a test wait forever, this makes it fail instead.
WALL_LIMIT = 40.0
#: The bar whose markers the transfer and insert tests stream, and its tracked name in the servo.
BAR, TRACKED = "bars/B20", "servo_B20"
ARMS = ("left_ur_arm", "right_ur_arm")


def run_guarded(scenario):
    """Run a scenario in a new event loop, failing (not hanging) after WALL_LIMIT seconds of wall clock."""
    return asyncio.run(asyncio.wait_for(scenario, WALL_LIMIT))


def offset(dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, roll_deg: float = 0.0) -> np.ndarray:
    """A small rigid error: a translation, metres, and a turn about x, degrees."""
    T = np.eye(4)
    angle = radians(roll_deg)
    T[1:3, 1:3] = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    T[:3, 3] = (dx, dy, dz)
    return T


class StubPlanner:
    """Answers a servo pass at once: IK by ssik at the live base, then the rows [live, (detour), goal].

    Attributes:
        fraction: How far along the correction the goal row goes (1: all of it; 0.5: half, so passes keep coming).
        detours: Pass number -> radians added to the first joint in a middle row, to make a pass travel far.
        requests: Every request received, for assertions.
    """

    busy, running = False, ""

    def __init__(self, lab: ServoLab, fraction: float = 1.0, detours: dict[int, float] | None = None):
        """Nothing asked yet."""
        self.lab, self.fraction, self.detours = lab, fraction, dict(detours or {})
        self.requests: list = []

    def request_cancel(self) -> None:
        """Nothing runs long enough to cancel."""

    def request_servo_step(self, request) -> asyncio.Future:
        """Solve `request.servo.targets` for the step's robot where the snapshot has it; a resolved future."""
        self.requests.append(request)
        ask = request.servo
        serial = serial_of(request.loaded, request.step.action.id, request.robots)
        robot = request.snapshot.robots[robot_id(serial)]
        urdf = robot.model.urdf
        names, start, goal, correction = [], [], [], {}
        for flange, target in ask.targets.items():
            arm = split_link_id(flange)[1][:-len(FLANGE_SUFFIX)]
            own = arm_joint_names(arm)
            reference = [robot.joints[name] for name in own]
            found = branches(urdf, robot.base, arm, fitting_solver(urdf, arm), target, reference)
            if not found:
                return _resolved(ServoStep(RouteResult(MovementResult("failed", "no IK for the servo target (stub)"),
                                                       None, planner="stub"), {}, False))
            q = unwrap_to_nearest(found[0], reference, *joint_limits(urdf, own))
            at_goal = link_pose(urdf, robot.base, dict(zip(own, q)), f"{arm}{FLANGE_SUFFIX}")
            at_start = link_pose(urdf, robot.base, dict(zip(own, reference)), f"{arm}{FLANGE_SUFFIX}")
            correction[flange] = float(np.linalg.norm(np.subtract(at_goal.position, at_start.position)))
            names, start, goal = names + own, start + reference, goal + q.tolist()
        if max(correction.values()) < ask.stop_below:
            return _resolved(ServoStep(RouteResult(MovementResult("not_planned", "nothing left to plan"), None,
                                                   planner="stub"), correction, True))
        start, goal = np.array(start), np.array(goal)
        end = start + self.fraction * (goal - start)
        rows = [start, end]
        detour = self.detours.get(len(self.requests))
        if detour is not None:
            middle = start.copy()
            middle[0] += detour
            rows = [start, middle, end]
        trajectory = Trajectory(request.step.action.robot, tuple(names), tuple(map(tuple, np.array(rows).tolist())))
        return _resolved(ServoStep(RouteResult(MovementResult("solved", trajectory=trajectory), None, planner="stub",
                                               goal_joints=dict(zip(names, goal.tolist()))), correction, False))


def _resolved(step: ServoStep) -> asyncio.Future:
    """A future already holding `step`."""
    future = asyncio.get_running_loop().create_future()
    future.set_result(step)
    return future


class ServoLab(RobotLab):
    """The acting robot at a movement's authored target joints, where the design has it, its base shifted.

    movement_exec is set up with a `StubPlanner`; `stream(t)` (set by the bar tests) feeds the servo's tracked bar its
    markers every tick.
    """

    stream = None

    def __init__(self, movement: str = "B20_J_transfer", shift: tuple = (0.01, 0.0, 0.0), **driver):
        """Build everything; `driver` options go to every arm's driver."""
        loaded = purged_process()
        at = step(loaded, movement)
        configs = tuple(robot_config(serial) for serial in (ALICE, BELLE, CINDY))
        serial = serial_of(loaded, at.action.id, configs)
        own = at.movement.target.robots[at.action.robot]
        arms = [arm for arm in ("left_ur_arm", "right_ur_arm", "ur_arm") if f"{arm}_shoulder_pan_joint" in own.joints]
        super().__init__(joints={serial: {arm: [own.joints[name] for name in arm_joint_names(arm)] for arm in arms}},
                         **driver)
        self.loaded, self.at, self.serial = loaded, at, serial
        for live, robot in design_scene(loaded, movement).robots.items():
            self.bases[live.split("/")[-1]] = robot.base
        self.shift(shift)
        self.run = FakeRun(loaded, (at.action.id, at.movement_index))
        self.planner = StubPlanner(self)
        self.ctx.plugins = {"assembly_run": self.run, "motion_planner": self.planner}
        self.plugin = MovementExecPlugin()
        self.plugin.setup(self.ctx)

    def shift(self, delta: tuple) -> None:
        """Move the acting robot's measured base by `delta`, metres."""
        base = self.bases[self.serial]
        self.bases[self.serial] = Pose(tuple(np.add(base.position, delta)), base.orientation)
        self.tick(self.ctx.t)

    def tick(self, t: float) -> None:
        """As the lab's, then the bar's markers into the servo's tracked object."""
        super().tick(t)
        obj = self.ctx.world.tracked_objects.get(TRACKED)
        if obj is not None and self.stream is not None:
            obj.markers, obj.last_update_time = self.stream(t), t

    @property
    def robot(self):
        """The acting robot's interface."""
        return self.robots[self.serial]

    def measured(self):
        """The acting robot in the snapshot now."""
        return self.ctx.scene.snapshot.robots[robot_id(self.serial)]

    def operator_step(self):
        """The operator step whose primary is the lab's movement (workaround on)."""
        return next(each for each in operator_steps(self.loaded.process, self.at.action, True)
                    if each.primary == self.at.movement_index)

    def servo_request(self, **changes) -> ExecRequest:
        """A request running only the step's servo part (an Exec after the step's other parts are done)."""
        action, loaded = self.at.action, self.loaded
        operator = self.operator_step()
        request = ExecRequest(loaded, action.id, operator, (servo_part(loaded, action.id, operator.primary, True),),
                              self.serial, loaded.process.robots[action.robot].name, None, None, None, 2.0,
                              self.at.movement.id)
        return replace(request, **changes)

    def step_request(self, delta: float = 0.2) -> ExecRequest:
        """The whole step, its motion a fake 3-row plan moving each arm's first joint by `delta`."""
        action, loaded, robot = self.at.action, self.loaded, self.robot
        operator = self.operator_step()
        arms = list(robot.arms)
        names = [name for arm in arms for name in arm_joint_names(arm)]
        start = np.concatenate([robot.arms[arm].joint_vector() for arm in arms])
        goal = start.copy()
        goal[::6] += delta
        rows = np.array([start, (start + goal) / 2, goal])
        route = RouteResult(MovementResult("solved", trajectory=Trajectory(action.robot, tuple(names),
                                                                           tuple(map(tuple, rows.tolist())))),
                            Routine.TRANSFER_FREE, planner="fake")
        primary = loaded.steps[loaded.index_of(action.id, operator.primary)]
        basis = basis_of(self.ctx.scene.snapshot, loaded, primary, self.serial, tuple(names),
                         dict(zip(names, start.tolist())), True, self.run.view.statuses, False, self.ctx.t)
        return ExecRequest(loaded, action.id, operator, step_parts(loaded, action.id, operator, self.configs, True),
                           self.serial, loaded.process.robots[action.robot].name, route, operator.primary, basis, 2.0,
                           primary.movement.id)

    def bar_plan(self, by_bar: bool = True):
        """The servo's plan for the lab's movement, for the bar's holder and layout."""
        return servo_plan(self.loaded, self.at.action.id, self.at.movement_index, RunSettings(servo_by_bar=by_bar),
                          bar_layouts(self.loaded.process))

    def markers(self, E: np.ndarray = np.eye(4), where: str = "world", count: int = 8, age: float = 0.0,
                pattern: float = 0.0):
        """Stream B20's markers each tick from where `E` puts the bar relative to the FK bar (world or bar frame).

        `count` markers of the layout, `age` seconds old at capture, `pattern` metres added alternately +/- along
        the bar's x (a stretch no rigid fit can absorb, so the fit's rms reads `pattern`).
        """
        plan = self.bar_plan()
        holder, grasp = plan.holder

        def stream(t: float) -> RawMarkers:
            flange = flange_poses(self.measured(), [holder])[holder]
            B_fk = compose(flange, grasp).matrix()
            T = E @ B_fk if where == "world" else B_fk @ E
            points = plan.layout.points[:count].copy()
            points[:, 0] += np.where(np.arange(count) % 2 == 0, pattern, -pattern)
            return RawMarkers(ids=np.arange(1, count + 1), positions=transform_points(T, points),
                              measured=np.ones(count, dtype=bool), residuals=np.full(count, 5e-4),
                              sizes=np.full(count, 0.019), capture_time=t - age)
        self.stream = stream

    def servo_record(self) -> dict | None:
        """The servo record written (the `file` of the "servo" event), or None."""
        return next((fields["file"] for event, fields in self.run.records if event == "servo"), None)

    def prompts(self) -> list[tuple[str, bool]]:
        """Every prompt's text and whether it warned, in order."""
        return [(fields["data"]["text"], fields["data"]["warn"]) for event, fields in self.run.records
                if event == "prompt"]

    def free(self) -> bool:
        """Whether every robot is given back: none leased."""
        return all(robot.lease_slot.lease is None for robot in self.ctx.world.robots.values())

    def ended(self) -> str:
        """How the last part ended, in its own words (the last exec record's reason)."""
        return self.run.files()[-1]["outcome"]["reason"]

    def holds(self) -> list[int]:
        """One-point trajectories, i.e. a hold, that each of the acting robot's arms got: a count per arm."""
        return [sum(len(message.points) == 1 for message in arm._trajectory.messages)
                for arm in self.robot.arms.values()]


async def answer(lab: ServoLab, ok: bool = True) -> None:
    """Answer the next prompt a little over 0.5 s after it opened; return at once if nothing runs."""
    while lab.plugin.prompt is None:
        if not lab.plugin.active:
            return
        await asyncio.sleep(0)
    prompt = lab.plugin.prompt
    while lab.ctx.t - prompt.opened_at < 0.6:
        await asyncio.sleep(0)
    lab.plugin.request_confirm() if ok else lab.plugin.request_cancel()
    while lab.plugin.prompt is prompt:
        await asyncio.sleep(0)


async def sent(lab: ServoLab, count: int) -> None:
    """Wait until `count` trajectories went out."""
    while lab.published() < count:
        await asyncio.sleep(0)


def settle(lab: ServoLab, seconds: float = 2.5) -> None:
    """Let ROS time pass with the robots still (a lease needs every arm still for 2 s)."""
    async def ticks():
        until = lab.ctx.t + seconds
        while lab.ctx.t < until:
            await lab.ctx.next_tick()
    run_guarded(ticks())


def execute(lab: ServoLab, request: ExecRequest, *answers: bool, during=None):
    """Run a request to its end, answering its prompts in order; `during(lab)` runs once the answers are given."""
    async def scenario():
        future = lab.plugin.request_execute(request)
        for ok in answers:
            await answer(lab, ok)
        if during is not None:
            await during(lab)
        result = await future
        await asyncio.sleep(0)
        return result
    return run_guarded(scenario())


def insert_route(lab: ServoLab, gap: float) -> RouteResult:
    """A fake insert plan whose last row is the live joints with the left shoulder turned so its end is `gap` m off."""
    robot = lab.measured()
    names = [name for arm in ARMS for name in arm_joint_names(arm)]
    live = np.array([robot.joints[name] for name in names])
    turn = 0.01
    for _ in range(3):
        end = live.copy()
        end[0] += turn
        got = insert_end_gap(robot, names, end, ARMS)
        if not got:  # a zero gap asked for: the end is the live joints
            break
        turn *= gap / got
    end = live.copy()
    end[0] += turn
    trajectory = Trajectory(lab.at.action.robot, tuple(names), (tuple(live.tolist()), tuple(end.tolist())))
    return RouteResult(MovementResult("solved", trajectory=trajectory), Routine.INSERT, planner="fake")


# --- --- --- --- --- THE PARTS --- --- --- --- ---

def test_the_servo_part_follows_the_steps_that_get_one():
    """The transfer and the approach steps end with a servo part; its text names the arms and the target."""
    loaded = purged_process()
    transfer = servo_part(loaded, "B20_J", step(loaded, "B20_J_transfer").movement_index, True)
    assert (transfer.index, transfer.kind, transfer.tools) == (SERVO_INDEX, SERVO_PART, ARMS)
    assert transfer.text == "servo left_ur_arm, right_ur_arm onto the insert start"
    approach = servo_part(loaded, "B21_H", 0, True)
    assert approach.text == "servo ur_arm onto the approach pose" and approach.movement == "B21_H_approach"
    assert servo_part(loaded, "B20_R", step(loaded, "B20_R_home").movement_index, True) is None


# --- --- --- --- --- (1) TOOL0 --- --- --- --- ---

@pytest.mark.safety
def test_tool0_servo_converges_in_one_pass_and_is_recorded():
    """Cindy at the transfer's end, base measured 10 mm off: prompt, one pass, converged; the record has 2 readings."""
    lab = ServoLab()
    seen = {}

    async def look(lab):
        await sent(lab, 1)
        lab.plugin.draw(lab.ctx)
        seen["line"] = lab.plugin._panel._servo.content
        seen["tracked"] = TRACKED in lab.ctx.world.tracked_objects

    result = execute(lab, lab.servo_request(), True, during=look)
    assert result.status == DONE and result.done == (SERVO_INDEX,), result.reason
    assert "converged on tool0" in lab.ended() and lab.published() == 1 and lab.free()
    assert "servo 0/8 · tool0 L 10.00 R 10.00 mm" in seen["line"] and seen["tracked"]
    assert not lab.ctx.world.tracked_objects
    text, warn = lab.prompts()[0]
    assert text.startswith("cindy (a200-0806): B20_J_transfer — SERVO 1/8 onto the insert start "
                           "(tool0): stub, 2 rows")
    assert "flanges move L 10.00 mm, R 10.00 mm" in text and not warn
    event, fields = next((event, fields) for event, fields in lab.run.records if event == "servo")
    assert (fields["action"], fields["movement"], fields["kind"], fields["figure"]) == ("B20_J", "B20_J_transfer",
                                                                                        "servo", plot_servo)
    record = fields["file"]
    assert record["servo_feedback"] == "tool0" and record["planner_mode"] == "transfer"
    assert [reading["iter"] for reading in record["servoing_data"]] == [0, 1]
    assert record["servoing_data"][1]["tool0"]["left"]["pos_norm_mm"] < 0.2
    assert record["outcome"] == {"status": DONE, "reason": lab.ended(), "passes": 1}
    exec_record = lab.run.files()[-1]
    assert exec_record["part"]["kind"] == SERVO_PART and "servo_file" in exec_record


# --- --- --- --- --- (2) THE MOCAP BAR --- --- --- --- ---

@pytest.mark.safety
def test_bar_servo_closes_a_flange_side_error_in_one_pass_and_carries_it():
    """Servo by bar: markers 3 mm / 0.1° off in the bar's frame: one pass puts the bar on its design, the carry set."""
    lab = ServoLab()
    lab.run.request_settings(servo_by_bar=True)
    lab.markers(offset(0.003, roll_deg=0.1), where="bar")
    result = execute(lab, lab.servo_request(), True)
    assert result.status == DONE and result.done == (SERVO_INDEX,), result.reason
    assert "converged on the MoCap bar" in lab.ended() and lab.published() == 1
    assert "(the MoCap bar B20)" in lab.prompts()[0][0]
    record = lab.servo_record()
    assert record["servo_feedback"] == "mocap_bar" and record["planner_mode"] == "transfer+mocap_bar"
    assert len(record["moves"]) == 1 and record["servoing_data"][1]["bar"]["usable"]
    # Every pass aimed T_corr . F of the reading before.
    reading = record["servoing_data"][0]
    T_corr = np.array(reading["bar"]["T_corr"])
    for flange, target in lab.planner.requests[0].servo.targets.items():
        side = "left" if "left" in flange else "right"
        position, orientation = reading["tool0"][side]["observed_pose"]
        expected = T_corr @ Pose(tuple(position), tuple(orientation)).matrix()
        assert np.allclose(target.matrix(), expected, atol=1e-9)
    carried = lab.plugin.carried
    assert carried is not None and carried.loaded is lab.loaded and carried.serial == lab.serial
    assert (carried.action, carried.insert) == ("B20_J", step(lab.loaded, "B20_J_insert").movement_index)


@pytest.mark.safety
def test_bar_servo_closes_a_world_frame_error_within_three_passes():
    """Markers 3 mm / 0.1° off in the world: converged in at most 3 passes, each aiming T_corr . F of the last."""
    lab = ServoLab()
    lab.run.request_settings(servo_by_bar=True)
    lab.markers(offset(0.0, 0.003, roll_deg=0.1), where="world")
    result = execute(lab, lab.servo_request(), True)
    assert result.status == DONE and "converged on the MoCap bar" in lab.ended(), result.reason
    record = lab.servo_record()
    assert 1 <= len(record["moves"]) <= 3 and lab.published() == len(record["moves"])
    for k, request in enumerate(lab.planner.requests):
        reading = record["servoing_data"][k]
        T_corr = np.array(reading["bar"]["T_corr"])
        for flange, target in request.servo.targets.items():
            position, orientation = reading["tool0"]["left" if "left" in flange else "right"]["observed_pose"]
            assert np.allclose(target.matrix(), T_corr @ Pose(tuple(position), tuple(orientation)).matrix(), atol=1e-9)
    assert lab.plugin.carried is not None


@pytest.mark.safety
def test_bar_already_on_target_moves_nothing():
    """The MoCap bar on its design before any move: done at once, nothing sent, the carry still set."""
    lab = ServoLab(shift=(0.0, 0.0, 0.0))
    lab.run.request_settings(servo_by_bar=True)
    lab.markers()
    result = execute(lab, lab.servo_request())
    assert result.status == DONE and result.done == (SERVO_INDEX,) and "already on its target" in lab.ended()
    assert lab.published() == 0 and lab.servo_record()["moves"] == [] and lab.plugin.carried is not None


# --- --- --- --- --- (3) UNUSABLE READINGS --- --- --- --- ---

@pytest.mark.safety
@pytest.mark.parametrize("markers, words", [
    (dict(age=0.6), "0.6 s old"),
    (dict(pattern=0.0031), "rms 3.1 mm"),
    (dict(E=offset(0.031)), "beyond the 30 mm / 3 deg gate"),
    (dict(count=5), "only 5 of 8"),
])
def test_an_unusable_bar_reading_leaves_the_loop_on_tool0(markers, words):
    """Stale, loose, far-off or too few markers: never steered on, said once; the loop converges on tool0."""
    lab = ServoLab()
    lab.run.request_settings(servo_by_bar=True)
    lab.markers(**markers)
    result = execute(lab, lab.servo_request(), True)
    assert result.status == DONE and "converged on tool0" in lab.ended(), result.reason
    record = lab.servo_record()
    assert record["servo_feedback"] == "tool0" and "(tool0)" in lab.prompts()[0][0]
    reasons = [reading["bar"]["reason"] for reading in record["servoing_data"]]
    assert len(reasons) == 2 and all(words in reason for reason in reasons), reasons
    assert sum("the MoCap bar is not usable" in line for line in lab.ctx.lines("warn")) == 1


@pytest.mark.safety
def test_a_fit_lost_after_engaging_stops_the_loop():
    """Good markers at the first reading, none after the move: failed "fit is lost", the arms stay, not done."""
    lab = ServoLab()
    lab.run.request_settings(servo_by_bar=True)
    lab.markers(offset(0.0, 0.003), where="world")

    async def lose(lab):
        await sent(lab, 1)
        lab.stream = lambda _t: None

    result = execute(lab, lab.servo_request(), True, during=lose)
    assert result.status == FAILED and result.done == () and "fit is lost" in result.reason, result.reason
    assert lab.servo_record()["outcome"]["status"] == FAILED and lab.published() == 1


# --- --- --- --- --- (4) (5) CANCELS --- --- --- --- ---

@pytest.mark.safety
def test_cancel_at_the_first_prompt_sends_nothing_and_records_only_with_a_bar_reading():
    """Cancel at pass 1's prompt: cancelled, nothing sent; Cindy's run (a bar measured) is recorded, Alice's not."""
    lab = ServoLab()
    result = execute(lab, lab.servo_request(), False)
    assert result.status == CANCELLED and result.done == () and lab.published() == 0
    assert "stopped at pass 1" in result.reason and lab.free()
    record = lab.servo_record()
    assert len(record["servoing_data"]) == 1 and record["outcome"]["status"] == CANCELLED
    alice = ServoLab("B21_H_approach")
    result = execute(alice, alice.servo_request(), False)
    assert result.status == CANCELLED and alice.servo_record() is None and alice.published() == 0


@pytest.mark.safety
def test_cancel_during_the_first_move_without_a_bar_is_still_recorded():
    """Alice's approach, Cancel while pass 1 moves: held, cancelled; the record has its one reading and the move."""
    lab = ServoLab("B21_H_approach")

    async def cut(lab):
        await sent(lab, 1)
        lab.plugin.request_cancel()

    result = execute(lab, lab.servo_request(), True, during=cut)
    assert result.status == CANCELLED and result.done == () and lab.holds() == [1]
    record = lab.servo_record()
    assert record["outcome"]["status"] == CANCELLED
    assert "during pass 1's move; every arm held" in record["outcome"]["reason"]
    assert len(record["servoing_data"]) == 1 and len(record["moves"]) == 1


@pytest.mark.safety
def test_a_cancel_after_the_bar_engaged_still_carries():
    """Servo by bar, Cancel while pass 2 moves: not done, but the bar correction is carried into the insert's plan."""
    lab = ServoLab()
    lab.run.request_settings(servo_by_bar=True)
    lab.planner.fraction = 0.5
    lab.markers(offset(0.0, 0.003), where="world")

    async def cut_pass_two(lab):
        await sent(lab, 2)
        lab.plugin.request_cancel()

    result = execute(lab, lab.servo_request(), True, during=cut_pass_two)
    assert result.status == CANCELLED and result.done == () and lab.holds() == [1, 1]
    carried = lab.plugin.carried
    assert carried is not None and (carried.action, carried.serial) == ("B20_J", lab.serial)


@pytest.mark.safety
def test_cancel_during_the_second_pass_holds_every_arm_and_exec_again_runs_the_servo_alone():
    """The whole transfer step, then Cancel while pass 2 moves: held, the step's parts done, the servo not.

    Exec again then runs only the servo part, to done.
    """
    lab = ServoLab(shift=(0.0, 0.0, 0.0))
    request = lab.step_request()
    assert [part.kind for part in request.parts] == ["motion", "operator", "grasp", SERVO_PART]

    async def cut_pass_two(lab):
        await sent(lab, 2)  # the step's motion, then servo pass 1
        lab.shift((0.003, 0.0, 0.0))  # read after pass 1: 3 mm left, so a pass 2 comes
        await sent(lab, 3)
        lab.plugin.request_cancel()

    result = execute(lab, request, True, True, True, True, during=cut_pass_two)
    assert result.status == CANCELLED and "operator" in result.reason, result.reason
    assert result.done == lab.operator_step().indices == (3, 1, 2) and lab.holds() == [1, 1] and lab.free()
    record = lab.servo_record()
    assert record["outcome"]["status"] == CANCELLED and len(record["servoing_data"]) == 2
    assert len(record["moves"]) == 2 and record["moves"][1]["outcome"] is None
    exec_record = lab.run.files()[-1]
    assert exec_record["part"]["kind"] == SERVO_PART and exec_record["outcome"]["result"] == CANCELLED
    settle(lab)
    again = execute(lab, lab.servo_request(), True)
    assert again.status == DONE and again.done == (SERVO_INDEX,), again.reason


# --- --- --- --- --- (6) (7) OTHERS MOVING, THE BASE --- --- --- --- ---

@pytest.mark.safety
def test_another_robot_starting_to_move_during_a_pass_interrupts_and_holds():
    """Belle starts a trajectory while Cindy's pass moves: interrupted, every arm held, recorded."""
    lab = ServoLab()
    belle = lab.belle.arms["ur_arm"]

    async def start_belle(lab):
        await sent(lab, 1)
        for _ in range(3):
            await lab.ctx.next_tick()
        with from_outside(lab.belle):
            belle.send_joint_move(belle.joint_vector() + np.radians([30, 0, 0, 0, 0, 0]))

    result = execute(lab, lab.servo_request(), True, during=start_belle)
    assert result.status == FAILED and "a200-0805 ur_arm is running a trajectory" in result.reason
    assert lab.holds() == [1, 1] and result.done == ()
    assert lab.run.files()[-1]["outcome"]["status"] == INTERRUPTED and lab.servo_record() is not None


@pytest.mark.safety
def test_another_robot_moved_just_before_a_pass_is_refused():
    """Belle moves after pass 1 arrived: pass 2 is refused before anything is planned or sent."""
    lab = ServoLab()
    lab.planner.fraction = 0.5  # half the correction per pass, so a second pass comes
    belle = lab.belle.arms["ur_arm"]

    async def nudge_belle(lab):
        while lab.serial not in lab.plugin._moved_at:  # pass 1 arrived: the 2 s settle before reading 1 runs
            await asyncio.sleep(0)
        with from_outside(lab.belle):
            belle.send_joint_move(belle.joint_vector() + np.radians([30, 0, 0, 0, 0, 0]))

    result = execute(lab, lab.servo_request(), True, during=nudge_belle)
    assert result.status == REFUSED and "a200-0805 ur_arm" in result.reason, result.reason
    assert len(lab.cindy._multi_arm.messages) == 1 and len(lab.planner.requests) == 1 and result.done == ()


@pytest.mark.safety
def test_the_base_moved_3_mm_after_confirm_is_refused():
    """The base moves between the prompt and Confirm: refused after Confirm, nothing sent."""
    lab = ServoLab()

    async def scenario():
        future = lab.plugin.request_execute(lab.servo_request())
        while lab.plugin.prompt is None:
            await asyncio.sleep(0)
        lab.shift((0.0, 0.003, 0.0))
        await answer(lab, True)
        return await future

    result = run_guarded(scenario())
    assert result.status == REFUSED and "base" in result.reason and "replan" in result.reason, result.reason
    assert lab.published() == 0


# --- --- --- --- --- (8) AFTER THE INSERT --- --- --- --- ---

@pytest.mark.safety
def test_after_the_insert_the_gate_the_load_guard_and_linear_only():
    """The flanges 3.1 mm from the insert's end: refused; 2.9 mm: runs, straight lines only, the load guard stops it.

    The force at the left flange changes by 25 N after pass 1: stopped, not done.
    """
    lab = ServoLab("B20_J_insert")
    insert = lab.at.movement_index
    for arm in ARMS:
        lab.cindy.arms[arm].state.wrench = np.array([5.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    result = execute(lab, lab.servo_request(route=insert_route(lab, 0.0031), route_movement=insert))
    assert result.status == REFUSED and "3.1 mm from the end of the insert" in result.reason, result.reason
    assert lab.published() == 0

    async def push(lab):
        await sent(lab, 1)
        lab.cindy.arms["left_ur_arm"].state.wrench = np.array([30.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    result = execute(lab, lab.servo_request(route=insert_route(lab, 0.0029), route_movement=insert), True,
                     during=push)
    assert result.status == FAILED and "changed by 25.0 N" in result.reason, result.reason
    ask = lab.planner.requests[0].servo
    assert ask.linear_only and ask.held and result.done == () and lab.published() == 1
    assert "onto the insert end (tool0)" in lab.prompts()[0][0]
    record = lab.servo_record()
    assert record["servoing_data"][1]["force_change_N"]["left"] == pytest.approx(25.0)
    assert record["settings"]["linear_only"] and record["settings"]["max_force_change_N"] == 20.0


@pytest.mark.safety
def test_after_the_insert_without_a_plan_is_refused_and_no_force_reading_warns():
    """A servo Exec on its own after the insert is refused (no plan to gate on); without F/T the guard is OFF, said."""
    lab = ServoLab("B20_J_insert", shift=(0.0, 0.0, 0.0))
    result = execute(lab, lab.servo_request())
    assert result.status == REFUSED and "no insert plan in this request" in result.reason, result.reason
    assert lab.published() == 0 and not lab.planner.requests
    result = execute(lab, lab.servo_request(route=insert_route(lab, 0.0), route_movement=lab.at.movement_index))
    assert result.status == DONE and "nothing left to plan" in lab.ended(), result.reason
    assert any("load guard is OFF" in line for line in lab.ctx.lines("warn"))


# --- --- --- --- --- (9) (10) PROMPTS, THE DRY RUN --- --- --- --- ---

@pytest.mark.safety
def test_a_later_pass_over_five_degrees_prompts_with_a_warning_and_one_of_four_does_not():
    """Pass 1 always asks; pass 2 travelling 6° asks as a warning; pass 3 travelling 4° runs unattended."""
    lab = ServoLab()
    lab.planner.fraction = 0.5
    lab.planner.detours = {2: radians(6.0), 3: radians(4.0)}
    result = execute(lab, lab.servo_request(), True, True)
    assert result.status == DONE and "converged on tool0" in lab.ended(), result.reason
    prompts = lab.prompts()
    assert len(prompts) == 2 and not prompts[0][1] and prompts[1][1]
    assert "SAFEGUARD servo 2/8: this path travels 6.0°" in prompts[1][0]
    moves = lab.servo_record()["moves"]
    assert [move["prompted"] for move in moves[:3]] == [True, True, False]
    assert moves[1]["excursion_deg"] == pytest.approx(6.0) and moves[2]["excursion_deg"] == pytest.approx(4.0)
    assert 3 < len(moves) <= 8 and lab.published() == len(moves)


@pytest.mark.safety
def test_a_dry_run_builds_one_pass_and_is_never_done():
    """Dry run: reading 0, pass 1 planned and built, the prompt says DRY RUN, nothing sent, nothing done."""
    lab = ServoLab()
    lab.plugin.request_dry_run(True)
    result = execute(lab, lab.servo_request(), True)
    assert result.status == DRY_RUN and result.done == () and lab.published() == 0
    assert lab.prompts()[0][0].startswith("cindy (a200-0806): DRY RUN — ")
    record = lab.servo_record()
    assert record["dry_run"] and [move["outcome"] for move in record["moves"]] == [SENT_DRY]
    assert any("DRY RUN a200-0806 left_ur_arm" in line for line in lab.ctx.lines("warn"))


# --- --- --- --- --- (11) (12) ONE ARM, A PLAN FROM BEFORE --- --- --- --- ---

@pytest.mark.safety
def test_alice_approach_servos_one_arm_along_two_rows():
    """Alice at the approach's end, base 10 mm off: one arm, the two-row path, converged; no bar in the record."""
    lab = ServoLab("B21_H_approach")
    result = execute(lab, lab.servo_request(), True)
    assert result.status == DONE and result.done == (SERVO_INDEX,), result.reason
    ask = lab.planner.requests[0].servo
    assert list(ask.targets) == ["robots/alice/ur_arm_tool0"] and not ask.held
    assert lab.published() == 1 and len(lab.alice.arms["ur_arm"]._trajectory.messages) == 1
    text = lab.prompts()[0][0]
    assert "onto the approach pose (tool0)" in text and "flanges move arm 10.00 mm" in text
    record = lab.servo_record()
    assert record["planner_mode"] == "single_arm_joint" and record["bar_name"] is None
    assert list(record["servoing_data"][0]["tool0"]) == ["arm"] and not lab.ctx.world.tracked_objects


@pytest.mark.safety
def test_a_plan_made_before_a_servo_pass_is_refused_at_exec():
    """After a servo moved the arms, a motion plan made before it is refused: replan."""
    lab = ServoLab()
    before = lab.ctx.t
    result = execute(lab, lab.servo_request(), True)
    assert result.status == DONE, result.reason
    request = lab.step_request()
    stale = replace(request, basis=replace(request.basis, time=before))
    result = execute(lab, stale)
    assert result.status == REFUSED and result.reason == "the arms moved since this plan was made: replan"
    assert lab.published() == 1
    fresh = execute(lab, request, True, False)
    assert fresh.status == CANCELLED and fresh.done == (3,)
