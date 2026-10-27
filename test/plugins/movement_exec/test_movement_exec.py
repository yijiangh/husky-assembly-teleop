"""The executor on the purged process with fake robots that follow what they are sent.

Refusals, the lease, prompts, cancels, records, the transfer workaround's parts, the dry run, and the compliant parts:
the insert, the retreat with its ungrasp, the Robotiq handoff.
"""

from __future__ import annotations

import asyncio
import json
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
from planning_fixtures import purged_process, step
from plugin_fakes import FakeContext, from_outside
from robot_lab import LEFT, FakeRun, RobotLab

from husky_assembly_teleop.monitor import HuskyMonitor
from husky_assembly_teleop.plugin_api.context import PluginContext
from husky_assembly_teleop.plugins.movement_exec import plugin as exec_module
from husky_assembly_teleop.plugins.movement_exec.plugin import MovementExecPlugin
from husky_assembly_teleop.plugins.movement_exec.sequences import PartOperator
from husky_assembly_teleop.robot_interface.base import BaseState, FollowerState
from husky_assembly_teleop.robot_interface.end_effectors import ScaffoldingV3
from husky_assembly_teleop.robot_interface.robot_lease import NotLeased
from husky_assembly_teleop.shared.execution.basis import basis_of
from husky_assembly_teleop.shared.execution.retreat import TAIL_STEP
from husky_assembly_teleop.shared.execution.stream import arm_base_in_world, tool0_fk
from husky_assembly_teleop.shared.execution.tools import ROBOTIQ_CLOSE
from husky_assembly_teleop.shared.movement_exec_contract import (CANCELLED, DEFAULT_TRAJ_TIME, DONE, DRY_RUN, FAILED,
                                                                 HANDOFF_PART, HOLD_NOT_SENT, INSERT_PART, LATER,
                                                                 REFUSED, REFUSED_PART, RETREAT_PART, SERVO_INDEX,
                                                                 SERVO_PART,
                                                                 WITH_MOTION_PART, ExecRequest, PartPlan, part_plan,
                                                                 step_parts)
from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names
from husky_assembly_teleop.shared.assembly_planning.movement import Routine
from husky_assembly_teleop.shared.assembly_planning.route_result import RouteResult
from husky_assembly_teleop.shared.assembly_planning.servo_route import ServoStep
from husky_assembly_teleop.shared.assembly_planning.steps import operator_steps
from husky_assembly_teleop.shared.runs.runs import RunSettings
from scaffolding_assembly_core.geometry import Pose, compose
from scaffolding_assembly_core.process.solutions import MovementResult, Trajectory


#: Wall-clock seconds a scenario may take: a guard missing makes a test wait forever, this makes it fail instead.
WALL_LIMIT = 30.0


def run_guarded(scenario):
    """Run a scenario in a new event loop, failing (not hanging) after WALL_LIMIT seconds of wall clock."""
    return asyncio.run(asyncio.wait_for(scenario, WALL_LIMIT))


def nothing_left(request) -> asyncio.Future:
    """A servo pass answered at once with nothing left to plan: a step's servo part is done without a move."""
    future = asyncio.get_running_loop().create_future()
    future.set_result(ServoStep(RouteResult(MovementResult("not_planned", "nothing left to plan"), None,
                                            planner="stub"), {}, True))
    return future


class Lab(RobotLab):
    """The three robots live and following commands, the run at `current`, movement_exec set up on them."""

    def __init__(self, current=("B20_R", 2), **driver):
        """Build everything; `driver` options go to every arm's driver."""
        super().__init__(**driver)
        self.loaded = purged_process()
        self.run = FakeRun(self.loaded, current)
        self.ctx.plugins = {"assembly_run": self.run,
                            "motion_planner": SimpleNamespace(request_servo_step=nothing_left)}
        self.plugin = MovementExecPlugin()
        self.plugin.setup(self.ctx)

    def free(self) -> bool:
        """Whether every robot is given back: none leased."""
        return all(robot.lease_slot.lease is None for robot in self.ctx.world.robots.values())

    def request(self, movement: str = "B20_R_home", serial: str = "a200-0806", delta: float = 0.2,
                route_movement: int | None = None, **changes) -> ExecRequest:
        """A request for the step holding `movement`, its plan moving each arm's first joint by `delta`."""
        at = step(self.loaded, movement)
        action = at.action.id
        steps = operator_steps(self.loaded.process, at.action, True)
        operator = next(each for each in steps if at.movement_index in (each.primary, *each.after_motion))
        robot = self.robots[serial]
        arms = list(robot.arms)
        names = [name for arm in arms for name in arm_joint_names(arm)]
        start = np.concatenate([robot.arms[arm].joint_vector() for arm in arms])
        goal = start.copy()
        goal[::6] += delta
        rows = np.array([start, (start + goal) / 2, goal])
        acting = at.action.robot
        route = RouteResult(MovementResult("solved", trajectory=Trajectory(acting, tuple(names),
                                                                           tuple(map(tuple, rows.tolist())))),
                            Routine.FREE_HOME, planner="fake")
        primary = next(each for each in self.loaded.steps if each.action.id == action
                       and each.movement_index == operator.primary)
        basis = basis_of(self.ctx.scene.snapshot, self.loaded, primary, serial, tuple(names),
                         dict(zip(names, start.tolist())), True, self.run.view.statuses, False,
                         self.ctx.t)
        request = ExecRequest(self.loaded, action, operator,
                              step_parts(self.loaded, action, operator, self.configs, True), serial,
                              self.loaded.process.robots[acting].name, route,
                              operator.primary if route_movement is None else route_movement, basis, 2.0,
                              primary.movement.id)
        return replace(request, **changes)


async def answer(lab: Lab, ok: bool = True, count: int = 1) -> None:
    """Answer the next `count` prompts, each a little over 0.5 s after it opened; stop early once nothing runs."""
    for _ in range(count):
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


async def settle(lab: Lab, seconds: float = 2.5) -> None:
    """Let ROS time pass, the arms still."""
    until = lab.ctx.t + seconds
    while lab.ctx.t < until:
        await lab.ctx.next_tick()


def execute(lab: Lab, request: ExecRequest, *answers: bool, during=None):
    """Run a request to its end, answering its prompts in order; `during(lab)` runs once the first answer is in."""
    async def scenario():
        future = lab.plugin.request_execute(request)
        held = lab.robot_lease = lab.cindy.lease_slot.lease
        for ok in answers:
            await answer(lab, ok)
        if during is not None:
            await during(lab)
        result = await future
        await asyncio.sleep(0)
        return result, held
    return run_guarded(scenario())


@pytest.mark.safety
def test_a_motion_is_confirmed_sent_arrives_and_is_recorded():
    """B20_R_home: prompt, Confirm, one multi-arm message, arrived; done, every robot leased and then given back."""
    lab = Lab()
    result, held = execute(lab, lab.request(), True)
    assert result.status == DONE and result.done == (2,), result.reason
    assert held is not None and set(held.serials) == set(lab.ctx.world.robots) and lab.free()
    assert lab.robot_lease.holder == "movement_exec: exec B20_R_home" and not lab.robot_lease.active
    assert len(lab.cindy._multi_arm.messages) == 1
    events = [event for event, _fields in lab.run.records]
    assert events == ["prompt", "confirm", "exec"]
    prompt = lab.run.records[0][1]["data"]["text"]
    assert prompt.startswith("cindy (a200-0806): B20_R_home — move left_ur_arm, right_ur_arm along the planned path")
    record, = lab.run.files()
    assert json.loads(json.dumps(record))["outcome"]["status"] == "ARRIVED"
    assert record["basis"]["serial"] == "a200-0806" and record["plan"]["planner"] == "fake"
    assert np.allclose(result.end["left_ur_arm_shoulder_pan_joint"], LEFT[0] + 0.2, atol=1e-3)
    # * Where the robot stood and when, at the part's start and end.
    assert record["exec"]["ros_time_start"] < record["exec"]["ros_time_end"]
    assert len(record["exec"]["base_at_start"]) == 7 and len(record["exec"]["base_at_end"]) == 7


@pytest.mark.safety
def test_every_robot_is_back_when_the_result_arrives():
    """The result resolves with every robot given back already, so its caller may start the next step at once."""
    lab = Lab()

    async def scenario():
        future = lab.plugin.request_execute(lab.request())
        await answer(lab, True)
        result = await future
        return result, lab.free()

    result, free = run_guarded(scenario())
    assert result.status == DONE and free


@pytest.mark.safety
def test_a_request_whose_task_cannot_start_gives_every_robot_back(monkeypatch):
    """Spawning the step's task raises, as a closing plugin's context does: every robot is given back, then raised.

    Else every other plugin would be refused every lease until a restart.
    """
    lab = Lab()

    def closed(_label, work, survives_soft_stop=False):
        work.close()
        raise RuntimeError("plugin 'movement_exec' is stopped; no new tasks")

    monkeypatch.setattr(lab.ctx, "spawn", closed)

    async def scenario():
        with pytest.raises(RuntimeError, match="no new tasks"):
            lab.plugin.request_execute(lab.request())

    run_guarded(scenario())
    assert lab.free() and lab.plugin.active == "" and lab.published() == 0
    other = FakeContext(name="robot_control")
    other.world = lab.ctx.world
    assert other.take_lease(["a200-0805"], "teleop").holder == "robot_control: teleop"


@pytest.mark.safety
def test_a_refused_lease_is_busy_and_leases_nothing(monkeypatch):
    """The robots' lease refused: busy, naming the lease, nothing leased or sent."""
    lab = Lab()

    def refused(_serials, _label):
        raise NotLeased("robot a200-0806 is leased by a plugin: teleop")

    monkeypatch.setattr(lab.ctx, "take_lease", refused)
    result, held = execute(lab, lab.request())
    assert result.status == REFUSED and result.reason == "busy: robot a200-0806 is leased by a plugin: teleop"
    assert held is None and lab.free() and lab.published() == 0


@pytest.mark.safety
def test_a_second_request_is_busy():
    """While one request runs, another is refused at once."""
    lab = Lab()

    async def scenario():
        first = lab.plugin.request_execute(lab.request())
        second = await lab.plugin.request_execute(lab.request())
        lab.plugin.request_cancel()
        return second, await first

    second, first = run_guarded(scenario())
    assert second.status == REFUSED and "busy" in second.reason and first.status == CANCELLED
    assert lab.published() == 0 and lab.free()


@pytest.mark.parametrize("changes, words", [
    (dict(serial="a200-0804"), "B20_R is for a200-0806, not a200-0804"),
    (dict(route_movement=1), "the plan is for movement 1"),
    (dict(route=None), "no plan"),
])
@pytest.mark.safety
def test_a_request_for_another_robot_or_movement_is_refused(changes, words):
    """The wrong robot, a plan for another movement, or no plan: refused, nothing sent, no lease."""
    lab = Lab()
    result, held = execute(lab, lab.request(**changes))
    assert result.status == REFUSED and words in result.reason
    assert held is None and lab.published() == 0 and lab.run.records == []


@pytest.mark.safety
def test_a_base_moved_3_mm_since_planning_is_refused_at_the_request():
    """The base 3 mm from where the plan was made: refused by the request's own check, before any prompt."""
    lab = Lab()
    request = lab.request()
    lab.bases["a200-0806"] = Pose((3.003, 1.0, 0.0))
    lab.tick(lab.ctx.t)
    result, held = execute(lab, request)
    assert result.status == REFUSED and held is None and lab.published() == 0
    assert "a200-0806's base is 3.0 mm" in result.reason and "from where the plan was made" in result.reason


@pytest.mark.safety
def test_a_base_moved_3_mm_at_the_prompt_is_refused_after_confirm():
    """The base moves 3 mm while the prompt waits: refused by the check after Confirm (its wording), nothing sent."""
    lab = Lab()

    async def scenario():
        future = lab.plugin.request_execute(lab.request())
        while lab.plugin.prompt is None:
            await asyncio.sleep(0)
        lab.bases["a200-0806"] = Pose((3.003, 1.0, 0.0))
        await answer(lab, True)
        return await future

    result = run_guarded(scenario())
    assert result.status == REFUSED and "a200-0806's base moved 3.0 mm" in result.reason
    assert "since planning" in result.reason and lab.published() == 0


@pytest.mark.safety
def test_a_plan_gone_stale_at_the_prompt_is_refused_after_confirm():
    """Belle moves 1.1 cm while the prompt waits: refused after Confirm, nothing sent."""
    lab = Lab()

    async def scenario():
        future = lab.plugin.request_execute(lab.request())
        while lab.plugin.prompt is None:
            await asyncio.sleep(0)
        lab.bases["a200-0805"] = Pose((1.011, 0.0, 0.0))
        await answer(lab, True)
        return await future

    result = run_guarded(scenario())
    assert result.status == REFUSED and "a200-0805 moved since planning" in result.reason and lab.published() == 0


@pytest.mark.safety
def test_a_cancel_at_the_prompt_sends_nothing():
    """Cancel at the prompt: cancelled, nothing sent, a cancel event and the part's record."""
    lab = Lab()
    result, _held = execute(lab, lab.request(), False)
    assert result.status == CANCELLED and result.done == () and lab.published() == 0
    assert [event for event, _fields in lab.run.records] == ["prompt", "cancel", "exec"]
    assert lab.free()


@pytest.mark.safety
def test_a_cancel_during_the_motion_holds_every_arm():
    """Cancel while the arms move: every arm held, cancelled by the operator, the lease given back."""
    lab = Lab()

    async def cancel_soon(lab):
        while not lab.cindy._multi_arm.messages:
            await asyncio.sleep(0)
        lab.plugin.request_cancel()

    result, _held = execute(lab, lab.request(), True, during=cancel_soon)
    assert result.status == CANCELLED and "operator" in result.reason
    holds = [sum(len(message.points) == 1 for message in arm._trajectory.messages) for arm in lab.cindy.arms.values()]
    assert holds == [1, 1] and lab.free()
    (event, fields), (last, _fields) = lab.run.records[-2:]
    assert (event, last, fields["data"]["reason"]) == ("cancel", "exec", "operator")


@pytest.mark.safety
def test_a_request_cancelled_before_it_starts_gives_the_robot_back():
    """A task cancelled before its first step (a soft stop in the same tick): cancelled, the lease given back."""
    lab = Lab()

    async def scenario():
        future = lab.plugin.request_execute(lab.request())
        lab.plugin._task.cancel()
        return await future

    result = run_guarded(scenario())
    assert result.status == CANCELLED and lab.free() and lab.plugin.prompt is None


@pytest.mark.safety
def test_an_error_inside_holds_and_gives_the_robot_back(monkeypatch):
    """A part that raises: failed, every arm held, the lease given back; the part recorded with the error."""
    lab = Lab()

    async def broken(job, run):
        raise RuntimeError("a bug")

    monkeypatch.setattr(exec_module, "run_part", broken)

    async def scenario():
        future = lab.plugin.request_execute(lab.request())
        result = await future
        await asyncio.sleep(0)
        return result

    result = run_guarded(scenario())
    assert result.status == FAILED and "a bug" in result.reason and lab.free()
    assert all(len(arm._trajectory.messages) == 1 for arm in lab.cindy.arms.values())
    record, = lab.run.files()
    assert record["outcome"]["result"] == "failed" and "RuntimeError('a bug')" in record["outcome"]["reason"]
    assert [event for event, _fields in lab.run.records] == ["exec", "error"]


def test_the_transfer_with_the_workaround_runs_motion_then_by_hand_then_grasp():
    """B20_J's transfer step: parts (3, 1, 2) then the servo; Cancel at placing the bar by hand leaves done = (3,)."""
    lab = Lab(current=("B20_J", 3))
    request = lab.request("B20_J_transfer")
    assert [(part.index, part.kind) for part in request.parts] == [(3, "motion"), (1, "operator"), (2, "grasp"),
                                                                   (SERVO_INDEX, "servo")]
    result, _held = execute(lab, request, True, False)
    assert result.status == CANCELLED and result.done == (3,)
    assert [file["movement"] for file in lab.run.files()] == ["B20_J_transfer", "B20_J_place_bar"]


def test_the_whole_transfer_step_ends_with_the_grasp_and_its_servo():
    """Confirm the motion, placing the bar by hand, the grasp (stalled); the servo has nothing left: done + SERVO_INDEX.

    This lab's planner answers every servo pass with nothing left to plan, so the servo part is done without a move,
    and still recorded.
    """
    lab = Lab(current=("B20_J", 3))
    result, _held = execute(lab, lab.request("B20_J_transfer"), True, True, True)
    assert result.status == DONE and result.done == (3, 1, 2, SERVO_INDEX), result.reason
    files = lab.run.files()
    assert files[2]["outcome"]["status"] == "stalled"
    assert files[-2]["kind"] == "transfer" and files[-2]["outcome"]["status"] == DONE
    assert files[-1]["part"]["kind"] == "servo" and files[-1]["outcome"]["status"] == "nothing left"


@pytest.mark.safety
def test_a_dry_run_never_gives_done_and_sends_nothing():
    """Dry run: every check and prompt, nothing sent, no part done; turning it off while one runs is refused."""
    lab = Lab()
    lab.plugin.request_dry_run(True)

    async def scenario():
        future = lab.plugin.request_execute(lab.request())
        while lab.plugin.prompt is None:
            await asyncio.sleep(0)
        lab.plugin.request_dry_run(False)
        assert lab.plugin.prompt.text.startswith("DRY RUN — ")
        await answer(lab, True)
        return await future

    result = run_guarded(scenario())
    assert result.status == DRY_RUN and result.done == () and lab.published() == 0
    assert lab.plugin.dry_run and any("Dry run not changed" in line for line in lab.ctx.lines("warn"))
    assert any("DRY RUN a200-0806 left_ur_arm" in line for line in lab.ctx.lines("warn"))
    assert lab.run.files()[0]["outcome"]["dry_run"] is True


@pytest.mark.safety
def test_robotiq_and_refused_parts():
    """Belle's open is a Robotiq part that reaches 0.426; B20_J's transfer with the bar held is a motion (M7)."""
    lab = Lab(current=("B35_H", 1))
    request = lab.request("B35_H_open", serial="a200-0805", route=None)
    assert [part.kind for part in request.parts] == ["robotiq_open"]
    result, _held = execute(lab, request, True)
    assert result.status == DONE and result.done == (1,) and lab.published() == 0
    assert lab.run.files()[0]["outcome"]["status"] == "reached"
    lab = Lab(current=("B20_J", 3))
    held = step_parts(lab.loaded, "B20_J", operator_steps(lab.loaded.process, lab.loaded.process.actions["B20_J"])[3],
                      lab.configs, False)
    assert (held[0].kind, held[0].routine) == ("motion", Routine.TRANSFER_HELD) and held[-1].kind == "servo"


@pytest.mark.safety
def test_the_first_row_sent_is_where_the_arms_are():
    """Arms 1° off the plan's start: the trajectory starts at the live joints; 3° off: refused, nothing sent."""
    lab = Lab()
    request = lab.request()
    for arm in lab.cindy.arms.values():
        arm.state.joint_positions[f"{arm.config.name}_elbow_joint"] += np.radians(1.0)
    run_guarded(settle(lab, 2.5))  # * a move just seen refuses the lease for 2 s
    result, _held = execute(lab, request, True)
    assert result.status == DONE
    first = lab.cindy._multi_arm.messages[0].trajectory1.points[0].positions
    assert np.isclose(first[2], LEFT[2] + np.radians(1.0)) and not np.isclose(first[2], LEFT[2])

    lab = Lab()
    request = lab.request()
    lab.cindy.arms["left_ur_arm"].state.joint_positions["left_ur_arm_elbow_joint"] += np.radians(3.0)
    run_guarded(settle(lab))
    result, _held = execute(lab, request)
    assert result.status == REFUSED and "arm moved 3.0°" in result.reason and lab.published() == 0


@pytest.mark.safety
def test_a_plan_for_another_robot_is_refused():
    """A route whose trajectory is Alice's, asked for Cindy's step: refused, nothing sent."""
    lab = Lab()
    request = lab.request()
    trajectory = replace(request.route.result.trajectory, robot="robots/alice")
    route = replace(request.route, result=replace(request.route.result, trajectory=trajectory))
    result, _held = execute(lab, replace(request, route=route))
    assert result.status == REFUSED and "the plan is for robots/alice" in result.reason and lab.published() == 0


@pytest.mark.safety
def test_stop_all_mid_motion_holds_every_arm_and_is_recorded_as_a_soft_stop():
    """Stop all mid-motion: every arm held, recorded where the arms stopped and when, and a soft_stop event.

    Cancelled by a plugin stop or shutdown, the reason says that instead.
    """
    lab = Lab()

    async def stop_all_soon(lab):
        while not lab.cindy._multi_arm.messages:
            await asyncio.sleep(0)
        for _ in range(5):
            await lab.ctx.next_tick()
        lab.plugin._task.cancel()

    result, _held = execute(lab, lab.request(), True, during=stop_all_soon)
    assert result.status == CANCELLED and "Stop all" in result.reason and lab.free()
    holds = [sum(len(message.points) == 1 for message in arm._trajectory.messages) for arm in lab.cindy.arms.values()]
    assert holds == [1, 1]
    events = [event for event, _fields in lab.run.records]
    assert events[-3:] == ["cancel", "exec", "soft_stop"]
    outcome = lab.run.files()[-1]["outcome"]
    assert outcome["status"] == "cancelled" and outcome["holds"] == {"left_ur_arm": True, "right_ur_arm": True}
    assert outcome["seconds"] > 0.0 and set(outcome["end"]) == {"left_ur_arm", "right_ur_arm"}

    lab = Lab()

    async def plugin_stopped(lab):
        while not lab.cindy._multi_arm.messages:
            await asyncio.sleep(0)
        lab.ctx.closed = True
        lab.plugin._task.cancel()

    result, _held = execute(lab, lab.request(), True, during=plugin_stopped)
    assert "plugin stop or shutdown" in result.reason and "soft_stop" not in [e for e, _f in lab.run.records]
    assert lab.free()


@pytest.mark.safety
def test_a_confirm_clicked_in_the_tick_of_stop_all_runs_after_it_and_moves_nothing():
    """Confirm and Stop all in one tick: Cindy latched and the step cancelled first; the Confirm runs after, moves nothing.

    Stop all keeps queued clicks. The step is recorded as cancelled by the soft stop, no trajectory goes out, and
    every robot is given back.
    """
    lab = Lab()
    # * The plugin's intent queue as the monitor keeps it, and the monitor's own soft stop working on it: it latches
    #   every robot (`HuskyRobotInterface.soft_stop`) and cancels the plugin's tasks but its survivors.
    queued = PluginContext("movement_exec", SimpleNamespace(), view=None, scene=None, kinematics=None)
    latch = lab.cindy.soft_stop_latch

    def soft_stop(reason: str = "") -> None:
        """As `HuskyRobotInterface.soft_stop`: latch Cindy, keeping the reason."""
        latch.stopped, latch.reason = True, reason

    cindy = SimpleNamespace(soft_stop=soft_stop)
    monitor = SimpleNamespace(_world=SimpleNamespace(robots={"a200-0806": cindy}), log_warn=lab.ctx.log_warn,
                              log_error=lab.ctx.log_error,
                              _loaded={"movement_exec": SimpleNamespace(name="movement_exec", ctx=queued)})

    async def scenario():
        future = lab.plugin.request_execute(lab.request())
        while lab.plugin.prompt is None:
            await asyncio.sleep(0)
        prompt = lab.plugin.prompt
        while lab.ctx.t - prompt.opened_at < 0.6:
            await lab.ctx.next_tick()
        queued._tasks.add(lab.plugin._task)  # * the step's task, as the plugin's context keeps it
        queued._survivors |= lab.ctx.survivors
        queued.submit("exec confirm", lab.plugin.request_confirm)  # clicked; it runs at the plugin's next step
        HuskyMonitor._soft_stop(monitor)  # Stop all, first in the same tick
        queued._drain_intents()  # the plugin's step later in the tick: the Confirm runs, after the stop
        return await future

    result = run_guarded(scenario())
    assert queued._intents.empty() and lab.cindy.soft_stopped and latch.reason == "Stop all"
    assert result.status == CANCELLED and "Stop all" in result.reason and lab.free()
    events = [event for event, _fields in lab.run.records]
    assert "confirm" not in events and events[-1] == "soft_stop" and not lab.cindy._multi_arm.messages
    assert all(len(message.points) == 1 for arm in lab.cindy.arms.values() for message in arm._trajectory.messages)


@pytest.mark.safety
def test_a_confirm_reaching_a_step_the_soft_stop_did_not_cancel_sends_nothing():
    """Cindy soft stopped at her prompt, the step not cancelled: Confirm is refused after it, saying Resume.

    As after a clearance trip, or for a task that survived the stop. Nothing is sent, every robot is given back.
    """
    lab = Lab()

    async def scenario():
        future = lab.plugin.request_execute(lab.request())
        while lab.plugin.prompt is None:
            await asyncio.sleep(0)
        lab.cindy.soft_stop_latch.stopped = True
        await answer(lab, True)
        return await future

    result = run_guarded(scenario())
    assert result.status == REFUSED and result.reason == ("B20_R_home: cindy (a200-0806) is soft stopped; press "
                                                          "Resume first")
    assert lab.published() == 0 and lab.free()


@pytest.mark.safety
def test_a_hold_not_sent_on_cancel_is_the_result_record_and_panel_line():
    """Cindy's joint states stop mid-motion, then Cancel: the result, record and panel say HOLD NOT SENT.

    The arms refuse to hold on stale joints, so nothing may say "cancelled, held".
    """
    lab = Lab()

    async def drop_then_cancel(lab):
        while not lab.cindy._multi_arm.messages:
            await asyncio.sleep(0)
        for driver in lab.drivers:
            if getattr(driver, "robot", None) is lab.cindy:
                driver.silent_from = lab.ctx.t
        for _ in range(10):
            await lab.ctx.next_tick()
        lab.plugin.request_cancel()

    result, _held = execute(lab, lab.request(delta=0.6), True, during=drop_then_cancel)
    assert result.status == HOLD_NOT_SENT and "HOLD NOT SENT on left_ur_arm, right_ur_arm" in result.reason
    outcome = lab.run.files()[-1]["outcome"]
    assert outcome["status"] == "hold_not_sent" and outcome["holds"] == {"left_ur_arm": False, "right_ur_arm": False}
    assert outcome["end_stale"] and all(age > 0.5 for age in outcome["end_age"].values())
    lab.plugin.draw(lab.ctx)
    assert any("HOLD NOT SENT" in widget.content for widget in lab.ctx.gui.of_kind("html"))
    panel = lab.plugin._panel
    assert all(line.content.count('class="chip-line"') == 1 for line in (panel._status, panel._phone_status))


@pytest.mark.safety
def test_a_robot_not_live_or_another_current_action_is_refused():
    """Cindy not live: refused before any prompt; the current action changed at the prompt: refused after Confirm.

    Nothing is sent either time.
    """
    lab = Lab()
    request = lab.request()
    lab.live = False
    lab.tick(lab.ctx.t)
    result, _held = execute(lab, request)
    assert result.status == REFUSED and "is not live" in result.reason and lab.run.records[0][0] == "exec"

    lab = Lab()

    async def scenario():
        future = lab.plugin.request_execute(lab.request())
        while lab.plugin.prompt is None:
            await asyncio.sleep(0)
        lab.run.view.current = ("B21_J", 3)
        await answer(lab, True)
        return await future

    result = run_guarded(scenario())
    assert result.status == REFUSED and "no longer B20_R" in result.reason and lab.published() == 0


# --- --- --- --- --- one motion at a time across every robot (review H1) --- --- --- --- ---

@pytest.mark.safety
def test_robot_control_cannot_move_another_robot_while_a_step_runs():
    """Cindy's prompt open: robot_control's Start on Belle is refused by the executor's lease; Confirm moves Cindy."""
    from husky_assembly_teleop.plugins.robot_control.plugin import RobotControlPlugin
    lab = Lab()
    control = RobotControlPlugin()
    control._ctx = lab.ctx
    belle = lab.belle.arms["ur_arm"]

    async def scenario():
        future = lab.plugin.request_execute(lab.request())
        while lab.plugin.prompt is None:
            await asyncio.sleep(0)
        control._start_move(lab.ctx, "a200-0805", belle, "cartesian move", control._stream_cartesian_move,
                            belle.state.tcp_position, belle.state.tcp_orientation)
        await asyncio.gather(*control._trajectory_tasks.values(), return_exceptions=True)
        await answer(lab, True)
        return await future

    result = run_guarded(scenario())
    assert result.status == DONE and not belle._trajectory.messages and len(lab.cindy._multi_arm.messages) == 1
    assert any("a200-0805 is leased by movement_exec: exec B20_R_home" in line for line in lab.ctx.lines("warn"))
    assert lab.free()


@pytest.mark.safety
def test_exec_is_busy_while_another_plugin_leases_any_robot():
    """robot_control's teleop lease on Belle, or a planner's commit lease on Cindy herself: Exec is refused at once.

    Nothing is sent, no robot is leased by movement_exec, and the reason names the lease.
    """
    for serial, holder in (("a200-0805", "robot_control"), ("a200-0806", "base_planner")):
        lab = Lab()
        other = FakeContext(name=holder)
        other.world = lab.ctx.world
        lease = other.take_lease([serial], "teleop" if holder == "robot_control" else "commit plan 1")
        result, held = execute(lab, lab.request())
        assert result.status == REFUSED and result.reason == f"busy: robot {serial} is leased by {lease.holder}"
        # ? `held` is Cindy's lease right after the request: the commit's own, when the commit leases her.
        assert held in (None, lease) and lab.ctx.world.robots[serial].lease_slot.lease is lease
        assert lab.published() == 0 and lab.run.records == []


@pytest.mark.safety
def test_no_other_plugin_gets_a_lease_while_a_step_runs():
    """Cindy's prompt open: robot_control gets no lease on Belle, nor on Cindy; Confirm moves Cindy.

    movement_exec holds every robot's lease the whole step, the prompt included.
    """
    lab = Lab()
    control = FakeContext(name="robot_control")
    control.world = lab.ctx.world
    refusals = []

    async def scenario():
        future = lab.plugin.request_execute(lab.request())
        while lab.plugin.prompt is None:
            await asyncio.sleep(0)
        assert lab.cindy.lease_slot.lease.holder == "movement_exec: exec B20_R_home"
        for serial in ("a200-0805", "a200-0806"):
            try:
                control.take_lease([serial], "teleop")
            except NotLeased as refused:
                refusals.append(str(refused))
        await answer(lab, True)
        return await future

    result = run_guarded(scenario())
    assert result.status == DONE and lab.free() and lab.belle.lease_slot.lease is None
    assert refusals == [f"robot {serial} is leased by movement_exec: exec B20_R_home"
                        for serial in ("a200-0805", "a200-0806")]


@pytest.mark.safety
def test_a_path_check_during_a_step_moves_nothing_and_leases_nothing(monkeypatch):
    """The arm planner checks a joint path for Belle while Cindy's step waits at its prompt.

    It answers, sends nothing and leases nothing; every robot stays leased to the step until it ends.
    """
    from husky_assembly_teleop.plugins.arm_planner import planner_plugin as arm_plugin
    lab = Lab()
    planner = arm_plugin.ArmPlannerPlugin()
    planner.setup(lab.ctx)
    checked = []
    planner._world = SimpleNamespace(sync=lambda _snapshot, _serial: "mirror", close=lambda: None)
    monkeypatch.setattr(arm_plugin, "path_hit",
                        lambda _world, _mirror, arm, waypoints: checked.append((arm, waypoints.shape)))

    async def scenario():
        future = lab.plugin.request_execute(lab.request())
        while lab.plugin.prompt is None:
            await asyncio.sleep(0)
        belle = lab.belle.arms["ur_arm"].joint_vector()
        found = await planner.check_path("a200-0805", "ur_arm", [belle, belle + 0.1])
        during = (lab.cindy.lease_slot.lease, lab.belle.lease_slot.lease, lab.published())
        lab.plugin.request_cancel()
        return found, during, await future

    found, (robot_lease, belle_lease, published), result = run_guarded(scenario())
    planner._executor.shutdown()
    assert found is None and checked == [("ur_arm", (2, 6))]
    assert robot_lease.holder == "movement_exec: exec B20_R_home" and belle_lease is robot_lease
    assert published == 0 and result.status == CANCELLED and lab.free()


@pytest.mark.safety
def test_another_robot_moving_by_other_means_at_the_prompt_is_refused_after_confirm():
    """Belle starts a trajectory from outside the plugins while Cindy's prompt waits: refused, nothing sent."""
    lab = Lab()
    belle = lab.belle.arms["ur_arm"]

    async def scenario():
        future = lab.plugin.request_execute(lab.request())
        while lab.plugin.prompt is None:
            await asyncio.sleep(0)
        with from_outside(lab.belle):
            belle.send_joint_move(belle.joint_vector() + np.radians([30, 0, 0, 0, 0, 0]))
        await answer(lab, True)
        return await future

    result = run_guarded(scenario())
    assert result.status == REFUSED and "a200-0805 ur_arm is running a trajectory" in result.reason
    assert not lab.cindy._multi_arm.messages


@pytest.mark.safety
def test_another_robot_starting_to_move_during_the_motion_holds_the_acting_robots_arms():
    """Belle starts moving while Cindy's arms move: Cindy's arms hold, the part fails naming Belle."""
    lab = Lab()
    belle = lab.belle.arms["ur_arm"]

    async def start_belle(lab):
        while not lab.cindy._multi_arm.messages:
            await asyncio.sleep(0)
        for _ in range(3):
            await lab.ctx.next_tick()
        with from_outside(lab.belle):
            belle.send_joint_move(belle.joint_vector() + np.radians([30, 0, 0, 0, 0, 0]))

    result, _held = execute(lab, lab.request(delta=0.6), True, during=start_belle)
    assert result.status == FAILED and "a200-0805 ur_arm is running a trajectory" in result.reason
    holds = [sum(len(message.points) == 1 for message in arm._trajectory.messages) for arm in lab.cindy.arms.values()]
    assert holds == [1, 1] and lab.run.files()[-1]["outcome"]["status"] == "INTERRUPTED"


@pytest.mark.safety
def test_a_robot_whose_link_dropped_while_moving_refuses_the_request():
    """Belle's joint states stop while she moves: Cindy's Exec is refused at once, "can't tell"."""
    lab = Lab()
    belle = lab.belle.arms["ur_arm"]
    driver = next(driver for driver in lab.drivers if getattr(driver, "arm", None) is belle)

    async def scenario():
        with from_outside(lab.belle):
            belle.send_joint_move(belle.joint_vector() + np.radians([30, 0, 0, 0, 0, 0]))
        for _ in range(5):
            await lab.ctx.next_tick()
        driver.silent_from = lab.ctx.t
        for _ in range(8):
            await lab.ctx.next_tick()
        return await lab.plugin.request_execute(lab.request())

    result = run_guarded(scenario())
    assert result.status == REFUSED and "can't tell whether a200-0805 ur_arm is moving" in result.reason
    assert result.reason.endswith("until they come back (check its link in Health) or the monitor restarts")
    assert lab.free() and not lab.cindy._multi_arm.messages


# --- --- --- --- --- a base step and the pendant's speed (review L7, L8) --- --- --- --- ---

def test_a_base_step_says_how_to_drive_while_the_panels_refuse():
    """A base drive by hand: the prompt says to drive with the joystick or the command line, or Cancel first.

    The monitor's D-pad and base planner refuse while the step has the robot's lease.
    """
    from husky_assembly_teleop.plugins.movement_exec.sequences import Job, PartRun, run_part
    from husky_assembly_teleop.shared.movement_exec_contract import BASE_PART, PartPlan
    lab = Lab()
    request = lab.request()
    part = PartPlan(0, "B20_R_drive", Routine.BASE_ONLY, BASE_PART, (), "drive the base for B20_R_drive by hand")
    job = Job(lab.ctx, request, lab.cindy, lab.run, lab.plugin._prompts, False, lambda *_args: None)
    run = PartRun(part)
    lines = []

    async def scenario():
        task = asyncio.ensure_future(run_part(job, run))
        while lab.plugin.prompt is None:
            await asyncio.sleep(0)
        lines.append(lab.plugin.prompt.line)
        lab.ctx.t += 1.0
        lab.plugin._prompts.answer(True, lab.ctx.t)
        await task

    run_guarded(scenario())
    assert "joystick or the command line" in lines[0] and "D-pad and the base planner refuse" in lines[0]
    assert run.result == DONE


def test_exec_says_the_pendant_speed_slider_must_be_at_100():
    """Every motion's Exec logs that the pendant's speed slider must be at 100 % (slower, it times out and holds)."""
    lab = Lab()
    execute(lab, lab.request(), True)
    assert any("speed slider" in line and "100 %" in line for line in lab.ctx.lines("info") + lab.ctx.lines("warn"))


# --- --- --- --- --- the watch's timing, and the tool parts' re-check (re-review F1, F3) --- --- --- --- ---

@pytest.mark.safety
def test_the_acting_robots_arms_hold_in_the_tick_another_robot_starts_and_moves_no_further():
    """Belle starts moving mid-motion: Cindy's hold goes out within a tick, and her arm makes no progress after it."""
    lab = Lab()
    belle, left = lab.belle.arms["ur_arm"], lab.cindy.arms["left_ur_arm"]
    holds, started = [], {}
    publish = left._trajectory.publish

    def recorded(message):
        if len(message.points) == 1:
            holds.append((lab.ctx.t, left.joint_vector().copy()))
        publish(message)

    left._trajectory.publish = recorded

    async def start_belle(lab):
        while not lab.cindy._multi_arm.messages:
            await asyncio.sleep(0)
        for _ in range(3):
            await lab.ctx.next_tick()
        started["t"], started["at"] = lab.ctx.t, left.joint_vector().copy()
        with from_outside(lab.belle):
            belle.send_joint_move(belle.joint_vector() + np.radians([30, 0, 0, 0, 0, 0]))

    result, _held = execute(lab, lab.request(delta=0.6), True, during=start_belle)
    (held_at, where), = holds
    assert result.status == FAILED and held_at - started["t"] <= 0.1 + 1e-9
    assert np.allclose(left.joint_vector(), where, atol=1e-9)
    assert np.max(np.abs(where - started["at"])) < np.radians(5.0)


@pytest.mark.safety
def test_another_robot_moving_at_a_robotiq_prompt_refuses_it_after_confirm():
    """Alice starts moving while Belle's open waits for Confirm: refused after Confirm, no gripper goal."""
    lab = Lab(current=("B35_H", 1))
    alice = lab.alice.arms["ur_arm"]

    async def scenario():
        future = lab.plugin.request_execute(lab.request("B35_H_open", serial="a200-0805", route=None))
        while lab.plugin.prompt is None:
            await asyncio.sleep(0)
        with from_outside(lab.alice):
            alice.send_joint_move(alice.joint_vector() + np.radians([20, 0, 0, 0, 0, 0]))
        await answer(lab, True)
        return await future

    result = run_guarded(scenario())
    assert result.status == REFUSED and "a200-0804 ur_arm is running a trajectory" in result.reason
    assert lab.belle.arms["ur_arm"].end_effector._action.goals == []


@pytest.mark.safety
def test_another_robot_moving_at_a_grasp_prompt_refuses_it_after_confirm():
    """Alice starts moving while Cindy's grasp waits for Confirm: refused after Confirm, nothing sent to the tools."""
    lab = Lab(current=("B20_J", 3))
    alice = lab.alice.arms["ur_arm"]
    request = lab.request("B20_J_transfer")
    grasp = replace(request, parts=(request.parts[2],))
    assert grasp.parts[0].kind == "grasp"

    async def scenario():
        future = lab.plugin.request_execute(grasp)
        while lab.plugin.prompt is None:
            await asyncio.sleep(0)
        with from_outside(lab.alice):
            alice.send_joint_move(alice.joint_vector() + np.radians([20, 0, 0, 0, 0, 0]))
        await answer(lab, True)
        return await future

    result = run_guarded(scenario())
    assert result.status == REFUSED and "a200-0804 ur_arm is running a trajectory" in result.reason
    assert all(not tool._command.messages for tool in lab.tools.values())


@pytest.mark.safety
def test_the_acting_robots_own_base_driving_during_its_motion_holds_its_arms():
    """Cindy's own wheels start turning while her arms move: her arms are held, the part fails naming her base."""
    lab = Lab()
    base = lab.cindy.base.state

    async def drive_soon(lab):
        while not lab.cindy._multi_arm.messages:
            await asyncio.sleep(0)
        base.joint_states_update_time, base.wheel_velocities = lab.ctx.t, {"front_left_wheel_joint": 0.4}

    result, _held = execute(lab, lab.request(delta=0.6), True, during=drive_soon)
    assert result.status == FAILED and "a200-0806 base" in result.reason
    assert lab.run.files()[-1]["outcome"]["status"] == "INTERRUPTED"


@pytest.mark.safety
def test_a_base_path_sent_just_before_the_lease_holds_the_acting_robots_arms_once_the_follower_reports_it():
    """Belle's base got a path just before the lease was taken; her follower reports "following": Cindy's arms hold.

    ? The monitor keeps no send time for a path, so the watch learns of it from the follower's report or the wheels.
    """
    lab = Lab()

    async def follow_soon(lab):
        while not lab.cindy._multi_arm.messages:
            await asyncio.sleep(0)
        lab.belle.base.state.follower = FollowerState(
            controller="pure_pursuit", state="following", reason="", path_id=7, timed=False, piece=0, piece_count=3,
            turning=False, progress=0.0, reference=np.zeros(3), lookahead=np.zeros(2), cross_track_error=0.0,
            along_track_error=np.nan, yaw_error=0.0, command=np.zeros(2), update_time=lab.ctx.t, stamp=lab.ctx.t)

    result, _held = execute(lab, lab.request(delta=0.6), True, during=follow_soon)
    assert result.status == FAILED and "a200-0805 base is following a path" in result.reason
    assert lab.run.files()[-1]["outcome"]["status"] == "INTERRUPTED"


def track_base(lab: Lab, serial: str) -> SimpleNamespace:
    """Give a robot's base rigid body 9, a live mocap pose at the origin every tick until `mocap.lost` is set.

    Returns:
        SimpleNamespace: `state`, the base's state, and `lost`.
    """
    mocap = SimpleNamespace(state=BaseState(), lost=False)
    lab.robots[serial].base = SimpleNamespace(mocap_id=9, state=mocap.state)
    step = lab.ctx.step

    def tick(t):
        step(t)
        state = mocap.state
        if state.position is None:
            state.position, state.orientation = np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0])
        state.last_update_time = t
        if not mocap.lost:
            state.tracked = state.tracking_valid = True
            state.marker_errors.append((t, 0.0))
            state.last_fix_time = t

    lab.ctx.step = tick
    tick(lab.ctx.t)
    return mocap


@pytest.mark.safety
def test_a_base_driving_or_pushed_during_the_motion_holds_the_acting_robots_arms():
    """Belle's tracked base moves 2 cm while Cindy's arms move, its wheels still (a push): the freeze holds Cindy.

    The watch sees a robot's first activity (wheels, follower, joints); the freeze sees displacement, whatever moved it.
    """
    lab = Lab()
    mocap = track_base(lab, "a200-0805")

    async def push_soon(lab):
        while not lab.cindy._multi_arm.messages:
            await asyncio.sleep(0)
        mocap.state.position = np.array([0.02, 0.0, 0.0])

    result, _held = execute(lab, lab.request(delta=0.6), True, during=push_soon)
    assert result.status == FAILED and "robot a200-0805 base moved 20 mm" in result.reason
    assert "Stop all" not in result.reason and lab.free()
    holds = [sum(len(message.points) == 1 for message in arm._trajectory.messages) for arm in lab.cindy.arms.values()]
    assert holds == [1, 1] and lab.run.files()[-1]["outcome"]["status"] == "INTERRUPTED"
    (_event, fields), = [record for record in lab.run.records if record[0] == "cancel"]
    assert fields["data"]["reason"] == "freeze broken: robot a200-0805 base moved 20 mm, 0.0 deg"


@pytest.mark.safety
def test_another_robot_lost_by_mocap_during_the_motion_holds_the_acting_robots_arms():
    """Belle's base pose is lost while Cindy's arms move: it may move unseen, so Cindy is held."""
    lab = Lab()
    mocap = track_base(lab, "a200-0805")

    async def hide_soon(lab):
        while not lab.cindy._multi_arm.messages:
            await asyncio.sleep(0)
        mocap.lost = True

    result, _held = execute(lab, lab.request(delta=0.6), True, during=hide_soon)
    assert result.status == FAILED and "robot a200-0805 base: no valid mocap pose" in result.reason


@pytest.mark.safety
def test_another_robot_lost_by_mocap_at_the_prompt_is_refused_after_confirm():
    """Belle's base pose, tracked before, is lost while Cindy's prompt waits: refused after Confirm, nothing sent."""
    lab = Lab()
    mocap = track_base(lab, "a200-0805")

    async def scenario():
        future = lab.plugin.request_execute(lab.request())
        while lab.plugin.prompt is None:
            await asyncio.sleep(0)
        mocap.lost = True
        await answer(lab, True)
        return await future

    result = run_guarded(scenario())
    assert result.status == REFUSED and "can't tell whether the other robots stand still: robot a200-0805 base" \
        in result.reason
    assert not lab.cindy._multi_arm.messages and lab.free()


@pytest.mark.safety
def test_another_robot_moving_while_the_grasp_clears_its_stall_tightens_nothing():
    """Alice starts moving after Cindy's grasp is confirmed, while its STOP clears the stall: nothing tightened."""
    lab = Lab(current=("B20_J", 3))
    alice = lab.alice.arms["ur_arm"]
    request = lab.request("B20_J_transfer")
    grasp = replace(request, parts=(request.parts[2],))
    left, = (driver for driver in lab.drivers if getattr(driver, "tool", None) is lab.tools["left_ur_arm"])
    left.motor = "STALLED"  # * left over, so the grasp waits a tick for its STOP to clear it
    lab.tick(lab.ctx.t)

    async def move_alice(lab):
        while not lab.tools["left_ur_arm"]._command.messages:
            await asyncio.sleep(0)
        with from_outside(lab.alice):
            alice.send_joint_move(alice.joint_vector() + np.radians([20, 0, 0, 0, 0, 0]))

    result, _held = execute(lab, grasp, True, during=move_alice)
    assert result.status == REFUSED and "a200-0804 ur_arm is running a trajectory; nothing tightened" in result.reason
    for tool in lab.tools.values():
        assert [(command.motor, command.direction) for command in tool._command.messages] == [(0, 0), (0, 0)]


# --- --- --- --- --- the safety sync (multi_arm_safety_sync) --- --- --- --- ---

@pytest.mark.safety
def test_an_arm_the_safety_sync_does_not_vouch_for_is_refused_before_the_prompt():
    """Cindy's right arm without a recent sync status: refused before any prompt, the line names the arm and why."""
    lab = Lab()
    request = lab.request()
    arm = lab.cindy.arms["right_ur_arm"]
    next(driver for driver in lab.drivers if getattr(driver, "arm", None) is arm).synced = False
    arm.state.status_update_time = lab.ctx.t - 5.0
    result, held = execute(lab, request, True)
    assert result.status == REFUSED and lab.published() == 0 and lab.run.records[0][0] == "exec"
    assert result.reason == ("B20_R_home: cindy (a200-0806) right_ur_arm: its safety is not known: no recent status "
                             "from multi_arm_safety_sync; start it on the robot")


@pytest.mark.safety
def test_a_program_stopped_at_the_prompt_is_refused_after_confirm():
    """Stop all stops Cindy's programs while the prompt waits: refused after Confirm, saying Resume; nothing sent."""
    lab = Lab()

    async def scenario():
        future = lab.plugin.request_execute(lab.request())
        while lab.plugin.prompt is None:
            await asyncio.sleep(0)
        for arm in lab.cindy.arms.values():
            arm.state.program_playing = False
        await answer(lab, True)
        return await future

    result = run_guarded(scenario())
    assert result.status == REFUSED and lab.published() == 0
    assert result.reason == ("B20_R_home: cindy (a200-0806) left_ur_arm: its program is stopped (Stop all, or the sync "
                             "stopped it); press Resume")


def test_confirm_and_cancel_are_buttons_of_their_own_in_the_assembly_tab():
    """Confirm and Cancel are two buttons, on the desktop as on the phone; the desktop's reach the plugin."""
    lab = Lab()
    buttons = lab.ctx.gui.of_kind("button")
    assert [(button.label, button.color) for button in buttons] == [("Confirm", "green"), ("Cancel", "red")] * 2
    assert lab.ctx.gui.of_kind("button_group") == []
    buttons[0].callback()
    assert lab.plugin._last == "nothing to answer"
    buttons[1].callback()
    assert lab.ctx.lines("info")[-1] == "Cancel: nothing to cancel"


# --- --- --- --- --- the compliant parts: insert, retreat and its ungrasp, Robotiq handoff (M6) --- --- --- --- ---

#: Cindy's joint names, left arm first; each insert's and retreat's path moves her elbows (as the shared tests').
CINDY_NAMES = tuple(arm_joint_names("left_ur_arm") + arm_joint_names("right_ur_arm"))
INSERT_STEP = np.array([0, 0, 0.03, 0, 0, 0, 0, 0, -0.03, 0, 0, 0])
RETREAT_STEP = np.array([0.0, -0.03, 0.06, -0.03, 0.0, 0.0] * 2)


def parts_of(lab: Lab, action: str, primary: int) -> tuple[PartPlan, ...]:
    """The parts of an action's step, by its primary movement, with the transfer workaround."""
    steps = operator_steps(lab.loaded.process, lab.loaded.process.actions[action], True)
    return step_parts(lab.loaded, action, next(each for each in steps if each.primary == primary), lab.configs, True)


def compliant_lab(current, settings: RunSettings | None = None) -> Lab:
    """The lab at `current`, Cindy with her URDF frames, her base seen by mocap and her F/T sensors reading 0."""
    lab = Lab(current=current)
    lab.track("a200-0806")
    for arm in lab.cindy.arms.values():
        arm.state.wrench = np.zeros(6)
    if settings is not None:
        lab.run.view.settings = settings
    return lab


def along(lab: Lab, movement: str, step: np.ndarray, goals: bool = False) -> ExecRequest:
    """Cindy's request for `movement`, its plan from her joints now along `step` in 20 rows.

    With `goals`, the plan holds each flange's world goal (`RouteResult.goals`), as a retreat's does.
    """
    start = np.concatenate([arm.joint_vector() for arm in lab.cindy.arms.values()])
    rows = np.linspace(start, start + step, 21)
    route = RouteResult(MovementResult("solved", trajectory=Trajectory("robots/cindy", CINDY_NAMES,
                                                                       tuple(map(tuple, rows.tolist())))),
                        Routine.RETREAT_LINEAR if goals else Routine.INSERT, planner="fake",
                        goals={f"robots/cindy/{arm}_tool0": compose(arm_base_in_world(lab.cindy, arm), tool0_fk(
                            lab.cindy.config.model.urdf, arm, dict(zip(CINDY_NAMES, rows[-1]))))
                            for arm in lab.cindy.arms} if goals else {})
    return lab.request(movement, route=route)


class TailPlanner:
    """The motion planner as the retreat asks it for its tail: from Cindy's joints now to `end`, or a failure.

    Attributes:
        requests: Every request asked.
    """

    def __init__(self, lab: Lab, end: np.ndarray, solved: bool = True):
        """Answer at once."""
        self.lab, self.end, self.solved, self.requests = lab, end, solved, []

    def request_plan(self, request):
        """The tail, answered at once."""
        self.requests.append(request)
        future = asyncio.get_running_loop().create_future()
        start = np.concatenate([arm.joint_vector() for arm in self.lab.cindy.arms.values()])
        rows = np.linspace(start, self.end, 20)
        result = MovementResult("solved", trajectory=Trajectory("robots/cindy", CINDY_NAMES, tuple(
            map(tuple, rows.tolist())))) if self.solved else MovementResult("failed", "no tail")
        future.set_result(RouteResult(result, Routine.RETREAT_LINEAR, planner="fake", waypoints=20))
        return future


def commands(lab: Lab, arm: str) -> list[tuple[int, int]]:
    """(motor, direction) of every command sent to one of Cindy's tools."""
    return [(command.motor, command.direction) for command in lab.tools[arm]._command.messages]


def prompts(lab: Lab) -> list[str]:
    """Every prompt's line, as recorded."""
    return [fields["data"]["text"] for event, fields in lab.run.records if event == "prompt"]


@pytest.mark.safety
def test_the_purged_process_gives_each_compliant_part_its_arms():
    """Each insert, retreat, ungrasp and Robotiq close of the purged process becomes its part, on its arms.

    B20's insert is a ground bar's (no tool); B21's tightens the left screw, B22's both; B20_R's ungrasp goes with its
    retreat on the same arms, any other arms refused; B21_H's close is the handoff; the ground bar's insert ends with
    its servo (M7).
    """
    lab = Lab()
    both = ("left_ur_arm", "right_ur_arm")
    assert [(part.kind, part.tools) for part in parts_of(lab, "B20_J", 4)] == [(INSERT_PART, ()), (SERVO_PART, both)]
    assert [(part.kind, part.tools) for part in parts_of(lab, "B21_J", 4)] == [(INSERT_PART, ("left_ur_arm",))]
    assert [(part.kind, part.tools) for part in parts_of(lab, "B22_J", 4)] == [
        (INSERT_PART, ("left_ur_arm", "right_ur_arm"))]
    assert [(part.index, part.kind, part.tools) for part in parts_of(lab, "B20_R", 1)] == [
        (1, RETREAT_PART, both), (0, WITH_MOTION_PART, both)]
    left = PartPlan(1, "B20_R_retreat", Routine.RETREAT_LINEAR, RETREAT_PART, ("left_ur_arm",), "")
    ungrasp = part_plan(lab.loaded, "B20_R", 0, lab.configs, True, during=left)
    assert ungrasp.kind == REFUSED_PART and "but B20_R_retreat moves left_ur_arm" in ungrasp.text
    assert [(part.kind, part.tools) for part in parts_of(lab, "B21_H", 3)] == [(HANDOFF_PART, ("ur_arm",))]
    assert not LATER
    assert DEFAULT_TRAJ_TIME[Routine.INSERT] == DEFAULT_TRAJ_TIME[Routine.RETREAT_LINEAR] == 5.0


@pytest.mark.safety
def test_b21s_insert_tightens_the_left_screw_only_and_ends_when_it_stalls():
    """B21_J_insert: one prompt naming Cindy, the left joint screw tightens, rigid then compliant; the stall ends it.

    The right screw only STOPs. The panel's live line shows the compliant part; the record is JSON.
    """
    lab = compliant_lab(("B21_J", 4))
    next(driver for driver in lab.drivers if getattr(driver, "tool", None) is lab.tools["left_ur_arm"]) \
        .screw_stall_after = 4.0
    lines = []

    async def watch_panel(lab):
        while lab.plugin.active:
            recorder = lab.plugin._job.recorder
            if recorder is not None and recorder.phase == "compliant" and recorder.samples:
                lab.plugin.draw(lab.ctx)
                lines.append(lab.plugin._panel._live.content)
                return
            await asyncio.sleep(0)

    result, _held = execute(lab, along(lab, "B21_J_insert", INSERT_STEP), True, during=watch_panel)
    assert result.status == DONE and result.done == (4,), result.reason
    assert prompts(lab)[0].startswith("cindy (a200-0806): B21_J_insert — INSERT: rigid ")
    joint = ScaffoldingV3.JOINT_MOTOR
    assert (joint, 1) in commands(lab, "left_ur_arm") and (joint, 1) not in commands(lab, "right_ur_arm")
    assert commands(lab, "right_ur_arm")[0] == (0, 0)
    record = json.loads(json.dumps(lab.run.files()[-1]))
    assert record["part"]["kind"] == "insert" and record["outcome"]["status"] == "STALLED"
    assert record["outcome"]["settings"] == record["command"]["settings"] == {
        "split_mm": 5.0, "rigid_only": False, "spiral": False, "spiral_seconds": 20.0, "spiral_turns": 4}
    assert [phase["name"] for phase in record["outcome"]["phases"]] == ["rigid", "switch", "compliant"]
    assert record["plan"]["planner"] == "fake" and result.end is not None
    line, = lines
    assert "compliant · e " in line and "|F| left_ur_arm 0.0 N, right_ur_arm 0.0 N" in line
    lab.plugin.draw(lab.ctx)
    assert "no compliant part running" in lab.plugin._panel._live.content


@pytest.mark.safety
def test_an_inserts_rigid_part_whose_hold_is_refused_is_hold_not_sent():
    """Cindy's joint states stop during B21's rigid part, then the screw stalls: the hold is refused on stale joints.

    The result, the record and the panel say HOLD NOT SENT, as a motion's; the pendant line is logged once, then the
    Exec banner.
    """
    lab = compliant_lab(("B21_J", 4))
    next(driver for driver in lab.drivers if getattr(driver, "tool", None) is lab.tools["left_ur_arm"]) \
        .screw_stall_after = 2.0

    async def drop(lab):
        while not lab.cindy._multi_arm.messages:
            await asyncio.sleep(0)
        for driver in lab.drivers:
            if getattr(driver, "robot", None) is lab.cindy:
                driver.silent_from = lab.ctx.t

    result, _held = execute(lab, along(lab, "B21_J_insert", INSERT_STEP), True, during=drop)
    assert result.status == HOLD_NOT_SENT and "HOLD NOT SENT on left_ur_arm, right_ur_arm" in result.reason
    outcome = lab.run.files()[-1]["outcome"]
    assert outcome["status"] == "HOLD_NOT_SENT" and outcome["phases"][0]["end"] == "HOLD_NOT_SENT"
    lab.plugin.draw(lab.ctx)
    assert any("HOLD NOT SENT" in widget.content for widget in lab.ctx.gui.of_kind("html"))
    lines = [line for _level, line in lab.ctx.logs if "HOLD NOT SENT on" in line]
    assert len(lines) == 2 and lines[0].startswith("HOLD NOT SENT on left_ur_arm, right_ur_arm: stop a200-0806")
    assert lines[1].startswith("Exec ")


@pytest.mark.safety
def test_the_insert_settings_are_read_once_and_reach_its_plan():
    """Rigid-only reaches the plan (its prompt); a spiral too fast is refused before any prompt, nothing sent.

    The spiral's seconds and turns reach B22's prompt (two mating joints); a split over 30 mm is taken as 30. B21, one
    mating joint, drops the spiral asked for: its prompt says so, the setting is recorded, the plan has none.
    """
    lab = compliant_lab(("B21_J", 4), RunSettings(rigid_only=True))
    result, _held = execute(lab, along(lab, "B21_J_insert", INSERT_STEP), False)
    assert result.status == CANCELLED and lab.published() == 0
    assert prompts(lab)[0].startswith("cindy (a200-0806): B21_J_insert — RIGID-ONLY insert: the whole ")
    assert lab.run.files()[-1]["command"]["settings"]["rigid_only"] is True

    lab = compliant_lab(("B22_J", 4), RunSettings(spiral=True, spiral_seconds=3.0))
    result, _held = execute(lab, along(lab, "B22_J_insert", INSERT_STEP))
    assert result.status == REFUSED and "spiral too fast" in result.reason and "need ≥ 7.6 s" in result.reason
    assert [event for event, _fields in lab.run.records] == ["exec"] and lab.published() == 0
    assert all(not tool._command.messages for tool in lab.tools.values())

    lab = compliant_lab(("B22_J", 4), RunSettings(spiral=True, spiral_seconds=30.0, spiral_turns=6,
                                                  insert_split_mm=40.0))
    result, _held = execute(lab, along(lab, "B22_J_insert", INSERT_STEP), False)
    assert result.status == CANCELLED and "spiral 15 mm, 30 s, 6 turns, pitch 2.5 mm" in prompts(lab)[0]
    assert lab.run.files()[-1]["command"]["settings"]["split_mm"] == 30.0

    lab = compliant_lab(("B21_J", 4), RunSettings(spiral=True, spiral_seconds=30.0, spiral_turns=6))
    result, _held = execute(lab, along(lab, "B21_J_insert", INSERT_STEP), False)
    assert result.status == CANCELLED and "straight line: no spiral for a bar with one mating joint" in prompts(lab)[0]
    command = lab.run.files()[-1]["command"]
    assert command["settings"]["spiral"] is True and command["plan"]["spiral"] is None


@pytest.mark.parametrize("solved", [True, False])
@pytest.mark.safety
def test_b20s_retreat_opens_the_screws_replans_its_tail_and_its_ungrasp_is_done(solved):
    """B20_R's step: one prompt; the M1 screws open, the tail is replanned and run; the retreat and its ungrasp done.

    The replan asks for the plan's goals in 1 mm steps. A tail not planned fails the retreat (A4): no tail, the
    ungrasp not done.
    """
    lab = compliant_lab(("B20_R", 1))
    request = along(lab, "B20_R_retreat", RETREAT_STEP, goals=True)
    start = np.array(request.route.result.trajectory.positions[0])
    planner = lab.ctx.plugins["motion_planner"] = TailPlanner(lab, start + RETREAT_STEP, solved)
    result, _held = execute(lab, request, True)
    asked, = planner.requests
    assert asked.goal_poses == request.route.goals and asked.linear_step == TAIL_STEP
    assert asked.step.movement.id == "B20_R_retreat" and asked.snapshot is not None
    assert len(prompts(lab)) == 1 and prompts(lab)[0].startswith(
        "cindy (a200-0806): B20_R_retreat — RETREAT: the M1 screws of left_ur_arm + right_ur_arm OPEN")
    for arm in lab.tools:
        assert (ScaffoldingV3.GRIPPER_MOTOR, -1) in commands(lab, arm)
    files = [json.loads(json.dumps(file)) for file in lab.run.files()]
    if solved:
        assert result.status == DONE and result.done == (1, 0), result.reason
        assert [(file["movement"], file["outcome"]["status"]) for file in files] == [
            ("B20_R_retreat", "RETREATED"), ("B20_R_ungrasp", "opened")]
        assert files[0]["outcome"]["tail"]["planner"] == "fake" and files[0]["outcome"]["checks"]
    else:
        assert result.status == FAILED and result.done == () and "the tail's replan failed" in result.reason
        assert [file["movement"] for file in files] == ["B20_R_retreat"] and "tail" not in files[0]["outcome"]


@pytest.mark.safety
def test_b21_hs_close_is_the_compliant_handoff():
    """B21_H_close: the prompt names Alice and the handoff; Confirm closes the Robotiq with her arm compliant, CLOSED.

    Cancel at the prompt sends nothing.
    """
    lab = Lab(current=("B21_H", 3))
    lab.track("a200-0804")
    request = lab.request("B21_H_close", serial="a200-0804", route=None)
    gripper = lab.alice.arms["ur_arm"].end_effector
    result, _held = execute(lab, request, False)
    assert result.status == CANCELLED and gripper._action.goals == [] and lab.alice.arms["ur_arm"].controllers \
        .switches == []
    assert prompts(lab)[0].startswith("alice (a200-0804): B21_H_close — CLOSE + compliant handoff on ur_arm")
    run_guarded(settle(lab))
    result, _held = execute(lab, request, True)
    assert result.status == DONE and result.done == (3,), result.reason
    assert [goal.command.position for goal in gripper._action.goals] == [ROBOTIQ_CLOSE]
    record = json.loads(json.dumps(lab.run.files()[-1]))
    assert record["part"]["kind"] == "robotiq_handoff" and record["outcome"]["status"] == "CLOSED"


def test_a_compliant_parts_question_names_the_robot_and_is_recorded():
    """The spiral's warning, asked through the job's prompts, names the robot and the movement first.

    The prompt and the Confirm are recorded once answered; closed unanswered, nothing more.
    """
    from husky_assembly_teleop.plugins.movement_exec.sequences import Job
    lab = Lab()
    events = []
    job = Job(lab.ctx, lab.request(), lab.cindy, lab.run, lab.plugin._prompts, False,
              lambda *args: events.append(args))
    operator = PartOperator(job, "B21_J_insert")
    prompt = operator.open("Spiral complete without a stall", warn=True)
    assert prompt.line == "cindy (a200-0806): B21_J_insert — Spiral complete without a stall" and prompt.warn
    assert operator.poll(prompt) is None
    lab.ctx.t += 1.0
    lab.plugin.request_confirm()
    assert operator.poll(prompt) is True and lab.plugin.prompt is None
    assert [event for event, _movement, _data in events] == ["prompt", "confirm"]
    operator.close(operator.open("again"))
    assert lab.plugin.prompt is None and len(events) == 3
