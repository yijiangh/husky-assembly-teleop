"""The executor on the purged process with fake robots that follow what they are sent.

Refusals, the lease, prompts, cancels, records, the transfer workaround's parts and the dry run.
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
from husky_assembly_teleop.robot_interface.base import FollowerState
from husky_assembly_teleop.robot_interface.robot_lease import NotLeased
from husky_assembly_teleop.shared.execution.basis import basis_of
from husky_assembly_teleop.shared.movement_exec_contract import (CANCELLED, DONE, DRY_RUN, FAILED, HOLD_NOT_SENT,
                                                                 REFUSED, ExecRequest, step_parts)
from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names
from husky_assembly_teleop.shared.assembly_planning.movement import Routine
from husky_assembly_teleop.shared.assembly_planning.route_result import RouteResult
from husky_assembly_teleop.shared.assembly_planning.steps import operator_steps
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.process.solutions import MovementResult, Trajectory


#: Wall-clock seconds a scenario may take: a guard missing makes a test wait forever, this makes it fail instead.
WALL_LIMIT = 30.0


def run_guarded(scenario):
    """Run a scenario in a new event loop, failing (not hanging) after WALL_LIMIT seconds of wall clock."""
    return asyncio.run(asyncio.wait_for(scenario, WALL_LIMIT))


class Lab(RobotLab):
    """The three robots live and following commands, the run at `current`, movement_exec set up on them."""

    def __init__(self, current=("B20_R", 2), **driver):
        """Build everything; `driver` options go to every arm's driver."""
        super().__init__(**driver)
        self.loaded = purged_process()
        self.run = FakeRun(self.loaded, current)
        self.ctx.plugins = {"assembly_run": self.run, "motion_planner": SimpleNamespace()}
        self.plugin = MovementExecPlugin()
        self.plugin.setup(self.ctx)

    def free(self) -> bool:
        """Whether Cindy is given back: neither held for an automatic motion (the lock) nor leased (her robot lease)."""
        return self.cindy.lease is None and self.cindy.lease_slot.lease is None

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
                         dict(zip(names, start.tolist())), self.run.view.settings, self.run.view.statuses, False,
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
        held = lab.cindy.lease
        lab.robot_lease = lab.cindy.lease_slot.lease
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
    """B20_R_home: prompt, Confirm, one multi-arm message, arrived; done, the lease held during and given back."""
    lab = Lab()
    result, held = execute(lab, lab.request(), True)
    assert result.status == DONE and result.done == (2,), result.reason
    assert held is not None and held.label == "B20_R_home" and lab.free()
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


@pytest.mark.safety
def test_both_leases_are_back_when_the_result_arrives():
    """The result resolves with Cindy given back already, so its caller may start the next step at once."""
    lab = Lab()

    async def scenario():
        future = lab.plugin.request_execute(lab.request())
        await answer(lab, True)
        result = await future
        return result, lab.free()

    result, free = run_guarded(scenario())
    assert result.status == DONE and free


@pytest.mark.safety
def test_a_request_whose_task_cannot_start_gives_both_leases_back(monkeypatch):
    """Spawning the step's task raises, as a closing plugin's context does: both leases are given back, then raised.

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
def test_a_robot_lease_refused_after_the_lock_gives_the_lock_back(monkeypatch):
    """The lock taken, then the robot lease refused (a lease `busy_reason` could not see): busy, the lock given back."""
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
    """B20_J's transfer step: parts (3, 1, 2); Cancel at the mount by hand leaves done = (3,)."""
    lab = Lab(current=("B20_J", 3))
    request = lab.request("B20_J_transfer")
    assert [(part.index, part.kind) for part in request.parts] == [(3, "motion"), (1, "operator"), (2, "grasp")]
    result, _held = execute(lab, request, True, False)
    assert result.status == CANCELLED and result.done == (3,)
    assert [file["movement"] for file in lab.run.files()] == ["B20_J_transfer", "B20_J_place_bar"]


def test_the_whole_transfer_step_ends_with_the_grasp():
    """Confirm all three: the motion, the mount by hand, the grasp (both screws stalled); done (3, 1, 2)."""
    lab = Lab(current=("B20_J", 3))
    result, _held = execute(lab, lab.request("B20_J_transfer"), True, True, True)
    assert result.status == DONE and result.done == (3, 1, 2), result.reason
    assert lab.run.files()[-1]["outcome"]["status"] == "stalled"


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
    """Belle's open is a Robotiq part that reaches 0.426; Cindy's retreat step (M6) is refused, nothing sent."""
    lab = Lab(current=("B35_H", 1))
    request = lab.request("B35_H_open", serial="a200-0805", route=None)
    assert [part.kind for part in request.parts] == ["robotiq_open"]
    result, _held = execute(lab, request, True)
    assert result.status == DONE and result.done == (1,) and lab.published() == 0
    assert lab.run.files()[0]["outcome"]["status"] == "reached"
    lab = Lab(current=("B20_R", 1))
    result, _held = execute(lab, lab.request("B20_R_retreat"))
    assert result.status == REFUSED and "runs in M6" in result.reason and lab.published() == 0


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

    Stop all no longer drops queued clicks. The step is recorded as cancelled by the soft stop, no trajectory goes
    out, and both leases are given back.
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

    As after a clearance trip, or for a task that survived the stop. Nothing is sent, both leases are given back.
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
def test_robot_control_cannot_move_another_robot_while_one_is_held():
    """Cindy's prompt open: robot_control's Cartesian Start on Belle is refused, no lease taken; Confirm moves Cindy."""
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
        await answer(lab, True)
        return await future

    result = run_guarded(scenario())
    assert result.status == DONE and not belle._trajectory.messages and len(lab.cindy._multi_arm.messages) == 1
    assert any("a200-0805 ur_arm: refused: a200-0806 is held" in line for line in lab.ctx.lines("warn"))
    assert not control._trajectory_tasks and lab.belle.lease_slot.lease is None


@pytest.mark.safety
def test_exec_is_busy_while_another_plugin_leases_any_robot():
    """robot_control's teleop lease on Belle, or a planner's commit lease on Cindy herself: Exec is refused at once.

    Nothing is sent, Cindy is neither held nor leased by movement_exec, and the reason names the lease.
    """
    for serial, holder in (("a200-0805", "robot_control"), ("a200-0806", "base_planner")):
        lab = Lab()
        other = FakeContext(name=holder)
        other.world = lab.ctx.world
        lease = other.take_lease([serial], "teleop" if holder == "robot_control" else "commit plan 1")
        result, held = execute(lab, lab.request())
        assert result.status == REFUSED and result.reason == f"busy: {serial} is leased by {lease.holder}"
        assert held is None and lab.cindy.lease is None and lab.ctx.world.robots[serial].lease_slot.lease is lease
        assert lab.published() == 0 and lab.run.records == []


@pytest.mark.safety
def test_no_other_plugin_gets_a_lease_while_a_step_runs():
    """Cindy's prompt open (her step holds her): robot_control gets no lease on Belle, nor on Cindy; Confirm moves Cindy.

    movement_exec holds Cindy's robot lease the whole step, the prompt included.
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
    assert refusals == ["robot a200-0806 is held by movement_exec (B20_R_home) for an automatic motion, one motion at "
                        "a time; Cancel it first"] * 2


@pytest.mark.safety
def test_a_path_check_during_a_step_moves_nothing_and_leases_nothing(monkeypatch):
    """The arm planner checks a joint path for Belle while Cindy's step waits at its prompt.

    It answers, sends nothing and leases nothing; Cindy stays held and leased until her step ends.
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
        during = (lab.cindy.lease, lab.cindy.lease_slot.lease, lab.belle.lease_slot.lease, lab.published())
        lab.plugin.request_cancel()
        return found, during, await future

    found, (lock, robot_lease, belle_lease, published), result = run_guarded(scenario())
    planner._executor.shutdown()
    assert found is None and checked == [("ur_arm", (2, 6))]
    assert lock is not None and robot_lease.holder == "movement_exec: exec B20_R_home" and belle_lease is None
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
def test_another_robot_starting_to_move_during_the_motion_holds_the_held_robot():
    """Belle starts moving while Cindy's arms move: Cindy is held, the part fails naming Belle."""
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

def test_a_base_step_says_how_to_drive_while_the_panels_are_locked():
    """A base drive by hand: the prompt says to drive with the joystick or the command line, or Cancel first.

    The monitor's D-pad and base planner stay locked while the step runs (the robot stays held).
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
    assert "joystick or the command line" in lines[0] and "D-pad and the base planner are locked" in lines[0]
    assert run.result == DONE


def test_exec_says_the_pendant_speed_slider_must_be_at_100():
    """Every motion's Exec logs that the pendant's speed slider must be at 100 % (slower, it times out and holds)."""
    lab = Lab()
    execute(lab, lab.request(), True)
    assert any("speed slider" in line and "100 %" in line for line in lab.ctx.lines("info") + lab.ctx.lines("warn"))


# --- --- --- --- --- the watch's timing, and the tool parts' re-check (re-review F1, F3) --- --- --- --- ---

@pytest.mark.safety
def test_the_held_robot_is_held_in_the_tick_another_robot_starts_and_moves_no_further():
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
def test_the_held_robots_own_base_driving_during_its_motion_holds_it():
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
def test_a_base_path_sent_just_before_the_lease_holds_the_held_robot_once_the_follower_reports_it():
    """Belle's base got a path just before Cindy was taken; her follower reports "following": Cindy is held.

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
