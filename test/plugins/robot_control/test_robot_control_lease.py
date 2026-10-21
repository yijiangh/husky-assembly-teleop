"""While any robot is held for an automatic motion, motion commands to every robot are refused; Hold and Stop work.

robot_control refuses before it takes its teleop lease (`_leased`), so no robot lease changes hands meanwhile; and a
robot it holds the teleop lease of keeps the executor off every robot (`interlock.busy_reason`).
"""

from __future__ import annotations

import ast
import asyncio
import inspect
import textwrap
from contextlib import nullcontext
from functools import partial
from types import SimpleNamespace

import numpy as np
import pytest
from plugin_fakes import FakeContext, fake_arm, fake_node, fake_robot, fake_robotiq

from husky_assembly_teleop.plugins.arm_planner.planner import ArmPath
from husky_assembly_teleop.plugins.arm_planner.planner_plugin import ArmPlannerPlugin
from husky_assembly_teleop.plugins.base_planner.path import BasePath
from husky_assembly_teleop.plugins.base_planner.planner_plugin import BasePlannerPlugin
from husky_assembly_teleop.plugins.health import robot_action
from husky_assembly_teleop.plugins.robot_control.arm_panel import build_phone_arm_inputs, build_tool_inputs
from husky_assembly_teleop.plugins.robot_control.dpad import MAX_LINEAR_SPEED
from husky_assembly_teleop.plugins.robot_control.plugin import FREE_DRIVE_RETRY, RobotControlPlugin, _BaseWidgets
from husky_assembly_teleop.plugins.robot_control.status import build_controller_panel
from husky_assembly_teleop.plugins.robot_control.teleop_lease import IDLE_AFTER
from husky_assembly_teleop.robot_interface.arm import FREE_DRIVE_CONTROLLER, SCALED_JOINT_TRAJECTORY_CONTROLLER
from husky_assembly_teleop.robot_interface.base import BaseState
from husky_assembly_teleop.robot_interface.controller_manager import CONTROLLER_STATE_MAX_AGE
from husky_assembly_teleop.robot_interface.robot import Lease
from husky_assembly_teleop.shared.arm_planner_contract import ArmPlanRequest
from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names
from husky_assembly_teleop.shared.base_planner_contract import BasePlanRequest
from husky_assembly_teleop.shared.execution.interlock import Busy, give_back, take
from husky_assembly_teleop.shared.planner.plan import Plan
from husky_assembly_teleop.world.clearance import ALL_ARMS, STOP_DISTANCE, ClearanceRefused

pytestmark = pytest.mark.safety

SERIAL = "a200-0804"
START = [0.0, -1.5, 1.5, 0.0, 0.5, 0.0]
KEY = (SERIAL, "ur_arm")


class Lab:
    """Alice with a Robotiq and Cindy, robot_control set up on them (no viser), and a robot to hold."""

    def __init__(self):
        """Build the robots and the plugin; nobody is held or leased yet."""
        self.ctx = FakeContext(name="robot_control")
        node = fake_node(self.ctx.now, self.ctx.logs)
        self.gripper = fake_robotiq(node)
        self.arm = fake_arm(node, "ur_arm", START, end_effector=self.gripper)
        self.robot = fake_robot(node, SERIAL, [self.arm])
        self.cindy = fake_robot(node, "a200-0806", [fake_arm(node, "left_ur_arm", START),
                                                    fake_arm(node, "right_ur_arm", START)])
        self.ctx.world.robots["a200-0806"] = self.cindy
        self.twists = []
        self.robot.base = SimpleNamespace(controllers=SimpleNamespace(is_active=lambda _name: True),
                                          send_twist=lambda linear, angular: self.twists.append((linear, angular)),
                                          state=BaseState(), soft_stopped=False)
        self.ctx.world.robots[SERIAL] = self.robot
        self.plugin = RobotControlPlugin()
        self.plugin._ctx = self.ctx
        self.stops = []
        #: How many commit tasks the planners started (`commits`): a refusal at the Commit starts none.
        self.commit_tasks = 0

    def hold_robot(self, serial: str = SERIAL):
        """Hold a robot (Alice by default), as the executor does; its stop is counted."""
        return take(self.ctx.world, serial, "movement_exec", "B21_H_approach", lambda: self.stops.append(1),
                    self.ctx.t)

    def set_held(self, serial: str) -> Lease:
        """Mark a robot held without `take`'s checks.

        It plays a robot taken while robot_control drives another: a bug since the robot lease, which `take` checks.
        It proves the per-tick checks that stay as a second line.
        """
        lease = self.ctx.world.robots[serial].lease = Lease("movement_exec", "B21_H_approach", self.ctx.t,
                                                            lambda: self.stops.append(1))
        return lease

    def later(self, seconds: float) -> None:
        """Let `seconds` pass, every arm measured, synced and its controller manager heard from again at the end."""
        self.ctx.t += seconds
        for robot in (self.robot, self.cindy):
            for arm in robot.arms.values():
                arm.state.last_update_time = arm.state.status_update_time = self.ctx.t
                arm.state.controllers.last_update_time = self.ctx.t

    def leased(self):
        """robot_control's `leased` for Alice's arm, as its builders get it."""
        return partial(self.plugin._leased, self.ctx, KEY)

    def refusals(self) -> list[str]:
        """The refusal lines logged so far."""
        return [line for line in self.ctx.lines("warn") if "is held by movement_exec" in line and "refused" in line]

    def free_drive_arm(self) -> None:
        """robot_control drives only Alice's arm, free drive's Keep on unticked."""
        self.plugin._arms = [SimpleNamespace(serial=SERIAL, arm=self.arm, free_drive_keep=SimpleNamespace(value=False),
                                             jog_speed=SimpleNamespace(value=0.5))]

    def commits(self) -> list[tuple[bool, str]]:
        """Commit a ready plan for Alice in each planner, through its API (`commit(plan_id)`), as a panel would."""
        arm_planner, base_planner = ArmPlannerPlugin(), BasePlannerPlugin()
        self.ctx.config.drive_root = None
        answers = []
        for planner, request, path in (
                (arm_planner, ArmPlanRequest(SERIAL, "ur_arm", tuple(START)),
                 ArmPath.at_preview_speed(arm_joint_names("ur_arm"), np.array([START, START]))),
                (base_planner, BasePlanRequest(SERIAL, (1.0, 0.0, 0.0)),
                 BasePath(np.array([0.0, 1.0]), np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])))):
            planner.setup(self.ctx)
            planner._plan = Plan(1, request, "ready", path)

            async def commit(planner=planner):
                return await planner.commit(1)

            answers.append(asyncio.run(commit()))
            planner._executor.shutdown()
            self.commit_tasks += len(planner._executions)
        return answers


def test_a_jog_and_a_cartesian_start_are_refused_while_any_robot_is_held():
    """A jog and a Cartesian Start on Alice send nothing while Alice is held, nor while Cindy is (one motion at a time).

    robot_control takes no lease meanwhile, and the chip says why.
    """
    lab = Lab()
    lab.free_drive_arm()
    for serial in (SERIAL, "a200-0806"):
        lease = lab.hold_robot(serial)
        lab.plugin._on_jog_hold(lab.ctx, KEY, 0, 1.0)
        lab.plugin._update_jogs(lab.ctx, lab.ctx.t)
        lab.plugin._start_move(lab.ctx, SERIAL, lab.arm, "cartesian move", lab.plugin._stream_cartesian_move,
                               np.array([0.3, 0.1, 0.4]), np.array([0.0, 0.0, 0.0, 1.0]))
        assert lab.plugin._refusals[KEY].label == "held" and lab.robot.lease_slot.lease is None
        give_back(lab.ctx.world, lease)
        lab.later(4.0)  # * past REFUSAL_SHOWN, so the next refusal is logged again
    assert not lab.plugin._trajectory_tasks and not lab.arm._trajectory.messages
    assert lab.refusals() == [f"a200-0804 ur_arm: refused: {serial} is held by movement_exec (B21_H_approach), one "
                              f"motion at a time; Cancel it first" for serial in (SERIAL, "a200-0806")]


def test_tool_buttons_are_refused_while_held_except_stop():
    """Open, Close, Go and Reactivate send nothing while held, on the laptop and the phone; Stop goes out."""
    lab = Lab()
    build_tool_inputs(lab.ctx, lab.ctx.gui, "a200-0804 ur_arm", lab.arm, lab.leased())
    build_phone_arm_inputs(lab.ctx, lab.ctx.gui, "a200-0804 ur_arm", lab.arm, lambda: None, lab.leased())
    grip, phone = (widget for widget in lab.ctx.gui.of_kind("button_group") if widget.label in ("Grip", "Gripper"))
    target, = (widget for widget in lab.ctx.gui.of_kind("button_group") if widget.label == "Target")
    driver, = (widget for widget in lab.ctx.gui.of_kind("button_group") if widget.label == "Driver")
    grip.callback("Open")  # * before the hold: a goal running, which Stop must still cancel
    goal, = lab.gripper._action.goals
    goal.on_answer(SimpleNamespace(result=lambda: goal))
    lab.robot.lease_slot.lease.release()  # robot_control gives the idle robot back
    lab.hold_robot()
    grip.callback("Open")
    grip.callback("Close")
    phone.callback("Open")
    target.callback()
    reactivations = []
    lab.gripper._reactivate = SimpleNamespace(service_is_ready=lambda: True, call_async=reactivations.append)
    driver.callback()
    assert len(lab.gripper._action.goals) == 1 and reactivations == [] and len(lab.refusals()) == 1
    assert lab.plugin._refusals[KEY].label == "held"
    grip.callback("Stop")
    assert goal.cancelled and len(lab.gripper._action.goals) == 1


def test_the_dpad_and_free_drive_are_refused_while_held():
    """A held D-pad sends no twist (a base already driving gets one stop); a held free drive button switches nothing.

    Each is said once while it stays held, not every tick. Released and idle, Alice is given back by robot_control;
    held then, her held buttons do nothing.
    """
    lab = Lab()
    lab.plugin._bases = [_BaseWidgets(serial=SERIAL, base=lab.robot.base, panel=None, status=None,
                                      speed=SimpleNamespace(value=0.5))]
    lab.plugin._on_hold(lab.ctx, SERIAL, False, 1.0, 0.0)
    lab.plugin.update(lab.ctx)
    assert lab.twists == [(0.5 * MAX_LINEAR_SPEED, 0.0)]
    # ? Alice marked held while robot_control drives her: since the robot lease, `take` refuses that, so this proves
    #   the D-pad's own check, a second line.
    marked = lab.set_held(SERIAL)
    for _ in range(3):
        lab.plugin._on_hold(lab.ctx, SERIAL, False, 1.0, 0.0)
        lab.plugin.update(lab.ctx)
    assert lab.twists[1:] == [(0.0, 0.0)] and len(lab.refusals()) == 1
    give_back(lab.ctx.world, marked)
    # * The same with Cindy marked held while Alice drives: Alice gets one stop, and no twist after it.
    lab.plugin._on_hold(lab.ctx, SERIAL, False, 1.0, 0.0)
    lab.plugin.update(lab.ctx)
    marked = lab.set_held("a200-0806")
    for _ in range(3):
        lab.plugin._on_hold(lab.ctx, SERIAL, False, 1.0, 0.0)
        lab.plugin.update(lab.ctx)
    assert lab.twists[2:] == [(0.5 * MAX_LINEAR_SPEED, 0.0), (0.0, 0.0)] and len(lab.refusals()) == 2
    give_back(lab.ctx.world, marked)
    lab.later(IDLE_AFTER + 0.5)
    lab.plugin.update(lab.ctx)
    assert lab.robot.lease_slot.lease is None  # * released and idle: given back

    lease = lab.hold_robot()
    lab.arm.controllers.state.switch_in_flight = None
    lab.free_drive_arm()
    for _ in range(3):
        lab.plugin._on_free_drive_hold(lab.ctx, KEY, True)
        lab.plugin._update_free_drive(lab.ctx, lab.ctx.t)
    assert lab.arm.controllers.switches == [] and len(lab.refusals()) == 3
    assert lab.arm._free_drive_until == float("-inf") and lab.robot.lease_slot.lease is None
    give_back(lab.ctx.world, lease)
    lab.plugin._update_free_drive(lab.ctx, lab.ctx.t)
    assert lab.arm.controllers.switches == [FREE_DRIVE_CONTROLLER]


def test_controller_switch_and_force_are_refused_while_held_and_a_zero_force_goes_out():
    """The CTRL buttons switch nothing and a force is not applied while held.

    A zero force still goes out, to the held robot too, without a lease (`clear_target_wrench`).
    """
    lab = Lab()
    switches = []
    controllers = SimpleNamespace(namespace="/a200_0804/ur5e", switchable=(FREE_DRIVE_CONTROLLER,),
                                  switch=switches.append)
    build_controller_panel(lab.ctx, lab.ctx.gui, controllers, {}, lab.leased())
    run, = lab.ctx.gui.of_kind("button_group")
    lab.hold_robot()
    run.callback("Free")
    assert switches == [] and len(lab.refusals()) == 1
    lab.plugin._apply_force(lab.ctx, KEY, lab.arm, np.array([0.0, 0.0, 5.0]))
    assert not lab.arm._target_wrench.messages and len(lab.refusals()) == 1  # * the same chip, said once
    lab.plugin._apply_force(lab.ctx, KEY, lab.arm, np.zeros(3))
    wrench, = lab.arm._target_wrench.messages  # * a zero force removes one: never refused
    assert wrench.wrench.force.z == 0.0 and lab.robot.lease_slot.lease is None


def test_hold_is_sent_and_cancels_the_holder():
    """Hold goes out while held, and calls the lock's stop, which cancels the executor."""
    lab = Lab()
    lab.hold_robot()
    lab.plugin._hold_arm(SERIAL, lab.arm)
    hold, = lab.arm._trajectory.messages
    assert len(hold.points) == 1 and lab.stops == [1]


def test_the_planners_do_not_send_to_a_held_robot():
    """Cindy held: the arm and base planners' Commit for Alice is refused with the holder's name; nothing is sent.

    Commit through the planner's API (`commit(plan_id)`), the way its panel and any other plugin send; the plan stays
    ready, and no lease is taken.
    """
    lab = Lab()
    lab.hold_robot("a200-0806")
    for sent, message in lab.commits():
        assert not sent and message == ("nothing sent: a200-0806 is held by movement_exec (B21_H_approach); "
                                        "Cancel it first")
    assert not lab.arm._trajectory.messages and lab.twists == [] and lab.robot.lease_slot.lease is None
    assert lab.commit_tasks == 0  # * refused at the Commit itself, before any task


def test_a_commit_whose_task_starts_after_a_robot_was_held_sends_nothing():
    """Commit taken while nobody is held, then Cindy held before its task runs: nothing sent, nothing leased.

    The check right before the lease answers why, in the Commit's own words.
    """
    lab = Lab()
    planner = ArmPlannerPlugin()
    planner.setup(lab.ctx)
    planner._plan = Plan(1, ArmPlanRequest(SERIAL, "ur_arm", tuple(START)),
                         "ready", ArmPath.at_preview_speed(arm_joint_names("ur_arm"), np.array([START, START])))

    async def scenario():
        future = planner.commit(1)
        lab.hold_robot("a200-0806")  # the commit's task has not run yet
        return await future

    sent, message = asyncio.run(scenario())
    planner._executor.shutdown()
    assert not sent and message == "nothing sent: a200-0806 is held by movement_exec (B21_H_approach); Cancel it first"
    assert not lab.arm._trajectory.messages and lab.robot.lease_slot.lease is None


def test_cancel_beside_commit_stops_a_running_commit_even_while_a_robot_is_held():
    """Cancel next to Commit is a stop, never refused: the running arm commit ends, the arm is held, its lease freed.

    Cindy is marked held while Alice's commit runs (since the robot lease, `take` refuses that): Cancel still works.
    """
    lab = Lab()
    planner = ArmPlannerPlugin()
    planner.setup(lab.ctx)
    goal = list(START)
    goal[0] += 0.2
    planner._plan = Plan(1, ArmPlanRequest(SERIAL, "ur_arm", tuple(goal)),
                         "ready", ArmPath.at_preview_speed(arm_joint_names("ur_arm"), np.array([START, goal])))

    async def scenario():
        sent = await planner.commit(1)
        leased = lab.robot.lease_slot.lease
        lab.set_held("a200-0806")
        cancelled = planner.cancel_commit()
        for _ in range(3):
            await asyncio.sleep(0)
        return sent, leased, cancelled

    (sent, _message), leased, cancelled = asyncio.run(scenario())
    planner._executor.shutdown()
    path, hold = lab.arm._trajectory.messages
    assert sent and cancelled and leased.holder == "robot_control: commit plan 1" and len(path.points) > 1
    assert len(hold.points) == 1 and lab.robot.lease_slot.lease is None


def test_robot_control_keeps_no_clearance_while_held_and_soft_stops_a_base_it_cannot_check():
    """Cindy held: a D-pad press on Alice starts no clearance and drives nothing; given back, an unchecked one stops.

    A clearance that cannot check Alice (not in the scene) soft stops her, saying why, as a trip would.
    """
    lab = Lab()
    stops, guards = [], []
    lab.robot.soft_stop = lambda reason="": stops.append(reason)

    def clearance(serials, margin=STOP_DISTANCE, unknown_arms=()):
        """`ctx.clearance` for a robot not in the scene: refused before its block runs."""
        guards.append((tuple(serials), unknown_arms))
        raise ClearanceRefused(f"robot {serials[0]} cannot be checked for clearance: it is not in the scene")

    lab.ctx.clearance = clearance
    plugin = lab.plugin
    plugin._bases = [_BaseWidgets(serial=SERIAL, base=lab.robot.base, panel=None, status=None,
                                  speed=SimpleNamespace(value=0.5), clearance=SimpleNamespace(value=True))]

    async def scenario():
        lease = lab.hold_robot("a200-0806")
        plugin._on_hold(lab.ctx, SERIAL, False, 1.0, 0.0)
        plugin.update(lab.ctx)
        await asyncio.sleep(0)
        held = (list(guards), list(lab.twists), dict(plugin._base_guards))
        give_back(lab.ctx.world, lease)
        plugin._on_hold(lab.ctx, SERIAL, False, 1.0, 0.0)
        plugin.update(lab.ctx)
        await asyncio.gather(*plugin._base_guards.values(), return_exceptions=True)
        return held

    held = asyncio.run(scenario())
    assert held == ([], [], {})
    refused = f"robot {SERIAL} cannot be checked for clearance: it is not in the scene"
    assert guards == [((SERIAL,), ALL_ARMS)] and stops == [refused]
    assert plugin._refusals[(SERIAL, "base")].label == "cannot check"


def test_the_view_only_paths_command_no_robot():
    """The collisions plugin and the arm planner's path check only read and draw: no command, lease or stop in them."""
    from husky_assembly_teleop.plugins import collisions
    from husky_assembly_teleop.plugins.arm_planner import planner as arm_planner
    commands = {"send_joint_trajectory", "send_joint_move", "send_cartesian_target", "send_target_wrench",
                "clear_target_wrench", "send_twist", "send_path", "stop_path", "send_multi_arm_trajectory", "hold",
                "switch", "restart", "request_free_drive", "zero_ft_sensor", "take_lease", "lease_robots", "soft_stop",
                "resume", "move", "open", "close", "stop", "drive_screw", "reactivate", "commit"}
    sources = {"collisions": inspect.getsource(collisions),
               "check_path": inspect.getsource(ArmPlannerPlugin.check_path) + inspect.getsource(
                   ArmPlannerPlugin._start_check),
               "path_hit": inspect.getsource(arm_planner.path_hit) + inspect.getsource(arm_planner.collision_hit)}
    for name, source in sources.items():
        tree = ast.parse(textwrap.dedent(source))
        called = {node.func.attr for node in ast.walk(tree)
                  if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)}
        assert not called & commands, (name, called & commands)


def test_health_unlock_resume_and_reconnect_are_refused_while_any_robot_is_held():
    """Unlock, Resume (they let arms move again) and Reconnect are refused for every robot while one is held."""
    lab = Lab()
    calls = []
    for robot in (lab.robot, lab.cindy):
        robot.unlock_protective_stop = lambda: calls.append("unlock")
        robot.resume = lambda: calls.append("resume")
        robot.reconnect = lambda: calls.append("reconnect")
    lab.hold_robot("a200-0806")
    for clicked in ("Unlock", "Resume", "Reconnect"):
        robot_action(lab.ctx, SERIAL, clicked)
    assert calls == [] and len(lab.refusals()) == 3
    give_back(lab.ctx.world, lab.cindy.lease)
    robot_action(lab.ctx, SERIAL, "Reconnect")
    assert calls == ["reconnect"]


def test_base_exp_does_not_start_while_a_robot_is_held():
    """A base experiment on Alice is not started while Cindy is held, nor while another plugin leases Alice."""
    from husky_assembly_teleop.plugins.base_exp.plugin import BaseExperimentPlugin
    lab = Lab()
    plugin = BaseExperimentPlugin()
    plugin.serial = SERIAL
    lease = lab.hold_robot("a200-0806")
    assert "a200-0806 is held by movement_exec" in plugin._start_problem(lab.ctx)
    give_back(lab.ctx.world, lease)
    lab.ctx.take_lease([SERIAL], "teleop")
    assert plugin._start_problem(lab.ctx) == "a200-0804 is leased by robot_control: teleop"


def test_a_base_experiment_sends_no_path_once_a_robot_is_held_before_its_send():
    """A run started while nobody was held, then Cindy held before its send: no path goes out, the outcome says why.

    ? Since the robot lease, Cindy cannot be taken while the run leases Alice; the mark skips `take`, to prove the
      check right before the send, a second line.
    """
    from husky_assembly_teleop.plugins.base_exp.plan import plan_from
    from husky_assembly_teleop.plugins.base_exp.plugin import BaseExperimentPlugin
    from husky_assembly_teleop.plugins.base_exp.templates import PARAMETERS, TEMPLATES
    lab = Lab()
    plugin = BaseExperimentPlugin()
    plugin.serial, plugin._experiment = SERIAL, SimpleNamespace(value="lease test")
    plugin._live = SimpleNamespace(clear=lambda: None)  # * the live plot's trace, made in setup
    settings = {"name": next(iter(TEMPLATES)), "parameters": {p.name: p.default for p in PARAMETERS},
                "mode": "Geometric", "linear_speed": 0.2, "turn_rate": 0.5}
    plan, problem = plan_from(SERIAL, settings, (0.0, 0.0, 0.0))
    paths = []
    lab.robot.base.send_path = lambda *args: paths.append(args) or 1
    lab.set_held("a200-0806")
    asyncio.run(plugin._run(lab.ctx, plan, recorded=False))
    assert problem == "" and paths == []
    assert plugin._outcome == "not sent: a200-0806 is held by movement_exec (B21_H_approach)"


def test_a_base_experiment_twist_run_stops_once_a_robot_is_held(monkeypatch):
    """A constant-command run, Cindy marked held as it starts: no command but zero goes out, and the outcome says why.

    ? As for the path run, `take` refuses Cindy while the run leases Alice; the mark proves the per-tick check.
    """
    from husky_assembly_teleop.plugins.base_exp import plugin as base_exp_module
    from husky_assembly_teleop.plugins.base_exp.twist import twist_plan
    lab = Lab()
    plugin = base_exp_module.BaseExperimentPlugin()
    plugin.serial, plugin._experiment = SERIAL, SimpleNamespace(value="lease test")
    plugin._live = SimpleNamespace(clear=lambda: None)
    plugin._tracking_signals, plugin._save = lambda: (), lambda *_args: None

    async def describe(*_args, **_options):
        return {"environment": "sim"}

    # * No recording or clearance here: the check is what is tested.
    monkeypatch.setattr(base_exp_module.record, "describe", describe)
    monkeypatch.setattr(base_exp_module, "_message_signals", lambda _robot, follower: ())
    monkeypatch.setattr(base_exp_module, "signals", SimpleNamespace(base_floor_pose=lambda _robot: None,
                                                                    follower_progress=lambda _robot: None))
    lab.ctx.record = lambda *_signals: nullcontext(SimpleNamespace())
    lab.ctx.clearance = lambda _serials, _margin: nullcontext(SimpleNamespace(problem=""))
    base = lab.robot.base
    base.mocap_id, base.state.last_fix_time = 1, 1e9
    base.state.marker_errors.append((1e9, 0.0))  # * a live pose throughout
    twists = []
    base.send_twist = lambda linear, angular, require_controller=True: twists.append((linear, angular)) or True
    lab.set_held("a200-0806")
    asyncio.run(plugin._run_twist(lab.ctx, twist_plan(SERIAL, 0.2, 0.0, (0.0, 0.0, 0.0)), {}))
    assert twists and not any(linear or angular for linear, angular in twists)
    assert plugin._outcome == "stopped: a200-0806 is held by movement_exec (B21_H_approach)"


def test_every_command_to_one_robot_is_refused_while_another_is_held():
    """Cindy held: every motion command to Alice is refused, nothing goes out to her, and she is never leased.

    Her jog, Cartesian Start, tool Open, controller switch, D-pad, free drive, and both planners' Commit.
    """
    lab = Lab()
    lab.hold_robot("a200-0806")
    control = lab.plugin
    control._bases = [_BaseWidgets(serial=SERIAL, base=lab.robot.base, panel=None, status=None,
                                   speed=SimpleNamespace(value=0.5))]
    control._on_hold(lab.ctx, SERIAL, False, 1.0, 0.0)
    control.update(lab.ctx)
    lab.free_drive_arm()
    control._on_jog_hold(lab.ctx, KEY, 0, 1.0)
    control._update_jogs(lab.ctx, lab.ctx.t)
    control._start_move(lab.ctx, SERIAL, lab.arm, "cartesian move", control._stream_cartesian_move,
                        np.array([0.3, 0.1, 0.4]), np.array([0.0, 0.0, 0.0, 1.0]))
    build_tool_inputs(lab.ctx, lab.ctx.gui, "a200-0804 ur_arm", lab.arm, lab.leased())
    switches = []
    controllers = SimpleNamespace(namespace="/a200_0804/ur5e", switchable=(FREE_DRIVE_CONTROLLER,),
                                  switch=switches.append)
    build_controller_panel(lab.ctx, lab.ctx.gui, controllers, {}, partial(control._leased, lab.ctx, (SERIAL, "base")))
    grip, = (widget for widget in lab.ctx.gui.of_kind("button_group") if widget.label == "Grip")
    run, = (widget for widget in lab.ctx.gui.of_kind("button_group") if widget.label == "Run")
    grip.callback("Open")
    run.callback("Free")
    lab.arm.controllers.state.switch_in_flight = None
    control._on_free_drive_hold(lab.ctx, KEY, True)
    control._update_free_drive(lab.ctx, lab.ctx.t)
    commits = lab.commits()
    assert not control._trajectory_tasks and not lab.arm._trajectory.messages and not lab.arm._target_frame.messages
    assert lab.gripper._action.goals == [] and switches == [] and lab.arm.controllers.switches == []
    assert lab.twists == [] and lab.robot.lease_slot.lease is None
    assert all(not sent and "a200-0806 is held by movement_exec" in message for sent, message in commits)
    # * One chip per part, said once: the arm's (jog, Start, tool, free drive) and the base's (switch, D-pad).
    assert len([line for line in lab.refusals() if line.startswith("a200-0804")]) == 2


def test_a_robot_robot_control_leases_keeps_the_executor_off_every_robot():
    """A jog on Alice leases her to robot_control: holding Cindy is refused meanwhile, naming the teleop lease.

    Once robot_control gives Alice back (idle), Cindy can be held.
    """
    lab = Lab()
    lab.free_drive_arm()
    lab.plugin._on_jog_hold(lab.ctx, KEY, 0, 1.0)
    lab.plugin._update_jogs(lab.ctx, lab.ctx.t)
    assert lab.arm._trajectory.messages and lab.robot.lease_slot.lease.holder == "robot_control: teleop"
    with pytest.raises(Busy, match="a200-0804 is leased by robot_control: teleop"):
        lab.hold_robot("a200-0806")
    lab.plugin._jog_held.clear()
    lab.plugin._update_jogs(lab.ctx, lab.ctx.t)  # released: the jog is held
    lab.arm.state.is_executing = False
    lab.later(IDLE_AFTER + 2.5)  # * idle, and still for the 2 s an automatic motion needs
    lab.plugin._leases.release_idle(lab.ctx, partial(lab.plugin._busy, lab.ctx.t))
    assert lab.robot.lease_slot.lease is None
    lab.hold_robot("a200-0806")


def test_a_cartesian_stream_stops_and_holds_when_another_robot_is_taken():
    """Cindy is marked held while Alice's Cartesian move streams: no target after it, one hold, one line naming both.

    ? Since the robot lease, `take` refuses Cindy while robot_control leases Alice; this per-tick check stays as a
      second line, so the mark here skips `take`.
    """
    lab = Lab()
    sent, holds = [], []
    lab.arm.state.tcp_position, lab.arm.state.tcp_orientation = np.array([0.3, 0.1, 0.4]), np.array([0, 0, 0, 1.0])
    lab.arm.send_cartesian_target = lambda _position, _orientation: sent.append(lab.ctx.t) or True
    lab.arm.hold = lambda: holds.append(lab.ctx.t) or True
    taken = []

    def take_cindy(t: float) -> None:
        """At 10.3 s, Cindy is held."""
        if t >= 10.25 and not taken:
            taken.append(lab.set_held("a200-0806"))

    lab.ctx.step = take_cindy
    asyncio.run(lab.plugin._stream_cartesian_move(lab.ctx, SERIAL, lab.arm, np.array([0.3, 0.1, 0.5]),
                                                  np.array([0, 0, 0, 1.0])))
    assert len(sent) == 3 and max(sent) < 10.25 and len(holds) == 1
    assert [line for line in lab.ctx.lines("warn") if "stopped" in line] == [
        "a200-0804 ur_arm: cartesian move stopped and held: a200-0806 is held by movement_exec (B21_H_approach), one "
        "motion at a time"]


def test_free_drive_ends_when_another_robot_is_taken():
    """Alice in free drive, her phone button still held; Cindy is marked held: back to the joint controller, once.

    ? Since the robot lease, `take` refuses Cindy while robot_control leases Alice for free drive; the mark here
      skips `take`, to prove the free-drive check that stays as a second line.
    """
    lab = Lab()
    lab.arm.controllers.state.switch_in_flight = None
    lab.free_drive_arm()
    lab.plugin._on_free_drive_hold(lab.ctx, KEY, True)
    lab.plugin._update_free_drive(lab.ctx, lab.ctx.t)
    lab.set_held("a200-0806")
    for _ in range(3):
        lab.plugin._on_free_drive_hold(lab.ctx, KEY, True)
        lab.plugin._update_free_drive(lab.ctx, lab.ctx.t)
    assert lab.arm.controllers.switches == [FREE_DRIVE_CONTROLLER, SCALED_JOINT_TRAJECTORY_CONTROLLER]
    assert lab.refusals() == ["a200-0804 ur_arm: refused: a200-0806 is held by movement_exec (B21_H_approach), one "
                              "motion at a time; Cancel it first"]


def test_free_drive_started_outside_the_monitor_ends_when_another_robot_is_taken():
    """Alice runs the Free controller, started outside the monitor; Cindy is taken: back to the joint controller, once.

    ! No button held, Keep on unticked, and not freed by this monitor: only the running Free controller says Alice is
      in free drive (chosen on the laptop with no button held, or started by the tool's buttons on the robot).
    """
    lab = Lab()
    switches = []
    lab.arm.controllers.switch = lambda controller, unguarded=False: switches.append(controller) or True
    lab.arm.controllers.state.switch_in_flight = None
    lab.free_drive_arm()
    lab.arm.state.controllers.active = FREE_DRIVE_CONTROLLER
    lab.hold_robot("a200-0806")
    lab.plugin._update_free_drive(lab.ctx, lab.ctx.t)
    assert switches == [SCALED_JOINT_TRAJECTORY_CONTROLLER] and lab.arm._free_drive_until == float("-inf")
    # * The arm still reports Free: the switch back is not asked again within FREE_DRIVE_RETRY.
    lab.later(FREE_DRIVE_RETRY / 2)
    lab.plugin._update_free_drive(lab.ctx, lab.ctx.t)
    assert switches == [SCALED_JOINT_TRAJECTORY_CONTROLLER] and lab.arm._free_drive_until == float("-inf")


def test_a_quiet_controller_manager_is_not_asked_to_switch_back_during_a_lease():
    """Alice's controller manager last said Free runs, then went quiet; Cindy is taken: no switch back, no error line.

    It could not switch anyway, and free drive ends within 1 s with no requests. Once it answers again, still in Free,
    the switch back goes out, once.
    """
    lab = Lab()
    lab.free_drive_arm()
    state = lab.arm.state.controllers
    state.active, state.last_update_time = FREE_DRIVE_CONTROLLER, lab.ctx.t - CONTROLLER_STATE_MAX_AGE - 1.0
    switches = []

    def switch(controller: str, unguarded: bool = False) -> bool:
        """Like the real switch: while its manager is quiet the service is gone, so an error line and nothing sent."""
        switches.append(controller)
        if lab.ctx.t - state.last_update_time > CONTROLLER_STATE_MAX_AGE:
            lab.arm._node.get_logger().error("/ur_arm: switch_controller is not available")
            return False
        return True

    lab.arm.controllers.switch = switch
    lab.hold_robot("a200-0806")
    for _ in range(100):  # 10 s of ticks
        lab.ctx.t += 0.1
        lab.plugin._update_free_drive(lab.ctx, lab.ctx.t)
    assert switches == [] and lab.ctx.lines("error") == [] and lab.arm._free_drive_until == float("-inf")
    state.last_update_time = lab.ctx.t
    for _ in range(3):
        lab.plugin._update_free_drive(lab.ctx, lab.ctx.t)
    assert switches == [SCALED_JOINT_TRAJECTORY_CONTROLLER] and lab.ctx.lines("error") == []


def test_free_drive_is_not_requested_while_another_robot_is_held():
    """Cindy held: Alice's laptop button, Keep on and phone button ask for no free drive, and Keep on is unticked.

    Alice runs the laptop's Free controller, so she goes back to the joint controller, once. Given back, the phone's
    button frees her again, under robot_control's lease.
    """
    lab = Lab()
    lab.free_drive_arm()
    keep = lab.plugin._arms[0].free_drive_keep
    lab.arm.state.controllers.active = lab.arm.state.controllers.requested = FREE_DRIVE_CONTROLLER
    keep.value = True
    lease = lab.hold_robot("a200-0806")
    for phone in (False, True):
        lab.plugin._on_free_drive_hold(lab.ctx, KEY, phone)
        lab.plugin._update_free_drive(lab.ctx, lab.ctx.t)
    assert lab.arm._free_drive_until == float("-inf") and not keep.value
    assert lab.arm.controllers.switches == [SCALED_JOINT_TRAJECTORY_CONTROLLER] and len(lab.refusals()) == 1
    give_back(lab.ctx.world, lease)
    lab.plugin._on_free_drive_hold(lab.ctx, KEY, True)
    lab.plugin._update_free_drive(lab.ctx, lab.ctx.t)
    assert lab.arm._free_drive_until > lab.ctx.t and lab.robot.lease_slot.lease in lab.arm._free_drive_leases
    assert lab.arm.controllers.switches == [SCALED_JOINT_TRAJECTORY_CONTROLLER, FREE_DRIVE_CONTROLLER]


def test_keep_on_is_unticked_while_another_robot_is_held_even_when_the_switch_back_fails():
    """Keep on frees Alice, whose switch service is down; Cindy taken: Keep on is unticked all the same.

    Else it would free Alice again the moment Cindy's automatic motion ends.
    """
    lab = Lab()
    lab.free_drive_arm()
    keep = lab.plugin._arms[0].free_drive_keep
    lab.arm.state.controllers.active = lab.arm.state.controllers.requested = FREE_DRIVE_CONTROLLER
    lab.arm.controllers.switch = lambda _controller, unguarded=False: False
    keep.value = True
    lease = lab.hold_robot("a200-0806")
    lab.plugin._update_free_drive(lab.ctx, lab.ctx.t)
    assert not keep.value and lab.arm._free_drive_until == float("-inf")
    give_back(lab.ctx.world, lease)
    lab.plugin._update_free_drive(lab.ctx, lab.ctx.t + 2.0)
    assert lab.arm._free_drive_until == float("-inf")
