"""While any robot is held for an automatic motion, motion commands to every robot are refused; Hold and Stop work."""

from __future__ import annotations

from functools import partial
from types import SimpleNamespace

import asyncio

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
from husky_assembly_teleop.robot_interface.arm import (CARTESIAN_COMPLIANCE_CONTROLLER, FREE_DRIVE_CONTROLLER,
                                                       SCALED_JOINT_TRAJECTORY_CONTROLLER)
from husky_assembly_teleop.robot_interface.base import BaseState
from husky_assembly_teleop.robot_interface.controller_manager import CONTROLLER_STATE_MAX_AGE
from husky_assembly_teleop.shared.arm_planner_contract import ArmPlanRequest
from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names
from husky_assembly_teleop.shared.base_planner_contract import BasePlanRequest
from husky_assembly_teleop.shared.execution.interlock import give_back, take
from husky_assembly_teleop.shared.planner.plan import Plan

pytestmark = pytest.mark.safety

SERIAL = "a200-0804"
START = [0.0, -1.5, 1.5, 0.0, 0.5, 0.0]


class Lab:
    """Alice with a Robotiq and Cindy, robot_control set up on them (no viser), and a lease to take."""

    def __init__(self):
        """Build the robots and the plugin; nobody is held yet."""
        self.ctx = FakeContext()
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
                                          state=BaseState())
        self.ctx.world.robots[SERIAL] = self.robot
        self.plugin = RobotControlPlugin()
        self.plugin._ctx = self.ctx
        self.stops = []

    def hold_robot(self, serial: str = SERIAL):
        """Take a robot's lease (Alice's by default), as the executor does; its stop is counted."""
        return take(self.ctx.world, serial, "movement_exec", "B21_H_approach", lambda: self.stops.append(1),
                    self.ctx.t)

    def refusals(self) -> list[str]:
        """The refusal lines logged so far."""
        return [line for line in self.ctx.lines("warn") if "is held by movement_exec" in line and "refused" in line]

    def free_drive_arm(self) -> None:
        """robot_control drives only Alice's arm, free drive's Keep on unticked."""
        self.plugin._arms = [SimpleNamespace(serial=SERIAL, arm=self.arm, free_drive_keep=SimpleNamespace(value=False))]

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
        return answers


def test_start_is_refused_while_any_robot_is_held():
    """A joint move's Start on Alice sends nothing while Alice is held, nor while Cindy is (one motion at a time)."""
    lab = Lab()
    for serial in (SERIAL, "a200-0806"):
        lease = lab.hold_robot(serial)
        lab.plugin._start_move(lab.ctx, SERIAL, lab.arm, "move", lab.plugin._execute_move, np.array(START))
        give_back(lab.ctx.world, lease)
    assert not lab.plugin._trajectory_tasks and not lab.arm._trajectory.messages
    assert lab.refusals() == [f"a200-0804 ur_arm move refused: {serial} is held by movement_exec (B21_H_approach), one "
                              f"motion at a time; Cancel it first" for serial in (SERIAL, "a200-0806")]


def test_tool_buttons_are_refused_while_held_except_stop():
    """Open, Close, Go and Reactivate send nothing while held, on the laptop and the phone; Stop goes out."""
    lab = Lab()
    lab.hold_robot()
    build_tool_inputs(lab.ctx, lab.ctx.gui, "a200-0804 ur_arm", lab.arm, partial(lab.plugin._allowed, SERIAL))
    build_phone_arm_inputs(lab.ctx, lab.ctx.gui, "a200-0804 ur_arm", lab.arm, lambda: None,
                           partial(lab.plugin._allowed, SERIAL))
    grip, phone = (widget for widget in lab.ctx.gui.of_kind("button_group") if widget.label in ("Grip", "Gripper"))
    target, = (widget for widget in lab.ctx.gui.of_kind("button_group") if widget.label == "Target")
    grip.callback("Open")
    grip.callback("Close")
    phone.callback("Open")
    target.callback()
    assert lab.gripper._action.goals == [] and len(lab.refusals()) == 4
    grip.callback("Stop")
    assert len(lab.gripper._action.goals) == 1


def test_the_dpad_and_free_drive_are_refused_while_held():
    """A held D-pad sends no twist (a base already driving gets one stop); a held free drive button switches nothing.

    Each is said once per lease, not every tick.
    """
    lab = Lab()
    lab.plugin._bases = [_BaseWidgets(serial=SERIAL, base=lab.robot.base, panel=None, status=None,
                                      speed=SimpleNamespace(value=0.5))]
    lab.plugin._on_hold(lab.ctx, SERIAL, False, 1.0, 0.0)
    lab.plugin.update(lab.ctx)
    assert lab.twists == [(0.5 * MAX_LINEAR_SPEED, 0.0)]
    lease = lab.hold_robot()
    for _ in range(3):
        lab.plugin._on_hold(lab.ctx, SERIAL, False, 1.0, 0.0)
        lab.plugin.update(lab.ctx)
    assert lab.twists[1:] == [(0.0, 0.0)] and len(lab.refusals()) == 1

    switches = []
    lab.arm.controllers.switch = lambda controller: switches.append(controller) or True
    lab.arm.controllers.state.switch_in_flight = None
    lab.free_drive_arm()
    for _ in range(3):
        lab.plugin._on_free_drive_hold(lab.ctx, (SERIAL, "ur_arm"), True)
        lab.plugin._update_free_drive(lab.ctx.t)
    assert switches == [] and len(lab.refusals()) == 2 and lab.arm._free_drive_until == float("-inf")
    give_back(lab.ctx.world, lease)
    lab.plugin._update_free_drive(lab.ctx.t)
    assert switches == [FREE_DRIVE_CONTROLLER]


def test_controller_switch_and_force_are_refused_while_held():
    """The CTRL buttons switch nothing and a force is not applied while held; a zero force still goes out."""
    lab = Lab()
    switches = []
    controllers = SimpleNamespace(namespace="/a200_0804/ur5e", switchable=(FREE_DRIVE_CONTROLLER,),
                                  switch=switches.append)
    build_controller_panel(lab.ctx, lab.ctx.gui, controllers, {}, partial(lab.plugin._allowed, SERIAL))
    run, = lab.ctx.gui.of_kind("button_group")
    lab.hold_robot()
    run.callback("Free")
    assert switches == [] and len(lab.refusals()) == 1
    lab.plugin._apply_force((SERIAL, "ur_arm"), lab.arm, np.array([0.0, 0.0, 5.0]))
    assert not lab.arm._target_wrench.messages and len(lab.refusals()) == 2
    lab.arm.state.controllers.active = CARTESIAN_COMPLIANCE_CONTROLLER
    lab.plugin._apply_force((SERIAL, "ur_arm"), lab.arm, np.zeros(3))
    assert len(lab.arm._target_wrench.messages) == 1 and len(lab.refusals()) == 2  # * a zero force removes one


def test_hold_is_sent_and_cancels_the_holder():
    """Hold goes out while held, and calls the lease's stop, which cancels the executor."""
    lab = Lab()
    lab.hold_robot()
    lab.plugin._hold_arm(SERIAL, lab.arm)
    hold, = lab.arm._trajectory.messages
    assert len(hold.points) == 1 and lab.stops == [1]


def test_the_planners_do_not_send_to_a_held_robot():
    """Cindy held: the arm and base planners' Commit for Alice is refused with the holder's name; nothing is sent.

    Commit through the planner's API (`commit(plan_id)`), the way its panel and any other plugin send; the plan stays
    ready.
    """
    lab = Lab()
    lab.hold_robot("a200-0806")
    for sent, message in lab.commits():
        assert not sent and message == ("nothing sent: a200-0806 is held by movement_exec (B21_H_approach); "
                                        "Cancel it first")
    assert not lab.arm._trajectory.messages and lab.twists == []


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
    """A base experiment on Alice is not started while Cindy is held."""
    from husky_assembly_teleop.plugins.base_exp.plugin import BaseExperimentPlugin
    lab = Lab()
    plugin = BaseExperimentPlugin()
    plugin.serial = SERIAL
    lab.hold_robot("a200-0806")
    assert "a200-0806 is held by movement_exec" in plugin._start_problem(lab.ctx)


def test_every_command_to_one_robot_is_refused_while_another_is_held():
    """Cindy held: every motion command to Alice is refused, and nothing goes out to her.

    Her joint Start, Cartesian Start, tool Open, controller switch, D-pad, free drive, and both planners' Commit.
    """
    lab = Lab()
    lab.hold_robot("a200-0806")
    control = lab.plugin
    control._start_move(lab.ctx, SERIAL, lab.arm, "move", control._execute_move, np.array(START))
    control._start_move(lab.ctx, SERIAL, lab.arm, "cartesian move", control._stream_cartesian_move,
                        np.array([0.3, 0.1, 0.4]), np.array([0.0, 0.0, 0.0, 1.0]))
    build_tool_inputs(lab.ctx, lab.ctx.gui, "a200-0804 ur_arm", lab.arm, partial(control._allowed, SERIAL))
    switches = []
    controllers = SimpleNamespace(namespace="/a200_0804/ur5e", switchable=(FREE_DRIVE_CONTROLLER,),
                                  switch=switches.append)
    build_controller_panel(lab.ctx, lab.ctx.gui, controllers, {}, partial(control._allowed, SERIAL))
    grip, = (widget for widget in lab.ctx.gui.of_kind("button_group") if widget.label == "Grip")
    run, = (widget for widget in lab.ctx.gui.of_kind("button_group") if widget.label == "Run")
    grip.callback("Open")
    run.callback("Free")
    control._bases = [_BaseWidgets(serial=SERIAL, base=lab.robot.base, panel=None, status=None,
                                   speed=SimpleNamespace(value=0.5))]
    control._on_hold(lab.ctx, SERIAL, False, 1.0, 0.0)
    control.update(lab.ctx)
    free_drive = []
    lab.arm.controllers.switch = lambda controller: free_drive.append(controller) or True
    lab.arm.controllers.state.switch_in_flight = None
    lab.free_drive_arm()
    control._on_free_drive_hold(lab.ctx, (SERIAL, "ur_arm"), True)
    control._update_free_drive(lab.ctx.t)
    commits = lab.commits()
    assert not control._trajectory_tasks and not lab.arm._trajectory.messages and not lab.arm._target_frame.messages
    assert lab.gripper._action.goals == [] and switches == [] and free_drive == [] and lab.twists == []
    assert all(not sent and "a200-0806 is held by movement_exec" in message for sent, message in commits)
    assert len([line for line in lab.refusals() if line.startswith("a200-0804")]) == 6


def test_a_cartesian_stream_stops_and_holds_when_another_robot_is_taken():
    """Cindy is taken while Alice's Cartesian move streams: no target after it, one hold, one line naming both."""
    lab = Lab()
    sent, holds = [], []
    lab.arm.state.tcp_position, lab.arm.state.tcp_orientation = np.array([0.3, 0.1, 0.4]), np.array([0, 0, 0, 1.0])
    lab.arm.send_cartesian_target = lambda _position, _orientation: sent.append(lab.ctx.t) or True
    lab.arm.hold = lambda: holds.append(lab.ctx.t) or True
    taken = []

    def take_cindy(t: float) -> None:
        """At 10.3 s, the executor takes Cindy."""
        if t >= 10.25 and not taken:
            taken.append(lab.hold_robot("a200-0806"))

    lab.ctx.step = take_cindy
    asyncio.run(lab.plugin._stream_cartesian_move(lab.ctx, SERIAL, lab.arm, np.array([0.3, 0.1, 0.5]),
                                                  np.array([0, 0, 0, 1.0])))
    assert len(sent) == 3 and max(sent) < 10.25 and len(holds) == 1
    assert [line for line in lab.ctx.lines("warn") if "stopped" in line] == [
        "a200-0804 ur_arm: cartesian move stopped and held: a200-0806 is held by movement_exec (B21_H_approach), one "
        "motion at a time"]


def test_free_drive_ends_when_another_robot_is_taken():
    """Alice in free drive, her button still held; Cindy is taken: Alice goes back to the joint controller, once."""
    lab = Lab()
    switches = []
    lab.arm.controllers.switch = lambda controller: switches.append(controller) or True
    lab.arm.controllers.state.switch_in_flight = None
    lab.free_drive_arm()
    lab.plugin._on_free_drive_hold(lab.ctx, (SERIAL, "ur_arm"), True)
    lab.plugin._update_free_drive(lab.ctx.t)
    lab.arm.state.controllers.active = FREE_DRIVE_CONTROLLER
    lab.hold_robot("a200-0806")
    for _ in range(3):
        lab.plugin._on_free_drive_hold(lab.ctx, (SERIAL, "ur_arm"), True)
        lab.plugin._update_free_drive(lab.ctx.t)
    assert switches == [FREE_DRIVE_CONTROLLER, SCALED_JOINT_TRAJECTORY_CONTROLLER]
    assert lab.refusals() == ["a200-0804 ur_arm free drive refused: a200-0806 is held by movement_exec "
                              "(B21_H_approach), one motion at a time; Cancel it first"]


def test_free_drive_started_outside_the_monitor_ends_when_another_robot_is_taken():
    """Alice runs the Free controller, started outside the monitor; Cindy is taken: back to the joint controller, once.

    ! No button held, Keep on unticked, and not freed by this monitor: only the running Free controller says Alice is
      in free drive (chosen on the laptop with no button held, or started by the tool's buttons on the robot).
    """
    lab = Lab()
    switches = []
    lab.arm.controllers.switch = lambda controller: switches.append(controller) or True
    lab.arm.controllers.state.switch_in_flight = None
    lab.free_drive_arm()
    lab.arm.state.controllers.active = FREE_DRIVE_CONTROLLER
    lab.hold_robot("a200-0806")
    lab.plugin._update_free_drive(lab.ctx.t)
    assert switches == [SCALED_JOINT_TRAJECTORY_CONTROLLER] and lab.arm._free_drive_until == float("-inf")
    # * The arm still reports Free: the switch back is not asked again within FREE_DRIVE_RETRY.
    lab.ctx.t += FREE_DRIVE_RETRY / 2
    lab.plugin._update_free_drive(lab.ctx.t)
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

    def switch(controller: str) -> bool:
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
        lab.plugin._update_free_drive(lab.ctx.t)
    assert switches == [] and lab.ctx.lines("error") == [] and lab.arm._free_drive_until == float("-inf")
    state.last_update_time = lab.ctx.t
    for _ in range(3):
        lab.plugin._update_free_drive(lab.ctx.t)
    assert switches == [SCALED_JOINT_TRAJECTORY_CONTROLLER] and lab.ctx.lines("error") == []


def test_free_drive_is_not_requested_while_another_robot_is_held():
    """Cindy held: Alice's laptop button, Keep on and phone button ask for no free drive, and Keep on is unticked.

    Alice runs the laptop's Free controller, so she goes back to the joint controller, once. Given back, the phone's
    button frees her again.
    """
    lab = Lab()
    lab.free_drive_arm()
    keep = lab.plugin._arms[0].free_drive_keep
    lab.arm.state.controllers.active = lab.arm.state.controllers.requested = FREE_DRIVE_CONTROLLER
    keep.value = True
    lease = lab.hold_robot("a200-0806")
    for phone in (False, True):
        lab.plugin._on_free_drive_hold(lab.ctx, (SERIAL, "ur_arm"), phone)
        lab.plugin._update_free_drive(lab.ctx.t)
    assert lab.arm._free_drive_until == float("-inf") and not keep.value
    assert lab.arm.controllers.switches == [SCALED_JOINT_TRAJECTORY_CONTROLLER] and len(lab.refusals()) == 1
    give_back(lab.ctx.world, lease)
    lab.plugin._on_free_drive_hold(lab.ctx, (SERIAL, "ur_arm"), True)
    lab.plugin._update_free_drive(lab.ctx.t)
    assert lab.arm._free_drive_until > lab.ctx.t
    assert lab.arm.controllers.switches == [SCALED_JOINT_TRAJECTORY_CONTROLLER, FREE_DRIVE_CONTROLLER]


def test_keep_on_is_unticked_while_another_robot_is_held_even_when_the_switch_back_fails():
    """Keep on frees Alice, whose switch service is down; Cindy taken: Keep on is unticked all the same.

    Else it would free Alice again the moment Cindy's automatic motion ends.
    """
    lab = Lab()
    lab.free_drive_arm()
    keep = lab.plugin._arms[0].free_drive_keep
    lab.arm.state.controllers.active = lab.arm.state.controllers.requested = FREE_DRIVE_CONTROLLER
    lab.arm.controllers.switch = lambda _controller: False
    keep.value = True
    lease = lab.hold_robot("a200-0806")
    lab.plugin._update_free_drive(lab.ctx.t)
    assert not keep.value and lab.arm._free_drive_until == float("-inf")
    give_back(lab.ctx.world, lease)
    lab.plugin._update_free_drive(lab.ctx.t + 2.0)
    assert lab.arm._free_drive_until == float("-inf")
