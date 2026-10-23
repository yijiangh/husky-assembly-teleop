"""
While the executor leases every robot, robot_control, the planners and base_exp get no lease and send nothing.

Hold and Stop need no lease and still go out. A robot robot_control holds the teleop lease of keeps the executor off
every robot, since the executor leases all of them or none.
"""

from __future__ import annotations

import ast
import asyncio
import inspect
import textwrap
from functools import partial
from types import SimpleNamespace

import numpy as np
import pytest
from plugin_fakes import FakeContext, fake_arm, fake_node, fake_robot, fake_robotiq

from husky_assembly_teleop.plugins.arm_planner.planner import ArmPath
from husky_assembly_teleop.plugins.arm_planner.planner_plugin import ArmPlannerPlugin
from husky_assembly_teleop.plugins.base_planner.path import BasePath
from husky_assembly_teleop.plugins.base_planner.planner_plugin import BasePlannerPlugin
from husky_assembly_teleop.plugins.robot_control.arm_panel import build_tool_inputs
from husky_assembly_teleop.plugins.robot_control.plugin import RobotControlPlugin, _BaseWidgets
from husky_assembly_teleop.plugins.robot_control.status import build_controller_panel
from husky_assembly_teleop.plugins.robot_control.teleop_lease import IDLE_AFTER, TeleopLeases
from husky_assembly_teleop.robot_interface.arm import CARTESIAN_COMPLIANCE_CONTROLLER, FREE_DRIVE_CONTROLLER
from husky_assembly_teleop.robot_interface.base import BaseState
from husky_assembly_teleop.robot_interface.robot_lease import Lease, NotLeased, take_lease
from husky_assembly_teleop.shared.arm_planner_contract import ArmPlanRequest
from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names
from husky_assembly_teleop.shared.base_planner_contract import BasePlanRequest
from husky_assembly_teleop.shared.planner.plan import Plan
from husky_assembly_teleop.shared.teleop.dpad import Hold

pytestmark = pytest.mark.safety

SERIAL = "a200-0804"
START = [0.0, -1.5, 1.5, 0.0, 0.5, 0.0]
KEY = (SERIAL, "ur_arm")
#: The executor's lease, as `ctx.take_lease` names it.
EXEC = "movement_exec: exec B21_H_approach"


class Lab:
    """Alice with a Robotiq and Cindy, and robot_control set up on them (no viser)."""

    def __init__(self):
        """Build the robots and the plugin; nobody is leased yet."""
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
        #: How many commit tasks the planners started (`commits`): a refusal at the Commit starts none.
        self.commit_tasks = 0

    def lease_every_robot(self) -> Lease:
        """Lease every robot as the executor does (`MovementExecPlugin.request_execute`).

        Raises:
            NotLeased: If any robot is leased already.
        """
        return take_lease([robot.lease_slot for robot in self.ctx.world.robots.values()], EXEC)

    def later(self, seconds: float) -> None:
        """Let `seconds` pass, every arm measured, synced and its controller manager heard from again at the end."""
        self.ctx.t += seconds
        for robot in (self.robot, self.cindy):
            for arm in robot.arms.values():
                arm.state.last_update_time = arm.state.status_update_time = self.ctx.t
                arm.state.controllers.last_update_time = self.ctx.t

    def compliance(self) -> None:
        """Alice's arm under the compliance controller, its TCP measured now."""
        state = self.arm.state
        state.controllers.active = state.controllers.requested = CARTESIAN_COMPLIANCE_CONTROLLER
        state.tcp_position, state.tcp_orientation = np.array([0.3, 0.1, 0.4]), np.array([0.0, 0.0, 0.0, 1.0])
        state.tcp_raw_position, state.tcp_raw_orientation = state.tcp_position, state.tcp_orientation
        state.tcp_update_time = self.ctx.t

    def leased(self):
        """robot_control's `leased` for Alice's arm, as its builders get it."""
        return partial(self.plugin._leased, self.ctx, KEY)

    def refusals(self) -> list[str]:
        """The refusal lines logged so far."""
        return [line for line in self.ctx.lines("warn") if f"is leased by {EXEC}" in line]

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
                 ArmPath.along(arm_joint_names("ur_arm"), np.array([START, np.add(START, 0.01)]))),
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


def test_every_command_to_alice_is_refused_while_the_executor_leases_every_robot():
    """Every motion command to Alice is refused, nothing goes out to her, and the lease stays the executor's.

    Her jog, Cartesian Start, tool Open, controller switch, D-pad, free drive, and both planners' Commit.
    """
    lab = Lab()
    lease = lab.lease_every_robot()
    control = lab.plugin
    control._bases = [_BaseWidgets(serial=SERIAL, base=lab.robot.base, panel=None, status=None,
                                   speed=SimpleNamespace(value=0.5))]
    control._on_hold(SERIAL, Hold(1.0, 0.0, control._bases[0].speed, lab.ctx.now()))
    control.update(lab.ctx)
    lab.free_drive_arm()
    control._on_jog_hold(lab.ctx, KEY, 0, 1.0)
    control._update_jogs(lab.ctx, lab.ctx.t)

    async def start():
        """Start a Cartesian move, as the Start button does; its task meets the lease."""
        control._start_move(lab.ctx, SERIAL, lab.arm, "cartesian move", control._stream_cartesian_move,
                            np.array([0.3, 0.1, 0.4]), np.array([0.0, 0.0, 0.0, 1.0]))
        await asyncio.gather(*control._trajectory_tasks.values(), return_exceptions=True)

    asyncio.run(start())
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
    assert not lab.arm._trajectory.messages and not lab.arm._target_frame.messages
    assert lab.gripper._action.goals == [] and switches == [] and lab.arm.controllers.switches == []
    assert lab.twists == [] and lab.robot.lease_slot.lease is lease and lab.cindy.lease_slot.lease is lease
    assert all(not sent and f"is leased by {EXEC}" in message for sent, message in commits)
    assert lab.refusals()


def test_stop_and_hold_go_out_while_the_executor_leases_every_robot():
    """A tool's Stop cancels its goal and Hold sends a one-point hold: neither needs a lease."""
    lab = Lab()
    build_tool_inputs(lab.ctx, lab.ctx.gui, "a200-0804 ur_arm", lab.arm, lab.leased())
    grip, = (widget for widget in lab.ctx.gui.of_kind("button_group") if widget.label == "Grip")
    grip.callback("Open")  # * before the lease: a goal running, which Stop must still cancel
    goal, = lab.gripper._action.goals
    goal.on_answer(SimpleNamespace(result=lambda: goal))
    lab.robot.lease_slot.lease.release()  # robot_control gives the idle robot back
    lab.lease_every_robot()
    grip.callback("Stop")
    assert goal.cancelled and len(lab.gripper._action.goals) == 1
    lab.plugin._hold_arm(SERIAL, lab.arm)
    hold, = lab.arm._trajectory.messages
    assert len(hold.points) == 1


def test_a_zero_force_under_compliance_targets_the_tcp_first_so_the_arm_stays():
    """Alice leased to the executor, under compliance: Zero sets a target at her measured TCP, then zeroes the wrench.

    No lease, never refused: else an arm the force pushed off its target would spring back to it (up to 10 cm). With
    the TCP stale the wrench is zeroed all the same, and the chip says the arm may move.
    """
    lab = Lab()
    lab.compliance()
    lease = lab.lease_every_robot()
    sent = []
    lab.arm.targets_sent.attach(lambda record: sent.append(("target", record.position.tolist(), record.hold)))
    lab.arm.wrenches_sent.attach(lambda record: sent.append(("wrench", record.force.tolist())))
    lab.plugin._apply_force(lab.ctx, KEY, lab.arm, np.zeros(3))
    assert sent == [("target", [0.3, 0.1, 0.4], 1), ("wrench", [0.0, 0.0, 0.0])]
    assert lab.robot.lease_slot.lease is lease
    lab.arm.state.tcp_update_time = lab.ctx.t - 1.0
    lab.plugin._apply_force(lab.ctx, KEY, lab.arm, np.zeros(3))
    assert sent[2:] == [("wrench", [0.0, 0.0, 0.0])] and "may move" in lab.arm.last_refusal.reason


def test_an_applied_force_keeps_robot_controls_lease_so_exec_stays_busy():
    """A 5 N force on Alice keeps robot_control's teleop lease past the idle time: the executor gets no robot meanwhile.

    Once the force is zeroed and Alice is idle, robot_control gives her back and the executor can.
    """
    lab = Lab()
    lab.compliance()
    lab.free_drive_arm()
    lab.plugin._apply_force(lab.ctx, KEY, lab.arm, np.array([0.0, 0.0, 5.0]))
    assert len(lab.arm._target_wrench.messages) == 1 and lab.robot.lease_slot.lease.holder == "robot_control: teleop"
    lab.later(IDLE_AFTER + 2.5)
    lab.plugin._leases.release_idle(lab.ctx, partial(lab.plugin._busy, lab.ctx.t))
    with pytest.raises(NotLeased, match="a200-0804 is leased by robot_control: teleop"):
        lab.lease_every_robot()
    lab.plugin._apply_force(lab.ctx, KEY, lab.arm, np.zeros(3))
    lab.later(IDLE_AFTER + 0.5)
    lab.plugin._leases.release_idle(lab.ctx, partial(lab.plugin._busy, lab.ctx.t))
    assert lab.robot.lease_slot.lease is None
    lab.lease_every_robot()


def test_robot_control_keeps_a_robot_it_still_uses_past_the_idle_time():
    """The teleop lease is given back only once the robot is unused for IDLE_AFTER and robot_control says idle."""
    lab = Lab()
    leases = TeleopLeases()
    with leases.use(lab.ctx, SERIAL):
        pass
    lab.later(IDLE_AFTER + 0.5)
    leases.release_idle(lab.ctx, lambda _serial: True)
    assert leases.held(SERIAL) and lab.robot.lease_slot.lease is not None
    leases.release_idle(lab.ctx, lambda _serial: False)
    assert not leases.held(SERIAL) and lab.robot.lease_slot.lease is None


def test_a_robot_robot_control_leases_keeps_the_executor_off_every_robot():
    """A jog on Alice leases her to robot_control: the executor gets no robot, the refusal naming the teleop lease.

    Once robot_control gives Alice back (idle), the executor gets them.
    """
    lab = Lab()
    lab.free_drive_arm()
    lab.plugin._on_jog_hold(lab.ctx, KEY, 0, 1.0)
    lab.plugin._update_jogs(lab.ctx, lab.ctx.t)
    assert lab.arm._trajectory.messages and lab.robot.lease_slot.lease.holder == "robot_control: teleop"
    with pytest.raises(NotLeased, match="a200-0804 is leased by robot_control: teleop"):
        lab.lease_every_robot()
    lab.plugin._jog_held.clear()
    lab.plugin._update_jogs(lab.ctx, lab.ctx.t)  # released: the jog is held
    lab.arm.state.is_executing = False
    lab.later(IDLE_AFTER + 0.5)
    lab.plugin._leases.release_idle(lab.ctx, partial(lab.plugin._busy, lab.ctx.t))
    assert lab.robot.lease_slot.lease is None
    lab.lease_every_robot()


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


def test_base_exp_does_not_start_while_the_executor_leases_every_robot():
    """A base experiment on Alice is not started while the executor leases her, and says by whom."""
    from husky_assembly_teleop.plugins.base_exp.plugin import BaseExperimentPlugin
    lab = Lab()
    plugin = BaseExperimentPlugin()
    plugin.serial = SERIAL
    lab.lease_every_robot()
    assert plugin._start_problem(lab.ctx) == f"{SERIAL} is leased by {EXEC}"
