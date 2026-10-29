"""robot_control's moves on an arm with its real checks: joint jogs, streamed Cartesian moves, Hold and the D-pad."""

from __future__ import annotations

import asyncio
from contextlib import contextmanager
from functools import partial
from types import SimpleNamespace

import numpy as np
import pytest
from robot_fakes import TCP_ORIENTATION, TCP_POSITION, FreeRobots, fake_arm, lease_context, wait_ticks_until
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.plugin_api.concurrency import WaitTimeout
from husky_assembly_teleop.shared.teleop.dpad import BLOCKED_COLOR, HOLD_TIMEOUT, UNGUARDED_COLOR, Hold, color_dpad
from husky_assembly_teleop.plugins.robot_control.arm_panel import jog_clearance_note
from husky_assembly_teleop.plugins.robot_control.markers import place_markers, target_from_world
from husky_assembly_teleop.plugins.robot_control.plugin import (BASE, STANDING_EVERY, START_RETRY,
                                                                RobotControlPlugin, _BaseWidgets)
from husky_assembly_teleop.plugins.robot_control.status import refusal_chip
from husky_assembly_teleop.plugins.robot_control.teleop_lease import IDLE_AFTER
from husky_assembly_teleop.robot_interface.arm import (CARTESIAN_COMPLIANCE_CONTROLLER, MOVING_WITHIN,
                                                       SCALED_JOINT_TRAJECTORY_CONTROLLER, UR_JOINT_LIMITS)
from husky_assembly_teleop.robot_interface.base import PLATFORM_VELOCITY_CONTROLLER
from husky_assembly_teleop.robot_interface.robot_lease import Lease, take_lease
from husky_assembly_teleop.world.clearance import (ALL_ARMS, ARM_STOP_DISTANCE, STOP_DISTANCE, Clearance,
                                                   ClearanceRefused)
from husky_assembly_teleop.ui.style import FAIL
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Scene

pytestmark = pytest.mark.safety

#: Seconds per tick of the fake context.
TICK = 0.05


class Context:
    """A plugin context whose clock is the arm's node clock and advances one TICK per `next_tick`.

    `follow` runs every tick, to play the arm moving.
    """

    def __init__(self, arm, follow=lambda: None):
        """Share `arm`'s clock; every robot's lease is free."""
        self.arm, self.follow, self.lines = arm, follow, []
        self.world = SimpleNamespace(robots=FreeRobots())
        self.scene = SimpleNamespace(snapshot=Scene())

    def take_lease(self, serials, label):
        """Lease robots of the fake world, as the monitor does."""
        return take_lease([self.world.robots[serial].lease_slot for serial in serials], f"robot_control: {label}")

    def now(self) -> float:
        """The arm's ROS time."""
        return self.arm._node.now

    async def next_tick(self) -> None:
        """Advance time by one tick, and let the arm move."""
        self.arm._node.now += TICK
        self.follow()
        await asyncio.sleep(0)

    async def wait_until(self, predicate, timeout_s=None, description="condition") -> None:
        """Tick until `predicate` holds, or raise WaitTimeout after `timeout_s`."""
        end = self.now() + timeout_s
        while not predicate():
            if self.now() > end:
                raise WaitTimeout(f"timed out waiting for {description}")
            await self.next_tick()

    def spawn(self, label, work):
        """Start `work` as a task."""
        return asyncio.get_running_loop().create_task(work, name=label)

    def log_info(self, message: str) -> None:
        """Keep it."""
        self.lines.append(("info", message))

    def log_warn(self, message: str) -> None:
        """Keep it."""
        self.lines.append(("warn", message))

    def text(self) -> str:
        """Everything logged."""
        return "\n".join(message for _level, message in self.lines)


def tracking(arm):
    """An arm that reaches each Cartesian target by the next tick, measured at once."""
    def follow():
        if arm.state.sent_target_position is not None:
            arm.state.tcp_position = arm.state.sent_target_position
            arm.state.tcp_orientation = arm.state.sent_target_orientation
        arm.state.tcp_update_time = arm.state.last_update_time = arm.state.controllers.last_update_time = arm._node.now
        arm.state.status_update_time = arm._node.now
    return follow


def test_a_streamed_cartesian_move_passes_every_check_and_arrives():
    """10 cm and 10 degrees: every target is within the step limits of the one before, and the last is the goal."""
    arm = fake_arm(CARTESIAN_COMPLIANCE_CONTROLLER)
    ctx = Context(arm, tracking(arm))
    goal = TCP_POSITION + [0.1, 0.0, 0.0]
    turned = np.array([0.0, 0.0, np.sin(np.radians(5.0)), np.cos(np.radians(5.0))])
    asyncio.run(RobotControlPlugin()._stream_cartesian_move(ctx, "0804", arm, goal, turned))
    assert "cartesian move done" in ctx.text() and not arm._node.log.text("error")
    assert len(arm._target_frame.sent) > 10
    np.testing.assert_allclose(arm.state.sent_target_position, goal, atol=1e-9)


def test_a_streamed_move_the_arm_refuses_stops_and_holds():
    """When a target is refused mid-move (here the controller changed), the move ends and the arm is held."""
    arm = fake_arm(CARTESIAN_COMPLIANCE_CONTROLLER)
    follow, ticks = tracking(arm), [0]

    def then_switch():
        follow()
        ticks[0] += 1
        if ticks[0] == 3:
            arm.state.controllers.active = SCALED_JOINT_TRAJECTORY_CONTROLLER

    ctx = Context(arm, then_switch)
    asyncio.run(RobotControlPlugin()._stream_cartesian_move(ctx, "0804", arm, TCP_POSITION + [0.1, 0, 0],
                                                            TCP_ORIENTATION))
    assert "stopped early" in ctx.text() and len(arm._target_frame.sent) == 3
    assert len(arm._trajectory.sent) == 1  # the hold, under the joint controller now running


def test_test_mode_checks_one_target_and_streams_nothing():
    """cartesian_test_mode: one target is checked at the TCP, nothing is published, and the log says so."""
    arm = fake_arm(CARTESIAN_COMPLIANCE_CONTROLLER, cartesian_test_mode=True)
    ctx = Context(arm)
    asyncio.run(RobotControlPlugin()._stream_cartesian_move(ctx, "0804", arm, TCP_POSITION + [0.1, 0, 0],
                                                            TCP_ORIENTATION))
    assert not arm._target_frame.sent and "TEST MODE" in ctx.text()


def test_a_cancelled_move_holds_the_arm():
    """Stop all, the plugin stopping or shutdown cancel the task: the targets must not run on."""
    arm = fake_arm(CARTESIAN_COMPLIANCE_CONTROLLER)
    ctx = Context(arm, tracking(arm))
    plugin = RobotControlPlugin()

    async def run():
        move = plugin._stream_cartesian_move(ctx, "0804", arm, TCP_POSITION + [0.05, 0.0, 0.0], TCP_ORIENTATION)
        task = asyncio.create_task(move)
        for _ in range(3):
            await asyncio.sleep(0)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(run())
    assert "hold sent" in arm._node.log.text("info")


def test_one_move_per_arm_at_a_time_and_hold_cancels_it():
    """A second Start while a move runs is ignored; Hold cancels the running move and holds the arm."""
    arm, plugin = fake_arm(), RobotControlPlugin()
    ctx = plugin._ctx = Context(arm)  # * Hold reads the robot's lock through the context kept from setup

    async def run():
        never = asyncio.Event()

        async def move(_ctx, _serial, _arm):
            await never.wait()

        plugin._start_move(ctx, "0804", arm, "move", move)
        plugin._start_move(ctx, "0804", arm, "move", move)
        task = plugin._trajectory_tasks[("0804", "ur_arm")]
        plugin._hold_arm("0804", arm)
        await asyncio.sleep(0)
        return task

    task = asyncio.run(run())
    assert "already running" in ctx.text() and task.cancelled() and len(arm._trajectory.sent) == 1


def test_hold_also_removes_an_applied_force():
    """After Hold the compliance controller must no longer push with the force set before."""
    arm, plugin = fake_arm(CARTESIAN_COMPLIANCE_CONTROLLER), RobotControlPlugin()
    plugin._ctx = Context(arm)  # * Hold reads the robot's lock through the context kept from setup
    plugin._apply_force(plugin._ctx, ("0804", "ur_arm"), arm, np.array([0.0, 0.0, 10.0]))
    plugin._hold_arm("0804", arm)
    assert arm._target_wrench.sent[-1].wrench.force.z == 0.0 and not plugin._applied_force


def _jogged_arm(joints=None):
    """A plugin with one arm under the joint controller, at full jog speed, and a context sharing its clock."""
    arm, plugin = fake_arm(joints=joints), RobotControlPlugin()
    plugin._arms = [SimpleNamespace(serial="0804", arm=arm, jog_speed=SimpleNamespace(value=1.0), jog_clearance=None,
                                    free_drive_keep=None)]
    return arm, plugin, Context(arm)


def _jog_ticks(arm, plugin, ctx, ticks: int, press=None) -> None:
    """Run `ticks` ticks of jog updates, the arm measured fresh each tick, with `press` (joint, direction) held."""
    for _ in range(ticks):
        arm._node.now += TICK
        arm.state.last_update_time = arm._node.now
        if press is not None:
            plugin._on_jog_hold(ctx, ("0804", "ur_arm"), *press)
        plugin._update_jogs(ctx, ctx.now())


def test_a_held_jog_button_sends_one_jog_and_its_release_holds():
    """Holding sends one jog, not one per report; once the reports stop, one hold goes out."""
    arm, plugin, ctx = _jogged_arm()
    _jog_ticks(arm, plugin, ctx, 5, press=(0, 1.0))
    assert len(arm._trajectory.sent) == 1 and len(arm._trajectory.sent[0].points) > 2
    _jog_ticks(arm, plugin, ctx, int(2 * HOLD_TIMEOUT / TICK))
    assert len(arm._trajectory.sent) == 2 and "hold sent" in arm._node.log.text("info")


def test_another_jog_button_holds_the_arm_and_jogs_once_it_is_still():
    """Switching buttons holds the arm; the new jog waits, quietly, until the arm could take it."""
    arm, plugin, ctx = _jogged_arm()
    _jog_ticks(arm, plugin, ctx, 2, press=(0, 1.0))
    _jog_ticks(arm, plugin, ctx, int(MOVING_WITHIN / TICK) - 1, press=(1, -1.0))
    assert [len(trajectory.points) == 1 for trajectory in arm._trajectory.sent] == [False, True]
    _jog_ticks(arm, plugin, ctx, 3, press=(1, -1.0))
    assert [len(trajectory.points) == 1 for trajectory in arm._trajectory.sent] == [False, True, False]
    assert not arm._node.log.text("error")


@pytest.mark.parametrize("stop", ["soft stop", "controller off"])
def test_a_jog_is_held_when_the_robot_stops_or_the_controller_changes(stop):
    """A soft stop or another controller ends the press at once, with a hold, and raises nothing."""
    arm, plugin, ctx = _jogged_arm()
    _jog_ticks(arm, plugin, ctx, 2, press=(0, 1.0))
    if stop == "soft stop":
        arm._latch.stopped = True
    else:
        arm.state.controllers.active = None
    _jog_ticks(arm, plugin, ctx, 3, press=(0, 1.0))
    assert len(arm._trajectory.sent) == 2 and "hold sent" in arm._node.log.text("info")


class JogContext(Context):
    """The jog context with clearances kept for the test, or refused with `refuse` as the reason."""

    wait_until = staticmethod(wait_ticks_until)

    def __init__(self, arm, refuse: str = ""):
        """As `Context`; no clearance kept yet."""
        super().__init__(arm)
        self.clearances, self.refuse = [], refuse

    @contextmanager
    def clearance(self, serials, margin=STOP_DISTANCE, unknown_arms=(), unknown_robots=(), refuse_first=False):
        """Keep the clearance, or refuse it; the test checks or trips it."""
        if self.refuse:
            raise ClearanceRefused(self.refuse)
        self.clearances.append(Clearance(tuple(serials), margin, unknown_arms=unknown_arms,
                                         refuse_first=refuse_first))
        yield self.clearances[-1]


def test_a_jog_waits_for_its_clearance_which_keeps_one_centimetre():
    """With Clearance ticked, the jog goes out only once a 1 cm clearance found the robot clear; unticking ends it."""
    arm, plugin, _ctx = _jogged_arm()
    plugin._arms[0].jog_clearance = SimpleNamespace(value=True)
    ctx = JogContext(arm)

    async def scenario() -> None:
        _jog_ticks(arm, plugin, ctx, 1, press=(0, 1.0))
        assert not arm._trajectory.sent, "the clearance comes first"
        await asyncio.sleep(0)
        _jog_ticks(arm, plugin, ctx, 1, press=(0, 1.0))
        assert not arm._trajectory.sent and ctx.clearances[0].refuse_first, "not before its first clear check"
        ctx.clearances[0].checked = True
        _jog_ticks(arm, plugin, ctx, 1, press=(0, 1.0))
        assert len(arm._trajectory.sent) == 1
        clearance, = ctx.clearances
        assert (clearance.serials, clearance.margin, clearance.unknown_arms) == (("0804",), ARM_STOP_DISTANCE, [])
        plugin._arms[0].jog_clearance.value = False
        await plugin._jog_guards[("0804", "ur_arm")]

    asyncio.run(scenario())
    assert ARM_STOP_DISTANCE == 0.01


def test_a_jog_that_starts_too_close_is_refused_until_the_next_press_without_a_soft_stop():
    """Too close at the first check: nothing sent, no soft stop, a red "starts in collision" chip."""
    arm, plugin, _ctx = _jogged_arm()
    plugin._arms[0].jog_clearance = SimpleNamespace(value=True)
    ctx = JogContext(arm)

    async def scenario() -> None:
        _jog_ticks(arm, plugin, ctx, 1, press=(0, 1.0))
        await asyncio.sleep(0)
        clearance, = ctx.clearances
        clearance.problem, clearance.refused = "robot 0804 came within 1 cm of the rack", True  # as the monitor's
        guard = plugin._jog_guards[("0804", "ur_arm")]
        guard.cancel()
        await asyncio.gather(guard, return_exceptions=True)
        _jog_ticks(arm, plugin, ctx, 3, press=(0, 1.0))

    asyncio.run(scenario())
    assert not arm._trajectory.sent and len(ctx.clearances) == 1, "the press is answered: not retried"
    assert "0804" not in plugin._tripped
    chip_html = plugin._start_chip(("0804", "ur_arm"), ctx.now(), "jog")
    assert "starts in collision" in chip_html and "within 1 cm of the rack" in chip_html and FAIL in chip_html


def test_a_jog_whose_clearance_cannot_check_is_refused_until_the_next_press():
    """Base not tracked: nothing is sent, the chip says to untick Clearance, and the held press is not retried."""
    arm, plugin, _ctx = _jogged_arm()
    plugin._arms[0].jog_clearance = SimpleNamespace(value=True)
    ctx = JogContext(arm, refuse="robot 0804 cannot be checked for clearance: its base is not tracked")

    async def scenario() -> None:
        _jog_ticks(arm, plugin, ctx, 1, press=(0, 1.0))
        await asyncio.sleep(0)
        _jog_ticks(arm, plugin, ctx, 3, press=(0, 1.0))

    asyncio.run(scenario())
    refusal = plugin._refusals[("0804", "ur_arm")]
    assert not arm._trajectory.sent and refusal.label == "cannot check" and "untick Clearance" in refusal.reason
    assert len(plugin._jog_guards) == 1 and plugin._jog_guards[("0804", "ur_arm")].done()


def test_a_jog_leases_the_robot_until_it_is_idle():
    """The jog goes out under robot_control's lease; once released and still for IDLE_AFTER, the lease is given back."""
    arm, plugin, ctx = _jogged_arm()
    _jog_ticks(arm, plugin, ctx, 2, press=(0, 1.0))
    assert plugin._leases.held("0804") and len(arm._trajectory.sent) == 1
    arm.state.is_executing = False
    for _ in range(int((2 * HOLD_TIMEOUT + IDLE_AFTER) / TICK) + 2):
        _jog_ticks(arm, plugin, ctx, 1)
        arm.state.is_executing = False
        plugin._leases.release_idle(ctx, partial(plugin._busy, ctx.now()))
    assert not plugin._leases.held("0804")


def test_a_jog_on_a_robot_another_plugin_holds_is_dropped_and_says_why():
    """Leased to the arm planner: nothing is sent, nothing raises, and a "lease refused" chip names the holder."""
    arm, plugin, ctx = _jogged_arm()
    Lease("arm_planner: commit plan 2", (ctx.world.robots["0804"].lease_slot,))
    _jog_ticks(arm, plugin, ctx, 5, press=(0, 1.0))
    assert not arm._trajectory.sent and not plugin._leases.held("0804")
    refused = plugin._refusals[("0804", "ur_arm")]
    assert refused.label == "lease refused" and "leased by arm_planner: commit plan 2" in refused.reason
    chip = refusal_chip([refused], ctx.now())
    assert ">lease refused<" in chip and "arm_planner: commit plan 2" in chip
    assert ctx.text().count("leased by arm_planner") == 1, "logged once, not every tick"


def test_a_jog_the_arm_refuses_shows_the_arms_reason():
    """The arm's own checks refuse the jog: its `last_refusal` carries the reason for the chip."""
    arm, plugin, ctx = _jogged_arm()
    arm.state.operational, arm.state.problem = False, "protective stop"
    _jog_ticks(arm, plugin, ctx, 2, press=(0, 1.0))
    assert not arm._trajectory.sent
    assert arm.last_refusal.label == "move refused" and "protective stop" in arm.last_refusal.reason
    assert ">move refused<" in refusal_chip([plugin._refusals.get(("0804", "ur_arm")), arm.last_refusal], ctx.now())


def test_a_tool_button_on_a_leased_robot_says_why():
    """A one-shot command refused for the lease returns None and leaves a chip instead of raising."""
    arm, plugin, ctx = _jogged_arm()
    Lease("movement_exec", (ctx.world.robots["0804"].lease_slot,))
    assert plugin._leased(ctx, ("0804", "ur_arm"), lambda: arm.send_target_wrench([0.0, 0.0, 1.0])) is None
    assert plugin._refusals[("0804", "ur_arm")].label == "lease refused"


def test_a_jog_at_the_joint_limit_is_refused_once_per_press():
    """At the limit nothing is sent, and a held button warns once, not every tick."""
    arm, plugin, ctx = _jogged_arm(joints=[UR_JOINT_LIMITS[0], 0.0, 0.0, 0.0, 0.0, 0.0])
    _jog_ticks(arm, plugin, ctx, 5, press=(0, 1.0))
    assert not arm._trajectory.sent and ctx.text().count("at its limit") == 1


def test_the_gizmo_and_the_sliders_name_the_same_target():
    """A target in the husky's base_link, drawn in the world, comes back the same from a gizmo at that pose."""
    arm = fake_arm(CARTESIAN_COMPLIANCE_CONTROLLER)
    arm.base_in_husky = Pose((0.2, -0.1, 0.4), tuple(Rotation.from_euler("xyz", [0.3, -0.2, 1.0]).as_quat()))
    inertia = Pose((1.0, 2.0, 0.5), tuple(Rotation.from_euler("z", 0.7).as_quat()))
    ctx = SimpleNamespace(config=SimpleNamespace(robots=[SimpleNamespace(serial="0804")]),
                          kinematics=SimpleNamespace(link_pose=lambda serial, link: inertia))
    position, orientation = np.array([0.5, 0.1, 0.6]), Rotation.from_euler("xyz", [10, 20, 30], degrees=True).as_quat()
    world = place_markers(ctx, "0804", arm, arm.from_husky(position, orientation), np.zeros(3), None)["target_world"]
    back = target_from_world(ctx, "0804", arm, Pose(tuple(world[0]), tuple(world[1])))
    np.testing.assert_allclose(back[0], position, atol=1e-9)
    assert (Rotation.from_quat(back[1]).inv() * Rotation.from_quat(orientation)).magnitude() < 1e-9


def _driven_base():
    """A plugin with one base whose velocity controller runs, recording its twists."""
    sent, controller = [], [PLATFORM_VELOCITY_CONTROLLER]
    base = SimpleNamespace(controllers=SimpleNamespace(is_active=lambda name: name == controller[0]),
                           send_twist=lambda linear, angular: sent.append((linear, angular)), soft_stopped=False)
    plugin = RobotControlPlugin()
    plugin._bases = [_BaseWidgets(serial="0804", base=base, panel=None, status=None,
                                  speed=SimpleNamespace(value=1.0), phone_speed=SimpleNamespace(value=1.0))]
    return plugin, sent, controller


def test_a_held_base_stops_when_its_controller_goes_off():
    """The hold is dropped as soon as the velocity controller stops, and one stop goes out."""
    plugin, sent, controller = _driven_base()
    ctx = lease_context(now=lambda: 0.0, log_warn=lambda _text: None)
    plugin._on_hold("0804", Hold(1.0, 0.0, plugin._bases[0].speed, ctx.now()))
    plugin.update(ctx)
    controller[0] = None
    plugin.update(ctx)
    plugin.update(ctx)
    assert sent[1:] == [(0.0, 0.0)]


def test_teardown_stops_a_base_that_is_being_driven():
    """Shutting down mid-drive sends a stop."""
    plugin, sent, _controller = _driven_base()
    ctx = lease_context(now=lambda: 0.0, log_warn=lambda _text: None)
    plugin._on_hold("0804", Hold(0.0, 1.0, plugin._bases[0].speed, ctx.now()))
    plugin.update(ctx)
    plugin.teardown(ctx)
    assert sent[-1] == (0.0, 0.0)


class BaseContext:
    """A plugin context for driving a base: real tasks and leases, and clearances kept for the test to trip."""

    def __init__(self):
        """At ROS time 0; every robot's lease is free."""
        self.world, self.clearances, self.lines = SimpleNamespace(robots=FreeRobots()), [], []
        self.scene = SimpleNamespace(snapshot=Scene())

    wait_until = staticmethod(wait_ticks_until)

    def now(self) -> float:
        """ROS time, seconds."""
        return 0.0

    def take_lease(self, serials, label):
        """Lease robots of the fake world, as the monitor does."""
        return take_lease([self.world.robots[serial].lease_slot for serial in serials], f"robot_control: {label}")

    def spawn(self, label, work):
        """Start `work` as a task."""
        return asyncio.get_running_loop().create_task(work, name=label)

    @contextmanager
    def clearance(self, serials, margin=STOP_DISTANCE, unknown_arms=(), unknown_robots=(), refuse_first=False):
        """Keep the clearance; the test checks or trips it."""
        self.clearances.append(Clearance(tuple(serials), margin, unknown_arms=unknown_arms,
                                         refuse_first=refuse_first))
        yield self.clearances[-1]

    def log_warn(self, message: str) -> None:
        """Keep it."""
        self.lines.append(message)


def test_driving_with_clearance_keeps_one_on_the_base_until_it_is_unticked():
    """A hold drives once its clearance found the base clear, every tick under the same one; unticking ends it."""
    plugin, sent, _controller = _driven_base()
    plugin._bases[0].clearance = SimpleNamespace(value=True)
    ctx = BaseContext()

    async def drive() -> bool:
        plugin._on_hold("0804", Hold(1.0, 0.0, plugin._bases[0].speed, ctx.now()))
        plugin.update(ctx)
        await asyncio.sleep(0)
        plugin.update(ctx)
        assert not sent and ctx.clearances[0].refuse_first  # * nothing goes out before the first clear check
        ctx.clearances[0].checked = True
        plugin.update(ctx)
        await asyncio.sleep(0)
        plugin.update(ctx)
        await asyncio.sleep(0)
        guard = plugin._base_guards["0804"]
        guarding = not guard.done()
        plugin._bases[0].clearance.value = False
        await guard
        plugin._bases[0].clearance.value = True
        plugin.update(ctx)  # * a new guard: the last one's checked clearance never lets a twist out
        plugin.update(ctx)
        assert sent[2:] == [(0.0, 0.0)]  # only the stop as the drive pauses
        return guarding

    assert asyncio.run(drive()) and len(sent) == 3
    assert [clearance.serials for clearance in ctx.clearances][0] == ("0804",)


def test_a_clearance_trip_says_on_the_base_why_driving_stopped():
    """Tripped, the base is soft stopped and a held button is dropped with what the base came too close to."""
    plugin, sent, _controller = _driven_base()
    plugin._bases[0].clearance = SimpleNamespace(value=True)
    base, ctx = plugin._bases[0].base, BaseContext()

    async def drive() -> None:
        plugin._on_hold("0804", Hold(1.0, 0.0, plugin._bases[0].speed, ctx.now()))
        plugin.update(ctx)
        await asyncio.sleep(0)
        assert ctx.clearances[0].unknown_arms == ALL_ARMS  # * driving by hand: an unknown arm counts as stowed
        ctx.clearances[0].checked = True
        plugin.update(ctx)
        ctx.clearances[0].problem = "robot 0804 came within 10 cm of wall"
        base.soft_stopped = True  # as the monitor's trip does
        guard = plugin._base_guards["0804"]
        guard.cancel()
        await asyncio.gather(guard, return_exceptions=True)
        plugin._on_hold("0804", Hold(1.0, 0.0, plugin._bases[0].speed, ctx.now()))
        plugin.update(ctx)

    asyncio.run(drive())
    refusal = plugin._refusals[("0804", BASE)]
    assert refusal.label == "too close" and "came within 10 cm of wall: soft stopped" in refusal.reason
    assert "0804" not in plugin._held and sent[-1] == (0.0, 0.0)


def test_a_drive_that_starts_too_close_is_refused_with_a_red_chip_and_tried_again_later():
    """Too close at the first check: nothing sent, no soft stop, a "starts in collision" chip; retried later."""
    plugin, sent, _controller = _driven_base()
    plugin._bases[0].clearance = SimpleNamespace(value=True)
    ctx, now = BaseContext(), [0.0]
    ctx.now = lambda: now[0]
    ctx.config = SimpleNamespace(robots=[SimpleNamespace(serial="0804", arms=())])
    ctx.scene.snapshot = Scene(robots={robot_id("0804"): RobotObject(
        robot_id("0804"), SimpleNamespace(movable_joints=()), Pose(), {})})

    async def drive() -> None:
        plugin._on_hold("0804", Hold(1.0, 0.0, plugin._bases[0].speed, ctx.now()))
        plugin.update(ctx)
        await asyncio.sleep(0)
        clearance = ctx.clearances[0]
        clearance.problem, clearance.refused = "robot 0804 came within 10 cm of wall", True  # as the monitor's refusal
        guard = plugin._base_guards["0804"]
        guard.cancel()
        await asyncio.gather(guard, return_exceptions=True)
        plugin.update(ctx)
        await asyncio.sleep(0)
        assert len(ctx.clearances) == 1  # * not tried again at once
        now[0] = START_RETRY
        plugin._on_hold("0804", Hold(1.0, 0.0, plugin._bases[0].speed, ctx.now()))
        plugin.update(ctx)
        await asyncio.sleep(0)
        assert len(ctx.clearances) == 2

    asyncio.run(drive())
    assert not sent and not plugin._bases[0].base.soft_stopped and "0804" not in plugin._tripped
    chips = plugin._base_warnings(ctx, "0804", now[0])
    assert "starts in collision" in chips and FAIL in chips and "Untick Clearance" in chips


class StandingChecker:
    """Stands in for the plugin's ClearanceChecker: keeps what it is asked, answers `found` once."""

    def __init__(self):
        """Nothing asked or found."""
        self.asked, self.found = [], None

    def check(self, _snapshot, asked) -> bool:
        """Keep what is asked."""
        self.asked.append(asked)
        return True

    def finished(self):
        """Hand out `found` once."""
        found, self.found = self.found, None
        return found


def test_a_base_standing_too_close_shows_the_chip_and_a_gray_dpad_before_any_press():
    """Checked every STANDING_EVERY: too close shows red and grays the D-pad; Clearance off makes it orange."""
    plugin, _sent, _controller = _driven_base()
    widgets = plugin._bases[0]
    widgets.clearance, widgets.dpad = SimpleNamespace(value=True), (SimpleNamespace(color=None),)
    plugin._standing, now = StandingChecker(), [0.0]
    ctx = BaseContext()
    ctx.now = lambda: now[0]
    ctx.config = SimpleNamespace(robots=[SimpleNamespace(serial="0804", arms=())])
    ctx.scene.snapshot = Scene(robots={robot_id("0804"): RobotObject(
        robot_id("0804"), SimpleNamespace(movable_joints=()), Pose(), {})})
    plugin._check_standing(ctx, now[0])
    plugin._check_standing(ctx, now[0] + STANDING_EVERY / 2)
    assert len(plugin._standing.asked) == 1 and plugin._standing.asked[0][0][:2] == ("0804", robot_id("0804"))
    plugin._standing.found = [("0804", "came within 10 cm of wall", SimpleNamespace())]
    plugin._check_standing(ctx, now[0] + STANDING_EVERY)
    chips = plugin._base_warnings(ctx, "0804", now[0])
    assert "starts in collision" in chips and "came within 10 cm of wall" in chips
    assert plugin._starts_in_collision("0804", now[0])
    plugin._standing.found = [("0804", "cannot be checked: base not measured", None)]
    plugin._check_standing(ctx, now[0] + 2 * STANDING_EVERY)
    assert not plugin._starts_in_collision("0804", now[0]), "a robot it cannot check shows nothing"
    buttons = [SimpleNamespace(color=None)]
    color_dpad(buttons, BLOCKED_COLOR)
    assert buttons[0].color == "gray" and UNGUARDED_COLOR == "orange"


def test_the_jog_buttons_and_their_note_follow_clearance_as_the_dpad_does():
    """Ticked: theme colour and a gray note; too close to start: gray; unticked: orange and an orange warning."""
    plugin, key = RobotControlPlugin(), ("0804", "ur_arm")
    buttons = [SimpleNamespace(color=None)]
    plugin._color_buttons(buttons, True, key, 0.0)
    assert buttons[0].color is None
    plugin._too_close[key] = "came within 1 cm of the rack"
    plugin._color_buttons(buttons, True, key, 0.0)
    assert buttons[0].color == BLOCKED_COLOR
    plugin._color_buttons(buttons, False, key, 0.0)
    assert buttons[0].color == UNGUARDED_COLOR
    ticked, unticked = jog_clearance_note(True, ARM_STOP_DISTANCE), jog_clearance_note(False, ARM_STOP_DISTANCE)
    assert "Clearance keeps 1 cm" in ticked and "Clearance off" in unticked
    assert "white-space:nowrap" in ticked and "white-space:nowrap" in unticked, "one line either way: nothing jumps"
