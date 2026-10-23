"""A plugin context for tests: ROS time ticking at 10 Hz, tasks on the running loop, other plugins, logs and a GUI.

Run the plugin's tasks inside `asyncio.run`; `step(t)` changes the world each tick (e.g. the snapshot). Widgets are
`FakeWidget`s: call a widget's `callback` to click it.

Robots: `fake_arm` and `fake_robot` are the real `ArmInterface` and `HuskyRobotInterface` without ROS, so every check
they run is the real one; their publishers keep what is sent (`Published`), and an `ArmDriver` moves an arm along what
it was sent, feeding its joint states and the safety sync's status as the robot would. `fake_robotiq` and
`fake_scaffolding` are the real tool interfaces, answered by a `RobotiqDriver` and a `ScaffoldingDriver`.

Leases: a fake robot's parts share its soft-stop latch and its robot lease slot, free at first, as on a real robot.
Code that sends to them takes the lease as the monitor's plugins do (`FakeContext.take_lease`, the real one), or a
test hands it the lease every test holds (`lease_to_test`).
"""

from __future__ import annotations

import asyncio
from contextlib import contextmanager, nullcontext
from functools import partial
from types import SimpleNamespace
from typing import Any, Callable, Iterator, Sequence

import numpy as np
from action_msgs.msg import GoalStatus
from robot_fakes import TEST_LEASE, Controllers
from sensor_msgs.msg import JointState

from husky_assembly_teleop.config import ArmConfig
from husky_assembly_teleop.plugin_api.concurrency import WaitTimeout
from husky_assembly_teleop.plugin_api.context import PluginContext
from husky_assembly_teleop.robot_interface.arm import SCALED_JOINT_TRAJECTORY_CONTROLLER, ArmInterface, ArmState
from husky_assembly_teleop.robot_interface.base import BaseState
from husky_assembly_teleop.robot_interface.controller_manager import ControllerManagerState
from husky_assembly_teleop.robot_interface.end_effectors import (RobotiqGripper, RobotiqState, ScaffoldingV3,
                                                                 ScaffoldingV3State)
from husky_assembly_teleop.robot_interface.event import Event
from husky_assembly_teleop.robot_interface.robot import HuskyRobotInterface
from husky_assembly_teleop.robot_interface.robot_lease import LeaseSlot, leased
from husky_assembly_teleop.robot_interface.soft_stop import SoftStopLatch
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.scene import Scene
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES


class FakeWidget(SimpleNamespace):
    """A viser widget: what was given when it was added, what the plugin wrote into it, and its callback."""

    def on_click(self, callback: Callable, **_options: Any) -> None:
        """Keep the callback."""
        self.callback = callback

    on_update = on_hold = on_click

    def __enter__(self) -> FakeWidget:
        """A folder or tab: widgets added inside land in the same list."""
        return self

    def __exit__(self, *_exc: Any) -> None:
        """Leave the folder."""


class FakeGui:
    """viser's GUI api: every `add_*` call makes a FakeWidget, kept in order in `widgets`."""

    def __init__(self):
        """No widgets yet."""
        self.widgets: list[FakeWidget] = []

    def __getattr__(self, name: str) -> Callable[..., FakeWidget]:
        """`add_<kind>(label, value, **options)`: a widget of that kind."""
        if not name.startswith("add_"):
            raise AttributeError(name)

        def add(label: str = "", *values: Any, **options: Any) -> FakeWidget:
            widget = FakeWidget(kind=name[4:], label=label, value=values[0] if values else options.get("initial_value"),
                                content="", color=options.get("color"), options=options)
            self.widgets.append(widget)
            return widget
        return add

    def of_kind(self, kind: str) -> list[FakeWidget]:
        """The widgets of one kind, e.g. "button", in order."""
        return [widget for widget in self.widgets if widget.kind == kind]


class FakeContext:
    """The part of a `PluginContext` plugins use, without a monitor; a test reads `logs` and moves `t`."""

    #: Robot leases, as the monitor gives them: the real `PluginContext` code.
    take_lease = PluginContext.take_lease
    lease_robots = PluginContext.lease_robots
    _lease_slots = PluginContext._lease_slots

    def __init__(self, robots: tuple = (), plugins: dict | None = None, snapshot: Scene | None = None,
                 step: Callable[[float], None] = lambda _t: None, drive_root=None, name: str = "a plugin"):
        """Start at ROS time 10 s.

        Args:
            robots: The configured robots, `config.robots`.
            plugins: Name -> plugin, what `require` returns.
            snapshot: The scene snapshot; tests may replace `scene.snapshot` at any time.
            step: Called with the new time at every tick.
            drive_root: `config.drive_root`.
            name: The plugin's name, which its leases are taken under.
        """
        self.name = name
        #: The leases taken with `take_lease`; the tasks spawned to survive a soft stop.
        self._leases: set = set()
        self.survivors: set[asyncio.Task] = set()
        self.t, self.step = 10.0, step
        #: Whether the plugin was stopped or torn down (`PluginContext.closed`).
        self.closed = False
        self.config = SimpleNamespace(robots=tuple(robots), drive_root=drive_root)
        self.plugins = dict(plugins or {})
        self.scene = SimpleNamespace(snapshot=snapshot if snapshot is not None else Scene())
        self.world = SimpleNamespace(robots={})
        self.gui = FakeGui()
        self.view = SimpleNamespace(ui=lambda: nullcontext(self.gui), phone=lambda *_tab: nullcontext(self.gui),
                                    scene_root="/plugin")
        #: (level, message) per log line.
        self.logs: list[tuple[str, str]] = []

    def now(self) -> float:
        """ROS time, seconds."""
        return self.t

    def require(self, name: str) -> Any:
        """Another plugin, by name."""
        return self.plugins[name]

    def defer(self, label: str, run: Callable[[], Any], operator_only: bool = False) -> Callable[..., Any]:
        """A widget callback that runs `run` at once (no viser thread here); every call counts as the operator's."""
        return lambda *_event: run()

    def defer_value(self, label: str, run: Callable[[Any], Any], operator_only: bool = False) -> Callable[[Any], Any]:
        """A widget callback handed the new value, run at once; every call counts as the operator's."""
        return run

    def submit(self, label: str, run: Callable[[], Any]) -> None:
        """A queued intent, run at once (no tick here)."""
        run()

    def spawn(self, label: str, work, survives_soft_stop: bool = False) -> asyncio.Task:
        """Start a task on the running loop; one a soft stop leaves alone is kept in `survivors`."""
        task = asyncio.get_running_loop().create_task(work, name=label)
        if survives_soft_stop:
            self.survivors.add(task)
        return task

    async def run_in_thread(self, function: Callable, *args: Any, **kwargs: Any) -> Any:
        """Run on the loop's default thread pool."""
        return await asyncio.get_running_loop().run_in_executor(None, partial(function, *args, **kwargs))

    async def next_tick(self) -> None:
        """Advance 0.1 s, step the world, let other tasks run."""
        self.t += 0.1
        self.step(self.t)
        await asyncio.sleep(0)

    async def wait_until(self, predicate: Callable[[], bool], timeout_s: float | None = None,
                         description: str = "condition") -> None:
        """Tick until `predicate` holds, as `PluginContext.wait_until`."""
        deadline = None if timeout_s is None else self.t + timeout_s
        while not predicate():
            if deadline is not None and self.t >= deadline:
                raise WaitTimeout(f"timed out after {timeout_s}s waiting for {description}")
            await self.next_tick()

    async def sleep(self, seconds: float) -> None:
        """Tick for `seconds`."""
        until = self.t + seconds
        await self.wait_until(lambda: self.t >= until)

    def log_info(self, message: str) -> None:
        """Keep the line."""
        self.logs.append(("info", message))

    def log_warn(self, message: str) -> None:
        """Keep the line."""
        self.logs.append(("warn", message))

    def log_error(self, message: str) -> None:
        """Keep the line."""
        self.logs.append(("error", message))

    def lines(self, level: str) -> list[str]:
        """The logged lines of one level."""
        return [message for kind, message in self.logs if kind == level]


# --- --- --- --- --- ROBOTS --- --- --- --- ---

class Published:
    """A publisher that keeps every message."""

    def __init__(self):
        """Nothing published yet."""
        self.messages: list = []

    def publish(self, message) -> None:
        """Keep the message."""
        self.messages.append(message)


def fake_node(clock: Callable[[], float], logs: list[tuple[str, str]]) -> SimpleNamespace:
    """The part of the monitor node the robot interfaces use: a ROS clock and a logger keeping its lines.

    Args:
        clock: ROS time now, seconds (e.g. `FakeContext.now`).
        logs: Gets (level, message) per line, like `FakeContext.logs`.
    """
    def line(level: str) -> Callable[..., None]:
        return lambda message, **_options: logs.append((level, message))

    logger = SimpleNamespace(info=line("info"), warning=line("warn"), error=line("error"))
    return SimpleNamespace(get_logger=lambda: logger,
                           get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
                               nanoseconds=int(round(clock() * 1e9)))))


def fake_arm(node: SimpleNamespace, name: str, joints: Sequence[float],
             active: str | None = SCALED_JOINT_TRAJECTORY_CONTROLLER, end_effector=None) -> ArmInterface:
    """A real `ArmInterface` without ROS: measured now at `joints`, `active` running, publishers kept.

    The safety sync vouches for it (a recent, operational status with a dashboard, its program playing), and the
    controller manager answered now; a controller switch takes effect at once and is kept in `controllers.switches`.

    Args:
        node: `fake_node`.
        name: The arm's name, e.g. "left_ur_arm".
        joints: Its six joint angles, UR order.
        active: The running controller.
        end_effector: The mounted tool's interface, or None.
    """
    arm = ArmInterface.__new__(ArmInterface)
    arm._node, arm.config = node, ArmConfig(name=name, ros_namespace=name)
    now = arm._now()
    measured = {f"{name}_{joint}": float(value) for joint, value in zip(UR_JOINT_NAMES, joints)}
    arm.state = ArmState(joint_positions=measured, last_update_time=now,
                         controllers=ControllerManagerState(active=active, last_update_time=now),
                         status_update_time=now, dashboard_up=True, operational=True, program_playing=True)
    arm.controllers = Controllers(arm.state.controllers, f"/{name}", node)
    arm.controllers.switches = []

    def switch(controller: str, unguarded: bool = False) -> bool:
        """Start `controller` at once, as an answered switch request would; refused by the arm's guard as the real one.

        The guard (`ArmInterface._switch_refusal`: the lease, and for free drive the latch and the safety sync) runs
        unless `unguarded`. Every switch asked is kept in `switches`, refused ones too.
        """
        arm.controllers.switches.append(controller)
        problem = None if unguarded else arm._switch_refusal(controller)
        if problem is not None:
            arm.state.controllers.switch_error = f"{controller} not started: {problem}"
            return False
        arm.state.controllers.active = arm.state.controllers.requested = controller
        return True

    arm.controllers.switch = switch
    arm._simulated, arm._free_drive_until, arm._free_drive_leases = lambda: None, float("-inf"), frozenset()
    # * Its own latch and lease slot, until `fake_robot` gives it the robot's.
    arm._latch, arm._lease_slot = SoftStopLatch(name), LeaseSlot(name)
    arm.urdf_problem, arm.pendant_tcp, arm.base_in_husky = None, Pose(), Pose()
    arm._test_mode_logged, arm._motion_reference, arm._slight_reference, arm._free_drive_on = {}, {}, {}, False
    arm._trajectory, arm._target_frame, arm._target_wrench = Published(), Published(), Published()
    arm.trajectories_sent, arm.targets_sent, arm.wrenches_sent = (Event(f"{name}_trajectory"), Event(f"{name}_target"),
                                                                  Event(f"{name}_wrench"))
    arm._zero_ft = SimpleNamespace(service_is_ready=lambda: True, calls=[])
    arm._zero_ft.call_async = arm._zero_ft.calls.append
    arm.end_effector = end_effector
    arm.state.end_effector = end_effector.state if end_effector is not None else None
    return arm


def fake_robot(node: SimpleNamespace, serial: str, arms: Sequence[ArmInterface]) -> HuskyRobotInterface:
    """A real `HuskyRobotInterface` without ROS around `arms`; a two-arm robot keeps its multi-arm messages."""
    robot = HuskyRobotInterface.__new__(HuskyRobotInterface)
    robot._node = node
    robot.config = SimpleNamespace(serial=serial, arms=tuple(arm.config for arm in arms))
    robot.arms = {arm.config.name: arm for arm in arms}
    robot.base = SimpleNamespace(state=BaseState())  # never measured, as in the simulator
    robot._multi_arm = Published() if len(arms) == 2 else None
    # * One latch and one lease slot for the whole robot, shared by its arms and tools; the lease starts free.
    robot.soft_stop_latch, robot.lease_slot = SoftStopLatch(serial), LeaseSlot(serial)
    for arm in arms:
        arm._latch, arm._lease_slot = robot.soft_stop_latch, robot.lease_slot
        if arm.end_effector is not None:
            arm.end_effector._latch, arm.end_effector._lease_slot = robot.soft_stop_latch, robot.lease_slot
    return robot


@contextmanager
def from_outside(robot: HuskyRobotInterface) -> Iterator[None]:
    """Send in the block as something outside this monitor would (the pendant, another monitor), leaving no lease.

    The robot's lease is held only inside the block, so after it only the motion shows, not a lease. Outside senders
    know nothing of this monitor's leases: one the robot has (e.g. the executor's) is set aside meanwhile, then put
    back.
    """
    kept, robot.lease_slot.lease = robot.lease_slot.lease, None
    try:
        with leased([robot.lease_slot], "outside the monitor"):
            yield
    finally:
        robot.lease_slot.lease = kept


def lease_to_test(part):
    """Lease a fake robot, or a fake arm or tool on its own, to the lease every test holds (`robot_fakes.TEST_LEASE`).

    A test may then send to it directly; returns `part`.
    """
    slot = part.lease_slot if isinstance(part, HuskyRobotInterface) else part._lease_slot
    slot.lease = TEST_LEASE
    return part


class ArmDriver:
    """Moves a fake arm along the last trajectory it was sent (alone or in a multi-arm message), as its controller.

    Each `tick(t)` feeds the arm a joint state, through the arm's own callback, so its motion detection runs.

    Attributes:
        follow: The fraction of each trajectory it runs before it stops dead (1: all of it).
        silent_from: ROS time from which no joint state is fed any more, or None.
        speed: How fast it runs a trajectory, 1 as timed.
        synced: Whether the safety sync reports on the arm every tick (False: its last status grows old).
    """

    def __init__(self, arm: ArmInterface, robot: HuskyRobotInterface | None = None, follow: float = 1.0,
                 silent_from: float | None = None, speed: float = 1.0):
        """Drive `arm`; `robot` is its robot, whose multi-arm messages it reads too."""
        self.arm, self.robot, self.follow, self.silent_from, self.speed = arm, robot, follow, silent_from, speed
        self.synced = True
        self._seen = (0, 0)
        self._times = self._points = None
        self._started = 0.0

    def tick(self, t: float) -> None:
        """Take a newly sent trajectory, move along it, and feed the joint state at time `t`.

        The controller manager and the safety sync answer too, as their polls would
        (`ControllerManagerInterface.refresh`, `ArmInterface.on_sync_status`).
        """
        self.arm.state.controllers.last_update_time = t
        if self.synced:
            self.arm.state.status_update_time = t
        single = self.arm._trajectory.messages
        multi = self.robot._multi_arm.messages if self.robot is not None and self.robot._multi_arm else []
        if (len(single), len(multi)) != self._seen:
            if len(single) > self._seen[0]:
                message = single[-1]
            else:
                first = next(iter(self.robot.arms)) == self.arm.config.name
                message = multi[-1].trajectory1 if first else multi[-1].trajectory2
            self._seen = (len(single), len(multi))
            self._take(message, t)
        if self.silent_from is not None and t >= self.silent_from:
            return
        here = self.arm.joint_vector()
        if self._times is not None:
            elapsed = min((t - self._started) * self.speed, self._times[-1] * self.follow)
            here = np.array([np.interp(elapsed, self._times, column) for column in self._points.T])
        state = JointState(name=list(UR_JOINT_NAMES), position=[float(value) for value in here])
        state.header.stamp.sec, state.header.stamp.nanosec = int(t), int((t % 1.0) * 1e9)
        self.arm._on_joint_state(state)

    def _take(self, message, t: float) -> None:
        """Start running a trajectory message from where the arm is (a hold has a single point, later than 0)."""
        times = [point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 for point in message.points]
        points = [list(point.positions) for point in message.points]
        if times[0] > 0.0:
            times, points = [0.0, *times], [list(self.arm.joint_vector()), *points]
        self._times, self._points, self._started = np.array(times), np.array(points), t


# --- --- --- --- --- TOOLS --- --- --- --- ---

class FakeGoal(SimpleNamespace):
    """One goal sent to a fake GripperCommand server: its message, its goal handle, and whether it was cancelled.

    `on_answer` and `on_result` are the gripper's callbacks, called by the `RobotiqDriver` when it answers.
    """

    def cancel_goal_async(self) -> SimpleNamespace:
        """Ask to cancel; the driver ends the goal as cancelled at its next tick."""
        self.cancelled = True
        return SimpleNamespace(add_done_callback=lambda _callback: None)


class FakeAction:
    """A GripperCommand action client: ready or not, keeping every goal sent (`FakeGoal`)."""

    def __init__(self, ready: bool = True):
        """Ready unless told otherwise."""
        self.ready, self.goals = ready, []

    def server_is_ready(self) -> bool:
        """Whether the server answers."""
        return self.ready

    def send_goal_async(self, goal) -> SimpleNamespace:
        """Keep the goal; the answer is the driver's."""
        sent = FakeGoal(command=goal.command, cancelled=False, accepted=True, on_answer=None, on_result=None)
        sent.get_result_async = lambda: SimpleNamespace(add_done_callback=partial(setattr, sent, "on_result"))
        self.goals.append(sent)
        return SimpleNamespace(add_done_callback=partial(setattr, sent, "on_answer"))


def fake_robotiq(node: SimpleNamespace, namespace: str = "/a200_0804/gripper", position: float = 0.0) -> RobotiqGripper:
    """A real `RobotiqGripper` without ROS, its fingers measured at `position`, its goals kept (`FakeAction`)."""
    gripper = RobotiqGripper.__new__(RobotiqGripper)
    gripper._node, gripper.namespace = node, namespace
    gripper.state = RobotiqState(position=position)
    gripper._action = FakeAction()
    gripper._goal, gripper._cancel_on_accept = None, False
    # * Its own latch and lease slot, until `fake_robot` gives it the robot's.
    gripper._latch, gripper._lease_slot = SoftStopLatch(namespace), LeaseSlot(namespace)
    return gripper


class RobotiqDriver:
    """Accepts a fake Robotiq's goals at once and ends them `delay` s after they are sent, its fingers at `reach(target)`.

    A cancelled goal ends cancelled at the next tick, the fingers where they are.

    Attributes:
        answer: False: never answers (the action server hangs).
    """

    def __init__(self, gripper: RobotiqGripper, delay: float = 0.5, reach: Callable[[float], float] = lambda t: t,
                 answer: bool = True):
        """Answer `gripper`'s goals."""
        self.gripper, self.delay, self.reach, self.answer = gripper, delay, reach, answer
        self._sent_at: dict[int, float] = {}
        self._ended: set[int] = set()

    def tick(self, t: float) -> None:
        """Accept new goals, then end the goals that were cancelled or whose time has come, in order."""
        if not self.answer:
            return
        for index, goal in enumerate(self.gripper._action.goals):
            if index in self._ended:
                continue
            if index not in self._sent_at:
                self._sent_at[index] = t
                goal.on_answer(SimpleNamespace(result=lambda goal=goal: goal))
            if goal.cancelled:
                self._ended.add(index)
                goal.on_result(SimpleNamespace(result=lambda: SimpleNamespace(status=GoalStatus.STATUS_CANCELED)))
            elif t - self._sent_at[index] >= self.delay:
                self._ended.add(index)
                target = goal.command.position
                reached = self.reach(target)
                self.gripper._on_joint_state(JointState(name=[RobotiqGripper.KNUCKLE_JOINT], position=[reached]))
                result = SimpleNamespace(reached_goal=abs(reached - target) < 1e-3,
                                         stalled=abs(reached - target) >= 1e-3, position=reached)
                goal.on_result(SimpleNamespace(result=lambda result=result: SimpleNamespace(
                    status=GoalStatus.STATUS_SUCCEEDED, result=result)))


def fake_scaffolding(node: SimpleNamespace, namespace: str = "/a200_0806/left_gripper") -> ScaffoldingV3:
    """A real `ScaffoldingV3` without ROS: no status yet, its commands kept (`Published`)."""
    tool = ScaffoldingV3.__new__(ScaffoldingV3)
    tool._node, tool.namespace = node, namespace
    tool.state = ScaffoldingV3State()
    tool._command = Published()
    # * Its own latch and lease slot, until `fake_robot` gives it the robot's.
    tool._latch, tool._lease_slot = SoftStopLatch(namespace), LeaseSlot(namespace)
    return tool


class ScaffoldingDriver:
    """The tool's firmware: STOP clears a stall; a gripper tighten runs until it stalls `stall_after` s later.

    A stalled motor ignores a start, as the firmware does. A status goes out every tick unless `silent`.

    Attributes:
        motor: The gripper motor's state now.
        stall_after: Seconds a tighten runs before it stalls; None: never.
        silent: No status is sent.
    """

    def __init__(self, tool: ScaffoldingV3, motor: str = ScaffoldingV3.IDLE, stall_after: float | None = 1.0,
                 silent: bool = False):
        """Run `tool`'s gripper motor, starting in state `motor`."""
        self.tool, self.motor, self.stall_after, self.silent = tool, motor, stall_after, silent
        self._seen, self._since = 0, 0.0

    def tick(self, t: float) -> None:
        """Take new commands, run the motor, and report."""
        commands = self.tool._command.messages
        for command in commands[self._seen:]:
            if command.direction == 0:
                self.motor = ScaffoldingV3.IDLE
            elif command.motor == ScaffoldingV3.GRIPPER_MOTOR and self.motor != ScaffoldingV3.STALLED:
                self.motor = ScaffoldingV3.TIGHTENING if command.direction > 0 else ScaffoldingV3.LOOSENING
                self._since = t
        self._seen = len(commands)
        running = self.motor == ScaffoldingV3.TIGHTENING and self.stall_after is not None
        if running and t - self._since >= self.stall_after:
            self.motor = ScaffoldingV3.STALLED
        if not self.silent:
            self.tool._on_status(SimpleNamespace(state_m1=self.motor, state_m2=ScaffoldingV3.IDLE, current=0,
                                                 pwm_pct=0, buttons_held=(), buttons_pressed=()))
