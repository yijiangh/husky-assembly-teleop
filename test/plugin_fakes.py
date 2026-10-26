"""A plugin context for tests: ROS time ticking at 10 Hz, tasks on the running loop, other plugins, logs and a GUI.

Run the plugin's tasks inside `asyncio.run`; `step(t)` changes the world each tick (e.g. the snapshot). Widgets are
`FakeWidget`s: call a widget's `callback` to click it.

Robots: `fake_arm` and `fake_robot` are the real `ArmInterface` and `HuskyRobotInterface` without ROS, so every check
they run is the real one; their publishers keep what is sent (`Published`), and an `ArmDriver` moves an arm along what
it was sent, feeding its joint states and the safety sync's status as the robot would. `fake_robotiq` and
`fake_scaffolding` are the real tool interfaces, answered by a `RobotiqDriver` and a `ScaffoldingDriver`.

Leases: a fake robot's parts share its soft-stop latch and its robot lease slot, free at first, as on a real robot.
Code that sends to them takes the lease as the monitor's plugins do (`FakeContext.take_lease`, the real one), or a
test hands it the lease every test holds (`lease_to_test`). Freezes are the real ones too, checked every tick.
"""

from __future__ import annotations

import asyncio
from contextlib import contextmanager, nullcontext
from functools import partial
from types import SimpleNamespace
from typing import Any, Callable, Iterator, Sequence

import numpy as np
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped
from robot_fakes import TEST_LEASE, Client
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState

from husky_assembly_teleop.config import ArmConfig
from husky_assembly_teleop.plugin_api.concurrency import WaitTimeout
from husky_assembly_teleop.plugin_api.context import PluginContext
from husky_assembly_teleop.robot_interface.arm import (ARM_CONTROLLERS, CARTESIAN_COMPLIANCE_CONTROLLER,
                                                       SCALED_JOINT_TRAJECTORY_CONTROLLER, ArmInterface, ArmState,
                                                       SentTarget)
from husky_assembly_teleop.robot_interface.base import BaseState
from husky_assembly_teleop.robot_interface.controller_manager import ControllerManagerInterface, ControllerManagerState
from husky_assembly_teleop.robot_interface.end_effectors import (RobotiqGripper, RobotiqState, ScaffoldingV3,
                                                                 ScaffoldingV3State)
from husky_assembly_teleop.robot_interface.event import Event
from husky_assembly_teleop.robot_interface.mocap_state import MARKER_ERROR_WINDOW
from husky_assembly_teleop.robot_interface.robot import HuskyRobotInterface
from husky_assembly_teleop.robot_interface.robot_lease import LeaseSlot, leased
from husky_assembly_teleop.robot_interface.soft_stop import SoftStopLatch
from husky_assembly_teleop.robot_interface.ur_frames import BASE_LINK_FROM_UR_BASE
from husky_assembly_teleop.world.measured import TrackedObject
from scaffolding_assembly_core.geometry import Pose, compose, invert
from scaffolding_assembly_core.kinematics import link_pose
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
                                content="", color=options.get("color"), disabled=options.get("disabled", False),
                                options=options,
                                _impl=SimpleNamespace(uuid=f"fake-{len(self.widgets)}", removed=False))
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
    #: Freezes, checked after each tick's step: the real `PluginContext` code.
    freeze = PluginContext.freeze
    _check_freezes = PluginContext._check_freezes
    _own_task = PluginContext._own_task
    #: Waiting for a task without taking over its cancellation (shielded), and Stop all's cancel of every task but
    #: the `survivors`: the real `PluginContext` code.
    join = PluginContext.join
    _cancel_for_soft_stop = PluginContext._cancel_for_soft_stop

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
        self.survivors = self._survivors = set()  # `_survivors` as `PluginContext` names it
        #: The tasks spawned, and the freezes being watched with the task each cancels.
        self._tasks: set[asyncio.Task] = set()
        self._freezes: dict = {}
        self.t, self.step = 10.0, step
        #: Whether the plugin was stopped or torn down (`PluginContext.closed`).
        self.closed = False
        self.config = SimpleNamespace(robots=tuple(robots), drive_root=drive_root)
        self.plugins = dict(plugins or {})
        self.scene = SimpleNamespace(snapshot=snapshot if snapshot is not None else Scene())
        self.world = SimpleNamespace(robots={}, tracked_objects={})
        self.gui = FakeGui()
        self.view = SimpleNamespace(ui=lambda band=None: nullcontext(self.gui),
                                    phone=lambda *_tab: nullcontext(self.gui), scene_root="/plugin")
        #: (level, message) per log line.
        self.logs: list[tuple[str, str]] = []

    def now(self) -> float:
        """ROS time, seconds."""
        return self.t

    def require(self, name: str) -> Any:
        """Another plugin, by name."""
        return self.plugins[name]

    def track_object(self, name: str, mocap_id: int, geometry=None, touches: tuple = (),
                     label: str = "") -> TrackedObject:
        """Track a mocap body as `world.tracked_objects[name]`; a test fills its `markers` from its `step`.

        Raises:
            ValueError: If `name` is already tracked, as the real context does.
        """
        if name in self.world.tracked_objects:
            raise ValueError(f"{name!r} is already tracked")
        self.world.tracked_objects[name] = TrackedObject(name, mocap_id)
        return self.world.tracked_objects[name]

    def untrack_object(self, name: str) -> None:
        """Stop tracking `name`; unknown names are ignored."""
        self.world.tracked_objects.pop(name, None)

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
        """Start a task on the running loop; one a soft stop leaves alone is kept in `survivors`.

        Raises:
            RuntimeError: If the plugin is `closed`, as `PluginContext.spawn`.
        """
        if self.closed:
            work.close()
            raise RuntimeError(f"plugin {self.name!r} is stopped; it cannot start {label!r}")
        task = asyncio.get_running_loop().create_task(work, name=label)
        self._tasks.add(task)
        if survives_soft_stop:
            self.survivors.add(task)
        return task

    async def run_in_thread(self, function: Callable, *args: Any, **kwargs: Any) -> Any:
        """Run on the loop's default thread pool."""
        return await asyncio.get_running_loop().run_in_executor(None, partial(function, *args, **kwargs))

    async def next_tick(self) -> None:
        """Advance 0.1 s, step the world, cancel the tasks whose freeze broke, let other tasks run."""
        self.t += 0.1
        self.step(self.t)
        self._check_freezes()
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

    def log_info(self, message: str, topic: str = "") -> None:
        """Keep the line; the topic is not kept."""
        self.logs.append(("info", message))

    def log_warn(self, message: str, topic: str = "") -> None:
        """Keep the line; the topic is not kept."""
        self.logs.append(("warn", message))

    def log_error(self, message: str, topic: str = "") -> None:
        """Keep the line; the topic is not kept."""
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


class FakeControllers(ControllerManagerInterface):
    """An arm's real controller manager interface on fake services, answered as the robot's controller manager would.

    The manager runs `running`; an accepted switch makes it run the controller asked for. Requests are answered in
    the order sent, `delay` ticks later (`tick`): 0 answers inside the request, None never (a switch stays in
    flight). Every switch asked, refused ones too, is kept in `switches`.

    Attributes:
        running: What the controller manager runs; `state` learns it from the answers.
        delay: Ticks before a request is answered, or None: never. A test may change it at any time.
        rejects: Whether the controller manager rejects every switch.
        switches: Every controller a switch was asked for, in order.
    """

    def __init__(self, node: SimpleNamespace, namespace: str, state: ControllerManagerState,
                 guard: Callable[[str], str | None]):
        """Run `state.active`, services up, answering at once; `guard` is the arm's `_switch_refusal`."""
        self._node, self.namespace, self.switchable, self.state = node, namespace, ARM_CONTROLLERS, state
        self._guard, self._reported_unavailable = guard, False
        self.running, self.delay, self.rejects, self.switches = state.active, 0, False, []
        #: Ticks so far; the requests not answered yet, with the tick each was sent in.
        self._ticks, self._queue, self._answering = 0, [], False
        self._list_client, self._switch_client = self._service(), self._service()
        self._configure_client = Client(ready=False)  # `restart` then switches as it is

    @property
    def ready(self) -> bool:
        """bool: Whether its services are up; while not, nothing goes out (`switch` and `refresh` say False)."""
        return self._switch_client.ready

    @ready.setter
    def ready(self, up: bool) -> None:
        self._switch_client.ready = self._list_client.ready = up

    def switch(self, controller: str, unguarded: bool = False) -> bool:
        """The real `switch`, kept in `switches`."""
        self.switches.append(controller)
        sent = super().switch(controller, unguarded)
        self._answer_due()
        return sent

    def refresh(self) -> bool:
        """The real `refresh`."""
        sent = super().refresh()
        self._answer_due()
        return sent

    def deactivate_all(self) -> bool:
        """The real `deactivate_all`."""
        sent = super().deactivate_all()
        self._answer_due()
        return sent

    def tick(self, _t: float = 0.0) -> None:
        """One tick passes: answer the requests whose time has come."""
        self._ticks += 1
        self._answer_due()

    def _service(self) -> Client:
        """A service client whose calls join the answer queue."""
        client = Client()
        call = client.call_async

        def queued(request):
            future = call(request)
            self._queue.append((self._ticks, future))
            return future

        client.call_async = queued
        return client

    def _answer_due(self) -> None:
        """Answer, in order, every request at least `delay` ticks old; a request an answer makes joins the queue."""
        if self._answering:
            return
        self._answering = True
        try:
            while self.delay is not None and self._queue and self._ticks - self._queue[0][0] >= self.delay:
                _tick, future = self._queue.pop(0)
                if isinstance(future.request, SwitchController.Request):
                    if not self.rejects:
                        activate = list(future.request.activate_controllers)
                        self.running = activate[0] if activate else None
                    future.answer(SimpleNamespace(ok=not self.rejects))
                else:
                    future.answer(SimpleNamespace(controller=[
                        SimpleNamespace(name=name, state="active" if name == self.running else "inactive")
                        for name in self.switchable]))
        finally:
            self._answering = False


def fake_arm(node: SimpleNamespace, name: str, joints: Sequence[float],
             active: str | None = SCALED_JOINT_TRAJECTORY_CONTROLLER, end_effector=None) -> ArmInterface:
    """A real `ArmInterface` without ROS: measured now at `joints`, `active` running, publishers kept.

    The safety sync vouches for it (a recent, operational status with a dashboard, its program playing), and the
    controller manager answered now. Its controllers are `FakeControllers`: a switch takes effect at once unless the
    test says otherwise, and is kept in `controllers.switches`.

    Args:
        node: `fake_node`.
        name: The arm's name, e.g. "left_ur_arm".
        joints: Its six joint angles, UR order.
        active: The running controller.
        end_effector: The interface of the tool on its flange, or None.
    """
    arm = ArmInterface.__new__(ArmInterface)
    arm._node, arm.config = node, ArmConfig(name=name, ros_namespace=name)
    now = arm._now()
    measured = {f"{name}_{joint}": float(value) for joint, value in zip(UR_JOINT_NAMES, joints)}
    arm.state = ArmState(joint_positions=measured, last_update_time=now,
                         controllers=ControllerManagerState(active=active, last_update_time=now),
                         status_update_time=now, dashboard_up=True, operational=True, program_playing=True)
    arm.controllers = FakeControllers(node, f"/{name}", arm.state.controllers, arm._switch_refusal)
    arm._simulated, arm._free_drive_until, arm._free_drive_leases = lambda: None, float("-inf"), frozenset()
    # * Its own latch and lease slot, until `fake_robot` gives it the robot's.
    arm._latch, arm._lease_slot = SoftStopLatch(name), LeaseSlot(name)
    arm.urdf_problem, arm.urdf, arm.pendant_tcp, arm.base_in_husky = None, None, Pose(), Pose()
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

    Each `tick(t)` feeds the arm a joint state, then its TCP, through the arm's own callbacks, so its motion
    detection and frame conversion run.

    - Under the trajectory controller the TCP is tool0 by the URDF's kinematics of the joints; only with `arm.urdf`.
    - Under the compliance controller the arm goes `lag` of the way to the last Cartesian target each tick: the joints
      take one damped least-squares step there on the URDF (at most FOLLOW_MAX_STEP each), and the TCP is their
      kinematics, so nothing jumps on a switch; next to a singularity the TCP lags. Without `arm.urdf` the TCP moves
      there and the joints stay put. Targets sent before the controller started are not followed.
    - A controller that starts holds the arm where it is: the trajectory the one before ran is dropped.

    Attributes:
        follow: The fraction of each trajectory it runs before it stops dead (1: all of it).
        silent_from: ROS time from which no joint state is fed any more, or None.
        speed: How fast it runs a trajectory, 1 as timed.
        lag: The fraction of the way to the last Cartesian target the TCP moves each tick (1: it jumps there).
        synced: Whether the safety sync reports on the arm every tick (False: its last status grows old).
    """

    #: The damping of the compliant joints' step, and the most a joint moves in one tick, radians.
    FOLLOW_DAMPING, FOLLOW_MAX_STEP = 0.01, 0.1

    def __init__(self, arm: ArmInterface, robot: HuskyRobotInterface | None = None, follow: float = 1.0,
                 silent_from: float | None = None, speed: float = 1.0, lag: float = 0.5):
        """Drive `arm`; `robot` is its robot, whose multi-arm messages it reads too."""
        self.arm, self.robot, self.follow, self.silent_from, self.speed = arm, robot, follow, silent_from, speed
        self.lag = lag
        self.synced = True
        self._seen = (0, 0)
        self._times = self._points = None
        self._started = 0.0
        #: The controller running at the last tick; the arm's base_link in the URDF's root; the last Cartesian target
        #: sent; the last tick the arm was under another controller than compliance.
        self._controller = arm.state.controllers.active
        self._base_link: Pose | None = None
        self._target: SentTarget | None = None
        self._outside = float("-inf")
        arm.targets_sent.attach(self._on_target)

    def tick(self, t: float) -> None:
        """Take a newly sent trajectory, move along it or follow the Cartesian target, and feed the state at time `t`.

        The controller manager and the safety sync answer too, as their polls would
        (`ControllerManagerInterface.refresh`, `ArmInterface.on_sync_status`).
        """
        arm = self.arm
        arm.state.controllers.last_update_time = t
        if self.synced:
            arm.state.status_update_time = t
        if arm.state.controllers.active != self._controller:
            self._controller, self._times = arm.state.controllers.active, None
        single = arm._trajectory.messages
        multi = self.robot._multi_arm.messages if self.robot is not None and self.robot._multi_arm else []
        if (len(single), len(multi)) != self._seen:
            if len(single) > self._seen[0]:
                message = single[-1]
            else:
                first = next(iter(self.robot.arms)) == arm.config.name
                message = multi[-1].trajectory1 if first else multi[-1].trajectory2
            self._seen = (len(single), len(multi))
            self._take(message, t)
        if self.silent_from is not None and t >= self.silent_from:
            return
        here = arm.joint_vector()
        if self._times is not None:
            elapsed = min((t - self._started) * self.speed, self._times[-1] * self.follow)
            here = np.array([np.interp(elapsed, self._times, column) for column in self._points.T])
        tool0 = None
        if arm.controllers.is_active(CARTESIAN_COMPLIANCE_CONTROLLER):
            tool0, moves = self._toward_target()
            if moves and arm.urdf is not None:
                here = self._follow(here, tool0)
                tool0 = self._fk(here)
        else:
            self._outside = t
            if arm.urdf is not None and arm.controllers.is_active(SCALED_JOINT_TRAJECTORY_CONTROLLER):
                tool0 = self._fk(here)
        state = JointState(name=list(UR_JOINT_NAMES), position=[float(value) for value in here])
        state.header.stamp.sec, state.header.stamp.nanosec = int(t), int((t % 1.0) * 1e9)
        arm._on_joint_state(state)
        if tool0 is not None:
            self._report_tcp(tool0)

    def _take(self, message, t: float) -> None:
        """Start running a trajectory message from where the arm is (a hold has a single point, later than 0)."""
        times = [point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 for point in message.points]
        points = [list(point.positions) for point in message.points]
        if times[0] > 0.0:
            times, points = [0.0, *times], [list(self.arm.joint_vector()), *points]
        self._times, self._points, self._started = np.array(times), np.array(points), t

    def _on_target(self, target: SentTarget) -> None:
        """Keep the last Cartesian target sent."""
        self._target = target

    def _toward_target(self) -> tuple[Pose | None, bool]:
        """The TCP `lag` of the way to the last target sent under this controller, and whether that moves it.

        Returns:
            tuple[Pose | None, bool]: In the arm's base_link; (None, False) before the first TCP.
        """
        state, target = self.arm.state, self._target
        if state.tcp_position is None:
            return None, False
        position, turn = np.asarray(state.tcp_position), Rotation.from_quat(state.tcp_orientation)
        here = Pose.from_arrays(position, turn.as_quat())
        # * A target sent in the tick the arm was last seen under another controller came after that look.
        if target is None or target.update_time < self._outside:
            return here, False
        shift, rotation = target.position - position, (turn.inv() * Rotation.from_quat(target.orientation)).as_rotvec()
        if np.linalg.norm(shift) <= 1e-7 and np.linalg.norm(rotation) <= 1e-7:
            return here, False
        return Pose.from_arrays(position + self.lag * shift,
                                (turn * Rotation.from_rotvec(self.lag * rotation)).as_quat()), True

    def _base(self) -> Pose:
        """The arm's base_link in the URDF's root (the arm's joints do not move it)."""
        if self._base_link is None:
            self._base_link = link_pose(self.arm.urdf, Pose(), {}, f"{self.arm.config.name}_base_link")
        return self._base_link

    def _root_tool0(self, joints: np.ndarray) -> Pose:
        """tool0 in the URDF's root at `joints` (UR order)."""
        name = self.arm.config.name
        return link_pose(self.arm.urdf, Pose(), {f"{name}_{joint}": float(value)
                                                 for joint, value in zip(UR_JOINT_NAMES, joints)}, f"{name}_tool0")

    def _fk(self, joints: np.ndarray) -> Pose:
        """tool0 in the arm's base_link by the URDF's kinematics of `joints` (UR order)."""
        return compose(invert(self._base()), self._root_tool0(joints))

    def _follow(self, joints: np.ndarray, tool0: Pose) -> np.ndarray:
        """The joints a damped least-squares step nearer to tool0 at `tool0` (base_link), each ≤ FOLLOW_MAX_STEP."""
        goal = compose(self._base(), tool0).matrix()

        def gap(values: np.ndarray) -> np.ndarray:
            """How far tool0 at `values` is from `goal`: shift, then turn as a rotation vector."""
            here = self._root_tool0(values).matrix()
            return np.concatenate([here[:3, 3] - goal[:3, 3],
                                   Rotation.from_matrix(here[:3, :3] @ goal[:3, :3].T).as_rotvec()])

        here = np.asarray(joints, dtype=float)
        error = gap(here)
        jacobian = np.column_stack([(gap(here + 1e-7 * unit) - error) / 1e-7 for unit in np.eye(6)])
        step = -jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + self.FOLLOW_DAMPING ** 2 * np.eye(6), error)
        return here + step * min(1.0, self.FOLLOW_MAX_STEP / max(float(np.max(np.abs(step))), 1e-12))

    def _report_tcp(self, tool0: Pose) -> None:
        """Feed `tool0` as the driver's tcp_pose would report it: in UR Base, the pendant's TCP on."""
        reported = compose(tool0, self.arm.pendant_tcp)
        message = PoseStamped()
        p, q = message.pose.position, message.pose.orientation
        p.x, p.y, p.z = map(float, BASE_LINK_FROM_UR_BASE.inv().apply(reported.position))
        q.x, q.y, q.z, q.w = map(float, (BASE_LINK_FROM_UR_BASE.inv() * Rotation.from_quat(reported.orientation))
                                 .as_quat())
        self.arm._on_tcp_pose(message)


class MocapDriver:
    """Mocap seeing a fake robot's base at `pose` every tick while `live`, as the relay would: `pose_lost` passes.

    Attributes:
        pose: Where the base's tracked frame is, world; change it to move the base.
        live: Whether samples come; off, the last one grows old (lost after POSE_TIMEOUT).
    """

    def __init__(self, robot: HuskyRobotInterface, pose: Pose, mocap_id: int = 6):
        """Track the base of `robot` (from `fake_robot`) as rigid body `mocap_id`."""
        robot.base.mocap_id = mocap_id
        self.robot, self.pose, self.live = robot, pose, True

    def tick(self, t: float) -> None:
        """Measure the base now, if live."""
        if not self.live:
            return
        state = self.robot.base.state
        state.position, state.orientation = np.array(self.pose.position), np.array(self.pose.orientation)
        state.tracked = state.tracking_valid = True
        state.last_update_time = state.last_fix_time = t
        state.marker_errors.append((t, 0.0))
        while state.marker_errors[0][0] < t - MARKER_ERROR_WINDOW:
            state.marker_errors.popleft()


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
    """The tool's firmware: one motor runs at a time and stalls a set time after it starts; STOP clears both.

    Starting a motor stops the other one if it runs; a stalled motor ignores a start; STOP idles both (crl_husky
    protocol.md). A status goes out every tick unless `silent`.

    Attributes:
        motor: The gripper motor's (M1) state now; a test may set it.
        screw: The joint screw motor's (M2) state now; a test may set it.
        stall_after: Seconds the gripper motor runs, either way, before it stalls; None: never.
        screw_stall_after: The same for the joint screw motor.
        silent: No status is sent.
    """

    RUNNING = (ScaffoldingV3.TIGHTENING, ScaffoldingV3.LOOSENING)

    def __init__(self, tool: ScaffoldingV3, motor: str = ScaffoldingV3.IDLE, stall_after: float | None = 1.0,
                 silent: bool = False, screw: str = ScaffoldingV3.IDLE, screw_stall_after: float | None = 1.0):
        """Run `tool`'s motors, starting in states `motor` (M1) and `screw` (M2)."""
        self.tool, self.motor, self.stall_after, self.silent = tool, motor, stall_after, silent
        self.screw, self.screw_stall_after = screw, screw_stall_after
        self._seen = 0
        #: When each motor last started.
        self._since = {ScaffoldingV3.GRIPPER_MOTOR: 0.0, ScaffoldingV3.JOINT_MOTOR: 0.0}

    def tick(self, t: float) -> None:
        """Take new commands, run the motors, and report."""
        commands = self.tool._command.messages
        for command in commands[self._seen:]:
            if command.direction == 0:
                self.motor = self.screw = ScaffoldingV3.IDLE
            else:
                self._start(command.motor, command.direction, t)
        self._seen = len(commands)
        self.motor = self._run(self.motor, ScaffoldingV3.GRIPPER_MOTOR, self.stall_after, t)
        self.screw = self._run(self.screw, ScaffoldingV3.JOINT_MOTOR, self.screw_stall_after, t)
        if not self.silent:
            self.tool._on_status(SimpleNamespace(state_m1=self.motor, state_m2=self.screw, current=0,
                                                 pwm_pct=0, buttons_held=(), buttons_pressed=()))

    def _start(self, motor: int, direction: int, t: float) -> None:
        """Start `motor` unless it is stalled; the other one stops if it runs."""
        gripper = motor == ScaffoldingV3.GRIPPER_MOTOR
        if (self.motor if gripper else self.screw) == ScaffoldingV3.STALLED:
            return  # the firmware answers "ERR 2 STALLED"
        self.motor = ScaffoldingV3.IDLE if self.motor in self.RUNNING else self.motor
        self.screw = ScaffoldingV3.IDLE if self.screw in self.RUNNING else self.screw
        started = ScaffoldingV3.TIGHTENING if direction > 0 else ScaffoldingV3.LOOSENING
        if gripper:
            self.motor = started
        else:
            self.screw = started
        self._since[motor] = t

    def _run(self, state: str, motor: int, stall_after: float | None, t: float) -> str:
        """`motor`'s state after this tick: a running one stalls `stall_after` s after it started."""
        running = state in self.RUNNING and stall_after is not None
        return ScaffoldingV3.STALLED if running and t - self._since[motor] >= stall_after else state
