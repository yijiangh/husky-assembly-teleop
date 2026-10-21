"""
The plugin contract: what a plugin is handed, and what it is allowed to touch.

Each plugin gets its own PluginContext, so its queue, tasks and UI are disposed of with it.

! Import only `concurrency`, `trace`, `robot_interface.soft_stop`, `robot_interface.robot_lease`, `world.freeze` and
  `world.clearance` at runtime, the rest under TYPE_CHECKING: `plugin` and `monitor` import this module.
"""

from __future__ import annotations

import asyncio
import queue
import traceback
from contextlib import contextmanager
from dataclasses import dataclass, replace
from functools import partial
from typing import TYPE_CHECKING, Any, Awaitable, Callable, Coroutine, Iterable, Iterator, TypeVar

from ..robot_interface.robot_lease import Lease, LeaseSlot, leased, take_lease
from ..robot_interface.soft_stop import MotionRefused
from ..world.clearance import STOP_DISTANCE, Clearance, ClearanceTrip
from ..world.freeze import Basis, Freeze, Tolerance, WorldMoved, basis_problem
from .concurrency import WaitTimeout, describe_exception, ros_future
from .trace import MAX_SAMPLES, Signal, Trace

T = TypeVar("T")

if TYPE_CHECKING:
    import viser
    from rclpy.task import Future as RosFuture

    from ..config import MonitorConfig
    from scaffolding_assembly_core.geometry import Geometry
    from ..monitor import HuskyMonitor
    from ..ui.visualization import PluginView
    from ..world.collision_hit import CollisionHit
    from ..world.kinematics import Kinematics
    from ..world.measured import TrackedObject, WorldState
    from ..world.scene import PluginScene
    from .plugin import HuskyPlugin


@dataclass(frozen=True)
class Intent:
    """Work handed in from a viser thread, to run on the ROS thread.

    Attributes:
        label: Human-readable name, used when the work raises.
        run: Called once with no arguments in the owning plugin's step; a returned coroutine is spawned.
    """

    label: str
    run: Callable[[], Any]


class PluginContext:
    """One plugin's handle on the monitor, and the monitor's bookkeeping for driving that plugin."""

    def __init__(self, name: str, monitor: HuskyMonitor, view: PluginView,
                 scene: PluginScene, kinematics: Kinematics, dependencies: tuple[str, ...] = ()):
        """Create a plugin's context.

        Args:
            name: The owning plugin's name.
            monitor: The monitor; plugins reach it only through this context.
            view: The plugin's private scene subtree and GUI folder.
            scene: The collision scene, limited to this plugin's own ids.
            kinematics: Robot base, joint and link poses, fixed for this tick.
            dependencies: Names declared in `requires`; only these resolve through `require`.
        """
        self.name = name

        #: This plugin's corner of the viser UI. Removed wholesale on teardown.
        self.view = view

        #: The collision scene, limited to ids under "<plugin name>/"; cleared when this plugin closes.
        #: ! Hand `scene.snapshot` to worker threads, never the live bodies.
        self.scene = scene

        #: Each robot's base pose, joints and link poses, fixed for this tick.
        #: ! Main thread only.
        self.kinematics = kinematics

        self._monitor = monitor
        self._dependencies = dependencies

        # Filled from viser threads, drained on the main thread at the start of this plugin's step.
        self._intents: queue.Queue[Intent] = queue.Queue()

        # Running tasks. ! Also keeps them alive: asyncio holds only a weak reference.
        self._tasks: set[asyncio.Task] = set()
        # The running tasks a soft stop leaves alone (`spawn(..., survives_soft_stop=True)`).
        self._survivors: set[asyncio.Task] = set()
        # Tasks that raised since the monitor last asked; see _take_task_failures.
        self._task_failures = 0
        # Futures of the tasks waiting for the next tick; see next_tick.
        self._tick_waiters: list[asyncio.Future] = []
        # Set once the plugin is stopped or torn down; no new tasks after that.
        self._closed = False
        # The freezes being watched, each with the task it cancels; see freeze.
        self._freezes: dict[Freeze, asyncio.Task] = {}
        # The clearances being checked, each with the task it cancels; see clearance.
        self._clearances: dict[Clearance, asyncio.Task] = {}
        # Leases taken with take_lease, given back when this plugin stops.
        self._leases: set[Lease] = set()
        # Traces being filled, each with the (event, listener) pairs of its message signals; see trace.
        self._traces: dict[Trace, list[tuple[Any, Callable[[Any], None]]]] = {}
        # Message signals that raised since the monitor last asked, and the last traceback; see _take_signal_failures.
        self._signal_failures = 0
        self._signal_failure = ""

    # --- --- --- --- --- STATE --- --- --- --- ---

    @property
    def config(self) -> MonitorConfig:
        """MonitorConfig: Read-only run configuration."""
        return self._monitor.config

    @property
    def world(self) -> WorldState:
        """WorldState: Measured reality. Read only; ROS callbacks write it."""
        return self._monitor.world

    def now(self) -> float:
        """Current ROS time in seconds; follows simulated time and bag playback."""
        return self._monitor.now()

    def log_info(self, message: str) -> None:
        """Log `message` at info level, tagged with this plugin's name."""
        self._monitor.log_info(f"[{self.name}] {message}")

    def log_warn(self, message: str) -> None:
        """Log `message` at warning level, tagged with this plugin's name."""
        self._monitor.log_warn(f"[{self.name}] {message}")

    def log_error(self, message: str) -> None:
        """Log `message` at error level, tagged with this plugin's name."""
        self._monitor.log_error(f"[{self.name}] {message}")

    def require(self, name: str) -> HuskyPlugin:
        """Get a plugin this one declared in `requires`. Undeclared ones do not resolve.

        Raises:
            KeyError: If `name` was not declared in `requires`, or is not loaded.
        """
        if name not in self._dependencies:
            raise KeyError(f"plugin {self.name!r} did not declare a dependency on {name!r}; "
                           f"add it to `requires`")
        return self._monitor.plugin(name)

    def track_object(self, name: str, mocap_id: int, geometry: Geometry | None = None,
                     touches: tuple[str, ...] = (), label: str = "") -> TrackedObject:
        """Start tracking mocap rigid body `mocap_id` as `world.tracked_objects[name]`, id "tracked/<name>".

        - ! Call from a hook, intent or task, never from a viser callback.
        - ! Untrack it in your own teardown; nothing does it for you.

        Args:
            name: Unique object name; shown on the health panel.
            mocap_id: Rigid-body id in the mocap system.
            geometry: Its shape, or None for a frame only that is not an obstacle.
            touches: Ids allowed to touch it, as for `Body.touches`.
            label: Display text; empty uses the id.

        Returns:
            TrackedObject: The entry ROS callbacks keep updating. Read only.

        Raises:
            ValueError: If `name` is already tracked.
        """
        return self._monitor.track_object(name, mocap_id, geometry, touches, label)

    def untrack_object(self, name: str) -> None:
        """Stop tracking `name` and remove it from `world.tracked_objects`. Unknown names are ignored."""
        self._monitor.untrack_object(name)

    # --- --- --- --- --- LEASES AND FREEZES --- --- --- --- ---

    @contextmanager
    def lease_robots(self, serials: Iterable[str], label: str) -> Iterator[Lease]:
        """Lease robots while the `with` block runs: this block, and tasks spawned in it, may command them.

        - ! Every motion command and controller switch needs the robot's lease; stopping and holding do not.
        - Commands from anywhere else raise NotLeased. Giving the lease back sends nothing: hold what still moves.
        - Hold the lease as long as the motion runs, not only while sending: wait for it inside the block.

        Args:
            serials: The robots to lease, all or none.
            label: What the lease is for, after this plugin's name in messages, e.g. "wiggle 0804 left".

        Raises:
            NotLeased: If any of them is leased already; none is leased then.
            KeyError: If a serial is not a connected robot.

        Example:
            >>> with ctx.lease_robots(["0804"], "wiggle"):
            ...     arm.send_joint_move(target)
            ...     await ctx.wait_until(lambda: not arm.state.is_executing, timeout_s=10.0)
        """
        with leased(self._lease_slots(serials), f"{self.name}: {label}") as lease:
            yield lease

    def take_lease(self, serials: Iterable[str], label: str) -> Lease:
        """Lease robots across ticks, until `lease.release()` or this plugin stops; send under `with lease.use():`.

        For leases no single task spans, e.g. teleop that holds a robot while its buttons are in use.

        Args:
            serials: The robots to lease, all or none.
            label: What the lease is for, after this plugin's name in messages.

        Raises:
            NotLeased: If any of them is leased already; none is leased then.
            KeyError: If a serial is not a connected robot.
        """
        lease = take_lease(self._lease_slots(serials), f"{self.name}: {label}")
        self._leases = {kept for kept in self._leases if kept.active} | {lease}
        return lease

    def _lease_slots(self, serials: Iterable[str]) -> list[LeaseSlot]:
        """The lease slots of robots `serials`; KeyError for an unknown one."""
        return [self.world.robots[serial].lease_slot for serial in serials]

    @contextmanager
    def freeze(self, basis: Basis, tolerance: Tolerance = Tolerance()) -> Iterator[Freeze]:
        """Watch `basis` while the `with` block runs; once it no longer holds, cancel the running task. Optional.

        - For a task that assumes other robots or objects stand still; leases cannot see what else moves them.
        - Checked every tick, after the plugins' updates; the reason is logged and kept in the freeze's `problem`.
        - ! Only inside a task of this plugin (`spawn`): that task is cancelled. Stopping what moves is its job.

        Args:
            basis: What must stand still, from `world.freeze.take_basis`.
            tolerance: How far each thing may drift.

        Raises:
            WorldMoved: At once, if the basis no longer holds; e.g. something moved while the plan was made.
            RuntimeError: Outside a task of this plugin.

        Example:
            >>> basis = take_basis(ctx.world, ctx.now(), robots=others, bases=[serial])
            >>> with ctx.freeze(basis) as freeze:
            ...     await move()
        """
        task = self._own_task("freeze")
        freeze = Freeze(basis, tolerance)
        problem = basis_problem(basis, self.world, self.now(), tolerance)
        if problem is not None:
            freeze.problem = problem
            raise WorldMoved(problem)
        self._freezes[freeze] = task
        try:
            yield freeze
        finally:
            self._freezes.pop(freeze, None)

    @contextmanager
    def clearance(self, serials: Iterable[str], margin: float = STOP_DISTANCE) -> Iterator[Clearance]:
        """While the `with` block runs, stop robots that come too close to anything in the scene. Optional.

        - Checked against the whole scene: obstacles, process bodies, tracked objects and the other robots. Allowed
          contacts (`Body.touches`, what the robot holds) are skipped. A lost base pose trips it too.
        - A trip soft stops that robot (Resume needed), cancels the running task, and is kept in `problem` and `hit`,
          and in `clearance_trips` for every plugin.
        - Checked on a worker, a tick or two behind: keep `margin` above what the robot covers meanwhile.
        - ! Only inside a task of this plugin (`spawn`). Not an emergency stop.

        Args:
            serials: The robots to keep clear, e.g. the one this task drives.
            margin: Distance below which it trips, metres.

        Raises:
            RuntimeError: Outside a task of this plugin.

        Example:
            >>> with ctx.clearance([serial]) as clearance:
            ...     await follow(path)
        """
        task = self._own_task("clearance")
        clearance = Clearance(tuple(serials), margin)
        self._clearances[clearance] = task
        try:
            yield clearance
        finally:
            self._clearances.pop(clearance, None)

    @property
    def clearance_trips(self) -> tuple[ClearanceTrip, ...]:
        """tuple[ClearanceTrip, ...]: The last clearance trips of every plugin, oldest first; replaced on each trip."""
        return self._monitor.clearance_trips

    def _own_task(self, what: str) -> asyncio.Task:
        """The running task, which must be one of this plugin's; RuntimeError otherwise."""
        try:
            task = asyncio.current_task()
        except RuntimeError:  # no event loop runs
            task = None
        if task not in self._tasks:
            raise RuntimeError(f"plugin {self.name!r}: {what}() runs only inside a task of this plugin")
        return task

    # --- --- --- --- --- TRACES --- --- --- --- ---

    def trace(self, *signals: Signal, max_samples: int = MAX_SAMPLES) -> Trace:
        """Record `signals` from now on, until `untrace` or this plugin stops; for live plots.

        Tick signals are sampled after every plugin's update and task step, message signals on each message.

        Args:
            signals: What to record, tick and message signals in any mix, e.g. from `plugin_api/signals.py`.
            max_samples: Cap of each trigger's samples; past it the oldest is dropped for each new one.

        Returns:
            Trace: The samples, growing each tick. Show it with `ui.trace_plot.TracePlot`.

        Example:
            >>> self._trace = ctx.trace(signals.joints(arm), max_samples=600)
        """
        trace = Trace(signals, max_samples)
        listeners = [(event, partial(self._take, trace, event)) for event in trace.events]
        for event, listener in listeners:
            event.attach(listener)
        self._traces[trace] = listeners
        return trace

    def untrace(self, trace: Trace) -> None:
        """Stop recording `trace`; it keeps its samples. Unknown traces are ignored."""
        for event, listener in self._traces.pop(trace, ()):
            event.detach(listener)

    @contextmanager
    def record(self, *signals: Signal, max_samples: int = MAX_SAMPLES) -> Iterator[Trace]:
        """Record `signals` while the `with` block runs, as `trace`; it stops even if the task is cancelled.

        ! Open the block before what it should record starts, e.g. before sending a path: it cannot look back.

        Example:
            >>> with ctx.record(signals.joints(arm), signals.follower_command(robot, every_message=True)) as rec:
            ...     await ctx.sleep(5.0)
            >>> rec.save(path)
        """
        trace = self.trace(*signals, max_samples=max_samples)
        try:
            yield trace
        finally:
            self.untrace(trace)

    # --- --- --- --- --- SCHEDULING --- --- --- --- ---

    def submit(self, label: str, run: Callable[[], Any]) -> None:
        """Queue `run` to happen on the main thread, at the start of this plugin's next step.

        ! The only safe way to act from a viser callback (another thread). A returned coroutine is spawned as a task.
        """
        self._intents.put(Intent(label=label, run=run))

    def defer(self, label: str, run: Callable[[], Any], operator_only: bool = False) -> Callable[..., Awaitable[None]]:
        """Wrap `run` as a thread-safe viser callback, for widgets whose value it does not need.

        `operator_only` as in `defer_value`.

        Returns:
            Callable[..., Awaitable[None]]: A callback that submits `run`.

        Example:
            >>> with view.ui() as gui:
            ...     button = gui.add_button("Plan movement")
            >>> button.on_click(ctx.defer("plan movement", self.start_planning))
        """
        return self.defer_value(label, lambda _value: run(), operator_only)

    def defer_value(self, label: str, run: Callable[[object], Any],
                    operator_only: bool = False) -> Callable[..., Awaitable[None]]:
        """Wrap `run` as a thread-safe viser callback that is handed the widget's new value.

        ! Use this value, not `widget.value` inside `run`, which could see a later change.
        * `operator_only`: skip changes the plugin made itself (`widget.value = ...`), which viser also calls back for,
          without a client; only an operator's change submits `run`.

        ? `async` so viser calls it in order on its own loop, not from its thread pool.

        Returns:
            Callable[..., Awaitable[None]]: A callback that submits `run(value)`.

        Example:
            >>> grip = gui.add_button_group("Grip", ["Open", "Close"])
            >>> grip.on_click(ctx.defer_value("grip", lambda label: self.grip(label)))
        """

        async def _callback(event: viser.GuiEvent) -> None:
            if operator_only and event.client is None:
                return
            value = event.target.value
            self.submit(label, lambda: run(value))

        return _callback

    def spawn(self, label: str, work: Coroutine[Any, Any, T], survives_soft_stop: bool = False) -> asyncio.Task[T]:
        """Start `work` as a task owned by this plugin.

        - Cancelled on soft stop (unless `survives_soft_stop`), plugin stop and shutdown.
        - Raising counts as a failure of this plugin; MotionRefused (soft stop, no lease, a broken freeze) does not.

        Args:
            label: Human-readable name, for logs.
            work: The coroutine, e.g. `self._stream_cartesian_move(ctx, serial, arm, position, orientation)`.
            survives_soft_stop: Keep running through a soft stop, e.g. loading, drawing or writing a report.
                ! Only for work that never commands a robot: it would raise RobotStopped, or move it after Resume.

        Returns:
            asyncio.Task: Handle to cancel, check or await.

        Raises:
            RuntimeError: If this plugin was stopped or torn down.
        """
        if self._closed:
            work.close()  # never started; closing it avoids a "never awaited" warning
            raise RuntimeError(f"plugin {self.name!r} is stopped; it cannot start {label!r}")
        name = f"{self.name}: {label}"
        task = asyncio.get_running_loop().create_task(self._monitor.timer.timed_task(name, work), name=name)
        self._tasks.add(task)
        if survives_soft_stop:
            self._survivors.add(task)
        task.add_done_callback(self._task_finished)
        return task

    async def next_tick(self) -> None:
        """Wait for the next tick, after this tick's measurements and every `update`.

        Tasks woken by one tick resume in plugin dependency order, before the tick draws.
        """
        waiter = asyncio.get_running_loop().create_future()
        self._tick_waiters.append(waiter)
        await waiter

    async def wait_until(self, predicate: Callable[[], bool], timeout_s: float | None = None,
                         description: str = "condition") -> None:
        """Wait until `predicate` holds, checking once per tick. Returns at once if it already does.

        Args:
            predicate: Checked on the main thread after each tick's `update`s. Keep it cheap.
            timeout_s: Seconds of ROS time before giving up, or None. ! Prefer one: silent hardware hangs unnoticed.
            description: Names the wait in the timeout message.

        Raises:
            WaitTimeout: If timeout_s elapses first.
        """
        deadline = None if timeout_s is None else self.now() + timeout_s
        while not predicate():
            if deadline is not None and self.now() >= deadline:
                raise WaitTimeout(f"timed out after {timeout_s}s waiting for {description}")
            await self.next_tick()

    async def sleep(self, seconds: float) -> None:
        """Wait `seconds` of ROS time, to tick resolution.

        ? Not `asyncio.sleep`, which counts wall time and ignores bag playback.
        """
        until = self.now() + seconds
        await self.wait_until(lambda: self.now() >= until)

    async def run_in_thread(self, function: Callable[..., T], *args: Any, **kwargs: Any) -> T:
        """Run a slow computation on a worker thread and wait for its result.

        - ! `function` must not touch world state, the live scene, kinematics, ROS or viser; hand it copies
          such as `scene.snapshot`.
        - ! Cancelling abandons the result, but the thread runs to the end.

        Returns:
            Whatever `function(*args, **kwargs)` returns. Its exception is raised here.
        """
        return await asyncio.get_running_loop().run_in_executor(None, partial(function, *args, **kwargs))

    async def ros(self, future: RosFuture) -> Any:
        """Wait for an rclpy future, from `call_async` or `send_goal_async`.

        ! Cancelling the awaiting task does not cancel the request or the goal.

        Returns:
            The future's result: the service response, or the goal handle.
        """
        return await ros_future(future)

    async def join(self, task: asyncio.Task[T]) -> T:
        """Wait for a task another plugin owns, without taking over its cancellation.

        ? Shielded: cancelling the waiter does not cancel the task.

        Returns:
            The task's result. Its exception, or CancelledError, is raised here.
        """
        return await asyncio.shield(task)

    def _task_finished(self, task: asyncio.Task) -> None:
        """Forget a finished task, and log it if it raised. Runs on the loop."""
        self._tasks.discard(task)
        self._survivors.discard(task)
        if task.cancelled():
            return
        error = task.exception()
        if isinstance(error, MotionRefused):
            self.log_warn(f"task {task.get_name()!r} ended: {error}")
        elif error is not None:
            self._task_failures += 1
            self.log_error(f"task {task.get_name()!r} failed:\n{describe_exception(error)}")

    # --- --- --- --- --- DRIVEN BY THE MONITOR --- --- --- --- ---
    # ! Only the monitor calls these. They log each error and report once; the monitor decides what it costs.

    def _drain_intents(self) -> None:
        """Run the intents queued when the drain starts; later ones wait a tick.

        - Every intent runs, even if an earlier one raised. MotionRefused is logged, and counts as no failure.
        - Running tasks are not paused. An intent that invalidates one must cancel it.

        Raises:
            RuntimeError: If one or more intents raised.
        """
        failed = False
        for _ in range(self._intents.qsize()):
            try:
                intent = self._intents.get_nowait()
            except queue.Empty:
                break
            try:
                result = intent.run()
                if asyncio.iscoroutine(result):
                    self.spawn(intent.label, result)
            except MotionRefused as refused:
                self.log_warn(f"intent {intent.label!r} ended: {refused}")
            except Exception:
                failed = True
                self.log_error(f"intent {intent.label!r} failed:\n{traceback.format_exc()}")
        if failed:
            raise RuntimeError("one or more intents failed")

    def _check_freezes(self) -> None:
        """Cancel the task of every freeze whose basis no longer holds, logging why."""
        now = self.now()
        for freeze, task in list(self._freezes.items()):
            problem = basis_problem(freeze.basis, self.world, now, freeze.tolerance)
            if problem is not None:
                freeze.problem = problem
                del self._freezes[freeze]
                self.log_warn(f"task {task.get_name()!r}: {problem}; cancelling it")
                task.cancel(f"freeze broken: {problem}")

    def _trip_clearance(self, clearance: Clearance, serial: str, why: str, hit: CollisionHit | None = None) -> None:
        """Soft stop robot `serial` and cancel the clearance's task, unless it ended meanwhile; tell the monitor."""
        task = self._clearances.pop(clearance, None)
        if task is None:
            return
        clearance.problem = f"robot {serial} {why}"
        clearance.hit = None if hit is None else replace(hit, where=task.get_name())
        self._monitor.clearance_tripped(ClearanceTrip(self.now(), serial, task.get_name(), clearance.problem,
                                                      clearance.hit))
        self.world.robots[serial].soft_stop()
        self.log_error(f"task {task.get_name()!r}: {clearance.problem}; soft stopped it and cancelling the task")
        task.cancel(f"clearance: {clearance.problem}")

    def _wake_tick_waiters(self) -> None:
        """Resume the tasks waiting in `next_tick`. They run once the tick yields."""
        waiters, self._tick_waiters = self._tick_waiters, []
        for waiter in waiters:
            if not waiter.done():  # done means its task was cancelled meanwhile
                waiter.set_result(None)

    def _sample_traces(self, now: float) -> None:
        """Sample every trace's tick signals at time `now`.

        Raises:
            Exception: Whatever a signal raised; later traces miss this tick.
        """
        if not self._traces:
            return
        with self._monitor.timer.part(f"plugin {self.name!r} traces"):
            for trace in list(self._traces):
                trace.sample(now)

    def _take(self, trace: Trace, event: Any, record: Any) -> None:
        """Record one message's record in `trace`, inside the ROS callback; a signal that raises skips it."""
        try:
            trace.take(event, record)
        except Exception:
            self._signal_failures += 1
            self._signal_failure = traceback.format_exc()

    def _take_signal_failures(self) -> str | None:
        """The last traceback of a message signal that raised since the last call, or None if none did."""
        if not self._signal_failures:
            return None
        failures, self._signal_failures = self._signal_failures, 0
        return f"{failures} messages skipped; the last:\n{self._signal_failure}"

    def _take_task_failures(self) -> int:
        """How many tasks raised since the last call."""
        failures, self._task_failures = self._task_failures, 0
        return failures

    def _cancel_for_soft_stop(self) -> list[str]:
        """Cancel every running task but those spawned with `survives_soft_stop`, and return their names."""
        tasks = [task for task in self._tasks if task not in self._survivors]
        for task in tasks:
            task.cancel()
        return [task.get_name() for task in tasks]

    def _cancel_all_tasks(self) -> list[asyncio.Task]:
        """Cancel every running task, and return them so the caller can wait for their cleanup."""
        tasks = list(self._tasks)
        for task in tasks:
            task.cancel()
        return tasks

    def _close(self) -> list[asyncio.Task]:
        """Refuse new tasks from now on, stop recording traces, give back `take_lease` leases, cancel the tasks.

        Returns:
            list[asyncio.Task]: The cancelled tasks, which may still be cleaning up.
        """
        self._closed = True
        for trace in list(self._traces):
            self.untrace(trace)
        for lease in self._leases:
            lease.release()
        self._leases.clear()
        return self._cancel_all_tasks()
