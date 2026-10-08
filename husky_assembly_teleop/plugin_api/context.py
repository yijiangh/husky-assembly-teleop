"""
The plugin contract: what a plugin is handed, and what it is allowed to touch.

Each plugin gets its own PluginContext, so its queue, tasks and UI are disposed of with it.

! Import only `concurrency` and `trace` at runtime, the rest under TYPE_CHECKING: `plugin` and `monitor` import this
  module.
"""

from __future__ import annotations

import asyncio
import queue
import traceback
from contextlib import contextmanager
from dataclasses import dataclass
from functools import partial
from typing import TYPE_CHECKING, Any, Awaitable, Callable, Coroutine, Iterator, TypeVar

from .concurrency import WaitTimeout, describe_exception, ros_future
from .trace import MAX_SAMPLES, Signal, Trace

T = TypeVar("T")

if TYPE_CHECKING:
    import viser
    from rclpy.task import Future as RosFuture

    from ..config import MonitorConfig
    from bar_assembly_core.design_io.geometry import Geometry
    from ..monitor import HuskyMonitor
    from ..ui.visualization import PluginView
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
        # Tasks that raised since the monitor last asked; see _take_task_failures.
        self._task_failures = 0
        # Futures of the tasks waiting for the next tick; see next_tick.
        self._tick_waiters: list[asyncio.Future] = []
        # Set once the plugin is stopped or torn down; no new tasks after that.
        self._closed = False
        # Traces sampled every tick; see trace.
        self._traces: list[Trace] = []

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

    # --- --- --- --- --- TRACES --- --- --- --- ---

    def trace(self, *signals: Signal, max_samples: int = MAX_SAMPLES) -> Trace:
        """Sample `signals` once per tick from now on, until `untrace` or this plugin stops; for live plots.

        Sampled after every plugin's update and task step, stamped with the tick's ROS time.

        Args:
            signals: What to sample, e.g. from `world/signals.py`.
            max_samples: Cap; past it the oldest sample is dropped for each new one.

        Returns:
            Trace: The samples, growing each tick. Show it with `ui.trace_plot.TracePlot`.

        Example:
            >>> self._trace = ctx.trace(signals.joints(arm), max_samples=600)
        """
        trace = Trace(signals, max_samples)
        self._traces.append(trace)
        return trace

    def untrace(self, trace: Trace) -> None:
        """Stop sampling `trace`; it keeps its samples. Unknown traces are ignored."""
        if trace in self._traces:
            self._traces.remove(trace)

    @contextmanager
    def record(self, *signals: Signal, max_samples: int = MAX_SAMPLES) -> Iterator[Trace]:
        """Sample `signals` once per tick while the `with` block runs; it stops even if the task is cancelled.

        Example:
            >>> with ctx.record(signals.joints(arm), signals.force(arm)) as rec:
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

    def defer(self, label: str, run: Callable[[], Any]) -> Callable[..., Awaitable[None]]:
        """Wrap `run` as a thread-safe viser callback, for widgets whose value it does not need.

        Returns:
            Callable[..., Awaitable[None]]: A callback that submits `run`.

        Example:
            >>> with view.ui() as gui:
            ...     button = gui.add_button("Plan movement")
            >>> button.on_click(ctx.defer("plan movement", self.start_planning))
        """
        return self.defer_value(label, lambda _value: run())

    def defer_value(self, label: str, run: Callable[[object], Any]) -> Callable[..., Awaitable[None]]:
        """Wrap `run` as a thread-safe viser callback that is handed the widget's new value.

        ! Use this value, not `widget.value` inside `run`, which could see a later change.

        ? `async` so viser calls it in order on its own loop, not from its thread pool.

        Returns:
            Callable[..., Awaitable[None]]: A callback that submits `run(value)`.

        Example:
            >>> grip = gui.add_button_group("Grip", ["Open", "Close"])
            >>> grip.on_click(ctx.defer_value("grip", lambda label: self.grip(label)))
        """

        async def _callback(event: viser.GuiEvent) -> None:
            value = event.target.value
            self.submit(label, lambda: run(value))

        return _callback

    def spawn(self, label: str, work: Coroutine[Any, Any, T]) -> asyncio.Task[T]:
        """Start `work` as a task owned by this plugin.

        Cancelled on soft stop, plugin stop and shutdown; raising counts as a failure of this plugin.

        Args:
            label: Human-readable name, for logs.
            work: The coroutine, e.g. `self._execute_move(ctx, arm, target)`.

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
        if task.cancelled():
            return
        error = task.exception()
        if error is not None:
            self._task_failures += 1
            self.log_error(f"task {task.get_name()!r} failed:\n{describe_exception(error)}")

    # --- --- --- --- --- DRIVEN BY THE MONITOR --- --- --- --- ---
    # ! Only the monitor calls these. They log each error and report once; the monitor decides what it costs.

    def _drain_intents(self) -> None:
        """Run the intents queued when the drain starts; later ones wait a tick.

        - Every intent runs, even if an earlier one raised.
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
            except Exception:
                failed = True
                self.log_error(f"intent {intent.label!r} failed:\n{traceback.format_exc()}")
        if failed:
            raise RuntimeError("one or more intents failed")

    def _wake_tick_waiters(self) -> None:
        """Resume the tasks waiting in `next_tick`. They run once the tick yields."""
        waiters, self._tick_waiters = self._tick_waiters, []
        for waiter in waiters:
            if not waiter.done():  # done means its task was cancelled meanwhile
                waiter.set_result(None)

    def _sample_traces(self, now: float) -> None:
        """Sample every trace at time `now`.

        Raises:
            Exception: Whatever a signal raised; later traces miss this tick.
        """
        if not self._traces:
            return
        with self._monitor.timer.part(f"plugin {self.name!r} traces"):
            for trace in list(self._traces):
                trace.sample(now)

    def _take_task_failures(self) -> int:
        """How many tasks raised since the last call."""
        failures, self._task_failures = self._task_failures, 0
        return failures

    def _cancel_all_tasks(self) -> list[asyncio.Task]:
        """Cancel every running task, and return them so the caller can wait for their cleanup."""
        tasks = list(self._tasks)
        for task in tasks:
            task.cancel()
        return tasks

    def _close(self) -> list[asyncio.Task]:
        """Refuse new tasks from now on, stop sampling traces and cancel the running tasks. Returns them."""
        self._closed = True
        self._traces.clear()
        return self._cancel_all_tasks()
