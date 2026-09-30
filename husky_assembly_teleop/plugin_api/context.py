"""
The plugin contract: what a plugin is handed, and what it is allowed to touch.

Each plugin gets its own PluginContext, so its queue, tasks and UI are scoped to
it and disposed of with it.

! Imports only `concurrency` at runtime; everything else is under TYPE_CHECKING,
  because `plugin`, `monitor` and `visualization` import this module.
"""

from __future__ import annotations

import asyncio
import queue
import traceback
from contextlib import AbstractContextManager
from dataclasses import dataclass
from functools import partial
from typing import TYPE_CHECKING, Any, Awaitable, Callable, Coroutine, Protocol, TypeVar

import viser
from rclpy.task import Future as RosFuture

from .concurrency import WaitTimeout, describe_exception, ros_future

T = TypeVar("T")

if TYPE_CHECKING:
    from ..config import MonitorConfig
    from ..world.geometry import Geometry
    from ..world.kinematics import Kinematics
    from .plugin import HuskyPlugin
    from ..world.scene import PluginScene
    from ..world.measured import TrackedObject, WorldState


@dataclass(frozen=True)
class Intent:
    """Work handed in from a viser thread, to run on the ROS thread.

    Attributes:
        label: Human-readable name, used when the work raises. "plan movement M2".
        run: The work. Called with no arguments, once, from the owning plugin's
            step. If it is an `async def`, the coroutine is spawned as a task
            under `label`; any other return value is ignored.
    """

    label: str
    run: Callable[[], Any]


class PluginView(Protocol):
    """A plugin's private scene subtree and GUI folder in viser.

    Plugins get one of these rather than the raw ViserServer, which would put
    every plugin back into one flat namespace. Implemented in ui/visualization.py.
    """

    @property
    def scene_root(self) -> str:
        """str: Scene path owned by this plugin, e.g. "/plugins/calibration"."""
        ...

    @property
    def scene(self) -> viser.SceneApi:
        """viser.SceneApi: Scene api. Paths must start with scene_root."""
        ...

    @property
    def gui(self) -> viser.GuiApi:
        """viser.GuiApi: GUI api. Prefer `ui()`, which also parents to the folder."""
        ...

    def ui(self) -> AbstractContextManager[viser.GuiApi]:
        """Context manager placing new widgets in this plugin's folder."""
        ...

    def panel(self) -> viser.PanelHandle:
        """Create a panel owned by this plugin: a window beside the main one.

        ! Always use this instead of `gui.add_panel()`: only panels made here are
          removed when the plugin is torn down.
        """
        ...

    def clear(self) -> None:
        """Remove every scene node and widget this plugin created."""
        ...


class MonitorServices(Protocol):
    """What the node provides to every plugin. Implemented by HuskyMonitor.

    Plugins use PluginContext, which wraps this; it exists to keep PluginContext
    from importing the monitor.
    """

    @property
    def config(self) -> "MonitorConfig":
        """MonitorConfig: Read-only run configuration."""
        ...

    @property
    def world(self) -> "WorldState":
        """WorldState: Measured reality."""
        ...

    def now(self) -> float:
        """Seconds from the node clock. Use instead of time.time(), so waits
        behave under simulated time and bag playback."""
        ...

    def log_info(self, message: str) -> None:
        """Log `message` at info level."""
        ...

    def log_warn(self, message: str) -> None:
        """Log `message` at warning level."""
        ...

    def log_error(self, message: str) -> None:
        """Log `message` at error level."""
        ...

    def plugin(self, name: str) -> "HuskyPlugin":
        """Look up another loaded plugin by name.

        Raises:
            KeyError: If no plugin by that name is loaded.
        """
        ...

    def track_object(self, name: str, mocap_id: int, geometry: "Geometry | None" = None,
                     touches: tuple[str, ...] = (), label: str = "") -> "TrackedObject":
        """Register a tracked object, describe it in the scene, and subscribe to its mocap pose."""
        ...

    def untrack_object(self, name: str) -> None:
        """Unsubscribe a tracked object and remove it from the registry."""
        ...


class PluginContext:
    """One plugin's handle on the monitor, and its own runtime bookkeeping.

    The monitor keeps one per plugin and uses it to drive that plugin; the
    plugin uses it to reach everything outside itself.
    """

    def __init__(self, name: str, services: MonitorServices, view: PluginView,
                 scene: "PluginScene", kinematics: "Kinematics", dependencies: tuple[str, ...] = ()):
        """Create a plugin's context.

        Args:
            name: The owning plugin's name.
            services: The monitor.
            view: The plugin's private scene subtree and GUI folder.
            scene: The collision scene, limited to this plugin's own ids.
            kinematics: Robot base, joint and link poses, fixed for this tick.
            dependencies: Names this plugin declared in `requires`. Only these
                resolve through `require`.
        """
        self.name = name

        #: This plugin's corner of the viser UI. Removed wholesale on teardown.
        self.view = view

        #: The collision scene (world/scene.py). Put your bodies under "<plugin name>/";
        #: the core draws them and removes them when this plugin closes.
        #: `scene.snapshot` is the whole world copied at the start of this tick:
        #: hand that, never the live bodies, to a worker thread.
        self.scene = scene

        #: Each robot's base pose, joints and link poses, fixed for this tick.
        #: ! Main thread only.
        self.kinematics = kinematics

        self._services = services
        self._dependencies = dependencies

        # Filled from viser threads, drained on the main thread at the start of
        # this plugin's step. queue.Queue is the thread-safe handoff.
        self._intents: queue.Queue[Intent] = queue.Queue()

        # This plugin's running tasks. The set also keeps them alive: asyncio
        # holds only a weak reference, so an unreferenced task can vanish mid-run.
        self._tasks: set[asyncio.Task] = set()
        # Tasks that raised since the monitor last asked; see _take_task_failures.
        self._task_failures = 0
        # Futures of the tasks waiting for the next tick; see next_tick.
        self._tick_waiters: list[asyncio.Future] = []
        # Set once the plugin is stopped or torn down; no new tasks after that.
        self._closed = False

    # --- --- --- --- --- STATE --- --- --- --- ---

    @property
    def config(self) -> "MonitorConfig":
        """MonitorConfig: Read-only run configuration."""
        return self._services.config

    @property
    def world(self) -> "WorldState":
        """WorldState: Measured reality. Read only; ROS callbacks write it."""
        return self._services.world

    def now(self) -> float:
        """Current ROS time in seconds."""
        return self._services.now()

    def log_info(self, message: str) -> None:
        """Log `message` at info level, tagged with this plugin's name."""
        self._services.log_info(f"[{self.name}] {message}")

    def log_warn(self, message: str) -> None:
        """Log `message` at warning level, tagged with this plugin's name."""
        self._services.log_warn(f"[{self.name}] {message}")

    def log_error(self, message: str) -> None:
        """Log `message` at error level, tagged with this plugin's name."""
        self._services.log_error(f"[{self.name}] {message}")

    def require(self, name: str) -> "HuskyPlugin":
        """Get a plugin this one declared in `requires`. Undeclared ones do not resolve.

        Raises:
            KeyError: If `name` was not declared in `requires`, or is not loaded.
        """
        if name not in self._dependencies:
            raise KeyError(f"plugin {self.name!r} did not declare a dependency on {name!r}; "
                           f"add it to `requires`")
        return self._services.plugin(name)

    def track_object(self, name: str, mocap_id: int, geometry: "Geometry | None" = None,
                     touches: tuple[str, ...] = (), label: str = "") -> "TrackedObject":
        """Start tracking mocap rigid body `mocap_id` as `world.tracked_objects[name]`, id "tracked/<name>".

        The core draws it and puts it in every snapshot once it has a fix.

        ! Call from a hook, intent or task, never from a viser callback. Untrack
          what you track in your own teardown; nothing does it for you.

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
        return self._services.track_object(name, mocap_id, geometry, touches, label)

    def untrack_object(self, name: str) -> None:
        """Stop tracking `name` and remove it from `world.tracked_objects`. Unknown names are ignored."""
        self._services.untrack_object(name)

    # --- --- --- --- --- SCHEDULING --- --- --- --- ---

    def submit(self, label: str, run: Callable[[], Any]) -> None:
        """Queue `run` to happen on the main thread, at the start of this plugin's next step.

        ! The only safe way to act from a viser callback, which runs on another
          thread. Safe from any thread. `label` names the work if it raises.

        `run` may be an `async def`: it is then started as a task under `label`.
        """
        self._intents.put(Intent(label=label, run=run))

    def defer(self, label: str, run: Callable[[], Any]) -> Callable[..., Awaitable[None]]:
        """Wrap `run` as a thread-safe viser callback, for widgets whose value it does not need.

        Returns:
            Callable[..., Awaitable[None]]: A callback that ignores viser's event
                argument and submits `run`.

        Example:
            >>> with view.ui() as gui:
            ...     button = gui.add_button("Plan movement")
            >>> button.on_click(ctx.defer("plan movement", self.start_planning))
        """
        return self.defer_value(label, lambda _value: run())

    def defer_value(self, label: str, run: Callable[[object], Any]) -> Callable[..., Awaitable[None]]:
        """Wrap `run` as a thread-safe viser callback that is handed the widget's new value.

        ! The value is captured when viser reports the change. Reading
          `widget.value` inside `run` could see a later change instead.

        ? `async`, so viser calls it in order on its event loop rather than from
          its thread pool, where the value might be read late. That is viser's
          loop on viser's thread, not ours: it only queues the intent.

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

    def spawn(self, label: str, work: Coroutine[Any, Any, T]) -> "asyncio.Task[T]":
        """Start `work` as a task owned by this plugin.

        For anything longer than a tick: planning, trajectory execution, a sweep.
        The task is cancelled on soft stop, when the plugin is stopped, and at
        shutdown. If it raises, that counts as a failure of this plugin.

        Args:
            label: Human-readable name, for logs.
            work: The coroutine, e.g. `self._execute_move(ctx, arm, target)`.

        Returns:
            asyncio.Task: Handle to cancel it (`task.cancel()`), check `task.done()`,
                or await its result.

        Raises:
            RuntimeError: If this plugin was stopped or torn down.
        """
        if self._closed:
            work.close()  # never started; closing it avoids a "never awaited" warning
            raise RuntimeError(f"plugin {self.name!r} is stopped; it cannot start {label!r}")
        task = asyncio.get_running_loop().create_task(work, name=f"{self.name}: {label}")
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
            timeout_s: Give up after this many seconds of ROS time, or None to wait forever.
                ! Prefer a timeout: a wait on silent hardware otherwise hangs unnoticed.
            description: Names the wait in the timeout message.

        Raises:
            WaitTimeout: If timeout_s elapses first. An `asyncio.TimeoutError`.
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

        ! `function` must not touch world state, the live scene, kinematics, ROS or
          viser: it runs beside the main thread. Hand it copies of what it needs,
          such as `scene.snapshot`.
        ! A thread cannot be interrupted. Cancelling the awaiting task abandons the
          result, but the thread runs to the end.

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

    async def join(self, task: "asyncio.Task[T]") -> T:
        """Wait for a task another plugin owns, without taking over its cancellation.

        ? Awaiting a task directly would cancel it when the awaiting task is
          cancelled. Shielded, the owner alone decides when its work stops.

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
    # ! Private: only the monitor calls these. On failure they log each error
    #   and report once, so the monitor alone decides what a failure costs.

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
        """Refuse new tasks from now on and cancel the running ones. Returns them."""
        self._closed = True
        return self._cancel_all_tasks()
