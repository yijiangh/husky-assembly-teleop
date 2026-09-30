"""
The plugin contract: what a plugin is handed, and what it is allowed to touch.

Each plugin gets its own PluginContext, so its queue, jobs and UI are scoped to
it and disposed of with it.

! Imports only `concurrency` at runtime; everything else is under TYPE_CHECKING,
  because `plugin`, `monitor`, `visualization` and `robot_scene` import this module.
"""

from __future__ import annotations

import queue
import traceback
from contextlib import AbstractContextManager
from dataclasses import dataclass
from typing import TYPE_CHECKING, Awaitable, Callable, Protocol

import viser

from .concurrency import Job, Task

if TYPE_CHECKING:
    from .config import MonitorConfig
    from .plugin import HuskyPlugin
    from .robot_scene import RobotScene
    from .world_state import WorldState


@dataclass(frozen=True)
class Intent:
    """Work handed in from a viser thread, to run on the ROS thread.

    Attributes:
        label: Human-readable name, used when the work raises. "plan movement M2".
        run: The work. Called with no arguments, once, from the owning plugin's
            step. Its return value is ignored.
    """

    label: str
    run: Callable[[], None]


class PluginView(Protocol):
    """A plugin's private scene subtree and GUI folder in viser.

    Plugins get one of these rather than the raw ViserServer, which would put
    every plugin back into one flat namespace. Implemented in visualization.py.
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


class PluginContext:
    """One plugin's handle on the monitor, and its own runtime bookkeeping.

    The monitor keeps one per plugin and uses it to drive that plugin; the
    plugin uses it to reach everything outside itself.
    """

    def __init__(self, name: str, services: MonitorServices, view: PluginView,
                 scene: "RobotScene", dependencies: tuple[str, ...] = ()):
        """Create a plugin's context.

        Args:
            name: The owning plugin's name.
            services: The monitor.
            view: The plugin's private scene subtree and GUI folder.
            scene: The shared PyBullet scene holding the live robots.
            dependencies: Names this plugin declared in `requires`. Only these
                resolve through `require`.
        """
        self.name = name

        #: This plugin's corner of the viser UI. Removed wholesale on teardown.
        self.view = view

        #: The shared PyBullet scene: the live robots, posed from measurements
        #: every tick. `scene.client_id` and `scene.robots[serial]` are raw
        #: PyBullet ids, and every hook already runs inside `scene.active()`.
        #:
        #: ! Shared: a body one plugin adds is an obstacle for every other. Remove
        #:   what you add in your own teardown; nothing tracks it for you.
        self.scene = scene

        self._services = services
        self._dependencies = dependencies

        # Filled from viser threads, drained on the ROS thread at the start of
        # this plugin's step. queue.Queue is the thread-safe handoff.
        self._intents: queue.Queue[Intent] = queue.Queue()

        # This plugin's running jobs, cancelled when it is torn down.
        self._jobs: list[Job] = []

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

    # --- --- --- --- --- SCHEDULING --- --- --- --- ---

    def submit(self, label: str, run: Callable[[], None]) -> None:
        """Queue `run` to happen on the ROS thread, at the start of this plugin's next step.

        ! The only safe way to act from a viser callback, which runs on another
          thread. Safe from any thread. `label` names the work if it raises.
        """
        self._intents.put(Intent(label=label, run=run))

    def defer(self, label: str, run: Callable[[], None]) -> Callable[..., Awaitable[None]]:
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

    def defer_value(self, label: str, run: Callable[[object], None]) -> Callable[..., Awaitable[None]]:
        """Wrap `run` as a thread-safe viser callback that is handed the widget's new value.

        ! The value is captured when viser reports the change. Reading
          `widget.value` inside `run` could see a later change instead.

        ? `async`, so viser calls it in order on its event loop rather than from
          its thread pool, where the value might be read late.

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

    def spawn(self, label: str, task: Task) -> Job:
        """Start `task` as a cancellable job, advanced one step per tick.

        For anything longer than a tick: planning, trajectory execution, a sweep.

        Returns:
            Job: Handle for cancelling it and checking whether it finished.
        """
        job = Job(label, task)
        self._jobs.append(job)
        return job

    # --- --- --- --- --- DRIVEN BY THE MONITOR --- --- --- --- ---
    # ! Private: only the monitor calls these. On failure they log each error
    #   and raise once, so the monitor alone decides what a failure costs.

    def _drain_intents(self) -> None:
        """Run the intents queued when the drain starts; later ones wait a tick.

        - Every intent runs, even if an earlier one raised.
        - Running jobs are not paused. An intent that invalidates one must cancel it.

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
                intent.run()
            except Exception:
                failed = True
                self.log_error(f"intent {intent.label!r} failed:\n{traceback.format_exc()}")
        if failed:
            raise RuntimeError("one or more intents failed")

    def _pump_jobs(self) -> None:
        """Advance each job one step, even if an earlier one raised, and drop finished ones.

        Raises:
            RuntimeError: If one or more jobs raised.
        """
        failed = False
        for job in self._jobs:
            try:
                job.step()
            except RuntimeError:
                failed = True
                self.log_error(f"job {job.label!r} failed:\n{traceback.format_exc()}")
        self._jobs = [job for job in self._jobs if not job.done]
        if failed:
            raise RuntimeError("one or more jobs failed")

    def _cancel_all_jobs(self) -> None:
        """Ask every running job to stop. Used when the plugin is torn down."""
        for job in self._jobs:
            job.cancel()
