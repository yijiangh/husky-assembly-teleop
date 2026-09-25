"""
The plugin contract: what a plugin is handed, and what it is allowed to touch.

! No runtime imports beyond concurrency.
  `plugin` imports this module, so importing it back would be a cycle; the rest
  are under TYPE_CHECKING for consistency with it. Never import `monitor`,
  `visualization` or `robot_scene` here either -- they import this module.
  Plugins depend on an interface, the monitor provides it, neither reaches
  around the other.

? One context per plugin, not one shared one.
  `submit`, `spawn` and `view` are already scoped to their owner, so a plugin
  cannot queue work onto another, and tearing one down disposes of exactly its
  own queue, jobs and UI. Nothing has to be filtered by owner afterwards.

Why the old arrangement (the whole monitor as an untyped first argument) had to
go: doc/refactor_rationale.md.
"""

from __future__ import annotations

import queue
from contextlib import AbstractContextManager
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Protocol

from .concurrency import Job, Task

if TYPE_CHECKING:
    import viser

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
    def scene(self) -> "viser.SceneApi":
        """viser.SceneApi: Scene api. Paths must start with scene_root."""
        ...

    @property
    def gui(self) -> "viser.GuiApi":
        """viser.GuiApi: GUI api. Prefer `ui()`, which also parents to the folder."""
        ...

    def ui(self) -> AbstractContextManager["viser.GuiApi"]:
        """Context manager placing new widgets in this plugin's folder."""
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
        #: every tick, and nothing else.
        #:
        #: ! Raw access, on purpose. `scene.client_id` and `scene.robots[serial]`
        #:   are real PyBullet ids; call `p` and `pp` with them directly, and
        #:   bracket pp free functions with `with ctx.scene.active():`.
        #:
        #: ! Whatever a plugin loads, it removes in its own teardown. Nothing
        #:   tracks it. That is the trade for having no wrapper.
        self.scene = scene

        self._services = services
        self._dependencies = dependencies

        # Filled from viser threads, drained on the ROS thread at the start of
        # this plugin's step. queue.Queue is the thread-safe handoff.
        self._intents: queue.Queue[Intent] = queue.Queue()

        # This plugin's running jobs, owned here so that disabling the plugin
        # disposes of them with it.
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
        """Get a plugin this one declared a dependency on.

        ! Only declared dependencies resolve. The declaration is what gives the
          monitor a load order and lets it refuse a cycle; reaching an
          undeclared plugin would be the old "everything touches everything"
          with extra steps.

        Raises:
            KeyError: If `name` was not declared in `requires`, or is not loaded.
        """
        if name not in self._dependencies:
            raise KeyError(f"plugin {self.name!r} did not declare a dependency on {name!r}; "
                           f"add it to `requires`")
        return self._services.plugin(name)

    # --- --- --- --- --- SCHEDULING --- --- --- --- ---

    def submit(self, label: str, run: Callable[[], None]) -> None:
        """Queue `run` onto this plugin's queue, to happen on the ROS thread.

        Safe from any thread. Drained at the start of this plugin's next step,
        before its run loop is resumed. `label` names the work in the error
        message if it raises.

        ! The only safe way to act on a viser callback. Those fire on a
          32-worker pool while everything else runs on the ROS thread, and
          neither PyBullet nor our state is thread-safe. `defer` wraps this up.
        """
        self._intents.put(Intent(label=label, run=run))

    def defer(self, label: str, run: Callable[[], None]) -> Callable[..., None]:
        """Wrap `run` as a viser callback that is safe to register.

        Returns:
            Callable[..., None]: A callback that accepts and ignores whatever
                event argument viser passes it, and submits `run`.

        Example:
            >>> with view.ui() as gui:
            ...     button = gui.add_button("Plan movement")
            >>> button.on_click(ctx.defer("plan movement", self.start_planning))
        """

        def _callback(*_event: object) -> None:
            self.submit(label, run)

        return _callback

    def spawn(self, label: str, task: Task) -> Job:
        """Start `task` as a cancellable job, advanced one step per tick.

        For anything longer than a tick: planning, trajectory execution, a
        calibration sweep. Runs alongside this plugin's loop, and is cancelled
        if the plugin is disabled.

        Returns:
            Job: Handle for cancelling it and checking whether it finished.
        """
        job = Job(label, task)
        self._jobs.append(job)
        return job

    # --- --- --- --- --- DRIVEN BY THE MONITOR --- --- --- --- ---
    # ! Private because the monitor, which owns this object, is the only caller.
    #   A plugin draining its own queue mid-step would re-enter its own step.
    #
    # ! None of these report their own failures. They raise, and the monitor's
    #   `_guard` decides what a plugin failure costs. Two error policies in two
    #   places is how the first draft got a counter that could never fire.

    def _drain_intents(self) -> None:
        """Run every intent queued since this plugin's last step.

        ! Only the items present when the drain starts. Work queued by an
          intent, or by a viser callback firing mid-drain, waits for the next
          tick; otherwise a callback that re-queues itself would spin here and
          the tick would never finish.

        ! An intent can land while a job is mid-sequence. The intent runs, the
          job gets no say. An intent that invalidates a running job -- loading a
          new action while the old one executes -- must cancel it explicitly.

        Raises:
            RuntimeError: If an intent raised. The drain stops there; intents
                still queued run on the next tick.
        """
        for _ in range(self._intents.qsize()):
            try:
                intent = self._intents.get_nowait()
            except queue.Empty:
                break
            try:
                intent.run()
            except Exception as failure:
                raise RuntimeError(f"intent {intent.label!r} failed") from failure

    def _pump_jobs(self) -> None:
        """Advance each running job one step and drop the ones that finished.

        Raises:
            RuntimeError: If a job raised. Jobs after it wait for the next tick;
                finished jobs are dropped either way.
        """
        try:
            for job in self._jobs:
                job.step()
        finally:
            self._jobs = [job for job in self._jobs if not job.done]

    def _cancel_all_jobs(self) -> None:
        """Ask every running job to stop. Used when the plugin is torn down."""
        for job in self._jobs:
            job.cancel()
