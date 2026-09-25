"""
The ROS2 node that owns everything and runs the tick.

Responsibilities, and nothing beyond them:

  - program lifecycle: bring the pieces up, run, tear them down again
  - own the world state, the PyBullet scene and the viser server
  - hold the robot interfaces, and keep the PyBullet robots in step with them
  - drive each plugin: drain its queue, resume its loop, step its jobs
  - run the tick in a fixed order, and be the place that order is written down

! No feature code here. Every experiment, panel and diagnostic is a plugin with
  its own state. What happens otherwise: doc/refactor_rationale.md.
"""

from __future__ import annotations

import time
import traceback
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from .concurrency import Task
from .config import MonitorConfig, config_from_ros_parameters
from .context import PluginContext
from .plugin import HuskyPlugin, load_plugins
from .robot_interface import HuskyRobotInterface
from .robot_scene import RobotScene
from .visualization import Visualization
from .world_state import WorldState

@dataclass
class _LoadedPlugin:
    """One plugin and everything the monitor tracks about it.

    Attributes:
        plugin: The plugin instance.
        ctx: Its context: UI slice, intent queue and jobs.
        runner: Its run loop. Set once, at setup, and never replaced: a run loop
            is expected to last the whole program, so there is no state in which
            a loaded plugin has no runner.
        errors: Consecutive ticks in which it raised.
        last_error_tick: Index of the tick it last raised in.
        last_slow_warning: ROS time of the last slow-step complaint.
    """

    plugin: HuskyPlugin
    ctx: PluginContext
    runner: Task
    errors: int = 0
    last_error_tick: int = -1
    last_slow_warning: float = 0.0

    @property
    def name(self) -> str:
        """str: The plugin's unique name."""
        return self.plugin.name


class HuskyMonitor(Node):
    """The monitor node. Implements MonitorServices for its plugins.

    ! Threading. Two threads, and the boundary between them is the single most
      important thing in this design.

      1. The ROS thread. rclpy.spin is single-threaded, so the tick timer and
         every subscription -- mocap included, it is an ordinary topic now --
         are serialised. All state, all PyBullet and all plugin code lives here
         and needs no locks because of it.
      2. viser's threads. A server thread, plus a 32-worker pool for GUI and
         scene-click callbacks. Neither PyBullet nor our state is thread-safe,
         so those callbacks may only call PluginContext.submit and return.

      submit() is the only crossing point: a plugin's queue is drained at the
      start of its own step, so an intent runs as single-threaded code.
    """

    def __init__(self):
        """Bring up state, scene, UI and plugins, then start the tick."""
        super().__init__("husky_monitor")

        self._config = config_from_ros_parameters(self)
        self._world = WorldState()
        self._loaded: dict[str, _LoadedPlugin] = {}

        # Counts ticks. Only used to tell "raised again this tick" from "raised
        # again next tick" when counting a plugin's consecutive failures.
        self._tick_index = 0

        # Set before anything can fail, so a construction that dies partway has
        # something to tear down instead of raising AttributeError on the way.
        self._scene: RobotScene | None = None
        self._viz: Visualization | None = None
        self._tick_timer = None
        self._shut_down = False

        try:
            self._build()
        except Exception:
            # Release what was already acquired. Leaking the PyBullet connection
            # is the one that bites: the next run finds a stale server.
            self.shutdown()
            raise

    def _build(self) -> None:
        """Construct the scene, the UI and the plugins. Called once, by __init__."""
        self._scene = RobotScene(log_warn=self.log_warn, use_gui=False)
        self._viz = Visualization(port=self._config.viser_port)

        for robot_config in self._config.robots:
            self._world.add_robot(HuskyRobotInterface(self, robot_config))
            self._scene.load_robot(robot_config)
            self._viz.load_robot(robot_config)

        # load_plugins returns dependency order and dicts keep insertion order,
        # so walking self._loaded is always that order: a plugin is set up,
        # stepped and drawn after everything it requires.
        for plugin in load_plugins(self._config.enabled_plugins, self.log_error):
            ctx = PluginContext(
                name=plugin.name,
                services=self,
                view=self._viz.view_for(plugin.name),
                scene=self._scene,
                dependencies=plugin.requires,
            )
            # The run loop is created here, before setup, so that `runner` is
            # never None. Creating a generator does not execute any of its body,
            # so nothing runs until the first tick resumes it.
            self._loaded[plugin.name] = _LoadedPlugin(
                plugin=plugin, ctx=ctx, runner=plugin.run(ctx))

        # Every record exists before any setup runs, so a plugin whose setup
        # fails can cascade its disable onto the plugins that require it.
        for loaded in tuple(self._loaded.values()):
            if self._loaded.get(loaded.name) is not loaded:
                continue  # already gone: a dependency of this one failed to set up
            try:
                loaded.plugin.setup(loaded.ctx)
            except Exception:
                # Never start a plugin that could not set itself up, and do not
                # bother counting towards the error budget: one strike is enough
                # when the plugin never got off the ground.
                self.log_error(f"disabling plugin {loaded.name!r}, which failed in setup:\n"
                               f"{traceback.format_exc()}")
                self._disable(loaded)

        # Created last, so the first tick cannot fire against a half-built node.
        self._tick_timer = self.create_timer(self._config.tick_period, self._tick)

    # --- --- --- --- --- MonitorServices --- --- --- --- ---

    @property
    def config(self) -> MonitorConfig:
        """MonitorConfig: Read-only run configuration."""
        return self._config

    @property
    def world(self) -> WorldState:
        """WorldState: Measured reality. Written only by ROS callbacks."""
        return self._world

    def now(self) -> float:
        """Seconds from the node clock, so waits behave under bag playback."""
        return self.get_clock().now().nanoseconds * 1e-9

    def log_info(self, message: str) -> None:
        """Log `message` at info level."""
        self.get_logger().info(message)

    def log_warn(self, message: str) -> None:
        """Log `message` at warning level."""
        self.get_logger().warning(message)

    def log_error(self, message: str) -> None:
        """Log `message` at error level."""
        self.get_logger().error(message)

    def plugin(self, name: str) -> HuskyPlugin:
        """Look up a loaded plugin by name.

        Raises:
            KeyError: If no plugin by that name is loaded.
        """
        try:
            return self._loaded[name].plugin
        except KeyError:
            raise KeyError(f"plugin {name!r} is not loaded") from None

    # --- --- --- --- --- TICK --- --- --- --- ---

    def _tick(self) -> None:
        """One tick. The order below is deliberate; see the comment on each step."""
        self._tick_index += 1

        # 1. Mirror measurements into PyBullet before any plugin runs, so
        #    kinematics and collision queries answer against this tick's reality
        #    rather than the previous one's.
        self._scene.sync_real(self._world.robot_states())

        # 2. Step each plugin, in dependency order. Anything in the scene that is
        #    not a live robot was put there by a plugin, which poses and removes
        #    it itself -- the core neither tracks nor understands it.
        for loaded in tuple(self._loaded.values()):
            self._step_plugin(loaded)

        # 3. Draw, batched into one update so a browser never renders a frame
        #    where the robot has moved but the bar it holds has not.
        with self._viz.atomic():
            self._viz.draw(self._world)
            for loaded in tuple(self._loaded.values()):
                if self._loaded.get(loaded.name) is not loaded:
                    continue  # disabled during step 2; its UI is already gone
                try:
                    loaded.plugin.draw(loaded.ctx)
                except Exception:
                    self._plugin_failed(loaded, "draw")

    def _step_plugin(self, loaded: _LoadedPlugin) -> None:
        """Give one plugin its turn: its intents, its loop, then its jobs.

        ! One try/except for the whole step, on purpose.
          The three parts are not independently recoverable -- if draining
          intents raised, resuming the loop would run against half-applied state
          -- so there is nothing to gain from catching them separately, and a
          single handler means a single error counter.

        ! A run loop that ends is a bug, not a state to support.
          A plugin is expected to run for the whole program. `run` should loop
          forever and wait by yielding; a sequential plugin that finishes its
          sweep goes back to waiting for the next trigger rather than returning.
          So StopIteration is reported like any other failure, which also means
          a plugin whose loop keeps ending is eventually disabled instead of
          sitting there loaded and never advancing.
        """
        started = time.perf_counter()
        try:
            # Intents first, so work handed in from the UI is applied before the
            # loop that reacts to it runs.
            loaded.ctx._drain_intents()
            next(loaded.runner)
            loaded.ctx._pump_jobs()
        except StopIteration:
            self._plugin_failed(loaded, "run", "its run loop returned; a run loop must "
                                               "keep going for the lifetime of the program")
        except Exception:
            self._plugin_failed(loaded, "step")

        self._warn_if_slow(loaded, time.perf_counter() - started)

    def _warn_if_slow(self, loaded: _LoadedPlugin, elapsed: float) -> None:
        """Complain when a plugin's step (`elapsed` seconds) ate too much budget.

        "Wait by yielding, never by blocking" is only enforceable if breaking it
        is noisy; otherwise it shows up as a sticky UI and nobody knows which
        plugin is responsible.
        """
        if elapsed <= self._config.tick_period * self._config.slow_step_warn_ratio:
            return
        now = self.now()
        if now - loaded.last_slow_warning < self._config.slow_step_warn_period:
            return
        loaded.last_slow_warning = now
        self.log_warn(f"plugin {loaded.name!r} took {elapsed * 1e3:.0f} ms of the "
                      f"{self._config.tick_period * 1e3:.0f} ms tick; "
                      f"it should yield more often")

    # --- --- --- --- --- ERRORS --- --- --- --- ---

    def _plugin_failed(self, loaded: _LoadedPlugin, hook: str, reason: str = "") -> None:
        """Record that a plugin raised, and disable it if it keeps doing so.

        Without containment here, one plugin raising aborts the tick and every
        plugin after it silently stops running, with no symptom beyond "the
        panel froze".

        `errors` counts consecutive *ticks* with a failure, not failures. The
        tick stamp is what makes that work: a second failure in the same tick --
        step and then draw -- does not count twice, and a clean tick in between
        starts the count over without anyone having to sweep the plugin list to
        reset it.

        Args:
            loaded: The plugin whose work raised.
            hook: Which part was running, for the log message.
            reason: Explanation to log instead of the traceback, for failures
                where the traceback says nothing useful.
        """
        if loaded.last_error_tick == self._tick_index:
            return  # already counted this tick
        # Consecutive only if it also failed in the tick immediately before.
        consecutive = loaded.last_error_tick == self._tick_index - 1
        loaded.errors = loaded.errors + 1 if consecutive else 1
        loaded.last_error_tick = self._tick_index

        detail = reason or traceback.format_exc()
        self.log_error(f"plugin {loaded.name!r} failed in {hook} "
                       f"({loaded.errors}/{self._config.max_plugin_errors}):\n{detail}")
        if loaded.errors >= self._config.max_plugin_errors:
            self.log_error(f"disabling plugin {loaded.name!r} after repeated failures")
            self._disable(loaded)

    def _disable(self, loaded: _LoadedPlugin) -> None:
        """Take a failing plugin out of the tick, with anything that requires it.

        ! Disabling cascades. A plugin that declared a dependency may assume it
          is there, so leaving its dependents running would move the failure
          somewhere harder to read.

        The failure path, and it says so in the log; shutdown calls `_teardown`
        directly. Calling this for a plugin that is already gone does nothing.
        """
        if self._loaded.get(loaded.name) is not loaded:
            return

        dependents = [other for other in tuple(self._loaded.values())
                      if other is not loaded and loaded.name in other.plugin.requires]
        for dependent in dependents:
            self.log_error(f"disabling plugin {dependent.name!r}, "
                           f"which requires {loaded.name!r}")
            self._disable(dependent)

        self._teardown(loaded)

    def _teardown(self, loaded: _LoadedPlugin) -> None:
        """Stop a plugin's jobs, run its teardown hook and take its UI away.

        The mechanics only: no cascade, nothing logged as a failure, so shutdown
        can use it for plugins that are perfectly healthy.
        """
        if self._loaded.pop(loaded.name, None) is None:
            return

        loaded.ctx._cancel_all_jobs()
        # Cancelled jobs get a few more steps, so cleanup that has to wait --
        # releasing a gripper, re-enabling a controller -- runs before the
        # plugin's UI disappears from under it.
        for _ in range(self._config.max_cleanup_steps):
            if not loaded.ctx._jobs:
                break
            try:
                loaded.ctx._pump_jobs()
            except Exception:
                self.log_error(f"plugin {loaded.name!r} raised while cancelling its "
                               f"jobs:\n{traceback.format_exc()}")
        if loaded.ctx._jobs:
            self.log_error(f"plugin {loaded.name!r} left {len(loaded.ctx._jobs)} job(s) "
                           f"unfinished after cancellation; dropping them")

        try:
            loaded.plugin.teardown(loaded.ctx)
        except Exception:
            self.log_error(f"plugin {loaded.name!r} failed in teardown:\n"
                           f"{traceback.format_exc()}")
        loaded.ctx.view.clear()

    # --- --- --- --- --- SHUTDOWN --- --- --- --- ---

    def shutdown(self) -> None:
        """Tear everything down, in the reverse order it was built.

        ! Must actually run on Ctrl-C. viser's server thread and the PyBullet
          client both outlive a bare rclpy.shutdown(), so main() calls this from
          a finally block.

        Safe to call twice, and safe when construction failed partway, which is
        how __init__ cleans up after itself.
        """
        if self._shut_down:
            return
        self._shut_down = True

        if self._tick_timer is not None:
            self._tick_timer.cancel()
        # Reverse dependency order, so a plugin goes before what it depends on.
        for loaded in reversed(tuple(self._loaded.values())):
            self._teardown(loaded)
        if self._viz is not None:
            self._viz.stop()
        if self._scene is not None:
            self._scene.disconnect()


# --- --- --- --- --- MAIN --- --- --- --- ---
def main(args: list[str] | None = None) -> None:
    """Run the monitor until interrupted, reading sys.argv when `args` is None."""
    rclpy.init(args=args)
    monitor = None
    try:
        # Inside the try: a constructor that fails after starting viser or
        # PyBullet still has to reach the cleanup below.
        monitor = HuskyMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        if monitor is not None:
            monitor.shutdown()
            monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
