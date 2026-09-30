"""
The ROS2 node that owns everything and runs the tick.

Responsibilities, and nothing beyond them:

  - program lifecycle: bring the pieces up, run, tear them down again
  - own the world state, the PyBullet scene and the viser server
  - hold the robot interfaces, and keep the PyBullet robots in step with them
  - drive each plugin: drain its queue, call its update, step its jobs
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
        ctx: Its context: UI slice, intent queue and jobs. Kept beside the
            plugin rather than on it, so the plugin's own state cannot collide
            with monitor-driven internals, and it is built before the context
            exists. Plugins receive it as a hook argument and never store it.
        stopped: Set once it failed in setup or too often. A stopped plugin is
            no longer stepped or drawn, but stays loaded until shutdown.
        errors: Consecutive ticks in which it raised.
        last_error_tick: Index of the tick it last raised in.
        last_slow_warning: ROS time of the last slow-step complaint.
    """

    plugin: HuskyPlugin
    ctx: PluginContext
    stopped: bool = False
    errors: int = 0
    last_error_tick: int = -1
    last_slow_warning: float = 0.0

    @property
    def name(self) -> str:
        """str: The plugin's unique name."""
        return self.plugin.name


class HuskyMonitor(Node):
    """The monitor node. Implements MonitorServices for its plugins.

    ! Threading. Two threads, and the boundary between them is crucial for correctness.

      1. The ROS thread. rclpy.spin is single-threaded, so the tick timer and
         every subscription are serialised. All state, all PyBullet and all plugin code lives here
         and needs no locks because of it.
      2. viser's threads. A server thread, plus a 32-worker pool for GUI and
         scene-click callbacks. Neither PyBullet nor our state is thread-safe,
         so those callbacks may only call PluginContext.submit and return.

      submit() is the only crossing point: a plugin's queue is drained at the
      start of its own step, so an intent runs as single-threaded code.
    """

    def __init__(self):
        """Bring up state, scene, UI and plugins, then start the tick.
        """
        super().__init__("husky_monitor")

        self._config = config_from_ros_parameters(self)
        self._world = WorldState()
        self._loaded: dict[str, _LoadedPlugin] = {}

        # Monotonic tick counter.
        self._tick_index = 0

        self._scene: RobotScene | None = None
        self._viz: Visualization | None = None
        self._tick_timer = None
        self._shut_down = False

        try:
            self._build()
        except Exception:
            # Release what was already acquired. (PyBullet for example)
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

        # Create plugins in dependency order
        for plugin in load_plugins(self._config.enabled_plugins, self.log_error):
            ctx = PluginContext(
                name=plugin.name,
                services=self,
                view=self._viz.view_for(plugin.name),
                scene=self._scene,
                dependencies=plugin.requires,
            )
            self._loaded[plugin.name] = _LoadedPlugin(plugin=plugin, ctx=ctx)

        # Plugins stay loaded until shutdown. One that fails in setup is
        # stopped straight away rather than run against half-built state.
        for loaded in self._loaded.values():
            try:
                with self._scene.active():
                    loaded.plugin.setup(loaded.ctx)
            except Exception:
                self.log_error(f"plugin {loaded.name!r} failed in setup:\n{traceback.format_exc()}")
                self._stop(loaded)

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

        # Soft stop has higest priority: it must run before any plugin can command new actions.
        if self._viz.take_stop_request():
            self._soft_stop()

        # 1. Mirror measurements into PyBullet before any plugin runs. Kinematics and collision query need to be up to date.
        self._scene.sync_real(self._world.robot_states())

        # 2. Step each plugin, in dependency order
        for loaded in self._loaded.values():
            if not loaded.stopped:
                self._step_plugin(loaded)

        # 3. Draw everything
        #  ! While frozen, plugins dont render to allow for text selection.
        #    Robot state still gets drawn even when frozen.
        with self._viz.atomic():
            self._viz.draw(self._world)
            for loaded in self._loaded.values() if not self._viz.frozen else ():
                if loaded.stopped:
                    continue
                try:
                    with self._scene.active():
                        loaded.plugin.draw(loaded.ctx)
                except Exception:
                    self._plugin_failed(loaded, "draw")

    def _soft_stop(self) -> None:
        """Stop every robot and cancel every plugin job.

        ! In the core, not a plugin: a stop must not depend on which plugins are
          loaded, and only the core reaches every plugin's jobs.
        """
        self.log_warn("SOFT STOP: stopping every robot and cancelling every plugin job")
        for robot in self._world.robots.values():
            robot.soft_stop()
        for loaded in self._loaded.values():
            loaded.ctx._cancel_all_jobs()

    def _step_plugin(self, loaded: _LoadedPlugin) -> None:
        """Give one plugin its turn: intents, update, then jobs.

        ! Each part is caught on its own, so a failure in one still lets the
          others run this tick.

        ! Inside `scene.active()`, so plugin code can
          call `pp` without bracketing it. Per plugin to improve isolation between plugins.
        """
        started = time.perf_counter()
        with self._scene.active():
            # Intents first, so work handed in from the UI is applied before
            # the update that reacts to it runs.
            try:
                loaded.ctx._drain_intents()
            except Exception:
                self._plugin_failed(loaded, "intents")
            try:
                loaded.plugin.update(loaded.ctx)
            except Exception:
                self._plugin_failed(loaded, "update")
            try:
                loaded.ctx._pump_jobs()
            except Exception:
                self._plugin_failed(loaded, "jobs")

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

    def _plugin_failed(self, loaded: _LoadedPlugin, hook: str) -> None:
        """Log that a plugin raised in `hook`, and stop it if it keeps doing so.

        Args:
            loaded: The plugin whose work raised.
            hook: Which part was running, for the log message.
        """
        self.log_error(f"plugin {loaded.name!r} failed in {hook}:\n{traceback.format_exc()}")
        if loaded.last_error_tick == self._tick_index:
            return  # already counted this tick
        # Consecutive only if it also failed in the tick immediately before.
        consecutive = loaded.last_error_tick == self._tick_index - 1
        loaded.errors = loaded.errors + 1 if consecutive else 1
        loaded.last_error_tick = self._tick_index

        if loaded.errors >= self._config.max_plugin_errors:
            self._stop(loaded)

    def _stop(self, loaded: _LoadedPlugin) -> None:
        """Stop stepping and drawing a failing plugin, so it cannot flood the log.

        ! Nothing else happens: its UI, jobs and dependents stay as they are
          until shutdown. A failing plugin is a bug to fix, not to recover from.
        """
        loaded.stopped = True
        self.log_error(f"stopped plugin {loaded.name!r}; it will not run again until restart")

    def _teardown(self, loaded: _LoadedPlugin) -> None:
        """Cancel a plugin's jobs, run its teardown hook and take its UI away.

        ! No waiting for jobs to finish: rclpy no longer spins at shutdown, so a
          job waiting on the robot would never see it move. Each cancelled job
          gets one step, so its cleanup runs up to its first wait.
        """
        loaded.ctx._cancel_all_jobs()
        with self._scene.active():
            try:
                loaded.ctx._pump_jobs()
            except Exception:
                self.log_error(f"plugin {loaded.name!r} raised while cancelling its "
                               f"jobs:\n{traceback.format_exc()}")
            if loaded.ctx._jobs:
                self.log_warn(f"plugin {loaded.name!r} left {len(loaded.ctx._jobs)} job(s) "
                              f"unfinished at shutdown; dropping them")
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
        for loaded in reversed(self._loaded.values()):
            self._teardown(loaded)
        if self._viz is not None:
            self._viz.stop()
        if self._scene is not None:
            self._scene.disconnect()


# --- --- --- --- --- MAIN --- --- --- --- ---
def main(args: list[str] | None = None) -> None:
    """Run the monitor until interrupted, reading sys.argv when `args` is None.

    Everything is configured through ROS parameters; see config.py.
    """
    rclpy.init(args=args)
    monitor = None
    try:
        monitor = HuskyMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        if monitor is not None:
            monitor.shutdown()
            monitor.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
