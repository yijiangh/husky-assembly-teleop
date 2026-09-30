"""
The ROS2 node that owns everything and runs the tick.

Responsibilities, and nothing beyond them:

  - program lifecycle: bring the pieces up, run, tear them down again
  - own the world state, the kinematics, the collision scene and the viser server
  - hold the robot interfaces, and copy the whole world once per tick
  - drive each plugin: drain its queue, call its update, wake its tasks
  - run the tick in a fixed order, and be the place that order is written down

! No feature code here. Every experiment, panel and diagnostic is a plugin with
  its own state. What happens otherwise: doc/refactor_rationale.md.
"""

from __future__ import annotations

import asyncio
import signal
import time
import traceback
from dataclasses import dataclass

import rclpy
from crl_husky_msgs.msg import MocapRigidBodyPose
from rclpy.executors import SingleThreadedExecutor, TimeoutException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from .plugin_api.concurrency import LoopWatchdog
from .config import MonitorConfig, config_from_ros_parameters
from .plugin_api.context import PluginContext
from .world.geometry import Geometry
from .world.kinematics import Kinematics
from .world.mocap import mocap_topic, store_sample
from .plugin_api.plugin import HuskyPlugin, load_plugins
from .robot_interface import HuskyRobotInterface
from .robot_interface.connections import RosConnections
from .world.scene import PluginScene, Scene, TrackedDescription
from .ui.visualization import Visualization
from .world.measured import TrackedObject, WorldState

#: Most ROS callbacks run in one tick. Beyond it the rest wait a tick, so a
#: flood of messages cannot starve the plugins.
ROS_CALLBACKS_PER_TICK = 5000


@dataclass
class _LoadedPlugin:
    """One plugin and everything the monitor tracks about it.

    Attributes:
        plugin: The plugin instance.
        ctx: Its context: UI slice, intent queue and tasks. Kept beside the
            plugin rather than on it, so the plugin's own state cannot collide
            with monitor-driven internals, and it is built before the context
            exists. Plugins receive it as a hook argument and never store it.
        stopped: Set once it, or a plugin it depends on, failed in setup or too
            often. A stopped plugin is no longer stepped or drawn, its tasks are
            cancelled, but it stays loaded until shutdown.
        set_up: Whether its setup was started, so teardown only runs if so.
        closed: Whether its teardown has run.
        errors: Consecutive ticks in which it raised.
        last_error_tick: Index of the tick it last raised in.
        last_slow_warning: ROS time of the last slow-step complaint.
    """

    plugin: HuskyPlugin
    ctx: PluginContext
    stopped: bool = False
    set_up: bool = False
    closed: bool = False
    errors: int = 0
    last_error_tick: int = -1
    last_slow_warning: float = 0.0

    @property
    def name(self) -> str:
        """str: The plugin's unique name."""
        return self.plugin.name


class HuskyMonitor(Node):
    """The monitor node. Implements MonitorServices for its plugins.

    ! Threading. The boundary between these is crucial for correctness.

      1. The main thread runs one asyncio loop: the tick, every ROS callback
         (pumped from inside the tick), every plugin hook and every plugin task.
         All state, the kinematics and the live scene live here and need no
         locks because of it.
      2. viser's threads. A server thread with its own asyncio loop, plus a
         worker pool for GUI and scene-click callbacks. Those callbacks may only
         call PluginContext.submit (via defer) and return.
      3. Worker threads started by PluginContext.run_in_thread or a planner's
         own executor, for computation on copies (the scene snapshot). They
         never touch shared state.

      submit() is the only crossing point from viser: a plugin's queue is
      drained at the start of its own step, so an intent runs as main-thread code.

    ! ROS callbacks run only at the start of each tick. So between ticks
      WorldState does not change, and it always matches the kinematics and
      the tick's scene snapshot.
      ? Cost: a topic faster than (queue depth x tick rate) loses messages.
        Give a subscription whose every sample matters a deeper queue.

    ! Construct it inside a running asyncio loop: plugin setup may start tasks.
    """

    def __init__(self):
        """Bring up state, scene, UI and plugins. `run` then starts ticking."""
        super().__init__("husky_monitor")

        self._config = config_from_ros_parameters(self)
        self._world = WorldState()
        self._loaded: dict[str, _LoadedPlugin] = {}
        # The mocap subscription of each tracked object, by object name.
        self._object_connections: dict[str, RosConnections] = {}

        # Monotonic tick counter.
        self._tick_index = 0

        # ? rclpy's executor, used only to find and run ready callbacks; the
        #   tick decides when. See _pump_ros.
        self._executor = SingleThreadedExecutor(context=self.context)
        self._executor.add_node(self)

        self._kinematics: Kinematics | None = None
        self._scene = Scene()
        self._viz: Visualization | None = None
        self._watchdog: LoopWatchdog | None = None
        # Why each stopped plugin stopped, by name, for the banner.
        self._stop_reasons: dict[str, str] = {}
        self._shut_down = False

        try:
            self._build()
        except Exception:
            # Release what was already acquired. (the viser server for example)
            self._release()
            raise

    def _build(self) -> None:
        """Construct the kinematics, the UI and the plugins. Called once, by __init__."""
        self._kinematics = Kinematics(self._config.robots, log_warn=self.log_warn)
        self._viz = Visualization(port=self._config.viser_port)

        for robot_config in self._config.robots:
            self._world.add_robot(HuskyRobotInterface(self, robot_config))
            self._viz.load_robot(robot_config)

        # Create plugins in dependency order
        for plugin in load_plugins(self._config.enabled_plugins, self.log_error):
            if plugin.experimental:
                self.log_warn(f"plugin {plugin.name!r} is experimental: its behaviour and API may change")
            ctx = PluginContext(
                name=plugin.name,
                services=self,
                view=self._viz.view_for(plugin.name),
                scene=PluginScene(self._scene, plugin.name,
                                  log_warn=lambda message, name=plugin.name: self.log_warn(f"[{name}] {message}")),
                kinematics=self._kinematics,
                dependencies=plugin.requires,
            )
            self._loaded[plugin.name] = _LoadedPlugin(plugin=plugin, ctx=ctx)

        # Plugins stay loaded until shutdown. One that fails in setup is
        # stopped straight away, with its dependents, rather than run against
        # half-built state. A dependent stopped that way is never set up.
        for loaded in self._loaded.values():
            if loaded.stopped:
                continue
            loaded.set_up = True
            try:
                loaded.plugin.setup(loaded.ctx)
            except Exception:
                self.log_error(f"plugin {loaded.name!r} failed in setup:\n{traceback.format_exc()}")
                self._stop(loaded, "failed in setup")

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

    def track_object(self, name: str, mocap_id: int, geometry: Geometry | None = None,
                     touches: tuple[str, ...] = (), label: str = "") -> TrackedObject:
        """Register a tracked object, describe it in the scene, and subscribe to its mocap pose.

        Args:
            name: Unique object name.
            mocap_id: Rigid-body id in the mocap system.
            geometry: Its shape, or None for a frame only.
            touches: Ids allowed to touch it.
            label: Display text; empty uses the id.

        Returns:
            TrackedObject: The registry entry, updated by every mocap message.

        Raises:
            ValueError: If an object with the same name is already tracked.
        """
        if name in self._world.tracked_objects:
            raise ValueError(f"object {name!r} is already tracked")
        obj = TrackedObject(name=name, mocap_id=mocap_id)
        connections = RosConnections(self)
        # * Stored exactly like a robot base's pose.
        connections.subscription(MocapRigidBodyPose, mocap_topic(mocap_id),
                                 lambda message: store_sample(obj, message, self.now()))
        if geometry is not None and not geometry.collision:
            self.log_warn(f"tracked object {name!r} has no collision meshes: it is drawn but never collides")
        self._world.tracked_objects[name] = obj
        self._scene.tracked[name] = TrackedDescription(geometry=geometry, touches=tuple(touches), label=label)
        self._object_connections[name] = connections
        return obj

    def untrack_object(self, name: str) -> None:
        """Unsubscribe a tracked object and remove it from the registry. Unknown names are ignored."""
        connections = self._object_connections.pop(name, None)
        if connections is not None:
            connections.destroy_all()
        self._world.tracked_objects.pop(name, None)
        self._scene.tracked.pop(name, None)

    # --- --- --- --- --- RUN --- --- --- --- ---

    async def run(self) -> None:
        """Tick every `tick_period` until Ctrl-C or SIGTERM.

        ! A tick that runs late is not made up with a burst of fast ones.
        """
        loop = asyncio.get_running_loop()
        stop = asyncio.Event()
        signals = (signal.SIGINT, signal.SIGTERM)
        for number in signals:
            loop.add_signal_handler(number, stop.set)

        # Stalls of this task are plugin hooks, which _step_plugin times itself.
        tick_task = asyncio.current_task()
        self._watchdog = LoopWatchdog(
            loop, limit=self._config.tick_period * self._config.slow_step_warn_ratio,
            repeat_after=self._config.slow_step_warn_period, log_warn=self.log_warn,
            ignore=lambda: tick_task)
        try:
            due = loop.time()
            while not stop.is_set():
                await self._tick()
                due = max(due + self._config.tick_period, loop.time())
                await asyncio.sleep(due - loop.time())
        finally:
            # * A second Ctrl-C during shutdown now interrupts it.
            for number in signals:
                loop.remove_signal_handler(number)
            self._watchdog.stop()
            self.log_info("stopping: cancelling plugin tasks and tearing down")

    # --- --- --- --- --- TICK --- --- --- --- ---

    async def _tick(self) -> None:
        """One tick. The order below is deliberate; see the comment on each step."""
        self._tick_index += 1

        # 1. Run every ROS callback that is waiting, so the world is as fresh as it gets.
        self._pump_ros()

        # 2. Soft stop before any plugin can command new actions.
        if self._viz.take_stop_request():
            self._soft_stop()

        # 3. Fix forward kinematics for this tick, before anything reads a link pose.
        self._kinematics.update(self._world)

        # 4. Copy the whole world before any plugin runs: one pump's measurements
        #    plus every plugin's complete writes of the previous tick. Planners
        #    and the 3D view read this copy, never the live scene.
        snapshot = self._scene.take_snapshot(self._world, self._kinematics, self._tick_index, self.now())

        # 5. Step each plugin, in dependency order: intents, then update.
        for loaded in self._loaded.values():
            if not loaded.stopped:
                self._step_plugin(loaded)

        # 6. Resume the tasks waiting for this tick, in dependency order. Yielding
        #    once lets each take its step before the draw.
        #    ? Stopped plugins too: their cancelled tasks may still be cleaning up.
        for loaded in self._loaded.values():
            loaded.ctx._wake_tick_waiters()
        await asyncio.sleep(0)

        # 7. Draw everything
        #  ! While frozen, plugins dont render to allow for text selection.
        #    Robots and the scene still get drawn even when frozen.
        with self._viz.atomic():
            self._viz.draw(snapshot)
            for loaded in self._loaded.values() if not self._viz.frozen else ():
                if loaded.stopped:
                    continue
                try:
                    loaded.plugin.draw(loaded.ctx)
                except Exception:
                    self._plugin_failed(loaded, "draw")

    def _pump_ros(self) -> None:
        """Run every ROS callback that is ready: subscriptions, service answers, timers.

        ! Guarded callbacks (RosConnections) log their own failures; anything
          else that raises is logged here and dropped, like a guarded one.
        """
        for _ in range(ROS_CALLBACKS_PER_TICK):
            try:
                handler, _entity, _node = self._executor.wait_for_ready_callbacks(timeout_sec=0.0)
            except TimeoutException:
                return  # nothing more is ready
            handler()
            if handler.exception() is not None:
                self.log_error(f"ROS callback failed:\n{handler.exception()!r}")
        self.log_warn(f"more than {ROS_CALLBACKS_PER_TICK} ROS callbacks were ready in one tick; "
                      f"the rest run next tick")

    def _soft_stop(self) -> None:
        """Stop every robot and cancel every plugin task.

        ! In the core, not a plugin: a stop must not depend on which plugins are
          loaded, and only the core reaches every plugin's tasks.
        """
        self.log_warn("SOFT STOP: stopping every robot and cancelling every plugin task")
        for robot in self._world.robots.values():
            robot.soft_stop()
        for loaded in self._loaded.values():
            loaded.ctx._cancel_all_tasks()

    def _step_plugin(self, loaded: _LoadedPlugin) -> None:
        """Give one plugin its turn: count failed tasks, run intents, then update.

        ! Each part is caught on its own, so a failure in one still lets the
          others run this tick, unless it got the plugin stopped.
        """
        started = time.perf_counter()
        if loaded.ctx._take_task_failures():
            self._plugin_failed(loaded, "a task", details="")
        # Intents first, so work handed in from the UI is applied before
        # the update that reacts to it runs.
        if not loaded.stopped:
            try:
                loaded.ctx._drain_intents()
            except Exception:
                self._plugin_failed(loaded, "intents")
        if not loaded.stopped:
            try:
                loaded.plugin.update(loaded.ctx)
            except Exception:
                self._plugin_failed(loaded, "update")

        self._warn_if_slow(loaded, time.perf_counter() - started)

    def _warn_if_slow(self, loaded: _LoadedPlugin, elapsed: float) -> None:
        """Complain when a plugin's step (`elapsed` seconds) ate too much budget.

        "Wait by awaiting, never by blocking" is only enforceable if breaking it
        is noisy; otherwise it shows up as a sticky UI and nobody knows which
        plugin is responsible. Tasks are watched by LoopWatchdog instead.
        """
        if elapsed <= self._config.tick_period * self._config.slow_step_warn_ratio:
            return
        now = self.now()
        if now - loaded.last_slow_warning < self._config.slow_step_warn_period:
            return
        loaded.last_slow_warning = now
        self.log_warn(f"plugin {loaded.name!r} took {elapsed * 1e3:.0f} ms of the "
                      f"{self._config.tick_period * 1e3:.0f} ms tick; "
                      f"move slow work into a task")

    # --- --- --- --- --- ERRORS --- --- --- --- ---

    def _plugin_failed(self, loaded: _LoadedPlugin, hook: str, details: str | None = None) -> None:
        """Log that a plugin raised in `hook`, and stop it if it keeps doing so.

        Args:
            loaded: The plugin whose work raised.
            hook: Which part was running, for the log message.
            details: What to log with it. None logs the exception being handled;
                "" logs nothing more, when the details were logged already.
        """
        if details is None:
            details = traceback.format_exc()
        self.log_error(f"plugin {loaded.name!r} failed in {hook}" + (f":\n{details}" if details else ""))
        if loaded.last_error_tick == self._tick_index:
            return  # already counted this tick
        # Consecutive only if it also failed in the tick immediately before.
        consecutive = loaded.last_error_tick == self._tick_index - 1
        loaded.errors = loaded.errors + 1 if consecutive else 1
        loaded.last_error_tick = self._tick_index

        if loaded.errors >= self._config.max_plugin_errors:
            self._stop(loaded, f"failed in {hook}, {loaded.errors} ticks in a row")

    def _stop(self, loaded: _LoadedPlugin, reason: str) -> None:
        """Stop a failing plugin and every plugin that depends on it, and show the monitor as broken.

        ! Stopping cancels their tasks, so their cleanup runs, and nothing else:
          their UI stays until shutdown. A stopped plugin is a bug to fix and
          restart for, not something to recover from.

        Args:
            loaded: The plugin that failed.
            reason: Why, for the banner and the log.
        """
        to_stop = {loaded.name: reason}
        # Plugins are in dependency order, so one pass also catches dependents of dependents.
        for other in self._loaded.values():
            if other.stopped or other.name in to_stop:
                continue
            broken = [name for name in other.plugin.requires if name in to_stop]
            if broken:
                to_stop[other.name] = f"stopped because it depends on {broken[0]!r}"

        for name, why in to_stop.items():
            stopping = self._loaded[name]
            stopping.stopped = True
            stopping.ctx._close()
            self._stop_reasons[name] = why
            self.log_error(f"STOPPED plugin {name!r}: {why}. The monitor is broken; restart it.")
        self._viz.show_broken(self._stop_reasons)

    # --- --- --- --- --- SHUTDOWN --- --- --- --- ---

    async def shutdown(self) -> None:
        """Cancel every plugin's tasks, let them clean up, then tear everything down.

        Plugins go in reverse dependency order, so a plugin's cleanup can still
        use what it depends on. ROS keeps running meanwhile, so cleanup can
        send a final hold and see it take effect.

        Safe to call twice.
        """
        if self._shut_down:
            return
        self._shut_down = True

        keep_alive = asyncio.get_running_loop().create_task(self._keep_alive(), name="shutdown ticks")
        try:
            for loaded in reversed(self._loaded.values()):
                tasks = loaded.ctx._close()
                if tasks:
                    _done, pending = await asyncio.wait(tasks, timeout=self._config.shutdown_grace)
                    if pending:
                        self.log_warn(f"plugin {loaded.name!r} has {len(pending)} task(s) still "
                                      f"cleaning up after {self._config.shutdown_grace} s; "
                                      f"tearing down without them")
                self._close(loaded)
        finally:
            keep_alive.cancel()
            self._release()

    async def _keep_alive(self) -> None:
        """A reduced tick during shutdown: ROS, kinematics, the world copy and tick waiters, no hooks."""
        while True:
            self._pump_ros()
            self._tick_index += 1
            self._kinematics.update(self._world)
            self._scene.take_snapshot(self._world, self._kinematics, self._tick_index, self.now())
            for loaded in self._loaded.values():
                loaded.ctx._wake_tick_waiters()
            await asyncio.sleep(self._config.tick_period)

    def _close(self, loaded: _LoadedPlugin) -> None:
        """Run a plugin's teardown hook, if it was set up, and take its UI away. Once."""
        if loaded.closed:
            return
        loaded.closed = True
        if loaded.set_up:
            try:
                loaded.plugin.teardown(loaded.ctx)
            except Exception:
                self.log_error(f"plugin {loaded.name!r} failed in teardown:\n"
                               f"{traceback.format_exc()}")
        loaded.ctx.view.clear()
        # * Its bodies go with it, so they stop being obstacles for everyone else.
        self._scene.remove_prefix(f"{loaded.name}/")

    def _release(self) -> None:
        """Release plugins, UI and the executor, in reverse build order.

        ! Must actually run on Ctrl-C. viser's server thread outlives a bare
          rclpy.shutdown(), so main() makes sure.

        Safe when construction failed partway, which is how __init__ cleans up after itself.
        """
        for loaded in reversed(self._loaded.values()):
            loaded.ctx._close()
            self._close(loaded)
        if self._viz is not None:
            self._viz.stop()
        self._executor.shutdown()


# --- --- --- --- --- MAIN --- --- --- --- ---
async def _run_monitor() -> None:
    """Build the monitor, tick until stopped, then shut it down cleanly."""
    monitor = HuskyMonitor()
    try:
        await monitor.run()
    finally:
        await monitor.shutdown()
        monitor.destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run the monitor until interrupted, reading sys.argv when `args` is None.

    Everything is configured through ROS parameters; see config.py.
    """
    # ! No rclpy signal handler: it would shut ROS down on Ctrl-C before plugin
    #   tasks could send their final commands. The monitor handles Ctrl-C itself.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        asyncio.run(_run_monitor())
    except KeyboardInterrupt:
        pass  # Ctrl-C during startup, or a second one during shutdown
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
