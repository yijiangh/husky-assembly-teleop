"""
Pick a joint target for one arm, plan a collision-free path to it, scrub through the path, and commit it.

! EXPERIMENTAL: a mockup. Commit is a stub.

* Planning (bidirectional RRT in joint space, planner.py) runs on a worker thread from
  a task, so the tick never waits; Clear or Stop all ends it early. It plans against
  the tick's scene snapshot in a compas_fab world: the robot itself (SRDF and stitched
  tools), the other robots and every scene body.
! The first plan for a robot loads its models and the others' (several seconds).
! Commit is a stub that only logs (`send_to_arm`).
! A plan is only good for where the robot was when planned: if the arm or base moves
  or the target changes it goes stale, stays on screen, and Commit refuses it.
"""

from __future__ import annotations

import asyncio
import math
import threading
from concurrent.futures import ThreadPoolExecutor
from html import escape

import numpy as np
import viser

from ...config import find_robot_serial
from ...plugin_api.concurrency import timeout
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...robot_interface.arm import UR_JOINT_LIMITS
from ...ui.ghost import PATH_COLOR, TARGET_COLOR, RecentUse, RobotGhost
from ...ui.pybullet_window import PyBulletWindowToggle
from ...ui.style import BUSY, FAIL, NONE, OK, SECTION_CTRL, SECTION_TOOL, block, chip, section, values
from .planner import TIME_LIMIT, ArmPath, ArmPlanningWorld, PlanResult, arm_joint_names, plan_arm

#: The plan goes stale once a joint moves this far from the planned start, or the base this far.
STALE_JOINT = math.radians(2.0)    # rad
STALE_POSITION = 0.05              # m
#: Extra time for the first plan of a robot, which loads the models, seconds.
LOAD_ALLOWANCE = 60.0

#: Time slider resolution, seconds.
SLIDER_STEP = 0.05
#: Short joint names for the sliders, UR driver order.
SLIDER_NAMES = ("pan", "lift", "elbow", "w1", "w2", "w3")


def send_to_arm(ctx: PluginContext, serial: str, arm_name: str, path: ArmPath) -> bool:
    """Hand a path to an arm. STUB: only logs.

    TODO Resample the path evenly in time with a smooth speed profile and send it with
         `ArmInterface.send_joint_trajectory`, which checks limits, speed and start.

    Args:
        ctx: The plugin's context, for logging.
        serial: Which robot.
        arm_name: Which arm.
        path: The path to follow.

    Returns:
        bool: Whether the arm took the path. Always True for the stub.
    """
    ctx.log_warn(f"[STUB] would send a {len(path.waypoints)}-waypoint path ({path.duration:.1f} s) "
                 f"to {serial} {arm_name}; nothing was sent")
    return True


@register
class ArmPlannerPlugin(HuskyPlugin):
    """Plans, shows and commits a collision-free joint path for one arm."""

    name = "arm_planner"
    experimental = True
    requires = ("cell",)

    def __init__(self):
        """Start with no robot chosen, no target and no plan."""
        #: The robot and arm being planned for.
        self.serial: str | None = None
        self.arm: str | None = None
        #: Whether a target has been given yet; until then no ghost is drawn.
        self._has_target = False
        #: The current plan, the robot and arm it is for, and the base pose it was planned from.
        self.path: ArmPath | None = None
        self._path_for: tuple[str, str] | None = None
        self._path_base: np.ndarray | None = None
        #: The plan's state: "none", "planning", "ready", "stale" or "sent".
        self.plan_state = "none"
        #: Time shown on the slider and the ghost, seconds into the path.
        self._t = 0.0
        self._playing = False
        self._last_tick: float | None = None
        #: One line of feedback on the last action, and whether it was a failure.
        self._message, self._message_failed = "", False
        #: The running search, and the flag that ends its worker early.
        self._plan_task: asyncio.Task | None = None
        self._abort = threading.Event()
        #: Robots whose models the worker has loaded, so later plans need no load allowance.
        self._loaded: set[str] = set()
        # One worker: it owns the planning worlds, so searches run one after another.
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="arm-plan")
        # ! Worker thread only (sync, search, close); see planner.py.
        self._world = ArmPlanningWorld()
        #: Target slider values to set on the next update (radians), or None.
        self._load: np.ndarray | None = None
        #: Load the sliders from where the arm is on the next update (at startup, and on a new
        #: robot or arm), unless `_load` is set by then. ? Needs ctx, which update has.
        self._load_now = True
        #: The target slider values update wrote last. ! Writing a slider fires its callback
        #: too; a callback that finds these values is ours, not the operator's.
        self._written: tuple[float, ...] | None = None
        #: When the operator last used this panel; ghosts show only while it counts (setup).
        self._used: RecentUse | None = None

    # --- --- --- --- --- SETUP --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Build the panel, and a target ghost and a path ghost per robot.

        Args:
            ctx: This plugin's context.
        """
        arms = {robot.serial: [arm.name for arm in robot.arms] for robot in ctx.config.robots if robot.arms}
        if not arms:
            raise RuntimeError("no robots with arms configured; the arm planner has nothing to plan for")
        self._arms = arms
        self.serial = next(iter(arms))
        self.arm = arms[self.serial][0]
        self._used = RecentUse(ctx.config.ghost_timeout)

        with ctx.view.ui() as gui:
            self._status = gui.add_html("")
            self._robot = gui.add_dropdown("Robot", options=list(arms), initial_value=self.serial)
            self._arm = gui.add_dropdown("Arm", options=arms[self.serial], initial_value=self.arm)

            gui.add_html(section("target", SECTION_TOOL))
            self._sliders = [gui.add_slider(f"{name} °", min=-math.degrees(limit), max=math.degrees(limit),
                                            step=0.5, initial_value=0.0)
                             for name, limit in zip(SLIDER_NAMES, UR_JOINT_LIMITS)]
            load = gui.add_button_group("Load", ["Now", "Stow", "Cell"],
                                        hint="Set the target to where the arm is now, to its stow pose, or to "
                                             "the arm's target in the step selected in the cell plugin")

            gui.add_html(section("path", SECTION_CTRL))
            plan = gui.add_button_group("Path", ["Plan", "Play", "Clear"],
                                        hint="Plan around the robot itself, the other robots and the scene, from "
                                             "where the arm is to the target; play or pause the preview; remove "
                                             "the plan (or stop planning)")
            self._slider = gui.add_slider("t s", min=0.0, max=1.0, step=SLIDER_STEP, initial_value=0.0,
                                          hint="Scrub through the planned path")
            commit = gui.add_button("Commit", color="blue", icon=viser.Icon.SEND,
                                    hint="Send the path to the arm (stub: logs only)")
            # * Debugging: watch the planning world, and the search moving the arm around in it.
            # ? The window shows the chosen robot's world. `self.serial` is read on the worker:
            #   a str, so at worst it is the robot chosen a moment later.
            self._window = PyBulletWindowToggle(
                ctx, gui, self._executor,
                lambda gui, snapshot: self._world.set_gui(gui, snapshot, self.serial),
                report=lambda message: self._window_problem(ctx, message))
            # ! Keep changing content BELOW the buttons, so nothing moves under the cursor.
            self._details = gui.add_html("")

        self._robot.on_update(ctx.defer_value("choose robot", lambda serial: self._choose(str(serial), None)))
        self._arm.on_update(ctx.defer_value("choose arm", lambda arm: self._choose(self.serial, str(arm))))
        for slider in self._sliders:
            slider.on_update(ctx.defer_value("target", lambda _value: self._target_changed()))
        load.on_click(ctx.defer_value("load target", lambda label: self._load_target(ctx, label)))
        plan.on_click(ctx.defer_value("path button", lambda label: self._on_path(ctx, label)))
        self._slider.on_update(ctx.defer_value("scrub", lambda value: self._scrub(float(value))))
        commit.on_click(ctx.defer("commit", lambda: self._commit(ctx)))

        # * Two ghosts per robot, built now (mesh loading is too slow for the tick): the
        #   target, and the plan at the slider time. Both can show at once.
        robots = [config for config in ctx.config.robots if config.serial in arms]
        self._target_ghosts = {config.serial: RobotGhost(ctx, config, f"target/{config.serial}", TARGET_COLOR)
                               for config in robots}
        self._path_ghosts = {config.serial: RobotGhost(ctx, config, f"path/{config.serial}", PATH_COLOR)
                             for config in robots}

    def teardown(self, ctx: PluginContext) -> None:
        """At shutdown, end any search, then close the planning worlds.

        Args:
            ctx: This plugin's context.
        """
        # ! Abort first, so the wait is short. The worlds are closed on their own worker.
        self._abort.set()
        self._executor.submit(self._world.close)
        self._executor.shutdown(wait=True)

    # --- --- --- --- --- COMMANDS (intents, on the main thread) --- --- --- --- ---

    def _choose(self, serial: str, arm: str | None) -> None:
        """Plan for another robot or arm; the old plan no longer applies.

        Args:
            serial: The robot.
            arm: The arm, or None for the robot's first.
        """
        self._used.touch()
        arm = arm if arm in self._arms[serial] else self._arms[serial][0]
        if (serial, arm) == (self.serial, self.arm):
            return
        self.serial, self.arm = serial, arm
        self._clear()
        self._has_target, self._load_now = False, True

    def _target_changed(self) -> None:
        """A target slider moved: by the operator, unless update just wrote these values."""
        if tuple(slider.value for slider in self._sliders) == self._written:
            return
        self._used.touch()
        self._has_target = True

    def _load_target(self, ctx: PluginContext, label: str) -> None:
        """Set the target to the arm now, its stow pose, or the cell's selected step.

        Args:
            ctx: This plugin's context.
            label: "Now", "Stow" or "Cell".
        """
        self._used.touch()
        if label == "Now":
            self._load = self._joints_now(ctx)
            self._say(f"target set to where {self.serial} {self.arm} is now")
        elif label == "Stow":
            arm = next(arm for arm in self._robot_config(ctx).arms if arm.name == self.arm)
            if arm.stow_joints is None:
                self._say(f"{self.arm} has no stow pose configured", failed=True)
                return
            self._load = np.array(arm.stow_joints)
            self._say(f"target set to the stow pose of {self.arm}")
        else:
            self._load_from_cell(ctx)
            return
        self._has_target = True

    def _load_from_cell(self, ctx: PluginContext) -> None:
        """Set robot, arm and target from the cell's selected step: its target, else its start.

        Args:
            ctx: This plugin's context.
        """
        step = ctx.require("cell").step
        if step is None:
            self._say("no design loaded in the cell plugin", failed=True)
            return
        serial = find_robot_serial(ctx.config.robots, step.action.robot)
        if serial is None or serial not in self._arms:
            self._say(f"{step.label} is for {step.action.robot}, which is not configured", failed=True)
            return
        movement = step.movement
        configuration = movement.target_configuration or movement.start_state.robot_configuration
        given = {} if configuration is None else dict(zip(configuration.joint_names, configuration.joint_values))
        # * The arm the step moves: the first of the robot's arms it names every joint of.
        arm = next((arm for arm in self._arms[serial] if all(name in given for name in arm_joint_names(arm))), None)
        if arm is None:
            self._say(f"{step.label} has no joint values for an arm of {step.action.robot}", failed=True)
            return
        self._choose(serial, arm)
        self._load = np.array([given[name] for name in arm_joint_names(arm)])
        self._has_target = True
        which = "target" if movement.target_configuration is not None else "start"
        self._say(f"target from the {which} of {step.label} ({arm})")

    def _on_path(self, ctx: PluginContext, label: str) -> None:
        """Plan, play or pause, or clear.

        Args:
            ctx: This plugin's context.
            label: The button clicked.
        """
        self._used.touch()
        if label == "Plan":
            self._plan(ctx)
        elif label == "Play":
            if self.path is None:
                ctx.log_info("play ignored: nothing planned")
                return
            if not self._playing and self._t >= self.path.duration:
                self._t = 0.0  # play again from the start
            self._playing = not self._playing
        elif label == "Clear":
            self._clear()

    def _plan(self, ctx: PluginContext) -> None:
        """Start a search from where the arm is to the target.

        Args:
            ctx: This plugin's context.
        """
        if not self._has_target:
            self._say("set a target first", failed=True)
            return
        if self.plan_state == "planning":
            ctx.log_info("plan ignored: already planning")
            return
        self._clear()
        # A fresh flag per search, so an old search's abort cannot end this one.
        self._abort = threading.Event()
        self.plan_state = "planning"
        loading = self.serial not in self._loaded
        self._say(f"planning for {self.serial} {self.arm}…" + (" (loading the models first)" if loading else ""))
        self._plan_task = ctx.spawn(f"plan {self.serial} {self.arm}",
                                    self._search(ctx, self.serial, self.arm, self._target(), loading))

    async def _search(self, ctx: PluginContext, serial: str, arm: str, goal: np.ndarray, loading: bool) -> None:
        """Run one search on the worker thread and take its result on the main thread.

        ! No path found is reported in the panel, not raised: it is not a bug.

        Args:
            ctx: This plugin's context.
            serial: The robot to plan for.
            arm: Its arm.
            goal: Joint values to end at, radians.
            loading: Whether the worker loads this robot's models first, which takes longer.
        """
        loop = asyncio.get_running_loop()
        # * The world as it stood at the start of this tick; the arm starts where it is in it.
        snapshot = ctx.scene.snapshot
        base = np.array(snapshot.robots[serial].base.position)
        abort = self._abort

        def search() -> tuple[PlanResult, tuple[bool, str]]:
            """Sync the planning world and search. Runs on the worker."""
            mirror = self._world.sync(snapshot, serial)
            return plan_arm(self._world, mirror, arm, goal, abort), self._world.window()

        limit = TIME_LIMIT + 5.0 + (LOAD_ALLOWANCE if loading else 0.0)
        try:
            async with timeout(limit):
                result, window = await loop.run_in_executor(self._executor, search)
        except asyncio.TimeoutError:
            self.plan_state = "none"
            self._say(f"timed out after {limit:.0f}s waiting for the arm planner", failed=True)
            return
        finally:
            # ! A thread cannot be interrupted: on cancel, timeout or failure, tell the
            #   worker to stop and drop its result. Harmless if it already finished.
            abort.set()

        self._loaded.add(serial)
        self._window.show(*window)
        if result.path is None:
            self.plan_state = "none"
            self._say(f"no plan: {result.reason}", failed=True)
            return
        self.path, self._path_for, self._path_base = result.path, (serial, arm), base
        self.plan_state = "ready"
        self._t, self._playing = 0.0, False
        how = "straight" if result.direct else "RRT"
        self._say(f"{how}: {len(self.path.waypoints)} waypoints, {self.path.duration:.1f} s, "
                  f"found in {result.seconds:.1f} s")

    def _scrub(self, t: float) -> None:
        """Show the path at time `t`."""
        # ! draw writes the slider back, which lands here again; the same value
        #   changes nothing, which is what ends that round trip.
        if abs(t - self._t) > 1e-9:
            self._t = t
            self._used.touch()

    def _commit(self, ctx: PluginContext) -> None:
        """Send the plan to the arm, if it is still good.

        Args:
            ctx: This plugin's context.
        """
        self._used.touch()
        if self.plan_state != "ready":
            self._say(f"nothing to commit: the plan is {self.plan_state}", failed=True)
            return
        serial, arm = self._path_for
        if send_to_arm(ctx, serial, arm, self.path):
            self.plan_state = "sent"
            self._say(f"sent to {serial} {arm} (stub: nothing was sent)")
        else:
            self._say(f"{serial} {arm} refused the path", failed=True)

    def _clear(self) -> None:
        """Forget the plan, and end a search in progress."""
        if self._plan_task is not None and not self._plan_task.done():
            self._plan_task.cancel()
            self._abort.set()  # stop the worker now, not at the task's next step
            self._say("planning cancelled")
        self._plan_task = None
        self.path, self._path_for, self._path_base, self.plan_state = None, None, None, "none"
        self._t, self._playing = 0.0, False

    def _window_problem(self, ctx: PluginContext, message: str) -> None:
        """Say why the PyBullet window asked for isn't open, in the panel and the log. "" says nothing."""
        if not message:
            return
        ctx.log_warn(message)
        self._say(message, failed=True)

    def _say(self, message: str, failed: bool = False) -> None:
        """Set the one line of feedback under the buttons."""
        self._message, self._message_failed = message, failed

    # --- --- --- --- --- READING --- --- --- --- ---

    def _robot_config(self, ctx: PluginContext):
        """The chosen robot's RobotConfig."""
        return next(robot for robot in ctx.config.robots if robot.serial == self.serial)

    def _target(self) -> np.ndarray:
        """The target from the sliders, radians."""
        return np.radians([slider.value for slider in self._sliders])

    def _joints_now(self, ctx: PluginContext) -> np.ndarray:
        """Where the chosen arm's joints are now, radians, UR driver order."""
        joints = ctx.kinematics.joints(self.serial)
        return np.array([joints.get(name, 0.0) for name in arm_joint_names(self.arm)])

    # --- --- --- --- --- TICK --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Advance playback, apply queued target loads, and notice when the plan no longer fits.

        Args:
            ctx: This plugin's context.
        """
        now = ctx.now()
        elapsed = 0.0 if self._last_tick is None else now - self._last_tick
        self._last_tick = now

        if self._load_now:
            self._load_now = False
            if self._load is None:
                self._load = self._joints_now(ctx)
        if self._load is not None:
            for slider, value in zip(self._sliders, np.degrees(self._load)):
                slider.value = round(float(value) * 2) / 2  # the sliders' 0.5 deg step
            self._written = tuple(slider.value for slider in self._sliders)
            self._load = None

        if self.path is None:
            return
        if self._playing:
            self._t = min(self._t + elapsed, self.path.duration)
            self._playing = self._t < self.path.duration

        # * A ready plan goes stale when the arm or base moves away from its start,
        #   or the target from its goal. A sent plan stays sent.
        if self.plan_state == "ready":
            serial, arm = self._path_for
            joints = ctx.kinematics.joints(serial)
            here = np.array([joints.get(name, 0.0) for name in arm_joint_names(arm)])
            base = np.array(ctx.kinematics.base_pose(serial).position)
            if np.max(np.abs(here - self.path.start)) > STALE_JOINT:
                reason = "the arm moved since planning"
            elif np.linalg.norm(base[:2] - self._path_base[:2]) > STALE_POSITION:
                reason = "the base moved since planning"
            elif np.max(np.abs(self._target() - self.path.goal)) > STALE_JOINT:
                reason = "the target changed since planning"
            else:
                reason = ""
            if reason:
                self.plan_state = "stale"
                self._say(reason)

    def draw(self, ctx: PluginContext) -> None:
        """Pose the ghost and fill the panel.

        Args:
            ctx: This plugin's context.
        """
        self._status.content = self._status_html()
        # Loading a target from the cell can switch the robot or arm; show that here.
        self._robot.value = self.serial
        if tuple(self._arm.options) != tuple(self._arms[self.serial]):
            self._arm.options = self._arms[self.serial]
        self._arm.value = self.arm
        self._details.content = self._details_html()

        duration = 0.0 if self.path is None else self.path.duration
        self._slider.max = max(duration, SLIDER_STEP)
        self._slider.value = round(self._t, 3)

        self._draw_ghosts(ctx)

    def _draw_ghosts(self, ctx: PluginContext) -> None:
        """Show the chosen arm at the target, and the planned arm at the slider time; hide the rest.

        * Only while the panel is in use (`RecentUse`), and only the arm (with its tool).
        ? Every tick: the base is drawn where it is now, and `show` does nothing when nothing changed.

        Args:
            ctx: This plugin's context.
        """
        in_use = self._used.active
        shown = {"target": (self.serial, self.arm, self._target()) if in_use and self._has_target else None,
                 "path": (*self._path_for, self.path.sample(self._t)) if in_use and self.path is not None else None}
        for kind, ghosts in (("target", self._target_ghosts), ("path", self._path_ghosts)):
            for serial, ghost in ghosts.items():
                if shown[kind] is None or shown[kind][0] != serial:
                    ghost.hide()
                    continue
                _, arm, values = shown[kind]
                joints = dict(ctx.kinematics.joints(serial))
                joints.update(zip(arm_joint_names(arm), values))
                ghost.show(ctx.kinematics.base_pose(serial), joints, parts=(f"{arm}_",))

    # --- --- --- --- --- PANEL --- --- --- --- ---

    def _status_html(self) -> str:
        """One line of chips: the robot and arm, and the plan's state.

        Returns:
            str: HTML.
        """
        chips = chip(escape(f"{self.serial} {self.arm}"), SECTION_CTRL)
        colors = {"none": NONE, "planning": BUSY, "ready": OK, "stale": BUSY, "sent": OK}
        labels = {"none": "no plan", "planning": "planning…", "ready": "plan ready", "stale": "plan stale: replan",
                  "sent": "sent (stub)"}
        chips += chip(labels[self.plan_state], colors[self.plan_state])
        return block(chips)

    def _details_html(self) -> str:
        """The plan and the joints at the slider time, and the last message. Fixed number of rows.

        Returns:
            str: HTML.
        """
        if self.path is None:
            path, at = "path    —", "at t    —"
        else:
            path = f"path    {len(self.path.waypoints)} waypoints  {self.path.duration:5.1f} s"
            at = "at t    " + " ".join(f"{math.degrees(v):+6.1f}" for v in self.path.sample(self._t))
        message = (f'<div style="font-size:11px;color:{FAIL if self._message_failed else NONE};'
                   f'white-space:nowrap;overflow:hidden;text-overflow:ellipsis">'
                   f'{escape(self._message) or "&nbsp;"}</div>')
        return block(values(path, at) + message)
