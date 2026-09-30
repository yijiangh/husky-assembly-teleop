"""
Pick a target for a husky base, plan a path to it, scrub through the path, and commit it.

! EXPERIMENTAL: a mockup. Commit is a stub and the planner is a placeholder.

* Planning (bidirectional RRT, planner.py) runs on a worker thread from a task, so
  the tick never waits; Clear or Stop all ends it early. It plans against the
  tick's scene snapshot: every robot and every scene body (obstacles, cell).

! Commit is a stub that only logs (`send_to_onboard_follower`).
! A plan is only good for the pose it was planned from: if the base, target or
  robot changes it goes stale, stays on screen, and Commit refuses it until replanned.
"""

from __future__ import annotations

import asyncio
import math
import threading
from concurrent.futures import ThreadPoolExecutor
from html import escape

import numpy as np
import viser

from ...plugin_api.concurrency import timeout
from ...config import find_robot_serial
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...ui.pose_input import PlanarPoseInput, yaw_from_xyzw
from ...ui.ghost import PATH_COLOR, TARGET_COLOR, RecentUse, RobotGhost
from ...ui.pybullet_window import PyBulletWindowToggle
from ...ui.style import BUSY, FAIL, NONE, OK, SECTION_CTRL, SECTION_TOOL, block, chip, section, values
from ...world.scene import Pose
from .path import BasePath
from .planner import TIME_LIMIT, PlanResult, PlanningWorld, plan_birrt

#: The plan goes stale once the base moves this far from its planned start.
STALE_POSITION = 0.05              # m
STALE_YAW = math.radians(3.0)      # rad

#: Look of the path line.
LINE_COLOR = (66, 99, 235)         # SECTION_CTRL blue
PATH_HEIGHT = 0.005                # m, just above the grid
PATH_THICKNESS = 0.015             # m

#: Time slider resolution, seconds.
SLIDER_STEP = 0.05


def send_to_onboard_follower(ctx: PluginContext, serial: str, path: BasePath) -> bool:
    """Hand a path to a robot's onboard path follower. STUB: only logs.

    TODO Publish the path (e.g. nav_msgs/Path, world frame) through a method on
         BaseInterface, so this plugin needs no topic names.

    Args:
        ctx: The plugin's context, for logging.
        serial: Which robot.
        path: The path to follow.

    Returns:
        bool: Whether the follower took the path. Always True for the stub.
    """
    ctx.log_warn(f"[STUB] would send a {len(path.poses)}-waypoint path ({path.length:.2f} m, "
                 f"{path.duration:.1f} s) to the onboard follower of {serial}; nothing was sent")
    return True


@register
class BasePlannerPlugin(HuskyPlugin):
    """Plans, shows and commits a base path from the current pose to a target."""

    name = "base_planner"
    experimental = True
    requires = ("cell",)

    def __init__(self):
        """Start with no robot chosen, no target and no plan."""
        #: The robot being planned for, by serial.
        self.serial: str | None = None
        #: Whether a target has been given yet; until then none is drawn.
        self._has_target = False
        #: The current plan, and the robot it is for.
        self.path: BasePath | None = None
        self._path_serial: str | None = None
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
        # One worker: it owns the planning world, so searches run one after another.
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="base-plan")
        # ! Worker thread only (sync, search, close); see planner.py.
        self._world = PlanningWorld()
        # Whether draw must rebuild the path line.
        self._stale_line = True

    # --- --- --- --- --- SETUP --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Build the panel, the path line, and a target ghost and a path ghost per robot.

        Args:
            ctx: This plugin's context.
        """
        serials = [robot.serial for robot in ctx.config.robots]
        if not serials:
            raise RuntimeError("no robots configured; the base planner has nothing to plan for")
        self.serial = serials[0]
        #: When the operator last used this panel; ghosts show only while it counts.
        self._used = RecentUse(ctx.config.ghost_timeout)
        root = ctx.view.scene_root

        with ctx.view.ui() as gui:
            self._status = gui.add_html("")
            self._robot = gui.add_dropdown("Robot", options=serials, initial_value=self.serial)

            gui.add_html(section("target", SECTION_TOOL))
            self._target = PlanarPoseInput(ctx, gui, f"{root}/target_gizmo", label="Target",
                                           on_change=self._target_changed)
            load = gui.add_button_group("Load", ["Robot", "Cell"],
                                        hint="Set the target to where the robot is now, or to its base "
                                             "pose in the step selected in the cell plugin")

            gui.add_html(section("path", SECTION_CTRL))
            plan = gui.add_button_group("Path", ["Plan", "Play", "Clear"],
                                        hint="Plan around the other robots from the robot's current pose to "
                                             "the target; play or pause the preview; remove the plan (or stop "
                                             "planning)")
            self._slider = gui.add_slider("t s", min=0.0, max=1.0, step=SLIDER_STEP, initial_value=0.0,
                                          hint="Scrub through the planned path")
            commit = gui.add_button("Commit", color="blue", icon=viser.Icon.SEND,
                                    hint="Send the path to the robot's onboard path follower (stub: logs only)")
            # * Debugging: watch the planning world, and the search moving the robot around in it.
            self._window = PyBulletWindowToggle(ctx, gui, self._executor, self._world.set_gui,
                                                report=lambda message: self._window_problem(ctx, message))
            # ! Keep changing content BELOW the buttons, so nothing moves under the cursor.
            self._details = gui.add_html("")

        self._robot.on_update(ctx.defer_value("choose robot", lambda serial: self._choose_robot(str(serial))))
        load.on_click(ctx.defer_value("load target", lambda label: self._load_target(ctx, label)))
        plan.on_click(ctx.defer_value("path button", lambda label: self._on_path(ctx, label)))
        self._slider.on_update(ctx.defer_value("scrub", lambda value: self._scrub(float(value))))
        commit.on_click(ctx.defer("commit", lambda: self._commit(ctx)))

        #: The path line, rebuilt per plan (viser fixes the colour count at creation).
        self._line: viser.LineSegmentsHandle | None = None
        # * Two ghosts per robot, built now (mesh loading is too slow for the tick): the
        #   robot at the target, and on the plan at the slider time. Both can show at once.
        self._target_ghosts = {config.serial: RobotGhost(ctx, config, f"target/{config.serial}", TARGET_COLOR)
                               for config in ctx.config.robots}
        self._path_ghosts = {config.serial: RobotGhost(ctx, config, f"ghost/{config.serial}", PATH_COLOR)
                             for config in ctx.config.robots}

    def teardown(self, ctx: PluginContext) -> None:
        """At shutdown, end any search, then close the planning world.

        Args:
            ctx: This plugin's context.
        """
        # ! Abort first, so the wait is short. The world is closed on its own worker,
        #   after any search still running there.
        self._abort.set()
        self._executor.submit(self._world.close)
        self._executor.shutdown(wait=True)

    # --- --- --- --- --- COMMANDS (intents, on the main thread) --- --- --- --- ---

    def _choose_robot(self, serial: str) -> None:
        """Plan for another robot. The old plan no longer applies."""
        self._used.touch()
        if serial != self.serial:
            self.serial = serial
            self._clear()

    def _target_changed(self) -> None:
        """The operator edited the target."""
        self._used.touch()
        self._has_target = True

    def _load_target(self, ctx: PluginContext, label: str) -> None:
        """Set the target from the robot's current pose, or from the cell's selected step.

        Args:
            ctx: This plugin's context.
            label: "Robot" or "Cell".
        """
        self._used.touch()
        if label == "Robot":
            x, y, yaw = self._current_pose(ctx)
            self._target.set(x, y, yaw)
            self._has_target = True
            self._say(f"target set to where {self.serial} is now")
            return

        step = ctx.require("cell").step
        if step is None:
            self._say("no design loaded in the cell plugin", failed=True)
            return
        frame = step.movement.start_state.robot_base_frame
        if frame is None:
            self._say(f"{step.label} has no robot base frame", failed=True)
            return
        # * The step says which robot it is for; plan for that one.
        serial = find_robot_serial(ctx.config.robots, step.action.robot)
        if serial is None:
            self._say(f"{step.label} is for {step.action.robot}, which is not configured", failed=True)
            return
        self._choose_robot(serial)
        # ? The cell's base frame is the model root (base_footprint), as in mocap and PyBullet.
        self._target.set(frame.point.x, frame.point.y, math.atan2(frame.xaxis.y, frame.xaxis.x))
        self._has_target = True
        self._say(f"target from {step.label} ({step.action.robot})")

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
        """Start a search from the robot's current pose to the target.

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
        self._say(f"planning for {self.serial}…")
        self._plan_task = ctx.spawn(f"plan {self.serial}",
                                   self._search(ctx, self.serial, self._current_pose(ctx), self._target.pose))

    async def _search(self, ctx: PluginContext, serial: str, start, goal) -> None:
        """Run one search on the worker thread and take its result on the main thread.

        ! No path found is reported in the panel, not raised: it is not a bug.

        Args:
            ctx: This plugin's context.
            serial: The robot to plan for.
            start: (x, y, yaw) it starts from.
            goal: (x, y, yaw) it should end at.
        """
        loop = asyncio.get_running_loop()
        # * The world as it stood at the start of this tick. The worker syncs its
        #   planning world from it, after any cancelled search still finishing there.
        snapshot = ctx.scene.snapshot
        abort = self._abort

        def search() -> tuple[PlanResult, tuple[bool, str]]:
            """Sync the planning world and search. Runs on the worker."""
            self._world.sync(snapshot)
            return plan_birrt(self._world, serial, start, goal, abort), self._world.window()

        try:
            async with timeout(TIME_LIMIT + 5.0):
                result, window = await loop.run_in_executor(self._executor, search)
        except asyncio.TimeoutError:
            self.plan_state = "none"
            self._say(f"timed out after {TIME_LIMIT + 5.0}s waiting for the base planner", failed=True)
            return
        finally:
            # ! A thread cannot be interrupted: on cancel, timeout or failure, tell the
            #   worker to stop and drop its result. Harmless if it already finished.
            abort.set()

        # ? Unticks the box if the window was closed by hand; the plan result below is the last word.
        self._window.show(*window)
        if result.path is None:
            self.plan_state = "none"
            self._say(f"no plan: {result.reason}", failed=True)
            return
        self.path, self._path_serial, self.plan_state = result.path, serial, "ready"
        self._t, self._playing = 0.0, False
        self._stale_line = True
        how = "straight" if result.direct else "RRT"
        tracked = ctx.world.robots[serial].base.state.tracked
        self._say(f"{how}: {self.path.length:.2f} m, {self.path.duration:.1f} s, found in {result.seconds:.1f} s"
                  + ("" if tracked else ", from an untracked pose"), failed=not tracked)

    def _scrub(self, t: float) -> None:
        """Show the path at time `t`."""
        # ! draw writes the slider back, which lands here again; the same value
        #   changes nothing, which is what ends that round trip.
        if abs(t - self._t) > 1e-9:
            self._t = t
            self._used.touch()

    def _commit(self, ctx: PluginContext) -> None:
        """Send the plan to the onboard follower, if it is still good.

        Args:
            ctx: This plugin's context.
        """
        self._used.touch()
        if self.plan_state != "ready":
            self._say(f"nothing to commit: the plan is {self.plan_state}", failed=True)
            return
        if not ctx.world.robots[self._path_serial].base.state.tracked:
            self._say(f"not committed: {self._path_serial} is not tracked by mocap", failed=True)
            return
        if send_to_onboard_follower(ctx, self._path_serial, self.path):
            self.plan_state = "sent"
            self._say(f"sent to {self._path_serial} (stub: nothing was sent)")
        else:
            self._say(f"{self._path_serial}'s follower refused the path", failed=True)

    def _clear(self) -> None:
        """Forget the plan, and end a search in progress."""
        if self._plan_task is not None and not self._plan_task.done():
            self._plan_task.cancel()
            self._abort.set()  # stop the worker now, not at the task's next step
            self._say("planning cancelled")
        self._plan_task = None
        self.path, self._path_serial, self.plan_state = None, None, "none"
        self._t, self._playing = 0.0, False
        self._stale_line = True

    def _window_problem(self, ctx: PluginContext, message: str) -> None:
        """Say why the PyBullet window asked for isn't open, in the panel and the log. "" says nothing."""
        if not message:
            return
        ctx.log_warn(message)
        self._say(message, failed=True)

    def _say(self, message: str, failed: bool = False) -> None:
        """Set the one line of feedback under the buttons."""
        self._message, self._message_failed = message, failed

    # --- --- --- --- --- TICK --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Advance playback, and notice when the plan no longer fits the world.

        Args:
            ctx: This plugin's context.
        """
        now = ctx.now()
        elapsed = 0.0 if self._last_tick is None else now - self._last_tick
        self._last_tick = now
        if self.path is None:
            return

        if self._playing:
            self._t = min(self._t + elapsed, self.path.duration)
            self._playing = self._t < self.path.duration

        # * A ready plan goes stale when the base moves away from its start, or
        #   the target moves away from its goal. A sent plan stays sent.
        if self.plan_state == "ready":
            if _differs(self._current_pose(ctx), self.path.start):
                reason = "the robot moved since planning"
            elif _differs(self._target.pose, self.path.goal):
                reason = "the target changed since planning"
            else:
                reason = ""
            if reason:
                self.plan_state = "stale"
                self._say(reason)

    def _current_pose(self, ctx: PluginContext) -> tuple[float, float, float]:
        """Where the chosen robot's base is now, (x, y, yaw).

        ? Read from kinematics, which holds the last mocap fix, or the
          robot's default pose before any. That keeps planning usable without
          mocap (in simulation, or on the bench); Commit is what insists on a
          tracked pose.

        Args:
            ctx: This plugin's context.

        Returns:
            tuple[float, float, float]: x, y in metres and yaw in radians, world frame.
        """
        base = ctx.kinematics.base_pose(self.serial)
        return float(base.position[0]), float(base.position[1]), yaw_from_xyzw(base.orientation)

    def draw(self, ctx: PluginContext) -> None:
        """Pose the ghosts, redraw the path line if needed, and fill the panel.

        Args:
            ctx: This plugin's context.
        """
        self._status.content = self._status_html(ctx)
        # Loading a target from the cell can switch the robot; show that here.
        self._robot.value = self.serial
        self._details.content = self._details_html()

        duration = 0.0 if self.path is None else self.path.duration
        self._slider.max = max(duration, SLIDER_STEP)
        self._slider.value = round(self._t, 3)

        if self._stale_line:
            self._stale_line = False
            self._draw_line(ctx)
        self._draw_ghosts(ctx)

    def _draw_line(self, ctx: PluginContext) -> None:
        """Replace the path line with the current plan's, or remove it.

        Args:
            ctx: This plugin's context.
        """
        if self._line is not None:
            self._line.remove()
            self._line = None
        if self.path is None:
            return
        points = self.path.floor_points()
        if len(points) < 2:
            return  # turning on the spot: nothing to draw on the floor
        points = np.column_stack([points, np.full(len(points), PATH_HEIGHT)])
        self._line = ctx.view.scene.add_line_segments(
            f"{ctx.view.scene_root}/path", points=np.stack([points[:-1], points[1:]], axis=1),
            colors=LINE_COLOR, thickness=PATH_THICKNESS)

    def _draw_ghosts(self, ctx: PluginContext) -> None:
        """Show the chosen robot at the target, and the planned robot at the slider time; hide the rest.

        ? Every tick: height and arms are drawn as they are now (a base move does not
          move the arms), and `show` does nothing when nothing changed.

        Args:
            ctx: This plugin's context.
        """
        # * Only while the panel is in use (`RecentUse`).
        in_use = self._used.active
        shown = {"target": (self.serial, self._target.pose) if in_use and self._has_target else None,
                 "path": (self._path_serial, self.path.sample(self._t)) if in_use and self.path is not None else None}
        for kind, ghosts in (("target", self._target_ghosts), ("path", self._path_ghosts)):
            for serial, ghost in ghosts.items():
                if shown[kind] is None or shown[kind][0] != serial:
                    ghost.hide()
                    continue
                x, y, yaw = (float(v) for v in shown[kind][1])
                # Height from the live robot, so the ghost stands where the robot stands.
                z = float(ctx.kinematics.base_pose(serial).position[2])
                base = Pose((x, y, z), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
                ghost.show(base, ctx.kinematics.joints(serial))

    # --- --- --- --- --- PANEL --- --- --- --- ---

    def _status_html(self, ctx: PluginContext) -> str:
        """One line of chips: the robot and whether it is tracked, and the plan's state.

        Args:
            ctx: This plugin's context.

        Returns:
            str: HTML.
        """
        tracked = ctx.world.robots[self.serial].base.state.tracked
        chips = chip(escape(self.serial), SECTION_CTRL)
        chips += chip("tracked" if tracked else "not tracked", OK if tracked else BUSY,
                      "" if tracked else "planning from the last known or default pose")
        colors = {"none": NONE, "planning": BUSY, "ready": OK, "stale": BUSY, "sent": OK}
        labels = {"none": "no plan", "planning": "planning…", "ready": "plan ready", "stale": "plan stale: replan",
                  "sent": "sent (stub)"}
        chips += chip(labels[self.plan_state], colors[self.plan_state])
        return block(chips)

    def _details_html(self) -> str:
        """Numbers of the target and the plan, and the last message. Fixed number of rows.

        Returns:
            str: HTML.
        """
        x, y, yaw = self._target.pose
        target = f"target  {x:+7.3f} {y:+7.3f} {math.degrees(yaw):+7.1f}°" if self._has_target else "target  —"
        if self.path is None:
            path = "path    —"
            at = "at t    —"
        else:
            px, py, pyaw = self.path.sample(self._t)
            path = f"path    {self.path.length:6.2f} m  {self.path.duration:5.1f} s  {len(self.path.poses)} waypoints"
            at = f"at t    {px:+7.3f} {py:+7.3f} {math.degrees(pyaw):+7.1f}°"
        message = (f'<div style="font-size:11px;color:{FAIL if self._message_failed else NONE};'
                   f'white-space:nowrap;overflow:hidden;text-overflow:ellipsis">'
                   f'{escape(self._message) or "&nbsp;"}</div>')
        return block(values(target, path, at) + message)


def _differs(pose, reference) -> bool:
    """Whether two floor poses are further apart than the stale thresholds.

    Args:
        pose: (x, y, yaw).
        reference: (x, y, yaw).

    Returns:
        bool: True if the position or the heading differs by more than allowed.
    """
    dx, dy = pose[0] - reference[0], pose[1] - reference[1]
    dyaw = math.atan2(math.sin(pose[2] - reference[2]), math.cos(pose[2] - reference[2]))
    return math.hypot(dx, dy) > STALE_POSITION or abs(dyaw) > STALE_YAW
