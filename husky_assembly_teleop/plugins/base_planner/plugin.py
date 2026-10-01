"""
Pick a target for a husky base, plan a path to it, scrub through the path, and commit it.

! EXPERIMENTAL. Commit is a stub that only logs (`send_to_onboard_follower`), and the
  path model (turn-drive-turn at constant speed) will change to follow the base's dynamics.

* The flow (plan on a worker, preview, stale, commit) is `planning.panel.PlannerPlugin`.
  Here: the planar target, the floor line, and a private PyBullet world (planner.py)
  holding every robot and scene body.
! A plan is only good for the pose it was planned from: if the base or target moves it
  goes stale, stays on screen, and Commit refuses it until replanned.
"""

from __future__ import annotations

import math
import threading
from html import escape

import numpy as np
import viser

from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import register
from ...planning.panel import WAIT_MARGIN, PlannerPlugin, Search
from ...planning.search import TIME_LIMIT, PlanResult
from ...ui.ghost import RobotGhost
from ...ui.pose_input import PlanarPoseInput, yaw_from_xyzw
from ...ui.pybullet_window import PyBulletWindowToggle
from ...ui.style import BUSY, OK, SECTION_CTRL, SECTION_TOOL, block, chip, section, values
from ...world.scene import Pose
from .path import BasePath
from .planner import PlanningWorld, plan_birrt

#: The plan goes stale once the base moves this far from its planned start.
STALE_POSITION = 0.05              # m
STALE_YAW = math.radians(3.0)      # rad

#: Look of the path line.
LINE_COLOR = (66, 99, 235)         # SECTION_CTRL blue
PATH_HEIGHT = 0.005                # m, just above the grid
PATH_THICKNESS = 0.015             # m


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
    ctx.log_warn(f"[STUB] would send a {len(path.points)}-waypoint path ({path.length:.2f} m, "
                 f"{path.duration:.1f} s) to the onboard follower of {serial}; nothing was sent")
    return True


@register
class BasePlannerPlugin(PlannerPlugin):
    """Plans, shows and commits a base path from the current pose to a target."""

    name = "base_planner"
    experimental = True
    worker_name = "base-plan"

    def __init__(self):
        """Start with no robot chosen, no target and no plan."""
        super().__init__()
        self.path: BasePath | None = None
        # ! Worker thread only (sync, search, close); see planner.py.
        self._world = PlanningWorld()
        # Whether draw must rebuild the path line.
        self._stale_line = True
        #: The PyBullet window's state, read on the worker after a search.
        self._window_state = (False, "")

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

            self._add_path_controls(ctx, gui,
                                    plan_hint="Plan around the other robots from the robot's current pose to "
                                              "the target; play or pause the preview; remove the plan (or stop "
                                              "planning)",
                                    commit_hint="Send the path to the robot's onboard path follower (stub: logs only)")
            # * Debugging: watch the planning world, and the search moving the robot around in it.
            self._window = PyBulletWindowToggle(ctx, gui, self._executor, self._world.set_gui,
                                                report=lambda message: self._report_problem(ctx, message))
            # ! Keep changing content BELOW the buttons, so nothing moves under the cursor.
            self._details = gui.add_html("")

        self._robot.on_update(ctx.defer_value("choose robot", lambda serial: self._choose_robot(str(serial))))
        load.on_click(ctx.defer_value("load target", lambda label: self._load_target(ctx, label)))

        #: The path line, rebuilt per plan (viser fixes the colour count at creation).
        self._line: viser.LineSegmentsHandle | None = None
        self._add_ghosts(ctx, ctx.config.robots)

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

        found = self._cell_step(ctx)
        if found is None:
            return
        step, serial, robot_name = found
        # * The step says which robot it is for; plan for that one.
        self._choose_robot(serial)
        # ? The design's base pose is the URDF root (base_footprint), as in mocap and PyBullet.
        base = step.movement.start.robots[step.action.robot].base
        self._target.set(float(base.position[0]), float(base.position[1]), yaw_from_xyzw(base.orientation))
        self._has_target = True
        self._say(f"target from {step.label} ({robot_name})")

    # --- --- --- --- --- PLANNER PLUGIN --- --- --- --- ---

    def make_search(self, ctx: PluginContext, abort: threading.Event) -> Search:
        """Search from the robot's current pose to the target, in the tick's snapshot."""
        serial, start, goal = self.serial, self._current_pose(ctx), self._target.pose
        # * The world as it stood at the start of this tick. The worker syncs its
        #   planning world from it, after any cancelled search still finishing there.
        snapshot = ctx.scene.snapshot

        def work() -> PlanResult:
            self._world.sync(snapshot)
            result = plan_birrt(self._world, serial, start, goal, abort)
            self._window_state = self._world.window()
            return result

        def accepted(result: PlanResult) -> tuple[str, bool]:
            self._stale_line = True
            how = "straight" if result.direct else "RRT"
            tracked = ctx.world.robots[serial].base.state.tracked
            return (f"{how}: {self.path.length:.2f} m, {self.path.duration:.1f} s, found in {result.seconds:.1f} s"
                    + ("" if tracked else ", from an untracked pose"), not tracked)

        return Search(serial=serial, label=f"plan {serial}", message=f"planning for {serial}…", work=work,
                      time_limit=TIME_LIMIT + WAIT_MARGIN, accepted=accepted,
                      # ? Unticks the box if the window was closed by hand.
                      finished=lambda: self._window.show(*self._window_state))

    def stale_reason(self, ctx: PluginContext) -> str:
        """Stale once the base moves away from the plan's start, or the target from its goal."""
        if _differs(self._current_pose(ctx), self.path.start):
            return "the robot moved since planning"
        if _differs(self._target.pose, self.path.goal):
            return "the target changed since planning"
        return ""

    def send_plan(self, ctx: PluginContext) -> tuple[bool, str]:
        """Send the plan to the onboard follower; refused while the robot is not tracked."""
        serial = self._path_serial
        if not ctx.world.robots[serial].base.state.tracked:
            return False, f"not committed: {serial} is not tracked by mocap"
        if send_to_onboard_follower(ctx, serial, self.path):
            return True, f"sent to {serial} (stub: nothing was sent)"
        return False, f"{serial}'s follower refused the path"

    def forget_plan(self) -> None:
        """The path line goes with the plan."""
        self._stale_line = True

    def show_target(self, ctx: PluginContext, ghost: RobotGhost) -> None:
        """The robot at the target pose, arms as they are now."""
        self._show_at(ctx, ghost, self.serial, self._target.pose)

    def show_path(self, ctx: PluginContext, ghost: RobotGhost) -> None:
        """The robot on the path at the slider time, arms as they are now."""
        self._show_at(ctx, ghost, self._path_serial, self.path.sample(self._t))

    def _show_at(self, ctx: PluginContext, ghost: RobotGhost, serial: str, pose) -> None:
        """Show `ghost` at floor pose (x, y, yaw), height and arms from the live robot.

        ? A base move does not move the arms, so they are drawn as they are now.
        """
        x, y, yaw = (float(v) for v in pose)
        z = float(ctx.kinematics.base_pose(serial).position[2])
        ghost.show(Pose((x, y, z), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))), ctx.kinematics.joints(serial))

    # --- --- --- --- --- READING --- --- --- --- ---

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

    # --- --- --- --- --- DRAW --- --- --- --- ---

    def draw(self, ctx: PluginContext) -> None:
        """Pose the ghosts, redraw the path line if needed, and fill the panel.

        Args:
            ctx: This plugin's context.
        """
        self._status.content = self._status_html(ctx)
        # Loading a target from the cell can switch the robot; show that here.
        self._robot.value = self.serial
        self._details.content = self._details_html()
        self._draw_slider()
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
        return block(chips + self._plan_chip())

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
            path = f"path    {self.path.length:6.2f} m  {self.path.duration:5.1f} s  {len(self.path.points)} waypoints"
            at = f"at t    {px:+7.3f} {py:+7.3f} {math.degrees(pyaw):+7.1f}°"
        return block(values(target, path, at) + self._message_html())


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
