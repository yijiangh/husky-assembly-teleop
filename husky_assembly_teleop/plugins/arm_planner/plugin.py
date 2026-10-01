"""
Pick a joint target for one arm, plan a collision-free path to it, scrub through the path, and commit it.

! EXPERIMENTAL, not yet tested thoroughly. Commit is a stub that only logs (`send_to_arm`).

* The flow (plan on a worker, preview, stale, commit) is `planning.panel.PlannerPlugin`.
  Here: the joint sliders, the arm choice, and a compas_fab world per robot (planner.py)
  holding the robot itself (SRDF and stitched tools), the other robots and every scene body.
! The first plan for a robot loads its models and the others' (several seconds).
! A plan is only good for where the robot was when planned: if the arm or base moves
  or the target changes it goes stale, stays on screen, and Commit refuses it.
"""

from __future__ import annotations

import math
import threading
from html import escape

import numpy as np

from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import register
from ...planning.panel import WAIT_MARGIN, PlannerPlugin, Search
from ...planning.search import TIME_LIMIT, PlanResult
from ...robot_interface.arm import UR_JOINT_LIMITS
from ...ui.ghost import RobotGhost
from ...ui.pybullet_window import PyBulletWindowToggle
from ...ui.style import SECTION_CTRL, SECTION_TOOL, block, chip, section, values
from .planner import ArmPath, ArmPlanningWorld, arm_joint_names, plan_arm

#: The plan goes stale once a joint moves this far from the planned start, or the base this far.
STALE_JOINT = math.radians(2.0)    # rad
STALE_POSITION = 0.05              # m
#: Extra time for the first plan of a robot, which loads the models, seconds.
LOAD_ALLOWANCE = 60.0

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
    ctx.log_warn(f"[STUB] would send a {len(path.points)}-waypoint path ({path.duration:.1f} s) "
                 f"to {serial} {arm_name}; nothing was sent")
    return True


@register
class ArmPlannerPlugin(PlannerPlugin):
    """Plans, shows and commits a collision-free joint path for one arm."""

    name = "arm_planner"
    experimental = True
    worker_name = "arm-plan"

    def __init__(self):
        """Start with no robot chosen, no target and no plan."""
        super().__init__()
        self.path: ArmPath | None = None
        #: The arm being planned for, and the arm and base position the plan was made for.
        self.arm: str | None = None
        self._path_arm: str | None = None
        self._path_base: np.ndarray | None = None
        #: Robots whose models the worker has loaded, so later plans need no load allowance.
        self._loaded: set[str] = set()
        # ! Worker thread only (sync, search, close); see planner.py.
        self._world = ArmPlanningWorld()
        #: The PyBullet window's state, read on the worker after a search.
        self._window_state = (False, "")
        #: Target slider values to set on the next update (radians), or None.
        self._load: np.ndarray | None = None
        #: Load the sliders from where the arm is on the next update (at startup, and on a new
        #: robot or arm), unless `_load` is set by then. ? Needs ctx, which update has.
        self._load_now = True
        #: The target slider values update wrote last. ! Writing a slider fires its callback
        #: too; a callback that finds these values is ours, not the operator's.
        self._written: tuple[float, ...] | None = None

    # --- --- --- --- --- SETUP --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Build the panel, and a target ghost and a path ghost per robot with arms.

        Args:
            ctx: This plugin's context.
        """
        arms = {robot.serial: [arm.name for arm in robot.arms] for robot in ctx.config.robots if robot.arms}
        if not arms:
            raise RuntimeError("no robots with arms configured; the arm planner has nothing to plan for")
        self._arms = arms
        self.serial = next(iter(arms))
        self.arm = arms[self.serial][0]

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

            self._add_path_controls(ctx, gui,
                                    plan_hint="Plan around the robot itself, the other robots and the scene, from "
                                              "where the arm is to the target; play or pause the preview; remove "
                                              "the plan (or stop planning)",
                                    commit_hint="Send the path to the arm (stub: logs only)")
            # * Debugging: watch the planning world, and the search moving the arm around in it.
            # ? The window shows the chosen robot's world. `self.serial` is read on the worker:
            #   a str, so at worst it is the robot chosen a moment later.
            self._window = PyBulletWindowToggle(
                ctx, gui, self._executor,
                lambda gui, snapshot: self._world.set_gui(gui, snapshot, self.serial),
                report=lambda message: self._report_problem(ctx, message))
            # ! Keep changing content BELOW the buttons, so nothing moves under the cursor.
            self._details = gui.add_html("")

        self._robot.on_update(ctx.defer_value("choose robot", lambda serial: self._choose(str(serial), None)))
        self._arm.on_update(ctx.defer_value("choose arm", lambda arm: self._choose(self.serial, str(arm))))
        for slider in self._sliders:
            slider.on_update(ctx.defer_value("target", lambda _value: self._target_changed()))
        load.on_click(ctx.defer_value("load target", lambda label: self._load_target(ctx, label)))

        self._add_ghosts(ctx, (config for config in ctx.config.robots if config.serial in arms))

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
            robot = next(robot for robot in ctx.config.robots if robot.serial == self.serial)
            arm = next(arm for arm in robot.arms if arm.name == self.arm)
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
        """Set robot, arm and target from the cell's selected step: its target, else its start."""
        found = self._cell_step(ctx)
        if found is None:
            return
        step, serial, robot_name = found
        if serial not in self._arms:
            self._say(f"{step.label} is for {robot_name}, which has no arms configured", failed=True)
            return
        movement = step.movement
        target = movement.target.joints.get(step.action.robot) if movement.target is not None else None
        given = target or movement.start.robots[step.action.robot].joints or {}
        # * The arm the step moves: the first of the robot's arms it names every joint of.
        arm = next((arm for arm in self._arms[serial] if all(name in given for name in arm_joint_names(arm))), None)
        if arm is None:
            self._say(f"{step.label} has no joint values for an arm of {robot_name}", failed=True)
            return
        self._choose(serial, arm)
        self._load = np.array([given[name] for name in arm_joint_names(arm)])
        self._has_target = True
        which = "target" if target else "start"
        self._say(f"target from the {which} of {step.label} ({arm})")

    # --- --- --- --- --- PLANNER PLUGIN --- --- --- --- ---

    def make_search(self, ctx: PluginContext, abort: threading.Event) -> Search:
        """Search from where the arm is in the tick's snapshot to the slider target."""
        serial, arm, goal = self.serial, self.arm, self._target()
        # * The world as it stood at the start of this tick; the arm starts where it is in it.
        snapshot = ctx.scene.snapshot
        base = np.array(snapshot.robots[serial].base.position)
        loading = serial not in self._loaded

        def work() -> PlanResult:
            mirror = self._world.sync(snapshot, serial)
            result = plan_arm(self._world, mirror, arm, goal, abort)
            self._window_state = self._world.window()
            return result

        def accepted(result: PlanResult) -> tuple[str, bool]:
            self._path_arm, self._path_base = arm, base
            how = "straight" if result.direct else "RRT"
            return (f"{how}: {len(self.path.points)} waypoints, {self.path.duration:.1f} s, "
                    f"found in {result.seconds:.1f} s"), False

        def finished() -> None:
            self._loaded.add(serial)
            self._window.show(*self._window_state)

        return Search(serial=serial, label=f"plan {serial} {arm}",
                      message=f"planning for {serial} {arm}…" + (" (loading the models first)" if loading else ""),
                      work=work, time_limit=TIME_LIMIT + WAIT_MARGIN + (LOAD_ALLOWANCE if loading else 0.0),
                      accepted=accepted, finished=finished)

    def stale_reason(self, ctx: PluginContext) -> str:
        """Stale once the arm or base moves away from the plan's start, or the target from its goal."""
        joints = ctx.kinematics.joints(self._path_serial)
        here = np.array([joints.get(name, 0.0) for name in arm_joint_names(self._path_arm)])
        base = np.array(ctx.kinematics.base_pose(self._path_serial).position)
        if np.max(np.abs(here - self.path.start)) > STALE_JOINT:
            return "the arm moved since planning"
        if np.linalg.norm(base[:2] - self._path_base[:2]) > STALE_POSITION:
            return "the base moved since planning"
        if np.max(np.abs(self._target() - self.path.goal)) > STALE_JOINT:
            return "the target changed since planning"
        return ""

    def send_plan(self, ctx: PluginContext) -> tuple[bool, str]:
        """Send the plan to the arm."""
        serial, arm = self._path_serial, self._path_arm
        if send_to_arm(ctx, serial, arm, self.path):
            return True, f"sent to {serial} {arm} (stub: nothing was sent)"
        return False, f"{serial} {arm} refused the path"

    def forget_plan(self) -> None:
        """Drop the arm and base position the plan was for."""
        self._path_arm, self._path_base = None, None

    def show_target(self, ctx: PluginContext, ghost: RobotGhost) -> None:
        """The chosen arm at the slider target."""
        self._show_arm(ctx, ghost, self.serial, self.arm, self._target())

    def show_path(self, ctx: PluginContext, ghost: RobotGhost) -> None:
        """The planned arm on the path at the slider time."""
        self._show_arm(ctx, ghost, self._path_serial, self._path_arm, self.path.sample(self._t))

    def _show_arm(self, ctx: PluginContext, ghost: RobotGhost, serial: str, arm: str, values) -> None:
        """Show only `arm` (with its tool) at joint `values`, the base where it is now."""
        joints = dict(ctx.kinematics.joints(serial))
        joints.update(zip(arm_joint_names(arm), values))
        ghost.show(ctx.kinematics.base_pose(serial), joints, parts=(f"{arm}_",))

    # --- --- --- --- --- READING --- --- --- --- ---

    def _target(self) -> np.ndarray:
        """The target from the sliders, radians."""
        return np.radians([slider.value for slider in self._sliders])

    def _joints_now(self, ctx: PluginContext) -> np.ndarray:
        """Where the chosen arm's joints are now, radians, UR driver order."""
        joints = ctx.kinematics.joints(self.serial)
        return np.array([joints.get(name, 0.0) for name in arm_joint_names(self.arm)])

    # --- --- --- --- --- TICK --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Apply queued target loads, then advance playback and check for staleness.

        Args:
            ctx: This plugin's context.
        """
        if self._load_now:
            self._load_now = False
            if self._load is None:
                self._load = self._joints_now(ctx)
        if self._load is not None:
            for slider, value in zip(self._sliders, np.degrees(self._load)):
                slider.value = round(float(value) * 2) / 2  # the sliders' 0.5 deg step
            self._written = tuple(slider.value for slider in self._sliders)
            self._load = None
        super().update(ctx)

    def draw(self, ctx: PluginContext) -> None:
        """Pose the ghosts and fill the panel.

        Args:
            ctx: This plugin's context.
        """
        self._status.content = block(chip(escape(f"{self.serial} {self.arm}"), SECTION_CTRL) + self._plan_chip())
        # Loading a target from the cell can switch the robot or arm; show that here.
        self._robot.value = self.serial
        if tuple(self._arm.options) != tuple(self._arms[self.serial]):
            self._arm.options = self._arms[self.serial]
        self._arm.value = self.arm
        self._details.content = self._details_html()
        self._draw_slider()
        self._draw_ghosts(ctx)

    def _details_html(self) -> str:
        """The plan and the joints at the slider time, and the last message. Fixed number of rows.

        Returns:
            str: HTML.
        """
        if self.path is None:
            path, at = "path    —", "at t    —"
        else:
            path = f"path    {len(self.path.points)} waypoints  {self.path.duration:5.1f} s"
            at = "at t    " + " ".join(f"{math.degrees(v):+6.1f}" for v in self.path.sample(self._t))
        return block(values(path, at) + self._message_html())
