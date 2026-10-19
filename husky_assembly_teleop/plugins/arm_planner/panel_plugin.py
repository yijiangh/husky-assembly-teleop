"""
The arm planner's panel: pick a joint target for one arm, plan to it, scrub through the path, and commit it.

The flow is `shared.planning.planner_panel.PlannerPanel`; this adds the joint sliders and the arm choice.
! Drive the planner through its public API (`ArmPlannerContract`: set target, plan, cancel, results) wherever you can,
  so those paths stay ready for other users such as an executor; only debug drawing and internal state reach inside.
"""

from __future__ import annotations

import math
from html import escape

import numpy as np

from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import register
from ...robot_interface.arm import UR_JOINT_LIMITS
from ...shared.planning.planner_contract import ArmPlanRequest
from ...shared.planning.planner_panel import PlannerPanel
from ...shared.planning.planner_contract import Plan
from ...shared.planning.planner_search import with_guesses
from ...ui.ghost import RobotGhost
from ...ui.pybullet_window import add_pybullet_window_toggle
from ...ui.style import SECTION_CTRL, SECTION_TOOL, block, chip, section, values
from scaffolding_assembly_core.ids import robot_id
from .planner import arm_joint_names
from .planner_plugin import STALE_JOINT

#: Short joint names for the sliders, UR driver order.
SLIDER_NAMES = ("pan", "lift", "elbow", "w1", "w2", "w3")


@register
class ArmPlannerPanel(PlannerPanel):
    """Joint sliders and the arm choice for the arm planner, with its plan, preview and commit controls."""

    name = "arm_planner_panel"
    experimental = True
    planner = "arm_planner"
    requires = ("arm_planner", "process")

    def __init__(self):
        """Start with no robot chosen and no target."""
        super().__init__()
        self.arm: str | None = None
        #: Target slider values to set on the next update (radians), or None.
        self._load: np.ndarray | None = None
        #: Load the sliders from the arm's current joints on the next update, unless `_load` is set.
        self._load_now = True
        #: The slider values update wrote last, so their callback is not taken for the operator's.
        self._written: tuple[float, ...] | None = None

    # --- --- --- --- --- SETUP --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Build the panel, and a target ghost and a path ghost per robot with arms."""
        self._connect(ctx)
        arms = {robot.serial: [arm.name for arm in robot.arms] for robot in ctx.config.robots if robot.arms}
        if not arms:
            self._set_up_idle(ctx, "no robots with arms configured: nothing to plan for")
            return
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
            load = gui.add_button_group("Load", ["Now", "Stow", "Step"],
                                        hint="Set the target to where the arm is now, to its stow pose, or to "
                                             "the arm's target in the step selected in the process plugin")

            self._add_path_controls(ctx, gui,
                                    plan_hint="Plan around the robot itself, the other robots and the scene, from "
                                              "where the arm is to the target; play or pause the preview; remove "
                                              "the plan (or stop planning)",
                                    commit_hint="Send the path to the arm; it stops at every waypoint")
            # * Debugging: watch the search in the chosen robot's planning world.
            # ! The one reach into the planner (the concrete plugin, not its API): the window runs on the planner's
            #   worker (`_executor`) and shows its world (`_world`).
            # ? `self.serial` is read on the worker; at worst it is the robot chosen a moment later.
            # * Guessed like a plan is, so an untracked robot still shows.
            planner = ctx.require(self.planner)
            add_pybullet_window_toggle(
                ctx, gui, planner._executor,
                lambda gui, snapshot: planner._world.set_gui(
                    gui, with_guesses(snapshot, robot_id(self.serial))[0], self.serial))
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
        self._api.clear()
        self._has_target, self._load_now = False, True

    def _target_changed(self) -> None:
        """A target slider moved: by the operator, unless update just wrote these values."""
        if tuple(slider.value for slider in self._sliders) == self._written:
            return
        self._used.touch()
        self._has_target = True

    def _load_target(self, ctx: PluginContext, label: str) -> None:
        """Set the target to the arm now, its stow pose, or the process plugin's selected step.

        Args:
            ctx: This plugin's context.
            label: "Now", "Stow" or "Step".
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
            self._load_from_step(ctx)
            return
        self._has_target = True

    def _load_from_step(self, ctx: PluginContext) -> None:
        """Set robot, arm and target from the process plugin's selected step: its target, else its start."""
        found = self._process_step(ctx)
        if found is None:
            return
        selection, serial, robot_name = found
        movement, robot, label = selection.movement, selection.action.robot, selection.movement.id
        if serial not in self._arms:
            self._say(f"{label} is for {robot_name}, which has no arms configured", failed=True)
            return
        target = movement.target.joints.get(robot) if movement.target is not None else None
        given = target or movement.start.robots[robot].joints or {}
        # * The arm the step moves: the first of the robot's arms it names every joint of.
        arm = next((arm for arm in self._arms[serial] if all(name in given for name in arm_joint_names(arm))), None)
        if arm is None:
            self._say(f"{label} has no joint values for an arm of {robot_name}", failed=True)
            return
        self._choose(serial, arm)
        self._load = np.array([given[name] for name in arm_joint_names(arm)])
        self._has_target = True
        which = "target" if target else "start"
        self._say(f"target from the {which} of {label} ({arm})")

    # --- --- --- --- --- PLANNER PANEL --- --- --- --- ---

    def make_request(self, ctx: PluginContext) -> ArmPlanRequest:
        """Plan the chosen arm to the slider target."""
        return ArmPlanRequest(self.serial, self.arm, tuple(float(value) for value in self._target()))

    def target_change(self, ctx: PluginContext, plan: Plan[ArmPlanRequest]) -> str:
        """How the chosen arm or the sliders differ from the plan's request."""
        request = plan.request
        if (self.serial, self.arm) != (request.serial, request.arm):
            return "the robot or arm changed since planning"
        if np.max(np.abs(self._target() - np.array(request.goal))) > STALE_JOINT:
            return "the target changed since planning"
        return ""

    def show_target(self, ctx: PluginContext, ghost: RobotGhost) -> None:
        """The chosen arm at the slider target."""
        self._show_arm(ctx, ghost, self.serial, self.arm, self._target())

    def show_path(self, ctx: PluginContext, ghost: RobotGhost, plan: Plan[ArmPlanRequest]) -> None:
        """The planned arm on the path at the slider time."""
        self._show_arm(ctx, ghost, plan.request.serial, plan.request.arm, plan.path.sample(self._t))

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
        """Apply queued target loads, then take the planner's news and advance playback."""
        if self.idle:
            return
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
        """Pose the ghosts and fill the panel."""
        if self.idle:
            return
        self._status.content = block(chip(escape(f"{self.serial} {self.arm}"), SECTION_CTRL) + self._plan_chip())
        # Loading a target from the step can switch the robot or arm; show that here.
        self._robot.value = self.serial
        if tuple(self._arm.options) != tuple(self._arms[self.serial]):
            self._arm.options = self._arms[self.serial]
        self._arm.value = self.arm
        self._details.content = self._details_html()
        self._draw_slider()
        self._draw_ghosts(ctx)

    def _details_html(self) -> str:
        """The plan, the joints at the slider time, and the last message, in a fixed number of rows."""
        plan = self._api.plan
        if plan is None or plan.path is None:
            path, at = "path    —", "at t    —"
        else:
            path = f"path    {len(plan.path.points)} waypoints  {plan.path.duration:5.1f} s"
            at = "at t    " + " ".join(f"{math.degrees(v):+6.1f}" for v in plan.path.sample(self._t))
        return block(values(path, at) + self._message_html())
