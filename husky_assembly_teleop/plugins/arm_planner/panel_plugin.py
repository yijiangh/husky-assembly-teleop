"""
The arm planner's panel: pick a joint target for one arm, plan to it, scrub through the path, and commit it.

The flow is `shared.planner.planner_panel.PlannerPanel`; this adds the joint sliders and the arm choice.
! Drive the planner through its public API (`ArmPlannerContract`: set target, plan, cancel, results) wherever you can,
  so those paths stay ready for other users such as an executor; only debug drawing and internal state reach inside.
"""

from __future__ import annotations

import math
from html import escape

import numpy as np
import viser

from ...config import short_serial
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import register
from ...shared.arm_planner_contract import MAX_SLOWDOWN, ArmPlanRequest
from ...colors import TARGET
from ...shared.collisions_contract import CollisionsContract
from ...shared.planner.planner_panel import PLAN_TAB, PlannerPanel, unknown_robots_chip
from ...shared.planner.plan import Plan
from ...shared.planner.planner_search import with_guesses
from ...ui.choice_row import ChoiceRow
from ...ui.ghost import RobotGhost
from ...ui.joint_sliders import add_joint_sliders, joint_slider_angles, set_joint_sliders
from ...ui.pybullet_window import add_pybullet_window_toggle
from ...ui.style import BUSY, FAIL, GAP, NONE, OK, block, check_chip, chip, chip_line, section, values
from scaffolding_assembly_core.ids import robot_id
from ...shared.assembly_planning.joints import arm_joint_names
from .planner_plugin import STALE_JOINT, ArmPlannerPlugin
from ...world.arm_reach import out_of_reach
from ...world.clearance import ARM_STOP_DISTANCE, PLAN_DISTANCE

#: The Slow down row's choices: the plan at its fastest timing, up to MAX_SLOWDOWN times as long.
SLOWDOWNS = tuple(f"×{factor}" for factor in range(1, int(MAX_SLOWDOWN) + 1))


@register
class ArmPlannerPanel(PlannerPanel):
    """Joint sliders and the arm choice for the arm planner, with its plan, preview and commit controls."""

    name = "arm_planner_panel"
    experimental = True
    planner = "arm_planner"
    tab_path = (PLAN_TAB, "Arm")
    icon = viser.Icon.CRANE
    requires = ("arm_planner", "process", "collisions")

    def __init__(self):
        """Start with no robot chosen and no target."""
        super().__init__()
        self.arm: str | None = None
        #: Target slider values to set on the next update (radians), or None.
        self._load: np.ndarray | None = None
        #: Load the sliders from the arm's current joints on the next update, unless `_load` is set.
        self._load_now = True
        #: The plan whose collision, if any, the collisions panel shows.
        self._reported: Plan | None = None
        #: How many times its fastest timing a plan takes, as last set on the Duration slider; new plans take it too.
        self._slowdown = 1.0

    # --- --- --- --- --- SETUP --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Build the panel, and a target ghost and a path ghost per robot with arms."""
        self._connect(ctx)
        # * The concrete plugin, for the window below and the last motion's run chip (internal state).
        self._planner: ArmPlannerPlugin = ctx.require(self.planner)
        self._collisions: CollisionsContract = ctx.require("collisions")
        arms = {robot.serial: [arm.name for arm in robot.arms] for robot in ctx.config.robots if robot.arms}
        #: Each robot's arms' short labels ("Arm", or "Left" and "Right"), for the Arm row.
        self._arm_labels = {robot.serial: tuple(arm.label for arm in robot.arms) for robot in ctx.config.robots
                            if robot.arms}
        if not arms:
            self._set_up_idle(ctx, "no robots with arms configured: nothing to plan for")
            return
        self._arms = arms
        self.serial = next(iter(arms))
        self.arm = arms[self.serial][0]

        with ctx.view.ui() as gui:
            self._status = gui.add_html("")
            gui.add_html(GAP)
            self._robot = ChoiceRow(ctx, gui, "Robot", tuple(arms), self.serial,
                                    on_choose=lambda serial: self._choose(serial, None),
                                    labels=tuple(short_serial(serial) for serial in arms), height=None)
            self._arm = ChoiceRow(ctx, gui, "Arm", tuple(arms[self.serial]), self.arm,
                                  on_choose=lambda arm: self._choose(self.serial, arm),
                                  labels=self._arm_labels[self.serial], height=None)

            gui.add_html(section("target", TARGET))
            self._sliders = add_joint_sliders(gui)
            load = gui.add_button_group("Load", ["Now", "Stow", "Step"],
                                        hint="Set the target to where the arm is now, to its stow pose, or to "
                                             "the arm's target in the step selected in the process plugin")

            self._add_path_controls(ctx, gui,
                                    plan_hint="Plan from the arm's pose to the target around itself, the robots and "
                                              "the scene; Play previews; Clear drops it. Hold or Stop all stops a "
                                              "sent path.",
                                    commit_hint="Send the path to the arm; it stops at every waypoint",
                                    clearance_hint=f"Soft stop a committed path once the robot comes within "
                                                   f"{ARM_STOP_DISTANCE * 100:g} cm of anything, itself included; a "
                                                   f"start that close is refused. Plans keep "
                                                   f"{PLAN_DISTANCE * 100:g} cm, or as much as the start and target "
                                                   f"have; then the clearance is half that (an amber chip says so). "
                                                   f"Untick to move out of a tight spot.",
                                    before_commit=lambda gui: self._add_slowdown(ctx, gui))
            # ! Keep changing content BELOW the buttons, so nothing moves under the cursor.
            self._details = gui.add_html("")

            # Everything beyond plan and commit, folded away until wanted (as in the base planner's panel).
            with gui.add_folder("Advanced", expand_by_default=False):
                gui.add_html(section("debug", NONE))
                # * Watch the search in the chosen robot's planning world.
                # ! The one reach into the planner (the concrete plugin, not its API): the window runs on the
                #   planner's worker (`_executor`) and shows its world (`_world`).
                # ? `self.serial` is read on the worker; at worst it is the robot chosen a moment later.
                # * Guessed like a plan is, so an untracked robot still shows.
                planner = self._planner
                add_pybullet_window_toggle(
                    ctx, gui, planner._executor,
                    lambda gui, snapshot: planner._world.set_gui(
                        gui, with_guesses(snapshot, robot_id(self.serial))[0], self.serial))

        for slider in self._sliders:
            slider.on_update(ctx.defer("target", self._target_changed, operator_only=True))
        load.on_click(ctx.defer_value("load target", lambda label: self._load_target(ctx, label)))

        self._add_ghosts(ctx, (config for config in ctx.config.robots if config.serial in arms))

    def _add_slowdown(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """Add the Slow down row below the preview: the plan at its fastest timing, up to MAX_SLOWDOWN times as long."""
        self._slowdown_row = ChoiceRow(
            ctx, gui, "Slow down", SLOWDOWNS, SLOWDOWNS[0],
            "How many times as long as its fastest timing the plan takes. Re-times it without replanning; Play, "
            "Commit and the next plans follow it.",
            on_choose=lambda label: self._set_slowdown(float(label[1:])), height=None)

    # --- --- --- --- --- COMMANDS (intents, on the main thread) --- --- --- --- ---

    def _set_slowdown(self, factor: float) -> None:
        """Take `factor` times the fastest timing: the ready plan now (the preview keeps its place), and the next."""
        self._used.touch()
        self._slowdown = factor
        plan = self._api.plan
        if plan is None or plan.state != "ready":
            return
        before = plan.path.duration
        self._api.set_duration(plan.id, plan.path.fastest * factor)
        self._t *= self._api.plan.path.duration / before

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
        self._clear_plan()
        self._has_target, self._load_now = False, True

    def _target_changed(self) -> None:
        """The operator moved a target slider."""
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
            self._say(f"target set to where {short_serial(self.serial)} {self.arm} is now")
        elif label == "Stow":
            robot = next(robot for robot in ctx.config.robots if robot.serial == self.serial)
            arm = next(arm for arm in robot.arms if arm.name == self.arm)
            if arm.stow_joints is None:
                self._say(f"{self.arm} has no stow pose configured", FAIL)
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
            self._say(f"{label} is for {robot_name}, which has no arms configured", FAIL)
            return
        target = movement.target.joints.get(robot) if movement.target is not None else None
        given = target or movement.start.robots[robot].joints or {}
        # * The arm the step moves: the first of the robot's arms it names every joint of.
        arm = next((arm for arm in self._arms[serial] if all(name in given for name in arm_joint_names(arm))), None)
        if arm is None:
            self._say(f"{label} has no joint values for an arm of {robot_name}", FAIL)
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

    def commit_out_of_reach(self, ctx: PluginContext, plan: Plan[ArmPlanRequest]) -> frozenset[str]:
        """Other robots whose only unknown arms the planned arm cannot reach, whatever the joints (`out_of_reach`)."""
        return out_of_reach(ctx.scene.snapshot, robot_id(plan.request.serial), [plan.request.arm])

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
        return joint_slider_angles(self._sliders)

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
            set_joint_sliders(self._sliders, self._load)
            self._load = None
        super().update(ctx)
        plan = self._api.plan
        # * A new plan takes the slowdown last set; the preview starts at 0 anyway.
        if plan is not None and plan.state == "ready" and abs(plan.path.slowdown - self._slowdown) > 1e-6:
            self._api.set_duration(plan.id, plan.path.fastest * self._slowdown)
            plan = self._api.plan
        if plan is not self._reported:
            # * A plan refused for a collision shows it at once; any newer plan takes it away.
            self._reported = plan
            if plan is not None and plan.hit is not None:
                label = f"{short_serial(plan.request.serial)} {plan.request.arm} {plan.hit.where}"
                self._collisions.replace(self.name, {label: plan.hit}, shown=(label,))
            else:
                self._collisions.clear(self.name)

    def draw(self, ctx: PluginContext) -> None:
        """Pose the ghosts and fill the panel."""
        if self.idle:
            return
        self._status.content = self._status_html(ctx)
        # Loading a target from the step can switch the robot or arm; show that here.
        self._robot.value = self.serial
        self._arm.set_options(tuple(self._arms[self.serial]), self._arm_labels[self.serial])
        self._arm.value = self.arm
        self._details.content = self._details_html()
        self._draw_slider()
        self._slowdown_row.value = f"×{self._slowdown:g}"
        self._draw_ghosts(ctx)

    def _status_html(self, ctx: PluginContext) -> str:
        """One line of chips: the robot and arm and whether the base is tracked, the plan's state and the last run's."""
        tracked = ctx.scene.snapshot.robots[robot_id(self.serial)].base_tracked
        robot = chip(escape(f"{short_serial(self.serial)} {self.arm} {'tracked' if tracked else 'not tracked'}"),
                     OK if tracked else BUSY,
                     "mocap tracks the base: plans start from its live pose" if tracked
                     else "plans from where the base was last seen; a commit needs it tracked by mocap")
        # * The chosen arm's run chip replaces "sent", and stays while the next plan is made (as the base planner's).
        plan, motion = self._api.plan, self._planner._motions.get((self.serial, self.arm))
        chips = robot + (self._plan_chip() if plan is None or plan.state != "sent" else "")
        if motion is not None:
            chips += check_chip(motion)
        return chip_line(chips + self.clearance_chips()
                         + unknown_robots_chip(ctx.scene.snapshot, [robot_id(self.serial)]))

    def _details_html(self) -> str:
        """The plan, the joints at the slider time, and the last message, in a fixed number of rows."""
        plan = self._api.plan
        if plan is None or plan.path is None:
            path, at = "path    —", "at t    —"
        else:
            path = (f"path    {plan.path.duration:5.1f} s (x{plan.path.slowdown:.1f})  {plan.path.rests} rests on "
                    f"the way")
            at = "at t    " + " ".join(f"{math.degrees(v):+6.1f}" for v in plan.path.sample(self._t))
        return block(values(path, at) + self._message_html())
