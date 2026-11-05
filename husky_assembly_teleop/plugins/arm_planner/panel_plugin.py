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
from scaffolding_assembly_core.robot import FLANGE_SUFFIX
from ...shared.assembly_planning.joints import arm_joint_names
from .planner_plugin import STALE_JOINT, ArmPlannerPlugin
from ...world.arm_reach import out_of_reach
from ...world.clearance import ARM_STOP_DISTANCE, PLAN_DISTANCE

#: The Slow down row's choices: the plan at its fastest timing, up to MAX_SLOWDOWN times as long.
SLOWDOWNS = tuple(f"×{factor}" for factor in range(1, int(MAX_SLOWDOWN) + 1))
#: The Arm row's choice that plans every arm of a two-arm robot in one search; its label.
BOTH, BOTH_LABEL = "both", "Both"
#: Load's step buttons: the step's joints as the process gives them, or its tool0 frame solved by IK at the live base.
STEP_JOINTS, STEP_IK = "Step joints", "Step IK"


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
        #: Each arm's target while Both is chosen: Load sets it, the sliders only show the first arm's.
        self._both: dict[str, np.ndarray] = {}
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
            options, labels = self._arm_options(self.serial)
            self._arm = ChoiceRow(ctx, gui, "Arm", options, self.arm,
                                  on_choose=lambda arm: self._choose(self.serial, arm), labels=labels, height=None,
                                  hint="The arm to plan for; Both plans every arm in one search, its target from "
                                       "Load (Now, Stow, Step joints, Step IK)")

            gui.add_html(section("target", TARGET))
            self._sliders = add_joint_sliders(gui)
            load = gui.add_button_group("Load", ["Now", "Stow", STEP_JOINTS, STEP_IK],
                                        hint="Set the target to where the arm is now, to its stow pose, to the "
                                             "joints the process gives the selected step (Step joints), or to its "
                                             "tool0 frame solved by IK at the live base, the solution nearest those "
                                             "joints (Step IK)")

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

    def _arm_choices(self, serial: str) -> tuple[str, ...]:
        """A robot's choices on the Arm row: its arms, and Both for a robot with two or more."""
        arms = tuple(self._arms[serial])
        return arms + (BOTH,) if len(arms) > 1 else arms

    def _arm_options(self, serial: str) -> tuple[tuple[str, ...], tuple[str, ...]]:
        """A robot's choices on the Arm row (`_arm_choices`) and their labels."""
        labels = tuple(self._arm_labels[serial])
        return self._arm_choices(serial), labels + (BOTH_LABEL,) if len(labels) > 1 else labels

    @property
    def planned_arms(self) -> tuple[str, ...]:
        """tuple[str, ...]: The arms a plan moves: every arm with Both chosen, else the chosen one."""
        return tuple(self._arms[self.serial]) if self.arm == BOTH else (self.arm,)

    def _set_targets(self, targets: dict[str, np.ndarray]) -> None:
        """Take a target per planned arm: the sliders show the first; with Both, each arm's is kept."""
        if self.arm == BOTH:
            self._both = {arm: np.asarray(targets[arm], dtype=float) for arm in self.planned_arms}
        self._load = np.asarray(targets[self.planned_arms[0]], dtype=float)

    def _targets(self) -> dict[str, np.ndarray]:
        """The target of each planned arm, radians, UR driver order."""
        if self.arm == BOTH:
            return dict(self._both)
        return {self.arm: self._target()}

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
        arm = arm if arm in self._arm_choices(serial) else self._arms[serial][0]
        if (serial, arm) == (self.serial, self.arm):
            return
        self.serial, self.arm = serial, arm
        self._clear_plan()
        self._has_target, self._load_now, self._both = False, True, {}
        if arm == BOTH:
            self._say("both arms in one search: the target comes from Load (Now, Stow, Step joints, Step IK); the "
                      "sliders show the "
                      "first arm's, greyed")

    def _target_changed(self) -> None:
        """The operator moved a target slider."""
        self._used.touch()
        self._has_target = True

    def _load_target(self, ctx: PluginContext, label: str) -> None:
        """Set the target to the arm now, its stow pose, or the process plugin's selected step.

        Args:
            ctx: This plugin's context.
            label: "Now", "Stow", STEP_JOINTS or STEP_IK.
        """
        self._used.touch()
        what = " + ".join(self.planned_arms)
        if label == "Now":
            self._set_targets({arm: self._joints_now(ctx, arm) for arm in self.planned_arms})
            self._say(f"target set to where {short_serial(self.serial)} {what} is now")
        elif label == "Stow":
            robot = next(robot for robot in ctx.config.robots if robot.serial == self.serial)
            stows = {arm.name: arm.stow_joints for arm in robot.arms if arm.name in self.planned_arms}
            missing = [arm for arm, joints in stows.items() if joints is None]
            if missing:
                self._say(f"{', '.join(missing)} has no stow pose configured", FAIL)
                return
            self._set_targets({arm: np.array(joints) for arm, joints in stows.items()})
            self._say(f"target set to the stow pose of {what}")
        else:
            self._load_from_step(ctx, solve=label == STEP_IK)
            return
        self._has_target = True

    def _load_from_step(self, ctx: PluginContext, solve: bool) -> None:
        """Set robot, arm and target from the process plugin's selected step.

        - Step joints (`solve` False): the joints the process gives the step's target (else its start), as authored.
        - Step IK (`solve` True): the step's tool0 frame solved by IK at the live base, the solution nearest those
          joints, so a move planned here ends where the step's own move would, not a base error away. Only for a step
          whose target gives the arm's tool0 a world pose (a transfer, an approach); the target is set once IK finds it.

        The arm chosen stays chosen when the step moves it; else the first arm the step names.
        """
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
        named = [arm for arm in self._arms[serial] if all(name in given for name in arm_joint_names(arm))]
        if not named:
            self._say(f"{label} has no joint values for an arm of {robot_name}", FAIL)
            return
        authored = next((each for each in selection.process.actions[selection.action.id].movements
                         if each.id == movement.id), movement)
        both = self.arm == BOTH and serial == self.serial and set(self._arms[serial]) <= set(named)
        arms = tuple(self._arms[serial]) if both else (
            self.arm if serial == self.serial and self.arm in named else named[0],)
        joints = {arm: np.array([given[name] for name in arm_joint_names(arm)]) for arm in arms}
        what = "both arms" if both else arms[0]
        others = [] if both else [each for each in named if each != arms[0]]
        also = f"; it moves {', '.join(others)} too: choose Both and load again" if others else ""
        if not both:
            self._choose(serial, arms[0])
        if not solve:
            self._set_targets(joints)
            self._has_target = True
            which = "target" if target else "start"
            self._say(f"target = the process's joints at the {which} of {label} ({what}){also}")
            return
        # * The authored movement's tool0 pose (the shown transit with the transfer workaround has joints only).
        poses = {arm: authored.target.link_poses.get(f"{robot}/{arm}{FLANGE_SUFFIX}") if authored.target else None
                 for arm in arms}
        missing = [arm for arm, pose in poses.items() if pose is None]
        if not target or missing:
            self._say(f"{label} gives no tool0 frame for {', '.join(missing or arms)}: use Step joints", FAIL)
            return
        guess = {name: float(value) for arm, values in joints.items()
                 for name, value in zip(arm_joint_names(arm), values)}
        ctx.spawn(f"solve {label} {what}", self._solve(serial, poses, guess, label, what, also))

    async def _solve(self, serial: str, poses: dict, guess: dict, label: str, what: str, also: str) -> None:
        """Set the targets to the step's tool0 frames solved at the live base, if found and the arms are still chosen.

        Args:
            serial: The robot.
            poses: Each arm's tool0 world pose from the step.
            guess: The step's joints by name; the solution nearest them wins.
            label: The step's id, for the line.
            what: The arm, or "both arms".
            also: The note on other arms the step moves, or "".
        """
        arm = BOTH if len(poses) > 1 else next(iter(poses))
        self._say(f"Step IK {label} ({what}): solving its tool0 frame at the live base…", BUSY)
        joints, line = await self._api.solve_goals(serial, poses, guess)
        if (self.serial, self.arm) != (serial, arm):
            return
        if joints is None:
            self._say(f"Step IK {label} ({what}): {line}; target not set (Step joints loads the process's joints)",
                      FAIL)
            return
        self._set_targets({each: np.array([joints[name] for name in arm_joint_names(each)]) for each in poses})
        self._has_target = True
        change = max(abs(joints[name] - value) for name, value in guess.items())
        self._say(f"target = Step IK of {label} ({what}): {line}, {math.degrees(change):.1f}° from the process's "
                  f"joints{also}")

    # --- --- --- --- --- PLANNER PANEL --- --- --- --- ---

    def make_request(self, ctx: PluginContext) -> ArmPlanRequest:
        """Plan the chosen arm, or both in one search, to the target."""
        arms, targets = self.planned_arms, self._targets()
        goals = {arm: tuple(float(value) for value in targets[arm]) for arm in arms}
        return ArmPlanRequest(self.serial, arms[0], goals[arms[0]], also=tuple((arm, goals[arm]) for arm in arms[1:]))

    def target_change(self, ctx: PluginContext, plan: Plan[ArmPlanRequest]) -> str:
        """How the chosen arms or their targets differ from the plan's request."""
        request = plan.request
        if (self.serial, self.planned_arms) != (request.serial, request.arms):
            return "the robot or arm changed since planning"
        targets = self._targets()
        if any(arm not in targets for arm in request.arms):
            return "no target for every arm"
        now = np.concatenate([targets[arm] for arm in request.arms])
        if np.max(np.abs(now - np.array(request.goals))) > STALE_JOINT:
            return "the target changed since planning"
        return ""

    def commit_out_of_reach(self, ctx: PluginContext, plan: Plan[ArmPlanRequest]) -> frozenset[str]:
        """Other robots whose only unknown arms the planned arm cannot reach, whatever the joints (`out_of_reach`)."""
        return out_of_reach(ctx.scene.snapshot, robot_id(plan.request.serial), list(plan.request.arms))

    def show_target(self, ctx: PluginContext, ghost: RobotGhost) -> None:
        """The chosen arms at their targets."""
        targets = self._targets()
        if all(arm in targets for arm in self.planned_arms):
            self._show_arms(ctx, ghost, self.serial, self.planned_arms,
                            np.concatenate([targets[arm] for arm in self.planned_arms]))

    def show_path(self, ctx: PluginContext, ghost: RobotGhost, plan: Plan[ArmPlanRequest]) -> None:
        """The planned arms on the path at the slider time."""
        self._show_arms(ctx, ghost, plan.request.serial, plan.request.arms, plan.path.sample(self._t))

    def _show_arms(self, ctx: PluginContext, ghost: RobotGhost, serial: str, arms, values) -> None:
        """Show only `arms` (with their tools) at joint `values`, six per arm in order, the base where it is now."""
        joints = dict(ctx.kinematics.joints(serial))
        joints.update(zip([name for arm in arms for name in arm_joint_names(arm)], values))
        ghost.show(ctx.kinematics.base_pose(serial), joints, parts=tuple(f"{arm}_" for arm in arms))

    # --- --- --- --- --- READING --- --- --- --- ---

    def _target(self) -> np.ndarray:
        """The target from the sliders, radians."""
        return joint_slider_angles(self._sliders)

    def _joints_now(self, ctx: PluginContext, arm: str | None = None) -> np.ndarray:
        """Where an arm's joints are now (the first planned arm's by default), radians, UR driver order."""
        joints = ctx.kinematics.joints(self.serial)
        return np.array([joints.get(name, 0.0) for name in arm_joint_names(arm or self.planned_arms[0])])

    # --- --- --- --- --- TICK --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Apply queued target loads, then take the planner's news and advance playback."""
        if self.idle:
            return
        if self._load_now:
            self._load_now = False
            if self._load is None:
                self._set_targets({arm: self._joints_now(ctx, arm) for arm in self.planned_arms})
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
        self._arm.set_options(*self._arm_options(self.serial))
        self._arm.value = self.arm
        # * With Both the sliders only show the first arm's target: Load sets every arm's.
        for slider in self._sliders:
            if slider.disabled != (self.arm == BOTH):
                slider.disabled = self.arm == BOTH
        self._details.content = self._details_html()
        self._draw_slider()
        self._slowdown_row.value = f"×{self._slowdown:g}"
        self._draw_ghosts(ctx)

    def _status_html(self, ctx: PluginContext) -> str:
        """One line of chips: the robot and arm and whether the base is tracked, the plan's state and the last run's."""
        tracked = ctx.scene.snapshot.robots[robot_id(self.serial)].base_tracked
        arms = " + ".join(self.planned_arms)
        robot = chip(escape(f"{short_serial(self.serial)} {arms} {'tracked' if tracked else 'not tracked'}"),
                     OK if tracked else BUSY,
                     "mocap tracks the base: plans start from its live pose" if tracked
                     else "plans from where the base was last seen; a commit needs it tracked by mocap")
        # * The chosen arm's run chip replaces "sent", and stays while the next plan is made (as the base planner's).
        plan, motion = self._api.plan, self._planner._motions.get((self.serial, self.planned_arms[0]))
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
