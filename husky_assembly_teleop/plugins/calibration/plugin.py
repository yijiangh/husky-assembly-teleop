"""
The base calibration as a guided procedure: one step at a time (`steps/`); Done goes to the work that is left.

! Every motion runs under the robot's lease, and while its Clearance box is ticked under a clearance (`ARM_MOVE` or
  `BASE_DRIVE`) that refuses a start too close; a take also freezes the robot's other arm. Free drive, hand guided,
  runs under none.
! The tool body must sit on the flange of the chosen arm, and the base must stand where the sweeps were checked.
! Work in progress, not yet tried on a robot: a banner on both tabs says so (`WIP_BANNER`).

The phone page gets a Calibration tab: the same overview, chips and Back / Next / Done, with the controls used next to
the robot (driving, free drive, the tool pad, Check collisions, Record tip pose and punch take); moves that wait for Go
start on the laptop.

Run with:  -p plugins:="['calibration']"
"""

from __future__ import annotations

import viser

from ...config import short_serial
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...shared.collisions_contract import CollisionsContract
from ...shared.guide.panel import GuidePanel
from ...shared.teleop.dpad import PHONE_DPAD_STYLE
from ...shared.teleop.tool_pad import ICONS as TOOL_PAD_ICONS
from ...ui.ghost import RecentUse, RobotGhost, robot_ghosts
from ...ui.phone import big_buttons
from ...ui.style import BUSY, FAIL, block, chip, check_chip
from ...colors import WHITE
from ...world.arm_reach import out_of_reach
from ...world.checks import GOOD
from ...world.clearance import ARM_STOP_DISTANCE
from ...world.mocap import mocap_check
from scaffolding_assembly_core.ids import robot_id
from .run import Run
from .steps import STEPS
from .steps.base import FAILS, NONE_YET, TOOL_NAME, WORK, Calibration, CalibrationStep
from .steps.s1_setup import Setup
from .robot.sweeps import load_sweeps, named


#: The phone tab's buttons, finger-sized: the D-pad, free drive, the tool pad, the record buttons.
#: ! Sized by icon on the whole phone page (`big_buttons`): another tab's button with one of these icons grows too.
PHONE_STYLE = (PHONE_DPAD_STYLE + f"<style>{big_buttons([viser.Icon.HAND_GRAB], 64, 28)}"
               f"{big_buttons(list(TOOL_PAD_ICONS.values()), 52, 20)}"
               f"{big_buttons([viser.Icon.PLAYER_RECORD, viser.Icon.REFRESH, viser.Icon.SHIELD_CHECK], 56, 24)}"
               "</style>")
#: Always at the top of the tab and the phone tab: the plugin is unfinished.
WIP_BANNER = block(f'<div style="background:{FAIL};color:{WHITE};font-weight:700;font-size:15px;padding:8px 10px;'
                   'border-radius:6px;text-align:center">⚠ WORK IN PROGRESS ⚠<div style="font-size:11px;'
                   'font-weight:600">Not yet tried on a robot: watch every move, check every result before using it.'
                   '</div></div>')
#: The body definitions without a relay: one object, so `update` sees nothing new on each tick.
NO_DEFINITIONS: dict = {}


@register
class CalibrationPlugin(HuskyPlugin):
    """Calibrates a robot's base rigid body, guided step by step."""

    name = "calibration"
    icon = viser.Icon.TARGET
    requires = ("arm_planner", "collisions")
    experimental = True

    def setup(self, ctx: PluginContext) -> None:
        """Build the steps and their panel, the ghost and the side panel; choose the first arm with sweeps."""
        hits: CollisionsContract = ctx.require("collisions")
        self.cal = Calibration(hits, self.switch_run, self.show_work_left)
        self.cal.procedure.out_of_reach = self._out_of_reach
        # Whether the run took new body definitions whose files were not moved yet (a procedure ran).
        self._untidy = False
        choices = [f"{short_serial(serial)} {arm}" for serial, robot in ctx.world.robots.items() for arm in robot.arms
                   if self._has_sweeps(ctx, serial, arm)]
        self.steps: list[CalibrationStep] = [Setup(self.cal, choices)] + [step(self.cal) for step in STEPS[1:]]
        self.cal.steps = {step.name: step for step in self.steps}
        self.panel = GuidePanel(
            "base calibration", self.steps, self.cal.procedure, on_done=self._done,
            auto_hint="Go without asking: moves, sweeps and validation poses; Record then goes on with the next take. "
                      "Cancel stops.")
        with ctx.view.ui() as gui:
            gui.add_html(WIP_BANNER)
            self.panel.build(ctx, gui)
        with ctx.view.phone("Calibration", viser.Icon.TARGET) as phone:
            phone.add_html(PHONE_STYLE)
            phone.add_html(WIP_BANNER)
            self.panel.build_phone(ctx, phone)
        self._ghosts: dict[str, RobotGhost] = robot_ghosts(ctx, [robot.config for robot in ctx.world.robots.values()],
                                                           "target")
        # * One plugin's ghosts at a time (`RecentUse`): the ghost counts as used whenever what it shows changes.
        self._used, self._shown = RecentUse(ctx.config.ghost_timeout), None
        self.cal.results.build(ctx)
        if choices:
            self.steps[0].choose(ctx, choices[0])

    @staticmethod
    def _has_sweeps(ctx: PluginContext, serial: str, arm: str) -> bool:
        try:
            load_sweeps(ctx.config.data_directory, serial[-4:], arm)
        except KeyError:
            return False
        return True

    def update(self, ctx: PluginContext) -> None:
        """Give the run Motive's body definitions whenever the relay announces new ones (`Run.follow`).

        Files are moved to and from archive/ only while no procedure runs: a take may be under way.
        """
        run, relay = self.cal.run, ctx.world.mocap_relay
        if run is None:
            return
        definitions = relay.state.body_definitions if relay is not None else NO_DEFINITIONS
        if definitions is not run.definitions:
            run.follow(definitions, tidy=False)
            self._untidy = True
            run.save()
        if self._untidy and not self.cal.procedure.running:
            self._untidy = False
            moved = run.tidy()
            if moved:
                ctx.log_warn(f"a body definition changed in Motive: {len(moved)} files of this run out of date "
                             f"(moved to archive/)")
            run.save()

    def teardown(self, ctx: PluginContext) -> None:
        """Stop tracking the tool body."""
        ctx.untrack_object(TOOL_NAME)

    # --- --- --- --- --- RUNS AND STEPS --- --- --- --- ---

    def switch_run(self, ctx: PluginContext, run: Run) -> None:
        """Make `run` the chosen one: track its tool body, and show it in every step; stay at Setup.

        Another run of the same arm keeps the checklist's ticks: they are about the arm, not the run.
        """
        previous = self.cal.run
        if previous is not None and (previous.serial, previous.arm) == (run.serial, run.arm):
            run.adopt_checklist(previous)
        self.cal.run = run
        self.cal.track_tool(ctx, run.tool_body)
        self.cal.hits.clear(self.name)
        for step in self.steps:
            step.run_changed(ctx)

    def work_left(self, ctx: PluginContext, from_here: bool = False) -> int:
        """The step with work left: the first result step whose state leaves work (`WORK`); the last step if none.

        If that step (the target) needs preconditions that have expired, the earliest of them comes first. `from_here`
        (Done) while the current step is one of them: the next expired one after it, else the target, so a
        precondition left expired never holds the operator.
        """
        work = next((index for index, step in enumerate(self.steps)
                     if not step.precondition and step.state(ctx).label in WORK), None)
        if work is None:
            return len(self.steps) - 1
        names = [step.name for step in self.steps]
        needs = self.steps[work].needs
        after = self.panel.index if from_here and self.panel.current.name in needs else -1
        target = self.steps[work]
        expired = [names.index(name) for name in needs
                   if names.index(name) > after and target.need_expired(ctx, self.cal.steps[name]) is not None]
        return min(expired, default=work)

    def show_work_left(self, ctx: PluginContext) -> str:
        """Go to the step with work left (`work_left`), as on resuming a run; return its name."""
        index = self.work_left(ctx)
        if index != self.panel.index:
            self.panel.show(ctx, index)
        return self.steps[index].name

    def _done(self, ctx: PluginContext, anyway: bool) -> int:
        """Done: let the step keep what it made, save, and go to the work left (else the next step).

        Pressed again on a failing result, or a result step's missing one (skipped), it accepts it (`Run.accept`): Done
        elsewhere no longer leads back to it. A result out of date is left as it is, unstamped.
        """
        run, step, here = self.cal.run, self.panel.current, self.panel.index
        if run is None:
            return here
        label = step.state(ctx).label
        if anyway and (label == FAILS or label == NONE_YET and not step.precondition):
            run.accept(step.name, step.judged_under(ctx))
        step.done(ctx)
        run.save()
        index = self.work_left(ctx, from_here=True)
        return min(here + 1, len(self.steps) - 1) if index == here else index

    # --- --- --- --- --- DRAWING --- --- --- --- ---

    def draw(self, ctx: PluginContext) -> None:
        """The panel, the ghost and the side panel."""
        run = self.cal.run
        self.panel.draw(ctx, f"{short_serial(run.serial)} {run.arm}" if run else "no arm", self._live(ctx))
        self._draw_ghost(ctx)
        self.cal.driving.draw()
        self.cal.hand.draw()
        self.cal.results.draw()

    def _live(self, ctx: PluginContext) -> str:
        """Chips for the current step's live problems: Go too close, a watched body's mocap, a soft stop, a lease."""
        run, tool, procedure, step = self.cal.run, self.cal.tool, self.cal.procedure, self.panel.current
        # * Watched where Go moves the arm, until a procedure moves it: Go's Clearance refuses a start too close.
        goes = run is not None and step.go_moves and (procedure.waiting or not procedure.running)
        margin = ARM_STOP_DISTANCE if procedure.go_margin is None else procedure.go_margin
        procedure.clearance.watch(ctx, run.serial if goes else None, margin)
        if run is None:
            return ""
        now = ctx.now()
        html = procedure.clearance.start_chip("Go") if goes else ""
        live = []
        if "base" in step.watches:
            live.append(mocap_check("base", run.robot.base.mocap_id, run.robot.base.state, now))
        if "tool" in step.watches and tool is not None:
            live.append(mocap_check("tool", tool.mocap_id, tool, now))
        html += "".join(check_chip(check) for check in live if check.level != GOOD)
        if not step.moves_robot:
            return html
        if run.robot.base.soft_stopped:
            html += chip("soft stopped", FAIL, "the robot is soft stopped: Resume it in Health")
        lease = run.robot.lease_slot.lease
        # * The lease only when it is in the way: held by someone else, not the planner moving for us.
        if lease is not None and not lease.holder.startswith(self.name) and not procedure.running:
            html += chip("leased", BUSY, f"held by {lease.holder}: the calibration cannot move the robot")
        return html

    def _out_of_reach(self, ctx: PluginContext) -> frozenset[str]:
        """Other robots whose only unknown arms the run's arm cannot reach: Go lets them be unknown without asking."""
        run = self.cal.run
        if run is None or robot_id(run.serial) not in ctx.scene.snapshot.robots:
            return frozenset()
        return out_of_reach(ctx.scene.snapshot, robot_id(run.serial), [run.arm])

    def _draw_ghost(self, ctx: PluginContext) -> None:
        """Show the arm where a procedure's preview stands, else where the step previews it; hide it otherwise.

        Shown only while this was used last of all the plugins with ghosts (`RecentUse`): a new preview, Preview time
        moved, another step, or the ghost's joints changing counts as use.
        """
        run = self.cal.run
        ghost = self._ghosts.get(run.serial) if run is not None else None
        if ghost is None:
            return
        procedure = self.cal.procedure
        joints = procedure.previewed()
        if joints is None:
            joints = self.panel.current.preview_joints(ctx)
        shown = (procedure.preview_path, procedure.preview, self.panel.index,
                 None if joints is None else tuple(joints))
        if shown != self._shown:
            self._shown = shown
            self._used.touch()
        if joints is None or not self._used.active:
            ghost.hide()
            return
        ghost.show(ctx.kinematics.base_pose(run.serial), {**dict(ctx.kinematics.joints(run.serial)),
                                                          **named(run.arm, joints)})
