"""
The assembly guide: the process part by part, Prepare → Check → Go, each part run by the layer below it.

- Drives go through `base_planner` (the robot's follower drives); every arm move, servo, insert, retreat, gripper
  step and step by hand through `movement_exec`, planned by `motion_planner`; Mark done through `assembly_run`.
- Before each action of a robot, every other robot holding no bar parks (Settings › Park idle robots, on): its arms
  stow on `arm_planner`, then it drives to its parked pose on `base_planner` (`parts.park_parts`). Parks are optional:
  one that fails warns and is left "not parked", Done leaves one with one press, and none holds a later part back.
- A part that finds no goal at the live base flags the drive before it "not good": that drive is to do again, and its
  Prepare drives even when the base stands within the arrival tolerance.
- Prepare and Go are for parts that move a robot. A part by hand is confirmed by ticking its boxes and Done (the
  executor records it); Mark done is Done too. Prepare and Go stay grey there.
- A part is done by its result: the executor's finished movements (`MovementExecContract.finished`, whoever sent
  them), the run's mark, a drive ended at its goal. A part fixed by hand one layer down (Plan › Base, Plan › Arm,
  Control) is at its end: Done goes on with one press; anything else asks twice (Skip).
- One part is shown, and Prepare, Go and Done act on it, as in the calibration guide; Back and Next move it. The next
  part to do is not kept: it is the first part with no result. A procedure belongs to the part that started it:
  shown elsewhere, the guide line names it. Redo from here makes a part and every later one to do again.
- Going out of order (earlier parts of the action not done) asks a second press of Go or Done.
- ! Judgements may be wrong: each refusal of one gives way to a second press ("trust me"), logged as a warning. Prepare
  twice does a done part again (only it, also in an action the run has marked), Done twice confirms a part by hand
  with boxes unticked, or marks an action whose robot is not live (the run assumes the export's end) or still counts
  as moving. Only nothing loaded, something running, and the God gate refuse for good. A procedure that only waits
  for Go is replaced by Prepare or Done on any part, Auto going on.
- The guide keeps the process selection on the part shown, so a panel's Load (Step; Step joints, Step IK) loads its
  target, and sets the run's current step to a part before it plans or records it (the executor checks a plan
  against it).
- Modes: Off (the operator presses Prepare, Go and Done), Auto (works on the next part to do from the part shown when
  it was switched on, shows it, prepares and sends it, waiting at each question and at each part by hand), God (Auto
  that answers every question and ticks every box too: only while every robot and the relay are simulated). Auto and
  God end on a failure, a refusal, Cancel, a soft stop of any robot, another plugin's lease, a part picked in the
  process panel, the parts built again (a setting, the process), or a Mark done with doubts; the guide line says why.
- ! The guide holds no lease itself: the planner or the executor does while a Go runs.

Run with:  -p plugins:="['assembly_guide', 'base_planner_panel', 'arm_planner_panel', 'robot_control',
                          'stop_recovery', ...]"
"""

from __future__ import annotations

import asyncio
import math
from dataclasses import replace
from html import escape
from typing import TYPE_CHECKING

import viser

from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id

from ...colors import ACTION, GREY, TEAL, rgb
from ...config import find_robot_serial, short_serial
from ...plugin_api.concurrency import WaitTimeout
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...shared.assembly_planning.joints import arm_joint_names, turn_limits, unwrap_to_nearest
from ...shared.assembly_planning.movement import Routine, UnknownMovement, classify
from ...shared.arm_planner_contract import ArmPlanRequest
from ...shared.process_scene.live_scene import showing
from ...shared.assembly_planning.route_result import arm_path_route, goal_line, pose_error
from ...shared.assembly_planning.step_plan import live_goal_poses, timed_route
from ...shared.assembly_planning.tamp_route import IK_GOALS
from ...shared.assembly_run_contract import is_live, live_problem
from ...shared.base_planner_contract import BasePlanRequest
from ...shared.execution.basis import STALE_BASE, basis_of, stale_reason
from ...shared.execution.moving import moving_reason, robot_moving
from ...shared.execution.operator import PROMPT_ARM_S
from ...shared.guide.panel import (overview_row, running_chip, set_changed, state_chip, step_buttons_css,
                                   step_header)
from ...shared.guide.procedure import Procedure
from ...shared.motion_planner_contract import MotionPlanRequest
from ...shared.movement_exec_contract import DEFAULT_TRAJ_TIME, DONE, DRY_RUN, MOTION_KINDS, ExecRequest, serial_of
from ...shared.planner.planner_panel import SLIDER_STEP
from ...shared.runs.runs import DONE as MARKED
from ...ui.choice_row import ChoiceRow
from ...ui.ghost import RecentUse, RobotGhost, robot_ghosts
from ...ui.second_press import SECOND_PRESS_SECONDS, SecondPress, again_label
from ...ui.style import (BUSY, CURRENT, FAIL, OK, SECTION_CTRL, block, check_chip, chip, chip_line,
                         chip_row, note, section, warning)
from ...world.arm_reach import out_of_reach
from ...world.checks import BAD, GOOD, INFO, PENDING, WARN, Check
from .kinds import KINDS, HandKind, Kind, description
from .parts import (ARM_ARRIVED, DRIVE, EXECUTED, GRIPPER, HAND, INSERT, MARK, MOVE, NOTHING_TO_MOVE, PARK, REFUSED,
                    RETREAT, SERVO, GuidePart, arm_arrived, base_arrived, floor_pose, schedule_parts)

if TYPE_CHECKING:
    from ...shared.arm_planner_contract import ArmPlannerContract
    from ...shared.assembly_run_contract import AssemblyRunContract
    from ...shared.base_planner_contract import BasePlannerContract
    from ...shared.motion_planner_contract import MotionPlannerContract
    from ...shared.movement_exec_contract import MovementExecContract
    from ...shared.planner.timed_path import TimedPath
    from ...shared.process_contract import ProcessContract
    from ...shared.runs.runs import RunSettings

#: The modes.
OFF, AUTO, GOD = "Off", "Auto", "God"
#: Seconds a commit's motion may take beyond twice its planned duration; a mark may take to show in the run.
MOTION_MARGIN, MARK_WAIT = 60.0, 3.0
#: Seconds a selection asked for may take to show in the process plugin before the guide follows it again.
SELECT_WAIT = 2.0
#: Plans of a free search Auto and God try before they give up; the routines planned by one.
AUTO_PLAN_TRIES = 3
FREE_SEARCHES = (Routine.FREE_TO_LOAD, Routine.TRANSFER_FREE, Routine.TRANSFER_HELD, Routine.FREE_HOME,
                 Routine.SUPPORT_APPROACH, Routine.SUPPORT_LEAVE)
#: The free moves the arm planner plans, one arm or both together (`arm_planner_moves`), and its name in the records;
#: a transfer holding the bar stays the motion planner's. ! No fallback: where the arm planner finds no goal or path,
#: the motion planner would neither, after minutes; the part fails at once.
FREE_MOVES = (Routine.TRANSFER_FREE, Routine.FREE_HOME, Routine.FREE_TO_LOAD, Routine.SUPPORT_APPROACH,
              Routine.SUPPORT_LEAVE)
ARM_PLANNER = "arm_planner.rrt_connect"
#: The planner chips' labels (`_planner_check`).
ARM_PLANNER_NAME, MOTION_PLANNER_NAME = "arm planner", "motion planner"
#: Seconds the robots may take to stand still before an executor part.
STILL_WAIT = 20.0
#: Seconds God waits beyond the executor's double-click guard before it confirms a question.
GOD_PAUSE = 0.3
#: Back, Next and Redo from here: Back and Next as narrow arrows (`step_buttons_css`).
REDO = "Redo from here"
NAV_STYLE = f"<style>{step_buttons_css(44)}</style>"
#: Seconds a stow's end may take to show in the snapshot.
STOW_SETTLE = 2.0
#: What Prepare says for each kind that moves a robot.
PREPARE_LABELS = {DRIVE: "Prepare: plan the drive", MOVE: "Prepare: plan the move",
                  PARK: "Prepare: plan the stow, then the drive", SERVO: "Prepare: plan the first pass",
                  INSERT: "Prepare: plan the insert", RETREAT: "Prepare: plan the retreat",
                  GRIPPER: "Prepare: ask to run it"}
#: Seconds a refusal that no second press overrides stays on the panel (a second press's question stays as long as
#: its window).
REFUSAL_KEPT = 8.0
#: Why Auto ends once nothing is left to do.
ALL_DONE = "every part is done"


def simulation_problem(ctx: PluginContext) -> str:
    """Why God may not run: a robot's base, arm or tool is real, or the relay reads Motive; "" when all is simulated.

    Each part counts as simulated only while its announcements keep coming (`simulated_parts`).
    """
    relay = ctx.world.mocap_relay
    if relay is None:
        return "no mocap relay runs"
    if not relay.state.simulated:
        return "the mocap relay reads Motive, not a simulation"
    for serial, robot in ctx.world.robots.items():
        if robot.base.simulation is None:
            return f"{serial}'s base is real"
        for name, arm in robot.arms.items():
            if arm.simulation is None:
                return f"{serial} {name} is real"
            tool = arm.config.end_effector_namespace
            if tool and robot.simulated.get(tool) is None:
                return f"{serial} {name}'s tool is real"
    return ""


@register
class AssemblyGuidePlugin(HuskyPlugin):
    """Walks the process part by part: Prepare plans, Go runs it through the layer below, Done goes on."""

    name = "assembly_guide"
    icon = viser.Icon.COMPASS
    tab_path = ("Assembly guide",)
    requires = ("process", "assembly_run", "base_planner", "arm_planner", "motion_planner", "movement_exec")
    experimental = True

    def setup(self, ctx: PluginContext) -> None:
        """Build the panel, a kind object per kind of part, and a ghost per robot; no process yet."""
        self._ctx = ctx
        self.procedure = Procedure()
        #: Every scheduled action's parts; the part shown, which Prepare, Go and Done act on.
        self.parts: list[GuidePart] = []
        self.index = 0
        #: The part whose procedure runs (it started it); the part by hand Done (or God) confirmed.
        self._owner: GuidePart | None = None
        self._by_done: GuidePart | None = None
        #: Action id -> its robot's serial, None by hand or not configured.
        self.serials: dict[str, str | None] = {}
        #: The parts done by the guide's own judgement (a drive, Done, a skip), by index; those skipped among them.
        self._done: set[int] = set()
        self._skipped: set[int] = set()
        #: Parts Redo from here made to do again while their robot stood at their end: standing there, measured live,
        #: is no result for them until they ran again, Done accepted them, or the robot, live, left that end (`update`).
        self._reopened: set[int] = set()
        #: Parts whose result is not good, by index, as their result chip: a park that did not get there, a drive the
        #: next part finds no goal from.
        self._flagged: dict[int, Check] = {}
        #: What the parts were built from; the selection last followed; a selection asked for (action, movement, at).
        self._built: tuple | None = None
        self._followed: tuple[str, str] | None = None
        self._asked: tuple[str, str, float] | None = None
        #: The run folder the guide's own judgements belong to, and each action's mark as last seen.
        self._run_folder = None
        self._marks: dict[str, str] = {}
        self.mode = OFF
        #: The last press that did nothing, when, and for how long it shows (`refusal`).
        self._refusal: tuple[Check, float, float] | None = None
        #: Why Auto or God last ended, shown on the guide line until the mode is chosen again or a part starts.
        self._auto_end: Check | None = None
        #: How each part's last procedure ended, by index, when it said something (failed, cancelled, a warning): it
        #: outlives showing another part.
        self._outcomes: dict[int, Check] = {}
        #: The part whose procedure waiting for Go a press replaces on purpose: its cancellation does not end Auto.
        self._replacing: GuidePart | None = None
        #: The Mark done whose robot never stood still in time, and the one Done twice then marks without waiting.
        self._unsettled: GuidePart | None = None
        self._mark_now: GuidePart | None = None
        #: Serial -> the robot's name in the process, for the operator's texts (`_who`).
        self._names: dict[str, str] = {}
        #: The part Auto last started, and the last part the guide ran to its end (Auto stops if it shows no result).
        self._auto_at: GuidePart | None = None
        self._auto_from = 0
        self._ran: GuidePart | None = None
        #: The part the last plan was for, the joints it ends at (joint name -> value), and the base it was planned at,
        #: for "at its end": a goal solved at one base says nothing at another.
        self._goal: tuple[GuidePart, dict[str, float], Pose] | None = None
        #: The part the last executor plan was for, and its basis: for the "plan stale" chip while it waits for Go.
        self._basis: tuple | None = None
        #: The part last planned and the planner that planned it, for its planner chip.
        self._planned: tuple[GuidePart, str] | None = None
        self.goal_text = ""
        self.traj_time = 10.0
        self.ignore_built = False
        #: Whether free moves are planned by the arm planner (else the motion planner).
        self.arm_planner_moves = True
        #: Park every robot an action does not use before it (`parts.park_parts`): the "Park idle robots" box.
        self.park_idle = True
        #: What the preview shows: ("base", serial) or ("joints", serial, joint names, base pose).
        self._previewing: tuple | None = None
        self._done_press, self._redo_press, self._auto_press = SecondPress(), SecondPress(), SecondPress()
        self._order_press, self._again_press = SecondPress(), SecondPress()
        #: The part Prepare twice does again, and the Mark done that marks with its robot not live.
        self._forced: GuidePart | None = None
        self._assume: GuidePart | None = None
        self._playing, self._played_at, self._preview_path = False, None, None
        self._ghosts: dict[str, RobotGhost] = robot_ghosts(ctx, [robot.config for robot in ctx.world.robots.values()],
                                                           "path")
        self._used, self._shown = RecentUse(ctx.config.ghost_timeout), None
        self.kinds: dict[str, Kind] = {kind.name: kind(self) for kind in KINDS}
        self.procedure.out_of_reach = self._out_of_reach
        self._build(ctx)

    def _build(self, ctx: PluginContext) -> None:
        """The panel, top to bottom; nothing above the kinds' folders changes height."""
        with ctx.view.ui() as gui:
            gui.add_html(NAV_STYLE)
            self._title = gui.add_html("")
            # * The part shown, always three lines: where, its name, its goal (`step_header`).
            self._header = gui.add_html("")
            # * The shown action's parts at a glance, one line.
            self._overview = gui.add_html("")
            # * Back and Next show another part; nothing else changes, what runs goes on.
            with gui.add_folder("Parts"):
                self._back = gui.add_button("Back", color=rgb(GREY), icon=viser.Icon.CHEVRON_LEFT,
                                            hint="Show the part before and select its movement in the process panel; "
                                                 "what runs goes on")
                self._next = gui.add_button("Next", color=rgb(GREY), icon=viser.Icon.CHEVRON_RIGHT,
                                            hint="Show the next part and select its movement in the process panel; "
                                                 "what runs goes on")
                self._redo = gui.add_button(REDO, icon=viser.Icon.RESTORE,
                                            hint="Make the part shown, done, and every later part to do again: the run "
                                                 "reopens their actions, the executor forgets what it finished. "
                                                 "Nothing moves. Press twice")
            # * The shown part's chips, a line each: what it needs, how it stands now, its result; then what runs.
            self._chips = gui.add_html("")
            self._guide_line = gui.add_html("")
            self._mode = ChoiceRow(ctx, gui, "Mode", (OFF, AUTO, GOD), OFF, height=None,
                                   hint="Off: you press Prepare, Go and Done. Auto: from the part shown on, "
                                        "prepares each part and goes on by itself; in simulation it sends them, on "
                                        "real robots it waits for your Go at every drive and move; it waits at each "
                                        "question, each part by hand and each warning. God (simulation only): Auto "
                                        "that answers every question and ticks every box too. Auto and God end on a "
                                        "failure, Cancel, a soft stop, another plugin's lease, Redo or a pick in the "
                                        "process panel; the guide line says why",
                                   on_choose=lambda option: self.set_mode(ctx, option))
            self._prepare = gui.add_button("Prepare", color=rgb(TEAL), icon=viser.Icon.ROUTE,
                                           hint="Get a part that moves a robot ready from where the robots are now: "
                                                "plan it, or open the executor's question; nothing moves")
            self._go = gui.add_button("Go", color=rgb(OK), icon=viser.Icon.PLAYER_PLAY,
                                      hint="Move the robot: send the previewed plan, or answer the executor's question "
                                           "before it moves; e-stop in hand")
            self._done_button = gui.add_button("Done", color=rgb(ACTION), icon=viser.Icon.CHECKS,
                                               hint="Confirm a part by hand (every box ticked) or Mark done; on a "
                                                    "finished part, go on. A part with no result yet asks once more "
                                                    "(Skip)")
            self._cancel = gui.add_button("Cancel", color=rgb(FAIL), icon=viser.Icon.PLAYER_STOP,
                                          hint="Stop what the guide runs: the robot stops, its arms hold; Auto "
                                               "ends")
            self._preview = gui.add_slider("Preview time", min=0.0, max=1.0, step=SLIDER_STEP, initial_value=0.0,
                                           hint="Scrub the plan Go sends, seconds from its start; the ghost shows it")
            self._play = gui.add_button("Play", icon=viser.Icon.PLAYER_TRACK_NEXT,
                                        hint="Play the plan in real time from the Preview time; again to pause")
            self.procedure.clearance.add(ctx, gui, "guide")
            # * The kinds' own controls, one folder each, the shown part's visible: below every button.
            self._folders, self._folder_shown = {}, None
            for name, kind in self.kinds.items():
                folder = gui.add_folder(name.capitalize(), visible=False)
                with folder:
                    kind.build(ctx, gui)
                self._folders[name] = folder
            # ! Text that comes and goes stays below the buttons.
            self._notes = gui.add_html("")
            self._description = gui.add_html("")
            self._list = gui.add_html("")
            with gui.add_folder("Settings", expand_by_default=False):
                ignore = gui.add_checkbox("Ignore built bars", False,
                                          hint="Plan as if the built bars were not there: for tests in the air")
                ignore.on_update(ctx.defer_value("ignore built bars", lambda value: setattr(
                    self, "ignore_built", bool(value)), operator_only=True))
                one_arm = gui.add_checkbox("Free moves on the arm planner", True,
                                           hint="A transfer with empty tools, a way home, an approach, a leave, one "
                                                "arm or both together: planned by the arm planner (1 cm from "
                                                "everything; Plan › Arm shows the plan); a part it finds no path for "
                                                "fails, without trying the motion planner. Off: always the motion "
                                                "planner")
                one_arm.on_update(ctx.defer_value("one-arm moves", lambda value: setattr(
                    self, "arm_planner_moves", bool(value)), operator_only=True))
                park = gui.add_checkbox("Park idle robots", True,
                                        hint="Before each action of a robot, every other robot holding no bar stows "
                                             "its arms and drives to its parked pose (park.yaml), so it is never in "
                                             "the way; parked already, it has nothing to do. Off: robots stay where "
                                             "they were left")
                park.on_update(ctx.defer_value("park idle robots", lambda value: setattr(
                    self, "park_idle", bool(value)), operator_only=True))
        self.procedure.confirm.button = self._go
        self._back.on_click(ctx.defer("back", lambda: self.look(ctx, -1)))
        self._next.on_click(ctx.defer("next", lambda: self.look(ctx, 1)))
        self._redo.on_click(ctx.defer("redo", lambda: self.redo(ctx)))
        self._prepare.on_click(ctx.defer("prepare", lambda: self.prepare(ctx)))
        self._go.on_click(ctx.defer("go", lambda: self.press_go(ctx)))
        self._done_button.on_click(ctx.defer("done", lambda: self.done(ctx)))
        self._cancel.on_click(ctx.defer("cancel", lambda: self.cancel(ctx)))
        self._preview.on_update(ctx.defer_value("preview", self._scrub, operator_only=True))
        self._play.on_click(ctx.defer("play", self._toggle_play))

    def teardown(self, ctx: PluginContext) -> None:
        """Stop what runs; hide the ghosts."""
        self.procedure.cancel()
        for ghost in self._ghosts.values():
            ghost.hide()

    # --- --- --- --- --- WHAT THE KINDS READ --- --- --- --- ---

    @property
    def part(self) -> GuidePart | None:
        """The part shown, which Prepare, Go and Done act on; None without a process."""
        return self.parts[self.index] if self.parts else None

    @property
    def serial(self) -> str | None:
        """The shown part's robot's serial; None by hand, not configured or without a process."""
        return self.serial_for(self.part) if self.part is not None else None

    def serial_for(self, part: GuidePart) -> str | None:
        """The serial of the robot a part acts on: a park part's own, else its action's; None by hand or unknown."""
        return part.robot or self.serials.get(part.action)

    @property
    def owner(self) -> GuidePart | None:
        """The part whose procedure runs; None while none runs."""
        return self._owner if self.procedure.running else None

    def next_to_do(self, ctx: PluginContext, start: int = 0) -> int | None:
        """The first part from `start` on with no result; None once every one is done."""
        return next((index for index in range(start, len(self.parts))
                     if not self.is_done(ctx, self.parts[index], index)), None)

    def undone_before(self, ctx: PluginContext, index: int, parks: bool = False) -> list[int]:
        """The parts of the same action before part `index` that have no result yet; the parks only if `parks`.

        * Parks are optional: one not done never holds a later part back.
        """
        first, _count = self._action_span(self.parts[index].action)
        return [each for each in range(first, index) if (self.parts[each].kind == PARK) == parks
                and not self.is_done(ctx, self.parts[each], each)]

    @property
    def can_rehearse(self) -> bool:
        """bool: Whether Rehearse can act now: Mode Off, a part shown, nothing running."""
        return self.mode == OFF and self.part is not None and not self.procedure.running

    @property
    def settings(self) -> RunSettings:
        """RunSettings: The run's settings."""
        return self._run(self._ctx).view.settings

    def request_settings(self, **changes) -> None:
        """Change run settings: the run keeps them, the Assembly tab shows them."""
        self._run(self._ctx).request_settings(**changes)

    def set_traj_time(self, seconds: float) -> None:
        """The motion time of the next plan, seconds."""
        self.traj_time = seconds

    def is_done(self, ctx: PluginContext, part: GuidePart, index: int) -> bool:
        """Whether a part's result is there: the executor finished it, the run marked it, or the guide judged it.

        Every part of an action the run marked done is done: the mark outlives a restart, the executor's memory does
        not. Redo from here (or Reopen in the Assembly tab) redoes them.
        """
        if index in self._done or self._run(ctx).view.statuses.get(part.action) == MARKED:
            return True
        if part.kind == MARK:
            return False
        if part.kind in EXECUTED and part.done_keys:
            loaded = self._run(ctx).view.process
            return part.done_keys <= self._executor(ctx).finished(loaded, part.action)
        if part.kind == PARK:
            # * Its result is where the robot stands: one parked already has nothing to do (unless redone).
            return index not in self._reopened and self.at_end(ctx, part)
        # * A flagged drive is to do again, even when later parts ran.
        return part.kind == DRIVE and index not in self._flagged and self._later_ran(ctx, index)

    def _later_ran(self, ctx: PluginContext, index: int) -> bool:
        """Whether the executor finished a later part of the same action: the drive before it happened then.

        * The guide's own record of a drive does not outlive a restart; the executor's, read back from the run, does.
        """
        action, loaded = self.parts[index].action, self._run(ctx).view.process
        finished = self._executor(ctx).finished(loaded, action)
        return any(part.kind in EXECUTED and part.done_keys and part.done_keys <= finished
                   for part in self.parts[index + 1:] if part.action == action)

    def at_end(self, ctx: PluginContext, part: GuidePart, tolerance: float = ARM_ARRIVED) -> bool:
        """Whether the robot stands where a drive or a move ends, measured live, whoever moved it there.

        `tolerance`: per arm joint; the guide skips a move by itself only within NOTHING_TO_MOVE.
        """
        serial = self.serial_for(part)
        robot = ctx.scene.snapshot.robots.get(robot_id(serial)) if serial else None
        if robot is None or not is_live(robot, ctx.now()):
            return False
        if part.kind == DRIVE:
            return base_arrived(ctx.kinematics.base_pose(serial), part.base)
        if part.kind == PARK:
            return self.stowed(ctx, part, tolerance) and base_arrived(ctx.kinematics.base_pose(serial), part.base)
        if part.kind == MOVE:
            goal = self._move_goal(ctx, part)
            return bool(goal) and arm_arrived([robot.joints.get(name) for name in goal], list(goal.values()),
                                              tolerance)
        return False

    def stowed(self, ctx: PluginContext, part: GuidePart, tolerance: float = ARM_ARRIVED) -> bool:
        """Whether every arm of a park part's robot stands at its stow joints, measured; False without a reading."""
        robot = ctx.scene.snapshot.robots.get(robot_id(part.robot))
        return robot is not None and all(
            arm_arrived([robot.joints.get(name) for name in arm_joint_names(arm)], goal, tolerance)
            for arm, goal in part.stow)

    def goal_error(self, ctx: PluginContext, part: GuidePart) -> float | None:
        """The largest distance of a move's joints from its goal, radians; None without a goal or live joints."""
        goal = self._move_goal(ctx, part)
        serial = self.serial_for(part)
        robot = ctx.scene.snapshot.robots.get(robot_id(serial)) if serial else None
        if not goal or robot is None:
            return None
        joints = [robot.joints.get(name) for name in goal]
        if any(value is None for value in joints):
            return None
        return max(abs(value - wanted) for value, wanted in zip(joints, goal.values()))

    def _move_goal(self, ctx: PluginContext, part: GuidePart) -> dict[str, float] | None:
        """The goal the last plan for `part` ends at while the base stands where it was planned; else the design's.

        ! A goal solved at the live base (`IK_GOALS`: a transfer, an approach) holds only at that base: none once the
          base moved, and never the design's, which is for the design's base.
        """
        serial = self.serial_for(part)
        if self._goal is not None and self._goal[0] is part and serial is not None:
            distance, turn = pose_error(ctx.kinematics.base_pose(serial), self._goal[2])
            if distance <= STALE_BASE[0] and turn <= STALE_BASE[1]:
                return self._goal[1]
        if part.routine in IK_GOALS:
            return None
        return self._authored_goal(ctx, part)

    def _authored_goal(self, ctx: PluginContext, part: GuidePart) -> dict[str, float] | None:
        """The joints a move ends at, as the design gives them; None if it gives none."""
        loaded = self._run(ctx).view.process
        action = loaded.process.actions[part.action]
        try:
            spec = classify(loaded.process, action, action.movements[part.movement_index],
                            self._process(ctx).transfer_as_free)
        except UnknownMovement:
            return None
        goal = spec.goal_joints or {}
        names = [name for arm in spec.arms for name in arm.joints]
        return {name: goal[name] for name in names} if names and all(name in goal for name in names) else None

    # --- --- --- --- --- FOLLOWING THE PROCESS --- --- --- --- ---

    def follow(self, ctx: PluginContext) -> None:
        """Build the parts for a new process; show a movement the operator picked in the process panel."""
        process = self._process(ctx)
        loaded, selection = self._run(ctx).view.process, process.selection
        if loaded is None or selection is None:
            self.parts, self._built, self._followed, self.index = [], None, None, 0
            return
        # ! The loaded process by identity: comparing it would walk the whole process.
        new = self._built is None or loaded is not self._built[0] or self._built[1:] != (process.transfer_as_free,
                                                                                         self.park_idle)
        if new and self._built is not None and self.procedure.running:
            return  # * Built again once what runs ends: its part's index must hold until then.
        picked = (selection.action.id, selection.movement.id)
        if self._asked is not None:
            action, movement, at = self._asked
            if picked != (action, movement) and ctx.now() - at < SELECT_WAIT:
                return
            self._asked = None
            if not new:
                self._followed = picked
        if new:
            if self._built is not None:
                self._end_auto(ctx, "the parts were built again (a setting or the process changed): choose Auto "
                                    "again from the part to go on")
            self._built, self._followed = (loaded, process.transfer_as_free, self.park_idle), picked
            self._names = {serial: robot.name for robot in loaded.process.robots.values()
                           if (serial := find_robot_serial(ctx.config.robots, robot.serial or ""))}
            self._outcomes.clear()
            self._goal = self._basis = None
            self.parts = schedule_parts(loaded, ctx.config.robots, process.transfer_as_free, self.park_idle)
            self.serials = {action: serial_of(loaded, action, ctx.config.robots) for action in loaded.process.schedule}
            self._done.clear()
            self._skipped.clear()
            self._flagged.clear()
            self._reopened.clear()
            self._show(ctx, self._part_of(ctx, *picked), select=False)
        elif picked != self._followed:
            self._followed = picked
            self._end_auto(ctx, "a movement was picked in the process panel")
            self._show(ctx, self._part_of(ctx, *picked), select=False)

    def _forget_redone(self, ctx: PluginContext) -> None:
        """Forget what was done in a new run, and in an action reopened (by the guide or the Assembly tab)."""
        view = self._run(ctx).view
        if view.folder != self._run_folder:
            # * A run's folder appears with its first record: from none to one is the same run.
            if self._run_folder is not None:
                self._done.clear()
                self._skipped.clear()
                self._flagged.clear()
                self._reopened.clear()
                ctx.log_info("guide: another run: every part is to do again")
            self._run_folder, self._marks = view.folder, dict(view.statuses)
            return
        for action, status in view.statuses.items():
            if self._marks.get(action) == MARKED and status != MARKED:
                self._executor(ctx).request_forget(action)
                redone = {index for index, part in enumerate(self.parts) if part.action == action}
                self._done -= redone
                self._skipped -= redone
                self._flagged = {index: flag for index, flag in self._flagged.items() if index not in redone}
                ctx.log_info(f"guide: {action} reopened: its parts are to do again")
        self._marks = dict(view.statuses)

    def _part_of(self, ctx: PluginContext, action: str, movement: str) -> int:
        """The first part of `movement` in `action` not done yet, else its first part; the current one if none."""
        matching = [index for index, part in enumerate(self.parts)
                    if (part.action, part.movement) == (action, movement)]
        undone = [index for index in matching if not self.is_done(ctx, self.parts[index], index)]
        return (undone or matching or [self.index])[0]

    def _show(self, ctx: PluginContext, index: int, select: bool = True) -> None:
        """Show part `index`; with `select`, select its movement too while nothing runs (`_select`)."""
        if index != self.index:
            self._keep_outcome()
            self.procedure.outcome = None
            self._done_press.forget()
            self._redo_press.forget()
            self._order_press.forget()
            self._again_press.forget()
        self.index = index
        part = self.parts[index]
        if part.kind == MOVE and self.owner is None:
            self.traj_time = DEFAULT_TRAJ_TIME.get(part.routine, self.traj_time)
        if select and not self.procedure.running:
            self._select(ctx, part)

    def _select(self, ctx: PluginContext, part: GuidePart) -> None:
        """Make a part's movement the run's current one, which selects it in the process plugin.

        Once it shows there, the guide follows the selection again.

        ! The run's current step is what the executor checks a plan against: set it before planning the part.
        """
        selection, run = self._process(ctx).selection, self._run(ctx)
        current = run.view.current
        if (selection is not None and (selection.action.id, selection.movement.id) == (part.action, part.movement)
                and current == (part.action, part.movement_index)):
            # * Shown already, e.g. by the run's Mark done: the guide's own, not the operator's pick.
            self._followed = (part.action, part.movement)
            return
        self._asked = (part.action, part.movement, ctx.now())
        run.request_set_current(part.action, part.movement_index)

    # --- --- --- --- --- THE BUTTONS --- --- --- --- ---

    def look(self, ctx: PluginContext, step: int) -> None:
        """Show the part `step` before or after the one shown; what runs goes on."""
        if self.parts:
            self._show(ctx, min(max(self.index + step, 0), len(self.parts) - 1))

    def set_mode(self, ctx: PluginContext, mode: str) -> None:
        """Choose Off, Auto or God; God only in a full simulation, Auto on real robots only on a second press."""
        if mode == GOD and (problem := simulation_problem(ctx)):
            self._refuse_mode(ctx, f"God only runs in a full simulation: {problem}")
            return
        if mode == AUTO and simulation_problem(ctx) and not self._auto_press.press(ctx.now(), "auto"):
            self._refuse_mode(ctx, f"Auto on real robots prepares each drive and move and waits for your Go at each; "
                                   f"area clear, e-stop in hand: press Auto again within {SECOND_PRESS_SECONDS:g} s",
                              ask=True)
            return
        # * Auto starts at the part shown: a run may begin anywhere in the schedule.
        self.mode, self._auto_at, self._ran, self._auto_from = mode, None, None, self.index
        self._mode.value = mode
        self.procedure.auto = mode != OFF
        self.procedure.outcome = None
        self._auto_end = None
        self._record(ctx, "mode", self.part, mode=mode)
        ctx.log_info(f"guide: mode {mode}" + (f" from {self.part.label}" if mode != OFF and self.part else ""))

    def _refuse_mode(self, ctx: PluginContext, why: str, ask: bool = False) -> None:
        """Keep the mode, and say why; `ask`: a second press goes on."""
        self._mode.value = self.mode
        self._refuse(ctx, "Mode", why, ask)

    def _refuse(self, ctx: PluginContext, button: str, why: str, ask: bool = False) -> None:
        """Say why a press did nothing: a chip on the guide line and a line below the buttons.

        `ask`: the press asks for a second one, shown as long as that may come; else a refusal, shown REFUSAL_KEPT s.
        """
        label = f"{button.lower()}: press again" if ask else f"{button.lower()} refused"
        self._refusal = (Check(label, WARN, why), ctx.now(), SECOND_PRESS_SECONDS if ask else REFUSAL_KEPT)
        ctx.log_info(f"guide: {button} {'asks again' if ask else 'refused'}: {why}")

    def refusal(self, now: float) -> Check | None:
        """The last press that did nothing, while it is shown."""
        if self._refusal is None or now - self._refusal[1] > self._refusal[2]:
            return None
        return self._refusal[0]

    def _cannot(self, ctx: PluginContext, button: str) -> str:
        """Why Prepare, Done or Redo from here cannot act on the part shown now; "" when it can.

        * A procedure that only waits for Go does not hold Prepare or Done back: they replace it (`_start`, `done`).
        """
        part, owner = self.part, self.owner
        if part is None:
            return "no process: Load one in the Assembly tab"
        if owner is not None and not self.procedure.waiting:
            return f"{owner.label} is busy ({self.procedure.doing or 'running'}): wait, or Cancel"
        if button == REDO:
            statuses = self._run(ctx).view.statuses
            if not any(self.is_done(ctx, each, index) or index in self._skipped or index in self._flagged
                       or statuses.get(each.action) == MARKED
                       for index, each in enumerate(self.parts[self.index:], self.index)) \
                    and not (part.kind == HAND and self._hand.count[0]):
                return "nothing from this part on is done: nothing to redo"
            if owner is not None:
                return f"{owner.label} waits for Go: Cancel it first"
            if active := self._executor(ctx).active:
                return f"the executor runs {active}: Cancel it first"
        elif button == "Prepare" and self.kinds[part.kind].manual:
            return "nothing to prepare: Done confirms this part"
        return ""

    def _end_auto(self, ctx: PluginContext, why: str) -> None:
        """Back to Off; the guide line says why until the mode is chosen again or a part starts. Nothing if Off."""
        if self.mode == OFF:
            return
        finished = why == ALL_DONE
        self._auto_end = Check(f"{self.mode.lower()} stopped", GOOD if finished else WARN, why)
        (ctx.log_info if finished else ctx.log_warn)(f"guide: {self.mode} ends: {why}")
        self._record(ctx, "mode", self.part, mode=OFF, ended=self.mode, why=why)
        self.mode, self.procedure.auto = OFF, False
        self._mode.value = OFF

    def prepare(self, ctx: PluginContext, rehearse: bool = False) -> None:
        """Get the part shown ready and ask Go; `rehearse`: its dry run (Mode Off only). Not for a part by Done."""
        button = "Rehearse" if rehearse else "Prepare"
        if why := self._cannot(ctx, "Prepare"):
            self._refuse(ctx, button, why)
            return
        if rehearse and self.mode != OFF:
            self._refuse(ctx, button, "Rehearse only with Mode Off")
            return
        part = self.part
        if self.is_done(ctx, part, self.index):
            if not self._again_press.press(ctx.now(), str(self.index)):
                if self._run(ctx).view.statuses.get(part.action) == MARKED:
                    why = (f"the run has {part.action} marked done: press {button} again within "
                           f"{SECOND_PRESS_SECONDS:g} s to do {part.label} again (the mark stays; {REDO} reopens the "
                           f"action)")
                else:
                    why = (f"{part.label} is done: press {button} again within {SECOND_PRESS_SECONDS:g} s to do it "
                           f"again (only this part), or Done goes on")
                self._refuse(ctx, button, why, ask=True)
                return
            self._again(ctx, self.index)
        self._start(ctx, part, rehearse)

    def _again(self, ctx: PluginContext, index: int) -> None:
        """Make one done part to do again, its later parts kept: the guide and the executor forget its result."""
        part = self.parts[index]
        self._done.discard(index)
        self._skipped.discard(index)
        self._reopened.discard(index)
        self._flagged.pop(index, None)
        if part.kind in EXECUTED and part.done_keys:
            self._executor(ctx).request_forget(part.action, set(part.done_keys))
        self._forced = part
        self._outcomes.pop(index, None)
        self._record(ctx, "again", part)
        ctx.log_warn(f"guide: {part.label} was done; doing it again (Prepare twice)")

    def _start(self, ctx: PluginContext, part: GuidePart, rehearse: bool = False) -> None:
        """Run the shown part's procedure, which then belongs to it: plan and ask Go, or record a part by Done.

        One waiting for Go, of this part or another, is replaced: its cancellation does not end Auto.
        """
        if self.procedure.waiting:
            self._replacing = self._owner
        self._outcomes.pop(self.index, None)
        self._auto_end = None
        self._used.touch()
        self._select(ctx, part)
        self._owner = part
        self.procedure.start(ctx, "rehearse" if rehearse else part.label,
                             self._run_part(ctx, part, self.index, rehearse), f"guide {part.label}")

    def press_go(self, ctx: PluginContext) -> None:
        """Answer the question of the part shown; out of order on a second press."""
        owner = self.owner
        if owner is not None and owner is not self.part:
            self._refuse(ctx, "Go", f"Go answers {owner.label}: Back or Next to it")
            return
        if not self.procedure.prompt:
            self._refuse(ctx, "Go", "nothing waits for Go: Prepare first")
            return
        if before := self.undone_before(ctx, self.index):
            labels = ", ".join(self.parts[each].label for each in before)
            if not self._order_press.press(ctx.now(), f"Go {self.index}"):
                # * Go's own question (other robots not measured) is asked now too: one second press answers both.
                asked = self.procedure.arm_confirm(ctx)
                self._refuse(ctx, "Go", f"out of order, not done yet: {labels}; " + (
                    asked or f"press Go again within {SECOND_PRESS_SECONDS:g} s to go anyway"), ask=True)
                return
            ctx.log_warn(f"guide: Go on {self.part.label} out of order (not done yet: {labels})")
            self._record(ctx, "out of order", self.part, before=[self.parts[each].label for each in before])
        self.procedure.press_go(ctx)

    def cancel(self, ctx: PluginContext) -> None:
        """Stop what the guide runs; Auto ends."""
        self._end_auto(ctx, "Cancel")
        self.procedure.cancel()

    def done(self, ctx: PluginContext) -> None:
        """Confirm a part by hand or Mark done; leave a finished part; any other part on a second press (Skip).

        A part by hand needs every box ticked: the executor then records it. Mark done marks the action in the run.
        Neither is ever skipped, and out of order both ask a second press. Leaving a part shows the next one not done.
        """
        if why := self._cannot(ctx, "Done"):
            self._refuse(ctx, "Done", why)
            return
        part = self.part
        # * A part Redo reopened at its end too: Done accepts where the robot stands.
        finished = self.is_done(ctx, part, self.index) or self.at_end(ctx, part)
        if not finished and part.kind in (HAND, MARK):
            doubts = self._doubts(ctx, part)
            if doubts and not self._done_press.press(ctx.now(), f"confirm {self.index}"):
                self._refuse(ctx, "Done", f"{'; '.join(doubts)}: press Done again within {SECOND_PRESS_SECONDS:g} s "
                                          f"to go anyway", ask=True)
                return
            if doubts:
                ctx.log_warn(f"guide: {part.label}: Done anyway ({'; '.join(doubts)})")
                self._record(ctx, "done anyway", part, doubts=doubts)
            if part.kind == HAND and not self._hand.ticked:
                self._hand.tick_all()
            self._by_done = part if part.kind == HAND else None
            self._assume = part if part.kind == MARK and not self._robot_live(ctx, part) else None
            self._mark_now = part if part.kind == MARK and self._unsettled is part else None
            self._start(ctx, part)
            return
        if not finished and part.kind != PARK and not self._done_press.press(ctx.now(), part.label):
            self._refuse(ctx, "Done", f"{part.label} has no result yet: press Skip within {SECOND_PRESS_SECONDS:g} s "
                                      f"to go on anyway", ask=True)
            return
        if self.owner is part and self.procedure.waiting:
            # * Done leaves the part: its plan waiting for Go is dropped, Auto goes on.
            self._replacing = part
            self.procedure.cancel()
            ctx.log_info(f"guide: {part.label}: Done drops its plan waiting for Go")
        self._reopened.discard(self.index)
        if not self.is_done(ctx, part, self.index):
            self._done.add(self.index)
            if not finished:
                self._skipped.add(self.index)
            self._record(ctx, "done" if finished else "skipped", part)
            if part.kind == PARK and not finished:
                ctx.log_warn(f"guide: {part.label} skipped: {self.serial_for(part)} is not parked and may stand in "
                             f"the way")
            else:
                ctx.log_info(f"guide: {part.label} {'done' if finished else 'skipped'}")
        if (following := self._following(ctx, self.index)) is not None:
            self._show(ctx, following)

    def _following(self, ctx: PluginContext, index: int) -> int | None:
        """The part going on from part `index` shows: the next one not done, past those done already; else the next.

        None after the last part. Back and Next still step one part at a time.
        """
        if index + 1 >= len(self.parts):
            return None
        target = self.next_to_do(ctx, index + 1)
        if target is None:
            return index + 1
        if target > index + 1:
            ctx.log_info(f"guide: on to {self.parts[target].label}; "
                         f"{', '.join(part.label for part in self.parts[index + 1:target])} done already")
        return target

    def _doubts(self, ctx: PluginContext, part: GuidePart) -> list[str]:
        """What speaks against confirming a part by hand or Mark done now; Done twice goes anyway."""
        doubts = []
        if before := self.undone_before(ctx, self.index):
            doubts.append(f"out of order, not done yet: {', '.join(self.parts[each].label for each in before)}")
        if part.kind == HAND and not self._hand.ticked:
            ticked, count = self._hand.count
            doubts.append(f"only {ticked} of {count} boxes ticked")
        if part.kind == MARK and not self._robot_live(ctx, part):
            doubts.append(f"{self._who(self.serial_for(part))} is not live: the run then assumes the export's end, not "
                          f"where it stands")
        if part.kind == MARK and self._unsettled is part and (moving := self._acting_moves(ctx, part)):
            doubts.append(f"{moving}; the run records where it stands now")
        return doubts

    def _acting_moves(self, ctx: PluginContext, part: GuidePart) -> str:
        """Why the part's robot counts as moving (`robot_moving`); "" if it stands still, or has none."""
        serial = self.serial_for(part)
        robot = ctx.world.robots.get(serial) if serial else None
        return robot_moving(serial, robot, ctx.now()) if robot is not None else ""

    def _robot_live(self, ctx: PluginContext, part: GuidePart) -> bool:
        """Whether the part's robot is in the snapshot and fresh; True by hand (no robot)."""
        serial = self.serial_for(part)
        robot = ctx.scene.snapshot.robots.get(robot_id(serial)) if serial else None
        return serial is None or robot is not None and is_live(robot, ctx.now())

    def redo(self, ctx: PluginContext) -> None:
        """Make the part shown, done, and every later part to do again, on a second press; nothing moves.

        The run reopens every action marked done from this part's on, and the executor forgets what it finished from
        this part on.
        """
        if why := self._cannot(ctx, REDO):
            self._refuse(ctx, REDO, why)
            return
        if not self._redo_press.press(ctx.now(), str(self.index)):
            return
        self._end_auto(ctx, REDO)
        run, executor, target = self._run(ctx), self._executor(ctx), self.part
        later = self.parts[self.index:]
        actions = list(dict.fromkeys(part.action for part in later))
        # * The latest first and the target's own action last: each reopen makes its action current in the run.
        reopened = [action for action in reversed(actions) if run.view.statuses.get(action) == MARKED]
        for action in reopened:
            run.request_reopen(action)
        still = [action for action in reopened if run.view.statuses.get(action) == MARKED]
        if still:
            self._refuse(ctx, REDO, f"the run did not reopen {', '.join(still)} (the log says why); nothing was "
                                    f"redone")
            return
        self._marks = dict(run.view.statuses)
        # * From the part on in its own action; every later action entirely.
        own = [part.done_keys for part in later if part.action == target.action]
        executor.request_forget(target.action, set().union(*own))
        for action in actions[1:]:
            executor.request_forget(action)
        self._done.difference_update(range(self.index, len(self.parts)))
        self._skipped.difference_update(range(self.index, len(self.parts)))
        self._flagged = {index: flag for index, flag in self._flagged.items() if index < self.index}
        self._outcomes = {index: outcome for index, outcome in self._outcomes.items() if index < self.index}
        # * Nothing of what ran before counts again: the last plan's goal and basis, the boxes ticked, the presses.
        self._goal = self._basis = self._planned = self._ran = self._auto_at = None
        self._unsettled = self._mark_now = self._assume = self._forced = self._by_done = None
        self.goal_text = ""
        self._reopened = {index for index in range(self.index, len(self.parts))
                          if self.parts[index].kind in (DRIVE, MOVE, PARK) and self.at_end(ctx, self.parts[index])}
        self._hand.forget()
        self._done_press.forget()
        self._order_press.forget()
        self._again_press.forget()
        self._record(ctx, "redo", target, reopened=reopened)
        ctx.log_info(f"guide: redo from {target.label}" + (f"; reopened {', '.join(reopened)}" if reopened else ""))
        self.procedure.outcome = None
        self._asked = None
        self._select(ctx, target)

    # --- --- --- --- --- PROCEDURES --- --- --- --- ---

    async def _run_part(self, ctx: PluginContext, part: GuidePart, index: int, rehearse: bool) -> None:
        """Run one part through its layer; a part by Done then shows the next one. Any failure ends Auto."""
        self._ran = None
        self._record(ctx, "rehearse" if rehearse else "start", part)
        try:
            forced, self._forced = self._forced is part, None
            if part.kind == DRIVE:
                await self._drive(ctx, part, again=forced or index in self._flagged)
                self._done.add(index)
                self._flagged.pop(index, None)
            elif part.kind == PARK:
                try:
                    await self._park(ctx, part, again=forced)
                except Exception as error:
                    # * Parks are optional: no path or not there warns and goes on, Auto too. A soft stop, a Go not
                    #   given or a motion not over in time stop it, as Cancel does.
                    if isinstance(error, WaitTimeout) or self._soft_stopped(ctx):
                        raise
                    self._not_parked(ctx, index, str(error))
                    return
            elif part.kind == MARK:
                await self._mark(ctx, part)
            elif part.kind == REFUSED:
                raise RuntimeError(f"{part.label}: {part.why}; do it by hand, then Done twice")
            elif part.kind == MOVE and not rehearse and not forced and self.at_end(ctx, part, NOTHING_TO_MOVE):
                # * Nothing to move: the arms stand at the design's goal (a monitor restarted after the move).
                ctx.log_info(f"guide: {part.label}: the arms stand at its goal already, nothing to move")
                self._done.add(index)
            else:
                await self._execute(ctx, part, rehearse)
        except BaseException as error:
            if isinstance(error, asyncio.CancelledError) and self._replacing is part:
                # * Replaced on purpose (Prepare or Done elsewhere): Auto goes on.
                self._replacing = None
                raise
            # * str() of a cancellation is empty: its type says it.
            self._end_auto(ctx, f"{part.label} failed: {str(error) or type(error).__name__}"
                           if not isinstance(error, asyncio.CancelledError) else f"{part.label} cancelled")
            raise
        finally:
            # * Only this part's: a procedure replaced ends after the next one started.
            if self._by_done is part:
                self._by_done = None
            if self._assume is part:
                self._assume = None
            if self._mark_now is part:
                self._mark_now = None
        if rehearse:
            self.procedure.outcome = Check("rehearsed", GOOD, f"{part.label}: every check passed, nothing sent")
            return
        self._ran = part
        self._reopened.discard(index)
        self._record(ctx, "done", part)
        ctx.log_info(f"guide: {part.label} done")
        after = None
        if part.kind == MARK or (self.mode == OFF and self.kinds[part.kind].manual):
            after = self._following(ctx, index) if self.mode == OFF else (index + 1 if index + 1 < len(self.parts)
                                                                          else None)
        following = self.parts[after] if after is not None else None
        if part.kind == MARK and following is not None:
            # ! The run moves its current step on a mark (to its first pending action): select the next part at once,
            #   or the guide takes that move for the operator's pick.
            self._select(ctx, following)
        if self.mode != OFF:
            return  # * Auto and God go on to the next part to do by themselves
        if self.kinds[part.kind].manual:
            # * Done was the operator's go-on already; the procedure still runs here, so select it ourselves.
            if self.index == index and following is not None:
                self._show(ctx, after, select=False)
                self._select(ctx, following)
        else:
            self.procedure.outcome = Check("done", GOOD, f"{part.label} is done: check the result chips, then Done "
                                                         f"goes on")

    async def _ask(self, ctx: PluginContext, path: TimedPath, prompt: str, go: str, acting: list[str],
                   waits: bool) -> frozenset[str]:
        """Preview `path` and wait for Go; Auto and God skip it unless `waits`.

        `waits`: a plan with a warning, or any move of a real robot (on real robots Auto only prepares).
        """
        self.procedure.auto = self.mode != OFF and not waits
        try:
            return await self.procedure.optional_go(ctx, path, prompt, go, acting)
        finally:
            self.procedure.auto = self.mode != OFF

    async def _drive(self, ctx: PluginContext, part: GuidePart, again: bool = False) -> None:
        """Plan the drive in the base planner, preview it, commit it on Go, and wait until it ends at its goal.

        `again`: drive even when the base stands within the arrival tolerance (a drive flagged not good).
        """
        serial = self._acting(part)
        if not again and self.at_end(ctx, part):
            # * Nothing to drive (a release starts where its jointing ended): planning from there would only refuse a
            #   start whose tools are still at the bar.
            ctx.log_info(f"guide: {part.label}: {self._who(serial)} stands at its goal already, nothing to drive")
            return
        planner: BasePlannerContract = ctx.require("base_planner")
        self.procedure.doing = f"waiting for the live scene to show {part.movement}"
        # * Go too waits inside: a plan made for a scene that changed before Go is refused, not driven.
        async with showing(ctx, self._process(ctx), part.action, part.movement):
            self.procedure.doing = "planning the drive"
            plan = await planner.request_plan(BasePlanRequest(serial, part.base))
            if plan.state != "ready":
                raise RuntimeError(plan.message if plan.message.startswith("no ") else f"no path: {plan.message}")
            self._previewing = ("base", serial)
            # * Auto waits at every drive of a real robot: a drive crosses the lab.
            real = bool(simulation_problem(ctx))
            unknown = await self._ask(ctx, plan.path, f"Drive {self._who(serial)}, {part.label}: {plan.message}. Check "
                                                      f"the path is clear of people and of what the scene does not "
                                                      f"know; e-stop in hand, then Go.", "drive", [robot_id(serial)],
                                      waits=real or plan.warning and self.mode != GOD)
        sent_here = await self._commit(ctx, planner, plan, unknown, serial, "the drive")
        if not sent_here and not self.at_end(ctx, part):
            # * The panel's drive of this plan ended elsewhere (stopped, or replanned there): plan again from here.
            ctx.log_info(f"guide: {part.label}: the drive sent from the panel ended away from its goal; planning again")
            await self._drive(ctx, part, again=again)
            return
        if not self.at_end(ctx, part):
            raise RuntimeError(f"{self._drive_failure(ctx, planner, part, serial)}: Prepare again")

    def _drive_failure(self, ctx: PluginContext, planner: BasePlannerContract, part: GuidePart, serial: str) -> str:
        """Why a drive did not end at its goal: how the run ended, else why the robot is not live, else how far off."""
        check = planner.run_check(ctx, serial)
        if check is not None and check.level >= WARN:
            return f"the drive {check.label}: {check.detail}" if check.detail else f"the drive {check.label}"
        if problem := self._not_live(ctx, serial):
            return f"the drive ended, but {self._who(serial)} is not live: {problem}"
        x, y, yaw = floor_pose(ctx.kinematics.base_pose(serial))
        turn = math.degrees((yaw - part.base[2] + math.pi) % (2 * math.pi) - math.pi)
        return (f"the drive ended {100 * math.hypot(x - part.base[0], y - part.base[1]):.0f} cm and {abs(turn):.1f}° "
                f"from its goal")

    def _not_live(self, ctx: PluginContext, serial: str) -> str:
        """Why `serial` is not live (`live_problem`); "" when it is."""
        robot = ctx.scene.snapshot.robots.get(robot_id(serial))
        return "it is not in the scene" if robot is None else live_problem(robot, ctx.now())

    def _not_parked(self, ctx: PluginContext, index: int, why: str) -> None:
        """Leave a park that did not get there, flagged "not parked", with a warning; the guide goes on."""
        part = self.parts[index]
        self._done.add(index)
        self._flagged[index] = Check("not parked", WARN, f"{why}: {self._who(self.serial_for(part))} may stand in "
                                                         f"the way")
        self._record(ctx, "not parked", part, why=why)
        ctx.log_warn(f"guide: {part.label}: not parked ({why}); going on")
        self.procedure.outcome = Check("not parked", WARN, f"{part.label}: {why}. Parks are optional: Done goes on")

    def _flag_drive_before(self, part: GuidePart, why: str) -> str:
        """Flag the drive before `part` in its action "not good", to do again; what to tell, "" without one."""
        index = next((each for each, other in enumerate(self.parts) if other is part), None)
        if index is None:
            return ""
        first, _count = self._action_span(part.action)
        drive = next((each for each in range(index - 1, first - 1, -1) if self.parts[each].kind == DRIVE), None)
        if drive is None:
            return ""
        self._done.discard(drive)
        self._skipped.discard(drive)
        self._flagged[drive] = Check("not good", BAD, f"{part.label} finds no goal from where this drive ended "
                                                      f"({why}): Prepare drives it again")
        return f"; {self.parts[drive].label} is flagged not good: Back to it and Prepare drives again"

    async def _park(self, ctx: PluginContext, part: GuidePart, again: bool = False) -> None:
        """Stow the robot's arms with the arm planner, then drive it to its parked pose with the base planner.

        Each is planned in the live scene, previewed and sent on Go, as a move and a drive are; a robot stowed or
        parked already skips that half.
        """
        serial = self._acting(part)
        if part.stow and not self.stowed(ctx, part):
            planner: ArmPlannerContract = ctx.require("arm_planner")
            self.procedure.doing = f"waiting for the live scene to show {part.movement}"
            # * Go too waits inside: a plan made for a scene that changed before Go is refused.
            async with showing(ctx, self._process(ctx), part.action, part.movement):
                self.procedure.doing = f"planning the stow of {self._who(serial)}"
                (arm, goal), *others = part.stow
                plan = await planner.request_plan(ArmPlanRequest(serial, arm, goal, also=tuple(others)))
                if plan.state != "ready":
                    raise RuntimeError(f"no stow path: {plan.message}")
                names = tuple(name for each, _goal in part.stow for name in arm_joint_names(each))
                self._previewing = ("joints", serial, names, ctx.kinematics.base_pose(serial))
                real = bool(simulation_problem(ctx))
                unknown = await self._ask(ctx, plan.path, f"Stow {self._who(serial)}'s arms, {part.label}: "
                                                          f"{plan.message}. Check the arms clear the bars and the "
                                                          f"other robots; e-stop in hand, then Go.", "stow",
                                          [robot_id(serial)], waits=real or plan.warning and self.mode != GOD)
            await self._commit(ctx, planner, plan, unknown, serial, "the stow")
            # * The snapshot shows the arms' last state from the next tick on.
            try:
                await ctx.wait_until(lambda: self.stowed(ctx, part), timeout_s=STOW_SETTLE,
                                     description=f"{serial}'s arms to show stowed")
            except WaitTimeout as error:
                problem = self._not_live(ctx, serial)
                raise RuntimeError(f"the stow ended, but {self._who(serial)} is not live: {problem}: Prepare again"
                                   if problem else f"the stow ended away from {self._who(serial)}'s stow joints: "
                                                   f"Prepare again") from error
        await self._drive(ctx, part, again=again)

    async def _commit(self, ctx: PluginContext, planner, plan, unknown: frozenset[str], serial: str,
                      what: str) -> bool:
        """Send a plan on its planner and wait until the robot is given back; stopped on any failure or Cancel.

        A plan sent already from its planner's panel is not sent again: the guide waits for that motion instead.

        Returns:
            bool: True if the guide sent the plan, False if it was sent already.
        """
        latest = planner.plan
        here = latest is None or latest.id != plan.id or latest.state != "sent"
        if here:
            try:
                sent, message = await planner.commit(plan.id, unknown, self.procedure.clearance.value)
            except BaseException:
                planner.cancel_commit(serial)
                raise
            if not sent:
                raise RuntimeError(f"not sent: {message}")
        else:
            ctx.log_info(f"guide: {what} of {self._who(serial)} was sent already from its planner's panel; waiting "
                         f"for it")
        self.procedure.doing = f"running {what}"
        slot = ctx.world.robots[serial].lease_slot
        try:
            await ctx.wait_until(lambda: slot.lease is None, timeout_s=2 * plan.path.duration + MOTION_MARGIN,
                                 description=f"{serial} to finish {what}")
        except BaseException:
            # ! Cancel or a motion not over in time: the robot stops before the guide says so.
            planner.cancel_commit(serial)
            raise
        return here

    async def _mark(self, ctx: PluginContext, part: GuidePart) -> None:
        """Mark the action done in the run once its robot stands still; not live, or never still, after Done twice."""
        assume = self._assume is part
        who = self._who(self.serial_for(part))
        if not assume and not self._robot_live(ctx, part):
            raise RuntimeError(f"{who} is not live: the run would only assume where it stands; Done twice marks it "
                               f"anyway")
        # * The run refuses a mark while the robot's arm still settles from the part before: it records where it stands.
        #   Only this robot: another one's stale wheel states (it counts as moving for good) hold nothing back here.
        if self._mark_now is not part and self._robot_live(ctx, part):
            self.procedure.doing = f"waiting for {who} to stand still"
            try:
                await ctx.wait_until(lambda: not self._acting_moves(ctx, part), timeout_s=STILL_WAIT,
                                     description=f"{who} to stand still")
            except WaitTimeout as late:
                self._unsettled = part
                raise RuntimeError(f"{self._acting_moves(ctx, part) or who + ' moved'} for {STILL_WAIT:g} s: Done "
                                   f"twice marks it anyway") from late
        run = self._run(ctx)
        run.request_mark_done(part.action)
        if assume and not self._robot_live(ctx, part):
            # * The run marks an action whose robot is not live on a second request: it assumes the export's end.
            run.request_mark_done(part.action)
        try:
            await ctx.wait_until(lambda: run.view.statuses.get(part.action) == MARKED, timeout_s=MARK_WAIT,
                                 description=f"{part.action} marked done")
        except Exception as error:
            raise RuntimeError(f"the run did not mark {part.action} done within {MARK_WAIT:g} s: the log says "
                               f"why") from error

    async def _execute(self, ctx: PluginContext, part: GuidePart, rehearse: bool) -> None:
        """Plan the part's motion if it has one, preview it, then let the executor run it, answering its questions."""
        serial = self._acting(part)
        route = basis = None
        unknown: frozenset[str] = frozenset()
        if any(plan.kind in MOTION_KINDS for plan in part.plans):
            route, basis, path, warned = await self._plan(ctx, part, serial)
            self._basis = (part, basis)
            self._previewing = ("joints", serial, tuple(route.result.trajectory.joint_names), basis.base)
            unknown = await self._ask(ctx, path, f"{'Rehearse' if rehearse else 'Run'} {part.label}: "
                                                 f"{path.duration:.1f} s. Play the preview, check the arms clear "
                                                 f"everything; e-stop in hand, then Go.",
                                      "rehearse" if rehearse else "run", [robot_id(serial)],
                                      waits=rehearse or bool(simulation_problem(ctx)) or warned and self.mode != GOD)
        executor, process = self._executor(ctx), self._process(ctx)
        loaded = self._run(ctx).view.process
        request = ExecRequest(loaded, part.action, part.step, part.plans, serial,
                              loaded.process.robots[loaded.process.actions[part.action].robot].name, route,
                              part.step.primary if route is not None else None, basis, self.traj_time, part.label,
                              ignore_built=lambda: self.ignore_built,
                              obstacles_collide=lambda: process.obstacles_collide,
                              transfer_as_free=lambda: process.transfer_as_free, unknown_robots=unknown,
                              clearance=self.procedure.clearance.value, dry_run=rehearse)
        # * The executor refuses while anything moves: an arm still settling from the part before, say.
        self.procedure.doing = "waiting for every robot to stand still"
        await ctx.wait_until(lambda: not moving_reason(ctx.world, ctx.now(), unknown={
            serial for serial in ctx.world.robots if robot_id(serial) in unknown}),
                             timeout_s=STILL_WAIT, description="every robot to stand still")
        self.procedure.doing = f"running {part.label}"
        future = executor.request_execute(request)
        try:
            await self._answer(ctx, part, executor, future)
        except BaseException:
            # ! Cancel, a Go not given in time, or any failure here: the executor stops, every robot held and given
            #   back, before the guide says so.
            if not future.done():
                executor.request_cancel()
                await ctx.wait_until(future.done, timeout_s=MOTION_MARGIN, description="the executor to stop")
            raise
        result = future.result()
        if result.status != (DRY_RUN if rehearse else DONE):
            raise RuntimeError(f"{result.status}: {result.reason}")

    async def _answer(self, ctx: PluginContext, part: GuidePart, executor: MovementExecContract,
                      future: asyncio.Future) -> None:
        """Pass each of the executor's questions on as the guide's Go until it ends.

        A part by hand was confirmed already (its boxes and Done): the guide answers its question itself, as God
        answers every question.
        """
        answered = None
        while not future.done():
            def asks() -> bool:
                return future.done() or (executor.prompt is not None and executor.prompt.id != answered)

            await ctx.wait_until(asks, timeout_s=None, description="the executor")
            prompt = executor.prompt
            if future.done() or prompt is None:
                continue
            self.procedure.doing = f"running {part.label}"
            by_done = self._by_done is part
            if by_done or self.mode == GOD and not simulation_problem(ctx):
                await ctx.sleep(max(0.0, prompt.opened_at + PROMPT_ARM_S + GOD_PAUSE - ctx.now()))
                # ! Again after the pause: God may have ended, or a part stopped being simulated, meanwhile.
                if not by_done and (self.mode != GOD or simulation_problem(ctx)):
                    continue
                if executor.prompt is not None and executor.prompt.id == prompt.id:
                    ctx.log_info(f"guide{'' if by_done else ' (God)'}: confirmed {prompt.line}")
                    executor.request_confirm()
                answered = prompt.id
                continue
            if self.mode == GOD:
                self._end_auto(ctx, f"not a full simulation any more: {simulation_problem(ctx)}")
            if part.kind == HAND:
                raise RuntimeError("a part by hand is confirmed by its boxes and Done")
            pressed = await self.procedure.ask_go_while(
                ctx, prompt.line, "confirm", lambda: not future.done() and executor.prompt is not None
                and executor.prompt.id == prompt.id)
            if pressed:
                executor.request_confirm()
            answered = prompt.id

    async def _plan(self, ctx: PluginContext, part: GuidePart, serial: str):
        """Solve the goal at the live base where the move has one, then plan the part's motion from the live robot.

        All of it in the live scene once it shows the part's movement (`showing`), and refused if it changed meanwhile.

        Returns:
            tuple: The route, its basis, the timed path, and whether the plan has warnings.
        """
        self.procedure.doing = f"waiting for the live scene to show {part.movement}"
        async with showing(ctx, self._process(ctx), part.action, part.movement):
            return await self._plan_shown(ctx, part, serial)

    async def _plan_shown(self, ctx: PluginContext, part: GuidePart, serial: str):
        """`_plan` once the live scene shows the part's movement."""
        run, process = self._run(ctx), self._process(ctx)
        loaded = run.view.process
        planner: MotionPlannerContract = ctx.require("motion_planner")
        step = loaded.steps[loaded.index_of(part.action, part.step.primary)]
        snapshot = ctx.scene.snapshot
        live = snapshot.robots.get(robot_id(serial))
        if live is None or not is_live(live, ctx.now()):
            raise RuntimeError(f"{serial} is not live: plans start from the live robot only")
        found = await self._plan_on_arm_planner(ctx, part, serial, step)
        if found is not None:
            self._planned = (part, ARM_PLANNER_NAME)
            return found
        goal_joints = goal_base = None
        if part.routine in IK_GOALS:
            self.procedure.doing = "solving the goal at the live base"
            check = await planner.request_ik(MotionPlanRequest(
                loaded, step, snapshot, ctx.config.robots, transfer_as_free=process.transfer_as_free,
                ignore_built=self.ignore_built, obstacles_collide=process.obstacles_collide,
                label=f"check goal {step.label}"))
            self.goal_text = goal_line(check)
            run.request_record("check_goal", action=part.action, movement=step.movement.id, data=check.as_record())
            if not check.ok:
                raise RuntimeError(f"no goal at the live base: {check.reason}"
                                   f"{self._flag_drive_before(part, check.reason)}")
            goal_joints, goal_base = dict(check.goal_joints), check.base
        self.procedure.doing = f"planning {part.label}"
        snapshot = ctx.scene.snapshot
        live = snapshot.robots[robot_id(serial)]
        carried = self._executor(ctx).carried if part.kind != MOVE else None
        goals = live_goal_poses(loaded, part.action, step, part.routine, live,
                                transfer_as_free=process.transfer_as_free, carried=carried,
                                retreat_live_axis=run.view.settings.retreat_live_axis, log_info=ctx.log_info,
                                log_warn=ctx.log_warn)
        request = MotionPlanRequest(loaded, step, snapshot, ctx.config.robots,
                                    transfer_as_free=process.transfer_as_free, ignore_built=self.ignore_built,
                                    obstacles_collide=process.obstacles_collide, goal_joints=goal_joints,
                                    goal_poses=goals, label=f"plan {step.label}")
        route = await planner.request_plan(request)
        tries = 1
        # * A free search is random: Auto and God plan again before they give up, as the operator would.
        while not route.ok and self.mode != OFF and part.routine in FREE_SEARCHES and tries < AUTO_PLAN_TRIES:
            tries += 1
            ctx.log_info(f"guide: no plan for {part.label} ({route.result.reason}); planning again, try {tries} of "
                         f"{AUTO_PLAN_TRIES}")
            self.procedure.doing = f"planning {part.label} again ({tries}/{AUTO_PLAN_TRIES})"
            route = await planner.request_plan(request)
        if not route.ok:
            raise RuntimeError(f"{route.result.status}: {route.result.reason}")
        timed = timed_route(route, self.traj_time)
        if isinstance(timed, str):
            raise RuntimeError(timed)
        path, _minimum = timed
        trajectory = route.result.trajectory
        names = tuple(trajectory.joint_names)
        basis = basis_of(snapshot, loaded, step, serial, names, dict(zip(names, trajectory.positions[0])),
                         process.transfer_as_free, run.view.statuses, self.ignore_built, ctx.now(), goal_base,
                         obstacles_collide=process.obstacles_collide)
        self._goal = (part, dict(zip(names, trajectory.positions[-1])), ctx.kinematics.base_pose(serial))
        self._planned = (part, MOTION_PLANNER_NAME)
        for line in route.warnings:
            ctx.log_warn(f"plan {step.movement.id}: {line}")
        run.request_record("plan", action=part.action, movement=step.movement.id, data={
            "planner": route.planner, "seconds": route.seconds, "waypoints": route.waypoints,
            "goal_reused": goal_joints is not None, "warnings": list(route.warnings), "by": self.name})
        return route, basis, path, bool(route.warnings)

    def _on_arm_planner(self, part: GuidePart) -> bool:
        """Whether the arm planner tries the part first: a free move, the box on, Ignore built bars off."""
        return self.arm_planner_moves and not self.ignore_built and part.routine in FREE_MOVES

    async def _plan_on_arm_planner(self, ctx: PluginContext, part: GuidePart, serial: str, step):
        """Plan a free move with the arm planner, one arm or both in one search, made a route the executor runs.

        In the live scene, which shows the part's movement (`_plan` waits for it): the process plugin puts that
        movement's bars and allowed contacts there. None leaves it to the motion planner: not a free move (a held
        transfer, a line), Ignore built bars (the live scene keeps them), the box off, or no target or joint goal in
        the process. Auto and God search a path again before they give up, as the motion planner's.

        Returns:
            tuple | None: The route, its basis, the timed path and whether the plan warns; or None.

        Raises:
            RuntimeError: If it finds no goal or no path: the motion planner is not tried (it would fail too, slowly).
        """
        if not self._on_arm_planner(part):
            return None
        run, process = self._run(ctx), self._process(ctx)
        loaded = run.view.process
        action = loaded.process.actions[part.action]
        try:
            spec = classify(loaded.process, action, action.movements[part.movement_index], process.transfer_as_free)
        except UnknownMovement:
            return None
        if not spec.arms:
            return None
        arms = [arm.arm for arm in spec.arms]
        names = tuple(name for arm in spec.arms for name in arm.joints)
        planner: ArmPlannerContract = ctx.require("arm_planner")
        authored = spec.goal_joints or {}
        reference = {name: authored[name] for name in names if name in authored}
        goal_base = None
        if part.routine in IK_GOALS:
            if any(arm.target is None for arm in spec.arms):
                return None
            self.procedure.doing = "solving the goal at the live base (arm planner)"
            solved, line = await planner.solve_goals(serial, {arm.arm: arm.target for arm in spec.arms},
                                                     reference or None)
            self.goal_text = f"{line} (arm planner)"
            if solved is None:
                raise RuntimeError(f"no goal at the live base (arm planner): {line}"
                                   f"{self._flag_drive_before(part, line)}")
            goal = [solved[name] for name in names]
            if part.routine is not Routine.SUPPORT_APPROACH:
                # * The process's joints only pick the branch: each joint turns by whole turns to the value nearest
                #   the arms now, as the motion planner's goals (`ik_goal`); the authored turn may cross the tool.
                #   A shoulder lift never turns through pointing down (`turn_limits`).
                robot = ctx.scene.snapshot.robots[robot_id(serial)]
                goal = unwrap_to_nearest(goal, [robot.joints[name] for name in names],
                                         *turn_limits(robot.model.urdf, names)).tolist()
            goal_base = ctx.kinematics.base_pose(serial)
        elif len(reference) == len(names):
            goal = [reference[name] for name in names]
        else:
            return None
        self.procedure.doing = f"planning {part.label} (arm planner)"
        snapshot = ctx.scene.snapshot
        goals = [tuple(float(value) for value in goal[6 * index:6 * index + 6]) for index in range(len(arms))]
        request = ArmPlanRequest(serial, arms[0], goals[0], also=tuple(zip(arms[1:], goals[1:])))
        plan = await planner.request_plan(request)
        tries = 1
        # * A free search is random: Auto and God plan again before they give up, as the operator would.
        while plan.state != "ready" and self.mode != OFF and tries < AUTO_PLAN_TRIES:
            tries += 1
            ctx.log_info(f"guide: no plan for {part.label} ({plan.message}); planning again, try {tries} of "
                         f"{AUTO_PLAN_TRIES}")
            self.procedure.doing = f"planning {part.label} again ({tries}/{AUTO_PLAN_TRIES}, arm planner)"
            plan = await planner.request_plan(request)
        if plan.state != "ready":
            raise RuntimeError(f"no path (arm planner): {plan.message}")
        # * The path's shape, not its 0.2 s samples: timed again, those would stop the arm at every sample on a curve.
        rows = plan.path.waypoints
        route = arm_path_route(action.robot, part.routine, names, rows, ARM_PLANNER, goal_joints=dict(zip(names, goal)),
                               warnings=(plan.message,) if plan.warning else ())
        timed = timed_route(route, self.traj_time)
        if isinstance(timed, str):
            raise RuntimeError(timed)
        path, _minimum = timed
        basis = basis_of(snapshot, loaded, step, serial, names, dict(zip(names, rows[0])), process.transfer_as_free,
                         run.view.statuses, self.ignore_built, ctx.now(), goal_base,
                         obstacles_collide=process.obstacles_collide)
        self._goal = (part, dict(zip(names, rows[-1])), ctx.kinematics.base_pose(serial))
        if plan.warning:
            ctx.log_warn(f"plan {part.movement} (arm planner): {plan.message}")
        run.request_record("plan", action=part.action, movement=part.movement, data={
            "planner": ARM_PLANNER, "message": plan.message, "waypoints": len(rows),
            "goal_reused": False, "warnings": list(route.warnings), "by": self.name})
        return route, basis, path, plan.warning

    def _acting(self, part: GuidePart) -> str:
        """The part's robot; refused without one."""
        serial = self.serial_for(part)
        if serial is None:
            raise RuntimeError(f"{part.action}'s robot is not configured")
        return serial

    # --- --- --- --- --- TICK --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Follow the selection; in Auto and God, work on the next part to do: show it and start it."""
        self.follow(ctx)
        if self.part is None:
            return
        self._forget_redone(ctx)
        self._keep_outcome()
        # * A part redone counts its end again once its robot left it: getting there afterwards is a new result.
        self._reopened = {index for index in self._reopened
                          if not self._robot_live(ctx, self.parts[index]) or self.at_end(ctx, self.parts[index])}
        if self.mode == GOD and (problem := simulation_problem(ctx)):
            self._end_auto(ctx, f"not a full simulation any more: {problem}")
        if self.mode == OFF or self.procedure.running:
            return
        target = self.next_to_do(ctx, self._auto_from)
        if target is None:
            self._end_auto(ctx, ALL_DONE)
            return
        part = self.parts[target]
        serial = self.serial_for(part)
        robot = ctx.world.robots.get(serial) if serial else None
        if stopped := self._soft_stopped(ctx):
            # * Any robot: a soft stop says something came too close, whichever robot the next part moves.
            self._end_auto(ctx, f"{self._who(stopped)} is soft stopped: Health › Resume, then choose Auto again")
            return
        if robot is not None and robot.lease_slot.lease is not None:
            self._end_auto(ctx, f"{robot.lease_slot.lease.holder} holds {self._who(serial)}: finish or Cancel it "
                                f"there, then choose Auto again")
            return
        if part is self._ran:
            self._end_auto(ctx, f"{part.label} ended, but shows no result")
            return
        if part is not self._auto_at:
            # * Once per part: the operator may look elsewhere while Auto waits there.
            self._auto_at = part
            self._show(ctx, target)
        if self.index != target:
            return
        if part.kind == HAND:
            # * Auto waits for the operator's boxes and Done; God ticks them.
            if self.mode == GOD:
                self._hand.show(ctx, part)  # * its own boxes first: the folder still holds the last part's
                self._hand.tick_all()
                self._by_done = part
                self._start(ctx, part)
        elif part.kind == REFUSED:
            self._end_auto(ctx, f"{part.label}: the executor cannot run it; do it by hand, then Done twice")
        elif part.kind == MARK:
            # * Auto never marks past a doubt (parts before not done, the robot not live): the operator decides.
            if doubts := self._doubts(ctx, part):
                self._end_auto(ctx, f"{part.label}: {'; '.join(doubts)}; Done twice marks it anyway")
            else:
                self._start(ctx, part)
        else:
            self.prepare(ctx)

    def _scrub(self, value: float) -> None:
        """The operator moved Preview."""
        self.procedure.preview = float(value)
        self._used.touch()

    def _toggle_play(self) -> None:
        """Play the preview, again from its start once at its end, or pause it."""
        path = self.procedure.preview_path
        if path is None:
            return
        if not self._playing and self.procedure.preview >= path.duration:
            self.procedure.preview = 0.0
        self._playing = not self._playing
        self._used.touch()

    # --- --- --- --- --- DRAWING --- --- --- --- ---

    def draw(self, ctx: PluginContext) -> None:
        """The title and the parts, the buttons as what they do now, the shown kind's folder, the notes, the ghost."""
        now = ctx.now()
        procedure, part = self.procedure, self.part
        self._advance_preview(now)
        procedure.confirm.draw(now)
        if part is None:
            self._title.content = section("assembly guide", SECTION_CTRL, "no process: Load one in the Assembly tab")
            set_changed(self._header, content=step_header("", "", ""))
            set_changed(self._overview, content=chip_line(""))
            self._chips.content = chip_row("needs", "") + chip_row("live", "") + chip_row("result", "")
            self._guide_line.content = chip_row("guide", "")
        else:
            first, count = self._action_span(part.action)
            serial = self.serial_for(part)
            self._title.content = section("assembly guide", SECTION_CTRL)
            set_changed(self._header, content=step_header(
                f"{part.action} · part {self.index - first + 1} of {count}", part.label,
                self.kinds[part.kind].goal_of(part),
                (part.kind, *([short_serial(serial)] if serial else []))))
            set_changed(self._overview, content=self._overview_html(ctx))
            self._chips.content = (chip_row("needs", "".join(map(check_chip, self._needs(ctx, self.index))))
                                   + chip_row("live", "".join(map(check_chip, self._live(ctx, part))))
                                   + chip_row("result", "".join(map(check_chip, self._results(ctx, self.index)))))
            self._guide_line.content = chip_row("guide", self._guide_chips(ctx))
        self._draw_buttons(ctx, now)
        kind = part.kind if part is not None else None
        if kind != self._folder_shown:
            # * Only on a change: every assignment is a message to each browser.
            for name, folder in self._folders.items():
                folder.visible = name == kind
            self._folder_shown = kind
        if part is not None:
            self.kinds[part.kind].show(ctx, part)
            set_changed(self._description, content=description(self.kinds[part.kind], part))
        self._notes.content = self._notes_html(ctx, now)
        self._list.content = self._list_html(ctx)
        self._draw_ghost(ctx)

    def _guide_chips(self, ctx: PluginContext) -> str:
        """The guide's own line: a press that did nothing, the mode, what runs and where, the last outcome."""
        procedure, owner = self.procedure, self.owner
        refusal = self.refusal(ctx.now())
        # * A press that did nothing first, so it is never scrolled out of the line.
        html = check_chip(refusal) if refusal is not None else ""
        start = self._name(self.parts[min(self._auto_from, len(self.parts) - 1)]) if self.parts else ""
        html += {OFF: "", AUTO: chip("AUTO", BUSY, f"From {start} on: prepares each part; in simulation sends it, on "
                                                   f"real robots waits for your Go at every drive and move; waits at "
                                                   f"each question, part by hand and warning"),
                 GOD: chip("GOD", FAIL, f"Simulation only, from {start} on: every part is prepared, sent and "
                                        f"confirmed")}[self.mode]
        if self.mode == OFF and self._auto_end is not None:
            html += check_chip(self._auto_end)
        waits = self._auto_waits()
        if owner is not None and owner is not self.part:
            # * What runs belongs to another part: name it, as the calibration guide does.
            what = "waits for Go" if procedure.prompt else (procedure.doing or "runs")
            html += chip(escape(f"{self._name(owner)} {what}"), BUSY,
                         escape(f"{owner.label}: Back or Next to it for Go and Preview; Cancel stops it from any part"))
        elif procedure.prompt:
            html += chip("question: see below", CURRENT, escape(procedure.prompt))
        elif procedure.doing:
            html += chip(escape(procedure.doing), BUSY, "Cancel stops it")
        elif waits:
            html += chip(escape(waits[0]), CURRENT, escape(waits[1]))
        if (outcome := self._outcome_of(self.index)) is not None:
            html += check_chip(outcome)
        return html

    def _auto_waits(self) -> tuple[str, str] | None:
        """What Auto waits for while nothing runs (chip text, hover); None when it waits for nothing of the operator."""
        target = self._auto_at
        if self.mode == OFF or self.procedure.running or target is None or self._index_of(target) is None:
            return None
        if target is not self.part:
            return (f"auto waits at {self._name(target)}", f"{target.label}: Back or Next to it, and Auto goes on")
        if target.kind == HAND and self.mode == AUTO:
            return "auto waits: tick the boxes, then Done", f"{target.label} is done by hand"
        return None

    def _needs(self, ctx: PluginContext, index: int) -> list[Check]:
        """What a part needs before it can act: its robot live, not stopped, not held; the parts before it done."""
        checks, part = [], self.parts[index]
        serial = self.serial_for(part)
        if serial is not None:
            robot, live = ctx.world.robots.get(serial), ctx.scene.snapshot.robots.get(robot_id(serial))
            who = self._who(serial)
            checks.append(Check("live", GOOD, f"{who}'s state is fresh") if live is not None
                          and is_live(live, ctx.now()) else Check("live", BAD, f"{who}'s state is not fresh"))
            if robot is not None and robot.soft_stop_latch.stopped:
                checks.append(Check("soft stopped", BAD, "Health › Resume before anything moves"))
            lease = robot.lease_slot.lease if robot is not None else None
            if lease is not None and not self.procedure.running:
                checks.append(Check("held", WARN, f"{lease.holder} holds {who}: finish or Cancel it there; Auto ends "
                                                  f"while it holds"))
        first, _count = self._action_span(part.action)
        before = self.undone_before(ctx, index)
        if index > first:
            labels = ", ".join(self.parts[each].label for each in before)
            checks.append(Check("earlier parts", GOOD, "Every part before this one is done", "done") if not before
                          else Check("earlier parts", WARN, f"Not done yet: {labels}", f"{len(before)} to do"))
        if parks := self.undone_before(ctx, index, parks=True):
            checks.append(Check("not parked", WARN, "May stand in the way (parks are optional): " +
                                ", ".join(self.parts[each].label for each in parks), str(len(parks))))
        if part.kind not in (DRIVE, PARK, MARK, REFUSED) and (active := self._executor(ctx).active):
            checks.append(Check("executor", WARN, f"The executor runs {active}"))
        if stale := self._stale(ctx, part):
            checks.append(Check("plan stale", BAD, f"{stale}: Go would be refused; Prepare again plans it anew"))
        planner = self._planner_check(part)
        return checks + ([planner] if planner is not None else [])

    def _planner_check(self, part: GuidePart) -> Check | None:
        """Which planner plans the part: the one that did once planned, else the one tried first; None if none."""
        if part.kind == DRIVE:
            return Check("base planner", INFO, "The base planner plans the drive; its plan shows in Plan › Base")
        if part.kind == SERVO:
            return Check("motion planner", INFO, "The executor plans each servo pass on the motion planner")
        if not any(plan.kind in MOTION_KINDS for plan in part.plans):
            return None
        first = ARM_PLANNER_NAME if self._on_arm_planner(part) else MOTION_PLANNER_NAME
        used = self._planned[1] if self._planned is not None and self._planned[0] is part else first
        if used != first:
            return Check(used, WARN, "The arm planner cannot plan this move (no target or joint goal in the "
                                     "process), so the motion planner planned it")
        if used == ARM_PLANNER_NAME:
            return Check(used, INFO, "The arm planner plans this free move (Settings › Free moves on the arm planner); "
                                     "its plan shows in Plan › Arm; no path fails the part")
        return Check(used, INFO, "The motion planner plans it (a held transfer, a line, or the arm planner turned "
                                 "off); Plan › Motion shows its state")

    def _live(self, ctx: PluginContext, part: GuidePart) -> list[Check]:
        """How the part stands now: the robots still or moving, and the kind's own (distance, boxes, parts)."""
        moving = moving_reason(ctx.world, ctx.now())
        checks = [Check("moving", PENDING, moving) if moving else Check("still", GOOD, "Every robot stands still")]
        return checks + self.kinds[part.kind].live(ctx, part)

    def _results(self, ctx: PluginContext, index: int) -> list[Check]:
        """The part's result: done, skipped, the executor's record, the run's mark, at its end."""
        part = self.parts[index]
        if index in self._flagged:
            return [self._flagged[index]]
        if index in self._skipped:
            return [Check("skipped", WARN, "Left without a result (Skip)")]
        marked = self._run(ctx).view.statuses.get(part.action) == MARKED
        if part.kind == MARK:
            return [Check("marked", GOOD, "The run has the action done") if marked else
                    Check("not marked", PENDING, "Done marks it")]
        if marked:
            return [Check("action marked", GOOD, f"The run has the action done: {REDO} redoes it")]
        checks = []
        if part.kind in EXECUTED and part.done_keys:
            loaded = self._run(ctx).view.process
            finished = part.done_keys <= self._executor(ctx).finished(loaded, part.action)
            checks.append(Check("executor finished", GOOD, "The executor ran it to its end") if finished else
                          Check("not run", PENDING, "The executor has not finished it in this run"))
        if part.kind in (DRIVE, MOVE, PARK):
            if self.at_end(ctx, part):
                checks.append(Check("at its end", PENDING, f"The robot stands where it ends, measured live, but "
                                    f"{REDO} made it to do again: Prepare runs it again")
                              if index in self._reopened else
                              Check("at its end", GOOD, "The robot stands where it ends, measured live"))
            elif index in self._done:
                checks.append(Check("done", GOOD, "Done by the guide"))
            elif part.kind == DRIVE and self._later_ran(ctx, index):
                checks.append(Check("driven", GOOD, "Later parts of its action ran in this run"))
            elif part.kind in (DRIVE, PARK):
                checks.append(Check("not there", PENDING, "The robot is away from its parked pose and stow"
                                    if part.kind == PARK else "The base is away from the target"))
        return checks

    def _overview_html(self, ctx: PluginContext) -> str:
        """The shown action's parts as numbered chips on one line (`state_chip`).

        The fill is the result, the shown one ringed, the earlier ones it needs without a result outlined, ▶ on the one
        whose procedure runs.
        """
        first, count = self._action_span(self.part.action)
        needed = set(self.undone_before(ctx, self.index))
        chips = [state_chip(self._number(index), self._state(ctx, index), False, index == self.index, index in needed)
                 for index in range(first, first + count)]
        return chip_line("".join(chips))

    def _draw_buttons(self, ctx: PluginContext, now: float) -> None:
        """Back, Next, Redo, Prepare, Go, Done, Cancel, Play as what they do now.

        Prepare, Done and Redo say why when they cannot act (`_cannot`), so they are grey only without a process, and
        Prepare on a part Done confirms; Go is grey while nothing of the part shown waits for it.
        """
        procedure, part, owner = self.procedure, self.part, self.owner
        set_changed(self._back, disabled=not self.parts or self.index == 0)
        set_changed(self._next, disabled=not self.parts or self.index >= len(self.parts) - 1)
        redoing = self._redo_press.waiting(now) == str(self.index)
        set_changed(self._redo, disabled=part is None, label=again_label(REDO) if redoing else REDO,
                    color=rgb(BUSY if redoing else ACTION))
        finished = part is not None and self.is_done(ctx, part, self.index)
        manual = part is not None and self.kinds[part.kind].manual
        preparing = self._again_press.waiting(now) == str(self.index)
        set_changed(self._prepare, disabled=part is None or manual,
                    label=again_label("Prepare") if preparing else
                    PREPARE_LABELS.get(part.kind, "Prepare") if part is not None else "Prepare",
                    color=rgb(BUSY if preparing else TEAL))
        # * Auto's second press on real robots: "Auto?" while it waits, as every second press.
        asking = self._auto_press.waiting(now) == "auto"
        self._mode.set_options((OFF, AUTO, GOD), (OFF, again_label(AUTO), GOD) if asking else None)
        self._mode.value = self.mode
        asks = bool(procedure.prompt) and owner is part and not manual
        going = asks and self._order_press.waiting(now) == f"Go {self.index}"
        set_changed(self._go, disabled=not asks, label=again_label("Go") if going else
                    f"Go: {procedure.go_label}" if asks else "Go", color=rgb(BUSY if going else OK))
        skipping = part is not None and self._done_press.waiting(now) == part.label
        again = part is not None and self._done_press.waiting(now) == f"confirm {self.index}"
        marks = part is not None and part.kind == MARK
        label = (again_label("Skip") if skipping else again_label("Done") if again else
                 "Done: next part" if finished else "Done: mark it" if marks else "Done")
        set_changed(self._done_button, disabled=part is None, label=label,
                    color=rgb(BUSY if skipping or again else ACTION))
        set_changed(self._cancel, disabled=not procedure.running)
        path = procedure.preview_path
        set_changed(self._play, disabled=path is None, label="Pause" if self._playing else "Play")
        if path is not self._preview_path:
            self._preview_path, self._playing = path, False
            self._preview.max = max(path.duration, SLIDER_STEP) if path is not None else 1.0
            self._preview.value = procedure.preview
        elif self._playing:
            self._preview.value = round(procedure.preview, 3)
        set_changed(self._preview, disabled=path is None)

    @property
    def _hand(self) -> HandKind:
        """The by-hand kind, with its boxes."""
        return self.kinds[HAND]

    def _action_span(self, action: str) -> tuple[int, int]:
        """The index of an action's first part, and how many parts it has."""
        indices = [index for index, part in enumerate(self.parts) if part.action == action]
        return indices[0], len(indices)

    def _advance_preview(self, now: float) -> None:
        """Move the preview on while Play plays."""
        path = self.procedure.preview_path
        if path is None:
            self._playing = False
        elif self._playing and self._played_at is not None:
            self.procedure.preview = min(self.procedure.preview + now - self._played_at, path.duration)
            self._playing = self.procedure.preview < path.duration
        self._played_at = now

    def _state(self, ctx: PluginContext, index: int) -> Check:
        """A part's result as a state: done, skipped, failed (its procedure's outcome) or to do."""
        part = self.parts[index]
        if (flag := self._flagged.get(index)) is not None:
            return Check(flag.label, flag.level, f"{part.label}: {flag.detail}")
        if index in self._skipped:
            return Check("skipped", WARN, f"{part.label}: left without a result (Skip)")
        if self.is_done(ctx, part, index):
            return Check("done", GOOD, part.label)
        outcome = self._outcome_of(index)
        if outcome is not None and outcome.level == BAD:
            return Check("failed", BAD, f"{part.label}: {outcome.detail}")
        return Check("to do", PENDING, part.label)

    def _number(self, index: int) -> str:
        """A part's number in its action; ▶ in front while its procedure runs."""
        first, _count = self._action_span(self.parts[index].action)
        return f"{'▶' if self.parts[index] is self.owner else ''}{index - first + 1}"

    def _name(self, part: GuidePart) -> str:
        """A part named by its action and number, e.g. "B20_J part 2"; its label if no longer among the parts."""
        index = self._index_of(part)
        if index is None:
            return part.label
        first, _count = self._action_span(part.action)
        return f"{part.action} part {index - first + 1}"

    def _index_of(self, part: GuidePart) -> int | None:
        """A part's index, by identity; None once the parts were built again."""
        return next((index for index, each in enumerate(self.parts) if each is part), None)

    def _who(self, serial: str | None) -> str:
        """A robot as the operator reads it: "cindy (0806)", its serial without a name in the process."""
        if serial is None:
            return "no robot"
        name = self._names.get(serial)
        return f"{name} ({short_serial(serial)})" if name else short_serial(serial)

    def _soft_stopped(self, ctx: PluginContext) -> str:
        """The serial of a soft stopped robot; "" while none is."""
        return next((serial for serial, robot in ctx.world.robots.items() if robot.soft_stop_latch.stopped), "")

    def _keep_outcome(self) -> None:
        """Keep how the last procedure ended for its part when it said something (`_outcomes`); not while it runs."""
        outcome, owner = self.procedure.outcome, self._owner
        if outcome is None or owner is None or self.procedure.running or outcome.level == GOOD:
            return
        if (index := self._index_of(owner)) is not None:
            self._outcomes[index] = outcome

    def _outcome_of(self, index: int) -> Check | None:
        """How part `index`'s last procedure ended: the procedure's own while the part owns it, else the one kept."""
        if self._owner is self.parts[index] and self.procedure.outcome is not None:
            return self.procedure.outcome
        return self._outcomes.get(index)

    def _stale(self, ctx: PluginContext, part: GuidePart) -> str:
        """Why `part`'s plan waiting for Go no longer fits, as the executor would say; "" if it does, or none waits."""
        if self._basis is None or self._basis[0] is not part or self.owner is not part or not self.procedure.waiting:
            return ""
        run, process = self._run(ctx), self._process(ctx)
        return stale_reason(self._basis[1], ctx.scene.snapshot, loaded=run.view.process, current=run.view.current,
                            transfer_as_free=process.transfer_as_free, ignore_built=self.ignore_built,
                            statuses=run.view.statuses, obstacles_collide=process.obstacles_collide)

    def _out_of_reach(self, ctx: PluginContext) -> frozenset[str]:
        """Other robots not measured that the arms Go moves cannot reach: Go needs no second press for them.

        None for a drive: a base may go anywhere.
        """
        owner = self.owner
        serial = self.serial_for(owner) if owner is not None else None
        if serial is None or serial not in ctx.world.robots or self._previewing is None or \
                self._previewing[0] != "joints":
            return frozenset()
        return out_of_reach(ctx.scene.snapshot, robot_id(serial), list(ctx.world.robots[serial].arms))

    def _notes_html(self, ctx: PluginContext, now: float) -> str:
        """The question Go answers, a refusal, the last outcome, where a running part is, the second presses."""
        procedure, part, owner = self.procedure, self.part, self.owner
        if part is None:
            return ""
        html = note(escape(procedure.prompt), CURRENT) if procedure.prompt and owner is part else ""
        if (refusal := self.refusal(now)) is not None:
            html += warning(escape(f"{refusal.label}: {refusal.detail}"))
        outcome = self._outcome_of(self.index)
        if (outcome is not None and outcome.detail and outcome.level in (BAD, WARN)
                and not self.is_done(ctx, part, self.index)):
            html += warning(escape(f"{part.label}: {outcome.detail}" if outcome.level == BAD else outcome.detail))
        if self.mode == OFF and self._auto_end is not None and self._auto_end.level == WARN:
            html += warning(escape(f"{self._auto_end.label}: {self._auto_end.detail}"))
        if owner is not None and owner is not part:
            what = "waits for Go" if procedure.prompt else "runs"
            html += note(escape(f"{self._name(owner)} {what}: Back or Next to it to answer; Cancel stops it here."),
                         CURRENT)
        if self._redo_press.waiting(now) == str(self.index):
            html += warning(escape(f"Press {REDO} again within {SECOND_PRESS_SECONDS:g} s: {part.label} and every "
                                   f"later part are to do again; their actions are reopened. Nothing moves."))
        elif self.at_end(ctx, part) and not self.is_done(ctx, part, self.index):
            html += note("The robot stands at this part's end already: Done goes on.", OK)
        return html

    def _list_html(self, ctx: PluginContext) -> str:
        """The shown action's parts, a line each (`overview_row`): number, label, what runs, the result."""
        if not self.parts:
            return ""
        first, count = self._action_span(self.part.action)
        rows = []
        for index in range(first, first + count):
            part, state = self.parts[index], self._state(ctx, index)
            shown = index == self.index
            running = running_chip(self.procedure) if part is self.owner else ""
            # * The kind after the state, unless the label says it already ("B20_J_place_bar: by hand").
            kind = "" if part.label.endswith(part.kind) else part.kind
            rows.append(overview_row(state_chip(self._number(index), state, False, shown), part.label, shown, running,
                                     replace(state, value=kind), "the part's chips"))
        return section("the action's parts", SECTION_CTRL, escape(self.part.action)) + block("".join(rows))

    def _draw_ghost(self, ctx: PluginContext) -> None:
        """Show the robot where Preview stands on the plan Go sends; hide it otherwise."""
        procedure = self.procedure
        point = procedure.previewed()
        # * The path by identity: two paths' arrays do not compare as one truth value.
        shown = (id(procedure.preview_path), procedure.preview)
        if shown != self._shown:
            self._shown = shown
            if point is not None:
                self._used.touch()
        serial = self._previewing[1] if self._previewing is not None else None
        for each, ghost in self._ghosts.items():
            if point is None or each != serial or not self._used.active:
                ghost.hide()
                continue
            joints = dict(ctx.kinematics.joints(each))
            if self._previewing[0] == "joints":
                joints.update(zip(self._previewing[2], (float(value) for value in point)))
                ghost.show(self._previewing[3], joints)
            else:
                x, y, yaw = (float(value) for value in point)
                z = float(ctx.kinematics.base_pose(each).position[2])
                ghost.show(Pose((x, y, z), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))), joints)

    # --- --- --- --- --- HELPERS --- --- --- --- ---

    def _record(self, ctx: PluginContext, event: str, part: GuidePart | None, **data) -> None:
        """A guide event in the run: what it did to which part, in which mode."""
        if part is None:
            return
        self._run(ctx).request_record("guide", action=part.action, movement=part.movement,
                                      data={"event": event, "part": part.label, "kind": part.kind, "mode": self.mode,
                                            **data})

    @staticmethod
    def _run(ctx: PluginContext) -> AssemblyRunContract:
        """The assembly_run plugin."""
        return ctx.require("assembly_run")

    @staticmethod
    def _process(ctx: PluginContext) -> ProcessContract:
        """The process plugin."""
        return ctx.require("process")

    @staticmethod
    def _executor(ctx: PluginContext) -> MovementExecContract:
        """The movement_exec plugin."""
        return ctx.require("movement_exec")
