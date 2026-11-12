"""
The Swiss Robotics Day demo: one bar into its joint and out again, on demand, the robot standing still, no mocap.

The tab **Insertion demo**: **Grasp** / **Release** close or open both gripper screws; **Tighten** / **Loosen**
run both joint screws by hand, until they stall or for **Loosen s**, and **Stop** halts both tools' motors at once;
**Zero F/T** zeroes both force sensors (with no bar in the tools); **Stow** plans a free move to the arms' park pose;
**Transit** plans a free dual-arm move from wherever the arms are to the insert's start (the transfer workaround, the
bar hidden); **Load** a free move to the loading pose, the tools empty; **Insert**
plans the process's insert from there (rigid, then compliant while the joint screws tighten until they stall);
**Retreat** loosens the joint screws for **Loosen s** and backs the tools out on the insert's line, the bar still held
(`parts.run_unscrew_retreat`); **Return** plans the insert's own planner backwards, a straight line with the bar held
to the insert's start, checked (`parts.return_problem`; a path that fails is shown, never run); **Compliant** puts
both arms under the compliance controller until **Rigid**. Every motion is planned on `motion_planner` from the live
joints.

**Insert mm** and **Retreat mm** set how far before its end the insert starts and how far the retreat backs out, on
the insert's line (`parts.line_start`); they start at the process's own length. **Save** writes the insert's start
for Insert mm into the process's action file, on its second press (`parts.save_action`).

With the Assembly tab's **Transfer workaround** box unticked the bar is carried instead, and the box stays unticked
all along: the Transit button reads **Transfer** and shows the path saved for the action (`parts.saved_carry`:
planned offline, the bar held by both arms) from the loading pose to the insert's start; **Load** shows the same path
reversed while the arms stand at the insert's start, and elsewhere a free move to the loading pose, only after a
**Release** (the tools empty); the cycle starts and ends at the loading pose.

A Move button only plans and shows its move: a see-through robot on the path at **Preview time**, timed for its own
motion time (**Transit s** or **Transfer s**, **Insert s**, **Retreat s**, **Return s**, **Load s**, **Stow s**; the
cycle takes each move's own), the bar and its joint halves in its tools on every move that holds the bar.
**Execute** runs the move shown, once.

**Cycle** runs one whole cycle from the insert's start: the insert, **Settle s** after the screws stall, the retreat,
then Return planned, checked and run. Nothing resumes after Cancel: Rigid, Retreat and Return by hand, then Cycle.
**Preview cycle** plans the same moves without sending anything and shows them as one path.

The Assembly tab's two plot windows are here too and open by themselves: **Plan plots** with the planned move's
joints and, for a move that holds the bar, how far one tool0 seen from the other moves from where it is at the first
waypoint (position, and the angle of each axis); **Exec plots** with both arms' force and torque while an insert, a
retreat or Compliant runs.

- Needs the process loaded in `process` (Load in the Assembly tab) and the base live: without mocap, `husky_fixed_base`
  publishes it standing at the origin, where the process puts it.
- ! One task at a time, the robot leased while a move runs. Cancel or Stop all ends it: a compliant part then only
  holds, the arms stay compliant, and Rigid brings them back.

Run with:  plugins:="insertion_demo robot_control"
"""

from __future__ import annotations

import asyncio
import math
from pathlib import Path
from dataclasses import dataclass
from typing import Awaitable

import numpy as np
import viser

from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.process import ProcessError
from scaffolding_assembly_core.process.solutions import CHAIN_TOLERANCE, Trajectory

from ...plugin_api.concurrency import WaitTimeout
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...robot_interface.arm import CARTESIAN_COMPLIANCE_CONTROLLER, SCALED_JOINT_TRAJECTORY_CONTROLLER
from ...robot_interface.end_effectors import ScaffoldingV3
from ...robot_interface.robot import HuskyRobotInterface
from ...shared.assembly_planning.joints import arm_joint_names
from ...shared.assembly_planning.movement import classify
from ...shared.assembly_planning.process_steps import LoadedProcess, loaded_process
from ...shared.assembly_planning.route_result import RouteResult
from ...shared.assembly_run_contract import is_live
from ...shared.assembly_tab.exec_plots import ExecPlots
from ...shared.assembly_tab.layout import EXEC_ICON, EXEC_PLOTS_X, PLAN_PLOTS_X, plots_window
from ...shared.assembly_tab.plan_plots import GraspDrift, PlanPlots
from ...shared.execution.controllers import leave_compliance
from ...shared.execution.force_sensors import zero_force_sensors
from ...shared.execution.insert import plan_insert, run_insert
from ...shared.execution.moving import arm_moving
from ...shared.execution.send import ARRIVED, motion_command, send_motion
from ...shared.execution.stream import HOLD_FOR_STALL, RETREATED, STALLED, Recorder
from ...shared.execution.tools import status_problem, stop_all
from ...shared.motion_planner_contract import MotionPlanRequest, MotionPlannerContract
from ...shared.movement_exec_contract import INSERT_PART, part_plan, serial_of
from ...shared.planner.planner_panel import SLIDER_STEP
from ...shared.planner.timed_path import TimedPath
from ...shared.process_contract import ProcessContract
from ...ui.ghost import RobotGhost
from ...ui.second_press import SecondPress, show_waiting
from ...ui.style import BUSY, FAIL, INFO, NONE, OK, SECTION_CTRL, block, chip, chip_line, note, section, values
from ...world.kinematics import stow_joints
from .parts import (AT_START, GRIP_TIMEOUT, back_out_rows, demo_steps, grasp_drift, held_bodies, line_length,
                    line_start, live_rows, load_step, off_start, relative_tool0, return_problem, run_compliant_hold,
                    run_grip, run_screws, run_unscrew_retreat, save_action, saved_carry, timed_preview, tool0_at,
                    with_insert_start, with_midpoints)

#: The sliders' ranges and starts: the insert's compliant millimetres, each motion time in seconds, the loosen, the
#: cycle's pause after the insert; the insert's and the retreat's length in millimetres (they start at the process's).
SPLIT_RANGE, SPLIT_DEFAULT = (0.5, 150.0), 10.0
LENGTH_RANGE = (5.0, 150.0)
SECONDS_RANGE, SECONDS_DEFAULT = (2.0, 60.0), 10.0
LOOSEN_RANGE, LOOSEN_DEFAULT = (1.0, 20.0), 6.0
SETTLE_RANGE, SETTLE_DEFAULT = (0.0, 30.0), 0.0
NO_ACTION = "no process"
SCAFFOLDING = "scaffolding_v3"
#: The Move buttons, by the step each plans.
MOVES = {"Insert": "insert", "Retreat": "retreat", "Return": "return", "Load": "load", "Stow": "stow"}
#: The key of the motion time slider the transit and the transfer share, and what each step's motion time sets.
TRAVEL_TIME = "travel"
TIME_HINTS = {
    TRAVEL_TIME: "the way to the insert's start: the transit, or the transfer (as the button reads)",
    "insert": "the insert, its rigid and its compliant part together",
    "retreat": "the retreat's back-out line, after the joint screws loosened for Loosen s",
    "return": "the return to the insert's start",
    "load": "the way to the loading pose: the free move, or the saved path reversed",
    "stow": "the move to the arms' park pose",
}
#: A cycle preview's parts, by the step whose motion time each takes.
PART_STEPS = {"back-out": "retreat", "transfer back": "load"}
#: The button of the way to the insert's start, by whether the bar is held on it: its label; the step is the same
#: word in small letters. Transit: a free move, the bar hidden. Transfer: the saved path, the bar in both tools.
TRAVEL = {False: "Transit", True: "Transfer"}
#: Seconds the cycle gives the arms to come to rest after a part, before the next is sent; and the least it waits,
#: so a twitch right after the switch back to joint control is seen.
REST_TIMEOUT, REST_AT_LEAST = 10.0, 1.0
#: The step name of a whole cycle shown only, and why Execute never runs it.
PREVIEW = "cycle preview"
PREVIEW_ONLY = "a preview only: Cycle runs the cycle"


@dataclass(frozen=True)
class Demo:
    """The loaded process's steps the demo runs: the action's transfer (index into `loaded.steps`) and insert.

    `held`: the bar travels in the tools along the saved carry (the process plugin's Transfer workaround unticked);
    False: Transit and Load are free moves.
    """

    loaded: LoadedProcess
    action_id: str
    transfer: int
    insert: int
    held: bool = False


@dataclass(frozen=True)
class Planned:
    """A move shown in the preview, waiting for Execute.

    Attributes:
        step: The button's step ("transit", "transfer", "insert", "retreat", "return", "load", "stow"), or PREVIEW.
        demo: The process steps it was planned for.
        names: The path's joint names.
        rows: The joint path from where the arms stood (N x len(names)), radians.
        base: The base pose it was planned at: where the preview robot stands.
        text: What made the path, for the panel and the log.
        limits: Whether it runs as a joint motion timed by the arms' limits; False for the back-out, a straight line
            streamed over the motion time.
        refused: Why it is only shown and never run (a return that failed its checks, a cycle's preview); "" if it
            may run.
        times: The steps whose motion times add up to the preview's, for a path that strings moves together; empty:
            the step's own.
        held: Whether the bar is in both tools all along: the preview robot then carries it.
        drift: How far one tool0 seen from the other moves along the path, for the Plan plots; None: not worked out.
    """

    step: str
    demo: Demo
    names: tuple[str, ...]
    rows: np.ndarray
    base: Pose
    text: str
    limits: bool = True
    refused: str = ""
    times: tuple[str, ...] = ()
    held: bool = False
    drift: GraspDrift | None = None


class NoQuestions:
    """A compliant part's operator that is never asked: the demo runs no spiral. A question is answered no, logged."""

    def __init__(self, ctx: PluginContext):
        """Keep the context for the log."""
        self._ctx = ctx

    def open(self, text: str, warn: bool = False) -> str:
        """Log the unexpected question; the handle is its text."""
        self._ctx.log_warn(f"insertion demo: unexpected question answered no: {text}")
        return text

    def poll(self, handle: str) -> bool | None:
        """No."""
        return False

    def close(self, handle: str) -> None:
        """Nothing to take away."""


@register
class InsertionDemoPlugin(HuskyPlugin):
    """The demo's tab: Grasp / Release, Tighten / Loosen, the Move buttons and their preview, Execute, Rigid, Cycle."""

    name = "insertion_demo"
    requires = ("process", "motion_planner")
    tab_path = ("Insertion demo",)
    icon = viser.Icon.REPEAT

    def __init__(self):
        """Idle, no robot found yet."""
        #: The serial of the robot with two scaffolding tools, found at setup; "" when there is none.
        self._serial = ""
        #: The running or last task, its step's name while it runs, and how the last one ended.
        self._task: asyncio.Task | None = None
        self.step = ""
        self.outcome = ""
        #: Set by Rigid while the Compliant task holds the arms; the task then brings them back.
        self._rigid_asked = False
        self._holding = False
        #: The loaded process's steps, rebuilt when the process plugin loads another process.
        self._loaded: LoadedProcess | None = None
        #: The demo robot's URDF; the process and action the length sliders were last set for, the millimetres they
        #: were set to (the process's own length), and the starts solved for other lengths, by (action, millimetres).
        self._urdf: Path | None = None
        self._lengths_for: tuple[LoadedProcess | None, str] = (None, "")
        self._authored_mm = math.nan
        self._lines: dict[tuple[str, float], dict[str, float] | str] = {}
        #: The saved carry last checked (its joint names, arms and rows), what `return_problem` said of it, and one
        #: tool0 seen from the other at its rows and between them (`with_midpoints`).
        self._carry_check: tuple[tuple, str, list[Pose]] | None = None
        #: Whether the tools are known to be empty: a Release ran to its end, and no gripper screw closed since.
        self._released = False
        #: Save's first press, waiting for its second.
        self._press = SecondPress()
        #: The move Execute runs, its path timed for the preview, and the shortest time the arms' limits allow (NaN
        #: without a joint motion).
        self._planned: Planned | None = None
        self._preview: TimedPath | None = None
        self.minimum = math.nan
        #: Seconds into the preview the slider and the see-through robot show; Play advances it.
        self._t = 0.0
        self._playing = False
        self._last_tick: float | None = None
        #: The preview's see-through robot; None without a demo robot. The process and action whose bar hangs in
        #: its tools; None: nothing hangs there.
        self._ghost: RobotGhost | None = None
        self._hung: tuple[LoadedProcess, str] | None = None
        #: The Plan plots window and what it shows: the preview and the planned move last plotted.
        self._plan_window: viser.PanelHandle | None = None
        self._plan_plots: PlanPlots | None = None
        self._plotted: TimedPath | None = None
        self._plotted_plan: Planned | None = None
        #: The Exec plots window's plots, the running compliant part's samples, and the time of the one last plotted.
        self._exec_plots: ExecPlots | None = None
        self._recorder: Recorder | None = None
        self._plotted_t: float | None = None

    @property
    def running(self) -> bool:
        """bool: Whether a task runs right now."""
        return self._task is not None and not self._task.done()

    # --- --- --- --- --- SETUP --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Find the robot, build the status line, the action list, the sliders, the buttons and the preview robot."""
        config = next((robot for robot in ctx.config.robots
                       if sum(arm.end_effector == SCAFFOLDING for arm in robot.arms) >= 2), None)
        self._serial = config.serial if config is not None else ""
        if config is None:
            ctx.log_warn("insertion demo: no configured robot carries two scaffolding tools; every button refuses")
        else:
            self._ghost = RobotGhost(ctx, config, "path")
            self._urdf = config.model.urdf
        with ctx.view.ui() as gui:
            # ! Always one line, so nothing below it moves.
            self._status: viser.GuiHtmlHandle = gui.add_html("")
            gui.add_html(section("demo", SECTION_CTRL))
            self._action = gui.add_dropdown("Action", [NO_ACTION], initial_value=NO_ACTION,
                                            hint="The jointing action whose insert the demo runs, once the process "
                                                 "is loaded in the Assembly tab")
            self._split = gui.add_slider("Compliant mm", min=SPLIT_RANGE[0], max=SPLIT_RANGE[1], step=0.5,
                                         initial_value=SPLIT_DEFAULT,
                                         hint="The insert's last millimetres run under the compliance controller")
            self._loosen = gui.add_slider("Loosen s", min=LOOSEN_RANGE[0], max=LOOSEN_RANGE[1], step=0.5,
                                          initial_value=LOOSEN_DEFAULT,
                                          hint="Seconds the joint screws run in reverse: in a retreat before the "
                                               "tools back out, and on Loosen")
            self._insert_mm = gui.add_slider("Insert mm", min=LENGTH_RANGE[0], max=LENGTH_RANGE[1], step=1.0,
                                             initial_value=LENGTH_RANGE[0],
                                             hint="How far before its end the insert starts, on its line: the "
                                                  "insert's start for Transit, Insert, Return and Cycle. Starts "
                                                  "at the process's own length.")
            self._retreat_mm = gui.add_slider("Retreat mm", min=LENGTH_RANGE[0], max=LENGTH_RANGE[1], step=1.0,
                                              initial_value=LENGTH_RANGE[0],
                                              hint="How far the retreat backs the tools out from the insert's end, "
                                                   "on the same line; Return then goes to the insert's start")
            self._save = gui.add_button("Save", icon=viser.Icon.DEVICE_FLOPPY,
                                        hint="Write the insert's start for Insert mm into the loaded process, "
                                             "overwriting its action file (on the Drive, for everyone); then press "
                                             f"Load in the Assembly tab. {self._press.hint}")
            bar = gui.add_button_group("Bar", ["Grasp", "Release"],
                                       hint="Close or open both gripper screws until they stall")
            joint = gui.add_button_group("Joint", ["Tighten", "Loosen", "Stop"],
                                         hint="Both joint screws by hand. Tighten: until they stall. Loosen: in "
                                              "reverse for Loosen s. Stop: both tools' motors halt at once, "
                                              "whatever runs them.")
            # * Its label says how the bar travels: Transit (Transfer workaround ticked) or Transfer (unticked).
            self._travel = gui.add_button(TRAVEL[False], icon=viser.Icon.ROUTE,
                                          hint="Plan the way to the insert's start and show it; nothing moves until "
                                               "Execute. Transit (Transfer workaround ticked in the Assembly tab): "
                                               "a free move from wherever the arms are, the bar hidden. Transfer "
                                               "(unticked): the saved path from the loading pose, the bar held in "
                                               "both tools; the preview robot carries the bar, and the Plan plots "
                                               "show how far the tools' relative pose moves.")
            move = gui.add_button_group("Move", list(MOVES),
                                        hint="Plan a move and show it in the preview; nothing moves until Execute. "
                                             "Insert: the process's insert from its start. Retreat: the joint "
                                             "screws loosen, then the tools back out "
                                             "Retreat mm, the bar still held. Return: a straight line "
                                             "with the bar held back to the insert's start, by the insert's "
                                             "planner, checked. Load: the way to the loading pose. With the "
                                             "Transfer button: the saved path reversed while the arms stand at the "
                                             "insert's start (the bar held), else a free move, only after Release "
                                             "(the tools empty). With Transit: always the free move. Stow: a free "
                                             "move to the arms' park pose, the bar unmounted and put away.")
            gui.add_html(section("motion time", SECTION_CTRL))
            # * One per move, also in the cycle. The first is labelled as the button above reads (`draw`).
            self._times = {step: gui.add_slider(f"{TRAVEL[False] if step == TRAVEL_TIME else step.capitalize()} s",
                                                min=SECONDS_RANGE[0], max=SECONDS_RANGE[1], step=0.5,
                                                initial_value=SECONDS_DEFAULT,
                                                hint=f"Seconds for {what}, by hand and in the cycle; the arms' "
                                                     f"limits may need longer")
                           for step, what in TIME_HINTS.items()}
            gui.add_html(section("preview", SECTION_CTRL))
            self._slider = gui.add_slider("Preview time", min=0.0, max=1.0, step=SLIDER_STEP, initial_value=0.0,
                                          hint="Scrub through the planned move: seconds from its start")
            # ! Always one line, so the buttons below never move.
            self._minimum: viser.GuiHtmlHandle = gui.add_html("")
            execute = gui.add_button("Execute", icon=EXEC_ICON,
                                     hint="Run the move the preview shows, once; a new Move button plans the next")
            path = gui.add_button_group("Path", ["Play", "Cancel"],
                                        hint="Play: run the preview. Cancel: stop the running step and hold; with "
                                             "nothing running, drop the planned move.")
            arms = gui.add_button_group("Arms", ["Compliant", "Rigid", "Zero F/T"],
                                        hint="Compliant: both arms give way to a push until Rigid. Rigid also "
                                             "brings back arms a cancelled part left compliant. Zero F/T: zero both "
                                             "force sensors where the arms stand, with no bar in the tools (a bar "
                                             "zeroed away makes the compliance sag under its weight later).")
            gui.add_html(section("autonomous", SECTION_CTRL))
            self._settle = gui.add_slider("Settle s", min=SETTLE_RANGE[0], max=SETTLE_RANGE[1], step=0.5,
                                          initial_value=SETTLE_DEFAULT,
                                          hint="The cycle's pause between the insert and the retreat, seconds")
            preview = gui.add_button("Preview cycle", icon=viser.Icon.EYE,
                                     hint="Plan the whole cycle for Insert mm and Retreat mm and show it as one "
                                          "path: the transit if the arms are not at the insert's start, the "
                                          "insert, the back-out, the return; with the Transfer button, the "
                                          "transfer before and after. Nothing moves; Execute refuses it.")
            cycle = gui.add_button("Cycle", icon=viser.Icon.REPEAT,
                                   hint="One cycle from the insert's start: Insert, the pause, Retreat, then Return "
                                        "planned, checked and run. With the Transfer button (Transfer workaround "
                                        "unticked): from the loading pose, the transfer before and after. Cancel "
                                        "stops it; then Rigid, Retreat and Return by hand before the next.")
            # * Refusals and outcomes come and go here, below the buttons.
            self._text: viser.GuiHtmlHandle = gui.add_html("")
        # * The Assembly tab's plot windows, minimized to their bars until a move is planned or a compliant part runs.
        with plots_window(ctx.view, "Plan plots", viser.Icon.CHART_LINE, PLAN_PLOTS_X) as self._plan_window:
            self._plan_plots = PlanPlots(ctx.view.gui, grasp=True)
        with plots_window(ctx.view, "Exec plots", viser.Icon.CHART_DOTS, EXEC_PLOTS_X) as window:
            self._exec_plots = ExecPlots(ctx.view.gui, window)
        for label, group in (("bar", bar), ("joint", joint), ("move", move), ("path", path), ("arms", arms)):
            group.on_click(ctx.defer_value(f"{label} button", lambda clicked: self._on_button(ctx, clicked)))
        self._travel.on_click(ctx.defer("travel", lambda: self._on_button(ctx, TRAVEL[self._held(ctx)])))
        execute.on_click(ctx.defer("execute", lambda: self._on_button(ctx, "Execute")))
        cycle.on_click(ctx.defer("cycle", lambda: self._on_button(ctx, "Cycle")))
        preview.on_click(ctx.defer("preview cycle", lambda: self._on_button(ctx, "Preview cycle")))
        self._save.on_click(ctx.defer("save", lambda: self._on_save(ctx)))
        for slider in (self._insert_mm, self._retreat_mm):
            slider.on_update(ctx.defer_value("length", lambda _value: self._on_length()))
        # ! Operator drags only: draw writes the slider every tick, and viser calls back for that too.
        self._slider.on_update(ctx.defer_value("scrub", lambda value: self._scrub(float(value)), operator_only=True))
        for slider in self._times.values():
            slider.on_update(ctx.defer_value("motion time", lambda _value: self._retime()))

    # --- --- --- --- --- BUTTONS (intents, on the main thread) --- --- --- --- ---

    def _on_button(self, ctx: PluginContext, clicked: str) -> None:
        """Start the step a button asks for, or say why not."""
        if clicked == "Cancel":
            if self.running:
                self._task.cancel()
                self._planner(ctx).request_cancel()
            elif self._planned is not None:
                self.outcome = f"the planned {self._planned.step} dropped"
                ctx.log_info(f"insertion demo: {self.outcome}")
                self._forget_plan()
            else:
                ctx.log_info("Cancel ignored: nothing runs and nothing is planned")
            return
        if clicked == "Play":
            self._play(ctx)
            return
        if clicked == "Stop":
            self._stop_tools(ctx)
            return
        if clicked == "Rigid" and self._holding:
            self._rigid_asked = True
            return
        robot = self._robot(ctx)
        if isinstance(robot, str):
            self._refuse(ctx, clicked, robot)
            return
        if self.running:
            self._refuse(ctx, clicked, f"{self.step} runs: wait, or Cancel it")
            return
        if clicked == "Rigid":
            self._start(ctx, "rigid", self._rigid(ctx, robot))
        elif clicked == "Compliant":
            self._start(ctx, "compliant", self._compliant(ctx, robot))
        elif clicked == "Zero F/T":
            self._start(ctx, "zero", self._zero(ctx, robot))
        elif clicked in ("Grasp", "Release"):
            self._start(ctx, clicked.lower(), self._grip(ctx, robot, clicked == "Grasp"))
        elif clicked in ("Tighten", "Loosen"):
            self._start(ctx, clicked.lower(), self._screws(ctx, robot, clicked == "Tighten"))
        elif clicked == "Execute":
            planned = self._planned
            if planned is None:
                self._refuse(ctx, clicked, "nothing planned: press a Move button first")
                return
            if planned.refused:
                # * Shown only: the path stays in the preview.
                self._refuse(ctx, clicked, f"the {planned.step} shown is not to run: {planned.refused}")
                return
            # ! Used once: a move never runs twice from one plan.
            self._forget_plan()
            self._start(ctx, planned.step, self._run(ctx, robot, planned))
        else:
            demo = self._demo(ctx)
            if isinstance(demo, str):
                self._refuse(ctx, clicked, demo)
                return
            self._forget_plan()
            if clicked == "Cycle":
                self._start(ctx, "cycle", self._cycle(ctx, robot, demo))
                return
            if clicked == "Preview cycle":
                self._start(ctx, "plan cycle preview", self._preview_cycle(ctx, robot, demo))
                return
            if clicked in TRAVEL.values():
                self._start(ctx, f"plan {TRAVEL[demo.held].lower()}", self._plan_travel(ctx, robot, demo))
                return
            plan = {"insert": self._plan_insert, "retreat": self._plan_retreat, "return": self._plan_return,
                    "load": self._plan_load, "stow": self._plan_stow}[MOVES[clicked]]
            self._start(ctx, f"plan {MOVES[clicked]}", plan(ctx, robot, demo))

    def _stop_tools(self, ctx: PluginContext) -> None:
        """STOP both tools' motors now, whatever runs them; a Tighten or Loosen under way is cancelled too."""
        robot = self._robot(ctx)
        if isinstance(robot, str):
            self._refuse(ctx, "Stop", robot)
            return
        stop_all(self._tools(robot))
        if self.step in ("tighten", "loosen"):
            self._task.cancel()
        else:
            self.outcome = "both tools' motors stopped"
            ctx.log_info(f"insertion demo: {self.outcome}")

    def _run(self, ctx: PluginContext, robot: HuskyRobotInterface, planned: Planned) -> Awaitable[str]:
        """The coroutine running a planned move: the insert, the retreat, or a joint motion (the other moves)."""
        if planned.step == "insert":
            return self._insert(ctx, robot, planned)
        if planned.step == "retreat":
            return self._retreat(ctx, robot, planned.demo)
        return self._move(ctx, robot, planned)

    def _start(self, ctx: PluginContext, step: str, work: Awaitable[str]) -> None:
        """Run `work` as the one task, keeping its step's name and its outcome for the panel."""
        self._task = ctx.spawn(f"demo {step}", self._guarded(ctx, step, work))

    async def _guarded(self, ctx: PluginContext, step: str, work: Awaitable[str]) -> None:
        """Run one step; its outcome, a cancel or a timeout ends up in `outcome`."""
        self.step, self.outcome = step, ""
        try:
            self.outcome = await work
        except asyncio.CancelledError:
            self.outcome = f"{step} cancelled"
            raise
        except WaitTimeout as timeout:
            self.outcome = f"{step} timed out: {timeout}"
        finally:
            self.step = ""
            (ctx.log_warn if "cancel" in self.outcome or "timed out" in self.outcome else ctx.log_info)(
                f"insertion demo: {self.outcome}")

    def _refuse(self, ctx: PluginContext, clicked: str, why: str) -> None:
        """Say why a button does nothing now."""
        self.outcome = f"{clicked} refused: {why}"
        ctx.log_warn(f"insertion demo: {self.outcome}")

    def _on_length(self) -> None:
        """Insert mm or Retreat mm changed: a move planned for the old length and a waiting Save are dropped."""
        self._forget_plan()
        self._press.forget()

    def _on_save(self, ctx: PluginContext) -> None:
        """Write the insert's start for Insert mm into the loaded process's action file, on the second press."""
        robot, demo = self._robot(ctx), self._demo(ctx)
        why = next((each for each in (robot, demo) if isinstance(each, str)), "")
        if not why and self.running:
            why = f"{self.step} runs: wait, or Cancel it"
        start = "" if why else self._line(robot, demo, self._insert_mm)
        if isinstance(start, str) and start:
            why = start
        elif not why and float(self._insert_mm.value) == self._authored_mm:
            why = f"Insert mm {self._authored_mm:g} is the process's own: nothing to save"
        if why:
            self._press.forget()
            self._refuse(ctx, "Save", why)
            return
        if not self._press.press(ctx.now(), "save"):
            return
        arms, joints = start
        mm = float(self._insert_mm.value)
        steps, process = demo.loaded.steps, demo.loaded.process
        moved = {name: joints[name] for arm in arms for name in arm_joint_names(arm)}
        try:
            edited = with_insert_start(process, demo.action_id, steps[demo.transfer].movement.id,
                                       steps[demo.insert].movement.id, moved, mm / 1e3)
            path = save_action(edited, demo.action_id)
        except (ProcessError, OSError) as error:
            self.outcome = f"Save failed, nothing written: {error}"
            ctx.log_error(f"insertion demo: {self.outcome}")
            return
        self.outcome = (f"saved: the insert starts {mm:g} mm before its end in {path.parent.parent.name}/actions/"
                        f"{path.name}; press Load in the Assembly tab to use it")
        ctx.log_warn(f"insertion demo: {self.outcome} ({path})")

    # --- --- --- --- --- THE PREVIEW --- --- --- --- ---

    def _show(self, ctx: PluginContext, planned: Planned) -> str:
        """Keep a planned move for Execute and time its preview; the panel's line, or why it can't be shown."""
        timed = timed_preview(planned.rows, self._seconds_of(planned), planned.limits)
        if isinstance(timed, str):
            return f"{planned.step}: {timed}"
        self._planned = planned
        self._preview, self.minimum = timed
        # * At the end, where the move arrives, as the Assembly tab's preview opens; Play runs it from the start.
        self._t, self._playing = self._preview.duration, False
        text = planned.text
        if planned.drift is not None:
            text += (f"; the tools' relative pose moves at most {planned.drift.mm.max():.2f} mm, "
                     f"{planned.drift.deg.max():.2f}°")
        if planned.refused:
            return f"{planned.step} shown ({text}), not to run: {planned.refused}"
        return f"{planned.step} planned ({text}): the motion takes {self._preview.duration:.1f} s; Execute runs it"

    def _forget_plan(self) -> None:
        """Drop the planned move and its preview."""
        self._planned = self._preview = None
        self.minimum = math.nan
        self._t, self._playing = 0.0, False

    def _seconds_for(self, step: str) -> float:
        """The motion time set for a step, seconds; the transit and the transfer share one slider."""
        return float(self._times[TRAVEL_TIME if step in ("transit", "transfer") else step].value)

    def _seconds_of(self, planned: Planned) -> float:
        """The motion time of a planned move, seconds: its step's, or the sum of its parts' (a cycle's preview)."""
        return sum(self._seconds_for(step) for step in planned.times or (planned.step,))

    def _retime(self) -> None:
        """A motion time changed: time the preview again."""
        if self._planned is None:
            return
        timed = timed_preview(self._planned.rows, self._seconds_of(self._planned), self._planned.limits)
        if not isinstance(timed, str):
            self._preview, self.minimum = timed
            self._t = min(self._t, self._preview.duration)

    def _scrub(self, t: float) -> None:
        """Show the preview at time `t`."""
        self._t, self._playing = t, False

    def _play(self, ctx: PluginContext) -> None:
        """Play the preview from where the slider stands, from the start when it stands at the end; or pause it."""
        if self._preview is None:
            ctx.log_info("Play ignored: nothing planned")
            return
        if not self._playing and self._t >= self._preview.duration:
            self._t = 0.0
        self._playing = not self._playing

    def update(self, ctx: PluginContext) -> None:
        """Advance a playing preview; drop a planned move the process, the action or the box no longer matches."""
        now = ctx.now()
        elapsed = 0.0 if self._last_tick is None else now - self._last_tick
        self._last_tick = now
        robot = ctx.world.robots.get(self._serial) if self._serial else None
        if robot is not None and any(tool.state.gripper_motor == ScaffoldingV3.TIGHTENING
                                     for tool in self._tools(robot).values()):
            # * Whoever closes a gripper screw (Grasp, a button on the tool): the tools may hold the bar again.
            self._released = False
        planned = self._planned
        if planned is None:
            return
        if self._loaded_now(ctx) is not planned.demo.loaded or str(self._action.value) != planned.demo.action_id \
                or planned.demo.held != self._held(ctx):
            self.outcome = (f"the planned {planned.step} dropped: the process, the action or the Transfer workaround "
                            f"box changed")
            ctx.log_warn(f"insertion demo: {self.outcome}")
            self._forget_plan()
        elif self._playing:
            self._t = min(self._t + elapsed, self._preview.duration)
            self._playing = self._t < self._preview.duration

    # --- --- --- --- --- WHAT THE STEPS NEED --- --- --- --- ---

    def _robot(self, ctx: PluginContext) -> HuskyRobotInterface | str:
        """The demo robot, or why there is none to command."""
        if not self._serial:
            return "no configured robot carries two scaffolding tools"
        robot = ctx.world.robots.get(self._serial)
        if robot is None:
            return f"{self._serial} is not connected"
        return robot

    def _planner(self, ctx: PluginContext) -> MotionPlannerContract:
        """The motion planner."""
        return ctx.require("motion_planner")

    def _process(self, ctx: PluginContext) -> ProcessContract:
        """The process plugin."""
        return ctx.require("process")

    def _loaded_now(self, ctx: PluginContext) -> LoadedProcess | None:
        """The loaded process's steps, built once per process the process plugin serves."""
        selection = self._process(ctx).selection
        if selection is None:
            self._loaded = None
        elif self._loaded is None or self._loaded.process is not selection.process:
            self._loaded = loaded_process(selection.process)
        return self._loaded

    def _actions(self, ctx: PluginContext, loaded: LoadedProcess) -> list[str]:
        """The scheduled actions of the demo robot with a transfer and an insert: the ones the demo can run."""
        return [action_id for action_id in loaded.process.schedule
                if serial_of(loaded, action_id, ctx.config.robots) == self._serial
                and not isinstance(demo_steps(loaded, action_id), str)]

    def _demo(self, ctx: PluginContext) -> Demo | str:
        """The chosen action's steps, or why there are none (no process loaded, no such action)."""
        loaded = self._loaded_now(ctx)
        if loaded is None:
            return "no process loaded: Load one in the Assembly tab"
        action_id = str(self._action.value)
        if action_id not in self._actions(ctx, loaded):
            return f"{action_id} is no jointing action of {self._serial} with a transfer and an insert"
        transfer, insert = demo_steps(loaded, action_id)
        return Demo(loaded, action_id, transfer, insert, self._held(ctx))

    def _held(self, ctx: PluginContext) -> bool:
        """Whether the bar is carried in the tools: the process plugin's Transfer workaround box is unticked."""
        return not self._process(ctx).transfer_as_free

    def _select(self, ctx: PluginContext, demo: Demo, index: int) -> None:
        """Select a step in the process plugin, so its bodies are in the scene and drawn."""
        movement = demo.loaded.steps[index].movement
        authored = next((k for k, (_action, each) in enumerate(demo.loaded.process.movements()) if each is movement),
                        None)
        if authored is not None:
            self._process(ctx).request_select(authored)

    async def _plan(self, ctx: PluginContext, demo: Demo, index: int, label: str,
                    goal_poses: dict[str, Pose] | None = None,
                    goal_joints: dict[str, float] | None = None,
                    start_joints: dict[str, float] | None = None) -> RouteResult | str:
        """Plan a step from the live joints on the motion planner; the route, or why there is none.

        `goal_poses` (process flange id -> world pose) or `goal_joints` (by joint name) replace the step's own goal
        where the planner takes them. `start_joints` replace the live joints as the start, for a preview of a move
        the arms are not yet at.
        """
        snapshot = ctx.scene.snapshot
        if start_joints is not None:
            snapshot = snapshot.copy()
            acting = snapshot.robots.get(robot_id(self._serial))
            if acting is not None:
                acting.joints = {**acting.joints, **start_joints}
        request = MotionPlanRequest(demo.loaded, demo.loaded.steps[index], snapshot, ctx.config.robots,
                                    transfer_as_free=True, obstacles_collide=self._process(ctx).obstacles_collide,
                                    goal_joints=goal_joints, goal_poses=goal_poses, label=label)
        try:
            route = await self._planner(ctx).request_plan(request)
        except asyncio.CancelledError:
            self._planner(ctx).request_cancel()
            raise
        if not route.ok:
            return f"{label}: no plan: {route.result.reason}"
        for warning in route.warnings:
            ctx.log_warn(f"{label}: {warning}")
        return route

    @staticmethod
    def _tools(robot: HuskyRobotInterface) -> dict[str, ScaffoldingV3]:
        """Arm name -> its scaffolding tool."""
        return {name: arm.end_effector for name, arm in robot.arms.items()
                if isinstance(arm.end_effector, ScaffoldingV3)}

    @staticmethod
    def _insert_start(demo: Demo) -> tuple[list[str], dict[str, float]] | str:
        """The insert's moving arms and its authored start joints, or why the insert has none."""
        step = demo.loaded.steps[demo.insert]
        spec = classify(demo.loaded.process, step.action, step.movement, transfer_as_free=True)
        robot = step.movement.start.robots.get(step.action.robot)
        if robot is None or robot.joints is None:
            return f"{step.movement.id} has no authored start joints"
        return [arm.arm for arm in spec.arms], dict(robot.joints)

    def _line(self, robot: HuskyRobotInterface, demo: Demo,
              slider: viser.GuiSliderHandle) -> tuple[list[str], dict[str, float]] | str:
        """The insert's moving arms and the joints `slider` millimetres before its end on its line; or why not.

        Insert mm gives the insert's start, Retreat mm where the retreat ends. A slider at the process's own length
        gives the authored start joints as they are.
        """
        start = self._insert_start(demo)
        if isinstance(start, str):
            return start
        arms, joints = start
        mm = float(slider.value)
        step = demo.loaded.steps[demo.insert]
        end = step.movement.target.joints.get(step.action.robot) if step.movement.target is not None else None
        if mm == self._authored_mm or not end or self._lengths_for != (demo.loaded, demo.action_id):
            return arms, joints
        key = (demo.action_id, mm)
        if key not in self._lines:
            self._lines[key] = line_start(robot.config.model.urdf, arms, joints, end, mm / 1e3)
        solved = self._lines[key]
        return f"{slider.label} {mm:g}: {solved}" if isinstance(solved, str) else (arms, solved)

    def _set_lengths(self, ctx: PluginContext) -> None:
        """Set Insert mm and Retreat mm to the process's own insert length when the process or the action changed."""
        loaded = self._loaded_now(ctx)
        chosen = (loaded, str(self._action.value))
        if chosen == self._lengths_for or self._urdf is None:
            return
        demo = self._demo(ctx)
        start = self._insert_start(demo) if not isinstance(demo, str) else demo
        if isinstance(start, str):
            return
        arms, joints = start
        step = demo.loaded.steps[demo.insert]
        end = step.movement.target.joints.get(step.action.robot) if step.movement.target is not None else None
        if not end:
            return
        mm = float(np.clip(round(line_length(self._urdf, arms[0], joints, end) * 1e3), *LENGTH_RANGE))
        self._lengths_for, self._authored_mm, self._lines = chosen, mm, {}
        self._insert_mm.value = self._retreat_mm.value = mm

    @staticmethod
    def _flanges(demo: Demo) -> dict[str, str]:
        """The insert's moving arms, each with its flange id as the process names it (the planner's goal keys)."""
        step = demo.loaded.steps[demo.insert]
        spec = classify(demo.loaded.process, step.action, step.movement, transfer_as_free=True)
        return {part.arm: part.flange for part in spec.arms}

    def _record(self, ctx: PluginContext, robot: HuskyRobotInterface, label: str) -> Recorder:
        """Open the Exec plots for a compliant part starting now; its samples go into the recorder returned."""
        self._recorder, self._plotted_t = Recorder(robot, ctx.now()), None
        self._exec_plots.start_compliant(list(robot.arms), f"{label} on {self._serial}")
        return self._recorder

    # --- --- --- --- --- THE STEPS (tasks) --- --- --- --- ---

    async def _grip(self, ctx: PluginContext, robot: HuskyRobotInterface, close: bool) -> str:
        """Close or open both gripper screws."""
        self._released, started = False, ctx.now()
        with ctx.lease_robots([self._serial], "grasp" if close else "release"):
            ok, text = await run_grip(ctx, self._tools(robot), close)
        # * Empty after a Release that ran to its end: the screws stalled open, or opened for the whole timeout (a
        #   screw may not stall when open). One refused or cancelled leaves the tools as they were.
        self._released = not close and (ok or ctx.now() - started >= GRIP_TIMEOUT)
        return text

    async def _screws(self, ctx: PluginContext, robot: HuskyRobotInterface, tighten: bool) -> str:
        """Run both joint screws: tightening until they stall, or loosening for Loosen s."""
        with ctx.lease_robots([self._serial], "tighten" if tighten else "loosen"):
            _ok, text = await run_screws(ctx, self._tools(robot), ScaffoldingV3.JOINT_MOTOR, tighten,
                                         seconds=None if tighten else float(self._loosen.value),
                                         timeout_s=HOLD_FOR_STALL)
        return text

    def _insert_tools(self, ctx: PluginContext, robot: HuskyRobotInterface,
                      demo: Demo) -> tuple[tuple[str, ...], dict[str, ScaffoldingV3]] | str:
        """The insert's mating arms and the robot's scaffolding tools, or why the insert can't run as one."""
        step = demo.loaded.steps[demo.insert]
        part = part_plan(demo.loaded, demo.action_id, step.movement_index, ctx.config.robots, transfer_as_free=True)
        if part.kind != INSERT_PART or not part.tools:
            return f"{part.movement}: {part.text}"
        tools = self._tools(robot)
        missing = [arm for arm in part.tools if arm not in tools]
        if missing:
            return f"{self._serial} has no scaffolding tool on {', '.join(missing)}"
        return tuple(part.tools), tools

    def _planned_route(self, ctx: PluginContext, step: str, demo: Demo, route: RouteResult, refused: str = "",
                       holding: tuple[HuskyRobotInterface, list[str]] | None = None) -> str:
        """Show a planner's route as the planned `step`; `refused` says why it is only shown.

        `holding`, the robot and the arms that hold the bar all along: the preview robot carries it and the Plan
        plots show the grasp.
        """
        trajectory = route.result.trajectory
        names, rows = tuple(trajectory.joint_names), np.array(trajectory.positions, dtype=float)
        return self._show(ctx, Planned(step, demo, names, rows, ctx.kinematics.base_pose(self._serial),
                                       f"{route.planner}, {route.waypoints} waypoints", refused=refused,
                                       held=holding is not None,
                                       drift=None if holding is None else self._drift(holding[0], names, rows,
                                                                                      holding[1])))

    @staticmethod
    def _drift(robot: HuskyRobotInterface, names: tuple[str, ...], rows: np.ndarray,
               arms: list[str]) -> GraspDrift | None:
        """How far one tool0 seen from the other moves along a path, at its rows and between them; None for one arm."""
        if len(arms) < 2 or len(rows) < 2:
            return None
        return grasp_drift(relative_tool0(robot, names, with_midpoints(rows), arms), 2)

    async def _plan_travel(self, ctx: PluginContext, robot: HuskyRobotInterface, demo: Demo) -> str:
        """Plan the way to the insert's start.

        The transit: a free dual-arm move from where the arms are, the held bar hidden. With the bar carried, the
        transfer: the saved carry, from the loading pose.
        """
        if demo.held:
            return self._plan_carry(ctx, robot, demo, "transfer")
        goal = self._transit_goal(robot, demo)
        if isinstance(goal, str):
            return f"transit: {goal}"
        self._select(ctx, demo, demo.transfer)
        route = await self._plan(ctx, demo, demo.transfer, "demo transit", goal_joints=goal)
        return route if isinstance(route, str) else self._planned_route(ctx, "transit", demo, route)

    def _transit_goal(self, robot: HuskyRobotInterface, demo: Demo) -> dict[str, float] | None | str:
        """The arm joints the transit goes to for Insert mm; None at the process's own start: the planner solves it."""
        start = self._line(robot, demo, self._insert_mm)
        if isinstance(start, str):
            return start
        arms, joints = start
        authored = self._insert_start(demo)[1]
        return None if joints == authored else {name: joints[name] for arm in arms for name in arm_joint_names(arm)}

    def _carry(self, robot: HuskyRobotInterface, demo: Demo) -> tuple[list[str], np.ndarray, list[Pose]] | str:
        """The saved carry from the loading pose to the insert's start, checked; or why it may not run.

        Checked on the monitor's robot model: the grasp kept along the way and no joint jump (`return_problem`), and
        its end at the insert's start for Insert mm.

        Returns:
            tuple[list[str], np.ndarray, list[Pose]] | str: Its joint names, its rows, and one tool0 seen from the
                other at the rows and between them (`with_midpoints`; empty for one arm); or the refusal.
        """
        start = self._line(robot, demo, self._insert_mm)
        if isinstance(start, str):
            return start
        arms, joints = start
        saved = saved_carry(demo.loaded.process, demo.action_id, demo.loaded.steps[demo.transfer].movement.id)
        if isinstance(saved, str):
            return saved
        names, rows = saved
        key = (tuple(names), tuple(arms), rows.tobytes())
        if self._carry_check is None or self._carry_check[0] != key:
            # * Once per saved path: the check takes a few hundred milliseconds of the tick.
            seen = relative_tool0(robot, names, with_midpoints(rows), arms) if len(arms) > 1 else []
            self._carry_check = (key, return_problem(robot, names, rows, arms, relative=seen[::2] or None), seen)
        _key, problem, seen = self._carry_check
        if problem:
            return f"the saved carry is refused: {problem}"
        gap = max((abs(value - joints.get(name, math.inf)) for name, value in zip(names, rows[-1])), default=math.inf)
        if gap > CHAIN_TOLERANCE:
            return (f"the saved carry does not end at the insert's start for Insert mm "
                    f"{float(self._insert_mm.value):g}: set Insert mm back to the process's own "
                    f"({self._authored_mm:g}), or Save, Load and plan the carry again")
        return names, rows, seen

    def _plan_carry(self, ctx: PluginContext, robot: HuskyRobotInterface, demo: Demo, step: str,
                    reverse: bool = False) -> str:
        """Show the saved carry as the planned `step`: to the insert's start, or reversed to the loading pose.

        No planner runs. With the arms away from the path's start it is shown only, never run.
        """
        carry = self._carry(robot, demo)
        if isinstance(carry, str):
            return f"{step}: {carry}"
        names, rows, seen = carry
        if reverse:
            rows, seen = rows[::-1].copy(), seen[::-1]
        refused = ""
        if isinstance(live_rows(robot, names, rows), str):
            refused = ("the arms do not stand at the insert's start, where the way back begins: press Return first"
                       if reverse else
                       "the arms do not stand at the loading pose, where the transfer begins: press Release and "
                       "Load (a free move, the tools empty), then place and Grasp the bar")
        self._select(ctx, demo, demo.transfer)
        return self._show(ctx, Planned(step, demo, tuple(names), rows, ctx.kinematics.base_pose(self._serial),
                                       f"the saved carry{' reversed' if reverse else ''}, {len(rows)} waypoints",
                                       refused=refused, held=True, drift=grasp_drift(seen, 2) if seen else None))

    async def _plan_load(self, ctx: PluginContext, robot: HuskyRobotInterface, demo: Demo) -> str:
        """Plan the way to the loading pose, the saved carry's first row.

        With the bar carried and the arms at the insert's start: the saved carry reversed, the bar held. Else a free
        move with empty tools; with the bar carried, only after a Release, as the tools may hold the bar.
        """
        if demo.held:
            carry = self._carry(robot, demo)
            if isinstance(carry, str):
                return f"load: {carry}"
            if not isinstance(live_rows(robot, carry[0], carry[1][::-1]), str):
                return self._plan_carry(ctx, robot, demo, "load", reverse=True)
            if not self._released:
                return ("load: the arms do not stand at the insert's start, where the way back with the bar begins, "
                        "and the tools may hold the bar: press Return first. With empty tools: Release, then Load "
                        "plans a free move to the loading pose")
        saved = saved_carry(demo.loaded.process, demo.action_id, demo.loaded.steps[demo.transfer].movement.id)
        index = load_step(demo.loaded, demo.action_id)
        why = next((each for each in (saved, index) if isinstance(each, str)), "")
        if why:
            return f"load: {why}"
        names, rows = saved
        self._select(ctx, demo, index)
        # * The travel to load goes to the joints given exactly, never a whole turn off: the carry starts there.
        route = await self._plan(ctx, demo, index, "demo load", goal_joints=dict(zip(names, rows[0].tolist())))
        if isinstance(route, str):
            return route
        return f"{self._planned_route(ctx, 'load', demo, route)} (a free move: the tools must be empty)"

    async def _plan_stow(self, ctx: PluginContext, robot: HuskyRobotInterface, demo: Demo) -> str:
        """Plan a free dual-arm move from where the arms are to their park pose: the transit's planner, that goal.

        The held bar is hidden from the search as for the transit, so plan it only once the bar is put away.
        """
        names = [name for arm in robot.arms for name in arm_joint_names(arm)]
        config = next((each for each in ctx.config.robots if each.serial == self._serial), None)
        parked = stow_joints(config, names) if config is not None else {}
        if len(parked) != len(names):
            return "stow: not every arm has park joints (crl-husky's park.yaml)"
        self._select(ctx, demo, demo.transfer)
        route = await self._plan(ctx, demo, demo.transfer, "demo stow", goal_joints=parked)
        return route if isinstance(route, str) else self._planned_route(ctx, "stow", demo, route)

    async def _plan_insert(self, ctx: PluginContext, robot: HuskyRobotInterface, demo: Demo) -> str:
        """Plan the process's insert from the live joints; refused unless the arms stand at its start."""
        start = self._line(robot, demo, self._insert_mm)
        if isinstance(start, str):
            return start
        arms, joints = start
        needs = self._insert_tools(ctx, robot, demo)
        if isinstance(needs, str):
            return needs
        off = off_start(robot, joints, arms)
        if off:
            return f"insert: {off}: press {TRAVEL[demo.held]} first"
        self._select(ctx, demo, demo.insert)
        route = await self._plan(ctx, demo, demo.insert, "demo insert")
        return route if isinstance(route, str) else self._planned_route(ctx, "insert", demo, route,
                                                                        holding=(robot, arms))

    async def _plan_retreat(self, ctx: PluginContext, robot: HuskyRobotInterface, demo: Demo) -> str:
        """Show the back-out: from where the arms are straight to Retreat mm before the end. No planner: a line."""
        start = self._line(robot, demo, self._retreat_mm)
        if isinstance(start, str):
            return start
        arms, joints = start
        path = back_out_rows(robot, joints, arms)
        if isinstance(path, str):
            return f"retreat: {path}"
        if tool0_at(robot, joints, arms) is None:
            return "retreat: no base pose from mocap"
        self._select(ctx, demo, demo.insert)
        names, rows = path
        return self._show(ctx, Planned("retreat", demo, tuple(names), rows, ctx.kinematics.base_pose(self._serial),
                                       f"the joint screws loosen {float(self._loosen.value):.1f} s, then a straight "
                                       f"back-out to {float(self._retreat_mm.value):g} mm before the insert's end",
                                       limits=False, held=True))

    async def _plan_return(self, ctx: PluginContext, robot: HuskyRobotInterface, demo: Demo) -> str:
        """Plan the way back to the insert's start: the insert's planner, a straight line with the bar held, checked.

        A path that fails its checks is still shown, marked refused: neither Execute nor the cycle runs it.
        """
        self._select(ctx, demo, demo.insert)
        found = await self._return_route(ctx, robot, demo, "demo return")
        if isinstance(found, str):
            return found
        route, problem = found
        shown = self._planned_route(ctx, "return", demo, route, refused=problem,
                                    holding=(robot, self._insert_start(demo)[0]))
        return f"return: refused: {problem}" if problem else shown

    async def _return_route(self, ctx: PluginContext, robot: HuskyRobotInterface, demo: Demo, label: str,
                            start_joints: dict[str, float] | None = None) -> tuple[RouteResult, str] | str:
        """The way back to the insert's start and why it may not run ("" if it may); or why there is no path.

        Checked: the grasp kept along the way, no joint jump, and the end at the insert's start (the planner steers
        only one arm's goal; the other follows the grasp). `start_joints` plan it from there instead of the live
        joints.
        """
        start = self._line(robot, demo, self._insert_mm)
        if isinstance(start, str):
            return start
        arms, joints = start
        ends = tool0_at(robot, joints, arms)
        if ends is None:
            return "return: no base pose from mocap"
        flanges = self._flanges(demo)
        route = await self._plan(ctx, demo, demo.insert, label, {flanges[arm]: ends[arm] for arm in arms},
                                 start_joints=start_joints)
        if isinstance(route, str):
            return route
        trajectory = route.result.trajectory
        problem = return_problem(robot, trajectory.joint_names, trajectory.positions, arms)
        for arm in arms:
            distance, turn = route.end_error.get(flanges[arm], (0.0, 0.0))
            if not problem and (distance > AT_START[0] or turn > AT_START[1]):
                problem = (f"{arm}'s tool0 ends {distance * 1e3:.1f} mm, {math.degrees(turn):.1f}° from the "
                           f"insert's start (limit {AT_START[0] * 1e3:.0f} mm, {math.degrees(AT_START[1]):.0f}°)")
        return route, problem

    async def _move(self, ctx: PluginContext, robot: HuskyRobotInterface, planned: Planned) -> str:
        """Run a planned joint motion (the transit or transfer, the return, the load, the stow) from the live joints."""
        step = planned.step
        with ctx.lease_robots([self._serial], step):
            path = live_rows(robot, planned.names, planned.rows)
            if isinstance(path, str):
                return f"{step}: {path}"
            try:
                command = motion_command(self._serial, list(robot.arms), *path, self._seconds_for(step))
            except ValueError as error:
                return f"{step}: the plan can't be sent: {error}"
            ctx.log_info(f"insertion demo: {step} along {planned.text}, {command.duration:.1f} s")
            sent = await send_motion(ctx, robot, command)
        return f"{step} {sent.status}: {sent.reason}"

    async def _insert(self, ctx: PluginContext, robot: HuskyRobotInterface, planned: Planned) -> str:
        """Run the planned insert from the live joints: rigid, then compliant while the joint screws tighten."""
        needs = self._insert_tools(ctx, robot, planned.demo)
        if isinstance(needs, str):
            return needs
        mating, tools = needs
        with ctx.lease_robots([self._serial], "insert"):
            path = live_rows(robot, planned.names, planned.rows)
            if isinstance(path, str):
                return f"insert: {path}"
            plan = plan_insert(robot, *path, mating=mating, grounded=False, split_mm=float(self._split.value),
                               spiral=None, traj_time=self._seconds_for("insert"))
            if isinstance(plan, str):
                return f"insert: {plan}"
            ctx.log_info(f"insertion demo: {plan.text}")
            try:
                outcome = await run_insert(ctx, robot, plan, tools, operator=NoQuestions(ctx), dry_run=False,
                                           recorder=self._record(ctx, robot, f"{planned.demo.action_id} insert"))
            finally:
                self._recorder = None
        return f"insert {outcome.status}: {outcome.reason}"

    async def _retreat(self, ctx: PluginContext, robot: HuskyRobotInterface, demo: Demo) -> str:
        """Loosen the joint screws, then back the tools out to Retreat mm before the end, the bar still held."""
        start = self._line(robot, demo, self._retreat_mm)
        if isinstance(start, str):
            return start
        arms, joints = start
        tools = self._tools(robot)
        missing = [arm for arm in arms if arm not in tools]
        if missing:
            return f"{self._serial} has no scaffolding tool on {', '.join(missing)}"
        ends = tool0_at(robot, joints, arms)
        if ends is None:
            return "retreat: no base pose from mocap"
        with ctx.lease_robots([self._serial], "retreat"):
            try:
                outcome = await run_unscrew_retreat(
                    ctx, robot, {arm: tools[arm] for arm in arms}, ends, loosen_s=float(self._loosen.value),
                    seconds=self._seconds_for("retreat"),
                    recorder=self._record(ctx, robot, f"{demo.action_id} retreat"))
            finally:
                self._recorder = None
        return f"retreat {outcome.status}: {outcome.reason}"

    @staticmethod
    async def _rest(ctx: PluginContext, robot: HuskyRobotInterface) -> None:
        """Wait until no arm counts as moving (`arm_moving`), REST_TIMEOUT at most; then the next part's guard rules."""
        since = ctx.now()
        try:
            await ctx.wait_until(lambda: ctx.now() - since >= REST_AT_LEAST and not any(
                arm_moving(arm, name, ctx.now()) for name, arm in robot.arms.items()),
                timeout_s=REST_TIMEOUT, description="the arms to come to rest")
        except WaitTimeout as timeout:
            ctx.log_warn(f"insertion demo: {timeout}; going on")

    async def _cycle(self, ctx: PluginContext, robot: HuskyRobotInterface, demo: Demo) -> str:
        """One cycle from the insert's start: insert, settle, retreat, then the way back planned, checked and run.

        With the bar carried (`demo.held`) it starts and ends at the loading pose: the saved carry before the insert
        and, reversed, after the return. Each move is shown while it runs. The first phase that does not end well
        stops the cycle, the arms left as that phase leaves them.
        """
        start = self._line(robot, demo, self._insert_mm)
        if isinstance(start, str):
            return f"cycle refused: {start}"
        arms, joints = start
        needs = self._insert_tools(ctx, robot, demo)
        if isinstance(needs, str):
            return f"cycle refused: {needs}"
        off = "" if demo.held else off_start(robot, joints, arms)
        if off:
            return f"cycle refused: {off}: press {TRAVEL[False]} or Return first"

        async def planned_move(phase: str, shown: str) -> str:
            """Run the move a plan phase just showed; how it ended, or why it did not run."""
            planned = self._planned
            if planned is None or planned.refused:
                # * A refused path stays in the preview, to look at.
                return f"cycle stopped at plan {phase}: {shown}"
            self.step = f"cycle: {phase}"
            try:
                return await self._run(ctx, robot, planned)
            finally:
                self._forget_plan()

        async def carried(phase: str, reverse: bool) -> str:
            """Run the saved carry as `phase`, then let the arms rest; "" if it arrived, else why the cycle stops."""
            self.step = f"cycle: plan {phase}"
            shown = self._plan_carry(ctx, robot, demo, phase, reverse)
            # ! A tick passes before the carry is sent: reading it took time, and an arm refuses a trajectory it
            #   would check against a joint state that old.
            await ctx.next_tick()
            text = await planned_move(phase, shown)
            if text.startswith("cycle stopped"):
                return text
            if not text.startswith(f"{phase} {ARRIVED}"):
                return f"cycle stopped at {phase}: {text}"
            self.step = "cycle: rest"
            await self._rest(ctx, robot)
            return ""

        if demo.held:
            stopped = await carried("transfer", reverse=False)
            if stopped:
                return stopped
        self.step = "cycle: plan insert"
        self._select(ctx, demo, demo.insert)
        route = await self._plan(ctx, demo, demo.insert, "demo cycle insert")
        if isinstance(route, str):
            return f"cycle stopped at plan insert: {route}"
        text = await planned_move("insert", self._planned_route(ctx, "insert", demo, route, holding=(robot, arms)))
        if text.startswith("cycle stopped"):
            return text
        if not text.startswith(f"insert {STALLED}"):
            return f"cycle stopped at insert: {text}"
        self.step = "cycle: settle"
        settle = float(self._settle.value)
        ctx.log_info(f"insertion demo: {text}; settling {settle:g} s before the retreat")
        await ctx.sleep(settle)
        await self._rest(ctx, robot)
        self.step = "cycle: retreat"
        text = await self._retreat(ctx, robot, demo)
        if not text.startswith(f"retreat {RETREATED}"):
            return f"cycle stopped at retreat: {text}"
        # * The arms settle for a moment after the switch back: a trajectory sent to a moving arm is refused.
        self.step = "cycle: rest"
        await self._rest(ctx, robot)
        self.step = "cycle: plan return"
        text = await planned_move("return", await self._plan_return(ctx, robot, demo))
        if text.startswith("cycle stopped"):
            return text
        if not text.startswith(f"return {ARRIVED}"):
            return f"cycle stopped at return: {text}"
        if demo.held:
            self.step = "cycle: rest"
            await self._rest(ctx, robot)
            return await carried("load", reverse=True) or "cycle done: back at the loading pose; Cycle runs the next"
        off = off_start(robot, joints, arms)
        if off:
            return f"cycle done, but {off}: press Return before the next"
        return f"cycle done ({text}): back at the insert's start; Cycle runs the next"

    async def _preview_cycle(self, ctx: PluginContext, robot: HuskyRobotInterface, demo: Demo) -> str:
        """Plan the whole cycle for Insert mm and Retreat mm and show it as one path; nothing is sent.

        The transit (only when the arms are not at the insert's start), the insert from that start, the back-out as
        a straight joint move, the return from where it ends. With the bar carried: the saved carry (the transfer)
        instead of the transit, and again, reversed, after the return. A part that can't be planned ends the path
        there.
        """
        start, back = self._line(robot, demo, self._insert_mm), self._line(robot, demo, self._retreat_mm)
        goal = self._transit_goal(robot, demo)
        why = next((each for each in (start, back, goal) if isinstance(each, str)), "")
        if why:
            return f"{PREVIEW}: {why}"
        arms, joints = start
        names = [name for arm in arms for name in arm_joint_names(arm)]

        def rows_of(route: RouteResult) -> np.ndarray:
            """A route's joint path, its columns in `names` order."""
            trajectory = route.result.trajectory
            columns = [list(trajectory.joint_names).index(name) for name in names]
            return np.array(trajectory.positions, dtype=float)[:, columns]

        parts: list[tuple[str, np.ndarray]] = []
        problem = ""
        carry = None
        if demo.held:
            saved = self._carry(robot, demo)
            if isinstance(saved, str):
                return f"{PREVIEW}: {saved}"
            carry = saved[1][:, [saved[0].index(name) for name in names]]
            parts.append(("transfer", carry))
        elif off_start(robot, joints, arms):
            self.step = f"plan {PREVIEW}: transit"
            self._select(ctx, demo, demo.transfer)
            route = await self._plan(ctx, demo, demo.transfer, "demo preview transit", goal_joints=goal)
            if isinstance(route, str):
                return f"{PREVIEW}: {route}"
            parts.append(("transit", rows_of(route)))
        self.step = f"plan {PREVIEW}: insert"
        self._select(ctx, demo, demo.insert)
        at_start = {name: joints[name] for name in names}
        route = await self._plan(ctx, demo, demo.insert, "demo preview insert", start_joints=at_start)
        if isinstance(route, str):
            problem = route
        else:
            parts.append(("insert", rows_of(route)))
            backed = {name: back[1][name] for name in names}
            parts.append(("back-out", np.array([parts[-1][1][-1], [backed[name] for name in names]])))
            self.step = f"plan {PREVIEW}: return"
            found = await self._return_route(ctx, robot, demo, "demo preview return", start_joints=backed)
            if isinstance(found, str):
                problem = found
            else:
                parts.append(("return", rows_of(found[0])))
                problem = f"return: refused: {found[1]}" if found[1] else ""
                if carry is not None and not problem:
                    parts.append(("transfer back", carry[::-1]))
        if not parts:
            return f"{PREVIEW}: {problem}"
        # * One path: each part starts where the one before ended; a repeated row would stall the timing.
        rows = [parts[0][1][0]]
        for _name, part in parts:
            for row in part:
                if not np.allclose(row, rows[-1], atol=1e-9):
                    rows.append(row)
        # * The bar is in the tools all along, unless the path starts with the free transit.
        held = parts[0][0] != "transit"
        rows = np.array(rows)
        shown = self._show(ctx, Planned(PREVIEW, demo, tuple(names), rows, ctx.kinematics.base_pose(self._serial),
                                        ", ".join(name for name, _part in parts), refused=PREVIEW_ONLY,
                                        times=tuple(PART_STEPS.get(name, name) for name, _part in parts),
                                        held=held,
                                        drift=grasp_drift(relative_tool0(robot, names, rows, arms))
                                        if held and len(arms) > 1 else None))
        return f"{shown}; stopped at: {problem}" if problem else shown

    async def _compliant(self, ctx: PluginContext, robot: HuskyRobotInterface) -> str:
        """Both arms under the compliance controller until Rigid."""
        self._rigid_asked, self._holding = False, True
        try:
            with ctx.lease_robots([self._serial], "compliant"):
                failed = await run_compliant_hold(ctx, list(robot.arms.values()), lambda: self._rigid_asked,
                                                  label=f"{self._serial}: compliant",
                                                  recorder=self._record(ctx, robot, "hold"))
        finally:
            self._holding, self._recorder = False, None
        return failed or "both arms back under joint control"

    async def _zero(self, ctx: PluginContext, robot: HuskyRobotInterface) -> str:
        """Zero both arms' force sensors where they stand; for arms holding no bar."""
        with ctx.lease_robots([self._serial], "zero"):
            zeroed = await zero_force_sensors(ctx, list(robot.arms.values()), f"{self._serial}: zero")
        missed = [name for name, each in zeroed.items() if not each["sent"]]
        if missed:
            return f"zero: {', '.join(missed)}: force sensor NOT zeroed (service not available)"
        return "force sensors zeroed: |F| now " + ", ".join(
            f"{name} {each['force_after_N']:.1f} N" for name, each in zeroed.items())

    async def _rigid(self, ctx: PluginContext, robot: HuskyRobotInterface) -> str:
        """Bring arms left compliant (a cancelled part) back under joint control."""
        with ctx.lease_robots([self._serial], "rigid"):
            back = await leave_compliance(ctx, list(robot.arms.values()), label=f"{self._serial}: rigid")
        return "both arms under joint control" if back else "not every arm is confirmed back: the log says"

    # --- --- --- --- --- DRAW --- --- --- --- ---

    def draw(self, ctx: PluginContext) -> None:
        """The status chips, the action list, the preview and the last outcome."""
        loaded = self._loaded_now(ctx)
        actions = self._actions(ctx, loaded) if loaded is not None else []
        options = actions or [NO_ACTION]
        if list(self._action.options) != options:
            self._action.options = options
            self._action.value = options[-1]
        self._set_lengths(ctx)
        show_waiting(self._save, bool(self._press.waiting(ctx.now())), "Save", None)
        travel = TRAVEL[self._held(ctx)]
        if self._travel.label != travel:
            self._travel.label = travel
            self._times[TRAVEL_TIME].label = f"{travel} s"
        robot = ctx.world.robots.get(self._serial) if self._serial else None
        self._status.content = chip_line(self._chips(ctx, robot, actions))
        self._text.content = note(self.outcome) if self.outcome else ""
        self._slider.max = max(self._preview.duration if self._preview is not None else 0.0, SLIDER_STEP)
        self._slider.value = round(self._t, 3)
        minimum = "—" if math.isnan(self.minimum) else f"{self.minimum:.1f}"
        self._minimum.content = block(values(f"min {minimum} s by the arms' limits"))
        if self._ghost is not None:
            planned = self._planned
            # * The bar and its joint halves hang in the preview robot's tools on a move that holds the bar.
            hung = (planned.demo.loaded, planned.demo.action_id) if planned is not None and planned.held else None
            if hung != self._hung:
                self._hung = hung
                missing = self._ghost.hold(held_bodies(planned.demo.loaded, planned.demo.insert) if hung else {})
                if missing:
                    ctx.log_warn(f"insertion demo: the preview robot has no link to carry {', '.join(missing)}")
            if planned is None:
                self._ghost.hide()
            else:
                path = dict(zip(planned.names, self._preview.sample(self._t)))
                self._ghost.show(planned.base, {**ctx.kinematics.joints(self._serial), **path})
        self._draw_plots()

    def _draw_plots(self) -> None:
        """The planned move's joints and grasp into the Plan plots, a compliant part's newest sample into the Exec's."""
        if self._preview is not self._plotted:
            # * A new plan, a retime after a motion-time change, or none (Execute, Cancel).
            planned = self._planned
            if planned is not None and planned is not self._plotted_plan:
                # * A new plan brings the window forward, as in the Assembly tab; only the operator closes it.
                self._plan_window.expand()
            self._plotted, self._plotted_plan = self._preview, planned
            self._plan_plots.show(self._preview, None if planned is None else Trajectory(
                self._serial, planned.names, tuple(map(tuple, planned.rows.tolist()))),
                None if planned is None else planned.drift)
        recorder = self._recorder
        if recorder is not None and recorder.samples and recorder.samples[-1]["t"] != self._plotted_t:
            self._plotted_t = recorder.samples[-1]["t"]
            self._exec_plots.compliant_sample(recorder.samples[-1])
        self._exec_plots.draw()

    def _chips(self, ctx: PluginContext, robot: HuskyRobotInterface | None, actions: list[str]) -> str:
        """The robot live or not, the action, the tools, the arms' controller, the planned move, the running step."""
        now = ctx.now()
        if robot is None:
            chips = chip("no robot", FAIL, "no connected robot with two scaffolding tools")
        else:
            live = is_live(ctx.scene.snapshot.robots.get(robot_id(self._serial)), now) \
                if robot_id(self._serial) in ctx.scene.snapshot.robots else False
            chips = chip(self._serial, OK if live else FAIL,
                         "base and joints measured and fresh" if live else "not live: base or joints missing or old")
        chips += chip(str(self._action.value) if actions else NO_ACTION, INFO if actions else FAIL,
                      "the action the demo runs" if actions else "load the process in the Assembly tab")
        held = self._held(ctx)
        chips += chip("bar held" if held else "bar free", INFO if held else NONE,
                      "Transfer workaround is unticked (Assembly tab): Transfer and Load carry the bar along the "
                      "saved path" if held else
                      "Transfer workaround is ticked (Assembly tab): Transit and Load are free moves, the bar "
                      "hidden or the tools empty")
        if robot is not None:
            silent = status_problem(ctx, self._tools(robot))
            chips += chip("tools", FAIL if silent else OK, silent or "both scaffolding tools report")
            running = {arm.state.controllers.active for arm in robot.arms.values()}
            text, color = {frozenset({SCALED_JOINT_TRAJECTORY_CONTROLLER}): ("joint", OK),
                           frozenset({CARTESIAN_COMPLIANCE_CONTROLLER}): ("compliant", BUSY)}.get(
                frozenset(running), ("mixed", FAIL))
            chips += chip(text, color, "what both arms' controller managers run; mixed or unknown is red")
        planned = self._planned
        if planned is None:
            chips += chip("no plan", NONE, "A Move button plans a move and shows it; Execute runs it")
        elif planned.step == PREVIEW:
            chips += chip(PREVIEW, INFO, "the whole cycle, shown only: Cycle runs it")
        elif planned.refused:
            chips += chip(f"{planned.step} refused", FAIL, f"shown only: {planned.refused}")
        elif self.running:
            chips += chip(f"{planned.step} running", BUSY, "the cycle runs the move shown")
        elif planned.limits and robot is not None and isinstance(live_rows(robot, planned.names, planned.rows), str):
            chips += chip(f"{planned.step} stale", BUSY, "the arms are not at the path's start: press the move again")
        else:
            chips += chip(f"{planned.step} ready", OK, "Execute runs the move the preview shows")
        chips += chip(self.step or "idle", BUSY if self.step else NONE, "the running step")
        return chips
