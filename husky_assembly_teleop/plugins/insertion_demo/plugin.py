"""
The Swiss Robotics Day demo: one bar into its joint and out again, on demand, the robot standing still, no mocap.

The tab **Insertion demo**: **Grasp** / **Release** close or open both gripper screws; **Approach** plans a free
dual-arm move from wherever the arms are to the insert's start (the transfer workaround, the bar hidden); **Insert**
plans the process's insert from there (rigid, then compliant while the joint screws tighten until they stall);
**Retreat** loosens the joint screws for **Loosen s** and backs the tools out to the insert's start, the bar still held
(`parts.run_unscrew_retreat`); **Compliant** puts both arms under the compliance controller until **Rigid**. Every
motion is planned on `motion_planner` from the live joints.

A Move button only plans and shows its move: a see-through robot on the path at **Preview time**, timed for **Motion
time**. **Execute** runs the move shown, once.

The Assembly tab's two plot windows are here too and open by themselves: **Plan plots** with the planned move's
joints, **Exec plots** with both arms' force and torque while an insert, a retreat or Compliant runs.

- Needs the process loaded in `process` (Load in the Assembly tab) and the base live: without mocap, `husky_fixed_base`
  publishes it standing at the origin, where the process puts it.
- ! One task at a time, the robot leased while a move runs. Cancel or Stop all ends it: a compliant part then only
  holds, the arms stay compliant, and Rigid brings them back.

Run with:  plugins:="insertion_demo robot_control"
"""

from __future__ import annotations

import asyncio
import math
from dataclasses import dataclass
from typing import Awaitable

import numpy as np
import viser

from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.process.solutions import Trajectory

from ...plugin_api.concurrency import WaitTimeout
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...robot_interface.arm import CARTESIAN_COMPLIANCE_CONTROLLER, SCALED_JOINT_TRAJECTORY_CONTROLLER
from ...robot_interface.end_effectors import ScaffoldingV3
from ...robot_interface.robot import HuskyRobotInterface
from ...shared.assembly_planning.movement import classify
from ...shared.assembly_planning.process_steps import LoadedProcess, loaded_process
from ...shared.assembly_planning.route_result import RouteResult
from ...shared.assembly_run_contract import is_live
from ...shared.assembly_tab.exec_plots import ExecPlots
from ...shared.assembly_tab.layout import EXEC_ICON, EXEC_PLOTS_X, PLAN_PLOTS_X, plots_window
from ...shared.assembly_tab.plan_plots import PlanPlots
from ...shared.execution.controllers import leave_compliance
from ...shared.execution.insert import plan_insert, run_insert
from ...shared.execution.send import motion_command, send_motion
from ...shared.execution.stream import Recorder
from ...shared.execution.tools import status_problem
from ...shared.motion_planner_contract import MotionPlanRequest, MotionPlannerContract
from ...shared.movement_exec_contract import INSERT_PART, part_plan, serial_of
from ...shared.planner.planner_panel import SLIDER_STEP
from ...shared.planner.timed_path import TimedPath
from ...shared.process_contract import ProcessContract
from ...ui.ghost import RobotGhost
from ...ui.style import BUSY, FAIL, INFO, NONE, OK, SECTION_CTRL, block, chip, chip_line, note, section, values
from .parts import (back_out_rows, demo_steps, live_rows, off_start, run_compliant_hold, run_grip,
                    run_unscrew_retreat, timed_preview, tool0_at)

#: The sliders' ranges and starts: the insert's compliant millimetres, the motion time in seconds, the loosen.
SPLIT_RANGE, SPLIT_DEFAULT = (0.5, 30.0), 10.0
SECONDS_RANGE, SECONDS_DEFAULT = (2.0, 60.0), 10.0
LOOSEN_RANGE, LOOSEN_DEFAULT = (1.0, 20.0), 6.0
NO_ACTION = "no process"
SCAFFOLDING = "scaffolding_v3"
#: The Move buttons, by the step each plans.
MOVES = {"Approach": "approach", "Insert": "insert", "Retreat": "retreat"}


@dataclass(frozen=True)
class Demo:
    """The loaded process's steps the demo runs: the action's transfer (index into `loaded.steps`) and insert."""

    loaded: LoadedProcess
    action_id: str
    transfer: int
    insert: int


@dataclass(frozen=True)
class Planned:
    """A move shown in the preview, waiting for Execute.

    Attributes:
        step: "approach", "insert" or "retreat".
        demo: The process steps it was planned for.
        names: The path's joint names.
        rows: The joint path from where the arms stood (N x len(names)), radians.
        base: The base pose it was planned at: where the preview robot stands.
        text: What made the path, for the panel and the log.
        limits: Whether it runs as a joint motion timed by the arms' limits; False for the back-out, a straight line
            streamed over the motion time.
    """

    step: str
    demo: Demo
    names: tuple[str, ...]
    rows: np.ndarray
    base: Pose
    text: str
    limits: bool = True


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
    """The demo's tab: Grasp / Release, Approach / Insert / Retreat with their preview, Execute, Compliant / Rigid."""

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
        #: The move Execute runs, its path timed for the preview, and the shortest time the arms' limits allow (NaN
        #: without a joint motion).
        self._planned: Planned | None = None
        self._preview: TimedPath | None = None
        self.minimum = math.nan
        #: Seconds into the preview the slider and the see-through robot show; Play advances it.
        self._t = 0.0
        self._playing = False
        self._last_tick: float | None = None
        #: The preview's see-through robot; None without a demo robot.
        self._ghost: RobotGhost | None = None
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
                                          hint="Seconds the joint screws run in reverse before the tools back out")
            bar = gui.add_button_group("Bar", ["Grasp", "Release"],
                                       hint="Close or open both gripper screws until they stall")
            move = gui.add_button_group("Move", list(MOVES),
                                        hint="Plan a move and show it in the preview; nothing moves until Execute. "
                                             "Approach: a free move to the insert's start from wherever the arms are "
                                             "(the bar in the tools, hidden). Insert: the process's insert from "
                                             "there. Retreat: the joint screws loosen, then the tools back out to "
                                             "the insert's start, the bar still held.")
            gui.add_html(section("preview", SECTION_CTRL))
            self._slider = gui.add_slider("Preview time", min=0.0, max=1.0, step=SLIDER_STEP, initial_value=0.0,
                                          hint="Scrub through the planned move: seconds from its start")
            self._seconds = gui.add_slider("Motion time", min=SECONDS_RANGE[0], max=SECONDS_RANGE[1], step=0.5,
                                           initial_value=SECONDS_DEFAULT,
                                           hint="How long the approach, the insert or the back-out takes, seconds; "
                                                "the arms' limits may need longer")
            # ! Always one line, so the buttons below never move.
            self._minimum: viser.GuiHtmlHandle = gui.add_html("")
            execute = gui.add_button("Execute", icon=EXEC_ICON,
                                     hint="Run the move the preview shows, once; a new Move button plans the next")
            path = gui.add_button_group("Path", ["Play", "Cancel"],
                                        hint="Play: run the preview. Cancel: stop the running step and hold; with "
                                             "nothing running, drop the planned move.")
            arms = gui.add_button_group("Arms", ["Compliant", "Rigid"],
                                        hint="Compliant: both arms give way to a push until Rigid. Rigid also "
                                             "brings back arms a cancelled part left compliant.")
            # * Refusals and outcomes come and go here, below the buttons.
            self._text: viser.GuiHtmlHandle = gui.add_html("")
        # * The Assembly tab's plot windows, minimized to their bars until a move is planned or a compliant part runs.
        with plots_window(ctx.view, "Plan plots", viser.Icon.CHART_LINE, PLAN_PLOTS_X) as self._plan_window:
            self._plan_plots = PlanPlots(ctx.view.gui)
        with plots_window(ctx.view, "Exec plots", viser.Icon.CHART_DOTS, EXEC_PLOTS_X) as window:
            self._exec_plots = ExecPlots(ctx.view.gui, window)
        for label, group in (("bar", bar), ("move", move), ("path", path), ("arms", arms)):
            group.on_click(ctx.defer_value(f"{label} button", lambda clicked: self._on_button(ctx, clicked)))
        execute.on_click(ctx.defer("execute", lambda: self._on_button(ctx, "Execute")))
        # ! Operator drags only: draw writes the slider every tick, and viser calls back for that too.
        self._slider.on_update(ctx.defer_value("scrub", lambda value: self._scrub(float(value)), operator_only=True))
        self._seconds.on_update(ctx.defer_value("motion time", lambda value: self._retime(float(value))))

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
        elif clicked in ("Grasp", "Release"):
            self._start(ctx, clicked.lower(), self._grip(ctx, robot, clicked == "Grasp"))
        elif clicked == "Execute":
            planned = self._planned
            if planned is None:
                self._refuse(ctx, clicked, "nothing planned: press Approach, Insert or Retreat first")
                return
            # ! Used once: a move never runs twice from one plan.
            self._forget_plan()
            run = {"approach": self._approach, "insert": self._insert, "retreat": self._retreat}[planned.step]
            self._start(ctx, planned.step, run(ctx, robot, planned))
        else:
            demo = self._demo(ctx)
            if isinstance(demo, str):
                self._refuse(ctx, clicked, demo)
                return
            self._forget_plan()
            plan = {"approach": self._plan_approach, "insert": self._plan_insert,
                    "retreat": self._plan_retreat}[MOVES[clicked]]
            self._start(ctx, f"plan {MOVES[clicked]}", plan(ctx, robot, demo))

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

    # --- --- --- --- --- THE PREVIEW --- --- --- --- ---

    def _show(self, ctx: PluginContext, planned: Planned) -> str:
        """Keep a planned move for Execute and time its preview; the panel's line, or why it can't be shown."""
        timed = timed_preview(planned.rows, float(self._seconds.value), planned.limits)
        if isinstance(timed, str):
            return f"{planned.step}: {timed}"
        self._planned = planned
        self._preview, self.minimum = timed
        # * At the end, where the move arrives, as the Assembly tab's preview opens; Play runs it from the start.
        self._t, self._playing = self._preview.duration, False
        return (f"{planned.step} planned ({planned.text}): the motion takes {self._preview.duration:.1f} s; "
                f"Execute runs it")

    def _forget_plan(self) -> None:
        """Drop the planned move and its preview."""
        self._planned = self._preview = None
        self.minimum = math.nan
        self._t, self._playing = 0.0, False

    def _retime(self, seconds: float) -> None:
        """The motion time changed: time the preview again."""
        if self._planned is None:
            return
        timed = timed_preview(self._planned.rows, seconds, self._planned.limits)
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
        """Advance a playing preview; drop a planned move the process or the action no longer matches."""
        now = ctx.now()
        elapsed = 0.0 if self._last_tick is None else now - self._last_tick
        self._last_tick = now
        planned = self._planned
        if planned is None:
            return
        if self._loaded_now(ctx) is not planned.demo.loaded or str(self._action.value) != planned.demo.action_id:
            self.outcome = f"the planned {planned.step} dropped: the process or the action changed"
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
        return Demo(loaded, action_id, transfer, insert)

    def _select(self, ctx: PluginContext, demo: Demo, index: int) -> None:
        """Select a step in the process plugin, so its bodies are in the scene and drawn."""
        movement = demo.loaded.steps[index].movement
        authored = next((k for k, (_action, each) in enumerate(demo.loaded.process.movements()) if each is movement),
                        None)
        if authored is not None:
            self._process(ctx).request_select(authored)

    async def _plan(self, ctx: PluginContext, demo: Demo, index: int, label: str) -> RouteResult | str:
        """Plan a step from the live joints on the motion planner; the route, or why there is none."""
        request = MotionPlanRequest(demo.loaded, demo.loaded.steps[index], ctx.scene.snapshot, ctx.config.robots,
                                    transfer_as_free=True, obstacles_collide=self._process(ctx).obstacles_collide,
                                    label=label)
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

    def _record(self, ctx: PluginContext, robot: HuskyRobotInterface, label: str) -> Recorder:
        """Open the Exec plots for a compliant part starting now; its samples go into the recorder returned."""
        self._recorder, self._plotted_t = Recorder(robot, ctx.now()), None
        self._exec_plots.start_compliant(list(robot.arms), f"{label} on {self._serial}")
        return self._recorder

    # --- --- --- --- --- THE STEPS (tasks) --- --- --- --- ---

    async def _grip(self, ctx: PluginContext, robot: HuskyRobotInterface, close: bool) -> str:
        """Close or open both gripper screws."""
        with ctx.lease_robots([self._serial], "grasp" if close else "release"):
            _ok, text = await run_grip(ctx, self._tools(robot), close)
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

    def _planned_route(self, ctx: PluginContext, step: str, demo: Demo, route: RouteResult) -> str:
        """Show a planner's route as the planned `step`."""
        trajectory = route.result.trajectory
        return self._show(ctx, Planned(step, demo, tuple(trajectory.joint_names),
                                       np.array(trajectory.positions, dtype=float),
                                       ctx.kinematics.base_pose(self._serial),
                                       f"{route.planner}, {route.waypoints} waypoints"))

    async def _plan_approach(self, ctx: PluginContext, robot: HuskyRobotInterface, demo: Demo) -> str:
        """Plan a free dual-arm move from where the arms are to the insert's start, the held bar hidden."""
        self._select(ctx, demo, demo.transfer)
        route = await self._plan(ctx, demo, demo.transfer, "demo approach")
        return route if isinstance(route, str) else self._planned_route(ctx, "approach", demo, route)

    async def _plan_insert(self, ctx: PluginContext, robot: HuskyRobotInterface, demo: Demo) -> str:
        """Plan the process's insert from the live joints; refused unless the arms stand at its start."""
        start = self._insert_start(demo)
        if isinstance(start, str):
            return start
        arms, joints = start
        needs = self._insert_tools(ctx, robot, demo)
        if isinstance(needs, str):
            return needs
        off = off_start(robot, joints, arms)
        if off:
            return f"insert: {off}: press Approach first"
        self._select(ctx, demo, demo.insert)
        route = await self._plan(ctx, demo, demo.insert, "demo insert")
        return route if isinstance(route, str) else self._planned_route(ctx, "insert", demo, route)

    async def _plan_retreat(self, ctx: PluginContext, robot: HuskyRobotInterface, demo: Demo) -> str:
        """Show the back-out: from where the arms are straight to the insert's start. No planner: a tool0 line."""
        start = self._insert_start(demo)
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
                                       f"back-out to the insert's start", limits=False))

    async def _approach(self, ctx: PluginContext, robot: HuskyRobotInterface, planned: Planned) -> str:
        """Run the planned approach from the live joints."""
        with ctx.lease_robots([self._serial], "approach"):
            path = live_rows(robot, planned.names, planned.rows)
            if isinstance(path, str):
                return f"approach: {path}"
            try:
                command = motion_command(self._serial, list(robot.arms), *path, float(self._seconds.value))
            except ValueError as error:
                return f"approach: the plan can't be sent: {error}"
            ctx.log_info(f"insertion demo: approach along {planned.text}, {command.duration:.1f} s")
            sent = await send_motion(ctx, robot, command)
        return f"approach {sent.status}: {sent.reason}"

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
                               spiral=None, traj_time=float(self._seconds.value))
            if isinstance(plan, str):
                return f"insert: {plan}"
            ctx.log_info(f"insertion demo: {plan.text}")
            try:
                outcome = await run_insert(ctx, robot, plan, tools, operator=NoQuestions(ctx), dry_run=False,
                                           recorder=self._record(ctx, robot, f"{planned.demo.action_id} insert"))
            finally:
                self._recorder = None
        return f"insert {outcome.status}: {outcome.reason}"

    async def _retreat(self, ctx: PluginContext, robot: HuskyRobotInterface, planned: Planned) -> str:
        """Loosen the joint screws, then back the tools out to the insert's start, the bar still held."""
        start = self._insert_start(planned.demo)
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
                    seconds=float(self._seconds.value),
                    recorder=self._record(ctx, robot, f"{planned.demo.action_id} retreat"))
            finally:
                self._recorder = None
        return f"retreat {outcome.status}: {outcome.reason}"

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
        robot = ctx.world.robots.get(self._serial) if self._serial else None
        self._status.content = chip_line(self._chips(ctx, robot, actions))
        self._text.content = note(self.outcome) if self.outcome else ""
        self._slider.max = max(self._preview.duration if self._preview is not None else 0.0, SLIDER_STEP)
        self._slider.value = round(self._t, 3)
        minimum = "—" if math.isnan(self.minimum) else f"{self.minimum:.1f}"
        self._minimum.content = block(values(f"min {minimum} s by the arms' limits"))
        if self._ghost is not None:
            if self._planned is None:
                self._ghost.hide()
            else:
                path = dict(zip(self._planned.names, self._preview.sample(self._t)))
                self._ghost.show(self._planned.base, {**ctx.kinematics.joints(self._serial), **path})
        self._draw_plots()

    def _draw_plots(self) -> None:
        """The planned move's joints into the Plan plots, the running compliant part's newest sample into the Exec's."""
        if self._preview is not self._plotted:
            # * A new plan, a retime after a motion-time change, or none (Execute, Cancel).
            planned = self._planned
            if planned is not None and planned is not self._plotted_plan:
                # * A new plan brings the window forward, as in the Assembly tab; only the operator closes it.
                self._plan_window.expand()
            self._plotted, self._plotted_plan = self._preview, planned
            self._plan_plots.show(self._preview, None if planned is None else Trajectory(
                self._serial, planned.names, tuple(map(tuple, planned.rows.tolist()))))
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
            chips += chip("no plan", NONE, "Approach, Insert or Retreat plans a move and shows it; Execute runs it")
        elif planned.limits and robot is not None and isinstance(live_rows(robot, planned.names, planned.rows), str):
            chips += chip(f"{planned.step} stale", BUSY, "the arms moved since planning: press the move again")
        else:
            chips += chip(f"{planned.step} ready", OK, "Execute runs the move the preview shows")
        chips += chip(self.step or "idle", BUSY if self.step else NONE, "the running step")
        return chips
