"""
The Swiss Robotics Day demo: one bar into its joint and out again, on demand, the robot standing still, no mocap.

The tab **Insertion demo**: **Grasp** / **Release** close or open both gripper screws; **Approach** plans a free
dual-arm move from wherever the arms are to the insert's start (the transfer workaround, the bar hidden); **Insert**
runs the process's insert from there (rigid, then compliant while the joint screws tighten until they stall);
**Retreat** loosens the joint screws for **Loosen s** and backs the tools out to the insert's start, the bar still held
(`parts.run_unscrew_retreat`); **Compliant** puts both arms under the compliance controller until **Rigid**. Every
motion is planned on `motion_planner` from the live joints.

- Needs the process loaded in `process` (Load in the Assembly tab) and the base live: without mocap, `husky_fixed_base`
  publishes it standing at the origin, where the process puts it.
- ! One task at a time, the robot leased for its whole run. Cancel or Stop all ends it: a compliant part then only
  holds, the arms stay compliant, and Rigid brings them back.

Run with:  plugins:="insertion_demo robot_control"
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Awaitable

import viser

from scaffolding_assembly_core.ids import robot_id

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
from ...shared.execution.controllers import leave_compliance
from ...shared.execution.insert import plan_insert, run_insert
from ...shared.execution.send import motion_command, send_motion
from ...shared.execution.tools import status_problem
from ...shared.motion_planner_contract import MotionPlanRequest, MotionPlannerContract
from ...shared.movement_exec_contract import INSERT_PART, part_plan, serial_of
from ...shared.process_contract import ProcessContract
from ...ui.style import BUSY, FAIL, INFO, NONE, OK, SECTION_CTRL, chip, chip_line, note, section
from .parts import demo_steps, live_rows, off_start, run_compliant_hold, run_grip, run_unscrew_retreat, tool0_at

#: The sliders' ranges and starts: the insert's compliant millimetres, the seconds of a motion, the loosen.
SPLIT_RANGE, SPLIT_DEFAULT = (0.5, 30.0), 10.0
SECONDS_RANGE, SECONDS_DEFAULT = (2.0, 60.0), 10.0
LOOSEN_RANGE, LOOSEN_DEFAULT = (1.0, 20.0), 6.0
NO_ACTION = "no process"
SCAFFOLDING = "scaffolding_v3"


@dataclass(frozen=True)
class Demo:
    """The loaded process's steps the demo runs: the action's transfer (index into `loaded.steps`) and insert."""

    loaded: LoadedProcess
    action_id: str
    transfer: int
    insert: int


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
    """The demo's tab: Grasp / Release, Approach / Insert / Retreat / Cancel, Compliant / Rigid, three sliders."""

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

    @property
    def running(self) -> bool:
        """bool: Whether a task runs right now."""
        return self._task is not None and not self._task.done()

    # --- --- --- --- --- SETUP --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Find the robot, build the status line, the action list, the sliders and the buttons."""
        config = next((robot for robot in ctx.config.robots
                       if sum(arm.end_effector == SCAFFOLDING for arm in robot.arms) >= 2), None)
        self._serial = config.serial if config is not None else ""
        if not self._serial:
            ctx.log_warn("insertion demo: no configured robot carries two scaffolding tools; every button refuses")
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
            self._seconds = gui.add_slider("Seconds", min=SECONDS_RANGE[0], max=SECONDS_RANGE[1], step=1.0,
                                           initial_value=SECONDS_DEFAULT,
                                           hint="Seconds the approach, the insert and the back-out each take (the "
                                                "arms' limits may need longer)")
            self._loosen = gui.add_slider("Loosen s", min=LOOSEN_RANGE[0], max=LOOSEN_RANGE[1], step=0.5,
                                          initial_value=LOOSEN_DEFAULT,
                                          hint="Seconds the joint screws run in reverse before the tools back out")
            bar = gui.add_button_group("Bar", ["Grasp", "Release"],
                                       hint="Close or open both gripper screws until they stall")
            move = gui.add_button_group("Move", ["Approach", "Insert", "Retreat", "Cancel"],
                                        hint="Approach: a free move to the insert's start from wherever the arms are "
                                             "(the bar in the tools, hidden). Insert: the process's insert from "
                                             "there. Retreat: the joint screws loosen, then the tools back out to "
                                             "the insert's start, the bar still held. Cancel: stop and hold.")
            arms = gui.add_button_group("Arms", ["Compliant", "Rigid"],
                                        hint="Compliant: both arms give way to a push until Rigid. Rigid also "
                                             "brings back arms a cancelled part left compliant.")
            # * Refusals and outcomes come and go here, below the buttons.
            self._text: viser.GuiHtmlHandle = gui.add_html("")
        bar.on_click(ctx.defer_value("bar button", lambda clicked: self._on_button(ctx, clicked)))
        move.on_click(ctx.defer_value("move button", lambda clicked: self._on_button(ctx, clicked)))
        arms.on_click(ctx.defer_value("arms button", lambda clicked: self._on_button(ctx, clicked)))

    # --- --- --- --- --- BUTTONS (intents, on the main thread) --- --- --- --- ---

    def _on_button(self, ctx: PluginContext, clicked: str) -> None:
        """Start the step a button asks for, or say why not."""
        if clicked == "Cancel":
            if self.running:
                self._task.cancel()
                self._planner(ctx).request_cancel()
            else:
                ctx.log_info("Cancel ignored: nothing runs")
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
        else:
            demo = self._demo(ctx)
            if isinstance(demo, str):
                self._refuse(ctx, clicked, demo)
                return
            work = {"Approach": self._approach, "Insert": self._insert, "Retreat": self._retreat}[clicked]
            self._start(ctx, clicked.lower(), work(ctx, robot, demo))

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

    # --- --- --- --- --- THE STEPS (tasks) --- --- --- --- ---

    async def _grip(self, ctx: PluginContext, robot: HuskyRobotInterface, close: bool) -> str:
        """Close or open both gripper screws."""
        with ctx.lease_robots([self._serial], "grasp" if close else "release"):
            _ok, text = await run_grip(ctx, self._tools(robot), close)
        return text

    async def _approach(self, ctx: PluginContext, robot: HuskyRobotInterface, demo: Demo) -> str:
        """A free dual-arm move from where the arms are to the insert's start, the held bar hidden."""
        with ctx.lease_robots([self._serial], "approach"):
            self._select(ctx, demo, demo.transfer)
            route = await self._plan(ctx, demo, demo.transfer, "demo approach")
            if isinstance(route, str):
                return route
            trajectory = route.result.trajectory
            path = live_rows(robot, trajectory.joint_names, trajectory.positions)
            if isinstance(path, str):
                return f"approach: {path}"
            try:
                command = motion_command(self._serial, list(robot.arms), *path, float(self._seconds.value))
            except ValueError as error:
                return f"approach: the plan can't be sent: {error}"
            ctx.log_info(f"insertion demo: approach along {route.planner}, {route.waypoints} rows, "
                         f"{command.duration:.1f} s")
            sent = await send_motion(ctx, robot, command)
        return f"approach {sent.status}: {sent.reason}"

    async def _insert(self, ctx: PluginContext, robot: HuskyRobotInterface, demo: Demo) -> str:
        """The process's insert from the live joints: rigid, then compliant while the joint screws tighten."""
        start = self._insert_start(demo)
        if isinstance(start, str):
            return start
        arms, joints = start
        step = demo.loaded.steps[demo.insert]
        part = part_plan(demo.loaded, demo.action_id, step.movement_index, ctx.config.robots, transfer_as_free=True)
        if part.kind != INSERT_PART or not part.tools:
            return f"{part.movement}: {part.text}"
        tools = self._tools(robot)
        missing = [arm for arm in part.tools if arm not in tools]
        if missing:
            return f"{self._serial} has no scaffolding tool on {', '.join(missing)}"
        with ctx.lease_robots([self._serial], "insert"):
            off = off_start(robot, joints, arms)
            if off:
                return f"insert: {off}: press Approach first"
            self._select(ctx, demo, demo.insert)
            route = await self._plan(ctx, demo, demo.insert, "demo insert")
            if isinstance(route, str):
                return route
            trajectory = route.result.trajectory
            path = live_rows(robot, trajectory.joint_names, trajectory.positions)
            if isinstance(path, str):
                return f"insert: {path}"
            plan = plan_insert(robot, *path, mating=part.tools, grounded=False, split_mm=float(self._split.value),
                               spiral=None, traj_time=float(self._seconds.value))
            if isinstance(plan, str):
                return f"insert: {plan}"
            ctx.log_info(f"insertion demo: {plan.text}")
            outcome = await run_insert(ctx, robot, plan, tools, operator=NoQuestions(ctx), dry_run=False)
        return f"insert {outcome.status}: {outcome.reason}"

    async def _retreat(self, ctx: PluginContext, robot: HuskyRobotInterface, demo: Demo) -> str:
        """Loosen the joint screws, then back the tools out to the insert's start, the bar still held."""
        start = self._insert_start(demo)
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
            self._select(ctx, demo, demo.insert)
            outcome = await run_unscrew_retreat(ctx, robot, {arm: tools[arm] for arm in arms}, ends,
                                                loosen_s=float(self._loosen.value),
                                                seconds=float(self._seconds.value))
        return f"retreat {outcome.status}: {outcome.reason}"

    async def _compliant(self, ctx: PluginContext, robot: HuskyRobotInterface) -> str:
        """Both arms under the compliance controller until Rigid."""
        self._rigid_asked, self._holding = False, True
        try:
            with ctx.lease_robots([self._serial], "compliant"):
                failed = await run_compliant_hold(ctx, list(robot.arms.values()), lambda: self._rigid_asked,
                                                  label=f"{self._serial}: compliant")
        finally:
            self._holding = False
        return failed or "both arms back under joint control"

    async def _rigid(self, ctx: PluginContext, robot: HuskyRobotInterface) -> str:
        """Bring arms left compliant (a cancelled part) back under joint control."""
        with ctx.lease_robots([self._serial], "rigid"):
            back = await leave_compliance(ctx, list(robot.arms.values()), label=f"{self._serial}: rigid")
        return "both arms under joint control" if back else "not every arm is confirmed back: the log says"

    # --- --- --- --- --- DRAW --- --- --- --- ---

    def draw(self, ctx: PluginContext) -> None:
        """The status chips, the action list and the last outcome."""
        loaded = self._loaded_now(ctx)
        actions = self._actions(ctx, loaded) if loaded is not None else []
        options = actions or [NO_ACTION]
        if list(self._action.options) != options:
            self._action.options = options
            self._action.value = options[-1]
        robot = ctx.world.robots.get(self._serial) if self._serial else None
        self._status.content = chip_line(self._chips(ctx, robot, actions))
        self._text.content = note(self.outcome) if self.outcome else ""

    def _chips(self, ctx: PluginContext, robot: HuskyRobotInterface | None, actions: list[str]) -> str:
        """The robot live or not, the action, the tools, the arms' controller, the running step."""
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
        chips += chip(self.step or "idle", BUSY if self.step else NONE, "the running step")
        return chips
