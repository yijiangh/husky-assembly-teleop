"""
Base experiment: drive test paths from where the robot stands with the onboard pure pursuit follower, and record them.

- Start: pick a template; the preview starts at the robot and follows it until Start, which sends it (geometric or
  timed) and waits for the follower. Stop, a soft stop or the plugin stopping ends a run.
- Start auto, by Automation:
  - Controller: standard paths, the same paths in every scenario (resumed from the experiment's earlier runs with
    the same setup), or random paths from the grid; geometric at one fixed speed (`auto.py`).
  - Robot: constant commands, one cell (v, ω) of the command grid per run, sent from here open loop (`twist.py`);
    for how the robot's response changes with speed and turn rate.
  Each run is checked against the other robots and every scene body (`obstacles` loads with it: the lab's border).
- Clearance: during every run the robot is soft-stopped, and automation stopped, once it comes within STOP_DISTANCE
  of anything in the scene (`ctx.clearance`); the operator resumes it, drives it clear and starts again.
- With the robot's arms not measured, Start and Start auto ask for a second press; the runs then check the arms where
  last seen, else stowed.
- A way back to the start that is not clear stops automation; the collisions panel shows what it comes too close to.
- Errors are measured here, the same for every controller (`tracking.py`): each new mocap pose against the path
  sent. After the follower ends, recording goes on until the robot stands still (SETTLE), for the error at the end.
- Each run is saved to a folder of its own under the experiment's name (`record.py`), with experiment.json and
  recording.npz; `report.py` adds report.json and overview.png.
- ! Needs the Drive root (HUSKY_DRIVE_ROOT, `drive.py`) with record.RECORDING_FOLDER in it; setup fails without.

! The follower (`crl_husky pure_pursuit`) runs on the robot, or with `pure_pursuit_sim.launch.py` for a test.
"""

from __future__ import annotations

import asyncio
import dataclasses
import math
import random
from collections import deque
from html import escape
from pathlib import Path
from typing import Any, Coroutine

import numpy as np
import viser
from crl_husky.follower_path import FollowerPath, wrap

from ...drive import drive_folder, folder_name
from ...plugin_api import signals
from ...plugin_api.concurrency import WaitTimeout
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...plugin_api.trace import Signal, Trace
from ...robot_interface.base import PLATFORM_VELOCITY_CONTROLLER, FollowerState, live_report
from ...robot_interface.mocap_state import pose_lost
from ...shared.collisions_contract import CollisionsContract
from ...shared.obstacles_contract import ObstaclesContract
from ...shared.planner.planner_panel import COMMIT_CONFIRM_SECONDS
from ...shared.planner.timed_path import timing
from ...ui.base_markers import Markers
from ...ui.pose_input import yaw_from_xyzw
from ...ui.style import (BUSY, FAIL, NONE, OK, SECTION_CTRL, SECTION_TOOL, block, chip, chip_line, hover, section,
                         values, warning)
from ...ui.trace_plot import TracePlot
from ...world.clearance import ALL_ARMS, STOP_DISTANCE, Clearance, ClearanceRefused
from ...world.checks import STALE_AFTER
from ...world.collision_hit import CollisionHit
from ...world.stand_ins import unknown_parts, unknown_robots
from scaffolding_assembly_core.ids import robot_id
from . import record
from .auto import PATH_CLEARANCE, SPEED, STANDARD_SET, PathCollisionCheck, Sampler, StandardSet
from .plan import Plan, plan_from
from .report import write_report
from .templates import PARAMETERS, TEMPLATES
from .tracking import Sample, Tracker
from .twist import TWIST, TwistGrid, grid, twist_plan

#: Samples the live plots keep: 60 s at 20 Hz.
LIVE_SAMPLES = 1200
#: Smallest y span of each live plot (m, °, m/s, °/s), so a run with no error keeps a readable axis.
LIVE_MIN_SPANS = (0.02, 2.0, 0.05, 5.0)
#: Seconds to wait for the follower to take a path.
ACK_TIMEOUT = 2.0
#: A run times out after its expected duration times this, plus RUN_MARGIN seconds.
RUN_FACTOR, RUN_MARGIN = 2.0, 10.0
#: Grid cells tried per automated run before giving up on finding a clear path, and per template before the next.
SAMPLE_TRIES, CELLS_PER_TEMPLATE = 50, 10
#: Seconds between automated runs.
AUTO_PAUSE = 2.0
#: What automated runs drive: the standard paths (`auto.STANDARD_SET`), random paths (`auto.GRID`), or constant
#: commands (`twist.grid`).
STANDARD, RANDOM, TWIST_GRID = "Controller: standard paths", "Controller: random paths", "Robot: constant commands"
#: Auto runs each choice sets: one round of the standard paths, ten random paths, one round of the command grid.
AUTO_RUNS = {STANDARD: len(STANDARD_SET), RANDOM: 10, TWIST_GRID: len(grid())}
#: Constant-command runs: seconds standing still with zero commands before the step, so the step starts from rest; and
#: how far the robot may wander from the series' start before it drives back, metres (each run is checked from where
#: the robot stands, so it need not start at the same pose).
TWIST_LEAD, TWIST_HOME_RADIUS = 0.5, 0.75
#: A drive back that times out still counts when it ends this close to the start pose, metres (it is for open space,
#: not precision): the follower may circle the goal on a robot with a turning-centre offset.
HOME_NEAR = 0.3
#: Follower abort reasons that are no controller failure but a problem around it: automation pauses on them.
#: Every other abort, and a run that does not finish in time, is a controller failure: recorded, and the series goes on.
NOT_THE_CONTROLLER = ("mocap lost", "bad path")
#: Automation drives back to where the series started when no clear path is found, or before every run with
#: "Return every run" unless within HOME_TOLERANCE of it (metres, radians); at RETURN_SPEED (m/s, rad/s), not recorded.
HOME_TOLERANCE = (0.05, math.radians(5.0))
RETURN_SPEED = (0.2, math.radians(30.0))
#: Clearance the way back must keep, metres; less than PATH_CLEARANCE, so a robot that ended a run near a wall can
#: still get away from it.
RETURN_CLEARANCE = 0.2
#: Settling: the robot counts as standing once it stayed within SETTLE_POSITION and SETTLE_YAW for SETTLE_FOR
#: seconds; recording gives up waiting after SETTLE_TIMEOUT.
SETTLE_POSITION, SETTLE_YAW, SETTLE_FOR, SETTLE_TIMEOUT = 0.01, math.radians(1.0), 0.5, 3.0
#: The outcome of a run the platform's e-stop interrupted: kept on disk, left out of the analysis, automation pauses.
E_STOPPED = "e-stopped"
#: The follower chip's colour per state; idle is gray.
FOLLOWER_COLORS = {"following": BUSY, "done": OK, "aborted": FAIL}
#: Phases of a run, recorded with each sample.
SENT, FOLLOWING, SETTLING = 0, 1, 2
#: The preview moves with the robot once it is this far from where it was drawn.
PREVIEW_POSITION, PREVIEW_YAW = 0.005, math.radians(0.5)


@register
class BaseExperimentPlugin(HuskyPlugin):
    """Sends template paths to the onboard follower, by hand or at random, shows them and records the runs."""

    name = "base_exp"
    icon = viser.Icon.CHART_LINE
    #: Automation needs the lab's border (`ObstaclesContract.lab`).
    requires = ("obstacles", "collisions")

    def __init__(self):
        """Start with no robot, no preview and no run."""
        self.serial: str | None = None
        self._preview: Plan | None = None
        self._preview_problem = ""
        #: The pose the preview was placed at.
        self._anchor: tuple[float, float, float] | None = None
        #: Whether an input changed since the preview was built.
        self._inputs_changed = True
        #: The run task (one run, or a series of automated runs).
        self._task: asyncio.Task | None = None
        #: The path of the current or last run, and its recording.
        self._sent: Plan | None = None
        self._record: Trace | None = None
        #: How the last run ended, what automation is doing, and why the guard stopped the robot (until next Start).
        self._outcome = ""
        self._auto_note = ""
        #: The automated series: runs done, runs asked for (0: until Stop), its start time; and where its paths stand.
        self._series: tuple[int, int, float] | None = None
        self._paths_note = ""
        #: Why the last series stopped early, shown until the next Start; the way back's hit if that stopped it, and
        #: the hit the collisions panel shows.
        self._auto_alert = ""
        self._return_hit: CollisionHit | None = None
        self._reported: CollisionHit | None = None
        self._alert = ""
        self._seed = random.randrange(2 ** 31)
        self._rng = random.Random(self._seed)
        self._path_check = PathCollisionCheck()
        #: Start's and Start auto's second press while the robot's arms are not measured: button -> ROS time it waits
        #: until. Once given, the run(s) check those arms where last seen, else stowed.
        self._confirm: dict[str, float] = {}
        self._unknown_arms = False
        #: The run's error measurement, its latest sample and phase, and the time of the mocap fix measured last.
        self._tracker: Tracker | None = None
        self._sample: Sample | None = None
        self._phase = SENT
        self._measured_fix: float | None = None
        self._sent_at = 0.0

    @property
    def running(self) -> bool:
        """bool: Whether a run or a series of runs is going on."""
        return self._task is not None and not self._task.done()

    # --- --- --- --- --- SETUP --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Build the panel, the live plots and the scene markers.

        Raises:
            FileNotFoundError: If the recordings folder on the Drive is missing (see `drive_folder`).
        """
        self._recordings = drive_folder(ctx.config.drive_root, record.RECORDING_FOLDER)
        serials = [robot.serial for robot in ctx.config.robots]
        with ctx.view.ui() as gui:
            self._status = gui.add_html("")
            if not serials:
                self._status.content = block(chip("no robots configured", NONE,
                                                  "The monitor's robots parameter names none: nothing to drive."))
                return
            self.serial = serials[0]
            self._robot = gui.add_dropdown("Robot", options=serials, initial_value=self.serial)

            gui.add_html(section("path", SECTION_TOOL))
            self._template = gui.add_dropdown("Template", options=list(TEMPLATES), initial_value="Straight")
            self._mode = gui.add_dropdown("Mode", options=["Geometric", "Timed"], initial_value="Geometric",
                                          hint="Geometric: the follower picks the speed, up to the limits. "
                                               "Timed: it tracks the pose due at each time")
            self._linear = gui.add_slider("Speed m/s", 0.05, 0.5, 0.05, 0.2,
                                          hint="Geometric: speed cap. Timed: the path is timed at this speed, "
                                               "and the follower may go faster, up to its own limit, to catch up")
            self._angular = gui.add_slider("Turn °/s", 5.0, 60.0, 5.0, 30.0, hint="As Speed, for turning")

            gui.add_html(section("run", SECTION_CTRL))
            run = gui.add_button_group("Run", ["Start", "Stop"],
                                       hint="Start: send the preview to the follower. Stop: stop the run(s)")
            self._recording = gui.add_checkbox("Record", initial_value=True,
                                               hint="Save each run, with a plot, to the Drive under <experiment>.")
            auto = gui.add_button_group("Auto", ["Start auto"],
                                        hint=f"Run experiments from where the robot stands at {SPEED[0]} m/s and "
                                             f"{SPEED[1]:.0f}°/s, each checked to keep {PATH_CLEARANCE:.2f} m from "
                                             f"everything. Stop ends them")
            self._auto_paths = gui.add_dropdown(
                "Automation", options=[STANDARD, RANDOM, TWIST_GRID], initial_value=STANDARD,
                hint="Standard: the same 10 paths, least tried first (resumes). Random: any grid path. Constant "
                     "commands: open loop, every grid cell (resumes).")
            self._auto_runs = gui.add_number("Auto runs", AUTO_RUNS[STANDARD], min=0, step=1,
                                             hint="Runs in total (standard, constant: Start resumes) or per Start "
                                                  "(random). 0: until Stop.")
            self._return_always = gui.add_checkbox(
                "Return every run", initial_value=False,
                hint="Drive back to the series' start pose before every automated run; off: only when no clear "
                     "path is found where the robot stands")
            # * Progress of the automated series (runs done of runs asked for), else of the current run.
            self._progress = gui.add_progress_bar(0.0, color="blue")
            self._experiment = gui.add_text("Experiment", initial_value="default",
                                            hint="Name of this data collection: runs are saved under it")
            # ! Template numbers BELOW the buttons; the ones the template does not use are disabled.
            self._numbers = {p.name: gui.add_number(p.label, p.default, step=p.step, hint=p.hint)
                             for p in PARAMETERS}
            self._details = gui.add_html("")

            # * The live plots in a tab, in a folder closed at start; last, so opening it moves no button.
            live = self._live_signals(ctx)
            self._live = ctx.trace(*live, max_samples=LIVE_SAMPLES)
            with gui.add_folder("Live plots", expand_by_default=False), \
                    gui.add_tab_group().add_tab("Errors and commands", viser.Icon.CHART_LINE):
                self._plots = [TracePlot(gui, self._live, signal, seconds=None, min_span=span)
                               for signal, span in zip(live, LIVE_MIN_SPANS)]
        self._markers = Markers(ctx)

        self._robot.on_update(ctx.defer_value("choose robot", lambda serial: self._choose_robot(str(serial))))
        for widget in (self._template, self._mode, self._linear, self._angular, *self._numbers.values()):
            widget.on_update(ctx.defer("edit path", self._edited))
        run.on_click(ctx.defer_value("run", lambda label: self._on_run(ctx, label)))
        auto.on_click(ctx.defer("auto", lambda: self._on_auto(ctx)))
        self._auto_paths.on_update(ctx.defer_value("choose automation", self._choose_automation))
        self._enable_numbers()

    def _choose_automation(self, paths: str) -> None:
        """Set Auto runs to one round of the chosen automation."""
        self._auto_runs.value = AUTO_RUNS[str(paths)]

    @staticmethod
    def _others(ctx: PluginContext, plan: Plan) -> frozenset[str]:
        """The other robots not measured as a run starts: its clearance may take them as stand-ins, no robot else."""
        return unknown_robots(ctx.scene.snapshot, {robot_id(plan.serial)})

    def _tracking_signals(self) -> list[Signal]:
        """The monitor's tracking errors of the current run, and their context; unknown outside a run.

        The context: turning (1 on a turn on the spot, else 0), path curvature, progress (0 to 1), path speed and turn
        rate, phase (0 sent, 1 following, 2 settling).
        """
        def read(pick):
            return lambda: None if self._sample is None else pick(self._sample)

        return [Signal("tracking_position", read(lambda s: (s.position, s.along)), ("position", "along"), "m"),
                Signal("tracking_heading", read(lambda s: s.heading), unit="rad"),
                Signal("tracking_context",
                       read(lambda s: (s.turning, s.curvature, s.progress, s.speed, s.turn_rate, self._phase)),
                       ("turning", "curvature", "progress", "speed", "turn_rate", "phase"),
                       ("", "1/m", "", "m/s", "rad/s", ""))]

    def _live_signals(self, ctx: PluginContext) -> list:
        """The run's tracking errors and the chosen robot's follower commands, under fixed names for the plots."""
        def of_chosen(make, name):
            return dataclasses.replace(make(ctx.world.robots[self.serial]), name=name,
                                       read=lambda: make(ctx.world.robots[self.serial]).read())

        def degrees(signal, unit="°"):
            return dataclasses.replace(signal, unit=unit, read=lambda: _degrees(signal.read()))

        def part(signal, index, name, unit):
            return dataclasses.replace(signal, name=name, labels=(), unit=unit,
                                       read=lambda: _pick(signal.read(), index))

        position, heading, _ = self._tracking_signals()
        command = of_chosen(signals.follower_command, "command")
        return [dataclasses.replace(position, name="position_error"),
                degrees(dataclasses.replace(heading, name="heading_error")),
                part(command, 0, "linear_velocity", "m/s"),
                degrees(part(command, 1, "angular_velocity", ""), "°/s")]

    def teardown(self, ctx: PluginContext) -> None:
        """Close the clearance checker's mirror and worker."""
        self._path_check.close()

    # --- --- --- --- --- INTENTS --- --- --- --- ---

    def _choose_robot(self, serial: str) -> None:
        """Preview for another robot; refused during a run."""
        if self.running:
            self._outcome = "robot not changed: a run is going on"
            return
        self.serial = serial
        self._inputs_changed = True
        self._live.clear()

    def _edited(self) -> None:
        """An input changed: rebuild the preview."""
        self._inputs_changed = True
        self._enable_numbers()

    def _on_run(self, ctx: PluginContext, label: str) -> None:
        """Start a run of the preview, or stop the run(s) or the follower."""
        if label == "Stop":
            if self.running:
                self._task.cancel()
            else:
                ctx.world.robots[self.serial].base.stop_path()
                self._outcome = "stop sent"
            return
        problem = self._start_problem(ctx) or ("" if self._preview else self._preview_problem or "no path")
        if problem:
            self._outcome = f"not started: {problem}"
            return
        if self._asks_second_press(ctx, "Start"):
            return
        self._alert, self._auto_note, self._auto_alert, self._return_hit = "", "", "", None
        self._task = ctx.spawn(f"base experiment {self.serial}",
                               self._leased(ctx, self.serial, self._run(ctx, self._preview)))

    def _on_auto(self, ctx: PluginContext) -> None:
        """Start automated runs, unless something keeps them from starting."""
        problem = self._start_problem(ctx)
        obstacles: ObstaclesContract = ctx.require("obstacles")
        if not problem and obstacles.lab.border is None:
            problem = "no lab border: set one in the obstacles plugin"
        if problem:
            self._auto_note = f"auto not started: {problem}"
            return
        if self._asks_second_press(ctx, "Start auto"):
            return
        self._alert, self._auto_alert = "", ""
        self._task = ctx.spawn(f"base experiment auto {self.serial}", self._leased(
            ctx, self.serial, self._auto(ctx, self.serial, int(self._auto_runs.value), self._auto_paths.value)))

    def _start_problem(self, ctx: PluginContext) -> str:
        """Why no run can start now, or ""."""
        base = ctx.world.robots[self.serial].base
        if self.running:
            return "a run is going on"
        lease = ctx.world.robots[self.serial].lease_slot.lease
        if lease is not None:
            return f"{self.serial} is leased by {lease.holder}"
        problem = base.follow_problem()
        return f"{self.serial}: {problem}" if problem else ""

    def _asks_second_press(self, ctx: PluginContext, button: str) -> bool:
        """Whether the press only asked: with the robot's arms not measured, the first press says to press it again.

        ! Call it last before starting, after every other refusal, so the second press is never asked for in vain.
        """
        robot = ctx.scene.snapshot.robots.get(robot_id(self.serial))
        unknown = robot is not None and bool(robot.unmeasured)
        if unknown and ctx.now() > self._confirm.pop(button, -math.inf):
            self._confirm[button] = ctx.now() + COMMIT_CONFIRM_SECONDS
            self._auto_alert = (f"Press {button} again within {COMMIT_CONFIRM_SECONDS:g} s to drive with "
                                f"{unknown_parts(robot)} not measured: checked where last seen, else stowed")
            return True
        self._unknown_arms = unknown
        return False

    # --- --- --- --- --- RUNS (tasks) --- --- --- --- ---

    async def _leased(self, ctx: PluginContext, serial: str, work: Coroutine[Any, Any, None]) -> None:
        """Run `work` holding the robot's lease, so nothing else in this monitor drives it meanwhile."""
        try:
            with ctx.lease_robots([serial], "experiment"):
                await work
        finally:
            work.close()  # never started if the lease was refused; closing it avoids a "never awaited" warning

    async def _run(self, ctx: PluginContext, plan: Plan, automation: dict | None = None,
                   recorded: bool = True) -> None:
        """Send the path, guard it, wait for the follower to finish, and save the recording.

        Args:
            ctx: This plugin's context.
            plan: The path to send.
            automation: Where this run stands in an automated series, for the record; None for a manual run.
            recorded: False for a drive that is no experiment, e.g. back to the start: nothing is saved.
        """
        robot = ctx.world.robots[plan.serial]
        base = robot.base
        self._sent, self._record, self._outcome, finished = plan, None, "reading the setup", False
        self._tracker, self._sample, self._phase, self._measured_fix = Tracker(plan.path), None, SENT, None
        self._live.clear()
        experiment: dict = {}
        path_id, sent_at, clearance, timed_out, estopped, lost = None, math.inf, None, False, False, ""

        def report() -> FollowerState | None:
            follower = base.state.follower
            ok = follower is not None and follower.path_id == path_id and follower.update_time >= sent_at
            return follower if ok else None

        def ended() -> bool:
            nonlocal lost
            # * Without its pose the clearance is blind: stop, though the follower also aborts on its own.
            lost = pose_lost(base.mocap_id, base.state, ctx.now())
            follower = base.state.follower
            return (bool(base.state.estopped) or bool(lost) or follower.path_id != path_id
                    or follower.state != "following" or ctx.now() - follower.update_time > 2 * STALE_AFTER)

        try:
            more = {"experiment_name": folder_name(self._experiment.value),
                    "clearance": {"path": PATH_CLEARANCE, "stop": STOP_DISTANCE}}
            if automation is not None:
                more["automation"] = automation
            if recorded:
                experiment = await record.describe(ctx, base, plan, **more)
            self._outcome = "running"
            # * The follower's own errors are kept as diagnostics; the tracking signals compare controllers. Its
            #   reports and the wheel odometry are kept per message too, from before the send.
            with ctx.record(signals.base_floor_pose(robot), *self._tracking_signals(),
                            signals.follower_position_error(robot), signals.follower_yaw_error(robot),
                            signals.follower_command(robot), signals.follower_progress(robot),
                            *_message_signals(robot, follower=True)) as rec, \
                    ctx.clearance([plan.serial], STOP_DISTANCE, unknown_arms=ALL_ARMS if self._unknown_arms else (),
                                  unknown_robots=self._others(ctx, plan)) as clearance:
                self._record = rec
                path_id = experiment["path_id"] = base.send_path(*plan.poses.T, plan.t, *plan.speed_caps)
                sent_at = self._sent_at = ctx.now()
                await ctx.wait_until(lambda: report() is not None, timeout_s=ACK_TIMEOUT,
                                     description="the follower to take the path")
                self._phase = FOLLOWING
                try:
                    await ctx.wait_until(ended, timeout_s=plan.expected * RUN_FACTOR + RUN_MARGIN,
                                         description="the follower to finish")
                except WaitTimeout:
                    # * Not finishing is the controller's failure: stop it, and record it settling like any run.
                    timed_out, finished = True, True
                    base.stop_path()
                follower, estopped = base.state.follower, bool(base.state.estopped)
                if lost:
                    base.stop_path()
                self._phase = SETTLING
                experiment["settled"] = await self._settle(ctx, plan.serial)
            # * An e-stop is no controller failure: the robot stood while the follower still commanded.
            if estopped:
                self._outcome = E_STOPPED
            elif timed_out:
                self._outcome = "timed out: did not finish"
            elif lost:
                self._outcome = f"aborted: mocap lost ({lost})"
            elif follower.path_id != path_id:
                self._outcome, finished = "aborted: another path replaced it", True
            elif follower.state == "following":
                self._outcome = "aborted: the follower went silent"
            else:
                self._outcome, finished = follower.state + (f": {follower.reason}" if follower.reason else ""), True
        except WaitTimeout as timeout:
            self._outcome = "timed out: no answer from the follower"
            ctx.log_warn(str(timeout))
        except ValueError as error:
            self._outcome = f"not sent: {error}"
        except ClearanceRefused as refused:
            self._outcome = f"not sent: {refused}"
        except asyncio.CancelledError:
            self._outcome = self._stopped(ctx, clearance)
            raise
        finally:
            self._phase = SENT
            if not finished and path_id is not None:
                base.stop_path()
            if recorded and experiment:
                self._save(ctx, plan, experiment)
            ctx.log_info(f"base experiment on {plan.serial}: {self._outcome}")

    async def _run_twist(self, ctx: PluginContext, plan: Plan, automation: dict) -> None:
        """Drive one constant-command run open loop: stand TWIST_LEAD, hold the command for its duration, stop, settle, save.

        The command is sent every tick from here; the clearance runs as in `_run`. On the real robot it needs the
        velocity controller (without it the run ends "not driven" and automation pauses); the simulator has none.
        """
        robot = ctx.world.robots[plan.serial]
        base = robot.base
        v, w = plan.settings["parameters"]["v"], plan.settings["parameters"]["w"]
        self._sent, self._record, self._outcome = plan, None, "reading the setup"
        self._tracker, self._sample, self._phase, self._measured_fix = Tracker(plan.path), None, SENT, None
        self._live.clear()
        experiment: dict = {}
        clearance = None
        try:
            experiment = await record.describe(
                ctx, base, plan, experiment_name=folder_name(self._experiment.value),
                clearance={"path": PATH_CLEARANCE, "stop": STOP_DISTANCE}, automation=automation,
                # ? Sent from the monitor over the network: the delay includes it; its stamps are the monitor's clock.
                command_source="monitor")
            self._outcome = "running"
            self._sent_at = ctx.now()
            with ctx.record(signals.base_floor_pose(robot), *self._tracking_signals(), signals.follower_progress(robot),
                            *_message_signals(robot, follower=False)) as rec, \
                    ctx.clearance([plan.serial], STOP_DISTANCE, unknown_arms=ALL_ARMS if self._unknown_arms else (),
                                  unknown_robots=self._others(ctx, plan)) as clearance:
                self._record = rec
                self._phase = FOLLOWING
                step_at = ctx.now() + TWIST_LEAD
                end = step_at + plan.expected
                while ctx.now() < end and not base.state.estopped:
                    if lost := pose_lost(base.mocap_id, base.state, ctx.now()):
                        # * Open loop, but the clearance needs the pose: stop.
                        self._outcome = f"aborted: mocap lost ({lost})"
                        return
                    command = (v, w) if ctx.now() >= step_at else (0.0, 0.0)
                    if not base.send_twist(*command):
                        # * No controller failure: automation pauses for the operator.
                        self._outcome = f"not driven: {PLATFORM_VELOCITY_CONTROLLER} is not active"
                        return
                    await ctx.next_tick()
                base.send_twist(0.0, 0.0, require_controller=False)
                self._phase = SETTLING
                experiment["settled"] = await self._settle(ctx, plan.serial)
            self._outcome = E_STOPPED if base.state.estopped else "done"
        except ClearanceRefused as refused:
            self._outcome = f"not sent: {refused}"
        except asyncio.CancelledError:
            self._outcome = self._stopped(ctx, clearance)
            raise
        finally:
            base.send_twist(0.0, 0.0, require_controller=False)
            self._phase = SENT
            if experiment:
                self._save(ctx, plan, experiment)
            ctx.log_info(f"base experiment on {plan.serial}: {plan.label}: {self._outcome}")

    def _save(self, ctx: PluginContext, plan: Plan, experiment: dict) -> None:
        """Save the run, if recording is on, and write its report on a worker thread."""
        rec = self._record
        if rec is None or not self._recording.value or len(rec) == 0:
            return
        try:
            saved = record.save(self._recordings, rec, plan, experiment, self._outcome, ctx.config.recording_format)
        except OSError as error:
            ctx.log_error(f"base experiment: recording not saved: {error}")
            return
        self._outcome += f" → {saved.parent.name}"
        try:
            ctx.spawn("report run", self._report(ctx, saved), survives_soft_stop=True)
        except RuntimeError:
            ctx.log_info(f"base experiment: no report while stopping; make it with report.py {saved.parent}")

    async def _report(self, ctx: PluginContext, saved: Path) -> None:
        """Write the run's experiment.json and overview.png beside its recording."""
        try:
            await ctx.run_in_thread(write_report, saved.parent)
        except Exception as error:  # a broken report must not count as a plugin failure
            ctx.log_warn(f"base experiment: report for {saved.parent.name} failed: {error}")
            return
        ctx.log_info(f"base experiment: saved to {saved.parent}")

    async def _auto(self, ctx: PluginContext, serial: str, runs: int, paths: str) -> None:
        """Run experiments until `runs` are done (0: until stopped), or something needs the operator.

        Standard paths and constant commands resume: `runs` is the total for this experiment, setup and automation,
        earlier attempts included, so a Start after a stop runs only the rest.

        The robot drives back to the series' start pose when no clear path is found where it stands, and with "Return
        every run" before every run (unless it stands on it).

        Args:
            ctx: This plugin's context.
            serial: The robot.
            runs: Runs to do; 0 until stopped.
            paths: STANDARD, RANDOM or TWIST_GRID, what the runs drive.
        """
        done, failures, started, before = 0, 0, ctx.now(), 0
        home = self._robot_pose(ctx, serial)
        name = folder_name(self._experiment.value)
        if paths in (STANDARD, TWIST_GRID):
            # * Resumed from this experiment's earlier attempts with the same setup, so every path gets as many.
            key = record.current_key(await record.setup(ctx, ctx.world.robots[serial].base))
            earlier = await ctx.run_in_thread(record.earlier_runs, self._recordings, name, key)
            tried = [e for e in earlier if _attempt(e.get("outcome", "").split(" → ")[0]) and "template" in e]
            attempts = [e["template"] for e in tried]
            chooser = StandardSet(attempts) if paths == STANDARD else TwistGrid(attempts, self._rng)
            before = sum((e.get("automation") or {}).get("paths") == paths for e in tried)
            if 0 < runs <= before:
                self._paths_note = chooser.summary
                self._auto_note = (f"auto: {name} already has {before} of {runs} runs on this setup; raise Auto runs "
                                   f"for more")
                ctx.log_info(f"base experiment {self._auto_note}")
                return
        else:
            # * Balanced within this collection run only.
            chooser = Sampler([], self._rng)
        self._paths_note, self._auto_alert, self._return_hit = chooser.summary, "", None
        resumed = f", resuming after {before}" if before else ""
        ctx.log_info(f"base experiment auto: starting {runs or 'unlimited'} runs{resumed} ({paths.lower()}) on "
                     f"{serial} from {_pose_text(home)}, experiment {name}")
        try:
            while runs <= 0 or before + done < runs:
                self._series = (done, runs - before if runs > 0 else 0, started)
                self._auto_note = f"auto: choosing run {before + done + 1}" + (f" of {runs}" if runs > 0 else "")
                pose = self._robot_pose(ctx, serial)
                back = (_far(pose, home, TWIST_HOME_RADIUS) if paths == TWIST_GRID
                        else self._return_always.value and _away(pose, home))
                if back and not await self._return(ctx, serial, home):
                    return
                plan, tries = await self._sample_clear_plan(ctx, serial, chooser)
                if plan is None and _far(self._robot_pose(ctx, serial), home, 0.1):
                    if not await self._return(ctx, serial, home):
                        return
                    plan, tries = await self._sample_clear_plan(ctx, serial, chooser)
                if plan is None:
                    self._stop_auto(ctx, f"stopped after {done} runs: no clear path in {tries} samples, even from the "
                                         f"start pose; move the robot to open space")
                    return
                count = f"{before + done + 1}" + (f"/{runs}" if runs > 0 else "")
                left = runs - before if runs > 0 else 0
                self._auto_note = f"auto: run {count} · {_timing_text(ctx.now() - started, done, left)}"
                ctx.log_info(f"base experiment auto: run {count}: {plan.label}")
                run_started = ctx.now()
                automation = {"run": before + done + 1, "runs": runs, "paths": paths, "samples": tries,
                              "seed": self._seed}
                if paths == TWIST_GRID:
                    await self._run_twist(ctx, plan, automation)
                else:
                    await self._run(ctx, plan, automation)
                done += 1
                outcome = self._outcome.split(' → ')[0]
                if _attempt(outcome):
                    chooser.count(plan.settings, outcome.startswith("done"))
                    self._paths_note = chooser.summary
                failures += _controller_failure(outcome)
                ctx.log_info(f"base experiment auto: run {count} {outcome} in {ctx.now() - run_started:.0f} s · "
                             f"{_timing_text(ctx.now() - started, done, left)} · {self._paths_note} · "
                             f"controller failures {failures}")
                if not _attempt(outcome):
                    self._stop_auto(ctx, f"paused after {done} runs: {outcome}")
                    return
                self._series = (done, left, started)
                await ctx.sleep(AUTO_PAUSE)
            self._auto_note = (f"auto finished: {done} runs in {_minutes(ctx.now() - started)}, "
                               f"{failures} controller failures")
            ctx.log_info(f"base experiment {self._auto_note}")
        except asyncio.CancelledError:
            stopper = "the clearance" if self._alert else "the operator"
            self._auto_note = f"auto stopped by {stopper} after {done} runs"
            ctx.log_info(f"base experiment {self._auto_note}")
            raise
        finally:
            self._series = None

    async def _return(self, ctx: PluginContext, serial: str, home) -> bool:
        """Drive back to `home` (turn, drive, turn), checked and guarded but not recorded.

        Returns:
            bool: True once there; False if the way is not clear or the drive did not end done (automation stopped).
        """
        pose = self._robot_pose(ctx, serial)
        plan = _return_plan(serial, pose, home)
        hit = await self._path_check.path_hit(ctx.scene.snapshot, serial, plan.path.polyline(), RETURN_CLEARANCE)
        if hit is not None:
            self._return_hit = hit
            self._stop_auto(ctx, f"stopped: the way back to the start passes {hit.name(hit.pairs[0][1])}; move the "
                                 f"robot to open space")
            return False
        self._auto_note = f"auto: driving back to the start ({math.hypot(pose[0] - home[0], pose[1] - home[1]):.1f} m)"
        ctx.log_info(f"base experiment auto: driving back to the start pose {_pose_text(home)}")
        await self._run(ctx, plan, recorded=False)
        near = self._outcome.startswith("timed out: did not finish") and not _far(
            self._robot_pose(ctx, serial), home, HOME_NEAR)
        if not self._outcome.startswith("done") and not near:
            self._stop_auto(ctx, f"stopped: the drive back to the start ended {self._outcome}")
            return False
        await ctx.sleep(AUTO_PAUSE)
        return True

    def _stop_auto(self, ctx: PluginContext, reason: str) -> None:
        """End the series early: say why in the panel (until the next Start) and in the log."""
        self._auto_note = f"auto {reason}"
        self._auto_alert = f"Automation {reason}"
        ctx.log_warn(f"base experiment: {self._auto_alert}")

    async def _sample_clear_plan(self, ctx: PluginContext, serial: str,
                                 chooser: Sampler | StandardSet | TwistGrid) -> tuple[Plan | None, int]:
        """The chooser's first path that, placed where the robot stands, keeps PATH_CLEARANCE from everything."""
        anchor = self._robot_pose(ctx, serial)
        base = ctx.world.robots[serial].base
        if anchor is None or pose_lost(base.mocap_id, base.state, ctx.now()):
            return None, 0
        snapshot = ctx.scene.snapshot
        tries = 0
        for tries, settings in enumerate(chooser.candidates(CELLS_PER_TEMPLATE), start=1):
            if settings["name"] == TWIST:
                plan = twist_plan(serial, settings["parameters"]["v"], settings["parameters"]["w"], anchor)
            else:
                plan, _ = plan_from(serial, settings, anchor)
            if plan is not None and await self._path_check.path_hit(snapshot, serial, plan.path.polyline()) is None:
                return plan, tries
            if tries >= SAMPLE_TRIES:
                break
        return None, tries

    async def _settle(self, ctx: PluginContext, serial: str) -> bool:
        """Wait until the robot stands still, for the error at the end.

        Returns:
            bool: True once it stayed within SETTLE_POSITION and SETTLE_YAW for SETTLE_FOR seconds; False if it did
            not within SETTLE_TIMEOUT.
        """
        recent: deque = deque()
        deadline = ctx.now() + SETTLE_TIMEOUT
        while ctx.now() < deadline:
            pose, now = self._robot_pose(ctx, serial), ctx.now()
            if pose is not None:
                recent.append((now, *pose))
                while recent and now - recent[0][0] > SETTLE_FOR:
                    recent.popleft()
                window = np.array(recent)
                if window[-1, 0] - window[0, 0] >= SETTLE_FOR * 0.9:
                    spread = np.ptp(window[:, 1:3], axis=0).max()
                    turned = np.ptp(np.unwrap(window[:, 3]))
                    if spread <= SETTLE_POSITION and turned <= SETTLE_YAW:
                        return True
            await ctx.next_tick()
        return False

    def _stopped(self, ctx: PluginContext, clearance: Clearance | None) -> str:
        """The outcome of a cancelled run; a clearance trip also raises the alert, which stops automation."""
        if clearance is None or clearance.problem is None:
            return "stopped"
        self._alert = (f"{clearance.problem}: soft stopped. Resume it in the health panel, drive it clear, then "
                       f"start again")
        ctx.log_error(f"base experiment: {self._alert}")
        return "soft stopped: too close"

    # --- --- --- --- --- UPDATE --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Measure the run's tracking error on each new mocap fix; while idle, keep the preview on the robot.

        A way back that stopped automation shows in the collisions panel until the next Start.
        """
        if self._return_hit is not self._reported:
            self._reported = self._return_hit
            collisions: CollisionsContract = ctx.require("collisions")
            if self._return_hit is not None:
                label = f"{self._return_hit.robot.rsplit('/', 1)[-1]} way back"
                collisions.replace(self.name, {label: self._return_hit}, shown=(label,))
            else:
                collisions.clear(self.name)
        if self.running:
            self._measure(ctx)
        if self.serial is None or self.running:
            return
        anchor = self._robot_pose(ctx)
        if anchor is None:
            self._preview, self._preview_problem = None, f"no mocap pose for {self.serial} yet"
            return
        if not self._inputs_changed and self._anchor is not None and not _moved(anchor, self._anchor):
            return
        self._inputs_changed, self._anchor = False, anchor
        template = self._template.value
        settings = {"name": template, "mode": self._mode.value,
                    "parameters": {name: float(self._numbers[name].value) for name in TEMPLATES[template].parameters},
                    "linear_speed": float(self._linear.value), "turn_rate": math.radians(float(self._angular.value))}
        self._preview, self._preview_problem = plan_from(self.serial, settings, anchor)

    def _measure(self, ctx: PluginContext) -> None:
        """Match the run's robot's newest mocap fix to the path, once per fix."""
        if self._tracker is None or self._sent is None:
            return
        state = ctx.world.robots[self._sent.serial].base.state
        # ? The fix's arrival time stands in for its measurement time until mocap is stamped.
        if not state.tracked or state.last_fix_time is None or state.last_fix_time == self._measured_fix:
            return
        self._measured_fix = state.last_fix_time
        x, y, yaw = self._robot_pose(ctx, self._sent.serial)
        self._sample = self._tracker.update(x, y, yaw, state.last_fix_time, state.last_fix_time - self._sent_at)

    # --- --- --- --- --- READING --- --- --- --- ---

    def _robot_pose(self, ctx: PluginContext, serial: str | None = None) -> tuple[float, float, float] | None:
        """A robot's floor pose from mocap (the chosen one by default), or None before the first fix."""
        state = ctx.world.robots[serial or self.serial].base.state
        if state.position is None:
            return None
        return float(state.position[0]), float(state.position[1]), yaw_from_xyzw(state.orientation)

    def _follower(self, ctx: PluginContext) -> FollowerState | None:
        """The chosen robot's follower report if it is recent, else None."""
        return live_report(ctx.world.robots[self.serial].base.state.follower, ctx.now())

    def _enable_numbers(self) -> None:
        """Enable the numbers the chosen template uses, disable the rest."""
        used = TEMPLATES[self._template.value].parameters
        for name, widget in self._numbers.items():
            widget.disabled = name not in used

    # --- --- --- --- --- DRAW --- --- --- --- ---

    def draw(self, ctx: PluginContext) -> None:
        """The scene markers, the status and the plots."""
        if self.serial is None:
            return
        self._robot.value = self.serial
        preview = None if self.running or self._preview is None else self._preview.path
        self._markers.paths(preview, None if self._sent is None else self._sent.path)
        trail = None if self._record is None or self._sent is None else \
            self._record.values(f"{self._sent.serial}_floor_pose")[:, :2]
        self._markers.trail(trail)
        self._markers.follower(self._follower(ctx))
        self._status.content = self._status_html(ctx)
        self._details.content = self._details_html(ctx)
        self._progress.value = self._progress_percent()
        for plot in self._plots:
            plot.draw()

    def _status_html(self, ctx: PluginContext) -> str:
        """Chips on one line: robot, mocap, follower, velocity controller, run."""
        base = ctx.world.robots[self.serial].base
        chips = chip(escape(self.serial), SECTION_CTRL, "The robot the experiment drives.")
        lost = pose_lost(base.mocap_id, base.state, ctx.now())
        chips += (chip("not tracked", FAIL, lost) if lost
                  else chip("tracked", OK, "Mocap tracks the base; a run needs it."))
        follower = self._follower(ctx)
        if follower is None:
            chips += chip("no follower", FAIL, "start crl_husky pure_pursuit (or pure_pursuit_sim.launch.py)")
        else:
            chips += chip(f"follower {follower.state}", FOLLOWER_COLORS.get(follower.state, NONE), follower.reason)
        if not base.controllers.is_active(PLATFORM_VELOCITY_CONTROLLER):
            chips += chip("velocity controller off", BUSY, "the base will not move (fine in the simulator)")
        if self.running:
            chips += chip("running", BUSY, "A run drives the base; Stop ends it.")
        elif self._outcome:
            failed = self._outcome.startswith(("aborted", "timed out", "not started", "not driven", "not sent",
                                               "robot not changed", "soft stopped", E_STOPPED))
            # * Short on the chip ("not started" without its reason), the whole outcome on hover.
            label = "not started" if self._outcome.startswith("not started") else self._outcome.split(" → ")[0][:40]
            color = FAIL if failed else OK if self._outcome.startswith("done") else NONE
            chips += chip(escape(label), color, hover(self._outcome))
        return chip_line(chips)

    def _details_html(self, ctx: PluginContext) -> str:
        """The guard's and automation's alerts, then the plan and the follower's numbers in a fixed number of rows.

        ! The alerts stay until the next Start: the operator has to deal with them. They sit here, below the controls,
        so they push nothing.
        """
        plan = self._sent if self.running else self._preview
        if plan is None:
            size = f"path      {self._preview_problem or '—'}"
        else:
            drive = sum(p.length for p in plan.path.pieces if not p.turning)
            turn = sum(abs(p.yaw1 - p.yaw0) for p in plan.path.pieces if p.turning)
            size = (f"path      {drive:5.2f} m  {math.degrees(turn):5.0f}°  ~{plan.expected:5.1f} s  "
                    f"{len(plan.path.pieces)} pieces")
        follower = ctx.world.robots[self.serial].base.state.follower
        if follower is None:
            progress = errors = command = "—"
        else:
            progress = f"{follower.progress * 100:5.1f} %  piece {follower.piece + 1}/{follower.piece_count}"
            errors = (f"cross {_cm(follower.cross_track_error)}  along {_cm(follower.along_track_error)}  "
                      f"yaw {math.degrees(follower.yaw_error):+6.1f}°")
            command = f"v {follower.command[0]:+5.2f} m/s  w {math.degrees(follower.command[1]):+6.1f} °/s"
        label = escape(plan.label) if plan is not None else "—"
        alerts = ((warning(escape(self._alert), FAIL) if self._alert else "")
                  + (warning(escape(self._auto_alert)) if self._auto_alert else ""))
        return alerts + block(values(f"plan      {label}", size, f"progress  {progress}", f"error     {errors}",
                                     f"command   {command}", f"last      {escape(self._outcome) or '—'}",
                                     f"auto      {escape(self._auto_note) or '—'}",
                                     f"paths     {escape(self._paths_note) or '—'}"))

    def _progress_percent(self) -> float:
        """Progress for the bar: of the automated series if one has a set length, else of the current run."""
        # * Only while a run follows or settles: between runs the last run's progress no longer counts.
        active = self._phase in (FOLLOWING, SETTLING) and self._sample is not None
        run = self._sample.progress if active else 0.0
        if self._series is not None and self._series[1] > 0:
            done, runs, _ = self._series
            return min(100.0, 100.0 * (done + run) / runs)
        if self.running:
            return 100.0 * run
        return self._progress.value


def _attempt(outcome: str) -> bool:
    """Whether a run's outcome says how the controller did: done, or the controller's failure."""
    return outcome.startswith("done") or _controller_failure(outcome)


def _controller_failure(outcome: str) -> bool:
    """Whether a run's outcome is the controller's failure (series goes on), not a problem around it (it pauses)."""
    if outcome.startswith("timed out: did not finish"):
        return True
    reason = outcome.removeprefix("aborted: ")
    return (outcome.startswith("aborted: ") and not any(reason.startswith(r) for r in NOT_THE_CONTROLLER)
            and reason not in ("another path replaced it", "the follower went silent"))


def _message_signals(robot, follower: bool) -> list[Signal]:
    """Every mocap sample and odometry message, and every path sent and follower report or else every twist sent."""
    every = [signals.base_floor_pose(robot, every_message=True), signals.wheel_odometry(robot, every_message=True)]
    if follower:
        every += [signals.path_sent(robot), signals.follower_command(robot, every_message=True),
                  signals.follower_state(robot, every_message=True)]
    else:
        every.append(signals.twist_sent(robot))
    return every


def _return_plan(serial: str, pose, home) -> Plan:
    """A plan from `pose` back to `home`: turn towards it, drive straight, turn to its heading (geometric)."""
    x, y, yaw = pose
    hx, hy, hyaw = home
    if math.hypot(hx - x, hy - y) < 0.05:
        poses = [(x, y, yaw), (x, y, yaw + wrap(hyaw - yaw))]
    else:
        facing = yaw + wrap(math.atan2(hy - y, hx - x) - yaw)
        poses = [(x, y, yaw), (x, y, facing), (hx, hy, facing), (hx, hy, facing + wrap(hyaw - facing))]
    poses = np.array(poses)
    linear, angular = RETURN_SPEED
    settings = {"name": "Return to start", "parameters": {}, "mode": "Geometric", "linear_speed": linear,
                "turn_rate": angular}
    expected = float(timing(poses, linear, angular)[-1])
    return Plan(serial, poses, None, FollowerPath.from_poses(*poses.T), expected, "Return to start", "return",
                settings)


def _far(pose, home, radius: float) -> bool:
    """Whether `pose` is farther than `radius` from `home`; False if either is unknown."""
    return pose is not None and home is not None and math.hypot(pose[0] - home[0], pose[1] - home[1]) > radius


def _away(pose, home) -> bool:
    """Whether `pose` is off `home` by more than HOME_TOLERANCE, in position or heading; False if either is unknown."""
    if pose is None or home is None:
        return False
    distance, angle = HOME_TOLERANCE
    return _far(pose, home, distance) or abs(wrap(pose[2] - home[2])) > angle


def _pose_text(pose) -> str:
    """A floor pose for the log, e.g. "(1.00, -1.50, 0°)"."""
    return "unknown" if pose is None else f"({pose[0]:.2f}, {pose[1]:.2f}, {math.degrees(pose[2]):.0f}°)"


def _timing_text(elapsed: float, done: int, runs: int) -> str:
    """Elapsed time, and the time left estimated from the runs so far (only for a set number of runs)."""
    text = f"{_minutes(elapsed)} elapsed"
    if runs > 0 and done > 0:
        text += f", ~{_minutes(elapsed / done * (runs - done))} left"
    return text


def _minutes(seconds: float) -> str:
    """A duration as minutes and seconds, e.g. "4 min 05 s"."""
    minutes, seconds = divmod(int(round(seconds)), 60)
    return f"{minutes} min {seconds:02d} s"


def _moved(pose, anchor) -> bool:
    """Whether a floor pose moved from `anchor` by more than PREVIEW_POSITION or PREVIEW_YAW."""
    dyaw = math.remainder(pose[2] - anchor[2], 2 * math.pi)
    return math.hypot(pose[0] - anchor[0], pose[1] - anchor[1]) > PREVIEW_POSITION or abs(dyaw) > PREVIEW_YAW


def _cm(metres: float) -> str:
    """Metres as signed centimetres, or a dash for NaN."""
    return "    —" if not math.isfinite(metres) else f"{metres * 100:+5.1f}cm"


def _degrees(value):
    """Radians to degrees; None stays None."""
    return None if value is None else np.degrees(value)


def _pick(value, index: int):
    """One entry of a reading; None stays None."""
    return None if value is None else value[index]
