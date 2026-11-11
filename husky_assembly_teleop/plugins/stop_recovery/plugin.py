"""
The Stop recovery tab: every robot's last rest poses, and a slow move back to the one picked.

After an e-stop or a protective stop the arms sag on their brakes; an arm that held a bar then pushes on it. Once the
robot runs again, moving it back to its pose from before the stop takes that load off. The tab offers the newest poses
per robot and shows how far each arm is from the one picked, and how far its force is off from its force then.

* Picked by default: the newest pose a Go back did not end at, so a Go back held halfway never hides the pose it aimed
  for. Older poses (a robot that stopped again before it went back) are a pick away.
* Go back asks first and acts on its second press. It sends one slow joint move (both arms together on a two-arm
  robot) under the robot's lease, waits until the joints arrive, and holds the robot as soon as an arm's force goes
  FORCE_RISE further off its force then. Forces are compared as vectors, raw: the sensor's offset and the tool's weight
  cancel out, as long as nobody zeroes the sensor in between.
* ! It never touches a tool, and is refused for more than MAX_JOINT_STEP or MAX_TCP_STEP: farther is a job for the arm
  planner. No collision check: within a sag it is not needed, and a gripper around a bar would always fail it.
* A robot that runs again after a stop seen by this monitor gets a window beside the main panel with the same controls
  (`Popup`), until Done, or until it rests somewhere else than a Go back left it (recovered some other way).
"""

from __future__ import annotations

import asyncio
from datetime import datetime

import numpy as np
import viser

from ...colors import rgb
from ...config import robot_name, short_serial
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...robot_interface.arm import SCALED_JOINT_TRAJECTORY_CONTROLLER, STILL_AFTER, ArmInterface, joint_move
from ...robot_interface.refusal import newest_reason
from ...robot_interface.robot import HuskyRobotInterface
from ...robot_interface.soft_stop import MotionRefused
from ...shared.guide.panel import set_changed
from ...ui.popup import Popup
from ...ui.second_press import SecondPress, show_waiting
from ...ui.style import (BUSY, FAIL, INFO, NONE, OK, SECTION_CTRL, block, chip, chip_line, note_line, section, values,
                         wrapped)
from .pose_log import SAME_POSE, ArmPose, PoseLog, RobotPose, Stop, append_lines

#: Poses offered per robot, newest first.
SHOWN = 5
#: Largest move Go back makes: per joint, radians, and of tool0, metres. A sag is far less.
MAX_JOINT_STEP = np.radians(3.0)
MAX_TCP_STEP = 0.02
#: How slowly it moves: peak joint speed, rad/s, and the shortest move, seconds.
MAX_SPEED = np.radians(2.0)
MIN_DURATION = 8.0
#: The robot is held once an arm's force is this much further off its force then than when the move started, newtons.
FORCE_RISE = 5.0
#: Arrived: every joint this close to the pose, radians.
ARRIVED = np.radians(0.05)
#: How long to wait for the arms to arrive: this many times the move's duration, plus ARRIVAL_SLACK seconds (a pendant
#: speed slider below 100 % slows the move).
ARRIVAL_FACTOR, ARRIVAL_SLACK = 2.0, 5.0
#: Seconds after arriving to wait for the arms to settle before reporting.
SETTLE = 1.0
#: A move smaller than this on every joint, radians, is not worth sending: the arm is already there.
ALREADY_THERE = np.radians(0.01)
#: The labels of the button that moves back, and of the window's button that closes a robot's part.
GO_BACK, DONE = "Go back", "Done"

GUIDE = (
    "After an e-stop or a protective stop while an arm holds something:",
    "1. Do not zero the force sensor, open or reactivate the gripper, or restart the robot's launch: the gripper "
    "would activate, open and drop what it holds.",
    "2. E-stop: release it, then on the pendant press ON and then START (the brakes release; the arm stays). "
    "Protective stop: clear its cause, then Unlock in Health. After Stop all: Resume. Wait for “running” here.",
    "3. The pose from before the stop is picked. Check how far each arm is from it and how far its force is off.",
    f"4. Press Go back twice (pendant speed slider at 100 %): the robot moves slowly back and holds if a force goes "
    f"{FORCE_RISE:g} N further off.",
    "5. Go on only once each force is back near 0 N off (a few N: after an e-stop the sensor may read a little "
    "differently).",
)
#: The window's guide: the robot already runs, so steps 3 to 5.
POPUP_GUIDE = (
    "Runs again after a stop. Do not zero the force sensor or touch the gripper.",
    "Check how far each arm is from the pose picked and how far its force is off, then press Go back twice (pendant "
    "speed slider at 100 %). Done once each force is back near 0 N off.",
)


def arm_force(arm: ArmInterface) -> np.ndarray | None:
    """An arm's measured force (fx, fy, fz), newtons, raw; None if not reported."""
    wrench = arm.state.wrench
    return None if wrench is None else np.asarray(wrench, dtype=float)[:3]


def force_off(arm: ArmInterface, then: ArmPose) -> float | None:
    """How far an arm's force now is from its force at `then`, newtons; None if either is not known."""
    now = arm_force(arm)
    return None if now is None or then.force is None else float(np.linalg.norm(now - np.asarray(then.force)))


def arm_moving(arm: ArmInterface, now: float) -> bool:
    """Whether an arm moved, even slightly, within the last STILL_AFTER seconds."""
    moved = arm.state.last_slight_motion_time
    return moved is not None and now - moved < STILL_AFTER


def running(robot: HuskyRobotInterface) -> bool:
    """Whether every arm of a robot runs: the safety sync calls it operational."""
    return bool(robot.arms) and all(arm.state.operational for arm in robot.arms.values())


def simulated(robot: HuskyRobotInterface) -> bool:
    """Whether a robot's arms run on fake hardware."""
    return any(arm.simulation is not None for arm in robot.arms.values())


def _vector(values) -> tuple[float, ...] | None:
    return None if values is None else tuple(float(value) for value in values)


def arm_pose(arm: ArmInterface) -> ArmPose | None:
    """An arm's pose now, or None before its six joints are known."""
    joints = arm.joint_vector()
    return None if joints is None else ArmPose(_vector(joints), _vector(arm.state.tcp_position),
                                               _vector(arm_force(arm)))


def distance(arm: ArmInterface, target: ArmPose) -> tuple[float | None, float | None]:
    """How far an arm is from `target`: largest joint step, radians, and tool0, metres (None where unknown)."""
    joints = arm.joint_vector()
    step = None if joints is None else float(np.max(np.abs(np.asarray(target.joints) - joints)))
    tcp = arm.state.tcp_position
    gap = None if tcp is None or target.tcp is None else float(np.linalg.norm(np.asarray(target.tcp) - tcp))
    return step, gap


def _newtons(value: float | None) -> str:
    return "  —  " if value is None else f"{value:5.1f}"


@register
class StopRecoveryPlugin(HuskyPlugin):
    """Logs every robot's rest poses and moves a robot back to one after a stop."""

    name = "stop_recovery"
    icon = viser.Icon.ARROW_BACK_UP

    def __init__(self):
        """Start with no poses; setup reads the day files."""
        self.log = PoseLog()
        #: Serial -> whether it moved while running since its last logged pose. At start, whether it runs: a robot that
        #: does not run yet (no status, or stopped and maybe sagging) logs nothing until it really moves.
        self._moved_running: dict[str, bool] = {}
        #: Serial -> whether it ran at the last tick, to log a stop once.
        self._was_running: dict[str, bool] = {}
        #: Serial -> ROS time it last started running; motion before then (a sag while stopped) never counts.
        self._running_since: dict[str, float] = {}
        #: Serial -> ROS time its last pose was logged.
        self._logged_at: dict[str, float] = {}
        #: Serials whose next logged pose is where a Go back ended.
        self._ended_go_back: set[str] = set()
        #: Serial -> the move back running, if any.
        self._moves: dict[str, asyncio.Task] = {}
        #: Serial -> the message under its buttons, and its colour.
        self._messages: dict[str, tuple[str, str]] = {}
        self._press = SecondPress()
        self._writer: asyncio.Task | None = None
        #: Serial -> its controls in the tab, which hold the pick and its options; and in the window.
        self._widgets: dict[str, dict] = {}
        self._popup_widgets: dict[str, dict] = {}
        #: Serial -> its folder in the window, and the pick last copied into the window's dropdown.
        self._popup_folders: dict[str, viser.GuiFolderHandle] = {}
        self._mirrored: dict[str, str] = {}
        #: Serials stopped while this monitor watched, and those whose window part shows (running again since).
        self._stopped: set[str] = set()
        self._asking: set[str] = set()
        self._popup: Popup | None = None

    # --- --- --- --- --- SETUP --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Read today's and yesterday's poses, then build a block per robot with arms under the guide."""
        robots = {serial: robot for serial, robot in ctx.world.robots.items() if robot.arms}
        self.log.load(datetime.now(), robots)
        with ctx.view.ui() as gui:
            gui.add_html(block(wrapped(*GUIDE)))
            if not robots:
                gui.add_html(block(chip("no arms", NONE, "start the monitor with robots that have arms")))
            for serial, robot in robots.items():
                gui.add_html(section(f"{robot_name(serial)} ({short_serial(serial)})", SECTION_CTRL,
                                     ", ".join(robot.arms)))
                self._widgets[serial] = {**self._controls(ctx, gui, serial, popup=False), "options": {},
                                         "default": None}
                self._moved_running[serial] = self._was_running[serial] = running(robot)
                self._running_since[serial] = float("-inf")
        if not robots:
            return
        self._popup = Popup(ctx.view, "Back from a stop", viser.Icon.ARROW_BACK_UP)
        with self._popup.body() as gui:
            gui.add_html(block(wrapped(*POPUP_GUIDE)))
            for serial, robot in robots.items():
                folder = gui.add_folder(f"{robot_name(serial)} ({short_serial(serial)})", visible=False)
                with folder:
                    self._popup_widgets[serial] = self._controls(ctx, gui, serial, popup=True)
                self._popup_folders[serial] = folder

    def _controls(self, ctx: PluginContext, gui: viser.GuiApi, serial: str, popup: bool) -> dict:
        """One robot's state, pose pick, distances, buttons and message; in the window also Done.

        The tab's dropdown holds the pick; the window's hands an operator's pick to it.
        """
        status = gui.add_html("")
        pick = gui.add_dropdown("Pose", ("no pose yet",), initial_value="no pose yet")
        readout = gui.add_html("")
        go = gui.add_button(GO_BACK, hint=f"Moves {robot_name(serial)} slowly back to the pose picked. "
                                          f"{self._press.hint}")
        stop = gui.add_button("Stop", color=rgb(FAIL), hint="Holds the robot where it is.")
        if popup:
            done = gui.add_button(DONE, hint="Closes this robot's part of the window; the Stop recovery tab keeps "
                                             "its poses and buttons.")
            done.on_click(ctx.defer(f"stop recovery done {serial}", lambda: self._done(serial)))
            pick.on_update(ctx.defer_value(f"stop recovery pick {serial}",
                                           lambda value: self._pick_from_popup(serial, value), operator_only=True))
        message = gui.add_html("")
        go.on_click(ctx.defer(f"go back {serial}", lambda: self._go_back(ctx, serial)))
        stop.on_click(ctx.defer(f"stop {serial}", lambda: self._stop(ctx, serial)))
        widgets = {"status": status, "pick": pick, "readout": readout, "go": go, "message": message}
        return {**widgets, "done": done} if popup else widgets

    def _pick_from_popup(self, serial: str, value: object) -> None:
        """An operator's pick in the window: the tab's dropdown holds it."""
        if value in self._widgets[serial]["options"]:
            self._widgets[serial]["pick"].value = self._mirrored[serial] = value

    def _done(self, serial: str) -> None:
        """Close a robot's part of the window (intent); not while it moves back."""
        if self._moving_back(serial):
            self._say(serial, "Moving back: Stop it first, or wait until it is back", FAIL)
            return
        self._asking.discard(serial)

    # --- --- --- --- --- LOGGING --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Log each robot's rest pose after it moved while running, and each stop; write new lines in the background."""
        now = ctx.now()
        for serial in self._widgets:
            robot = ctx.world.robots[serial]
            runs = running(robot)
            if runs and not self._was_running[serial]:
                self._running_since[serial] = now
                if serial in self._stopped:
                    self._asking.add(serial)
            if self._was_running[serial] and not runs:
                self._log_stop(ctx, serial, robot)
                self._stopped.add(serial)
                # ! Motion seen just before the stop was reported may be the sag itself: forget it.
                self._moved_running[serial] = False
            self._was_running[serial] = runs
            moving = any(arm_moving(arm, now) for arm in robot.arms.values())
            # * Only motion since it runs again and since its last pose counts.
            since = max(self._running_since[serial], self._logged_at.get(serial, float("-inf")))
            moved = (arm.state.last_slight_motion_time for arm in robot.arms.values())
            if runs and any(time is not None and time > since for time in moved):
                self._moved_running[serial] = True
            if runs and not moving and self._moved_running[serial] and not self._moving_back(serial):
                arms = {name: arm_pose(arm) for name, arm in robot.arms.items()}
                if all(pose is not None for pose in arms.values()):
                    # * Not logged when it is the newest pose again (a Go back that ended on it): the mark goes too.
                    self.log.add_pose(RobotPose(serial, now, datetime.now().isoformat(timespec="seconds"),
                                                simulated(robot), arms, go_back=serial in self._ended_go_back))
                    self._moved_running[serial], self._logged_at[serial] = False, now
                    # * Resting somewhere a Go back did not leave it: recovered some other way, the window is done.
                    if serial not in self._ended_go_back:
                        self._asking.discard(serial)
                    self._ended_go_back.discard(serial)
        # * Here, not in draw: draw is skipped while the panels are paused, and the window must still open.
        self._show_popup()
        if self.log.unwritten and (self._writer is None or self._writer.done()):
            lines, path = self.log.take_unwritten(), self.log.day_file(datetime.now())
            self._writer = ctx.spawn("stop recovery log", self._write(ctx, path, lines), survives_soft_stop=True)

    def _log_stop(self, ctx: PluginContext, serial: str, robot: HuskyRobotInterface) -> None:
        """Log a robot stopping, with what the sync says and each arm's force."""
        reason = next((arm.state.problem for arm in robot.arms.values() if arm.state.problem), "not operational")
        stop = Stop(serial, ctx.now(), datetime.now().isoformat(timespec="seconds"), simulated(robot), reason,
                    {name: _vector(arm_force(arm)) for name, arm in robot.arms.items()})
        self.log.add_stop(stop)
        ctx.log_warn(f"{robot_name(serial)} stopped ({reason}); its pose before is in Stop recovery")

    async def _write(self, ctx: PluginContext, path, lines: list[str]) -> None:
        """Append lines to the day file on a thread; a failure is logged and the lines are lost."""
        try:
            await ctx.run_in_thread(append_lines, path, lines)
        except OSError as error:
            ctx.log_warn(f"stop recovery: could not write {path}: {error}")

    # --- --- --- --- --- GO BACK --- --- --- --- ---

    def _moving_back(self, serial: str) -> bool:
        """Whether a Go back of this robot runs."""
        task = self._moves.get(serial)
        return task is not None and not task.done()

    def _picked(self, serial: str) -> RobotPose | None:
        """The pose picked for a robot, or None."""
        widgets = self._widgets[serial]
        return widgets["options"].get(widgets["pick"].value)

    def _say(self, serial: str, text: str, color: str = NONE) -> None:
        self._messages[serial] = (text, color)

    def _forget_press(self, serial: str) -> None:
        """Forget a first press of this robot's Go back; another robot's waits on."""
        if self._press.key.startswith(f"{serial} "):
            self._press.forget()

    def _go_back(self, ctx: PluginContext, serial: str) -> None:
        """Ask, then on the second press start moving the robot back to the pose picked (intent)."""
        robot = ctx.world.robots[serial]
        target = self._picked(serial)
        refusal = self._refusal(ctx, serial, robot, target)
        if refusal:
            self._forget_press(serial)
            self._say(serial, f"Go back refused: {refusal}", FAIL)
            return
        if not self._press.press(ctx.now(), f"{serial} {target.order}"):
            self._say(serial, f"Move {robot_name(serial)} back to {target.clock}? Press {GO_BACK} again.", BUSY)
            return
        self._moves[serial] = ctx.spawn(f"go back {serial}", self._move_back(ctx, serial, robot, target))

    def _refusal(self, ctx: PluginContext, serial: str, robot: HuskyRobotInterface,
                 target: RobotPose | None) -> str:
        """Why Go back may not start now, or ""."""
        if target is None:
            return "no pose picked"
        if self._moving_back(serial):
            return "it is already moving back"
        if robot.soft_stopped:
            return "the robot is soft stopped; press Resume first"
        if set(target.arms) != set(robot.arms):
            return f"the pose has arms {sorted(target.arms)}, the robot {sorted(robot.arms)}"
        if any(arm_moving(arm, ctx.now()) for arm in robot.arms.values()):
            return "an arm is moving; wait until it stands"
        steps = []
        for name, arm in robot.arms.items():
            problem = arm.safety_problem()
            if problem:
                return f"{name}: {problem}"
            if not arm.controllers.is_active(SCALED_JOINT_TRAJECTORY_CONTROLLER):
                return f"{name} is not under the joint controller: Control › {name} › Controller: Joint"
            if force_off(arm, target.arms[name]) is None:
                return f"{name} has no force reading, now or then: the move could not be watched"
            step, gap = distance(arm, target.arms[name])
            if step is None:
                return f"{name}: no joint state"
            if step > MAX_JOINT_STEP:
                return f"{name} is {np.degrees(step):.1f}° away (at most {np.degrees(MAX_JOINT_STEP):.0f}°): " \
                       f"use the arm planner"
            if gap is not None and gap > MAX_TCP_STEP:
                return f"{name}'s tool0 is {gap * 1000:.0f} mm away (at most {MAX_TCP_STEP * 1000:.0f} mm): " \
                       f"use the arm planner"
            steps.append(step)
        if max(steps) < ALREADY_THERE:
            return f"already at {target.clock}"
        return ""

    async def _move_back(self, ctx: PluginContext, serial: str, robot: HuskyRobotInterface,
                         target: RobotPose) -> None:
        """Send one slow joint move to `target` under the lease, hold on a force rise, report once arrived."""
        arms = robot.arms
        try:
            with ctx.lease_robots([serial], "stop recovery"):
                starts = {name: arm.joint_vector() for name, arm in arms.items()}
                goals = {name: np.asarray(target.arms[name].joints) for name in arms}
                duration = max(joint_move(starts[name], goals[name], MAX_SPEED, MIN_DURATION)[2] for name in arms)
                moves = {name: joint_move(starts[name], goals[name], MAX_SPEED, duration) for name in arms}
                if len(arms) == 2:
                    sent = robot.send_multi_arm_trajectory({name: move[0] for name, move in moves.items()}, duration,
                                                           {name: move[1] for name, move in moves.items()})
                else:
                    (name, arm), = arms.items()
                    sent = arm.send_joint_trajectory(moves[name][0], duration, moves[name][1])
                if not sent:
                    self._say(serial, f"Go back refused: {newest_reason(*arms.values())}", FAIL)
                    return
                off = {name: force_off(arm, target.arms[name]) for name, arm in arms.items()}
                ctx.log_info(f"{robot_name(serial)}: moving back to {target.clock} over {duration:.0f} s")
                self._say(serial, f"Moving back to {target.clock} ({duration:.0f} s)…", BUSY)
                try:
                    if not await self._watch(ctx, serial, robot, target, off, duration):
                        return
                except asyncio.CancelledError:
                    self._hold(robot)
                    self._say(serial, "Go back stopped (Stop or Stop all): held where it is", NONE)
                    raise
                finally:
                    # * The next rest pose is where this Go back ended only if it moved the robot at all.
                    if any(np.max(np.abs(arm.joint_vector() - starts[name])) > SAME_POSE
                           for name, arm in arms.items() if arm.joint_vector() is not None):
                        self._ended_go_back.add(serial)
        except MotionRefused as refused:
            self._say(serial, f"Go back refused: {refused}", FAIL)
            return
        forces = ", ".join(f"{name} {_newtons(off[name]).strip()} → "
                           f"{_newtons(force_off(arm, target.arms[name])).strip()}" for name, arm in arms.items())
        ctx.log_info(f"{robot_name(serial)}: back at {target.clock}; force off by {forces} N")
        self._say(serial, f"Back at {target.clock}. Force off by {forces} N", OK)

    async def _watch(self, ctx: PluginContext, serial: str, robot: HuskyRobotInterface, target: RobotPose,
                     off: dict[str, float], duration: float) -> bool:
        """Tick until the arms arrive and settle; hold on a force rise or a timeout. True once back."""
        start, arrived = ctx.now(), None
        while arrived is None or ctx.now() < arrived + SETTLE:
            await ctx.next_tick()
            for name, arm in robot.arms.items():
                now_off = force_off(arm, target.arms[name])
                if now_off is None or now_off > off[name] + FORCE_RISE:
                    self._hold(robot)
                    text = (f"Held: {name}'s force is {_newtons(now_off).strip()} N off its force then "
                            f"({_newtons(off[name]).strip()} N at the start); check what it holds")
                    ctx.log_warn(f"{robot_name(serial)}: {text}")
                    self._say(serial, text, FAIL)
                    return False
            steps = [distance(arm, target.arms[name])[0] for name, arm in robot.arms.items()]
            if arrived is None and all(step is not None and step <= ARRIVED for step in steps):
                arrived = ctx.now()
            if arrived is None and ctx.now() - start > ARRIVAL_FACTOR * duration + ARRIVAL_SLACK:
                self._hold(robot)
                away = max((step for step in steps if step is not None), default=float("nan"))
                text = (f"Not back after {ctx.now() - start:.0f} s, {np.degrees(away):.2f}° away: held. Is the "
                        f"pendant's speed slider at 100 %, or is the arm blocked?")
                ctx.log_warn(f"{robot_name(serial)}: {text}")
                self._say(serial, text, FAIL)
                return False
        return True

    @staticmethod
    def _hold(robot: HuskyRobotInterface) -> None:
        """Hold every arm of a robot where it is."""
        for arm in robot.arms.values():
            arm.hold()

    def _stop(self, ctx: PluginContext, serial: str) -> None:
        """Hold the robot and cancel its move back (intent)."""
        self._forget_press(serial)
        task = self._moves.get(serial)
        if task is not None and not task.done():
            task.cancel()
        self._hold(ctx.world.robots[serial])
        self._say(serial, "Stopped: held where it is", NONE)

    # --- --- --- --- --- DRAW --- --- --- --- ---

    def draw(self, ctx: PluginContext) -> None:
        """Show each robot's state, poses to pick, distances, forces and message; in the window too while it shows."""
        waiting = self._press.waiting(ctx.now())
        for serial, widgets in self._widgets.items():
            robot = ctx.world.robots[serial]
            self._draw_choices(serial, widgets, simulated(robot))
            self._draw_popup_choices(serial)
            target = self._picked(serial)
            stops = [stop for stop in self.log.stops.get(serial, []) if stop.sim == simulated(robot)]
            if self._moving_back(serial):
                state = chip("moving back", BUSY, "Go back runs; Stop holds the robot")
            elif not running(robot):
                problem = next((arm.state.problem for arm in robot.arms.values() if arm.state.problem), "")
                state = chip("stopped", FAIL, problem or "not operational")
            elif stops and target is not None and stops[-1].order > target.order:
                state = chip(f"running, stopped at {stops[-1].clock}", INFO, stops[-1].reason)
            else:
                state = chip("running", OK, "every arm is operational")
            readout = block(values(*self._readout(robot, target)))
            text, color = self._messages.get(serial, ("", NONE))
            shown = [widgets] + ([self._popup_widgets[serial]] if serial in self._asking else [])
            for view in shown:
                set_changed(view["status"], content=chip_line(state))
                set_changed(view["readout"], content=readout)
                show_waiting(view["go"], waiting.startswith(f"{serial} "), GO_BACK, None)
                set_changed(view["message"], content=note_line(text, color))

    def _show_popup(self) -> None:
        """Show the window with the robots back from a stop, or close it when none is."""
        if self._popup is None:
            return
        for serial, folder in self._popup_folders.items():
            if folder.visible != (serial in self._asking):
                folder.visible = serial in self._asking
        if self._asking:
            self._popup.show("")
        elif self._popup.is_open:
            self._popup.close()

    def _draw_popup_choices(self, serial: str) -> None:
        """Give the window's dropdown the tab's options and, when it changed, the tab's pick."""
        widgets, popup = self._widgets[serial], self._popup_widgets.get(serial)
        if popup is None:
            return
        options = tuple(widgets["options"]) or ("no pose yet",)
        if tuple(popup["pick"].options) != options:
            popup["pick"].options = options
            self._mirrored.pop(serial, None)
        if self._mirrored.get(serial) != widgets["pick"].value:
            popup["pick"].value = self._mirrored[serial] = widgets["pick"].value

    def _draw_choices(self, serial: str, widgets: dict, sim: bool) -> None:
        """Offer the newest SHOWN poses, newest first; when they change, pick the default unless one was picked."""
        poses = self.log.last(serial, SHOWN, sim)
        options = {}
        for index, pose in enumerate(poses):
            stop = self.log.stop_after(pose)
            label = pose.clock + (" · Go back ended" if pose.go_back else "")
            if stop is not None:
                label += f" · before stop {stop.clock}"
            if label in options:  # two poses in one second
                label += f" ({index + 1})"
            options[label] = pose
        if not options or list(options) == list(widgets["options"]):
            return
        default = next((label for label, pose in options.items() if not pose.go_back), next(iter(options)))
        picked = widgets["pick"].value
        # * Keep the operator's own pick; else the default.
        keep = picked in options and picked != widgets["default"]
        widgets["options"], widgets["default"] = options, default
        widgets["pick"].options = tuple(options)
        widgets["pick"].value = picked if keep else default

    @staticmethod
    def _readout(robot: HuskyRobotInterface, target: RobotPose | None) -> list[str]:
        """A line per arm: how far from the pose picked, and how far its force is off from its force then."""
        lines = []
        for name, arm in robot.arms.items():
            then = None if target is None else target.arms.get(name)
            step, gap = (None, None) if then is None else distance(arm, then)
            away = "     —      " if step is None else (
                f"{np.degrees(step):5.2f}° {'  — ' if gap is None else f'{gap * 1000:4.1f}'} mm")
            off = None if then is None else force_off(arm, then)
            lines.append(f"{name:<13} away {away}  force off {_newtons(off)} N")
        return lines
