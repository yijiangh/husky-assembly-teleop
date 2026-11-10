"""
The Stop recovery tab: every robot's last rest poses, and a slow move back to the one picked.

After an e-stop or a protective stop the arms sag on their brakes; an arm that held a bar then pushes on it. Once the
robot runs again, moving it back to its pose from before the stop takes that load off. The tab shows the newest
poses per robot (the newest is picked), how far each arm is from the one picked and its force now and then.

* Go back asks first and acts on its second press. It sends one slow joint move (both arms together on a two-arm
  robot) under the robot's lease, and holds the robot as soon as a force rises by FORCE_RISE.
* ! It never touches a tool, and is refused for more than MAX_JOINT_STEP or MAX_TCP_STEP: farther is a job for the arm
  planner. No collision check: within a sag it is not needed, and a gripper around a bar would always fail it.
"""

from __future__ import annotations

import asyncio
from datetime import datetime

import numpy as np
import viser

from ...config import robot_name, short_serial
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...robot_interface.arm import STILL_AFTER, ArmInterface, joint_move
from ...robot_interface.robot import HuskyRobotInterface
from ...robot_interface.soft_stop import MotionRefused
from ...colors import rgb
from ...ui.second_press import SecondPress, show_waiting
from ...ui.style import (BUSY, FAIL, INFO, NONE, OK, SECTION_CTRL, block, chip, chip_line, note_line, section, values,
                         wrapped)
from .pose_log import ArmPose, PoseLog, RobotPose, Stop, append_lines

#: Poses offered per robot, newest first.
SHOWN = 5
#: Largest move Go back makes: per joint, radians, and of tool0, metres. A sag is far less.
MAX_JOINT_STEP = np.radians(3.0)
MAX_TCP_STEP = 0.02
#: How slowly it moves: peak joint speed, rad/s, and the shortest move, seconds.
MAX_SPEED = np.radians(2.0)
MIN_DURATION = 8.0
#: The robot is held once an arm's force is this much above its value when the move started, newtons.
FORCE_RISE = 5.0
#: Seconds after the move's end to wait for the arms to settle before reporting.
SETTLE = 1.0
#: A move smaller than this on every joint, radians, is not worth sending: the arm is already there.
ALREADY_THERE = np.radians(0.01)
#: The label of the button that moves back.
GO_BACK = "Go back"

GUIDE = (
    "After an e-stop or a protective stop while an arm holds something:",
    "1. Do not zero the force sensor, open or reactivate the gripper, or restart the arm driver: the gripper would "
    "activate, open and drop what it holds.",
    "2. Release the e-stop, then Resume the robot (Health).",
    "3. Pick the pose from before the stop (the newest is picked) and check how far each arm is and its force now "
    "and then.",
    "4. Press Go back twice: the robot moves slowly back and stops if a force rises by 5 N.",
    "5. Check that each force is back near its value then before going on.",
)


def arm_force(arm: ArmInterface) -> float | None:
    """The magnitude of an arm's measured force, newtons, raw; None if not reported."""
    wrench = arm.state.wrench
    return None if wrench is None else float(np.linalg.norm(np.asarray(wrench, dtype=float)[:3]))


def arm_moving(arm: ArmInterface, now: float) -> bool:
    """Whether an arm moved, even slightly, within the last STILL_AFTER seconds."""
    moved = arm.state.last_slight_motion_time
    return moved is not None and now - moved < STILL_AFTER


def running(robot: HuskyRobotInterface) -> bool:
    """Whether every arm of a robot runs: the safety sync calls it operational."""
    return bool(robot.arms) and all(arm.state.operational for arm in robot.arms.values())


def arm_pose(arm: ArmInterface) -> ArmPose | None:
    """An arm's pose now, or None before its six joints are known."""
    joints = arm.joint_vector()
    if joints is None:
        return None
    tcp = arm.state.tcp_position
    return ArmPose(tuple(float(value) for value in joints),
                   None if tcp is None else tuple(float(value) for value in tcp), arm_force(arm))


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
        #: Serial -> the move back running, if any.
        self._moves: dict[str, asyncio.Task] = {}
        #: Serial -> the message under its buttons, and its colour.
        self._messages: dict[str, tuple[str, str]] = {}
        self._press = SecondPress()
        self._writer: asyncio.Task | None = None
        self._widgets: dict[str, dict] = {}

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
                status = gui.add_html("")
                pick = gui.add_dropdown("Pose", ("no pose yet",), initial_value="no pose yet")
                readout = gui.add_html("")
                go = gui.add_button(GO_BACK, hint=f"Moves {robot_name(serial)} slowly back to the pose picked. "
                                                  f"{self._press.hint}")
                stop = gui.add_button("Stop", color=rgb(FAIL), hint="Holds the robot where it is.")
                message = gui.add_html("")
                go.on_click(ctx.defer(f"go back {serial}", lambda serial=serial: self._go_back(ctx, serial)))
                stop.on_click(ctx.defer(f"stop {serial}", lambda serial=serial: self._stop(ctx, serial)))
                self._widgets[serial] = {"status": status, "pick": pick, "readout": readout, "go": go,
                                         "message": message, "options": {}}
                self._moved_running[serial] = self._was_running[serial] = running(robot)
                self._running_since[serial] = float("-inf")

    # --- --- --- --- --- LOGGING --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Log each robot's rest pose after it moved while running, and each stop; write new lines in the background."""
        now = ctx.now()
        for serial in self._widgets:
            robot = ctx.world.robots[serial]
            runs = running(robot)
            if runs and not self._was_running[serial]:
                self._running_since[serial] = now
            if self._was_running[serial] and not runs:
                self._log_stop(ctx, serial, robot)
            self._was_running[serial] = runs
            moving = any(arm_moving(arm, now) for arm in robot.arms.values())
            # * Only motion since it runs again and since its last pose counts.
            since = max(self._running_since[serial], self._logged_at.get(serial, float("-inf")))
            moved = (arm.state.last_slight_motion_time for arm in robot.arms.values())
            if runs and any(time is not None and time > since for time in moved):
                self._moved_running[serial] = True
            if runs and not moving and self._moved_running[serial]:
                arms = {name: arm_pose(arm) for name, arm in robot.arms.items()}
                if all(pose is not None for pose in arms.values()):
                    self.log.add_pose(RobotPose(serial, now, datetime.now().isoformat(timespec="seconds"), arms))
                    self._moved_running[serial], self._logged_at[serial] = False, now
        if self.log.unwritten and (self._writer is None or self._writer.done()):
            lines, path = self.log.take_unwritten(), self.log.day_file(datetime.now())
            self._writer = ctx.spawn("stop recovery log", self._write(ctx, path, lines), survives_soft_stop=True)

    def _log_stop(self, ctx: PluginContext, serial: str, robot: HuskyRobotInterface) -> None:
        """Log a robot stopping, with what the sync says and each arm's force."""
        reason = next((arm.state.problem for arm in robot.arms.values() if arm.state.problem), "not operational")
        stop = Stop(serial, ctx.now(), datetime.now().isoformat(timespec="seconds"), reason,
                    {name: arm_force(arm) for name, arm in robot.arms.items()})
        self.log.add_stop(stop)
        ctx.log_warn(f"{robot_name(serial)} stopped ({reason}); its last pose before is in Stop recovery")

    async def _write(self, ctx: PluginContext, path, lines: list[str]) -> None:
        """Append lines to the day file on a thread; a failure is logged and the lines are lost."""
        try:
            await ctx.run_in_thread(append_lines, path, lines)
        except OSError as error:
            ctx.log_warn(f"stop recovery: could not write {path}: {error}")

    # --- --- --- --- --- GO BACK --- --- --- --- ---

    def _picked(self, serial: str) -> RobotPose | None:
        """The pose picked for a robot, or None."""
        widgets = self._widgets[serial]
        return widgets["options"].get(widgets["pick"].value)

    def _say(self, serial: str, text: str, color: str = NONE) -> None:
        self._messages[serial] = (text, color)

    def _go_back(self, ctx: PluginContext, serial: str) -> None:
        """Ask, then on the second press start moving the robot back to the pose picked (intent)."""
        robot = ctx.world.robots[serial]
        target = self._picked(serial)
        refusal = self._refusal(ctx, serial, robot, target)
        if refusal:
            self._press.forget()
            self._say(serial, f"Go back refused: {refusal}", FAIL)
            return
        if not self._press.press(ctx.now(), f"{serial} {target.wall}"):
            self._say(serial, f"Move {robot_name(serial)} back to {target.clock}? Press {GO_BACK} again.", BUSY)
            return
        self._moves[serial] = ctx.spawn(f"go back {serial}", self._move_back(ctx, serial, robot, target))

    def _refusal(self, ctx: PluginContext, serial: str, robot: HuskyRobotInterface,
                 target: RobotPose | None) -> str:
        """Why Go back may not start now, or ""."""
        if target is None:
            return "no pose picked"
        running_move = self._moves.get(serial)
        if running_move is not None and not running_move.done():
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
        """Send one slow joint move to `target` under the lease, hold on a force rise, then report the forces."""
        name = robot_name(serial)
        arms = robot.arms
        try:
            with ctx.lease_robots([serial], "stop recovery"):
                starts = {arm_name: arm.joint_vector() for arm_name, arm in arms.items()}
                goals = {arm_name: np.asarray(target.arms[arm_name].joints) for arm_name in arms}
                duration = max(joint_move(starts[each], goals[each], MAX_SPEED, MIN_DURATION)[2] for each in arms)
                moves = {each: joint_move(starts[each], goals[each], MAX_SPEED, duration) for each in arms}
                if len(arms) == 2:
                    sent = robot.send_multi_arm_trajectory({each: move[0] for each, move in moves.items()}, duration,
                                                           {each: move[1] for each, move in moves.items()})
                else:
                    (arm_name, arm), = arms.items()
                    sent = arm.send_joint_trajectory(moves[arm_name][0], duration, moves[arm_name][1])
                if not sent:
                    self._say(serial, "Go back refused by the arm; the console says why", FAIL)
                    return
                before = {arm_name: arm_force(arm) for arm_name, arm in arms.items()}
                ctx.log_info(f"{name}: moving back to {target.clock} over {duration:.0f} s")
                self._say(serial, f"Moving back to {target.clock} ({duration:.0f} s)…", BUSY)
                end = ctx.now() + duration + SETTLE
                try:
                    while ctx.now() < end:
                        await ctx.next_tick()
                        for arm_name, arm in arms.items():
                            force, start = arm_force(arm), before[arm_name]
                            if force is not None and start is not None and force > start + FORCE_RISE:
                                self._hold(robot)
                                text = (f"Held: {arm_name}'s force rose from {start:.1f} to {force:.1f} N; "
                                        f"check what it holds")
                                ctx.log_warn(f"{name}: {text}")
                                self._say(serial, text, FAIL)
                                return
                except asyncio.CancelledError:
                    self._hold(robot)
                    raise
        except MotionRefused as refused:
            self._say(serial, f"Go back refused: {refused}", FAIL)
            return
        forces = ", ".join(f"{arm_name} {_newtons(before[arm_name]).strip()} → {_newtons(arm_force(arm)).strip()} N "
                           f"(then {_newtons(target.arms[arm_name].force).strip()})"
                           for arm_name, arm in arms.items())
        ctx.log_info(f"{name}: back at {target.clock}; force {forces}")
        self._say(serial, f"Back at {target.clock}. Force {forces}", OK)

    @staticmethod
    def _hold(robot: HuskyRobotInterface) -> None:
        """Hold every arm of a robot where it is."""
        for arm in robot.arms.values():
            arm.hold()

    def _stop(self, ctx: PluginContext, serial: str) -> None:
        """Hold the robot and cancel its move back (intent)."""
        self._press.forget()
        task = self._moves.get(serial)
        if task is not None and not task.done():
            task.cancel()
        self._hold(ctx.world.robots[serial])
        self._say(serial, "Stopped: held where it is", NONE)

    # --- --- --- --- --- DRAW --- --- --- --- ---

    def draw(self, ctx: PluginContext) -> None:
        """Show each robot's state, its poses to pick from, the distances and forces, and its message."""
        now = ctx.now()
        waiting = self._press.waiting(now)
        for serial, widgets in self._widgets.items():
            robot = ctx.world.robots[serial]
            self._draw_choices(serial, widgets)
            target = self._picked(serial)
            moving_back = serial in self._moves and not self._moves[serial].done()
            stops = self.log.stops.get(serial, [])
            if moving_back:
                state = chip("moving back", BUSY, "Go back runs; Stop holds the robot")
            elif not running(robot):
                problem = next((arm.state.problem for arm in robot.arms.values() if arm.state.problem), "")
                state = chip("stopped", FAIL, problem or "not operational")
            elif stops and target is not None and stops[-1].ros_time > target.ros_time:
                state = chip(f"stopped at {stops[-1].clock}", INFO, f"{stops[-1].reason}; it runs again")
            else:
                state = chip("running", OK, "every arm is operational")
            widgets["status"].content = chip_line(state)
            widgets["readout"].content = block(values(*self._readout(robot, target)))
            show_waiting(widgets["go"], waiting.startswith(f"{serial} "), GO_BACK, None)
            text, color = self._messages.get(serial, ("", NONE))
            widgets["message"].content = note_line(text, color)

    def _draw_choices(self, serial: str, widgets: dict) -> None:
        """Offer the newest SHOWN poses, newest first; a new pose is picked once it arrives."""
        poses = self.log.last(serial, SHOWN)
        options = {}
        for index, pose in enumerate(poses):
            stop = self.log.stop_after(pose)
            label = pose.clock + (" newest" if index == 0 else "")
            if stop is not None:
                label += f" · before stop {stop.clock}"
            if label in options:  # two poses in one second
                label += f" ({index + 1})"
            options[label] = pose
        if not options:
            return
        if list(options) != list(widgets["options"]):
            newest_was = next(iter(widgets["options"]), None)
            picked = widgets["pick"].value
            widgets["options"] = options
            widgets["pick"].options = tuple(options)
            # * Keep the operator's pick unless a newer pose came: then the newest is picked.
            keep = picked in options and picked != newest_was
            widgets["pick"].value = picked if keep else next(iter(options))

    @staticmethod
    def _readout(robot: HuskyRobotInterface, target: RobotPose | None) -> list[str]:
        """A line per arm: how far from the pose picked, and its force now and then."""
        lines = []
        for name, arm in robot.arms.items():
            then = None if target is None else target.arms.get(name)
            step, gap = (None, None) if then is None else distance(arm, then)
            away = "     —      " if step is None else (
                f"{np.degrees(step):5.2f}° {'  — ' if gap is None else f'{gap * 1000:4.1f}'} mm")
            force_then = None if then is None else then.force
            lines.append(f"{name:<13} away {away}  force {_newtons(arm_force(arm))} N, then {_newtons(force_then)} N")
        return lines
