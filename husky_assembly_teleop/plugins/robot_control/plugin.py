"""
The robot control panel: a tab per robot, with inline tabs per part showing state and basic control.

* The pattern to copy when writing a plugin:
  - `setup` builds every widget once; `draw` only copies state into them.
  - ! Widget callbacks go through `ctx.defer` / `ctx.defer_value`: viser calls them
    on its own threads, where touching robots or the scene is not safe.
  - `update` does per-tick work; anything longer than a tick is a `ctx.spawn` task.
* Robots are commanded and read only through their interfaces; no topic names here.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from functools import partial
from typing import Any, Callable, Coroutine

import numpy as np
import viser
from scipy.spatial.transform import Rotation

from ...plugin_api.concurrency import WaitTimeout
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...robot_interface.arm import (CARTESIAN_COMPLIANCE_CONTROLLER, FREE_DRIVE_CONTROLLER,
                                    SCALED_JOINT_TRAJECTORY_CONTROLLER, TARGET_WRENCH_FRAME, UR_JOINT_NAMES,
                                    ArmInterface, cartesian_move)
from ...robot_interface.base import PLATFORM_VELOCITY_CONTROLLER, BaseInterface
from ...ui.ghost import RecentUse, RobotGhost, robot_ghosts
from ...ui.style import SECTION_SENSOR, block, numbers, section, values
from .arm_panel import (ComplianceInputs, JointInputs, build_compliance_inputs, build_free_drive_inputs,
                        build_joint_inputs, build_tool_inputs, joint_plan_line, pose_from_sliders, pose_plan_line)
from .dpad import HOLD_TIMEOUT, MAX_ANGULAR_SPEED, MAX_LINEAR_SPEED, build_velocity_inputs
from .markers import add_force_arrows, add_frame_markers, draw_markers, place_markers
from .status import ControllerPanel, arm_status, base_status, build_controller_panel, show_only_running, tool_status

#: A move has arrived when every joint is this close to its target, degrees.
ARRIVED_TOLERANCE = 0.5
#: Seconds past the planned duration before an unfinished move is reported as timed out.
ARRIVE_TIMEOUT_MARGIN = 5.0
#: Starting panel width, pixels. Fits the arm's joint row, the widest line.
PANEL_WIDTH = 420
#: Tab labels for arms, by ArmConfig.name.
ARM_LABELS = {"ur_arm": "Arm", "left_ur_arm": "Left", "right_ur_arm": "Right"}


@dataclass
class _BaseWidgets:
    """Handles for one base."""

    serial: str
    base: BaseInterface
    panel: ControllerPanel
    status: viser.GuiHtmlHandle
    speed: viser.GuiSliderHandle


@dataclass
class _ArmWidgets:
    """Handles for one arm and the tool on it."""

    serial: str
    arm: ArmInterface
    panel: ControllerPanel
    status: viser.GuiHtmlHandle
    tool_status: viser.GuiHtmlHandle | None
    joint: JointInputs
    compliance: ComplianceInputs
    frames: dict[str, viser.FrameHandle]
    arrows: dict[str, viser.ArrowsHandle]


@register
class RobotControlPlugin(HuskyPlugin):
    """Shows each robot's state and gives basic control over it."""

    name = "robot_control"

    def __init__(self):
        """Start with no widgets; setup builds them."""
        self._bases: list[_BaseWidgets] = []
        self._arms: list[_ArmWidgets] = []

        #: Bases whose twist was streamed last tick, so update sends one stop on release.
        self._driving: set[str] = set()

        #: Last hold report per base serial: (linear sign, angular sign, arrival time).
        self._held: dict[str, tuple[float, float, float]] = {}

        #: Running move task per arm, keyed by (serial, arm name).
        self._trajectory_tasks: dict[tuple[str, str], asyncio.Task] = {}

        #: Arms whose joint sliders are set on the next update: angles (radians), or None for the measured joints.
        self._load_sliders: dict[tuple[str, str], np.ndarray | None] = {}

        #: Arms whose pose sliders are set to the measured TCP on the next update.
        self._load_pose: set[tuple[str, str]] = set()

        #: Target force last sent to each arm, N; the controller keeps applying it.
        self._applied_force: dict[tuple[str, str], np.ndarray] = {}

        #: This tick's 3D marker poses and force-arrow segments per arm; None without the compliance controller.
        self._markers: dict[tuple[str, str], dict[str, tuple] | None] = {}

        #: Per robot, a ghost at its arms' joint slider values.
        self._target_ghosts: dict[str, RobotGhost] = {}
        #: Per arm, when the operator last used its joint inputs; the ghost shows only while recent.
        self._joints_used: dict[tuple[str, str], RecentUse] = {}
        #: Per arm, the joint slider values we wrote last. ! Writing a slider fires its callback
        #: too; a callback that finds these values was ours, not the operator's.
        self._written: dict[tuple[str, str], tuple[float, ...]] = {}

    # --- --- --- --- --- SETUP --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Build one panel with a tab per robot, holding inline tabs for its base and arms.

        Args:
            ctx: This plugin's context.
        """
        # Raw gui api, not ui(): ui() would leave an empty folder in the main panel.
        gui = ctx.view.gui
        panel = ctx.view.panel()
        for serial, robot in ctx.world.robots.items():
            # "a200-0806" -> "0806"
            with panel.add_tab(serial.split("-")[-1], icon=viser.Icon.ROBOT):
                parts = gui.add_tab_group()
                with parts.add_tab("Base", icon=viser.Icon.CAR):
                    self._bases.append(self._build_base(ctx, gui, serial, robot.base))
                for arm_name, arm in robot.arms.items():
                    with parts.add_tab(ARM_LABELS.get(arm_name, arm_name), icon=viser.Icon.ROBOT):
                        self._arms.append(self._build_arm(ctx, gui, serial, arm))
        panel.dock_right()
        panel.set_width(PANEL_WIDTH)
        # ? Built here: loading meshes is too slow for a tick.
        self._target_ghosts = robot_ghosts(ctx, ctx.config.robots, "target")

    def _build_base(self, ctx: PluginContext, gui: viser.GuiApi, serial: str,
                    base: BaseInterface) -> _BaseWidgets:
        """Build the widgets of one base.

        Args:
            ctx: This plugin's context.
            gui: The GUI api, inside the base's tab.
            serial: The robot's serial.
            base: The base to show and drive.

        Returns:
            _BaseWidgets: The handles.
        """
        status = gui.add_html("")
        velocity_inputs = partial(build_velocity_inputs, ctx, gui, serial, partial(self._on_hold, ctx, serial))
        panel, built = build_controller_panel(ctx, gui, base.controllers,
                                              {PLATFORM_VELOCITY_CONTROLLER: velocity_inputs})
        return _BaseWidgets(serial=serial, base=base, panel=panel, status=status,
                            speed=built[PLATFORM_VELOCITY_CONTROLLER])

    def _build_arm(self, ctx: PluginContext, gui: viser.GuiApi, serial: str,
                   arm: ArmInterface) -> _ArmWidgets:
        """Build the widgets of one arm and its end effector.

        Args:
            ctx: This plugin's context.
            gui: The GUI api, inside the arm's tab.
            serial: The robot's serial.
            arm: The arm to show and command.

        Returns:
            _ArmWidgets: The handles.
        """
        label = f"{serial} {arm.config.name}"
        key = (serial, arm.config.name)
        self._load_sliders[key] = None
        self._joints_used[key] = RecentUse(ctx.config.ghost_timeout)
        self._load_pose.add(key)

        status = gui.add_html("")
        hold = partial(self._hold_arm, serial, arm)
        joint_inputs = partial(build_joint_inputs, ctx, gui, label, arm, used=self._joints_used[key],
                               written=partial(self._written.get, key),
                               load=partial(self._request_joint_load, key),
                               start=partial(self._start_move, ctx, serial, arm, "move", self._execute_move),
                               hold=hold)
        compliance_inputs = partial(build_compliance_inputs, ctx, gui, label, arm,
                                    load=partial(self._load_pose.add, key),
                                    start=partial(self._start_move, ctx, serial, arm, "cartesian move",
                                                  self._stream_cartesian_move),
                                    hold=hold, apply_force=partial(self._apply_force, key, arm))
        panel, built = build_controller_panel(ctx, gui, arm.controllers, {
            SCALED_JOINT_TRAJECTORY_CONTROLLER: joint_inputs,
            CARTESIAN_COMPLIANCE_CONTROLLER: compliance_inputs,
            FREE_DRIVE_CONTROLLER: partial(build_free_drive_inputs, gui),
        })

        # Independent of the running controller.
        gui.add_html(section("sensor", SECTION_SENSOR))
        zero = gui.add_button_group("FT", ["Zero"], hint="Tares the current load. Only with nothing gripped.")
        zero.on_click(ctx.defer(f"zero FT {label}", arm.zero_ft_sensor))

        tool = None if arm.end_effector is None else build_tool_inputs(ctx, gui, label, arm)
        return _ArmWidgets(serial=serial, arm=arm, panel=panel, status=status, tool_status=tool,
                           joint=built[SCALED_JOINT_TRAJECTORY_CONTROLLER],
                           compliance=built[CARTESIAN_COMPLIANCE_CONTROLLER],
                           frames=add_frame_markers(ctx, serial, arm), arrows=add_force_arrows(ctx, serial, arm))

    def teardown(self, ctx: PluginContext) -> None:
        """At shutdown, stop any base this plugin was driving.

        Args:
            ctx: This plugin's context.
        """
        for widgets in self._bases:
            if widgets.serial in self._driving:
                widgets.base.send_twist(0.0, 0.0)

    # --- --- --- --- --- INTENTS (widget callbacks, run on the main thread) --- --- --- --- ---

    def _on_hold(self, ctx: PluginContext, serial: str, linear: float, angular: float) -> None:
        """Record that a hold button is still pressed.

        Args:
            ctx: This plugin's context.
            serial: The base's robot serial.
            linear: Sign of the linear speed the button asks for.
            angular: Sign of the angular speed the button asks for.
        """
        self._held[serial] = (linear, angular, ctx.now())

    def _request_joint_load(self, key: tuple[str, str], angles: np.ndarray | None) -> None:
        """Ask update to set one arm's joint sliders to `angles` (radians), or to the measured joints for None."""
        self._load_sliders[key] = angles

    def _apply_force(self, key: tuple[str, str], arm: ArmInterface, force: np.ndarray) -> None:
        """Send a target force (N, tool frame) to the arm and remember it if accepted."""
        if arm.send_target_wrench(force):
            self._applied_force[key] = force

    # --- --- --- --- --- EVERY TICK --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Stream the twist of every held base every tick (the base stops when cmd_vel goes quiet), and update arms.

        Args:
            ctx: This plugin's context.
        """
        now = ctx.now()
        for widgets in self._bases:
            held = self._held.get(widgets.serial)
            # Drop the hold if it timed out or the controller is no longer running.
            if held is not None and (now - held[2] > HOLD_TIMEOUT
                                     or not widgets.base.controllers.is_active(PLATFORM_VELOCITY_CONTROLLER)):
                del self._held[widgets.serial]
                held = None
            if held is not None:
                speed = widgets.speed.value
                widgets.base.send_twist(held[0] * speed * MAX_LINEAR_SPEED, held[1] * speed * MAX_ANGULAR_SPEED)
                self._driving.add(widgets.serial)
            elif widgets.serial in self._driving:
                # Just released: send one stop now.
                widgets.base.send_twist(0.0, 0.0)
                self._driving.discard(widgets.serial)
        for widgets in self._arms:
            key = (widgets.serial, widgets.arm.config.name)
            self._load_joint_sliders(widgets)
            self._load_pose_sliders(widgets)
            target = widgets.arm.from_husky(*pose_from_sliders(widgets.compliance.pose))
            force = np.array([slider.value for slider in widgets.compliance.force])
            self._markers[key] = place_markers(ctx, widgets.serial, widgets.arm, target, force,
                                               self._applied_force.get(key))

    def draw(self, ctx: PluginContext) -> None:
        """Copy every robot's state into the widgets.

        Args:
            ctx: This plugin's context.
        """
        now = ctx.now()
        for widgets in self._bases:
            show_only_running(widgets.panel)
            widgets.status.content = base_status(widgets.base, now)
        for widgets in self._arms:
            key = (widgets.serial, widgets.arm.config.name)
            show_only_running(widgets.panel)
            widgets.status.content = arm_status(widgets.arm, now)
            widgets.joint.plan.content = joint_plan_line(widgets.arm, widgets.joint.joints)
            widgets.compliance.pose_plan.content = pose_plan_line(widgets.arm, widgets.compliance.pose)
            draw_markers(widgets.frames, widgets.arrows, self._markers.get(key) or {})
            applied = self._applied_force.get(key)
            widgets.compliance.wrench_line.content = block(values(
                f"applied {numbers(applied, 3, 6, 1)} N ({TARGET_WRENCH_FRAME})"))
            if widgets.tool_status is not None:
                widgets.tool_status.content = tool_status(widgets.arm.state.end_effector, now)
        self._draw_ghosts(ctx)

    def _draw_ghosts(self, ctx: PluginContext) -> None:
        """Show each arm (with its tool) at its joint sliders while they are in use, even on top of the arm.

        Args:
            ctx: This plugin's context.
        """
        for serial, ghost in self._target_ghosts.items():
            joints = dict(ctx.kinematics.joints(serial))
            parts = []
            for widgets in self._arms:
                name = widgets.arm.config.name
                if widgets.serial != serial or not self._joints_used[(serial, name)].active:
                    continue  # another robot's arm, or sliders not in use
                parts.append(f"{name}_")
                target = np.radians([slider.value for slider in widgets.joint.joints])
                joints.update(zip((f"{name}_{joint}" for joint in UR_JOINT_NAMES), target))
            if parts:
                ghost.show(ctx.kinematics.base_pose(serial), joints, parts=tuple(parts))
            else:
                ghost.hide()

    def _load_pose_sliders(self, widgets: _ArmWidgets) -> None:
        """Set one arm's pose sliders to its TCP, if asked.

        Args:
            widgets: One arm's handles.
        """
        key = (widgets.serial, widgets.arm.config.name)
        state = widgets.arm.state
        if key not in self._load_pose or state.tcp_position is None:
            return  # not asked, or no TCP yet
        position, orientation = widgets.arm.to_husky(state.tcp_position, state.tcp_orientation)
        angles = Rotation.from_quat(orientation).as_euler("xyz", degrees=True)
        for slider, value in zip(widgets.compliance.pose, [*position, *angles]):
            slider.value = round(float(value) / slider.step) * slider.step
        self._load_pose.discard(key)

    def _load_joint_sliders(self, widgets: _ArmWidgets) -> None:
        """Set one arm's joint sliders, if asked.

        ! Call from update, not draw: draw is skipped while panels are frozen, leaving a stale target for Start.

        Args:
            widgets: One arm's handles.
        """
        key = (widgets.serial, widgets.arm.config.name)
        if key not in self._load_sliders:
            return
        # None means the measured joints, which need joint data first.
        angles = widgets.arm.joint_vector() if self._load_sliders[key] is None else self._load_sliders[key]
        if angles is not None:
            for slider, value in zip(widgets.joint.joints, np.degrees(angles)):
                slider.value = round(float(value) * 2) / 2  # the sliders' 0.5 deg step
            self._written[key] = tuple(slider.value for slider in widgets.joint.joints)
            del self._load_sliders[key]

    # --- --- --- --- --- MOVE TASKS --- --- --- --- ---

    def _start_move(self, ctx: PluginContext, serial: str, arm: ArmInterface, kind: str,
                    move: Callable[..., Coroutine[Any, Any, None]], *args: Any) -> None:
        """Start `move(ctx, serial, arm, *args)` as the arm's move task, unless one is already running.

        ! Joint and Cartesian moves share `_trajectory_tasks`: one motion per arm at a time.

        Args:
            ctx: This plugin's context.
            serial: The robot's serial.
            arm: The arm to move.
            kind: Task name prefix, e.g. "move".
            move: `_execute_move` or `_stream_cartesian_move`.
            *args: The move's target, after (ctx, serial, arm).
        """
        key = (serial, arm.config.name)
        running = self._trajectory_tasks.get(key)
        if running is not None and not running.done():
            ctx.log_warn(f"{serial} {arm.config.name}: a move is already running; Start ignored")
            return
        self._trajectory_tasks[key] = ctx.spawn(f"{kind} {serial} {arm.config.name}", move(ctx, serial, arm, *args))

    def _hold_arm(self, serial: str, arm: ArmInterface) -> None:
        """Hold the arm where it is and cancel its running move task.

        ! Holds even when no task of ours runs: a trajectory may come from elsewhere.

        Args:
            serial: The robot's serial.
            arm: The arm to hold.
        """
        arm.hold()
        running = self._trajectory_tasks.get((serial, arm.config.name))
        if running is not None and not running.done():
            running.cancel()

    async def _execute_move(self, ctx: PluginContext, serial: str, arm: ArmInterface, target: np.ndarray) -> None:
        """Send a smooth joint move and wait until the time is up and every joint is within ARRIVED_TOLERANCE.

        Args:
            ctx: This plugin's context.
            serial: The robot's serial, for messages.
            arm: The arm to move.
            target: Joint angles to move to, radians, UR_JOINT_NAMES order.
        """
        name = f"{serial} {arm.config.name}"
        duration = arm.send_joint_move(target)
        if duration is None:
            return  # the arm logged why it refused
        ctx.log_info(f"{name}: moving, {duration:.1f} s")
        end = ctx.now() + duration

        def arrived() -> bool:
            here = arm.joint_vector()
            return (ctx.now() >= end and here is not None
                    and bool(np.all(np.abs(here - target) < np.radians(ARRIVED_TOLERANCE))))

        try:
            await ctx.wait_until(arrived, timeout_s=duration + ARRIVE_TIMEOUT_MARGIN,
                                 description=f"{name} to arrive")
            ctx.log_info(f"{name}: arrived")
        except WaitTimeout as timeout:
            # Logged, not raised: a raising task counts as a plugin failure.
            ctx.log_warn(str(timeout))
        # ! Sliders are never touched here: only Current or Stow change them.

    async def _stream_cartesian_move(self, ctx: PluginContext, serial: str, arm: ArmInterface,
                                     position: np.ndarray, orientation: np.ndarray) -> None:
        """Move the TCP by sending the next point of a smooth `cartesian_move` each tick.

        ? Streamed because the compliance controller jumps straight to its target.

        Args:
            ctx: This plugin's context.
            serial: The robot's serial, for messages.
            arm: The arm to move.
            position: TCP target position, metres, arm base frame.
            orientation: TCP target orientation, quaternion (x, y, z, w).
        """
        name = f"{serial} {arm.config.name}"
        if arm.state.tcp_position is None:
            ctx.log_warn(f"{name}: no TCP pose yet, nothing sent")
            return
        duration, sample = cartesian_move(arm.state.tcp_position, arm.state.tcp_orientation,
                                          position, orientation)
        if arm.config.cartesian_test_mode:
            # ! Test mode: send one target at the TCP so the checks run; the move is not streamed.
            arm.send_cartesian_target(*sample(0.0))
            ctx.log_warn(f"{name}: TEST MODE, planned cartesian move of {duration:.1f} s to "
                         f"{np.round(position, 3)} m was not streamed")
            return
        ctx.log_info(f"{name}: cartesian move, {duration:.1f} s")
        start = ctx.now()
        while True:
            elapsed = ctx.now() - start
            if not arm.send_cartesian_target(*sample(elapsed)):
                # The arm logged why; hold so it settles where it is.
                ctx.log_warn(f"{name}: cartesian move stopped early")
                arm.hold()
                return
            if elapsed >= duration:
                break
            await ctx.next_tick()
        ctx.log_info(f"{name}: cartesian move done")
