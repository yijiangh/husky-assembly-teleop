"""
The robot control panel: a tab per robot, with inline tabs per part showing state and basic control.

On the phone, a Control tab per robot: the base's D-pad, and per arm its gripper and free drive while held.

* The pattern to copy when writing a plugin:
  - `setup` builds every widget once; `draw` only copies state into them.
  - ! Widget callbacks go through `ctx.defer` / `ctx.defer_value`: viser calls them
    on its own threads, where touching robots or the scene is not safe.
  - `update` does per-tick work; anything longer than a tick is a `ctx.spawn` task.
* Robots are commanded and read only through their interfaces; no topic names here.
! While another plugin holds any robot for an automatic motion (a lease, `shared/execution/interlock.py`), every
  motion command here, to any robot, is refused with a log line; never Hold, the tools' Stop or a zero force. Hold
  on the held robot also cancels that motion. What sends every tick checks before every send: a lease taken
  meanwhile ends the Cartesian stream (hold), the D-pad (one stop) and free drive (no requests, Keep on unticked,
  back to the joint controller).
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
from ...shared.execution.interlock import held, lease_of, refuse_while_held
from ...ui.ghost import RecentUse, RobotGhost, robot_ghosts
from ...ui.joint_sliders import joint_slider_angles, set_joint_sliders
from ...ui.phone import big_buttons, phone_text
from ...ui.style import SECTION_SENSOR, block, numbers, section, values
from .arm_panel import (FREE_DRIVE_ICON, ComplianceInputs, JointInputs, build_compliance_inputs,
                        build_free_drive_inputs, build_joint_inputs, build_phone_arm_inputs, build_tool_inputs,
                        joint_plan_line, pose_from_sliders, pose_plan_line)
from .dpad import HOLD_TIMEOUT, MAX_ANGULAR_SPEED, MAX_LINEAR_SPEED, PHONE_DPAD_STYLE, build_velocity_inputs
from .markers import add_force_arrows, add_frame_markers, draw_markers, place_markers
from .status import (ControllerPanel, arm_status, base_status, build_controller_panel, phone_arm_status,
                     phone_base_status, show_only_running, tool_status)

#: A move has arrived when every joint is this close to its target, degrees.
ARRIVED_TOLERANCE = 0.5
#: Seconds past the planned duration before an unfinished move is reported as timed out.
ARRIVE_TIMEOUT_MARGIN = 5.0
#: Starting panel width, pixels. Fits the arm's joint row, the widest line.
PANEL_WIDTH = 420
#: Seconds to wait before asking again after a refused free drive switch, so a dead service is not asked every tick.
FREE_DRIVE_RETRY = 1.0


@dataclass
class _BaseWidgets:
    """Handles for one base."""

    serial: str
    base: BaseInterface
    panel: ControllerPanel
    status: viser.GuiHtmlHandle
    speed: viser.GuiSliderHandle
    #: The same on the phone's Control tab, set once that tab is built.
    phone_status: viser.GuiHtmlHandle | None = None
    phone_speed: viser.GuiSliderHandle | None = None


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
    #: Keep on, in the free drive controller's folder.
    free_drive_keep: viser.GuiCheckboxHandle | None = None
    #: The arm's status line on the phone's Control tab, set once that tab is built.
    phone_status: viser.GuiHtmlHandle | None = None


@register
class RobotControlPlugin(HuskyPlugin):
    """Shows each robot's state and gives basic control over it."""

    name = "robot_control"

    def __init__(self):
        """Start with no widgets; setup builds them."""
        self._ctx: PluginContext | None = None
        self._bases: list[_BaseWidgets] = []
        self._arms: list[_ArmWidgets] = []

        #: Bases whose twist was streamed last tick, so update sends one stop on release.
        self._driving: set[str] = set()

        #: Last hold report per base serial: (linear sign, angular sign, arrival time, from the phone).
        self._held: dict[str, tuple[float, float, float, bool]] = {}

        #: Last report of each arm's held free drive button, by (serial, arm name): (arrival time, from the phone).
        self._free_drive_held: dict[tuple[str, str], tuple[float, bool]] = {}
        #: Arms this plugin switched to free drive for a held phone button, so a release switches them back.
        self._free_driving: set[tuple[str, str]] = set()
        #: Per arm, the time before which a refused free drive switch, or a switch back under a lease, is not asked
        #: again.
        self._free_drive_retry: dict[tuple[str, str], float] = {}

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
        #: Per (serial, command) refused every tick while a robot is held (the D-pad, free drive), the lease logged.
        self._refusals_said: dict[tuple[str, str], object] = {}

    # --- --- --- --- --- SETUP --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Build one panel with a tab per robot, holding inline tabs for its base and arms, and the phone's Control tab.

        Args:
            ctx: This plugin's context.
        """
        self._ctx = ctx
        # Raw gui api, not ui(): ui() would leave an empty folder in the main panel.
        gui = ctx.view.gui
        panel = ctx.view.panel()
        for serial, robot in ctx.world.robots.items():
            # "a200-0806" -> "0806"
            with panel.add_tab(serial.split("-")[-1], icon=viser.Icon.ROBOT):
                parts = gui.add_tab_group()
                with parts.add_tab("Base", icon=viser.Icon.CAR):
                    self._bases.append(self._build_base(ctx, gui, serial, robot.base))
                for arm in robot.arms.values():
                    with parts.add_tab(arm.config.label, icon=viser.Icon.ROBOT):
                        self._arms.append(self._build_arm(ctx, gui, serial, arm))
        panel.dock_right()
        panel.set_width(PANEL_WIDTH)
        # * The phone's Control tab: a sub-tab per robot with its own speed slider and the same D-pad.
        #   Below it, a folder per arm: Open and Close, and free drive while held.
        with ctx.view.phone("Control", viser.Icon.DEVICE_GAMEPAD_2) as phone:
            phone.add_html(PHONE_DPAD_STYLE + f"<style>{big_buttons([FREE_DRIVE_ICON], 64, 28)}</style>")
            robots = phone.add_tab_group()
            for widgets in self._bases:
                with robots.add_tab(widgets.serial.split("-")[-1], icon=viser.Icon.ROBOT):
                    widgets.phone_status = phone.add_html("")
                    widgets.phone_speed = build_velocity_inputs(
                        ctx, phone, widgets.serial, partial(self._on_hold, ctx, widgets.serial, True))
                    for arm_widgets in (arm for arm in self._arms if arm.serial == widgets.serial):
                        name = arm_widgets.arm.config.name
                        with phone.add_folder(arm_widgets.arm.config.label):
                            arm_widgets.phone_status = build_phone_arm_inputs(
                                ctx, phone, f"{widgets.serial} {name}", arm_widgets.arm,
                                partial(self._on_free_drive_hold, ctx, (widgets.serial, name), True),
                                partial(self._allowed, widgets.serial))
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
        velocity_inputs = partial(build_velocity_inputs, ctx, gui, serial, partial(self._on_hold, ctx, serial, False))
        panel, built = build_controller_panel(ctx, gui, base.controllers,
                                              {PLATFORM_VELOCITY_CONTROLLER: velocity_inputs},
                                              partial(self._allowed, serial))
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
            FREE_DRIVE_CONTROLLER: partial(build_free_drive_inputs, ctx, gui, label,
                                           partial(self._on_free_drive_hold, ctx, key, False)),
        }, partial(self._allowed, serial))

        # Independent of the running controller.
        gui.add_html(section("sensor", SECTION_SENSOR))
        zero = gui.add_button_group("FT", ["Zero"], hint="Tares the current load. Only with nothing gripped.")
        zero.on_click(ctx.defer(f"zero FT {label}", lambda: self._allowed(serial, f"{arm.config.name} FT Zero")
                                and arm.zero_ft_sensor()))

        allowed = partial(self._allowed, serial)
        tool = None if arm.end_effector is None else build_tool_inputs(ctx, gui, label, arm, allowed)
        return _ArmWidgets(serial=serial, arm=arm, panel=panel, status=status, tool_status=tool,
                           joint=built[SCALED_JOINT_TRAJECTORY_CONTROLLER],
                           compliance=built[CARTESIAN_COMPLIANCE_CONTROLLER],
                           free_drive_keep=built[FREE_DRIVE_CONTROLLER],
                           frames=add_frame_markers(ctx, serial, arm), arrows=add_force_arrows(ctx, serial, arm))

    def teardown(self, ctx: PluginContext) -> None:
        """At shutdown, stop any base this plugin was driving, and hold any arm it left a force on.

        Moves still running hold their arm as they are cancelled.

        Args:
            ctx: This plugin's context.
        """
        for widgets in self._bases:
            if widgets.serial in self._driving:
                widgets.base.send_twist(0.0, 0.0)
        for widgets in self._arms:
            if (widgets.serial, widgets.arm.config.name) in self._applied_force:
                self._hold_arm(widgets.serial, widgets.arm)

    # --- --- --- --- --- INTENTS (widget callbacks, run on the main thread) --- --- --- --- ---

    def _on_hold(self, ctx: PluginContext, serial: str, phone: bool, linear: float, angular: float) -> None:
        """Record that a hold button is still pressed.

        Args:
            ctx: This plugin's context.
            serial: The base's robot serial.
            phone: Whether the button is on the phone, whose speed slider then applies.
            linear: Sign of the linear speed the button asks for.
            angular: Sign of the angular speed the button asks for.
        """
        self._held[serial] = (linear, angular, ctx.now(), phone)

    def _on_free_drive_hold(self, ctx: PluginContext, key: tuple[str, str], phone: bool) -> None:
        """Record that an arm's free drive button is still held; on the phone it also switches the controller."""
        self._free_drive_held[key] = (ctx.now(), phone)

    def _request_joint_load(self, key: tuple[str, str], angles: np.ndarray | None) -> None:
        """Ask update to set one arm's joint sliders to `angles` (radians), or to the measured joints for None."""
        self._load_sliders[key] = angles

    def _apply_force(self, key: tuple[str, str], arm: ArmInterface, force: np.ndarray) -> None:
        """Send a target force (N, tool frame) to the arm and remember it if accepted; while a robot is held, only zero.

        ? Zero removes a force, like a Stop: never refused.
        """
        if (not np.any(force) or self._allowed(key[0], f"{arm.config.name} force")) and arm.send_target_wrench(force):
            self._applied_force[key] = force

    def _allowed(self, serial: str, what: str) -> bool:
        """Whether a motion command may go out now: not while any robot is held (the refusal is logged).

        Args:
            serial: The robot it is for.
            what: The command, for the log, e.g. "tool Open".
        """
        return not refuse_while_held(self._ctx.world, f"{serial} {what}", self._ctx.log_warn)

    def _held_by_other(self, serial: str, what: str) -> bool:
        """Whether any robot is held, for a command refused every tick; the refusal is logged once per lease.

        Args:
            serial: The robot the command is for.
            what: The command, for the log, e.g. "D-pad".
        """
        found = held(self._ctx.world)
        if found is not None and self._refusals_said.get((serial, what)) is not found[1]:
            self._refusals_said[(serial, what)] = found[1]
            refuse_while_held(self._ctx.world, f"{serial} {what}", self._ctx.log_warn)
        return found is not None

    # --- --- --- --- --- EVERY TICK --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Stream the twist of every held base every tick (the base stops when cmd_vel goes quiet), and update arms.

        Args:
            ctx: This plugin's context.
        """
        now = ctx.now()
        for widgets in self._bases:
            held = self._held.get(widgets.serial)
            # Drop the hold if it timed out, the controller is no longer running, or another plugin holds the robot.
            if held is not None and (now - held[2] > HOLD_TIMEOUT
                                     or not widgets.base.controllers.is_active(PLATFORM_VELOCITY_CONTROLLER)
                                     or self._held_by_other(widgets.serial, "D-pad")):
                del self._held[widgets.serial]
                held = None
            if held is not None:
                speed = (widgets.phone_speed if held[3] else widgets.speed).value
                widgets.base.send_twist(held[0] * speed * MAX_LINEAR_SPEED, held[1] * speed * MAX_ANGULAR_SPEED)
                self._driving.add(widgets.serial)
            elif widgets.serial in self._driving:
                # Just released: send one stop now.
                widgets.base.send_twist(0.0, 0.0)
                self._driving.discard(widgets.serial)
        self._update_free_drive(now)
        for widgets in self._arms:
            key = (widgets.serial, widgets.arm.config.name)
            self._load_joint_sliders(widgets)
            self._load_pose_sliders(widgets)
            target = widgets.arm.from_husky(*pose_from_sliders(widgets.compliance.pose))
            force = np.array([slider.value for slider in widgets.compliance.force])
            self._markers[key] = place_markers(ctx, widgets.serial, widgets.arm, target, force,
                                               self._applied_force.get(key))

    def _update_free_drive(self, now: float) -> None:
        """Keep free drive on while a free drive button is held or Keep on is ticked; the phone's button also switches.

        - Laptop: the Free controller button switches; holding its button, or Keep on, frees the arm.
        - Phone: holding switches to free drive and frees the arm; on release the joint controller holds it.
        ! On release, switch back only while free drive still runs: after Stop all or a switch on the laptop, a
          release must not start a controller. If the switch back fails, free drive still ends within 1 s, since
          the requests stop (`ArmInterface.request_free_drive`).
        ! While any robot is held for an automatic motion: no requests, Keep on unticked, and every arm in free drive
          switched back to the joint controller; logged once per lease.

        Args:
            now: This tick's time, as the hold reports use.
        """
        for widgets in self._arms:
            key = (widgets.serial, widgets.arm.config.name)
            controllers = widgets.arm.controllers
            last = self._free_drive_held.get(key)
            held = last is not None and now - last[0] <= HOLD_TIMEOUT
            keep = widgets.free_drive_keep
            if keep is not None and keep.value and controllers.state.requested != FREE_DRIVE_CONTROLLER:
                keep.value = False  # another controller was chosen: Keep on must be ticked again
            wanted = held or (keep is not None and keep.value)
            # ! While any robot is held for an automatic motion, no arm is freed: no requests, Keep on unticked (it
            #   would free the arm again the moment the motion ends), and the joint controller holds the arm.
            taken = (wanted or key in self._free_driving or controllers.is_active(FREE_DRIVE_CONTROLLER)) and \
                self._held_by_other(widgets.serial, f"{widgets.arm.config.name} free drive")
            if taken and keep is not None and keep.value:
                keep.value = False
            if wanted and not taken:
                widgets.arm.request_free_drive()
            # Switch for the phone only once a pending switch is answered, and not straight after a refused one.
            phone_held = held and last[1] and not taken
            if controllers.state.switch_in_flight is not None or now < self._free_drive_retry.get(key, now):
                continue
            if phone_held and key not in self._free_driving:
                switched = controllers.switch(FREE_DRIVE_CONTROLLER)
                if switched:
                    self._free_driving.add(key)
            elif not phone_held and (key in self._free_driving or taken):
                # Released, or a robot was taken for an automatic motion: the joint controller holds the arm.
                switched = (not controllers.is_active(FREE_DRIVE_CONTROLLER)
                            or controllers.switch(SCALED_JOINT_TRAJECTORY_CONTROLLER))
                if switched:
                    self._free_driving.discard(key)
            else:
                continue
            # ? Under a lease the switch back is asked again only after a while: a failed one leaves free drive running.
            if not switched or (taken and controllers.is_active(FREE_DRIVE_CONTROLLER)):
                self._free_drive_retry[key] = now + FREE_DRIVE_RETRY

    def draw(self, ctx: PluginContext) -> None:
        """Copy every robot's state into the widgets.

        Args:
            ctx: This plugin's context.
        """
        now = ctx.now()
        for widgets in self._bases:
            show_only_running(widgets.panel)
            widgets.status.content = base_status(widgets.base, now)
            if widgets.phone_status is not None:
                widgets.phone_status.content = phone_text(phone_base_status(widgets.base))
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
            if widgets.phone_status is not None:
                widgets.phone_status.content = phone_text(phone_arm_status(widgets.arm))
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
                target = joint_slider_angles(widgets.joint.joints)
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
            set_joint_sliders(widgets.joint.joints, angles)
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
        if not self._allowed(serial, f"{arm.config.name} {kind}"):
            return
        running = self._trajectory_tasks.get(key)
        if running is not None and not running.done():
            ctx.log_warn(f"{serial} {arm.config.name}: a move is already running; Start ignored")
            return
        self._trajectory_tasks[key] = ctx.spawn(f"{kind} {serial} {arm.config.name}", move(ctx, serial, arm, *args))

    def _hold_arm(self, serial: str, arm: ArmInterface) -> None:
        """Hold the arm where it is and cancel its running move task, and any automatic motion holding the robot.

        ! Holds even when no task of ours runs: a trajectory may come from elsewhere.

        Args:
            serial: The robot's serial.
            arm: The arm to hold.
        """
        arm.hold()
        self._applied_force.pop((serial, arm.config.name), None)  # hold zeroes the target wrench
        running = self._trajectory_tasks.get((serial, arm.config.name))
        if running is not None and not running.done():
            running.cancel()
        lease = lease_of(self._ctx.world.robots[serial])
        if lease is not None:
            # * Its own error handling holds every arm of the robot.
            self._ctx.log_warn(f"Hold on {serial} {arm.config.name}: cancelling {lease.holder} ({lease.label})")
            lease.stop()

    async def _execute_move(self, ctx: PluginContext, serial: str, arm: ArmInterface, target: np.ndarray) -> None:
        """Send a smooth joint move and wait until the time is up and every joint is within ARRIVED_TOLERANCE.

        ! Cancelled (Hold, Stop all, the plugin stopping, shutdown), it holds the arm: the trajectory would run on.

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
        except asyncio.CancelledError:
            arm.hold()
            raise
        # ! Sliders are never touched here: only Current or Stow change them.

    async def _stream_cartesian_move(self, ctx: PluginContext, serial: str, arm: ArmInterface,
                                     position: np.ndarray, orientation: np.ndarray) -> None:
        """Move the TCP by sending the next point of a smooth `cartesian_move` each tick.

        ? Streamed because the compliance controller jumps straight to its target.
        ! Cancelled, it holds the arm at its TCP.

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
        try:
            while True:
                elapsed = ctx.now() - start
                found = held(ctx.world)
                if found is not None:
                    # ! A robot was taken for an automatic motion mid-stream: one motion at a time, so stop here.
                    arm.hold()
                    ctx.log_warn(f"{name}: cartesian move stopped and held: {found[0]} is held by "
                                 f"{found[1].holder} ({found[1].label}), one motion at a time")
                    return
                if not arm.send_cartesian_target(*sample(elapsed)):
                    # The arm logged why; hold so it settles where it is.
                    ctx.log_warn(f"{name}: cartesian move stopped early")
                    arm.hold()
                    return
                if elapsed >= duration:
                    break
                await ctx.next_tick()
        except asyncio.CancelledError:
            arm.hold()
            raise
        ctx.log_info(f"{name}: cartesian move done")
