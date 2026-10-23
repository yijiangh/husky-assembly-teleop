"""
The robot control panel: the side panel's Control tab, holding a tab per robot with a tab per part.

Each part's tab shows its state and basic control. Arms jog joint by joint; moving to a pose is the arm planner's job.

On the phone, a Control tab per robot: the base's D-pad, and per arm its gripper and free drive while held.

* The pattern to copy when writing a plugin:
  - `setup` builds every widget once; `draw` only copies state into them.
  - ! Widget callbacks go through `ctx.defer` / `ctx.defer_value`: viser calls them
    on its own threads, where touching robots or the scene is not safe.
  - `update` does per-tick work; anything longer than a tick is a `ctx.spawn` task.
* Robots are commanded and read only through their interfaces; no topic names here.
* A robot is leased while in use (`teleop_lease.py`): a jog or drive is refused while another plugin holds it.
* Base Clearance (ticked by default): while driving, the base is soft stopped once it comes too close to anything
  in the scene (`ctx.clearance`).
* ! Every button that does nothing says why: a refusal chip on its part's status line (`_refuse`, `refusal_chip`).
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from functools import partial
from typing import Any, Callable, Coroutine

import numpy as np
import viser
from scipy.spatial.transform import Rotation

from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...robot_interface.arm import (CARTESIAN_COMPLIANCE_CONTROLLER, FREE_DRIVE_CONTROLLER, JOG_MAX_SPEED,
                                    MOVING_WITHIN, SCALED_JOINT_TRAJECTORY_CONTROLLER, TARGET_WRENCH_FRAME,
                                    ArmInterface, cartesian_move, joint_jog)
from ...robot_interface.base import PLATFORM_VELOCITY_CONTROLLER, BaseInterface
from ...robot_interface.end_effectors import EndEffectorState, RobotiqState, ScaffoldingV3, ScaffoldingV3State
from ...robot_interface.refusal import Refusal
from ...robot_interface.soft_stop import MotionRefused
from ...ui.phone import big_buttons, phone_text
from ...ui.style import REFUSAL_SHOWN, SECTION_SENSOR, block, numbers, section, values
from ...shared.teleop.dpad import HOLD_TIMEOUT, PHONE_DPAD_STYLE, Hold, build_velocity_inputs
from ...world.clearance import ALL_ARMS, STOP_DISTANCE, ClearanceRefused
from scaffolding_assembly_core.geometry import Pose
from .arm_panel import (FREE_DRIVE_ICON, ComplianceInputs, build_compliance_inputs, build_free_drive_inputs,
                        build_jog_inputs, build_phone_arm_inputs, build_tool_inputs, pose_from_sliders,
                        pose_plan_line)
from .markers import (add_force_arrows, add_frame_markers, add_target_gizmo, draw_markers, place_markers,
                      target_from_world)
from .status import (ControllerPanel, arm_status, base_status, build_controller_panel, phone_arm_status,
                     phone_base_status, refusal_chip, show_only_running, tool_status)
from .teleop_lease import TeleopLeases

#: Seconds to wait before asking again after a refused free drive switch, so a dead service is not asked every tick.
FREE_DRIVE_RETRY = 1.0
#: The part name of a robot's base in refusal keys; arms use their own names.
BASE = "base"


@dataclass
class _BaseWidgets:
    """Handles for one base."""

    serial: str
    base: BaseInterface
    panel: ControllerPanel
    status: viser.GuiHtmlHandle
    speed: viser.GuiSliderHandle
    #: Clearance: while ticked, driving soft stops the base once it comes too close to anything (`_guard_base`).
    clearance: viser.GuiCheckboxHandle | None = None
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
    #: The jog speed slider, a fraction of JOG_MAX_SPEED.
    jog_speed: viser.GuiSliderHandle
    compliance: ComplianceInputs
    frames: dict[str, viser.FrameHandle]
    arrows: dict[str, viser.ArrowsHandle]
    gizmo: viser.TransformControlsHandle
    #: Keep on, in the free drive controller's folder.
    free_drive_keep: viser.GuiCheckboxHandle | None = None
    #: The arm's status line on the phone's Control tab, set once that tab is built.
    phone_status: viser.GuiHtmlHandle | None = None


@register
class RobotControlPlugin(HuskyPlugin):
    """Shows each robot's state and gives basic control over it."""

    name = "robot_control"
    tab_path = ("Control",)
    icon = viser.Icon.DEVICE_GAMEPAD_2

    def __init__(self):
        """Start with no widgets; setup builds them."""
        self._bases: list[_BaseWidgets] = []
        self._arms: list[_ArmWidgets] = []

        #: Bases whose twist was streamed last tick, so update sends one stop on release.
        self._driving: set[str] = set()

        #: Last hold report per base serial: (linear sign, angular sign, arrival time, from the phone).
        self._held: dict[str, Hold] = {}

        #: Last report of each arm's held free drive button, by (serial, arm name): (arrival time, from the phone).
        self._free_drive_held: dict[tuple[str, str], tuple[float, bool]] = {}
        #: Arms this plugin switched to free drive for a held phone button, so a release switches them back.
        self._free_driving: set[tuple[str, str]] = set()
        #: Per arm, the time before which a refused free drive switch is not asked again.
        self._free_drive_retry: dict[tuple[str, str], float] = {}

        #: Last report of each arm's held jog button, by (serial, arm name): (joint index, direction, arrival time).
        self._jog_held: dict[tuple[str, str], tuple[int, float, float]] = {}
        #: Per arm, the press last answered: (joint index, direction, whether a jog was sent), until released.
        self._jogging: dict[tuple[str, str], tuple[int, float, bool]] = {}

        #: Running Cartesian move task per arm, keyed by (serial, arm name).
        self._trajectory_tasks: dict[tuple[str, str], asyncio.Task] = {}

        #: Arms whose pose sliders are set to the measured TCP on the next update.
        self._load_pose: set[tuple[str, str]] = set()
        #: Per arm, the gizmo's last dragged world pose, applied to the pose sliders on the next update.
        self._dragged: dict[tuple[str, str], Pose] = {}

        #: Target force last sent to each arm, N; the controller keeps applying it.
        self._applied_force: dict[tuple[str, str], np.ndarray] = {}

        #: This tick's 3D marker poses and force-arrow segments per arm; None without the compliance controller.
        self._markers: dict[tuple[str, str], dict[str, tuple] | None] = {}

        #: The robots this plugin holds the lease of while the operator uses them.
        self._leases = TeleopLeases()
        #: Per (serial, part), the last of this plugin's commands that did nothing, and why; part is BASE or an arm.
        self._refusals: dict[tuple[str, str], Refusal] = {}

        #: Per base serial, the task keeping a clearance while this plugin drives it, and why the last one tripped.
        self._base_guards: dict[str, asyncio.Task] = {}
        self._tripped: dict[str, str] = {}

    # --- --- --- --- --- SETUP --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Build the Control tab with a tab per robot, holding tabs for its base and arms, and the phone's Control tab.

        Args:
            ctx: This plugin's context.
        """
        with ctx.view.ui() as gui:
            robots = gui.add_tab_group()
            for serial, robot in ctx.world.robots.items():
                # "a200-0806" -> "0806"
                with robots.add_tab(serial.split("-")[-1], icon=viser.Icon.ROBOT):
                    parts = gui.add_tab_group()
                    with parts.add_tab("Base", icon=viser.Icon.CAR):
                        self._bases.append(self._build_base(ctx, gui, serial, robot.base))
                    for arm in robot.arms.values():
                        with parts.add_tab(arm.config.label, icon=viser.Icon.CRANE):
                            self._arms.append(self._build_arm(ctx, gui, serial, arm))
        # * The phone's Control tab: a sub-tab per robot with its own speed slider and the same D-pad.
        #   Below it, a folder per arm: Open and Close, and free drive while held.
        with ctx.view.phone("Control", viser.Icon.DEVICE_GAMEPAD_2) as phone:
            phone.add_html(PHONE_DPAD_STYLE + f"<style>{big_buttons([FREE_DRIVE_ICON], 64, 28)}</style>")
            robots = phone.add_tab_group()
            for widgets in self._bases:
                with robots.add_tab(widgets.serial.split("-")[-1], icon=viser.Icon.ROBOT):
                    widgets.phone_status = phone.add_html("")
                    widgets.phone_speed = build_velocity_inputs(
                        ctx, phone, widgets.serial, partial(self._on_hold, widgets.serial))
                    for arm_widgets in (arm for arm in self._arms if arm.serial == widgets.serial):
                        name = arm_widgets.arm.config.name
                        with phone.add_folder(arm_widgets.arm.config.label):
                            arm_widgets.phone_status = build_phone_arm_inputs(
                                ctx, phone, f"{widgets.serial} {name}", arm_widgets.arm,
                                partial(self._on_free_drive_hold, ctx, (widgets.serial, name), True),
                                partial(self._leased, ctx, (widgets.serial, name)))

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

        def velocity_inputs() -> tuple[viser.GuiSliderHandle, viser.GuiCheckboxHandle]:
            speed = build_velocity_inputs(ctx, gui, serial, partial(self._on_hold, serial))
            clearance = gui.add_checkbox(
                "Clearance", initial_value=True,
                hint=f"While driving, soft stop the base once it comes within {STOP_DISTANCE * 100:.0f} cm of anything "
                     f"in the scene; an arm whose joints are not known counts as stowed. Untick to drive out of a "
                     f"tight spot; the phone's D-pad follows it too.")
            return speed, clearance

        panel, built = build_controller_panel(ctx, gui, base.controllers,
                                              {PLATFORM_VELOCITY_CONTROLLER: velocity_inputs},
                                              partial(self._leased, ctx, (serial, BASE)))
        speed, clearance = built[PLATFORM_VELOCITY_CONTROLLER]
        return _BaseWidgets(serial=serial, base=base, panel=panel, status=status, speed=speed, clearance=clearance)

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
        self._load_pose.add(key)

        status = gui.add_html("")
        hold = partial(self._hold_arm, serial, arm)
        jog_inputs = partial(build_jog_inputs, ctx, gui, label, partial(self._on_jog_hold, ctx, key), hold)
        compliance_inputs = partial(build_compliance_inputs, ctx, gui, label, arm,
                                    load=partial(self._load_pose.add, key),
                                    start=partial(self._start_move, ctx, serial, arm, "cartesian move",
                                                  self._stream_cartesian_move),
                                    hold=hold, apply_force=partial(self._apply_force, ctx, key, arm))
        panel, built = build_controller_panel(ctx, gui, arm.controllers, {
            SCALED_JOINT_TRAJECTORY_CONTROLLER: jog_inputs,
            CARTESIAN_COMPLIANCE_CONTROLLER: compliance_inputs,
            FREE_DRIVE_CONTROLLER: partial(build_free_drive_inputs, ctx, gui, label,
                                           partial(self._on_free_drive_hold, ctx, key, False)),
        }, partial(self._leased, ctx, key))

        # Independent of the running controller.
        gui.add_html(section("sensor", SECTION_SENSOR))
        zero = gui.add_button_group("FT", ["Zero"], hint="Tares the current load. Only with nothing gripped.")
        zero.on_click(ctx.defer(f"zero FT {label}", arm.zero_ft_sensor))

        tool = None if arm.end_effector is None else build_tool_inputs(ctx, gui, label, arm,
                                                                       partial(self._leased, ctx, key))
        return _ArmWidgets(serial=serial, arm=arm, panel=panel, status=status, tool_status=tool,
                           jog_speed=built[SCALED_JOINT_TRAJECTORY_CONTROLLER],
                           compliance=built[CARTESIAN_COMPLIANCE_CONTROLLER],
                           free_drive_keep=built[FREE_DRIVE_CONTROLLER],
                           frames=add_frame_markers(ctx, serial, arm), arrows=add_force_arrows(ctx, serial, arm),
                           gizmo=add_target_gizmo(ctx, serial, arm, partial(self._dragged.__setitem__, key)))

    def teardown(self, ctx: PluginContext) -> None:
        """At shutdown, stop any base this plugin was driving, and hold any arm it jogs or left a force on.

        Moves still running hold their arm as they are cancelled.

        Args:
            ctx: This plugin's context.
        """
        for widgets in self._bases:
            if widgets.serial in self._driving:
                widgets.base.send_twist(0.0, 0.0)
        for widgets in self._arms:
            key = (widgets.serial, widgets.arm.config.name)
            if key in self._applied_force or (key in self._jogging and self._jogging[key][2]):
                self._hold_arm(widgets.serial, widgets.arm)

    # --- --- --- --- --- INTENTS (widget callbacks, run on the main thread) --- --- --- --- ---

    def _on_hold(self, serial: str, hold: Hold) -> None:
        """Record that a hold button of base `serial` is still pressed (desktop or phone: its own speed applies)."""
        self._held[serial] = hold

    def _on_free_drive_hold(self, ctx: PluginContext, key: tuple[str, str], phone: bool) -> None:
        """Record that an arm's free drive button is still held; on the phone it also switches the controller."""
        self._free_drive_held[key] = (ctx.now(), phone)

    def _on_jog_hold(self, ctx: PluginContext, key: tuple[str, str], joint: int, direction: float) -> None:
        """Record that an arm's jog button is still held."""
        self._jog_held[key] = (joint, direction, ctx.now())

    def _apply_force(self, ctx: PluginContext, key: tuple[str, str], arm: ArmInterface, force: np.ndarray) -> None:
        """Send a target force (N, tool frame) to the arm and remember it if accepted.

        ? Zero goes out as `clear_target_wrench`, which first sets the target at the TCP so the arm stays where it is.
        """
        send = arm.clear_target_wrench if not np.any(force) else partial(arm.send_target_wrench, force)
        if self._leased(ctx, key, send):
            self._applied_force[key] = force

    def _leased(self, ctx: PluginContext, key: tuple[str, str], command: Callable[[], Any]) -> Any:
        """Run `command` under this plugin's lease of robot `key[0]`, leasing it first if free.

        Returns:
            Any: What `command` returned; None if refused (no lease, soft stop), which `_refuse` then shows.
        """
        try:
            with self._leases.use(ctx, key[0]):
                return command()
        except MotionRefused as refused:
            self._refuse(ctx, key, refused.label, str(refused))
            return None

    def _refuse(self, ctx: PluginContext, key: tuple[str, str], label: str, reason: str) -> None:
        """Show on part `key`'s status line that a button did nothing, and why; also log it.

        Args:
            ctx: This plugin's context.
            key: (serial, part): BASE or an arm's name.
            label: Short chip text, e.g. "jog refused".
            reason: The whole reason, shown on hover.
        """
        last = self._refusals.get(key)
        if last is None or last.reason != reason or ctx.now() - last.time > REFUSAL_SHOWN:
            ctx.log_warn(f"{key[0]} {key[1]}: {reason}")  # once, not every tick while a button stays held
        self._refusals[key] = Refusal(label, reason, ctx.now())

    def _drop_reason(self, ctx: PluginContext, serial: str, soft_stopped: bool) -> tuple[str, str] | None:
        """Why a held button on robot `serial` does nothing now, as (label, reason): soft stop or another's lease."""
        if soft_stopped and serial in self._tripped:
            return "too close", f"{self._tripped[serial]}: soft stopped; drive clear after Resume, or untick Clearance"
        if soft_stopped:
            return "soft stopped", f"robot {serial} is soft stopped; press Resume first"
        taken = self._leases.taken(ctx, serial)
        return None if taken is None else ("lease refused", taken)

    # --- --- --- --- --- EVERY TICK --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Stream the twist of every held base every tick (the base stops when cmd_vel goes quiet), and update arms.

        Args:
            ctx: This plugin's context.
        """
        now = ctx.now()
        for widgets in self._bases:
            if not widgets.base.soft_stopped:
                self._tripped.pop(widgets.serial, None)
            held = self._held.get(widgets.serial)
            # Drop the hold if it timed out, the robot was soft stopped or leased, or the controller no longer runs.
            dropped = None if held is None else self._drop_reason(ctx, widgets.serial, widgets.base.soft_stopped)
            if dropped is None and held is not None \
                    and not widgets.base.controllers.is_active(PLATFORM_VELOCITY_CONTROLLER):
                dropped = "drive refused", "the velocity controller is not running; switch to Vel"
            if held is not None and (held.released(now) or dropped is not None):
                if dropped is not None:
                    self._refuse(ctx, (widgets.serial, BASE), *dropped)
                del self._held[widgets.serial]
                held = None
            if held is not None:
                self._keep_clear(ctx, widgets)
                self._leased(ctx, (widgets.serial, BASE), lambda: widgets.base.send_twist(*held.twist()))
                self._driving.add(widgets.serial)
            elif widgets.serial in self._driving:
                # Just released: send one stop now.
                widgets.base.send_twist(0.0, 0.0)
                self._driving.discard(widgets.serial)
        self._update_free_drive(ctx, now)
        self._update_jogs(ctx, now)
        for widgets in self._arms:
            key = (widgets.serial, widgets.arm.config.name)
            self._load_pose_sliders(widgets)
            dragged = self._dragged.pop(key, None)
            if dragged is not None:
                self._set_pose_sliders(widgets, *target_from_world(ctx, widgets.serial, widgets.arm, dragged))
            target = widgets.arm.from_husky(*pose_from_sliders(widgets.compliance.pose))
            force = np.array([slider.value for slider in widgets.compliance.force])
            self._markers[key] = place_markers(ctx, widgets.serial, widgets.arm, target, force,
                                               self._applied_force.get(key))
        self._leases.release_idle(ctx, partial(self._busy, now))

    def _keep_clear(self, ctx: PluginContext, widgets: _BaseWidgets) -> None:
        """While Clearance is ticked, have a clearance kept on the base this plugin is about to drive."""
        guard = self._base_guards.get(widgets.serial)
        if widgets.clearance is not None and widgets.clearance.value and (guard is None or guard.done()):
            self._base_guards[widgets.serial] = ctx.spawn(f"clearance {widgets.serial}",
                                                          self._guard_base(ctx, widgets))

    async def _guard_base(self, ctx: PluginContext, widgets: _BaseWidgets) -> None:
        """Keep a clearance on the base while this plugin holds its lease and Clearance stays ticked.

        A trip soft stops the base (`ctx.clearance`); the status line then says what it came too close to. Driving
        by hand, an arm whose joints are not known is checked at its stow pose; the base pose must be known.
        """
        serial, clearance = widgets.serial, None
        try:
            with ctx.clearance([serial], unknown_arms=ALL_ARMS) as clearance:
                await ctx.wait_until(lambda: not self._leases.held(serial) or not widgets.clearance.value)
        except ClearanceRefused as refused:
            # ! The drive streams its twists apart from this guard: stop the robot, as a trip would.
            self._tripped[serial] = str(refused)
            ctx.world.robots[serial].soft_stop(str(refused))
            self._refuse(ctx, (serial, BASE), "cannot check", f"{refused}: soft stopped")
        except asyncio.CancelledError:
            if clearance is not None and clearance.problem:
                self._tripped[serial] = clearance.problem
                self._refuse(ctx, (serial, BASE), "too close", f"{clearance.problem}: soft stopped")
            raise

    def _busy(self, now: float, serial: str) -> bool:
        """Whether this plugin still uses robot `serial`: an input held, its own move or force, or a part moving."""
        if serial in self._held or serial in self._driving:
            return True
        for widgets in self._arms:
            key = (widgets.serial, widgets.arm.config.name)
            if widgets.serial != serial:
                continue
            task, force = self._trajectory_tasks.get(key), self._applied_force.get(key)
            free_drive = self._free_drive_held.get(key)
            keep = widgets.free_drive_keep
            if (key in self._jog_held or key in self._jogging or key in self._free_driving
                    or (free_drive is not None and now - free_drive[0] <= HOLD_TIMEOUT)
                    or (keep is not None and keep.value) or (task is not None and not task.done())
                    or (force is not None and np.any(force)) or widgets.arm.state.is_executing
                    or _tool_moving(widgets.arm.state.end_effector)):
                return True
        return False

    def _update_free_drive(self, ctx: PluginContext, now: float) -> None:
        """Keep free drive on while a free drive button is held or Keep on is ticked; the phone's button also switches.

        - Laptop: the Free controller button switches; holding its button, or Keep on, frees the arm.
        - Phone: holding switches to free drive and frees the arm; on release the joint controller holds it.
        ! On release, switch back only while free drive still runs: after Stop all or a switch on the laptop, a
          release must not start a controller. If the switch back fails, free drive still ends within 1 s, since
          the requests stop (`ArmInterface.request_free_drive`).

        Args:
            ctx: This plugin's context.
            now: This tick's time, as the hold reports use.
        """
        for widgets in self._arms:
            key = (widgets.serial, widgets.arm.config.name)
            controllers = widgets.arm.controllers
            last = self._free_drive_held.get(key)
            # * While another plugin has the robot's lease or it is soft stopped, held buttons and Keep on do nothing.
            dropped = self._drop_reason(ctx, widgets.serial, widgets.arm.soft_stopped)
            pressed = last is not None and now - last[0] <= HOLD_TIMEOUT
            keep = widgets.free_drive_keep
            if keep is not None and keep.value and controllers.state.requested != FREE_DRIVE_CONTROLLER:
                keep.value = False  # another controller was chosen: Keep on must be ticked again
            kept = keep is not None and keep.value
            if dropped is not None and (pressed or kept):
                self._refuse(ctx, key, *dropped)
            held = pressed and dropped is None
            if held or (kept and dropped is None):
                self._leased(ctx, key, widgets.arm.request_free_drive)
            # Switch for the phone only once a pending switch is answered, and not straight after a refused one.
            phone_held = held and last[1]
            if controllers.state.switch_in_flight is not None or now < self._free_drive_retry.get(key, now):
                continue
            if phone_held and key not in self._free_driving:
                switched = self._leased(ctx, key, lambda: controllers.switch(FREE_DRIVE_CONTROLLER))
                if switched:
                    self._free_driving.add(key)
            elif not phone_held and key in self._free_driving:
                # * Switching back holds the arm, which needs no lease.
                switched = (not controllers.is_active(FREE_DRIVE_CONTROLLER)
                            or controllers.switch(SCALED_JOINT_TRAJECTORY_CONTROLLER, unguarded=True))
                if switched:
                    self._free_driving.discard(key)
            else:
                continue
            if not switched:
                self._free_drive_retry[key] = now + FREE_DRIVE_RETRY

    def _update_jogs(self, ctx: PluginContext, now: float) -> None:
        """Send one jog when a jog button is pressed, and hold the arm once it is released or another is pressed.

        - A press waits until the arm has been still for MOVING_WITHIN, since the arm refuses a trajectory sooner.
        - One jog per press: a refused one (the log says why) is not tried again until the next press.

        Args:
            ctx: This plugin's context.
            now: This tick's time, as the hold reports use.
        """
        for widgets in self._arms:
            key, arm = (widgets.serial, widgets.arm.config.name), widgets.arm
            held = self._jog_held.get(key)
            # Drop the press if it timed out, the robot was soft stopped or leased, or the controller no longer runs.
            dropped = None if held is None else self._drop_reason(ctx, widgets.serial, arm.soft_stopped)
            if dropped is None and held is not None \
                    and not arm.controllers.is_active(SCALED_JOINT_TRAJECTORY_CONTROLLER):
                dropped = "jog refused", "the joint controller is not running; switch to Joint"
            if held is not None and (now - held[2] > HOLD_TIMEOUT or dropped is not None):
                if dropped is not None:
                    self._refuse(ctx, key, *dropped)
                del self._jog_held[key]
                held = None
            press = None if held is None else held[:2]
            jogging = self._jogging.get(key)
            if jogging is not None and jogging[:2] != press:
                del self._jogging[key]
                if jogging[2]:
                    arm.hold()
                jogging = None
            moved = arm.state.last_motion_time
            if press is not None and jogging is None and (moved is None or now - moved >= MOVING_WITHIN):
                self._jogging[key] = (*press, self._send_jog(ctx, widgets, *press))

    def _send_jog(self, ctx: PluginContext, widgets: _ArmWidgets, joint: int, direction: float) -> bool:
        """Send a jog of one joint at the speed slider's speed; whether the arm took it (if not, a chip says why)."""
        key = (widgets.serial, widgets.arm.config.name)
        here = widgets.arm.joint_vector()
        if here is None:
            self._refuse(ctx, key, "jog refused", "no joint state yet, not jogging")
            return False
        try:
            positions, velocities, duration = joint_jog(here, joint, direction,
                                                        widgets.jog_speed.value * JOG_MAX_SPEED)
        except ValueError as error:
            self._refuse(ctx, key, "jog refused", f"not jogging: {error}")
            return False
        # * A refusal by the arm's own checks shows through `arm.last_refusal`.
        sent = self._leased(ctx, key, lambda: widgets.arm.send_joint_trajectory(positions, duration, velocities))
        return bool(sent)

    def draw(self, ctx: PluginContext) -> None:
        """Copy every robot's state into the widgets.

        Args:
            ctx: This plugin's context.
        """
        now = ctx.now()
        for widgets in self._bases:
            show_only_running(widgets.panel)
            refused = refusal_chip((self._refusals.get((widgets.serial, BASE)), widgets.base.last_refusal), now)
            widgets.status.content = base_status(widgets.base, now, refused)
            if widgets.phone_status is not None:
                widgets.phone_status.content = phone_text(phone_base_status(widgets.base, refused))
        for widgets in self._arms:
            key = (widgets.serial, widgets.arm.config.name)
            show_only_running(widgets.panel)
            tool = widgets.arm.end_effector
            refused = refusal_chip((self._refusals.get(key), widgets.arm.last_refusal,
                                    None if tool is None else tool.last_refusal), now)
            widgets.status.content = arm_status(widgets.arm, now, refused)
            widgets.compliance.pose_plan.content = pose_plan_line(widgets.arm, widgets.compliance.pose)
            draw_markers(widgets.frames, widgets.arrows, widgets.gizmo, self._markers.get(key) or {})
            applied = self._applied_force.get(key)
            widgets.compliance.wrench_line.content = block(values(
                f"applied {numbers(applied, 3, 6, 1)} N ({TARGET_WRENCH_FRAME})"))
            if widgets.tool_status is not None:
                widgets.tool_status.content = tool_status(widgets.arm.state.end_effector, now)
            if widgets.phone_status is not None:
                widgets.phone_status.content = phone_text(phone_arm_status(widgets.arm, refused))

    def _load_pose_sliders(self, widgets: _ArmWidgets) -> None:
        """Set one arm's pose sliders to its TCP, if asked.

        Args:
            widgets: One arm's handles.
        """
        key = (widgets.serial, widgets.arm.config.name)
        state = widgets.arm.state
        if key not in self._load_pose or state.tcp_position is None:
            return  # not asked, or no TCP yet
        self._set_pose_sliders(widgets, *widgets.arm.to_husky(state.tcp_position, state.tcp_orientation))
        self._load_pose.discard(key)

    @staticmethod
    def _set_pose_sliders(widgets: _ArmWidgets, position: np.ndarray, orientation: np.ndarray) -> None:
        """Set one arm's pose sliders to a TCP target in the husky's base_link, rounded to the sliders' steps."""
        angles = Rotation.from_quat(orientation).as_euler("xyz", degrees=True)
        for slider, value in zip(widgets.compliance.pose, [*position, *angles]):
            slider.value = round(float(value) / slider.step) * slider.step

    # --- --- --- --- --- MOVE TASKS --- --- --- --- ---

    def _start_move(self, ctx: PluginContext, serial: str, arm: ArmInterface, kind: str,
                    move: Callable[..., Coroutine[Any, Any, None]], *args: Any) -> None:
        """Start `move(ctx, serial, arm, *args)` as the arm's move task, unless one is already running.

        Args:
            ctx: This plugin's context.
            serial: The robot's serial.
            arm: The arm to move.
            kind: Task name prefix, e.g. "cartesian move".
            move: The move, e.g. `_stream_cartesian_move`.
            *args: The move's target, after (ctx, serial, arm).
        """
        key = (serial, arm.config.name)
        running = self._trajectory_tasks.get(key)
        if running is not None and not running.done():
            self._refuse(ctx, key, "start refused", "a move is already running; Start ignored")
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
        self._applied_force.pop((serial, arm.config.name), None)  # hold zeroes the target wrench
        running = self._trajectory_tasks.get((serial, arm.config.name))
        if running is not None and not running.done():
            running.cancel()

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
        try:
            with self._leases.use(ctx, serial):
                await self._cartesian_move(ctx, serial, arm, position, orientation)
        except MotionRefused as refused:
            self._refuse(ctx, (serial, arm.config.name), refused.label, str(refused))

    async def _cartesian_move(self, ctx: PluginContext, serial: str, arm: ArmInterface,
                              position: np.ndarray, orientation: np.ndarray) -> None:
        """The body of `_stream_cartesian_move`, run under the robot's lease."""
        name = f"{serial} {arm.config.name}"
        if arm.state.tcp_position is None:
            self._refuse(ctx, (serial, arm.config.name), "start refused", "no TCP pose yet, nothing sent")
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


def _tool_moving(state: EndEffectorState | None) -> bool:
    """Whether a tool still runs a command: a Robotiq goal, or a scaffolding v3 motor."""
    if isinstance(state, RobotiqState):
        return state.moving
    if isinstance(state, ScaffoldingV3State):
        return ScaffoldingV3.TIGHTENING in (state.gripper_motor, state.joint_motor) or \
            ScaffoldingV3.LOOSENING in (state.gripper_motor, state.joint_motor)
    return False
