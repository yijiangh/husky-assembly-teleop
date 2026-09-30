"""
The robot control panel: a tab per robot showing its state, with basic control.

* Small on purpose, as the pattern to copy when writing a plugin:
  - `setup` builds every widget once and keeps the handles.
  - Widget callbacks go through `ctx.defer` / `ctx.defer_value`, never do the
    work themselves: viser calls them on its own threads, where touching
    robots or the scene is not safe.
  - `update` does per-tick work (here, streaming the base twist).
  - Anything longer than a tick is a task started with `ctx.spawn`.
  - `draw` only copies state into widgets.
* Robots are commanded and read only through their interfaces; no topic names here.

Each part tab (Base, Left, Right) has a status line, a CTRL section (controller
buttons plus the running controller's inputs) and SENSOR/TOOL actions.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from typing import Callable

import numpy as np
import viser
from scipy.spatial.transform import Rotation

from ..plugin_api.concurrency import WaitTimeout
from ..plugin_api.context import PluginContext
from ..world.mocap import mocap_check
from ..plugin_api.plugin import HuskyPlugin, register
from ..robot_interface import ArmInterface, BaseInterface, RobotiqGripper, ScaffoldingV1, ScaffoldingV3
from ..robot_interface.end_effectors import RobotiqState, ScaffoldingV1State, ScaffoldingV3State
from ..robot_interface.arm import (CARTESIAN_COMPLIANCE_CONTROLLER,
                                   CARTESIAN_MOVE_MAX_ROTATION_SPEED,
                                   CARTESIAN_MOVE_MAX_SPEED, CARTESIAN_MOVE_MIN_DURATION, JOINT_MOVE_MAX_SPEED,
                                   JOINT_MOVE_MIN_DURATION, MAX_TARGET_FORCE, SCALED_JOINT_TRAJECTORY_CONTROLLER,
                                   TARGET_WRENCH_FRAME, UR_JOINT_NAMES,
                                   cartesian_move, joint_move)
from ..robot_interface.base import PLATFORM_VELOCITY_CONTROLLER
from ..robot_interface.controller_manager import ControllerManagerInterface
from ..robot_interface.ur_frames import STOCK_YAW
from ..world.scene import Pose, compose
from ..ui.ghost import TARGET_COLOR, RecentUse, RobotGhost
from ..ui.visualization import quaternion_to_wxyz
from ..ui.style import (STALE_AFTER, BUSY, FAIL, NONE, OK, SECTION_CTRL, SECTION_SENSOR, SECTION_TOOL, block,
                        check_chip, chip, freshness_chip, note, numbers, section, values, warning)

#: Base speed at a Speed of 1. Kept low: this is a test panel.
MAX_LINEAR_SPEED = 0.3   # m/s
MAX_ANGULAR_SPEED = 0.5  # rad/s
#: Hold buttons in D-pad order (Forward, Left, Back, Right): label, icon, and the
#: signs of the linear and angular speed they ask for.
HOLD_BUTTONS = (("", viser.Icon.ARROW_UP, 1.0, 0.0),
                ("", viser.Icon.ROTATE, 0.0, 1.0),
                ("", viser.Icon.ARROW_DOWN, -1.0, 0.0),
                ("", viser.Icon.ROTATE_CLOCKWISE, 0.0, -1.0))
#: Each hold button's D-pad cell, (row, column), in HOLD_BUTTONS order.
DPAD_CELLS = ((1, 2), (2, 1), (2, 2), (2, 3))
#: How often the browser reports a held button, Hz. Matches the 20 Hz tick.
HOLD_CALLBACK_HZ = 20.0
#: A hold counts as released after this long without a report, seconds.
#: ! viser never reports a release, so this timeout is what stops the base,
#:   including when the browser disconnects mid-hold.
HOLD_TIMEOUT = 0.25
#: Slider label and range (degrees) per UR joint, in UR_JOINT_NAMES order.
JOINT_SLIDERS = (("pan", 360), ("lift", 360), ("elbow", 180), ("w1", 360), ("w2", 360), ("w3", 360))
#: TCP position slider range (metres, around the arm's base) and step.
POSITION_RANGE, POSITION_STEP = 1.5, 0.001
POSE_SLIDERS = (("x m", "p"), ("y m", "p"), ("z m", "p"), ("roll °", "a"), ("pitch °", "a"), ("yaw °", "a"))
#: Poses drawn in 3D while the Cartesian controller runs: name, label, origin
#: colour (r, g, b), axis length (m).
FRAME_MARKERS = (
    ("target", "target (sliders)", (255, 200, 0), 0.12),
    ("reported", "TCP reported", (40, 170, 60), 0.08),
)
#: Force arrows drawn from the reported TCP: name, label, colour.
FORCE_ARROWS = (
    ("force_preview", "force (sliders)", (255, 200, 0)),
    ("force_applied", "force applied", (255, 110, 0)),
)
FORCE_ARROW_SCALE = 0.01  # metres per newton: 20 N draws 20 cm
#: Target force slider step, newtons.
FORCE_STEP = 0.5
#: A move has arrived when every joint is this close to its target, degrees.
ARRIVED_TOLERANCE = 0.5
#: Seconds past the planned duration before an unfinished move is reported as timed out.
ARRIVE_TIMEOUT_MARGIN = 5.0

#: Starting panel width, pixels. Fits the arm's joint row, the widest line.
PANEL_WIDTH = 420

#: Short button labels for controllers; unlisted ones show their full name.
CONTROLLER_LABELS = {
    PLATFORM_VELOCITY_CONTROLLER: "Vel",
    SCALED_JOINT_TRAJECTORY_CONTROLLER: "Joint",
    CARTESIAN_COMPLIANCE_CONTROLLER: "Cart",
}
#: Tab labels for arms, by ArmConfig.name.
ARM_LABELS = {"ur_arm": "Arm", "left_ur_arm": "Left", "right_ur_arm": "Right"}
#: Short names for end effector kinds, shown next to the TOOL section label.
TOOL_LABELS = {"robotiq": "robotiq", "scaffolding_v1": "v1", "scaffolding_v3": "v3"}


# --- --- --- --- --- WIDGET HANDLES --- --- --- --- ---
# ? Handles built in setup, for draw to find. Plugin state lives on the plugin.

@dataclass
class _ControllerPanel:
    """Controller buttons for one controller manager, and each controller's inputs.

    Attributes:
        controllers: The controller manager this panel shows and switches.
        inputs: One folder of inputs per controller; only the running
            controller's folder is visible.
    """

    controllers: ControllerManagerInterface
    inputs: dict[str, viser.GuiFolderHandle] = field(default_factory=dict)


@dataclass
class _BaseWidgets:
    """Handles for one base."""

    serial: str
    base: BaseInterface
    panel: _ControllerPanel
    status: viser.GuiHtmlHandle
    speed: viser.GuiSliderHandle


@dataclass
class _ArmWidgets:
    """Handles for one arm and the tool on it."""

    serial: str
    arm: ArmInterface
    panel: _ControllerPanel
    status: viser.GuiHtmlHandle
    tool_status: viser.GuiHtmlHandle | None
    joints: list[viser.GuiSliderHandle]
    plan: viser.GuiHtmlHandle
    pose: list[viser.GuiSliderHandle]
    pose_plan: viser.GuiHtmlHandle
    force: list[viser.GuiSliderHandle]
    wrench_line: viser.GuiHtmlHandle
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

        #: Serials of bases whose twist was streamed last tick, so update can send one final stop on release.
        self._driving: set[str] = set()

        #: Last hold report per base serial: (linear sign, angular sign, arrival time).
        self._held: dict[str, tuple[float, float, float]] = {}

        #: Running move task per arm, keyed by (serial, arm name). A press while one runs is ignored.
        self._trajectory_tasks: dict[tuple[str, str], asyncio.Task] = {}

        #: Arms whose joint sliders are set on the next update, with the angles
        #: (radians) or None for the measured joints. Filled by Current and Stow,
        #: and at startup so the sliders show where the arm is.
        self._load_sliders: dict[tuple[str, str], np.ndarray | None] = {}

        #: Arms whose pose sliders are set to the measured TCP on the next update.
        self._load_pose: set[tuple[str, str]] = set()

        #: Target force last sent to each arm, N. The controller keeps it, so the panel shows it.
        self._applied_force: dict[tuple[str, str], np.ndarray] = {}

        #: This tick's 3D marker poses and force-arrow segments per arm; absent
        #: while the compliance controller is not running on that arm.
        self._markers: dict[tuple[str, str], dict[str, tuple]] = {}

        #: Per robot, a ghost at its arms' joint slider targets, built in setup.
        self._previews: dict[str, RobotGhost] = {}
        #: Per arm, when the operator last used its joint inputs; its preview shows only while it counts.
        self._joints_used: dict[tuple[str, str], RecentUse] = {}
        #: Per arm, the joint slider values `_load_joint_sliders` wrote last. ! Writing a slider
        #: fires its callback too; a callback that finds these values is ours, not the operator's.
        self._written: dict[tuple[str, str], tuple[float, ...]] = {}

    # --- --- --- --- --- SETUP --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Build one panel with a tab per robot, holding inline tabs for its base and arms.

        Args:
            ctx: This plugin's context.
        """
        # Raw gui api, not ui(): ui() would leave an empty folder in the main panel.
        gui = ctx.view.gui
        # * One panel with a tab per robot; inside each, inline tabs per part.
        #   A robot tab can be dragged out into its own panel, parts included.
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
        # ? Built now: mesh loading is too slow for the tick.
        self._previews = {config.serial: RobotGhost(ctx, config, f"preview/{config.serial}", TARGET_COLOR)
                          for config in ctx.config.robots}

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
        handles: dict[str, viser.GuiInputHandle] = {}

        def velocity_inputs() -> None:
            """Inputs for platform_velocity_controller: hold buttons that drive while pressed."""
            handles["speed"] = gui.add_slider("Speed", min=0.05, max=1.0, step=0.05, initial_value=0.5,
                                              hint="Fraction of the full speed "
                                                   f"({MAX_LINEAR_SPEED} m/s, {MAX_ANGULAR_SPEED} rad/s).")
            grid = gui.add_html("")
            with gui.add_folder("Drive"):
                buttons = [gui.add_button(label, icon=icon, hint="Drives while held.")
                           for label, icon, _, _ in HOLD_BUTTONS]
            grid.content = _dpad_style([button._impl.uuid for button in buttons])
            for button, (label, _, linear, angular) in zip(buttons, HOLD_BUTTONS):
                # ! Bind loop values as defaults, or every button drives like the last.
                button.on_hold(ctx.defer(f"hold {label} {serial}",
                                         lambda linear=linear, angular=angular: self._on_hold(ctx, serial,
                                                                                              linear, angular)),
                               callback_hz=HOLD_CALLBACK_HZ)

        panel = _build_controller_panel(ctx, gui, base.controllers,
                                        {PLATFORM_VELOCITY_CONTROLLER: velocity_inputs})
        return _BaseWidgets(serial=serial, base=base, panel=panel, status=status, speed=handles["speed"])

    def _on_hold(self, ctx: PluginContext, serial: str, linear: float, angular: float) -> None:
        """Record that a hold button is still pressed. Runs as an intent.

        Args:
            ctx: This plugin's context.
            serial: The base's robot serial.
            linear: Sign of the linear speed the button asks for.
            angular: Sign of the angular speed the button asks for.
        """
        self._held[serial] = (linear, angular, ctx.now())

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
        status = gui.add_html("")

        key = (serial, arm.config.name)
        self._load_sliders[key] = None
        self._joints_used[key] = RecentUse(ctx.config.ghost_timeout)
        self._load_pose.add(key)
        handles: dict[str, object] = {}

        def trajectory_inputs() -> None:
            """Inputs for the joint trajectory controller: a target per joint, and Send."""
            gui.add_html(warning("no collision checking · raw joint trajectories"))
            gui.add_html(note("yellow ghost: the arm at the sliders"))
            handles["joints"] = [gui.add_slider(f"{name} °", min=-limit, max=limit, step=0.5, initial_value=0.0)
                                 for name, limit in JOINT_SLIDERS]
            handles["plan"] = gui.add_html("")
            load = gui.add_button_group("Load", ["Current", "Stow"],
                                        hint="Set the sliders to where the arm is, or to its stow pose.")
            move = gui.add_button_group("Move", ["Start", "Hold"],
                                        hint="Start: move to the sliders, smoothly, in 5 s or more. "
                                             "Hold: brake to a standstill where the arm is.")

            def on_slider(_value: object) -> None:
                """A joint slider moved: by the operator, unless we just wrote these values. Intent."""
                if tuple(slider.value for slider in handles["joints"]) != self._written.get(key):
                    self._joints_used[key].touch()

            def on_load(clicked: str) -> None:
                """Load the sliders. Runs as an intent, on the main thread."""
                self._joints_used[key].touch()
                if clicked == "Current":
                    self._load_sliders[key] = None
                elif arm.config.stow_joints is None:
                    ctx.log_warn(f"{label}: no stow pose configured (ArmConfig.stow_joints)")
                else:
                    self._load_sliders[key] = np.array(arm.config.stow_joints)

            def on_move(clicked: str) -> None:
                """Start or Hold. Runs as an intent, on the main thread."""
                self._joints_used[key].touch()
                if clicked == "Hold":
                    self._hold_arm(serial, arm)
                else:
                    # ! Read sliders here, when the intent runs: the operator commits on Start.
                    target = np.radians([slider.value for slider in handles["joints"]])
                    self._start_move(ctx, serial, arm, target)

            for slider in handles["joints"]:
                slider.on_update(ctx.defer_value(f"joint slider {label}", on_slider))
            load.on_click(ctx.defer_value(f"load sliders {label}", on_load))
            move.on_click(ctx.defer_value(f"move {label}", on_move))

        def compliance_inputs() -> None:
            """Inputs for the compliance controller: a TCP target, and a target force."""
            if arm.urdf_problem is not None:
                gui.add_html(warning("URDF frames wrong · targets blocked (see log)", color=FAIL))
            if arm.config.cartesian_test_mode:
                gui.add_html(warning("test mode · checks run, nothing is sent", color=FAIL))
            gui.add_html(warning("no collision checking · straight TCP line, joints unchecked"))
            gui.add_html(note("target: TCP in the husky's base_link"))
            handles["pose"] = [
                gui.add_slider(name, min=-POSITION_RANGE, max=POSITION_RANGE, step=POSITION_STEP, initial_value=0.0)
                if kind == "p" else gui.add_slider(name, min=-180.0, max=180.0, step=0.1, initial_value=0.0)
                for name, kind in POSE_SLIDERS]
            handles["pose_plan"] = gui.add_html("")
            load = gui.add_button_group("Load", ["Current"],
                                        hint="Set the sliders to where the TCP is (in the husky's base_link).")
            move = gui.add_button_group("Move", ["Start", "Hold"],
                                        hint="Start: move the TCP to the sliders in a straight line, "
                                             "5 s or more. Hold: stop where the TCP is.")

            # ! Force is in the tool frame (TARGET_WRENCH_FRAME in arm.py).
            gui.add_html(section("force", SECTION_CTRL, f"in {TARGET_WRENCH_FRAME} frame"))
            limit = MAX_TARGET_FORCE / np.sqrt(3.0)  # all three at their limit stay within MAX_TARGET_FORCE
            handles["force"] = [gui.add_slider(f"F{axis} N", min=-limit, max=limit, step=FORCE_STEP, initial_value=0.0,
                                               hint=f"Along the tool's own {axis} axis ({TARGET_WRENCH_FRAME}).")
                                for axis in "xyz"]
            handles["wrench_line"] = gui.add_html("")
            wrench = gui.add_button_group("Force", ["Apply", "Zero"],
                                          hint=f"Apply: push with this force, in the tool frame "
                                               f"({TARGET_WRENCH_FRAME}), until changed. Zero: remove it.")

            # Legend for the 3D markers.
            gui.add_html(block("".join(
                f'<span style="font-size:11px;margin-right:8px;white-space:nowrap">'
                f'<span style="color:rgb{color}">●</span> {text}</span>'
                for _, text, color, _ in FRAME_MARKERS)
                + "".join(f'<span style="font-size:11px;margin-right:8px;white-space:nowrap">'
                          f'<span style="color:rgb{color}">➜</span> {text}</span>'
                          for _, text, color in FORCE_ARROWS)))

            def on_load(_clicked: str) -> None:
                """Load the pose sliders from the TCP. Runs as an intent."""
                self._load_pose.add(key)

            def on_move(clicked: str) -> None:
                """Start or Hold. Runs as an intent, on the main thread."""
                if clicked == "Hold":
                    self._hold_arm(serial, arm)
                else:
                    position, orientation = arm.from_husky(*_pose_from_sliders(handles["pose"]))
                    self._start_cartesian_move(ctx, serial, arm, position, orientation)

            def on_force(clicked: str) -> None:
                """Apply or remove the target force. Runs as an intent."""
                if clicked == "Zero":
                    force = np.zeros(3)
                    # Zero the sliders too, so a later Apply does not restore the old force.
                    for slider in handles["force"]:
                        slider.value = 0.0
                else:
                    force = np.array([slider.value for slider in handles["force"]])
                if arm.send_target_wrench(force):
                    self._applied_force[key] = force

            load.on_click(ctx.defer_value(f"load pose {label}", on_load))
            move.on_click(ctx.defer_value(f"cartesian move {label}", on_move))
            wrench.on_click(ctx.defer_value(f"force {label}", on_force))

        panel = _build_controller_panel(ctx, gui, arm.controllers, {
            SCALED_JOINT_TRAJECTORY_CONTROLLER: trajectory_inputs,
            CARTESIAN_COMPLIANCE_CONTROLLER: compliance_inputs,
        })

        # Independent of the running controller.
        gui.add_html(section("sensor", SECTION_SENSOR))
        zero = gui.add_button_group("FT", ["Zero"], hint="Tares the current load. Only with nothing gripped.")
        zero.on_click(ctx.defer(f"zero FT {label}", arm.zero_ft_sensor))

        tool_status = None
        if arm.end_effector is not None:
            kind = arm.config.end_effector
            gui.add_html(section("tool", SECTION_TOOL, TOOL_LABELS.get(kind, kind)))
            tool_status = gui.add_html("")
            tool = arm.end_effector
            commands = {"Open": tool.open, "Close": tool.close, "Stop": tool.stop}
            grip = gui.add_button_group("Grip", list(commands),
                                        hint="Stop halts every motor of the tool. A v1 gripper only opens or "
                                             "closes, so there Stop switches the screw off and leaves the grip.")
            grip.on_click(ctx.defer_value(f"grip {label}", lambda clicked: commands[clicked]()))
            if isinstance(tool, RobotiqGripper):
                _build_robotiq_inputs(ctx, gui, label, tool)
            elif isinstance(tool, (ScaffoldingV1, ScaffoldingV3)):
                # v1 runs its screw one way only; v3 both ways.
                directions = ({"Run": 1, "Stop": 0} if isinstance(tool, ScaffoldingV1)
                              else {"Tighten": 1, "Loosen": -1, "Stop": 0})
                screw = gui.add_button_group("Screw", list(directions), hint="The motor that tightens the bar "
                                                                             "to the joint. Runs until Stop.")
                screw.on_click(ctx.defer_value(f"screw {label}",
                                               lambda clicked, tool=tool: tool.drive_screw(directions[clicked])))

        return _ArmWidgets(serial=serial, arm=arm, panel=panel, status=status, tool_status=tool_status,
                           joints=handles["joints"], plan=handles["plan"], pose=handles["pose"],
                           pose_plan=handles["pose_plan"], force=handles["force"],
                           wrench_line=handles["wrench_line"],
                           frames=_add_frame_markers(ctx, serial, arm), arrows=_add_force_arrows(ctx, serial, arm))

    def teardown(self, ctx: PluginContext) -> None:
        """At shutdown, stop any base this plugin was driving.

        Args:
            ctx: This plugin's context.
        """
        for widgets in self._bases:
            if widgets.serial in self._driving:
                widgets.base.send_twist(0.0, 0.0)

    # --- --- --- --- --- EVERY TICK --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Stream the twist of every held base, and apply queued slider loads and marker updates for arms.

        ? Sent every tick because the base stops when cmd_vel goes quiet.

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
            self._load_joint_sliders(widgets)
            self._load_pose_sliders(widgets)
            self._update_markers(ctx, widgets)

    def draw(self, ctx: PluginContext) -> None:
        """Copy every robot's state into the widgets.

        Args:
            ctx: This plugin's context.
        """
        now = ctx.now()
        for widgets in self._bases:
            _show_only_running(widgets.panel)
            widgets.status.content = _base_status(widgets.base, now)
        for widgets in self._arms:
            _show_only_running(widgets.panel)
            widgets.status.content = _arm_status(widgets.arm, now)
            widgets.plan.content = _plan_line(widgets)
            widgets.pose_plan.content = _pose_plan_line(widgets)
            check = self._markers.get((widgets.serial, widgets.arm.config.name), {})
            for name, frame in widgets.frames.items():
                pose = check.get(f"{name}_world")
                frame.visible = pose is not None
                if pose is not None:
                    frame.position, frame.wxyz = pose[0], quaternion_to_wxyz(np.array(pose[1]))
            for name, arrow in widgets.arrows.items():
                segment = check.get(f"{name}_world")
                arrow.visible = segment is not None
                if segment is not None:
                    arrow.points = np.array([segment])
            applied = self._applied_force.get((widgets.serial, widgets.arm.config.name))
            widgets.wrench_line.content = block(values(
                f"applied {numbers(applied, 3, 6, 1)} N ({TARGET_WRENCH_FRAME})"))
            if widgets.tool_status is not None:
                widgets.tool_status.content = _tool_status(widgets.arm.state.end_effector, now)
        self._draw_previews(ctx)

    def _draw_previews(self, ctx: PluginContext) -> None:
        """Show each arm at its joint sliders while they are in use and differ from the arm.

        * Only the arms that qualify are drawn (with their tools); the base stays where it is.

        Args:
            ctx: This plugin's context.
        """
        for serial, ghost in self._previews.items():
            joints = dict(ctx.kinematics.joints(serial))
            parts = []
            for widgets in self._arms:
                name = widgets.arm.config.name
                here = widgets.arm.joint_vector()
                if widgets.serial != serial or here is None or not self._joints_used[(serial, name)].active:
                    continue
                target = np.radians([slider.value for slider in widgets.joints])
                if np.max(np.abs(target - here)) > np.radians(ARRIVED_TOLERANCE):
                    parts.append(f"{name}_")
                    joints.update(zip((f"{name}_{joint}" for joint in UR_JOINT_NAMES), target))
            if parts:
                ghost.show(ctx.kinematics.base_pose(serial), joints, parts=tuple(parts))
            else:
                ghost.hide()

    # --- --- --- --- --- TASKS --- --- --- --- ---
    # Started from intents, so always on the main thread.

    def _start_move(self, ctx: PluginContext, serial: str, arm: ArmInterface, target: np.ndarray) -> None:
        """Start the move task for one arm, unless one is already running.

        Args:
            ctx: This plugin's context.
            serial: The robot's serial.
            arm: The arm to move.
            target: Joint angles to move to, radians, UR_JOINT_NAMES order.
        """
        key = (serial, arm.config.name)
        running = self._trajectory_tasks.get(key)
        if running is not None and not running.done():
            ctx.log_warn(f"{serial} {arm.config.name}: a move is already running; Send ignored")
            return
        self._trajectory_tasks[key] = ctx.spawn(f"move {serial} {arm.config.name}",
                                               self._execute_move(ctx, serial, arm, target))

    def _hold_arm(self, serial: str, arm: ArmInterface) -> None:
        """Hold the arm where it is and cancel its running move task.

        ! The hold is sent even when no task of ours runs: a trajectory may come from elsewhere.

        Args:
            serial: The robot's serial.
            arm: The arm to hold.
        """
        arm.hold()
        running = self._trajectory_tasks.get((serial, arm.config.name))
        if running is not None and not running.done():
            running.cancel()

    async def _execute_move(self, ctx: PluginContext, serial: str, arm: ArmInterface, target: np.ndarray) -> None:
        """Send a smooth joint move and wait until the arm has arrived.

        ? Arrived means the planned time is up and every joint is within
          ARRIVED_TOLERANCE of its target, not `is_executing`.

        Args:
            ctx: This plugin's context.
            serial: The robot's serial, for messages.
            arm: The arm to move.
            target: Joint angles to move to, radians.
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

    def _start_cartesian_move(self, ctx: PluginContext, serial: str, arm: ArmInterface,
                              position: np.ndarray, orientation: np.ndarray) -> None:
        """Start the Cartesian move task for one arm, unless any move is already running.

        ! Shares `_trajectory_tasks` with joint moves: one motion per arm at a time.

        Args:
            ctx: This plugin's context.
            serial: The robot's serial.
            arm: The arm to move.
            position: TCP target position, metres, arm base frame.
            orientation: TCP target orientation, quaternion (x, y, z, w).
        """
        key = (serial, arm.config.name)
        running = self._trajectory_tasks.get(key)
        if running is not None and not running.done():
            ctx.log_warn(f"{serial} {arm.config.name}: a move is already running; Start ignored")
            return
        self._trajectory_tasks[key] = ctx.spawn(f"cartesian move {serial} {arm.config.name}",
                                               self._stream_cartesian_move(ctx, serial, arm, position, orientation))

    async def _stream_cartesian_move(self, ctx: PluginContext, serial: str, arm: ArmInterface,
                                     position: np.ndarray, orientation: np.ndarray) -> None:
        """Move the TCP to a target by sending one close target per tick.

        ? Streamed because the compliance controller jumps straight to its
          target; each tick sends the next point of a smooth `cartesian_move`.

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

    def _update_markers(self, ctx: PluginContext, widgets: _ArmWidgets) -> None:
        """Place one arm's 3D markers: target, reported TCP and force arrows. Runs every tick.

        ? Markers are placed in the controller's own `base_link` (the arm's
          `base_link_inertia` turned by STOCK_YAW), so they show where the
          controller will take them. See doc/ur_frames.md.

        Args:
            ctx: This plugin's context. Link poses come from `ctx.kinematics`.
            widgets: One arm's handles.
        """
        key = (widgets.serial, widgets.arm.config.name)
        arm, state = widgets.arm, widgets.arm.state
        # ! link_pose raises KeyError for a robot kinematics doesn't know: skip the markers instead.
        known = any(robot.serial == widgets.serial for robot in ctx.config.robots)
        if not known or not arm.controllers.is_active(CARTESIAN_COMPLIANCE_CONTROLLER):
            self._markers.pop(key, None)
            return
        inertia = ctx.kinematics.link_pose(widgets.serial, f"{arm.config.name}_base_link_inertia")
        base = compose(inertia, Pose(orientation=tuple(Rotation.from_euler("z", -STOCK_YAW).as_quat())))
        markers: dict[str, tuple] = {}
        local_poses = {"target": arm.from_husky(*_pose_from_sliders(widgets.pose))}
        if state.tcp_position is not None:
            local_poses["reported"] = (state.tcp_position, state.tcp_orientation)
        for name, (position, orientation) in local_poses.items():
            # * (position, orientation as xyzw), the form `draw` reads.
            world = compose(base, Pose.from_arrays(position, orientation))
            markers[f"{name}_world"] = (world.position, world.orientation)
        # Forces are in tool0: they start at the reported TCP and turn with it. None for zero force.
        tcp = markers.get("reported_world")
        forces = {"force_preview": np.array([slider.value for slider in widgets.force]),
                  "force_applied": self._applied_force.get(key)}
        for name, force in forces.items():
            if tcp is not None and force is not None and np.linalg.norm(force) > 0.0:
                start = np.array(tcp[0])
                markers[f"{name}_world"] = (start, start + Rotation.from_quat(tcp[1]).apply(force) * FORCE_ARROW_SCALE)
        self._markers[key] = markers

    def _load_pose_sliders(self, widgets: _ArmWidgets) -> None:
        """Set one arm's pose sliders to its TCP, if Load Current or startup asked.

        Args:
            widgets: One arm's handles.
        """
        key = (widgets.serial, widgets.arm.config.name)
        state = widgets.arm.state
        if key not in self._load_pose or state.tcp_position is None:
            return  # not asked, or no TCP yet
        position, orientation = widgets.arm.to_husky(state.tcp_position, state.tcp_orientation)
        angles = Rotation.from_quat(orientation).as_euler("xyz", degrees=True)
        for slider, value in zip(widgets.pose, [*position, *angles]):
            slider.value = round(float(value) / slider.step) * slider.step
        self._load_pose.discard(key)

    def _load_joint_sliders(self, widgets: _ArmWidgets) -> None:
        """Set one arm's joint sliders, if Current, Stow or startup asked.

        ! Call from update, not draw: draw is skipped while panels are frozen,
          which would leave a stale target in the sliders for Start to send.

        Args:
            widgets: One arm's handles.
        """
        key = (widgets.serial, widgets.arm.config.name)
        if key not in self._load_sliders:
            return
        # None means the measured joints, which need joint data first.
        angles = widgets.arm.joint_vector() if self._load_sliders[key] is None else self._load_sliders[key]
        if angles is not None:
            for slider, value in zip(widgets.joints, np.degrees(angles)):
                slider.value = round(float(value) * 2) / 2  # the sliders' 0.5 deg step
            self._written[key] = tuple(slider.value for slider in widgets.joints)
            del self._load_sliders[key]


# --- --- --- --- --- SHARED BUILDING BLOCKS --- --- --- --- ---

def _build_controller_panel(ctx: PluginContext, gui: viser.GuiApi,
                            controllers: ControllerManagerInterface,
                            input_builders: dict[str, Callable[[], None]]) -> _ControllerPanel:
    """Build the CTRL section: one button per controller, and a folder of inputs for each.

    Every switchable controller gets a folder, even with no inputs yet.

    Args:
        ctx: This plugin's context.
        gui: The GUI api, inside the tab the panel goes in.
        controllers: The controller manager to show and switch.
        input_builders: Per controller, a function that adds its inputs to `gui`.
            Controllers without one get a placeholder.

    Returns:
        _ControllerPanel: The handles.
    """
    gui.add_html(section("ctrl", SECTION_CTRL))
    by_label = {CONTROLLER_LABELS.get(name, name): name for name in controllers.switchable}
    switch = gui.add_button_group("Run", list(by_label))
    switch.on_click(ctx.defer_value(f"switch controller {controllers.namespace}",
                                    lambda clicked: controllers.switch(by_label[clicked])))

    panel = _ControllerPanel(controllers=controllers)
    for controller in controllers.switchable:
        folder = gui.add_folder(CONTROLLER_LABELS.get(controller, controller), visible=False)
        with folder:
            build = input_builders.get(controller)
            if build is None:
                gui.add_html(note("no inputs yet"))
            else:
                build()
        panel.inputs[controller] = folder
    return panel


def _plan_line(widgets: _ArmWidgets) -> str:
    """What Start would do from here: the largest joint change and the duration."""
    here = widgets.arm.joint_vector()
    if here is None:
        return note("no joint state yet")
    target = np.radians([slider.value for slider in widgets.joints])
    _, _, duration = joint_move(here, target)
    largest = float(np.degrees(np.max(np.abs(target - here))))
    return block(values(f"Δmax {largest:6.1f} °   T {duration:5.1f} s   "
                        f"(≤{np.degrees(JOINT_MOVE_MAX_SPEED):.0f} °/s, ≥{JOINT_MOVE_MIN_DURATION:.0f} s)"))


def _dpad_style(uuids: list[str]) -> str:
    """CSS that lays out the four hold buttons, in HOLD_BUTTONS order, as a D-pad.

    ? viser has no grid container, so CSS turns the Drive folder (which holds
      only these buttons) into a three-column grid. Needs `:has()`.

    Args:
        uuids: The buttons' uuids, Forward first.

    Returns:
        str: A <style> element, for an html widget outside the Drive folder
            (inside, it would take a grid cell).
    """
    forward = f'[id="{uuids[0]}"]'
    grid = f"div:has(> div > {forward}) {{ display: grid; grid-template-columns: repeat(3, 1fr); }}"
    # ! Give every button an explicit cell, or Left fills the empty cell beside Forward.
    cells = "".join(f'div:has(> [id="{uuid}"]) {{ grid-area: {row} / {column}; }}'
                    for uuid, (row, column) in zip(uuids, DPAD_CELLS))
    return f"<style>{grid}{cells}</style>"


def _pose_from_sliders(sliders: list[viser.GuiSliderHandle]) -> tuple[np.ndarray, np.ndarray]:
    """The TCP target the pose sliders describe, in the husky's base_link: position and quaternion.

    Args:
        sliders: x, y, z (metres) then roll, pitch, yaw (degrees, fixed axes x-y-z).

    Returns:
        tuple[np.ndarray, np.ndarray]: Position (3,) and quaternion (x, y, z, w).
    """
    values_ = [slider.value for slider in sliders]
    return np.array(values_[:3]), Rotation.from_euler("xyz", values_[3:], degrees=True).as_quat()


def _add_frame_markers(ctx: PluginContext, serial: str, arm: ArmInterface) -> dict[str, viser.FrameHandle]:
    """Create one arm's frame-check markers in the 3D view, hidden until needed.

    Args:
        ctx: This plugin's context.
        serial: The robot's serial.
        arm: The arm.

    Returns:
        dict[str, viser.FrameHandle]: One marker per FRAME_MARKERS name.
    """
    root = f"{ctx.view.scene_root}/{serial}/{arm.config.name}"
    return {name: ctx.view.scene.add_frame(f"{root}/{name}", axes_length=length, axes_radius=0.004,
                                           origin_radius=0.012, origin_color=color, visible=False)
            for name, _, color, length in FRAME_MARKERS}


def _add_force_arrows(ctx: PluginContext, serial: str, arm: ArmInterface) -> dict[str, viser.ArrowsHandle]:
    """Create one arm's force arrows in the 3D view, hidden until there is a force to show.

    Args:
        ctx: This plugin's context.
        serial: The robot's serial.
        arm: The arm.

    Returns:
        dict[str, viser.ArrowsHandle]: One arrow per FORCE_ARROWS name.
    """
    root = f"{ctx.view.scene_root}/{serial}/{arm.config.name}"
    return {name: ctx.view.scene.add_arrows(f"{root}/{name}", points=np.zeros((1, 2, 3)), colors=color,
                                            shaft_radius=0.004, head_radius=0.012, head_length=0.03, visible=False)
            for name, _, color in FORCE_ARROWS}


def _pose_plan_line(widgets: _ArmWidgets) -> str:
    """What Start would do from here: distance, turn and duration of the Cartesian move."""
    state = widgets.arm.state
    if state.tcp_position is None:
        return note("no TCP pose yet")
    position, orientation = widgets.arm.from_husky(*_pose_from_sliders(widgets.pose))
    duration, _ = cartesian_move(state.tcp_position, state.tcp_orientation, position, orientation)
    distance = float(np.linalg.norm(position - state.tcp_position))
    turn = float(np.degrees((Rotation.from_quat(state.tcp_orientation).inv()
                             * Rotation.from_quat(orientation)).magnitude()))
    return block(values(f"Δ {distance:6.3f} m {turn:6.1f} °   T {duration:5.1f} s",
                        f"(≤{CARTESIAN_MOVE_MAX_SPEED * 100:.0f} cm/s, "
                        f"≤{np.degrees(CARTESIAN_MOVE_MAX_ROTATION_SPEED):.0f} °/s, "
                        f"≥{CARTESIAN_MOVE_MIN_DURATION:.0f} s)"))


def _show_only_running(panel: _ControllerPanel) -> None:
    """Show the running controller's inputs and hide the others."""
    for controller, folder in panel.inputs.items():
        folder.visible = controller == panel.controllers.state.active


def _controller_chip(controllers: ControllerManagerInterface) -> str:
    """The running controller, coloured by what the last switch did."""
    state = controllers.state
    if state.switch_in_flight:
        return chip(f"→ {CONTROLLER_LABELS.get(state.switch_in_flight, state.switch_in_flight)}", BUSY)
    if state.switch_error:
        return chip("switch failed", FAIL, state.switch_error)
    if state.active is None:
        return chip("no ctrl", NONE, "no controller running, or no answer yet")
    return chip(CONTROLLER_LABELS.get(state.active, state.active), OK)


def _base_status(base: BaseInterface, now: float) -> str:
    """Status HTML for a base: chips, then its pose (placeholders before the first fix)."""
    state = base.state
    # * The same mocap chip as the health panel and every tracked object.
    chips = _controller_chip(base.controllers) + check_chip(mocap_check("mocap", base.mocap_id, state, now))
    # None before the first valid fix.
    yaw = None if state.orientation is None else Rotation.from_quat(state.orientation).as_euler(
        "xyz", degrees=True)[2:]
    stale = not state.tracked or now - state.last_fix_time >= STALE_AFTER
    return block(chips + values(f"xyz {numbers(state.position, 3, 7, 3)} m",
                                f"yaw {numbers(yaw, 1, 7, 1)} °", dim=stale))


def _arm_status(arm: ArmInterface, now: float) -> str:
    """Status HTML for an arm: chips, then joints, TCP and force (placeholders when missing)."""
    state = arm.state
    chips = _controller_chip(arm.controllers) + freshness_chip("joints", state.last_update_time, now)
    if state.is_executing:
        chips += chip("moving", BUSY)
    joints = arm.joint_vector()
    stale = state.last_update_time is None or now - state.last_update_time >= STALE_AFTER
    return block(chips + values(
        f"q   {numbers(None if joints is None else np.degrees(joints), 6, 6, 1)} °",
        f"tcp {numbers(state.tcp_position, 3, 6, 3)} m",
        f"F   {numbers(None if state.wrench is None else state.wrench[:3], 3, 6, 1)} N",
        dim=stale))


def _build_robotiq_inputs(ctx: PluginContext, gui: viser.GuiApi, label: str, gripper: RobotiqGripper) -> None:
    """Add the Robotiq-only inputs under Grip: a target opening with a force limit, and Reactivate.

    ? Open and Close always use DEFAULT_EFFORT; the force slider applies only to Go.

    Args:
        ctx: This plugin's context.
        gui: The GUI api, inside the arm's tab.
        label: "<serial> <arm name>", for intent names.
        gripper: The gripper to command.
    """
    position = gui.add_slider("pos rad", min=gripper.OPEN_POSITION, max=gripper.CLOSED_POSITION, step=0.01,
                              initial_value=gripper.OPEN_POSITION,
                              hint=f"Knuckle angle: {gripper.OPEN_POSITION} open, {gripper.CLOSED_POSITION} closed.")
    force = gui.add_slider("force", min=0.0, max=gripper.MAX_EFFORT, step=0.05, initial_value=gripper.DEFAULT_EFFORT,
                           hint="Force limit, as a fraction of the gripper's full force.")
    target = gui.add_button_group("Target", ["Go"], hint="Move the fingers to the slider position.")
    # ! Sliders are read when the intent runs, on the main thread.
    target.on_click(ctx.defer(f"gripper target {label}", lambda: gripper.move(position.value, force.value)))
    driver = gui.add_button_group("Driver", ["Reactivate"],
                                  hint="After a fault or power loss. The gripper opens and closes once: "
                                       "hold nothing in it.")
    driver.on_click(ctx.defer(f"reactivate gripper {label}", gripper.reactivate))


def _tool_status(state: object, now: float) -> str:
    """Status HTML for an end effector: chips, then its numbers, one layout per tool kind."""
    if isinstance(state, RobotiqState):
        chips = freshness_chip("joints", state.last_update_time, now)
        if state.reactivating:
            chips += chip("reactivating", BUSY)
        elif state.reactivate_error:
            chips += chip("reactivate failed", FAIL, state.reactivate_error)
        if state.moving:
            chips += chip("moving", BUSY)
        elif state.last_result_ok is False:
            chips += chip("short of target", FAIL, "the last command did not reach its target")
        # From the fingers' position, or the target until the first joint state.
        shown = state.position if state.position is not None else state.commanded_position
        if shown is not None:
            chips += chip("closed" if shown > RobotiqGripper.CLOSED_POSITION / 2 else "open", OK)
        stale = state.last_update_time is None or now - state.last_update_time >= STALE_AFTER
        measured = None if state.position is None else [state.position]
        commanded = None if state.commanded_position is None else [state.commanded_position]
        effort = None if state.commanded_effort is None else [state.commanded_effort]
        return block(chips + values(f"pos {numbers(measured, 1, 6, 3)} rad",
                                    f"cmd {numbers(commanded, 1, 6, 3)} rad  force {numbers(effort, 1, 5, 2)}",
                                    dim=stale))
    if isinstance(state, ScaffoldingV3State):
        chips = freshness_chip("status", state.last_update_time, now)
        if state.last_update_time is not None:  # every field is set together
            chips += _motor_chip("grip", state.gripper_motor) + _motor_chip("screw", state.joint_motor)
        stale = state.last_update_time is None or now - state.last_update_time >= STALE_AFTER
        current = "—".rjust(6) if state.current is None else f"{state.current:6d}"
        pwm = "—".rjust(4) if state.pwm_pct is None else f"{state.pwm_pct:4d}"
        return block(chips + values(f"I   {current} mA   pwm {pwm} %", dim=stale))
    if isinstance(state, ScaffoldingV1State):
        chips = freshness_chip("io", state.last_update_time, now)
        if state.last_request_ok is False:
            chips += chip("set_io failed", FAIL, "the arm refused the last output change")
        # Outputs as the UR reports them; the tool reports nothing itself.
        if state.gripper_closed is not None:
            chips += chip("grip closed" if state.gripper_closed else "grip open", OK)
        if state.screw_on is not None:
            chips += chip("screw on", BUSY) if state.screw_on else chip("screw off", OK)
        return block(chips)
    return ""


def _motor_chip(name: str, motor_state: str | None) -> str:
    """One v3 motor's chip: busy while it runs, red when stalled, green when idle."""
    color = {ScaffoldingV3.IDLE: OK, ScaffoldingV3.STALLED: FAIL}.get(motor_state, BUSY)
    hint = "stalled: Stop clears it before it runs again" if motor_state == ScaffoldingV3.STALLED else ""
    return chip(f"{name} {str(motor_state).lower()}", color, hint)
