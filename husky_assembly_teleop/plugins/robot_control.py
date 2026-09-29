"""
The robot control panel: every connected robot's state, and basic control of it.

* Start here when writing a plugin. This one is kept small on purpose, so the
  pattern is easy to see. Every plugin follows the same five rules:

  1. `setup` builds every widget once and keeps the handles. Nothing is built
     later. viser keeps widgets alive, so a later change is an assignment.
  2. A widget callback never does the work itself. It goes through
     `ctx.defer` (or `ctx.defer_value`, when the work needs to know which
     button of a group was clicked), which runs the work on the ROS thread at
     the start of this plugin's next step. viser calls callbacks on its own
     threads, where touching robots or PyBullet is not safe.
  3. `update` does whatever has to happen every tick. Here that is streaming
     the base twist, since the base stops if cmd_vel goes quiet.
  4. Anything that takes longer than a tick is a job: a generator started with
     `ctx.spawn`, which waits by yielding. Here that is executing a trajectory.
  5. `draw` copies state into the widgets and does nothing else. Plain
     assignments, every tick: viser only sends real changes.

  Robots are only commanded through their interfaces (ctx.world.robots), and
  only read through their state. This plugin knows no topic names.

Layout: one panel beside the main one, with a tab per robot ("0804", "0806").
Inside each, inline tabs per part (Base, Left, Right). A robot tab can be
dragged out into a panel of its own. In each part tab:

  status        coloured chips (controller, freshness, executing) and values
  CTRL          one button per controller; below it, the running controller's inputs
  SENSOR/TOOL   actions that do not depend on the controller

* The look (chips, section bars, number rows) comes from ui_style.py, shared
  by every plugin so they read alike.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import numpy as np
import pybullet_planning as pp
import viser
from scipy.spatial.transform import Rotation

from ..concurrency import Job, Task, WaitTimeout, wait_until
from ..context import PluginContext
from ..plugin import HuskyPlugin, register
from ..robot_interface import ArmInterface, BaseInterface, ScaffoldingV1
from ..robot_interface.end_effectors import RobotiqState, ScaffoldingV1State, ScaffoldingV3State
from ..robot_interface.arm import (CARTESIAN_COMPLIANCE_CONTROLLER,
                                   CARTESIAN_MOVE_MAX_ROTATION_SPEED,
                                   CARTESIAN_MOVE_MAX_SPEED, CARTESIAN_MOVE_MIN_DURATION, JOINT_MOVE_MAX_SPEED,
                                   JOINT_MOVE_MIN_DURATION, MAX_TARGET_FORCE, SCALED_JOINT_TRAJECTORY_CONTROLLER,
                                   TARGET_WRENCH_FRAME,
                                   cartesian_move, joint_move)
from ..robot_interface.base import PLATFORM_VELOCITY_CONTROLLER
from ..robot_interface.controller_manager import ControllerManagerInterface
from ..robot_interface.ur_frames import STOCK_YAW
from ..visualization import quaternion_to_wxyz
from ..ui_style import (STALE_AFTER, BUSY, FAIL, NONE, OK, SECTION_CTRL, SECTION_SENSOR, SECTION_TOOL, block, chip,
                        freshness_chip, note, numbers, section, values, warning)

#: The base's speed at a Speed of 1. Kept low: this is a test panel, not a joystick.
MAX_LINEAR_SPEED = 0.3   # m/s
MAX_ANGULAR_SPEED = 0.5  # rad/s
#: The hold buttons: label, icon, and the twist they ask for, as signs of the
#: linear and angular speed. Left turns counter-clockwise (positive ω).
#: In D-pad order: Forward alone on the top row, then Left, Back, Right.
HOLD_BUTTONS = (("", viser.Icon.ARROW_UP, 1.0, 0.0),
                ("", viser.Icon.ROTATE, 0.0, 1.0),
                ("", viser.Icon.ARROW_DOWN, -1.0, 0.0),
                ("", viser.Icon.ROTATE_CLOCKWISE, 0.0, -1.0))
#: Each hold button's D-pad cell, (row, column), in HOLD_BUTTONS order.
DPAD_CELLS = ((1, 2), (2, 1), (2, 2), (2, 3))
#: How often the browser reports a held button, Hz. Matches the 20 Hz tick.
HOLD_CALLBACK_HZ = 20.0
#: A hold counts as released once no report has come for this long, seconds.
#: viser never says "released", so this is the dead man: also what stops the
#: base when the browser disconnects mid-hold. Five reports' worth, for jitter.
HOLD_TIMEOUT = 0.25
#: Short slider labels for the six UR joints, in UR_JOINT_NAMES order, and each
#: joint's range in degrees. The UR5e allows +-360 everywhere but the elbow.
JOINT_SLIDERS = (("pan", 360), ("lift", 360), ("elbow", 180), ("w1", 360), ("w2", 360), ("w3", 360))
#: Cartesian target sliders: TCP position range, metres, around the arm's base,
#: and its step; orientation is roll, pitch, yaw in degrees.
POSITION_RANGE, POSITION_STEP = 1.5, 0.001
POSE_SLIDERS = (("x m", "p"), ("y m", "p"), ("z m", "p"), ("roll °", "a"), ("pitch °", "a"), ("yaw °", "a"))
#: Poses drawn in the 3D view while the Cartesian controller runs: name, label,
#: origin colour (r, g, b) and axis length (m). Both are placed through the
#: controller's own base_link (see `_update_markers`).
FRAME_MARKERS = (
    ("target", "target (sliders)", (255, 200, 0), 0.12),
    ("reported", "TCP reported", (40, 170, 60), 0.08),
)
#: Force arrows in the 3D view, drawn from the reported TCP: the sliders'
#: force (preview) and the force last applied, with their colours, and how long
#: an arrow is per newton.
FORCE_ARROWS = (
    ("force_preview", "force (sliders)", (255, 200, 0)),
    ("force_applied", "force applied", (255, 110, 0)),
)
FORCE_ARROW_SCALE = 0.01  # metres per newton: 20 N draws 20 cm
#: Target force sliders, newtons: each axis within the arm's MAX_TARGET_FORCE.
FORCE_STEP = 0.5
#: A move counts as arrived when every joint is this close to its target, degrees.
ARRIVED_TOLERANCE = 0.5
#: Extra seconds, past the planned duration, before a move that has not arrived
#: is reported as timed out.
ARRIVE_TIMEOUT_MARGIN = 5.0

#: Starting width of the robot panel, pixels. Sized for its widest line, the
#: arm's joint row: 47 monospace characters at 11 px (~310 px), plus the
#: readout indent and the padding of two tab levels and the panel. The operator
#: can still drag it wider or narrower.
PANEL_WIDTH = 420

#: Short button labels for controllers. Anything not listed shows its full name.
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
# ? Plain containers for the handles built in setup, so draw can find them. The
#   plugin's actual state (what is being driven, which job runs) lives on the
#   plugin itself, not here.

@dataclass
class _ControllerPanel:
    """Controller buttons for one controller manager, and each controller's inputs.

    The base and every arm get one of these, built by the same function, because
    they all have a controller manager that works the same way.

    Attributes:
        controllers: The controller manager this panel shows and switches.
        inputs: One folder of inputs per controller. Only the running
            controller's folder is visible, so an input can only reach a
            controller that will act on it.
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

        #: Bases whose twist was being streamed on the last tick, by serial. Lets
        #: update notice when a hold button is released and send one final stop.
        self._driving: set[str] = set()

        #: The last hold report per base, by serial: the signs of the twist the
        #: held button asks for, and when the report arrived (ctx.now()).
        self._held: dict[str, tuple[float, float, float]] = {}

        #: The running trajectory job per arm, keyed by (serial, arm name). One
        #: at a time per arm: a second press while one runs is ignored.
        self._trajectory_jobs: dict[tuple[str, str], Job] = {}

        #: Arms whose joint sliders should be set at the next draw, keyed like
        #: `_trajectory_jobs`, with the angles to set (radians), or None for
        #: the measured joints. Filled by the Current and Stow buttons -- and at
        #: startup with None, so the sliders show where the arm is, not zero,
        #: before anyone presses Start. Nothing else touches the sliders.
        self._load_sliders: dict[tuple[str, str], np.ndarray | None] = {}

        #: Arms whose Cartesian pose sliders should be set to the measured TCP
        #: at the next update: filled by Load Current, and by every arm at start.
        self._load_pose: set[tuple[str, str]] = set()

        #: The target force last sent to each arm's compliance controller, N, or
        #: absent if none was. The controller keeps it until told otherwise, so
        #: the panel has to remember and show it.
        self._applied_force: dict[tuple[str, str], np.ndarray] = {}

        #: This tick's 3D markers per arm, keyed like `_trajectory_jobs`: the
        #: world pose of each FRAME_MARKERS entry, and the start and end of each
        #: FORCE_ARROWS entry, that has a value. Absent while the compliance
        #: controller is not running on that arm.
        self._markers: dict[tuple[str, str], dict[str, tuple]] = {}

    # --- --- --- --- --- SETUP --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Build one panel with a tab per robot, holding inline tabs for its base and arms.

        Args:
            ctx: This plugin's context.
        """
        # Straight to the gui api: this plugin uses only its own panels, so it
        # does not open ui(), which would leave an empty folder in the main panel.
        gui = ctx.view.gui
        # * Two levels of tabs: one panel beside the main one with a tab per
        #   robot, and inside each robot tab an inline tab per part (base, each
        #   arm). The top bar stays one tab per robot however many arms there
        #   are, and dragging a robot tab out gives that robot a panel of its
        #   own -- parts and all, since inline tabs cannot be dragged apart.
        panel = ctx.view.panel()
        for serial, robot in ctx.world.robots.items():
            # "a200-0806" -> "0806": the digits are what people say.
            with panel.add_tab(serial.split("-")[-1], icon=viser.Icon.ROBOT):
                parts = gui.add_tab_group()
                with parts.add_tab("Base", icon=viser.Icon.CAR):
                    self._bases.append(self._build_base(ctx, gui, serial, robot.base))
                for arm_name, arm in robot.arms.items():
                    with parts.add_tab(ARM_LABELS.get(arm_name, arm_name), icon=viser.Icon.ROBOT):
                        self._arms.append(self._build_arm(ctx, gui, serial, arm))
        panel.dock_right()
        panel.set_width(PANEL_WIDTH)

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
            """Inputs for platform_velocity_controller: hold buttons, driving only while pressed."""
            handles["speed"] = gui.add_slider("Speed", min=0.05, max=1.0, step=0.05, initial_value=0.5,
                                              hint="Fraction of the full speed "
                                                   f"({MAX_LINEAR_SPEED} m/s, {MAX_ANGULAR_SPEED} rad/s).")
            grid = gui.add_html("")
            with gui.add_folder("Drive"):
                buttons = [gui.add_button(label, icon=icon, hint="Drives while held.")
                           for label, icon, _, _ in HOLD_BUTTONS]
            grid.content = _dpad_style([button._impl.uuid for button in buttons])
            for button, (label, _, linear, angular) in zip(buttons, HOLD_BUTTONS):
                # ! Bind the loop values as defaults, or every button would drive like the last one.
                button.on_hold(ctx.defer(f"hold {label} {serial}",
                                         lambda linear=linear, angular=angular: self._on_hold(ctx, serial,
                                                                                              linear, angular)),
                               callback_hz=HOLD_CALLBACK_HZ)

        panel = _build_controller_panel(ctx, gui, base.controllers,
                                        {PLATFORM_VELOCITY_CONTROLLER: velocity_inputs})
        return _BaseWidgets(serial=serial, base=base, panel=panel, status=status, speed=handles["speed"])

    def _on_hold(self, ctx: PluginContext, serial: str, linear: float, angular: float) -> None:
        """Note that a hold button is still pressed. Runs as an intent, on the ROS thread.

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
        self._load_pose.add(key)
        handles: dict[str, object] = {}

        def trajectory_inputs() -> None:
            """Inputs for the joint trajectory controller: a target per joint, and Send."""
            gui.add_html(warning("no collision checking · raw joint trajectories"))
            handles["joints"] = [gui.add_slider(f"{name} °", min=-limit, max=limit, step=0.5, initial_value=0.0)
                                 for name, limit in JOINT_SLIDERS]
            handles["plan"] = gui.add_html("")
            # Loading only sets the sliders; nothing moves until Start.
            load = gui.add_button_group("Load", ["Current", "Stow"],
                                        hint="Set the sliders to where the arm is, or to its stow pose.")
            move = gui.add_button_group("Move", ["Start", "Hold"],
                                        hint="Start: move to the sliders, smoothly, in 5 s or more. "
                                             "Hold: brake to a standstill where the arm is.")

            def on_load(clicked: str) -> None:
                """Load the sliders. Runs as an intent, on the ROS thread."""
                if clicked == "Current":
                    self._load_sliders[key] = None
                elif arm.config.stow_joints is None:
                    ctx.log_warn(f"{label}: no stow pose configured (ArmConfig.stow_joints)")
                else:
                    self._load_sliders[key] = np.array(arm.config.stow_joints)

            def on_move(clicked: str) -> None:
                """Start or Hold. Runs as an intent, on the ROS thread."""
                if clicked == "Hold":
                    self._hold_arm(serial, arm)
                else:
                    # ! Slider values are read here, when the intent runs: they
                    #   are settings, and this is the moment the operator committed.
                    target = np.radians([slider.value for slider in handles["joints"]])
                    self._start_move(ctx, serial, arm, target)

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

            # ! In the tool frame: see TARGET_WRENCH_FRAME in arm.py for why.
            gui.add_html(section("force", SECTION_CTRL, f"in {TARGET_WRENCH_FRAME} frame"))
            limit = MAX_TARGET_FORCE / np.sqrt(3.0)  # all three at their limit stay within MAX_TARGET_FORCE
            handles["force"] = [gui.add_slider(f"F{axis} N", min=-limit, max=limit, step=FORCE_STEP, initial_value=0.0,
                                               hint=f"Along the tool's own {axis} axis ({TARGET_WRENCH_FRAME}).")
                                for axis in "xyz"]
            handles["wrench_line"] = gui.add_html("")
            wrench = gui.add_button_group("Force", ["Apply", "Zero"],
                                          hint=f"Apply: push with this force, in the tool frame "
                                               f"({TARGET_WRENCH_FRAME}), until changed. Zero: remove it.")

            # Legend for what the 3D view shows while this controller runs.
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
                """Start or Hold. Runs as an intent, on the ROS thread."""
                if clicked == "Hold":
                    self._hold_arm(serial, arm)
                else:
                    position, orientation = arm.from_husky(*_pose_from_sliders(handles["pose"]))
                    self._start_cartesian_move(ctx, serial, arm, position, orientation)

            def on_force(clicked: str) -> None:
                """Apply or remove the target force. Runs as an intent."""
                if clicked == "Zero":
                    force = np.zeros(3)
                    # The sliders go to zero too, so a later Apply does not bring the old force back.
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

        # Does not depend on which controller runs.
        gui.add_html(section("sensor", SECTION_SENSOR))
        zero = gui.add_button_group("FT", ["Zero"], hint="Tares the current load. Only with nothing gripped.")
        zero.on_click(ctx.defer(f"zero FT {label}", arm.zero_ft_sensor))

        tool_status = None
        if arm.end_effector is not None:
            kind = arm.config.end_effector
            gui.add_html(section("tool", SECTION_TOOL, TOOL_LABELS.get(kind, kind)))
            tool_status = gui.add_html("")
            if isinstance(arm.end_effector, ScaffoldingV1):
                # ! Its commands raise until it is ported, and a raising intent
                #   counts against this plugin. So offer no buttons at all.
                gui.add_html(note("not ported yet"))
            else:
                commands = {"Open": arm.end_effector.open, "Close": arm.end_effector.close,
                            "Stop": arm.end_effector.stop}
                grip = gui.add_button_group("Grip", list(commands))
                grip.on_click(ctx.defer_value(f"grip {label}", lambda clicked: commands[clicked]()))

        return _ArmWidgets(serial=serial, arm=arm, panel=panel, status=status, tool_status=tool_status,
                           joints=handles["joints"], plan=handles["plan"], pose=handles["pose"],
                           pose_plan=handles["pose_plan"], force=handles["force"],
                           wrench_line=handles["wrench_line"],
                           frames=_add_frame_markers(ctx, serial, arm), arrows=_add_force_arrows(ctx, serial, arm))

    def teardown(self, ctx: PluginContext) -> None:
        """Stop any base this plugin was driving, so it does not keep rolling.

        Args:
            ctx: This plugin's context.
        """
        for widgets in self._bases:
            if widgets.serial in self._driving:
                widgets.base.send_twist(0.0, 0.0)

    # --- --- --- --- --- EVERY TICK --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Stream the twist of every base with a hold button pressed.

        ? Why every tick. The base stops by itself when cmd_vel goes quiet, which
          is the safe behaviour we want, so a twist has to be repeated for as
          long as the operator wants motion.

        Args:
            ctx: This plugin's context.
        """
        now = ctx.now()
        for widgets in self._bases:
            held = self._held.get(widgets.serial)
            # A controller that stopped cannot be driven, so its hold is dropped.
            if held is not None and (now - held[2] > HOLD_TIMEOUT
                                     or not widgets.base.controllers.is_active(PLATFORM_VELOCITY_CONTROLLER)):
                del self._held[widgets.serial]
                held = None
            if held is not None:
                speed = widgets.speed.value
                widgets.base.send_twist(held[0] * speed * MAX_LINEAR_SPEED, held[1] * speed * MAX_ANGULAR_SPEED)
                self._driving.add(widgets.serial)
            elif widgets.serial in self._driving:
                # Just released: one explicit stop, rather than waiting for the timeout.
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
                widgets.tool_status.content = _tool_status(widgets.arm.state.end_effector)

    # --- --- --- --- --- JOBS --- --- --- --- ---
    # Started from intents, so always on the ROS thread.

    def _start_move(self, ctx: PluginContext, serial: str, arm: ArmInterface, target: np.ndarray) -> None:
        """Start the move job for one arm, unless one is already running.

        Args:
            ctx: This plugin's context.
            serial: The robot's serial.
            arm: The arm to move.
            target: Joint angles to move to, radians, UR_JOINT_NAMES order.
        """
        key = (serial, arm.config.name)
        running = self._trajectory_jobs.get(key)
        if running is not None and not running.done:
            ctx.log_warn(f"{serial} {arm.config.name}: a move is already running; Send ignored")
            return
        self._trajectory_jobs[key] = ctx.spawn(f"move {serial} {arm.config.name}",
                                               self._execute_move(ctx, serial, arm, target))

    def _hold_arm(self, serial: str, arm: ArmInterface) -> None:
        """Hold the arm where it is, and end the move job that was waiting for it to arrive.

        ! The hold goes out first and unconditionally -- also when no job of
          ours runs, since a trajectory may come from somewhere else.

        Args:
            serial: The robot's serial.
            arm: The arm to hold.
        """
        arm.hold()
        running = self._trajectory_jobs.get((serial, arm.config.name))
        if running is not None and not running.done:
            running.cancel()

    def _execute_move(self, ctx: PluginContext, serial: str, arm: ArmInterface, target: np.ndarray) -> Task:
        """Send a smooth joint move and wait until the arm has arrived.

        ? Arrival is judged from the joints, not from `is_executing`: the arm
          has arrived when the planned time is up and every joint is within
          ARRIVED_TOLERANCE of its target.

        Args:
            ctx: This plugin's context.
            serial: The robot's serial, for messages.
            arm: The arm to move.
            target: Joint angles to move to, radians.

        Yields:
            None: Once per tick while the arm moves.
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
            yield from wait_until(ctx, arrived, timeout_s=duration + ARRIVE_TIMEOUT_MARGIN,
                                  description=f"{name} to arrive")
            ctx.log_info(f"{name}: arrived")
        except WaitTimeout as timeout:
            # Reported, not raised: a raising job counts against the plugin.
            ctx.log_warn(str(timeout))
        # ! The sliders are left alone, however the move ends: they are the
        #   operator's target, and only Current or Stow change them.

    def _start_cartesian_move(self, ctx: PluginContext, serial: str, arm: ArmInterface,
                              position: np.ndarray, orientation: np.ndarray) -> None:
        """Start the Cartesian move job for one arm, unless any move is already running.

        ! Shares `_trajectory_jobs` with joint moves: one motion per arm at a
          time, whichever controller it is for, and Hold ends either.

        Args:
            ctx: This plugin's context.
            serial: The robot's serial.
            arm: The arm to move.
            position: TCP target position, metres, arm base frame.
            orientation: TCP target orientation, quaternion (x, y, z, w).
        """
        key = (serial, arm.config.name)
        running = self._trajectory_jobs.get(key)
        if running is not None and not running.done:
            ctx.log_warn(f"{serial} {arm.config.name}: a move is already running; Start ignored")
            return
        self._trajectory_jobs[key] = ctx.spawn(f"cartesian move {serial} {arm.config.name}",
                                               self._stream_cartesian_move(ctx, serial, arm, position, orientation))

    def _stream_cartesian_move(self, ctx: PluginContext, serial: str, arm: ArmInterface,
                               position: np.ndarray, orientation: np.ndarray) -> Task:
        """Move the TCP to a target by sending one close target per tick.

        ? Why streamed. The compliance controller pulls straight towards its
          target, with no path or speed of its own, so a far target is a jump.
          This sends the next point of a smooth `cartesian_move` every tick
          instead, each one close to where the TCP already is.

        Args:
            ctx: This plugin's context.
            serial: The robot's serial, for messages.
            arm: The arm to move.
            position: TCP target position, metres, arm base frame.
            orientation: TCP target orientation, quaternion (x, y, z, w).

        Yields:
            None: Once per tick while the TCP moves.
        """
        name = f"{serial} {arm.config.name}"
        if arm.state.tcp_position is None:
            ctx.log_warn(f"{name}: no TCP pose yet, nothing sent")
            return
        duration, sample = cartesian_move(arm.state.tcp_position, arm.state.tcp_orientation,
                                          position, orientation)
        if arm.config.cartesian_test_mode:
            # ! Test mode: one target, at the TCP, so every check still runs and
            #   the arm logs its TEST MODE banner. The move itself is not streamed;
            #   the yellow target marker and the plan line show where it would go.
            arm.send_cartesian_target(*sample(0.0))
            ctx.log_warn(f"{name}: TEST MODE, planned cartesian move of {duration:.1f} s to "
                         f"{np.round(position, 3)} m was not streamed")
            return
        ctx.log_info(f"{name}: cartesian move, {duration:.1f} s")
        start = ctx.now()
        while True:
            elapsed = ctx.now() - start
            if not arm.send_cartesian_target(*sample(elapsed)):
                # The arm logged why. Its last accepted target was close to the
                # TCP, so it settles there; a hold makes that explicit.
                ctx.log_warn(f"{name}: cartesian move stopped early")
                arm.hold()
                return
            if elapsed >= duration:
                break
            yield
        ctx.log_info(f"{name}: cartesian move done")

    def _update_markers(self, ctx: PluginContext, widgets: _ArmWidgets) -> None:
        """Place one arm's 3D markers: target, reported TCP and force arrows. Runs every tick.

        ? Where they are placed. The compliance controller reads targets in its
          own `base_link` -- the stock ur_description one the robot runs, which
          sits on the arm's physical base (`base_link_inertia`) turned back by
          the stock 180 deg about z. That frame is rebuilt here the same way, so
          what is drawn is where the controller will take it. Background:
          doc/ur_frames.md.

        Args:
            ctx: This plugin's context. Plugin hooks run with the scene active,
                so `pp` calls here act on it.
            widgets: One arm's handles.
        """
        key = (widgets.serial, widgets.arm.config.name)
        arm, state = widgets.arm, widgets.arm.state
        body = ctx.scene.robots.get(widgets.serial)
        if body is None or not arm.controllers.is_active(CARTESIAN_COMPLIANCE_CONTROLLER):
            self._markers.pop(key, None)
            return
        inertia = pp.get_link_pose(body, pp.link_from_name(body, f"{arm.config.name}_base_link_inertia"))
        base = pp.multiply(inertia, ((0.0, 0.0, 0.0), tuple(Rotation.from_euler("z", -STOCK_YAW).as_quat())))
        markers: dict[str, tuple] = {}
        local_poses = {"target": arm.from_husky(*_pose_from_sliders(widgets.pose))}
        if state.tcp_position is not None:
            local_poses["reported"] = (state.tcp_position, state.tcp_orientation)
        for name, (position, orientation) in local_poses.items():
            markers[f"{name}_world"] = pp.multiply(base, (tuple(position), tuple(orientation)))
        # Forces are in tool0, so they turn with the reported TCP and start at it.
        # No arrow without a TCP, or for a zero force.
        tcp = markers.get("reported_world")
        forces = {"force_preview": np.array([slider.value for slider in widgets.force]),
                  "force_applied": self._applied_force.get(key)}
        for name, force in forces.items():
            if tcp is not None and force is not None and np.linalg.norm(force) > 0.0:
                start = np.array(tcp[0])
                markers[f"{name}_world"] = (start, start + Rotation.from_quat(tcp[1]).apply(force) * FORCE_ARROW_SCALE)
        self._markers[key] = markers

    def _load_pose_sliders(self, widgets: _ArmWidgets) -> None:
        """Set one arm's pose sliders to its TCP, if Load Current (or startup) asked for it.

        Args:
            widgets: One arm's handles.
        """
        key = (widgets.serial, widgets.arm.config.name)
        state = widgets.arm.state
        if key not in self._load_pose or state.tcp_position is None:
            return  # nothing asked, or no TCP yet: try again next tick
        position, orientation = widgets.arm.to_husky(state.tcp_position, state.tcp_orientation)
        angles = Rotation.from_quat(orientation).as_euler("xyz", degrees=True)
        for slider, value in zip(widgets.pose, [*position, *angles]):
            slider.value = round(float(value) / slider.step) * slider.step
        self._load_pose.discard(key)

    def _load_joint_sliders(self, widgets: _ArmWidgets) -> None:
        """Set one arm's joint sliders, if Current or Stow (or startup) asked for it.

        ! Called from update, not draw. Loading is the answer to a button, and
          draw is skipped while the panels are frozen -- a Stow pressed then
          would otherwise leave the old target in the sliders for Start to send.

        Args:
            widgets: One arm's handles.
        """
        key = (widgets.serial, widgets.arm.config.name)
        if key not in self._load_sliders:
            return
        # None means "where the arm is", which has to wait for joint data.
        angles = widgets.arm.joint_vector() if self._load_sliders[key] is None else self._load_sliders[key]
        if angles is not None:
            for slider, value in zip(widgets.joints, np.degrees(angles)):
                slider.value = round(float(value) * 2) / 2  # the sliders' 0.5 deg step
            del self._load_sliders[key]


# --- --- --- --- --- SHARED BUILDING BLOCKS --- --- --- --- ---

def _build_controller_panel(ctx: PluginContext, gui: viser.GuiApi,
                            controllers: ControllerManagerInterface,
                            input_builders: dict[str, Callable[[], None]]) -> _ControllerPanel:
    """Build the CTRL section: one button per controller, and a folder of inputs for each.

    Every controller the manager may switch to gets a folder, even one with no
    inputs yet, so there is always an obvious place to add them.

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

    ? Why CSS. viser has no grid or row container: every button is a full-width
      row of its own. The button carries its uuid as its HTML id, sitting in a
      wrapper div, and the Drive folder holds nothing else, so the folder's
      content div can be made a three-column grid with Forward in the middle.
      Needs `:has()`, which every current browser has.

    Args:
        uuids: The buttons' uuids, Forward first.

    Returns:
        str: A <style> element, for an html widget outside the Drive folder
            (inside, it would take a grid cell).
    """
    forward = f'[id="{uuids[0]}"]'
    grid = f"div:has(> div > {forward}) {{ display: grid; grid-template-columns: repeat(3, 1fr); }}"
    # ! Every button gets its cell explicitly: left to the grid, Left would fill
    #   the empty cell beside Forward instead of starting the second row.
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
    chips = _controller_chip(base.controllers) + freshness_chip("mocap", state.last_update_time, now)
    # Only worth a chip when mocap is talking but says the pose is invalid; with
    # no data at all the gray mocap chip already says it.
    if state.last_update_time is not None and not state.tracked:
        chips += chip("untracked", FAIL)
    # None before the first valid fix: numbers() then shows dashes.
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


def _tool_status(state: object) -> str:
    """Status HTML for an end effector: chips, then its numbers, one layout per tool kind."""
    if isinstance(state, RobotiqState):
        if state.commanded_position is None:
            chips = chip("idle", NONE, "nothing sent yet")
        elif state.moving:
            chips = chip("moving", BUSY)
        else:
            chips = chip("closed" if state.commanded_position > 0.4 else "open",
                         OK if state.last_result_ok else FAIL,
                         "" if state.last_result_ok else "did not reach its target")
        position = None if state.commanded_position is None else [state.commanded_position]
        return block(chips + values(f"pos {numbers(position, 1, 6, 3)} rad"))
    if isinstance(state, ScaffoldingV3State):
        seen = state.last_update_time is not None  # every field is set together
        chips = (chip(f"grip {state.gripper_motor}", OK) + chip(f"screw {state.joint_motor}", OK)
                 if seen else chip("tool", NONE, "no status yet"))
        # Current is in the driver's own units; the message does not say which.
        current = "—".rjust(6) if state.current is None else f"{state.current:6d}"
        pwm = "—".rjust(4) if state.pwm_pct is None else f"{state.pwm_pct:4d}"
        return block(chips + values(f"I   {current}   pwm {pwm} %", dim=not seen))
    if isinstance(state, ScaffoldingV1State):
        grip = {None: "grip ?", True: "grip closed", False: "grip open"}[state.gripper_closed]
        screw = {None: "screw ?", True: "screw on", False: "screw off"}[state.screw_on]
        return block(chip(grip, NONE, "last output set; the tool reports nothing back")
                     + chip(screw, NONE, "last output set; the tool reports nothing back"))
    return ""
