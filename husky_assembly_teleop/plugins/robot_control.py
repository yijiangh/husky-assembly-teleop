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

Layout: one folder per robot, one tab per part (base, each arm). In each tab:

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
import viser
from scipy.spatial.transform import Rotation

from ..concurrency import Job, Task, wait_until
from ..context import PluginContext
from ..plugin import HuskyPlugin, register
from ..robot_interface import ArmInterface, BaseInterface, ScaffoldingV1
from ..robot_interface.end_effectors import RobotiqState, ScaffoldingV1State, ScaffoldingV3State
from ..robot_interface.arm import CARTESIAN_COMPLIANCE_CONTROLLER, SCALED_JOINT_TRAJECTORY_CONTROLLER
from ..robot_interface.base import PLATFORM_VELOCITY_CONTROLLER
from ..robot_interface.controller_manager import ControllerManagerInterface
from ..ui_style import (STALE_AFTER, BUSY, FAIL, NONE, OK, SECTION_CTRL, SECTION_SENSOR, SECTION_TOOL, block, chip,
                        freshness_chip, note, numbers, section, values)

#: Limits of the base twist sliders. Kept low: this is a test panel, not a joystick.
MAX_LINEAR_SPEED = 0.3   # m/s
MAX_ANGULAR_SPEED = 0.5  # rad/s
#: How long the stub trajectory takes, and how long to wait for it at most.
STUB_TRAJECTORY_DURATION = 2.0
STUB_TRAJECTORY_TIMEOUT = 10.0

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
    linear: viser.GuiSliderHandle
    angular: viser.GuiSliderHandle
    drive: viser.GuiCheckboxHandle


@dataclass
class _ArmWidgets:
    """Handles for one arm and the tool on it."""

    serial: str
    arm: ArmInterface
    panel: _ControllerPanel
    status: viser.GuiHtmlHandle
    tool_status: viser.GuiHtmlHandle | None


@register
class RobotControlPlugin(HuskyPlugin):
    """Shows each robot's state and gives basic control over it."""

    name = "robot_control"

    def __init__(self):
        """Start with no widgets; setup builds them."""
        self._bases: list[_BaseWidgets] = []
        self._arms: list[_ArmWidgets] = []

        #: Bases whose twist was being streamed on the last tick, by serial. Lets
        #: update notice when "Drive" is unticked and send one final stop.
        self._driving: set[str] = set()

        #: The running trajectory job per arm, keyed by (serial, arm name). One
        #: at a time per arm: a second press while one runs is ignored.
        self._trajectory_jobs: dict[tuple[str, str], Job] = {}

    # --- --- --- --- --- SETUP --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Build one folder per robot, with a tab for the base and each arm.

        Args:
            ctx: This plugin's context.
        """
        with ctx.view.ui() as gui:
            for serial, robot in ctx.world.robots.items():
                # "a200-0806" -> "0806": the digits are what people say.
                with gui.add_folder(serial.split("-")[-1]):
                    tabs = gui.add_tab_group()
                    with tabs.add_tab("Base", icon=viser.Icon.CAR):
                        self._bases.append(self._build_base(ctx, gui, serial, robot.base))
                    for arm_name, arm in robot.arms.items():
                        with tabs.add_tab(ARM_LABELS.get(arm_name, arm_name), icon=viser.Icon.ROBOT):
                            self._arms.append(self._build_arm(ctx, gui, serial, arm))

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
            """Inputs for platform_velocity_controller: a twist, streamed while Drive is on."""
            handles["linear"] = gui.add_slider("v m/s", min=-MAX_LINEAR_SPEED, max=MAX_LINEAR_SPEED,
                                               step=0.01, initial_value=0.0)
            handles["angular"] = gui.add_slider("ω rad/s", min=-MAX_ANGULAR_SPEED, max=MAX_ANGULAR_SPEED,
                                                step=0.01, initial_value=0.0)
            handles["drive"] = gui.add_checkbox("Drive", initial_value=False,
                                                hint="While on, the twist is sent every tick.")
            stop = gui.add_button("Stop", color="red", icon=viser.Icon.HAND_STOP)
            # Stopping only switches Drive off; update then sends the final zero twist.
            stop.on_click(ctx.defer(f"stop base {serial}", lambda: setattr(handles["drive"], "value", False)))

        panel = _build_controller_panel(ctx, gui, base.controllers,
                                        {PLATFORM_VELOCITY_CONTROLLER: velocity_inputs})
        return _BaseWidgets(serial=serial, base=base, panel=panel, status=status,
                            linear=handles["linear"], angular=handles["angular"], drive=handles["drive"])

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

        def trajectory_inputs() -> None:
            """Inputs for the joint trajectory controller. A stub for now."""
            run = gui.add_button_group("Traj", ["Run stub"],
                                       hint="Sends a trajectory that holds the current pose.")
            run.on_click(ctx.defer(f"trajectory {label}", lambda: self._start_trajectory(ctx, serial, arm)))

        def compliance_inputs() -> None:
            """Inputs for the compliance controller. A placeholder for now."""
            # TODO target frame / target wrench inputs, sent with
            #      arm.send_cartesian_target and arm.send_target_wrench.
            gui.add_html(note("target / wrench: todo"))

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

        return _ArmWidgets(serial=serial, arm=arm, panel=panel, status=status, tool_status=tool_status)

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
        """Stream the twist of every base whose Drive box is on.

        ? Why every tick. The base stops by itself when cmd_vel goes quiet, which
          is the safe behaviour we want, so a twist has to be repeated for as
          long as the operator wants motion.

        Args:
            ctx: This plugin's context.
        """
        for widgets in self._bases:
            if widgets.drive.value:
                widgets.base.send_twist(widgets.linear.value, widgets.angular.value)
                self._driving.add(widgets.serial)
            elif widgets.serial in self._driving:
                # Just switched off: one explicit stop, rather than waiting for the timeout.
                widgets.base.send_twist(0.0, 0.0)
                self._driving.discard(widgets.serial)

    def draw(self, ctx: PluginContext) -> None:
        """Copy every robot's state into the widgets.

        Args:
            ctx: This plugin's context.
        """
        now = ctx.now()
        for widgets in self._bases:
            _show_only_running(widgets.panel)
            widgets.status.content = _base_status(widgets.base, now)
            # A controller that stopped cannot be driven; switch Drive off so the operator sees it.
            if widgets.drive.value and not widgets.base.controllers.is_active(PLATFORM_VELOCITY_CONTROLLER):
                widgets.drive.value = False
        for widgets in self._arms:
            _show_only_running(widgets.panel)
            widgets.status.content = _arm_status(widgets.arm, now)
            if widgets.tool_status is not None:
                widgets.tool_status.content = _tool_status(widgets.arm.state.end_effector)

    # --- --- --- --- --- JOBS --- --- --- --- ---
    # Started from intents, so always on the ROS thread.

    def _start_trajectory(self, ctx: PluginContext, serial: str, arm: ArmInterface) -> None:
        """Start the trajectory job for one arm, unless one is already running.

        Args:
            ctx: This plugin's context.
            serial: The robot's serial.
            arm: The arm to move.
        """
        key = (serial, arm.config.name)
        running = self._trajectory_jobs.get(key)
        if running is not None and not running.done:
            ctx.log_warn(f"{serial} {arm.config.name}: a trajectory is already running")
            return
        self._trajectory_jobs[key] = ctx.spawn(f"trajectory {serial} {arm.config.name}",
                                               self._execute_trajectory(ctx, serial, arm))

    def _execute_trajectory(self, ctx: PluginContext, serial: str, arm: ArmInterface) -> Task:
        """Send a trajectory and wait until the arm has finished it.

        TODO stub. The trajectory holds the current pose, so the whole path --
        controller check, start-pose check, send, wait -- runs without the arm
        moving. Replace the waypoints with a planned path once a planner exists.

        Args:
            ctx: This plugin's context.
            serial: The robot's serial, for messages.
            arm: The arm to move.

        Yields:
            None: Once per tick while the arm executes.
        """
        here = arm.joint_vector()
        if here is None:
            ctx.log_warn(f"{serial} {arm.config.name}: no joint state yet, nothing sent")
            return
        if not arm.send_joint_trajectory([here, here], duration=STUB_TRAJECTORY_DURATION):
            return  # the arm logged why it refused
        yield from wait_until(ctx, lambda: not arm.state.is_executing,
                              timeout_s=STUB_TRAJECTORY_TIMEOUT,
                              description=f"{serial} {arm.config.name} to finish its trajectory")
        ctx.log_info(f"{serial} {arm.config.name}: trajectory finished")


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
    if not state.active:
        return chip("no ctrl", NONE, "no controller running, or no answer yet")
    return chip(CONTROLLER_LABELS.get(state.active, state.active), OK)


def _base_status(base: BaseInterface, now: float) -> str:
    """Status HTML for a base: chips, then its pose (placeholders before the first fix)."""
    state = base.state
    chips = _controller_chip(base.controllers) + freshness_chip("mocap", state.last_update_time, now)
    # Only worth a chip when mocap is talking but says the pose is invalid; with
    # no data at all the gray mocap chip already says it.
    if state.last_update_time and not state.tracked:
        chips += chip("untracked", FAIL)
    # Before the first valid fix the stored pose is only the default, so show placeholders.
    seen = state.has_fix
    yaw = Rotation.from_quat(state.orientation).as_euler("xyz", degrees=True)[2:] if seen else None
    stale = not state.tracked or now - state.last_fix_time >= STALE_AFTER
    return block(chips + values(f"xyz {numbers(state.position if seen else None, 3, 7, 3)} m",
                                f"yaw {numbers(yaw, 1, 7, 1)} °", dim=stale))


def _arm_status(arm: ArmInterface, now: float) -> str:
    """Status HTML for an arm: chips, then joints, TCP and force (placeholders when missing)."""
    state = arm.state
    chips = _controller_chip(arm.controllers) + freshness_chip("joints", state.last_update_time, now)
    if state.is_executing:
        chips += chip("moving", BUSY)
    joints = arm.joint_vector()
    stale = now - state.last_update_time >= STALE_AFTER
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
        seen = state.last_update_time > 0.0
        chips = (chip(f"grip {state.gripper_motor}", OK) + chip(f"screw {state.joint_motor}", OK)
                 if seen else chip("tool", NONE, "no status yet"))
        # Current is in the driver's own units; the message does not say which.
        current = f"{state.current:6d}" if seen else "—".rjust(6)
        pwm = f"{state.pwm_pct:4d}" if seen else "—".rjust(4)
        return block(chips + values(f"I   {current}   pwm {pwm} %", dim=not seen))
    if isinstance(state, ScaffoldingV1State):
        return block(chip(f"grip {'closed' if state.gripper_closed else 'open'}", NONE)
                     + chip(f"screw {'on' if state.screw_on else 'off'}", NONE))
    return ""
