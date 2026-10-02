"""
The CTRL section that switches controllers, and the status readouts of bases, arms and tools.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import numpy as np
import viser
from scipy.spatial.transform import Rotation

from ...plugin_api.context import PluginContext
from ...robot_interface.arm import (CARTESIAN_COMPLIANCE_CONTROLLER, FREE_DRIVE_CONTROLLER,
                                    SCALED_JOINT_TRAJECTORY_CONTROLLER, ArmInterface)
from ...robot_interface.base import PLATFORM_VELOCITY_CONTROLLER, BaseInterface
from ...robot_interface.controller_manager import ControllerManagerInterface
from ...robot_interface.stream_stats import WINDOW, StreamQuality
from ...robot_interface.end_effectors import (RobotiqGripper, RobotiqState, ScaffoldingV1State, ScaffoldingV3,
                                              ScaffoldingV3State)
from ...ui.style import (BUSY, FAIL, INFO, NONE, OK, SECTION_CTRL, block, check_chip, chip, freshness_chip, note, numbers,
                         section, values)
from ...world.checks import STALE_AFTER
from ...world.mocap import MARKER_ERROR_WARN, MocapBody, mocap_check

#: Short button labels for controllers; unlisted ones show their full name.
CONTROLLER_LABELS = {
    PLATFORM_VELOCITY_CONTROLLER: "Vel",
    SCALED_JOINT_TRAJECTORY_CONTROLLER: "Joint",
    CARTESIAN_COMPLIANCE_CONTROLLER: "Cart",
    FREE_DRIVE_CONTROLLER: "Free",
}


# --- --- --- --- --- CTRL SECTION --- --- --- --- ---

@dataclass
class ControllerPanel:
    """Controller buttons for one controller manager, and one folder of inputs per controller.

    Attributes:
        controllers: The controller manager this panel shows and switches.
        inputs: Per controller, its inputs folder; only the running one is visible.
    """

    controllers: ControllerManagerInterface
    inputs: dict[str, viser.GuiFolderHandle] = field(default_factory=dict)


def build_controller_panel(ctx: PluginContext, gui: viser.GuiApi, controllers: ControllerManagerInterface,
                           input_builders: dict[str, Callable[[], object]],
                           ) -> tuple[ControllerPanel, dict[str, object]]:
    """Build the CTRL section: one button per switchable controller, and a folder of inputs for each.

    Args:
        ctx: The plugin's context.
        gui: The GUI api, inside the tab the panel goes in.
        controllers: The controller manager to show and switch.
        input_builders: Per controller, a function that adds its inputs; missing ones get a placeholder.

    Returns:
        tuple[ControllerPanel, dict[str, object]]: The handles, and what each input builder returned.
    """
    gui.add_html(section("ctrl", SECTION_CTRL))
    by_label = {CONTROLLER_LABELS.get(name, name): name for name in controllers.switchable}
    switch = gui.add_button_group("Run", list(by_label))
    switch.on_click(ctx.defer_value(f"switch controller {controllers.namespace}",
                                    lambda clicked: controllers.switch(by_label[clicked])))

    panel = ControllerPanel(controllers=controllers)
    built: dict[str, object] = {}
    for controller in controllers.switchable:
        folder = gui.add_folder(CONTROLLER_LABELS.get(controller, controller), visible=False)
        with folder:
            build = input_builders.get(controller)
            if build is None:
                gui.add_html(note("no inputs yet"))
            else:
                built[controller] = build()
        panel.inputs[controller] = folder
    return panel, built


def show_only_running(panel: ControllerPanel) -> None:
    """Show the running controller's inputs and hide the others."""
    for controller, folder in panel.inputs.items():
        folder.visible = controller == panel.controllers.state.active


# --- --- --- --- --- READOUTS --- --- --- --- ---

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


def base_status(base: BaseInterface, now: float) -> str:
    """Status HTML for a base: chips, then its pose (placeholders before the first fix)."""
    state = base.state
    chips = _controller_chip(base.controllers) + check_chip(mocap_check("mocap", base.mocap_id, state, now))
    yaw = None if state.orientation is None else Rotation.from_quat(state.orientation).as_euler(
        "xyz", degrees=True)[2:]
    stale = not state.tracked or now - state.last_fix_time >= STALE_AFTER
    return block(chips + values(f"xyz {numbers(state.position, 3, 7, 3)} m",
                                f"yaw {numbers(yaw, 1, 7, 1)} °", dim=stale) + _mocap_line(state, now))


def _mocap_line(body: MocapBody, now: float) -> str:
    """The fix behind the mocap chip: marker error, age of the last valid fix, and why it is invalid.

    * Live numbers belong here, not in the health tooltip, which closes on every change.
    """
    error = "—".rjust(5) if body.marker_error is None else f"{body.marker_error * 1e3:5.2f}"
    age = "—".rjust(5) if body.last_fix_time is None else f"{(now - body.last_fix_time) * 1e3:5.0f}"
    flags = ""
    if body.tracking_valid is False:
        flags = "  lost"
    elif body.last_update_time is not None and not body.tracked:
        flags = "  invalid"
    return values(f"mocap err {error} mm (warn {MARKER_ERROR_WARN * 1e3:g})  fix {age} ms ago{flags}",
                  dim=body.last_update_time is None)


def arm_status(arm: ArmInterface, now: float) -> str:
    """Status HTML for an arm: chips, then joints, TCP, force and wifi (placeholders when missing)."""
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
        dim=stale) + _wifi_line(state.joint_states_stats.quality(now)))


def _wifi_line(quality: StreamQuality) -> str:
    """joint_states rate and largest gap over the last WINDOW seconds; gray while still measuring.

    * Live numbers belong here, not in the health tooltip, which closes on every change.
    """
    rate = "—".rjust(4) if quality.rate is None else f"{quality.rate:4.0f}"
    gap = "—".rjust(4) if quality.max_gap is None else f"{quality.max_gap * 1e3:4.0f}"
    return values(f"wifi {rate} Hz  max gap {gap} ms  ({WINDOW:g} s)", dim=quality.rate is None)


def tool_status(state: object, now: float) -> str:
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
        # The target until the first joint state arrives.
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
            chips += "".join(chip(name, INFO, "held on the tool")
                             for name, held in zip(ScaffoldingV3.BUTTONS, state.buttons_held) if held)
        stale = state.last_update_time is None or now - state.last_update_time >= STALE_AFTER
        current = "—".rjust(6) if state.current is None else f"{state.current:6d}"
        pwm = "—".rjust(4) if state.pwm_pct is None else f"{state.pwm_pct:4d}"
        return block(chips + values(f"I   {current} mA   pwm {pwm} %", dim=stale))
    if isinstance(state, ScaffoldingV1State):
        chips = freshness_chip("io", state.last_update_time, now)
        if state.last_request_ok is False:
            chips += chip("set_io failed", FAIL, "the arm refused the last output change")
        # Outputs as the UR reports them; the tool itself reports nothing.
        if state.gripper_closed is not None:
            chips += chip("grip closed" if state.gripper_closed else "grip open", OK)
        return block(chips)
    return ""


def _motor_chip(name: str, motor_state: str | None) -> str:
    """One v3 motor's chip: busy while it runs, blue when stalled (screwed tight, as intended), green when idle."""
    color = {ScaffoldingV3.IDLE: OK, ScaffoldingV3.STALLED: INFO}.get(motor_state, BUSY)
    hint = "screwed tight; Stop clears the stall before it runs again" if motor_state == ScaffoldingV3.STALLED else ""
    return chip(f"{name} {str(motor_state).lower()}", color, hint)
