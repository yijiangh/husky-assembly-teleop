"""
The inputs of an arm's tab: joint jog buttons, a Cartesian target with a force, and its tool's buttons.

Builders add widgets and wire their callbacks; what a click does to the plugin's state comes in as callables.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np
import viser
from scipy.spatial.transform import Rotation

from ...plugin_api.context import PluginContext
from ...robot_interface.arm import (CARTESIAN_MOVE_MAX_ROTATION_SPEED, CARTESIAN_MOVE_MAX_SPEED,
                                    CARTESIAN_MOVE_MIN_DURATION, JOG_DISTANCE, JOG_MAX_SPEED, MAX_TARGET_FORCE,
                                    TARGET_WRENCH_FRAME, ArmInterface, cartesian_move)
from ...robot_interface.end_effectors import EndEffector, RobotiqGripper, ScaffoldingV1, ScaffoldingV3
from ...ui.joint_sliders import JOINT_SLIDER_NAMES
from ...ui.phone import columns_css
from ...ui.style import FAIL, SECTION_CTRL, SECTION_TOOL, block, note, section, values, warning
from ...shared.teleop.dpad import HOLD_CALLBACK_HZ
from .markers import marker_legend

#: TCP position slider range (metres, around the arm's base) and step.
POSITION_RANGE, POSITION_STEP = 1.5, 0.001
POSE_SLIDERS = (("x m", "p"), ("y m", "p"), ("z m", "p"), ("roll °", "a"), ("pitch °", "a"), ("yaw °", "a"))
#: Target force slider step, newtons.
FORCE_STEP = 0.5
#: Short names for end effector kinds, shown next to the TOOL section label.
TOOL_LABELS = {"robotiq": "robotiq", "scaffolding_v1": "v1", "scaffolding_v3": "v3"}
#: Icon of the free drive button; the phone CSS finds the button by it.
FREE_DRIVE_ICON = viser.Icon.HAND_GRAB
#: Icons and directions of each joint's two jog buttons; the CSS finds their folder by the first icon.
JOG_BUTTONS = ((viser.Icon.MINUS, -1.0), (viser.Icon.PLUS, 1.0))
#: Lays out the Jog folder as a row per joint, - then +, for an html widget outside that folder.
JOG_STYLE = f"<style>{columns_css(JOG_BUTTONS[0][0], 2)}</style>"
#: Hint on the Grip buttons, per end effector kind.
GRIP_HINTS = {"robotiq": "Stop holds the fingers where they are.",
              "scaffolding_v1": "The gripper is only ever open or closed.",
              "scaffolding_v3": "A screw clamps the bar and stops by itself when tight. Stop halts both "
                                "motors and clears both stalls; a stalled motor will not start until then."}


@dataclass
class ComplianceInputs:
    """Handles of the compliance controller's inputs."""

    pose: list[viser.GuiSliderHandle]
    pose_plan: viser.GuiHtmlHandle
    force: list[viser.GuiSliderHandle]
    wrench_line: viser.GuiHtmlHandle


# --- --- --- --- --- BUILDERS --- --- --- --- ---

def build_jog_inputs(ctx: PluginContext, gui: viser.GuiApi, label: str, on_hold: Callable[[int, float], None],
                     hold: Callable[[], None]) -> viser.GuiSliderHandle:
    """Add the inputs for the joint trajectory controller: a speed slider, a - and + hold button per joint, and Hold.

    Args:
        ctx: The plugin's context.
        gui: The GUI api, inside the controller's folder.
        label: "<serial> <arm name>", for intent names.
        on_hold: Called every report of a held button, with the joint's index and the direction (-1 or +1).
        hold: Holds the arm where it is.

    Returns:
        viser.GuiSliderHandle: The speed slider, a fraction of JOG_MAX_SPEED.
    """
    gui.add_html(warning("no collision checking · moves while held"))
    gui.add_html(note("to move to a pose: plan it in Plan › Arm"))
    speed = gui.add_slider("Speed", min=0.05, max=1.0, step=0.05, initial_value=0.5,
                           hint=f"Fraction of the full speed, {np.degrees(JOG_MAX_SPEED):.0f} °/s.")
    gui.add_html(JOG_STYLE)
    with gui.add_folder("Jog"):
        for joint, name in enumerate(JOINT_SLIDER_NAMES):
            for icon, direction in JOG_BUTTONS:
                button = gui.add_button(name, icon=icon, hint=f"Turns {name} while held, at most "
                                                              f"{np.degrees(JOG_DISTANCE):.0f}° per press.")
                # ! Bind loop values as defaults, or every button jogs like the last.
                button.on_hold(ctx.defer(f"jog {name} {label}", lambda joint=joint, direction=direction:
                                         on_hold(joint, direction)), callback_hz=HOLD_CALLBACK_HZ)
    move_group = gui.add_button_group("Move", ["Hold"],
                                      hint="Brake to a standstill where the arm is, also during a planned path.")
    move_group.on_click(ctx.defer(f"hold {label}", hold))
    return speed


def build_compliance_inputs(ctx: PluginContext, gui: viser.GuiApi, label: str, arm: ArmInterface,
                            load: Callable[[], None], start: Callable[[np.ndarray, np.ndarray], None],
                            hold: Callable[[], None], apply_force: Callable[[np.ndarray], None]) -> ComplianceInputs:
    """Add the inputs for the compliance controller: a TCP target, and a target force.

    Args:
        ctx: The plugin's context.
        gui: The GUI api, inside the controller's folder.
        label: "<serial> <arm name>", for intent names.
        arm: The arm.
        load: Asks to set the pose sliders to the measured TCP.
        start: Starts a Cartesian move to this TCP position (metres) and quaternion, arm base frame.
        hold: Holds the arm where it is.
        apply_force: Sends this target force, N, tool frame.

    Returns:
        ComplianceInputs: The handles.
    """
    if arm.urdf_problem is not None:
        gui.add_html(warning("URDF frames wrong · targets blocked (see log)", color=FAIL))
    if arm.config.cartesian_test_mode:
        gui.add_html(warning("test mode · checks run, nothing is sent", color=FAIL))
    gui.add_html(warning("no collision checking · straight TCP line, joints unchecked"))
    gui.add_html(note("target: TCP in the husky's base_link; set the sliders or drag the gizmo"))
    pose = [gui.add_slider(name, min=-POSITION_RANGE, max=POSITION_RANGE, step=POSITION_STEP, initial_value=0.0)
            if kind == "p" else gui.add_slider(name, min=-180.0, max=180.0, step=0.1, initial_value=0.0)
            for name, kind in POSE_SLIDERS]
    pose_plan = gui.add_html("")
    load_group = gui.add_button_group("Load", ["Current"],
                                      hint="Set the sliders to where the TCP is (in the husky's base_link).")
    move_group = gui.add_button_group("Move", ["Start", "Hold"],
                                      hint="Start: move the TCP to the sliders in a straight line, "
                                           "5 s or more. Hold: stop where the TCP is.")

    gui.add_html(section("force", SECTION_CTRL, f"in {TARGET_WRENCH_FRAME} frame"))
    limit = MAX_TARGET_FORCE / np.sqrt(3.0)  # all three at their limit stay within MAX_TARGET_FORCE
    force = [gui.add_slider(f"F{axis} N", min=-limit, max=limit, step=FORCE_STEP, initial_value=0.0,
                            hint=f"Along the tool's own {axis} axis ({TARGET_WRENCH_FRAME}).")
             for axis in "xyz"]
    wrench_line = gui.add_html("")
    wrench_group = gui.add_button_group("Force", ["Apply", "Zero"],
                                        hint=f"Apply: push with this force, in the tool frame "
                                             f"({TARGET_WRENCH_FRAME}), until changed. Zero: remove it.")
    gui.add_html(marker_legend())

    def on_move(clicked: str) -> None:
        """Start or Hold (intent)."""
        if clicked == "Hold":
            hold()
        else:
            start(*arm.from_husky(*pose_from_sliders(pose)))

    def on_force(clicked: str) -> None:
        """Apply or remove the target force (intent)."""
        if clicked == "Zero":
            # Zero the sliders too, so a later Apply does not bring the old force back.
            for slider in force:
                slider.value = 0.0
            apply_force(np.zeros(3))
        else:
            apply_force(np.array([slider.value for slider in force]))

    load_group.on_click(ctx.defer(f"load pose {label}", load))
    move_group.on_click(ctx.defer_value(f"cartesian move {label}", on_move))
    wrench_group.on_click(ctx.defer_value(f"force {label}", on_force))
    return ComplianceInputs(pose=pose, pose_plan=pose_plan, force=force, wrench_line=wrench_line)


def build_free_drive_inputs(ctx: PluginContext, gui: viser.GuiApi, label: str,
                            on_hold: Callable[[], None]) -> viser.GuiCheckboxHandle:
    """Add the free drive controller's inputs: the arm is free while the button is held, or while Keep on is ticked.

    Args:
        ctx: The plugin's context.
        gui: The GUI api, inside the controller's folder.
        label: "<serial> <arm name>", for intent names.
        on_hold: Called on the main thread at every report of the held button.

    Returns:
        viser.GuiCheckboxHandle: Keep on; robot_control unticks it once another controller is chosen.
    """
    gui.add_html(note("Running this controller keeps the arm stiff until you hold the button or tick Keep on; then "
                      "it can be pushed by hand. Check the payload on the pendant first: a wrong one makes the arm "
                      "sag or rise."))
    free = gui.add_button("Free drive (hold)", icon=FREE_DRIVE_ICON,
                          hint="The arm can be pushed by hand while you hold this.")
    free.on_hold(ctx.defer(f"free drive {label}", on_hold), callback_hz=HOLD_CALLBACK_HZ)
    return gui.add_checkbox("Keep on", initial_value=False,
                            hint="Free drive without holding the button, until unticked or another controller runs. "
                                 "Only with someone at the arm.")


def tool_commands(tool: EndEffector, leased: Callable[[Callable], object]) -> dict[str, Callable[[], None]]:
    """The gripper buttons, the same on the laptop and the phone: Open, Close, and Stop where the tool can stop.

    Open and Close run under the robot's lease (`leased`); Stop needs none.
    """
    commands = {"Open": lambda: leased(tool.open), "Close": lambda: leased(tool.close)}
    if not isinstance(tool, ScaffoldingV1):  # its gripper cannot stop halfway
        commands["Stop"] = tool.stop
    return commands


def build_tool_inputs(ctx: PluginContext, gui: viser.GuiApi, label: str, arm: ArmInterface,
                      leased: Callable[[Callable], object]) -> viser.GuiHtmlHandle:
    """Add the TOOL section for the arm's end effector: its status line and buttons.

    Args:
        ctx: The plugin's context.
        gui: The GUI api, inside the arm's tab.
        label: "<serial> <arm name>", for intent names.
        arm: The arm; must have an end effector.
        leased: Runs a command under the robot's lease, which every tool command but a stop needs.

    Returns:
        viser.GuiHtmlHandle: The tool's status line.
    """
    kind = arm.config.end_effector
    gui.add_html(section("tool", SECTION_TOOL, TOOL_LABELS.get(kind, kind)))
    status = gui.add_html("")
    tool = arm.end_effector
    commands = tool_commands(tool, leased)
    grip = gui.add_button_group("Grip", list(commands), hint=GRIP_HINTS.get(kind))
    grip.on_click(ctx.defer_value(f"grip {label}", lambda clicked: commands[clicked]()))
    if isinstance(tool, RobotiqGripper):
        _build_robotiq_inputs(ctx, gui, label, tool, leased)
    elif isinstance(tool, ScaffoldingV3):
        directions = {"Tighten": 1, "Loosen": -1, "Stop": 0}
        screw = gui.add_button_group("Screw", list(directions),
                                     hint="Fixes the bar to the joint; stops by itself when tight. Only one "
                                          "motor runs at a time, so starting this stops the gripper. Stop "
                                          "halts both.")
        screw.on_click(ctx.defer_value(f"screw {label}", lambda clicked: tool.stop() if clicked == "Stop" else
                                       leased(lambda: tool.drive_screw(directions[clicked]))))
    return status


def build_phone_arm_inputs(ctx: PluginContext, gui: viser.GuiApi, label: str, arm: ArmInterface,
                           on_free_drive: Callable[[], None], leased: Callable[[Callable], object]
                           ) -> viser.GuiHtmlHandle:
    """Add an arm's inputs on the phone: its status line, the gripper buttons, and free drive while held.

    Args:
        ctx: The plugin's context.
        gui: The phone's GUI api, inside the robot's tab.
        label: "<serial> <arm name>", for intent names.
        arm: The arm.
        on_free_drive: Called on the main thread at every report of the held free drive button; it switches to
            the free drive controller and keeps free drive on.
        leased: Runs a command under the robot's lease, which the gripper's Open and Close need.

    Returns:
        viser.GuiHtmlHandle: The arm's status line.
    """
    status = gui.add_html("")
    tool = arm.end_effector
    if tool is not None:
        commands = tool_commands(tool, leased)
        grip = gui.add_button_group("Gripper", list(commands))
        grip.on_click(ctx.defer_value(f"grip {label}", lambda clicked: commands[clicked]()))
    free = gui.add_button("Free drive (hold)", icon=FREE_DRIVE_ICON,
                          hint="The arm can be pushed by hand while you hold this; on release the joint controller "
                               "holds it where it is. Check the payload on the pendant first.")
    free.on_hold(ctx.defer(f"free drive {label}", on_free_drive), callback_hz=HOLD_CALLBACK_HZ)
    return status


def _build_robotiq_inputs(ctx: PluginContext, gui: viser.GuiApi, label: str, gripper: RobotiqGripper,
                          leased: Callable[[Callable], object]) -> None:
    """Add the Robotiq-only inputs: a target opening with a force limit (Go only), and Reactivate.

    Args:
        ctx: The plugin's context.
        gui: The GUI api, inside the arm's tab.
        label: "<serial> <arm name>", for intent names.
        gripper: The gripper to command.
        leased: Runs a command under the robot's lease.
    """
    position = gui.add_slider("pos rad", min=gripper.OPEN_POSITION, max=gripper.CLOSED_POSITION, step=0.01,
                              initial_value=gripper.OPEN_POSITION,
                              hint=f"Knuckle angle: {gripper.OPEN_POSITION} open, {gripper.CLOSED_POSITION} closed.")
    force = gui.add_slider("force", min=0.0, max=gripper.MAX_EFFORT, step=0.05, initial_value=gripper.DEFAULT_EFFORT,
                           hint="Force limit, as a fraction of the gripper's full force.")
    target = gui.add_button_group("Target", ["Go"], hint="Move the fingers to the slider position.")
    target.on_click(ctx.defer(f"gripper target {label}",
                              lambda: leased(lambda: gripper.move(position.value, force.value))))
    driver = gui.add_button_group("Driver", ["Reactivate"],
                                  hint="After a fault or power loss. The gripper opens and closes once: "
                                       "hold nothing in it.")
    driver.on_click(ctx.defer(f"reactivate gripper {label}", lambda: leased(gripper.reactivate)))


# --- --- --- --- --- READING THE SLIDERS --- --- --- --- ---

def pose_from_sliders(sliders: list[viser.GuiSliderHandle]) -> tuple[np.ndarray, np.ndarray]:
    """The TCP target the pose sliders describe, in the husky's base_link: position and quaternion.

    Args:
        sliders: x, y, z (metres) then roll, pitch, yaw (degrees, fixed axes x-y-z).

    Returns:
        tuple[np.ndarray, np.ndarray]: Position (3,) and quaternion (x, y, z, w).
    """
    values_ = [slider.value for slider in sliders]
    return np.array(values_[:3]), Rotation.from_euler("xyz", values_[3:], degrees=True).as_quat()


def pose_plan_line(arm: ArmInterface, pose: list[viser.GuiSliderHandle]) -> str:
    """What Start would do from here: distance, turn and duration of the Cartesian move; gray "—" before a TCP."""
    state = arm.state
    limits = (f"(≤{CARTESIAN_MOVE_MAX_SPEED * 100:.0f} cm/s, "
              f"≤{np.degrees(CARTESIAN_MOVE_MAX_ROTATION_SPEED):.0f} °/s, "
              f"≥{CARTESIAN_MOVE_MIN_DURATION:.0f} s)")
    if state.tcp_position is None:
        return block(values(f"Δ {'—':>6} m {'—':>6} °   T {'—':>5} s", limits, dim=True))
    position, orientation = arm.from_husky(*pose_from_sliders(pose))
    duration, _ = cartesian_move(state.tcp_position, state.tcp_orientation, position, orientation)
    distance = float(np.linalg.norm(position - state.tcp_position))
    turn = float(np.degrees((Rotation.from_quat(state.tcp_orientation).inv()
                             * Rotation.from_quat(orientation)).magnitude()))
    return block(values(f"Δ {distance:6.3f} m {turn:6.1f} °   T {duration:5.1f} s", limits))
