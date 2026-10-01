"""
The inputs of an arm's tab: joint targets, a Cartesian target with a force, and its tool's buttons.

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
                                    CARTESIAN_MOVE_MIN_DURATION, JOINT_MOVE_MAX_SPEED, JOINT_MOVE_MIN_DURATION,
                                    MAX_TARGET_FORCE, TARGET_WRENCH_FRAME, ArmInterface, cartesian_move, joint_move)
from ...robot_interface.end_effectors import RobotiqGripper, ScaffoldingV1, ScaffoldingV3
from ...ui.ghost import RecentUse
from ...ui.style import FAIL, SECTION_CTRL, SECTION_TOOL, block, note, section, values, warning
from .markers import marker_legend

#: Slider label and range (degrees) per UR joint, in UR_JOINT_NAMES order.
JOINT_SLIDERS = (("pan", 360), ("lift", 360), ("elbow", 180), ("w1", 360), ("w2", 360), ("w3", 360))
#: TCP position slider range (metres, around the arm's base) and step.
POSITION_RANGE, POSITION_STEP = 1.5, 0.001
POSE_SLIDERS = (("x m", "p"), ("y m", "p"), ("z m", "p"), ("roll °", "a"), ("pitch °", "a"), ("yaw °", "a"))
#: Target force slider step, newtons.
FORCE_STEP = 0.5
#: Short names for end effector kinds, shown next to the TOOL section label.
TOOL_LABELS = {"robotiq": "robotiq", "scaffolding_v1": "v1", "scaffolding_v3": "v3"}


@dataclass
class JointInputs:
    """Handles of the joint trajectory controller's inputs."""

    joints: list[viser.GuiSliderHandle]
    plan: viser.GuiHtmlHandle


@dataclass
class ComplianceInputs:
    """Handles of the compliance controller's inputs."""

    pose: list[viser.GuiSliderHandle]
    pose_plan: viser.GuiHtmlHandle
    force: list[viser.GuiSliderHandle]
    wrench_line: viser.GuiHtmlHandle


# --- --- --- --- --- BUILDERS --- --- --- --- ---

def build_joint_inputs(ctx: PluginContext, gui: viser.GuiApi, label: str, arm: ArmInterface, used: RecentUse,
                       written: Callable[[], tuple[float, ...] | None],
                       load: Callable[[np.ndarray | None], None],
                       start: Callable[[np.ndarray], None], hold: Callable[[], None]) -> JointInputs:
    """Add the inputs for the joint trajectory controller: a target per joint, Load and Move.

    Args:
        ctx: The plugin's context.
        gui: The GUI api, inside the controller's folder.
        label: "<serial> <arm name>", for intent names.
        arm: The arm.
        used: Touched whenever the operator works these inputs; the target ghost shows while recent.
        written: The slider values the plugin wrote last, so their callbacks are not taken for the operator's.
        load: Asks to set the sliders to these angles (radians), or to the measured joints for None.
        start: Starts a move to these joint angles (radians).
        hold: Holds the arm where it is.

    Returns:
        JointInputs: The handles.
    """
    gui.add_html(warning("no collision checking · raw joint trajectories"))
    gui.add_html(note("yellow ghost: the arm at the sliders"))
    joints = [gui.add_slider(f"{name} °", min=-limit, max=limit, step=0.5, initial_value=0.0)
              for name, limit in JOINT_SLIDERS]
    plan = gui.add_html("")
    load_group = gui.add_button_group("Load", ["Current", "Stow"],
                                      hint="Set the sliders to where the arm is, or to its stow pose.")
    move_group = gui.add_button_group("Move", ["Start", "Hold"],
                                      hint="Start: move to the sliders, smoothly, in 5 s or more. "
                                           "Hold: brake to a standstill where the arm is.")

    def on_slider(_value: object) -> None:
        """Mark the joints as used, unless the values are the ones the plugin just wrote (intent)."""
        if tuple(slider.value for slider in joints) != written():
            used.touch()

    def on_load(clicked: str) -> None:
        """Load the sliders from the arm or its stow pose (intent)."""
        used.touch()
        if clicked == "Current":
            load(None)
        elif arm.config.stow_joints is None:
            ctx.log_warn(f"{label}: no stow pose configured (ArmConfig.stow_joints)")
        else:
            load(np.array(arm.config.stow_joints))

    def on_move(clicked: str) -> None:
        """Start or Hold (intent)."""
        used.touch()
        if clicked == "Hold":
            hold()
        else:
            # ! Read sliders here, when the intent runs: the operator commits on Start.
            start(np.radians([slider.value for slider in joints]))

    for slider in joints:
        slider.on_update(ctx.defer_value(f"joint slider {label}", on_slider))
    load_group.on_click(ctx.defer_value(f"load sliders {label}", on_load))
    move_group.on_click(ctx.defer_value(f"move {label}", on_move))
    return JointInputs(joints=joints, plan=plan)


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
    gui.add_html(note("target: TCP in the husky's base_link"))
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


def build_tool_inputs(ctx: PluginContext, gui: viser.GuiApi, label: str, arm: ArmInterface) -> viser.GuiHtmlHandle:
    """Add the TOOL section for the arm's end effector: its status line and buttons.

    Args:
        ctx: The plugin's context.
        gui: The GUI api, inside the arm's tab.
        label: "<serial> <arm name>", for intent names.
        arm: The arm; must have an end effector.

    Returns:
        viser.GuiHtmlHandle: The tool's status line.
    """
    kind = arm.config.end_effector
    gui.add_html(section("tool", SECTION_TOOL, TOOL_LABELS.get(kind, kind)))
    status = gui.add_html("")
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
        screw.on_click(ctx.defer_value(f"screw {label}", lambda clicked: tool.drive_screw(directions[clicked])))
    return status


def _build_robotiq_inputs(ctx: PluginContext, gui: viser.GuiApi, label: str, gripper: RobotiqGripper) -> None:
    """Add the Robotiq-only inputs: a target opening with a force limit (Go only), and Reactivate.

    Args:
        ctx: The plugin's context.
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
    target.on_click(ctx.defer(f"gripper target {label}", lambda: gripper.move(position.value, force.value)))
    driver = gui.add_button_group("Driver", ["Reactivate"],
                                  hint="After a fault or power loss. The gripper opens and closes once: "
                                       "hold nothing in it.")
    driver.on_click(ctx.defer(f"reactivate gripper {label}", gripper.reactivate))


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


def joint_plan_line(arm: ArmInterface, joints: list[viser.GuiSliderHandle]) -> str:
    """What Start would do from here: the largest joint change and the duration."""
    here = arm.joint_vector()
    if here is None:
        return note("no joint state yet")
    target = np.radians([slider.value for slider in joints])
    _, _, duration = joint_move(here, target)
    largest = float(np.degrees(np.max(np.abs(target - here))))
    return block(values(f"Δmax {largest:6.1f} °   T {duration:5.1f} s   "
                        f"(≤{np.degrees(JOINT_MOVE_MAX_SPEED):.0f} °/s, ≥{JOINT_MOVE_MIN_DURATION:.0f} s)"))


def pose_plan_line(arm: ArmInterface, pose: list[viser.GuiSliderHandle]) -> str:
    """What Start would do from here: distance, turn and duration of the Cartesian move."""
    state = arm.state
    if state.tcp_position is None:
        return note("no TCP pose yet")
    position, orientation = arm.from_husky(*pose_from_sliders(pose))
    duration, _ = cartesian_move(state.tcp_position, state.tcp_orientation, position, orientation)
    distance = float(np.linalg.norm(position - state.tcp_position))
    turn = float(np.degrees((Rotation.from_quat(state.tcp_orientation).inv()
                             * Rotation.from_quat(orientation)).magnitude()))
    return block(values(f"Δ {distance:6.3f} m {turn:6.1f} °   T {duration:5.1f} s",
                        f"(≤{CARTESIAN_MOVE_MAX_SPEED * 100:.0f} cm/s, "
                        f"≤{np.degrees(CARTESIAN_MOVE_MAX_ROTATION_SPEED):.0f} °/s, "
                        f"≥{CARTESIAN_MOVE_MIN_DURATION:.0f} s)"))
