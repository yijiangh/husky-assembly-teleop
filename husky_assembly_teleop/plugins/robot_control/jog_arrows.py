"""The 3D arrows an arm's jog can show: which way each joint turns, and which way tool0 moves and turns.

Per joint a red arrow the way its + button turns it and a green one the way − does; at tool0, per axis in the axis's
colour, a straight arrow the way + moves it and a curved one the way + turns it.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import viser
from scipy.spatial.transform import Rotation

from ...colors import BLUE, GREEN, RED, rgb
from ...plugin_api.context import PluginContext
from ...robot_interface.arm import SCALED_JOINT_TRAJECTORY_CONTROLLER, ArmInterface
from ...ui.joint_sliders import JOINT_SLIDER_NAMES
from ...ui.quaternion import quaternion_to_wxyz

#: The link each UR joint turns, in UR_JOINT_NAMES order; the joint turns about the link's own z axis.
JOINT_LINKS = ("shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link")
#: Arc radius per joint, metres: just outside the UR5e's housings.
JOINT_RADII = (0.15, 0.15, 0.12, 0.09, 0.09, 0.09)
#: Each arrow runs 50° about z away from the link's +x, where the joint's name sits, and from its -x, where the UR's
#: upper arm and forearm run.
ARC_ANGLES = np.radians(np.linspace(12.0, 62.0, 6))
LINE_THICKNESS = 0.006
HEAD_LENGTH, HEAD_RADIUS = 0.03, 0.013
#: The colour of the arrow for each jog direction.
DIRECTION_COLORS = {1.0: RED, -1.0: GREEN}
#: Label size, a fraction of viser's.
LABEL_SCALE = 0.7
#: tool0's arrows, in the colours of its axes (x red, y green, z blue): move arrow length, and the turn arrow's radius
#: about the axis, centred this far along it, metres: out of a Robotiq or a V3 tool (~17 cm).
AXIS_COLORS = (RED, GREEN, BLUE)
TOOL_LENGTH, TOOL_RADIUS, TOOL_TURN_AT = 0.2, 0.04, 0.17
TOOL_ARC_ANGLES = np.radians(np.linspace(0.0, 180.0, 13))
TOOL_THICKNESS, TOOL_HEAD_LENGTH, TOOL_HEAD_RADIUS = LINE_THICKNESS, HEAD_LENGTH, HEAD_RADIUS
#: Turns that take z onto tool0's x, y and z axes.
TO_AXIS = (Rotation.from_euler("y", 90, degrees=True), Rotation.from_euler("x", -90, degrees=True),
           Rotation.identity())


@dataclass
class JogArrows:
    """One arm's jog arrows (`add_jog_arrows`); `draw_jog_arrows` places them.

    Attributes:
        joint_group: Holds the joints' arrows; hidden to hide them.
        joints: A frame per joint, at its child link, holding its name and its two arrows.
        tool0: A frame at tool0, holding its arrows; hidden to hide them.
    """

    joint_group: viser.FrameHandle
    joints: tuple[viser.FrameHandle, ...]
    tool0: viser.FrameHandle


def _add_arc(scene: viser.SceneApi, path: str, angles: np.ndarray, radius: float, color: str,
             size: tuple[float, float, float]) -> tuple[np.ndarray, np.ndarray]:
    """Add a curved arrow about z through `angles` (radians, its head at the last), in the xy plane.

    Args:
        size: Line thickness, head length and head radius, metres.

    Returns:
        tuple[np.ndarray, np.ndarray]: Where its head starts, and the way it points.
    """
    thickness, head_length, head_radius = size
    points = radius * np.stack([np.cos(angles), np.sin(angles), np.zeros_like(angles)], axis=1)
    scene.add_line_segments(f"{path}/arc", np.stack([points[:-1], points[1:]], axis=1), rgb(color),
                            thickness=thickness)
    turn = np.sign(angles[-1] - angles[0])
    tangent = turn * np.array([-np.sin(angles[-1]), np.cos(angles[-1]), 0.0])
    scene.add_arrows(f"{path}/head", np.array([[points[-1], points[-1] + head_length * tangent]]), rgb(color),
                     shaft_radius=thickness / 2, head_radius=head_radius, head_length=head_length)
    return points[-1], tangent


def _add_turn(scene: viser.SceneApi, path: str, radius: float, sign: float) -> None:
    """Add a joint's arrow about z the way a turn of `sign` goes, in its direction's colour, "+" or "−" at its head."""
    end, tangent = _add_arc(scene, path, sign * ARC_ANGLES, radius, DIRECTION_COLORS[sign],
                            (LINE_THICKNESS, HEAD_LENGTH, HEAD_RADIUS))
    scene.add_label(f"{path}/sign", "+" if sign > 0 else "−", position=end + 2 * HEAD_LENGTH * tangent,
                    font_screen_scale=LABEL_SCALE, anchor="center-center")


def _add_tool_arrows(scene: viser.SceneApi, path: str) -> None:
    """Add tool0's arrows: per axis, in its colour, a straight one the way + moves and a curved one the way + turns."""
    for axis, (color, to_axis) in enumerate(zip(AXIS_COLORS, TO_AXIS)):
        name = "xyz"[axis]
        end = np.zeros(3)
        end[axis] = TOOL_LENGTH
        scene.add_arrows(f"{path}/{name}", np.array([[np.zeros(3), end]]), rgb(color), shaft_radius=TOOL_THICKNESS / 2,
                         head_radius=TOOL_HEAD_RADIUS, head_length=TOOL_HEAD_LENGTH)
        scene.add_frame(f"{path}/r{name}", show_axes=False, wxyz=quaternion_to_wxyz(to_axis.as_quat()),
                        position=end * TOOL_TURN_AT / TOOL_LENGTH)
        _add_arc(scene, f"{path}/r{name}", TOOL_ARC_ANGLES, TOOL_RADIUS, color,
                 (TOOL_THICKNESS, TOOL_HEAD_LENGTH, TOOL_HEAD_RADIUS))


def add_jog_arrows(ctx: PluginContext, serial: str, arm: ArmInterface) -> JogArrows:
    """Create one arm's jog arrows in the 3D view, hidden until `draw_jog_arrows` shows them."""
    scene = ctx.view.scene
    root = f"{ctx.view.scene_root}/jog_arrows/{serial}/{arm.config.name}"
    scene.add_frame(root, show_axes=False)
    joint_group = scene.add_frame(f"{root}/joints", show_axes=False, visible=False)
    joints = []
    for joint, (name, radius) in enumerate(zip(JOINT_SLIDER_NAMES, JOINT_RADII)):
        path = f"{root}/joints/{joint}"
        joints.append(scene.add_frame(path, show_axes=False))
        scene.add_label(f"{path}/name", name, position=(radius, 0.0, 0.0), font_screen_scale=LABEL_SCALE,
                        anchor="center-center")
        for sign, node in ((1.0, "plus"), (-1.0, "minus")):
            _add_turn(scene, f"{path}/{node}", radius, sign)
    tool0 = scene.add_frame(f"{root}/tool0", show_axes=False, visible=False)
    _add_tool_arrows(scene, f"{root}/tool0")
    return JogArrows(joint_group, tuple(joints), tool0)


def draw_jog_arrows(ctx: PluginContext, serial: str, arm: ArmInterface, arrows: JogArrows, joints: bool,
                    tool0: bool, robot_frame: bool = False) -> None:
    """Place one arm's joint and tool0 arrows while their box is ticked and the joint controller runs.

    With `robot_frame` (the tool0 jog's Frame at Robot) tool0's arrows keep the robot's axes, at tool0.
    """
    # link_pose raises KeyError for a robot kinematics doesn't know: never shown for it.
    known = any(robot.serial == serial for robot in ctx.config.robots)
    jogging = known and arm.controllers.is_active(SCALED_JOINT_TRAJECTORY_CONTROLLER)
    placed = ((arrows.joint_group, joints, zip(arrows.joints, JOINT_LINKS)),
              (arrows.tool0, tool0, ((arrows.tool0, "tool0"),)))
    for group, ticked, frames in placed:
        visible = ticked and jogging
        if group.visible != visible:
            group.visible = visible
        for frame, link in frames if visible else ():
            pose = ctx.kinematics.link_pose(serial, f"{arm.config.name}_{link}")
            turn = ctx.kinematics.base_pose(serial).orientation if robot_frame and link == "tool0" else pose.orientation
            frame.position, frame.wxyz = pose.position, quaternion_to_wxyz(np.array(turn))
