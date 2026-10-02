"""The 3D markers drawn while an arm's Cartesian controller runs: target and TCP frames, and force arrows."""

from __future__ import annotations

import numpy as np
import viser
from scipy.spatial.transform import Rotation

from ...design_io.pose import Pose, compose
from ...plugin_api.context import PluginContext
from ...robot_interface.arm import CARTESIAN_COMPLIANCE_CONTROLLER, ArmInterface
from ...robot_interface.ur_frames import STOCK_YAW
from ...ui.quaternion import quaternion_to_wxyz
from ...ui.style import block

#: Poses drawn in 3D while the Cartesian controller runs: kind, label, origin colour (r, g, b), axis length (m).
FRAME_MARKERS = (
    ("target", "target (sliders)", (255, 200, 0), 0.12),
    ("tcp", "TCP reported", (40, 170, 60), 0.08),
)
#: Force arrows drawn from the reported TCP: kind, label, colour.
FORCE_ARROWS = (
    ("force_target", "force (sliders)", (255, 200, 0)),
    ("force_applied", "force applied", (255, 110, 0)),
)
FORCE_ARROW_SCALE = 0.01  # metres per newton: 20 N draws 20 cm


def _marker_path(ctx: PluginContext, kind: str, serial: str, arm: ArmInterface) -> str:
    """Scene path "<plugin root>/cartesian/<kind>/<serial>/<arm>"; kind first, so one toggle hides it on every arm.

    Args:
        ctx: The plugin's context.
        kind: A FRAME_MARKERS or FORCE_ARROWS kind, e.g. "target".
        serial: The robot's serial.
        arm: The arm.
    """
    return f"{ctx.view.scene_root}/cartesian/{kind}/{serial}/{arm.config.name}"


def add_frame_markers(ctx: PluginContext, serial: str, arm: ArmInterface) -> dict[str, viser.FrameHandle]:
    """Create one arm's frame markers in the 3D view, hidden until needed.

    Args:
        ctx: The plugin's context.
        serial: The robot's serial.
        arm: The arm.

    Returns:
        dict[str, viser.FrameHandle]: One marker per FRAME_MARKERS kind.
    """
    return {name: ctx.view.scene.add_frame(_marker_path(ctx, name, serial, arm), axes_length=length, axes_radius=0.004,
                                           origin_radius=0.012, origin_color=color, visible=False)
            for name, _, color, length in FRAME_MARKERS}


def add_force_arrows(ctx: PluginContext, serial: str, arm: ArmInterface) -> dict[str, viser.ArrowsHandle]:
    """Create one arm's force arrows in the 3D view, hidden until there is a force to show.

    Args:
        ctx: The plugin's context.
        serial: The robot's serial.
        arm: The arm.

    Returns:
        dict[str, viser.ArrowsHandle]: One arrow per FORCE_ARROWS kind.
    """
    return {name: ctx.view.scene.add_arrows(_marker_path(ctx, name, serial, arm), points=np.zeros((1, 2, 3)),
                                            colors=color, shaft_radius=0.004, head_radius=0.012, head_length=0.03,
                                            visible=False)
            for name, _, color in FORCE_ARROWS}


def marker_legend() -> str:
    """HTML legend of the 3D markers, for the arm's panel."""
    style = "font-size:11px;margin-right:8px;white-space:nowrap"
    return block("".join(f'<span style="{style}"><span style="color:rgb{color}">●</span> {text}</span>'
                         for _, text, color, _ in FRAME_MARKERS)
                 + "".join(f'<span style="{style}"><span style="color:rgb{color}">➜</span> {text}</span>'
                           for _, text, color in FORCE_ARROWS))


def place_markers(ctx: PluginContext, serial: str, arm: ArmInterface, target: tuple[np.ndarray, np.ndarray],
                  force_target: np.ndarray, force_applied: np.ndarray | None) -> dict[str, tuple] | None:
    """Where one arm's markers go this tick, in the world.

    ? Placed in the controller's own `base_link` (`base_link_inertia` turned by STOCK_YAW),
      so they show where the controller will go. See doc/ur_frames.md.

    Args:
        ctx: The plugin's context.
        serial: The robot's serial.
        arm: The arm.
        target: TCP target from the sliders, (position, quaternion), arm base frame.
        force_target: Force on the sliders, N, tool frame.
        force_applied: Force last sent, N, tool frame; None if none was.

    Returns:
        dict[str, tuple] | None: Per "<kind>_world", a pose (position, quaternion) for frames or a segment
            (start, end) for arrows; kinds with nothing to show are absent. None without the compliance controller.
    """
    # link_pose raises KeyError for a robot kinematics doesn't know: skip it.
    known = any(robot.serial == serial for robot in ctx.config.robots)
    if not known or not arm.controllers.is_active(CARTESIAN_COMPLIANCE_CONTROLLER):
        return None
    state = arm.state
    inertia = ctx.kinematics.link_pose(serial, f"{arm.config.name}_base_link_inertia")
    base = compose(inertia, Pose(orientation=tuple(Rotation.from_euler("z", -STOCK_YAW).as_quat())))
    markers: dict[str, tuple] = {}
    local_poses = {"target": target}
    if state.tcp_position is not None:
        local_poses["tcp"] = (state.tcp_position, state.tcp_orientation)
    for name, (position, orientation) in local_poses.items():
        world = compose(base, Pose.from_arrays(position, orientation))
        markers[f"{name}_world"] = (world.position, world.orientation)
    # Forces are in tool0: drawn from the reported TCP, turned with it.
    tcp = markers.get("tcp_world")
    for name, force in {"force_target": force_target, "force_applied": force_applied}.items():
        if tcp is not None and force is not None and np.linalg.norm(force) > 0.0:
            start = np.array(tcp[0])
            markers[f"{name}_world"] = (start, start + Rotation.from_quat(tcp[1]).apply(force) * FORCE_ARROW_SCALE)
    return markers


def draw_markers(frames: dict[str, viser.FrameHandle], arrows: dict[str, viser.ArrowsHandle],
                 markers: dict[str, tuple]) -> None:
    """Move one arm's markers to `markers` (from `place_markers`) and hide those absent from it."""
    for name, frame in frames.items():
        pose = markers.get(f"{name}_world")
        frame.visible = pose is not None
        if pose is not None:
            frame.position, frame.wxyz = pose[0], quaternion_to_wxyz(np.array(pose[1]))
    for name, arrow in arrows.items():
        segment = markers.get(f"{name}_world")
        arrow.visible = segment is not None
        if segment is not None:
            arrow.points = np.array([segment])
