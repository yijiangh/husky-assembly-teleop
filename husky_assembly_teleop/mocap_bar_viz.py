"""Draw a bar measured with MoCap next to its Rhino pose, in PyBullet (the by-eye check).

Two see-through copies ("ghosts") of a bar and the joints mounted on it: one
at the Rhino (designed) pose, one at the pose fitted from the MoCap markers
(``bar_mocap_fit``). Plus debug lines and text, which the viewer always draws
(bodies created at run time may not show when the window's mesh buffer is
full):

- both bar centre lines with a label (``B20 Rhino``, ``B20 MoCap 8/8 rms 0.9 mm``);
  the pink Rhino line and label can be left out (the monitor's 'Rhino bar
  lines (pink)' checkbox)
- every sensed marker as a small cross: white = matched, red = stray
- a black line from each matched marker to the design marker it was matched to
- a red label (the marker's name) at each design marker nobody matched

What is here:

- `build_bar_joint_ghost(...)` -> one visual-only body per mesh of the bar and its joints
- `set_ghost_pose(ghost, pose, client_id)`, `remove_ghost(ghost, client_id)`
- `bar_z_range(robot_cell, bar_name)`, `bar_end_points(T, z_range)` -> the bar's ends
- `draw_bar_fit(items, client_id, ...)` -> the debug lines and text of one fit
- `remove_debug_items(items, client_id)`

! No ROS here, so ``scripts/headless_mocap_bar_smoke.py`` runs it without a window.
"""

from __future__ import annotations

import os
from contextlib import contextmanager
from typing import Iterator, Sequence

import numpy as np
import pybullet as p
from compas_fab.robots import RobotCell, RobotCellState
from pybullet_planning import CLIENTS, invert, multiply

from husky_assembly_teleop.bar_mocap_fit import BarFit, BarMarkers, transform_points
from husky_assembly_teleop.utils import pose_from_frame

# * Where a hidden ghost waits: far below the floor, out of every view.
GHOST_HIDDEN_POSE = ((0.0, 0.0, -100.0), (0.0, 0.0, 0.0, 1.0))
# Half the arm length of a sensed-marker cross, metres (the balls are 19 mm).
MARKER_CROSS_HALF_M = 0.012
WHITE_RGB = (1.0, 1.0, 1.0)
RED_RGB = (1.0, 0.0, 0.0)
BLACK_RGB = (0.0, 0.0, 0.0)
# How high above the bar's middle the labels sit, metres.
RHINO_LABEL_RISE_M = 0.12
MOCAP_LABEL_RISE_M = 0.22


# * ================================================================ rendering

@contextmanager
def rendering_paused(client_id: int) -> Iterator[None]:
    """Pause the GUI's rendering while drawing, so each debug line does not wait for a frame.

    ! Not pybullet_planning's ``LockRenderer``: that one puts back the on/off
    ! state it found when it started. The monitor redraws from its tick thread
    ! while a button (e.g. 'Plan Movement') may hold a ``LockRenderer`` on the
    ! panel's thread; interleaved, the two can leave rendering off for good.
    ! This one puts back pybullet_planning's shared record of the state
    ! (``CLIENTS[client]``, which only ``LockRenderer`` / ``set_renderer``
    ! change), so either order ends right.

    Args:
        client_id (int): The PyBullet client; nothing is paused without a GUI.

    Yields:
        None: Draw inside the ``with`` block.
    """
    gui = p.getConnectionInfo(physicsClientId=client_id).get('connectionMethod') == p.GUI
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=client_id)
    try:
        yield
    finally:
        if gui:
            on = CLIENTS.get(client_id, True) is not False
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, int(on), physicsClientId=client_id)


# * ================================================================ ghosts

def build_bar_joint_ghost(robot_cell: RobotCell, state: RobotCellState, bar_name: str, joint_names: list,
                          rgba: Sequence[float], mesh_dir: str,
                          client_id: int) -> list:
    """A see-through copy of a bar and the joints on it, placed relative to the bar.

    One visual-only body (no collision shape, not in the cfab cell, so no
    planner sees it) per visual mesh of the bar and of each joint, built the
    way compas_fab builds the cell's own bodies (body pose = rigid-body frame).
    Each body keeps its offset from the bar as it is in ``state``, so moving
    the ghost to a bar pose puts the joints where they sit on that bar.

    Args:
        robot_cell (RobotCell): The cfab cell; its ``rigid_body_models`` hold the meshes.
        state (RobotCellState): A state where the bar and its joints have world frames
            (e.g. the bar resting at its assembled pose).
        bar_name (str): The bar's rigid-body name, e.g. ``'bar_B20'``.
        joint_names (list): The joint bodies on the bar (``bar_action_io.joints_on_bar``);
            a joint without a frame in ``state`` is left out.
        rgba: Colour of every body, e.g. ``(1.0, 0.3, 0.7, 0.5)``.
        mesh_dir (str): Folder for the OBJ files, written once per rigid body
            (named after its guid and the mesh index).
        client_id (int): The PyBullet client to build in.

    Returns:
        list[tuple]: ``(body id, bar_from_body pose)`` per body, the bar's own
        bodies first; empty when the bar has no frame in ``state``.
    """
    rb_states = state.rigid_body_states or {}
    bar_state = rb_states.get(bar_name)
    if bar_state is None or bar_state.frame is None:
        return []
    bar_from_world = invert(pose_from_frame(bar_state.frame))
    ghost = []
    for name in [bar_name] + list(joint_names):
        rb_state = rb_states.get(name)
        model = robot_cell.rigid_body_models.get(name)
        if rb_state is None or rb_state.frame is None or model is None:
            continue
        bar_from_body = multiply(bar_from_world, pose_from_frame(rb_state.frame))
        paths = [os.path.join(mesh_dir, f"{model.guid}_visual_{k}.obj") for k in range(len(model.visual_meshes))]
        # Scaling to metres copies every mesh, so only do it when a file is missing.
        if not all(os.path.exists(path) for path in paths):
            for path, mesh in zip(paths, model.visual_meshes_in_meters):
                mesh.to_obj(path)
        for path in paths:
            visual = p.createVisualShape(p.GEOM_MESH, fileName=path, rgbaColor=list(rgba),
                                         physicsClientId=client_id)
            body = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=visual,
                                     physicsClientId=client_id)
            ghost.append((body, bar_from_body))
    return ghost


def set_ghost_pose(ghost: list, world_from_bar_pose: tuple, client_id: int) -> None:
    """Move a ghost so its bar sits at a world pose.

    Args:
        ghost (list): From `build_bar_joint_ghost`.
        world_from_bar_pose (tuple): pybullet ``(pos, quat_xyzw)`` of the bar frame;
            `GHOST_HIDDEN_POSE` hides the ghost.
        client_id (int): The PyBullet client it was built in.
    """
    for body, bar_from_body in ghost:
        pos, quat = multiply(world_from_bar_pose, bar_from_body)
        p.resetBasePositionAndOrientation(body, pos, quat, physicsClientId=client_id)


def remove_ghost(ghost: list, client_id: int) -> None:
    """Delete a ghost's bodies.

    Args:
        ghost (list): From `build_bar_joint_ghost`.
        client_id (int): The PyBullet client it was built in.
    """
    for body, _bar_from_body in ghost:
        try:
            p.removeBody(body, physicsClientId=client_id)
        except p.error:
            pass  # the client was closed already (a new design problem was loaded)


# * ================================================================ bar ends

def bar_z_range(robot_cell: RobotCell, bar_name: str) -> tuple:
    """How far the bar's mesh reaches along its own z (the bar axis).

    Args:
        robot_cell (RobotCell): The cfab cell.
        bar_name (str): The bar's rigid-body name.

    Returns:
        tuple: ``(z_min, z_max)`` in metres, in the bar frame; ``(0.0, 0.0)``
        when the bar has no visual mesh.
    """
    zs = [z for mesh in robot_cell.rigid_body_models[bar_name].visual_meshes_in_meters
          for _x, _y, z in mesh.vertices_attributes('xyz')]
    return (float(min(zs)), float(max(zs))) if zs else (0.0, 0.0)


def bar_end_points(T_world_from_bar: np.ndarray, z_range: tuple) -> np.ndarray:
    """The two ends of a bar's centre line in world.

    Args:
        T_world_from_bar (np.ndarray): 4x4 bar pose.
        z_range (tuple): ``(z_min, z_max)`` from `bar_z_range`.

    Returns:
        np.ndarray: (2, 3) start and end, metres.
    """
    return transform_points(T_world_from_bar, np.array([[0.0, 0.0, z_range[0]], [0.0, 0.0, z_range[1]]]))


# * ================================================================ debug lines and text

def draw_bar_fit(items: dict, client_id: int, *, bar_label: str, z_range: tuple, T_design: np.ndarray,
                 fit: BarFit, sensed_points: np.ndarray, design: BarMarkers, rhino_rgb: Sequence[float],
                 mocap_rgb: Sequence[float], show_rhino_lines: bool = True) -> dict:
    """Draw (or redraw in place) the debug lines and text of one bar fit.

    Every item is keyed by a fixed name (``'rhino_axis'``, ``'sensed_3_0'``, ...)
    and re-issued over the item of the last drawing with the same name, so
    the 10 Hz live redraw does not flicker. Items of the last drawing
    that are not drawn this time (a marker less in view) are removed.

    Args:
        items (dict): ``{name: debug item id}`` of the last drawing (``{}`` the first time).
        client_id (int): The PyBullet client.
        bar_label (str): Short bar name for the labels, e.g. ``'B20'``.
        z_range (tuple): ``(z_min, z_max)`` of the bar, from `bar_z_range`.
        T_design (np.ndarray): 4x4 Rhino (designed) bar pose.
        fit (BarFit): The fit; its MoCap centre line is drawn only when ``fit.ok``.
        sensed_points (np.ndarray): (N, 3) the sensed markers, row i = sensed index i of the fit.
        design (BarMarkers): The bar's design markers.
        rhino_rgb: RGB of the Rhino centre line and label.
        mocap_rgb: RGB of the MoCap centre line and label.
        show_rhino_lines (bool): Draw the Rhino centre line and its label; when
            False they are left out (and so removed if the last drawing had them).

    Returns:
        dict: ``{name: debug item id}`` of this drawing, for the next call.
    """
    drawn = {}

    def line(key: str, a: Sequence[float], b: Sequence[float], rgb: Sequence[float], width: float) -> None:
        """Draw (or redraw in place) the line ``key`` from a to b."""
        drawn[key] = p.addUserDebugLine(list(a), list(b), lineColorRGB=list(rgb), lineWidth=width, lifeTime=0,
                                        replaceItemUniqueId=items.get(key, -1), physicsClientId=client_id)

    def text(key: str, words: str, at: Sequence[float], rgb: Sequence[float], size: float = 1.2) -> None:
        """Draw (or redraw in place) the text ``key`` at a point."""
        drawn[key] = p.addUserDebugText(words, list(at), textColorRGB=list(rgb), textSize=size, lifeTime=0,
                                        replaceItemUniqueId=items.get(key, -1), physicsClientId=client_id)

    # * The two centre lines, each with its label above the bar's middle.
    rhino_ends = bar_end_points(T_design, z_range)
    rhino_mid = rhino_ends.mean(axis=0)
    if show_rhino_lines:
        line('rhino_axis', rhino_ends[0], rhino_ends[1], rhino_rgb, 6.0)
        text('rhino_label', f"{bar_label} Rhino", rhino_mid + [0.0, 0.0, RHINO_LABEL_RISE_M], rhino_rgb)
    T_fit = fit.T_world_from_bar
    if fit.ok:
        mocap_ends = bar_end_points(T_fit, z_range)
        line('mocap_axis', mocap_ends[0], mocap_ends[1], mocap_rgb, 6.0)
        text('mocap_label', f"{bar_label} MoCap {fit.n_matched}/{fit.n_design} rms {fit.rms_m * 1000:.1f} mm",
             mocap_ends.mean(axis=0) + [0.0, 0.0, MOCAP_LABEL_RISE_M], mocap_rgb)
    else:
        text('mocap_label', f"{bar_label} MoCap FAILED: {fit.reason}",
             rhino_mid + [0.0, 0.0, MOCAP_LABEL_RISE_M], RED_RGB)

    # * Every sensed marker as a small 3-line cross: white = matched, red = stray.
    sensed_points = np.asarray(sensed_points, dtype=float).reshape(-1, 3)
    for i, point in enumerate(sensed_points):
        rgb = WHITE_RGB if i in fit.correspondences else RED_RGB
        for k in range(3):
            arm = np.zeros(3)
            arm[k] = MARKER_CROSS_HALF_M
            line(f"sensed_{i}_{k}", point - arm, point + arm, rgb, 3.0)

    # * Matched marker -> its design marker at the fitted pose (black), and the
    # * names of the design markers nobody matched (red), at the fitted pose
    # * when there is one, else at the Rhino pose.
    placed = transform_points(T_fit if T_fit is not None else T_design, design.points())
    if T_fit is not None:
        for i, d in fit.correspondences.items():
            line(f"match_{i}", sensed_points[i], placed[d], BLACK_RGB, 2.0)
    names = design.names()
    for d in fit.missing_design:
        text(f"missing_{d}", names[d], placed[d], RED_RGB, 1.0)

    remove_debug_items({key: item for key, item in items.items() if key not in drawn}, client_id)
    return drawn


def remove_debug_items(items: dict, client_id: int) -> None:
    """Delete debug lines / text.

    Args:
        items (dict): ``{name: debug item id}``.
        client_id (int): The PyBullet client.
    """
    for item in items.values():
        try:
            p.removeUserDebugItem(item, physicsClientId=client_id)
        except p.error:
            pass  # the client was closed already (a new design problem was loaded)
