"""Measure where the parts and the table really are, by touching them with the punch tip.

* The planner scatters the parts on a synthetic table. To run a plan on the real robot, it
* has to be told where the parts actually lie. This tool is how the operator says so.
*
* The workflow has three stages:
*
*   1. OFF the robot: `export_layout.py` (in the fixtureless-assembly repo) writes a nominal
*      layout JSON plus a 1:1 outline sheet. The sheet is printed and laid on the table, and
*      the real parts are placed on their outlines. That fixes every part's pose RELATIVE to
*      every other -- the whole layout is now one rigid body.
*
*   2. HERE: the operator jogs the arm (pendant free-drive) so the punch tip sits on each
*      RED CROSS printed on the sheet, and clicks "Record mark point". The crosses are the
*      calibration reference, not the parts: they are printed, so their positions are exact
*      by construction; they are crisp marks rather than a moulded edge; and they span the
*      whole sheet, which is a far longer baseline than any single part. One planar fit of
*      the sheet then moves every part at once, because stage 1 made them one rigid body.
*
*   3. BACK in the planner: `main.py --layout-json <the file this writes>` re-solves for the
*      measured cell. The layout travels inside the saved plan, so the motion-planning, sim
*      and export stages need no extra flag.
*
* Nothing here commands the robot. The arms are only READ, over RTDE, so both pendants stay
* in local/free-drive mode the whole time -- the same read-only posture as
* grasp_calib_monitor. The punch tip offset comes from the pendant's 4-point TCP wizard, via
* data/calibration_data/<date>/config.yaml.

Frames: every coordinate is in the robot base frame (base_footprint, z=0 at the floor),
which is exactly the fixtureless rai world frame -- so a point measured here is directly a
planner coordinate and needs no conversion.

Run (after colcon build):
    ros2 run husky_assembly_teleop pickup_calib <layout_nominal.json>
    ros2 run husky_assembly_teleop pickup_calib <layout_nominal.json> --offline   # no robot
"""

import argparse
import json
import os
import shutil
import sys
import time
from datetime import datetime

import numpy as np
import pybullet as p
import pybullet_planning as pp
import yaml
from scipy.spatial.transform import Rotation as Rot

from husky_assembly_teleop import CALIBRATION_DATE, DATA_DIRECTORY
from husky_assembly_teleop import common as _common
from husky_assembly_teleop.common import (Button, Dropdown, Separator, Slider,
                                          TextInput,
                                          HUSKY_DUAL_UR5e_JOINT_NAMES,
                                          create_end_effector, load_robot)
from husky_assembly_teleop.husky_robot import UR5e_HOME_STATE
from husky_assembly_teleop.ui_backend import make_backend

# ? ur_rtde is only needed to read the live arms; --offline must work without it.
try:
    from rtde_receive import RTDEReceiveInterface
except ImportError:
    RTDEReceiveInterface = None

# --- --- CONSTANTS --- ---

LAYOUT_SCHEMA = 'assembly-pickup-layout-v1'
TICK_PERIOD_S = 0.05               # 20 Hz UI tick, same as the other monitors
ARM_SIDES = ('left', 'right')      # index 0 = left, 1 = right (repo convention)
DEFAULT_IPS = ('192.168.131.40', '192.168.131.41')

# A fit needs at least this many marks; three is what makes the turn trustworthy.
MIN_FIT_POINTS = 2
JOINT_LABELS = ['pan', 'lift', 'elbow', 'w1', 'w2', 'w3']


# --- --- LAYOUT FILE --- ---

def load_layout(path: str) -> dict:
    """Read a layout JSON and check that it is one.

    Args:
        path (str): Path to the layout file.

    Returns:
        dict: The layout.

    Raises:
        ValueError: If the schema marker or a required block is missing.
    """
    with open(path) as f:
        layout = json.load(f)
    if layout.get('schema') != LAYOUT_SCHEMA:
        raise ValueError(f"{path}: schema {layout.get('schema')!r} is not {LAYOUT_SCHEMA!r}")
    for key in ('table', 'parts'):
        if key not in layout:
            raise ValueError(f'{path}: layout has no {key!r} block')
    return layout


def save_layout(path: str, layout: dict) -> None:
    """Write a layout JSON.

    Args:
        path (str): Destination path.
        layout (dict): The layout to write.
    """
    with open(path, 'w') as f:
        json.dump(layout, f, indent=2)


def load_punch_tool_offsets(date: str = CALIBRATION_DATE) -> dict:
    """Per-arm tool0 -> punch-tip offsets from the calibration config.

    The values come from the UR pendant's 4-point TCP wizard and are typed into
    ``punch_tool.<arm>.offset_xyz``. Mirrors husky_monitor._load_punch_tool_config, kept as a
    free function so this tool never imports the ROS-bound monitor.

    Args:
        date (str): Calibration date folder under data/calibration_data.

    Returns:
        dict: ``{0: np.array([x, y, z]), 1: ...}`` for left and right.

    Raises:
        FileNotFoundError: If that date has no config.yaml.
        KeyError: If the file carries no punch_tool offsets at all.
    """
    path = os.path.join(DATA_DIRECTORY, 'calibration_data', date, 'config.yaml')
    with open(path) as f:
        config = yaml.safe_load(f) or {}
    punch = config.get('punch_tool') or {}
    offsets = {}
    legacy = punch.get('offset_xyz')          # older files carry one shared offset
    if legacy is not None:
        offsets = {0: np.array(legacy, dtype=float), 1: np.array(legacy, dtype=float)}
    for i, arm in enumerate(ARM_SIDES):
        arm_config = punch.get(arm) or {}
        if 'offset_xyz' in arm_config:
            offsets[i] = np.array(arm_config['offset_xyz'], dtype=float)
    if not offsets:
        raise KeyError(f'{path} has no punch_tool offsets; run the pendant TCP wizard and '
                       f'write punch_tool.<arm>.offset_xyz')
    return offsets


# --- --- LAYOUT GEOMETRY --- ---

def yaw_matrix(yaw: float) -> np.ndarray:
    """2x2 rotation about world z.

    Args:
        yaw (float): Angle in radians.

    Returns:
        np.ndarray: The rotation matrix.
    """
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]])


def corner_world_xy(entry: dict) -> np.ndarray:
    """The part footprint's four corners in the robot base frame.

    Only used to warn when a part hangs off the table; the calibration itself measures the
    printed sheet, not the parts.

    Args:
        entry (dict): One layout part entry.

    Returns:
        np.ndarray: A (4, 2) array of corner positions [m].
    """
    hx, hy = entry['footprint_half_xy']
    local = np.array([[hx, hy], [-hx, hy], [-hx, -hy], [hx, -hy]])
    return (np.asarray(entry['xy'], dtype=float)
            + local @ yaw_matrix(float(entry['yaw'])).T)


def part_pose_for_pybullet(layout: dict, entry: dict) -> tuple:
    """The part's full 6-DOF pose for the 3D view.

    The part lies the way the assembly says (its scatter quaternion), turned by the measured
    yaw about world z, resting on the table with the planner's small float.

    Args:
        layout (dict): The whole layout (for the table and the part float).
        entry (dict): One layout part entry.

    Returns:
        tuple: ``(position_xyz, quaternion_xyzw)`` for PyBullet.
    """
    z = (float(layout['table']['top_z']) + float(entry['height']) / 2.0
         + float(layout.get('part_float', 0.0)))
    # ! The layout stores quaternions wxyz (rai's order); PyBullet wants xyzw.
    w, x, y, zq = entry['scatter_quat_wxyz']
    scatter = Rot.from_quat([x, y, zq, w])
    total = Rot.from_euler('z', float(entry['yaw'])) * scatter
    return ([float(entry['xy'][0]), float(entry['xy'][1]), z],
            [float(v) for v in total.as_quat()])


def fit_planar_pose(local_xy, world_xy) -> tuple:
    """Fit the x/y and yaw that best carry known corners onto touched points.

    A 2-D Kabsch fit: centre both point sets, take the SVD of their cross-covariance, and
    guard against a reflection (which would mirror the part instead of turning it).

    Args:
        local_xy: (n, 2) corner positions in the part's outline frame.
        world_xy: (n, 2) matching touched positions in the base frame.

    Returns:
        tuple: ``(xy, yaw, rms)`` -- the fitted centre [m], the turn [rad], and the
        root-mean-square residual [m].

    Raises:
        ValueError: If fewer than MIN_FIT_POINTS points are given.
    """
    local = np.asarray(local_xy, dtype=float).reshape(-1, 2)
    world = np.asarray(world_xy, dtype=float).reshape(-1, 2)
    if len(local) < MIN_FIT_POINTS:
        raise ValueError(f'a planar fit needs at least {MIN_FIT_POINTS} points, got {len(local)}')
    lc, wc = local.mean(axis=0), world.mean(axis=0)
    H = (local - lc).T @ (world - wc)
    U, _S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:                  # a mirror is never a rigid placement
        Vt[1, :] *= -1.0
        R = Vt.T @ U.T
    xy = wc - R @ lc
    residual = world - (local @ R.T + xy)
    rms = float(np.sqrt((residual ** 2).sum(axis=1).mean()))
    return xy, float(np.arctan2(R[1, 0], R[0, 0])), rms


# ! How far the crosses may have moved before a re-touch counts as drift rather than
# ! touch noise. Yesterday's take had 0.34 mm rms; a millimetre or two is a nudged sheet.
DRIFT_WARN_MM = 2.0
# ! And how far the modelled table may sit from where the crosses were actually touched.
# ! A mark is printed on the sheet, the sheet lies on the table: the touch z IS the table.
TABLE_MISMATCH_WARN_MM = 3.0


def touch_drift(marks: list, points: list, reference: dict = None,
                table_top_z: float = None) -> dict:
    """Compare fresh touches of the crosses with where the layout says they are.

    Loading a CALIBRATED layout and touching its crosses again is the cheapest
    check there is of whether a calibration still holds: the marks in that file
    are where the sheet was measured to be, so any distance between a new touch
    and its mark is drift (the sheet moved, the robot moved, or the punch tip
    changed) -- before anything else in the pipeline is blamed.

    Nothing here changes the layout; it only reports.

    Args:
        marks (list): The layout's `calibration_marks` (name, xy) -- for a
            calibrated file these are the previously measured positions.
        points (list): The new touches (`mark`, `tip_xyz`), repeats averaged.
        reference (dict): The layout's saved `measurements`, if any; its
            `points` give the previous touch of each mark, so z can be compared
            touch to touch as well.
        table_top_z (float): The layout's modelled table top [m], or None.

    Returns:
        dict: ``{'rows': [...], 'fit': {...} | None, 'text': str}``. Each row has
        the mark name, the new touch, the drift in xy [mm], and -- when known --
        dz against the previous touch and against the modelled table [mm].
        `fit` is the rigid shift/turn/rms the new touches imply, when >= 2 marks.
    """
    by_mark = {}
    for pt in points:
        by_mark.setdefault(pt['mark'], []).append(np.asarray(pt['tip_xyz'], dtype=float))
    saved_xy = {m['name']: np.asarray(m['xy'], dtype=float) for m in marks}
    previous = {}
    for pt in (reference or {}).get('points', []):
        previous.setdefault(pt['mark'], []).append(np.asarray(pt['tip_xyz'], dtype=float))

    rows, local, world = [], [], []
    for name, touches in by_mark.items():
        if name not in saved_xy:
            continue
        new = np.mean(touches, axis=0)
        row = {'mark': name, 'new_xyz': new.tolist(),
               'dxy_mm': (1000.0 * (new[:2] - saved_xy[name])).tolist(),
               'd_mm': 1000.0 * float(np.linalg.norm(new[:2] - saved_xy[name]))}
        if name in previous:
            row['dz_vs_previous_touch_mm'] = 1000.0 * float(new[2] - np.mean(previous[name], axis=0)[2])
        if table_top_z is not None:
            row['dz_vs_table_mm'] = 1000.0 * float(new[2] - table_top_z)
        rows.append(row)
        local.append(saved_xy[name]); world.append(new[:2])

    fit = None
    if len(local) >= 2:
        xy, yaw, rms = fit_planar_pose(local, world)
        fit = {'shift_mm': [1000.0 * float(xy[0]), 1000.0 * float(xy[1])],
               'turn_deg': float(np.rad2deg(yaw)), 'rms_mm': 1000.0 * rms, 'n_marks': len(local)}

    lines = []
    for row in rows:
        line = (f"  {row['mark']:14s} drift {row['d_mm']:5.1f} mm  (dx {row['dxy_mm'][0]:+6.1f}, "
                f"dy {row['dxy_mm'][1]:+6.1f})")
        if 'dz_vs_previous_touch_mm' in row:
            line += f"  dz vs last touch {row['dz_vs_previous_touch_mm']:+6.1f} mm"
        if 'dz_vs_table_mm' in row:
            line += f"  z vs modelled table {row['dz_vs_table_mm']:+6.1f} mm"
        lines.append(line)
    if fit:
        lines.append(f"  rigid: shift ({fit['shift_mm'][0]:+.1f}, {fit['shift_mm'][1]:+.1f}) mm, "
                     f"turn {fit['turn_deg']:+.2f} deg, rms {fit['rms_mm']:.2f} mm "
                     f"from {fit['n_marks']} mark(s)")
    worst = max((r['d_mm'] for r in rows), default=0.0)
    if rows:
        lines.append('  => ' + ('calibration HOLDS (all within %.1f mm)' % DRIFT_WARN_MM
                                if worst <= DRIFT_WARN_MM else
                                f'DRIFT: worst mark {worst:.1f} mm -- re-fit and save'))
    return {'rows': rows, 'fit': fit, 'text': '\n'.join(lines)}


def anchor_delta(nominal_xy, nominal_yaw: float, measured_xy, measured_yaw: float) -> dict:
    """The rigid move that carries a part from where it was drawn to where it was measured.

    Args:
        nominal_xy: The part's layout position before measuring [m].
        nominal_yaw (float): Its layout yaw before measuring [rad].
        measured_xy: The fitted position [m].
        measured_yaw (float): The fitted yaw [rad].

    Returns:
        dict: ``{"dyaw": float, "t": [x, y]}`` -- turn first, then translate.
    """
    dyaw = float(measured_yaw) - float(nominal_yaw)
    t = np.asarray(measured_xy, dtype=float) - yaw_matrix(dyaw) @ np.asarray(nominal_xy,
                                                                            dtype=float)
    return {'dyaw': dyaw, 't': [float(t[0]), float(t[1])]}


def apply_delta(entry: dict, delta: dict) -> None:
    """Move one part by a rigid delta, in place.

    Args:
        entry (dict): The layout part entry to move.
        delta (dict): From `anchor_delta`.
    """
    dyaw, t = float(delta['dyaw']), np.asarray(delta['t'], dtype=float)
    xy = yaw_matrix(dyaw) @ np.asarray(entry['xy'], dtype=float) + t
    entry['xy'] = [float(xy[0]), float(xy[1])]
    entry['yaw'] = float(entry['yaw']) + dyaw


# --- --- THE TOOL --- ---

class PickupCalibTool:
    """PyBullet + DPG tool that turns punch-tip touches into a measured layout."""

    def __init__(self, args):
        self.args = args
        self.layout = load_layout(args.layout_json)
        self.nominal = json.loads(json.dumps(self.layout))    # for the "nominal" ghosts
        self.arm_index = ARM_SIDES.index(args.arm)
        # ? A layout that already carries a measurement record is a CALIBRATED one being
        # ? re-checked: its marks are the measured positions, fresh touches against them
        # ? are drift, and saving overwrites it (after a timestamped backup) instead of
        # ? minting layout_nominal_calibrated_calibrated.json.
        self.reference = self.layout.get('measurements')
        self.out_path = args.out or (
            args.layout_json if self.reference else
            os.path.splitext(args.layout_json)[0] + '_calibrated.json')
        self._warn_table_vs_saved_touches()

        # Punch tip offset: the pendant's 4-point TCP result, or an explicit override.
        if args.punch_offset is not None:
            self.punch_offset = np.array(args.punch_offset, dtype=float)
            self.offset_source = 'command line --punch-offset'
        else:
            self.punch_offset = load_punch_tool_offsets(args.calib_date)[self.arm_index]
            self.offset_source = (f'data/calibration_data/{args.calib_date}/config.yaml '
                                  f'punch_tool.{args.arm}')

        # --- measurement records ---
        self.points = []          # every touch: part, corner, level, q6, tip xyz
        self.fits = {}            # part -> {"xy", "yaw", "rms_mm", "n_points"}
        self.table_points = []    # touches used only for the table top / near edge
        self.delta = None         # the anchor transform, once applied
        self.near_edge_x = None
        self.status = 'ready -- select a part and a corner, then touch it'

        # ! These must be set BEFORE _build_scene: it draws the selected part's corner
        # ! markers and reads the live configuration, so both are needed already.
        # ! Both arms are read and displayed. Only the measuring arm takes touches, but an
        # ! idle arm left at the URDF zero hangs through the floor and tells the operator
        # ! nothing about where the other arm really is.
        self.rtde = [None, None]
        self._last_q = [UR5e_HOME_STATE.copy(), UR5e_HOME_STATE.copy()]
        self.selected_part = sorted(self.layout['parts'])[0]
        # The calibration reference is the printed sheet, not the parts: the marks are drawn
        # at exact coordinates, they are crisp crosses, and they span the whole sheet.
        self.marks = [dict(m) for m in (self.layout.get('calibration_marks') or [])]
        self.nominal_marks = [dict(m) for m in self.marks]
        if not self.marks:
            raise ValueError(
                f"{args.layout_json} has no 'calibration_marks'. Re-run export_layout.py: "
                f"the sheet's printed crosses are what this tool measures.")
        self.mark_names = [m['name'] for m in self.marks]
        self.selected_mark = self.mark_names[0]

        self._build_scene()
        self._build_ui()

    # --- --- 3D SCENE --- ---

    def _build_scene(self):
        """Robot, punch cone, table and one ghost + one solid mesh per part."""
        pp.connect(use_gui=True, shadows=True, color=[0.9, 0.9, 1.0])
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=pp.CLIENT)
        pp.draw_pose(pp.unit_pose(), 0.3)
        with pp.LockRenderer(), pp.HideOutput():
            self.robot = load_robot(dual_arm=True)
        self.joints12 = pp.joints_from_names(
            self.robot, HUSKY_DUAL_UR5e_JOINT_NAMES[0] + HUSKY_DUAL_UR5e_JOINT_NAMES[1])
        # Per-arm handles: index 0 = left, 1 = right. Both are written every tick.
        self.arm_joints_by_index = [
            pp.joints_from_names(self.robot, HUSKY_DUAL_UR5e_JOINT_NAMES[i])
            for i in range(2)]
        self.arm_joints = self.arm_joints_by_index[self.arm_index]
        # ! Pose both arms before anything reads the flange: the URDF zero folds an arm
        # ! through the floor, which is both wrong on screen and a nonsense FK reading.
        for index in range(2):
            pp.set_joint_positions(self.robot, self.arm_joints_by_index[index],
                                   [float(v) for v in UR5e_HOME_STATE])
        self.tool0_link = pp.link_from_name(
            self.robot, f'{self.args.arm}_ur_arm_tool0')

        # The punch, drawn as the cone whose apex IS the calibrated tip.
        with pp.LockRenderer(), pp.HideOutput():
            self.punch = create_end_effector('punch_tool',
                                             punch_tool_offset=self.punch_offset)
        # ! create_attachment freezes the CURRENT parent-to-child transform, so the cone has
        # ! to be standing on the flange BEFORE the attachment is made. Skipping this leaves
        # ! it attached at whatever pose it was created at (the world origin), and the punch
        # ! then floats away from the arm. Same order as the engine's gripper models.
        # The cone mesh is authored in tool0 coordinates -- base ring at the flange, apex at
        # the calibrated tip -- so the flange pose IS the body pose, with no extra offset.
        pp.set_pose(self.punch, pp.get_link_pose(self.robot, self.tool0_link))
        self.punch_attach = pp.create_attachment(self.robot, self.tool0_link, self.punch)
        self.punch_attach.assign()

        self.table_body = None
        self.part_bodies = {}         # part -> solid body (the current pose)
        self.ghost_bodies = {}        # part -> translucent body (the nominal pose)
        self._rebuild_table()
        for name, entry in sorted(self.layout['parts'].items()):
            self.ghost_bodies[name] = self._spawn_part(entry, (0.55, 0.65, 0.85, 0.25))
            self.part_bodies[name] = self._spawn_part(entry, (0.82, 0.66, 0.42, 1.0))
        self.corner_markers = []
        self.mark_labels = []
        self.part_labels = []
        self.tip_handles = []
        self._refresh_parts()

    def _spawn_part(self, entry, color):
        """Create one part body: its visual mesh when there is one, else a box."""
        mesh = entry.get('visual_mesh')
        with pp.LockRenderer(), pp.HideOutput():
            if mesh and os.path.exists(mesh):
                body = pp.create_obj(mesh, scale=1.0, collision=False, color=color)
            else:
                hx, hy = entry['footprint_half_xy']
                body = pp.create_box(2 * hx, 2 * hy, entry['height'], color=color)
        return body

    def _rebuild_table(self):
        """Draw the table box at the layout's current geometry."""
        table = self.layout['table']
        if self.table_body is not None:
            pp.remove_body(self.table_body)
        sx, sy = table['size_xy']
        with pp.LockRenderer(), pp.HideOutput():
            self.table_body = pp.create_box(sx, sy, table['thickness'],
                                            color=(0.55, 0.45, 0.35, 0.45))
        pp.set_pose(self.table_body, pp.Pose(pp.Point(
            table['center_xy'][0], table['center_xy'][1],
            table['top_z'] - table['thickness'] / 2.0)))

    def _refresh_parts(self):
        """Move every part body to its current layout pose, ghosts to the nominal one."""
        for name, entry in self.layout['parts'].items():
            pos, quat = part_pose_for_pybullet(self.layout, entry)
            pp.set_pose(self.part_bodies[name], (pos, quat))
        for name, entry in self.nominal['parts'].items():
            pos, quat = part_pose_for_pybullet(self.nominal, entry)
            pp.set_pose(self.ghost_bodies[name], (pos, quat))
        self._refresh_labels()
        self._refresh_corner_markers()

    def _refresh_labels(self):
        """Float each part's name above it, so the operator can tell them apart.

        The label also carries the part's state, because "which part is which" and "have I
        done this one yet" are the same question while measuring. The selected part is red
        and the others are dark, matching the corner markers.
        """
        for handle in self.part_labels:
            pp.remove_debug(handle)
        self.part_labels = []
        top_z = float(self.layout['table']['top_z'])
        for name, entry in sorted(self.layout['parts'].items()):
            if entry.get('measured'):
                state = 'measured'
            elif entry.get('anchored'):
                state = 'anchored'
            else:
                state = 'nominal'
            selected = name == self.selected_part
            # A hand's width above the part, so the text clears the mesh and the markers.
            z = top_z + float(entry['height']) + 0.08
            self.part_labels.append(pp.add_text(
                f'{name} ({state})' + ('  <-- selected' if selected else ''),
                position=[entry['xy'][0], entry['xy'][1], z],
                color=(0.9, 0.1, 0.1) if selected else (0.15, 0.15, 0.2)))

    def _refresh_corner_markers(self):
        """Show the sheet's calibration crosses, the target one highlighted.

        They sit on the table top, because that is where the printed sheet lies.
        """
        for handle in self.corner_markers:
            pp.remove_body(handle)
        for handle in self.mark_labels:
            pp.remove_debug(handle)
        self.corner_markers, self.mark_labels = [], []
        z = float(self.layout['table']['top_z'])
        for mark in self.marks:
            hit = mark['name'] == self.selected_mark
            with pp.LockRenderer(), pp.HideOutput():
                marker = pp.create_sphere(0.007 if hit else 0.004,
                                          color=((1.0, 0.15, 0.15, 1.0) if hit
                                                 else (0.9, 0.4, 0.4, 0.7)))
            pp.set_pose(marker, pp.Pose(pp.Point(mark['xy'][0], mark['xy'][1], z)))
            self.corner_markers.append(marker)
            touched = sum(1 for pt in self.points if pt['mark'] == mark['name'])
            label = mark['name'] + (f' [{touched}]' if touched else '')
            self.mark_labels.append(pp.add_text(
                label + ('  <-- touch this' if hit else ''),
                position=[mark['xy'][0], mark['xy'][1], z + 0.02],
                color=(0.9, 0.1, 0.1) if hit else (0.5, 0.25, 0.25)))

    # --- --- LIVE ROBOT --- ---

    def tip_position(self, q6) -> np.ndarray:
        """Where the punch tip is, for a given configuration of the measuring arm.

        Sets the arm in the 3D view and reads forward kinematics, exactly as
        husky_world.record_punch_reference does: ``world_from_tool0 * tool0_from_tip``.

        Args:
            q6: Six joint angles of the measuring arm [rad].

        Returns:
            np.ndarray: The tip position in the base frame [m].
        """
        pp.set_joint_positions(self.robot, self.arm_joints, [float(v) for v in q6])
        world_from_tool0 = pp.get_link_pose(self.robot, self.tool0_link)
        tip, _ = pp.multiply(world_from_tool0, pp.Pose(point=self.punch_offset))
        return np.asarray(tip, dtype=float)

    def live_arm_q6(self, index: int) -> np.ndarray:
        """One arm's live joint values, from RTDE or (measuring arm only) the sliders.

        Args:
            index (int): 0 for the left arm, 1 for the right.

        Returns:
            np.ndarray: Six joint angles [rad]. The last good value is returned when that
            arm is not connected, so a dropped link never blanks the display.
        """
        if self.args.offline and index == self.arm_index:
            return np.asarray([s.value for s in self.joint_sliders], dtype=float)
        if self.rtde[index] is None:
            return self._last_q[index]
        try:
            self._last_q[index] = np.asarray(self.rtde[index].getActualQ(), dtype=float)
        except Exception as e:                 # a dropped link must not kill the window
            self.status = f'{ARM_SIDES[index]} RTDE read failed ({e}); showing the last value'
        return self._last_q[index]

    def live_q6(self) -> np.ndarray:
        """The measuring arm's live joint values."""
        return self.live_arm_q6(self.arm_index)

    def on_connect(self):
        """Open read-only RTDE receive links to BOTH arms.

        The measuring arm is required -- without it there is nothing to measure. The other
        arm is only for the display, so a failure there is reported and tolerated.
        """
        if RTDEReceiveInterface is None:
            self.status = 'ur_rtde is not installed -- use --offline, or pip install ur_rtde'
            return
        ips = (self.args.left_ip, self.args.right_ip)
        connected = []
        for index, ip in enumerate(ips):
            try:
                self.rtde[index] = RTDEReceiveInterface(ip, float(self.args.freq))
                connected.append(f'{ARM_SIDES[index]}@{ip}')
            except Exception as e:
                self.rtde[index] = None
                required = index == self.arm_index
                print(f'[pickup] {"ERROR" if required else "note"}: RTDE connect to the '
                      f'{ARM_SIDES[index]} arm at {ip} failed: {e}')
        if self.rtde[self.arm_index] is None:
            self.status = (f'the {self.args.arm} arm (the measuring one) did NOT connect -- '
                           f'see the terminal')
        else:
            self.status = 'RTDE connected: ' + ', '.join(connected)
        print(f'[pickup] {self.status}')

    # --- --- UI --- ---

    def _build_ui(self):
        """Create the control panel."""
        _common._global_backend = make_backend(
            use_dpg=True, window_title=f'Pickup calibration - {os.path.basename(self.args.layout_json)}',
            width=760, height=980, font_size=18)

        self.widgets = []
        self.widgets.append(Separator(
            f'Pickup calibration -- {self.args.arm} arm'
            + (' -- VERIFY / RE-CALIBRATE a saved layout' if self.reference else '')))
        self.widgets.append(Separator(f'layout: {os.path.basename(self.args.layout_json)}'))
        self.widgets.append(Separator(
            f'punch tip offset [mm]: {1000 * self.punch_offset[0]:.2f}, '
            f'{1000 * self.punch_offset[1]:.2f}, {1000 * self.punch_offset[2]:.2f}'))
        self.widgets.append(Separator(f'  from {self.offset_source}'))

        if self.args.offline:
            self.widgets.append(Separator('OFFLINE: drive the arm with these sliders'))
            # ! Six separate Sliders, not a SliderGroup: only Slider exposes .value, which
            # ! is read live from the widget. A group only reports through its callback, and
            # ! a callback can be missed (the reset_ui slider gotcha).
            defaults = [0.0, -1.2, 1.2, -1.5, -1.57, 0.0]
            self.joint_sliders = [
                Slider(f'{self.args.arm} {label}', lambda *_: None, -np.pi, np.pi, default)
                for label, default in zip(JOINT_LABELS, defaults)]
            self.widgets += self.joint_sliders
        else:
            self.widgets.append(Button('Connect RTDE (read-only)', self.on_connect))

        self.widgets.append(Separator('--- measure the printed sheet ---'))
        self.widgets.append(Separator('touch the RED CROSSES on the paper, not the parts'))
        self.widgets.append(Dropdown('mark', self._on_mark, self.mark_names, 0))
        self.widgets.append(Button('Record mark point', self.on_record_mark))
        self.widgets.append(Button('Undo last point', self.on_undo))
        self.widgets.append(Button('Check drift vs saved crosses (changes nothing)',
                                   self.on_check_drift))
        self.widgets.append(Button('Fit sheet (moves every part)', self.on_fit))
        self.widgets.append(Separator('--- view ---'))
        self.widgets.append(Dropdown('highlight part', self._on_part,
                                     sorted(self.layout['parts']), 0))

        self.widgets.append(Separator('--- table ---'))
        table = self.layout['table']
        self.size_x_input = TextInput('table size x [m]', lambda *_: None,
                                      f"{table['size_xy'][0]:.4f}", numeric=True)
        self.size_y_input = TextInput('table size y [m]', lambda *_: None,
                                      f"{table['size_xy'][1]:.4f}", numeric=True)
        self.thickness_input = TextInput('table thickness [m]', lambda *_: None,
                                         f"{table['thickness']:.4f}", numeric=True)
        self.widgets += [self.size_x_input, self.size_y_input, self.thickness_input]
        self.widgets.append(Button('Record table-top point', self.on_record_table_top))
        self.widgets.append(Button('Record near-edge point', self.on_record_near_edge))
        self.widgets.append(Button('Apply table numbers', self.on_apply_table))

        self.widgets.append(Separator('--- save ---'))
        self.widgets.append(Button('Save layout', self.on_save))

        self.tip_sep = Separator('tip: --')
        self.table_sep = Separator('table: --')
        self.parts_sep = Separator('parts: --')
        self.status_sep = Separator('status: ready')
        self.widgets += [self.tip_sep, self.table_sep, self.parts_sep, self.status_sep]

    def _on_part(self, index):
        self.selected_part = sorted(self.layout['parts'])[int(index)]
        self._refresh_labels()

    def _on_mark(self, index):
        self.selected_mark = self.mark_names[int(index)]
        self._refresh_corner_markers()

    # --- --- ACTIONS --- ---

    def on_record_mark(self):
        """Store the current tip position as a touch of the selected sheet mark."""
        q6 = self.live_q6()
        tip = self.tip_position(q6)
        self.points.append({
            'mark': self.selected_mark, 'q6': [float(v) for v in q6],
            'tip_xyz': [float(v) for v in tip],
            'timestamp': datetime.now().isoformat()})
        pp.draw_pose(pp.Pose(pp.Point(*tip)), length=0.03)
        # ! A mark touch measures the table top too (the sheet lies on it), and the readout
        # ! already counted it as such -- but until 2026-09-10 only the table buttons
        # ! applied it, so a take that never pressed them saved the PLANNER's guessed
        # ! height. The first real take did exactly that: four touches at z=0.413 m,
        # ! table saved at 0.440 m, every part 27 mm too high.
        self._update_table_top()
        done = sorted({pt['mark'] for pt in self.points})
        self.status = (f"recorded mark '{self.selected_mark}' at "
                       f'({tip[0]:.4f}, {tip[1]:.4f}, {tip[2]:.4f}) -- '
                       f'{len(done)}/{len(self.marks)} mark(s) touched')
        print(f'[pickup] {self.status}')

    def on_undo(self):
        """Drop the most recent touch."""
        if not self.points:
            self.status = 'nothing to undo'
            return
        dropped = self.points.pop()
        self._update_table_top()
        self.status = f"undid mark '{dropped['mark']}'"

    def on_fit(self):
        """Fit the sheet's pose from the touched marks and move every part with it.

        The marks are PRINTED, so where each one sits on the sheet is exact. Touching them
        measures the sheet, and because the parts were placed on their printed outlines, the
        same rigid shift and turn carries all of them at once -- one fit for the whole
        layout, with a baseline as long as the sheet rather than as long as one part.
        """
        touches = list(self.points)
        # Average repeat touches of the same mark before fitting, so a mark touched twice
        # does not simply outvote the others.
        by_mark = {}
        for pt in touches:
            by_mark.setdefault(pt['mark'], []).append(pt['tip_xyz'][:2])
        nominal = {m['name']: np.asarray(m['xy'], dtype=float) for m in self.marks}
        local = [nominal[name] for name in by_mark if name in nominal]
        world = [np.mean(v, axis=0) for name, v in by_mark.items() if name in nominal]
        try:
            xy, yaw, rms = fit_planar_pose(local, world)
        except ValueError as e:
            self.status = str(e)
            return
        # fit_planar_pose returns the transform that carries the nominal marks onto the
        # measured ones; that same transform is what the whole sheet underwent.
        delta = {'dyaw': yaw, 't': [float(xy[0]), float(xy[1])]}
        for name, entry in self.layout['parts'].items():
            entry['xy'] = list(self.nominal['parts'][name]['xy'])
            entry['yaw'] = float(self.nominal['parts'][name]['yaw'])
            apply_delta(entry, delta)
            entry['measured'] = True
        for i, mark in enumerate(self.marks):
            moved = yaw_matrix(yaw) @ np.asarray(self.nominal_marks[i]['xy'], dtype=float) + np.asarray(delta['t'])
            mark['xy'] = [float(moved[0]), float(moved[1])]
        self.delta = delta
        self.fits = {'sheet': {'t_mm': [1000 * delta['t'][0], 1000 * delta['t'][1]],
                               'yaw_deg': float(np.rad2deg(yaw)),
                               'rms_mm': 1000 * rms, 'n_marks': len(local)}}
        self._refresh_parts()
        self.status = (f'sheet: shift ({1000 * xy[0]:+.1f}, {1000 * xy[1]:+.1f}) mm, '
                       f'turn {np.rad2deg(yaw):+.2f} deg from {len(local)} mark(s), '
                       f'rms {1000 * rms:.2f} mm -- all {len(self.layout["parts"])} part(s) moved')
        print(f'[pickup] {self.status}')
        if len(local) < 3:
            print('[pickup] NOTE: two marks fix the turn with no redundancy -- the rms is 0 '
                  'by construction and says nothing about accuracy. Touch a third mark.')

    def on_check_drift(self):
        """Report how far the fresh touches are from the layout's crosses. Changes nothing."""
        report = touch_drift(self.marks, self.points, self.reference,
                             self.layout['table']['top_z'])
        if not report['rows']:
            self.status = 'no marks touched yet -- touch the crosses, then check'
            return
        what = 'saved calibration' if self.reference else 'nominal sheet'
        print(f'[pickup] drift of {len(report["rows"])} touched mark(s) vs the {what}:')
        print(report['text'])
        self.status = report['text'].splitlines()[-1].strip('=> ').strip()

    def _warn_table_vs_saved_touches(self):
        """Loud check that a calibrated layout's table sits where its crosses were touched."""
        heights = [pt['tip_xyz'][2] for pt in (self.reference or {}).get('points', [])]
        if not heights:
            return
        touched, modelled = float(np.mean(heights)), float(self.layout['table']['top_z'])
        gap = 1000.0 * (modelled - touched)
        if abs(gap) > TABLE_MISMATCH_WARN_MM:
            print(f'[pickup] WARNING: this layout models the table top at {modelled:.4f} m, '
                  f'but its own {len(heights)} cross touches were made at z={touched:.4f} m '
                  f'(mean) -- {gap:+.1f} mm apart. Every part in it sits that far off the '
                  f'real table. Touch the marks again and Save: the table now follows.')

    def on_record_table_top(self):
        """Store a touch of the bare table surface."""
        q6 = self.live_q6()
        tip = self.tip_position(q6)
        self.table_points.append({'kind': 'top', 'q6': [float(v) for v in q6],
                                  'tip_xyz': [float(v) for v in tip]})
        self._update_table_top()
        self.status = f'table top point at z={tip[2]:.4f} m'

    def on_record_near_edge(self):
        """Store a touch of the table edge closest to the robot, to place the table in x."""
        q6 = self.live_q6()
        tip = self.tip_position(q6)
        self.table_points.append({'kind': 'near_edge', 'q6': [float(v) for v in q6],
                                  'tip_xyz': [float(v) for v in tip]})
        self.near_edge_x = float(tip[0])
        self._apply_table_inputs()
        self.status = f'table near edge at x={tip[0]:.4f} m'

    def _table_top_evidence(self):
        """Every touch that says something about the table top height [m]."""
        heights = [pt['tip_xyz'][2] for pt in self.table_points if pt['kind'] == 'top']
        # A mark is printed on the sheet, which lies flat on the table, so every mark touch
        # measures the table top too (one sheet thickness high, which is noise here).
        heights += [pt['tip_xyz'][2] for pt in self.points]
        return heights

    def _update_table_top(self):
        """Set the table top to the mean of every touch that measured it."""
        heights = self._table_top_evidence()
        if not heights:
            return
        self.layout['table']['top_z'] = float(np.mean(heights))
        self._rebuild_table()
        self._refresh_parts()

    def _apply_table_inputs(self):
        """Read the typed table numbers and place the table from the near edge."""
        table = self.layout['table']
        for widget, key, index in ((self.size_x_input, 'size_xy', 0),
                                   (self.size_y_input, 'size_xy', 1)):
            value = widget.value
            if value is not None:
                table[key][index] = float(value)
        thickness = self.thickness_input.value
        if thickness is not None:
            table['thickness'] = float(thickness)
        if self.near_edge_x is not None:
            # The near edge plus half the depth is the centre.
            table['center_xy'][0] = self.near_edge_x + table['size_xy'][0] / 2.0
        self._rebuild_table()

    def on_apply_table(self):
        """Apply the typed size/thickness and any recorded edge."""
        self._apply_table_inputs()
        self._update_table_top()
        self.status = 'table geometry updated'

    def on_save(self):
        """Write the measured layout, with the full measurement record beside it."""
        unmeasured = sorted(n for n, e in self.layout['parts'].items()
                            if not e.get('measured'))
        self.layout['calibration_marks'] = self.marks
        self.layout['measurements'] = {
            'timestamp': datetime.now().isoformat(),
            'arm': self.args.arm,
            'punch_offset_xyz': [float(v) for v in self.punch_offset],
            'punch_offset_source': self.offset_source,
            'points': self.points,
            'sheet_fit': self.fits.get('sheet'),
            'sheet_delta': self.delta,
            'nominal_marks': self.nominal_marks,
            'table_points': self.table_points,
            # Which planner robot is which physical arm, recorded so a later reader never has
            # to guess: the fixtureless husky scene names the RIGHT arm a1 and the LEFT a2.
            'robot_sides': {'a1': 'right', 'a2': 'left'},
            'unmeasured_parts': unmeasured,
            'reference_layout': os.path.abspath(self.args.layout_json),
        }
        if os.path.abspath(self.out_path) == os.path.abspath(self.args.layout_json):
            backup = (os.path.splitext(self.out_path)[0]
                      + datetime.now().strftime('.bak-%Y%m%d-%H%M%S.json'))
            shutil.copyfile(self.out_path, backup)
            print(f'[pickup] previous calibration kept as {backup}')
        save_layout(self.out_path, self.layout)
        self.status = f'saved -> {self.out_path}'
        print(f'[pickup] {self.status}')
        if unmeasured:
            print(f'[pickup] WARNING: the sheet was never fitted, so {unmeasured} keep their '
                  f'NOMINAL poses. Touch the marks and press "Fit sheet".')
        self._warn_parts_off_the_table()

    def _warn_parts_off_the_table(self):
        """Say so when a part's footprint leaves the table: it would fall in the sim."""
        table = self.layout['table']
        (cx, cy), (sx, sy) = table['center_xy'], table['size_xy']
        for name, entry in sorted(self.layout['parts'].items()):
            corners = corner_world_xy(entry)
            if (np.any(np.abs(corners[:, 0] - cx) > sx / 2.0)
                    or np.any(np.abs(corners[:, 1] - cy) > sy / 2.0)):
                print(f'[pickup] WARNING: {name} hangs over the table edge -- the planner '
                      f'would rest it on nothing and the sim would drop it.')

    # --- --- TICK --- ---

    def tick(self):
        """One UI frame: refresh the readouts and the live tip."""
        # Mirror BOTH arms, so the idle one shows where it really is; the measuring arm is
        # written last because tip_position reads the flange straight afterwards.
        other = 1 - self.arm_index
        pp.set_joint_positions(self.robot, self.arm_joints_by_index[other],
                               [float(v) for v in self.live_arm_q6(other)])
        q6 = self.live_q6()
        tip = self.tip_position(q6)
        self.punch_attach.assign()
        for handle in self.tip_handles:
            pp.remove_debug(handle)
        self.tip_handles = pp.draw_pose(pp.Pose(pp.Point(*tip)), length=0.05)

        self.tip_sep.set_text(f'tip: ({tip[0]:.4f}, {tip[1]:.4f}, {tip[2]:.4f}) m')
        heights = self._table_top_evidence()
        if heights:
            spread = 1000 * (max(heights) - min(heights))
            self.table_sep.set_text(
                f"table top: {self.layout['table']['top_z']:.4f} m from {len(heights)} "
                f'touch(es), spread {spread:.1f} mm')
        else:
            self.table_sep.set_text(
                f"table top: {self.layout['table']['top_z']:.4f} m (nominal)")
        touched = sorted({pt['mark'] for pt in self.points})
        fitted = self.fits.get('sheet')
        self.parts_sep.set_text(
            f'marks touched: {len(touched)}/{len(self.marks)}'
            + (f" | sheet fitted, rms {fitted['rms_mm']:.2f} mm" if fitted
               else ' | sheet NOT fitted -- parts still at nominal'))
        self.status_sep.set_text(f'status: {self.status}')

    def run(self):
        """Drive the UI until the window is closed."""
        backend = _common._global_backend
        while backend.step():
            for widget in self.widgets:
                widget.update()
            self.tick()
            time.sleep(TICK_PERIOD_S)
        print('[pickup] window closed')


def main(args=None):
    cli = argparse.ArgumentParser(
        description='Measure the real part and table poses with the punch tip.')
    cli.add_argument('layout_json', help='nominal layout from export_layout.py')
    cli.add_argument('--out', default=None,
                     help='where to write the measured layout '
                          '(default: <input>_calibrated.json)')
    cli.add_argument('--arm', choices=ARM_SIDES, default='left',
                     help='which arm carries the punch (default: left)')
    cli.add_argument('--left-ip', default=DEFAULT_IPS[0])
    cli.add_argument('--right-ip', default=DEFAULT_IPS[1])
    cli.add_argument('--freq', type=float, default=20.0, help='RTDE receive rate [Hz]')
    cli.add_argument('--punch-offset', type=float, nargs=3, default=None,
                     metavar=('X', 'Y', 'Z'),
                     help='override the tool0 -> tip offset [m] from the config file')
    cli.add_argument('--calib-date', default=CALIBRATION_DATE,
                     help='calibration_data folder holding the punch offsets')
    cli.add_argument('--offline', action='store_true',
                     help='no robot: drive the arm with joint sliders (for trying the tool)')
    parsed = cli.parse_args(sys.argv[1:] if args is None else args)

    tool = PickupCalibTool(parsed)
    tool.run()
    return 0


if __name__ == '__main__':
    sys.exit(main())
