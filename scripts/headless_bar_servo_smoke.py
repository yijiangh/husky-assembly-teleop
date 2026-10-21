"""Smoke test: the servo on the MoCap-fitted bar of a grounded bar (B20), with the real planners and IK.

No ROS, no hardware, no Motive, no window. The monitor is built without its
``__init__`` (``headless_plan_skeleton_smoke.make_monitor``: a stub Cindy, the
cfab session in PyBullet DIRECT mode) on a scratch copy of the design problem
``261016_2ModulesDemo_YH``, action ``B20_J_joint`` (B20 is a GROUNDED bar: its
two ground joints ride in the tools and its insert is 15 mm straight down onto
the floor). ``FAKE_HARDWARE = 0``, ``USE_MOCAP = 1``,
``USE_CELL_STATE_BASE_POSE = 0``, the transfer workaround box as the user has
it (``TRANSFER_AS_FREE_MOVE``), 'Servo: aim grounded bar by MoCap' ticked
(``_servo_bar_feedback``). The stub arms arrive at every commanded last
waypoint.

What is real: the monitor's exec dispatch, ``execute_transfer_then_servo``,
``execute_grounded_insert_then_servo``, ``servo_to_movement_start_live``, the
live-base IK, ``replan_transfer_to_movement_start_live`` with the REAL
bar-held straight-line planner (``plan_constrained_dual_arm_linear``, env
collisions on), 'Plan Movement' on the insert, the compliant exec of the
insert, the bar fit and the Motive listener (``receive_labeled_marker``).
Stubbed: the display-only previews (``_validate_cdfm_planned_path``,
``show_transfer_validation``, ``show_planned_joint_values``, as
``transfer_servo_check`` does). The IK and the correction planner are wrapped
by pass-through recorders (what they were asked, what they returned); the
servo record is written by the real ``_save_servoing_data``, but always into
the scratch folder.

* The simulated systematic error: the real bar sits at ``E @ T_fk_bar``, where
* ``T_fk_bar`` is FK of the left tool0 (stub base + stub arms) composed with the
* grasp, and E is a fixed world-frame rigid error that moves the bar's origin by
* 3 mm and turns it by 0.1 deg at the insert start (the base calibration / tool
* mount / grasp offset the servo cannot see in FK). Motive "streams" B20's 8
* joint markers at that pose under B20's asset id (Bxx -> 10xx): each time the
* monitor reads the marker cache, the latest frame is pushed through the
* monitor's own listener first (Motive streams from its own thread, so the
* frame a reading sees is always fresh). The markers are exact (no noise).

The checks:

1. Pre-insert servo, through 'Exec Selected Mv Traj' on the transfer (M3):
   the transfer, the mount pause (auto-confirmed), then the servo on the
   insert (M4) aiming the MoCap bar: iteration 0 reads E; it converges on the
   MoCap bar within 3 iterations; every bar-corrected IK target is
   ``T_corr @ FK`` of the reading before it (both flanges) and the IK aims it;
   every correction is a real straight-line plan; every record carries
   ``'bar'``; afterwards ``_servo_bar_carry`` names the insert, the override is
   None and ``_mocap_bar_reference`` is empty; the simulated real bar ends
   within 0.5 mm / 0.1 deg of the insert-start design pose (E cancelled).
2. 'Plan Movement' on the insert: the carry line is printed; the goal handed
   to the planner is the authored insert move (15 mm down) applied to where
   each flange stands (exact); the planned end bar is that move applied to the
   start bar, within the IK's own stop (ssik: 0.1 mm / 0.01 deg; gradient:
   1 mm / 0.01 rad, its cartesian IK accepts every waypoint there); the
   simulated real bar lands within 0.5 mm of the assembled pose at the carried
   goal (within 0.5 mm + that stop at the planned end).
3. 'Exec Selected Mv Traj' on the grounded insert ->
   ``execute_grounded_insert_then_servo``: the insert runs, M5 (the manual fix
   of the foundation) is loaded and the servo after the insert runs, straight
   lines only. ? To give it something to correct, the real bar shifts in the
   ? tools when its ground joints touch the floor (1.0 mm sideways, 0.05 deg
   ? about the vertical). The KEY question: does the live-base IK succeed and
   ? does the straight-line correction plan from M5's start, with the ground
   ? joints on the floor and env collisions on? Then it converges (the real bar
   on the assembled pose). Load guard: the same exec again, the stub force
   vector changing by 25 N after the first correction move -> the loop stops
   with the load-guard line.
4. Box unticked (tool0 mode), through the transfer exec again: the comparison
   is in every record, no bar-corrected IK target ever, no carry.
5. Box ticked, Motive silent, made-up markers (``MOCAP_BAR_FAKE_MARKERS = 1``,
   source 'fake'): never steers, says why, and its record says it aimed tool0.
6. Every servo record went to the scratch folder; the real design folder is
   untouched.

! It ALWAYS runs on a scratch copy of the problem folder in a temp dir (the
! RobotCell files are symlinked and only read); the last check confirms the
! real design folder is untouched. Servo records (JSON + PNG) go to the temp dir.

Usage:

    cd /home/yijiangh/Code/ros2_ws && source venv/bin/activate && source install/setup.bash
    INSYNC="/home/yijiangh/Insync/yijiang94817@gmail.com/Google Drive - Shared with me"
    export DESIGN_DATA_DIRECTORY="$INSYNC/2025-03 Husky Assembly/data_design_study"
    export DESIGN_PROBLEM_NAME=261016_2ModulesDemo_YH
    export HUSKY_IK_BACKEND=gradient   # ssik is not installed in this venv
    python src/husky-assembly-teleop/scripts/headless_bar_servo_smoke.py [--keep]

    With ``HUSKY_IK_BACKEND=gradient`` the goal IK is PyBullet's (it stops
    within 1 mm / 0.01 rad), so a correction lands within ~1 mm and the loop
    needs a second pass; with ssik it is exact. --keep leaves the temp dir (the
    servo JSON + PNG, progress.json) for inspection.

Exit code 0 when every check passes, 1 otherwise. Loads Cindy's ~340 MB
RobotCell once; takes about a minute.
"""

import argparse
import glob
import json
import os
import re
import shutil
import sys
import tempfile
import threading
import traceback
from types import SimpleNamespace
from typing import Optional

import numpy as np
import pybullet_planning as pp

# The other smokes in this same folder (importable because Python puts the
# script's folder on sys.path).
from headless_mocap_bar_smoke import as_natnet_frame, markers_at
from headless_plan_skeleton_smoke import (
    INSERT_TRAJ_TIME_S, Results, StubHuskyObject, add_viz_huskies, call_keeping_prints, folder_snapshot,
    make_monitor, make_scratch_problem, point_package_at, ready_for_compliant_exec, run_tasks, select_movement,
)
from husky_assembly_teleop import DESIGN_DATA_DIRECTORY, DESIGN_PROBLEM_NAME
from husky_assembly_teleop import husky_world
from husky_assembly_teleop.as_built_io import as_built_dir, read_as_built
from husky_assembly_teleop.bar_action_io import MovementKind, held_ground_joints
from husky_assembly_teleop.bar_mocap_fit import MOCAP_MARKERS_FILENAME, pose_deviation
from husky_assembly_teleop.husky_world import (
    SERVO_BAR_ANG_TOL_DEG, SERVO_BAR_POS_TOL_MM, SERVO_CONTACT_MAX_LOAD_RISE_N, mocap_bar_streaming_id,
)
from husky_assembly_teleop.plan_skeleton_io import load_plan_skeleton, problem_root
from husky_assembly_teleop.robot_registry import robot_by_name
from husky_assembly_teleop.utils import (
    joint_trajectory_from_path, path_12_from_joint_trajectory, pose_from_frame, vec12_from_conf,
)
from husky_assembly_tamp.motion_planner.api import _fk_link_frame
from husky_assembly_tamp.motion_planner.ssik_ik import ik_backend

ACTION_ID = 'B20_J_joint'
BAR = 'bar_B20'
# * The simulated systematic error E: a world-frame rigid error that moves the
# * bar's origin by this much (along a mixed direction, partly upward) and turns
# * it by this much (a tilt mostly about world y, a little yaw), at the insert start.
SYS_ERR_MM = 3.0
SYS_ERR_DEG = 0.1
SYS_ERR_DIR = (1.0, -2.0, 1.0)
SYS_ERR_AXIS = (0.2, 1.0, 0.5)
# * When its ground joints touch the floor the bar shifts in the tools by this
# * much (sideways, a turn about the vertical, at the assembled pose): what the
# * servo after the insert has to correct. Without it the carried insert already
# * lands within tolerance and the second servo stops before planning anything.
TOUCH_ERR_MM = 1.0
TOUCH_ERR_DEG = 0.05
TOUCH_ERR_DIR = (1.0, 1.0, 0.0)
TOUCH_ERR_AXIS = (0.0, 0.0, 1.0)
# The force the stub reads on both arms after the first correction move of the
# load-guard run (from 0 N, so the force vector changes by this much): above the servo's limit.
FT_PRESS_N = SERVO_CONTACT_MAX_LOAD_RISE_N + 5.0
# The transfer is a two-point standstill path at the insert's start; this short traj time keeps the run quick.
TRANSFER_TRAJ_TIME_S = 1.0
# Boxes off / fake markers: the stub base settles this far (x) off its authored
# pose after the transfer, so the tool0 loop has a correction to make.
TOOL0_RUN_BASE_SHIFT_M = 0.005
# * Tolerances. The bar-corrected IK target vs T_corr @ FK and the carried
# * insert goal vs the authored move from the actual start (pure arithmetic);
# * the iteration-0 reading vs E (exact markers); the real bar at the carried
# * goal vs the assembled pose.
OVERRIDE_TOL_MM = 1e-6
OVERRIDE_TOL_DEG = 1e-6
READING_TOL_MM = 1e-3
READING_TOL_DEG = 1e-4
LAND_TOL_MM = 0.5
# ! Where the planned insert ends vs the goal it was given: the IK's own stop.
# ! ssik is exact (the task's 0.1 mm / 0.01 deg); the gradient backend's
# ! cartesian IK accepts every waypoint within 1 mm / 0.01 rad
# ! (motion_planner.api._run_dual_arm_cartesian_ik_loop, FrameTarget tolerances).
IK_STOP_TOL = {'ssik': (0.1, 0.01), 'gradient': (1.0, float(np.degrees(0.01)))}
PRE_MAX_ITERS = 3
# One servo run on the stub (a few hundred ticks per correction move).
MAX_TICKS = 20000
# Display-only previews, stubbed for the whole run (as transfer_servo_check does).
DISPLAY_ONLY = ('_validate_cdfm_planned_path', 'show_transfer_validation', 'show_planned_joint_values')
# What every servo record's 'bar' holds (husky_world.measure_bar_servo_feedback).
BAR_RECORD_KEYS = {'bar', 'grounded', 'source', 'fit', 'T_design', 'T_fk', 'T_mocap', 'T_corr',
                   'fk_vs_design', 'mocap_vs_design', 'mocap_vs_fk', 'usable', 'reason'}


# * ---------------------------------------------------------------------------
# * The simulated bar and Motive
# * ---------------------------------------------------------------------------

def rigid_error_about(center: np.ndarray, offset_mm: float, angle_deg: float, direction, axis) -> np.ndarray:
    """A world-frame rigid error that moves a body whose origin sits at ``center`` by ``offset_mm`` and turns it by ``angle_deg``.

    Args:
        center (np.ndarray): (3,) world point the turn is about (the bar's origin), metres.
        offset_mm (float): How far ``center`` moves.
        angle_deg (float): How far the body turns.
        direction (Sequence[float]): Direction ``center`` moves along (normalised here).
        axis (Sequence[float]): Axis of the turn, world (normalised here).

    Returns:
        np.ndarray: 4x4 ``E`` with ``E @ center = center + offset`` and a turn of ``angle_deg``.
    """
    d = np.asarray(direction, dtype=float) / np.linalg.norm(direction)
    a = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    K = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    angle = np.radians(angle_deg)
    R = np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * K @ K
    E = np.eye(4)
    E[:3, :3] = R
    E[:3, 3] = np.asarray(center, dtype=float) + d * offset_mm / 1000.0 - R @ np.asarray(center, dtype=float)
    return E


class SimulatedBar:
    """B20 as it really sits in Cindy's tools, and Motive streaming its joint markers.

    The model (the monitor) puts the bar at ``T_fk_bar`` = FK of the grasping
    tool0 on the stub base and the stub arms, composed with the grasp. The real
    bar sits at ``error @ T_fk_bar``. Its design markers at that pose are what
    Motive streams under the bar's asset id.
    """

    def __init__(self, monitor, iface, stub_obj: StubHuskyObject, grasp_link: str, grasp: np.ndarray,
                 error: np.ndarray):
        """Keep what the pose is read from; Motive streams from the start.

        Args:
            monitor (HuskyMonitor): The headless monitor (its listener and marker table).
            iface (StubInterface): The stub robot (its base and arms are the truth).
            stub_obj (StubHuskyObject): FK on the planner at a base and arms.
            grasp_link (str): The tool0 holding the bar, e.g. ``'left_ur_arm_tool0'``.
            grasp (np.ndarray): 4x4 tool0 -> bar (the grasp actually held).
            error (np.ndarray): 4x4 world-frame error E.
        """
        self.monitor = monitor
        self.iface = iface
        self.stub_obj = stub_obj
        self.grasp_link = grasp_link
        self.grasp = grasp
        self.error = error
        self.streaming = True
        self.asset_id = mocap_bar_streaming_id(BAR[len('bar_'):])
        self.design_points = monitor._mocap_bar_design(BAR).points()

    def T_model(self) -> np.ndarray:
        """Where the monitor's model puts the bar now (FK at the stub base and arms, composed with the grasp).

        Returns:
            np.ndarray: 4x4 world pose.
        """
        self.stub_obj.set_pose((self.iface.position, self.iface.rotation), self.iface.arm_joint_pose)
        return pp.tform_from_pose(self.stub_obj.get_link_pose_from_name(self.grasp_link)) @ self.grasp

    def T_real(self) -> np.ndarray:
        """Where the bar really is now.

        Returns:
            np.ndarray: 4x4 world pose ``error @ T_model``.
        """
        return self.error @ self.T_model()

    def stream(self) -> None:
        """One Motive frame of the bar's asset, handed to the monitor's own listener (y-up, under the asset id)."""
        if self.streaming:
            self.monitor.receive_labeled_marker(
                as_natnet_frame(markers_at(self.T_real(), self.design_points), model_id=self.asset_id))

    def go_silent(self) -> None:
        """Motive stops streaming the bar; what it sent before is forgotten."""
        self.streaming = False
        with self.monitor._mocap_cache_lock:
            self.monitor._mocap_labeled_marker_cache.clear()
            self.monitor._mocap_labeled_marker_stamp.clear()


# * ---------------------------------------------------------------------------
# * Harness helpers
# * ---------------------------------------------------------------------------

def dev_of(T_a: np.ndarray, T_b: np.ndarray) -> tuple:
    """Distance (mm) and angle (deg) between two 4x4 poses (``pose_deviation``).

    Args:
        T_a (np.ndarray): One pose.
        T_b (np.ndarray): The other.

    Returns:
        tuple: ``(dpos_mm, dang_deg)``.
    """
    dev = pose_deviation(T_a, T_b)
    return dev.dpos_mm, dev.dang_deg


def dev_text(dev: tuple) -> str:
    """A ``(mm, deg)`` pair as text."""
    return f"{dev[0]:.3f} mm / {dev[1]:.4f} deg"


def tform_of(frame) -> np.ndarray:
    """A compas Frame as a 4x4."""
    return pp.tform_from_pose(pose_from_frame(frame))


def pump(monitor, iface) -> Optional[str]:
    """Pump the monitor tasks to their end (``run_tasks``, confirms answered).

    Args:
        monitor (HuskyMonitor): The headless monitor.
        iface (StubInterface): The stub.

    Returns:
        str | None: None when the tasks ended; else the error and its traceback
        (returned, not raised, so the prints captured around this call are kept).
    """
    try:
        run_tasks(monitor, iface, max_ticks=MAX_TICKS)
        return None
    except Exception as e:
        monitor.tasks.clear()
        return f"{type(e).__name__}: {e}\n{traceback.format_exc()}"


def stage_transfer(monitor, iface, action, authored_position: np.ndarray, base_shift_m: float = 0.0) -> tuple:
    """Load the action fresh and stand everything where a planned transfer is about to run.

    The transfer gets an accepted two-point standstill path at the insert's
    exported start (its exec then runs the mount pause and the servo); the stub
    arms stand there; the stub base settles ``base_shift_m`` (x) off its
    authored pose.

    Args:
        monitor (HuskyMonitor): The headless monitor.
        iface (StubInterface): The stub.
        action (SkeletonAction): The B20 jointing action.
        authored_position (np.ndarray): The stub base's authored position.
        base_shift_m (float): How far the base settles off it, metres.

    Returns:
        tuple: ``(transfer index, insert index)``.
    """
    monitor.load_action(action.index)
    ti = monitor._loaded_index_of(MovementKind.DUAL_CONSTRAINED_FREE)
    ii = monitor._loaded_index_of(MovementKind.DUAL_CONSTRAINED_LINEAR)
    q = vec12_from_conf(monitor._loaded_movements[ii].start_state.robot_configuration)
    select_movement(monitor, ti)
    monitor._accept_trajectory(monitor._loaded_movements[ti], joint_trajectory_from_path([q, q]), source='Smoke')
    for i in range(iface.n_arms):
        iface.arm_joint_pose[i] = q[6 * i:6 * (i + 1)].copy()
        monitor.set_arm_trajectory((np.asarray([q[6 * i:6 * (i + 1)]] * 2), None, TRANSFER_TRAJ_TIME_S, None), i)
    monitor.trajectory_time = TRANSFER_TRAJ_TIME_S
    iface.position = authored_position + np.array([base_shift_m, 0.0, 0.0])
    return ti, ii


def stage_insert(monitor, iface, insert) -> np.ndarray:
    """Stage the insert's planned path with a short traj time and stand the stub arms at its start.

    Args:
        monitor (HuskyMonitor): The headless monitor, the insert loaded.
        iface (StubInterface): The stub.
        insert (Movement): The insert, with its planned trajectory.

    Returns:
        np.ndarray: (n, 12) the path.
    """
    path = np.asarray(path_12_from_joint_trajectory(insert.trajectory), dtype=float)
    monitor.trajectory_time = INSERT_TRAJ_TIME_S
    for i in range(iface.n_arms):
        monitor.set_arm_trajectory((path[:, 6 * i:6 * (i + 1)], None, INSERT_TRAJ_TIME_S, None), i)
        iface.arm_joint_pose[i] = path[0, 6 * i:6 * (i + 1)].copy()
    return path


def override_gaps(calls: list, records: list, n_rec0: int) -> tuple:
    """How far each bar-corrected IK target is from ``T_corr @ FK`` of the reading that set it, and from what the IK aimed.

    Args:
        calls (list): The IK calls of one run (``SimpleNamespace(override, target, n_rec)``).
        records (list): That run's servo records (``data`` handed to ``_save_servoing_data``).
        n_rec0 (int): How many servo readings were logged before the run.

    Returns:
        tuple: ``(n calls with an override, worst gap to T_corr @ FK as (mm, deg),
        worst gap between the override and the IK's target as (mm, deg))``.
    """
    n, worst, aimed = 0, (0.0, 0.0), (0.0, 0.0)
    for call in calls:
        if call.override is None:
            continue
        n += 1
        k = call.n_rec - n_rec0 - 1
        if not 0 <= k < len(records) or records[k].get('bar', {}).get('T_corr') is None:
            return n, (float('inf'), float('inf')), aimed   # no reading with a correction before it
        rec = records[k]
        T_corr = np.asarray(rec['bar']['T_corr'], dtype=float)
        for side, frame in call.override.items():
            want = T_corr @ pp.tform_from_pose(rec['tool0'][side]['observed_pose'])
            gap = dev_of(tform_of(frame), want)
            worst = (max(worst[0], gap[0]), max(worst[1], gap[1]))
            gap = dev_of(tform_of(call.target[side]), tform_of(frame))
            aimed = (max(aimed[0], gap[0]), max(aimed[1], gap[1]))
    return n, worst, aimed


def reading_text(rec: Optional[dict], key: str) -> str:
    """One deviation of a servo record's bar reading as text (``'n/a'`` without it).

    Args:
        rec (dict | None): One servo record.
        key (str): ``'mocap_vs_design'``, ``'fk_vs_design'`` or ``'mocap_vs_fk'``.

    Returns:
        str: ``'1.234 mm / 0.0450 deg'`` or ``'n/a'``.
    """
    dev = ((rec or {}).get('bar') or {}).get(key)
    return 'n/a' if dev is None else dev_text((dev['dpos_mm'], dev['dang_deg']))


def saved_json(record: Optional[dict]) -> dict:
    """The JSON one servo run wrote (the real ``_save_servoing_data``), or {}.

    Args:
        record (dict | None): One entry of the save recorder.

    Returns:
        dict: The file's content.
    """
    if record is None:
        return {}
    folder = record['root_dir']
    for dirpath, _dirs, files in os.walk(folder):
        for name in files:
            if name.endswith('.json'):
                with open(os.path.join(dirpath, name)) as f:
                    return json.load(f)
    return {}


# * ---------------------------------------------------------------------------
# * The run
# * ---------------------------------------------------------------------------

def run(results: Results, problem: str, root: str, servo_root: str) -> None:
    """The pre-insert servo, the carried insert plan, the grounded insert + its servo, the guards.

    Args:
        results (Results): Where the checks go.
        problem (str): Problem folder name.
        root (str): The scratch problem folder.
        servo_root (str): Scratch folder the servo records go to.
    """
    skeleton = load_plan_skeleton(root)
    action = next(a for a in skeleton.actions if a.action_id == ACTION_ID)
    cindy = robot_by_name(action.robot)
    base = skeleton.load_bar_action(action, prefer_sidecar=False).movements[-1].start_state.robot_base_frame
    monitor, iface, log = make_monitor(cindy, base, problem)
    add_viz_huskies(monitor, cindy)
    ready_for_compliant_exec(monitor)
    stub_obj = StubHuskyObject(monitor)
    monitor.huskies[0].object = stub_obj
    # * A real robot whose base mocap tracks; the box ticked. The class default of
    # * MOCAP_BAR_FAKE_MARKERS is the desk rehearsal's: off, Motive streams the bar.
    monitor.FAKE_HARDWARE, monitor.USE_MOCAP, monitor.USE_CELL_STATE_BASE_POSE = False, 1, 0
    monitor.MOCAP_BAR_FAKE_MARKERS = 0
    monitor._servo_bar_feedback = True
    # What __init__ sets for the MoCap bar panel and the Motive listener (the harness skips it).
    monitor.marker_set_data = []
    monitor.name_from_mocap_id = {}
    monitor._mocap_cache_lock = threading.Lock()
    monitor._mocap_labeled_marker_cache = {}
    monitor._mocap_labeled_marker_stamp = {}
    monitor.mocap_client = None
    for name in DISPLAY_ONLY:
        setattr(monitor, name, lambda *_args, **_kwargs: None)
    authored_position = np.asarray(iface.position, dtype=float).copy()

    # * --- recorders: the IK, the correction planner, the servo record, the arm moves
    ik_calls = []    # every live-base IK call: its override, its target, how many readings were logged before
    replans = []     # every correction plan: its kwargs and the path it staged
    saved = []       # every servo record: what the loop handed to _save_servoing_data
    carries = []     # every carried insert goal: the authored goal and what the plan got instead
    move_hooks = []  # run after each dual-arm command; a hook returns True when it is done
    real_ik = monitor.ik_live_base_for_selected_movement
    real_replan = monitor.replan_transfer_to_movement_start_live
    real_snapshot = monitor.labeled_markers_snapshot
    real_carry = monitor._carried_insert_goal
    real_save = husky_world._save_servoing_data
    real_send = iface.send_dual_arm_cmd
    bar = None       # the SimulatedBar, made once the action is loaded

    def n_readings() -> int:
        return len(log.since(0, 'info', '] tool0 pos L='))

    def ik_recorder() -> bool:
        override = getattr(monitor, '_servo_ee_target_override', None)
        n_rec = n_readings()
        ok = real_ik()
        ik_calls.append(SimpleNamespace(
            movement=monitor.current_movement.movement_id, ok=ok, n_rec=n_rec,
            override=None if override is None else dict(override),
            target=dict(getattr(monitor, '_last_ik_target_ee_frames', None) or {})))
        return ok

    def replan_recorder(show_validation=True, prefer_linear=False, linear_only=False) -> None:
        monitor.planned_arm_trajectory[0] = (None, None, None, None)
        real_replan(show_validation=show_validation, prefer_linear=prefer_linear, linear_only=linear_only)
        path = monitor.planned_arm_trajectory[0][0]
        replans.append(SimpleNamespace(movement=monitor.current_movement.movement_id, prefer_linear=prefer_linear,
                                       linear_only=linear_only, n_waypoints=0 if path is None else len(path)))

    def carry_recorder(mv, goal_ee: dict) -> dict:
        goal = real_carry(mv, goal_ee)
        carries.append(SimpleNamespace(movement=mv.movement_id, authored=dict(goal_ee), goal=dict(goal)))
        return goal

    def snapshot_latest(key: str) -> tuple:
        # Motive streams from its own thread: the cache always holds its latest frame.
        if key == BAR and bar is not None:
            bar.stream()
        return real_snapshot(key)

    def save_recorder(_monitor, data, planner_mode='free', root_dir='', label='', movement_id=''):
        folder = os.path.join(servo_root, f"{len(saved):02d}")
        saved.append({'data': data, 'planner_mode': planner_mode, 'label': label, 'movement_id': movement_id,
                      'asked_root_dir': root_dir, 'root_dir': folder})
        real_save(_monitor, data, planner_mode=planner_mode, root_dir=folder, label=label, movement_id=movement_id)

    def send_and_hook(multi_arm_trajectory) -> None:
        real_send(multi_arm_trajectory)
        for hook in list(move_hooks):
            if hook():
                move_hooks.remove(hook)

    monitor.ik_live_base_for_selected_movement = ik_recorder
    monitor.replan_transfer_to_movement_start_live = replan_recorder
    monitor.labeled_markers_snapshot = snapshot_latest
    monitor._carried_insert_goal = carry_recorder
    husky_world._save_servoing_data = save_recorder
    iface.send_dual_arm_cmd = send_and_hook

    n_execs = [0]    # how many execs ran: each writes a start and an end as-built record

    def as_built_records() -> list:
        """Every as-built record written so far (the scratch folder's day file(s))."""
        files = sorted(glob.glob(os.path.join(as_built_dir(husky_world.BAR_ACTION_LIVE_REPLAN_EXE_DIR), '*.jsonl')))
        return [record for path in files for record in read_as_built(path)]

    def mat(value) -> np.ndarray:
        """A recorded 4x4 as an array; NaN when missing, so every comparison with it fails."""
        return np.full((4, 4), np.nan) if value is None else np.asarray(value, dtype=float)

    def exec_and_pump() -> tuple:
        """'Exec Selected Mv Traj', then the monitor ticks to the task's end.

        Returns:
            tuple: ``(n log messages before, error or None, everything printed)``.
        """
        n_execs[0] += 1
        n_log = len(log.msgs)
        monitor.exec_selected_movement_traj()
        error, printed = call_keeping_prints(lambda: pump(monitor, iface))
        if error:
            print(f"[smoke] the run stopped: {error}")
        return n_log, error, printed

    try:
        monitor._load_skeleton_state()
        monitor.load_action(action.index)
        movements = monitor._loaded_movements
        ti = monitor._loaded_index_of(MovementKind.DUAL_CONSTRAINED_FREE)
        ii = monitor._loaded_index_of(MovementKind.DUAL_CONSTRAINED_LINEAR)
        transfer, insert, after = movements[ti], movements[ii], movements[ii + 1]
        rb = insert.start_state.rigid_body_states[BAR]
        grasp = tform_of(rb.attachment_frame)
        # * The authored bar poses: the insert's start (the transfer's left target
        # * with the grasp), the assembled pose (the insert's), the move between.
        T_start_design = tform_of(transfer.target_ee_frames['left']) @ grasp
        T_assembled = tform_of(insert.target_ee_frames['left']) @ grasp
        delta = T_assembled @ np.linalg.inv(T_start_design)
        E = rigid_error_about(T_start_design[:3, 3], SYS_ERR_MM, SYS_ERR_DEG, SYS_ERR_DIR, SYS_ERR_AXIS)
        E_touch = rigid_error_about(T_assembled[:3, 3], TOUCH_ERR_MM, TOUCH_ERR_DEG, TOUCH_ERR_DIR, TOUCH_ERR_AXIS)
        bar = SimulatedBar(monitor, iface, stub_obj, rb.attached_to_link, grasp, E)
        designed = monitor._designed_bar_frame(BAR)
        print(f"[smoke] {ACTION_ID}: {[m.movement_id for m in movements]}; {BAR} held by {rb.attached_to_link}, "
              f"ground joints {held_ground_joints(insert.start_state)}; insert move "
              f"{np.round(delta[:3, 3] * 1000, 3).tolist()} mm; Rhino pose vs the insert's end "
              f"{dev_text(dev_of(pp.tform_from_pose(designed[0]), T_assembled))}; E at the insert start "
              f"{dev_text(dev_of(E @ T_start_design, T_start_design))}")

        # * =====================================================================
        # * 1. Pre-insert servo: exec the transfer -> mount pause -> servo on the MoCap bar
        # * =====================================================================
        print(f"\n{'=' * 30} 1. PRE-INSERT SERVO (exec {transfer.movement_id}, box ticked) {'=' * 30}")
        stage_transfer(monitor, iface, action, authored_position)
        # 'Load action' parses the file again: fresh movement objects.
        transfer, insert, after = (monitor._loaded_movements[i] for i in (ti, ii, ii + 1))
        n_ik, n_rep, n_saved, n_rec0 = len(ik_calls), len(replans), len(saved), n_readings()
        n_log, error, printed = exec_and_pump()
        run_calls = [c for c in ik_calls[n_ik:] if c.movement == insert.movement_id]
        run_plans = replans[n_rep:]
        record = saved[n_saved] if len(saved) > n_saved else None
        data = record['data'] if record else []
        prompts = log.since(n_log, 'info', 'mount the bar and clamp it with the tool buttons')
        results.check(
            f"pre-insert: exec of {transfer.movement_id} (workaround {'ticked' if monitor.TRANSFER_AS_FREE_MOVE else 'unticked'}) "
            f"runs the transfer, the mount pause, then the servo on {insert.movement_id} aiming the MoCap bar",
            error is None and len(prompts) == int(bool(monitor.TRANSFER_AS_FREE_MOVE))
            and len(log.since(n_log, 'info', 'servo feedback: MoCap bar')) == 1
            and len(log.since(n_log, 'info', f"[servo] feedback: the MoCap bar {BAR}")) == 1
            and monitor.current_movement is insert,
            f"error {error}, mount prompts {len(prompts)}, loaded {monitor.current_movement.movement_id}")

        # * --- the as-built record of the transfer: a start and an end line, the end 'before_insert'
        records = as_built_records()
        ab_start, ab_end = (records[-2], records[-1]) if len(records) >= 2 else ({}, {})
        ab_bar = (ab_end.get('bars') or {}).get(BAR) or {}
        ab_fit = ab_bar.get('bar_fit') or {}
        ab_arms = (ab_end.get('robot_state') or {}).get('arms') or {}
        fit_vs_real = dev_of(mat(ab_fit.get('T_world_from_bar')), bar.T_real())
        ref_vs_design = dev_of(mat(ab_bar.get('T_reference')), T_start_design)
        results.check(
            f"as-built: the transfer wrote a start and an end record; at the end {BAR} is 'before_insert', "
            f"referenced to the insert's start (the transfer's target with the grasp) and fitted where the bar is; "
            f"the robot state holds the live joints and tool0 FK",
            [(r.get('step') or {}).get('phase') for r in (ab_start, ab_end)] == ['start', 'end']
            and all((r.get('step') or {}).get('index') == ti for r in (ab_start, ab_end))
            and ab_end['step']['outcome'] == 'ok' and ab_bar.get('milestone') == 'before_insert'
            and str(ab_bar.get('reference_kind', '')).startswith('held_authored:')
            and ref_vs_design[0] < READING_TOL_MM and ab_fit.get('ok') is True and fit_vs_real[0] < 0.01
            and (ab_end.get('robot_state') or {}).get('base_tracked') is True
            and (ab_arms.get('left') or {}).get('tool0_fk') is not None
            and np.allclose((ab_arms.get('left') or {}).get('joints', []), iface.arm_joint_pose[0]),
            f"{len(records)} records, phases {[(r.get('step') or {}).get('phase') for r in (ab_start, ab_end)]}, "
            f"milestone {ab_bar.get('milestone')}, reference {ab_bar.get('reference_kind')} "
            f"({dev_text(ref_vs_design)} from the insert start), fit vs the real bar {dev_text(fit_vs_real)}")

        bar0 = (data[0].get('bar') if data else None) or {}
        fit0 = bar0.get('fit') or {}
        read0 = bar0.get('mocap_vs_design')
        sys_dev = dev_of(E @ T_start_design, T_start_design)
        results.check(
            f"pre-insert: iteration 0 reads the simulated error (MoCap-vs-design = E = {dev_text(sys_dev)}, "
            f"FK-vs-design 0, source 'asset', 8/8 markers)",
            read0 is not None and bar0['source'] == 'asset' and fit0.get('n_matched') == 8
            and abs(read0['dpos_mm'] - sys_dev[0]) < READING_TOL_MM
            and abs(read0['dang_deg'] - sys_dev[1]) < READING_TOL_DEG
            and bar0['fk_vs_design']['dpos_mm'] < READING_TOL_MM
            and dev_of(np.asarray(bar0['T_design']), T_start_design)[0] < READING_TOL_MM,
            f"MoCap-vs-design {reading_text(data[0] if data else None, 'mocap_vs_design')}, FK-vs-design "
            f"{reading_text(data[0] if data else None, 'fk_vs_design')}, source {bar0.get('source')}, "
            f"matched {fit0.get('n_matched')}")

        converged = log.since(n_log, 'info', 'converged on the MoCap bar at iter')
        found = re.search(r'at iter (\d+)', converged[0]) if converged else None
        results.check(
            f"pre-insert: converges on the MoCap bar within {PRE_MAX_ITERS} iterations",
            found is not None and int(found.group(1)) <= PRE_MAX_ITERS,
            converged[0] if converged else f"no convergence line; last servo lines "
            f"{log.since(n_log, 'warn', '### SERVOING') + log.since(n_log, 'info', '### SERVOING stopped')}")

        n_override, worst, aimed = override_gaps(run_calls, data, n_rec0)
        n_printed = printed.count('[IK Live Base] target is BAR-CORRECTED')
        results.check(
            "pre-insert: every bar-corrected IK target is T_corr @ FK of the reading before it (both flanges), "
            "the IK aims it, and says so",
            n_override >= 2 and worst[0] < OVERRIDE_TOL_MM and worst[1] < OVERRIDE_TOL_DEG
            and aimed[0] < OVERRIDE_TOL_MM and n_printed == n_override and all(c.ok for c in run_calls),
            f"{n_override} of {len(run_calls)} IK calls bar-corrected, worst gap to T_corr @ FK {worst[0]:.1e} mm / "
            f"{worst[1]:.1e} deg, override vs IK target {aimed[0]:.1e} mm, 'BAR-CORRECTED' printed {n_printed}x")

        results.check(
            "pre-insert: every correction is a REAL bar-held straight-line plan (prefer_linear, a path staged)",
            bool(run_plans) and all(r.prefer_linear and r.n_waypoints >= 2 for r in run_plans)
            and printed.count('bar-held, LINEAR).') == len(run_plans),
            f"plans {[(r.prefer_linear, r.linear_only, r.n_waypoints) for r in run_plans]}")

        written = saved_json(record)
        results.check(
            f"pre-insert: every record carries 'bar' (usable, source 'asset', grounded, mocap_vs_fk, ...); the JSON "
            f"says servo_feedback 'mocap_bar', bar_name '{BAR}', planner_mode 'transfer+mocap_bar'",
            bool(data) and all(set(d.get('bar', {})) == BAR_RECORD_KEYS and d['bar']['usable']
                               and d['bar']['source'] == 'asset' and d['bar']['grounded']
                               and d['bar']['mocap_vs_fk'] is not None for d in data)
            and record['label'] == f'{insert.movement_id}_servoing'
            and written.get('servo_feedback') == 'mocap_bar' and written.get('bar_name') == BAR
            and written.get('planner_mode') == 'transfer+mocap_bar'
            and len(written.get('servoing_data', [])) == len(data),
            f"{len(data)} records, label {record['label'] if record else None}, JSON "
            f"{ {k: written.get(k) for k in ('servo_feedback', 'bar_name', 'planner_mode')} }")

        reference = monitor.__dict__.get('_mocap_bar_reference', {})
        results.check(
            "pre-insert: afterwards _servo_bar_carry names the insert, the override is None, "
            "_mocap_bar_reference is empty",
            monitor._servo_bar_carry == insert.movement_id and monitor._servo_ee_target_override is None
            and not reference,
            f"carry {monitor._servo_bar_carry}, override {monitor._servo_ee_target_override}, reference {list(reference)}")

        real_end = dev_of(bar.T_real(), T_start_design)
        model_end = dev_of(bar.T_model(), T_start_design)
        live = np.concatenate([np.asarray(a, dtype=float) for a in iface.arm_joint_pose])
        results.check(
            f"pre-insert: the simulated real bar ends within {SERVO_BAR_POS_TOL_MM} mm / {SERVO_BAR_ANG_TOL_DEG} deg "
            f"of the insert-start design pose (E cancelled); the insert starts at the corrected arms",
            real_end[0] < SERVO_BAR_POS_TOL_MM and real_end[1] < SERVO_BAR_ANG_TOL_DEG
            and float(np.abs(vec12_from_conf(insert.start_state.robot_configuration) - live).max()) < 1e-9,
            f"real bar {dev_text(real_end)} off (model bar {dev_text(model_end)} off: the FK error the loop accepted)")

        # * =====================================================================
        # * 2. 'Plan Movement' on the insert: the correction is carried
        # * =====================================================================
        print(f"\n{'=' * 30} 2. INSERT PLAN ({insert.movement_id}, 'Plan Movement') {'=' * 30}")
        _, printed = call_keeping_prints(monitor.plan_selected_movement)
        carried = [line.strip() for line in printed.splitlines() if 'MoCap-bar correction carried' in line]
        results.check(
            f"insert plan: 'Plan Movement' on {insert.movement_id} prints the carry line and plans",
            len(carried) == 1 and insert.trajectory is not None,
            f"{carried}, trajectory {'planned' if insert.trajectory is not None else 'NONE'}")
        if insert.trajectory is None:
            return
        path = np.asarray(path_12_from_joint_trajectory(insert.trajectory), dtype=float)

        def model_bar_at(conf12: np.ndarray) -> np.ndarray:
            state = insert.start_state.copy()
            monitor._merge_arm_values(state, cindy.all_arm_joint_names, conf12)
            return tform_of(_fk_link_frame(monitor.cfab.planner, state, rb.attached_to_link)) @ grasp

        # * The goal handed to the planner: the authored flange move, from where each flange stands.
        start_state = insert.start_state.copy()
        monitor._merge_arm_values(start_state, cindy.all_arm_joint_names, path[0])
        carry = carries[-1] if carries and carries[-1].movement == insert.movement_id else None
        goal_gap = (float('inf'), float('inf'))
        if carry is not None:
            gaps = []
            for side in cindy.side_keys:
                T_actual = tform_of(_fk_link_frame(monitor.cfab.planner, start_state, cindy.flange_for_side(side)))
                want = (tform_of(insert.target_ee_frames[side]) @ np.linalg.inv(tform_of(transfer.target_ee_frames[side]))
                        @ T_actual)
                gaps.append(dev_of(tform_of(carry.goal[side]), want))
            goal_gap = (max(g[0] for g in gaps), max(g[1] for g in gaps))
        results.check(
            f"insert plan: the goal handed to the planner is the authored insert move "
            f"({np.round(delta[:3, 3] * 1000, 2).tolist()} mm) applied to where each flange stands (both flanges, "
            f"exact)",
            goal_gap[0] < OVERRIDE_TOL_MM and goal_gap[1] < OVERRIDE_TOL_DEG,
            f"worst gap {goal_gap[0]:.1e} mm / {goal_gap[1]:.1e} deg; left goal "
            f"{dev_text(dev_of(tform_of(carry.goal['left']), tform_of(carry.authored['left'])))} from the authored end"
            if carry is not None else 'the planner was not handed a carried goal')

        T_bar_start, T_bar_end = model_bar_at(path[0]), model_bar_at(path[-1])
        move_gap = dev_of(T_bar_end, delta @ T_bar_start)
        backend = ik_backend()
        stop_mm, stop_deg = IK_STOP_TOL.get(backend, IK_STOP_TOL['ssik'])
        results.check(
            f"insert plan: the planned end bar is the authored move applied to the start bar, within the "
            f"{backend} IK's stop ({stop_mm} mm / {stop_deg:.2f} deg)",
            move_gap[0] < stop_mm and move_gap[1] < stop_deg,
            f"{len(path)} waypoints, end bar vs delta @ start bar {dev_text(move_gap)}")

        land_goal = dev_of(E @ delta @ T_bar_start, T_assembled)
        land = dev_of(E @ T_bar_end, T_assembled)
        results.check(
            f"insert plan: the simulated real bar lands within {LAND_TOL_MM} mm of the assembled pose at the carried "
            f"goal, within {LAND_TOL_MM} + {stop_mm} mm at the planned end",
            land_goal[0] < LAND_TOL_MM and land[0] < LAND_TOL_MM + stop_mm,
            f"at the carried goal {dev_text(land_goal)}, at the planned end {dev_text(land)} (planned to the "
            f"authored end instead, the real bar would land {dev_text(dev_of(E @ T_assembled, T_assembled))} off)")

        # * =====================================================================
        # * 3. Exec of the grounded insert: the insert, then the servo after it
        # * =====================================================================
        print(f"\n{'=' * 30} 3. GROUNDED INSERT + SERVO AFTER IT (exec {insert.movement_id}) {'=' * 30}")
        stage_insert(monitor, iface, insert)

        def touch_down() -> bool:
            # The insert's move: the bar's ground joints reach the floor and it shifts in the tools.
            bar.error = E_touch @ E
            return True

        move_hooks.append(touch_down)
        n_ik, n_rep, n_saved, n_rec0 = len(ik_calls), len(replans), len(saved), n_readings()
        n_calls = len(iface.calls)
        n_log, error, printed = exec_and_pump()
        run_calls = [c for c in ik_calls[n_ik:] if c.movement == after.movement_id]
        run_plans = [r for r in replans[n_rep:] if r.movement == after.movement_id]
        record = saved[n_saved] if len(saved) > n_saved else None
        data = record['data'] if record else []
        moves = [c for c in iface.calls[n_calls:] if c[0] == 'dual_arm']
        started = log.since(n_log, 'info', f"servo to the start of {after.movement_id} (the insert's end), "
                                           f"feedback: MoCap bar; straight-line corrections only.")
        results.check(
            f"grounded insert: exec of {insert.movement_id} runs the rigid insert, loads {after.movement_id}, and "
            f"the servo after it runs aiming the MoCap bar, straight lines only, load guard on",
            error is None and len(started) == 1 and monitor.current_movement is after
            and len(log.since(n_log, 'info', 'grounded bar: rigid insert done (no tighten).')) == 1
            and not log.since(n_log, 'warn', 'load guard is OFF') and len(moves) >= 1,
            f"error {error}, loaded {monitor.current_movement.movement_id}, dual-arm commands {len(moves)}")

        # * --- the as-built record of the insert: its end is 'after_insert', referenced to the assembled pose
        records = as_built_records()
        ab_end = records[-1] if records else {}
        ab_bar = (ab_end.get('bars') or {}).get(BAR) or {}
        ref_vs_assembled = dev_of(mat(ab_bar.get('T_reference')), T_assembled)
        stored_vs_rhino = dev_of(mat(ab_bar.get('T_assembled')), pp.tform_from_pose(designed[0]))
        results.check(
            f"as-built: the insert's end record has {BAR} 'after_insert', referenced to the assembled pose "
            f"(held_authored:{insert.movement_id}), its Rhino pose stored beside it",
            (ab_end.get('step') or {}).get('index') == ii and ab_end['step']['phase'] == 'end'
            and ab_bar.get('milestone') == 'after_insert'
            and ab_bar.get('reference_kind') == f'held_authored:{insert.movement_id}'
            and ref_vs_assembled[0] < READING_TOL_MM and stored_vs_rhino[0] < READING_TOL_MM,
            f"step {(ab_end.get('step') or {}).get('index')} {(ab_end.get('step') or {}).get('phase')}, milestone "
            f"{ab_bar.get('milestone')}, reference {ab_bar.get('reference_kind')} ({dev_text(ref_vs_assembled)} from "
            f"the assembled pose)")

        # ! The KEY question: ground contact vs the live-base IK and the straight line.
        ik_failed = log.since(n_log, 'warn', 'IK at live base FAILED')
        line_failed = log.since(n_log, 'warn', 'straight lines only here, so nothing is planned')
        ik_lines = [line.strip() for line in printed.splitlines()
                    if line.startswith('[IK Live Base] OK') or 'FK self-test residual' in line]
        plan_lines = [line.strip() for line in printed.splitlines() if line.startswith('[transfer plan]')]
        results.check(
            f"grounded insert (KEY): from {after.movement_id}'s start (ground joints on the floor, env collisions "
            f"on) every live-base IK succeeds",
            bool(run_calls) and all(c.ok for c in run_calls) and not ik_failed,
            f"IK calls {[c.ok for c in run_calls]}, failures {ik_failed}, last IK lines {ik_lines[-2:]}")
        results.check(
            "grounded insert (KEY): every correction is a REAL straight-line plan (linear_only) that is found "
            "with the ground joints on the floor",
            bool(run_plans) and all(r.linear_only and r.n_waypoints >= 2 for r in run_plans) and not line_failed,
            f"plans {[(r.prefer_linear, r.linear_only, r.n_waypoints) for r in run_plans]}, planner lines "
            f"{plan_lines + line_failed}")

        converged = log.since(n_log, 'info', 'converged on the MoCap bar at iter')
        stops = log.since(n_log, 'warn', '### SERVOING stopped') + log.since(n_log, 'info', '### SERVOING stopped')
        real_end = dev_of(bar.T_real(), T_assembled)
        results.check(
            f"grounded insert: the servo after it converges on the MoCap bar; the simulated real bar ends within "
            f"{SERVO_BAR_POS_TOL_MM} mm / {SERVO_BAR_ANG_TOL_DEG} deg of the assembled pose",
            len(converged) == 1 and real_end[0] < SERVO_BAR_POS_TOL_MM and real_end[1] < SERVO_BAR_ANG_TOL_DEG,
            f"{converged or stops}, real bar {dev_text(real_end)} off the assembled pose, iteration 0 read "
            f"MoCap-vs-design {reading_text(data[0] if data else None, 'mocap_vs_design')}")

        n_override, worst, aimed = override_gaps(run_calls, data, n_rec0)
        written = saved_json(record)
        live = np.concatenate([np.asarray(a, dtype=float) for a in iface.arm_joint_pose])
        reference = monitor.__dict__.get('_mocap_bar_reference', {})
        results.check(
            f"grounded insert: bar-corrected targets are T_corr @ FK; afterwards no override, no reference, nothing "
            f"staged for Exec, {after.movement_id} starts at the live arms; the JSON '{insert.movement_id}_end_servoing' "
            f"says 'mocap_bar'",
            n_override >= 1 and worst[0] < OVERRIDE_TOL_MM and worst[1] < OVERRIDE_TOL_DEG
            and monitor._servo_ee_target_override is None and not reference
            and monitor.planned_arm_trajectory[0][0] is None
            and float(np.abs(vec12_from_conf(after.start_state.robot_configuration) - live).max()) < 1e-9
            and record is not None and record['label'] == f'{insert.movement_id}_end_servoing'
            and written.get('servo_feedback') == 'mocap_bar' and all('bar' in d for d in data),
            f"{n_override} bar-corrected IK calls (worst {worst[0]:.1e} mm), label "
            f"{record['label'] if record else None}, servo_feedback {written.get('servo_feedback')}")

        # * --- the load guard: the same exec, the force changing after the first correction move
        print(f"\n{'=' * 30} 3b. LOAD GUARD (exec {insert.movement_id} again, force rises) {'=' * 30}")
        select_movement(monitor, ii)
        stage_insert(monitor, iface, insert)
        bar.error = E
        n_log0 = len(log.msgs)

        def press() -> bool:
            # The first move after the servo started: the bar pushes on the floor.
            if not log.since(n_log0, 'info', '### SERVOING iteration'):
                return False
            iface.arm_ft_sensor = [[FT_PRESS_N, 0.0, 0.0, 0.0, 0.0, 0.0] for _ in range(iface.n_arms)]
            return True

        move_hooks.extend([touch_down, press])
        n_calls = len(iface.calls)
        try:
            n_log, error, printed = exec_and_pump()
        finally:
            iface.arm_ft_sensor = [[0.0] * 6 for _ in range(iface.n_arms)]
            move_hooks.clear()
        guard = log.since(n_log, 'warn', f'since iter 0 (limit {SERVO_CONTACT_MAX_LOAD_RISE_N:.0f} N)')
        moves = [c for c in iface.calls[n_calls:] if c[0] == 'dual_arm']
        results.check(
            f"load guard: the force vector changing by {FT_PRESS_N:.0f} N after the first correction move stops the servo "
            f"after the insert at iteration 1, with the load-guard line, no convergence",
            error is None and len(guard) == 1 and 'stopped at iter 1' in guard[0]
            and not log.since(n_log, 'info', 'converged on the MoCap bar') and len(moves) == 2
            and monitor._servo_ee_target_override is None,
            f"{guard}, dual-arm commands {len(moves)} (the insert + 1 correction)")

        # * =====================================================================
        # * 4. Box unticked: tool0 mode, the MoCap bar recorded alongside
        # * =====================================================================
        print(f"\n{'=' * 30} 4. BOX UNTICKED (tool0 mode, exec {transfer.movement_id}) {'=' * 30}")
        monitor._servo_bar_feedback = False
        stage_transfer(monitor, iface, action, authored_position, base_shift_m=TOOL0_RUN_BASE_SHIFT_M)
        transfer = monitor._loaded_movements[ti]
        bar.error = E
        n_ik, n_saved = len(ik_calls), len(saved)
        n_log, error, printed = exec_and_pump()
        run_calls = ik_calls[n_ik:]
        record = saved[n_saved] if len(saved) > n_saved else None
        data = record['data'] if record else []
        written = saved_json(record)
        results.check(
            "box unticked: tool0 mode ('servo feedback: tool0', the MoCap bar 'recorded alongside'); no IK target "
            "ever bar-corrected, no 'BAR-CORRECTED' print, no carry",
            error is None and len(log.since(n_log, 'info', 'servo feedback: tool0')) == 1
            and len(log.since(n_log, 'info', f"the MoCap bar {BAR} is recorded alongside")) == 1
            and bool(run_calls) and all(c.override is None for c in run_calls)
            and '[IK Live Base] target is BAR-CORRECTED' not in printed
            and monitor._servo_bar_carry is None and monitor._servo_ee_target_override is None,
            f"error {error}, {len(run_calls)} IK calls, carry {monitor._servo_bar_carry}")
        last = data[-1] if data else None
        results.check(
            "box unticked: every record carries the MoCap comparison (usable, source 'asset'); the JSON says "
            "servo_feedback 'tool0', planner_mode 'transfer'",
            bool(data) and all(d.get('bar', {}).get('usable') and d['bar']['source'] == 'asset' for d in data)
            and written.get('servo_feedback') == 'tool0' and written.get('planner_mode') == 'transfer',
            f"{len(data)} records; at the end MoCap-vs-design {reading_text(last, 'mocap_vs_design')} (the error "
            f"the tool0 loop leaves), FK-vs-design {reading_text(last, 'fk_vs_design')}, tool0 "
            f"{[round(last['tool0'][s]['pos_norm_mm'], 3) for s in last['tool0']] if last else '-'} mm")

        # * =====================================================================
        # * 5. Box ticked, Motive silent, made-up markers: never steers
        # * =====================================================================
        print(f"\n{'=' * 30} 5. FAKE MARKERS (box ticked, Motive silent, exec {transfer.movement_id}) {'=' * 30}")
        monitor._servo_bar_feedback = True
        monitor.MOCAP_BAR_FAKE_MARKERS = 1
        bar.go_silent()
        stage_transfer(monitor, iface, action, authored_position, base_shift_m=TOOL0_RUN_BASE_SHIFT_M)
        transfer = monitor._loaded_movements[ti]
        n_ik, n_saved = len(ik_calls), len(saved)
        n_log, error, printed = exec_and_pump()
        run_calls = ik_calls[n_ik:]
        record = saved[n_saved] if len(saved) > n_saved else None
        data = record['data'] if record else []
        written = saved_json(record)
        bars = [d.get('bar') for d in data]
        not_usable = log.since(n_log, 'warn', 'the MoCap bar is not usable (fake markers')
        results.check(
            "fake markers: every reading is source 'fake', not usable, reason 'fake markers'; the loop warns that "
            "it aims tool0 because of them",
            error is None and bool(bars) and all(b is not None and b['source'] == 'fake' and not b['usable']
                                                 and 'fake markers' in b['reason'] for b in bars)
            and len(not_usable) >= 1,
            f"{len(bars)} readings, reasons {sorted({b['reason'] for b in bars if b})}, warnings {not_usable[:1]}")
        ends = (log.since(n_log, 'info', '### SERVOING converged') + log.since(n_log, 'info', '### SERVOING stopped')
                + log.since(n_log, 'warn', '### SERVOING stopped'))
        results.check(
            "fake markers: never steers (no bar-corrected IK target, no convergence on the MoCap bar, no carry); "
            "its record says it aimed tool0 (planner_mode 'transfer', servo_feedback 'tool0')",
            bool(run_calls) and all(c.override is None for c in run_calls)
            and '[IK Live Base] target is BAR-CORRECTED' not in printed
            and not log.since(n_log, 'info', 'converged on the MoCap bar')
            and monitor._servo_bar_carry is None and monitor._servo_ee_target_override is None
            and written.get('planner_mode') == 'transfer' and written.get('servo_feedback') == 'tool0',
            f"{len(run_calls)} IK calls, end {ends}, JSON "
            f"{ {k: written.get(k) for k in ('planner_mode', 'servo_feedback', 'bar_name')} }")

        # * --- the as-built file: two lines per exec, one per click, all in the scratch folder
        n_before = len(as_built_records())
        monitor.record_as_built_now()
        records = as_built_records()
        files = glob.glob(os.path.join(as_built_dir(husky_world.BAR_ACTION_LIVE_REPLAN_EXE_DIR), '*.jsonl'))
        results.check(
            "as-built: every exec wrote a start and an end record, 'As-built: record now' one more ('manual'), "
            "all in one file under the scratch folder",
            n_before == 2 * n_execs[0] and len(records) == n_before + 1
            and (records[-1].get('step') or {}).get('phase') == 'manual' and len(files) == 1
            and files[0].startswith(os.path.dirname(servo_root)),
            f"{n_before} records for {n_execs[0]} execs, then {len(records)}; files {files}")

        # * --- every servo record went to the scratch folder
        results.check(
            "records: every servo record (JSON + PNG) was written under the scratch folder",
            bool(saved) and all(r['root_dir'].startswith(servo_root) for r in saved),
            f"{len(saved)} records under {servo_root}; the loop asked for "
            f"{sorted({r['asked_root_dir'] for r in saved})}")
    finally:
        husky_world._save_servoing_data = real_save
        monitor.tasks.clear()
        monitor.cfab.close()
        # The bar copies' OBJ files (the live monitor keeps them until the machine restarts).
        if getattr(monitor, '_mocap_bar_mesh_dir', None):
            shutil.rmtree(monitor._mocap_bar_mesh_dir, ignore_errors=True)


def main(argv: list = None) -> int:
    """Build the scratch problem, run the checks, print the summary.

    Args:
        argv (list | None): Command line (None = ``sys.argv[1:]``).

    Returns:
        int: 0 when every check passed, else 1.
    """
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--keep', action='store_true',
                        help="keep the temp dir (servo JSON + PNG, progress.json) for inspection")
    args = parser.parse_args(argv)

    problem = DESIGN_PROBLEM_NAME
    real_root = problem_root(problem, DESIGN_DATA_DIRECTORY)
    skeleton = load_plan_skeleton(real_root)
    if (not os.path.isfile(os.path.join(real_root, MOCAP_MARKERS_FILENAME)) or skeleton is None
            or not any(a.action_id == ACTION_ID for a in skeleton.actions)):
        print(f"{problem} has no {MOCAP_MARKERS_FILENAME} or no {ACTION_ID}; this smoke needs 261016_2ModulesDemo_YH.")
        return 1
    before = folder_snapshot(real_root)
    scratch_design = tempfile.mkdtemp(prefix='husky_bar_servo_smoke_')
    print(f"[smoke] real problem folder (read only): {real_root}")
    print(f"[smoke] scratch copy: {os.path.join(scratch_design, problem)}")
    results = Results()
    try:
        root = make_scratch_problem(real_root, scratch_design, problem)
        shutil.copy(os.path.join(real_root, MOCAP_MARKERS_FILENAME), os.path.join(root, MOCAP_MARKERS_FILENAME))
        point_package_at(scratch_design)
        # ! Servo records go to the scratch folder, never into the experiment data.
        servo_root = os.path.join(scratch_design, 'servo_records')
        husky_world.BAR_ACTION_LIVE_REPLAN_EXE_DIR = os.path.join(scratch_design, 'bar_action_live_replan_exe')
        husky_world.BAR_HOLDING_ACC_EXPERIMENT_DIR = os.path.join(scratch_design, 'bar_holding_acc_data')
        try:
            run(results, problem, root, servo_root)
        except Exception as e:  # keep going: the summary shows where it stopped
            traceback.print_exc()
            results.check('the smoke ran to the end', False, f"{type(e).__name__}: {e}")
        results.check('real design folder untouched', folder_snapshot(real_root) == before)
    finally:
        if args.keep:
            print(f"[smoke] kept the scratch copy: {scratch_design}")
        else:
            shutil.rmtree(scratch_design, ignore_errors=True)
            print("[smoke] removed the scratch copy (use --keep to inspect it)")

    results.print_summary()
    return 1 if results.n_failed() else 0


if __name__ == '__main__':
    sys.exit(main())
