"""Open-loop dual-arm execution engine for precomputed assembly trajectories.

* Loads one "assembly-open-loop-json-v2" trajectory (see open_loop_traj.py)
* and either previews it in PyBullet or executes it on both UR5e arms of
* Cindy (/a200_0806) over ur_rtde -- bypassing the ROS scaled joint
* trajectory controller so both arms follow one shared clock with explicit
* speed/acceleration control (lockstep open-loop execution).
*
* The tracking law is a Python port of the ReferencePath mode of Valentin's
* controller (robot_ipc_control/controller/impedance_controller.cpp): at a
* fixed frequency, per arm,
*     qd_cmd = clamp(qd_ref(t) + p_gain * (q_ref(t) - q_actual), +-vmax)
*     speedJ(qd_cmd, joint_accel, 1/frequency)
* with the reference splines evaluated on time since a single shared start
* instant -- that shared clock IS the dual-arm synchronization.

Preview mode (default):
    ros2 run husky_assembly_teleop open_loop_engine <traj.json>
    Play/pause + speed + a scrubbable time slider drive the PyBullet robot;
    a live 12-joint readout plot and gripper/servo-flag indicators follow.

Execute mode:
    ros2 run husky_assembly_teleop open_loop_engine <traj.json> --execute
    Button flow: Connect RTDE -> Check start pose -> Move to start (slow,
    sequential) -> START TRACKING. Space (or the PAUSE button) ramps both arms
    to a hold over 0.5 s and back, without leaving the plan. STOP (or closing
    the window, or Ctrl+C) always speedStops both arms. Gripper open/close
    annotations fire through the existing ROS2 GripperCommand action path
    (--no-gripper logs only).

! Ops prerequisites for --execute (operator, on the husky, before running):
!   1. stop the UR arm drivers AND multi_arm_safety_sync -- that node keeps
!      re-loading ros_control.urp, which kills the script ur_rtde uploads;
!   2. the Robotiq bridge dies with the arm driver, so relaunch the
!      gripper-only stack: crl_gripper.launch.py with
!      start_tool_communication:=true robot_ip:=192.168.131.40 (and .41);
!   3. this laptop must reach the arms on the 192.168.131.x network.
"""

import argparse
import json
import os
import sys
import threading
import time
from datetime import datetime

import numpy as np
import pybullet as p
import pybullet_planning as pp
import rclpy
# Agg-only figure (no pyplot): safe to render headless from inside the node.
from matplotlib.figure import Figure
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from scipy.spatial.transform import Rotation

from husky_assembly_teleop import DATA_DIRECTORY
from husky_assembly_teleop import common as _common
from husky_assembly_teleop.common import (Button, HistoryPlot, LiveMultiPlot,
                                          Separator, Slider, Toggle, load_robot,
                                          HUSKY_DUAL_UR5e_JOINT_NAMES)
from husky_assembly_teleop.husky_robot import HuskyRobotInterface
from husky_assembly_teleop.open_loop_approach import (JOINT_NAMES_12,
                                                      OBSTACLE_LABELS,
                                                      TABLE_TOP_Z,
                                                      approach_traj_from_path,
                                                      build_obstacles,
                                                      plan_approach)
from husky_assembly_teleop.open_loop_parts import (COLOR_ASSEMBLED, COLOR_HELD,
                                                   COLOR_TABLE, PartTracker,
                                                   fit_jaws, load_part_bodies)
from husky_assembly_teleop.open_loop_insertion import (
    PHASES as INSERTION_PHASES, InsertionController, InsertionParams)
from husky_assembly_teleop.open_loop_traj import (ARM_SIDES, OpenLoopTraj,
                                                  TrajClock, find_insertions,
                                                  load_open_loop_traj,
                                                  rebase_to_branch)
from husky_assembly_teleop.ui_backend import make_backend
from husky_assembly_teleop.utils import (ROBOTIQ_COUPLING_M,
                                         TOOL0_FROM_GRIPPER_TCP)

# ? ur_rtde is only needed for --execute; preview must work without it.
try:
    from rtde_control import RTDEControlInterface
    from rtde_receive import RTDEReceiveInterface
except ImportError:
    RTDEControlInterface = RTDEReceiveInterface = None

# --- --- CONSTANTS --- ---

TICK_PERIOD_S = 0.05        # 20 Hz UI tick == the trajectory's sample spacing
JOINT_LABELS_12 = [f'{side} {j}' for side in ('L', 'R')
                   for j in ('pan', 'lift', 'elbow', 'w1', 'w2', 'w3')]
# Wrench plots: x/y/z per arm, reds for the left arm and greens for the right,
# the same convention grasp_calib_monitor uses.
AXIS_LABELS = ['x', 'y', 'z']
WRENCH_PALETTE = [(240, 130, 120), (225, 70, 55), (150, 20, 20),
                  (150, 220, 140), (70, 180, 80), (20, 130, 40)]

# Gripper commands, as knuckle-joint angles in radians (0 = fully open,
# 0.803 = fully closed). The close and the effort are the monitor's own values
# (husky_world.close_gripper_for_bar).
# ! OPEN means FULLY open here (2026-09-10), not the monitor's 0.426 rad
# ! "pre-open" (husky_world.open_gripper_full, ~41 mm between the pads). The
# ! planner certifies every pick and every release with the jaws all the way
# ! out -- 83 mm on the model, 85 mm stroke on the real 2F-85 -- and a half-
# ! open jaw would approach a part the planner never checked it against. The
# ! same constant is the view's open angle and the start of the jaw-fit sweep,
# ! so all three stay in step. The only cost: a full open takes about twice as
# ! long to finish, and opening is a one-sample command that the arm does not
# ! wait for.
GRIPPER_OPEN, GRIPPER_CLOSE, GRIPPER_EFFORT = 0.0, 0.8, 0.1
# Full stroke of the 2F-85, to turn a reported knuckle angle into a jaw opening
# for the log. The linkage is not quite linear, but the driver maps the angle
# linearly onto the 0-255 position register and Robotiq quotes ~0.4 mm per
# count, so this is good to a couple of millimetres -- enough to tell a 22 mm
# leg from a 35 mm seat plate in the record, not for anything tighter.
GRIPPER_STROKE_MM = 85.0
# ! Where the fingers stop when they meet NOTHING: measured 0.7894 rad on both
# ! of Cindy's grippers (2026-09-09). The 0.8 rad close target sits just past
# ! it -- outside the controller's 0.01 rad goal tolerance -- so an empty close
# ! is reported as `stalled`, exactly like a grasp. The flags alone cannot tell
# ! them apart; the final position can, and this is the line between them.
GRIPPER_EMPTY_ANGLE = 0.77


def jaw_width_mm(knuckle_angle: float) -> float:
    """Approximate jaw opening for a knuckle angle.

    Args:
        knuckle_angle (float): The gripper joint angle [rad], 0 = open.

    Returns:
        float: Opening between the pads [mm], 0 when fully closed.
    """
    return GRIPPER_STROKE_MM * max(0.0, 1.0 - knuckle_angle / GRIPPER_CLOSE)


def grasp_verdict(kind: str, stalled: bool, reached_goal: bool,
                  position: float = None) -> str:
    """What a GripperCommand result says happened to the part.

    The Robotiq stops its fingers the moment they meet something and the
    controller reports that as `stalled`. But fingers that meet nothing also
    stop -- at their own mechanical limit, which the 0.8 rad target overshoots
    by more than the goal tolerance, so that too comes back `stalled`. A close
    is therefore judged by WHERE the fingers ended: past GRIPPER_EMPTY_ANGLE
    they closed on air, short of it they closed on a part.

    Args:
        kind (str): 'close' or 'open', what was commanded.
        stalled (bool): The result's stalled flag.
        reached_goal (bool): The result's reached_goal flag.
        position (float): The result's final knuckle angle [rad]; None falls
            back to the flags alone (an empty close then reads as GRASPED).

    Returns:
        str: 'GRASPED' / 'MISSED' for a close, 'OPENED' / 'BLOCKED' for an
        open, 'UNKNOWN' when the result says neither (cancelled, error).
    """
    if kind == 'close':
        if position is not None and position >= GRIPPER_EMPTY_ANGLE:
            return 'MISSED'
        return 'GRASPED' if stalled else 'MISSED' if reached_goal else 'UNKNOWN'
    return 'OPENED' if reached_goal else 'BLOCKED' if stalled else 'UNKNOWN'


# * Articulated Robotiq model for the 3D view, attached to each tool0 flange.
# * PyBullet ignores URDF <mimic> tags, so every finger joint is written
# * explicitly as factor * knuckle angle (factors from the URDF mimic tags).
GRIPPER_URDF = os.path.join(
    DATA_DIRECTORY, 'husky_urdf/robotiq_85/urdf/robotiq_85_gripper_simple.urdf')
GRIPPER_VIZ_JOINTS = [
    'robotiq_85_left_knuckle_joint', 'robotiq_85_right_knuckle_joint',
    'robotiq_85_left_inner_knuckle_joint', 'robotiq_85_right_inner_knuckle_joint',
    'robotiq_85_left_finger_tip_joint', 'robotiq_85_right_finger_tip_joint']
GRIPPER_VIZ_FACTORS = [1.0, 1.0, 1.0, 1.0, -1.0, -1.0]
# tool0 -> gripper base mount, read right to left: pitch -90 deg to swing the
# URDF's own +x (which the gripper extends along) onto tool0's +z, then out
# along tool0 z by the coupling's thickness, then the quarter turn.
# ! Both of those are the REAL mount, and both were wrong before 2026-09-10.
# ! The grippers are bolted a quarter turn about the flange axis, so the pads
# ! close along tool0 X, not tool0 Y; and they sit on a coupling, not on the
# ! flange (utils.ROBOTIQ_COUPLING_M carries the manual's derivation). The
# ! planner has the same two in src/rai/husky/calibrated/husky_calibrated.g;
# ! if either is ever changed, change it in both places.
GRIPPER_MOUNT_POSE = pp.multiply(
    pp.Pose(euler=pp.Euler(yaw=np.pi / 2)),
    pp.Pose(point=(0.0, 0.0, ROBOTIQ_COUPLING_M)),
    pp.Pose(euler=pp.Euler(pitch=-np.pi / 2)))

# Slow, supervised approach to the trajectory's first sample.
MOVE_TO_START_SPEED = 0.3   # rad/s
MOVE_TO_START_ACCEL = 0.5   # rad/s^2
# Beyond this per-joint distance the operator jogs by pendant instead --
# a blind moveJ across a large distance could sweep through the other arm.
MAX_MOVE_TO_START_DELTA = 1.5   # rad
# ? How much room a joint must keep to its limit after the trajectory is
# ? rebased onto a wrapped branch. The wrapped branch is legal, but it can run
# ? a joint right up against its end stop where the authored branch had the
# ? whole range -- and there tracking error alone can trip a protective stop.
BRANCH_LIMIT_MARGIN = 0.10  # rad

# ! t0 is placed this far in the future when START is pressed: both tracker
# ! threads' first cycles see t < 0, whose clamped reference (start pose,
# ! zero velocity) makes them HOLD position until the shared clock reaches 0.
# ! Thread startup jitter therefore never desynchronizes the arms.
START_PREROLL_S = 0.5
END_SETTLE_S = 1.0          # keep holding the final pose this long past the end
# ! A cutoff before the end of the file usually lands MID-MOTION. The
# ! reference decelerates over this long instead of jumping to a standstill,
# ! so the arm brakes along its own path rather than overshooting it.
BRAKE_TIME_S = 0.4
# * Execution speed: how many trajectory seconds the tracker plays per wall
# * second. The slider spans MIN..MAX, but the usable maximum is whatever keeps
# * the scaled reference under --max-joint-vel (a clipped feed-forward would
# * distort the path, not merely slow it), so it is capped again at connect.
MIN_SPEED_SCALE, MAX_SPEED_SCALE = 0.1, 2.0
# * Pause (Space bar / the PAUSE button): the clock's rate walks linearly down
# * to zero over this long instead of dropping there at once, so the arms
# * decelerate along their own path rather than being asked to stop abruptly.
# * Resuming walks it back up over the same time.
# ! Do not shorten it: the deceleration it asks for is |qd_ref| / PAUSE_RAMP_S,
# ! which at the top of the speed range is already close to --joint-accel.
PAUSE_RAMP_S = 0.5
# * Insertion wrench log: how much quiet trajectory time to keep on either side
# * of a mate's window, so the plot shows the force baseline before the parts
# * touch and after the gripper lets go. The window itself runs from the funnel
# * mouth to the release; both are recorded, so the pad can be trimmed later.
WRENCH_LOG_PAD_S = 1.0
# * How long to let the force/torque reading settle after asking the controller
# * to zero it. `zeroFtSensor` subtracts the CURRENT measurement from every
# * later one, so the arm must be still while it happens and the read-back has
# * to wait a beat for the new bias to come through on the RTDE stream.
FT_ZERO_SETTLE_S = 0.5
# States in which the robot is connected but standing still, so a button that
# starts something new is allowed to act.
IDLE_STATES = ('connected', 'ready', 'done', 'aborted')
# States in which threads are driving the arms, so STOP has something to stop.
# 'held' is deliberately NOT here: there the arms are already parked and the
# operator's own buttons decide what happens next.
RUNNING_STATES = ('tracking', 'inserting', 'releasing')


def load_viz_grippers(robot: int, tool0_links: list) -> list:
    """Load one articulated Robotiq per arm and pin it to that arm's flange.

    The URDF's mesh references resolve through the search path the caller has
    already set (`DATA_DIRECTORY/husky_urdf`).

    ! `pp.create_attachment` freezes the CURRENT parent->child transform, so
    ! each gripper is posed on its flange BEFORE it is attached; attaching
    ! first would pin it at the world origin.

    Args:
        robot (int): The robot body id.
        tool0_links (list): Per arm [left, right], the tool0 link index.

    Returns:
        list: Per arm, (gripper body id, attachment, driven joint indices).
    """
    grippers = []
    for tool0_link in tool0_links:
        with pp.LockRenderer(), pp.HideOutput():
            grip_body = pp.load_pybullet(GRIPPER_URDF, fixed_base=False)
        pp.set_pose(grip_body,
                    pp.multiply(pp.get_link_pose(robot, tool0_link),
                                GRIPPER_MOUNT_POSE))
        grip_att = pp.create_attachment(robot, tool0_link, grip_body)
        grip_joints = pp.joints_from_names(grip_body, GRIPPER_VIZ_JOINTS)
        grippers.append((grip_body, grip_att, grip_joints))
    return grippers


class OpenLoopEngine(Node):
    """Previewer + RTDE executor node for one open-loop dual-arm trajectory."""

    def __init__(self, args):
        super().__init__('open_loop_engine')
        self.args = args
        self.mode = 'execute' if args.execute else 'preview'

        # Fail fast on a bad file, and give the operator the event list.
        self.traj = load_open_loop_traj(args.traj_json, swap_arms=args.swap_arms)
        self.get_logger().info('loaded trajectory:\n' + self.traj.summary())

        # * Gripper path: the lean HuskyRobotInterface recipe (same as
        # * grasp_calib_monitor) -- subscriptions + publishers + the two
        # * GripperCommand action clients, none of the slow service waits.
        # * connect_gripper=True adds a 2.5 s/arm "is the action server up"
        # * check, which is exactly the preflight we want against the
        # * gripper-only stack. Preview and --no-gripper skip ROS entirely.
        self.robot = None
        if self.mode == 'execute' and not args.no_gripper:
            # ! Read via getattr in the interface ctor -- must be set first.
            self.CONNECT_IO_SERVICES = 0
            self.LIST_CONTROLLER_SERVICES = 0
            self.robot = HuskyRobotInterface(
                self, name=args.robot_name, use_odom=False, connect_arm=False,
                connect_gripper=True, dual_arm=True)

        # * PyBullet 3D viewer: calibrated dual-arm husky model (same boot as
        # * grasp_calib_monitor / husky_monitor.start_pybullet).
        pp.connect(use_gui=True, shadows=True, color=[0.9, 0.9, 1.0])
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=pp.CLIENT)
        pp.draw_pose(pp.unit_pose(), 1)
        with pp.LockRenderer(), pp.HideOutput():
            self.viz_robot = load_robot(dual_arm=True)
        self._viz_joints = pp.joints_from_names(
            self.viz_robot,
            HUSKY_DUAL_UR5e_JOINT_NAMES[0] + HUSKY_DUAL_UR5e_JOINT_NAMES[1])

        # * Articulated Robotiq models on both flanges, so gripper open/close
        # * is visible in the 3D view. The search path resolves the URDF's
        # * package://robotiq_85/... mesh references.
        p.setAdditionalSearchPath(os.path.join(DATA_DIRECTORY, 'husky_urdf'),
                                  physicsClientId=pp.CLIENT)
        self._tool0_links = [pp.link_from_name(self.viz_robot, name)
                             for name in ('left_ur_arm_tool0', 'right_ur_arm_tool0')]
        self.viz_grippers = load_viz_grippers(self.viz_robot, self._tool0_links)
        # Displayed knuckle angle per arm; both grippers assumed open at start.
        self.grip_viz_angle = [GRIPPER_OPEN, GRIPPER_OPEN]

        # * Static collision geometry: the floor and the placeholder work
        # * table. These are the very bodies the approach planner checks
        # * against, so what is on screen is what is avoided.
        # * A measured layout replaces the guessed table and adds the parts, so the
        # * approach is planned against the real cell (pickup_calib writes the file).
        layout = None
        if args.layout_json:
            from husky_assembly_teleop.pickup_calib import load_layout
            layout = load_layout(args.layout_json)
        self.obstacles = build_obstacles(args.table_top_z, not args.no_table,
                                         layout=layout)

        # * The parts themselves, drawn from their real meshes and followed
        # * through the trajectory: on the table, then in a gripper, then
        # * mated into the assembly. Visual only -- the planner keeps checking
        # * against the boxes build_obstacles made.
        self.parts = None
        self.part_bodies = {}
        self._jaw_fit = {}       # (arm, part, episode start) -> that grasp's fit
        self._viz_sample = 0
        self._part_roles = {}
        if layout is not None:
            self.part_bodies = load_part_bodies(layout)
            self.parts = PartTracker(self.traj, layout, self._tool0_pose_at,
                                     plan_json=args.plan_json,
                                     log=self.get_logger().warn,
                                     tcp_offset=TOOL0_FROM_GRIPPER_TCP)
            mates = (f'from {os.path.basename(args.plan_json)}'
                     if args.plan_json else 'UNKNOWN -- pass --plan-json')
            self.get_logger().info(
                f'part visualization ({len(self.part_bodies)} parts, mates '
                f'{mates}):\n{self.parts.summary()}')
            self._fit_every_grasp()
        else:
            self.get_logger().info(
                'no --layout-json: the parts are not drawn (the robot is)')

        # ! The backend must exist before any widget is created.
        _common._global_backend = make_backend(
            use_dpg=True,
            window_title=f'Open-Loop Engine [{self.mode}] - '
                         f'{os.path.basename(args.traj_json)}',
            width=1750, height=960, font_size=18)

        # --- shared display state (read by the polled joint plot) ---
        self._plot_q12 = self.traj.q12[0].copy()

        # --- preview state ---
        self.playing = False
        self.play_t = 0.0
        self._last_wall = time.monotonic()
        self._slider_written = 0.0   # last value THIS code wrote to the slider

        # --- execute state ---
        # state machine: disconnected -> connected -> ready -> tracking ->
        # done | aborted, with 'moving' and 'previewing' as transient busy
        # states. Buttons ignore presses in the wrong state and say why in the
        # log.
        self.state = 'disconnected'
        self.rtde_c = [None, None]
        self.rtde_r = [None, None]
        self.live_q = [self.traj.q12[0, :6].copy(), self.traj.q12[0, 6:].copy()]
        self.live_err = [np.zeros(6), np.zeros(6)]
        self.start_deltas = None          # per-arm |q_now - q_start|, from Check
        self.stop_evt = None
        # * The one clock both tracker threads read. Its rate is the execution
        # * speed slider; see TrajClock for why changing it mid-run is safe.
        self.clock = TrajClock()
        self.scale_slider = None          # execute mode only
        self.speed_scale = 1.0            # what the slider last asked for
        self.scale_cap = MAX_SPEED_SCALE  # tightened at connect by the vel check
        self.scale_changes = []           # (trajectory time, scale) per run
        # * Pause: what the operator last asked for. The ramp itself lives in
        # * the shared clock (TrajClock.ramp_to), so the arms feel it at their
        # * own 125 Hz however slowly this UI happens to be ticking. A paused
        # * run is still under speedJ -- STOP is what releases the arms.
        self.paused = False
        # Why the run stopped BY ITSELF (the marking stop), shown in the status
        # line until it resumes; None for an operator's own pause.
        self.stop_reason = None
        self.stop_toggle = None           # execute mode only
        self.pauses = []                  # {'paused_at', 'resumed_at', 'reason'} per run
        self.threads = []
        self.thread_done = [False, False]
        self.thread_error = [None, None]
        self.exec_log = None
        self.next_event_idx = 0
        self.fired_events = []
        self._log_saved = True            # nothing to save until a run starts
        self.last_run_folder = None
        # * Cutoff: run only the first N samples (1-based, as the readouts
        # * count). Set from the slider at START; the events past it never fire.
        self.end_idx = min(args.end_sample or self.traj.n_samples,
                           self.traj.n_samples)
        self.t_end = float(self.traj.times[self.end_idx - 1])
        self.run_events = list(self.traj.events)
        # * Approach motion: planned on demand, then executed by the very same
        # * tracker threads, so it inherits their safety net and logging.
        # * active_traj is what the trackers follow -- the file's trajectory
        # * normally, the planned approach while that is running.
        self.active_traj = self.traj
        self.run_label = 'trajectory'
        self.approach_traj = None
        self.preview_t = 0.0

        # * Compliant insertion. Off unless --insertions: with the flag clear
        # * `self.insertions` stays empty, `on_start` builds a single tracking
        # * phase, and every code path below behaves exactly as it did before.
        self.insertions = []
        self.ins_params = self._build_insertion_params()
        self.live_wrench = [np.zeros(6), np.zeros(6)]
        self.ft_zeros = []                # one record per zeroing, for the log
        self.live_role = ['', '']
        self.hold_wrench = [None, None]   # wrench when a hold started
        self.resume_offset = np.zeros(12)
        self.resume_from = 0.0
        self.phases = []
        self.phase_idx = 0
        self.insertion = None             # the running InsertionController
        self.insertion_msg = 'insertions: off'
        self.insertion_records = []
        self.insertion_logs = {}
        # * Where the plan's mates are in the file. Located once, on demand:
        # * --insertions needs them at startup to build its phases, the wrench
        # * log only at the end of a run. See _mates.
        self._mates_cache = None
        self.wrench_toggle = None         # execute mode only
        if args.insertions:
            if not args.plan_json:
                self.get_logger().error(
                    '--insertions needs --plan-json to know which gripper-open '
                    'is a mate -- running as plain tracking instead')
            else:
                self.insertions = self._mates()
                self.insertion_msg = (f'{len(self.insertions)} insertion(s) '
                                      f'will run compliant')
                self.get_logger().info(
                    'compliant insertions:\n'
                    + '\n'.join(f'  {ins.describe()}' for ins in self.insertions))
        # Written by the planning worker, rendered by the UI thread, so no
        # widget is ever touched from a background thread.
        self.approach_msg = 'approach: not planned'

        self.build_ui()
        self.tick_timer = self.create_timer(TICK_PERIOD_S, self.update)

    # --- --- UI --- ---

    def build_ui(self):
        """Create the control panel plus the joint / error plot windows."""
        self.widgets = []
        backend = _common._global_backend

        info = (f'{self.traj.n_samples} samples, {self.traj.duration:.1f}s, '
                f'dt={self.traj.dt}s, {len(self.traj.events)} gripper events, '
                f'swap_arms={self.traj.swap_arms}')

        if self.mode == 'preview':
            self.widgets.append(Separator('Open-loop trajectory preview'))
            self.widgets.append(Separator(info))
            self.widgets.append(Button('Play / Pause', self.on_play_pause))
            self.speed_slider = Slider('speed x', lambda *_: None, 0.1, 4.0, 1.0)
            self.widgets.append(self.speed_slider)
            self.time_slider = Slider('time [s]', lambda *_: None,
                                      0.0, self.traj.duration, 0.0)
            self.widgets.append(self.time_slider)
            self.readout_sep = Separator('sample 1')
            self.widgets.append(self.readout_sep)
            self.grip_sep = Separator('grippers:')
            self.widgets.append(self.grip_sep)
        else:
            self.widgets.append(Separator('Open-loop RTDE execution - '
                                          + self.args.robot_name))
            self.widgets.append(Separator(info))
            self.widgets.append(Separator(
                f'L={self.args.left_ip} R={self.args.right_ip} '
                f'{self.args.frequency:.0f}Hz p_gain={self.args.p_gain} '
                f'gripper={"OFF (log only)" if self.args.no_gripper else "ROS"}'
                f' [ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "<unset>")}, '
                f'RMW={os.environ.get("RMW_IMPLEMENTATION", "<default>")}]'))
            self.widgets.append(Button('Connect RTDE', self.on_connect))
            # * Zeroing happens automatically at connect; this is for doing it
            # * again after something changed -- a tool swapped, a cable moved,
            # * or a drift noticed in the Wrench window.
            self.widgets.append(Button('Zero force sensors (both arms)',
                                       self.on_zero_ft))
            self.widgets.append(Button('Check start pose', self.on_check_start_pose))
            self.widgets.append(Button('Plan approach to start',
                                       self.on_plan_approach))
            self.widgets.append(Button('Preview approach',
                                       self.on_preview_approach))
            self.widgets.append(Button('Execute approach',
                                       self.on_execute_approach))
            self.approach_sep = Separator(self.approach_msg)
            self.widgets.append(self.approach_sep)
            # Kept as an escape hatch: a straight joint-space move with NO
            # collision checking, for when the planner cannot find a way out.
            self.widgets.append(Button('Move to start (BLIND moveJ)',
                                       self.on_move_to_start))
            # * How much of the trajectory to run. Read live at START -- a
            # * slider's on-change callback can be missed, but the widget
            # * always holds the real position (same idiom as Slider.value).
            self.end_slider = Slider('run until sample', lambda *_: None,
                                     1, self.traj.n_samples, self.end_idx,
                                     integer=True)
            self.widgets.append(self.end_slider)
            # * Execution speed. Read live at every tick (a slider's on-change
            # * callback can be missed) and applied to the running phase, so it
            # * can be dragged mid-run. Insertions ignore it -- see _tick_speed.
            self.scale_slider = Slider('execution speed x', lambda *_: None,
                                       MIN_SPEED_SCALE, MAX_SPEED_SCALE, 1.0)
            self.widgets.append(self.scale_slider)
            self.cutoff_sep = Separator('')
            self.widgets.append(self.cutoff_sep)
            # * Marking stop: freeze right after each part's FIRST pick so its
            # * outline can be traced on the sheet where the robot really
            # * grasped it. Read live at every close (Toggle.value), so it can
            # * be flipped mid-run; the CLI flag only sets the initial state.
            self.stop_toggle = Toggle(
                'stop after each first pick (trace the outline)',
                lambda *_: None, bool(getattr(self.args, 'stop_after_grasp', False)))
            self.widgets.append(self.stop_toggle)
            # * Force record around every mate: both arms' wrench from the
            # * funnel mouth to the release, written as json + png at the end
            # * of the run. Read once, when the run is saved.
            self.wrench_toggle = Toggle(
                'log insertion wrench (json + png)', lambda *_: None,
                not getattr(self.args, 'no_wrench_log', False))
            self.widgets.append(self.wrench_toggle)
            self.widgets.append(Button('START TRACKING', self.on_start))
            # * Pause: ramps both arms to a hold over PAUSE_RAMP_S and back.
            # * The Space shortcut below does the same thing, from either
            # * window. Note it does NOT release the arms -- STOP does.
            self.widgets.append(Button('PAUSE / RESUME  (Space)',
                                       self.on_pause_toggle))
            # * DONE is the same resume as Space. It exists so that, once the
            # * outline is traced, there is one obvious thing to press.
            self.widgets.append(Button('DONE -- outline traced, continue  (Space)',
                                       self.on_done))
            self.widgets.append(Button('STOP', self.on_stop))
            self.delta_sep = Separator('start pose: not checked')
            self.widgets.append(self.delta_sep)
            self.event_sep = Separator('gripper events: none fired')
            self.widgets.append(self.event_sep)
            self._build_insertion_ui()
            # Space anywhere in the control panel pauses/resumes. The 3D
            # window has its own keyboard, polled in _tick_execute.
            backend.add_key_handler('space', self.on_pause_toggle)

        self.status_sep = Separator(f'status: {self.mode}')
        self.widgets.append(self.status_sep)

        # * Joint window (both modes): the 12 current joint values as a
        # * scrolling plot + rad/deg readout table, left | right columns.
        backend.add_window('Joint values', tag='ol_q_window',
                           width=590, height=930, pos=(560, 10))
        self.q_plot = LiveMultiPlot(
            'joint values [rad]',
            lambda: [float(v) for v in self._plot_q12],
            JOINT_LABELS_12, history=600, parent='ol_q_window', group_size=6)

        # * Tracking-error window (execute only): one plot per arm with a
        # * shared axis scale so the arms stay directly comparable.
        if self.mode == 'execute':
            backend.add_window('Tracking error', tag='ol_err_window',
                               width=580, height=930, pos=(1160, 10))
            self.err_plots = [
                HistoryPlot(f'{side} tracking error', JOINT_LABELS_12[6 * i:6 * i + 6],
                            'q_ref - q_actual [rad]', parent='ol_err_window',
                            history=8192, link_group='ol_err')
                for i, side in enumerate(ARM_SIDES)]

            # * Wrench window: both arms' force and torque, always streaming.
            # * Force and torque are split because N and Nm cannot share a y
            # * axis, and each pair is linked across the arms so the two are
            # * directly comparable. Which arm is inserting and which is
            # * holding is written into the role lines in the control panel,
            # * whose text names the same side as the plot titles here.
            backend.add_window('Wrench', tag='ol_wrench_window',
                               width=600, height=930, pos=(1745, 10))
            self.force_plots, self.torque_plots = [], []
            for i, side in enumerate(ARM_SIDES):
                self.force_plots.append(HistoryPlot(
                    f'{side} TCP force (base frame)', AXIS_LABELS, 'force [N]',
                    parent='ol_wrench_window', history=8192,
                    palette=WRENCH_PALETTE[3 * i:3 * i + 3],
                    link_group='ol_force'))
                self.torque_plots.append(HistoryPlot(
                    f'{side} TCP torque (base frame)', AXIS_LABELS,
                    'torque [Nm]', parent='ol_wrench_window', history=8192,
                    palette=WRENCH_PALETTE[3 * i:3 * i + 3],
                    link_group='ol_torque'))

    # --- --- 20 Hz TICK --- ---

    def _build_insertion_ui(self):
        """The compliant-insertion section of the control panel.

        Always present in execute mode, so the operator can see whether the
        mates are being replayed open-loop or run compliant, and which arm is
        doing what. The sliders are read live at the start of each insertion.
        """
        self.widgets.append(Separator('--- compliant insertion ---'))
        self.insert_sep = Separator(self.insertion_msg)
        self.widgets.append(self.insert_sep)
        for ins in self.insertions:
            self.widgets.append(Separator(f'  {ins.describe()}'))
        # Role lines: what each arm is doing, named by the same side as the
        # titles in the Wrench window.
        self.role_seps = [Separator(f'{side}: idle') for side in ARM_SIDES]
        self.widgets += self.role_seps
        if not self.insertions:
            self.insert_toggle = None
            self.push_slider = self.radius_slider = self.approach_slider = None
            return
        self.insert_toggle = Toggle('skip insertions (plain tracking)',
                                    lambda *_: None, False)
        self.widgets.append(self.insert_toggle)
        self.push_slider = Slider('push force [N]', lambda *_: None,
                                  2.0, 40.0, self.ins_params.push_force)
        self.radius_slider = Slider('search radius [mm]', lambda *_: None,
                                    2.0, 25.0,
                                    self.ins_params.search_radius * 1000.0)
        self.approach_slider = Slider('approach speed [mm/s]', lambda *_: None,
                                      2.0, 50.0,
                                      self.ins_params.approach_speed * 1000.0)
        self.widgets += [self.push_slider, self.radius_slider,
                         self.approach_slider]
        # ! Only reachable in the 'held' state, after an insertion did not
        # ! seat. Nothing opens a gripper on an unseated part by itself.
        self.widgets.append(Button('Retry insertion', self.on_retry_insertion))
        self.widgets.append(Button('Release & continue (UNSEATED)',
                                   self.on_release_and_continue))
        self.widgets.append(Button('Abort run', self.on_abort_run))

    def update(self):
        """Per-tick: render the UI, then advance the active mode."""
        if not _common._global_backend.step():
            # Main window closed -> stop the arms first, then shut down.
            self._stop_tracking('UI window closed')
            rclpy.shutdown()
            return
        for w in self.widgets:
            w.update()
        if self.mode == 'preview':
            self._tick_preview()
        else:
            self._tick_execute()
        # Mirror the current display configuration into the 3D view, with the
        # gripper models following their flanges and finger state.
        pp.set_joint_positions(self.viz_robot, self._viz_joints,
                               [float(v) for v in self._plot_q12])
        for i, (grip_body, grip_att, grip_joints) in enumerate(self.viz_grippers):
            grip_att.assign()   # re-pin the gripper base to the moved flange
            angle = self.grip_viz_angle[i]
            pp.set_joint_positions(
                grip_body, grip_joints,
                [factor * angle for factor in GRIPPER_VIZ_FACTORS])
        self._draw_parts()

    def _mates(self) -> list:
        """The plan's mates (child part, parent part, funnel, release), cached.

        Located from the plan's `assemble` actions rather than the per-sample
        servo flag -- see `find_insertions`. Two callers want them: --insertions
        routes each one to the compliant skill, and the wrench log slices the
        run's force record to each one's window. Reading them needs the
        planner's forward kinematics, so this is done once and only when asked.

        Returns:
            list: Insertion records in trajectory order; empty when there is no
            --plan-json, or the planner's fk could not be imported.
        """
        if self._mates_cache is not None:
            return self._mates_cache
        self._mates_cache = []
        if not self.args.plan_json:
            return self._mates_cache
        try:
            from husky_assembly_tamp.keyframe import ssik_inprocess
            self._mates_cache = find_insertions(
                self.traj, self.args.plan_json, ssik_inprocess.fk,
                min_depth_m=self.args.ins_min_depth / 1000.0,
                log=self.get_logger().warn)
        except Exception as e:
            # Not fatal: without the mates the run is plain tracking and the
            # wrench log is simply skipped.
            self.get_logger().warn(f'could not locate the plan\'s mates: {e}')
        return self._mates_cache

    def _build_insertion_params(self) -> InsertionParams:
        """Insertion tunables, with the CLI overriding the defaults.

        Returns:
            InsertionParams: The parameters every insertion of this run starts
            from (the UI sliders adjust a copy at START).
        """
        params = InsertionParams()
        # ! The UR wants [x, y, z, rx, ry, rz] with a rotation VECTOR, while
        # ! the repo's constant is a pybullet pose with a quaternion -- and it
        # ! carries a 180 degree yaw, which must survive the conversion.
        params.tcp_offset = tuple(TOOL0_FROM_GRIPPER_TCP[0]) + tuple(
            Rotation.from_quat(TOOL0_FROM_GRIPPER_TCP[1]).as_rotvec())
        args = self.args
        for value, name in ((args.ins_push_force, 'push_force'),
                            (args.ins_contact_force, 'contact_force'),
                            (args.ins_guard_force, 'guard_force'),
                            (args.ins_budget, 'budget_s')):
            if value is not None:
                setattr(params, name, float(value))
        for value, name in ((args.ins_search_radius, 'search_radius'),
                            (args.ins_search_pitch, 'search_pitch'),
                            (args.ins_approach_speed, 'approach_speed'),
                            (args.ins_insert_speed, 'insert_speed')):
            if value is not None:
                setattr(params, name, float(value) / 1000.0)
        return params

    def _tool0_pose_at(self, arm_index: int, q12) -> tuple:
        """Tool0 pose of one arm at a given 12-joint configuration.

        Moves the viewer's robot to `q12` to read the flange, so the caller
        gets the SAME kinematics the 3D view shows. The configuration is left
        in place -- `update()` rewrites it every tick anyway.

        Args:
            arm_index (int): 0 = left, 1 = right.
            q12 (np.ndarray): 12 joint values, left arm then right [rad].

        Returns:
            tuple: The tool0 pose (point, quaternion xyzw).
        """
        pp.set_joint_positions(self.viz_robot, self._viz_joints,
                               [float(v) for v in q12])
        return pp.get_link_pose(self.viz_robot, self._tool0_links[arm_index])

    def _fit_every_grasp(self):
        """Close the jaws onto each grasped part once, and report the clearance.

        ! Showing the fingers snapped to FULLY closed drove the meshes through
        ! the part, so the picture said nothing about whether the closing
        ! fingertips would sweep the table. Every grasp in the trajectory is
        ! fitted here, at the configuration the plan grasps from, and the
        ! resulting knuckle angle is what the view shows from then on.

        The Robotiq is a four-bar: closing swings the inner finger ~13 mm
        further along the approach axis, so the clearance below is measured at
        the FITTED angle, which is the pose that would really reach the table.
        """
        table = next((body for body in self.obstacles
                      if OBSTACLE_LABELS.get(body, '').startswith('table')), None)
        self._jaw_fit = {}
        for part, episodes in self.parts.timeline.items():
            for index, parent, _rel in episodes:
                if parent == 'world' or parent[0] != 'arm':
                    continue
                arm = parent[1]
                grip_body, grip_att, grip_joints = self.viz_grippers[arm]
                with pp.WorldSaver():
                    pp.set_joint_positions(self.viz_robot, self._viz_joints,
                                           [float(v) for v in self.traj.q12[index]])
                    grip_att.assign()
                    tool0 = [pp.get_link_pose(self.viz_robot, link)
                             for link in self._tool0_links]
                    for name, pose in self.parts.poses_at(index, tool0).items():
                        pp.set_pose(self.part_bodies[name], pose)
                    # The pads may be on a MATED member rather than on the
                    # chain's root part, so offer the whole carried assembly.
                    held = [self.part_bodies[name]
                            for name in self.parts.held_chain(arm, index)]
                    fit = fit_jaws(grip_body, grip_joints, GRIPPER_VIZ_FACTORS,
                                   held or [self.part_bodies[part]], table,
                                   GRIPPER_OPEN, GRIPPER_CLOSE)
                self._jaw_fit[(arm, part, index)] = fit
                self._log_jaw_fit(arm, part, self.traj.times[index], fit)

    def _log_jaw_fit(self, arm: int, part: str, t: float, fit: dict):
        """One line per grasp: how far the jaws close and what the pads clear.

        Args:
            arm (int): 0 = left, 1 = right.
            part (str): The part being grasped.
            t (float): Trajectory time of the grasp [s].
            fit (dict): The `fit_jaws` result.
        """
        clearance = fit['table_clearance_mm']
        line = (f'[jaws] {ARM_SIDES[arm]} closes on {part} @t={t:.2f}s: knuckle '
                f'{fit["angle"]:.3f} rad, opening {fit["opening_mm"]:.1f} mm')
        if clearance is None:
            self.get_logger().info(line + ', pads nowhere near the table')
        elif clearance > 0.0:
            self.get_logger().info(
                line + f', pad tip {clearance:+.1f} mm above the table')
        else:
            self.get_logger().warn(
                line + f', pad tip {clearance:.1f} mm BELOW the table -- the '
                f'closing fingers hit it')
        if fit['closed_on_nothing']:
            self.get_logger().warn(
                f'[jaws] {ARM_SIDES[arm]} reaches full close without meeting '
                f'{part} (gaps {fit["gaps_mm"][0]:.1f}/{fit["gaps_mm"][1]:.1f} '
                f'mm) -- the planned grasp holds nothing here')

    def _close_angle(self, arm: int, part: str, index: int) -> float:
        """Knuckle angle to SHOW for one arm closed on one part.

        The grasp is named by its EPISODE, not just by (arm, part): a part can
        be grasped more than once by the same arm, with a different grasp each
        time, and each of those closes to its own angle.

        Args:
            arm (int): 0 = left, 1 = right.
            part (str): The part being held, or None when it is not known.
            index (int): Trajectory sample the grasp is being drawn at.

        Returns:
            float: The fitted angle, or the fully-closed one with no fit.
        """
        if part is None or self.parts is None:
            return GRIPPER_CLOSE
        key = (arm, part, self.parts.episode_start(part, index))
        return self._jaw_fit.get(key, {}).get('angle', GRIPPER_CLOSE)

    def _draw_parts(self):
        """Move every part body to where it is at the displayed sample.

        Held parts hang off the flanges the 3D view is already showing, so
        during execution they follow the REAL arms; the grasp itself is the
        planned one.
        """
        if self.parts is None:
            return
        tool0 = [pp.get_link_pose(self.viz_robot, link)
                 for link in self._tool0_links]
        for name, pose in self.parts.poses_at(self._viz_sample, tool0).items():
            pp.set_pose(self.part_bodies[name], pose)
        # Recolour only on a change: set_color is a visual-shape write.
        roles = self.parts.role_at(self._viz_sample)
        if roles != self._part_roles:
            for name, role in roles.items():
                if self._part_roles.get(name) != role:
                    pp.set_color(self.part_bodies[name],
                                 {'table': COLOR_TABLE, 'held': COLOR_HELD,
                                  'assembled': COLOR_ASSEMBLED}[role])
            self._part_roles = roles

    # --- --- PREVIEW MODE --- ---

    def on_play_pause(self):
        """Toggle playback; restarts from 0 when pressed at the end."""
        if not self.playing and self.play_t >= self.traj.duration:
            self.play_t = 0.0
        self.playing = not self.playing

    def _tick_preview(self):
        """Advance the wall-clock playhead, honor scrubbing, update readouts."""
        now = time.monotonic()
        speed = float(self.speed_slider.value or 1.0)
        if self.playing:
            self.play_t = min(self.play_t + (now - self._last_wall) * speed,
                              self.traj.duration)
            if self.play_t >= self.traj.duration:
                self.playing = False
        self._last_wall = now

        # ? Two-way slider: if the widget differs from what we last wrote,
        # ? the user dragged it -- adopt that as the playhead. The 0.01 s
        # ? tolerance absorbs the widget's float32 rounding of our writes.
        slider_t = self.time_slider.value
        if slider_t is not None and abs(float(slider_t) - self._slider_written) > 0.01:
            self.play_t = float(slider_t)
        _common._global_backend.set_value(self.time_slider._handle,
                                          float(self.play_t))
        self._slider_written = float(self.play_t)

        i = self.traj.state_at(self.play_t)
        self._plot_q12 = self.traj.q12[i]
        self._viz_sample = i
        # A closed gripper is drawn at the angle its jaws actually fit the
        # part at, not snapped shut through it.
        self.grip_viz_angle = [
            self._close_angle(k, self.parts.held_by(k, i) if self.parts else None, i)
            if closed else GRIPPER_OPEN
            for k, closed in enumerate(self.traj.grip_closed[i])]

        self.readout_sep.set_text(
            f'sample {i + 1}/{self.traj.n_samples}  t={self.play_t:7.2f}s  '
            f'{"PLAYING" if self.playing else "paused"} {speed:.1f}x')
        # Gripper state at the playhead + the most recent event before it.
        grip = ['CLOSED' if c else 'open' for c in self.traj.grip_closed[i]]
        past = [ev for ev in self.traj.events if ev.time <= self.play_t]
        last = (f'   last event: {past[-1].kind} {ARM_SIDES[past[-1].arm_index]} '
                f'@{past[-1].time:.2f}s' if past else '')
        self.grip_sep.set_text(f'grippers: L {grip[0]} | R {grip[1]}{last}')
        self.status_sep.set_text(
            f'servo_controller flag: {bool(self.traj.servo_flags[i])}')

    # --- --- EXECUTE MODE: BUTTONS --- ---

    def on_connect(self):
        """Open RTDEControl + RTDEReceive to both arms (blocks a few seconds)."""
        if self.state != 'disconnected':
            self.get_logger().warn(f'Connect ignored in state {self.state}')
            return
        # Refuse a trajectory whose reference ever exceeds the velocity clamp:
        # clamping the reference would distort the path, not just slow it.
        ref_vmax = self.traj.check_velocity_limit(self.args.max_joint_vel)
        if ref_vmax > self.args.max_joint_vel:
            self.get_logger().error(
                f'reference velocity {ref_vmax:.3f} rad/s exceeds '
                f'--max-joint-vel {self.args.max_joint_vel} -- refusing')
            return
        # The same guard bounds the speed slider: at scale s the reference
        # asks for s * ref_vmax, which must still fit under the clamp.
        self.scale_cap = min(MAX_SPEED_SCALE,
                             self.args.max_joint_vel / max(ref_vmax, 1e-6))
        self.get_logger().info(
            f'reference velocity check OK ({ref_vmax:.3f} '
            f'< {self.args.max_joint_vel} rad/s); execution speed capped at '
            f'x{self.scale_cap:.2f}')
        ips = (self.args.left_ip, self.args.right_ip)
        try:
            for i, ip in enumerate(ips):
                self.get_logger().info(f'connecting {ARM_SIDES[i]} arm @ {ip} ...')
                self.rtde_c[i] = RTDEControlInterface(ip, self.args.frequency)
                self.rtde_r[i] = RTDEReceiveInterface(ip, self.args.frequency)
            self.state = 'connected'
            self.get_logger().info('RTDE connected to both arms')
            # ! Report the tool setting and the raw wrench once, on the record.
            # ! Push each tool by hand along base +x/+y/+z and check the signs
            # ! before trusting any force number -- an insertion presses along
            # ! whatever this frame says, and a pendant tool offset that is not
            # ! the gripper's puts the forces at the wrong point entirely.
            for i in range(2):
                self.get_logger().info(
                    f'{ARM_SIDES[i]} pendant TCP offset '
                    f'{np.round(self.rtde_c[i].getTCPOffset(), 4)} | wrench now '
                    f'{np.round(self.rtde_r[i].getActualTCPForce(), 2)} '
                    f'(base frame, un-zeroed)')
            # * Zero both sensors as soon as they are connected, so every force
            # * the run reports is measured from the parked arms rather than
            # * from whatever bias the sensors woke up with. --no-zero-ft
            # * leaves them as they are.
            if getattr(self.args, 'no_zero_ft', False):
                self.get_logger().warn(
                    '--no-zero-ft: the force sensors keep their existing bias')
            else:
                self._zero_ft('at connect')
        except Exception as e:
            self.get_logger().error(f'RTDE connect failed: {e}')
            self.rtde_c = [None, None]
            self.rtde_r = [None, None]

    def _zero_ft(self, reason: str) -> bool:
        """Zero both arms' force/torque sensors where they stand.

        `zeroFtSensor` subtracts whatever the sensor reads now from everything
        it reports afterwards, so this makes the CURRENT load the new origin --
        the tool's own weight, the cable pull, and the part in the jaws if
        there is one. Both arms are done together so their numbers stay
        comparable.

        ! The arms must be still, and they must be in the pose whose load you
        ! want subtracted: gravity on the tool changes with the wrist, so a
        ! zero taken at the park pose is only approximately right further along
        ! the trajectory. That is why a compliant insertion re-zeroes for
        ! itself, with the part already held, at the start of every mate.

        Args:
            reason (str): Why this zeroing happened; goes in the run log.

        Returns:
            bool: True when both arms were zeroed.
        """
        if any(c is None for c in self.rtde_c):
            self.get_logger().warn('zero FT ignored -- not connected')
            return False
        before = [np.round(self._read_wrench(i, self.rtde_r[i]), 2)
                  for i in range(2)]
        ok = True
        for i in range(2):
            try:
                self.rtde_c[i].zeroFtSensor()
            except Exception as e:
                ok = False
                self.get_logger().error(
                    f'{ARM_SIDES[i]} zeroFtSensor failed: {e}')
        # Let the new bias reach the RTDE stream before reading it back, so
        # the log shows what the run will actually see.
        time.sleep(FT_ZERO_SETTLE_S)
        after = [np.round(self._read_wrench(i, self.rtde_r[i]), 2)
                 for i in range(2)]
        self.ft_zeros.append({
            'reason': reason,
            'at': f'{datetime.now():%Y-%m-%d %H:%M:%S}',
            'before': [b.tolist() for b in before],
            'after': [a.tolist() for a in after],
            'ok': ok})
        for i, side in enumerate(ARM_SIDES):
            self.get_logger().info(
                f'{side} FT zeroed ({reason}): {before[i]} -> {after[i]} '
                f'(base frame)')
        # A sensor that still reads far from zero afterwards usually means the
        # arm moved while it happened, or the zero never landed.
        worst = max(float(np.abs(a).max()) for a in after)
        if ok and worst > 5.0:
            self.get_logger().warn(
                f'FT still reads up to {worst:.1f} N/Nm after zeroing -- was '
                f'an arm moving, or being touched?')
        return ok

    def on_zero_ft(self):
        """Zero FT button: make the arms' current load the new zero."""
        if self.state not in IDLE_STATES:
            self.get_logger().warn(
                f'zero FT ignored in state {self.state} -- only while the arms '
                f'are standing still and nothing is running')
            return
        self._zero_ft('operator button')

    def _rebase_traj_to_live_branch(self):
        """Move the trajectory onto the 2*pi branch the arms actually stand on.

        ! The approach planner unwraps its goal to the branch nearest the live
        ! pose (`open_loop_approach.unwrap_goal`), so after running an approach
        ! an arm can sit a full turn away from the AUTHORED start on one joint
        ! -- physically in exactly the right place, but 6.2832 rad off on
        ! paper. Without this the start check refuses a pose that is correct,
        ! and forcing it through would make the tracker's first command ask
        ! for a whole revolution.
        !
        ! Shifting the joint's entire column is geometrically a no-op (same
        ! tool path, same velocities), but the wrapped branch can sit far
        ! closer to a joint's end stop than the authored one did -- measured
        ! on the bench file, the left pan went from 4.78 rad of room to 0.042
        ! rad. So the shift is refused unless every sample keeps
        ! BRANCH_LIMIT_MARGIN to the URDF limits, and the operator is told to
        ! jog back instead. This is the same rule the approach planner
        ! unwraps by, so the two can never disagree about the branch.
        """
        if self.rtde_r[0] is None:                     # preview only, nothing live
            return
        lower, upper = pp.get_custom_limits(self.viz_robot, self._viz_joints, {})
        # The SAME rule the approach planner unwraps by: the branch is only
        # acceptable when the whole trajectory keeps its margin to the limits.
        lower = np.asarray(lower) + BRANCH_LIMIT_MARGIN
        upper = np.asarray(upper) - BRANCH_LIMIT_MARGIN
        traj, turns, blocked = rebase_to_branch(
            self.traj, self._live_q12(), lower, upper)
        if blocked:
            names = ', '.join(JOINT_NAMES_12[j] for j in blocked)
            self.get_logger().error(
                f'the arms sit a full turn from the trajectory start on '
                f'{names}, and running the trajectory on that branch would '
                f'come within {BRANCH_LIMIT_MARGIN} rad of the joint limit -- '
                f'jog that joint a full turn back by pendant, then plan the '
                f'approach again')
            return
        if not turns.any():
            return
        moved = ', '.join(f'{JOINT_NAMES_12[j]} {turns[j]:+d} turn(s)'
                          for j in np.nonzero(turns)[0])
        self.traj = traj
        self.get_logger().warn(
            f'trajectory rebased onto the branch the arms are on: {moved}. '
            f'Same tool path and same velocities -- only the joint numbers '
            f'move, so the tracker starts from where the arms actually are.')
        for j in np.nonzero(turns)[0]:
            self.get_logger().info(
                f'  {JOINT_NAMES_12[j]} now runs '
                f'{traj.q12[:, j].min():+.3f}..{traj.q12[:, j].max():+.3f} rad')

    def on_check_start_pose(self):
        """Compare live joints against the trajectory start; gate START on it.

        Also allowed after a finished run ('done'/'aborted'): the RTDE
        connections are still up and the tracker threads have ended, so the
        operator can re-arm and run again without restarting the program.
        """
        if self.state not in IDLE_STATES:
            self.get_logger().warn(f'Check ignored in state {self.state}')
            return
        self._rebase_traj_to_live_branch()
        self.start_deltas = []
        for i in range(2):
            q_now = np.asarray(self.rtde_r[i].getActualQ())
            q_start = self.traj.sample(i, 0.0)[0]
            delta = np.abs(q_now - q_start)
            self.start_deltas.append(delta)
            self.get_logger().info(
                f'{ARM_SIDES[i]} start deltas [rad]: '
                + np.array2string(delta, precision=4))
        worst = [float(d.max()) for d in self.start_deltas]
        ok = max(worst) <= self.args.start_tol
        self.state = 'ready' if ok else 'connected'
        self.delta_sep.set_text(
            f'start pose: {"OK" if ok else "OFF"} '
            f'(max delta L {worst[0]:.4f} | R {worst[1]:.4f} rad, '
            f'tol {self.args.start_tol})')

    def on_move_to_start(self):
        """Slow sequential moveJ of both arms to the first trajectory sample.

        ! No collision checking whatsoever -- a straight line in joint space.
        ! Prefer 'Plan approach to start'; this stays only as an escape hatch.
        """
        if self.state not in IDLE_STATES:
            self.get_logger().warn(f'Move-to-start ignored in state {self.state}')
            return
        if self.start_deltas is None:
            self.get_logger().warn('run Check start pose first')
            return
        worst = max(float(d.max()) for d in self.start_deltas)
        if worst > MAX_MOVE_TO_START_DELTA:
            self.get_logger().error(
                f'largest start delta {worst:.2f} rad > '
                f'{MAX_MOVE_TO_START_DELTA} -- jog closer by pendant first')
            return
        self.state = 'moving'

        def worker():
            # Sequential left-then-right: arms never move at the same time
            # from an unplanned configuration. Blocking moveJ keeps the UI
            # thread free; 'moving' state keeps everyone else off the RTDE
            # interfaces meanwhile.
            try:
                for i in range(2):
                    q_start = self.traj.sample(i, 0.0)[0]
                    self.get_logger().info(f'moving {ARM_SIDES[i]} arm to start ...')
                    self.rtde_c[i].moveJ(list(q_start), MOVE_TO_START_SPEED,
                                         MOVE_TO_START_ACCEL)
                    self.live_q[i] = q_start.copy()
                self.state = 'connected'
                self.on_check_start_pose()   # re-check -> 'ready' on success
            except Exception as e:
                self.get_logger().error(f'move to start failed: {e}')
                self.state = 'connected'

        threading.Thread(target=worker, daemon=True).start()

    def _grippers_reachable(self) -> bool:
        """Are both gripper action servers visible right now? Logs the fix if not.

        ! `_fire_gripper_event` skips a command whose server is missing, which
        ! is the right thing for ONE lost server mid-run -- but a run whose
        ! servers were never seen executes a whole assembly with the jaws
        ! never closing (2026-09-10: ten events, all SKIPPED-no-server). The
        ! usual cause is not the Husky but this terminal: the gripper stacks
        ! live in ROS_DOMAIN_ID 86 on Cyclone, and a shell without those
        ! exports discovers nothing. So this is checked again at START, when
        ! discovery has had every chance, and a missing server is a refusal.

        Returns:
            bool: True when both servers are ready, or grippers are off.
        """
        if self.robot is None:
            return True
        missing = [ARM_SIDES[i] for i, client in enumerate(self.robot.act_grippers)
                   if not client.server_is_ready()]
        if not missing:
            return True
        self.get_logger().error(
            f'gripper action server(s) NOT visible for: {", ".join(missing)} '
            f'({self.args.robot_name}/<side>_gripper/robotiq_gripper_controller/gripper_cmd). '
            f'This shell has ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "<unset>")} '
            f'RMW_IMPLEMENTATION={os.environ.get("RMW_IMPLEMENTATION", "<default>")}; '
            f'Cindy\'s stacks are on ROS_DOMAIN_ID=86 with rmw_cyclonedds_cpp '
            f'(doc/rtde_network_setup.md). If the env is right, check `tmux ls` on the '
            f'Husky for gripper_left/gripper_right. Pass --no-gripper to run without.')
        return False

    def on_start(self):
        """Run the loaded trajectory up to the slider's cutoff sample."""
        if self.state != 'ready':
            self.get_logger().warn(
                f'START ignored in state {self.state} (need a passed Check)')
            return
        if not self._grippers_reachable():
            self.get_logger().error('START refused: the trajectory\'s gripper commands '
                                    'would all be skipped')
            return
        # Freeze the cutoff for this run: reference, break condition and the
        # gripper event list all follow from it.
        self.end_idx = self._read_end_idx()
        t_end = float(self.traj.times[self.end_idx - 1])
        events = [ev for ev in self.traj.events if ev.time <= t_end]
        self.get_logger().info(
            f'tracking started: samples 1..{self.end_idx} of '
            f'{self.traj.n_samples}, {t_end:.1f}s to go at speed '
            f'x{self.speed_scale:.2f} ({t_end / max(self.speed_scale, 1e-6):.1f}s '
            f'of wall time), {len(events)} gripper events (reference speed at '
            f'the cutoff {self.traj.speed_at(t_end):.3f} rad/s)')
        self.resume_offset = np.zeros(12)
        self.resume_from = 0.0
        self.phases = self._build_phases(t_end, events)
        self.phase_idx = -1
        self._next_phase(fresh=True)

    def _build_phases(self, t_end: float, events: list) -> list:
        """Cut the run into tracking, insertion and release phases.

        With no insertions this is one tracking phase and the run behaves
        exactly as it always has.

        Args:
            t_end (float): Cutoff time of the whole run [s].
            events (list): Gripper events inside the cutoff.

        Returns:
            list: Phase dicts, each with a 'kind' of 'track', 'insert' or
            'release'.
        """
        wanted = [ins for ins in self.insertions if ins.t_open <= t_end]
        if self.insert_toggle is not None and self.insert_toggle.value:
            wanted = []
            self.get_logger().info('"skip insertions" is on -- plain tracking')
        # ! An insertion the cutoff lands inside cannot be run: half a mate is
        # ! worse than none, so the run stops at its funnel mouth instead.
        straddling = [ins for ins in self.insertions
                      if ins.t_start < t_end < ins.t_open]
        if straddling:
            t_end = straddling[0].t_start
            events = [ev for ev in events if ev.time <= t_end]
            self.get_logger().warn(
                f'the cutoff lands inside the {straddling[0].child} mate -- '
                f'stopping at its funnel mouth, t={t_end:.2f}s')

        phases, at = [], 0.0
        for ins in wanted:
            if ins.t_start > t_end:
                break
            phases.append({'kind': 'track', 'from': at, 'to': ins.t_start,
                           'events': [ev for ev in events
                                      if at <= ev.time <= ins.t_start]})
            phases.append({'kind': 'insert', 'insertion': ins})
            phases.append({'kind': 'release', 'insertion': ins})
            at = ins.t_open
        phases.append({'kind': 'track', 'from': at, 'to': t_end,
                       'events': [ev for ev in events if ev.time > at]})
        return phases

    def _next_phase(self, fresh: bool = False):
        """Start the next phase of the run, or finish if there are none left.

        Args:
            fresh (bool): True for the first phase of a run, which resets the
                logs and the plots.
        """
        self.phase_idx += 1
        if self.phase_idx >= len(self.phases):
            self._finish()
            return
        phase = self.phases[self.phase_idx]
        if phase['kind'] == 'track':
            if phase['to'] <= phase['from'] and not fresh:
                self._next_phase()      # nothing to do in a zero-length gap
                return
            self.get_logger().info(
                f'phase {self.phase_idx + 1}/{len(self.phases)}: tracking '
                f't={phase["from"]:.2f}..{phase["to"]:.2f}s, '
                f'{len(phase["events"])} gripper events')
            self.live_role = ['tracking', 'tracking']
            self._launch_trackers(self.traj, phase['to'], phase['events'],
                                  'trajectory', t_start=phase['from'],
                                  fresh=fresh)
        elif phase['kind'] == 'insert':
            self._launch_insertion(phase['insertion'])
        else:
            self._launch_release(phase['insertion'])

    def _launch_trackers(self, traj, t_end: float, events: list, label: str,
                         t_start: float = 0.0, hold_arm=(False, False),
                         hold_q12=None, fresh: bool = True):
        """Start one tracker thread per arm, both on one shared clock.

        Args:
            traj (OpenLoopTraj): Reference the threads follow.
            t_end (float): Cutoff time [s].
            events (list): Gripper events to fire during this run.
            label (str): 'trajectory' or 'approach'; names the log folder.
            t_start (float): Trajectory time this phase resumes from [s]. The
                shared clock is offset so the reference picks up there.
            hold_arm (tuple): Per arm, whether it should stand still on
                `hold_q12` instead of following the reference.
            hold_q12 (np.ndarray): The 12-joint configuration a held arm
                keeps; defaults to the reference at `t_start`.
            fresh (bool): Start a new log, or keep appending to the run's.
        """
        self.active_traj = traj
        self.t_end = t_end
        self.run_events = events
        self.run_label = label
        self.hold_arm = list(hold_arm)
        self.hold_q12 = (traj.q12[traj.state_at(t_start)].copy()
                         if hold_q12 is None else np.asarray(hold_q12).copy())
        if fresh:
            self.exec_log = [{k: [] for k in ('t', 'q_ref', 'q_actual',
                                              'qd_cmd', 'wrench', 'mode',
                                              'speed_scale')}
                             for _ in range(2)]
            self.scale_changes = []
            self.pauses = []
            self.fired_events = []
            self.insertion_records = []
            self.insertion_logs = {}
            for plot in self.err_plots:
                plot.reset()
            self._log_saved = False
        self.thread_done = [False, False]
        self.thread_error = [None, None]
        self.next_event_idx = 0
        self.stop_evt = threading.Event()
        # A phase always starts running, never frozen -- including the one
        # after a run that was aborted while it was paused. `clock.start`
        # throws away any ramp that was in progress.
        self.paused = False
        self.stop_reason = None
        # ! Start the clock BEFORE spawning: it IS the synchronization. It
        # ! begins at t_start (so a phase can resume mid-file) after a pre-roll
        # ! both threads wait out at the start pose, and runs at whatever rate
        # ! the execution-speed slider last asked for.
        self.clock.start(t_start, START_PREROLL_S, self.speed_scale)
        self.threads = [threading.Thread(target=self._arm_tracker, args=(i,),
                                         daemon=True) for i in range(2)]
        self.state = 'tracking'
        for th in self.threads:
            th.start()

    # --- --- EXECUTE MODE: APPROACH MOTION --- ---

    def _launch_insertion(self, ins):
        """Run one mate: the holding arm stands still, the other one inserts.

        Args:
            ins (Insertion): The mate to run.
        """
        self.get_logger().info(
            f'phase {self.phase_idx + 1}/{len(self.phases)}: INSERTING '
            f'{ins.describe()}')
        params = self._slider_params()
        self.insertion = InsertionController(
            self.rtde_c[ins.arm_index], self.rtde_r[ins.arm_index], params,
            1.0 / self.args.frequency, log_fn=self.get_logger().info)
        cols = slice(6 * ins.arm_index, 6 * ins.arm_index + 6)
        self.insertion.setup(ins.q12_start[cols], ins.q12_open[cols])
        self.insertion_of = ins
        self.live_role = ['', '']
        self.live_role[ins.holder_index] = (
            f'HOLDING {ins.parent} (guard {params.guard_force:.0f} N)')

        # The holder runs on the ordinary tracker in hold mode, so it keeps
        # its own logging, its own error abort and the shared stop event.
        hold = [False, False]
        hold[ins.holder_index] = True
        self.hold_arm = hold
        self.hold_q12 = ins.q12_start.copy()
        self.hold_wrench = [None, None]
        self.run_events = []
        self.next_event_idx = 0
        self.thread_done = [False, False]
        self.thread_error = [None, None]
        self.stop_evt = threading.Event()
        # ! Always real time (scale 1), whatever the speed slider says: the
        # ! insertion is force-controlled against the world, and its contact
        # ! thresholds, stall timer and budget are all in real seconds.
        self.clock.start(ins.t_start, START_PREROLL_S, 1.0)
        self.threads = [
            threading.Thread(target=self._arm_tracker,
                             args=(ins.holder_index,), daemon=True),
            threading.Thread(target=self._insertion_thread,
                             args=(ins,), daemon=True)]
        self.state = 'inserting'
        for th in self.threads:
            th.start()

    def _insertion_thread(self, ins):
        """Drive the insertion skill in its own thread, at the RTDE rate.

        Args:
            ins (Insertion): The mate being run.
        """
        arm_i = ins.arm_index
        rtde_c = self.rtde_c[arm_i]
        control = self.insertion
        try:
            # ! Wait out the pre-roll like the trackers do, so the holding arm
            # ! is already settled on its configuration before anything pushes
            # ! against it.
            while self.clock.now() < ins.t_start:
                time.sleep(0.005)
            t0 = time.monotonic()
            while not self.stop_evt.is_set():
                cycle = rtde_c.initPeriod()
                self._read_wrench(arm_i, self.rtde_r[arm_i])
                running = control.step(time.monotonic() - t0)
                self.live_q[arm_i] = np.asarray(
                    self.rtde_r[arm_i].getActualQ())
                self.live_role[arm_i] = (
                    f'INSERTING {ins.child}: {control.phase} '
                    f'r={control.search_r * 1000:4.1f}mm '
                    f'|F|={np.linalg.norm(self.live_wrench[arm_i][:3]):5.1f}N')
                if not running:
                    break
                rtde_c.waitPeriod(cycle)
        except Exception as e:
            self.thread_error[arm_i] = f'insertion: {e}'
        finally:
            try:
                rtde_c.speedStop()
            except Exception:
                pass
            # ! The holding arm's loop has no end of its own -- it stands
            # ! still until told to stop. The insertion finishing IS that
            # ! signal, so set it here whatever the outcome, or the phase
            # ! would never complete.
            self.stop_evt.set()
            self.thread_done[arm_i] = True

    def _launch_release(self, ins):
        """Open the gripper on a seated part and let both arms settle.

        Args:
            ins (Insertion): The mate that just finished.
        """
        self.get_logger().info(
            f'phase {self.phase_idx + 1}/{len(self.phases)}: releasing '
            f'{ins.child} and settling for {self.args.release_wait:.1f}s')
        event = next((ev for ev in self.traj.events
                      if ev.kind == 'open' and ev.arm_index == ins.arm_index
                      and abs(ev.time - ins.t_open) < 1e-6), None)
        if event is not None:
            self._fire_gripper_event(event, ins.t_open)
        # Both arms stand still on where they ACTUALLY are, so releasing does
        # not drag the freshly seated part anywhere.
        self.live_role = ['settling', 'settling']
        self.release_until = time.monotonic() + self.args.release_wait
        self._launch_trackers(self.traj, ins.t_open, [], 'trajectory',
                              t_start=ins.t_open, hold_arm=(True, True),
                              hold_q12=self._live_q12(), fresh=False)
        self.state = 'releasing'

    def _slider_params(self) -> InsertionParams:
        """Insertion parameters with the operator's live slider values folded in.

        Returns:
            InsertionParams: A copy of the run's parameters, adjusted.
        """
        params = InsertionParams(**dict(self.ins_params.__dict__))
        if self.push_slider is not None:
            params.push_force = float(self.push_slider.value)
        if self.radius_slider is not None:
            params.search_radius = float(self.radius_slider.value) / 1000.0
        if self.approach_slider is not None:
            params.approach_speed = float(self.approach_slider.value) / 1000.0
        return params

    def _live_q12(self) -> np.ndarray:
        """Both arms' live joint configuration as one 12-vector."""
        return np.concatenate([np.asarray(self.rtde_r[i].getActualQ())
                               for i in range(2)])

    def on_plan_approach(self):
        """Plan a collision-checked path from the live pose to sample 0."""
        if self.state not in IDLE_STATES:
            self.get_logger().warn(f'Plan ignored in state {self.state}')
            return
        start_q12 = self._live_q12()
        goal_q12 = self.traj.q12[0]
        # ! The collision check sees the VIEW's fingers, not the real ones:
        # ! after a finished run they still stand at the last fitted close, and
        # ! a closed model finger beside a part box reads as a collision the
        # ! open real gripper does not have. Say what was checked with, so an
        # ! "already in collision" on a visibly clear arm can be read right.
        self.get_logger().info(
            f'[approach] checking with the model fingers at '
            f'L {self.grip_viz_angle[0]:.3f} / R {self.grip_viz_angle[1]:.3f} rad '
            f'(0 = fully open, {GRIPPER_CLOSE} = closed); the real jaws may differ')
        # ! Runs inline, like the monitor's planning buttons: the search and
        # ! the 3D view share one PyBullet world, so nothing else may touch it
        # ! meanwhile. The window freezes for the search (bounded by the two
        # ! passes' time budgets) while the arms stand still.
        self.get_logger().info('planning the approach, the window will not '
                               'respond until it finishes ...')
        try:
            attachments = [grip[1] for grip in self.viz_grippers]
            path, info = plan_approach(
                self.viz_robot, attachments, self.obstacles,
                start_q12, goal_q12, log=self.get_logger().info,
                traj_q12=self.traj.q12, branch_margin=BRANCH_LIMIT_MARGIN)
        except Exception as e:
            path, info = None, {'failure_reason': f'error {e}'}
            self.get_logger().error(f'approach planning error: {e}')
        if path is None:
            self.approach_traj = None
            self.approach_msg = f'approach: FAILED ({info.get("failure_reason")})'
            return
        self.approach_traj = approach_traj_from_path(
            path, max_joint_vel=self.args.approach_vel)
        self.approach_msg = (f'approach: {len(path)} waypoints, '
                             f'{self.approach_traj.duration:.1f}s at '
                             f'{self.args.approach_vel} rad/s -- preview it')
        self.get_logger().info(self.approach_msg)

    def on_preview_approach(self):
        """Play the planned approach in the 3D view without moving anything."""
        if self.approach_traj is None:
            self.get_logger().warn('no approach planned yet')
            return
        if self.state not in IDLE_STATES:
            self.get_logger().warn(f'Preview ignored in state {self.state}')
            return
        self.preview_t = 0.0
        self._preview_wall = time.monotonic()
        self.state = 'previewing'

    def on_execute_approach(self):
        """Run the planned approach through the normal tracker threads."""
        if self.approach_traj is None:
            self.get_logger().warn('no approach planned yet')
            return
        if self.state not in IDLE_STATES:
            self.get_logger().warn(f'Execute ignored in state {self.state}')
            return
        # ! The plan is only valid from the pose it was planned at. If the
        # ! arms moved since (a jog, an earlier run), the first tracked
        # ! reference would yank them back along an unchecked line.
        drift = np.abs(self._live_q12() - self.approach_traj.q12[0]).max()
        if drift > self.args.start_tol:
            self.get_logger().error(
                f'arms moved {drift:.3f} rad since the approach was planned '
                f'(tolerance {self.args.start_tol}) -- plan it again')
            return
        self.get_logger().info(
            f'executing approach: {self.approach_traj.duration:.1f}s, '
            f'peak {np.abs(self.approach_traj.qd12).max():.3f} rad/s')
        self._launch_trackers(self.approach_traj,
                              self.approach_traj.duration, [], 'approach')

    def on_stop(self):
        """Operator STOP: halt both arms, save what was recorded."""
        if self.state in RUNNING_STATES:
            self._stop_tracking('operator STOP')
        elif self.state == 'moving':
            # moveJ blocks its worker thread; interrupting it cross-thread
            # is not safe with ur_rtde -- use the pendant if it must stop NOW.
            self.get_logger().warn(
                'STOP during move-to-start: wait for the slow moveJ to end '
                '(or use the pendant e-stop)')
        else:
            self.get_logger().warn(f'STOP ignored in state {self.state}')

    def on_pause_toggle(self):
        """Space bar / PAUSE button: ramp the arms to a hold, or back up.

        Pausing is a slow-down of the shared clock, NOT a stop: the rate walks
        to zero over `PAUSE_RAMP_S`, so the reference decelerates along its own
        path instead of being cut off. At the bottom both arms are still under
        speedJ, held on the reference by the tracker's P term. STOP (or the
        pendant) is what actually releases them.

        ! The ramp is handed to the clock rather than stepped forward here.
        ! This UI tick can stall for a second at a time (DearPyGui waits on the
        ! display's vsync), and a ramp driven from here would then become the
        ! abrupt stop it exists to avoid. The clock works the rate out from the
        ! wall clock, so both tracker threads see a smooth ramp at their own
        ! 125 Hz and stay in lockstep through it.

        ! Accepted during a tracking phase only -- that covers the trajectory
        ! and the planned approach, which both run on the shared clock. A
        ! force-controlled insertion has no clock to slow down (its thresholds
        ! and timers are in real seconds), so there Space is refused and STOP,
        ! the held-state buttons or the pendant are the way out.

        The resume branch is shared with the marking stop (`_fire_gripper_event`
        freezes the run after a part's first pick; DONE and Space both land
        here to carry on).
        """
        if self.state != 'tracking':
            self.get_logger().warn(
                f'PAUSE ignored in state {self.state} -- only a tracking '
                f'phase can be paused (use STOP)')
            return
        t = self.clock.now()
        if not self.paused:
            self._pause(PAUSE_RAMP_S, 'operator')
            self.get_logger().warn(
                f'PAUSED at t={t:.2f}s -- ramping both arms down over '
                f'{PAUSE_RAMP_S:.1f}s (still under speedJ; STOP to release)')
        else:
            self.paused = False
            self.stop_reason = None
            self.clock.ramp_to(self.speed_scale, PAUSE_RAMP_S)
            if self.pauses:
                self.pauses[-1]['resumed_at'] = round(t, 3)
            self.get_logger().info(
                f'resuming at t={t:.2f}s -- ramping back up to '
                f'x{self.speed_scale:.2f} over {PAUSE_RAMP_S:.1f}s')

    def _pause(self, ramp_s: float, reason: str):
        """Freeze the shared clock over `ramp_s` wall seconds and record it.

        The one pause path: the operator's toggle uses it with the full ramp,
        the marking stop uses it instantly, from a standstill.

        ! The marking stop must NOT go through on_pause_toggle: a first pick
        ! can fire while an operator pause is still ramping down, and a toggle
        ! would resume it instead.

        Args:
            ramp_s (float): Wall seconds to walk the rate to zero; 0 = at once.
            reason (str): 'operator' for Space / the PAUSE button, otherwise
                why the run stopped by itself (shown in the status line).
        """
        self.paused = True
        self.stop_reason = None if reason == 'operator' else reason
        self.clock.ramp_to(0.0, ramp_s)
        self.pauses.append({'paused_at': round(self.clock.now(), 3),
                            'resumed_at': None, 'reason': reason})

    def on_done(self):
        """DONE button: carry on after a marking stop (the same resume as Space)."""
        if self.paused:
            self.on_pause_toggle()
        else:
            self.get_logger().warn('DONE ignored -- the run is not stopped')

    def _read_end_idx(self) -> int:
        """Cutoff sample number read live from the slider, clamped in range.

        Returns:
            int: 1-based sample index to stop at (falls back to the stored
            value if the widget cannot be read).
        """
        value = self.end_slider.value
        if value is None:
            return self.end_idx
        return int(min(max(int(round(float(value))), 1), self.traj.n_samples))

    def _refresh_cutoff_text(self):
        """Describe the slider's cutoff: time, events kept, and speed there.

        The speed matters: stopping where the reference is still moving means
        braking mid-motion, so the operator is warned to pick one of the
        trajectory's still moments instead.
        """
        idx = self._read_end_idx()
        t_end = float(self.traj.times[idx - 1])
        speed = self.traj.speed_at(t_end)
        n_events = sum(1 for ev in self.traj.events if ev.time <= t_end)
        note = '' if speed <= 0.05 else f'  ! still moving {speed:.2f} rad/s'
        self.cutoff_sep.set_text(
            f'-> sample {idx}/{self.traj.n_samples}  t={t_end:.2f}s  '
            f'{n_events} gripper events{note}  |  speed x{self.speed_scale:.2f}'
            f' (max x{self.scale_cap:.2f})')

    # --- --- EXECUTE MODE: TRACKING --- ---

    def _arm_tracker(self, arm_i: int):
        """One arm's speedJ path-tracking loop (runs in its own thread).

        Port of the ReferencePath branch of Valentin's controller: reference
        feed-forward velocity plus a P term on the position error, clamped,
        sent as speedJ at the configured frequency. Any exception (RTDE
        drop, tracking blowup) stops BOTH arms via the shared stop event.

        In HOLD mode the same loop keeps the arm on one configuration for as
        long as the other arm needs -- that is how the arm holding the part
        being inserted INTO stands still -- and watches its own wrench, since
        the press has to go through its grasp.

        Args:
            arm_i (int): 0 = left, 1 = right.
        """
        rtde_c, rtde_r = self.rtde_c[arm_i], self.rtde_r[arm_i]
        p_gain = self.args.p_gain[arm_i]
        vmax = self.args.max_joint_vel
        dt_cmd = 1.0 / self.args.frequency
        log = self.exec_log[arm_i]   # owned by this thread until it finishes
        holding = self.hold_arm[arm_i]
        cols = slice(6 * arm_i, 6 * arm_i + 6)
        q_hold = self.hold_q12[cols].copy() if holding else None
        self.hold_wrench[arm_i] = None
        try:
            while not self.stop_evt.is_set():
                t_cycle = rtde_c.initPeriod()
                # ! Both arms read the SAME clock -- that is the whole
                # ! synchronization, and it stays true when its rate changes.
                t = self.clock.now()
                scale = self.clock.scale
                wrench = self._read_wrench(arm_i, rtde_r)
                if holding:
                    q_ref, qd_ref = q_hold, np.zeros(6)
                    self._check_hold_wrench(arm_i, wrench)
                else:
                    q_ref, qd_ref = self.active_traj.sample(
                        arm_i, t, t_end=self.t_end, brake_time=BRAKE_TIME_S)
                    q_ref = q_ref + self._resume_bias(arm_i, t)
                q_act = np.asarray(rtde_r.getActualQ())
                err = q_ref - q_act
                if np.abs(err).max() > self.args.err_abort:
                    # * The open-loop safety net: if reality drifts this far
                    # * from the plan, something is wrong -- stop everything.
                    raise RuntimeError(
                        f'{ARM_SIDES[arm_i]} tracking error '
                        f'{np.abs(err).max():.3f} rad > --err-abort '
                        f'{self.args.err_abort}')
                # ! The reference is walked at `scale` trajectory seconds per
                # ! wall second, so by the chain rule the feed-forward is
                # ! scale * qd_ref. Without the factor the P term would have to
                # ! produce the whole motion and the arm would lag the plan.
                qd_cmd = np.clip(scale * qd_ref + p_gain * err, -vmax, vmax)
                rtde_c.speedJ(list(qd_cmd), self.args.joint_accel, dt_cmd)
                # Fresh-object writes: GIL-atomic snapshots for the UI thread.
                self.live_q[arm_i] = q_act
                self.live_err[arm_i] = err
                log['t'].append(t)
                log['q_ref'].append(q_ref)
                log['q_actual'].append(q_act)
                log['qd_cmd'].append(qd_cmd)
                log['wrench'].append(wrench)
                log['mode'].append(1 if holding else 0)
                log['speed_scale'].append(scale)
                # ! A holding arm has no end of its own: it stops when the
                # ! phase that asked for it says so, via the stop event.
                if not holding and t > self.t_end + BRAKE_TIME_S + END_SETTLE_S:
                    break   # braked and held long enough -- clean finish
                rtde_c.waitPeriod(t_cycle)
        except Exception as e:
            self.thread_error[arm_i] = str(e)
            self.stop_evt.set()   # one arm failing stops BOTH arms
        finally:
            try:
                rtde_c.speedStop()
            except Exception:
                pass
            self.thread_done[arm_i] = True

    def _read_wrench(self, arm_i: int, rtde_r) -> np.ndarray:
        """This arm's TCP wrench, published for the plots and the log.

        Args:
            arm_i (int): 0 = left, 1 = right.
            rtde_r: That arm's receive interface.

        Returns:
            np.ndarray: (6,) wrench in the arm's base frame, zeros if the
            read fails (a wrench is never worth aborting a run over).
        """
        try:
            wrench = np.asarray(rtde_r.getActualTCPForce())
        except Exception:
            wrench = np.zeros(6)
        self.live_wrench[arm_i] = wrench
        return wrench

    def _check_hold_wrench(self, arm_i: int, wrench):
        """Abort if the load on a holding arm jumps while it stands still.

        ! Measured as a CHANGE since the hold began, not as an absolute: the
        ! arm is already carrying the part's weight, and the press reaction is
        ! what we care about.

        Args:
            arm_i (int): 0 = left, 1 = right.
            wrench (np.ndarray): This cycle's wrench.

        Raises:
            RuntimeError: If the load has grown past the insertion guard.
        """
        if self.hold_wrench[arm_i] is None:
            self.hold_wrench[arm_i] = np.asarray(wrench).copy()
            return
        change = float(np.linalg.norm(
            np.asarray(wrench)[:3] - self.hold_wrench[arm_i][:3]))
        if change > self.ins_params.guard_force:
            raise RuntimeError(
                f'the holding {ARM_SIDES[arm_i]} arm took {change:.1f} N more '
                f'than when it started holding (guard '
                f'{self.ins_params.guard_force:.0f} N)')

    def _resume_bias(self, arm_i: int, t: float) -> np.ndarray:
        """The fading joint offset a resumed phase starts with.

        An insertion leaves both arms a little off the planned configuration:
        the inserting one found the hole somewhere else, the holding one has
        been standing still. Snapping back to the plan the instant tracking
        resumes would be a jerk, so the difference is blended out.

        Args:
            arm_i (int): 0 = left, 1 = right.
            t (float): Trajectory time [s].

        Returns:
            np.ndarray: (6,) offset to add to this arm's reference [rad].
        """
        blend = self.args.resume_blend
        if blend <= 0.0:
            return np.zeros(6)
        left = 1.0 - (t - self.resume_from) / blend
        if left <= 0.0:
            return np.zeros(6)
        return self.resume_offset[6 * arm_i:6 * arm_i + 6] * min(left, 1.0)

    def _tick_speed(self):
        """Read the execution-speed slider and apply it to a running phase.

        Read live every tick rather than through the slider's callback, which
        can be missed, and capped by what the velocity clamp allows. Applying
        it to the ONE shared clock is what keeps the arms together: they see
        the change on their next cycle, at most one control period apart.

        ! While the run is PAUSED the clock is left alone -- it is on its way
        ! to zero, or already there. The new speed is what the resume ramps
        ! back up to, so dragging the slider on a paused robot changes how fast
        ! it will go, not whether it stays still.

        ! Only a tracking phase is re-scaled. An insertion is force-controlled
        ! against the world and runs in real seconds; a release is a settle
        ! measured in real seconds too. A change made during either simply
        ! takes effect at the next tracking phase.
        """
        if self.scale_slider is None:
            return
        scale = min(float(self.scale_slider.value or 1.0), self.scale_cap)
        if abs(scale - self.speed_scale) <= 1e-3:
            return
        self.speed_scale = scale
        if self.state == 'tracking' and not self.paused:
            self.clock.set_scale(scale)
            self.scale_changes.append((round(self.clock.now(), 3), scale))
            self.get_logger().info(
                f'execution speed -> x{scale:.2f} at t={self.clock.now():.2f}s')

    def _tick_execute(self):
        """20 Hz supervision: mirror, gripper events, plots, finish detection."""
        # Space bar from the 3D window. The DPG handler only sees the key when
        # the control panel has focus, and the operator's eyes are usually on
        # the robot view -- so poll PyBullet's own keyboard here as well.
        # KEY_WAS_TRIGGERED latches the press edge, so one tap is one toggle
        # however long the key is held.
        keys = p.getKeyboardEvents(physicsClientId=pp.CLIENT)
        if keys.get(ord(' '), 0) & p.KEY_WAS_TRIGGERED:
            self.on_pause_toggle()
        self._tick_speed()
        # Live mirror. Outside tracking/moving the UI thread may poll the
        # receive interfaces itself (nobody else is using them then).
        if (self.state in ('connected', 'ready', 'done', 'aborted')
                and self.rtde_r[0] is not None):
            for i in range(2):
                try:
                    self.live_q[i] = np.asarray(self.rtde_r[i].getActualQ())
                except Exception:
                    pass
        if self.state != 'disconnected':
            self._plot_q12 = np.concatenate(self.live_q)

        status = f'state: {self.state}'
        if self.state == 'previewing':
            # Play the planned approach in the 3D view only; nothing moves.
            now = time.monotonic()
            self.preview_t += now - self._preview_wall
            self._preview_wall = now
            traj = self.approach_traj
            self._plot_q12 = traj.q12[traj.state_at(self.preview_t)]
            status = (f'PREVIEW approach t={self.preview_t:4.1f}/'
                      f'{traj.duration:.1f}s (nothing is moving)')
            if self.preview_t >= traj.duration:
                self.state = 'connected'
        elif self.state == 'tracking':
            t = self.clock.now()
            # ? Parts follow the trajectory's own clock. While the APPROACH
            # ? runs, active_traj is that generated motion, whose samples mean
            # ? nothing to the parts -- so they simply stay where they are.
            if self.active_traj is self.traj:
                self._viz_sample = self.traj.state_at(t)
            # Fire every gripper event whose time has come (<= 50 ms late,
            # ample against the 0.5 s hold the planner builds around closes).
            while (self.next_event_idx < len(self.run_events)
                   and self.run_events[self.next_event_idx].time <= t):
                self._fire_gripper_event(self.run_events[self.next_event_idx], t)
                self.next_event_idx += 1
            for i in range(2):
                self.err_plots[i].push([float(v) for v in self.live_err[i]],
                                       x=max(t, 0.0))
            nxt = (self.run_events[self.next_event_idx]
                   if self.next_event_idx < len(self.run_events) else None)
            # Where the pause ramp stands, in front of the usual line.
            if self.stop_reason:
                prefix = (f'STOPPED after {self.stop_reason} -- trace the '
                          f'outline, then DONE / Space ')
            elif self.paused:
                prefix = ('pausing.. ' if self.clock.ramping else
                          'PAUSED (Space to resume; STOP to release) ')
            else:
                prefix = 'resuming.. ' if self.clock.ramping else ''
            status = (f'{prefix}TRACKING {self.run_label} '
                      f't={t:7.1f}/{self.t_end:.1f}s'
                      + (f' | next: {nxt.kind} {ARM_SIDES[nxt.arm_index]} '
                         f'@{nxt.time:.1f}s' if nxt else ' | no more events'))
            if all(self.thread_done):
                self._phase_done()
                status = f'state: {self.state}'
        elif self.state == 'inserting':
            ins = self.insertion_of
            control = self.insertion
            t = self.clock.now()
            for i in range(2):
                self.err_plots[i].push([float(v) for v in self.live_err[i]],
                                       x=max(t, 0.0))
            status = (f'INSERTING {ins.child} -> {ins.parent}: '
                      f'{control.phase} '
                      f'{control.log.axial[-1] * 1000:.1f}/'
                      f'{control.depth * 1000:.1f} mm'
                      if control.log.axial else f'INSERTING {ins.child}')
            if all(self.thread_done):
                self._insertion_done()
                status = f'state: {self.state}'
        elif self.state == 'releasing':
            status = f'settling after {self.insertion_of.child}'
            if time.monotonic() >= self.release_until:
                self.stop_evt.set()
                for th in self.threads:
                    th.join(timeout=2.0)
                self._resume_after_insertion()
                status = f'state: {self.state}'
        elif self.state == 'held':
            status = (f'HELD after {self.insertion_of.child}: '
                      f'{self.insertion.outcome} -- Retry, Release & continue, '
                      f'or Abort')
        else:
            # Idle: keep the cutoff readout following the slider.
            self._refresh_cutoff_text()
        if self.last_run_folder:
            status += f' | saved -> {self.last_run_folder}'
        self.status_sep.set_text(status)
        self.approach_sep.set_text(self.approach_msg)
        self._tick_wrench()

    def _tick_wrench(self):
        """Stream both arms' wrench into the plots and refresh the role lines.

        Runs in every state once RTDE is up, so the operator can watch the
        forces while jogging or settling, not just during a run.
        """
        if self.state == 'disconnected' or not self.force_plots:
            return
        # Idle: nobody else is polling the receive interfaces, so do it here.
        if self.state in IDLE_STATES and self.rtde_r[0] is not None:
            for i in range(2):
                self._read_wrench(i, self.rtde_r[i])
        stamp = self.clock.now()
        for i in range(2):
            wrench = self.live_wrench[i]
            self.force_plots[i].push([float(v) for v in wrench[:3]], x=stamp)
            self.torque_plots[i].push([float(v) for v in wrench[3:]], x=stamp)
            role = self.live_role[i] or self.state
            self.role_seps[i].set_text(
                f'{ARM_SIDES[i]}: {role} | |F|='
                f'{np.linalg.norm(wrench[:3]):5.1f} N '
                f'|T|={np.linalg.norm(wrench[3:]):4.1f} Nm')
        if self.insertions:
            self.insert_sep.set_text(
                f'{self.insertion_msg} | done: {len(self.insertion_records)}'
                + (f' | LAST: {self.insertion_records[-1]["child"]} '
                   f'{self.insertion_records[-1]["outcome"]}'
                   if self.insertion_records else ''))

    def _fire_gripper_event(self, ev, t_now: float):
        """Send (or just log) one gripper command scheduled by the trajectory.

        Args:
            ev (GripperEvent): The event to fire.
            t_now (float): Current shared trajectory time [s] (for lateness
                bookkeeping in the run log).
        """
        pos = GRIPPER_CLOSE if ev.kind == 'close' else GRIPPER_OPEN
        # Mirror the commanded state in the 3D view (also under --no-gripper,
        # where it shows what WOULD have been sent). The COMMAND stays a full
        # close -- the real fingers stall on the part; only the drawn angle is
        # the fitted one, so the meshes stop on the part instead of in it.
        if ev.kind == 'close' and self.parts is not None:
            index = self.traj.state_at(ev.time)
            part = self.traj.attached[index].get(ev.robot)
            self.grip_viz_angle[ev.arm_index] = self._close_angle(
                ev.arm_index, part, index)
        else:
            self.grip_viz_angle[ev.arm_index] = pos
        record = {'kind': ev.kind, 'arm': ARM_SIDES[ev.arm_index],
                  'robot': ev.robot, 'planned_t': ev.time,
                  'fired_t': round(t_now, 3)}
        if self.robot is None:
            record['status'] = 'log-only'
        elif not self.robot.act_grippers[ev.arm_index].server_is_ready():
            record['status'] = 'SKIPPED-no-server'
            self.get_logger().warn(
                f'{ARM_SIDES[ev.arm_index]} gripper action server not ready -- '
                f'{ev.kind} @{ev.time:.2f}s skipped')
        else:
            # The result comes back a second or more later (the controller
            # waits out its stall timeout) and is judged in _on_gripper_result.
            self.robot.send_gripper_cmd(
                pos, GRIPPER_EFFORT, ev.arm_index,
                on_result=lambda status, result, rec=record:
                self._on_gripper_result(rec, status, result))
            record['status'] = 'sent'
        self.fired_events.append(record)
        self.event_sep.set_text(
            f'gripper: {ev.kind} {ARM_SIDES[ev.arm_index]} @{ev.time:.2f}s '
            f'({record["status"]}, {len(self.fired_events)}'
            f'/{len(self.run_events)})')
        self.get_logger().info(f'gripper event: {record}')

        # * Marking stop: the operator traces this part's outline on the sheet
        # * while the jaws still hold it there. The freeze is INSTANT -- the
        # * planner parks BOTH arms for its grasp_wait dwell after every close
        # * (>= 0.40 s on open-loop.json, reference speed ~0 at the close), so
        # * there is nothing to ramp down, and a ramp would carry the reference
        # * past the grasp pose. Resuming still ramps up (on_pause_toggle). An
        # * operator pause already in progress wins; only its label is lost.
        part = (self.traj.first_pick_part(ev)
                if self.stop_toggle is not None and bool(self.stop_toggle.value)
                else None)
        if part is not None and not self.paused:
            self._pause(0.0, f'grasp of {part} ({ARM_SIDES[ev.arm_index]})')
            self.get_logger().warn(
                f'STOPPED after {self.stop_reason} at t={t_now:.2f}s -- trace '
                f'the outline on the sheet, then press DONE (or Space)')

    def _on_gripper_result(self, record: dict, status, result):
        """Judge a gripper command by what its action reported back.

        Stalled fingers on a close met a part; fingers that reached the fully
        closed target met nothing. The verdict goes into the event record
        (and so into run_info.json) and onto the event line, and a MISSED
        grasp stops the run unless --no-grasp-abort says otherwise.

        ! Runs on the node's executor thread, the same one as the UI tick, so
        ! it may touch engine state directly.

        Args:
            record (dict): The fired-event record to fill in.
            status: action_msgs GoalStatus code, None if the goal was rejected.
            result: control_msgs GripperCommand.Result, None if rejected.
        """
        if result is None:
            record['result'] = 'REJECTED'
            self.get_logger().error(f'gripper goal REJECTED: {record}')
            return
        record.update(
            stalled=bool(result.stalled), reached_goal=bool(result.reached_goal),
            position=round(float(result.position), 4),
            width_mm=round(jaw_width_mm(float(result.position)), 1),
            goal_status=None if status is None else int(status))
        verdict = grasp_verdict(record['kind'], result.stalled, result.reached_goal,
                                float(result.position))
        record['result'] = verdict
        line = (f'{record["arm"]} {record["kind"]} @{record["planned_t"]:.2f}s: '
                f'{verdict} (jaw ~{record["width_mm"]:.0f} mm)')
        self.event_sep.set_text(f'gripper: {line}')
        if verdict in ('GRASPED', 'OPENED'):
            self.get_logger().info(f'gripper: {line}')
        else:
            self.get_logger().error(f'gripper: {line} -- {record}')
        if verdict == 'MISSED' and not self.args.no_grasp_abort:
            # ! Carrying on with an empty gripper is the one outcome worse
            # ! than stopping: every later action assumes the part is there.
            self._stop_tracking(f'missed grasp: {line}')

    # --- --- EXECUTE MODE: FINISH / ABORT / SAVE --- ---

    def _phase_done(self):
        """A tracking phase's threads ended: go on, or classify and save."""
        for th in self.threads:
            th.join(timeout=2.0)
        if [e for e in self.thread_error if e]:
            self._finish()
            return
        if self.phase_idx + 1 < len(self.phases):
            self._next_phase()
        else:
            self._finish()

    def _insertion_done(self):
        """An insertion's threads ended: release it, or ask the operator."""
        for th in self.threads:
            th.join(timeout=2.0)
        control, ins = self.insertion, self.insertion_of
        self.insertion_records.append({**control.record,
                                       'child': ins.child,
                                       'parent': ins.parent,
                                       'arm': ARM_SIDES[ins.arm_index],
                                       't_start': ins.t_start,
                                       't_open': ins.t_open,
                                       'planned_depth_m': ins.depth_m})
        self.insertion_logs[f'ins{len(self.insertion_records) - 1}'] = \
            control.log.arrays()
        errors = [e for e in self.thread_error if e]
        if errors:
            self.get_logger().error(f'insertion aborted: {"; ".join(errors)}')
            self._finish()
            return
        if control.record.get('seated'):
            self.get_logger().info(
                f'{ins.child} seated ({control.outcome}), '
                f'{control.record["short_by_mm"]:.1f} mm short of the plan')
            self._next_phase()
            return
        # ! Not seated. Both arms hold where they are and the operator picks:
        # ! nothing here decides on its own to open a gripper on a part that
        # ! is not in its joint.
        self.get_logger().error(
            f'{ins.child} did NOT seat: {control.outcome} '
            f'({control.record.get("short_by_mm", 0):.1f} mm short) -- '
            f'holding both arms, waiting for the operator')
        self._hold_both_arms()
        self.state = 'held'

    def _hold_both_arms(self):
        """Keep both arms exactly where they are until told otherwise."""
        self.stop_evt = threading.Event()
        self.hold_arm = [True, True]
        self.hold_q12 = self._live_q12()
        self.hold_wrench = [None, None]
        self.run_events = []
        self.next_event_idx = 0
        self.thread_done = [False, False]
        self.thread_error = [None, None]
        self.threads = [threading.Thread(target=self._arm_tracker, args=(i,),
                                         daemon=True) for i in range(2)]
        for th in self.threads:
            th.start()

    def _stop_hold(self):
        """End the holding threads started by `_hold_both_arms`."""
        self.stop_evt.set()
        for th in self.threads:
            th.join(timeout=2.0)

    def _resume_after_insertion(self):
        """Pick the trajectory back up where the mate left the arms.

        The arms are wherever the insertion put them, which is not exactly the
        planned configuration. That difference becomes the fading bias the
        tracker blends out over `--resume-blend`.
        """
        ins = self.insertion_of
        planned = self.traj.q12[self.traj.state_at(ins.t_open)]
        self.resume_offset = self._live_q12() - planned
        self.resume_from = ins.t_open
        self.get_logger().info(
            f'resuming at t={ins.t_open:.2f}s, blending out a '
            f'{np.abs(self.resume_offset).max():.4f} rad offset over '
            f'{self.args.resume_blend:.1f}s')
        self.insertion = None
        self._next_phase()

    def on_retry_insertion(self):
        """Operator: lift clear and search again."""
        if self.state != 'held':
            self.get_logger().warn(f'Retry ignored in state {self.state}')
            return
        self.get_logger().info('operator: retrying the insertion')
        self._stop_hold()
        control, ins = self.insertion, self.insertion_of
        control.retry(0.0)
        self.hold_arm = [False, False]
        self.hold_arm[ins.holder_index] = True
        self.hold_q12 = self._live_q12()
        self.hold_wrench = [None, None]
        self.thread_done = [False, False]
        self.thread_error = [None, None]
        self.stop_evt = threading.Event()
        # ! Always real time (scale 1), whatever the speed slider says: the
        # ! insertion is force-controlled against the world, and its contact
        # ! thresholds, stall timer and budget are all in real seconds.
        self.clock.start(ins.t_start, START_PREROLL_S, 1.0)
        self.threads = [
            threading.Thread(target=self._arm_tracker,
                             args=(ins.holder_index,), daemon=True),
            threading.Thread(target=self._insertion_thread,
                             args=(ins,), daemon=True)]
        self.state = 'inserting'
        for th in self.threads:
            th.start()

    def on_release_and_continue(self):
        """Operator: open the gripper anyway and carry on with the plan."""
        if self.state != 'held':
            self.get_logger().warn(f'Release ignored in state {self.state}')
            return
        self.get_logger().warn(
            f'operator: releasing {self.insertion_of.child} UNSEATED and '
            f'continuing -- everything downstream is now off-plan')
        if self.insertion_records:
            self.insertion_records[-1]['operator'] = 'released_unseated'
        self._stop_hold()
        self.phase_idx += 1          # the release phase we were about to run
        self._launch_release(self.insertion_of)

    def on_abort_run(self):
        """Operator: stop here and save what happened."""
        if self.state != 'held':
            self.get_logger().warn(f'Abort ignored in state {self.state}')
            return
        if self.insertion_records:
            self.insertion_records[-1]['operator'] = 'aborted'
        self._stop_hold()
        self.state = 'aborted'
        self._save_run_log(f'aborted: insertion {self.insertion.outcome}')

    def _finish(self):
        """Both tracker threads ended on their own: classify and save."""
        for th in self.threads:
            th.join(timeout=2.0)
        errors = [e for e in self.thread_error if e]
        if errors:
            self.state = 'aborted'
            self._save_run_log('aborted: ' + '; '.join(errors))
            return
        self.state = 'done'
        self._save_run_log('done')
        if self.run_label == 'approach':
            # The arms should now be sitting on the trajectory's first sample;
            # confirm it right away so START becomes available (or not).
            self.on_check_start_pose()

    def _stop_tracking(self, reason: str):
        """Stop a live run from outside the tracker threads (STOP/close/^C).

        Safe to call in any state and more than once; only acts when a run
        is actually in progress. Always ends with speedStop on both arms.

        Args:
            reason (str): Human-readable cause, recorded in the run log.
        """
        if self.state not in RUNNING_STATES:
            return
        self.get_logger().warn(f'stopping tracking: {reason}')
        self.stop_evt.set()
        for th in self.threads:
            th.join(timeout=2.0)
            if th.is_alive():
                self.get_logger().error('a tracker thread did not stop in 2 s')
        # An insertion also owns a tool offset, which has to go back.
        if self.insertion is not None:
            try:
                self.insertion.stop()
            except Exception:
                pass
            self.insertion = None
        for c in self.rtde_c:
            # Belt and braces -- the threads' finally already speedStopped.
            try:
                c.speedStop()
            except Exception:
                pass
        self.state = 'aborted'
        self._save_run_log(f'aborted: {reason}')

    def _save_run_log(self, outcome: str):
        """Write log.npz + run_info.json + plots.png next to the input json.

        Args:
            outcome (str): 'done' or 'aborted: <reason>'.
        """
        if self._log_saved or self.exec_log is None:
            return
        self._log_saved = True
        if not any(self.exec_log[i]['t'] for i in range(2)):
            self.get_logger().info(f'run {outcome} -- no samples, nothing saved')
            return
        stem = os.path.splitext(os.path.basename(self.traj.source_path))[0]
        folder = os.path.join(
            os.path.dirname(self.traj.source_path),
            f'{stem}-{self.run_label}-{datetime.now():%Y%m%d-%H%M%S}')
        os.makedirs(folder, exist_ok=True)

        arrays = {}
        for i, side in enumerate(ARM_SIDES):
            for key, rows in self.exec_log[i].items():
                arrays[f'{side}_{key}'] = np.asarray(rows)
        # Each insertion's own full-rate record, keyed ins0_, ins1_, ...
        for name, log in self.insertion_logs.items():
            arrays.update({f'{name}_{k}': v for k, v in log.items()})
        np.savez(os.path.join(folder, 'log.npz'), **arrays)
        if self.insertion_records:
            with open(os.path.join(folder, 'insertions.json'), 'w') as f:
                json.dump(self.insertion_records, f, indent=2)

        info = {
            'outcome': outcome,
            'what_ran': self.run_label,
            'traj_json': self.traj.source_path,
            'args': vars(self.args),
            'started_preroll_s': START_PREROLL_S,
            'speed_scale_at_start': self.speed_scale,
            'speed_scale_changes': self.scale_changes,
            'speed_scale_cap': self.scale_cap,
            'pauses': self.pauses,
            'pause_ramp_s': PAUSE_RAMP_S,
            'end_sample': self.end_idx,
            'end_time_s': self.t_end,
            'n_samples_in_file': self.traj.n_samples,
            'start_deltas': [d.tolist() for d in (self.start_deltas or [])],
            'events_fired': self.fired_events,
            'ft_zeros': self.ft_zeros,
            'thread_errors': self.thread_error,
            'n_cycles': [len(self.exec_log[i]['t']) for i in range(2)],
            'insertions': self.insertion_records,
        }
        with open(os.path.join(folder, 'run_info.json'), 'w') as f:
            json.dump(info, f, indent=2)
        try:
            self._save_plots_png(folder)
        except Exception as e:
            self.get_logger().warn(f'plots.png generation failed: {e}')
        try:
            self._save_insertion_wrench(folder)
        except Exception as e:
            self.get_logger().warn(f'insertion wrench log failed: {e}')
        self.last_run_folder = folder
        self.get_logger().info(f'run log ({outcome}) saved -> {folder}')

    def _save_plots_png(self, folder: str):
        """Render the whole run as one static image: q, error, qd per arm.

        Args:
            folder (str): The run's output folder (where log.npz lives).
        """
        rows = 4 if any(self.exec_log[i]['wrench'] for i in range(2)) else 3
        fig = Figure(figsize=(14, 4.7 * rows))
        grid = fig.add_gridspec(rows, 2, hspace=0.35, wspace=0.2)
        for i, side in enumerate(ARM_SIDES):
            log = self.exec_log[i]
            t = np.asarray(log['t'])
            if not len(t):
                continue
            q_ref, q_act = np.asarray(log['q_ref']), np.asarray(log['q_actual'])
            for row, (title, unit) in enumerate((
                    ('joints: ref (dashed) vs actual', 'q [rad]'),
                    ('tracking error', 'q_ref - q_actual [rad]'),
                    ('commanded speed', 'qd_cmd [rad/s]'),
                    ('TCP wrench (base frame)', 'force [N] / torque [Nm]'))[:rows]):
                ax = fig.add_subplot(grid[row, i])
                labels = JOINT_LABELS_12[6 * i:6 * i + 6]
                if row == 0:
                    for k, lb in enumerate(labels):
                        line, = ax.plot(t, q_act[:, k], label=lb)
                        ax.plot(t, q_ref[:, k], '--', color=line.get_color(),
                                linewidth=0.8)
                elif row == 1:
                    ax.plot(t, q_ref - q_act, label=labels)
                elif row == 2:
                    ax.plot(t, np.asarray(log['qd_cmd']), label=labels)
                else:
                    wrench = np.asarray(log['wrench'])
                    ax.plot(t, wrench[:, :3], label=['fx', 'fy', 'fz'])
                    ax.plot(t, wrench[:, 3:], '--', linewidth=0.8,
                            label=['tx', 'ty', 'tz'])
                    # Shade the stretches where this arm was holding still.
                    self._shade_holds(ax, t, np.asarray(log['mode']))
                # Gripper events of this arm as vertical markers.
                for rec in self.fired_events:
                    if rec['arm'] == side:
                        ax.axvline(rec['planned_t'], color='k', alpha=0.25,
                                   linestyle=':' if rec['kind'] == 'open' else '-')
                ax.set_title(f'{side} {title}')
                ax.set_ylabel(unit)
                ax.grid(alpha=0.3)
                ax.legend(loc='upper right', fontsize=7, ncol=3)
                if row == rows - 1:
                    ax.set_xlabel('trajectory time [s]')
        fig.suptitle(f'{os.path.basename(self.traj.source_path)} '
                     f'[{self.run_label}]  ({datetime.now():%Y-%m-%d %H:%M})',
                     fontsize=13)
        fig.savefig(os.path.join(folder, 'plots.png'), dpi=110,
                    bbox_inches='tight')
        for index, record in enumerate(self.insertion_records):
            try:
                self._save_insertion_png(folder, index, record)
            except Exception as e:
                self.get_logger().warn(f'insertion_{index}.png failed: {e}')

    @staticmethod
    def _shade_holds(ax, t, mode):
        """Shade the spans where an arm was holding rather than tracking.

        Args:
            ax: The axes to shade.
            t (np.ndarray): Sample times.
            mode (np.ndarray): 1 while holding, 0 while tracking.
        """
        if not len(mode):
            return
        edges = np.flatnonzero(np.diff(np.r_[0, mode.astype(int), 0]))
        for start, end in zip(edges[::2], edges[1::2]):
            ax.axvspan(t[start], t[min(end, len(t) - 1)], color='tab:orange',
                       alpha=0.12, lw=0)

    def _save_insertion_png(self, folder: str, index: int, record: dict):
        """One figure per insertion: depth, forces, search and the phases.

        Args:
            folder (str): The run's output folder.
            index (int): Which insertion this is.
            record (dict): Its summary record.
        """
        log = self.insertion_logs.get(f'ins{index}')
        if not log or not len(log['t']):
            return
        t = log['t']
        fig = Figure(figsize=(12, 11))
        grid = fig.add_gridspec(4, 1, hspace=0.35)

        ax = fig.add_subplot(grid[0])
        ax.plot(t, log['axial'] * 1000, label='travelled along the axis')
        ax.axhline(record['planned_depth_mm'], color='k', ls='--', lw=0.8,
                   label='seated depth')
        ax.set_ylabel('[mm]')
        ax.set_title(f'{record["child"]} -> {record["parent"]} '
                     f'({record["arm"]} arm): {record["outcome"]}')

        ax = fig.add_subplot(grid[1])
        ax.plot(t, log['wrench_lp'][:, :3], label=['fx', 'fy', 'fz'])
        ax.axhline(record['params']['push_force'], color='k', ls=':', lw=0.8)
        ax.axhline(-record['params']['push_force'], color='k', ls=':', lw=0.8)
        ax.set_ylabel('force, base frame [N]')

        ax = fig.add_subplot(grid[2])
        ax.plot(t, log['search_r'] * 1000, label='search radius')
        ax.plot(t, log['lateral_err'] * 1000, label='lateral lag')
        ax.set_ylabel('[mm]')

        ax = fig.add_subplot(grid[3])
        ax.step(t, log['phase'], where='post')
        ax.set_yticks(range(len(INSERTION_PHASES)))
        ax.set_yticklabels(INSERTION_PHASES)
        ax.set_xlabel('time since the insertion started [s]')

        for axis in fig.axes:
            axis.grid(alpha=0.3)
            if axis.get_legend_handles_labels()[1]:
                axis.legend(loc='upper right', fontsize=8)
        fig.savefig(os.path.join(folder, f'insertion_{index}.png'), dpi=110,
                    bbox_inches='tight')

    def _save_insertion_wrench(self, folder: str) -> int:
        """Write both arms' force record around every mate: json plus figures.

        One record per mate in the plan, covering the window from the funnel
        mouth -- where the straight insertion approach begins, i.e. the
        pre-insertion pose -- to the gripper-open that releases the part, with
        `WRENCH_LOG_PAD_S` of quiet on either side for the baseline. Both arms
        are in it: the inserting arm feels the joint going together, and the
        arm holding the parent part feels the same push through its own grasp.

        The samples are sliced out of the tracker's own full-rate log, so this
        costs nothing during the run and carries exactly what the arms
        measured. Forces are `getActualTCPForce`, i.e. the wrench at the
        pendant's TCP expressed in that arm's BASE frame.

        ! The signs have never been checked against Cindy (see the handover
        ! note). Treat the numbers as relative until somebody pushes each tool
        ! along base +x/+y/+z by hand and writes the result down.

        Args:
            folder (str): The run's output folder.

        Returns:
            int: How many mates were written.
        """
        if self.wrench_toggle is None or not bool(self.wrench_toggle.value):
            return 0
        mates = self._mates()
        if not mates:
            # Say why rather than writing nothing silently: a ticked checkbox
            # that produces no file looks like a bug from the outside.
            self.get_logger().warn(
                'insertion wrench log: nothing written -- '
                + ('there is no --plan-json, so which gripper-open is a mate '
                   'is unknown' if not self.args.plan_json else
                   'the plan has no mate inside this trajectory'))
            return 0
        times = [np.asarray(self.exec_log[i]['t'], dtype=float)
                 for i in range(2)]
        wrenches = [np.asarray(self.exec_log[i]['wrench'], dtype=float)
                    for i in range(2)]
        if any(w.ndim != 2 or w.shape[0] != t.shape[0]
               for w, t in zip(wrenches, times)):
            self.get_logger().warn('insertion wrench log skipped: the run has '
                                   'no matching force samples')
            return 0

        records = []
        for index, ins in enumerate(mates):
            lo = ins.t_start - WRENCH_LOG_PAD_S
            hi = ins.t_open + WRENCH_LOG_PAD_S
            arms = {}
            for i, side in enumerate(ARM_SIDES):
                keep = (times[i] >= lo) & (times[i] <= hi)
                if not keep.any():
                    continue
                arms[side] = {
                    'role': ('inserting' if i == ins.arm_index else
                             'holding' if i == ins.holder_index else 'other'),
                    't_s': np.round(times[i][keep], 4).tolist(),
                    'force_N': np.round(wrenches[i][keep, :3], 4).tolist(),
                    'torque_Nm': np.round(wrenches[i][keep, 3:], 4).tolist()}
            if not arms:
                # The run never reached this mate (a cutoff, a STOP, an abort).
                continue
            records.append({
                'index': index,
                'child': ins.child,
                'parent': ins.parent,
                'inserting_arm': ARM_SIDES[ins.arm_index],
                'holding_arm': ARM_SIDES[ins.holder_index],
                'pre_insertion_t_s': round(ins.t_start, 3),
                'release_t_s': round(ins.t_open, 3),
                'funnel_depth_mm': round(ins.depth_m * 1000.0, 2),
                'pad_s': WRENCH_LOG_PAD_S,
                'ran_compliant': any(r.get('child') == ins.child
                                     for r in self.insertion_records),
                'arms': arms})

        if not records:
            return 0
        payload = {
            'schema': 'insertion-wrench-v1',
            'traj_json': self.traj.source_path,
            'plan_json': self.args.plan_json,
            'what_ran': self.run_label,
            'frame': 'TCP wrench in each arm base frame (RTDE '
                     'getActualTCPForce); signs unverified on this cell',
            'rate_hz': self.args.frequency,
            'ft_zeros': self.ft_zeros,
            'time_base': "trajectory seconds, the same clock as log.npz's *_t",
            'insertions': records}
        with open(os.path.join(folder, 'insertion_wrench.json'), 'w') as f:
            json.dump(payload, f, indent=2)
        for record in records:
            try:
                self._save_wrench_png(folder, record)
            except Exception as e:
                self.get_logger().warn(
                    f'wrench figure for {record["child"]} failed: {e}')
        self.get_logger().info(
            f'insertion wrench log: {len(records)} mate(s) -> '
            f'insertion_wrench.json + insertion_wrench_*.png')
        return len(records)

    def _save_wrench_png(self, folder: str, record: dict):
        """One figure per mate: both arms' force and torque over its window.

        Force and torque are split because N and Nm cannot share an axis, the
        same convention as the live Wrench window. The funnel mouth and the
        release are drawn as vertical lines, so what happened between them is
        readable at a glance.

        Args:
            folder (str): The run's output folder.
            record (dict): One entry of `insertion_wrench.json`.
        """
        fig = Figure(figsize=(12, 7))
        grid = fig.add_gridspec(2, 1, hspace=0.28)
        axes = [fig.add_subplot(grid[0]), fig.add_subplot(grid[1])]
        for i, side in enumerate(ARM_SIDES):
            arm = record['arms'].get(side)
            if arm is None:
                continue
            t = np.asarray(arm['t_s'])
            colors = [tuple(c / 255.0 for c in rgb)
                      for rgb in WRENCH_PALETTE[3 * i:3 * i + 3]]
            for ax, key, unit in ((axes[0], 'force_N', 'force [N]'),
                                  (axes[1], 'torque_Nm', 'torque [Nm]')):
                values = np.asarray(arm[key])
                for axis in range(3):
                    ax.plot(t, values[:, axis], color=colors[axis], lw=1.0,
                            label=f'{side} {arm["role"]} '
                                  f'{AXIS_LABELS[axis]}')
                ax.set_ylabel(unit)
        for ax in axes:
            ax.axvline(record['pre_insertion_t_s'], color='k', ls='--', lw=0.8)
            ax.axvline(record['release_t_s'], color='k', ls=':', lw=0.8)
            ax.grid(alpha=0.3)
            ax.legend(loc='upper left', fontsize=7, ncol=2)
        axes[0].set_title(
            f'{record["child"]} -> {record["parent"]}: '
            f'{record["inserting_arm"]} arm inserts, '
            f'{record["holding_arm"]} arm holds  |  funnel '
            f'{record["funnel_depth_mm"]:.0f} mm  |  dashed = pre-insertion, '
            f'dotted = release')
        axes[1].set_xlabel('trajectory time [s]')
        fig.savefig(
            os.path.join(folder,
                         f'insertion_wrench_{record["index"]}_'
                         f'{record["child"]}.png'),
            dpi=110, bbox_inches='tight')

    # --- --- SHUTDOWN --- ---

    def destroy_node(self):
        # ! In 'held' the arms are parked by their own threads waiting for the
        # ! operator, so there is no run for _stop_tracking to end -- those
        # ! threads still have to be told to let go.
        if self.state == 'held':
            self.get_logger().warn('shutting down while holding after a failed '
                                   'insertion -- releasing the arms')
            try:
                self._stop_hold()
            except Exception:
                pass
            self.state = 'aborted'
            self._save_run_log('aborted: shutdown while held')
        self._stop_tracking('node shutdown')
        for c in self.rtde_c:
            # End the uploaded RTDE control script so the arm returns to
            # normal; harmless if never connected or already stopped.
            try:
                c.stopScript()
            except Exception:
                pass
        if _common._global_backend is not None:
            try:
                _common._global_backend.shutdown()
            except Exception as e:
                self.get_logger().warn(f'UI backend shutdown error: {e}')
            _common._global_backend = None
        super().destroy_node()


# --- --- MAIN --- ---

def main(args=None):
    cli = argparse.ArgumentParser(
        description='Preview or execute an open-loop dual-arm assembly '
                    'trajectory (schema assembly-open-loop-json-v2).')
    cli.add_argument('traj_json', help='path to the trajectory json')
    cli.add_argument('--execute', action='store_true',
                     help='run on the real arms over ur_rtde (default: preview)')
    cli.add_argument('--swap-arms', action='store_true',
                     help='map the file\'s first robot to the RIGHT arm')
    cli.add_argument('--robot-name', default='/a200_0806',
                     help='ROS namespace for the gripper actions')
    cli.add_argument('--left-ip', default='192.168.131.40')
    cli.add_argument('--right-ip', default='192.168.131.41')
    cli.add_argument('--frequency', type=float, default=125.0,
                     help='speedJ tracking loop rate [Hz]')
    cli.add_argument('--p-gain', type=float, nargs=2, default=[1.0, 1.0],
                     metavar=('LEFT', 'RIGHT'),
                     help='per-arm path tracking P gain')
    cli.add_argument('--joint-accel', type=float, default=3.0,
                     help='speedJ acceleration [rad/s^2]')
    cli.add_argument('--max-joint-vel', type=float, default=2.0,
                     help='commanded speed clamp [rad/s]')
    cli.add_argument('--start-tol', type=float, default=0.05,
                     help='per-joint start pose tolerance [rad]')
    cli.add_argument('--err-abort', type=float, default=0.25,
                     help='per-joint tracking error that aborts the run [rad]')
    cli.add_argument('--no-grasp-abort', action='store_true',
                     help='keep running after a close whose fingers met nothing '
                          '(default: a MISSED grasp stops both arms)')
    cli.add_argument('--no-gripper', action='store_true',
                     help='log gripper events instead of sending them')
    cli.add_argument('--stop-after-grasp', action='store_true',
                     help='freeze the run right after each part is picked off the '
                          'table for the FIRST time, so its outline can be traced '
                          'on the layout sheet where the robot really grasped it; '
                          'DONE (or Space) continues. Handover receives and '
                          're-grasps do not stop it. Also a live checkbox in the '
                          'panel.')
    cli.add_argument('--approach-vel', type=float, default=0.25,
                     help='peak joint speed of the planned approach [rad/s]')
    cli.add_argument('--no-table', action='store_true',
                     help='drop the placeholder work table from the '
                          'planning scene')
    cli.add_argument('--table-top-z', type=float, default=TABLE_TOP_Z,
                     help='height of the placeholder table top [m] '
                          '(default: half the arm base height)')
    cli.add_argument('--layout-json', default=None, metavar='PATH',
                     help='measured cell layout from pickup_calib: the approach then '
                          'avoids the surveyed table and the real parts instead of the '
                          'placeholder table, and the parts are drawn in the 3D view')
    cli.add_argument('--plan-json', default=None, metavar='PATH',
                     help="the planner's plan.json, so the part visualization knows "
                          'which gripper-open mates a part onto another (without it a '
                          'mated part is left where it was released)')
    # --- compliant insertion (off unless asked for) ---
    cli.add_argument('--insertions', action='store_true',
                     help='hand the plan\'s mates to the force-controlled insertion '
                          'skill instead of replaying them open-loop; needs '
                          '--plan-json. Without this flag the run is exactly the '
                          'speedJ tracking it has always been.')
    cli.add_argument('--ins-push-force', type=float, default=None,
                     help='force held along the mate axis while searching and '
                          'pressing [N]')
    cli.add_argument('--ins-contact-force', type=float, default=None,
                     help='force above which the part counts as touching [N]')
    cli.add_argument('--ins-guard-force', type=float, default=None,
                     help='any force this large aborts the insertion at once [N]')
    cli.add_argument('--ins-search-radius', type=float, default=None,
                     help='give up searching past this radius [mm]')
    cli.add_argument('--ins-search-pitch', type=float, default=None,
                     help='radial growth per spiral turn [mm]; keep it under the '
                          "joint's clearance or the search steps over the hole")
    cli.add_argument('--ins-approach-speed', type=float, default=None,
                     help='tool speed down the funnel before contact [mm/s]')
    cli.add_argument('--ins-insert-speed', type=float, default=None,
                     help='tool speed while pressing in after contact [mm/s]')
    cli.add_argument('--ins-budget', type=float, default=None,
                     help='give up on one insertion after this long [s]')
    cli.add_argument('--no-zero-ft', action='store_true',
                     help='do NOT zero both force/torque sensors at Connect '
                          'RTDE (the default is to zero them, so every force '
                          'the run reports starts from the parked arms). The '
                          '"Zero force sensors" button does it again on demand.')
    cli.add_argument('--no-wrench-log', action='store_true',
                     help='start with the "log insertion wrench" checkbox OFF. '
                          'On (the default) every mate in --plan-json gets both '
                          'arms\' force/torque from its funnel mouth to the '
                          'release written to insertion_wrench.json plus a png.')
    cli.add_argument('--ins-min-depth', type=float, default=10.0,
                     help='ignore mates whose straight approach is shorter than '
                          'this [mm] -- they stay on the tracker (default: 10)')
    cli.add_argument('--release-wait', type=float, default=1.0,
                     help='hold still this long after opening the gripper on a '
                          'seated part, before the trajectory resumes [s]')
    cli.add_argument('--resume-blend', type=float, default=2.0,
                     help='blend the joint offset left by an insertion back out '
                          'over this long when tracking resumes [s]')
    cli.add_argument('--end-sample', type=int, default=0,
                     help='initial value of the "run until sample" slider '
                          '(1-based; 0 = the whole trajectory)')
    parsed = cli.parse_args(remove_ros_args(sys.argv if args is None else args)[1:])

    if parsed.execute and RTDEControlInterface is None:
        print('ur_rtde is not installed in this env -- run:\n'
              '    source /home/su/ros2_ws/venv/bin/activate && pip install ur_rtde')
        return 1

    rclpy.init(args=args)
    node = OpenLoopEngine(parsed)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C, a closed UI window, or an external SIGTERM -- all normal.
        pass
    finally:
        # ! Every exit path funnels through here: a live run is always
        # ! speedStopped and its log saved before the process ends.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
