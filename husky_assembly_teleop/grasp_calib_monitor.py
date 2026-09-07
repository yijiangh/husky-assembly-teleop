"""Standalone recorder for the Robotiq gripper grasp slip-calibration experiment.

* Experiment: on dual-arm Cindy (/a200_0806), the LEFT arm holds a box piece in
* the Robotiq gripper and stays fixed; the operator free-drives the RIGHT arm so
* a pointy punch pushes on chosen points of the box, until the piece slips in
* the gripper. This tool only READS the robot (wrench + joint states) -- both
* pendants stay in LOCAL mode the whole time, no ROS commanding at all.
* Reading keeps working in local mode because the UR driver's joint-state and
* force-torque broadcasters are "consistent controllers" that never stop.

Workflow in the UI:
    1. Type an experiment name, click "Start recording".
    2. Recording begins immediately: one aligned sample per 20 Hz tick with
       raw wrench, joint config, and analytic FK (tool0 in the arm base
       frame) for BOTH arms, all live-plotted. Wrench values are RAW sensor
       readings; per-arm "Zero FT" buttons call the UR zero_ftsensor service
       (needs that arm's driver running, and may be ignored in local pendant
       mode -- the quiet seconds before pushing always work as a fallback
       reference for offset removal in post-processing).
    3. Walk to the pendant, free-drive the right arm and push. When done,
       click "Stop & save" (or "Discard" to throw the take away) -- stopping
       is always manual.
    4. Each take is saved in its own timestamped subfolder of the Insync
       experiment drive (see OUTPUT_ROOT below): record.json with all the
       data, plus plots.png -- a matplotlib snapshot of every plot over the
       whole take.

The wrench plots live in one window per arm (left | right), with the force
pair and the torque pair each sharing one axis scale so the arms stay
directly comparable. They and the right-arm joint stream plot run
continuously from launch, whether or not a recording is active; only the EE
pose plots are per-take. A PyBullet 3D viewer shows the calibrated dual-arm
husky model mirroring the live joint states while free-driving.

Run (after colcon build):
    ros2 run husky_assembly_teleop grasp_calib_monitor
    ros2 run husky_assembly_teleop grasp_calib_monitor --check   # hardware preflight
"""

import json
import os
import sys
import time
from datetime import datetime

import numpy as np
import pybullet as p
import pybullet_planning as pp
import rclpy
# Agg-only figure (no pyplot): safe to render headless from inside the node.
from matplotlib.figure import Figure
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation as R

from husky_assembly_teleop import EXPERIMENT_DATA_DIRECTORY
from husky_assembly_teleop import common as _common
from husky_assembly_teleop.common import (Button, HistoryPlot, LiveMultiPlot,
                                          Separator, TextInput,
                                          load_robot,
                                          HUSKY_DUAL_UR5e_JOINT_NAMES)
from husky_assembly_teleop.husky_robot import (ARM_JOINT_NAMES,
                                               HuskyRobotInterface,
                                               UR5e_HOME_STATE)
from husky_assembly_teleop.mocap_experiment import sanitize_slug
from husky_assembly_teleop.ui_backend import make_backend
from husky_assembly_tamp.keyframe import ssik_inprocess

# --- --- CONSTANTS --- ---

ROBOT_NAME = '/a200_0806'          # Cindy, the dual-arm husky
ARM_SIDES = ('left', 'right')      # index 0 = left, 1 = right (repo convention)
TICK_PERIOD_S = 0.05               # 20 Hz UI + sampling tick, same as husky_monitor
PLOT_HISTORY = 8192                # per-take plots: full take, ~7 min at 20 Hz
LIVE_STREAM_HISTORY = 600          # always-on plots: scrolling ~30 s window
SCHEMA_VERSION = 1
OUTPUT_ROOT = os.path.join(EXPERIMENT_DATA_DIRECTORY, 'robotiq_grasp_calibration')

# Left arm = reds, right arm = greens (x, y, z per arm). Same color families as
# husky_world's servoing tracker, copied here so this script never imports
# husky_world (which drags in pybullet scene code this recorder doesn't need).
LEFT_RGB = [(240, 130, 120), (225, 70, 55), (150, 20, 20)]
RIGHT_RGB = [(150, 220, 140), (70, 180, 80), (20, 130, 40)]
PLOT_PALETTE = LEFT_RGB + RIGHT_RGB
AXIS_LABELS = ['x', 'y', 'z']
XYZ_LABELS = ['L x', 'L y', 'L z', 'R x', 'R y', 'R z']
EULER_LABELS = ['L r', 'L p', 'L y', 'R r', 'R p', 'R y']
JOINT_LABELS = ['pan', 'lift', 'elbow', 'w1', 'w2', 'w3']

# * Topics read by the --check preflight (same ones HuskyRobotInterface
# * subscribes to). 100 Hz rate-limited streams from the husky side.
CHECK_TOPICS = [
    (f'{ROBOT_NAME}/left_ur5e/rate_limiter/ft_sensor_wrench', WrenchStamped, 'left wrench'),
    (f'{ROBOT_NAME}/right_ur5e/rate_limiter/ft_sensor_wrench', WrenchStamped, 'right wrench'),
    (f'{ROBOT_NAME}/left_ur5e/rate_limiter/joint_states', JointState, 'left joints'),
    (f'{ROBOT_NAME}/right_ur5e/rate_limiter/joint_states', JointState, 'right joints'),
]


def ee_pose_from_q(arm: str, q) -> tuple:
    """Forward kinematics of one arm's tool0 flange from a joint configuration.

    Args:
        arm (str): "left" or "right".
        q: 6 joint angles in radians (driver order = kinematic order).

    Returns:
        tuple: ``(pos_m, quat_xyzw, euler_xyz_deg)`` -- position in meters,
        quaternion, and static-xyz euler angles in degrees (URDF RPY
        convention), all of tool0 expressed in that arm's
        ``*_ur_arm_base_link`` frame.
    """
    T = ssik_inprocess.fk(arm, q)
    rot = R.from_matrix(T[:3, :3])
    return T[:3, 3], rot.as_quat(), rot.as_euler('xyz', degrees=True)


class GraspCalibRecorder(Node):
    """Read-only DPG recorder node: wrench + joints + FK of both arms per tick."""

    def __init__(self):
        super().__init__('grasp_calib_monitor')

        # ! These two attributes are read by HuskyRobotInterface's ctor via
        # ! getattr -- they must be set BEFORE constructing it. 0 skips the
        # ! SetIO / list_controllers service waits, which would each block
        # ! 2.5 s per arm (those controllers may not even run in local mode).
        self.CONNECT_IO_SERVICES = 0
        self.LIST_CONTROLLER_SERVICES = 0

        # Reuse the existing interface purely as a read-only stream cache:
        # arm_ft_sensor[i] (raw [fx,fy,fz,tx,ty,tz]) and arm_joint_pose[i]
        # (reordered by ARM_JOINT_NAMES). With every connect flag off the ctor
        # creates only subscriptions, so it returns immediately. Its state
        # fields are class-level lists -- fine with one instance per process.
        self.robot = HuskyRobotInterface(
            self, name=ROBOT_NAME, use_odom=False, connect_arm=False,
            connect_gripper=False, dual_arm=True)

        # * Zero-FT service clients. The interface only creates these under
        # * connect_compliant_controller, which drags in force-mode/controller
        # * clients and their 2.5 s waits we don't want. Creating just the two
        # * Trigger clients here (no wait) and handing them to the interface
        # * keeps its existing zero_ft_sensor() working too.
        self.robot.zero_ft_sensor_client = [
            self.create_client(
                Trigger,
                f'{ROBOT_NAME}/{side}_ur5e/io_and_status_controller/zero_ftsensor')
            for side in ARM_SIDES]

        # Warm up the FK artifacts once so a missing ssik install fails right
        # here with its pip-install hint, not in the middle of a recording.
        for side in ARM_SIDES:
            ee_pose_from_q(side, UR5e_HOME_STATE)

        # * PyBullet 3D viewer: the calibrated dual-arm husky model, mirroring
        # * the live joint states every tick so the operator can see the real
        # * robot configuration while free-driving (same viewer setup as
        # * husky_monitor.start_pybullet, debug side panel hidden).
        pp.connect(use_gui=True, shadows=True, color=[0.9, 0.9, 1.0])
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=pp.CLIENT)
        pp.draw_pose(pp.unit_pose(), 1)
        with pp.LockRenderer(), pp.HideOutput():
            self.viz_robot = load_robot(dual_arm=True)
        # Joint ids resolved once; per tick we just write left + right angles.
        self._viz_joints = pp.joints_from_names(
            self.viz_robot,
            HUSKY_DUAL_UR5e_JOINT_NAMES[0] + HUSKY_DUAL_UR5e_JOINT_NAMES[1])

        # ! The backend must exist before any widget is created (widgets go
        # ! through common._global_backend). Wide viewport so the control panel
        # ! and both plot windows fit side by side without dragging.
        _common._global_backend = make_backend(
            use_dpg=True, window_title='Grasp Calib Recorder',
            width=1800, height=960, font_size=18)

        # --- recording state ---
        self.state = 'idle'            # idle -> recording -> idle
        self.experiment_name = 'unnamed'
        self.samples = None            # columnar buffers, created per recording
        self.last_saved_path = None
        # x-axis origin for the always-on wrench stream (seconds since launch)
        self._node_t0 = time.monotonic()

        self.build_ui()
        self.tick_timer = self.create_timer(TICK_PERIOD_S, self.update)

    # --- --- UI --- ---

    def build_ui(self):
        """Create the control panel and the two live-plot windows (built once)."""
        self.widgets = []

        # * Control panel (the primary "root" window on the left).
        self.widgets.append(Separator('Robotiq grasp calibration - ' + ROBOT_NAME))
        self.name_input = TextInput('Experiment name', lambda _text: None)
        self.widgets.append(self.name_input)
        self.widgets.append(Button('Start recording', self.on_start))
        self.widgets.append(Button('Stop & save', self.on_stop_save))
        self.widgets.append(Button('Discard recording', self.on_discard))
        # Per-arm sensor taring. Needs the UR driver's zero_ftsensor service
        # to be reachable; the result is reported in the terminal log.
        self.widgets.append(Button('Zero FT Left', lambda: self.zero_ft(0)))
        self.widgets.append(Button('Zero FT Right', lambda: self.zero_ft(1)))
        self.status_sep = Separator('status: idle')
        self.widgets.append(self.status_sep)

        backend = _common._global_backend

        # * Wrench windows, one per arm: raw sensor values, exactly what gets
        # * logged. They stream CONTINUOUSLY (pushed every tick in update(),
        # * x = seconds since launch) as a scrolling ~30 s window. The left and
        # * right force plots share one axis scale via link_group (same for the
        # * torque pair), so the two arms stay directly comparable.
        # * Force and torque split into two plots (N and Nm can't share a y axis).
        self.force_plots, self.torque_plots = [], []
        for side, tag, pos, rgb in (('Left', 'gc_wrench_l_window', (560, 10), LEFT_RGB),
                                    ('Right', 'gc_wrench_r_window', (1180, 10), RIGHT_RGB)):
            backend.add_window(f'Grasp Calib - Wrench {side}', tag=tag,
                               width=615, height=555, pos=pos)
            self.force_plots.append(HistoryPlot(
                f'{side} tool0 force (raw)', AXIS_LABELS, 'force [N]',
                parent=tag, palette=rgb, history=LIVE_STREAM_HISTORY,
                link_group='gc_force'))
            self.torque_plots.append(HistoryPlot(
                f'{side} tool0 torque (raw)', AXIS_LABELS, 'torque [Nm]',
                parent=tag, palette=rgb, history=LIVE_STREAM_HISTORY,
                link_group='gc_torque'))

        # * EE pose window: FK of both flanges, each in its own arm base frame.
        # * Position in mm so the ~2 mm motion threshold is legible on screen.
        backend.add_window('Grasp Calib - EE Pose', tag='gc_pose_window',
                           width=545, height=480, pos=(10, 470))
        self.pos_plot = HistoryPlot(
            'tool0 position', XYZ_LABELS, 'position [mm]',
            parent='gc_pose_window', group_size=3, palette=PLOT_PALETTE,
            history=PLOT_HISTORY)
        self.euler_plot = HistoryPlot(
            'tool0 orientation', EULER_LABELS, 'euler xyz [deg]',
            parent='gc_pose_window', group_size=3, palette=PLOT_PALETTE,
            history=PLOT_HISTORY)
        # Only the pose plots are per-take (reset on Start/Discard); the wrench
        # plots keep their continuous stream across takes.
        self._per_take_plots = [self.pos_plot, self.euler_plot]

        # * Right-arm live joint stream: polled by the backend every tick, so
        # * it runs continuously (also outside recordings) -- handy while
        # * free-driving the punch arm. Radians on the plot, degrees readout.
        backend.add_window('Grasp Calib - Right Arm Joints',
                           tag='gc_joints_window', width=615, height=375,
                           pos=(560, 575))
        self.right_joint_plot = LiveMultiPlot(
            'right arm joints',
            lambda: [float(v) for v in self.robot.arm_joint_pose[1]],
            JOINT_LABELS,
            history=LIVE_STREAM_HISTORY, parent='gc_joints_window')

    # --- --- BUTTON CALLBACKS --- ---

    def on_start(self):
        """Begin a take: read the name box and start recording right away."""
        if self.state != 'idle':
            self.get_logger().info(f'Start ignored, already {self.state}')
            return
        # Read the text box live -- its on-change callback only fires on Enter,
        # so this is the only reliable way to get what the user typed.
        self.experiment_name = sanitize_slug(self.name_input.value or '')
        # Fresh per-take plots + fresh buffers (the wrench stream keeps running).
        for plot in self._per_take_plots:
            plot.reset()
        self.samples = {'t': [],
                        'left': {k: [] for k in ('wrench_raw', 'q', 'ee_pos',
                                                 'ee_quat_xyzw', 'ee_euler_deg')},
                        'right': {k: [] for k in ('wrench_raw', 'q', 'ee_pos',
                                                  'ee_quat_xyzw', 'ee_euler_deg')}}
        self._recorded_at = datetime.now()
        self._t0 = time.monotonic()
        self.state = 'recording'
        self.get_logger().info(f'Recording "{self.experiment_name}" started')

    def on_stop_save(self):
        """Manual stop: save whatever has been recorded so far."""
        if self.state == 'recording':
            self._save()
        else:
            self.get_logger().info('Stop ignored, not recording')

    def zero_ft(self, index: int):
        """Tare one arm's force/torque sensor via the UR zero_ftsensor service.

        ! Only correct when that arm holds nothing but its own tool: zeroing
        ! while the box is gripped tares away the box's weight. The service is
        ! served by the UR driver, so the arm's driver stack must be running,
        ! and the command may be ignored while the pendant is in local mode --
        ! watch the logged service reply and the live force plot to confirm.

        Args:
            index (int): Arm index (0 = left, 1 = right).
        """
        side = ARM_SIDES[index]
        client = self.robot.zero_ft_sensor_client[index]
        if not client.service_is_ready():
            self.get_logger().warn(
                f'zero_ftsensor service for the {side} arm is not available '
                '(is that arm\'s UR driver running?)')
            return

        def _report(future):
            # Runs when the service reply arrives; only logs the outcome.
            try:
                result = future.result()
                self.get_logger().info(
                    f'{side} FT zero: success={result.success} {result.message}')
            except Exception as e:
                self.get_logger().warn(f'{side} FT zero call failed: {e}')

        client.call_async(Trigger.Request()).add_done_callback(_report)

    def on_discard(self):
        """Throw the current take away without saving."""
        if self.state == 'idle':
            return
        self.state = 'idle'
        self.samples = None
        for plot in self._per_take_plots:
            plot.reset()
        self.get_logger().info('Recording discarded')

    # --- --- 20 Hz TICK --- ---

    def update(self):
        """Per-tick: mirror joints into the 3D view, render the UI, then record."""
        # Pose the pybullet model to the live joint states of both arms.
        pp.set_joint_positions(
            self.viz_robot, self._viz_joints,
            list(self.robot.arm_joint_pose[0]) + list(self.robot.arm_joint_pose[1]))
        if not _common._global_backend.step():
            # Main window closed -> shut the whole node down.
            rclpy.shutdown()
            return
        for w in self.widgets:
            w.update()
        # Always-on wrench stream: raw force/torque of both arms every tick,
        # x = seconds since the recorder launched (independent of takes).
        t_live = time.monotonic() - self._node_t0
        wrench = [list(self.robot.arm_ft_sensor[i]) for i in (0, 1)]
        for i in (0, 1):
            self.force_plots[i].push(wrench[i][:3], x=t_live)
            self.torque_plots[i].push(wrench[i][3:], x=t_live)
        if self.state == 'recording':
            self._tick_recording()
        self.status_sep.set_text(self._status_text())

    def _status_text(self) -> str:
        """One-line human status for the panel's bottom separator.

        Returns:
            str: current state, and while recording the elapsed time and
            sample count.
        """
        if self.state == 'recording':
            t = time.monotonic() - self._t0
            return f'REC {t:.0f}s  n={len(self.samples["t"])}'
        if self.last_saved_path:
            return f'status: idle, last saved -> {self.last_saved_path}'
        return 'status: idle'

    def _tick_recording(self):
        """One aligned sample: snapshot streams, FK, and feed the per-take plots."""
        t = time.monotonic() - self._t0
        # The subscription callbacks assign fresh list/array objects, so plain
        # reads here are consistent snapshots (no locking needed).
        wrench = [list(self.robot.arm_ft_sensor[i]) for i in (0, 1)]
        q = [np.asarray(self.robot.arm_joint_pose[i], dtype=float) for i in (0, 1)]
        poses = [ee_pose_from_q(side, q[i]) for i, side in enumerate(ARM_SIDES)]

        # Append one sample to the columnar buffers, everything as plain lists
        # so json.dump needs no numpy handling later.
        self.samples['t'].append(t)
        for i, side in enumerate(ARM_SIDES):
            pos, quat, euler = poses[i]
            arm = self.samples[side]
            arm['wrench_raw'].append(wrench[i])
            arm['q'].append(q[i].tolist())
            arm['ee_pos'].append(pos.tolist())
            arm['ee_quat_xyzw'].append(quat.tolist())
            arm['ee_euler_deg'].append(euler.tolist())

        # Per-take live plots (the wrench plots are fed in update() instead):
        # position in mm, euler in degrees, x = seconds since Start.
        self.pos_plot.push([v * 1000.0 for i in (0, 1) for v in poses[i][0]], x=t)
        self.euler_plot.push([v for i in (0, 1) for v in poses[i][2]], x=t)


    # --- --- SAVING --- ---

    def _save(self):
        """Write the finished take to its own timestamped Insync subfolder."""
        folder = os.path.join(
            OUTPUT_ROOT,
            f'{self._recorded_at:%Y%m%d-%H%M}-{self.experiment_name}')
        os.makedirs(folder, exist_ok=True)
        payload = {
            'schema_version': SCHEMA_VERSION,
            'experiment': self.experiment_name,
            'recorded_at': self._recorded_at.isoformat(timespec='seconds'),
            'robot': ROBOT_NAME,
            'sample_period_s': TICK_PERIOD_S,
            'n_samples': len(self.samples['t']),
            # Everything a future reader needs to interpret the numbers.
            'frames': {
                'wrench': 'UR ft_sensor_wrench in the tool0 frame; RAW values, '
                          'no zeroing or baseline subtraction',
                'fk': 'tool0 in <side>_ur_arm_base_link via ssik calibrated FK '
                      '(NOT *_base_link_inertia, which differs by 180 deg yaw)',
                'joint_order': list(ARM_JOINT_NAMES),
                'euler': 'static xyz (URDF RPY convention), degrees',
                'position_unit': 'm',
            },
            'samples': self.samples,
        }
        out_path = os.path.join(folder, 'record.json')
        with open(out_path, 'w') as f:
            json.dump(payload, f, indent=2)
        # Static matplotlib snapshot of the take next to the JSON. A plotting
        # problem must never lose data, so it only warns on failure.
        try:
            self._save_plots_png(folder)
        except Exception as e:
            self.get_logger().warn(f'plots.png generation failed: {e}')
        self.last_saved_path = out_path
        self.state = 'idle'   # plots keep their curves until the next Start
        self.get_logger().info(
            f'Saved {payload["n_samples"]} samples -> {out_path}')

    def _save_plots_png(self, folder: str):
        """Render the recorded take as one static matplotlib image (plots.png).

        Mirrors the live DPG plots, but over the WHOLE take (the on-screen
        wrench plots only show a scrolling window): force and torque as
        left | right subplot pairs with a shared y scale per row, EE position
        and orientation of both arms, and the right-arm joint angles.

        Args:
            folder (str): The take's output folder (where record.json lives).
        """
        t = self.samples['t']
        if not t:
            return
        # Same color families as the DPG plots: left arm reds, right greens.
        mpl_colors = {side: [tuple(c / 255.0 for c in rgb) for rgb in arm_rgb]
                      for side, arm_rgb in (('left', LEFT_RGB), ('right', RIGHT_RGB))}
        wrench = {side: np.asarray(self.samples[side]['wrench_raw'])
                  for side in ARM_SIDES}

        fig = Figure(figsize=(14, 16))
        grid = fig.add_gridspec(4, 2, hspace=0.4, wspace=0.25)

        # Rows 0-1: force / torque, one column per arm, shared y scale per row.
        for row, (cols, unit, title) in enumerate(
                ((slice(0, 3), 'force [N]', 'tool0 force (raw)'),
                 (slice(3, 6), 'torque [Nm]', 'tool0 torque (raw)'))):
            ax_left = fig.add_subplot(grid[row, 0])
            ax_right = fig.add_subplot(grid[row, 1], sharey=ax_left)
            for ax, side in ((ax_left, 'left'), (ax_right, 'right')):
                for k, label in enumerate(AXIS_LABELS):
                    ax.plot(t, wrench[side][:, cols][:, k],
                            color=mpl_colors[side][k], label=label)
                ax.set_title(f'{side} {title}')
                ax.set_ylabel(unit)
                ax.grid(alpha=0.3)
                ax.legend(loc='upper right', fontsize=8)

        # Row 2: EE position [mm] and orientation [deg], both arms together.
        for col, (key, scale, unit, title) in enumerate(
                (('ee_pos', 1000.0, 'position [mm]', 'tool0 position'),
                 ('ee_euler_deg', 1.0, 'euler xyz [deg]', 'tool0 orientation'))):
            ax = fig.add_subplot(grid[2, col])
            for side, prefix in (('left', 'L'), ('right', 'R')):
                data = np.asarray(self.samples[side][key]) * scale
                for k, label in enumerate(AXIS_LABELS):
                    ax.plot(t, data[:, k], color=mpl_colors[side][k],
                            label=f'{prefix} {label}')
            ax.set_title(title)
            ax.set_ylabel(unit)
            ax.grid(alpha=0.3)
            ax.legend(loc='upper right', fontsize=8, ncol=2)

        # Row 3: right-arm joint angles across the take, spanning both columns.
        ax = fig.add_subplot(grid[3, :])
        q_right = np.asarray(self.samples['right']['q'])
        for k, label in enumerate(JOINT_LABELS):
            ax.plot(t, q_right[:, k], label=label)
        ax.set_title('right arm joints')
        ax.set_ylabel('angle [rad]')
        ax.set_xlabel('time since recording start [s]')
        ax.grid(alpha=0.3)
        ax.legend(loc='upper right', fontsize=8, ncol=3)

        fig.suptitle(f'{self.experiment_name}  ({self._recorded_at:%Y-%m-%d %H:%M})',
                     fontsize=14)
        fig.savefig(os.path.join(folder, 'plots.png'), dpi=110,
                    bbox_inches='tight')

    def destroy_node(self):
        if _common._global_backend is not None:
            try:
                _common._global_backend.shutdown()
            except Exception as e:
                self.get_logger().warn(f'UI backend shutdown error: {e}')
            _common._global_backend = None
        super().destroy_node()


# --- --- HARDWARE PREFLIGHT (--check) --- ---

def run_check(duration_s: float = 3.0) -> int:
    """Verify the read-only streams and FK before the first real experiment.

    Subscribes directly to both arms' wrench and joint topics (no UI, no
    HuskyRobotInterface -- this needs per-topic message counters), listens for
    a few seconds, prints each topic's rate and last message, then runs the
    ssik FK on the latest joint configs. Run this with the husky stack up and
    the pendants in LOCAL mode: it proves that reading works without remote
    control.

    Args:
        duration_s (float): how long to listen for messages.

    Returns:
        int: 0 when every topic is alive and FK ran, 1 otherwise.
    """
    rclpy.init()
    node = Node('grasp_calib_check')
    counts = [0] * len(CHECK_TOPICS)
    last_msgs = [None] * len(CHECK_TOPICS)

    def _make_cb(idx):
        def _cb(msg):
            counts[idx] += 1
            last_msgs[idx] = msg
        return _cb

    subs = [node.create_subscription(msg_type, topic, _make_cb(i), 10)
            for i, (topic, msg_type, _label) in enumerate(CHECK_TOPICS)]

    print(f'[check] listening on {len(subs)} topics for {duration_s} s ...')
    t_end = time.monotonic() + duration_s
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)

    ok = True
    joint_q = {}
    for i, (topic, _msg_type, label) in enumerate(CHECK_TOPICS):
        rate = counts[i] / duration_s
        alive = counts[i] > 0
        ok = ok and alive
        print(f'[check] {label:13s} {topic}\n'
              f'        {counts[i]} msgs (~{rate:.0f} Hz) '
              f'{"OK" if alive else "!! NO DATA"}')
        if not alive:
            continue
        msg = last_msgs[i]
        if isinstance(msg, WrenchStamped):
            w = msg.wrench
            print(f'        last wrench: F=({w.force.x:+.2f}, {w.force.y:+.2f}, '
                  f'{w.force.z:+.2f}) N  T=({w.torque.x:+.3f}, '
                  f'{w.torque.y:+.3f}, {w.torque.z:+.3f}) Nm')
        else:
            # Reorder to driver order, exactly like HuskyRobotInterface does.
            reorder = [msg.name.index(n) for n in ARM_JOINT_NAMES]
            q = np.array(msg.position)[reorder]
            side = 'left' if 'left' in topic else 'right'
            joint_q[side] = q
            print(f'        last q [rad]: {np.round(q, 3).tolist()}')

    # FK on whatever joint configs arrived -- proves the ssik path end to end.
    for side, q in joint_q.items():
        pos, _quat, euler = ee_pose_from_q(side, q)
        print(f'[check] {side} FK: tool0 in {side}_ur_arm_base_link  '
              f'pos [m] = {np.round(pos, 4).tolist()}  '
              f'euler xyz [deg] = {np.round(euler, 2).tolist()}')

    node.destroy_node()
    rclpy.shutdown()
    print(f'[check] {"PASS" if ok else "FAIL"}')
    return 0 if ok else 1


# --- --- MAIN --- ---

def main(args=None):
    if '--check' in sys.argv[1:]:
        sys.exit(run_check())
    rclpy.init(args=args)
    node = GraspCalibRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
