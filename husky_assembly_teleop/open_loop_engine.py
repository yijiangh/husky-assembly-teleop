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
    sequential) -> START TRACKING. STOP (or closing the window, or Ctrl+C)
    always speedStops both arms. Gripper open/close annotations fire through
    the existing ROS2 GripperCommand action path (--no-gripper logs only).

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

from husky_assembly_teleop import DATA_DIRECTORY
from husky_assembly_teleop import common as _common
from husky_assembly_teleop.common import (Button, HistoryPlot, LiveMultiPlot,
                                          Separator, Slider, load_robot,
                                          HUSKY_DUAL_UR5e_JOINT_NAMES)
from husky_assembly_teleop.husky_robot import HuskyRobotInterface
from husky_assembly_teleop.open_loop_traj import (ARM_SIDES, OpenLoopTraj,
                                                  load_open_loop_traj)
from husky_assembly_teleop.ui_backend import make_backend

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

# Gripper commands reuse the exact values the monitor's buttons send
# (husky_world.open_gripper_full / close_gripper_for_bar). The values are
# knuckle-joint angles in radians (0 = fully open, 0.803 = fully closed).
GRIPPER_OPEN, GRIPPER_CLOSE, GRIPPER_EFFORT = 0.426, 0.8, 0.1

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
# tool0 -> gripper base mount: the URDF's gripper extends along its own +x,
# but physically it extends along tool0 +z (utils.TOOL0_FROM_GRIPPER_TCP puts
# the TCP at tool0 +z 0.164 = 0.152 gripper + 0.012 coupler). So: 12 mm
# coupler offset, then pitch -90 deg to swing the URDF's x onto tool0's z.
GRIPPER_MOUNT_POSE = pp.multiply(
    pp.Pose(point=(0.0, 0.0, 0.012)),
    pp.Pose(euler=pp.Euler(pitch=-np.pi / 2)))

# Slow, supervised approach to the trajectory's first sample.
MOVE_TO_START_SPEED = 0.3   # rad/s
MOVE_TO_START_ACCEL = 0.5   # rad/s^2
# Beyond this per-joint distance the operator jogs by pendant instead --
# a blind moveJ across a large distance could sweep through the other arm.
MAX_MOVE_TO_START_DELTA = 1.5   # rad

# ! t0 is placed this far in the future when START is pressed: both tracker
# ! threads' first cycles see t < 0, whose clamped reference (start pose,
# ! zero velocity) makes them HOLD position until the shared clock reaches 0.
# ! Thread startup jitter therefore never desynchronizes the arms.
START_PREROLL_S = 0.5
END_SETTLE_S = 1.0          # keep holding the final pose this long past the end


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
        self.viz_grippers = []   # per arm: (body id, attachment, finger joint ids)
        for tool0_name in ('left_ur_arm_tool0', 'right_ur_arm_tool0'):
            tool0_link = pp.link_from_name(self.viz_robot, tool0_name)
            with pp.LockRenderer(), pp.HideOutput():
                grip_body = pp.load_pybullet(GRIPPER_URDF, fixed_base=False)
            pp.set_pose(grip_body,
                        pp.multiply(pp.get_link_pose(self.viz_robot, tool0_link),
                                    GRIPPER_MOUNT_POSE))
            grip_att = pp.create_attachment(self.viz_robot, tool0_link, grip_body)
            grip_joints = pp.joints_from_names(grip_body, GRIPPER_VIZ_JOINTS)
            self.viz_grippers.append((grip_body, grip_att, grip_joints))
        # Displayed knuckle angle per arm; both grippers assumed open at start.
        self.grip_viz_angle = [GRIPPER_OPEN, GRIPPER_OPEN]

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
        # state machine: disconnected -> connected -> ready -> (moving ->
        # ready) -> tracking -> done | aborted. Buttons ignore presses in the
        # wrong state and say why in the log.
        self.state = 'disconnected'
        self.rtde_c = [None, None]
        self.rtde_r = [None, None]
        self.live_q = [self.traj.q12[0, :6].copy(), self.traj.q12[0, 6:].copy()]
        self.live_err = [np.zeros(6), np.zeros(6)]
        self.start_deltas = None          # per-arm |q_now - q_start|, from Check
        self.stop_evt = None
        self.exec_t0 = None
        self.threads = []
        self.thread_done = [False, False]
        self.thread_error = [None, None]
        self.exec_log = None
        self.next_event_idx = 0
        self.fired_events = []
        self._log_saved = True            # nothing to save until a run starts
        self.last_run_folder = None

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
                f'gripper={"OFF (log only)" if self.args.no_gripper else "ROS"}'))
            self.widgets.append(Button('Connect RTDE', self.on_connect))
            self.widgets.append(Button('Check start pose', self.on_check_start_pose))
            self.widgets.append(Button('Move to start (slow)', self.on_move_to_start))
            self.widgets.append(Button('START TRACKING', self.on_start))
            self.widgets.append(Button('STOP', self.on_stop))
            self.delta_sep = Separator('start pose: not checked')
            self.widgets.append(self.delta_sep)
            self.event_sep = Separator('gripper events: none fired')
            self.widgets.append(self.event_sep)

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

    # --- --- 20 Hz TICK --- ---

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
        self.grip_viz_angle = [GRIPPER_CLOSE if closed else GRIPPER_OPEN
                               for closed in self.traj.grip_closed[i]]

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
        self.get_logger().info(
            f'reference velocity check OK ({ref_vmax:.3f} '
            f'< {self.args.max_joint_vel} rad/s)')
        ips = (self.args.left_ip, self.args.right_ip)
        try:
            for i, ip in enumerate(ips):
                self.get_logger().info(f'connecting {ARM_SIDES[i]} arm @ {ip} ...')
                self.rtde_c[i] = RTDEControlInterface(ip, self.args.frequency)
                self.rtde_r[i] = RTDEReceiveInterface(ip, self.args.frequency)
            self.state = 'connected'
            self.get_logger().info('RTDE connected to both arms')
        except Exception as e:
            self.get_logger().error(f'RTDE connect failed: {e}')
            self.rtde_c = [None, None]
            self.rtde_r = [None, None]

    def on_check_start_pose(self):
        """Compare live joints against the trajectory start; gate START on it.

        Also allowed after a finished run ('done'/'aborted'): the RTDE
        connections are still up and the tracker threads have ended, so the
        operator can re-arm and run again without restarting the program.
        """
        if self.state not in ('connected', 'ready', 'done', 'aborted'):
            self.get_logger().warn(f'Check ignored in state {self.state}')
            return
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
        """Slow sequential moveJ of both arms to the first trajectory sample."""
        if self.state not in ('connected', 'ready', 'done', 'aborted'):
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

    def on_start(self):
        """Arm and launch the two tracker threads on one shared clock."""
        if self.state != 'ready':
            self.get_logger().warn(
                f'START ignored in state {self.state} (need a passed Check)')
            return
        self.exec_log = [{k: [] for k in ('t', 'q_ref', 'q_actual', 'qd_cmd')}
                         for _ in range(2)]
        self.thread_done = [False, False]
        self.thread_error = [None, None]
        self.next_event_idx = 0
        self.fired_events = []
        self._log_saved = False
        for plot in self.err_plots:
            plot.reset()
        self.stop_evt = threading.Event()
        # ! Set t0 BEFORE spawning: the shared clock is the synchronization.
        self.exec_t0 = time.monotonic() + START_PREROLL_S
        self.threads = [threading.Thread(target=self._arm_tracker, args=(i,),
                                         daemon=True) for i in range(2)]
        self.state = 'tracking'
        for th in self.threads:
            th.start()
        self.get_logger().info(
            f'tracking started, {self.traj.duration:.1f}s to go')

    def on_stop(self):
        """Operator STOP: halt both arms, save what was recorded."""
        if self.state == 'tracking':
            self._stop_tracking('operator STOP')
        elif self.state == 'moving':
            # moveJ blocks its worker thread; interrupting it cross-thread
            # is not safe with ur_rtde -- use the pendant if it must stop NOW.
            self.get_logger().warn(
                'STOP during move-to-start: wait for the slow moveJ to end '
                '(or use the pendant e-stop)')
        else:
            self.get_logger().warn(f'STOP ignored in state {self.state}')

    # --- --- EXECUTE MODE: TRACKING --- ---

    def _arm_tracker(self, arm_i: int):
        """One arm's speedJ path-tracking loop (runs in its own thread).

        Port of the ReferencePath branch of Valentin's controller: reference
        feed-forward velocity plus a P term on the position error, clamped,
        sent as speedJ at the configured frequency. Any exception (RTDE
        drop, tracking blowup) stops BOTH arms via the shared stop event.

        Args:
            arm_i (int): 0 = left, 1 = right.
        """
        rtde_c, rtde_r = self.rtde_c[arm_i], self.rtde_r[arm_i]
        p_gain = self.args.p_gain[arm_i]
        vmax = self.args.max_joint_vel
        dt_cmd = 1.0 / self.args.frequency
        log = self.exec_log[arm_i]   # owned by this thread until it finishes
        try:
            while not self.stop_evt.is_set():
                t_cycle = rtde_c.initPeriod()
                t = time.monotonic() - self.exec_t0
                q_ref, qd_ref = self.traj.sample(arm_i, t)
                q_act = np.asarray(rtde_r.getActualQ())
                err = q_ref - q_act
                if np.abs(err).max() > self.args.err_abort:
                    # * The open-loop safety net: if reality drifts this far
                    # * from the plan, something is wrong -- stop everything.
                    raise RuntimeError(
                        f'{ARM_SIDES[arm_i]} tracking error '
                        f'{np.abs(err).max():.3f} rad > --err-abort '
                        f'{self.args.err_abort}')
                qd_cmd = np.clip(qd_ref + p_gain * err, -vmax, vmax)
                rtde_c.speedJ(list(qd_cmd), self.args.joint_accel, dt_cmd)
                # Fresh-object writes: GIL-atomic snapshots for the UI thread.
                self.live_q[arm_i] = q_act
                self.live_err[arm_i] = err
                log['t'].append(t)
                log['q_ref'].append(q_ref)
                log['q_actual'].append(q_act)
                log['qd_cmd'].append(qd_cmd)
                if t > self.traj.duration + END_SETTLE_S:
                    break   # final pose held long enough -- clean finish
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

    def _tick_execute(self):
        """20 Hz supervision: mirror, gripper events, plots, finish detection."""
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
        if self.state == 'tracking':
            t = time.monotonic() - self.exec_t0
            # Fire every gripper event whose time has come (<= 50 ms late,
            # ample against the 0.5 s hold the planner builds around closes).
            while (self.next_event_idx < len(self.traj.events)
                   and self.traj.events[self.next_event_idx].time <= t):
                self._fire_gripper_event(self.traj.events[self.next_event_idx], t)
                self.next_event_idx += 1
            for i in range(2):
                self.err_plots[i].push([float(v) for v in self.live_err[i]],
                                       x=max(t, 0.0))
            nxt = (self.traj.events[self.next_event_idx]
                   if self.next_event_idx < len(self.traj.events) else None)
            status = (f'TRACKING t={t:7.1f}/{self.traj.duration:.1f}s'
                      + (f' | next: {nxt.kind} {ARM_SIDES[nxt.arm_index]} '
                         f'@{nxt.time:.1f}s' if nxt else ' | no more events'))
            if all(self.thread_done):
                self._finish()
                status = f'state: {self.state}'
        if self.last_run_folder:
            status += f' | saved -> {self.last_run_folder}'
        self.status_sep.set_text(status)

    def _fire_gripper_event(self, ev, t_now: float):
        """Send (or just log) one gripper command scheduled by the trajectory.

        Args:
            ev (GripperEvent): The event to fire.
            t_now (float): Current shared trajectory time [s] (for lateness
                bookkeeping in the run log).
        """
        pos = GRIPPER_CLOSE if ev.kind == 'close' else GRIPPER_OPEN
        # Mirror the commanded state in the 3D view (also under --no-gripper,
        # where it shows what WOULD have been sent).
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
            self.robot.send_gripper_cmd(pos, GRIPPER_EFFORT, ev.arm_index)
            record['status'] = 'sent'
        self.fired_events.append(record)
        self.event_sep.set_text(
            f'gripper: {ev.kind} {ARM_SIDES[ev.arm_index]} @{ev.time:.2f}s '
            f'({record["status"]}, {len(self.fired_events)}'
            f'/{len(self.traj.events)})')
        self.get_logger().info(f'gripper event: {record}')

    # --- --- EXECUTE MODE: FINISH / ABORT / SAVE --- ---

    def _finish(self):
        """Both tracker threads ended on their own: classify and save."""
        for th in self.threads:
            th.join(timeout=2.0)
        errors = [e for e in self.thread_error if e]
        if errors:
            self.state = 'aborted'
            self._save_run_log('aborted: ' + '; '.join(errors))
        else:
            self.state = 'done'
            self._save_run_log('done')

    def _stop_tracking(self, reason: str):
        """Stop a live run from outside the tracker threads (STOP/close/^C).

        Safe to call in any state and more than once; only acts when a run
        is actually in progress. Always ends with speedStop on both arms.

        Args:
            reason (str): Human-readable cause, recorded in the run log.
        """
        if self.state != 'tracking':
            return
        self.get_logger().warn(f'stopping tracking: {reason}')
        self.stop_evt.set()
        for th in self.threads:
            th.join(timeout=2.0)
            if th.is_alive():
                self.get_logger().error('a tracker thread did not stop in 2 s')
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
            f'{stem}-run-{datetime.now():%Y%m%d-%H%M%S}')
        os.makedirs(folder, exist_ok=True)

        arrays = {}
        for i, side in enumerate(ARM_SIDES):
            for key, rows in self.exec_log[i].items():
                arrays[f'{side}_{key}'] = np.asarray(rows)
        np.savez(os.path.join(folder, 'log.npz'), **arrays)

        info = {
            'outcome': outcome,
            'traj_json': self.traj.source_path,
            'args': vars(self.args),
            'started_preroll_s': START_PREROLL_S,
            'start_deltas': [d.tolist() for d in (self.start_deltas or [])],
            'events_fired': self.fired_events,
            'thread_errors': self.thread_error,
            'n_cycles': [len(self.exec_log[i]['t']) for i in range(2)],
        }
        with open(os.path.join(folder, 'run_info.json'), 'w') as f:
            json.dump(info, f, indent=2)
        try:
            self._save_plots_png(folder)
        except Exception as e:
            self.get_logger().warn(f'plots.png generation failed: {e}')
        self.last_run_folder = folder
        self.get_logger().info(f'run log ({outcome}) saved -> {folder}')

    def _save_plots_png(self, folder: str):
        """Render the whole run as one static image: q, error, qd per arm.

        Args:
            folder (str): The run's output folder (where log.npz lives).
        """
        fig = Figure(figsize=(14, 14))
        grid = fig.add_gridspec(3, 2, hspace=0.35, wspace=0.2)
        for i, side in enumerate(ARM_SIDES):
            log = self.exec_log[i]
            t = np.asarray(log['t'])
            if not len(t):
                continue
            q_ref, q_act = np.asarray(log['q_ref']), np.asarray(log['q_actual'])
            for row, (title, unit) in enumerate((
                    ('joints: ref (dashed) vs actual', 'q [rad]'),
                    ('tracking error', 'q_ref - q_actual [rad]'),
                    ('commanded speed', 'qd_cmd [rad/s]'))):
                ax = fig.add_subplot(grid[row, i])
                labels = JOINT_LABELS_12[6 * i:6 * i + 6]
                if row == 0:
                    for k, lb in enumerate(labels):
                        line, = ax.plot(t, q_act[:, k], label=lb)
                        ax.plot(t, q_ref[:, k], '--', color=line.get_color(),
                                linewidth=0.8)
                elif row == 1:
                    ax.plot(t, q_ref - q_act, label=labels)
                else:
                    ax.plot(t, np.asarray(log['qd_cmd']), label=labels)
                # Gripper events of this arm as vertical markers.
                for rec in self.fired_events:
                    if rec['arm'] == side:
                        ax.axvline(rec['planned_t'], color='k', alpha=0.25,
                                   linestyle=':' if rec['kind'] == 'open' else '-')
                ax.set_title(f'{side} {title}')
                ax.set_ylabel(unit)
                ax.grid(alpha=0.3)
                ax.legend(loc='upper right', fontsize=7, ncol=3)
                if row == 2:
                    ax.set_xlabel('trajectory time [s]')
        fig.suptitle(f'{os.path.basename(self.traj.source_path)}  '
                     f'({datetime.now():%Y-%m-%d %H:%M})', fontsize=13)
        fig.savefig(os.path.join(folder, 'plots.png'), dpi=110,
                    bbox_inches='tight')

    # --- --- SHUTDOWN --- ---

    def destroy_node(self):
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
    cli.add_argument('--no-gripper', action='store_true',
                     help='log gripper events instead of sending them')
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
