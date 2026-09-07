"""Generate a tiny test trajectory around the arms' CURRENT configuration.

* First-hardware-contact tool for open_loop_engine --execute: the written
* file starts EXACTLY at the live joint configuration of both arms, so the
* start-pose check passes with ~zero delta and there is no catch-up motion.
* Each repetition is: OPEN both grippers (holding still), one slow
* out-and-back "wiggle" where each selected joint follows
* q0 + A/2 * (1 - cos(2*pi*t/T)) (zero velocity at both ends), then CLOSE
* both grippers (holding still) -- so one file exercises arm tracking AND
* the gripper command path together, mimicking the real trajectories'
* "arms hold still around gripper events" structure. With the defaults
* (0.05 rad on the three wrist joints, 5 s per wiggle, 2 repetitions,
* 1 s holds) the peak arm speed is ~0.03 rad/s.

Reading the current configuration uses RTDEReceive only, which works even
with the pendant in LOCAL mode and the ROS driver still running -- so the
file can be generated before any mode switching. No gripper events are
written; run the engine with --no-gripper.

Run (standalone, venv python -- no ROS env needed):
    python scripts/make_wiggle_traj.py -o wiggle_test.json
    python scripts/make_wiggle_traj.py --left-q 0 -1.57 1.57 -1.57 -1.57 0 \
        --right-q 0 -1.57 1.57 -1.57 -1.57 0     # offline, without the arms
Then:
    ros2 run husky_assembly_teleop open_loop_engine wiggle_test.json            # preview
    ros2 run husky_assembly_teleop open_loop_engine wiggle_test.json \
        --execute --no-gripper --err-abort 0.1                                  # hardware
"""

import argparse
import importlib.util
import json
import os
import sys

import numpy as np

try:
    from rtde_receive import RTDEReceiveInterface
except ImportError:
    RTDEReceiveInterface = None

# ! Written in the SAME mapping the engine uses by default: a1 = left arm,
# ! a2 = right arm. Do not pass --swap-arms to the engine for these files.
ROBOTS = ('a1', 'a2')
JOINT_SUFFIXES = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                  'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')


def read_current_q(ip: str, side: str) -> np.ndarray:
    """Read one arm's live joint configuration over RTDE (receive-only).

    Args:
        ip (str): The arm's IP address.
        side (str): 'left' or 'right', for log messages only.

    Returns:
        np.ndarray: 6 joint angles [rad].
    """
    if RTDEReceiveInterface is None:
        sys.exit('ur_rtde is not installed -- pip install ur_rtde, or pass '
                 '--left-q/--right-q instead')
    print(f'reading {side} arm current q @ {ip} ...')
    recv = RTDEReceiveInterface(ip)
    q = np.asarray(recv.getActualQ())
    mode = recv.getRobotMode()
    recv.disconnect()
    print(f'  {side} q = {np.array2string(q, precision=4)} (robot_mode={mode})')
    # ! A powered-off arm (robot_mode 3) reports q = all zeros -- writing a
    # ! trajectory around that would command a huge jump on the real robot.
    # ! Modes: 3 POWER_OFF, 5 IDLE (on, brakes engaged, q valid), 7 RUNNING.
    if mode < 5 or not np.any(q):
        sys.exit(f'{side} arm is not powered on (robot_mode={mode}) -- its '
                 'joint values are not valid. Power it on first (silver '
                 'button / pendant), then rerun.')
    return q


def main() -> int:
    cli = argparse.ArgumentParser(
        description='Write a tiny wiggle trajectory (assembly-open-loop-'
                    'json-v2) starting at the arms\' current configuration.')
    cli.add_argument('-o', '--out', default='wiggle_test.json')
    cli.add_argument('--left-ip', default='192.168.131.40')
    cli.add_argument('--right-ip', default='192.168.131.41')
    cli.add_argument('--left-q', type=float, nargs=6, default=None,
                     help='use these 6 joint values instead of reading RTDE')
    cli.add_argument('--right-q', type=float, nargs=6, default=None)
    cli.add_argument('--amplitude', type=float, default=0.05,
                     help='peak joint excursion [rad]')
    cli.add_argument('--joints', default='3,4,5',
                     help='comma-separated joint indices to move, 0=pan .. '
                          '5=wrist_3 (default: the three wrist joints)')
    cli.add_argument('--wiggle-time', type=float, default=5.0,
                     help='seconds per out-and-back wiggle repetition')
    cli.add_argument('--cycles', type=int, default=2,
                     help='open -> wiggle -> close repetitions')
    cli.add_argument('--grip-hold', type=float, default=1.0,
                     help='still hold [s] around each gripper command')
    cli.add_argument('--no-grip-events', action='store_true',
                     help='plain wiggle, no gripper open/close annotations')
    cli.add_argument('--dt', type=float, default=0.05)
    args = cli.parse_args()

    q0 = [np.asarray(args.left_q) if args.left_q is not None
          else read_current_q(args.left_ip, 'left'),
          np.asarray(args.right_q) if args.right_q is not None
          else read_current_q(args.right_ip, 'right')]

    moved = [int(j) for j in args.joints.split(',')]
    amp = np.zeros(12)
    for i in range(2):
        for j in moved:
            amp[6 * i + j] = args.amplitude

    # * Segment plan per repetition: [open + hold] [wiggle] [close + hold].
    # * The wiggle offset is A/2 * (1 - cos(w t)) with w = 2*pi/wiggle_time:
    # * zero position offset AND zero velocity at both segment ends, so the
    # * hold/wiggle boundaries are perfectly smooth. Analytic qd/qdd go
    # * straight into the file. Gripper events are marked on every sample of
    # * their hold (the loader only reacts to the rising edge).
    dt = args.dt
    n_hold = int(round(args.grip_hold / dt))
    n_wiggle = int(round(args.wiggle_time / dt))
    w = 2.0 * np.pi / args.wiggle_time
    q0_12 = np.concatenate(q0)
    both = [] if args.no_grip_events else list(ROBOTS)

    # Per-sample rows: (position offset scale, velocity scale, accel scale,
    # closing list, opening list); scales multiply the amplitude vector.
    rows = []
    for _ in range(args.cycles):
        rows += [(0.0, 0.0, 0.0, [], both)] * n_hold           # open + hold
        rows += [(0.5 * (1 - np.cos(w * k * dt)),              # the wiggle
                  0.5 * w * np.sin(w * k * dt),
                  0.5 * w * w * np.cos(w * k * dt),
                  [], []) for k in range(n_wiggle)]
        rows += [(0.0, 0.0, 0.0, both, [])] * n_hold           # close + hold
    rows.append((0.0, 0.0, 0.0, [], []))                       # final rest sample

    samples = [{'time': round(i * dt, 6),
                'q': (q0_12 + s_q * amp).tolist(),
                'qd': (s_qd * amp).tolist(),
                'qdd': (s_qdd * amp).tolist(),
                'closing_gripper': closing, 'opening_gripper': opening,
                'servo_controller': False}
               for i, (s_q, s_qd, s_qdd, closing, opening) in enumerate(rows)]
    peak_qd = float(args.amplitude * 0.5 * w)
    payload = {
        'schema': 'assembly-open-loop-json-v2',
        'dt': dt,
        'grasp_wait': args.grip_hold,
        'joint_names': [f'{r}_ur_{s}' for r in ROBOTS for s in JOINT_SUFFIXES],
        'robots': list(ROBOTS),
        'robot_slices': {'a1': [0, 6], 'a2': [6, 12]},
        'samples': samples,
    }
    with open(args.out, 'w') as f:
        json.dump(payload, f)
    print(f'wrote {args.out}: {len(samples)} samples, '
          f'{(len(samples) - 1) * dt:.1f}s, {args.cycles}x '
          f'(open/hold + wiggle + close/hold), joints {moved} '
          f'amp {args.amplitude} rad, peak arm speed {peak_qd:.4f} rad/s')

    # * Self-check: round-trip the file through the engine's own loader
    # * (loaded by path so this works without the ROS env / colcon install).
    loader_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               '..', 'husky_assembly_teleop', 'open_loop_traj.py')
    spec = importlib.util.spec_from_file_location('open_loop_traj', loader_path)
    olt = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(olt)
    traj = olt.load_open_loop_traj(args.out)
    print('\nloader round-trip OK:')
    print(traj.summary())
    return 0


if __name__ == '__main__':
    sys.exit(main())
