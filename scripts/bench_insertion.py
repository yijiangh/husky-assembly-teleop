#!/usr/bin/env python3
"""Run ONE compliant insertion on one arm, from wherever it is standing now.

The engine is not involved: no trajectory, no plan, no second arm, no ROS.
Put the part in the gripper, jog the arm to the funnel mouth -- the pose the
part would be in just before it goes in -- and this drives the rest of the way
under force control, then writes the forces to a folder next to itself.

That makes it the first thing to run on the robot: it exercises the whole
insertion skill (zeroing, approach, spiral search, press, jam handling, the
guard) with one arm, one part and one operator hand on the pendant.

    # look before touching: prints the pose, the wrench and the plan
    python3 scripts/bench_insertion.py --arm left --dry-run

    # then, with a hand on the e-stop
    python3 scripts/bench_insertion.py --arm left --depth-mm 30 --axis tool-z

! Read `doc/rtde_network_setup.md` first: the UR drivers and
! multi_arm_safety_sync must be stopped or they will fight for the arm, and
! the pendant must be in Remote.

! --axis tool-z presses along the gripper's own approach direction, which is
! what a leg going into a seat does. --axis base-z presses straight down.
! Check the arrow the dry run prints before letting it move.
"""

import argparse
import json
import os
import sys
from datetime import datetime

import numpy as np
from matplotlib.figure import Figure
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from husky_assembly_teleop.open_loop_insertion import (  # noqa: E402
    PHASES, InsertionController, InsertionParams)

ARM_IPS = {'left': '192.168.131.40', 'right': '192.168.131.41'}
# tool0 -> gripper TCP, as everywhere else in this package
# (`utils.TOOL0_FROM_GRIPPER_TCP`: 152 mm gripper + 12 mm coupler).
TCP_OFFSET = (0.0, 0.0, 0.164, 0.0, 0.0, 0.0)


def insertion_axis(pose, which: str) -> np.ndarray:
    """The direction to press in, as a unit vector in the base frame.

    Args:
        pose (np.ndarray): The current TCP pose [x, y, z, rx, ry, rz].
        which (str): 'tool-z' to press along the tool's own z (the way a
            gripper approaches), or 'base-z' to press straight down.

    Returns:
        np.ndarray: (3,) unit vector.
    """
    if which == 'base-z':
        return np.array([0.0, 0.0, -1.0])
    return Rotation.from_rotvec(pose[3:]).apply([0.0, 0.0, 1.0])


def save_run(control, folder: str, meta: dict):
    """Write the log, the summary and a plot of one bench insertion.

    Args:
        control (InsertionController): The controller that just ran.
        folder (str): Directory to create and write into.
        meta (dict): Extra fields for record.json (arm, ip, arguments).
    """
    os.makedirs(folder, exist_ok=True)
    np.savez(os.path.join(folder, 'log.npz'), **control.log.arrays())
    with open(os.path.join(folder, 'record.json'), 'w') as f:
        json.dump({**meta, **control.record}, f, indent=2)

    t = np.asarray(control.log.t)
    if not len(t):
        return
    wrench = np.asarray(control.log.wrench_lp)
    fig = Figure(figsize=(12, 11))
    grid = fig.add_gridspec(4, 1, hspace=0.35)

    ax = fig.add_subplot(grid[0])
    ax.plot(t, np.asarray(control.log.axial) * 1000, label='travelled')
    ax.axhline(control.depth * 1000, color='k', ls='--', lw=0.8,
               label='seated depth')
    ax.set_ylabel('along the axis [mm]')
    ax.set_title(f'bench insertion: {control.outcome}')

    ax = fig.add_subplot(grid[1])
    ax.plot(t, wrench[:, :3], label=['fx', 'fy', 'fz'])
    ax.axhline(control.params.push_force, color='k', ls=':', lw=0.8)
    ax.axhline(-control.params.push_force, color='k', ls=':', lw=0.8)
    ax.set_ylabel('force, base frame [N]')

    ax = fig.add_subplot(grid[2])
    ax.plot(t, np.asarray(control.log.search_r) * 1000, label='search radius')
    ax.plot(t, np.asarray(control.log.lateral_err) * 1000, label='lateral lag')
    ax.set_ylabel('[mm]')

    ax = fig.add_subplot(grid[3])
    ax.step(t, control.log.phase, where='post')
    ax.set_yticks(range(len(PHASES)))
    ax.set_yticklabels(PHASES)
    ax.set_xlabel('time [s]')

    for axis in fig.axes:
        axis.grid(alpha=0.3)
        if axis.get_legend_handles_labels()[1]:
            axis.legend(loc='upper right', fontsize=8)
    fig.savefig(os.path.join(folder, 'insertion.png'), dpi=110,
                bbox_inches='tight')


def main():
    """Connect, optionally run one insertion, and save what happened."""
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--arm', choices=('left', 'right'), default='left')
    cli.add_argument('--ip', default=None, help='override the arm IP')
    cli.add_argument('--depth-mm', type=float, default=30.0,
                     help='how far to press in from the current pose')
    cli.add_argument('--axis', choices=('tool-z', 'base-z'), default='tool-z')
    cli.add_argument('--frequency', type=float, default=125.0)
    cli.add_argument('--dry-run', action='store_true',
                     help='report the pose, wrench and intended motion, move nothing')
    cli.add_argument('--out', default=None, help='where to write the run folder')
    # The knobs most likely to need changing at the bench.
    cli.add_argument('--push-force', type=float, default=None)
    cli.add_argument('--guard-force', type=float, default=None)
    cli.add_argument('--search-radius-mm', type=float, default=None)
    cli.add_argument('--approach-speed-mm', type=float, default=None)
    args = cli.parse_args()

    from rtde_control import RTDEControlInterface
    from rtde_receive import RTDEReceiveInterface

    ip = args.ip or ARM_IPS[args.arm]
    print(f'connecting to the {args.arm} arm at {ip} ...')
    rtde_c = RTDEControlInterface(ip, args.frequency)
    rtde_r = RTDEReceiveInterface(ip, args.frequency)

    params = InsertionParams()
    params.tcp_offset = TCP_OFFSET
    for value, field in ((args.push_force, 'push_force'),
                         (args.guard_force, 'guard_force')):
        if value is not None:
            setattr(params, field, value)
    if args.search_radius_mm is not None:
        params.search_radius = args.search_radius_mm / 1000.0
    if args.approach_speed_mm is not None:
        params.approach_speed = args.approach_speed_mm / 1000.0

    rtde_c.setTcp(list(TCP_OFFSET))
    q_now = np.asarray(rtde_r.getActualQ())
    pose = np.asarray(rtde_r.getActualTCPPose())
    wrench = np.asarray(rtde_r.getActualTCPForce())
    axis = insertion_axis(pose, args.axis)
    depth = args.depth_mm / 1000.0

    print(f'  joints   {np.round(q_now, 4)}')
    print(f'  TCP      {np.round(pose[:3], 4)} m, rotvec {np.round(pose[3:], 4)}')
    print(f'  wrench   {np.round(wrench, 2)} (base frame, NOT yet zeroed)')
    print(f'  pressing {args.depth_mm:.1f} mm along {np.round(axis, 3)} '
          f'({args.axis}), ending at {np.round(pose[:3] + depth * axis, 4)}')
    print(f'  push {params.push_force:.0f} N, guard {params.guard_force:.0f} N, '
          f'search radius {params.search_radius * 1000:.0f} mm')

    if args.dry_run:
        print('\ndry run: nothing was commanded. Check the direction above, '
              'then run again without --dry-run.')
        rtde_c.stopScript()
        return 0

    # ! The controller measures its own line from two CONFIGURATIONS, so the
    # ! target is expressed the same way here: ask the robot which joints put
    # ! the TCP at the seated pose.
    seated = list(pose[:3] + depth * axis) + list(pose[3:])
    q_seated = rtde_c.getInverseKinematics(seated, list(q_now))
    if not len(q_seated):
        print('the seated pose has no IK solution from here -- aborting')
        rtde_c.stopScript()
        return 1

    control = InsertionController(rtde_c, rtde_r, params, 1.0 / args.frequency)
    control.setup(q_now, np.asarray(q_seated))
    print('\nrunning -- hand on the e-stop. Ctrl+C stops the arm.\n')
    import time
    t0 = time.monotonic()
    try:
        while True:
            cycle = rtde_c.initPeriod()
            if not control.step(time.monotonic() - t0):
                break
            rtde_c.waitPeriod(cycle)
    except KeyboardInterrupt:
        print('\ninterrupted by the operator')
        control.outcome = control.outcome or 'operator_stop'
    finally:
        control.stop()

    print(f'\noutcome: {control.outcome}')
    for key in ('reached_depth_mm', 'planned_depth_mm', 'short_by_mm',
                'contact_depth_mm', 'search_radius_mm', 'peak_force_N'):
        if key in control.record:
            print(f'  {key}: {control.record[key]}')

    folder = args.out or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'bench_insertions',
        f'{datetime.now():%Y%m%d-%H%M%S}-{args.arm}')
    save_run(control, folder, {'arm': args.arm, 'ip': ip, 'args': vars(args),
                               'axis_base': [float(v) for v in axis]})
    print(f'saved -> {folder}')
    rtde_c.stopScript()
    return 0 if control.record.get('seated') else 1


if __name__ == '__main__':
    sys.exit(main())
