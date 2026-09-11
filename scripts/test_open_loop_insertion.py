#!/usr/bin/env python3
"""Headless checks for the compliant insertion skill and the mate finder.

Drives the real `InsertionController` against the fake RTDE pair and a virtual
stool joint (30 x 22 mm tenon, 32 x 32 mm blind mortise, 25 mm deep), with the
hole displaced by an error the controller does not know about:

1. no error         -> goes straight in, no search;
2. 3 x 1 mm error   -> touches the rim, spirals, catches, seats;
3. 12 mm error      -> searches, gives up cleanly (search_exhausted);
4. fouled hole      -> jams, backs off once, then reports stalled;
5. guard            -> a hard obstruction aborts on the force guard;
6. `find_insertions` on the real trajectory finds the planner's mates.

Run (from the workspace root, with the venv and install sourced):

    python3 src/husky-assembly-teleop/scripts/test_open_loop_insertion.py
"""

import argparse
import os
import sys

import numpy as np

# The fake RTDE pair lives next to this script, not in the package.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from fake_rtde import FakeControl, FakeReceive, FakeState, VirtualMortise  # noqa: E402
from husky_assembly_teleop.open_loop_insertion import (  # noqa: E402
    InsertionController, InsertionParams, deadband, pose_error, spiral_offset)
from husky_assembly_teleop.open_loop_traj import (  # noqa: E402
    find_insertions, load_open_loop_traj)

# The joint geometry the search has to cope with, from the planner's own
# YandVStoolAssembly: a 30 x 22 mm leg end into a 32 x 32 mm blind pocket.
PEG_HALF = (0.015, 0.011)
HOLE_HALF = (0.016, 0.016)
POCKET_DEPTH = 0.0245
FUNNEL = 0.050

DEFAULT_TRAJ = ('/home/su/Insync/2025-03 Husky Assembly/data_experiment/'
                'fixtureless_assembly_trajs/open-loop.json')
HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PLAN = os.path.join(HERE, 'test_data', 'stool_plan.json')

_failures = []


def check(ok: bool, what: str):
    """Record one assertion, printing PASS or FAIL for it.

    Args:
        ok (bool): Whether the assertion held.
        what (str): What was being asserted.
    """
    print(f'{"PASS" if ok else "FAIL"}: {what}')
    if not ok:
        _failures.append(what)


def run_insertion(offset=(0.0, 0.0), floor_depth=None, params=None,
                  max_seconds=40.0, frequency=125.0) -> tuple:
    """Drive one whole insertion against the virtual joint.

    The tool starts at the funnel mouth, 50 mm above the rim, and the mate
    goes straight down; the hole is displaced by `offset`, which is exactly
    the pickup and grasp error the controller has to find.

    Args:
        offset (tuple): Lateral displacement of the real hole [m].
        floor_depth (float): Depth at which the pocket is blocked [m], or
            None for a clean pocket.
        params (InsertionParams): Tunables, or None for the defaults.
        max_seconds (float): Give up driving after this much fake time.
        frequency (float): Control rate [Hz].

    Returns:
        tuple: (controller, state) after the skill has ended.
    """
    axis = np.array([0.0, 0.0, -1.0])            # straight down
    mouth_point = np.array([0.7, 0.0, 0.50])     # rim of the mortise
    start_point = mouth_point - axis * FUNNEL    # 50 mm above it
    contact = VirtualMortise(mouth_point, axis, POCKET_DEPTH, hole=HOLE_HALF,
                             peg=PEG_HALF, offset=offset,
                             floor_depth=floor_depth)
    # Joints map to the TCP one-for-one (scale 1), so a configuration IS a
    # position and the controller's forward kinematics are exact.
    state = FakeState(q0=np.zeros(6), origin=start_point, scale=1.0,
                      contact=contact)
    rtde_c, rtde_r = FakeControl(state, frequency), FakeReceive(state)
    params = params or InsertionParams()
    params.tcp_offset = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)   # the fake TCP is tool0

    control = InsertionController(rtde_c, rtde_r, params, 1.0 / frequency,
                                  log_fn=lambda *_a: None)
    # The seated configuration is the mouth plus the pocket depth, in joints.
    q_start = np.zeros(6)
    q_open = np.zeros(6)
    q_open[:3] = (mouth_point + POCKET_DEPTH * axis) - start_point
    control.setup(q_start, q_open)

    while state.now < max_seconds:
        rtde_c.initPeriod()
        if not control.step(state.now):
            break
        rtde_c.waitPeriod(None)
    return control, state


def test_helpers():
    """The pure helpers behave as the control law assumes."""
    print('\n--- helpers ---')
    check(np.allclose(deadband([5.0, -5.0, 1.0], 4.0), [1.0, -1.0, 0.0]),
          'deadband removes the sensor noise band and keeps the excess')
    p_err, r_err = pose_error([1, 2, 3, 0, 0, 0], [1, 2, 0, 0, 0, 0])
    check(np.allclose(p_err, [0, 0, 3]) and np.allclose(r_err, 0),
          'pose_error points from the measured pose to the reference')
    # A spiral must sweep at a constant speed and grow by its pitch per turn.
    pitch = 0.0015
    radii = [spiral_offset(s, pitch)[1] for s in np.linspace(0, 0.05, 400)]
    steps = [np.linalg.norm(spiral_offset(s + 1e-4, pitch)[0]
                            - spiral_offset(s, pitch)[0])
             for s in np.linspace(0.002, 0.05, 200)]
    check(radii == sorted(radii), 'the spiral radius grows monotonically')
    check(max(steps) / min(steps) < 1.2,
          f'the spiral is walked at a near-constant speed '
          f'(step spread {max(steps) / min(steps):.2f}x)')
    check(spiral_offset(0.05, pitch)[1] < 0.010,
          'a 50 mm sweep stays within a 10 mm radius, as the pitch implies')


def test_straight_in():
    """With no error the part goes in without ever searching."""
    print('\n--- 1. perfectly aligned ---')
    control, _state = run_insertion(offset=(0.0, 0.0))
    print(f'   outcome={control.outcome} '
          f'depth={control.record["reached_depth_mm"]:.1f}/'
          f'{control.record["planned_depth_mm"]:.1f} mm')
    check(control.record['seated'], f'seats (outcome {control.outcome})')
    check(not any(p['phase'] == 'search' for p in control.phase_log),
          'never needs to search when it is already aligned')
    check(control.record['short_by_mm'] < 2.5,
          f'reaches full depth ({control.record["short_by_mm"]:.2f} mm short)')


def test_search_finds_it():
    """A few millimetres of error is found by the spiral and seated."""
    print('\n--- 2. 3 x 1 mm pickup error ---')
    control, _state = run_insertion(offset=(0.003, 0.001))
    phases = [p['phase'] for p in control.phase_log]
    print(f'   outcome={control.outcome} phases={phases} '
          f'caught at r={control.record["search_radius_mm"]:.1f} mm, '
          f'depth={control.record["reached_depth_mm"]:.1f} mm')
    check(control.record['seated'], f'seats (outcome {control.outcome})')
    check('search' in phases, 'the rim contact triggers a search')
    check(control.record['search_radius_mm'] < 8.0,
          f'the hole is found inside the search radius '
          f'({control.record["search_radius_mm"]:.1f} mm)')
    check(control.record['short_by_mm'] < 2.5,
          f'reaches full depth ({control.record["short_by_mm"]:.2f} mm short)')


def test_search_gives_up():
    """An error beyond the search radius fails cleanly, not silently."""
    print('\n--- 3. 12 mm error, outside the search radius ---')
    control, _state = run_insertion(offset=(0.012, 0.0))
    print(f'   outcome={control.outcome} '
          f'r={control.record["search_radius_mm"]:.1f} mm')
    check(control.outcome == 'search_exhausted',
          f'reports search_exhausted (got {control.outcome})')
    check(not control.record['seated'], 'does not claim to have seated')


def test_jam_backs_off():
    """A blocked pocket is retried once, then reported as a jam."""
    print('\n--- 4. pocket fouled 5 mm down ---')
    control, _state = run_insertion(offset=(0.0, 0.0), floor_depth=0.005)
    phases = [p['phase'] for p in control.phase_log]
    print(f'   outcome={control.outcome} phases={phases} '
          f'depth={control.record["reached_depth_mm"]:.1f} mm')
    check(control.outcome == 'stalled',
          f'reports the jam as stalled (got {control.outcome})')
    check(phases.count('backoff') == 1,
          f'backs off exactly once before giving up (got {phases.count("backoff")})')
    check(control.record['retried'], 'the record says it retried')


def test_guard_aborts():
    """A hard obstruction trips the force guard rather than pushing through."""
    print('\n--- 5. force guard ---')
    params = InsertionParams()
    params.guard_force = 20.0        # below the rim reaction, so it must trip
    params.push_force = 25.0
    control, _state = run_insertion(offset=(0.012, 0.0), params=params)
    print(f'   outcome={control.outcome} '
          f'peak={control.record["peak_force_N"]:.1f} N')
    check(control.outcome == 'wrench_guard',
          f'aborts on the guard (got {control.outcome})')
    check(control.record['peak_force_N'] >= params.guard_force,
          'the guard fired on a force at least as large as its threshold')


def test_find_insertions(traj_path: str, plan_path: str):
    """The mate finder locates the planner's assembles in a real trajectory.

    Args:
        traj_path (str): The open-loop trajectory json.
        plan_path (str): The planner's plan.json.
    """
    print('\n--- 6. find_insertions on the real trajectory ---')
    if not os.path.exists(traj_path):
        print(f'   SKIPPED: {traj_path} is not on this machine')
        return
    sys.path.insert(0, os.path.join(
        os.path.dirname(HERE), 'external', 'husky_assembly_tamp'))
    from husky_assembly_tamp.keyframe import ssik_inprocess

    traj = load_open_loop_traj(traj_path, swap_arms=True)
    found = find_insertions(traj, plan_path, ssik_inprocess.fk,
                            log=lambda m: print(f'   {m}'))
    for ins in found:
        print(f'   {ins.describe()}')
    check(len(found) >= 3, f'finds the plan\'s mates ({len(found)} of 4; the '
                           f'fourth is a 7.6 mm funnel, under the minimum)')
    check(all(ins.parent == 'obj_0' for ins in found),
          'every mate goes into the seat, as the plan says')
    check(all(0.010 <= ins.depth_m <= 0.055 for ins in found),
          f'every funnel is a plausible length '
          f'({[round(i.depth_m * 1000) for i in found]} mm)')
    check(all(ins.holder_index != ins.arm_index for ins in found),
          'the holding arm is always the other one')
    check(all(ins.i_start < ins.i_open for ins in found),
          'every funnel ends at its gripper-open')


def main():
    """Run every check and exit non-zero if any of them failed."""
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--traj', default=DEFAULT_TRAJ)
    cli.add_argument('--plan', default=DEFAULT_PLAN)
    args = cli.parse_args()

    test_helpers()
    test_straight_in()
    test_search_finds_it()
    test_search_gives_up()
    test_jam_backs_off()
    test_guard_aborts()
    test_find_insertions(args.traj, args.plan)

    print(f'\n{"ALL CHECKS PASSED" if not _failures else "FAILURES: " + "; ".join(_failures)}')
    return 1 if _failures else 0


if __name__ == '__main__':
    sys.exit(main())
