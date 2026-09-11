#!/usr/bin/env python3
"""Drive the whole engine through an insertion, with fake arms.

Builds a short trajectory by cutting one real mate out of the planner's
open-loop file, hands the engine a fake RTDE pair for each arm, and checks the
run goes:

    track -> INSERT -> release -> track -> done

with the holding arm standing still through the insertion, the gripper opening
only after the part seats, and the log carrying the wrench, the per-insertion
record and its plot. Then it runs the same file again with the insertion
forced to fail, and checks the engine parks in `held` instead of opening the
gripper on a part that is not in its joint.

! This opens the engine's real windows for a few seconds -- it drives the
! shipped node, not a stub.

Run (from the workspace root, with the venv and install sourced):

    python3 src/husky-assembly-teleop/scripts/test_open_loop_engine_flow.py
"""

import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from fake_rtde import FakeControl, FakeReceive, FakeState  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SOURCE_TRAJ = ('/home/su/Insync/2025-03 Husky Assembly/data_experiment/'
               'fixtureless_assembly_trajs/open-loop.json')
# The window around the obj_1 mate: its funnel is samples 436..467.
SLICE_FROM, SLICE_TO = 400, 500
# t 5.0..15.0 s of the real file: nothing held at its start, then the FIRST
# picks of obj_0 (right arm, slice t=1.70 s) and obj_1 (left, 8.85 s), no opens.
PICK_FROM, PICK_TO = 100, 300

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


def build_slice(out_dir: str, lo: int = SLICE_FROM, hi: int = SLICE_TO,
                stem: str = 'flow') -> tuple:
    """Cut a window of the real trajectory into a short standalone file.

    Args:
        out_dir (str): Where to write the trajectory and its plan.
        lo (int): First sample of the window.
        hi (int): One past the last sample of the window.
        stem (str): File-name stem, so two windows can live side by side.

    Returns:
        tuple: (trajectory path, plan path).
    """
    with open(SOURCE_TRAJ) as f:
        data = json.load(f)
    samples = data['samples'][lo:hi]
    t0 = samples[0]['time']
    for i, sample in enumerate(samples):
        sample['time'] = round(i * data['dt'], 6)
    data['samples'] = samples
    os.makedirs(out_dir, exist_ok=True)
    traj_path = os.path.join(out_dir, f'{stem}_traj.json')
    with open(traj_path, 'w') as f:
        json.dump(data, f)
    plan_path = os.path.join(out_dir, f'{stem}_plan.json')
    with open(plan_path, 'w') as f:
        json.dump({'config': {'assembly': 'real-stool', 'pre_insertion': 0.05},
                   'plan': [['assemble', 'a2', 'obj_1', 'obj_0'],
                            ['release', 'a2', 'obj_1']]}, f)
    print(f'   sliced samples {lo}..{hi} (t0={t0:.2f}s) '
          f'-> {len(samples)} samples, {samples[-1]["time"]:.2f}s')
    return traj_path, plan_path


def make_args(traj_path: str, plan_path: str, **overrides):
    """The engine's parsed-argument object, with test-friendly defaults.

    Args:
        traj_path (str): Trajectory json.
        plan_path (str): Plan json.
        **overrides: Any argument to change.

    Returns:
        object: Something with the attributes `OpenLoopEngine` reads.
    """
    fields = dict(
        traj_json=traj_path, execute=True, swap_arms=True,
        robot_name='/a200_0806', left_ip='fake', right_ip='fake',
        frequency=125.0, p_gain=[1.0, 1.0], joint_accel=3.0,
        max_joint_vel=2.0, start_tol=0.05, err_abort=0.35, no_gripper=True,
        approach_vel=0.25, no_table=False, table_top_z=0.245,
        layout_json=None, plan_json=plan_path, end_sample=0,
        insertions=True, ins_push_force=None, ins_contact_force=None,
        ins_guard_force=None, ins_search_radius=None, ins_search_pitch=None,
        ins_approach_speed=None, ins_insert_speed=None, ins_budget=8.0,
        ins_min_depth=1.0, release_wait=0.5, resume_blend=1.0,
        no_grasp_abort=False, stop_after_grasp=False, no_wrench_log=False,
        no_zero_ft=False)
    fields.update(overrides)
    return type('Args', (), fields)()


def attach_fakes(node, contacts=(None, None), realtime=True):
    """Give the node a fake RTDE pair per arm, already at the start pose.

    Args:
        node (OpenLoopEngine): The node under test.
        contacts (tuple): Per arm, a VirtualMortise or None.
        realtime (bool): Run the fakes on the wall clock (the engine needs it).

    Returns:
        list: The two FakeState objects.
    """
    states = []
    for i in range(2):
        state = FakeState(q0=node.traj.q12[0, 6 * i:6 * i + 6],
                          origin=(0.5, 0.0, 0.5), scale=1.0,
                          contact=contacts[i], realtime=realtime)
        node.rtde_r[i] = FakeReceive(state)
        node.rtde_c[i] = FakeControl(state, node.args.frequency)
        states.append(state)
    node.state = 'connected'
    return states


def spin_until(node, done, timeout: float, rclpy) -> bool:
    """Spin the node until a predicate holds or the time runs out.

    Args:
        node (OpenLoopEngine): The node to spin.
        done (callable): Called with no arguments; stop when it is true.
        timeout (float): Seconds to wait.
        rclpy: The rclpy module.

    Returns:
        bool: Whether the predicate became true.
    """
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.02)
        if done():
            return True
    return False


def test_flow(traj_path: str, plan_path: str, rclpy, ole):
    """A whole run: track, insert, release, track, done.

    Args:
        traj_path (str): The sliced trajectory.
        plan_path (str): Its plan.
        rclpy: The rclpy module.
        ole: The open_loop_engine module.
    """
    print('\n--- 1. the whole chain, insertion succeeds ---')
    node = ole.OpenLoopEngine(make_args(traj_path, plan_path))
    try:
        check(len(node.insertions) == 1,
              f'the engine finds the one mate in the slice '
              f'(got {len(node.insertions)})')
        if not node.insertions:
            return
        ins = node.insertions[0]
        attach_fakes(node)
        node.on_check_start_pose()
        check(node.state == 'ready', f'start pose accepted (got {node.state})')

        node.on_start()
        kinds = [p['kind'] for p in node.phases]
        check(kinds == ['track', 'insert', 'release', 'track'],
              f'the run is cut into track/insert/release/track (got {kinds})')

        seen = set()

        def watch():
            seen.add(node.state)
            return node.state in ('done', 'aborted', 'held')

        spin_until(node, watch, 60.0, rclpy)
        print(f'   final state: {node.state}, states seen: {sorted(seen)}')
        print(f'   thread errors: {node.thread_error}')
        check(node.state == 'done', f'the run finishes (got {node.state})')
        check('inserting' in seen, 'it went through an insertion phase')
        check('releasing' in seen, 'it went through a release phase')

        check(len(node.insertion_records) == 1,
              f'one insertion is on the record (got {len(node.insertion_records)})')
        record = node.insertion_records[0]
        print(f'   insertion: {record["outcome"]}, '
              f'{record["reached_depth_mm"]:.1f}/'
              f'{record["planned_depth_mm"]:.1f} mm')
        check(record['seated'], f'the insertion seated ({record["outcome"]})')
        check(record['child'] == 'obj_1' and record['parent'] == 'obj_0',
              'the record names the parts it mated')

        opened = [e for e in node.fired_events if e['kind'] == 'open']
        check(len(opened) == 1 and opened[0]['arm'] == ole.ARM_SIDES[ins.arm_index],
              f'the gripper opened once, on the inserting arm (got {opened})')

        folder = node.last_run_folder
        check(bool(folder) and os.path.isdir(folder), 'a run folder was saved')
        if folder:
            log = np.load(os.path.join(folder, 'log.npz'))
            for side in ole.ARM_SIDES:
                check(f'{side}_wrench' in log and len(log[f'{side}_wrench']),
                      f'{side} arm wrench is in log.npz '
                      f'({len(log.get(f"{side}_wrench", []))} cycles)')
            check('ins0_wrench_lp' in log,
                  'the insertion has its own full-rate log')
            side = ole.ARM_SIDES[ins.holder_index]
            hold = log[f'{side}_mode']
            check(hold.max() == 1,
                  'the holding arm is marked as holding in the log')
            # The whole point of the hold: that arm must not move while the
            # other one presses a part into what it is carrying.
            held_q = log[f'{side}_q_actual'][hold == 1]
            drift = (float(np.abs(held_q - held_q[0]).max())
                     if len(held_q) else float('inf'))
            check(drift < 0.01,
                  f'the {side} arm stands still while holding '
                  f'({drift * 1000:.2f} mrad of drift over {len(held_q)} cycles)')
            check(os.path.exists(os.path.join(folder, 'insertions.json')),
                  'insertions.json was written')
            check(os.path.exists(os.path.join(folder, 'insertion_0.png')),
                  'the insertion got its own plot')
            info = json.load(open(os.path.join(folder, 'run_info.json')))
            check(info['outcome'] == 'done',
                  f'run_info records the outcome (got {info["outcome"]})')
    finally:
        node.destroy_node()


def test_held_on_failure(traj_path: str, plan_path: str, rclpy, ole):
    """A failed insertion must park in `held`, not open the gripper.

    Args:
        traj_path (str): The sliced trajectory.
        plan_path (str): Its plan.
        rclpy: The rclpy module.
        ole: The open_loop_engine module.
    """
    print('\n--- 2. the insertion fails: park and ask ---')
    # A budget far too short for the skill to finish is the simplest failure
    # that does not depend on the contact model.
    node = ole.OpenLoopEngine(make_args(traj_path, plan_path, ins_budget=0.5))
    try:
        attach_fakes(node)
        node.on_check_start_pose()
        node.on_start()
        reached = spin_until(node, lambda: node.state in ('held', 'done',
                                                          'aborted'), 60.0, rclpy)
        print(f'   final state: {node.state}')
        check(reached and node.state == 'held',
              f'a failed insertion parks in held (got {node.state})')
        check(not any(e['kind'] == 'open' for e in node.fired_events),
              'no gripper was opened on the unseated part')
        check(bool(node.insertion_records)
              and not node.insertion_records[0]['seated'],
              'the failure is on the record')

        # The operator chooses to carry on anyway.
        node.on_release_and_continue()
        spin_until(node, lambda: node.state in ('done', 'aborted'), 40.0, rclpy)
        print(f'   after "release & continue": {node.state}')
        check(node.state == 'done',
              f'"release & continue" finishes the run (got {node.state})')
        check(any(e['kind'] == 'open' for e in node.fired_events),
              'that choice did open the gripper')
        check(node.insertion_records[0].get('operator') == 'released_unseated',
              'the record says the operator released it unseated')
    finally:
        node.destroy_node()


def test_default_path_unchanged(traj_path: str, plan_path: str, rclpy, ole):
    """WITHOUT --insertions the run is the plain speedJ tracking it always was.

    This is the guarantee the flag exists for: the mates are replayed
    open-loop, there is one tracking phase, and no insertion machinery runs.
    The force record around the mate is still written, which is the whole
    point of it: it is there to measure an open-loop insertion.

    Args:
        traj_path (str): The sliced trajectory.
        plan_path (str): Its plan.
        rclpy: The rclpy module.
        ole: The open_loop_engine module.
    """
    print('\n--- 3. no --insertions: nothing changes ---')
    node = ole.OpenLoopEngine(make_args(traj_path, plan_path, insertions=False))
    try:
        check(node.insertions == [],
              f'no mates are taken over (got {len(node.insertions)})')
        attach_fakes(node)
        node.on_check_start_pose()
        node.on_start()
        check([p['kind'] for p in node.phases] == ['track'],
              f'the run is a single tracking phase (got '
              f'{[p["kind"] for p in node.phases]})')
        spin_until(node, lambda: node.state in ('done', 'aborted'), 40.0, rclpy)
        print(f'   final state: {node.state}')
        check(node.state == 'done', f'it tracks to the end (got {node.state})')
        check(node.insertion_records == [], 'no insertion was recorded')
        opened = [e for e in node.fired_events if e['kind'] == 'open']
        check(len(opened) == 1,
              f'the mate\'s gripper-open still fires open-loop (got {len(opened)})')
        folder = node.last_run_folder
        check(bool(folder)
              and not os.path.exists(os.path.join(folder, 'insertions.json')),
              'no insertions.json is written for a plain run')

        # The wrench log rides on the plan's mates, not on --insertions.
        path = os.path.join(folder, 'insertion_wrench.json')
        check(os.path.exists(path), 'insertion_wrench.json is written anyway')
        if os.path.exists(path):
            data = json.load(open(path))
            recs = data['insertions']
            check(len(recs) == 1,
                  f'the slice\'s one mate is recorded (got {len(recs)})')
            rec = recs[0]
            check(rec['child'] == 'obj_1' and rec['parent'] == 'obj_0',
                  f'it names the parts (got {rec["child"]} -> {rec["parent"]})')
            check(rec['inserting_arm'] == 'left'
                  and rec['holding_arm'] == 'right',
                  f'and which arm did what (got {rec["inserting_arm"]} into '
                  f'{rec["holding_arm"]})')
            check(not rec['ran_compliant'],
                  'it is marked as an open-loop mate, not a compliant one')
            check(set(rec['arms']) == set(ole.ARM_SIDES),
                  f'BOTH arms are logged (got {sorted(rec["arms"])})')
            for side, arm in rec['arms'].items():
                n = len(arm['t_s'])
                check(n > 100
                      and len(arm['force_N']) == n
                      and len(arm['torque_Nm']) == n
                      and all(len(row) == 3 for row in arm['force_N'][:5]),
                      f'{side}: {n} samples of 3-axis force and torque')
                span = arm['t_s'][-1] - arm['t_s'][0]
                check(arm['t_s'][0] <= rec['pre_insertion_t_s']
                      and arm['t_s'][-1] >= rec['release_t_s']
                      and span > 1.0,
                      f'{side}: the window covers pre-insertion '
                      f'{rec["pre_insertion_t_s"]:.2f}s to release '
                      f'{rec["release_t_s"]:.2f}s ({span:.2f}s logged)')
            png = os.path.join(folder,
                               f'insertion_wrench_{rec["index"]}_'
                               f'{rec["child"]}.png')
            check(os.path.exists(png) and os.path.getsize(png) > 5000,
                  f'its figure is rendered ({os.path.basename(png)})')
    finally:
        node.destroy_node()


def test_first_pick(olt):
    """`first_pick_part` flags exactly the five first picks of the real file.

    Pins the rule the marking stop relies on, with no GUI: the 4 handover
    receives, 3 re-closes on an already-held part and the 1 re-pick of the
    assembly must all come back None.

    Args:
        olt: The open_loop_traj module.
    """
    print('\n--- 0b. which closes are first picks ---')
    traj = olt.load_open_loop_traj(SOURCE_TRAJ, swap_arms=True)
    picks = {(round(ev.time, 2), traj.first_pick_part(ev))
             for ev in traj.events if traj.first_pick_part(ev)}
    expected = {(6.7, 'obj_0'), (13.85, 'obj_1'), (34.45, 'obj_2'),
                (85.8, 'obj_3'), (105.85, 'obj_4')}
    check(picks == expected,
          f'the five first picks, and only those, are flagged '
          f'(got {sorted(picks)})')
    closes = [ev for ev in traj.events if ev.kind == 'close']
    quiet = sum(traj.first_pick_part(ev) is None for ev in closes)
    check(len(closes) == 13 and quiet == 8,
          f'the other 8 closes (handovers, re-closes, re-pick) are not '
          f'({quiet} of {len(closes)})')
    check(all(traj.first_pick_part(ev) is None
              for ev in traj.events if ev.kind == 'open'),
          'an open is never a pick')


def test_traj_clock(olt):
    """The shared clock maps wall time to trajectory time at its scale.

    Args:
        olt: The open_loop_traj module.
    """
    print('\n--- 0a. the shared trajectory clock ---')
    c = olt.TrajClock()
    check(c.now() == 0.0 and not c.started and c.scale == 1.0,
          'an unstarted clock reads 0 at scale 1')
    c.start(0.0, delay_s=0.5)
    check(-0.51 < c.now() < -0.45,
          f'a pre-roll puts the clock BEFORE the start ({c.now():.3f} s), which '
          f'is what makes both arms hold the start pose')

    c.start(0.0, 0.0, 0.5)
    time.sleep(0.2)
    half = c.now()
    check(0.085 < half < 0.115,
          f'at scale 0.5, 0.2 s of wall time is {half:.4f} s of trajectory time')

    before = c.now()
    c.set_scale(2.0)
    after = c.now()
    check(abs(after - before) < 1e-4,
          f'changing the scale does not move the clock ({abs(after - before) * 1e6:.1f} us)')
    check(c.scale == 2.0, 'the new scale is in force')
    t0 = c.now()
    time.sleep(0.1)
    check(0.17 < c.now() - t0 < 0.23,
          f'after the change it runs at the new rate ({c.now() - t0:.4f} s in 0.1 s)')

    # Two "arms" reading the same object must agree, which is the whole
    # synchronization argument.
    readings = [c.now() for _ in range(2)]
    check(abs(readings[0] - readings[1]) < 1e-3,
          'two readers of one clock agree')

    # --- the pause ramp: the rate walks down, trajectory time stays smooth ---
    c.start(0.0, 0.0, 1.0)
    time.sleep(0.1)
    before = c.now()
    c.ramp_to(0.0, 0.4)
    check(abs(c.now() - before) < 1e-4 and abs(c.scale - 1.0) < 0.02,
          'a ramp starts from the rate and the time the clock already had')
    time.sleep(0.2)
    check(0.3 < c.scale < 0.7,
          f'halfway down the ramp the rate is about half (x{c.scale:.2f})')
    check(c.ramping, 'the ramp reports itself as running')
    time.sleep(0.3)
    check(c.scale == 0.0 and not c.ramping,
          f'the ramp ends at a standstill (x{c.scale:.2f})')
    stopped = c.now()
    # The trajectory covers half the ramp's worth of time on the way down --
    # the area under a rate falling linearly from 1 to 0 over 0.4 s.
    check(abs(stopped - before - 0.2) < 0.03,
          f'braking covers half the ramp in trajectory time '
          f'({stopped - before:.3f} s, 0.2 expected)')
    time.sleep(0.2)
    check(abs(c.now() - stopped) < 1e-6,
          f'a stopped clock does not move ({abs(c.now() - stopped):.2e} s)')

    # Reversing mid-ramp must turn around from where the rate is, not jump.
    c.ramp_to(1.0, 0.4)
    time.sleep(0.2)
    mid_rate, mid_t = c.scale, c.now()
    c.ramp_to(0.0, 0.4)
    check(abs(c.scale - mid_rate) < 0.05 and abs(c.now() - mid_t) < 1e-3,
          f'reversing mid-ramp keeps both the rate and the time continuous '
          f'(x{mid_rate:.2f} -> x{c.scale:.2f})')


def test_grasp_verdict(ole):
    """The gripper result is read the way the Robotiq behaves.

    Args:
        ole: The open_loop_engine module.
    """
    print('\n--- 0. grasp verdicts from the action result ---')
    v = ole.grasp_verdict
    check(v('close', stalled=True, reached_goal=False, position=0.59) == 'GRASPED',
          'a close that stalls at leg width (0.59 rad) is a grasp')
    check(v('close', stalled=False, reached_goal=True, position=0.79) == 'MISSED',
          'a close that reaches the fully-closed target met nothing')
    # Measured on Cindy 2026-09-09: an EMPTY close stops at 0.7894 rad and the
    # controller still says stalled, because 0.8 overshoots the fingers' limit.
    check(v('close', stalled=True, reached_goal=False, position=0.7894) == 'MISSED',
          'a close that stalls at the fingers\' own limit (0.789 rad) met nothing')
    check(v('close', stalled=False, reached_goal=False) == 'UNKNOWN',
          'a close with neither flag is not judged')
    check(v('open', stalled=False, reached_goal=True) == 'OPENED',
          'an open that reaches its target opened')
    check(v('open', stalled=True, reached_goal=False) == 'BLOCKED',
          'an open that stalls was blocked')
    check(abs(ole.jaw_width_mm(0.0) - 85.0) < 1e-9 and ole.jaw_width_mm(0.8) == 0.0,
          'jaw width runs from 85 mm open to 0 mm closed')
    check(20.0 < ole.jaw_width_mm(0.6) < 25.0,
          f'a leg-sized stall reads as a leg ({ole.jaw_width_mm(0.6):.1f} mm at 0.6 rad)')


def test_speed_scale(traj_path: str, plan_path: str, rclpy, ole):
    """Halving the execution speed doubles the wall time and keeps lockstep.

    Args:
        traj_path (str): The sliced trajectory.
        plan_path (str): Its plan.
        rclpy: The rclpy module.
        ole: The open_loop_engine module.
    """
    print('\n--- 4. execution speed x0.5 ---')
    node = ole.OpenLoopEngine(make_args(traj_path, plan_path, insertions=False))
    try:
        attach_fakes(node)
        node.on_check_start_pose()
        # Drive the real widget, not the attribute: _tick_speed reads the
        # slider live every tick and would otherwise overwrite anything set
        # behind its back.
        import husky_assembly_teleop.common as _common
        _common._global_backend.set_value(node.scale_slider._handle, 0.5)
        node._tick_speed()
        check(node.speed_scale == 0.5,
              f'the slider is picked up before the run (got {node.speed_scale})')
        node.on_start()
        started = time.time()
        spin_until(node, lambda: node.state in ('done', 'aborted'), 60.0, rclpy)
        wall = time.time() - started
        print(f'   final state: {node.state}, wall time {wall:.1f} s')
        check(node.state == 'done', f'the run finishes (got {node.state})')

        # The file is 4.95 s; at half speed that is ~9.9 s of trajectory time,
        # plus the 0.5 s pre-roll and the brake + end settle (trajectory
        # seconds too, so they stretch as well) -- about 13 s nominally, vs
        # ~7 s at full speed. The upper bound is deliberately loose: these
        # tests open real GUI windows and a busy machine stretches them.
        check(8.0 < wall < 25.0,
              f'it takes about twice as long ({wall:.1f} s for a 4.95 s file, '
              f'~13 s nominal, ~7 s at x1)')

        folder = node.last_run_folder
        log = np.load(os.path.join(folder, 'log.npz'))
        for side in ole.ARM_SIDES:
            scales = log[f'{side}_speed_scale']
            check(len(scales) and scales.max() <= 0.5 + 1e-9,
                  f'{side} arm ran at the reduced scale (max {scales.max():.2f})')
        ends = [log[f'{side}_t'][-1] for side in ole.ARM_SIDES]
        check(abs(ends[0] - ends[1]) < 0.1,
              f'both arms end within {abs(ends[0] - ends[1]) * 1000:.0f} ms of '
              f'each other in trajectory time -- still lockstep')

        fired = [e for e in node.fired_events if e['kind'] == 'open']
        check(len(fired) == 1 and abs(fired[0]['fired_t'] - 3.35) < 0.2,
              f'the gripper event still fires at its planned trajectory time '
              f'(got {[e["fired_t"] for e in fired]}, planned 3.35)')
        info = json.load(open(os.path.join(folder, 'run_info.json')))
        check(info['speed_scale_at_start'] == 0.5,
              'run_info records the speed it ran at')
    finally:
        node.destroy_node()


def test_pause_resume(traj_path: str, plan_path: str, rclpy, ole):
    """Space freezes the run mid-motion, and a second Space finishes it.

    Checks the ramp reaches zero, that the clock and both arms then stand
    still, and that resuming carries the run to a normal finish still in
    lockstep.

    Args:
        traj_path (str): The sliced trajectory.
        plan_path (str): Its plan.
        rclpy: The rclpy module.
        ole: The open_loop_engine module.
    """
    print('\n--- 5. Space pause / resume ---')
    # The marking stop is ON here on purpose: this slice's only close re-grips
    # a part the arm already holds, which must NOT stop the run -- the checks
    # below expect exactly one pause, the operator's.
    node = ole.OpenLoopEngine(make_args(traj_path, plan_path, insertions=False,
                                        stop_after_grasp=True))
    try:
        attach_fakes(node)
        # Before a run there is nothing to pause: the toggle must refuse.
        node.on_pause_toggle()
        check(not node.paused,
              'pausing before START is refused (state connected)')

        node.on_check_start_pose()
        node.on_start()
        moving = spin_until(node, lambda: node.clock.now() > 1.0, 20.0, rclpy)
        check(moving, 'the run reaches t=1.0 s before being paused')
        node.on_pause_toggle()
        check(node.paused, 'the toggle registers the pause')

        # ! Halfway through the ramp WITHOUT spinning the node: the arms must
        # ! be slowing down even though the UI has not ticked once since the
        # ! key was pressed. DearPyGui really does stall this loop for a second
        # ! at a time, so a ramp that needed the UI would be an abrupt stop.
        time.sleep(0.5 * ole.PAUSE_RAMP_S)
        mid = node.clock.scale
        check(0.15 < mid < 0.85,
              f'the rate is partway down without a single UI tick '
              f'(x{mid:.2f} halfway through the ramp)')
        check(node.clock.ramping, 'the clock reports the ramp still running')

        # One wall second: the 0.5 s ramp has finished with room to spare.
        spin_until(node, lambda: False, 1.0, rclpy)
        check(node.clock.scale == 0.0,
              f'the shared clock is frozen (scale {node.clock.scale:.3f})')
        check(not node.clock.ramping, 'the ramp is over')

        # Nothing may move while paused: not the clock, not the arms.
        t1 = node.clock.now()
        q1 = np.concatenate([np.asarray(q) for q in node.live_q])
        spin_until(node, lambda: False, 0.5, rclpy)
        drift = float(np.abs(np.concatenate(
            [np.asarray(q) for q in node.live_q]) - q1).max())
        check(abs(node.clock.now() - t1) < 1e-6,
              f'trajectory time stands still ({abs(node.clock.now() - t1):.2e} s '
              f'over half a second of wall time)')
        check(drift < 5e-3,
              f'both arms hold their pose while paused ({drift * 1000:.2f} mrad)')

        node.on_pause_toggle()
        check(not node.paused, 'the second press resumes')
        spin_until(node, lambda: node.state in ('done', 'aborted'), 60.0, rclpy)
        print(f'   final state: {node.state}')
        check(node.state == 'done', f'the run finishes (got {node.state})')

        folder = node.last_run_folder
        log = np.load(os.path.join(folder, 'log.npz'))
        for side in ole.ARM_SIDES:
            scales = log[f'{side}_speed_scale']
            check(len(scales) and scales.min() == 0.0,
                  f'{side} arm was commanded a frozen clock (min '
                  f'{scales.min():.2f})')
            check(len(scales) and abs(scales[-1] - 1.0) < 1e-6,
                  f'{side} arm ends back at full speed ({scales[-1]:.2f})')
            # The ramp is what makes it gentle: the arm must see a spread of
            # part-speeds on the way down and up, not one step to a standstill.
            # Two 0.5 s ramps at 125 Hz are ~125 cycles; allow for scheduling.
            between = int(((scales > 0.02) & (scales < 0.98)).sum())
            check(between > 60,
                  f'{side} arm was walked down and back up gradually '
                  f'({between} cycles at a part-speed, ~125 expected)')
        ends = [log[f'{side}_t'][-1] for side in ole.ARM_SIDES]
        check(abs(ends[0] - ends[1]) < 0.1,
              f'both arms end within {abs(ends[0] - ends[1]) * 1000:.0f} ms of '
              f'each other in trajectory time -- still lockstep')

        info = json.load(open(os.path.join(folder, 'run_info.json')))
        pauses = info['pauses']
        check(len(pauses) == 1 and pauses[0]['resumed_at'] is not None
              and pauses[0]['resumed_at'] >= pauses[0]['paused_at'],
              f'run_info records the pause and its resume (got {pauses})')
        check(info['pause_ramp_s'] == ole.PAUSE_RAMP_S,
              'run_info records the ramp length')
    finally:
        node.destroy_node()


def test_stop_after_grasp(traj_path: str, plan_path: str, rclpy, ole):
    """The run freezes after each first pick, and DONE carries it on.

    On the pick slice that is twice: obj_0 (right) at 1.70 s and obj_1 (left)
    at 8.85 s. Each stop must be instant, land where the close fired, hold
    both arms at the grasp pose and clear on DONE; the run then ends
    normally, still in lockstep, with both stops on the record.

    Args:
        traj_path (str): The pick slice.
        plan_path (str): Its (unused) plan.
        rclpy: The rclpy module.
        ole: The open_loop_engine module.
    """
    print('\n--- 6. stop after each first pick ---')
    node = ole.OpenLoopEngine(make_args(traj_path, plan_path, insertions=False,
                                        stop_after_grasp=True))
    try:
        attach_fakes(node)
        node.on_check_start_pose()
        node.on_start()
        for part, side, planned in (('obj_0', 'right', 1.70),
                                    ('obj_1', 'left', 8.85)):
            stopped = spin_until(node, lambda: node.paused, 20.0, rclpy)
            check(stopped, f'the run stops itself for {part}')
            if not stopped:
                break
            rec = node.fired_events[-1]
            check(rec['kind'] == 'close' and rec['arm'] == side
                  and abs(rec['planned_t'] - planned) < 1e-6,
                  f'...right after the {side} close planned at {planned:.2f}s '
                  f'(got {rec["kind"]} {rec["arm"]} @{rec["planned_t"]:.2f}s)')
            check(node.clock.scale == 0.0 and not node.clock.ramping,
                  'the freeze is instant (no ramp from a standstill)')
            check(abs(node.clock.now() - rec['fired_t']) < 0.02,
                  f'trajectory time is frozen where the close fired '
                  f'({node.clock.now():.3f} vs {rec["fired_t"]:.3f} s)')
            check(node.stop_reason == f'grasp of {part} ({side})',
                  f'the status names the part (got {node.stop_reason!r})')
            t1 = node.clock.now()
            spin_until(node, lambda: False, 0.5, rclpy)
            q_ref = node.traj.q12[node.traj.state_at(rec['planned_t'])]
            q_now = np.concatenate([np.asarray(q) for q in node.live_q])
            off = float(np.abs(q_now - q_ref).max())
            check(abs(node.clock.now() - t1) < 1e-6 and node.state == 'tracking',
                  'it stays frozen, still in the tracking state')
            check(off < 0.01,
                  f'both arms hold the grasp pose ({off * 1000:.1f} mrad off)')
            node.on_done()
            check(not node.paused and node.stop_reason is None,
                  'DONE resumes and clears the reason')
        spin_until(node, lambda: node.state in ('done', 'aborted'), 40.0, rclpy)
        check(node.state == 'done', f'the run finishes (got {node.state})')
        node.on_done()   # nothing to resume: must only warn
        check(not node.paused, 'DONE after the run only warns')

        folder = node.last_run_folder
        info = json.load(open(os.path.join(folder, 'run_info.json')))
        pauses = info['pauses']
        reasons = [p.get('reason') for p in pauses]
        check(reasons == ['grasp of obj_0 (right)', 'grasp of obj_1 (left)'],
              f'run_info records both marking stops (got {reasons})')
        check(all(p['resumed_at'] is not None
                  and p['resumed_at'] >= p['paused_at'] for p in pauses),
              'each stop has its resume')
        log = np.load(os.path.join(folder, 'log.npz'))
        ends = [log[f'{side}_t'][-1] for side in ole.ARM_SIDES]
        check(abs(ends[0] - ends[1]) < 0.1,
              f'both arms end within {abs(ends[0] - ends[1]) * 1000:.0f} ms of '
              f'each other in trajectory time -- still lockstep')
    finally:
        node.destroy_node()


def test_stop_after_grasp_off(traj_path: str, plan_path: str, rclpy, ole):
    """The same pick slice with the marking stop OFF runs straight through.

    Args:
        traj_path (str): The pick slice.
        plan_path (str): Its (unused) plan.
        rclpy: The rclpy module.
        ole: The open_loop_engine module.
    """
    print('\n--- 7. the pick slice with the marking stop off ---')
    node = ole.OpenLoopEngine(make_args(traj_path, plan_path, insertions=False))
    try:
        attach_fakes(node)

        # Zeroing the force sensors: allowed while parked, refused once the
        # arms are being driven (it would move the ground under the hold guard).
        check(node.on_zero_ft() is None and len(node.ft_zeros) == 1,
              f'the zero-FT button records one zeroing '
              f'(got {len(node.ft_zeros)})')
        rec = node.ft_zeros[-1]
        check(rec['ok'] and rec['reason'] == 'operator button',
              f'...and says it worked (got {rec["ok"]}, {rec["reason"]!r})')
        check(len(rec['before']) == 2 and len(rec['after']) == 2,
              'both arms are in the record')
        check(max(abs(v) for arm in rec['after'] for v in arm) < 1e-6,
              f'both sensors read zero afterwards (got {rec["after"]})')

        node.on_check_start_pose()
        node.on_start()
        node.on_zero_ft()
        check(len(node.ft_zeros) == 1,
              'zeroing is refused once the arms are running')
        spin_until(node, lambda: node.state in ('done', 'aborted'), 40.0, rclpy)
        check(node.state == 'done', f'the run finishes (got {node.state})')
        closes = [e for e in node.fired_events if e['kind'] == 'close']
        check(len(closes) == 2,
              f'both first picks still fire their close (got {len(closes)})')
        info = json.load(open(os.path.join(node.last_run_folder,
                                           'run_info.json')))
        check(info['pauses'] == [], 'nothing stopped the run')
        check(len(info['ft_zeros']) == 1,
              'run_info carries the zeroing that happened before it')
    finally:
        node.destroy_node()


def main():
    """Build the fixture, run both flows, report.

    ! PyBullet allows ONE gui connection per process and the engine opens one,
    ! so the two scenarios cannot share a process. Without --case this runs the
    ! first itself and the second as a subprocess of itself.
    """
    import argparse
    import subprocess

    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--case', type=int, choices=(1, 2, 3, 4, 5, 6, 7),
                     default=None,
                     help='run only one scenario (used for the subprocess)')
    args = cli.parse_args()

    import rclpy

    from husky_assembly_teleop import open_loop_engine as ole
    from husky_assembly_teleop import open_loop_traj as olt

    out_dir = os.path.join(HERE, 'test_data')
    traj_path, plan_path = build_slice(out_dir)
    pick_traj, pick_plan = build_slice(out_dir, PICK_FROM, PICK_TO, 'pick')
    rclpy.init()
    try:
        if args.case in (None, 1):
            test_traj_clock(olt)
            test_first_pick(olt)
            test_grasp_verdict(ole)
            test_flow(traj_path, plan_path, rclpy, ole)
        if args.case == 2:
            test_held_on_failure(traj_path, plan_path, rclpy, ole)
        if args.case == 3:
            test_default_path_unchanged(traj_path, plan_path, rclpy, ole)
        if args.case == 4:
            test_speed_scale(traj_path, plan_path, rclpy, ole)
        if args.case == 5:
            test_pause_resume(traj_path, plan_path, rclpy, ole)
        if args.case == 6:
            test_stop_after_grasp(pick_traj, pick_plan, rclpy, ole)
        if args.case == 7:
            test_stop_after_grasp_off(pick_traj, pick_plan, rclpy, ole)
    finally:
        rclpy.shutdown()

    if args.case is None:
        for case, what in ((2, 'the held-on-failure scenario'),
                           (3, 'the unchanged default path'),
                           (4, 'the execution speed scaling'),
                           (5, 'the Space pause/resume ramp'),
                           (6, 'the stop after each first pick'),
                           (7, 'the pick slice with the marking stop off')):
            print(f'\n(running scenario {case} in its own process)')
            other = subprocess.run([sys.executable, os.path.abspath(__file__),
                                    '--case', str(case)])
            if other.returncode:
                _failures.append(what)
    print(f'\n{"ALL CHECKS PASSED" if not _failures else "FAILURES: " + "; ".join(_failures)}')
    return 1 if _failures else 0


if __name__ == '__main__':
    sys.exit(main())
