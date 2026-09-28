"""Derive (or fully plan) the bar transfer for one or many bars without the robot.

The headless twin of the monitor's "M1: Derive Start/Goal only (no RRT)" and
"Plan Movement" buttons for the bar-held transfer (classic role M1: the
loading pose -> the insertion approach). It builds the design problem's cell,
loads a bar action, applies the same setup the live monitor does (ground body
injected; optionally the already-built bars hidden), runs the start derivation
alone or the whole planner, and writes a run file the dashboard picks up.

* Both export schemas load: the legacy ``B6.json`` (M0..M4 in one file) and
* the split ``B6__J.json`` + ``B6__R.json`` (see ``bar_action_io``).

* Bars the export left without a base (robot base at the world origin) are
* first placed with the planner's own heuristic (``place_base``): stand behind
* the bar facing the insertion axis, then sample the walkable ground until the
* transfer -> insert -> retreat IK chain solves. The result is saved next to
* the export as ``<stem>.solved_keyframe.json`` and reused on the next run.

Usage:
    cd /home/su/ros2_ws
    source venv/bin/activate
    source install/setup.bash
    python src/husky-assembly-teleop/scripts/derive_m1_headless.py --bar B3
    python src/husky-assembly-teleop/scripts/derive_m1_headless.py --bar B3 --anchor back
    python src/husky-assembly-teleop/scripts/derive_m1_headless.py --bar B3 --export-scene-only
    python src/husky-assembly-teleop/scripts/derive_m1_headless.py --bar B3 --plan   # derive + RRT
    # every bar of a problem, built bars kept as obstacles, full plans:
    python src/husky-assembly-teleop/scripts/derive_m1_headless.py \
        --problem 260921_motion_sample --bar all --plan --no-hide-built
"""
import argparse
import os
import time
import traceback
import types

import numpy as np
import pybullet_planning as pp
from compas.data import json_dump
from husky_assembly_teleop import DESIGN_DATA_DIRECTORY, DESIGN_PROBLEM_NAME
from husky_assembly_teleop.bar_action_io import (
    list_bar_actions, movement_role, parse_bar_action, sibling_action_path,
)
from husky_assembly_teleop.cfab_session import CfabSession
from husky_assembly_teleop.dashboard.run_schema import runs_dir_default, scenes_dir_default
from husky_assembly_teleop.dashboard.run_writer import write_m1_run
from husky_assembly_teleop.dashboard.scene_export import ensure_scene_glb
from husky_assembly_teleop.husky_monitor import (
    CDFM_POSITION_RES, CDFM_ROTATION_RES, HUSKY_DUAL_ARM_HOME_CONF_12,
    HUSKY_DUAL_UR5e_JOINT_NAMES, M1_PLANNER_STAGE, HuskyMonitor,
)
from husky_assembly_teleop.m1_derive_report import print_m1_derivation_summary
from husky_assembly_teleop.m1_manual_start import manual_m1_start
from husky_assembly_teleop.utils import conf_from_12vec
from husky_assembly_tamp.keyframe.dual_arm_ik import resolve_arm_groups
from husky_assembly_tamp.keyframe.ik_keyframe import mm4_to_frame
from husky_assembly_tamp.keyframe.walkable_ground import (
    load_walkable_grounds, solve_chain_with_base_search,
)
from husky_assembly_tamp.motion_planner.api import (
    TOOL_LINK_LEFT, TOOL_LINK_RIGHT, _bar_body_id, _collect_obstacle_puids,
    _derive_constrained_start_for_plan, plan_constrained_dual_arm,
)

NAMES_12 = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]) + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])


class _Log:
    """The few logger calls the borrowed monitor helpers make."""

    def info(self, message):
        print('  [info]', message)

    def warn(self, message):
        print('  [WARN]', message)


# * ----------------------------------------------------------- action files
def roles_of(action) -> dict:
    """Map the classic roles ('M0'..'M4') to the movements that play them.

    Args:
        action: a parsed bar action (either schema).

    Returns:
        dict: ``{role: movement}``; movements without a role are left out.
    """
    roles = {}
    for mv in action.movements:
        role = movement_role(mv)
        if role and role not in roles:
            roles[role] = mv
    return roles


def action_files(problem_dir: str, bar: str) -> tuple:
    """Find a bar's transfer action: the clean export and, if any, its keyframe sidecar.

    Args:
        problem_dir (str): the design problem folder.
        bar (str): the bar name, e.g. ``'B3'``.

    Returns:
        tuple[str, str | None]: ``(clean_path, solved_keyframe_path_or_None)``.
    """
    folder = os.path.join(problem_dir, 'BarActions')
    for stem in (f'{bar}__J', bar):  # split export first, then legacy
        clean = os.path.join(folder, f'{stem}.json')
        if os.path.isfile(clean):
            sidecar = os.path.join(folder, f'{stem}.solved_keyframe.json')
            return clean, (sidecar if os.path.isfile(sidecar) else None)
    raise FileNotFoundError(f'no {bar}__J.json or {bar}.json under {folder}')


def all_bars(problem_dir: str) -> list:
    """Every bar with a clean transfer action file, in natural order (B3, B6, B12 ...)."""
    bars = []
    for name in list_bar_actions(os.path.join(problem_dir, 'BarActions')):
        stem = name[:-len('.json')]
        if '.' in stem or stem.endswith('__R'):
            continue  # sidecars and release files
        bars.append(stem[:-len('__J')] if stem.endswith('__J') else stem)
    return bars


def base_is_authored(state) -> bool:
    """False when the export left the robot base at the world origin, i.e. unplaced."""
    frame = getattr(state, 'robot_base_frame', None)
    return frame is not None and float(np.linalg.norm(frame.point)) > 1e-6


def _vec12(conf) -> list:
    """The twelve arm joint values of a configuration, left arm first."""
    return [float(conf[name]) for name in NAMES_12]


# * ------------------------------------------------------- base placement
def stamp_keyframes(action, base_frame, confs: dict, carry=None):
    """Write a found base and keyframe configurations onto an action, as the export does.

    Every movement gets the base frame. Each solved role's configuration becomes
    that movement's goal, and is carried forward as the start of every following
    movement (the screw-tool and manual steps hold the arms still) until the next
    solved role. Movements before the first solved role keep their empty start:
    the transfer's start is the planner's to fill.

    Args:
        action: the jointing / release / legacy action to edit in place.
        base_frame (Frame): the accepted robot base frame (metres).
        confs (dict): ``{role: 12-vector}`` for the solved roles.
        carry: the 12-vector the previous action ended on (release files start
            where the jointing file ended), or None.

    Returns:
        list | None: the 12-vector the action ends on, to carry into the next file.
    """
    for mv in action.movements:
        if mv.start_state is not None:
            mv.start_state.robot_base_frame = base_frame
            if carry is not None and mv.start_state.robot_configuration is None:
                mv.start_state.robot_configuration = conf_from_12vec(carry)
        role = movement_role(mv)
        if role in confs:
            if mv.target_configuration is None:
                mv.target_configuration = conf_from_12vec(confs[role])
            carry = confs[role]
    return carry


def place_base(session, stub, action, release, clean_path: str, problem_dir: str) -> tuple:
    """Stand the robot where the code's own heuristic puts it, and solve the keyframes there.

    Mirrors the headless planner's ``--solve-keyframes --base sample`` for one
    bar: the seed base stands ``config.IK_BASE_STANDOFF_MM`` behind the bar's
    ground projection facing the insertion axis (``derive_seed_base``), and rings
    of samples around it are tried until the transfer -> insert -> retreat IK
    chain solves at one of them (``solve_chain_with_base_search``). The base and
    the keyframe configurations are then stamped onto the action(s) and saved as
    ``<stem>.solved_keyframe.json`` sidecars next to the clean export.

    Args:
        session (CfabSession): the open cell session.
        stub: the monitor stand-in (for the ground injection helper).
        action: the transfer's action (jointing or legacy), edited in place.
        release: the paired release action (split export), or None.
        clean_path (str): the transfer action's clean export path.
        problem_dir (str): the design problem folder (for WalkableGround.json).

    Returns:
        tuple[Frame, list[str]]: the base frame and the sidecar paths written.

    Raises:
        RuntimeError: when the inputs are incomplete or no sampled base solves.
    """
    roles = roles_of(action)
    if release is not None:
        for role, mv in roles_of(release).items():
            roles.setdefault(role, mv)
    missing = [role for role in ('M1', 'M2', 'M3') if role not in roles]
    if missing:
        raise RuntimeError(f'cannot place the base: no movement plays {missing}')
    # The IK chain pushes each movement's start state to the cell, which insists
    # on the ground body the monitor adds.
    for act in (action, release):
        for mv in (act.movements if act is not None else []):
            HuskyMonitor._inject_ground_rigid_body_state(stub, mv.start_state)

    grounds = load_walkable_grounds(os.path.join(problem_dir, 'WalkableGround.json'))
    ground_ids = list(getattr(action, 'walkable_ground_ids', None) or grounds)
    soups = [grounds[g] for g in ground_ids if g in grounds]
    if not soups:
        raise RuntimeError(f'no walkable ground for ids {ground_ids} (have {list(grounds)})')

    # Bar centre = midpoint of the assembled tool0 targets; the base faces the
    # average tool0 +Z, the direction the bar is pushed into its joints.
    targets = roles['M2'].target_ee_frames or {}
    left, right = targets.get('left'), targets.get('right')
    if left is None or right is None:
        raise RuntimeError('the insertion movement has no left/right target frames')
    midpoint_mm = 500.0 * (np.asarray(left.point, dtype=float) + np.asarray(right.point, dtype=float))
    heading = np.asarray(left.zaxis, dtype=float) + np.asarray(right.zaxis, dtype=float)
    home = roles.get('M4')
    home12 = (_vec12(home.target_configuration)
              if home is not None and home.target_configuration is not None
              else [float(v) for v in HUSKY_DUAL_ARM_HOME_CONF_12])

    solved, base_mm = solve_chain_with_base_search(
        session.planner, {role: roles[role] for role in ('M1', 'M2', 'M3')}, soups,
        midpoint_mm, heading_dir_mm=heading, check_collision=True,
        groups=resolve_arm_groups(session.robot_cell), home_conf_12=home12)
    if solved is None:
        raise RuntimeError('no base on the walkable ground solves the '
                           'transfer -> insert -> retreat IK chain')
    base_frame = mm4_to_frame(base_mm)
    confs = {role: _vec12(solved[role].robot_configuration) for role in ('M1', 'M2', 'M3')}

    saved = []
    carry = stamp_keyframes(action, base_frame, confs)
    stem, ext = os.path.splitext(clean_path)
    json_dump(action, f'{stem}.solved_keyframe{ext}')
    saved.append(f'{stem}.solved_keyframe{ext}')
    if release is not None:
        stamp_keyframes(release, base_frame, confs, carry=carry)
        release_path = sibling_action_path(saved[0])
        json_dump(release, release_path)
        saved.append(release_path)
    print(f'[base] {os.path.basename(clean_path)}: heuristic base at '
          f'({base_frame.point.x:.3f}, {base_frame.point.y:.3f}, {base_frame.point.z:.3f}) m; '
          f'saved {", ".join(os.path.basename(p) for p in saved)}')
    return base_frame, saved


# * ------------------------------------------------------------- planning
def prepare_state(session, stub, roles: dict, hide_built: bool = True) -> tuple:
    """Bring the transfer's start state to exactly what the live monitor would plan from.

    Args:
        session (CfabSession): the open cell session.
        stub: the monitor stand-in (carries ``active_bar_name`` for the helpers).
        roles (dict): ``{role: movement}`` from ``roles_of``.
        hide_built (bool): hide the already-built bars, as the mocap-accuracy
            experiment does (they are reaching locations, not real obstacles).

    Returns:
        tuple: ``(transfer_start_state, goal_conf)``.
    """
    transfer = roles['M1']
    state = transfer.start_state
    # The cell carries a ground body the Rhino export does not, and compas_fab
    # insists the two agree.
    HuskyMonitor._inject_ground_rigid_body_state(stub, state)
    if hide_built:
        HuskyMonitor._hide_built_assembly_for_mocap(stub, state, sync_visibility=False)
        n_hidden = sum(1 for rb in state.rigid_body_states.values()
                       if getattr(rb, 'is_hidden', False))
        print(f'[setup] {n_hidden} of {len(state.rigid_body_states)} bodies hidden '
              f'(already-built bars)')
    if state.robot_configuration is None:
        state.robot_configuration = session.robot_cell.zero_full_configuration()
        for name, value in zip(NAMES_12, HUSKY_DUAL_ARM_HOME_CONF_12):
            state.robot_configuration[name] = float(value)
    # The goal is where the insertion starts (same as the monitor), else the
    # transfer's own authored goal.
    insert = roles.get('M2')
    goal_conf = (insert.start_state.robot_configuration
                 if insert is not None and insert.start_state is not None else None)
    if goal_conf is None:
        goal_conf = transfer.target_configuration
    if goal_conf is None:
        raise RuntimeError('no goal configuration for the transfer: neither the '
                           'insertion start nor the transfer target is set')
    return state, goal_conf


def plan_bar(session, stub, args, bar: str, problem_dir: str) -> dict:
    """Derive or plan one bar's transfer and record it; return a summary row.

    Args:
        session (CfabSession): the open cell session.
        stub: the monitor stand-in.
        args: the parsed command line.
        bar (str): the bar name, e.g. ``'B3'``.
        problem_dir (str): the design problem folder.

    Returns:
        dict: one row for the batch table (bar, file, base, result, time, run).
    """
    clean_path, sidecar = action_files(problem_dir, bar)
    load_path = sidecar or clean_path
    action = parse_bar_action(load_path)
    release_path = sibling_action_path(load_path)
    release = (parse_bar_action(release_path)
               if release_path and os.path.isfile(release_path) else None)
    roles = roles_of(action)
    if 'M1' not in roles:
        raise RuntimeError(f'{os.path.basename(load_path)} has no bar-held transfer movement')
    bar_name = f'bar_{getattr(action, "active_bar_id", None) or bar}'
    stub.active_bar_name = bar_name
    row = {'bar': bar, 'file': os.path.basename(load_path), 'base': 'authored'}
    print(f'\n========== {bar}: {os.path.basename(load_path)} ==========')

    # Nothing measures the base here (no robot, no mocap), so the run file says
    # which stand-in it used -- see run_schema.BASE_SOURCE_PLAIN. A sidecar IS
    # the base-placement search's own answer, saved on an earlier run.
    base_source = 'base_placement_heuristic' if sidecar else 'bar_action_file'
    if not base_is_authored(roles['M1'].start_state):
        started = time.perf_counter()
        base_frame, _saved = place_base(session, stub, action, release, clean_path, problem_dir)
        row['base'] = (f'heuristic ({base_frame.point.x:.2f}, {base_frame.point.y:.2f}) m '
                       f'in {time.perf_counter() - started:.0f} s')
        base_source = 'base_placement_heuristic'

    state, goal_conf = prepare_state(session, stub, roles, hide_built=not args.no_hide_built)
    planner = session.planner
    robot_puid = planner.client.robot_puid
    arm_joints = pp.joints_from_names(robot_puid, NAMES_12)
    planner.set_robot_cell_state(state)
    meta = dict(problem=args.problem, bar_action=os.path.splitext(os.path.basename(clean_path))[0],
                active_bar=bar_name, movement_id=getattr(roles['M1'], 'movement_id', None),
                home_anchor=args.anchor, source='headless', base_source=base_source,
                runs_dir=args.runs_dir, scenes_dir=args.scenes_dir)

    started = time.perf_counter()
    if args.manual_start:
        # * The human-in-the-loop start: no sweep, one collision-checked IK for
        # * the chosen bar pose (what the monitor's Confirm button does).
        parts = [p.strip() for p in args.manual_start.split(',')]
        anchor = parts[0]
        numbers = [float(v) for v in parts[1:]] + [0.0] * 4
        slide_m, roll_deg, perp1_m, perp2_m = numbers[:4]
        result = manual_m1_start(
            planner, state, bar_name, goal_conf, NAMES_12, robot_puid, arm_joints,
            pp.link_from_name(robot_puid, TOOL_LINK_LEFT), pp.link_from_name(robot_puid, TOOL_LINK_RIGHT),
            anchor=anchor, slide_m=slide_m, roll_deg=roll_deg, perp1_m=perp1_m, perp2_m=perp2_m)
        elapsed = time.perf_counter() - started
        pos = result.get('bar_mid_mb')
        where = (f"bar centre at {100 * pos[0]:+.0f} cm forward, {100 * pos[1]:+.0f} cm left, "
                 f"{100 * pos[2]:+.0f} cm up (anchor {result['anchor']}, perp axes {result['perp_axes']})"
                 if pos is not None else 'no bar pose')
        verdict = ('start found' + (' (re-seeded branch)' if result['reseeded'] else '')
                   if result['start_conf'] is not None else f"no start: {result['reason']}")
        print(f'[manual] {bar}: {verdict}; {where}; {elapsed:.2f} s')
        row['result'] = f'{verdict}; {where}'
        row['time'] = f'{elapsed:.2f} s'
        return row

    if args.plan:
        # * The whole transfer planner, exactly as the monitor's Plan Movement
        # * runs it on a fresh action: derive the start, then search for the path.
        print(f'[plan] {bar}, anchor {args.anchor or "all"}: derive + RRT, '
              f'up to {args.max_time:.0f} s each ...')
        plan_path, info = plan_constrained_dual_arm(
            planner, state, active_bar_id=bar_name, goal_conf=goal_conf, goal_ee_frames=None,
            stage=M1_PLANNER_STAGE, position_res=CDFM_POSITION_RES,
            rotation_res=CDFM_ROTATION_RES, max_time=args.max_time,
            derive_start=True, start_home_anchor=args.anchor)
        planner.set_robot_cell_state(state)
        elapsed = time.perf_counter() - started
        print(f'[plan] {elapsed:.1f} s; path found: {plan_path is not None}; '
              f'reason: {info.get("failure_reason")}; planner: {info.get("planner")}')
        row['result'] = (f'path, {len(plan_path)} waypoints ({info.get("planner")})'
                         if plan_path else f'no path: {info.get("failure_reason")}')
    else:
        print(f'[derive] {bar}, anchor {args.anchor or "all"}, up to 120 s ...')
        plan_path = None
        start_conf, _bar_start, _bar_goal, _goal_arr, _gl, _gr, info = \
            _derive_constrained_start_for_plan(
                planner, state,
                active_bar_id=bar_name,
                bar_body=_bar_body_id(planner, bar_name),
                obstacles=_collect_obstacle_puids(planner, exclude={bar_name}),
                robot_puid=robot_puid, arm_joints=arm_joints,
                tool_link_left=pp.link_from_name(robot_puid, TOOL_LINK_LEFT),
                tool_link_right=pp.link_from_name(robot_puid, TOOL_LINK_RIGHT),
                joint_names_12=NAMES_12,
                goal_conf=goal_conf, goal_ee_frames=None,
                random_seed=None, max_ik_attempts=20, bar_sweep_box=None,
                position_res=CDFM_POSITION_RES, rotation_res=CDFM_ROTATION_RES,
                home_anchor=args.anchor)
        planner.set_robot_cell_state(state)
        elapsed = time.perf_counter() - started
        print(f'[derive] {elapsed:.1f} s; start found: {start_conf is not None}; '
              f'reason: {info.get("failure_reason")}')
        row['result'] = ('start found' if start_conf is not None
                         else f'no start: {info.get("failure_reason")}')

    stage_t = info.get('stage_times') or {}
    row['time'] = f'{elapsed:.0f} s' + (
        ' (' + ', '.join(f'{k} {v:.0f}' for k, v in stage_t.items() if v >= 0.5) + ')'
        if stage_t else '')
    print_m1_derivation_summary(info)
    run_path = write_m1_run(planner, state, info, plan_path=plan_path, **meta)
    row['run'] = os.path.basename(run_path)
    return row


def print_batch_table(rows: list, path: str = None) -> None:
    """Print the per-bar outcomes as a markdown table, and save it if a path is given."""
    columns = ('bar', 'file', 'base', 'result', 'time', 'run')
    lines = ['| ' + ' | '.join(columns) + ' |', '|' + '---|' * len(columns)]
    for row in rows:
        lines.append('| ' + ' | '.join(str(row.get(c, '')) for c in columns) + ' |')
    text = '\n'.join(lines)
    print('\n' + text)
    if path:
        with open(path, 'w') as handle:
            handle.write(text + '\n')
        print(f'[batch] table saved to {path}')


def main():
    """Derive or plan the transfer for the requested bars and write their run files."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--bar', nargs='+', default=None,
                        help="bar name(s), e.g. B3 B6, or 'all' for every bar of the problem "
                             "(default: the problem's first bar)")
    parser.add_argument('--anchor', default=None,
                        choices=[None, 'horizontal', 'vertical', 'back'],
                        help='restrict to one carry anchor (default: all three)')
    parser.add_argument('--problem', default=DESIGN_PROBLEM_NAME)
    parser.add_argument('--runs-dir', default=runs_dir_default())
    parser.add_argument('--scenes-dir', default=scenes_dir_default())
    parser.add_argument('--max-time', type=float, default=120.0,
                        help='time budget in seconds for the RRT search (--plan); the start '
                             'derivation always keeps its own 120 s')
    parser.add_argument('--no-hide-built', action='store_true',
                        help='keep the already-built bars as obstacles')
    parser.add_argument('--export-scene-only', action='store_true',
                        help='bake the problem scene for the dashboard and exit')
    parser.add_argument('--manual-start', default=None, metavar='ANCHOR,SLIDE_M,ROLL_DEG[,PERP1_M,PERP2_M]',
                        help="skip the sweep: IK-check the operator's bar pose, e.g. "
                             "'horizontal,0,0' or 'all,0.1,-30,0,0.05' (what the monitor's "
                             "'M1: Confirm manual start pose' button does)")
    parser.add_argument('--plan', action='store_true',
                        help='run the full transfer plan (derive + RRT), as Plan Movement '
                             'does, instead of the derive stage alone')
    args = parser.parse_args()
    problem_dir = os.path.join(DESIGN_DATA_DIRECTORY, args.problem)

    started = time.perf_counter()
    session = CfabSession(args.problem, connection_type='direct')
    print(f'[setup] cell ready in {time.perf_counter() - started:.1f} s')

    if args.export_scene_only:
        ensure_scene_glb(session.robot_cell, args.problem, scenes_dir=args.scenes_dir)
        return

    if args.bar is None:
        bars = all_bars(problem_dir)[:1]
    elif args.bar == ['all']:
        bars = all_bars(problem_dir)
    else:
        bars = args.bar
    stub = types.SimpleNamespace(cfab=session, active_bar_name=None, get_logger=lambda: _Log())
    rows = []
    batch_started = time.perf_counter()
    for bar in bars:
        try:
            rows.append(plan_bar(session, stub, args, bar, problem_dir))
        except Exception as exc:  # one bar's failure must not end the batch
            traceback.print_exc()
            rows.append({'bar': bar, 'result': f'ERROR {type(exc).__name__}: {exc}'})
    print(f'\n[batch] {len(rows)} bar(s) in {(time.perf_counter() - batch_started) / 60:.1f} min')
    table_path = None
    if len(bars) > 1:
        os.makedirs(args.runs_dir, exist_ok=True)
        table_path = os.path.join(
            args.runs_dir, f'batch_{time.strftime("%Y%m%d-%H%M%S")}_{args.problem}.md')
    print_batch_table(rows, table_path)


if __name__ == '__main__':
    main()
