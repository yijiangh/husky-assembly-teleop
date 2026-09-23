"""Run one M1 start derivation without the robot, and record it for the dashboard.

The headless twin of the monitor's "M1: Derive Start/Goal only (no RRT)" button:
it builds the design problem's cell, loads a bar action, applies the same
mocap-accuracy setup the live monitor does (ground body injected, already-built
bars hidden), runs ONLY the start-derivation stage, and writes a run file the
dashboard picks up.

Use it to reproduce and study a slow derivation at the desk, and to export a
design problem's scene the first time (the export happens automatically).

Usage:
    cd /home/su/ros2_ws
    source venv/bin/activate
    source install/setup.bash
    python src/husky-assembly-teleop/scripts/derive_m1_headless.py --bar B3
    python src/husky-assembly-teleop/scripts/derive_m1_headless.py --bar B3 --anchor back
    python src/husky-assembly-teleop/scripts/derive_m1_headless.py --bar B3 --export-scene-only
"""
import argparse
import os
import time
import types

import pybullet_planning as pp
from husky_assembly_teleop import DESIGN_DATA_DIRECTORY, DESIGN_PROBLEM_NAME
from husky_assembly_teleop.bar_action_io import parse_bar_action
from husky_assembly_teleop.cfab_session import CfabSession
from husky_assembly_teleop.dashboard.run_schema import runs_dir_default, scenes_dir_default
from husky_assembly_teleop.dashboard.run_writer import write_m1_run
from husky_assembly_teleop.dashboard.scene_export import ensure_scene_glb
from husky_assembly_teleop.husky_monitor import (
    CDFM_POSITION_RES, CDFM_ROTATION_RES, HUSKY_DUAL_ARM_HOME_CONF_12,
    HUSKY_DUAL_UR5e_JOINT_NAMES, HuskyMonitor,
)
from husky_assembly_teleop.m1_derive_report import print_m1_derivation_summary
from husky_assembly_tamp.motion_planner.api import (
    TOOL_LINK_LEFT, TOOL_LINK_RIGHT, _bar_body_id, _collect_obstacle_puids,
    _derive_constrained_start_for_plan,
)


class _Log:
    """The few logger calls the borrowed monitor helpers make."""

    def info(self, message):
        print('  [info]', message)

    def warn(self, message):
        print('  [WARN]', message)


def prepare_state(session, action, bar_name, hide_built=True):
    """Bring M1's start state to exactly what the live monitor would plan from.

    Args:
        session (CfabSession): the open cell session.
        action: the parsed BarAssemblyAction.
        bar_name (str): the active bar's rigid-body name.
        hide_built (bool): hide the already-built bars, as the mocap-accuracy
            experiment does (they are reaching locations, not real obstacles).

    Returns:
        tuple: ``(m1_start_state, goal_conf)``.
    """
    names_12 = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]) + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
    stub = types.SimpleNamespace(cfab=session, active_bar_name=bar_name,
                                 get_logger=lambda: _Log())
    m1, m2 = action.movements[1], action.movements[2]
    state = m1.start_state
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
        for name, value in zip(names_12, HUSKY_DUAL_ARM_HOME_CONF_12):
            state.robot_configuration[name] = float(value)
    return state, m2.start_state.robot_configuration


def main():
    """Derive one M1 start and write the run file."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bar', default='B3', help='bar action file stem, e.g. B3')
    parser.add_argument('--anchor', default=None,
                        choices=[None, 'horizontal', 'vertical', 'back'],
                        help='restrict to one carry anchor (default: all three)')
    parser.add_argument('--problem', default=DESIGN_PROBLEM_NAME)
    parser.add_argument('--runs-dir', default=runs_dir_default())
    parser.add_argument('--scenes-dir', default=scenes_dir_default())
    parser.add_argument('--no-hide-built', action='store_true',
                        help='keep the already-built bars as obstacles')
    parser.add_argument('--export-scene-only', action='store_true',
                        help='bake the problem scene for the dashboard and exit')
    args = parser.parse_args()

    started = time.perf_counter()
    session = CfabSession(args.problem, connection_type='direct')
    planner = session.planner
    print(f'[setup] cell ready in {time.perf_counter() - started:.1f} s')

    if args.export_scene_only:
        ensure_scene_glb(session.robot_cell, args.problem, scenes_dir=args.scenes_dir)
        return

    bar_name = f'bar_{args.bar}'
    action = parse_bar_action(os.path.join(
        DESIGN_DATA_DIRECTORY, args.problem, 'BarActions', f'{args.bar}.json'))
    state, goal_conf = prepare_state(session, action, bar_name,
                                     hide_built=not args.no_hide_built)

    names_12 = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]) + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
    robot_puid = planner.client.robot_puid
    arm_joints = pp.joints_from_names(robot_puid, names_12)
    planner.set_robot_cell_state(state)

    print(f'[derive] {args.bar}, anchor {args.anchor or "all"}, up to 120 s ...')
    started = time.perf_counter()
    start_conf, _bar_start, _bar_goal, _goal_arr, _gl, _gr, info = \
        _derive_constrained_start_for_plan(
            planner, state,
            active_bar_id=bar_name,
            bar_body=_bar_body_id(planner, bar_name),
            obstacles=_collect_obstacle_puids(planner, exclude={bar_name}),
            robot_puid=robot_puid, arm_joints=arm_joints,
            tool_link_left=pp.link_from_name(robot_puid, TOOL_LINK_LEFT),
            tool_link_right=pp.link_from_name(robot_puid, TOOL_LINK_RIGHT),
            joint_names_12=names_12,
            goal_conf=goal_conf, goal_ee_frames=None,
            random_seed=None, max_ik_attempts=20, bar_sweep_box=None,
            position_res=CDFM_POSITION_RES, rotation_res=CDFM_ROTATION_RES,
            home_anchor=args.anchor)
    planner.set_robot_cell_state(state)
    print(f'[derive] {time.perf_counter() - started:.1f} s; start found: '
          f'{start_conf is not None}; reason: {info.get("failure_reason")}')

    print_m1_derivation_summary(info)
    write_m1_run(planner, state, info, problem=args.problem, bar_action=args.bar,
                 active_bar=bar_name, movement_id=getattr(action.movements[1], 'movement_id', None),
                 home_anchor=args.anchor, source='headless',
                 runs_dir=args.runs_dir, scenes_dir=args.scenes_dir)


if __name__ == '__main__':
    main()
