"""Smoke test: a support robot's single-arm planning + other robots posed from beliefs.

Planning only -- no ROS, no monitor, nothing written to disk. It opens the
robot cells in a hidden (DIRECT) PyBullet world and checks, on the fixture's
first hold (Alice holds bar B3, schedule entry B3_H):

  1. FK workaround. compas_fab's forward kinematics applies the robot base
     twice (see ``cfab_session._world_frame_forward_kinematics``). Prints the
     stock and the corrected flange point next to PyBullet's own link pose.
  2. H_M2, the linear approach to the bar (``plan_linear_motion``): number of
     points and how close the last point lands to the exported target conf.
  3. H_M0, the free move from UR5e home (``plan_free_motion``).
  4. Obstacle beliefs in Cindy's cell. B3__R's movement 0 exports Alice parked
     at (50, 50, 0) (exporter defect D1). ``apply_obstacle_robot_beliefs`` with
     ``belief_after(B3__H)`` must move her to where she holds B3 -- the same
     pose B4__J exports for her.

! About ``--collisions`` (what H_M0 / H_M2 are checked against):
!   exported   (default) the exported hold scene as-is. It is built with the
!              RELEASE-time geometry (future bars B4..B9 present) and Cindy frozen
!              at her assembled pose, so the H_M0 / H_M2 start states already
!              collide (ObstacleRobotCindy <-> env_bar_B5). Both plans are
!              EXPECTED to fail here; the user chose to keep the exported scene.
!   whitelist  the diagnosis-only allowances: every contact between a frozen
!              obstacle robot and a rigid body at the start state is allowed
!              (in memory), then the plans run with full collision checks.
!   off        no scene collisions: the linear plan runs with
!              check_collision=False; the free planner always checks, so it runs
!              with every scene body hidden (in memory) -- only the robot's
!              self-collision is left.

PASS / FAIL: the FK correction and the belief re-posing must always pass. The
two plans must succeed with ``--collisions off`` or ``whitelist``; with
``exported`` they are only reported. Exit code 1 when a required check fails.

Usage (loads two ~340 MB RobotCell files, one at a time, ~1 GB RAM each;
about 10 s):

    cd /home/yijiangh/Code/ros2_ws && source venv/bin/activate && source install/setup.bash
    export DESIGN_DATA_DIRECTORY="/home/yijiangh/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/2025-03 Husky Assembly/data_design_study"
    export DESIGN_PROBLEM_NAME=260920_RobArch_demo_revamp_backup
    python src/husky-assembly-teleop/scripts/smoke_single_arm_plan.py
    python src/husky-assembly-teleop/scripts/smoke_single_arm_plan.py --collisions whitelist
    python src/husky-assembly-teleop/scripts/smoke_single_arm_plan.py --collisions off

    (If an IK call complains that ``ssik`` is not installed, also
    ``export HUSKY_IK_BACKEND=gradient``.)
"""

import argparse
import sys
import time
from typing import Optional

import numpy as np
import pybullet_planning as pp
from compas.geometry import Frame
from compas_fab.backends import CollisionCheckError
from compas_fab.robots import TargetMode

from husky_assembly_teleop import DESIGN_PROBLEM_NAME
from husky_assembly_teleop.bar_action_io import find_bar_body
from husky_assembly_teleop.cfab_session import (
    CfabSession, _world_frame_forward_kinematics, apply_obstacle_robot_beliefs,
    arm_joint_names_for_group, inject_ground_rigid_body_state, plan_free_motion,
    plan_linear_motion,
)
from husky_assembly_teleop.husky_robot import UR5e_HOME_STATE
from husky_assembly_teleop.progress_io import (
    PARKED_BASE_FRAME, belief_after, new_progress, obstacle_sources, obstacle_tool_states,
)
from husky_assembly_teleop.robot_registry import ROBOTS, robot_by_name
from husky_assembly_teleop.schedule_io import load_schedule, problem_root
from husky_assembly_teleop.utils import path_from_joint_trajectory

# * The fixture's first hold: Alice holds B3 (entry B3_H) until Cindy has built
# * B4..B9; B3_R is Cindy's release of B3 and B4_J the next jointing after it.
SUPPORT_ROBOT = 'Alice'
ASSEMBLY_ROBOT = 'Cindy'
HELD_BAR = 'B3'
NEXT_BAR = 'B4'

# Tolerances for "the same pose / configuration".
SAME_TOL = 1e-6
FK_TOL = 1e-6
# The linear plan ends within ~1e-3 rad of the exported target configuration
# (1 mm / 1 mrad IK tolerance per point).
CONF_TOL = 2e-3

OBSTACLE_TOOL_NAMES = {spec.obstacle_tool_name for spec in ROBOTS.values()}


# * ---------------------------------------------------------------------------
# * Small helpers
# * ---------------------------------------------------------------------------

class Results:
    """Collects PASS / FAIL / INFO lines and prints the summary at the end."""

    def __init__(self):
        self.lines = []  # (tag, name, detail)

    def check(self, name: str, ok: bool, detail: str = '', *, required: bool = True) -> bool:
        """Record one check.

        Args:
            name (str): What was checked.
            ok (bool): Whether it held.
            detail (str): Numbers to print next to it.
            required (bool): False = only reported (INFO / EXPECTED FAIL), never fails the run.

        Returns:
            bool: ``ok``.
        """
        if required:
            tag = 'PASS' if ok else 'FAIL'
        else:
            tag = 'INFO' if ok else 'INFO (failed, not required)'
        self.lines.append((tag, name, detail))
        print(f"[{tag}] {name}" + (f" -- {detail}" if detail else ''))
        return ok

    def n_failed(self) -> int:
        """Number of required checks that failed."""
        return sum(tag == 'FAIL' for tag, _, _ in self.lines)

    def print_summary(self) -> None:
        """Print every check again, grouped at the end of the log."""
        print("\n" + "=" * 78 + "\nSUMMARY\n" + "=" * 78)
        for tag, name, detail in self.lines:
            print(f"  [{tag}] {name}" + (f" -- {detail}" if detail else ''))
        n_req = sum(tag in ('PASS', 'FAIL') for tag, _, _ in self.lines)
        print(f"\n  {n_req - self.n_failed()}/{n_req} required checks passed"
              + (" -> PASS" if self.n_failed() == 0 else " -> FAIL"))


def open_session(problem: str, cell_filename: str) -> CfabSession:
    """Open a hidden cfab planner session on one robot's cell.

    Args:
        problem (str): Design problem folder name.
        cell_filename (str): ``RobotCell.json`` (Cindy) or ``RobotCell_<Name>.json``.

    Returns:
        CfabSession: The open session (close it when done: each holds ~1 GB).
    """
    t0 = time.time()
    session = CfabSession(problem, connection_type='direct', cell_filename=cell_filename)
    # pybullet_planning keeps its own "current client"; point it at this world
    # so plan_free_motion's sample / extend / FK calls read the right bodies.
    pp.CLIENT = session.client.client_id
    pp.CLIENTS[session.client.client_id] = None
    print(f"[cell] {cell_filename} loaded in {time.time() - t0:.1f} s")
    return session


def frame_dist(a: Frame, b: Frame) -> float:
    """Largest difference between two frames' point and axes (m / unitless).

    Args:
        a (Frame): First frame.
        b (Frame): Second frame.

    Returns:
        float: Max absolute component difference over point, xaxis and yaxis.
    """
    return max(float(np.abs(np.subtract(list(getattr(a, k)), list(getattr(b, k)))).max())
               for k in ('point', 'xaxis', 'yaxis'))


def pt(frame: Frame) -> list:
    """A frame's point rounded to 4 decimals, for printing."""
    return np.round(list(frame.point), 4).tolist()


def collision_lines(planner, state) -> list:
    """Every colliding pair in a state (empty list = collision free).

    Args:
        planner: The session's PyBulletPlanner.
        state (RobotCellState): The state to check.

    Returns:
        list[str]: compas_fab's one line per colliding pair.
    """
    try:
        planner.check_collision(state, {'full_report': True})
    except CollisionCheckError as e:
        return (e.message or '').splitlines()
    return []


def whitelist_frozen_contacts(planner, robot_cell, state) -> list:
    """Allow every contact between a frozen obstacle robot and a rigid body (in memory).

    ! Diagnosis only -- the monitor does NOT do this. It mirrors what the
    ! Rhino side's ``whitelist_frozen_contact`` would have to do for the
    ! exported hold scene (e.g. frozen Cindy touching future bar B5).

    Args:
        planner: The session's PyBulletPlanner.
        robot_cell (RobotCell): The session's cell (to name the colliding models).
        state (RobotCellState): Edited in place: the bars get ``touch_bodies`` entries.

    Returns:
        list[str]: ``'<tool> <-> <body>'`` for every allowance added.
    """
    try:
        planner.check_collision(state, {'full_report': True})
        return []
    except CollisionCheckError as e:
        pairs = e.collision_pairs
    # compas_fab reports the colliding MODELS; map them back to their names.
    name_of = {id(m): n for n, m in robot_cell.tool_models.items()}
    name_of.update({id(m): n for n, m in robot_cell.rigid_body_models.items()})
    added = []
    for a, b in pairs:
        tool, body = name_of.get(id(a)), name_of.get(id(b))
        if tool in OBSTACLE_TOOL_NAMES and body in state.rigid_body_states:
            rb = state.rigid_body_states[body]
            rb.touch_bodies = sorted(set(rb.touch_bodies or []) | {tool})
            added.append(f"{tool} <-> {body}")
    return added


def hide_scene(state) -> None:
    """Hide every rigid body and tool that is not carried by the robot (in memory).

    Args:
        state (RobotCellState): Edited in place.
    """
    for rb in state.rigid_body_states.values():
        if not (rb.attached_to_tool or rb.attached_to_link):
            rb.is_hidden = True
    for ts in state.tool_states.values():
        if ts.attached_to_group is None:
            ts.is_hidden = True


def prepare_scene(session: CfabSession, state, mode: str) -> None:
    """Add the ground and apply the ``--collisions`` mode to one start state.

    Args:
        session (CfabSession): The support robot's session.
        state (RobotCellState): The movement's start state (edited in place).
        mode (str): ``'exported'``, ``'whitelist'`` or ``'off'``.
    """
    inject_ground_rigid_body_state(session.robot_cell, state)
    lines = collision_lines(session.planner, state)
    print("  start state as exported: "
          + ("collision free" if not lines else f"{len(lines)} colliding pair(s):"))
    for line in lines[:8]:
        print(f"    {line}")
    if mode == 'whitelist':
        added = whitelist_frozen_contacts(session.planner, session.robot_cell, state)
        print(f"  whitelisted (diagnosis only): {added or 'nothing'}")
    elif mode == 'off':
        print("  scene collisions OFF for this plan")


# * ---------------------------------------------------------------------------
# * The four checks
# * ---------------------------------------------------------------------------

def check_fk_workaround(results: Results, session: CfabSession, spec, state) -> None:
    """Stock vs corrected forward kinematics vs PyBullet's own flange pose.

    Args:
        results (Results): Where the checks go.
        session (CfabSession): The support robot's session.
        spec (RobotSpec): The support robot.
        state (RobotCellState): A state with a full robot configuration (H_M2's start).
    """
    print(f"\n--- 1. forward kinematics (flange = {spec.flange_links[0]}) ---")
    planner, client = session.planner, session.client
    group = spec.planning_groups[0]
    stock = planner.forward_kinematics(state, TargetMode.ROBOT, group=group)
    with _world_frame_forward_kinematics(planner):
        fixed = planner.forward_kinematics(state, TargetMode.ROBOT, group=group)
    leaked = 'forward_kinematics' in vars(planner)
    # The truth: PyBullet's link pose with the state applied (already in world coordinates).
    planner.set_robot_cell_state(state)
    pos, quat = pp.get_link_pose(client.robot_puid, client.robot_link_puids[spec.flange_links[0]])
    truth = Frame.from_quaternion([quat[3], quat[0], quat[1], quat[2]], point=list(pos))
    print(f"  robot base          {pt(state.robot_base_frame)}")
    print(f"  stock compas_fab FK {pt(stock)}   (off by {frame_dist(stock, truth):.3f})")
    print(f"  corrected FK        {pt(fixed)}")
    print(f"  PyBullet link pose  {pt(truth)}")
    results.check('FK: corrected flange == PyBullet link pose', frame_dist(fixed, truth) < FK_TOL,
                  f"max diff {frame_dist(fixed, truth):.1e}")
    results.check('FK: the correction does not stay on the planner', not leaked)
    results.check('FK: stock compas_fab still double-applies the base (bug present)',
                  frame_dist(stock, truth) > 1e-3, f"off by {frame_dist(stock, truth):.3f}",
                  required=False)


def plan_h_m2(results: Results, session: CfabSession, spec, mv, mode: str, required: bool) -> None:
    """Plan the linear approach H_M2 and compare its end with the exported target conf.

    Args:
        results (Results): Where the checks go.
        session (CfabSession): The support robot's session.
        spec (RobotSpec): The support robot.
        mv: The SingleArmLinearMovement (its start state is edited in memory).
        mode (str): ``--collisions`` mode.
        required (bool): Whether a failed plan fails the run.
    """
    print(f"\n--- 2. {mv.movement_id}: linear approach, --collisions {mode} ---")
    names = arm_joint_names_for_group(session.robot_cell, spec.planning_groups[0])
    prepare_scene(session, mv.start_state, mode)
    t0 = time.time()
    jt = plan_linear_motion(session.planner, mv.start_state, mv.target_ee_frames['arm'],
                            group=spec.planning_groups[0], check_collision=(mode != 'off'))
    dt = time.time() - t0
    if jt is None:
        results.check(f'{mv.movement_id} linear plan', False,
                      f"no plan in {dt:.1f} s (reason: the [linear plan] line above)", required=required)
        return
    path = path_from_joint_trajectory(jt, names)
    target = np.array([mv.target_configuration[n] for n in names])
    err = float(np.abs(path[-1] - target).max())
    results.check(f'{mv.movement_id} linear plan', err < CONF_TOL,
                  f"{len(jt.points)} points in {dt:.1f} s, max |end - target conf| = {err:.1e} rad",
                  required=required)


def plan_h_m0(results: Results, session: CfabSession, spec, mv, mode: str, required: bool) -> None:
    """Plan the free move H_M0 from UR5e home to the exported target conf.

    Args:
        results (Results): Where the checks go.
        session (CfabSession): The support robot's session.
        spec (RobotSpec): The support robot.
        mv: The SingleArmFreeMovement (its start state is edited in memory).
        mode (str): ``--collisions`` mode.
        required (bool): Whether a failed plan fails the run.
    """
    print(f"\n--- 3. {mv.movement_id}: free move from UR5e home, --collisions {mode} ---")
    names = arm_joint_names_for_group(session.robot_cell, spec.planning_groups[0])
    # The export leaves H_M0's start conf empty (the monitor fills in the live
    # joints); UR5e home stands in for them here.
    state = mv.start_state
    state.robot_configuration = session.robot_cell.zero_full_configuration()
    for name, value in zip(names, UR5e_HOME_STATE):
        state.robot_configuration[name] = float(value)
    prepare_scene(session, state, mode)
    if mode == 'off':
        hide_scene(state)
    t0 = time.time()
    path, info = plan_free_motion(session.planner, state, mv.target_configuration,
                                  group=spec.planning_groups[0])
    dt = time.time() - t0
    if path is None:
        results.check(f'{mv.movement_id} free plan', False,
                      f"{info.get('failure_reason')} ({dt:.1f} s)", required=required)
        return
    target = np.array([mv.target_configuration[n] for n in names])
    err = float(np.abs(path[-1] - target).max())
    results.check(f'{mv.movement_id} free plan', err < 1e-6,
                  f"{len(path)} waypoints in {dt:.1f} s, max |end - target conf| = {err:.1e} rad",
                  required=required)


def check_obstacle_beliefs(results: Results, problem: str, schedule) -> None:
    """Re-pose the support robot in the assembly robot's cell from its belief.

    Marks the hold entry done in an IN-MEMORY progress (nothing is saved), then
    applies the resulting beliefs to the release's first state and compares the
    support robot's obstacle tool with what the next jointing exports.

    Args:
        results (Results): Where the checks go.
        problem (str): Design problem folder name.
        schedule (ActionSchedule): The problem's schedule.
    """
    support, assembly = robot_by_name(SUPPORT_ROBOT), robot_by_name(ASSEMBLY_ROBOT)
    tool = support.obstacle_tool_name
    hold = schedule.find_entry(HELD_BAR, 'H')
    release = schedule.find_entry(HELD_BAR, 'R')
    oracle_entry = schedule.find_entry(NEXT_BAR, 'J')
    print(f"\n--- 4. {tool} in {assembly.name}'s cell: entry {release.index} {release.action_id}, "
          f"movement 0 ---")

    progress = new_progress(schedule)  # in memory only
    held_action = schedule.load_action(hold, prefer_sidecar=False)
    progress.mark_done(hold, support.name, 'smoke', belief_after(held_action, support, hold.index))
    release_action = schedule.load_action(release, prefer_sidecar=False)
    oracle = schedule.load_action(oracle_entry, prefer_sidecar=False).movements[0].start_state.tool_states[tool]

    session = open_session(problem, assembly.cell_file)
    try:
        state = release_action.movements[0].start_state
        inject_ground_rigid_body_state(session.robot_cell, state)
        before = state.tool_states[tool].frame.copy()
        lines_before = collision_lines(session.planner, state)
        results.check(f'{release.action_id} exports {tool} parked at (50, 50, 0) (exporter defect D1)',
                      frame_dist(before, PARKED_BASE_FRAME) < SAME_TOL, f"exported {pt(before)}",
                      required=False)

        beliefs = obstacle_tool_states(progress, assembly.name, exported_action=release_action)
        print(f"  belief sources: {obstacle_sources(progress, assembly.name, exported_action=release_action)}")
        print(f"  holding bars: {progress.holding_bars()}")
        apply_obstacle_robot_beliefs(state, session.robot_cell, assembly.name, beliefs,
                                     holding_bars=progress.holding_bars())
        after = state.tool_states[tool]
        conf_err = max(abs(after.configuration[n] - oracle.configuration[n])
                       for n in oracle.configuration.joint_names)
        print(f"  {tool} before {pt(before)} -> after {pt(after.frame)}; "
              f"{oracle_entry.action_id} exports {pt(oracle.frame)}")
        results.check(f'{tool} re-posed from belief_after({hold.action_id}) (not parked)',
                      frame_dist(after.frame, PARKED_BASE_FRAME) > 1.0, f"at {pt(after.frame)}")
        results.check(f"{tool} frame == {oracle_entry.action_id}'s export",
                      frame_dist(after.frame, oracle.frame) < SAME_TOL,
                      f"max diff {frame_dist(after.frame, oracle.frame):.1e}")
        results.check(f"{tool} arm joints == {oracle_entry.action_id}'s export (by joint name)",
                      conf_err < SAME_TOL, f"max diff {conf_err:.1e} rad")
        bar_name = find_bar_body(state.rigid_body_states, HELD_BAR)
        bar_rb = state.rigid_body_states.get(bar_name) if bar_name else None
        results.check(f"{bar_name} may touch {tool} (it is clamped on it)",
                      bar_rb is not None and tool in (bar_rb.touch_bodies or []))
        try:
            session.planner.set_robot_cell_state(state)
            pushed = True
        except Exception as e:  # report any compas_fab refusal as a failed check
            print(f"  set_robot_cell_state raised {type(e).__name__}: {e}")
            pushed = False
        results.check('re-posed state still loads into the cell', pushed)
        lines_after = collision_lines(session.planner, state)
        results.check(f'collision check of {release.action_id} movement 0 before / after re-posing',
                      True, f"{len(lines_before)} / {len(lines_after)} colliding pair(s)", required=False)
        for line in lines_after[:8]:
            print(f"    {line}")
    finally:
        session.close()


# * ---------------------------------------------------------------------------
# * Main
# * ---------------------------------------------------------------------------

def main(argv: Optional[list] = None) -> int:
    """Run the four checks and print the summary.

    Args:
        argv (list | None): Command line (None = ``sys.argv[1:]``).

    Returns:
        int: 0 when every required check passed, else 1.
    """
    parser = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="See the module docstring for what each check means.")
    parser.add_argument('--problem', default=DESIGN_PROBLEM_NAME,
                        help="design problem folder (default: env DESIGN_PROBLEM_NAME = %(default)s)")
    parser.add_argument(
        '--collisions', choices=('exported', 'off', 'whitelist'), default='exported',
        help="what H_M0 / H_M2 are collision-checked against. 'exported' (default): the "
             "exported hold scene as-is -- it uses RELEASE-time geometry, so the start states "
             "collide with future bar B5 (frozen Cindy <-> env_bar_B5) and both plans are "
             "expected to fail; the user chose to keep this scene. 'whitelist': the "
             "diagnosis-only allowances (frozen obstacle robot <-> bar contacts allowed in "
             "memory). 'off': no scene collision checks.")
    args = parser.parse_args(argv)

    results = Results()
    schedule = load_schedule(problem_root(args.problem))
    if schedule is None:
        print(f"{args.problem} has no ActionSchedule.json; this smoke needs the support-robot fixture.")
        return 1
    support = robot_by_name(SUPPORT_ROBOT)
    hold = schedule.find_entry(HELD_BAR, 'H')
    held_action = schedule.load_action(hold, prefer_sidecar=False)
    mv0, mv2 = held_action.movements[0], held_action.movements[2]
    plans_required = args.collisions != 'exported'
    print(f"problem {args.problem} | {support.name} on {support.cell_file} | "
          f"entry {hold.index} {hold.action_id} | --collisions {args.collisions}")

    session = open_session(args.problem, support.cell_file)
    try:
        inject_ground_rigid_body_state(session.robot_cell, mv2.start_state)
        check_fk_workaround(results, session, support, mv2.start_state)
        plan_h_m2(results, session, support, mv2, args.collisions, plans_required)
        plan_h_m0(results, session, support, mv0, args.collisions, plans_required)
    finally:
        session.close()

    check_obstacle_beliefs(results, args.problem, schedule)

    results.print_summary()
    return 1 if results.n_failed() else 0


if __name__ == '__main__':
    sys.exit(main())
