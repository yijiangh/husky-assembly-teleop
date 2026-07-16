"""Stand-alone trajectory inspector for a saved BarAction movement.

Loads a BarAction, snaps the selected movement (default M0), reads its saved
``<mv>_trajectory.json`` from disk, opens cfab's PyBullet GUI, and exposes:

  - A slider ``Traj t (0..N-1)`` to scrub the waypoint stream. Each tick
    pushes the waypoint's RobotCellState through ``planner.set_robot_cell_state``
    so the robot, tools and attached bar all move in lockstep — same wiring
    as the live monitor's traj-time slider.
  - A keyboard trigger (press ``c`` in the PyBullet window) to run
    ``monitor.cfab.planner.check_collision(state, {"verbose": True,
    "full_report": True})`` at the currently-selected waypoint. The verbose
    output enumerates every CC.1..CC.5 pair the cfab checker considered and
    why each was SKIPPED or PASSED, so you can see *why* an obvious overlap
    isn't getting flagged.

Usage (ros2_ws venv active + install/setup.bash sourced):

  # Trajectory scrubber (default mode):
  python src/husky-assembly-teleop/scripts/m0_trajectory_inspector.py \\
      --bar-action B6.json --movement M0

  # Target-EE-frames IK check (no trajectory file required):
  python src/husky-assembly-teleop/scripts/m0_trajectory_inspector.py \\
      --bar-action B6.json --movement M2 --check-target-ee

In the cfab window (scrubber mode):
  - drag ``Traj t (0..N-1)`` to scrub the trajectory
  - press ``c`` to run a verbose cfab collision check on the current waypoint
  - Ctrl+C in this terminal to exit
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from types import SimpleNamespace

import numpy as np


DEFAULT_PROBLEM = "2026-05-16_double_kissing_jig_demo"
DEFAULT_BAR_ACTION = "B6.json"
DEFAULT_MOVEMENT = "M0"

# DEFAULT_PROBLEM = "2026-05-19_reoriented2"
# DEFAULT_BAR_ACTION = "B122.json"
# DEFAULT_MOVEMENT = "M1"

class StubLogger:
    def warn(self, msg):  print(f"[WARN] {msg}")
    def info(self, msg):  print(f"[INFO] {msg}")
    def error(self, msg): print(f"[ERROR] {msg}")


def _patch_design_problem(problem: str) -> None:
    from husky_assembly_teleop import husky_monitor as hm
    hm.DESIGN_PROBLEM_NAME = problem


def _bypass_init_monitor():
    """Build a HuskyMonitor without running __init__ (no ROS/mocap/pp)."""
    from husky_assembly_teleop.husky_monitor import HuskyMonitor

    monitor = object.__new__(HuskyMonitor)
    monitor.huskies = []
    monitor.selected_robot_id = 0
    monitor.static_obstacles = {}
    monitor.active_bar_body = None
    monitor.active_bar_aabb_dims = None
    monitor.active_bar_name = None
    monitor.active_extra_bodies = []
    monitor.bar_from_extra = []
    monitor.cfab = None
    monitor.current_action = None
    monitor.current_movement = None
    monitor.current_movement_index = None
    monitor.movement_start_state = None
    monitor.target_ee_frames = None
    monitor.grasp_link_from_bar = None
    monitor.movement_goal_state = None
    monitor.constrained_planner_stage = 3
    monitor.staging_free_trajectory = [None, None]
    monitor.constrained_trajectory = [None, None]
    monitor.constrained_display_mode = 0
    monitor.available_bar_actions = []
    monitor.selected_state_index = 0
    monitor.available_joint_trajectories = []
    monitor.selected_trajectory_index = 0
    monitor.goal_arm_pose = [np.zeros(6), np.zeros(6)]
    monitor.goal_base_pose = (np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]))
    monitor.goal_model = SimpleNamespace(
        set_pose=lambda base_pose, arm_pose: None,
        dual_arm=True,
        set_color=lambda *a, **kw: None,
        get_link_pose_from_name=lambda name: ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    )
    monitor.show_goal_state = False
    monitor.trajectory_time = 20.0
    monitor.selected_arm_index = 0
    monitor._selected_action_file_idx = 0
    monitor._selected_movement_idx = 0
    monitor._loaded_movements = []
    monitor._loaded_action = None
    monitor._current_action_path = None
    monitor._traj_ghost_bodies = []
    monitor._traj_ghost_orig_colors = {}
    monitor._ee_target_pose_uids = []
    monitor.planned_arm_trajectory = None
    monitor.BAR_ACTION_LIVE_REPLAN_EXE = True
    monitor.FAKE_HARDWARE = False
    monitor._is_live_monitor = False
    monitor.goal_base_pose_frozen = False

    def _noop(*a, **kw):
        return None

    monitor.set_arm_trajectory = lambda traj, index=0: None
    monitor.set_to_show_traj_state = _noop
    monitor.set_to_show_goal_state = _noop
    monitor.reset_ui = lambda *a, **kw: None
    monitor._hide_cfab_robot = _noop

    monitor.get_logger = lambda _logger=StubLogger(): _logger
    return monitor


def _attach_stub_husky_interface(monitor, m1_start_state):
    """Set huskies[0].interface so _inject_live_conf_into_state works in headless.
    Same logic as headless_live_monitor_test.py: base from M1, arm at HOME.
    """
    from husky_assembly_teleop.utils import HUSKY_DUAL_UR5e_JOINT_NAMES, pose_from_frame  # noqa: F401
    from husky_assembly_teleop.husky_robot import UR5e_HOME_STATE
    from husky_assembly_teleop.utils import pose_from_frame  # re-import for clarity

    pos, rot = pose_from_frame(m1_start_state.robot_base_frame)
    home = np.asarray(UR5e_HOME_STATE, dtype=float)
    iface = SimpleNamespace(
        position=np.asarray(pos, dtype=float),
        rotation=np.asarray(rot, dtype=float),
        arm_joint_pose=[home.copy(), home.copy()],
    )
    monitor.huskies = [SimpleNamespace(interface=iface, object=None)]
    monitor.selected_robot_id = 0


def _find_movement_by_role(monitor, role: str):
    for i, mv in enumerate(monitor._loaded_movements):
        if monitor._match_movement_role(mv) == role:
            return i, mv
    return None, None


def _state_at_waypoint(template_state, q12, joint_names_12):
    state = template_state.copy()
    for n, v in zip(joint_names_12, q12):
        state.robot_configuration[n] = float(v)
    return state


def _print_conf12(conf) -> None:
    """Pretty-print a 12-DOF dual-arm configuration as two per-arm rows.

    Accepts either a 12-element array-like (np array / list, ordered
    left[0..5] then right[0..5]) OR a name-keyed Configuration / dict
    using HUSKY_DUAL_UR5e_JOINT_NAMES.
    """
    from husky_assembly_teleop.utils import HUSKY_DUAL_UR5e_JOINT_NAMES
    left_names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0])
    right_names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
    try:
        arr = np.asarray(conf, dtype=float).ravel()
    except (TypeError, ValueError):
        arr = None
    if arr is not None and arr.size == 12:
        left = arr[:6].tolist()
        right = arr[6:].tolist()
    else:
        left = [float(conf[n]) for n in left_names]
        right = [float(conf[n]) for n in right_names]
    def _fmt(vals):
        return "[" + ", ".join(f"{v:+.4f}" for v in vals) + "]"
    print(f"  left  : {_fmt(left)}")
    print(f"  right : {_fmt(right)}")


def _check_target_ee_frames(monitor, mv) -> None:
    """IK-solve mv.target_ee_frames in 3 escalating stages.

    Stage 1: full collision check. Stage 2 (only if stage 1 fails): skip
    env collisions (CC.3/4/5). Stage 3 (only after stage 2 succeeds): run
    ``planner.check_collision(verbose=True, full_report=True)`` on the
    skip-env solution and enumerate every offending pair.

    Pushes the IK solution into the cfab GUI via ``set_robot_cell_state``
    so the user can visually confirm.
    """
    if not mv.target_ee_frames \
            or 'left' not in mv.target_ee_frames \
            or 'right' not in mv.target_ee_frames:
        print(f"[ik] {mv.movement_id!r} has no target_ee_frames; nothing to IK.")
        return
    if mv.start_state is None or mv.start_state.robot_configuration is None:
        print(f"[ik] {mv.movement_id!r}.start_state.robot_configuration is None; "
              f"need a seed conf — plan M1 first.")
        return

    from husky_assembly_teleop.husky_world import _solve_bar_action_goal_ik
    from compas_fab.backends import CollisionCheckError

    # _solve_bar_action_goal_ik returns a 12-DOF np.array (left||right) on
    # success and side-effects monitor.movement_goal_state with the full
    # RobotCellState (husky_world.py:1913-1917). We use the array for the
    # printout and the state for cfab GUI viz + the verbose collision check.

    # Stage 1: full collision check.
    print(f"\n=== [ik stage 1] full collision check ===")
    conf12 = _solve_bar_action_goal_ik(
        monitor, mv.start_state, verbose=True, skip_env_collisions=False,
    )
    if conf12 is not None:
        print(f"\n[ik] FOUND (full CC). Goal 12-DOF conf:")
        _print_conf12(conf12)
        gs = monitor.movement_goal_state
        if gs is not None:
            monitor.cfab.planner.set_robot_cell_state(gs)
            print(f"[ik] pushed goal state to cfab GUI for visual confirmation.")
        return

    # Stage 2: skip env collisions.
    print(f"\n=== [ik stage 2] skip env collisions (CC.3 link↔rb, "
          f"CC.4 attached-rb↔rb, CC.5 tool↔rb) ===")
    conf12_no_env = _solve_bar_action_goal_ik(
        monitor, mv.start_state, verbose=True, skip_env_collisions=True,
    )
    if conf12_no_env is None:
        print(f"\n[ik] FAIL: also unreachable when env collisions skipped — "
              f"target_ee_frames not kinematically reachable from current "
              f"start_state.robot_base_frame.")
        return

    print(f"\n[ik] FOUND (skip-env). Goal 12-DOF conf:")
    _print_conf12(conf12_no_env)
    gs_no_env = monitor.movement_goal_state
    if gs_no_env is None:
        print(f"[ik] WARN: monitor.movement_goal_state was not set; "
              f"cannot run verbose collision check.")
        return
    monitor.cfab.planner.set_robot_cell_state(gs_no_env)
    print(f"[ik] pushed goal state to cfab GUI; env is colliding — "
          f"dumping pairs:")

    # Stage 3: enumerate env collisions on the skip-env solution.
    try:
        monitor.cfab.planner.check_collision(
            gs_no_env, {"verbose": True, "full_report": True},
        )
        print(f"[ik] (collision check passed under full CC — unexpected "
              f"after stage-1 failure; likely a non-determinism / ACM "
              f"transient.)")
    except CollisionCheckError as e:
        msg = getattr(e, 'message', None) or str(e)
        print(f"---- collision messages ----\n{msg}")
        pairs = getattr(e, 'collision_pairs', None) or []
        if pairs:
            print(f"---- collision_pairs ({len(pairs)}) ----")
            for a, b in pairs:
                an = getattr(a, 'name', repr(a))
                bn = getattr(b, 'name', repr(b))
                print(f"  {an}  <->  {bn}")


def main(bar_action: str, problem: str, movement_role: str,
         trajectory_path: str | None, check_target_ee: bool = False) -> int:
    print(f"=== m0_trajectory_inspector: problem={problem!r} "
          f"bar_action={bar_action!r} movement={movement_role!r} ===")

    _patch_design_problem(problem)

    monitor = _bypass_init_monitor()
    try:
        # cfab GUI session.
        from husky_assembly_teleop.cfab_session import CfabSession
        print("[cfab] opening cfab PyBullet (gui) session...")
        monitor.cfab = CfabSession(
            problem, connection_type="gui", enable_debug_gui=True,
        )
        import pybullet_planning as pp
        pp.CLIENT = monitor.cfab.client.client_id
        pp.CLIENTS[monitor.cfab.client.client_id] = True

        # Populate BarAction file slider; pick the requested file.
        monitor.available_bar_actions = monitor._load_available_bar_actions()
        if not monitor.available_bar_actions:
            print("FAIL: no BarAction files available")
            return 1
        if bar_action not in monitor.available_bar_actions:
            print(f"FAIL: {bar_action!r} not in available BarActions; have "
                  f"{monitor.available_bar_actions}")
            return 1
        monitor._selected_action_file_idx = monitor.available_bar_actions.index(bar_action)

        # Probe-parse to install the stub interface BEFORE the native M0's
        # live-conf injection runs.
        from husky_assembly_teleop.bar_action_io import parse_bar_action
        from husky_assembly_teleop import DESIGN_DATA_DIRECTORY
        action_path = os.path.join(
            DESIGN_DATA_DIRECTORY, problem, 'BarActions', bar_action,
        )
        probe = parse_bar_action(action_path)
        if not probe.movements:
            print(f"FAIL: BarAction {bar_action!r} has no movements")
            return 1
        _attach_stub_husky_interface(monitor, probe.movements[0].start_state)

        print(f"\n--- 'Load BarAction' ({bar_action}) ---")
        monitor.load_bar_action_file()

        # Pick the target movement and push it into cfab.
        idx, mv = _find_movement_by_role(monitor, movement_role)
        if mv is None:
            print(f"FAIL: no movement with role {movement_role!r} in loaded movements.")
            return 1
        print(f"\n--- 'Load Movement' (idx={idx}, role={movement_role}, "
              f"id={mv.movement_id!r}) ---")
        monitor._selected_movement_idx = idx
        monitor.load_selected_movement()

        # IK-check mode: solve mv.target_ee_frames + report, skip the
        # trajectory scrubber entirely (no trajectory file required).
        if check_target_ee:
            _check_target_ee_frames(monitor, mv)
            try:
                input("\n[inspector] press Enter to close the cfab GUI…")
            except (EOFError, KeyboardInterrupt):
                pass
            return 0

        # Resolve the trajectory file. If the user gave an explicit path, honor
        # it. Otherwise rely on monitor._trajectory_file_for + auto-load.
        if trajectory_path is not None:
            traj_path = trajectory_path
        else:
            traj_path = monitor._trajectory_file_for(mv)

        if mv.trajectory is None:
            # Auto-load may have rejected it on consistency-check; re-load
            # directly so the scrubber has something to show.
            from compas.data import json_load
            if not os.path.exists(traj_path):
                print(f"FAIL: trajectory file not found at {traj_path}")
                return 1
            print(f"[trajectory] auto-load did not keep the file; "
                  f"loading directly from {traj_path}")
            mv.trajectory = json_load(traj_path)
        else:
            print(f"[trajectory] mv.trajectory was loaded via auto-load.")
        print(f"[trajectory] resolved path: {traj_path}")

        # Build the per-waypoint RobotCellState list.
        from husky_assembly_teleop.utils import (
            HUSKY_DUAL_UR5e_JOINT_NAMES, path_12_from_joint_trajectory,
        )
        left_names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0])
        right_names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
        joint_names_12 = left_names + right_names

        path12 = path_12_from_joint_trajectory(mv.trajectory)
        N = len(path12)
        if N == 0:
            print("FAIL: trajectory has no waypoints.")
            return 1
        states = [_state_at_waypoint(mv.start_state, q, joint_names_12)
                  for q in path12]
        print(f"[trajectory] {N} waypoints; left_names={left_names}; "
              f"right_names={right_names}")

        # Snap to the first waypoint so the GUI shows a sensible pose
        # immediately.
        monitor.cfab.planner.set_robot_cell_state(states[0])

        import pybullet
        cid = monitor.cfab.client.client_id
        traj_slider = pybullet.addUserDebugParameter(
            f"Traj t (0..{N - 1})", 0.0, float(N - 1), 0.0,
            physicsClientId=cid,
        )
        # PyBullet supports a button via addUserDebugParameter with
        # rangeMin > rangeMax — each click increments the returned value
        # by 1. We track the previous count and fire the check on each
        # increment.
        check_button = pybullet.addUserDebugParameter(
            "Check collision", 1, 0, 0,
            physicsClientId=cid,
        )

        print(f"\n=== INSPECTOR READY ===")
        print(f"  - drag 'Traj t (0..{N - 1})' to scrub waypoints")
        print(f"  - click 'Check collision' button to run verbose cfab "
              f"check_collision on the current waypoint")
        print(f"  - press the 'c' key in the cfab window to do the same")
        print(f"  - Ctrl+C in this terminal to exit\n")

        from compas_fab.backends import CollisionCheckError

        def _run_check(idx_: int):
            state = states[idx_]
            print(f"\n========================================")
            print(f"[check] waypoint idx={idx_}/{N - 1}: running "
                  f"cfab check_collision(verbose=True, full_report=True)")
            print(f"========================================")
            try:
                monitor.cfab.planner.check_collision(
                    state, {"verbose": True, "full_report": True},
                )
                print(f"\n[check] result @ idx={idx_}: NO COLLISION reported.")
            except CollisionCheckError as e:
                msg = getattr(e, 'message', None) or str(e)
                print(f"\n[check] result @ idx={idx_}: COLLISION")
                print(f"---- collision messages ----")
                print(msg)
                pairs = getattr(e, 'collision_pairs', None) or []
                if pairs:
                    print(f"---- collision_pairs ({len(pairs)}) ----")
                    for a, b in pairs:
                        an = getattr(a, 'name', repr(a))
                        bn = getattr(b, 'name', repr(b))
                        print(f"  {an}  <->  {bn}")
            print(f"========================================\n")

        last_idx = -1
        last_click_count = pybullet.readUserDebugParameter(
            check_button, physicsClientId=cid,
        )
        c_key = ord('c')
        try:
            while True:
                t = pybullet.readUserDebugParameter(traj_slider, physicsClientId=cid)
                idx_ = max(0, min(N - 1, int(round(t))))
                if idx_ != last_idx:
                    monitor.cfab.planner.set_robot_cell_state(states[idx_])
                    print(f"\r[scrub] waypoint idx={idx_:4d}/{N - 1}  ",
                          end="", flush=True)
                    last_idx = idx_

                # Button click is detected as an integer increment of the
                # parameter's returned value.
                click_count = pybullet.readUserDebugParameter(
                    check_button, physicsClientId=cid,
                )
                if click_count > last_click_count:
                    _run_check(idx_)
                last_click_count = click_count

                # Keyboard 'c' trigger — fires on key-down edge only.
                keys = pybullet.getKeyboardEvents(physicsClientId=cid)
                if c_key in keys and (keys[c_key] & pybullet.KEY_WAS_TRIGGERED):
                    _run_check(idx_)

                time.sleep(0.03)
        except KeyboardInterrupt:
            print("\n[inspector] exiting.")

        return 0
    finally:
        try:
            if getattr(monitor, 'cfab', None) is not None:
                monitor.cfab.close()
        except Exception:
            pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--bar-action", type=str, default=DEFAULT_BAR_ACTION,
                        help=f"BarAction *.json filename under "
                             f"<DESIGN_DATA_DIRECTORY>/<problem>/BarActions/. "
                             f"Default: {DEFAULT_BAR_ACTION!r}.")
    parser.add_argument("--problem", type=str, default=DEFAULT_PROBLEM,
                        help=f"DESIGN_PROBLEM_NAME directory. "
                             f"Default: {DEFAULT_PROBLEM!r}.")
    parser.add_argument("--movement", type=str, default=DEFAULT_MOVEMENT,
                        choices=('M0', 'M1', 'M2', 'M3', 'M4'),
                        help=f"Which movement's trajectory to inspect. "
                             f"Default: {DEFAULT_MOVEMENT!r}.")
    parser.add_argument("--trajectory", type=str, default=None,
                        help="Override the trajectory JSON path. Default: "
                             "resolved via monitor._trajectory_file_for(mv).")
    parser.add_argument("--check-target-ee", action="store_true",
                        help="Skip trajectory inspection; instead IK-solve "
                             "the movement's target_ee_frames and report "
                             "(full CC → skip-env CC fallback → verbose "
                             "collision dump on the skip-env solution).")
    args = parser.parse_args()
    sys.exit(main(
        bar_action=args.bar_action,
        problem=args.problem,
        movement_role=args.movement,
        trajectory_path=args.trajectory,
        check_target_ee=args.check_target_ee,
    ))
