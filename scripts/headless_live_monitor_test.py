"""Headless full-sequence test for the BarAction (cfab) planning path.

Mirrors the BAR_HOLDING_ACCURACY_TEST UI button sequence in HuskyMonitor:

  1. ``load_bar_action_file()``                <- 'Load BarAction'
  2. for idx in [1, 2, 3, 4, 0]:               <- one click per movement
        ``load_selected_movement()``           <- 'Load Movement'
        ``plan_selected_movement()``           <- 'Plan Movement'

Each ``plan_selected_movement`` dispatches to ``_plan_M{0,1,2,3,4}_dispatch``
and routes through ``_accept_trajectory``, which (a) writes the trajectory to
``<DESIGN_DATA_DIRECTORY>/<problem>/Trajectories/<movement_id>_trajectory.json``
and (b) propagates the end configuration into the next movement's
``start_state.robot_configuration``. Hence the planning order M1 -> M2 -> M3
-> M4 -> M0: each Mk's plan seeds M(k+1)'s start.

Env-collision behavior matches the live monitor: each dispatcher now plans
WITH environment obstacles enabled (see husky_monitor.py _plan_M{1,2,3}_dispatch
and _build_pp_scene_for_free). The headless script does NOT inject any global
'ignore env obstacles' escape hatch.

Usage (ros2_ws venv active + install/setup.bash sourced):
  python src/husky-assembly-teleop/scripts/headless_live_monitor_test.py \\
      --bar-action B6.json [--gui] [--only-movement M2] [--no-save]
"""

from __future__ import annotations

import argparse
import os
import sys
from types import SimpleNamespace

import numpy as np


DEFAULT_PROBLEM = "2026-05-16_double_kissing_jig_demo"
DEFAULT_BAR_ACTION = "B6.json"


class StubLogger:
    def warn(self, msg):  print(f"[WARN] {msg}")
    def info(self, msg):  print(f"[INFO] {msg}")
    def error(self, msg): print(f"[ERROR] {msg}")


def _patch_design_problem(problem: str) -> None:
    from husky_assembly_teleop import husky_monitor as hm
    hm.DESIGN_PROBLEM_NAME = problem


def _bypass_init_monitor():
    """Construct a HuskyMonitor without running __init__ (no ROS / mocap / pp).

    We only fill in the attributes that load_bar_action_file ->
    load_selected_movement -> plan_selected_movement -> _accept_trajectory
    actually read. UI side effects (reset_ui / show goal state) are stubbed.
    """
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
    monitor.movement_type = None
    monitor.movement_start_state = None
    monitor.target_ee_frames = None
    monitor.grasp_link_from_bar = None
    monitor.movement_goal_state = None

    monitor.constrained_planner_stage = 3
    monitor.staging_free_trajectory = [None, None]
    monitor.constrained_trajectory = [None, None]
    monitor.constrained_display_mode = 0

    monitor.available_robot_cell_states = []
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

    monitor.BAR_HOLDING_ACCURACY_TEST = True
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

    _logger = StubLogger()
    monitor.get_logger = lambda: _logger
    return monitor


def _attach_stub_husky_interface(monitor, m1_start_state):
    """Provide huskies[0].interface for _make_synthetic_m0.

    _make_synthetic_m0 (husky_monitor.py:1334-1356) reads .position,
    .rotation and .arm_joint_pose off huskies[0].interface to snapshot
    "live" robot state. Headless has no ROS / mocap, so we synthesize an
    interface that exactly matches M1.start_state -- making the synthetic
    M0 a no-op pair-up to M1.start. If you want non-trivial M0 planning
    in headless, mutate the returned interface before load_bar_action_file.
    """
    from husky_assembly_teleop.utils import HUSKY_DUAL_UR5e_JOINT_NAMES, pose_from_frame

    pos, rot = pose_from_frame(m1_start_state.robot_base_frame)
    rc = m1_start_state.robot_configuration
    left = np.array([rc[n] for n in HUSKY_DUAL_UR5e_JOINT_NAMES[0]], dtype=float)
    right = np.array([rc[n] for n in HUSKY_DUAL_UR5e_JOINT_NAMES[1]], dtype=float)
    iface = SimpleNamespace(
        position=np.asarray(pos, dtype=float),
        rotation=np.asarray(rot, dtype=float),
        arm_joint_pose=[left, right],
    )
    monitor.huskies = [SimpleNamespace(interface=iface, object=None)]
    monitor.selected_robot_id = 0
    return iface


def _print_roster(monitor, header):
    print(f"\n--- {header} ---")
    for i, mv in enumerate(monitor._loaded_movements):
        role = monitor._match_movement_role(mv)
        has_traj = getattr(mv, 'trajectory', None) is not None
        has_conf = (mv.start_state is not None
                    and getattr(mv.start_state, 'robot_configuration', None) is not None)
        mark = '[PLAN]' if has_traj else '[ -- ]'
        cmark = '[CONF]' if has_conf else '[ -- ]'
        print(f"  [{i}] {mark} {cmark} role={role} id={mv.movement_id!r}")


def _diagnose_free_plan_collision(monitor, mv) -> None:
    """When plan_free_dual_arm rejects `mv.start_state` as in-collision,
    name every body that penetrates the robot at that conf.

    Mirrors what plan_transit_motion's `pp.get_collision_fn` checks:
      - self_collisions on the dual-arm (uses pp.get_self_link_pairs).
      - robot vs each scene["obstacles"] body.
      - attachment-child vs each obstacle.
    Cross-references body ids to names from monitor.static_obstacles +
    rigid_bodies_puids + ghost set + active_bar_name, so 'body10' becomes
    e.g. 'joint_J4-6_male' instead of an opaque integer.
    """
    import pybullet as pb
    import pybullet_planning as pp
    from husky_assembly_teleop.utils import (
        HUSKY_DUAL_UR5e_JOINT_NAMES, vec12_from_conf,
    )

    if mv.start_state is None or mv.start_state.robot_configuration is None:
        print("[diagnose] no start_state.robot_configuration; skipping diagnosis.")
        return

    cid = monitor.cfab.client.client_id
    husky = getattr(monitor, "_bar_action_husky", None)
    if husky is None:
        print("[diagnose] monitor._bar_action_husky unset; skipping.")
        return
    robot = husky.object.robot
    arm_joints = list(pp.joints_from_names(robot, HUSKY_DUAL_UR5e_JOINT_NAMES[0])) \
                 + list(pp.joints_from_names(robot, HUSKY_DUAL_UR5e_JOINT_NAMES[1]))
    start_conf = vec12_from_conf(mv.start_state.robot_configuration)

    # Build body -> name map (covers active_bar, static_obstacles, ghosts, robot).
    name_from_body: dict[int, str] = {}
    for n, ids in (monitor.cfab.client.rigid_bodies_puids or {}).items():
        for i in ids:
            name_from_body[i] = n
    ghosts = getattr(monitor, "_bar_action_ghost_bodies", set()) or set()
    for k, g in enumerate(ghosts):
        name_from_body[g] = f"ghost_ee_{k}"
    if monitor.active_bar_body is not None and monitor.active_bar_name:
        name_from_body[monitor.active_bar_body] = f"{monitor.active_bar_name} (active_bar)"
    name_from_body[robot] = pp.get_body_name(robot) or f"robot#{robot}"

    def _bn(b: int) -> str:
        return f"{name_from_body.get(b, '?')}(#{b})"

    print("\n=== free-plan initial-collision diagnosis ===")
    print(f"start_conf (12): {[round(float(v), 4) for v in start_conf]}")
    print(f"all rigid bodies in cfab scene ({len(name_from_body)}):")
    for b in sorted(name_from_body):
        print(f"  {_bn(b)}")

    # Apply start_state + start_conf, run a real broadphase check.
    saved_client = pp.CLIENT
    pp.CLIENT = cid
    pp.CLIENTS.setdefault(cid, True)
    try:
        with pp.WorldSaver():
            monitor.cfab.planner.set_robot_cell_state(mv.start_state)
            pp.set_joint_positions(robot, arm_joints, start_conf)
            pb.performCollisionDetection(physicsClientId=cid)

            # Self-link collisions (no SRDF disabled set on purpose -- mirrors
            # plan_transit_motion's default since scene["disabled_collisions"]
            # is None).
            self_pairs = pp.get_self_link_pairs(robot, arm_joints, disabled_collisions=set())
            self_hits = []
            for a, b in self_pairs:
                pts = pb.getClosestPoints(robot, robot, distance=0.0,
                                          linkIndexA=a, linkIndexB=b,
                                          physicsClientId=cid)
                if pts:
                    depths = sorted(round(p[8], 4) for p in pts)
                    self_hits.append(
                        f"{pp.get_link_name(robot, a)} <-> "
                        f"{pp.get_link_name(robot, b)} depths={depths}"
                    )
            print(f"\nself-collision hits ({len(self_hits)}):")
            for h in self_hits:
                print(f"  {h}")
            if not self_hits:
                print("  (none)")

            # Robot <-> every other body. Use a small margin (1 mm) so we
            # also surface near-misses worth investigating.
            env_hits = []
            for body, name in sorted({b: name_from_body.get(b, '?')
                                      for b in name_from_body}.items()):
                if body == robot:
                    continue
                pts = pb.getClosestPoints(robot, body, distance=0.001,
                                          physicsClientId=cid)
                if not pts:
                    continue
                depths = sorted(round(p[8], 4) for p in pts)
                links_hit = sorted({p[3] for p in pts})
                link_names = [pp.get_link_name(robot, l) for l in links_hit]
                in_obs = body in set((monitor.static_obstacles or {}).values())
                env_hits.append({
                    "name": _bn(body), "depths": depths,
                    "links": link_names, "in_scene_obstacles": in_obs,
                })
            print(f"\nrobot<->body hits at start_conf "
                  f"(distance<=1mm; <0 == penetration) ({len(env_hits)}):")
            for h in env_hits:
                tag = "OBSTACLE" if h["in_scene_obstacles"] else "non-obstacle"
                print(f"  [{tag}] {h['name']} depths={h['depths']} robot_links={h['links']}")
            if not env_hits:
                print("  (none)")
    finally:
        pp.CLIENT = saved_client

    print("=== end diagnosis ===\n")


def _replay_saved_trajectories(monitor, sequence) -> int:
    """Load <mv>_trajectory.json files from disk for each movement in
    `sequence` (in load order), then open an interactive PyBullet slider
    that scrubs the concatenated waypoint stream across all movements.

    set_robot_cell_state per waypoint moves the robot AND repositions any
    attached rigid bodies (e.g. the bar held to left tool0) rigidly with
    the link, so transitions between movements with different attachments
    render correctly. The renderer-visible 'snap' at M3->M4 represents
    the bar being released into its installed pose -- expected.
    """
    import time
    import pybullet
    from husky_assembly_teleop.utils import (
        HUSKY_DUAL_UR5e_JOINT_NAMES, path_12_from_joint_trajectory,
    )

    # Suppress CDFM validation during replay (it tries to open a matplotlib
    # window per loaded M1-CDFM trajectory). load_selected_movement_trajectory
    # routes through _accept_trajectory -> _validate_cdfm_planned_path.
    orig_validate = monitor._validate_cdfm_planned_path
    monitor._validate_cdfm_planned_path = lambda mv, path: None

    loaded_indices = []
    try:
        for idx in sequence:
            mv = monitor._loaded_movements[idx]
            role = monitor._match_movement_role(mv)
            print(f"\n--- loading trajectory: {role} idx={idx} "
                  f"id={mv.movement_id!r} ---")
            monitor._selected_movement_idx = idx
            monitor.load_selected_movement()
            # Saved filenames are now bar-action-keyed for ALL roles
            # (monitor._trajectory_file_for prepends the active action_id
            # for synthetic ids like M0), so cross-BarAction stale-file
            # contamination is no longer a concern.
            traj_path = monitor._trajectory_file_for(mv)
            if not os.path.exists(traj_path):
                print(f"  skipped: no file at {traj_path}")
                continue
            monitor.load_selected_movement_trajectory()
            if getattr(mv, 'trajectory', None) is not None:
                loaded_indices.append(idx)
    finally:
        monitor._validate_cdfm_planned_path = orig_validate

    if not loaded_indices:
        print("FAIL: no trajectories loaded for replay.")
        return 1

    left_names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0])
    right_names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])

    states = []
    labels = []
    for idx in loaded_indices:
        mv = monitor._loaded_movements[idx]
        path12 = path_12_from_joint_trajectory(mv.trajectory)
        for i, q12 in enumerate(path12):
            wp_state = mv.start_state.copy()
            for j, n in enumerate(left_names):
                wp_state.robot_configuration[n] = float(q12[j])
            for j, n in enumerate(right_names):
                wp_state.robot_configuration[n] = float(q12[6 + j])
            states.append(wp_state)
            labels.append(f"{mv.movement_id} [{i + 1}/{len(path12)}]")

    print(f"\n=== REPLAY: {len(states)} waypoints across "
          f"{len(loaded_indices)} movement(s) ===")
    monitor.cfab.planner.set_robot_cell_state(states[0])

    cid = monitor.cfab.client.client_id
    slider = pybullet.addUserDebugParameter(
        f"Replay t (0..{len(states) - 1})", 0.0, float(len(states) - 1), 0.0,
        physicsClientId=cid,
    )
    print(f"[replay] Drag the slider on the cfab PyBullet panel to scrub. "
          f"Ctrl+C to exit.")

    last_idx = -1
    try:
        while True:
            t = pybullet.readUserDebugParameter(slider, physicsClientId=cid)
            idx = max(0, min(len(states) - 1, int(round(t))))
            if idx != last_idx:
                monitor.cfab.planner.set_robot_cell_state(states[idx])
                print(f"\r[replay] {labels[idx]:<60}", end="", flush=True)
                last_idx = idx
            time.sleep(0.03)
    except KeyboardInterrupt:
        print("\n[replay] exiting.")
    return 0


_PATCHED_PROBLEM = None


def _patched_problem():
    return _PATCHED_PROBLEM


def _design_data_dir():
    from husky_assembly_teleop import DESIGN_DATA_DIRECTORY
    return DESIGN_DATA_DIRECTORY


def main(bar_action: str = DEFAULT_BAR_ACTION,
         problem: str = DEFAULT_PROBLEM,
         use_gui: bool = False,
         only_movement: str | None = None,
         no_save: bool = False,
         replay: bool = False) -> int:
    global _PATCHED_PROBLEM
    _PATCHED_PROBLEM = problem
    mode = 'REPLAY' if replay else 'PLAN'
    print(f"=== headless full-sequence test ({mode}): problem={problem!r} "
          f"bar_action={bar_action!r} only_movement={only_movement!r} ===")

    _patch_design_problem(problem)

    # Replay always needs the GUI (the scrubber slider lives in pybullet's
    # debug-parameter panel).
    if replay and not use_gui:
        print("[replay] --replay forces --gui on; opening the cfab window.")
        use_gui = True

    monitor = _bypass_init_monitor()
    try:
        # Pre-create cfab so load_bar_action_file's `if self.cfab is None`
        # branch is skipped (it would otherwise hardcode connection_type='gui').
        from husky_assembly_teleop.cfab_session import CfabSession
        ctype = "gui" if use_gui else "direct"
        print(f"[cfab] opening cfab PyBullet ({ctype}) session...")
        monitor.cfab = CfabSession(
            problem, connection_type=ctype, enable_debug_gui=use_gui,
        )

        # Pin pp.CLIENT to cfab's client so set_color / draw_pose / Attachment
        # calls inside load_selected_movement + _bridge_cfab_to_pp_for_bar_action
        # route to the right pybullet instance. In a real UI run the monitor
        # already has pp.CLIENT pointed at its own world; in headless we
        # don't have a monitor pp world at all, so we point pp at cfab.
        import pybullet_planning as pp
        pp.CLIENT = monitor.cfab.client.client_id
        pp.CLIENTS[monitor.cfab.client.client_id] = True if use_gui else None

        # Populate the BarAction file slider (UI does this on focus).
        monitor.available_robot_cell_states = monitor._load_available_bar_actions()
        if not monitor.available_robot_cell_states:
            print("FAIL: no BarAction files available")
            return 1
        if bar_action not in monitor.available_robot_cell_states:
            print(f"FAIL: {bar_action!r} not in available BarActions; have "
                  f"{monitor.available_robot_cell_states[:8]}"
                  f"{'...' if len(monitor.available_robot_cell_states) > 8 else ''}")
            return 1
        monitor._selected_action_file_idx = monitor.available_robot_cell_states.index(bar_action)

        # Probe-parse to set up the stub husky interface BEFORE
        # load_bar_action_file calls _make_synthetic_m0 (which reads
        # huskies[0].interface).
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

        print(f"\n--- simulating 'Load BarAction' click ({bar_action}) ---")
        monitor.load_bar_action_file()
        if not monitor._loaded_movements:
            print("FAIL: load_bar_action_file did not populate _loaded_movements")
            return 1

        # Build the per-role index map so --only-movement can target one
        # role without re-running the full sequence.
        role_to_idx: dict[str, int] = {}
        for i, mv in enumerate(monitor._loaded_movements):
            r = monitor._match_movement_role(mv)
            if r and r not in role_to_idx:
                role_to_idx[r] = i

        if only_movement:
            if only_movement not in role_to_idx:
                print(f"FAIL: --only-movement {only_movement!r} not in roster "
                      f"(have {sorted(role_to_idx)})")
                return 1
            sequence = [role_to_idx[only_movement]]
        else:
            # M1 -> M2 -> M3 -> M0 -> M4. Canonical plan order: forward chain
            # M1..M3 first (each Mk plan seeds M(k+1)'s start in memory or via
            # the auto-load propagation). M0 then plans live->M1.start (live
            # = stub interface, goal = M1.start.robot_configuration after M1
            # planning). M4 plans last from M3-end (already propagated to
            # M4.start when M3 was planned) to fixed home.
            sequence = []
            for r in ('M1', 'M2', 'M3', 'M0', 'M4'):
                if r in role_to_idx:
                    sequence.append(role_to_idx[r])

        if not sequence:
            print("FAIL: empty planning sequence (no recognized movement roles found)")
            return 1

        # --- REPLAY BRANCH: skip planning, load saved trajectories, animate ---
        if replay:
            return _replay_saved_trajectories(monitor, sequence)

        # Optional: suppress trajectory JSON writes (the live UI button
        # saves; headless mirrors that by default but --no-save flips it
        # for iteration speed).
        if no_save:
            orig_accept = monitor._accept_trajectory

            def _accept_nosave(mv, jt, **kw):
                kw['save_to_disk'] = False
                return orig_accept(mv, jt, **kw)

            monitor._accept_trajectory = _accept_nosave

        sequence_ids = [monitor._loaded_movements[i].movement_id for i in sequence]
        print(f"\n=== planning sequence ({len(sequence)}): {sequence_ids} ===")

        for step, idx in enumerate(sequence, start=1):
            mv = monitor._loaded_movements[idx]
            role = monitor._match_movement_role(mv)
            print(f"\n=== [{step}/{len(sequence)}] {role} idx={idx} id={mv.movement_id!r} ===")

            print(f"--- simulating 'Load Movement' click (idx={idx}) ---")
            monitor._selected_movement_idx = idx
            monitor.load_selected_movement()
            if monitor.current_movement is None:
                print(f"FAIL: load_selected_movement did not set current_movement.")
                return 1

            print(f"--- simulating 'Plan Movement' click ({role}) ---")
            monitor.plan_selected_movement()
            if getattr(monitor.current_movement, 'trajectory', None) is None:
                print(f"FAIL: {role} {mv.movement_id!r} planning produced no trajectory "
                      f"-- aborting sequence.")
                # For free-plan failures (M0/M4) the most common cause is
                # the initial conf landing inside an env-obstacle. Run the
                # name-per-body diagnostic so we know what to ACM-exclude
                # or fix.
                if role in ('M0', 'M4'):
                    try:
                        _diagnose_free_plan_collision(monitor, mv)
                    except Exception as e:
                        print(f"[diagnose] ERROR: {e}")
                _print_roster(monitor, "roster at failure")
                return 1

        _print_roster(monitor, "FINAL roster")
        print(f"\n=== SEQUENCE COMPLETE: planned {len(sequence)} movement(s). ===")

        if use_gui:
            try:
                input("\n[gui] press Enter to close the PyBullet window...")
            except (EOFError, KeyboardInterrupt):
                pass

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
    parser.add_argument("--gui", action="store_true",
                        help="Open cfab's PyBullet GUI window. Hold the "
                             "window open at the end of the sequence.")
    parser.add_argument("--only-movement", type=str, default=None,
                        choices=('M0', 'M1', 'M2', 'M3', 'M4'),
                        help="Plan a single role only (no sequence). Useful "
                             "for triage after a failure.")
    parser.add_argument("--no-save", action="store_true",
                        help="Skip writing <mv>_trajectory.json under "
                             "<problem>/Trajectories/ (default: save, "
                             "matching the live UI button).")
    parser.add_argument("--replay", action="store_true",
                        help="Skip planning. Load previously saved "
                             "<mv>_trajectory.json files and open an "
                             "interactive scrubber in the cfab GUI window "
                             "to visually inspect them. Forces --gui on.")
    args = parser.parse_args()
    sys.exit(main(
        bar_action=args.bar_action, problem=args.problem,
        use_gui=args.gui, only_movement=args.only_movement,
        no_save=args.no_save, replay=args.replay,
    ))
