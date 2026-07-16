"""Headless full-sequence test for the BarAction (cfab) planning path.

Mirrors the BAR_ACTION_LIVE_REPLAN_EXE UI button sequence in HuskyMonitor:

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

Env-collision behavior matches the live monitor: each dispatcher plans WITH
environment obstacles enabled (obstacles + ACM come from the movement's
start_state; the BarAction JSONs author touch_links/touch_bodies natively).

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
    # 2-slot list so set_arm_trajectory can index into it (real monitor
    # inits this the same way).
    monitor.planned_arm_trajectory = [(None, None, None, None),
                                      (None, None, None, None)]

    monitor.BAR_ACTION_LIVE_REPLAN_EXE = True
    monitor.FAKE_HARDWARE = False
    monitor._is_live_monitor = False
    monitor.goal_base_pose_frozen = False

    def _noop(*a, **kw):
        return None

    # Mirror the real set_arm_trajectory: write into planned_arm_trajectory[index]
    # so headless assertions (e.g. --button replan-m2) can see whether a plan
    # actually populated the per-arm slot.
    def _set_arm_trajectory(traj, index=0):
        monitor.planned_arm_trajectory[index] = traj

    monitor.set_arm_trajectory = _set_arm_trajectory
    monitor.set_to_show_traj_state = _noop
    monitor.set_to_show_goal_state = _noop
    monitor.reset_ui = lambda *a, **kw: None
    monitor._hide_cfab_robot = _noop

    _logger = StubLogger()
    monitor.get_logger = lambda: _logger
    return monitor


def _attach_stub_husky_interface(monitor, m1_start_state):
    """Provide huskies[0].interface for _inject_live_conf_into_state.

    _inject_live_conf_into_state reads .position, .rotation and
    .arm_joint_pose off huskies[0].interface to snapshot the
    "live" robot state. Headless has no ROS / mocap, so we synthesize one.

    Base frame: take from M1.start_state.robot_base_frame (the BarAction
    is authored in M1's base frame; headless puts the robot there).

    Arm joints: use UR5e HOME, NOT M1.start_state.robot_configuration.
    The authored M1 arm conf in BarAction files is typically a placeholder
    (all zeros for B6), not a feasible robot pose. With zeros as "live",
    M0's plan_free_dual_arm needs to sweep the right shoulder ~265° from
    zero to M1's derived start_conf, and the straight-line interp runs
    through dense robot self-collision (base, bulkhead, wheels, arm-vs-
    arm). BiRRT can't find a detour in 30s. The live monitor doesn't see
    this because the real `huskies[0].interface` reports the actual robot
    pose (near HOME). UR5e HOME is the closest analogue in headless.
    """
    from husky_assembly_teleop.utils import HUSKY_DUAL_UR5e_JOINT_NAMES, pose_from_frame
    from husky_assembly_teleop.husky_robot import UR5e_HOME_STATE

    pos, rot = pose_from_frame(m1_start_state.robot_base_frame)
    home = np.asarray(UR5e_HOME_STATE, dtype=float)
    iface = SimpleNamespace(
        position=np.asarray(pos, dtype=float),
        rotation=np.asarray(rot, dtype=float),
        arm_joint_pose=[home.copy(), home.copy()],
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


def _diagnose_m0_transit_failure(monitor, mv) -> None:
    """When M0's plan_free_dual_arm fails with 'transit path not found' but
    initial-conf and goal-conf checks both passed, the BiRRT has feasible
    endpoints it can't connect. This usually means one of:
      - some interpolated waypoint between A (M0.start) and D (M1.start
        after the chain rule) hits an obstacle the BiRRT has to detour
        around within max_time.
      - one or more joints need to traverse > π (e.g. a 2π wrap) and the
        sampler can't find a feasible region quickly.
      - time budget too tight (RNG-dependent miss).

    This helper characterises start → goal in joint space, verifies both
    endpoints are collision-free against the mounted-body-filtered
    obstacle list, and runs a 21-sample
    linear-interpolation sweep to identify any per-step obstacle hits
    along the straight-line path. If no hits, the failure is most likely
    time-budget / RNG; if hits, it points at which bodies are blocking.
    """
    import math
    import pybullet as pb
    import pybullet_planning as pp
    from husky_assembly_teleop.utils import (
        HUSKY_DUAL_UR5e_JOINT_NAMES, vec12_from_conf,
    )

    if len(monitor._loaded_movements) < 2:
        print("[M0 diagnose] no M1 in loaded movements; skipping.")
        return
    m1 = monitor._loaded_movements[1]
    if (m1.start_state is None or m1.start_state.robot_configuration is None
            or mv.start_state is None or mv.start_state.robot_configuration is None):
        print("[M0 diagnose] missing start_state.robot_configuration; skipping.")
        return

    start = np.asarray(vec12_from_conf(mv.start_state.robot_configuration), dtype=float)
    goal = np.asarray(vec12_from_conf(m1.start_state.robot_configuration), dtype=float)
    delta = goal - start

    cid = monitor.cfab.client.client_id
    husky = getattr(monitor, "_bar_action_husky", None)
    if husky is None:
        print("[M0 diagnose] monitor._bar_action_husky unset; skipping.")
        return
    robot = husky.object.robot
    left_names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0])
    right_names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
    joint_names_12 = left_names + right_names
    arm_joints = pp.joints_from_names(robot, joint_names_12)

    name_from_body: dict[int, str] = {}
    for n, ids in (monitor.cfab.client.rigid_bodies_puids or {}).items():
        for i in ids:
            name_from_body[i] = n
    ghosts = getattr(monitor, "_bar_action_ghost_bodies", set()) or set()
    for k, g in enumerate(ghosts):
        name_from_body[g] = f"ghost_ee_{k}"
    if monitor.active_bar_body is not None and monitor.active_bar_name:
        name_from_body[monitor.active_bar_body] = f"{monitor.active_bar_name} (active_bar)"

    # Filter out robot-mounted bodies (tools, held bar) so we check the
    # environment obstacle set only.
    mounted_names: set[str] = set()
    for name, rbs in (getattr(mv.start_state, 'rigid_body_states', None) or {}).items():
        if getattr(rbs, 'attached_to_link', None) is not None:
            mounted_names.add(name)
    obstacle_bodies = [
        body for name, body in (monitor.static_obstacles or {}).items()
        if name not in mounted_names
    ]

    print("\n=== M0 transit-failure diagnosis ===")
    print(f"per-joint Δ (D - A, rad)  |  start (A) -> goal (D):")
    max_idx = int(np.argmax(np.abs(delta)))
    for j, jn in enumerate(joint_names_12):
        d = float(delta[j])
        ad = abs(d)
        wrap_note = ""
        if abs(ad - 2.0 * math.pi) < 0.2:
            wrap_note = "  ~±2π wrap candidate"
        elif ad > math.pi:
            wrap_note = "  > π — sampler may struggle"
        marker = "  <-- max" if j == max_idx else ""
        print(f"  [{j:2d}] {jn}: {start[j]:+.4f} -> {goal[j]:+.4f}, "
              f"Δ = {d:+.4f} rad ({math.degrees(d):+.1f}°){wrap_note}{marker}")
    print(f"max joint Δ: {float(np.abs(delta).max()):.4f} rad "
          f"({math.degrees(float(np.abs(delta).max())):.1f}°)")
    print(f"L2 joint distance: {float(np.linalg.norm(delta)):.4f} rad")
    print(f"scene[\"obstacles\"] count: {len(obstacle_bodies)} "
          f"(mounted bodies excluded by ACM: {sorted(mounted_names)})")

    saved_client = pp.CLIENT
    pp.CLIENT = cid
    pp.CLIENTS.setdefault(cid, True)
    try:
        with pp.WorldSaver():
            monitor.cfab.planner.set_robot_cell_state(mv.start_state)
            self_pairs = pp.get_self_link_pairs(
                robot, arm_joints, disabled_collisions=set(),
            )

            def _check_at(q):
                pp.set_joint_positions(robot, arm_joints, q)
                pb.performCollisionDetection(physicsClientId=cid)
                hits = []
                for a, b in self_pairs:
                    pts = pb.getClosestPoints(
                        robot, robot, distance=0.0,
                        linkIndexA=a, linkIndexB=b, physicsClientId=cid,
                    )
                    if pts:
                        depths = sorted(round(p[8], 4) for p in pts)
                        hits.append(
                            f"SELF: {pp.get_link_name(robot, a)} <-> "
                            f"{pp.get_link_name(robot, b)} d={depths}"
                        )
                for body in obstacle_bodies:
                    pts = pb.getClosestPoints(
                        robot, body, distance=0.0, physicsClientId=cid,
                    )
                    if not pts:
                        continue
                    depths = sorted(round(p[8], 4) for p in pts)
                    links_hit = sorted({p[3] for p in pts})
                    link_names = [pp.get_link_name(robot, l) for l in links_hit]
                    nm = name_from_body.get(body, f"#{body}")
                    hits.append(f"ENV:  {nm} via {link_names} d={depths}")
                return hits

            print(f"\nendpoint feasibility under the planner's collision_fn:")
            for label, q in (("A (M0.start)", start), ("D (M1.start)", goal)):
                hits = _check_at(q)
                print(f"  {label}: {'OK' if not hits else f'{len(hits)} hit(s)'}")
                for h in hits:
                    print(f"    {h}")

            n_samples = 21
            interp_hits: list[tuple[float, list[str]]] = []
            for s in range(n_samples):
                t = s / (n_samples - 1)
                q = start * (1.0 - t) + goal * t
                hits = _check_at(q)
                if hits:
                    interp_hits.append((t, hits))

            print(f"\nlinear-interpolation collision sweep "
                  f"(21 samples, t in [0,1]):")
            if not interp_hits:
                print("  (no collisions along the straight-line interpolation)")
                print("  -> BiRRT failure is most likely RNG / time-budget-bound;")
                print("     try increasing max_time on plan_free_dual_arm "
                      "(default 30s in _plan_M0_dispatch) or pinning the seed.")
            else:
                # Aggregate blockers by name across all colliding t-values.
                blocker_counts: dict[str, int] = {}
                for t, hits in interp_hits:
                    seen = set()
                    for h in hits:
                        # First token after 'ENV:'/'SELF:' is the body/link pair.
                        prefix, _, rest = h.partition(": ")
                        key = f"{prefix}: {rest.split(' d=')[0]}"
                        if key not in seen:
                            blocker_counts[key] = blocker_counts.get(key, 0) + 1
                            seen.add(key)
                print(f"  {len(interp_hits)}/{n_samples} interp points have "
                      f"collisions. Per-blocker count:")
                for key, count in sorted(blocker_counts.items(), key=lambda x: -x[1]):
                    print(f"    {count:2d}/{n_samples}  {key}")
                print(f"  Per-t hits:")
                for t, hits in interp_hits:
                    print(f"    t={t:.3f}:")
                    for h in hits:
                        print(f"      {h}")
                print(f"  -> BiRRT must detour around the listed blocker(s).")
    finally:
        pp.CLIENT = saved_client

    # Probe 1: re-run plan_free_dual_arm with a much larger time budget to
    # see if the default 30s budget was the only limiter.
    # Probe 2: also try planning to a *canonical-form* goal where every
    # joint is unwrapped to within ±π of start (short-way around). This
    # tells us whether the obstruction is the 2π wrap on right shoulder
    # specifically.
    try:
        import math
        from husky_assembly_tamp.motion_planner.api import plan_free_dual_arm
        import pybullet_planning as pp

        saved_client2 = pp.CLIENT
        pp.CLIENT = cid
        pp.CLIENTS.setdefault(cid, True)
        try:
            # Probe 1: same goal, large time budget.
            print("[probe-1] re-running plan_free_dual_arm with max_time=120s...")
            path1, info1 = plan_free_dual_arm(
                monitor.cfab.planner, mv.start_state, goal.tolist(), max_time=120.0,
            )
            if path1 is not None:
                print(f"  -> SUCCESS with 120s: {len(path1)} waypoints. "
                      f"The smaller budget was the only limiter.")
            else:
                print(f"  -> still failed at 120s "
                      f"(failure_reason={info1.get('failure_reason')!r}). "
                      f"More time alone won't help; the wrap is the issue.")

            # Probe 2: unwrap goal to within ±π of start (short-way).
            two_pi = 2.0 * math.pi
            canonical_goal = goal - np.round((goal - start) / two_pi) * two_pi
            max_canon_delta = float(np.abs(canonical_goal - start).max())
            print(f"[probe-2] re-running plan_free_dual_arm to a CANONICAL "
                  f"(±π-of-start) goal (max joint Δ {max_canon_delta:.4f} rad)...")
            path2, info2 = plan_free_dual_arm(
                monitor.cfab.planner, mv.start_state, canonical_goal.tolist(), max_time=30.0,
            )
            if path2 is not None:
                print(f"  -> SUCCESS to canonical goal: {len(path2)} waypoints. "
                      f"Confirms the wrap is what's blocking; M0 can plan "
                      f"the short-way, would need a final wrap-up step to "
                      f"land on the saved M1.start.robot_configuration.")
            else:
                print(f"  -> canonical goal also failed "
                      f"(failure_reason={info2.get('failure_reason')!r}). "
                      f"The path is hard for reasons beyond the wrap.")
        finally:
            pp.CLIENT = saved_client2
    except Exception as e:
        print(f"[probe] ERROR: {e}")

    print("=== end M0 transit-failure diagnosis ===\n")


_TREE_DRAW_ROLES = ('M0', 'M1', 'M4')


def _install_tree_drawing(monitor):
    """Enable BiRRT / SE(3) tree visualization for M0/M4 (free) and M1
    (constrained). Returns a callable that reverts the patches.

    M0/M4 free BiRRT
      pybullet_planning's rrt_connect already supports an optional
      ``draw_fn`` kwarg (rrt_connect.py:69-81) — we monkey-patch the
      module-level ``rrt_connect`` symbol to inject an FK-based draw_fn
      that adds a polyline between parent/child confs on both tool0
      links. ``birrt(...)`` references ``rrt_connect`` via the module
      namespace, so this propagates through ``solve_motion_plan ->
      birrt -> random_restarts(rrt_connect, ...)``.

    M1 constrained
      ``plan_pose_rrt`` already draws its SE(3) tree when
      ``use_draw=True``. We patch ``HuskyMonitor._plan_M1_dispatch``
      to pass ``use_draw=True`` into ``plan_constrained_dual_arm``.
    """
    import contextlib
    import importlib
    import pybullet
    import pybullet_planning as pp
    # Use importlib to get the SUBMODULE: pybullet_planning.motion_planners
    # re-exports the rrt_connect function, so `from … import rrt_connect`
    # returns the function rather than the module we want to monkey-patch.
    _rrt_mod = importlib.import_module(
        'pybullet_planning.motion_planners.rrt_connect',
    )
    _task_rrt_core = importlib.import_module(
        'husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.core',
    )
    from husky_assembly_teleop.utils import HUSKY_DUAL_UR5e_JOINT_NAMES

    husky = getattr(monitor, "_bar_action_husky", None)
    if husky is None or getattr(husky, "object", None) is None:
        # _bar_action_husky is set when load_selected_movement bridges
        # the cfab scene to pp; safe to call this helper after the first
        # load. If absent, fall back to first registered husky.
        if monitor.huskies:
            husky = monitor.huskies[0]
        else:
            print("[draw-tree] no husky available; tree drawing disabled.")
            return lambda: None
    robot = husky.object.robot
    cid = monitor.cfab.client.client_id

    # Clear any existing debug overlays so the tree drawing starts on a
    # clean canvas (preview-arrow uids etc. would otherwise mingle).
    try:
        pp.remove_all_debug()
        print("[draw-tree] pp.remove_all_debug() cleared prior overlays.")
    except Exception as e:
        print(f"[draw-tree] remove_all_debug warn: {e}")

    left_joints = list(pp.joints_from_names(robot, HUSKY_DUAL_UR5e_JOINT_NAMES[0]))
    right_joints = list(pp.joints_from_names(robot, HUSKY_DUAL_UR5e_JOINT_NAMES[1]))
    arm_joints = left_joints + right_joints
    left_tool = pp.link_from_name(robot, 'left_ur_arm_tool0')
    right_tool = pp.link_from_name(robot, 'right_ur_arm_tool0')

    # --- Bar-midpoint offset for M1 drawing (local frame) ---
    # Walk the active bar's visual mesh vertices once; centroid =
    # ((min+max)/2 per axis) in the bar's local frame. Multiplying the bar
    # pose by (centroid, identity_quat) yields the bar's geometric
    # midpoint in world frame at any tree node. Falls back to (0,0,0) if
    # the mesh data isn't reachable.
    def _compute_bar_midpoint_offset_local():
        try:
            name = getattr(monitor, 'active_bar_name', None)
            if not name:
                return (0.0, 0.0, 0.0)
            rb_model = monitor.cfab.robot_cell.rigid_body_models.get(name)
            if rb_model is None:
                return (0.0, 0.0, 0.0)
            meshes = (getattr(rb_model, 'visual_meshes_in_meters', None)
                      or getattr(rb_model, 'collision_meshes_in_meters', None)
                      or [])
            if not meshes:
                return (0.0, 0.0, 0.0)
            mn = [float('inf')] * 3
            mx = [float('-inf')] * 3
            for m in meshes:
                for v in m.vertices():
                    pt = m.vertex_coordinates(v)
                    for i in range(3):
                        if pt[i] < mn[i]: mn[i] = pt[i]
                        if pt[i] > mx[i]: mx[i] = pt[i]
            return ((mn[0] + mx[0]) / 2,
                    (mn[1] + mx[1]) / 2,
                    (mn[2] + mx[2]) / 2)
        except Exception as e:
            print(f"[draw-tree] bar midpoint offset compute warn: {e}")
            return (0.0, 0.0, 0.0)

    bar_mid_offset_local = _compute_bar_midpoint_offset_local()
    print(f"[draw-tree] M1 bar midpoint offset (local frame): "
          f"{tuple(round(c, 4) for c in bar_mid_offset_local)}")

    def _bar_midpoint_world(pose):
        # pose is (pos_xyz, quat_xyzw); pp.multiply(pose, (offset, identity))
        # composes orientation onto the offset before adding pos.
        midpoint_pose = pp.multiply(pose, (bar_mid_offset_local, (0.0, 0.0, 0.0, 1.0)))
        return midpoint_pose[0]

    def _fk_xyz(conf):
        pp.set_joint_positions(robot, arm_joints, conf)
        return (pp.get_link_pose(robot, left_tool)[0],
                pp.get_link_pose(robot, right_tool)[0])

    drawn_edges: set = set()

    def _draw_fn(config, segment, *args):
        # rrt_connect calls draw_fn(target, []) on each sample plus
        # node.draw(draw_fn) on each tree node (config, [child, parent]).
        if not segment:
            return
        try:
            child_conf, parent_conf = segment
        except (TypeError, ValueError):
            return
        key = (tuple(round(float(v), 5) for v in parent_conf),
               tuple(round(float(v), 5) for v in child_conf))
        if key in drawn_edges:
            return
        drawn_edges.add(key)
        try:
            lc, rc = _fk_xyz(child_conf)
            lp, rp = _fk_xyz(parent_conf)
            pp.add_line(lp, lc, color=(0.2, 0.6, 1.0), width=1.5)
            pp.add_line(rp, rc, color=(1.0, 0.5, 0.2), width=1.5)
        except Exception:
            pass

    # --- patch rrt_connect to inject draw_fn ---
    # Only inject draw_fn when planning M0/M4 (free dual-arm BiRRT).
    # M1 also internally drives joint-space BiRRT (free staging inside
    # plan_pose_birrt); for M1 we want the bar-midpoint SE(3) tree only,
    # not the left/right tool0 FK trees, so we gate by current role.
    _BIRRT_DRAW_ROLES = {'M0', 'M4'}
    _orig_rrt_connect = _rrt_mod.rrt_connect

    def _rrt_connect_with_draw(q1, q2, distance_fn, sample_fn, extend_fn,
                                collision_fn, **kwargs):
        cur_mv = getattr(monitor, 'current_movement', None)
        role = monitor._match_movement_role(cur_mv) if cur_mv is not None else None
        if role in _BIRRT_DRAW_ROLES:
            kwargs.setdefault('draw_fn', _draw_fn)
        return _orig_rrt_connect(q1, q2, distance_fn, sample_fn, extend_fn,
                                  collision_fn, **kwargs)
    _rrt_mod.rrt_connect = _rrt_connect_with_draw
    print("[draw-tree] M0/M4 free BiRRT: rrt_connect patched to draw tree "
          "(left=blue, right=orange). Disabled for M1 (bar-midpoint only).")

    # --- patch extend_toward to draw M1 SE(3) edges at the bar's midpoint ---
    # Original draws pp.add_line(current.config[0], node.config[0]) which is
    # the bar's frame-origin trajectory. We want the bar's geometric centroid
    # trajectory instead. Strategy: wrap extend_toward, suppress its internal
    # draw (force use_draw=False), then walk newly appended nodes and emit
    # midpoint edges.
    _orig_extend_toward = _task_rrt_core.extend_toward

    def _extend_toward_midpoint(nodes, source, target_pose, *args, **kwargs):
        n_before = len(nodes)
        # Preserve the caller's draw_color for our redraw; force internal off.
        caller_use_draw = kwargs.get('use_draw', None)
        if caller_use_draw is None and len(args) >= 4:
            # use_draw is positional index 4 (after collision_fn,
            # joint_collision_fn, draw_color, use_draw...). The current
            # call site passes by keyword so this branch is defensive.
            caller_use_draw = args[3]
        draw_color = kwargs.get('draw_color', None)
        if draw_color is None and len(args) >= 3:
            draw_color = args[2]
        if draw_color is None:
            draw_color = (0.2, 0.6, 1.0, 1.0)
        kwargs['use_draw'] = False
        result = _orig_extend_toward(nodes, source, target_pose, *args, **kwargs)
        if caller_use_draw:
            for i in range(n_before, len(nodes)):
                node = nodes[i]
                parent = node.parent
                if parent is None:
                    continue
                try:
                    p1 = _bar_midpoint_world(parent.config)
                    p2 = _bar_midpoint_world(node.config)
                    pp.add_line(p1, p2, width=1.5, color=draw_color)
                except Exception:
                    pass
        return result

    _task_rrt_core.extend_toward = _extend_toward_midpoint
    print("[draw-tree] M1 constrained: extend_toward patched to draw SE(3) "
          "tree at bar midpoint (color from planner's per-tree palette).")

    # --- patch _plan_M1_dispatch: same state-based call, use_draw=True ---
    _orig_m1_dispatch = monitor._plan_M1_dispatch.__func__

    def _patched_m1(self_, mv):
        from husky_assembly_tamp.motion_planner.api import plan_constrained_dual_arm
        from husky_assembly_teleop.utils import joint_trajectory_from_path
        from husky_assembly_teleop.husky_monitor import M1_POSITION_RES, M1_ROTATION_RES
        path, info = plan_constrained_dual_arm(
            self_.cfab.planner, mv.start_state,
            active_bar_id=self_.active_bar_name,
            goal_ee_frames=mv.target_ee_frames,
            stage=self_.constrained_planner_stage,
            position_res=M1_POSITION_RES,
            rotation_res=M1_ROTATION_RES,
            max_time=60.0,
            derive_start=True,
            use_draw=True,
        )
        if path is None:
            print(f"[M1] plan_constrained_dual_arm failed: {info.get('failure_reason')}")
            return None
        self_.constrained_trajectory = [
            (np.asarray([q[:6] for q in path]), None, self_.trajectory_time, None),
            (np.asarray([q[6:] for q in path]), None, self_.trajectory_time, None),
        ]
        return joint_trajectory_from_path(path)

    # Bind the patched dispatch onto the instance.
    monitor._plan_M1_dispatch = _patched_m1.__get__(monitor, type(monitor))
    print("[draw-tree] M1 constrained: _plan_M1_dispatch patched (use_draw=True).")

    def _revert():
        _rrt_mod.rrt_connect = _orig_rrt_connect
        _task_rrt_core.extend_toward = _orig_extend_toward
        monitor._plan_M1_dispatch = _orig_m1_dispatch.__get__(monitor, type(monitor))
        print("[draw-tree] patches reverted.")

    return _revert


def _replay_saved_trajectories(monitor, sequence) -> int:
    """Iterate each movement in `sequence`, pull its in-memory trajectory into
    the viz, then open an interactive PyBullet slider that scrubs the
    concatenated waypoint stream across all movements.

    Trajectories now live only on `mv.trajectory` (set by planning during
    this run, or by loading a `<action>.live-solved.json` sidecar via
    `load_bar_action_file`). Any movement without an in-memory trajectory is
    skipped.

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
            print(f"\n--- staging trajectory: {role} idx={idx} "
                  f"id={mv.movement_id!r} ---")
            monitor._selected_movement_idx = idx
            monitor.load_selected_movement()
            if getattr(mv, 'trajectory', None) is None:
                print(f"  skipped: no in-memory trajectory (plan first, or "
                      "load a `.live-solved.json` sidecar via --bar-action).")
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


def _run_button_mode(monitor, sequence, role_to_idx, button: str,
                     bar_action: str) -> int:
    """Drive one of the new consolidated BarAction buttons end-to-end.

    button choices:
      * ``chain``      -- call ``plan_movement_chain_live()`` and assert the
                          `<action>.live-solved.json` sidecar was written and
                          round-trips via ``parse_bar_action`` with at least
                          one movement carrying a fresh trajectory.
      * ``replan-m2``  -- plan M1 first (so M2's ``start_state.robot_configuration``
                          gets propagated), load M2, then call
                          ``replan_free_to_movement_start_live()`` and verify
                          ``monitor.planned_arm_trajectory`` was populated.
      * ``replan-m3``  -- plan M1 + M2 first (so M3's start is populated),
                          load M3, then call
                          ``replan_free_to_movement_start_live()`` and verify
                          ``monitor.planned_arm_trajectory``.
    """
    if button == 'chain':
        print(f"\n=== [button=chain] running plan_movement_chain_live() ===")
        monitor.plan_movement_chain_live()

        # Assert sidecar written + round-trips.
        stem, ext = os.path.splitext(monitor._current_action_path)
        sidecar_path = f"{stem}.live-solved{ext}"
        if not os.path.isfile(sidecar_path):
            print(f"FAIL: expected sidecar {sidecar_path!r} not found.")
            return 1
        from husky_assembly_teleop.bar_action_io import parse_bar_action
        try:
            reloaded = parse_bar_action(sidecar_path)
        except Exception as e:
            print(f"FAIL: sidecar {sidecar_path!r} did not round-trip: {e}")
            return 1
        with_traj = [
            mv.movement_id for mv in reloaded.movements
            if getattr(mv, 'trajectory', None) is not None
        ]
        if not with_traj:
            print(f"FAIL: sidecar round-trip has zero trajectories.")
            return 1
        print(f"[button=chain] OK: sidecar has {len(with_traj)} movement "
              f"trajectories: {with_traj}")
        _print_roster(monitor, "FINAL roster (button=chain)")
        return 0

    if button in ('replan-m2', 'replan-m3'):
        target_role = 'M2' if button == 'replan-m2' else 'M3'
        # Prerequisites: plan M1 (and M2 for the m3 case) so the target
        # movement's start_state.robot_configuration is populated by
        # forward-chain propagation.
        prereq_roles = ['M1'] if target_role == 'M2' else ['M1', 'M2']
        missing = [r for r in prereq_roles + [target_role] if r not in role_to_idx]
        if missing:
            print(f"FAIL: BarAction lacks required roles {missing}.")
            return 1

        for r in prereq_roles:
            idx = role_to_idx[r]
            mv = monitor._loaded_movements[idx]
            print(f"\n=== [button={button}] pre-plan {r} idx={idx} "
                  f"id={mv.movement_id!r} ===")
            monitor._selected_movement_idx = idx
            monitor.load_selected_movement()
            monitor.plan_selected_movement()
            if getattr(monitor.current_movement, 'trajectory', None) is None:
                print(f"FAIL: prerequisite {r} planning failed; cannot "
                      f"exercise replan on {target_role}.")
                return 1

        # Now load the target movement and exercise Button 2. Reset
        # planned_arm_trajectory to a sentinel first so we can tell whether
        # Button 2 actually wrote a fresh plan (M1's plan already populated
        # planned_arm_trajectory; a failed IK inside Button 2 would leave
        # that old data behind and mislead the assertion).
        idx = role_to_idx[target_role]
        mv = monitor._loaded_movements[idx]
        print(f"\n=== [button={button}] running "
              f"replan_free_to_movement_start_live() on {target_role} "
              f"idx={idx} id={mv.movement_id!r} ===")
        monitor._selected_movement_idx = idx
        monitor.load_selected_movement()

        # Note: `replan_free_to_movement_start_live` internally applies the
        # MOCK live pose when HuskyMonitor.MOCK_LIVE_POSE_FOR_REPLAN is on
        # (see husky_monitor.py:_apply_mock_live_pose_for_replan). Toggle
        # that class flag to disable the mock.
        monitor.planned_arm_trajectory = [(None, None, None, None),
                                          (None, None, None, None)]
        monitor.replan_free_to_movement_start_live()

        pat = monitor.planned_arm_trajectory
        if (pat is None
                or pat[0] is None or pat[0][0] is None
                or pat[1] is None or pat[1][0] is None):
            print(f"FAIL [button={button}]: planned_arm_trajectory not "
                  f"populated after replan_free_to_movement_start_live "
                  f"(the live-base IK or composite free plan step failed).")
            _print_roster(monitor, f"FINAL roster (button={button})")
            return 1
        n_left = len(pat[0][0])
        n_right = len(pat[1][0])
        print(f"[button={button}] OK: planned_arm_trajectory left={n_left} wp, "
              f"right={n_right} wp.")
        _print_roster(monitor, f"FINAL roster (button={button})")
        return 0

    print(f"FAIL: unknown button mode {button!r}.")
    return 1


def _smoke_check_conf12_from_target() -> None:
    """Fail fast if the tamp helper still crashes on a Configuration goal.

    Regression guard for the Configuration-adoption refactor (Change 1 of
    the plan): `_conf12_from_target` used to reject numpy 12-vec goals with
    `IndexError`; the tamp helper's documented contract is that a compas
    Configuration works, and every monitor/world call site now passes one.
    """
    import numpy as _np
    from husky_assembly_teleop.utils import conf_from_12vec, HUSKY_DUAL_UR5e_JOINT_NAMES
    from husky_assembly_tamp.motion_planner.api import _conf12_from_target
    names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]) + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
    v = _conf12_from_target(conf_from_12vec(_np.zeros(12)), names)
    assert v.shape == (12,), f"smoke check produced shape {v.shape}, expected (12,)"
    print(f"[smoke] _conf12_from_target(Configuration) -> shape {v.shape} OK")


def main(bar_action: str = DEFAULT_BAR_ACTION,
         problem: str = DEFAULT_PROBLEM,
         use_gui: bool = False,
         only_movement: str | None = None,
         no_save: bool = False,
         replay: bool = False,
         draw_tree: bool = False,
         button: str | None = None) -> int:
    global _PATCHED_PROBLEM
    _PATCHED_PROBLEM = problem
    mode = 'REPLAY' if replay else ('BUTTON' if button else 'PLAN')
    print(f"=== headless full-sequence test ({mode}): problem={problem!r} "
          f"bar_action={bar_action!r} only_movement={only_movement!r} "
          f"draw_tree={draw_tree} button={button!r} ===")

    _smoke_check_conf12_from_target()

    _patch_design_problem(problem)

    # Replay always needs the GUI (the scrubber slider lives in pybullet's
    # debug-parameter panel).
    if replay and not use_gui:
        print("[replay] --replay forces --gui on; opening the cfab window.")
        use_gui = True

    # Tree drawing only makes sense with the GUI open.
    if draw_tree and not use_gui:
        print("[draw-tree] --draw-tree forces --gui on (rendering required).")
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
        monitor.available_bar_actions = monitor._load_available_bar_actions()
        if not monitor.available_bar_actions:
            print("FAIL: no BarAction files available")
            return 1
        if bar_action not in monitor.available_bar_actions:
            print(f"FAIL: {bar_action!r} not in available BarActions; have "
                  f"{monitor.available_bar_actions[:8]}"
                  f"{'...' if len(monitor.available_bar_actions) > 8 else ''}")
            return 1
        monitor._selected_action_file_idx = monitor.available_bar_actions.index(bar_action)

        # Probe-parse to set up the stub husky interface BEFORE
        # load_bar_action_file calls _inject_live_conf_into_state on the
        # native M0 (which reads huskies[0].interface).
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

        # --- BUTTON MODE: drive Button 1 (chain) or Button 2 (replan-m2/m3)
        # end-to-end, then exit. Verifies the new consolidated UI methods
        # without user interaction.
        if button:
            return _run_button_mode(monitor, sequence, role_to_idx, button,
                                    bar_action)

        # `--no-save` used to suppress the per-movement JSON write inside
        # `_accept_trajectory`; per-movement JSONs no longer exist (all
        # persistence now goes through the `<action>.live-solved.json`
        # sidecar written by `plan_movement_chain_live`). Kept for CLI
        # backward-compat -- warn and ignore.
        if no_save:
            print("[--no-save] deprecated; per-mv JSON persistence removed "
                  "(sidecar-only). Ignoring the flag.")

        sequence_ids = [monitor._loaded_movements[i].movement_id for i in sequence]
        print(f"\n=== planning sequence ({len(sequence)}): {sequence_ids} ===")

        revert_tree_drawing = None
        sequence_roles = {monitor._match_movement_role(monitor._loaded_movements[i])
                          for i in sequence}
        draw_tree_active = draw_tree and bool(sequence_roles & set(_TREE_DRAW_ROLES))
        if draw_tree and not draw_tree_active:
            print(f"[draw-tree] sequence has no role in {_TREE_DRAW_ROLES}; "
                  f"skipping patch install.")

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

            # _bar_action_husky is set by load_selected_movement's
            # cfab->pp bridge; install tree-draw patches on first load.
            if draw_tree_active and revert_tree_drawing is None:
                revert_tree_drawing = _install_tree_drawing(monitor)

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
                    # When start_conf was feasible but BiRRT couldn't
                    # connect to goal, dig into per-joint Δ and linear-
                    # interpolation collisions to identify what's blocking.
                    if role == 'M0':
                        try:
                            _diagnose_m0_transit_failure(monitor, mv)
                        except Exception as e:
                            print(f"[diagnose] M0 transit ERROR: {e}")
                _print_roster(monitor, "roster at failure")
                return 1

        _print_roster(monitor, "FINAL roster")
        print(f"\n=== SEQUENCE COMPLETE: planned {len(sequence)} movement(s). ===")

        if revert_tree_drawing is not None:
            try:
                revert_tree_drawing()
            except Exception as e:
                print(f"[draw-tree] revert ERROR: {e}")

        if use_gui:
            # Drop into the replay scrubber so the user can step through
            # the trajectory they just planned. Replay reads the trajectories
            # that are now in memory on each mv (no per-mv JSON on disk).
            print(f"\n=== entering REPLAY mode for the planned trajectory ===")
            return _replay_saved_trajectories(monitor, sequence)

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
                        help="Deprecated no-op (per-movement JSON persistence "
                             "was replaced by the `<action>.live-solved.json` "
                             "sidecar written by Plan Chain (Live)).")
    parser.add_argument("--replay", action="store_true",
                        help="Skip planning. Uses in-memory trajectories "
                             "populated by loading a `.live-solved.json` "
                             "sidecar via --bar-action. Opens an interactive "
                             "scrubber in the cfab GUI window. Forces --gui on.")
    parser.add_argument("--draw-tree", action="store_true",
                        help="Draw planning trees in the cfab GUI for M0 / "
                             "M1 / M4 plans (free BiRRT + constrained "
                             "SE(3) RRT). Forces --gui on. Left arm edges "
                             "are blue, right arm edges are orange.")
    parser.add_argument("--button", type=str, default=None,
                        choices=('chain', 'replan-m2', 'replan-m3'),
                        help="Exercise one of the new consolidated BarAction "
                             "buttons instead of the standard M1->M2->M3->M0->M4 "
                             "loop. 'chain' calls plan_movement_chain_live() and "
                             "asserts the sidecar. 'replan-m2' / 'replan-m3' "
                             "plan the M1 (+M2 for m3) prerequisites, then "
                             "call replan_free_to_movement_start_live().")
    args = parser.parse_args()
    sys.exit(main(
        bar_action=args.bar_action, problem=args.problem,
        use_gui=args.gui, only_movement=args.only_movement,
        no_save=args.no_save, replay=args.replay,
        draw_tree=args.draw_tree, button=args.button,
    ))
