"""Headless integration test for the BarAction (cfab) planning path.

Uses ONLY the cfab PyBullet client (compas_fab `PyBulletPlanner`) — no
parallel `pybullet_planning` world. Drives real ``HuskyMonitor`` methods
without ROS / mocap:

  1. ``monitor.load_bar_action(action_path, movement)``  <- "Load BarAction" button
  2. ``husky_world.plan_and_stage_constrained(monitor)`` <- emulates the
     "Plan & Stage Constrained" button; runs the staging + constrained
     dual-arm planners against the loaded movement.

With ``--gui``, the cfab client opens its own PyBullet window (showing
the full RobotCell scene: husky + rigid bodies including tools and
bars) and two debug sliders (staging + constrained) let you scrub the
two resulting trajectories.

Usage (with ros2_ws venv active + install/setup.bash sourced):
  python src/husky-assembly-teleop/scripts/headless_live_monitor_test.py \\
      --bar-action B6.json --movement M1 [--gui]

  # save successful plan + replay later:
  ... --save-plan /tmp/B6_M1.json
  ... --replay  /tmp/B6_M1.json --gui
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from types import SimpleNamespace

import numpy as np


DEFAULT_PROBLEM = "2026-05-14_foc_demo_reduced"
DEFAULT_BAR_ACTION = "B6.json"
DEFAULT_MOVEMENT = "M1"
HEADLESS_DISABLE_ENVIRONMENT_COLLISIONS = True


class StubLogger:
    def warn(self, msg):  print(f"[WARN] {msg}")
    def info(self, msg):  print(f"[INFO] {msg}")
    def error(self, msg): print(f"[ERROR] {msg}")


def _patch_validation_problem(problem: str) -> None:
    from husky_assembly_teleop import husky_monitor as hm
    hm.DESIGN_PROBLEM_NAME = problem


def _bypass_init_monitor():
    """Construct a HuskyMonitor without running __init__ (no ROS / mocap / pp)."""
    from husky_assembly_teleop.husky_monitor import HuskyMonitor

    monitor = object.__new__(HuskyMonitor)

    # NOTE: no pp client, no pp robot, no static_obstacles dict. cfab owns
    # the full scene. `huskies` is an empty list — load_bar_action and the
    # planner entry don't read from it (we patch it post-load in main()).
    monitor.huskies = []
    monitor.selected_robot_id = 0
    monitor.static_obstacles = {}

    # Legacy active-bar tracking (unused on BarAction path).
    monitor.active_bar_body = None
    monitor.active_bar_aabb_dims = None
    monitor.active_bar_name = None
    monitor.active_extra_bodies = []
    monitor.bar_from_extra = []

    # BarAction / cfab state (populated by load_bar_action).
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

    # State-slider state (BarAction filenames in this list).
    monitor.available_robot_cell_states = []
    monitor.selected_state_index = 0
    monitor.available_joint_trajectories = []
    monitor.selected_trajectory_index = 0

    # Goal interface state.
    monitor.goal_arm_pose = [np.zeros(6), np.zeros(6)]
    monitor.goal_base_pose = (np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]))
    monitor.goal_model = SimpleNamespace(set_pose=lambda base_pose, arm_pose: None)
    monitor.show_goal_state = False
    monitor.trajectory_time = 20.0
    monitor.selected_arm_index = 0

    # Method stubs.
    captured = {"reset_ui_calls": 0, "show_goal_state_calls": 0,
                "set_arm_trajectory": []}

    def _set_arm_trajectory(traj, index=0):
        captured["set_arm_trajectory"].append((index, traj))

    def _set_to_show_traj_state():
        pass

    def _set_to_show_goal_state():
        captured["show_goal_state_calls"] += 1

    def _reset_ui(target_conf=None):
        captured["reset_ui_calls"] += 1

    monitor.set_arm_trajectory = _set_arm_trajectory
    monitor.set_to_show_traj_state = _set_to_show_traj_state
    monitor.set_to_show_goal_state = _set_to_show_goal_state
    monitor.reset_ui = _reset_ui

    _logger = StubLogger()
    monitor.get_logger = lambda: _logger
    monitor._captured = captured
    return monitor


def _sample_feasible_staging_seed(monitor, base_conf, *,
                                  max_attempts: int = 200, perturb: float = 0.6,
                                  seed: int = 0):
    """Sample a collision-free 12-DOF config near `base_conf` for the staging
    plan's START. Only relevant in the headless test — the live monitor
    starts from a real (always-feasible) robot pose.

    UR5e HOME = [0, -π/2, 0, -π/2, 0, π/2] sits ~2mm inside the
    shoulder/base self-collision margin on the dual-arm husky URDF, which
    makes `plan_free_dual_arm` reject it as "initial conf in collision".

    Uses `pp.get_collision_fn` with `self_collisions=1, max_distance=0` to
    match exactly what `plan_transit_motion` (the staging planner) checks
    internally — so a seed that passes here will pass there too.
    """
    import pybullet_planning as pp
    from husky_assembly_teleop.utils import HUSKY_DUAL_UR5e_JOINT_NAMES

    all_names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]) + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
    robot = monitor.cfab.client.robot_puid
    all_joints = pp.joints_from_names(robot, all_names)
    collision_fn = pp.get_collision_fn(
        robot, all_joints,
        obstacles=[],          # staging excludes the bar; static obstacles are world objs
        attachments=[],        # staging plan doesn't carry the bar
        self_collisions=1,
        max_distance=0,
    )

    base_conf = np.asarray(base_conf, dtype=float)
    rng = np.random.default_rng(seed)

    with pp.WorldSaver():
        for attempt in range(max_attempts):
            q = base_conf if attempt == 0 else (
                base_conf + rng.uniform(-perturb, perturb, size=12))
            if not collision_fn(tuple(q.tolist())):
                if attempt > 0:
                    print(f"[seed] feasible staging seed sampled at attempt "
                          f"{attempt+1}/{max_attempts} (|Δ|={float(np.linalg.norm(q - base_conf)):.3f} rad).")
                return q
    print(f"[seed] WARN: no collision-free staging seed in {max_attempts} "
          f"attempts; falling back to base conf.")
    return base_conf


def _save_plan(monitor, path: str) -> None:
    """Dump the planned trajectories + metadata to JSON for offline replay."""
    from husky_assembly_teleop.utils import HUSKY_DUAL_UR5e_JOINT_NAMES

    def _arm_path_from_traj(traj):
        if traj is None or traj[0] is None or traj[1] is None:
            return None
        return {
            "left":  np.asarray(traj[0][0], dtype=float).tolist(),
            "right": np.asarray(traj[1][0], dtype=float).tolist(),
        }

    payload = {
        "schema": "husky_bar_action_plan/v1",
        "bar_action": getattr(monitor.current_action, "action_id", None),
        "movement_id": monitor.current_movement.movement_id,
        "movement_index": monitor.current_movement_index,
        "joint_names": {
            "left":  list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]),
            "right": list(HUSKY_DUAL_UR5e_JOINT_NAMES[1]),
        },
        "staging_trajectory": _arm_path_from_traj(
            getattr(monitor, "staging_free_trajectory", None)),
        "constrained_trajectory": _arm_path_from_traj(
            getattr(monitor, "constrained_trajectory", None)),
    }
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    with open(path, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"[save] plan written to {path}")


def _load_plan(monitor, path: str) -> bool:
    """Populate `monitor.constrained_trajectory` and `staging_free_trajectory`
    from a saved plan JSON. Returns True iff a constrained trajectory was
    loaded (the staging trajectory is optional).
    """
    with open(path) as f:
        payload = json.load(f)
    if payload.get("schema") != "husky_bar_action_plan/v1":
        print(f"[load] WARN: unknown schema {payload.get('schema')!r}")

    def _traj_from_dict(d):
        if not d:
            return [None, None]
        return [
            (np.asarray(d["left"],  dtype=float), None, monitor.trajectory_time, None),
            (np.asarray(d["right"], dtype=float), None, monitor.trajectory_time, None),
        ]

    monitor.staging_free_trajectory = _traj_from_dict(payload.get("staging_trajectory"))
    monitor.constrained_trajectory  = _traj_from_dict(payload.get("constrained_trajectory"))
    has_c = monitor.constrained_trajectory[0] is not None
    has_s = monitor.staging_free_trajectory[0] is not None
    print(f"[load] plan loaded: movement_id={payload.get('movement_id')!r}, "
          f"staging={'yes' if has_s else 'no'}, "
          f"constrained={'yes' if has_c else 'no'}")
    return has_c


def _load_compas_trajectory(monitor, path: str) -> bool:
    """Load a compas `JointTrajectory` JSON (as written by the husky monitor's
    'Export Constrained Dual-Arm Traj' button) into
    `monitor.constrained_trajectory` and reconstruct enough
    `_bar_action_plan_ctx` for `_run_path_validation` to run.

    Requires the BarAction scene to be set up first (load_bar_action +
    cfab→pp bridge), so monitor.cfab, monitor._bar_action_husky,
    monitor.active_bar_body and monitor.movement_start_state are populated.
    Returns True iff a constrained trajectory was loaded.
    """
    import pybullet_planning as pp
    from compas_fab.robots import JointTrajectory
    from husky_assembly_teleop.utils import HUSKY_DUAL_UR5e_JOINT_NAMES, pose_from_frame
    from husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.run import (
        HUSKY_DUAL_ARM_JOINT_NAMES,
    )

    try:
        jt = JointTrajectory.from_json(path)
    except Exception as e:
        print(f"[trajectory] FAIL: could not load {path}: {e}")
        return False
    traj_names = list(jt.joint_names) if jt.joint_names else None
    if not traj_names or len(traj_names) < 12:
        print(f"[trajectory] FAIL: insufficient joint_names in {path} ({traj_names!r})")
        return False
    try:
        order = [traj_names.index(n) for n in HUSKY_DUAL_ARM_JOINT_NAMES]
    except ValueError as e:
        print(f"[trajectory] FAIL: missing required joint: {e}")
        return False
    left_idx, right_idx = order[:6], order[6:]

    left_path, right_path = [], []
    for pt in jt.points:
        names = list(pt.joint_names) if pt.joint_names else traj_names
        if names == traj_names:
            li, ri = left_idx, right_idx
        else:
            local = [names.index(n) for n in HUSKY_DUAL_ARM_JOINT_NAMES]
            li, ri = local[:6], local[6:]
        jv = pt.joint_values
        left_path.append(np.array([jv[i] for i in li], dtype=float))
        right_path.append(np.array([jv[i] for i in ri], dtype=float))
    n = len(left_path)
    if n == 0:
        print(f"[trajectory] FAIL: empty trajectory in {path}")
        return False
    left_arr = np.asarray(left_path)
    right_arr = np.asarray(right_path)
    monitor.constrained_trajectory = [
        (left_arr,  None, monitor.trajectory_time, None),
        (right_arr, None, monitor.trajectory_time, None),
    ]
    monitor.staging_free_trajectory = [None, None]

    # Reconstruct _bar_action_plan_ctx for _run_path_validation.
    ss = monitor.movement_start_state
    if ss is None:
        print("[trajectory] FAIL: monitor.movement_start_state unset; "
              "load_bar_action must run first.")
        return False
    rb = (ss.rigid_body_states or {}).get(monitor.active_bar_name)
    if rb is None or rb.attachment_frame is None:
        print(f"[trajectory] FAIL: active bar {monitor.active_bar_name!r} has no "
              f"attachment_frame in start_state.")
        return False
    left_tool0_from_bar = pose_from_frame(rb.attachment_frame)
    grasp_bar_from_left = pp.invert(left_tool0_from_bar)

    husky = getattr(monitor, "_bar_action_husky", None)
    if husky is None:
        print("[trajectory] FAIL: monitor._bar_action_husky unset; "
              "cfab→pp bridge must run first.")
        return False
    robot = husky.object.robot
    arm_joints = (
        list(pp.joints_from_names(robot, HUSKY_DUAL_UR5e_JOINT_NAMES[0]))
        + list(pp.joints_from_names(robot, HUSKY_DUAL_UR5e_JOINT_NAMES[1]))
    )
    tool_link_L = pp.link_from_name(robot, 'left_ur_arm_tool0')
    tool_link_R = pp.link_from_name(robot, 'right_ur_arm_tool0')

    # FK at every waypoint to derive the bar-pose path. Same convention as
    # the constrained planner (bar pose = world_from_left * tool0_from_bar).
    pose_path = []
    with pp.WorldSaver():
        for i in range(n):
            pp.set_joint_positions(
                robot, arm_joints,
                np.concatenate([left_arr[i], right_arr[i]]),
            )
            wt0L = pp.get_link_pose(robot, tool_link_L)
            pose_path.append(pp.multiply(wt0L, left_tool0_from_bar))
        # FK the goal once more to derive grasp_bar_from_right.
        pp.set_joint_positions(
            robot, arm_joints, np.concatenate([left_arr[-1], right_arr[-1]]),
        )
        wt0R_goal = pp.get_link_pose(robot, tool_link_R)
    grasp_bar_from_right = pp.multiply(pp.invert(pose_path[-1]), wt0R_goal)

    # Match the constrained planner's obstacle filter as closely as we can
    # without re-running its name-pattern logic: every loaded rigid body
    # except the active bar and the EE ghost spheres.
    ghost_set = set()
    for slot in (husky.object.ee_list or []):
        gp = slot[0] if slot[0] is not None else (
            slot[1].child if slot[1] is not None else None)
        if gp is not None:
            ghost_set.add(gp)
    all_rb_puids = sorted({
        ids[0] for ids in (monitor.cfab.client.rigid_bodies_puids or {}).values()
        if ids
    })
    obstacles = [b for b in all_rb_puids
                 if b != monitor.active_bar_body and b not in ghost_set]

    monitor._bar_action_plan_ctx = {
        "stage": getattr(monitor, "constrained_planner_stage", 3),
        "grasp_bar_from_left": grasp_bar_from_left,
        "grasp_bar_from_right": grasp_bar_from_right,
        "obstacles_for_constrained": obstacles,
        "path_poses": pose_path,
        "position_res": None,
        "rotation_res": None,
    }
    monitor.constrained_pose_path = pose_path
    monitor.constrained_start_conf = np.concatenate([left_arr[0], right_arr[0]])
    monitor.constrained_goal_conf = np.concatenate([left_arr[-1], right_arr[-1]])

    print(f"[trajectory] loaded compas JointTrajectory: {n} waypoints from {path}")
    return True


def _run_traj_slider(monitor):
    """Add two PyBullet sliders to scrub the staging + constrained trajectories."""
    import time
    import pybullet
    from husky_assembly_teleop.utils import HUSKY_DUAL_UR5e_JOINT_NAMES

    left_names = HUSKY_DUAL_UR5e_JOINT_NAMES[0]
    right_names = HUSKY_DUAL_UR5e_JOINT_NAMES[1]
    client_id = monitor.cfab.client.client_id

    def _build_waypoint_states(traj):
        if traj is None or traj[0] is None or traj[1] is None:
            return []
        left_path = traj[0][0]
        right_path = traj[1][0]
        n = len(left_path)
        if n < 1 or n != len(right_path):
            return []
        states = []
        for i in range(n):
            wp = monitor.movement_start_state.copy()
            for j, name in enumerate(left_names):
                wp.robot_configuration[name] = float(left_path[i][j])
            for j, name in enumerate(right_names):
                wp.robot_configuration[name] = float(right_path[i][j])
            states.append(wp)
        return states

    staging_states = _build_waypoint_states(getattr(monitor, "staging_free_trajectory", None))
    constrained_states = _build_waypoint_states(getattr(monitor, "constrained_trajectory", None))

    ns = len(staging_states)
    nc = len(constrained_states)
    if ns == 0 and nc == 0:
        print("[gui] no trajectories to scrub.")
        return

    staging_slider = None
    constrained_slider = None
    if ns > 0:
        staging_slider = pybullet.addUserDebugParameter(
            f"Staging t (0..{ns-1})", 0.0, float(max(ns - 1, 0)), 0.0,
            physicsClientId=client_id,
        )
    if nc > 0:
        constrained_slider = pybullet.addUserDebugParameter(
            f"Constrained t (0..{nc-1})", 0.0, float(max(nc - 1, 0)), 0.0,
            physicsClientId=client_id,
        )
    print(f"\n[gui] '{monitor.current_movement.movement_id}' plan loaded: "
          f"staging={ns} wp, constrained={nc} wp. Drag the sliders on the "
          f"PyBullet panel to scrub. Ctrl+C in this terminal to exit.")

    try:
        last_staging = -1
        last_constrained = -1
        while True:
            if staging_slider is not None:
                t = pybullet.readUserDebugParameter(staging_slider, physicsClientId=client_id)
                idx = max(0, min(ns - 1, int(round(t))))
                if idx != last_staging:
                    monitor.cfab.planner.set_robot_cell_state(staging_states[idx])
                    last_staging = idx
            if constrained_slider is not None:
                t = pybullet.readUserDebugParameter(constrained_slider, physicsClientId=client_id)
                idx = max(0, min(nc - 1, int(round(t))))
                if idx != last_constrained:
                    monitor.cfab.planner.set_robot_cell_state(constrained_states[idx])
                    last_constrained = idx
            time.sleep(0.03)
    except KeyboardInterrupt:
        print("\n[gui] exiting trajectory scrubber.")


def _print_collision_setup(monitor):
    """Print, for BOTH the constrained-RRT plan and the free-staging plan, the
    collision-check setup: moving bodies, static obstacles checked (+ excluded),
    robot self-collision link pairs, and the body-pair checks. Mirrors what
    get_joint_collision_fn (constrained) and plan_transit_motion (staging) feed
    into pp.get_collision_fn — reuses the same SRDF parsing as path_validation.
    """
    import pybullet_planning as pp
    from husky_assembly_teleop.utils import HUSKY_DUAL_UR5e_JOINT_NAMES
    from husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.path_validation import (
        get_disabled_collisions_from_link_names,
    )
    from husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.run import (
        HUSKY_DUAL_URDF_PATH, HUSKY_DUAL_SRDF_PATH,
    )
    from husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.core import (
        STAGE3_GRASP_MASK_LINKS,
    )
    from compas_fab.robots import RobotSemantics
    from compas_robots import RobotModel

    husky = getattr(monitor, "_bar_action_husky", None)
    if husky is None:
        print("[collision] no monitor._bar_action_husky — bridge not run; skipping.")
        return
    robot = husky.object.robot
    arm_joints = (list(pp.joints_from_names(robot, HUSKY_DUAL_UR5e_JOINT_NAMES[0]))
                  + list(pp.joints_from_names(robot, HUSKY_DUAL_UR5e_JOINT_NAMES[1])))
    bar_body = monitor.active_bar_body
    ctx = getattr(monitor, "_bar_action_plan_ctx", None)

    # puid -> name
    puid_name = {}
    for n, ids in (monitor.cfab.client.rigid_bodies_puids or {}).items():
        for i in ids:
            puid_name[i] = n
    ghosts = []
    for k, slot in enumerate(husky.object.ee_list or []):
        gp = slot[0] if slot[0] is not None else (slot[1].child if slot[1] is not None else None)
        if gp is not None:
            ghosts.append(gp)
            puid_name[gp] = f"ghost_ee_{'L' if k == 0 else 'R'}"
    puid_name[robot] = pp.get_body_name(robot) or f"robot#{robot}"
    nm = lambda p: f"{puid_name.get(p, '?')}({p})" if p is not None else "None"

    # SRDF-disabled self-collision pairs (constrained plan + path_validation use these)
    rmodel = RobotModel.from_urdf_file(HUSKY_DUAL_URDF_PATH)
    sem = RobotSemantics.from_srdf_file(HUSKY_DUAL_SRDF_PATH, rmodel)
    srdf_disabled = get_disabled_collisions_from_link_names(robot, sem.disabled_collisions)

    def _self_pairs(disabled):
        pairs = pp.get_self_link_pairs(robot, arm_joints, disabled_collisions=disabled)
        return [(pp.get_link_name(robot, a), pp.get_link_name(robot, b)) for a, b in pairs]

    all_rb_puids = sorted({ids[0] for ids in (monitor.cfab.client.rigid_bodies_puids or {}).values() if ids})
    print("\n=== Collision-check setup ===")
    print(f"world: robot={nm(robot)} ({len(arm_joints)} arm joints); bar={nm(bar_body)}; "
          f"ghost_ee=[{', '.join(nm(g) for g in ghosts)}]; {len(all_rb_puids)} rigid bodies total")

    # ---- CONSTRAINED ----
    print("\n[CONSTRAINED]  plan_constrained_dual_arm -> get_joint_collision_fn -> pp.get_collision_fn(self_collisions=True, max_distance=0)")
    if ctx is not None:
        cobs = list(ctx.get("obstacles_for_constrained") or [])
    else:
        cobs = []
        print("  (monitor._bar_action_plan_ctx unset — planner returned before the RRT; constrained obstacle list unknown)")
    excluded = sorted(set(all_rb_puids) - set(cobs) - {bar_body})
    print(f"  moving bodies: {nm(robot)} + {nm(bar_body)} [attachment @ left_ur_arm_tool0]")
    print(f"  static obstacles CHECKED ({len(cobs)}): {', '.join(nm(b) for b in cobs) or '(none)'}")
    if excluded:
        print(f"  static obstacles EXCLUDED ({len(excluded)}): {', '.join(nm(b) for b in excluded)}"
              f"  [reasons: bar touches it within 5mm at goal / name matches b\\d+(_0|_joint_N) / active-bar extra]")
    cpairs = _self_pairs(srdf_disabled)
    print(f"  robot self-collision link pairs ({len(cpairs)}; SRDF-disabled pairs already removed):")
    for a, b in cpairs:
        print(f"    {a} <-> {b}")
    print(f"  body-pair checks: {nm(robot)} <-> each of {len(cobs)} obstacle(s); "
          f"{nm(bar_body)} <-> each of {len(cobs)} obstacle(s); {nm(bar_body)} <-> {nm(robot)} "
          f"EXCEPT bar <-> {{{', '.join(STAGE3_GRASP_MASK_LINKS)}}}")

    # ---- FREE STAGING ----
    print("\n[FREE STAGING]  plan_free_dual_arm -> plan_transit_motion -> pp.get_collision_fn(self_collisions=1, max_distance=0)")
    sobs = sorted(b for b in all_rb_puids if b != bar_body)
    gp_l = ghosts[0] if len(ghosts) > 0 else None
    gp_r = ghosts[1] if len(ghosts) > 1 else None
    print(f"  moving bodies: {nm(robot)} + {nm(gp_l)} [@ left_ur_arm_tool0] + {nm(gp_r)} [@ right_ur_arm_tool0]")
    print(f"  static obstacles CHECKED ({len(sobs)}): {', '.join(nm(b) for b in sobs) or '(none)'}"
          f"  [= ALL rigid bodies except {nm(bar_body)}; the staging plan does NOT apply the constrained plan's design-study / expected-contact filter]")
    spairs = _self_pairs(set())  # plan_transit_motion: disabled_collisions = scene.get('disabled_collisions') -> {} in headless
    print(f"  robot self-collision link pairs ({len(spairs)}; NO SRDF disabled set — empty disabled_collisions, only auto-excluded adjacent links):")
    for a, b in spairs:
        print(f"    {a} <-> {b}")
    print(f"  body-pair checks: {nm(robot)} <-> each of {len(sobs)} obstacle(s); "
          f"{nm(gp_l)} <-> each of {len(sobs)} obstacle(s); {nm(gp_r)} <-> each of {len(sobs)} obstacle(s); "
          f"{nm(robot)} <-> {nm(gp_l)} EXCEPT robot:left_ur_arm_wrist_3_link; "
          f"{nm(robot)} <-> {nm(gp_r)} EXCEPT robot:right_ur_arm_wrist_3_link; {nm(gp_l)} <-> {nm(gp_r)}")
    print("=== end collision-check setup ===\n")


def _run_path_validation(monitor):
    """Validate the CONSTRAINED dual-arm trajectory only (the staging free
    motion is intentionally NOT validated — there the bar isn't held so the
    EE relative transform is meaningless). Reuses
    husky_assembly_tamp.path_validation.validate_stage_trajectory (the same
    call run_stage_trial makes in dual_arm_task_space_rrt/run.py). Writes the 6-panel
    drift/collision plot + a tiny markdown report next to it.

    Returns the validation dict, or None if there's nothing to validate.
    """
    import pybullet_planning as pp
    from husky_assembly_teleop.utils import HUSKY_DUAL_UR5e_JOINT_NAMES
    from husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.path_validation import (
        validate_stage_trajectory,
    )
    from husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.run import (
        HUSKY_DUAL_URDF_PATH, HUSKY_DUAL_SRDF_PATH,
        log_validation_summary,
    )
    from husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.core import (
        STAGE3_GRASP_MASK_LINKS, DEFAULT_USE_ANGLE_NORMALIZATION,
    )

    ctx = getattr(monitor, "_bar_action_plan_ctx", None)
    if ctx is None:
        print("[validate] no plan context on monitor — did plan_and_stage_constrained run?")
        return None
    traj_c = monitor.constrained_trajectory
    if not (traj_c and traj_c[0] is not None and traj_c[1] is not None):
        print("[validate] no constrained trajectory; nothing to validate.")
        return None

    robot = monitor._bar_action_husky.object.robot
    left_names = HUSKY_DUAL_UR5e_JOINT_NAMES[0]
    right_names = HUSKY_DUAL_UR5e_JOINT_NAMES[1]
    arm_joints = (list(pp.joints_from_names(robot, left_names))
                  + list(pp.joints_from_names(robot, right_names)))
    scene = {
        "robot": robot,
        "arm_joints": arm_joints,
        "tool_link_left": pp.link_from_name(robot, 'left_ur_arm_tool0'),
        "tool_link_right": pp.link_from_name(robot, 'right_ur_arm_tool0'),
        "bar_body": monitor.active_bar_body,
        "grasp_bar_from_left": ctx["grasp_bar_from_left"],
        "grasp_bar_from_right": ctx["grasp_bar_from_right"],
        "collision_obstacles": ctx["obstacles_for_constrained"],
        "bar_label": monitor.active_bar_name,
    }
    left_arr = np.asarray(traj_c[0][0], dtype=float)
    right_arr = np.asarray(traj_c[1][0], dtype=float)
    joint_path = [np.concatenate([left_arr[i], right_arr[i]])
                  for i in range(len(left_arr))]
    pose_path = ctx.get("path_poses") or getattr(monitor, "constrained_pose_path", None)

    reports_dir = os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "reports", "bar_action_validation"))
    print(f"[validate] CONSTRAINED motion only — validate_stage_trajectory("
          f"stage={ctx['stage']}, waypoints={len(joint_path)}, "
          f"use_angle_normalization={DEFAULT_USE_ANGLE_NORMALIZATION}) → {reports_dir}")
    validation = validate_stage_trajectory(
        stage=ctx["stage"],
        scene=scene,
        path=pose_path,
        joint_path=joint_path,
        original_joint_path=None,
        joint_path_source="planner",
        joint_path_reason=None,
        urdf_path=HUSKY_DUAL_URDF_PATH,
        srdf_path=HUSKY_DUAL_SRDF_PATH,
        grasp_mask_links=STAGE3_GRASP_MASK_LINKS,
        target_label=monitor.active_bar_name,
        position_res=ctx.get("position_res"),
        rotation_res=ctx.get("rotation_res"),
        # The constrained RRT uses angle normalization internally for IK, but
        # command-space validation must reject raw ±2π joint wraps because the
        # UR controller can interpret them as real high-speed moves.
        use_angle_normalization=DEFAULT_USE_ANGLE_NORMALIZATION,
        reports_dir=reports_dir,
    )
    log_validation_summary(validation)

    # Minimal markdown report next to the plot.
    mv_id = monitor.current_movement.movement_id if monitor.current_movement else "?"
    rep_path = os.path.join(reports_dir, "bar_action_validation_report.md")
    plot_rel = (os.path.basename(validation["plot_path"])
                if validation.get("plot_path") else "-")
    drift = validation.get("relative_transform_max_axis_angle_deg") or {}
    lines = [
        f"# BarAction path validation — {mv_id}",
        "",
        f"- bar_action: `{getattr(monitor.current_action, 'action_id', '?')}`",
        f"- movement: `{mv_id}`  (stage {ctx['stage']})",
        f"- waypoints: pose={validation.get('path_waypoints')}, "
        f"joint={validation.get('joint_path_waypoints')}, "
        f"dense={validation.get('dense_joint_validation_waypoints')}",
        f"- collision_free: **{validation.get('collision_free')}**",
        f"- joint_continuity_ok: **{validation.get('joint_continuity_ok')}** "
        f"(max dq={validation.get('joint_continuity_max_delta_rad')} rad, "
        f"thr={validation.get('joint_continuity_threshold_rad')} rad)",
        f"- relative_transform_ok: **{validation.get('relative_transform_ok')}** "
        f"(max pos={validation.get('relative_transform_max_translation_m')} m, "
        f"max axis drift xyz=[{drift.get('x')}, {drift.get('y')}, {drift.get('z')}] deg)",
        f"- collision_breakdown: `{validation.get('collision_breakdown')}`",
        f"- joint_path_reason: `{validation.get('joint_path_reason')}`",
        "",
        f"![validation plot]({plot_rel})",
        "",
        "## raw validation dict",
        "```json",
        json.dumps(validation, indent=2, default=str),
        "```",
    ]
    os.makedirs(reports_dir, exist_ok=True)
    with open(rep_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"[validate] plot:   {validation.get('plot_path')}")
    print(f"[validate] report: {rep_path}")
    return validation


def main(bar_action: str = DEFAULT_BAR_ACTION,
         movement: str = DEFAULT_MOVEMENT,
         problem: str = DEFAULT_PROBLEM,
         use_gui: bool = False,
         verbose: bool = False,
         max_time: float = None,
         max_attempts: int = None,
         save_plan: str = None,
         replay: str = None,
         trajectory: str = None,
         random_seed: int = None,
         draw_rrt: bool = False,
         validate: bool = False,
         position_res: float = None,
         rotation_res: float = None,
         free_joint_resolution: float = None,
         show_collision_setup: bool = False,
         stage: int = None) -> int:
    print(f"=== headless_live_monitor_test: problem={problem!r} "
          f"bar_action={bar_action!r} movement={movement!r} ===")

    _patch_validation_problem(problem)

    rc = 0
    monitor = None
    try:
        monitor = _bypass_init_monitor()
        if stage is not None:
            # 1=pose-only RRT (no IK, no collision); 2=pose RRT + IK in
            # extend, no robot collision; 3=pose RRT + IK + joint-space
            # robot collision (full). plan_constrained_dual_arm reads
            # monitor.constrained_planner_stage.
            if stage not in (1, 2, 3):
                print(f"FAIL: --stage must be 1, 2 or 3; got {stage}")
                return 1
            monitor.constrained_planner_stage = stage
            print(f"[stage] constrained_planner_stage = {stage}")

        # Pre-create the cfab session in GUI mode if --gui was requested,
        # BEFORE calling load_bar_action (which would otherwise create a
        # direct-mode session). enable_debug_gui=True turns on
        # pybullet's COV_ENABLE_GUI so the sidebar/parameter panel (for
        # the 'Path t' slider) renders.
        if use_gui:
            from husky_assembly_teleop.cfab_session import CfabSession
            print("[gui] opening cfab PyBullet window (BarAction scene)...")
            monitor.cfab = CfabSession(
                problem, connection_type="gui", enable_debug_gui=True,
            )

        # Step 1: populate available BarActions (simulate slider) + select.
        monitor.available_robot_cell_states = monitor._load_available_bar_actions()
        if not monitor.available_robot_cell_states:
            print("FAIL: no BarAction files available")
            return 1
        if bar_action not in monitor.available_robot_cell_states:
            print(f"FAIL: {bar_action!r} not in available BarActions; have "
                  f"{monitor.available_robot_cell_states[:8]}"
                  f"{'...' if len(monitor.available_robot_cell_states) > 8 else ''}")
            return 1
        monitor.selected_state_index = monitor.available_robot_cell_states.index(bar_action)
        print(f"selected BarAction {monitor.selected_state_index}: {bar_action}; "
              f"movement={movement}")

        # Step 2: load the BarAction movement.
        print("\n--- simulating 'Load BarAction' click ---")
        ok = monitor.load_bar_action(movement=movement)
        if not ok:
            print("FAIL: load_bar_action returned False")
            return 1

        # NOTE: no pp husky to re-pose; the cfab client already has the
        # robot at start_state.robot_configuration + robot_base_frame
        # (via planner.set_robot_cell_state inside load_bar_action).

        # Bridge cfab → pp: reuse cfab's already-loaded husky + rigid bodies
        # as pp body ids so plan_and_stage_constrained can drive the same
        # PyBullet world via pp.* utilities.
        import pybullet_planning as pp_module
        pp_module.CLIENT = monitor.cfab.client.client_id
        # Register cfab's PyBullet connection with pp's CLIENTS registry
        # (pp normally populates it via pp.connect; cfab created the client
        # directly so we inject the entry — None = no GUI lock, True = GUI).
        pp_module.CLIENTS[monitor.cfab.client.client_id] = (
            True if use_gui else None
        )

        robot_puid = monitor.cfab.client.robot_puid
        left_tool_link = pp_module.link_from_name(robot_puid, 'left_ur_arm_tool0')
        right_tool_link = pp_module.link_from_name(robot_puid, 'right_ur_arm_tool0')
        identity_grasp = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

        # Create two tiny invisible "ghost" sphere bodies as EE attachment
        # children. plan_transit_motion (the staging planner) builds
        # extra_disabled_collisions from (wrist_3_link, attachment.child) and
        # routes attachments through pp.get_collision_fn — so the child MUST
        # be a distinct body, not the husky itself. The live monitor uses
        # the real gripper proxy meshes; here we substitute a far-away 1mm
        # sphere. They're excluded from static_obstacles below.
        import pybullet as _pb
        _cid = monitor.cfab.client.client_id
        def _make_ee_ghost():
            col = _pb.createCollisionShape(_pb.GEOM_SPHERE, radius=0.001,
                                           physicsClientId=_cid)
            return _pb.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                                       basePosition=[0.0, 0.0, -100.0],
                                       physicsClientId=_cid)
        ghost_L = _make_ee_ghost()
        ghost_R = _make_ee_ghost()
        husky_stub = SimpleNamespace(object=SimpleNamespace(
            robot=robot_puid,
            ee_list=[
                (ghost_L, pp_module.Attachment(robot_puid, left_tool_link,
                                               identity_grasp, ghost_L)),
                (ghost_R, pp_module.Attachment(robot_puid, right_tool_link,
                                               identity_grasp, ghost_R)),
            ],
        ))
        _ghost_set = {ghost_L, ghost_R}
        monitor.huskies = [husky_stub]
        # monitor._bar_action_husky = husky_stub
        monitor.selected_robot_id = 0

        puids = monitor.cfab.client.rigid_bodies_puids
        monitor.active_bar_body = (puids.get(monitor.active_bar_name) or [None])[0]
        monitor.static_obstacles = {
            n: ids[0] for n, ids in puids.items()
            if ids and n != monitor.active_bar_name and ids[0] not in _ghost_set
        }
        if HEADLESS_DISABLE_ENVIRONMENT_COLLISIONS:
            n_env = len(monitor.static_obstacles)
            monitor.static_obstacles = {}
            print(f"[collision] headless env collisions disabled: skipped {n_env} static bodies; "
                  "robot self-collision and attached tool/bar-vs-robot checks remain.")
        monitor.active_extra_bodies = []
        monitor.bar_from_extra = []
        monitor.active_bar_aabb_dims = monitor.get_active_bar_aabb_dims()
        # Staging-plan seed: real UR5e HOME, perturbed until collision-free
        # (HOME itself sits ~2mm inside a self-collision margin on the
        # dual-arm husky URDF). Headless-only — the live monitor's
        # current_conf is always a feasible physical robot pose.
        from husky_assembly_teleop.husky_robot import UR5e_HOME_STATE
        home_dual = np.concatenate([UR5e_HOME_STATE, UR5e_HOME_STATE])
        monitor.bar_action_staging_seed_conf = _sample_feasible_staging_seed(
            monitor, home_dual)

        # Step 2 assertions
        ok = True
        print(f"\n--- post-load assertions ---")
        if monitor.cfab is None:
            print("FAIL: monitor.cfab not initialized")
            ok = False
        else:
            n_bodies = len(monitor.cfab.client.rigid_bodies_puids)
            print(f"PASS: cfab session connected; rigid_bodies={n_bodies}")
        if monitor.movement_type is None:
            print("FAIL: movement_type not set")
            ok = False
        else:
            print(f"PASS: movement_type={monitor.movement_type}")
        if monitor.active_bar_name is None:
            print("FAIL: active_bar_name not set")
            ok = False
        else:
            print(f"PASS: active_bar_name={monitor.active_bar_name!r}")
        if monitor.target_ee_frames is None:
            if monitor.movement_type == "free":
                print("INFO: target_ee_frames None (free movement, expected)")
            else:
                print("WARN: target_ee_frames is None (unexpected for "
                      f"{monitor.movement_type})")
        else:
            lp = monitor.target_ee_frames["left"].point
            rp = monitor.target_ee_frames["right"].point
            print(f"PASS: target_ee_frames "
                  f"L=({lp[0]:.3f},{lp[1]:.3f},{lp[2]:.3f}) "
                  f"R=({rp[0]:.3f},{rp[1]:.3f},{rp[2]:.3f})")
        aabb = monitor.get_active_bar_aabb_dims()
        if aabb is not None:
            print(f"PASS: active_bar_aabb_dims (from mesh) = "
                  f"({aabb[0]:.3f}, {aabb[1]:.3f}, {aabb[2]:.3f}) m")
        else:
            print("WARN: active_bar_aabb_dims could not be computed from mesh")

        if not ok:
            return 1

        # Step 3: BarAction-driven planning entry (constrained dual-arm) —
        # OR skip planning entirely if --replay / --trajectory was passed.
        if trajectory:
            traj_path = trajectory
            if not os.path.isabs(traj_path):
                from husky_assembly_teleop import DESIGN_DATA_DIRECTORY
                traj_path = os.path.join(
                    DESIGN_DATA_DIRECTORY, problem, "Trajectories", traj_path,
                )
            print(f"\n--- trajectory mode: loading compas JointTrajectory "
                  f"from {traj_path} ---")
            plan_ok = _load_compas_trajectory(monitor, traj_path)
            # Auto-enable validation in trajectory mode unless caller already
            # opted in.
            if plan_ok and not validate:
                print("[trajectory] --validate auto-enabled in trajectory mode.")
                validate = True
        elif replay:
            print(f"\n--- replay mode: loading plan from {replay} ---")
            plan_ok = _load_plan(monitor, replay)
        else:
            print("\n--- simulating 'Plan & Stage Constrained' click ---")
            from husky_assembly_teleop import husky_world
            import pybullet_planning as pp_module
            plan_kwargs = {}
            if max_time is not None:
                plan_kwargs["max_time"] = max_time
            if max_attempts is not None:
                plan_kwargs["max_attempts"] = max_attempts
            if random_seed is not None:
                plan_kwargs["random_seed"] = random_seed
            if position_res is not None:
                plan_kwargs["position_res"] = position_res
            if rotation_res is not None:
                plan_kwargs["rotation_res"] = rotation_res
            if free_joint_resolution is not None:
                plan_kwargs["free_joint_resolution"] = free_joint_resolution
            if draw_rrt:
                if not use_gui:
                    print("[draw-rrt] WARN: --draw-rrt needs --gui; ignoring.")
                else:
                    plan_kwargs["use_draw"] = True
                    print("[draw-rrt] drawing RRT tree edges live (slower); "
                          "best with --max-attempts 1 + --random-seed N.")
            # Disable rendering while planning — the RRT/IK loops sample
            # thousands of configs; rendering each one dominates the time
            # budget in --gui mode. set_renderer() has its own has_gui()
            # guard, so this is a no-op without a GUI window. (Don't use
            # pp.LockRenderer here — its restore() asserts CLIENTS[client]
            # is not None, which fails for an externally-created client.)
            _lock_render = True # not (draw_rrt and use_gui)
            if _lock_render:
                pp_module.set_renderer(False)
            try:
                husky_world.plan_and_stage_constrained(monitor, **plan_kwargs)
            finally:
                if _lock_render:
                    pp_module.set_renderer(True)
            plan_ok = bool(monitor.constrained_trajectory
                           and monitor.constrained_trajectory[0] is not None
                           and monitor.constrained_trajectory[1] is not None)

        traj_c = monitor.constrained_trajectory
        traj_s = monitor.staging_free_trajectory

        print("\n=== Result ===")
        if plan_ok:
            nc = len(traj_c[0][0])
            ns = (len(traj_s[0][0])
                  if traj_s and traj_s[0] is not None and traj_s[1] is not None else 0)
            print(f"PASS: plan for {monitor.current_movement.movement_id}: "
                  f"staging={ns} wp, constrained={nc} wp.")
            rc = 0
            if save_plan and not replay and not trajectory:
                _save_plan(monitor, save_plan)
        else:
            print("FAIL: see error logs above (no constrained trajectory).")
            rc = 1

        # Optional: dump the collision-check setup (which body/link pairs each
        # planner checks) for the constrained + free-staging plans.
        if show_collision_setup and not replay and not trajectory:
            try:
                _print_collision_setup(monitor)
            except Exception as e:
                print(f"[collision] ERROR: {e}")

        # Optional: validate the constrained trajectory (drift / continuity /
        # collisions) + write the plot & report. Reuses
        # path_validation.validate_stage_trajectory. (Skipped on --replay,
        # which has no plan context.)
        if validate and plan_ok and not replay:  # trajectory-mode is allowed (ctx is reconstructed)
            try:
                import pybullet_planning as _pp
                _pp.set_renderer(False)
                try:
                    _run_path_validation(monitor)
                finally:
                    _pp.set_renderer(True)
            except Exception as e:
                print(f"[validate] ERROR: {e}")

        if use_gui:
            # Interactive scrubbing on the cfab GUI window. Two PyBullet
            # sliders (staging + constrained) interpolate the planned
            # trajectories and re-pose the robot via
            # planner.set_robot_cell_state (which also moves the attached
            # bar / tools rigidly with the link).
            if plan_ok and monitor.cfab is not None:
                _run_traj_slider(monitor)
            else:
                try:
                    input("\n[gui] press Enter to close the PyBullet window...")
                except (EOFError, KeyboardInterrupt):
                    pass

        return rc
    finally:
        # Clean up cfab session if we got that far. (No pp connection to
        # disconnect.)
        try:
            if monitor is not None and getattr(monitor, 'cfab', None) is not None:
                monitor.cfab.close()
        except Exception:
            pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--bar-action", type=str, default=DEFAULT_BAR_ACTION,
                        help=f"BarAction *.json filename under "
                             f"<DESIGN_DATA_DIRECTORY>/<problem>/BarActions/. "
                             f"Default: {DEFAULT_BAR_ACTION!r}.")
    parser.add_argument("--movement", type=str, default=DEFAULT_MOVEMENT,
                        help=f"Movement id substring (e.g. 'M1') or integer "
                             f"index. Default: {DEFAULT_MOVEMENT!r}.")
    parser.add_argument("--problem", type=str, default=DEFAULT_PROBLEM,
                        help=f"DESIGN_PROBLEM_NAME directory. "
                             f"Default: {DEFAULT_PROBLEM!r}.")
    parser.add_argument("--gui", action="store_true",
                        help="Open the pp-side PyBullet GUI.")
    parser.add_argument("--verbose", action="store_true",
                        help="Verbose collision check output.")
    parser.add_argument("--max-time", type=float, default=None,
                        help="Per-attempt time budget (s) for the constrained "
                             "planner. Default: planner default (30).")
    parser.add_argument("--max-attempts", type=int, default=None,
                        help="Constrained planner outer attempts. Default: "
                             "planner default (5). Bump (e.g. 15) to ride out "
                             "RRT randomness for hard scenes.")
    parser.add_argument("--save-plan", type=str, default=None,
                        help="On planner success, save the staging + "
                             "constrained trajectories as a JSON for later "
                             "replay with --replay.")
    parser.add_argument("--replay", type=str, default=None,
                        help="Replay a previously saved plan JSON instead of "
                             "running the planner. Still loads the BarAction "
                             "to reconstruct the scene.")
    parser.add_argument("--trajectory", type=str, default=None,
                        help="Skip planning and load a compas JointTrajectory "
                             "JSON (as written by the husky monitor's 'Export "
                             "Constrained Dual-Arm Traj' button). Absolute path "
                             "or bare filename (resolved under "
                             "<problem>/Trajectories/). Reconstructs the BarAction "
                             "scene, populates monitor.constrained_trajectory + "
                             "_bar_action_plan_ctx, auto-enables --validate, and "
                             "(with --gui) opens the trajectory scrubber.")
    parser.add_argument("--random-seed", type=int, default=None,
                        help="Pin the constrained RRT's RNG for a reproducible "
                             "run. Default: fresh entropy each run.")
    parser.add_argument("--draw-rrt", action="store_true",
                        help="Draw the constrained pose-RRT tree edges in the "
                             "cfab GUI window (needs --gui). Best paired with "
                             "--max-attempts 1 --random-seed N.")
    parser.add_argument("--validate", action="store_true",
                        help="After a successful plan, run "
                             "path_validation.validate_stage_trajectory on the "
                             "CONSTRAINED trajectory (not staging) and write the "
                             "drift/collision plot + a markdown report under "
                             "scripts/../reports/bar_action_validation/.")
    parser.add_argument("--position-res", type=float, default=None,
                        help="Constrained-RRT translational step resolution (m) "
                             "for extend_toward. Default: plan_constrained_dual_arm "
                             "default (0.01). Smaller = less per-step IK drift, "
                             "slower.")
    parser.add_argument("--rotation-res", type=float, default=None,
                        help="Constrained-RRT rotational step resolution (rad) "
                             "for extend_toward. Default: plan_constrained_dual_arm "
                             "default (0.025).")
    parser.add_argument("--free-joint-resolution", type=float, default=None,
                        help="Free-staging BiRRT joint interpolation resolution "
                             "(rad). Default: husky_world.FREE_JOINT_RESOLUTION.")
    parser.add_argument("--show-collision-setup", action="store_true",
                        help="After planning, print the collision-check setup "
                             "(moving bodies, static obstacles checked/excluded, "
                             "robot self-collision link pairs, body-pair checks) "
                             "for BOTH the constrained and free-staging plans.")
    parser.add_argument("--stage", type=int, default=None, choices=(1, 2, 3),
                        help="Constrained-RRT stage (dual_arm_task_space_rrt.core.plan_pose_rrt): "
                             "1=pose-only RRT (no IK, no collision); "
                             "2=pose RRT + IK in extend, NO robot collision; "
                             "3=pose RRT + IK + robot collision (full). "
                             "Default: 3.")
    # Legacy flags accepted (ignored) so old command lines don't fail.
    parser.add_argument("--state", type=str, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--diagnose", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.state is not None:
        print(f"NOTE: --state {args.state!r} is legacy; use --bar-action instead "
              f"(ignored).")
    sys.exit(main(bar_action=args.bar_action, movement=args.movement,
                  problem=args.problem, use_gui=args.gui, verbose=args.verbose,
                  max_time=args.max_time, max_attempts=args.max_attempts,
                  save_plan=args.save_plan, replay=args.replay,
                  trajectory=args.trajectory,
                  random_seed=args.random_seed, draw_rrt=args.draw_rrt,
                  validate=args.validate,
                  position_res=args.position_res, rotation_res=args.rotation_res,
                  free_joint_resolution=args.free_joint_resolution,
                  show_collision_setup=args.show_collision_setup,
                  stage=args.stage))
