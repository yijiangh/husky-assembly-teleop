"""Headless integration test that drives the *real* HuskyMonitor methods.

Unlike `headless_constrained_monitor.py` (which uses a SimpleNamespace mock),
this script bypasses HuskyMonitor.__init__ via `object.__new__`, populates the
attributes the methods read, then invokes the actual methods that the GUI
buttons would invoke:

  1. monitor.load_board_validation_state()    <- "Load Robot Cell State" button
  2. husky_world.plan_and_stage_constrained() <- "Plan & Stage Constrained" button

This catches integration regressions in the loading flow (e.g., the
active-bar identification that the antenna dataset broke) without needing
ROS, GUI, or manual button clicks.

Usage (with ros2_ws venv active):
  python src/husky-assembly-teleop/scripts/headless_live_monitor_test.py [--target D1] [--stage 3]
"""
from __future__ import annotations

import argparse
import os
import sys
from types import SimpleNamespace
from typing import Optional

import numpy as np
import pybullet_planning as pp


URDF = (
    "/home/yijiangh/Code/ros2_ws/src/husky-assembly-teleop/data/husky_urdf/"
    "mt_husky_dual_ur5_e_moveit_config/urdf/"
    "husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf"
)

ANTENNA_PROBLEM = "250929_New_Antenna_with_GH_RH_Packed"


class StubLogger:
    def warn(self, msg):  print(f"[WARN] {msg}")
    def info(self, msg):  print(f"[INFO] {msg}")
    def error(self, msg): print(f"[ERROR] {msg}")


def _patch_validation_problem(antenna_problem: str) -> None:
    """Monkey-patch VALIDATION_PROBLEM_NAME inside husky_monitor's module
    scope so methods that close over it see the antenna dataset."""
    from husky_assembly_teleop import husky_monitor as hm
    hm.VALIDATION_PROBLEM_NAME = antenna_problem


def _build_husky(robot_body):
    """Construct a husky-shaped object with .object.robot and .object.ee_list."""
    from husky_assembly_tamp.motion_planner.stage1.minimal_rrt import (
        TOOL_LINK_LEFT, TOOL_LINK_RIGHT,
    )
    tool0_L = pp.link_from_name(robot_body, TOOL_LINK_LEFT)
    tool0_R = pp.link_from_name(robot_body, TOOL_LINK_RIGHT)
    pose_L = pp.get_link_pose(robot_body, tool0_L)
    pose_R = pp.get_link_pose(robot_body, tool0_R)
    gL = pp.create_box(0.05, 0.05, 0.05, color=(0.2, 0.2, 0.8, 1.0), mass=pp.STATIC_MASS)
    gR = pp.create_box(0.05, 0.05, 0.05, color=(0.8, 0.2, 0.2, 1.0), mass=pp.STATIC_MASS)
    pp.set_pose(gL, pose_L)
    pp.set_pose(gR, pose_R)
    aL = pp.create_attachment(robot_body, tool0_L, gL)
    aR = pp.create_attachment(robot_body, tool0_R, gR)
    husky_object = SimpleNamespace(
        robot=robot_body,
        ee_list=[(gL, aL), (gR, aR)],
        dual_arm=True,
    )
    return SimpleNamespace(object=husky_object, interface=SimpleNamespace())


def _bypass_init_monitor(robot_body, stage: int = 3):
    """Construct a HuskyMonitor without running its real __init__.

    We use object.__new__ to skip ROS Node init, mocap, world.init, etc.,
    then populate exactly the attributes the methods we'll call read.
    """
    from husky_assembly_teleop.husky_monitor import HuskyMonitor

    monitor = object.__new__(HuskyMonitor)

    # State the loading flow reads
    monitor.huskies = [_build_husky(robot_body)]
    monitor.selected_robot_id = 0
    monitor.static_obstacles = {}
    monitor.active_bar_body = None
    monitor.active_bar_aabb_dims = None
    monitor.active_bar_name = None
    monitor.constrained_planner_stage = stage
    monitor.staging_free_trajectory = [None, None]
    monitor.constrained_trajectory = [None, None]
    monitor.constrained_display_mode = 0
    monitor.grasp_targets_override = None

    # Cache for _load_robot_cell
    monitor._robot_cell_cache = None
    monitor._robot_cell_cache_path = None

    # Cell-state slider state
    monitor.available_robot_cell_states = []
    monitor.selected_state_index = 0
    monitor.available_joint_trajectories = []
    monitor.selected_trajectory_index = 0

    # Goal interface state
    monitor.goal_arm_pose = [np.zeros(6), np.zeros(6)]
    monitor.goal_base_pose = (np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]))
    monitor.show_goal_state = False
    monitor.trajectory_time = 20.0
    monitor.selected_arm_index = 0

    # Methods that get called as side-effects — stub them out
    captured = {"set_arm_trajectory": [], "show_traj_state_calls": 0,
                "show_goal_state_calls": 0, "reset_ui_calls": 0}

    def _set_arm_trajectory(traj, index=0):
        captured["set_arm_trajectory"].append((index, traj))
    def _set_to_show_traj_state():
        captured["show_traj_state_calls"] += 1
    def _set_to_show_goal_state():
        captured["show_goal_state_calls"] += 1
    def _reset_ui(target_conf=None):
        captured["reset_ui_calls"] += 1
    def _refresh_constrained_displayed_trajectory():
        src = (monitor.constrained_trajectory if monitor.constrained_display_mode == 1
               else monitor.staging_free_trajectory)
        if src[0] is not None and src[1] is not None:
            monitor.set_arm_trajectory(src[0], index=0)
            monitor.set_arm_trajectory(src[1], index=1)
            monitor.set_to_show_traj_state()

    monitor.set_arm_trajectory = _set_arm_trajectory
    monitor.set_to_show_traj_state = _set_to_show_traj_state
    monitor.set_to_show_goal_state = _set_to_show_goal_state
    monitor.reset_ui = _reset_ui
    monitor._refresh_constrained_displayed_trajectory = _refresh_constrained_displayed_trajectory

    # ROS Node.get_logger() stub
    _logger = StubLogger()
    monitor.get_logger = lambda: _logger

    # Capture handle for assertions
    monitor._captured = captured
    return monitor


def _diagnose_endpoint_collisions(monitor, robot, arm_joints, start_conf,
                                  goal_conf, world_from_bar_start,
                                  world_from_bar_goal):
    """Print all bodies in close-contact with the active bar at start and goal poses.

    Useful when plan_pose_rrt reports 'start_in_collision' or 'goal_in_collision'
    to identify exactly which obstacle is the offender. Probes within 5cm so we
    see near-misses too.
    """
    import pybullet as pb
    name_from_body = {body: name for name, body in monitor.static_obstacles.items()}
    name_from_body[robot] = "ROBOT"
    name_from_body[monitor.active_bar_body] = monitor.active_bar_name

    for label, conf, bar_pose in [
        ("START", start_conf, world_from_bar_start),
        ("GOAL", goal_conf, world_from_bar_goal),
    ]:
        with pp.WorldSaver():
            pp.set_joint_positions(robot, arm_joints, conf)
            pp.set_pose(monitor.active_bar_body, bar_pose)
            pb.performCollisionDetection()
            print(f"  [{label}] bar+robot contacts within 5cm:")
            shown = 0
            for body in pp.get_bodies():
                if body == monitor.active_bar_body:
                    continue
                # bar vs body
                pts = pb.getClosestPoints(monitor.active_bar_body, body, distance=0.05)
                if pts:
                    name = name_from_body.get(body, f"<body {body}>")
                    depths = sorted([round(pt[8], 4) for pt in pts])[:3]
                    flag = " *INSIDE*" if depths[0] < 0 else ""
                    print(f"    bar  vs {name:24s}: gap/penetration {depths}{flag}")
                    shown += 1
                # robot vs body (skip bar — already covered)
                if body == robot:
                    continue
                rpts = pb.getClosestPoints(robot, body, distance=0.05)
                if rpts:
                    rdepths = sorted([round(pt[8], 4) for pt in rpts])[:3]
                    rflag = " *INSIDE*" if rdepths[0] < 0 else ""
                    if rdepths[0] < 0.001:  # only show actual penetration / tight contact
                        name = name_from_body.get(body, f"<body {body}>")
                        print(f"    robot vs {name:24s}: gap/penetration {rdepths}{rflag}")
                        shown += 1
            if shown == 0:
                print("    (none)")


def main(target: str = "D1", stage: int = 3, max_time: float = 10.0,
         max_attempts: int = 5, diagnose: bool = False,
         antenna_problem: str = ANTENNA_PROBLEM) -> int:
    print(f"=== headless_live_monitor_test: target={target}, stage={stage} ===")

    _patch_validation_problem(antenna_problem)

    pp.connect(use_gui=False)
    try:
        robot_body = pp.load_pybullet(URDF, fixed_base=True)
        monitor = _bypass_init_monitor(robot_body, stage=stage)

        # Step 1: simulate "available state slider" populating + select target.
        # _load_available_robot_cell_states is a real instance method.
        monitor.available_robot_cell_states = monitor._load_available_robot_cell_states()
        if not monitor.available_robot_cell_states:
            print("FAIL: no robot cell state files available")
            return 1
        target_filename = f"{target}_RobotCellState.json"
        if target_filename not in monitor.available_robot_cell_states:
            print(f"FAIL: {target_filename} not in available states; have {monitor.available_robot_cell_states[:5]}...")
            return 1
        monitor.selected_state_index = monitor.available_robot_cell_states.index(target_filename)
        print(f"selected state index {monitor.selected_state_index}: {target_filename}")

        # Step 2: simulate clicking 'Load Robot Cell State' button — call the real method.
        print("\n--- simulating 'Load Robot Cell State' click ---")
        monitor.load_board_validation_state()

        # Note: `load_board_validation_state` now auto-loads the
        # GraspTargets JSON alongside the cell state and applies the
        # override (see HuskyMonitor._load_grasp_targets_if_available).
        # No additional setup needed in the harness.

        # Step 2 assertions
        ok = True
        if monitor.active_bar_body is None:
            print(f"FAIL: active_bar_body is None after load (expected non-None for target={target!r})")
            ok = False
        else:
            print(f"PASS: active_bar_body set: name={monitor.active_bar_name!r} body={monitor.active_bar_body}")
        if not np.any(monitor.goal_arm_pose[0]):
            print("FAIL: goal_arm_pose[0] is all zeros (left arm not loaded)")
            ok = False
        else:
            print(f"PASS: goal_arm_pose loaded: L={np.round(monitor.goal_arm_pose[0], 3)}")
            print(f"                              R={np.round(monitor.goal_arm_pose[1], 3)}")
        if monitor._captured["reset_ui_calls"] == 0:
            print("FAIL: reset_ui was not called during load")
            ok = False
        else:
            print(f"PASS: reset_ui called {monitor._captured['reset_ui_calls']}x")
        if not ok:
            return 1

        # Step 3: simulate clicking 'Plan & Stage Constrained' button.
        print("\n--- simulating 'Plan & Stage Constrained' click ---")
        from husky_assembly_teleop import husky_world

        if diagnose:
            # Pre-flight: show what the planner sees at start_conf (derived) and goal_conf.
            from husky_assembly_tamp.motion_planner.api import (
                derive_grasps_from_state, derive_constrained_start,
            )
            from husky_assembly_tamp.motion_planner.stage1.minimal_rrt import (
                HUSKY_DUAL_ARM_JOINT_NAMES,
            )
            arm_joints = pp.joints_from_names(robot_body, HUSKY_DUAL_ARM_JOINT_NAMES)
            tool_link_L = pp.link_from_name(robot_body, "left_ur_arm_tool0")
            tool_link_R = pp.link_from_name(robot_body, "right_ur_arm_tool0")
            wfb_goal = pp.get_pose(monitor.active_bar_body)
            gL, gR = derive_grasps_from_state(
                robot_body, arm_joints, tool_link_L, tool_link_R,
                np.concatenate([monitor.goal_arm_pose[0], monitor.goal_arm_pose[1]]), wfb_goal,
            )
            wfb_start, start_c = derive_constrained_start(
                robot_body, arm_joints, tool_link_L, tool_link_R, gL, gR,
                wfb_goal,
                seed_conf=np.concatenate([monitor.goal_arm_pose[0], monitor.goal_arm_pose[1]]),
            )
            if start_c is not None:
                print("\n--- collision diagnostics (pre-plan) ---")
                _diagnose_endpoint_collisions(
                    monitor, robot_body, arm_joints,
                    start_c, np.concatenate([monitor.goal_arm_pose[0], monitor.goal_arm_pose[1]]),
                    wfb_start, wfb_goal,
                )
                print()

        husky_world.plan_and_stage_constrained(
            monitor,
            max_time=max_time,
            max_attempts=max_attempts,
        )

        # Step 3 assertions
        ok_constrained = (monitor.constrained_trajectory[0] is not None and
                          monitor.constrained_trajectory[1] is not None)
        ok_staging = (monitor.staging_free_trajectory[0] is not None and
                      monitor.staging_free_trajectory[1] is not None)

        print()
        print("=== Result ===")
        print(f"constrained_trajectory: {'OK' if ok_constrained else 'MISSING'}")
        if ok_constrained:
            print(f"  waypoints (per arm): {len(monitor.constrained_trajectory[0][0])}")
        print(f"staging_free_trajectory: {'OK' if ok_staging else 'MISSING'}")
        if ok_staging:
            print(f"  waypoints (per arm): {len(monitor.staging_free_trajectory[0][0])}")
        print(f"set_arm_trajectory calls: {len(monitor._captured['set_arm_trajectory'])}")
        print(f"set_to_show_traj_state calls: {monitor._captured['show_traj_state_calls']}")

        return 0 if ok_constrained else 1
    finally:
        pp.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", type=str, default="D1",
                        help="Antenna case-study target (e.g. D1, G1, V1, H1).")
    parser.add_argument("--stage", type=int, default=3, choices=[1, 2, 3])
    parser.add_argument("--max-time", type=float, default=30.0,
                        help="Per-attempt RRT budget in seconds.")
    parser.add_argument("--max-attempts", type=int, default=5,
                        help="Number of independent RRT restarts.")
    parser.add_argument("--diagnose", action="store_true",
                        help="Print detailed collision diagnostics at derived start_conf and goal_conf before planning.")
    parser.add_argument("--problem", type=str, default=ANTENNA_PROBLEM,
                        help="VALIDATION_PROBLEM_NAME directory under data/husky_assembly_design_study/")
    args = parser.parse_args()
    sys.exit(main(target=args.target, stage=args.stage, max_time=args.max_time,
                  max_attempts=args.max_attempts, diagnose=args.diagnose,
                  antenna_problem=args.problem))
