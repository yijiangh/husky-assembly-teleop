"""Headless harness for husky_world.plan_and_stage_constrained.

Builds a synthetic dual-arm scene in PyBullet DIRECT mode and a minimal
mock monitor with just the attributes the planner reads, then exercises
the full constrained + staging flow without ROS or the GUI.

Goal: let us iterate on the constrained planner integration without
clicking through the live monitor.

Usage (with the ros2_ws venv active):
  python src/husky-assembly-teleop/scripts/headless_constrained_monitor.py [--stage 1|2|3]
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


class StubLogger:
    def warn(self, msg):  # ROS Node.get_logger().warn shape
        print(f"[WARN] {msg}")

    def info(self, msg):
        print(f"[INFO] {msg}")


def _make_stub_monitor(robot, goal_conf, active_bar_body, active_bar_aabb_dims,
                       attachments_pair, trajectory_time=5.0, stage=3):
    """Construct a SimpleNamespace shaped like HuskyMonitor for the planner.

    Only attributes read by world.plan_and_stage_constrained are populated.
    """
    husky_object = SimpleNamespace(
        robot=robot,
        ee_list=[(None, attachments_pair[0]), (None, attachments_pair[1])],
        # `dual_arm` flag isn't read by plan_and_stage_constrained but kept for parity.
        dual_arm=True,
    )
    husky = SimpleNamespace(object=husky_object, interface=SimpleNamespace())

    captured = {"set_arm_trajectory": [], "show_traj_state_calls": 0}

    def set_arm_trajectory(traj, index):
        captured["set_arm_trajectory"].append((index, traj))

    def set_to_show_traj_state():
        captured["show_traj_state_calls"] += 1

    monitor = SimpleNamespace(
        huskies={0: husky},
        selected_robot_id=0,
        active_bar_body=active_bar_body,
        active_bar_aabb_dims=active_bar_aabb_dims,
        active_bar_name="synthetic_bar",
        goal_arm_pose=[np.asarray(goal_conf[:6]), np.asarray(goal_conf[6:])],
        static_obstacles={"floor": -1, "active_bar": active_bar_body},  # bar stored same as monitor would
        grasp_targets_override=None,
        constrained_planner_stage=stage,
        constrained_display_mode=0,
        staging_free_trajectory=[None, None],
        constrained_trajectory=[None, None],
        trajectory_time=trajectory_time,
        get_logger=lambda: StubLogger(),
        set_arm_trajectory=set_arm_trajectory,
        set_to_show_traj_state=set_to_show_traj_state,
        _captured=captured,
    )

    def _refresh_constrained_displayed_trajectory():
        src = (monitor.constrained_trajectory if monitor.constrained_display_mode == 1
               else monitor.staging_free_trajectory)
        if src[0] is not None and src[1] is not None:
            monitor.set_arm_trajectory(src[0], index=0)
            monitor.set_arm_trajectory(src[1], index=1)
            monitor.set_to_show_traj_state()
    monitor._refresh_constrained_displayed_trajectory = _refresh_constrained_displayed_trajectory
    return monitor


def _spawn_floor():
    """Match what the live monitor adds (a flat ground)."""
    floor = pp.create_box(4.0, 4.0, 0.01, color=(0.7, 0.7, 0.7, 1.0), mass=pp.STATIC_MASS)
    pp.set_pose(floor, ((0.0, 0.0, -0.005), (0.0, 0.0, 0.0, 1.0)))
    return floor


def _build_attachments(robot):
    """Mimic husky.object.ee_list: rigid attachments at left/right tool0.

    Real grippers in the live monitor are separate bodies. For headless we
    just create two tiny boxes anchored to each tool0 to satisfy the
    plan_free_dual_arm len==2 attachment requirement.
    """
    tool0_L = pp.link_from_name(robot, "left_ur_arm_tool0")
    tool0_R = pp.link_from_name(robot, "right_ur_arm_tool0")
    pose_L = pp.get_link_pose(robot, tool0_L)
    pose_R = pp.get_link_pose(robot, tool0_R)
    gL = pp.create_box(0.05, 0.05, 0.05, color=(0.2, 0.2, 0.8, 1.0), mass=pp.STATIC_MASS)
    gR = pp.create_box(0.05, 0.05, 0.05, color=(0.8, 0.2, 0.2, 1.0), mass=pp.STATIC_MASS)
    pp.set_pose(gL, pose_L)
    pp.set_pose(gR, pose_R)
    aL = pp.create_attachment(robot, tool0_L, gL)
    aR = pp.create_attachment(robot, tool0_R, gR)
    return [aL, aR]


def _build_known_feasible_goal(robot, arm_joints, tool_link_L, tool_link_R):
    """Pick a goal bar pose + goal_conf that is known-feasible via IK.

    Strategy: anchor the left tool0 at MOBILE_BASE_FROM_TOOL0_LEFT_HOME (which the
    submodule guarantees is reachable), define a simple symmetric grasp, derive
    bar pose + right tool0 pose, then solve dual-arm endpoint IK to get the
    12-DOF goal_conf. Returns (world_from_bar_goal, goal_conf).
    """
    from husky_assembly_tamp.motion_planner.stage1.minimal_rrt import (
        MOBILE_BASE_FROM_TOOL0_LEFT_HOME,
        INIT_ARM_JOINT_ANGLES,
        solve_endpoint_dual_arm_ik,
    )
    # Symmetric grasp: bar oriented along its long axis (x) at the midpoint.
    # bar_from_tool0_left translates +0.3m along bar x from bar center.
    # bar_from_tool0_right is the mirror.
    bar_from_tool0_left = ((0.3, 0.0, 0.0), (0.0, 0.7071068, 0.0, 0.7071068))   # tool z faces -bar_x
    bar_from_tool0_right = ((-0.3, 0.0, 0.0), (0.0, -0.7071068, 0.0, 0.7071068))  # mirror
    grasp_bar_from_left = pp.invert(bar_from_tool0_left)
    grasp_bar_from_right = pp.invert(bar_from_tool0_right)

    world_from_tool0_L_goal = MOBILE_BASE_FROM_TOOL0_LEFT_HOME  # base==world
    world_from_bar_goal = pp.multiply(world_from_tool0_L_goal, bar_from_tool0_left)

    rng = np.random.default_rng(0)
    with pp.WorldSaver():
        pp.set_joint_positions(robot, arm_joints, INIT_ARM_JOINT_ANGLES)
        goal_conf = solve_endpoint_dual_arm_ik(
            robot=robot, arm_joints=arm_joints,
            tool_link_left=tool_link_L, tool_link_right=tool_link_R,
            bar_pose=world_from_bar_goal,
            grasp_bar_from_left=grasp_bar_from_left,
            grasp_bar_from_right=grasp_bar_from_right,
            seed_conf=INIT_ARM_JOINT_ANGLES,
            rng=rng, max_attempts=20,
        )
    if goal_conf is None:
        raise RuntimeError("Could not solve IK at the synthetic home goal pose")
    return world_from_bar_goal, np.asarray(goal_conf), grasp_bar_from_left, grasp_bar_from_right


def _build_real_antenna_case(robot, arm_joints, tool_link_L, tool_link_R, target: str):
    """Load real antenna case-study data for a given bar target (D1, G1, ..., V3).

    Returns (world_from_bar_goal, goal_conf, grasp_bar_from_left, grasp_bar_from_right,
             bar_body, bar_aabb_dims).
    """
    from husky_assembly_tamp.motion_planner.stage1.minimal_rrt import (
        load_grasp_targets, load_robot_cell_state, get_goal_pose_from_grasp_targets,
        load_design_study_bar_mesh, create_bar_mesh_body,
    )
    base_dir = (
        "/home/yijiangh/Code/ros2_ws/src/husky-assembly-teleop/data/"
        "husky_assembly_design_study/250929_New_Antenna_with_GH_RH_Packed"
    )
    grasp_json = os.path.join(base_dir, "RobotCellStates", f"{target}_GraspTargets.json")
    state_json = os.path.join(base_dir, "RobotCellStates", f"{target}_RobotCellState.json")
    robot_cell_json = os.path.join(base_dir, "RobotCell.json")
    if not all(os.path.exists(p) for p in [grasp_json, state_json, robot_cell_json]):
        raise FileNotFoundError(f"missing antenna case files for target={target}")

    grasp_targets = load_grasp_targets(grasp_json)
    if len(grasp_targets) < 2:
        raise ValueError(f"Expected dual-arm grasps in {grasp_json}; got {len(grasp_targets)}")
    world_from_bar_l, world_from_tool0_left = grasp_targets[0]
    world_from_bar_r, world_from_tool0_right = grasp_targets[1]
    grasp_bar_from_left = pp.multiply(pp.invert(world_from_bar_l), world_from_tool0_left)
    grasp_bar_from_right = pp.multiply(pp.invert(world_from_bar_r), world_from_tool0_right)
    world_from_bar_goal = get_goal_pose_from_grasp_targets(grasp_targets)
    goal_conf = np.asarray(load_robot_cell_state(state_json))
    if goal_conf.shape[0] != 12:
        raise ValueError(f"Expected 12-DOF goal_conf, got shape {goal_conf.shape}")

    mesh_spec = load_design_study_bar_mesh(robot_cell_json, target)
    bar_body = create_bar_mesh_body(mesh_spec, color=(0.8, 0.4, 0.1, 0.8), collision=True)
    pp.set_pose(bar_body, world_from_bar_goal)
    bar_aabb_dims = mesh_spec["aabb_dims"]

    return (world_from_bar_goal, goal_conf, grasp_bar_from_left,
            grasp_bar_from_right, bar_body, bar_aabb_dims)


def main(stage: int = 3, max_time: float = 5.0,
         antenna_target: Optional[str] = None) -> int:
    from husky_assembly_teleop import husky_world
    from husky_assembly_tamp.motion_planner.stage1.minimal_rrt import (
        HUSKY_DUAL_ARM_JOINT_NAMES,
        INIT_ARM_JOINT_ANGLES,
    )

    pp.connect(use_gui=False)
    try:
        robot = pp.load_pybullet(URDF, fixed_base=True)
        # No floor in this harness — the prototype's setup_planning_scene
        # doesn't load one either; adding one can clash with the husky base.
        floor = None
        arm_joints = pp.joints_from_names(robot, HUSKY_DUAL_ARM_JOINT_NAMES)
        tool_link_L = pp.link_from_name(robot, "left_ur_arm_tool0")
        tool_link_R = pp.link_from_name(robot, "right_ur_arm_tool0")

        if antenna_target:
            print(f"loading real antenna case-study data: target={antenna_target}")
            (world_from_bar_goal, goal_conf, grasp_bar_from_left,
             grasp_bar_from_right, bar_body, bar_aabb_dims) = (
                _build_real_antenna_case(robot, arm_joints, tool_link_L, tool_link_R, antenna_target)
            )
        else:
            world_from_bar_goal, goal_conf, grasp_bar_from_left, grasp_bar_from_right = (
                _build_known_feasible_goal(robot, arm_joints, tool_link_L, tool_link_R)
            )
            bar_body = pp.create_box(0.6, 0.05, 0.05, color=(0.8, 0.4, 0.1, 1.0), mass=pp.STATIC_MASS)
            pp.set_pose(bar_body, world_from_bar_goal)
            bar_aabb_dims = pp.get_aabb_extent(pp.get_aabb(bar_body))

        left_goal = goal_conf[:6]
        right_goal = goal_conf[6:]

        # "Current" conf for the live robot: prototype's INIT pose (different from goal_conf)
        current_conf = np.asarray(INIT_ARM_JOINT_ANGLES)
        current_left = current_conf[:6]
        current_right = current_conf[6:]
        pp.set_joint_positions(robot, arm_joints, current_conf)

        attachments_pair = _build_attachments(robot)

        monitor = _make_stub_monitor(
            robot=robot,
            goal_conf=goal_conf,
            active_bar_body=bar_body,
            active_bar_aabb_dims=bar_aabb_dims,
            attachments_pair=attachments_pair,
            stage=stage,
        )
        # No external static obstacles in this synthetic harness. The active
        # bar must NOT appear here — it is tracked separately via
        # monitor.active_bar_body. The constrained planner uses bar_body as
        # the manipulated body (attached via the grasp); keeping it in
        # obstacles too would force the bar to collide with itself.
        monitor.static_obstacles = {}
        # When using real antenna data the loaded grasps are authoritative;
        # bypass FK-based derivation in plan_and_stage_constrained.
        if antenna_target:
            monitor.grasp_targets_override = (grasp_bar_from_left, grasp_bar_from_right)

        print(f"=== headless_constrained_monitor: stage={stage}, max_time={max_time}s ===")
        print(f"current_conf  L: {np.round(current_left, 3)}")
        print(f"              R: {np.round(current_right, 3)}")
        print(f"goal_conf     L: {np.round(left_goal, 3)}")
        print(f"              R: {np.round(right_goal, 3)}")
        print(f"world_from_bar_goal pos: {np.round(world_from_bar_goal[0], 3)}")
        print(f"bar_aabb_dims: {np.round(bar_aabb_dims, 4)}")
        print()

        husky_world.plan_and_stage_constrained(monitor, debug=False)

        ok_constrained = monitor.constrained_trajectory[0] is not None and monitor.constrained_trajectory[1] is not None
        ok_free = monitor.staging_free_trajectory[0] is not None and monitor.staging_free_trajectory[1] is not None
        n_set_calls = len(monitor._captured["set_arm_trajectory"])

        print()
        print("=== Result ===")
        print(f"constrained_trajectory: {'OK' if ok_constrained else 'MISSING'}")
        if ok_constrained:
            n = len(monitor.constrained_trajectory[0][0])
            print(f"  waypoints (per arm): {n}")
        print(f"staging_free_trajectory: {'OK' if ok_free else 'MISSING'}")
        if ok_free:
            n = len(monitor.staging_free_trajectory[0][0])
            print(f"  waypoints (per arm): {n}")
        print(f"set_arm_trajectory calls: {n_set_calls}")
        print(f"set_to_show_traj_state calls: {monitor._captured['show_traj_state_calls']}")

        # success criteria: constrained must be present; staging may legitimately
        # fail in stage 3 if endpoints are tight, but we report both.
        return 0 if ok_constrained else 1
    finally:
        pp.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", type=int, default=3, choices=[1, 2, 3])
    parser.add_argument("--max-time", type=float, default=5.0)
    parser.add_argument(
        "--antenna",
        type=str,
        default=None,
        help="Real antenna case target (e.g. D1, G1, V3). When unset, uses synthetic scene.",
    )
    args = parser.parse_args()
    sys.exit(main(stage=args.stage, max_time=args.max_time, antenna_target=args.antenna))
