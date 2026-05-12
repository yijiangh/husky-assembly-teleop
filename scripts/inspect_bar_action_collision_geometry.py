"""Inspect cfab vs bridged-pp collision geometry for one BarAction.

This is intentionally read-only: it loads the same BarAction path used by
``HuskyMonitor.load_bar_action`` and reports which PyBullet bodies each
collision path can see.
"""

from __future__ import annotations

import argparse
import os
import re
from collections import Counter
from types import SimpleNamespace

import numpy as np


DEFAULT_PROBLEM = "2026-05-08_dual-arm_transfer_test"
DEFAULT_BAR_ACTION = "B6.json"
DEFAULT_MOVEMENT = "M1"


class StubLogger:
    def warn(self, msg):
        print(f"[WARN] {msg}")

    def info(self, msg):
        print(f"[INFO] {msg}")

    def error(self, msg):
        print(f"[ERROR] {msg}")


def _patch_problem(problem):
    from husky_assembly_teleop import husky_monitor as hm

    hm.VALIDATION_PROBLEM_NAME = problem


def _make_monitor(problem, use_gui):
    from husky_assembly_teleop.cfab_session import CfabSession
    from husky_assembly_teleop.husky_monitor import HuskyMonitor
    import pybullet_planning as pp

    monitor = object.__new__(HuskyMonitor)
    monitor.huskies = []
    monitor.selected_robot_id = 0

    monitor.static_obstacles = {}
    monitor.active_bar_body = None
    monitor.active_bar_aabb_dims = None
    monitor.active_bar_name = None
    monitor.active_extra_bodies = []
    monitor.bar_from_extra = []

    monitor._bar_action_husky = None
    monitor._bar_action_ghost_bodies = set()
    monitor._bar_action_cfab_id = None
    monitor.bar_action_staging_seed_conf = None
    monitor._bar_action_scrub = None

    monitor.current_action = None
    monitor.current_movement = None
    monitor.current_movement_index = None
    monitor.movement_type = None
    monitor.movement_start_state = None
    monitor.target_ee_frames = None
    monitor.grasp_link_from_bar = None

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
    monitor.show_goal_state = False
    monitor.trajectory_time = 20.0
    monitor.selected_arm_index = 0

    monitor.set_arm_trajectory = lambda traj, index=0: None
    monitor.set_to_show_traj_state = lambda: None
    monitor.set_to_show_goal_state = lambda: None
    monitor.reset_ui = lambda target_conf=None: None
    monitor.get_logger = lambda: StubLogger()

    monitor.cfab = CfabSession(
        problem,
        connection_type="gui" if use_gui else "direct",
        enable_debug_gui=use_gui,
    )
    pp.CLIENT = monitor.cfab.client.client_id
    pp.CLIENTS.setdefault(monitor.cfab.client.client_id, True if use_gui else None)
    return monitor


def _collision_shape_count(body, *, client_id, link_index=None):
    import pybullet as p

    if link_index is None:
        data = p.getCollisionShapeData(body, -1, physicsClientId=client_id)
        try:
            n_links = p.getNumJoints(body, physicsClientId=client_id)
        except Exception:
            n_links = 0
        for link in range(n_links):
            data += p.getCollisionShapeData(body, link, physicsClientId=client_id)
        return len(data)
    return len(p.getCollisionShapeData(body, link_index, physicsClientId=client_id))


def _body_label_map(client):
    labels = {}
    if client.robot_puid is not None:
        labels[client.robot_puid] = "robot"
    for name, puid in client.tools_puids.items():
        labels[puid] = f"tool:{name}"
    for name, puids in client.rigid_bodies_puids.items():
        for puid in puids:
            labels[puid] = f"rigid_body:{name}"
    return labels


def _count_cfab_collision_candidates(client):
    from itertools import combinations

    state = client.robot_cell_state
    robot_links = client.robot_link_puids
    tools = client.tools_puids
    bodies = client.rigid_bodies_puids

    counts = Counter()
    skipped = Counter()

    for link_1, link_2 in combinations(robot_links.keys(), 2):
        if {link_1, link_2} in client.unordered_disabled_collisions:
            skipped["cc1_semantics"] += 1
        else:
            counts["cc1_robot_link_vs_robot_link"] += 1

    for link_name in robot_links:
        for tool_name in tools:
            ts = state.tool_states[tool_name]
            if ts.is_hidden:
                skipped["cc2_tool_hidden"] += 1
            elif link_name in ts.touch_links:
                skipped["cc2_touch_link"] += 1
            else:
                counts["cc2_robot_link_vs_tool"] += 1

    for link_name in robot_links:
        for body_name, body_ids in bodies.items():
            rs = state.rigid_body_states[body_name]
            if rs.is_hidden:
                skipped["cc3_rb_hidden"] += len(body_ids)
            elif link_name in rs.touch_links:
                skipped["cc3_touch_link"] += len(body_ids)
            else:
                counts["cc3_robot_link_vs_rigid_body_mesh"] += len(body_ids)

    for body_name, body_ids in bodies.items():
        rs = state.rigid_body_states[body_name]
        if rs.is_hidden:
            skipped["cc4_attached_rb_hidden"] += 1
            continue
        if not (rs.attached_to_tool or rs.attached_to_link):
            skipped["cc4_not_attached"] += 1
            continue
        for other_name, other_ids in bodies.items():
            other = state.rigid_body_states[other_name]
            if other.is_hidden or body_name == other_name:
                continue
            if body_name in other.touch_bodies or other_name in rs.touch_bodies:
                skipped["cc4_touch_body"] += len(body_ids) * len(other_ids)
                continue
            counts["cc4_attached_rigid_body_vs_rigid_body_mesh"] += len(body_ids) * len(other_ids)

    for tool_name in tools:
        ts = state.tool_states[tool_name]
        if ts.is_hidden:
            skipped["cc5_tool_hidden"] += 1
            continue
        for body_name, body_ids in bodies.items():
            rs = state.rigid_body_states[body_name]
            if rs.is_hidden:
                skipped["cc5_rb_hidden"] += len(body_ids)
            elif rs.attached_to_tool == tool_name:
                skipped["cc5_rb_attached_to_tool"] += len(body_ids)
            elif tool_name in rs.touch_bodies:
                skipped["cc5_touch_body"] += len(body_ids)
            else:
                counts["cc5_tool_vs_rigid_body_mesh"] += len(body_ids)

    return counts, skipped


def _print_cfab_report(monitor):
    client = monitor.cfab.client
    state = client.robot_cell_state
    cid = client.client_id

    print("\n=== cfab RobotCell client ===")
    print(f"client_id: {cid}")
    print(f"robot body: {client.robot_puid}")
    print(f"robot links tracked: {len(client.robot_link_puids)}")
    print(f"robot collision shapes: {_collision_shape_count(client.robot_puid, client_id=cid)}")

    print(f"\ntools ({len(client.tools_puids)}):")
    for name, puid in client.tools_puids.items():
        ts = state.tool_states[name]
        print(
            f"  {name}: body={puid}, shapes={_collision_shape_count(puid, client_id=cid)}, "
            f"hidden={ts.is_hidden}, attached_to_group={ts.attached_to_group}, "
            f"touch_links={len(ts.touch_links)}"
        )

    print(f"\nrigid bodies ({len(client.rigid_bodies_puids)} models):")
    for name, puids in client.rigid_bodies_puids.items():
        rs = state.rigid_body_states[name]
        shape_count = sum(_collision_shape_count(puid, client_id=cid) for puid in puids)
        print(
            f"  {name}: bodies={puids}, shapes={shape_count}, hidden={rs.is_hidden}, "
            f"attached_to_tool={rs.attached_to_tool}, attached_to_link={rs.attached_to_link}, "
            f"touch_links={len(rs.touch_links)}, touch_bodies={len(rs.touch_bodies)}"
        )

    counts, skipped = _count_cfab_collision_candidates(client)
    print("\ncfab check_collision candidate pairs after RobotCellState skip rules:")
    for key in sorted(counts):
        print(f"  {key}: {counts[key]}")
    print("cfab skipped pair counts:")
    for key in sorted(skipped):
        print(f"  {key}: {skipped[key]}")


def _print_pp_report(monitor):
    client = monitor.cfab.client
    cid = client.client_id
    labels = _body_label_map(client)
    ghost_bodies = set(getattr(monitor, "_bar_action_ghost_bodies", set()) or [])

    print("\n=== bridged pybullet_planning state ===")
    print(f"pp client_id: {cid}")
    print(f"active_bar_name: {monitor.active_bar_name}")
    print(f"active_bar_body: {monitor.active_bar_body} ({labels.get(monitor.active_bar_body)})")
    print(f"ghost EE bodies: {sorted(ghost_bodies)}")
    for body in sorted(ghost_bodies):
        print(f"  ghost body {body}: shapes={_collision_shape_count(body, client_id=cid)}")

    print(f"\nstatic_obstacles ({len(monitor.static_obstacles)}):")
    for name, body in monitor.static_obstacles.items():
        label = labels.get(body, "unknown")
        print(f"  {name}: body={body}, label={label}, shapes={_collision_shape_count(body, client_id=cid)}")

    tool_body_set = set(client.tools_puids.values())
    obstacle_body_set = set(monitor.static_obstacles.values())
    tool_obstacles = sorted(tool_body_set & obstacle_body_set)
    print("\npp inclusion summary:")
    print(f"  RobotCell tool bodies in pp static_obstacles: {tool_obstacles}")
    print("  pp attachments used for BarAction staging are ghost bodies, not RobotCell tool bodies.")
    attached_rigid_obstacles = []
    state = client.robot_cell_state
    for name, body in monitor.static_obstacles.items():
        rs = state.rigid_body_states.get(name)
        if rs is not None and (rs.attached_to_link or rs.attached_to_tool):
            attached_rigid_obstacles.append((name, body, rs.attached_to_link, rs.attached_to_tool))
    print(f"  attached RobotCell rigid bodies treated as pp static_obstacles: {len(attached_rigid_obstacles)}")
    for name, body, link, tool in attached_rigid_obstacles:
        print(f"    {name}: body={body}, attached_to_link={link}, attached_to_tool={tool}")

    # Mirror the name-based filter in husky_world._plan_and_stage_body.
    bar_name_re = re.compile(r"^b\d+(_0|_joint_\d+)$")
    name_filtered = [name for name in monitor.static_obstacles if bar_name_re.match(name)]
    bar_like = [name for name in monitor.static_obstacles if name.startswith("bar_")]
    print("  husky_world name filter preview:")
    print(f"    regex-excluded assembly names: {name_filtered}")
    print(f"    bar_* names not matched by that regex: {bar_like}")

    husky = getattr(monitor, "_bar_action_husky", None)
    if husky is not None:
        for idx, (_body, attachment) in enumerate(husky.object.ee_list):
            print(
                f"  attachment[{idx}]: parent robot={attachment.parent}, "
                f"parent_link={attachment.parent_link}, child={attachment.child}"
            )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--problem", default=DEFAULT_PROBLEM)
    parser.add_argument("--bar-action", default=DEFAULT_BAR_ACTION)
    parser.add_argument("--movement", default=DEFAULT_MOVEMENT)
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()

    _patch_problem(args.problem)

    monitor = None
    try:
        monitor = _make_monitor(args.problem, args.gui)
        monitor.available_robot_cell_states = monitor._load_available_robot_cell_states()
        if args.bar_action not in monitor.available_robot_cell_states:
            raise RuntimeError(
                f"{args.bar_action!r} not available. Found: {monitor.available_robot_cell_states}"
            )
        monitor.selected_state_index = monitor.available_robot_cell_states.index(args.bar_action)
        ok = monitor.load_bar_action(movement=args.movement, update_goal_state=False)
        if not ok:
            return 1
        _print_cfab_report(monitor)
        _print_pp_report(monitor)
        return 0
    finally:
        if monitor is not None and getattr(monitor, "cfab", None) is not None:
            monitor.cfab.close()


if __name__ == "__main__":
    raise SystemExit(main())
