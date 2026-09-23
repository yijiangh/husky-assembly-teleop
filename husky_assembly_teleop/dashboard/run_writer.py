"""Turn one M1 start-derivation result into a run file for the dashboard.

Runs on the PRODUCER side -- the live monitor's derive button, or the headless
script -- because only there do the planner, the cell state and the collision
checker exist together. Everything the dashboard later needs is captured here:
the scene's pose bookkeeping, the per-candidate trace the sweep recorded, and
the collision pairs behind the failures that are worth explaining.

! The collision annotation has to run against the SAME state the sweep used
! (its allowed-collision matrix, its hidden built bars, the live base), which
! is exactly why it happens here and not in the server.
"""
import json
import os
from datetime import datetime

import numpy as np
import pybullet_planning as pp

from husky_assembly_teleop.cc_diagnosis import (
    collect_collision_contacts, _deepest_point, _penetration_depth, _PT_POS_A,
)
from husky_assembly_teleop.dashboard.run_schema import (
    RUN_KIND, SCHEMA, run_id, runs_dir_default, validate_run,
)
from husky_assembly_teleop.dashboard.scene_export import ensure_scene_glb
from husky_assembly_teleop.utils import pose_from_frame

# How many colliding candidates to explain in detail. Each check re-runs the
# full cfab collision report (~50 ms), and the dashboard only needs enough
# examples to show the pattern.
MAX_COLLISION_CHECKS = 40


def _pose(pose):
    """A pybullet ``(pos, quat)`` pair as the run file's pose dict."""
    return {'pos': [round(float(v), 6) for v in pose[0]],
            'quat_xyzw': [round(float(v), 6) for v in pose[1]]}


def _node_name(cc_name):
    """Map a collision-report body name onto its glTF node name.

    compas_fab names robot links ``robot_<link>`` and everything else by its
    bare rigid-body / tool name; the exported scene uses ``robot__<link>`` and
    ``body__<name>`` (see ``scene_export``).

    Args:
        cc_name (str): the name as it appears in a collision record.

    Returns:
        str: the matching scene node name.
    """
    if cc_name.startswith('robot_') and not cc_name.startswith('robot__'):
        return 'robot__' + cc_name[len('robot_'):]
    return 'body__' + cc_name


def _rotation_of(variant_label):
    """Split a variant label into its rotation kind and size.

    Args:
        variant_label (str): e.g. ``'back/roll+30'`` or ``'vertical/canonical'``.

    Returns:
        dict: ``{'kind': 'canonical'|'roll'|'yaw', 'deg': float}``.
    """
    _, _, rotation = str(variant_label).partition('/')
    if not rotation or rotation == 'canonical':
        return {'kind': 'canonical', 'deg': 0.0}
    kind = 'roll' if rotation.startswith('roll') else 'yaw'
    try:
        return {'kind': kind, 'deg': float(rotation[4:])}
    except ValueError:
        return {'kind': kind, 'deg': 0.0}


def _quat_angle_deg(quat_a, quat_b):
    """Angle between two xyzw quaternions, in degrees."""
    dot = abs(float(np.dot(np.asarray(quat_a, dtype=float), np.asarray(quat_b, dtype=float))))
    return float(np.degrees(2.0 * np.arccos(np.clip(dot, 0.0, 1.0))))


def _parent_link_of(state, robot_cell, rb_state):
    """Which robot link a held body rides on, or None when it is static.

    Args:
        state (RobotCellState): the cell state being recorded.
        robot_cell (RobotCell): the cell (to resolve a tool's group).
        rb_state (RigidBodyState): the body's state.

    Returns:
        str | None: the robot link name, or None for a free-standing body.
    """
    if rb_state.attached_to_link:
        return rb_state.attached_to_link
    if rb_state.attached_to_tool:
        tool_state = (state.tool_states or {}).get(rb_state.attached_to_tool)
        group = getattr(tool_state, 'attached_to_group', None)
        if group:
            return robot_cell.get_link_names(group)[-1]
    return None


def _tool_root_node(robot_cell, tool_name):
    """The scene node a tool's pose should be written onto.

    ``scene_export`` gives every tool link a node named
    ``tool__<tool>__<link>``; the tool's own pose belongs on its BASE link.

    ! compas's ``get_base_link_name()`` reads ``joints[0]``, which raises on a
    ! single-link tool -- and the assembly tools in these cells have exactly
    ! one link. So the base is found as the link no joint claims as a child,
    ! which is right for both shapes.

    Args:
        robot_cell (RobotCell): the cell holding the tool model.
        tool_name (str): the tool's name in the cell.

    Returns:
        str | None: the node name, or None when the tool has no links.
    """
    tool_model = robot_cell.tool_models[tool_name]
    links = list(tool_model.iter_links())
    if not links:
        return None
    children = {joint.child.link for joint in tool_model.iter_joints()}
    base = next((link for link in links if link.name not in children), links[0])
    return f'tool__{tool_name}__{base.name}'


def _scene_poses(planner, state):
    """Split the cell's bodies into 'rides with the robot' and 'stays put'.

    The riding ones are stored relative to their parent link, so the viewer can
    re-place them from forward kinematics at any configuration; the static ones
    are stored in world coordinates once.

    Args:
        planner: the compas_fab ``PyBulletPlanner`` (already holding ``state``).
        state (RobotCellState): the state the derivation ran on.

    Returns:
        tuple[list, list]: ``(attachments, static_bodies)`` for the run file.
    """
    client = planner.client
    robot_cell = client.robot_cell
    attachments, static_bodies = [], []

    for name, rb_state in (state.rigid_body_states or {}).items():
        parent_link = _parent_link_of(state, robot_cell, rb_state)
        if parent_link is None:
            # ! Read a hidden body's pose from its state, never from pybullet:
            # ! compas_fab skips repositioning hidden bodies, so the simulator
            # ! still holds wherever that body was last placed.
            if rb_state.frame is None:
                continue
            static_bodies.append({
                'node': f'body__{name}',
                'world': _pose(pose_from_frame(rb_state.frame)),
                'hidden': bool(rb_state.is_hidden),
            })
            continue
        puids = (client.rigid_bodies_puids or {}).get(name) or []
        if not puids:
            continue
        link_puid = (client.robot_link_puids or {}).get(parent_link)
        if link_puid is None:
            continue
        world_from_link = pp.get_link_pose(client.robot_puid, link_puid)
        attachments.append({
            'node': f'body__{name}',
            'parent_link': parent_link,
            'link_from_body': _pose(pp.multiply(pp.invert(world_from_link),
                                                pp.get_pose(puids[0]))),
        })

    for tool_name, tool_state in (state.tool_states or {}).items():
        group = getattr(tool_state, 'attached_to_group', None)
        tool_puid = (client.tools_puids or {}).get(tool_name)
        if not group or tool_puid is None or tool_state.is_hidden:
            continue
        parent_link = robot_cell.get_link_names(group)[-1]
        link_puid = (client.robot_link_puids or {}).get(parent_link)
        node = _tool_root_node(robot_cell, tool_name)
        if link_puid is None or node is None:
            continue
        world_from_link = pp.get_link_pose(client.robot_puid, link_puid)
        attachments.append({
            'node': node,
            'parent_link': parent_link,
            'link_from_body': _pose(pp.multiply(pp.invert(world_from_link),
                                                pp.get_pose(tool_puid))),
        })
    return attachments, static_bodies


def build_run_record(planner, state, info, *, problem, bar_action, active_bar,
                     movement_id=None, home_anchor=None, source='monitor',
                     scene_ref=None, when=None):
    """Assemble the dashboard's run record from a derivation result.

    Args:
        planner: the compas_fab ``PyBulletPlanner`` the derivation used.
        state (RobotCellState): the M1 start state it ran on.
        info (dict): what ``_derive_constrained_start_for_plan`` returned.
        problem (str): design problem name.
        bar_action (str): bar action file stem, e.g. ``'B3'``.
        active_bar (str): the active bar's rigid-body name, e.g. ``'bar_B3'``.
        movement_id (str | None): the movement this derivation belongs to.
        home_anchor (str | None): anchor selection (None = all).
        source (str): ``'monitor'`` or ``'headless'``.
        scene_ref (str | None): relative path of the exported scene.
        when (datetime | None): run timestamp (now by default).

    Returns:
        dict: the run record, ready for ``validate_run`` / ``write_run``.
    """
    when = when or datetime.now()
    context_in = info.get('context') or {}
    tracked = info.get('tracked') or {}
    profile = tracked.get('profile') or {}
    candidates_in = tracked.get('candidates') or []

    world_from_mb = context_in.get('world_from_mobile_base') or ((0, 0, 0), (0, 0, 0, 1))
    bar_goal = context_in.get('world_from_bar_goal') or ((0, 0, 0), (0, 0, 0, 1))
    mb_from_bar_goal = pp.multiply(pp.invert(world_from_mb), bar_goal)
    rebranch_deg = float(np.degrees(context_in.get('goal_rebranch_max_rad') or 0.0))

    attachments, static_bodies = _scene_poses(planner, state)
    sweep = tracked.get('sweep_params') or {}

    candidates = []
    for i, cand in enumerate(candidates_in):
        home_quat = cand.get('home_quat_mb') or [0.0, 0.0, 0.0, 1.0]
        travel_m = float(np.linalg.norm(
            np.asarray(cand.get('home_mb') or [0, 0, 0], dtype=float)
            - np.asarray(context_in.get('goal_mb') or tracked.get('goal_mb') or [0, 0, 0],
                         dtype=float)))
        brk = cand.get('break')
        if brk:
            brk = dict(brk)
            if 'jump_rad' in brk:
                brk['jump_deg'] = float(np.degrees(brk.pop('jump_rad')))
            if isinstance(brk.get('pose'), dict):
                brk['pose_world'] = brk.pop('pose')
        candidates.append({
            'i': i,
            'variant': cand.get('variant'),
            'anchor': cand.get('anchor'),
            'rotation': _rotation_of(cand.get('variant')),
            'delta_m': cand.get('delta'),
            'home_pos_mb': cand.get('home_mb'),
            'home_quat_mb': home_quat,
            'travel_cm': round(travel_m * 100.0, 2),
            'travel_deg': round(_quat_angle_deg(
                home_quat, context_in.get('goal_quat_mb') or mb_from_bar_goal[1]), 2),
            'n_total': cand.get('n_total'),
            'n_wp': cand.get('n_wp'),
            'reached': cand.get('reached'),
            'blocked_at': cand.get('blocked_at'),
            'blocked_index': cand.get('blocked_index'),
            'blocked_conf': cand.get('blocked_conf'),
            'arrival_conf': cand.get('arrival_conf'),
            'last_conf': cand.get('last_conf'),
            'track': cand.get('track') or {'indices': [], 'confs': []},
            't_at_s': round(float(cand.get('t_at') or 0.0), 3),
            't_track_s': round(float(cand.get('t_track') or 0.0), 3),
            'outcome': cand.get('outcome') or 'track_break',
            'break': brk,
            'collisions': None,
            'collision_conf': None,
        })

    # The winner: a clear corridor if one turned up, else the first candidate
    # that at least arrived cleanly (what the sweep keeps as its answer).
    winner = next((c['i'] for c in candidates if c['outcome'] == 'corridor'), None)
    if winner is None:
        winner = next((c['i'] for c in candidates
                       if c['outcome'] in ('blocked', 'fine_reverify_failed')), None)
    found = info.get('derived_start_conf') is not None
    if found:
        kind = ('corridor' if any(c['outcome'] == 'corridor' for c in candidates)
                else 'partial' if tracked.get('partial') else 'start_only')
    else:
        kind = 'failed'

    run = {
        'schema': SCHEMA,
        'kind': RUN_KIND,
        'id': run_id(problem, bar_action, home_anchor, when),
        'created': when.isoformat(timespec='seconds'),
        'source': source,
        'problem': problem,
        'bar_action': bar_action,
        'active_bar': active_bar,
        'movement_id': movement_id,
        'anchor_selection': home_anchor or 'all',
        'result': {
            'found': bool(found),
            'kind': kind,
            'failure_reason': info.get('failure_reason'),
            'winner_candidate': winner,
            't_found_s': (candidates[winner]['t_at_s'] if winner is not None else None),
            'start_conf': info.get('derived_start_conf'),
            'winner_variant': tracked.get('variant'),
            'goal_collisions': None,
        },
        'stage_times': info.get('stage_times') or {},
        'context': {
            'joint_names_12': context_in.get('joint_names_12') or [],
            'world_from_mobile_base': _pose(world_from_mb),
            'goal': {
                'conf': context_in.get('goal_conf') or [],
                'conf_authored': context_in.get('goal_conf_authored') or [],
                'bar_pose_world': _pose(bar_goal),
                'bar_pos_mb': [round(float(v), 6) for v in mb_from_bar_goal[0]],
                'bar_quat_mb': [round(float(v), 6) for v in mb_from_bar_goal[1]],
                'rebranched': rebranch_deg > 0.06,   # ~0.001 rad, i.e. a real change
                'rebranch_max_deg': round(rebranch_deg, 2),
                'pairing_cross_distance_deg': context_in.get('pairing_cross_distance_deg'),
            },
            'grasps': {
                'bar_from_left_tool0': _pose(context_in.get('grasp_bar_from_left')
                                             or ((0, 0, 0), (0, 0, 0, 1))),
                'bar_from_right_tool0': _pose(context_in.get('grasp_bar_from_right')
                                              or ((0, 0, 0), (0, 0, 0, 1))),
                'attach_link': context_in.get('attach_link'),
                'tool0_from_bar': _pose(context_in.get('tool0_from_bar')
                                        or ((0, 0, 0), (0, 0, 0, 1))),
            },
            'variants_mb': tracked.get('variants_mb') or [],
            'budget': {
                'max_time_s': profile.get('max_time'),
                'per_anchor_s': profile.get('anchor_allowance'),
                'n_variants': profile.get('n_variants'),
                'n_deltas': profile.get('n_deltas'),
                'grid_step_m': sweep.get('grid_step'),
                'screen_step_m': sweep.get('screen_pos'),
                'screen_step_rad': sweep.get('screen_rot'),
                'continuity_limit_deg': round(
                    float(np.degrees(sweep.get('continuity_rad') or 0.0)), 2),
            },
            'attachments': attachments,
            'static_bodies': static_bodies,
            'scene': {'glb': scene_ref or f'{problem}/scene.glb'},
        },
        'profile': profile,
        'candidates': candidates,
    }
    return run


def annotate_collisions(planner, state, run, max_checks=MAX_COLLISION_CHECKS):
    """Fill in WHICH link hit WHICH body for the colliding candidates.

    The sweep only records a yes/no collision verdict (that is all it needs),
    so the pairs are recovered here by re-running the full collision report at
    the offending configuration. Only candidates whose failure IS a collision
    are worth the ~50 ms each, and only the first ``max_checks`` of them.

    Args:
        planner: the compas_fab ``PyBulletPlanner``.
        state (RobotCellState): the state the derivation ran on.
        run (dict): the run record, edited in place.
        max_checks (int): cap on the number of collision reports.

    Returns:
        int: how many candidates were annotated.
    """
    from husky_assembly_tamp.motion_planner.api import _state_with_conf12

    names_12 = run['context']['joint_names_12']

    def _pairs(conf):
        """Every colliding pair at one configuration, deepest first."""
        records = collect_collision_contacts(
            planner, _state_with_conf12(state, conf, names_12))
        pairs = []
        for record in records:
            point = _deepest_point(record)
            pairs.append({
                'a': _node_name(record['name_a']),
                'b': _node_name(record['name_b']),
                'depth_mm': round(_penetration_depth(record) * 1000.0, 2),
                'point': ([round(float(v), 5) for v in point[_PT_POS_A]]
                          if point is not None else None),
            })
        return pairs

    annotated = 0
    try:
        for cand in run['candidates']:
            if annotated >= max_checks:
                break
            if cand['outcome'] == 'arrival_collision' and cand.get('arrival_conf'):
                cand['collisions'] = _pairs(cand['arrival_conf'])
                cand['collision_conf'] = 'arrival'
                annotated += 1
            elif cand['outcome'] == 'blocked' and cand.get('blocked_conf'):
                cand['collisions'] = _pairs(cand['blocked_conf'])
                cand['collision_conf'] = 'blocked'
                annotated += 1
        # A goal that collides is the whole story when the derivation never
        # started, so explain that one too.
        if run['result'].get('failure_reason') == 'goal_in_collision':
            goal_conf = run['context']['goal'].get('conf')
            if goal_conf:
                run['result']['goal_collisions'] = _pairs(goal_conf)
    finally:
        # Leave the simulator exactly as the derivation left it.
        planner.set_robot_cell_state(state)
    return annotated


def write_run(run, runs_dir=None):
    """Validate a run record and write it atomically into the watched folder.

    The dashboard watches this folder, so the file must never be visible while
    half-written: it is written to a temporary name and renamed into place.

    Args:
        run (dict): the run record.
        runs_dir (str | None): destination folder (the default is watched).

    Returns:
        str: the written path.
    """
    validate_run(run)
    runs_dir = runs_dir or runs_dir_default()
    os.makedirs(runs_dir, exist_ok=True)
    path = os.path.join(runs_dir, f'{run["id"]}.json')
    tmp = path + '.tmp'
    with open(tmp, 'w') as handle:
        json.dump(run, handle)
    os.replace(tmp, path)
    return path


def write_m1_run(planner, state, info, *, problem, bar_action, active_bar,
                 movement_id=None, home_anchor=None, source='monitor',
                 runs_dir=None, scenes_dir=None):
    """Export the scene (once), build, annotate and write one run file.

    This is the single call the producers make after a derivation.

    Args:
        planner: the compas_fab ``PyBulletPlanner`` the derivation used.
        state (RobotCellState): the M1 start state it ran on.
        info (dict): what ``_derive_constrained_start_for_plan`` returned.
        problem (str): design problem name.
        bar_action (str): bar action file stem, e.g. ``'B3'``.
        active_bar (str): the active bar's rigid-body name.
        movement_id (str | None): the movement id.
        home_anchor (str | None): anchor selection (None = all).
        source (str): ``'monitor'`` or ``'headless'``.
        runs_dir (str | None): where to write (default: the watched folder).
        scenes_dir (str | None): scene cache root.

    Returns:
        str: the written run path.
    """
    scene_ref = ensure_scene_glb(planner.client.robot_cell, problem, scenes_dir=scenes_dir)
    run = build_run_record(planner, state, info, problem=problem, bar_action=bar_action,
                           active_bar=active_bar, movement_id=movement_id,
                           home_anchor=home_anchor, source=source, scene_ref=scene_ref)
    annotate_collisions(planner, state, run)
    path = write_run(run, runs_dir=runs_dir)
    print(f'[dashboard] run written: {path}')
    return path
