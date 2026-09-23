"""Tests for the M1 derivation dashboard that need neither robot nor mocap.

The schema and wording tests run on a hand-written record, so they always run.
The scene/kinematics tests need a real run on disk (produce one with
``scripts/derive_m1_headless.py --bar B3``) and skip when there is none.
"""
import json
import os
import struct

import numpy as np
import pytest

from husky_assembly_teleop.dashboard.kinematics import (
    SceneKinematics, _matrix_from_pose,
)
from husky_assembly_teleop.dashboard.run_schema import (
    SCHEMA, describe_candidate, describe_run, runs_dir_default,
    scenes_dir_default, validate_run, variant_plain,
)


def _pose(pos=(0, 0, 0), quat=(0, 0, 0, 1)):
    """A pose dict in the run file's convention."""
    return {'pos': list(pos), 'quat_xyzw': list(quat)}


def _synthetic_run():
    """A minimal but complete run record: one broken walk, one collision."""
    return {
        'schema': SCHEMA, 'kind': 'm1_derive', 'id': 'test-run',
        'created': '2026-09-23T15:04:12', 'source': 'headless',
        'problem': 'test_problem', 'bar_action': 'B3', 'active_bar': 'bar_B3',
        'movement_id': 'B3_M1', 'anchor_selection': 'all',
        'result': {'found': True, 'kind': 'start_only', 'failure_reason': None,
                   'winner_candidate': 1, 't_found_s': 3.1, 'start_conf': [0.0] * 12},
        'stage_times': {'tracked_sweep': 120.0},
        'context': {
            'joint_names_12': [f'left_ur_arm_j{i}_joint' for i in range(6)]
                              + [f'right_ur_arm_j{i}_joint' for i in range(6)],
            'world_from_mobile_base': _pose(),
            'goal': {'conf': [0.0] * 12, 'conf_authored': [0.0] * 12,
                     'bar_pose_world': _pose((1.0, 0.0, 0.9)),
                     'bar_pos_mb': [1.0, 0.0, 0.9], 'bar_quat_mb': [0, 0, 0, 1],
                     'rebranched': True, 'rebranch_max_deg': 38.2,
                     'pairing_cross_distance_deg': 45.9},
            'grasps': {'bar_from_left_tool0': _pose(), 'bar_from_right_tool0': _pose(),
                       'attach_link': 'left_ur_arm_tool0', 'tool0_from_bar': _pose()},
            'variants_mb': [['back/canonical', [0, 0, 1], [0, 0, 0, 1]]],
            'budget': {'max_time_s': 120.0, 'per_anchor_s': 40.0, 'n_variants': 39,
                       'n_deltas': 343, 'grid_step_m': 0.1, 'screen_step_m': 0.01,
                       'screen_step_rad': 0.025, 'continuity_limit_deg': 10.0},
            'attachments': [{'node': 'body__bar_B3', 'parent_link': 'left_ur_arm_tool0',
                             'link_from_body': _pose()}],
            'static_bodies': [{'node': 'body__obstacle_env2', 'world': _pose(), 'hidden': False}],
            'scene': {'glb': 'test_problem/scene.glb'},
        },
        'profile': {'t_total': 120.0, 't_track': 117.0, 'n_ik': 40495, 'n_cc': 73,
                    'anchor_allowance': 40.0, 'max_time': 120.0,
                    'budget_cuts': [['back', 'back/yaw-60', 812]]},
        'candidates': [
            {'i': 0, 'variant': 'back/canonical', 'anchor': 'back',
             'rotation': {'kind': 'canonical', 'deg': 0.0},
             'delta_m': [0, 0, 0.1], 'home_pos_mb': [0.0, 0.0, 1.2],
             'home_quat_mb': [0, 0, 0, 1], 'travel_cm': 63.0, 'travel_deg': 12.0,
             'n_total': 64, 'n_wp': 12, 'reached': 0.1746, 'blocked_at': None,
             'blocked_index': None, 'blocked_conf': None, 'arrival_conf': None,
             'last_conf': [0.1] * 12,
             'track': {'indices': [0, 11], 'confs': [[0.0] * 12, [0.1] * 12]},
             't_at_s': 0.42, 't_track_s': 0.031, 'outcome': 'track_break',
             'break': {'reason': 'branch_flip', 'joint': 9, 'jump_deg': 47.3,
                       'pose_world': _pose((0.9, 0.0, 0.95))},
             'collisions': None, 'collision_conf': None},
            {'i': 1, 'variant': 'back/roll+30', 'anchor': 'back',
             'rotation': {'kind': 'roll', 'deg': 30.0},
             'delta_m': [0, 0, 0.1], 'home_pos_mb': [0.1, -0.2, 1.2],
             'home_quat_mb': [0, 0, 0, 1], 'travel_cm': 63.0, 'travel_deg': 30.0,
             'n_total': 64, 'n_wp': 64, 'reached': 1.0, 'blocked_at': 0.33,
             'blocked_index': 21, 'blocked_conf': [0.2] * 12, 'arrival_conf': [0.3] * 12,
             'last_conf': [0.3] * 12,
             'track': {'indices': [0, 63], 'confs': [[0.0] * 12, [0.3] * 12]},
             't_at_s': 3.1, 't_track_s': 0.4, 'outcome': 'blocked', 'break': None,
             'collisions': [{'a': 'robot__left_ur_arm_forearm_link', 'b': 'body__obstacle_env2',
                             'depth_mm': 12.3, 'point': [1.0, 0.0, 0.5]}],
             'collision_conf': 'blocked'},
        ],
    }


def _latest_run():
    """The newest run file on disk, or None when none has been produced."""
    runs_dir = runs_dir_default()
    if not os.path.isdir(runs_dir):
        return None
    names = sorted(name for name in os.listdir(runs_dir) if name.endswith('.json'))
    if not names:
        return None
    with open(os.path.join(runs_dir, names[-1])) as handle:
        return json.load(handle)


def test_synthetic_run_validates():
    """A complete record passes, and a missing field is reported by name."""
    run = _synthetic_run()
    validate_run(run)

    del run['candidates'][0]['home_quat_mb']
    with pytest.raises(ValueError, match='home_quat_mb'):
        validate_run(run)


def test_wording_is_physical():
    """The sentences talk in centimetres, degrees and named parts."""
    run = _synthetic_run()

    broken = describe_candidate(run, run['candidates'][0])
    assert 'died after 11 of 63 cm' in broken
    assert 'jumped 47' in broken
    assert 'branch' in broken

    blocked = describe_candidate(run, run['candidates'][1])
    assert 'collides 21 cm into the 63 cm walk' in blocked
    assert 'left forearm against obstacle_env2' in blocked

    summary = ' '.join(describe_run(run))
    assert 'different IK branch' in summary
    assert '40 s share' in summary
    assert variant_plain('back/roll+30') == \
        'bar fore-aft over the robot, rolled +30 deg about its own axis'


def test_glb_is_valid_and_matches_its_sidecar():
    """The exported scene parses as glTF 2 and lists the nodes it claims to."""
    run = _latest_run()
    if run is None:
        pytest.skip('no run on disk; run scripts/derive_m1_headless.py --bar B3')
    problem = run['problem']
    glb_path = os.path.join(scenes_dir_default(), problem, 'scene.glb')
    nodes_path = os.path.join(scenes_dir_default(), problem, 'scene_nodes.json')
    assert os.path.isfile(glb_path), glb_path

    with open(glb_path, 'rb') as handle:
        magic, version, total = struct.unpack('<III', handle.read(12))
        assert magic == 0x46546C67 and version == 2
        assert total == os.path.getsize(glb_path)
        json_len, json_type = struct.unpack('<II', handle.read(8))
        assert json_type == 0x4E4F534A
        gltf = json.loads(handle.read(json_len).decode('utf-8'))

    named = {node['name'] for node in gltf['nodes'] if 'name' in node}
    with open(nodes_path) as handle:
        sidecar = {node['name'] for node in json.load(handle)['nodes']}
    assert named == sidecar


def test_scene_covers_every_node_the_run_references():
    """Nothing a run wants to pose is missing from the exported scene."""
    run = _latest_run()
    if run is None:
        pytest.skip('no run on disk; run scripts/derive_m1_headless.py --bar B3')
    nodes_path = os.path.join(scenes_dir_default(), run['problem'], 'scene_nodes.json')
    with open(nodes_path) as handle:
        scene_nodes = {node['name'] for node in json.load(handle)['nodes']}

    wanted = {att['node'] for att in run['context']['attachments']}
    wanted |= {body['node'] for body in run['context']['static_bodies']}
    missing = wanted - scene_nodes
    assert not missing, f'run references nodes the scene does not have: {sorted(missing)}'


def test_forward_kinematics_matches_the_planner():
    """Server-side FK reproduces the bar pose the planner measured.

    The run stores the goal bar pose as PyBullet computed it; rebuilding it from
    the URDF, the goal configuration and the recorded grasp must agree, or every
    pose in the viewer would be subtly wrong.
    """
    run = _latest_run()
    if run is None:
        pytest.skip('no run on disk; run scripts/derive_m1_headless.py --bar B3')
    from husky_assembly_teleop.cfab_session import HUSKY_DUAL_URDF_PATH

    kinematics = SceneKinematics(HUSKY_DUAL_URDF_PATH)
    context = run['context']
    world_from_mb = _matrix_from_pose(context['world_from_mobile_base'])
    links = kinematics.link_matrices(
        world_from_mb, context['goal']['conf'], context['joint_names_12'])

    attach_link = context['grasps']['attach_link']
    got = links[attach_link] @ _matrix_from_pose(context['grasps']['tool0_from_bar'])
    want = _matrix_from_pose(context['goal']['bar_pose_world'])

    position_mm = np.linalg.norm(got[:3, 3] - want[:3, 3]) * 1000.0
    cosine = (np.trace(got[:3, :3].T @ want[:3, :3]) - 1.0) / 2.0
    angle_deg = np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0)))
    assert position_mm < 1.0, f'bar position off by {position_mm:.3f} mm'
    assert angle_deg < 0.1, f'bar orientation off by {angle_deg:.3f} deg'


def test_frames_payload_is_consistent():
    """Every frame supplies exactly one matrix per animated node."""
    run = _latest_run()
    if run is None:
        pytest.skip('no run on disk; run scripts/derive_m1_headless.py --bar B3')
    from husky_assembly_teleop.cfab_session import HUSKY_DUAL_URDF_PATH

    nodes_path = os.path.join(scenes_dir_default(), run['problem'], 'scene_nodes.json')
    with open(nodes_path) as handle:
        scene_nodes = {node['name'] for node in json.load(handle)['nodes']}

    kinematics = SceneKinematics(HUSKY_DUAL_URDF_PATH)
    payload = kinematics.frames_for_candidate(run, 0, scene_nodes=scene_nodes)
    assert payload['frames'], 'a candidate must produce at least one frame'
    assert not set(payload['nodes']) - scene_nodes, 'animated node missing from the scene'
    for frame in payload['frames']:
        assert len(frame['matrices']) == len(payload['nodes'])
        assert all(len(matrix) == 16 for matrix in frame['matrices'])
    assert payload['caption']
