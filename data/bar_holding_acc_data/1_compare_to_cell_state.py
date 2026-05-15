"""Compare observed bar pose (from mocap) to the BarAction cell-state goal.

For each marker take in a batch folder:
1. Read the top-level `bar_action_path` + `movement_id` stamped by the monitor.
2. Load the BarAction; find the matching movement.
3. Compute the goal bar world pose from the movement's `target_ee_frames`
   composed with the bar's `attachment_frame`.
4. Run fit_bar_from_markerset on the take; compare via bar_deviation_from_goal.
5. Print per-take deviations and aggregate stats.
"""

import json
import os
import sys
import argparse

import numpy as np
import pybullet_planning as pp

from husky_assembly_teleop import EXPERIMENT_DATA_DIRECTORY
from husky_assembly_teleop.utils import pose_from_frame
from husky_assembly_teleop.bar_action_io import parse_bar_action, find_movement
from husky_assembly_teleop.mocap_experiment import (
    fit_bar_from_markerset,
    bar_deviation_from_goal,
)


def goal_bar_pose_from_movement(mv, bar_name):
    """world_from_bar = world_from_tool0 (target_ee_frames) ∘ tool0_from_bar (attachment_frame)."""
    bar_rb = mv.start_state.rigid_body_states[bar_name]
    attached_link = bar_rb.attached_to_link
    side = 'left' if 'left' in attached_link else 'right'
    target = mv.target_ee_frames[side]
    world_from_tool = pose_from_frame(target)
    tool_from_bar = pose_from_frame(bar_rb.attachment_frame)
    return pp.multiply(world_from_tool, tool_from_bar), side


def _make_corrector(saved_convention):
    if saved_convention == 'rhino':
        return lambda p: list(p)
    if saved_convention == 'rotated':
        return lambda p: [p[1], -p[0], p[2]]
    raise ValueError(f"unknown mocap_axis_convention {saved_convention!r}")


def _convert_markerset(labeled_marker_dict, correct):
    out = {}
    for mid, info in labeled_marker_dict.items():
        new_info = dict(info)
        new_info['pos'] = correct(info['pos'])
        out[mid] = new_info
    return out


def process_file(file_path, override_bar_action=None, override_movement=None):
    with open(file_path, 'r') as f:
        data = json.load(f)

    saved_convention = data.get('mocap_axis_convention', 'rotated')
    correct = _make_corrector(saved_convention)

    bar_action_path = override_bar_action or data.get('bar_action_path')
    movement_id = override_movement or data.get('movement_id')
    movement_index = data.get('movement_index')

    if not bar_action_path:
        print(f"  SKIP {os.path.basename(file_path)}: no bar_action_path in JSON")
        return []
    if not os.path.isabs(bar_action_path):
        print(f"  SKIP {os.path.basename(file_path)}: bar_action_path not absolute: {bar_action_path}")
        return []
    if not os.path.exists(bar_action_path):
        print(f"  SKIP {os.path.basename(file_path)}: bar_action_path not found: {bar_action_path}")
        return []

    action = parse_bar_action(bar_action_path)
    key = movement_id if movement_id else (movement_index if movement_index is not None else 2)
    idx, mv = find_movement(action, key)
    bar_name = f"bar_{action.active_bar_id}"

    goal_bar_pose, side = goal_bar_pose_from_movement(mv, bar_name)

    print(f"\n=== {os.path.basename(file_path)} (mocap_axis_convention={saved_convention}) ===")
    print(f"  action={os.path.basename(bar_action_path)} | mv[{idx}]={mv.movement_id} | bar={bar_name} | held_by={side}")
    print(f"  goal_bar_pos={[round(v,4) for v in goal_bar_pose[0]]}")

    rows = []
    for i, entry in enumerate(data['raw_data']):
        marker_pts_saved = entry.get('bar_rig', {})
        marker_pts = _convert_markerset(marker_pts_saved, correct)
        try:
            fit = fit_bar_from_markerset(marker_pts)
        except Exception as e:
            print(f"  take {i}: fit failed ({e})")
            continue
        dev = bar_deviation_from_goal(fit, goal_bar_pose)
        ocf = fit['ocf_position']
        print(
            f"  take {i}: "
            f"pos_dev={dev['pos_dev_m']*1000:.2f} mm | "
            f"angle_dev={np.rad2deg(dev['angle_rad']):.3f} deg | "
            f"lateral_dev={dev['lateral_dev_m']*1000:.2f} mm | "
            f"center_to_line_dist_max={fit['center_to_line_dist_max_m']*1000:.2f} mm | "
            f"ocf=({ocf[0]:.4f}, {ocf[1]:.4f}, {ocf[2]:.4f})"
        )
        rows.append({
            'source_file': os.path.basename(file_path),
            'take_index': i,
            'bar_action_path': bar_action_path,
            'movement_id': mv.movement_id,
            'goal_bar_position': list(goal_bar_pose[0]),
            'goal_bar_quaternion': list(goal_bar_pose[1]),
            'ocf_position': fit['ocf_position'],
            'fitted_line': fit['fitted_line'],
            'bar_end_points': fit['bar_end_points'],
            'bar_length_observed': fit['bar_length_observed'],
            'center_to_line_dist_max_m': fit['center_to_line_dist_max_m'],
            'center_to_line_dist_rms_m': fit['center_to_line_dist_rms_m'],
            'pos_dev_m': dev['pos_dev_m'],
            'angle_dev_rad': dev['angle_rad'],
            'lateral_dev_m': dev['lateral_dev_m'],
        })

    return rows


def aggregate(rows):
    if not rows:
        print("\nno valid takes")
        return
    pos = np.array([r['pos_dev_m'] for r in rows]) * 1000
    ang = np.rad2deg([r['angle_dev_rad'] for r in rows])
    lat = np.array([r['lateral_dev_m'] for r in rows]) * 1000
    st = np.array([r['center_to_line_dist_max_m'] for r in rows]) * 1000
    print(f"\n=== aggregate over {len(rows)} takes ===")
    print(f"  pos_dev_mm:        mean={pos.mean():.2f}  std={pos.std():.2f}  max={pos.max():.2f}")
    print(f"  angle_dev_deg:     mean={ang.mean():.3f}  std={ang.std():.3f}  max={ang.max():.3f}")
    print(f"  lateral_dev_mm:    mean={lat.mean():.2f}  std={lat.std():.2f}  max={lat.max():.2f}")
    print(f"  center_to_line_dist_max_mm: mean={st.mean():.2f}  std={st.std():.2f}  max={st.max():.2f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('batch', help='batch folder name under EXPERIMENT_DATA_DIRECTORY/bar_holding_acc_data/')
    parser.add_argument('--bar-action', default=None, help='override stamped bar_action_path (absolute)')
    parser.add_argument('--movement', default=None, help='override movement_id (e.g. M2)')
    parser.add_argument('--export', action='store_true', help='dump compared_<batch>.json')
    args = parser.parse_args()

    data_folder = os.path.join(EXPERIMENT_DATA_DIRECTORY, 'bar_holding_acc_data', args.batch)
    if not os.path.isdir(data_folder):
        sys.exit(f"data folder not found: {data_folder}")

    files = sorted(
        os.path.join(data_folder, f) for f in os.listdir(data_folder)
        if f.startswith('bar_holding_acc_') and f.endswith('.json')
    )
    if not files:
        sys.exit(f"no bar_holding_acc_*.json files in {data_folder}")

    all_rows = []
    for fp in files:
        all_rows.extend(process_file(fp, args.bar_action, args.movement))
    aggregate(all_rows)

    if args.export:
        out_path = os.path.join(data_folder, 'compared_to_cell_state.json')
        with open(out_path, 'w') as f:
            json.dump(all_rows, f, indent=4)
        print(f"\nexported to {out_path}")


if __name__ == '__main__':
    main()
