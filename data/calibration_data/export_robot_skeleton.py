"""Export the husky+UR5e ARM skeleton (per calibration point) to JSON for Grasshopper.

For each take/point in `{batch}_analysis.json` (same takes/points as the outlier
exporter, so take_idx + pt_idx line up with visualize_calibration_outliers_rhino.py),
this reconstructs the robot pose AS CAPTURED and runs forward kinematics to get every
arm link's world position, then writes a light skeleton (no meshes) for the viewer.

Frame / origin: everything is in the mocap WORLD frame, whose origin IS the Motive
(OptiTrack) origin -- the same frame the calibration points use. We place the robot so its
URDF flange link lands on the measured `flange_mocap_pose` (uses only Motive captures, no
tool0_fk_pose; works for any date):
    base_from_flange  = inv(get_pose(robot)) * get_link_pose(flange_link)   # from URDF FK
    world_from_base   = flange_mocap_pose * inv(base_from_flange)
    set_pose(robot, world_from_base)  -> read each arm link pose (world)

KNOWN CAVEAT: `flange_mocap_pose` is the pose of the mocap RIGID BODY on the tool, which is
offset from the URDF flange link by a fixed unknown transform -- that offset is exactly what
the calibration pipeline solves (tool0_from_flange_mocap). Anchoring the URDF flange link
directly onto the rigid-body pose IGNORES that offset, so the arm pose is APPROXIMATE
(position/orientation off by the rigid-body offset). Good enough to eyeball the configuration;
not a calibrated placement.

Output (into the shared Drive folder EXPORT_DIR):
    {date}_{batch}_robot_skeleton.json
      { date, batch, arm, frame:"mocap_world_mm", link_order:[8 names],
        takes:[ {take_file, traj_label, take_idx,
                 points:[ {pt_idx, joint_conf:[6], link_xyz_mm:[[x,y,z]..8],
                           tool0_quat:[4]} ]} ] }

Usage:
    python export_robot_skeleton.py --batch j0|j1|both [--date 20260615] [--gui]
"""

import argparse
import json
import os
import re
import shutil

import pybullet_planning as pp

from config_loader import (
    load_config, get_robot_urdf, get_joint_names, get_tool0_link_name, HERE,
)

# ---- set these once, then just run the file (CLI flags override if you want) ----
DATA_TO_VISUALISE = '20260608'
BATCH = 'both'              # 'j0', 'j1', or 'both' (all batches in config.yaml data_batches)

# Shared Google-Drive folder (same one the outlier CSVs go to).
EXPORT_DIR = (
    "/home/su/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/"
    "2025-03 Husky Assembly/data_experiment/visualise_calibration_to_rhino"
)


def traj_label(file_name, data_batch, take_idx):
    """Short curve label like 'j0_traj1' parsed from the take file name."""
    m = re.search(r'_J(\d+)_traj(\d+)', file_name)
    if m:
        return f'j{m.group(1)}_traj{m.group(2)}'
    return f'{data_batch}_take{take_idx}'


def arm_link_names(arm):
    """Ordered UR5e arm chain: base -> tool0 (skeleton bones connect consecutive links)."""
    p = f'{arm}_ur_arm_'
    return [p + n for n in (
        'base_link_inertia', 'shoulder_link', 'upper_arm_link', 'forearm_link',
        'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'tool0',
    )]


def process_batch(robot, arm_joints, link_ids, tool0_id, flange_id, data_batch, date_folder, arm):
    """Build the skeleton dict for one batch (or None if its analysis file is missing)."""
    analysis_file = os.path.join(HERE, date_folder, data_batch, f'{data_batch}_analysis.json')
    if not os.path.exists(analysis_file):
        print(f'[{data_batch}] skip: {analysis_file} not found')
        return None

    with open(analysis_file, 'r') as f:
        analysis = json.load(f)

    takes_out = []
    n_points = 0
    for take_idx, take in enumerate(analysis['takes']):
        label = traj_label(take['file_name'], data_batch, take_idx)
        points = []
        for ri, entry in enumerate(take['raw_data']):
            flange_mocap_pose = entry.get('flange_mocap_pose')
            joint_conf = entry.get('joint_conf')
            if not flange_mocap_pose or not joint_conf or joint_conf[0] is None:
                continue

            # FK at this config, then anchor the URDF flange link onto flange_mocap_pose.
            # (Approximate -- ignores the mocap-rigid-body vs URDF-flange offset; see module docstring.)
            pp.set_joint_positions(robot, arm_joints, joint_conf)
            base_from_flange = pp.multiply(pp.invert(pp.get_pose(robot)),
                                           pp.get_link_pose(robot, flange_id))
            world_from_base = pp.multiply(flange_mocap_pose, pp.invert(base_from_flange))
            pp.set_pose(robot, world_from_base)

            link_xyz_mm = [[c * 1000.0 for c in pp.get_link_pose(robot, lid)[0]]
                           for lid in link_ids]
            tool0_quat = list(pp.get_link_pose(robot, tool0_id)[1])
            points.append({
                'pt_idx': ri,
                'joint_conf': list(joint_conf),
                'link_xyz_mm': link_xyz_mm,
                'tool0_quat': tool0_quat,
            })
        n_points += len(points)
        takes_out.append({
            'take_file': take['file_name'], 'traj_label': label,
            'take_idx': take_idx, 'points': points,
        })
        print(f'[{data_batch}] take {take_idx} ({label}): {len(points)} points')

    out = {
        'date': date_folder, 'batch': data_batch, 'arm': arm,
        'frame': 'mocap_world_mm', 'link_order': arm_link_names(arm),
        'takes': takes_out,
    }
    os.makedirs(EXPORT_DIR, exist_ok=True)
    out_path = os.path.join(EXPORT_DIR, f'{date_folder}_{data_batch}_robot_skeleton.json')
    with open(out_path, 'w') as f:
        json.dump(out, f)
    print(f'[{data_batch}] {len(takes_out)} takes, {n_points} points -> {out_path}')
    return n_points


def sync_self_to_drive():
    """Keep a copy of this script in EXPORT_DIR. No-op from a non-file path or unmounted dir."""
    try:
        src = os.path.abspath(__file__)
    except NameError:
        return
    if not os.path.isfile(src):
        return
    dest = os.path.join(EXPORT_DIR, os.path.basename(src))
    if not os.path.isdir(EXPORT_DIR) or os.path.abspath(dest) == src:
        return
    try:
        shutil.copy2(src, dest)
        print(f'[sync] copied script -> {dest}')
    except OSError as e:
        print(f'[sync] skip copy: {e}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--date', default=DATA_TO_VISUALISE)
    parser.add_argument('--batch', choices=['j0', 'j1', 'both'], default=BATCH)
    parser.add_argument('--gui', action='store_true', help='show one pose in pybullet (origin = Motive)')
    args = parser.parse_args()

    config = load_config(args.date)
    robot_name = config['robot_name']
    arm = config['arm']
    batches = config['data_batches'] if args.batch == 'both' else [args.batch]

    pp.connect(use_gui=args.gui)
    robot = pp.load_pybullet(get_robot_urdf(robot_name), fixed_base=False)
    arm_joints = pp.joints_from_names(robot, get_joint_names(robot_name, arm))
    link_ids = [pp.link_from_name(robot, n) for n in arm_link_names(arm)]
    tool0_id = pp.link_from_name(robot, get_tool0_link_name(robot_name, arm))
    flange_id = pp.link_from_name(robot, f'{arm}_ur_arm_flange')

    os.makedirs(EXPORT_DIR, exist_ok=True)
    sync_self_to_drive()
    print(f'Robot {robot_name} arm {arm} | date {args.date} | batches {batches}')
    print(f'Frame: mocap world (origin = Motive). Export dir: {EXPORT_DIR}\n')

    total = 0
    for data_batch in batches:
        n = process_batch(robot, arm_joints, link_ids, tool0_id, flange_id, data_batch, args.date, arm)
        total += n or 0
    print(f'\nDone. {total} robot poses exported.')

    if args.gui:
        input('GUI: press Enter to quit...')
    pp.disconnect()


if __name__ == '__main__':
    main()
