"""Stage calibration analysis data to the shared Drive folder for the Grasshopper viewer.

This no longer generates a derived CSV. The viewer (rhino8_import_outliers.py) reads the
raw data directly, so all this does is COPY each batch's `{batch}_analysis.json` into the
shared Drive folder under a dated subfolder:

    {date}/{batch}/{batch}_analysis.json
      -> EXPORT_DIR/{date}/{batch}_analysis.json

`{batch}_analysis.json` (output of 0_circle_fitting.py) already contains, per take, the
UNCHANGED raw_data samples (base_mocap_pose, flange_mocap_pose, joint_conf, ...) PLUS the
fitted circle (center, normal) in the base_mocap frame. The viewer reuses that fit (no
re-fitting) and re-expresses everything in the Motive world frame.

Usage:
    python visualize_calibration_outliers_rhino.py --batch j0|j1|both [--date 20260615]
"""

import argparse
import os
import shutil

from config_loader import load_config, HERE

# ---- set these once, then just run the file (CLI flags override) ----
DATA_TO_VISUALISE = '20260615'
BATCH = 'both'              # 'j0', 'j1', or 'both' (all batches in config.yaml data_batches)

# Shared Google-Drive folder; analysis files land in a dated subfolder under here.
EXPORT_DIR = (
    "/home/su/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/"
    "2025-03 Husky Assembly/data_experiment/visualise_calibration_to_rhino"
)


def stage_batch(data_batch, date_folder):
    """Copy {date}/{batch}/{batch}_analysis.json into EXPORT_DIR/{date}/. Returns dest or None."""
    src = os.path.join(HERE, date_folder, data_batch, f'{data_batch}_analysis.json')
    if not os.path.exists(src):
        print(f'[{data_batch}] skip: {src} not found')
        return None
    dest_dir = os.path.join(EXPORT_DIR, date_folder)
    os.makedirs(dest_dir, exist_ok=True)
    dest = os.path.join(dest_dir, f'{data_batch}_analysis.json')
    shutil.copy2(src, dest)
    print(f'[{data_batch}] staged -> {dest}')
    return dest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--date', default=DATA_TO_VISUALISE)
    parser.add_argument('--batch', choices=['j0', 'j1', 'both'], default=BATCH)
    args = parser.parse_args()

    config = load_config(args.date)
    batches = config['data_batches'] if args.batch == 'both' else [args.batch]

    print(f'Staging date {args.date} | batches {batches}')
    print(f'Drive dir: {EXPORT_DIR}/{args.date}\n')

    n = sum(stage_batch(b, args.date) is not None for b in batches)
    print(f'\nDone. {n} analysis file(s) staged.')


if __name__ == '__main__':
    main()
