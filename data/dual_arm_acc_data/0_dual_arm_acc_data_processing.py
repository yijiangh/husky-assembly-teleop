# Analyze dual-arm EE mocap recordings produced by husky_world.execute_and_log_mocap.
#
# Per recording, computes two complementary metrics:
#   1. Variance-around-mean (controller jitter): de-meaned pos/rot offsets.
#   2. Absolute deviation vs reference (planner-vs-tracker bias): only when the
#      JSON includes metadata.reference_right_from_left (saved at start_conf
#      before execution begins).
#
# Backward compatible with old JSONs that have no metadata block: only metric (1)
# is reported in that case.

import json, os
import logging, datetime
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import numpy as np
import pybullet_planning as pp

# Override to re-process an older batch:
DATA_BATCH = '20260514'
# DATA_BATCH = None
EXPORT = 1


HERE = os.path.dirname(os.path.abspath(__file__))

if DATA_BATCH is None:
    subfolders = sorted(
        d for d in os.listdir(HERE)
        if os.path.isdir(os.path.join(HERE, d)) and d.isdigit()
    )
    if not subfolders:
        raise RuntimeError(f"No date subfolders under {HERE}")
    DATA_BATCH = subfolders[-1]
    print(f"DATA_BATCH not set; defaulting to latest: {DATA_BATCH}")

data_folder = os.path.join(HERE, DATA_BATCH)

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger()
file_handler = logging.FileHandler(os.path.join(data_folder, f'dual_arm_acc_processing_log_{DATA_BATCH}.txt'), mode='w')
file_handler.setLevel(logging.INFO)
file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
logger.addHandler(file_handler)

json_files = [f for f in os.listdir(data_folder) if f.startswith('dual_arm_acc_') and f.endswith('.json')]
json_files.sort()
print(json_files)

for i, file_name in enumerate(json_files):
    logger.info('Working on file: %s', file_name)
    file_path = os.path.join(data_folder, file_name)

    with open(file_path, 'r') as file:
        data = json.load(file)

    metadata = data.get('metadata', {}) or {}
    ref_rel = metadata.get('reference_right_from_left')  # [pos, quat] or None

    # Per-sample relative TF: right_from_left
    rel_pos_data = []          # 3-vec per sample (right_from_left position)
    rel_rot_data_quat = []     # quat per sample

    for entry in data['raw_data']:
        L_pose = entry.get('left_EE_pose', [])
        Rp_pose = entry.get('right_EE_pose', [])
        L = (tuple(L_pose[0]), tuple(L_pose[1]))
        Rp = (tuple(Rp_pose[0]), tuple(Rp_pose[1]))
        rel = pp.multiply(pp.invert(Rp), L)
        rel_pos_data.append(np.array(rel[0]))
        rel_rot_data_quat.append(np.array(rel[1]))

    rel_pos_data = np.array(rel_pos_data)
    rel_pos_norm = np.linalg.norm(rel_pos_data, axis=1)

    rel_rot_objs = R.from_quat(np.array(rel_rot_data_quat))

    # ----- Metric 1: variance-around-mean (controller jitter) -----
    # Position: norm of rel_pos minus its mean (1D scalar vs sample idx).
    mean_pos_norm = float(np.mean(rel_pos_norm))
    std_pos_norm = float(np.std(rel_pos_norm))
    pos_jitter = rel_pos_norm - mean_pos_norm
    pos_jitter_range = float(np.max(np.abs(pos_jitter)))

    # Rotation: euler offset against the per-file mean rotation.
    rel_rot_euler = rel_rot_objs.as_euler('xyz', degrees=True)
    mean_rot_euler = np.mean(rel_rot_euler, axis=0)
    std_rot_euler = np.std(rel_rot_euler, axis=0)
    rot_jitter = rel_rot_euler - mean_rot_euler
    rot_jitter_range = float(np.max(np.abs(rot_jitter)))

    logger.info('[Jitter] Mean pos offset norm: %f m', mean_pos_norm)
    logger.info('[Jitter] Std pos offset norm: %f m   range +/-%f m', std_pos_norm, pos_jitter_range)
    logger.info('[Jitter] Mean rot euler (deg): %f %f %f', *mean_rot_euler)
    logger.info('[Jitter] Std rot euler  (deg): %f %f %f   range +/-%f deg',
                *std_rot_euler, rot_jitter_range)

    # ----- Metric 2: absolute deviation vs reference (when available) -----
    has_ref = ref_rel is not None
    if has_ref:
        ref_pos = np.array(ref_rel[0])
        ref_rot = R.from_quat(np.array(ref_rel[1]))
        # Position deviation: euclidean distance between sample rel_pos and ref_pos
        abs_pos_dev = np.linalg.norm(rel_pos_data - ref_pos, axis=1)
        # Rotation deviation: relative rotation (ref^-1 * sample) as euler
        abs_rot_dev_obj = ref_rot.inv() * rel_rot_objs
        abs_rot_dev = abs_rot_dev_obj.as_euler('xyz', degrees=True)

        logger.info('[AbsDev] Mean pos dev: %f m   max: %f m',
                    float(np.mean(abs_pos_dev)), float(np.max(abs_pos_dev)))
        logger.info('[AbsDev] Mean rot dev (deg): %f %f %f', *np.mean(abs_rot_dev, axis=0))
        logger.info('[AbsDev] Max  rot dev (deg, abs per axis): %f %f %f',
                    *np.max(np.abs(abs_rot_dev), axis=0))
    else:
        logger.info('[AbsDev] No metadata.reference_right_from_left in JSON; skipped.')

    # ----- Plot -----
    n_panels = 4 if has_ref else 2
    fig = plt.figure(figsize=(8, 2.6 * n_panels))

    ax = fig.add_subplot(n_panels, 1, 1)
    ax.plot(pos_jitter)
    ax.set_title(f'[Jitter] Pos offset (mu={mean_pos_norm:.5f} std={std_pos_norm:.5f} r=+/-{pos_jitter_range:.5f}) [m]')
    ax.set_xlabel('Sample idx'); ax.set_ylabel('Offset [m]'); ax.grid(True)

    ax = fig.add_subplot(n_panels, 1, 2)
    ax.plot(rot_jitter)
    ax.set_title(
        f'[Jitter] Rot offset\n'
        f'mu=[{mean_rot_euler[0]:.4f}, {mean_rot_euler[1]:.4f}, {mean_rot_euler[2]:.4f}] '
        f'std=[{std_rot_euler[0]:.4f}, {std_rot_euler[1]:.4f}, {std_rot_euler[2]:.4f}] '
        f'r=+/-{rot_jitter_range:.4f} [deg]'
    )
    ax.set_xlabel('Sample idx'); ax.set_ylabel('Offset [deg]'); ax.grid(True)
    ax.legend(['X', 'Y', 'Z'])

    if has_ref:
        ax = fig.add_subplot(n_panels, 1, 3)
        ax.plot(abs_pos_dev)
        ax.set_title(
            f'[AbsDev vs ref] Pos dev '
            f'(mean={np.mean(abs_pos_dev):.5f} max={np.max(abs_pos_dev):.5f}) [m]'
        )
        ax.set_xlabel('Sample idx'); ax.set_ylabel('|rel_pos - ref_pos| [m]'); ax.grid(True)

        ax = fig.add_subplot(n_panels, 1, 4)
        ax.plot(abs_rot_dev)
        ax.set_title('[AbsDev vs ref] Rot dev (ref^-1 * sample) [deg]')
        ax.set_xlabel('Sample idx'); ax.set_ylabel('Dev [deg]'); ax.grid(True)
        ax.legend(['X', 'Y', 'Z'])

    plt.tight_layout()

    if EXPORT:
        out_path = os.path.join(data_folder, f'dual_arm_acc_{DATA_BATCH}_{i+1}.png')
        plt.savefig(out_path)
        logger.info('Exported plot to %s', out_path)

# plt.show()
