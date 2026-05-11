# Migrate experiment data folders to gdrive (data only; scripts stay in repo)

Status: **planned, not implemented.** Resume by reading this file, then executing the edits below.

## Context
Four experiment data folders currently live under `data/` in the git-tracked repo. Move *data files* to `/home/yijiangh/gdrive/0_projects/2025-03 Husky Assembly/data_experiment/` (gdrive-synced) while keeping *processing scripts* in the repo at their current paths. Update all read/write references in those scripts and in `husky_world.py` to point at gdrive.

User decisions confirmed:
- Hardcoded gdrive path. Add package constant `EXPERIMENT_DATA_DIRECTORY` in `husky_assembly_teleop/__init__.py`.
- User will move the data files manually after code lands. Claude only edits code.
- Add `KISSING_EXPERIMENT_DATA_DIR` constant for future use even though no current-branch code references it (the kissing branch defines `KISSING_DATA_DIR` locally — wire on merge).
- gdrive subfolder for intrinsic calib keeps the hyphen: `dual-arm_intrinsic_calibration`.

## In scope
- `bar_holding_acc_data/` — used by `husky_world.py` (writes) + 4 in-repo scripts + 1 nested script.
- `dual_arm_acc_data/` — used by `husky_world.py` + 1 in-repo script.
- `dual-arm_intrinsic_calibration/` — used by 4 in-repo scripts.
- `kissing_experiment_data/` — 5 plot scripts, all `Path(__file__).parent`-based; also adds package constant for the kissing branch's future merge.

Out of scope:
- `calibration_data/` (not requested by user; leave `CALIB_DATA_DIR` pointing at repo).
- `mocap_experiments/`.
- Anything under `external/husky_assembly_tamp/`.

## Edits

### 1. `husky_assembly_teleop/__init__.py`
Add right after the `DESIGN_DATA_DIRECTORY` line (currently line 53):
```python
EXPERIMENT_DATA_DIRECTORY = '/home/yijiangh/gdrive/0_projects/2025-03 Husky Assembly/data_experiment'
KISSING_EXPERIMENT_DATA_DIR = os.path.join(EXPERIMENT_DATA_DIRECTORY, 'kissing_experiment_data')
```

### 2. `husky_assembly_teleop/husky_world.py`
- Top-of-file import (currently line 16) — extend to include `EXPERIMENT_DATA_DIRECTORY`:
  `from husky_assembly_teleop import DATA_DIRECTORY, EXPERIMENT_DATA_DIRECTORY, CALIBRATION_DATE`
- Lines 38–39 — repoint to gdrive root:
  ```python
  BAR_HOLDING_ACC_DATA_DIR = os.path.join(EXPERIMENT_DATA_DIRECTORY, "bar_holding_acc_data")
  DUAL_ARM_ACC_DATA_DIR = os.path.join(EXPERIMENT_DATA_DIRECTORY, "dual_arm_acc_data")
  ```
- `CALIB_DATA_DIR` (line 37) and downstream uses (181, 218, 847, 940) **unchanged** — calibration is out of scope.

### 3. In-repo scripts under `data/<folder>/*.py`

These scripts cannot cleanly `import husky_assembly_teleop` (the `data/` tree isn't on `PYTHONPATH` and they're typically run as standalone). Hardcode the gdrive base — introduce a single `EXP_DATA_DIR` literal at the top of each script and route only *data* I/O through it. Keep `HERE` for non-data paths (e.g. URDF lookups via `HERE/..`).

Pattern per script:
```python
EXP_DATA_DIR = '/home/yijiangh/gdrive/0_projects/2025-03 Husky Assembly/data_experiment'
```
Then change every `os.path.join(HERE, ...)` (or `Path(__file__).parent / ...`) that resolves to a *data* file/folder so it joins under `EXP_DATA_DIR/<folder>/...`. URDF/mesh paths via `HERE/..` stay as-is.

#### bar_holding_acc_data
- `data/bar_holding_acc_data/0_bar_acc_data_processing.py`
  - L30 `data_folder = os.path.join(HERE, DATA_BATCH)` → `os.path.join(EXP_DATA_DIR, "bar_holding_acc_data", DATA_BATCH)`
  - L65 `robot_urdf` via `HERE/..` → unchanged (URDF stays in repo).
- `data/bar_holding_acc_data/1_bar_acc_stat_analysis.py`
  - L24 `data_folder = os.path.join(HERE, DATA_BATCH)` → gdrive form.
- `data/bar_holding_acc_data/2_grasp_data_analysis.py`
  - L37 `data_folder = os.path.join(HERE, DATA_BATCH)` → gdrive form.
- `data/bar_holding_acc_data/compute_robot_com.py`
  - L298 `data_folder = os.path.join(HERE, data_batch)` → gdrive form.
  - Pre-existing Windows path on L307 — leave with a one-line TODO; not in scope.
- `data/bar_holding_acc_data/20250505-openclosejaw/open-close-jaw_analysis.py`
  - L9 json_file_path, L68 angle_diff plot, L88 position_diff plot — all → `os.path.join(EXP_DATA_DIR, "bar_holding_acc_data", "20250505-openclosejaw", "<filename>")` (the script itself stays in `data/.../20250505-openclosejaw/`, but the data lives in gdrive; drop reliance on `HERE`).

#### dual_arm_acc_data
- `data/dual_arm_acc_data/0_dual_arm_acc_data_processing.py`
  - L29–32 auto-detection scans `HERE` for date-named subfolders. Redirect:
    ```python
    DUAL_ARM_ACC_DATA_DIR = os.path.join(EXP_DATA_DIR, "dual_arm_acc_data")
    candidates = [d for d in os.listdir(DUAL_ARM_ACC_DATA_DIR)
                  if os.path.isdir(os.path.join(DUAL_ARM_ACC_DATA_DIR, d)) and d.isdigit()]
    ```
  - L37 `data_folder = os.path.join(HERE, DATA_BATCH)` → `os.path.join(DUAL_ARM_ACC_DATA_DIR, DATA_BATCH)`.

#### dual-arm_intrinsic_calibration
- `data/dual-arm_intrinsic_calibration/1_dual_arm_calibration.py`
  - L262 LOG_PATH, L272 json_file_path, L342 output_file → all join under `EXP_DATA_DIR/dual-arm_intrinsic_calibration/`.
- `data/dual-arm_intrinsic_calibration/2_compute_right_arm_mount_update.py`
  - L84 log_path, L95 calibration_results_path → gdrive form.
  - L115 `urdf_file = os.path.join(HERE, '..', 'husky_urdf/...')` — URDF stays in repo, do NOT change. Leave `HERE`-relative for this single line.
- `data/dual-arm_intrinsic_calibration/3_verify_calibration.py`
  - L74 has Windows-style URDF path (`D:\…`) — pre-existing rot, leave with a one-line TODO. Not in scope.
  - L85 json_path, L148 LOG_PATH, L174 calibration_results_path, L335 output_file, L419 png save → all gdrive form.
- `data/dual-arm_intrinsic_calibration/verify_calibration.py` — same pattern as `3_verify_calibration.py`.
- `data/dual-arm_intrinsic_calibration/resources/Cali_Transformation.py`, `resources/test_original_data.py` — no relevant path refs, skip.

#### kissing_experiment_data
Six scripts: `create_force_plots.py`, `dual_create_force_plots.py`, `create_a_x_plot.py`, `create_b_x_plot.py`, `create_x_y_plot.py`, `create_offset_plots.py`. All use `base_dir = Path(__file__).parent`. Replace per file:
```python
EXP_DATA_DIR = Path('/home/yijiangh/gdrive/0_projects/2025-03 Husky Assembly/data_experiment')
base_dir = EXP_DATA_DIR / "kissing_experiment_data"
```
Keep the rest of each script's relative-glob / subfolder logic as-is (they walk `base_dir` further).

## Out-of-scope items called out (with one-line TODO comments inline)
- Windows-style absolute URDF path in `compute_robot_com.py:307` and `verify_calibration.py:74` — pre-existing dev-machine artifact.
- Missing gdrive subfolders for `dual-arm_intrinsic_calibration/` and `kissing_experiment_data/` — user creates by syncing manually.

## Verification
1. Build (from inside `ros2_ws/venv`):
   `cd /home/yijiangh/Code/ros2_ws && python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop`
2. Constant resolution:
   ```
   python -c "from husky_assembly_teleop import EXPERIMENT_DATA_DIRECTORY, KISSING_EXPERIMENT_DATA_DIR; print(EXPERIMENT_DATA_DIRECTORY); print(KISSING_EXPERIMENT_DATA_DIR)"
   python -c "from husky_assembly_teleop.husky_world import BAR_HOLDING_ACC_DATA_DIR, DUAL_ARM_ACC_DATA_DIR; print(BAR_HOLDING_ACC_DATA_DIR); print(DUAL_ARM_ACC_DATA_DIR)"
   ```
   Both `*_DATA_DIR` constants must contain the gdrive root, not `…/src/husky-assembly-teleop/data`.
3. Static path-grep sanity (post-edit):
   ```
   grep -rn "os.path.join(HERE, DATA_BATCH" data/bar_holding_acc_data/ data/dual_arm_acc_data/
   ```
   Should return zero matches.
4. End-to-end smoke (per user instruction — pause if this surfaces a non-data-path failure):
   `python src/husky-assembly-teleop/scripts/headless_constrained_monitor.py` — confirms imports still resolve and `husky_world` constants didn't break runtime startup. The headless harness doesn't actually read experiment data, so this just guards against import/typing regressions.
5. After user manually syncs data into gdrive, optionally run one in-repo script (e.g. `python data/dual_arm_acc_data/0_dual_arm_acc_data_processing.py`) and confirm it logs into the gdrive subfolder.

If verification reveals a planning/runtime failure outside the change scope, STOP — surface findings and discuss before in-lining a fix (per the project workflow rule).

## End-of-plan log (per CLAUDE.md)
Implementer should append a note in `tasks/cc_lessons.md` if a new pattern emerges (e.g. how to introduce a hardcoded gdrive constant for `data/`-resident scripts that can't import the package).
