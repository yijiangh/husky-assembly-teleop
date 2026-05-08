# Dual-arm tracking-controller validation workflow

Goal: validate that the UR5e tracking controllers preserve the planned
**fixed relative EE transform** during execution of a constrained dual-arm
trajectory. Records mocap left/right EE poses each tick, saves JSON with a
reference TF captured at start_conf, and post-processes into jitter +
absolute-deviation metrics.

## Prerequisites

- Dual-arm Husky `a200_0806`.
- OptiTrack streaming with rigid bodies defined in Motive:
  - `left_EE` — id `4572`
  - `right_EE` — id `4573`
  (IDs hardcoded at `husky_world.py:260-264`; change there + in Motive
  together if you use different IDs.)

## Procedure

1. **Onboard workspace setup** (`~/workspace/` on the Husky NUC). Launch
   husky drivers + dual-arm controller. DDS config: ROS_DOMAIN_ID=86 +
   cyclonedds (per project convention).

2. **Start the assembly teleop monitor** with the test flags. Set in
   `husky_assembly_teleop/husky_monitor.py`:
   - line 51: `USE_MOCAP = 1`
   - line 52: `FAKE_HARDWARE = 0`
   - line 62: `DUAL_ARM_ACCURACY_TEST = 1`

   The `DUAL_ARM_ACCURACY_TEST` flag controls both marker registration
   (TrackedObjects for left_EE/right_EE) and the `Exec Arms and Record`
   button visibility — flip both with this single flag.

3. **Load antenna case-study scene state.**

4. **Click `Plan and Stage Constrained`** → planner finds:
   - free-space staging plan (current → start_conf)
   - constrained dual-arm trajectory (start_conf → goal_conf)

   Bar grasp transforms `grasp_bar_from_left/right` are derived from FK at
   `goal_conf` (or loaded from a sibling `_GraspTargets.json` if present).

5. **(Optional) Execute the staging portion first** to bring the arms to
   `start_conf` where the relative-EE constraint is satisfied. The
   reference TF captured in step 6 will then represent the
   constraint-satisfying state.

6. **Click `Exec Arms and Record`** (button at `husky_monitor.py:1756`,
   handler `world.execute_and_log_mocap` at `husky_world.py:1074`):
   - Captures reference `right_from_left` TF from the current mocap pose.
   - Sends `MultiArmTrajectory` to both UR5e arms.
   - Records mocap left/right EE poses each tick until both
     `is_arm_executing` flags clear.
   - Saves JSON to
     `data/dual_arm_acc_data/YYYYMMDD/dual_arm_acc_YYYYMMDD_HHMM_.json`
     with `raw_data` + `metadata.reference_right_from_left`.

7. **Run the analysis** from the venv:
   ```bash
   source ~/Code/ros2_ws/venv/bin/activate
   python data/dual_arm_acc_data/0_dual_arm_acc_data_processing.py
   ```
   - Defaults to the most recent date subfolder (override the
     `DATA_BATCH` constant at the top of the script to re-process older
     batches).
   - Outputs per-file PNG with **4 panels** (when metadata is present):
     1. Pos jitter (de-meaned offset norm)
     2. Rot jitter (de-meaned euler offset)
     3. Pos abs-dev vs reference
     4. Rot abs-dev vs reference (`ref^-1 * sample`, well-defined near
        wraparound)
   - Plus a `dual_arm_acc_processing_log_YYYYMMDD.txt` with mean / std /
     max for both metrics per file.

## Pass criteria (qualitative — tune per platform)

| Metric | Threshold |
| --- | --- |
| Variance-around-mean — pos std (controller jitter) | < ~2 mm |
| Variance-around-mean — rot std per axis (controller jitter) | < ~1 deg |
| Absolute deviation vs reference — pos max | < ~5 mm |
| Absolute deviation vs reference — rot max per axis | < ~2 deg |

## Failure-mode triage

- **Abs-dev grows monotonically over time** → tracker accumulating error,
  or the two arms running out of sync (one ahead/behind the other).
- **Abs-dev has a step at a specific tick** → likely a single-arm
  controller stutter at that knot. Cross-reference with the trajectory
  time-stamp.
- **Jitter rot std blows up to tens/hundreds of degrees** → Euler
  wraparound at +/-180°, not real noise. Look at the **abs-dev** rot
  panel instead (which uses `ref^-1 * sample` and stays well-defined).
- **`Could not capture reference relative EE TF; aborting`** logged →
  mocap not streaming or `left_EE`/`right_EE` rigid bodies not yet
  visible in Motive at the moment the button was clicked. Verify both
  rigid bodies are tracked and re-click.

## Code paths touched

- `husky_assembly_teleop/husky_world.py:260-264` — TrackedObject
  registration (mocap IDs).
- `husky_assembly_teleop/husky_world.py:1019-1054` —
  `record_dual_arm_E_mocap` (per-tick pose grab).
- `husky_assembly_teleop/husky_world.py:_capture_reference_relative_EE`
  (helper, ref TF capture).
- `husky_assembly_teleop/husky_world.py:execute_and_log_mocap` —
  generator, scheduled via `monitor.tasks` (`husky_monitor.py:2146-2150`).
- `husky_assembly_teleop/husky_world.py:save_dual_arm_E_mocap` — writes
  `{raw_data, metadata}`.
- `husky_assembly_teleop/husky_monitor.py:1752-1756` — button.
- `data/dual_arm_acc_data/0_dual_arm_acc_data_processing.py` —
  post-processing (jitter + abs-dev metrics, latest-batch default).
