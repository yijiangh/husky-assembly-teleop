# Robot-position visualization (step 2) + FRAMES.md — 2026-06-17

## Goal
1. Visualize the husky+UR5e ARM pose at each calibration point in Grasshopper: pick a
   curve (take) and point with sliders, see the arm skeleton — light on the PC.
2. A clear doc of how coordinate frames flow between the pipeline files, with all
   visualizations (pybullet, step-1, step-2) sharing the Motive (OptiTrack) origin.

## Files
- NEW `data/calibration_data/export_robot_skeleton.py` — pybullet FK exporter (Linux).
- NEW `data/calibration_data/robot_skeleton_viewer.py` — GH+terminal viewer (step 2).
- EDIT `data/calibration_data/rhino8_import_outliers.py` — refocused to grasshopper+terminal.
- NEW `data/calibration_data/FRAMES.md` — Mermaid frame-flow + per-file table.

## Design decisions
- Robot drawn as an **arm skeleton** (8 link-origin points + bones + tool0 frame); no meshes
  (URDF uses package:// ROS meshes not present on the GH/Windows machine).
- **All outputs in mocap world frame == Motive origin.** Capture converts Motive Y-up→Z-up
  with no translation, so origin is preserved; step-1 (`FRAME='world'`) and step-2 both emit mm.
- Placement is **calibration-free**: `world_from_base = tool0_fk_pose ∘ inv(URDF FK base→tool0)`,
  so tool0 lands exactly on the captured `tool0_fk_pose`. Works for ANY date (incl. 20260615,
  which has no `calibrated_transformation_*.json`). Mirrors `3_verify_calibration.py:100-107`.

## export_robot_skeleton.py
- Top constants `DATA_TO_VISUALISE`, `BATCH`, `EXPORT_DIR` (+ CLI `--date/--batch/--gui`).
- Loads URDF once headless (`pp.connect(use_gui=False)`), reuses `config_loader`
  (`get_robot_urdf/get_joint_names/get_tool0_link_name`), reads `{batch}_analysis.json` (so
  take_idx + pt_idx match step-1).
- Per point: set joints → `base_from_tool0 = inv(get_pose(robot))*get_link_pose(tool0)` →
  `world_from_base = tool0_fk_pose * inv(base_from_tool0)` → set_pose → read 8 arm link xyz (mm).
- Writes `{date}_{batch}_robot_skeleton.json` to the Drive folder:
  `{date,batch,arm,frame,link_order[8], takes:[{take_file,traj_label,take_idx,
   points:[{pt_idx,joint_conf[6],link_xyz_mm[8],tool0_quat[4]}]}]}`.

## robot_skeleton_viewer.py (step 2)
- Pure stdlib (json) → runs in Rhino8 GhPython (CPython) AND terminal; no pybullet.
- `RUN_MODE` grasshopper|terminal; `TAKE_IDX`/`POINT_IDX` (GH input params `TAKE`/`POINT`
  override via globals); indices clamped to range.
- grasshopper: assigns outputs `joints` (Point3d), `bones` (Line), `tool0_plane` (Plane from
  tool0 pos+quat), `info` (str). terminal: prints take/point, joint_conf, 8 link xyz.

## rhino8_import_outliers.py (step 1, refocused)
- `RUN_MODE` values renamed `rhino`→`grasshopper`. Dropped Rhino-bake `draw_rhino` +
  `LABEL_TEXT_SIZE`. New `build_grasshopper()` assigns outputs `circles` (Circle),
  `circle_labels`, `points` (Point3d), `point_labels`, `origin` (Point3d 0,0,0).
- terminal mode + label/threshold/date-time logic + `sync_self_to_drive` unchanged.

## Verification (env: source /home/su/ros2_ws/venv/bin/activate) — all passed
1. FK export 20260615 j0 → 9 takes, 279 poses; tool0 link == captured `tool0_fk_pose` (diff
   0.0 mm); UR5e bone lengths correct (upper arm 424.9 mm). 20260608 both → 310 poses.
2. Viewer terminal: prints selected take/point + 8 link xyz; clamps overshoot (point 999→30);
   loads both 20260615 j0 and 20260608 j1 skeletons.
3. Step-1 terminal mode still prints circles+outliers (with date/time labels) after refactor.
4. FRAMES.md written (Mermaid + table) with verified file:line refs.
5. GH (manual, user): paste viewers into Rhino8 GhPython CPython components, wire sliders,
   confirm skeleton + tool0 frame draw and overlay step-1 circles/points at the same origin.

## Notes
- Step-1 points (flange trace) and step-2 tool0 do NOT coincide — tool0 is offset from the
  flange by the calibration tool length; they share the origin and frame, which is the point.
- Rhino.Geometry "could not resolve" warnings are expected (only resolves inside Rhino);
  terminal paths never import it.
