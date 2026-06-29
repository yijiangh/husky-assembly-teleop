# 2026-06-24 — Rhino8 visualization manual

## Goal
Document the two Rhino8/Grasshopper visualization workflows end-to-end, with explicit
frames/origins, in `doc/visualization_manual.md` (alongside `calibration_manual.md`).
Documentation only — no code changes.

## Deliverable
- `doc/visualization_manual.md` — new manual.

## Contents
1. Overview & coordinate frames — the shared frame: **Motive origin, Z-up "rhino"
   (x,−z,y), millimeter Rhino document**. Why both workflows land there (calibration
   poses converted at capture in `husky_monitor.receive_rigid_body_frame`; cameras
   converted by the exporter).
2. Workflow 1 — cameras: `collect cameras data` GUI button OR `export_mocap_cameras.py`
   → `*_cameras.json/.csv` (meters) → `import_mocap_cameras_rhino.py` (`UNIT_SCALE=1000`
   for mm) → point + RGB axes + gray −Z view line + name dot.
3. Workflow 2 — calibration: pipeline `0_circle_fitting.py` → `{batch}_analysis.json`;
   stage with `visualize_calibration_outliers_rhino.py`; import points+circles with
   `rhino8_import_outliers.py` (error distance to fitted circle; `THRESHOLD` mean/number
   selects outliers); robot config via `export_robot_skeleton.py` +
   `robot_skeleton_viewer.py` with `TAKE`/`POINT` sliders. Skeleton placement is
   approximate (ignores rigid-body↔flange offset).
4. Frames & origins reference table.
5. Troubleshooting + disambiguation: `convert_to_rhino.py` is a runtime calibration
   fixer, NOT a Rhino importer.

## Verification (headless, from venv)
- `visualize_calibration_outliers_rhino.py --batch both --date 20260622` → staged 2 files.
- `export_robot_skeleton.py --batch j0 --date 20260622` → 186 poses exported.
- `rhino8_import_outliers.py` (RUN_MODE=terminal, temp copy) → 6 circles, 87 outliers.
- `robot_skeleton_viewer.py` (RUN_MODE=terminal, temp copy) → take/point joints + link xyz.
- `export_mocap_cameras.py` (validated earlier same session) → 21 cameras.
All real-output snippets in the manual come from these runs.

## Notes
- `20260615/j0` is currently empty; used `20260622` as the example date.
- Two Drive folders: `visualise_mocap_camera` (cameras), `visualise_calibration_to_rhino`
  (calibration).
