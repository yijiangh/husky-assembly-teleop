# Calibration outlier export/import upgrade — 2026-06-17

## Goal
Make the calibration-outlier tooling easier to share and debug:
- export CSVs to a shared Google-Drive folder with date-prefixed names
- adaptive (per-trajectory mean) threshold, selectable vs a fixed number
- choose which batch (j0/j1/both) to export
- import script runnable in Rhino/Grasshopper OR terminal (VSCode-safe), with
  controllable point labels.

## Files changed
- `data/calibration_data/visualize_calibration_outliers_rhino.py` (exporter)
- `data/calibration_data/rhino8_import_outliers.py` (importer / debug)

## Exporter (`visualize_calibration_outliers_rhino.py`)
Set-once constants at top (CLI flags override):
- `DATA_TO_VISUALISE` (date), `BATCH` = `'j0'|'j1'|'both'`, `THRESHOLD` = `'mean'` or a number(mm).
- `EXPORT_DIR` = shared Drive folder
  `.../2025-03 Husky Assembly/data_experiment/visualise_calibration_to_rhino`.

Behavior:
- Writes `{date}_{batch}_rhino_outliers.csv` and `{date}_{batch}_rhino_circles.csv` into
  `EXPORT_DIR` (flat; `os.makedirs(..., exist_ok=True)`). CSV columns unchanged.
- Threshold per take: `mean` → cutoff = that trajectory's own mean projection error;
  number → fixed cutoff for all takes. A point is flagged when `err_mm > cutoff`.
- Per-take print: `[j0] take 0 (j0_traj1): cutoff 0.44 mm (mean), 9/31 flagged`.
- CLI: `--date`, `--batch {j0,j1,both}`, `--threshold mean|<mm>` (all default to the constants).
- `--batch both` iterates `config['data_batches']`; missing batch analysis files are skipped
  with a message (existing behavior).

## Importer (`rhino8_import_outliers.py`)
Set-once constants at top:
- `RUN_MODE` = `'rhino'` (draw) | `'terminal'` (print here, no Rhino import → safe under the
  VSCode play button / `python` on Linux/Windows).
- `THRESHOLD_MM` — hide points with `err_mm` below it (0 = show all exported).
- `SHOW_ERROR_DISTANCE` — append `  <err>mm` to the basic `jX_tY #Z` label.
- `LABEL_TEXT_SIZE` — text-dot font height (rhino mode).
- `POINT_LABELS` — label points at all.
- `OUTLIERS_CSV` / `CIRCLES_CSV` default to the shared `EXPORT_DIR` with date-prefixed names.

Behavior:
- `point_label(row)`: basic `jX_tY #Z` (`traj` → `t`); optional `+err mm`. Used by both modes.
- Rhino imports (`Rhino`, `Rhino.Geometry`, `scriptcontext`, `System.Drawing`,
  `rhinoscriptsyntax`) live inside `draw_rhino()` only — terminal mode never imports them.
- Rhino mode: circles + outlier points colored per trajectory; labels via
  `Rhino.Geometry.TextDot` with `.FontHeight = LABEL_TEXT_SIZE`.
- Terminal mode: ASCII-only structured summary — circles (radius/center) then outliers grouped
  by trajectory, each line shows the label + x/y/z mm. Honors `THRESHOLD_MM`.

## Verification (date 20260615, j0 only) — all passed
Env: `source /home/su/ros2_ws/venv/bin/activate` (note: this machine is `/home/su`, not the
`/home/yijiangh` path in CLAUDE.md).
1. `--batch j0` (mean): 9 takes, per-take cutoffs ~0.4–0.6 mm, 107 flagged; both CSVs written to EXPORT_DIR.
2. `--batch j0 --threshold 1.0` (fixed): cutoff `1.00 mm (fixed)`, 27 flagged.
3. `--batch both`: iterates [j0, j1]; j1 skipped (no analysis json).
4. `--threshold foo`: argparse error.
5. Importer terminal mode: prints circles + 107 outliers with distances, no Rhino import error.
   `THRESHOLD_MM=1.5` → 7 points; `SHOW_ERROR_DISTANCE=False` → distance suffix removed.
6. Rhino mode: manual (run in Rhino8 ScriptEditor) — not testable headless.

## Follow-up (same day)
### Exporter: output frame = Motive world (request)
- Added `FRAME = 'world' | 'base'` constant (default `'world'`).
- Circle fit + per-point errors are still computed in the **base frame** (flagging unchanged);
  only exported coordinates are transformed. `'world'` maps geometry into the Motive/mocap
  world via a fixed reference base pose (first sample of the take) using new helper
  `transform_point(base_pose, p)` (`pp.point_from_pose(pp.multiply(base, (p, identity)))`),
  so the circle stays clean (base drift removed) but lands at its true Motive location and
  Rhino/terminal `(0,0,0)` == the Motive origin. `'base'` keeps the old robot-base origin.
- Verified: centers move from base-frame (e.g. -139,308,223) to world (e.g. -171,-1002,655),
  radius preserved (551.0), flagged counts identical (107). World point ≈ raw flange
  (diff ~= the <1mm base drift), confirming correctness.

### Importer: optional date/time labels (request)
- `SHOW_DATE` / `SHOW_TIME` constants (default False). Date/time parsed from `take_file`
  (`calibration_<date>_<time>_...`) via `_TAKE_RE` + `_date_time(row)`.
- New `label_prefix(row)` = optional `date time` + `jX_tY`; shared by points and curves
  (`point_label` builds on it; circles use it in both rhino + terminal modes). `jX_tY` and the
  point `#index` are always shown; date/time/error are the toggleable extras. This disambiguates
  curves that share a `jX_tY` but were recorded at different times (e.g. several `j0_t5`).
- Verified in terminal mode: circle + point labels show `20260615 1329 j0_t1 #20 ...`.

## Notes
- Rhino import "could not be resolved" warnings in the editor are expected (modules only exist
  inside Rhino); terminal mode avoids them by importing lazily inside `draw_rhino()`.
- Windows/Grasshopper path edit: use raw strings and don't end a line with `\` (a trailing
  `\"` escapes the quote -> "EOL while scanning string literal").
