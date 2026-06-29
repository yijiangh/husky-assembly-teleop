# 2026-06-25 — Pre-Calibration Checklist audit

## Goal
Existing §4.0 Pre-Calibration Checklist in `doc/calibration_manual.md` only had 3 items
the author noticed. Find ALL hardcoded user-changeable knobs (date, path, folder, flags,
id, ip) and consolidate into a checklist so nothing is missed before a calibration run.

## Output
- New temporary doc: `doc/pre_calibration_checklist_DRAFT.md` — superset of old §4.0
  items + newly found ones, grouped into 4 categories. Placement into the real manual
  (which chapter) decided later.

## Findings (4 categories)
1. **Per-session knobs**: `ROS_DOMAIN_ID`/`RMW_IMPLEMENTATION` (master switch, picks
   robot/namespace/mocap_id/gripper/EE), `CLIENT_IP`/`MOCAP_IP`, flags `USE_MOCAP`/
   `FAKE_HARDWARE`/`CALIBRATION`/`PUNCH_CALIB_VALIDATION`/`USE_CELL_STATE_BASE_POSE`,
   `CALIBRATION_DATE`+`DEFAULT_DATE_FOLDER` (must match), base `mocap_id`, calib tool
   IDs 1013/1012 (+ which arm enabled), `CALIBRATION_STATE_SETS`.
2. **config.yaml** (date folder): robot_name, arm, data_batches, validation_data_batch,
   punch_tool offsets, punch_validation.arm — overlaps §7.1, placement TBD.
3. **One-time per machine**: absolute `/home/su/...` paths in `__init__.py:59-60` and
   `husky_monitor.py:68-71`.
4. **Stale line numbers** in current §4.0 to fix on merge: `__init__.py` 57→67,
   `config_loader.py` 21→25, flags 73→101, calib tool 327→335.

## Key insight
Robot is now selected by the `ROS_DOMAIN_ID` env var (84/85/86) via `ROBOT_CONFIGS`
([husky_world.py:178-212]), NOT a hardcoded namespace — the old checklist never
mentioned it.

## Excluded
Internal tuning (RRT res, kissing timing, UI colors/fonts, joint/link maps) — not
per-session user knobs.

## Next
Decide which chapter each category goes into, then merge draft into
`doc/calibration_manual.md` and apply the Category-4 line-number fixes.
