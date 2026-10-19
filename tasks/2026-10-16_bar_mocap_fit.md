# Bar 6-DoF pose from MoCap joint markers (husky-assembly-teleop)

Written 2026-10-16 in the Rhino design repo (`bar_joint_rhino_design_workflow`,
`tasks/mocap_bar_pose_monitor_handover.md`), copied here and **refined the same day against this
repo** (HEAD `ede985c`). The refinements are listed in the next section and folded into B1–B6;
where the two disagree, this file wins. The user's test of this round is
`doc/mocap_bar_pose_test_manual.md`.

## Refinements after checking this repo (2026-10-16)

**Data.** `261016_2ModulesDemo_YH/MoCapMarkers.json` was re-exported with schema 2 at 15:21:
`bar_B20` and `bar_B21` with 8 markers each (`nearest_miss_m` 0.0483 for both), 31 bars in
`bars_without_markers`. Checked: the joint frame in the first state of `B20__J.json` /
`B20__R.json` times the block point agrees with `bar_m` within 0.007 mm, and B20 rests (not
attached) in both states at its assembled pose (1.326, -2.278, 0.150).

**What the hand-over assumed about this repo, and what is there:**

| hand-over | this repo | this round |
|---|---|---|
| a new `self._mocap_cache_lock` | already exists (`husky_monitor.py:489`), used only for the rigid-body cache | reuse it for the marker cache |
| `receive_labeled_marker` (`:12302`) | skips model 0, keeps markers that dropped out (never cleared, no time, no lock) | as B2 says: `'_unlabeled'` key, a fresh dict per model per frame, a time stamp |
| listener gated on `BAR_ACTION_MOCAP_ACCURACY_TEST` (`:12197`) | correct | register whenever `USE_MOCAP` |
| read `rigid_body_list` | never read by the monitor; `RigidBodyDescription.sz_name` is **bytes**, the id is `id_num` (`optitrack/DataDescriptions.py:168`) | decode the name, map `id_num` |
| marker `size` is a float | NatNet hands a 1-tuple (`NatNetClient.py:547`); saved takes hold `[0.0188]` | accept a number or a 1-element list |
| designed pose via `get_movement_start_bar_pose` + `Transformation.from_frame` | returns `(pos, quat)`; for a held bar it uses the **previous** movement's EE targets and raises without one. The package uses `pose_from_frame` (`utils.py:108`) and `.get` on `rigid_body_states` | the bar's **resting frame** in the loaded action (B2). The held-bar reference waits for B4 |
| "attached to another robot" | no such field | attached = `attached_to_link or attached_to_tool` |
| buttons via `add_button` | `self.buttons.append(Button(label, cb))`, `Toggle(label, cb, default)` (`husky_monitor.py:11752-11762`) | use those |
| draw like `_show_assembled_bar_ghost` | a debug **line**; its docstring says run-time bodies do not show, which predates `PYBULLET_GUI_MESH_BUFFER_MB = 512` (8cd0e1b) | see-through mesh ghosts of the bar **and its joints**, plus debug lines that always render |
| — | no "joints of a bar" helper | `bar_action_io.joints_on_bar` (name rule) |
| B20 joints at 75 / 430 / 1135 / 1670 mm | those are the joint origins; the markers sit at 84.9 / 420 / 1145.3 / 1680 mm (±10 mm plate offset) | tests use the real `bar_m` table |

**Scope of this round:** B1, B2 (with the by-eye overlay), the save fields of B3, B5, and the
manual. **Next round:** the offline refit script of B3, and B4.

**Decided while implementing (2026-10-16):**
- **Seen from Motive (2026-10-16, laptop on the mocap network).**
  - NatNet 4.0, so open question 1 is answered: marker names and residuals stream.
  - Motive names the bar assets by bar id: `B20` = 1020, `B21` = 1021, `B34` = 1034, `B35` = 1035.
  - B20 streamed its 8 balls, none occluded, plus 14 unlabelled markers in the volume.
- **This PC's address on the mocap network.** The env var `MOCAP_CLIENT_IP` overrides
  `CLIENT_IP` in `husky_monitor.py`. The manual's PC block reads it off the port on `192.168.0.x`.
- **Streaming IDs.** Lab convention: bar Bxx's Motive asset streams as 10xx (B20 = 1020,
  `husky_world.mocap_bar_streaming_id`). The order is an asset named like the bar (`B20` or
  `bar_B20`), then `MOCAP_BAR_STREAMING_IDS`, then the convention. The ids live in their own map
  (`_mocap_bar_asset_from_id`), not in `name_from_mocap_id`. Motive's asset list is read at
  connect only, so a new asset needs a monitor restart.
- **Occluded markers.** A hidden ball of a tracked asset still streams, flagged occluded, at
  Motive's guess. `NatNetClient` now passes `flags` to the listener, and `fit_bar_from_cache`
  drops occluded markers. The old `bar_rig` flow is unchanged.
- **Threads and drawing.**
  - `measure_bar_pose` holds a re-entrant class-level lock: buttons run on the panel thread,
    'follow live' on the tick.
  - The overlay is drawn under `pp.LockRenderer()`. In the GUI one redraw took ~100 ms without
    it and ~1 ms with it; the fit itself takes ~20 ms.
- **Matcher on the real B20.**
  - With both J20-169 balls hidden, a second assignment fits the other 6 within ~2 mm, so the
    fit says "ambiguous". Refusing is correct there.
  - With 7 or 8 balls in view, the best wrong assignment matches only 6.
  - A mirrored B20 set would pass at rms ~2.3 mm: the layout is nearly mirror-symmetric. Both
    axis conventions are proper rotations, so no code path produces a mirror.
- **B5 case 2 thresholds.** These were loosened to: axis tilt < 0.2°, roll < 3°, dpos < 2.5 mm.
  The balls sit only 26 mm off the axis, so 1 mm noise moves the roll by ~0.8°.

**User requests after the first lab look (2026-10-16):**
- **Every streaming bar.** `measure_mocap_bars` measures every bar that has a marker table and a
  Rhino pose in the loaded action, as long as its asset streams fresh markers, whatever action is
  loaded. The loaded bar keeps the unlabelled / fake fallback. Motive bar assets with no marker
  table (B34, B35 today) are named once in the log.
  - A take is recorded per bar, with `design_markers` and `design_T_world_from_bar` inside the
    take. The file-level copies of these are gone; only `mocap_markers_file` stays at file level.
- **Live at 10 Hz.** Live mode starts each fit from the bar's last pose:
  `fit_bar_from_cache(T_hint=...)`, with a 20 mm gate, then the 4 mm check.
  - Measured in the GUI with two bars: ~1 ms per refresh, every third ~12 ms.
  - A click always runs the full search with the ambiguity check (~24 ms per bar).
  - **Review fixes.** The quick fit is kept only when EVERY design marker matches, re-pairing
    until the pairs are stable. Otherwise the full search runs: with balls missing the quick fit
    could stay locked on B20's wrong 6-of-8 assignment, and live then agrees with a click on
    ambiguity.
  - **Render pause.** The live redraw pauses rendering with `mocap_bar_viz.rendering_paused`,
    not `pp.LockRenderer`. Interleaved with a `LockRenderer` on the panel thread ('Plan
    Movement'), two `LockRenderer`s could leave rendering off for good; a stubbed interleaving
    reproduced that, and the fix ends with rendering on in both orders.
  - **Tick and wipe.** The tick skips a frame instead of waiting for a click (non-blocking lock).
    'Remove all drawing' forgets the pink and MoCap debug ids.
  - **GUI timing.** Measure it with the screen unlocked: with the session locked the PyBullet
    window stops drawing and every call into it times out after ~1 s (2026-10-16).
- **Pink line checkbox.** `Rhino bar lines (pink)` (`SHOW_RHINO_BAR_LINES`) shows or hides every
  pink line and label: the assembled line of each bar loaded this run (they used to leak and stay)
  and the overlay's Rhino centre line and label.

**Heads-up (Rhino repo, not touched here):** `scripts/core/env_collision (2).py` is an Insync
conflict copy that holds the current code, while `env_collision.py` is an Oct-2 version. The
15:21 export worked; check before the next one.

## What this is about

A `T20_MoCap` joint is a T20 Female joint with a plate carrying two OptiTrack ball markers. Several
such joints are rigidly mounted on a scaffold bar, so the bar's full 6-DoF pose can be read from
Motive. We want that pose for two things:

1. **Grounded bars**: use the measured bar pose as the visual-servo target instead of the authored
   tool0 frames (follow-up, see B4).
2. **As-built study**: after each assembly / hold action, log measured vs designed pose and
   per-marker residuals, live and post-mortem (follow-up, see B4).

This round builds the measuring tool: load the design marker table, match live Motive markers to it
without using Motive's marker ids, fit the bar pose, report the deviation from the designed pose,
record it.

## What the Rhino side exports (schema 2, being implemented in the Rhino repo)

`MoCapMarkers.json` next to `RobotCell.json` in the design problem folder. Metres, same world as
the RobotCell / BarActions (Motive's frame relabelled z-up, the "rhino" axis convention).

```json
{
  "schema_version": 2, "source": "261016_2ModulesDemo_YH.3dm", "units": "m", "cell_file": "RobotCell.json",
  "bar_frame": "frame of the bar body of the same name in RobotCell.json / BarActions states: origin at the bar start, z along the bar; world = bar_state_frame @ bar_m",
  "marker_name": "<joint object name>_<label>, e.g. J20-21_mocap_M1 (name the Motive marker this)",
  "layout_tolerance_m": 0.002,
  "marker_blocks": {"T20_MoCap": {"marker_diameter_m": 0.019,
                                  "points_block_m": {"M1": [-0.01, 0.02415, 0.0255], "M2": [-0.01, -0.02415, 0.0255]}}},
  "bars": {
    "bar_B20": {"bar_id": "B20", "assembly_seq": 1, "layout": {"n_markers": 8, "nearest_miss_m": 0.0412},
                "markers": [{"name": "J20-22_mocap_M1", "joint": "joint_J20-22_mocap", "label": "M1",
                             "block": "T20_MoCap", "bar_m": [-0.0274, -0.0220, 0.0849]}, "..."]}},
  "bars_without_markers": ["bar_B22", "bar_B23"]
}
```

- `bars` is keyed by the cell body name; only bars that carry markers are listed.
  `bars_without_markers` lists the other bar bodies of the same export, so the monitor knows which
  bars are untracked.
- `bar_m` is the marker centre in the **bar body frame**: the same frame as
  `rigid_body_states["bar_B20"].frame` in a BarAction state. So the measured bar pose
  `T_world_from_bar` is directly comparable to that state frame.
- The Rhino export **refuses** a bar whose layout is ambiguous (see the rule below), so every bar in
  the file has ≥ 3 non-collinear markers and a unique rigid fit.

## Facts about the marker geometry that drive the matcher

Measured on the real export `261016_2ModulesDemo_YH` (B20: 1800 mm bar, 4 joints at
75/430/1135/1670 mm; B21: 2000 mm, joints at 406/598/1090/1482 mm; 8 markers each):

- A pair's two markers are 48.3 mm apart; the pair centre sits 25.5 mm off the bar axis.
- **M1 and M2 of one pair cannot be told apart by pairwise distances**: their sorted distance lists
  differ by ~0.35 mm because all other markers are ~1 m away along the bar. So any distance-based
  pre-filter will produce both assignments. The decision must come from the **full-set residual**
  after transforming all design points — the wrong assignment leaves ~48 mm errors elsewhere.
- **A three-point triple cannot reject a mirror**: a flipped triangle is reachable by a proper
  rotation. Mirror rejection comes from enforcing det = +1 in Kabsch *and* scoring on all markers
  with a minimum of 4 inliers.
- **Same-label markers on same-roll joints are exactly collinear** (a line parallel to the bar).
  Kabsch on such a triple leaves the rotation about that line undetermined — skip those triples.
- Exact ambiguities (zero residual) that the Rhino export now refuses, for your understanding:
  two joints on a bar are always ambiguous (end-for-end half-turn); any number of joints all facing
  the same way are always ambiguous (half-turn about the plate midline swaps M1↔M2 everywhere).
  Rule of thumb: ≥ 3 joints, not all facing the same way, not mirror-symmetric about the middle.
- `pp.superimposition_matrix` (Kabsch) imports pybullet through the package `__init__`, so the
  pure module below carries its own 12-line Kabsch (same SVD + det fix as
  `external/pybullet_planning/src/pybullet_planning/utils/transformations.py:845-856`).
- Motive raw data as saved by `save_markerset_data` (e.g.
  `data_experiment/bar_holding_acc_data/20261001/bar_holding_acc_20261001_1514.json`): per marker
  `{'pos': [x,y,z] m, 'size': [~0.018], 'error': ~1e-3}`, ids `1..8` (strings after JSON), already
  in the "rhino" z-up convention. That take is the **old** 4-marker cross rig (82 mm pairs), not a
  `T20_MoCap` bar — a good negative test.
- NatNet: `__unpack_labeled_marker_data` (`optitrack/NatNetClient.py:526-581`) decodes the
  occluded / point-cloud / model-solved flags but passes only `pos, size, error` to the listener;
  model id 0 is the unlabelled bucket. Rigid-body descriptions (`:836-915`) carry the asset name
  and streaming id and are stored in `get_latest_data_descriptions().rigid_body_list` after
  `request_model_definitions()` (`husky_monitor.py:12210`).

## B1. New pure module `husky_assembly_teleop/bar_mocap_fit.py` (numpy + stdlib only)

No ROS, no pybullet, no compas, no scipy, so it is unit-testable on any machine.

Data classes: `DesignMarker(name, joint, label, bar_m)`, `BarMarkers(bar_name, assembly_seq,
markers; points(), names())`, `MoCapMarkerSet(schema_version, source, units, bar_frame_note,
blocks, bars: dict[str, BarMarkers], bars_without_markers, path, warnings)`,
`MoCapMarkersError(RuntimeError)`.

- `load_mocap_markers(problem_root) -> MoCapMarkerSet` — plain `json.load` of
  `<problem_root>/MoCapMarkers.json` (same pattern as `WalkableGround.json`,
  `cfab_session.py:146`). Raise `MoCapMarkersError` with the full path when: file missing;
  `schema_version != 2` (name the found version and say to re-export from Rhino); `units != 'm'`;
  a bar with < 3 markers; malformed `bar_m`. Warn (in `warnings`) when a bar's markers are
  collinear.
- `kabsch(src (K,3), dst (K,3)) -> 4x4` proper rigid transform (det = +1): centre both, SVD of
  `dst_c.T @ src_c`, flip the last singular vector when the determinant is negative.
- `match_and_fit(sensed_points_m (N,3), design_points_bar_m (M,3), *, tol_m=0.004,
  side_tol_m=0.002, min_inliers=None, min_triangle_height_m=0.015, max_sensed=24) -> BarFit`
  1. Checks: N, M ≥ 3; N ≤ `max_sensed` else fail ("too many markers in view; disable other
     assets or use the per-bar asset"). `min_inliers` defaults to `max(4, ceil(0.75·M))` → 6 of 8.
  2. Pairwise distance tables `dS` (sensed) and `dD` (design).
  3. Design triangles: every unordered design triple whose smallest altitude ≥ 15 mm (skips the
     collinear same-label chains; pair + far-marker triangles have ≥ 25 mm altitude).
  4. Per triangle (a,b,c): all **ordered** sensed triples (i,j,k) whose three side lengths match
     within `side_tol_m` (boolean masks on `dS`). This deliberately includes M1/M2-swapped and
     mirror hypotheses.
  5. Per hypothesis: Kabsch on the triple → transform all design points → list every
     (sensed, design) pair closer than `tol_m`, sort ascending, assign greedily one-to-one →
     score `(n_inliers, -rms)`. Skip when n_inliers < 3.
  6. Refine the best: refit Kabsch on all inliers, re-assign once, refit again. Fill residuals,
     `missing_design`, `unmatched_sensed`.
  7. Ambiguity: dedupe hypotheses by their inlier correspondence set; `second_best` = the best set
     that differs from the winner; `ambiguous = second_best.n == best.n and
     second_best.max_residual ≤ tol_m`. Always report `second_best` so near-misses are visible.
  8. `ok = n_matched ≥ min_inliers and not ambiguous`; otherwise a plain-English `reason`
     ("only k of M design markers matched (need n)", "two different marker assignments fit equally
     well (symmetric layout?)", "no three sensed markers match any design triangle (wrong bar, or
     wrong unit / axis convention?)").
  Cost: N = M = 8 → ~56 triangles × ≤ 336 ordered triples, mostly pruned by the masks → tens of
  milliseconds. N = 16 (two bars in view) well under a second.
- `BarFit` fields: `ok, reason, bar_name, T_world_from_bar (4x4), correspondences {sensed idx →
  design idx}, sensed_ids, residuals_m, rms_m, max_residual_m, n_matched, n_design,
  unmatched_sensed, missing_design, ambiguous, second_best, n_hypotheses, to_json()`.
- `fit_bar_from_cache(cache {id: {'pos','size','error'}}, design: BarMarkers | ndarray, *, age_s,
  max_age_s, **kwargs) -> BarFit`: int or string ids (saved takes use strings); drop markers with
  `error` > 3 mm or `size` outside 12–26 mm (`size` is a number **or a 1-element list**: NatNet
  hands a 1-tuple); refuse when the snapshot is older than `max_age_s`.
- **(refined)** `fake_sensed_markers(T_world_from_bar, design_points, *, slide_m=0.03,
  roll_deg=2.0, noise_m=0.0005, seed=0) -> dict`: a cache-shaped snapshot of the design markers
  moved by a known offset (along and about the bar z), plus one stray marker 0.3 m away. Used by
  the monitor's desk-rehearsal flag `MOCAP_BAR_FAKE_MARKERS`.
- `pose_deviation(T_meas, T_design) -> PoseDeviation(dpos_mm, dang_deg, dpos_world_mm,
  dpos_bar_mm, rotvec_bar_deg, to_json())` — the error re-expressed in the **design bar frame**
  (x, y lateral; z along the bar) so "slid 5 mm along the bar" reads directly.
- `transform_points(T, pts)`, `design_bar_axis_endpoints(T, length_m)` for drawing.

How the failure cases play out: a ghost marker never joins a winning triple and ends in
`unmatched_sensed`; one occluded marker → 7 of 8, still ok; a second bar's markers do not fit this
bar's joint spacing and become `unmatched_sensed`; stale cache entries are cured at the source
(B2). Tolerances: `tol_m = 4 mm` is 10× below the 48 mm pair spacing and 3–4× above the Motive
per-marker residual (~0.8 mm in the Oct-1 takes); `side_tol_m = 2 mm` is a pre-filter only — if
the first live fit shows `max_residual_m` above 2 mm (jig placement), raise it to 3 mm.

## B2. Monitor integration (`husky_monitor.py`, `husky_world.py`)

- **(refined) Designed bar frame**: `_designed_bar_frame(bar_name) -> (pose, state) | None`:
  the first start state in `self._loaded_movements` where the bar is not attached
  (`attached_to_link or attached_to_tool` falsy) and has a `frame` → `pose_from_frame(frame)`.
  For a J or R action of that bar this is its Rhino assembled pose (the same in both, checked on
  B20). Other bars of the loaded action work the same way (their frame is present even when
  hidden). The held-bar reference (`get_movement_start_bar_pose`) is left for B4.
- **Load the sidecar** in `_finish_action_load` (`:3457`), after the cfab session block (`:3534`):
  `self._mocap_markers = load_mocap_markers(self._problem_dir())` in a try; warn on
  `MoCapMarkersError`. Initialise `self._mocap_markers = None` near `:495`. Log
  `[mocap bars] N instrumented bar(s): bar_B20 (8), ...`.
- **(refined) Sidecar check** at load, active bar only: for each of its markers, joint frame in
  the resting state × `marker_blocks[block].points_block_m[label]`, expressed in the bar frame,
  against `bar_m`. Log the largest gap; warn above 1 mm (a sidecar stale against these
  BarActions: the markers would not sit on the drawn plates).
- **Per-bar Motive assets**: `_register_mocap_bar_assets()` walks
  `self.mocap_client.get_latest_data_descriptions().rigid_body_list`; for every asset whose decoded
  name is a key of `self._mocap_markers.bars`, set `self.name_from_mocap_id[id] = name` (refuse and
  warn on a clash, e.g. 1002 = `bar_rig`). Call it lazily from `measure_bar_pose` and once from the
  20 Hz tick while the map is empty. Fallback config `MOCAP_BAR_STREAMING_IDS = {}` in
  `husky_world.py` next to the `bar_rig` registration (`:535-538`). Keep `bar_rig` untouched.
- **Fix the labelled-marker cache** (`receive_labeled_marker`, `:12302-12325`): key by
  `name_from_mocap_id.get(model_id)` or `'_unlabeled'` for model 0; convert positions as today
  (`mocap_pos_y_up_to_z_up(..., self.MOCAP_AXIS_CONVENTION)`); replace the **whole dict** per model
  per frame under the existing `self._mocap_cache_lock` (`:489`) so an occluded marker disappears instead of
  lingering; record `self._mocap_labeled_marker_stamp[key] = time.monotonic()`. Add
  `labeled_markers_snapshot(name) -> (deep copy, age_s)` and switch the two existing readers to it
  (`husky_world.py:1139-1143` in `request_marketset_button`, `husky_monitor.py:9706-9709` in
  `record_bar_take_with_shared_viz`). Register the listener whenever `USE_MOCAP` is on (today it is
  gated on `BAR_ACTION_MOCAP_ACCURACY_TEST`, `:12197-12198`). Optional: add `'flags': param` to the
  listener dict in `NatNetClient.py:569-573`.
  (Refined: `rigid_body_list` items are `RigidBodyDescription` with `sz_name` as **bytes** and
  `id_num`; the monitor reads `rigid_body_list` nowhere today.)
- **`measure_bar_pose(bar_name=None, *, log=True) -> BarFit | None`**: default to
  `self.active_bar_name`; `None` with a warning if the bar is untracked; snapshot for the bar's
  asset, falling back to `'_unlabeled'` (or `fake_sensed_markers` at the Rhino pose when the new
  class flag `MOCAP_BAR_FAKE_MARKERS = 1`; default 0); `fit_bar_from_cache`; `pose_deviation`
  against `_designed_bar_frame`. Log lines: (1) bar, marker source, n/M, rms mm, max mm, dpos mm,
  dang deg, dpos_bar xyz mm, ambiguous, reason; (2) per-marker residuals by name and the missing
  names; (3) the bar's end points in world (mm), Rhino vs MoCap, for a tape-measure check.
- **(refined) By-eye overlay.** `_build_bar_joint_ghost(bar_name, rgba)`: one visual-only PyBullet
  body (`createVisualShape(GEOM_MESH, rgbaColor)`, no collision shape, not in the cfab cell) per
  visual mesh of the bar and of each `bar_action_io.joints_on_bar` joint, the OBJ written with
  `mesh.to_obj()` from `robot_cell.rigid_body_models[name].visual_meshes_in_meters`;
  `bar_from_body = inv(bar frame) · body frame` from the resting state. Built once per bar, moved
  with `pp.set_pose`. Two ghosts: **Rhino** (`ASSEMBLED_BAR_PINK`, see-through) at the designed
  pose, **MoCap** (new `MOCAP_BAR_CYAN`, see-through) at the fitted pose, hidden when the fit
  fails. Debug items (always rendered, ids kept for removal): both bar axes with labels
  (`B20 Rhino`, `B20 MoCap 8/8 rms 0.9 mm`), each sensed marker as a small cross (white matched,
  red stray), a black line from each matched marker to its fitted design marker, a red label at
  each missing design marker.
- **(refined) Panel** (a "MoCap bar" separator, `self.buttons.append(Button(...))` / `Toggle`):
  `MoCap bar: measure + draw (loaded bar)`; toggle `MoCap bar: follow live (2 Hz)` (`update()`
  re-fits twice a second and moves the MoCap ghost; logs only when the fit turns ok ↔ failed);
  `MoCap bar: clear drawing` (also done at the next action load); `MoCap bar: record take`
  (mirrors `record_bar_take_with_shared_viz` `:9693-9731`: snapshot, `tool_ft`, then
  `take = {bar_name: markers, 'markerset_key': bar_name, 'bar_fit': fit.to_json(),
  'deviation': dev.to_json() or None, 'marker_source': 'asset'|'unlabeled'|'fake', 'marker_age_s': age}`
  appended to `self.marker_set_data`, saved by the existing `Save markerset data`). Keep the
  `bar_rig` buttons.
- **(refined)** `bar_action_io.joints_on_bar(rb_names, bar_id) -> list[str]`, pure, after the
  Rhino naming (`joint_name_conventions.py:22-24`): `joint_J<a>-<b>_male` sits on bar b, any other
  role of a pair id (`_female`, `_mocap`, sub-females) on bar a (the receiver);
  `joint_G<a>-<Type>-<i>_*` / `joint_M<a>-<Type>-<i>_*` on bar a; an `env_` prefix is allowed.
  For B20 this is the set held with B20 in its J action.

## B3. Recording and offline re-analysis

- `save_markerset_data` (`husky_world.py:1213-1294`, payload at `:1280-1291`): add
  `mocap_markers_file` (path), `mocap_markers_design` (the active bar's design table so a take can
  be re-fitted from its own file), `design_T_world_from_bar` (4x4 list). Existing fields stay, so
  `data/bar_holding_acc_data/1_compare_to_cell_state.py` keeps working for the old rig.
- **(next round)** New `data/bar_holding_acc_data/4_refit_mocap_bar.py <take.json> [--markers MoCapMarkers.json]
  [--bar bar_B20] [--tol-mm 4] [--out refit.json]`: per take, pick the marker dict
  (`markerset_key` → `'bar_rig'` → the first dict of `{'pos': ...}` entries; ids are strings),
  require `mocap_axis_convention == 'rhino'` (warn and skip otherwise), `fit_bar_from_cache`,
  `pose_deviation` against `design_T_world_from_bar` (else `bar_start_position/quaternion`), print
  one row per take (n_matched, rms, max, dpos, dang, ambiguous, reason), write `_refit.json` next
  to the take. On `20261001/bar_holding_acc_20261001_1514.json` against a T20 design every take
  must report `ok=False`.

## B4. Follow-ups (context only, not in this round)

- **Grounded-bar visual servo: shipped 2026-10-19**, in a different form from the one sketched here.
  - **Which bar.** The HELD grounded bar itself is measured (B20 / B34: held on ground joints,
    nothing to mate with), not a partner bar.
  - **The loop.** `husky_world.servo_to_movement_start_live(bar_feedback=...)`, behind the panel
    checkbox `Servo: aim grounded bar by MoCap (else tool0)`. Every pass:
    - `measure_bar_servo_feedback` fits the bar;
    - `T_corr = T_design @ inv(T_mocap)` (`bar_mocap_fit.bar_servo_correction`) is applied to both
      LIVE FK flanges (`monitor._servo_ee_target_override`, read by `ik_live_base_for_selected_movement`);
    - it converges at 0.5 mm / 0.1° of the bar's pose;
    - it refuses fits beyond 30 mm / 3° or with an rms over 3 mm, and never steers on fake markers.
  - **Carry into the insert.** The insert then makes the authored move from where the flanges
    actually stand (`monitor._carried_insert_goal`).
  - **Servo after the insert.** `husky_world.execute_grounded_insert_then_servo` runs a second servo
    onto the assembled pose: straight lines only, with a +20 N load guard.
  - **Comparison.** Every servo with an instrumented held bar records FK vs MoCap vs design per
    reading (`rec['bar']`).
  - Lab steps: `doc/mocap_bar_pose_test_manual.md` section 10. A partner-bar correction (for
    bars that mate) is still open.
- **As-built logging.** After each assembly / hold action is marked done, `measure_bar_pose` for
  every visible instrumented bar → append `{movement_id, bar, fit, deviation, timestamp}` to
  `<problem>/as_built.jsonl` next to `progress.json`; add a dashboard run kind in the style of
  `dashboard/run_writer.py` (`dashboard/run_schema.py:18-19`) so the live page can plot designed vs
  measured per bar and per-marker residuals (bending: per pair, midpoint distance from the axis and
  angle about it vs designed; pair length 48.3 mm — a length error means a mislabelled or occluded
  marker).

## B5. Tests — `test/test_bar_mocap_fit.py` (numpy + pytest only)

Fixture `t20_design()`: joints at z = 0.075 / 0.430 / 1.135 / 1.670 m with roll angles
0 / 90 / 180 / 45° about the bar z; block-local M1 = (-0.010, 0.02415, 0.0255),
M2 = (-0.010, -0.02415, 0.0255) m; joint-in-bar transform = rotate about z by roll, translate along
z. Helper `random_rigid(seed)`.

1. Exact recovery: transform, shuffle rows → `ok`, `n_matched == 8`, `||T - T_true|| < 1e-9`,
   correspondences invert the shuffle.
2. 1 mm Gaussian noise → `dpos_mm < 1`, `dang_deg < 0.2`, `rms_m < 2e-3`.
3. One marker dropped → ok, `n_matched == 7`, `missing_design` holds the dropped index.
4. One ghost marker 0.3 m away → ok, ghost in `unmatched_sensed`.
5. Three dropped → `ok is False`, reason mentions "5 of 8".
6. Mirror: sensed = design · diag(1, 1, -1) → `ok is False`; assert `det(T[:3,:3]) > 0` on every
   returned fit.
7. Symmetric layout (joints at 0.1 / 0.5 / 1.3 / 1.7, all rolls 0°) → `ambiguous is True`,
   `second_best['n_matched'] == 8`, `ok is False`.
8. M1/M2 swap resolved: the winner's correspondences equal the truth (the swapped hypothesis
   scores lower).
9. Collinear design → clear failure (loader warning / `ok=False` with the collinear reason).
10. Loader: schema-2 file in `tmp_path` round-trips; schema-1 file (shape of the real v1 export)
    → `MoCapMarkersError` naming version 1; missing file → `MoCapMarkersError`.
11. Negative on real data (`skipif` absent): `EXPERIMENT_DATA_DIRECTORY/bar_holding_acc_data/
    20261001/bar_holding_acc_20261001_1514.json`, `raw_data[0]['bar_rig']` (string ids) through
    `fit_bar_from_cache` against `t20_design()` → `ok is False`, no exception.
12. `pose_deviation`: 5 mm along the bar z → `dpos_bar_mm ≈ (0, 0, 5)`, `dang_deg ≈ 0`; 1° roll
    → `dang_deg ≈ 1`.

Refined additions:

13. The real B20 table (`bar_m` of `261016_2ModulesDemo_YH`, copied into the test) through cases
    1, 3, 4 and 8.
14. `fake_sensed_markers` + fit → `dpos_bar_mm ≈ (0, 0, 30)`, `dang_deg ≈ 2`, the stray marker in
    `unmatched_sensed`.
15. `fit_bar_from_cache` accepts `size` as a float and as `[0.0188]`.
16. Data test (`skipif` absent): the 2ModulesDemo sidecar loads (schema 2, `bar_B20` and
    `bar_B21` with 8 markers), and `B20__R.json`'s joint frames × block points agree with `bar_m`
    (< 0.05 mm).
17. `test/test_bar_action_io.py`: `joints_on_bar` gives B20's eight joints
    (`J20-22/21/169/25_mocap`, `J20-42/44_female`, `G20-T20-0/1_ground`) and B21's
    (`J20-21_male`, `J157-21_male`, `J21-165_female`, `J21-22/26/37/38_mocap`) from name lists.

## B6. Verification

Offline (Claude): `python -m pytest test/test_bar_mocap_fit.py test/test_bar_action_io.py -q`,
the existing servo / plan-skeleton tests, `python3 -m colcon build --symlink-install
--packages-select husky_assembly_teleop`; a headless render (CfabSession in DIRECT mode on
2ModulesDemo, `B20__J`, both ghosts, fake markers, `p.getCameraImage` → PNG) to see the joints sit
on the bar and the cyan ghost 30 mm along / 2° rolled; a GUI dry start of the monitor with
`FAKE_HARDWARE=1`, `USE_MOCAP=0`, `MOCAP_BAR_FAKE_MARKERS=1`.

Live (the user): `doc/mocap_bar_pose_test_manual.md`.

## Open questions to check on the real system

1. NatNet / Motive versions (`mocap_client.get_nat_net_requested_version()`,
   `NatNetClient.py:1438`): marker names stream only with NatNet ≥ 4, the `error` residual with ≥ 3.
2. Are per-bar rigid-body assets defined in Motive and named exactly like the bars (`bar_B20`)?
   If not, rename in Motive or fill `MOCAP_BAR_STREAMING_IDS`.
3. Do labelled markers still arrive with model id 0 when the asset is disabled or has lost
   tracking? The fallback relies on it.
4. Does Motive also report a *tracked* asset's markers in model 0 (duplicates)? If yes, dedupe
   points closer than 1 mm in `fit_bar_from_cache`.
5. On the first real take, do the pairwise marker distances agree with the design within
   `side_tol_m`? If not, raise it to 3 mm.
6. Bar-frame z sign: the comment at `husky_monitor.py:8160-8161` says only the sign varies per
   export. The sidecar's `bar_m` and the `RigidBodyState.frame` come from the same
   `canonical_bar_frame_from_line`, so they agree; a wrong sign would show as a 180° `dang_deg` on
   an otherwise perfect fit.
