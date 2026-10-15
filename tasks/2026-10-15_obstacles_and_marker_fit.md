# Obstacles editing and bar marker fitting (2026-10-15)

Status: **both done** (2026-10-15, `jg/viser-cleanup`), not run in the lab yet. Open: "Fit from probe points", and
measuring whether a live marker fit is needed. Two plugin refactors agreed in discussion: the lab layout moves into a file that
`obstacles` edits, `debug_gizmo` shrinks to a pure debug tool, and the bar marker fit leaves `mocap_probe` for its own
plugin.

Code today: `plugins/obstacles.py` (lab as Python constants, no UI), `plugins/debug_gizmo.py` (moves any body, adds
and resizes temporary boxes and cylinders, saves them to `~/husky_debug_gizmo.json`), `plugins/mocap_probe.py`
("raw markers (debug)" section: prefab upload, Fit bar), `world/marker_fit.py` (`fit_markers`, pure).

## Decisions

- **Obstacles owns all obstacles** (user): lab furniture and temporary obstacles alike, with the editing tools that are
  in `debug_gizmo` today (select by click or dropdown, move, resize, add, remove).
- **Lab layout in a file in git**: `data/lab/<name>.json`, chosen by a `lab` parameter (default: the current lab).
  The lab is shared physical truth, so it is reviewed in git and the same for everyone; another file covers a second
  room or a test layout. Saved only on an explicit Save (dirty chip, Revert), so a stray drag never rewrites it.
- **The file stores resolved shapes** (type, centre, size, yaw; types box, cylinder, truss, border), plus an optional
  `source` note ("probe 2026-09-30", "by hand"). Probe points are an import, not the stored form: once a box is
  dragged they no longer match it.
- **Temporary obstacles are part of obstacles but never go into the lab file** (user). They are kept in a purely local
  file outside the repo, `~/husky_obstacles_temporary.json`, written automatically, so they survive a restart as now
  (commit d8b3a92). Read the old `~/husky_debug_gizmo.json` once so nobody loses theirs. Ids
  `obstacles/temporary/<n>`, purple as now. "Promote to lab" turns one into a lab obstacle (saved on the next Save).
- **debug_gizmo moves everything, nothing else** (user): click or pick any body, drag or type its pose, Log, Reset.
  No Add, no resize, no file, no skipped bodies (obstacles included). Its moves are temporary and never saved.
- **Ids stay stable**: `obstacles/boxes/<slug>`, `obstacles/cylinders/<slug>`, `obstacles/trusses/<slug>`;
  `base_exp` relies on the `obstacles/` prefix for the lab border.
- **Bar marker fitting becomes its own plugin** (user), working name `mocap_accuracy` (the roadmap row for the old
  `BAR_ACTION_MOCAP_ACCURACY_TEST`), `requires = ("process",)`. `mocap_probe` goes back to probe points and the
  coverage map.
- **Not in the mocap relay**: the relay (crl-husky) stays a generic NatNet → ROS bridge, one per domain and shared by
  every consumer; design knowledge does not belong there. The only relay change that might ever be wanted is generic:
  streaming unlabelled markers, if bars must be tracked without a Motive asset.
- **Marker layouts come from the process** (`BodySpec.markers`, "for placement checks and mocap registration"), not
  from a prefab upload.

## Todo

### Obstacles / debug_gizmo

- [x] Lab file schema and loader: `data/lab/<name>.json`, `lab` parameter in `config.py` (default the current lab).
- [x] Write the current `BOXES`, `CYLINDERS`, `TRUSSES` and `BORDER` from `obstacles.py` into the file; `obstacles`
      loads it and keeps the same ids.
- [x] Move picking (`ray_distance`, the scene click callback), the move gizmo and the corner resize gizmo from
      `debug_gizmo.py` into one reusable widget in `ui/`, used by both plugins.
- [x] Obstacles panel: select, move, resize, add, remove, rename; for lab obstacles Save, Revert and a dirty chip;
      for temporary ones Promote to lab.
- [x] Temporary obstacles in `obstacles`: `obstacles/temporary/<n>`, saved automatically to
      `~/husky_obstacles_temporary.json`; read `~/husky_debug_gizmo.json` once if the new file is missing.
- [x] Shrink `debug_gizmo` to move anything, Log and Reset.
- [x] Tests: lab file round trip, ids unchanged, temporary file migration, `base_exp` still finds the border
      (`OBSTACLES_PREFIX`).
- [x] Docs: both plugins' docstrings, README plugin table, `doc/plugin_roadmap.md` (add a `debug_gizmo` row; the
      `obstacles` row lists only boxes and tripods).
- Done as: `plugins/obstacles/` (`lab.py` data and files, `plugin.py`), `data/lab/main.json` (one obstacle per line,
  the same 15 ids), `ui/body_gizmo.py`, `config.MonitorConfig.lab`. A drag keeps only the yaw: lab shapes are
  upright. Temporary obstacles are boxes and cylinders; Add puts them at the origin. `base_planner/figures.py` draws
  the lab from the file.
- [ ] Later, optional: "Fit from probe points" in obstacles, reading `mocap_probe` exports
      (`Box.from_top_points`, `Cylinder.around_points`).

### Marker fitting plugin

- [x] New plugin `mocap_accuracy` (working name), `requires = ("process",)`, experimental: the raw-marker view and the
      bar fit moved out of `mocap_probe`.
- [x] Marker layouts from the process (`world/marker_fit.bar_markers`): the markers of the halves mounted on a bar,
      moved into the bar frame by both assembly poses. The legacy converter writes none, so old exports have no
      layouts; `261008_2ModulesDemo` has 4 bars with 8 each (B20, B21, B34, B35). The operator picks the bar and the
      Motive rigid body (Bar, Mocap ID, Track).
- [x] Fit, residuals and the deviation from the planned bar in the scene (`process/bars/<id>`: across, along, tilt;
      roll left out); Record take saves an .npz to `data_experiment/mocap_accuracy/<experiment>/` (create it in
      Drive first). On synthetic data from that process: about 1-2 ms per fit, 0.7 mm rms at 0.5 mm noise.
- [x] Raw-marker and fit section removed from `mocap_probe` (and `bar_markers_from_prefab`); its docstring, the README
      and the roadmap row updated.
- [ ] Measure whether a live fit is needed, or a fixed CAD offset per Motive rigid body is enough.

### Later, only when servoing or bar_action needs fitted bars in the scene

- [ ] A marker-fit pose source on `ctx.track_object`, so a fitted bar is a tracked object like any other (health
      chip, `tracked/<name>` in the scene). Match once, keep a Motive member id → model marker map, refit by least
      squares per sample, re-match only when the residual jumps (the matching search is exponential and runs on the
      main thread).
- [ ] Only if bars must be tracked without a Motive asset: propose an unlabelled-marker topic for the relay in
      crl-husky.

## Open

- Name of the marker-fitting plugin (`mocap_accuracy` is a working name).
- Whether temporary obstacles need a "clear all".
