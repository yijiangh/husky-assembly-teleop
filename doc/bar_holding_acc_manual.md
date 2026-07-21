# Bar-holding accuracy data processing

Two scripts turn raw mocap marker takes (recorded while the robot holds a bar
at a movement's start state) into accuracy numbers:

- **`0_bar_acc_data_processing.py`** — fits the bar axis from the markers and
  reports the bar's pose/orientation/length. No BarAction needed; this is the
  raw "what does mocap say the bar is" pass.
- **`1_compare_to_cell_state.py`** — additionally compares the fitted bar
  against the *intended* bar pose (the movement's start-state pose), so you get
  a deviation in mm / degrees.

Run `0_` first to sanity-check the fits, then `1_` for the actual accuracy.

---

## Where the data lives

Experiment data now lives on Google Drive (not the local repo). The scripts read
`EXPERIMENT_DATA_DIRECTORY` from `husky_assembly_teleop/__init__.py`, currently:

```
/home/su/Insync/.../2025-03 Husky Assembly/data_experiment/bar_holding_acc_data/
```

Layout — one date folder per session, one JSON per "Save markerset data" click:

```
bar_holding_acc_data/
  20260517/
    bar_holding_acc_20260517_1406.json    <- one saved batch (>=1 take)
  20260706/
    bar_holding_acc_20260706_1556.json
```

You pass the **date folder name** (the batch) as the script argument, e.g. `20260517`.

### Saved JSON schema

The live monitor (`Save markerset data` button) writes:

| field | meaning |
|-------|---------|
| `mocap_axis_convention` | `rhino` (current) or legacy `rotated`; drives axis correction on load |
| `bar_action_path` | absolute path to the BarAction the movement came from |
| `movement_id` | **string** role of the chosen movement, e.g. `M2`, `M3` |
| `bar_name` | active bar id, e.g. `bar_B6` |
| `bar_start_position` / `bar_start_quaternion` | bar world pose in the movement's **start state** (the reference `1_` compares against) |
| `bar_dimensions` | bar AABB extents `[dx, dy, dz]` in metres; the longest is the nominal bar length |
| `raw_data` | list of takes; each has a `bar_rig` marker dict (`Record + Fit + Viz` also stamps `joint_conf`, base poses) |

`bar_start_position/quaternion` and `bar_dimensions` are stamped so the offline
scripts don't have to re-parse (and re-resolve) the BarAction file. Older takes
lack them — see **Old data** below.

---

## Recording a take (live monitor)

Do this **before** processing, on the real robot with the live monitor. Steps
1–11 drive the robot to a movement's start state and mount the instrumented bar;
step 12 is the actual `Record + Fit + Viz (shared)` → `Save markerset data`.

> The bar/movement metadata (`bar_action_path`, `movement_id`, `bar_start_*`,
> `bar_dimensions`) is stamped from the **currently loaded movement**, so you
> must Load BarAction *and* Load Movement (steps 3–4) before recording —
> otherwise those fields save as `null` and the take can't be matched later.

1. Set `DESIGN_PROBLEM_NAME` to the right problem in
   `husky_assembly_teleop/__init__.py` (line ~63).
2. Launch `husky_monitor`.
3. Click **Load BarAction**.
4. Set the **Movement (idx; 0=M0_synth)** slider to **3** (M3 = retreat, bar
   already installed/held at a known pose), then **Load Movement**.
5. Use the joystick to drive the real (colored) mobile base so it roughly aligns
   with the red ghost robot shown in the PyBullet monitor.
6. Replan the arms from the live base to the movement start, in order:
   1. First click **1) IK Live Base → Set Mv Start Goal (no traj)** — solves
      live-base IK and sets the goal pose (no trajectory yet).
   2. Then click **2) IK + Plan Transit → Mv Start (live, M2/M3)** — plans the
      transit trajectory to that goal (enables the traj viz slider).
7. If a path is found, scrub the **trajectory viz slider** to preview it and
   confirm it's collision-free (it should be, but check).
8. Click **Exec Both Arm Trajs** to move both arms to the movement start.
9. Prepare the bar joints per the BarAction, and mount **four pairs** of mocap
   rig markers on the bar. The **outer two pairs must align with the bar's ends**;
   the inner two pairs' exact positions don't matter (the fit auto-pairs markers
   by cross-bar distance and uses the end pairs for length/axis).
10. Define the bar rigid body in Motive (the `phase1_test` rig, streaming id
    `1002`), read its **streaming id**, and set it in
    `husky_assembly_teleop/husky_world.py` (the `bar_rig` `TrackedObject`,
    line ~335 — replace the `1002` streaming id).
11. Manually mount the bar (with rig) onto the robot's tools.
12. Click **Record + Fit + Viz (shared)** (once per take), then **Save markerset
    data**. The save lands under
    `EXPERIMENT_DATA_DIRECTORY/bar_holding_acc_data/<YYYYMMDD>/`.

---

## Setup

From the ros2 workspace root, with the project venv active and the overlay sourced:

```bash
cd /home/su/ros2_ws
source venv/bin/activate
source install/setup.bash          # so `import husky_assembly_teleop` resolves
```

---

## `0_bar_acc_data_processing.py` — fit + report

The `batch` (date folder) argument is optional and **defaults to `20260706`**;
pass another folder name to override.

```bash
python src/husky-assembly-teleop/data/bar_holding_acc_data/0_bar_acc_data_processing.py            # default batch 20260706
python .../0_bar_acc_data_processing.py 20260517               # a specific batch
python .../0_bar_acc_data_processing.py 20260517 --no-export   # don't write compiled JSON
python .../0_bar_acc_data_processing.py 20260517 --viewer      # 3D matplotlib per take

python src/husky-assembly-teleop/data/bar_holding_acc_data/0_bar_acc_data_processing.py 20260708

python src/husky-assembly-teleop/data/bar_holding_acc_data/1_compare_to_cell_state.py 20260708
```

Writes `compiled_bar_holding_acc.json` in the batch folder (unless `--no-export`).

**Per-take line, how to read it:**

| field | meaning |
|-------|---------|
| `ocf` | fitted bar mid-point (Object Coordinate Frame origin), metres, rhino frame |
| `d_ocf_from_take0` | signed OCF drift vs take 0 in this file (mm) — repeatability across takes |
| `axis` | fitted bar direction (unit vector) |
| `angle_to_Z` | angle between the bar axis and world +Z (deg); ~0 = vertical |
| `bar_len` | fitted tip-to-tip length (m) |
| `bar_len_err_vs_nominal` | `bar_len − max(bar_dimensions)` (mm); only shown when dims were stamped |
| `center_to_line_dist_max` / `_rms` | how well the pair mid-points sit on one line (mm). **Fit-quality gate**: a few mm or less = clean; large = bad marker pairing or noise, treat that take with suspicion |

Use `0_` to spot bad takes (large `center_to_line_dist_*`) and to check
run-to-run repeatability (`d_ocf_from_take0`) before trusting `1_`.

---

## `1_compare_to_cell_state.py` — compare to the intended pose

The `batch` argument is optional and **defaults to `20260706`**.

```bash
python .../1_compare_to_cell_state.py                          # default batch 20260706
python .../1_compare_to_cell_state.py 20260517                 # a specific batch
python .../1_compare_to_cell_state.py 20260517 --export        # write compared_to_cell_state.json
python .../1_compare_to_cell_state.py 20260517 --viewer        # 3D goal-vs-fitted plots
python .../1_compare_to_cell_state.py 20260517 --pp-viewer     # pybullet: cell state + goal bar + takes
python .../1_compare_to_cell_state.py 20260517 --movement M2 --bar-action /abs/path/B6.json   # overrides

python src/husky-assembly-teleop/data/bar_holding_acc_data/0_bar_acc_data_processing.py 20260708

python src/husky-assembly-teleop/data/bar_holding_acc_data/1_compare_to_cell_state.py 20260708
```

**Reference pose:** if the take has `bar_start_position/quaternion`, that stamped
pose is used directly (no BarAction parse). Otherwise the script falls back to
re-parsing the BarAction and deriving the goal from `target_ee_frames ∘ grasp`
(attached bar) or the installed `frame`. In that fallback it doesn't trust the
stamped filename: it scans the take's `BarActions/` folder (re-rooted onto this
machine), uses the file the take named when it's still there or else the first
one, and warns when several exist (pass `--bar-action` to pick).

**Per-take deviations, how to read them:**

| field | meaning |
|-------|---------|
| `start_dev` | distance from the fitted bar's lower tip to the reference pose origin (mm) — the primary position error (see caveat) |
| `angle_dev` | angle between fitted and reference bar axes (deg) |
| `lateral_dev` | perpendicular offset of the fitted OCF from the reference bar axis (mm) — sideways slip |
| `pos_dev(ocf↔goal)` / `d_ocf_vs_goal` | OCF-vs-reference (mm). **Not** the true centre error while the OCF-origin caveat holds |
| `d_mid_vs_goalmid` | mid-point-vs-mid-point per-axis diff (mm), reconstructed along the reference axis |

The `--export` JSON and the final `=== aggregate ===` block report mean / std /
max of `start_dev`, `angle_dev`, `lateral_dev`, and the fit-quality residual.

### ⚠️ OCF-origin caveat (temporary)

The rhino RobotCell export writes the bar's **lower tip** (smallest world-Z) as
the frame origin instead of the mid-point. So the reference `bar_start_position`
is a *tip*, not the centre. The script works around this by comparing the fitted
bar's lower tip to it (`start_dev`), which is the trustworthy position metric.
`pos_dev`/`d_ocf_vs_goal` compare mid-point to tip and will read ~half a bar
length off — kept only for reference. Remove this workaround once the export is
fixed.

---

## Layout diagram (assembly context panel)

`--viewer` draws **each bar-action in its own cell** (two per row), and overlays a
small **layout inset at that cell's top-right** showing **where this bar sits in
the whole assembly**: **origin** yellow, all bars **grey**, tested bar **red**,
environment **blue** (steelblue), and the **robot base** orange **parked for this
bar's action** (it differs per bar). `0_`'s inset is a **2D top view**; `1_`'s is a
**3D** view. With many bars the figure is tall and opens in a **scrollable window**
— drag the right scrollbar to see more rows (see Viewer controls below).

### Where each element's data comes from

| Element | Data source |
|---|---|
| **current tested bar** (red) | The take JSON's `bar_name` field (e.g. `bar_B2`) picks *which* bar; its geometry is the same as any whole-model bar below. For a batch, the set of all takes' `bar_name`s. Fallback if that bar isn't in the cell-state: the goal/fitted endpoints from the take. |
| **whole bars** (grey) | Poses from `<problem>/BarActions/*.solved_keyframe.json` → each bar's `frame` (point + x/y axes), read as **raw JSON**. Lengths from `<problem>/RobotCell.json` → bar mesh AABB. `<problem>` is resolved from the take's `bar_action_path` (or the `--problem` override). |
| **environment** (blue) | Meshes on layer **`Environment Obstacles`** in a Rhino `.3dm`. Auto-loaded from **`DEFAULT_ENV_3DM`** (see below) — no flag needed; override per-run with `--env-3dm <path>`. Optional `WalkableGround.json` floor when present (drawn only in `1_`'s 3D view). |
| **robot base** (orange) | `robot_base_frame.point` from the **tested bar's own** BarAction (`bar_action_path`). The base is parked per bar — constant across that bar's M0..M4 but different between bars — so it's read from the tested file, not an arbitrary one. |
| **world origin** (yellow) | Fixed `(0,0,0)` — the Rhino world origin; not read from any file. |

Two notes on alignment:
- Bars + robot base come from a **problem folder** (RobotCell / solved-keyframe);
  the environment comes from the **`.3dm`**. Different files, but both in
  Rhino-world coordinates, so they overlay (roughly — see the placement TODO).
- The `.3dm` lives outside any problem folder, so its path is a config value
  (`DEFAULT_ENV_3DM`) with a per-run `--env-3dm` override.

### Setting the environment `.3dm`

The environment auto-loads from **`DEFAULT_ENV_3DM`** in
[`husky_assembly_teleop/__init__.py`](../husky_assembly_teleop/__init__.py) (next to
`DESIGN_PROBLEM_NAME`) — so `--viewer` shows it with no extra flag. To use a
different file for one run, pass `--env-3dm "<abs path to .3dm>"` (the argument is
a real path, not a placeholder). To change it permanently, edit `DEFAULT_ENV_3DM`.
Reading the `.3dm` needs `rhino3dm` (`pip install rhino3dm`).

Read notes: bar world poses come from `*.solved_keyframe.json` as **raw JSON**
(not `parse_bar_action`, so it survives an in-progress `rs_data_structure`
refactor); `RobotCell.json` (~146 MB) is loaded once and cached; the `.3dm`
env layer is cached per path.

If no populated cell-state is found, it **degrades** to origin + the tested bar
only and prints a `[layout] no solved cell-state ...` note (never crashes).
Validated on `260715_phase1_test_batch_solve` (81 bars, full model).

**Viewer controls:** drag the **right scrollbar** to scroll through the bar-actions
(two per row); **mouse-wheel** over any subplot zooms it in/out (2D + 3D panels and
the marker-validation figure); **drag** to rotate the 3D panels; the toolbar
pans/saves. The scrollable window needs a Qt backend (the session default) — on
other backends it falls back to a single non-scrollable window. The per-take plot
titles are compact 3-line (`file`, `bar / bar_len`, `angle / ctr→line`).

**Flags** (both scripts) — two overrides drive the layout panel:

```bash
# force the layout model to any problem folder (abs path or name under DESIGN_DATA_DIRECTORY)
0_bar_acc_data_processing.py 20260708 --viewer --problem 260715_phase1_test_batch_solve
# use a DIFFERENT environment .3dm than DEFAULT_ENV_3DM (else it auto-loads)
0_bar_acc_data_processing.py 20260708 --viewer \
  --env-3dm "…/assembly - demo/260715_phase1_test_v2.3dm"
```

> **⚠️ TODO (important): Rhino environment placement accuracy.** The environment +
> bars are placed **roughly** in Rhino (by hand, from approximate mocap-camera
> positions). The Rhino-origin ↔ bar relationship is internally consistent, but is
> **not** accurately tied to the real world — so the diagram shows bar positions
> only roughly. A calibrated Rhino↔mocap placement method should be worked out
> later (deferred on purpose; do not fix inline). The panel draws the fixed world
> origin (0,0,0, yellow) and the robot base (orange square) as separate labeled
> markers; the world origin's tie to the real cell is still uncalibrated.

---

## Old data compatibility

- **Wrong home dir in `bar_action_path`.** Takes stamped on another machine
  (e.g. `/home/yijiangh/...`) are re-rooted onto this machine's Google Drive
  copy automatically (any path under `2025-03 Husky Assembly`).
- **BarAction won't parse.** Some older BarActions reference renamed classes;
  `1_` skips those takes with a message instead of crashing. Newer takes carry
  the stamped `bar_start_position`, so they don't need the BarAction at all.
- **Missing metadata.** Takes with no `bar_action_path` and no stamped pose
  (e.g. recorded before a movement was loaded) are skipped by `1_`; `0_` still
  fits and reports them (it needs only the markers).
