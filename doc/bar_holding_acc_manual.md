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

## Running a session (live monitor)

This is the **mount-once** protocol: the instrumented bar is mounted in the
grippers once at the start of the session and **never dismounted**. For every
bar-action you drive the mobile base to that action's parked pose, let the
visual-servoing loop bring the held bar onto the action's assembled pose with
the **constrained (bar-held) transfer planner**, record marker takes, and move
on to the next bar. Bars already "built" earlier in the sequence are *not*
physically there — they only serve as different reaching locations — so their
collisions are ignored automatically (see the `[mocap-acc]` log line in Step B).

> The bar/movement metadata (`bar_action_path`, `movement_id`, `bar_start_*`,
> `bar_dimensions`) is stamped from the **currently loaded movement**, so you
> must Load BarAction *and* Load Movement before recording — otherwise those
> fields save as `null` and the take can't be matched later.

---

### Pre-flight checklist (per session)

**1. Monitor flags** — class attributes on `HuskyMonitor`
([`husky_monitor.py`](../husky_assembly_teleop/husky_monitor.py)). The first
three move together; the file carries the same note next to
`BAR_ACTION_MOCAP_ACCURACY_TEST`.

| Flag | Line | Mocap accuracy test | Robot-centric demo |
|------|------|---------------------|--------------------|
| `USE_MOCAP` | ~239 | **1** | 0 |
| `USE_CELL_STATE_BASE_POSE` | ~259 | **0** | 1 |
| `BAR_ACTION_MOCAP_ACCURACY_TEST` | ~275 | **1** | 0 |
| `FAKE_HARDWARE` | ~240 | 0 | 0 |
| `BAR_ACTION_LIVE_REPLAN_EXE` | ~269 | 1 | 1 |
| `CONNECT_COMPLIANT_CONTROLLER` | ~337 | 1 | 1 |
| `USE_DPG_UI` | ~260 | 1 | 1 |
| `MOCK_LIVE_POSE_FOR_REPLAN` | ~224 | 0 | 0 |
| `REPLAN_SKIP_ENV_COLLISIONS_IN_MOTION_PLAN` | ~234 | 0 | 0 |
| `CALIBRATION`, `PUNCH_CALIB_VALIDATION`, `DUAL_ARM_*` | — | 0 | 0 |

With `USE_MOCAP=0` no mocap client starts; with `USE_CELL_STATE_BASE_POSE=1`
the live base is never written into the movement state
(`_base_pose_is_tracked()`), so every replan would silently plan from the
authored base. With `BAR_ACTION_MOCAP_ACCURACY_TEST=0` the Record / Servo
buttons are not built at all.

**2. Design problem** — `DESIGN_PROBLEM_NAME = '260716_phase1_test'` in
[`__init__.py`](../husky_assembly_teleop/__init__.py) (~line 60). It holds 27
bar-actions `B3 … B81`; all use the same **1.4 m** bar and the *same* grasp
frame, so one physical bar + rig serves the whole session. The `BarActions/`
folder also contains `B33.solved_motion.json` / `B33.solved_keyframe.json`,
which are **not** actions — the monitor prints the indexed file list at
startup, read it and skip those two indices.

**3. Calibration date** — `CALIBRATION_DATE`
([`__init__.py:80`](../husky_assembly_teleop/__init__.py#L80)), with the offline
scripts' `CALIBRATION_ANALYSIS_DATE` right below it. The date folders
on the gdrive under `data_experiment/calibration_data/` (`CALIBRATION_DATA_DIRECTORY`,
see [`calibration_manual.md` §4.3](calibration_manual.md#43-data-folder-structure)) are
**per robot and per arm**, not a timeline:

| Folder | Robot / arm | Output file |
|--------|-------------|-------------|
| `20260622` | Cindy 0806, **left** arm | `calibrated_transformation_0806_rhino.json` |
| `20260623` | Alice 0804, single arm | `calibrated_transformation_0804_rhino.json` |
| `20260625` | Cindy 0806, **right** arm | `calibrated_transformation_0806_rhino.json` |

Use **`20260622`** for this experiment. The monitor consumes only
`base_mocap_from_base_footprint` from the chosen file (the arm mount offsets
come from the URDF, not from here), and 0622/0625 are two independent estimates
of that one transform which differ by ≈(1.0, 2.9, 0.6) mm and ≈0.2°. The bar's
world pose is defined through the **left** tool0 in every BarAction
(`attached_to_link = left_ur_arm_tool0`), so the left-arm calibration keeps that
chain exact; all previous takes (20260716 … 20260806) used 0622 too. Expect that
≈3 mm / 0.2° as the floor of the **right** arm's residual — it is the left/right
calibration discrepancy, not a servoing failure. `20260623` is Alice's
single-arm calibration and does not apply to Cindy at all.

**4. Motive**

- Rigid bodies with the right **Streaming IDs**: husky base `a200_0806` = **1860**
  (`ROBOT_CONFIGS` in [`husky_world.py`](../husky_assembly_teleop/husky_world.py)),
  bar rig = **1002** (the `bar_rig` `TrackedObject`, ~line 346; the name comes
  from `MOCAP_SET_RIG_RB_NAME`). If Motive shows a different id for the bar rig,
  change it in Motive rather than in the code.
- In **Edit > Settings > Streaming**: NatNet enabled, and **both labeled and
  unlabeled markers ON** — the fit reads the labeled-marker stream, not the
  rigid body (see [`check_mocap_data.md`](../data/bar_holding_acc_data/check_mocap_data.md)).
- `CLIENT_IP` / `MOCAP_IP` at the top of `husky_monitor.py` (~216-217) must match
  this workstation and the Motive PC.

**5. Robot**

- On the husky: `ros2 launch crl_husky crl_dual_ur5e.launch.py namespace:='/a200_0806' gripper:=none`
- Pendant in **Remote** mode, with the **correct tool TCP loaded**. A constant
  tool-Z offset in the "not in correct start pose!" message is a *pendant tool
  setting*, not a code frame bug — check the pendant first.
- In every workstation terminal (these are **not** in `~/.bashrc`):
  ```bash
  export ROS_DOMAIN_ID=86
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ```

**6. Launch**

```bash
cd /home/su/ros2_ws
source venv/bin/activate
python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop
source install/setup.bash
ros2 run husky_assembly_teleop husky_monitor
```

Confirm on startup: the green `mocap client connected: True` line, the indexed
`Found N BarAction files:` list, and the real (coloured) husky moving in
PyBullet when you nudge the base.

---

### Step A — mount the bar (first action only)

The mount pose is **M1's start**, the bar-loading configuration; M0 is the free
motion that takes the arms there. In this protocol M1 itself is **never
executed** (Step B transfers from the bar-loading pose straight to each bar's
assembled pose), so only M1's *start* is needed. **You choose it by hand** —
the automatic derivation (a 120 s sweep) is kept as a fallback, see the note.

1. Set **BarAction file (idx)** to your first action (e.g. `B3.json`) and
   click **Load BarAction**.
2. **Movement (idx; 0=M0_synth)** → **1** → **Load Movement**. Then pick the
   bar-loading pose on the sliders and confirm it:
   - **M1 home anchor** — the carry: `1` horizontal (bar across the front),
     `2` vertical (bar upright in front), `3` back (bar fore-aft over the
     robot); `0` tries them in that order and keeps the first that works.
   - **M1 manual start: slide along bar (m)** — along the bar's own axis
     (`+` toward the left gripper's end).
   - **M1 manual start: roll about bar (deg)** — turns the bar about its own
     axis (the tools swing with it). This is the knob for the "bar rolled 180°
     between start and goal" look: try `0` and `180`.
   - **shift perp. 1 / perp. 2 (m)** — moves the bar along the two robot-base
     axes that are perpendicular to it (the log names them on every confirm):

     | anchor | bar along | perp. 1 | perp. 2 |
     |---|---|---|---|
     | horizontal | base y (left) | forward (x) | up (z) |
     | vertical | base z (up) | forward (x) | left (y) |
     | back | base x (forward) | left (y) | up (z) |

   - As soon as M1 is loaded, and while you drag any of these sliders, a
     **see-through orange bar** (a cylinder with a stub at each grasp point
     showing where the tools sit, so the roll is visible) in PyBullet shows
     the pose you are describing, placed at the live base (no IK yet — the
     arms appear after Confirm; anchor `0` previews horizontal).
   - **M1: Confirm manual start pose (IK check)** — solves the dual-arm IK
     that holds the bar there with the same grasps as at the goal (branch
     nearest the goal), checks it against the full cell, and prints the bar
     pose in the robot frame plus the verdict. On success it shows both
     endpoints (**Traj viz time** `0` = START, `1` = GOAL on the green preview
     robot, bar in the grippers; **Constrained t** on the PyBullet panel steps
     the red cfab robot) and **adopts the start at once** — M1's start and
     M0's goal are written (and the `.live-solved.json` sidecar too when the
     *Adopt also saves* toggle is ticked). Each confirm takes well under a
     second: adjust and confirm again until the pose looks right for mounting.
   - If it fails: *no arm configuration holds the bar there* = out of reach
     (slide / shift the bar closer, or another anchor); *the arms collide* =
     the colliding pair is drawn (usually the bar or a forearm against the
     base) — roll, slide or shift the bar, or change the anchor.
3. **Movement (idx; 0=M0_synth)** → **0** → **Load Movement** →
   **Plan Movement**: M0 is the free dual-arm motion from wherever the arms are
   now to that start.

   > *Alternatives*: **M1: Derive Start/Goal only (no RRT)** runs the
   > automatic start derivation (up to 120 s, may use the whole budget and is
   > sensitive to the base pose — see `m1_planner_changelog.md`), then
   > **M1: Adopt derived start -> M0 goal**. **Plan Movement** on M1 runs the
   > BiRRT from whichever start is stored (several minutes) and ends in the
   > same state, plus an M1 trajectory you will not use here.
4. Scrub **Traj viz time** to preview the M0 path, set **traj time** to ≥ 20 s,
   then **Exec Selected Mv Traj (auto)** (M0 runs joint tracking and re-zeros
   the force sensors at the end — the only safe place to tare).
5. With the arms parked at the bar-loading pose: fit **four pairs** of mocap
   markers on the bar. The **outer two pairs must sit at the bar's ends**; the
   inner two pairs' exact positions don't matter (the fit pairs markers by
   cross-bar distance and uses the end pairs for length and axis).
6. Mount the bar (with rig) into both tools. Fit the two male joints if you want
   the physical geometry to match — the marker fit ignores them, and the
   collision model already carries them either way.
7. Sanity check: the red rig cylinder in PyBullet follows the real bar, and one
   **Record + Fit + Viz (shared)** click reports `bar_len ≈ 1.400 m` with a small
   `max_resid`. Then click **Discard unsaved takes**. This take was made at the
   bar-loading pose, which is *not* a measurement pose — there is no valid
   reference for it (M0's bar frame is a placeholder; M1 has no predecessor
   targets) — and every recorded take stays in memory until the next Save, so
   without the discard it would land in the first bar's file in Step B and be
   scored against that bar's assembled pose. **Never press `Save markerset
   data` in Step A.**

> **If no start can be confirmed or M0 will not plan**, fall back to the older
> protocol for the mount only: **Movement (idx; 0=M0_synth)** → **3** →
> **Load Movement** → drive
> the base to the ghost → **3) Servo to Mv Start (live loop)** (the *free*
> planner, no bar mounted) → mount the bar at the assembled pose. Later bars do
> not need M1 at all.

Step B below is unchanged by the manual start: the transfer loop always plans
from the live arm configuration (the bar already in the grippers) and never
calls the start derivation.

---

#### Diagnosing a slow or failing M1 derivation

The derivation tries a thousand home bar poses and normally uses its whole
120 s budget. Every run — from the button above **or** from the headless
script — is recorded for a small web dashboard that shows which home poses were
tried, why each one failed (no IK solution / the IK jumped to another branch /
which link hit which body), where the time went, and the scene in 3D.

Start it once per session, in its own terminal:

```bash
cd /home/su/ros2_ws
source venv/bin/activate && source install/setup.bash
bash src/husky-assembly-teleop/scripts/fetch_dashboard_vendor.sh   # first time only
python src/husky-assembly-teleop/scripts/m1_dashboard_server.py    # http://127.0.0.1:8765
```

Open the page and leave it open: each new derivation appears by itself (it can
open automatically, or wait for you to click **Open**). Click any dot in the
charts to inspect that attempt in the 3D viewer — drag to orbit, scrub the
slider to walk the bar from the goal pose to where the attempt died, and the
colliding link and body are highlighted.

**How to read what it shows — including what each outcome means and how to tell
a reach problem from a branch problem — is in
[`m1_derive_dashboard.md`](m1_derive_dashboard.md).**

To reproduce a derivation at the desk, with no robot and no mocap:

```bash
python src/husky-assembly-teleop/scripts/derive_m1_headless.py --bar B3
python src/husky-assembly-teleop/scripts/derive_m1_headless.py --bar B3 --anchor back
```

Run files land in `recorded_data/m1_derive_runs/` and the baked 3D scene in
`recorded_data/m1_dashboard_scenes/<problem>/` (exported once per design
problem, a few seconds). Both are local and git-ignored — copy a run to the
Drive by hand if it is worth keeping.

---

### Step B — per-bar loop (bar stays mounted)

Repeat for each bar-action. Nothing here dismounts the bar.

1. **BarAction file (idx)** → next action → **Load BarAction**, then
   **Movement (idx; 0=M0_synth)** → **3** (M3 = the assembled pose, the same
   reference the 20260717 takes used) → **Load Movement**.
   Expect the log line
   `[mocap-acc] ignoring collisions with N built assembly bodies during planning/IK.`
   and the already-built bars to disappear from the view — that is the
   "previous bars are only reaching locations" rule being applied. The ghost
   robot now stands at this action's parked base pose, and a **thick pink
   line** (with a label) marks the bar's central axis where it sits when
   assembled (M3's start pose, the reference the takes are scored against);
   it stays until the next Load BarAction (log line `[BarAction] pink line =
   bar_B… at its assembled pose`).
2. **Drive the mobile base** (joystick, or
   `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/a200_0806/joy_teleop/cmd_vel`)
   until the mocap-tracked husky roughly overlaps the ghost — i.e. until the
   bar in the grippers can plausibly reach the pink line. A few centimetres
   and a couple of degrees are fine — the servo loop absorbs the rest.
3. **Set `traj time` to 20–30 s.** Load Movement resets it to M3's 5 s default,
   which is far too fast for the first transfer of a bar. (The loop re-reads the
   slider when you confirm, so you can also drag it during the pause in step 4.)
4. **3b) Servo to Mv Start (transfer loop)** — every iteration plans a *bar-held
   constrained transfer*, so both tool0s stay rigidly locked to the mounted bar.
   - Iteration 1 = live-base IK + constrained plan (up to 120 s; the GUI is
     deliberately frozen during the search), then a **confirm pause**: the log
     prints the waypoint count, the max joint delta and the duration. Scrub
     **Traj viz time**, check the **Movement Preview** window (max joint step,
     bar-hold EE drift), then click **Confirm Exec**.
   - Later iterations run unattended; a safeguard pause appears for any plan
     over 10 waypoints or 5°. **Cancel Exec** stops before the next send (a
     trajectory already sent still finishes).
   - Watch progress with **Toggle Servoing Tracker**. The loop stops when both
     arms are under 0.2 mm or after 8 iterations, and auto-saves
     `servoing_data_<ts>.json` + `servoing_performance_<ts>.png` under
     `<YYYYMMDD>-servoing/`. A typical good run: iteration 1 leaves a few mm,
     by iteration 3–5 both arms sit at 0.2–0.5 mm.
5. **Record + Fit + Viz (shared)** once per take — take at least 3, watching
   `max_resid` (a few mm or less) and `bar_len` (≈ 1.400 m). A bad fit can be
   thrown out with **Discard unsaved takes** (it drops *all* takes recorded
   since the last Save, so re-record the good ones). Then **Save markerset
   data** once for this bar — one file, one reference pose, all its takes. The log prints the saved path; the
   stamped reference pose must not be null — an `ERROR ... NULL reference pose`
   line means the take was saved without one (keep it, the offline script can
   re-derive it from the BarAction).
6. Go back to 1 for the next bar. Do **not** press:
   - **Exec Selected Mv Traj (auto)** while M3 is loaded — for M2/M3 it
     dispatches to the *cartesian compliance controller*, which is the assembly
     execution path, not this measurement path;
   - **Move Arms to Movement Start (offline target)** — it drives to the
     authored configuration, which belongs to the authored base, not the live one.

---

### Step C — wrap up

1. Unmount the bar and rig.
2. Optionally send the arms home: **Movement (idx; 0=M0_synth)** → **4** →
   **Load Movement** →
   **Plan Movement** → **Exec Selected Mv Traj (auto)**.
3. Everything is already on the Drive under
   `EXPERIMENT_DATA_DIRECTORY/bar_holding_acc_data/<YYYYMMDD>/`. Process it with
   `0_bar_acc_data_processing.py` then `1_compare_to_cell_state.py` (below),
   passing today's date folder as the batch.

---

### Troubleshooting

- **`IK at live base FAILED`** — the collision diagnosis draws and highlights
  whatever rejected the solution. Usually the base is parked too far off or at
  the wrong yaw: re-park closer to the ghost and retry. Environment obstacles
  are still collision-checked (only the *built bars* are ignored), and their
  Rhino placement is approximate, so a marginal park can read as a collision.
- **`[M1 manual] the arms collide holding the bar there`** — the drawn pair says
  what blocks (forearm / bar against the base or a tool): roll the bar 180°,
  slide it along its axis, or shift it forward/up with the perpendicular
  sliders; `vertical` hangs the bar in front of the base with its lower end
  near the floor, `back` carries it over the robot.
- **`GOAL COLLISION: CC.4 between attached rigid body 'bar_…' and rigid body
  'joint_…_male'`** on 3b — the bar and its own fitted joints were re-attached
  for the mount-once test without the allowed-touch lists of the movement they
  were copied from, so cfab checked the two overlapping held bodies against
  each other. Fixed on 2026-09-28 (`_ensure_bar_attached_for_mocap` now carries
  the touch lists along and lets every held body touch every other held body);
  if it reappears, the loaded movement's M2 sibling lacks those lists.
- **`[transfer plan] constrained plan failed`** — click **3b** again (the
  planner is randomized), or improve the base alignment first. The very first
  transfer of a session (bar-loading pose → assembled pose) is the longest path
  and the most likely to need a retry.
- **A slider seems to be ignored** — any UI rebuild (`Load Movement`, live-base
  IK) recreates the widgets, and a freshly rebuilt slider can miss its next drag
  callback. Drag it again and read the printed value; `traj time`, the M2 split,
  the swept-check and the BarAction/Movement sliders are all re-read live at the
  moment they matter.
- **The UI drops to ~1 fps** — a known intermittent DearPyGui vsync stall, not a
  planner problem. Restart the monitor before concluding anything from timings.
- **Base drifts ~1 mm between iterations** — expected: the arms' centre of
  gravity moves the husky on its suspension. That is exactly what the loop
  re-solves for, and why the residual converges rather than jumping.
- **Right arm plateaus around 3 mm / 0.2°** — that is the left/right calibration
  discrepancy described in the pre-flight checklist, not something servoing can
  remove.
- **`mocap client connected: False`** — check `CLIENT_IP` / `MOCAP_IP`, the
  Motive streaming pane, and that both machines are on the same network.

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
