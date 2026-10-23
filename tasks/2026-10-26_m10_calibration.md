# M10: calibration as a plugin in the new monitor (spec)

**Summary:**
- A new `calibration` plugin runs the old move-stop-record sweeps as cancellable tasks under the robot lease. It also records validation points and punch takes, fits, and writes the result where the relay reads it.
- The math is the old pipeline's, ported to plain numpy. A scratch re-computation already reproduces three stored sessions within 0.12 µm and 1e-5° (§0.11).
- The relay gains a second topic per robot body, `raw_pose`, so the monitor always records uncalibrated base poses.
- Nothing in crl-husky has to change.

**Layout.** This spec targets the merged layout: branch `yh/core-port-jg-sync`, worktree
`~/Code/ros2_ws_next/dev/jg-sync` (HEAD 93561f5 or later). Paths below are relative to that worktree.

> **If the merge is not adopted**, the work lands on `yh/core-port` (9165944, worktree
> `~/Code/ros2_ws_next/src/husky-assembly-teleop`), which has the older layout. Mapping (from
> `tasks/2026-10-25_jg_sync_60d011a.md` §3.2–3.3):
>
> | Here (merged) | At 9165944 |
> |---|---|
> | `shared/calibration/*` (new) | the same path. Helpers may also sit at the top of `shared/` there, but keep the subfolder |
> | `shared/arm_planner_contract.py`, `ArmPlannerContract.request_plan(ArmPlanRequest)` | **missing.** `plugins/arm_planner/plugin.py` is a panel plugin with no headless API, so the plan's edit 5 must be built first: `request_target(serial, arm, goal) -> Future[ArmPath \| str]` on it, plus a Protocol in a new `shared/arm_planner.py` |
> | `ArmInterface.safety_problem()` (arm.py:534) | **missing** (Jakob's 397f174). Drop that check. `START_TOLERANCE` there is 0.1 rad, not 0.01 |
> | `robot_interface/mocap_state.pose_lost` | `world/mocap.pose_lost` |
> | `shared/execution/{interlock,operator,send,retime}.py` | the same paths and names (`refuse_while_held`, `busy_reason`, `base_moving`) |
> | `shared/runs/runs.py` (`record_event`, `commits`) | `shared/runs.py` |
> | `shared/assembly_planning/joints.arm_joint_names` | `shared/planning/joints.arm_joint_names` |
> | `test/<area>/test_*.py`, `test/structure/test_layers.py` | flat `test/test_*.py`, `test/test_layers.py` |
> | the fake arm with the safety sync (`test/plugin_fakes.py`) | the fake arm without it |

**Assumptions** (the user can override any of them; each also appears in §10):
- A1. One plugin, `calibration`, also does punch validation, as the approved plan's row says. The roadmap's separate `punch_validation` row is folded into it.
- A2. One session folder holds one robot and one arm (today's rule).
- A3. Only the robot's reference arm writes the file the relay reads: `ur_arm` for Alice and Belle, `left_ur_arm` for Cindy.
- A4. The old record files stay readable by the old scripts 0_–4_. The old scripts themselves are not ported in M10; the new code replaces their math (§0).

---

## 0. Math first (for approval before any motion code)

All poses are (x, y, z, qx, qy, qz, qw), in metres. "T·U" composes poses; "T⁻¹" inverts one. Old code is cited as
`<file>:<line>` at 0e45758:
- `W` = `husky_assembly_teleop/husky_world.py`;
- `M` = `husky_assembly_teleop/husky_monitor.py`;
- `0_`…`4_` = `data/calibration_data/<n>_*.py`.

New code is cited at the sidecar's HEAD.

### 0.1 Frames

| Symbol | Frame / pose | Where it comes from |
|---|---|---|
| W | mocap world, Z-up, origin = Motive's origin | NatNet Y-up → Z-up: (x, y, z) → (x, −z, y), q → (qx, −qz, qy, qw). Old `utils.py:43-57` and relay `node.py:444-445` are identical |
| R = ᵂT_M | **raw** pose of the robot's base rigid body M (1840 Alice, 1850 Belle, 1860 Cindy) | Motive, after the remap above |
| B = ᵂT_F | base_footprint F, the URDF root (`world_link` ≡ `base_footprint`) | published by the relay as `pose` |
| C = ᴹT_F | `base_mocap_from_base_footprint`, the base calibration | the file the relay applies |
| A = ᶠT_L | L = `<arm>_base_link_inertia`, through the URDF's fixed joints | `robot_interface/frames.py:24 fixed_transform(urdf, "base_footprint", f"{arm}_base_link_inertia")` |
| K = ᴹT_L | `base_mocap_from_<arm>_base_link_inertia`, the measured arm base | the fit (§0.6) |
| ᵂT_T | raw pose of the calibration tool's rigid body T (Alice 1862, Belle 1866, Cindy left 1866, Cindy right 1861; old `W:67-69`, origin c9e63b9 8bfaa25). The old records call it `flange_mocap_pose` | Motive; tool bodies are never calibrated by the relay |
| FK_s(q) = ᶠT_tool0 | `<arm>_tool0` from base_footprint at joints q | the robot's URDF copy with its UR calibration applied (`RobotConfig.model.urdf`) |

Checked: A from husky_urdf aa6a261 (old) and from 4d0c854 (new) gives the same C for all three robots (both differ from
the stored files by the same 0.05–0.11 µm). Jakob's fix kept every link but `<arm>_base_link` in place
(`doc/ur_frames.md`).

### 0.2 Raw poses: what the relay publishes, and how a raw pose is recovered

The relay applies the calibration for robot bodies (`mocap_relay/node.py:119-123`, `:446-448`):
```
p_B = p_R + rot(q_R)·p_C ,   q_B = q_R ⊗ q_C            i.e.  B = R·C
```
- **Before M10:** `/mocap/rigid_body/id_<n>/pose` = B for robot bodies, and R for every other body. Nothing publishes a robot's R.
- **After M10:** in addition, every robot body that has a calibration (all three robots always have crl-husky's default) is published as R on `/mocap/rigid_body/id_<n>/raw_pose`. It is the same `MocapRigidBodyPose` message, with the same stamp, validity and markers.

Recovery and identification (new `shared/calibration/poses.py`):
```
remove:  R = B·C⁻¹ :  q_R = q_B ⊗ q_C* ,  p_R = p_B − rot(q_R)·p_C
applied: C = R⁻¹·B :  q_C = q_R* ⊗ q_B ,  p_C = rot(q_R*)·(p_B − p_R)      (a raw_pose / pose pair with the same stamp)
```
The records always store R (as the old monitor did: its NatNet cache held raw poses, `M:13348-13389`, and it applied C
itself at `M:13386`). The data therefore never depends on which C the relay applied during collection. The pair rule
lets the panel show which calibration the relay applies right now, by comparing the C it recovers with:
- the file just accepted;
- the robot's newest Drive session;
- crl-husky's default.

That is how "the relay picks it up after reload" becomes visible.

### 0.3 One point (move, stop, record)

At point i of a sweep, after the stop test (§0.10), the record holds:
- R̄ᵢ and T̄ᵢ, the mean raw poses over the still window;
- q̄ᵢ, the mean measured joints.

The fit uses the tool origin in M:
```
pᵢ = translation( R̄ᵢ⁻¹ · T̄ᵢ )                                  old 0_:72-79
```
**New:**
- The pose is the mean over 1 s, not the last cached sample (`W:881-887`).
- The joints are the measured ones. The old loop overwrote them with the commanded waypoint (`W:1480`). The stored old records are a mix: 0–0.004° from the waypoints, and Belle's J0 traj5 ended 6.5° short of its commanded 367°.
- The old points were transformed with PyBullet's float32 `multiply`/`invert`. The new code uses float64, which explains the 1.2e-7 m difference in §0.11.

### 0.4 Circle fit per take (old `0_:83`: `circle_fitting_3d.Circle3D` 1.0.0 on scikit-spatial 9.0.1)

For the N unique points (rows sorted as `np.unique` sorts them):
```
m  = mean pᵢ ;  [U,S,·] = svd((P − m)ᵀ) ;  n = U[:,2]                       (best-fit plane; sign arbitrary)
qᵢ = pᵢ − ((pᵢ − m)·n̂) n̂                                                  (projection; mean of q = m)
u  = unit((q₁ − m) − (q₀ − m)) (first two rows) ; v = u × n̂ ; (xᵢ, yᵢ) = ((qᵢ − m)·u , (qᵢ − m)·v)
[a b e] = argmin ‖ [2x 2y 1]·[a b e]ᵀ − (x² + y²) ‖²                       (Kasa algebraic fit)
centre c = m + a·u + b·v ;  radius r = √(e + a² + b²) ;  normal n̂
residual dᵢ = ‖ c + r·unit(qᵢ − c) − pᵢ ‖                                    old 0_:88-101
```
**New:** the same formula in numpy, so the two libraries, which are not in the new venv, are not needed. Each take keeps its centre, normal, radius and max dᵢ.

### 0.5 Joint axis lines and their positive directions (old `1_:430-481`, `1_:499-530`)

Per batch (j0 sweeps J0, j1 sweeps J1), over its takes k = 1…n:
```
nₖ ← nₖ · sign(nₖ·n₁)                       NEW: align normals before averaging (old averaged raw SVD signs, 1_:443)
N̄  = unit(Σ nₖ) ;  line point a = mean(cₖ) (= the old "projected centres" anchor)
quality: ∠(nₖ, N̄) per take ; ‖(cₖ − a) − ((cₖ − a)·N̄)N̄‖ per take
positive axis â (right-hand rule from the commanded motion), per take k and consecutive points i:
  sᵢ = sign( ((pᵢ − cₖ) × (pᵢ₊₁ − cₖ)) · nₖ ) · sign(qⱼ,ᵢ₊₁ − qⱼ,ᵢ) ;  âₖ = nₖ if Σsᵢ > 0 else −nₖ ;  â = unit(Σ âₖ)
```
The normal alignment changes nothing on 20260623, 20261013 and 20261016 (all normals already agreed), but a flipped SVD
sign would have silently bent the old axis. Old `1_:583-594` also flipped signs by URDF-nominal axes (PyBullet FK).
`1_:607-615` then always overrides that by the motion sign, so the nominal step cannot change the result. **New:** it is dropped.

### 0.6 The arm base frame K (old `1_:533-788`)

```
v0 = N̄₀ oriented so v0·â₀ > 0                 (+J0)
y  = N̄₁ oriented so N̄₁·(−â₁) > 0             (the old code reaches y = −(+J1) on both branches: 1_:607-615 with :705/:746)
k  = unit(v0 × y)
plane P0: through a₀ (J0 line point), normal y ;  plane P1: through a₁ (J1 line point), normal k
ℓ  = P0 ∩ P1 , direction unit(y × k) ;  p01 = projection of a₁ onto ℓ          (old 1_:648-694)
d1 = z of the `<arm>_shoulder_pan_joint` origin in the URDF (old 1_:60-86; 0.16258–0.16278 m on the three arms)
z  = v0 ;  x = unit(y × z) ;  z ← unit(x × y)                                   (y kept exactly; old 1_:705-711, :744-751)
o  = p01 − d1·z                                                                 (old 1_:767)
K  = [x y z o]
```
The old code has two branches, one for the dual-arm robot and one for single-arm robots. Both reduce to the same
rule: z = +J0 and y = −(+J1), for every arm. **New:** one rule, no robot-name branch. Checked: this rule reproduces the
stored K of all three sessions to 1e-16.

Preconditions, checked when a reference trajectory is loaded:
- every J1 sweep holds J0 at 0 (within 0.5°), since its axis is −y_L only there (true for every reference file: all 34 are single-joint sweeps, and all 16 J1 sweeps sit at J0 = 0);
- every sweep moves exactly one joint.

**New metric**, reported only: the skew distance between the two axis lines, |(a₁ − a₀)·(v0×y)| / ‖v0×y‖. On a UR the
axes meet.

### 0.7 Base calibration C and what the relay reads (old `2_:97-103`, `2_:148-154`)

```
C = K · A⁻¹                                   base_mocap_from_base_footprint
```
The output is `<session>/calibrated_transformation_<digits>_rhino.json`, with:
- first the old keys, in the old order: `base_mocap_from_<arm>_base_link_inertia` = pose(K), then `base_mocap_from_base_footprint` = C;
- then, **new**: `mocap_id` (the base body, `RobotConfig.mocap_id`), `robot_serial`, `arm`.

The relay's `read_session_file` checks `mocap_id` (`mocap_relay/calibration.py:152`). Its hash covers only the
`base_mocap_from_*` keys (`:94-97`), so the extra keys do not change the hash.

Quaternion signs:
- K uses the same Shepperd conversion as the old `pp.pose_from_tform`: `pybullet_planning.utils.transformations.quaternion_from_matrix`, which may give w < 0, as in 20261013.
- C is turned to w ≥ 0, which matches all three stored files.

The old C was float32, from PyBullet's `multiplyTransforms`; the new one is float64.

### 0.8 Verification metrics (old `3_:93-140`, `3_:258-297`; `4_:127-176`)

Validation entry e (robot still, anywhere in the room):
```
Bₑ = R̄ₑ·C ;  Oₑ = (Bₑ·FK(q̄ₑ))⁻¹ · T̄ₑ        (tool0_from_flange_mocap: constant if C is right)
offset spread  = maxₑ ‖p(Oₑ) − mean p(O)‖     NEW headline; old reported ‖p(Oₑ)‖ − mean‖p(O)‖ (3_:268-269), kept as a second number
axis spread    = max over x, y, z of maxₑ ∠(axisₑ, mean axis)                 (3_:286-297)
```
Punch take j, with the tip offset o from the session's config.yaml (`punch_tool.<left|right>.offset_xyz`; old
`M:1756-1792`). Single-arm robots use `left`:
```
ᵂT_tip,j = B_j · FK(q_j) · Trans(o)         (B_j from the relay, i.e. the calibration in use)    old W:1059-1100
tip spread = max_j ‖p_j − mean p‖ ; pairwise max ‖p_j − p_k‖                                    old 4_:127-147
```
Comparison of two calibrations, as the relay's `offset` does (`calibration.py:157-162`):
```
Δp = ‖p(C₁) − p(C₂)‖ ,  Δθ = ∠(R(C₂)ᵀ R(C₁))
```

### 0.9 Acceptance thresholds

The Accept gate refuses to write the relay's file on any "refuse" line. Warnings are shown, and the Accept prompt turns
orange. The numbers in brackets are the last good sessions, Belle 20261013 and Cindy 20261016.

| Check | Warn | Refuse |
|---|---|---|
| points per take | – | < 20 |
| takes per batch (j0, j1) | < 5 | < 4 (old checklist "at least 4 circles") |
| circle residual, max dᵢ per take [0.30–1.08 mm] | > 1.5 mm | > 3.0 mm |
| normal spread per batch, max ∠(nₖ, N̄) [0.01–0.05°] | > 0.2° | > 0.5° |
| centres to their line [0.11–0.53 mm] | > 1.0 mm | > 2.0 mm |
| axis skew distance (new) | > 2 mm | – |
| validation points | < 10 | < 5 |
| validation offset spread [max 1.6–2.4 mm, std 0.7–1.1] | > 3 mm | > 5 mm (old goal "95 % under 5 mm") |
| validation axis spread [0.18–0.50°] | > 1° | > 2° |
| Δ(C_new, C in use) | > 5 mm or 0.5° | – |
| Δ(C_new, crl-husky's default) | – | > 5 cm or 5° (the relay would ignore the file: `calibration.py:165-172`) |

Punch (after Accept and the relay reload; reported, not gated): tip spread ≤ 2 mm (Belle: within 1.5 mm of the mean;
Cindy: 0.7 mm RMS).

**Lab acceptance** (the plan's M10): Δ(C_new, the robot's previous session) ≤ 1 mm and ≤ 0.1°. Cindy 20261016 vs
20260916 was 0.8 mm / 0.02°.

### 0.10 Motion rules: move, stop, record

Joint targets:
- The reference sweep's waypoints W₀…W₃₀, read as they are (radians, `data.points[i].joint_values`).
- Joint names must equal `arm_joint_names(arm)`; a file for another arm is refused.
- Consecutive waypoints may differ on one joint only, by at most 15° (real files: 12° on J0, 4.8–6.4° on J1).
- The moving joint must match the J number in the file name.

Limits:
- Every waypoint must be inside the arm's URDF joint limits.
- **New rule (Q16):** waypoints past a limit are dropped from the end of the sweep only, if at least 20 remain; otherwise the file is refused. Today this cuts Alice/Belle `J0_traj5` at waypoint 30 (367° > 360°), and every Cindy right-arm J0 sweep after waypoint 22 (shoulder_pan > 270°).
- Old: no check; the UR stopped Belle at 361°.

Transit to W₀:
- planned by `arm_planner` (`ArmPlanRequest(serial, arm, W₀)`), sent as one command from the live joints;
- default 20 s (old manual: 20–40 s); skipped if every joint is within 0.5° of W₀.

Each step k:
```
rows = [q_live, W_k] ;  motion_command(serial, arms, arm_joint_names(arm), rows, STEP_TIME = 2.0 s)   (retime may lengthen it)
send_motion(…, watch = others_moving)  →  ARRIVED within 0.5° (send.py:32), else the sweep ends, arm held
```
Before each step: the arm is within 0.5° of W_{k−1}, and the gap to W_k is at most 15° on the moving joint and 0.5° on
the others. **Old:** `send_arm_cmd([current, conf], 1 s)`, then a blind `sleep(1 + 6)` (`W:1450-1483`).

Stop test, after ARRIVED: wait SETTLE_MIN = 2.0 s. Then each tick, keep the last STILL_WINDOW = 1.0 s of samples
(about 20 at the 50 ms tick). The window is **still** when all of these hold:
- the arm: no slight motion in the window (`now − last_slight_motion_time ≥ 1.0 s`; slight = 1e-3 rad or 1e-2 rad/s, `arm.py:102-103`), and every joint's spread is ≤ 2e-4 rad;
- base raw pose R: every sample valid and live (`pose_lost == ""`, `mocap_state.py:88`); ≥ 80 % of ticks bring a new capture stamp; spread ≤ 0.3 mm and ≤ 0.05°;
- tool pose T: the same, with spread ≤ 0.3 mm and ≤ 0.2°.

Moving, which ends the sweep (arm held, lease returned). Checked before each prompt, after each Confirm, and every tick while the arm moves or settles:
- this robot's base: wheels faster than 0.05 rad/s, or the follower following (`interlock.base_moving`);
- any other robot: held by a lease, or an arm executing or slightly moved within 2 s, or a base driving or following, or states gone stale (`interlock.busy_reason(world, now, besides=serial)`).

These are the same rules as movement_exec's `others_moving` (`plugins/movement_exec/sequences.py:165-172`).

What is recorded:
- the means over the still window: R̄ and T̄ (Markley mean for orientations: the main eigenvector of Σqqᵀ, with its sign taken from the last sample) and q̄;
- in the session's event log: the spreads, the sample count, the marker errors and the commanded W_k.

If the window is not still within SETTLE_TIMEOUT = 15 s, a warning prompt names the failing test and its value: Confirm
waits another 15 s, Cancel ends the sweep. Losing mocap during a step does not stop the arm (the motion is joint
space); losing it during the settle keeps the settle waiting.

Duration: about 31 × (2 + 2 + 1 + 0.5) s ≈ 3 min per sweep. The old loop took 31 × 8 s ≈ 4 min.

### 0.11 Measured equivalence (scratch re-computation, `tasks/2026-10-26_m10_calibration_scripts/m10_probe_port.py` and `m10_probe_e2e.py`)

The stored sessions were re-computed from their own `<batch>_analysis.json` take lists with the formulas above (numpy
float64, URDF A from yourdfpy). This writes nothing on the Drive.

| Session | circle centres vs stored | K from the stored fits | K re-fit from raw | C re-fit vs stored file |
|---|---|---|---|---|
| 20261016 (0806 left) | 1.2e-7 m | 1.6e-16 m | 0.051 µm, 2.9e-6° | 0.090 µm, 9.9e-6° |
| 20261013 (0805) | 1.1e-7 m | 1.7e-16 m | 0.052 µm, 3.7e-6° | 0.114 µm, 6.3e-6° |
| 20260623 (0804) | 5.5e-8 m | 1.4e-17 m | 0.026 µm, 1.9e-6° | 0.049 µm, 7.1e-6° |

So the test tolerance is 1 µm and 1e-4° (10× margin). Every old JSON file tested (71: takes, outputs,
`base_frame_calibration.json`) round-trips byte for byte through `json.dumps(json.loads(f), indent=4)`.

---

## 1. Context

Calibration is the last feature of the old monitor (Jakob's report §8 item 5: "sequence tasks over joint moves,
recording measured joints and mocap to the Drive").
- **What exists now.** The new stack already applies calibrations: the relay picks each robot's newest Drive session (within 5 cm / 5° of crl-husky's default), and the monitor checks each arm's URDF against its UR calibration.
- **What is missing.** A way to make a new calibration.
- **Why the relay is involved.** The relay publishes only calibrated base poses, and calibration needs raw ones.
- **The outcome.** A session run from the laptop or phone, under the same motion safety as the executor, writing files the old analysis also reads. Its accepted result is written where the relay reads it, with `mocap_id`.

## 2. Architecture

### 2.1 Plugin and contracts

- `plugins/calibration/` registers `calibration`:
  - `requires = ("arm_planner",)`, typed `ArmPlannerContract` (`shared/arm_planner_contract.py:30`), for the transit to a sweep's start;
  - `experimental = True`;
  - dependencies load automatically (`plugin_api/plugin.py:111 resolve_order`), so `plugins:="['calibration']"` is enough.
- **No contract file.** Nothing requires `calibration`; like `assembly_steps`, it offers nothing.
- **Not used:**
  - `movement_exec`: its `ExecRequest` is bound to a process step (`shared/movement_exec_contract.py`);
  - `motion_planner`: its requests are bound to a process movement (`shared/motion_planner_contract.py:20-45`).
- **Instead, calibration takes its own robot lease** (holder `"calibration"`) through the same `shared/execution` helpers movement_exec uses, so every M5 guard applies unchanged:
  - one motion at a time across all robots;
  - other plugins refuse while the robot is held;
  - Hold calls the lease's `stop`;
  - Stop all cancels the task;
  - `send_motion` holds every arm on any non-arrival.

### 2.2 New shared modules: `shared/calibration/` (helpers; never import a contract, `test_layers.py:151`)

| Module | ~Lines | Holds |
|---|---|---|
| `__init__.py` | 15 | the module list |
| `poses.py` | 110 | `apply_base_calibration`, `remove_base_calibration`, `calibration_between` (§0.2); `mean_pose`, `pose_spread`, `pose_offset` (§0.8); `pose_json` / `pose_from_json` (old `[[x,y,z],[qx,qy,qz,qw]]`); `shepperd_quaternion` (reuses `pybullet_planning.utils.transformations.quaternion_from_matrix`), `w_positive` |
| `fit.py` | 220 | `Circle`, `fit_circle` (§0.4); `tool_points` (§0.3); `AxisLine`, `fit_axis_line`, `positive_axis` (§0.5); `ArmBaseFit`, `arm_base_frame` (§0.6); `base_calibration(K, A) -> Pose` (§0.7) |
| `reference.py` | 120 | `REFERENCE_SETS` and `DEFAULT_TOOL_IDS` tables (§5.1); `Reference` (name, batch, joint index, waypoints, dropped tail, other arm's cell-state joints); `reference_files(root, serial, arm)`; `read_reference(path, arm, limits) -> Reference` with every load-time refusal of §0.10 |
| `records.py` | 200 | the old JSON formats, byte-compatible (§5.3): `take_entry`, `write_take`, `read_takes`, `punch_take`, `write_punch`, `write_analysis`, `write_base_frame`, `write_output`; never overwrites (adds `_2`, `_3`) |
| `session.py` | 170 | `SESSIONS_FOLDER`, `SESSION_NAME`, `SESSION_OUTPUT`, `BASE_KEY` (equal to the relay's, by test); `Session` (folder, serial, digits, arm, arm index, robot index, base and tool ids); `open_session` (§5.1, Q4); config.yaml read/write (old template, `robot_name`/`arm` filled); `punch_offset(config, arm_index)`; `log_event` → `shared/runs/runs.py:395 record_event`; `previous_output(root, digits, before)` |
| `settle.py` | 130 | `Sample`, `StillWindow.add(sample)`, `.why_not_still(now) -> str`, `.means()`; the thresholds of §0.10 |
| `verify.py` | 170 | `ArmKinematics` (yourdfpy, no meshes, like `world/kinematics.py:181`; `tool0(joints) -> Pose` from base_footprint); `validation_metrics`, `punch_metrics`, `comparisons`, `checks(report) -> list[Check]` (§0.9) |
| `pipeline.py` | 150 | `run_fit(folder, model_urdf, arm, refs) -> Report` (reads `j0/`, `j1/`, `validation/`, `punch_validation/`; writes `<batch>_analysis.json`, `base_frame_calibration.json`, `calibration_report.json`); `accept(folder, report, session) -> Path` (writes the relay's file, marks the report accepted) |

Also `experiments/calibration_pipeline.py` (~90; top layer): a command-line wrapper around `run_fit` and `accept`, for offline re-analysis (§7).

### 2.3 Reused, with where

| Piece | Where | Use |
|---|---|---|
| robot lease | `shared/execution/interlock.py:39 take`, `:66 give_back`, `:83 busy_reason`, `:131 base_moving` | one lease per sweep; the moving checks of §0.10 |
| prompt | `shared/execution/operator.py:48 PromptBox` | every prompt; Confirm within 0.5 s ignored (`:21`) |
| timing and send | `shared/execution/send.py:148 motion_command`, `:190 send_motion` (holds every arm on cancel, timeout, stop, interrupt; `HOLD_NOT_SENT` said loudly) | transit and every step |
| arm checks | `robot_interface/arm.py:534 safety_problem`, `:442 _check_trajectory` (runs inside every send) | checked before each prompt and after each Confirm, for a clear message |
| transit planner | `shared/planner/planner_service.py:80 request_plan`, `:92 clear`; `plugins/arm_planner/planner_plugin.py:88-118` | plan only; **never** its `commit` (it sends without lease, Confirm or wait) |
| pattern to copy | `plugins/movement_exec/plugin.py:92-134` (request, lease, done-callback safety net, cancel), `sequences.py:165-231` (`others_moving`, `live_command`, `confirmed`), `panel.py:25-80` (desktop and phone) | the sweep task and the panel |
| mocap state | `ctx.track_object` (`plugin_api/context.py:144`), `robot_interface/mocap_state.py:88 pose_lost` | tool body and raw base body |
| kinematics | `ctx.kinematics.link_pose(serial, f"{arm}_tool0")` (`world/kinematics.py:139`) | the record's informational `tool0_fk_pose`, punch takes |
| URDF frames | `robot_interface/frames.py:24 fixed_transform` | A (§0.1) |
| Drive | `drive.py:31 drive_folder`; `ctx.config.drive_root` | session folder (never `drive_root()`, which falls back to the env variable) |
| runs-style records | `shared/runs/runs.py:395 record_event` (append, fsync), `:460 commits` | `<session>/events.jsonl`, session provenance |
| ghosts | `ui/ghost.py:114 robot_ghosts(…, "target")` | the selected sweep's start W₀ |
| phone | `ui/phone.py:52 big_buttons`, `:93 phone_text` | the Calib tab |

Searched and **not** found:
- a plugin-level ROS subscription or service-client API (only `track_object`);
- a calibration tool kind (`config.py:33`: robotiq, scaffolding_v1/v3; `none` is a bare arm);
- any monitor code reading `/mocap_relay/calibrations`;
- `circle_fitting_3d` and `skspatial` in the ros2_ws_next venv.

### 2.4 Edits to Jakob's code in this repo (small; listed for his review)

| # | File:line | Change |
|---|---|---|
| J1 | `mocap_relay/node.py:75-95` BodySample, `:431-453` `_receive_rigid_body_frame`, `:481-511` `_publish_cached_poses`, `:184` | keep the raw pose next to the calibrated one for bodies in `self._calibrations`. Publish it on `f"{topic_prefix}/id_{n}/raw_pose"` with the same stamp, validity and markers (new dict `_raw_publishers`). Module docstring: one line |
| J2 | `robot_interface/mocap.py:18-37` | `RAW_POSE_SUFFIX`; `mocap_topic(mocap_id, raw=False)`; `subscribe_mocap(…, raw=False)`. Docstring: raw is the robot's rigid body as Motive gives it, for calibration only |
| J3 | `monitor.py:233` `track_object`, `plugin_api/context.py:144` | new keyword `raw: bool = False`, passed down; docstring line |
| J4 | `shared/__init__.py` listing | add `calibration/` |
| J5 | `test/mocap_relay/test_mocap_relay_node.py:26-38` `relay()` fake | add `node._raw_publishers = {}` |
| J6 | `test/plugin_fakes.py:78` FakeContext | `track_object(name, mocap_id, raw=False, …) -> TrackedObject` and `untrack_object`, kept in `self.tracked`; a `kinematics` stub with `link_pose(serial, link)` from a settable function |

The relay's `calibration.py` stays untouched: a test pins the session constants equal to `shared/calibration/session.py`.

### 2.5 crl-husky (7ed1e8d): nothing is required

Things that need Jakob's agreement, listed separately (none blocks M10):
- K1. After an accepted session, update the robot's default in `crl_husky/config/robots/<serial>/mocap_config.json` by hand. This is TODO 9 of `tasks/2026-10-16_frames_and_calibration.md`, and is optional: the relay already prefers the newer Drive session. M10 only prints the JSON block to paste.
- K2. A proper message for `/mocap_relay/calibrations`: not needed, since the panel recovers the applied C from the `raw_pose`/`pose` pair (§0.2).
- K3. No new message type: `raw_pose` reuses `crl_husky_msgs/MocapRigidBodyPose`.

## 3. Files to add or change

### 3.1 `plugins/calibration/plugin.py` (~240): `CalibrationPlugin(HuskyPlugin)`, `name = "calibration"`

State:
- the chosen robot and arm, and the open `Session` (or None);
- tracked objects `"calibration/base_raw"` (`track_object(name, robot.config.mocap_id, raw=True, label=…)`) and `"calibration/tool"` (`track_object(name, tool_id, label=…)`);
- the `PromptBox`; the running task and its label; `_cancel_reason`;
- the validation entries and punch takes not yet saved;
- the last `Report`; the C the relay applies (§0.2).

Requests (main thread; the panel defers to them):
- `request_open(serial, arm, session_name, tool_id)`: refuse while a task runs.
  - The tool id must not be any robot's base id nor the probe's 1863 (`plugins/mocap_probe.py:51`).
  - Re-track the bodies, open or create the session (§5.1), write `calibration_session.json`, log `session_opened`.
- `request_sweep(reference_name, step_time, transit_time) -> Future[str]`:
  - Refusals first: no session; busy; `read_reference` refuses; the other arm (Cindy) is more than 5° off the cell state's joints for it, in which case the refusal names the joints and points to robot_control's Stow or jog; `safety_problem()`; robot joints or base not live.
  - Then `interlock.take(world, serial, "calibration", label, stop=self.request_cancel, now)`. `Busy` → refused.
  - Spawn `sweep.run_sweep(job)`, with a done-callback safety net as `movement_exec/plugin.py:113-114`. The `finally` gives the lease back and clears the prompt.
- `request_confirm()`, `request_cancel()`: as `movement_exec/plugin.py:117-134`. Cancel answers an open prompt, else cancels the task with `_cancel_reason = "operator"`.
- `request_validation_point()`, `request_save_validation()`, `request_punch_take()`, `request_save_punch()`, `request_reload_config()` → `takes.py`. Refused while a sweep runs.
- `request_fit()`:
  - refused while a sweep or a fit runs;
  - `await ctx.run_in_thread(pipeline.run_fit, …)`;
  - fills the comparisons: the C in use, crl-husky's default from `crl_husky.config_resolver.get_default_mocap_calibration`, and the previous Drive session.
- `request_accept()`: refused unless the last report passes every "refuse" check and nothing runs. Opens a warning prompt (§4.2 P6) and, on Confirm, calls `pipeline.accept`. Refused on a non-reference arm (A3).

Hooks:
- `setup`: build the panel and ghosts.
- `update`: a same-stamp pair of `robot.base.state` and the raw body → `calibration_between` → the C in use, compared with the known files (≤ 0.01 mm / 0.001° = "same").
- `draw`: panel.
- `teardown`: cancel the task (its `finally` holds and gives back), untrack both bodies.

### 3.2 `plugins/calibration/sweep.py` (~240): the task (pattern: `movement_exec/sequences.py`)

`SweepJob` (frozen):
- ctx, robot, arm, session, reference;
- step and transit times;
- prompts;
- the two tracked bodies;
- an `event(name, **fields)` writer.

`async def run_sweep(job) -> str` returns the outcome line:
1. `world_problem(job)`: the robot is live, `safety_problem()` is None for every arm of the robot, and `others_moving` (§0.10) is clear.
2. Transit, unless every joint is within 0.5° of W₀:
   - `await planner.request_plan(ArmPlanRequest(serial, arm, tuple(W₀)))`;
   - a plan state other than ready → end with its message;
   - `planner.clear()` right after taking `plan.path` (so the arm_planner panel cannot Commit it);
   - `command = live_command(path)`: row 0 is the live joints, refused if over 2° from the path start; `motion_command(..., transit_time)`;
   - prompt P1; after Confirm: `world_problem` again, rebuild the command, then `send_motion(…, watch=others_moving)`;
   - anything but ARRIVED → end.
3. Prompt P2 (warning) for the sweep; after Confirm, `world_problem` again.
4. For k = 0…N−1:
   - for k > 0: step check (§0.10), `motion_command([q_live, W_k], step_time)`, `send_motion(…, watch=others_moving)`, anything but ARRIVED → end;
   - `await settle(job, k)`: `ctx.sleep(SETTLE_MIN)`, then fill a `StillWindow` every `ctx.next_tick()`. Every tick `others_moving` ends the sweep (`_hold_all`, as `send.py:307`). Timeout → prompt P3;
   - build `records.take_entry` from the window means; `event("point", …)`.
5. All N points → `records.write_take(...)` → `event("take", file=…)` → "wrote j0/<file> (N points)".

Every `except CancelledError`:
- event `cancel` with reason "operator", or `cancelled_by(ctx)` (copy `movement_exec/plugin.py:47-49`);
- re-raise.

`send_motion` has already held the arm when cancelled mid-move. When cancelled during a settle, no hold is needed (the arm is stopped), but `_hold_all` is still called, so the controller holds where it is.

A sweep that does not finish writes **no** take file; its points stay in `events.jsonl` (§5.2).

### 3.3 `plugins/calibration/takes.py` (~140): validation points and punch takes (no motion, no lease)

- `async record_validation_point(ctx, …)`: wait up to 5 s for a still window (§0.10, the same thresholds) → entry with the old keys → kept in memory and logged as event `validation_point`.
- `save_validation` → `validation/calibration_<YYYYmmdd_HHMM>_validation.json` (old `M:945-955`, `W:1018-1043`).
- `record_punch_take`: needs B live (`pose_lost`) and the arm still → §0.8 tip pose from `ctx.kinematics.link_pose`. Entry with old keys `W:1083-1101`; `arm_name` is "single" for one-arm robots (`W:143-146`).
- `save_punch` → `punch_validation/punch_validation_<arm_name>_<YYYYmmdd_HHMM>.json` (`W:1110-1144` layout).
- The C in use (§0.2) is logged with every punch take.

### 3.4 `plugins/calibration/panel.py` (~230): desktop folder and phone tab (§4)

### 3.5 Edits J1–J6 (§2.4); `README.md` plugin row; `doc/plugin_roadmap.md`

In the roadmap:
- `calibration` gets a status line;
- `punch_validation` reads "folded into `calibration` (M10)";
- `calibration_checklist.md`: its header points to the plugin.

## 4. UI

### 4.1 Laptop panel (folder "calibration"; never hide a row, keep changing lines one line tall, as `movement_exec/panel.py:3`)

1. **Session**:
   - inputs: Robot dropdown (configured robots), Arm dropdown (its arms), Session text (today `YYYYMMDD`), Tool rigid body number (default from §5.1), button [Open session];
   - chips: Drive folder; config.yaml robot/arm; base raw body (id, live/lost, marker error); tool body; **relay applies**: crl-husky default / session `<date>` / this session's file / unknown, with Δ to this session's result once fitted.
2. **Sweeps**:
   - the reference set name (read-only); a trajectory dropdown, each marked done if this session holds a take of it;
   - Step time (2.0 s) and Transit time (20 s) numbers; buttons [Run sweep] [Cancel];
   - a status line: `<robot> · J0 traj2 · point 12/31 · settling: base 0.08 mm, tool 0.21 mm (wait for ≤ 0.3)`;
   - a prompt line, buttons [Confirm] [Cancel], and the last result line.
   - The 3D view shows W₀ as a yellow target ghost while a trajectory is selected.
3. **Validation**: [Record point] [Save validation (n)] [Discard]; the last points with their spreads.
4. **Punch**: the tip offset read from config.yaml, [Reload config.yaml], [Record punch take] [Save punch takes (n)].
5. **Fit and accept**:
   - [Fit] → a table: takes per batch (file, points, radius, max residual); line fits (normal spread, centre distances, skew); K and C; validation and punch spreads; Δ to the C in use, to crl-husky's default and to the previous session; every check of §0.9 as a green, orange or red chip;
   - [Accept…] (red unless every refuse check passes);
   - after Accept, a line: "Reload the relay while no robot drives: `ros2 service call /mocap_relay/reload_calibration std_srvs/srv/Trigger`", and the JSON block for crl-husky's default (K1).

### 4.2 Prompts (each starts with the robot, e.g. "belle (a200-0805): …"; `operator.py:42-45`)

| # | When | Text (after the robot) | Warning style |
|---|---|---|---|
| P1 | transit | "move <arm> to the start of <traj> along a planned path, T s, largest joint travel d°. Keep the e-stop in hand. Confirm to move." | if d > 60° (`send.py:38`) |
| P2 | sweep | "sweep <joint> of <arm> from a° to b° in N−1 steps of s° (about M min), stopping to record at each; the other arm stays where it is[; waypoints n… past the <joint> limit are left out]. Keep the e-stop in hand. Confirm to start." | always |
| P3 | not still after 15 s | "point k/N is not still after 15 s: <test> <value> (limit <x>). Confirm to wait 15 s more; Cancel ends the sweep, the arm held." | always |
| P4 | session folder missing (Q4) | "create <date>/ with j0, j1, validation, punch_validation under data_experiment/calibration_data on the Drive?" | no |
| P5 | another robot/arm in that folder | refused, no prompt: "<date> holds <serial> <arm>; pick another session name" | – |
| P6 | Accept | "write calibrated_transformation_<digits>_rhino.json into <date>? The relay applies it on its next reload or start. Δ to the calibration in use: x mm / y°.[ Warnings: …]" | if any warning, or Δ > 5 mm / 0.5° |
| P7 | the file already exists | the same as P6, plus "it replaces the file accepted at <time>" | always |

### 4.3 Phone tab "Calib" (`ctx.view.phone`; keep it to what is needed beside the robot)

- the big status line (robot · trajectory · point k/N · settling / recording / done);
- the prompt line;
- big [Confirm] and [Cancel];
- [Record validation point] and [Record punch take];
- the last result line, whole and wrapped.

### 4.4 Stop and Cancel

| Event | What happens | Recorded |
|---|---|---|
| Cancel (laptop or phone) at a prompt | the prompt is answered no; nothing moves; the lease is given back | event `cancel` at "prompt" |
| Cancel during a step | task cancelled → `send_motion` holds every arm of the robot (`send.py:275-285`) → lease given back | `cancel`, reason "operator", point k; no take file |
| Cancel during a settle | task cancelled → hold sent anyway → lease given back | same |
| Hold on the arm (robot_control) | the lease's `stop` = `request_cancel` (`interlock.py:7`) | same |
| Stop all (soft stop) | every task cancelled and queued clicks dropped in the same tick (including a Confirm) → as above | reason "Stop all (the soft stop)" |
| Another robot moves or is held, or this base drives | `send_motion` INTERRUPTED (during a step), or the settle loop ends the sweep → every arm held | `interrupted`, with the reason |
| E-stop, protective stop, program stopped | send STOPPED or TIMEOUT → hold → the sweep ends "failed"; Resume from Health | `failed` |
| A hold not sent (stale joints) | `HOLD_NOT_SENT`, a red line on the laptop and phone: "stop <serial> with the teach pendant or the e-stop" (`send.py:317-322`) | `failed` |
| Plugin stop or monitor shutdown | teardown cancels the task → held → lease given back | reason "the plugin stop or shutdown" |

Validation points, punch takes and Fit never move anything and take no lease. They are refused while a sweep runs.
Accept is refused while anything runs.

## 5. Data

### 5.1 Where

- **Session folder** (old layout, unchanged): `<drive_root>/data_experiment/calibration_data/<YYYYMMDD>/`, with `config.yaml`, `j0/`, `j1/`, `validation/` and `punch_validation/`.
  - The plugin reads the root from `ctx.config.drive_root` (the `drive_root` parameter or `HUSKY_DRIVE_ROOT`).
  - Today's rule in `drive.py:8-9` is never to create folders under the root. Hence Q4 and prompt P4.
  - If `config.yaml` is missing, the plugin writes the old template (`data/calibration_data/_data_template/config.yaml` at 0e45758; copy it into `shared/calibration/` as a string constant, since the sidecar's copy is older) with `robot_name: "<digits>"` and `arm: "left"/"right"`.
  - If `config.yaml` exists, its `robot_name` and `arm` must match the session's robot and arm, else the session is refused (P5).
- **Reference sweeps**, read-only: `<drive_root>/data_design_study/<set>/RobotCellStates/*_JointTrajectory.json` (+ `_RobotCellState.json` for the other arm's joints). Old `M:348-351`, origin 8bfaa25:

  | Robot, arm | Set | Default tool body |
  |---|---|---|
  | a200-0804 `ur_arm` | `260630_calib_trajs_Alice` | 1862 |
  | a200-0805 `ur_arm` | `260630_calib_trajs_Alice` (the same mounting) | 1866 |
  | a200-0806 `left_ur_arm` | `260108_extrinsic_calib_trajs` (12 joints: the right arm parked) | 1866 |
  | a200-0806 `right_ur_arm` | `260225_extrinsic_calib_trajs_Cindy_Right` | 1861 |

- **What the relay reads:** `<session>/calibrated_transformation_<digits>_rhino.json`, written only by Accept (§0.7).
  - The relay takes it on its next start or `~/reload_calibration` (`mocap_relay/node.py:260-266`), if it is the newest session holding that robot's file and within 5 cm / 5° of crl-husky's default (`calibration.py:120-133`, `:165-172`).
  - The relay's 30 s watch logs it once as "newer … not applied" (`node.py:268-285`).
- **Never write into the Drive during tests:**
  - every test uses `tmp_path` roots;
  - plugin tests pass `FakeContext(drive_root=tmp_path)` and set `HUSKY_DRIVE_ROOT` to a separate empty folder whose content is checked unchanged at the end;
  - the equivalence tests only read the Drive and copy what they need into `tmp_path` first.

### 5.2 New files (top level of the session folder; the old scripts and the relay ignore them)

| File | Contents |
|---|---|
| `calibration_session.json` | schema `husky_calibration/1`, kind `session`, created_at, robot_serial, arm, arm_index, robot_index, base mocap_id, tool mocap_id, reference set, URDF (`RobotConfig.source_urdf`, relative) and its sha256, UR calibration files, commits (`runs.commits()`), the C in use at opening (source guess and the sha256 of its transforms, as the relay hashes them) |
| `events.jsonl` | `runs.record_event` lines: session_opened, prompt, confirm, cancel, transit, point (full entry + commanded W_k, spreads, samples, marker errors, settle seconds), take, interrupted, failed, validation_point, punch_take, fit, accept |
| `calibration_report.json` | everything of §4.1 item 5; `accepted`, `accepted_at` |

### 5.3 Old formats (byte-compatible writers; checked against real files, §6)

| File | Layout (old cite) | Writer rule |
|---|---|---|
| `<batch>/calibration_<YYYYmmdd_HHMM>_<traj stem>.json`, `validation/calibration_<YYYYmmdd_HHMM>_validation.json` | `{"raw_data": [entry…]}`; entry keys in order: `robot_id`, `arm_index`, `joint_conf`, `base_mocap_pose`, `flange_mocap_pose`, `tool0_fk_pose`, `tool0_fk_from_mocap` (`W:1008-1016`, `W:1018-1043`) | `json.dumps(obj, indent=4)`, no trailing newline, plain Python floats and ints; `robot_id` = the robot's index in `ctx.config.robots`; `arm_index` = the arm's index in `robot.config.arms`; never overwrite (add `_2` before `.json`; the old minute stamp could overwrite) |
| `punch_validation/punch_validation_<left\|right\|single>_<YYYYmmdd_HHMM>.json` | `{arm_index, arm_name, tool0_link_name, tool0_from_punch_tip, takes}`; take keys `W:1083-1101` | the same |
| `<batch>/<batch>_analysis.json` | `{"takes": [{file_name, raw_data, center, normal}]}` (`0_:121-133`) | the same; `normal` after the alignment of §0.5 |
| `base_frame_calibration.json` | `{base_mocap_from_<arm>_base_link_inertia: 4×4, j0_line {point, direction}, j1_line, base_offset}` (`1_:1077-1092`) | the same; `direction` = N̄ before the motion sign, as the old code stored it |
| `calibrated_transformation_<digits>_rhino.json` | `{base_mocap_from_<arm>_base_link_inertia, base_mocap_from_base_footprint}` (`2_:148-154`) | the same two keys first, then `mocap_id`, `robot_serial`, `arm` |

**What is not byte-identical, and why:**
1. The output gains `mocap_id`, `robot_serial` and `arm`. The plan asks for `mocap_id`; the relay's hash is unchanged.
2. The last digits of the transforms. The old pipeline went through PyBullet's float32 for the tool points (`0_:77`) and for C (`2_:103`); the new one is float64. On the same records the values agree within 0.12 µm and 1e-5° (§0.11). Reproducing the old bytes would need PyBullet's float32 arithmetic in the pipeline (Q7).
3. New records hold means over 1 s and measured joints, not one sample and a mostly commanded waypoint (§0.3). The keys, units, frames and nesting are unchanged, so the old 0_–4_ scripts in the old checkout read new sessions.

## 6. Tests (headless, on fakes; `[S]` = `pytest.mark.safety`)

`test/shared/test_calibration_poses.py`:
- `test_removing_the_calibration_inverts_the_relay` [S]: the **raw-pose inversion round trip**. For 20 random R and each robot's real crl-husky default C:
  - `remove_base_calibration(C, relay.apply_base_calibration(C, R)) == R`, to 1e-12 m / 1e-12 rad;
  - `apply(C, remove(C, B)) == B`;
  - the relay function comes from `mocap_relay/node.py:119`.
- `test_the_applied_calibration_is_found_from_a_raw_and_published_pair` [S]: `calibration_between(R, R·C) == C` to 1e-12.
- `test_mean_pose_and_spread`: noise of 0.1 mm / 0.05° around a pose gives the mean within 0.05 mm, spreads as built, and quaternion signs mixed on purpose.
- `test_w_positive_and_shepperd_keep_the_old_signs`: the stored K and C of the three sessions (copied into a fixture) come out with their stored component signs.

`test/shared/test_calibration_fit.py`:
- `test_a_known_circle_is_fitted`: noise-free points → centre, normal and radius to 1e-12; with 0.1 mm noise, the centre within 0.1 mm.
- `test_flipped_normals_give_the_same_axis`: one take's normal negated → the same N̄ and line.
- `test_the_positive_axis_follows_the_joint_motion`: reversed waypoint order → â flips; reversed take files → it does not.
- `test_synthetic_session_recovers_the_true_calibration` [S]: the pipeline's own round trip.
  - Setup, for each robot's URDF (Alice, Belle, Cindy left and right): a true C, a tool offset E (23 mm, as Belle's), and the real reference waypoints (fixture copies of 2 J0 + 2 J1 files per set, or synthetic sweeps);
  - generate R (base drifting 1 mm) and T = R·C·FK(q)·E;
  - result: K and C equal the truth to 1e-9 noise-free, and to 0.3 mm / 0.05° with 0.1 mm / 0.02° noise.
- `test_one_frame_rule_for_every_arm` [S]: the frame from §0.6 gives z = +J0 and y = −(+J1) for single-arm, left and right fixtures; equal to the old code's branches.
- `test_a_j1_sweep_off_j0_zero_is_refused` [S].

`test/shared/test_calibration_reference.py` [S]:
- `test_a_file_for_another_arm_is_refused`: Alice's set opened for `left_ur_arm`.
- `test_two_moving_joints_or_a_step_over_15_degrees_is_refused`.
- `test_the_moving_joint_must_match_the_file_name`: J0 vs J1.
- `test_waypoints_past_the_urdf_limits_are_dropped_from_the_end_only`:
  - Alice `J0_traj5` keeps 30 of 31, dropped = 1;
  - Cindy right `J0_traj0` keeps 23, dropped = 8;
  - a limit broken in the middle → refused;
  - fewer than 20 left → refused.
- `test_the_other_arm_joints_come_from_the_cell_state`: Cindy's 12-joint cell state.

`test/shared/test_calibration_records.py`:
- `test_take_files_are_written_byte_for_byte_like_the_old_monitor`: committed fixture `test/shared/calibration_fixtures/take_3_points.json` (the first 3 entries of a 20261013 take, written by `json.dumps(indent=4)`) → read → write → equal bytes. The same for a validation file and a punch file.
- `test_the_output_keeps_the_old_keys_first_then_adds_the_ids`:
  - key order; `mocap_id` an int;
  - the relay's `read_session_file` reads it;
  - its sha256 equals that of the same transforms without the ids;
  - a wrong `mocap_id` is refused by the relay.
- `test_files_are_never_overwritten`: a second take in the same minute → `_2`.
- `test_session_constants_equal_the_relays` [S]: `SESSIONS_FOLDER`, `SESSION_NAME`, `SESSION_OUTPUT`, `BASE_KEY`.
- `test_config_yaml_is_written_for_the_robot_and_refused_for_another` [S].

`test/shared/test_calibration_settle.py` [S]:
- `test_still_needs_quiet_joints_base_and_tool`: each threshold at 0.9× (still) and 1.1× (not), naming the test.
- `test_a_repeated_capture_stamp_is_not_a_new_sample`: the relay republishing its last frame.
- `test_a_lost_body_says_which`.
- `test_means_are_over_the_window_only`.

`test/shared/test_calibration_verify.py`:
- `test_validation_of_a_perfect_calibration_has_zero_spread`: synthetic, with FK from the real URDF.
- `test_a_1_mm_error_in_C_shows_in_the_spread`.
- `test_punch_spread_and_pairwise`.
- `test_checks_follow_the_threshold_table` [S]: every row of §0.9, each side.

`test/experiments/test_calibration_equivalence.py` (slow by folder, `test/conftest.py:14-17`; skipped if `HUSKY_DRIVE_ROOT` has no `data_experiment/calibration_data/20261013`):
- `test_old_session_is_reproduced[20260623|20261013|20261016]`: the **JSON identical to the old one** test.
  - Copy `config.yaml` and, for each batch, write every take of the stored `<batch>_analysis.json` (its take list, not the folder's current files) into `tmp_path/<date>/<batch>/<file_name>` with the new writer; copy `validation/`.
  - Run `run_fit` + `accept` on the copy.
  - Then: centres within 1e-6 m of the stored ones; K and C within 1 µm and 1e-4° of the stored `base_frame_calibration.json` and output file; quaternion components of K and C within 1e-6 (same signs).
  - The output's first two keys are equal in name and order to the stored file's, with `mocap_id`/`robot_serial`/`arm` after; `json.loads` of both, minus the extra keys, are equal within those tolerances.
- `test_every_old_take_file_is_rewritten_byte_for_byte`: every `calibration_*.json` under the three sessions → read → write into `tmp_path` → equal bytes.
- `test_the_relay_takes_the_new_output`: tmp Drive root with the new output under a newer date → `mocap_relay.calibration.resolve` picks it; no warnings vs crl-husky's default.
- `test_the_command_line_fits_a_copied_session`: `experiments/calibration_pipeline.py` on a tmp copy → exit 0, files written there only.

`test/mocap_relay/test_mocap_relay_node.py` (file already `safety`):
- `test_a_robot_body_is_also_published_raw`: calibrated body → `pose` = base_footprint and `raw_pose` = the Z-up raw pose, with the same stamp, validity and markers; a body without a calibration gets no `raw_pose`.
- `test_raw_and_published_poses_of_one_frame_give_back_the_applied_calibration`: through the node.

`test/robot_interface/test_mocap_raw_topic.py` [S]:
- `test_raw_topic_name_and_subscription`: `mocap_topic(1850, raw=True)` is `/mocap/rigid_body/id_1850/raw_pose`; `track_object(..., raw=True)` subscribes there.

`test/plugins/calibration/test_calibration_sweep.py` [S] (on `test/robot_lab.RobotLab` plus J6's fakes; a fake `arm_planner` whose `request_plan` resolves a ready `Plan` with an `ArmPath`):
- `test_a_sweep_is_confirmed_steps_settles_records_and_writes_one_take`: 31 points; the take file with old keys; the lease given back; events prompt, confirm, transit, point ×31, take.
- `test_the_transit_is_planned_by_arm_planner_and_sent_from_the_live_joints`: row 0 = live; `commit` never called; `clear` called.
- `test_cancel_at_either_prompt_sends_nothing`.
- `test_cancel_during_a_step_holds_every_arm_and_writes_no_take`.
- `test_cancel_during_a_settle_holds_and_gives_the_robot_back`.
- `test_stop_all_mid_sweep_holds_and_is_logged_as_the_soft_stop`.
- `test_hold_from_robot_control_cancels_the_sweep`.
- `test_a_sweep_is_refused_while_another_robot_is_held_or_moving`.
- `test_another_robot_starting_to_move_mid_sweep_holds_the_arm`: during a step and during a settle.
- `test_the_own_base_driving_ends_the_sweep`.
- `test_a_step_is_refused_when_the_arm_is_not_at_the_previous_waypoint`.
- `test_the_safety_sync_not_vouching_refuses_before_and_after_confirm`.
- `test_a_stale_transit_plan_is_refused_after_confirm`: arm moved > 2°.
- `test_not_still_prompts_and_cancel_ends_the_sweep`.
- `test_the_other_arm_must_be_at_its_cell_state` (Cindy).
- `test_the_tool_id_cannot_be_a_robot_base_or_the_probe`.
- `test_a_second_sweep_request_is_busy`.

`test/plugins/calibration/test_calibration_takes.py`:
- `test_a_validation_point_waits_for_still_and_saves_the_old_layout`.
- `test_a_punch_take_uses_the_live_base_and_the_config_offset`: single → "single" / left / right.
- `test_takes_are_refused_while_a_sweep_runs` [S].
- `test_nothing_is_written_outside_the_session_folder` [S]: the `HUSKY_DRIVE_ROOT` sentinel folder is unchanged.

`test/plugins/calibration/test_calibration_accept.py` [S]:
- `test_accept_needs_a_passing_report_and_a_confirm`.
- `test_accept_is_refused_for_the_non_reference_arm`.
- `test_accept_is_refused_while_anything_runs`.
- `test_the_panel_shows_which_calibration_the_relay_applies`: default → after Accept and a simulated reload, the raw and published pair gives "this session's file".

## 7. Step order (tasks) and verification

Every command runs through the sidecar environment: domain 70, localhost only, the Drive read-only by rule, cwd = the
worktree. Set `J=/home/yijiangh/Code/ros2_ws_next/src/husky-assembly-teleop/tasks/2026-10-26_m10_calibration_scripts/jg_env.sh`. Never start the relay or a monitor on domain 80.
Commit each task on the branch only after `$J python3 -m pytest test -q -p no:cacheprovider -m ""` passes (AGENTS.md).

| # | Task | Files | Depends on |
|---|---|---|---|
| T0 | Copy this spec to `tasks/<date>_m10_calibration.md` (AGENTS.md "Task notes") | tasks/ | – |
| T1 | Pose math and fits | `shared/calibration/{__init__,poses,fit}.py`, tests poses and fit | – |
| T2 | References, records, session | `shared/calibration/{reference,records,session}.py`, tests reference and records, fixtures | – |
| T3 | Relay raw pose and raw tracking (J1–J5) | `mocap_relay/node.py`, `robot_interface/mocap.py`, `monitor.py`, `plugin_api/context.py`, `shared/__init__.py`, relay and mocap tests | – |
| T4 | Stop test | `shared/calibration/settle.py`, its test | T1 |
| T5 | Verify, pipeline, command line, equivalence | `shared/calibration/{verify,pipeline}.py`, `experiments/calibration_pipeline.py`, tests verify and equivalence | T1, T2 |
| T6 | The plugin | `plugins/calibration/*`, `test/plugin_fakes.py` (J6), plugin tests | T1–T5 |
| T7 | Docs | README row, `doc/plugin_roadmap.md`, `calibration_checklist.md` header, the task note's status | T6 |

T1–T3 can run in parallel; T4 and T5 after T1 (T5 also after T2); T6 last.

```bash
J=/home/yijiangh/Code/ros2_ws_next/src/husky-assembly-teleop/tasks/2026-10-26_m10_calibration_scripts/jg_env.sh
$J python3 -m pytest test/shared/test_calibration_poses.py test/shared/test_calibration_fit.py -q -p no:cacheprovider   # T1
$J python3 -m pytest test/shared/test_calibration_reference.py test/shared/test_calibration_records.py -q -p no:cacheprovider  # T2
$J python3 -m pytest test/mocap_relay test/robot_interface/test_mocap_raw_topic.py -q -p no:cacheprovider          # T3
$J python3 -m pytest test/shared/test_calibration_settle.py test/shared/test_calibration_verify.py -q -p no:cacheprovider  # T4, T5
$J python3 -m pytest test/experiments/test_calibration_equivalence.py -q -p no:cacheprovider -m slow             # T5: JSON identical
$J python3 -m pytest test/plugins/calibration -q -p no:cacheprovider                                             # T6
$J python3 -m pytest test/structure/test_layers.py -q -p no:cacheprovider -m ""                                  # layers
$J python3 -m pytest test -q -p no:cacheprovider -m safety                                                       # every guard
$J python3 -m pytest test -q -p no:cacheprovider -m ""                                                           # everything, before each commit
# Offline fit of a copied session (never point it at the Drive in a test run):
$J python3 -m husky_assembly_teleop.experiments.calibration_pipeline /tmp/<copy>/20261013 --robot 0805 --arm ur_arm
# Start-up smoke: every plugin sets up, no "failed", clean teardown (viser on 8095; no robot answers on domain 70):
$J timeout 25 python3 -m husky_assembly_teleop.monitor --ros-args -p robots:="['belle','cindy']" \
  -p tools:="['0805:none','0806:none,scaffolding_v3']" -p plugins:="['calibration','robot_control']" -p viser_port:=8095
```

The smoke is run with `python3 -m …monitor` from the sidecar, not `ros2 run`, which would run `yh/core-port` through
the shared install (merge spec §7).

## 8. Lab checklist for the M10 acceptance (monitor and visual steps; the manual rewrite comes in M11)

Robot: Belle (Q13). The last session is `20261013`.

Before:
1. Motive:
   - camera calibration "exceptional" if cameras moved or it is over a week old;
   - rigid bodies 1850 (Belle's base, the same asset as for 20261013) and 1866 (the calibration tool) are streaming;
   - labeled markers are on.
2. Belle:
   - the calibration tool on the flange, the Robotiq removed, the driver started with `gripper:=none`;
   - the base jacked up, wheels off the floor, in the middle of the volume, ≥ 1 m from anything;
   - the teach pendant's speed slider at 100 %; the e-stop in hand.
3. Lab PC: `ros2 launch husky_assembly_teleop mocap.launch.py`. Its table lists `1850 a200-0805 Drive session 20261013`.
4. Monitor: `-p robots:="['belle']" -p tools:="['0805:none']" -p plugins:="['calibration','robot_control']"`. Health is green for Belle, the mocap chip is green, and there is no URDF chip.

Session:
5. Calibration panel: Belle, `ur_arm`, session today, tool 1866 → [Open session]. Check:
   - every chip is green;
   - the relay chip says "session 20261013";
   - on the Drive, today's folder has `config.yaml` with `robot_name: "0805"`.
6. For J0 traj0–traj5:
   - select the trajectory, and see the yellow ghost at its start;
   - [Run sweep] → read P1 → Confirm; watch the arm reach the start without touching anything;
   - P2 → Confirm (traj5 says waypoint 31 is left out);
   - watch the status count to 31/31, with the arm stopping at each point;
   - the result line names the new `j0/` file.
7. The same for J1 traj0–traj5 (`j1/`).
8. Safety, once each, during a sweep:
   - Cancel on the phone: the arm stops within a fraction of a second, the status reads "cancelled at point k", and no file appears;
   - Hold on robot_control's arm row: the same;
   - Stop all: the same.

   Then Resume in Health, and re-run that trajectory.
9. Validation:
   - lower the base onto its wheels, then drive it (D-pad) to 15–20 different places and headings around the room;
   - at each, jog the arm (robot_control) to a different pose, wait for "still", then [Record point] (phone allowed);
   - [Save validation (n)].
10. [Fit]. Check:
    - every chip is green (or orange with a reason you accept);
    - **lab acceptance**: Δ to the previous session (20261013) ≤ 1 mm and ≤ 0.1°;
    - send back a screenshot of the table.
11. [Accept…] → read P6 → Confirm. Within about 30 s (more if the relay runs on another PC: wait for Insync), the relay terminal shows the box "newer calibration on the Drive, not applied".
12. With no robot driving, run the reload command shown in the panel. Check:
    - the relay's table now lists `Drive session <today>`;
    - the panel's relay chip reads "this session's file";
    - Belle's base in the 3D view shifts by about the Δ of step 10.
13. Punch, after step 12:
    - mount the punch tool and set its 4-point TCP on the pendant;
    - enter the offset in metres in today's `config.yaml` (`punch_tool.left.offset_xyz`; check the digits twice), then [Reload config.yaml];
    - from 4–6 base poses, align the tip by hand with robot_control's Free drive (hold), then [Record punch take];
    - [Save punch takes], then [Fit] again: the tip spread ≤ 2 mm.
14. Send back: the session folder name, the Fit table, the relay table after reload, and the monitor log.

## 9. Risks (by severity)

1. **An accepted calibration changes every later drive of that robot.** The relay applies the newest session within 5 cm / 5° of the default on its next start or reload. Mitigations:
   - nothing but Accept writes that file, behind the §0.9 gate and a warning prompt;
   - reload only while no robot drives (the follower would jump; `node.py:11-12`).
2. **Sweeps are not collision-checked.** Only the transit is planned.
   - The reference sets were designed and run before.
   - The other arm must sit at its cell-state joints (5°).
   - Waypoints never leave the URDF limits.
   - The model has no calibration tool (`none`), so the transit planner does not see it (Q9, Q10). The operator keeps the e-stop in hand.
3. **Drive writes.** The plugin writes on the real Drive in the lab (as the old monitor did). Tests must never write there:
   - the plugin reads only `ctx.config.drive_root`;
   - `drive_root()` falls back to `HUSKY_DRIVE_ROOT`, which jg_env.sh sets to the real Drive, so tests set a sentinel folder (§5.1).
4. **Insync lag.** A session written on the laptop reaches a relay on the lab PC later. The relay chip (§0.2) shows what is really applied; never trust the file alone.
5. **`raw_pose` and Jakob's `origin/jg/mocap-marker-cloud` (faeaaa1, not yet on this branch).** Both edit `node.py` (`__init__`, `_receive_*`) and `robot_interface/mocap.py`. Small textual conflicts are expected whichever lands second; keep both publishers.
6. **Stale old data in the session folder.** `read_takes` takes every top-level `calibration_*.json` in the batch folder, as the old 0_ did (§11 finding 2). The Fit table lists each take with its file so a stray one is seen; move it to `archive/` to drop it.

## 10. Open questions (each with my recommended answer)

- **Q1. Raw base pose.** The relay publishes `raw_pose` per robot body, versus the monitor inverting B with the relay's calibration list. **Rec: `raw_pose`.** The plan says so, there are no race conditions on reload, and the pair still identifies the applied C.
- **Q2. Settle.** SETTLE_MIN 2 s plus the measured stop test, versus the old blind 6 s. **Rec: 2 s + stop test.** It waits as long as the base needs, and its numbers are logged.
- **Q3. Recorded value.** The mean over the 1 s still window, versus the last sample. **Rec: the mean.** It lowers the noise; the format is unchanged.
- **Q4. Session folders.** The plugin creates `<date>/` and its four batch folders after P4, as the old monitor did, versus the operator creating them in Drive. **Rec: create them after P4.** Only under an existing, synced `calibration_data/` and only these five names. This is an exception to Jakob's rule in `drive.py`, so it needs his OK.
- **Q5. Cindy's right arm.** Its session is fitted and compared (a mount check), never written for the relay. **Rec: yes**, until an arm-mount overlay exists (Q12).
- **Q6. Two robots on one day.** **Rec: one robot and arm per date folder** (P5, today's rule; the relay accepts only 8-digit names). Widen `SESSION_NAME` only if it is ever needed.
- **Q7. Byte identity of the output.** **Rec: value identity** (1 µm / 1e-4°) with identical keys, order and format, plus the appended ids. Byte identity would need PyBullet's float32 arithmetic in the pipeline, which only reproduces old rounding.
- **Q8. Plots.** **Rec: numbers and the Fit table in M10.** The old 0_–4_ scripts and Rhino viewers keep working from the old checkout on new sessions until M11; port plots later if missed.
- **Q9. Sweep collision check.** **Rec: none in M10** (the risk-2 measures). An optional check of every waypoint through arm_planner's mirror can follow.
- **Q10. Calibration tool model.** **Rec: `tools:="['<serial>:none']"`** (bare arm) for now. A `calib_tool` kind with a 0.1 m cylinder URDF is a later, small addition.
- **Q11. A relay-reload button.** **Rec: no.** Plugins have no service-client API; the panel shows the command, and the relay prints it too.
- **Q12. Arm-calibration overlay** (Jakob §8 item 5: "the arm-calibration overlay format decided first"). **Rec: keep the old meaning:** C absorbs the reference arm's mount (C = K·A⁻¹). A per-arm mount-joint overlay is its own later milestone.
- **Q13. Lab acceptance robot.** **Rec: Belle** (one arm, the freshest one-arm session 20261013, tool 1866).
- **Q14. Step and transit times.** **Rec: 2.0 s per step** (old 1 s; gentler, still about 3 min per sweep) and **20 s transit** (old manual 20–40 s).
- **Q15. Thresholds** (§0.9, §0.10). **Rec: as tabled**, from the last two good sessions; revisit after the first lab session.
- **Q16. Waypoints past a joint limit.** **Rec: drop them from the end only, if ≥ 20 remain, and say so in P2.** It never commands past a limit, and Alice/Belle `J0_traj5` and Cindy's right J0 sweeps stay usable.
- **Q17. Cindy's URDF limits** (shoulder_pan ±270°, wrists ±240°) versus the UR's ±360°. **Rec: obey the URDF** (they are deliberate, for her cables). Her right-arm J0 sweeps then cover 264°, enough for a circle fit; regenerate that set later if wanted.

## 11. Effort

| Part | Implementer days |
|---|---|
| T1–T2 shared math and files (about 800 lines, about 450 lines of tests) | 1.5 |
| T3 relay raw pose and raw tracking | 0.3 |
| T4–T5 stop test, verification, pipeline, command line, equivalence | 1.0 |
| T6 plugin, panel, phone, plugin tests | 2.0 |
| T7 docs; review and fixes | 1.0 |
| **Total** | **about 5.5 days**, plus a half-day lab session (12 sweeps × about 3 min, validation, Accept, reload, punch) |

## Deviations from the request or the plan

1. **No crl-husky change.** `raw_pose` reuses `MocapRigidBodyPose`, and the relay lives in this repo (moved 16 Oct), so the edits are to Jakob's code here (J1–J6), not in crl-husky.
2. **The plan's edit 5 ("arm_planner's core … plus a `request_target` service") is already covered** on the merged layout by Jakob's headless `arm_planner` (`ArmPlannerContract.request_plan(ArmPlanRequest)`). Calibration uses it and never its `commit`. On 9165944, edit 5 must still be built (layout note).
3. **Calibration takes its own lease** through `shared/execution`, instead of going through `movement_exec`, whose request is bound to a process step.
4. **"JSON identical" means** byte-identical record files, and an output with identical keys, order and format whose values match within 1 µm / 1e-4°. It is not byte-identical: float32 versus float64, and the added ids (§5.3, Q7).
5. **The roadmap's separate `punch_validation` plugin is folded into `calibration`** (the approved plan's row).

## Out-of-scope findings (reported, not designed around)

1. **The sidecar's analysis scripts are stale.** `data/calibration_data/*.py` on `yh/core-port-jg-sync` is an older copy than 0e45758:
   - it lacks the 2026-10-14 motion-sign fix (`joint_axis_from_motion`), so it gives Belle's frame turned 180°;
   - it reads data from the repo;
   - 77 MB of old session data (20250310–20260625) is committed there.

   M11's switch-over must take 0e45758's scripts or drop the folder.
2. **Two Drive session folders hold files that their stored analysis did not use:**
   - `20261013/j0/` holds two J1 takes (`..._J1_traj0`, `..._J1_traj1`);
   - `20261016/j0/` holds an Alice-set take (`..._ext_calib_0804_J0_traj0_...`, 12:14).

   Re-running the old 0_ on those folders today would mix them in. The equivalence test uses the stored take lists. Moving those files to `archive/` is the user's call.
3. **The old records' `joint_conf` is mostly the commanded waypoint, and occasionally measured** (`W:1480` raced the joint callback). Belle's 20261013 `J0_traj5` point 31 is 360.98° measured against 367.45° commanded: the UR stopped at its limit.
4. **Reference sets that leave the URDF joint limits:** all six of Cindy's right-arm J0 sweeps leave the ±270° shoulder_pan limit for their last 8 waypoints, and Alice's `J0_traj5` reaches 367° (Q16, Q17).
