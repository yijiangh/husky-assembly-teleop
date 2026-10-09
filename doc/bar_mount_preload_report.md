# Bar mount preload: why the servo says "on target" while the bar is not

Written 2026-10-09. Status: **analysis + proposal + experiment plan**. Nothing in here is
implemented yet; the experiments in section 7 decide what gets built. Living note: add the
results and the decisions to section 9 as they come in.

Related:
- `doc/bar_holding_acc_manual.md`: the bar-holding accuracy test, the servo loop, marker takes.
- `doc/support_robot_schedule_manual.md`, section "Visual-servo correction after the transfer":
  the servo that now runs at the end of every transfer.
- Session report 20261001: `<drive>/data_experiment/bar_holding_acc_data/20261001-result/RESULTS.md`
  (`<drive>` = `~/Insync/2025-03 Husky Assembly`).

---

## 1. Summary

- When a bar is mounted in both tools, the joint-to-tool seat is about **5 mm off**. The grasp
  screw pulls it in anyway, so the bar is **forced** into the two tools. Both force sensors then
  read roughly **40 N** of push/pull, and the arms bend a little. The joint encoders do not move.
- Because the bend is not in the encoders, **the robot cannot see it**. The tool0 servo, the
  grasp-mismatch split and the bar-held planners all keep the stressed flange-to-flange relation
  exactly as it is.
- The stored bending is released when the arms go compliant or let go:
  - On a **male-female** joint the joint takes up the release; the bar still seats.
  - A **foundation (ground) bar** has no joint. The operator tapes it to the floor and its
    accuracy depends on the robot alone, so the release shows up as placement error.
- A second, independent error sits on top: a **fixed calibration offset**. In the accuracy test of
  2026-10-01 the bar landed 4.3 mm off (median) while the servo reported 0.7 mm, and all 20 error
  arrows pointed the same way in the robot's frame.
- Proposal, in order:
  1. Unload the bar right after mounting, with a force-guided, position-controlled move of one arm.
  2. For foundation bars, close the servo loop on the **bar** (mocap markers), not on tool0,
     before taping.
  3. Calibrate the fixed offset out once the data shows where it lives.
- **First run Experiment 1** (section 7). It needs no new code and it sizes all three.

---

## 2. What was observed on the robot

| observation | detail |
|---|---|
| gap at the joint-tool seat | about 5 mm when the bar is offered to the second tool |
| how it is taken up | the grasp screw pulls the joint into the jaws |
| force afterwards | about 40 N pushing or pulling, on **both** end effectors |
| robot configuration | the arms bend slightly, but the joint encoders **do not change** |
| tool0 error (mocap base + FK vs Rhino) | small, so the servo loop is satisfied |
| when compliance turns on | the stored bending is released and the tools move |
| male-female joint | the release is absorbed by the joint; the bar still seats |
| foundation bar | taped by hand, no joint to absorb anything: placement = robot accuracy |

Likely sources of the 5 mm, not yet separated: how each joint is installed on the bar, how each
tool is mounted on its flange, and how each tool is assembled.

---

## 3. What the servo loop actually measures

The servo error is computed from the robot's own model (`measure_servo_tool0_error`):

```
mocap marker body ─► base calibration ─► base_footprint ─► arm mount ─► joint encoders + FK ─► tool0
└──────────────────────────── seen by the servo loop ─────────────────────────────────────┘

tool0 ─► tool mounting ─► tool assembly ─► jaws + grasp screw ─► joint on the bar ─► bar
└────────────────────────── invisible to the servo loop ───────────────────────────┘

plus, along the whole arm: elastic bending in gears, links and tools (not in the encoders)
```

- **Observed tool0**: FK of the live joint encoders on the calibrated URDF, placed at the live
  mocap base pose.
- **Target**: the Rhino tool0 frames (the transfer's `target_ee_frames`).
- **Error**: position difference (mm) and per-axis angle (deg). Convergence: both arms under 0.2 mm
  of position error, or less than 0.5 mm left to plan, or 8 iterations.

The loop can always drive this number down, because it compares two quantities it computes
itself. The accuracy-test manual calls it *precise, but not accurate*.

**One more limit, found 2026-10-09.** The live-base IK inside the loop is PyBullet's built-in IK
(through compas_fab), which stops within 1 mm / 0.01 rad. That is why the headless test leaves
about 0.7 mm and why the 0.2 mm test rarely ends the loop. ssik (analytic, exact on the same
calibrated URDF) would remove that floor. This is a separate, smaller issue; see section 8.

### Why the planners keep the preload

- The **grasp-mismatch split** (`split_grasp_mismatch`) and the **bar-held straight-line planner**
  (`plan_constrained_dual_arm_linear`) both read the left-to-right flange relation from the live
  joints.
- Under preload the joints hold the **commanded** relation, not the bar's natural one, so both
  faithfully keep the stressed relation.
- They never add preload and never remove it. The servo corrections and the insert carry the
  bar's stress along unchanged.

---

## 4. Evidence from the accuracy test (session 20261001, 20 bars)

From `20261001-result/RESULTS.md`:

| | value |
|---|---|
| bar placement error (cameras, bar markers) | mean 4.32 mm; best 2.61, worst 6.61, median 4.28 |
| servo residual (the robot's own number) | median 0.73 mm (0.23 to 1.24) |
| repeatability of one bar's reading | 0.035 mm, so the cameras are not the problem |
| error arrows that agree in the room | 8 of 20 |
| error arrows that agree **in the robot's frame** | **20 of 20**, mean [+2.54, +1.21, −2.17] mm |
| one constant correction in the robot frame | 4.10 → 1.99 mm (removes 72%) |
| one constant correction at the flange | 4.10 → 1.81 mm (removes 77%) |
| share of the error that is a tilt | 71%, a 0.125° angle somewhere in the chain |
| share the two grippers' disagreement can explain | 0.036° of it (29%; 13% before the grasp moved) |
| what the robot could see | 0.73 mm, 17%: even a perfect servo removes only this |

What it says:
- The error **turns with the robot**. It is fixed somewhere between the marker body on the husky
  and the bar in the jaws. It is a calibration offset, not noise.
- The session could not say where exactly: **no joint values were saved in the takes**, and the
  grasp orientation is tied to where the robot parks, so calibration, arm and tool offset move
  together.
- In that test the bar was mounted once for the whole session, so its preload was present in every
  take and is folded into those numbers.

---

## 5. Source of the problem: two separate errors

| | **A. Mount preload** | **B. Fixed offset** |
|---|---|---|
| what | a geometric mismatch between the two grasps, forced in by the grasp screws | a constant error in the chain (base calibration, arm, tool offset) |
| size seen | ~5 mm at the seat, ~40 N per tool | ~4 mm at the bar (2026-10-01) |
| seen by the servo | no (bending is not in the encoders) | no (it sits in the assumed transforms) |
| changes when | each mount; maybe per tool, maybe per bar | slowly, or with re-calibration |
| released by | compliance mode, or letting go | never: it is where the robot really is |
| hurts | foundation bars (nothing absorbs the release) | every bar; foundation bars most |

### Where the preload goes in the current flow

- **Male-female bar**: the insert runs rigid, then compliant for the last mm. When compliance
  engages, the preload drives the tools toward the bar's natural relation, and the joint absorbs
  it.
- **Foundation bar**: the insert runs **rigid only** and never tightens (Rhino note D9), so the
  preload is still in the bar at the assembled pose while the operator tapes it. It is released
  later, during the ungrasp and the retreat's compliant part, **after** the bar is taped. The bar
  then springs by however much the tape lets it.
- **Hypothesis to check (Exp 2)**: the retreat zeroes both force sensors right before its
  compliant part. That zero is taken **with the preload in place**. When the grippers then open,
  the preload disappears from the reading, and the compliance may react to that change.

---

## 6. Proposed fixes, in order

### Fix 1: unload the bar right after mounting (a force loop)

**When:** in the post-transfer task, right after the operator confirms the mount and before the
servo runs.

**How:**
1. Tare both force sensors at the mount pause, while the tools are still empty and the arms stand
   still. That is the one moment the sensors carry nothing.
2. After mount + clamp, read both wrenches and put them in **one frame**:
   - force each arm carries in common = the bar's weight: `f_L + f_R` (should equal the bar +
     joints weight after an empty tare);
   - force the arms push **against each other** = the preload: `(f_L − f_R) / 2`;
   - for torques, first move both to one common point (`τ + r × f`), then split the same way.
3. Move **only the right arm** (the bar body rides on the left), in small position-controlled
   straight-line steps, until the preload is under a few N:
   `Δ = −K⁻¹ · preload`, where K is the stiffness of the two-arm hold (N per mm of relative flange
   motion). Measure K once (Exp 1 gives a first value).
4. Stay in joint control the whole time.

**Why not compliance mode:** the compliance controller runs at 500 N/m translational
(`crl_husky/config/ur_controllers.yaml`). With the sensors tared empty, the bar's weight alone
would let it sag by centimetres. That is the same reason the insert is not fully compliant.

**Why one unload is enough:** afterwards the encoders hold the bar's **natural** relation. Every
bar-held move after it (the servo corrections, the insert) keeps that relation exactly, so nothing
reloads, nothing springs back at compliance or release, and the grasp split starts seeing the real
grasp.

**Thresholds to set from Exp 1:** stop below about 5 N (above the sensor noise at rest); at most
1 mm per step.

### Fix 2: for foundation bars, close the loop on the bar, not tool0

1. At the assembled pose, before taping, measure the bar with markers.
2. Apply its pose error as one rigid correction to both flanges, as a bar-held straight-line move.
3. Re-measure; repeat until under about 1 mm. Then tape and release.

This is the only loop that can see error B and the tool/joint geometry. It needs:
- a marker set on the foundation bar that can be fitted live (see the open questions);
- the user's OK to aim at **live flange + measured correction**. That is a second exception to the
  rule "EE targets never from FK", like the retreat's (`memory: ee-targets-never-from-fk`).

### Fix 3: calibrate the fixed parts out (the root cause)

If Exp 1 shows the mismatch is the same at every mount and for every bar, it belongs to the
**tools** (or to the base/arm calibration), not to the bars. Measure it once, correct the tool
offsets or the base calibration, and the transfer then arrives with the right flange relation. The
bar mounts without preload, and Fixes 1 and 2 shrink to checks.

The 20261001 result already says a constant correction in the robot frame or at the flange removes
72 to 77% of the placement error. With joint values in the takes, the next session can say which
one.

---

## 7. Experiment manual

### Before you start

- Robot: Cindy (`/a200_0806`, ROS domain 86), base **fixed** (do not drive during an experiment).
- Bar: the accuracy-test bar with its `bar_rig` marker set (so `Record + Fit + Viz (shared)` can fit it).
- Monitor up with mocap, `Toggle FT Watch (live)` on, `Toggle Servoing Tracker` optional.
- A second terminal for the rosbag. Joint states and force readings keep streaming even with a
  pendant in **local** mode, so the bag covers the freedrive step:

```bash
ros2 bag record \
  /a200_0806/left_ur5e/rate_limiter/joint_states \
  /a200_0806/right_ur5e/rate_limiter/joint_states \
  /a200_0806/left_ur5e/rate_limiter/ft_sensor_wrench \
  /a200_0806/right_ur5e/rate_limiter/ft_sensor_wrench
```

- **Safety**:
  - In freedrive the right arm does not know the bar's weight (it is not in its payload setting).
    Support it by hand the whole time.
  - Before the first run, check that switching the **right** pendant to local does not stop the
    **left** arm's program (the multi-arm safety sync). The left arm must keep holding the bar.

### Experiment 1: the preload and its spring-back (no new code, about 30 min)

Arms at a bar-held pose (any insert start). Note the time of each step in the log.

1. **Tare.** Tools empty: `Zero Force Sensor (BOTH)`. Stay still for 10 s (sensor noise at rest).
2. **Mount.** Mount the bar and clamp it with the **tool buttons**. Take `Record + Fit + Viz (shared)`
   three times → takes **mounted**. Note both force readings.
3. **Relax by hand.**
   - Switch the right pendant to **local** and freedrive the right arm, supporting it.
   - Let it settle until the right arm's force reading stops changing (it should fall toward the
     bar-weight share).
   - Stop freedrive, switch the pendant back to **remote**.
   - Take three takes → **relaxed**. Note both force readings.
4. **Re-clamp.** Loosen the right grip and clamp it again with the tool buttons. Three takes →
   **re-clamped**. Does the preload come back?
5. **Repeatability.**
   - Unmount the bar, mount it again, and repeat steps 2–4 once.
   - Then repeat steps 2–4 once with another bar, or with the same bar flipped end for end
     (tells "per tool" from "per bar").
6. **Save.** `Save markerset data`; stop the bag. Send both, plus the step times.

What it gives:

| quantity | from |
|---|---|
| preload size and direction, per arm | force readings, mounted vs relaxed |
| relative flange shift that releases it | joint states before/after step 3 (FK) |
| stiffness of the two-arm hold, K | the two above together |
| **how far the bar moves on release** | bar fit, mounted vs relaxed (the number that matters for foundation bars) |
| whether re-clamping restores the preload | step 4 |
| per tool or per bar | step 5 (same shift every time = the tools) |

Decision rules:
- Bar moves **1 mm or more** on release → build Fix 1.
- Relaxing shift **the same** across remounts and bars → Fix 3 (calibrate the tools), instead of or
  as well as Fix 1.
- Shift **varies per bar** → the joints on the bars; Fix 1 is needed at every mount.
- After the relax the bar is still about 4 mm off → that is error B; Fix 2 / Fix 3.

### Experiment 2: a foundation bar end to end (after Exp 1)

1. Run a grounded bar's insert as today (rigid only).
2. At the assembled pose, three takes → **held**.
3. Tape the foundation, then ungrasp and retreat. Three takes → **released**.
   **Spring-back = released − held.**
4. Repeat once with a manual relax (Exp 1, step 3) before the insert.

Decision rules:
- Spring-back mostly gone with the relax → Fix 1 is the fix for foundation bars.
- What is left between the **released** bar and Rhino is what Fix 2 has to remove.
- Also read the retreat's check lines (`compliant part`, `contact`) in both runs for the
  hypothesis in section 5.

---

## 8. Related: the servo's own precision (IK engine)

Checked 2026-10-09:

| path | IK engine |
|---|---|
| `ik_live_base_for_selected_movement` (every servo pass) | PyBullet built-in IK via compas_fab, tolerance 1 mm / 0.01 rad |
| transfer IK check, workaround box ticked | PyBullet built-in IK (same function) |
| transfer IK check, box unticked | ssik (tamp `solve_endpoint_dual_arm_ik`) |
| servo corrections, insert / retreat straight lines, bar-held RRT | ssik at every waypoint |

- Every model is the same calibrated URDF, `husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf`:
  - the planner robot (from the problem's `RobotCell.json`, joint origins identical);
  - the drawn live robot and the green goal robot;
  - the ssik solvers (FK agrees with PyBullet within 0.0001 mm).
- Proposed: move the two PyBullet-IK paths to ssik (`ssik_ik.ssik_arm_branches`, the call the
  straight-line planner already makes). Keep PyBullet IK only as a logged fallback, and for the
  single-arm support robots, which have no ssik solver built.

This removes the 1 mm floor of the servo's own residual. It does **not** touch errors A or B, which
the servo cannot see at all.

---

## 9. Open questions and decisions

| # | question | status |
|---|---|---|
| 1 | Add joint values and the base pose to every marker take (two-line change), so a session can tell calibration, arm and tool apart? | open |
| 2 | Which markers will the real foundation bars carry? The export puts a marker set on `joint_J20-21_mocap` (B20's female half). Does Motive track it as a rigid body, or do we fit it like `bar_rig`? | open |
| 3 | In which frame does the UR driver publish the wrench (`ft_sensor_wrench`)? Needed to put both arms in one frame. | to check |
| 4 | OK to aim the bar loop (Fix 2) at "live flange + measured correction" (exception to "targets never from FK")? | open |
| 5 | Move the servo's live-base IK to ssik (section 8)? | open |

Results log (add below as experiments run):

- *(none yet)*
