# Handover: compliant insertion, part visualization, grasp verification

**Written 2026-09-09.** Context note for picking this task up in a fresh session. Operator
instructions live in [open_loop_engine_handover.md](open_loop_engine_handover.md) (the UI
flow, the flags) and [rtde_network_setup.md](rtde_network_setup.md) (network, gripper stacks,
the Robotiq driver fix); this file is the state of the WORK — what is done, what is not, and
what will bite you.

Sibling notes: [pickup_calibration_handover.md](pickup_calibration_handover.md) owns the
layout-measuring tool that feeds `--layout-json`; [controllers.md](controllers.md) is the
math — the joint tracker's law and the insertion skill's hybrid law, spiral and state
machine, with every symbol mapped to its code name.

---

## What the task is

The open-loop engine replays planned joint angles. That is fine everywhere except the
**mates**: a stool leg tenon is 30 x 22 mm and its mortise 32 x 32 mm, so one millimetre of
accumulated pickup and grasp error on the tight axis jams the part on the rim. Three pieces
were built:

1. **Part visualization** — draw the actual parts in the engine's 3D view, so the operator
   can see what each gripper carries and watch a leg go into the seat.
2. **A force-aware insertion skill** — take over the last few centimetres of each mate under
   force control, with a spiral search that finds the hole instead of assuming it.
3. **Grasp verification** — read the gripper action's result so a missed grasp stops the run
   instead of continuing with an empty gripper.

Plan file: `.claude/plans/20260909-compliant-insertion-controller.md` (gitignored). Its
`## Outcome` section carries the implementation record.

## State: NOTHING COMMITTED

Branch `yh/fixtureless_assembly`, last commit `cfc2860`. Everything below is uncommitted in
the working tree, alongside the pickup-calibration work from an earlier session.

| New file | Responsibility |
|---|---|
| `husky_assembly_teleop/open_loop_parts.py` | `PartTracker`: where every part is at every sample (world / arm / part parent chain), `load_part_bodies`, `part_world_pose` |
| `husky_assembly_teleop/open_loop_insertion.py` | `InsertionController`: the force-controlled skill, `InsertionParams`, `spiral_offset`, `deadband`, `pose_error` |
| `scripts/fake_rtde.py` | Fake RTDE pair + `VirtualMortise` contact model, so everything runs with no robot |
| `scripts/bench_insertion.py` | ONE insertion on one arm from the live pose, no trajectory, no engine — the first hardware step |
| `scripts/test_open_loop_parts.py` | 17 checks on the part tracking (`--png` renders the cell) |
| `scripts/test_open_loop_insertion.py` | 24 checks on the skill against the virtual mortise + `find_insertions` |
| `scripts/test_open_loop_engine_flow.py` | 39 checks driving the real node through the phase chain |
| `scripts/test_data/` | `stool_layout.json` / `stool_plan.json` fixtures (see trap 8) |
| `scripts/husky/patch_robotiq_driver.py` | The Robotiq driver fix, to run ON a husky |

| Modified file | What changed |
|---|---|
| `open_loop_traj.py` | `OpenLoopTraj.attached` / `.arm_of_robot`; `Insertion` dataclass + `find_insertions` |
| `open_loop_engine.py` | Part drawing; the whole `--insertions` phase machine; wrench window + logging; grasp verdicts |
| `husky_robot.py` | `send_gripper_cmd(..., on_result=)` — optional result callback, default unchanged |
| `common.py` | `Toggle.value` (live read, mirroring `Slider.value`) |
| `doc/rtde_network_setup.md` | Robotiq force/speed facts, the applied fix, the pull test |
| `doc/open_loop_engine_handover.md` | Operator sections for `--layout-json` / `--plan-json` / `--insertions`; traps 0 and 0b |

**Open question, never answered** (inherited from the pickup-calibration session): what scope
to commit and push. Do not push without re-asking.

Also modified and not from this work: the `husky_assembly_tamp` submodule bump and a
`.gitignore` edit, both predating it.

## How to run it

```bash
cd ~/ros2_ws && source venv/bin/activate && source install/setup.bash
export ROS_DOMAIN_ID=86 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml

# preview with the parts drawn (no robot)
ros2 run husky_assembly_teleop open_loop_engine <traj.json> --swap-arms \
    --layout-json <layout.json> --plan-json <plan.json>

# execute with the mates run compliant (default is OFF: plain speedJ as before)
ros2 run husky_assembly_teleop open_loop_engine <traj.json> --swap-arms --execute \
    --plan-json <plan.json> --insertions --ins-push-force 10 --ins-guard-force 30

# off-robot tests
python3 src/husky-assembly-teleop/scripts/test_open_loop_parts.py
python3 src/husky-assembly-teleop/scripts/test_open_loop_insertion.py
python3 src/husky-assembly-teleop/scripts/test_open_loop_engine_flow.py
```

A run with `--insertions` becomes `track → insert → release → track`. The insertion skill is
`zero → approach → search → insert → [backoff] → seated`; on failure both arms park in a
**`held`** state and three buttons decide (Retry / Release & continue / Abort). Logs gain
`{side}_wrench`, `{side}_mode`, per-insertion `ins<k>_*` arrays, `insertions.json` and one
`insertion_<k>.png` per mate.

## What will bite you

**1. The per-sample `servo_controller` flag does NOT mark the insertions.** It is
`controller_mode != 'trajectory'` in the sim export
(`~/Code/fixtureless-assembly/export_open_loop.py:357`), bundling the settle, wrench-guard,
approach-pacing and place-servo windows. In `open-loop.json` it is true on **1110 of 3895
samples across 57 windows, and the four leg→seat mates contain none of them**. Mates come
from `plan.json`'s `assemble` actions instead (`open_loop_traj.find_insertions`), each tied
to the gripper-open that releases that part.

**2. A mate is a straight line in the HOLDING arm's frame, not the robot base.** The planner
builds it as a pure translation of the child relative to the parent, and the parent-holding
arm is moving too — in the base frame the same motion is a curve. Measuring straightness in
the base frame found only **2 of 4** mates; in the holder's frame, 3 of 4 (51 / 48 / 32 mm).
The fourth (obj_2) is a real **7.6 mm** funnel — the planner's fraction ladder
(`solver.py:4347-4361`) shortens a funnel when the full 50 mm will not plan — and is skipped
below `--ins-min-depth`.

**3. A mate is a part's LAST gripper-open.** An earlier one is a handover give. Matching the
first open put obj_4 onto the seat at t=135.20 s (the handover) instead of t=149.65 s (the
real mate). Symptom: a part "assembles" then is picked up again.

**4. Match mates by CHILD part, not by (robot, child).** The `plan.json` on disk disagreed
with `open-loop.json` about which arm assembles obj_3 (plan said a1, trajectory used a2) —
they are different solves. The trajectory is the truth about which arm acts; the mismatch
is warned about, not fatal.

**5. Straightness must not be judged on a short chord.** A mate ends with a 1–2 mm dwell, and
over such a chord any jitter reads as a huge deviation; obj_3 came out as a 1.5 mm funnel
until the stray test was gated on `length > max(0.005, 2*tol)`.

**6. A position reference tied to the measured position never builds force.** The first
control law used `p_ref = p_measured + speed*dt*4`, so when the part touched something the
reference stopped advancing too and **contact was never detected** — every case ended in
`budget`. The law is now hybrid: across the insertion axis, position control (that is what
the search steers); along it, force control (`v = A*(f_axial - f_target)`), which presses
home without knowing exactly how far away home is. The dead-band belongs only on the axes
whose target force is zero — dead-banding a regulated 10 N push just biases it by the
dead-band's width.

**7. "On the rim" and "inside but blocked" need separating.** A spiral that runs away from
the part (lateral tracking error > `constrained_lateral`) means the part cannot move
sideways: it is in the joint and jammed, which is a back-off-and-retry, not more searching.
Without this a fouled pocket reported `search_exhausted` instead of `stalled`.

**8. The experiment directory was re-solved mid-session.**
`~/Code/fixtureless-assembly/experiments/real-stool-husky-current/` is now **`real-bench`,
3 parts** (12:30 on 2026-09-09), while the trajectory in
`~/Insync/.../fixtureless_assembly_trajs/open-loop.json` is the 5-part `real-stool` from
09-08. **No layout or plan on disk matches that trajectory any more.**
`scripts/test_data/stool_{layout,plan}.json` is a reconstruction for the tests only. A real
run needs a layout and plan exported from the same solve as its trajectory.

**9. A wrong layout is silent unless you check it.** The test fixture initially had the table
at 0.44 m (copied from the bench layout); the truth is **0.54 m**, and the grasps happened
91 mm above where the parts were drawn. `PartTracker` now checks the first grasp of every
part against the layout (gripper TCP inside the part's own box + 50 mm) and says so in the
startup summary: *"this layout does NOT match this trajectory"*. A 100 mm table error is
caught on 5 of 5 parts.

**10. The holding arm is frozen during a mate, but the plan may move it.** For obj_3 the plan
moves the holding arm 38 mm while the leg goes in. The mate itself is unaffected (it is
defined relative to the holder), but the assembly ends that far from its planned world pose.
`find_insertions` warns above 10 mm; `--resume-blend` blends the offset out afterwards.

**11. The insertion thread must set `stop_evt` when it ends.** The holding arm's tracker loop
has no end of its own — it stands still until told to stop. Without it, `all(thread_done)`
never became true and the phase hung forever with both arms holding.

**12. An EMPTY gripper close reports `stalled`, not `reached_goal`.** Measured on both of
Cindy's grippers: the fingers stop at **0.7894 rad**, and the 0.8 rad target overshoots that
by more than the controller's 0.01 rad goal tolerance. So the flags alone cannot tell a grasp
from air. Closes are judged by **final position** instead: past `GRIPPER_EMPTY_ANGLE`
(0.77 rad) → MISSED. A 22 mm leg stops at ~0.597 rad (jaw 21.5 mm), the 35 mm seat plate
near 0.47.

**13. `GripperCommand.max_effort` never reaches the hardware.** The husky runs stock
`ros-humble-gripper-controllers 2.45.0`, whose `GripperActionController` has no
`use_effort_interface` parameter (a rolling-era option); the `use_effort_interface: true` in
`crl_robotiq_controllers.yaml` is silently ignored and `set_gripper_max_effort` stays
unclaimed. Grip force comes from the driver alone — see `rtde_network_setup.md`.

**14. PyBullet allows one GUI connection per process,** and the engine opens one. The engine
flow test therefore runs its second and third scenarios as subprocesses of itself
(`--case 2` / `--case 3`).

**15. Small API facts.** `Separator` takes no `parent=` (role lines live in the control
panel, not the Wrench window). `Toggle` had no live `.value` — one was added to `common.py`
mirroring `Slider.value`, because a callback-kept flag can be missed.

## Verification status

**All off-robot, all green as of 2026-09-09** — 80 checks:

| Script | Checks | Covers |
|---|---|---|
| `test_open_loop_parts.py` | 17 | timelines; re-parenting moves a part < 0.2 µm; handover vs mate; layout-mismatch detection; `_draw_parts` itself |
| `test_open_loop_insertion.py` | 24 | the skill at 0 / 3×1 / 12 mm error, a fouled pocket, the force guard; `find_insertions` on the real file |
| `test_open_loop_engine_flow.py` | 39 | the real node through track → insert → release → track → done; the held-on-failure path; **and that without `--insertions` the run is a single tracking phase with no insertion machinery touched** |

Notable measured results: the skill seats at 0 error and at 3×1 mm error (catching at
r = 3.2 mm), reports `search_exhausted` at 12 mm, `stalled` on a fouled pocket after exactly
one back-off, `wrench_guard` on an obstruction; the holding arm drifts 4.79 mrad over 588
cycles during a mate.

**NOT verified — the insertion controller has never touched a robot.** Everything above ran
against `scripts/fake_rtde.py`, whose contact model is a stiff spring with no mass, no
friction and no gripper compliance. `bench_insertion.py` has not been run at all, not even
`--dry-run`.

**What DID run on hardware** (Cindy, 2026-09-09):
- The Robotiq driver fix: applied, rebuilt, gripper stacks relaunched. Close 1.8 s / open
  1.1 s end to end on both arms (was ~4 s of finger travel alone).
- A grasp on a real leg: stalled at 0.597 rad, jaw ~21.5 mm.
- A hand pull test on the left arm's wrist FT: the leg held **~50–60 N** (peak 62 N, first
  give ~49 N) at the 0.25 force multiplier (~74 N clamp) — effective pad-on-wood μ ≈ 0.35–0.4.
  Raw recording in `data_experiment/robotiq_grasp_calibration/20260909-pull-test-left/`.
  That puts slip (~55 N) above the wrench guard (40 N) above the push (10–30 N), which is the
  ordering the insertion needs.

**Asserted from documentation, not measured:** `getActualTCPForce` returns the wrench at the
TCP in the **base** frame. Nothing has confirmed the signs on Cindy. `Connect RTDE` now logs
each arm's pendant TCP offset and raw wrench for exactly this check.

## Live and ephemeral state

- **The gripper-only stacks are RUNNING on the husky** in tmux (`gripper_left`,
  `gripper_right`). Kill them before launching the full `crl_dual_ur5e.launch.py` stack —
  two tool-communication bridges fight over TCP 54321.
- The husky's driver files are patched with backups beside them
  (`hardware_interface.cpp.bak-20260909`, `2f_85.ros2_control.xacro.bak-20260909`);
  `git checkout` in `~/workspace/src/ros2_robotiq_gripper` also restores them.
- `.claude/settings.local.json` gained allow rules for ssh/scp to `administrator@192.168.131.1`.
- The husky's wall clock reads four months behind (2026-05-11) — its log timestamps look odd.
- Scratchpad files under `/tmp/claude-1000/.../scratchpad/` (a rehearsal copy of the husky's
  driver sources, the parts render) will be garbage-collected; nothing depends on them.

## Next steps

1. **Confirm the FT frame before commanding any push.** `Connect RTDE` (or
   `bench_insertion.py --arm left --dry-run`), then push each tool by hand along base
   +x / +y / +z and check the logged signs. Write the result into
   `open_loop_engine_handover.md` trap 0b.
2. **One bench insertion**: `scripts/bench_insertion.py --arm left --depth-mm 30 --axis tool-z`
   with a leg in the gripper and the seat clamped or held. Inspect `insertion.png`.
3. **Get a layout + plan that match a trajectory** — re-export all three from one solve
   (trap 8), then check the startup summary raises no layout complaint.
4. **In the engine**: `--insertions --end-sample` just past the first mate, then extend.
5. Re-ask the push-scope question and commit.
6. Deferred: the same driver patch on Alice and Belle
   (`scripts/husky/patch_robotiq_driver.py`); per-grasp force under humble would need a
   `forward_command_controller` on `set_gripper_max_effort`.
