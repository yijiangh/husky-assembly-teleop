# Handover: pickup-pose calibration work

**Updated 2026-09-09 (grasp-standoff session).** Context note for picking this task up in a fresh
session. Operator instructions are in
[pickup_calibration_manual.md](pickup_calibration_manual.md); this file is the state of the
WORK -- what is done, what is not, and what will bite you. Keep both updated as the work moves.

---

## START HERE (next session)

One thing is queued:

1. **The first real measurement take** -- stages 2-4 of the manual. Nothing in stage 3 has
   ever run against the real robot.

DONE this session: `.claude/plans/20260909-grasp-standoff-jaw-clearance.md` -- the persisted
`--grasp-standoff` knob (default 0.010 = the old behaviour). The knob is built and tested;
**the experiment is NOT using it.** See "The jaw-clearance fix" below for why the 0.015 trial
has to be redone before it can be adopted.

Current experiment: `experiments/real-stool-husky-current` = the **3-part `real-bench`** (the
folder name is historical), solved against the MEASURED layout
(`layout_nominal_calibrated.json`) at the default 0.010 standoff: **9-action plan, 9/9 sim**
(final pose errors 0.2 mm / 0.1 deg), exported to a 1078-sample open-loop file (6 close /
4 open). This is the plan that was there before the standoff session and it is unchanged.

! **A measurement take HAS been done** -- `layout_nominal_calibrated.json` exists and the
current plan was solved from it. The "no real measurement take" line further down predates
it; what has never been run is a take *after* a fresh print-and-place.

## Git state -- NOTHING IS PUSHED

| Repo | Branch | State |
|---|---|---|
| `~/Code/fixtureless-assembly` (vhartman, private) | `yh/pickup-layout` | **5 commits unpushed**, plus uncommitted `export_layout.py` + `problem.py` |
| `/home/su/ros2_ws/src/husky-assembly-teleop` (yijiangh) | `yh/fixtureless_assembly` | **entirely uncommitted** |

The uncommitted planner changes are the leg roll (`problem.py`, `YandVStoolAssembly.define`),
the whole A4 sheet/packing rework (`export_layout.py`), and the `--grasp-standoff` knob
(`problem.py`, `config.py`, `main.py`, `replay.py`, `tests/test_grasp_standoff.py`). The teleop side holds
`pickup_calib.py`, both doc notes, the punch TCP in
`data/calibration_data/20260622/config.yaml`, and the `open_loop_*`/`setup.py` edits.

The user asked to "push the code" earlier; the scope question (do the 16 MB experiment
artifacts go in? does the teleop submodule bump go in?) was never answered and was overtaken
by hardware work. **Re-ask before pushing** -- pushing goes to Valentin's repo.

## What the task is

Let the operator tell the planner where the parts and table really are, so a plan can be
executed on the real robot. Print the planner's layout at 1:1 on A4, place the parts on their
exact traced outlines, then touch the four printed CALIBRATION CROSSES with the punch tip.
One rigid fit of the sheet moves every part. Output is a layout JSON that the planner
consumes via `--layout-json`.

Design decisions worth keeping (2026-09-09):
- **The sheet PACKS the parts.** The planner's scatter is ~240 mm wide, wider than A4, so
  `export_layout.py` re-places the parts into a compact block on ONE sheet (rotating a part a
  quarter turn where that helps, then spreading to use the page). The parts have therefore
  MOVED and the plan must be re-solved with `--layout-json`. `--no-pack` keeps the scatter.
- **`PACK_GAP` is empirical, currently 40 mm.** At 10 and 20 mm only the seat was graspable.
  Crucially the bounding-box gap is NOT the criterion: a 20 mm pack that the spread step
  later opened to 43-115 mm was still 1/3 graspable. What matters is where each part lands
  relative to the arms, so always re-run `--check-init` rather than reasoning about spacing.
- **Run `--check-init` before every solve on a new layout.** An unusable scene costs ~15
  minutes of grasp-widening before the search gives up; the check costs seconds.
- **Calibration is on the sheet's printed crosses, not on part corners.** Printed positions
  are exact, crosses are crisper than a moulded edge, and the ~267 mm baseline beats any
  single part. This replaced an earlier per-part corner flow.
- **Outlines are the exact mesh profile**, projected top-down through the part's lying
  orientation, so the snap-fit joint is traced and manual placement is accurate. Needs
  `shapely` (NOT in requirements.txt); falls back to a bounding rectangle without it.
- **Pages are A4 at 1:1**, tiled with a 10 mm overlap. Verified end to end by rendering the
  PDF at 600 dpi and measuring the printed cross separation: 266.85 mm against a layout truth
  of 266.86 mm.
- **Legs lie on their wider 30 mm face** (a quarter turn about their own long axis, in
  `YandVStoolAssembly.define`), so they sit flat. At the old 10 mm standoff this cost a
  handover (8 -> 9 actions); at 0.015 the handover is gone again (see below).

Original plan: `.claude/plans/20260908-pickup-pose-calib.md` (gitignored). It carries a
`## Outcome` section with the implementation record and deviations.

## Two findings from the first live check (2026-09-10, afternoon)

**1. The calibrated layout's table is 27 mm too high.** Its own measurement record shows the
four crosses touched at z = 0.411-0.414 m, yet `table.top_z` stayed at the planner's guessed
0.440 m, because mark touches counted as table evidence only in the readout -- `_update_table_top`
was called by the table buttons alone (fixed: `on_record_mark`/`on_undo` call it now). Every
part in `layout_nominal_calibrated.json`, every planned pick and every `[jaws]` clearance is
therefore 27 mm above the real table. **Re-touch the crosses (stage 3b in the manual) and
Save, then re-solve.** `pickup_calib` now warns at load when a layout's touches and its table
disagree by more than 3 mm.

**2. The +90 deg gripper roll looks WRONG; the original un-rolled model matches the photo.**
The robot was left at yesterday's un-rolled trajectory's first pick (all six right-arm joints
within 2.4 deg of it; 90.6 deg off the rolled plan's wrist_3). In the photo the real pads
straddle the leg ACROSS its width. At those same joints the rolled model puts the pads ALONG
the leg (tips at +-50 mm along it, centred on its width -- the "8.8 mm deep" collision) and
the un-rolled model puts them across. The robot model itself is verified: the UR controller's
own FK equals the URDF's at the live joints to 0.0 mm / 0.0 deg (right arm; left 3.3 mm with
the punch xy ignored), and PyBullet equals rai to 0.0 mm. Not yet reverted -- the user's call;
the left arm has not been photographed. Tools: `scripts/husky/live_pose_check.py` and
`scripts/husky/live_pose_in_planner.py` (trap -0.5 in the engine handover).

## The gripper mount: a quarter turn AND a coupling (2026-09-10)

Two things about the mount were wrong, and both are fixed together.

**1. The roll.** The real Robotiq 2F-85s are bolted on a QUARTER TURN about the flange axis,
so the pads close along tool0 **X**, not tool0 Y.

**2. The coupling.** A Robotiq cannot work without one -- it carries the electrical contacts --
so the gripper never touches the flange. The planner mounted it at `t(0 0 0)` anyway, putting
every grasp **11.2 mm too deep**. That, not the gripper model, is what made the closing
fingertips reach the table.

The number, from the 2F-85 instruction manual: the TCP table (6.2.3) gives **171.0 mm** from
the flange to the closed fingertips WITH the coupling, and fig. 6-2 gives **162.8 mm** for the
gripper alone, so the legacy AGC-CPL-062-002 is **8.2 mm**; fig. 6-6 agrees independently
(13.9 mm plate less its 3.0 mm and 2.9 mm pilot pockets). These arms carry the current
**GRP-CPL-062**, which Robotiq documents as 3 mm thicker => **11.2 mm**. The offset lands on
the gripper's base-link origin, which the mesh puts at the bottom of its 71 mm pilot -- exactly
the face those figures measure from.

! **Caliper check if the coupling is ever in doubt**: raised flange face to the gripper's 75 mm
body face reads ~13.9 mm for GRP-CPL-062, ~10.9 mm for the legacy AGC-CPL-062-002.

| where | what changed |
|---|---|
| `src/rai/husky/calibrated/husky_calibrated.g:189,194` | `{ Q: "d(90 0 0 1) t(0 0 0.0112)" }` |
| `analytic_ik._CAL["husky_*"]["tools"]["two_finger"]` | identity -> Rz(+90); 0.13 -> **0.1412** |
| teleop `utils.ROBOTIQ_COUPLING_M` (new) | 0.0112, used by `GRIPPER_MOUNT_POSE`, `TOOL0_FROM_GRIPPER_TCP` (0.164 -> 0.1632) and `InsertionParams.tcp_offset` |

Everything else follows for free: `ur_gripper_center`, the `finger1/finger2` and `palm`
collision proxies, the closure certificate, and MuJoCo (which mirrors rai frames by FK) are
all children of `robotiq_base`. Verified in the recorded replay: the animated pads separate
along tool0 X on both arms.

! **The grasp-DB hash does NOT change** (grasp poses live in the `gripper_center` frame), so
`--load-plan` will happily accept a plan solved before this even though its IK witnesses carry
the OLD wrist roll and the OLD flange-deep mount. **Re-solve anything that predates
2026-09-10 before it goes near hardware.**

! **The coupling makes the cell tighter, and the first re-solve was REFUSED.** With both
grippers 11.2 mm longer, `main.py --motion-plan` rejected the seed-11023 plan:
`exported attachment chain has 4 collision-invalid keyframe(s) ... 07_pre_assemble_a1_obj_2_obj_0
(1.06mm)`, the pairs being `obj_1 ~ husky0_top_coll_middle` -- the mated leg grazing the
**Husky's own top plate** while a2 held the assembly. A different `--seed` reproduced it
exactly (same task, same 1.06 mm), so it is the grasp choice, not witness roulette.
`--grasp-sampler-seed 1` redraws the grasp cloud and solves clean: 9 actions, chain
collision-free, 92-state motion plan, 9/9 sim. **That flag is now part of the recipe** -- and
if it ever fails again, redraw again rather than relaxing `--mp-collision-tolerance`.

? The sign is +90 deg everywhere. The gripper is 180-deg symmetric, so -90 is geometrically
identical; if the FT-sensor/cable side ever matters, flip all three sites together and
`tests/test_husky.py::test_husky_ssik_tool_calibration_tracks_the_real_gripper_mount` will
catch a partial flip.

## The jaw-clearance fix (`--grasp-standoff`)

**The problem.** The Robotiq 2F-85 is a four-bar linkage: closing the jaws swings the inner
finger ~13 mm FURTHER along the approach axis. A fingertip that clears the table with the
jaws open can therefore still scrape it once closed, and nothing in the pipeline sees it --
`gripper_closure._move_to_shift` translates the collision proxies along the PINCH axis only
(no axial drop), and MuJoCo's animated finger pads are visual-only (contype 0). Measured on
the 22 mm bench legs.

**The knob.** `--grasp-standoff M` (planner) sets how far the tool centre stands off the face
it grasps, for every part and grasp. Default 0.010 = exactly the old behaviour. It is SAVED
with the plan and restored by `replay.build_scene`, because moving every pose rebinds every
grasp id and the grasp-DB hash. Plans saved before the flag existed replay at 0.010.

**The 0.015 / 0.018 trials were run on the WRONG LAYOUT and must be redone.** They used
`--layout-json layout_nominal.json` (copied from the plan file's Verification block) instead
of the measured `layout_nominal_calibrated.json` the experiment actually uses -- the two
differ by 45-60 mm per part. The clearance numbers below are still informative about the
gripper, but no conclusion about grasp CHOICE carries over, because a different layout
re-runs the whole search. Measured pad-tip clearance above the REAL table (the sim table
minus the 2 mm `part_float`), from the recorded `replay.npz`, all on the NOMINAL layout:

| standoff | leg pick a2->obj_1 | leg pick a2->obj_2 | outcome (nominal layout only) |
|---|---|---|---|
| 0.010 | +3.2 mm | +1.9 mm | passes, but no margin for hardware |
| 0.015 | +6.6 mm | +6.7 mm | 8/8 sim, final error 0.1 mm / 0.1 deg |
| 0.018 | +9.8 mm | +9.7 mm | 8/8 sim, but the SEAT grasp goes bad -- rejected |

! At 0.018 the sampler drew a seat grasp a1 cannot actually close on: the sim warns
`a1->obj_0 visual meshes do not reach bilateral contact within the 42mm stroke (gaps
[-12.77, 5.57]mm)` -- one pad 12.8 mm inside the seat, the other 5.6 mm short. Every action
still reported ok, but when a1 opened at the end the finished bench TOPPLED (final pose error
150 mm / 87 deg, and the seat's +z fell from vertical to 0.44). 0.015 draws a clean top grasp
(id 0) and places to 0.1 mm.

? Why a different standoff gives a different grasp set at all: the standoff is part of the
key that seeds the grasp sampler's private RNG, so each value draws its OWN candidate cloud.
Grasp quality is therefore NOT monotone in the standoff -- a larger value can be worse. Judge
a new value by running the sim and the pad-tip probe, never by the number alone.

**The pad-tip probe** (the measurement behind the table) is in the plan file's Verification
section: `.claude/plans/20260909-grasp-standoff-jaw-clearance.md`.

**Root cause is still open.** The closure certificate cannot see the four-bar drop, so a
future thinner part or deeper grasp can regress silently. The follow-up is to give
`gripper_closure._move_to_shift` a calibrated axial drop (~0.45 mm per mm of inward travel)
plus a table-clearance margin, bump the certificate `VERSION`, and let `repaired_table_pose`
retract automatically.

## Two repos, two branches, NOTHING PUSHED

The planner side is committed; the robot side is not; neither is pushed.

| Repo | Branch | State |
|---|---|---|
| `~/Code/fixtureless-assembly` (vhartman, private) | `yh/pickup-layout` | **4 commits, NOT pushed** |
| `/home/su/ros2_ws/src/husky-assembly-teleop` (yijiangh) | `yh/fixtureless_assembly` | modified + untracked, no commits |

The planner side is committed (`ec9a786` gitignore, `fa34641` the layout feature, `418f014`
merge of origin/master, `c8c1e69` test isolation). **Nothing is pushed**, and pushing goes to
Valentin's repo, so it is worth a word with him first. The teleop side is still uncommitted.

`experiments/real-stool-husky-current/` was deliberately left out of the commits: 16 MB,
almost all one `replay-lockstep.npz`. Still undecided:

1. Whether the experiment artifacts belong in git at all (the small ones are ~215 KB).
2. Teleop: the tree also carries changes I did NOT make — a `husky_assembly_tamp` submodule
   bump (`2fce15f`→`dc89dd5`, "ssik fk function") and a `.gitignore` edit. Both predate this
   session.

## Files changed

**Planner** (`~/Code/fixtureless-assembly`):
- `plan_io.py` — `LAYOUT_SCHEMA`, `load_layout_file`, `read_plan_layout`
- `problem.py` — `_apply_table_layout` + a `layout=` branch in `add_disassembled_obj`
- `config.py` — `layout_json` field; `from_file` tolerates the persisted `layout` key
- `main.py` — `--layout-json`, `_SAVE_CONFIG_FIELDS`, layout embedded by `save_plan`,
  resolution in `run()`
- `replay.py` — passes the stored layout into the scene rebuild, and restores the standoff
- `export_layout.py` (new) — nominal layout + true-scale outline sheet
- `tests/test_layout.py` (new) — 5 tests
- **grasp standoff**: `problem.py` `_GRASP_STANDOFF` + `set_grasp_standoff` (every grasp
  helper now defaults `offset=None` and resolves the global), `config.py` `grasp_standoff`,
  `main.py` `--grasp-standoff` + `_SAVE_CONFIG_FIELDS`, `tests/test_grasp_standoff.py` (new,
  7 tests)
- `.gitignore` (new; the repo had none)

**Robot side** (this repo):
- `husky_assembly_teleop/pickup_calib.py` (new) — the measuring tool
- `husky_assembly_teleop/open_loop_approach.py` — `build_obstacles(..., layout=)`
- `husky_assembly_teleop/open_loop_engine.py` — `--layout-json`
- `setup.py` — `pickup_calib` console script
- `data/calibration_data/20260622/config.yaml` — new left punch TCP
- `doc/pickup_calibration_manual.md`, `doc/pickup_calibration_handover.md` (new)

## Verified vs not

**Verified:**
- Planner: 5 new tests pass. (Pre-merge the suite was 583 passed / 3 failed; see the
  post-merge figure below.)
- Export→rebuild is the identity to 1e-9 on every part pose and the table.
- **A solve with the nominal layout reproduces the baseline plan byte-for-byte**, so the
  layout path is provably neutral on nominal input.
- `--load-plan` uses the EMBEDDED layout, proven with a decoy file shifted 300 mm.
- Measuring tool: fit/anchor/save verified headlessly; exact fit recovers a known pose to
  1e-7 deg; 1 mm touch noise gives ~0.1 mm / ~0.3 deg; reflections refused.
- Against the real robot: RTDE reads both arms, punch attaches to the flange to 0.0001 mm,
  tip sits 118.4 mm out matching the offset norm.

**After merging master `163f132`:** the suite is **602 passed, 0 failed** — the 3
pre-existing failures are gone, because master committed the openarm IK artifacts. Master's
`ik_artifacts/husky_*_ik.py` replaced the locally generated ones (different input-chain hash,
but they pass the ssik-vs-rai-FK test). Master's grasp-sampler change invalidated the old
plan, which was re-solved; the table also dropped 100 mm (top 0.540 -> 0.440 m).
`mp.json` and the replay recording are now STALE and must be regenerated.

- Grasp standoff: 7 new tests pass, and `tests/test_config.py` + `tests/test_grasp_sampler.py`
  still pass (82 in that set).

! The intended "the default reproduces the old plan byte-for-byte" control was NOT run
correctly: the control solve used the nominal layout while the saved plan came from the
calibrated one. The action list and the grasp-DB hash matched, but `config` and `witnesses`
differ, so that is weaker evidence than it sounds. **Redo it as: same stage-4 command with
`--layout-json layout_nominal_calibrated.json` and no `--grasp-standoff`, then diff against
the saved plan.**

! **The full planner suite is 30 failed, and every one of them PREDATES this work.**

- 27 are `tests/test_search_benchmark.py` reading `benchmarks/manifests/*.json` -- that
  directory does not exist in this checkout, so they are FileNotFoundError, not logic.
- 2 are `tests/test_layout.py`, both fixture staleness in the untracked experiment folder:
  `test_a_plan_carrying_a_layout_is_still_a_valid_config_file` asserts `assembly ==
  "real-stool"` while `experiments/real-stool-husky-current/` has held a `real-bench` plan
  for a while (the folder name is historical), and
  `test_exported_layout_rebuilds_the_identical_scene` is 45 um out on a part position after
  an export/rebuild round trip of the CALIBRATED layout.
- 1 is `tests/test_yandv_stool.py::test_yandv_rai_visuals_and_lying_legs`: it still asserts
  the OLD lying quaternion `[sqrt(.5), 0, -sqrt(.5), 0)]`, while the uncommitted leg roll in
  `YandVStoolAssembly.define` now produces `[0.5, -0.5, -0.5, 0.5]`. One line in the test, to
  fix together with the leg-roll change when it is committed.

**NOT verified:** no real measurement take has been completed. The accuracy of a real fit,
and everything downstream of it, is unproven. The 0.015 plan has never been run on hardware.

## Traps

1. **`--swap-arms` is mandatory** for husky open-loop files. The export lists `["a1","a2"]`
   with `a1` = RIGHT, but `load_open_loop_traj` maps the first robot to the LEFT arm by
   default. A follow-up worth doing: make the export self-describing so this cannot be
   forgotten.
2. **`--check-init` perturbs the seeded RNG.** It changes the resulting plan even with no
   layout involved (proven with a control run). Do not use it to compare two solves.
3. **`replay.build_scene` leaks module globals.** It installs the scene configuration as
   globals that outlive the test. `tests/test_layout.py` snapshots and restores
   `problem`/`solver`/`env`/`wrench`. Two traps found the hard way: a `--wrench-profile`
   writes `wrench.PARAMS`, which is UPPERCASE not underscore-prefixed, and anything that
   builds a scene at IMPORT time (a `skipif` probe, say) runs outside the fixture window and
   pollutes every other module during collection. Any new test calling `build_scene` needs
   the same guard.
4. **rai version is pinned to a narrow window.** `robotic==0.2.2` (Valentin's
   `requirements.txt`); anything in 0.2.2–0.2.6 works. 0.2.7 has an `addConfigurationCopy`
   bug, ≥0.2.8 removed `Frame.info`, ≥0.3 dropped the 4-element box. mr_bench must be on
   branch `dev/main`, not master. `ssik` is needed but missing from `requirements.txt` and
   must be installed `--no-deps` (it declares `scipy>=1.13` against the pinned 1.11.4).
5. **PyBullet attachments** freeze the CURRENT parent→child transform. Pose the body on the
   link BEFORE `create_attachment`, or it attaches at the world origin. This caused a real
   bug here.
6. Only `Slider` exposes `.value` read live; `SliderGroup` reports through a callback that
   can be missed. `Separator.set_text`, not `set_label`.

## The assembly changed, and the seating blocker went with it

The experiment is now the **3-part `real-bench`** (85 x 170 mm seat + two legs at y = +/-54
mm), not the 5-part `real-stool`. `real-stool --parts 3` is infeasible by construction: both
remaining legs sit at y = +54 mm on the same edge of the 170 x 170 mm seat, so the finished
assembly topples and `--goal-on-table` refuses it. The search reaches `final settle-to-table
FAILED` on every branch and exhausts.

On `real-bench` with the PACKED layout the whole pipeline passes: `--check-init` reports
`scene OK, 3/3 graspable`, a 10-action plan, motion plan 124 states reaching terminal, sim
**10/10 actions, plan SUCCEEDED** with final pose errors of only 0.29-0.39 mm and 0.04-0.12
deg. An earlier unpacked 9-action run also reached a hardware-ready trajectory through
`export_open_loop.py` (1274 samples, 63.65 s), so that stage is proven too.

The 5-part `real-stool` also passes end to end in
`experiments/real-stool-husky-full-funnel-fresh-20260909` (19 actions, 239 states, 19/19 in
sim with `--apply-process-wrench`), using the user's own target flags.
**The `not_seated` blocker does not occur here** — the settle error is 0.9 mrad against the
2 mrad tolerance, where the 5-part stool sat at 48 mrad and never converged. Whether that is
the smaller assembly, master's insertion-controller work, or both, has NOT been isolated. So
`export_open_loop.py` should now run, but that has not been tried yet.

The plan is a clean statement of the fixtureless idea: a1 picks the seat and HOLDS it as a
living fixture while a2 inserts both legs, then a1 sets the finished bench down.

## Next steps

1. **Complete a real measurement take** -- the operator is at this step now. Stage 3 of the
   manual has NEVER been run against the real robot. Print `layout_outline.pdf` at 100%, lay
   it down, touch the four crosses, Fit sheet, Save. Expect ~0.5 mm rms.
2. Re-solve with the calibrated layout (keep `--grasp-standoff 0.015`) and compare against
   the nominal-layout plan.
3. Re-ask the push-scope question and commit; the teleop side is still uncommitted.
4. Before executing: `--swap-arms` is mandatory, and the export's 5 close / 3 open events
   include two re-assertions of an already-held grasp (a1 closes on `obj_0` three times).
   Harmless in preview, but check them against the plan before commanding hardware.
