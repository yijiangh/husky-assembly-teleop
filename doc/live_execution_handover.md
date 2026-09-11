# Handover: first live runs of the fixtureless bench on Cindy

**Written 2026-09-10, evening.** For a fresh session with no memory of this one. It covers the
day the pipeline first ran on hardware end to end -- what was fixed to get there, what the
robot then told us, and what is still open. Sibling notes own the details and are linked
rather than repeated:

- [pickup_calibration_manual.md](pickup_calibration_manual.md) -- the operator run guide
  (stages 1-7c + "After the experiment"); the commands below are all in it.
- [pickup_calibration_handover.md](pickup_calibration_handover.md) -- the calibration and
  gripper-mount work, including today's two findings (table height, mount roll).
- [open_loop_engine_handover.md](open_loop_engine_handover.md) -- the execution engine;
  its "what will bite you" list gained entries -0.7, -0.5, -1 today.
- [rtde_network_setup.md](rtde_network_setup.md) -- network, ROS/DDS env, gripper stacks.
- [compliant_insertion_handover.md](compliant_insertion_handover.md) -- the insertion skill
  (not touched today).

Two repos: the planner `~/Code/fixtureless-assembly` (Valentin's, private, branch
`yh/pickup-layout`) and this one (branch `yh/fixtureless_assembly`). Planner venv
`~/Code/fixtureless-assembly/.venv`; robot side `~/ros2_ws/venv` + `install/setup.bash`.
The experiment folder everywhere below: `$E = experiments/real-stool-husky-current`
(3-part `real-bench`; the folder name is historical).

## What the task is

Run the planner's fixtureless bench assembly (seat + two legs, one arm holds the seat while
the other inserts legs) on Cindy, the dual-UR5e Husky, from a measured part layout: print
the sheet, place the parts, touch the crosses, re-solve, motion-plan, simulate, export,
execute open-loop over RTDE with the Robotiq grippers driven over ROS.

## State (2026-09-10 evening; planner merge refreshed 2026-09-11)

**Nothing is committed on either side today. Nothing is pushed.**

| Repo | Branch | Committed today | Uncommitted |
|---|---|---|---|
| planner | `yh/pickup-layout` | `fb18385`, the second of two merges of `origin/master` on 2026-09-11, on top of `9fcd99e` (2026-09-10). Upstream since then: `691cc7f` "Rotate husky ee by 90", `772d9a4` Fabrica/standard profile, `65aa262` benchmark runners, `482276c` "Move waiting for gripper to sim". `9fcd99e` itself brought the 2026-09-09 batch (joint limits, capsules, over-table gate, gripper-close wait, vacuum tool, off-centre tolerance, closure `VERSION` 2) | `husky_calibrated.g` (mount roll + coupling), `analytic_ik.py` (husky tools), `tests/test_husky.py` (guard), `tests/test_grasp_standoff.py`, and the older `problem.py`/`config.py`/`main.py`/`replay.py` (`--grasp-standoff`), `export_layout.py` (packing) |
| teleop | `yh/fixtureless_assembly` | none (HEAD `cfc2860`) | everything from today plus the earlier open-loop/pickup work; see `git status` |

Also present: `.claude/plans/20260910-speed-scale-and-controller-writeup.md` and a
`speed_scale`/`mode` column in the run logs -- **another session's work in progress on the
engine** (a playback speed scale). It is in the same uncommitted `open_loop_engine.py`; do
not treat every engine hunk as this session's.

Unanswered questions, still: push scope (experiment artifacts in git? the
`husky_assembly_tamp` submodule bump?); `--grasp-standoff` removal (user leaned "remove", not
done); whether to revert the gripper mount roll (evidence below, user has not decided).

**Ephemeral state to know about:**
- At ~17:50 both arms stopped answering (`No route to host` to 192.168.131.40/.41 on 30004;
  they pinged fine at 17:35). The Husky's two gripper stacks are still running in tmux but
  their tool bridges are dead (`IO Exception: Requested 8 bytes, but got 0` ... `Failed to
  set gripper position`; goals ABORT instantly at the current position). That is what an
  unpowered tool port looks like: the grippers' 24 V comes through the arm. When the arms
  are back, `scripts/husky/start_gripper_stacks.sh --restart`, then stage 7b.
- By ~18:05 the Husky PC itself stopped answering (`ssh 192.168.131.1` times out), so
  the whole robot is off, not just the arms.
- The Husky's clock is wrong (`date` on it says May 11 2026); its log timestamps and
  `tmux ls` "created" dates are not comparable with anything here.
- `/tmp/live_q.json` (a saved RTDE pose) and the session scratchpad are throwaway.

## The pieces added or changed today

| File | Responsibility |
|---|---|
| `husky_assembly_teleop/utils.py` | `ROBOTIQ_COUPLING_M = 0.0112` (derivation in the comment); `TOOL0_FROM_GRIPPER_TCP` z is `0.152 + coupling` = 0.1632 |
| `husky_assembly_teleop/open_loop_engine.py` | `GRIPPER_MOUNT_POSE` (+90° roll, coupling); `GRIPPER_OPEN = 0.0` (was 0.426); `load_viz_grippers`; jaw fit (`_fit_every_grasp`, `_close_angle`, `[jaws]` log); `_rebase_traj_to_live_branch` + `BRANCH_LIMIT_MARGIN`; `_grippers_reachable` (START refuses without gripper servers); env in the banner; finger angle logged before the approach check |
| `husky_assembly_teleop/open_loop_approach.py` | `describe_collision` (names every colliding pair, joint-limit violations, arm-labelled grippers); `unwrap_goal` trajectory-aware; `OBSTACLE_LABELS` |
| `husky_assembly_teleop/open_loop_parts.py` | parts as exact concave trimeshes; `fit_jaws` (+ `opening_mm` = real pad gap), `held_chain`, `held_by`, `episode_start` |
| `husky_assembly_teleop/open_loop_traj.py` | accepts schema v2 **and v3**; `rebase_to_branch` |
| `husky_assembly_teleop/open_loop_insertion.py` | `InsertionParams.tcp_offset` default 0.1632 |
| `husky_assembly_teleop/pickup_calib.py` | VERIFY mode for a calibrated layout, `touch_drift`, mark touches now update `top_z`, load-time table warning, backup-then-overwrite save |
| `scripts/husky/live_pose_check.py` | live RTDE pose → engine's collision verdict at open/0.426/closed jaws, boxes vs meshes, tip-vs-box geometry, UR-controller FK vs URDF |
| `scripts/husky/live_pose_in_planner.py` | same joints in the planner's rai model (planner venv, any cwd) |
| `scripts/husky/open_grippers.sh`, `close_grippers.sh`, `gripper_env.sh` | open (0.0) / close (0.8) one or both grippers over the action server, with the Cindy ROS env |
| `scripts/test_open_loop_parts.py` | jaw-fit, engine-wiring and hull-vs-mesh tests; runs on the bench and the 5-part fixtures |
| planner `src/rai/husky/calibrated/husky_calibrated.g:189,194` | `Edit *_robotiq_base (*_tool0): { Q: "d(90 0 0 1) t(0 0 0.0112)" }` |
| planner `analytic_ik.py` husky tools | `Rz(+90)`, z 0.1412 |
| planner `tests/test_husky.py::test_husky_ssik_tool_calibration_tracks_the_real_gripper_mount` | pins the typed matrix to the loaded frames; asserts the closing axis is tool0 ±X |

Run flow: manual stages 3b → 4 → 5 → 6 → 7a → 7b → 7c, then "After the experiment".

## What will bite you

1. **The calibrated layout's table was 27 mm too high, and every plan before 16:29 today
   inherited it.** The 2026-09-09 take touched the crosses at z = 0.411-0.414 m but saved
   `top_z` = 0.440 (the planner's guess): `pickup_calib` counted mark touches as table
   evidence only in the readout; `_update_table_top` was called by the table buttons alone.
   Fixed (`on_record_mark`/`on_undo` call it), and the tool warns at load. The re-touch at
   16:29 saved `top_z = 0.4133` and found the sheet had also moved **(+3.4, +2.9) mm,
   +0.08°, rms 0.36 mm** since yesterday (drift report in the note's sibling). The plan
   solved at 16:36 is the first one on the right table height.

2. **The +90° gripper roll is probably wrong.** Evidence (all measured): the robot stood at
   yesterday's *un-rolled* trajectory's first pick (all six right-arm joints within 2.4°,
   wrist_3 within 1.1°; vs the rolled plan wrist_3 is 90.6° off); the photo shows the real
   pads straddling the leg ACROSS its width; at those joints the rolled model puts the pads
   ALONG the leg (tips ±50 mm along, centred on its width -- the "8.8 mm deep" collision)
   and the un-rolled model puts them across. The robot model is not the reason: the UR
   controller's own FK equals the URDF's at the live joints to 0.0 mm / 0.0° (right; left
   3.3 mm with the punch xy ignored), and PyBullet equals rai to 0.0 mm. Only the right arm
   was photographed. If reverted, all three mount sites change together (`.g`,
   `analytic_ik.py`, `GRIPPER_MOUNT_POSE`), the guard test flips to tool0 ±Y, and everything
   re-solves. Yet the grasps at 17:01/17:31 DID close on the legs (below). Unresolved.

3. **Gripper events `SKIPPED-no-server` = your terminal, not the robot.** The 16:42 run
   skipped all ten because the shell lacked `ROS_DOMAIN_ID=86 RMW_IMPLEMENTATION=
   rmw_cyclonedds_cpp` (only `CYCLONEDDS_URI` is in `~/.bashrc`); domain 0 on FastDDS sees
   nothing. With the exports set the 17:01 and 17:31 runs sent every event. START now refuses
   without both servers; the banner prints the env.

4. **"tracking error > --err-abort" is a symptom: the arms STOPPED OBEYING speedJ, and
   in two runs both arms stopped within 50 ms of each other.** Read from `log.npz`
   (`*_q_actual` displacement vs `*_qd_cmd`; the log's `t` is trajectory time, wall time
   is `t / speed_scale`):

   | run | speed_scale | froze | at (traj s) | what the log shows |
   |---|---|---|---|---|
   | 16:42 | (none) | -- | -- | ran the whole 52.1 s to `done`, max err 0.115 / 0.085 rad; grippers all skipped (item 3) |
   | 17:01 | 0.36 | both arms | 37.5 | no force event (right `\|F\|` a flat 35 N bias all run); left commanded up to 0.23 rad/s, right 0.57, neither moved a millirad; abort at 42.25 s when the right ref left its dwell |
   | 17:16 | 0.30 | both arms | 3.5 | 6.7 N blip; right w1 commanded 0.07 rad/s for 1.8 s, static; left settles 7 mrad on all six joints at the same instant (a stop, not a command) |
   | 17:31 | 0.20 | right only | 44.35 | 54 N hit at 38.4 s (approaching the seat re-grasp), **136 N** hit at 44.21 s, 0.15 s into the move away with the seat; left kept tracking at 2 rad/s with 1 mrad error to the end |

   The engine calls `rtde_c.speedJ` every cycle and never looks at its return value or at
   `getSafetyMode()` / `isProtectiveStopped()`, so the abort text names the consequence
   (the reference walked away from a standing arm), not the cause. The 17:31 stop after a
   136 N impact reads like a right-arm protective stop; the two simultaneous stops have no
   force signature and need the operator's memory or the pendants' Log tab (protective
   stop? safeguard stop? which arm first?). Simultaneous means a shared path: the arms'
   safety I/O, `multi_arm_safety_sync` (must be stopped for RTDE; `start_gripper_stacks.sh`
   refuses to start while it runs, so it was down when the stacks came up), or a pendant.
   Not resolved; the `speed_scale` feature landed between the run that completed and the
   runs that froze, so keep it in view.

5. **Grasps did happen.** Right closes on legs reported `GRASPED` at 0.597 rad / 21.5 mm
   (the leg body is 22 mm), seat grasps at 10.4 mm, opens at 84-85 mm; one left open at
   38.85 s came back `BLOCKED` at 21.5 mm in the 17:01 run and `OPENED` in the 17:31 run.
   An empty close stalls at **0.789 rad** (measured); `grasp_verdict` keys on that -- and
   on nothing else: the 17:16 run's first close stalled at 0.353 rad / **47.5 mm** and was
   still called `GRASPED`, though no 22 mm leg reads 47.5 mm (the parts had not been reset
   after the 17:01 abort). The engine already knows the expected width (`fit_jaws`'s
   `opening_mm`, 24.5 mm for a leg); the verdict does not use it yet.

6. **Merging planner master re-keys the grasp DB.** `gripper_closure.VERSION` 2 and the
   0.15 → 0.0015 m off-centre tolerance changed every certified grasp: the old plan was
   refused (`sampled grasp database hash differs`). Any pull from master means re-solve. The
   export schema is **v3** now (handover give delayed to the end of the receiver's dwell);
   the teleop loader accepts v2 and v3.

6b. **The 2026-09-11 merge had three conflicts, all resolved toward US on husky hardware
    numbers.** Upstream's `691cc7f` applies the same +90 degree mount roll we do (independent
    of us -- it is Valentin's own commit), but its tool z is `0.13`: the coupling-free value.
    Ours (`0.1412` = 11.2 mm coupling + 130 mm) wins in `analytic_ik.py`, and upstream's new
    literal pin in `tests/test_husky.py::test_husky_side_specific_ssik_matches_real_rai_fk_and_inverse`
    was updated to match, so the two pins and `husky_calibrated.g` now say the same thing.
    The other two: `main.py::_SAVE_CONFIG_FIELDS` keeps BOTH our `layout_json` and upstream's
    `ik_seed`; `problem.py` keeps upstream's `_profile_grasp_rows` timing wrapper around our
    `offset=None` grasp-standoff signatures. **If a later merge re-raises the 0.13 vs 0.1412
    conflict, ours is the measured one.**

7. **`--grasp-sampler-seed 1` is in the stage-4 recipe for a reason**: the default cloud's
   seat hold grazes the Husky's own top plate (`obj_1 ~ husky0_top_coll_middle: −1.06 mm`,
   same with `--seed` changed), and the chain gate refuses it. Redraw again rather than
   relaxing `--mp-collision-tolerance`.

8. **The approach check uses the VIEW's finger angle.** After a finished run that is the
   last fitted close, and a closed model finger beside a leg reads 15.8 mm deep where the
   open real one is clear. The engine logs the angle it checked with; open the grippers
   (real and view) before planning.

9. **The convex hull was lying about the leg.** Its snap-fit lug drags the hull out along
   the whole length (22.9-28.4 mm where the body is 22.0); the pads stopped at 30.2 mm.
   Parts are exact trimeshes now (24.5 mm = 22 + Bullet's ~1 mm/side margin). Bounding
   extents are a ceiling on a pinch, never a floor -- the seat is legitimately grasped at a
   12.6 mm rib.

10. **Coupling: 11.2 mm, from the manual, not from the old 0.012.** Robotiq's TCP table
    (171.0 mm flange→closed tips, coupling included) minus the closed height (162.8) gives
    8.2 mm for the legacy AGC-CPL-062-002; the drawing (13.9 mm plate less 3.0 + 2.9 mm
    pilot pockets) agrees; GRP-CPL-062 is documented 3 mm thicker. With it in both models
    the teleop and MuJoCo pad-tip clearances agree to ~1 mm (were 13 mm apart). Caliper
    check still owed: raised flange face → gripper body face ≈ 13.9 mm.

## Verification status

- Planner: targeted tests (`test_husky`, `test_analytic_ik_tools`, `test_gripper_closure`,
  `test_cograsp`, `test_config`, `test_grasp_standoff`) pass on the merged tree; full suite
  629 passed / 30 failed, the 30 pre-existing (27 `test_search_benchmark` FileNotFoundError,
  2 `test_layout` fixture staleness, 1 `test_yandv_stool` leg-roll quaternion).
- Teleop: `scripts/test_open_loop_parts.py` ALL CHECKS PASSED on the bench file and the
  5-part default; the engine's START gate and `describe_collision` exercised on stubs and
  headlessly; the gripper scripts ran against the real servers (left: close 0.789 stalled,
  open 0.003 reached) before the arms went away.
- Hardware: four live runs today; grippers commanded successfully in the last three; every
  run aborted (items 3-4). **No assembly has completed on hardware.** The insertion skill
  did not run today.
- NOT verified: the roll (item 2), the caliper check (item 10), anything about the
  speed-scale feature, the left gripper's mount orientation.

## Next steps

1. Bring the arms back, `start_gripper_stacks.sh --restart`, stage 7b, and re-run the
   drift check (stage 3b) -- a 5 mm move in a day says the sheet or the parts get bumped.
2. Before more runs, find out what stopped the arms (item 4): ask the operator / read the
   pendants' Log tab for 17:01 (traj 37.5 s, ≈ 104 s wall after START), 17:16 (3.5 s, ≈
   12 s wall) and 17:31 (44.35 s, ≈ 220 s wall). Then make the engine say it itself: poll
   `rtde_r.getSafetyMode()` / `isProtectiveStopped()` and `speedJ`'s return each cycle and
   abort with that reason. Add the fitted width to `grasp_verdict` (item 5).
3. The 136 N and 54 N hits in 17:31 (item 4) are unplanned contacts of the right arm near
   the seat; the preview shows free space there. Re-place the parts, run stage 3b, and
   watch that segment (traj 38-45 s) at a low speed scale before trusting the model.
4. Decide the roll (item 2): photograph the LEFT gripper at a part, or simply free-drive an
   arm so the pads visibly straddle a leg across its width and read wrist_3 -- compare with
   `scripts/husky/live_pose_check.py --traj ... --sample <pick>`; then revert or keep, and
   re-solve either way if anything changes.
5. Caliper the coupling (item 10).
6. Decide `--grasp-standoff` (remove or keep) and the push scope; commit.
