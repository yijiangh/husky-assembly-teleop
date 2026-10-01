# Session note — 2026-10-01 (pick up here)

Session: support robots in the live monitor ("multi-robot monitor rebuild").
Spec + full change log: `tasks/2026-09-30_support_robot_schedule_monitor.md`.
Approved plan + exporter-defect note: `~/.claude/plans/i-am-about-to-clever-lerdorf.md`.
Operator manual: `doc/support_robot_schedule_manual.md`. Test manual: `doc/support_robot_test_manual.md`.

## 1. State of the code

- Repo `husky-assembly-teleop`, branch `yh/mocap_bar_reaching_acc_test`. Committed on 2026-10-01 on top of
  Su's `a602bb7` (not pushed): data layer → robot interface → monitor → dry-run tooling → docs
  (`git log --oneline a602bb7..`). Only uncommitted change: the dry-run flag values below.
- New modules: `robot_registry.py`, `schedule_io.py`, `progress_io.py`, `schedule_ui.py`.
  Heavily edited: `husky_monitor.py`, `husky_world.py`, `husky_robot.py`, `cfab_session.py`,
  `bar_action_io.py`, `common.py`, `utils.py`, `ui_backend.py`.
- New support files: `config/cyclonedds_localhost.xml`, `config/dryrun_env.sh`,
  `scripts/smoke_single_arm_plan.py`, `scripts/headless_schedule_smoke.py`, tests in `test/`.
- ! `husky_monitor.py` class flags are currently set for the dry run: `FAKE_HARDWARE = 1`,
  `USE_MOCAP = 0`. Put back `FAKE_HARDWARE = 0`, `USE_MOCAP = 1` before any hardware run.
- Automated checks last green before the GUI session (2026-09-30 evening): 96 pytest passed /
  4 skipped, `headless_schedule_smoke.py` 34/34, `smoke_single_arm_plan.py` 7/7, legacy harness
  identical to baseline.
- Design repo (the Rhino repo lives in Dropbox, synced with the Windows PC where it is developed;
  ignore any copy under `~/Code/`): `/home/yijiangh/Insync/yijiang94817@gmail.com/Dropbox/0_Projects/2025_husky_assembly/Code/bar_joint_rhino_design_workflow/docs/support_export_issues_from_monitor.md`
  (exporter defects D1–D4, for a separate session there).

## 2. User decisions (do not re-litigate)

One robot ROS-connected per monitor run (by `ROS_DOMAIN_ID`, Zenoh later) · unit of work = one
ActionSchedule entry · progress in `<problem>/progress.json` · Mark done + Reopen only (no redo) ·
grip handoff triggered by GripperCommand feedback fraction · compas_fab FK bug worked around only
inside `cfab_session.plan_linear_motion` · keep the exported H scenes (release-time geometry) ·
R_M0 untighten is mark-only · Cindy's M2/M3 compliance stays role-keyed · test manuals contain only
monitor/visual steps (Claude runs pytest/smoke scripts itself).

## 3. Environment learned this session

- **Dry run without the robot network**: `~/.bashrc` pins CycloneDDS to the USB-ethernet adapter
  `enx34298f73396f` (`~/.cyclonedds.xml`); without it the node fails (`rmw handle is invalid`).
  Per terminal: `source ~/Code/ros2_ws/src/husky-assembly-teleop/config/dryrun_env.sh 86` (84 =
  Alice) — loopback-only DDS, scratch problem `~/husky_dryrun/design/260920_dryrun`, calibration
  from Insync, `HUSKY_IK_BACKEND=gradient` (ssik is NOT installed in the venv).
- **Shared terminal**: the user runs `tmux new -s husky`; Claude drives it with
  `tmux send-keys -t husky ...` / `tmux capture-pane -t husky -p`. Mouse scroll/select enabled in
  `~/.tmux.conf`. Pane log: `tmux pipe-pane -t husky -o 'cat >> ~/husky_dryrun/monitor_pane.log'`
  (re-run after a new tmux session). Claude's own sandboxed shell cannot open OpenGL windows; GUI
  things must run inside the user's tmux session.
- **NVIDIA**: an unattended upgrade (2026-10-01 06:32, 580.159 → 580.178) broke every OpenGL window
  until a reboot ("Driver/library version mismatch"). If windows fail again after an update: reboot.
- The reboot wipes Claude's scratchpad (`/tmp/claude-1000/...`); keep useful probes in
  `~/husky_dryrun/probe/`.
- Window capture for checking the GUI: `xwd -id <window id>` + numpy/PIL conversion (32-bit
  ZPixmap); window ids via `xwininfo -root -tree`. `xwd -root` fails (BadColor).
- Piping the monitor (`| tee`) appeared to stall its printed output mid start-up; run it unpiped
  and log via tmux pipe-pane. (Cause not confirmed — possibly just output buffering.)

## 4. Where Part A (dry run) stands

- Monitor (restarted after the ghost fix, running in tmux `husky` pane 0) starts as Cindy over loopback: schedule panel + roster, header
  `robot Cindy (domain 86) | problem 260920_dryrun | progress 0/48 done`, ROS node listening with
  no robot publishing. The user stopped it with Ctrl-C (pane 0 of tmux `husky`).
- The user did: Load entry 0 → Movement 3 → `M1: Confirm manual start pose (IK check)`.
  Correct entry-0 order (fixed in both manuals): movement 3 + M1 Confirm FIRST (gives J_M0 its
  goal), then movement 0 Plan/Exec.

## 5. Open issues (next steps)

**Next work = `tasks/2026-10-01_dryrun_fixes_plan.md`** (F1–F8, from the user's worklog of the dry run; not
implemented yet). Rhino-side problems D5–D7 were added to the Dropbox repo's
`docs/support_export_issues_from_monitor.md` for a Windows-side session.

1. ~~Goal ghost / red planning robot not visible~~ — **FIXED 2026-10-01.** Root cause: the PyBullet
   window keeps all mesh data in one fixed-size buffer; the two extra viz-only huskies (Alice, Belle)
   pushed it over PyBullet's default, and bodies loaded later (the goal ghost, the compas_fab
   planning robot) were silently not drawn (pybullet_planning's `HideOutput()` hides any message).
   Fix: `HuskyMonitor.start_pybullet` requests `--max_shape_capacity_in_bytes` =
   `PYBULLET_GUI_MESH_BUFFER_MB` (512) and `--max_num_object_capacity` = 256k (scoped wrapper around
   `pybullet.connect`). Verified with GPU renders (`~/husky_dryrun/probe/gui_capacity_probe.py`,
   variants: current / no_extra / only_alice_urdf / cap512 ...). A 1024 MB request did not take
   effect. Note: `getVisualShapeData` returns [] for every body in a GUI client — useless as a check.
2. **Planning failures to debug together** (user's request): with default M1 sliders on entry 0,
   J_M0 `plan_free_dual_arm` → `birrt_failed` (59 s); J_M3 transfer found no path in 4 × 2-min
   attempts. Gradient IK backend (no ssik) may matter. Exported hold scenes collide
   (`ObstacleRobotCindy <-> env_bar_B5`) unless `BAR_ACTION_MOCAP_ACCURACY_TEST=1` (user decision).
3. `Move Arms to Movement Start (offline target)` has no FAKE_HARDWARE branch — it publishes real
   trajectories (harmless on loopback; open question to the user whether to add a sim branch).
4. Hardware prerequisites (Part B): Alice has no base calibration in `CALIBRATION_DATE=20260916`
   (newest: `20260623/calibrated_transformation_0804_rhino.json`); Belle never calibrated. Robotiq
   feedback during the stroke, single-arm TCP frame, compliant hold drift limit — hardware-only
   checks listed in the test manual (B3).
5. The `traj time` slider showed 90 s right after start-up (expected to switch to the per-movement
   default on Load Movement — worth a glance).
