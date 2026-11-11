# Insertion demo plugin (Swiss Robotics Day), robot-centric, no mocap

Status (2026-11-04): implemented on `yh/insertion-demo` (off `origin/core-port` 68b042e3): the plugin, the parts,
`husky_fixed_base`, `scripts/insertion_demo/`, `doc/lab/insertion_demo_manual.md`, tests under
`test/plugins/insertion_demo/` and `test/mocap_relay/test_fixed_base.py`. The plugin's panel is `insertion_demo`'s
tab (not in the Assembly tab); the process plugin still waits for Load (`process_directory` only fills the box).
Open: the lab check with the cable (section 5 of the manual).

Dry run (2026-11-09): `scripts/insertion_demo/sim_tmux.sh` opens the demo against a simulated Cindy on the laptop,
`doc/lab/insertion_demo_sim_manual.md` is its step-by-step manual. `DEMO_SIM=1` switches `env.sh` to the fake lab's
settings (domain 71, a loopback router, the scratch Drive `sim_drive.sh` fills); `base.sh` and `monitor.sh` are the
hall's. Run end to end from the buttons, the interruptions included. Found there: after Stop all, Resume comes
before Rigid (Rigid alone does not last while the robot is soft stopped); a cancelled insert is undone by Rigid, then
Retreat.

Preview before each move (2026-11-09, asked after the first dry run): **Approach**, **Insert** and **Retreat** only
plan and show the move, as the Assembly tab's Plan does: a see-through robot on the path at **Preview time** (scrub
or **Play**), timed for **Motion time** (the old **Seconds**; the line below says the arms' minimum). **Execute**
runs the move shown, once, from the live joints (`live_rows`: refused when the arms left the plan's start).
**Cancel** with nothing running drops the plan. The retreat has no planner: its preview is the straight back-out
from the live joints to the insert's start (`parts.back_out_rows`, `parts.timed_preview`).

Plots (2026-11-09, asked next): the Assembly tab's two plot windows in the demo, opening by themselves: **Plan
plots** when a move is planned, **Exec plots** (force and torque, from the part's `Recorder`) when an insert, a
retreat or Compliant starts. `PlanPlots` and `ExecPlots` moved from `plugins/assembly_steps` and
`plugins/movement_exec` to `shared/assembly_tab/`, since plugins never import each other.

Cable check (2026-11-09): Cindy answers at 192.168.131.1 (ssh 22 and her router 7447 open, the arms at .40/.41); her
bridge has no DHCP, so the laptop's port needs its fixed address (manual 1.3). tmux.sh empties `DEMO_SIM` in its
panes: a tmux server started by sim_tmux.sh handed it on, and the hall panes took themselves for the dry run.

First run on the cable (2026-11-09): the laptop's nodes as Zenoh peers on her router got nothing of hers (her launch
warned "Unable to connect to any locator of scouted peer … [tcp/127.0.0.1:…]"): her router relays between routers and
for clients, not between two peers on it, and her nodes listen on her loopback only. env.sh now sets `mode="client"`.
Her router is joined to the lab's over WiFi, so base.sh refuses while anything else publishes her body's pose.
`husky_fixed_base` no longer raises on Ctrl-C (its `_publishers` shadowed rclpy's own list).

Autonomous cycle (2026-11-10, asked next): a fourth Move button **Return** plans the straight way back to the
insert's start with the insert's own planner (`Route._insert` on the insert step with `goal_poses` = the tool0
poses at the authored start joints: `tamp.plan_constrained_dual_arm_linear`, the bar held; it steers the left goal
only, the right follows the grasp measured at the live start), checked by `parts.return_problem` (the relative tool0
pose within `COUPLING` = 2 mm / 1° of row 0 along the path, no joint step over `JOINT_STEP` = 5°) and the route's
`end_error` per flange within `AT_START`. Under a new **autonomous** section, **Settle s** (5) and **Cycle**: one
click runs insert (must end `STALLED`), the pause, the retreat (must end `RETREATED`), then Return planned, checked
and run; `self.step` reads `cycle: insert` … `cycle: return`; before the retreat and the return the cycle waits until
no arm counts as moving (`moving.arm_moving`, at least `REST_AT_LEAST` 1 s: the dry run's return was refused 0.3 s
after the switch back, "the arm is still moving"); the first phase that does not end well stops the cycle
(`cycle stopped at …`), the arms left as that part leaves them. A cancelled cycle never resumes: Rigid, Retreat and
Return by hand, then Cycle. The manual controls are unchanged (`_approach` became `_move`, run for both the approach
and the return). Tests: `test/plugins/insertion_demo/test_demo_cycle.py` (a coupled test path solved on the URDF
by the fake arms' least-squares step; the whole cycle on the fake Cindy; its stops; Cancel during the pause).
2026-11-11, first hardware cycle: the return was refused, "right_ur_arm's tool0 ends 0.6 mm, 1.1° from the
insert's start": the grasp twisted 1.1° over the compliant insert and retreat, and since the planner steers the left
arm only, all of that lands on the right arm's end. `AT_START`'s angle went from 1° to 3° (the distance stays 5 mm);
it gates both the return's end and the next cycle's start. If the angle grows cycle after cycle, the bar creeps in a
gripper. Same day: **Tighten** / **Loosen** under **Joint** run both joint screws by hand (`parts.run_screws`,
which `run_grip` now wraps): tightening until the stall (`HOLD_FOR_STALL`), loosening for **Loosen s**; Cancel stops
them, every motor stopped in the `finally`; **Stop** sends STOP to both tools at once on the main thread, whatever runs
them, and cancels a Tighten or Loosen under way. **Zero F/T** (Arms group) zeroes both sensors where the arms stand
(`zero_force_sensors`, leased, meant for no bar in the tools: the retreat zeroes with the bar still in its joint, so
the compliance later sags under the bar's weight once it is out). **Stow** (Move group) plans the transfer step with
`goal_joints` = `world.kinematics.stow_joints` (`Route.ik_goal` returns the override; `tamp.plan_free_dual_arm`, the
held bar hidden) and shows it; Execute runs it as a joint motion.

Lengths, preview and Save (2026-11-11, plan approved the same day):
- **Settle s** starts at 0.
- A Return failing its checks is shown, never run: `Planned.refused` (the reason), the chip `return refused`,
  Execute refuses, the cycle's `planned_move` stops and keeps it on screen.
- **Insert mm** / **Retreat mm** (5 to 150, starting at the process's own length): `parts.line_start` solves the
  joints that far before the insert's end on its line, from the authored start in 5 mm tool0 steps
  (`robot_interface.tool_step.tool_step_target`), no planner, no collision check. `plugin._line(robot, demo, slider)`
  replaces the authored start everywhere it was read (Insert's and Cycle's "at the start", Return's goal, the
  retreat's end); Approach passes those joints as `goal_joints`. The insert still plans to the authored end.
- **Preview cycle** plans approach (if off the start), insert, back-out (two joint rows), return, each from where the
  one before ends (`_plan(..., start_joints=)` on a copy of the snapshot), and shows them as one `Planned` with
  `step=PREVIEW`, `refused=PREVIEW_ONLY`, `parts=n` (the preview takes n times Motion time).
- **Save** (second press): `parts.with_insert_start` edits the insert's start joints and line distances and the
  transfer's target (joints and link poses, so the chain and the target checks hold), `parts.save_action` writes
  that one `actions/<action>.json` with the core's own serialiser (private imports `_action`, `_write_json`,
  `_writer`: the core has no public one-action writer), swapped in by rename, the file's CRLF kept. Then Load.
  ! It edits the real Drive, only on the operator's two presses. Retreat mm is not saved.
- Dry run 2026-11-11 (scratch Drive): Preview cycle at 60 / 40, Approach, two Cycles (`rigid 50.0 mm, then COMPLIANT
  10.0 mm`, back-out 40 mm, return 6 waypoints), Save changed 29 lines of `B6_J.json` only, Load then showed 60 / 60.

## Context

For the Swiss Robotics Day, Cindy (0806, dual arm, AT3L/AT3R scaffolding tools) shows one diagonal bar (B6 of
`data_design_study/261104_SRD_insertion`) being inserted into a jig and pulled out again, over and over, with a bar
rack and the jig on her own base. There is no mocap, no WiFi router: the laptop hangs on an Ethernet cable. The
operator mounts the bar once, then clicks: **Approach**, **Insert** (compliant, screws tighten, as today), **Retreat**
(new: the joint screws run in reverse, then the arms back out along the insert line with the bar still in the tools),
and a **Compliant/Rigid** switch for both arms to let visitors push the robot. Every motion is planned from the live
joints, since the compliant phases and the pushing move the arms off the authored confs.

The existing assembly/process plugins stay untouched: this is a new standalone plugin, `insertion_demo`, built from
the shared building blocks (`shared/execution`, `shared/assembly_planning`, the `motion_planner` and `process`
plugins through their contracts).

Decisions made with the user:
- Mounting pose: none authored (`B6_J_load` has no target in this export). **Approach** plans a free dual-arm move
  (the transfer workaround, bar hidden) from wherever the arms are to the insert's start.
- Loosening does not stall: it runs for a time set by a slider (default 6 s), then the arms back out.
- Cable: laptop on Cindy's wired bridge `br0` (192.168.131.x); nothing changes onboard.
- Base: build on `origin/core-port` (local `core-port` is 43 commits behind; the execution layer was reworked there:
  `compliant_part.py`, `force_sensors.py`, `monitor.launch.py`).

## The process (read from the folder)

`process.json` schedule `B1_P B2_P B3_P B4_J B4_R B6_J B6_R`; Cindy's base authored at the origin. `B6_J`:
`B6_J_load` (free, **no target**) → `B6_J_place_bar` (operator) → `B6_J_grasp` (tools closed) → `B6_J_transfer`
(free, coupled, position; start joints null, target = the approach conf + link poses) → `B6_J_insert` (linear 15 mm
along `(0.972, 0, -0.233)`, coupled, compliant, drives both `tighten`, ends on tools). B1–B4 built and present; both
mates (J3-6, J4-6) are on built bars, so both arms mate (no one-mate case, spiral allowed but not used).
`B6_R`: ungrasp → linear retreat 50 mm → home. Grasp in the start state: `attached: bars/B6 -> [left tool0 grasp,
right tool0 grasp]`.

## Design

### 1. Setup: branch, workspace

1. `git merge --ff-only origin/core-port` on the clean local `core-port`; `git switch -c yh/insertion-demo`;
   `git submodule update --init --recursive`; pull crl-husky (`jg/ros2`) and rebuild (`python3 -m colcon build
   --symlink-install --packages-select husky_assembly_teleop` in the venv; see AGENTS.md).
2. Copy this plan to `tasks/2026-11-04_insertion_demo.md` (AGENTS.md "Task notes"); keep it updated.

### 2. Base at the origin without mocap: `husky_fixed_base`

The monitor's only base source is the relay topic (`robot_interface/base.py`: "Mocap is the only source of the base
pose"), and every planner and compliant part refuses without a live pose (`precheck.part_problem`: "no base pose from
mocap"; `stream` stops on `pose_lost`; `motion_planner/work.py` "its base is not tracked"). The relay's `sim:=true`
is refused in domain 80. So: a small node publishing the base where the demo assumes it.

- New `husky_assembly_teleop/mocap_relay/fixed_base.py` + console script `husky_fixed_base` in `setup.py`
  (next to `husky_sim_motive`). Parameters: `bodies` (ints, default `[1860]`), `pose` (`"x y z yaw_deg"`, default
  origin), `rate` (default 20 Hz). Publishes `crl_husky_msgs/MocapRigidBodyPose` on
  `mocap.mocap_topic(id)` = `/mocap/rigid_body/id_<id>/pose` with `header.stamp = now` (this laptop's clock, the
  monitor's too), `pose_valid = tracking_valid = True`, `marker_error = 0`, no markers; QoS as the relay's publisher
  (`mocap_relay/node.py`). Logs loudly at start: the pose is fixed, never drive the base from it.
- Health's mocap chip goes green (no relay calibration/frame → at most amber; cosmetic). Clearances see a known base.
- ! The real relay must not run at the same time (the demo manual says so).

### 3. The plugin `husky_assembly_teleop/plugins/insertion_demo/`

`__init__.py` (imports `plugin`), `plugin.py` (panel, buttons, the tasks), `parts.py` (the new compliant
removal part and the pure helpers, testable without the panel). `requires = ("process", "motion_planner")`;
`name = "insertion_demo"`, `tab_path = ("Insertion demo",)`, an `icon`. Robot: the first configured robot with two
scaffolding tools (Cindy); refuse everything with a chip if none.

**Panel** (AGENTS "nothing jumps": fixed widgets, chips change colour, text below the buttons):
- status `chip_line`: robot live / not; process action (e.g. `B6_J`) / none loaded; tools reporting; arms
  `joint` / `compliant`; the running step; last outcome.
- dropdown **Action**: the `bar_jointing` actions of this robot whose insert has two tighten drives; default the last
  in the schedule (`B6_J`). Loaded from `process.selection.process` (`ProcessContract`); until the process plugin has
  loaded (Load in the Assembly tab, or `process_directory` at startup — check on origin whether it auto-loads),
  every button refuses with "load the process first".
- sliders: **Compliant mm** (insert split, default 10 of the 15 mm; `SPLIT_RANGE` as movement_exec), **Seconds**
  (the insert path and the back-out line, default 10 s), **Loosen s** (default 6, 1–20).
- button groups: `Bar` [Grasp, Release] · `Move` [Approach, Insert, Retreat, Cancel] · `Arms` [Compliant, Rigid].
- `add_html` below the buttons for refusals and outcomes (comes and goes).

**Tasks**: one at a time (`ctx.spawn`, Cancel cancels it; a click that does not apply is logged and ignored, as
`examples/sequence.py`). Each holds `ctx.lease_robots([serial], label)` for its whole run. Before any step, select
the step's movement in the process plugin (`process.request_select(...)`) so the jig and built bars are in the live
scene and drawn. Steps and movements are found with `loaded_process(selection.process)` and `index_of(action_id,
movement_index)`; the insert step = the action's movement classified `Routine.INSERT` (`assembly_planning.movement.
classify`), the transfer = the free coupled movement before it.

1. **Grasp / Release**: `stop_and_clear(ctx, tools, GRIPPER_MOTOR)` → `tool.close()` / `tool.open()` on both →
   `StallGate(ctx, tools, GRIPPER_MOTOR)` until `all_stalled()` or `GRIP_TIMEOUT` (15 s) → `stop_all(tools)` in
   `finally` (`shared/execution/tools.py`).
2. **Approach** (transfer workaround, from the live joints): `motion_planner.request_plan(MotionPlanRequest(loaded,
   transfer_step, ctx.scene.snapshot, ctx.config.robots, transfer_as_free=True, obstacles_collide=
   process.obstacles_collide, label="demo approach"))` → `RouteResult` (refusal text if not ok) → rows with row 0
   replaced by the live joints (as `movement_exec/sequences.live_path_of`, plugin-private: rewrite the 6 lines with
   the `STALE_ARM` check in `parts.py`) → `motion_command(serial, list(robot.arms), names, rows, seconds)` →
   `send_motion(ctx, robot, command)` (`shared/execution/send.py`).
3. **Insert**: refuse unless every tool0 is within `AT_START = (5 mm, 3°)` of the insert's authored start (FK of
   `insert_step.movement.start` joints vs the live joints, `world/tcp.tool0_in_base_link`; text: "press Approach
   first"). Then `request_plan` on the insert step (no `goal_poses`: `_insert` plans the coupled line from the live
   joints to the authored targets, keeping the live grasp) → live rows → mating arms and tools from `part_plan(...)`
   (`shared/movement_exec_contract.py`, as `sequences.py:470–484`) → `plan_insert(robot, names, rows,
   mating=part.tools, grounded=False, split_mm, spiral=None, traj_time)` → `run_insert(ctx, robot, plan, tools,
   operator=<never asked: no spiral>, dry_run=False)`. Ends STALLED with the arms back under joint control.
4. **Retreat** (new, `parts.run_unscrew_retreat`, mirrors `retreat.run_retreat` without the gripper opening and
   without a tail, built from `compliant_part.py`):
   ```
   problem = check_part(ctx, robot, arms, tools, record)          -> REFUSED
   ends = FK of the insert's authored start joints at the live base (arm_base_in_world + tool0_in_base_link)
   started, recorder = started_part(...); phase = phase_keeper(...)
   try:
       stop_and_clear(ctx, tools, JOINT_MOTOR)                     -> REFUSED on timeout
       record["ft_zero"] = await zero_force_sensors(ctx, arms, label)   # bar still in its joints
       failed = await enter_compliance(ctx, arms, record, phase=phase)  -> FAILED
       tool.drive_screw(-1) on both (MotionRefused -> FAILED)      # loosen, under compliance
       await ctx.sleep(loosen_s)                                   # the slider; no stall expected
       stop_all(tools)                                             # motors off before the arms move
       segment = Segment(start={measured_tool0}, end=ends, seconds, wait=0.0)
       ended = await stream(ctx, robot, segment, keep_going=<line time, then until every tool0 within ARRIVAL>,
                            recorder=recorder)                     # as retreat._break_out
       record["end_error_mm"] = progress(robot, segment); log travelled/short
       status = RETREATED if ended in ("ended", "time") else FAILED
   finally:
       await stop_then_leave(ctx, partial(stop_all, tools), arms, label=label, record=record, cancelled=cancelled)
   ```
   Returns a `PartOutcome` (`RETREATED`, `REFUSED`, `FAILED`); the arms end at the approach poses under joint
   control, bar in hand, so **Insert** can run again at once.
5. **Compliant / Rigid**: a task that keeps the lease while the arms are compliant: `zero_force_sensors` →
   `enter_compliance(ctx, arms, record)` (switch + zero target wrench; on failure `leave_compliance`) →
   `ctx.wait_until(rigid pressed)` (no timeout) → `leave_compliance(ctx, arms, label)` (hold, switch back, wait).
   Cancel / Stop all → `stop_then_leave(..., cancelled=True)` = `hold_compliant`: the arms stay compliant, said
   loudly, switched back with robot_control's Joint buttons (so `robot_control` is in the demo launch).

Records: each part's `PartOutcome.record` is logged (status, reason, peak |F|); no run file (nothing to replay).
No freeze (one robot), no runtime clearance (as movement_exec: the planner's collisions).

### 4. Network and scripts (`scripts/insertion_demo/`, `doc/lab/insertion_demo_manual.md`)

Facts (from the repo): 192.168.0.115 is Cindy's **WiFi** address (`real_lab/env.sh`, crl-husky
`config/robots/0806/robot.yaml`); her wired ports are the onboard bridge `br0`, where the UR arms sit
(192.168.131.40/.41, `crl_dual_ur5e.launch.py`). Her Zenoh router (`crl-zenoh-router.service`) listens on
`tcp/[::]:7447`, all interfaces, multicast off, dials nobody. The laptop's lab router script insists on a
192.168.0.x address. Robots take time only from 192.168.0.201–.210 (chrony), so over the cable Cindy's clock is not
synced: joint-state freshness is by arrival time, the fixed base is stamped on the laptop, and trajectories carry
no monitor-clock stamp, so nothing blocks; health's signal chip may colour.

- `env.sh`: domain 80, `rmw_zenoh_cpp`, and **no laptop router**: every node dials Cindy's router directly, as the
  fake lab dials its own: `ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/${DEMO_CINDY_IP:-192.168.131.1}:7447"];
  scouting/multicast/enabled=false'`. `HUSKY_DRIVE_ROOT` as `real_lab/env.sh`; `DEMO_PROCESS=
  data_design_study/261104_SRD_insertion`. The lab's `crl-zenoh-router` user service must be stopped
  (`systemctl --user stop crl-zenoh-router`), else two routers on 7447.
- `base.sh`: `ros2 run husky_assembly_teleop husky_fixed_base` (body 1860 at the origin).
- `monitor.sh`: `lab_monitor` (the pytest-import workaround from `scripts/lab_common.sh`) with
  `robots:="cindy" plugins:="insertion_demo robot_control" process_directory:=$DEMO_PROCESS` — no `obstacles`
  (the ETH lab's furniture is not in the hall).
- `tmux.sh`: panes base / monitor / a checks shell (reuse `scripts/lab_tmux.sh` helpers).
- Manual (short, like `real_lab_short_manual.md`): laptop dongle static IP 192.168.131.100/24, plugged into Cindy's
  onboard switch; read her `br0` address onboard (`ip -4 addr show br0`), ssh to it; onboard launch as in the lab
  (`crl_arms_and_base.launch.py gripper:=scaffolding_v3`, pendant programs); then base.sh, monitor.sh, Load, the
  demo cycle; what to do when arms are left compliant.

## Files

- new `husky_assembly_teleop/plugins/insertion_demo/{__init__,plugin,parts}.py`
- new `husky_assembly_teleop/mocap_relay/fixed_base.py`; `setup.py` console script `husky_fixed_base`
- new `scripts/insertion_demo/{env.sh,base.sh,monitor.sh,tmux.sh}`; new `doc/lab/insertion_demo_manual.md`;
  one line in `doc/lab/plugins.md` and the README plugin list
- new `test/plugins/insertion_demo/`, `test/mocap_relay/test_fixed_base.py`
- `tasks/2026-11-04_insertion_demo.md` (this plan)
- untouched: `plugins/process`, `plugins/assembly_*`, `plugins/movement_exec`, `shared/execution/*`

Reused (origin/core-port): `shared/execution/compliant_part.py` (`check_part`, `enter_compliance`,
`stop_then_leave`, `started_part`, `phase_keeper`), `controllers.py` (`leave_compliance`, `hold_compliant`),
`force_sensors.zero_force_sensors`, `stream.py` (`Segment`, `stream`, `measured_tool0`, `arm_base_in_world`,
`progress`, `ARRIVAL`), `tools.py` (`stop_and_clear`, `StallGate`, `stop_all`), `insert.py` (`plan_insert`,
`run_insert`), `send.py` (`motion_command`, `send_motion`), `basis.STALE_ARM`,
`shared/movement_exec_contract.part_plan`, `shared/motion_planner_contract.MotionPlanRequest`,
`shared/process_contract.ProcessContract`, `shared/assembly_planning/process_steps.loaded_process`,
`world/tcp.tool0_in_base_link`, `ui.style` chips, `examples/sequence.py` as the task pattern.

## Tests (`pytest -m insertion_demo`, fast; mark the robot-commanding ones `safety`)

Copy the `Lab` fixture of `test/shared/test_execution_insert.py` (fake Cindy with real frames, `MocapDriver`,
`ArmDriver`, `ScaffoldingDriver`, `FakeContext`):
- `test_unscrew_retreat.py`: both tools get `(JOINT_MOTOR, -1)` then `(0, 0)` after `loosen_s`; the switch to
  compliance comes before the loosen and the stream after the stop; the streamed targets end at the approach poses;
  every arm is back under joint control at the end; a cancel mid-stream stops the motors and only holds (arms left
  compliant); refused (nothing sent) when an arm is not under joint control or a tool is silent.
- `test_demo_steps.py`: Insert refuses when the tool0s are off the authored start; Grasp sends `(GRIPPER_MOTOR, 1)`
  and stops at the stall / timeout; Compliant → Rigid switches both arms there and back, lease held in between.
- `test_fixed_base.py`: the message fields and the topic name for a body and pose.
- The layer test (`test/structure/test_layers.py`) and linters run in the default set.

## Verification

1. `python3 -m pytest -n 0 test/plugins/insertion_demo test/mocap_relay` while iterating; `pytest -m safety`;
   `pytest` before the commit (AGENTS.md).
2. Fake lab end to end (domain 70, `scripts/fake_lab`): copy the SRD process into the fake lab's scratch Drive,
   start sim Cindy + fake Motive + relay, the monitor with `plugins:="insertion_demo robot_control"`; run Grasp →
   Approach → Insert → Retreat → Compliant → Rigid → Approach → Insert; watch the screw commands and controller
   switches in the console. (The sim's compliant parts need `src/cartesian_controllers` built.)
3. Lab, with the cable: laptop on `br0`, `base.sh`, `monitor.sh`; first a jog in robot_control (does the trajectory
   run with the unsynced clock?), then the cycle on the real jig. Check: Cindy's `br0` address and a free port on her
   switch; the loosen time (6 s enough to free the bar?); the tighten stall still ends the insert.

## Open points to confirm in the lab (not blocking the code)

- The bar rack on the base is not in the process: Approach plans blind to it; keep the rack out of the arms' way or
  add it as a temporary box in the obstacles plugin (then load `obstacles` with an empty lab file).
- The jig must sit where the process puts it relative to Cindy's base (the process is authored with her at the
  origin).
- `process_directory` at startup: on origin, whether the process plugin loads it or still waits for Load.
