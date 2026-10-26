# Fake lab end to end: Jakob's simulated tools in, the manual rewritten for the first three bars (2026-10-26)

Branch `yh/process_replan_execution` (worktree `src/husky-assembly-teleop`), off `yh/assembly-tab` 9f36444.

## Goal
1. After the M6 (`yh/m6-compliance`, 2b8aa26 + M6b d424ab4, done) and M7 (`yh/m7-servo`, in progress in `dev/m7`) sessions finish,
   merge their branches here, then Jakob's `origin/core-port` (13 commits: tools from crl-husky's `model.yaml`, a
   `SIM` chip per simulated tool, the console plugin, newest-only state decoding, collision primitives, test groups).
2. Move `src/crl-husky` from 934fc9a to `origin/jg/ros2` (03056b7: the simulated scaffolding tool `scaffolding_tool_sim`,
   simulated Robotiq grippers, tools from `model.yaml`; `SimulatedPart.msg` changed, so crl_husky_msgs and the monitor
   must be rebuilt together). Rebuild the workspace.
3. Rewrite `doc/fake_lab_test_manual.md`: no workarounds, no faked refusals. One walk through the first three bars,
   B20 (ground, Cindy), B21 (Cindy joints it, Alice holds it: B21_H), B22, with their retreats: schedule entries 1–7.
   Every step as the process has it: load, place the bar by hand, grasp (the simulated screw stalls after 2 s),
   the held transfer, the servo (by tool0), the insert, the retreat, Alice's approach, open, to grasp, close (handoff).

## Preview of the merge (trial merge of origin/core-port into 2b8aa26, 2026-10-26)
Three conflicts, all small: `plugins/process/plugin.py` (keep our `tab_path`/`icon`, drop `experimental = True` as
Jakob did), `README.md` (our Assembly-tab paragraphs, minus "Experimental." on process, base_planner_panel,
arm_planner_panel, collisions, base_exp as Jakob did), `doc/fake_lab_test_manual.md` (rewritten anyway).

## What the simulation still lacks, and what the manual does about it
- **The Cartesian compliance controller** (`cartesian_compliance_controller`, fzi `cartesian_controllers`) is not in
  this workspace: the simulated arms' spawner dies on it (the manual's four `process has died` lines). M6's insert
  (compliant part), retreat (break-out) and handoff (compliant close) switch to it, so they cannot run in the fake lab
  until it is built. Cloned to `src/cartesian_controllers` (branch `ros2`, f8aad77; simulation/tests/handles/utilities
  under COLCON_IGNORE). ! Building freshly cloned code was refused by this session's permissions: the user builds it
  (`python3 -m colcon build --symlink-install --packages-select cartesian_controller_base cartesian_motion_controller
  cartesian_force_controller cartesian_compliance_controller`). Risk: the UR fake hardware's `tcp_fts_sensor` values
  under mock_components may be NaN rather than 0; check the first compliant part's log.
- **No bar markers in the fake Motive**: the M7 bar fit has nothing to fit. The manual runs with "Servo by bar" off
  (the tool0 servo). Measured flanges come from the relay's base pose and the arms' joints, so it converges in a pass.
- **No forces**: the F/T sensor reads zero. The retreat's force checks pass trivially; the handoff's "gives way" is
  invisible.
- **The transfer workaround** stays off in the manual: the process's own load, place, grasp and held transfer run.
- **Rhino obstacles collide** stays unticked (the default): Rhino's cabinet is 0.38 m off the measured one.

## Steps
1. Wait for M7's commit (subscribed to the m6 and m7 sessions' idle notices).
2. `git merge yh/m7-servo` (it contains M6), then `git merge origin/core-port`; resolve as previewed. The two task
   notes modified in this worktree at session start are an older state of what M6 committed: keep the merged versions.
3. crl-husky: `git checkout origin/jg/ros2` (detached, as before); README pin 78041cf → 03056b7.
4. Build (monitor + crl-husky + msgs), run `pytest -m ""`.
5. Run the fake lab myself (own tmux session, FAKE_LAB_DOMAIN=72, PORT=8097, HOME=scratch) through B20_J … B22_R;
   note every prompt and log line for the manual; fix what blocks (surface anything outside this change's scope).
6. Rewrite the manual from the run. Commit, one grouped commit.

## Open for the user
- `sudo apt install ros-humble-robotiq-description ros-humble-robotiq-controllers`: the simulated Robotiq (manual 0.1).
- Build `src/cartesian_controllers` (manual 0.2): the compliant insert, the retreat's break-out and the handoff
  cannot run in the simulation without it. Then run manual 4.4, 5.2 and 5.3 step 5, the parts nobody has run yet.
- The transfer workaround stays on: this export has no loading pose (Rhino side).
- The T20 ground foot sits 15.6 mm into the floor at its design pose (Rhino side); the planner now lets ground-mated
  halves touch the lab floor.
- Mark done accepts an action with steps left: a guard that asks first is your call.
- The compas_fab stdout race (`monitor.sh` imports pytest) still needs a real fix: your decision, as before.
- The browser driver I used (`cdp.py`: open, clicktext, clickbtn, clickbox, select, setnum, runstep) lives in this
  session's scratchpad, not in the repo; say if you want it under `scripts/fake_lab/`.
- Sections dropped from the old manual: the Rhino box on a base plan, the link drop (decision F5), the phone page;
  `git show 9f36444:doc/fake_lab_test_manual.md` has them.

## Run log (2026-10-26 evening, my own fake lab on domain 72)
- Merged: `yh/m7-servo` 10c49ea + `yh/m6-compliance` d424ab4 → e40147b; `origin/core-port` b1b7122 → 76295ca
  (branch `yh/process_replan_execution`). 1506 tests pass. crl-husky at 03056b7, rebuilt; the monitor rebuilt.
- **The simulated Robotiq needs apt packages** `ros-humble-robotiq-description ros-humble-robotiq-controllers`:
  without them `sim.launch.py robot:=all` dies at Alice ("package 'robotiq_description' not found"). For my run I
  launched the robots one by one (`sim.sh robot:=0806`, `sim.sh robot:=0804 tool:=none fake_motive:=false`, same
  for 0805). The user installs them (sudo).
- **The travel to load cannot be planned from this export**: `B20_J_load`'s goal is the transfer's start, and this
  export has no joints there (the transfer's start and the load's target carry none). So the transfer workaround
  stays ON: it is how the process runs today, not a test fake. The manual says so.
- **Jakob's floor slab refused every ground bar's goal**: the T20 ground foot's mesh reaches 15.6 mm below z = 0 at
  its design pose ("all 64 solutions collide … G20-T20-0_ground with obstacles/floor"). Fixed in our planning scene
  (`scene_for.py`): a half the process mates to a ground body may touch the lab floor (`ground_mated_halves`), as the
  core lets it touch the process's own `ground/` bodies. Test in `test_planning_scene_for.py`. For Jakob or the
  Rhino side: the foot sits 15.6 mm into the floor by design.
- B20_J ran end to end with the workaround: transfer (17 s), place by hand, grasp (the simulated screws stall after
  2 s: "every gripper screw STALLED: the bar is clamped"), servo on tool0 ("nothing left to plan", 0.06/0.08 mm);
  insert (RIGID, ground bar, 5 s), servo after it ("nothing left", 0.15/0.19 mm). Mark done.
- B20_R's retreat FAILED at the switch to `cartesian_compliance_controller` ("controller manager rejected the
  switch"): the controllers are not built here. With Insert split 0 mm the retreat has no compliant part.
- B20_R with Insert split 0: screws OPEN, the tail replanned (52 rows, 5 s), RETREATED "2 check(s) FAILED": the
  contact check (no force reading) and the grippers check (the simulated screws LOOSEN until STOP, never STALLED =
  fully open). Both are the simulation's; the manual says to expect them. Mark done works.
- Mark done accepted B20_R after its first step, with the home step not run; the next base plan then refused
  ("start: right_ur_arm tool ↔ bars/B20, 36 mm apart"): Cindy's arms were still at the bar. Reopen, mv ▶ to the home
  step, run it, Mark done. The manual says to watch for "every step … done" before Mark done. (A guard that asks
  before marking an action with steps left: the user's call.)
- B21_J ran: transfer, place, grasp, servo; the RIGID-ONLY insert ends when the simulated left joint screw stalls,
  2 s after its TIGHTEN, so the 5 s path is held 40 % in ("STALLED during the motion"). The simulated tool's
  `tighten_s` (2 s) is a parameter: a longer one lets the path finish and stall in the hold.
- Alice's arm never came up: her spawners died at start ("Failed loading controller joint_state_broadcaster",
  controller-manager-timeout 10 s) while three robots launched at once on this laptop, so the monitor had no joints
  for her ("a200-0804 can't be planned for: joints not known"). Restarting her sim fixed it. The manual's health
  check (every arm chip green) catches this; the fix is to restart that robot's sim.
