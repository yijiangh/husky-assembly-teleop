# Assembly tab: planning printout and diagnosis, advanced settings, ghosts, old-monitor Confirms, plots that come forward

## Context

Testing the fake lab (doc/fake_lab_test_manual.md) on 2026-10-27, the Assembly tab turned out blind and mute next to the
old monitor, and asked for too many clicks:

- **Printout.** `plan B21_J_transfer` ran 3 × 120 s with nothing in the console or log: `RouteSearch.free`
  (`shared/assembly_planning/route_search.py:140-166`) never calls `self.log` during the search, tamp runs with
  `debug=False`, and the few `print`s (pybullet_planning's "initial/end configuration is in collision", tamp's
  `[ssik]` lines) go to the terminal only. One `describe()` line comes at the end (`plugins/motion_planner/plugin.py:185-193`).
- **Failure reason.** `RouteSearch.free` drops tamp's `failure_reason` ("birrt_failed") and writes "no path in 120 s".
  Such a route carries no `CollisionHit`, so the Collisions tab (which draws any hit: red/cyan links, contact lines)
  has nothing to show. Start and goal collisions are pre-checked and drawn; a failed search between clear endpoints
  is not diagnosed. The old monitor's `_diagnose_free_plan_search` drew the straight path's first collision and
  printed a verdict.
- **Hyperparameters.** All constants: `FAR` (120 s, 50 iterations, 0.1 rad), `HOME` (30 s), `ATTEMPTS` 3,
  `SWEPT_STEP` 0.01 (`tamp_route.py:47-48`, `route_search.py:35`, `collision.py:32`). Nothing in the UI.
- **Ghosts.** The yellow goal ghost already shows the IK joints at the live base, but
  `step_panel_base._draw_ghosts` hides it 20 s after the last click in the panel (`RecentUse`, `ghost_timeout`); the
  Live IK answer arriving is not a click. The see-through robot that stays is the process plugin's overlay at the
  Rhino base; after Live IK its `Carry` lays the IK joints over the authored ones but keeps the Rhino base.
- **Confirms.** Old monitor: Exec sent an arm motion at once; Confirm only before the bar mount (one prompt: "mount
  the bar and clamp it with the tool buttons"), servo iteration 1 (plain) and later ones over 5° (SAFEGUARD, warn),
  Robotiq open/close, manual steps, the spiral end. Retreat and home: none. New executor: every part asks
  (`sequences.py` `confirmed(...)` in `_motion`, `_insert`, `_retreat`, `_handoff`, `_robotiq`, `_by_hand`), so
  B20_J's transfer step takes 4 Confirms instead of 2, B20_R 2 instead of 0, B21_H 5 instead of 3.
- **Plots.** The old DPG windows opened by themselves: the joint plot when a plan arrived, the F/T live plot when a
  compliant part started, the servo tracker. Now the plan plots sit in the Assembly tab's last band (`PLOTS_BAND`,
  always drawn, but off-screen below the settings) and the servo / compliant F/T plots in a top-level **Plots** tab
  (`plugins/movement_exec/plots.py` `ExecPlots`) that nothing selects. viser 1.1.1 cannot select a tab or open a
  folder from the server (client-only state); it can `expand()` / `minimize()` a floating panel
  (`PanelHandle`, precedent: the console `plugins/console.py:88-101`).

Decisions taken with the user (2026-10-27):
1. Printout: our own per-try lines in the console/log, plus tamp's and pybullet_planning's stdout captured into the
   console under source `motion_planner/tamp`. birrt stays non-verbose.
2. Diagnosis on "no path": tamp's reason kept; both endpoints' pairs; the straight joint-space path's first collision
   drawn as a hit in the Collisions tab; one verdict line. No neighbourhood sampling.
3. Advanced planning section: max_time, max_iterations, joint_resolution, tries, swept check; saved with the run.
4. One resolved overlay, no timeout.
5. Confirms exactly as the old monitor: a planned motion never asks, whatever its travel.
6. Plots come forward by themselves.

## 1. Planning printout

`shared/assembly_planning/route_search.py`
- `RouteSearch.free`: per try, before the call `self.log(f"{planner} try {k}/{tries}: max_time {s:.0f} s, max_iterations {n},
  joint_resolution {r} rad")`; time it (`time.monotonic`); after it `try {k}: path of {N} waypoints in {t:.1f} s` or
  `try {k}: no path in {t:.0f} s (tamp: {info.get('failure_reason', 'no reason')})` (the same string goes into the failure
  text, change 2). The free search has no seed (tamp's `plan_free_dual_arm` takes none); the held search's line
  (`tamp_route._transfer_held`) adds `seed {SEED_STEP * k}`.
- `one_arm`: the same two lines (`SUPPORT_TIME`, `found.reason`).
- `swept_failure`: log `swept check: {samples} samples between {n} waypoints, clear` or the existing "segment i of n
  sweeps through …". `refuse_endpoints`: log `{where} clear` / `the {where} collides: …` as it checks.

New `shared/assembly_planning/stdout_lines.py` (~25 lines): `captured_lines(line: Callable[[str], None])`, a context
manager: `os.pipe()`, `os.dup(1)`, `os.dup2(write_end, 1)`, a daemon thread reading the pipe line by line into
`line(...)`; on exit flush, `os.dup2(saved, 1)`, join. fd-level because PyBullet redirects stdout itself. Facts that
make it safe: the ROS log and `ctx.log_info` write to **stderr** (rclpy's default; nothing sets
`RCUTILS_LOGGING_USE_STDOUT`), so capturing fd 1 swallows nothing of the monitor's and the captured lines cannot loop;
compas_fab's own fd redirect (`external/compas_fab/.../pybullet/utils.py:13-48`) nests LIFO with ours. Leave
`experiments/assembly_plan_check.py:_captured` as is.

`plugins/motion_planner/plugin.py`: `self._log_tamp = lambda line: ctx.log_info(line, "tamp")` (source
`motion_planner/tamp`: `PluginContext.log_info(message, topic)` already names sources that way, `plugin_api/context.py:141-143`;
the console's Source/Hide work on it); wrap `work(...)` in `_work` with `captured_lines(self._log_tamp)`, i.e. the whole
request on the one worker thread. Docstring: say so.

Tests: `test/plugins/motion_planner/test_motion_planner.py`: a fake work that `print("[ssik] hello")`s and
`os.write(1, b"b3Printf\n")`; both reach `_log_tamp`, and after `idle(plugin)` stdout is restored.
`test/shared/test_planning_tamp_route.py::test_the_workaround_searches_without_the_bar`: collect `log`, assert a
`try 1/3` line and three `no path in … (tamp: birrt_failed)` lines.

## 2. Failure diagnosis: endpoints and the straight path

`route_search.py`
- `free`/`one_arm`: the failure text keeps tamp's reason; when every try ended "no path" (no swept hit), the refusal
  becomes `f"{planner}: {'; '.join(failures)} ({tries} tries); {self.straight_line(goal)}"`.
- New `straight_line(self, goal) -> str`:
  1. `for where, joints in (("start", None), ("goal", goal))`: `pairs = self.hit(where, joints)` (full report; `hit`
     already pushes a `CollisionHit` to `self.hits`); log `{where} clear` or return `the {where} collides: …`.
  2. `self.mirror.sync(self.scene)`; `report = swept_check(self.mirror, self.names, [start, goal])` (`collision.py:209-247`
     samples strictly between the endpoints at `SWEPT_STEP`); sync again.
  3. `report.ok` → `"start, goal and the straight path between them are clear: the search found no way in {tries}×{max_time:.0f} s"`;
     `report.error` → `"the straight path could not be checked: …"`.
  4. else `share = max|first − start| / max|goal − start|`; `pairs = self.hit(f"the straight path at {share:.0%}",
     report.colliding_joints)` (the motion planner's existing push to the Collisions tab,
     `motion_planner/plugin.py:80-89, 224-234`, draws the first hit ticked); return
     `"start and goal clear; the straight path first hits {pairs} at {share:.0%} — the search found no way around in {tries}×{max_time:.0f} s"`.
  Log the verdict too. `hit`, `swept_check`, `collision_hit` unchanged.

Tests: `test_planning_tamp_route.py`: a slow test modelled on `test_a_box_at_home_fails_the_way_home` (`:229-251`):
monkeypatch `route_search.plan_free_dual_arm` → `(None, {"failure_reason": "birrt_failed"})`, a 0.1 m box at Cindy's
left tool0 at the midpoint joints of the way home; assert `"birrt_failed" in reason`, `"the straight path first hits"`,
a hit whose `where` starts with "the straight path at" and names `obstacles/box`. Extend the workaround test to
assert the verdict names start and goal clear.

## 3. Advanced planning settings

- `route_search.py`: `@dataclass(frozen=True) class FreeSearch(max_time=120.0, max_iterations=50, joint_resolution=0.1,
  tries=3, swept_check=True)` with `tamp() -> dict` (the three tamp kwargs). `RouteSearch.__init__(…, search: FreeSearch
  = FreeSearch())`; `free(goal, goals=None)` drops its `settings` parameter, uses `self.search`, and skips
  `swept_failure` when `not swept_check` (`route.swept` stays None); `one_arm` uses `tries` and the toggle too.
  `ATTEMPTS` stays for HELD only.
- `tamp_route.py`: delete `FAR` and `HOME`; `plan_movement(…, search: FreeSearch | None = None)` → `_route` → `_Route`;
  `_transfer_free`, `_free_to_load`, `_free_home` call `self.free(...)`. **Behaviour change, deliberate:** home runs
  with the step settings (120 s default, was 30 s), and tries/swept also govern the one-arm support searches.
  `_transfer_held`, `servo_route.py`, `check_goal` untouched.
- `shared/motion_planner_contract.py`: `MotionPlanRequest.search: FreeSearch | None = None`;
  `plugins/motion_planner/work.py::run_plan` passes it.
- `shared/runs/runs.py::RunSettings`: `plan_max_time=120.0`, `plan_max_iterations=50`, `plan_joint_resolution=0.1`,
  `plan_tries=3`, `plan_swept_check=True`, ranges `(5, 600) s`, `(1, 1000)`, `(0.01, 0.5) rad`, `(1, 10)`; older
  `run.json` files already read defaults (`runs.py:225-227`). `plugins/assembly_run/writes.py::NUMBER_SETTINGS`: the
  four numbers (int derived for iterations/tries as `spiral_turns`).
- `ui/assembly_tab.py`: `PLAN_SETTINGS_BAND = 650.0` between `RUN_BAND` (600) and the plots' band, so opening the
  folder moves no button (nothing-jumps); the plots leave the tab in change 6, so the folder becomes last.
- `plugins/assembly_steps/panel.py`: `with ctx.view.ui(band=PLAN_SETTINGS_BAND) as gui, gui.add_folder("Advanced
  planning", expand_by_default=False)`: sliders Max time s, Max iterations, Joint resolution rad, Tries; checkbox Swept
  check; defaults from `RunSettings()`; callbacks `ctx.defer_value(…, operator_only=True)` →
  `ctx.require("assembly_run").request_settings(**{name: value})` as `assembly_run/panel.py:76-120`; `draw` writes the
  widgets back when `view.settings` changed (`assembly_run/panel.py:166-175` pattern).
- `plugins/assembly_steps/plugin.py`: drop `PLAN_TIME_LIMIT` and the `ATTEMPTS`/`FAR` imports; `make_search` builds
  `FreeSearch` from `view.settings`, passes `search=` in the `MotionPlanRequest`, `time_limit = tries * max_time + 60 +
  WAIT_MARGIN`.

Tests: `test_planning_tamp_route.py` (fake `plan_free_dual_arm` records kwargs; `swept_check=False` → `route.swept is
None`); `test/plugins/motion_planner/test_motion_planner_work.py` (`search` reaches `plan_movement`);
`test/plugins/assembly_steps/test_assembly_steps.py` (`lab.run.request_settings(plan_tries=1, plan_max_time=10)` → the
request's `search` and `Search.time_limit`; the folder's widgets written back); `test/plugins/assembly_run/test_assembly_run.py`
(refusal rows `plan_tries=0`, `plan_max_time=1000`); `test/shared/test_runs.py` (older run.json drops the `plan_*`
names); `test/ui/test_assembly_tab.py` band order.

## 4. Ghosts

- (a) `plugins/assembly_steps/step_panel_base.py::_draw_ghosts`: drop the `RecentUse` gate for both ghosts: the target
  ghost shows while the step has a target, the path ghost while there is a plan, no timeout. `self._used` stays for
  the diagnosis nodes; fix `_add_ghosts`'s docstring. base_planner/arm_planner (`shared/planner/planner_panel.py`) keep
  theirs. Test: a fake ghost in `lab.plugin._target_ghosts` with `RecentUse._latest = None`; after `lab.update()` its
  `show` was called; no current step → `hide`.
- (b) `shared/process_contract.py::Carry`: `base: Pose | None = None` (the base the joints were solved or planned at;
  None draws the authored base). `assembly_steps/plugin.py`: `checked_goal` passes `base=check.base`, `accepted`
  `base=self._basis.base`. `plugins/process/plugin.py::draw` (`:600-608`): when the carry applies and has a base,
  `state = replace(movement.start, robots={**robots, carry.robot: replace(robot_state, base=carry.base)})` before
  `drawing.show` (robot, tools and held bar move together; `drawing.py` untouched); `_details_html` chip says "and
  base". Tests: `test/plugins/process/test_process_marks.py` carry test asserts Cindy's drawn base is the carried one
  at and after the movement, the authored one before; `test_assembly_steps.py::test_check_goal_asks…` and
  `…carries_its_end…` assert `carry.base`.

## 5. Confirms as in the old monitor

`plugins/movement_exec/sequences.py`
- `_motion`, `_insert`, `_retreat`: no `confirmed(...)`. Exec is the go. The prompt text becomes a log line
  (`ctx.log_info`, or `log_warn` when the largest travel is over `LARGE_TRAVEL`, 60°, so the warning still shows). The
  world re-check and the rebuild from the live joints stay, right before the send.
- `_handoff`, `_robotiq`: keep their Confirm (old: step button + Confirm). `spiral.py` end prompt: keep.
- Servo (`servo_part.py:195-201`): pass 1 always prompts, **plain** (old iteration 1 skipped the safeguard); later passes
  prompt only over `CONFIRM_TRAVEL` 5°, warn. Change `warn = travel > CONFIRM_TRAVEL and k > 1`.
- The bar mount is one prompt: `shared/movement_exec_contract.py::step_parts` folds adjacent BY_HAND parts into one
  `PartPlan` (index and movement of the first, `movement` text "B20_J_place_bar + B20_J_grasp", texts joined: "Operator
  places the bar by hand; clamp it: tighten the gripper screws of left_ur_arm, right_ur_arm with the tools' buttons until
  they stall") with a new field `also: tuple[int, ...]` (the folded indices). `movement_exec/plugin.py::_run` adds
  `part.also` to `done` when the part is DONE; the record file keeps the first movement's name. `assembly_steps/panel.py`
  exec line: "waits for B20_J_place_bar + B20_J_grasp by hand".
- `shared/execution/operator.py` untouched.

Click count after: B20_J transfer step = Exec + Confirm (mount) + Confirm (servo pass 1); insert = Exec + Confirm
(servo pass 1); B20_R = Exec, Exec; B21_H = Exec, Confirm (servo), Exec+Confirm (open), Exec, Exec+Confirm (close).

Tests to update (they pin counts/texts): `test_assembly_steps.py:731` (B21_H `[1,1,1,1]` → `[0,1,0,1]`), `:750` (B20_J
transfer 3 → 1), `test_servo_chain.py:257` (4 → 2) and `:281-282` (files: place_bar+grasp become one record),
`test_movement_exec.py:1288`, `test_exec_compliant.py:157` (retreat 1 → 0; the retreat's prompt text test becomes a
log-line test), `test_servo_part.py:661-664` (`prompted == [True, True, False]` stays; pass 1 now plain),
`test_movement_exec.py:366-390` (parts `(3,"motion"),(1,"operator"),(SERVO)` with `also=(2,)`; done `(3,1,2,SERVO)`),
`:1337` spiral warn stays; my earlier `test_the_grasp_is_done_by_hand_and_sends_the_tools_nothing` becomes the
folded-prompt test; `test_the_exec_line_names_the_servo_after_the_motion`. Safety tests that Confirm a motion then
move another robot (`test_another_robot_moving_at_a_…_prompt_refuses_it_after_confirm`) change to "moving when Exec
is clicked refuses it" (the check still runs before the send).

Docs: `doc/fake_lab_test_manual.md` 4.2–5.3 (fewer Confirm steps, the merged mount prompt), README's movement_exec row.

## 6. Plots that come forward

Floating panels, started minimized to their bar, expanded from code at the moment the old windows opened (the only
server-side "bring forward" viser offers). Never minimized by code again: the operator closes them with the bar
control, as with the DPG windows.
- `plugins/assembly_steps`: the two plan plots (`plan_plots.py`) move from `PLOTS_BAND` into `ctx.view.panel()`
  "Plan plots" → `panel.float(x=15, y=-15, width=W, height=H)` + `minimize()` in setup (console pattern,
  `plugins/console.py:88-101`); `panel.expand()` in `panel.py:138-142` when `plugin.path` is a new plan. `PLOTS_BAND`
  removed from `ui/assembly_tab.py`.
- `plugins/movement_exec`: the **Plots** top tab (`plugin.py:199-201`) becomes a floating panel "Exec plots" next to the
  plan one (`x=15+W+10, y=-15`), minimized in setup; `expand()` in `start_servo` / `start_compliant`
  (`plugin.py:290-294`). The servo and F/T groups sit in two folders; `visible` switches to the running kind (no
  buttons in this panel, so nothing-jumps is not at stake).
- `test/plugin_fakes.py::FakeContext.view`: add `panel()` returning a fake with `float`/`minimize`/`expand` recording
  calls (as `test_console.py:68-84`). Tests: `test_assembly_steps.py:186-212` (plan fills the plots → also "expand was
  called once per plan"); `test/plugins/movement_exec/test_exec_plots.py` (expand on start_servo/start_compliant, the
  right folder visible). README's assembly_steps/movement_exec rows, fake-lab manual 2.1 (two minimized bars at the
  bottom left) and 4.2 ("the Plan plots panel opens").

## Order and verification

1. Change 3 (signatures) → 1 and 2 (route_search.py) → the capture in the motion planner → 4 → 5 → 6.
2. While iterating: `python3 -m pytest -n 0 test/shared/test_planning_tamp_route.py`, `-n 0 test/plugins/motion_planner
   test/plugins/assembly_steps test/plugins/process test/plugins/assembly_run test/plugins/movement_exec test/shared/test_runs.py
   test/ui`, `-n 0 test/structure`.
3. Before commit: `cd ~/Code/ros2_ws_next && source venv/bin/activate && source /opt/ros/humble/setup.bash && source
   install/setup.bash && export HUSKY_DRIVE_ROOT=$HOME/husky_fake_lab/drive && cd src/husky-assembly-teleop && python3 -m pytest`
   (the user's fake lab keeps running; never touch its monitor; one known slow timeout,
   `test_exec_dry_run.py::test_real_plans…`, predates this work).
4. Fake-lab check in my own slot (domain 72, port 8097, scratch HOME; headless Chrome via CDP): Plan a transfer and see
   the try lines and `motion_planner/tamp` lines in the console; make B21_J's transfer fail (tries 1, max_time 5 in
   Advanced planning) and see the verdict and the straight-path hit in Collisions; Live IK → the overlay at the live base
   and the yellow ghost still there a minute later; Exec a transfer → it moves at once, one mount prompt, one servo prompt;
   the Plan plots panel opens on Plan, the Exec plots panel on the servo.
5. Task note `tasks/2026-10-27_assembly_planning_feedback.md` with this spec and the two deliberate behaviour changes
   (home at the step settings; tries/swept on the support searches). Manual and README rows as listed. One commit per
   item or two grouped (few, grouped commits), on top of the uncommitted fake-lab/console work.

## Status 2026-10-27 evening: implemented, not committed

All six items are in the working tree (with the morning's fake-lab, console and by-hand grasp work), reviewed, default
suite green. Deviations from the spec above, all deliberate:
- Home and the support one-arm searches run with the Advanced planning settings (tries, swept check; the support
  search keeps its own 30 s per try, `SUPPORT_TIME`). A held transfer (workaround off) keeps HELD's 3 × 120 s; the
  Plan wait limit never drops under that.
- The free search has no seed (tamp takes none); the held search's try line names its seed.
- The straight-path diagnosis test uses B20_J's transfer, not the way home: the way home's straight path already hits
  the ground joint at 1 %, so a box there is never the first hit.
- `captured_lines` sets `sys.stdout` to line buffering, so tamp's lines reach the console as they are printed even
  after compas_fab swaps `sys.stdout` for a block-buffered file at model loading; the first request's lines after
  that loading may still arrive at its end.
- The two plot windows are 460 × 560 px at the bottom left (Plan plots at x = 15, Exec plots next to it); the Plan
  plots window opens once per new plan (not on a retime or Clear).
- The "Advanced" folder of the other session's want-list WIP (RRT time / Joint resolution) was replaced by the
  "Advanced planning" folder; the rest of that WIP (ghost colours, `ghost_timeout` 0, movement buttons, collisions'
  2-minute untick) is untouched and not part of this spec.
- Flaky, not ours: `test/robot_interface/test_newest_only.py::test_a_garbled_message_is_dropped_not_fatal` segfaults
  in rclpy's deserializer under 4 workers now and then (2 of 4 runs); it passes alone and in the other runs.
- The fake-lab check (step 4 of the verification) is not done yet.
