# One blue ghost, an orange base target, and the representation note

## Context

Three asks from the user (2026-10-29), all about seeing where a robot is meant to be:

1. **The base planner's target ghost is blue, like the process panel's robot.** The user wants it orange so the two never pass for each other.
2. **The Assembly tab draws two blue robots for the selected step.** The assembly_steps "target" ghost (G1) always stands on the *live* mocap base with the Rhino or Live IK joints, so its tool0 is off by the base offset; the process plugin's overlay robot (G3) stands at the Rhino base with the Rhino (or assumed) joints and is overlaid with the `Carry` that Live IK or a plan posts (base and joints found live). The old monitor had **one** goal ghost showing the selected movement's state: Rhino's, the propagated one, or the Live IK / plan result at the live base. G3 already does those three cases; G1 is the duplicate. Decision (user): **remove G1, G3 is the one blue ghost**; the green path ghost stays. Also (user): the green path ghost opens at the plan's **end**, as the old trajectory view did ("preview end" from the 27 Oct want list).
3. **A note comparing the action / movement / state representation of the old and the new monitor.** No such note exists: `doc/run_record.md` §4 covers the propagation story only, `process_format.md` Appendix B maps the file format only, the Claude Doc "Old vs new monitor: features" is a port-status table. Decision (user): write it as **`doc/representation_old_vs_new.md`**, next to `run_record.md`.

Already done this session, uncommitted, in `plugins/base_planner` (+ README, its panel test): margin default 0.05 → 0.02 with one **Margin m** slider on the panel; target ghost hidden once the robot stands at the target; Load's **Step** button orange. These go into the first commit below.

## Part A: ghosts

### A1. Orange base-planner target ghost

- `husky_assembly_teleop/ui/ghost.py`
  - after `TARGET_COLOR` (l.28): `BASE_TARGET_COLOR = (0.94, 0.55, 0.0, 0.5)  # orange (#f08c00, the TARGET section): the base planner's target`
  - `KIND_COLORS` (l.31): add `"base_target": BASE_TARGET_COLOR`.
  - header comment l.25-26 and the `kind:` lines in `RobotGhost.__init__` (l.70) and `robot_ghosts` (l.121): name the new kind; the module docstring l.4-5 is stale (assembly_steps no longer gates by `RecentUse`): "The planner panels show theirs only while `RecentUse.active`; assembly_steps shows its path ghost while there is a plan."
- `husky_assembly_teleop/shared/planner/planner_panel.py`: class attribute after `planner = ""` (l.56): `target_kind = "target"` with a `#:` comment ("a key of `ui.ghost.KIND_COLORS`; a subclass may pick another colour"); `_add_ghosts` l.145 uses `self.target_kind`.
- `husky_assembly_teleop/plugins/base_planner/panel_plugin.py`: after `requires` (l.63): `target_kind = "base_target"` with `#: Orange, so the target never passes for the process panel's blue robot.` The arm planner inherits blue: no edit.
- Test, `test/plugins/base_planner/test_base_planner_panel.py`: one assert next to `test_target_ghost_hides_once_the_robot_stands_at_the_target`: `KIND_COLORS[BasePlannerPanel.target_kind] != KIND_COLORS["target"]` (import `KIND_COLORS` from `husky_assembly_teleop.ui.ghost`).

### A2. Remove the assembly_steps target ghost (G1)

- `plugins/assembly_steps/step_panel_base.py`
  - l.8 docstring: drop `show_target` from the subclass list; l.135-137: delete the `show_target` abstract.
  - `_has_target` stays (it gates Plan at l.204 and is set at `plugin.py:182`); fix its comment l.84: "Whether there is a step to plan for; until then Plan is refused."
  - `_add_ghosts` l.148-157: delete `self._target_ghosts = ...`; docstring "Build a path ghost per robot, and start counting the panel's use (`_used`)." `RecentUse` and `RobotGhost` imports stay (both still used).
  - `_draw_ghosts` l.308-318: drop the target loop and the `target =` line; docstring "Show the path ghost while there is a plan; hide every other. * No timeout: a plan is no click, and the operator looks at it for minutes."
- `plugins/assembly_steps/plugin.py`: delete `show_target` (l.556-570). Keep the `goal` property and l.182. Module docstring l.11: "logged, shown as the target ghost and recorded" → "logged, recorded and carried into the process panel's drawing".
- `plugins/assembly_steps/panel.py:59` Live IK hint: "...and show it as the target ghost; nothing moves." → "...and show it on the process panel's blue robot; nothing moves."
- `plugins/assembly_steps/goal.py:2`: "kept for the target ghost and the step's next plan" → "kept for the process panel's drawing and the step's next plan".
- Tests:
  - `test/plugins/assembly_steps/test_assembly_steps.py:137`, `:763` and `test/plugins/assembly_steps/test_servo_chain.py:137`: `._target_ghosts, ._path_ghosts = {}, {}` → `._path_ghosts = {}`.
  - `test_assembly_steps.py:224-248` → `test_the_path_ghost_stays_while_there_is_a_plan_whatever_the_panel_was_last_used`: only the path ghost; expected calls `["path hidden"]`, `["path shown"]`, `["path hidden"]`.
  - `test_assembly_steps.py:661-690` (`test_check_goal_asks_with_the_run_and_records_the_answer`): drop the two `show_target` checks (l.678-682, l.687-690) and the docstring line l.664; keep the carry asserts l.683-686, which pin what G3 shows.

### A3. The carry at a later movement shown at Target

Today `_carry_applies` (`plugins/process/plugin.py:658-677`) overlays the carried joints on every later movement of the action, start or target; so after a transfer plan, the insert shown at Target shows the transfer's end instead of its own target joints. The old monitor carried into starts only.

- Rule: at the carry's own movement, unchanged (Target only); at a later movement, the carry applies when shown at start, or at Target when that movement's target names no joints for the carry's robot. Replace l.677 with:
  ```python
  if number == at:
      return self.at_target
  own = movement.target.joints.get(carry.robot) if movement.target is not None else None
  return number > at and not (self.at_target and own)
  ```
  Docstring l.661 accordingly; one clause that when the carry does not apply the authored base shows too (`_with_base` not called): the pose those joints were authored for. The "joints from live IK / plan" chip (l.770-774) uses the same test, so it follows.
- Test, `test/plugins/process/test_process_marks.py` `test_the_carry_is_drawn_at_and_after_its_movement_and_dropped_on_load` (l.57-74): after l.72 add `assert cindy("B2_M3_insert", True)["left_joint1"] == 0.35, "its own target joints, not the carry's"` (the fixture's insert has cindy target joints `joints(0.35)`, `test/process_fixtures.py:249`); extend the docstring.

### A4. Preview opens at the plan's end

- `plugins/assembly_steps/step_panel_base.py:243`: on a new plan `self._t, self._playing = self.path.duration, False` (comment: the plan's end, where the old monitor's trajectory view opened; Play runs from the start). `_on_path` l.196-197 already restarts Play from 0 when `_t >= duration`. The shared `PlannerPanel` (base/arm) keeps opening at 0.
- Test: in `test_assembly_steps.py`, where a plan arrives (grep `plan_state == "ready"` / `_t`), assert `plugin._t == plugin.path.duration` after the plan and `_t == 0.0` after the first Play click.

### A5. Docs

- `doc/glossary.md:55-57` ghost entry: "a see-through robot copy for a target, a plan or a preview: blue where a robot should go (the process panel's one robot per movement, the arm planner's target), orange at the base planner's target, green where a plan has it; the planner panels show theirs only while last used, assembly_steps' path ghost while there is a plan (only the live robots show their true colours)."
- `README.md:315` (`assembly_steps`): "logged, shown as the target ghost, recorded, carried into `process`'s drawing, and reused" → "logged, recorded, carried into `process`'s drawing (its blue robot moves to the live base with the joints found), and reused"; also "**Play** the preview ... (slider **Preview time**)" gets "opens at the motion's end".
- `README.md:310` (`base_planner_panel`): before "The floor shows the path": "An orange ghost stands at the target (hidden once the robot is there) and a green one rides the planned path."
- `doc/real_lab_test_manual.md:210` ghost row: "blue where the selected movement has the robot (after Live IK or a plan, on the live base), orange at the base planner's target, green along a plan"; l.769: "the blue process robot turns to Target and shows the arms at the insert start on the live base."
- `plugins/process/drawing.py:13-14`: "the goal blue (`ui.ghost.GOAL_COLOR`): the Assembly tab's one blue robot, as the old monitor drew a movement's state".

## Part B: `doc/representation_old_vs_new.md`

"Action, movement and state: the old monitor and the new, side by side." Written for the user, who knows the old monitor; plain language, tables over prose, ~150 lines. The source checkouts:

- OLD: `~/Code/ros2_ws/src/husky-assembly-teleop` at `6c19ff9c` (`yh/multi-robot-monitor`), the final old monitor. Say at the top that the in-repo `husky_assembly_teleop/old/` is an older snapshot without the sidecars, `progress_io.py` and the carry functions. Files: `husky_assembly_teleop/husky_monitor.py` (M), `bar_action_io.py`, `plan_skeleton_io.py`, `progress_io.py`, `external/rs_data_structure/rs_data_structure/bar_action.py`.
- NEW: `external/scaffolding_assembly_core/doc/process_format.md` + `scaffolding_assembly_core/process/types.py`, `plan_check.py`, `carry.py`; `shared/runs/runs.py`; `shared/process_contract.py` (`Carry`); `plugins/assembly_steps/goal.py`, `plugin.py`; `shared/execution/basis.py`; `shared/assembly_planning/steps.py`, `scene_for.py`; `plugins/movement_exec/plugin.py` (the recorded payload).

Sections (each a table where the content is a comparison; cite `file:line` or function names, verified against both checkouts while writing):

1. **Where the data lives** — one table, old | new: schedule (`ActionSchedule.json` `schedule[]` → `process.json` `schedule` + `actions/<id>.json`), actions (`BarActions/B3__J.json` → `actions/B3_J.json`, types), movements, trajectories (`Movement.trajectory`, `*.solved_keyframe.json` → `solutions/<action>.json`, stale on hash mismatch), run state (`progress.json` per problem + `.live-solved.json` sidecar → `runs/<YYYYMMDD_HHMMSS>/` with `run.json`, `progress.json`, `events.jsonl`, `movements/…`, `servo/…`).
2. **A movement** — side by side: old `Movement` (`movement_id`, `tag`, `start_state: RobotCellState`, `target_ee_frames`, `target_configuration`, `trajectory`, `controller`, the subclass as kind, `ToolMovement` extras) vs schema 2.1 `Movement` (`id`, `start: State`, `arms`, `base`, `path`, `coupled`, `controller`, `line`, `drives`, `ends_on`, `target: Target`; the kind derived). Note: old stores no end state (the next movement's `start_state` repeats the target); new: `end_state = start ⊕ target`, checked by B11.
3. **A state** — `RobotCellState` (`robot_base_frame`, `robot_configuration`, `tool_states`, `rigid_body_states` with `attached_to_link`/`attachment_frame`) vs `State` (`robots{id: RobotState(base, joints)}`, `present`, `attached{bar: [Holder]}`, `built`, `poses`, `tools`) and `Target` (partial: `robots{RobotTarget(base, joints, link_poses)}`, `tools`, `on`, `attached`, `built`); `null` = not decided.
4. **Live, in memory: what changes while the monitor runs** — the heart of the note. Old: the loaded `rs` objects are mutated in place: `_apply_live_base_to_movement` (live base into `start_state` on Plan), `_accept_trajectory` (traj[0] into the start, traj[-1] carried into later stationary steps and the next arm movement by `_carry_configuration_forward`), `_propagate_live_end` after Exec (measured joints + base), `_adopt_servo_corrected_end`, IK Live Base (`goal_arm_pose`/`goal_base_pose`), Reset to the clean export. New: the process is frozen and never edited; the plan starts from the live snapshot (`MotionPlanRequest(snapshot)`); `StepGoal`/`GoalCheck` (Live IK joints + the base solved at, reused within 2 mm / 0.5°); `PlanBasis` (start joints, base, other robots, statuses; stale rules); `Carry` (drawing only); `movement_exec.carried` → `goal_carried`; other robots from `beliefs_of` (Mark done's `ends`) via `scene_for` (live → belief → design → parked); the transfer workaround (`OperatorStep`, `as_run`) reorders for display and planning only. Point to `run_record.md` §4 for the narrative; keep this a table: event → old (what was overwritten where) | new (what is kept where, what reads it).
5. **Recorded on disk** — old: `.live-solved.json` sidecar (the whole mutated action, written on Mark done / Adopt / Export, loaded in preference), `progress.json` (`actions{status pending|done|skipped…}`, `robots{base_frame, configuration, source live|action_end_state|assumed|exported|parked}`, `holds`, `current_index`, `skeleton_fingerprint`), `<date>-servoing/`, `<date>-asbuilt/*.jsonl`. New: `run.json` (hashes, robots, commits, settings), `progress.json` (`actions{ActionRecord(status pending|done, seq, marked_at, assumed, ends)}`, `current{action, movement}`, `robots{RobotRecord(base, joints, source live|assumed, at, after_action)}`), `events.jsonl` (`plan`, `check_goal`, `check_tool0`: numbers only), `movements/<action>/<movement>.<n>.json` (`basis`, `plan.rows`, `command`, `exec`, `outcome`), `servo/…json` + `.png`; all through `assembly_run.request_record`. As-built record: not ported (say so).
6. **Old term → new term** — the mapping table from the exploration (schedule, holds, action files, ids `B4_J_M5_LM_insert` → `B4_J_insert`, `MovementKind` → `arms/path/coupled/controller`, `joint_tracking`/`cartesian_compliant` → `position`/`compliant`, `ManualMovement` → `ends_on: operator`, `tool_action`/`overlaps_next` → target `tools` / `drives`, `start_state` → `start`, `robot_base_frame` → `robots[r].base`, `robot_configuration` → `robots[r].joints`, `target_ee_frames` → `target.robots[r].link_poses`, `target_configuration` → `target.robots[r].joints`, `.live-solved.json` → none, `_carry_configuration_forward`/`_propagate_live_end` → live snapshot + Mark done `ends` + `Carry`, IK Live Base → `StepGoal`/`check_goal`, `OperatorStep(primary, absorbed)` → `OperatorStep(primary, with_motion, after_motion, skipped)`, `_transfer_as_free_move` → `transfer_as_free` + `as_run`, Reset → new run).
7. **Two known mismatches** (one line each): `process_format.md` names the run folder `runs/<yyyy-mm-dd_hhmmss>/` but the code writes `runs/<YYYYMMDD_HHMMSS>/`; §3.2 says `run.json` keeps solution hashes, `RunInfo` has none.

Link the note from `doc/run_record.md` (one line at the top of §4) and from `AGENTS.md` "Where things are" (next to the glossary line), and add the terms it introduces to `doc/glossary.md` only if missing (`carry`, `basis`, `belief` — check).

## Status (2026-10-29)

Parts A and B are implemented and tested, NOT committed: the user lands them with the search-tree work that is in
progress in the same checkout (motion_planner/plugin.py imports the untracked shared/assembly_planning/search_tree.py;
README.md, base_planner/panel_plugin.py and shared/planner/planner_panel.py hold hunks of both). Suggested grouping:

## Commits (few, grouped; on `yh/process_replan_execution`)

1. `[feat] base planner: one Margin slider on the panel (2 cm), the target ghost orange and hidden at the target, Step in orange` — this morning's uncommitted base_planner changes + A1.
2. `[feat] Assembly tab: one blue robot (the process panel's), the carry stops at a later movement's own target, the preview opens at the plan's end` — A2–A5.
3. `[doc] representation note: action, movement and state in the old monitor and the new` — Part B.

Leave the user's other uncommitted files alone (assembly_run, console, health, movement_exec, shared/assembly_planning incl. the untracked `search_tree.py` whose l.143 is 121 chars and fails flake8 — tell the user; do not fix it).

## Verification

- While iterating: `python3 -m pytest -n 0 test/plugins/assembly_steps test/plugins/process test/plugins/base_planner test/ui`.
- Before each commit: `python3 -m pytest` (default set, linters included; ~30 s; log to a file in the scratchpad). The flake8 failure in the untracked `search_tree.py` is pre-existing: report it, do not touch the file.
- Manual, fake lab (`doc/fake_lab_test_manual.md`): Assembly tab → step slider to a jointing action's transfer → **one** blue see-through robot at the Rhino base with the Rhino start joints; Show Target → Rhino target joints. Live IK → the blue robot moves to the live base with the IK joints, chip "joints from live IK". mv ▶ to the insert at Start → IK joints on the live base; Show Target → the insert's own Rhino target joints at the Rhino base, chip gone. Plan → the blue robot shows the plan's end ("joints from plan"); the green robot appears at the plan's **end** (slider at the duration); Play runs from the start. Plan › Base → Load Robot, move the target → an **orange** ghost at the target, hidden within 5 cm / 3° of the robot; Plan › Arm → its target ghost still blue.
- Task note: `tasks/2026-10-29_one_blue_ghost.md` with this spec (AGENTS.md "Task notes").
