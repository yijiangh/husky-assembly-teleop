# Action, movement and state: the old monitor and the new, side by side

The old monitor edited the loaded action in place (its start states, target configurations and trajectories) and saved
the edits beside the Rhino export, as `.live-solved.json` sidecars and one `progress.json` per problem. The new monitor
holds a frozen process (`scaffolding_assembly_core`, schema 2.1), keeps everything live in plugin state, and writes
only into a run folder. The narrative of how the state moves from step to step is `run_record.md` §4, the run files
are laid out in its §2; this note puts the data models side by side and says what each event changes.

* Citations. OLD is the final old monitor: the `ros2_ws` checkout of this repo at `6c19ff9c` (branch
  `yh/multi-robot-monitor`, 2026-10-21); M is its `husky_assembly_teleop/husky_monitor.py`. This repo's
  `husky_assembly_teleop/old/` is an older snapshot: it writes the sidecar but has no `plan_skeleton_io.py`,
  `progress_io.py` or `as_built_io.py`, and none of the carry functions. NEW is this repo; `types.py`, `plan_check.py`,
  `carry.py` and `process_format.md` are the core's (`external/scaffolding_assembly_core/`).

## 1. Where the data lives

| | Old | New |
|---|---|---|
| Schedule | `<problem>/ActionSchedule.json` (`plan_skeleton_io.py:1-27`): `robots`, `assembly_seq`, `holds`, `schedule` of `{index, action_id, type, bar_id, robot, file}` (`SkeletonAction` l.63-121, `PlanSkeleton` l.150-171) | `process.json`: robots, tools, bodies, mates and `schedule`, a list of action ids (`process_format.md` §2 l.49-60, §4) |
| Action files | `BarActions/B3__J.json`, `B3__R.json`: one `BarSceneAction` each, with `action_id`, `robot_id`, `active_bar_id`, `assembly_seq`, `walkable_ground_ids`, `movements` (`bar_action.py:275-357`); the `.live-solved.json` sidecar loads in preference (`plan_skeleton_io.py:432-444`) | `actions/B3_J.json`, `B3_R.json`: one frozen `Action` each (`types.py:302`), `type` one of `bar_jointing`, `bar_release`, `bar_holding`, `bar_holding_release`, `bar_placing` (§5 l.174-186); the monitor never writes one (`plugins/process/plugin.py:11`) |
| Movements | `Movement` subclasses inside the action, edited in memory (`bar_action.py:67-271`) | `Movement` frozen dataclasses inside the action (`types.py:259-300`) |
| Trajectories | `mv.trajectory`, in memory and in the sidecar; offline keyframes `*.solved_keyframe.json` (`bar_action_io.py:219`) | offline: `solutions/<action>.json`, stale once `solved_against` no longer matches (§10.1 l.408-438); live: the step's `_route` and the movement record (§5 below) |
| Run state | `<problem>/progress.json`, one per problem (`progress_io.py:1-25`) | `<process>/runs/<YYYYMMDD_HHMMSS>/` (`runs.py:1-24`): `run.json`, `progress.json`, `events.jsonl`, the records |
| Servo records | `<EXPERIMENT_DATA>/bar_action_live_replan_exe/<YYYYMMDD>-servoing/` (`husky_world.py:103, 2688-2712`) | `runs/<run>/servo/<action>/<movement>.<n>.json` and `.png` |
| As-built record | `<YYYYMMDD>-asbuilt/as_built_<problem>.jsonl` (`as_built_io.py:1-40`) | not ported |

## 2. A movement

| Old `Movement` (`bar_action.py:67-117`) | New `Movement` (`types.py:259-300`; §5.1 l.190-227) | Note |
|---|---|---|
| `movement_id` | `id` | |
| `tag` | `label` | |
| the subclass: `IndependentDualArmFreeMovement`, `EndEffectorConstrainedDualArmLinearMovement`, ... (l.120-271; `MovementKind`, `bar_action_io.py:462-473`) | `arms`, `base`, `path`, `coupled`, `controller`, `line`, `drives`, `ends_on` | The kind follows from the parts and is not stored (l.192-196); the monitor's `Routine` (`shared/assembly_planning/movement.py:31-45`) classifies it |
| `start_state`, a `RobotCellState` | `start: State` | §3 |
| `target_ee_frames` keyed `left`/`right` or `arm` | `target.robots[robot].link_poses` keyed by flange link id | |
| `target_configuration` (None until solved) | `target.robots[robot].joints` | The keyframe: one IK solution chosen in Rhino, kept beside the pose (l.270-281); `null` means not decided (l.44) |
| `trajectory` (None until planned) | none in the process | §1, trajectories |
| `controller`: `joint_tracking`, `cartesian_compliant`, `none` | `controller`: `position`, `compliant`; absent when no arm moves | |
| `ToolMovement.tool_action`, `tool_names`, `overlaps_next` (l.207-271) | `target.tools{tool: grip}`, `drives{tool: tighten|loosen}`, `ends_on: tools` | A screw that ran on through the next movement is now one combined movement, arms and tool together (l.230-237) |
| `ManualMovement` (l.176-205) | no `arms`, no tool part, `ends_on: operator` | |
| `notes` | `notes` | People only; the library never reads them (l.47, §5.4) |
| no end state: the next movement's `start_state` repeats the target | `end_state(movement)`: the start with the target laid over it (`plan_check.py:133-147`) | B11 `check_chain` (l.356) checks that each movement ends where the next starts, within `JOINT_TOLERANCE` (l.28) |

## 3. A state

| `RobotCellState` (compas_fab `robots/state.py:22`) | `State` (`types.py:153-171`; §5.2 l.239-255) and `Target` (`types.py:182-257`; §5.3 l.257-268) | Note |
|---|---|---|
| `robot_base_frame`: the one planned robot | `robots{id: RobotState(base, joints)}`: every robot; `null` = not in the scene | The old monitor layered the other robots in from `progress.json` (`_apply_obstacle_beliefs`, M:3060-3061; `obstacle_tool_states`, `progress_io.py:704`) |
| `robot_configuration`, a compas `Configuration` | `RobotState.joints`: joint name to value, or `null` | `null` = not decided by the process (l.44, l.219-222): the monitor fills it from measurements |
| `tool_states{name: ToolState(frame, attached_to_group, attachment_frame, configuration)}` (`state.py:391`) | `tools{id: ToolState(grip, on)}` | Where a tool is mounted is in `process.json`; the state says only its grip and what it is on |
| `rigid_body_states{name: RigidBodyState(frame, attached_to_link, attachment_frame)}` (`state.py:490`) | `present`, `attached{bar: [Holder(to, grasp)]}`, `built`, `poses` | One rule says where a bar is (l.43): built, at its assembly pose; attached, with its first holder; else `poses` |
| the target lives on the movement: `target_ee_frames` and `target_configuration` | `Target`, a partial `State`: `robots{RobotTarget(base, joints, link_poses)}`, `tools`, `on`, `attached`, `built` | Only what the movement decides; `present` and `poses` are never in a target; `link_poses` replace `target_ee_frames` |

## 4. Live, in memory

| Event | Old: what was overwritten, where, by what | New: what is kept where, who reads it |
|---|---|---|
| Load an action | The file (sidecar first) becomes the working copy; `_backfill_m0_target_from_m1` (M:6706-6721) gives the travel to load the transfer's start as its target | `process._take` (`process/plugin.py:502-508`) keeps the frozen process, drops marks and carry; `assembly_steps._new_step` (`plugin.py:170-193`) starts the step clean: no plan, no goal, no offer |
| Plan clicked | `_apply_live_base_to_movement` (M:3013-3068) writes the mocap base into `mv.start_state.robot_base_frame`, and the other robots' beliefs into the same state, on every Plan | `make_search` (`assembly_steps/plugin.py:462-501`) builds `MotionPlanRequest(loaded, step, snapshot, robots, beliefs, goal_joints, goal_poses)` (`motion_planner_contract.py:49-81`) from this tick's snapshot; nothing is written |
| Plan accepted | `_accept_trajectory` (M:6190-6300): sets `mv.trajectory`; a free move's traj[0] goes into `start_state.robot_configuration`; an insert or retreat starting >1e-3 off the propagated start is rejected; traj[-1] is written into the next arm movement's start and the stationary steps before it (`_carry_configuration_forward`, M:6429-6460) | `accepted` (`plugin.py:511-531`) keeps `_route` and `_basis` (`PlanBasis`, `basis.py:36-82`: start joints, base when planned, other robots, statuses, time, `goal_base`, `transfer_as_free`), records a `plan` event and posts `Carry(..., "plan", basis.base)` for the drawing; stale past 2° on the arm, 2 mm / 0.5° on the base, 1 cm / 1° on another robot (`basis.py:27-32`) |
| Live IK | `ik_live_base_for_selected_movement` (M:9893-10049): IK at the live base to the previous movement's authored targets; sets `goal_arm_pose` and `goal_base_pose` = the live base (M:10044-10049); `mv.trajectory` untouched | `check_goal` / `checked_goal` (`plugin.py:360-411`): `StepGoal` (`goal.py:37-92`) keeps one `GoalCheck` (the goal joints and the base solved at; `route_result.py:77-95`) per step; Plan reuses it while the base stays within `STALE_BASE` (`basis.py:30`); posts `Carry(..., base=check.base)`; records `check_goal` |
| Exec done | `_exec_then_propagate` (M:6556-6602), then `_propagate_live_end` (M:6468-6509): measured joints and mocap base written into the following start states; the next plan dropped if >1e-3 off (`_drop_movement_trajectory`, M:6775-6790); `_save_live_belief` (M:6511-6553) writes `progress.json` after every step | `_executed` (`plugin.py:620-636`) clears the plan, used once (it comes back only when nothing was sent); `_record_part` (`movement_exec/plugin.py:412-443`) writes the movement record; nothing is carried: the next Plan starts from the live robot again |
| Servo done | the same carry, with the mocap base (`source='Servo'`, M:6468-6509); `_adopt_servo_corrected_end` (M:6609) | `run.carry = Carried(loaded, action, insert, serial, at)` (`servo_part.py:355`; `movement_exec_contract.py:163-177`); the insert's Plan goes to `carried_insert_goals` (`servo.py:500-521`) and its `plan` event says `goal_carried`; dropped when another action runs or the transfer runs anew (`movement_exec/plugin.py:286-294`) |
| Check tool0 | the servo loop's readout, `measure_servo_tool0_error` (`husky_world.py:1523`) | `check_tool0` (`plugin.py:276-303`): the live tool0 against `target.link_poses`, drawn and recorded; nothing kept |
| Mark done | `_mark_action_done_task` (M:12019-12100): the belief (live; another robot's exported end as `assumed`) into `progress.json`; the planned trajectories into the sidecar (M:12088-12092) | `request_mark_done` (`assembly_run/plugin.py:163-185`), then `_apply_mark` (`writes.py:109-131`): every live robot's `live_record` (`assembly_run_contract.py:110-112`) into the action's `ends` and the run's `robots` (`runs.mark_done`, l.268-291); the acting robot not live: `exported_end` as `assumed` on a second click (l.115-132); a `mark_done` event |
| Cancel / off-target end | nothing carried (M:6590-6601; `_cancel_exec`, M:6604) | the record's `outcome` says how it ended (`movement_exec/plugin.py:438-440`); nothing to undo, since nothing was written |
| Reset / new action | Reset (`_reset_skeleton_task`, M:12282) moves `progress.json` and every sidecar to `archive/<time>/` (`progress_io.py:489-497`); another action loads from its file | New run (`request_new_run`, `assembly_run/plugin.py:207-216`, on a second click): a new run folder, the old one kept; another action: `_new_step` drops plan, goal and carry |
| Transfer workaround | `_transfer_as_free_move` (M:4648), `_plan_transfer_as_free_move` (M:4702); the steps before the transfer get its last joints (M:6254-6262); `OperatorStep(primary, absorbed)` (`bar_action_io.py:726-737`) | `OperatorStep(primary, with_motion, after_motion, skipped)` (`steps.py:29-49`); `as_run` (l.109-130) reorders for the panel and the planners' targets only, `_transit` (l.180-188) builds the free move; the box is the process plugin's, per session (`process/plugin.py:13-15`), recorded in `PlanBasis.transfer_as_free` |

`run_record.md` §4 tells how these add up over a step. The rule behind every row: the new monitor never writes into
the process (`process/plugin.py:11`). The joints the old monitor carried forward survive only as `Carry`
(`process_contract.py:27-48`), and only in the drawing: `_carry_applies` (`process/plugin.py:658-680`) lays its joints
and base over the authored state at the carry's movement (shown at its target) and at the action's later movements;
without a carry the panel draws the authored joints, `null` ones assumed by `assumed_start_all` and `joints_at`
(`carry.py:31, 94`). Plans, runs and records never read it. The other robots come from the snapshot when measured, else
from the run's beliefs (`beliefs_of`, `assembly_run_contract.py:135-137`), else the design, else parked (`scene_for.py:1-10`).

## 5. Recorded on disk

| File | Keys | Written by |
|---|---|---|
| OLD `<stem>.live-solved.json` | the whole `BarSceneAction`: start states, target configurations, trajectories; the clean export is never overwritten (`bar_action_io.py:236-310`; `_bar_action_write_path` M:6826-6854, `_write_action_halves_for` M:6856-6888) | Mark done (M:12088), Adopt (`_save_m1_m0_confs_to_bar_action_file` M:6890, the write at l.6929), Export travel-to-load (`export_m0_plan_to_bar_action_file` M:6946, the write at l.6985) |
| OLD `<problem>/progress.json` | `actions{index: {status pending|done|skipped, marked_at, marked_by_robot, run_id}}`; `robots{name: {base_frame, configuration (arm joints only), source live|action_end_state|assumed|exported|parked, at, after_entry}}`; `holds`; `current_index`; `skeleton_fingerprint` (`progress_io.py:1-25, 71-173`) | every executed step (`_save_live_belief`, M:6511) and Mark done; every monitor writes the same file; Reset moves it to `archive/` (`progress_io.py:489-497`) |
| OLD `<YYYYMMDD>-servoing/<label>_data_<ts>.json` and `_performance_<ts>.png` | per pass: tool0 errors, base drift, the bar reading (`_save_servoing_data`, `husky_world.py:2688-2712`) | each servo run |
| OLD `<YYYYMMDD>-asbuilt/as_built_<problem>.jsonl` | one line per MoCap bar at the start and the end of every step (`as_built_io.py:1-40`) | `_as_built_record` (M:10907), around every Exec (M:6575, 6589) |
| NEW `run.json` | `RunInfo` (`runs.py:171-185`): `process_hash`, `action_hashes`, `robots`, `commits`, `settings`, `continued_from`, `changed_since` | once, when the run starts (`save_info` l.400; `writes._started` l.168) |
| NEW `progress.json` | `actions{id: ActionRecord(status pending|done, seq, marked_at, assumed, ends)}`; `current{action, movement}`; `robots{live id: RobotRecord(base, joints (every movable joint), source live|assumed, at, after_action)}` (`runs.py:55-56, 116-168`) | Mark done, Reopen, a change of the current movement (`save_progress` l.415) |
| NEW `events.jsonl` | one line per `session`, `continue`, `new_run`, `settings`, `mark_done`, `reopen`, `mocap_offset`, `check_goal`, `check_tool0`, `plan`, `exec`, `servo`; numbers only (`record_event` l.429-447) | the event itself; every write goes through `request_record` (`assembly_run/plugin.py:246-263`) to the writer thread (`writes.py:48-72`) |
| NEW `movements/<action>/<movement>.<n>.json` | `basis`, `plan.rows`, `command`, `exec{ros_time_start, ros_time_end, base_at_start, base_at_end}`, `outcome` (`movement_exec/plugin.py:412-443`; `record_movement`, `runs.py:471-491`) | every part run (the `exec` event) |
| NEW `servo/<action>/<movement>.<n>.json` and `.png` | `servo_payload` (`servo_record.py:31`): the mode, the bar, every pass's reading, the moves, the outcome | every servo (`servo_part.py:362`, the `servo` event) |

## 6. Old term, new term

| Old | New |
|---|---|
| `ActionSchedule.json`, `PlanSkeleton` | `process.json` and its `schedule` |
| `holds`, `HoldWindow` | `bar_holding` actions with `supports_until` (§5 l.185) |
| `BarActions/B3__J.json`, `B3__R.json`; `action_id` `B3_J_joint`; `SkeletonAction.type` to kind `J`/`R`/`H`/`HR` (`plan_skeleton_io.py:52-57`) | `actions/B3_J.json`, `B3_R.json`; the id is the file name; `type` `bar_jointing`, `bar_release`, `bar_holding`, `bar_holding_release`, `bar_placing` |
| jointing roles M0-M5, release M0-M3 (`bar_action.py:359-394`) | no roles: the parts of each movement, classified live as a `Routine` (`free_to_load`, `transfer_held`, `insert`, ...) |
| `MovementKind`: `dual_free`, `dual_constrained_free`, `dual_constrained_linear`, `dual_independent_linear`, `single_free`, `single_linear`, `gripper_tool`, `scaffolding_tool`, `manual` | derived from `arms`, `path`, `coupled`, `controller`, `drives`, `ends_on` (§5.1 l.192-196) |
| `COMPLIANT_KINDS` (`bar_action_io.py:505`) | `controller: compliant` |
| `controller` `joint_tracking` / `cartesian_compliant` / `none` | `controller` `position` / `compliant` / absent |
| `ManualMovement` | a movement with no `arms` and no tool part, `ends_on: operator` |
| `ScaffoldingToolMovement`, `GripperToolMovement` (`tool_action`, `tool_names`, `overlaps_next`) | `target.tools`, `drives`, `ends_on: tools`; the overlap is one combined movement |
| `start_state` (`RobotCellState`) | `start` (`State`) |
| `target_ee_frames` | `target.robots[robot].link_poses` |
| `target_configuration` | `target.robots[robot].joints` |
| `trajectory` | `solutions/<action>.json` offline; `_route` and `movements/<action>/<movement>.<n>.json` live |
| `cycle_start_ee_sources` (`bar_action_io.py:156-212`): a movement starts at the previous authored target | `authored_start` (`servo.py:487-497`): the nearest earlier movement's `link_poses` |
| `_backfill_m0_target_from_m1` | `assumed_start_all` (`carry.py:31`): for drawing and seeding only, `null` joints take the own target, else the next known start |
| `.live-solved.json` sidecar | none: nothing is written into the process; the run folder keeps what was planned and run |
| `progress.json` per problem, shared by the three monitors | `runs/<run>/progress.json` per run |
| `ActionStatus(status, marked_at, marked_by_robot, run_id)` | `ActionRecord(status, seq, marked_at, assumed, ends)` |
| `RobotBelief(base_frame, configuration, source, at, after_entry)` | `RobotRecord(base, joints, source, at, after_action)`; `RobotBelief(base, joints)` lives on as the planning belief (`scene_for.py`, `beliefs_of`) |
| `current_index` | `current {action, movement}` |
| `skeleton_fingerprint` | `process_hash` and `action_hashes` in `run.json` |
| `LIVE_BELIEF_MAX_AGE_S` = 1 s (M:286) | `LIVE_BASE_AGE` 1 s, `LIVE_JOINTS_AGE` 0.5 s (`runs.py:58-59`); `is_live` (`assembly_run_contract.py:96`) |
| `_carry_configuration_forward`, `_propagate_live_end` | nothing carried: `MotionPlanRequest.snapshot` for the acting robot, `beliefs_of` for the others; `Carry` for the drawing |
| `_accept_trajectory` | `_route` and `PlanBasis` |
| IK Live Base (`ik_live_base_for_selected_movement`; `goal_arm_pose`, `goal_base_pose`) | Live IK (`check_goal`; `StepGoal`, `GoalCheck`) |
| the servo loop's tool0 readout (`measure_servo_tool0_error`) | Check tool0 (`check_tool0`) |
| `OperatorStep(primary, absorbed)` | `OperatorStep(primary, with_motion, after_motion, skipped)` |
| `_transfer_as_free_move` (the panel box) | the process plugin's "Transfer workaround" box; `steps.as_run`; `PlanBasis.transfer_as_free` |
| `<YYYYMMDD>-servoing/` | `runs/<run>/servo/<action>/<movement>.<n>.json` |
| `<YYYYMMDD>-asbuilt/` | not ported |
| Reset plan skeleton (to `archive/`) | New run (the old run kept) |

## 7. Known mismatches

- `process_format.md` names the run folder `runs/<yyyy-mm-dd_hhmmss>/` (l.59, l.440); the code writes
  `runs/<YYYYMMDD_HHMMSS>/` (`runs.py:4`; `run_record.md` §1).
- `process_format.md` §3.2 (l.117) says `run.json` keeps the hash of each solution; `RunInfo` (`runs.py:171-185`) has
  `process_hash` and `action_hashes` only.
