# 2026-09-30 — Support robots in the live monitor (ActionSchedule-driven, one robot per run)

Master spec for implementer / reviewer subagents. The approved high-level plan (context, ground
truth, locked decisions, exporter-defect note) is in
`/home/yijiangh/.claude/plans/i-am-about-to-clever-lerdorf.md` — read its "Ground truth" and
"Decisions locked with the user" sections first. This file holds the DETAILED design per work package.

## 0. Rules for every implementer

- Repo root: `/home/yijiangh/Code/ros2_ws/src/husky-assembly-teleop` (package dir `husky_assembly_teleop/`).
  Branch `yh/mocap_bar_reaching_acc_test`, HEAD `a602bb7` (Su's split J/R commit, fast-forwarded) + WP0 edits.
- **Line numbers in this spec are approximate** (some were read at c71d611, before the fast-forward).
  Always locate code by function / symbol name with grep, never by line number alone.
- Only edit the files your WP lists. Do not reformat unrelated code. Do not touch `external/` submodules.
- Style (CLAUDE.md): simplest possible change, reuse existing functions; google-style docstrings with type
  hints on every new/changed function; imports at the top of the file, `from x import fn` style; plain-language
  comments (no jargon like "no-op"), Better Comments markers `# *` (important), `# !` (warning), `# ?` (question)
  to highlight and divide sections.
- Data fixture (LOCAL, fast — never use `~/gdrive-shared`, it is a slow rclone mount):
  `FIXTURE_ROOT = "/home/yijiangh/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/2025-03 Husky Assembly/data_design_study/260920_RobArch_demo_revamp_backup"`
  Tests may open ONLY `ActionSchedule.json` and `BarActions/*.json` (≤ 1.3 MB each). **Never load `RobotCell*.json`
  in pytest** (340 MB each). Tests that need the fixture use `pytest.mark.skipif(not os.path.isfile(...))`.
  Tests that write must copy the needed files into `tmp_path` — never write into the fixture folder.
- Environment for running:
  ```bash
  cd /home/yijiangh/Code/ros2_ws && source venv/bin/activate && source install/setup.bash
  # (build/husky_assembly_teleop/husky_assembly_teleop is a symlink to the source dir, so new modules import without a rebuild)
  export DESIGN_DATA_DIRECTORY="/home/yijiangh/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/2025-03 Husky Assembly/data_design_study"
  export DESIGN_PROBLEM_NAME=260920_RobArch_demo_revamp_backup
  python -m pytest src/husky-assembly-teleop/test/<file>.py -q -p no:cacheprovider
  ```
  (`__init__.py` now reads `DESIGN_DATA_DIRECTORY`, `EXPERIMENT_DATA_DIRECTORY`, `DESIGN_PROBLEM_NAME` from the env.)
  Do not run the ament linters (`test_flake8/pep257/copyright`) as a gate. Colcon build (only when asked):
  `python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop` from `ros2_ws` inside the venv.
- Legacy problems (no `ActionSchedule.json`, e.g. `260929_phase1_retest`, the accuracy-test data) must keep
  working exactly as today through the existing `load_bar_action_file` / `load_action_cycle` path.

## 1. Fixture facts (verified)

- `ActionSchedule.json`: `schema_version 1`, `robots {Cindy:{robot_id:'dual-arm_husky_Cindy',role:'assembly'},
  Alice:{robot_id:'single-arm_husky_Alice',role:'support'}, Belle:{...support}}`, `assembly_seq` (20 bars:
  B1,B3,B4,B5,B7,B8,B9,B10,B12,B13,B15,B16,B17,B18,B19,B20,B21,B22,B24,B23), `holds` [{B3,Alice,B9},{B7,Belle,B9},
  {B12,Alice,B21},{B15,Belle,B21}], `schedule` 48 entries `{index, action_id, type, bar_id, robot, file}` with
  `file` like `BarActions/B3__H.json`. Entries 0..8: `B1_J, B1_R, B3_J, B3_H(Alice), B3_R, B4_J, B4_R, B5_J, B5_R`.
  Alice's entries: 3, 16, 21, 40. Belle's: 10, 17, 26, 41. Entry 16 = `B3_HR`.
- JSON dtypes: `rs_data_structure.bar_action/BarAssemblyJointingAction`, `.../BarAssemblyReleaseAction`,
  `rs_data_structure.hold_action/BarHoldingAction`, `.../BarHoldingReleaseAction`. Load with the existing
  `bar_action_io.parse_bar_action` (compas `json_load`).
- `B3__J` movements: `J_M0 IndependentDualArmFree (joint_tracking, start conf None)`, `J_M1 Manual (none)`,
  `J_M2 ScaffoldingTool grasp (AT3L/AT3R)`, `J_M3 EEConstrainedDualArmFree` (targets left/right),
  `J_M4 ScaffoldingTool tighten overlaps_next=True`, `J_M5 EEConstrainedDualArmLinear cartesian_compliant`.
  `B3__R`: `R_M0 Scaffolding untighten`, `R_M1 Scaffolding ungrasp`, `R_M2 IndependentDualArmLinear joint_tracking`
  (targets left/right), `R_M3 IndependentDualArmFree` (target conf only).
  `B3__H` (robot_id `single-arm_husky_Alice`, `supported_until ['B4','B9']`): `H_M0 SingleArmFree` (target `arm` +
  6-joint `target_configuration`, start conf None), `H_M1 Gripper open (SupportGripper)`, `H_M2 SingleArmLinear`
  (target `arm`, target conf, start conf = M0 target), `H_M3 Gripper close` (no target conf).
  `B3__HR` (`released_after_bar_id B9`): `HR_M0 Gripper open`, `HR_M1 SingleArmLinear` (target `arm`).
- Cindy states: 12 joints `left_ur_arm_*`, `right_ur_arm_*`; tools `AT3L`, `AT3R`, `ObstacleRobotAlice`,
  `ObstacleRobotBelle`; bodies `bar_*`/`joint_*`. Support states: 6 joints `ur_arm_*`; tools `SupportGripper`
  (attached to group `manipulator`), `ObstacleRobotCindy` (12-joint cfg), other support robot; bodies
  `env_bar_*`/`env_joint_*`. Parked obstacle = frame (50,50,0) + zeros.
- Oracle: `belief_after(B3__H)` for Alice (base ≈ (-4.5075, 1.7620, -0.0156), conf ≈
  [1.035, -1.612, 2.393, -0.793, 0.331, 0.012]) equals `B4__J` movement 0's `tool_states['ObstacleRobotAlice']`.
  `B3__R` parks Alice (exporter defect D1) — the monitor's belief must override it.
- Pitfall: `movement_role('B3_H_M2_LM_to_grasp') == 'M2'` today (legacy regex fallback).

## 2. WP1 — data / schedule layer (pure python; no ROS, no PyBullet)

Files: NEW `husky_assembly_teleop/robot_registry.py`, `husky_assembly_teleop/schedule_io.py`,
`husky_assembly_teleop/progress_io.py`; EDIT `husky_assembly_teleop/bar_action_io.py`;
NEW tests `test/test_robot_registry.py`, `test/test_schedule_io.py`, `test/test_progress_io.py`;
EXTEND `test/test_bar_action_io.py`. Nothing else.

### 2.1 `robot_registry.py`
Imports only stdlib + `from husky_assembly_teleop import DATA_DIRECTORY` (no compas/ROS/pybullet).
```python
@dataclass(frozen=True)
class RobotSpec:
    name: str                 # 'Cindy'
    robot_id: str             # 'dual-arm_husky_Cindy'  (Action.robot_id / ActionSchedule robots{}.robot_id)
    role: str                 # 'assembly' | 'support'
    namespace: str            # '/a200_0806'
    domain_id: str            # '86' (str: env vars are str)
    mocap_id: int             # 1860
    dual_arm: bool
    cell_file: str            # 'RobotCell.json' | 'RobotCell_Alice.json' | 'RobotCell_Belle.json'
    planning_groups: tuple    # ('base_left_arm_manipulator','base_right_arm_manipulator') | ('manipulator',)
    side_keys: tuple          # ('left','right') | ('arm',)   -- keys of Movement.target_ee_frames
    arm_joint_names: tuple    # one 6-tuple per side, same order as side_keys
    flange_links: tuple       # ('left_ur_arm_tool0','right_ur_arm_tool0') | ('ur_arm_tool0',)
    arm_base_links: tuple     # ('left_ur_arm_base_link','right_ur_arm_base_link') | ('ur_arm_base_link',)
    tool_names: tuple         # ('AT3L','AT3R') | ('SupportGripper',)
    obstacle_tool_name: str   # 'ObstacleRobotCindy'
    gripper_kind: str         # 'scaffolding' | 'robotiq'
    ee_types: tuple           # husky_world.init defaults: ('assembly_tool_v3_left','assembly_tool_v3_right') | ('robotiq_gripper',)
    connect_gripper: bool     # husky_world.init: False for Cindy, True for Alice/Belle
    urdf_path: str            # calibrated URDF (strings currently in cfab_session.py HUSKY_*_URDF_PATH(S))
    srdf_path: str
    rb_prefix: str            # '' (Cindy cell: bar_*) | 'env_' (support cells: env_bar_*)
    # properties / helpers
    serial -> str             # '0806' = namespace.rsplit('_',1)[-1]
    all_arm_joint_names -> list   # flat, side order (12 or 6)
    n_arms -> int
    side_of_link(link) -> str      # 'left_ur_arm_tool0'->'left', 'ur_arm_tool0'->'arm'; KeyError otherwise
    group_for_side(side) -> str; flange_for_side(side) -> str
    base_calibration_filename(convention='rotated') -> str   # 'calibrated_transformation_0806.json' | '..._0806_rhino.json' (see husky_world.init)
ROBOTS: dict[str, RobotSpec]   # Cindy, Alice, Belle — build with two small private factories, no 3x copy-paste
robot_by_name / robot_by_id / robot_by_namespace / robot_by_domain_id(int|str) / robot_by_serial -> RobotSpec  (KeyError listing valid keys)
robot_from_env(default_domain_id='86', environ=os.environ) -> (RobotSpec, was_default: bool)   # unknown/missing domain -> Cindy, True
other_robots(name) -> list[RobotSpec]      # stable order
robot_name_from_id(robot_id) -> str        # 'single-arm_husky_Alice' -> 'Alice'
```
Values: namespaces/domains/mocap ids/ee_types/connect_gripper from `husky_world.init`'s `ROBOT_CONFIGS` (Alice
`/a200_0804` 84 1840, Belle `/a200_0805` 85 1850, Cindy `/a200_0806` 86 1860). Joint names reference the existing
lists in `utils.py` (`HUSKY_DUAL_UR5e_JOINT_NAMES`, `UR5E_JOINT_NAMES` or equivalents — grep) rather than copying,
IF importing `utils` does not pull in pybullet at import time; if it does, write the names out (the registry must
stay import-light). URDF/SRDF path strings come from `cfab_session.py` (`HUSKY_DUAL_URDF_PATH`, `HUSKY_SINGLE_URDF_PATHS`
etc.); after this WP, `cfab_session.py` does NOT change yet (WP2 switches it to read from the registry).

### 2.2 `bar_action_io.py` additions (keep every existing function; a602bb7 added several)
- Paths: `LIVE_SOLVED_TAG='live-solved'`; `clean_action_path(path)` (public alias of `_clean_action_path`, keep the old
  name working); `sidecar_action_path(path, tag=LIVE_SOLVED_TAG)` ('B3__J.json' or 'B3__J.live-solved.json' ->
  'B3__J.live-solved.json', idempotent); `preferred_action_path(path, tag=...)` (sidecar if it exists on disk else clean);
  `write_path_for(path)` = pure-path version of `HuskyMonitor._bar_action_write_path` (clean export -> sidecar; an already
  tagged path -> itself). Do NOT edit the monitor in this WP.
- Kinds:
  ```python
  class MovementKind(str, Enum):
      DUAL_FREE='dual_free'; DUAL_CONSTRAINED_FREE='dual_constrained_free'; DUAL_CONSTRAINED_LINEAR='dual_constrained_linear'
      DUAL_INDEPENDENT_LINEAR='dual_independent_linear'; SINGLE_FREE='single_free'; SINGLE_LINEAR='single_linear'
      GRIPPER_TOOL='gripper_tool'; SCAFFOLDING_TOOL='scaffolding_tool'; MANUAL='manual'
  movement_kind(mv) -> MovementKind        # exact-class table; GripperToolMovement/ScaffoldingToolMovement before ToolMovement; TypeError on unknown
  DUAL_ARM_KINDS, SINGLE_ARM_KINDS, ARM_KINDS, STATIONARY_KINDS (frozensets)
  kind_fits_robot(kind, dual_arm) -> bool  # DUAL_* only dual, SINGLE_* only single, tool/manual both
  step_kind(mv) -> str                     # 'arm' | 'gripper' | 'scaffold' | 'manual'  (UI button dispatch)
  movement_controller(mv, role=None) -> str   # returns mv.controller; when role given (Cindy) compare with
                                              # {'M2': 'cartesian_compliant'} else 'joint_tracking' and print ONE warning on disagreement
  tool_event(mv) -> (tool_action, tool_names, overlaps_next)
  TRAJECTORY_TIME_BY_KIND_S = {DUAL_FREE:30.0, DUAL_CONSTRAINED_FREE:10.0, DUAL_CONSTRAINED_LINEAR:5.0,
                               DUAL_INDEPENDENT_LINEAR:5.0, SINGLE_FREE:15.0, SINGLE_LINEAR:5.0}
  default_trajectory_time(mv, role=None, role_table=None) -> float | None   # role_table (the monitor's MOVEMENT_TRAJECTORY_TIME_S) wins when role given
  ```
- **Hold-id guard**: at the top of `movement_role(mv)`: `if re.search(r'_HR?_M[0-9]_', movement_id): return None`.
  `CINDY_ACTION_TYPES = (BarAssemblyJointingAction, BarAssemblyReleaseAction, BarAssemblyAction)`;
  `roles_for_action(action) -> list` = `cycle_roles([(action, None)])` for Cindy actions else `[None]*n`; add the same
  isinstance guard per slot inside `cycle_roles` so its id-vs-class mismatch print never fires for H/HR.
- `cycle_start_ee_sources(movements, side_keys=('left','right'))`: only change = carried dict keyed by `side_keys`.
- Rigid-body naming: `BUILT_ASSEMBLY_RB_PREFIXES = ('bar_','joint_','env_bar_','env_joint_')`,
  `is_built_assembly_body(name)` (False for `obstacle_*`), `bar_body_name(bar_id, rb_prefix='')`,
  `find_bar_body(rb_names, bar_id) -> str|None` (tries 'bar_<id>' and 'env_bar_<id>'), `bar_id_of_body(name) -> str|None`.
  (Monitor keeps its own constant until WP3 switches the import.)

### 2.3 `schedule_io.py`
```python
SCHEDULE_FILENAME='ActionSchedule.json'; KIND_BY_TYPE={'BarAssemblyJointingAction':'J','BarAssemblyReleaseAction':'R',
  'BarHoldingAction':'H','BarHoldingReleaseAction':'HR'}; PREDECESSOR_KIND={'R':'J','HR':'H'}
@dataclass(frozen=True) class ScheduleEntry(index:int, action_id:str, type:str, bar_id:str, robot:str, file:str)
    kind -> str (from type, never from ids); is_support -> bool; from_dict / to_dict
@dataclass(frozen=True) class HoldWindow(bar_id, robot, release_after_bar_id, hold_start_seq, release_after_seq,
                                         hold_entry_index, release_entry_index: int|None)
@dataclass class ActionSchedule(problem_root, schema_version, robots, assembly_seq, holds, entries)
    problem_name; entry(i); entries_for_robot(robot); entries_for_bar(bar); find_entry(bar, kind); predecessor(entry)
    seq_position(bar); hold_window(bar); hold_end_index(hold)  # HR index if present else (R index of release_after_bar)+1
    robots_holding_at(entry) -> {robot: bar}   # hold_entry_index < entry.index < hold_end_index
    held_bar_ids(hold) -> list                 # assembly_seq[hold_start_seq : release_after_seq+1]
    executable_by(robot) -> list[ScheduleEntry]   # KeyError on unknown robot
    action_path(entry, prefer_sidecar=True) -> str   # normpath(join(problem_root, entry.file)) then preferred_action_path
    load_action(entry, prefer_sidecar=True)          # parse_bar_action + checks type name == entry.type and robot matches (ValueError)
is_executable_by(entry, robot_name) -> bool          # entry.robot == robot_name
schedule_path(problem_root); problem_root(problem_name, design_dir=DESIGN_DATA_DIRECTORY)
load_schedule(problem_root) -> ActionSchedule | None # None when the file does not exist (legacy problem); validates otherwise
@dataclass class LoadedEntry(entry, action, path, spec, predecessor: 'LoadedEntry'|None, roles, kinds, start_ee_sources)
    movements; start_ee_source(idx, side) -> Movement|None; start_ee_frames(idx) -> {side: Frame}; missing_start_sides(idx)
load_entry(schedule, entry, *, prefer_sidecar=True, with_predecessor=True) -> LoadedEntry
    # spec = robot_by_name(entry.robot); pred = predecessor loaded with with_predecessor=False (must be same robot_id);
    # sources = cycle_start_ee_sources(pred.movements + action.movements, spec.side_keys)[len(pred.movements):]
```
Expected: `load_entry(entry 4 = B3_R).start_ee_source(2,'left').movement_id == 'B3_J_M5_LM_insert'` (also for idx 0,1);
`load_entry(entry 16 = B3_HR).start_ee_source(1,'arm').movement_id == 'B3_H_M2_LM_to_grasp'` (also idx 0);
`load_entry(entry 3).start_ee_source(0,'arm') is None`, `(2,'arm').movement_id == 'B3_H_M0_free_to_approach'`.
Hold rule on the fixture: Alice holds B3 during entries 4..15, not at 3 (she is the actor) and not at 16 (releasing);
`held_bar_ids(B3) == ['B3','B4','B5','B7','B8','B9']`.

### 2.4 `progress_io.py`
`<problem_root>/progress.json`, schema v1:
```json
{"schema_version":1,"problem":"260920_RobArch_demo_revamp_backup","schedule_fingerprint":"48:<sha1 of action_ids>",
 "updated_at":"...","current_index":4,
 "entries":{"0":{"status":"done","marked_at":"...","marked_by_robot":"Cindy","run_id":"Cindy-20260930T135501"}},
 "robots":{"Alice":{"base_frame":{compas Frame data},"configuration":{compas Configuration data},
                    "source":"action_end_state","at":"...","after_entry":3}},
 "holds":{"B3":{"robot":"Alice","state":"holding","since_index":3,"released_at_index":null}}}
```
```python
STATUS_PENDING/DONE/SKIPPED; BELIEF_LIVE='live', BELIEF_ACTION_END='action_end_state', BELIEF_ASSUMED='assumed',
BELIEF_EXPORTED='exported', BELIEF_PARKED='parked'; HOLD_PENDING/HOLDING/RELEASED
PARKED_BASE_FRAME = Frame((50.0, 50.0, 0.0), (1,0,0), (0,1,0))
@dataclass EntryStatus(status='pending', marked_at=None, marked_by_robot=None, run_id=None)
@dataclass RobotBelief(base_frame: Frame, configuration: Configuration, source: str, at: str, after_entry: int|None=None)
@dataclass HoldState(bar_id, robot, state, since_index=None, released_at_index=None)
@dataclass class Progress(problem, schema_version, schedule_fingerprint, current_index, updated_at, entries, robots, holds)
    status(i); is_done(i); next_pending_index() -> int|None; mark_done(entry, robot, run_id, belief=None)
    # mark_done: status done + stamps; belief -> robots[entry.robot] with after_entry=index; H -> holds[bar]=holding(since=index);
    #            HR -> the hold of that bar = released(released_at_index=index); current_index = next_pending_index() (or len)
    skip(entry, robot, run_id); reopen(entry)   # reopen: pending; H -> hold pending; HR -> holding; current_index=min(current, index);
                                                 # drop robots[entry.robot] if its after_entry == entry.index
    set_belief(robot, belief); belief(robot); hold_state(bar); holding_bars() -> {robot: bar_id} (state holding)
    released_robots() -> set; to_dict(); from_dict()
progress_path(root); schedule_fingerprint(schedule); new_progress(schedule)
load_progress(root, schedule) -> Progress   # missing -> new; problem mismatch -> ValueError; fingerprint mismatch -> one printed warning, keep statuses
save_progress(progress, root) -> str        # compas json_dumps(pretty) -> path+'.tmp' -> os.replace (pattern dashboard/run_writer.py write_run)
new_run_id(robot, now=None); now_iso()
arm_configuration(conf, spec) -> Configuration   # only the arm joints, in spec.all_arm_joint_names order
belief_after(action, spec, entry_index=None, source=BELIEF_ACTION_END) -> RobotBelief
    # last movement: target_configuration or else start_state.robot_configuration; base = its start_state.robot_base_frame; ValueError if missing
belief_from_live(spec, base_pose, arm_joint_values, entry_index=None) -> RobotBelief   # base_pose=(pos, quat_xyzw); one 6-list per arm
belief_from_exported(action, spec) -> RobotBelief|None   # movement 0 start_state.tool_states[spec.obstacle_tool_name] (frame, configuration)
parked_belief(spec) -> RobotBelief
recompute_belief(progress, schedule, robot) -> RobotBelief|None   # from that robot's last DONE entry
obstacle_tool_states(progress, active_robot, exported_action=None) -> {obstacle_tool_name: (Frame, Configuration)}
    # for each other robot: released (its hold released and no later H) -> parked; elif belief -> belief;
    # elif exported tool_state in exported_action -> exported; else parked
obstacle_sources(progress, active_robot, exported_action=None) -> {obstacle_tool_name: source}
```
Frame/Configuration serialization: `compas.data.json_dumps/json_loads` (compas 2.15) round-trip them nested in dicts.
Use `from compas_robots import Configuration` (check the installed import path; compas_fab re-exports may differ).

### 2.5 Tests for WP1 (all must pass)
`test_robot_registry.py`: table, lookups (int/str domain), `robot_from_env` (85 -> Belle False; {} -> Cindy True; '99' -> Cindy True),
side/link helpers, calibration filename, `other_robots`, `robot_name_from_id`, Alice/Belle urdf paths differ and exist.
`test_bar_action_io.py` (extend, keep Su's tests): hold-id guard (H/HR ids -> None; J/R ids unchanged), `movement_kind` for
every class (build bare instances), `kind_fits_robot`, `step_kind`, `movement_controller` warning (capsys), `default_trajectory_time`,
path helpers with `tmp_path`, RB naming helpers, `cycle_start_ee_sources(side_keys=('arm',))` carry/clear,
`roles_for_action` on real B3__J/B3__R (`['M0',None,None,'M1',None,'M2']`, `[None,None,'M3','M4']`) and B3__H/HR (all None, nothing printed).
`test_schedule_io.py`: see 2.3 expectations + synthetic schedule without HR (fallback end index), sidecar preference in a tmp copy,
type mismatch -> ValueError, `load_schedule(tmp_dir_without_file) is None`.
`test_progress_io.py`: new/round-trip/atomic save (no .tmp left), problem mismatch, fingerprint warning, mark/reopen/skip/hold states,
`belief_after(B3__H, Alice)` == B4__J exported Alice tool_state (joint names 6 `ur_arm_*`, values and base point ≈ 1e-6),
`belief_after(B3__R, Cindy)` 12 joints, `belief_from_live`, `belief_from_exported(B3__R, Alice)` point (50,50,0),
`parked_belief`, `obstacle_tool_states(progress_after_entry3_done, 'Cindy', exported_action=B3__R)` Alice from belief not parked,
Belle parked, sources map; after marking entry 16 done Alice -> parked; `recompute_belief`.

## 3. WP2a — `HuskyRobotInterface` instance state + gripper feedback (parallel with WP1)

Files: `husky_assembly_teleop/husky_robot.py`, `husky_assembly_teleop/husky_monitor.py` (ONLY the `is_arm_executing`
bool-assignment bug fix), `scripts/headless_live_monitor_test.py` (ONLY extend its stub interface with the new attributes).

- Move every class-level mutable attribute of `HuskyRobotInterface` into a new `_init_state(self, n_arms: int)` called first
  in `__init__`: `position, rotation, velocity, angular_velocity, odom_offset, _odom_position, _last_mocap_data,
  _velocity_samples, _angular_velocity_samples, _velocity_samples_time` and per-arm lists sized `n_arms` (2 if dual_arm else 1):
  `arm_joint_pose (UR5e_HOME_STATE.copy() each), arm_tcp_pose, arm_ft_sensor, is_arm_executing, last_arm_movement, io_states,
  active_controller, controller_switch_error, gripper_states, screw_states, scaffolding_status`, plus NEW
  `gripper_feedback, gripper_result, gripper_goal_handle` (`[None]*n`). Delete the class-level block and the dual-arm
  `.append` block. Keep true constants (e.g. `velocity_filter_time`, deprecation flags) at class level.
- New kwarg `connect_ros: bool = True`: when False, return right after `_init_state` (viz-only husky: no subscriptions,
  publishers, service or action clients). Confirm `mocap_callback` and anything the monitor's `update()` reads work without ROS.
  Also set `self.n_arms`.
- Gripper feedback: `send_gripper_cmd(pos, effort, index=0) -> bool`: clear `gripper_feedback[index]`/`gripper_result[index]`,
  `send_goal_async(goal, feedback_callback=lambda m: self._on_gripper_feedback(index, m))`, add a done-callback
  `_on_gripper_goal_response(index, fut)` that stores the goal handle and chains `get_result_async()` ->
  `_on_gripper_result(index, fut)`. Store plain dicts: feedback `{position, effort, stalled, reached_goal, t}`; result
  `{position, effort, stalled, reached_goal, status, t}` (control_msgs GripperCommand feedback/result fields; `fb_msg.feedback.*`,
  `result.result.*`). Return False (with a log) when no gripper action client exists. Follow the existing
  goal_response/get_result callback pattern in the same file.
- `husky_monitor.py`: the two lines that do `hi.is_arm_executing = True/False` (fake-hardware single-arm path, grep
  `is_arm_executing =`) must index the list: `hi.is_arm_executing[<arm index>] = ...`.
- Do not touch the dead helpers (`set_screw`, `tighten_tool`, `ScaffoldingToolClient`) except to add a `# ! broken, do not use`
  comment above each.
- Verify: `python -c "from husky_assembly_teleop.husky_robot import HuskyRobotInterface"` in the venv; a tiny script that builds
  two instances with `connect_ros=False` (node=None allowed in that path) — one dual, one single — and asserts their lists are
  distinct objects with lengths 2 and 1, and that `mocap_callback` updates only the one called.

## 4. WP2b + WP3 — multi-robot world, one CfabSession per connected robot, obstacle beliefs, single-arm planning

Files: `husky_world.py`, `husky_monitor.py`, `cfab_session.py`, `common.py`, `utils.py`. Depends on WP1 + WP2a.

WP2b (world):
- `common.Husky` and `husky_world.create_husky_with_end_effectors`: pass `connect_ros` through.
- `husky_world.init`: replace `ROBOT_CONFIGS` with `robot_registry`; `spec, was_default = robot_from_env()` is the connected
  robot (keep the existing warn when defaulted). Create a Husky for EVERY registry robot, connected FIRST (so it is
  `huskies[0]` and `monitor.selected_robot_id = 0` keeps all existing uses valid). Per robot: `connect_ros=connected`,
  `connect_arm=connected and not FAKE_HARDWARE`, `connect_gripper=connected and spec.connect_gripper and not FAKE_HARDWARE`,
  `connect_compliant_controller=connected and CONNECT_COMPLIANT_CONTROLLER and not FAKE_HARDWARE` (match how the flags are
  combined today), `dual_arm=spec.dual_arm`, `mocap_id=spec.mocap_id`, ee_types from spec (punch/calibration overrides only
  for the connected robot). Factor the base-calibration-file lookup into `_base_calibration_file_for(monitor, spec)` and call
  it per robot (a missing file for a viz-only robot = warn and continue). Set `monitor.connected_robot = spec` and
  `monitor.husky_by_name = {spec.name: husky}`. Assert `monitor.huskies[0].name == spec.namespace`. Seed the viz-only huskies'
  `arm_joint_pose` / base from `progress_io` beliefs when the schedule exists (else `UR5e_HOME_STATE`, base stays at origin until
  mocap arrives). `build_default_robot_cell` stays for the connected robot only.
- A `common.load_robot` fix: pick the URDF from the registry by the husky's namespace/serial (today every single arm loads Alice's).
- `husky_monitor.update()` draw loop: only `i == self.selected_robot_id` falls back to `goal_base_pose` when untracked; others
  always draw at `(hi.position, hi.rotation)`.
- `cfab_session.py`: `CfabSession.__init__(problem_name, *, cell_filename='RobotCell.json', ...)` replaces the literal
  `"RobotCell.json"`; store `self.cell_filename`. URDF/SRDF path constants read from the registry (keep the module-level names
  as aliases so other imports keep working). Expose `inject_ground_rigid_body_state(cell, state)` as a module function (the
  monitor's `_inject_ground_rigid_body_state` calls it).
- Monitor: wherever the session is (re)created for a problem (`load_bar_action_file`), recreate when `problem_name` OR
  `cell_filename` differs and pass `cell_filename=self.connected_robot.cell_file` (fallback `'RobotCell.json'` when the attribute
  is missing, for the headless harness).
- `apply_obstacle_robot_beliefs(state, robot_cell, active_robot: str, beliefs: dict, holding_bars: dict | None = None)` in
  `cfab_session.py`: for each `{obstacle_tool_name: (Frame, Configuration|None)}` present in `state.tool_states` set `.frame`
  and merge the configuration BY JOINT NAME into `robot_cell.tool_models[name].zero_configuration()` (never by order);
  for `holding_bars {robot_name: bar_id}` add the robot's obstacle tool name to that bar body's `touch_bodies` (probe
  `bar_<id>` and `env_bar_<id>` via `bar_action_io.find_bar_body`). Mirrors Rhino `configure_robot_obstacle` /
  `whitelist_frozen_contact`.
- Monitor `_obstacle_beliefs(self) -> dict` = `progress_io.obstacle_tool_states(self._progress, self.connected_robot.name,
  exported_action=self._loaded_action)` (empty dict when there is no schedule/progress) overlaid with the LIVE mocap base of
  any other husky that is currently tracked (its name is in the mocap rigid-body cache); when a live base is used for a robot
  with no belief, use its home configuration (UR5e_HOME_STATE / dual home 12). Call `apply_obstacle_robot_beliefs` in
  `_apply_live_base_to_movement` right before `set_robot_cell_state`, and in the per-movement load path so the GUI shows
  re-posed obstacles (after `_inject_ground_rigid_body_state`). Log once per load which source won per obstacle robot
  (`progress_io.obstacle_sources`).
- Generalize Cindy assumptions (grep each): `_bridge_cfab_to_pp_for_bar_action` (loop over `connected_robot.flange_links`,
  one ghost sphere + attachment each); `_arm_joint_name_sets` single-arm branch -> `connected_robot.planning_groups[0]`;
  `load_selected_movement` goal-pose extraction via `_arm_joint_name_sets()` instead of `HUSKY_DUAL_UR5e_JOINT_NAMES`;
  `_fill_missing_start_conf` -> `UR5e_HOME_STATE` for one arm; `get_movement_start_bar_pose` side via
  `connected_robot.side_of_link(link)`; monitor `BUILT_ASSEMBLY_RB_PREFIXES` imported from `bar_action_io`; the two
  `active_bar_name = f"bar_{...}"` sites -> `find_bar_body(state.rigid_body_states, bar_id) or bar_body_name(bar_id, rb_prefix)`;
  the `startswith(BUILT_ASSEMBLY_RB_PREFIXES)` filter -> `is_built_assembly_body`; live-conf injection rule at load: movement 0
  with `start_state.robot_configuration is None` OR role 'M0' (both in the file loader and `load_selected_movement`);
  `_match_movement_role` fallback stays (now safe because of the hold-id guard).

WP3 (single-arm planning):
- `utils.joint_trajectory_from_path(path, joint_names=None)` and `path_from_joint_trajectory(jt, joint_names=None)`
  generalizing the existing 12-name helpers (default = today's behaviour; keep old function names working).
- `cfab_session.plan_linear_motion(planner, start_state, target_frame, *, group, max_step_distance=0.005,
  max_step_angle=0.05, max_jump_revolute=0.35, check_collision=True, verbose=False) -> JointTrajectory | None`:
  `FrameWaypoints([target_frame], target_mode=TargetMode.ROBOT, tolerance_position=1e-3, tolerance_orientation=1e-3)` (equal
  tolerances: compas_fab pybullet_plan_cartesian_motion.py passes tolerance_orientation as the position tolerance);
  `planner.plan_cartesian_motion(wp, start_state, group, options={...})`; catch compas_fab `MP*` exceptions and
  `PlanningGroupNotSupported` -> log `str(e)`, return None. `points[0]` is the start configuration.
- Free single-arm: reuse `cfab_session.plan_free_motion(planner, start_state, mv.target_configuration, group=...)`.
- `_accept_trajectory`: when `get_active_arm_count() == 1`: `mv.trajectory = jt`; merge `path[0]` / `path[-1]` BY NAME into this
  movement's and the next movement's `start_state.robot_configuration`; `planned_arm_trajectory = [(np.asarray(path6), None, t, None),
  (None, None, None, None)]`; skip the Cindy M-role chain rules. Dual-arm path unchanged.
- `plan_selected_movement`: when the role is None dispatch on `movement_kind`: `SINGLE_FREE` -> live-conf start + `plan_free_motion`
  to `target_configuration`; `SINGLE_LINEAR` -> `plan_linear_motion` to `target_ee_frames['arm']` from the movement start state
  (start conf = previous movement's end, already chained); tool/manual kinds -> info "nothing to plan for <kind>". Both use the
  live base (`_apply_live_base_to_movement`) and obstacle beliefs. Trajectory time from `default_trajectory_time(mv, role,
  MOVEMENT_TRAJECTORY_TIME_S)`.
- Group-generic live-base IK: `husky_world.solve_goal_ik_generic(planner, start_state, targets: {group: Frame}, *, seed_confs=(),
  check_collision=True, verbose=False) -> Configuration | None` built from the existing per-group solve + merge in
  `_solve_bar_action_goal_ik` (keep that as a thin Cindy wrapper with its seeds/visualization). `ik_live_base_for_selected_movement`:
  sides from `connected_robot.side_keys`, targets from `_start_ee_source`/target frames per side; write `goal_arm_pose[i]` per side.

## 4b. As built (WP2b + WP3 hand-back, for later WPs)

- Monitor helpers now available: `_connected_robot()` (RobotSpec with Cindy fallback for headless harnesses),
  `_cfab_needs_design_session()`, `_active_bar_body_name()`, `_movement_starts_live()`, `_obstacle_beliefs_with_sources()`,
  `_obstacle_beliefs()`, `_apply_obstacle_beliefs()`, `_plan_by_kind()`, `_merge_arm_values()`, `_accept_single_arm_trajectory()`,
  `_finish_action_load(action, path, loaded=None, *, slots=None)`; attribute `self._progress` (None until WP5 sets it).
  World: `create_registry_huskies(...)`, `_base_calibration_file_for(...)`, `_seed_viz_huskies_from_progress(...)`,
  `solve_goal_ik_generic(...)`. cfab_session: `inject_ground_rigid_body_state`, `apply_obstacle_robot_beliefs`,
  `plan_linear_motion`, `GROUND_TOUCH_LINKS`. Use `_connected_robot().n_arms` (not `.dual_arm` on the husky) to branch.
- Single-arm end configuration is carried through gripper/manual steps into the next ARM movement's start.
- The injected ground lists the ObstacleRobot tools in its `touch_bodies` (their wheels sit on the floor).
- Blockers found by WP2b/WP3 and the user's decisions:
  (1) compas_fab `forward_kinematics` multiplies by `robot_base_frame` a second time (pybullet_forward_kinematics.py:99-102)
      → DECIDED: scoped workaround in teleop. `cfab_session._world_frame_forward_kinematics(planner)` puts a corrected FK on
      the planner INSTANCE only while `plan_linear_motion` runs (verified: corrected flange == PyBullet link pose; B3__H
      H_M2 plans 22 points, end within 9.2e-4 rad of the target conf; no leak after the call).
  (2) exported H states use release-time geometry (future bars B4..B9 visible) with Cindy frozen at her assembled pose
      → H_M0/H_M2 start states collide (`ObstacleRobotCindy <-> env_bar_B5`). DECIDED: keep the exported scene as-is (no
      monitor-side visibility rule). With `BAR_ACTION_MOCAP_ACCURACY_TEST=1` the built assembly is hidden and these plan.
- ! Another robot's action can NOT be pushed into the connected robot's cell (joint names differ). WP5's display-only
  loading of a non-executable entry must NOT call `_finish_action_load` / `set_robot_cell_state`; show the entry's movement
  list and flags only, and allow Mark done.
- The DPG planned-joint plot has 12 series; 6-value single-arm rows are dropped → WP5 must size it by `n_arms`.
- `BAR_ACTION_MOCAP_ACCURACY_TEST=1` (default) hides the built assembly (now incl. `env_*` bars) from collision; the real
  assembly run should set it to 0 — operator note, not a code change.

## 5. WP4 — single-arm execution, gripper handoff, tool / manual steps

Files: `husky_world.py`, `husky_monitor.py`. Depends on WP2a + WP2b/WP3.
- `husky_world.execute_arm_trajectory_all(monitor, traj_time=None)`: generalize `execute_arm_trajectory_both` over
  `n = monitor.get_active_arm_count()` (`n==2` -> existing `send_dual_arm_cmd`; `n==1` -> `hi.send_arm_cmd(path, vel, t, index=0)`;
  fake-hardware branch loops `range(n)`); keep `execute_arm_trajectory_both = execute_arm_trajectory_all` alias.
- `move_arms_to_movement_start`: joint names from `monitor._arm_joint_name_sets()`, per-arm pi/3 guard, single-arm send.
- `_live_tool0_poses` / `execute_linear_cartesian_move`: link names from `monitor.connected_robot` (`flange_links`, `arm_base_links`).
- `switch_arm_controllers(monitor, to_ctrl, timeout_s=..., arm_indices=None)` = `switch_dual_arm_controller` over
  `range(get_active_arm_count())`; keep the old name as alias.
- `exec_selected_movement_traj`: slot checks over `range(n)`; before the role dispatch, `SINGLE_FREE/SINGLE_LINEAR` ->
  `execute_arm_trajectory_all` (warn if `mv.controller == 'cartesian_compliant'`).
- Generator `grip_with_compliant_handoff(monitor, close_pos=0.8, effort=0.1, switch_fraction=0.8, timeout_s=30.0, arm_index=0)`
  modeled on `execute_planned_trajectory_compliant`: (1) require `monitor.CONNECT_COMPLIANT_CONTROLLER` and a gripper client,
  else fall back to plain close + warn; (2) ensure `scaled_joint_trajectory_controller` active; (3) `start = feedback position`
  (or the open value 0.426 when unknown); send close goal; (4) poll `gripper_feedback[arm_index]` until
  `(pos - start) / (close_pos - start) >= switch_fraction` or the result arrives or timeout; (5) switch to
  `cartesian_compliance_controller` (abort handoff if the switch fails); (6) every tick publish the current tool pose as
  `target_frame` (`send_arm_cmd_cartesian`, reuse the FK / frame convention of `execute_linear_cartesian_move`) and a zero
  wrench (`send_arm_cmd_cartesian_force([0,0,0], arm_index)`); (7) poll `gripper_result[arm_index]` until `stalled or
  reached_goal` (ceiling `timeout_s`); (8) `finally`: switch back to `scaled_joint_trajectory_controller`, loud error if that fails.
  Log each phase with the controller name.
- `exec_gripper_tool_movement(monitor, mv)` generator: reset `monitor._servo_abort = False`; confirm gate
  (`wait_for_operator_confirm` with a message naming the action); `tool_action == 'open'` -> `open_gripper_full` + wait for the
  result (short timeout); `'close'` -> `grip_with_compliant_handoff` (or `close_gripper_for_bar` + warn without compliance).
- `run_manual_step(monitor, mv)` generator: reset `_servo_abort`; confirm gate with the movement tag/id.
- `run_scaffolding_tool_step(monitor, mv)` generator (Cindy): `overlaps_next=True` (J_M4 tighten) and `tool_action=='ungrasp'`
  (R_M1) -> mark-only: log "issued by the compliant insert/retreat execution" and return (the M2/M3 compliant flow already
  commands them — grep the tighten / loosen commands in `execute_planned_trajectory_compliant`). [AMENDED: `untighten` (R_M0) is
  mark-only too, user decision.] `grasp` (J_M2) -> confirm gate
  naming motor + direction -> `send_scaffolding_cmd(<same direction the manual 'Tighten Gripper' buttons use>, GRIPPER_MOTOR, i)`
  for every arm -> wait until `_scaffolding_gripper_motor_stalled(hi, i)` (new, mirror of `_scaffolding_joint_motor_stalled`
  reading `state_m1`) for all arms or timeout (`HOLD_FOR_STALL_TIMEOUT_S`) -> stop. `untighten` (R_M0) -> confirm gate stating
  "joint motor reverse for N s" -> `send_scaffolding_cmd(<loosen direction>, JOINT_MOTOR, i)` for `SCAFFOLD_UNTIGHTEN_S`
  (new monitor class constant, default 2.0, comment "tune on hardware") -> stop. Never call `tighten_tool/loosen_tool/set_screw`.
- Monitor `exec_selected_movement_step()`: dispatch by `step_kind(mv)`: gripper/manual/scaffold -> append the generator to
  `self.tasks`; arm -> `exec_selected_movement_traj()`. (The UI button is WP5.)

## 6. WP5 — schedule UI + operator flow

Files: NEW `husky_assembly_teleop/schedule_ui.py`, NEW `test/test_schedule_ui.py`; EDIT `husky_monitor.py`, `common.py`,
`ui_backend.py`. Depends on WP1..WP4.

- ! Amendments after WP2b/WP3/WP4 (these override anything below that disagrees):
  - **Schedule mode is only for COMPLETE schedules.** Legacy/accuracy-test problems also ship an `ActionSchedule.json`
    (e.g. `260929_phase1_retest`: 180 entries, only 16 action files exported). New monitor class flag `USE_ACTION_SCHEDULE = 1`;
    schedule mode iff the flag is on AND `load_schedule` returns a schedule AND every entry's file exists (clean export or
    sidecar). Otherwise keep the legacy file slider + `Load BarAction` exactly as today and log ONE line saying why
    (e.g. "ActionSchedule.json lists 180 entries but 164 files are missing -- using the legacy BarAction list").
  - **Other robots' entries are display-only and must NOT be pushed into the connected robot's cell** (joint names differ).
    `load_schedule_entry` on a non-executable entry: set `_loaded_entry` / `_loaded_entry_bundle`, print the movement list
    (`id [kind] ctrl=...`), update the readouts, and DO NOT call `_finish_action_load` / `set_robot_cell_state`; Plan/Exec/Step
    stay gated. Mark done works through the Confirm Exec pause with belief source 'assumed'.
  - The DPG planned-joint plot (grep the plot series creation in `build_ui` / the floating windows) must size its series by
    `self._connected_robot().n_arms * 6` so single-arm rows are drawn.
  - "Exec Selected Mv Traj (auto)" already refuses non-arm movements (fix pass); the new step button is the only way to run
    gripper / manual / scaffold steps.
  - Use `self._connected_robot()` (not `self.connected_robot`) for the connected RobotSpec (it has the headless fallback).
- DPG callbacks run inside `backend.step()` on the rclpy timer thread (no `manual_callback_management`), so callbacks may
  change monitor state and call `reset_ui`, but must not block — anything that waits is a generator appended to `self.tasks`.
- `schedule_ui.py` (pure, no DPG): `EntryFlags(n_arm, n_ik, n_traj, sidecar)`; `scan_entry_flags(action_path)` (plain
  `json.load`, count arm movements = dtype not Manual/Tool*, how many have `target_configuration` / `trajectory`);
  `step_button_label(mv) -> str|None` (arm -> None; gripper -> 'Exec gripper step: OPEN' | 'Exec gripper step: CLOSE +
  compliant handoff'; manual -> 'Operator done (manual step) -> then Confirm Exec'; scaffold -> 'Tool step: <action>' or,
  when overlaps_next / ungrasp, 'Mark tool step done (<action> runs with the next movement)');
  `entry_row_text(entry, flags, status, *, executable, selected)` e.g. `> [03] H   B3   Alice   IK 2/2  TRJ 0/2  done`
  (fixed widths, ' (other robot)' suffix when not executable); `now_line_text(entry, mv_idx, n_mv, mv)` e.g.
  `Now: entry 3 — H B3 by Alice — movement 3/4 B3_H_M2_LM_to_grasp [single_linear] ctrl=joint_tracking`;
  `visible_row_window(n, selected, size) -> (lo, hi)` ((48,0,12)->(0,12), (48,47,12)->(36,48), (48,20,12)->(14,26));
  `knobs_for_assembly_robot(entry, connected_spec) -> bool` ((None,Cindy) True, (None,Alice) False, (entry3,Cindy) False,
  (entry0,Cindy) True).
- Wrappers: `StatusText(name, default='', *, color=None)` -> backend `add_status_text(label, default, color=None)` (DPG uses it,
  PyBullet ignores); `Slider.set_value(v)` mirroring `TextInput.set_value`.
- Monitor state (init next to `_loaded_action`): `_schedule, _progress, _schedule_flags, _selected_entry_idx, _loaded_entry,
  _loaded_entry_bundle (LoadedEntry), schedule_entry_slider, schedule_entry_text, schedule_now_text, schedule_rows`.
  Every new method reads them with `getattr(self, name, default)` (the headless harness builds the monitor with `object.__new__`).
- `_load_schedule_state()` called in `__init__` right after `_load_available_bar_actions()`: `load_schedule(problem_dir)`; if a
  schedule exists: `load_progress`, `_selected_entry_idx = progress.current_index`, `_rescan_schedule_flags()`, print the roster.
- `build_ui`: inside the bar-action live-replan block, when `_schedule` is not None build `_build_schedule_section()` INSTEAD of
  the file slider + `Load BarAction` (legacy path unchanged otherwise): header `StatusText` (robot + domain, problem, progress
  k/n done), `Schedule entry (idx)` integer slider, `-> entry` readout, `Prev entry`, `Next entry`, `Load entry`,
  `Mark entry done`, `Reopen entry (reload clean)`, `Rescan schedule status`, `Now:` line, collapsible group of 12 row
  StatusTexts (grey = other robot, green = done, yellow = selected, default = pending). Under `Exec Selected Mv Traj (auto)`
  add ONE button labelled by `step_button_label(current_movement)` (only when not None) -> `exec_selected_movement_step`.
  Hide Cindy-only knobs (M1 anchor/manual sliders + Confirm/Derive/Adopt + adopt toggle, `Save M0 Plan`, `M2 split`,
  `M2 rigid only`) when `not knobs_for_assembly_robot(self._loaded_entry, self.connected_robot)`, setting their attributes to None.
- Methods: `_problem_dir`, `_load_schedule_state`, `_rescan_schedule_flags(indices=None)`, `_selected_schedule_index` (live
  slider read, same rule as `_slider_index`), `_refresh_schedule_readouts` (per tick, next to `_refresh_bar_action_readouts`),
  `select_prev_entry`, `select_next_entry` (clamp + `slider.set_value`, no rebuild), `load_schedule_entry(index=None)`,
  `_finish_action_load(action, path, loaded=None)` (the tail of `load_bar_action_file` factored out and shared by both loaders:
  cfab session for the connected robot's cell, `_loaded_action/_current_action_path/_loaded_action_slots=[(action,path)]`,
  `_loaded_movements`, roles/kinds/start-EE sources from the `LoadedEntry` when given else the legacy cycle functions,
  live-conf injection, ground body, obstacle beliefs, `reset_ui`, load movement 0), `mark_entry_done` (appends
  `_mark_entry_done_task()`), `reopen_entry`, `_entry_is_executable_here(entry=None)` (True in legacy mode), `exec_selected_movement_step`.
- `load_schedule_entry`: `entry = schedule.entry(idx)`; not executable here -> warn "entry k belongs to <robot>; loaded for display
  only — Plan/Exec are disabled"; `loaded = load_entry(schedule, entry)`; `_loaded_entry = entry`; `_finish_action_load(loaded.action,
  loaded.path, loaded)`.
- Gating: `plan_selected_movement`, `exec_selected_movement_traj`, `exec_selected_movement_step`, `plan_movement_chain_live`,
  `move arms to movement start`, and IK-live-base refuse with one warning when `not _entry_is_executable_here()`.
- `_mark_entry_done_task()` generator: refuse without a loaded entry; if not executable here -> confirm gate "Entry k belongs to
  <robot>. Confirm Exec to mark it done with the EXPORTED end state (belief 'assumed'); Cancel Exec to abort." -> belief =
  `belief_after(action, spec, idx, source=BELIEF_ASSUMED)`; if executable here -> belief = `belief_from_live(spec, live base
  (mocap-tracked) or the action's base, live arm joints, idx)`; `progress.mark_done(entry, connected.name, run_id, belief)`;
  `save_progress`; write the entry's live-solved sidecar only when the in-memory action carries planned trajectories
  (via the existing sidecar writer / `write_path_for`; the clean export is never overwritten); `_rescan_schedule_flags([idx])`;
  select `progress.current_index`; `reset_ui`.
- `reopen_entry()`: `progress.reopen(entry)`; save; reload the CLEAN export (`prefer_sidecar=False`) through `_finish_action_load`;
  sidecar stays on disk; rescan; rebuild.
- `test/test_schedule_ui.py`: flags on B1__J (n_arm 3, n_ik 2 — verify the real counts and pin them), B3__H, B1__R, a tmp sidecar with
  one trajectory; `step_kind` sequences for B1__J/B1__R/B3__H/B3__HR (`arm,manual,scaffold,arm,scaffold,arm` /
  `scaffold,scaffold,arm,arm` / `arm,gripper,arm,gripper` / `gripper,arm`); labels; row/now text; window; knob rule.

## 7. WP6 — manual smoke scripts

- `scripts/smoke_single_arm_plan.py`: DIRECT client; `CfabSession(problem, connection_type='direct', cell_filename='RobotCell_Alice.json')`
  (measured: 3.0 s load, 1.07 GB peak RSS from the local Insync copy; groups `base_arm_manipulator` + `manipulator`; `SupportGripper` frame = identity); `B3__H` movement 2 -> `inject_ground_rigid_body_state` -> `plan_linear_motion(..., group='manipulator')` (expect
  ≈ 20 points at 5 mm for the ≈ 0.10 m approach, end ≈ `target_configuration`); movement 0 from `UR5e_HOME_STATE` ->
  `plan_free_motion`; `apply_obstacle_robot_beliefs` + `check_collision` before/after. Print a PASS/FAIL summary.
- `scripts/headless_schedule_smoke.py`: `object.__new__(HuskyMonitor)` harness like `scripts/headless_live_monitor_test.py` with the stub
  interface extended (gripper feedback/result filled over ticks by a stub `send_gripper_cmd`, stub `switch_controller`, cartesian sends,
  scaffolding sends); scratch copy of the problem dir (copy only `ActionSchedule.json`, `WalkableGround.json`, `BarActions/`; point the
  cell load at the fixture cells read-only); Alice run: load entry 3, plan H_M0/H_M2, drive `grip_with_compliant_handoff` and assert the
  controller sequence joint -> compliance -> joint, mark done -> belief ≈ B3__H end; Cindy run: load entry 4 -> movement start state's
  `ObstacleRobotAlice` frame ≈ belief (not (50,50,0)) and == B4__J's exported frame; reopen 3.

## 8. Status

- [x] WP0 — FF `a602bb7`; `setup.py` pin 8912325; env-overridable data dirs + problem name; exporter note written to
      `bar_joint_rhino_design_workflow/docs/support_export_issues_from_monitor.md` (the repo is the Dropbox copy:
      `/home/yijiangh/Insync/yijiang94817@gmail.com/Dropbox/0_Projects/2025_husky_assembly/Code/bar_joint_rhino_design_workflow`).
- [x] WP1 — robot_registry / schedule_io / progress_io + bar_action_io additions; 74 tests pass. Review: PASS WITH FIXES (nits).
- [x] WP2a — per-instance HuskyRobotInterface state, connect_ros=False, gripper feedback/result. Review: should-fix = per-goal
      sequence guard in the gripper callbacks (applied in the fix pass), plus `server_is_ready()` check.
- [x] WP2b+WP3 — legacy Cindy flow verified identical; belief re-posing verified (Alice == B4__J export);
      linear planner blocked by the compas_fab FK bug (see §4b).
- [x] WP4 — execution / gripper handoff / tool & manual step generators; stub checks pass; legacy harness output identical.
      Review: FAIL → fixed. Compliant hold now publishes the tool0 pose CAPTURED AT THE SWITCH (both links read from one body
      at the live joints: 0.0 mm error for a 3 cm base offset), drift limit GRIP_HANDOFF_MAX_DRIFT_M=0.02, FK-vs-TCP 5 cm
      check before switching, F/T zero before the close, GeneratorExit-safe finally, late-ack switch-back, STOP before the
      grasp drive, `_servo_abort` reset after each step, "Exec Selected Mv Traj" refuses non-arm steps.
      User decision: R_M0 'untighten' is MARK-ONLY (joint motor never reversed; operator uses 'Loosen Joint' by hand).
- [x] WP5 — schedule panel, display-only entries, Mark done / Reopen / Rescan, step button, knob hiding, gating.
      Review: PASS WITH FIXES → fixed (one task at a time incl. the servo buttons; planned path cleared on entry /
      movement load; Reset All reloads the entry; Reopen targets the loaded entry, confirms when the slider shows another
      one, restores the previous belief; other huskies follow mocap only while seen; guard ignores the running task).
- [x] WP6 — `scripts/smoke_single_arm_plan.py` (7/7 required exported, 9/9 whitelist/off),
      `scripts/headless_schedule_smoke.py` (34/34, incl. Cindy's R entry: R_M2 12 pts, R_M3 75 pts with Alice from belief).
- Final state (2026-09-30): pytest 96 passed / 4 skipped (m1 dashboard needs a run on disk); legacy harness (B6 M2,
      B3__J M3) identical to baseline; colcon build OK. Operator manual: `doc/support_robot_schedule_manual.md`.
- Hardware-only checks and follow-ups: see the manual's "To confirm on hardware" and "Known limits".
