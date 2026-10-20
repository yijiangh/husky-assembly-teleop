# M5: the operator step panel and the executor, part 1 (spec)

The first milestone that moves robots. Adds `assembly_steps` (operator panel: current step, Check goal, Plan / Play /
Exec) and `movement_exec` part 1 (trajectories, tool steps, operator steps) — one movement as a cancellable task,
recorded into `runs/`, holding the robot exclusively while it works — plus Jakob's edits 1–3 and a 4th: a robot lease.
Routines covered: FREE_HOME, SUPPORT_APPROACH/LINEAR/LEAVE, TRANSFER_FREE (no servo), FREE_TO_LOAD; TOOL_ONLY (Robotiq
open/close, scaffolding grasp); OPERATOR. **Safety first:** every path to a motor is checked on the panel every tick AND
in the executor right after Confirm and immediately before sending.

Worktree `ros2_ws_next/src/husky-assembly-teleop`; old code at 0e45758: W = husky_world.py, M = husky_monitor.py,
R = husky_robot.py, BAI = bar_action_io.py, MAN = doc/three_robot_assembly_test_manual.md (line numbers verified).

## Assumptions (accepted as defaults; the user may revise before the lab day)
- A1 Plans start only from the acting robot's live state (`is_live`, shared/assembly_run.py:65); no planning ahead.
- A2 Every motion asks for Confirm (desktop or phone); the prompt names robot + serial, movement, duration, largest
  joint travel.
- A3 Robotiq uses the old values: open 0.426, close 0.8, effort 0.1, result timeout 5 s (W:2828-2833). (Jakob's
  `RobotiqGripper.open()` goes to 0.0, end_effectors.py:153 — check in the lab.)
- A4 A TOOL_ONLY Robotiq close in M5 is a plain stiff close (old `_plain_gripper_close`, W:6613); M6 replaces it with the
  compliant handoff for H_close. Air only.
- A5 "Another robot moved" = base > 1 cm or 1°, or any arm joint > 1°.
- A6 Dry run = a checkbox in movement_exec, off at start (plugins can't read ROS parameters).

## 1. Edits to Jakob's runtime
| # | File:line | Change |
|---|---|---|
| E1 | robot_interface/arm.py:642-643 | run `_cartesian_step_refusal` only when `not hold` (a hold targets the measured TCP, which can lag the last streamed target > 10 mm under compliance) |
| E2 | robot_interface/robot.py:230-261 | `send_multi_arm_trajectory(positions, duration, velocities: dict[str, Sequence[Sequence[float]]] \| None = None)`; keys must equal the arms; :251-252 pass `velocities[name]` to `build_trajectory` |
| E3 | shared/planning/panel.py:53-73, :255-266, :168-184 | `Search.work: Callable[[], PlanResult] \| None` + a new last field `run: Callable[[], Awaitable[PlanResult]] \| None = None` (exactly one set); `_run` awaits `search.run()` inside the same `timeout`, else the executor path as today; `_add_path_controls(…, commit_label: str = "Commit")` |
| E4 | robot.py after :100 | `@dataclass(frozen=True) class Lease(holder: str, label: str, since: float, stop: Callable[[], None])`; `HuskyRobotInterface.lease: Lease \| None = None`. Docstring: set only through `shared/execution/interlock.py`; Hold is always allowed; robot_interface does not enforce it |

There is no trajectory test mode (`cartesian_test_mode` covers Cartesian targets only; `send_joint_trajectory`,
arm.py:566, always publishes). The dry run calls `arm.build_trajectory` (arm.py:394): every `_check_trajectory` check
against live state, nothing published.

## 2. New shared modules (`husky_assembly_teleop/shared/`)
### 2.1 `execution/retime.py` (~170)
Each "run" (stretch between stops) gets Jakob's cosine profile (arm.py:154-220), sampled evenly in time; the arm glides
through gentle bends and stops only at bends that would break the acceleration limit (tamp's free paths are dense, so
stopping at every waypoint would be stop-and-go). Constants from arm.py: `JOINT_MOVE_MAX_SPEED` 20°/s (:99),
`JOINT_PATH_MAX_ACCELERATION` 60°/s² (:105, half the check's limit), `SPACING` = `JOINT_MOVE_WAYPOINT_SPACING` 0.2 s
(:103); new `SPLIT_ACCELERATION` = 1.5 × design acceleration.
```
class RetimeError(ValueError)
@dataclass(frozen=True) class Retimed: positions (M×D), velocities (M×D, last row 0), duration, minimum, stops; times -> np.ndarray
def retime(path, duration: float = 0.0, *, max_speed=…, max_acceleration=…, spacing=…) -> Retimed
def sampled_limits(positions, duration, velocities) -> tuple[float, float]   # exactly as arm.py:467-483
```
Algorithm: (1) drop repeated rows; refuse < 2 rows, NaN, outside `UR_JOINT_LIMITS`; (2) one run; arc length by the
max-norm of each segment (every joint's speed ≤ ṡ); (3) T_k = max(π/2·L/v, π·√(L/2a)) rounded up to whole SPACING
(arm.py:210-211); (4) sample run k at step i: φ = πi/n, s = L(1−cos φ)/2, ṡ = πL/(2T)·sin φ; position = the polyline at
s; velocity = outgoing segment direction × ṡ, 0 at run ends; runs share their boundary sample; (5) sampled acceleration
at interior samples (|Δ²q|/dt² and |Δv|/dt) > SPLIT_ACCELERATION → add a stop at the nearest path vertex; rebuild until
none (worst case = `joint_path_move`); (6) `minimum` = Σ T_k; an operator duration > minimum scales every run's step
count (only slows); (7) hard check against `TRAJECTORY_MAX_JOINT_SPEED/ACCELERATION/DURATION/WAYPOINTS` (arm.py:67-74),
last velocity 0, first/last rows exactly the path's ends → RetimeError (backstop).

### 2.2 `execution/send.py` (~200)
Constants: `ARRIVED_TOLERANCE = END_TOLERANCE = radians(0.5)` (robot_control/plugin.py:44), `ARRIVE_MARGIN = 5.0`
(:46), `STILL_SETTLE = 1.0`, `LARGE_TRAVEL = π/3` (old W:6257 guard → warning-prompt threshold).
```
@dataclass(frozen=True) class ArmMotion: arm; positions; velocities                     # UR_JOINT_NAMES order
@dataclass(frozen=True) class MotionCommand: serial; arms; duration; minimum; requested; travel; as_record()
def motion_command(serial, arm_names, joint_names, rows, traj_time) -> MotionCommand   # retime all columns together, split per arm by name (joints.py:20; arm_planner/plugin.py:51); ValueError if a joint belongs to no configured arm or an arm is only partly named
@dataclass(frozen=True) class SendOutcome: status (ARRIVED|REFUSED|TIMEOUT|STOPPED|DRY_RUN); reason; seconds; end; end_error; ok; as_record()
async def send_motion(ctx, robot: HuskyRobotInterface, command, *, dry_run=False) -> SendOutcome
```
Flow: dry run → `build_trajectory` per arm (any None → REFUSED) + banner "DRY RUN <serial> <arm>: passed every check,
NOT SENT" → DRY_RUN. Send: two arms → `robot.send_multi_arm_trajectory(…, velocities)` (E2); one →
`arm.send_joint_trajectory`; False → REFUSED. Wait `ctx.wait_until(timeout_s=duration+ARRIVE_MARGIN)` (as robot_control
`_execute_move`, plugin.py:466-494): arrived = duration passed and every arm within tolerance; stopped = > duration +
STILL_SETTLE, not moving, not arrived; joint state older than 0.5 s → one error "lost <serial> joint states: the arm runs
the trajectory it has; e-stop if needed". Endings: WaitTimeout → hold every arm, TIMEOUT; stopped → hold, STOPPED with
the remaining error; CancelledError → hold every arm, re-raise; else ARRIVED with the measured end and per-arm error.

### 2.3 `execution/tools.py` (~170)
Constants: `ROBOTIQ_OPEN 0.426`, `ROBOTIQ_CLOSE 0.8`, `ROBOTIQ_EFFORT 0.1`, `ROBOTIQ_TIMEOUT 5.0` (W:2828-2833),
`REACHED_TOLERANCE 0.02`, `GRASP_TIMEOUT 30.0` (W:4701), `CLEAR_TIMEOUT 2.0`, `STATUS_MAX_AGE 1.0`.
`ToolOutcome(status: reached|stalled|fell_short|timeout|refused|dry_run, reason, seconds, detail)`, `.ok`, `as_record()`.
- `robotiq_move(ctx, gripper, position, *, effort, timeout_s, dry_run)`: refuse if not `server_is_ready()`; refuse if
  `state.moving` (a late answer from an earlier goal would end this wait early — Jakob's `_on_result` :276 doesn't track
  goal numbers; old R:907 did); `move()` → `wait_until(not moving)`; timeout/cancel → `gripper.stop()` (cancel
  re-raises); "reached" if `last_result_ok` and within tolerance, else "fell_short" (old `must_reach_goal`, W:6369).
- `scaffolding_grasp(ctx, tools: Mapping[arm, ScaffoldingV3], *, timeout_s, dry_run)` (port W:6944-7054): refuse if any
  status missing or > 1 s old; `stop()` every tool, wait ≤ 2 s for a status newer than the stop that is not STALLED
  (`_drive` refuses while STALLED, end_effectors.py:348); `close()` every tool; wait ≤ 30 s until every tool reports
  STALLED in a status newer than the close; timeout → TIMEOUT "check the grip"; `finally` always `stop()` every tool
  (W:7045-7049). The M2 tighten is M6.

### 2.4 `execution/operator.py` (~80)
```
PROMPT_ARM_S = 0.5      # a Confirm sooner than this after the prompt opened is a double click: ignored
@dataclass(frozen=True) class Prompt: id; robot ("alice (a200-0804)"); text; warn; opened_at; line -> "robot: text"
class PromptBox:   # main thread only
    current: Prompt | None
    ask(robot, text, now, warn=False) -> Prompt   # drops any earlier answer (W:4846)
    answer(ok: bool, now) -> str                  # "" if accepted, else why ignored (no prompt / too soon)
    async wait(ctx, prompt) -> bool               # no timeout (mounting can take minutes); finally clears its own prompt
```

### 2.5 `execution/interlock.py` (~80) — the lock, on top of E4
| Function | What it does |
|---|---|
| `take(world, serial, holder, label, stop, now) -> Lease` | raises `Busy(reason)` if any robot has a lease, or any arm has `state.is_executing` with fresh joints (≤ 0.5 s; `is_executing` stays True when joint states stop, arm.py:835) → one motion at a time across all robots |
| `give_back(world, lease)` | clears only if the robot still holds this lease |
| `lease_of(robot)` / `busy_reason(world)` | queries |
| `refuse_if_leased(robot, what, log) -> bool` | logs "`<what>` refused: `<serial>` is held by `<holder>` (`<label>`); Cancel it first" |

### 2.6 `execution/basis.py` (~130) — what a plan was made from; when it goes stale
```
STALE_ARM = radians(2); STALE_BASE = (0.002 m, radians(0.5)); STALE_OTHER = (0.01 m, radians(1), joint radians(1))
@dataclass(frozen=True) class PlanBasis: loaded; action; movement_index; serial; names; start; base; others; transfer_as_free; ignore_built; statuses; time; as_record()
def basis_of(snapshot, loaded, step, serial, names, start, settings, statuses, ignore_built, now) -> PlanBasis
def stale_reason(basis, snapshot, *, loaded, current, transfer_as_free, ignore_built, statuses) -> str
```
Order: process reloaded (identity); current step changed; workaround / ignore-built changed; an action marked done or
reopened; acting robot missing; arm moved > 2°; base > 2 mm or 0.5° (`pose_error`, route_result.py:131); another robot
> 1 cm / 1° or joints > 1° (the reason names it).

### 2.7 `shared/movement_exec.py` (~150) — the API (as shared/assembly_run.py is for assembly_run)
- `DEFAULT_TRAJ_TIME` = {FREE_TO_LOAD 30 (BAI:526), TRANSFER_FREE 10 (BAI:527, MAN §4), FREE_HOME 10 (BAI:535),
  SUPPORT_APPROACH 15, SUPPORT_LEAVE 15 (BAI:530), SUPPORT_LINEAR 5 (BAI:531)}; `MAX_TRAJ_TIME = 90` (M:777); `MOTION`
  = the six motion routines; `LATER` = {INSERT: "M6", RETREAT_LINEAR: "M6", TRANSFER_HELD: "M7"}.
- `PartPlan(index, movement, routine, kind, tools, text)`, kind ∈ motion | robotiq_open | robotiq_close | grasp |
  operator | base | refused. `part_plan(loaded, action, index, robots, transfer_as_free)`: a TOOL_ONLY grip tool →
  its flange via `process.robots[robot].tools` → the arm; process `Tool.kind` must match `ArmConfig.end_effector`;
  refused: a scaffolding "open" outside a retreat (M6), scaffolding_v1, `with_motion`.
- `ExecRequest` (frozen): `loaded, action, step: OperatorStep, parts, serial, robot_name, route: RouteResult|None,
  route_movement, basis: PlanBasis|None, traj_time, label`. `ExecResult` (frozen): `status` (done|dry_run|refused|
  cancelled|failed), `reason`, `done`, `end`, `base`, `ok`.
- `MovementExecPluginApi(Protocol)`: `active -> str`, `prompt -> Prompt|None`, `dry_run -> bool`,
  `request_execute(r) -> asyncio.Future[ExecResult]`, `request_confirm()`, `request_cancel()`, `request_dry_run(on)`.
- `serial_of(loaded, action, robots)` via `find_robot_serial` (config.py:465).

## 3. `plugins/movement_exec/{__init__,plugin,sequences,panel}.py` — `requires=("motion_planner","assembly_run")`, experimental
- `request_execute` (main thread): refuse at once if busy, if `part_plan` refuses, or a wrong-robot guard fails (§7 #2);
  `interlock.take(…, stop=self.request_cancel)` (Busy → refused); spawn `_run(request)`; a done-callback safety net
  (task cancelled before start → resolve, give the lease back, clear the prompt).
- `_run`: each part through `sequences.run_part`; stop at the first non-ok; `finally`: give back the lease, clear the
  prompt, resolve the future.
- Records: per part `run.request_record("exec", action=…, movement=…, data={status}, file=payload)`; payload
  `{movement, routine, robot, serial, step, part, basis: basis.as_record(), plan: {planner, attempts, seconds,
  joint_names, rows, goal_joints, end_error, warnings}, command: MotionCommand/tool/prompt record, outcome: {…,
  dry_run}}`; events `prompt`, `confirm`, `cancel` (reason "operator" or "from outside (Stop all, a plugin stop or
  shutdown)").
- `request_cancel`: an open prompt → answer False; else a running task → reason "operator", `task.cancel()`; else log
  "nothing to cancel".
- Panel: chips (idle / `<robot> · <label>` / "waiting for Confirm" / orange "DRY RUN"), prompt line, [Confirm, Cancel],
  checkbox "Dry run (every check, nothing sent)", last result. Phone tab "Exec" (`ctx.view.phone`): status, prompt
  (`phone_text`), big Confirm/Cancel (`big_buttons`).

Before every motion part: robot `is_live`; `stale_reason` empty; `run.view.process is r.loaded` and the current action
is still `r.action`. Re-checked after Confirm; then the command is rebuilt with **row 0 = the live joints** (≤ 2° from
the planned start, no jump at t=0).

| Kind | Sequence |
|---|---|
| FREE_HOME, SUPPORT_APPROACH/LINEAR/LEAVE, TRANSFER_FREE | checks → prompt "`<robot>`: `<movement>` — move `<arms>` along the planned path, T s, largest joint travel d° (Play it first). Confirm to move." (warning prompt if d > 60°) → checks again → `send_motion`. After TRANSFER_FREE / SUPPORT_APPROACH log "servo correction: M7, not run"; after SUPPORT_LEAVE "drive the base away by hand" |
| FREE_TO_LOAD | as above; once ARRIVED (not dry run): wait 2 s (W:5781), `zero_ft_sensor()` on every arm (W:5807-5849) |
| robotiq_open | prompt "OPEN the gripper (0.426)" → `robotiq_move(ROBOTIQ_OPEN)`; fell short → warning |
| robotiq_close | warning prompt "STIFF close (the compliant handoff comes in M6): jaws empty / no bar held by another robot" → `robotiq_move(ROBOTIQ_CLOSE)` |
| grasp (scaffolding_v3 close) | prompt "GRASP: STOP, then the M1 screw of `<tools>` TIGHTENS until STALLED (≤30 s), then STOP" → `scaffolding_grasp` |
| operator / base | prompt "do `<movement label>` by hand (or drive the base there), then Confirm" → done when confirmed |
| refused | ExecResult refused: "`<routine>` runs in `<M6/M7>`" |
With the workaround on, a transfer step has parts `(3, 1, 2)`: motion, place_bar (operator), grasp; a Cancel at a
prompt gives `done=(3,)`.

## 4. `plugins/assembly_steps/{__init__,plugin,panel}.py` — PlannerPlugin subclass; `requires=("process","assembly_run","motion_planner","movement_exec")`, experimental
- Current step from `run.view.current` → `operator_steps(...)` with the run's workaround setting → `step_index_of`. On
  change: `_clear()`, default traj time, reset done parts, the Mark-done offer and the goal check. Workaround toggled and
  the current movement no longer a primary → `request_set_current(action, steps[i].primary)` (M4 open question 3).
- Panel rows: chips (action · robot (serial) · step k/n · plan state · movement_exec busy); step text (primary + "then:
  …" after-motion + "skipped: …"); [◀ Prev] [Next ▶] (`request_set_current`; refused while busy); [Check goal] (only
  for `IK_GOALS`, tamp_route.py:57); Path: Plan / Play / Clear + slider; **Exec** (`commit_label`, E3); "Exec moves:
  `<robot>` `<arms>`"; traj time slider 1–90 s with "min X s by the arm's limits"; checkboxes Transfer workaround
  (`run.request_settings`), Ignore built bars (`PlanRequest.ignore_built`, for air tests); [Mark `<action>` done] after
  the action's last step finished ok; message line.
- Plan: `make_search` → `Search(serial, …, work=None, run=…, time_limit=3×120+60+5)` (ATTEMPTS route_search.py:34, FAR
  tamp_route.py:46, panel WAIT_MARGIN). `run` refuses (failed PlanResult) when the step has no motion or the robot is
  not `is_live`; else `PlanRequest(view.process, step, snapshot, robots, beliefs, transfer_as_free, ignore_built,
  goal_joints=…)` → `await planner.request_plan` → `PlanResult(TimedPath(retime(rows, traj_time).times, positions) or
  None, reason, seconds)`. `goal_joints` reuses the last Check goal's result for the same step if the base moved < 2 mm /
  0.5° since (MAN §7.2). `accepted` keeps the route + `basis_of(...)`, records a "plan" event.
- Staleness: `stale_reason` = `basis.stale_reason(...)`. Preview: `show_path` poses the acting arms at `path.sample(t)`
  on the planning base; `show_target` at the Check goal's IK joints, else the authored goal; changing traj time retimes
  the preview.
- Check goal: port `GoalCheckPlugin.check` (goal_check.py:98-129) into a task; move `goal_line` (:39-61) to
  `shared/planning/route_result.py`; delete `plugins/goal_check.py` and `test/test_goal_check.py` (its tests move);
  update README (:280) and roadmap (:22, :68).
- Exec: override `_commit` (panel.py:284): a step without motion, or whose primary is already done → ExecRequest without
  a route; else require `plan_state == "ready"`; `exec.request_execute(...)`; on the future's done-callback: add the
  finished parts; when every part is done → `_clear()` → `request_set_current(next.primary)`, or the Mark-done offer
  after the last step; a dry run never advances.

## 5. Lock checks in other plugins
| File:line | Change |
|---|---|
| robot_control/plugin.py:135 | keep `self._ctx`; `_allowed(serial, what) -> bool` (`refuse_if_leased`) |
| :431 `_start_move`, :274 `_apply_force`, :233-234 FT Zero | check `_allowed` first |
| :288-302 base twist, :331 free drive | drop the hold / skip the switch while leased |
| :452 `_hold_arm` | after `arm.hold()`: if leased, `lease.stop()` (cancels the executor; its error handling holds every arm) |
| arm_panel.py:204-235, :237-262, :264-285; status.py:50-81 | new `allowed: Callable[[str], bool]`; wrap grip, screw, Robotiq target, reactivate, controller-switch callbacks |
| arm_planner/plugin.py:255, base_planner/plugin.py:354 | `send_plan` → (False, reason) while leased |
| assembly_run/plugin.py (`_mark_refusal`) | also refuse while any robot is leased |

## 6. Old → new (0e45758)
| Old | Where | New |
|---|---|---|
| `execute_arm_trajectory_all` (dual `send_dual_arm_cmd`, single `send_arm_cmd`) | W:2869-2909; R:1009-1112 (0.1 rad guard R:1035/1068; dt=T/(n−1) R:1040) | `send_motion` + E2; start guard arm.py:497 + the 2° basis check |
| wait by time, not `is_arm_executing` | W:5784-5804 | time AND 0.5° arrival |
| `execute_trajectory_and_zero_ft` | W:5807-5849 | FREE_TO_LOAD sequence |
| `move_arms_to_movement_start` (π/3 guard) | W:6257-6331 | not ported (plans start live); π/3 = warning-prompt threshold |
| support plain joint moves | M:6062-6066 | SUPPORT_*, FREE_HOME |
| dispatch by kind; refusal for stationary kinds | M:5872-6066, M:5920 | `part_plan`; a command exists only for MOTION parts |
| steps gripper / manual / scaffold | M:6069-6114; W:6871-6916, 6919-6941, 6944-7054 | robotiq / operator / grasp parts |
| gripper helpers + values | W:2826-2856, 6338-6406, 6613-6634 | tools.py |
| `send_scaffolding_cmd` | R:1172-1194 | `ScaffoldingV3.close/stop` (end_effectors.py:319-330) |
| `wait_for_operator_confirm` | W:4825-4862 | `PromptBox` |
| Confirm/Cancel; `_cancel_exec` | M:12897-12905; M:6534 | movement_exec panel + phone; Cancel now holds the arm |
| `_refuse_while_tasks_run` | M:11269-11294 | `active` + `interlock.take` |
| `_exec_then_propagate` (carry end if within 0.1 rad, M:5852) | M:6493-6532 | `ExecResult.end` + record; the next plan uses the live state |
| `_save_live_belief` | M:6449-6491 | not ported (one monitor sees every robot; beliefs at Mark done, M4) |
| default traj time; slider re-read at Exec | M:4072-4100, BAI:525-535, M:777; M:5941 | `DEFAULT_TRAJ_TIME`; stored via `defer_value`, read at Exec |
| traj preview | M:1000-1030, 1055, 12472 | PlannerPlugin Play/slider + path ghost |

## 7. Tasks
| # | Title | Files | Depends |
|---|---|---|---|
| A | E1–E4 | arm.py, robot.py, panel.py; test/test_runtime_edits.py | – |
| B | retime | execution/retime.py; test_execution_retime.py | – |
| C | operator, interlock, basis | execution/{operator,interlock,basis}.py; test_execution_basis.py | A |
| D | send | execution/send.py; test_execution_send.py; fakes in plugin_fakes.py | A, B |
| E | tools | execution/tools.py; test_execution_tools.py | – |
| F | lock checks in other plugins | robot_control/*, arm_planner, base_planner, assembly_run; test_robot_control_lease.py | C |
| G | movement_exec | shared/movement_exec.py, plugins/movement_exec/*; test_movement_exec.py | C, D, E |
| H | assembly_steps; remove goal_check | plugins/assembly_steps/*, route_result.py; test_assembly_steps.py | A, C, G |
| I | slow dry run, docs, spec status, full test run, monitor smoke | test_exec_dry_run.py, README, doc/plugin_roadmap.md | all |

## 8. Verification
Tests (quick unless marked; fakes via `ArmInterface.__new__` as test_base_send_guards.py:26):
| File | Asserts |
|---|---|
| test_execution_retime | 100 seeded paths (`connect()` in 6-D and 12-D with random obstacles, raw and densified to 0.1 rad) with random traj time 1–30 s: `_check_trajectory` → None for every arm; rows on the polyline (1e-9); exact ends; last velocity 0; duration ≥ traj time. A gently curved path has no interior stop; repeated rows dropped; a path that doesn't move or exceeds 300 s raises |
| test_runtime_edits | E1: under compliance a target 20 mm from the last one 0.1 s earlier is refused, but `hold()` is sent. E2: MultiArmTrajectory points carry the velocities. E3: a `Search.run` result → "ready"; a `run` that never answers → "timed out"; the `work` path unchanged. E4: lease defaults to None |
| test_execution_basis | each stale reason at its threshold (2.1° vs 1.9°; 3 mm; 1.1 cm; reload; step; workaround; statuses); `take` raises on any lease or fresh `is_executing`, ignores a stale one; Prompt: Confirm within 0.5 s ignored, an answer reaches only its own prompt, every line starts with the robot |
| test_execution_send | ARRIVED (fake arms follow); REFUSED sends nothing; TIMEOUT holds; cancel holds every arm and re-raises; STOPPED at 50% holds; two arms use the multi-arm call with velocities; dry run builds and publishes nothing; a joint-state gap logged once |
| test_execution_tools | Robotiq: not ready / moving → refused; reached; fell short; timeout → stop; cancel → stop. Grasp: STOP, wait for the stall to clear, close; both STALLED → ok; one never stalls → timeout + final stop; cancel stops; silent driver refused |
| test_movement_exec | second request "busy"; lease during a run, released after done / cancel / exception / cancel before start; wrong serial or wrong route movement refused, nothing sent; base moved 3 mm → refused; cancel at a prompt sends nothing; cancel during motion holds; one record per part (`json.dumps` works); workaround parts (3,1,2) with cancel at place_bar → done=(3,); dry run never gives done |
| test_assembly_steps | step walking (after-motion, skipped); workaround toggle re-homes the step; PlanRequest carries settings, beliefs, cached goal_joints; not live → no request; stale reasons on the panel; Exec ok → `request_set_current(next)`; last step → Mark done; the 3 ported goal_check tests. Slow: B20_J and B21_H walking on the purged process |
| test_robot_control_lease | while leased: Start, tool, twist refused; Hold calls `lease.stop` |
| test_exec_dry_run (slow) | real tamp plans on the purged process (B20_J_transfer with the workaround, B20_R_home, B21_H_approach; `design_scene` as test_motion_planner.py:175) → `motion_command` → `send_motion(dry_run=True)` on fake-node ArmInterfaces → DRY_RUN, every JointTrajectory has M points with zero final velocity, publishers empty |

Commands:
```
cd /home/yijiangh/Code/ros2_ws_next && source venv/bin/activate && source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop
cd src/husky-assembly-teleop
python3 -m pytest test -q -p no:cacheprovider
HUSKY_DRIVE_ROOT="/home/yijiangh/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/2025-03 Husky Assembly" python3 -m pytest test -q -p no:cacheprovider -m slow
python3 -m pytest test -q -p no:cacheprovider -m linter -k "flake8 or pep257"
env -u CYCLONEDDS_URI ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1 timeout 25 ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['alice','belle','cindy']" -p plugins:="['assembly_steps','robot_control']" -p viser_port:=8095
```
Headless chain before the lab (as M4): on a scratch copy of the purged process, real plugins with fake robots that follow
commands: plan B20_R_home, Exec, Confirm → `movements/B20_R/B20_R_home.1.json` with outcome ARRIVED.

Lab acceptance (monitor + visual steps only; robots on Zenoh; e-stops in hand; workspace clear; no bars):
1. `cd ~/Code/ros2_ws_next && source venv/bin/activate && source install/setup.bash && ros2 daemon stop && ros2 run
   husky_assembly_teleop husky_monitor --ros-args -p robots:="['alice','belle','cindy']"
   -p plugins:="['assembly_steps','robot_control']" -p process_directory:=<B20–B37 process>`; Load; assembly_run →
   New run. The movement_exec panel sits above assembly_steps; the phone shows an "Exec" tab.
   On every robot's teach pendant, check the speed slider is at 100 % (the log repeats it at every Exec): slower, a
   motion lags its timing, times out after duration + 5 s and is held mid-path.
2. Dry run: tick Dry run; click the B20_R row, Next ▶ to B20_R_home (tick Ignore built bars if the start collides);
   Plan, Play (the ghost goes home, clear of the floor); Exec; the phone prompt names "cindy (a200-0806)"; Confirm →
   "DRY RUN … NOT SENT" for both arms; nothing moves. Untick.
3. §6 by hand (Belle, then Alice): robot_control tool Open/Close; controller to compliance and back.
4. §6 gripper steps: B35_H row → B35_H_open, Exec, Confirm → opens; while that prompt is open, robot_control's Belle
   tool button is refused (log line). B35_H_close, Exec → STIFF warning prompt → Confirm → closes (~0.79). Repeat the
   close and Cancel during it → fingers stop. Open again. Same for Alice (B21_H). (Handoff in the air = M6.)
5. Alice approach without servo: drive Alice near the design base; B21_H_approach; Check goal (note "max |goal −
   authored|", look at the ghost); Plan, Play; traj time ≥ 15 s; Exec; with the phone and Alice's e-stop in hand read
   the prompt (alice, T, travel) and Confirm. Watch: the arm follows the preview, stops with open jaws ~100 mm short of
   where B21 would be; "arrived, end within …" and "servo: M7".
6. Cindy home: move her arms ~20° off with robot_control; B20_R_home: Plan, Play, traj time ≥ 10 s, Exec, Confirm with
   Cindy's e-stop → arrived, end ≤ 0.5°. Repeat and Cancel mid-motion → both arms stop within ~0.5 s ("hold sent").
7. Send back: the terminal log (`… 2>&1 | tee ~/m5_lab.log`), the `runs/<name>/` folder, pass/fail per step with any
   refusal lines, both grippers' end positions, Alice's goal line, Cindy's end-error lines.

## 9. Safety: every way M5 could move a robot unexpectedly
| # | Hazard | Guard |
|---|---|---|
| 1 | stale plan executed | panel stale check every tick (panel.py:348) with `basis.stale_reason`; Commit only from "ready" (:287); NEW: the executor re-checks after Confirm and just before sending; row 0 = live joints |
| 2 | wrong robot / a plan for one robot sent to another | NEW: `r.serial == serial_of(step)`, route robot == `step.action.robot`, route movement == the primary; basis base matches that robot's live base within 2 mm; joint names belong to that robot's arms (`motion_command`); the prompt names robot + serial |
| 3 | double click | one request at a time; `take` refuses; Confirm within 0.5 s ignored; an answer only reaches its own prompt id |
| 4 | base moved since planning | 2 mm / 0.5° at Plan and at send; the lease blocks the D-pad and base_planner; a command-line twist can't be blocked (manual rule) |
| 5 | cancel doesn't reach the robot | Cancel → `task.cancel()` → hold in the same tick (context.py:399, monitor.py:311-313); hold on stale state logs "HOLD NOT SENT … pendant" (arm.py:553); Stop all also stops through the safety sync (robot.py:175); e-stop in hand (MAN §4) |
| 6 | Zenoh/wifi drop mid-trajectory | the onboard JTC has the whole planned, swept-checked path; the monitor logs stale joints, waits duration+5 s → TIMEOUT + hold; M5 streams nothing |
| 7 | a manual command during an automatic motion | lease (E4) + §5 checks; Hold always works and cancels the executor |
| 8 | another robot already moving | `take` refuses on a fresh `is_executing` on any arm |
| 9 | too fast, or a jump | retime at half `_check_trajectory`'s limits + its own hard check; `_check_trajectory` (arm.py:425-500): controller active, sync operational, start within 0.1 rad, state ≤ 0.5 s old, not moved in the last 0.5 s |
| 10 | a leftover trajectory replayed (old M:5920) | a command exists only for a MOTION part of this request; the plan is cleared after Exec |
| 11 | a late gripper answer, a screw motor left running | refuse while `moving`; `finally` always stops the screw motors |

Risks (by severity): (1) a late trajectory after a timeout's hold if Zenoh reconnects late (5 s margin, lease kept until
then; e-stop); (2) outside the lease: command-line twists, a second monitor, the pendant (manual rules); (3) stiff
Robotiq close (air only until M6); (4) open position 0.426 vs Jakob's 0.0 (check in the lab); (5) stops on noisy linear
IK paths (test asserts a single run on a smooth path); (6) mocap noise vs the 2 mm threshold (spurious stale); (7) two
Confirm buttons (assembly_run's and movement_exec's prompts each answer only their own); (8) overriding the private
`_commit`.

## Deviations from the plan
1. E4 lease + lock checks in robot_control, arm_planner, base_planner, assembly_run (the plan said robot_control is
   reused unchanged).
2. New shared modules interlock.py, basis.py, shared/movement_exec.py.
3. Confirm before every motion; row 0 = live joints (the old Exec sent at once).
4. Cancel holds the arm (old: the trajectory finished).
5. Robotiq: stiff close in M5; a Robotiq cancel stops the fingers; the old 0.426 for open.
6. Not ported: preview ahead of the current step; `_save_live_belief`; TRANSFER_HELD, INSERT, RETREAT_LINEAR and a
   standalone scaffolding open are refused until M6/M7.
7. Dry run: a checkbox calling `build_trajectory`.
8. Soft stop recorded as a `cancel` event with its reason.
9. "Ignore built bars" checkbox for air tests.
10. Other-robot staleness: 1 cm and 1°, or 1° on its joints.

## Status (2026-10-20)

Tasks A–I done on `yh/core-port`: ab5d21c runtime edits E1–E4 (A), 8ec5934 retime (B), 298c74a operator, interlock,
basis (C), d7ee939 send (D), c2669a4 tools (E), db2afe0 lock checks in other plugins (F), ed87c3c movement_exec (G),
07c9065 assembly_steps, goal_check removed (H); ce2fbce more executor guard tests, d8ec86f a refused or dry-run Exec
gives the plan back, 30ec9cd the slow dry run, and this spec, README and roadmap with I.

Tests: default 457 → 553 passed, 2 skipped; `-m slow` 47 → 49 passed, 1 xpassed; flake8 and pep257 clean.
After the review's fixes (below): default 578 passed, 2 skipped; `-m slow` 49 passed, 1 xpassed. Files:
test_runtime_edits (4), test_execution_retime (19; 100 seeded paths and the real B20_R_home plan through
`_check_trajectory` and a dense check of the cubic the controller runs), test_execution_basis (26),
test_execution_send (13), test_execution_tools (10), test_robot_control_lease (8), test_movement_exec (27),
test_assembly_steps (16 + 1 slow: B21_H's four steps and B20_J's transfer step planned for real and run on fake robots
that follow), test_exec_dry_run (1 slow), lease tests in test_assembly_run; test_goal_check's tests moved into
test_assembly_steps. Shared fakes: `test/plugin_fakes.py` (the real `ArmInterface`, `HuskyRobotInterface`,
`RobotiqGripper` and `ScaffoldingV3` without ROS, and drivers that follow what they are sent), `test/robot_lab.py`
(the three robots live, and a fake run), `test/b20_r_home_plan.json` (a real plan's rows).

Monitor smoke (`env -u CYCLONEDDS_URI ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1`, only `ros2_ws_next` sourced,
`plugins:=['assembly_steps','robot_control']`): health, process, assembly_run, motion_planner, movement_exec,
assembly_steps (511 ms: its ghosts) and robot_control set up in that order (the movement_exec panel above
assembly_steps), first tick 24 ms, no "failed", clean teardown; the only warnings are the experimental plugins and the
robots' missing controller managers.

Headless chain (a scratch copy of the purged process outside the Drive; the real assembly_run, motion_planner,
movement_exec and assembly_steps plugins on one `FakeContext`; the robots at B20_R_home's design start, following what
they are sent): B20_R_home planned by `tamp.plan_free_dual_arm` in 1 try (32.7 s, 76 rows), timed to 95 samples,
18.8 s (also its minimum; the default 10 s is too fast for it); the prompt "cindy (a200-0806): B20_R_home — move
left_ur_arm, right_ur_arm along the planned path, 19 s, largest joint travel 228° …" (a warning, over 60°); Confirm →
one multi-arm message → ARRIVED, end within 0.03°; `runs/<run>/movements/B20_R/B20_R_home.1.json` (kind movement,
outcome ARRIVED); events new_run, plan, prompt, confirm, exec; the lease given back; no error line; the panel offers
Mark B20_R done.

### Safety (§9): each guard, where it is, and its test (after the review)

| # | Guard in code | Tests |
|---|---|---|
| 1 stale plan | `PlannerPlugin.update` → `AssemblyStepsPlugin.stale_reason` (assembly_steps/plugin.py:333) → `basis.stale_reason` (execution/basis.py:110, incl. the reused goal's base :147); Exec only from "ready" and not stale now (plugin.py:385, :388), then "sent"; a plan given back is checked first (`_executed`, :406); executor: `world_problem` before the prompt and after Confirm (movement_exec/sequences.py:132, :248) against the current Ignore built bars (`ExecRequest.ignore_built`), row 0 = the live joints, ≤ 2° from the plan's start (sequences.py:171-199) | test_execution_basis (thresholds), test_assembly_steps::test_a_plan_goes_stale_on_the_panel, ::test_exec_needs_a_ready_plan_and_uses_it_once, ::test_unticking_ignore_built_bars_with_exec_sends_nothing, ::test_unticking_ignore_built_bars_at_the_prompt_is_refused_after_confirm, ::test_a_plan_on_a_reused_goal_goes_stale_once_the_base_leaves_where_the_goal_was_solved, ::test_a_plan_given_back_is_checked_for_staleness_first, test_movement_exec::test_a_plan_gone_stale_at_the_prompt_is_refused_after_confirm, ::test_the_first_row_sent_is_where_the_arms_are |
| 2 wrong robot | `_refusal`: serial = `serial_of(step)` (movement_exec/plugin.py:171); `_plan_refusal`: route robot = the action's (:191), route movement = the primary (:193), basis for this robot and step, its base within 2 mm / 0.5° of the live one (:202), joint names of this robot's arms (`motion_command`, :206); `send_motion` refuses another serial (send.py:204); every prompt starts "<robot> (<serial>): " (sequences.py:206) | test_movement_exec::test_a_request_for_another_robot_or_movement_is_refused, ::test_a_plan_for_another_robot_is_refused, ::test_a_base_moved_3_mm_since_planning_is_refused_at_the_request, ::test_a_motion_is_confirmed_sent_arrives_and_is_recorded (prompt), test_execution_send::test_joints_that_are_not_the_robots_arms_are_refused, ::test_a_command_for_another_robot_is_refused |
| 3 double click | one request at a time (plugin.py:164), the lease (`take`, interlock.py:37; plugin.py:98), a Confirm within 0.5 s ignored and answers keyed by prompt id (execution/operator.py:78-97) | test_movement_exec::test_a_second_request_is_busy, test_execution_basis::test_one_lease_at_a_time, ::test_a_confirm_too_soon_is_ignored_and_answers_reach_only_their_prompt |
| 4 base moved | 2 mm / 0.5° in `stale_reason` (every tick, at Exec, after Confirm) and at the request (plugin.py:202); a base driving or following a path is "moving" (interlock.py:129); while any robot is held the D-pad (robot_control/plugin.py:330), base_planner's Commit (base_planner/plugin.py:358) and base_exp (base_exp/plugin.py:328, :379, :483) are refused | test_execution_basis::test_movements_at_their_thresholds, ::test_a_driving_base_is_busy, test_movement_exec::test_a_base_moved_3_mm_since_planning_is_refused_at_the_request, ::test_a_base_moved_3_mm_at_the_prompt_is_refused_after_confirm, test_robot_control_lease::test_the_dpad_and_free_drive_are_refused_while_held, ::test_the_planners_do_not_send_to_a_held_robot, ::test_base_exp_does_not_start_while_a_robot_is_held |
| 5 cancel | Cancel / Hold (via `lease.stop`) / Stop all cancel the task; `send_motion` holds every arm on CancelledError before re-raising and says whether each hold went out (send.py:269); a hold an arm refuses is the outcome HOLD_NOT_SENT, logged as an error, in the result, the record, the panel and the phone (send.py:254-308; movement_exec/plugin.py:247-290); a cancelled part is recorded with where the arms stopped, and Stop all with a soft_stop event (:265) | test_execution_send::test_a_cancel_holds_every_arm_and_is_raised, ::test_lost_joint_states_are_said_once_then_the_unsent_hold_is_the_outcome, test_movement_exec::test_a_cancel_during_the_motion_holds_every_arm, ::test_stop_all_mid_motion_holds_every_arm_and_is_recorded_as_a_soft_stop, ::test_a_hold_not_sent_on_cancel_is_the_result_record_and_panel_line, ::test_a_request_cancelled_before_it_starts_gives_the_robot_back, test_robot_control_lease::test_hold_is_sent_and_cancels_the_holder |
| 6 link lost | one error "lost <serial> joint states …" (send.py:243), lost joints never count as stopped, duration + 5 s → TIMEOUT, then the hold (refused on stale joints: HOLD_NOT_SENT) (send.py:267) | test_execution_send::test_lost_joint_states_are_said_once_then_the_unsent_hold_is_the_outcome, ::test_too_slow_times_out_and_holds_every_arm |
| 7 manual command | `Lease` (robot.py:56, :90); while ANY robot is held, `refuse_while_held` (interlock.py:150) refuses every motion command to every robot: robot_control Start / force / FT Zero / tool / screw / Robotiq Go and Reactivate / controller switches (plugin.py:294, :486; arm_panel.py:241-305; status.py:69), D-pad and free drive (plugin.py:330, :370), arm_planner / base_planner Commit (:259, :358), base_exp (:328, :379, :483), health Unlock / Resume / Reconnect (health.py:102), Mark done, row select, Reopen, settings, new run, continue (assembly_run/plugin.py, writes.py); Hold, the tools' Stop and a zero force always go out (plugin.py:291, :511). What sends every tick checks before every send: a robot taken meanwhile ends robot_control's Cartesian stream (hold, one line naming both robots) and free drive (back to the joint controller); the D-pad sends one stop | test_robot_control_lease (11: also every command to Alice while Cindy is held, the stream and free drive ended by a lease, Reconnect), test_assembly_run::test_a_robot_held_for_a_motion_refuses_the_mark, ::test_a_held_robot_refuses_selecting_reopening_and_settings, test_movement_exec::test_robot_control_cannot_move_another_robot_while_one_is_held |
| 8 another robot moving | `busy_reason` (interlock.py:81): any lease; an arm executing or moved slightly (0.001 rad or a reported joint speed, arm.py:99, :846) within 2 s; a base driving or following; an arm or base whose states went stale after being live. At `take`, before the prompt, after Confirm and just before sending (`others_moving`, sequences.py:161), right before a grasp's screws tighten, and every tick while the arms move (`send_motion`'s watch, send.py:236: INTERRUPTED, every arm held, in the tick the other robot starts). A base path sent just before a lease counts once the follower reports it or the wheels turn (base.py keeps no send time) | test_execution_basis::test_an_arm_running_moving_or_unknown_refuses_a_lease, ::test_a_slow_move_is_busy_from_its_start_to_a_second_after_its_end, ::test_an_arm_whose_joint_states_went_stale_is_busy, ::test_a_joint_velocity_or_a_millimetre_counts_as_moving, ::test_a_driving_base_is_busy, test_movement_exec::test_another_robot_moving_by_other_means_at_the_prompt_is_refused_after_confirm, ::test_another_robot_starting_to_move_during_the_motion_holds_the_held_robot, ::test_the_held_robot_is_held_in_the_tick_another_robot_starts_and_moves_no_further, ::test_another_robot_moving_at_a_robotiq_prompt_refuses_it_after_confirm, ::test_another_robot_moving_at_a_grasp_prompt_refuses_it_after_confirm, ::test_another_robot_moving_while_the_grasp_clears_its_stall_tightens_nothing, ::test_the_held_robots_own_base_driving_during_its_motion_holds_it, ::test_a_base_path_sent_just_before_the_lease_holds_the_held_robot_once_the_follower_reports_it, ::test_a_robot_whose_link_dropped_while_moving_refuses_the_request |
| 9 too fast / jump | `retime` (retime.py): velocities by finite differences; the cubic the controller runs between samples checked analytically, a bend that needs over SPLIT_ACCELERATION (90°/s², :30) or leaves the polyline by over CORNER_CUT (0.05°, :32) gets a stop; a run still over it with no bend left to stop at (a short move) gets one step longer at a time; its own hard check on the samples and the cubic; the arm's `_check_trajectory` on every send and every dry run | test_execution_retime (100 paths and the real B20_R_home plan through `_check_trajectory` and a dense independent cubic check; a fine spacing; short straight moves 0.05–40°; the hard check on the cubic), test_execution_send::test_a_refused_trajectory_sends_nothing, ::test_a_dry_run_runs_every_check_and_publishes_nothing, test_exec_dry_run (slow, real plans) |
| 10 leftover trajectory | a command is built only in a motion part, from this request's route (sequences.py:171); the plan goes "sent" at Exec (assembly_steps/plugin.py:399) and is cleared after it ran, or given back (checked) only when nothing was sent (`_executed`, :406) | test_assembly_steps::test_exec_needs_a_ready_plan_and_uses_it_once, test_movement_exec::test_robotiq_and_refused_parts (a tool part publishes no trajectory) |
| 11 gripper / screw | Robotiq refused while `moving` (tools.py:90), stopped on timeout and cancel (:100, :104); the grasp always stops every tool on the way out, refuses a silent driver, and asks the others-moving check again right before the screws tighten (`ready`) | test_execution_tools (11) |

Mutation check (each guard removed in turn in a scratch copy, the M5 tests run): 33 mutants of the guards above,
the review's twelve among them; every one fails a test (the retime acceleration check only at a fine spacing,
`test_at_a_fine_spacing_the_acceleration_stops_a_gentle_bend`: at 0.2 s the corner cut is always the stricter
check). Taking the velocities from the path instead of the samples is not caught, and is no guard: the checks measure
the cubic of whatever velocities are sent; that choice only changes how many stops a path needs.

### Where the code differs from this spec

1. **E4**: `HuskyRobotInterface.lease` is a class attribute (`None`), so robots built without `__init__` (tests) have it
   too; `Lease` sits after `RobotState` in robot.py.
2. **`busy_reason(world, now, besides=None)`** takes the time (freshness) and leaves out the robot a motion is for.
   After the review it counts as moving: an arm executing or whose joints moved slightly (0.001 rad, or a reported
   joint speed over 0.01 rad/s: a new `ArmState.last_slight_motion_time`, runtime edit E5 in arm.py) within 2 s; a
   base whose wheels turn or whose follower follows a path; an arm or base whose states went stale after being live
   this session (robots never measured are left out). So Exec right after any motion waits about 2 s ("… moved within
   the last 2 s"). `refuse_while_held(world, what, log)` replaces `refuse_if_leased(robot, …)`: one held robot locks
   every robot.
3. **`send_motion`** judges "stopped" by the last detected motion on fresh joint states (`last_motion_time` older
   than STILL_SETTLE), not by `is_executing`, for the same reason; lost joint states never count as stopped (they end
   in the timeout). `SendOutcome.ok` is ARRIVED or DRY_RUN. After the review: `watch` (another robot moving →
   INTERRUPTED), `progress` (`SendProgress`: where the arms were and the holds, for a cancel's record), `holds`, and
   HOLD_NOT_SENT when an arm refuses its hold.
4. **Robotiq**: FELL_SHORT is not ok: the part is not done (a warning says "check the fingers") and the request
   stops there. A grasp whose stall does not clear within 2 s after STOP ends TIMEOUT, nothing tightened.
5. **PromptBox**: a Cancel always counts (no 0.5 s arming) and wins over a Confirm not yet taken; a second Confirm on
   an answered prompt is ignored.
6. **shared/movement_exec.py**: kind constants `MOTION_PART`, `OPEN_PART`, `CLOSE_PART`, `GRASP_PART`,
   `BY_HAND_PART`, `BASE_PART`, `REFUSED_PART` (values as specified); `part_plan(loaded, action_id, …)`; added
   `step_parts` (a movement run during the motion is refused: M6), `current_step` and `current_primary` (the stale
   check compares the current step's primary, so a current movement that is not a primary still matches). A tool
   movement with a kept drive or without a grip, or with mixed tools or grips, is refused. `ExecRequest.action` is
   the id; `ExecResult.end` the acting arms' joints by name.
7. **movement_exec** checks the process, the current action and that nothing else moves before (and after Confirm
   for) every part, not only motions; live and stale only for motions; the robot stays held through a part done by
   hand (review L7: the panels stay locked, which keeps the run's step from moving under it; its prompt says so, and
   a base drive says to use the robot's joystick or the command line, or Cancel first). An unexpected error inside holds every arm, resolves FAILED, then
   raises (the monitor counts it). The dry run is fixed when a request starts; changing it while one runs is
   refused; its prompts start "DRY RUN — "; a part done by hand in a dry run counts as DRY_RUN. Every part is
   recorded, refused and cancelled ones too; its outcome adds `result`, `status`, `reason`, `dry_run`.
8. **assembly_steps**: an Exec refused before anything was sent, or a dry run, gives the plan back ("ready", still
   checked for staleness every tick); anything else clears it. §9 #10 is about replaying a sent trajectory, which this
   does not allow; without it a transient refusal (row 2) costs a replan (minutes for a transfer).
9. **assembly_steps**: a planned path that does not move (the robot already at its goal) says "the arms are already
   at the goal: nothing to move (Next ▶ goes on)"; its motion part can't be run. Found by the slow walk:
   B21_H_approach and B20_J_transfer have no authored start, so the design snapshot stands the robot at its own
   target; the walk and the dry-run test start such a robot with stowed arms (as the M3 check).
10. **assembly_steps** files: plugin.py 448 lines, the goal check in goal.py (`StepGoal`, `same_step`), panel.py.
    The check's IK joints show on PlannerPlugin's (yellow) target ghost; the dark-blue "goal" ghost kind of M4 is
    left in ui/ghost.py unused. `goal_line` is in route_result.py.
11. **robot_control**: the tools' Stop (grip Stop, screw Stop) and a zero force are never refused while a robot is
    held, like Hold: they move nothing. The D-pad and free-drive refusals are logged once per lease. plugin.py is
    590 lines (was 533), not split (Jakob's file, his newer commits on jg/viser-cleanup not merged yet).
13. **After the review**, besides the above: `PluginContext.closed` (read only) tells a task cancelled by its
    plugin's stop or shutdown from one cancelled by Stop all; `ExecRequest.ignore_built` reads the step panel's
    Ignore built bars now; `PlanBasis.goal_base`; the retime velocities are the samples' finite differences and the
    cubic between samples is what is checked (`executed_limits`); ExecResult and SendOutcome statuses INTERRUPTED
    and HOLD_NOT_SENT; health's Unlock / Resume, base_exp and assembly_run's row select, Reopen, settings, new run
    and continue refused while a robot is held.
12. Tests touched outside M5's files: test_phone (robot_control now needs its context in `update`),
    test_base_planner_commit (its fake robot has a `lease`), test_assembly_run (its fake robots too, and a lease
    test).

### Review (request changes) and its fixes

| Finding | Commit | What changed |
|---|---|---|
| H1 another robot moving during a held motion | fc3ab3a, bf7f5c7 | one held robot locks every robot's motion commands (robot_control incl. phone, planners, base_exp, health Unlock/Resume); the executor checks every other robot after Confirm and right before sending, and watches them while its arms move (INTERRUPTED, every arm held) |
| M1 moving arms missed | fc3ab3a | slight motion (0.001 rad, joint speed) within 2 s, stale states after live and bases count as moving |
| M2 Ignore built bars not re-checked | 66583aa | checked against the setting now at Exec and after Confirm; the prompt says "(built bars ignored)" |
| M3 a refused hold reported held | 51ae025 | HOLD_NOT_SENT in the outcome, result, record, panel and phone; logged as an error |
| M4 the controller's cubic off the limits | c7bc2bb, 114a998 | finite-difference velocities; the cubic checked analytically (≤ 90°/s²) and against the polyline (≤ 0.05°); the real B20_R_home plan: 250 → 57°/s², 0.138° → 0.036° off (two stops more, 18.8 → 19.2 s) |
| L1 a raising part unrecorded | 51ae025 | recorded (failed, the error) with an error event; every arm held |
| L2 a cancelled motion's record | 51ae025 | where the arms stopped, seconds, holds; Stop all told from a plugin stop or shutdown, a soft_stop event |
| L3 base_exp, health | fc3ab3a | refused while a robot is held |
| L4 the reused goal's base | 0d47f02 | the plan goes stale 2 mm / 0.5° from where its goal was solved |
| L5 a plan given back stale | 0d47f02 | checked before it is "ready" again |
| L6 the panel losing done parts | 0d47f02 | assembly_run refuses row select, Reopen, settings, new run and continue while a robot is held |
| L7 a base step with the D-pad locked | 0d47f02 | the robot stays held; the prompt says to drive with the joystick or the command line, or Cancel first |
| L8 the pendant's speed slider | 0d47f02 | logged at every motion's Exec; lab note below |
| L9 Force Zero | fc3ab3a | never refused |
| test gaps | 6206603 | every movement_exec scenario fails after 30 s instead of hanging; the request-time and after-Confirm base checks each tested with their own wording |

Every finding's reproduction is a test that failed before its fix (run before each fix: M1 4, H1 2, M2 2, M3/L1/L2
import, M4 5, L4–L8 5 failing). Re-run after the fixes: default 578 passed, 2 skipped; `-m slow` 49 passed, 1 xpassed
(one earlier run failed test_exec_dry_run once: tamp's unseeded free search for B20_R_home swept through the ground
halves near its start in all three tries; it passed 3 of 3 alone, and the test now seeds `random` and `np.random`
as the M3 check does); flake8 and pep257 clean; monitor smoke (domain 70, localhost only) as before, no "failed", clean teardown; headless
chain: B20_R_home planned (78 rows, 14.0 s), timed to 20.2 s, ARRIVED, end within 0.09°, lease given back, no error.

**Lab manual note (L8):** before any Exec, the teach pendant's speed slider must be at 100 %: slower, a motion runs
behind its timing, times out after duration + 5 s and is held mid-path.

### Re-review (approve with nits) and its fixes

| Nit | Commit | What changed |
|---|---|---|
| F1 watch timing | f7ba444 | test: the held robot is held in the tick another robot starts, and moves no further |
| F2 commanding X while Y is held | f7ba444 | test: Cindy held, Alice's joint Start, Cartesian Start, tool Open, controller switch, D-pad, free drive and both planners' Commit are refused and send nothing |
| F3 more guards pinned | f7ba444 | tests: the Robotiq and the grasp re-checked after Confirm; the held robot's own base; retime's hard check measures the cubic |
| F4 what sends every tick | 24db711 | robot_control's Cartesian stream checks the lock before every send: a robot taken meanwhile ends it with a hold and one line naming both robots; free drive goes back to the joint controller (the D-pad already sent one stop). A base path sent just before a lease: base.py keeps no send time, so no new tracking; the watch catches it once the follower reports "following" (30 Hz) or the wheels turn (`base_moving`'s docstring, a test) |
| F5 a stale link stops everyone | 525e5f7 | behaviour kept (user decision, open question 7); the refusal and the interrupt now say what clears it; Health's Reconnect is refused while a robot is held (README too) |
| F6 short moves over 90°/s² | b3b21c1 | retime gives a run one step more at a time while its cubic needs over SPLIT_ACCELERATION and no bend is left to stop at: 1.94° on one joint 3 → 4 samples, 97 → 36°/s²; a test sweeps straight moves of 0.05–40° on one and two joints, at full speed and at 1 s |
| F7 the pendant's slider | 4df05cb | §8 lab step 1 |
| F8 the grasp's 2 s wait | 9ef13bc | `scaffolding_grasp(ready=…)`: the others-moving and process check asked again right after the stall clears, with no wait before the screws tighten |
| F9 a cancel with refused holds | 308347b | the record adds `end_stale` and `end_age` (seconds per arm) |

Also on this branch: the M4 re-review nits (96c1be2, see the M4 spec) and `plugins/assembly_run/plugin.py` split into
plugin.py (400 lines) and writes.py (391, mixed in), no change in behaviour (8a224e9).

Mutation check, re-run at 8a224e9 in a throwaway worktree: the reviewer's 17 mutants (I3's text moved to the new
stale message) and 10 more on the new guards (the stream and free drive check, the grasp's re-ask, the short-run
stretch, Reconnect, the stale end, a second Continue, used-last order, an unreadable newer run, the select after a
soft stop): all 27 fail a test.

Re-run: default 599 passed, 2 skipped; `-m slow` 49 passed, 1 xpassed; flake8 and pep257 clean; colcon build;
monitor smoke (domain 70, localhost only) as before, first tick 25 ms, no "failed", clean teardown; headless chain:
B20_R_home planned (80 rows, 23 s), timed to 20.4 s, ARRIVED, end within 0.04°, events new_run, plan, prompt,
confirm, exec; lease given back; no error line.

### Open questions

1. Robotiq open at 0.426 (old) vs Jakob's `OPEN_POSITION` 0.0 (A3): check in the lab.
2. A Robotiq move that falls short counts as not done (deviation 4): right for the open; for M6's close on a bar a
   stall short of 0.8 is the expected end.
3. Keeping the plan after a refusal or a dry run (deviation 8), and the 2 s wait after any arm motion
   (deviation 2): acceptable for the lab?
7. A robot whose joint or wheel states go stale (over 0.5 s) after being live now refuses every Exec and interrupts
   (holds) a running motion of another robot: fail-safe, but a robot with a flaky link, even parked, will stop the
   others. Keep it, or count only robots in the step's scene, or use a longer age for parked robots? (Re-review F5:
   kept as is until the user decides; the refusal and the interrupt now say "no automatic motion until they come
   back (check its link in Health) or the monitor restarts", and Health's Reconnect is refused while a robot is
   held.)
8. Retime now refuses very noisy paths (e.g. 2000 rows with 1 mm noise: a stop at almost every waypoint, over 300 s);
   tamp's and ssik's plans pass (the slow tests, the chain).
4. A step whose motion has nothing to move (deviation 9): should Exec mark it done without sending?
5. The prompt's "largest joint travel" is the furthest any joint travels along the path (B20_R_home: 228°, so its
   prompt is a warning); the old guard compared start and end only.
6. README's arm_planner row still says "Commit only logs the path" (it sends); not changed here.
