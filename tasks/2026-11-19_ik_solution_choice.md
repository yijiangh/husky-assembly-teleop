# Support approach: the operator picks the next IK solution; guide motion time changeable until Go

## Context

B21_H_approach failed again and again in the lab (19 Nov). The process wrote Alice's shoulder lift a whole turn off,
and IK at the live base only ever handed the planner one goal: the solution nearest the process's joints. Already done
today, uncommitted on `hs/experiment`:
- the IK reference lift is wrapped into range (`shared/assembly_planning/ik.py`);
- `solve_ik(..., skip=n)`, passed through `ArmPlannerContract.solve_goals(..., skip)`;
- in the guide, an automatic move to the next IK solution (up to 4), slower, with a warning.

The user now wants, in both the **Assembly tab** and the **Assembly guide**:
- when a support approach fails, a pop-up says what failed and lets the operator choose **Next IK solution**,
  **Move the grasp** (closes the pop-up and points to Jakob's grasp box: Assembly › process › grasp, Slide mm / Roll °,
  Apply), or **Give up**. The automatic move to the next solution goes away.
- a plan to solution 2 or later: it warns, Auto waits at Go, and it takes at least 2× the default motion time
  (user's choice);
- the guide's Motion time changeable while planning and while the plan waits for Go (preview re-timed, no replan),
  greyed only while the arm moves (user's choice: the robot has the whole trajectory once sent, and the pendant
  speed slider must stay at 100 %).

## What exists (reuse)

- `ui/popup.py` `Popup`: a window floating beside the main panel, buttons on top, changing text below. Pattern:
  `plugins/movement_exec/panel.py:67-71,110-113` (built hidden, buttons wired with `ctx.defer`, `show(html)` / `close()`).
  Layers allow `shared` to import `ui`.
- Assembly tab re-times without replanning: `assembly_steps/plugin.py:277-284` `set_traj_time` → `_timed` (817-823),
  `timed_route` (`shared/assembly_planning/step_plan.py:73-82`). The guide copies this.
- Assembly tab flow (motion planner only, never the arm planner): Live IK `checked_goal` (plugin.py:403-456) →
  `StepGoal.solve` (goal.py) → `request_ik`; Plan `make_search` (505-600) reuses the kept Live IK goal via
  `joints_near(_key_of_goal(), base)`; failures go through `_say(..., failed=True)` (Live IK: plugin.py:448; Plan:
  `step_panel_base.py:245-247`).
- Motion planner IK: `MotionPlanRequest` (`shared/motion_planner_contract.py:48-82`) → `plugins/motion_planner/work.py`
  `run_plan` / `run_ik` → `tamp_route.plan_movement` / `check_goal` → `_route` → `Route` → `Route.ik_goal`
  (`tamp_route.py:458-499`, `solve_ik` at :486).
- Guide: a planning RuntimeError ends Auto and becomes the part's BAD outcome (`_run_part` plugin.py:947-955,
  `Procedure._run`); Prepare again replans from the live robot (`prepare` → `_start`). Preview: `procedure.preview_go`
  keeps `preview_path` (`shared/guide/procedure.py:231-238`); Exec reads `self.traj_time` after Go
  (`ExecRequest`, plugin.py ~1196). Slider greyed by `MoveKind.show` (kinds.py:236-245) for the whole procedure.

## Changes

### 1. Shared pop-up and fallback rule — new `shared/assembly_tab/ik_choice.py`
- `NEXT_SOLUTION, MOVE_GRASP, GIVE_UP` labels; `GRASP_HINT` ("Assembly › process › grasp: Slide mm along the bar,
  Roll ° about it (the grape gripper shows where), Apply, then Plan / Prepare again: IK starts again at solution 1").
- `IkChoice(view)`: one `Popup("Approach: no path", ALERT_TRIANGLE, {NEXT_SOLUTION: busy, MOVE_GRASP: None,
  GIVE_UP: None})`; `offer(label, solution, why)` shows what failed below the buttons and greys Next when `why` says
  "no more solutions" (`ik.py` reason); `close()`. The caller wires the three buttons.
- `FALLBACK_SLOWDOWN = 2.0`, `fallback_time(traj_time, routine) = max(traj_time, 2 × DEFAULT_TRAJ_TIME[routine])`,
  `fallback_warning(solution)`: "The goal is IK solution n, not the one nearest the process's joints: the arm ends in
  another configuration than the process's, timed slower (T s). Play the preview carefully."
  (moved out of `assembly_guide/plugin.py`).

### 2. IK solution through the motion planner
- `MotionPlanRequest.ik_skip: int = 0`; thread it through `work.py` (`run_plan`, `run_ik`), `plan_movement`,
  `check_goal`, `_route`, `Route.__init__` (keyword extra like `goal_poses`), and `Route.ik_goal` →
  `solve_ik(..., skip=self.ik_skip)`. Servo routes keep 0.

### 3. Assembly tab (`plugins/assembly_steps/`)
- `_ik_skip` kept with the step's goal key: reset to 0 when that key changes (a new step, or a grasp Apply, which
  rebuilds the process). `_key_of_goal()` gets the solution as its last item, so a goal kept for solution 1 is never
  reused for solution 2 (`joints_near`, `same_step` compare keys). Live IK and Plan requests carry `ik_skip`.
- A failed Live IK or Plan of a support approach (not a cancel) calls `IkChoice.offer`. Buttons:
  - Next: `_ik_skip += 1` (the key changes, so the kept goal is not reused), close, rerun what failed (Live IK or
    Plan);
  - Move the grasp: close, `_say(GRASP_HINT)`;
  - Give up: close.
- The pop-up closes on a step change or a plan found.
- A plan or Live IK at solution ≥ 2: `traj_time = fallback_time(...)` before timing, the fallback warning in the
  plan's message (amber), "IK solution n" in the Live IK line, `ik_solution` in the plan / check_goal records.

### 4. Assembly guide (`plugins/assembly_guide/`)
- Remove the `GOAL_OPTIONS` loop; restore the original loop (Auto/God plan the same goal again up to
  AUTO_PLAN_TRIES), now with `skip=self._ik_skip` in `solve_goals` and `ik_skip` in the `_plan_shown` `request_ik`.
- `_ik_skip` kept with (part index, loaded process); reset when either changes.
- Where the planners' failures are raised (`_plan_on_arm_planner`, `_plan_shown`: no goal, no path, not a
  cancel), a support approach calls `IkChoice.offer`, then raises as before (the part fails and Auto ends); a
  scene not shown or a robot not live opens nothing. Buttons:
  - Next: `_ik_skip += 1`, close, `prepare(ctx)` for that part (refused if another part is shown);
  - Move the grasp: close, and GRASP_HINT goes into the notes below the buttons;
  - Give up: close.
- Solution ≥ 2: `fallback_time` and `fallback_warning`, as today (the warning makes Auto wait at Go, and the Go
  question shows it).

### 5. Guide motion time until Go
- `_execute` keeps `self._waiting = (part, route, question by path)` while `_ask` waits; `self.moving = part` from
  Go until the executor returns (the sending moved into `_send`).
- `set_traj_time`: sets the time; while a plan waits, re-times it with `timed_route` and replaces
  `procedure.preview_path` (preview back to 0) and `procedure.prompt` (its "N s").
- `MoveKind.show`: the slider greyed only while `guide.moving is part`; its hint says "changeable until Go; a move
  already sent keeps its time".

### 6. Docs
- `doc/lab/plugins.md`: replace today's guide sentence on trying the next solution; add the pop-up to the Assembly
  tab's Live IK / Plan text; Motion time "changeable until Go" in the guide.
- Write this plan to `tasks/2026-11-19_ik_solution_choice.md` first (AGENTS.md).

## Verification

- Unit tests, while iterating (`-n 0`):
  - `test/shared/test_planning_tamp_route.py`: `ik_skip` on the request reaches `solve_ik` (the monkeypatched
    `solve` already forwards kwargs).
  - `test/plugins/assembly_guide`:
    - an approach that fails opens the pop-up; Next replans with skip 1 (the fake planner's `skips == [0, 1]`);
    - a plan at solution 2 warns and takes 2× the default time; "no more solutions" greys Next; Give up closes;
    - a Motion time change while Go waits re-times `preview_path`, and the ExecRequest gets the new time;
    - the slider is greyed only while moving;
    - update the tests written today for the removed automatic loop.
  - `test/plugins/assembly_steps`: a failed Live IK of an approach opens the pop-up; Next reruns Live IK with
    `ik_skip=1` and drops the kept goal; a plan at solution 2 warns and is slower.
  - `test/shared/test_planning_ik.py` (skip tests exist).
- Before handing over: `python3 -m pytest` (default set with linters), plus `-m slow` for `test/shared`,
  `test/plugins/arm_planner`, `test/plugins/assembly_guide`, `test/plugins/assembly_steps`.
- In the fake lab (`scripts/fake_lab/drive.sh` on a copy of 261117_2ModulesDemo_order_bracing): select
  B21_H_approach, check the pop-up appears in both tabs on a failure, Next plans to solution 2 with the warning and
  30 s, and the guide's Motion time slider re-times the preview while Go waits. Headless screenshots per memory
  `headless-page-screenshots` if a visual check is needed.
- Nothing is committed unless the user asks.
