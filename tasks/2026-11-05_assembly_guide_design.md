# Assembly guide: design

The operator's one panel for running a process (`plugins/assembly_guide/`). It drives Prepare → Check → Go over
the layers below it and never does their work itself; every layer below stays usable on its own for a fix.

## Layers

| layer | plugins | the guide uses it for | the operator uses it directly to |
|---|---|---|---|
| guide | `assembly_guide` | — | walk the process |
| execution | `movement_exec` (through its contract) | every process movement: moves, by hand, servo, insert, retreat, gripper; their records and safety checks | (the Assembly tab, until the merge) |
| planners | `motion_planner`; `base_planner` (+ panel); `arm_planner` (+ panel) | plans of process movements (free moves on `arm_planner`, both arms in one search, in the live scene once `process` shows the part's movement (`scene_shows`), made a route by `arm_path_route`); the drive | fix a base or an arm: Plan › Base, Plan › Arm, `Load: Step` |
| robot | `robot_control` | — | drive, jog, controllers, tools, gripper |
| state | `process` (selection), `assembly_run` (current step, marks, settings, records) | where the process stands; Mark done; settings; `guide` events | pick an action |

- The guide makes each part's movement the run's current step (`request_set_current`), which selects it in the
  process plugin: the executor checks plans against it, and the panels' Load (`Step`; `Step joints`, `Step IK`) loads the part's target.
- The guide holds no lease: the planner or the executor holds the robots while a Go runs.

## Parts

An action's parts: the parks, the drive to its start (none for a robot holding a bar at the action's start: it stays
at the bar), then the executor's own split of each step (`step_parts`), then Mark done. The ungrasp goes with its
retreat, and a ground bar's servo with its insert, in one request.

| kind | Prepare | Go | done when |
|---|---|---|---|
| drive | `base_planner` plans to the action's start; nothing when the base stands there already | commit; the follower drives | it ended at its goal (6 cm, 6°) |
| move | IK at the live base (transfer, approach), then `arm_planner` for a free move, one arm or both (a held transfer, or no path: `motion_planner`); nothing when the arms stand at the goal (a goal by IK only at the base it was solved at) | the executor runs it | the executor finished it |
| by hand | — (Prepare and Go grey) | — : every box ticked, Done; the executor records it | the executor finished it |
| servo, gripper | the executor's request: its question | answers the question | the executor finished it |
| insert, retreat | `motion_planner` (insert from the servo's correction, retreat along the live axes) | the executor runs it | the executor finished it |
| mark done | — (Prepare and Go grey) | — : Done marks it (its robot still and live, or Done twice) | the run says done |

"Finished" is the executor's record (`MovementExecContract.finished`, per run), whoever sent the request; the run's log keeps it (each `exec` event with its keys, each `forget`), and the executor reads it back when a run is loaded or continued (`shared/runs/finished.py`). A drive counts as done once a later part of its action finished. A part whose
robot stands at its end, measured live, goes on with one Done; any other part takes a second press (Skip), Mark done
never. In Mode Off the guide stays on a finished part until Done goes on. A new run, or an action reopened anywhere,
makes its parts to do again.

## Panel

Top to bottom; nothing above the kinds' folders changes height: the title (action, robot, part n/m); the overview
(the shown action's parts, a numbered chip each); Back · Next · Redo from here; the chips of the part shown: needs
(robot live, not soft stopped, not held, earlier parts done, executor free, planner, plan stale), live (robots still,
distance to the goal, boxes ticked, parts done), result (executor finished, at its end, marked, skipped), then guide
(a press that asks again or was refused, mode, why Auto stopped, what runs and where, what Auto waits for, the shown
part's last outcome, kept per part); Mode Off · Auto · God; Prepare · Go · Done · Cancel; Preview time · Play; Clearance; the
shown kind's folder (a part by hand: its checklist); the question and warnings; the description: goal, what to do
(buttons in order, the fix one layer down last), what to check; the action's parts; Settings.

- The calibration guide's model: one part shows, and Prepare, Go and Done act on it; Back and Next show another and
  select its movement. The next part to do is not stored: it is the first part with no result.
- A procedure belongs to the part that started it: elsewhere, the guide line names it ("B20_J part 2 waits for Go"),
  Go and Preview act only there, Cancel anywhere.
- The overview chips are the calibration guide's (`shared/guide/panel.py`: `state_chip`, `overview_row`,
  `running_chip`): the fill is the result, the shown part ringed black inside, ▶ on the running one, the earlier
  parts the shown one needs without a result outlined dotted black.
- Go moves a robot. Done confirms what the operator did (a part by hand, its boxes all ticked), marks an action, or
  goes on from a finished part. Out of order (earlier parts of the action not done) both ask a second press; Go's
  one second press also answers its question for other robots not measured.
- A procedure that only waits for Go never holds a press back: Prepare or Done on any part replaces it, Auto going on.
  Prepare twice does a done part again, also in an action the run has marked (the mark stays).
- Redo from here, on a done part: it and every later part are to do again (the run reopens every marked action from
  its action on; the executor forgets what it finished from the part on).

## Modes

- Off: the operator presses Prepare, Go and Done; the guide never goes on by itself.
- Auto: works on the first part with no result from the part shown when it was switched on (skipping parts already
  done), shows it once and prepares and sends it; waits at every executor question, at every part by hand, at a plan
  with a warning, and on real robots at every drive and move (Auto only prepares there); a second press turns it on
  on real robots.
- God: Auto that answers every question, ticks every box by hand and goes past warnings; only while every robot's
  base, arms and tools and the relay announce themselves simulated (rechecked every tick and right before each
  answer); never accepts unknown robots, never skips, never marks a robot not live.
- Auto and God end on a failure, a refusal, Cancel, a soft stop of any robot, another plugin's lease, a part that ends
  without a result, a movement picked in the process panel, the parts built again (never while a part runs), or a
  Mark done with a doubt; the guide line says why until the mode is chosen again, and the run records it.

## Checked

- `test/plugins/assembly_guide/test_guide_pipeline.py` (slow): God on the real motion planner and executor, fake
  robots that follow: B21_H; B20_J and B20_R; the real lab test's seven actions B20_J to B22_R in one go.
- The fake lab (three simulated robots, the relay, the monitor): God from B20_J on; it found the two races now
  fixed (an executor part or a mark sent while an arm still settled).

## Open, for the merge into the Assembly tab and the process plugin

1. The planning of a step is built twice: `assembly_steps/plugin.py` (`make_search`) and the guide's `_plan`. Move
   it into one async `plan_step` in `shared/assembly_planning/step_plan.py` (the planner passed in), used by both.
2. `plugin.py` holds navigation and the procedures; split the procedures out once the merge decides their home.
3. `simulation_problem` belongs below the plugins (`world/` or `shared/`) once a second plugin needs it.
4. The fallback is text ("Plan › Arm, Load: Step IK"); buttons that open the panel would make it one click.
5. A phone tab for the guide (the executor's own phone Exec tab answers its questions already).
6. Clearance under the guide's Go, short of the unified rules (noted 5 Nov, not fixed):
   - The executor runs every plain motion with a contact-only clearance (`PLANNED_MARGIN = 0`, the motion
     planner's), also the arm planner's one-arm paths, which keep its 1 cm margin and should stop at 5 mm. The route
     should carry its planner's margin, and `motion_clearance` use `ARM_MOVE.stop_for(margin)`.
   - The guide shows none of the clearance chips Plan › Base and Plan › Arm show before Go: the robot already too
     close (the box's watch), a small clearance (a contact-only plan), no clearance (Ignore built bars, or the box
     unticked). They belong in the needs row of the current part.
   - Ignore built bars (guide Settings) turns the executor's clearance off without saying so in the guide.
7. tamp's collision check is slow: its free search clones the whole `RobotCellState` for every configuration (`husky_assembly_tamp/motion_planner/api.py:116`, `:136`) and compas_fab's `check_collision` copies it again (`pybullet_set_robot_cell_state.py:48`): about 0.2 s per check with Cindy's cell, so B20_R's way home took 50 to 117 s (99 of the 117 in `deepcopy`); the arm planner's `SearchCheck` plans it in 2 s. Free moves go to the arm planner, and one it finds no goal or path for fails at once (tamp would fail too, slowly): tamp is no fallback; the held transfer, the insert and the retreat still use tamp. Fix in tamp: a check that sets the joints and asks PyBullet (as `SearchCheck`), no copy per configuration.
8. Every plan should be made in the live scene, with `process` the one source of which bars stand where: the plan
   runs on the real robots. Every caller planning a process movement now waits for the live scene to show it and
   refuses the plan if it changed meanwhile (`shared/process_scene/live_scene.py`: `showing`, over
   `ProcessContract.scene_shows`): the guide (drive, IK, arm and motion planner), the executor (servo passes, the
   retreat's tail) and the Assembly tab (Live IK, Plan). The motion planner still builds its own `scene_for` scene;
   to drop it, its four differences from the live scene move into `process` (user's decisions, 6 Nov):
   - The held bar hidden for an empty transfer: the bar is not in the scene until the guide confirms it mounted
     (the guide tells `process` through its contract).
   - Ignore built bars: a `process` setting, not a planner option.
   - Robots not measured: the runtime's stand-ins, not the design's poses.
   - Measured bodies: mocap bars are a diagnostic tool only, never bodies in the scene (not for planning, not for
     the clearance); this also settles open decision 2 (B22_J tripped on `joints/J20-22_mocap`).
