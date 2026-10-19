# Plugin roadmap

The set of plugins the port is aiming at, and the old feature flag each one
replaces. See `doc/refactor_rationale.md` for why the old flags had to go, and
`husky_assembly_teleop/plugins/__init__.py` for the rules a plugin follows.

| Plugin | Replaces | Notes |
| --- | --- | --- |
| `example_*` | — | **Done.** Six small teaching plugins in `plugins/examples/`: `example_ui` (widgets, intents), `example_robot_state` (config vs state), `example_pybullet` (scene use and cleanup), `example_sequence` (timers, Next, Cancel), `example_recording` (live plot and recording of signals), `example_plot` (live uPlot). |
| `robot_control` | the per-robot status readouts and manual buttons | **Done; the reference plugin.** Shows every robot's state, switches controllers, and holds each controller's inputs (base twist; arm joint moves, streamed Cartesian moves with a compliance force, zero FT; tool open/close/stop, plus Robotiq position and reactivate, scaffolding screw). Read it before writing a new plugin. |
| `health` | — | **Done; loaded by default** (`config.DEFAULT_PLUGINS`, off with `-p no_default_plugins:=true`). One glance at every robot and tracked object: mocap live and its marker error, controller managers answering, joint_states fresh, gripper action connected / tool status fresh. Green banner when all is well. |
| `process` | BarAction loading and stepping | **Milestone 1 done: load and view.** Loads a schema 2.1 process folder through the core's `process` package (its `doc/process_format.md`; `-p process_directory:=...` or the panel; an old compas_fab export is converted on load into `<export>_process`, reused while up to date; `scripts/convert_process.py` does the same offline), steps through every movement of the schedule and draws its authored `RobotCellState` (start or target) from forward kinematics. Purely a viewer and provider: no planner, no PyBullet. Planning plugins declare `requires = ("process",)` and read `step` / `loaded` / `revision` for their goal and initial guess, and bring their own compas_fab client. Next: show precomputed trajectories; put the process's objects into the scene (`doc/scene_refactor_plan.md` phase 4), ids prefixed `process/` (e.g. `process/bars/B1`). |
| `obstacles` | — | **Done.** The lab's obstacles from the lab file `data/lab/<lab>.json` (git, `-p lab:=`): boxes, cylinders and trusses with name, centre, size, yaw and source, and the border of the usable floor as walls; ids `obstacles/boxes|cylinders|trusses/<name>`, unchanged. Edits them: select by click or dropdown, move (yaw only), resize, type, rename, remove, border range; Save, Revert and an unsaved chip. Temporary obstacles (`obstacles/temporary/<n>`, `~/husky_obstacles_temporary.json`, taken over from the debug gizmo's old file) and Promote to lab. Gizmos and picking in `ui/body_gizmo.py`, shared with `debug_gizmo`. Later: fit from probe points (`lab.box_from_top_points`, `lab.cylinder_around_points`). |
| `debug_gizmo` | — | **Done; debug only.** Moves any scene body with a world pose (click or dropdown, gizmo or typed pose), Log, Reset; never saved. |
| `mocap_probe` | — | **Done.** Tracks the probe rigid body (id 1863 by default, via `ctx.track_object`, so it also shows on the health panel), shows its live pose, and records points for measuring corners of obstacles and structures, numbered per label (`table_1`, `table_2`, ...). Set Motive's pivot to the probe tip. A big status panel ("Probe" tab, minimized on the right) shows green / amber / red from across the room and flashes blue on each recorded point. Samples in a tight checkbox list at the bottom, all selected by default with Select all (`checklist.CheckList`, reusable); Delete and Export act on the selection, export to `~/husky_probe/*.json`; Load points adds an exported file back, unselected, renumbering names already taken. Coverage map: with "Map coverage" ticked, every probe message counts in its 1 m cell of a 15 × 15 m grid centred on the mocap origin as tracked or not; floor tiles go grey → red … green by tracked rate; each cell also keeps the mean and std of the marker error; Export / Clear, and Load (browser upload) replaces the map and keeps adding to it. |
| `bar_action` | `BAR_ACTION_LIVE_REPLAN_EXE` | Plan / replan / execute for the current step. Its trajectories, preview ghosts and validation results are its own state. |
| `base_planner` | the base planning path | **Milestone 1: plan, inspect, commit flow. Experimental.** Target typed, dragged with a gizmo (`pose_input.PlanarPoseInput`, reusable), or loaded from the robot, its parked pose (`RobotConfig.park_pose`) or the `process` step's base frame. Plans from the current pose with a selectable algorithm (`base_planner/algorithms/`, each with its settings and a debug view; so far a hybrid A* lattice (the default) over the follower's turning centre (moves it drives without repositioning) that drives curves where they fit and lines up exactly with narrow slanted passages, its path checked as the onboard follower drives it, simulated (`base_planner/follow.py`), and RRT-Connect over (x, y, yaw) with corners rounded into arcs; both checked for a range of xICR), around every other robot and scene body, in a private PyBullet world on a worker thread; previews with a time slider and ghosts; goes stale when the robot or target moves. Commit sends the path to the onboard pure pursuit follower (`BaseInterface.send_path`, geometric, capped at the planning speeds) after checking tracking, e-stop and that a follower reports; the status line follows its progress, the floor shows the follower's path and the robot's trail (`ui.base_markers`, shared with `base_exp`), and Record saves each run to `data_experiment/base_planner/`. |
| `base_exp` | the base MPC experiment (crl-husky `mpc_trajectory_ui`, `mpc_live_viz`; removed 15 Oct with the MPC) | **Milestone 1, experimental.** Template paths sent to the onboard pure pursuit follower (`crl_husky pure_pursuit`, message `crl_husky_msgs/BasePath`), geometric or timed; preview, live error and command plots, .npz recordings. Automation: standard paths, random paths, constant commands; analysis of the controllers, the robot model and its change with speed and turn rate (`scripts/analyze_base_exp.py`). **To do:** MPC as a second follower, retire the matplotlib tools. |
| `arm_planner`, `arm_planner_panel` | the arm planning path (single-arm goal plans) | A headless planner (`arm_planner`, `ArmPlannerPluginApi`) plus its panel (`arm_planner_panel`; enable the panel). **Milestone 1: plan, inspect, commit flow. Experimental, not yet tested thoroughly.** Joint target from sliders, the arm now, its stow pose, or the `process` step. RRT-Connect in one arm's joint space against a compas_fab world per robot (the robot itself with SRDF and its tools, the other robots, every scene body); first plan per robot loads the models. One arm per plan: no dual-arm or constrained planning yet (`bar_action`). Commit sends the path as one joint trajectory that stops at every waypoint (`send_to_arm`, `joint_path_move`); not yet tried on a robot. |
| `joint_control` | the direct joint-level jog and trajectory controls | Depends on nothing: talks to the real robots through `ctx.world` and needs no design at all. |
| `calibration` | `CALIBRATION` | |
| `punch_validation` | `PUNCH_CALIB_VALIDATION` | |
| `mocap_bars` | `BAR_ACTION_MOCAP_ACCURACY_TEST` | **Milestone 1, experimental.** Requires `process`. Picks a bar of the loaded process and the Motive rigid body made of its markers; fits the bar's CAD markers (`BodySpec.markers` of the halves mounted on it, `world/marker_fit.bar_markers`) to the measured ones without Motive's labels on every message; draws the real bar (its mesh at the fitted pose) and shows residuals and the deviation from the planned bar in the scene (`process/bars/<id>`: across, along, tilt); Record take saves an .npz to `data_experiment/mocap_bars/`. Replaces `mocap_probe`'s prefab fit. Not yet: servo loops, dual-arm accuracy, the old take and report framework (`old/mocap_experiment.py`). |
| `dual_arm_accuracy` | `DUAL_ARM_EE_CONSTR_ACCURACY_MOCAP_TEST` | |
| `kissing_experiment` | `DUAL_ARM_KISSING_REP_EXPERIMENT` | |
| `visual_servoing` | the live tracker and its plots | |
| `compliant_control` | `CONNECT_COMPLIANT_CONTROLLER` | |
| `collision_diagnosis` | `cc_diagnosis`, previously always-on | |
| `scaffolding` | the scaffolding tool buttons | |
| `joint_stream` | live joint plots | |

## Planner plugins share one panel

`base_planner` and `arm_planner` are subclasses of `shared.planning.panel.PlannerPlugin`,
which holds the flow every planner has: target → Plan on a worker thread →
preview with a time slider and ghosts → stale when the robot or target moves →
Commit. It declares `requires = ("process",)`, so both need `process` loaded. A subclass supplies the problem (`make_search`, `stale_reason`,
`send_plan`, `show_target`, `show_path`) and its own collision world; the base
knows no PyBullet or compas_fab. `shared.planning.search.connect` runs RRT-Connect
(`birrt`) for any state space, and `shared.planning.path.TimedPath` is the shared timed path.
`bar_action` should start from the same base.

## Per tick, or as a task

Anything that just keeps something in step -- `process`, `joint_stream`,
`collision_diagnosis` -- does it in `update`, a one-tick function.

Anything with a shape -- calibration sweep, kissing experiment,
plan-then-execute -- is a task: an `async def` started with `ctx.spawn` from a
button, and cancelled with `task.cancel()` from another. The logic reads in the
order it happens, and the task's `finally` block is where hardware is made safe.
`plugins/examples/sequence.py` is the template.

## Recording and live plots: once per tick

A `Signal` (`plugin_api/trace.py`) is any function of live state, so derived
values (FK, tracking errors) record like measurements. `ctx.trace(...)` samples
signals every tick for a live `ui/trace_plot.TracePlot`; `with ctx.record(...)`
does the same for the length of a block, then `rec.save(path)`. Common signals
are in `plugin_api/signals.py`. Recording at the full stream rate was dropped: the
rate limiter only forwards 50 Hz, and what needs more than 20 Hz (contact
transients, controller tuning) needs the driver's 500 Hz, i.e. `ros2 bag
record` on the robot.

Still experimental: `process`, `base_planner` and `arm_planner`.
