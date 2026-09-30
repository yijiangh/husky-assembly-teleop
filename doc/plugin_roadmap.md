# Plugin roadmap

The set of plugins the port is aiming at, and the old feature flag each one
replaces. See `doc/refactor_rationale.md` for why the old flags had to go, and
`husky_assembly_teleop/plugins/__init__.py` for the rules a plugin follows.

| Plugin | Replaces | Notes |
| --- | --- | --- |
| `example_*` | — | **Done.** Four small teaching plugins in `plugins/examples/`: `example_ui` (widgets, intents), `example_robot_state` (config vs state), `example_pybullet` (scene use and cleanup), `example_sequence` (timers, Next, Cancel). |
| `robot_control` | the per-robot status readouts and manual buttons | **Done; the reference plugin.** Shows every robot's state, switches controllers, and holds each controller's inputs (base twist, trajectory stub, zero FT, gripper open/close). Read it before writing a new plugin. |
| `health` | — | **Done; loaded by default** (`config.DEFAULT_PLUGINS`, off with `-p no_default:=true`). One glance at every robot and tracked object: mocap live and its marker error, controller managers answering, joint_states fresh, gripper action connected / tool status fresh. Green banner when all is well. |
| `cell` | BarAction loading and stepping | **Milestone 1 done: load and view.** Loads a design folder (`ActionSchedule.json`, `BarActions/`, `RobotCell*.json`; `-p design_directory:=...` or the panel), steps through every movement of the schedule and draws its authored `RobotCellState` (start or target) from forward kinematics. Purely a viewer and provider: no planner, no PyBullet. Planning plugins declare `requires = ("cell",)` and read `step` / `cell` / `revision` for their goal and initial guess, and bring their own compas_fab client. Next: show precomputed trajectories. |
| `obstacles` | — | **Done, placeholder layout.** Fixed boxes (tables, cabinets) in `BOXES`, drawn and added to the shared PyBullet scene; no UI. Replace with the measured lab furniture. `base_planner` requires it and plans around `boxes`. |
| `bar_action` | `BAR_ACTION_LIVE_REPLAN_EXE` | Plan / replan / execute for the current step. Its trajectories, preview ghosts and validation results are its own state. |
| `base_planner` | the base planning path | **Milestone 1 done: plan, inspect, commit flow.** Target typed, dragged with a gizmo (`pose_input.PlanarPoseInput`, reusable), or loaded from the robot or the `cell` step's base frame. Plans from the current pose, previews with a time slider and a see-through ghost, goes stale when the robot or target moves. Plans with pybullet_planning's bidirectional RRT (`birrt`) over (x, y, yaw) with a turn-drive-turn steer, around the other robots and the `obstacles` boxes, on a worker thread with a private PyBullet client. **Stub:** Commit only logs (`send_to_onboard_follower`), to be wired to the onboard path follower. |
| `joint_control` | the direct joint-level jog and trajectory controls | Depends on nothing: talks to the real robots through `ctx.world` and needs no design at all. |
| `calibration` | `CALIBRATION` | |
| `punch_validation` | `PUNCH_CALIB_VALIDATION` | |
| `mocap_accuracy` | `BAR_ACTION_MOCAP_ACCURACY_TEST` | |
| `dual_arm_accuracy` | `DUAL_ARM_EE_CONSTR_ACCURACY_MOCAP_TEST` | |
| `kissing_experiment` | `DUAL_ARM_KISSING_REP_EXPERIMENT` | |
| `visual_servoing` | the live tracker and its plots | |
| `compliant_control` | `CONNECT_COMPLIANT_CONTROLLER` | |
| `collision_diagnosis` | `cc_diagnosis`, previously always-on | |
| `scaffolding` | the scaffolding tool buttons | |
| `joint_stream` | live joint plots | |

## Per tick, or as a task

Anything that just keeps something in step -- `cell`, `joint_stream`,
`collision_diagnosis` -- does it in `update`, a one-tick function.

Anything with a shape -- calibration sweep, kissing experiment,
plan-then-execute -- is a task: an `async def` started with `ctx.spawn` from a
button, and cancelled with `task.cancel()` from another. The logic reads in the
order it happens, and the task's `finally` block is where hardware is made safe.
`plugins/examples/sequence.py` is the template.

## Open: recording high-rate state

Experiments need state at its full update rate, not the 20 Hz tick. The
experimental `recording.py` only taps raw ROS messages of a few arm topics. It
should be just as easy to record any interface state (joint positions, TCP
pose, a mocap fix, a derived value) at the rate it updates. Undecided how;
`cell` and `base_planner` are experimental too.
