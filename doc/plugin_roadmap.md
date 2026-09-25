# Plugin roadmap

The set of plugins the port is aiming at, and the old feature flag each one
replaces. See `doc/refactor_rationale.md` for why the old flags had to go, and
`husky_assembly_teleop/plugins/__init__.py` for the rules a plugin follows.

| Plugin | Replaces | Notes |
| --- | --- | --- |
| `example_*` | — | **Done.** Four small teaching plugins in `plugins/examples/`: `example_ui` (widgets, intents), `example_robot_state` (config vs state), `example_pybullet` (scene use and cleanup), `example_sequence` (timers, Next, Cancel). |
| `robot_control` | the per-robot status readouts and manual buttons | **Done; the reference plugin.** Shows every robot's state, switches controllers, and holds each controller's inputs (base twist, trajectory stub, zero FT, gripper open/close). Read it before writing a new plugin. |
| `health` | — | **Done; loaded by default** (`config.DEFAULT_PLUGINS`, off with `--no-default` or `-p no_default:=true`). One glance at every robot and tracked object: mocap live and its marker error, controller managers answering, joint_states fresh, gripper action connected / tool status fresh. Green banner when all is well. |
| `cell` | the cfab / BarAction session | Owns the design, the selected step, and the compas_fab planning session (its own PyBullet client, separate from `ctx.scene`). Almost everything else declares `requires = ("cell",)` and asks it for the current movement's state and the planner. |
| `bar_action` | `BAR_ACTION_LIVE_REPLAN_EXE` | Plan / replan / execute for the current step. Its trajectories, preview ghosts and validation results are its own state. |
| `base_planner` | the base planning path | The plan in progress is internal state, not shared. |
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

## Per tick, or as a job

Anything that just keeps something in step -- `cell`, `joint_stream`,
`collision_diagnosis` -- does it in `update`, a one-tick function.

Anything with a shape -- calibration sweep, kissing experiment,
plan-then-execute -- is a job: a generator started with `ctx.spawn` from a
button, and cancelled with `job.cancel()` from another. The logic reads in the
order it happens, and the job's `finally` block is where hardware is made safe.
`plugins/examples/sequence.py` is the template.
