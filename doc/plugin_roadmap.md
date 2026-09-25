# Plugin roadmap

The set of plugins the port is aiming at, and the old feature flag each one
replaces. See `doc/refactor_rationale.md` for why the old flags had to go, and
`husky_assembly_teleop/plugins/__init__.py` for the rules a plugin follows.

| Plugin | Replaces | Notes |
| --- | --- | --- |
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

## Which plugins are sequential and which are reactive

Anything with a shape -- calibration sweep, kissing experiment,
plan-then-execute -- overrides `run` and writes the sequence as a generator, so
the logic reads in the order it happens.

Anything that just keeps something in step -- `cell`, `joint_stream`,
`collision_diagnosis` -- overrides `update` instead and stays a one-tick
function.

Both can put a discrete operation in `ctx.spawn` so it can be cancelled from a
button.
