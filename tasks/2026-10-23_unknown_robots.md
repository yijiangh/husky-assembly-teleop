# Unknown robots: one definition, one policy

Branch `jg/unknown-robots` off `core-port`.

## Before

How each part treated another robot whose state is not measured:

| Part | Never measured | Measured before, nothing fresh now | Operator told |
|---|---|---|---|
| Live snapshot | base at park pose, untracked; arms at stow, unmeasured | base: last fix, still "tracked" while the relay is silent; arms: last value, **not** unmeasured | n/a |
| Base planner, clearances (base D-pad, path runs) | obstacle at the guess | obstacle at the last value | no |
| Arm planner (compas_fab cell) | parked far away, no obstacle | obstacle at the last value | yes |
| Arm joint jog | no collision check at all | same | no |
| Process planning (`scene_for`) | belief (assembly run), else design, else parked | taken as live | yes |
| Process execution (`movement_exec`) | left out of the freeze and the motion watch | refused | yes |

## Decisions (JG, 23 Oct)

- Missing information means unknown, for everyone. A base is untracked once its mocap pose is lost (`pose_lost`,
  catches a silent relay too); an arm's joints are unmeasured once its joint states are older than
  `JOINT_STATE_MAX_AGE`. Set in `take_snapshot`, so every reader sees the same.
- The runtime's policy depends on measurements only (`world/stand_ins.py`): another robot not measured stands where
  it was last seen, else at its default pose with its arms stowed: the values the snapshot keeps. Used by the base
  planner, the arm planner, clearances and the base experiments' path check.
- The process planner (`scene_for`) lays the step's design over unknown fields first, then the runtime's stand-ins.
  The assembly run's recorded robots (beliefs) are no longer used for planning.
- No sphere around unknown arms: with a 0.5 s joint-state age limit it would appear on every short gap and trip
  nearby clearances; process execution already refuses to start while an arm is stale.
- Every guess is a warning: planners turn the plan into a warning plan and name the robot; a clearance logs it
  when it starts.
- Arm moves keep 1 cm (`ARM_STOP_DISTANCE`): joint jogs while their Clearance box is ticked (default; untick to jog
  without it, e.g. without mocap), and every arm planner commit.
- Terminology to settle later: `doc/glossary.md`, "To clean up".

## Visibility and permission (JG, 28 Oct)

- The 3D view draws the real robots as one-colour meshes (each keeps its own colour); a base not tracked and each
  arm not measured are drawn see-through.
- Each planner (base, arm, process steps) shows an amber "N robots unknown" chip, hidden at zero, named on hover. The
  base and arm planners show the robot and whether its base is tracked in one chip.
- Commit with other robots unknown asks for a second press within 2 s (Commit turns amber). The second press is the
  operator's permission: the robots unknown then may stay unknown during the motion (stand-ins, live again once
  measured, no new warning when lost again); any other robot that is not measured soft stops it. Without the
  permission a clearance refuses to start, or trips, on any other robot not measured.
- The D-pad, jog and base experiment clearances allow the robots unknown when they start (no second press there).
- Process execution (`movement_exec`) keeps no clearance: its Exec asks the same second press, but the permission
  is not passed on; its freeze and motion watch stay as they are.

## Not changed

- The acting robot's own guesses (`with_guesses`) and the lenient own-arm rule of the base D-pad (`ALL_ARMS`).
- Process execution's freeze and motion watch: they already refuse on stale measurements.
- The core's compas_fab cell still parks robots with problems; stand-ins arrive known, so it no longer sees any.
