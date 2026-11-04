# Glossary

Words this project gives one specific meaning, or that are easily mixed up with another, each in one or two sentences
with a pointer to where it is described. Not a description of the systems: the pointer holds the details. Plain
robotics or UI words are not listed. Paths are relative to `husky_assembly_teleop/` unless they start with `doc/`,
`core:` (`external/scaffolding_assembly_core/`) or `crl-husky:`.

## Structure

- **core / runtime**: core is only `scaffolding_assembly_core`, the library shared with the Rhino plugin and the
  planners; the runtime is the monitor's own framework that ticks and drives the plugins. Never call the runtime "the
  core". `core:AGENTS.md`, `doc/design/refactor_rationale.md`.
- **contract / context**: a contract (`<Name>Contract` in `shared/<name>_contract.py`) is the only way one plugin
  reads another's live data; what the runtime hands a plugin is its context (`PluginContext`). `shared/__init__.py`.
- **shared value**: a contract output that is frozen and replaced on every change, so a reader may compare by
  identity. `shared/__init__.py`.
- **tick**: the 50 ms main-thread cycle in which everything that touches state runs. `monitor.py` (`_tick`).
- **intent**: a viser callback's work, queued to run on the main thread at the plugin's next step.
  `plugin_api/context.py`.
- **task**: an asyncio coroutine a plugin starts with `ctx.spawn`, cancelled on soft stop unless it survives it. Not
  an **experiment task**. `plugin_api/context.py`.
- **stopped plugin**: a plugin stopped after repeated errors, with its dependents; unrelated to soft stop.
  `monitor.py`.

## Scene and state

- **measured vs planned**: measured state is written only by ROS callbacks; planners never write into it.
  `world/measured.py`.
- **unmeasured**: joints never measured, or stale; planners and clearances refuse them unless told to assume them.
  `world/kinematics.py`, `core:robot.py`.
- **scene / live scene / snapshot / mirror**: the live scene is the one plugins edit (`ctx.scene`); the snapshot is
  its copy taken once per tick for planners, clearances and the 3D view; a mirror is one planner's private copy in a
  collision backend. `world/scene.py`, `core:mirrors/`.
- **tracked object**: a mocap rigid body that is not a robot (`tracked/<name>`). ! Its `tracked` flag stays true
  when the relay goes silent: check `pose_lost`. `robot_interface/mocap_state.py`.
- **pose_lost / stale**: `pose_lost` means no valid mocap sample for 0.3 s; a reading is stale past its age limit. In
  the snapshot both count as unknown. `world/checks.py`.
- **stand-in**: how a collision check takes another robot that is not measured: where it was last seen, else at its
  default pose, arms stowed. Committing against one needs a second press. `world/stand_ins.py`.
- **ghost**: a see-through robot copy for a target (blue) or a plan (green). `ui/ghost.py`.

## Safety

- **lease**: the right to command a robot within this monitor; stop and `hold` need none. It keeps code apart, not
  people: laptop and phones share one lease. Say "has the lease", not "holds". `robot_interface/robot_lease.py`.
- **soft stop**: "Stop all": every robot stops and latches (`RobotStopped`) until Resume. Not an e-stop.
  `robot_interface/soft_stop.py`.
- **MotionRefused / refusal**: MotionRefused is a motion refused for an operator's reason, logged as a normal end; a
  refusal is a command declined without raising, shown next to its button. `robot_interface/refusal.py`.
- **freeze**: a task's promise that other robots or objects stand still; if one moves, the task is cancelled
  (`WorldMoved`). Not the panels' Live/Paused button, not a frozen dataclass. `world/freeze.py`.
- **clearance / trip / watch / path clearance**: a clearance soft stops a robot a task drives once it comes within
  its margin of anything, always below what its plans keep (arms plan 1 cm, stop at 5 mm; bases plan 10 cm, stop at
  5 cm; a plan keeping less stops at half of it, `stop_for`); a trip is one firing; a watch is checked the same way
  but only reports. The Clearance box turns it on or off. The base planner's path clearance is instead a recorded
  distance. [plugins.md, Clearance](lab/plugins.md#clearance), `world/clearance.py`.
- **hold**: the arm command "stop and stay" (no lease needed). Not a D-pad hold button, a support robot's bar hold
  (`bar_holding`), or a precondition that holds (below). `robot_interface/arm.py`.
- **safety sync**: the onboard `multi_arm_safety_sync`, which must vouch for an arm before any arm motion goes out.
  Not time sync, not the Drive's sync. `robot_interface/arm.py`.
- **simulated part**: a base, arm or mocap counts as simulated only while its announcements keep coming.
  `robot_interface/simulated_parts.py`.

## Process format

Details: `core:doc/process_format.md`.

- **process / assembly**: the process is the plan for one assembly; the assembly is the structure it builds.
- **schedule / action / movement**: the schedule orders all actions; an action is one robot's movements on one bar;
  a movement is one segment that runs without stopping.
- **state / target**: a state is one complete moment; a target is the part of a state a movement decides. `null`
  means "not decided by the process".
- **present / attached / built**: bodies that exist now / bars held by robot links / bars fixed in the structure.
- **half / mount / mate**: a half is a connector piece; its mount is the bar it is fixed to; a mate pairs two halves
  (or a half and the ground). "Mount" means only this: a tool sits on a flange, the operator places a bar.
- **keyframe**: the solved joints stored in a target. Planners may not move it.
- **solution**: planner results tied to the content hash they were solved against; a mismatch makes them stale.
- **file checks (A…) / plan checks (B…)**: file checks reject a file on every read and write; plan checks run on
  demand and block execution.

## Planning and motion

- **stale (plan)**: a ready plan that no longer fits the robot. `shared/planner/plan.py`.
- **commit**: sending a ready plan as one task that keeps the lease for the whole motion.
  `shared/planner/planner_service.py`.
- **follower**: crl_husky's onboard pure pursuit, which drives a base path. `plugins/base_planner/`.
- **xICR (turning centre)**: where the skid steer really turns, along its body from base_footprint; the lattice
  plans it, not base_footprint. `plugins/base_planner/follow.py`.
- **lattice / lead / portal**: the lattice is the default base planner (hybrid A*); a lead is the straight drive onto
  the target; a portal is a narrow place crossed in one move. `plugins/base_planner/algorithms/lattice.py`.
- **Margin (base planner)**: the distance a base path keeps from every other robot and body (10 cm at start,
  `BASE_PLAN_DISTANCE`). `plugins/base_planner/world.py`.
- **park / stow / parked**: park is the base's floor pose, stow the arm's rest joints (`park.yaml`); inside a mirror,
  parked means moved far away because absent or unknown. A robot **Parked** in Health is instead taken as known where
  it stands, measured or not (`world/stand_ins.py`, `parked`).

## Assembly runs

Details: `doc/lab/plugins.md`, `doc/design/run_record.md`.

- **assembly run**: how far the assembly of one process got, in `runs/<YYYYMMDD_HHMMSS>/` inside the process folder.
  Not an experiment's run folder, not a calibration run. `shared/runs/runs.py`.
- **operator step / part**: an operator step is a movement with its tool movements folded in; its parts are how the
  executor runs them (motion, tool step, by hand, refused). Not a guided procedure's step.
  `shared/assembly_planning/steps.py`.
- **live robot**: in the snapshot with every joint measured and its base tracked, both fresh (`is_live`).
- **Live IK**: solving a step's start by IK at the live base, nothing moved (`check_goal` in code).
  `plugins/assembly_steps/plugin.py`.
- **ground / one-mate / two-mate bar**: a bar on ground joints (rigid insert, no screw), or mating a built bar at one
  or both tools. `shared/execution/insert.py`.
- **gripper screw / joint screw**: the tool's two motors: one clamps the bar, one fixes it to its joint. The driver
  calls them M1 and M2; text never does. `robot_interface/end_effectors.py`.
- **insert / split / stall / settle**: the insert mates a held bar, rigid up to the split, then compliant until the
  joint screws stall; settle is the other arm finishing under compliance after a one-mate stall.
  `shared/execution/insert.py`, `shared/execution/stream.py`.
- **retreat / break-out**: backing the tools out of their joints; the break-out is its first, compliant part.
  `shared/execution/retreat.py`.
- **handoff**: a support robot's Robotiq close on a bar another robot holds, its arm compliant.
  `shared/execution/handoff.py`.
- **servo**: passes that move the flanges onto the authored targets after a transfer, approach or ground insert.
  `shared/execution/servo.py`.
- **transfer workaround**: a session setting: the transfer runs with empty tools and the operator places the bar at
  the insert start. `plugins/process/`.
- **carry / basis / robot record**: carry is joints found live that the process panel draws over the authored
  state; basis is what a step's plan was made from; a robot record is where the run last recorded a robot, for the
  report only. `shared/process_contract.py`, `shared/execution/basis.py`, `shared/runs/runs.py`.
- **Rhino obstacles**: the process's own obstacles, left out of collision checks unless ticked. Not the measured
  room (`obstacles` plugin). `shared/process_scene/scene_bodies.py`.

## Guided procedure (calibration)

Details: `doc/lab/calibration_manual.md`, `shared/guide/`.

- **current / out of date**: a result is current while what it was built from is unchanged; time never ages it.
  Calibration says "out of date", never "stale".
- **precondition holds / expired**: a precondition holds for a while or until the base moves; expired warns, except
  the safety boxes, which refuse.
- **acceptance**: Done twice on a failing result, kept while its inputs match.
- **work left**: the earliest result step that is missing, out of date, pending, or failing unaccepted.

## Lab and data

- **relay**: the one mocap relay per ROS domain; it reads Motive or a simulation, never both. `doc/lab/mocap_relay.md`.
- **raw / lab frame**: raw is Motive's frame turned Z-up; the lab frame is the one every relayed pose is in, set by
  the marks and anchors of `data/lab/<lab>_frame.json`. `world/mocap_frame.py`.
- **floor calibration / refit**: the floor calibration (`floor_from_raw`) puts raw poses in the lab frame; a refit
  fits the anchors or truss cameras to where Motive puts them now. Both applied only on command.
  `doc/lab/floor_calibration_manual.md`.
- **mark / anchor / truss camera**: probed floor points / markers that hold Motive's continuous calibration / the six
  bolted cameras.
- **body definition / refine**: a rigid body's definition is its markers only; a refine changes it and puts data
  measured with it out of date.
- **base calibration**: `base_mocap_from_base_footprint`, per robot. Unqualified, "calibration" means this one; the
  others say which (floor, Motive's camera, UR factory, arm mount check). `doc/lab/calibration_manual.md`.
- **environment sim / real**: written into every file, decided by the data source, never by the ROS domain.
- **fleet domain / sim domain**: the real robots run on domain 80; every simulation uses another.
  `doc/lab/simulation_manual.md`.
- **Drive**: the synced Google Drive folder (`HUSKY_DRIVE_ROOT`) with all experiment and calibration data. Driving
  the base is "driving", never "the Drive". `drive.py`.
- **calibration session / calibration run / lab session**: a dated folder of calibration data / one arm's progress
  through the guided calibration / a day in the lab.
- **take**: in calibration, one recorded sweep or punch; in mocap_bars, one recording.
- **experiment / run folder**: an experiment names a folder of run folders, each one recording.
  `shared/recordings/runs.py`.
- **experiment task / attempt**: a named start, target and obstacles / one plan from its start. Experiments A, B, C
  are a different experiment in each manual. `doc/experiments/`.
- **hypothesis / report / results**: a numbered refutable claim / the generated numbers / the text written from them.
  `doc/experiments/experiment_design_guide.md`.
- **temporary / virtual obstacles**: the operator's, kept locally until promoted / a base planner task's, never
  saved. `plugins/obstacles/`.

## To clean up

Words that overlap; settle each on one name.

- **tracked / measured / known**: `base_tracked` and `unmeasured` both mean "not known now"; the relay's `tracked`
  only means the last sample was valid.
- **guess / stand-in / default pose**: `with_guesses` fills in the acting robot, `with_stand_ins` the others, from
  the same values.
- **parked**: the park pose, a robot a mirror moves away, and a parked robot in a base planner task.
