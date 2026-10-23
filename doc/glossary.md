# Glossary

The words this project gives a meaning of its own, one or two sentences each, with where the idea lives. Paths are
relative to `husky_assembly_teleop/` unless they start with `doc/`, `core:` (`external/scaffolding_assembly_core/`)
or `crl-husky:`. Plain robotics words (pose, FK, URDF, frame) are in `doc/Su_Note.md` §0.

When a word has more than one meaning, the entry says which is which. Use the qualified name ("calibration run",
"run folder") wherever the context does not settle it.

## Structure

- **core**: only `scaffolding_assembly_core`, the library shared with the Rhino plugin and the planners: the process
  format, robots, scenes and mirrors. It imports nothing from the monitor, ROS or viser. `core:AGENTS.md`.
- **runtime**: the monitor's own framework that ticks, owns all state and drives the plugins: `monitor.py`,
  `config.py`, `plugin_api/`, `world/`, `robot_interface/`, `ui/`. Never call it "the core". `doc/refactor_rationale.md`.
- **plugin**: one feature with its own state, panel and scene bodies, registered with `@register` and loaded with
  `-p plugins:=[...]`. Plugins never import each other. `plugins/__init__.py`, `plugin_api/plugin.py`.
- **contract**: the `<Name>Contract` Protocol in `shared/<name>_contract.py`, the only way one plugin reads another's
  live data (`ctx.require`, for names in `requires`). What the runtime hands a plugin is its **context**
  (`PluginContext`), not a contract. `shared/__init__.py`.
- **shared value**: a contract output (`Plan`, `Selection`, `Lab`) that is frozen and replaced on every change, never
  edited, so a reader may compare by identity. `shared/__init__.py`.
- **tick**: the 50 ms cycle in which everything that touches state runs, on the main thread, in a fixed order: ROS
  callbacks, soft stop, kinematics, snapshot, plugin steps, clearances and freezes, tick waiters, traces, draw.
  `monitor.py` (`_tick`).
- **hook**: `setup`, `update` (once per tick), `draw` (copies state into widgets only; skipped while the panels are
  paused) and `teardown`. `plugin_api/plugin.py`.
- **intent**: a viser callback's work, queued (`ctx.defer`, `submit`) to run on the main thread at the start of the
  plugin's next step. `plugin_api/context.py`.
- **task**: an asyncio coroutine started with `ctx.spawn`, owned by one plugin, cancelled on soft stop (unless
  `survives_soft_stop`), when its plugin stops and at shutdown. Anything with a shape (a sequence, a motion) is a task;
  keeping something in step is `update`. Not to be confused with an **experiment task** (below).
- **stopped plugin / broken monitor**: a plugin failing `max_plugin_errors` ticks in a row is stopped with its
  dependents (tasks cancelled, UI kept), and a red banner asks for a restart. Unrelated to soft stop. `monitor.py`.

## Scene and state

- **measured vs planned**: measured state is written only by ROS callbacks (`WorldState`, `RobotState`); planners
  never write into it. `doc/refactor_rationale.md`, `world/measured.py`.
- **None until measured**: no stand-in values; a reading that has not arrived is `None`, a judgement starts False.
  `robot_interface/__init__.py`.
- **unmeasured**: joints never measured. Drawings fill them with the stow value or 0; planners and clearances refuse
  them unless told to assume them. `world/kinematics.py`, `core:robot.py`.
- **scene**: the world at one moment as core types (`Body`, `RobotObject`). The **live scene** is the one plugins edit
  (`ctx.scene`, ids under `<plugin>/`; `robots/` and `tracked/` are the runtime's). `world/scene.py`, `core:scene.py`.
- **snapshot**: the copy of the live scene taken once per tick, before the plugins run, for planners, clearances and
  the 3D view. A write shows in the next tick's snapshot. `world/scene.py` (`take_snapshot`).
- **mirror**: a planner's private copy of a snapshot in one collision backend (PyBullet, compas_fab), synced on that
  planner's own worker thread and used by no other. `core:mirrors/`, `doc/scene_refactor_plan.md` §6.
- **tracked object**: a mocap rigid body that is not a robot, scene id `tracked/<name>`, never put in the scene by a
  plugin (`ctx.track_object`). ! Its `tracked` flag stays true when the relay goes silent: check `pose_lost`.
- **fix / pose_lost / stale reading**: a fix is one valid mocap sample; `pose_lost` means none for 0.3 s, so nothing
  may drive from it; a reading is stale past its age limit (`STALE_AFTER`, `JOINT_STATE_MAX_AGE`). In the snapshot a
  lost base is untracked and stale joints are unmeasured: missing means unknown. `robot_interface/mocap_state.py`,
  `world/checks.py`, `world/scene.py`.
- **stand-in**: how a collision check takes another robot that is not measured: where it was last seen, else at its
  default pose with its arms stowed (the values the snapshot keeps); the process planner lays the step's design over
  it first. Each one is a warning for the operator. `world/stand_ins.py`, `shared/assembly_planning/scene_for.py`.
- **ghost**: a see-through robot copy for a target, a plan or a preview, shown only while its panel was the last one
  used. `ui/ghost.py`.

## Safety

- **lease**: the right to command a robot, held by one holder in this monitor: every motion command and controller
  switch needs it, else `NotLeased`; stop and `hold` need none. It only keeps this monitor's code apart, not people:
  the laptop and every phone share robot_control's lease. Say "has the lease", not "holds". `robot_interface/robot_lease.py`.
- **soft stop** ("Stop all" on the panel, `SOFT STOP` in the log): every robot stops and latches, refusing motion
  commands (`RobotStopped`) until Resume, and every task is cancelled but survivors. Not an e-stop.
  `robot_interface/soft_stop.py`, `monitor.py` (`_soft_stop`).
- **Resume**: clears a robot's latch, restarts its base controller and resumes its arms; on the laptop's health panel
  only. `robot_interface/robot.py`.
- **MotionRefused**: a motion refused for an operator's reason, not a bug (soft stop, no lease, a broken freeze, a
  clearance that cannot check), logged as a normal end. A **refusal** is the quiet kind: a command a part declined
  without raising, shown in the chip next to its button. `robot_interface/soft_stop.py`, `robot_interface/refusal.py`.
- **freeze**: a task's promise that other robots or objects stand still (`ctx.freeze` over a measured basis); once
  one moves past its tolerance, or its measurement gets too old, the task is cancelled (`WorldMoved`).
  `world/freeze.py`. Not the panels' **Live/Paused** button, and not a frozen dataclass.
- **clearance**: the distance kept from everything else. `ctx.clearance` is the runtime's guard: while a task drives a
  robot, it soft stops that robot and cancels the task once the robot comes within the margin (0.1 m by default) of
  anything in the snapshot, other robots as their stand-ins; unknown arm joints of its own robot trip it unless
  `unknown_arms` lets them count where last seen, or stowed. A **trip** is one firing (`ctx.clearance_trips`).
  robot_control's Clearance box turns it on for D-pad driving. `world/clearance.py`. The base planner's **path
  clearance** is a measured number instead: a planned path's nearest distance to anything, recorded in
  `experiment.json`.
- **hold**: the arm command "stop where you are and stay" (`arm.hold()`, the Hold button), which needs no lease and is
  refused only without a fresh measurement. Also: a D-pad **hold button** (counts as released 0.25 s after its last
  report), a support robot's **bar hold** (`bar_holding`), and a guided step's precondition that **holds** (below).
- **safety sync**: the onboard `multi_arm_safety_sync`, which must vouch for an arm (recent, operational, program
  playing) before any arm motion goes out; Hold is never refused for it. Not time sync, not the Drive's sync.
  `robot_interface/arm.py`.
- **simulated part**: a part (base, an arm, mocap) counts as simulated only while its once-a-second announcements keep
  coming; the monitor then relaxes its real-only checks and shows a `SIM` chip. `robot_interface/simulated_parts.py`.

## Process format

- **process**: the plan for one assembly: robots, tools, bodies, the schedule, and per movement a start state and a
  target. **Assembly** is the finished structure it builds. `core:doc/process_format.md` §1.
- **schedule / action / movement**: the schedule is the one order of all actions; an action is one robot's movements
  on one bar (`bar_jointing`, `bar_holding`, ...); a movement is one segment that runs without stopping.
- **state / target**: a state is one complete moment, readable alone; a target is the part of a state a movement
  decides, laid over its start to give its end state. `null` means "not decided by the process".
- **present / attached / built**: present bodies exist now (bars, ground, obstacles; halves with their bar); attached
  bars are held by robot links (`Holder(to, grasp)`); built bars are fixed in the structure at their assembly pose.
- **half / mount / mate**: a half is a connector piece; its mount is the bar it is fixed to for the whole process; a
  mate pairs two halves (or a half and the ground) in the finished structure. "Mount" means only this; a tool sits on
  a flange, and the operator places a bar in the tools (`place_bar`).
- **keyframe**: the solved joints stored in a target, the one planner result inside the process. Planners may not
  move it.
- **solution**: planner results in `solutions/`, tied to the content hash of what they were solved against; a
  mismatch makes them stale (shown, never executed).
- **plan checks (B1–B15) / file checks (A1–A14)**: file checks reject a file on every read and write; plan checks run
  on demand and their errors block execution.

## Planning and motion

- **plan states**: planning → ready → stale or sent, or planning → failed or cancelled. **Stale** is a ready plan
  that no longer fits the robot (the base or a joint moved). `shared/planner/plan.py`.
- **commit**: sending the ready plan as one task that keeps the robot's lease (and its freeze or clearance) for the
  whole motion. `shared/planner/planner_service.py`.
- **follower**: crl_husky's onboard pure pursuit, which drives a geometric base path. `plugins/base_planner/`.
- **xICR (turning centre)**: where the skid steer really turns, along its body, from base_footprint (negative is
  ahead). The lattice plans the turning centre, not base_footprint. `plugins/base_planner/follow.py`.
- **lattice**: the default base planner, a hybrid A* over straight drives, arcs and turns on the spot, steered by its
  **cost map** (where the robot's shape fits, learned from hits). `plugins/base_planner/algorithms/lattice.py`.
- **lead / portal**: the lead is the straight drive onto the target with no turn on the spot; a portal is a narrow
  place the lattice crosses in one move, lined up `PORTAL_LEAD` before it.
- **park / stow**: park is the base's parked floor pose (`park_pose`); stow is the arm's rest joints (`stow_joints`);
  both come from crl-husky's `park.yaml`. Inside a mirror, **parked** means moved far away because absent or
  unknown (a stand-in).

## Assembly runs

- **assembly run**: how far the assembly of one process got (actions done, where each robot was left, the run's
  settings, an event log), kept in `runs/<YYYYMMDD_HHMMSS>/` inside the process folder. Not an experiment's run
  folder, not a calibration run. `shared/runs/runs.py`, `plugins/assembly_run/`.
- **operator step / part**: an operator step is one movement of an action with the tool movements folded into it, as
  the step panel walks them (`OperatorStep`); its parts are how the executor runs each of those movements: a motion,
  a tool step, a step by hand, or refused (`PartPlan`). Not a guided procedure's step.
  `shared/assembly_planning/steps.py`, `shared/movement_exec_contract.py`.
- **live robot**: a robot measured now: in the snapshot, every movable joint measured and its base tracked, both
  fresh (`is_live`). Plan and Exec need it live. `shared/assembly_run_contract.py`.
- **transfer workaround**: a run setting: the transfer runs as a free move with empty tools, the operator places and
  grasps the bar at the insertion start after it, and the travel to load is skipped.
  `shared/assembly_planning/steps.py`.
- **Rhino obstacles**: the process's own obstacles from Rhino (bodies under `obstacles/`, e.g. the cabinet). Unless
  "Rhino obstacles collide" is ticked, they are drawn see-through and left out of every collision check; the measured
  room is the `obstacles` plugin's. `shared/process_scene/scene_bodies.py` (`process_obstacles`).

## Guided procedure (calibration)

- **guided procedure**: steps the operator walks through one at a time, each judged by checks; calibration is built
  on it. `shared/guide/`.
- **current / out of date**: a result is current while what it was built from is unchanged; time never ages data.
  Calibration says "out of date", never "stale". `plugins/calibration/run.py`.
- **precondition holds / expired**: a precondition (checklist, marker error, collision check) holds for an hour, or
  until the base moves; expired only warns at Go, except the safety boxes (area clear, e-stop at hand), which refuse.
- **acceptance**: Done twice on a failing result; stamped on its inputs and kept while they match.
- **work left**: the earliest result step that is missing, out of date, pending, or failing unaccepted.
- **procedure / Go / Auto / Preview**: the step's one long action; it may wait for Go after a Preview, and Auto skips
  the optional Gos.

## Lab and data

- **relay**: the one mocap relay per ROS domain: owns the NatNet client, turns Y-up into Z-up, applies each robot's
  base calibration and republishes per rigid body. `mocap_relay/`, `doc/mocap_relay.md`.
- **base calibration**: `base_mocap_from_base_footprint`, the one transform per robot from its base rigid body to its
  URDF's base_footprint, applied by the relay. Other calibrations say which: Motive's camera calibration, the UR
  factory calibration (applied to the model at startup), the arm mount check. `doc/calibration_manual.md`.
- **Drive**: the synced Google Drive folder (`HUSKY_DRIVE_ROOT`) that holds all experiment and calibration data.
  Driving the base is "driving", never "the Drive". `drive.py`.
- **calibration session / calibration run**: a session is one dated folder of calibration data on the Drive; a run is
  one arm's progress through the guided calibration, saved in its run file. A **lab session** is a day in the lab.
- **take**: in calibration, one recorded sweep of J0 or J1 (or one punch on its target); in mocap_bars, one recording.
- **experiment / run folder**: an experiment names a folder of run folders `<date>_<time>_<parts>/` (each with
  `experiment.json`, `recording.npz`, `report.json`); one report covers one experiment. `shared/recordings/runs.py`.
- **experiment task / attempt**: in base planner experiments, a task is a named start, target and obstacles, and an
  attempt is one plan from its start. Experiments A, B, C are named per manual (`doc/base_exp_manual.md`,
  `doc/base_planner_manual.md`): the same letter is a different experiment in each.
- **hypothesis / report / results**: a hypothesis is a numbered claim the data can refute; the generated report holds
  numbers only, the results are written from it. `doc/experiment_design_guide.md`, `doc/experiment_report_guide.md`.
- **fleet domain / sim domain**: the real robots run zenoh on ROS domain 80; a simulation uses a domain of its own,
  and every sim launch refuses 80. `crl-husky:README.md`, `doc/simulation_manual.md`.
- **temporary / virtual obstacles**: temporary obstacles are the operator's, kept in a local file until promoted into
  the lab file; virtual obstacles belong to a base planner experiment task and are never saved. Both are drawn purple
  and stop nothing physical. `plugins/obstacles/`, `plugins/base_planner/tasks.py`.

## To clean up

Words that overlap or mean too much; settle each on one name across the runtime, the core and the panels.

- **tracked / measured / known**: `base_tracked` and `unmeasured` (core `RobotObject`) both mean "not known now", one
  for the base, one per joint; the relay's `tracked` flag only means the last sample was valid and stays true when
  the relay goes silent (`pose_lost` is the real test). One word, e.g. "known", for all three.
- **guess / stand-in / default pose**: `with_guesses` fills in the acting robot, `with_stand_ins` the others, from
  the same values; the default pose is `default_position`, which is the park pose unless configured.
- **parked**: the park pose (`park.yaml`), a robot a mirror moves far away, and a parked robot in a base planner task.
- **belief / record**: the assembly run's robot records are still called beliefs inside `shared/runs/runs.py`;
  planning no longer uses them.
