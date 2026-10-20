"""
What plugins share with each other: contracts, data types, base classes and helpers; nothing else goes here.

`plugin_api/` is what the monitor gives every plugin; this is what plugins give each other.

- ! Plugins never import each other: what two plugins need lives here, and a plugin reaches another's live data
  only with `ctx.require`, for names in its `requires`.
- ! Holds types, interfaces and helpers, never a plugin's live state, and imports nothing from `plugins/`.
- Contracts sit in this folder, one file per plugin that offers something: plugin `<name>` offers a Protocol
  `<Name>Contract` in `<name>_contract.py`, with the types it hands out. Its users type `ctx.require("<name>")` with
  it: `process: ProcessContract = ctx.require("process")`.
- Shared values are replaced on every change, never edited, so a user may cache one and compare by identity.
- Subfolders hold base classes and helpers. ! They never import a contract file, except for type hints.

  arm_planner_contract.py     what the arm planner offers: plans for one arm (`ArmPlanRequest`, `ArmPlannerContract`)
  assembly_run_contract.py    what the assembly_run plugin offers: the run's record (`RunView`,
                              `AssemblyRunContract`), and the rules it marks actions by
  base_planner_contract.py    what the base planner offers: base paths (`BasePlanRequest`, `BasePlannerContract`)
  motion_planner_contract.py  what the motion planner offers: plans, IK and collision checks for the process's
                              movements (`MotionPlanRequest`, `CollisionReport`, `MotionPlannerContract`)
  movement_exec_contract.py   what the movement_exec plugin offers: running a planned step on the robots
                              (`ExecRequest`, `ExecResult`, `MovementExecContract`), and a step's parts
  obstacles_contract.py       what the obstacles plugin offers: the lab as in the scene (`ObstaclesContract`)
  process_contract.py         what the process plugin offers: its selected movement (`Selection`, `ProcessContract`)
  assembly_planning/          the assembly's planning: the process as steps, routines, planning scenes, IK and
                              collision checks on a mirror pool, the tamp route
  base_collision/             what a robot base at a floor pose comes near (`BaseCollisionWorld`)
  execution/                  running a planned movement on the robots: timing, sending, tools, the operator's
                              prompt, the robot lease, and when a plan goes stale
  planner/                    base classes and helpers for the planner plugins
  process_scene/              the process scene at a movement as live scene bodies, and the colours of bodies
  runs/                       the `runs/` record of an assembly: actions done, where robots were left, settings
"""
