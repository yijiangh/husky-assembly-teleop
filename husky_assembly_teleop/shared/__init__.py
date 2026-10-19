"""
For plugins to share code, share data and offer services to each other; nothing else goes here.

- ! Plugins never import each other: what two plugins need lives here, and a plugin reaches another's live data
  only with `ctx.require`, for names in its `requires`.
- ! Holds types, interfaces and helpers, never a plugin's live state, and imports nothing from `plugins/`.
- What plugin `<name>` offers others is a Protocol named `<Name>PluginApi`, e.g. `ProcessPluginApi`; its users type
  `ctx.require("<name>")` with it: `process: ProcessPluginApi = ctx.require("process")`.

  planning/  the planner plugins' base class, search and timed path; process movements, steps, planning scenes,
             collision checks and IK for the process planners; the motion planner service (`MotionPlannerPluginApi`)
  process.py a process and its steps as data, and what the process plugin offers (`ProcessPluginApi`)
  runs.py    the `runs/` record of an assembly: actions done, where robots were left, settings
  assembly_run.py  what the assembly_run plugin offers (`AssemblyRunPluginApi`), and the rules it marks actions by
"""
