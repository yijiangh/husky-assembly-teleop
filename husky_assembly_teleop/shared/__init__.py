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

  arm_planner_contract.py   what the arm planner offers: plans for one arm (`ArmPlanRequest`, `ArmPlannerContract`)
  base_planner_contract.py  what the base planner offers: base paths (`BasePlanRequest`, `BasePlannerContract`)
  obstacles_contract.py     what the obstacles plugin offers: the lab as in the scene (`ObstaclesContract`)
  process_contract.py       what the process plugin offers: its selected movement (`Selection`, `ProcessContract`)
  base_collision/           what a robot base at a floor pose comes near (`BaseCollisionWorld`)
  planner/                  base classes and helpers for the planner plugins
  recordings/               experiment runs on the Drive: run folders, descriptions, recordings read by label
"""
