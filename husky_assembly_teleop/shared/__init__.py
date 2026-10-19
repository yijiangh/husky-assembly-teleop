"""
What plugins share with each other: contracts, data types, base classes and helpers; nothing else goes here.

`plugin_api/` is what the monitor gives every plugin; this is what plugins give each other.

- ! Plugins never import each other: what two plugins need lives here, and a plugin reaches another's live data
  only with `ctx.require`, for names in its `requires`.
- ! Holds types, interfaces and helpers, never a plugin's live state, and imports nothing from `plugins/`.
- What a plugin offers others is a Protocol named `<Name>Contract` in a `*_contract.py` file; its users type
  `ctx.require("<name>")` with it: `process: ProcessContract = ctx.require("process")`.

  process_contract.py  what the process plugin offers: its selected step (`Selection`, `ProcessContract`)
  loaded_process.py    a process loaded for the monitor: its steps, and the scene bodies and ids of a step
  planning/            the planner plugins' contracts, base classes and helpers
"""
