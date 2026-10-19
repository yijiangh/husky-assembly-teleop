"""
For plugins to share code, share data and offer services to each other; nothing else goes here.

- ! Plugins never import each other: what two plugins need lives here, and a plugin reaches another's live data
  only with `ctx.require`, for names in its `requires`.
- ! Holds types, interfaces and helpers, never a plugin's live state, and imports nothing from `plugins/`.

  planning/  the planner plugins' base class, search and timed path
"""
