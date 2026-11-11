"""
The base calibration: the base rigid body's base_footprint, measured by sweeping one arm's first two joints.

  plugin.py   the plugin: the steps in a guided panel (`shared/guide/`), the ghost, the side panel
  run.py      one run's state: what it measured and found, which of it is current, its run file
  steps/      the steps, one module each, numbered in their order: widgets, checks, actions and drawing
  analysis/   the math (kept as in the old offline pipeline), the checks and their limits, the written results
  robot/      moving and measuring the arm: moves with Go, the D-pad, samples, the predefined sweeps
  storage/    the run file and the session's files on the Drive, in the old pipeline's formats
  views/      the takes' chips and arcs, and the side panel

`scripts/solve_calibration.py` solves a session again offline.
"""

from . import plugin  # noqa: F401  registers CalibrationPlugin
