# Agent notes for husky-assembly-teleop

Project-wide instructions for coding agents. Keep them generic: no personal paths or per-user preferences.

## Principles

- Simplicity first: make every change as simple as possible and touch as little code as possible. Reuse existing functions instead of writing new ones.
- Code is read by humans and agents: write comments, in plain language without jargon (no "no-op").

## Code style

- Google-style docstrings with type hints in the signature (`def fn(a: float) -> int:`).
- Comments use Better Comments markers: `!` for rules a caller must follow, `?` for open questions, `*` for highlights and section headers.
- Imports at the top of the file; prefer `from x import fn` over `import x; x.fn()`.
- Lines up to 120 characters.

## Writing comments and docstrings

A few well designed sentences are worth more than three paragraphs: long docstrings hide the one fact that matters.

1. First list the key messages as bullets in your head. Ask what a caller or maintainer must *know or do*. Everything else goes.
2. Summary line: one sentence saying what it does. Fold the qualifier in when you can, e.g. "Run the intents queued when the drain starts; later ones wait a tick."
3. Keep `!` warnings only for rules a caller must act on ("Always use this instead of `gui.add_panel()`: only panels made here are removed on teardown"). State the rule and give the reason in one clause.
4. Cut:
   - rationale essays, and "why not X" debates
   - history ("the old arrangement…", "the first draft…")
   - repeats of what the code or signature already shows
   - examples inside prose ("releasing a gripper, re-enabling a controller")
   - pointers to design docs, unless there's nowhere else to find the reason
5. If there are several independent rules, use 2–3 short `-` bullets, not paragraphs.
6. Aim for about 1–3 sentences in total. A module docstring gets a few sentences plus any import or threading rule.
7. Keep Google-style Args/Returns/Raises/Example sections and the Better Comments markers. Plain language, no jargon.
8. When code changes, fix comments that went stale (e.g. references to removed methods).

When briefing a subagent to write or edit comments, paste this section into its prompt.

## Where things are

- `README.md`: install, run, parameters, plugins.
- `external/scaffolding_assembly_core/`: the core shared with the Rhino plugin and the planners: the process format (`process/`, schema 2.1), robots, scenes and their mirrors; layers in its `scaffolding_assembly_core/__init__.py`, the process format in its `doc/process_format.md`. A submodule of `yijiangh/scaffolding_assembly_core`, installed editable in the venv.
  - ! It imports nothing from `husky_assembly_teleop`, ROS or viser, and runs on Python 3.9: change it in the submodule following its own `AGENTS.md`, run its own tests there (`cd external/scaffolding_assembly_core && python3 -m pytest`), commit and push it, then commit the new submodule pointer here.
- `husky_assembly_teleop/plugins/__init__.py`: the rules every plugin follows. `plugins/robot_control/` is the reference plugin, `plugins/examples/` the templates.
- `husky_assembly_teleop/shared/`: for plugins to share code, data and services; plugins never import each other. `test/structure/test_layers.py` checks that imports only go down the layers.
- `doc/refactor_rationale.md`: why the core is built the way it is. `doc/plugin_roadmap.md`: which old features are still to be ported. `doc/scene_refactor_plan.md`: the scene.
- `doc/experiment_design_guide.md` and `doc/experiment_report_guide.md`: JG's Experiment Design Guide: how to plan, build and analyse an experiment, and how its generated reports are written.
- `husky_assembly_teleop/old/`: the old monitor, reference only. It does not run; never import it from new code.
- Docs marked outdated at the top describe the old monitor.

## Build and test

Run everything from the workspace root (the folder holding `src/`, `venv/`, `install/`), in its venv, with ROS sourced. The monitor needs `src/crl-husky` built in the same workspace.

```bash
source venv/bin/activate
source /opt/ros/humble/setup.bash
# Build with the venv's Python, never the bare `colcon`: otherwise the installed scripts run the system Python.
python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop
source install/setup.bash

cd src/husky-assembly-teleop
python3 -m pytest test/plugins/base_planner   # while iterating: only the area you change (seconds)
python3 -m pytest                             # quick set (~15 s): no slow tests, no linters
python3 -m pytest -m safety                   # before every lab session: command guards, frames, layers, linters
python3 -m pytest -m ""                       # everything (~1 min), before every commit
```

- ! Run everything (`-m ""`) before every commit, and commit only when it passes.
- `test/` mirrors the package: `test/<area>/` for `robot_interface`, `world`, `ui`, `plugin_api`, `config`, `mocap_relay`, `core` (the core's mirrors as used here), `experiments`, and `test/plugins/<plugin>/`. `test/structure/` holds the layer test and the linters.
- Mark a test `safety` when it guards a command to a real robot, or the frames and models commands rely on. Mark it `slow` when it takes over a second or checks offline analysis; every test under `test/experiments/` is slow.
- Helpers for several folders (`repo_paths.py`, `*_fixtures.py`) sit in `test/` itself and import by name.

## Task notes

At the end of plan mode, write the spec and instructions to `tasks/<yyyy-mm-dd>_<topic>.md`. Update it when the plan changes during the work.
