# Calibration plugin: steps as modules, results that know their inputs (2026-10-21)

Status: **done, not committed; untried on a robot and in the simulator** (tested with fakes, see Tests).

## Goal

The calibration plugin's procedure gets a structure that is easy to read, extend and reuse:

- each step is one module with everything it owns: name, instructions, widgets, checks, actions, drawing;
- one way to state what a step needs: `Check` lists, judged by one rule (`blocked`);
- every result records the inputs it was made from, and is out of date as soon as one differs: no hand-written
  resets.

Behaviour, the run file and the session files stay as they were; old run files still resume.

## Problems it fixes (from the review)

- Steps existed only by naming (`_build_step_N`, `_checks_N`, `_draw_step_N` via `getattr`) and magic numbers
  (`GO_STEPS = (3, 5, 7)`, `DRIVE_STEPS`, `step == 2`); one class with ~50 attributes.
- Actions repeated their step's conditions as raised errors instead of using checks.
- The step list marked earlier steps green by index, not by whether they still pass.
- Five reset functions each listed fields by hand. Gaps: a tool body change kept the old takes; punch takes outlived
  a new Solve or Apply; the Results tab kept old figures after a take changed; `applied_while_measuring` came from
  the first take only, unchecked.
- State was held twice (plugin attributes and `Progress`), copied field by field.

## Design (as built)

```
shared/guide/          generic, for any guided procedure
  step.py              Step base class; blocked(), worst()
  procedure.py         Procedure: one at a time, Go prompts, preview path, Auto, Cancel, outcome
  panel.py             GuidePanel: step list, current step, Back / Next / Confirm, the procedure's controls
plugins/calibration/
  plugin.py            CalibrationPlugin: builds the steps and panel; switches runs; ghost; side panel
  run.py               Run: all of a run's state, its results with their inputs, save / resume / tidy
  steps/               base.py (Calibration: what steps share; CalibrationStep), s1_setup.py … s8_punch.py
  analysis/            fit.py, checks.py (was steps.py: the check functions and limits), report.py, old_outputs.py
  robot/               motion.py, driving.py (both out of plugin.py), sampling.py, sweeps.py
  storage/             progress.py, session_files.py
  views/               takes_view.py, results.py (both out of plugin.py)
```

Step modules carry their number (`s1_` … `s8_`): Python module names cannot start with a digit.

### Two kinds of checks

- `Step.checks`: the step's result as it stands now. Gates Next (this step and every earlier one), colours each
  reached step in the list.
- `Step.ready`: whether the step's actions can run now (e.g. base moved, no tool body, nothing to apply). Shown as a
  second chip row when not green; `CalibrationStep.start` refuses with its red checks; never gates Next.

Base moved is a `ready` check (Sweeps, Record), not a result check: otherwise driving in Validate would turn Sweeps
red and block Next.

### Results with their inputs

| Result | Made from | Out of date when |
|---|---|---|
| take, validation spot | `Made(tool_body, refines)` | another tool body, or a refine since |
| solution (+ output) | the files of the good current takes | that set changes |
| validation judgement | solution, current validation files | computed on demand (memoised), never stored |
| applied | the output hash the relay was seen applying | output out of date |
| punch take | the relay's applied hash when taken | relay applies another hash |

Out-of-date results stay in the run, shown as such and left out. `Run.tidy` archives this run's out-of-date take and
validation files; Refined and Solve call it. Live conditions never delete anything. Results figures are tagged with
the run and solution they came from and hidden once those are not current.

### Run file

`Progress` gained `refines`, `made` (file -> [tool body, refines]) and `punch_shas`, all with defaults; `Run.save`
is the only writer. Only a current solution's output hash is saved, so a resume that solves again finds its file.

## Tests

- `test/shared/test_guide.py`: Next / Confirm / Back, earlier red steps, ready vs checks, the list, Procedure.
- `test/plugins/calibration/test_calibration_run.py`: every row of the table, resume, an older run file.
- `test/plugins/calibration/test_calibration_plugin.py`: the plugin on a real viser server with fakes: the steps,
  a tool body change turning Record and Solve red, resume; and every procedure (Record on Auto with a tool body on
  tool0, Solve, a validation spot, Apply, a punch take).
- `test/calibration_fixtures.py`: Belle's 20261013 takes for both.

## Open

- A run in the simulator (`doc/calibration_manual.md`, Testing in a simulation) before the first lab run.
- `GuidePanel` is generic but used by calibration only; a second user (e.g. a mocap_probe procedure) would show
  whether its hooks (`before_forward`, `on_override`, `on_step`) fit.
