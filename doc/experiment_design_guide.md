# JG's Experiment Design Guide

How to plan, build and analyse an experiment on the robots, for anyone (human or agent) who sets one up. Two parts:

- **Part 1** (this file): the three stages, from the question to the analysis script.
- **Part 2** (`experiment_report_guide.md`): how the generated reports are written.

Examples come from the base controller experiments (`doc/base_exp_manual.md`, `plugins/base_exp/`).

## The three stages

| Stage | What | Deliverable | Done when |
|---|---|---|---|
| **1. Question and method** | Co-author the hypotheses and how to check them | The experiment manual's first two sections: "Goal and hypotheses", "The experiments" | The user agrees, and every hypothesis has a run plan and a report section that would decide it |
| **2. Collection** | The plugin that runs and records the experiment | Plugin code with tests; the manual's operating sections | A pilot round, collected in the simulator end to end, loads in the analysis |
| **3. Analysis** | The script that turns all runs into reports | Analysis script, report generators, results prompt | The reports on pilot data pass the checklist of Part 2, and every check passes |

Finish a stage's deliverable before starting the next. Going back is normal: when stage 2 or 3 shows the method is
wrong, fix the manual first, then the code.

---

## Stage 1: question and method

The agent proposes, the user decides. Write it down in the manual as you agree on it, not afterwards.

### 1. Goal

One sentence: what the result is for. E.g. "a base controller that follows paths accurately enough for assembly,
including tight passages."

### 2. Hypotheses

Numbered, each one a claim the data can refute. For each, agree on what would refute it before collecting:

- Name the deciding number and, where you can, its threshold. "Good enough" needs a number, e.g. "the end error
  well below the passage's clearance".
- One claim per hypothesis. "The model changes with speed, arm and floor" is three claims, so say which one each
  experiment tests.

### 3. The experiments

One table, a row per experiment: question, hypothesis it tests, runs to collect, report. Split experiments that
answer different questions, even when they use the same robot. Name them so their data never mixes (base_exp: model
and controllers per condition and speed, constant commands per condition, under separate names).

### 4. The method, per experiment

Answer each of these in the manual:

- **What varies, what is held fixed, what is measured.** Held-fixed conditions are recorded with every run, so that
  runs made under other conditions can be left out later.
- **Measured independently of what is tested.** The measurement must be the same for every variant: base_exp's
  monitor measures every controller with mocap against the path sent, not with the controller's own error.
- **The unit of comparison.** A run that starts clean, e.g. a path planned from the robot's pose. Parts of a run
  carry over each other's errors.
- **A standard set.** The same inputs in every scenario, so scenarios compare one to one. Random inputs only add
  coverage.
- **Repeats.** How many per input and scenario, and why. A real robot needs several (aim for 5) to show the spread;
  a deterministic simulator needs one.
- **References.** A setup with a known answer to compare against, e.g. the ideal simulator, or the controller with
  the ideal model.
- **Checks with known answers.** Before trusting a real result, the method must give back what it was given: a
  simulator running a known model, synthetic data. A method that fails there says nothing about the robot.
- **Effects the system has anyway.** Name them before measuring so they aren't read as results. E.g. a skid
  steer's tracked point circles on a turn on the spot, whatever the controller does.
- **Failure.** What counts as a failure of the thing under test (it timed out, it circled the goal and never
  settled), and what is a problem of the setup (e-stop, lost tracking) that leaves the run out.
- **Order and cost.** The simulator first: it's free and shows what to expect. Plan real-robot time for the scenarios
  only the real robot can answer, and know which ones are missing.

### Pitfalls

- A metric that can't tell the hypotheses apart. Ask what each number would be under each hypothesis.
- One experiment answering two questions: its pooled data then answers neither cleanly.
- No decision criterion: the results then become a matter of opinion.

---

## Stage 2: collection

### The plugin

- **Record raw, decide later.** Each run gets its own folder (`<date>_<time>_<robot>_<kind>_<sim|real>/`) holding
  the raw signals (`recording.npz`), a description (`experiment.json`: setup, conditions, parameters, outcome,
  metrics) and an overview plot. The analysis recomputes everything from the raw data.
- **Record what defines the scenario, automatically.** Simulator or real, models, controller and its parameters,
  conditions, experiment name. Never rely on folder names or memory. Conditions that can't be set in the panel are
  code constants, recorded with every run (base_exp `CONDITIONS`).
- **Times with their clock.** Record when each signal was captured and when each command was sent, and which host
  stamped it. A stamp can still trail the event it marks: base_exp's negative wheel delay came from wheel odometry
  stamped about 150–200 ms late on the robot, not from clocks (robot and monitor agreed within about 20 ms).
- **Outcomes, explicit and distinct.** E.g. done, timed out, soft stopped, stopped by the operator, e-stopped, each
  with a reason text. A failure of the thing under test is recorded and the series goes on. A problem of the setup
  pauses the series and says why.
- **Automation.** It runs the standard set, resumes where the experiment stands, and counts totals per experiment and
  setup, so stopping and restarting is safe and the repeats stay equal.
- **Leave out, never delete.** A bad run stays on disk and is listed with its reason in `_excluded.txt` in the
  experiment folder.
- **Safety.** The simulator runs on its own isolated ROS domain. On the real robot, a guard stops near obstacles, and
  the e-stop stays within reach.
- **Tests.** Test the measurement on its own, with synthetic data and simulator runs whose answer is known.

### Where data lives

Runs on the project's shared Drive, one folder per experiment: `data_experiment/<plugin>/<experiment name>/`.
Reports go to its `_analysis/` folder. Create folders in Drive, never in a local copy.

### The manual's operating sections

- **Setup**: what every terminal needs (venv, ROS, environment variables), and the build.
- **Simulator** and **Real robot**: the terminals to start, with exact commands; safety first.
- **Collecting runs**: the panel, step by step.
- **Per experiment**: numbered steps, with an analysis step where a result is needed before going on (base_exp A:
  analyse, save the model, then run the simulator with it).
- **Troubleshooting**: a table of symptom, then cause and fix.

### Pilot

Collect one round in the simulator, run the analysis, look at the overview plots of a few runs. Only then go to the
real robot.

### Pitfalls

- Unequal repeats from stopped series: count totals, not runs per start.
- Repeating a deterministic simulator: the time is better spent on real runs.
- A scenario nobody collected: list the required ones, and make the report show what's missing.

---

## Stage 3: analysis

### The script

- **One script, all the data.** It reads every run folder under the data root, filtered by experiment name and
  protocol (speed, conditions, exclusions), and says what it left out and why.
- **Load once, label from the recordings.** Robot and scenario names come from the recorded description, each made
  by one function, so every table uses the same names.
- **One module per report.** Each writes its Markdown report, a CSV per table and its plots into `_analysis/`; the
  results prompt goes beside them (Part 2).
- **Repeatable.** The same data gives the same reports, with no manual steps. Trial runs go elsewhere (`--out`), never
  over the real reports.
- **Checks every time.** The known-answer checks from stage 1 are computed with every analysis and shown in the
  report.

### Results

The reports hold numbers only. For the written results, a subagent reads them with `results_prompt.md` and writes
`results.md`. Its interpretation then gets reviewed against the reports.

### Review and iterate

After the first real data, have the reports reviewed (by the user, or by an agent with Part 2's checklist). Fix the
generator, never a report by hand. Record decisions and their reasons in the task note (`tasks/<date>_<topic>.md`).

---

## Checklists

**Stage 1**
- [ ] Goal in one sentence; numbered hypotheses, each with what would refute it.
- [ ] One experiment per question, with names that keep their data apart.
- [ ] Varied, fixed and measured quantities named; measurement independent of what is tested.
- [ ] Standard set, repeats, references and known-answer checks planned.
- [ ] Inherent effects and the meaning of failure written down.

**Stage 2**
- [ ] Every run records raw data, its full setup, conditions, times with their clock, and an explicit outcome.
- [ ] Automation resumes and keeps repeats equal; bad runs are excluded, not deleted.
- [ ] Measurement tested on known answers; simulator isolated; real-robot safety in the manual.
- [ ] The manual's operating sections let someone else run it; a simulator pilot is collected.

**Stage 3**
- [ ] One script analyses everything and says what it left out.
- [ ] Reports pass Part 2's checklist; the checks pass on pilot data.
- [ ] The results prompt is written; the decisions are in the task note.
