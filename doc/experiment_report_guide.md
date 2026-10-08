# JG's Experiment Design Guide, part 2: reports

How the generated reports of an experiment are written, for anyone (human or agent) who writes or changes a report
generator. Part 1 (`experiment_design_guide.md`) covers the question, the method, the collection and the analysis
script. Examples come from `plugins/base_exp/analysis/` (`summary.md`, `robot_model.md`).

## Two layers, kept apart

1. **The report** (generated): why, what was measured, then numbers. It never interprets.
2. **The results** (written): a subagent reads the reports with the prompt saved beside them (`results_prompt.md`)
   and writes `results.md`: what the numbers say, caveats, next steps.

An interpretation in the report ("better, but not perfectly", "one model may mix several", a finding picked by code)
blocks the second layer: the reader can't tell measurement from opinion, and a wrong pick spreads.

## The order of a report

1. **Title and one data line**: experiment, number of runs, dates, what was left out.
2. **Why** (static): the goal, the hypotheses tested, and numbered questions, each pointing to the section that
   answers it.
3. **What we measured** (static): the conditions held fixed, every metric with its unit, what counts (outcomes,
   success), the comparison rules (what bold means), and known effects the reader must keep in mind, as `!` notes
   (e.g. a skid steer's tracked point circles on a turn on the spot). Then the scenario table: what was run.
4. **Data**: one section per question, each with a one-line goal and its tables.

Static means no numbers taken from the data in prose. Constants such as the speed or a tolerance are fine.

**Self-contained**: every report has its own definitions and scenario table. Point to another report only when the
data lives there.

## One goal per table

Before writing a table, write the one line it answers; that line goes above it.

- Two tables that answer one question with the same columns are one table.
- Keep only the columns the goal needs; details go to the CSV.
- Reference and check rows (an ideal simulator) form their own group or section. They must not shrink the set of
  paths the real comparison uses.

## Comparisons and bold

- Decide the comparison group first (e.g. the controllers on one robot). Its rows sit together, the group label shows
  once, and an empty row separates the groups.
- Compare like with like: the same paths, each path once. Where the sets differ, show it (e.g. "own" paths) and give
  no bold.
- Bold the best value in a group only when the gap is larger than the spread between repeats. No bold for ties, for
  single rows, or where the question is whether two rows are close (simulator against real).
- Show the spread next to the mean ("9.6 ± 2.1") wherever repeats exist, so the reader can judge.

## Counting

- Say what n is (runs, paths, starts). With unequal repeats, average per path first, so one path doesn't weigh more.
- Runs the operator stopped count neither way. A run the controller never finished (timed out, circling the goal)
  is a failure.
- Break down only by units that start clean. A path template starts at the robot's pose; a segment inside a path
  carries the error the one before left, so segment breakdowns bleed.

## Missing data

Missing data must look missing: "not run", "not collected", "none done", "—". Never 0 or a blank cell. In a bar plot a
missing bar looks like zero error: label it.

## Formatting

- Units in the column headers; the same decimals for a quantity in every table.
- The same names and order everywhere: scenarios, templates, path labels (one function makes each label).
- No `|` inside a Markdown cell: it splits the cell.
- Failures as a table grouped by scenario, path and outcome, with counts, how far the run got, how far off it went,
  and short links. Not as a list of near-identical lines.
- Each plot answers one question, said in one line in the plots section. Leave out series that add only zeros, and
  say so.

## Files

- One CSV per table, named after it. The results prompt names the files to read, so stale files from older versions
  don't mislead.
- The results prompt fixes the sections of `results.md` and its rules: only what the data shows, with its numbers; a
  difference within the spread is no difference.

## Checklist

- [ ] Why and What we measured hold no data-derived numbers or judgements.
- [ ] Every question in Why points to a section, and every section answers one.
- [ ] Every table has a one-line goal above it, and no two tables repeat one another.
- [ ] Bold only within a group, on the same paths, beyond the spread.
- [ ] Groups separated by empty rows; reference rows apart.
- [ ] Missing data labelled; no zeros for "not run".
- [ ] Names, order and decimals the same across tables; tables render (no `|` in cells).
- [ ] The report reads on its own, without the other reports.
