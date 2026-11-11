# Calibration: resume at the first open step, evidence that expires (2026-10-22)

Status: done, untried on a robot (tested with fakes).

## Goal

A resumed run opens where work is left, and the operator rechecks only what has gone stale. A normal run relaunched
for the tool change before the punch lands at Setup with one box to tick (Tool on flange), then goes on at Tip offset.

## Rules (agreed with the user)

- **Results and evidence.** Record, Solve, Validate, Apply, Tip offset and Punch keep results with their inputs (as
  before). Setup's checklist, Marker error and the collision check are evidence a step needs before it acts. Evidence
  expires; results go out of date only when their inputs change.
- **Expired evidence is amber in its own step**, never red, so Next does not jam on later steps. It is red in the
  ready row of each step that needs it (`CalibrationStep.needs`), which then refuses to act.
- **Resume** goes to the first step with a red check not covered by an earlier override, and from there to the first
  expired step it needs.
- **Expiry**:
  - Setup's checklist: each box 1 h after it was ticked (the run file keeps each box's time). Tool on flange holds
    only for the tool model it was ticked for, so a relaunch with the punch asks for that one box again. The boxes
    show whether they hold; Setup's chip is amber when every box was ticked but some lapsed (naming which and when),
    red while a box was never ticked.
  - Marker error: 1 h after the last measured tick, and on a refine.
  - Collisions: when the base moves (as before), or a check is cancelled half-way.
- **One source each.** `Run.checklist_holds`, `marker_holds` and `collisions_hold` say why evidence lapsed; each
  evidence step's `stale` returns that text, and its own chips show it amber. Evidence times come from
  `Calibration.wall_now`, which tests replace.
- **What results depend on**:
  - Record: the tool body and refines of the **base** body only (a tool-body refine moves the tool point rigidly on
    the flange, so the axes are unchanged; to confirm with calib_bench).
  - Validate: the tool body and refines of **either** body; not the solution (samples are re-judged under each) and
    not the relay's calibration (taken off the samples).
  - One predicate per result kind (`take_current`, `validation_current`, `punch_current`) filters what counts
    (`current_takes`, `current_validation_files`, `current_punch_takes`) and decides what `tidy` and
    `add_punch_take` archive.
  - Punch takes: the relay's applied calibration, the base body's refines and the TCP offset (kept per take in the
    run file; an older run file's takes count with the offset each holds). A take after one that no longer holds
    starts a new punch file.
  - Tip offset: nothing the monitor can see (clear it after remounting the punch).

## Needs

| Step | Needs fresh |
|---|---|
| Marker error, Validate, Tip offset, Punch | Setup |
| Validate | Marker error |
| Record | Setup, Marker error, Sweeps |
