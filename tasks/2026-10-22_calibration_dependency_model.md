# Calibration: one dependency model for results, preconditions and navigation (2026-10-22)

Status: agreed with the user, to build. Picture: the "Calibration Dependency Map" page (claude.ai artifact
SFQyLGAXUBDRKegGuDHEYP). Replaces the resume/expiry rules of `2026-10-22_calibration_resume_and_expiry.md` where they
differ.

## The model

Two relations:
- **Built from** (data): a result is current while what it was built from is unchanged; else **out of date**, and so is
  everything built on it. Work starts again at the earliest out-of-date result. Time never ages data; putting the input
  back makes it current again.
- **May only act while** (preconditions): checked when a step acts; it **holds** or has **expired**. It expires when it
  has become likely that something changed (an hour, the base moved, another tool model launched), so a days-old tick
  is never copied forward. It never makes a result out of date.

Five kinds of thing (only stored things go out of date):
- **Measurements**: takes, validation samples, tip poses, punch takes. Each records what it was made with: tool body,
  base and tool body definitions (as the relay announces them, stored with the measurement and compared only against
  the current ones, no history in between: a data change ("frame moved", "refined", "redefined", "other body") makes
  it out of date; on the lab's Motive 3.0.3.1 a refine sets no "tracked models changed" flag, so the definitions are
  read again every 2 s), punch mount (a counter bumped when the operator says it was remounted), punch target (a
  counter bumped by "Target moved").
- **Calculations**: the solution and every verdict (take fits, validation, tip fit, punch spread). Worked out again
  from what is current: pending, passes or fails, never out of date.
- **Effects**: the calibration file Solve writes, what the relay applies, the TCP offset Done commits. Out of date when
  they differ from what the current data gives.
- **Acceptances**: "Done anyway" on a failing result, stamped on the inputs it accepted; gone when they change.
- **Preconditions**: Setup's checklist (each box 1 h; Tool on flange per tool model), markers small under motion
  (measured within 1 h and not red), the collision check (where the base stands).

Words: data **current / out of date**; preconditions **holds / expired**; calculations **passes / fails / pending**;
acceptances **accepted**. Never "lapsed" or "stale" (rename `stale()` to `expired()`).

## Decisions (user, 22 Oct)

1. Back and Next only move: never refused (they only wait while a procedure runs: "Cancel first"), never write,
   never start work. **Done** saves and jumps to the work that is left. On a failing result without an acceptance
   it turns into **Accept as is** for 5 s, which stamps the acceptance (shown "accepted" in the overview); on a
   result missing or out of date into **Skip**, which goes on without a stamp: only a failing result can be
   accepted.
2. The goal is fixed: the calibration (Setup to Apply), then the punch check (Tip offset, Punch).
3. "Validate before Apply" stays: Apply needs the validation verdict to pass or be accepted.
4. Validation samples and punch takes are raw measurements, judged under the current solution. Punch takes store the
   base body's own pose, the joints, the TCP offset used and what the relay applied, so they can be judged under any
   solution and offset; they stay current through new solutions, offsets and Apply. Show the punch verdict under the
   current solution and, end to end, under what the relay applies. Validation may be sampled before Solve.
5. Readiness is per step. Don't block user input: chips warn, and acting with an expired precondition asks twice.
   A robot move is refused only when: a live safety condition fails (lease, soft stop, clearance, base mocap: already
   enforced below the UI); Record has no collision check where the base stands; or "Area clear" and "E-stop at hand"
   were not confirmed within the hour (a button in the Go prompt confirms them again and re-stamps those boxes; Auto
   stays off until then). Besides, an action is refused where it is impossible: no tool body, no clear take, no
   validation poses, an implausible TCP offset, no relay, the wrong tool model for a tip pose or punch take, Apply
   without a solution or a good verdict.
6. Marker error exists to make sure the errors stay small under motion: the precondition is "measured within the hour,
   driven DRIVE_SECONDS, and not red".
7. Choosing another tool body changes no precondition (it still puts takes and samples out of date).
8. A punch remount cannot be seen: revisiting Tip offset with poses asks "Punch remounted since?"; yes bumps the mount
   counter (tip poses and punch takes out of date). Clear tip poses clears the poses and the fit, not the committed
   offset; only Done in Tip offset commits the fit as the TCP offset (typing it in Punch stays possible).
9. A Motive camera recalibration is not a data change.
10. A refined or redefined base body never invalidates the applied calibration; the monitor warns it was made for an
    earlier body (once the relay reads body definitions; later, not in this task).
11. Every mocap sample stores the marker errors reported with it (takes and validation samples both bodies', punch
    takes the base body's; older punch files show it unknown), and the takes, validation and punch views show each
    sample's, red over MARKER_ERROR_LIMIT. The punch check shows the live base error and has no marker precondition:
    a take made while it was red is flagged in the verdict, never refused. Validate keeps its markers precondition.

## Work left

From the first step: the earliest step whose result is missing, out of date, or failing without an acceptance. If that
step's preconditions have expired, go to the earliest expired precondition step first and remember the target; Done
there returns to it. Live conditions (mocap, relay, soft stop, tool model, pendant TCP) are chips only and never move
the operator. Resume uses the same rule.

## Steps and their preconditions

| Step | Preconditions | Robot move refused also when |
|---|---|---|
| Marker error | Setup checklist | Area clear / E-stop not confirmed within the hour |
| Record | Setup checklist, markers, collision check here | no collision check where the base stands; Area clear / E-stop |
| Validate | Setup checklist, markers (collisions checked per spot) | Area clear / E-stop |
| Tip offset, Punch | Setup checklist | Area clear / E-stop |
| Apply | verdict passes or accepted; "nobody is driving" confirmed in Apply's own prompt | (moves no robot) |
| Setup, Sweeps, Solve | none | (move no robot) |

## Nothing runs on its own

- Sweeps' collision check runs from its button only (no check on entering).
- The run file starts with the first thing recorded (a take, a sample, a tick saved), not on leaving Setup.
- A mount check counts as applied once its file is written (not on entering Apply).
- Showing a step changes no data; the run file is not saved on navigation. `reached` goes: every step shows its state.

## Overview (step list)

Each row shows its own state on a fixed line (AGENTS.md "Panels: nothing jumps"): result steps solid (current,
out of date, none yet, passes/fails, accepted); precondition steps dashed (holds, expired with why and when). The
current step outlines the precondition steps it needs that have expired. Phone page: the same list and Done; Go,
Auto and Preview stay on the laptop.

## Stages

A. **Model** (calibration plugin `run.py`, `storage/`, `analysis/`, steps): vocabulary and `expired()`; measurements'
   records (punch mount and target counters; raw punch takes with backward-compatible reading of old punch files);
   calculations always recomputed; the TCP offset as an effect committed by Done; validation before Solve;
   acceptances stamped on inputs (old label overrides are read, not used); markers include "not red"; Apply's
   preconditions; tests; run.py docstring tables; manual sections touched.
   **Done** (branch `jg/calibration-model`). For Stage B, in `Run`: `inputs(step)`, `accept(step)`,
   `accepted(step)`; `commit_tip_offset()` (for Tip offset's Done), `tip_offset_used()`; `remounted()` (for the
   "Punch remounted since?" question), `target_moved()`; `validation_verdict()`, `validation_passes()`;
   `footprint(relay)`, `punch_tips(footprint)`; `marker_age(now)` (time only) beside `marker_holds(now)`; and
   `CalibrationStep.expired(ctx)`. Decided differently or beyond the note:
   - Every step has an acceptance stamp, Setup, Marker error, Sweeps and Apply too, so the second Next press keeps
     working everywhere (it accepts every step named in the red checks). Marker error's is stamped on the largest
     errors and the refines; Punch's on its current takes, the TCP offset and the solution's hash, since its verdict
     depends on them.
   - Takes from an older punch file (no base body pose) are judged under the base_footprint they stored, but with the
     TCP offset now, not their stored tips.
   - Takes stay current through a new offset, so a punch file can hold takes with different offsets; each keeps the
     one it used, and `4_punch_validation.py` refuses such a file.
   - Tip offset's "Use this offset" button commits the fit (refused while a check is red and not accepted); its
     "in use" chip is amber while Punch uses another offset. "Punch remounted" (Tip offset) and "Target moved"
     (Punch) are plain buttons for now.
   - Apply's Go confirmation is not skipped by Auto. The run file drops `punch_shas` and `punch_offsets` (each punch
     take holds its calibration hash and offset). `EVIDENCE_HOLDS` is now `PRECONDITION_HOLDS`.
   - Left for Stage B: a mount check still counts as applied on entering Apply (done there: once written).
B. **Guide and navigation** (`shared/guide/`, plugin, steps): Back/Done/Next; work left and the return target; the
   overview rows; warn-don't-block with the safety refusals and the Go-prompt confirmation; no side effects on
   navigation; phone page; tests; manual (panel, steps, resuming).
   **Done** (branch `jg/calibration-model`). The generic guide knows a step's `state()` (a Check: its label is the
   state; BAD or PENDING is unfinished, so Done asks once more), `precondition`, `needs` and `expired()`; the panel's
   one hook is `on_done(ctx, anyway) -> index`. The plugin's `work_left` works the return target out afresh. Decided differently
   or beyond the note (as revised by Stage C):
   - Back, Done and Next still do nothing while a procedure runs ("Cancel first"): leaving its step mid-take would
     hide what it draws. The target is not stored: Done in one of the work's expired preconditions (e.g. the checklist
     not ticked, also when reached by hand) goes on to the next expired one after it, then to the work: warn, don't
     block (`work_left(from_here=True)`). Done elsewhere goes to the earliest expired one. Done whose work left is the
     current step goes to the next step.
   - Done asks once more on any unfinished state ("none yet", "out of date", "fails", "pending"); only "fails" is then
     accepted. Apply needs `Run.validation_good`: the verdict passes, or fails with current samples and solution and
     is accepted.
   - Extra states: Validate "pending" (no red check, no current solution), Tip offset "not in use" (amber, no work
     left: an offset typed in Punch stays in use; only Done in Tip offset commits the fit). Preconditions also show
     "none yet" (never measured) and "fails" (Marker error red or driven too short, Sweeps too few clear takes: Done
     accepts). Setup's state is the checklist's alone.
   - The remount question is a warning below the controls (`Step.question`, laptop and phone), answered by Punch
     remounted (now on the phone too), or "no" by recording a pose or one Done, which accepts nothing. "Use this
     offset" is gone: Tip offset's Done commits the fit.
   - Apply's state follows what the run applied (`Run.finished`); the relay's live hash is a "relay now" chip only
     (amber when another calibration is applied, grey before an announcement), never work left.
   - Punch's acceptance also stamps the base_footprint its verdict is judged under (`CalibrationStep.judged_under`).
   - Solve and a changed body definition archive out-of-date take and validation files but the run keeps them (not
     while a procedure runs); Solve moves current ones back, so the session's folders hold what Solve used and a
     measurement whose input comes back is current again.
   - The safety boxes are a ready chip "area, e-stop" on the steps that move the robot. The D-pad, free drive and the
     tool pad refuse with "tick them again in Setup" (they have no Go prompt); takes and validation spots ask at Go.
     Recording a tip pose or punch take is no robot move: it asks only about expired needs. Auto skips neither
     question. The tool model chip still refuses recording a tip pose or punch take (not the movers).
   - The run file starts on the first tick, take, sample, tip pose, punch take, solve or safety confirmation
     (`Run.save(start=True)`); other saves write only an existing file. Marker errors measured while driving are kept
     in the run until then. Run files drop `step`; older ones still load. The Run list says "in progress" instead of
     the step reached.
   - The phone's overview stays one row of number chips, styled as the laptop's rows; the current step's state shows
     in its section header on both pages.
C. **Review** against this note, then fixes. **Done**: the marker precondition also expires while driven too short;
   a stale target no longer skips expired preconditions; only failing results are accepted; "not in use" and the
   remount question no longer hold the operator; Apply follows the run's record, the relay is a live chip; Punch's
   stamp covers what it is judged under; measurements put out of date are kept. Renames: `*_holds`/`collisions_hold`
   are `*_expired`, `blocked` is `fails`, `show_open` is `show_work_left`; `worst()` is gone. Re-review: the
   "relay now" chip is neutral until the run applied something; a solution stays current while its takes move to
   archive/ and back (compared by file name), and Validate restores archived current spots before writing its
   outputs; the return target is worked out from where Done is pressed, not stored.
Later: Apply writes the validation result into the calibration file; the relay publishes it and body definitions; the
health panel shows "calibrated within" and the earlier-body warning.
