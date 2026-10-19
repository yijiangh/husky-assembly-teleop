---
name: commit
description: Commit the working tree in this repo's style. Groups the changes into as few commits as make sense — one per change the user would describe in one sentence, each with its tests, smoke and docs — writes "[tag] scope: what changed" subjects with a plain-prose body, keeps run-mode flag flips out, and never pushes.
argument-hint: [optional: what to commit, how to split, or "include flags"]
disable-model-invocation: true
---

# Commit

Commit what is in the working tree, the way this repo's history does it. User notes for this
run (scope, split, "include flags"): $ARGUMENTS

## 1. Look before staging

Run these and read them all; never stage from memory:

```bash
git status --short
git diff --stat
git log -15 --format='%h %s'       # the style to match
git branch --show-current
```

- On `master` / `main`: create a branch first (`git switch -c yh/<topic>`) and say so.
- Read the full diff of every modified file (`git diff <file>`) and skim every untracked file
  you might add. Know what each hunk is before it goes anywhere.
- If a change is not yours to judge (not from this session and not explained by the diff
  itself), leave it unstaged and list it in the report rather than guess.

## 2. Group: as few commits as make sense, no fewer

One commit = one change the user would describe in one sentence. It carries everything that
change needs:

- the code, **its** tests, **its** smoke / headless script, **its** manual or doc updates,
  and its `tasks/` note all go in the **same** commit (see `a33f159`: monitor + robot +
  smoke + two manuals);
- a fix found while building a feature, in the feature's own new code, is part of the
  feature, not a separate `[fix]`.

Split only along real seams:

- two independent features or fixes (each would make sense reverted alone);
- a doc-only change to a manual unrelated to the code change (`[doc]`);
- a settings / calibration / submodule bump (`[chore]`);
- a pure rename or move with no behaviour change (`[refactor]`), when it is large enough to
  hide the real change.

Do **not** split a feature into "model", "UI", "tests" and "docs" commits, and do **not**
lump unrelated changes together because they were made in the same session. A session
typically ends in 1–3 commits. If the user's arguments say how to split, follow them.

## 3. Keep local run-mode settings out

These change per run or per PC and are usually left over from a dry run or desk test:

- class flags in `husky_monitor.py` such as `FAKE_HARDWARE`, `USE_MOCAP`,
  `MOCAP_BAR_FAKE_MARKERS`, `CALIBRATION`, `PUNCH_CALIB_VALIDATION`,
  `BAR_ACTION_MOCAP_ACCURACY_TEST`, `USE_CELL_STATE_BASE_POSE`, `CONNECT_COMPLIANT_CONTROLLER`;
- module values such as `CLIENT_IP`, and `DESIGN_PROBLEM_NAME` / calibration-date defaults in
  `__init__.py`.

By default a flip of these is **not** committed:

1. Note the line's working-tree value.
2. Set it back to its committed value: `git show HEAD:<file>` tells you which.
3. `git add` the file.
4. Restore the user's value in the working tree.

Then confirm `git diff --cached` has no flag line and `git diff` still has it. The repo did
this before with a `sed` swap; never `git stash` or `git checkout` the user's tree.

Commit flags only when the arguments say so ("include flags") or the session's task was a
settings change. Then they go in their own `[chore] monitor settings for <run>: ...` commit
(see `56babc5`, `f052b29`).

Never commit:
- `.claude/plans/`;
- scratch, temp or test-output files;
- recorded data (`recorded_data/`, take JSONs);
- anything that looks like credentials;
- large binaries that are not clearly meant for the repo.

## 4. Message: match the history

```
[tag] scope: what changed, in plain words

What changes for the person running the monitor / the robot, and why, in a few
wrapped sentences (about 80 columns). Name the button / log line / file the user
sees, in quotes. Key numbers that justify it. Then what verified it and what did
not: "Covered by unit tests and the headless smoke (N checks); not yet run on
hardware."

<attribution lines>
```

- **Tags:**
  - `feat`: new behaviour.
  - `fix`: wrong behaviour corrected.
  - `doc`: docs only.
  - `chore`: settings, calibration, submodule bumps, housekeeping.
  - `refactor`: same behaviour, new shape.
  - `data`: datasets.
- **Subject:** lower-case after the tag, no period. Plain words, no code jargon. A short
  scope prefix the user would recognise ("plan skeleton:", "insert:", "three-robot manual:",
  "MoCap bar:"), then the change itself, not the files touched. Long is fine when it carries
  meaning: the history runs to ~110 characters.
- **Body:** prose, not a bullet dump; bullets only for genuinely list-shaped content. Say what
  was NOT verified (hardware, GUI, a robot) as plainly as what was.
- **Attribution:** end with exactly the lines the harness gives for commits in this session
  (the system reminder, e.g. `Co-Authored-By: ...` and `Claude-Session: ...`), with one blank
  line before them. Do not invent or copy older trailers.
- Pass the message with a heredoc so the wrapping survives:
  `git commit -F - <<'EOF'` ... `EOF`.

## 5. Stage, commit, check

- Stage explicit paths (`git add <file> ...`), never `git add -A` / `git add .`.
- Interactive staging (`git add -p`, `-i`) does not work here. To commit part of a file, use
  the swap of step 3, or ask.
- Before each commit, read `git diff --cached --stat` and make sure it is exactly that
  commit's group.
- Never `--no-verify`, never `--amend` a commit you did not just make in this run, never push
  (unless the arguments say to).
- If a hook fails, fix the cause and make a new commit attempt; report what the hook said.

## 6. Report

A short table: commit hash → subject → files. Then:
- what was left uncommitted and why (flags kept local, files not yours to judge);
- the branch.

Offer to push only if a remote branch exists and the user has pushed before in this repo.
