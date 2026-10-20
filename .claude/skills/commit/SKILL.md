---
name: commit
description: Commit the pending changes in this repo. Groups them into a few commits by topic (feature, fix, docs, chore), writes messages in the repo's "[tag] plain words" style, and reports what went into each commit and what was left out. Never pushes, amends, or rewrites history.
argument-hint: [optional: what to include or how to group, e.g. "only the teleop work"]
disable-model-invocation: true
---

# Commit

Commit the working tree's pending changes. Scope / hints from the user: $ARGUMENTS
(empty = everything pending that belongs in the repo).

## Steps

1. **Look first.** `git status`, `git diff`, `git diff --cached`, `git log --oneline -20`
   (for the message style), and the current branch. On `master`, stop and ask whether to
   branch first.
2. **Group by topic.** One commit per thing a reader would describe in one sentence: a
   feature, a fix, a doc update, a settings / submodule bump. A feature's code, its test
   and its doc go in the SAME commit.
3. **Bundle, but don't overdo it.**
   - Usually 1-4 commits. Don't make one commit per file, and don't split a feature into
     "add helper" / "use helper" commits.
   - Don't mix unrelated topics in one commit just to keep the count down either.
   - Split by FILE. Only split a file's hunks across commits when the topics sit in
     clearly separate hunks and it is easy (`git diff <file>` → trim → `git apply --cached`);
     otherwise put the file in the commit its main change belongs to and say so in the body.
     Never use interactive `-i` / `-p` flags (not supported here).
4. **Leave out** what doesn't belong: scratch / run outputs, `__pycache__`, large data,
   anything that looks like a secret, and changes that look accidental (a stray debug
   print, an unexplained deleted file). Ask about a file only when you can't tell; otherwise
   leave it out and list it in the report.
5. **Write the message** in the repo's style:
   - Subject: `[feat]`, `[fix]`, `[doc]`, `[chore]`, `[test]` or `[refactor]`, then
     plain words about what the user / operator gets, e.g.
     `[feat] Teleop panel: hold-to-jog the arm in its tool0 frame`.
   - Body only when it adds something: why, and caveats a reviewer must know (e.g.
     "not yet run on hardware"). A few lines, no essays, no file-by-file list.
   - End with the attribution line(s) from the session's system reminder.
   - Pass the message with a heredoc so the formatting survives.
6. **Commit** with `git add <files>` + `git commit`. If a hook fails, fix the cause and
   commit again as a NEW commit attempt: never `--amend`, `--no-verify`, or force.
7. **Report:** one line per commit (short hash, subject, files), then what was left
   uncommitted and why.

## Never

- `git push` (unless the user asks in this same message), `--amend`, rebase, reset,
  `--force`, or changing git config.
- Committing again later in the session without being asked: this skill covers this one
  invocation only.
