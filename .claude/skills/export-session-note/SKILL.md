---
name: export-session-note
description: Write a handover note capturing this session's working context — what was built, what state it is in, the hard-won facts and dead ends, and what is not yet verified — so another Claude session can pick the work up cold. Saves to doc/<topic>_handover.md.
argument-hint: [topic] | update [path to existing note]
disable-model-invocation: true
---

# Export session note

Write the context of THIS session to a handover note, for a fresh session that has none of
it. Target: `doc/<topic>_handover.md` (create `doc/` only if the repo has no equivalent).
With `update <path>`, revise that file in place instead — keep what still holds, correct
what changed, and delete what is now wrong.

Task/topic: $ARGUMENTS

## Rules

- **Write for someone with zero memory of this conversation.** No "as discussed", no
  codenames invented here without defining them, no references to what "we tried earlier".
- **Facts over narrative.** Every claim that cost effort to learn should carry the evidence
  that proves it: a measured number, a file:line, an error string, a command. "The planner
  is slow in the GUI client" is worthless; "a 4.6 s headless search ran past 15 minutes in
  the GUI client because check_collision re-pushes the cell state per sample" is the note.
- **Record the dead ends and why they failed.** This is the highest-value part of a
  handover — it is the part the next session cannot rediscover cheaply, and the part it
  will otherwise repeat. Include the symptom that identifies each one.
- **Never inflate verification.** State plainly what was tested, how, and what was NOT.
  If something only ran against a simulated/fake backend, say so. If hardware never ran it,
  say so.
- **Flag anything ephemeral** — scratchpad test scripts, background processes, temporary
  network or robot state that will not survive.
- No secrets, no credentials, no pasted customer data.

## Steps

1. **Check the real state, do not recall it.** Run `git log --oneline -5`, `git status
   --short`, and look for sibling notes (`doc/*handover*`, `doc/*manual*`) to match the
   repo's convention and to avoid duplicating or contradicting them. If another note owns
   part of this story, link it rather than restating it.
2. **Draft the note** with these sections, dropping any that would be empty:
   - Title, the date it was written, one line on who it is for, links to sibling docs.
   - **What the task is** — the goal and why, in a few sentences.
   - **State** — branch, what is committed (table of commit → what), what is uncommitted
     and why, and any question that was asked but never answered.
   - **The pieces** — a file → responsibility table, plus the exact command(s) to run it
     and the UI/CLI flow.
   - **What will bite you** — the numbered gotchas, each with its evidence.
   - **Verification status** — tested how, and explicitly what is untested.
   - **Next steps** — concrete, ordered.
3. **Write it**, then report the path and a two-line summary of what it covers.
4. If a fact in the note is durable and repo-independent (a library's behaviour, an ops
   procedure), also check whether it belongs in memory — but do not duplicate the whole
   note there.

## Do not

- Summarize the conversation turn by turn. The note is about the WORK, not the session.
- Delegate to a subagent: the context being exported lives in this conversation only.
- Commit the note unless the user asks.
