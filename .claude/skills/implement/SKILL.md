---
name: implement
description: Implement a plan written by /plan, on Opus 5 (1M context). Reads the plan file, makes the changes step by step following CLAUDE.md, runs the plan's verification, and records the outcome in the plan file.
argument-hint: [plan file, defaults to the newest in .claude/plans/]
model: opus[1m]
disable-model-invocation: true
---

# Implement (runs on Opus 5, 1M context)

You are the **implementer** in a two-model workflow: Fable 5.1 wrote the plan via `/plan`;
you build it. Follow-up messages after this turn stay on the session model (Opus 5, 1M in
this repo), so the whole implementation conversation runs here.

Plan file: $ARGUMENTS (if empty, use `ls -t .claude/plans/*.md | head -1`)

## Steps

1. **Read the whole plan.** Spot-check its Context against the current code; if the code has
   moved since the plan was written, adapt and note it in the Outcome.
2. **Only stop to ask if blocked.** Routine judgment calls: decide, state the assumption,
   keep going. A plan that is fundamentally wrong: stop and suggest `/plan revise <why>`
   rather than re-planning yourself.
3. **Execute the steps in order**, following the `CLAUDE.md` conventions already in context
   (minimal code, reuse, docstrings, plain comments with Better Comments markers).
4. **Run the plan's Verification** (the build/run recipe from `CLAUDE.md`). Report results
   faithfully: failing output verbatim, skipped steps named.
5. **Append an `## Outcome` section** to the plan file: date, what was done, deviations from
   the plan and why, anything left undone. Do not rewrite the rest of the plan.
6. **Reply** with a short summary: files changed, verification result, deviations.
