# Three-Subagent Strategy: Planner / Implementer / Reviewer

A repeatable workflow for non-trivial multi-file changes in this repo. Designed
to keep the main conversation context small, get independent review, and run
work in parallel where safe.

## TL;DR

1. **Planner** turns a fuzzy goal into a concrete, self-contained task spec
   (read-only; thinks hard).
2. **Implementer** executes one task spec by writing code (edit + test).
3. **Reviewer** independently audits an Implementer's output against the spec
   (read-only; reports issues; never fixes).

Parent (the main Claude Code loop) orchestrates them: spawns Planner first,
then dispatches multiple Implementers **in parallel** for independent tasks,
and runs Reviewers afterward. **Subagents cannot talk to each other directly**
— all coordination flows through the parent (Claude Code's design).

## Why bother (and when not to)

**Use the chain when:**
- The change spans 3+ files or 200+ lines.
- The codebase is unfamiliar enough that exploration matters before coding.
- Independent review is valuable (e.g., a planning algorithm, a security touch,
  a refactor that affects callers across the repo).
- The task decomposes into 2+ pieces that can be done in parallel.

**Skip the chain for:**
- Typo fixes, single-line bug fixes, renames, simple refactors.
- Pure research / exploration ("how does X work?") — use the `Explore` agent
  directly.
- Tasks the parent can finish in 1–2 tool calls.

The overhead of subagent spawning (~5–15s per call, plus a non-cached prompt)
is wasted on small tasks.

## Roles in detail

### Planner (`.claude/agents/planner.md`)

**Input:** the user's goal + a pointer to the relevant code area.
**Output:** a Markdown spec covering each task with:
- File path(s) and line numbers to edit.
- A precise description of the change (function signatures, pseudo-code,
  branching logic, tests to add).
- Any pre-existing helpers/utilities to reuse, with file paths.
- Verification commands the Implementer should run after writing code.
- Dependencies between tasks (so the parent can parallelize what's independent
  and serialize what isn't).

**Tools:** read-only (Read, Grep, Glob, Bash for grep/ls/git, WebFetch).
**No Edit/Write/NotebookEdit.**

**Effort:** `xhigh` — planning quality dominates everything downstream.

### Implementer (`.claude/agents/implementer.md`)

**Input:** one self-contained task from the Planner's spec, plus the Planner
spec file as context.
**Output:** edited/written files, plus a short report of what changed and
the result of the verification commands the spec required.

**Tools:** Read, Edit, Write, Bash (for tests).
**Effort:** inherit (medium default is fine).

**Style hints in the body prompt:**
- Mirror existing code conventions in the file.
- Don't add dead code, defensive scaffolding, or comments narrating what
  the code already says.
- Run the spec's verification commands before declaring done.
- Report deviations from the spec with a one-line reason.

### Reviewer (`.claude/agents/reviewer.md`)

**Input:** path to the Planner's spec + paths the Implementer touched.
**Output:** PASS / FAIL / PARTIAL per spec item with file:line evidence,
followed by a top-3 list of the most important issues.

**Tools:** Read, Grep, Glob, Bash (read-only — `grep`, `ls`, `git diff`,
`git log`).
**No Edit/Write.**

**Effort:** `xhigh` — independent thinking is the whole point. Reviewer must
not just rubber-stamp.

## Communication & parallelism

### Parent dispatches subagents

The parent (main loop) is the orchestrator. To run two Implementers in
parallel, the parent emits a single message with two `Agent` tool calls
(one per task). Both run concurrently; results return when each finishes.

```
Parent: Agent(implementer, task=A) + Agent(implementer, task=B)  // single msg
        ↓                          ↓
        result A                   result B
        ↓
Parent: Agent(reviewer, audit=[A, B])  // after both done
```

### Iteration — re-engage with cache-warm context

When a Reviewer flags issues in an Implementer's work, **don't spawn a new
Implementer** — the new agent would re-read everything from scratch. Use
`SendMessage(to=<implementer_id>)` to continue the same agent, passing the
review notes. The Implementer's prompt cache stays warm; the fix lands
quickly and cheaply.

```
Reviewer: "issue 1: derive_constrained_start FK at seed_conf is wrong — should
           use grasp_bar_from_left to reconstruct goal-state tool0 instead."
Parent: SendMessage(to=implementer_A, "review found 3 issues — fix: ...")
        ↓
Parent: Agent(reviewer, "audit again")  // fresh-context reviewer
```

### Two-state mental model

For every subagent you spawn, ask:
- Is it **fresh-context** (Agent call) or **continuing** (SendMessage)?
- Is its work **independent** of others (parallel) or **sequential**?

Fresh-context reviewers are especially valuable — they spot what the
implementer rationalized away. Continuing implementers are valuable — they
remember the codebase they just navigated.

## Worked example: constrained planner integration (2026-05)

The integration spanned `api.py` (new), `husky_monitor.py` (state slots +
buttons), `husky_world.py` (refactored composite branch + new function),
`tasks/cc_lessons.md` (notes), and a headless test harness. Steps actually
taken:

1. **Plan mode** + 3 parallel `Explore` agents → plan file written.
2. **Implementer A**: write `api.py` + smoke import. *(spawned via Agent)*
3. **Reviewer A**: independent audit of `api.py`. *(spawned fresh)*
4. **Parent fixes**: 3 critical bugs Reviewer flagged (FK at wrong conf,
   stage-1 mis-classified as failure, empty-attachments crash) — applied
   directly via Edit since they were small and well-scoped.
5. **Implementer B**: edit `husky_monitor.py` + `husky_world.py`. *(parallel
   with Implementer A would have been possible if api.py weren't a hard
   dependency — in this case sequential was correct.)*
6. **Reviewer B**: independent audit of monitor/world wiring.
7. **Parent**: writes `tasks/cc_lessons.md` directly (small file).
8. **Parent**: end-to-end venv smoke tests (no agent — direct Bash).

Lessons:
- For Step 4, the parent could have used `SendMessage` to Implementer A
  instead of fixing directly. Both work; pick by edit complexity.
- Reviewers' "false alarms" (issue #1 of Reviewer B) are useful too — they
  surface implicit assumptions the Implementer should document.
- Parallelism gain is real when files are independent. For the constrained
  planner case, Implementer A and B had a hard ordering dependency (A's
  output is B's import surface), so they ran serially.

## Practical wiring (Claude Code specifics)

- Place agent specs at `.claude/agents/<name>.md` (project-level) or
  `~/.claude/agents/<name>.md` (user-level). Project-level overrides
  user-level.
- Frontmatter `tools:` is a **comma-separated list**, e.g.,
  `tools: Read, Grep, Glob, Bash`. Wildcards are not supported. Omit the
  field to inherit all parent tools.
- A subagent **cannot spawn other subagents** (per official docs). Don't put
  `Agent` in a subagent's tools list — it would be ignored.
- Frontmatter `model:` accepts `sonnet`, `opus`, `haiku`, full IDs, or
  `inherit` (default). The override env var is
  `CLAUDE_CODE_SUBAGENT_MODEL`.
- Invocation:
  - From the parent: `Agent({subagent_type: "planner", prompt: "…"})`.
  - From the user: `@planner` in chat to force-route a request.
- Use `permissionMode: plan` for the Planner (extra safety against
  accidentally writing files); `default` for Implementer; `default` for
  Reviewer.

## When to override per-call

The frontmatter sets defaults. The parent can override `effort`, `model`,
or pass extra prompt content per call. Useful overrides:
- Reviewer at `effort: xhigh` for security-sensitive code.
- Implementer at `model: haiku` for pure mechanical edits (renames,
  formatting).
- Planner at `effort: max` for architecturally hairy designs.

## Open questions / future work

- **Long-running parallel implementers**: when Implementer A is mid-work
  and the parent realizes Implementer B should also start, is it cheaper
  to (a) wait for A, then spawn B, or (b) spawn B now in a separate
  message? Currently (b) requires no waiting — Bash + Agent can run
  concurrently — so prefer (b) when work is independent.
- **Auto-iteration cap**: define a parent-side rule like "if Reviewer
  flags ≥3 critical issues, spawn a *fresh* Implementer instead of
  SendMessage to the original — fresh context may break a stuck mental
  model."
- **Memory writes**: which subagent owns memory? Currently parent writes
  memory; Implementer or Planner could too if given Write. Probably best
  to keep memory writes parent-only so changes are visible in the main
  conversation.
