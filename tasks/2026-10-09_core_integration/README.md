# Core integration: state on 2026-10-09

Schema 2 of the design format is implemented in `bar_assembly_core` and matches the spec, apart from the gaps below.
Two reviews (Rhino plugin, tamp) found that schema 2 fits both consumers. One blocker is shared: the allowed tool
contacts. Eleven decisions were open; all were taken the same day (last section) and are implemented in
`jeliag/bar_assembly_core`, branch `jg/decisions-2026-10-09`.

## Files in this folder

| File | What it is |
| --- | --- |
| `README.md` | This synthesis: the spec check of the implementation, the shared findings, the open decisions |
| `schema2_implementation_status.md` | The implementation's public API, converted designs, deviations and gaps |
| `rhino_integration_review.md` | Rhino plugin review: field mapping, pain points, change requests, integration proposal (6–8 weeks) |
| `tamp_integration_review.md` | Tamp review: what ran, mappings, pain points, change requests, integration proposal (2–3.5 weeks) |
| `prototypes/rhino/` | `proto_builder.py` (state-machine builder over the schedule), Python 3.9 checks, keyframe collision probe |
| `prototypes/tamp/` | `try_tamp.py` (design → scene → mirror → `lend()` → tamp) and collision diagnostics, with logs |

Specs (Claude Docs): schema 2 proposal <https://claude.ai/code/artifact/581409c8-ffe6-41f1-a008-8643277c85ae>,
shared core report <https://claude.ai/code/artifact/df1d4fec-eeeb-424b-b3c2-ca422117d42f>. In-repo copy of the format:
`doc/design_format.md`. Implementation notes: `tasks/2026-10-08_shared_core.md` §12.

## State of the code

- Branch `jg/viser-cleanup`, working tree on top of e328f5b, **uncommitted**. It includes another session's change that
  removes yourdfpy from FK (`bar_assembly_core/kinematics.py` is now pure numpy).
- Tests: quick suite 223 passed, 4 skipped; `-m slow` 13 passed; Python 3.9 import test passed.
- Both benchmark exports convert to schema 2 with 0 file (A) errors. The plan checker finds only problems in the
  exports themselves:
  - B11: Alice's joints jump between hold releases (a Rhino export artefact; the Rhino reviewer's prototype builder
    removes it);
  - B14: B16's grasp in 260920 is 295 mm off its design pose;
  - B5 warnings: 4 male halves whose partners sit on the fake bars B2, B6, B11 and B14.
- Action files: 0.70 MB on 260814, against 1.11 MB in schema 1.

## Spec check of the implementation

Checked by reading the converted 260814 design and rerunning the plan checker.

**Matches the spec:**

- mounts and parts on halves; 36 mates; `ground_links`; `mount_contacts`; no `touches` or `connections`;
- the insert as one compliant coupled move with `drives: tighten` and `ends_on: tools`, the bar built at its end;
- no untighten in releases; the ungrasp in form B;
- `attached` with one holder per flange; Alice holding the built B3 together with Cindy;
- `on` and `grip` in tool states; the `null` target on the travel to the loading pose;
- a deterministic content hash.

**Gaps:**

1. `on` changes are not written in targets. `end_state()` keeps the start's `on`, B11 does not compare it, and B9
   checks only that a retreat is linear, not that it moves away.
2. The insert keeps schema 1's id (`B10_J_M4_tool_tighten_joint`); tamp's role regex then reads it as home.
3. Two support "open" movements were dropped as no-ops (`B12_H_M1`, `B15_H_M1`). That changes the procedure: on the
   real robot, a repeated open may be deliberate.
4. `writer.library` still says `design_io`. Converted bars stay meshes; the spec says primitives.
5. Ambiguities the implementer resolved:
   - ground halves mate the ground body nearest in plan view;
   - `target.attached` and `target.built` hold the whole value;
   - stricter A11 and A12;
   - part seats hard-coded: `T20/Male` identity, `T20/Ground` 180° about the TCP z axis.

## Shared findings of both reviews

- **Blocker, tool contacts.** compas_fab loads each tool as one convex hull, which overlaps the carried bar by 1.3 mm,
  although the real meshes are 1.0 mm apart. A convex decomposition (VHACD, 4 parts) still overlaps by 2.0 mm.
  - With the derived contacts (a tool may touch only the half it is on), Cindy's tools collide with the carried bar in
    41 of 80 keyframe states, and tamp's coupled transfer fails with `goal_in_collision`.
  - Male halves also touch their partner's bar at the assembled pose (14 of 80), and a ground half can touch two
    walkable surfaces.
  - Both reviewers propose the same fix, which is what Rhino allows today.
- **`null` has two meanings.** "A planner fills it" (tamp derives the transfer start offline) and "measured at
  execution". Unknown robots currently stand at the origin with zero joints in the mirror.
- **`on` in targets:** both reviewers need to know which movement sets or clears it.
- **Movement ids:** both Rhino and tamp find movements by id pattern; role-named ids would serve both.
- **Packaging:** both want the core as its own installable repository, with a 3.9 CI job and a `[mirrors]` extra.

## Rhino review in brief

- **Fit:** every schema 2 field has a source in Rhino, except three constants: serials, wheel ground links, tool kinds.
- **Prototype:** a builder threading one state through the schedule rebuilt 260814 with 0 A and 0 B errors.
- **Python 3.9:** the core and the compas_fab mirror ran on 3.9 with Rhino's exact pins (Linux only).
- **Blockers:**
  - tool contacts;
  - the hold helper builds the release-check scene, not the scene the hold is solved in.
- **Major:**
  - the builders must become one state machine;
  - ground halves on two surfaces;
  - unknown robots at the origin;
  - `on` not in targets;
  - adopting a solution makes it stale;
  - hash gaps: dict order, mesh and URDF content not covered, no in-memory hash;
  - Rhino's per-command `importlib.reload` breaks caches of core objects.
- **Proposal:** extract the core and pin it as a submodule; `RhinoDesignSource` rebuilds the `Design` on command
  entry with a geometry cache; a Rhino-free `design_builder` state machine replaces `_build_m*`; keyframe and support
  IK on mirrors; the fingerprint and `RSRebuildRobotCell` go away. Effort about 6–8 weeks.

## Tamp review in brief

- **Fit:** a converted design reaches tamp through `read` → `scene_at` → mirror → `lend()` with no change to tamp, on
  517d692, 2fce15f and holding-prm.
- **Ran:** insert, retreat, home (needs more iterations), coupled transfer (needs the contact fix) and keyframe IK.
  Solutions were written and read back.
- **Blocker:** tool contacts.
- **Major:**
  - `null` starts that tamp derives offline;
  - only dual-arm Cindy is planned; 24 support movements have no planner;
  - the chaining tolerance of 1e-6 is too strict for tamp's IK;
  - movements with no arm motion have no solution status;
  - keyframe ownership between design and `solutions/`;
  - the core cannot be installed on its own;
  - ssik is the default backend but missing in the teleop venv.
- **Proposal:** tamp stays independent; an optional `husky_assembly_tamp.design` adapter (`plan_movement`,
  `plan_action`, `solve_keyframes`, a CLI replacing the headless planner's I/O) depends on `bar_assembly_core[mirrors]`.
  Effort about 2–2.5 weeks, plus about a week for PRM and VAMP.

## Decisions (open on the morning of 2026-10-09)

| # | Decision | Raised by |
| --- | --- | --- |
| 1 | A tool may touch its whole part (bar plus halves); a mate also lets each half touch the partner's bar; ground halves may touch any ground body | both |
| 2 | `on` may be written in `target.tools`, so a movement shows when a tool arrives or leaves | both, spec check |
| 3 | Split `null` into "a planner fills it" and "measured at execution"; the core seeds nulls for offline planning; mirrors park robots with unknown state instead of placing them at the origin | both |
| 4 | Two hold scenes: one to solve the hold (Cindy at the assembled pose), one for the release check | Rhino |
| 5 | Untighten before every ungrasp in Rhino's builder: remove it, or keep it as a relax step (`loosen` with `built` unchanged) | Rhino (hardware) |
| 6 | Hashes: deterministic order, mesh and URDF content included, an in-memory hash, and `adopt()` so a solution written back into Rhino stays valid | Rhino |
| 7 | Chaining tolerance 1e-3 rad instead of 1e-6; movements with no arm motion are `solved`; the design's `target.joints` win over planner keyframes | tamp |
| 8 | Role-named movement ids (`B10_J_insert`) | both |
| 9 | Part seats move into a `parts` catalogue in `design.json`, written from Rhino's catalogue | Rhino |
| 10 | The core becomes its own installable repository (3.9 CI, `[mirrors]` extra), a submodule in Rhino and a dependency of the tamp adapter | both |
| 11 | Who plans single-arm support moves: tamp, or Rhino's support IK | tamp |

Also from the implementer (some overlap with the table): the release form per bar (A or B), whether the converter
should repair inconsistent export states, and the seats for `T20/Female` and `T20/MoCap`.

### Taken on 2026-10-09

| # | Outcome |
| --- | --- |
| 1 | Yes. `relations.allowed_contacts`: a tool touches its whole part; pending or engaged mates' halves also each other's bar; a ground-mated half every ground body. |
| 2 | Yes. `target.tools.<tool>` is `{grip?, on?}`; `Target.on`; B11 compares `on`. |
| 3 | No new marker: `null` = not decided by the design; planners may fill it (into `solutions/`), the monitor at execution. The compas_fab mirror parks other robots with unknown state (`mirror.parked`); `scenes.seeded` fills the acting robot. |
| 4 | No special scenes: `hold.hold_scenes` = `scene_at` of the closing movement (solve) and of the hold release (check). |
| 5 | R_M0 folds into the ungrasp: `drives: loosen` with the grip opening, `built` unchanged. |
| 6 | Ids sorted, meshes `meshes/<content hash>.obj`, robot `files_hash` (A4 on read), `design_hashes` in memory. `adopt()` deferred. |
| 7 | Yes: `CHAIN_TOLERANCE = 1e-3`, `MovementResult.still`, solved ends must reach the design's target joints. |
| 8 | Yes: `<bar>_<J/R/H/HR>` and `<action>_<role>`; the converter renames. |
| 9 | Yes: optional `parts` in `design.json`; B14 uses it before `PART_SEATS`. |
| 10 | Done: `jeliag/bar_assembly_core`, a submodule in Rhino; CI on 3.9 and 3.10. Teleop switches to it with the monitor integration. |
| 11 | Rhino's support IK for now; a tamp single-arm planner later. |

Implementer's questions: no release-form flag (the movements show it); the converter converts faithfully (repeated
opens kept, no repair); seats for `T20/Female` and `T20/MoCap` come from Rhino's `parts` when a tool acts on them.
`writer.library` is now `bar_assembly_core`. Bars stay meshes in converted designs; Rhino writes cylinders.

## Housekeeping

The reviewers left git worktrees in a temporary scratchpad: four for tamp (`tamp_wt_main`, `tamp_wt_prm`,
`tamp_wt_ssik`, `tamp_wt_vamp`, registered in the tamp repo's `.git/worktrees`) and one for Rhino (`rhino_wt`). Remove
them with `git worktree remove <path>` (or `git worktree prune` once the scratchpad is gone).
