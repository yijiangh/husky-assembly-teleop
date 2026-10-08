# Schema 2 implementation status (2026-10-09)

Implemented in `husky-assembly-teleop` working tree on `jg/viser-cleanup` (uncommitted; HEAD e328f5b holds an older
draft). Package `bar_assembly_core`, design library `bar_assembly_core/design/`. In-repo spec copy:
`doc/design_format.md`; notes: `tasks/2026-10-08_shared_core.md` §12.

## Public API
- `bar_assembly_core.design`: `read(folder) -> Design`, `write(design, folder, *, overwrite=False, package_dirs=())`,
  `validate(design, *, check_robot_meshes=True)` (A checks), `content_hash(path)`; types `Design(folder, writer, robots,
  tools, bodies, schedule, actions, mates, producer)` with `.halves_of(bar)`, `.movements()`, `.scene_at(movement)`,
  `.scene_after(bar)`; `RobotSpec(..., ground_links)`, `BodySpec(id, pose, geometry, label, part, markers, mount)`,
  `RobotState`, `Holder(to, grasp)`, `ToolState(grip, on)`, `State(robots, present, attached, built, poses, tools)`,
  `LineSpec`, `Target(joints, links, tools, attached, built)`, `Movement` (`.grip_change`, `.has_tool_part`), `Action`;
  constants `GRIP`, `TOOL_KINDS`, `SCHEMA`.
- `design.relations`: `bar_of`, `is_present`, `mount_offset`, `placement(design, state, body) -> Pose | Holder`,
  `mate_status`, `mate_statuses`, `allowed_contacts(design, state)`, `OPEN/PENDING/ENGAGED/NOT_RELEVANT`.
- `design.plan_check`: `check_plan(design, *, geometry=True, seats=PART_SEATS) -> PlanReport(errors, warnings, ok)`,
  `end_state(movement)`, `PART_SEATS`, `GRASP_TOLERANCE`.
- `design.solutions`: `Solution`, `MovementResult`, `Planner`, `Trajectory`, `SolvedAgainst`, `solved_against`,
  `is_stale`, `write_solution`, `read_solutions`, `solution_warnings`.
- `bar_assembly_core.kinematics.link_pose(urdf_file, base, joints, link) -> Pose` (numpy, no yourdfpy).
- Scenes: `scene_at`, `scene_after`, hold helper, `retarget`; mirrors `CompasFabMirror` (with `lend()`),
  `PyBulletMirror`; `legacy/` converts old compas_fab exports to schema 2 (`scripts/convert_design.py`).

## Converted benchmark designs (schema 2)
`/tmp/claude-1000/-home-jakob-ra-workspace-design-core-src/ebd6696b-192b-4364-a2f6-676e208c1501/scratchpad/converted/`
`260814_RobArch_support_ik/` and `260920_RobArch_demo_revamp_backup/`: 48 actions, 182 movements, 0 A errors.
B errors: B11 Alice joint jumps between hold releases (export inconsistency), B14 B16 grasp off by 295 mm in 260920.
B warnings: 4 male halves without partner (fake bars B2, B6, B11, B14).

## Deviations / gaps found (by the implementer and by review)
- `on` changes are not written in `target`; `end_state()` keeps the start's `on`, so a movement alone does not say when
  a tool arrives/leaves; B11 does not compare `on`; B9 checks only that the path is linear, not that it moves away.
- Insert keeps schema 1's id (`B10_J_M4_tool_tighten_joint`) although it is now the insert.
- Two support "open" movements were dropped as no-ops (B12_H_M1, B15_H_M1).
- `writer.library` still says `design_io`. Bars stay meshes in converted designs (spec: primitives).
- Tool collision: compas_fab loads tools as convex hulls, so every carried bar collides with Cindy's tools in planning
  (real meshes are >= 1.72 mm apart). New pending-mate contacts at B19/B21 (bar and tools vs the female halves).
- Ground mate rule: a ground half mates the ground body nearest in plan view.
- `target.attached`/`target.built` hold the whole value at the end, written only when changed.
- Part seats: `T20/Male` identity, `T20/Ground` 180° about TCP z; `T20/Female`, `T20/MoCap` unknown (B14 warning).
