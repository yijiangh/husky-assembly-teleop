# Collision shapes: primitives first, convex by default (2026-10-20)

Status: **plan**, nothing built. Spans the core (`scaffolding_assembly_core`), the Rhino plugin
(`bar_joint_rhino_design_workflow`, branch `jg/rhino-new-core`) and the monitor.

## Why

Measured on `261008_2ModulesDemo_purged_process` (scripts in the session scratchpad, results in memory
`project_coal_vs_pybullet`):

- Every exported mesh counts as concave, even the tables: `TriMesh.from_arrays` tests `is_convex` on the raw arrays,
  and Rhino and compas write unwelded triangles, so no mesh is ever closed. The PyBullet mirror builds every free mesh
  body as a concave trimesh.
- The 20 letter obstacles (10 mm concave panels) make one `getClosestPoints` return up to 3,600 contacts, which holds
  the GIL for 45–89 ms. That is the planner and clearance stall in the monitor.
- Joint halves and tools are hand-modelled from separate boxes and convex pieces that `RSDefineJointHalf` and
  `RSDefineRoboticTool` merge into one OBJ. Splitting them again is exact.
- With exact pieces and the letters boxed, PyBullet's longest call drops from 36 to 1.8 ms; none of 2000 planner poses
  changes its answer.

## The rule

Each collision shape is, in order of preference:

1. an exact primitive (`box`, `cylinder`);
2. an exact convex piece;
3. the convex hull of a concave piece: the default, always with a warning;
4. a concave mesh, only where the designer asked for it (override below).

When collision is approximated (3), `visual` keeps the full mesh, so the views still show the real shape.

## Override

Per Rhino object, user text `collision_shape` (set in Rhino's Attribute User Text panel):

| Value | Meaning |
|---|---|
| absent or `auto` | 1 → 2 → 3, warn on 3 |
| `hull` | the convex hull of the whole object, no warning |
| `box` | its oriented bounding box, no warning |
| `concave` | keep concave pieces as meshes; a note, not a warning |

Joint halves and tools: the same key on their definition (`joint_pairs.json` half, `robotic_tools.json` tool),
asked for by `RSDefineJointHalf` / `RSDefineRoboticTool` when a picked piece is concave.

In the process file, a mesh kept concave on purpose carries `"concave": true` (shape key, §4.4). Without it, a
concave mesh is a warning (B16).

## Work

### Core (submodule; Python 3.9; its AGENTS.md: consumer review, both sides' tests, schema decision)

- C1. `TriMesh.from_arrays`: compute `convex` on a welded copy (by position, 0.1 mm), with a tolerance test: every
  triangle on the hull surface, open meshes included; flat pieces tested in 2D. The stored arrays stay as written.
  Old exports get honest flags with no re-export (tables, boxes, T20 Female, sheared plates become convex).
- C2. New module `collision_shapes.py`: `collision_pieces(mesh)` (exact split: boxes as `BoxShape`, convex pieces,
  concave pieces), and `collision_shapes(mesh, mode)` applying the rule and the override, returning the shapes and
  one note per hulled or kept-concave piece (triangle count). Sketch: scratchpad `bench/collision_pieces.py`.
  Cost on the 261008 process: 266 ms for its 50 mesh files, once per file.
- C3. Format: optional shape key `"concave": true`; read, write, validate (A2); `doc/process_format.md` §4.4.
  A 2.1 reader rejects the new key, so **schema 2.2** (D1).
- C4. Plan check B16 (warning): a concave collision mesh without `"concave": true`, naming body or tool, shape index,
  triangles and file. Shown by `RSExportProcess`, `RSCheckProcess` and the monitor. Doc §B.
- C5. PyBullet mirror: convex meshes already become hull shapes once C1 is honest. Build each body's shapes as one
  compound body (`createCollisionShapeArray`), so splitting adds no queries per tick; load URDFs with
  `URDF_USE_IMPLICIT_CYLINDER` (wheels are cylinders, not 64-point meshes).
- C6. compas mirror (Rhino IK) joins a tool's shapes into one hull (`mirrors/compas.py` 182–215): keep, or check per
  piece (D5).
- C7. Tests: the helper on boxes, the chamfered box, two touching boxes, spheres, an open mesh, a flat plane and a
  letter; the convex flag on the real unwelded OBJs; read/write of boxes and `concave`; B16.

### Rhino plugin (`jg/rhino-new-core`)

- R1. `_rhino_geometry` (`process_source.py` 404–435): meshed as now, then `collision_shapes` with the object's
  override. No reading of Rhino extrusions or profiles (user, 20 Oct: too fragile): exact boxes come from the mesh
  check, letters and the truss become hulls with a warning; a designer who needs better models it as separate
  convex pieces or boxes.
- R2. `_file_geometry` (345–354): `collision_shapes` on the OBJ, with the definition's override; cached per digest.
- R3. Write `visual` as the full mesh where collision was approximated.
- R4. Export notes once per source (a definition, not each of its 60 instances; obstacles by object and env id), and
  a warning at definition time in `RSDefineJointHalf` / `RSDefineRoboticTool`, per picked piece.
- R5. Pin the new core; tests (`tests/fake_rhino.py` needs extrusion stubs; `tests/equivalence/compas_compare.py`
  436–445 compares shape kinds and needs a volume-based comparison).
- R6. The user re-exports the 261008 process.

### Monitor

- M1. Bump the core pointer. Old exports: honest convex flags (C1) and B16 warnings; no hulling at read (D3).
- M2. Check on the re-exported process: the benchmark harness answers the same on its poses, and PyBullet's longest
  call stays under a few ms; then a live planning and clearance run without late ticks.

### Robots (separate, later)

The URDF meshes are convex hulls already and not slow. Primitives would be: top plate and iPad rack as boxes, the
bulkhead as two boxes (husky_urdf in crl-husky; never bump it on master). Alice's and Belle's URDF have the bumper box
commented out, so about 7 cm front and back at bumper height is not in their collision model (D6).

## Order

C1 → C2 → C3/C4 → C5 (core branch, consumer review, both test suites, push) → M1 (pointer) → R1–R5 on
`jg/rhino-new-core` with the new pin → R6 re-export → M2.

## Decisions needed

- D1. Record overrides in the file (`"concave": true`, schema 2.2), or keep the format and warn only at export?
- D2. Override key and values as above?
- D3. Old exports: warn only (proposed), or hull concave meshes at read until re-exported?
- D4. MoCap body: hull by default (proposed) or re-model it as convex pieces?
- D5. Rhino IK: keep one hull per tool, or check per piece (tighter, but changes the documented 1 mm overlap)?
- D6. Alice and Belle: enable the bumper box?
