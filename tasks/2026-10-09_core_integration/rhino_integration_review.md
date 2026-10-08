# Rhino integration review: shared core and design format schema 2

Reviewed on 2026-10-09:

- Rhino plugin at `origin/hs/mocap-experiment` @ 069275f (`R:` below means `scripts/` of that commit).
- Core at `husky-assembly-teleop` `jg/viser-cleanup`, working tree on top of e328f5b (`C:` below means `bar_assembly_core/`).
- Tamp at 517d692.
- Converted designs 260814 and 260920.

Line numbers refer to those commits.

Prototype code is in `prototypes/rhino/` next to this file (the logs and Python 3.9 environments stayed in a temporary scratchpad):

- `py39_check.py`: the pure core on 3.9.
- `py39_mirror_check.py`: the mirror on 3.9.
- `py39_keyframe_collisions.py`: derived contacts at Rhino's keyframes.
- `proto_builder.py`: a Rhino-style state-machine builder.

## 1. Verdict

Schema 2 fits Rhino well:

- Every field has a source in the document, in `joint_pairs.json`, in `robotic_tools.json` or in `config.py`, except three constants: robot serials, ground links and tool kinds.
- The procedure (merged insert, form-B release, holds with two holders on a built bar) maps onto Rhino's keyframes with no new user input.
- A prototype builder that threads one state through the schedule turns Rhino's per-bar keyframe records into a design with 0 A errors and 0 B errors on 260814. It also removes the B11 errors of today's export.
- The pure core and the compas_fab mirror both import and run on CPython 3.9 with Rhino's exact package pins.

Two things block Rhino from planning on the core today:

- **Derived contacts make keyframe IK fail.** With compas_fab's convex hulls, the derived allowed contacts report Cindy's tool against the carried bar in 41 of 80 keyframe states (19 of 20 bars). They also report the male half against its partner's bar in 14 of 80.
- **The hold helper builds the wrong scene.** It gives Rhino's release-check scene, not the scene the hold keyframe is solved in.

Beyond these, the cost is the builder rewrite, about 1.5 weeks, plus some deterministic-export and hashing details. The whole integration is about 6–8 weeks.

## 2. Field mapping: schema 2 → Rhino source

Status values:

- **exists**: read directly, or with a unit or name conversion.
- **adapter**: the data exists, but `RhinoDesignSource` or the builder must derive it.
- **missing**: Rhino has no source.

| Schema 2 field | Rhino source | Status | Evidence |
| --- | --- | --- | --- |
| `writer` | Core library | exists (core) | C:`design/write.py:59` |
| `producer {repo, commit, dirty, command}` | Plugin git state plus the command name | adapter (git read; `dirty` needs `git status`, which may not be on PATH in Rhino) | none today |
| `source/` (.3dm, `joint_pairs.json`, `robotic_tools.json`) | The document (unsaved edits need `RhinoDoc.WriteFile` to a copy; not verified). Both JSONs live in the repo, not the doc | adapter. `write()` leaves `source/` alone | R:`core/joint_pairs.json`, R:`core/robotic_tools.json`; C:`design/write.py:31-33` |
| **Robots** `robots/cindy|alice|belle` | `ASSEMBLY_ROBOT_NAME`, `SUPPORT_ROBOT_NAMES`, `ROBOT_IDS` (`dual-arm_husky_Cindy`, etc.) | adapter (lowercase name) | R:`core/config.py:274-294` |
| `urdf`, `srdf` | `HUSKY_URDF_*`, `SUPPORT_ROBOTS[...]` in `asset/husky_urdf` (submodule aa6a261, the non-Stock calibrated files) | exists. The converted Cindy URDF has the same joint origins | R:`core/config.py:133-135, 279-290` |
| `serial` | none (the converter took `SERIALS` from teleop) | missing (optional) | C:`legacy/conversion.py:39` |
| `tools` (flange → tool) | `ASSEMBLY_TOOL0_LINKS`, the active pair in `robotic_tools.json`, `SUPPORT_TOOL0_LINK` | exists | R:`core/config.py:327-331`; `robotic_tools.json["active"]` |
| `ground_links` | none | missing (constant: the four `*_wheel_link`) | none |
| **Tools** id `tools/AT3L` | Active pair names | exists | `robotic_tools.json` |
| Tool `geometry` | `asset/<collision_filename>.obj` (mm); `ROBOTIQ_GRIPPER_TOOL_MESH` | exists (scale to m) | R:`core/config.py:303` |
| Tool `tcp` | `M_tcp_from_block` (`world_tcp = world_block @ M`, block = flange). Robotiq: inverse of `BAR_GRASP_TO_TOOL0["Robotiq"]` | exists for Cindy's tools (mm→m; matches the converted value [-0.07, 0, 0.08]); adapter for Robotiq (the converter wrote identity) | R:`core/robotic_tool.py:111`, R:`core/config_generated_ik.py:19` |
| Tool `kind` | none; implied by role | missing (constant `scaffolding_v3` / `robotiq`) | none |
| Tool `mount_contacts` | `_ARM_TOOL_WRIST_TOUCH_LINKS` (wrist_2, wrist_3), `SUPPORT_TOOL_TOUCH_LINKS` (wrist_3) | exists | R:`core/robot_cell.py:514`, R:`core/config.py:299` |
| **Bars** id `bars/B7` | `bar_id` user text on the centerline | exists | R:`core/rhino_bar_registry.py:49` |
| Bar `pose` | Midpoint frame of the centerline (z along the bar) | exists | R:`docs/coordinate_conventions.md` §2; R:`core/env_collision.py:172` |
| Bar `geometry` (primitive) | `BAR_RADIUS` or `tube_radius`, plus the curve length → `CylinderShape(r, L)` | exists. Rhino meshes 12 sides; the core 32 | R:`core/config.py:82`, R:`core/env_collision.py:134` |
| Fake bars | `scaffolding.fake_bar`. Must be dropped together with their halves | adapter | R:`core/rhino_bar_registry.py:70, 564-590` |
| **Halves** id `joints/<jid>_<role>` | `joint_id` user text plus the layer's subtype (`J40-53_mocap`, `G4-T20-0_ground`, `M7-T20-0_mocap`) | exists | R:`core/joint_name_conventions.py:315, 391` |
| Half `pose` | Block instance transform | exists (check for scale or reflection) | R:`core/rhino_helpers.py` |
| Half `geometry` | `collision_filename` OBJ of the block (one `Geometry` per block definition) | exists | R:`core/env_collision.py:198` |
| Half `part` | Block name `<Type>_<Subtype>` → `<Type>/<Subtype>`. Seen: `T20/Male\|Female\|Ground\|MoCap`, `T20Deck12/Male`, `T20SubLeft/Female`, `T20SubRight/Female` | exists (rename) | R:`core/joint_name_conventions.py:144`; `joint_pairs.json` |
| Half `mount` | `parent_bar_id` | exists | R:`core/joint_name_conventions.py:399` |
| Half `markers` | `marker_points_mm` in `joint_pairs.json` (block frame, mm; `T20_MoCap` only) | exists (scale) | R:`core/marker_points.py:5`; `joint_pairs.json` |
| **Ground** `ground/WG0` | WalkableGround breps, meshed coarse | exists | R:`core/rhino_walkable_ground.py:127, 147` |
| **Obstacles** `obstacles/<name>` | `LAYER_ENVIRONMENT` meshes and breps | exists | R:`core/env_collision.py:640` |
| `mates` male–female | Paired joint ids `J<receiver>-<male>` with `_female`/`_mocap` and `_male`, plus `joint_pair_name`. These are instance mates; the `mates` in `joint_pairs.json` are only the catalogue | adapter | R:`core/joint_name_conventions.py:236, 397` |
| `mates` ground half ↔ ground | The bar's `walkable_ground_ids`, plus the surface under the foot | adapter. A foot can touch two surfaces (see §3 P4) | R:`core/rhino_walkable_ground.py:268, 432` |
| `schedule` | `build_action_schedule` (J, H, R, HR), Rhino-free | exists | R:`core/hold_schedule.py:144` |
| **Action** `id`, `type`, `bar`, `label` | `SCHEDULE_KINDS`, `active_bar_id`, `tag` | exists | R:`core/hold_action_builder.py:995-1000` |
| Action `robot` | `config.ROBOT_IDS` → `robots/...` | adapter | none |
| Action `ground` | `walkable_ground_ids` → `ground/WGn` | exists | R:`core/rhino_walkable_ground.py:268` |
| Action `supports_until` | `supported_until` user text | exists | R:`core/rhino_bar_registry.py:547` |
| **Movement** `arms` | Movement class plus `ASSEMBLY_TOOL0_LINKS` / `SUPPORT_TOOL0_LINK` | adapter | R:`core/config.py:327-331` |
| `path`, `coupled` | Movement class (`...LinearMovement`, `EndEffectorConstrained...`) | exists | R:`core/bar_action.py:96-108` |
| `controller` | `CONTROLLER_CARTESIAN_COMPLIANT` on the insert; `position` otherwise | exists | R:`core/bar_action.py:1015` |
| `line` | Today only in notes: `approach_axis` / `lm_distance_mm`, `retreat_axes_world`, `SUPPORT_LM_DISTANCE_MM` | adapter (computed today, but stored as notes) | R:`core/bar_action.py:168-198, 1111, 1127`; R:`core/config.py:146, 150, 348` |
| `drives` | J_M4 `tool_action="tighten"`, `overlaps_next` | adapter: the **merged insert** is new builder logic | R:`core/bar_action.py:1529-1534` |
| `ends_on` | Notes `"ends_on": "tool_stall_signal"`; manual and tool movements by class | adapter | R:`core/bar_action.py:1027` |
| `target.links` | `target_ee_frames` `{"left", "right", "arm"}` (Frames, m) → link ids | exists (rename keys) | R:`core/bar_action.py:1325+` |
| `target.joints` | approach / assembled / retreat / home; support approach / held | exists | R:`core/config.py:155, 366-385` |
| `target.tools` (grip) | `tool_action` grasp / ungrasp / open / close | exists | R:`core/bar_action.py:1523, 1553`; R:`core/hold_action_builder.py:876, 893, 954` |
| `target.attached`, `target.built` | none | adapter (builder logic) | none |
| `target: null` | J_M0 goal "backfilled" | exists | R:`core/bar_action.py:1186-1258` |
| **State** `robots.base` / `joints` | `KEY_ASSEMBLY_BASE_FRAME` (mm 4x4), per-arm groups; `KEY_SUPPORT_*`. Parked → `null`; no IK → `null` (today an identity placeholder) | adapter (mm→m, two groups into one 12-joint map) | R:`core/config.py:338, 366-385`; R:`core/bar_action.py:1627` |
| `present` | Today `is_hidden` flags in a template state | adapter (from the sequence) | R:`core/bar_action.py:1338-1346` |
| `attached` (holders with grasps) | Tool0 at the assembled pose = placed tool block transforms; `grasp = inv(tool0) · bar`. Today the bar is on the left flange only, and the halves are attached too | adapter: two holders, bar only | R:`core/bar_action.py:139-143, 567-637` |
| `attached`: Alice on a built bar | Support grasp frame plus `BAR_GRASP_TO_TOOL0` → tool0; `grasp = inv(tool0) · bar` | adapter (new) | R:`core/hold_action_builder.py:770-800` |
| `built` | Sequence order (earlier bars, plus the current bar after its insert) | adapter (new) | none |
| `poses` (staged bars) | none; Rhino never stages a bar | not needed | none |
| `tools.grip` | Order of tool events | adapter | none |
| `tools.on` | `find_tool_for_joint` (male or ground half) for Cindy; the held bar for the Robotiq | adapter | R:`core/rhino_tool_place.py:510` |
| Ungrasp form A/B | Rhino is form B today: tool-only ungrasp R_M1, then a per-arm linear retreat R_M2 | exists as form B | R:`core/bar_action.py:1032-1129, 1550-1559` |
| Untighten | R_M0 "jointing screws untighten (bar still gripped)" before every ungrasp | **contradiction** (see §3 C1) | R:`core/bar_action.py:1537-1548` |

Prototype: `rhino_review/proto_builder.py` takes the per-bar records (base, approach/assembled/retreat/home, tool0 frames, tool-on joints, support keyframes). For the test they are extracted from the converted design. It builds 48 actions and 184 movements: J = 5, R = 3, H = 4, HR = 2. Results:

- **260814:** A passes; B gives 0 errors and the 4 B5 warnings (fake-bar partners).
- **260920:** A passes; B gives only the 2 B14 errors for B16's stale keyframe (295 mm).

The B11 "Alice joints jump" errors of both converted designs disappear (see §3 P3).

## 3. Pain points, missing features and contradictions

| # | Finding | Evidence | Severity |
| --- | --- | --- | --- |
| P1 | **Derived contacts fail Rhino's keyframe IK.** With compas_fab's convex hulls, Cindy's tool collides with the bar its half is mounted on in 41 of 80 keyframe states (19 of 20 bars; the converted 260814 and 260920 give the same numbers). The male half collides with its partner's bar at the assembled pose in 14 of 80 (12 bars). Rhino allows both on purpose today: tool↔tube in M1–M3, male↔female's bar in M2, and the cradle females | `rhino_review/py39_keyframe_collisions.py` output; R:`core/bar_action.py:661-863` (comments at 674, 704); C:`design/relations.py:98-129` | **blocker** |
| P2 | **Hold scene mismatch.** `hold_scene_for(B3_H)` enables Alice and Belle, with Cindy absent: the release-time robots. Rhino solves the hold keyframe against the release-time **bodies**, with Cindy frozen at the held bar's assembled pose and other holders at **hold start**. Rhino's separate release check is what the core's helper gives | C:`design/hold.py:364-381`; R:`rs_ik_keyframe.py:2478-2500`, R:`core/hold_action_builder.py:609-680` (solve), `456-533` (release check); `py39_check.py` output | **blocker** for support IK |
| P3 | **Builders must become one state machine over the schedule.** Today each action clones its own template state, and hold actions are exported in the release-time scene (future bars present, Cindy frozen). The `<=` interval also leaves Alice frozen at her held pose in B7's release, after she has let go of B3, which gives the B11 errors of the converted designs. The prototype threads one state and has 0 B11 errors | R:`core/hold_action_builder.py:503, 836-847`; R:`core/bar_action.py:1336-1346`; prototype output | major |
| P4 | **Ground halves can touch two walkable surfaces**, but A12 allows one mate per half. The converter's nearest-in-plan rule picked WG1 for `G1-T20Ground-0_ground`, which then collides with WG0 at the insert (260814; the reverse in 260920). New ground collisions also appear: Cindy's forearm vs WG1 at B13 (Rhino never collides with walkable ground) | `py39_keyframe_collisions.py` examples; C:`design/validate.py:262-265` | major |
| P5 | **Unknown robots stand at the origin.** `scene_at` turns `base: null` into `Pose()` and `joints: null` into zeros with flags, and the mirror loads non-acting robots as obstacles there, at zero joints. Rhino exports bars without IK, and those have `null` bases | C:`design/scenes.py:303-308`; C:`mirrors/compas_fab.py:170-174` | major |
| P6 | **`on` cannot be in a target.** The mount sets `on` and the retreat clears it, but `Target.tools` holds grips only and `end_state` keeps the start's `on`. The builder has to change `on` between movements (the prototype does), so a movement alone never says when a tool arrives or leaves | C:`design/types.py:185-193`; C:`design/plan_check.py:79-86` | major |
| P7 | **Solution write-back makes its own solution stale.** `RSLoadSolvedBarAction` writes solved bases and keyframes into user text. The next export then changes `design.json` and the action hashes, so the solution that was just adopted is stale, though nothing physical changed | R:`core/bar_action.py:481-562`; C:`design/solutions.py:94-108` | major |
| P8 | **Hash and determinism gaps.** Mesh files are named after their first owner in dict order. `design.json` stores only mesh file names, so a changed half, tool or ground mesh keeps the hash. Robot files are not hashed at all. No in-memory hash exists, so Rhino cannot tell whether the document still equals an export without writing | C:`design/write.py:70-75, 126-150, 283-299`; C:`design/solutions.py:94-104` (needs `design.folder`) | major |
| P9 | **Module reloads break caches.** Rhino commands `importlib.reload` their modules on every run. A cache in `sc.sticky` that holds core `Geometry` or `Tool` objects breaks `isinstance` checks after a core reload (the writer raises "unknown shape"). `design_model` is cached by `Tool` value with `Geometry` identity, so new tool objects on every command would make the mirror rebuild the full cell on every command | R:`rs_ik_keyframe.py:99, 152-153`; C:`design/scenes.py:239-245`; C:`design/write.py:181-190` | major |
| P10 | **Keyframe IK needs several start states at once.** Tamp's `solve_keyframe_chain` takes movements with a `start_state` (`RobotCellState`) and `target_ee_frames` `{"left", "right"}`, with the base in mm. The mirror gives a state only through `sync` and then `mirror.state`. Doable: sync three scenes and copy each state. The acting robot's `null` joints and base must be filled in the scene copy first (the mirror refuses them, as verified) | tamp `keyframe/ik_keyframe.py:70-140`; C:`mirrors/compas_fab.py:145-147` | minor |
| P11 | **Part seats are hard-coded in the core:** `T20/Male` identity and `T20/Ground` 180°. `T20Deck12/Male` and the T20Sub receivers are missing, but they are Rhino catalogue data (males sit at the TCP by construction; ground uses `M_tool_from_block`) | C:`design/plan_check.py:25`; `joint_pairs.json` `ground_joints[0].M_tool_from_block` | minor |
| P12 | **The core widens tool contacts.** It always adds `wrist_2`, `wrist_3`, `flange` and `tool0` to `mount_contacts`, so the Robotiq may touch `wrist_2`, which Rhino forbids | C:`robot.py:100-108`, C:`urdf.py:194`; R:`core/config.py:299` | minor |
| P13 | **Rhino finds movements by id.** It parses `_(J\|R\|H\|HR)_M<n>_` to locate approach, assembled and retreat. Schema 2 renumbers movements, and the converter keeps `B10_J_M4_tool_tighten_joint` for the insert | R:`core/bar_action.py:441-463` | minor |
| P14 | **In-memory validation trips on robot meshes.** Rhino's URDFs use `package://` meshes, so `validate(design)` with its default `check_robot_meshes=True` fails A4. Rhino must pass `False` (`write` does) | C:`design/validate.py:30-43, 222-227` | minor |
| P15 | **Copy/paste duplicates ids.** Copy/paste leaves joints pointing at the source bars, which gives duplicate joint ids. `Design.bodies` is a dict, so a duplicate would silently overwrite. The adapter must refuse duplicates (Rhino already has `_raise_on_duplicate_joint_key`) | R:`rs_reorder_bar_id.py:10-18`; R:`core/env_collision.py:261` | minor |
| P16 | **Units.** The document unit is arbitrary (`doc_unit_scale_to_mm`). Keyframes, `joint_pairs.json` and tamp use mm; the core uses m. A forgotten scale passes every A check (quaternions stay unit) and is caught only by B14 | R:`core/rhino_helpers.py:40-60`; tamp `dual_arm_ik.py:85-92` | minor |
| P17 | **No `RSImportDesign`.** A design without `source/` loses: fake bars; the joint placement DOFs (`position_mm`, `rotation_deg`, `ori`, `le_rev`, `ln_rev`, `variant_index`, `flipped`); the tool candidates; `supported_until` of bars that need no hold; ground and obstacle breps (meshes only); the base and grasp picks; build stage and colours. The keyframes, sequence, mates, markers and parts can be recovered | R:`core/joint_name_conventions.py:391-413` | minor (needs `source/`) |
| C1 | **Contradiction: untighten before every ungrasp.** Rhino runs R_M0 before every ungrasp, while the spec drops it ("untighten removes a bar"). B10 would actually accept a loosen drive that leaves `built` unchanged, but the spec says otherwise | R:`core/bar_action.py:1537-1548`; spec, Actions and movements | major (decision) |
| C2 | **Contradiction: Cindy's place at hold releases.** Rhino parks Cindy at every hold release ("finished and driven away"), and the converter wrote `robots/cindy: null` in HR states. In the schedule Cindy is still in the cell (B9_R, then B3_HR, then B10_J). Under schema 2 she should be present with her last known state, or `base: null`; with P5 that is unsafe today | R:`core/config.py:333-341`; converted `B3_HR_hold_release.json` | minor |
| C3 | **Contradiction: keyframes vs movement states.** Rhino stores 4 assembly keyframes per bar and 4 support values per hold. They map onto movement targets and starts with no loss, except the loading pose (`null`, as today). But the shared-core briefing says "keyframe IK syncs a mirror from `scene_after(bar)`". That scene is after the bar; Rhino needs `scene_at` of the transfer, insert and retreat with the bar carried | shared core report, Briefing: Rhino; C:`design/scenes.py:268-291` | minor (doc fix) |
| C4 | **Contradiction: renumbering and hashes.** `RSReorderBarID` renames bars, joints and tools: by design every hash changes and every solution goes stale. Because of P8, though, reordering bodies in the document or editing a mesh changes the hash wrongly (or not at all) | R:`rs_reorder_bar_id.py:30-40`; C:`design/write.py:126-150` | minor |
| C5 | **Contradiction: hold export states.** Rhino exports hold states in the release-time scene with the held bar hidden; schema 2 states must be the true world. Consistent with the spec once P3 is done; the hold helper (P2) carries the planning scene instead | R:`core/hold_action_builder.py:803-847` | (covered by P2, P3) |

## 4. Change requests, in priority order

**P0: needed before Rhino plans on the core**

1. **Allowed contacts follow the part** (P1, P4). In `relations.allowed_contacts`:
   - `on` extends to the part: the tool may touch `bar_of(on)` and that bar's halves.
   - A pending or engaged mate also allows each half to touch the partner's bar.
   - A half whose part is `*/Ground` may touch every present `ground/` body, the same rule as `ground_links`.

   Why: these are exactly the pairs Rhino allows today, and the measured collisions are hull artefacts or a 2.5 mm screw tip. Smallest change: three `add` loops, about 10 lines. Alternative: convex decomposition of tool meshes in the mirror, which costs more and does not cover the screw tip.
2. **Two hold scenes** (P2):
   - `hold_scene_for(design, action)`: bodies of `scene_after(release_bar)` with the held bar disabled, plus robots taken from the start of the hold's closing movement (Cindy at the assembled pose, other holders at that moment).
   - `release_scene_for`: the current behaviour.

   Why: these are the scenes Rhino solves the hold in and checks the release in. Smallest change: about 15 lines in `design/hold.py`.
3. **Unknown robots are parked, not placed at the origin** (P5). Smallest change: in both mirrors, a non-acting robot with `not base_tracked` is parked, and one with `unmeasured` joints is parked too or refused. Report it (for example `mirror.parked_unknown`). Why: Rhino exports most bars without IK while designing.

**P1: needed before the builder rewrite**

4. **`on` in targets** (P6). Make `Target.tools` entries `{grip?, on?}` (with `on` as a body or `null`), and have `end_state` apply them. Why: the mount and the approach set `on`, the retreat clears it, and B9 ("until `on` clears") and the monitor need to know which movement does it.
5. **Decide R_M0** (C1). Either hardware confirms it can go, and Rhino drops it; or the spec documents "loosen drive, `built` unchanged" as a relax step, which B10 already accepts.
6. **Deterministic, complete hashes** (P8, C4):
   - The writer sorts robots, tools and bodies.
   - Mesh files are named by content hash (`meshes/<sha1[:16]>.obj`), so `design.json` covers the meshes. URDF and SRDF content hashes go into `robots`.
   - New `design_hashes(design) -> {design, actions}`, computed in memory with the values the files would hash to.

   Why: Rhino must show "document ≠ export" and stale solutions without writing.
7. **Adopting a solution keeps it valid** (P7). New `solutions.adopt(solution, old_design, new_design)` re-stamps `solved_against` when the only difference is `null` bases and joints filled with that solution's own values (1e-9). Why: without it, the write-back from tamp to Rhino invalidates the trajectories it came from.
8. **A part catalogue in the design** (P11). A `parts: {"T20/Male": {"seat": pose}, ...}` table in `design.json`, written by Rhino from `joint_pairs.json` and `robotic_tools.json`; `check_plan` reads it, with `PART_SEATS` as the fallback. Why: seats are catalogue data, and `T20Deck12/Male` and future types would otherwise warn.
9. **Public `scene_of(design, state)` and `end_state`** in `design.scenes` (C3, viewers). Why: Rhino's viewers step through movement ends (keyframes), and IK draws candidate states.

**P2: small**

10. `mount_contacts` becomes authoritative: drop the implicit `TOOL_TOUCHES_ARM_LINKS` (P12), or have the converter write the four links explicitly.
11. Role-named movement ids as Rhino's convention (`B10_J_insert`, `B10_R_retreat`, `B3_H_close`). The converter renames to match (P13).
12. `CompasFabMirror.cell_state(scene)` returns a `RobotCellState` without syncing; it raises if the models or geometry differ from the built cell (P10).
13. Fix `requirements.txt`:
    - compas_fab is `wip_process` 995122d, not 1.1.0;
    - add `roslibpy==1.8.1` (compas_fab's backends import fails without it, verified);
    - note that pip `pybullet_planning==0.6.1` works.
14. Spec text:
    - mates come from the joint instances (`J<receiver>-<male>`, `joint_pair_name`), not from `joint_pairs.json` (a catalogue);
    - part names are block names with `_` → `/`;
    - the shared-core briefing should say `scene_at(transfer | insert | retreat)` instead of `scene_after(bar)`.

## 5. Integration proposal

**Where the core lives.** Extract `bar_assembly_core` into its own repository first. It imports nothing from teleop, verified on 3.9. Then add it to Rhino as the submodule `external/bar_assembly_core`, pinned to a commit and put on `sys.path` the way `core/robot_cell.py` does for compas_fab. Pinning the teleop repository instead would bring in ROS, data and 40 MB of meshes.

Entry scripts that import the core add `# r: trimesh==4.12.2`. The existing pins `numpy==1.24.4` and `scipy==1.13.1` already work.

The core is imported once and never reloaded: the per-command `importlib.reload` stays for Rhino modules only (P9).

**The adapter**, in two modules:

- **`scripts/core/design_source.py`, Rhino-only: `RhinoDesignSource.build() -> Design`.** It is called right after `repair_on_entry` and keeps no `Design` between commands. It reads:
  - bars as cylinders; halves with `part`, `mount` and `markers`; ground and obstacles;
  - robots and tools from config, `robotic_tools.json` and the asset URDFs;
  - mates; the schedule (`hold_schedule`);
  - the keyframe records from user text;
  - `producer`.

  It converts document units to metres once and refuses duplicate ids, fake bars and their halves, and reflected or scaled block transforms. A `GeometryCache` in `sc.sticky` is keyed by:
  - the block definition name plus the OBJ content hash (halves, tools);
  - `(radius, length)` (bars);
  - object id plus mesh CRC (ground, obstacles);
  - the `bar_assembly_core` module identity.

  `Tool` objects are cached the same way, so `design_model` and the mirror keep their models. Moving a bar changes only `BodySpec.pose`, so its `Geometry` is reused and no rebuild happens.
- **`scripts/core/design_builder.py`, Rhino-free and covered by pytest: the state machine of `proto_builder.py`.** It covers J (load, mount, grasp, transfer, insert), R form B (ungrasp, retreat, home), H (approach, open, to-grasp, close) and HR (open, retreat, leave). It emits typed `line`, `drives` and `ends_on`, and the `attached`, `built` and `on` changes. It replaces the `_build_m*` functions and the template-state patching for schema 2.

**Commands that change**

| Command | Change |
| --- | --- |
| `RSExportAllBarActions` / `RSExportBarAction` | `write(design)` plus `source/`. PyBullet is no longer needed for export. The legacy export is still written for tamp's headless planner until tamp reads schema 2 |
| `RSIKKeyframe`, `RSIKKeyframeAll` | Build the design with `null` joints. Sync `scene_at` of transfer, insert and retreat into `CompasFabMirror("robots/cindy")` with the base and seed filled in, and collect the states. Run tamp's `solve_keyframe_chain` inside `mirror.lend()` (with an mm shim). Write the keyframes back to user text as today |
| Support flow | `CompasFabMirror("robots/alice" \| "robots/belle")` with `hold_scene_for` (solve) and `release_scene_for` (release check); `solve_support_ik` stays Rhino code |
| `RSRebuildRobotCell`, the fingerprint and the Rebuild/Proceed/Abort prompt | Removed: the mirror's identity diff replaces them |
| `RSShowAssemblyPlan`, `RSShowBarActionPlan`, GH sequence preview | Steps = movement ends via `scene_of(end_state(m))`; robots by `kinematics.link_pose`, drawn with Rhino's own meshes |
| `RSLoadSolvedBarAction(All)` | Use `read_solutions` with stale marking, and write back through `adopt` |
| New `RSCheckDesign` | Runs A and B before export (B14 catches stale keyframes such as 260920 B16) |
| New `RSImportDesign` (last) | Opens `source/*.3dm` and checks that its hashes equal the design's |

**What stays Rhino-only:**

- modelling: the bar and joint solvers, registry editing, tool definition and swapping;
- user-text storage and undo;
- base picking and sampling on breps;
- ik_viz and display conduits;
- MoCap reading, stability JSON, prefab export;
- the ssik sidecar (tamp).

**Order of steps**

| Step | Work | Done when | Effort |
| --- | --- | --- | --- |
| 0 | Extract the core repo with a 3.9 CI job (ideally also Windows); merge `hs/mocap-experiment` into `main`; add the submodule | A Rhino 8 smoke command reads a converted design and runs `check_plan` from the pinned core | 2–3 d |
| 1 | Bridge: `RSExportAllBarActions` also runs `legacy.conversion.convert_export` on its own output (imports on 3.9, verified) and sets `producer` | The study .3dm exports a schema 2 design with 0 A errors and only the known B errors | 1–2 d |
| 2 | `RhinoDesignSource` without actions: bodies, mates, robots, tools, geometry cache. Re-baseline ids for 0ebbe4d and `_mocap` | Bodies equal the converted ones within 1e-6 after the id map; two commands on an unchanged document give the same `Geometry` and `RobotModel` objects; rebuild time measured on the largest .3dm | 4–5 d |
| 3 | Core change requests 1–4 and 6–9 (with the core owner); decide C1 | Core tests pass; on the converted designs, `py39_keyframe_collisions.py` leaves only reviewed pairs | 3–5 d (core) |
| 4 | `design_builder` state machine, tests | `write` + `check_plan` on the study document: 0 errors other than data errors (stale keyframes); an equality diff against the converted design shows only intended changes | 6–8 d |
| 5 | Plan on the mirror: keyframe IK, support IK, release check; remove the fingerprint and rebuild prompt | Re-solving all bars of the study document gives the same solved set as the old cell, and no unreviewed collision pairs | 6–8 d |
| 6 | Viewers and solutions: scenes, core FK, `read_solutions`, `adopt` | `RSShowAssemblyPlan` and the GH preview match today's captions step by step; stale solutions are marked | 4–6 d |
| 7 | `RSImportDesign`; retire the legacy export once tamp reads schema 2 | Import, then export, of an unchanged design gives identical hashes | 3–4 d |

The total is about 30–40 working days (6–8 weeks) for one developer who knows the plugin. Steps 2 and 4 can run in parallel with step 3.

**What I could and could not verify here.** On Linux CPython 3.9.25 with Rhino's exact pins (numpy 1.24.4, scipy 1.13.1, trimesh 4.12.2, compas 2.13.0, compas_robots 0.6.0, pybullet 3.2.7, pybullet_planning 0.6.1, roslibpy 1.8.1, compas_fab 995122d), all of these ran:

- `read`, `validate`, `check_plan`, `scene_at`, `scene_after`, `hold_scene_for`, `link_pose` and `content_hash`;
- `CompasFabMirror` build (0.4 s), sync (0.05 s), `collisions` and `lend()`;
- the imports of `PyBulletMirror` and `legacy.conversion`.

Measured on 260814:

| Step | Time |
| --- | --- |
| `read` | 0.08 s |
| `check_plan` | 0.15 s |
| `scene_at` | about 1 ms per movement |
| FK of all 37 links | 6–7 ms |

Not verified:

- Rhino's embedded CPython on Windows and its `# r:` installer;
- .NET threading together with PyBullet;
- the rebuild cost of `RhinoDesignSource` on a real document;
- `RhinoDoc.WriteFile` for `source/`;
- tamp's chain solve on mirror-built states;
- whether `scaffolding_env` already has roslibpy.
