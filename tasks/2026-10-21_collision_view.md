# Collision view: see what a collision check hit (2026-10-21)

Status: **built**; tried in the monitor on a simulated Belle (arm planner panel, take 1's waypoint 10 as target). Teleop only (branch `jg/viser-cleanup`); replaces the roadmap's `collision_diagnosis`.

## Goal

Show in the viser monitor *what* collided when a check or plan fails: both parts of each colliding pair, red and
see-through, posed as they were checked, with the contact points and depth. Hits from several checks (calibration
takes, a failed arm plan, later the base planner) can be listed and shown at once, each toggled on its own.

Calibration needs it first. Its collision check skips takes the real robots ran:

- Alice/Belle takes 1, 2: wrist links hit `ipad_rack_link`. The rack's collision mesh is one solid 72 L block.
  Showing the shape PyBullet tests (each mesh's convex hull) settles this at a glance.
- Take 8: the upper arm hits `top_plate_link` at waypoint 1 (shoulder at −3.15 rad). The depth tells whether this
  is millimetres (rounding) or a real overlap.
- Cindy: forearm or upper arm hits the other arm or its tool. Showing the world as checked (other arm at stow, its
  default tool) tells whether the check's setup matches the old sessions.

No core change in this task: contacts are computed in teleop and marked as a candidate to move to the core.

## Decisions

- A hit lists **every pair that blocks** the configuration. A path or target check moves one arm, so a pair blocks
  when one side moves with it, as in `SearchCheck`; that side comes first. A start is checked in full: every pair.
- A hit carries the scene as checked (`scene_at`: the snapshot with the robot at the hit joints, held bodies moved
  with it). The viewer draws from it with the core's `scene_parts`: no FK, no PyBullet, no drift.
- Producers return hits; UI plugins push them to the viewer. Headless planners never depend on it.
- The reporting plugin decides which hits start ticked (`shown`): the arm planner panel ticks its hit, calibration
  ticks none.
- `CollisionHit.message` keeps the old form ("target is in collision: A with B") but names links and tools short
  (`CollisionHit.name`: "ur_arm tool with ipad_rack", not raw ids); calibration's progress file keeps the text.
- No core change: contacts are computed in teleop, marked as a core candidate.

## What was built

- `world/collision_hit.py`: `Contact` (narrowed pair, two points, signed depth) and `CollisionHit` (scene,
  robot, where, pairs, one contact or None per pair, `hulls`), `message`; `scene_at`.
- `plugins/arm_planner/planner.py`: `collision_hit` runs compas_fab's full report at the hit joints, keeps the pairs
  with a moving side (`_moving_ids`), and takes each pair's deepest `getClosestPoints` contact (`_contact`, `_bodies`).
  A whole other robot (one tool in the cell) is narrowed to the link or mounted tool the contact touched
  (`_touched`). `ArmPlanningWorld.snapshot` replaces `label`. `path_hit` and `plan_arm` return hits.
- `PlanResult.hit`, `Plan.hit`; `PlannerService._run` copies it into the failed plan.
- `ArmPlannerContract.check_path` resolves with `CollisionHit | None`; a robot that can't be checked raises
  ValueError, a failed check RuntimeError.
- `shared/collisions_contract.py`: `replace(source, hits by label, shown=())`, `clear(source)`.
- `plugins/collisions.py`, in **Plan › Collisions**: As checked, Focus, then one folder per source (what its hits
  hit most, Clear, a `CheckList` of hits). Ticked hits draw both sides of every pair (moved solid red, hit see-through cyan), contacts
  as two dots, a line and "x mm deep"; Focus adds every robot of the hit's scene faint grey. A hit is prepared on a
  thread before it draws (`_parts_and_hulls`): a robot's first `scene_parts` loads its meshes (~0.8 s, which stalled
  the first trial for 918 ms), and hulls are computed there too, cached per mesh.
- Drawing: viser (three.js) has no order-independent transparency; it sorts see-through objects by their centres.
  See-through shapes that write depth hide each other's insides where they intersect; ones that don't (GLB, alphaMode
  BLEND: `_see_through`) pop in front of each other as the view turns. So the moving side is solid red (viser's own
  shapes, depth-correct per pixel) and only the hit side is see-through cyan, without depth writes: one see-through
  colour cannot pop, and the red stays visible inside the cyan. The focus robot is faint grey, also without depth
  writes. Only the deepest contact gets a depth label; `CheckList` labels wrap.
- Calibration: `requires` adds `collisions`; `_check_collisions` reports the colliding takes ("take N"),
  `_forget` clears them. `arm_planner_panel`: a plan refused for a collision shows its hit, ticked; the next plan
  removes it.
- Tests: `test/plugins/arm_planner/test_arm_planner.py` (every blocking pair with contacts, start in full, held body
  moved, another robot narrowed to a link), `test_arm_planner_plugin.py` (the panel reports and clears),
  `test/plugins/collisions/test_collisions_plugin.py` (real viser server: ticking, replace, clear, focus, hulls,
  labels).

## The calibration takes, default tools (`take_hits.py` in the session scratchpad)

| Robot / arm | Take | Pairs that block it (moving side → hit), depth |
|---|---|---|
| Belle 0805 ur_arm | 1 | tool → ipad_rack_link at waypoint 10, 10.9 mm |
| | 2 | tool → ipad_rack_link on the move to waypoint 14, 18.6 mm |
| | 8 | upper arm → top_plate_link at waypoint 1, 5.0 mm |
| Cindy 0806 left | 6, 9 | forearm / wrist 1 → own tool, 16.6 / 1.0 mm |
| | 7 | forearm → right arm's tool, 2.4 mm |
| | 8, 10 | six pairs with the right arm, upper arm up to 96 mm |
| Cindy 0806 right | 7, 9, 11, 12 | wrist 1 / forearm / upper arm → left arm's tool, 1.3–22 mm |
| | 8 | four pairs with the left arm, up to 41 mm |

Take 8's top plate overlap is 5 mm: more than rounding, small enough for a hull or pose difference.

## Clearance trips

- `CollisionHit` lives in `world/collision_hit.py`: the clearance check runs in `world/`, below `shared/`. It has
  `concave` (ids whose meshes the check used as they are; the PyBullet mirror builds free, unmeasured bodies with a
  non-convex mesh concave, the rest as hulls) and `margin` (0 for a collision; a clearance says "came within").
- `BaseCollisionWorld.collision_hit` (replaces `hit_where_synced`) lists every pair within the margin, robot or held
  body first, with its closest contact, allowed contacts left out; a robot side is narrowed to its link or mounted
  tool (`_names`, a core candidate next to `PyBulletMirror.collisions`). A contact's depth is positive for a gap.
- `ClearanceChecker` returns (key, why, hit); `Clearance.hit`; the monitor keeps the last ten trips
  (`ClearanceTrip`: time, robot, task, problem, hit) for `ctx.clearance_trips`.
- The collisions plugin lists trips with a hit under "clearance", newest ticked, labelled time and robot; Clear hides
  the trips up to then. It shows trips only while loaded: calibration and the arm planner panel load it, other
  plugins don't (user, 21 Oct: load it by name).

## Still to do

- Try calibration's own flow (Sweeps checks the takes) on a simulated robot.

## Follow-ups

- **Clearance at speed** (user, 21 Oct: noted, not now): the 10 cm margin holds up to about 0.15 m/s (D-pad Speed
  0.5); at 0.21 m/s (0.7) the robot touches. From the pose a check sees to standing still takes about speed × 0.5 s:
  base_exp measured on Alice a 210 ms delay from command to wheels (Wi-Fi included) and a 300–340 ms wheel rise (a
  lag, no acceleration limit up to 0.3 m/s; `alice_tiles_commands`), and the monitor adds 0.1–0.15 s (mocap age, a
  tick to the snapshot, a tick to act; a check itself takes ~5 ms). Proposed: also check the robot along its current
  motion (arc) up to where it would stop, stop time ~0.6 s, with the same margin (`BaseCollisionWorld.hit_by` per
  pose); motion from the last sent twist, the follower's command and wheel odometry, whichever are fresh; direction
  aware, so driving away from something never trips. Open: one stop time for all robots or per robot.

- **Move `hit_contacts` to the core** as `CompasFabMirror.contacts(joints)` (submodule commit first), then delete
  the teleop copy. Same for the base planner's world. With Coal (`project_coal_vs_pybullet`) contacts come natively.
- Joint limits before collisions in `path_hit`: report "waypoint 3 is outside the joint limits" as its own reason.
  Several yh/ debugging sessions found an "in collision" that was really a limit or 2π wrap.
- Allowed contacts in the hint: each side's `touch_links` / `touch_bodies` from the checked state, to answer "why is
  this pair checked at all".
- Path hits: the last clear sample as a second faint ghost next to the hit (tamp holding-prm dashboard).
- Jog the focused hit's joints and re-check live (Rhino `InteractiveCollisionCheck`), likely through the arm
  planner panel's sliders.
- When the tamp pin moves to yh/ssik-speedup: turn its failure `report` (`first_colliding_state`, `rejected_by`)
  into `CollisionHit`s instead of a second report shape.

## Prior art

| Where | What it shows | Kept | Left behind |
|---|---|---|---|
| Old monitor `old/cc_diagnosis.py` (Huang Su, Aug; same on master and yh/multi-robot-monitor) | Goal IK failures only: full-report pairs re-queried with `getClosestPoints`, the 4 deepest drawn in PyBullet (A orange, B cyan, AABB, witness line, "a <-> b (x mm)"), all printed | full report, deepest first, contact link from the contact tuple, depth from index 8, contrasting colours, non-blocking | one failure at a time; drawings at fixed world points drift once the cfab robot is re-posed; colour cache keyed by reused PyBullet ids; monkeypatches `client._check_collision` |
| pybullet_planning `draw_collision_diagnosis` (old `utils.py` start/end checks) | First contact, clones in red/green, camera moves | — | blocks on `input()`, moves the camera, unsigned depth |
| yh/multi-robot-monitor `husky_monitor.py` (DPG + PyBullet, not viser) | Endpoint and search diagnoses with a one-line VERDICT, "boxed in" sampling, pair histograms; an M1 three.js dashboard with contact spheres | source summary of frequent obstacles; contact spheres | the sampling verdicts (planner-specific; later, per planner) |
| tamp yh/holding-prm dashboard | Rejected node as a ghost next to the last accepted one, colliding parts red, "what collided most" | (follow-ups above) | — |
| Rhino (main, hs/mocap-experiment) | Offending links/tools/bars painted red on CheckCollision; IK candidate cycling; live jog dialog; display/collision mesh toggle | red for the offender, "As checked" toggle | candidate cycling (no IK candidates in teleop yet) |

All earlier tools turned names into text or PyBullet ids at the checker. Here the hit keeps scene ids and the
scene as checked, so it draws correctly however the world moves afterwards and works for every source.

