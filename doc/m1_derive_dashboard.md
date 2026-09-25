# M1 start-derivation dashboard

A local web page that explains one run of the M1 "derive start" sweep: which
home bar poses the planner tried, why each one was rejected, where the time
went, and what any of it looks like in 3D.

Written for the case that prompted it — *"this scene looks easy, why does the
derivation take two minutes?"*

---

## 1. What the sweep actually does

Before M1 can plan a path, it needs a **start**: a bar-loading pose the robot
can hold the bar in. That pose is not authored anywhere, so the planner derives
it, working **backwards from the goal**:

1. Take the goal configuration (normally M2's authored start conf) and the bar
   pose it implies.
2. Pick a candidate **home** bar pose — one of three carry anchors
   (bar across the front / upright in front / fore-aft over the robot), at one
   of ~13 orientations (rolled about the bar axis, or yawed about the base
   vertical), offset by one of 343 positions on a 10 cm grid.
3. **Walk** the bar in a straight line from the goal pose to that home pose in
   1 cm / 1.4° steps, solving dual-arm IK at every step, each seeded from the
   previous one.
4. Keep the first home pose the walk reaches with a collision-free arm.

The walk is the expensive part, and it is where nearly everything fails.

**Why a successful run still takes the full 120 s:** the sweep only stops early
if it finds a *completely clear corridor* (then that corridor IS the M1 path and
the RRT is skipped). A merely-usable start is remembered and the search
continues, until each anchor has spent its 40 s share. So on a scene with no
clear corridor, every run costs the whole budget by construction.

**Which goal it walks back from.** When M1's goal is M2's authored start
configuration, that configuration is now **kept** as long as a short test walk
(collisions ignored, at most 8 s, usually well under 1 s) shows the arm can
reach *some* home pose from it continuously. Only when it cannot is the goal
moved onto another IK branch — the "re-pairing" that exists for genuinely
stuck goals. The summary says which happened. This matters because a moved
goal is no longer where M2 starts, and on B3 the move alone was what made
every home pose fail (see §5).

---

## 2. Running it

One terminal, left open for the session:

```bash
cd /home/su/ros2_ws
source venv/bin/activate && source install/setup.bash
bash src/husky-assembly-teleop/scripts/fetch_dashboard_vendor.sh   # first time on a machine
python src/husky-assembly-teleop/scripts/m1_dashboard_server.py    # http://127.0.0.1:8765
```

Runs arrive from either producer, and the open page is notified within a second:

- the monitor's **M1: Derive Start/Goal only (no RRT)** button, during a session;
- the monitor's **Plan Movement** on M1 — every attempt is recorded, including
  the RRT search that follows the sweep (one run per retry, labelled
  *attempt N*);
- `scripts/derive_m1_headless.py --bar B3 [--anchor back]`, at the desk with no
  robot and no mocap; add `--plan` to run the whole planner (derive + RRT) the
  way Plan Movement does, and `--no-hide-built` to keep the already-built bars
  as obstacles.

### A whole design problem in one go

```bash
python src/husky-assembly-teleop/scripts/derive_m1_headless.py \
    --problem 260921_motion_sample --bar all --plan --no-hide-built
```

`--bar all` walks every bar that has a transfer action file, in bar order, and
one bar's failure does not stop the rest. At the end it prints a table (bar,
file, base, result, time, run file) and saves it as
`recorded_data/m1_derive_runs/batch_<timestamp>_<problem>.md`; every bar's run
is on the dashboard as usual.

Two things happen on the way that the single-bar form also does:

- **Both export schemas load.** The legacy `B6.json` (M0..M4 in one file) and
  the split `B6__J.json` (jointing: load, mount, grasp, transfer, tighten,
  insert) + `B6__R.json` (release: untighten, ungrasp, retreat, home). The
  transfer is `M1` in the old ids and `J_M3` in the new ones;
  `bar_action_io.movement_role` translates, and the monitor uses the same
  helper.
- **Bars without a base get one.** An export whose robot base sits at the
  world origin is not placed yet. The script then runs the code's own
  heuristic (`place_base`): stand 0.7 m behind the bar's ground projection,
  facing the insertion axis, and sample the walkable ground in growing rings
  until the transfer -> insert -> retreat IK chain solves. The base and the
  keyframe configurations are stamped onto the action and saved as
  `B6__J.solved_keyframe.json` + `B6__R.solved_keyframe.json` next to the
  export; the next run picks those sidecars up instead of searching again
  (delete them to force a new search). The batch table's *base* column says
  `authored` or `heuristic (x, y) m`.

Files land in `recorded_data/m1_derive_runs/` (~4 MB each) and the baked 3D
scene in `recorded_data/m1_dashboard_scenes/<problem>/` (exported once per
design problem, ~0.4 MB). Both are local and git-ignored — copy a run onto the
Drive by hand if it is worth keeping.

---

## 3. Reading the page

### The summary

Six or seven sentences at the top. The ones that matter most:

> The goal has the middle of the bar 73 cm ahead, 77 cm right, 87 cm up of the
> robot base.

Everything on this page is in the **robot's own frame** — x forward, y left,
z up, from the base. Not world coordinates, not "delta from the anchor".

> The base everything is measured from was measured live by the mocap while the
> derivation ran.

Every position in a run is relative to the mobile base, so a run that stood the
robot in the wrong place is wrong *everywhere*. The line names which base it
was, and the note under the scatter repeats it with the coordinates:

- **measured live by the mocap** — the monitor's derive button with mocap
  tracking the base (`_apply_live_base_to_movement` wrote the live reading into
  M1's start state before the sweep ran);
- **read from the BarAction file's own M1 start state** — no mocap, so the
  authored base stands in;
- **chosen by the headless base-placement search** — the export left the robot
  at the world origin and `derive_m1_headless.py` placed it, saving the answer
  as the `.solved_keyframe` sidecar.

> bar_B46 is 200 cm long, and every bar pose in this run is the pose of the
> bar's own frame, whose origin sits right at one end of it — not in the middle.

**A bar's stored pose is the pose of its own frame, and that frame sits at the
end the assembly joint is on.** On B46 the grippers hold the bar 17 cm and
81 cm from that origin, and the bar runs 2 m the other way. So the page does
not plot the stored position: it moves every bar onto **the middle of the bar**
(the bar extent's centre, or the grips' midpoint on a run file written before
the extent was recorded), and the sentences quote that same point. A dot means
what it looks like — the bar's middle, with the bar drawn through it.

> The goal configuration was moved onto a different IK branch before the sweep,
> by up to 203 deg on one joint (worst-arm branch distance 46 deg) — so M1 no
> longer ends exactly at the configuration M2 starts from.

**Read this line first.** Before the sweep, ssik re-picks the goal
configuration on whichever IK branch it thinks is most compatible with a home
anchor. A large number here (tens of degrees, let alone 200) means the goal the
sweep walks back from is *not* the configuration M2 was authored at — and every
candidate then inherits that branch. See §5.

> A usable start was found after 75 s, but the sweep kept looking for a fully
> clear path for the remaining 46 s of its 120 s budget.

The gap between these two numbers is time you could get back (§6).

> The "bar upright in front" carry used up its 40 s share at the yaw+30
> orientation after 747 candidates, so it stopped trying more positions for it.

Each carry anchor gets an equal slice of the budget so a hopeless one cannot
starve the others. A carry that runs out has **not** exhausted its options —
it was cut off. If the answer came from a carry that was cut off early, that
carry deserves a longer look (`--anchor <name>` gives it the whole budget).

### The per-variant table

One row per home orientation, in the order tried. The columns that carry
information:

| Column | Means |
|---|---|
| **IK missed** | no solution holds the bar at that pose — genuinely out of reach |
| **branch flip** | there *is* a solution, but it is on another branch: a joint jumped more than 10° in one 1 cm step |
| **collides at home** | the walk arrived, but the arm is in collision there |
| **blocked** | arrived, home is clear, but the straight path hits something in between |
| **median cm reached** | how far the typical walk got before dying — small means it fails immediately |
| **seconds** | time spent on that orientation |

The split between **IK missed** and **branch flip** is the single most
diagnostic number on the page. See §5.

### The 3D scatter — "Where the home poses were"

Every dot is one home bar pose tried, plotted in the robot frame, at **the
middle of the bar**; the stick through it is the bar itself. The white diamond
and the thick white stick are the goal bar, drawn the same way, so you have a
real-scale reference to compare against.

The second dropdown sets how much of the bar is drawn. Everything is measured
along the bar's own direction from its stored origin (one end), so the drawn
piece always lands where the planner really had it:

- *at its full length* (the default) — the whole bar, from the recorded
  `active_bar_extent_local` (2 m on B46). The dot is its exact centre;
- *only the piece between the grippers* — what the two tool0s actually hold
  (0.96 m on B3, 0.64 m on B46). It lands exactly on the grippers of the robot
  overlay, which is the quickest reachability check there is. On a bar gripped
  off-centre the dot is not on this segment — the dot is still the bar's middle;
- *direction only* — a 30 cm marker centred on the dot, readable when the dots
  are dense. This one is a **direction hint, not the bar**;
- *off*.

A run file written before this readout has no bar extent: the grips' midpoint
then stands in for the bar's middle, and *at its full length* falls back to the
grip span.

**The RRT card is the exception.** Its trees and path are bar positions the
search logged without orientation, so they cannot be moved onto the bar's
middle; that chart plots bar origins, and its legend says so.

The third dropdown draws **the husky itself** into the same plot, as a
see-through grey shape, at one of two configurations:

- **robot at the goal pose** (the default) — the arms exactly where the sweep
  started walking back from, i.e. where M1 has to end;
- **robot at the pick-up pose** — the bar-loading pose the sweep settled on, so
  you can see the two ends of M1 side by side and judge how far the arms have
  to travel between them. Greyed out when the run found no start.
- **no robot** — the dots on their own.

Without the robot the cloud is just a blob of coordinates. With it you can see
at a glance whether the candidates sit *in front of* the arms, behind the
chassis, or down through the deck — which is usually the first thing to check
when a whole carry anchor fails. The robot is a stand-in drawn from the URDF's
collision meshes (the heaviest link, the top chassis, is drawn as a plain box to
keep the plot responsive), not the exact shape the planner checks; for that, open
the 3D viewer.

This is how you check the obvious thing first: **do any of the candidates point
the same way as the goal?** If the goal bar is horizontal and every stick near
it is horizontal too, the failures are not about orientation — which is what
sends you to the branch question in §5.

Hovering gives the full sentence for that attempt; clicking opens the 3D viewer
on it.

Colour follows outcome by default; `track_break` (the usual majority) is
deliberately recessive grey so the informative outcomes stand out. The dropdown
re-colours by **time spent** or **how far the walk got** instead — useful for
seeing whether the failures are uniform or clustered in one region of space.

### The timeline — "How the time was spent"

Each attempt on the clock, one lane per carry anchor. The green line marks when
a usable start was found; the dashed amber lines mark where a carry's 40 s share
ran out. A long stretch of grey after the green line is the corridor hunt that
found nothing.

### The RRT search card

Shown only for runs that went on to search (Plan Movement, or `--plan`). It
answers the question the derive stage cannot: **given this start and this
goal, did a path exist?**

The text says how the search ended — the trees met, the time budget ran out,
the iteration cap was hit, or an endpoint itself collides — with the time,
iteration count, and how big each tree grew. When the trees never met it also
gives their **closest approach in centimetres**: a few cm means they were about
to connect and more time might do it; tens of cm means they were growing in
different places and time will not help.

The second line is the more telling one: **why the tree extensions stopped**,
as percentages — *jumped joint branch*, *hit something*, *found no IK
solution*. A search dominated by branch jumps is failing for the same reason
the sweep did (§5, item 1); one dominated by collisions is boxed in; one
dominated by IK misses is reaching out of the workspace.

The plot shows where the bar went during the search, in the robot frame: the
tree grown from the pick-up pose in red, the tree grown from the goal in blue,
the pick-up pose as a green diamond, the goal as a white one, and the path as
a thick green line when there is one. Two clouds that stall with a gap between
them, each hugging its own end, are the picture of a branch-sheet problem.

### The 3D viewer

Opens when you click any dot.

- **Drag** to orbit, scroll to zoom, right-drag to pan; **Reset view** re-fits.
- The **slider** (or ← →) walks the bar from the goal pose to where the attempt
  died. The caption names each step in centimetres travelled.
- **Ghost bars** mark the three reference poses: blue = the goal, green = the
  home pose this attempt was aiming at, red = where the walk broke.
- On a colliding frame the two offending parts turn **orange** and **cyan**
  (the same pair the PyBullet-side diagnosis uses), with a white dot at the
  deepest contact point.
- **"show the built bars the planner ignored"** reveals the already-built
  assembly, which the mocap-accuracy setup hides from collision checking. Worth
  a look when a collision seems to come from nowhere — or when you want to
  confirm a bar that *should* have been in the way was correctly ignored.

---

## 4. The five outcomes

| Outcome | What happened | What it tells you |
|---|---|---|
| **clear path found** | walked the whole way, and the straight path is collision-free | M1 is already solved — this corridor becomes the path, no search needed |
| **reached home, path blocked** | arrived, home is clear, something blocks the middle | the endpoints are fine; the RRT must find a detour |
| **reached home, collides there** | arrived, but the arm collides at the home pose | that home pose is unusable; look at *what* it hits |
| **never reached home** | the walk broke partway | split by reason — see below |
| **clear coarse, not fine** | passed the coarse screen, failed the fine re-walk | rare; the corridor is marginal |

A **never reached home** is further split:

- **IK found no way to hold the bar there** — the pose is out of the arms'
  reach. Expected far from the robot; suspicious close to it.
- **a joint jumped N° in a single 1 cm step** — the pose *is* reachable, but not
  from the goal's branch without the arm flipping. This is a **continuity**
  failure, not a reach failure, and it is the one worth chasing.

---

## 5. How to read a slow run

Work down this list; the first line that matches usually explains the run.

1. **Is the split dominated by branch flips?**
   In the first B3 run: 728 walks broke, of which **722 were branch flips and
   only 6 were IK misses**. Almost every home pose is reachable; what fails is
   getting there continuously from the goal.
   → The problem is the **goal branch**, not the home poses. Check the
   re-branch line in the summary (203° on one joint for that run) and the
   worst-arm branch distance. A goal on an awkward branch dooms the whole sweep,
   and no amount of extra sweep time will help.

2. **Is it dominated by IK misses?**
   The home poses are genuinely out of reach. Try a different carry anchor, or
   look at where the base is parked relative to the goal.

3. **Are there many "collides at home"?**
   Click a few and look at *what* they hit. Self-collisions and tool-against-own-wrist
   pairs (e.g. *right wrist 1 against AT3R, 4 mm*) point at the collision model
   or the allowed-collision matrix rather than the scene.

4. **Are the blocked corridors all blocked near the goal?**
   *"collides 10 cm into the 158 cm walk: dual arm bulkhead against right upper
   arm"* means there is no straight exit from the goal at all — only a detour
   can work, so the RRT will be doing real work.

5. **Did a carry anchor get cut off before it got anywhere?**
   Re-run that carry alone with `--anchor <name>` to give it the full 120 s.

6. **The sweep found a start but the RRT failed.**
   A start that was found without a clear corridor certifies two endpoints,
   nothing more — the corridor test is a free-win detector, not a feasibility
   gate. Open the RRT card: if the extensions mostly *jumped joint branch* and
   the trees stalled far apart, the endpoints are on branches that do not
   connect; check the goal line in the summary. If they mostly *hit
   something*, look at the closest approach and the plot for what boxed them
   in.

---

## 6. Things you can change

The sweep's cost is roughly *candidates × steps per walk × IK time*. From the
first B3 run: 747 candidates, 23 040 IK solves, **118 of the 120 s spent purely
in IK** (collision checks were 2.4 s).

- **Bail out earlier.** Once a usable start exists, the remaining time only buys
  a clear corridor. On scenes where corridors are blocked right at the goal
  exit, that time is spent for nothing — a short grace period after the first
  usable start would cut a typical run from 120 s to ~10 s.
- **Screen more coarsely.** The screening step is 1 cm / 1.4°
  (`screen_step_m`, `screen_step_rad` in the run's `budget`); a winning corridor
  is re-walked finely anyway, so coarser screening costs accuracy nowhere.
- **The goal branch is now protected.** M2's authored configuration is only
  moved when a test walk shows it cannot reach any home pose (§1). On B3 this
  is the difference between 120 s with no corridor and 36 s with one.

---

## 7. Where things live

| | |
|---|---|
| server + page | `husky_assembly_teleop/dashboard/` (`server.py`, `static/app.js`, `static/viewer.js`) |
| run files | `recorded_data/m1_derive_runs/<timestamp>_<problem>_<bar>_<anchor>.json` |
| baked scenes | `recorded_data/m1_dashboard_scenes/<problem>/scene.glb` |
| run schema + wording | `husky_assembly_teleop/dashboard/run_schema.py` |
| headless producer | `scripts/derive_m1_headless.py` (`--plan` for derive + RRT, `--bar all` for a whole problem) |
| action loading, roles | `husky_assembly_teleop/bar_action_io.py` (`parse_bar_action`, `movement_role`, `sibling_action_path`) |
| base heuristic | `husky_assembly_tamp/keyframe/walkable_ground.py` (`derive_seed_base`, `solve_chain_with_base_search`), called from the script's `place_base` |
| tests | `test/test_m1_dashboard.py` |

The per-candidate trace comes from `derive_constrained_start_tracked` in the
**`husky_assembly_tamp` submodule** (`motion_planner/dual_arm_task_space_rrt/core.py`),
the RRT's trees and outcome from `plan_pose_birrt` in the same file, and the
run-level context plus the goal-branch probe from `_derive_constrained_start_for_plan`
in `motion_planner/api.py`. All are submodule edits — commit them there, or a
`git submodule update` will wipe them.

A run file is self-contained apart from the scene: it stores configurations,
not poses, and the server rebuilds the poses with forward kinematics from the
URDF (checked against the planner's own numbers to 0.001 mm by
`test_forward_kinematics_matches_the_planner`).
