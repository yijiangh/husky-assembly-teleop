# The monitor's UI and plugins

What each panel, tab and plugin of the monitor does. To install and start the monitor, see the
[README](../../README.md); for the phone page, [phone_manual.md](phone_manual.md).

## Panels

Some plugins show content in panels separate from the main control panel. Panels can be moved, docked, collapsed and resized:

- Drag the bar at the top of a panel to float it, or drop it on a screen edge or next to another docked panel to dock it.
- The control at the right end of that bar collapses and expands the panel.
- Drag the edge of a floating panel to resize it.

Where they start:

- The main panel (Stop all, health) floats in the top left corner.
- The console, minimized, in the bottom right, and the **Plan plots** and **Exec plots** windows (`assembly_steps`,
  `movement_exec`), minimized to their bars, at the bottom, right of the main panel; each opens by itself when it has
  something new to show.
- The plugins' tabs are docked on the right, in the order the plugins start: **Assembly** (below), **Collisions**,
  **Control** (`robot_control`, a tab per robot), **Plan** with a tab per planner panel (**Base**, **Motion**,
  **Arm**), and a tab per other plugin; each plugin's tabs carry its icon. Drag a tab out to give it a window of its
  own, or into another panel.
- Tools and plots not needed every session (`obstacles`, `debug_gizmo`, `mocap_probe`, the calibration plots) start in
  a minimized strip at the right edge, nested under their plugin's name.
- Some plugins keep extra views in folders at the bottom of their tab, closed at start: `mocap_probe` shows the probe
  state big enough to read from a distance in a **Status** tab inside the **Big status** folder; `base_exp` shows its
  live error and command plots in an **Errors and commands** tab inside the **Live plots** folder.

In the browser's 3D view, left-drag rotates, right-drag pans, scroll zooms.

### Your own layout

Dragged windows are not remembered (viser never reports them). To keep a layout, save it as
`~/.config/husky_monitor/layout.json` on the laptop running the monitor (`mkdir -p ~/.config/husky_monitor` first), or
name another file with `layout:=<path>`. It is yours, never in git; without it the monitor uses the layout above.

It can give a top-level folder a window of its own, move a plugin's tab or start it hidden, place a window a plugin
made itself, by its title, and set preferences: the monitor's `ghost_timeout` under `preferences`, and each plugin's
own under `plugins.<name>.preferences` (`console`: `hidden`, the sources hidden at start; `assembly_run`:
`big_buttons`, big buttons in the Assembly tab); the monitor logs each entry it cannot use and skips it. The format is
in `ui/layout.py`.

Example, YH's layout: the Plan tabs in a floating window right of the main panel; the two plot windows minimized side
by side at the bottom left (a negative coordinate counts from the right or bottom); the console's ticks and runtime
lines hidden; big buttons in the Assembly tab; ghosts kept until another plugin shows its own (`0`):

```json
{
  "windows": {"Plan": {"float": [410, 15], "width": 420}},
  "plugins": {
    "console": {"preferences": {"hidden": ["ticks", "runtime"]}},
    "assembly_run": {"preferences": {"big_buttons": true}}
  },
  "panels": {
    "Plan plots": {"float": [15, -15], "width": 460, "height": 560, "minimized": true},
    "Exec plots": {"float": [485, -15], "width": 460, "height": 560, "minimized": true}
  },
  "preferences": {"ghost_timeout": 0}
}
```

### For plugin authors

Plugins add tabs with `ctx.view.ui()` and `ctx.view.tab()`, and plugins with the same tab path
share it through bands, `ctx.view.ui(band=...)` (`plugins/__init__.py`). To pick a plugin's icon,
`scripts/icon_gallery.py` shows every icon viser ships, searchable (tabler.io shows newer versions that can look
different).

## The Assembly tab

The **Assembly** tab holds `process`, `assembly_steps`, `movement_exec` and `assembly_run`; it is where you pick and
run the current step, top to bottom:

1. the process section (folder, Load, its two boxes, the action list, the step slider, the movement's details);
2. the step's chips and lines;
3. **Live IK**; **Check tool0**; **Plan**; Play / Clear; the **Preview time** and **Motion time** sliders;
4. **Exec**; the prompt with **Confirm** | **Cancel**;
5. **Mark done** | **Reopen**;
6. the settings (Ignore built bars, Visual diagnosis, Insert split mm, Spiral, Spiral s, Spiral turns, Servo by bar,
   Retreat live axis, Retreat compliant, Dry run);
7. the run (its chips, Continue newest / Continue match / New run, where each robot is);
8. the folded **Advanced planning**.

The **Plan plots** and **Exec plots** windows float at the bottom left.

The action list shows each action's mark from the run (`· pending`; `· done`, green; `· done (assumed)`, orange; the
selected action blue), and the movement you pick there (a row, the step slider, ◀ mv / mv ▶, a movement button)
becomes the run's current step, except while `movement_exec` has a robot's lease for a step.

Buttons are coloured by what they do: teal computes without moving (Live IK, Plan), blue starts a motion (Exec; while
other robots are not measured it turns amber, reads **Exec?** and asks for a second press within 5 s), green lets a waiting step go on
(Confirm), red stops (Cancel), yellow records (Mark done).

## Phone

A phone page for walking next to the robots: open `http://<laptop's 192.168.0.x address>:8081` on a phone on the robot
WiFi, held upright. It has **Stop all**, the health banner, and a tab per plugin: **Control** (D-pad, gripper
Open/Close/Stop, Free drive while held), **Health**, **Mocap** and **Exec** (Confirm and Cancel for `movement_exec`).
Setup, every button and troubleshooting: [phone_manual.md](phone_manual.md). Plugins add a tab with `ctx.view.phone()`
(`plugins/__init__.py`).

## Clearance

Every button that moves a robot from the monitor keeps it clear of everything in the scene while it moves: the other
robots, the obstacles, the bars and the robot itself. Once the robot comes closer than the move's **clearance**, the
monitor soft stops it (Resume in Health) and the button's chip says what it came too close to; the **Collisions** tab
shows the hit. It is checked a tick or two behind the robot, from mocap and the joint states: it is no emergency stop,
keep the e-stop at hand.

How close it may come depends on the kind of move. Planned moves keep more than that from everything, so a robot that
follows its plan a little off does not trip it:

| Move | Its plan keeps | It soft stops within |
|---|---|---|
| An arm along a plan: the arm planner's **Commit**, calibration's **Go** (moves and sweeps) | 1 cm | 5 mm |
| An arm by hand: robot_control's joint and tool0 jogs, calibration's tool pad | (no plan) | 5 mm |
| A base along a plan: the base planner's **Commit**, base_exp's **Start** | 10 cm (the **Margin**) | 5 cm |
| A base by hand: every D-pad (robot_control, calibration, phone) | (no plan) | 5 cm |
| Assembly **Exec**, a plain motion along its plan | contact only | contact |

- **A plan that keeps less.** An arm plan whose start or target is closer than 1 cm keeps only as much as they have
  (the plan's line says "keeps only 0.6 cm"), and a base plan keeps the **Margin** you set, which you may lower below
  10 cm to get through a tight spot. Then the clearance is half of what the plan keeps (0.3 cm, or 4 cm for an 8 cm
  Margin), and an amber **small clearance** chip says so before you commit (calibration says it in Go's prompt). The
  assembly planner checks contact only,
  so Exec's clearance trips on contact only, with the same chip.
- **A start already too close is refused.** Nothing is sent and a red **starts too close** chip (robot_control greys
  its buttons) says what is in the way. Before the press, the chip already shows while the robot stands that close.
- **The Clearance box**, below each set of buttons, ticked at start. Untick it to move out of a tight spot: the buttons
  turn amber and nothing soft stops the robot. One box covers a robot's buttons on the laptop and the phone alike.
  base_exp has none: its runs always keep their clearance.
- **Other robots not measured** (mocap lost, no joint states) are checked as stand-ins where they were last seen. An
  arm's Commit or Go takes a robot whose only unknown parts are arms out of the moving arm's reach without asking; any
  other makes the button ask for a second press. A base drive always asks (its Commit and Start): driving changes the
  distance to every robot, however far. The D-pads and jogs never ask; the amber chip naming the robots not measured
  is the warning.
- **No clearance**: free drive (the arm is in your hand), Exec with **Ignore built bars** ticked (the plan may pass
  through them; an amber **no clearance** chip says so), and Exec's inserts, retreats, Robotiq handoffs and servo
  passes, which touch the bars by design.

Stopping in time at speed is not covered: the base rolls about 10 cm after a trip at 0.2 m/s. Drive slowly near
things. For plugin authors: `ctx.clearance` and the kinds of move (`ARM_MOVE`, `BASE_DRIVE`) in
`world/clearance.py`, the box in `shared/teleop/clearance_toggle.py`.

## Examples

```shell
# Drive one robot
ros2 launch husky_assembly_teleop monitor.launch.py robots:="alice" plugins:="robot_control"

# Assembly: a process, the lab obstacles and the base planner
ros2 launch husky_assembly_teleop monitor.launch.py robots:="alice cindy" \
    plugins:="robot_control process obstacles base_planner_panel" \
    process_directory:=data_design_study/260814_RobArch_support_ik

# Plan arm motions too
ros2 launch husky_assembly_teleop monitor.launch.py robots:="cindy" plugins:="robot_control obstacles arm_planner_panel"

# Try the plugin examples
ros2 launch husky_assembly_teleop monitor.launch.py robots:="alice" \
    plugins:="example_plot example_pybullet example_sequence example_ui"
```

More in the README, "Examples".

## Plugins

Load plugins with `plugins:="<name> <name>"` (README, Parameters). A plugin that needs others loads them itself
("Loads ..." in its section). `health` and `console` are loaded by default.

### `health`

Loaded by default. Shows the status of each robot and tracked object, with Unlock, Resume and Reconnect. **Parked**, after the robot's buttons and filled while on (a click toggles it), marks a robot standing still out of the way: the snapshot takes it as known where it stands (last seen, else its default pose; `stand_ins.parked`), so no Commit, Exec or Go asks about it and no planner, clearance or chip warns about it while it is not measured; it stays an obstacle there. It is unparked, with a warning, once seen moving (its base measured 2 cm or 2° from where it was parked, or an arm joint 1°) and when a plugin leases it to move it. Not kept across a restart. A chip's colour change is logged once it has held for 1.5 s, one line per row, so a flickering chip makes one line. A signal chip shows a worse level at once and a better one only after it held 1.5 s, so marginal wifi reads steadily amber.

### `console`

Loaded by default. A window at the bottom right with the newest log lines, newest first; it starts minimized to its bar, the control at the bar's right end opens it. **Show**: **Plugins** (what the plugins say, without health's colour changes and the late-tick reports), **Warnings** (warnings from every source but the ticks), **All**; errors always show. **Source** narrows to one plugin, `health/chips` (health's colour changes), `runtime`, `robots` or `ticks`; **Hide** is a dropdown like Source in which several sources can be ticked: picking a source hides it (☑), picking it again shows it; closed, it says what it hides. They stay out whatever Show says, except their errors, and a name without a `/` also hides its topics. It starts hiding nothing; a layout file may name sources to hide at start (`plugins.console.preferences.hidden`). Messages wrap; a tick report or traceback shows in full on hover. A line under its chips shows the share of the last minute's tick periods that were late, a freeze counting for every period it lasted, and how many lines Show and Hide leave out. Pause the page (**Live**) to read; stdout and the ROS log still get every line.

### `robot_control`

The **Control** tab, with a tab per robot: drives the base, jogs each arm joint by joint while a button is held, or its tool0 along or about one axis of the **Frame** chosen (Robot: x forward, y left, z up, whatever the flange's turn; Tool: tool0's own) (the **Tool0** buttons, at the **Tool speed** in mm/s; one arm at a time, a point streamed every tick 0.2 s ahead on a line from where the button was pressed), moves it to a Cartesian target (sliders or gizmo, in the husky's base_link) with a compliance force, holds it, operates the tools. An arm's **Joint arrows** box (below the joint jog) draws in the 3D view, while the joint controller runs, two curved arrows per joint around its name, red the way its + button turns it and green the way − does, each marked + or −; its **Tool0 arrows** box (below the tool0 jog) draws at tool0, per axis of the chosen **Frame** in its colour, a straight arrow the way + moves it and a curved one the way + turns it, long enough to reach out of the tool. To move an arm to a pose, plan it in `arm_planner_panel`. Free drive is the arm's **Free** controller: the arm can be pushed by hand only while its **Free drive (hold)** button is held or **Keep on** is ticked. Each arm's **Live plots** folder (closed at start) shows its raw force and torque over the last 30 s.

### `obstacles`

The lab's obstacles as collision bodies for the planners: tables, cabinets, tripods and trusses, and walls around the usable floor, from the lab file `data/lab/<lab>.json` (in git; `lab:=<name>`); and the floor, a slab under z = 0 out to the walls, not drawn, which arms, tools and what they hold collide with while the robots' platforms (wheels, chassis) may touch it. Click an obstacle in the 3D view or pick it, then drag it with the gizmo (only yaw is kept), resize it with the corner gizmo, or type position, yaw and size; rename and remove it, and set the border's x and y range. Edits stay unsaved (amber chip) until **Save** writes the lab file; **Revert** goes back to it. **Add** puts temporary boxes and cylinders (purple) into the scene, kept in `~/husky_obstacles_temporary.json` across restarts and never in the lab file; **Promote** moves one into the lab.

### `debug_gizmo`

Debug only. Click any scene body with a world pose in the 3D view, or pick it from the dropdown, then drag it with a gizmo or type its position and roll/pitch/yaw; **Log** prints its pose, **Reset** puts it back. Nothing is saved; to change the lab's obstacles for good, use `obstacles`.

### `mocap_probe`

Tracks the probe and records points with it, e.g. obstacle corners; exports JSON and loads it back. Shows its state in a separate panel, see [Panels](#panels). In the closed **Debug** folder: **Map coverage** keeps the probe's position and marker error every tick while you walk, and draws a map smoothed from them on the floor (15 × 15 m, centred on the mocap origin) by the mean marker error: green, deeper the lower, up to the mocap chip's 1 mm; amber → deep orange up to 2 mm; red above; greyer where the probe was often lost, grey where never tracked, nothing where you did not walk. Export and load the samples as JSON. **Floor calibration**: floor points, the marks and Motive's anchors place the lab frame on the floor, exported for the relay to apply, on the laptop and the phone: [floor_calibration_manual.md](floor_calibration_manual.md). **Show cameras** (also in **Debug**): Motive's cameras where its calibration puts them, each a wireframe of its view with its name, its view's edges as faint rays down to a plane at a chosen height and a cross where its aim meets it, from the mocap relay; **Views on the floor** colours the map's area by how many cameras see each point of that plane, each region outlined with its count (grey none, red one, to green for six or more; range and occlusion not modelled).

### `mocap_bars`

Tracks every bar of the loaded process that carries markers as its Motive asset (bar Bxx streams as asset 10xx; **Mocap ID** + **Track** re-track the chosen bar under another id) and fits each bar's CAD markers (from the markers on its connector halves) to the measured ones with the bar fit of `shared/bars/layouts.py` (the matcher in `world/marker_match.py`): the markers matched by their pairwise distances without Motive's labels, around the last fit or the planned pose, ten times a second at most and one full search per tick across the bars, markers with a residual or size out of range left out. **Ghost bars** draws every fitted bar see-through (the old monitor's MoCap cyan) at its fitted pose; **Show markers** the chosen bar's measured, fitted and planned markers; the stats block below the buttons reads the chosen bar's fit the old monitor's way (each marker's Motive and fit residual, each plate's ball pair and roll, the spacing between plates, the bow, the deviation from the planned pose), and **Measure + log** / **Log live** print the old `[mocap bar]` report lines; **Record take** saves the chosen bar's fit to `data_experiment/mocap_bars/<experiment>/` on the Drive. Offers every bar's latest fit and a fresh fit on demand (`MocapBarsContract`, for the as-built record). The servo part steers on its own fit of the held bar with **Servo by bar**. Loads `process`. Experimental.

### `process`

At the top of the **Assembly** tab. Loads the process picked in **Folder** (the processes and old exports two folders below the Drive root, newest first, and the `process_directory` parameter's) and shows the state at one movement at a time, its planned robots in the goal blue, see-through at start (View **Ghost** toggles it); the planners load their targets from its selected step, and the run (`assembly_run`) follows it: the movement you pick with the step slider, ◀ mv / mv ▶ or an action row becomes the run's current step, except while `movement_exec` has a robot's lease for a step. Each action row shows its mark from the run (pending, done in green, done (assumed) in orange). After **Live IK** or a plan, the action's later movements are drawn with the joints found, on the live base they were found at (chip `joints from live IK` or `joints from plan`), and the panel turns to the target of the movement they are for; plans, runs and records never read them. Each movement shows its name in the old monitor (e.g. `J_M3_CDFM_transfer_to_approach`), a sentence on what it does, and its action's movements as a row of buttons, coloured by kind as in the Rhino repo's `docs/action_flowchart_2026.drawio` (FM, LM, CDFM, CDLM, Manual, TM), the selected one ringed: click one to select it; hovering another shows its old name and what it does. **Transfer workaround** is on at start, for this session only (runs don't keep it): Cindy's transfer runs as a free move with empty tools to the insert start, where you place the bar; while it is ticked, each jointing action is listed and drawn as it runs (`J_transit`, place, grasp, insert; the travel to load left out), and assembly_run, assembly_steps and movement_exec follow it; replan after a change. **Rhino obstacles collide** is off at start: the process's own obstacles from Rhino (e.g. the cabinet) are drawn see-through and left out of every collision check (base and arm planners, clearances, assembly planning); tick it to count them, load `obstacles` for the measured room, and replan after a change. **Rhino bar lines** (off at start) draws every bar's centre line and name, pink, where the design puts it once built: a ruler of the Rhino poses.

### `base_planner_panel`

In **Plan › Base**: plans a collision-free path for a base around the other robots and scene bodies, keeping the **Margin** (m, 10 cm at start) from everything (Commit's clearance soft stops the base within 5 cm, or half the Margin below 10 cm, with an amber "small clearance" chip; it refuses a start already that close, and the "starts too close" chip warns before the press; **Clearance** below Commit, unticked, drives without it; with another robot not measured Commit asks twice: [Clearance](#clearance)), with the algorithm chosen under Advanced (each with its settings, and a Debug view that draws how the search worked); Commit sends it to the onboard pure pursuit follower (`crl_husky pure_pursuit`, or a simulation in your own ROS domain, never 80: [simulation_manual.md](simulation_manual.md)), which drives it at once if the velocity controller is active. A blue ghost stands at the target (hidden once the robot is there) and a green one rides the planned path. The floor shows the path as the follower cuts it and the robot's trail; with Record ticked each run is saved to `data_experiment/base_planner/` on the Drive (`drive_root`, create the folder in Drive first). Loads `base_planner`, the headless planner it drives (other plugins can request and commit plans through it too), and `process`. Experimental.

### `arm_planner_panel`

In **Plan › Arm**: plans a collision-free arm motion to a joint target (the arm now, its stow pose, the process step, or sliders) and sends it as one joint trajectory that runs through rounded corners and rests only where a corner cannot be rounded clear of collisions; the preview shows the same timing, and **Slow down** (×1 to ×5) stretches it without replanning, for the next plans too. A path keeps 1 cm from everything, more than the 5 mm at which Commit's clearance soft stops it; when the arm starts or ends closer, the path keeps as much as they have, the plan's line says so in amber, and the clearance is half of that (an amber "small clearance" chip). **Clearance** below Commit, ticked, runs the path under that clearance and refuses a start already that close (a red "starts too close" chip says so before the press); unticked, nothing soft stops it (the chip turns amber); [Clearance](#clearance). Commit does not ask twice for another robot whose only unknown parts are arms beyond the planned arm's reach (about 2.5 m between shoulders, `world/arm_reach.py`). The panel for the headless `arm_planner` plugin (loaded with it), which others can use through `ArmPlannerContract`. Experimental.

### `collisions`

In its own **Collisions** tab: lists what collision checks hit, per source (calibration takes, a failed arm or base plan, the motion planner's last answer, base_exp's blocked way back), and draws the ticked hits as they were checked: what moved solid red, what it moved into see-through cyan, the deepest contact with its depth; Focus adds every robot as checked, faint; each drawing goes after 2 minutes, its hit unticked but still listed. "As checked" shows meshes as the convex hulls compas_fab tests. Clearance trips of any plugin (a driven robot stopped for coming within its margin of something) come under **clearance**, the newest ticked, with the gap as "x mm apart". Loaded by calibration, the arm and base planner panels, motion_planner and base_exp; load it by name to see clearance trips with other plugins. It only draws: it sends nothing to a robot.

### `calibration`

Calibrates a robot's base rigid body (`base_mocap_from_base_footprint`, what the mocap relay applies) by sweeping one arm's first two joints, guided step by step: Setup, Marker error (drive with the D-pad; refine in Motive if too large), Sweeps (collision-checked by the arm planner; the colliding takes are listed in the **Collisions** tab), Record (each take planned to its start, then sampled once still), Solve (the old pipeline's math), Validate, Apply (reloads the relay), then optionally Tip offset and Punch, and an Overview of the result and its figures; the session goes to `data_experiment/calibration_data/<date>/` on the Drive, where the relay finds it. Next only once the step's checks pass. Progress is saved after every change: after a crash or restart, choosing the arm offers to resume its unfinished run. The takes are predefined, taken from the old monitor (`data/calibration/sweeps.yaml`); a simulated robot's sessions stay in `<Drive>/data_experiment/sim/calibration_data/domain_<ROS_DOMAIN_ID>/<YYYYMMDD>/`, where the relay finds them for a fake Motive's bodies ([simulation_manual.md](simulation_manual.md)). `scripts/solve_calibration.py` solves a session again offline, and with `--old-scripts` compares it with the old pipeline's scripts run on it. Loads `arm_planner` and `collisions`. Experimental, not yet tried on a robot. How to run: [calibration_manual.md](calibration_manual.md).

### `assembly_run`

In the **Assembly** tab: remembers how far the assembly of the loaded process got, in an assembly run, kept in `runs/<YYYYMMDD_HHMMSS>/` inside the process folder (not an experiment's run folder on the Drive): which actions are done, where each robot was left, the run's settings (insert split, spiral, servo by bar, the spiral's seconds and turns, and Advanced planning's free-search settings; the transfer workaround is `process`'s) and an event log. On Load it continues the newest run when it was made on these very process files; otherwise the operator chooses: **Continue newest** (the newest run, made on another export, goes on in a new run), **Continue match** (an older run made on these files) or **New run**. Nothing is written until the runs were read (a failed or stopped read is tried again). Then it selects the first pending action's first step in `process` once its robot is live; after that it follows what you pick in `process`. **Mark done** records every live robot where it is; for a robot not live, a second press records the export's end as assumed (it waits with no time limit; any other press forgets it) (meanwhile the button is amber, its label ending in **?**). **Reopen** puts an action back to pending. **New run** too needs a second press (**New run?**). Each action's mark shows in `process`'s action rows. Loads `process`. Experimental. Its writer thread also appends the as-built records to the Drive.

### `motion_planner`

In **Plan › Motion**: plans process movements, solves their goals by IK and checks collisions for other plugins, one request at a time on one worker thread with a compas_fab world per robot (the first request for a robot loads its models, up to a minute); **Cancel** stops them; optional PyBullet window. What the last answer collided with (a start, a goal, a swept segment, a linear step) shows in the collisions panel. While a free search runs (tamp's dual-arm birrt: a transit, the transfer workaround, the way home; or a support robot's one-arm birrt: its approach and leave), **Search tree** (on) draws it every 2 s at the moving arms' tool0s: the tree grown from the start (blue) and from the goal (purple), a yellow line labelled with how far apart in joint space their closest nodes are, and, for tamp's search, the configurations checked since the iteration began as dots (red where they collided); the panel, and the Assembly tab after "planning …", say the try, birrt's run and iteration, that gap and the nodes, and each run's end is logged. **Search tree legend**, a folder at the bottom of the panel, explains each mark and term. A found path takes the drawing away; after a failure the trees stay until the next search. birrt draws only as many nodes of the bigger tree as the smaller one has, so its newest nodes show later. Recording costs about 0.1 % of tamp's search and 1 % of a one-arm one (a fast search records an iteration every 0.5 s). Experimental.

### `assembly_steps`

The operator's steps, in the **Assembly** tab: the run's current step (its action's movements grouped as the operator walks them, with `process`'s transfer workaround; move between steps in `process`), **Live IK** (where the next movement starts, by IK at the live base: the insert start after a transfer, the linear move's start after an approach; logged, recorded, carried into `process`'s drawing (its blue robot moves to the live base with the joints found), and reused by the next plan while the base stays within 2 mm / 0.5°), **Plan** from the live robot through `motion_planner` (only when it is live; on a movement with nothing to plan, such as an ungrasp, a grasp or an operator step, it says so and selects the next robot movement: click Plan again), **Play** the preview timed as it will run (slider **Preview time**, opening at the motion's end), **Exec** through `movement_exec`, the **Motion time** (1–90 s, at least what the arms' limits allow), Ignore built bars (for tests in the air), **Advanced planning** (folded, saved with the run like the other settings: **Max time** per try of a free search, s; **Max iterations**; **Joint resolution** of its collision checks, rad; **Tries**; **Swept check** of the found path on or off; the same values govern a transfer, the way home and the support moves; **CDFM planner** (birrt, or prm: a precomputed roadmap of this process and bar, one try, about 1 to 2 min, no time limit; used with the Transfer workaround off); **Roadmap root** (the folder holding the `prm/<process>__<bar>/` roadmaps; empty: the Drive's `data_experiment/cdfm_planner_benchmark`; nothing is written there)), and the **Plan plots** window at the bottom left, minimized to its bar until a plan arrives, when it opens by itself (close it with the control at the bar's right end): every joint along the preview (deg) and the largest joint step between the plan's rows (deg); the old monitor's bar-hold drift plot comes later, with the held moves. **Check tool0** draws each flange's live tool0 (mocap base + FK, red) against the step's authored target (blue) with a line between them, says the gap per side (mm, and degrees per axis) and records it (`check_tool0`); nothing moves. A retreat is planned to the live tool0 poses backed out along their own -z, as the encoders read them (the run's **Retreat live axis**, on by default; off: the export's axis), logged as `[retreat live]` with each tool's angle from the Rhino axis (a warning over 20°) and recorded as `goal_live`; when the full line is blocked it plans a shorter one, 5 mm at a time down to 25 mm, with a warning. **Visual diagnosis** (settings, off at start, this session only) draws the real robots see-through and, after each insert or retreat plan, its tool0 path in green with the Rhino (blue) and planned (red) start and end frames and the gaps between them, logged as `[visual diagnosis]`. A transfer, an approach or a ground bar's insert ends with its servo (the step line says "then servos … onto the insert start"), which Exec runs alone once the step's other parts are done. An insert after its transfer's mocap-bar servo is planned from where the servo left the flanges (logged as `[insert plan] MoCap-bar correction carried`, recorded as `goal_carried`). After the action's last step it says to Mark done (assembly_run's button). A plan goes stale when the process, the step, the transfer workaround or the run's statuses, the acting arm (2°) or base (2 mm / 0.5°), or another robot (1 cm / 1°) changes, and is used by one Exec only. Loads `process`, `assembly_run`, `motion_planner` and `movement_exec`. Experimental.

### `movement_exec`

In the **Assembly** tab: runs one step on the real robot with the old monitor's Confirms: **Exec** sends a motion along the plan at once (timed for the arms, its time and largest joint travel logged, rebuilt from the live joints and sent right after the last check; with other robots not measured, **Exec** asks for a second press, as **Commit** does). **Clearance** below Exec, ticked, runs such a motion under a clearance that soft stops the robot once it touches anything and refuses a start in contact (contact only: the assembly planner checks no margin, which an amber "small clearance" chip says); none with **Ignore built bars** ticked (amber "no clearance" chip), nor for the inserts, retreats, handoffs and servo passes, which touch by design ([Clearance](#clearance)); a Robotiq open (0.426) or stiff close (0.8), a step by hand (the bar placed and clamped with the tools' own buttons: one prompt for both) and the servo's first pass wait for **Confirm** (there or on the phone's **Exec** tab), the prompt naming the robot; and the parts that run partly compliant (`shared/execution/`), sent at once too: the INSERT (rigid under joint control up to the split, then the last millimetres under the compliance controller along a straight line or a 15 mm spiral while the mating joint screw tightens, until it stalls or 30 s pass; a bar with one mating joint runs the straight line whatever Spiral says (the box is greyed then), and after its stall the other arm finishes its line under compliance, settles against the force (≤ 3 s) and switches back; a ground bar is always rigid with no tighten, every other bar always rigid then compliant: no setting changes the mode), the RETREAT (the old monitor's stiff screw-out: under the joint controller throughout, both jaws LOOSEN while the tools back straight out 12 mm at the screw rate, 2.4 mm/s, then the rest of the planned line rigid, the tools STOPped at the end; the path checks logged PASS/FAIL before anything is sent, and the arms over 0.57° from the plan's start refused; with the run's **Retreat compliant**, for tests only, the gripper screws open, the force sensors are zeroed, a compliant break-out of the first millimetres, back to joint control, then the rest replanned from where the tools are), and the Robotiq HANDOFF (the arm goes compliant before the close, giving way up to 10 mm; a stiff close when its frames disagree). The run's settings (`assembly_run`'s) shape them: Insert split mm (0–30; a compliant retreat's break-out too, from its start), Spiral, Spiral s (1–60) and Spiral turns (1–10); a spiral over 50 mm/s is refused at Plan. After a transfer, a support approach or a ground bar's insert the step ends with the servo (`servo_part.py`): pass by pass, each planned on `motion_planner` from the live arms (IK at the live base; a straight bar-held line for two arms, two rows for one), the first pass after Confirm (plain) and any later one travelling over 5° after a warning's Confirm, up to 8 passes, until both flanges are within 0.2 mm of the authored targets (tool0) or, with the run's **Servo by bar** (on by default; it steers only a ground bar held in the tools, every other bar servos on tool0) and the bar's Motive asset streaming, until the bar's mocap fit is within 0.5 mm / 0.1° of its design pose; a transfer's bar correction is carried into the insert's plan; after a ground bar's insert straight lines only, stopped when a flange's force changes by over 20 N. One panel line shows each pass's errors, and each servo is recorded as `runs/<run>/servo/<action>/<movement>.<n>.json` with a .png of its readings. Refuses another robot's or another movement's plan, a stale plan, a plan made before a servo moved the arms, a robot not live or soft stopped, an arm the safety sync (`multi_arm_safety_sync`) does not vouch for (no recent status, not operational, or its program stopped: press Resume), a compliant part whose arm is not under the joint controller or whose measured TCP is over 10 mm / 2° from the URDF's tool0; refused too while another plugin leases any robot (a teleop drive, a committed plan, a base experiment). Leases every robot while it works: no other plugin gets a lease, so nothing else here moves any robot (one motion at a time), and before sending and while its arms move it checks that nothing else moves (another robot's arms or base, or one whose states went stale); Hold in `robot_control` stops its motion short of the end. **Cancel** (or Stop all) holds every arm; in a compliant part it also STOPs the tools, holds each arm at its TCP and switches it back to the joint controller. A hold an arm refuses (stale joint states) is said loudly: HOLD NOT SENT, stop it with the pendant or the e-stop; a switch back not confirmed within 10 s is said loudly too: ARMS MAY BE LEFT under the compliance controller, switch them in `robot_control` before anything else. The teach pendant's speed slider must be at 100 % (slower, a motion times out and is held mid-path). **Dry run**: every check, nothing sent. Every part is recorded in the assembly run (`runs/<run>/movements/`; each record keeps the ROS time and the robot's base at the part's start and end; a compliant part's record keeps its plan and settings, phases, switches, stall times, peak force, end errors, the retreat's checks and tail, and one sample per tick). The **Exec plots** window at the bottom left, minimized to its bar until a servo or a compliant part starts, when it opens by itself with that kind's plots (close it with the control at the bar's right end), shows the running servo pass by pass (tool0 position and rotation errors per side, the base's drift, the mocap bar's deviation, the force change; a chip says which phase the pass is in, and under each plot stand the last pass's numbers) or a compliant part's force and torque per arm each tick; the curves stay until the next part of that kind. Loads `motion_planner` and `assembly_run`. Experimental: the first plugin that runs process movements on the robots, e-stops in hand. Around every part (the start of a request's first, the end of each) it measures every MoCap bar through `mocap_bars` and appends an as-built record to the day's file on the Drive (`shared/runs/as_built.py`; **As-built: record now** any time); **Build as-built page** builds the day's 3D page from the records so far (`scripts/as_built_page.py` in a subprocess) and serves it on port 8082, the link under the buttons.

### `base_exp`

Drives test paths (straight, turn, arc, sine, drive-turn-drive; geometric or timed) from where the base stands with the onboard pure pursuit follower, with live error plots; the monitor measures every run's errors itself, the same for every controller, and keeps recording until the robot settles; each run gets its own folder under `data_experiment/base_exp/<experiment name>/` on the Drive (`drive_root`) with its setup (sim or real, sim model, controller and parameters) and outcome (experiment.json), the recording (.npz), and once reported its numbers (report.json) and an overview plot (.png). **Start auto** runs paths at one fixed speed, the standard paths (the same 10 in every scenario, resumed per experiment and setup) or random paths from a grid of distances and angles, or constant commands open loop over a grid of speeds and turn rates; each run is checked to keep 0.3 m from every robot and scene body (`obstacles` loads with it, for the lab border); during any run the robot is soft-stopped and automation paused if it comes within 5 cm of anything, and a run starting that close is refused, nothing sent (always on, no box); with the robot's arms or another robot not measured, Start and Start auto ask for a second press ([Clearance](#clearance)). Experimental; needs `crl_husky pure_pursuit` on the robot, or a simulation ([simulation_manual.md](simulation_manual.md)). Analyse the runs with `scripts/analyze_base_exp.py`: `summary.md` compares the controllers, `robot_model.md` fits a model of the robot, `constant_commands.md` shows how it changes with speed and turn rate. How to run: [base_exp_manual.md](../experiments/base_exp_manual.md).

### `insertion_demo`

The **Insertion demo** tab: one bar into its joint and out again, as often as you click, on a robot with two scaffolding tools (Cindy) that stands still, for a demo hall without mocap ([insertion_demo_manual.md](insertion_demo_manual.md), its dry run on a simulated Cindy [insertion_demo_sim_manual.md](insertion_demo_sim_manual.md); `husky_fixed_base` publishes her base at the origin). **Action** picks a jointing action of the loaded process with a transfer and an insert (the last one at start). **Grasp** / **Release** close or open both gripper screws until they stall; **Tighten** / **Loosen** run both joint screws by hand, until they stall or in reverse for **Loosen s**, and **Stop** halts both tools' motors at once, whatever runs them. A Move button only plans its move and shows it, as the Assembly tab's Plan does: a see-through robot on the path at **Preview time** (drag it, or **Play**), timed for **Motion time** (the line below says the arms' minimum); **Execute** then runs the move shown, once, refused when the arms left the plan's start. The Assembly tab's two plot windows are here too and open by themselves (`shared/assembly_tab/plan_plots.py`, `exec_plots.py`): **Plan plots** for a planned move (its joints and, for a move that holds the bar, how far one tool0 seen from the other moves from where it is at the first waypoint, in position and per axis: `parts.grasp_drift`, at every waypoint and halfway between), **Exec plots** with both arms' force and torque while an insert, a retreat or Compliant runs. The first Move button reads **Transit** and plans on `motion_planner` a free dual-arm move from wherever the arms are to the insert's start (the transfer workaround: the held bar hidden). **Load** plans a free move to the loading pose, the tools empty (the process's travel to load, to the saved carry's first row). With the Assembly tab's **Transfer workaround** box unticked, once and for the whole session (chip `bar held`), the bar is carried instead: the button reads **Transfer** and shows the carry saved in the process's `solutions/<action>.json` (`parts.saved_carry`; planned offline with the bar held by both arms, from the loading pose to the insert's start, by tamp's `scripts/plan_process_carry.py --goal joints --write-solution`), checked on the monitor's robot model (`parts.return_problem`) and only shown, never run, while the arms stand over 2° from its first row; Load then shows the same path reversed while the arms stand at the insert's start, and elsewhere plans the free move, only after a Release that ran to its end (the tools empty; a gripper screw closing takes that back), else it refuses; on every move that holds the bar the preview robot carries the bar and its joint halves in its tools (`parts.held_bodies`, `RobotGhost.hold`); Cycle then starts and ends at the loading pose (the carry, the cycle, the carry back) and Preview cycle shows that loop. A carry that is missing, stale (the process or the action changed since, also by Save) or does not end at the insert's start for Insert mm is refused with the reason. **Insert** plans the process's insert from the live joints and runs it as `movement_exec` does (`shared/execution/insert.py`: rigid up to the last **Compliant mm**, then compliant along the line while the joint screws tighten until they stall), refused until both tool0s stand within 5 mm / 3° of the insert's start. **Retreat** keeps the bar in the tools: force sensors zeroed, both arms compliant, the joint screws run in reverse for **Loosen s**, the motors stop, the tools back out along the insert line to the insert's start, then joint control. **Return** plans the straight way back to the insert's start with the insert's own planner (`tamp.plan_constrained_dual_arm_linear`, the bar held, the start's tool0 poses as the goal), checked before it is shown: the arms' relative pose within 2 mm / 1° along the path, no joint step over 5° between waypoints, each tool0 ending within 5 mm / 3° of the start (`parts.return_problem`); a path that fails is shown with the chip `return refused`, and neither Execute nor the cycle runs it. **Insert mm** and **Retreat mm** set how far before its end the insert starts and how far the retreat backs out, on the insert's line (`parts.line_start`, solved on the URDF from the authored start); they act at once, and **Save** (second press) writes the insert's start for Insert mm into the process's `actions/<action>.json` (`parts.with_insert_start`, `save_action`). **Preview cycle** plans the cycle's moves without sending anything and shows them as one path. **Compliant** zeroes the force sensors and puts both arms under the compliance controller until **Rigid**, which also brings back arms a cancelled part left compliant; **Zero F/T** zeroes both force sensors where the arms stand, for arms holding no bar. **Stow** plans, with the transit's planner, a free dual-arm move to the arms' park pose (`ArmConfig.stow_joints`), for the bar put away. **Cycle** (under **autonomous**) runs one whole cycle on one click from the insert's start: Insert, a pause of **Settle s** (0 at start) after the stall, Retreat, then Return planned, checked and run; it stops at the first step that does not end well and never resumes after Cancel (Rigid, Retreat and Return by hand, then Cycle again). One step at a time, the robot leased while a move runs; **Cancel** or Stop all holds, and **Cancel** with nothing running drops the planned move. Loads `process` (press Load in the Assembly tab) and `motion_planner`. Experimental.

### `example_plot`, `example_ui`, `example_sequence`, `example_pybullet`, `example_recording`, `example_robot_state`, `example_lease_and_freeze`

Minimal plugins, as templates for new ones.
