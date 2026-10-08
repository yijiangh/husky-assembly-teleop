# Why the monitor was rewritten the way it was

Background for the `husky_assembly_teleop` core (`monitor.py`, `config.py`,
`plugin_api/`, `world/`, `robot_interface/`, `ui/`).

This file holds the *archaeology*: what the old code did, and which specific
failure each design decision is defending against. It lives here rather than in
the module docstrings because it justifies choices against code that is being
deleted, and it goes stale the moment `husky_assembly_teleop/old/` goes away.
The modules themselves keep only the rules a reader has to follow today.

## The god object

`old/husky_monitor.py` was 7085 lines in one class. There was nowhere else for a
new feature to go, so every feature went there.

Concretely:

- ~25 class-level booleans gated the features: `CALIBRATION`,
  `BAR_ACTION_LIVE_REPLAN_EXE`, `BAR_ACTION_MOCAP_ACCURACY_TEST`,
  `DUAL_ARM_KISSING_REP_EXPERIMENT`, `PUNCH_CALIB_VALIDATION`, and so on. Each
  switched on a block of a 575-line `build_ui` method, plus a scattered set of
  methods and attributes on the same class.
- Roughly 60% of the monitor's attributes were neither measured nor planned
  state: trajectories, preview ghosts, saved body colours, collision-diagnosis
  handles, plot histories, a base plan in progress.
- ~90 free functions took the whole `HuskyMonitor` as an untyped first argument.
  The import checker was happy, but every feature could reach every other
  feature.

**What the rewrite does about it.** One plugin per old flag, each owning its own
derived state on its own instance. Splitting files without giving that state a
home just moves the problem, which is why `HuskyPlugin` instances -- not the
monitor -- hold it. `PluginContext` is constructed per plugin, so a plugin can
only reach another plugin through a declared `requires` entry.

## Measured versus planned state

The old code let state flow backwards, repeatedly:

- a UI slider wrote into `interface.position`
- a goal pose wrote into `hi.position`
- a mocked replan wrote into `hi.arm_joint_pose`

Once the planner can write into the measurement, a reading no longer means
anything, and no amount of reading the code tells you which is which.

The old `TrackedObject` mixed the measured pos/rot with `self.body`, a PyBullet
id, which is what made "real" and "simulated" impossible to pull apart later.

**What the rewrite does about it.** `WorldState` is a registry, not a second
copy: per-robot measurements live in `RobotState`, owned by that robot's
`HuskyRobotInterface`. Nothing outside a ROS callback writes there. Geometry for
a tracked object belongs to whichever plugin put it in the scene; the link
between measurement and geometry is the name.

## Shared mutable class attributes

`old`'s `HuskyRobotInterface` declared its state at class scope
(`arm_joint_pose = [UR5e_HOME_STATE]`, `io_states = [[False] * 18]`, ...) and
then mutated it with `self.arm_joint_pose.append(...)`. With exactly one robot
that behaves. With two, both robots share one list and the lists grow on every
construction. Since the whole point of `WorldState` is to hold N robots, that
pattern could not come across -- hence `RobotState` as a dataclass where every
field is per-instance.

Registration had the same shape of problem: a Husky registered itself by calling
`monitor.add_husky(self)` from its own constructor, so constructing an object
mutated a global scene as a side effect and the two could never be separated for
a test. `WorldState.add_robot` is now explicit.

## Configuration as module globals

The old package kept configuration as module-level constants in `__init__.py`:
`DATA_DIRECTORY`, `DESIGN_DATA_DIRECTORY` (hardcoded to a path under `/home/su`),
`CALIBRATION_DATE`, `DESIGN_PROBLEM_NAME`, `DEFAULT_ENV_3DM`. Module globals are
convenient right up to the point where you want to run two configurations, or
run a test, or run on a second machine. Then they are immovable, because every
import site has already baked them in.

**What the rewrite does about it.** One frozen `MonitorConfig`, built once and
passed down. Everything about configuring a run lives in `config.py`: the shape
of the configuration, the table saying which URDF each robot runs, and the
factory that reads ROS parameters. Nothing imports a global; everything receives
a config.

## Robot configuration inferred from strings

`dual_arm` was inferred from a string compare (`robot_name == '0806'`) and then
leaked into ROS topic names and into a hardcoded -180/-90 degree TCP correction.
Every new end effector meant another branch. Joint names came from
per-configuration constant tables (`HUSKY_UR5e_JOINT_NAMES` versus
`HUSKY_DUAL_UR5e_JOINT_NAMES`), selected by that same boolean.

**What the rewrite does about it.** The kinematic configuration comes from the
URDF and only from the URDF. Which tool is mounted is configuration (the
`tools` parameter), and the tool's own URDF (`data/tool_urdf/`) is stitched onto
the arm's `tool0` at load time (`tool_urdfs.py`). One combined URDF is written
and everything reads that one, so the robot and its tools cannot disagree.

Stitching rather than one full URDF per robot and tool combination, because the
tools change from run to run and per arm, and every full copy of a robot is one
more file a calibration change has to reach -- the multiplication the next
section is about.

## Calibration baked into filenames

The old layout encoded calibration in URDF *filenames*
(`husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf` versus
`..._Alice_Calibrated.urdf`), multiplying files by tool by calibration.

Do not diff two URDFs against each other either: a calibrated URDF and a working
URDF differ in which end effectors are mounted, so their link sets differ, and
xacro expansion order makes a structural diff unreliable.

**What the rewrite does about it (planned, not yet implemented).** Calibration
is a small explicit overlay keyed by joint name -- each entry a delta on that
joint's origin xyz/rpy -- applied after parsing. Reviewable and diffable in git,
and independent of which tool is mounted. For now `RobotConfig.calibration_file`
exists but nothing reads it, and the calibrated URDFs are still used (TODO in
`config.py`).

## Blocking waits on the single thread

Everything that touches state runs on one thread: the tick, every ROS callback,
every plugin hook and task. viser runs its own thread, and its GUI callbacks may
only `submit` work to the main thread and return; long planning searches run on
worker threads, on a copy of the world. A plugin that waits by looping until the arm stops moving freezes the
entire node, including the subscriptions that would have told it the arm
stopped. The old code hit this constantly.

**What the rewrite does about it.** One asyncio loop on the main thread runs
the tick and every plugin task. Anything longer than a tick is an `async def`
started with `ctx.spawn`, and it waits by awaiting: `ctx.wait_until`,
`ctx.sleep`, `ctx.ros(future)`, or `ctx.run_in_thread` for heavy computation.
Between two awaits a task has the thread to itself and needs no locks.

The tick pumps ROS: it runs every waiting ROS callback at its start, then
handles a soft stop, updates forward kinematics, copies the whole world for
planners and the view, runs the plugin hooks (intents, then update), resumes
the tasks waiting for it, and draws. Between ticks
`WorldState` does not change and always matches the kinematics and the copy. The price is that a fast topic needs a queue deep enough for one tick.

Considered and rejected:

- *Generator jobs advanced once per tick* (the first version of this rewrite).
  They worked, but cancellation, timeouts, waiting on ROS futures and handing
  work to threads were all hand-built.
- *`rclpy.spin` on its own thread.* Every subscription would then write state
  concurrently with plugin code, bringing the locking back.
- *rclpy's own coroutine support.* In Humble it has no sleep, no timeout and no
  real cancellation.

A plugin that fails too often is stopped together with every plugin that
requires it, and the panel shows the monitor as broken until restart. Carrying
on with a dependent reading frozen state would be worse than stopping it.
Stopping only cancels their tasks, to end the error spam: their UI and scene
bodies stay until shutdown, on purpose, so the operator can investigate before
restarting.

## Rebuilding the UI on every state change

The old UI tore down and rebuilt the entire panel on every state change --
`reset_ui()` was called on every `BarAction` load and every servoing iteration.
That is why plot contents had to be manually re-pushed afterwards by
`_repopulate_*` methods.

viser is retained mode, so this must not come across: build handles once in
`setup`, mutate them in `draw`. Re-adding nodes every tick would leak and
flicker.

## Why the scene is backend-free, and copied once per tick

The first version of this rewrite had no abstraction over PyBullet: one shared
PyBullet client with the live robots, raw body ids handed to every plugin, and
`RobotScene.active()` (now `PyBulletMirror.active()` in
`bar_assembly_core/mirrors/pybullet.py`) as the only wrapper (it fixed the old code's scattered
`saved = pp.CLIENT; pp.CLIENT = ...`, which corrupted the global on early returns).
Of the old code's PyBullet calls, ~160 were forward-kinematics plumbing and only
15 were collision or planning, so wrapping "the scene" looked unjustified.

It stopped fitting once planners came in (`base_planner` first):

- **Every planner needs its own world anyway.** A search runs for seconds on a
  worker thread, and PyBullet is not thread-safe. The base planner already
  copied robots out of the shared scene into a private one before each search.
- **Geometry can't be read back out of PyBullet** (`getMeshData` gives nothing
  for concave meshes, hull points only for convex ones), so a PyBullet world is
  a poor source of truth for other backends (compas_fab, later coal).
- **A body one plugin added was an obstacle for every other, and nothing
  removed it** when that plugin went away. Now a plugin's bodies go with it at
  shutdown (not when it is stopped for failing; see above).
- **The forward-kinematics plumbing needs no physics engine.** `kinematics.py`
  does it with yourdfpy, once per tick.

So the core now keeps a plain-Python scene (`bar_assembly_core/scene.py`, filled by the monitor's `world/scene.py`) that plugins put bodies
into, and copies the whole world once per tick, right after the ROS pump and
forward kinematics. Planners sync their own mirror (`bar_assembly_core/mirrors/pybullet.py`) from
that copy on their own thread; the 3D view draws the same copy. The design, its
alternatives and measurements: `scene_refactor_plan.md`.

## Naming

The UI module is `ui/visualization.py`, not `husky_viser.py`. Naming a module after
its library means swapping the library renames every import above it. Plugins
talk to a `PluginView` (in `ui/visualization.py`) through their context, so most of them
never mention viser at all.

The core is grouped by what a module is about, not by who calls it:
- `monitor.py`, `config.py`, `tool_urdfs.py` stay at the top: the entry point
  and the run configuration everything else is built from.
- `plugin_api/`: what a plugin is (`plugin.py`) and what it is handed (`context.py`, `concurrency.py`).
- `plugins/`: one module or package per feature; `plugins/planning/` is the
  shared base for the planner plugins and registers none itself.
- `world/`: what is in the world and where. `measured.py` (formerly `world_state.py`)
  holds what sensors report; `scene.py` what we put there; `kinematics.py` the
  link poses; `mirrors/` the planners' private copies; `checks.py` and `mocap.py`
  verdicts on a sensor or mocap fix, as data.
- `robot_interface/`: the only package that talks to a robot.
- `ui/`: the viser server (`visualization.py`), the 3D view (`scene_view.py`) and
  the panel widgets (`style.py`, formerly `ui_style.py`; `ghost.py`,
  `pose_input.py`, `checklist.py`, `pybullet_window.py`); `quaternion.py` holds
  the quaternion order at the viser boundary.
- `design_io/`: reads, writes and validates design folders; imports nothing else
  of ours, since it moves to its own repository later.
