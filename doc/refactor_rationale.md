# Why the monitor was rewritten the way it was

Background for the `husky_assembly_teleop` core (`monitor.py`, `plugin.py`,
`context.py`, `concurrency.py`, `config.py`, `world_state.py`,
`robot_interface.py`, `robot_scene.py`, `visualization.py`).

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
URDF and only from the URDF. Mount a different tool, ship a different URDF.

## Calibration baked into filenames

The old layout encoded calibration in URDF *filenames*
(`husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf` versus
`..._Alice_Calibrated.urdf`), multiplying files by tool by calibration.

Do not diff two URDFs against each other either: a calibrated URDF and a working
URDF differ in which end effectors are mounted, so their link sets differ, and
xacro expansion order makes a structural diff unreliable.

**What the rewrite does about it.** Calibration is a small explicit overlay
keyed by joint name -- each entry a delta on that joint's origin xyz/rpy --
applied after parsing. Reviewable and diffable in git, and independent of which
tool is mounted.

## Blocking waits on the single thread

Everything runs on one thread: the tick, every ROS callback, all planning, all
PyBullet. A plugin that waits by looping until the arm stops moving freezes the
entire node, including the subscriptions that would have told it the arm
stopped. The old code hit this constantly.

**What the rewrite does about it.** `concurrency.py`: plugin code is written as
a generator and yields whenever it is willing to be interrupted; the monitor
advances it one step per tick. Between two yields a plugin has the thread to
itself and needs no locks.

## Rebuilding the UI on every state change

The old UI tore down and rebuilt the entire panel on every state change --
`reset_ui()` was called on every `BarAction` load and every servoing iteration.
That is why plot contents had to be manually re-pushed afterwards by
`_repopulate_*` methods.

viser is retained mode, so this must not come across: build handles once in
`setup`, mutate them in `draw`. Re-adding nodes every tick would leak and
flicker.

## Why there is no abstraction over PyBullet

No scene partitions, no collision-filter type, no body-id hiding, no add/remove
API. Plugins get the raw client id and the raw body ids.

A study of the old code settled this. Of the PyBullet calls that survive the
viser move, ~160 are forward-kinematics plumbing and 15 are collision or
planning -- and all 15 sit on a path the old monitor's own docstring calls the
legacy fallback. `pairwise_collision` is never called at all. The live planning
path hands everything to `compas_fab`, which materializes its own cell and takes
a declarative `RobotCellState`. An abstraction over "the scene" would be
wrapping an API the code barely uses and whose interesting half `compas_fab`
already owns.

If a real shared need shows up once the plugins are ported, factor it out then,
against actual call sites.

The one thing that *is* wrapped is client selection: `RobotScene.active()`. `pp`
reads a module global to decide which client to talk to, and the old code did
`saved = pp.CLIENT; pp.CLIENT = ...` at scattered call sites, corrupting the
global whenever something returned early.

## Naming

The UI module is `visualization.py`, not `husky_viser.py`. Naming a module after
its library means swapping the library renames every import above it. Plugins
talk to a `PluginView`, which is a Protocol in `context.py`, so most of them
never mention viser at all.
