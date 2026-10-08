# Prompt: extract the shared core and move the monitor onto it

You are working in `husky-assembly-teleop`, branch `jg/viser-cleanup`. Your job is to extract a shared core out of
the monitor and move the monitor onto it.

The core is three things: `design_io`, the scene, and the mirrors. Later, the Rhino plugin
(`bar_joint_rhino_design_workflow`) and the planner library (`husky_assembly_tamp`) will use the same core.
Do not change those two repos. Their maintainers review the core's API once you are done, so your final report
is written for them.

Read `AGENTS.md` first and follow it. That covers simplicity first, the comment style and the build and test
commands. It also requires a plan file: write yours to `tasks/<yyyy-mm-dd>_shared_core.md` at the end of plan
mode, and keep it updated.

## Decisions already made (do not reopen them)

### The model

1. **The design is the high-level plan.**
   - It holds robots, tools, bodies, the schedule, and authored states and targets.
   - It gives scenes through two convenience methods:
     - `design.scene_at(movement)`: the world at one movement;
     - `design.scene_after(bar)`: the world once that bar is built, following the schedule.
   - It knows nothing about planning: no collision scopes, IK seeds, ssik artifacts or cell layouts.
   - It stays an ordinary editable object. Do not turn design_io into immutable or versioned types.
2. **A scene is one moment: `Body` and `RobotObject` objects.**
   - Whoever owns a scene may edit it, for example to disable the held bar for a plan.
   - `scene_at` and `scene_after` return scenes that share nothing mutable with the design. Joints, poses and
     touches are copied. Only `Geometry` and `RobotModel` are shared.
3. **`Geometry` and `RobotModel` are immutable values shared by reference.**
   - A new shape or model is a new object.
   - An unchanged one keeps its object, so a mirror can tell "unchanged" by identity.
4. **Mirrors and other backends only ever receive scene copies, never a design.**
   - A mirror diffs each scene against the last one it applied: poses, joints, `enabled` and attachments by
     value; `Geometry` and `RobotModel` by identity.
   - From that diff it writes a state or rebuilds its world.
   - Both mirrors use the same test. Today `CompasFabMirror` compares `RobotConfig` by equality
     (`world/mirrors/compas_fab.py:187,196`), while `PyBulletMirror` compares by identity
     (`world/mirrors/pybullet.py:142`).

### Robots

5. **One `RobotObject` type for planned and real robots.**
   - A real robot is a scene object whose base and joints the robot interface writes each tick.
   - It carries validity flags: `base_tracked`, `unmeasured` joints and timestamps.
   - Mirrors and planners refuse an acting robot whose base is untracked or whose joints are unmeasured. Today
     the monitor fills in a default base and the stow pose (`world/kinematics.py:62-67`), sets these flags, and
     nothing reads them.
   - An absent robot is disabled in the scene. The compas_fab mirror parks it, because compas_fab needs every
     tool in every state.
6. **No merge rules.** Plugins decide which robots a scene holds: the plan as ghosts, the real robots, or both.
7. **`RobotModel` versus `RobotConfig`.**
   - `RobotModel` is what mirrors build from: URDF, SRDF, mounted tools with their touches, the flange list, and
     the frame convention (StockUrFrames or not). Today's `RobotSpec` is the starting point.
   - Move `TOOL_TOUCHES_ARM_LINKS` (`tool_urdfs.py:30`) onto the model.
   - `RobotConfig` stays in the monitor only. It holds serials, namespaces, controllers and drivers; it builds the
     `robot_interface` and a live `RobotObject`.
   - After this change the mirrors import nothing from the monitor.
8. **Tools in compas_fab planning cells are separate `ToolModel`s attached to their groups.** That is how Rhino and
   tamp build cells; tamp's keyframe IK refuses a cell without `tool_models`. Stitched tool URDFs remain only for
   drawing and for the live model.

### Attached, on, mounted and mated bodies

9. **Attached:** a body on a robot link, with a grasp in that link's frame (`State.attached`, as in schema 1).
   Always to a robot link, never to another body.
   - `attached` means a robot holds the bar; schema 2 adds a separate per-state flag `built` (fixed in the
     structure once one of its halves is mated). Pose rule: a built bar takes its design pose; an attached, unbuilt
     bar follows its first holding link times the grasp; any other bar takes a pose written in the state, or its
     design pose. A bar may have several holders, each with its grasp: Cindy's two flanges, or Alice and Cindy both
     on a built B3.
   - Schema 1 data attaches every body moved with a bar, its joint halves included. Keep reading that; schema 2 will
     attach only the bar.
   - Moving the plan onto the real robot swaps the robot id only: `retarget(scene, robot_map)`. This covers
     attachments and tool `on` relations. The link names are the same in both URDF variants. Refuse a link that the
     target model lacks.
   - A robot that holds a built bar is frozen: it may only move under compliant control, in a movement whose tool
     part releases that bar (compliant ungrasp or untighten). A tool on a body it does not hold allows only a careful
     retreat. Schema 1 data has no `built`: treat a present, unattached bar as built.
10. **Mounted** (a connector half fixed to its bar) is static and rigid: the half follows its bar, so in schema 2
    only bars are attached. **On** (the body a tool touches and acts on) and **mated** (male to female half, or
    ground half to ground) are recorded relations that never move anything. A mate is open, pending or engaged,
    derived from whether its bars are present and attached.
    - `mount`, `mates`, `on` and derived allowed contacts (which replace `touches`) are schema 2 and out of scope
      here. Leave room for them, but do not add them.
11. **Mocap is used for robots, and for bars only to verify placement.** A measured bar pose never enters a
    planning scene.

### Hosts

12. **Hold scenes are a planning helper, not a schema field.** The helper takes `scene_after(release bar)` and
    disables the held bar; the release bar comes from `schedule` and `supports_until`. Put it in the core,
    outside `design_io`: scenes in, scenes out.
13. **The monitor's cell plugin only loads designs and produces scenes.**
    - It never edits a design.
    - Plugins edit scenes.
    - Execution writes results; it never writes designs.
14. **Ids.**
    - The core uses one canonical id per object (`bars/B1`, `robots/cindy`).
    - Small explicit maps sit at the edges: planned robot to real robot (`robots/cindy` to `robots/a200-0806`),
      and mocap rigid body to design body.
    - An unmapped id is refused.
15. **`mirror.lend()` (compas_fab mirror)** hands its planner to code such as tamp.
    - It calls `pybullet_planning.set_client` for the mirror's client and restores the previous client afterwards.
    - It marks the mirror dirty, and the next `sync` writes the full state.
    - Tamp relies on the process-global `pp` client and leaves the world in whatever state it last set.
16. **Tick budget: 50 ms on the main thread.**
    - Run `scene_at` only when the loaded design or the selected step changes.
    - Never deep-copy a `Design` per tick: it takes 27 ms on the 260814 design.
    - Copying the scene costs about 0.04 ms for 94 bodies.

## Out of scope

- Schema changes: the format stays at schema 1. Keep notes on the schema 2 candidates you hit, such as
  `connections`, tool state per `State`, typed `notes` and `solutions/`.
- The Rhino and tamp repos, a VAMP or PRM scene export, IK, and the Commit stubs.
- Moving the core to its own git repository. Build it so that move is a copy.

## Where the core goes

- Create the core as a separate top-level Python package in this repo, next to `husky_assembly_teleop/`.
  - The working name is `husky_core`. Confirm the name with the user during planning.
  - It holds `design_io/` (moved), the scene, the mirrors, a ROS-free FK provider, the id maps and the hold helper.
- **Rules for the package:**
  - It imports nothing from `husky_assembly_teleop`, nor ROS, viser or `crl_husky`.
  - It runs on Python 3.9.
  - Its dependencies are numpy, scipy (`design_io/pose.py:9` already needs it, undeclared) and trimesh. The
    mirrors may also use compas_fab, compas_robots, pybullet and pybullet_planning.
- **Stays in the monitor:** `RobotConfig`, `robot_interface`, the tick, `take_snapshot` (it reads monitor
  state), plugins, viser drawing and `config.py`.

## Known facts to work with

- **Kinematics imports ROS.** `world/kinematics.py:18` imports `UR_JOINT_NAMES` from `robot_interface.arm`, which
  imports rclpy and ROS messages (`robot_interface/arm.py:13-23`).
  - Move the names to a ROS-free module.
  - Build the core's FK provider from the yourdfpy code.
- **`design_io/compas_fab.py` is used at runtime.**
  - `CompasFabMirror` imports `filled`, `frame_from_pose`, `load_model`, `rigid_body` and `subtree` from it
    (`world/mirrors/compas_fab.py:31`).
  - `legacy.py:25` imports `PARKED_POSITION` and `pose_from_frame` from it.
  - Move these helpers into the mirror package first.
  - After that, keep `to_robot_cell` / `to_cell_state` only as a legacy test harness.
- **Scene types to unify.** Today `Body` is mutable (`world/scene.py:59-88`), while robots are `RobotEntry` and
  tracked objects `TrackedEntry`, each in its own snapshot dict (`scene.py:108-171`).
  - `Attachment` already allows a robot link or a tracked object as parent (`scene.py:45-56`).
  - Keep resolved world poses in the snapshot: `PyBulletMirror` reads `snapshot.world_poses`
    (`pybullet.py:130`).
- **The cell plugin flattens the design.** `plugins/cell/design.py:166-202` (`obstacles`) puts design bodies into
  the scene. It disables held bodies and bodies at placeholder poses, and drops contacts with robots and tools.
  `scene_at` plus scene edits replaces it.
  - The plugin re-puts bodies only when `revision` changes (`cell/plugin.py:312-331`), and planners poll
    `revision`. Keep that mechanism.
- **Scene ids are prefixed by the plugin that put the body** (`cell/bars/B3`, `scene.py:208,355`), while design
  ids are `bars/B3`. This is where the id map applies.
- **`static_contacts` hides contacts.** It allows any stationary tool/body pair that already touches
  (`compas_fab.py:266-278`). With real robots in a scene, that would hide a real tool pressing into a body. Flag it
  in your report; do not change it silently.
- **The URDF variants differ at the arm base links.**
  - The monitor plans on `*_Calibrated_StockUrFrames.urdf` (`config.py:166-174`).
  - The design copy is the non-Stock `*_Calibrated.urdf` (`design_io/conversion.py:25-34`).
  - Both have the same links and joints and give the same flange poses, but `<arm>_base_link` differs by 90°.
  - Record the frame convention on `RobotModel`. Compare models by forward kinematics of named links, not by file.

## Steps

Each step ships on its own. Run the quick tests after every step and the slow set (`-m slow`) after steps 3 to 5.

1. **Move `design_io` into the core unchanged, and move the mirror helpers out of `design_io/compas_fab.py`.**
   Done when all tests pass and nothing in `husky_assembly_teleop` imports `husky_assembly_teleop.design_io`.
2. **Remove ROS from FK, then move the scene and mirrors into the core.**
   - Done when `python3.9 -c "import husky_core, husky_core.mirrors"` works in an environment without ROS.
   - Use `uv` or a 3.9 venv. If no 3.9 interpreter can be installed, stop and ask; do not fake this check.
   - Add a test that fails if any core module imports `husky_assembly_teleop`, rclpy or viser.
3. **Unify the scene and split `RobotModel` from `RobotConfig`.**
   - Make `RobotObject` carry validity flags and represent tracked objects as `Body`.
   - Give both mirrors one identity test.
   - Make mirrors and planners refuse an acting robot whose values are invalid.
   - Done when the monitor runs the existing plugins (`robot_control`, `cell`, `base_planner`, `arm_planner`) as
     before, and both planners' tests pass.
4. **Add `scene_at`, `scene_after`, held attachments with `retarget`, absent robots, and the hold helper.**
   - Switch the compas_fab mirror to `ToolModel` tools with touches taken from `RobotModel`.
   - Make the cell plugin pass design scenes to the mirrors instead of flattening the design through `obstacles()`.
   - Done when `scripts/design_io_equivalence.py`, run through the mirror, gives the same collision pairs as today
     on both benchmark exports, except for pairs that involve the floor.
     - Point `DESIGN_IO_EQUIVALENCE_EXPORT` and `HUSKY_DESIGN_DIRECTORY` at the exports; ask the user for the paths.
5. **Add `mirror.lend()`.**
   - Done when a test runs a tamp call on a lent planner, for example `plan_free_dual_arm` from
     `external/husky_assembly_tamp`, after a `sync`.
   - Then run a second `sync`. It must write the full state, and `pp.CLIENT` must be restored.

Keep the diff small: move code before rewriting it, and reuse existing functions. Update the docs that the change
makes stale: `doc/scene_refactor_plan.md`, `doc/design_format.md` (only where it describes the library, not the
format), and the module docstrings.

## Ask the user before

- choosing the package name, or putting it anywhere other than next to `husky_assembly_teleop/`;
- changing anything in `external/` submodules or their pins;
- any change to the file format.

## Final report (for the Rhino and tamp reviewers)

Write it to the plan file and also return it.

- The public API of the core: each module, its main types, and functions with their signatures.
- What a host must do to use the core:
  - build a `RobotModel`;
  - get a scene from a design;
  - edit that scene;
  - sync a mirror and lend its planner.
- Every decision above that you could not follow, and why.
- Schema 2 candidates found along the way, and anything still stubbed or deferred.
- Test results: the quick and slow runs, the Python 3.9 import check, and the equivalence numbers.
