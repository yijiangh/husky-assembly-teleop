# Scene refactor plan: one backend-free scene, copied once per tick, mirrored into each planner

Status: **phases 0, 1–3 and 5 implemented** (see §9); phases 4 and 6 open. It replaces the shared PyBullet scene
(`robot_scene.py`) described in `refactor_rationale.md` ("Why there is no abstraction over PyBullet").

Read this first if you are picking it up:
- `husky_assembly_teleop/world/`: `scene.py`, `geometry.py`, `kinematics.py`, `mirrors/pybullet.py`, `mirrors/compas_fab.py`; `ui/scene_view.py`
- `monitor.py` (`_tick`), `plugin_api/context.py` (`ctx.scene`, `ctx.kinematics`)
- `husky_assembly_teleop/old/cfab_session.py` and `husky_assembly_teleop/old/husky_monitor.py::_bridge_cfab_to_pp_for_bar_action` (how compas_fab was used before)

---

## 1. Goals

1. **One scene of collision objects**, owned by the core and free of any physics or collision backend. Plugins (`obstacles`, `cell`) add their objects to it.
2. **Planners get a consistent copy of the whole world**: measured robots and tracked objects plus every scene object, all from the same moment. They mirror it into their own backend: PyBullet (`base_planner`), later compas_fab, pinocchio + coal.
3. **The core draws the scene** (robots, tracked objects and scene objects), with toggles per group.
4. **Syncing a mirror is cheap.** Geometry is converted once and uploaded once per backend. Afterwards a sync only moves what moved.

Non-goals, for now: a planner changing an attachment while it plans, and several planners controlling one robot (execution locking).

## 2. The tick

```
1 pump ROS → 2 soft stop → 3 kinematics.update → 4 take_snapshot → 5 plugin steps → 6 resume tasks → 7 draw
```

- **Measured state changes only in step 1**, and forward kinematics is fixed in step 3. Step 4 copies the whole world: robots, tracked objects and every scene body with its resolved world pose.
- **Plugins read and write the scene freely** during steps 5 and 6: in place (`body.placement = pose`) or with `ctx.scene.put`. One thread, fixed order, so there is no race.
- **The copy is consistent by construction.** Nothing runs between the ROS pump and the copy, so it holds one pump's measurements (base, joints and attached bodies agree) and every plugin's complete writes up to the end of the previous tick. Writes made during a tick show up in the next tick's copy: one tick of latency (50 ms).
- **Planners hand the copy (`ctx.scene.snapshot`) to their worker thread**, which syncs its own mirror from it. The 3D view draws the same copy, so what you see is what a planner gets.
- ! Base and arm come from different sensors (mocap, joint states), sampled at different instants. The copy can't make them agree in time. `RobotEntry.base_time` and `joints_time` record when each was measured, so a planner can check the skew while a robot moves.

## 3. Decisions and why

| Decision | Why | Rejected |
|---|---|---|
| The core scene is **plain Python data** and imports no physics or collision library | Planners use different backends. PyBullet only for kinematics and collision checks is more than we need. compas_fab is only an import source (`Geometry.from_rigid_body`). | A central, shared PyBullet world. Every planner needs its own world anyway, because searches run for seconds on a worker thread. |
| **Bodies are mutable; the world is copied once per tick** | Simplest for plugins: change a field. The copy (0.7 ms for 2000 bodies) is what worker threads read, so they never see a half-written change. | Frozen objects compared by identity, a commit point after all plugins, and a shared diff. More rules for the same guarantee. |
| **Geometry is immutable, and is recognised by object identity** | Only building is expensive (2–30 ms per mesh body in PyBullet). A new `Geometry` object is the signal to rebuild. Comparing meshes by value would cost as much as converting them. | Content hashing each sync. |
| **Mirrors detect ids appearing and disappearing, and compare poses by value** | Comparing 2000 poses costs 0.1 ms; resetting 2000 bodies in PyBullet costs 14 ms. So only changed poses are sent. | Change logs, revision counters, dirty flags: each needs bookkeeping in every writer. |
| **`RobotConfig` is the robot's description** | It already names the stitched URDF and is frozen. A new tool means a new config, which is the signal for mirrors to reload. | A separate `RobotDescription` class. |
| **Kinematics is its own core object** (`kinematics.py`), updated once per tick | One place owns the changing yourdfpy state and the rule "an absent measurement keeps its last value". Its own parse, without meshes (~2 ms), so the 3D view's models are separate. | A yourdfpy model shared with the drawing, where every caller must set the configuration first. |
| **compas_fab structures stay at the boundary** | `RobotCell` describes the world from one robot's point of view (link names without a robot), and copying compas data is slow (`RobotCellState` with 2000 bodies: 194 ms). Its `RigidBody` only takes meshes (no primitives). So the scene has its own shapes; `RigidBody`, `RobotCell` and `RobotCellState` are built inside a compas_fab mirror. | `RobotCellState` as the scene. |
| **Drawing the scene is core** | Generic, follows every plugin's objects (including removal when a plugin closes), and keeps updating while panels are frozen. | A `scene_view` plugin. |

Measurements (this machine):

| Operation | Time |
|---|---|
| `p.connect(DIRECT)` | 4-19 ms |
| `loadURDF` one robot (28 joints) | ~180 ms (355 ms cold) |
| Create box body / convex mesh 2k vertices / concave trimesh 10k vertices | 0.13 / 1.8 / ~30 ms |
| PyBullet `resetBasePositionAndOrientation` × 2000 | 14 ms |
| Copy 2000 body records (sharing geometry) | 0.7 ms |
| Compare 2000 poses by value | 0.1 ms |
| yourdfpy parse without meshes / `update_cfg` + all links (Panda) | ~2 ms / ~0.3 ms |
| compas `Frame.copy()` / `RigidBodyState.copy()` / `RobotCellState(2000).copy()` | 22 µs / 75 µs / 194 ms |

Also verified:
- **PyBullet reuses body ids.** Never hold a PyBullet id outside its mirror.
- **`getMeshData` can't give geometry back** (0 vertices for concave meshes, hull points only for convex ones).
- **compas_fab's `PyBulletClient.connect()` always creates its own world.** Don't adopt another world's id.
- **compas_fab's collision check only considers bodies on its own lists** (`robot_puid`, `tools_puids`, `rigid_bodies_puids`).
- **`viser.extras.ViserUrdf.update_cfg`** sets the configuration and reads every transform in one call, so several instances can share one parsed model on one thread.
- **compas `Frame.__eq__` compares with a tolerance.**
- **A `GEOM_MESH` given `indices` is always concave**, even without `GEOM_FORCE_CONCAVE_TRIMESH`, and two such meshes never report contacts. Convex shapes must be built from vertices only.
- **`removeCollisionShape` refuses a shape any body has ever used** (PyBullet 3.2.x), even after that body is gone. Replaced shapes stay allocated until the world is closed (~12 MB per 1000 boxes); rebuild the mirror if that ever matters.

## 4. Ids

Each object has **exactly one id**, a string path we assign. Backend ids exist only inside a mirror and are translated back in every result.

```
robots/0806                         robot           (core)
robots/0806/left_ur_arm_tool0       one link of it  (for attachments and touches)
tracked/probe                       tracked object  (core)
obstacles/tables/operator_table_A   scene body, owner = plugin "obstacles"
cell/bars/B1                        scene body, owner = plugin "cell"
```

- **Allowed characters:** `[A-Za-z0-9_.-]` and `/`. Display text goes in `label`.
- **Owner:** a plugin may only `put` and `remove` ids starting with its own name (`ctx.scene` checks). When a plugin closes, the monitor removes everything under `<plugin>/`.
- **Groups:** the path is the grouping. The view's toggles are path prefixes.
- **Stable:** an id never changes while its object exists.

## 5. Data model

### 5.1 `scene.py`
- `Pose(position, orientation)`: frozen, compared by value. `compose(a, b)`, `Pose.from_matrix`, `Pose.matrix()`.
- `Attachment(parent, link, grasp)`: held by `robots/<serial>` (link, or None for the base) or `tracked/<name>`.
- `Body(id, geometry, placement: Pose | Attachment, touches, label, color)`: **mutable**. `touches` lists ids allowed to touch it (bodies, `robots/<serial>` for the whole robot, `robots/<serial>/<link>`, `tracked/<name>`); it is symmetric.
- `TrackedDescription(geometry, touches, label)`: given to `ctx.track_object`. `WorldState.TrackedObject` stays measurement only.
- `Scene`: `bodies` (live dict), `tracked` (descriptions), `put`, `remove`, `remove_prefix`, `snapshot`, `take_snapshot` (monitor only).
- `SceneSnapshot(tick, time, bodies, world_poses, robots, tracked)`: `bodies` are copies (sharing geometry); `world_poses` holds every body's resolved world pose; `robots` is `RobotEntry` by serial; `tracked` is `TrackedEntry` by name, only those with a fix.
- `PluginScene` (`ctx.scene`): owner-checked `put`, `put_many`, `remove`, `remove_prefix`; `bodies`; `snapshot`.

### 5.2 `geometry.py`
- A **shape** is `TriMesh | BoxShape | CylinderShape`, in the body's frame. Later maybe more primitives, or a URDF.
  - `TriMesh(vertices, faces, convex)`: read-only numpy arrays, `convex` computed once. Compared by identity.
  - `BoxShape(size, origin)`, `CylinderShape(radius, height, origin)` (along Z, centred): frozen, compared by value. `origin` places the shape inside the body, as a URDF `<origin>` does.
- `Geometry(visual, collision)`: tuples of shapes. `box_geometry`, `cylinder_geometry`; `from_rigid_body` imports compas_fab meshes (slow, on a loading thread).
- **Each backend uses the primitives it supports natively** (viser `add_box` / `add_cylinder`, PyBullet `GEOM_BOX` / `GEOM_CYLINDER`, coal `Box` / `Cylinder`) and turns the rest into triangles with `shape_mesh(shape)`, which is cached (equal primitives share one mesh).
- ! Never change a `Geometry` in place. To change a shape, assign a new `Geometry` to the body.
- ! No collision meshes means the body never collides (as in compas_fab); there is no fallback to the visual meshes. `ctx.scene.put` and `track_object` warn about such bodies, once per id.
- Share one `Geometry` between bodies with the same shape (e.g. repeated joint pieces): each mirror then uploads it once.

### 5.3 `kinematics.py`
- `Kinematics(robots, log_warn)`: parses each stitched URDF once without meshes.
- `update(world)`: base from mocap when tracked, else the last one (the default at start); joints from the latest measurement, else the last value; then one `update_cfg` per robot.
- `base_pose`, `joints`, `unmeasured`, `joint_names`, `link_names`, `link_pose(serial, link)`.
- Plugins use `ctx.kinematics`. Nothing else outside the 3D view parses URDFs.

## 6. Mirrors (`world/mirrors/`)

Rules shared by every mirror:
- It keeps what it built, by our id, together with the `Geometry` (or `RobotConfig`) object and the pose it last applied.
- On `sync(snapshot)`: an id no longer present is removed; a new id, or a different `Geometry` / `RobotConfig` object, is (re)built; otherwise the pose is compared by value and applied only if it changed. Robots get base and joints every sync (cheap).
- Shape caches are keyed by the `TriMesh` object itself, never by `id(obj)`, because Python reuses ids.
- Each mirror is used by exactly one thread. A planner owns one mirror and one worker (`ThreadPoolExecutor(max_workers=1)`):

  ```python
  snapshot = ctx.scene.snapshot                                  # main thread
  result = await loop.run_in_executor(self._executor, self._plan, snapshot, goal)
  # worker: self._mirror.sync(snapshot); then search in the mirror's world
  ```

### 6.1 `PyBulletMirror` (`world/mirrors/pybullet.py`)
- Its own `p.connect(p.DIRECT)`; raw `p.*` calls with `physicsClientId` only.
  - ! `pp.CLIENT` is one global for the process. Use `pp` only inside `mirror.active()` on the owning thread, and only one `pp` planner thread at a time.
- Robots: `loadURDF(config.urdf_file, useFixedBase=False)`, reloaded when the `RobotConfig` object changes; base and joints set every sync.
- Bodies and tracked objects with geometry: one multibody per collision shape. Boxes and cylinders are exact PyBullet primitives, their `origin` as the collision frame. Meshes are built from our arrays (faster than compas_fab's OBJ round trip). Equal primitives share one PyBullet shape.
  - ! Concave (`GEOM_FORCE_CONCAVE_TRIMESH`) only for a free body whose mesh isn't convex. Attached bodies and tracked objects are always convex: Bullet can't collide two concave meshes. A body that changes between free and attached is rebuilt.
- API: `robot(serial)`, `body_ids(id)`, `id_of(pybullet_id)`, `allowed(a, b)` (symmetric, from `touches`), `collisions(serial, margin)` returning our ids, `obstacle_ids()`, `close()`.
- A planner may add its own private bodies to its mirror's world (e.g. proxy spheres); they're never part of the shared scene.

### 6.2 `CompasFabMirror` (`world/mirrors/compas_fab.py`, one per acting robot)
- compas_fab's own `PyBulletClient("direct")`. `RobotCell`: the acting robot's stitched URDF and SRDF (`RobotConfig.srdf_file`); every other robot as a `ToolModel` from its stitched URDF, key `robots/<serial>` (the design does the same: `ObstacleRobot<Name>`); every body with collision meshes as `rigid_body_models[id]`, keys are our ids. Bodies without collision meshes are left out (compas_fab would fail on them).
- ! The SRDFs predate the stitched tools: each tool link may also touch its tool and `<arm>_{wrist_2_link, wrist_3_link, flange, tool0}` (`tool_urdfs.TOOL_TOUCHES_ARM_LINKS`), as the design's `ToolState.touch_links` allow.
- ! compas_fab checks every stationary tool against every stationary body. Pairs already touching at `sync` can't change while this robot plans, so they are allowed for that snapshot (`static_contacts`).
- `collisions(joints)` is compas_fab's `check_collision` (~25 ms with 50 bodies: it deep-copies the state per call). `search_check(joint_names)` resolves compas_fab's allowed pairs once and checks only pairs with a moving side (~1.5 ms); tests hold it equal to `check_collision`.
- **Converting shapes is this mirror's job:** each `Geometry` becomes a `RigidBody` of compas meshes, from `shape_mesh(shape)` for every visual and collision shape, cached per `Geometry` object for the mirror's lifetime.
- `RobotCellState` from the snapshot:

  | Our data | compas_fab |
  |---|---|
  | Acting robot base and joints | `robot_base_frame`, `robot_configuration` |
  | Body attached to the acting robot | `attached_to_link=link`, `attachment_frame=grasp` |
  | Body attached to another robot or a tracked object | stationary: `frame` = `world_poses[id]`, the parent's id in `touch_bodies` |
  | `touches` entry that is one of the acting robot's links | `touch_links` |
  | any other `touches` entry | `touch_bodies` |

- Any id added or removed, or a geometry changed: rebuild with `set_robot_cell` (slow, OBJ files), only at plan start. Otherwise `set_robot_cell_state`.
- compas_fab 1.1.0's PyBullet backend treats a `RigidBody` without collision meshes as "no collision" (its docstring also promises a fallback to the visual meshes, which the code doesn't do). Our `Geometry` does the same, so the two agree.

### 6.3 `CoalMirror` + pinocchio (later, sketch)
- Robots from `pinocchio.buildModelsFromUrdf`; bodies as `coal.BVHModel` / `coal.Convex`; attached bodies on their link's frame. Boxes and cylinders map to exact `coal.Box` / `coal.Cylinder`.

## 7. Drawing (core, `ui/scene_view.py`, owned by `Visualization`)

`Visualization.draw(snapshot)` runs every tick, also while panels are frozen.

| What | Nodes | Updated |
|---|---|---|
| Robots | `/robots/<serial>` base frame + `ViserUrdf` | every tick from `snapshot.robots` |
| Tracked objects | `/tracked/<name>` frame + visual meshes (if any); hidden without a fix | every tick |
| Scene bodies | `/scene/<id>` frame, meshes `/scene/<id>/<i>` | built when new or the geometry changed (at most N per tick), moved when the pose changed, removed when gone |

- No visibility checkboxes of its own: viser's debug view already toggles each scene path.
- Batched building: a large cell fills in over a few ticks. Moves and removes always apply in full, so a body is at worst missing for a few ticks, never shown at an old pose.

## 8. Rules for plugin authors (in `plugins/__init__.py`)

- Put collision objects in `ctx.scene` under `<plugin name>/…`. Don't draw them yourself; the core does. Removal when your plugin closes is automatic.
- Change a body in place or `put` a new one with the same id. To change its shape, assign a new `Geometry`; never edit one.
- Build `Geometry` for large meshes on a loading thread (`ctx.run_in_thread`), then `put` on the main thread.
- For robot poses and link poses use `ctx.kinematics`. Don't parse URDFs yourself.
- To plan: take `ctx.scene.snapshot` on the main thread, sync your own mirror on your own worker thread, report results with our ids.
- Measured objects are never put in the scene. Track them with `ctx.track_object(name, mocap_id, geometry=..., touches=...)`.

## 9. Phases

0. ✅ **compas_fab spike.** Other robots as `ToolModel`s work (the design uses them too). `set_robot_cell`: ~2.3 s (robot + 2 tools ~1.9 s, 92 bodies ~0.4 s); loading 3 models ~8 s. Findings in §6.2.
1. ✅ **Kinematics.** `kinematics.py`, `ctx.kinematics`; `robot_control` and `base_planner` stop reading PyBullet.
2. ✅ **Scene and drawing.** `geometry.py`, `scene.py`, `ctx.scene`, `take_snapshot` in the tick, core drawing with toggles; `mocap_probe` tracks without drawing its own frame.
3. ✅ **`PyBulletMirror` and the switch.** `obstacles` and `base_planner` move to the scene together; `example_pybullet` rewritten; `robot_scene.py` deleted.
4. **Cell objects.** The cell plugin puts bodies (ids, ground with the wheel links in `touches`, attachments from authored states) and drops its own body drawing. Needs open questions 1 and 2.
5. ✅ **`CompasFabMirror`.** Same collision pairs as the design's `RobotCell` on 10 authored Cindy states and 5 with the held bar pushed into the robot (`test/test_compas_fab_mirror.py`, needs `HUSKY_DESIGN_DIRECTORY`). First user: the `arm_planner` plugin (joint-space `birrt`, commit is a stub).
6. **Published copies.** A planner publishes a modified snapshot for the view (`/worlds/<name>/…`, see-through robots); `base_planner` publishes instead of drawing its own ghost.

## 10. Open questions

1. **Design naming.** Is `env_` plus `bar_` / `joint_` a fixed convention of the design export? Where does the map from design robot id to serial belong?
2. **Tool mapping.** Partly answered: `AT3L` → `left_ur_arm_scaffolding_v3_left`, `AT3R` → `right_ur_arm_scaffolding_v3_right` (Cindy); bodies attach to `<arm>_tool0` with the grasp as `attachment_frame`. `SupportGripper` (Alice, Belle) is still to map.
3. **Attachments.** Static and set by `cell` for now. Decide later whether planners take over while they plan.
4. **pinocchio layout.** One model per robot, or all appended into one.
5. **Drawing budget.** N meshes per tick (start at 50); simplified visual meshes for very large cells?
6. **Snapshot freshness.** Refuse or flag robots with `base_tracked=False`, `unmeasured` joints, or a base/joint time skew over a limit? Or leave that to each planner?
7. **`pp.CLIENT`.** (`CompasFabMirror` and `arm_planner` use raw `p` only.) Two `pp` planners at once would conflict. Then either raw `p`, or one `pp` planner at a time.
