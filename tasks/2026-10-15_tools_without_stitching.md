# Tools as core `Tool`s, without URDF stitching

Branch `jg/viser-cleanup`, fast-forwarded to `jg/base-planner` @ `5309040` (base planner and the shared core
included). Worktree `/home/jakob/ra/teleop-viser-cleanup`, outside the colcon workspace.

## Status (2026-10-15)

Steps 1–7 done, one commit each (`af2f363`..`c8f1a23`), not pushed. Quick, slow and linter tests pass.
Changed from the plan:
- Step 5 lives in the monitor (`world/model_check.py`), not the core: it only reads core types, and a core change
  needs the consumer review of the core's `AGENTS.md`. Move it there when Rhino wants the same check. The core is
  unchanged; its stale `tools/AT3L` docstrings are not fixed.
- The comparison covers links with shapes and the flanges only: the processes' URDFs turn `<arm>_base_link` (no
  shapes), which changes no plan and would warn on every load.
- No `tcp` link was added to the tool URDFs: the values are open (below). Until then every live `Tool.tcp` is tool0,
  so pendant TCPs must stay at zero, and the process check reports the processes' TCPs as different.

What the check reports for the two local processes (`data_design_study/`, 2026-10-15):
- Cindy's tools: TCP (∓0.07, 0, 0.08) m in both processes.
- The Robotiq: TCP at tool0 in `260814_RobArch_support_ik`, at 166.1 mm with a 180° turn in `261008_2ModulesDemo`.
- The Robotiq's collision bounds differ by 37 mm from `data/tool_urdf/robotiq_2f_85.urdf`.

## The TCP: decided 2026-10-16, pendant at tool0

The pendant's TCP stays at zero (tool0) on every arm, whatever tool is mounted. The robot then reports tool0 itself,
which is what the compliance controller moves and what the force reading is labelled in. The tool's own TCP,
`Tool.tcp`, is only for planners and the process check. The two are kept apart in the code:
`ArmConfig.pendant_tcp` (tool0 by default) is what `arm.flange_from_reported` and the health `tcp` chip use.
The report "Husky Coordinate Frames" (claude.ai artifact) shows why and what goes wrong if the two disagree.

**Done**
- [x] `ArmConfig.pendant_tcp`, used by the conversion, the first-target log line and the health `tcp` chip instead
      of `Tool.tcp` (16 Oct). The chip names the TCP it expects ("zero, i.e. tool0").
- [x] Process check: warns when a process robot, or the configured one, has turned arm base frames (an old URDF).
- [x] `doc/ur_frames.md` says so.

**Values for `Tool.tcp` (planners only; no pendant change)**
- [ ] The support gripper (Robotiq): the new design (`261008_2ModulesDemo`) puts its TCP 166.1 mm out on the held
      bar's axis (to 0.007 mm, "Support Gripper TCP Check" artifact), with its z along the bar; the old export had it
      at the flange. Take the new design's.
- [ ] Cindy's `scaffolding_v3` left/right: (∓0.07, 0, 0.08) m in both local processes; confirm against the CAD.
- [ ] Then add a `tcp` link on a fixed joint to `data/tool_urdf/robotiq_2f_85.urdf`, `scaffolding_v3_left.urdf` and
      `scaffolding_v3_right.urdf` (`tool_urdfs.TCP_LINK`): safe now, robot I/O does not read it.
- [ ] The Robotiq's collision shape: ours (static, closed to 20 mm) or the design's (37 mm apart)?
- [ ] `scaffolding_v1` has no model: give it one, or refuse Cartesian commands on it.

**On the robots**
- [ ] Check the active TCP is zero on all four arms; the `tcp` chip confirms it at three still poses.
- [ ] Payload mass and centre of gravity of the mounted tool on each pendant (the force reading depends on it).
- [ ] Optional: a lower tool speed limit for margin; it guards the flange, the tool tip is up to 166 mm further out.

**Monitor**
- [ ] `ArmState.tcp_*` holds tool0: rename to `flange_*`, or keep and document.
- [ ] robot_control shows and commands tool0 but says "TCP": label it, or show and command the tool tip (compose
      with `Tool.tcp`; invert on command). Turning about the tool tip needs the same.
- [ ] Should an amber `tcp` chip refuse Cartesian commands? Today it only warns.

**Open question: does a pendant TCP change our force reading?**
The UR hardware's `tcp_fts_sensor` is published as `ft_sensor_wrench`, labelled `tool0`, and the compliance
controller reads it as given in tool0 (`ft_sensor_ref_link: tool0`, crl-husky `config/ur_controllers.yaml`). If the
UR gives the wrench about the active TCP, a TCP 166 mm out shifts every torque by the lever arm times the force
while the label stays tool0. With the pendant at zero both are the same point, so it does not matter yet. Test
before any TCP goes on a pendant: one arm still, F/T zeroed, a known sideways load at the tool tip, record; set a test
TCP of (0, 0, 150) mm on the pendant without moving, record again; same torques mean "about the flange". Set it back.

## Goal

A mounted tool is described once, as the core's `Tool` on the robot's `RobotModel`, and everything reads that:
drawing, kinematics, mirrors, planners and the checks. The robot URDF is no longer rewritten with tools stitched on.

## Rules (from the discussion, 2026-10-15)

- Everything in a scene is a core type. Live state (mocap, robot interfaces) is turned into a scene by
  `take_snapshot`, as now: `RobotObject` with `config.model`, tracked objects as `Body`s.
- Reuse the core's notions: `RobotModel.tools`, `Tool` (visual + collision `Geometry`, `tcp`, `kind`),
  `robot_parts`, `process_model`, `IdMap` / `live_ids`, core `link_pose`.
- A tool URDF in `data/tool_urdf/` is only an import source for a `Tool`, as compas_fab is for `Geometry`.
- All URDF drawing uses `FastViserUrdf`. The real robot and its tools always draw this tick's state, never a
  planner's older snapshot.
- Where a tool is must be accurate: its mount on `<arm>_tool0`, its TCP, and checked against the robot.
- The TCP on each teach pendant stays at tool0 (16 Oct; was "set to match the mounted tool" on 15 Oct).

## Safety: the reported TCP is not `tool0`

The driver's `tcp_pose` (crl-husky `onboard/rate_limiter.py:37`) includes the pendant TCP, but the compliance
controllers track `tool0` (crl-husky `config/ur_controllers.yaml:143`, `end_effector_link: "tool0"`). A pendant TCP
the monitor does not know of makes "Load Current", `hold()` and every streamed Cartesian move jump by that TCP, which
the 5 cm check cannot see. Handled by step 1 and `ArmConfig.pendant_tcp` (zero); the `tcp` chip shows a mismatch.

## Steps

1. **Reported TCP to flange** (`robot_interface/arm.py`).
   - `ArmInterface` gets its mounted `Tool` (`config.model.tools.get(f"{arm}_tool0")`).
   - `_on_tcp_pose` keeps the raw report (`tcp_raw_*`) and stores the flange pose:
     reported ∘ inverse(pendant TCP), in base_link. Done; since 16 Oct the pendant TCP is
     `ArmConfig.pendant_tcp` (zero), no longer `Tool.tcp`.
   - Proposal: rename `ArmState.tcp_position/orientation` to `flange_position/orientation`, since that is what
     the controller tracks. Readers: `plugins/robot_control/{plugin,arm_panel,markers,status}.py`,
     `world/signals.py`.
   - Update `doc/ur_frames.md` (chain: reported TCP ─(inverse pendant TCP)─► tool0).
   - Tests: a non-zero `Tool.tcp` gives the flange pose; hold and Load Current target the flange.
2. **`Tool` read fully from the tool URDF** (`tool_urdfs.py`).
   - `tool_spec` reads visual and collision elements separately into `Geometry(visual, collision)`.
   - Each tool URDF gets a `tcp` link on a fixed joint from the root; `tool_spec` reads its pose into `Tool.tcp`.
     A data edit, not code editing XML.
   - Test: the root link sits on `tool0` with no offset; visual and collision may differ.
3. **No stitching** (`config.py`, `tool_urdfs.py`).
   - Drop `stitch_tools` and `RobotConfig.urdf_file`. Keep one written copy of the robot URDF with mesh paths
     made absolute (attributes only), as `model.urdf`: the robot URDFs use `package://`, which PyBullet and
     compas_robots cannot resolve. Keep `resolve_mesh_path`.
   - `world/kinematics.py`, `robot_interface/robot.py` (frame checks), `test/test_kinematics.py`,
     `test/test_ur_frames.py`: read `config.model.urdf`. No code reads tool link poses.
4. **Drawing tools on the fast path** (`ui/visualization.py`, `ui/ghost.py`, `plugins/process/drawing.py`).
   - One helper: given a `FastViserUrdf` and a model's `tools`, add each tool's meshes (from `Tool.geometry`,
     visual and collision, toggled with the robot's) as children of the frame of the joint whose child is the
     flange. They then move in the same update as the links; no pose of their own, so they cannot lag.
   - Mesh building shared with `ui/scene_view.py` (one function from a `Geometry` shape to viser meshes).
   - Ghost `parts`: a tool belongs to the part of its flange's arm.
   - `process/drawing.py`: process robots and their tools through the same helper, replacing its own tool frames.
   - Test: the live robot draws from this tick's kinematics in step 8 (`monitor.py:_tick`), never a stored
     snapshot; tools appear under the flange frame.
5. **Live model vs planned model** (core, then `plugins/process/plugin.py`).
   - Core: one function comparing two `RobotModel`s (next to `robot_model`): links, joints, flanges; forward
     kinematics of every link at a few random joint values (method of `test/test_core_scene.py:47`); per tool:
     kind, `tcp`, the bounding box of its collision geometry in the flange frame. Returns the differences with
     their sizes (mm, deg). Core tests on Python 3.9 and 3.10.
   - Monitor: replace `tool_mismatches` with it, pairing `process_model(process, robot_id)` with `config.model`
     through `live_ids` (`plugins/process/steps.py:190`). Warn in the log, the panel chip and the planners' status
     lines; never refuse.
   - Fix the stale tool-id form `tools/AT3L` in core `scene.py:179` (`robot_parts`) and `:220`, `:346`.
6. **TCP check against the robot** (`plugins/health.py`, `world/checks.py`).
   - Per arm, every tick: the reported TCP against FK of the flange from `<arm>_base_link`, composed with
     `ArmConfig.pendant_tcp`. WARN above a tolerance. It catches URDF or calibration errors in the arm and a
     pendant TCP that is not `pendant_tcp`. It cannot check the arm mount or the mocap base.
7. **Docs.** `doc/refactor_rationale.md` (the stitching section), `doc/plugin_roadmap.md` ("stitched tools"),
   `doc/scene_refactor_plan.md` §5.3, `config.py` and `tool_urdfs.py` docstrings, test docstrings ("stitched
   config").

Order: 1 first (safety). Then 2 → 3 → 4. 5 and 6 need 2 (real TCPs) and can go in parallel with 4.

## Open questions

- The TCP values for the Robotiq and the scaffolding v3 tools: from the design's tool definitions (process
  `tools[*].tcp`) or measured? Planners only now; the pendants stay at tool0.
- `scaffolding_v1` has no model: Cartesian commands on such an arm assume TCP = `tool0`. Refuse instead?
- Tolerances: model comparison 1 mm / 0.1°, TCP check 2 mm / 0.5° are guesses.
- Rename `tcp_*` to `flange_*` in `ArmState` (step 1)? With the pendant at tool0 the values are tool0 either way.
- Articulated tools (moving fingers) would need the core `Tool` to grow joints, a schema change. Not now.

## Working in this worktree

- Core changes go in `external/scaffolding_assembly_core` here, following its `AGENTS.md`; its own tests run
  there. The venv's editable install points at the main checkout's submodule, so run this worktree's tests with
  `PYTHONPATH=external/scaffolding_assembly_core` to pick up core changes.
- The main checkout (`src/husky-assembly-teleop`, `jg/base-planner`) has uncommitted work, including
  `world/kinematics.py`, which step 3 also touches: expect a merge there.
