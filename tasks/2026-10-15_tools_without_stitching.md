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

## Before the TCP works: problems, settings, to-dos

Decided 2026-10-15: the pendant's TCP is the tool's real TCP (not the flange). `Tool.tcp` is the one value for
planners, the pendant, the monitor's tool0 conversion (`arm.flange_from_reported`) and the health `tcp` chip.

**Values (decide first)**
- [ ] TCP of each tool, in its tool0 frame: Cindy's `scaffolding_v3` left/right are (∓0.07, 0, 0.08) m in both
      local processes; confirm against the CAD or by measuring.
- [ ] The Robotiq's TCP: the processes disagree because the old export (`260814_RobArch_support_ik`) put it at the
      flange; the new design (`261008_2ModulesDemo`) puts it 166.1 mm out on the held bar's axis (to 0.007 mm, see
      the "Support Gripper TCP Check" artifact). Take the new design's: pendant X 0, Y 0, Z 166.1 mm, rotation vector
      (0, 2.2214, 2.2214) rad. Its z axis runs along the bar: confirm that convention for tool-frame jogging.
- [ ] Should an amber health `tcp` chip refuse Cartesian commands? Today it only warns, so a pendant changed before
      the tool URDFs lets Hold drive tool0 166 mm (see the "Arm Frames and the TCP" artifact).
- [ ] Single-arm robots get the *left* v3 tool (`tool_urdfs.TOOL_URDFS`): confirm its TCP holds there.
- [ ] `scaffolding_v1` has no model, so no TCP: Cartesian commands on such an arm assume TCP = tool0. Refuse them
      instead, or give it a model.
- [ ] The Robotiq's collision shape: ours (static, closed to 20 mm) or the processes' (37 mm apart)?
- [ ] Payload mass and centre of gravity per tool: not modelled anywhere yet, needed for the pendant (below).

**Data in this repo**
- [ ] Add a `tcp` link on a fixed joint to `data/tool_urdf/robotiq_2f_85.urdf`, `scaffolding_v3_left.urdf` and
      `scaffolding_v3_right.urdf` (`tool_urdfs.TCP_LINK`). ! Before any pendant is changed: until then the monitor
      assumes TCP = tool0.
- [ ] Fix the TCPs in the process sources (Rhino side) where they differ, so the process check goes green.

**On each robot (Alice, Belle, Cindy left and right)**
- [ ] Set the pendant's active TCP to the mounted tool's, one named TCP per tool in the installation.
- [ ] Set the payload and its centre of gravity to the mounted tool's: the F/T reading, compliance and protective
      stops depend on it.
- [ ] Check the safety configuration's tool settings (tool position, tool speed and force limits): confirm on the
      pendant that they follow the active TCP, and update them if they are set per tool.
- [ ] Swapping a tool (`-p tools:=...`) means switching the pendant's TCP and payload too. Option: the monitor sets
      both at startup from the model (URScript `set_tcp` / `set_payload` through the driver). ? Test first that it
      works while `ros_control.urp` runs without stopping it.

**Monitor**
- [ ] `ArmState.tcp_*` now holds tool0, not the tool tip: rename to `flange_*` (readers: `robot_control`
      plugin, arm panel, markers, status, `world/signals.py`), or keep the name and document it.
- [ ] `robot_control` labels say "TCP" but show and command tool0. Decide what the operator sees: the tool tip
      (compose with `Tool.tcp` on display, invert on command), or tool0 named as such.
- [ ] Cartesian moves turn about tool0 (`cartesian_move` interpolates tool0): to turn about the tool tip, plan the
      move at the tip and convert each sample to tool0.
- [ ] Recorded signals `<arm>_tcp` and `<arm>_tcp_error` now mean tool0: rename them or say so in their
      docstrings. Old recordings were made with pendant TCP zero, so they are tool0 too.
- [ ] `ArmConfig.cartesian_test_mode` docstring: add "after changing the pendant's TCP" to when to use it.
- [ ] Health `tcp` chip: confirm on each robot that it is green at several still poses; the tolerances
      (2 mm / 0.5°) are guesses. ? Pairing joint_states and tcp_pose by stamp would let it judge while moving.
- [ ] Fake hardware publishes no TCP (the monitor ignores an all-zero pose), so the chip stays amber and Cartesian
      commands are refused in the simulator, as before.

**Onboard controller (crl-husky, decision)**
- [ ] The compliance controller's `end_effector_link`, `compliance_ref_link` and `ft_sensor_ref_link` are tool0
      (`config/ur_controllers.yaml`): stiffness and compliance act about tool0, not the tool tip. Insertion may want
      compliance at the TCP, which needs a TCP link in the onboard robot description. Decide before compliant
      execution (`bar_action`).

**First run on a robot**
1. Tool URDFs have their `tcp` links; the pendant has the same TCP and payload.
2. Start with `cartesian_test_mode` on that arm; the health `tcp` chip must be green at three or more still poses.
3. Log line of the first Cartesian target ("first target vs reported tool0"): difference under 1 mm.
4. Jog in the pendant's tool frame: it must turn about the tool tip.
5. Turn test mode off; a short Cartesian move, then Hold.

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
- The TCP on each teach pendant is set to match the mounted tool.

## Safety first: the reported TCP is not `tool0`

The driver's `tcp_pose` (crl-husky `onboard/rate_limiter.py:37`) includes the pendant TCP, but the compliance
controllers track `tool0` (crl-husky `config/ur_controllers.yaml:143`, `end_effector_link: "tool0"`). Today the
monitor treats the reported TCP as `tool0` (`robot_interface/arm.py:_on_tcp_pose`, `doc/ur_frames.md`). With a real
tool TCP on the pendant, "Load Current", `hold()` and every streamed Cartesian move would send the TCP pose as the
`tool0` target: a jump by the TCP offset (~15 cm for the Robotiq) that the 5 cm check cannot see.

! Do step 1 before any pendant is set to a non-zero TCP. Until then, keep pendant TCPs at zero.

## Steps

1. **Reported TCP to flange** (`robot_interface/arm.py`).
   - `ArmInterface` gets its mounted `Tool` (`config.model.tools.get(f"{arm}_tool0")`).
   - `_on_tcp_pose` keeps the raw report (`tcp_raw_*`) and stores the flange pose:
     reported ∘ inverse(`Tool.tcp`), in base_link. A bare arm or a tool without a model: TCP = `tool0`.
   - Proposal: rename `ArmState.tcp_position/orientation` to `flange_position/orientation`, since that is what
     the controller tracks. Readers: `plugins/robot_control/{plugin,arm_panel,markers,status}.py`,
     `world/signals.py`.
   - Update `doc/ur_frames.md` (chain: reported TCP ─(inverse tool TCP)─► tool0).
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
     `Tool.tcp`. WARN above a tolerance. It catches URDF or calibration errors in the arm and a pendant TCP that
     does not match the mounted tool. It cannot check the arm mount or the mocap base.
7. **Docs.** `doc/refactor_rationale.md` (the stitching section), `doc/plugin_roadmap.md` ("stitched tools"),
   `doc/scene_refactor_plan.md` §5.3, `config.py` and `tool_urdfs.py` docstrings, test docstrings ("stitched
   config").

Order: 1 first (safety). Then 2 → 3 → 4. 5 and 6 need 2 (real TCPs) and can go in parallel with 4.

## Open questions

- The TCP values for the Robotiq and the scaffolding v3 tools: from the design's tool definitions (process
  `tools[*].tcp`) or measured? They must equal what the pendants are set to.
- `scaffolding_v1` has no model: Cartesian commands on such an arm assume TCP = `tool0`. Refuse instead?
- Tolerances: model comparison 1 mm / 0.1°, TCP check 2 mm / 0.5° are guesses.
- Rename `tcp_*` to `flange_*` in `ArmState` (step 1)?
- Articulated tools (moving fingers) would need the core `Tool` to grow joints, a schema change. Not now.

## Working in this worktree

- Core changes go in `external/scaffolding_assembly_core` here, following its `AGENTS.md`; its own tests run
  there. The venv's editable install points at the main checkout's submodule, so run this worktree's tests with
  `PYTHONPATH=external/scaffolding_assembly_core` to pick up core changes.
- The main checkout (`src/husky-assembly-teleop`, `jg/base-planner`) has uncommitted work, including
  `world/kinematics.py`, which step 3 also touches: expect a merge there.
