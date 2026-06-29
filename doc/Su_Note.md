# Su_Note — Learning Notes for the Husky calibration/teleop code

Plain-language notes on the files touched this month (June 2026), for someone who
**knows basic Python and basic forward-kinematics (FK) but has never used PyBullet or motion capture**.
Each section is one file; bullets say *what the code does* and *where* (`file:line`), and explain any
PyBullet / mocap / FK term the first time it appears.

---

## 0. Core concepts primer (read this first)

These ideas come up everywhere below.

- **Pose** = where something is in 3D = `(position, orientation)`.
  - `position` = `[x, y, z]` in metres.
  - `orientation` = a **quaternion** `[x, y, z, w]` — 4 numbers that encode a rotation (a compact,
    gimbal-lock-free alternative to roll/pitch/yaw). You rarely read quaternions by eye; you feed them
    to helper functions.
  - In code a pose is usually the tuple `(position, quaternion)`.
- **Forward kinematics (FK)**: given the robot's joint angles, compute where each part (link) and the
  tool tip end up in space. PyBullet does this for us.
- **Frame / transform**: a "frame" is a local coordinate system (e.g. the robot base, the tool, the
  mocap world). A "transform" `A_from_B` tells you where frame B sits, expressed in frame A. To move a
  point/pose from one frame to another you multiply transforms.
- **Motion capture (mocap) = OptiTrack/Motive**: ceiling cameras track reflective markers and report
  the live pose of each **rigid body** (a named cluster of markers, e.g. the husky base, a calibration
  tool). The poses stream into our program continuously.
- **Axis convention — "y-up" vs "z-up"**: Motive reports poses with **Y pointing up**. Our simulator
  (and Rhino) use **Z pointing up**. So every incoming mocap pose is re-labelled. We support two
  conventions: `'rhino'` (current default) and `'rotated'` (legacy). They are *different relabelings*
  of the same physical pose — using the wrong one rotates everything.

### PyBullet helpers you will see (all prefixed `pp.`, from `pybullet_planning`)

- `pp.load_pybullet(urdf)` — load a robot model (a URDF file) into the physics world.
- `pp.set_joint_positions(robot, joints, angles)` — move the robot's joints (this is the input to FK).
- `pp.get_link_pose(robot, link)` — read where a link ended up = the **FK result**, returned as `(position, quaternion)`.
- `pp.set_pose(robot, pose)` — place the whole robot base at a pose.
- `pp.multiply(A, B)` — chain two poses/transforms (compose them). Order matters.
- `pp.invert(A)` — reverse a transform (go the other way).
- `pp.matrix_from_quat(q)` — turn a quaternion into a 3×3 rotation matrix (whose columns are the X, Y, Z axis directions).
- `pp.draw_pose(pose)` / `pp.add_text(...)` — draw little RGB axis arrows / labels in the 3D viewer for debugging.
- `pp.connect(use_gui=False)` / `pp.disconnect()` — open/close the physics world (GUI on = 3D window; off = headless/no window).

### The calibration pipeline at a glance

We want the fixed transform between the **mocap's idea of the husky base** and the **robot's own base
frame** (`base_mocap_from_base_footprint`). The scripts run in order (or via `run_calibration_pipeline.py`):

`0_circle_fitting` → `1_calibration_analysis` → `2_convert_and_visualize_transformation` → `3_verify_calibration`,
with `4_punch_validation` as an extra accuracy check. The idea: spin one joint at a time, watch the
tool trace a **circle**, the circle's axis = that joint's axis; combine the first two joint axes to
locate and orient the base; then check the result against held-out data.

---

## 1. data/calibration_data/0_circle_fitting.py — fit circles to joint sweeps

- **Purpose**: for each joint sweep (j0 = shoulder-pan, j1 = shoulder-lift), read the recorded tool
  ("flange") poses and fit a 3D **circle**. When a single joint rotates, the tool traces a circle whose
  centre lies on the joint axis and whose normal (the perpendicular direction) *is* the joint axis
  direction. (`process_data_batch`, top of file.)
- **Putting mocap into the base frame** (`~line 86`): `base_from_flange = pp.multiply(pp.invert(base_mocap_pose), flange_mocap_pose)`.
  In words: "undo the base pose, then apply the flange pose" → the tool position expressed relative to
  the husky base instead of the mocap world. This is the standard "change of frame" trick with
  `invert` + `multiply`.
- **Circle fit** (`~line 92`): a helper finds the best circle (centre + normal) through the cloud of
  measured points. The **normal** is the joint axis; the **centre** is a point on it.
- **Fit error** (`~lines 96–110`): each measured point is projected onto the fitted circle; the
  leftover distance is the error — it tells you how clean the sweep was (collisions/noise show up here).
- **`traj_label()`** (`~line 28`): small helper that turns a filename like `..._J0_traj3_...json` into a
  short plot label `J0_T3` using a regular expression. Pure string bookkeeping, no robotics.

## 2. data/calibration_data/1_calibration_analysis.py — turn circles into the base frame

- **Purpose**: take the fitted circles from step 0, fit a straight **line** through each joint's circle
  centres (that line *is* the joint axis), then build the robot base frame from the two axes.
- **Read fixed numbers from the URDF** (`parse_base_offset_from_urdf ~line 60`, `parse_joint_axes_from_urdf ~line 89`):
  a URDF is the robot description (an XML file). These read the nominal joint axis directions and the
  small offset from joint 0 to the base origin. Note: `parse_joint_axes_from_urdf` reads each joint's
  *local* axis only (both read `[0,0,1]`) — it does **not** know the real mounted direction; see the
  PyBullet trick below.
- **Fit line through circle centres** (`~lines 430–481`): all the j0 circle centres lie on the j0 axis,
  so a line through them recovers that axis as a 3D line (a point + a direction).
- **Locate the base origin** (`~lines 484–634`): joint 0 and joint 1 axes meet near the shoulder. The
  code intersects two planes built from those axes to find that meeting point, then slides along the
  joint-0 axis by the URDF offset to reach the base origin.
- **Fitted-line SIGN disambiguation** (`~lines 528–560`, **the 2026-06-26 fix**): a fitted line has no
  built-in direction — the fitter may return the axis pointing either way. If the sign is wrong the
  base frame ends up rolled 90° (robot appears lying on the floor) and validation explodes. Fix: flip
  each fitted axis so it agrees (positive dot product) with the **URDF-nominal** axis direction. See
  `tasks/2026-06-26_calibration_base_frame_sign_fix.md`.
- **Getting the *true* nominal axes with PyBullet FK** (`main`, `~lines 903–924`): because the arms sit
  on a ~45°-tilted bracket, joint 0 is **not** vertical. To get the real axis directions we load the
  URDF in PyBullet, read each joint's axis and its child link's rotation, and express them in the base
  frame. This is why we use FK here instead of trusting the flat URDF text.
- **Build the base frame** (`~lines 640–694`): with axes correctly signed, joint-0 axis → Z, joint-1
  axis → Y, and X = Y×Z (cross product), re-orthogonalised. After the sign fix the left/right arm cases
  are identical, so they were merged into one block (old per-arm code kept commented for reference).

## 3. data/calibration_data/2_convert_and_visualize_transformation.py — finish + visualise the transform

- **Purpose**: convert the result of step 1 (`base_mocap_from_arm_base_link`) into the transform we
  actually want (`base_mocap_from_base_footprint`) and show it in the 3D viewer.
- **Chain the transforms** (`~lines 98–103`):
  `base_mocap_from_base_footprint = pp.multiply(base_mocap_from_arm_base_link, arm_base_link_from_base_footprint)`.
  The second piece (`arm_base_link_from_base_footprint`) is fixed by the robot's URDF and read with
  `pp.get_relative_pose(...)`. Composing the measured part with the fixed part gives mocap → robot base.
- **Visual check** (`~lines 116–145`): `pp.set_pose` places the robot at the computed pose, then
  `pp.draw_pose` / `pp.add_text` draw the frames so you can eyeball that the robot stands upright.
  Tip: a healthy `base_mocap_from_base_footprint` has roll/pitch ≈ 0° (only a yaw); roll or pitch ≈ 90°
  means a sign/frame bug.

## 4. data/calibration_data/3_verify_calibration.py — score the calibration

- **Purpose**: measure how good the calibration is, using held-out "validation" recordings.
- **The check** (`compute_tool0_flange_offset ~lines 65–154`): for each validation sample, place the
  robot using the calibration (`world_from_footprint = pp.multiply(base_mocap_pose, base_mocap_from_base_footprint)`),
  set the recorded joint angles, FK the tool pose with `pp.get_link_pose`, then compute the leftover
  offset between FK-tool and the mocap-measured tool: `pp.multiply(pp.invert(world_from_tool0), flange_mocap_pose)`.
  If calibration is perfect this offset is the **same constant** for every sample.
- **Metric = consistency, not absolute distance** (`analyze_results ~line 267`): it takes the spread of
  that offset around its own mean. Small spread = good. (Our right-arm bug showed up as a 500 mm spread;
  after the fix it was ~1 mm.)
- **CDF plots & 95th percentile** (`~lines 433–467`): sort the per-sample errors and read off "95% of
  samples are under X mm/deg". Goal: position 95% < 5 mm, angle 95% < 0.3°.
- **Quaternion → axes for the angle check** (`~lines 279–296`): `pp.matrix_from_quat(q)` gives a 3×3
  matrix whose columns are the X/Y/Z axes; comparing each sample's axes to the mean axis (via `arccos`
  of a dot product) gives the orientation error in degrees.

## 5. data/calibration_data/4_punch_validation.py — physical tip-touch accuracy

- **Purpose**: a separate accuracy test where a pointed "punch" tool touches the same spot several
  times; if calibration + FK are right, the computed tip lands in the same world position every time.
- **Position consistency** (`~lines 127–146`): computes how far each take drifts from the mean tip
  position, and reports mean/std/max/95th-percentile in mm.
- **Pick which arm to analyse** (`~lines 96–103, 505–522`): dual-arm data can contain both arms; the
  `punch_validation.arm` field in `config.yaml` selects left/right so you don't mix them.
- **Tool-offset sanity check** (`~lines 105–124`): the test assumes a fixed tool0→tip offset; if that
  changed between takes the analysis is meaningless, so it stops early with a clear error.

## 6. data/calibration_data/5_compare_base_pose_files.py — diff two base-pose recordings

- **Purpose**: compare two recordings of the husky base pose (e.g. before vs after a change) and report
  how much the position and orientation differ.
- **Relative rotation via quaternions** (`~lines 41–63`): `q_rel = q_B ⊗ conjugate(q_A)` gives "how much
  B is rotated relative to A". The conjugate (negate x,y,z) is a quaternion's inverse.
- **Human-readable angles** (`~lines 66–91`): converts the relative rotation to roll/pitch/yaw degrees
  so you can see which axis moved.
- **Single-number summaries** (`~lines 160–182`): translation as one distance `sqrt(dx²+dy²+dz²)`, and
  rotation as the shortest angle (0–180°), so results are easy to threshold.

## 7. data/calibration_data/run_calibration_pipeline.py — run steps 0→3 in order

- **Purpose**: convenience runner that calls scripts 0,1,2,3 one after another and stops if any fails
  (`subprocess.run`, `~lines 30–51`). It does **not** run step 4 (punch).
- **Choosing the dataset**: it does *not* switch datasets itself — set
  `config_loader.DEFAULT_DATE_FOLDER` first (it only prints a reminder, `~lines 72–77`).
- **Summary** (`~lines 102–121`): prints which steps passed and how long each took.

## 8. data/calibration_data/config_loader.py — one place for settings & robot names

- **`DEFAULT_DATE_FOLDER`** (`~line 21`): the single constant that picks which dated folder all scripts
  read/write when no folder is passed. Change it to switch datasets without editing every script.
- **`load_config()`** (`~line 29`): reads `config.yaml` inside the chosen date folder (robot, arm,
  which batches, etc.) so paths/robot types aren't hard-coded.
- **Robot-name helpers** (`~lines 87–133`): `get_joint_names`, `get_tool0_link_name`,
  `get_arm_base_link_name`, etc. centralise the "is this the dual-arm 0806 or a single-arm robot, and
  which arm?" logic instead of scattering `if robot_name == '0806'` everywhere.

## 9. data/calibration_data/convert_to_rhino.py — relabel a calibration file's axis convention

- **Purpose**: take a calibration file saved under the old `'rotated'` convention and re-tag it for the
  `'rhino'` convention. The physical calibration is unchanged — only the y-up→z-up labelling differs.
- **The actual fix** (`~lines 1–20, 51–58`): position is left as-is; the stored orientation quaternion
  is right-multiplied by a −90° rotation about Z to absorb the yaw difference between the two
  conventions. ("Right-multiply" applies the extra turn in the object's own frame.)
- **Self-describing output** (`~lines 74–89`): writes a `mocap_axis_convention: 'rhino'` tag so the live
  app knows which convention the file uses (and can fall back to the untagged file if missing).

## 10. data/calibration_data/logging_utils.py — coloured logs to console + file

- **`setup_logger()`**: gives every script colourful console messages (errors red, info green) plus a
  plain-text log file. Helper-level code, no robotics.
- **Two small idioms**: it checks `if logger.handlers:` to avoid printing every message twice if set up
  more than once; and uses `os.makedirs(..., exist_ok=True)` to create the log folder without erroring
  if it already exists. The file handler omits colour codes (they'd corrupt a text file).

## 11. data/calibration_data/export_mocap_cameras.py — save Motive camera poses for Rhino

- **Purpose**: pull the **camera** positions/orientations that Motive recorded in a take and write them
  to CSV+JSON so you can recreate the real camera layout in Rhino.
- **Standalone** (plain Python, no PyBullet/numpy). Run with a take file or `--latest` to grab the
  newest (`main`, `_find_latest_take ~lines 67–72`).
- **Frame conversion** (`~lines 46–64`): converts each camera pose from Motive y-up to Rhino z-up,
  including reordering the quaternion components — the same y-up→z-up idea as the rest of the codebase.
- **Units**: positions are exported in metres; the Rhino-side importer scales them (commonly ×100 to cm).

## 12. data/calibration_data/export_robot_skeleton.py — FK the arm and export link positions

- **Purpose**: for each calibration point, run FK and export where every arm link sits, so a Grasshopper
  viewer can draw the "skeleton" (base → shoulder → … → tool0).
- **Standalone Python + PyBullet.** The FK loop (`~lines 97–105`): `pp.set_joint_positions(...)` then
  `pp.get_link_pose(robot, link)` for each link; positions ×1000 to store millimetres.
- **Anchoring to the measured flange** (`~lines 90–101`): it places the robot so the URDF flange link
  lands on the mocap-measured tool pose. This is approximate (it ignores the small marker-vs-URDF
  offset that calibration solves) and is meant only for a quick visual.
- **Output frame**: everything is in the Motive world frame ("mocap_world_mm"), the same frame the
  calibration uses.

## 13. data/calibration_data/robot_skeleton_viewer.py — view the skeleton (Rhino or terminal)

- **Purpose**: read the skeleton JSON from file 12 and either feed geometry to **Grasshopper** or print
  a summary in a terminal. Toggle with `RUN_MODE` (`~line 24`).
- **Runs inside Rhino (GhPython)**: GhPython is CPython *inside* Rhino with no numpy/PyBullet, so this
  script uses only the standard library and `Rhino.Geometry`. That's why it re-implements small bits by
  hand.
- **Points + bones** (`~lines 84–90`): the 8 link positions become points; consecutive points are
  joined by line "bones" (skipping near-zero-length ones).
- **Quaternion order gotcha** (`~line 96`): PyBullet quaternions are `[x,y,z,w]` but Rhino wants
  `[w,x,y,z]` — the code reorders them when building a Rhino `Plane` for the tool frame.

## 14. data/calibration_data/rhino8_import_outliers.py — show calibration circles + outliers in Rhino

- **Purpose**: read the analysis JSON and draw, in Grasshopper, the fitted circles plus the **outlier**
  points (samples that sit far from their fitted circle — likely noise/collisions).
- **Pure Python, no numpy** (so it runs in GhPython): it hand-codes vector maths and quaternion
  rotation (`_sub`, `_cross`, `_dot`, `qrot`, `~lines 52–81`).
- **Base frame → Motive world** (`~lines 131–145`): circle centre/normal are stored in the robot base
  frame; the script maps them into the Motive world using a reference base pose, then draws them.
- **Outlier threshold** (`~line 161`): flag points above either the per-curve mean error or a fixed mm
  value (`THRESHOLD`).

## 15. data/calibration_data/visualize_calibration_outliers_rhino.py — stage analysis files to Drive

- **Purpose**: a tiny file-copier — it copies each `{date}/{batch}/{batch}_analysis.json` into the
  shared Google Drive folder (under a dated subfolder) so the Rhino machine can read it (`stage_batch ~lines 36–47`).
- **No computation / no re-fitting**: it reuses the circle fit already stored in the analysis JSON;
  the Rhino viewer (file 14) reads from there. Single source of truth, less duplication.

## 16. husky_assembly_teleop/__init__.py — package-wide paths & the calibration date

- **`DATA_DIRECTORY`** (`~lines 8–52`): tries several locations (installed ROS package, source folder,
  cwd) so the code runs both deployed and standalone.
- **Hard-coded Google-Drive paths** (`~lines 53–55`): `DESIGN_DATA_DIRECTORY` etc. point at the
  Insync-mounted Drive and start with `/home/su` — **edit these when moving to another PC** (the mount
  path is machine-specific).
- **`CALIBRATION_DATE`** (`~line 56`): the dated dataset the **live app** uses. Every part of the app
  imports this one name, so bumping it repoints everything at a new capture. (This is separate from
  `config_loader.DEFAULT_DATE_FOLDER`, which only the offline calibration scripts use.)

## 17. husky_assembly_teleop/husky_world.py — build the sim world (calibration parts)

- **Pick robot by `ROS_DOMAIN_ID`** (`init`, `~lines 178–212`): three robots (Alice/Belle/Cindy) run in
  parallel terminals; the environment variable `ROS_DOMAIN_ID` selects the right mocap id, gripper, and
  dual-arm settings from a dict — no code edits to switch robot.
- **Choose end-effectors by mode** (`~lines 215–226`): normal vs calibration vs punch-validation each
  load different tools onto the arms.
- **Load the base calibration file** (`~lines 228–256`): prefers the `_rhino`-tagged calibration file
  (see file 9) and falls back to the untagged one if missing — so the live robot is placed using the
  calibration we computed.
- **`TrackedObject(...)`** (`~lines 326–340`): registers a mocap **rigid body** (e.g. id 1013 = left
  calibration tool) so its live pose streams into the sim each frame. Arguments are name, mocap id,
  a position/orientation offset (here zero = identity), and a draw size. `assign_calibration_tool_to_robot(0, 0, name)`
  attaches it to robot 0, arm 0 (left).

## 18. husky_assembly_teleop/husky_monitor.py — main node (mocap + run-mode parts)

- **Run-mode flags** (`~lines 78–89`): `USE_MOCAP=1` streams real mocap poses (vs simulating them);
  `FAKE_HARDWARE=0` talks to the real robot (vs fake interfaces). They're independent — set both to live
  values for a real session, both to test values at the desk. Keep them consistent by hand.
- **`MOCAP_AXIS_CONVENTION`** (`~line 99`): chooses the y-up→z-up relabeling, `'rhino'` (default) or
  `'rotated'` (legacy). See the primer.
- **Per-frame conversion** (`receive_rigid_body_frame ~line 4069`): every incoming mocap pose is passed
  through `mocap_pos_y_up_to_z_up` / `mocap_quat_y_up_to_z_up` (defined in `utils.py`) and cached, so the
  rest of the program only ever sees z-up poses.
- **`collect_mocap_camera_data()`** (`~line 546`): button-triggered method that snapshots the mocap
  camera poses, converts them to z-up, and saves JSON+CSV to the Drive folder (`MOCAP_CAMERA_EXPORT_DIR`).
- **`CALIBRATION_STATE_SETS`** (`~lines 72–79`): maps arm index (0=left, 1=right) to the folder of saved
  robot states/trajectories used in calibration mode, so each arm pulls its own files.

---

*Maintenance: when you change one of these files, update its section here. Keep new explanations at the
same beginner level (assume basic Python + FK; explain PyBullet/mocap terms on first use).*
