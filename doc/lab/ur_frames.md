# UR arm frames

**Rule:** in our URDFs, each arm's `<arm>_base_link` must be the frame the
robot's own controller calls `base_link`. The robots run the stock
`ur_description`, where the joints below it are fixed:

| Joint | Origin |
|---|---|
| `<arm>_base_link` → `<arm>_base_link_inertia` | `xyz 0 0 0`, `rpy 0 0 π` |
| `<arm>_base_link` → `<arm>_base` | `xyz 0 0 0`, `rpy 0 0 π` |

Calibration goes elsewhere: how the arm sits on the husky into the mount joint
*above* `<arm>_base_link` (`arm_mount_joint`, `right_arm_mount_joint`, ...),
the UR's own calibration into the six revolute joints.

**Why:** `husky_urdf` had the 90° mount turn in the joints below
`<arm>_base_link` instead: on Alice and Belle in both (`bcd5c07`, 2023-07-05),
on Cindy's right arm in its `base` joint (`2aababb`, 2025-06-26). The arm was
still drawn in the right place, but `ur_arm_base_link` was 90° off the
controller's `base_link`. Cartesian commands computed in it swung 0804
(e-stop, 2026-09-29). The old monitor never noticed: it rotated the reported
TCP by a hard-coded −90° (one arm) or −180° (Cindy).

**Now:**
- The reported TCP (UR Base frame) is turned into the controller's frame by the
  stock 180° (`robot_interface/ur_frames.py`), never through our URDF. Targets
  reach that frame from the husky frame through the URDF's fixed mount chain,
  which is only right because `<arm>_base_link` is stock (the rule above).
- **The pendant's TCP stays at `tool0` (zero) on every arm**, so the driver's
  `tcp_pose` is `tool0` itself, which is what the compliance controller moves
  (`end_effector_link` in crl-husky's `ur_controllers.yaml`) and what the force
  reading is labelled in (`ft_sensor_ref_link`). `arm.flange_from_reported`
  takes the pendant's TCP (`ArmConfig.pendant_tcp`, zero by default) off the
  report. The tool's own TCP, `Tool.tcp` (the `tcp` link of its URDF in
  `data/tool_urdf/`), is for planners and the process check only; it never
  touches robot I/O.
  ! A TCP set on a pendant without changing `pendant_tcp` puts every Cartesian
  command off by that TCP: the health `tcp` chip turns amber. Before anyone
  sets one, check whether the UR reports the force about the TCP (open
  question in `tasks/2026-10-15_tools_without_stitching.md`).
- Every URDF in `husky_urdf` is fixed in place, with
  `scripts/fix_ur_base_frames.py`: every link except a mis-set `<arm>_base`
  stays where it was. Run it again if a file comes back turned, e.g. after a
  calibration rewrites one. ! Old process exports still carry the turned
  frames; never bump `husky_urdf` on `master` (the old monitor's −90° assumes
  the turned files).
- A URDF that breaks the rule still *looks* right in the viewer but moves wrong.
  So it is caught three times: `test/robot_interface/test_ur_frames.py` fails for every arm
  URDF in `husky_urdf` that breaks it, at startup the monitor logs an error and
  refuses that arm's Cartesian targets (Hold still works), and while connected
  the health `tcp` chip would show tool0 90° off.
- **The arm's own controller has its own URDF**: the stock `ur_description`
  built on the robot with that arm's UR factory calibration (`kinematics_params_file`).
  It starts at `base_link` and ends at `tool0`, knows nothing of the husky,
  and is fixed. Ours must agree with it from `<arm>_base_link` down: the two
  180° joints above, and the six joint origins, which hold the same factory
  calibration. The monitor applies the arm's calibration file from crl-husky
  (`config/robots/<serial>/ur_calibration*.yaml`) to its copy of the URDF at
  startup: nothing changes if the URDF is up to date, otherwise it logs the
  difference boldly, shows a `URDF` chip on the health panel, and uses the
  calibration.

**Chain from the world to the TCP** (`robot_interface/frames.py`):

```
world ─(mocap)─► base_footprint ─(URDF fixed)─► base_link [husky] ─(URDF fixed, mount calibration)─► <arm>_base_link ─(reported TCP)─► tool0 (pendant TCP zero) ─(Tool.tcp, planners only)─► tool tip
```

The Cartesian sliders are in the husky's `base_link`. `arm.to_husky` /
`arm.from_husky` and `robot.to_world` / `robot.from_world` convert along it.
The URDF root `world_link` is the same frame as `base_footprint` (fixed
identity joint `world_footprint`), not the mocap world.
