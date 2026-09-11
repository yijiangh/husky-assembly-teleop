# Grasp-force calibration (Robotiq slip test)

How hard does the Robotiq 2F-85 actually hold a part? This is the tool that
answers it, and where its data lands.

The force numbers come from the **arm's own wrist force/torque sensor**, not from a
separate force plate: the gripper holds the part, the other arm (or a hand) pulls
or pushes on it, and the wrist FT reading at the moment the part slips is the grip
force. Nothing else in the rig measures force.

| | |
|---|---|
| Tool | `husky_assembly_teleop/grasp_calib_monitor.py` |
| Run | `ros2 run husky_assembly_teleop grasp_calib_monitor` |
| Preflight | `ros2 run husky_assembly_teleop grasp_calib_monitor --check` |
| Re-plot | `ros2 run husky_assembly_teleop grasp_calib_monitor --replot [FOLDER ...]` |
| Data | `/home/su/Insync/2025-03 Husky Assembly/data_experiment/robotiq_grasp_calibration/` |

The output root is `EXPERIMENT_DATA_DIRECTORY` (`husky_assembly_teleop/__init__.py:54`)
plus `robotiq_grasp_calibration`; change the drive there, not in the recorder.

## What it records

Both arms, one aligned sample per 20 Hz tick (`TICK_PERIOD_S = 0.05`):

- `wrench_raw` -- the UR `ft_sensor_wrench`, six numbers in the **tool0** frame,
  **raw**: never zeroed and never baseline-subtracted.
- `q` -- the six joint angles in driver order.
- `ee_pos`, `ee_quat_xyzw`, `ee_euler_deg` -- tool0 in `<side>_ur_arm_base_link`,
  from the calibrated analytic FK (`ssik_inprocess.fk`). Note the frame: NOT
  `*_base_link_inertia`, which is 180 degrees of yaw away.

It only ever **reads** the robot. Both pendants stay in LOCAL mode for the whole
experiment -- the UR driver's joint-state and force-torque broadcasters are
"consistent controllers" and keep streaming at 100 Hz regardless. `multi_arm_safety_sync`
will spam "not remote" errors meanwhile; they are harmless.

## Running a take

1. Build and source as usual, with the gripper/arm stack up on the husky:

   ```bash
   cd ~/ros2_ws && source venv/bin/activate
   python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop
   source install/setup.bash
   ```

   The shell also needs Cindy's DDS settings, or it will see no topics at all:
   `ROS_DOMAIN_ID=86`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
   `CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml` (see `doc/rtde_network_setup.md`).

2. `... grasp_calib_monitor --check` first. It listens for 3 s and prints a rate for
   each of the four topics plus one FK result; `[check] PASS` means reading works.
   `NO DATA` on every line is the ROS environment, not the robot.

3. `... grasp_calib_monitor` opens the control panel, two live wrench windows
   (left | right, force and torque sharing axis scales so the arms compare
   directly), a right-arm joint stream, and a PyBullet view of the live model.
   The wrench plots run from launch, recording or not.

4. Type an experiment name, click **Start recording** -- it starts immediately.
   Free-drive the pushing arm on the pendant and load the grasped part until it
   slips. Click **Stop & save** (or **Discard**). Stopping is manual, always:
   there is deliberately no auto-stop.

5. The take is written to its own folder,
   `<YYYYMMDD-HHMM>-<experiment name>/record.json` + `plots.png` (the whole take
   as a static figure).

The **Zero FT** buttons call the UR `zero_ftsensor` service, which needs that arm's
driver running and the pendant in REMOTE -- in LOCAL mode the call is simply
ignored. That is fine: record a few quiet seconds before loading the part and
subtract that as the baseline in post-processing. It is why the wrench is stored raw.

## record.json

```
schema_version 1, experiment, recorded_at, robot, sample_period_s, n_samples,
frames { wrench, fk, joint_order, euler, position_unit },   <- how to read the numbers
samples {
  t: [s since Start],
  left  { wrench_raw, q, ee_pos, ee_quat_xyzw, ee_euler_deg },
  right { ... }
}
```

Every list is plain JSON (no numpy), so `json.load` is all it takes.

## Re-plotting a saved take

`--replot` needs no robot and no UI. With no arguments it walks every take under
the output root and renders `plots.png` for the ones that have none; with folder
arguments it re-renders exactly those, overwriting the figure.

```bash
cd ~/ros2_ws && source venv/bin/activate && source install/setup.bash
python3 -m husky_assembly_teleop.grasp_calib_monitor --replot
```

It also repairs a take whose columns came out ragged (below), rewriting
`record.json` only when values actually had to be dropped, and prints what it
dropped. It is idempotent: a second run finds nothing to repair.

A take that is not from this recorder -- a raw RTDE capture, i.e. an `.npz` with
`t` and `wrench` next to a hand-written `record.json` -- is plotted too, with
only the panels its data supports.

## Watch out for

1. **Takes from before and after 2026-09-09 are not comparable.** Until that day
   the husky's Robotiq driver ignored its multipliers and gripped at the full
   235 N (and crawled at 15 % speed). `scripts/husky/patch_robotiq_driver.py`
   fixed it and set the force multiplier to 0.25, about 74 N. The five takes of
   2026-09-04 are therefore *pre-fix*; anything recorded later is not.
2. **`stalled` in a gripper result does not mean "holding something".** An empty
   close stops at 0.789 rad and still reports `stalled`; judge a grasp by the
   final position instead.
3. **Takes saved before 2026-09-11 can be one sample ragged.** Saving used to
   hand `json.dump` the live buffers while the 20 Hz tick was still appending to
   them, so a sample could land mid-write: `t` was serialized before it and the
   arm columns after, leaving every arm column one value longer than `t` (in the
   2026-09-11 take, even one arm's own columns disagreed). Any reader had to
   cope with columns of different lengths, and `plots.png` just failed -- which
   is why three takes had no figure. Fixed: `_save` now stops sampling first and
   serializes its own aligned copy (`align_samples`). The three affected takes
   were repaired in place with `--replot` on 2026-09-11; the dropped values are
   the last 50 ms of each, and nothing else in those files changed.
4. The recorder imports `ssik_inprocess` from the `husky_assembly_tamp` submodule,
   so that submodule must be checked out and `ssik` installed in the venv.
5. A plotting failure never loses a take -- `record.json` is written first and
   `plots.png` only warns if it fails.

## What is already on the drive

```
robotiq_grasp_calibration/
  20260904-1634-global-z_8cm/          <- five pre-fix takes, punch test:
  20260904-1641-global-z_0mm/             left arm holds the box, right arm's
  20260904-1649-globalx_80mm/             punch pushes at a named point
  20260904-1654-globalx_20mm/
  20260904-1724-rotation_global-y_grasped/
  20260909-pull-test-left/             <- post-fix hand pull test (see below)
```

`20260909-pull-test-left` is **not** from this recorder: it is an ad-hoc RTDE
capture (`pull_test_left.npz` with `t`, `wrench`, `baseline`, plus a hand-written
`record.json` describing it). Its result is the current working number: a 22 mm
stool leg in the left gripper held roughly 50-60 N of pull, peaking at 62.4 N over
idle, with the first give a 36 N drop at t = 6.16 s. That is comfortably above the
10-30 N an insertion pushes with, which is why the force multiplier stays at 0.25.

To redo it properly -- repeatable, both arms logged, plotted -- use the recorder
above rather than another one-off script.

`20260911-1535-global-y_push` is the newest take and the first one recorded after
the mount and coupling corrections; both arms see the push (the left tool0 force
reaches about -80 N at t = 45 s).

Every folder listed above has a `plots.png`. There is also a
`robotiq_grasp_calibration (copy)/` tree beside this one, an Insync duplicate
holding the same seven takes; it was left untouched.
