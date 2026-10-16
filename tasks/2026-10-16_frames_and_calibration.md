# Arm frames, UR calibration and the mocap relay (2026-10-16)

Status: **committed 16 Oct, not pushed.** Four repos touched; quick, slow and linter tests pass in teleop,
the core's tests pass, crl-husky's linter findings went down (its 2 failing linter tests failed before too).
The calibration pipeline (`data/calibration_data/`, Drive sessions) is unchanged.

## What changed

### husky_urdf (`data/husky_urdf`, branch `main`)

- The turned UR base frames are fixed in place in all 8 arm URDFs (`scripts/fix_ur_base_frames.py`, now in place):
  - one-arm, from `bcd5c07` (2023-07-05): `husky_ur5_e_no_base_joint`, `_Alice_Calibrated`, `_Belle_Calibrated`;
  - Cindy's right arm `base` joint, from `2aababb` (2025-06-26): `husky_dual_ur5_e_no_base_joint`,
    `_All_Calibrated`, `_Arm_Calibrated`, `_All_Calibrated_objs`, `husky_dual_ur5e_vamp.urdf`.
  - Checked: only `ur_arm_base_link` (one arm) or `right_ur_arm_base` (Cindy) moved; the fixed `_Calibrated` files
    match the former `_StockUrFrames` copies apart from the added comments (same XML, 0 mm / 0° FK).
- The 3 `_StockUrFrames` copies are deleted.
- `husky_ur_description/urdf/husky_ur5_e_description.urdf.xacro`: mount default `0 0 -π/2` (was `π`, which with the
  stock joints below put the arm 90° off its real mounting).
- SRDFs and moveit configs unchanged: they name links only.

### scaffolding_assembly_core (submodule, branch `main`)

- `urdf.apply_ur_calibration(urdf_text, arm_name, calibration)` + `JointOriginChange`: sets an arm's six joint
  origins from a UR calibration (the driver's yaml, parsed), rewriting only joints that differ beyond 0.01 mm /
  1e-5 rad. Additive; nothing existing changed, so no consumer review was run (`AGENTS.md` exempts behaviour-neutral
  changes; say so if you want one). Tests in `tests/test_urdf.py`.
- `legacy/conversion.py`: comment only (the converter's `_Calibrated` files are stock now).

### crl-husky (`jg/ros2`)

- `config_resolver.py` rewritten: identity, one default mocap calibration per id inside
  `config/robots/<serial>/mocap_config.json` (no pinning, no "newest wins"), lazy (nothing read at import),
  `get_ur_calibration_file`, `get_onboard_ur_calibration_file`. Removed: `get_mocap_calibration_for_id`,
  `get_mocap_calibration_by_file_name`, `reload_mocap_calibration_cache` (only the relay used them).
- Defaults = the files the old resolver picked (newest): 0804 20260623, 0805 20261013, 0806 20260916.
  `config/mocap/` deleted; the two others (0805 20261001 "manual from 0804", 0806 20260625 right arm) are in git
  history, 0625 also on the Drive.
- Onboard launches (`crl_ur5e`, `crl_dual_ur5e`) take `kinematics_params_file` from the resolver.
- Relay removed: `control_pc/mocap_relay.py`, its entry point, `launch/control_pc/mocap.launch.py`;
  `pure_pursuit_sim.launch.py` lost `mocap_relay:=true` (its docstring says how to start teleop's relay instead).
- Docs: `MOCAP_SETUP.md` is now a pointer to teleop's `doc/mocap_relay.md`; `config/robots/README.md`.
- `config/robots/<serial>/model.yaml`: the robot's URDF and SRDF (paths in husky_urdf), read by
  `get_robot_model` / `get_robot_models` with the name from `env/setup.bash`; `python3-yaml` exec_depend.

### husky-assembly-teleop (`jg/viser-cleanup`)

- Relay moved here: `husky_assembly_teleop/mocap_relay/` (`node.py`, `calibration.py`), entry point
  `husky_mocap_relay`, `launch/mocap.launch.py` (installed). Same topics, service and behaviour, plus:
  - per robot rigid body: the newest Drive session (`data_experiment/calibration_data/<YYYYMMDD>/`, top level only)
    if within 5 cm / 5° of crl-husky's default, else the default with a warning; malformed or half-synced files
    warn and fall back;
  - logs id, robot, source, sha256 (of the transforms only) and the arm it was measured on;
  - latched `/mocap_relay/calibrations`, service `/mocap_relay/reload_calibration`;
  - watches the Drive every 30 s and only logs newer sessions, never applies them.
  - its log: a table of what each robot got and why a newer one was refused (`calibration.explain`), boxes for
    warnings, green for success, lines wrapped at 100 characters (`log_text.py`); the launch forces ROS's
    severity colours. ! An edit to the file in use is not noticed, only a newer session folder.
- `config.py`: URDF, SRDF and robot names from crl-husky's `model.yaml` (arms, tools, poses stay here); back to
  the `_Calibrated` URDFs; each arm's UR calibration from crl-husky is applied to the URDF copy
  at startup (`UrCalibrationCheck` on `RobotConfig.ur_calibration`, replacing the unused `calibration_file` and its
  TODO). No-op when up to date; otherwise a bold error at startup, an amber `URDF` health chip, and the monitor
  uses the calibration.
- Tests: `test_ur_calibration_check.py`, `test_mocap_relay_calibration.py`, every arm URDF stock
  (`test_ur_frames.py`), the turned-frame FK test rebuilt from a temporary copy (`test_core_scene.py`).
- Docs: README (relay section), `doc/ur_frames.md`.

Checked live (isolated domain 70, fastrtps, localhost, no Motive): the relay starts, applies the Drive sessions
(0804 and 0805 equal their defaults by hash; 0806 takes today's 20261016 session, 0.8 mm from the default),
publishes and reloads; a session added to a temporary Drive copy while running is logged once, not applied.

## Temporary compatibility (remove later)

- `config_resolver.LEGACY_UR_CALIBRATION_FILES` and the fallback in `get_onboard_ur_calibration_file`: the robots
  keep reading `/home/administrator/config/ur/*calibration.yaml` until the files are in crl-husky.
- `mocap_relay/calibration.read_session_file`: the pipeline's output has no `mocap_id`, so the id comes from
  crl-husky by serial; a file that names one must match.
- `/mocap_relay/calibrations` is a JSON `std_msgs/String`, not a proper message.

## TODO

1. ~~Copy each arm's `calibration.yaml` from the robots~~ done (16 Oct): all four are in
   `crl-husky/config/robots/<serial>/`, and every URDF matches them exactly (test
   `test_the_urdfs_have_the_robots_calibrations`). Checked in a plain (non-symlink) build of crl-husky, as on the
   robots: `crl_ur5e` loads `share/crl_husky/config/robots/<serial>/ur_calibration.yaml`, `crl_dual_ur5e` passes
   `_left` and `_right` to the right arms. That needed a resolver fix: in a plain install the module sits in
   site-packages, so `config/robots` is now looked up in the installed share as a fallback (the old resolver had the
   same path, so on a plain install it would not have found `mocap_config.json` either).
   ! The files are untracked: `upload.sh` skips them until they are committed or `git add`ed; rebuild on the robot.
2. **Build and switch the relay**: `python3 -m colcon build --symlink-install --packages-select crl_husky
   husky_assembly_teleop`, then `ros2 launch husky_assembly_teleop mocap.launch.py` instead of
   `crl_husky mocap.launch.py`. Push crl-husky before teleop.
3. **Bump submodules**: `husky_urdf` and the core in teleop; `husky_urdf` in the Rhino repo (it loads the
   `_Calibrated` names, so its exports get the fixed frames). ! Never bump `husky_urdf` on teleop `master`: the old
   monitor's hard-coded −90° assumes the turned files.
4. **Calibration pipeline** (not touched here): write `mocap_id` (and robot, arm) into its output and
   `config.yaml`; then drop the id-by-serial fallback.
5. **Ask Yijiang** why `2aababb` turned Cindy's right `base` joint, in case something depended on it.
6. Process check: warn when a process robot has `RobotModel.stock_ur_frames` false (old exports).
7. Health: show which calibration the relay applied (subscribe to `/mocap_relay/calibrations`).
8. A proper `crl_husky_msgs` message for the published calibrations.
9. Update crl-husky's default calibrations when a Drive session is accepted (today 0806's 20261016 is newer).
10. Consolidate a robot's config further, so each fact is stored once: today it is spread over crl-husky's
    `robot.yaml`, `env/setup.bash`, `mocap_config.json`, `model.yaml`, the UR yamls, and teleop's `_ROBOTS_BY_SERIAL`
    (arms, tools, poses).
