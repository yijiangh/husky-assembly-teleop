# Husky Extrinsic Calibration Manual

This manual walks you through the complete extrinsic calibration procedure for the Husky robot, from powering on the motion capture system to running the calibration pipeline. Follow each section in order.

> **Audience**: This guide assumes basic familiarity with the robot hardware but provides detailed command-line instructions for each step.

---

## Table of Contents

0. [Before Starting](#0-before-starting)
1. [OptiTrack Motion Capture Setup](#1-optitrack-motion-capture-setup)
2. [Robot Startup & ROS2 Nodes](#2-robot-startup--ros2-nodes)
3. [Launching the Monitor (Workstation Side)](#3-launching-the-monitor-workstation-side)
4. [Calibration Data Collection](#4-calibration-data-collection)
5. [Validation Data Collection](#5-validation-data-collection)
6. [Punch Validation](#6-punch-validation)
7. [Running the Calibration Pipeline](#7-running-the-calibration-pipeline)
8. [Running Punch Validation Analysis](#8-running-punch-validation-analysis)
9. [Troubleshooting](#9-troubleshooting)

---

## 0. Before Starting

### 0.1 New Workstation Setup (one-time)

If you are running on a **new workstation (PC)** for the first time, point the code
at this machine's gdrive (Insync) mount, which differs per machine.

| What | Where | Note |
|------|-------|------|
| `DESIGN_DATA_DIRECTORY`, `EXPERIMENT_DATA_DIRECTORY` | [`__init__.py:59-64`](../husky_assembly_teleop/__init__.py#L59) | gdrive (Insync) mount paths. The defaults are the robot-control PC's; on another PC **set the environment variables of the same names** instead of editing the file. |
| `CALIBRATION_DATA_DIRECTORY` | [`__init__.py:68`](../husky_assembly_teleop/__init__.py#L68) | where the dated calibration datasets live (`EXPERIMENT_DATA_DIRECTORY/calibration_data`, see [§4.3](#43-data-folder-structure)). Derived -- nothing to edit once the mount path above is right. |
| `MOCAP_CAMERA_EXPORT_DIR` | [`husky_monitor.py:296`](../husky_assembly_teleop/husky_monitor.py#L296) | where the `collect cameras data` button drops JSON+CSV (`EXPERIMENT_DATA_DIRECTORY/visualise_mocap_camera`). Derived. |
| gdrive root in standalone scripts | `export_mocap_cameras.py`, `rhino8_import_outliers.py`, `robot_skeleton_viewer.py` in `data/calibration_data/` | these do not import the package (they also run inside Rhino), so the gdrive path is a plain string -- edit it by hand. |

> **Where things live**: the calibration **scripts** stay in the repo under
> `data/calibration_data/`; the recorded **data** (the dated `YYYYMMDD/` folders)
> lives on the gdrive under `data_experiment/calibration_data/` (see [§4.3](#43-data-folder-structure)).

### 0.2 Robot Reference Table

Quick reference for all Husky robots. Later sections link here for IPs, ROS domain,
serials, and Motive Streaming IDs.

| Name | Serial | IP | ROS Domain | Motive ID (base) | Calib-tool ID |
|------|--------|-----|-----------|------------------|---------------|
| Alice | 0804 | 192.168.0.113 | 84 | 1840 | 1862 (arm 0) |
| Belle | 0805 | 192.168.0.114 | 85 | 1850 | 1866 (arm 0) |
| Cindy | 0806 | 192.168.0.115 | 86 | 1860 | right 1861 / left 1866 (since 2026-10-16) |

1. **ROS Domain** selects the robot at launch: the monitor reads `ROS_DOMAIN_ID` from the
   terminal it is started in (see [§2.4](#24-verify-ros2-connection) and [§3.1](#31-launch-the-monitor-node))
   - this drives namespace, base `mocap_id`, gripper and EE. The other two huskies are
     drawn too, but only moved by mocap (no ROS connection).
2. **Motive ID** = the rigid-body **Streaming ID** set in Motive (see [§1.3](#13-create--verify-rigid-bodies)).
   - These must match the base-tracker `mocap_id` values in the robot registry,
     [`robot_registry.py:312-314`](../husky_assembly_teleop/robot_registry.py#L312)
   - and the calib-tool IDs: arm `0` (left / single arm) = `1862` unless the robot is listed in `CALIB_TOOL_ARM0_ID_BY_ROBOT` at [`husky_world.py:66`](../husky_assembly_teleop/husky_world.py#L66) (Belle and Cindy: `1866`); arm `1` (Cindy's right) = `1861`.
   - A single-arm robot (Alice, Belle) only has arm `0`, so it always uses its
     arm-0 calib tool -- `1862` on Alice, `1866` on Belle. Mount that tracker on its flange.

---

## 1. OptiTrack Motion Capture Setup

### 1.1 Power On & Start Motive

1. Turn on the PoE (Power over Ethernet) switch that powers the OptiTrack cameras. Wait for all camera indicator LEDs to turn on.
2. On the OptiTrack PC, boot into **Ubuntu**.
3. Launch the **Motive** software.

> For detailed instructions on starting up the OptiTrack system and using Motive, refer to the [OptiTrack Motive Documentation](https://docs.optitrack.com/motive).

<!-- SCREENSHOT: OptiTrack cameras powered on with LEDs visible -->
<!-- SCREENSHOT: Motive software main screen after startup -->

### 1.2 Calibrate the Camera System

If the cameras have been moved or it has been more than a week since the last calibration, you should recalibrate the system.

> Follow the official [OptiTrack Calibration Guide](https://docs.optitrack.com/motive/calibration) for the wanding and ground plane procedure.

<!-- SCREENSHOT: Motive calibration wanding screen -->
<!-- SCREENSHOT: Motive calibration result showing quality metrics -->

### 1.3 Create / Verify Rigid Bodies

If you need to create a new rigid body (e.g., for the robot base or flange tracker):

1. In Motive, select the markers that form the rigid body.
2. Right-click and choose **Create Rigid Body**.
3. Note the **Rigid Body ID** assigned by Motive -- you will need this for the PyBullet configuration.
4. In the code, the rigid body IDs are configured in two places:
   - **Robot base tracker**: the `mocap_id` in the robot registry — Cindy `1860` / Alice `1840` / Belle `1850` at [`robot_registry.py:312-314`](../husky_assembly_teleop/robot_registry.py#L312).
   - **Flange/calib-tool tracker**: arm `0` = `1862`, or the robot's entry in `CALIB_TOOL_ARM0_ID_BY_ROBOT` at [`husky_world.py:66`](../husky_assembly_teleop/husky_world.py#L66) (Belle `1866`); arm `1` = `1861` ([`husky_world.py:529`](../husky_assembly_teleop/husky_world.py#L529)).

   Make sure the IDs you see in Motive match the IDs in these two locations (and the Motive ID column in the [Robot Reference Table (§0.2)](#02-robot-reference-table)).

<!-- SCREENSHOT: Motive rigid body creation dialog -->
<!-- SCREENSHOT: Motive rigid body properties showing the ID number -->

### 1.4 Network Connection (MoCap to Workstation)

The workstation must be on the same network as the OptiTrack PC to receive streaming data.

**Key IP addresses:**

| Device | IP Address |
|--------|-----------|
| Workstation (your PC) | your own IP (this PC: `192.168.0.201` since 2026-10-22) |
| OptiTrack PC (`DESKTOP-T1HDH98`) | `192.168.0.28` |

These are configured in the code at [`husky_monitor.py`](../husky_assembly_teleop/husky_monitor.py) (`CLIENT_IP`, `MOCAP_IP`):
```python
CLIENT_IP = '192.168.0.201'
MOCAP_IP = '192.168.0.28'
```

> **Tip**: find your workstation IP with `ip -c address` in a terminal, then set
> `CLIENT_IP` to it.

> **Both of these are DHCP leases from the Fritz!Box, not fixed addresses.** They
> move when a machine is re-connected or re-cabled. The OptiTrack PC was on
> `192.168.0.117` until Sept 2026, when its lease changed to `192.168.0.28`.
> Treat a mismatch here as the first suspect whenever mocap will not connect --
> see *MoCap not connecting* in Section 9.

**Two NICs on the OptiTrack PC.** That machine has two Ethernet adapters, and
telling them apart matters:

| Adapter | Address | Purpose |
|---------|---------|---------|
| Camera NIC | `169.254.x.x` (link-local, no gateway) | The OptiTrack cameras, on their own isolated switch |
| LAN NIC | `192.168.0.x` (has the `fritz.box` DNS suffix) | The lab network -- this is the one NatNet streams from |

A `169.254.x.x` address with no default gateway is **normal and correct** for the
camera NIC -- that is how the cameras are addressed, not a fault. Never
re-address it to fix a streaming problem; you will drop the cameras.

**To verify the network connection:**

1. Make sure your workstation is connected to the Husky switch (either via Ethernet cable or the lab network).
2. Verify you can ping the OptiTrack PC:
   ```bash
   ping 192.168.0.28
   ```
3. In Motive (3.0.3), go to **Edit > Settings > Streaming** and make sure:
   - **NatNet** streaming is **enabled**
   - **Local Interface** = `192.168.0.28` -- the **LAN** address, *not* the
     `169.254.x.x` camera one and not loopback
   - **Transmission Type** = **Unicast** (the client calls
     `set_use_multicast(False)`, so Multicast here will not connect)

<!-- SCREENSHOT: Motive streaming settings panel showing enabled streaming and correct IP -->
<!-- SCREENSHOT: Terminal showing successful ping to 192.168.0.28 -->

> **Tip**: Motive builds the **Local Interface** list once, at application
> startup. If you enable or re-cable an adapter while Motive is running, the new
> address will not appear until you fully close Motive (confirm no `Motive.exe`
> remains in Task Manager) and re-open it.

> **Tip**: If the mocap connection fails in the monitor, you will see a red log message. Check the Motive streaming settings and verify both IPs are correct.

---

## 2. Robot Startup & ROS2 Nodes

This section covers starting up the Husky robot and the UR5e arm ROS2 drivers. Make sure the motion capture system is already running before proceeding.

### 2.1 Power On the Husky

1. Press the **power button** on the Husky.
2. Turn on the **remote e-stop** and press **"Go"**.
   > Do NOT start multiple huskies at the same time -- sometimes both huskies connect to one e-stop if started in parallel.

   > **Tip**: If the remote e-stop is not reacting, press and hold longer until the signal lights go off, restart it, then press **"Go"** again.
3. Wait until the **communication light** on the Husky turns **green** (or yellow if e-stop is still active).
4. (Optional) Connect the **PS controller** by pressing the PlayStation logo button. Wait for the backlight to stop blinking and stay on permanently. You can then move the Husky by pressing **L1 or L2 + left joystick**.

### 2.2 Power On the UR5e Arm

1. Move the Husky to a **safe spot** using the remote where the arm cannot hit any equipment.
2. Press the **UR5e startup button** on the robot.
3. Wait for the UR5e to boot (check the **teach pendant tablet**).
4. On the tablet, press **"Power"** in the lower-left corner to power on the arm.
5. Set the mode to **External Control** in the upper-right corner of the tablet.

> **Warning**: Do NOT manually start `ros_control.urp` on the tablet. Make sure to stop it if switching back to local control. The ROS2 driver handles this automatically.

<!-- SCREENSHOT: UR5e teach pendant showing External Control mode selected -->

### 2.3 Start the UR5e ROS2 Driver (On the Husky)

1. SSH into the Husky:
   ```bash
   # For Cindy (dual-arm, serial 0806):
   ssh administrator@192.168.0.115
   # Password: 12345678

   # For Belle (single-arm, serial 0805):
   ssh administrator@192.168.0.114
   # Password: 12345678

   # For Alice (single-arm, serial 0804):
   ssh administrator@192.168.0.113
   # Password: clearpath
   ```

2. Launch the appropriate ROS2 driver:
   ```bash
   # For Cindy (dual arm):
   ros2 launch crl_husky crl_dual_ur5e.launch.py namespace:='/a200_0806' gripper:=none

   # For Belle (single arm) -- two variants, pick by how Belle is connected
   # (see "Belle: which launch" below):

   # (a) ORIGINAL launch -- Cyclone left to choose the network interface itself.
   #     Use when Belle is on the lab network by cable, or whenever the
   #     workstation already sees her topics with it.
   ros2 launch crl_husky crl_single_ur5e.launch.py namespace:='/a200_0805' gripper:=none

   # (b) CURRENT launch (since 2026-10-13) -- Belle on Wi-Fi (wlp2s0).
   #     Pins Cyclone to the Wi-Fi interface; the export must come first, in the
   #     same terminal as the launch.
   export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="wlp2s0"/></Interfaces></General></Domain></CycloneDDS>'
   ros2 launch crl_husky crl_single_ur5e.launch.py namespace:='/a200_0805' gripper:=none

   # For Alice (single arm):
   ros2 launch crl_husky crl_single_ur5e.launch.py namespace:='/a200_0804' gripper:=none
   
   # Gripper (`gripper:=` arg, must pass at least `none`):
   gripper:=robotiq_2F_85
   gripper:=none
   gripper:=scaffolding_v1
   gripper:=scaffolding_v3
   ```

   > **Belle: which launch.** Belle has two networks: the robot-internal `br0`
   > (192.168.131.x, to the UR controller) and the lab network, which on
   > 2026-10-13 she reached over **Wi-Fi (`wlp2s0`)**. Run `ip -br addr | grep 192.168`
   > on Belle to see which interface holds `192.168.0.114`:
   >
   > | `192.168.0.114` is on | Use | Why |
   > |---|---|---|
   > | a cable interface, and the workstation sees Belle's topics | **(a) original** | Cyclone finds the lab network by itself |
   > | `wlp2s0` (Wi-Fi) | **(b) current** | without it Cyclone picks `br0`: Belle sees her own topics, but the workstation sees **nothing** (`ros2 topic list` shows only `/parameter_events` and `/rosout`) |
   >
   > When unsure, start with (a) and check from the workstation ([§2.4](#24-verify-ros2-connection)).
   > If nothing arrives, stop the driver and relaunch with (b). If (b) still shows
   > nothing, Wi-Fi is dropping the discovery multicast; give Belle the
   > workstation's address as well:
   > `...</Interfaces></General><Discovery><Peers><Peer address="192.168.0.201"/></Peers></Discovery></Domain></CycloneDDS>`.
   > The export only lasts for that terminal; nothing is changed on Belle permanently.

   > For calibration use `gripper:=none`, also on Alice and Belle (the calib tool
   > replaces the Robotiq gripper). The monitor still looks for the Robotiq
   > gripper of a single-arm robot, gives up after 2.5 s and carries on -- that
   > warning is expected.

   > To use a **virtual robot** (no real arm, for testing), do NOT change the launch
   > command. Instead set `FAKE_HARDWARE=1` on the workstation in
   > [`husky_monitor.py:313`](../husky_assembly_teleop/husky_monitor.py#L313).

### 2.4 Verify ROS2 Connection

On your **workstation** (a new local terminal, not the SSH one -- the prompt
should read `su@CRLRobotxPC`), set the correct ROS2 domain and middleware. The
workstation's `~/.bashrc` does not set these two, so every new terminal needs them:

```bash
export ROS_DOMAIN_ID=85   # 85 for Belle, 84 for Alice, 86 for Cindy
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 daemon stop          # drop the topic list cached from an earlier domain
```

> **Workstation Cyclone config (already set, nothing to type).** Since
> 2026-07-22 the workstation's `~/.bashrc` sets
> `export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml`, which pins Cyclone to
> `eno1`, the wired lab network (192.168.0.201 since 2026-10-22). Check it with
> `echo $CYCLONEDDS_URI`. This is the workstation half; the robot half is the
> `CYCLONEDDS_URI` export in Belle's launch (b) ([§2.3](#23-start-the-ur5e-ros2-driver-on-the-husky)).
>
> ```xml
> <!-- ~/.cyclonedds.xml on the workstation -->
> <CycloneDDS><Domain><General>
>   <Interfaces><NetworkInterface name="eno1"/></Interfaces>
>   <AllowMulticast>default</AllowMulticast>
> </General></Domain></CycloneDDS>
> ```

Quick check -- this one line is enough (should print ~500 Hz):

```bash
ros2 topic hz /a200_0805/ur5e/rate_limiter/joint_states
```

If it prints nothing, see *Workstation sees no topics from the robot* in
[§9](#9-troubleshooting). More checks:

```bash
# List all available topics
ros2 topic list

# Check if joint states are streaming (should see data at ~500Hz)
   # single arm (Belle 0805; Alice is 0804):
   ros2 topic echo /a200_0805/ur5e/joint_states
   # dual arm:
   ros2 topic echo /a200_0806/left_ur5e/joint_states

# Check the frequency
   # single arm:
   ros2 topic hz /a200_0805/ur5e/joint_states
   # dual arm:
   ros2 topic hz /a200_0806/left_ur5e/joint_states

# The monitor itself listens to the rate-limited copy, so check that one too:
   ros2 topic hz /a200_0805/ur5e/rate_limiter/joint_states

# Visualize the full node graph
rqt
```

<!-- SCREENSHOT: Terminal showing ros2 topic list output with ur5e topics visible -->
<!-- SCREENSHOT: rqt node graph showing connected nodes -->

> See the [Robot Reference Table (§0.2)](#02-robot-reference-table) for each robot's serial, IP, ROS domain, and Motive Streaming IDs.

> **Warning**: The E-Stop on the Husky does NOT stop the UR5e! Always use the **remote E-Stop** when operating the robot arms.

---

## 3. Launching the Monitor (Workstation Side)

The monitor is the PyBullet-based GUI that you use to plan motions, execute trajectories, and collect calibration data.

### 3.1 Launch the Monitor Node

In a **workstation** terminal (not the SSH session on the robot -- the robot
does not have this package, so it answers `Package 'husky_assembly_teleop' not
found`), for example the one from [§2.4](#24-verify-ros2-connection), run:

```bash
cd ~/ros2_ws
source venv/bin/activate
source install/setup.bash
export ROS_DOMAIN_ID=85   # picks the robot: 85 Belle, 84 Alice, 86 Cindy
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run husky_assembly_teleop husky_monitor
```

> **The domain picks the robot.** The monitor connects to the robot whose
> ROS domain is in `ROS_DOMAIN_ID` (see [§0.2](#02-robot-reference-table)). If the
> variable is missing it silently falls back to **Cindy** (86), so always export
> it in the same terminal. Check the start-up log for the robot name.

> **Expected warning while calibrating a robot for the first time** (here from Belle's
> first session, 2026-10-13; her file exists since 2026-10-14):
> `Base calibration file not found for robot 0805: .../20261013/calibrated_transformation_0805_rhino.json. Continuing without base calibration.`
> That file is the *output* of this calibration ([§7.3](#73-run-the-pipeline)),
> so it cannot exist yet. Data collection does not need it -- every point stores
> the raw mocap poses. The warning disappears on the first launch after the
> pipeline has run. The other two robots must still report a file each
> (`Base calibration of Cindy (0806): 20261016/...`).

This will open the **PyBullet monitor window** (the 3D view) and the **Dear PyGui
control panel** (the sliders and buttons of [§4.1](#41-understanding-the-gui-controls)).

<!-- SCREENSHOT: PyBullet monitor window after startup, showing the grey robot model -->

### 3.2 Verify the Connection

If the ROS2 connection is working correctly, the **grey robot model** in PyBullet should match the **real robot's joint configuration**. Move the real robot's joints and verify that the PyBullet model moves accordingly.

<!-- SCREENSHOT: Side-by-side of real robot and PyBullet model in matching configuration -->

**If the robot model does NOT match the real robot**, check:
1. Is the ROS2 driver running on the Husky side? ([§2.3](#23-start-the-ur5e-ros2-driver-on-the-husky))
2. Is the robot powered on and in **External Control** mode? ([§2.2](#22-power-on-the-ur5e-arm))
3. Are the ROS2 domain ID and middleware set correctly? ([§2.4](#24-verify-ros2-connection))
4. Try running `rqt` to verify topics are visible.

---

## 4. Calibration Data Collection

This is the main calibration data collection procedure. You will collect data for **J0** (shoulder pan joint rotation) and **J1** (shoulder lift joint rotation) batches.

### 4.0 Pre-Calibration Checklist

Before collecting data, verify these settings in the code match your setup.

> **Current settings (2026-10-16): assembly mode.** Cindy's (0806) left-arm calibration is
> done (folder `20261016`) and in use: `BASE_CALIBRATION_DATE_BY_ROBOT['Cindy']` = `20261016`
> (validation 1.6 mm RMS over 15 poses, punch 0.7 mm RMS). The old `20260916` (a copy of the
> June `20260622` result) agrees within 0.8 mm / 0.02° and is the fallback. The mode flags
> below are back at their assembly values (`CALIBRATION = 0`, `PUNCH_CALIB_VALIDATION = 0`,
> both `BAR_ACTION_*` = `1`, `CONNECT_COMPLIANT_CONTROLLER = 1`, as in the three-robot manual);
> both dates still point at `20261016`. For the next session set the flags and both dates as below.
> * Cindy's left arm: calib tool Motive ID `1866`, `ROS_DOMAIN_ID=86`, `arm id` = `0`, which loads
> her own `260108_extrinsic_calib_trajs` (`ext_calib_0806_*`). Her left punch TCP (4-point,
> 2026-10-16) is `[-0.93, -4.05, 110.5]` mm → `punch_tool.left.offset_xyz: [-0.00093, -0.00405, 0.1105]`.
> * The calib tool may be put back turned (it was, by 90°, before the 2026-10-16 validation):
> the base calibration only uses the circles the tool traces, and `3_verify_calibration.py`
> only checks that `tool0_from_flange_mocap` stays the same *within* one batch.
> ! Do not take the tool off in the middle of a batch.
> * Belle again: calib tool `1866`, `ROS_DOMAIN_ID=85`, same `260630_calib_trajs_Alice` set.

**Values, IDs & IPs** — set to match your robot / network / session:

| What | Where | Note |
|------|-------|------|
| `ROS_DOMAIN_ID` | the monitor's terminal ([§3.1](#31-launch-the-monitor-node)) | **picks the robot** being calibrated (85 = Belle) |
| base `mocap_id` (Streaming IDs) | [`robot_registry.py:312-314`](../husky_assembly_teleop/robot_registry.py#L312) | one per robot (see [§0.2](#02-robot-reference-table)); match Motive **properties > Streaming ID** |
| calib-tool IDs | [`husky_world.py:66`](../husky_assembly_teleop/husky_world.py#L66) | match Motive (see [§0.2](#02-robot-reference-table)). **Which tool is used = the `arm id` slider**: arm `0` → `1862` (or the robot's `CALIB_TOOL_ARM0_ID_BY_ROBOT` entry: Belle and Cindy `1866`), arm `1` → `1861`. A single-arm robot only has arm `0`; nothing to comment out. The start-up log prints `Calib tool of <robot> arm 0: Motive ID ...` |
| `CALIBRATION_DATE` + `CALIBRATION_ANALYSIS_DATE` | [`__init__.py:89`](../husky_assembly_teleop/__init__.py#L89) and [`:104`](../husky_assembly_teleop/__init__.py#L104) | both sit in **one file**; set both to today's date folder, which must exist on the gdrive (`data_experiment/calibration_data/YYYYMMDD/`, [§4.3](#43-data-folder-structure)) with a `config.yaml` whose `robot_name` is the robot you calibrate ([§7.1](#71-prepare-the-config-file)). Every robot whose base calibration lives in another folder needs an entry in `BASE_CALIBRATION_DATE_BY_ROBOT` ([`__init__.py:96`](../husky_assembly_teleop/__init__.py#L96), [§7.2](#72-set-the-date-folder)). Nothing to edit in `config_loader.py` — its `DEFAULT_DATE_FOLDER` just re-exports `CALIBRATION_ANALYSIS_DATE` |
| `CALIBRATION_STATE_SET_ARM0_BY_ROBOT` + `CALIBRATION_STATE_SET_ARM1` | [`husky_monitor.py:320`](../husky_assembly_teleop/husky_monitor.py#L320) | the reference cell states + trajectories, a folder under `DESIGN_DATA_DIRECTORY`. Arm `0` is picked per robot and **every robot must be listed** (no default; a missing one raises `KeyError`): Alice and Belle = `260630_calib_trajs_Alice`, Cindy = `260108_extrinsic_calib_trajs` (Alice's set breaks Cindy's −240° wrist_1 limit in J0 traj1 / J1 traj4). Arm `1` (Cindy's right) = `260225_extrinsic_calib_trajs_Cindy_Right`. The monitor prints `Found N calib RobotCellState files under ...` at start-up |


**Mode flags** — class `HuskyMonitor` switches, each set to `0` or `1`:

| Flag | Where | Set to | Note |
|------|-------|--------|------|
| `USE_MOCAP` | [`husky_monitor.py:312`](../husky_assembly_teleop/husky_monitor.py#L312) | `1` | `0` = simulated; `1` = stream real mocap|
| `FAKE_HARDWARE` | [`husky_monitor.py:313`](../husky_assembly_teleop/husky_monitor.py#L313) | `0` | `0` = real robot; `1` = virtual robot. Keep consistent with `USE_MOCAP` by hand |
| `USE_CELL_STATE_BASE_POSE` | [`husky_monitor.py:344`](../husky_assembly_teleop/husky_monitor.py#L344) | `0` | `0` = base tracks mocap (normal); `1` = pin base to loaded cell state |
| `CALIBRATION` | [`husky_monitor.py:354`](../husky_assembly_teleop/husky_monitor.py#L354) | `1` | `1` = calibration mode enabled |
| `BAR_ACTION_LIVE_REPLAN_EXE` | [`husky_monitor.py:356`](../husky_assembly_teleop/husky_monitor.py#L356) | `0` | `1` adds the bar-assembly panel (plan skeleton, movements); not used for calibration |
| `BAR_ACTION_MOCAP_ACCURACY_TEST` | [`husky_monitor.py:373`](../husky_assembly_teleop/husky_monitor.py#L373) | `0` | `1` adds the bar-holding accuracy test (bar rig tracker, markerset buttons); not used for calibration |
| `PUNCH_CALIB_VALIDATION` | [`husky_monitor.py:451`](../husky_assembly_teleop/husky_monitor.py#L451) | `0` | `1` only for a punch-validation session (switches EE to punch tips) |

> **Going back to assembly work afterwards**: set `CALIBRATION = 0` and
> `PUNCH_CALIB_VALIDATION = 0`, the two `BAR_ACTION_*` flags back to `1` (the mocap
> accuracy test setting) and `CONNECT_COMPLIANT_CONTROLLER = 1` (Belle's session ran
> with it at `0`). Point the calibrated robot's `BASE_CALIBRATION_DATE_BY_ROBOT` entry at
> the new folder ([§7.2](#72-set-the-date-folder)); the two dates can then stay.

### 4.1 Understanding the GUI Controls

The sliders and buttons live in the Dear PyGui control panel. The **When shown** column
marks which controls only appear in a specific mode (Calibration = `CALIBRATION=1`, Punch =
`PUNCH_CALIB_VALIDATION=1`). A few general tools (joint live stream, teleop panel) are
always there but are not needed for calibration; the bar-assembly controls only appear
when `BAR_ACTION_LIVE_REPLAN_EXE=1`.

**Sliders:**

| Slider | When shown | Description |
|--------|-----------|-------------|
| `arm id (0:L,1:R)` | always (`arm id (0 only)` if single-arm) | Select left (0) or right (1) arm |
| `traj time` | always | Duration of the planned trajectory in seconds (1 to `trajectory_time_max`, default 90) |
| `Traj viz time` | always | Scrub through a planned trajectory to preview it (0.0 to 1.0) |
| `Mode (0:validation, 1:data_collection)` | Calibration | Toggle between validation and data collection mode |
| `Batch (0:j0,1:j1,2:valid,3:punch)` | Calibration | Select which calibration batch subfolder data saves into |
| `Calib RobotCellState (idx)` | Calibration (only if ≥2 cell states found) | Select which pre-defined cell state to load |
| `Calib JointTrajectory (idx)` | Calibration (only if ≥2 trajectories found) | Select which pre-defined calibration trajectory to load |

**Buttons:**

| Button | When shown | Description |
|--------|-----------|-------------|
| `Toggle Goal / Trajectory view` | always | Switch the ghost model between goal (blue) and trajectory (green) |
| `Load Calib RobotCellState` | Calibration | Load the selected cell state as the arm goal configuration |
| `Plan S.Arm to conf target` | Calibration | Plan a motion from the current configuration to the goal (the arm picked by `arm id`) |
| `Exec S.Arm Traj` | Calibration | Execute that planned motion on the real robot, over `traj time` seconds |
| `Load Calib JointTrajectory` | Calibration | Load the selected calibration trajectory for execution |
| `Execute calib traj` | Calibration | Execute the loaded calibration trajectory with move-stop-record. **Starts with a 1 s move to its first waypoint** -- the arm must already be at the cell state |
| `Record current calib conf` | Calibration | Record one configuration (joint conf + mocap poses); used for validation batch |
| `Export calib data to json` | Calibration | Save collected calibration data to a JSON file |
| `collect cameras data` | Calibration | Export the current mocap camera inventory (poses) to JSON+CSV in `MOCAP_CAMERA_EXPORT_DIR` |
| `Record Punch Take` | Punch | Record one punch take (joint conf, base mocap pose, FK world-frame punch-tip position) |
| `Save Punch Validation Data` | Punch | Save all recorded punch takes to a JSON file |

<!-- SCREENSHOT: PyBullet GUI showing the sliders and buttons panel on the right side -->

### 4.2 Data Collection Workflow (J0 and J1)

For each calibration batch (J0 and J1), you will:
1. Load a cell state (starting configuration)
2. Plan and execute a motion to reach that configuration
3. Load and execute a calibration trajectory (which does move-stop-record automatically)
4. The data is automatically exported to JSON after the trajectory finishes

**Step-by-step:**

#### Step 1: Set the Batch Slider

1. Set the **"Batch"** slider to:
   - `0` for **J0** data collection
   - `1` for **J1** data collection
2. Make sure the **"Mode"** slider is set to `1` (data collection mode).

<!-- SCREENSHOT: GUI showing Batch slider set to 0 (j0) and Mode slider set to 1 -->

#### Step 2: Load a Robot Cell State

1. Use the **"Calib RobotCellState (idx)"** slider to select the desired cell state. These are pre-defined starting configurations for the calibration trajectories.
2. Click **"Load Calib RobotCellState"**.
   - The goal configuration will be shown as a **blue ghost model** in PyBullet.

<!-- SCREENSHOT: PyBullet showing blue ghost model at the target cell state -->

#### Step 3: Plan and Preview the Motion

1. Set the **"traj time"** slider to a reasonable duration. For calibration, a longer time (e.g., **20-40 seconds**) is recommended so the robot moves slowly.
2. Click **"Plan S.Arm to conf target"** to plan a motion from the current configuration to the goal.
3. Use the **"Traj viz time"** slider to scrub through the planned trajectory and preview the motion.
   - Click **"Toggle Goal / Trajectory view"** if needed to switch to the green trajectory visualization.
4. Verify the trajectory looks safe and collision-free.

<!-- SCREENSHOT: PyBullet showing green trajectory visualization with Traj viz time slider -->

#### Step 4: Execute the Motion to the Cell State

1. **Keep your hand on the e-stop** at all times during execution.
2. Click **"Exec S.Arm Traj"** to execute the planned trajectory.
3. Wait for the robot to reach the cell state configuration.

<!-- SCREENSHOT: Robot moving to cell state (optional action photo) -->

#### Step 5: Load and Execute the Calibration Trajectory

1. Use the **"Calib JointTrajectory (idx)"** slider to select the calibration trajectory that belongs to the current cell state -- the file names pair up, e.g. `ext_calib_0804_J0_traj2_RobotCellState.json` ↔ `ext_calib_0804_J0_traj2_JointTrajectory.json` (the terminal lists both with their index at start-up). The `0804` in the names is the robot the set was made on; the same files are used for Belle.
2. Click **"Load Calib JointTrajectory"** to load it.
3. Check the arm is **at** the cell state from Step 4. **"Execute calib traj"** moves to the first waypoint in about 1 s, so starting from anywhere else gives a fast, unplanned jump.
4. Click **"Execute calib traj"** to start the calibration trajectory.
   - This will automatically perform **move-stop-record** for each waypoint in the trajectory.
   - At the end, it will **automatically save** the recorded data to a JSON file.
5. You should see the data being saved in the terminal output.

<!-- SCREENSHOT: Terminal showing calibration data being recorded and saved -->
<!-- SCREENSHOT: PyBullet during calibration trajectory execution -->

> **Important**: If the e-stop triggers or something goes wrong and the robot stops mid-trajectory, you must:
> 1. Load the correct cell state again (Step 2)
> 2. Plan a motion back to that cell state (Step 3)
> 3. Execute the motion (Step 4)
> 4. Re-run the calibration trajectory from the beginning (Step 5)
>
> You need a **complete** data collection for each trajectory -- partial data cannot be used.

#### Step 6: Repeat for All Cell States

Repeat Steps 2-5 for **every cell state** available in the slider for the current batch (J0).

Then switch the **Batch slider to `1` (J1)** and repeat the entire process for all J1 cell states.

### 4.3 Data Folder Structure

The collected data is **not** stored in the repo. It goes to the gdrive (Insync)
folder `CALIBRATION_DATA_DIRECTORY` ([`__init__.py:68`](../husky_assembly_teleop/__init__.py#L68)):

```
/home/su/Insync/2025-03 Husky Assembly/data_experiment/calibration_data/
```

The rest of this manual refers to it as **`$CALIB`**:

```bash
CALIB="/home/su/Insync/2025-03 Husky Assembly/data_experiment/calibration_data"
```

The analysis scripts, the `config.yaml` template (`_data_template/`) and
`debug_utils/` stay in the repo under `data/calibration_data/`. Inside `$CALIB`
the data is automatically organized into folders:

```
$CALIB/
  └── YYYYMMDD/                          # Date folder (e.g., 20260126)
      ├── config.yaml                    # Configuration file
      ├── j0/                            # J0 batch data
      │   ├── calibration_YYYYMMDD_HHMM_..._J0_traj0_JointTrajectory.json
      │   ├── calibration_YYYYMMDD_HHMM_..._J0_traj1_JointTrajectory.json
      │   └── ...
      ├── j1/                            # J1 batch data
      │   ├── calibration_YYYYMMDD_HHMM_..._J1_traj0_JointTrajectory.json
      │   └── ...
      ├── validation/                    # Validation data
      └── punch_validation/              # Punch validation data
```

The batch slider controls which subfolder (`j0`, `j1`, `validation`, `punch_validation`) the data is saved into.

> The serial in a recorded file's name comes from the **trajectory** file, not
> the robot: Belle's data is saved as `..._ext_calib_0804_J0_traj0_...json`.
> That is harmless -- the scripts take the robot from `robot_name` in the
> folder's `config.yaml`, so one date folder must only hold one robot's data.

---

## 5. Validation Data Collection

After collecting all J0 and J1 data, you should collect validation data to verify the calibration quality. This involves recording the robot at random configurations with diverse base positions.

### 5.1 Set Up for Validation

1. Set the **"Batch"** slider to `2` (validation).
2. Set the **"Mode"** slider to `0` (validation mode).

<!-- SCREENSHOT: GUI showing Batch slider at 2 and Mode slider at 0 -->

### 5.2 Record Diverse Configurations

1. **Drive the Husky** to a random position using the remote controller (L1/L2 + joystick).
2. **Move the arm** to a random configuration. You can:
   - Use the teach pendant to jog the arm manually
   - Use free-drive mode if available
   - Or plan/execute to different goal configurations in the GUI
3. Click **"Record current calib conf"** to record the current configuration.
   - Each click records one data point (joint configuration + mocap poses).
4. Repeat: move the base and arm to a **different** configuration, then record again.
5. Collect **as many diverse configurations as possible** -- vary both the base position and joint angles widely.

<!-- SCREENSHOT: Robot in different base positions for validation data collection -->

### 5.3 Save Validation Data

When you have collected enough validation data points:

1. Click **"Export calib data to json"** to save all recorded configurations to a JSON file.
2. The file will be saved in the `validation/` subfolder of the current date folder.

> **Tip**: Aim for at least 15-20 diverse configurations. The more diverse the base positions and joint angles, the better the validation will be.

---

## 6. Punch Validation

Punch validation verifies the calibration accuracy by measuring how consistently a physical punch tool maps to the same world-space point from different robot base positions.

### 6.1 Hardware Setup

1. Make sure **punch calibration validation is enabled** before launching the monitor. In the current code this is controlled by `PUNCH_CALIB_VALIDATION = 1` in `husky_assembly_teleop/husky_monitor.py`.
2. Launch the monitor. When punch validation is enabled, the end effector visualization changes to **sharp cone-like punch tips**.
   - On dual-arm setups, the monitor may show **two punch tips** even if only one physical punch tool is installed. This is expected.
3. **Mount the punch tool** onto the robot's tool flange.
4. **Calibrate the punch tool offset** using the UR **4-point TCP calibration** procedure on the teach pendant:
   - Go to **Installation > TCP** on the tablet.
   - Follow the 4-point method to determine the **X, Y, Z** offset from `tool0` to the punch tip.
   - Enter the values in **meters**.
   - Click **Save** after entering the TCP values. Do not skip this step.
   - In practice, you typically only need to repeat the 4-point TCP calibration when you remove the tool and mount a new one.
5. **Update the config**: Enter the calibrated offset in the `config.yaml` file:
   ```yaml
   punch_tool:
     offset_xyz: [X, Y, Z]   # Values from 4-point calibration, in meters
   ```
   The config file is located at `$CALIB/YYYYMMDD/config.yaml` on the gdrive ([§4.3](#43-data-folder-structure)). Save the file after editing it.

<!-- SCREENSHOT: UR teach pendant showing 4-point TCP calibration procedure -->
<!-- SCREENSHOT: Punch tool mounted on the robot flange -->

### 6.2 Set Up the Punch Target

1. Place the **punch base / foundation / target tip fixture** in the workspace.
2. Once you place it in the chosen slot or location, **do NOT move it during the entire punch validation session**.
3. Make sure the punch target is reachable from multiple Husky base positions.
4. After moving the robot base or jogging the arm, double-check that you did not accidentally bump the fixture.
5. If visual alignment is hard because the background is cluttered, put a sheet of **A4 paper behind the target** to make the punch tip easier to see.

<!-- SCREENSHOT: External punch target fixture in the workspace -->

### 6.3 Collect Punch Validation Data

1. Set the **"Batch"** slider to `3` (punch). This ensures the data is saved to the `punch_validation/` folder.
2. On the UR teach pendant, switch from **Remote Mode** to **Local Mode** so you can use **free-drive** for manual alignment.
3. If it helps, switch to the **tool coordinate frame / tool perspective** while jogging so motion is relative to the tool axes.

For each take:

1. **Drive the Husky base** to a new position.
2. **Manually jog the robot** using local mode / free-drive until the physical punch tip matches the target tip.
   - The alignment should be done physically, not just visually in PyBullet.
   - Re-check that the fixture has not moved.
3. In the monitor GUI, click **"Record Punch Take"** to record the current configuration.
   - In older notes this may be described as "Record Current Calibration Configuration"; for the punch workflow, use the dedicated **"Record Punch Take"** button when available.
   - This records the joint configuration, base mocap pose, and computes the world-frame punch tip position via forward kinematics.
4. Optionally click **"Export calib data to json"** as a safety backup while collecting data.
5. Repeat from a **different base position**: move the base, re-align the punch tip to the same fixed target, and record again.

<!-- SCREENSHOT: Robot with punch tool aligned to external target -->
<!-- SCREENSHOT: GUI showing "Record Punch Take" button -->

Aim for about **4-6 takes** from diverse base positions. Collect more if you want extra coverage.

### 6.4 Save Punch Validation Data

When done collecting:

1. Click **"Save Punch Validation Data"** in the GUI.
2. The data will be saved to `$CALIB/YYYYMMDD/punch_validation/punch_validation_<arm>_YYYYMMDD_HHMM.json` on the gdrive ([§4.3](#43-data-folder-structure)).

---

## 7. Running the Calibration Pipeline

After collecting all J0 and J1 data (and optionally validation data), run the calibration pipeline to compute the extrinsic calibration transformation.

### 7.1 Prepare the Config File

1. Navigate to the calibration **data** folder on the gdrive ([§4.3](#43-data-folder-structure)):
   ```bash
   CALIB="/home/su/Insync/2025-03 Husky Assembly/data_experiment/calibration_data"
   cd "$CALIB"
   ```

2. If this is a new date folder, **copy an existing `config.yaml`** into your date folder
   (or start from the template kept in the repo):
   ```bash
   mkdir -p YYYYMMDD
   cp 20260623/config.yaml YYYYMMDD/config.yaml   # Alice's (single arm); 20261016 is Cindy's
   # or: cp ~/ros2_ws/src/husky-assembly-teleop/data/calibration_data/_data_template/config.yaml YYYYMMDD/config.yaml
   ```

3. Edit the config file to match your setup:
   ```bash
   nano YYYYMMDD/config.yaml
   ```

   Key settings to verify (full `config.yaml` field reference):
   ```yaml
   # Which data batches to process (scripts 0-1)
   data_batches:
     - "j0"
     - "j1"

   # Which batch 3_verify_calibration.py uses to verify
   validation_data_batch: "validation"   # "j0" / "j1" / "validation"

   # Robot settings - make sure these match your robot
   robot_name: "0805"   # "0806" Cindy (dual-arm), "0804" Alice / "0805" Belle (single-arm)
   arm: "left"          # "left" or "right" (dual-arm only)

   # Punch tool TCP offset tool0->tip (meters), per arm, from UR 4-point calibration
   punch_tool:
     left:
       offset_xyz: [0.0038, 0.0001, 0.11896]
     right:
       offset_xyz: [0.0038, 0.0001, 0.11896]

   # Arm to analyze in 4_punch_validation.py — independent of `arm` above (see §8)
   punch_validation:
     arm: null            # "left" / "right" / null
   ```

   `robot_name` also picks the robot's own kinematically calibrated URDF for the
   scripts (from the robot registry, `RobotSpec.urdf_path`), so a wrong serial
   here gives slightly wrong forward kinematics, not an error.

   > **Belle, 2026-10-14**: `$CALIB/20261013/config.yaml` has `robot_name: "0805"`
   > and her own 4-point TCP, `[0.58, 1.58, 110.54]` mm → `offset_xyz: [0.00058, 0.00158, 0.11054]`.
   > ! Check the metres twice: her first five punch takes were recorded with
   > `1.1054` (a digit slipped), which puts the punch tip a metre off. They were
   > recomputed with `0.11054`; the originals are in
   > `punch_validation/archive_wrong_offset_1.1054m/`.

### 7.2 Set the Date Folder

Both calibration dates live in **one file** — [`husky_assembly_teleop/__init__.py`](../husky_assembly_teleop/__init__.py#L89):

```bash
nano ~/ros2_ws/src/husky-assembly-teleop/husky_assembly_teleop/__init__.py
```

```python
CALIBRATION_DATE = 'YYYYMMDD'            # live app: records into / loads from
CALIBRATION_ANALYSIS_DATE = 'YYYYMMDD'   # offline scripts 0_ ... 4_: analyse
```

Normally set **both** to the same folder. Keep them different only when you
deliberately want the live app recording into a new folder while you re-analyse
an older capture.

The monitor reads each robot's base calibration
(`calibrated_transformation_<serial>_rhino.json`) from `CALIBRATION_DATE`, unless
the robot has its own folder in `BASE_CALIBRATION_DATE_BY_ROBOT` (same file, just
below). The robots are calibrated on different days, so after calibrating one
robot, point its entry at the new folder. Since 2026-10-15 every robot has one,
so `CALIBRATION_DATE` decides no robot's base:

```python
BASE_CALIBRATION_DATE_BY_ROBOT = {
    'Alice': '20260623',  # 0804, the newest Alice calibration
    'Belle': '20261013',  # 0805, the newest Belle calibration (recorded 2026-10-14)
    'Cindy': '20261016',  # 0806 left arm, re-calibrated 2026-10-16 (fallback: '20260916')
}
```

During a robot's first session its file does not exist yet, so the monitor warns
*Base calibration file not found* for it and draws its base uncalibrated. That is
expected during data collection: the recorded data uses the raw mocap poses.

**Belle's calibration (2026-10-14, folder `20261013`)**, for comparison with the
next one:
- Validation (`3_verify_calibration.py`, 17 poses): the calib tool's offset from
  tool0 is 22.8–24.7 mm long (mean `[17.3, -1.8, 16.1]` mm), each pose within
  1.6 mm of the mean (std 0.7 mm), axes within 0.5°.
- Punch validation (`4_punch_validation.py`, 5 takes): the tip within 1.5 mm of its
  mean (std 0.3 mm), pairwise at most 3.0 mm.
- `base_mocap_from_base_footprint` turns by +90° about z where Alice's turns by
  −90°: Belle's base tracker is mounted 180° turned compared with Alice's. Expected.

At start-up the monitor prints the file it loaded for every robot, e.g.
`Base calibration of Alice (0804): 20260623/calibrated_transformation_0804_rhino.json`.

There is nothing to edit in `config_loader.py` — its `DEFAULT_DATE_FOLDER` is a
re-export of `CALIBRATION_ANALYSIS_DATE`. Every script prints the folder it is
working on when it starts:

```
[config_loader] Using calibration date folder: YYYYMMDD
```

> Changing `CALIBRATION_DATE` only takes effect on the **next launch** of the
> monitor — restart the app after editing, or it keeps writing into the folder
> it read at startup.

### 7.3 Run the Pipeline

```bash
cd ~/ros2_ws
source venv/bin/activate
source install/setup.bash
cd ~/ros2_ws/src/husky-assembly-teleop/data/calibration_data/   # the scripts (repo)
python run_calibration_pipeline.py
```

The scripts run from the repo but read and write the **data** in
`$CALIB/YYYYMMDD/` on the gdrive ([§4.3](#43-data-folder-structure)).

The pipeline runs 4 steps in sequence:

| Step | Script | Description |
|------|--------|-------------|
| 1 | `0_circle_fitting.py` | Fit circles to the mocap data for J0 and J1 joints |
| 2 | `1_calibration_analysis.py` | Analyze circle fits and compute the base frame transformation |
| 3 | `2_convert_and_visualize_transformation.py` | Convert and save the calibrated transformation |
| 4 | `3_verify_calibration.py` | Verify calibration quality using validation data |

After completion, you should see:
- `calibrated_transformation_<robot>_rhino.json` (e.g. `..._0805_rhino.json` for Belle) in
  the gdrive date folder `$CALIB/YYYYMMDD/` -- the main output, and the file the
  monitor loads for `CALIBRATION_DATE`
- Various analysis plots (`.png` files) and logs, next to the data they came from
- A pipeline summary showing all steps passed

<!-- SCREENSHOT: Terminal showing pipeline running and completing successfully -->
<!-- SCREENSHOT: Example calibration analysis result plot -->

---

## 8. Running Punch Validation Analysis

After the calibration pipeline has completed and you have collected punch validation data:

```bash
cd ~/ros2_ws/src/husky-assembly-teleop/data/calibration_data/   # the scripts (repo)
python 4_punch_validation.py
```

The script uses the active date folder from `CALIBRATION_ANALYSIS_DATE`
([`__init__.py:104`](../husky_assembly_teleop/__init__.py#L104)) — it prints it on
the first line — and reads
`$CALIB/YYYYMMDD/punch_validation/` on the gdrive ([§4.3](#43-data-folder-structure)).
The plots are written into that same folder. If both arms have punch
validation files in that folder, set `punch_validation.arm` in
`$CALIB/YYYYMMDD/config.yaml` to `left` or `right`. This setting
is separate from the main `arm` field used in the calibration pipeline.

This script will:
1. Find all `punch_validation_*.json` files in the date folder
2. Analyze position consistency (how closely the punch tip maps to the same world point from different base positions)
3. Analyze orientation consistency
4. Analyze base pose diversity
5. Generate plots and print a summary

The output includes:
- `punch_validation_position_<arm>.png` -- position error analysis
- `punch_validation_3d_<arm>.png` -- 3D visualization of punch tip positions
- `punch_validation_diversity_<arm>.png` -- base pose diversity analysis
- A text summary with mean/max position errors

<!-- SCREENSHOT: Example punch validation position plot -->
<!-- SCREENSHOT: Example punch validation 3D plot -->

---

## 9. Troubleshooting

### Monitor does not start
- Make sure you have activated the virtual environment and sourced the workspace:
  ```bash
  source venv/bin/activate
  source install/setup.bash
  ```

### Robot model does not match real robot
- Check that the ROS2 driver is running on the Husky (Section 2.3)
- Check ROS domain ID: `echo $ROS_DOMAIN_ID`
- Check that topics are visible: `ros2 topic list`
- Verify joint states are streaming: `ros2 topic hz /a200_0805/ur5e/rate_limiter/joint_states` (Belle; `/a200_0806/left_ur5e/...` on Cindy)
- Check the start-up log names the robot you expect -- without `ROS_DOMAIN_ID` the monitor connects to Cindy ([§3.1](#31-launch-the-monitor-node))

### Workstation sees no topics from the robot

Symptom: `ros2 topic echo /a200_0805/ur5e/joint_states` says *does not appear to
be published yet*, and `ros2 topic list` on the workstation shows only
`/parameter_events` and `/rosout`. Narrow it down in this order (it took the
whole of the first Belle session, 2026-10-13, to find):

1. **Workstation terminal**: `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` exported,
   then `ros2 daemon stop` and list again ([§2.4](#24-verify-ros2-connection)).
2. **Is the robot reachable?** `ping 192.168.0.114`. Slow replies (tens of ms)
   mean the robot is on Wi-Fi.
3. **On the robot** (SSH), does the robot see its own topics, and with which settings?
   ```bash
   echo "DOMAIN=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION CYCLONE=$CYCLONEDDS_URI"
   ros2 topic list | grep ur5e
   ip -br addr | grep 192.168
   ```
   - no `ur5e` topics → the driver is not running; relaunch it ([§2.3](#23-start-the-ur5e-ros2-driver-on-the-husky)).
   - `DOMAIN` / `RMW` differ from the workstation → export them on the robot and relaunch.
   - both match and `CYCLONE` is empty, with two interfaces listed (e.g. `br0`
     and `wlp2s0`) → Cyclone is on the wrong interface. Relaunch the driver
     with `CYCLONEDDS_URI` set to the `192.168.0.x` interface (launch (b) in [§2.3](#23-start-the-ur5e-ros2-driver-on-the-husky), "Belle: which launch").

### Arm topics listed, but no data arrives

Symptom: `ros2 topic list` on the workstation shows the `/a200_0805/ur5e/...`
topics, yet `ros2 topic hz .../ur5e/rate_limiter/joint_states` keeps saying *does
not appear to be published yet*. First tell the network apart from the arm:

```bash
ros2 topic echo --once /a200_0805/platform/joint_states   # the husky platform, not the arm
```

- **Platform data arrives, arm data does not** (seen 2026-10-14) → the network is
  fine; the UR driver is running but not talking to the arm. Check the launch
  terminal on the robot for `Connected to reverse interface. Ready to receive
  control commands.`, then the pendant: arm powered, brakes released, **Remote /
  External Control**, program running (a protective stop or e-stop stops it).
  If still nothing, Ctrl+C the driver and relaunch it ([§2.3](#23-start-the-ur5e-ros2-driver-on-the-husky)).
- **Nothing arrives at all** → network side; see *Workstation sees no topics
  from the robot* above.

### MoCap not connecting

Symptom: the monitor logs `mocap client connected: False` in red, and the husky
in the PyBullet view sits at the world origin instead of where it really is.
The arms still follow the real robot, because they come from `/joint_states`;
only the **base** pose is driven by mocap, so a stale base with live arms is the
signature of this problem rather than a separate bug.

Work through these in order -- each step tells you whether to stop or continue:

1. **Can you reach the OptiTrack PC at all?**
   ```bash
   ping 192.168.0.28
   ```
   If this replies, skip to step 4. If not, continue.

2. **Has its DHCP lease moved?** The address is not fixed (Section 1.4). Ask the
   router what it knows, then sweep the subnet for what is actually alive:
   ```bash
   getent hosts DESKTOP-T1HDH98          # what the Fritz!Box has on record
   for i in $(seq 1 254); do (ping -c1 -W1 192.168.0.$i >/dev/null 2>&1 \
       && echo "alive: 192.168.0.$i") & done; wait
   for i in $(seq 1 254); do n=$(getent hosts 192.168.0.$i | awk '{print $2}'); \
       [ -n "$n" ] && echo "192.168.0.$i -> $n"; done
   ```
   The router keeps **stale** records for addresses a machine no longer holds,
   so a name that resolves but does not ping is an old lease, not a live host.
   Trust the ping sweep over the name lookup. If the PC turns up on a different
   address, update `MOCAP_IP` and you are done.

3. **Is it on our network segment at all?** This check does not depend on IPv4
   being configured correctly, so it separates "wrong address" from "not
   connected":
   ```bash
   ping6 -c 4 -I eno1 ff02::1        # every IPv6 host on this segment answers
   ```
   Match the replies against `ip neigh show` to identify them by MAC. If the
   OptiTrack PC is absent here, no IP setting will help -- its LAN adapter is
   either **disabled in Windows** or genuinely unplugged. On that machine run
   `ipconfig /all` (plain `ipconfig` hides adapters) or `getmac /v`, and check
   `ncpa.cpl` for a greyed-out adapter to enable. Note that a *disabled* adapter
   does not appear in `ipconfig` output at all, while an unplugged one still
   shows as `Media disconnected`.

4. **Did Motive pick up the interface?** Motive enumerates adapters only at
   startup, so an adapter enabled after launch will not be in the **Local
   Interface** list. Fully close and re-open Motive, then select the
   `192.168.0.x` address (Section 1.4).

5. **Check the remaining settings:** streaming **enabled**, **Unicast** (not
   Multicast), and `CLIENT_IP` / `MOCAP_IP` in `husky_monitor.py` matching your
   workstation and the OptiTrack PC respectively.

> **Multicast does not help here.** It only changes how NatNet addresses its
> packets; it cannot create a network path that does not exist. It is also
> harder to route than unicast and can be dropped by IGMP snooping on the
> managed switch. Leave Motive on **Unicast**.

**To keep working without mocap**, switch the monitor to robot-centric mode --
the base then comes from each movement's authored `start_state.robot_base_frame`
instead of being measured. Flip all three flags together in
[`husky_monitor.py`](../husky_assembly_teleop/husky_monitor.py#L312):
```python
USE_MOCAP = 0
USE_CELL_STATE_BASE_POSE = 1
BAR_ACTION_MOCAP_ACCURACY_TEST = 0
```

### E-stop triggers during trajectory
- Release the e-stop and press "Go" on the remote
- Re-power the UR5e if needed (Section 2.2)
- Load the cell state again, plan, and re-execute from the beginning
- You must collect a **complete** trajectory -- partial data cannot be used

### Both huskies paired to one e-stop
- Turn all huskies and e-stops off (press the red button until lights stop flashing)
- For each pair, one at a time:
  - Put the e-stop in pairing mode (press and hold the button until both lights flash)
  - Power on the corresponding husky

### Husky does not respond to joystick
- This is likely a bug in the ROS joy node
- Reboot the Husky fixes it
- Alternative: use keyboard teleop:
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/a200_0806/joy_teleop/cmd_vel
  ```

### Pipeline fails at a specific step
- Check the terminal output for error messages
- Make sure the `config.yaml` has the correct settings
- Make sure `CALIBRATION_ANALYSIS_DATE` in [`husky_assembly_teleop/__init__.py`](../husky_assembly_teleop/__init__.py#L104) matches your data folder — every script prints `[config_loader] Using calibration date folder: ...` at startup, so check that line first
- Verify that the data files exist in the expected subfolders (j0/, j1/) of the **gdrive** date folder `$CALIB/YYYYMMDD/` ([§4.3](#43-data-folder-structure)), not in the repo
- `Config file not found: ...` → the date folder is missing on this machine: check Insync has finished syncing `data_experiment/calibration_data/`, and that `EXPERIMENT_DATA_DIRECTORY` ([§0.1](#01-new-workstation-setup-one-time)) points at this machine's gdrive mount

### Monitor warns "Base calibration file not found"
- **For the robot you are calibrating right now this is expected**: the file is what the pipeline produces ([§3.1](#31-launch-the-monitor-node)). Only act on it for a robot that has been calibrated before
- The monitor loads `calibrated_transformation_<robot>_rhino.json` from `$CALIB/<CALIBRATION_DATE>/` on the gdrive, or from the robot's own folder in `BASE_CALIBRATION_DATE_BY_ROBOT` (`__init__.py`, [§7.2](#72-set-the-date-folder)); the warning names the folder it looked in. If it is missing, the base pose runs **uncalibrated**
- Check that the folder exists and holds that file, that Insync has synced it, and that `EXPERIMENT_DATA_DIRECTORY` ([§0.1](#01-new-workstation-setup-one-time)) is right for this machine
