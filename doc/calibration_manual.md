# Husky Extrinsic Calibration Manual

This manual walks you through the complete extrinsic calibration procedure for the Husky robot, from powering on the motion capture system to running the calibration pipeline. Follow each section in order.

> **Audience**: This guide assumes basic familiarity with the robot hardware but provides detailed command-line instructions for each step.

---

## Table of Contents

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

## 1. OptiTrack Motion Capture Setup

### 1.1 Power On & Start Motif

1. Turn on the PoE (Power over Ethernet) switch that powers the OptiTrack cameras. Wait for all camera indicator LEDs to turn on.
2. On the OptiTrack PC, boot into **Ubuntu**.
3. Launch the **Motif** software.

> For detailed instructions on starting up the OptiTrack system and using Motif, refer to the [OptiTrack Motif Documentation](https://docs.optitrack.com/motive).

<!-- SCREENSHOT: OptiTrack cameras powered on with LEDs visible -->
<!-- SCREENSHOT: Motif software main screen after startup -->

### 1.2 Calibrate the Camera System

If the cameras have been moved or it has been more than a week since the last calibration, you should recalibrate the system.

> Follow the official [OptiTrack Calibration Guide](https://docs.optitrack.com/motive/calibration) for the wanding and ground plane procedure.

<!-- SCREENSHOT: Motif calibration wanding screen -->
<!-- SCREENSHOT: Motif calibration result showing quality metrics -->

### 1.3 Create / Verify Rigid Bodies

If you need to create a new rigid body (e.g., for the robot base or flange tracker):

1. In Motif, select the markers that form the rigid body.
2. Right-click and choose **Create Rigid Body**.
3. Note the **Rigid Body ID** assigned by Motif -- you will need this for the PyBullet configuration.
4. In the code, the rigid body IDs are configured in two places in [`husky_world.py`](husky_assembly_teleop/husky_world.py):
   - **Robot base tracker**: the `mocap_id` parameter in the `create_husky_with_end_effectors` call (e.g., `mocap_id=4617` at [line 97](husky_assembly_teleop/husky_world.py#L97)).
   - **Flange/tool tracker**: the second argument to `TrackedObject` (e.g., `4616` at [line 163](husky_assembly_teleop/husky_world.py#L163)).

   Make sure the IDs you see in Motif match the IDs in these two locations.

<!-- SCREENSHOT: Motif rigid body creation dialog -->
<!-- SCREENSHOT: Motif rigid body properties showing the ID number -->

### 1.4 Network Connection (MoCap to Workstation)

The workstation must be on the same network as the OptiTrack PC to receive streaming data.

**Key IP addresses:**

| Device | IP Address |
|--------|-----------|
| Workstation (your PC) | `192.168.0.7` |
| OptiTrack PC | `192.168.0.117` |

These are configured in the code at [`husky_monitor.py:45-46`](husky_assembly_teleop/husky_monitor.py#L45-L46):
```python
CLIENT_IP = '192.168.0.7'   # Your workstation IP
MOCAP_IP = '192.168.0.117'  # OptiTrack PC IP
```

**To verify the network connection:**

1. Make sure your workstation is connected to the Husky switch (either via Ethernet cable or the lab network).
2. Verify you can ping the OptiTrack PC:
   ```bash
   ping 192.168.0.117
   ```
3. In Motif, go to **Settings > Streaming** and make sure:
   - Streaming is **enabled**
   - The **Local Interface** matches the OptiTrack PC IP (`192.168.0.117`)
   - **NatNet** streaming is active

<!-- SCREENSHOT: Motif streaming settings panel showing enabled streaming and correct IP -->
<!-- SCREENSHOT: Terminal showing successful ping to 192.168.0.117 -->

> **Tip**: If the mocap connection fails in the monitor, you will see a red log message. Check the Motif streaming settings and verify both IPs are correct.

---

## 2. Robot Startup & ROS2 Nodes

This section covers starting up the Husky robot and the UR5e arm ROS2 drivers. Make sure the motion capture system is already running before proceeding.

### 2.1 Power On the Husky

1. Press the **power button** on the Husky.
2. Turn on the **remote e-stop** and press **"Go"**.
   > Do NOT start multiple huskies at the same time -- sometimes both huskies connect to one e-stop if started in parallel.
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

   # For Alice (single-arm, serial 0804):
   ssh administrator@192.168.0.113
   # Password: clearpath
   ```

2. Launch the appropriate ROS2 driver:
   ```bash
   # For Cindy (dual arm):
   ros2 launch crl-husky crl_dual_ur5e.launch.py namespace:='/a200_0806'

   # For Alice (single arm):
   ros2 launch crl-husky crl_single_ur5e.launch.py namespace:='/a200_0804'

   # For Alice with gripper:
   ros2 launch crl-husky crl_single_ur5e_gripper.launch.py namespace:='/a200_0804'
   ```

   > Add `use_fake_hardware:=true` to use a simulated UR5e instead of the real robot (for testing).

### 2.4 Verify ROS2 Connection

On your workstation, set the correct ROS2 domain and middleware:

```bash
export ROS_DOMAIN_ID=86   # 86 for Cindy, 84 for Alice, 85 for Belle
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Verify the connection:

```bash
# List all available topics
ros2 topic list

# Check if joint states are streaming (should see data at ~500Hz)
ros2 topic echo /a200_0806/ur5e/joint_states

# Check the frequency
ros2 topic hz /a200_0806/ur5e/joint_states

# Visualize the full node graph
rqt
```

<!-- SCREENSHOT: Terminal showing ros2 topic list output with ur5e topics visible -->
<!-- SCREENSHOT: rqt node graph showing connected nodes -->

**Husky Reference Table:**

| Name | Serial | IP | ROS Domain |
|------|--------|-----|-----------|
| Alice | 0804 | 192.168.0.113 | 84 |
| Belle | 0805 | 192.168.0.114 | 85 |
| Cindy | 0806 | 192.168.0.115 | 86 |

> **Warning**: The E-Stop on the Husky does NOT stop the UR5e! Always use the **remote E-Stop** when operating the robot arms.

---

## 3. Launching the Monitor (Workstation Side)

The monitor is the PyBullet-based GUI that you use to plan motions, execute trajectories, and collect calibration data.

### 3.1 Launch the Monitor Node

Open a terminal on your workstation and run:

```bash
cd ~/ros2_ws
source venv/bin/activate
source install/setup.bash
ros2 run husky_assembly_teleop husky_monitor
```

This will open the **PyBullet monitor window**.

<!-- SCREENSHOT: PyBullet monitor window after startup, showing the grey robot model -->

### 3.2 Verify the Connection

If the ROS2 connection is working correctly, the **grey robot model** in PyBullet should match the **real robot's joint configuration**. Move the real robot's joints and verify that the PyBullet model moves accordingly.

<!-- SCREENSHOT: Side-by-side of real robot and PyBullet model in matching configuration -->

**If the robot model does NOT match the real robot**, check:
1. Is the ROS2 driver running on the Husky side? (Section 2.3)
2. Is the robot powered on and in **External Control** mode? (Section 2.2)
3. Are the ROS2 domain ID and middleware set correctly? (Section 2.4)
4. Try running `rqt` to verify topics are visible.

---

## 4. Calibration Data Collection

This is the main calibration data collection procedure. You will collect data for **J0** (shoulder pan joint rotation) and **J1** (shoulder lift joint rotation) batches.

### 4.1 Understanding the GUI Controls

The monitor GUI has several sliders and buttons relevant to calibration:

**Sliders:**

| Slider | Description |
|--------|-------------|
| `robot id` | Select which robot to control (if multiple) |
| `arm id (0:L,1:R)` | Select left or right arm (for dual-arm robot) |
| `traj time` | Duration of the trajectory in seconds (range: 1-60s) |
| `Traj viz time` | Scrub through a planned trajectory to preview it (0.0 to 1.0) |
| `Mode (0:validation, 1:data_collection)` | Toggle between validation and data collection mode |
| `Batch (0:j0,1:j1,2:valid,3:punch)` | Select which calibration batch you are collecting |
| `Robot Cell State` | Select which pre-defined cell state to load |
| `Joint Trajectory` | Select which pre-defined joint trajectory to load |

**Buttons:**

| Button | Description |
|--------|-------------|
| `Load Robot Cell State` | Load the selected cell state as the arm goal configuration |
| `Load Joint Trajectory` | Load the selected joint trajectory for execution |
| `Plan S.Arm to conf target` | Plan a motion from the current configuration to the goal |
| `Exec S.Arm Traj` | Execute the planned single-arm trajectory on the real robot |
| `Execute calib traj` | Execute the loaded calibration trajectory with move-stop-record |
| `Export calib data to json` | Save collected calibration data to a JSON file |
| `Toggle Goal/Trajectory` | Switch the ghost model between showing goal (blue) or trajectory (green) |

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

1. Use the **"Robot Cell State"** slider to select the desired cell state. These are pre-defined starting configurations for the calibration trajectories.
2. Click **"Load Robot Cell State"**.
   - The goal configuration will be shown as a **blue ghost model** in PyBullet.

<!-- SCREENSHOT: PyBullet showing blue ghost model at the target cell state -->

#### Step 3: Plan and Preview the Motion

1. Set the **"traj time"** slider to a reasonable duration. For calibration, a longer time (e.g., **20-40 seconds**) is recommended so the robot moves slowly.
2. Click **"Plan S.Arm to conf target"** to plan a motion from the current configuration to the goal.
3. Use the **"Traj viz time"** slider to scrub through the planned trajectory and preview the motion.
   - Click **"Toggle Goal/Trajectory"** if needed to switch to the green trajectory visualization.
4. Verify the trajectory looks safe and collision-free.

<!-- SCREENSHOT: PyBullet showing green trajectory visualization with Traj viz time slider -->

#### Step 4: Execute the Motion to the Cell State

1. **Keep your hand on the e-stop** at all times during execution.
2. Click **"Exec S.Arm Traj"** to execute the planned trajectory.
3. Wait for the robot to reach the cell state configuration.

<!-- SCREENSHOT: Robot moving to cell state (optional action photo) -->

#### Step 5: Load and Execute the Calibration Trajectory

1. Use the **"Joint Trajectory"** slider to select the appropriate calibration trajectory file for the current cell state.
2. Click **"Load Joint Trajectory"** to load it.
3. Click **"Execute calib traj"** to start the calibration trajectory.
   - This will automatically perform **move-stop-record** for each waypoint in the trajectory.
   - At the end, it will **automatically save** the recorded data to a JSON file.
4. You should see the data being saved in the terminal output.

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

The collected data is automatically organized into folders:

```
data/calibration_data/
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

1. **Mount the punch tool** onto the robot's tool flange.
2. **Calibrate the punch tool offset** using the **4-point TCP calibration** procedure on the UR teach pendant:
   - Go to **Installation > TCP** on the tablet.
   - Follow the 4-point method to determine the X, Y, Z offset from the flange to the punch tip.
   - Record the offset values.
3. **Update the config**: Enter the calibrated offset in the `config.yaml` file:
   ```yaml
   punch_tool:
     offset_xyz: [X, Y, Z]   # Values from 4-point calibration, in meters
   ```
   The config file is located at `data/calibration_data/YYYYMMDD/config.yaml`.

<!-- SCREENSHOT: UR teach pendant showing 4-point TCP calibration procedure -->
<!-- SCREENSHOT: Punch tool mounted on the robot flange -->

### 6.2 Set Up the Punch Target

1. Place or use the **external punch target** (a fixed point in the workspace). **Do NOT move this target during the entire punch validation session.**
2. Make sure the punch target is in a location reachable from multiple base positions.

<!-- SCREENSHOT: External punch target fixture in the workspace -->

### 6.3 Collect Punch Validation Data

1. Set the **"Batch"** slider to `3` (punch).

For each take:

1. **Drive the Husky base** to a new position.
2. **Manually jog the robot** (via teach pendant or free-drive) to align the punch tool tip with the external punch target. The punch tip should physically touch/match the target point.
3. In the monitor GUI, click **"Record Punch Take"** to record the current configuration.
   - This records the joint configuration, base mocap pose, and computes the world-frame punch tip position via forward kinematics.
4. Repeat from a **different base position**: drive the base somewhere else, re-align the punch, and record again.

<!-- SCREENSHOT: Robot with punch tool aligned to external target -->
<!-- SCREENSHOT: GUI showing "Record Punch Take" button -->

Collect at least **8-10 takes** from diverse base positions.

### 6.4 Save Punch Validation Data

When done collecting:

1. Click **"Save Punch Validation Data"** in the GUI.
2. The data will be saved to `data/calibration_data/YYYYMMDD/punch_validation/punch_validation_YYYYMMDD_HHMM.json`.

---

## 7. Running the Calibration Pipeline

After collecting all J0 and J1 data (and optionally validation data), run the calibration pipeline to compute the extrinsic calibration transformation.

### 7.1 Prepare the Config File

1. Navigate to the calibration data folder:
   ```bash
   cd ~/ros2_ws/src/husky-assembly-teleop/data/calibration_data/
   ```

2. If this is a new date folder, **copy an existing `config.yaml`** into your date folder:
   ```bash
   cp 20260126/config.yaml YYYYMMDD/config.yaml
   ```

3. Edit the config file to match your setup:
   ```bash
   nano YYYYMMDD/config.yaml
   ```

   Key settings to verify:
   ```yaml
   # Which data batches to process
   data_batches:
     - "j0"
     - "j1"

   # Which batch to use for verification
   validation_data_batch: "j1"   # or "validation" if you collected separate validation data

   # Robot settings - make sure these match your robot
   robot_name: "0806"   # "0806" for Cindy (dual-arm), "0804" for Alice (single-arm)
   arm: "left"          # "left" or "right" (for dual-arm only)

   # Punch tool offset (if doing punch validation)
   punch_tool:
     offset_xyz: [0.0038, 0.0001, 0.11896]   # From 4-point calibration
   ```

### 7.2 Set the Date Folder in config_loader.py

Edit `config_loader.py` to point to your date folder:

```bash
nano data/calibration_data/config_loader.py
```

Change the `DEFAULT_DATE_FOLDER` to your date:

```python
DEFAULT_DATE_FOLDER = 'YYYYMMDD'   # Change to your date folder
```

### 7.3 Run the Pipeline

```bash
cd ~/ros2_ws/src/husky-assembly-teleop/data/calibration_data/
python run_calibration_pipeline.py
```

The pipeline runs 4 steps in sequence:

| Step | Script | Description |
|------|--------|-------------|
| 1 | `0_circle_fitting.py` | Fit circles to the mocap data for J0 and J1 joints |
| 2 | `1_calibration_analysis.py` | Analyze circle fits and compute the base frame transformation |
| 3 | `2_convert_and_visualize_transformation.py` | Convert and save the calibrated transformation |
| 4 | `3_verify_calibration.py` | Verify calibration quality using validation data |

After completion, you should see:
- `calibrated_transformation_0806.json` in the date folder (the main output)
- Various analysis plots (`.png` files)
- A pipeline summary showing all steps passed

<!-- SCREENSHOT: Terminal showing pipeline running and completing successfully -->
<!-- SCREENSHOT: Example calibration analysis result plot -->

---

## 8. Running Punch Validation Analysis

After the calibration pipeline has completed and you have collected punch validation data:

```bash
cd ~/ros2_ws/src/husky-assembly-teleop/data/calibration_data/
python 4_punch_validation.py
```

This script will:
1. Find all `punch_validation_*.json` files in the date folder
2. Analyze position consistency (how closely the punch tip maps to the same world point from different base positions)
3. Analyze orientation consistency
4. Analyze base pose diversity
5. Generate plots and print a summary

The output includes:
- `punch_validation_position.png` -- position error analysis
- `punch_validation_3d.png` -- 3D visualization of punch tip positions
- `punch_validation_diversity.png` -- base pose diversity analysis
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
- Verify joint states are streaming: `ros2 topic hz /a200_0806/ur5e/joint_states`

### MoCap not connecting
- Verify network connectivity: `ping 192.168.0.117`
- Check Motif streaming settings (Section 1.4)
- Verify `CLIENT_IP` and `MOCAP_IP` in `husky_monitor.py` match your setup

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
- Make sure the `DEFAULT_DATE_FOLDER` in `config_loader.py` matches your data folder
- Verify that the data files exist in the expected subfolders (j0/, j1/)
