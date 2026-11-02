# The old monitor

The old monitor is `husky_monitor.py` on `master`. A copy is kept in `husky_assembly_teleop/old/` for reference; it
does not run from there. This page says what changed, how to run the old monitor, and how to move a workspace to the
new one.

## What changed

| | Old monitor | New monitor |
|---|---|---|
| **Robots** | One per run, selected by `ROS_DOMAIN_ID` (84, 85, 86), Cyclone DDS. | Any number per run, selected with `robots`. All robots run Zenoh on `ROS_DOMAIN_ID=80`. |
| **Configuration** | Constants in `husky_monitor.py` (`USE_MOCAP`, `USE_DPG_UI`, `BAR_ACTION_...`), changed in the code. | ROS parameters on the command line. Features are plugins, selected with `plugins`. |
| **UI** | PyBullet window and Dear PyGui panel on the PC running the monitor. | Web page (viser) at http://localhost:8080, reachable from other PCs. PyBullet only as an optional debug window in some plugins. |
| **Mocap** | NatNet client inside the monitor; `CLIENT_IP` and `MOCAP_IP` set in the code; axis conversion and calibration in the monitor. | Separate `mocap_relay` node ([mocap_relay.md](mocap_relay.md)). Poses arrive calibrated, in the Rhino Z-up frame. |
| **Tracked objects** | `TrackedObject(...)` in `husky_world.py`. | `ctx.track_object(name, mocap_id, ...)` in a plugin, as in `plugins/mocap_probe.py`. |
| **Tools** | Fixed per robot in the code. | Defaults per robot, overridden with `tools`. Cindy's onboard launch takes one tool per arm (`gripper_left`, `gripper_right`; see crl-husky's README). |
| **Bar actions, live replan, servoing, accuracy tests** | Built in. | Step by step with `assembly_steps` and `movement_exec` (free moves, support moves, the free or held transfer, Robotiq and scaffolding grasps, steps by hand, the insert and the retreat with their compliant parts, the compliant Robotiq handoff, and the servo after a transfer, an approach or a ground bar's insert, on tool0 or on the mocap bar; a motion sent on Exec, placing the bar by hand, the Robotiq steps and the servo's first pass after Confirm). Live replan and the accuracy tests are not ported yet; use the old monitor. |
| **Quit** | Close the windows or Ctrl-C. | Ctrl-C once, then wait for the final hold commands. |

## Running the old monitor

The old monitor requires Cyclone DDS and the robot's own domain. Switch the robot to Cyclone first, as described in
crl-husky's README ([Still using Cyclone DDS](../../crl-husky/README.md#still-using-cyclone-dds-eg-for-the-old-monitor)).
The old monitor connects to Motive directly and does not use the relay.

```shell
git -C src/husky-assembly-teleop checkout master
git -C src/husky-assembly-teleop submodule update --init --recursive
python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=86   # robot's domain: 84 Alice, 85 Belle, 86 Cindy
ros2 run husky_assembly_teleop husky_monitor
```

To return to the new monitor, check out its branch, update the submodules and rebuild.

### Tracikpy (Linux only)

Only the old monitor (`old/`) uses tracikpy, and the dependencies in the last block of `requirements.txt`. The new
monitor runs without them; they stay until the remaining features are ported.

[Tracikpy](https://github.com/mjd3/tracikpy) is a minimal, fast inverse kinematics solver that takes a URDF and a
target pose and returns a solution. It only works on Linux, so the Grasshopper/Rhino design interface cannot use it.

```shell
sudo apt-get install libeigen3-dev liborocos-kdl-dev libkdl-parser-dev liburdfdom-dev libnlopt-dev libnlopt-cxx-dev
pip install -r requirements.txt
```

## Migrating a workspace to the new monitor

1. Update both repositories:
   ```shell
   git -C src/husky-assembly-teleop pull
   git -C src/husky-assembly-teleop submodule update --init --recursive
   git -C src/crl-husky pull
   ```
2. Install the new dependencies: the ROS packages and Python packages in the README's Install section. New compared
   with the old monitor: `viser`, `yourdfpy`, `trimesh`, `async-timeout`, `compas_robots`, and the Zenoh packages.
3. Switch to Zenoh: set `export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=80` in `~/.bashrc`, replacing any
   per-robot `ROS_DOMAIN_ID`, and set up the router as described in crl-husky's README
   ([Zenoh](../../crl-husky/README.md#zenoh)).
4. Set up time sync on this PC: a fixed IP slot and chrony, as in the README's Install section and crl-husky's
   [TIME_SYNC](../../crl-husky/TIME_SYNC.md#control-pc-ubuntu).
5. Build both packages with the venv's Python, as in the README:
   `python3 -m colcon build --symlink-install --packages-up-to husky_assembly_teleop` also builds crl-husky.
6. Verify: `ros2 topic list | grep a200` lists the robots' topics, and `chronyc -n sources` marks a time source `^*`
   (or none, if this PC is the reference).

For an agent setting up another lab PC beside the old monitor: [monitor_pc_setup.md](monitor_pc_setup.md).
