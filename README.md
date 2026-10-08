# husky_assembly_teleop

This is a python package for controlling huskies in the mocap space.

> ⚠️ **Heavy refactor; ✅ ready for peer testing**

# Installation
## Clone and update submodules
Install this library from source by cloning this repo to local and install from source.
```
git clone --recursive git@github.com:yijiangh/husky-assembly-teleop.git
```
> ⚠️ The monitor depends on [crl-husky](https://gitlab.inf.ethz.ch/crl/robot-control/crl-husky) (mocap relay, robot configs, `crl_husky_msgs`), cloned next to this repo as `src/crl-husky`. **Whenever you pull this repo, pull and rebuild crl-husky too**: the two change together.

The `--recursive` flag when cloning above is used for initializing all the git submodules. You can learn more about submodules [here](https://github.com/CGAL/cgal-swig-bindings/wiki/Installation).

Later in the development, whenever you need to update the submodules, issue the following:
```
git submodule update --init --recursive
```

### Updating Submodules

`git pull` will only update the main repository. To update all submodules recursively, you need to run:
```bash
git pull
git submodule update --init --recursive
git -C ../crl-husky pull
```

Alternatively, you can configure git to automatically update submodules during pulls:
```bash
git config --global submodule.recurse true
```

This setting ensures that commands like `git pull` will also update submodules recursively.

## Python Virtual Environment Setup

It is **strongly recommended** to use a Python virtual environment (venv) for development and running this package. This is especially important because the ROS2 package depends on Python packages (such as `compas_fab`, included as a submodule) that are being developed together with this package and installed locally in "editable" mode. Using a venv allows you to install and update these packages without affecting your global Python environment, which avoids conflicts and keeps your system clean. See [this ROS2 issue comment](https://github.com/ros2/ros2/issues/1094#issuecomment-2916700723) for more details.

If you use the `--system-site-packages` flag when creating your venv, you can use system-installed tools like `colcon` without needing to install them inside the venv. This approach has been used reliably for years.

### One-time setup

All commands in this README run in the workspace root, the folder that holds `src/`, `venv/` and `install/`. The monitor also needs [crl-husky](https://gitlab.inf.ethz.ch/crl/robot-control/crl-husky) in `src/crl-husky`: it provides the mocap relay, the robot configs and `crl_husky_msgs`.

```bash
# ROS packages
sudo apt install ros-humble-ur-msgs ros-humble-ur-dashboard-msgs ros-humble-control-msgs \
    ros-humble-controller-manager-msgs ros-humble-rmw-zenoh-cpp ros-humble-zenoh-cpp-vendor

# The venv
python3 -m venv venv --system-site-packages
source venv/bin/activate

# Ubuntu 22.04's pip (22.0.2) cannot install pyproject-only packages such as
# rs_data_structure in editable mode ("missing the 'build_editable' hook")
python3 -m pip install -U pip wheel

# Packages developed alongside this one, editable
python3 -m pip install -e src/husky-assembly-teleop/external/pybullet_planning
python3 -m pip install -e src/husky-assembly-teleop/external/compas_fab
python3 -m pip install -e src/husky-assembly-teleop/external/rs_data_structure
# --no-deps: husky_assembly_tamp pins compas_fab and rs_data_structure to git URLs,
# and pip would replace the editable copies above with those
python3 -m pip install --no-deps -e src/husky-assembly-teleop/external/husky_assembly_tamp

# The monitor's dependencies (versions as in requirements.txt); matplotlib is for husky_assembly_tamp
python3 -m pip install "viser==1.1.1" "yourdfpy==0.0.60" "trimesh==4.12.2" "async-timeout>=4.0" "compas_robots>=0.6" \
    "scipy>=1.8" "pybullet>=3.2" "compas>=2.0" "matplotlib==3.10.3"

# Check: all four should list a location under src/husky-assembly-teleop/external
python3 -m pip list | grep -iE "pybullet.planning|compas.fab|rs.data.structure|husky.assembly.tamp"
```

Set up Zenoh as described in crl-husky's README ([Zenoh](../crl-husky/README.md#zenoh)): `export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=80` in `~/.bashrc`, and the router once.

**Time sync (every control PC, once).** All machines on the lab network keep one time, so TF, logs and recorded data from the robots, the mocap PC and the control PCs line up. The control PCs are the time reference: the robots and the mocap PC follow them. So every PC that runs the monitor must take part, with a fixed IP in `.201`–`.210` reserved in the router and chrony configured as in crl-husky's [TIME_SYNC](../crl-husky/TIME_SYNC.md#control-pc-ubuntu). Check with `chronyc -n sources`: one source is marked `^*`, or none if this PC is the reference itself.

### Build and run

Build with the venv's Python; otherwise the installed scripts do not use the venv.

```bash
# Build terminal
source venv/bin/activate
python3 -m colcon build --symlink-install

# Running terminal
source venv/bin/activate
source install/setup.bash
ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['alice']" -p plugins:="['robot_control']"
```

<details>
<summary><strong>Troubleshooting <code>ros2</code> pkg installation</strong></summary>

<br/>

If you ran into issues related to `setuptools`, try the following **inside** the venv:

1. **Make sure venv tools are current**
   ```bash
   python3 -m pip install -U pip wheel
   ```

2. **Use a safe setuptools for Humble AND a new enough packaging**
   ```bash
   python3 -m pip install "setuptools==68.2.2" "packaging>=24.2"
   ```

   *Optional sanity check: see exactly what you’re using and from where*
   ```python
   python3 - <<'PY'
   import sys, setuptools, packaging
   print("python:", sys.executable)
   print("setuptools:", setuptools.__version__)
   print("packaging:", packaging.__version__)
   print("packaging path:", packaging.__file__)
   PY
   ```

3. **Then clean and rebuild:**
   ```bash
   rm -rf build/ install/ log/
   python3 -m colcon build --symlink-install
   ```

</details>


## Mocap Connection Setup
Read the [Mocap wiki](https://gitlab.inf.ethz.ch/crl/crl-wiki/-/wikis/HW/OptiTrack) for more information on how to create a rigid body in Motive and how to set the IP address of the OptiTrack server.

## Tracikpy (Linux-only, potentially obsolete)
> ⚠️ Only the old monitor (`old/`) uses tracikpy, and the dependencies in the last block of `requirements.txt`. The new monitor runs without them; they stay until the remaining features are ported.

Tracikpy is a minimal yet reliable and fast inverse kinematics solver that simply takes a URDF and a target pose and returns a solution.
However, it only works on Linux and is very hard to configure on a Windows machine. Thus, atm we couldn't use it for the Grasshopper/Rhino design interface.

Install system package dependencies for [tracikpy](https://github.com/mjd3/tracikpy):
```
sudo apt-get install libeigen3-dev liborocos-kdl-dev libkdl-parser-dev liburdfdom-dev libnlopt-dev libnlopt-cxx-dev
```
If using a Mac, you can install the dependencies using [Homebrew](https://stackoverflow.com/questions/19688424/why-is-the-apt-get-function-not-working-in-the-terminal-on-mac-os-x-v10-9-maver).

Install python dependencies:
```
pip install -r requirements.txt
```

# Code Structure

The monitor is a small core that ticks and owns the shared state; every feature is a plugin. Each folder's `__init__.py` lists its files.

| Folder | What is in it |
|---|---|
| `monitor.py`, `config.py` | The ROS node and its tick; the run configuration, read from the ROS parameters. |
| `plugin_api/` | What a plugin is (`HuskyPlugin`, `@register`) and what it gets (`PluginContext`). |
| `plugins/` | One module or package per feature. Start from `robot_control/` or `examples/` when writing one. |
| `robot_interface/` | The only code that talks to robots: base, arms, tools, mocap subscriptions. |
| `world/` | Measured state, mocap checks, kinematics, the scene planners read. |
| `ui/` | The viser web UI and shared widgets. |
| `old/` | The old monitor, for reference only. |

`bar_assembly_core/` (next to the package) is the core shared with the Rhino plugin and the planners: the design file format (`design/`), robots and scenes, and their mirrors in PyBullet and compas_fab. Its layers are listed in `bar_assembly_core/__init__.py`.

## Documentation

| Doc | What it covers |
|---|---|
| `AGENTS.md` | Conventions, build and test commands; for coding agents and people alike. |
| `doc/refactor_rationale.md` | Why the core is built the way it is. |
| `doc/plugin_roadmap.md` | Which plugins exist, and which old features are still to be ported. |
| `doc/scene_refactor_plan.md`, `doc/design_format.md`, `doc/ur_frames.md` | The scene, the design file format, the UR arm frames. |
| `tasks/` | Specs written while building a feature. |
| `doc/calibration_manual.md` and the other manuals | **Outdated**, written for the old monitor; each says at the top what is missing or different now. |
| `DOCKER.md` | **Unknown state**, not tested with the new monitor. |

> 🚧 A fuller overview (how the monitor fits with crl-husky, the planners and the Rhino design workflow) is still to be written.

# Usage

## 1. Start the mocap relay

The monitor does not connect to Motive. The `mocap_relay` node from crl-husky receives the OptiTrack stream and publishes each rigid body as a ROS topic, calibrated and in the Z-up world frame. The monitor and all other mocap consumers subscribe to these topics.

> ⚠️ Exactly one relay must run per ROS domain, and it must stay running while mocap is in use. Two relays in one domain would publish interleaved poses with different delays and possibly different calibrations. A relay therefore refuses to start if another one already runs in its domain. Stopping the relay stops mocap for every consumer in that domain.

```shell
ros2 launch crl_husky mocap.launch.py                           # Motive PC at 192.168.0.28 (default)
ros2 launch crl_husky mocap.launch.py server_ip:=192.168.0.117  # another Motive PC
```

Without a relay, no mocap is available. Details in crl-husky's `MOCAP_SETUP.md`.

## 2. Start the monitor

```shell
ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['alice']" -p plugins:="['robot_control']"
```

The UI is served at http://localhost:8080, or the next free port (8081, ...) if 8080 is in use; the log states the port. Other PCs on the network reach it at `http://<host-ip>:8080`.

**Quit:** press Ctrl-C once and wait. During shutdown the plugins send their final hold commands; a second Ctrl-C skips them.

**Stop all (Esc):** stops every robot and cancels every plugin task. The monitor keeps running.

### Panels

Some plugins show content in panels separate from the main control panel. Panels can be moved, docked, collapsed and resized:

- Drag the bar at the top of a panel to float it, or drop it on a screen edge or next to another docked panel to dock it.
- The control at the right end of that bar collapses and expands the panel.
- Drag the edge of a floating panel to resize it.

Example: `mocap_probe` shows the probe state in a **Probe** panel, collapsed on the right at start. Floated and enlarged, its text scales with the panel and is readable from a distance.

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `robots` | none | Robots to connect to, by serial or name: `0804`, `alice`, `a200-0806`, `Cindy`. `"[]"` runs without robots. |
| `plugins` | none | Plugins to load, see below. `health` is always added, unless `no_default_plugins:=true`. |
| `tools` | per robot | Only where the mounted tools differ from the defaults: `'<robot>:<tool>[,<tool>...]'`, one tool per arm in arm order. Tools: `robotiq`, `scaffolding_v1`, `scaffolding_v3`, `none`. Defaults: Alice and Belle `robotiq`, Cindy `scaffolding_v3,scaffolding_v3`. |
| `design_directory` | none | Design folder the `cell` plugin loads at startup, relative to `drive_root` (e.g. `data_design_study/260814_RobArch_support_ik`) or absolute; without it, pick one in its panel. |
| `data_directory` | `data/` of this repo | Root for meshes, URDFs and designs. |
| `drive_root` | `$HUSKY_DRIVE_ROOT` | Local copy of the project's Google Drive folder (`2025-03 Husky Assembly`), for experiment data; see [Experiment data](#experiment-data-google-drive). |
| `no_default_plugins` | `false` | `true` skips the default plugins (`health`). |
| `ghost_timeout` | `20.0` | Seconds a plugin's ghost robots stay after the last input in it; 0 keeps them. |

### Experiment data (Google Drive)

Experiment data lives in the project's Google Drive folder, synced to your PC (e.g. with Insync), not in this repo. Point the monitor and the analysis scripts at your local copy of the `2025-03 Husky Assembly` folder itself; the path above it differs per user.

```shell
# In ~/.bashrc, or each terminal: read by the monitor and the scripts
export HUSKY_DRIVE_ROOT="$HOME/Insync/<account>/Google Drive - Shared with me/2025-03 Husky Assembly"

# Or for one monitor run; the parameter wins over the variable
ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['alice']" -p plugins:="['base_exp']" \
    -p drive_root:="$HOME/Insync/<account>/Google Drive - Shared with me/2025-03 Husky Assembly"
```

Paths are given relative to this folder, e.g. `data_design_study/260814_RobArch_support_ik`; absolute paths still work, for data elsewhere. `base_exp` does not start without the root and its `data_experiment/base_exp/` folder; `cell` needs the root to load a relative design folder.

**Folders under the root are never created by the code.** Create them in Google Drive and sync them instead. The reason: with selective sync, a folder can exist in Drive but be missing on your PC, because it is not synced there. If the code then created it locally, Insync would upload it as a new folder, and Google Drive, unlike a normal file system, allows several folders with the same name side by side. You would end up with two `base_exp` folders, the runs split between them. The code only creates new folders inside an existing one (e.g. a folder per run), which cannot clash.

### Plugins

| Plugin | What it does |
|---|---|
| `health` | Loaded by default. Shows the status of each robot and tracked object, with Unlock, Resume and Reconnect. |
| `robot_control` | One tab per robot: drives the base, moves the arms (joint targets or compliance), stows them, operates the tools. |
| `obstacles` | Adds the fixed lab furniture to the scene as collision obstacles for the planners. |
| `debug_gizmo` | Debug only. Click any scene body with a world pose (e.g. an obstacle) in the 3D view, or pick it from the dropdown, then drag it with a gizmo or type its position and roll/pitch/yaw. **Add** puts temporary boxes and cylinders into the scene (planners avoid them); resize them with the small corner gizmo or by typing **Size**, and **Remove** them. **Log** prints position, quaternion, roll/pitch/yaw (and size), **Reset** puts a body back. Nothing is saved. |
| `mocap_probe` | Tracks the probe and records points with it, e.g. obstacle corners; exports JSON and loads it back. Shows its state in a separate panel, see [Panels](#panels). **Map coverage** walks a 15 × 15 m grid of 1 m cells, centred on the mocap origin, and colours each on the floor by how often the probe was tracked there (grey: not visited, red → green), with the mean ± std marker error per cell; export and load as JSON. |
| `cell` | Loads a design and shows the cell state of one movement at a time. Experimental. |
| `base_planner` | Plans a collision-free path for a base. Experimental: Commit only logs the path. |
| `arm_planner` | Plans a collision-free arm motion to a joint target. Experimental: Commit only logs the path. |
| `example_plot`, `example_ui`, `example_sequence`, `example_pybullet`, `example_recording`, `example_robot_state` | Minimal plugins, as templates for new ones. |

### Examples

```shell
# Drive one robot
ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['alice']" -p plugins:="['robot_control']"

# All three robots at once
ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['alice','belle','cindy']" -p plugins:="['robot_control']"

# Alice with the scaffolding tool instead of the Robotiq, Cindy with only one tool (on the left arm)
ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['alice','cindy']" \
    -p tools:="['alice:scaffolding_v3', 'cindy:scaffolding_v3,none']" -p plugins:="['robot_control']"

# Measure obstacles with the mocap probe (no robots needed)
ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="[]" -p plugins:="['mocap_probe']"

# Assembly: a design, the lab obstacles and the base planner
ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['alice','cindy']" \
    -p plugins:="['robot_control', 'cell', 'obstacles', 'base_planner']" -p design_directory:=data_design_study/260814_RobArch_support_ik

# Plan arm motions too
ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['cindy']" \
    -p plugins:="['robot_control', 'obstacles', 'arm_planner']"

# Try the plugin examples
ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['alice']" \
    -p plugins:="['example_plot', 'example_pybullet', 'example_sequence', 'example_ui']"
```

## What changed from the old monitor

The old monitor is `husky_monitor.py` on `master`. A copy is kept in `husky_assembly_teleop/old/` for reference; it does not run from there.

| | Old monitor | New monitor |
|---|---|---|
| **Robots** | One per run, selected by `ROS_DOMAIN_ID` (84, 85, 86), Cyclone DDS. | Any number per run, selected with `robots`. All robots run Zenoh on `ROS_DOMAIN_ID=80`. |
| **Configuration** | Constants in `husky_monitor.py` (`USE_MOCAP`, `USE_DPG_UI`, `BAR_ACTION_...`), changed in the code. | ROS parameters on the command line. Features are plugins, selected with `plugins`. |
| **UI** | PyBullet window and Dear PyGui panel on the PC running the monitor. | Web page (viser) at http://localhost:8080, reachable from other PCs. PyBullet only as an optional debug window in some plugins. |
| **Mocap** | NatNet client inside the monitor; `CLIENT_IP` and `MOCAP_IP` set in the code; axis conversion and calibration in the monitor. | Separate `mocap_relay` node ([step 1](#1-start-the-mocap-relay)). Poses arrive calibrated, in the Rhino Z-up frame. |
| **Tracked objects** | `TrackedObject(...)` in `husky_world.py`. | `ctx.track_object(name, mocap_id, ...)` in a plugin, as in `plugins/mocap_probe.py`. |
| **Tools** | Fixed per robot in the code. | Defaults per robot, overridden with `tools`. Cindy's onboard launch takes one tool per arm (`gripper_left`, `gripper_right`; see crl-husky's README). |
| **Bar actions, live replan, servoing, accuracy tests** | Built in. | Not ported yet; use the old monitor. |
| **Quit** | Close the windows or Ctrl-C. | Ctrl-C once, then wait for the final hold commands. |

### Running the old monitor

The old monitor requires Cyclone DDS and the robot's own domain. Switch the robot to Cyclone first, as described in crl-husky's README ([Still using Cyclone DDS](../crl-husky/README.md#still-using-cyclone-dds-eg-for-the-old-monitor)). The old monitor connects to Motive directly and does not use the relay.

```shell
git -C src/husky-assembly-teleop checkout master
git -C src/husky-assembly-teleop submodule update --init --recursive
python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=86   # robot's domain: 84 Alice, 85 Belle, 86 Cindy
ros2 run husky_assembly_teleop husky_monitor
```

To return to the new monitor, check out its branch, update the submodules and rebuild.

## Migrating a workspace to the new monitor

1. Update both repositories:
   ```shell
   git -C src/husky-assembly-teleop pull
   git -C src/husky-assembly-teleop submodule update --init --recursive
   git -C src/crl-husky pull
   ```
2. Install the new dependencies: the ROS packages and Python packages in [One-time setup](#one-time-setup). New compared with the old monitor: `viser`, `yourdfpy`, `trimesh`, `async-timeout`, `compas_robots`, and the Zenoh packages.
3. Switch to Zenoh: set `export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=80` in `~/.bashrc`, replacing any per-robot `ROS_DOMAIN_ID`, and set up the router as described in crl-husky's README ([Zenoh](../crl-husky/README.md#zenoh)).
4. Set up time sync on this PC: a fixed IP slot and chrony, as in [Time sync](#one-time-setup) and crl-husky's [TIME_SYNC](../crl-husky/TIME_SYNC.md#control-pc-ubuntu).
5. Build both packages with the venv's Python, as in [Build and run](#build-and-run): `python3 -m colcon build --symlink-install --packages-up-to husky_assembly_teleop` also builds crl-husky.
6. Verify: `ros2 topic list | grep a200` lists the robots' topics, and `chronyc -n sources` marks a time source `^*` (or none, if this PC is the reference).

## Mocap rigid bodies

### Register a new rigid body in Motive
Register a new rigid body by selecting a few markers in Motive, following [the documentation](https://docs.optitrack.com/motive/rigid-body-tracking), and note its `Streaming ID` (click it in the `Assets` panel). Or activate an existing rigid body in the `Assets` panel. The relay publishes it as `/mocap/rigid_body/id_<Streaming ID>/pose` as soon as Motive streams it.

### Calibrate rigid body

Add the 3D model to the object in Motive and move the pivot until the model aligns with the markers. If more precision is needed, use a probe to sample corners on the real object. These additional probe points can be used to improve the alignment of the model.

### Tips 💡
In the browser view: left-drag rotates, right-drag pans, scroll zooms.
