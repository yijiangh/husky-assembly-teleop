# husky_assembly_teleop

This is a python package for controlling huskies in the mocap space.

> ⚠️ **Heavy refactor; ✅ ready for peer testing**

# Installation
## Clone and update submodules
Install this library from source by cloning this repo to local and install from source.
```
git clone --recursive git@github.com:yijiangh/husky-assembly-teleop.git
```
> ⚠️ The monitor depends on [crl-husky](https://gitlab.inf.ethz.ch/crl/robot-control/crl-husky) (robot configs, the onboard launch files with the safety sync, `crl_husky_msgs`), cloned next to this repo as `src/crl-husky`, on branch `jg/ros2` at 78041cf or later (the mocap relay's NatNet client lives here, husky_sim publishes only true poses, and model.yaml names each arm's tool). **Whenever you pull this repo, pull and rebuild crl-husky too**: the two change together.

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

All commands in this README run in the workspace root, the folder that holds `src/`, `venv/` and `install/`. The monitor also needs [crl-husky](https://gitlab.inf.ethz.ch/crl/robot-control/crl-husky) in `src/crl-husky`: it provides the robot configs, the onboard launch files with the safety sync, and `crl_husky_msgs`.

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
# The shared core (github.com/yijiangh/scaffolding_assembly_core)
python3 -m pip install --no-deps -e src/husky-assembly-teleop/external/scaffolding_assembly_core

# The monitor's dependencies (versions as in requirements.txt); matplotlib is for husky_assembly_tamp
python3 -m pip install "viser==1.1.1" "yourdfpy==0.0.60" "trimesh==4.12.2" "async-timeout>=4.0" "compas_robots>=0.6" \
    "scipy>=1.8" "pybullet>=3.2" "compas>=2.0" "matplotlib==3.10.3" "pandas>=2.0"

# Check: all five should list a location under src/husky-assembly-teleop/external
python3 -m pip list | grep -iE "pybullet.planning|compas.fab|rs.data.structure|husky.assembly.tamp|scaffolding.assembly.core"
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

The monitor is a small runtime that ticks and owns the shared state; every feature is a plugin. Each folder's `__init__.py` lists its files.

| Folder | What is in it |
|---|---|
| `monitor.py`, `config.py` | The ROS node and its tick; the run configuration, read from the ROS parameters. |
| `plugin_api/` | What a plugin is (`HuskyPlugin`, `@register`) and what it gets (`PluginContext`). |
| `plugins/` | One module or package per feature. Start from `robot_control/` or `examples/` when writing one. |
| `shared/` | What plugins share: each plugin's contract (`<name>_contract.py`), base classes and helpers. |
| `robot_interface/` | The only code that talks to robots: base, arms, tools, mocap subscriptions. |
| `world/` | Measured state, mocap checks, kinematics, the scene planners read. |
| `ui/` | The viser web UI, the phone page and shared widgets. |
| `mocap_relay/` | The mocap relay, its own process (section 1 below). |
| `experiments/` | Offline analysis of recorded experiments, run from `scripts/`. |
| `drive.py`, `plots.py`, `tool_urdfs.py` | Experiment folders on the Drive; plot helpers; tool URDFs and mesh paths. |
| `old/` | The old monitor, for reference only. |

`external/scaffolding_assembly_core/` is the core shared with the Rhino plugin and the planners, a submodule of [yijiangh/scaffolding_assembly_core](https://github.com/yijiangh/scaffolding_assembly_core): the process file format (`process/`, schema 2.1), robots and scenes, and their mirrors in PyBullet and compas_fab. Its layers are listed in its `scaffolding_assembly_core/__init__.py`; the process format is in its `doc/process_format.md`.

## Documentation

| Doc | What it covers |
|---|---|
| `AGENTS.md` | Conventions, build and test commands; for coding agents and people alike. |
| `doc/glossary.md` | The project's words and what each means where. |
| `doc/refactor_rationale.md` | Why the runtime is built the way it is. |
| `doc/plugin_roadmap.md` | Which plugins exist, and which old features are still to be ported. |
| `doc/scene_refactor_plan.md`, `doc/ur_frames.md` | The scene, the UR arm frames. The process file format is in the core's `doc/process_format.md`. |
| `tasks/` | Specs written while building a feature. |
| `doc/base_planner_manual.md` | Base planner experiments: goals, tasks, how attempts are recorded, collected and analysed. |
| `doc/base_exp_manual.md` | Base controller experiments: goal and hypotheses, the three experiments (robot model, model over conditions, controllers), how to run them and where the results are. |
| `doc/fake_lab_test_manual.md` | The fake lab: the monitor against three simulated robots on one laptop (our simulation with a fake Motive and the relay, on Zenoh), started with `scripts/fake_lab/tmux.sh`; a step-by-step test of the assembly steps, which parts are Jakob's and which ours, and the known issues. |
| `doc/phone_manual.md` | The phone page: connecting a phone, every tab and button, what to do after a stop, troubleshooting. |
| `doc/simulation_manual.md` | Simulated robots and mocap: one robot, several, realistic mocap, base experiments, every argument, what is not simulated. |
| `doc/mocap_relay.md` | The mocap relay: topics, calibrations, simulated inputs. |
| `doc/calibration_manual.md` and the other manuals | **Outdated**, written for the old monitor; each says at the top what is missing or different now. |
| `DOCKER.md` | **Unknown state**, not tested with the new monitor. |

> 🚧 A fuller overview (how the monitor fits with crl-husky, the planners and the Rhino design workflow) is still to be written.

# Usage

## 1. Start the mocap relay

The monitor does not connect to Motive. The mocap relay (`husky_mocap_relay`, in this package, its own process) receives the OptiTrack stream and publishes each rigid body as a ROS topic, in the Z-up world frame; a robot's rigid body as its base pose. The monitor, the robots' onboard follower and all other mocap consumers subscribe to these topics.

**Base calibration.** Each robot's rigid body gets crl-husky's default calibration (`config/robots/<serial>/mocap_config.json`), or the newest calibration session on the Drive (`data_experiment/calibration_data/<YYYYMMDD>/`, found through `HUSKY_DRIVE_ROOT` or `drive_root:=`) if it is within 5 cm / 5° of the default. The relay logs what it applies with its hash, and publishes it on `/mocap_relay/calibrations`. A newer session appearing while it runs is only logged; apply it with `ros2 service call /mocap_relay/reload_calibration std_srvs/srv/Trigger` while no robot drives.

> ⚠️ Exactly one relay must run per ROS domain, and it must stay running while mocap is in use. Two relays in one domain would publish interleaved poses with different delays and possibly different calibrations. A relay therefore refuses to start if another one already runs in its domain. Stopping the relay stops mocap for every consumer in that domain.

```shell
ros2 launch husky_assembly_teleop mocap.launch.py                           # Motive PC at 192.168.0.28 (default)
ros2 launch husky_assembly_teleop mocap.launch.py server_ip:=192.168.0.117  # another Motive PC
ros2 launch husky_assembly_teleop mocap.launch.py use_drive:=false          # crl-husky's defaults only
```

Without a relay, no mocap is available. Details in [doc/mocap_relay.md](doc/mocap_relay.md).

**Simulations** use the same relay, started the same way in the simulation's own domain: see [Simulation](#simulation).

## 2. Start the monitor

```shell
ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['alice']" -p plugins:="['robot_control']"
```

The UI is served at http://localhost:8080; other PCs on the network reach it at `http://<host-ip>:8080`. The ports are fixed, so bookmarks keep working: if 8080 or 8081 is taken (another monitor?), the monitor refuses to start and says so. Run a second monitor with `-p viser_port:=8095` (its phone page is then 8096).

**Quit:** press Ctrl-C once and wait. During shutdown the plugins send their final hold commands; a second Ctrl-C skips them.

**Stop all (Esc):** stops every robot (bases, arms through their safety sync, tools) and cancels the plugin tasks that
may move one; loading, drawing and reports go on. Until **Resume** in the health panel, each robot refuses every
command that could move it (clicks included); Resume starts the arms and the base controller again. The monitor keeps
running. Not an emergency stop.

**Arms need the safety sync.** Arm moves and free drive go out only while the robot's `multi_arm_safety_sync` (crl-husky's
onboard arm launch) reports the arm operational with its program playing; otherwise they are refused and the log says
why. Arms on fake hardware are announced as simulated and need no dashboard.

### Panels

Some plugins show content in panels separate from the main control panel. Panels can be moved, docked, collapsed and resized:

- Drag the bar at the top of a panel to float it, or drop it on a screen edge or next to another docked panel to dock it.
- The control at the right end of that bar collapses and expands the panel.
- Drag the edge of a floating panel to resize it.

The main panel (Stop all, health) floats in the top left corner, the console, minimized, in the bottom right. The plugins' tabs are docked on the right, in the order the plugins start: **Assembly** (below), **Collisions**, **Plan**, with a tab per planner panel (**Base**, **Motion**, **Arm**), **Control** (`robot_control`, a tab per robot), and a tab per other plugin; each plugin's tabs carry its icon. Drag a tab out to give it a window of its own, or into another panel. Tools and plots not needed every session (`obstacles`, `debug_gizmo`, `mocap_probe`, the calibration plots) start in a minimized strip at the right edge, nested under their plugin's name. Plugins add tabs with `ctx.view.ui()` and `ctx.view.tab()`, and plugins with the same tab path share it through bands, `ctx.view.ui(band=...)` (`plugins/__init__.py`). To pick a plugin's icon, `scripts/icon_gallery.py` shows every icon viser ships, searchable (tabler.io shows newer versions that can look different).

The **Assembly** tab holds `process`, `assembly_steps`, `movement_exec` and `assembly_run`; it is where you pick and run the current step, top to bottom: the process section (folder, Load, its two boxes, the action list, the step slider, the movement's details); the step's chips and lines; **Live IK**; **Plan**; Play / Clear; the preview slider **t** and the motion time **s**; **Exec**; the prompt with **Confirm** | **Cancel**; **Mark done** | **Reopen**; the settings (Ignore built bars, Insert split mm, Spiral, Rigid insert, Spiral s, Spiral turns, Servo by bar, Dry run); the run (its chips, Continue newest / Continue match / New run, where each robot is); the plan's plots. The action list shows each action's mark from the run (`· pending`; `· done`, green; `· done (assumed)`, orange; the selected action blue), and the movement you pick there (a row, the step slider, ◀ mv / mv ▶) becomes the run's current step, except while `movement_exec` has a robot's lease for a step. Buttons are coloured by what they do: teal computes without moving (Live IK, Plan), orange starts a motion (Exec), green lets a waiting step go on (Confirm), red stops (Cancel), yellow records (Mark done).

Example: `mocap_probe` shows the probe state big enough to read from a distance, in a **Status** tab inside the **Big status** folder at the bottom of its tab (closed at start).
`base_exp` shows its live error and command plots in an **Errors and commands** tab inside the **Live plots** folder at the bottom of its tab (closed at start).

### Phone

A phone page for walking next to the robots: open `http://<laptop's 192.168.0.x address>:8081` on a phone on the robot WiFi, held upright. It has **Stop all**, the health banner, and a tab per plugin: **Control** (D-pad, gripper Open/Close/Stop, Free drive while held), **Health**, **Mocap** and **Exec** (Confirm and Cancel for `movement_exec`). Setup, every button and troubleshooting: [doc/phone_manual.md](doc/phone_manual.md). Plugins add a tab with `ctx.view.phone()` (`plugins/__init__.py`).

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `robots` | none | Robots to connect to, by serial or name: `0804`, `alice`, `a200-0806`, `Cindy`. `"[]"` runs without robots. |
| `plugins` | none | Plugins to load, see below. `health` and `console` are always added, unless `no_default_plugins:=true`. |
| `tools` | per robot | Only where the mounted tools differ from the defaults: `'<robot>:<tool>[,<tool>...]'`, one tool per arm in arm order. Tools: `robotiq`, `scaffolding_v1`, `scaffolding_v3`, `none`. Defaults: Alice and Belle `robotiq`, Cindy `scaffolding_v3,scaffolding_v3`. |
| `process_directory` | none | Process folder (with `process.json`) or old compas_fab export the `process` plugin loads at startup, relative to `drive_root` (e.g. `data_design_study/260814_RobArch_support_ik`) or absolute; without it, pick one in its panel. An export is converted into `<export>_process` next to it. |
| `data_directory` | `data/` of this repo | Root for meshes, URDFs and processes. |
| `drive_root` | `$HUSKY_DRIVE_ROOT` | Local copy of the project's Google Drive folder (`2025-03 Husky Assembly`), for experiment data; see [Experiment data](#experiment-data-google-drive). |
| `recording_format` | `npz` | How runs are saved: `npz` (compact) or `json` (readable text, about twice the size: one row per sample, unknown values as `null`). Both load the same way (`load_recording`). |
| `no_default_plugins` | `false` | `true` skips the default plugins (`health`, `console`). |
| `ghost_timeout` | `20.0` | Seconds a plugin's ghost robots stay after the last input in it; 0 keeps them. |
| `lab` | `main` | The lab file `obstacles` loads: a name in `data/lab/`, or a path to a .json file. |
| `viser_port` | `8080` | Port of the desktop page. Fixed: the monitor refuses to start if it is taken. |
| `phone_port` | `viser_port + 1` | Port of the [phone page](#phone). Fixed too. |

### Experiment data (Google Drive)

Experiment data lives in the project's Google Drive folder, synced to your PC (e.g. with Insync), not in this repo. Point the monitor and the analysis scripts at your local copy of the `2025-03 Husky Assembly` folder itself; the path above it differs per user.

```shell
# In ~/.bashrc, or each terminal: read by the monitor and the scripts
export HUSKY_DRIVE_ROOT="$HOME/Insync/<account>/Google Drive - Shared with me/2025-03 Husky Assembly"

# Or for one monitor run; the parameter wins over the variable
ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['alice']" -p plugins:="['base_exp']" \
    -p drive_root:="$HOME/Insync/<account>/Google Drive - Shared with me/2025-03 Husky Assembly"
```

Paths are given relative to this folder, e.g. `data_design_study/260814_RobArch_support_ik`; absolute paths still work, for data elsewhere. `base_exp` does not start without the root and its `data_experiment/base_exp/` folder; `process` needs the root to load a relative process folder.

**Every plugin saves its runs the same way** (`shared/recordings/`): under its own folder `data_experiment/<plugin>/`
(e.g. `data_experiment/base_exp/`), a folder per experiment name, and in it a folder per run,
`<date>_<time>_<robot>_<what>_<sim|real>/`, with `experiment.json` (what the plugin knew: setup, conditions,
outcome), `recording.npz` (the signals, per tick and per message; `recording.json` with `recording_format:=json`)
and, once analysed, `report.json` and pictures.
Load a run with `load_recording(folder)`. A new plugin needs only its `data_experiment/<plugin>/` folder, made in
Drive.

**Folders under the root are never created by the code.** Create them in Google Drive and sync them instead. The reason: with selective sync, a folder can exist in Drive but be missing on your PC, because it is not synced there. If the code then created it locally, Insync would upload it as a new folder, and Google Drive, unlike a normal file system, allows several folders with the same name side by side. You would end up with two `base_exp` folders, the runs split between them. The code only creates new folders inside an existing one (an experiment, a run), which cannot clash.

### Plugins

| Plugin | What it does |
|---|---|
| `health` | Loaded by default. Shows the status of each robot and tracked object, with Unlock, Resume and Reconnect. A chip's colour change is logged once it has held for 1.5 s, one line per row, so a flickering chip makes one line. |
| `console` | Loaded by default. A window at the bottom right with the newest log lines, newest first; it starts minimized to its bar, the control at the bar's right end opens it. **Show**: **Plugins** (what the plugins say, without health's colour changes and the late-tick reports), **Warnings** (warnings from every source but the ticks), **All**; errors always show. **Source** narrows to one plugin, `health/chips` (health's colour changes), `runtime`, `robots` or `ticks`. Its chip line shows the share of the last minute's tick periods that were late, a freeze counting for every period it lasted. Pause the page (**Live**) to read; stdout and the ROS log still get every line. |
| `robot_control` | The **Control** tab, with a tab per robot: drives the base, jogs each arm joint by joint while a button is held, or its tool0 along or about one of tool0's own axes (the **Tool0** buttons, at the **Tool speed** in mm/s; one arm at a time, a point streamed every tick 0.2 s ahead on a line from where the button was pressed), moves it to a Cartesian target (sliders or gizmo, in the husky's base_link) with a compliance force, holds it, operates the tools. To move an arm to a pose, plan it in `arm_planner_panel`. Free drive is the arm's **Free** controller: the arm can be pushed by hand only while its **Free drive (hold)** button is held or **Keep on** is ticked. Each arm's **Live plots** folder (closed at start) shows its raw force and torque over the last 30 s. The base's **Mocap offset** folder shifts that base's mocap pose by x, y, z in the world for this session (Apply / Reset; a chip shows it while on): everything, planners and records included, sees the shifted pose, and paths to the onboard follower are converted back; the open run notes each change in its events. |
| `obstacles` | The lab's obstacles as collision bodies for the planners: tables, cabinets, tripods and trusses, and walls around the usable floor, from the lab file `data/lab/<lab>.json` (in git; `-p lab:=<name>`); and the floor, a slab under z = 0 out to the walls, not drawn, which arms, tools and what they hold collide with while the robots' platforms (wheels, chassis) may touch it. Click an obstacle in the 3D view or pick it, then drag it with the gizmo (only yaw is kept), resize it with the corner gizmo, or type position, yaw and size; rename and remove it, and set the border's x and y range. Edits stay unsaved (amber chip) until **Save** writes the lab file; **Revert** goes back to it. **Add** puts temporary boxes and cylinders (purple) into the scene, kept in `~/husky_obstacles_temporary.json` across restarts and never in the lab file; **Promote** moves one into the lab. |
| `debug_gizmo` | Debug only. Click any scene body with a world pose in the 3D view, or pick it from the dropdown, then drag it with a gizmo or type its position and roll/pitch/yaw; **Log** prints its pose, **Reset** puts it back. Nothing is saved; to change the lab's obstacles for good, use `obstacles`. |
| `mocap_probe` | Tracks the probe and records points with it, e.g. obstacle corners; exports JSON and loads it back. Shows its state in a separate panel, see [Panels](#panels). **Map coverage** walks a 15 × 15 m grid of 1 m cells, centred on the mocap origin, and colours each on the floor by how often the probe was tracked there (grey: not visited, red → green), with the mean ± std marker error per cell; export and load as JSON. |
| `mocap_bars` | Fits a process bar's CAD markers (from the markers on its connector halves) to the Motive asset made of them (bar Bxx streams as asset 10xx) with the bar fit of `world/bar_fit.py`: the markers matched by their pairwise distances without Motive's labels, around the last fit or the planned pose, ten times a second at most, markers with a residual (or, once the relay sends sizes, a size) out of range left out; draws the real bar at the fitted pose next to the planned one, and shows the fit's residuals (or why it failed) and how far the bar is from where the selected step plans it (across and along its axis, tilt); **Record take** saves the fit to `data_experiment/mocap_bars/<experiment>/` on the Drive. The same fit steers `movement_exec`'s servo with **Servo by bar**. Loads `process`. Experimental. |
| `process` | At the top of the **Assembly** tab. Loads a process and shows the state at one movement at a time, its planned robots see-through at start (View **Ghost** toggles it); the planners load their targets from its selected step, and the run (`assembly_run`) follows it: the movement you pick with the step slider, ◀ mv / mv ▶ or an action row becomes the run's current step, except while `movement_exec` has a robot's lease for a step. Each action row shows its mark from the run (pending, done in green, done (assumed) in orange). After **Live IK** or a plan, the action's later movements are drawn with the joints found (chip `joints from live IK` or `joints from plan`); plans, runs and records never read them. Each movement shows its name in the old monitor (e.g. `J_M3_CDFM_transfer_to_approach`), a sentence on what it does, and a row of its action's movements, coloured by kind as in the Rhino repo's `docs/action_flowchart_2026.drawio` (FM, LM, CDFM, CDLM, Manual, TM). **Transfer workaround** is on at start, for this session only (runs don't keep it): Cindy's transfer runs as a free move with empty tools to the insert start, where you place the bar; while it is ticked, each jointing action is listed and drawn as it runs (`J_transit`, place, grasp, insert; the travel to load left out), and assembly_run, assembly_steps and movement_exec follow it; replan after a change. **Rhino obstacles collide** is off at start: the process's own obstacles from Rhino (e.g. the cabinet) are drawn see-through and left out of every collision check (base and arm planners, clearances, assembly planning); tick it to count them, load `obstacles` for the measured room, and replan after a change. |
| `base_planner_panel` | In **Plan › Base**: plans a collision-free path for a base around the other robots and scene bodies, with the algorithm chosen in its panel (each with its settings, and a Debug view that draws how the search worked); Commit sends it to the onboard pure pursuit follower (`crl_husky pure_pursuit`, or a simulation in your own ROS domain, never 80: [doc/simulation_manual.md](doc/simulation_manual.md)), which drives it at once if the velocity controller is active. The floor shows the path as the follower cuts it and the robot's trail; with Record ticked each run is saved to `data_experiment/base_planner/` on the Drive (`drive_root`, create the folder in Drive first). Loads `base_planner`, the headless planner it drives (other plugins can request and commit plans through it too), and `process`. Experimental. |
| `arm_planner_panel` | In **Plan › Arm**: plans a collision-free arm motion to a joint target (the arm now, its stow pose, the process step, or sliders) and sends it as one joint trajectory that runs through rounded corners and rests only where a corner cannot be rounded clear of collisions; the preview shows the same timing. The panel for the headless `arm_planner` plugin (loaded with it), which others can use through `ArmPlannerContract`. Experimental. |
| `collisions` | In its own **Collisions** tab: lists what collision checks hit, per source (calibration takes, a failed arm or base plan, the motion planner's last answer, base_exp's blocked way back), and draws the ticked hits as they were checked: what moved solid red, what it moved into see-through cyan, the deepest contact with its depth; Focus adds every robot as checked, faint. "As checked" shows meshes as the convex hulls compas_fab tests. Clearance trips of any plugin (a driven robot stopped for coming within its margin of something) come under **clearance**, the newest ticked, with the gap as "x mm apart". Loaded by calibration, the arm and base planner panels, motion_planner and base_exp; load it by name to see clearance trips with other plugins. It only draws: it sends nothing to a robot. |
| `assembly_run` | In the **Assembly** tab: remembers how far the assembly of the loaded process got, in an assembly run, kept in `runs/<YYYYMMDD_HHMMSS>/` inside the process folder (not an experiment's run folder on the Drive): which actions are done, where each robot was left, the run's settings (insert split, spiral, rigid insert, the spiral's seconds and turns, servo by bar; the transfer workaround is `process`'s) and an event log. On Load it continues the newest run when it was made on these very process files; otherwise the operator chooses: **Continue newest** (the newest run, made on another export, goes on in a new run), **Continue match** (an older run made on these files) or **New run**. Nothing is written until the runs were read (a failed or stopped read is tried again). Then it selects the first pending action's first step in `process` once its robot is live; after that it follows what you pick in `process`. **Mark done** records every live robot where it is; for a robot not live, a second click within 10 s records the export's end as assumed (meanwhile the button reads **Click again**). **Reopen** puts an action back to pending. **New run** too needs a second click within 10 s. Each action's mark shows in `process`'s action rows. Loads `process`. Experimental. |
| `motion_planner` | In **Plan › Motion**: plans process movements, solves their goals by IK and checks collisions for other plugins, one request at a time on one worker thread with a compas_fab world per robot (the first request for a robot loads its models, up to a minute); **Cancel** stops them; optional PyBullet window. What the last answer collided with (a start, a goal, a swept segment, a linear step) shows in the collisions panel. Experimental. |
| `assembly_steps` | The operator's steps, in the **Assembly** tab: the run's current step (its action's movements grouped as the operator walks them, with `process`'s transfer workaround; move between steps in `process`), **Live IK** (where the next movement starts, by IK at the live base: the insert start after a transfer, the linear move's start after an approach; logged, shown as the target ghost, recorded, carried into `process`'s drawing, and reused by the next plan while the base stays within 2 mm / 0.5°), **Plan** from the live robot through `motion_planner` (only when it is live), **Play** the preview timed as it will run (slider **t**), **Exec** through `movement_exec`, the motion time **s** (1–90 s, at least what the arms' limits allow), Ignore built bars (for tests in the air), and two plots of the plan: every joint along the preview (deg) and the largest joint step between the plan's rows (deg); the old monitor's bar-hold drift plot comes later, with the held moves. A transfer, an approach or a grounded insert ends with its servo (the step line says "then servos … onto the insert start"), which Exec runs alone once the step's other parts are done. An insert after its transfer's MoCap-bar servo is planned from where the servo left the flanges (logged as `[insert plan] MoCap-bar correction carried`, recorded as `goal_carried`). After the action's last step it says to Mark done (assembly_run's button). A plan goes stale when the process, the step, the transfer workaround or the run's statuses, the acting arm (2°) or base (2 mm / 0.5°), or another robot (1 cm / 1°) changes, and is used by one Exec only. Loads `process`, `assembly_run`, `motion_planner` and `movement_exec`. Experimental. |
| `movement_exec` | In the **Assembly** tab: runs one step on the real robot, every part after **Confirm** (there or on the phone's **Exec** tab), the prompt naming the robot: a motion along the plan (timed for the arms, rebuilt from the live joints and sent right after the last check), a Robotiq open (0.426) or stiff close (0.8), a scaffolding grasp (STOP, tighten until stalled, STOP), a step by hand, and the parts that run partly compliant (`shared/execution/`): the INSERT (rigid under joint control up to the split, then the last millimetres under the compliance controller along a straight line or a 15 mm spiral while the mating joint screw tightens, until it stalls or 30 s pass; a bar with one mating joint runs the straight line whatever Spiral says (the box is greyed then), and after its stall the other arm finishes its line under compliance before switching back; a ground bar is rigid with no tighten; with **Rigid insert** the whole path runs under joint control and the end is held for the stall), the RETREAT (the M1 screws open, the force sensors are zeroed, a compliant break-out of the first millimetres, back to joint control, then the rest replanned from where the tools are and run at once, its checks logged PASS/FAIL), and the Robotiq HANDOFF (the arm goes compliant before the close, giving way up to 10 mm; a stiff close when its frames disagree). The run's settings (`assembly_run`'s) shape them: Insert split mm (0–30; the retreat's compliant part too, from its start), Spiral, Rigid insert, Spiral s (1–60) and Spiral turns (1–10); a spiral over 50 mm/s is refused at Plan. After a transfer, a support approach or a grounded insert the step ends with the servo (`servo_part.py`): pass by pass, each planned on `motion_planner` from the live arms (IK at the live base; a straight bar-held line for two arms, two rows for one), the first pass and any later one travelling over 5° after Confirm, up to 8 passes, until both flanges are within 0.2 mm of the authored targets (tool0) or, with the run's **Servo by bar** and the bar's Motive asset streaming, until the bar's MoCap fit is within 0.5 mm / 0.1° of its design pose; a transfer's bar correction is carried into the insert's plan; after a grounded insert straight lines only, stopped when a flange's force changes by over 20 N. One panel line shows each pass's errors, and each servo is recorded as `runs/<run>/servo/<action>/<movement>.<n>.json` with a .png of its readings. Refuses another robot's or another movement's plan, a stale plan, a plan made before a servo moved the arms, a robot not live or soft stopped, an arm the safety sync (`multi_arm_safety_sync`) does not vouch for (no recent status, not operational, or its program stopped: press Resume), a compliant part whose arm is not under the joint controller or whose measured TCP is over 10 mm / 2° from the URDF's tool0; refused too while another plugin leases any robot (a teleop drive, a committed plan, a base experiment). Leases every robot while it works: no other plugin gets a lease, so nothing else here moves any robot (one motion at a time), and before sending and while its arms move it checks that nothing else moves (another robot's arms or base, or one whose states went stale); Hold in `robot_control` stops its motion short of the end. **Cancel** (or Stop all) holds every arm; in a compliant part it also STOPs the tools, holds each arm at its TCP and switches it back to the joint controller. A hold an arm refuses (stale joint states) is said loudly: HOLD NOT SENT, stop it with the pendant or the e-stop; a switch back not confirmed within 10 s is said loudly too: ARMS MAY BE LEFT under the compliance controller, switch them in `robot_control` before anything else. The teach pendant's speed slider must be at 100 % (slower, a motion times out and is held mid-path). **Dry run**: every check, nothing sent. Every part is recorded in the assembly run (`runs/<run>/movements/`; a compliant part's record keeps its plan and settings, phases, switches, stall times, peak force, end errors, the retreat's checks and tail, and one sample per tick). Loads `motion_planner` and `assembly_run`. Experimental: the first plugin that runs process movements on the robots, e-stops in hand. |
| `base_exp` | Drives test paths (straight, turn, arc, sine, drive-turn-drive; geometric or timed) from where the base stands with the onboard pure pursuit follower, with live error plots; the monitor measures every run's errors itself, the same for every controller, and keeps recording until the robot settles; each run gets its own folder under `data_experiment/base_exp/<experiment name>/` on the Drive (`drive_root`) with its setup (sim or real, sim model, controller and parameters) and outcome (experiment.json), the recording (.npz), and once reported its numbers (report.json) and an overview plot (.png). **Start auto** runs paths at one fixed speed, the standard paths (the same 10 in every scenario, resumed per experiment and setup) or random paths from a grid of distances and angles, or constant commands open loop over a grid of speeds and turn rates; each run is checked to keep 0.3 m from every robot and scene body (`obstacles` loads with it, for the lab border); during any run the robot is soft-stopped and automation paused if it comes within 0.1 m of anything. Experimental; needs `crl_husky pure_pursuit` on the robot, or a simulation ([doc/simulation_manual.md](doc/simulation_manual.md)). Analyse the runs with `scripts/analyze_base_exp.py`: `summary.md` compares the controllers, `robot_model.md` fits a model of the robot, `constant_commands.md` shows how it changes with speed and turn rate. How to run: [doc/base_exp_manual.md](doc/base_exp_manual.md). |
| `example_plot`, `example_ui`, `example_sequence`, `example_pybullet`, `example_recording`, `example_robot_state`, `example_lease_and_freeze` | Minimal plugins, as templates for new ones. |

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

# Assembly: a process, the lab obstacles and the base planner
ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['alice','cindy']" \
    -p plugins:="['robot_control', 'process', 'obstacles', 'base_planner_panel']" -p process_directory:=data_design_study/260814_RobArch_support_ik

# Run an assembly step by step (assembly_run, motion_planner and movement_exec come with assembly_steps)
ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['alice','belle','cindy']" \
    -p plugins:="['assembly_steps', 'robot_control']" -p process_directory:=data_design_study/261008_2ModulesDemo_purged_process

# Plan arm motions too
ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['cindy']" \
    -p plugins:="['robot_control', 'obstacles', 'arm_planner_panel']"

# Try the plugin examples
ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['alice']" \
    -p plugins:="['example_plot', 'example_pybullet', 'example_sequence', 'example_ui']"
```

## Simulation

Simulated robots run beside the same relay and monitor as the real ones, in a ROS domain of their own (never 80): the
monitor drives them unchanged, and health shows a `SIM` chip for a simulated base and arm. Start the relay and the monitor once;
simulations come and go:

```shell
export ROS_DOMAIN_ID=70
ros2 launch husky_assembly_teleop mocap.launch.py                                  # once
ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['belle']" -p plugins:="['robot_control']"  # once
ros2 launch husky_assembly_teleop sim.launch.py robot:=0805                        # one robot, base and arms
ros2 launch husky_assembly_teleop sim.launch.py robot:=all fake_motive:=true       # every robot, a fake Motive
```

- `sim.launch.py` starts crl-husky's `sim_robot.launch.py` (`sim_all.launch.py` for `robot:=all`) and, with
  `fake_motive:=true`, `sim_mocap.launch.py`; it takes every argument of both (`--show-args`).
- The robot (crl-husky) publishes its true poses: the base, and with `tool:=calibration_marker` an arm's tool0 as the
  marker's body, turned by `joint_offsets` when its encoders should be off.
- `sim_motive`, the fake Motive, adds Motive's imperfections to every true pose (`delay_sec`, `noise`, `offsets`,
  `marker_error`, `drift`, `dropout`); without it the relay uses the true poses as they are.

Workflows (one robot, several, realistic mocap from measured values, base experiments, in the lab),
every argument and what is not simulated: [doc/simulation_manual.md](doc/simulation_manual.md).
The fake lab (`scripts/fake_lab/tmux.sh`) scripts all of this for every robot with the assembly plugins, isolated
on a Zenoh router of its own: [doc/fake_lab_test_manual.md](doc/fake_lab_test_manual.md).

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
| **Bar actions, live replan, servoing, accuracy tests** | Built in. | Step by step with `assembly_steps` and `movement_exec` (free moves, support moves, the free or held transfer, Robotiq and scaffolding grasps, steps by hand, the insert and the retreat with their compliant parts, the compliant Robotiq handoff, and the servo after a transfer, an approach or a grounded insert, on tool0 or on the MoCap bar; every motion after Confirm). Live replan and the accuracy tests are not ported yet; use the old monitor. |
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
