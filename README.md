# husky_assembly_teleop

A web UI to watch and drive the Husky robots (Alice, Belle, Cindy) in the mocap lab, and to plan and run assembly
steps on them. Every feature is a plugin you choose at startup. The package also holds the mocap relay, which turns the
OptiTrack stream into ROS topics, and the simulations.

> ⚠️ **Heavy refactor; ✅ ready for peer testing**

- [Install](#install)
- [Try it in simulation](#try-it-in-simulation)
- [Run in the lab](#run-in-the-lab)
- [Parameters](#parameters) and [plugins](#plugins)
- [Documentation](#documentation)
- [Code structure](#code-structure)

# Install

Once per PC, on Ubuntu 22.04 with ROS 2 Humble. All commands run in the workspace root, the folder that holds `src/`,
`venv/` and `install/`.

**1. Clone** this repo with its submodules into `src/`, and next to it
[crl-husky](https://gitlab.inf.ethz.ch/crl/robot-control/crl-husky) on branch `jg/ros2` at fa404f9 or later (ETH
GitLab: ask the lab for access). crl-husky provides the robot configs, the onboard launch files with the safety sync,
and `crl_husky_msgs`. The Cartesian controllers are needed only to simulate the arms; the robots carry their own.

```bash
sudo apt install git-lfs && git lfs install   # once per machine: the lab's obstacle meshes are in Git LFS
mkdir -p src && cd src
git clone --recursive git@github.com:yijiangh/husky-assembly-teleop.git
git clone -b jg/ros2 git@gitlab.inf.ethz.ch:crl/robot-control/crl-husky.git
git clone -b ros2 https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers.git
touch cartesian_controllers/cartesian_controller_{handles,simulation,tests,utilities}/COLCON_IGNORE   # not needed
cd ..
```

A clone made before installing Git LFS draws the obstacles as plain shapes: run `git lfs pull` in it.

**2. ROS packages** (the UR driver, Robotiq and gripper controllers are for the simulated arms)

```bash
sudo apt install ros-humble-ur-msgs ros-humble-ur-dashboard-msgs ros-humble-control-msgs \
    ros-humble-controller-manager-msgs ros-humble-rmw-zenoh-cpp ros-humble-zenoh-cpp-vendor \
    ros-humble-ur-robot-driver ros-humble-robotiq-description ros-humble-robotiq-controllers \
    ros-humble-gripper-controllers
```

**3. The Python venv.** Packages developed alongside this one (`compas_fab`, the core, ...) are installed editable from
`external/`, so use a venv; `--system-site-packages` keeps the system's `colcon` and ROS Python packages
([why](https://github.com/ros2/ros2/issues/1094#issuecomment-2916700723)).

```bash
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
    "scipy>=1.8" "pybullet>=3.2" "compas>=2.0" "matplotlib==3.10.3" "pandas>=2.0" "shapely>=2.1"
# Optional: the planners' and the servo's exact IK; without it they use PyBullet's IK
python3 -m pip install "ssik>=4.1,<5" "numpy>=1.26,<2"

# Check: all five should list a location under src/husky-assembly-teleop/external
python3 -m pip list | grep -iE "pybullet.planning|compas.fab|rs.data.structure|husky.assembly.tamp|scaffolding.assembly.core"
```

**4. Zenoh.** Set up Zenoh as described in crl-husky's README ([Zenoh](../crl-husky/README.md#zenoh)):
`export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=80` in `~/.bashrc`, and the router once.

**5. Build** with the venv's Python; otherwise the installed scripts do not use the venv.

```bash
source /opt/ros/humble/setup.bash
source venv/bin/activate
python3 -m colcon build --symlink-install
```

<details>
<summary><strong>Build fails with <code>setuptools</code> errors</strong></summary>

Inside the venv:

```bash
python3 -m pip install -U pip wheel
python3 -m pip install "setuptools==68.2.2" "packaging>=24.2"   # a safe setuptools for Humble, a new enough packaging
rm -rf build/ install/ log/
python3 -m colcon build --symlink-install
```

To see which versions you use and from where:
`python3 -c "import sys, setuptools, packaging; print(sys.executable, setuptools.__version__, packaging.__version__, packaging.__file__)"`

</details>

**6. Experiment data** (optional; the relay and the monitor start without it). Point `HUSKY_DRIVE_ROOT` at your synced
copy of the project's Google Drive folder, in `~/.bashrc`: processes are loaded from there, and experiments and
calibrations are saved there. Details: [doc/setup/experiment_data.md](doc/setup/experiment_data.md).

```bash
export HUSKY_DRIVE_ROOT="$HOME/Insync/<account>/Google Drive - Shared with me/2025-03 Husky Assembly"
```

**7. Time sync** (lab only). All machines on the lab network keep one time, so TF, logs and recorded data line up; the
control PCs are the reference the robots and the mocap PC follow. Every PC that runs the monitor in the lab must take
part: give it a fixed IP in `.201`–`.210`, reserved in the router, and configure chrony as in crl-husky's
[TIME_SYNC](../crl-husky/TIME_SYNC.md#control-pc-ubuntu). Check with `chronyc -n sources`: one source is marked `^*`,
or none if this PC is the reference itself.

## Every terminal

```bash
source /opt/ros/humble/setup.bash
source venv/bin/activate
source install/setup.bash
```

## Updating

**Whenever you pull this repo, pull and rebuild crl-husky too**: the two change together.

```bash
git -C src/husky-assembly-teleop pull
git -C src/husky-assembly-teleop submodule update --init --recursive
git -C src/crl-husky pull
python3 -m colcon build --symlink-install   # in a terminal set up as above
```

`git config --global submodule.recurse true` makes `git pull` update the submodules too.

# Try it in simulation

No robots or mocap needed. Simulated robots run with the same relay and monitor as the real ones, in a ROS domain of
their own: never 80, where the real robots listen. Use three terminals, each set up as in
[Every terminal](#every-terminal), then `export ROS_DOMAIN_ID=70` (a domain nobody else on the network uses). The
Zenoh router from Install step 4 must be running.

```shell
# Terminal 1: the relay, reading the simulation instead of Motive
ros2 launch husky_assembly_teleop mocap.launch.py sim:=true
# Terminal 2: the monitor
ros2 launch husky_assembly_teleop monitor.launch.py robots:="belle" plugins:="robot_control"
# Terminal 3: Belle (0805), base and arms
ros2 launch husky_assembly_teleop sim.launch.py robot:=0805
```

Open http://localhost:8080. Until the simulation runs, Belle's health chips are red; then she appears parked, her
chips turn green and each simulated part (base, arm, tool) shows a blue `sim` chip. Drive her from the **Control** tab. Simulated arms need no
pendant or dashboard.

The relay and the monitor keep running while simulations come and go. To simulate every robot and a fake Motive
instead, stop Terminal 3 (Ctrl-C) and run `sim.launch.py robot:=all fake_motive:=true` there; restart the monitor with
`robots:="alice belle cindy"` to see them all.

Several robots, realistic mocap, base experiments, every argument and what is not simulated:
[doc/lab/simulation_manual.md](doc/lab/simulation_manual.md). The fake lab (`scripts/fake_lab/tmux.sh`) starts all of it for
every robot with the assembly plugins, on a Zenoh router of its own:
[doc/lab/fake_lab_test_manual.md](doc/lab/fake_lab_test_manual.md).

# Run in the lab

First the robots: power them on, pair their remote e-stops and start their onboard launches (crl-husky's README,
[Startup](../crl-husky/README.md#startup); step by step in [doc/lab/real_lab_test_manual.md](doc/lab/real_lab_test_manual.md)).
Then, on the control PC, in terminals set up as in [Every terminal](#every-terminal):

**1. Start the mocap relay**, unless one already runs in the lab (a second one refuses to start). The relay reads the
OptiTrack stream from Motive and publishes each rigid body as a ROS topic, a robot's as its calibrated base pose. The
monitor reads these topics; without the relay it has no mocap.

```shell
ros2 launch husky_assembly_teleop mocap.launch.py                           # Motive PC at 192.168.0.28 (default)
ros2 launch husky_assembly_teleop mocap.launch.py server_ip:=192.168.0.117  # another Motive PC
ros2 launch husky_assembly_teleop mocap.launch.py use_drive:=false          # crl-husky's default calibrations only
```

> ⚠️ Keep the relay running while mocap is in use: stopping it stops mocap for everyone in that domain.

Calibrations, topics, services and the lab frame: [doc/lab/mocap_relay.md](doc/lab/mocap_relay.md).

**2. Start the monitor** with the robots and plugins you need:

```shell
ros2 launch husky_assembly_teleop monitor.launch.py robots:="alice" plugins:="robot_control"
```

**3. Open the UI** at http://localhost:8080, or `http://<host-ip>:8080` from another PC. A phone on the robot WiFi opens
`http://<host-ip>:8081` ([doc/lab/phone_manual.md](doc/lab/phone_manual.md)). The ports are fixed: if one is taken (another
monitor?), the monitor refuses to start. A second monitor on the same PC: `viser_port:=8095` (its phone page is
then 8096).

Before anything moves:

- **The remote e-stop of the robot that moves is in someone's hand, for every motion.** The Husky's own e-stop stops
  only the base, not the arms.
- **Stop all (Esc)** stops every robot (bases, arms through their safety sync, tools) and cancels the plugin tasks
  that could move one; loading, drawing and reports go on. It is **not an emergency stop**: it goes over WiFi. Each
  robot then refuses every command that could move it, clicks included, until **Resume** in the health panel. Resume
  starts the arms and the base controller again. The monitor keeps running.
- **Arms need the safety sync.** Arm moves and free drive go out only while the robot's `multi_arm_safety_sync`
  (crl-husky's onboard arm launch) reports the arm operational with its program playing. Each teach pendant must be on
  **Remote**. Otherwise the move is refused and the log says why.
- **Quit** with Ctrl-C once and wait: the plugins send their final hold commands. A second Ctrl-C skips them.

What each panel and button does: [doc/lab/plugins.md](doc/lab/plugins.md).

## Examples

```shell
# All three robots at once
ros2 launch husky_assembly_teleop monitor.launch.py robots:="alice belle cindy" plugins:="robot_control"

# Alice with the scaffolding tool instead of the Robotiq, Cindy with only one tool (on the left arm)
ros2 launch husky_assembly_teleop monitor.launch.py robots:="alice cindy" plugins:="robot_control" \
    tools:="alice:scaffolding_v3 cindy:scaffolding_v3:none"

# Measure obstacles with the mocap probe (no robots needed)
ros2 launch husky_assembly_teleop monitor.launch.py plugins:="mocap_probe"

# Run an assembly step by step (assembly_run, motion_planner and movement_exec come with assembly_steps);
# the process folder is on the Drive
ros2 launch husky_assembly_teleop monitor.launch.py robots:="alice belle cindy" \
    plugins:="assembly_steps robot_control" process_directory:=data_design_study/261008_2ModulesDemo_purged_process
```

More, for the planners and the plugin examples: [doc/lab/plugins.md](doc/lab/plugins.md#examples).

# Parameters

Each is an argument of `monitor.launch.py` (`--show-args` lists them); an argument left out keeps the default. Lists
are always quoted, also for one item, and their items separated by spaces (commas work too): `robots:="alice belle"`,
`plugins:="robot_control"`. The monitor's ROS parameters are the same: `ros2 run husky_assembly_teleop husky_monitor
--ros-args -p robots:="['alice']"` works too, with lists in that form.

| Parameter | Default | Meaning |
|---|---|---|
| `robots` | none | Robots to connect to, by serial or name: `0804`, `alice`, `a200-0806`, `Cindy`. Left out, the monitor runs without robots. |
| `plugins` | none | Plugins to load, see below. `health` and `console` are always added, unless `no_default_plugins:=true`. |
| `tools` | per robot | Only where the mounted tools differ from the defaults: `<robot>:<tool>[:<tool>...]` per robot, one tool per arm in arm order: `cindy:scaffolding_v3:none`. Tools: `robotiq`, `scaffolding_v1`, `scaffolding_v3`, `none`. Defaults: Alice and Belle `robotiq`, Cindy `scaffolding_v3:scaffolding_v3`. |
| `process_directory` | none | Process folder (with `process.json`) or old compas_fab export the `process` plugin loads at startup, relative to `drive_root` (e.g. `data_design_study/260814_RobArch_support_ik`) or absolute; without it, pick one in its panel. An export is converted into `<export>_process` next to it. |
| `data_directory` | `data/` of this repo | Root for meshes, URDFs and processes. |
| `drive_root` | `$HUSKY_DRIVE_ROOT` | Local copy of the project's Google Drive folder (`2025-03 Husky Assembly`), for experiment data: [doc/setup/experiment_data.md](doc/setup/experiment_data.md). |
| `recording_format` | `npz` | How runs are saved: `npz` (compact) or `json` (readable text, about twice the size: one row per sample, unknown values as `null`). Both load the same way (`load_recording`). |
| `no_default_plugins` | `false` | `true` skips the default plugins (`health`, `console`). |
| `ghost_timeout` | `20.0` | Seconds a plugin's ghost robots stay after the last input in it; 0 keeps them until another plugin's ghosts show. A layout file's `ghost_timeout` replaces it. |
| `layout` | `""` | A person's layout file (`ui/layout.py`); empty for `~/.config/husky_monitor/layout.json`. Never in git. See [doc/lab/plugins.md](doc/lab/plugins.md#your-own-layout). |
| `lab` | `main` | The lab file `obstacles` loads: a name in `data/lab/`, or a path to a .json file. |
| `viser_port` | `8080` | Port of the desktop page. Fixed: the monitor refuses to start if it is taken. |
| `phone_port` | `viser_port + 1` | Port of the phone page. Fixed too. |

# Plugins

What each one does in detail, and the panels and tabs it shows: [doc/lab/plugins.md](doc/lab/plugins.md).

| Plugin | In short |
|---|---|
| `health` | Default. The status of each robot and tracked object; Unlock, Resume, Reconnect. |
| `console` | Default. The newest log lines, filtered by source and level. |
| `robot_control` | The **Control** tab: drive the base, jog arms and tool0, Cartesian targets, free drive, tools. |
| `obstacles` | The lab's obstacles for the planners, from `data/lab/<lab>.json`; edit and save them. |
| `debug_gizmo` | Debug only: move any scene body by hand; nothing is saved. |
| `mocap_probe` | Record points with the mocap probe; coverage map, cameras, floor calibration. |
| `mocap_bars` | Fit a bar's mocap markers to its design pose. Experimental. |
| `process` | Load a process and step through its movements; the planners' targets come from it. |
| `assembly_run` | Remembers how far the assembly got: done actions, where each robot was left. Experimental. |
| `assembly_steps` | The operator's steps: Live IK, Plan, preview, Exec. Experimental. |
| `movement_exec` | Runs one step on the real robots, with Confirms, compliant inserts and the servo. Experimental. |
| `motion_planner` | Plans process movements and checks collisions for other plugins. Experimental. |
| `base_planner_panel` | Plans a collision-free base path and sends it to the onboard follower. Experimental. |
| `arm_planner_panel` | Plans a collision-free arm motion to a joint target. Experimental. |
| `collisions` | Lists and draws what collision checks and clearance trips hit. |
| `base_exp` | Base controller experiments: test paths, recordings, analysis. Experimental. |
| `insertion_demo` | One bar in and out on demand, Cindy standing still, no mocap: the demo-hall plugin. Experimental. |
| `example_*` | Minimal plugins, as templates for new ones. |

# Documentation

Every doc, by folder: [doc/README.md](doc/README.md). The ones you need first:

| Doc | What it covers |
|---|---|
| [doc/lab/plugins.md](doc/lab/plugins.md) | The UI's panels, the Assembly tab, and every plugin in detail. |
| [doc/lab/real_lab_test_manual.md](doc/lab/real_lab_test_manual.md) | The real robots, step by step: setup, the robots' launches, checks, tests in the air, the first three bars. |
| [doc/lab/fake_lab_test_manual.md](doc/lab/fake_lab_test_manual.md) | The same test against three simulated robots on one laptop. |
| [doc/lab/simulation_manual.md](doc/lab/simulation_manual.md) | Simulated robots and mocap, every argument, what is not simulated. |
| [doc/lab/mocap_relay.md](doc/lab/mocap_relay.md) | The mocap relay: topics, calibrations, the lab frame, rigid bodies in Motive. |
| [doc/glossary.md](doc/glossary.md) | The project's words and what each means where. |
| [AGENTS.md](AGENTS.md) | Conventions, build and test commands; for coding agents and people alike. |

> 🚧 A fuller overview (how the monitor fits with crl-husky, the planners and the Rhino design workflow) is still to be written.

# Code structure

The monitor is a small runtime that ticks and owns the shared state; every feature is a plugin. Each folder's
`__init__.py` lists its files. Conventions, build and test commands for contributors: [AGENTS.md](AGENTS.md).

| Folder | What is in it |
|---|---|
| `monitor.py`, `config.py` | The ROS node and its tick; the run configuration, read from the ROS parameters. |
| `plugin_api/` | What a plugin is (`HuskyPlugin`, `@register`) and what it gets (`PluginContext`). |
| `plugins/` | One module or package per feature. Start from `robot_control/` or `examples/` when writing one. |
| `shared/` | What plugins share: each plugin's contract (`<name>_contract.py`), base classes and helpers. |
| `robot_interface/` | The only code that talks to robots: base, arms, tools, mocap subscriptions. |
| `world/` | Measured state, mocap checks, kinematics, the scene planners read. |
| `ui/` | The viser web UI, the phone page and shared widgets. |
| `mocap_relay/` | The mocap relay, its own process. |
| `experiments/` | Offline analysis of recorded experiments, run from `scripts/`. |
| `drive.py`, `plots.py`, `tool_urdfs.py` | Experiment folders on the Drive; plot helpers; tool URDFs and mesh paths. |

`external/scaffolding_assembly_core/` is the core shared with the Rhino plugin and the planners, a submodule of
[yijiangh/scaffolding_assembly_core](https://github.com/yijiangh/scaffolding_assembly_core): the process file format
(`process/`, schema 2.1), robots and scenes, and their mirrors in PyBullet and compas_fab. Its layers are listed in its
`scaffolding_assembly_core/__init__.py`; the process format is in its `doc/process_format.md`.
