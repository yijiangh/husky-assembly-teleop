# Fake lab: the first three bars on three simulated robots

The real-lab test ([real_lab_short_manual.md](real_lab_short_manual.md)), run on this laptop against simulated Alice,
Belle and Cindy. It uses the same process, the same monitor command and the same buttons. Only the robots, Motive and
the Drive are stand-ins. The simulation itself is described in [simulation_manual.md](simulation_manual.md).

Replace the placeholders in the commands with your own paths:

| placeholder | points at |
|---|---|
| `<workspace>` | your ROS workspace: the folder holding `src/` (this repo and crl-husky), `venv/` and `install/` |
| `<scratch drive>` | an empty local folder outside the synced Drive; the fake lab's runs go there |

## What runs

| component | command (section 2) |
|---|---|
| Zenoh router, the fake lab's own, on `127.0.0.1:7470` only | `ros2 run rmw_zenoh_cpp rmw_zenohd` |
| three simulated robots, their tools, a fake Motive | `ros2 launch husky_assembly_teleop sim.launch.py robot:=all fake_motive:=true` |
| mocap relay | `ros2 launch husky_assembly_teleop mocap.launch.py sim:=true` |
| monitor | `ros2 launch husky_assembly_teleop monitor.launch.py ...` |

- ! Every terminal runs `crl_ros_source 70` first. A new terminal may be in domain 80 (if `~/.bashrc` sets it), the
  robots' domain, and a monitor started there drives the real robots. The simulation and the relay refuse domain 80;
  the monitor does not.
- ! Every terminal also points its nodes at the fake lab's own router, which listens on the loopback only and
  connects to nothing (`ZENOH_CONFIG_OVERRIDE`, in every block below). So no robot can reach this test, and it
  reaches no robot or other simulation.
- ! The relay and the monitor get `drive_root:=` set to a scratch copy, so nothing is written to the real Drive.

## 1. Once

**1.1** Install the UR driver, the Robotiq packages, the gripper controllers and Zenoh:

```bash
sudo apt install ros-humble-ur-robot-driver ros-humble-robotiq-description ros-humble-robotiq-controllers ros-humble-gripper-controllers ros-humble-rmw-zenoh-cpp ros-humble-zenoh-cpp-vendor
```

**1.2** Shell setup: the `crl_ros_source` helper, as in the short real-lab manual, 1.1.

**1.3** The compliant insert, the retreat and the handoff need the Cartesian controllers. Clone them into the
workspace (skip the `git clone` line if `src/cartesian_controllers` exists), then build. Build again after every pull:

```bash
cd <workspace>/src
git clone -b ros2 https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers.git && touch cartesian_controllers/cartesian_controller_{handles,simulation,tests,utilities}/COLCON_IGNORE
cd <workspace>
crl_ros_source 70 && python3 -m colcon build --symlink-install
```

**1.4** Copy the process to a scratch Drive. This reads the real Drive and writes only the copy. To start over
from no runs, delete the copy's `runs/`:

```bash
mkdir -p <scratch drive>/data_design_study
rsync -a --exclude runs --exclude solutions "$HUSKY_DRIVE_ROOT/data_design_study/261008_2ModulesDemo_purged_process" <scratch drive>/data_design_study/
```

## 2. Start

Use one terminal for each component. Or run
`<workspace>/src/husky-assembly-teleop/scripts/lab_tmux.sh fake <scratch drive>`: a tmux session
with a pane per component, `crl_ros_source 70` and the router setting already run in each, and the command typed
in: press Enter in each, the router first.

**2.1 Router.** Start it first: a node without its router waits, silently. If port 7470 is taken, another fake lab
runs on this PC.

```bash
crl_ros_source 70 && ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7470"];connect/endpoints=[];scouting/multicast/enabled=false' ros2 run rmw_zenoh_cpp rmw_zenohd
```

**2.2 Robots.** Wait about 20 s, until the terminal goes quiet:

```bash
crl_ros_source 70 && export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/127.0.0.1:7470"];scouting/multicast/enabled=false' && ros2 launch husky_assembly_teleop sim.launch.py robot:=all fake_motive:=true
```

For Motive's measured latency and marker errors, add the arguments from
[simulation_manual.md, Realistic mocap](simulation_manual.md#realistic-mocap). If an arm's chips stay red
(`Failed loading controller` while three robots start at once), `Ctrl-C` and start it again.

**2.3 Relay.** With `sim:=true` it never reads the real Motive:

```bash
crl_ros_source 70 && export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/127.0.0.1:7470"];scouting/multicast/enabled=false'
ros2 launch husky_assembly_teleop mocap.launch.py sim:=true drive_root:=<scratch drive>
```

**2.4 Monitor.** The same command as in the lab, on the scratch Drive:

```bash
crl_ros_source 70 && export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/127.0.0.1:7470"];scouting/multicast/enabled=false'
ros2 launch husky_assembly_teleop monitor.launch.py robots:="alice belle cindy" plugins:="assembly_steps doc assembly_guide robot_control base_planner_panel arm_planner_panel obstacles stop_recovery" process_directory:=data_design_study/261008_2ModulesDemo_purged_process drive_root:=<scratch drive>
```

Open http://localhost:8080. Every base, arm and tool should show a blue `sim` chip, and the robots should be on their
park row with the arms stowed. ! If there are no `sim` chips, this monitor sees the real robots: `Ctrl-C` it now.

## 3. The run

Follow the short real-lab manual, sections 4 and 6. These things differ in the simulation:

- Bars are placed by hand only in the simulation's mind: just **Confirm**.
- The fake Motive streams no bar markers. B20's servo warns `no marker sample of bars/B20` and uses tool0 instead.
- The simulated screws stall 2 s after a tighten, whatever the arms do. To let an insert reach its end first, run:
  `ros2 param set --no-daemon /a200_0806/left_gripper/scaffolding_tool_sim tighten_s 8`, and the same for
  `right_gripper`.
- A retreat logs `check FAIL grippers: … LOOSENING (want STALLED = fully open)` and `1 check(s) FAILED`: the
  simulated screws never stall fully open.
- The simulated follower sometimes never starts a short path (`timed out: still following`): plan and commit again.

## 4. Stop

`Ctrl-C` the monitor (once, then wait), then the robots, the relay, and the router. Never leave a router running.

Before starting the robots again, `ros2 node list` (in domain 70) should show none of their nodes. A start that died
part way leaves some behind: stop them first, or two controller managers answer for one arm.
