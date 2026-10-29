# Simulation

Simulated robots run on a PC beside the real system's own relay and monitor, unchanged: the monitor drives a simulated
robot as it drives a real one, and health shows a `SIM` chip for a simulated base, each simulated arm and each
simulated tool. A simulation runs in a ROS domain of its own and refuses the robots' domain 80.

- **The robot** (crl-husky): its base is `husky_sim`, a kinematic model of the skid-steer base driven by the real pure
  pursuit follower; its arms run on the UR driver's fake hardware with their safety sync; each arm carries the
  robot's tool (crl-husky's `model.yaml`), simulated without motion: a Robotiq reaches every goal at once, a
  scaffolding screw stalls `tighten_s` (2 s) after a tighten and loosens until stopped. It publishes what is truly
  there: the base's true pose and, with the calibration marker on an arm, that arm's true tool0.
- **Force/torque** (teleop, on by default): `sim_wrench`, a near-zero wrench per simulated arm on its
  `ft_sensor_wrench` (the fake hardware's sensor reads NaN), so the F/T plots, the retreat's contact check and the
  servo's load guard read something; `wrench_bias:=` (six values) and `contact_n:=` (a push along the tool's z while
  a simulated screw is stalled) add to it. ! The compliance controller reads the same topic: keep them small.
- **Motive** (teleop, optional): `sim_motive`, a fake Motive between the robot and the relay, adds Motive's
  imperfections to every true pose. It knows no robots or arms.
- **The relay**: the same relay as in the lab, started with `sim:=true` (refused in the robots' domain): it reads
  only the simulated poses, never Motive, so everything measured through it is simulated.

## How it fits together

| Who | Publishes | The relay does |
|---|---|---|
| `husky_sim` (crl-husky) | `/mocap_sim/truth/id_<n>/pose`: the base's true base_footprint; with `tool:=calibration_marker`, the arm's true tool0 as the marker's body (1866) | publishes it as it is |
| `sim_motive` (teleop, optional) | `/mocap_sim/motive/id_<n>/pose`: each true pose with Motive's imperfections; a robot's base relative to crl-husky's default calibration | takes the default off, then applies the simulated session's calibration (`sim/calibration_data/domain_<id>/`) |

Per rigid body the relay publishes the fake Motive's pose, else the truth, whichever streams; when one stops, the
other comes back. Motive is never read: a relay reads either Motive or a simulation, never both. Every simulated part announces itself on `/<robot>/simulated_parts` once a second
(`base`, each arm, each tool, `mocap`); the monitor treats a part as simulated while it was heard within 3 s.

## Every terminal

```bash
source ~/ra/workspace/venv/bin/activate
source /opt/ros/humble/setup.bash
source ~/ra/workspace/install/setup.bash
export ROS_DOMAIN_ID=70          # a domain of your own, never 80; with zenoh everyone simulating picks another
```

## Start once: relay and monitor

```bash
ros2 launch husky_assembly_teleop mocap.launch.py sim:=true
ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['belle']" -p plugins:="['robot_control']"
```

They keep running while simulations come and go. Only one relay runs per domain; a second monitor needs
`-p viser_port:=8095`. With `sim:=true` the relay applies the simulation's base and floor calibrations
(`sim/calibration_data/domain_<ROS_DOMAIN_ID>/`), the probe writes there, and every file says `"environment": "sim"`.
Without it the relay reads Motive only, and a simulated robot gets no mocap pose (its base's mocap chip stays red).

## One robot

```bash
ros2 launch husky_assembly_teleop sim.launch.py robot:=0805
```

- Base, follower and arms, parked where the monitor's Park and Stow put them (crl-husky `config/robots/<serial>/`).
- `arms:=false` for the base only; `start_x`, `start_y`, `start_yaw` (metres, radians) move its start.
- `sim_model` (how the base moves) and `controller_model` (what the follower assumes): models in crl-husky's
  `config/base_models` (`ideal`, `alice_tiles_2`); both the robot's own by default.
- Ctrl-C it and start it again at any time.

## Several robots

All of them, parked 1.4 m apart, each arm with the robot's tool:

```bash
ros2 launch husky_assembly_teleop sim.launch.py robot:=all fake_motive:=true
ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['alice','belle','cindy']" -p plugins:="['robot_control']"
```

Some of them: one launch per robot, and one fake Motive for all of them:

```bash
ros2 launch husky_assembly_teleop sim.launch.py robot:=0804
ros2 launch husky_assembly_teleop sim.launch.py robot:=0806
ros2 launch husky_assembly_teleop sim_mocap.launch.py
```

- ! Run one fake Motive per domain: each fakes every simulated body, so two would both publish them. Nothing refuses
  a second one.

## The fake lab: every robot with the assembly plugins, scripted

`scripts/fake_lab/` runs this simulation (`robot:=all`, with the fake Motive at the measured values below), the relay
and the monitor on the assembly plugins, each in a clean environment on a Zenoh router of its own on the loopback
(domain 70, port 7470), with a scratch copy of the Drive, in one tmux session: `scripts/fake_lab/tmux.sh`. `link.sh`
freezes one robot as if its WiFi dropped. The step-by-step test of the assembly steps on it is
[fake_lab_test_manual.md](fake_lab_test_manual.md).

- `FAKE_LAB_DOMAIN` (the router's port is 7400 + the domain), `FAKE_LAB_PORT` (the monitor's) and `FAKE_LAB_HOME` (the
  scratch data) run a second fake lab beside yours.
- `FAKE_LAB_WS` and `FAKE_LAB_OVERLAY` run a checkout its workspace does not install, e.g. a worktree (`env.sh`).

## Realistic mocap

Motive as measured: a static-body night with continuous calibration off (13–14 Oct 2026,
`mocap_drift_20261013_201614.csv`) and the Motive latency the relay logs at every start.

```bash
ros2 launch husky_assembly_teleop sim.launch.py robot:=0805 fake_motive:=true \
    delay_sec:=0.006 \
    noise:="0.000015,0.003 1866:0.00001,0.015" \
    marker_error:="0.0008 1840:0.0005 1850:0.0012 1860:0.0009 1866:0.0003"
```

| | Measured | Above |
|---|---|---|
| Latency, mid-exposure to transmit | 5.8–6.5 ms | `delay_sec:=0.006` |
| Base jitter (1840, 1850, 1860) | 0.017, 0.012, 0.024 mm per axis; 0.002–0.005° | 0.015 mm, 0.003° |
| Marker jitter (1866) | 0.009 mm, 0.015° | 0.010 mm, 0.015° |
| Reported marker error | bases 0.5–1.2 mm, marker 0.33 mm | as measured |
| Drift, calibration off | under 0.4 mm over a night | off |

- The bodies were still: a moving body jitters more. Dropouts were not measured (none happened); for a robustness
  test, e.g. `dropout:=0.002 dropout_burst_s:=0.5`.
- Base angle noise turns the base about base_footprint, where a real tracker turns about itself, about 0.48 m higher:
  the same angle moves the footprint a little less than on the robot. No bias.

## Base experiments and the base planner

```bash
ros2 launch husky_assembly_teleop sim.launch.py robot:=0804 arms:=false sim_model:=alice_tiles_2 \
    controller_model:=ideal start_x:=1.0 start_y:=-1.5 fake_motive:=true delay_sec:=0.006
ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['alice']" -p plugins:="['base_exp','obstacles']"
```

- Each base_exp run records its sim model and the fake Motive's delay and base noise (0 without one).
- Different `sim_model` and `controller_model` make a model mismatch. Single values override the sim model, e.g.
  `ros2 launch crl_husky pure_pursuit_sim.launch.py ... xICR:=-0.15` (the base alone).
- More: [base_exp_manual.md](base_exp_manual.md), [base_planner_manual.md](base_planner_manual.md).

## While it runs

- Emergency stop and battery: `ros2 param set /a200_0805/husky_sim emergency_stop true`, `battery_percentage 0.2`.
- The base stops 0.5 s after the last twist, as the real one does. Soft stop and Stop all switch the controllers off as
  on the real robot (the base's controller manager is a stand-in inside husky_sim).
- What each source gives: the relay logs every switch ("rigid body 1850: now from the fake Motive, was the
  simulator's truth").

## Arguments

`ros2 launch husky_assembly_teleop sim.launch.py --show-args` lists them all. It starts crl-husky's
`sim_robot.launch.py` (or `sim_all.launch.py` with `robot:=all`) and, with `fake_motive:=true`, this package's
`sim_mocap.launch.py`, with every argument of both; each can also run on its own.

The robot (`sim_robot`):

| Argument | Default | |
|---|---|---|
| `robot` | `0804` | serial; `all` for every robot (sim.launch.py only) |
| `base`, `arms` | `true` | simulate the base, the arms |
| `sim_model`, `controller_model` | the robot's `base_model` | how the base moves, what the follower assumes |
| `start_x`, `start_y`, `start_yaw` | parked (park.yaml) | metres, radians |
| `tool`, `tool_left`, `tool_right` | the robot's (`model.yaml`) | `robotiq_2F_85`, `scaffolding_v3` simulate that tool; `calibration_marker` puts the marker on that arm; `none` leaves it bare |
| `marker_body` | 1866 | the marker's rigid body |
| `joint_offsets` | 0 | the tool arms' encoder offsets, degrees ×6 shoulder_pan to wrist_3; `<arm namespace>:<values>` per arm |
| `husky_urdf` | teleop's `data/husky_urdf` beside crl-husky | for the arms' mounts |

The fake Motive (`sim_mocap`, with `fake_motive:=true`); per-body values are `<values>` for every body or
`<id>:<values>` for one, space separated, a body's own entry winning:

| Argument | Default | |
|---|---|---|
| `delay_sec` | 0 | Motive's latency; poses keep the stamp of when they were true |
| `noise` (per body) | 0 | position std m, angle std ° per axis |
| `marker_error` (per body) | 0.0003 | reported mean marker error, m |
| `offsets` (per body) | 0 | the true pose in Motive's body frame, m and ° (roll, pitch, yaw); a base's over the default |
| `drift` (per body) | 0 | the body frame creeping, m/min and °/min (rotation vector) |
| `dropout`, `dropout_burst_s` | 0 | chance per sample that a body is lost, and for how long |
| `noise_seed` | 0 | the same noise and dropouts again |
| `refines` (per body) | 0 | how often the body was refined: its markers nudged a few tenths of a mm each time; also `ros2 param set /sim_motive refines "1850:1"` while it runs |
| `recalibration` | 0 | Motive's frame moved, x, y, z m and roll, pitch, yaw °: every body and camera seen from it, as after a recalibration; also `ros2 param set /sim_motive recalibration "0.02,-0.01,0,0,0,0.5"` while it runs |
| `cameras_file` | the lab's | the cameras to announce, JSON as `/mocap_relay/cameras`; `none` for none (default `data/sim/motive_cameras.json`) |

## Not simulated

- The tools' motion and grip: they answer at once or after a fixed time, whatever is between the fingers; no Robotiq
  reactivation, no tool buttons (so no free drive from them). Physics, contacts and forces (the F/T sensor reads
  nothing), the UR dashboard, programs and protective stops.
- The base's slip, accelerations and wheel dynamics; twist_mux and the joystick (husky_sim obeys `cmd_vel` itself).
- Time: everything runs on the wall clock, never faster.

## Pitfalls

- A calibration applied in a simulation stays with that domain: later fake-Motive runs there are off by it. The
  relay's start log lists them under "fake Motive's bodies, sessions in …/domain_<id>"; use another domain, or remove
  that domain's folder in `sim/calibration_data/` to start from the defaults.
- `sim_mocap.launch.py` and `ros2 launch` refuse an empty `name:=`: leave an argument out instead.
- A simulation in domain 80 is refused by every launch and simulator; a robot there obeys the same `cmd_vel`.

## Where the code is

| What | Where |
|---|---|
| The base simulator, the tool's true pose | crl-husky `crl_husky/sim/husky_sim.py`, `crl_husky/sim/tool_truth.py` |
| The robot launches | crl-husky `launch/sim/sim_robot.launch.py`, `sim_all.launch.py`, `pure_pursuit_sim.launch.py` |
| Announcing simulated parts, refusing the robots' domain | crl-husky `crl_husky/simulated_parts.py` |
| The relay's simulated inputs | `husky_assembly_teleop/mocap_relay/node.py`, [mocap_relay.md](mocap_relay.md) |
| The fake Motive | `husky_assembly_teleop/mocap_relay/sim_motive.py`, `launch/sim_mocap.launch.py` |
| One launch for a simulation | `launch/sim.launch.py` |
| The fake lab around it | `scripts/fake_lab/`, [fake_lab_test_manual.md](fake_lab_test_manual.md) |
| The monitor's view of simulated parts | `husky_assembly_teleop/robot_interface/simulated_parts.py` |
