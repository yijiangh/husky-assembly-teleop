# Real lab: the first three bars on Alice, Belle and Cindy

Builds the first seven actions of `data_design_study/261103_SRD` (B20_J, B20_R, B21_J,
B21_H, B21_R, B22_J, B22_R) with the monitor on a lab laptop. Cindy inserts B20, B21 and B22. Alice holds B21. Belle
stays parked but is loaded so the planners know where she is. Practise it first in the fake lab
([fake_lab_short_manual.md](fake_lab_short_manual.md)); the page and buttons are the same there,
but it runs the older `261008_2ModulesDemo_purged_process`: other base targets, no foundation step.

Replace the placeholders in the commands with your own paths:

| placeholder | points at |
|---|---|
| `<workspace>` | your ROS workspace: the folder holding `src/` (this repo and crl-husky), `venv/` and `install/` |
| `<drive>` | your synced copy of the shared Drive folder `2025-03 Husky Assembly` |

## What runs where

| component | where | command (section 2) |
|---|---|---|
| Zenoh router | the laptop | systemd service `crl-zenoh-router` (installed once, 1.3) |
| robot launch: arms, tools, base follower | each robot's PC, over ssh | `ros2 launch crl_husky crl_arms_and_base.launch.py gripper:=<tool>` |
| mocap relay | the laptop | `ros2 launch husky_assembly_teleop mocap.launch.py` |
| monitor | the laptop | `ros2 launch husky_assembly_teleop monitor.launch.py ...` |
| Motive | Motive PC `192.168.0.28` | streams the robots' rigid bodies, B20 (1020) and B21 (1021), with Labeled Markers on |

| robot | PC | namespace | rigid body | gripper |
|---|---|---|---|---|
| Alice | `192.168.0.113` | `/a200_0804` | 1840 | `robotiq_2F_85` |
| Belle | `192.168.0.114` | `/a200_0805` | 1850 | `robotiq_2F_85` |
| Cindy | `192.168.0.115` | `/a200_0806` | 1860 | `scaffolding_v3` (both arms) |

Robot login: `administrator`, password `12345678`. Everything runs on Zenoh, ROS domain 80.

## 1. Once per laptop

**1.1 Shell setup.** Add a helper to `~/.bashrc` that sources the workspace and sets the middleware. Then
`crl_ros_source 80` gives a terminal for the lab, and `crl_ros_source 70` one for the fake lab:

```bash
export HUSKY_DRIVE_ROOT="<drive>"      # read by the relay (calibrations) and the monitor (process, runs)
crl_ros_source () {
    source <workspace>/venv/bin/activate
    source /opt/ros/humble/setup.bash
    source <workspace>/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=$1
}
```

**1.2 Build**, and again after every pull of this repo or crl-husky. Use the venv's Python, never bare `colcon`:

```bash
cd <workspace>
crl_ros_source 80 && python3 -m colcon build --symlink-install
```

**1.3 Zenoh router.** Install it as a service; it then starts at login and connects to every robot's router:

```bash
<workspace>/src/crl-husky/crl_husky/scripts/zenoh_router.sh --install
systemctl --user is-active crl-zenoh-router      # active
```

**1.4 Network and time.** Cable the laptop to the lab switch. It needs a reserved address in `.201`–`.210` (check
with `ip -4 -brief addr`); the slots taken are listed in TIME_SYNC.md. Set up chrony as in crl-husky's
[TIME_SYNC.md](../../../crl-husky/TIME_SYNC.md#control-pc-ubuntu) (Control PC). Then `chronyc -n sources` should mark a
source with `^*`.

**1.5 ssh keys** (optional; saves typing the password): run
`for ip in 192.168.0.113 192.168.0.114 192.168.0.115; do ssh-copy-id administrator@$ip; done`.

## 2. Start

Use one terminal per component. `<workspace>/src/husky-assembly-teleop/scripts/lab_tmux.sh real` opens them as panes
of a tmux session (`sudo apt install tmux` once). Each pane has run `crl_ros_source 80` and has the command from below
typed in but not run: press Enter when its step comes. The robots' panes log in to their robot at once (if she was
still off: Up, Enter there); her launch is yours to start in 2.1. `Ctrl-b d` detaches, everything keeps running, and
the same command re-attaches.

**2.0 Before each session.**
- Tell everyone in the lab that this test drives the robots.
- A fake-lab monitor on this laptop holds ports 8080/8081: stop it.
- Each robot's base powered on.
- Each robot's remote e-stop paired and in reach.
- Each arm's teach pendant: powered on, brakes released, **Remote** mode, speed slider at **100 %** (slower, a motion
  times out). Do not start a program by hand: the launch loads and plays `ros_control.urp` itself.

**2.1 Each robot** (one terminal per robot). Start a launch only once that robot's arms are on and in Remote:

```bash
ssh -o ServerAliveInterval=10 -o ServerAliveCountMax=30 administrator@192.168.0.115      # Cindy; Alice .113, Belle .114
ros2 launch crl_husky crl_arms_and_base.launch.py gripper:=scaffolding_v3     # Alice, Belle: gripper:=robotiq_2F_85
```

- The login banner shows her middleware and domain: `rmw_zenoh_cpp (domain 80)`. If it shows anything else, run
  `crl-husky-mw-zenoh` and log in again. It restarts her base services, so never run it while she holds a bar.
- ! `Ctrl-C` in this terminal, or closing it, stops her arms, tools and follower. Never while she holds a bar.

**2.2 Mocap relay.** It reads the base calibrations from `HUSKY_DRIVE_ROOT`:

```bash
crl_ros_source 80 && ros2 launch husky_assembly_teleop mocap.launch.py          # another Motive PC: server_ip:=192.168.0.117
```

! Stopping it stops mocap for everyone in domain 80, including the robots' followers. Never stop it while a base
drives.

**2.3 Check.** In another terminal:

```bash
crl_ros_source 80 && ros2 topic list | grep -c a200 && ros2 topic echo --once /mocap/rigid_body/id_1860/pose
```

**2.4 Monitor.** This is the README's assembly command, plus the base planner and the lab obstacles:

```bash
crl_ros_source 80 && ros2 launch husky_assembly_teleop monitor.launch.py robots:="alice belle cindy" plugins:="assembly_steps robot_control base_planner_panel obstacles" process_directory:=data_design_study/261103_SRD
```

To keep a log, append `2>&1 | tee -i ~/monitor_$(date +%F_%H%M).log`. Quit with one `Ctrl-C`, then wait: the plugins
send their final holds.

Open http://localhost:8080 in Chrome, one page only. The phone page is `http://<this laptop's 192.168.0.x>:8081`, on
the robot Wi-Fi ([phone_manual.md](phone_manual.md)). Every chip under Health should turn green within a few seconds;
hover a chip to see why it is not. Belle's `mocap` chip may stay amber (1.2 mm marker error).

## 3. Safety

- The remote e-stop of the robot that moves is in someone's hand for every motion, base or arm. The Husky's own e-stop
  does not stop the arms.
- **Stop all (Esc)** soft-stops every robot until its **Resume** under Health. It runs over Wi-Fi, so it is not an
  emergency stop. **Cancel** ends the running step and holds the arms.
- Preview every motion with **Play** before **Exec**.
- On Cindy's tools, press one button at a time: B1 and B2 together put that arm in free drive.
- If an arm is left under the compliance controller (`ARMS MAY BE LEFT UNDER cartesian_compliance_controller`), switch
  it back before anything else: Control › robot › arm › **Run: Joint**.

## 4. Set up

1. Assembly › PROCESS: click **Load** once and wait 5–10 s. You should see `76 actions · 269 movements`. The amber
   `robots differ` chip is expected.
2. Leave **Transfer workaround** ticked: Cindy moves her empty tools to the insert start, where you place the bar by
   hand. Leave **Rhino obstacles collide** unticked.
3. Settings: leave the defaults (**Insert split mm** 5, **Servo by bar** on, **Dry run** off).
4. Each base's `ctrl` chip must be green. If it is not, click Control › robot › Base › **Run: Vel**: the follower only
   drives with it.
5. At the start of a session: Load continues the run used last if it was made on these process files. If the run
   line says `continuing run …`, click **New run** twice, so the tests below get a run of their own. (After a restart
   of the monitor mid-assembly, do not: see section 8.)

## 5. Tests before the assembly

Required on the first session with the new monitor, and after a change to the follower or the handoff; optional
afterwards. Do these without a bar, and do not click Mark done.

- **Base:** Plan › Base, pick the robot, **Load: Robot**, set the target about 0.5 m away, **Plan**, **Commit**. Then
  drive back the same way. Do this for Cindy and Alice.
- **Alice's gripper:** Control › `0804` › **Gripper: Open**, then **Close**, then **Open** again.
- **Alice's handoff in the air:** go to row `4. B21_H`, movement `H_M3`. Exec once with **Dry run** ticked and note the
  `anchor X mm, Y°`. Then untick it, Exec, and Confirm. The arm should stay still while the empty jaws close. Open the
  gripper again afterwards.

## 6. The assembly

Click **New run**, then click it again. This starts a fresh run (`0/76 done`) at B20_J, separate from the tests. Runs
are recorded on the Drive in the process folder, under `runs/<date>_<time>/`.

Run the seven actions in order. Each action is made of the procedures below. After every Exec, read the log lines
that follow it in the monitor terminal or in the Console. Where the table says "Dry run first", Exec once with
**Dry run** ticked, then untick it and Exec for real.

**B: drive a base.** Plan › Base, pick the robot, **Load: Step** → **Plan**. Check by eye that the path is clear,
put a hand on the e-stop, then **Commit**. If you see `timed out: still following`, plan and commit again. To stop a
driving base, use Stop all, not the panel's Cancel.

**J: Cindy inserts a bar.**
1. **Live IK: insert start** → **Plan** (15–40 s) → **Play** → **Exec**: the empty tools go to the insert start.
2. By hand: place the bar in both tools and clamp it with the tool buttons, one at a time, until each screw stalls.
   Check the bar is held, then **Confirm**.
3. Servo: scrub the first pass with **Preview time**, then **Confirm**. Later passes run by themselves, unless one
   travels over 5° (`SAFEGUARD`): scrub and Confirm it too.
4. **Plan** → **Exec**: the insert. Watch the bar enter its joint and the held bars stay in their jaws.
5. Ground bar only (B20): **Exec**, fix the foundation to the ground by hand while Cindy keeps holding the bar, then
   **Confirm**.
6. When the line under the buttons says `every step of … done`, click **Mark done**.

**R: Cindy lets go.**
1. **Plan** → **Exec**: the screws open, the tools break out compliantly and back off. Note any `check FAIL` lines.
2. **mv ▶** → **Plan** → **Play** → **Exec**: home.
3. When the line says `every step of … done`, click **Mark done**.

**H: Alice holds B21.**
1. **Live IK: linear start** → **Plan** → **Play** → **Exec**: the approach, then the servo (Confirm its first pass).
2. Her jaws should now be about 100 mm in front of B21 and centred on it. If they are off by more than 1–2 cm
   sideways, stop: her base calibration is off.
3. **Exec** → **Confirm**: the gripper opens.
4. **Plan** → **Play** → **Exec**: the open jaws slide onto B21.
5. **Exec** → **Confirm**: the jaws close while the arm is compliant. Watch that Cindy's tools are not pulled.
6. **Mark done**.

| # | action | procedures | base target (x y yaw) | notes |
|---|---|---|---|---|
| 1 | B20_J | B (Cindy), J | +1.116 -1.516 +90.0° | ground bar: rigid insert, no tighten, then a second servo; it stops by itself if the force changes by more than 20 N. Then the foundation by hand (J, step 5) |
| 2 | B20_R | R | | first retreat: Dry run first |
| 3 | B21_J | B (Cindy), J | +1.919 -1.578 +135.7° | left joint screw tightens into B20; first compliant insert: Dry run first |
| 4 | B21_H | B (Alice), H | +0.875 -0.236 -8.8° | |
| 5 | B21_R | R | | in the preview, check that the tools back off clear of Alice's gripper |
| 6 | B22_J | B (Cindy), J | +1.242 -1.591 +80.0° | both screws tighten; the transfer plan can take up to 2 min; on `no path in 120 s`, Plan again. Watch that B21 stays in Alice's jaws |
| 7 | B22_R | R | | afterwards the current step is B34_J |

**Optional: Alice lets go of B21.** In the process she lets go at action 17 (`B21_HR`). To end here: **action ▶**
until `B21_HR`, then run `HR_M0` (open), `HR_M1` (retreat) and `HR_leave`. If the retreat's plan refuses because of
unbuilt bars, tick **Ignore built bars** for that plan only.

## 7. Stop

1. Monitor terminal: `Ctrl-C` once, and wait for `Server stopped`.
2. Each robot's terminal: `Ctrl-C`, but only if she holds nothing.
3. Relay terminal: `Ctrl-C`.
4. Pendants: stop `ros_control.urp` before switching back to Local. To power a robot off, run `sudo shutdown now` on
   her first, then press the power button.

## 8. If something goes wrong

| what | do |
|---|---|
| a robot's chips red, or something does not start | `<workspace>/src/husky-assembly-teleop/scripts/real_lab/check_robots.sh` (add `cindy` to check one robot; read-only, about 20 s): this laptop's network, router and clock, and per robot her launch, controllers (compliance loaded), crl-husky messages and rigid body. Each FAIL says what to do |
| restart the monitor mid-assembly | `Ctrl-C` once, wait, start it again, reload the page, **Load**: it says `continuing run …` and the done actions stay done. Do not click New run |
| `HOLD NOT SENT on …` | that arm did not get its hold: stop it with the pendant or the e-stop |
| an arm still under `cartesian_compliance_controller` | Control › robot › arm › **Run: Joint** |
| an arm's `arm` chip is red | hover it. Pendant in Local: switch to Remote. Protective stop: find the cause, then **Unlock**. After Stop all: **Resume**. Launch started before the arm was on and in Remote: `Ctrl-C` her launch (only if she holds nothing) and start it again |
| an arm behaves oddly or two drivers fight for it | a second launch may run on her: `pgrep -fa "launch crl_husky"` on the robot lists every launch; stop all but one |
| a motion times out and is held mid-path | the pendant's speed slider is below 100 % |
| topics stall, or a router logs `Closing transport!` | restart the router: `crl-husky-zenoh-restart` on a robot, `systemctl --user kill -s KILL crl-zenoh-router` here (it restarts itself) |
| `undefined symbol` in `librmw_zenoh_cpp.so` | `sudo apt install ros-humble-zenoh-cpp-vendor ros-humble-rmw-zenoh-cpp` |
| Robotiq dead after an e-stop (`Requested 8 bytes, but got 0`) | crl-husky README, Restart: power off the arm and reboot the robot PC |
| Live IK: `no collision-free IK` | the base is too far from the design pose: drive it again (B) |
| redo an action | select it, click **Reopen**, then run its steps again from where the robots stand |

By hand, in the monitor's **Control** tab (a tab per robot: `0804` Alice, `0806` Cindy):
- **An arm back to the joint controller:** the arm's **Run: Joint**.
- **Drive a base:** its **Base** section, hold a drive button. Keep **Clearance** ticked: it soft-stops the base within
  5 cm of anything; untick it only to drive out of a tight spot. Hand on the e-stop.

For the panels and buttons in detail, see [plugins.md](plugins.md). For what a run records, see
[run_record.md](../design/run_record.md).
