# Setting up another PC for the new monitor (for agents)

For a coding agent on a lab PC that runs the old monitor. Set up the new monitor, branch
`yh/process_replan_execution`, in a workspace of its own beside the old one, so that both keep working and the
real-lab test (`doc/real_lab_test_manual.md`) runs from this PC as it does from Yijiang's laptop. Written on 2026-10-29
from that laptop (Ubuntu 22.04, ROS 2 Humble, Python 3.10.12), where all of this works; the versions named are the
laptop's.

! Never pull or check out this branch in the old monitor's checkout: that replaces the old monitor's code. Read this
note from a fresh clone in the new workspace (step 2's first `git clone` is then done), or without changing any
checkout (a fetch only downloads):

```bash
git -C <old checkout> fetch origin yh/process_replan_execution
git -C <old checkout> show FETCH_HEAD:doc/monitor_pc_setup.md
```

## Rules

- ! Leave the old monitor's workspace as it is: the old monitor needs its checkouts, venv and install. Change
  `~/.bashrc` only in step 7, with the user's OK.
- ! Nothing that reaches a robot: no ssh to a robot, no `ros2 topic pub`, `service call` or `action send_goal`, no
  crl-husky `upload.sh`, and never start `scripts/real_lab/tmux.sh`, `robot.sh`, `relay.sh` or `monitor.sh`: they
  start the robots' launches, the relay their followers steer by, and the monitor that commands them.
  `check_robots.sh` only reads, but it logs in to the robots: run it only when the user says so.
- Commands that need `sudo`, a password or the lab router's settings: give them to the user to run (`! <command>` in
  Claude Code). Run the rest yourself.
- Build and test in a clean shell (step 4): your shell may source the old workspace and the old monitor's settings.
- No commits, no pushes. Report what you found and did (step 9).

## 0. Look first, and tell the user what you found

Read-only:

```bash
lsb_release -ds; ls /opt/ros; /usr/bin/python3 --version   # the laptop's: Ubuntu 22.04, humble, Python 3.10.12
grep -nE 'ROS_DOMAIN_ID|RMW_IMPLEMENTATION|setup\.bash|venv|HUSKY_DRIVE_ROOT|REAL_LAB' ~/.bashrc   # the old workspace
ls -d ~/Code/*/src/*/ ~/ros2_ws/src/*/ 2>/dev/null   # old monitor: ~/Code/ros2_ws (laptop), ~/ros2_ws (RobotX PC)
git -C <each checkout> rev-parse --abbrev-ref HEAD; git -C <each checkout> log --oneline -1
dpkg-query -W ros-humble-rmw-zenoh-cpp ros-humble-zenoh-cpp-vendor ros-humble-ur-dashboard-msgs tmux chrony
ip -4 -brief addr | grep 192.168.0                            # this PC's lab address, if it is on the lab network
systemctl --user cat crl-zenoh-router 2>&1 | grep ExecStart   # a router already installed, and from which checkout
ls -d ~/Insync/*/"Google Drive - Shared with me/2025-03 Husky Assembly" ~/Insync/"2025-03 Husky Assembly"  # Drive
ssh -T git@github.com                                         # "Hi <name>!" means access (its exit code is 1)
git ls-remote git@gitlab.inf.ethz.ch:crl/robot-control/crl-husky.git jg/ros2
```

## 1. What is new compared with the old monitor

The old monitor ran Cyclone DDS with one robot per run in that robot's domain (84 Alice, 85 Belle, 86 Cindy), its own
NatNet client and a PyBullet and Dear PyGui window (README, What changed from the old monitor). New on this PC:

| New | Why |
|---|---|
| a workspace of its own, `~/Code/ros2_ws_next` (`src/`, `venv/`, `install/`) | the old one stays as it is; the laptop's path, so the manuals' commands work as written |
| this repo on `yh/process_replan_execution`, with the new submodule `external/scaffolding_assembly_core`, and `external/husky_assembly_tamp` on `yh/core-port` | the new monitor, and the core it shares with the Rhino plugin and the planners |
| crl-husky on `jg/ros2` at 03056b7, in the new workspace | the robots' configs, base models and messages; the laptop's commit, which the robots should run too (manual 1.g) |
| apt: `ros-humble-rmw-zenoh-cpp` and `ros-humble-zenoh-cpp-vendor` (0.1.11), `ros-humble-ur-dashboard-msgs`, `tmux` | Zenoh; the health chips read the arms' robot and safety modes; the real-lab session |
| a Zenoh router, the user service `crl-zenoh-router` | all robots and control PCs run Zenoh on domain 80; this PC's nodes reach the robots through it |
| chrony, with this PC in a control-PC slot (`.201`–`.210`) | the robots take their time from the control PCs; the health chips flag a robot whose clock is off |
| this PC's ssh key on each robot | `robot.sh` starts each robot's launch over ssh; `check_robots.sh` reads her settings |
| the Drive (Insync) with the test's process | the monitor loads the process and records the run there; the relay reads the base calibrations there |
| `~/.bashrc`: Zenoh on domain 80 and the new install by default; the old monitor in `old_monitor_shell <robot>` | the new monitor is the default from now on; the old one stays for occasional tests |
| Python: viser 1.1.1, yourdfpy, trimesh, async-timeout, pandas, matplotlib 3.10.3, ssik 4.1.0 with numpy 1.26.4; the core and compas_fab editable from `external/` | the web page, the robot meshes in it, the planners |

Not needed by the new monitor: dearpygui, kdtree, opencv-python and tracikpy (old monitor only), and the laptop's
`src/ur_dashboard_msgs_src` (a stopgap from before the apt package was installed).

## 2. The workspace

```bash
mkdir -p ~/Code/ros2_ws_next/src && cd ~/Code/ros2_ws_next/src
git clone --recursive -b yh/process_replan_execution git@github.com:yijiangh/husky-assembly-teleop.git
git clone -b jg/ros2 git@gitlab.inf.ethz.ch:crl/robot-control/crl-husky.git
git -C husky-assembly-teleop submodule status   # seven lines, none starting with - or +
git -C crl-husky log --oneline -1               # the laptop's: 03056b7 no second rclpy shutdown on ctrl-c or sigterm
```

- crl-husky: if `jg/ros2` has moved past 03056b7, ask the user whether to `git -C crl-husky checkout 03056b7`. The
  laptop runs 03056b7 (the fake lab tested it), and the real-lab check (manual Part 3) compares each robot's
  crl-husky file by file with this PC's.
- Clone nothing else for the real robots. `src/cartesian_controllers` is for the fake lab only (step 8).
- Later updates: `git pull` and `git submodule update --init --recursive` in this repo, crl-husky to the commit the
  user names, then the build (step 4) again.

## 3. apt packages (the user runs this)

```bash
sudo apt install ros-humble-ur-msgs ros-humble-ur-dashboard-msgs ros-humble-control-msgs \
    ros-humble-controller-manager-msgs ros-humble-rmw-zenoh-cpp ros-humble-zenoh-cpp-vendor tmux
dpkg-query -W ros-humble-rmw-zenoh-cpp ros-humble-zenoh-cpp-vendor   # the laptop's: 0.1.11 for both
```

! Zenoh must have the same version on every machine (crl-husky README, Zenoh). If this PC gets another version than
0.1.11, stop and tell the user.

## 4. The venv and the build, in a clean shell

colcon writes the workspaces sourced at build time into `install/setup.bash`, so a build in a shell that sourced the
old workspace would load it too, and the old monitor's settings in `~/.bashrc` would leak into the tests. Run the
commands of steps 4 and 5 in a clean shell:

```bash
env -i HOME="$HOME" USER="$USER" PATH=/usr/local/bin:/usr/bin:/bin TERM=xterm LANG=C.UTF-8 bash --noprofile --norc -c '
cd ~/Code/ros2_ws_next && source /opt/ros/humble/setup.bash
<the commands>'
```

The venv: README's One-time setup, everything from `# The venv` to the `pip list` check, in order. Then two more:

```bash
source venv/bin/activate
python3 -m pip install "ssik>=4.1,<5" "numpy==1.26.4"                 # the planners' exact IK, as in the fake lab
python3 -m pip install "pytest-xdist==2.5.0" "pytest-timeout==2.2.0"   # the tests' workers and time limit
```

- Without ssik the planners fall back to PyBullet's and compas_fab's IK, and plan differently from the fake lab.
- Don't `pip install -r requirements.txt`: it pins `numpy<1.25.0`, and ssik needs `numpy>=1.26`.
- Check `python3 -m pip list`: viser 1.1.1, yourdfpy 0.0.60, trimesh 4.12.2, ssik 4.1.0, numpy 1.26.4, scipy 1.15.3,
  matplotlib 3.10.3, and the five editable packages located under `src/husky-assembly-teleop/external`.

The build, with the venv's Python (the bare `colcon` would install scripts that run the system Python):

```bash
source venv/bin/activate
python3 -m colcon build --symlink-install --packages-up-to husky_assembly_teleop
```

Expect `crl_husky_msgs`, `crl_husky_cpp`, `crl_husky` and `husky_assembly_teleop` finished, none failed.

## 5. Tests

```bash
source venv/bin/activate && source install/setup.bash && cd src/husky-assembly-teleop
python3 -m pytest -m safety     # command guards, frames, models: ~10 s
python3 -m pytest               # the default set: about a minute
```

They start no ROS nodes and need no robot. Report each summary line; report failures, don't fix them.

## 6. This PC on the lab network (the user, once)

As in the real-lab manual 1.b–1.e and 1.h, with this PC's own address:

1. **Its address.** `ip -4 -brief addr | grep 192.168.0` shows a control-PC slot, `.201`–`.210`, reserved for this
   PC in the lab router. crl-husky `TIME_SYNC.md` lists the slots taken (`.201` RobotX PC, `.202` Jakob's laptop,
   `.203` Yijiang's laptop); a new one goes in that table too. Without a slot the PC works, but the robots take no
   time from it.
2. **The Zenoh router** (no sudo; it starts at every login and links this PC to every robot's router):
   ```bash
   ~/Code/ros2_ws_next/src/crl-husky/crl_husky/scripts/zenoh_router.sh --install
   systemctl --user is-active crl-zenoh-router      # active
   ```
   The router reads the robots' addresses from its own checkout's `config/robots/`. If step 0 found one installed
   from another checkout, `--install` rewrites the service but leaves the old router running: then also
   `systemctl --user restart crl-zenoh-router`.
3. **Time sync:** crl-husky `TIME_SYNC.md`, Control PC, with this PC's slot in the first `sed` (the laptop's
   commands, for `.203`, are in manual 1.d). `chronyc -n sources` then marks one source `^*`.
4. **ssh keys:** `for ip in 192.168.0.115 192.168.0.113 192.168.0.114; do ssh-copy-id administrator@$ip; done`, with
   the robots' password (manual 0.3); first `ssh-keygen -t ed25519` if this PC has no key in `~/.ssh/`.
5. **The Drive:** Insync syncing the shared folder `2025-03 Husky Assembly`, holding
   `data_design_study/261008_2ModulesDemo_purged_process/process.json`. The scripts find it under
   `~/Insync/<account>/Google Drive - Shared with me/` (the laptop) or `~/Insync/` (the RobotX PC); anywhere else,
   `export REAL_LAB_DRIVE=<the folder>` before `scripts/real_lab/tmux.sh`.

## 7. The new monitor as the default (with the user's OK)

Once steps 4–6 work, make every new terminal the new monitor's and keep the old monitor one command away. Plain
terminals, a monitor started by hand and agents' commands then use Zenoh on domain 80 and the new install, as
crl-husky expects of every lab PC; the real-lab and fake-lab scripts set their own settings either way.

- ! If others use this account (the RobotX PC's), the user decides: it changes their terminals too.
- Back up first: `cp ~/.bashrc ~/.bashrc.before_new_monitor`.
- Replace the ROS lines step 0 printed (the old workspace's `setup.bash` or venv, `RMW_IMPLEMENTATION`,
  `ROS_DOMAIN_ID`, `CYCLONEDDS_URI`) with this block, once; keep the rest of the file. Set the two paths under `!` to
  this PC's (step 0).

```bash
# ROS 2 Humble. The new monitor is the default: Zenoh on the robots' domain 80 (crl-husky README, Zenoh) and the
# workspace ~/Code/ros2_ws_next (its programs run in its venv by themselves; activate the venv to build or test).
# The real-lab and fake-lab scripts set all of this themselves.
source /opt/ros/humble/setup.bash
source ~/Code/ros2_ws_next/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=80

# The old monitor (archive, occasional tests), in one terminal: `old_monitor_shell cindy` opens a shell with the old
# workspace on Cyclone DDS in her domain (84 Alice, 85 Belle, 86 Cindy), on the lab network port; `exit` leaves it,
# and `old_monitor_shell cindy -c '<command>'` runs one command there. The robot must be on Cyclone too.
old_monitor_shell() {
    # ! This PC's old workspace and Drive folder.
    local old_ws="$HOME/Code/ros2_ws"
    local drive="$HOME/Insync/<account>/Google Drive - Shared with me/2025-03 Husky Assembly"
    local robot="${1,,}" domain nic address cyclone extra=()
    case "$robot" in
        alice) domain=84 ;; belle) domain=85 ;; cindy) domain=86 ;;
        *) echo "usage: old_monitor_shell alice|belle|cindy [-c '<command>']" >&2; return 1 ;;
    esac
    # The lab network port, found by its address (a dongle's name changes with the dongle); off it, Cyclone's defaults.
    read -r nic address < <(ip -4 -o addr | awk '$4 ~ /^192\.168\.0\./ {print $2, $4; exit}')
    if [ -n "$nic" ]; then
        cyclone="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$nic\"/></Interfaces>"
        cyclone+="<AllowMulticast>default</AllowMulticast></General></Domain></CycloneDDS>"
        extra+=(CYCLONEDDS_URI="$cyclone" MOCAP_CLIENT_IP="${address%/*}")
    fi
    env -i HOME="$HOME" USER="$USER" TERM="$TERM" LANG="$LANG" DISPLAY="$DISPLAY" XAUTHORITY="$XAUTHORITY" \
        XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" PATH=/usr/local/bin:/usr/bin:/bin \
        RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID="$domain" "${extra[@]}" \
        DESIGN_DATA_DIRECTORY="$drive/data_design_study" EXPERIMENT_DATA_DIRECTORY="$drive/data_experiment" \
        bash --rcfile <(printf '%s\n' \
            "source '$old_ws/venv/bin/activate'" \
            'source /opt/ros/humble/setup.bash' \
            "source '$old_ws/install/setup.bash'" \
            "PS1='(old monitor: $robot, Cyclone $domain) \\w\\$ '") -i "${@:2}"
}
```

Check in a new terminal: `echo $RMW_IMPLEMENTATION $ROS_DOMAIN_ID` gives `rmw_zenoh_cpp 80`, `ros2 pkg prefix
husky_assembly_teleop` the new workspace, and `old_monitor_shell cindy -c 'cd ~ && ros2 pkg prefix
husky_assembly_teleop'` the old one. Off the lab network a ROS command prints `Unable to connect to a Zenoh router`
and sees nothing; it does not hang.

! The old monitor sees a robot only while she runs Cyclone: `crl-husky-mw-cyclone` on her before, `crl-husky-mw-zenoh`
after. Both restart her Clearpath services: never while she holds a bar.

## 8. Only if this PC should also run the fake lab

The simulation needs more (`doc/fake_lab_test_manual.md`, section 0): apt `ros-humble-ur-robot-driver
ros-humble-robotiq-description ros-humble-robotiq-controllers ros-humble-gripper-controllers`, and fzi's Cartesian
controllers built into the workspace (the robots carry their own):

```bash
cd ~/Code/ros2_ws_next/src
git clone -b ros2 https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers.git   # the laptop: f8aad77
touch cartesian_controllers/cartesian_controller_{handles,simulation,tests,utilities}/COLCON_IGNORE
```

Then build them as in the fake-lab manual's 0.2.

## 9. Check, and report to the user

Without the robots:

```bash
source ~/Code/ros2_ws_next/src/husky-assembly-teleop/scripts/real_lab/env.sh
ros2 pkg prefix husky_assembly_teleop; ros2 pkg prefix crl_husky    # both under ~/Code/ros2_ws_next/install
```

Only when the user says so: `scripts/real_lab/check_robots.sh` (manual Part 3; it only reads). With the robots off
its first block still checks this PC: the lab address and slot, the router, chrony, ports 8080 and 8081, the Drive,
the Motive PC.

Report:
- the commits: `git log --oneline -1` of this repo and of crl-husky, and `git submodule status`;
- Zenoh's version, and pip's ssik, numpy and viser;
- the build's summary and both test summaries;
- this PC's lab address, `systemctl --user is-active crl-zenoh-router`, `chronyc -n sources`, the ssh keys, the
  Drive folder;
- anything skipped, and why.

## 10. For the operator: the real-lab manual on this PC

The manual is written for Yijiang's laptop. On this PC:
- 0.3, 1.b: this PC's lab address and network port, not the laptop's `.203` on its USB dongle;
- 1.d: this PC's slot in the first `sed`;
- 0.3, 1.h: the Drive folder of this PC's Insync account;
- 0.3, 4.a, 4.b: the phone page at `http://<this PC's address>:8081`.

Everything else works as written. ! Once both PCs' routers run, both reach the robots: run a session
(`scripts/real_lab/tmux.sh`) from one PC at a time. A second relay refuses to start, and so does a second monitor,
but the monitor checks only once, when it starts.
