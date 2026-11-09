# Insertion demo: one bar in and out, Cindy over a cable, no mocap

Cindy inserts B6, a diagonal bar, into the jig mounted on her base and pulls it out again, as often as you click,
with the `insertion_demo` plugin on the process `data_design_study/261104_SRD_insertion`. Nothing else of the lab
comes along: no Motive, no WiFi router, no other robot. The laptop hangs on an Ethernet cable to Cindy's onboard
switch, and a small node tells the monitor that her base stands at the origin, where the process puts it.

Replace the placeholders with your own paths: `<workspace>` is the ROS workspace (the folder holding `src/`, `venv/`,
`install/`), `<drive>` your synced copy of the shared Drive folder `2025-03 Husky Assembly`.

## What runs where

| component | where | command |
|---|---|---|
| robot launch: arms, tools | Cindy's PC, over ssh | `ros2 launch crl_husky crl_arms_and_base.launch.py gripper:=scaffolding_v3` |
| Zenoh router | Cindy's PC | her `crl-zenoh-router` service, port 7447, every interface; the laptop's nodes dial it |
| fixed base | the laptop | `scripts/insertion_demo/base.sh` (`husky_fixed_base`: rigid body 1860 at the origin) |
| monitor | the laptop | `scripts/insertion_demo/monitor.sh` (robots `cindy`, plugins `insertion_demo robot_control`) |

Everything runs on Zenoh, ROS domain 80, as in the lab; `scripts/insertion_demo/env.sh` sets it. The laptop runs
no router of its own: the lab's `crl-zenoh-router` service must be stopped (1.3). Its nodes dial her router as Zenoh
clients, so her router relays everything between them and her nodes.

! In the lab her router is still joined to the lab's over WiFi: the laptop then sees the whole lab's domain 80, Alice's
and Belle's nodes too. So only one monitor may run in the lab at that time, this one (monitor.sh refuses a second),
and nobody may run the mocap relay: base.sh refuses to start while anything else publishes her body's pose. In the
hall there is only Cindy.

## 1. Once per laptop

**1.1 Build**, as for the lab (real_lab_short_manual.md 1.1–1.2): the venv, `python3 -m colcon build
--symlink-install` with the venv's Python, crl-husky built in the same workspace.

**1.2 The process.** `<drive>/data_design_study/261104_SRD_insertion/process/process.json` must be synced; the
scripts find the Drive as the lab scripts do (`DEMO_DRIVE` names another copy).

**1.3 The cable.** `192.168.0.115` is Cindy's WiFi address: over the cable she is reached on her wired bridge `br0`,
the arms' network (`192.168.131.x`, the UR arms at `.40` and `.41`).

- Her address there is `192.168.131.1`, env.sh's default (checked over the cable on 2026-11-09: she answers at `.1`,
  her arms at `.40` and `.41`, and nobody else). Should it ever differ, `ip -4 addr show br0` on Cindy says it: put it
  in `DEMO_CINDY_IP`, e.g. `export DEMO_CINDY_IP=192.168.131.2` in `~/.bashrc`.
- The laptop's Ethernet port (or USB dongle) gets a fixed address on her subnet, with no gateway, e.g.
  `192.168.131.100/24`. Keep the WiFi for the internet; ROS does not use it.
  - ! Her bridge hands out no addresses. A port left on automatic (DHCP) gives up after 45 s and stays without one:
    `nmcli device` shows it `disconnected`, and the ssh of 2.2 ends with `Connection timed out`.
  - Either Settings › Network › Wired › IPv4 › Manual, or once in a terminal, with the port's name from `nmcli
    device` (a USB dongle's starts with `enx`):

    ```shell
    nmcli connection add type ethernet ifname <port> con-name cindy-cable ipv4.method manual \
        ipv4.addresses 192.168.131.100/24 ipv4.never-default yes ipv6.method link-local \
        connection.autoconnect-priority 10
    ```

    From then on the port takes the address whenever the cable is in; `ip -4 addr show <port>` shows it.
- Stop the lab's router service, which would also hold port 7447: `systemctl --user stop crl-zenoh-router`
  (`systemctl --user start crl-zenoh-router` brings it back for the lab).
- Check: with the cable in and Cindy on, `ping 192.168.131.1` answers, and once her launch runs (2.2),
  `source scripts/insertion_demo/env.sh && ros2 node list --no-daemon` lists her nodes (about 60 under
  `/a200_0806/`).

Time is not synced over the cable (the robots take time only from the lab's `.201`–`.210`): the health tab's signal
chip may colour; nothing else reads the clocks.

**1.4 tmux** (optional): `sudo apt install tmux`.

## 2. Start

`<workspace>/src/husky-assembly-teleop/scripts/insertion_demo/tmux.sh` opens the panes below as a tmux session with
each command typed in but not run; press Enter when its step comes. Or use one terminal per component.

**2.1 Cindy on.** Power her on, pair her remote e-stop, put both pendants in Remote (real_lab_short_manual.md 2.1).
The bar rack and the jig are mounted on her base as the process has them: she stands at the origin, the jig in front.

**2.2 Her launch**, over the cable: `ssh administrator@192.168.131.1` (typed into the `cindy` pane; the password as in
the lab, no key is set up), then `ros2 launch crl_husky crl_arms_and_base.launch.py gripper:=scaffolding_v3`. Her base
follower starts too; it is never used here. The first login over the cable asks whether to trust the host at this
address: answer `yes` (it is her key, the same as over WiFi).

**2.3 The fixed base**: `base.sh`. It refuses until her router answers on the cable, and while anything else publishes
her body's pose; it starts about 5 s after Enter.

**2.4 The monitor**: `monitor.sh`, then http://localhost:8080. In the **Assembly** tab the process folder is filled in:
press **Load** (about 15 s). Health shows Cindy green, her mocap chip green (the fixed pose; amber at most).

## 3. The demo

The **Insertion demo** tab. Its status line: Cindy live or not, the action (`B6_J`), her tools reporting, what her
arms run (`joint` / `compliant`), the planned move, the running step. What a button refuses, and how each step ended,
shows below the buttons and in the console.

**Every move takes two clicks.** **Approach**, **Insert** or **Retreat** only plans the move and shows it: a
see-through robot stands on the path at **Preview time** (drag the slider, or **Play**). **Motion time** sets how
long the move takes (10 s by default; the line below says the shortest the arms' limits allow). **Execute** then
runs the move shown, once. **Cancel**, with nothing running, drops the planned move.

**The plots open by themselves**, as in the Assembly tab. Two windows sit minimized at the bottom of the page:
**Plan plots** comes forward when a move is planned (every joint along the preview, and the joint step between the
plan's rows), **Exec plots** when an insert, a retreat or **Compliant** starts (the force and the torque at both
tools, live). A window stays open, its curves too, until the next move replaces them; close it with the `–` in its
bar.

**3.1 Mount the bar, once.** With the arms wherever they stand (jog them in **Control** if needed), put B6 into both
tools as the process holds it (the male halves J4-6 in AT3L, J3-6 in AT3R), then **Grasp**: both gripper screws close
until they stall.

**3.2 Approach.** A free dual-arm move from where the arms are to the insert's start (the transfer workaround: the
bar in the tools is hidden from the planner, as in the lab). Look at the preview, then **Execute**, and watch the
bar clear the rack and the jig.

**3.3 Insert.** Refused unless both tool0s stand within 5 mm / 1° of the insert's start (then: Approach first). Plans
the process's insert from the live joints; **Execute** runs it: rigid up to the last **Compliant mm** (10 of the
15 mm by default), then compliant along the line while both joint screws tighten, until they stall (30 s at most);
then STOP and back under joint control.

**3.4 Retreat.** The preview shows the back-out; **Execute** runs it. The bar stays in the tools: the force sensors
are zeroed, both arms go compliant, the joint screws run in reverse for **Loosen s** (6 by default), the motors stop,
and the tools back out along the insert line to the insert's start; then back under joint control. Insert runs again
right after.

**3.5 Compliant / Rigid.** **Compliant** zeroes the force sensors and puts both arms under the compliance controller:
visitors may push them; they spring back, and the **Exec plots** show each push. **Rigid** holds and brings them back
under joint control. The arms are
then wherever the push left them: **Approach** before the next **Insert**.

**Cancel** stops the running step and holds the arms; **Stop all** (Esc) too. A compliant step cancelled leaves the
arms compliant, said loudly: press **Rigid**; after **Stop all**, first **Resume** under Health, then **Rigid**. An
insert cancelled part way in: **Rigid**, then **Retreat** brings the tools back to the insert's start. **Release**
opens the gripper screws (end of the day).

## 4. If something goes wrong

- `ssh: connect to host 192.168.131.1 port 22: Connection timed out`: the laptop's port has no address on her subnet
  (1.3: `ip -4 addr show <port>` shows none), the cable is not in her switch, or she is off.
- Her launch prints `Unable to connect to any locator of scouted peer …: [tcp/127.0.0.1:…]` for every node, the
  monitor says `/a200_0806: controller manager is not available`, and `ros2 node list` shows none of her nodes: a
  laptop node runs as a Zenoh peer, with settings from before 2026-11-09. Stop base.sh and monitor.sh and start them
  again; in a terminal of your own, `source scripts/insertion_demo/env.sh` again. Her launch can stay.
- `ros2 node list` shows an old or empty list while the monitor is fine: a ROS daemon from another terminal answers.
  `ros2 daemon stop`, or add `--no-daemon`.
- base.sh says `not starting: could not tell that nobody else publishes /mocap/rigid_body/id_1860/pose`: a mocap
  relay or another base.sh publishes her pose (the line ends with the publisher count), or ROS did not answer.
- base.sh or monitor.sh says `insertion demo dry run: no Zenoh router at 127.0.0.1:…`: its terminal carries the dry
  run's settings (`DEMO_SIM`). Use a fresh terminal, or `tmux kill-session -t insertion_demo` and tmux.sh again.
- A plan fails with "its base is not tracked", her `mocap` chip red, or a button refuses with "not live": base.sh is
  not running (press Enter in the `base` pane), or Cindy's joint states do not arrive (the cable, her launch).
- **Execute** refuses with "the arms are … from the plan's start: plan again": the arms moved since the plan; press
  the Move button again.
- "is not under scaled_joint_trajectory_controller": press **Rigid**, or switch the arm in **Control** › her tab ›
  **Run** › Joint.
- "no tool status": her scaffolding tool driver is silent; restart her launch.
- The tighten never stalls: the insert ends NO_STALL after 30 s and comes back under joint control; Retreat still
  works.
- Everything else: the e-stop, then **Resume** in health once it is released.

## 5. Before the demo day, in the lab

First on the laptop alone: [insertion_demo_sim_manual.md](insertion_demo_sim_manual.md) runs these scripts and buttons
against a simulated Cindy (`scripts/insertion_demo/sim_tmux.sh`), the interruptions included.

With the cable, on the real jig: the first jog in **Control** (the clock is not synced: the trajectory must still run),
then Grasp → Approach → Insert → Retreat → Compliant → Rigid → Approach → Insert. Check that 6 s of loosening frees
the bar, that the tighten still stalls, and that the rack on her base is out of the arms' way: the planner does not
know it (it is not in the process).
