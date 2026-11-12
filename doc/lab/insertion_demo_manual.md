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

**Every move takes two clicks.** **Transit**, **Insert**, **Retreat** or **Return** only plans the move and shows
it: a see-through robot stands on the path at **Preview time** (drag the slider, or **Play**). Each move has its own
motion time, under **motion time**: **Transit s** (it reads **Transfer s** with the Transfer button), **Insert s**,
**Retreat s** (the back-out line, after **Loosen s**), **Return s**, **Load s** and **Stow s**, 10 s each by default;
the line under **Preview time** says the shortest the arms' limits allow for the move shown. **Cycle** takes each
move's own time, and **Preview cycle** their sum. **Execute** then runs the move shown, once. **Cancel**, with
nothing running, drops the planned move.

**The plots open by themselves**, as in the Assembly tab. Two windows sit minimized at the bottom of the page:
**Plan plots** comes forward when a move is planned (every joint along the preview, and the joint step between the
plan's rows), **Exec plots** when an insert, a retreat or **Compliant** starts (the force and the torque at both
tools, live). A window stays open, its curves too, until the next move replaces them; close it with the `–` in its
bar.

**3.1 Mount the bar, once.** With the arms wherever they stand (jog them in **Control** if needed), put B6 into both
tools as the process holds it (the male halves J4-6 in AT3L, J3-6 in AT3R), then **Grasp**: both gripper screws close
until they stall.

**Tighten** / **Loosen** / **Stop** (under **Joint**) run both joint screws by hand, off the cycle: Tighten until
they stall (30 s at most), Loosen in reverse for **Loosen s**, and **Stop** halts both tools' motors at once, whatever
runs them (a Tighten or Loosen under way ends `cancelled`). For a screw left half in, or to show the tools on their
own.

**3.2 Transit.** A free dual-arm move from where the arms are to the insert's start (the transfer workaround: the
bar in the tools is hidden from the planner, as in the lab). Look at the preview, then **Execute**, and watch the
bar clear the rack and the jig.

**3.3 Insert.** Refused unless both tool0s stand within 5 mm / 3° of the insert's start (then: Transit first). Plans
the process's insert from the live joints; **Execute** runs it: rigid up to the last **Compliant mm** (10 of the
15 mm by default), then compliant along the line while both joint screws tighten, until they stall (30 s at most);
then STOP and back under joint control.

**3.4 Retreat.** The preview shows the back-out; **Execute** runs it. The bar stays in the tools: the force sensors
are zeroed, both arms go compliant, the joint screws run in reverse for **Loosen s** (6 by default), the motors stop,
and the tools back out along the insert line to the insert's start; then back under joint control. Insert runs again
right after.

**3.5 Compliant / Rigid.** **Compliant** zeroes the force sensors and puts both arms under the compliance controller:
visitors may push them; they spring back, and the **Exec plots** show each push. **Rigid** holds and brings them back
under joint control. The arms are then wherever the push left them: **Transit** before the next **Insert**.
**Zero F/T** zeroes both force sensors where the arms stand. Do it with no bar in the tools (before 3.1, or after
**Release**): a bar zeroed away reads as a push later, and the compliance sags under its weight.

**3.6 Return.** The straight way back to the insert's start, planned by the insert's own planner (the bar held, a
line in the world) from wherever the arms stand, and checked: the arms' relative pose keeps
within 2 mm and 1° along the path, no joint steps over 5° between two waypoints, and each tool0 ends within 5 mm
and 3° of the insert's start (the planner steers only the left arm; the right ends where the grasp, twisted a little
by the compliant insert and retreat, puts it). A path that fails is still shown in the preview, with the chip
`return refused` and the reason below the buttons, but **Execute** refuses it and the cycle stops there: look at it,
then press **Return** again, or **Transit**. **Execute** runs a good one as a joint motion.

**3.7 Cycle.** One click runs a whole cycle: the insert as 3.3, a pause of **Settle s** (0 by default) once the
screws stalled and the arms are back under joint control, the retreat as 3.4, a moment until the arms stand still
(an arm still moving refuses a trajectory), then Return planned, checked and run.
The line below the buttons ends with `cycle done … Cycle runs the next`: click again for the next visitor. It
refuses until the arms stand at the insert's start (`press Transit or Return first`), and it stops at the first
step that does not end well (`cycle stopped at …` with that step's reason), the arms left as that step leaves
them: an insert that never stalls ends it with the bar in; the operator then presses **Retreat**.
**Preview cycle** plans the same moves without sending anything and shows them as one path (the transit first when
the arms are not at the insert's start, then the insert, the back-out, the return): scrub or **Play** it; the chip
reads `cycle preview` and **Execute** refuses it. Use it after changing a length below, before **Cycle**.

**3.8 Stow.** With the bar unmounted and put away (**Release**, take it out), **Stow** plans a free dual-arm move from
wherever the arms are to their park pose (crl-husky's park.yaml, where the simulator starts them), with the transit's
planner; **Execute** runs it over **Stow s**. The held bar is hidden from the planner, as for Transit: never
with the bar still in the tools.

**Cancel** stops the running step and holds the arms; **Stop all** (Esc) too. A compliant step cancelled leaves the
arms compliant, said loudly: press **Rigid**; after **Stop all**, first **Resume** under Health, then **Rigid**. An
insert cancelled part way in: **Rigid**, then **Retreat** brings the tools back to the insert's start. **Release**
opens the gripper screws (end of the day).

A cancelled cycle does not resume: the next **Cycle** starts over from the insert's start. Before it, by hand:
**Rigid** when the arms were left compliant (cut during the insert or the retreat), **Retreat** when the bar is still
in its joints (cut during the insert, the pause or the retreat), then **Return**, then **Cycle**.

**3.9 A longer insert and retreat.** **Insert mm** sets how far before its end the insert starts, **Retreat mm** how
far the retreat backs the tools out, both on the insert's own line; they start at the process's length (15). They act
at once, with nothing saved: **Transit** goes to the new start, **Insert** and **Cycle** want the arms there,
**Retreat** backs out to Retreat mm, and **Return** goes from there to the insert's start. The new start's joints are
solved on the robot model in 5 mm steps from the process's start, with no collision check: the planners check when
they plan, so look at the preview. A length the arms can't reach is refused with the reason. For a longer compliant
part raise **Compliant mm** too. First time on the robot: **Preview cycle**, then **Transit**, **Execute**, then one
manual Insert and Retreat before **Cycle**.

**Save** writes the insert's start for **Insert mm** into the loaded process: the insert's start joints and line
distance and the transfer's target in `actions/<action>.json`, overwriting that one file on the Drive, for everyone.
It asks first (amber `Save?`): press it again. Then press **Load** in the Assembly tab; the sliders start from the
saved length. Save is never needed to run. Retreat mm is not part of the process and is not saved. Runs and solutions
recorded for that action no longer match the file afterwards.

**3.10 Carrying the bar from a loading pose.** A variant of the demo: the bar is loaded over Cindy's back and both
arms carry it to the insert's start and back, along one path planned beforehand (once per process, see below) and
saved in the process folder as `solutions/B6_J.json`. Untick the Assembly tab's **Transfer workaround** box once and
leave it: the **Transit** button then reads **Transfer**, and the chip next to the action `bar held`.

1. Tools empty: **Release**, then **Load**, **Execute**. A free move from wherever the arms are to the loading pose.
2. Put B6 into both tools there, **Grasp**.
3. **Transfer**, **Execute**: the saved path, the bar held, ending at the insert's start. Then **Insert**,
   **Retreat** and **Return** as in 3.3, 3.4 and 3.6.
4. **Load**, **Execute**: the same path backwards, from the insert's start to the loading pose.

**Cycle** runs steps 3 and 4 on one click, from the loading pose and back to it: the transfer, the insert, the pause,
the retreat, the return, the transfer back. **Preview cycle** shows that loop as one path.

- Checking a transfer before **Execute**: the green preview robot carries the bar and its joint halves, so scrub
  **Preview time** and look for anything they touch. The **Plan plots** window shows, below the joints, how far one
  tool0 seen from the other moves from where it is at the first waypoint, at every waypoint and halfway between: its
  position in mm, and the angle of its x, y and z axis in degrees. Both must stay near zero (the saved path: under
  0.05 mm and 0.01°); the line below the buttons gives the largest. Insert, Retreat and Return show the same.
- No planner runs for a transfer: the preview shows the saved path. **Transfer** wants the arms at the loading pose
  within 2° at every joint; away from it the path is only shown (chip `transfer refused`).
- **Load** picks its move by where the arms stand. At the insert's start (within 2°): the saved path backwards, the
  bar held. Anywhere else: a free move to the loading pose, for empty tools, planned only after a **Release** that
  ran to its end; a **Grasp**, or a gripper screw closed by a button on the tool, takes that back. Otherwise it
  refuses: `the tools may hold the bar: press Return first`.
- The saved path ends at the process's own insert start. With **Insert mm** moved away from it, Transfer and Load say
  so: set it back, or **Save**, **Load** the process again and plan the path again.
- `the saved carry is stale`: the process or the action changed since it was planned (also by **Save**). Plan it again.
- ! The bar rack on the base is not in the process: the path over the back is planned blind to it. Run the first
  transfer slowly (**Transfer s** up) and watch.
- ! After an insert and a retreat the grasp is twisted by about 1°; the transfer back pulls the arms onto the planned
  path in its first segment. Watch the first one.
- With the box ticked (chip `bar free`) the button reads **Transit** and plans the free move of 3.2, and **Load** is
  always the free move.

Planning the carry, once per process (about 10 min), in the workspace's venv, from `external/husky_assembly_tamp`.
The benchmark root and the plans folder are any local folders: only `solutions/B6_J.json` belongs to the process.

```bash
export HUSKY_BENCH_ROOT=<a local folder>
PLANS=<a local folder>
PROC="$HUSKY_DRIVE_ROOT/data_design_study/261104_SRD_insertion/process"
FINE="--goal joints --cdfm-position-res 0.004 --cdfm-rotation-res 0.01 --save-dir $PLANS --no-plot"
python scripts/benchmark_cdfm.py build-cells "$PROC" --bars B6
# 1. The loading pose: tamp's search derives it from its "back" carry pose (the bar fore-aft over the back) and
#    plans a first carry from it, in about 30 s.
python -u scripts/plan_process_carry.py "$PROC" --bar B6 --cdfm-backend birrt --cdfm-home-anchor back $FINE \
    --save-tag direct
#    Its line "world_from_bar_start: ((x, y, z), (qx, qy, qz, qw))" goes into the roadmap's one carry start,
#    $HUSKY_BENCH_ROOT/prm/carry/srd_demo/starts.json:
#      {"starts": [{"world_from_bar": [[x, y, z], [qx, qy, qz, qw]], "label": "back_high"}]}
# 2. The roadmap anchored on that pose (7 min), then the carry on it (45 s), saved into the process.
python scripts/prm_pilot.py build --name 261104_SRD_insertion__B6 --problem-dir "$PROC" --bar B6 --nodes 6000 \
    --starts-file carry/srd_demo/starts.json --workers 10
python -u scripts/plan_process_carry.py "$PROC" --bar B6 --cdfm-backend prm --prm-run 261104_SRD_insertion__B6 \
    --prm-workers 8 --prm-tolerances-mm 10,20,30,45 --prm-tube-rounds 2 $FINE --save-tag prm --write-solution
# 3. Check what was saved: collisions, the grasp between waypoints, the lowest point. Not ok: delete the solution.
python -u scripts/prm_check_solved.py "$PROC" --bar B6 --solved-dir "$PLANS" --tags prm direct
```

- The finer steps (`--cdfm-position-res 0.004 --cdfm-rotation-res 0.01`) keep every joint step under the demo's 5°.
- Step 1's line with `--write-solution` saves the search's own path instead of the roadmap's: the same loading
  pose, a shorter way (0.68 m of bar travel against 1.04 m on 2026-11-11).
- From tamp's own back pose (grasp midpoint 0.10 m ahead of the base centre, 1.04 m up) the roadmap found no route
  past the jig; the search's pose is 0.2 m further back and 0.3 m higher.
- On 2026-11-11 the carry was planned on the dry run's copy of the process only
  (`~/husky_insertion_demo_sim/drive/…/process/solutions/B6_J.json`, both plans beside it in `carry_plans/`); for
  the hall it has to be planned on, or copied to, the Drive's process.

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
- "return: refused: the arms' relative pose drifts …" or "… a joint jumps …": the planner's way back bends the grasp
  or flips an arm; press **Return** again (a new plan from the live joints), else **Transit**.
- "cycle stopped at …": that step's reason follows; see 3.7 and the Cancel paragraph for what to press next.
- The tighten never stalls: the insert ends NO_STALL after 30 s and comes back under joint control; Retreat still
  works.
- Everything else: the e-stop, then **Resume** in health once it is released.

## 5. Before the demo day, in the lab

First on the laptop alone: [insertion_demo_sim_manual.md](insertion_demo_sim_manual.md) runs these scripts and buttons
against a simulated Cindy (`scripts/insertion_demo/sim_tmux.sh`), the interruptions included.

With the cable, on the real jig: the first jog in **Control** (the clock is not synced: the trajectory must still run),
then Grasp → Transit → Insert → Retreat → Return → Compliant → Rigid → Transit → Insert, then one **Cycle**. Check
that 6 s of loosening frees the bar, that the tighten still stalls, that Return's checks pass on the real grasp, and
that the rack on her base is out of the arms' way: the planner does not know it (it is not in the process).
