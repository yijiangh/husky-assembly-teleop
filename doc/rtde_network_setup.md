# Laptop network setup for direct RTDE control of Cindy's UR arms

*(2026-09-07, laptop `su`; companion to the ops notes in
`husky_assembly_teleop/open_loop_engine.py`)*

## What was configured, and why ping suddenly worked

Cindy's internals (robot PC `192.168.131.1`, left UR `192.168.131.40`, right UR
`192.168.131.41`) all hang off **one internal Ethernet switch**, and the
externally accessible port(s) are on that same switch — so a single cable from
the laptop reaches both arms (verified: Clearpath's factory test sheet pings
both arm IPs from the customer port, and we measured ~0.5 ms to all three).

**Why it did not work before:** the husky LAN has **no DHCP server**. A freshly
plugged laptop NIC sits in NetworkManager's auto-generated DHCP profile
("Wired connection 2") forever "getting IP configuration" — it never obtains an
IPv4 address, so there is no route to `192.168.131.0/24` and pings to `.40`
go nowhere. The fix is simply a **static IP profile** on the USB adapter:

```bash
# one-time setup (no sudo needed -- NetworkManager allows this for a local desktop user)
nmcli con add type ethernet ifname enx00133bfc2314 con-name cindy \
    ipv4.method manual ipv4.addresses 192.168.131.19/24
nmcli con up cindy
nmcli con mod cindy connection.autoconnect-priority 10   # beat the DHCP profile on replug
```

- `192.168.131.19` is Clearpath's documented example "customer IP"; nothing on
  the robot uses it (`.1` PC, `.40`/`.41` arms).
- The `/24` installs a directly-connected route
  (`192.168.131.0/24 dev enx00133bfc2314`), which is what makes `.40`/`.41`
  reachable.
- **No gateway is set**, so this link never becomes a default route: internet,
  the mocap/lab network on `eno1` (`192.168.0.x`), and DNS are untouched. Only
  traffic addressed to `192.168.131.x` uses this cable.

Quick health check after plugging in:

```bash
ping -c2 192.168.131.1 && ping -c2 192.168.131.40 && ping -c2 192.168.131.41
```

## Caveats

1. **Bound to one specific USB adapter.** The profile matches interface
   `enx00133bfc2314` (the name encodes that dongle's MAC). A different dongle
   or the built-in port won't pick it up — either edit the profile's
   `connection.interface-name` or add a second profile.
2. **Replug behavior.** With `autoconnect-priority 10` the `cindy` profile
   should win automatically when the adapter appears. If the NIC ever shows
   "getting IP configuration" again (i.e. "Wired connection 2" grabbed it),
   run `nmcli con up cindy`.
3. **IP collision.** If a second laptop uses `.19` on the husky LAN at the same
   time, they conflict — pick another free address for the second machine.
4. **Subnet overlap.** If this laptop ever joins some *other* network that also
   uses `192.168.131.0/24` (Clearpath's convention; rare elsewhere), routing
   becomes ambiguous while both are connected. Deactivate one
   (`nmcli con down cindy`).
5. The husky PC's wifi (`192.168.0.115`) is a separate interface — ROS-over-
   wifi (gripper stack, monitor) coexists with wired RTDE traffic by design.

## ROS 2 / DDS to the husky (for the gripper path)

Verified 2026-09-07. Three things must all be right, or discovery is silently empty:

1. `export ROS_DOMAIN_ID=86 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (Cindy; see crl-husky README).
   ! **This line is NOT in `~/.bashrc`** (only item 2 is, checked 2026-09-10). A terminal that
   forgets it runs in domain 0 on FastDDS: `ros2 action list` is empty, the engine's startup
   banner shows `Gripper Action Server False`, and every gripper event of a live run is
   logged `SKIPPED-no-server` while the arms carry on -- which is exactly what happened on
   2026-09-10 (ten events, none sent, the Husky's stacks were up the whole time). The engine
   now prints its `ROS_DOMAIN_ID`/`RMW` in the banner and **refuses START** when a server is
   missing. Put the export in `~/.bashrc` next to item 2, or in the terminal every time.
2. `export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml` — the file (pins `eno1`, the
   192.168.0.x lab NIC) already existed on this laptop but the export was missing from
   `~/.bashrc` (now added, per crl-husky's DEBUG_CYCLONE.md). Without it Cyclone auto-picks
   the eduroam wifi and nothing is discovered. DDS runs over 192.168.0.x (laptop `eno1` ↔
   husky `br0`, which holds 192.168.131.1 AND a 192.168.0.x DHCP lease), NOT over the
   RTDE cable.
3. **The ros2 CLI daemon caches the env of its first invocation.** After changing any of
   the above, run `ros2 daemon stop` once — otherwise `ros2 node list` etc. keep using the
   stale settings and return nothing.

The open_loop_engine process needs the same three exports for its gripper action clients.

## Gripper-only stack (no arm driver) on the husky

ssh in (`administrator@192.168.131.1` wired or `@192.168.0.115` wifi; key auth from this
laptop is set up), then:

```bash
tmux new-session -d -s gripper_left  "source /etc/clearpath/setup.bash; ros2 launch crl_husky crl_gripper.launch.py namespace:=/a200_0806/left_gripper  gripper:=robotiq_2F_85 com_port:=/tmp/ttyUR_left  start_tool_communication:=true robot_ip:=192.168.131.40 2>&1 | tee /tmp/gripper_left.log"
tmux new-session -d -s gripper_right "source /etc/clearpath/setup.bash; ros2 launch crl_husky crl_gripper.launch.py namespace:=/a200_0806/right_gripper gripper:=robotiq_2F_85 com_port:=/tmp/ttyUR_right start_tool_communication:=true robot_ip:=192.168.131.41 2>&1 | tee /tmp/gripper_right.log"
```

Arms must be powered on (tool 24 V comes through the arm; RUNNING or IDLE). Stop them with
`scripts/husky/stop_gripper_stacks.sh` — run it from the laptop (it ssh's in itself) or on
the husky; `--status` reports without stopping anything. It kills only the two gripper
sessions and lists any leftover gripper process, and never touches the Clearpath **platform**
`ros2_control_node` (namespace `/a200_0806`, params from
`/etc/clearpath/platform/config/control.yaml`), which belongs to the robot's own boot.
By hand that is `tmux kill-session -t gripper_left` (and `_right`).

! Always stop these before launching the full `crl_dual_ur5e.launch.py` stack, which starts
its own tool bridges — two bridges to one arm fight over TCP 54321.

Manual test of one gripper (close 0.8, open 0.426 — the monitor's button values; the
open-loop engine itself opens FULLY, 0.0, since 2026-09-10, to match the planner):

```bash
ros2 action send_goal /a200_0806/left_gripper/robotiq_gripper_controller/gripper_cmd \
    control_msgs/action/GripperCommand "{command: {position: 0.8, max_effort: 0.1}}"
```

A `stalled: true / ABORTED` result just means the fingers met resistance before the exact
target — for a grasp that is the success case. Since 2026-09-09 the engine reads that result
(`send_gripper_cmd(..., on_result=)` → `OpenLoopEngine._on_gripper_result`): a close that
stalls is logged GRASPED with the jaw width, a close that reaches the fully-closed target met
nothing and is logged MISSED — and stops both arms, unless `--no-grasp-abort`. The verdict
lands in `run_info.json` under `events_fired`.

**Known issue (deferred, 2026-09-07): grippers run at ~15% of max speed.** A bug in the
ros2_robotiq_gripper fork's `hardware_interface.cpp` `write()` folds `kGripperMaxSpeed`
into the speed state before scaling to the 0–255 Modbus register, capping it at 38/255
forever — no yaml/xacro setting can raise it. Fix (2 lines, then
`colcon build --packages-select robotiq_driver` on the husky + restart gripper stacks):

```cpp
// replace the two speed lines in write() with:
const double speed_fraction = std::clamp(fabs(gripper_speed_) / kGripperMaxSpeed, 0.0, 1.0);
write_speed_.store(uint8_t(speed_fraction * 0xFF));
```

**The force register lands on 255 = FULL force, 235 N, on every grasp (verified on the
husky 2026-09-09).** Three bugs in the husky's checkout (`2ff8545`, pre-upstream-#83) line up:

1. `export_command_interfaces()` (~line 195-203) seeds the values with the parameter's
   **`count()`**, not its value: `gripper_force_ = params.count("gripper_force_multiplier") ?
   params.count(...) : 1.0` → the xacro passes the parameter, so `gripper_force_ = 1`
   (the 0.5 written in `2f_85.ros2_control.xacro:41` is never read). Same for speed.
2. `write()` (~line 297-300) then folds the constant in: `235 * clamp(1/235) = 1.0` →
   `uint8_t(1.0 * 0xFF)` = **255**. (Speed: `0.150 * clamp(1/0.150) = 0.150` → 38 = 15 %.)
3. The `max_effort` in the `GripperCommand` goal (`GRIPPER_EFFORT = 0.1` in the engine)
   **never reaches the gripper**: the husky runs stock `ros-humble-gripper-controllers
   2.45.0`, whose `GripperActionController` has no `use_effort_interface` parameter (a
   rolling-era option) — `crl_robotiq_controllers.yaml`'s `use_effort_interface: true` is
   silently ignored and `set_gripper_max_effort` stays unclaimed.

So there is NO configuration that changes the grip force today, and it is at the maximum.
The gripper itself IS force-aware — it stops the fingers on contact (gOBJ 0x02,
`OBJECT_DETECTED_CLOSING` in `default_driver.cpp`) and holds the rFR force — so commanding
a full close is the right usage and cannot hurt the gripper; 235 N is simply more than a
wooden leg needs. The 15 % speed is the more urgent problem: from the 0.426 rad pre-open
(~40 mm jaw) the pads take ~0.8 s to reach a 22 mm leg, longer than the 0.5 s `grasp_wait`
the trajectories hold still for, so the arm starts lifting before the grip is secure. (The
engine now opens fully, 0.0 rad / 85 mm, so at 15 % speed that close would take ~1.6 s —
another reason the speed patch matters.)

**The fix is upstream PR #83 (`e5656b1`).** The laptop's checkout `a29c69b` already has it;
its lines are the reference. On the husky, either cherry-pick it or hand-apply:

```cpp
// export_command_interfaces(): read the multiplier's VALUE (the fork used count())
gripper_speed_ = kGripperMaxSpeed * (info_.hardware_parameters.count("gripper_speed_multiplier") ?
                     std::stod(info_.hardware_parameters.at("gripper_speed_multiplier")) : 1.0);
gripper_force_ = kGripperMaxforce * (info_.hardware_parameters.count("gripper_force_multiplier") ?
                     std::stod(info_.hardware_parameters.at("gripper_force_multiplier")) : 1.0);
// write(): scale against the CONSTANT maxima, without mutating the interface storage
const auto speed_fraction = std::clamp(fabs(gripper_speed_) / kGripperMaxSpeed, 0.0, 1.0);
write_speed_.store(uint8_t(speed_fraction * 0xFF));
const auto force_fraction = std::clamp(fabs(gripper_force_) / kGripperMaxforce, 0.0, 1.0);
write_force_.store(uint8_t(force_fraction * 0xFF));
```

(Upstream #83 keeps the maxima in two members fed by optional `gripper_max_speed` /
`gripper_max_force` parameters; dividing by the constants is the same thing without them.
Do NOT divide by a copy of `gripper_speed_` itself — that clamps to 1 and loses the
multiplier all over again.)

`scripts/husky/patch_robotiq_driver.py` applies exactly this (plus the xacro's force
multiplier → 0.25) with backups, and refuses to write unless every anchor matches once. It
was rehearsed on a copy of the husky's files and the result syntax-checked against the
humble headers, then **applied to Cindy and rebuilt on 2026-09-09** (backups
`*.bak-20260909` next to both files; `git checkout` in the fork restores the old code).
For another husky, run it from the laptop:

```bash
scp scripts/husky/patch_robotiq_driver.py administrator@192.168.131.1:/tmp/
ssh administrator@192.168.131.1 'python3 /tmp/patch_robotiq_driver.py'
ssh administrator@192.168.131.1 'source /opt/ros/humble/setup.bash && cd ~/workspace && colcon build --packages-select robotiq_driver 2>&1 | tail -3'
```

then relaunch the gripper stacks (tmux recipe above). **Done and verified on Cindy
2026-09-09:** a full close on nothing takes 1.8 s end to end (≈0.5 s of finger travel plus
the controller's 1 s stall wait) and an open 1.1 s, against ~4 s of travel alone before.
Both sides identical. **Grip force checked by hand the same day:** a leg on its 22 mm face
(gripper stalled at 0.597 rad, jaw ~21.5 mm) held a pull of **~50–60 N** measured on the
left arm's wrist FT (peak 62 N; first give at ~49 N), with a rough ~30–40 N sustained while
creeping — an effective pad-on-wood μ ≈ 0.35–0.4 at the ~74 N clamp. That puts slip (~55 N)
above the insertion wrench guard (40 N), which is above the push (10–30 N): the gripper holds
through anything the controller may do, and the guard trips before the leg moves in the
jaws. Raw recording: `robotiq_grasp_calibration/20260909-pull-test-left/` in the experiment
data folder. Raise `gripper_force_multiplier` to ~0.35 only if the carry needs more margin.

! Measured while doing that: an EMPTY close stops at **0.7894 rad** and comes back
! `stalled: true / ABORTED`, not `reached_goal` — the 0.8 rad target overshoots the fingers'
! mechanical limit by more than the controller's 0.01 rad goal tolerance. So `stalled` alone
! does NOT mean "grasped"; the engine judges a close by the final position instead
! (`GRIPPER_EMPTY_ANGLE` = 0.77 rad in `open_loop_engine.py`: past it, the fingers met air).
! A 22 mm leg stops near 0.59 rad, the 35 mm seat plate near 0.47. After that the
xacro's `gripper_force_multiplier` finally means what it says: 0.5 → ~117 N; set it to
~0.3 (~70 N) for the wooden parts and tune with a pull test. Then on the husky
`colcon build --packages-select robotiq_driver` and restart the gripper stacks.

Applies to all huskies using this fork, not just Cindy.

## Do you ever need to reverse it?

For normal use, **no** — unplugging the cable removes the route automatically,
and the profile carries no gateway/DNS so it cannot affect other networking
while idle. If you want it gone anyway:

```bash
nmcli con down cindy      # deactivate now (returns on replug)
nmcli con delete cindy    # remove permanently
```
