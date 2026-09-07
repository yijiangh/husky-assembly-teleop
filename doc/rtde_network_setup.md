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

Arms must be powered on (tool 24 V comes through the arm; RUNNING or IDLE). Stop with
`tmux kill-session -t gripper_left` (and `_right`) — and always stop these before
launching the full `crl_dual_ur5e.launch.py` stack, which starts its own tool bridges
(two bridges to one arm fight over TCP 54321).

Manual test of one gripper (close 0.8, open 0.426 — the repo's standard values):

```bash
ros2 action send_goal /a200_0806/left_gripper/robotiq_gripper_controller/gripper_cmd \
    control_msgs/action/GripperCommand "{command: {position: 0.8, max_effort: 0.1}}"
```

A `stalled: true / ABORTED` result just means the fingers met resistance before the exact
target — for a grasp that is the success case; `send_gripper_cmd` ignores results anyway.

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

The same pattern on the force register happens to land on 255 (full force) — harmless.
Applies to all huskies using this fork, not just Cindy.

## Do you ever need to reverse it?

For normal use, **no** — unplugging the cable removes the route automatically,
and the profile carries no gateway/DNS so it cannot affect other networking
while idle. If you want it gone anyway:

```bash
nmcli con down cindy      # deactivate now (returns on replug)
nmcli con delete cindy    # remove permanently
```
