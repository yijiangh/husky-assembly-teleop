# Dual-husky parallel launch (0804 + 0806) — 2026-05-15

## Problem

Want to control 0804 (single-arm) and 0806 (dual-arm) at the same time on one
workstation, each in its own terminal with `ROS_DOMAIN_ID=84` / `ROS_DOMAIN_ID=86`.
Currently per-robot config (namespace, mocap_id, ee_types, connect_gripper) is
hardcoded in `husky_assembly_teleop/husky_world.py:174-220`, so switching robots
needs hand edits each launch.

## Mocap (NatNetClient) parallel-streaming check

**Verdict: two `husky_monitor` instances on the same `CLIENT_IP` can stream
mocap concurrently.**

- `start_mocap` (`husky_monitor.py:1818-1843`) sets unicast
  (`set_use_multicast(False)`).
- Unicast cmd socket: `bind((local_ip, 0))` → ephemeral port
  (`NatNetClient.py:255`).
- Unicast data socket: `bind(('', 0))` with `SO_REUSEADDR`
  (`NatNetClient.py:307-310`).
- `data_port=1511` / `command_port=1510` are only *destination* ports on the
  Motive server — never bound locally in unicast.
- Each `NatNetClient` instance gets unique ephemeral source ports; Motive sends
  unicast back to each source addr:port. Motive supports multiple unicast clients.

Runtime checks: Motive Streaming pane Unicast on, Local Interface =
`CLIENT_IP='192.168.0.21'`. If frames drop on one client, flip both to multicast
in `husky_monitor.py:1823` (`set_use_multicast(True)`).

## Per-robot diff captured in config

| field             | 0804                 | 0806                                                  |
| ----------------- | -------------------- | ----------------------------------------------------- |
| robot_namespace   | `/a200_0804`         | `/a200_0806`                                          |
| mocap_id          | 4568                 | 4617                                                  |
| connect_gripper   | True                 | False                                                 |
| ee_types_default  | `['robotiq_gripper']` | `['assembly_tool_v3_left','assembly_tool_v3_right']` |

Everything else (dual_arm, base_calibration_file path, pos, PUNCH_CALIB_VALIDATION
overrides) keeps deriving from `robot_name` / monitor mode flags.

## Implementation

Only `husky_world.init(monitor)` changes. A `ROBOT_CONFIGS` dict keyed by
`ROS_DOMAIN_ID` is read at the top of `init`, with a default-to-`'86'` fallback
and a warning if an unknown domain id is set.

## Usage

```
# Terminal A — 0804
cd /home/yijiangh/Code/ros2_ws && source venv/bin/activate && source install/setup.bash
ROS_DOMAIN_ID=84 python3 -m husky_assembly_teleop.husky_monitor

# Terminal B — 0806
cd /home/yijiangh/Code/ros2_ws && source venv/bin/activate && source install/setup.bash
ROS_DOMAIN_ID=86 python3 -m husky_assembly_teleop.husky_monitor
```

## Verification

1. Two PyBullet GUIs, single-arm vs dual-arm.
2. Both log `mocap client connected: True`; bases update from 4568 / 4617.
3. `ros2 topic list` per terminal shows disjoint topics; cross-domain commands
   don't move the other robot.
4. Colcon build clean (`python3 -m colcon build --symlink-install
   --packages-select husky_assembly_teleop`).
