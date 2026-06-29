# Pre-Calibration Checklist (DRAFT — temporary)

> **Status**: temporary working doc. Holds hardcoded knobs / settings **not yet placed**
> in [calibration_manual.md](calibration_manual.md). When a category is merged into the
> official manual it is removed from here (and logged below), so this file always shows
> what's still outstanding.

Each row: **What** / **Where** (clickable file:line) / **Note**.
`(was in §4.0)` marks items already in the manual's §4.0 checklist.

## Already merged into the manual (removed from this draft)
- **Robot Reference Table** (Name/Serial/IP/ROS domain/Motive ID) → manual **§0**.
- **MoCap Streaming-ID explanation** (deleted from §2.4) → manual §0 + Category 1 below.
- **`config.yaml` field reference** (old Category 2) → manual **§7.1.3**.
- **Stale §4.0 line numbers** — resolved: source comments were trimmed, so §4.0's numbers (L57/L21/L182/L327/L91) now match the code.

---

## Category 1 — Per-session knobs (verify/change most sessions)

| What | Where | Note |
|------|-------|------|
| `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` | terminal env (see [§2.4](calibration_manual.md#24-verify-ros2-connection)) | **Master switch.** 84=Alice/0804, 85=Belle/0805, 86=Cindy/0806. Picks namespace, base `mocap_id`, gripper, default EE via `ROBOT_CONFIGS` ([husky_world.py:178-212](../husky_assembly_teleop/husky_world.py#L178)). `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. |
| `CLIENT_IP`, `MOCAP_IP` | [husky_monitor.py:60-61](../husky_assembly_teleop/husky_monitor.py#L60) (see [§1.4](calibration_manual.md#14-network-connection-mocap-to-workstation)) | workstation IP / Motive PC IP. |
| `USE_MOCAP`, `FAKE_HARDWARE` | [husky_monitor.py:78-79](../husky_assembly_teleop/husky_monitor.py#L78) | real session: `1` / `0`. Desk-testing: `0` / `1`. Independent flags, keep consistent by hand. |
| `CALIBRATION` | [husky_monitor.py:91](../husky_assembly_teleop/husky_monitor.py#L91) | `1` = calibration mode. *(was in §4.0)* |
| `PUNCH_CALIB_VALIDATION` | [husky_monitor.py:102](../husky_assembly_teleop/husky_monitor.py#L102) | `1` only for a punch-validation session (switches EE to punch tips); `0` otherwise. |
| `USE_CELL_STATE_BASE_POSE` | [husky_monitor.py:87](../husky_assembly_teleop/husky_monitor.py#L87) | `0` = base tracks mocap (normal); `1` = pin base to loaded cell state. |
| `CALIBRATION_DATE` + `DEFAULT_DATE_FOLDER` | [__init__.py:57](../husky_assembly_teleop/__init__.py#L57) & [config_loader.py:21](../data/calibration_data/config_loader.py#L21) | **Both must match today's date folder**; folder must exist with a `config.yaml` inside. *(was in §4.0)* |
| base `mocap_id` (Streaming IDs) | [husky_world.py:182/189/196](../husky_assembly_teleop/husky_world.py#L182) | per ROS domain (Alice 1031 / Belle 1021 / Cindy 1011). Match Motive **properties > Streaming ID**. *(was in §4.0)* |
| calib tool IDs `1013` L / `1012` R | [husky_world.py:327/331](../husky_assembly_teleop/husky_world.py#L327) | Match Motive. **Which arm gets calibrated = which `calib_tool_*` block is enabled** — comment out the unused arm's block for a single-arm run (calib-tool block at [L326](../husky_assembly_teleop/husky_world.py#L326)). *(was in §4.0)* |
| `CALIBRATION_STATE_SETS` (per arm index) | [husky_monitor.py:72-75](../husky_assembly_teleop/husky_monitor.py#L72) | `0`=left/single → `260108_extrinsic_calib_trajs`, `1`=Cindy right → `260225_extrinsic_calib_trajs_Cindy_Right`. Verify the traj-state folder for your arm exists. |

---

## Category 2 — One-time per new machine (absolute `/home/su/...` paths)

Hardcoded to this machine's home dir — must update when moving to another PC.

| What | Where | Note |
|------|-------|------|
| `DESIGN_DATA_DIRECTORY`, `EXPERIMENT_DATA_DIRECTORY` | [__init__.py:53-54](../husky_assembly_teleop/__init__.py#L53) | gdrive (Insync) mount paths; leading `/home/su` is machine-specific. |
| `MOCAP_CAMERA_EXPORT_DIR` | [husky_monitor.py:64-67](../husky_assembly_teleop/husky_monitor.py#L64) | where 'collect cameras data' drops JSON+CSV. |

---

## Excluded (internal tuning — NOT user checklist items)

RRT resolutions, kissing-experiment timing constants, UI colors/fonts, default
punch-offset fallback, joint/link name maps — derived or tuned, not per-session knobs.
