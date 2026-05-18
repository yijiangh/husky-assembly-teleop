# 2026-05-16 — Cherry-pick scaffolding driver buttons (commit 4f41ad6) + EE-gated UI

## Goal
Pull only the scaffolding-tool-control bits from `4f41ad6 scaffolding driver buttons`
into master, and make the new Scaffolding V3 button block appear only when the
active robot's `ee_types` contains `assembly_tool_v3_left` / `assembly_tool_v3_right`
(as configured in `husky_world.py:ROBOT_CONFIGS`).

Also: expose the gripper helpers (`open_gripper_full`, `close_gripper_for_bar`,
`set_gripper`) only when `connect_gripper=True` for the active robot. (Already
done in the previous turn; same gating philosophy.)

## In / Out of scope from 4f41ad6

In:
- `husky_robot.py`:
  - import `ScaffoldingToolCmd, ScaffoldingToolStatus` from `crl_husky_msgs.msg`
  - per-arm `scaffolding_status` slot (class default `[None]` + instance init + dual-arm append)
  - subscriptions: `<ns>/{left_,right_,}gripper/tool_status` → `scaffolding_status_callback`
  - publishers:   `<ns>/{left_,right_,}gripper/tool_cmd`    ← `send_scaffolding_cmd`
  - new methods `scaffolding_status_callback(index, msg)` and `send_scaffolding_cmd(direction, motor, index)`
- `husky_monitor.py`:
  - `----------Scaffolding V3` separator + buttons (Stop M1+M2, Tighten/Loosen M1, Tighten/Loosen M2) for L and (dual_arm only) R

Out:
- `VALIDATION_PROBLEM_NAME = '250902_kissing_experiment'` — unrelated, dropped.
- Original outer wrapping of the buttons inside `if dual_arm:` — removed.
  The L-side block now appears whenever `assembly_tool_v3_left` is in ee_types,
  even on a single-arm robot. R-side still requires `dual_arm`.

## Gating logic

`common.py:Husky` already gained `self.connect_gripper` (prior turn). Add:

    self.ee_types = list(ee_types or [])

`husky_monitor.py:build_ui` reads it:

    active_husky = self.huskies[self.selected_robot_id]
    has_scaffold_left  = any('assembly_tool_v3_left'  in (t or '') for t in active_husky.ee_types)
    has_scaffold_right = any('assembly_tool_v3_right' in (t or '') for t in active_husky.ee_types)
    if has_scaffold_left or has_scaffold_right:
        ... separator + buttons ...
        if has_scaffold_right and active_husky.dual_arm: ... R buttons ...

Effect by ROS_DOMAIN_ID against `husky_world.py:ROBOT_CONFIGS`:
- 84 (`robotiq_gripper`)               → no Scaffolding section
- 85 (`robotiq_gripper`)               → no Scaffolding section
- 86 (`assembly_tool_v3_left`, `..._right`) → full L + R Scaffolding section

Reset path: `update_selected_robot_id` calls `reset_ui` → `build_ui`, so the
section re-evaluates if the active robot ever changes.

## Verification

- `python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop` → clean
- `python3 -c "from husky_assembly_teleop import husky_robot, husky_monitor, common"` → clean
- `python3 -c "from crl_husky_msgs.msg import ScaffoldingToolCmd, ScaffoldingToolStatus"` → resolves (msgs exist in workspace)

Not exercised on hardware in this session.
