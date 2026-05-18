# 2026-05-18 — Live mocap base XYZ offset textbox

## Goal
Type a small XYZ correction (m, world frame) in a textbox and click Apply.
Every downstream consumer (visualization, goal_base_pose sync, planning,
IK, husky_robot.py) immediately sees the corrected base pose. Default
(0,0,0). Reset button zeroes it out.

## User decisions
- World-frame addition: `pos_used = mocap_pos + (x,y,z)`. Orientation untouched.
- Per-husky storage (one robot active per session; UI edits selected robot).
- Keep PyBullet as the primary UI. **Only** the offset textboxes use DPG,
  in a standalone side window.
- The old `robot id` slider in PyBullet is dead code (single-robot session
  guarantee) → removed.

## Why a separate DPG window
`PyBulletBackend.add_text_input` raises `NotImplementedError`
(`ui_backend.py:163-165`) — pybullet's `addUserDebugParameter` is
fundamentally a slider, no text widget. The existing `TextInput` class
(`common.py:517`) routes through `_common._global_backend`, so it works
only when the primary backend is DPG.

Solution: spawn a standalone DPG context owned by `HuskyMonitor`,
independent of `_common._global_backend`. PyBullet + DPG run side-by-side;
DPG is pumped once per `update()` frame. Guard against
`USE_DPG_UI=1` (double `create_context()` would crash) and missing
`dearpygui` install — both fall back cleanly.

## Implementation

### Files
1. **`husky_assembly_teleop/common.py`** — new field on `Husky`:
   ```python
   self.mocap_base_offset_xyz = np.zeros(3)
   ```
   placed just after `self.base_mocap_from_base_footprint` initialization.

2. **`husky_assembly_teleop/husky_monitor.py`** — four changes:

   **(a) Apply offset in mocap callback** (in `receive_mocap_frame`):
   ```python
   calibrated_pose = pp.multiply(world_from_mocap, h.base_mocap_from_base_footprint)
   pos_with_offset = np.array(calibrated_pose[0]) + h.mocap_base_offset_xyz
   h.interface.mocap_callback(pos_with_offset, np.array(calibrated_pose[1]), ts)
   ```
   Single application point → every consumer benefits.

   **(b) Remove dead `selected_robot_slider`**: deleted
   - the `self.selected_robot_slider = None` placeholder in `__init__`
   - the `update_selected_robot_id` method (only the deleted slider called it)
   - the `Slider("robot id", ...)` creation in `build_ui`
   - the `.update()` poll in `update()`
   `self.selected_robot_id = 0` plain attribute remains (read by many sites).

   **(c) New methods on `HuskyMonitor`** (just above `build_ui`):
   - `_init_mocap_offset_window()` — creates standalone DPG context +
     viewport "Husky Base Mocap Offset", 3× `input_float` (x/y/z) +
     Apply + Reset to Zero buttons. Guards: skip if primary backend is
     `DearPyGuiBackend` (avoid double `create_context()`); skip if
     `dearpygui` not installed.
   - `_set_pending_offset(i, v)` — store typed float, ignore garbage.
   - `_apply_base_offset()` — copy pending → `husky.mocap_base_offset_xyz`.
   - `_reset_base_offset()` — zero the offset and the displayed textbox values.
   - `_pump_mocap_offset_window()` — call once per `update()` to render DPG.
   - `_shutdown_mocap_offset_window()` — destroy DPG context on shutdown.
   All defensively use `getattr(self, '_offset_dpg', None)` so they're
   no-ops when init never ran.

   **(d) Wiring**:
   - `build_ui` end: `if self.USE_MOCAP: self._init_mocap_offset_window()`
   - `update()` top (after `_global_backend.step()`): `self._pump_mocap_offset_window()`
   - `destroy_node` (after backend shutdown): `self._shutdown_mocap_offset_window()`

## Race-condition note
Mocap callback runs on the NatNet thread; UI button on the main thread.
`h.mocap_base_offset_xyz = np.array([...])` is a pointer rebind, atomic
in CPython. Callback reads the attribute once per frame. Safe without a
lock.

## Verification

```
cd /home/yijiangh/Code/ros2_ws
source venv/bin/activate
python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop
source install/setup.bash
ros2 run husky_assembly_teleop husky_monitor    # or the actual launch
```

Manual checks:
1. PyBullet debug GUI opens as before (all existing widgets functional).
2. Separate window "Husky Base Mocap Offset" with x/y/z input boxes,
   Apply and Reset to Zero buttons.
3. `robot id` slider is gone from PyBullet GUI.
4. Mocap streaming → visualized base at sensed pose.
5. Type `0.1` in x → Apply → visualized base jumps +10 cm in world X
   on next redraw; console: `[mocap offset] applied: [0.1, 0.0, 0.0]`.
6. Click any planning button (e.g. `Plan S.Arm to conf target`) —
   planner sees `interface.position` already including the offset.
7. Reset to Zero → base snaps back; textbox values reset to 0.0000.
8. Non-numeric input dropped silently; previous pending retained.
9. Closing PyBullet GUI → monitor shuts down cleanly; DPG window closes too.
10. Negative: uninstall `dearpygui` → monitor still boots; warning
    `dearpygui not installed; offset textboxes disabled`; everything else works.

## Build status
`python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop`
→ clean, 0 warnings. Import-smoke (`import husky_assembly_teleop.husky_monitor`)
green. Live test with mocap pending user.
